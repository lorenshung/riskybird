#!/usr/bin/env python3
"""Load a Zephyr ELF into the drone SoC through the ESP32 instead of JTAG.

Talks to the esp_bridge load service (workloads/esp_bridge/src/load_link.c),
which drives the FPGA's EspLinkMux into UART-TSI mode: the SoC is reset, hart 0
waits in the boot ROM, this script writes every loadable segment into DDR over
TSI, reads all of it back, then sets the boot address and raises MSIP.

Two transports, one protocol:
  --url socket://192.168.4.1:14552    over the drone's WiFi
  --usb /dev/ttyACM0                  over the ESP's USB console (bench test)

Other jobs on the same service:
  --ping           which bitstream answers (its USR_ACCESS build timestamp)
  --stat           the ESP's RBLINK line (what the motor frames say)
  --stash --elf    keep this program on the ESP (with --policy autoload=1 the
                   ESP reloads it by itself after a power cut)
  --policy "..."   boot=golden|update autoload=0|1, or "" to just report
  --auto           run the ESP's power-on sequence now (test it without
                   cutting power)
  --commit --elf   the remote `commit.gdb`: write ROSE_COMMIT_KEY into the
                   RUNNING flight controller's g_flight_commit_key with no
                   reset (a POKE session), after checking that the running
                   image is this ELF, then confirm COMMITTED from the motor
                   frames the ESP receives.

Exit status is 0 only if every byte read back matched. Runs in the zephyr env
(needs pyserial and pyelftools); `rb load --port socket://...` invokes it.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

import serial
from elftools.elf.elffile import ELFFile

CMD_READ = 0
CMD_WRITE = 1
CMD_END = 0xE5D0E5D0          # ESP-local: end the session, never forwarded
BOOT_ADDR_REG = 0x1000        # boot ROM jumps here after MSIP
CLINT_MSIP0 = 0x2000000
CHUNK = 1024
BARRIER = 16 * 1024           # read back after this many written bytes
LINK_BPS = 115200 / 10        # the FPGA end of every path: 8N1 at 115200


class LoadError(RuntimeError):
    pass


class Tsi:
    def __init__(self, ser: serial.SerialBase, timeout: float) -> None:
        self.ser = ser
        self.timeout = timeout
        # Bytes handed to the transport since the last read completed. Writes
        # get no reply, so over WiFi tens of KB can sit in socket buffers ahead
        # of a read request; its deadline has to include draining them.
        self.backlog = 0

    def _header(self, cmd: int, addr: int, nbytes: int) -> bytes:
        words = (nbytes + 3) // 4
        return struct.pack("<IQQ", cmd, addr, max(words - 1, 0))

    def write(self, addr: int, data: bytes) -> None:
        pad = (-len(data)) % 4
        msg = self._header(CMD_WRITE, addr, len(data)) + data + b"\xff" * pad
        self.ser.write(msg)
        self.backlog += len(msg)

    def read(self, addr: int, nbytes: int) -> bytes:
        want = (nbytes + 3) // 4 * 4
        self.ser.write(self._header(CMD_READ, addr, nbytes))
        got = bytearray()
        wire = (self.backlog + 20 + want) / LINK_BPS
        deadline = time.monotonic() + self.timeout + wire * 1.5
        while len(got) < want:
            if time.monotonic() > deadline:
                raise LoadError(
                    f"read of {nbytes} B at {addr:#x} timed out with {len(got)} B")
            got += self.ser.read(want - len(got))
        self.backlog = 0
        return bytes(got[:nbytes])

    def end(self) -> None:
        self.ser.write(struct.pack("<IQQ", CMD_END, 0, 0))
        self.ser.flush()


def open_link(args: argparse.Namespace) -> serial.SerialBase:
    if not args.url:
        return serial.Serial(args.usb, 115200, timeout=0.5)
    # The ESP serves one connection at a time and refuses a new one for a
    # moment after the last closed (its TCP close/TIME_WAIT), so retry briefly.
    deadline = time.monotonic() + 5.0
    while True:
        try:
            return serial.serial_for_url(args.url, timeout=0.5)
        except serial.SerialException as exc:
            if "refused" not in str(exc) or time.monotonic() > deadline:
                raise
            time.sleep(0.25)


def handshake(ser: serial.SerialBase, timeout: float, hello: str = "RBLOAD1",
              name: str = "RBLOAD") -> str:
    """Send the hello and wait for the ESP's verdict, skipping console text.
    Returns what follows "<name> OK" (e.g. "id=6a1b2c3d")."""
    ser.reset_input_buffer()
    ser.write(hello.encode() + b"\n")
    ser.flush()
    deadline = time.monotonic() + timeout
    # The USB tunnel discards input that arrives while the ESP is still
    # finishing a previous session, so resend once if nothing came back.
    resend = time.monotonic() + 1.5 if isinstance(ser, serial.Serial) else None
    line = bytearray()
    while time.monotonic() < deadline:
        c = ser.read(1)
        if not c:
            if resend is not None and time.monotonic() > resend:
                resend = None
                ser.write(hello.encode() + b"\n")
                ser.flush()
            continue
        if resend is not None and time.monotonic() > resend:
            resend = None      # console text keeps arriving on USB; time it anyway
            ser.write(hello.encode() + b"\n")
            ser.flush()
        if c != b"\n":
            line += c
            continue
        text = line.decode("ascii", "replace").strip()
        line.clear()
        if text.startswith(name):
            resend = None
        if text == f"{name} OK" or text.startswith(f"{name} OK "):
            return text[len(name) + 3:].strip()
        if text.startswith(f"{name} ERR") or text.startswith("RB ERR"):
            raise LoadError(f"ESP refused: {text}")
        # Anything else is console output on the USB tunnel; ignore it.
    raise LoadError(
        f"no answer to {hello} -- is the ESP running an esp_bridge image with "
        "program loading (load_link)?")


ROSE_COMMIT_KEY = 0x464C5921
COMMIT_SYMBOL = "g_flight_commit_key"
MOTOR_LINK_FLAG_COMMIT = 0x20


def elf_symbol(elf_path: str, name: str) -> int | None:
    with open(elf_path, "rb") as f:
        elf = ELFFile(f)
        tab = elf.get_section_by_name(".symtab")
        if tab is None:
            return None
        syms = tab.get_symbol_by_name(name)
        return syms[0]["st_value"] if syms else None


def text_samples(elf_path: str, nbytes: int = 1024) -> list[tuple[int, bytes]]:
    """The first nbytes of every executable PT_LOAD: what the running image
    must hold if it is this ELF (code is never written at run time)."""
    out = []
    with open(elf_path, "rb") as f:
        elf = ELFFile(f)
        for seg in elf.iter_segments():
            if seg["p_type"] == "PT_LOAD" and seg["p_flags"] & 1 and seg["p_filesz"]:
                out.append((seg["p_paddr"], seg.data()[:nbytes]))
    return out


def stat_line(args: argparse.Namespace) -> str:
    ser = open_link(args)
    try:
        return handshake(ser, args.timeout, "RBSTAT1", "RBSTAT")
    finally:
        ser.close()


def link_flags(line: str) -> int | None:
    for tok in line.split():
        if tok.startswith("fl=0x"):
            return int(tok[5:7], 16)
    return None


def segments(elf_path: str) -> list[tuple[int, bytes]]:
    out = []
    with open(elf_path, "rb") as f:
        elf = ELFFile(f)
        for seg in elf.iter_segments():
            if seg["p_type"] != "PT_LOAD" or seg["p_filesz"] == 0:
                continue
            if seg["p_paddr"] % 4:
                # TSI addresses whole 32-bit words and ignores addr[1:0]: an
                # unaligned segment would land shifted and still read back
                # "correctly" through the same shift.
                raise LoadError(f"{elf_path}: segment at {seg['p_paddr']:#x} "
                                "is not 4-byte aligned")
            out.append((seg["p_paddr"], seg.data()))
    if not out:
        raise LoadError(f"{elf_path}: no loadable segments")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    where = ap.add_mutually_exclusive_group(required=True)
    where.add_argument("--url", help="pyserial URL, e.g. socket://192.168.4.1:14552")
    where.add_argument("--usb", help="ESP USB console device, e.g. /dev/ttyACM0")
    ap.add_argument("--elf", help="Zephyr ELF to load")
    ap.add_argument("--probe", action="store_true",
                    help="handshake and read the boot ROM's first words only")
    ap.add_argument("--no-start", action="store_true",
                    help="load and verify, but leave hart 0 waiting in the boot ROM")
    ap.add_argument("--ping", action="store_true",
                    help="report which bitstream answers; touches nothing else")
    ap.add_argument("--stat", action="store_true",
                    help="print the ESP's RBLINK line; never touches the FPGA link")
    ap.add_argument("--stash", action="store_true",
                    help="keep --elf on the ESP for a reload after a power cut "
                         "(after loading it, unless --stash-only)")
    ap.add_argument("--stash-only", action="store_true",
                    help="with --stash: do not load it now")
    ap.add_argument("--policy", metavar="SETTINGS",
                    help='e.g. "boot=update autoload=1"; "" just reports')
    ap.add_argument("--auto", action="store_true",
                    help="run the ESP's power-on sequence now")
    ap.add_argument("--commit", action="store_true",
                    help="commit the RUNNING flight controller (--elf) to a flight test")
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args()
    if args.stat:
        try:
            print(stat_line(args))
            return 0
        except (LoadError, serial.SerialException) as exc:
            print(f"esp_tsi_load: {exc}", file=sys.stderr)
            return 1
    if args.ping:
        try:
            ser = open_link(args)
            try:
                print(f"ESP: RBPING OK {handshake(ser, args.timeout, 'RBPING1', 'RBPING')}")
            finally:
                ser.close()
            return 0
        except (LoadError, serial.SerialException) as exc:
            print(f"esp_tsi_load: {exc}", file=sys.stderr)
            return 1
    try:
        if args.policy is not None:
            hello = f"RBPOLICY1 {args.policy}".rstrip()
            print(esp_line_session(args, hello, "RBPOLICY")[0])
            return 0
        if args.auto:
            esp_line_session(args, "RBAUTO1", "RBAUTO", until="RBAUTO DONE", timeout=120)
            return 0
    except (LoadError, serial.SerialException) as exc:
        print(f"esp_tsi_load: {exc}", file=sys.stderr)
        return 1
    if args.stash and args.stash_only:
        if not args.elf:
            ap.error("--stash needs --elf")
        return stash(args)
    if args.commit:
        if not args.elf:
            ap.error("--commit needs --elf: the flight controller that is running")
        return commit(args)
    if not args.probe and not args.elf:
        ap.error("--elf is required unless --probe")

    plan = [] if args.probe else segments(args.elf)
    total = sum(len(d) for _, d in plan)
    try:
        ser = open_link(args)
    except serial.SerialException as exc:
        print(f"esp_tsi_load: cannot reach the ESP load service: {exc}", file=sys.stderr)
        return 1
    tsi = Tsi(ser, args.timeout)
    t0 = time.monotonic()
    in_session = False
    rc = load(args, ser, tsi, plan, total, t0)
    if rc == 0 and args.stash:
        time.sleep(0.6)
        rc = stash(args)
    return rc


def load(args, ser, tsi, plan, total, t0) -> int:
    in_session = False
    try:
        ident = handshake(ser, args.timeout)
        in_session = True
        print(f"ESP: RBLOAD OK {ident} -- SoC reset, hart 0 parked in the boot ROM")

        rom = tsi.read(0x10000, 16)
        print(f"boot ROM @0x10000: {rom.hex()}")
        if rom == b"\x00" * 16 or rom == b"\xff" * 16:
            raise LoadError("boot ROM read back blank -- TSI is not reaching the bus")
        if args.probe:
            return 0

        done = 0
        for addr, data in plan:
            print(f"writing {len(data)} B at {addr:#x}")
            for off in range(0, len(data), CHUNK):
                piece = data[off:off + CHUNK]
                tsi.write(addr + off, piece)
                done += len(piece)
                if tsi.backlog >= BARRIER:
                    # Barrier: wait for the link to catch up, so progress is
                    # real and a dead link shows up now, not at verify time.
                    tail = (addr + off + len(piece) - 4) & ~3
                    tsi.read(tail, 4)
                print(f"\r  {done}/{total} B", end="", flush=True)
            print()

        bad = 0
        for addr, data in plan:
            print(f"verifying {len(data)} B at {addr:#x}")
            for off in range(0, len(data), CHUNK):
                want = data[off:off + CHUNK]
                got = tsi.read(addr + off, len(want))
                if got != want:
                    first = next(i for i in range(len(want)) if got[i] != want[i])
                    print(f"  MISMATCH at {addr + off + first:#x}: "
                          f"wrote {want[first]:#04x}, read {got[first]:#04x}")
                    bad += 1
        if bad:
            raise LoadError(f"{bad} chunk(s) failed verification; hart 0 NOT started")
        print(f"verified all {total} B")

        if not args.no_start:
            with open(args.elf, "rb") as f:
                entry = ELFFile(f)["e_entry"]
            tsi.write(BOOT_ADDR_REG, struct.pack("<Q", entry))
            tsi.write(CLINT_MSIP0, struct.pack("<I", 1))
            print(f"started hart 0 at {entry:#x}")
        return 0
    except LoadError as exc:
        print(f"esp_tsi_load: {exc}", file=sys.stderr)
        return 1
    finally:
        if in_session:
            try:
                tsi.end()
            except Exception:
                pass
        ser.close()
        if in_session:
            print(f"({time.monotonic() - t0:.1f} s; the ESP hands the FPGA link "
                  f"back to uart1 about 0.3 s after this)")


def stash_blob(elf_path: str) -> bytes:
    """The flat image persist.c keeps: a 512-byte header (magic, version,
    segment table, entry, data length, CRC32) and the segment data."""
    import zlib
    segs = segments(elf_path)
    if len(segs) > 16:
        raise LoadError(f"{elf_path}: {len(segs)} segments; the stash holds 16")
    with open(elf_path, "rb") as f:
        entry = ELFFile(f)["e_entry"]
    data = bytearray()
    table = b""
    for paddr, d in segs:
        table += struct.pack("<QII", paddr, len(data), len(d))
        data += d
        data += b"\xff" * ((-len(data)) % 4)
    hdr = b"RBSTASH1" + struct.pack("<IIQII", 1, len(segs), entry, len(data),
                                    zlib.crc32(bytes(data)) & 0xFFFFFFFF) + table
    return hdr.ljust(512, b"\xff") + bytes(data)


def esp_line_session(args: argparse.Namespace, hello: str, name: str,
                     until: str | None = None, timeout: float | None = None) -> list[str]:
    """A session that only exchanges text lines with the ESP."""
    ser = open_link(args)
    try:
        first = handshake(ser, args.timeout, hello, name)
        lines = [f"{name} OK {first}".strip()]
        if until:
            deadline = time.monotonic() + (timeout or 60)
            buf = bytearray()
            while time.monotonic() < deadline:
                c = ser.read(1)
                if not c:
                    continue
                if c != b"\n":
                    buf += c
                    continue
                text = buf.decode("ascii", "replace").strip()
                buf.clear()
                lines.append(text)
                print(f"  {text}")
                if text.startswith(until):
                    break
        return lines
    finally:
        ser.close()


def wait_line(ser: serial.SerialBase, prefixes: tuple[str, ...], timeout: float) -> str:
    deadline = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < deadline:
        c = ser.read(1)
        if not c:
            continue
        if c != b"\n":
            buf += c
            continue
        text = buf.decode("ascii", "replace").strip()
        buf.clear()
        if text.startswith(prefixes):
            return text
    raise LoadError(f"no {'/'.join(prefixes)} from the ESP within {timeout:.0f} s")


def stash(args: argparse.Namespace) -> int:
    """Keep args.elf on the ESP for a reload after a power cut."""
    blob = stash_blob(args.elf)
    ser = open_link(args)
    try:
        ser.reset_input_buffer()
        ser.write(f"RBSTASH1 {len(blob):x}\n".encode())
        text = wait_line(ser, ("RBSTASH READY", "RBSTASH ERR", "RB ERR"), 15)
        if not text.startswith("RBSTASH READY"):
            raise LoadError(f"ESP refused the stash: {text}")
        for off in range(0, len(blob), 4096):
            ser.write(blob[off:off + 4096])
        text = wait_line(ser, ("RBSTASH OK", "RBSTASH ERR"), 30 + len(blob) / 20000)
        if not text.startswith("RBSTASH OK"):
            raise LoadError(text)
        print(f"ESP: {text}")
        return 0
    except (LoadError, serial.SerialException) as exc:
        print(f"esp_tsi_load: {exc}", file=sys.stderr)
        return 1
    finally:
        ser.close()


def commit(args: argparse.Namespace) -> int:
    """Remote commit.gdb. Both halves of the interlock stay: the symbol exists
    only in a -DROSE_FLIGHT_COMMIT=1 build, and the write is one specific
    constant, made deliberately, at run time."""
    addr = elf_symbol(args.elf, COMMIT_SYMBOL)
    if addr is None:
        print(f"esp_tsi_load: No symbol \"{COMMIT_SYMBOL}\" in {args.elf}. That is the "
              "commit interlock working: this build was made without "
              "-DROSE_FLIGHT_COMMIT=1 and cannot be committed.", file=sys.stderr)
        return 1
    if addr % 4:
        print(f"esp_tsi_load: {COMMIT_SYMBOL} at {addr:#x} is not word-aligned",
              file=sys.stderr)
        return 1
    samples = text_samples(args.elf)
    try:
        before = stat_line(args)
    except (LoadError, serial.SerialException) as exc:
        print(f"esp_tsi_load: {exc}", file=sys.stderr)
        return 1
    print(f"before: {before}")
    try:
        ser = open_link(args)
    except serial.SerialException as exc:
        print(f"esp_tsi_load: cannot reach the ESP load service: {exc}", file=sys.stderr)
        return 1
    tsi = Tsi(ser, args.timeout)
    in_session = False
    try:
        ident = handshake(ser, args.timeout, "RBPOKE1", "RBPOKE")
        in_session = True
        print(f"ESP: RBPOKE OK {ident} -- no reset; the program keeps running")
        for paddr, want in samples:
            got = tsi.read(paddr, len(want))
            if got != want:
                raise LoadError(
                    f"the running image is NOT {args.elf} (code at {paddr:#x} differs); "
                    "refusing to write into it")
        print(f"running image matches {args.elf} ({len(samples)} code segment(s) checked)")
        cur = struct.unpack("<I", tsi.read(addr, 4))[0]
        if cur == ROSE_COMMIT_KEY:
            print(f"{COMMIT_SYMBOL} already holds the key")
        else:
            tsi.write(addr, struct.pack("<I", ROSE_COMMIT_KEY))
            cur = struct.unpack("<I", tsi.read(addr, 4))[0]
            if cur != ROSE_COMMIT_KEY:
                raise LoadError(f"{COMMIT_SYMBOL} reads {cur:#010x} after the write")
            print(f"wrote {ROSE_COMMIT_KEY:#010x} to {COMMIT_SYMBOL} at {addr:#x}")
    except LoadError as exc:
        print(f"esp_tsi_load: {exc}", file=sys.stderr)
        return 1
    finally:
        if in_session:
            try:
                tsi.end()
            except Exception:
                pass
        ser.close()
    # The acceptance test is the flight controller saying so, in the frames
    # it sends the ESP, not our own read-back.
    deadline = time.monotonic() + 5.0
    line = "?"
    time.sleep(0.6)    # the ESP hands the wire back to uart1 ~0.3 s after END
    while time.monotonic() < deadline:
        try:
            line = stat_line(args)
        except (LoadError, serial.SerialException):
            time.sleep(0.3)
            continue
        fl = link_flags(line)
        if fl is not None and fl & MOTOR_LINK_FLAG_COMMIT and " ml=up " in f" {line} ":
            print(f"after:  {line}")
            print("COMMITTED -- confirmed by the flight controller's own motor frames. "
                  "Props are live for the committed budget.")
            return 0
        time.sleep(0.3)
    print(f"after:  {line}")
    print("esp_tsi_load: the key is written but the motor frames do not show COMMITTED "
          "(the window may already have latched, or this is not an esp-motors build)",
          file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
