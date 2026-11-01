#!/usr/bin/env python3
"""Load a Zephyr ELF into the drone SoC through the ESP32 instead of JTAG.

Talks to the esp_bridge load service (workloads/esp_bridge/src/load_link.c),
which drives the FPGA's EspLinkMux into UART-TSI mode: the SoC is reset, hart 0
waits in the boot ROM, this script writes every loadable segment into DDR over
TSI, reads all of it back, then sets the boot address and raises MSIP.

Two transports, one protocol:
  --url socket://192.168.4.1:14552    over the drone's WiFi
  --usb /dev/ttyACM0                  over the ESP's USB console (bench test)

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
    if args.url:
        ser = serial.serial_for_url(args.url, timeout=0.5)
    else:
        ser = serial.Serial(args.usb, 115200, timeout=0.5)
    return ser


def handshake(ser: serial.SerialBase, timeout: float) -> None:
    """Send the hello and wait for the ESP's verdict, skipping console text."""
    ser.reset_input_buffer()
    ser.write(b"RBLOAD1\n")
    ser.flush()
    deadline = time.monotonic() + timeout
    line = bytearray()
    while time.monotonic() < deadline:
        c = ser.read(1)
        if not c:
            continue
        if c != b"\n":
            line += c
            continue
        text = line.decode("ascii", "replace").strip()
        line.clear()
        if text == "RBLOAD OK":
            return
        if text.startswith("RBLOAD ERR"):
            raise LoadError(f"ESP refused: {text}")
        # Anything else is console output on the USB tunnel; ignore it.
    raise LoadError(
        "no answer to RBLOAD1 -- is the ESP running an esp_bridge image with "
        "program loading (load_link)?")


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
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args()
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
    try:
        handshake(ser, args.timeout)
        in_session = True
        print("ESP: RBLOAD OK -- SoC reset, hart 0 parked in the boot ROM")

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


if __name__ == "__main__":
    sys.exit(main())
