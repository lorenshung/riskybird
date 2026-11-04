#!/usr/bin/env python3
"""Write, check and boot FPGA bitstreams in the drone's configuration flash
through the ESP32, over WiFi or the ESP's USB console.

The flash (S25FL256S, 32 MiB) holds two images:

  0x000000  GOLDEN   the image the FPGA loads at power-on. Written only over
                     JTAG (`openFPGALoader --write-flash`); the FPGA's flash
                     bridge refuses to program or erase anything below
                     0x800000, so nothing here can damage it.
  0x800000  UPDATE   written by this script; entered with --boot update.

Addresses here are flash byte addresses; the ESP sends WBSTAR = addr >> 8,
which is what SPI 32-bit addressing (every build) expects.

Both must be ESP-loader bitstreams (RocketArty200T*LoaderConfig), or the drone
can only be reached again by power-cycling it into golden.

  --url socket://192.168.4.1:14552 | --usb /dev/ttyACM0
  --info                    flash ID and status registers
  --write X.bit             erase + program + verify the update slot
  --verify X.bit [--at A]   compare the flash at A (default: update slot): by
                            CRC-32 computed on the FPGA (seconds), or by reading
                            it back (--readback, or a loader without CRC)
  --boot golden|update|HEX  reconfigure the FPGA from that address, then wait
                            for the new image to answer and report its id
  --boot-mode 1|2|3         ICAP diagnostics (EspLinkIprog): 1 IPROG without a
                            WBSTAR write, 2 WBSTAR only (no reboot), 3 packed
  --ping                    which image is running now

Each image identifies itself by its USR_ACCESS word (the bitstream's build
TIMESTAMP); this script reads the same word out of a .bit file, so after
--boot it can say whether the FPGA landed in the image it was sent to or fell
back to golden. Runs in the zephyr env python (needs pyserial).
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
import zlib

import serial

sys.path.insert(0, __import__("os").path.dirname(__file__))
from esp_tsi_load import LoadError, handshake, open_link  # noqa: E402

GOLDEN = 0x000000
UPDATE = 0x800000
GUARD = 0x800000
FLASH_SIZE = 32 << 20
SECTOR = 64 << 10
PAGE = 256
OK = 0xA5
ERRS = {0xE1: "refused by the golden-image guard (or misaligned)",
        0xE2: "XFER opcode refused", 0xE3: "flash stayed busy past the timeout",
        0xEE: "unknown frame (framing lost)", 0xEF: "FPGA input FIFO overflowed"}
WINDOW = 8                      # PROG frames in flight: 8 x 261 B < the 4 KiB FIFO
EXPECT_JEDEC = bytes([0x01, 0x02, 0x19])   # Spansion S25FL256S
CRC_CHUNK = 1 << 20             # ~0.7 s each on the FPGA: well inside the ESP's 5 s stall limit


def bit_payload(path: str) -> bytes:
    """The configuration data of a Xilinx .bit file (field 'e'), which is
    exactly what goes into SPI flash."""
    b = open(path, "rb").read()
    if b[:2] != b"\x00\x09":
        raise LoadError(f"{path}: not a Xilinx .bit file")
    i = 13
    while i < len(b):
        key = b[i:i + 1]
        if key == b"e":
            n = struct.unpack(">I", b[i + 1:i + 5])[0]
            return b[i + 5:i + 5 + n]
        n = struct.unpack(">H", b[i + 1:i + 3])[0]
        i += 3 + n
    raise LoadError(f"{path}: no configuration data")


def spi_32bit(payload: bytes) -> bool:
    """True if the image switches the configuration reads to 4-byte addressing
    (BSPI = 0x0C, BITSTREAM.CONFIG.SPI_32BIT_ADDR YES) in its header. The ESP
    turns a boot address into WBSTAR = addr >> 8, which is only right for such
    images; every build from fpga-shells arty-config.xdc is one."""
    sync = payload.find(bytes.fromhex("aa995566"))
    return sync >= 0 and bytes.fromhex("3003e0010000000c") in payload[sync:sync + 64]


def watchdog(payload: bytes) -> int:
    """The TIMER value the image's header arms (BITSTREAM.CONFIG.TIMER_CFG), 0 if
    none. Without it, a slot cut short by a power loss mid-write leaves the FPGA
    scanning the flash forever after --boot update instead of falling back."""
    sync = payload.find(bytes.fromhex("aa995566"))
    head = payload[sync:sync + 128] if sync >= 0 else b""
    k = head.find(bytes.fromhex("30022001"))
    while k >= 0 and k % 4:
        k = head.find(bytes.fromhex("30022001"), k + 1)
    return struct.unpack(">I", head[k + 4:k + 8])[0] if k >= 0 else 0


def usr_access(payload: bytes) -> int | None:
    """USR_ACCESS (AXSS, register 0x0D) as written by the bitstream: a type-1
    write packet 0x3001A001 followed by the value."""
    k = payload.find(b"\x30\x01\xa0\x01")
    while k >= 0 and k % 4:
        k = payload.find(b"\x30\x01\xa0\x01", k + 1)
    return struct.unpack(">I", payload[k + 4:k + 8])[0] if k >= 0 else None


def describe_id(v: int | None) -> str:
    """USR_ACCESS TIMESTAMP: day(5) month(4) year-2000(6) hour(5) min(6) sec(6)."""
    if v is None:
        return "unknown"
    d, mo, y = v >> 27, (v >> 23) & 0xF, (v >> 17) & 0x3F
    h, mi, s = (v >> 12) & 0x1F, (v >> 6) & 0x3F, v & 0x3F
    if 1 <= d <= 31 and 1 <= mo <= 12:
        return f"{v:08x} (built 20{y:02d}-{mo:02d}-{d:02d} {h:02d}:{mi:02d}:{s:02d})"
    return f"{v:08x}"


def parse_id(text: str) -> int | None:
    for tok in text.split():
        if tok.startswith("id=") and tok[3:11] != "unknown":
            try:
                return int(tok[3:11], 16)
            except ValueError:
                return None
    return None


class Flash:
    def __init__(self, ser: serial.SerialBase, baud: int, timeout: float) -> None:
        self.ser = ser
        self.bps = baud / 10
        self.timeout = timeout

    def _read(self, n: int, extra: float = 0.0) -> bytes:
        got = bytearray()
        deadline = time.monotonic() + self.timeout + extra + n / self.bps * 1.5
        while len(got) < n:
            if time.monotonic() > deadline:
                raise LoadError(f"flash bridge: wanted {n} B, got {len(got)}")
            got += self.ser.read(n - len(got))
        return bytes(got)

    def xfer(self, wdata: bytes, rlen: int) -> bytes:
        self.ser.write(struct.pack("<BHH", 0x01, len(wdata), rlen) + wdata)
        r = self._read(rlen + 1, len(wdata) / self.bps)
        if r[-1] != OK:
            raise LoadError(f"XFER {wdata[0]:#04x}: {ERRS.get(r[-1], hex(r[-1]))}")
        return r[:-1]

    def status(self) -> tuple[int, int, int]:
        return self.xfer(b"\x05", 1)[0], self.xfer(b"\x07", 1)[0], self.xfer(b"\x35", 1)[0]

    def read(self, addr: int, n: int) -> bytes:
        out = bytearray()
        while n > 0:
            k = min(n, 4096)
            out += self.xfer(bytes([0x13]) + struct.pack(">I", addr), k)
            addr += k
            n -= k
        return bytes(out)

    def crc(self, addr: int, n: int) -> int:
        """zlib.crc32 of n flash bytes at addr, computed by the FPGA."""
        self.ser.write(struct.pack("<BII", 0x04, addr, n))
        r = self._read(1, n / 1.2e6 + 1.0)
        if r[0] in ERRS and r[0] != 0xEE:
            raise LoadError(f"CRC {addr:#x}+{n}: {ERRS[r[0]]}")
        r += self._read(4, 0.5)
        if r[4] != OK:
            raise LoadError(f"CRC {addr:#x}+{n}: bad reply {r.hex()}")
        return struct.unpack("<I", r[:4])[0]

    def erase(self, addr: int) -> None:
        self.ser.write(struct.pack("<BI", 0x03, addr))
        st = self._read(1, 3.0)[0]
        if st in ERRS or st & 0x60:
            raise LoadError(f"erase {addr:#x}: SR1={st:#04x} {ERRS.get(st, 'E_ERR/P_ERR')}")

    def program(self, pages: list[tuple[int, bytes]], progress) -> None:
        inflight = 0
        sent = 0
        done = 0
        while done < len(pages):
            while sent < len(pages) and inflight < WINDOW:
                a, d = pages[sent]
                self.ser.write(struct.pack("<BI", 0x02, a) + d)
                sent += 1
                inflight += 1
            st = self._read(1, WINDOW * 261 / self.bps)[0]
            if st in ERRS or st & 0x60:
                raise LoadError(f"program {pages[done][0]:#x}: SR1={st:#04x} "
                                f"{ERRS.get(st, 'P_ERR')}")
            inflight -= 1
            done += 1
            progress(done)

    def end(self) -> None:
        self.ser.write(b"\xe5")
        self.ser.flush()


def flash_session(args, fast: bool):
    ser = open_link(args)
    text = handshake(ser, args.timeout, "RBFLASH1 FAST" if fast else "RBFLASH1", "RBFLASH")
    print(f"ESP: RBFLASH OK {text}")
    return ser, Flash(ser, 921600 if fast else 115200, args.timeout)


def check_chip(fl: Flash) -> None:
    jedec = fl.xfer(b"\x9f", 3)
    sr1, sr2, cr1 = fl.status()
    print(f"flash: JEDEC {jedec.hex()}  SR1={sr1:#04x} SR2={sr2:#04x} CR1={cr1:#04x}")
    if jedec != EXPECT_JEDEC:
        raise LoadError(f"unexpected flash ID {jedec.hex()} (want {EXPECT_JEDEC.hex()}): "
                        "the bridge is not talking to the S25FL256S")
    if sr1 & 0x1C:
        raise LoadError(f"block protection is set (SR1 BP bits {sr1 >> 2 & 7}); "
                        "clear it over JTAG first")
    if sr1 & 0x60:
        fl.xfer(b"\x30", 0)    # CLSR: clear a stale E_ERR/P_ERR


def ping(args) -> int | None:
    ser = open_link(args)
    try:
        return parse_id(handshake(ser, args.timeout, "RBPING1", "RBPING"))
    finally:
        ser.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    where = ap.add_mutually_exclusive_group(required=True)
    where.add_argument("--url")
    where.add_argument("--usb")
    ap.add_argument("--info", action="store_true")
    ap.add_argument("--write", metavar="BIT")
    ap.add_argument("--verify", metavar="BIT")
    ap.add_argument("--at", type=lambda s: int(s, 0), default=None,
                    help="flash address for --verify (default: update slot)")
    ap.add_argument("--readback", action="store_true",
                    help="verify by reading the flash back instead of by CRC (slow)")
    ap.add_argument("--boot", metavar="golden|update|HEX")
    ap.add_argument("--boot-mode", type=int, choices=(0, 1, 2, 3), default=0,
                    help=argparse.SUPPRESS)
    ap.add_argument("--ping", action="store_true")
    ap.add_argument("--fast", action="store_true",
                    help="921600 baud for flash data (EXPERIMENTAL: dropped bytes on the bench, 2026-11-01)")
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args()
    fast = args.fast

    try:
        if args.ping:
            print(f"running image: {describe_id(ping(args))}")

        if args.info or args.write or args.verify:
            ser, fl = flash_session(args, fast)
            try:
                check_chip(fl)
                if args.write:
                    data = bit_payload(args.write)
                    if len(data) > FLASH_SIZE - UPDATE:
                        raise LoadError(f"{args.write}: {len(data)} B does not fit the slot")
                    print(f"{args.write}: {len(data)} B, image id "
                          f"{describe_id(usr_access(data))}")
                    if not spi_32bit(data):
                        raise LoadError(f"{args.write} does not use SPI 32-bit addressing "
                                        "(BITSTREAM.CONFIG.SPI_32BIT_ADDR): booting it, or "
                                        "booting anything from it, would use the wrong WBSTAR")
                    if not watchdog(data) & 0x3FFFFFFF:
                        raise LoadError(f"{args.write} does not arm the configuration watchdog "
                                        "(BITSTREAM.CONFIG.TIMER_CFG): a write cut short by a "
                                        "power loss would hang the FPGA at --boot update")
                    t0 = time.monotonic()
                    end = UPDATE + len(data)
                    for a in range(UPDATE, end, SECTOR):
                        fl.erase(a)
                        print(f"\r  erased {a + SECTOR - UPDATE}/{end - UPDATE} B",
                              end="", flush=True)
                    print()
                    pages = []
                    for off in range(0, len(data), PAGE):
                        d = data[off:off + PAGE].ljust(PAGE, b"\xff")
                        if d != b"\xff" * PAGE:
                            pages.append((UPDATE + off, d))
                    fl.program(pages, lambda n: print(
                        f"\r  programmed {n}/{len(pages)} pages", end="", flush=True)
                        if n % 64 == 0 or n == len(pages) else None)
                    print(f"\n  written in {time.monotonic() - t0:.0f} s")
                    args.verify = args.verify or args.write
                    if args.at is None:
                        args.at = UPDATE
                if args.verify:
                    data = bit_payload(args.verify)
                    at = UPDATE if args.at is None else args.at
                    t0 = time.monotonic()
                    bad = None
                    use_crc = not args.readback
                    for off in range(0, len(data), CRC_CHUNK):
                        want = data[off:off + CRC_CHUNK]
                        if use_crc:
                            try:
                                same = fl.crc(at + off, len(want)) == zlib.crc32(want)
                            except LoadError as exc:
                                if off:
                                    raise
                                # A loader or ESP image from before CRC: the ESP
                                # drops the session, so start a fresh one.
                                print(f"  no on-FPGA CRC ({exc}); reading back instead")
                                fl.end()
                                ser.close()
                                time.sleep(2.0)
                                ser, fl = flash_session(args, fast)
                                use_crc = False
                        if use_crc and not same:
                            # Bisect by CRC to the first bad 4 KiB, read only that.
                            lo, hi = 0, len(want)
                            while hi - lo > 4096:
                                mid = lo + ((hi - lo) // 2 + 4095) // 4096 * 4096
                                part = want[lo:mid]
                                if fl.crc(at + off + lo, len(part)) == zlib.crc32(part):
                                    lo = mid
                                else:
                                    hi = mid
                            got = fl.read(at + off + lo, hi - lo)
                            i = next((i for i in range(hi - lo) if got[i] != want[lo + i]), None)
                            if i is not None:
                                bad = at + off + lo + i
                                break
                            same = True
                        if not use_crc:
                            got = fl.read(at + off, len(want))
                            same = got == want
                            if not same:
                                i = next(i for i in range(len(want)) if got[i] != want[i])
                                bad = at + off + i
                                break
                        print(f"\r  verified {off + len(want)}/{len(data)} B"
                              f"{' (CRC)' if use_crc else ''}", end="", flush=True)
                    print()
                    if bad is not None:
                        raise LoadError(f"flash differs from {args.verify} at {bad:#x}")
                    print(f"flash at {at:#x} matches {args.verify} "
                          f"({time.monotonic() - t0:.0f} s)")
            finally:
                try:
                    fl.end()
                except Exception:
                    pass
                ser.close()

        if args.boot:
            addr = {"golden": GOLDEN, "update": UPDATE}.get(args.boot)
            addr = int(args.boot, 16) if addr is None else addr
            # Seconds after a flash session, not less: an IPROG ~0.3 s after
            # one was ignored on the bench (2026-11-01).
            time.sleep(3.0)
            ser = open_link(args)
            try:
                hello = f"RBBOOT1 {addr:x}" + (f" {args.boot_mode}" if args.boot_mode else "")
                print(f"ESP: RBBOOT OK {handshake(ser, args.timeout, hello, 'RBBOOT')}")
            finally:
                ser.close()
            if args.boot_mode == 2:
                return 0       # WBSTAR only: the FPGA keeps running
            t0 = time.monotonic()
            new = None
            while time.monotonic() - t0 < 30:
                time.sleep(1.0)
                try:
                    new = ping(args)
                    break
                except (LoadError, serial.SerialException):
                    continue
            if new is None:
                raise LoadError("no image answered within 30 s of IPROG: the FPGA is "
                                "running something without the ESP loader -- power-cycle "
                                "it to get golden back")
            print(f"after {time.monotonic() - t0:.0f} s the running image is "
                  f"{describe_id(new)}")
        return 0
    except (LoadError, serial.SerialException) as exc:
        print(f"esp_flash: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
