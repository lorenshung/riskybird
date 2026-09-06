#!/usr/bin/env python3
"""Reshape a raw HM01B0 capture into a viewable PNG.

    python3 hardware/ospi/frame-to-png.py camera-frame.raw 326

The raw file is what hardware/ospi/capture-frame.gdb dumps out of pixels[]: one
grayscale byte per pixel, in the order the capture core drained them, with no
header and no row padding.

Two things about the geometry, both of which look like a broken capture if you
get them wrong:

  * Reshape rows at the MEASURED width the core reports (LASTWIDTH), not at the
    width GEOM is configured for. On this sensor they differ -- 326 measured
    against 324 configured -- because each line carries two dummy pixels ahead
    of the active window. Reshaping at 324 shears the image one pixel per row.
  * Then drop those two leading columns, which is what --crop does by default.

Writes the PNG with zlib and struct rather than an imaging library, because the
bench Python has neither PIL nor numpy.
"""

import argparse
import pathlib
import struct
import sys
import zlib


def png(path, width, height, rows):
    """Write 8-bit grayscale rows (a list of bytes objects) as a PNG."""
    raw = b"".join(b"\x00" + r for r in rows)  # filter type 0 per scanline

    def chunk(tag, data):
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))

    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("raw", type=pathlib.Path, help="dump from capture-frame.gdb")
    ap.add_argument("width", type=int, help="MEASURED line width (the core's LASTWIDTH)")
    ap.add_argument("-o", "--out", type=pathlib.Path,
                    help="output PNG (default: the raw file with a .png suffix)")
    ap.add_argument("--crop", type=int, default=2, metavar="N",
                    help="drop N leading columns per row (default 2, the dummy pixels)")
    ap.add_argument("--stretch", action="store_true",
                    help="rescale to full black-to-white; the sensor's default exposure is dark")
    args = ap.parse_args()

    data = args.raw.read_bytes()
    if not data:
        sys.exit(f"{args.raw}: empty")
    if args.width <= args.crop:
        sys.exit(f"width {args.width} leaves nothing after cropping {args.crop} columns")

    height = len(data) // args.width
    if height == 0:
        sys.exit(f"{len(data)} bytes is less than one {args.width}-pixel row")
    trailing = len(data) - height * args.width

    lo, hi = min(data), max(data)
    if args.stretch and hi > lo:
        span = hi - lo
        table = bytes(((v - lo) * 255) // span if lo <= v <= hi else 0 for v in range(256))
        data = data.translate(table)

    rows = [data[y * args.width + args.crop:(y + 1) * args.width] for y in range(height)]
    width = args.width - args.crop
    out = args.out or args.raw.with_suffix(".png")
    png(out, width, height, rows)

    print(f"{args.raw}: {len(data)} bytes -> {width}x{height} "
          f"(cropped {args.crop} columns, {trailing} trailing bytes ignored)")
    print(f"  pixel range {lo}..{hi}" + ("  [stretched]" if args.stretch else ""))
    if hi - lo < 32 and not args.stretch:
        print("  low contrast -- re-run with --stretch to see whether there is an image in it")
    print(f"  wrote {out}")


if __name__ == "__main__":
    main()
