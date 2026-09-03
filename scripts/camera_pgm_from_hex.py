#!/usr/bin/env python3
"""camera_pgm_from_hex.py -- turn a camera_image_fpga console dump into a PGM (and PNG if Pillow).

The Rocket app (samples/riskybird/camera_image_fpga) prints one grayscale frame as hex rows
between markers:

    <<<PGM <W> <H>>>>
    <hex row 0>            # 2 hex chars per pixel, W pixels
    ...
    <hex row H-1>
    <<<END>>>

Usage:
    scripts/camera_pgm_from_hex.py CONSOLE.log [-o out]   # -> out.pgm (+ out.png if Pillow present)
    cat CONSOLE.log | scripts/camera_pgm_from_hex.py -     # read stdin
"""
import re
import sys
import argparse


def parse(text):
    m = re.search(r"<<<PGM\s+(\d+)\s+(\d+)>>>", text)
    if not m:
        sys.exit("no <<<PGM W H>>> marker found in input")
    w, h = int(m.group(1)), int(m.group(2))
    start = text.index("\n", m.end()) + 1
    end = text.find("<<<END>>>", start)
    body = text[start:end if end != -1 else len(text)]
    rows = []
    for line in body.splitlines():
        line = line.strip()
        if not re.fullmatch(r"[0-9a-fA-F]+", line or ""):
            continue
        vals = bytes(int(line[i:i + 2], 16) for i in range(0, len(line) - len(line) % 2, 2))
        rows.append(vals)
    if not rows:
        sys.exit("found the PGM header but no hex pixel rows")
    return w, h, rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="console log file, or - for stdin")
    ap.add_argument("-o", "--out", default="camera_frame", help="output basename (default camera_frame)")
    ap.add_argument("-m", "--median", type=int, default=0, metavar="N",
                    help="apply an NxN median filter (e.g. 3) to knock out fixed-pattern checkerboard noise")
    ap.add_argument("-s", "--stretch", action="store_true",
                    help="contrast-stretch to the 2nd..98th percentile (for underexposed frames)")
    ap.add_argument("-u", "--upscale", type=int, default=1, metavar="N", help="upscale the PNG Nx")
    args = ap.parse_args()

    text = sys.stdin.read() if args.input == "-" else open(args.input, "r", errors="replace").read()
    w, h, rows = parse(text)

    # Normalize to a rectangle: pad/truncate each row to the declared width, keep only full rows.
    rows = [(r + bytes(w))[:w] for r in rows]
    hh = len(rows)
    print(f"parsed {w}x{h} header; got {hh} pixel rows")

    pgm = f"{args.out}.pgm"
    with open(pgm, "wb") as f:
        f.write(f"P5\n{w} {hh}\n255\n".encode())
        for r in rows:
            f.write(r)
    print(f"wrote {pgm}  ({w}x{hh})")

    try:
        from PIL import Image, ImageFilter
        im = Image.frombytes("L", (w, hh), b"".join(rows))
        if args.median and args.median >= 2:
            im = im.filter(ImageFilter.MedianFilter(size=args.median | 1))  # size must be odd
        if args.stretch:
            import numpy as np
            a = np.asarray(im).astype("float32")
            lo, hi = np.percentile(a, 2), np.percentile(a, 98)
            im = Image.fromarray(np.clip((a - lo) * 255 / max(hi - lo, 1), 0, 255).astype("uint8"))
        if args.upscale > 1:
            im = im.resize((w * args.upscale, hh * args.upscale), Image.LANCZOS)
        im.save(f"{args.out}.png")
        print(f"wrote {args.out}.png"
              + (f"  (median{args.median})" if args.median else "")
              + ("  (stretched)" if args.stretch else ""))
    except Exception as e:  # noqa: BLE001
        print(f"(PNG skipped: {e}; the .pgm is viewable directly)")


if __name__ == "__main__":
    main()
