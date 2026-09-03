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
        from PIL import Image
        im = Image.frombytes("L", (w, hh), b"".join(rows))
        im.save(f"{args.out}.png")
        print(f"wrote {args.out}.png")
    except Exception as e:  # noqa: BLE001
        print(f"(PNG skipped: {e}; the .pgm is viewable directly)")


if __name__ == "__main__":
    main()
