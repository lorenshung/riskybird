#!/usr/bin/env python3
"""Feed a ground station from the FPGA's wired console instead of a radio.

    python3 tools/bench/console-to-udp.py --follow ~/riskybird-logs/fpga-console.log
    python3 tools/bench/console-to-udp.py --serial /dev/ttyUSB9

WHY THIS EXISTS
---------------
The ground stations under
`tools/bench/groundstation/`
are Dima's, and they fly: `riskybird_panel.py` and the mobile page have both
driven a real drone. But they were written for the ESP-NATIVE flight
controller, where the board itself has WiFi and unicasts telemetry to UDP
:14550. They take no other source -- `riskybird_panel.py` has `--telem-port`
and nothing else.

The FPGA carrier has no radio. The Rocket core reaches the outside world over
uart1 to the ESP32-C6 and over the FTDI console, and `CONFIG_WIFI` is never set
in any `flight_controller` build, so nothing on this carrier ever sends that UDP
packet. That is the ONLY reason Dima's panel shows nothing here -- not the
format. The telemetry line the FPGA prints on its console already matches the
panel's own `CORE_RE` byte for byte.

So this bridge does one thing: read lines from the wired console and unicast
them to the port the panel is already listening on. No radio, no WiFi
association, no network configuration, and nothing on the board changes.

    FPGA console (FTDI) ---> this bridge ---> UDP 127.0.0.1:14550 ---> panel

PICK THE RIGHT SOURCE. `--serial` opens the tty directly and is the lowest
latency, but a tty has exactly one reader: if a bench console logger already
holds it (`rb console`, or the `cat /dev/ttyUSBn` that `rb-watch.sh` starts),
opening it here steals bytes from that logger and BOTH get a corrupted stream.
`--follow` reads the log file that logger is already writing, which costs a few
milliseconds and cannot contend. On a bench with `rb-watch` running, use
`--follow`.
"""
import argparse
import os
import socket
import subprocess
import sys
import time


def follow(path, from_start=0):
    """Yield lines appended to `path`, surviving truncation and rotation.

    The bench rotates `fpga-console.log` by replacing the symlink target, so
    the inode is re-checked rather than the handle being held forever.
    """
    handle = None
    inode = None
    while True:
        try:
            st = os.stat(path)
        except FileNotFoundError:
            time.sleep(0.5)
            continue
        if handle is None or st.st_ino != inode:
            if handle is not None:
                handle.close()
            handle = open(path, "r", errors="replace")
            inode = st.st_ino
            # A fresh open starts at the end: the point is live telemetry, and
            # replaying a megabyte of old console at the panel would draw a
            # flight that already happened.
            handle.seek(0 if from_start else 0, os.SEEK_END)
            if from_start:
                handle.seek(0)
        line = handle.readline()
        if line:
            yield line.rstrip("\n")
        else:
            # Truncated in place (not rotated) -- go back to the top.
            if handle.tell() > os.stat(path).st_size:
                handle.seek(0)
            time.sleep(0.05)


def serial_lines(port, baud):
    """Read a tty as a file, after stty puts it in raw mode.

    Deliberately no pyserial: this has to run on a bench box with nothing
    installed. Same approach riskybird-dash.py uses.
    """
    subprocess.run(
        ["stty", "-F", port, str(baud), "raw", "-echo", "-echoe", "-echok",
         "-crtscts", "-ixon", "-ixoff", "min", "0", "time", "5"],
        check=True,
    )
    with open(port, "rb", buffering=0) as fh:
        buf = b""
        while True:
            chunk = fh.read(4096)
            if not chunk:
                time.sleep(0.02)
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                yield line.decode("utf-8", "replace").rstrip("\r")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--follow", metavar="LOGFILE",
                     help="tail a console log file (safe alongside a bench logger)")
    src.add_argument("--serial", metavar="TTY",
                     help="read a tty directly (only if nothing else holds it)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--host", default="127.0.0.1",
                    help="where the ground station is listening (default: localhost)")
    ap.add_argument("--port", type=int, default=14550,
                    help="telemetry port the ground station binds (default: 14550)")
    ap.add_argument("--from-start", action="store_true",
                    help="with --follow, replay the whole file first")
    ap.add_argument("--echo", action="store_true",
                    help="also print forwarded lines, to see what is matching")
    ap.add_argument("--all", action="store_true",
                    help="forward every line; by default only lines that look "
                         "like telemetry are sent, so boot spam does not reach "
                         "the panel")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = (args.host, args.port)

    if args.serial:
        source = serial_lines(args.serial, args.baud)
        what = f"serial {args.serial} @ {args.baud}"
    else:
        source = follow(args.follow, args.from_start)
        what = f"follow {args.follow}"

    print(f"[bridge] {what}  ->  udp {args.host}:{args.port}", flush=True)
    if args.serial:
        print("[bridge] NOTE: a tty has one reader. If a bench console logger "
              "holds this port, both streams corrupt -- use --follow instead.",
              flush=True)

    sent = 0
    seen = 0
    last_report = time.time()
    for line in source:
        seen += 1
        # The two lines a ground station cares about. Everything else on this
        # console is boot chatter, stage reports and camera dumps, and sending
        # it only makes the panel's parser work harder.
        interesting = args.all or "it=" in line or "RBLINK" in line
        if interesting:
            try:
                sock.sendto((line + "\n").encode("utf-8", "replace"), dst)
                sent += 1
            except OSError as exc:
                print(f"[bridge] send failed: {exc}", file=sys.stderr, flush=True)
            if args.echo:
                print(line, flush=True)
        now = time.time()
        if now - last_report >= 10.0:
            print(f"[bridge] {sent} forwarded / {seen} seen", flush=True)
            last_report = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
