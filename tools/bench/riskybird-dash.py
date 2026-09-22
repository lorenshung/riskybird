#!/usr/bin/env python3
"""
RiskyBird ground station for the FPGA carrier: UDP telemetry in, dashboard out.

    python3 tools/bench/riskybird-dash.py            # -> http://127.0.0.1:8080
    python3 tools/bench/riskybird-dash.py --selftest # no drone needed; proves the path

ONE FILE, STDLIB ONLY, NO NETWORK FETCHES. The page is served from a string in
this file, so it works on a laptop that is joined to the drone's SoftAP and
therefore has no route to anything else. Python 3.8+; macOS ships one.

WHY THIS EXISTS RATHER THAN riskybird_panel.py
----------------------------------------------
backends/zephyr-chipyard-sw/.../tools/groundstation/riskybird_panel.py is the
older, richer panel (3D view, drift rings, trend strip) and it still works -- its
CORE_RE matches this carrier's telemetry line, so run it if you want the 3D view.
But it was written for the ESP-NATIVE flight controller, and three of the things
it shows are structurally blank or wrong on the FPGA carrier:

  * it draws "motor duty" from `u=[...]`, the controller's command in
    [-0.583, +0.417] space, NOT from the `duty=[...]` field the FPGA actually
    emits. Those are different numbers and only one of them is a duty cycle.
  * it reads battery, velocity and flight state from the v1.1 tail
    (vx/vy/vz/zsp/vbat/st). The FPGA build does not emit that tail at all, so
    `st` is always 0 and the banner reads DISARMED forever -- including while the
    motors are spinning.
  * it has no idea about the ESP's own RBLINK health line, which is the only
    thing on the wire that can tell "link dead" from "motors commanded off".

This one parses what this carrier actually sends, and says "not in this build"
where a field is absent instead of drawing a zero.

SOURCES
-------
  --source udp     (default)  bind :14550, the ESP bridge unicasts to us
  --source serial  --port /dev/tty.usbserial-XXXX   the FTDI console, tethered.
                   No pyserial: the port is opened as a file after `stty` puts it
                   in raw mode. This is the zero-radio fallback -- it exercises
                   the whole dashboard with no WiFi in the picture at all.
  --source stdin   pipe anything line-oriented in
  --replay FILE    read a captured log as fast as it arrives (for demos)
"""

import argparse
import csv
import datetime
import json
import math
import os
import re
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# =============================================================================
# Wire formats
# =============================================================================
#
# The flight controller's line, copied out of the format string in
# backends/zephyr-chipyard-sw/samples/rose_flight_controller/src/main.cpp (the
# `if (ROSE_TELEM && (iter % ROSE_TELEM_DIV) == 0)` block) rather than guessed:
#
#   flight_controller: it=%d dt=%dms roll=%s%d.%03d pitch=%s%d.%03d yaw=%s%d.%03d
#     z=%s%d.%03d tofv=%d tofh=%s%d.%03d
#     u=[%s%d.%03d x4] duty=[%s%d.%03d x4]
#     tilt=est%ddeg/acc%ddeg |a|=%s%d.%03d
#
# The %s%d.%03d triples are the FP3 macro: a sign string, the integer part and
# three fraction digits, so every value reads as -0.012 / 0.034 on the wire.
#
# CORE_RE is deliberately the same shape as riskybird_panel.py's, so a line that
# parses there parses here. Everything after `u=[...]` is optional and matched
# separately: older builds do not emit duty/tilt, and a build with ROSE_BUMPER or
# ROSE_FLOW emits extra lines that are not telemetry at all.

_F = r"(-?\d+\.\d+)"

CORE_RE = re.compile(
    r"it=(-?\d+)\s+dt=(-?\d+)ms\s+"
    r"roll=" + _F + r"\s+pitch=" + _F + r"\s+yaw=" + _F + r"\s+"
    r"z=" + _F + r"\s+tofv=(\d+)\s+tofh=" + _F + r"\s+"
    r"u=\[" + _F + r"\s+" + _F + r"\s+" + _F + r"\s+" + _F + r"\]"
)
# The field riskybird_panel.py does not read: what was actually commanded to each
# gate, 0..1. This is the one to put on a dashboard next to the ESP's applied duty.
DUTY_RE = re.compile(
    r"duty=\[" + _F + r"\s+" + _F + r"\s+" + _F + r"\s+" + _F + r"\]"
)
# The honest tilt, in DEGREES. roll/pitch/yaw above are the Rodrigues/Gibbs
# vector (r = tan(theta/2) per axis) -- neither radians nor degrees -- and reading
# them as radians is what once put a 57-degree guard's real trip point at 90.
TILT_RE = re.compile(r"tilt=est(-?\d+)deg/acc(-?\d+)deg")
AMAG_RE = re.compile(r"\|a\|=" + _F)
# v1.1 enrichment. NOT emitted by the FPGA build today; parsed so that the day it
# is, the dashboard lights up instead of needing an edit.
TAIL_RE = re.compile(
    r"vx=" + _F + r"\s+vy=" + _F + r"\s+vz=" + _F + r"\s+"
    r"zsp=" + _F + r"\s+vbat=" + _F + r"\s+st=(\d+)"
)
POS_RE = re.compile(r"\bx=" + _F + r"\s+y=" + _F)

# The ESP's own health line, emitted by workloads/esp_bridge --mode motors. It is
# the only thing on the link that knows whether the motors are off because they
# were told to be or because the frames stopped.
RBLINK_RE = re.compile(
    r"RBLINK v1\s+ml=(\w+)\s+up=(\d+)\s+seq=(\d+)\s+fl=0x([0-9a-fA-F]+)\[([^\]]*)\]\s+"
    r"duty=\[(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\]\s+"
    r"fr=(\d+)\s+gap=(-?\d+)\s+lost=(\d+)\s+stale=(\d+)\s+"
    r"crc=(\d+)\s+noise=(\d+)\s+cuts=(\d+)\s+pwmerr=(\d+)\s+"
    r"kill=(\d+)\s+txt=(\d+)\s+cl=(-?\d+)"
)

# st= bits, from main.cpp's TELEM_FLAG_* (only present with the v1.1 tail).
FLAG_ARMED, FLAG_ESTOP, FLAG_ARMING, FLAG_CALDONE = 1, 2, 4, 8


def gibbs_to_euler(rx, ry, rz):
    """Gibbs/Rodrigues vector -> true quaternion -> proper ZYX Euler, in radians.

    The naive reading (treat rx as roll in radians) is wrong everywhere and
    catastrophically wrong past ~60 degrees, because r = tan(theta/2). Going
    through the quaternion is exact at all attitudes, including inverted on the
    bench, where q_w -> 0 and the Gibbs vector itself blows up.
    """
    n2 = rx * rx + ry * ry + rz * rz
    qw = 1.0 / math.sqrt(1.0 + n2)
    qx, qy, qz = rx * qw, ry * qw, rz * qw
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    sp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sp)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def parse_telem(line):
    m = CORE_RE.search(line)
    if not m:
        return None
    g = m.groups()
    roll, pitch, yaw = gibbs_to_euler(float(g[2]), float(g[3]), float(g[4]))
    d = {
        "it": int(g[0]), "dt_ms": int(g[1]),
        "gibbs": [float(g[2]), float(g[3]), float(g[4])],
        "roll": math.degrees(roll), "pitch": math.degrees(pitch),
        "yaw": math.degrees(yaw),
        "z": float(g[5]), "tofv": int(g[6]), "tofh": float(g[7]),
        "u": [float(g[8]), float(g[9]), float(g[10]), float(g[11])],
        "duty": None, "tilt_est": None, "tilt_acc": None, "amag": None,
        "vx": None, "vy": None, "vz": None, "zsp": None, "vbat": None,
        "st": None, "x": None, "y": None,
    }
    t = DUTY_RE.search(line)
    if t:
        d["duty"] = [float(v) for v in t.groups()]
    t = TILT_RE.search(line)
    if t:
        d["tilt_est"], d["tilt_acc"] = int(t.group(1)), int(t.group(2))
    t = AMAG_RE.search(line)
    if t:
        d["amag"] = float(t.group(1))
    t = TAIL_RE.search(line)
    if t:
        tg = t.groups()
        d.update(vx=float(tg[0]), vy=float(tg[1]), vz=float(tg[2]),
                 zsp=float(tg[3]), vbat=float(tg[4]), st=int(tg[5]))
    t = POS_RE.search(line)
    if t:
        d["x"], d["y"] = float(t.group(1)), float(t.group(2))
    st = d["st"]
    if st is not None:
        d["armed"] = bool(st & FLAG_ARMED)
        d["estop"] = bool(st & FLAG_ESTOP)
        d["arming"] = bool(st & FLAG_ARMING)
        d["caldone"] = bool(st & FLAG_CALDONE)
    return d


def parse_rblink(line):
    m = RBLINK_RE.search(line)
    if not m:
        return None
    g = m.groups()
    return {
        "ml": g[0], "uptime_ms": int(g[1]), "seq": int(g[2]),
        "flags": int(g[3], 16), "flags_text": g[4].strip(),
        "duty": [int(g[5]), int(g[6]), int(g[7]), int(g[8])],
        "frames": int(g[9]), "gap_ms": int(g[10]), "lost": int(g[11]),
        "stale": int(g[12]), "crc": int(g[13]), "noise": int(g[14]),
        "cuts": int(g[15]), "pwmerr": int(g[16]),
        "kill": bool(int(g[17])), "txt": int(g[18]), "clients": int(g[19]),
    }


# =============================================================================
# Shared state
# =============================================================================
TREND_LEN = 240   # ~24 s at 10 Hz, ~10 s at 25 Hz


class Hub:
    def __init__(self):
        self.cond = threading.Condition()
        self.telem = None
        self.rblink = None
        self.rblink_rx = 0.0
        self.count = 0
        self.other = 0            # console lines that are not telemetry
        self.last_line = ""
        self.rate = 0.0
        self.last_rx = 0.0
        self.worst_gap_ms = 0.0
        self.src = None
        self._prev = None
        self.trend = deque(maxlen=TREND_LEN)

    def on_telem(self, d, src, raw):
        now = time.monotonic()
        with self.cond:
            if self._prev is not None:
                dt = now - self._prev
                if dt > 0:
                    inst = 1.0 / dt
                    self.rate = inst if self.rate == 0 else (0.85 * self.rate + 0.15 * inst)
                    self.worst_gap_ms = max(self.worst_gap_ms, dt * 1000.0)
            self._prev = now
            self.last_rx = now
            self.count += 1
            self.telem = d
            self.src = src
            self.last_line = raw
            self.trend.append([round(d["z"], 4), round(d["tofh"], 4)])
            self.cond.notify_all()

    def on_rblink(self, d, src, raw):
        with self.cond:
            self.rblink = d
            self.rblink_rx = time.monotonic()
            self.src = src
            self.last_line = raw
            self.cond.notify_all()

    def on_other(self, raw):
        with self.cond:
            self.other += 1
            self.last_line = raw

    def snapshot(self):
        now = time.monotonic()
        with self.cond:
            age = None if self.last_rx == 0 else int((now - self.last_rx) * 1000)
            rb_age = None if self.rblink_rx == 0 else int((now - self.rblink_rx) * 1000)
            return {
                "link": {
                    # 1500 ms, not 750: a deliberately slow telemetry divisor is
                    # the RECOMMENDED setting for an offloaded flight build
                    # (~5 Hz = 200 ms between lines), and a dashboard that calls
                    # that "disconnected" would cry wolf on every flight.
                    "live": age is not None and age < 1500,
                    "rate": round(self.rate, 1),
                    "count": self.count,
                    "other": self.other,
                    "age_ms": age,
                    "worst_gap_ms": round(self.worst_gap_ms, 1),
                    "src": self.src,
                },
                "t": self.telem,
                "rb": self.rblink,
                "rb_age_ms": rb_age,
                "rb_live": rb_age is not None and rb_age < 1500,
                "trend": list(self.trend),
                "raw": self.last_line,
            }

    def wait(self, timeout):
        with self.cond:
            self.cond.wait(timeout=timeout)


HUB = Hub()
LOGGER = None

LOG_FIELDS = ["host_ts", "host_iso", "it", "dt_ms", "roll_deg", "pitch_deg",
              "yaw_deg", "z", "tofv", "tofh", "tilt_est", "tilt_acc", "amag",
              "u0", "u1", "u2", "u3", "duty0", "duty1", "duty2", "duty3",
              "vbat", "st"]


class Logger:
    """Every parsed telemetry packet, host clock plus all fields, one CSV a run."""

    def __init__(self, path):
        self.path = path
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(LOG_FIELDS)
        self.n = 0

    def write(self, d):
        duty = d["duty"] or [None] * 4
        self.w.writerow([
            f"{time.time():.3f}",
            datetime.datetime.now().isoformat(timespec="milliseconds"),
            d["it"], d["dt_ms"], round(d["roll"], 3), round(d["pitch"], 3),
            round(d["yaw"], 3), d["z"], d["tofv"], d["tofh"],
            d["tilt_est"], d["tilt_acc"], d["amag"],
            d["u"][0], d["u"][1], d["u"][2], d["u"][3],
            duty[0], duty[1], duty[2], duty[3], d["vbat"], d["st"],
        ])
        self.n += 1
        if self.n % 25 == 0:
            self.f.flush()


def feed_line(raw, src):
    """One line of anything -> the hub. The only entry point for every source."""
    raw = raw.rstrip("\r\n")
    if not raw:
        return
    d = parse_rblink(raw)
    if d:
        HUB.on_rblink(d, src, raw)
        return
    d = parse_telem(raw)
    if d:
        HUB.on_telem(d, src, raw)
        if LOGGER is not None:
            try:
                LOGGER.write(d)
            except Exception:
                pass
        return
    HUB.on_other(raw)


# =============================================================================
# Sources
# =============================================================================
def udp_source(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", port))
    print(f"[dash] telemetry: UDP 0.0.0.0:{port}")
    while True:
        try:
            data, src = s.recvfrom(4096)
        except OSError:
            continue
        who = f"{src[0]}:{src[1]}"
        for line in data.decode("utf-8", "replace").splitlines():
            feed_line(line, who)


def serial_source(port, baud):
    """No pyserial. `stty` puts the port in raw mode, then it is read as a file.

    macOS uses `stty -f`, Linux `stty -F`, and neither accepts the other's flag,
    so both are tried. A port that cannot be configured is still opened -- some
    USB CDC devices (the ESP's own console among them) ignore line settings
    entirely and work fine without them.
    """
    for flag in ("-f", "-F"):
        try:
            subprocess.run(["stty", flag, port, str(baud), "raw", "-echo"],
                           check=True, capture_output=True)
            break
        except (subprocess.CalledProcessError, FileNotFoundError):
            continue
    else:
        print(f"[dash] warning: could not stty {port}; reading it anyway")
    print(f"[dash] telemetry: serial {port} @ {baud}")
    with open(port, "rb", buffering=0) as fh:
        buf = b""
        while True:
            chunk = fh.read(256)
            if not chunk:
                time.sleep(0.05)
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                feed_line(line.decode("utf-8", "replace"), port)


def stdin_source():
    print("[dash] telemetry: stdin")
    for line in sys.stdin:
        feed_line(line, "stdin")


def replay_source(path, hz):
    print(f"[dash] telemetry: replaying {path} at {hz} Hz")
    delay = 1.0 / hz if hz > 0 else 0.0
    while True:
        with open(path, "r", errors="replace") as fh:
            for line in fh:
                feed_line(line, f"replay:{os.path.basename(path)}")
                if delay:
                    time.sleep(delay)
        if not delay:
            return


def selftest_source(port, hz):
    """Synthesise a drone and send it over the real UDP socket to ourselves.

    This is not a mock of the parser -- it is the same datagrams, the same
    socket, the same regexes and the same SSE stream the drone drives. It exists
    so the dashboard can be proven end-to-end on a laptop with no hardware
    attached, which is exactly the situation this tool gets set up in.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = ("127.0.0.1", port)
    print(f"[dash] SELFTEST: synthesising telemetry -> {dst[0]}:{dst[1]} at {hz} Hz")
    t0 = time.time()
    it = 0
    seq = 0
    frames = 0
    while True:
        t = time.time() - t0
        it += 1
        # Gibbs components, i.e. tan(theta/2): a +-12 degree roll/pitch wobble.
        roll_g = math.tan(math.radians(12.0 * math.sin(t * 0.7)) / 2)
        pitch_g = math.tan(math.radians(9.0 * math.sin(t * 0.5 + 1.1)) / 2)
        yaw_g = math.tan(math.radians(20.0 * math.sin(t * 0.13)) / 2)
        z = 0.25 + 0.05 * math.sin(t * 0.9)
        hover = 0.46
        duty = [max(0.0, min(1.0, hover + 0.03 * math.sin(t * 0.7 + i)))
                for i in range(4)]
        u = [d - 0.583 for d in duty]
        line = (
            "flight_controller: it=%d dt=1ms roll=%.3f pitch=%.3f yaw=%.3f "
            "z=%.3f tofv=1 tofh=%.3f u=[%.3f %.3f %.3f %.3f] "
            "duty=[%.3f %.3f %.3f %.3f] tilt=est%ddeg/acc%ddeg |a|=%.3f\n"
            % (it, roll_g, pitch_g, yaw_g, z, z + 0.008,
               u[0], u[1], u[2], u[3], duty[0], duty[1], duty[2], duty[3],
               int(abs(12.0 * math.sin(t * 0.7))), int(abs(12.5 * math.sin(t * 0.7))),
               9.81 + 0.05 * math.sin(t * 3))
        )
        sock.sendto(line.encode(), dst)
        if it % 2 == 0:
            seq = (seq + 1) % 256
            frames += 10
            applied = [int(d * 10000) for d in duty]
            applied = [min(a, 2500) for a in applied]     # the ESP's 25% ceiling
            rb = (
                "RBLINK v1 ml=up up=%d seq=%d fl=0x21[ARMED COMMITTED ] "
                "duty=[%d %d %d %d] fr=%d gap=20 lost=0 stale=0 crc=0 noise=0 "
                "cuts=0 pwmerr=0 kill=0 txt=%d cl=1\n"
                % (int(t * 1000), seq, applied[0], applied[1], applied[2],
                   applied[3], frames, it * 190)
            )
            sock.sendto(rb.encode(), dst)
        time.sleep(1.0 / hz if hz > 0 else 0.1)


# =============================================================================
# Uplink
# =============================================================================
def send_command(drone_ip, cmd_port, text):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0.6)
    try:
        s.sendto(text.encode("ascii", "ignore"), (drone_ip, cmd_port))
        try:
            reply, _ = s.recvfrom(512)
            return reply.decode("utf-8", "replace").strip()
        except socket.timeout:
            return "(no ACK -- sent, but nothing answered on :%d)" % cmd_port
    except OSError as exc:
        return f"(send error: {exc})"
    finally:
        s.close()


# =============================================================================
# HTTP
# =============================================================================
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self._send(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
        elif self.path == "/stream":
            self._stream()
        elif self.path == "/config":
            body = json.dumps({
                "drone": self.server.drone_ip,
                "cmd_port": self.server.cmd_port,
                "selftest": self.server.selftest,
            }).encode()
            self._send(200, "application/json", body)
        else:
            self._send(404, "text/plain", b"not found")

    def do_POST(self):
        if self.path != "/cmd":
            self._send(404, "text/plain", b"not found")
            return
        n = int(self.headers.get("Content-Length", 0))
        cmd = self.rfile.read(n).decode("utf-8", "replace").strip()
        head = cmd.split()[0].upper() if cmd else ""
        if head not in ("ESTOP", "RESET", "DISARM", "HOVER_Z", "PING"):
            self._send(400, "application/json",
                       json.dumps({"ack": "(rejected: unknown command)"}).encode())
            return
        ack = send_command(self.server.drone_ip, self.server.cmd_port, cmd)
        self._send(200, "application/json",
                   json.dumps({"sent": cmd, "ack": ack}).encode())

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            while True:
                payload = json.dumps(HUB.snapshot())
                self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
                HUB.wait(0.5)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return


# =============================================================================
# The page
# =============================================================================
# Colors are the validated default data-viz palette: categorical slot 1 (blue)
# and slot 2 (orange) for the two altitude series, and the reserved status
# palette for state. Both categorical pairs were run through the palette
# validator in both modes and pass every gate (worst-adjacent CVD dE 24.7 light /
# 26.8 dark). Status colours never carry meaning alone -- each ships with a word.
#
# The four motor bars deliberately share ONE hue: they are one measure (duty)
# across four entities, each directly labelled, which is a single-hue bar chart
# and not a four-series categorical problem.

PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RiskyBird Ground Station</title>
<style>
:root{
  color-scheme: light;
  --plane:#f9f9f7; --surface:#fcfcfb;
  --ink:#0b0b0b; --ink2:#52514e; --muted:#898781;
  --grid:#e1e0d9; --axis:#c3c2b7; --ring:rgba(11,11,11,0.10);
  --s1:#2a78d6; --s2:#eb6834;
  --good:#0ca30c; --warn:#fab219; --serious:#ec835a; --crit:#d03b3b;
}
@media (prefers-color-scheme: dark){
  :root:not([data-theme="light"]){
    color-scheme: dark;
    --plane:#0d0d0d; --surface:#1a1a19;
    --ink:#ffffff; --ink2:#c3c2b7; --muted:#898781;
    --grid:#2c2c2a; --axis:#383835; --ring:rgba(255,255,255,0.10);
    --s1:#3987e5; --s2:#d95926;
  }
}
:root[data-theme="dark"]{
  color-scheme: dark;
  --plane:#0d0d0d; --surface:#1a1a19;
  --ink:#ffffff; --ink2:#c3c2b7; --muted:#898781;
  --grid:#2c2c2a; --axis:#383835; --ring:rgba(255,255,255,0.10);
  --s1:#3987e5; --s2:#d95926;
}
*{box-sizing:border-box}
body{margin:0;background:var(--plane);color:var(--ink);
     font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif;}
.wrap{max-width:1080px;margin:0 auto;padding:16px;}
h1{font-size:15px;margin:0;font-weight:600;letter-spacing:.01em}
header{display:flex;align-items:baseline;gap:12px;flex-wrap:wrap;margin-bottom:12px}
header .sub{color:var(--muted);font-size:12px}
.card{background:var(--surface);border:1px solid var(--ring);border-radius:10px;
      padding:14px 16px;margin-bottom:12px}
.card h2{font-size:11px;letter-spacing:.09em;text-transform:uppercase;
         color:var(--muted);margin:0 0 10px;font-weight:600}
.grid{display:grid;gap:12px}
@media(min-width:760px){.cols2{grid-template-columns:1fr 1fr}}
.banner{display:flex;align-items:center;gap:12px;border-radius:10px;padding:14px 16px;
        border:1px solid var(--ring);background:var(--surface);margin-bottom:12px}
.banner .dot{width:14px;height:14px;border-radius:50%;flex:0 0 auto}
.banner .txt{font-size:17px;font-weight:650;letter-spacing:.01em}
.banner .why{color:var(--ink2);font-size:13px;margin-top:2px}
.tiles{display:grid;grid-template-columns:repeat(2,1fr);gap:12px}
@media(min-width:760px){.tiles{grid-template-columns:repeat(4,1fr)}}
.tile{background:var(--surface);border:1px solid var(--ring);border-radius:10px;padding:12px 14px}
.tile .k{font-size:11px;letter-spacing:.08em;text-transform:uppercase;color:var(--muted)}
.tile .v{font-size:26px;font-weight:600;margin-top:2px;line-height:1.1}
.tile .u{font-size:13px;color:var(--ink2);font-weight:400}
.tile .n{font-size:12px;color:var(--muted);margin-top:3px}
.row{display:flex;align-items:center;gap:10px;margin:7px 0}
.row .lab{width:74px;color:var(--ink2);font-size:12px;flex:0 0 auto}
.row .val{width:82px;text-align:right;font-variant-numeric:tabular-nums;font-size:13px;flex:0 0 auto}
.meter{position:relative;flex:1 1 auto;height:12px;background:var(--grid);border-radius:6px}
.meter i{position:absolute;top:0;bottom:0;background:var(--s1);border-radius:4px;display:block}
.meter .zero{position:absolute;top:-3px;bottom:-3px;width:1px;background:var(--axis)}
table{width:100%;border-collapse:collapse;font-size:13px}
td,th{padding:3px 0;text-align:left}
td.n,th.n{text-align:right;font-variant-numeric:tabular-nums}
th{color:var(--muted);font-weight:500;font-size:11px;letter-spacing:.06em;text-transform:uppercase}
.legend{display:flex;gap:14px;font-size:12px;color:var(--ink2);margin-bottom:6px;flex-wrap:wrap}
.legend b{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:5px;
          vertical-align:-1px}
button{font:inherit;padding:8px 14px;border-radius:8px;border:1px solid var(--ring);
       background:var(--surface);color:var(--ink);cursor:pointer}
button:hover{border-color:var(--axis)}
button.danger{background:var(--crit);border-color:var(--crit);color:#fff;font-weight:650}
.cmds{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.ack{margin-top:9px;font-size:12px;color:var(--ink2);font-family:ui-monospace,Menlo,monospace;
     word-break:break-all}
.note{font-size:12px;color:var(--ink2);margin-top:8px;padding-left:10px;
      border-left:3px solid var(--warn)}
.raw{font-family:ui-monospace,Menlo,Consolas,monospace;font-size:11px;color:var(--ink2);
     word-break:break-all;white-space:pre-wrap}
.absent{color:var(--muted);font-style:italic}
svg{display:block;max-width:100%}
</style>
</head>
<body>
<div class="wrap">

<header>
  <h1>RiskyBird ground station</h1>
  <span class="sub" id="srcline">waiting for a source...</span>
</header>

<div class="banner" id="telemBanner">
  <span class="dot" id="telemDot"></span>
  <div><div class="txt" id="telemTxt">NO TELEMETRY</div>
       <div class="why" id="telemWhy">Nothing has arrived yet.</div></div>
</div>

<div class="banner" id="motorBanner">
  <span class="dot" id="motorDot"></span>
  <div><div class="txt" id="motorTxt">MOTOR LINK: not reporting</div>
       <div class="why" id="motorWhy">No RBLINK line seen. Either the ESP is running the
       plain bridge (no motors in that image) or the motors image is not up.</div></div>
</div>

<div class="tiles" style="margin-bottom:12px">
  <div class="tile"><div class="k">Altitude</div>
    <div class="v"><span id="tZ">--</span><span class="u"> m</span></div>
    <div class="n" id="tZn">estimator</div></div>
  <div class="tile"><div class="k">Tilt</div>
    <div class="v"><span id="tTilt">--</span><span class="u">&deg;</span></div>
    <div class="n" id="tTiltn">est / acc</div></div>
  <div class="tile"><div class="k">Battery</div>
    <div class="v"><span id="tBat">--</span><span class="u"> V</span></div>
    <div class="n" id="tBatn">not in this build</div></div>
  <div class="tile"><div class="k">Telemetry rate</div>
    <div class="v"><span id="tRate">--</span><span class="u"> Hz</span></div>
    <div class="n" id="tRaten">&nbsp;</div></div>
</div>

<div class="grid cols2">

  <div class="card">
    <h2>Attitude</h2>
    <svg id="horizon" viewBox="0 0 320 150" role="img"
         aria-label="Artificial horizon showing roll and pitch"></svg>
    <div class="row"><span class="lab">roll</span>
      <span class="meter"><span class="zero" style="left:50%"></span><i id="mRoll"></i></span>
      <span class="val" id="vRoll">--</span></div>
    <div class="row"><span class="lab">pitch</span>
      <span class="meter"><span class="zero" style="left:50%"></span><i id="mPitch"></i></span>
      <span class="val" id="vPitch">--</span></div>
    <div class="row"><span class="lab">yaw</span>
      <span class="meter"><span class="zero" style="left:50%"></span><i id="mYaw"></i></span>
      <span class="val" id="vYaw">--</span></div>
    <div class="note">roll/pitch/yaw on the wire are the Rodrigues vector,
      r&nbsp;=&nbsp;tan(&theta;/2) per axis &mdash; not degrees and not radians. The degrees
      shown here are reconstructed through the quaternion. The <b>tilt</b> tile is the
      firmware's own degrees and is the number to trust when checking the guard.</div>
  </div>

  <div class="card">
    <h2>Motor duty</h2>
    <div class="legend">
      <span><b style="background:var(--s1)"></b>commanded by the FPGA (<code>duty=[]</code>)</span>
      <span><b style="background:var(--s2)"></b>applied at the gates (ESP <code>RBLINK</code>)</span>
    </div>
    <div id="motors"></div>
    <div class="note" id="motorNote">The two differ when the ESP's own duty ceiling
      (25&nbsp;% by default), a disarmed frame or a latched kill is in the way. That gap is
      the point of showing both.</div>
  </div>

  <div class="card">
    <h2>Altitude, last <span id="trendN">0</span> samples</h2>
    <div class="legend">
      <span><b style="background:var(--s1)"></b>z (estimator)</span>
      <span><b style="background:var(--s2)"></b>tofh (raw down-ToF)</span>
    </div>
    <svg id="trend" viewBox="0 0 320 110" role="img"
         aria-label="Altitude estimate and raw time-of-flight over time"></svg>
    <table style="margin-top:8px">
      <tr><td>z</td><td class="n" id="dZ">--</td>
          <td>tofh</td><td class="n" id="dToF">--</td>
          <td>tofv</td><td class="n" id="dTofv">--</td></tr>
    </table>
  </div>

  <div class="card">
    <h2>Link health</h2>
    <table>
      <tr><th>telemetry</th><th class="n">value</th><th>motor link</th><th class="n">value</th></tr>
      <tr><td>packets</td><td class="n" id="hCount">0</td>
          <td>frames</td><td class="n" id="hFrames">--</td></tr>
      <tr><td>rate</td><td class="n" id="hRate">--</td>
          <td>gap</td><td class="n" id="hGap">--</td></tr>
      <tr><td>age</td><td class="n" id="hAge">--</td>
          <td>lost</td><td class="n" id="hLost">--</td></tr>
      <tr><td>worst gap</td><td class="n" id="hWorst">--</td>
          <td>stale</td><td class="n" id="hStale">--</td></tr>
      <tr><td>non-telemetry lines</td><td class="n" id="hOther">0</td>
          <td>crc errors</td><td class="n" id="hCrc">--</td></tr>
      <tr><td>iteration</td><td class="n" id="hIt">--</td>
          <td>noise bytes</td><td class="n" id="hNoise">--</td></tr>
      <tr><td>loop dt</td><td class="n" id="hDt">--</td>
          <td>failsafe cuts</td><td class="n" id="hCuts">--</td></tr>
    </table>
    <div class="note" id="linkNote">A rising <b>noise</b> count means something is on the
      wire at the wrong rate. A rising <b>crc</b> count with noise flat means frames are
      being damaged &mdash; expected in small numbers when telemetry shares the wire.</div>
  </div>
</div>

<div class="card">
  <h2>Commands &rarr; UDP :<span id="cmdPort">14551</span></h2>
  <div class="cmds">
    <button class="danger" id="bEstop">ESTOP</button>
    <button id="bReset">RESET</button>
    <button id="bDisarm">DISARM</button>
    <button id="bPing">PING</button>
  </div>
  <div class="ack" id="ack">&nbsp;</div>
  <div class="note"><b>Read this before trusting a button.</b> The flight controller on
    this FPGA carrier has <em>no</em> uplink command reader &mdash; every <code>rose_cmd_*</code>
    hook in main.cpp is inside <code>#if defined(CONFIG_WIFI)</code>, which is never true
    for an FPGA build. DISARM therefore does nothing at all, and the "ACK" you see is
    generated by the ESP bridge to mean "delivered to the FPGA's UART", not "acted on".
    <b>ESTOP is different and is real</b> &mdash; but only when the ESP runs
    <code>esp_bridge --mode motors</code>, where it cuts the gates at the ESP itself and
    latches. The ACK says so in words when that happened. RESET clears that latch.</div>
</div>

<div class="card">
  <h2>Last line seen</h2>
  <div class="raw" id="raw">&mdash;</div>
</div>

</div>
<script>
const $ = id => document.getElementById(id);
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
let CFG = {drone:"192.168.4.1", cmd_port:14551, selftest:false};

fetch("/config").then(r=>r.json()).then(c=>{CFG=c; $("cmdPort").textContent=c.cmd_port;});

function fmt(v, d){ return (v===null||v===undefined) ? "--" : Number(v).toFixed(d===undefined?2:d); }
function absent(el, why){ el.textContent = "--"; el.classList.add("absent"); if(why) el.title = why; }

// Signed meter: a bar growing from the centre, clamped to +-span.
function meter(el, v, span){
  if(v===null||v===undefined){ el.style.width="0%"; return; }
  const f = Math.max(-1, Math.min(1, v/span));
  el.style.left  = (f<0 ? 50+f*50 : 50) + "%";
  el.style.width = Math.abs(f)*50 + "%";
}

function banner(dot, txt, why, color, label, reason){
  dot.style.background = color;
  txt.textContent = label;
  why.textContent  = reason;
}

function horizon(svg, rollDeg, pitchDeg){
  const W=320,H=150, cx=W/2, cy=H/2;
  const r = (rollDeg||0)*Math.PI/180;
  const off = Math.max(-60, Math.min(60, (pitchDeg||0)*1.6));
  const dx = Math.cos(r)*260, dy = Math.sin(r)*260;
  const px = -Math.sin(r)*off, py = Math.cos(r)*off;
  const parts = [];
  parts.push(`<rect x="0" y="0" width="${W}" height="${H}" rx="8" fill="${css('--grid')}"/>`);
  // the horizon itself
  parts.push(`<line x1="${cx+px-dx/2}" y1="${cy+py-dy/2}" x2="${cx+px+dx/2}" y2="${cy+py+dy/2}"
              stroke="${css('--s1')}" stroke-width="2.5" stroke-linecap="round"/>`);
  // pitch ladder, +-10 and +-20 degrees
  for(const d of [-20,-10,10,20]){
    const o = off - d*1.6, w = (Math.abs(d)===10? 34: 22);
    const lx = -Math.sin(r)*o, ly = Math.cos(r)*o;
    parts.push(`<line x1="${cx+lx-Math.cos(r)*w}" y1="${cy+ly-Math.sin(r)*w}"
                x2="${cx+lx+Math.cos(r)*w}" y2="${cy+ly+Math.sin(r)*w}"
                stroke="${css('--axis')}" stroke-width="1.5"/>`);
  }
  // fixed aircraft reference
  parts.push(`<path d="M${cx-30} ${cy} h20 l10 8 l10 -8 h20" fill="none"
              stroke="${css('--ink')}" stroke-width="2.5" stroke-linejoin="round"/>`);
  parts.push(`<circle cx="${cx}" cy="${cy}" r="2.5" fill="${css('--ink')}"/>`);
  svg.innerHTML = parts.join("");
}

function trend(svg, rows){
  const W=320,H=110,P=6;
  if(!rows.length){ svg.innerHTML=""; return; }
  let lo=Infinity, hi=-Infinity;
  for(const r of rows) for(const v of r){ if(v<lo)lo=v; if(v>hi)hi=v; }
  if(hi-lo < 0.05){ const m=(hi+lo)/2; lo=m-0.025; hi=m+0.025; }
  const x = i => P + i*(W-2*P)/Math.max(1, rows.length-1);
  const y = v => H-P - (v-lo)*(H-2*P)/(hi-lo);
  const path = k => rows.map((r,i)=>`${i?"L":"M"}${x(i).toFixed(1)} ${y(r[k]).toFixed(1)}`).join(" ");
  svg.innerHTML =
    `<rect x="0" y="0" width="${W}" height="${H}" rx="8" fill="none" stroke="${css('--grid')}"/>`+
    `<path d="${path(1)}" fill="none" stroke="${css('--s2')}" stroke-width="2"
       stroke-linejoin="round" stroke-linecap="round"/>`+
    `<path d="${path(0)}" fill="none" stroke="${css('--s1')}" stroke-width="2"
       stroke-linejoin="round" stroke-linecap="round"/>`+
    `<text x="${W-P}" y="${P+9}" text-anchor="end" font-size="10"
       fill="${css('--muted')}">${hi.toFixed(2)} m</text>`+
    `<text x="${W-P}" y="${H-P}" text-anchor="end" font-size="10"
       fill="${css('--muted')}">${lo.toFixed(2)} m</text>`;
}

function motorRows(cmd, applied){
  const box = $("motors");
  let html = "";
  for(let i=0;i<4;i++){
    const c = cmd ? cmd[i] : null;                 // 0..1
    const a = applied ? applied[i]/10000 : null;   // per 10000 -> 0..1
    html +=
      `<div class="row"><span class="lab">M${i+1} cmd</span>`+
      `<span class="meter"><i style="left:0;width:${((c||0)*100).toFixed(1)}%;`+
      `background:var(--s1)"></i></span>`+
      `<span class="val">${c===null?"--":(c*100).toFixed(1)+" %"}</span></div>`;
    if(applied){
      html +=
        `<div class="row"><span class="lab" style="color:var(--muted)">M${i+1} gate</span>`+
        `<span class="meter"><i style="left:0;width:${((a||0)*100).toFixed(1)}%;`+
        `background:var(--s2)"></i></span>`+
        `<span class="val">${(a*100).toFixed(1)} %</span></div>`;
    }
  }
  box.innerHTML = html;
}

function apply(s){
  const t = s.t, rb = s.rb, L = s.link;

  $("srcline").textContent =
    (CFG.selftest ? "SELFTEST (synthetic) — " : "") +
    "source " + (L.src || "none") + " · " + L.count + " telemetry packet(s)";

  // ---- telemetry banner
  if(L.live){
    banner($("telemDot"),$("telemTxt"),$("telemWhy"), css("--good"),
      "TELEMETRY LIVE",
      `${L.rate.toFixed(1)} Hz, last packet ${L.age_ms} ms ago.`);
  } else if(L.count > 0){
    banner($("telemDot"),$("telemTxt"),$("telemWhy"), css("--crit"),
      "TELEMETRY STOPPED",
      `Last packet ${L.age_ms} ms ago after ${L.count}. The FPGA stopped printing, the `+
      `ESP stopped forwarding, or the radio dropped.`);
  } else {
    banner($("telemDot"),$("telemTxt"),$("telemWhy"), css("--warn"),
      "NO TELEMETRY YET",
      L.other ? `${L.other} console line(s) arrived but none parsed as telemetry — `+
                `the link works; the build may not have ROSE_TELEM on.`
              : "Nothing has arrived. Check the AP, the lease, and that the FPGA is running.");
  }

  // ---- motor-link banner: the "dead vs commanded off" distinction
  if(rb && s.rb_live){
    if(rb.kill){
      banner($("motorDot"),$("motorTxt"),$("motorWhy"), css("--crit"),
        "KILL LATCHED AT THE ESP",
        "A remote ESTOP is latched. Every gate is held at 0 regardless of what the FPGA "+
        "commands. Press RESET or power-cycle the ESP to clear it.");
    } else if(rb.ml !== "up"){
      banner($("motorDot"),$("motorTxt"),$("motorWhy"), css("--crit"),
        "MOTOR LINK DEAD — FAILSAFE CUT",
        `No valid duty frame within the failsafe window, so all four gates went to 0 `+
        `(${rb.cuts} cut(s) so far). The FPGA is halted, crashed, or not sending.`);
    } else if(rb.flags & 0x01){
      banner($("motorDot"),$("motorTxt"),$("motorWhy"), css("--serious"),
        "ARMED — GATES LIVE",
        `Frames arriving (seq ${rb.seq}, ${rb.gap_ms} ms apart). Flags: ${rb.flags_text}.`);
    } else {
      banner($("motorDot"),$("motorTxt"),$("motorWhy"), css("--good"),
        "MOTORS COMMANDED OFF — LINK HEALTHY",
        `Frames are arriving normally; the FPGA is asking for zero. Flags: `+
        `${rb.flags_text}. This is NOT a dead link.`);
    }
  } else if(rb){
    banner($("motorDot"),$("motorTxt"),$("motorWhy"), css("--warn"),
      "MOTOR LINK: stale",
      `Last RBLINK ${s.rb_age_ms} ms ago. The ESP has stopped reporting.`);
  }

  if(!t){ return; }

  // ---- tiles
  $("tZ").textContent = fmt(t.z, 3);
  $("tZn").textContent = (t.zsp===null) ? "estimator" : `setpoint ${fmt(t.zsp,3)} m`;
  if(t.tilt_est===null){ absent($("tTilt"), "older build: no tilt= field"); }
  else { $("tTilt").textContent = t.tilt_est; $("tTilt").classList.remove("absent"); }
  $("tTiltn").textContent = (t.tilt_acc===null) ? "est / acc" :
      `est ${t.tilt_est}° / acc ${t.tilt_acc}°`;
  if(t.vbat===null){ absent($("tBat")); $("tBatn").textContent = "not in this build"; }
  else { $("tBat").textContent = fmt(t.vbat,2); $("tBat").classList.remove("absent");
         $("tBatn").textContent = "AIN5 sense"; }
  $("tRate").textContent = L.rate.toFixed(1);
  $("tRaten").textContent = L.live ? `age ${L.age_ms} ms` : "stopped";

  // ---- attitude
  horizon($("horizon"), t.roll, t.pitch);
  meter($("mRoll"),  t.roll,  45); $("vRoll").textContent  = fmt(t.roll,1)+"°";
  meter($("mPitch"), t.pitch, 45); $("vPitch").textContent = fmt(t.pitch,1)+"°";
  meter($("mYaw"),   t.yaw,  180); $("vYaw").textContent   = fmt(t.yaw,1)+"°";

  // ---- motors
  motorRows(t.duty, (rb && s.rb_live) ? rb.duty : null);
  if(!t.duty){
    $("motorNote").textContent =
      "This build emits no duty=[] field, so only u=[] is available and it is a "+
      "controller command, not a duty cycle. Nothing is drawn rather than a wrong number.";
  }

  // ---- altitude
  $("trendN").textContent = s.trend.length;
  trend($("trend"), s.trend);
  $("dZ").textContent   = fmt(t.z,3)+" m";
  $("dToF").textContent = fmt(t.tofh,3)+" m";
  $("dTofv").textContent = t.tofv ? "valid" : "INVALID";

  // ---- link health
  $("hCount").textContent = L.count;
  $("hRate").textContent  = L.rate.toFixed(1)+" Hz";
  $("hAge").textContent   = (L.age_ms===null?"--":L.age_ms+" ms");
  $("hWorst").textContent = L.worst_gap_ms.toFixed(0)+" ms";
  $("hOther").textContent = L.other;
  $("hIt").textContent    = t.it;
  $("hDt").textContent    = t.dt_ms+" ms";
  const setrb = (id, v, suffix) => $(id).textContent = (rb ? v+(suffix||"") : "--");
  setrb("hFrames", rb&&rb.frames); setrb("hGap", rb&&rb.gap_ms, " ms");
  setrb("hLost",   rb&&rb.lost);   setrb("hStale", rb&&rb.stale);
  setrb("hCrc",    rb&&rb.crc);    setrb("hNoise", rb&&rb.noise);
  setrb("hCuts",   rb&&rb.cuts);

  $("raw").textContent = s.raw || "—";
}

const es = new EventSource("/stream");
es.onmessage = e => { try{ apply(JSON.parse(e.data)); }catch(err){ console.error(err); } };

function cmd(text){
  fetch("/cmd", {method:"POST", body:text})
    .then(r=>r.json())
    .then(j=> $("ack").textContent = "> "+(j.sent||text)+"   "+(j.ack||""))
    .catch(e=> $("ack").textContent = "send failed: "+e);
}
$("bEstop").onclick  = ()=>cmd("ESTOP");
$("bReset").onclick  = ()=>cmd("RESET");
$("bDisarm").onclick = ()=>cmd("DISARM");
$("bPing").onclick   = ()=>cmd("PING");
document.addEventListener("keydown", e=>{ if(e.key==="e"||e.key==="E") cmd("ESTOP"); });
</script>
</body>
</html>
"""


# =============================================================================
def main():
    ap = argparse.ArgumentParser(
        description="RiskyBird ground station (FPGA carrier)")
    ap.add_argument("--source", choices=("udp", "serial", "stdin", "replay"),
                    default="udp")
    ap.add_argument("--telem-port", type=int, default=14550)
    ap.add_argument("--cmd-port", type=int, default=14551)
    ap.add_argument("--drone", default="192.168.4.1",
                    help="command target (the SoftAP gateway)")
    ap.add_argument("--port", help="serial device for --source serial")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--replay", help="log file for --source replay")
    ap.add_argument("--replay-hz", type=float, default=10.0)
    ap.add_argument("--http", type=int, default=8080)
    ap.add_argument("--bind", default="127.0.0.1",
                    help="0.0.0.0 to view from a phone on the same AP")
    ap.add_argument("--log-dir", default=None,
                    help="directory for the session CSV (default: ./riskybird-logs)")
    ap.add_argument("--no-log", action="store_true")
    ap.add_argument("--selftest", action="store_true",
                    help="synthesise telemetry over the real UDP path; no drone needed")
    ap.add_argument("--selftest-hz", type=float, default=10.0)
    args = ap.parse_args()

    global LOGGER
    if not args.no_log:
        d = args.log_dir or os.path.join(os.getcwd(), "riskybird-logs")
        os.makedirs(d, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(d, f"riskybird_{stamp}.csv")
        LOGGER = Logger(path)
        print(f"[dash] logging:   {path}")

    if args.source == "udp" or args.selftest:
        threading.Thread(target=udp_source, args=(args.telem_port,),
                         daemon=True).start()
    if args.selftest:
        threading.Thread(target=selftest_source,
                         args=(args.telem_port, args.selftest_hz),
                         daemon=True).start()
    elif args.source == "serial":
        if not args.port:
            ap.error("--source serial needs --port")
        threading.Thread(target=serial_source, args=(args.port, args.baud),
                         daemon=True).start()
    elif args.source == "stdin":
        threading.Thread(target=stdin_source, daemon=True).start()
    elif args.source == "replay":
        if not args.replay:
            ap.error("--source replay needs --replay FILE")
        threading.Thread(target=replay_source,
                         args=(args.replay, args.replay_hz), daemon=True).start()

    httpd = ThreadingHTTPServer((args.bind, args.http), Handler)
    httpd.drone_ip = args.drone
    httpd.cmd_port = args.cmd_port
    httpd.selftest = args.selftest
    print(f"[dash] dashboard: http://{args.bind}:{args.http}")
    print(f"[dash] commands:  udp {args.drone}:{args.cmd_port}  "
          f"(ESTOP is real only with esp_bridge --mode motors)")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[dash] bye")


if __name__ == "__main__":
    main()
