#!/usr/bin/env python3
"""
riskybird v3 -- CONTROLLER / VISUALIZER ground-station panel (WiFi).

Join the drone's SoftAP ("riskybird-<id>", open network -- see telem_wifi.c), then run this and
open the printed URL in a browser. It is a self-contained bridge + dashboard:

  * binds UDP :14550 and parses the 50 Hz telemetry downlink (the v1.1 text line from
    telem_wifi.c: it=/roll=/.../u=[...] vx=/vy=/vz=/zsp=/vbat=/st=),
  * serves a live dashboard over HTTP (attitude, altitude, drift/velocity, motors, battery,
    flight state, link stats) via Server-Sent Events -- NO external deps (stdlib only),
  * sends uplink commands to UDP :14551 (ESTOP / DISARM / HOVER_Z <mm> / PING) from the
    dashboard's buttons and shows the firmware's ACK.

Stdlib only (socket, threading, http.server, json) -- runs anywhere Python 3.8+ is, no pip.

Usage:
    python3 riskybird_panel.py                     # defaults: telem :14550, cmd->192.168.4.1:14551, UI :8080
    python3 riskybird_panel.py --drone 192.168.4.1 --http 8080
    python3 riskybird_panel.py --telem-port 14550 --cmd-port 14551

Then open  http://127.0.0.1:8080  (the URL is printed on start).
"""
import argparse
import base64
import csv
import datetime
import json
import math
import os
import re
import socket
import struct
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))

# ---- telemetry wire format (telem_wifi.c telem_format_v1) -------------------------------------
_F = r"(-?\d+\.\d+)"
CORE_RE = re.compile(
    r"it=(-?\d+)\s+dt=(-?\d+)ms\s+"
    r"roll=" + _F + r"\s+pitch=" + _F + r"\s+yaw=" + _F + r"\s+"
    r"z=" + _F + r"\s+tofv=(\d+)\s+tofh=" + _F + r"\s+"
    r"u=\[" + _F + r"\s+" + _F + r"\s+" + _F + r"\s+" + _F + r"\]"
)
# enriched tail (present on v1.1 firmware; optional so older builds still parse)
TAIL_RE = re.compile(
    r"vx=" + _F + r"\s+vy=" + _F + r"\s+vz=" + _F + r"\s+"
    r"zsp=" + _F + r"\s+vbat=" + _F + r"\s+st=(\d+)"
)
# dead-reckoned position (present on firmware >= the 3D-view build; \b so it doesn't match vx=)
POS_RE = re.compile(r"\bx=" + _F + r"\s+y=" + _F)
# v1.2 guard diagnostics on the WiFi line (telem_wifi.c telem_format_v1 tail):
#   amag=<|a| m/s2> apk=<decaying peak |a| m/s2> why=<estop reason or -> wm=<value at the latch>
# So a cut explains itself on the panel (the console printk that says so is unreadable in flight),
# and the operator can watch vibration (apk) build toward the impact/free-fall envelopes.
WHY_RE = re.compile(r"amag=" + _F + r"\s+apk=" + _F + r"\s+why=(\S+)\s+wm=" + _F)
# down-ToF health: consecutive failed fetches, last error rc (-5 EIO bus / -16,-116 timeout), recoveries
TOFDIAG_RE = re.compile(r"tfl=(-?\d+)\s+trc=(-?\d+)\s+trv=(\d+)")
# raw PMW3901 counts for FLOW CAL (flow.h flow_counts): running totals cx/cy/n/nbad/nerr since boot,
# per-line peak |delta| x,y, latest SQUAL + shutter. Sensor axes (drone +x = -fcx, +y = +fcy).
FLOWC_RE = re.compile(r"fcx=(-?\d+)\s+fcy=(-?\d+)\s+fn=(\d+)\s+fbad=(\d+)\s+ferr=(\d+)\s+"
                      r"fpk=(\d+),(\d+)\s+fsq=(\d+)\s+fsh=(\d+)(?:\s+fraw=(\d+),(\d+),(\d+))?")
# raw 35x35 PMW3901 frame row (FLOWIMG command): "FIMG <seq> <rc> <row> <70 hex chars>"
FIMG_RE = re.compile(r"^FIMG\s+(\d+)\s+(-?\d+)\s+(\d+)\s+([0-9a-f]{70})\s*$")

# compact telemetry from the FPGA build (telem_uart.c "RBT ..." line -- fewer fields than the ESP
# format; motors/drift/battery are absent on the FPGA path so those stay None/blank in the UI).
RBT_RE = re.compile(
    r"RBT\s+it=(-?\d+)\s+r=" + _F + r"\s+p=" + _F + r"\s+y=" + _F + r"\s+z=" + _F +
    r"\s+vz=" + _F + r"\s+h=" + _F + r"\s+tv=(\d+)\s+fl=(-?\d+)"
    r"(?:\s+cam=(\d+)\s+camm=(\d+))?"
)
# side-ToF walls appended to the RBT line: "wl=<front>,<back>,<left>,<right> wsq=<seq>" (mm; -1=no target)
WALLS_RE = re.compile(r"wl=(-?\d+),(-?\d+),(-?\d+),(-?\d+)\s+wsq=(\d+)")
# camera snapshot chunk: "IMG s=<seq> k=<chunk>/<total> w=<W> h=<H> <base64>"
IMG_RE = re.compile(r"IMG\s+s=(\d+)\s+k=(\d+)/(\d+)\s+w=(\d+)\s+h=(\d+)\s+([A-Za-z0-9+/=]+)")
# camera clip markers framing a WiFi-drained clip: "CLIP start c=<id> n=<n> w=<W> h=<H> src=.." / "CLIP done c=<id> n=<n>"
CLIP_RE = re.compile(r"CLIP\s+(start|done)\s+c=(\d+)\s+n=(\d+)(?:\s+w=(\d+)\s+h=(\d+))?")
# firmware ACK line (e.g. "ACK SNAP", "ACK PONG") -- surfaced to the dashboard.
ACK_RE = re.compile(r"^ACK\s+(.+?)\s*$")

FLAG_ARMED, FLAG_ESTOP, FLAG_ARMING, FLAG_CALDONE = 1, 2, 4, 8


def gibbs_to_quat_euler(rx, ry, rz):
    """The FC emits attitude as the Gibbs/Rodrigues vector (q_xyz / q_w), which is singular near
    180 deg (q_w -> 0). Reconstruct the true unit quaternion (exact) + proper ZYX Euler for a
    display that is correct at ALL attitudes (incl. inverted on the bench)."""
    n2 = rx * rx + ry * ry + rz * rz
    qw = 1.0 / math.sqrt(1.0 + n2)
    qx, qy, qz = rx * qw, ry * qw, rz * qw
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    sp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sp)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return qw, qx, qy, qz, roll, pitch, yaw


def parse_rbt(line):
    """Compact FPGA telemetry ('RBT it=.. r=.. p=.. y=.. z=.. vz=.. h=.. tv=.. fl=.. cam=.. camm=..').
    Maps onto the same dict shape as the ESP format; absent fields stay None."""
    m = RBT_RE.search(line)
    if not m:
        return None
    g = m.groups()
    fl = int(g[8])                    # fl= is the status-flag bitfield (same bits as st=), not flow
    d = {
        "it": int(g[0]), "dt_ms": 0,
        "roll": float(g[1]), "pitch": float(g[2]), "yaw": float(g[3]),
        "z": float(g[4]), "vz": float(g[5]), "tofh": float(g[6]),
        "tofv": int(g[7]), "st": fl,
        "cam": int(g[9]) if g[9] is not None else None,
        "camm": int(g[10]) if g[10] is not None else None,
        "u": [None, None, None, None],
        "vx": None, "vy": None, "zsp": None, "vbat": None, "x": None, "y": None,
    }
    qw, qx, qy, qz, roll, pitch, yaw = gibbs_to_quat_euler(d["roll"], d["pitch"], d["yaw"])
    d["qw"], d["qx"], d["qy"], d["qz"] = qw, qx, qy, qz
    d["roll"], d["pitch"], d["yaw"] = roll, pitch, yaw
    d["armed"] = bool(fl & FLAG_ARMED); d["estop"] = bool(fl & FLAG_ESTOP)
    d["arming"] = bool(fl & FLAG_ARMING); d["caldone"] = bool(fl & FLAG_CALDONE)
    w = WALLS_RE.search(line)         # side-ToF walls, when the bumper build appends them
    if w:
        d["walls"] = {"front": int(w.group(1)), "back": int(w.group(2)),
                      "left": int(w.group(3)), "right": int(w.group(4)), "seq": int(w.group(5))}
    return d


def parse_line(line):
    m = CORE_RE.search(line)
    if not m:
        return parse_rbt(line)
    g = m.groups()
    d = {
        "it": int(g[0]), "dt_ms": int(g[1]),
        "roll": float(g[2]), "pitch": float(g[3]), "yaw": float(g[4]),
        "z": float(g[5]), "tofv": int(g[6]), "tofh": float(g[7]),
        "u": [float(g[8]), float(g[9]), float(g[10]), float(g[11])],
        "vx": None, "vy": None, "vz": None, "zsp": None, "vbat": None, "st": 0,
        "x": None, "y": None,
    }
    # roll/pitch/yaw from the wire are the Gibbs vector -> reconstruct the true quaternion + Euler.
    qw, qx, qy, qz, roll, pitch, yaw = gibbs_to_quat_euler(d["roll"], d["pitch"], d["yaw"])
    d["qw"], d["qx"], d["qy"], d["qz"] = qw, qx, qy, qz
    d["roll"], d["pitch"], d["yaw"] = roll, pitch, yaw
    t = TAIL_RE.search(line)
    if t:
        tg = t.groups()
        d.update(vx=float(tg[0]), vy=float(tg[1]), vz=float(tg[2]),
                 zsp=float(tg[3]), vbat=float(tg[4]), st=int(tg[5]))
    p = POS_RE.search(line)
    if p:
        d["x"] = float(p.group(1)); d["y"] = float(p.group(2))
    wm = WHY_RE.search(line)
    if wm:
        d["amag"] = float(wm.group(1)); d["apk"] = float(wm.group(2))
        d["why"] = wm.group(3); d["why_meas"] = float(wm.group(4))
    tm = TOFDIAG_RE.search(line)
    if tm:
        d["tof_fails"] = int(tm.group(1)); d["tof_rc"] = int(tm.group(2)); d["tof_recov"] = int(tm.group(3))
    fm = FLOWC_RE.search(line)
    if fm:
        g = [int(v) if v is not None else None for v in fm.groups()]
        d["flowc"] = {"cx": g[0], "cy": g[1], "n": g[2], "nbad": g[3], "nerr": g[4],
                      "pkx": g[5], "pky": g[6], "sq": g[7], "sh": g[8],
                      "rsum": g[9], "rmax": g[10], "rmin": g[11]}
    st = d["st"]
    d["armed"] = bool(st & FLAG_ARMED)
    d["estop"] = bool(st & FLAG_ESTOP)
    d["arming"] = bool(st & FLAG_ARMING)
    d["caldone"] = bool(st & FLAG_CALDONE)
    return d


# ---- shared state ----------------------------------------------------------------------------
class Hub:
    def __init__(self):
        self.cond = threading.Condition()
        self.latest = None          # last parsed telemetry dict
        self.count = 0              # total telemetry packets
        self.rate = 0.0             # EMA packets/sec
        self.last_rx = 0.0          # monotonic time of last packet
        self.last_client = None     # (ip, port) the telemetry came from (for reference)
        self._prev_rx = None

    def on_packet(self, d, src):
        now = time.monotonic()
        with self.cond:
            if self._prev_rx is not None:
                dt = now - self._prev_rx
                if dt > 0:
                    inst = 1.0 / dt
                    self.rate = inst if self.rate == 0 else (0.85 * self.rate + 0.15 * inst)
            self._prev_rx = now
            self.last_rx = now
            self.count += 1
            self.latest = d
            self.last_client = src
            self.cond.notify_all()

    def snapshot(self):
        now = time.monotonic()
        with self.cond:
            age_ms = None if self.last_rx == 0 else int((now - self.last_rx) * 1000)
            connected = age_ms is not None and age_ms < 750
            link = {
                "connected": connected,
                "rate": round(self.rate, 1) if connected else 0.0,
                "count": self.count,
                "age_ms": age_ms,
                "quality": min(100, int(round(self.rate / 50.0 * 100))) if connected else 0,
            }
            return {"link": link, "t": self.latest}

    def wait(self, timeout):
        with self.cond:
            self.cond.wait(timeout=timeout)


HUB = Hub()


# ---- camera snapshot reassembly --------------------------------------------------------------
def _gray_png(w, h, pix):
    """Encode w*h 8-bit grayscale bytes as a PNG (stdlib zlib only)."""
    def chunk(typ, data):
        return (struct.pack(">I", len(data)) + typ + data +
                struct.pack(">I", zlib.crc32(typ + data) & 0xffffffff))
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)   # 8-bit, colour type 0 = grayscale
    raw = bytearray()
    for y in range(h):
        raw.append(0)                                     # per-scanline filter 0 (none)
        raw += pix[y * w:(y + 1) * w]
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) +
            chunk(b"IDAT", zlib.compress(bytes(raw), 6)) + chunk(b"IEND", b""))


class SnapStore:
    """Reassembles IMG chunks by seq; keeps the last completed frame as a PNG."""
    def __init__(self):
        self.lock = threading.Lock()
        self.cur_seq = None
        self.chunks = {}          # k -> base64 str for the in-progress frame
        self.total = 0
        self.w = self.h = 0
        self.png = None           # last completed frame, PNG bytes
        self.png_seq = 0
        self.rx_at = 0.0

    def on_chunk(self, seq, k, total, w, h, b64):
        with self.lock:
            if seq != self.cur_seq:
                self.cur_seq, self.chunks, self.total = seq, {}, total
                self.w, self.h = w, h
            self.chunks[k] = b64
            self.rx_at = time.monotonic()
            if self.total and len(self.chunks) >= self.total:
                try:
                    data = base64.b64decode("".join(self.chunks[i] for i in range(self.total)))
                    if len(data) >= w * h:
                        self.png = _gray_png(w, h, data[:w * h])
                        self.png_seq = seq
                except Exception:
                    pass
                self.cur_seq, self.chunks = None, {}

    def status(self):
        with self.lock:
            prog = None
            if self.cur_seq is not None and self.total:
                prog = {"have": len(self.chunks), "total": self.total}
            return {"seq": self.png_seq, "have_png": self.png is not None,
                    "w": self.w if self.png else 0, "h": self.h if self.png else 0,
                    "progress": prog}

    def get_png(self):
        with self.lock:
            return self.png, self.png_seq


SNAP = SnapStore()


class ClipStore:
    """Collects a WiFi-drained camera clip: 'CLIP start c=<id> n=<n> w=<W> h=<H>' opens a clip, the
    per-frame IMG chunks (seq = (clip_id<<8)|frame_index) reassemble into grayscale frames, and
    'CLIP done c=<id>' finalizes it. The record worker then turns the ordered frames into a GIF."""
    def __init__(self):
        self.lock = threading.Lock()
        self.active = False
        self.clip_id = None
        self.exp_n = 0
        self.w = self.h = 0
        self.part = {}        # seq -> {total, chunks{k:b64}}
        self.frames = {}      # frame_index -> grayscale bytes
        self.done_id = None
        self.done_frames = []
        self.done_wh = (0, 0)

    def on_marker(self, kind, c, n, w, h):
        with self.lock:
            if kind == "start":
                self.active, self.clip_id, self.exp_n = True, c, n
                if w:
                    self.w, self.h = w, h
                self.part, self.frames = {}, {}
            elif kind == "done" and c == self.clip_id:
                idxs = sorted(self.frames)
                self.done_frames = [self.frames[i] for i in idxs]
                self.done_wh = (self.w, self.h)
                self.done_id = c
                self.active = False

    def on_chunk(self, seq, k, total, w, h, b64):
        with self.lock:
            if not self.active or (seq >> 8) != self.clip_id:
                return
            idx = seq & 0xff
            p = self.part.setdefault(seq, {"total": total, "chunks": {}})
            p["chunks"][k] = b64
            if len(p["chunks"]) >= p["total"]:
                try:
                    data = base64.b64decode("".join(p["chunks"][i] for i in range(p["total"])))
                    if len(data) >= w * h:
                        self.frames[idx] = data[:w * h]
                        self.w, self.h = w, h
                except Exception:
                    pass
                self.part.pop(seq, None)

    def arm(self):
        """Clear any previously completed clip so the worker waits for a fresh one."""
        with self.lock:
            self.done_id, self.done_frames = None, []

    def take_done(self):
        with self.lock:
            if self.done_id is not None and self.done_frames:
                return self.done_id, list(self.done_frames), self.done_wh
            return None


CLIP = ClipStore()


# ---- camera clip recorder (WiFi UDP: 'REC <n>' -> :14551, IMG chunks back on :14550) ----------
class RecordStore:
    """State for the 'Record' button: a background worker triggers a clip over the WiFi UDP path
    (REC command -> recorder drains base64 IMG chunks -> CLIP) and leaves the grayscale GIF + raw
    frames here for the dashboard. No loader link / ELF needed."""
    def __init__(self):
        self.lock = threading.Lock()
        self.state = "idle"       # idle | recording | done | error
        self.msg = ""
        self.gif = None
        self.raw = None           # concatenated raw frames (for download/offline testing)
        self.w = self.h = self.n = 0
        self.seq = 0

    def status(self):
        with self.lock:
            return {"state": self.state, "msg": self.msg, "w": self.w, "h": self.h,
                    "n": self.n, "seq": self.seq, "have_gif": self.gif is not None}

    def get_gif(self):
        with self.lock:
            return self.gif, self.seq

    def get_raw(self):
        with self.lock:
            return self.raw, self.w, self.h, self.n, self.seq


REC = RecordStore()


# ---- manual-pilot command store (Steam Deck / gamepad) ---------------------------------------
class PilotStore:
    """Holds the latest commanded stick setpoint from the panel's gamepad controller so (a) the
    uplink thread can transmit it at a steady rate and (b) the session CSV can log the command that
    was active for each telemetry row. SI units in, clamped + formatted to the PILOT wire string by
    format_pilot(). engaged=False means the pilot is NOT steering (the FC watchdog then hovers)."""
    def __init__(self):
        self.lock = threading.Lock()
        self.vx = self.vy = self.yawrate = 0.0   # m/s, m/s, rad/s (commanded)
        self.alt = 0.0                            # m (commanded altitude target)
        self.engaged = False                      # pilot actively steering (deadman/engage on)
        self.trace = False                        # user flagged this as a trace-collection run
        self.armed = False                        # last ARM state the UI believes (informational)
        self.ts = 0.0                             # host clock of the last /pilot update
        self.seq = 0
        self.last_sent = ""                       # last PILOT datagram text (or "(dry-run) ...")
        self.tx_count = 0

    def update(self, vx, vy, yawrate, alt, engaged, trace, armed):
        with self.lock:
            self.vx, self.vy, self.yawrate, self.alt = vx, vy, yawrate, alt
            self.engaged, self.trace, self.armed = engaged, trace, armed
            self.ts = time.time()
            self.seq += 1

    def mark_sent(self, text):
        with self.lock:
            self.last_sent = text
            self.tx_count += 1

    def snapshot(self):
        with self.lock:
            age_ms = None if self.ts == 0 else int((time.time() - self.ts) * 1000)
            return {"vx": round(self.vx, 3), "vy": round(self.vy, 3),
                    "yawrate": round(self.yawrate, 3), "alt": round(self.alt, 3),
                    "engaged": self.engaged, "trace": self.trace, "armed": self.armed,
                    "age_ms": age_ms, "tx": self.tx_count, "last_sent": self.last_sent.strip()}

    def log_row(self):
        """Columns appended to each telemetry CSV row (the commanded setpoint at that instant)."""
        with self.lock:
            return [int(self.engaged), int(self.trace),
                    round(self.vx, 3), round(self.vy, 3),
                    round(self.yawrate, 3), round(self.alt, 3)]


PILOT = PilotStore()


def format_pilot(cfg, vx, vy, yawrate, alt):
    """Clamp SI stick commands and format the PILOT uplink line the FC parses:
        PILOT <vx_mm/s> <vy_mm/s> <yawrate_mrad/s> <alt_mm>
    Clamps mirror the FC-side guards so a bad value never leaves the host either."""
    def clamp(v, lo, hi):
        return lo if v < lo else hi if v > hi else v
    vmax = cfg["pilot_vmax"]
    ymax = cfg["pilot_yawmax"]
    amax = cfg["pilot_altmax"]
    vx = clamp(vx, -vmax, vmax)
    vy = clamp(vy, -vmax, vmax)
    yawrate = clamp(yawrate, -ymax, ymax)
    alt = clamp(alt, 0.0, amax)
    return "PILOT %d %d %d %d\n" % (round(vx * 1000), round(vy * 1000),
                                    round(yawrate * 1000), round(alt * 1000))


def record_worker(drone_ip, cmd_port, nframes, reqs=4, req_gap=3.0, timeout=60.0):
    """Record a camera clip over the working WiFi UDP path: send 'REC <n>\\n' to the drone's command
    port (a datagram can be missed by the ~8-deep uart FIFO, so fire a few), let the recorder drain
    base64 IMG chunks to UDP :14550 (the rx thread feeds them into CLIP), then assemble the grayscale
    frames into a GIF. No loader link / ELF needed."""
    n = max(1, min(int(nframes), 16))
    CLIP.arm()
    with REC.lock:
        REC.state, REC.msg, REC.gif, REC.raw = "recording", f"requesting {n} frames", None, None
    got = None
    for i in range(reqs):
        ack = send_command(drone_ip, cmd_port, f"REC {n}\n")
        with REC.lock:
            REC.msg = f"REC {n} sent ({i + 1}/{reqs}); {ack[:40]}"
        t1 = time.time()
        while time.time() - t1 < req_gap:
            got = CLIP.take_done()
            if got:
                break
            time.sleep(0.2)
        if got:
            break
    t0 = time.time()
    while got is None and time.time() - t0 < timeout:
        got = CLIP.take_done()
        if got:
            break
        time.sleep(0.2)
    if got is None:
        with REC.lock:
            REC.state, REC.msg = "error", f"no clip received within {int(timeout)}s"
        print("[panel] record error: no clip (is the ttyACM console drain attached + lease up?)")
        return
    cid, frames, (w, h) = got
    raw = b"".join(frames)
    gif = None
    try:
        import io
        from PIL import Image
        imgs = [Image.frombytes("L", (w, h), fr.ljust(w * h, b"\0")) for fr in frames if fr]
        big = [im.resize((im.width * 3, im.height * 3), Image.NEAREST) for im in imgs]
        buf = io.BytesIO()
        big[0].save(buf, format="GIF", save_all=True, append_images=big[1:], duration=150, loop=0)
        gif = buf.getvalue()
    except Exception as exc:
        print(f"[panel] gif skipped: {exc}")
    with REC.lock:
        REC.state = "done"
        REC.msg = f"clip {cid}: {len(frames)} frames {w}x{h}" + ("" if gif else " (raw only)")
        REC.gif, REC.raw = gif, raw
        REC.w, REC.h, REC.n, REC.seq = w, h, len(frames), REC.seq + 1
    print(f"[panel] record done: clip {cid} {len(frames)} frames {w}x{h}")


# ---- optical-flow calibration (FLOW CAL mode) ------------------------------------------------
def _load_flowcal_solver():
    """tools/flow-cal.py (hyphenated, so importlib): its Thresholds + reject() are the rules every
    FLOW CAL run is judged by, so the panel and the offline solver can never disagree."""
    import importlib.util
    path = os.path.normpath(os.path.join(HERE, "..", "..", "flow-cal.py"))
    try:
        spec = importlib.util.spec_from_file_location("flow_cal", path)
        mod = importlib.util.module_from_spec(spec)
        import sys
        sys.modules["flow_cal"] = mod       # @dataclass resolves its module through sys.modules
        spec.loader.exec_module(mod)
        return mod
    except Exception as exc:   # the panel still records; runs just aren't judged
        print(f"[panel] flow-cal solver not loaded ({path}): {exc}")
        return None


FLOWCAL_SOLVER = _load_flowcal_solver()


class FlowCal:
    """Hand-slide optical-flow calibration on the flow rail (riskybot-scad-prints/yaw_flow/FLOW_RAIL.md).

    START snapshots the firmware's running raw-count totals; the operator slides the stand stop to
    stop at the paced speed; STOP (or auto-stop: motion, then 0.8 s still) differences them. Per run
    K = D / (h * |sum counts on the chosen axis|) -- dt cancels, so the pacing only keeps the slide
    smooth (and inside the sensor's speed range); it does not enter the maths. Runs are judged by
    tools/flow-cal.py's reject() and also written as FLOWCAL RUN lines so that tool can re-solve them.
    Motors are never touched: this only reads telemetry."""
    MOVE_COUNTS = 6        # counts per telemetry line (~20 ms) that mean "still moving"
    START_COUNTS = 80      # travel-axis counts since START that mean "the slide has begun"
                           #   (~1.6 cm at h 0.1; ~1.6% of a 1 m slide, which is ~5000 counts)
    STILL_S = 0.8          # auto-stop after this long still, once it has moved
    K_FW = 0.00205         # FLOW_RAD_PER_COUNT the firmware flies with now (flow.c default)

    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None          # latest flowc dict (+ host time, tofh)
        self.run = None             # the run in progress
        self.runs = []
        self.next_id = 1
        self.log_dir = None
        self.stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    # -- telemetry hook (rx thread) --
    def on_packet(self, d):
        fc = d.get("flowc")
        if fc is None:
            return
        now = time.monotonic()
        with self.lock:
            prev = self.latest
            self.latest = dict(fc, t=now, tofh=d.get("tofh"), tofv=d.get("tofv"))
            r = self.run
            if r is None:
                return
            step = 0 if prev is None else abs(fc["cx"] - prev["cx"]) + abs(fc["cy"] - prev["cy"])
            b = r["base"]
            along = abs((fc["cx"] - b["cx"]) if r["axis"] == "x" else (fc["cy"] - b["cy"]))
            # The slide STARTS only once the travel axis has really moved (a stationary sensor
            # drifts ~20 counts/s in bursts, which a per-line threshold alone mistakes for motion
            # and auto-stop would then end the run before the push). After that, any line with
            # >= MOVE_COUNTS keeps it alive; STILL_S without one ends it.
            if r["t_move0"] is None:
                if along >= self.START_COUNTS:
                    r["t_move0"] = r["t_move1"] = now
            elif step >= self.MOVE_COUNTS:
                r["t_move1"] = now
            if r["t_move0"] is not None:          # stats over the slide itself
                r["sq"].append(fc["sq"]); r["sh"].append(fc["sh"])
                r["pkx"] = max(r["pkx"], fc["pkx"]); r["pky"] = max(r["pky"], fc["pky"])
                if d.get("tofv") and d.get("tofh") is not None:
                    r["tof"].append(d["tofh"])
            r["last"] = self.latest
            if r["auto"] and r["t_move1"] is not None and now - r["t_move1"] > self.STILL_S:
                self._finish_locked("auto")

    # -- operator actions (HTTP threads) --
    def start(self, axis, distance, seconds, height, auto=True):
        with self.lock:
            if self.latest is None:
                return "no flow counts in telemetry -- flash the flow-cal firmware (fcx=/fcy= fields)"
            if self.run is not None:
                return "a run is already in progress"
            self.run = {"id": self.next_id, "axis": axis, "D": distance, "T": seconds, "h": height,
                        "auto": auto, "base": dict(self.latest), "last": dict(self.latest),
                        "t0": time.monotonic(), "t_move0": None, "t_move1": None,
                        "sq": [], "sh": [], "tof": [], "pkx": 0, "pky": 0}
            self.next_id += 1
            return None

    def stop(self):
        with self.lock:
            if self.run is None:
                return "no run in progress"
            self._finish_locked("stop")
            return None

    def cancel(self):
        with self.lock:
            self.run = None

    def clear(self):
        with self.lock:
            self.runs = []

    def drop(self, rid):
        with self.lock:
            self.runs = [r for r in self.runs if r["id"] != rid]

    def _finish_locked(self, how):
        r, self.run = self.run, None
        b, e = r["base"], r["last"]
        sumdx, sumdy = e["cx"] - b["cx"], e["cy"] - b["cy"]
        n, nbad, nerr = e["n"] - b["n"], e["nbad"] - b["nbad"], e["nerr"] - b["nerr"]
        ms = int(1000 * (r["t_move1"] - r["t_move0"])) if r["t_move0"] and r["t_move1"] else 0
        sq, sh = r["sq"] or [0], r["sh"] or [0]
        fields = {"id": r["id"], "ms": ms, "n": n, "err": nerr, "sumdx": sumdx, "sumdy": sumdy,
                  "peakdx": r["pkx"], "peakdy": r["pky"], "n_ok": n - nbad,
                  "okdx": sumdx, "okdy": sumdy,
                  "sq_min": min(sq), "sq_max": max(sq), "sq_mean": round(sum(sq) / len(sq)),
                  "sh_min": min(sh), "sh_max": max(sh), "sh_mean": round(sum(sh) / len(sh))}
        counts = sumdx if r["axis"] == "x" else sumdy
        other = sumdy if r["axis"] == "x" else sumdx
        why = None
        if FLOWCAL_SOLVER is not None:
            why = FLOWCAL_SOLVER.reject(FLOWCAL_SOLVER.Run(fields), r["axis"], FLOWCAL_SOLVER.Thresholds())
        k = r["D"] / (r["h"] * abs(counts)) if counts else None
        tof = r["tof"]
        res = {"id": r["id"], "axis": r["axis"], "D": r["D"], "h": r["h"], "T": r["T"], "how": how,
               "counts": counts, "other": other, "dir": "+" if counts >= 0 else "−",
               "ms": ms, "n": n, "nbad": nbad, "nerr": nerr, "sq_mean": fields["sq_mean"],
               "leak": round(abs(other) / abs(counts), 3) if counts else None,
               "tof_mean": round(sum(tof) / len(tof), 4) if tof else None,
               "K": k, "ok": why is None and k is not None, "why": why,
               "ts": datetime.datetime.now().isoformat(timespec="seconds")}
        self.runs.append(res)
        self._log(r, fields)

    def _log(self, r, f):
        if not self.log_dir:
            return
        # one file per (axis, D, h): exactly what `tools/flow-cal.py --axis --distance --height` takes
        name = f"flowcal_{self.stamp}_{r['axis']}_D{round(r['D'] * 1000)}_h{round(r['h'] * 1000)}.log"
        path = os.path.join(self.log_dir, name)
        new = not os.path.exists(path)
        try:
            with open(path, "a") as fh:
                if new:
                    fh.write(f"# riskybird panel FLOW CAL  axis={r['axis']} D={r['D']} m h={r['h']} m\n"
                             f"# solve: tools/flow-cal.py --axis {r['axis']} --distance {r['D']} "
                             f"--height {r['h']} {name}\n")
                fh.write("FLOWCAL RUN " + " ".join(f"{k}={v}" for k, v in f.items()) + "\n")
        except OSError as exc:
            print(f"[panel] flowcal log failed: {exc}")

    @staticmethod
    def _stats(ks):
        import statistics
        if not ks:
            return None
        med = statistics.median(ks)
        sp = statistics.pstdev(ks) if len(ks) > 1 else 0.0
        return {"n": len(ks), "median": med, "mean": statistics.fmean(ks), "spread": sp,
                "spread_pct": (100.0 * sp / med) if med else None}

    def status(self):
        now = time.monotonic()
        with self.lock:
            lat = self.latest
            live = None
            r = self.run
            if r is not None:
                e, b = r["last"], r["base"]
                c = (e["cx"] - b["cx"]) if r["axis"] == "x" else (e["cy"] - b["cy"])
                o = (e["cy"] - b["cy"]) if r["axis"] == "x" else (e["cx"] - b["cx"])
                t_go = r["t_move0"]
                live = {"id": r["id"], "axis": r["axis"], "D": r["D"], "T": r["T"], "h": r["h"],
                        "counts": c, "other": o,
                        "est_m": abs(c) * self.K_FW * r["h"],        # with the firmware's current K
                        "moving": t_go is not None,
                        "elapsed": (now - t_go) if t_go else 0.0,
                        "armed_s": now - r["t0"],
                        "sq": e.get("sq"), "nbad": e["nbad"] - b["nbad"]}
            summ = {}
            for ax in ("x", "y"):
                good = [x for x in self.runs if x["ok"] and x["axis"] == ax]
                summ[ax] = {"all": self._stats([x["K"] for x in good]),
                            "+": self._stats([x["K"] for x in good if x["dir"] == "+"]),
                            "−": self._stats([x["K"] for x in good if x["dir"] == "−"])}
            return {"have_counts": lat is not None and now - lat["t"] < 1.0,
                    "sq": lat["sq"] if lat else None, "sh": lat["sh"] if lat else None,
                    "raw": [lat.get("rsum"), lat.get("rmax"), lat.get("rmin")] if lat else None,
                    "tofh": lat["tofh"] if lat else None, "tofv": lat["tofv"] if lat else None,
                    "k_fw": self.K_FW, "live": live, "runs": self.runs[-40:], "summary": summ,
                    "solver": FLOWCAL_SOLVER is not None}


FLOWCAL = FlowCal()


# ---- UDP telemetry receiver ------------------------------------------------------------------
# ---- session CSV log (timestamped, one file per run) -----------------------------------------
LOG_FIELDS = ["host_ts", "host_iso", "it", "dt_ms", "roll", "pitch", "yaw",
              "qw", "qx", "qy", "qz", "x", "y", "z",
              "tofv", "tofh", "vx", "vy", "vz", "zsp", "vbat",
              "u0", "u1", "u2", "u3", "st", "armed", "estop", "arming", "caldone",
              # commanded manual-pilot stick setpoint active at this telemetry row (trace data):
              "cmd_engaged", "cmd_trace", "cmd_vx", "cmd_vy", "cmd_yawrate", "cmd_alt",
              # v1.2 guard diagnostics (blank on older firmware): WHY an estop latched + the value
              # measured at the latch, and |accel| / decaying-peak |accel| (vibration), m/s^2
              "why", "why_meas", "amag", "apk",
              "tof_fails", "tof_rc", "tof_recov",
              # raw PMW3901 counts (flow-cal firmware; blank otherwise): running totals since boot
              # (difference consecutive rows for per-tick deltas), per-tick peaks, SQUAL, shutter, pixels
              "fcx", "fcy", "fn", "fbad", "ferr", "fpkx", "fpky", "fsq", "fsh", "frsum", "frmax", "frmin"]


class Logger:
    """Append every parsed telemetry packet to a timestamped CSV (host clock + all fields)."""
    def __init__(self, path):
        self.path = path
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(LOG_FIELDS)
        self.n = 0

    def write(self, d):
        now = time.time()
        self.w.writerow([
            f"{now:.3f}", datetime.datetime.now().isoformat(timespec="milliseconds"),
            d["it"], d["dt_ms"], d["roll"], d["pitch"], d["yaw"],
            d["qw"], d["qx"], d["qy"], d["qz"], d["x"], d["y"], d["z"],
            d["tofv"], d["tofh"], d["vx"], d["vy"], d["vz"], d["zsp"], d["vbat"],
            d["u"][0], d["u"][1], d["u"][2], d["u"][3], d["st"],
            int(d["armed"]), int(d["estop"]), int(d["arming"]), int(d["caldone"]),
            *PILOT.log_row(),
            d.get("why", ""), d.get("why_meas", ""), d.get("amag", ""), d.get("apk", ""),
            d.get("tof_fails", ""), d.get("tof_rc", ""), d.get("tof_recov", ""),
            *[("" if (fc := d.get("flowc")) is None or fc.get(k) is None else fc[k])
              for k in ("cx", "cy", "n", "nbad", "nerr", "pkx", "pky", "sq", "sh", "rsum", "rmax", "rmin")],
        ])
        self.n += 1
        if self.n % 25 == 0:      # flush a few times/second so a kill loses < 0.5 s
            self.f.flush()


LOGGER = None   # set in main() unless --no-log


class FlowFrame:
    """Raw 35x35 PMW3901 frames (FLOWIMG -> FIMG rows on the downlink). Assembles rows per sequence
    number, computes exposure / texture / focus stats, and logs each complete frame as a PGM."""
    N = 35

    def __init__(self):
        self.lock = threading.Lock()
        self.rows = {}; self.seq = None; self.rc = 0
        self.last = None            # latest complete frame: dict(seq, rc, t, px, stats, file)
        self.log_dir = None
        self.requested_t = 0.0
        self.rx_rows = 0; self.bad_lines = []

    def on_row(self, seq, rc, row, hexs):
        with self.lock:
            self.rx_rows += 1
            # the firmware sends each frame twice (loss cover); ignore the repeat of a completed frame
            # (a capture starves the downlink thread, so repeats can trail by seconds: key on "no new
            # FLOWIMG request since that frame completed", not on a time window)
            if self.last and seq == self.last["seq"] and self.requested_t < self.last["t"]:
                return
            if seq != self.seq:
                self.seq, self.rc, self.rows = seq, rc, {}
            self.rows[row] = bytes.fromhex(hexs)
            if len(self.rows) == self.N:
                px = [v for r in range(self.N) for v in self.rows[r]]
                self.last = {"seq": seq, "rc": rc, "t": time.time(), "px": px,
                             "stats": self._stats(px), "file": self._save(seq, px)}
                self.rows = {}

    @classmethod
    def _stats(cls, px):
        n = cls.N; m = len(px)
        mean = sum(px) / m
        std = (sum((v - mean) ** 2 for v in px) / m) ** 0.5
        at = lambda r, c: px[r * n + c]
        grad = [abs(at(r, c + 1) - at(r, c)) + abs(at(r + 1, c) - at(r, c))
                for r in range(n - 1) for c in range(n - 1)]
        lap = [4 * at(r, c) - at(r - 1, c) - at(r + 1, c) - at(r, c - 1) - at(r, c + 1)
               for r in range(1, n - 1) for c in range(1, n - 1)]
        lm = sum(lap) / len(lap)
        return {"mean": round(mean, 1), "std": round(std, 1), "min": min(px), "max": max(px),
                "sat": round(sum(v >= 250 for v in px) / m, 3), "dark": round(sum(v <= 5 for v in px) / m, 3),
                "grad": round(sum(grad) / len(grad), 2),
                "lapvar": round(sum((v - lm) ** 2 for v in lap) / len(lap), 1)}

    def _save(self, seq, px):
        if not self.log_dir:
            return None
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            fn = os.path.join(self.log_dir, time.strftime("flowimg_%Y%m%d_%H%M%S") + f"_s{seq}.pgm")
            with open(fn, "wb") as f:
                f.write(b"P5\n%d %d\n255\n" % (self.N, self.N) + bytes(px))
            return os.path.basename(fn)
        except OSError:
            return None

    def status(self):
        with self.lock:
            return {"last": self.last, "pending_rows": len(self.rows), "pending_seq": self.seq,
                    "rows_have": sorted(self.rows), "requested_t": self.requested_t, "rx_rows": self.rx_rows,
                    "bad_lines": self.bad_lines[-5:]}


FLOWIMG = FlowFrame()


def udp_rx_thread(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", port))
    print(f"[panel] telemetry: listening on UDP 0.0.0.0:{port}")
    while True:
        try:
            data, src = s.recvfrom(2048)
        except OSError:
            continue
        for line in data.decode("utf-8", "replace").splitlines():
            fi = FIMG_RE.match(line)
            if not fi and line.startswith("FIMG"):
                FLOWIMG.bad_lines.append(line[:120])
            if fi:
                FLOWIMG.on_row(int(fi.group(1)), int(fi.group(2)), int(fi.group(3)), fi.group(4))
                continue
            cm = CLIP_RE.search(line)
            if cm:
                kind, c, nn = cm.group(1), int(cm.group(2)), int(cm.group(3))
                cw = int(cm.group(4)) if cm.group(4) else 0
                ch = int(cm.group(5)) if cm.group(5) else 0
                CLIP.on_marker(kind, c, nn, cw, ch)
                continue
            im = IMG_RE.search(line)
            if im:
                seq, k, total, w, h, b64 = im.groups()
                SNAP.on_chunk(int(seq), int(k), int(total), int(w), int(h), b64)
                CLIP.on_chunk(int(seq), int(k), int(total), int(w), int(h), b64)
                continue
            d = parse_line(line)
            if d:
                HUB.on_packet(d, f"{src[0]}:{src[1]}")
                try:
                    FLOWCAL.on_packet(d)
                except Exception as exc:
                    print(f"[panel] flowcal: {exc}")
                if LOGGER is not None:
                    try:
                        LOGGER.write(d)
                    except Exception:
                        pass


# ---- uplink command sender -------------------------------------------------------------------
def send_command(drone_ip, cmd_port, text):
    """Send one command datagram to the drone and wait briefly for the firmware ACK."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0.4)
    try:
        s.sendto(text.encode("ascii", "ignore"), (drone_ip, cmd_port))
        try:
            reply, _ = s.recvfrom(256)
            return reply.decode("utf-8", "replace").strip()
        except socket.timeout:
            return "(no ACK -- sent, drone may not be joined)"
    except OSError as e:
        return f"(send error: {e})"
    finally:
        s.close()


# One reused UDP socket for the high-rate pilot stream (fire-and-forget, no per-packet ACK wait).
_PILOT_SOCK = None
_PILOT_SOCK_LOCK = threading.Lock()


def send_pilot(drone_ip, cmd_port, text):
    """Transmit one PILOT setpoint datagram WITHOUT waiting for an ACK (the stream runs at 20-50 Hz,
    so a 0.4 s recvfrom per packet is not an option). Uses a single reused socket."""
    global _PILOT_SOCK
    with _PILOT_SOCK_LOCK:
        if _PILOT_SOCK is None:
            _PILOT_SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock = _PILOT_SOCK
    try:
        sock.sendto(text.encode("ascii", "ignore"), (drone_ip, cmd_port))
        return True
    except OSError:
        return False


# ---- HTTP server -----------------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass  # quiet

    def _send(self, code, ctype, body, extra=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            try:
                with open(os.path.join(HERE, "panel.html"), "rb") as f:
                    body = f.read()
            except OSError:
                body = b"<h1>panel.html not found next to riskybird_panel.py</h1>"
            self._send(200, "text/html; charset=utf-8", body)
        elif self.path == "/stream":
            self._stream()
        elif self.path == "/config":
            body = json.dumps({"drone": self.server.drone_ip, "cmd_port": self.server.cmd_port,
                               "pilot_vmax": self.server.pilot_vmax,
                               "pilot_yawmax": self.server.pilot_yawmax,
                               "pilot_altmax": self.server.pilot_altmax,
                               "pilot_dry_run": self.server.pilot_dry_run}).encode()
            self._send(200, "application/json", body)
        elif self.path.startswith("/snapshot.png"):
            png, seq = SNAP.get_png()
            if png is None:
                self._send(404, "text/plain", b"no snapshot yet")
            else:
                self._send(200, "image/png", png, extra={"X-Snap-Seq": str(seq)})
        elif self.path == "/snapshot.json":
            self._send(200, "application/json", json.dumps(SNAP.status()).encode())
        elif self.path == "/record.json":
            self._send(200, "application/json", json.dumps(REC.status()).encode())
        elif self.path == "/flowcal.json":
            self._send(200, "application/json", json.dumps(FLOWCAL.status()).encode())
        elif self.path == "/flowimg.json":
            self._send(200, "application/json", json.dumps(FLOWIMG.status()).encode())
        elif self.path == "/pilot.json":
            self._send(200, "application/json", json.dumps(PILOT.snapshot()).encode())
        elif self.path.startswith("/clip.gif"):
            gif, seq = REC.get_gif()
            if gif is None:
                self._send(404, "text/plain", b"no clip yet")
            else:
                self._send(200, "image/gif", gif, extra={"X-Clip-Seq": str(seq)})
        elif self.path.startswith("/clip.raw"):
            raw, w, h, n, seq = REC.get_raw()
            if raw is None:
                self._send(404, "text/plain", b"no clip yet")
            else:
                self._send(200, "application/octet-stream", raw,
                           extra={"X-Clip-Seq": str(seq), "X-Clip-WH": f"{w}x{h}", "X-Clip-N": str(n),
                                  "Content-Disposition": f'attachment; filename="clip_{seq}_{w}x{h}_{n}f.raw"'})
        else:
            self._send(404, "text/plain", b"not found")

    def do_POST(self):
        if self.path.startswith("/record"):
            self._do_record()
            return
        if self.path == "/pilot":
            self._do_pilot()
            return
        if self.path == "/flowcal":
            self._do_flowcal()
            return
        if self.path != "/cmd":
            self._send(404, "text/plain", b"not found")
            return
        n = int(self.headers.get("Content-Length", 0))
        cmd = self.rfile.read(n).decode("utf-8", "replace").strip()
        # whitelist: only forward known commands
        head = cmd.split()[0].upper() if cmd else ""
        if head not in ("ESTOP", "DISARM", "RESET", "HOVER_Z", "PROFILE", "PING", "SNAP", "PILOT", "FLOWIMG", "FLOWINIT", "TOFRESET"):
            self._send(400, "application/json", json.dumps({"ack": "(rejected: unknown command)"}).encode())
            return
        if head == "FLOWIMG":
            FLOWIMG.requested_t = time.time()
        ack = send_command(self.server.drone_ip, self.server.cmd_port, cmd)
        self._send(200, "application/json", json.dumps({"sent": cmd, "ack": ack}).encode())

    def _do_record(self):
        n = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(n).decode("utf-8", "replace").strip() if n else ""
        try:
            nframes = int(float(body)) if body else 8
        except ValueError:
            nframes = 8
        nframes = max(1, min(nframes, 16))
        if REC.status()["state"] == "recording":
            self._send(409, "application/json", json.dumps({"error": "already recording"}).encode())
            return
        threading.Thread(target=record_worker,
                         args=(self.server.drone_ip, self.server.cmd_port, nframes),
                         daemon=True).start()
        self._send(200, "application/json",
                   json.dumps({"started": True, "frames": nframes}).encode())

    def _do_pilot(self):
        """High-rate manual-pilot setpoint from the gamepad. JSON body:
           {vx, vy, yawrate (SI), alt (m), engaged, trace, armed}. Stores it (for logging) and, when
           engaged and not --pilot-dry-run, transmits the PILOT datagram to the drone (no ACK wait).
           Returns the exact wire string so the browser AND a headless curl test can verify mapping."""
        n = int(self.headers.get("Content-Length", 0))
        try:
            body = json.loads(self.rfile.read(n).decode("utf-8", "replace")) if n else {}
        except ValueError:
            self._send(400, "application/json", json.dumps({"error": "bad json"}).encode())
            return
        vx = float(body.get("vx", 0.0)); vy = float(body.get("vy", 0.0))
        yawrate = float(body.get("yawrate", 0.0)); alt = float(body.get("alt", 0.0))
        engaged = bool(body.get("engaged", False)); trace = bool(body.get("trace", False))
        armed = bool(body.get("armed", False))
        cfg = {"pilot_vmax": self.server.pilot_vmax, "pilot_yawmax": self.server.pilot_yawmax,
               "pilot_altmax": self.server.pilot_altmax}
        PILOT.update(vx, vy, yawrate, alt, engaged, trace, armed)
        text = format_pilot(cfg, vx, vy, yawrate, alt)
        tx = False
        if engaged:
            if self.server.pilot_dry_run:
                PILOT.mark_sent("(dry-run) " + text)
            else:
                tx = send_pilot(self.server.drone_ip, self.server.cmd_port, text)
                PILOT.mark_sent(text)
        self._send(200, "application/json",
                   json.dumps({"sent": text.strip(), "tx": tx,
                               "dry_run": self.server.pilot_dry_run}).encode())

    def _do_flowcal(self):
        """FLOW CAL actions. JSON body {action: start|stop|cancel|clear|drop, axis: x|y,
        distance (m), seconds (intended slide time), height (m, lens over floor), auto, id}."""
        n = int(self.headers.get("Content-Length", 0))
        try:
            b = json.loads(self.rfile.read(n).decode("utf-8", "replace")) if n else {}
        except ValueError:
            self._send(400, "application/json", json.dumps({"error": "bad json"}).encode())
            return
        a = b.get("action")
        err = None
        if a == "start":
            axis = "y" if b.get("axis") == "y" else "x"
            try:
                D = float(b.get("distance", 1.0)); T = float(b.get("seconds", 4.0))
                h = float(b.get("height", 0.1))
            except (TypeError, ValueError):
                D = T = h = -1.0
            if not (0.05 <= D <= 5.0 and 0.3 <= T <= 60.0 and 0.02 <= h <= 2.0):
                err = "distance / time / height out of range"
            else:
                err = FLOWCAL.start(axis, D, T, h, bool(b.get("auto", True)))
        elif a == "stop":
            err = FLOWCAL.stop()
        elif a == "cancel":
            FLOWCAL.cancel()
        elif a == "clear":
            FLOWCAL.clear()
        elif a == "drop":
            FLOWCAL.drop(int(b.get("id", -1)))
        else:
            err = "unknown action"
        self._send(200 if err is None else 409, "application/json",
                   json.dumps({"ok": err is None, "error": err, "state": FLOWCAL.status()}).encode())

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            while True:
                snap = HUB.snapshot()
                snap["snap"] = SNAP.status()   # camera snapshot availability/progress
                snap["pilot"] = PILOT.snapshot()  # last commanded stick setpoint (server echo)
                snap["flowcal"] = FLOWCAL.status()  # FLOW CAL mode: live slide + finished runs
                payload = json.dumps(snap)
                msg = f"data: {payload}\n\n".encode("utf-8")
                self.wfile.write(msg)
                self.wfile.flush()
                HUB.wait(0.5)   # wake on a fresh packet, else heartbeat every 0.5 s
        except (BrokenPipeError, ConnectionResetError, OSError):
            return


def main():
    ap = argparse.ArgumentParser(description="riskybird ground-station panel")
    ap.add_argument("--drone", default="192.168.4.1", help="drone SoftAP IP (command target)")
    ap.add_argument("--telem-port", type=int, default=14550)
    ap.add_argument("--cmd-port", type=int, default=14551)
    ap.add_argument("--http", type=int, default=8080, help="dashboard HTTP port")
    ap.add_argument("--bind", default="127.0.0.1", help="dashboard bind address")
    ap.add_argument("--log-dir", default=os.path.join(HERE, "logs"),
                    help="directory for timestamped session CSV logs (default: ./logs)")
    ap.add_argument("--no-log", action="store_true", help="disable session logging")
    ap.add_argument("--recorder-elf", default=os.environ.get("RB_RECORDER_ELF", ""),
                    help="camera_record zephyr.elf; enables the Record button (WiFi clip over RBPOKE)")
    ap.add_argument("--loader-url", default="socket://192.168.4.1:14552",
                    help="ESP loader link for the camera recorder")
    ap.add_argument("--cam-fps", type=int, default=30, help="camera fps (for secs->frames)")
    # ---- manual pilot (Steam Deck / gamepad) ----
    ap.add_argument("--pilot-dry-run", action="store_true",
                    help="gamepad PILOT setpoints are logged + echoed but NOT transmitted (bench test)")
    ap.add_argument("--pilot-vmax", type=float, default=0.5,
                    help="max commanded horizontal velocity (m/s) the stick maps to [FC also clamps]")
    ap.add_argument("--pilot-yawmax", type=float, default=0.6,
                    help="max commanded yaw rate (rad/s) the stick maps to [FC also clamps]")
    ap.add_argument("--pilot-altmax", type=float, default=1.8,
                    help="max commanded altitude target (m); must stay <= FC SAFE_MAX_HEIGHT_M (2.0)")
    args = ap.parse_args()

    global LOGGER
    if not args.no_log:
        os.makedirs(args.log_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        logpath = os.path.join(args.log_dir, f"riskybird_{stamp}.csv")
        LOGGER = Logger(logpath)
        print(f"[panel] logging:    {logpath}")
        FLOWCAL.log_dir = args.log_dir
    FLOWIMG.log_dir = args.log_dir

    threading.Thread(target=udp_rx_thread, args=(args.telem_port,), daemon=True).start()

    httpd = ThreadingHTTPServer((args.bind, args.http), Handler)
    httpd.drone_ip = args.drone
    httpd.cmd_port = args.cmd_port
    httpd.recorder_elf = args.recorder_elf
    httpd.loader_url = args.loader_url
    httpd.cam_fps = args.cam_fps
    httpd.pilot_dry_run = args.pilot_dry_run
    httpd.pilot_vmax = args.pilot_vmax
    httpd.pilot_yawmax = args.pilot_yawmax
    httpd.pilot_altmax = args.pilot_altmax
    url = f"http://{args.bind}:{args.http}"
    print(f"[panel] dashboard:  {url}")
    print(f"[panel] commands ->  udp {args.drone}:{args.cmd_port}   (ESTOP / DISARM / HOVER_Z / PING)")
    print(f"[panel] camera:      Record button -> UDP 'REC <n>' {args.drone}:{args.cmd_port} "
          "(WiFi clip, no ELF; needs the ttyACM console drain + an active DHCP lease)")
    print(f"[panel] pilot:       gamepad PILOT -> udp {args.drone}:{args.cmd_port} "
          f"(vmax {args.pilot_vmax} m/s, yawmax {args.pilot_yawmax} rad/s, altmax {args.pilot_altmax} m"
          + (", DRY-RUN" if args.pilot_dry_run else "") + ")")
    print("[panel] join the drone AP 'riskybird-<id>' first; Ctrl-C to stop.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[panel] bye")


if __name__ == "__main__":
    main()
