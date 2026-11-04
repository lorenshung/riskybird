#!/usr/bin/env python3
"""
RiskyBird roll-harness ("rig") tuning viewer: a curses TUI over a live batch.

    ./rb rig tui                         # newest raw log, no batch
    ./rb rig tui --latest                # + newest results/rig/<stamp>-<name>/
    ./rb rig tui --batch results/rig/20261104-120000-roll1
    ./rb rig tui --replay results/rig/20261104-120000-roll1   # finished batch, read-only
    ./rb rig tui --selftest              # synthetic drone + fake runner, no hardware
    python3 tools/rb/rig_tui.py ...      # the same, without the rb environment

ONE FILE, STDLIB ONLY, PYTHON 3.8+, for the same reason as tools/bench/riskybird-dash.py:
it has to run on a laptop joined to the drone's SoftAP, with no route to a package index.

WHAT IT READS (the "Host-side contracts" section of the rig spec; nothing else is assumed)
  * the live raw log written by `rb ground monitor` (results/riskybird-logs/raw_*.log, one
    `<host unix ts> <text>` per line), followed like `tail -F`, including rotation to a newer
    file. Lines it cares about: `RIG it=...`, `RIGGAIN`, `RIGGAINS`, `RIG EVENT`, `FCACK`,
    `FCNAK` and the ESP's `RBLINK v1 ...` health line.
  * a batch directory: batch.json, journal.jsonl (append-only), status.json (runner, ~2 Hz),
    control.json (this tool), trials/<n>/rec.csv.

WHAT IT WRITES
  * control.json, atomically (temp file + rename): {"pause", "abort", "seq"}. The runner acts
    on a higher seq. Pause never clears abort and abort never clears pause.
  * one `estop` event (source "tui") appended to journal.jsonl when ESTOP is pressed.
  * UDP to the drone's command port 14551: `ESTOP\\n` x3 on E/space, `STOP\\n` on s.
    The socket is created at start-up so ESTOP never waits on anything. ESTOP goes out FIRST,
    before any file is touched, and works with no batch, a stale log, or a broken render.

`q` QUITS THE VIEWER ONLY. The batch runner is a separate process and keeps going.
"""

from __future__ import annotations

import argparse
import csv
import datetime
import json
import locale
import math
import os
import random
import re
import shutil
import socket
import sys
import tempfile
import threading
import time
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "results" / "riskybird-logs"
RIG_DIR = ROOT / "results" / "rig"

DEFAULT_DRONE = "192.168.4.1"
CMD_PORT = 14551
ESTOP_REPEATS = 3
PLOT_WINDOW_S = 10.0
DUTY_CEILING = 0.95          # AUTOFLIGHT_MAX_DUTY in the rig-roll preset
FLASH_S = 4.0                # header flashes red this long after an ESTOP is sent
RIG_STALE_S = 1.0
RBLINK_STALE_S = 1.5
STATUS_STALE_S = 5.0
BACKLOG_BYTES = 256 * 1024   # how much of an existing log to read on attach

# The contract gives `score` but not its direction. Every other metric in the event is an
# error/cost (overshoot, rise, settle, rms, sat), so lower is treated as better.
SCORE_LOWER_IS_BETTER = True

# Colour bands for result cells: (green below, amber below, else red). Heuristics, tune freely.
METRIC_BANDS = {
    "overshoot_pct": (10.0, 25.0),
    "settle_ms": (800.0, 2000.0),
    "rms_err_deg": (1.0, 3.0),
    "sat_pct": (5.0, 20.0),
}

GAIN_ORDER = ["ATT_P", "RATE_P", "ATT_I", "RATE_I", "RATE_D", "ATT_GAIN", "D_LPF_HZ",
              "GYRO_LPF_HZ", "ROLL_TRIM", "MOTOR_TRIM_LEFT", "ATT_I_LIM", "RATE_I_LIM"]
RIG_STATES = ("IDLE", "ARMING", "RAMP_UP", "RUN", "RAMP_DOWN", "DONE", "ESTOP")
FLAG_ARMED, FLAG_ESTOP, FLAG_ARMING, FLAG_CALDONE = 1, 2, 4, 8

# =============================================================================
# Wire parsing (pure)
# =============================================================================

_TS_RE = re.compile(r"^\s*(\d{9,11}(?:\.\d+)?)\s(.*)$")
_HEAD_RE = re.compile(r"(?:^|[^A-Za-z0-9_])(RIGGAINS|RIGGAIN|RIG\s+EVENT|RIG|FCACK|FCNAK)(?=\s|$)")
_KV_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=(\[[^\]]*\]|\S+)")
_NUM_RE = re.compile(r"^[+-]?(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?$")
_RIG_INT_FIELDS = ("it", "st", "r", "p", "gx", "sp", "tgt", "coll", "exc", "ex", "gs", "vb")
_RIG_REQUIRED = ("it", "st", "rs", "r")


def split_log_line(line, last_ts=None):
    """`<host unix ts> <text>` -> (ts, text). A line without a timestamp keeps last_ts."""
    line = line.rstrip("\r\n")
    m = _TS_RE.match(line)
    if m:
        return float(m.group(1)), m.group(2)
    return last_ts, line


def _num(text):
    if not _NUM_RE.match(text):
        raise ValueError(text)
    return float(text)


def parse_rig_line(text):
    """Parse one FC rig line. Returns a dict with "kind" or None (not a rig line, or malformed).

    kinds: rig (telemetry), gain (RIGGAIN), gains (RIGGAINS), event (RIG EVENT), ack, nak.
    Leading junk before the keyword is tolerated (a datagram split mid-line).
    """
    if not text:
        return None
    m = _HEAD_RE.search(text)
    if not m:
        return None
    head = m.group(1)
    rest = text[m.end():].strip()
    if head.startswith("RIG") and "EVENT" in head:
        return {"kind": "event", "text": rest} if rest else None
    if head in ("FCACK", "FCNAK"):
        verb = rest.split()[0] if rest else ""
        if not verb:
            return None
        return {"kind": "ack" if head == "FCACK" else "nak", "verb": verb, "text": rest}
    if head == "RIGGAIN":
        mm = re.match(r"^([A-Za-z_][A-Za-z0-9_]*)=(\S+)\s*$", rest)
        if not mm:
            return None
        try:
            value = _num(mm.group(2))
        except ValueError:
            return None
        return {"kind": "gain", "name": mm.group(1), "value": value}
    if head == "RIGGAINS":
        gains, gs = {}, None
        for k, v in _KV_RE.findall(rest):
            try:
                val = _num(v)
            except ValueError:
                return None
            if k == "gs":
                gs = int(val)
            else:
                gains[k] = val
        if gs is None and not gains:
            return None
        return {"kind": "gains", "gs": gs, "gains": gains}
    # RIG telemetry
    fields = dict(_KV_RE.findall(rest))
    if any(k not in fields for k in _RIG_REQUIRED):
        return None
    out = {"kind": "rig"}
    try:
        for k in _RIG_INT_FIELDS:
            if k in fields:
                out[k] = int(fields[k])
            else:
                out[k] = None
        d = fields.get("d")
        if d is not None:
            if not (d.startswith("[") and d.endswith("]")):
                return None
            parts = d[1:-1].split()
            if len(parts) != 4:
                return None
            out["d"] = [int(x) for x in parts]
        else:
            out["d"] = None
    except ValueError:
        return None
    rs = fields["rs"].upper()
    if not re.match(r"^[A-Z_]+$", rs):
        return None
    out["rs"] = rs
    # ax=R|P (RIG-SPEC "Pitch axis"): the axis sp=/tgt= refer to; r=/p= stay physical. Absent on
    # firmware from before rig-pitch, which only ever controlled roll.
    out["ax"] = "P" if fields.get("ax", "R").upper() == "P" else "R"
    return out


AXIS_WORDS = {"R": "roll", "P": "pitch"}


def ctl_angle_mdeg(rig):
    """The controlled axis' angle (mdeg) of a parsed RIG line: p= when ax=P, else r=."""
    return rig.get("p") if rig.get("ax") == "P" else rig.get("r")


def axis_letter(v):
    """"roll"/"pitch"/"R"/"P"/0/1 -> "R"/"P"; None for anything else."""
    if isinstance(v, bool):
        return None
    if v in (0, 1):
        return "RP"[v]
    if isinstance(v, str):
        t = v.strip().lower()
        if t in ("r", "roll", "rig-roll"):
            return "R"
        if t in ("p", "pitch", "rig-pitch"):
            return "P"
    return None


RBLINK_RE = re.compile(
    r"RBLINK v1\s+ml=(\w+)\s+up=(\d+)\s+seq=(\d+)\s+fl=0x([0-9a-fA-F]+)\[([^\]]*)\]\s+"
    r"duty=\[(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\]\s+"
    r"fr=(\d+)\s+gap=(-?\d+)\s+lost=(\d+)\s+stale=(\d+)\s+"
    r"crc=(\d+)\s+noise=(\d+)\s+cuts=(\d+)\s+pwmerr=(\d+)\s+"
    r"kill=(\d+)\s+txt=(\d+)\s+cl=(-?\d+)"
)


def parse_rblink(line):
    """The ESP's health line, same regex as tools/bench/riskybird-dash.py."""
    m = RBLINK_RE.search(line or "")
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


def st_flag_names(st):
    if st is None:
        return []
    out = []
    if st & FLAG_ARMED:
        out.append("ARMED")
    if st & FLAG_ESTOP:
        out.append("ESTOP")
    if st & FLAG_ARMING:
        out.append("ARMING")
    if st & FLAG_CALDONE:
        out.append("CAL")
    return out


# =============================================================================
# Journal / batch files (pure-ish)
# =============================================================================

def parse_journal_text(text):
    """-> (events, n_bad). A last line without a newline that does not parse is a write in
    progress (not counted as bad); one that does parse is kept."""
    events, bad = [], 0
    lines = text.split("\n")
    for i, line in enumerate(lines):
        if not line.strip():
            continue
        last = i == len(lines) - 1
        try:
            obj = json.loads(line)
        except ValueError:
            if not last:
                bad += 1
            continue
        if isinstance(obj, dict):
            events.append(obj)
        elif not last:
            bad += 1
    return events, bad


def load_journal(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return parse_journal_text(f.read())
    except FileNotFoundError:
        return [], 0


class JsonlFollower:
    """Incremental reader of an append-only JSONL file; holds back an incomplete last line."""

    def __init__(self, path):
        self.path = Path(path)
        self.pos = 0
        self.buf = b""
        self.bad = 0
        self.ino = None

    def poll(self, max_bytes=1 << 20):
        try:
            st = os.stat(self.path)
        except OSError:
            return []
        if self.ino is not None and (st.st_ino != self.ino or st.st_size < self.pos):
            self.pos, self.buf = 0, b""          # replaced or truncated: start over
        self.ino = st.st_ino
        if st.st_size <= self.pos:
            return []
        with open(self.path, "rb") as f:
            f.seek(self.pos)
            data = f.read(max_bytes)
        self.pos += len(data)
        self.buf += data
        *complete, self.buf = self.buf.split(b"\n")
        out = []
        for raw in complete:
            if not raw.strip():
                continue
            try:
                obj = json.loads(raw.decode("utf-8", "replace"))
            except ValueError:
                self.bad += 1
                continue
            if isinstance(obj, dict):
                out.append(obj)
            else:
                self.bad += 1
        return out


def read_json(path, default=None):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def write_json_atomic(path, obj):
    """Write obj to path so a concurrent reader sees the old or the new file, never a mix."""
    path = Path(path)
    fd, tmp = tempfile.mkstemp(prefix="." + path.name + ".", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(obj, f, sort_keys=True)
            f.write("\n")
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, str(path))
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def read_control(batch_dir):
    c = read_json(Path(batch_dir) / "control.json", {}) or {}
    if not isinstance(c, dict):
        c = {}
    return {"pause": bool(c.get("pause", False)), "abort": bool(c.get("abort", False)),
            "seq": int(c.get("seq", 0) or 0)}


def update_control(batch_dir, pause=None, abort=None, own_seq=0):
    """Read-modify-write control.json. Only the fields given change. seq = max(file, own) + 1."""
    cur = read_control(batch_dir)
    if pause is not None:
        cur["pause"] = bool(pause)
    if abort is not None:
        cur["abort"] = bool(abort)
    cur["seq"] = max(cur["seq"], int(own_seq)) + 1
    write_json_atomic(Path(batch_dir) / "control.json", cur)
    return cur


def append_journal(path, obj):
    """Append one event as a single O_APPEND write. If the file ends mid-line (a writer died),
    start on a fresh line so neither record is corrupted further."""
    line = json.dumps(obj, separators=(",", ":"), sort_keys=True) + "\n"
    fd = os.open(str(path), os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
    try:
        try:
            size = os.fstat(fd).st_size
            if size > 0:
                with open(str(path), "rb") as f:
                    f.seek(size - 1)
                    if f.read(1) != b"\n":
                        line = "\n" + line
        except OSError:
            pass
        os.write(fd, line.encode("utf-8"))
    finally:
        os.close(fd)


def newest_batch_dir(rig_dir=RIG_DIR):
    try:
        dirs = [p for p in Path(rig_dir).iterdir() if p.is_dir()]
    except OSError:
        return None
    if not dirs:
        return None
    return max(dirs, key=lambda p: (p.name[:15], p.stat().st_mtime))


def newest_log(log_dir=LOG_DIR, pattern="raw_*.log"):
    try:
        files = list(Path(log_dir).glob(pattern))
    except OSError:
        return None
    best, key = None, None
    for p in files:
        try:
            k = (p.stat().st_mtime, p.name)
        except OSError:
            continue
        if key is None or k > key:
            best, key = p, k
    return best


# =============================================================================
# Batch model (pure functions + a small accumulator)
# =============================================================================

def compute_eta(durations, n_total, n_done, current_elapsed=0.0):
    """Seconds left from the mean completed-trial time, or None if nothing to average yet."""
    durs = [d for d in durations if d is not None and d > 0]
    if not durs or not n_total:
        return None
    left = max(0, int(n_total) - int(n_done))
    if left == 0:
        return 0.0
    avg = sum(durs) / len(durs)
    return max(0.0, avg * left - max(0.0, current_elapsed))


def best_trial(rows, lower_is_better=SCORE_LOWER_IS_BETTER):
    """Trial number of the best score among trials that ended ok (estop/aborted never win)."""
    best, best_score = None, None
    for r in rows:
        if r.get("status") not in ("ok", None, "running"):
            continue
        if r.get("estop"):
            continue
        m = r.get("metrics") or {}
        s = m.get("score")
        if not isinstance(s, (int, float)) or isinstance(s, bool) or not math.isfinite(s):
            continue
        if best_score is None or (s < best_score if lower_is_better else s > best_score):
            best, best_score = r.get("trial"), s
    return best


def fmt_num(v):
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return str(v)
    if float(v).is_integer() and abs(v) < 1e9:
        return "%d" % int(v)
    return ("%.4g" % v)


def gains_diff(gains, baseline):
    """Compact `NAME=v` list of gains that differ from the baseline; '=base' if none."""
    if not gains:
        return "-"
    out = []
    for k in sorted(gains):
        v = gains[k]
        b = (baseline or {}).get(k)
        if b is None or not _close(v, b):
            out.append("%s=%s" % (k, fmt_num(v)))
    return " ".join(out) if out else "=base"


def _close(a, b):
    try:
        return abs(float(a) - float(b)) <= 1e-9 * max(1.0, abs(float(a)), abs(float(b)))
    except (TypeError, ValueError):
        return a == b


_EXC_ABBR = {"doublet": "dbl", "chirp": "chirp", "step": "step", "sine": "sine", "prbs": "prbs"}


def fmt_exc(exc, long=False):
    if not exc:
        return "-"
    t = str(exc.get("type", "?"))
    s = "%s %s %s" % (_EXC_ABBR.get(t, t) if not long else t, exc.get("target", "?"),
                      fmt_num(exc.get("amp", "?")))
    if long:
        unit = {"att": "mrad", "rate": "mrad/s", "moment": "uN.m"}.get(exc.get("target"), "")
        s += unit and (" " + unit)
        if exc.get("dur_ms") is not None:
            s += " %sms" % fmt_num(exc["dur_ms"])
        if exc.get("f0_mHz"):
            s += " f0=%smHz" % fmt_num(exc["f0_mHz"])
        if exc.get("f1_mHz"):
            s += " f1=%smHz" % fmt_num(exc["f1_mHz"])
    return s


def fmt_dur(sec):
    if sec is None:
        return "--"
    sec = max(0.0, float(sec))
    if sec < 60:
        return "%.0fs" % sec if sec >= 10 else "%.1fs" % sec
    if sec < 3600:
        return "%dm%02ds" % (sec // 60, sec % 60)
    return "%dh%02dm" % (sec // 3600, (sec % 3600) // 60)


def _tkey(n):
    try:
        return int(n)
    except (TypeError, ValueError):
        return n


class BatchState:
    """Everything known about one batch directory, rebuilt from its files."""

    def __init__(self, path=None):
        self.dir = Path(path) if path else None
        self.meta = {}
        self.trials = {}
        self.order = []
        self.events = []
        self.status = {}
        self.status_mtime = None
        self.control = {"pause": False, "abort": False, "seq": 0}
        self.batch_start_t = None
        self.batch_end = None
        self.current = None
        self.journal_bad = 0

    def row(self, n):
        k = _tkey(n)
        if k not in self.trials:
            self.trials[k] = {"trial": k, "gains": {}, "exc": {}, "status": None,
                              "metrics": None, "fetch": None, "estop": False,
                              "start_t": None, "end_t": None}
            self.order.append(k)
        return self.trials[k]

    def apply(self, ev):
        self.events.append(ev)
        kind = ev.get("ev")
        t = ev.get("t")
        if kind == "batch_start":
            self.batch_start_t = t
        elif kind == "trial_start" and "trial" in ev:
            r = self.row(ev["trial"])
            r["gains"] = dict(ev.get("gains") or {})
            r["exc"] = dict(ev.get("exc") or {})
            r["start_t"] = t
            r["status"] = "running"
            self.current = r["trial"]
        elif kind == "trial_end" and "trial" in ev:
            r = self.row(ev["trial"])
            r["status"] = ev.get("status") or "ok"
            r["end_t"] = t
            if r["status"] == "estop":
                r["estop"] = True
            if self.current == r["trial"]:
                self.current = None
        elif kind == "metrics" and "trial" in ev:
            self.row(ev["trial"])["metrics"] = ev
        elif kind == "fetch" and "trial" in ev:
            self.row(ev["trial"])["fetch"] = ev
        elif kind == "estop":
            if self.current is not None:
                self.trials[self.current]["estop"] = True
        elif kind == "batch_end":
            self.batch_end = ev

    def rows(self):
        return [self.trials[k] for k in sorted(self.order, key=lambda k: (str(type(k)), k))]

    def spec(self):
        s = self.meta.get("spec") if isinstance(self.meta, dict) else None
        return s if isinstance(s, dict) else {}

    def axis(self):
        """"R"/"P" from batch.json (top-level axis, the spec's, or the mode), None if it says
        nothing (older batches: roll)."""
        m = self.meta if isinstance(self.meta, dict) else {}
        for v in (m.get("axis"), self.spec().get("axis"), m.get("mode")):
            a = axis_letter(v)
            if a:
                return a
        return None

    def baseline(self):
        grid = self.spec().get("grid") or []
        if grid and isinstance(grid[0], dict) and isinstance(grid[0].get("gains"), dict):
            return grid[0]["gains"]
        for r in self.rows():
            if r["gains"]:
                return r["gains"]
        return {}

    def n_planned(self):
        st = self.status.get("n_trials") if isinstance(self.status, dict) else None
        if isinstance(st, int) and st > 0:
            return st
        s = self.spec()
        grid = len(s.get("grid") or []) or 1
        exc = len(s.get("excitations") or []) or 1
        rep = s.get("repeats") or 1
        n = grid * exc * (rep if isinstance(rep, int) and rep > 0 else 1)
        mx = (s.get("limits") or {}).get("max_trials")
        if isinstance(mx, int) and mx > 0:
            n = min(n, mx)
        return n if self.spec() else len(self.trials)

    def durations(self):
        return [r["end_t"] - r["start_t"] for r in self.rows()
                if isinstance(r["start_t"], (int, float)) and isinstance(r["end_t"], (int, float))]

    def n_done(self):
        return sum(1 for r in self.rows() if r["status"] not in (None, "running"))


# =============================================================================
# rec.csv (trial recordings)
# =============================================================================

def load_rec_csv(path, max_rows=200000):
    """Tolerant loader: column names are not fixed by the contract.

    time: t_s | time_s | t (s) | t_us (us); controlled angle: ang_deg | roll_deg | roll (rad) |
    roll_rad; setpoint: ang_tgt_deg | sp_deg | setpoint_deg | roll_sp_deg | roll_tgt_deg |
    roll_tgt (rad); excitation: exc_val | exc, exc_id. "axis" (R/P) is the axis column's value,
    "R" without one (a CSV from before rig-pitch). The "roll" key holds the controlled axis.
    """
    with open(path, "r", encoding="utf-8", errors="replace", newline="") as f:
        rd = csv.DictReader(f)
        cols = [c.strip() for c in (rd.fieldnames or [])]
        rows = []
        for i, row in enumerate(rd):
            if i >= max_rows:
                break
            rows.append({(k or "").strip(): v for k, v in row.items()})

    def pick(*names):
        for n in names:
            if n in cols:
                return n
        return None

    tcol = pick("t_s", "time_s", "t", "t_us")
    rcol = pick("ang_deg", "roll_deg", "roll", "roll_rad")
    scol = pick("ang_tgt_deg", "sp_deg", "setpoint_deg", "roll_sp_deg", "roll_tgt_deg", "roll_tgt", "sp")
    ecol = pick("exc_val", "exc")
    icol = pick("exc_id")
    if rcol is None:
        raise ValueError("rec.csv has no roll column (have: %s)" % ", ".join(cols[:12]))

    def f(row, col, scale=1.0):
        if col is None:
            return None
        try:
            v = float(row.get(col, ""))
        except (TypeError, ValueError):
            return None
        return v * scale if math.isfinite(v) else None

    rscale = 180.0 / math.pi if rcol in ("roll", "roll_rad") else 1.0
    sscale = 180.0 / math.pi if scol == "roll_tgt" else (0.001 if scol == "sp" else 1.0)
    tscale = 1e-6 if tcol == "t_us" else 1.0
    letters = {(row.get("axis") or "").strip().upper() for row in rows} - {""}
    out = {"t": [], "roll": [], "sp": [], "exc": [], "exc_id": [], "columns": cols,
           "sp_col": scol, "axis": letters.pop() if len(letters) == 1 and letters <= {"R", "P"}
           else "R"}
    for i, row in enumerate(rows):
        t = f(row, tcol, tscale) if tcol else i * 0.01
        out["t"].append(t)
        out["roll"].append(f(row, rcol, rscale))
        out["sp"].append(f(row, scol, sscale))
        out["exc"].append(f(row, ecol))
        out["exc_id"].append(f(row, icol))
    ts = [t for t in out["t"] if t is not None]
    t0 = ts[0] if ts else 0.0
    out["t"] = [None if t is None else t - t0 for t in out["t"]]
    return out


def exc_marks(t, exc, exc_id):
    """x positions where an excitation starts or stops (exc_id change, or exc 0 <-> nonzero)."""
    marks = []
    prev_id, prev_on = None, False
    for i, ti in enumerate(t):
        if ti is None:
            continue
        eid = exc_id[i] if i < len(exc_id) else None
        on = bool(exc[i]) if i < len(exc) and exc[i] is not None else False
        if eid is not None and prev_id is not None and eid != prev_id:
            marks.append(ti)
        elif on != prev_on and (eid is None or prev_id is None or eid == prev_id):
            marks.append(ti)
        prev_id = eid if eid is not None else prev_id
        prev_on = on
    dedup = []
    for m in marks:
        if not dedup or m - dedup[-1] > 1e-6:
            dedup.append(m)
    return dedup


# =============================================================================
# Plot rasteriser (pure)
# =============================================================================

_BRAILLE_BITS = ((0x01, 0x08), (0x02, 0x10), (0x04, 0x20), (0x40, 0x80))


def rasterise(series, width, height, x_range, y_range, mode="braille", hlines=(), vmarks=()):
    """Draw line series into a width x height character grid.

    series: [(points [(x, y)], style, ascii_char)], later series win the colour of a cell.
    mode: braille (2x4 dots per cell), block (half blocks, 1x2), ascii (1x1, series char).
    hlines: [(y, style)] dim gridlines; vmarks: [(x, style)] dim vertical markers.
    Returns (rows, styles): `height` strings of exactly `width` chars, and a style grid.
    """
    w, h = max(0, int(width)), max(0, int(height))
    chars = [[" "] * w for _ in range(h)]
    styles = [[None] * w for _ in range(h)]
    if w == 0 or h == 0:
        return ["".join(r) for r in chars], styles
    sx, sy = {"braille": (2, 4), "block": (1, 2)}.get(mode, (1, 1))
    pw, ph = w * sx, h * sy
    x0, x1 = float(x_range[0]), float(x_range[1])
    y0, y1 = float(y_range[0]), float(y_range[1])
    if x1 <= x0:
        x1 = x0 + 1.0
    if y1 <= y0:
        y1 = y0 + 1.0
    unicode_ok = mode in ("braille", "block")

    def fx(x):
        return (x - x0) / (x1 - x0) * (pw - 1)

    def fy(y):
        return (y1 - y) / (y1 - y0) * (ph - 1)

    for yv, st in hlines:
        if y0 <= yv <= y1:
            cy = int(round(fy(yv))) // sy
            for cx in range(w):
                chars[cy][cx] = "┈" if unicode_ok else "-"
                styles[cy][cx] = st
    for xv, st in vmarks:
        if x0 <= xv <= x1:
            cx = int(round(fx(xv))) // sx
            for cy in range(h):
                chars[cy][cx] = "┊" if unicode_ok else ":"
                styles[cy][cx] = st

    bits = {}
    owner = {}
    for pts, st, ach in series:
        prev = None
        for p in pts:
            x, y = p[0], p[1]
            if (x is None or y is None or not math.isfinite(x) or not math.isfinite(y)
                    or x < x0 or x > x1):
                prev = None
                continue
            X = fx(x)
            Y = min(max(fy(y), 0.0), ph - 1.0)
            if prev is None:
                steps = [(X, Y)]
            else:
                n = int(max(abs(X - prev[0]), abs(Y - prev[1]))) + 1
                steps = [(prev[0] + (X - prev[0]) * k / n, prev[1] + (Y - prev[1]) * k / n)
                         for k in range(1, n + 1)]
            for (xx, yy) in steps:
                ix = min(max(int(round(xx)), 0), pw - 1)
                iy = min(max(int(round(yy)), 0), ph - 1)
                cx, cy = ix // sx, iy // sy
                key = (cy, cx)
                if mode == "braille":
                    bits[key] = bits.get(key, 0) | _BRAILLE_BITS[iy % 4][ix % 2]
                elif mode == "block":
                    bits[key] = bits.get(key, 0) | (1 if iy % 2 == 0 else 2)
                else:
                    bits[key] = ach
                owner[key] = st
            prev = (X, Y)
    for (cy, cx), b in bits.items():
        if mode == "braille":
            chars[cy][cx] = chr(0x2800 + b)
        elif mode == "block":
            chars[cy][cx] = {1: "▀", 2: "▄", 3: "█"}[b]
        else:
            chars[cy][cx] = b
        styles[cy][cx] = owner[(cy, cx)]
    return ["".join(r) for r in chars], styles


def nice_range(values, min_span=6.0, limit=40.0):
    vals = [v for v in values if v is not None and math.isfinite(v)]
    if not vals:
        return -min_span / 2, min_span / 2
    lo, hi = min(vals), max(vals)
    mid = (lo + hi) / 2
    span = max(hi - lo, min_span) * 1.15
    lo, hi = mid - span / 2, mid + span / 2
    lo, hi = max(lo, -limit), min(hi, limit)
    step = nice_step(hi - lo, 4)
    return math.floor(lo / step) * step, math.ceil(hi / step) * step


def nice_step(span, n):
    raw = span / max(1, n)
    for s in (0.5, 1, 2, 2.5, 5, 10, 15, 20, 25, 50, 100, 200, 500, 1000):
        if s >= raw:
            return s
    return raw


# =============================================================================
# Glyphs and styles
# =============================================================================

GLYPHS_UNICODE = {
    "tl": "╭", "tr": "╮", "bl": "╰", "br": "╯", "h": "─", "v": "│",
    "star": "★", "eighths": " ▏▎▍▌▋▊▉█",
    "veighths": " ▁▂▃▄▅▆▇█", "dot": "●",
    "deg": "°", "sep": " │ ", "arrow": "→", "plot": "braille", "lcap": "▕",
    "rcap": "▏", "tick": "┤", "mid": "·", "up": "↑↓", "enter": "⏎",
    "cross": "✖", "delta": "Δ", "dash": "─", "ellipsis": "…",
}
GLYPHS_ASCII = {
    "tl": "+", "tr": "+", "bl": "+", "br": "+", "h": "-", "v": "|",
    "star": "*", "eighths": " ........#", "veighths": " ..::||##", "dot": "o",
    "deg": "d", "sep": " | ", "arrow": "->", "plot": "ascii", "lcap": "[", "rcap": "]",
    "tick": "+", "mid": ".", "up": "up/dn", "enter": "Enter", "cross": "x", "delta": "d",
    "dash": "-", "ellipsis": "~",
}

# style -> (fg256, bg256, fg8, bg8, attrs). attrs: b bold, d dim, r reverse, u underline.
STYLE_TABLE = {
    "text": (252, None, "white", None, ""),
    "dim": (242, None, "white", None, "d"),
    "label": (246, None, "white", None, ""),
    "value": (255, None, "white", None, "b"),
    "title": (153, None, "cyan", None, "b"),
    "border": (239, None, "white", None, "d"),
    "key": (117, None, "cyan", None, "b"),
    "ok": (114, None, "green", None, ""),
    "ok_b": (114, None, "green", None, "b"),
    "warn": (214, None, "yellow", None, ""),
    "warn_b": (214, None, "yellow", None, "b"),
    "bad": (203, None, "red", None, ""),
    "bad_b": (203, None, "red", None, "b"),
    "estop_hint": (196, None, "red", None, "b"),
    "roll": (213, None, "magenta", None, "b"),
    "sp": (51, None, "cyan", None, ""),
    "tgt": (75, None, "blue", None, "b"),
    "exc": (141, None, "magenta", None, ""),
    "best": (114, None, "green", None, "b"),
    "info": (110, None, "blue", None, ""),
    "banner": (231, 124, "white", "red", "b"),
    "banner_alt": (124, 224, "red", "white", "b"),
    "badge_IDLE": (255, 240, "black", "white", "b"),
    "badge_ARMING": (16, 214, "black", "yellow", "b"),
    "badge_RAMP": (16, 80, "black", "cyan", "b"),
    "badge_RUN": (16, 114, "black", "green", "b"),
    "badge_DONE": (231, 68, "white", "blue", "b"),
    "badge_ESTOP": (231, 160, "white", "red", "b"),
    "badge_NONE": (250, 236, "white", "black", ""),
    # the controlled axis: distinct, so a pitch rig is never mistaken for the roll one
    "badge_AXIS_ROLL": (16, 213, "black", "magenta", "b"),
    "badge_AXIS_PITCH": (231, 99, "white", "magenta", "b"),   # violet: no run-state badge uses it
}
SEL_BG = (237, "blue")
MONO_ATTRS = {"dim": "d", "value": "b", "title": "b", "key": "b", "ok_b": "b", "warn_b": "b",
              "bad": "b", "bad_b": "b", "estop_hint": "b", "roll": "b", "best": "b",
              "banner": "rb", "banner_alt": "b", "border": "d"}


def axis_badge_style(ax):
    return "badge_AXIS_PITCH" if ax == "P" else "badge_AXIS_ROLL"


def badge_style(rs):
    if rs in ("RAMP_UP", "RAMP_DOWN"):
        return "badge_RAMP"
    if rs in ("IDLE", "ARMING", "RUN", "DONE", "ESTOP"):
        return "badge_" + rs
    return "badge_NONE"


def level_style(v, bands):
    if v is None:
        return "dim"
    lo, hi = bands
    return "ok" if v < lo else ("warn" if v < hi else "bad")


def duty_style(frac):
    return "ok" if frac < 0.70 else ("warn" if frac < 0.90 else "bad")


def xterm_hex(n):
    if n is None:
        return None
    base = ["000000", "800000", "008000", "808000", "000080", "800080", "008080", "c0c0c0",
            "808080", "ff0000", "00ff00", "ffff00", "0000ff", "ff00ff", "00ffff", "ffffff"]
    if n < 16:
        return "#" + base[n]
    if n < 232:
        n -= 16
        lv = [0, 95, 135, 175, 215, 255]
        return "#%02x%02x%02x" % (lv[n // 36], lv[(n // 6) % 6], lv[n % 6])
    g = 8 + (n - 232) * 10
    return "#%02x%02x%02x" % (g, g, g)


# =============================================================================
# Canvas: a style-tagged character grid, painted to curses, text, ANSI or HTML
# =============================================================================

class Canvas:
    def __init__(self, w, h, ascii_only=False):
        self.ascii_only = ascii_only
        self.w, self.h = max(0, int(w)), max(0, int(h))
        self.ch = [[" "] * self.w for _ in range(self.h)]
        self.st = [[None] * self.w for _ in range(self.h)]
        self.hl = [[False] * self.w for _ in range(self.h)]

    def put(self, y, x, text, style=None, limit=None):
        """Write text clipped to the canvas (and to x < limit). Returns the next x."""
        lim = self.w if limit is None else min(self.w, limit)
        if 0 <= y < self.h:
            row, srow = self.ch[y], self.st[y]
            for i, c in enumerate(text):
                xx = x + i
                if xx >= lim:
                    break
                if xx >= 0:
                    o = ord(c)
                    # data from the wire must never reach curses as a control character
                    if o < 32 or o == 127 or (self.ascii_only and o > 127) or 0x80 <= o < 0xa0:
                        c = "?"
                    row[xx] = c
                    srow[xx] = style
        return x + len(text)

    def segs(self, y, x, segs, limit=None):
        for text, style in segs:
            x = self.put(y, x, text, style, limit)
        return x

    def put_r(self, y, x_end, text, style=None):
        return self.put(y, x_end - len(text), text, style)

    def fill(self, y, x, w, ch=" ", style=None):
        self.put(y, x, ch * max(0, w), style)

    def restyle(self, y, x0, x1, style):
        if 0 <= y < self.h:
            for xx in range(max(0, x0), min(self.w, x1)):
                self.st[y][xx] = style

    def highlight(self, y, x0, x1):
        if 0 <= y < self.h:
            for xx in range(max(0, x0), min(self.w, x1)):
                self.hl[y][xx] = True

    def box(self, y, x, h, w, g, title=(), right=(), style="border"):
        if h < 2 or w < 2:
            return
        self.put(y, x, g["tl"] + g["h"] * (w - 2) + g["tr"], style)
        for yy in range(y + 1, y + h - 1):
            self.put(yy, x, g["v"], style)
            self.put(yy, x + w - 1, g["v"], style)
        self.put(y + h - 1, x, g["bl"] + g["h"] * (w - 2) + g["br"], style)
        lim = x + w - 2
        if title:
            xx = self.put(y, x + 1, " ", style, lim)
            xx = self.segs(y, xx, title, lim)
            self.put(y, xx, " ", style, lim)
        if right:
            total = sum(len(t) for t, _ in right) + 2
            xr = x + w - 2 - total
            if xr > x + 4:
                xx = self.put(y, xr, " ", style)
                xx = self.segs(y, xx, right, x + w - 1)
                self.put(y, xx, " ", style, x + w - 1)

    def to_text(self):
        return "\n".join("".join(r) for r in self.ch)

    def runs(self, y):
        row, srow, hrow = self.ch[y], self.st[y], self.hl[y]
        x = 0
        while x < self.w:
            s, hh = srow[x], hrow[x]
            x2 = x + 1
            while x2 < self.w and srow[x2] == s and hrow[x2] == hh:
                x2 += 1
            yield x, "".join(row[x:x2]), s, hh
            x = x2

    def to_ansi(self):
        out = []
        for y in range(self.h):
            line = []
            for _, text, s, hh in self.runs(y):
                fg, bg, _, _, attrs = STYLE_TABLE.get(s, STYLE_TABLE["text"])
                if hh and bg is None:
                    bg = SEL_BG[0]
                codes = ["0"]
                if "b" in attrs:
                    codes.append("1")
                if "d" in attrs:
                    codes.append("2")
                if fg is not None:
                    codes.append("38;5;%d" % fg)
                if bg is not None:
                    codes.append("48;5;%d" % bg)
                line.append("\x1b[%sm%s" % (";".join(codes), text))
            out.append("".join(line) + "\x1b[0m")
        return "\n".join(out)

    def to_html(self, title="rig tui"):
        import html as _html
        bgc = "#16161e"
        rows = []
        for y in range(self.h):
            parts = []
            for _, text, s, hh in self.runs(y):
                fg, bg, _, _, attrs = STYLE_TABLE.get(s, STYLE_TABLE["text"])
                if hh and bg is None:
                    bg = SEL_BG[0]
                css = ["color:%s" % xterm_hex(fg)] if fg is not None else []
                if bg is not None:
                    css.append("background:%s" % xterm_hex(bg))
                if "b" in attrs:
                    css.append("font-weight:bold")
                if "d" in attrs:
                    css.append("opacity:.75")
                parts.append('<span style="%s">%s</span>' % (";".join(css), _html.escape(text)))
            rows.append("".join(parts))
        return ("<!DOCTYPE html>\n<html><head><meta charset=\"utf-8\"><title>%s</title>"
                "<style>body{background:%s;margin:24px}pre{font:14px/1.2 'DejaVu Sans Mono',"
                "Menlo,Consolas,monospace;color:#d0d0d0;background:%s;padding:12px;"
                "display:inline-block;border-radius:8px}</style></head><body><pre>%s</pre>"
                "</body></html>\n" % (_html.escape(title), bgc, bgc, "\n".join(rows)))


# =============================================================================
# Live model + app state
# =============================================================================

class LiveModel:
    def __init__(self):
        self.samples = deque(maxlen=4000)   # (ts, ctl_deg, sp_deg, tgt_deg, exc, ex): ctl = ax's angle
        self.rig = None
        self.rig_ts = None
        self.rblink = None
        self.rblink_ts = None
        self.gains = {}
        self.gains_gs = None
        self.gains_ts = None
        self.last_ts = None
        self.lines = 0
        self.bad = 0

    def ingest(self, raw_line, events):
        ts, text = split_log_line(raw_line, self.last_ts)
        if ts is not None:
            self.last_ts = ts
        if ts is None:
            ts = time.time()
        self.lines += 1
        if "RBLINK" in text:
            rb = parse_rblink(text)
            if rb:
                self.rblink, self.rblink_ts = rb, ts
            return
        if "RIG" not in text and "FCACK" not in text and "FCNAK" not in text:
            return
        d = parse_rig_line(text)
        if d is None:
            if re.search(r"\bRIG it=", text):
                self.bad += 1
            return
        k = d["kind"]
        if k == "rig":
            if self.rig is not None and self.rig.get("ax") != d["ax"]:
                self.samples.clear()          # another build: never plot roll and pitch as one trace
            self.rig, self.rig_ts = d, ts
            ang = ctl_angle_mdeg(d)
            self.samples.append((ts, None if ang is None else ang / 1000.0,
                                 None if d["sp"] is None else d["sp"] / 1000.0,
                                 None if d["tgt"] is None else d["tgt"] / 1000.0,
                                 d["exc"], d["ex"]))
        elif k == "gains":
            self.gains = dict(d["gains"])
            self.gains_gs, self.gains_ts = d["gs"], ts
        elif k == "gain":
            self.gains[d["name"]] = d["value"]
            self.gains_ts = ts
            events.append((ts, "live", "RIGGAIN %s=%s" % (d["name"], fmt_num(d["value"])), "info"))
        elif k == "event":
            low = d["text"].lower()
            st = "bad_b" if "estop" in low else ("warn" if "timeout" in low else "info")
            events.append((ts, "live", "RIG EVENT " + d["text"], st))
        elif k == "ack":
            events.append((ts, "live", "FCACK " + d["text"], "ok"))
        elif k == "nak":
            events.append((ts, "live", "FCNAK " + d["text"], "bad_b"))


class UdpCommander:
    """Pre-created, non-blocking UDP socket to the drone's command port."""

    def __init__(self, host, port):
        self.host, self.port = host, int(port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    def send(self, text, repeats=1):
        data = text.encode("ascii", "ignore")
        sent, errors = 0, []
        for _ in range(repeats):
            try:
                self.sock.sendto(data, (self.host, self.port))
                sent += 1
            except OSError as exc:       # ENETUNREACH off the SoftAP: keep trying the rest
                errors.append(str(exc))
        return sent, errors

    def estop(self):
        return self.send("ESTOP\n", ESTOP_REPEATS)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def do_estop(commander, batch_dir=None, writable=True, reason="operator key", own_seq=0,
             now=None):
    """ESTOP: UDP first, then the journal, then control.json. Never raises."""
    res = {"sent": 0, "errors": [], "journal": None, "control": None, "seq": own_seq}
    try:
        res["sent"], res["errors"] = commander.estop()
    except Exception as exc:  # noqa: BLE001 - the key must never die
        res["errors"].append(repr(exc))
    if batch_dir is not None and writable:
        try:
            append_journal(Path(batch_dir) / "journal.jsonl",
                           {"t": round(now or time.time(), 3), "ev": "estop", "reason": reason,
                            "source": "tui"})
            res["journal"] = True
        except Exception as exc:  # noqa: BLE001
            res["journal"] = repr(exc)
        try:
            c = update_control(batch_dir, abort=True, own_seq=own_seq)
            res["control"], res["seq"] = True, c["seq"]
        except Exception as exc:  # noqa: BLE001
            res["control"] = repr(exc)
    return res


class App:
    def __init__(self, drone=DEFAULT_DRONE, port=CMD_PORT, batch_dir=None, readonly=False,
                 selftest=None, log_desc=None):
        self.lock = threading.RLock()
        self.live = LiveModel()
        self.batch = BatchState(batch_dir)
        self.readonly = readonly
        self.selftest = selftest
        self.commander = UdpCommander(drone, port)
        self.events = deque(maxlen=400)       # (ts, source, text, style)
        self.notices = deque(maxlen=50)       # appended from the key path without the lock
        self.estops = []
        self.ctl_seq = 0
        self.sel = None
        self.view = "main"
        self.log_desc = log_desc
        self.log_path = None
        self.errors = deque(maxlen=5)
        self.rec_cache = {}

    # -- ingest (ingest thread) --
    def ingest_log(self, lines):
        ev = []
        for ln in lines:
            try:
                self.live.ingest(ln, ev)
            except Exception as exc:  # noqa: BLE001
                self.errors.append("log: %r" % exc)
        for e in ev:
            self.events.append(e)

    def attach_batch(self, path):
        self.batch = BatchState(path)

    def ingest_journal(self, evs):
        for e in evs:
            try:
                self.batch.apply(e)
                self.events.append(_journal_event_line(e))
            except Exception as exc:  # noqa: BLE001
                self.errors.append("journal: %r" % exc)

    # -- actions (key path; must not wait on the ingest lock) --
    def estop(self, now=None):
        now = now or time.time()
        b = self.batch.dir
        res = do_estop(self.commander, b, writable=not self.readonly, own_seq=self.ctl_seq, now=now)
        self.ctl_seq = max(self.ctl_seq, res["seq"])
        self.estops.append((now, res))
        msg = "ESTOP sent x%d -> %s:%d" % (res["sent"], self.commander.host, self.commander.port)
        if res["errors"]:
            msg += " (errors: %s)" % res["errors"][0]
        if b is not None and self.readonly:
            msg += " (replay: journal/control untouched)"
        elif b is not None:
            msg += " journal=%s control=%s" % ("ok" if res["journal"] is True else "FAIL",
                                              "ok" if res["control"] is True else "FAIL")
        self.notices.append((now, "tui", msg, "bad_b"))
        return res

    def stop(self, now=None):
        sent, errs = self.commander.send("STOP\n")
        self.notices.append((now or time.time(), "tui",
                             "STOP sent -> %s:%d%s" % (self.commander.host, self.commander.port,
                                                     " (%s)" % errs[0] if errs else ""),
                             "warn_b" if sent else "bad_b"))

    def toggle_pause(self, now=None):
        now = now or time.time()
        b = self.batch.dir
        if b is None or self.readonly:
            self.notices.append((now, "tui", "pause: no live batch attached", "warn"))
            return None
        try:
            cur = read_control(b)
            c = update_control(b, pause=not cur["pause"], own_seq=self.ctl_seq)
            self.ctl_seq = c["seq"]
            self.notices.append((now, "tui", "control.json pause=%s seq=%d" %
                                 (str(c["pause"]).lower(), c["seq"]), "warn_b"))
            return c
        except Exception as exc:  # noqa: BLE001
            self.notices.append((now, "tui", "pause failed: %r" % exc, "bad_b"))
            return None

    def drain_notices(self):
        while self.notices:
            self.events.append(self.notices.popleft())

    def move_sel(self, delta):
        rows = self.batch.rows()
        if not rows:
            return
        keys = [r["trial"] for r in rows]
        if self.sel not in keys:
            i = len(keys) - 1
        else:
            i = min(max(keys.index(self.sel) + delta, 0), len(keys) - 1)
        self.sel = keys[i]

    def axis(self):
        """(letter, known): the live RIG line's ax= wins, then the batch (replay), else roll."""
        if self.live.rig is not None:
            return self.live.rig.get("ax") or "R", True
        a = self.batch.axis()
        return (a, True) if a else ("R", False)

    def rec_for(self, trial):
        if self.batch.dir is None:
            return None, "no batch directory attached"
        p = self.batch.dir / "trials" / str(trial) / "rec.csv"
        try:
            mt = p.stat().st_mtime
        except OSError:
            return None, "trials/%s/rec.csv not fetched yet" % trial
        key = (str(p), mt)
        if key not in self.rec_cache:
            try:
                self.rec_cache = {key: (load_rec_csv(p), None)}
            except Exception as exc:  # noqa: BLE001
                self.rec_cache = {key: (None, "rec.csv unreadable: %s" % exc)}
        return self.rec_cache[key]


def _journal_event_line(e):
    t = e.get("t")
    kind = e.get("ev", "?")
    n = e.get("trial")
    style = "text"
    if kind == "batch_start":
        text, style = "batch start", "title"
    elif kind == "trial_start":
        text = "trial %s start %s %s" % (n, gains_diff(e.get("gains") or {}, {}), fmt_exc(e.get("exc")))
        style = "info"
    elif kind == "cmd":
        text = "> %s %s %s" % (e.get("text", ""), "->", e.get("reply", ""))
        style = "bad" if "NAK" in str(e.get("reply", "")) else "dim"
    elif kind == "state":
        text, style = "state %s" % e.get("rs"), "dim"
    elif kind == "event":
        text = str(e.get("text", ""))
        style = "bad_b" if "estop" in text.lower() else "info"
    elif kind == "estop":
        text, style = "ESTOP (%s) %s" % (e.get("source", "?"), e.get("reason", "")), "bad_b"
    elif kind == "trial_end":
        s = e.get("status")
        text = "trial %s end %s" % (n, s)
        style = "ok" if s == "ok" else "bad_b"
    elif kind == "fetch":
        text = "trial %s fetched %s samples, %s gaps" % (n, e.get("samples"), e.get("gaps"))
        style = "warn" if e.get("gaps") else "dim"
    elif kind == "metrics":
        text = "trial %s OS %s%% rms %s score %s" % (
            n, _f(e.get("overshoot_pct"), 1), _f(e.get("rms_err_deg"), 2), _f(e.get("score"), 3))
        style = "text"
    elif kind == "note":
        text, style = "note: %s" % e.get("text", ""), "warn"
    elif kind == "batch_end":
        s = e.get("status")
        text = "batch end %s" % s
        style = "ok_b" if s in ("ok", "done", "complete") else "bad_b"
    else:
        text = json.dumps({k: v for k, v in e.items() if k != "t"})[:120]
    return (t if isinstance(t, (int, float)) else None, "jrnl", text, style)


def _f(v, nd):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        return "-"
    return "%.*f" % (nd, v)


# =============================================================================
# Rendering (pure: App snapshot -> Canvas)
# =============================================================================

def render_frame(app, w, h, now, g):
    c = Canvas(w, h, ascii_only=g["plot"] == "ascii")
    if c.w == 0 or c.h == 0:
        return c
    with app.lock:
        if app.view == "trial" and app.sel is not None and h >= 8 and w >= 30:
            _render_header(c, app, now, g, 1 if h < 20 else 2)
            hdr = 1 if h < 20 else 2
            _render_trial_view(c, app, hdr, 0, h - hdr - 1, w, g)
            _render_footer(c, app, h - 1, g, trial_view=True)
            return c
        if h < 6 or w < 30:
            _render_header(c, app, now, g, 1)
            if h >= 2:
                _render_footer(c, app, h - 1, g)
            if h >= 3:
                c.put(1, 0, "enlarge the terminal", "dim")
            return c
        hdr = 2
        _render_header(c, app, now, g, hdr)
        _render_footer(c, app, h - 1, g)
        top = hdr
        avail = h - hdr - 1
        if w >= 100 and h >= 30:
            top_h = max(10, min(18, int(avail * 0.46)))
            lw = int(w * 0.6)
            rw = w - lw
            _render_plot(c, app, top, 0, top_h, lw, g)
            batch_h = 7
            gains_h = top_h - batch_h
            _render_gains(c, app, top, lw, gains_h, rw, g)
            _render_batch(c, app, now, top + gains_h, lw, batch_h, rw, g)
            bot = top + top_h
            bh = h - 1 - bot
            rlw = max(lw, min(w - 36, 80))       # wide enough for every results column
            _render_results(c, app, bot, 0, bh, rlw, g)
            _render_events(c, app, bot, rlw, bh, w - rlw, g)
            return c
        side = w >= 72
        specs = [("plot", 7, 3), ("gb" if side else "batch", 6, 0), ("results", 4, 3)]
        if not side:
            specs.append(("gains", 4, 0))
        specs.append(("events", 5, 1))
        alloc = stack_alloc(avail, specs)
        y = top
        for name, hh in alloc:
            if name == "plot":
                _render_plot(c, app, y, 0, hh, w, g)
            elif name == "gb":
                bw = w // 2
                _render_batch(c, app, now, y, 0, hh, bw, g)
                _render_gains(c, app, y, bw, hh, w - bw, g)
            elif name == "batch":
                _render_batch(c, app, now, y, 0, hh, w, g)
            elif name == "gains":
                _render_gains(c, app, y, 0, hh, w, g)
            elif name == "results":
                _render_results(c, app, y, 0, hh, w, g)
            elif name == "events":
                _render_events(c, app, y, 0, hh, w, g)
            y += hh
    return c


def stack_alloc(avail, specs):
    """specs [(name, min_h, weight)] in priority order -> [(name, h)] in the same order,
    dropping lower-priority panels whose minimum does not fit, then sharing the rest."""
    chosen, used = [], 0
    for name, mn, wt in specs:
        if used + mn <= avail:
            chosen.append([name, mn, wt])
            used += mn
    extra = avail - used
    tw = sum(x[2] for x in chosen)
    if tw and extra > 0:
        give = [extra * x[2] // tw for x in chosen]
        rem = extra - sum(give)
        for i, x in enumerate(chosen):
            if rem <= 0:
                break
            if x[2]:
                give[i] += 1
                rem -= 1
        for i, x in enumerate(chosen):
            x[1] += give[i]
    elif chosen and extra > 0:
        chosen[-1][1] += extra
    return [(x[0], x[1]) for x in chosen]


def _estop_hint(w):
    if w >= 60:
        return " [ ESTOP: E / SPACE ] "
    if w >= 14:
        return " ESTOP:E/SPC "
    return "E=ESTOP"[:max(0, w)]


def _estop_msg(app, now):
    """(text, failed) for the last ESTOP: the banner must never read like success when it failed."""
    t, res = app.estops[-1]
    stamp = time.strftime("%H:%M:%S", time.localtime(t))
    if res["sent"] < ESTOP_REPEATS:
        err = (res["errors"] or ["?"])[0]
        return "ESTOP SEND FAILED %d/%d %s: %s" % (res["sent"], ESTOP_REPEATS, stamp, err), True
    return "ESTOP SENT x%d -> %s:%d %s" % (res["sent"], app.commander.host, app.commander.port,
                                           stamp), False


def _render_header(c, app, now, g, rows):
    w = c.w
    live = app.live
    last = app.estops[-1][0] if app.estops else None
    flashing = last is not None and 0 <= now - last < FLASH_S
    hint = _estop_hint(w)
    if flashing:
        msg, _ = _estop_msg(app, now)
        if w >= len(msg) + 30:
            hint = " %s " % msg
    hx = w - len(hint)
    _render_header_body(c, app, now, g, rows, hint, hx)
    if flashing:
        # The whole header inverts red, but its content (rs badge, link) stays readable.
        st = "banner" if int((now - last) * 3) % 2 == 0 else "banner_alt"
        for y in range(min(rows, c.h)):
            c.restyle(y, 0, w, st)


def _render_header_body(c, app, now, g, rows, hint, hx):
    w = c.w
    live = app.live
    c.put(0, hx, hint, "estop_hint")
    rig = live.rig
    x = c.put(0, 0, " RIG ", "title", hx)
    rs = rig["rs"] if rig else None
    stale = live.rig_ts is None or now - live.rig_ts > RIG_STALE_S
    x = c.put(0, x, " %s " % (rs or "NO DATA"), badge_style(rs) if rs else "badge_NONE", hx)
    x = c.put(0, x, " ", None, hx)
    ax, ax_known = app.axis()
    x = c.put(0, x, " %s%s " % (AXIS_WORDS[ax].upper(), "" if ax_known else "?"),
              axis_badge_style(ax), hx)
    x = c.put(0, x, " ", None, hx)
    if rig:
        if stale:
            x = c.put(0, x, "stale %s " % fmt_dur(now - live.rig_ts), "warn_b", hx)
        flags = st_flag_names(rig["st"])
        for f in flags:
            fs = {"ARMED": "ok_b", "ESTOP": "bad_b", "ARMING": "warn", "CAL": "dim"}[f]
            x = c.put(0, x, f + " ", fs, hx)
        ang = ctl_angle_mdeg(rig)
        segs = [(g["sep"][1:], "border"), ("%s " % AXIS_WORDS[ax], "label"),
                (("%+6.1f%s" % (ang / 1000.0, g["deg"])) if ang is not None else "   ?", "roll")]
        if rig["sp"] is not None:
            segs += [("  sp ", "label"), ("%+5.1f%s" % (rig["sp"] / 1000.0, g["deg"]), "sp")]
        if rig["tgt"] is not None:
            segs += [("  tgt ", "label"), ("%+5.1f%s" % (rig["tgt"] / 1000.0, g["deg"]), "tgt")]
        locked = rig.get("r") if ax == "P" else rig.get("p")
        if locked is not None and w >= 160:          # wide terminals only: never crowd st/gs
            segs += [("  %s(locked) " % AXIS_WORDS["R" if ax == "P" else "P"], "dim"),
                     ("%+.1f%s" % (locked / 1000.0, g["deg"]), "dim")]
        segs += [(g["sep"], "border")]
        if rig["coll"] is not None:
            segs += [("coll ", "label"), ("%5.1f%%" % (rig["coll"] / 10.0), "value")]
        if rig["vb"] is not None:
            vb = rig["vb"] / 1000.0
            segs += [("  vbat ", "label"), ("%.2fV" % vb, "value")]
        segs += [(g["sep"], "border"), ("st ", "label"), ("%d" % rig["st"], "value")]
        if rig["gs"] is not None:
            segs += [("  gs ", "label"), ("%d" % rig["gs"], "value")]
        x = c.segs(0, x, segs, hx)
    else:
        x = c.put(0, x, "no RIG telemetry yet", "dim", hx)
    if rows < 2 or c.h < 2:
        return
    # row 1: link + budget
    rb = live.rblink
    segs = [(" link ", "label")]
    if rb is None:
        segs.append(("no RBLINK", "warn"))
    else:
        age = now - live.rblink_ts if live.rblink_ts else 99.0
        up = rb["ml"].lower() == "up"
        if age > 3 * RBLINK_STALE_S:
            segs.append(("%s DOWN?" % g["dot"], "bad_b"))
        else:
            segs.append(("%s %s" % (g["dot"], "UP" if up else rb["ml"].upper()),
                         "ok_b" if up else "bad_b"))
        segs += [(" age ", "label"),
                 ("%s" % fmt_dur(age), "ok" if age < RBLINK_STALE_S else
                  ("warn" if age < 3 * RBLINK_STALE_S else "bad_b")),
                 (" gap ", "label"), ("%dms" % rb["gap_ms"],
                                      "ok" if rb["gap_ms"] < 50 else "warn"),
                 (" lost ", "label"), ("%d" % rb["lost"], "ok" if rb["lost"] == 0 else "warn"),
                 (" crc ", "label"), ("%d" % rb["crc"], "ok" if rb["crc"] == 0 else "warn"),
                 (" kill ", "label"), ("%d" % int(rb["kill"]), "bad_b" if rb["kill"] else "ok")]
    st = app.batch.status if isinstance(app.batch.status, dict) else {}
    if st:
        cl = st.get("commit_left_ms")
        if isinstance(cl, (int, float)):
            segs += [(g["sep"], "border"), ("commit ", "label"),
                     (fmt_dur(cl / 1000.0), "ok" if cl > 120000 else
                      ("warn" if cl > 30000 else "bad_b")), (" left", "label")]
        mu, mb = st.get("motor_ms"), st.get("motor_budget_ms")
        if isinstance(mu, (int, float)):
            frac = (mu / mb) if isinstance(mb, (int, float)) and mb > 0 else None
            segs += [(g["sep"], "border"), ("motor ", "label"),
                     (fmt_dur(mu / 1000.0), "value" if frac is None or frac < 0.8 else
                      ("warn_b" if frac < 1.0 else "bad_b"))]
            if frac is not None:
                segs += [("/%s" % fmt_dur(mb / 1000.0), "label")]
    if app.selftest is not None:
        segs += [(g["sep"], "border"),
                 ("SELFTEST :%d rx ESTOP %d" % (
                     app.commander.port, app.selftest.counts.get("ESTOP", 0)), "warn_b")]
    if app.estops:
        segs += [(g["sep"], "border"), (_estop_msg(app, now)[0], "bad_b")]
    c.segs(1, 0, segs)


def _render_footer(c, app, y, g, trial_view=False):
    w = c.w
    # ESTOP and the q warning always win; the other key hints are dropped from the end.
    head = [(" E/space", "estop_hint"), (" ESTOP  ", "dim")]
    if trial_view:
        groups = [[("Esc", "key"), (" back  ", "dim")], [(g["up"], "key"), (" prev/next  ", "dim")]]
    else:
        groups = [[("s", "key"), (" STOP  ", "dim")], [("p", "key"), (" pause  ", "dim")],
                  [(g["up"] + "/jk", "key"), (" select  ", "dim")],
                  [(g["enter"], "key"), (" plot  ", "dim")]]
    tail = [("q", "key"), (" quits the viewer only; the run continues ", "warn")]

    def ln(segs):
        return sum(len(t) for t, _ in segs)

    if ln(head) + ln(tail) > w:
        tail = [("q", "key"), (" quit viewer", "warn")]
    x = c.segs(y, 0, head)
    room = w - ln(tail) - x
    for grp in groups:
        if ln(grp) > room:
            break
        x = c.segs(y, x, grp)
        room -= ln(grp)
    c.segs(y, max(x, w - ln(tail)), tail)


def _render_plot(c, app, y, x, h, w, g):
    live = app.live
    samples = list(live.samples)
    t_end = samples[-1][0] if samples else time.time()
    t0 = t_end - PLOT_WINDOW_S
    win = [s for s in samples if s[0] >= t0]
    axw = AXIS_WORDS[app.axis()[0]]
    legend = [(axw, "roll"), (" sp", "sp"), (" tgt", "tgt"), (" exc", "exc")]
    c.box(y, x, h, w, g, title=[("%s " % axw.capitalize(), "title"),
                                ("last %ds" % PLOT_WINDOW_S, "dim")],
          right=legend)
    ih, iw = h - 2, w - 2
    if ih < 2 or iw < 12:
        return
    bars_w = 14 if iw >= 50 else 0
    axis_w = 7
    pw = iw - axis_w - bars_w - (1 if bars_w else 0)
    ph = ih - 1
    if pw < 4 or ph < 1:
        return
    vals = []
    for s in win:
        vals += [s[1], s[2], s[3]]
    lo, hi = nice_range(vals)
    ex_marks = []
    prev = None
    for s in win:
        on = (s[4] or 0) != 0
        if prev is not None and on != prev:
            ex_marks.append((s[0] - t0, "exc"))
        prev = on
    rows, styles = rasterise(
        [([(s[0] - t0, s[3]) for s in win], "tgt", "."),
         ([(s[0] - t0, s[2]) for s in win], "sp", "-"),
         ([(s[0] - t0, s[1]) for s in win], "roll", "*")],
        pw, ph, (0.0, PLOT_WINDOW_S), (lo, hi), g["plot"],
        hlines=[(0.0, "border")], vmarks=ex_marks)
    px, py = x + 1 + axis_w, y + 1
    for i, row in enumerate(rows):
        for j, ch in enumerate(row):
            c.put(py + i, px + j, ch, styles[i][j] or "dim")
    # y axis
    step = nice_step(hi - lo, max(1, ph // 2))
    v = math.ceil(lo / step) * step
    seen = set()
    while v <= hi + 1e-9:
        r = int(round((hi - v) / (hi - lo) * (ph - 1)))
        if 0 <= r < ph and r not in seen:
            seen.add(r)
            c.put(py + r, x + 1, ("%+5.0f%s" % (v, g["deg"]) if step >= 1 else "%+5.1f" % v)[-6:].rjust(6),
                  "dim")
            c.put(py + r, x + 1 + axis_w - 1, g["tick"], "border")
        v += step
    # x labels
    ly = py + ph
    for frac, lab in ((0.0, "-%ds" % PLOT_WINDOW_S), (0.5, "-%ds" % (PLOT_WINDOW_S / 2)), (1.0, "now")):
        xx = px + int(frac * (pw - 1))
        xx = min(xx, px + pw - len(lab))
        c.put(ly, xx, lab, "dim")
    if not win:
        msg = "waiting for RIG telemetry" + (" (%s)" % app.log_desc if app.log_desc else "")
        c.put(py + ph // 2, px + max(0, (pw - len(msg)) // 2), msg[:pw], "warn")
    if bars_w:
        _render_duty(c, app, py, px + pw + 1, ph + 1, bars_w, g)


def _render_duty(c, app, y, x, h, w, g):
    rig = app.live.rig
    duty = rig["d"] if rig and rig.get("d") else None
    c.put(y, x, " duty%".center(w - 1), "label")
    bh = h - 3
    for i in range(4):
        bx = x + 1 + i * 3
        c.put(y + h - 1, bx, "M%d" % (i + 1), "label")
        if duty is None:
            c.put(y + 1, bx, " -", "dim")
            continue
        frac = max(0.0, min(1.0, duty[i] / 10000.0))
        c.put(y + 1, bx, "%2d" % min(99, int(round(frac * 100))), duty_style(frac) + ("" if frac < 0.9 else ""))
        if bh <= 0:
            continue
        cells = frac * bh
        for k in range(bh):
            cy = y + 2 + (bh - 1 - k)
            level = (k + 0.5) / bh
            fill = cells - k
            if fill >= 1:
                ch = g["veighths"][8]
            elif fill > 0:
                ch = g["veighths"][max(1, int(fill * 8))]
            else:
                ch = g["mid"] if abs(level - DUTY_CEILING) < 0.5 / bh else " "
                c.put(cy, bx, ch * 2, "border")
                continue
            c.put(cy, bx, ch * 2, duty_style(level))


def _render_gains(c, app, y, x, h, w, g):
    live, b = app.live, app.batch
    gains, src, gs = {}, None, None
    if live.gains:
        gains, gs = live.gains, live.gains_gs
        src = "live"
        if live.rig and live.rig.get("gs") is not None:
            gs = live.rig["gs"]
    else:
        cur = b.trials.get(b.current) if b.current is not None else (b.rows()[-1] if b.rows() else None)
        if cur and cur["gains"]:
            gains, src = cur["gains"], "journal trial %s" % cur["trial"]
    right = [("gs ", "label"), ("%s" % ("-" if gs is None else gs), "value")]
    if src:
        right += [("  " + src, "dim")]
    c.box(y, x, h, w, g, title=[("Gains", "title")], right=right)
    ih, iw = h - 2, w - 2
    if ih < 1 or iw < 10:
        return
    if not gains:
        c.put(y + 1, x + 2, "no RIGGAINS yet", "dim", x + w - 1)
        return
    base = b.baseline()
    names = sorted(gains, key=lambda n: (GAIN_ORDER.index(n) if n in GAIN_ORDER else 99, n))
    nw = min(15, max(len(n) for n in names))
    colw = nw + 8
    ncol = max(1, iw // colw)
    pad = 1 if iw - ncol * colw >= 1 else 0
    per = max(1, int(math.ceil(len(names) / float(ncol))))
    for i, n in enumerate(names):
        col, r = divmod(i, per)
        if col >= ncol or r >= ih:
            continue
        cx = x + 1 + pad + col * colw
        v = gains[n]
        diff = base and n in base and not _close(v, base[n])
        c.put(y + 1 + r, cx, ("%-*s" % (nw, n))[:nw], "label")
        c.put(y + 1 + r, cx + nw, "%7.3f" % v if isinstance(v, (int, float)) else "%7s" % v,
              "warn_b" if diff else "value", x + w - 1)


def _progress_bar(frac, width, g):
    frac = max(0.0, min(1.0, frac))
    cells = frac * width
    full = int(cells)
    rem = int((cells - full) * 8)
    s = g["eighths"][8] * full
    if full < width:
        s += g["eighths"][rem] if rem else " "
        s += " " * (width - full - 1)
    return s


def _render_batch(c, app, now, y, x, h, w, g):
    b = app.batch
    st = b.status if isinstance(b.status, dict) else {}
    name = (b.meta or {}).get("name") if isinstance(b.meta, dict) else None
    right = []
    if b.dir is not None:
        right = [(name or b.dir.name, "dim")]
    c.box(y, x, h, w, g, title=[("Batch", "title")], right=right)
    ih, iw = h - 2, w - 2
    if ih < 1 or iw < 10:
        return
    lim = x + w - 1
    if b.dir is None:
        c.put(y + 1, x + 2, "no batch attached (--batch DIR / --latest)", "dim", lim)
        return
    n = b.n_planned() or 0
    done = b.n_done()
    cur = b.current
    trial_i = st.get("trial", cur if cur is not None else done)
    durs = b.durations()
    cur_el = 0.0
    if cur is not None and b.trials[cur]["start_t"]:
        cur_el = max(0.0, now - b.trials[cur]["start_t"])
    eta = compute_eta(durs, n, done, cur_el)
    avg = sum(durs) / len(durs) if durs else None
    frac = 0.0
    if n:
        part = min(cur_el / avg, 0.99) if (avg and cur is not None) else 0.0
        frac = min(1.0, (done + part) / n)
    # state
    ctl = b.control
    if b.batch_end:
        s = b.batch_end.get("status")
        state, sst = ("DONE " + str(s)).upper(), ("ok_b" if s in ("ok", "done", "complete") else "bad_b")
    elif st.get("paused"):
        state, sst = "PAUSED", "warn_b"
    elif ctl.get("abort"):
        state, sst = "ABORT REQUESTED", "bad_b"
    elif ctl.get("pause"):
        state, sst = "PAUSE REQUESTED", "warn_b"
    elif st:
        state, sst = "RUNNING", "ok_b"
    else:
        state, sst = "WAITING", "dim"
    if b.status_mtime and now - b.status_mtime > STATUS_STALE_S and not b.batch_end:
        state, sst = state + " (status %s old)" % fmt_dur(now - b.status_mtime), "warn_b"
    lines = []
    head = "trial %3s/%-3s " % (trial_i if trial_i is not None else "-", n or "?")
    barw = max(4, iw - len(head) - 9)
    lines.append([(head, "value"), (g["lcap"], "border"), (_progress_bar(frac, barw, g), "ok"),
                  (g["rcap"], "border"), ("%4.0f%%" % (frac * 100), "value")])
    lines.append([("ETA ", "label"), ("%-8s" % fmt_dur(eta), "value"), ("avg ", "label"),
                  ("%-11s" % ((fmt_dur(avg) + "/trial") if avg else "--"), "value"), (state, sst)])
    curd = st.get("current") if isinstance(st.get("current"), dict) else {}
    gains = curd.get("gains") or (b.trials[cur]["gains"] if cur is not None else {})
    exc = curd.get("exc") or (b.trials[cur]["exc"] if cur is not None else {})
    lines.append([("gains ", "label"), (gains_diff(gains, b.baseline()) if gains else "-", "text")])
    lines.append([("exc   ", "label"), (fmt_exc(exc, long=True), "exc")])
    phase = st.get("phase")
    lines.append([("phase ", "label"), (str(phase or "-"), "text"),
                  ("  journal ", "label"), ("%d ev" % len(b.events), "dim")] +
                 ([("  %d bad lines" % b.journal_bad, "warn")] if b.journal_bad else []))
    for i, segs in enumerate(lines[:ih]):
        c.segs(y + 1 + i, x + 2, segs, lim)


_RESULT_COLS = [
    # key, header, width, align, priority (higher survives longer)
    ("trial", "#", 4, "r", 9),
    ("gains", "gains", 0, "l", 8),
    ("exc", "exc", 11, "l", 4),
    ("os", "OS%", 5, "r", 7),
    ("rise", "rise", 5, "r", 3),
    ("settle", "settle", 6, "r", 5),
    ("rms", "rms", 5, "r", 6),
    ("sat", "sat%", 5, "r", 2),
    ("score", "score", 6, "r", 6),
    ("status", "status", 7, "l", 8),
]


def _fit_columns(iw):
    cols = list(_RESULT_COLS)
    min_gains = 10

    def need(cs):
        return sum((min_gains if k == "gains" else wd) + 1 for k, _, wd, _, _ in cs)

    while need(cols) > iw and len(cols) > 2:
        drop = min((c for c in cols if c[0] not in ("trial", "status")), key=lambda c: c[4])
        cols.remove(drop)
    fixed = sum(wd + 1 for k, _, wd, _, _ in cols if k != "gains")
    out = []
    for k, hd, wd, al, _ in cols:
        if k == "gains":
            wd = max(min_gains, min(40, iw - fixed - 1))
        out.append((k, hd, wd, al))
    return out


def _render_results(c, app, y, x, h, w, g):
    b = app.batch
    rows = b.rows()
    best = best_trial(rows)
    right = []
    if best is not None:
        right = [(g["star"] + " best #%s" % best, "best")]
    c.box(y, x, h, w, g, title=[("Results", "title"), (" %d trials" % len(rows), "dim")],
          right=right)
    ih, iw = h - 2, w - 2
    if ih < 2 or iw < 20:
        return
    cols = _fit_columns(iw - 1)
    lim = x + w - 1
    xx = x + 2
    for k, hd, wd, al in cols:
        if k == "gains":
            hd = g["delta"] + " gains"
        if k == "rms" and g["plot"] != "ascii":
            hd = "rms" + g["deg"]
        c.put(y + 1, xx, (hd.rjust(wd) if al == "r" else hd.ljust(wd))[:wd], "label", lim)
        xx += wd + 1
    vis = ih - 1
    if not rows:
        c.put(y + 2, x + 2, "no trials yet", "dim", lim)
        return
    keys = [r["trial"] for r in rows]
    si = keys.index(app.sel) if app.sel in keys else None
    if si is None:
        start = max(0, len(rows) - vis)
    else:
        start = min(max(0, si - vis // 2), max(0, len(rows) - vis))
    if start > 0:
        c.put(y + 1, x + 1, g["up"][0], "dim")
    base = b.baseline()
    for i, r in enumerate(rows[start:start + vis]):
        ry = y + 2 + i
        m = r["metrics"] or {}
        failed = r["estop"] or r["status"] in ("estop", "aborted", "timeout")
        xx = x + 2
        for k, hd, wd, al in cols:
            text, st = "", "text"
            if k == "trial":
                mark = g["star"] if r["trial"] == best else (g["cross"] if failed else " ")
                text = "%s%s" % (mark, str(r["trial"]).rjust(wd - 1))
                st = "best" if r["trial"] == best else ("bad_b" if failed else "value")
            elif k == "gains":
                text = gains_diff(r["gains"], base)
                st = "dim" if text == "=base" else "text"
            elif k == "exc":
                text, st = fmt_exc(r["exc"]), "exc"
            elif k == "os":
                v = m.get("overshoot_pct")
                text, st = _f(v, 1), level_style(v, METRIC_BANDS["overshoot_pct"])
            elif k == "rise":
                text, st = _f(m.get("rise_ms"), 0), "text"
            elif k == "settle":
                v = m.get("settle_ms")
                text, st = _f(v, 0), level_style(v, METRIC_BANDS["settle_ms"])
            elif k == "rms":
                v = m.get("rms_err_deg")
                text, st = _f(v, 2), level_style(v, METRIC_BANDS["rms_err_deg"])
            elif k == "sat":
                v = m.get("sat_frac")
                v = v * 100.0 if isinstance(v, (int, float)) else None
                text, st = _f(v, 1), level_style(v, METRIC_BANDS["sat_pct"])
            elif k == "score":
                text = _f(m.get("score"), 3)
                st = "best" if r["trial"] == best else "value"
            elif k == "status":
                s = r["status"] or "-"
                if r["estop"] and s != "estop":
                    s = s + "+E"
                text = s.upper() if failed else s
                st = {"ok": "ok", "running": "info"}.get(r["status"], "bad_b" if failed else "dim")
            if text == "-":
                st = "dim"
            if failed and k not in ("trial", "status"):
                st = "bad"
            if len(text) > wd:
                text = text[:wd - 1] + g["ellipsis"]
            c.put(ry, xx, text.rjust(wd) if al == "r" else text.ljust(wd), st, lim)
            xx += wd + 1
        if r["trial"] == app.sel:
            c.highlight(ry, x + 1, x + w - 1)


def _render_events(c, app, y, x, h, w, g):
    evs = sorted(app.events, key=lambda e: e[0] or 0.0)
    c.box(y, x, h, w, g, title=[("Events", "title")], right=[("%d" % len(evs), "dim")])
    ih, iw = h - 2, w - 2
    if ih < 1 or iw < 10:
        return
    lim = x + w - 1
    show = evs[-ih:]
    for i, (t, src, text, st) in enumerate(show):
        ts = time.strftime("%H:%M:%S", time.localtime(t)) if t else "--:--:--"
        xx = c.put(y + 1 + i, x + 2, ts + " ", "dim", lim)
        xx = c.put(y + 1 + i, xx, "%-4s " % src[:4], "border", lim)
        if len(text) > lim - xx:
            text = text[:max(0, lim - xx - 1)] + g["ellipsis"]
        c.put(y + 1 + i, xx, text, st, lim)


def _render_trial_view(c, app, y, x, h, w, g):
    b = app.batch
    r = b.trials.get(app.sel)
    title = [("Trial %s" % app.sel, "title")]
    if r:
        title += [("  " + gains_diff(r["gains"], b.baseline()), "text"),
                  ("  " + fmt_exc(r["exc"], long=True), "exc")]
    status = (r or {}).get("status") or "-"
    c.box(y, x, h, w, g, title=title,
          right=[(status, "ok" if status == "ok" else ("info" if status == "running" else "bad_b"))])
    ih, iw = h - 2, w - 2
    rec, err = app.rec_for(app.sel)
    lim = x + w - 1
    m = (r or {}).get("metrics") or {}
    mline = [("OS ", "label"), (_f(m.get("overshoot_pct"), 1) + "%", "value"),
             ("  rise ", "label"), (_f(m.get("rise_ms"), 0) + "ms", "value"),
             ("  settle ", "label"), (_f(m.get("settle_ms"), 0) + "ms", "value"),
             ("  rms ", "label"), (_f(m.get("rms_err_deg"), 2) + g["deg"], "value"),
             ("  steady ", "label"), (_f(m.get("steady_err_deg"), 2) + g["deg"], "value"),
             ("  peak duty ", "label"), (_f(m.get("peak_duty"), 2), "value"),
             ("  sat ", "label"), (_f((m.get("sat_frac") or 0) * 100 if m.get("sat_frac") is not None else None, 1) + "%", "value"),
             ("  osc ", "label"), (_f(m.get("osc_hz"), 1) + "Hz", "value"),
             ("  score ", "label"), (_f(m.get("score"), 3), "best")]
    c.segs(y + ih, x + 2, mline, lim)
    ph = ih - 3
    if rec is None:
        c.put(y + 1 + ph // 2, x + 4, err or "no recording", "warn", lim)
        return
    t = rec["t"]
    tmax = max([v for v in t if v is not None] or [1.0])
    axis_w = 7
    pw = iw - axis_w - 1
    if pw < 4 or ph < 2:
        return
    lo, hi = nice_range(rec["roll"] + rec["sp"])
    marks = [(mx, "exc") for mx in exc_marks(t, rec["exc"], rec["exc_id"])]
    rows, styles = rasterise(
        [(list(zip(t, rec["sp"])), "sp", "-"), (list(zip(t, rec["roll"])), "roll", "*")],
        pw, ph, (0.0, tmax), (lo, hi), g["plot"], hlines=[(0.0, "border")], vmarks=marks)
    px, py = x + 1 + axis_w, y + 2
    for i, row in enumerate(rows):
        for j, ch in enumerate(row):
            c.put(py + i, px + j, ch, styles[i][j] or "dim")
    step = nice_step(hi - lo, max(1, ph // 3))
    v = math.ceil(lo / step) * step
    while v <= hi + 1e-9:
        rr = int(round((hi - v) / (hi - lo) * (ph - 1)))
        c.put(py + rr, x + 1, ("%+5.0f%s" % (v, g["deg"]))[-6:].rjust(6), "dim")
        c.put(py + rr, x + axis_w, g["tick"], "border")
        v += step
    ly = py + ph
    tstep = nice_step(tmax, max(1, pw // 12))
    tv = 0.0
    while tv <= tmax + 1e-9:
        xx = px + int(tv / tmax * (pw - 1)) if tmax > 0 else px
        lab = "%gs" % round(tv, 2)
        c.put(ly, min(xx, px + pw - len(lab)), lab, "dim")
        tv += tstep
    c.segs(y + 1, px + 1, [(" %s " % AXIS_WORDS.get(rec.get("axis"), "roll"), "roll"), ("vs ", "dim"),
                           (" %s " % (rec.get("sp_col") or "setpoint"), "sp"),
                           (" %s exc start/stop " % ("|" if g["plot"] == "ascii" else "┊"), "exc")], lim)


# =============================================================================
# Data ingest (thread): follows the log and the batch files
# =============================================================================

class LogFollower:
    """tail -F over one explicit file, or over the newest raw_*.log in a directory."""

    def __init__(self, path=None, directory=None, pattern="raw_*.log", backlog=BACKLOG_BYTES):
        self.explicit = Path(path) if path else None
        self.directory = Path(directory) if directory else None
        self.pattern = pattern
        self.backlog = backlog
        self.f = None
        self.cur = None
        self.ino = None
        self.pos = 0
        self.buf = b""
        self.last_check = 0.0

    def _open(self, path, from_start=False):
        if self.f:
            self.f.close()
        self.f = open(path, "rb")
        st = os.fstat(self.f.fileno())
        self.cur, self.ino = Path(path), st.st_ino
        start = 0 if from_start else max(0, st.st_size - self.backlog)
        self.f.seek(start)
        self.pos = start
        self.buf = b""
        if start > 0:
            self.f.readline()           # drop the partial first line
            self.pos = self.f.tell()

    def close(self):
        if self.f:
            self.f.close()
            self.f = None

    def _target(self):
        if self.explicit is not None:
            return self.explicit
        return newest_log(self.directory, self.pattern) if self.directory else None

    def poll(self, max_bytes=1 << 20, now=None):
        now = now if now is not None else time.monotonic()
        out = []
        if self.f is None or now - self.last_check >= 1.0:
            self.last_check = now
            tgt = self._target()
            if tgt is not None:
                try:
                    st = os.stat(tgt)
                except OSError:
                    st = None
                if st is not None:
                    if self.f is None:
                        self._open(tgt)
                    elif Path(tgt) != self.cur or st.st_ino != self.ino or st.st_size < self.pos:
                        out += self._read(max_bytes, final=True)    # drain the old file
                        self._open(tgt, from_start=True)
        if self.f is None:
            return out
        return out + self._read(max_bytes)

    def _read(self, max_bytes, final=False):
        data = self.f.read(max_bytes)
        self.pos += len(data)
        self.buf += data
        *complete, self.buf = self.buf.split(b"\n")
        if final and self.buf:
            complete.append(self.buf)
            self.buf = b""
        if len(self.buf) > 65536:
            complete.append(self.buf)
            self.buf = b""
        return [ln.decode("utf-8", "replace") for ln in complete]


class Ingestor:
    def __init__(self, app, log=None, batch_dir=None, latest=False):
        self.app = app
        self.log = log
        self.latest = latest
        self.journal = None
        self.last_disc = 0.0
        self.last_status = 0.0
        self.meta_loaded = False
        if batch_dir is not None:
            self._attach(batch_dir)

    def _attach(self, d):
        with self.app.lock:
            self.app.attach_batch(d)
        self.journal = JsonlFollower(Path(d) / "journal.jsonl")
        self.meta_loaded = False

    def poll_once(self, now=None):
        now = now or time.time()
        app = self.app
        if self.log is not None:
            lines = self.log.poll()
            if lines:
                with app.lock:
                    app.ingest_log(lines)
                    app.log_path = self.log.cur
        if self.latest and now - self.last_disc > 2.0:
            self.last_disc = now
            nb = newest_batch_dir()
            if nb is not None and (app.batch.dir is None or nb != app.batch.dir):
                self._attach(nb)
        b = app.batch
        if b.dir is None:
            return
        if not self.meta_loaded:
            meta = read_json(b.dir / "batch.json")
            if isinstance(meta, dict):
                with app.lock:
                    b.meta = meta
                self.meta_loaded = True
        if self.journal is not None:
            evs = self.journal.poll()
            if evs:
                with app.lock:
                    app.ingest_journal(evs)
            b.journal_bad = self.journal.bad
        if now - self.last_status >= 0.4:
            self.last_status = now
            sp = b.dir / "status.json"
            st = read_json(sp)
            if isinstance(st, dict):
                try:
                    mt = sp.stat().st_mtime
                except OSError:
                    mt = None
                upd = st.get("updated")
                with app.lock:
                    b.status = st
                    b.status_mtime = upd if isinstance(upd, (int, float)) else mt
            ctl = read_control(b.dir)
            with app.lock:
                b.control = ctl

    def run(self, stop):
        while not stop.is_set():
            try:
                self.poll_once()
            except Exception as exc:  # noqa: BLE001
                self.app.errors.append("ingest: %r" % exc)
            stop.wait(0.05)


# =============================================================================
# Selftest world: synthetic FC stream + fake batch runner (steppable clock)
# =============================================================================

SELFTEST_BASE = {"ATT_P": 5.0, "ATT_I": 0.0, "RATE_P": 40.0, "RATE_I": 0.0, "RATE_D": 0.0,
                 "ATT_GAIN": 1.0, "D_LPF_HZ": 30.0, "GYRO_LPF_HZ": 0.0, "ROLL_TRIM": 0.0,
                 "MOTOR_TRIM_LEFT": 1.0}
SELFTEST_GRID = [{}, {"ATT_P": 8.0}, {"ATT_P": 8.0, "RATE_P": 60.0}, {"ATT_P": 12.0},
                 {"ATT_P": 12.0, "RATE_P": 60.0}, {"ATT_P": 16.0, "RATE_P": 30.0},
                 {"ATT_P": 6.0, "RATE_P": 50.0}, {"ATT_P": 10.0, "RATE_P": 50.0}]
SELFTEST_EXC = {"type": "doublet", "target": "att", "amp": 140, "dur_ms": 2000,
                "f0_mHz": 0, "f1_mHz": 0}
_PHASES = (("ARMING", 0.6), ("RAMP_UP", 0.9), ("RUN", 3.0), ("RAMP_DOWN", 0.6), ("DONE", 0.3))


class SelftestWorld:
    """A fake drone + fake runner writing a raw log and a batch dir, advanced by step(now)."""

    TICK = 0.04
    SUB = 8

    def __init__(self, base, t0, seed=7, listen=True):
        self.base = Path(base)
        self.t = t0
        self.t0 = t0
        self.rng = random.Random(seed)
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(t0))
        (self.base / "riskybird-logs").mkdir(parents=True, exist_ok=True)
        self.log_path = self.base / "riskybird-logs" / "raw_selftest.log"
        self.batch_dir = self.base / "rig" / ("%s-selftest" % stamp)
        (self.batch_dir / "trials").mkdir(parents=True, exist_ok=True)
        self.logf = open(self.log_path, "a", encoding="utf-8")
        self.counts = {"ESTOP": 0, "STOP": 0, "other": 0}
        self.sock = None
        self.cmd_port = 0
        if listen:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(("127.0.0.1", 0))
            self.sock.setblocking(False)
            self.cmd_port = self.sock.getsockname()[1]
        self.n = len(SELFTEST_GRID)
        self.trial = 0
        self.phase_i = None
        self.phase_t = t0
        self.gap_until = t0 + 0.5
        self.rs = "IDLE"
        self.st = FLAG_CALDONE
        self.it = 0
        self.gs = 0
        self.ex = 0
        self.gains = dict(SELFTEST_BASE)
        self.th, self.om, self.m = 0.49, 0.0, 0.0
        self.sp_cmd, self.sp = 0.0, 0.49
        self.coll_cmd, self.coll = 0.95, 0.0
        self.exc_t0 = None
        self.exc_val = 0.0
        self.duty = [0.0] * 4
        self.motor_s = 0.0
        self.first_spin = None
        self.rec = []
        self.trial_start_t = None
        self.paused = False
        self.aborted = False
        self.ended = False
        self.ctl_seq = 0
        self.last_status = 0.0
        self.last_rb = 0.0
        self.rb_seq = 0
        self.estop_pending = False
        self.batch_t = t0
        meta = {"name": "selftest", "created": t0, "mode": "rig-roll", "elf": "(selftest)",
                "elf_sha1": "0" * 40,
                "spec": {"collective_permille": 950, "roll_sp_mdeg": 0,
                         "grid": [{"gains": dict(SELFTEST_BASE, **gd)} for gd in SELFTEST_GRID],
                         "excitations": [SELFTEST_EXC], "repeats": 1,
                         "limits": {"max_trials": self.n, "max_coll_permille": 1100,
                                    "max_run_ms": 8000, "motor_budget_s": 120}},
                "approval_note": "selftest: synthetic, no hardware"}
        write_json_atomic(self.batch_dir / "batch.json", meta)
        self._j({"ev": "batch_start"})

    # -- output helpers --
    def _j(self, obj):
        obj = dict(obj)
        obj["t"] = round(self.t, 3)
        append_journal(self.batch_dir / "journal.jsonl", obj)

    def _log(self, text):
        self.logf.write("%.3f %s\n" % (self.t, text))

    def close(self):
        try:
            self.logf.close()
        except OSError:
            pass
        if self.sock:
            self.sock.close()

    # -- uplink --
    def _poll_cmds(self):
        if not self.sock:
            return
        while True:
            try:
                data, _ = self.sock.recvfrom(256)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                return
            cmd = data.decode("ascii", "replace").strip().upper()
            if cmd == "ESTOP":
                self.counts["ESTOP"] += 1
                self.estop_pending = True
            elif cmd == "STOP":
                self.counts["STOP"] += 1
                if self.rs in ("ARMING", "RAMP_UP", "RUN"):
                    self._log("FCACK STOP")
                    self._enter(3, "stop")
            else:
                self.counts["other"] += 1

    # -- runner + FC state machine --
    def _enter(self, i, why=None):
        self.phase_i = i
        self.phase_t = self.t
        self.rs = _PHASES[i][0]
        if self.rs == "ARMING":
            self.st = FLAG_CALDONE | FLAG_ARMING
            self._log("RIG EVENT arm request")
        elif self.rs == "RAMP_UP":
            self.st = FLAG_CALDONE | FLAG_ARMED
            self.sp = self.th
            self._log("RIG EVENT armed roll=%d mdeg" % int(math.degrees(self.th) * 1000))
            self._log("RIGGAINS gs=%d %s" % (self.gs, " ".join(
                "%s=%s" % (k, fmt_num(v)) for k, v in sorted(self.gains.items()))))
        elif self.rs == "RUN":
            self._log("RIG EVENT liftoff")
        elif self.rs == "RAMP_DOWN":
            self._log("RIG EVENT ramp-down %s" % (why or "run end"))
        elif self.rs == "DONE":
            self.st = FLAG_CALDONE
        self._j({"ev": "state", "rs": self.rs})

    def _start_trial(self):
        self.trial += 1
        gd = SELFTEST_GRID[self.trial - 1]
        want = dict(SELFTEST_BASE, **gd)
        for k in sorted(want):
            if not _close(want[k], self.gains.get(k)):
                self.gs += 1
                self.gains[k] = want[k]
                self._log("RIGGAIN %s=%s" % (k, fmt_num(want[k])))
                self._log("FCACK GAIN")
                self._j({"ev": "cmd", "text": "GAIN %s %s" % (k, fmt_num(want[k])),
                         "reply": "FCACK GAIN"})
        if self.trial == 3:
            self._log("FCNAK GAIN")
            self._j({"ev": "cmd", "text": "GAIN RATE_D 50", "reply": "FCNAK GAIN"})
        self.trial_start_t = self.t
        self.rec = []
        self._j({"ev": "trial_start", "trial": self.trial, "gains": dict(want),
                 "exc": dict(SELFTEST_EXC)})
        self._enter(0)

    def _end_trial(self, status):
        self._j({"ev": "trial_end", "trial": self.trial, "status": status})
        d = self.batch_dir / "trials" / str(self.trial)
        d.mkdir(parents=True, exist_ok=True)
        cols = ["t_us", "rig_state", "exc_id", "gain_seq", "roll", "roll_tgt", "exc_val",
                "duty0", "duty1", "duty2", "duty3"]
        tmp = d / ".rec.csv.tmp"
        with open(tmp, "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(cols)
            for row in self.rec:
                wr.writerow(row)
        os.replace(str(tmp), str(d / "rec.csv"))
        self._j({"ev": "fetch", "trial": self.trial, "csv": "trials/%d/rec.csv" % self.trial,
                 "bin": "trials/%d/rec.bin" % self.trial, "samples": len(self.rec), "gaps": 0})
        if status == "ok":
            m = self._metrics()
            write_json_atomic(d / "metrics.json", m)
            ev = {"ev": "metrics", "trial": self.trial}
            ev.update(m)
            self._j(ev)
        self.phase_i = None
        self.rs = "IDLE"
        self.gap_until = self.t + 0.5

    def _metrics(self):
        amp = SELFTEST_EXC["amp"] / 1000.0
        rows = [r for r in self.rec if r[2] == self.ex]
        if self.exc_t0 is None or not rows:
            return {}
        t0us = int(self.exc_t0 * 1e6)
        half = [r for r in rows if t0us <= r[0] <= t0us + 1e6]
        th = [r[4] for r in half] or [0.0]
        peak = max(th)
        os_pct = max(0.0, (peak - amp) / amp * 100.0)
        rise = next(((r[0] - t0us) / 1000.0 for r in half if r[4] >= 0.9 * amp), None)
        settle = 0.0
        for r in half:
            if abs(r[4] - amp) > 0.05 * amp:
                settle = (r[0] - t0us) / 1000.0
        run = [r for r in self.rec if r[1] == "RUN"]
        err = [math.degrees(r[4] - r[5]) for r in run] or [0.0]
        rms = math.sqrt(sum(e * e for e in err) / len(err))
        duties = [max(r[7:11]) for r in run] or [0.0]
        sat = sum(1 for d in duties if d >= DUTY_CEILING - 1e-3) / float(len(duties))
        steady = math.degrees(sum(r[4] for r in half[-8:]) / max(1, len(half[-8:])) - amp)
        zc, prev = 0, None
        for e in err:
            s = e > 0
            if prev is not None and s != prev:
                zc += 1
            prev = s
        dur = len(run) * self.TICK
        return {"overshoot_pct": round(os_pct, 2), "rise_ms": rise and round(rise, 1),
                "settle_ms": round(settle, 1), "rms_err_deg": round(rms, 3),
                "steady_err_deg": round(steady, 3), "peak_duty": round(max(duties), 3),
                "sat_frac": round(sat, 3), "osc_hz": round(zc / 2.0 / dur, 2) if dur else None,
                "score": round(rms + 0.03 * os_pct + 4.0 * sat, 3)}

    def _check_control(self):
        c = read_control(self.batch_dir)
        if c["seq"] > self.ctl_seq:
            self.ctl_seq = c["seq"]
            if c["abort"] and not self.aborted:
                self.aborted = True
                self._j({"ev": "note", "text": "abort requested via control.json (seq %d)" % c["seq"]})
            self.paused = c["pause"]

    def _tick_runner(self):
        if self.ended:
            return
        if self.estop_pending:
            self.estop_pending = False
            if self.rs != "ESTOP":
                self.rs = "ESTOP"
                self.st = FLAG_CALDONE | FLAG_ESTOP
                self._log("FCACK ESTOP")
                self._log("RIG EVENT estop: host ESTOP cmd")
                self._j({"ev": "estop", "reason": "host ESTOP cmd", "source": "fc"})
                if self.phase_i is not None:
                    self._end_trial("estop")
                self.rs = "ESTOP"
        if self.phase_i is None:
            self._check_control()
            if self.aborted or self.rs == "ESTOP":
                if self.trial_start_t is not None or self.aborted:
                    self._j({"ev": "batch_end", "status": "aborted"})
                    self.ended = True
                return
            if self.paused or self.t < self.gap_until:
                return
            if self.trial >= self.n:
                self._j({"ev": "batch_end", "status": "ok"})
                self.ended = True
                return
            self._start_trial()
            return
        name, dur = _PHASES[self.phase_i]
        el = self.t - self.phase_t
        if name == "RUN":
            if self.exc_t0 is None and el >= 0.4:
                self.ex += 1
                self.exc_t0 = self.t
                self._log("FCACK EXC")
                self._j({"ev": "cmd", "text": "EXC doublet att 140 2000", "reply": "FCACK EXC"})
            if self.trial == 6 and self.exc_t0 is not None and self.t - self.exc_t0 >= 1.3:
                self.rs = "ESTOP"
                self.st = FLAG_CALDONE | FLAG_ESTOP
                self._log("RIG EVENT estop: tilt 25.6deg > 25.0deg")
                self._j({"ev": "estop", "reason": "tilt 25.6deg", "source": "fc"})
                self._end_trial("estop")
                self.exc_t0 = None
                self._log("FCACK RESET")
                self._j({"ev": "cmd", "text": "RESET", "reply": "FCACK RESET"})
                self.rs = "IDLE"
                self.st = FLAG_CALDONE
                return
        if el >= dur:
            if self.phase_i + 1 < len(_PHASES):
                self._enter(self.phase_i + 1)
            else:
                self._end_trial("ok")
                self.exc_t0 = None

    def _tick_physics(self):
        dt = self.TICK / self.SUB
        exc = 0.0
        if self.rs == "RUN" and self.exc_t0 is not None:
            te = self.t - self.exc_t0
            amp = SELFTEST_EXC["amp"] / 1000.0
            d = SELFTEST_EXC["dur_ms"] / 1000.0
            exc = amp if 0 <= te < d / 2 else (-amp if d / 2 <= te < d else 0.0)
        self.exc_val = exc
        armed = self.rs in ("RAMP_UP", "RUN", "RAMP_DOWN")
        el = self.t - self.phase_t
        if self.rs == "RAMP_UP":
            self.coll = self.coll_cmd * min(1.0, el / _PHASES[1][1])
        elif self.rs == "RAMP_DOWN":
            self.coll = self.coll_cmd * max(0.0, 1.0 - el / _PHASES[3][1])
        elif self.rs == "RUN":
            self.coll = self.coll_cmd
        else:
            self.coll = 0.0
        if armed and self.rs != "RAMP_DOWN":
            step = 0.5 * self.TICK
            self.sp += max(-step, min(step, self.sp_cmd - self.sp))
        kp, kr = self.gains["ATT_P"], self.gains["RATE_P"]
        auth = min(1.0, self.coll / self.coll_cmd) if armed else 0.0
        tgt = self.sp + exc
        for _ in range(self.SUB):
            gyro = self.om + self.rng.gauss(0, 0.02)
            mcmd = kr * (kp * (tgt - self.th) - gyro) * auth
            self.m += (mcmd - self.m) * dt / 0.045
            acc = self.m + (0.0 if auth > 0.6 else 3.0 * (0.6 - auth))  # unbalance -> stop
            self.om += acc * dt
            self.th += self.om * dt
            if self.th > 0.5236:
                self.th, self.om = 0.5236, min(0.0, self.om)
            if self.th < -0.5236:
                self.th, self.om = -0.5236, max(0.0, self.om)
        base = 0.55 * (self.coll / 0.95) if armed else 0.0
        diff = max(-0.6, min(0.6, self.m * 0.0045)) if armed else 0.0
        self.duty = [max(0.0, min(DUTY_CEILING, base + s * diff + self.rng.gauss(0, 0.004) * (base > 0)))
                     for s in (1, 1, -1, -1)]
        if armed:
            self.motor_s += self.TICK
            if self.first_spin is None:
                self.first_spin = self.t
        return tgt

    def step(self, now):
        self._poll_cmds()
        while self.t + self.TICK <= now:
            self.t += self.TICK
            self.it += 1
            self._tick_runner()
            tgt = self._tick_physics()
            mdeg = lambda rad: int(round(math.degrees(rad) * 1000))  # noqa: E731
            d = [int(round(x * 10000)) for x in self.duty]
            self._log("RIG it=%d st=%d rs=%s r=%d p=%d gx=%d sp=%d tgt=%d coll=%d exc=%d ex=%d gs=%d "
                      "d=[%d %d %d %d] vb=%d ax=R" % (
                          self.it * 8, self.st, self.rs, mdeg(self.th), mdeg(0.004),
                          int(self.om * 1000), mdeg(self.sp if self.rs != "IDLE" else 0.0),
                          mdeg(tgt), int(self.coll * 1000), int(self.exc_val * 1000), self.ex,
                          self.gs, d[0], d[1], d[2], d[3],
                          7900 - int(self.motor_s * 2) - int(sum(self.duty) * 60)))
            if self.phase_i is not None:
                self.rec.append((int(self.t * 1e6), self.rs, self.ex, self.gs, round(self.th, 5),
                                 round(tgt, 5), round(self.exc_val, 4)) + tuple(
                                     round(x, 4) for x in self.duty))
            if self.t - self.last_rb >= 0.2:
                self.last_rb = self.t
                self.rb_seq = (self.rb_seq + 1) % 256
                self._log("RBLINK v1 ml=up up=%d seq=%d fl=0x21[ARMED COMMITTED ] duty=[%d %d %d %d] "
                          "fr=%d gap=%d lost=0 stale=0 crc=0 noise=0 cuts=0 pwmerr=0 kill=0 "
                          "txt=%d cl=1" % (int((self.t - self.t0) * 1000) + 41000, self.rb_seq,
                                           d[0], d[1], d[2], d[3], self.it * 8,
                                           18 + self.rng.randint(0, 6), self.it * 3))
            if self.t - self.last_status >= 0.5:
                self.last_status = self.t
                self._write_status()
        self.logf.flush()

    def _write_status(self):
        cur = {}
        if self.phase_i is not None:
            cur = {"gains": dict(SELFTEST_BASE, **SELFTEST_GRID[self.trial - 1]),
                   "exc": dict(SELFTEST_EXC)}
        phase = "done" if self.ended else ("paused" if self.paused and self.phase_i is None
                                           else (self.rs.lower() if self.phase_i is not None else "between"))
        write_json_atomic(self.batch_dir / "status.json", {
            "phase": phase, "trial": self.trial, "n_trials": self.n, "current": cur,
            "motor_ms": int(self.motor_s * 1000), "motor_budget_ms": 120000,
            "commit_left_ms": int(900000 - ((self.t - self.first_spin) * 1000 if self.first_spin else 0)),
            "paused": bool(self.paused and self.phase_i is None), "updated": round(self.t, 3)})


# =============================================================================
# curses front end
# =============================================================================

def pick_glyphs(force_ascii=False):
    if force_ascii:
        return GLYPHS_ASCII
    enc = (locale.getpreferredencoding(False) or "").lower().replace("-", "")
    return GLYPHS_UNICODE if "utf8" in enc else GLYPHS_ASCII


class CursesPainter:
    def __init__(self, curses, scr):
        self.curses = curses
        self.scr = scr
        self.attr = {}
        self.attr_hl = {}
        self._init()

    def _init(self):
        cu = self.curses
        amap = {"b": cu.A_BOLD, "d": cu.A_DIM, "r": cu.A_REVERSE, "u": cu.A_UNDERLINE}

        def attrs(s):
            a = 0
            for ch in s:
                a |= amap.get(ch, 0)
            return a

        colors = False
        try:
            colors = cu.has_colors()
            if colors:
                cu.start_color()
        except cu.error:
            colors = False
        if not colors:
            for name in STYLE_TABLE:
                a = attrs(MONO_ATTRS.get(name, ""))
                if name.startswith("badge_"):
                    a |= cu.A_REVERSE | cu.A_BOLD
                self.attr[name] = a
                self.attr_hl[name] = a | cu.A_REVERSE
            self.mode = "mono"
            return
        bg_default = -1
        try:
            cu.use_default_colors()
        except cu.error:
            bg_default = cu.COLOR_BLACK
        rich = getattr(cu, "COLORS", 8) >= 256
        self.mode = "256" if rich else "8"
        c8 = {"black": cu.COLOR_BLACK, "red": cu.COLOR_RED, "green": cu.COLOR_GREEN,
              "yellow": cu.COLOR_YELLOW, "blue": cu.COLOR_BLUE, "magenta": cu.COLOR_MAGENTA,
              "cyan": cu.COLOR_CYAN, "white": cu.COLOR_WHITE}
        max_pairs = getattr(cu, "COLOR_PAIRS", 64)
        pair = [1]

        def mk(fg, bg):
            if pair[0] >= max_pairs:
                return 0
            try:
                cu.init_pair(pair[0], fg, bg)
            except cu.error:
                return 0
            p = cu.color_pair(pair[0])
            pair[0] += 1
            return p

        for name, (f256, b256, f8, b8, at) in STYLE_TABLE.items():
            fg = f256 if rich else c8[f8]
            bg = (b256 if rich else c8[b8]) if (b256 is not None) else bg_default
            self.attr[name] = mk(fg, bg) | attrs(at)
            if b256 is None and rich:
                self.attr_hl[name] = mk(fg, SEL_BG[0]) | attrs(at)
            elif b256 is None:
                self.attr_hl[name] = self.attr[name] | cu.A_REVERSE   # 8 colours: no spare bg
            else:
                self.attr_hl[name] = self.attr[name]
        self.attr[None] = mk(-1 if bg_default == -1 else cu.COLOR_WHITE, bg_default)
        self.attr_hl[None] = (mk(-1 if bg_default == -1 else cu.COLOR_WHITE, SEL_BG[0]) if rich
                              else self.attr[None] | cu.A_REVERSE)

    def paint(self, canvas):
        scr, cu = self.scr, self.curses
        scr.erase()
        for y in range(canvas.h):
            for x, text, s, hh in canvas.runs(y):
                if y == canvas.h - 1 and x + len(text) >= canvas.w:
                    text = text[:max(0, canvas.w - 1 - x)]   # never the bottom-right cell
                    if not text:
                        continue
                a = (self.attr_hl if hh else self.attr).get(s, 0)
                try:
                    scr.addstr(y, x, text, a)
                except Exception:  # noqa: BLE001 - one bad cell must not drop the frame
                    pass
        scr.noutrefresh()
        cu.doupdate()


def _curses_main(scr, app, g, stop):
    import curses
    try:
        curses.curs_set(0)
    except curses.error:
        pass
    scr.keypad(True)
    scr.timeout(50)
    painter = CursesPainter(curses, scr)
    last = 0.0
    while True:
        keys = []
        ch = scr.getch()
        while ch != -1:
            keys.append(ch)
            scr.timeout(0)
            ch = scr.getch()
        scr.timeout(50)
        for k in keys:
            # ESTOP first, before anything that touches the model lock.
            if k in (ord("E"), ord("e"), ord(" ")):
                app.estop()
                continue
            if k in (ord("q"), ord("Q")):
                return 0
            if k == ord("s"):
                app.stop()
            elif k == ord("p"):
                app.toggle_pause()
            elif k in (curses.KEY_UP, ord("k")):
                with app.lock:
                    app.move_sel(-1)
            elif k in (curses.KEY_DOWN, ord("j")):
                with app.lock:
                    app.move_sel(+1)
            elif k in (10, 13, curses.KEY_ENTER):
                with app.lock:
                    if app.sel is None:
                        app.move_sel(0)
                    if app.sel is not None:
                        app.view = "trial"
            elif k in (27, curses.KEY_BACKSPACE, 127):
                app.view = "main"
            elif k == curses.KEY_RESIZE:
                last = 0.0
        now = time.time()
        if now - last >= 0.1 or keys:
            last = now
            app.drain_notices()
            try:
                h, w = scr.getmaxyx()
                frame = render_frame(app, w, h, now, g)
                painter.paint(frame)
            except Exception as exc:  # noqa: BLE001 - never lose the key loop to a render bug
                app.errors.append("render: %r" % exc)
                try:
                    scr.erase()
                    scr.addstr(0, 0, "RENDER ERROR %r" % exc)
                    scr.addstr(1, 0, "E / space = ESTOP still works.  q = quit viewer.")
                    scr.noutrefresh()
                    curses.doupdate()
                except Exception:  # noqa: BLE001
                    pass


# =============================================================================
# Entry points
# =============================================================================

def add_arguments(p):
    src = p.add_mutually_exclusive_group()
    src.add_argument("--batch", metavar="DIR", help="batch directory to follow (results/rig/...)")
    src.add_argument("--latest", action="store_true", help="follow the newest results/rig/* batch")
    src.add_argument("--replay", metavar="DIR",
                     help="browse a finished batch: no live log, journal/control left untouched")
    p.add_argument("--log", metavar="RAWLOG",
                   help="raw log to follow (default: newest results/riskybird-logs/raw_*.log)")
    p.add_argument("--selftest", action="store_true",
                   help="synthetic drone + fake runner in a temp dir; UDP goes to 127.0.0.1 only")
    p.add_argument("--drone", default=DEFAULT_DRONE, help="drone IP for ESTOP/STOP (default %(default)s)")
    p.add_argument("--port", type=int, default=CMD_PORT, help="drone command port (default %(default)s)")
    p.add_argument("--ascii", action="store_true", help="ASCII glyphs only (no braille/box drawing)")
    p.add_argument("--snapshot", metavar="WxH",
                   help="render one frame as text to stdout and exit (no terminal needed)")
    p.add_argument("--snapshot-html", metavar="PATH", help="with --snapshot: also write an HTML render")
    return p


def _parse_size(s):
    m = re.match(r"^(\d+)x(\d+)$", s or "")
    if not m:
        raise SystemExit("--snapshot wants WxH, e.g. 120x40")
    return int(m.group(1)), int(m.group(2))


def run_from_args(args):
    try:
        locale.setlocale(locale.LC_ALL, "")
    except locale.Error:
        pass
    g = pick_glyphs(args.ascii)
    tmp = None
    world = None
    log = None
    stop = threading.Event()
    try:
        if args.selftest:
            tmp = tempfile.mkdtemp(prefix="rig-tui-selftest-")
            now = time.time()
            world = SelftestWorld(tmp, now - 17.0)
            world.step(now)
            drone, port = "127.0.0.1", world.cmd_port      # never the real drone
            log = LogFollower(path=world.log_path, backlog=1 << 30)
            batch_dir, latest, readonly = world.batch_dir, False, False
            desc = "selftest"
        else:
            drone, port = args.drone, args.port
            readonly = bool(args.replay)
            batch_dir = Path(args.replay or args.batch) if (args.replay or args.batch) else None
            if batch_dir is not None and not batch_dir.is_dir():
                print("rig tui: no such batch directory: %s" % batch_dir, file=sys.stderr)
                return 2
            latest = bool(args.latest)
            if args.log:
                log, desc = LogFollower(path=args.log), args.log
            elif args.replay:
                log, desc = None, "replay"
            else:
                log, desc = LogFollower(directory=LOG_DIR), "newest %s/raw_*.log" % (
                    os.path.relpath(str(LOG_DIR), os.getcwd()))
        app = App(drone=drone, port=port, batch_dir=batch_dir, readonly=readonly,
                  selftest=world, log_desc=desc)
        ing = Ingestor(app, log=log, batch_dir=batch_dir, latest=latest)
        if args.snapshot:
            w, h = _parse_size(args.snapshot)
            if world is not None:
                world.step(world.t0 + 23.0)
            for _ in range(50):
                ing.poll_once()
            app.drain_notices()
            now = world.t if world is not None else time.time()
            frame = render_frame(app, w, h, now, g)
            sys.stdout.write(frame.to_text() + "\n")
            if args.snapshot_html:
                Path(args.snapshot_html).write_text(frame.to_html("rb rig tui %dx%d" % (w, h)),
                                                    encoding="utf-8")
            return 0
        if not sys.stdout.isatty():
            print("rig tui: needs a terminal (use --snapshot WxH for a headless frame)", file=sys.stderr)
            return 2
        threading.Thread(target=ing.run, args=(stop,), daemon=True).start()
        if world is not None:
            def drive():
                while not stop.is_set():
                    try:
                        world.step(time.time())
                    except Exception as exc:  # noqa: BLE001
                        app.errors.append("selftest: %r" % exc)
                    stop.wait(0.02)
            threading.Thread(target=drive, daemon=True).start()
        os.environ.setdefault("ESCDELAY", "25")
        import curses
        rc = curses.wrapper(_curses_main, app, g, stop)
        stop.set()
        if world is not None:
            time.sleep(0.05)
            world._poll_cmds()
            print("rig tui selftest: local listener 127.0.0.1:%d received ESTOP x%d, STOP x%d; "
                  "estop keypresses %d (nothing sent to %s)" % (
                      world.cmd_port, world.counts["ESTOP"], world.counts["STOP"],
                      len(app.estops), DEFAULT_DRONE))
        print("rig tui: viewer closed; the batch runner (if any) is still running.")
        return rc or 0
    finally:
        stop.set()
        if log is not None:
            log.close()
        if world is not None:
            world.close()
        if tmp:
            shutil.rmtree(tmp, ignore_errors=True)


def main(argv=None):
    p = argparse.ArgumentParser(prog="rig_tui", description="RiskyBird roll-harness tuning TUI")
    add_arguments(p)
    return run_from_args(p.parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
