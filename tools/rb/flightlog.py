"""Publish and read RiskyBird flight logs: split raw logs into trials, index them, compare them.

The ground-station monitor (tools/bench/riskybird-flightready.py) writes one raw log per
session to results/riskybird-logs/raw_<YYYYmmdd_HHMMSS>.log. Each line is
"<host unix time> <datagram text>", holding flight_controller lines AND the ESP's RBLINK
lines. Those logs are published to a separate git repo (default ~/flight-logs) with one
folder per flight trial:

    trials/<date>_T<nn>_<HHMMSS of ARMING>/
        raw.log       the slice, unchanged (from 10 s before ARMING)
        fc.csv        parsed flight_controller lines, t = seconds from ARMING
        rblink.csv    parsed RBLINK lines, same clock
        metrics.json  numbers computed here (no interpretation)
        summary.md    one generated line, then interpretation added by hand

This module is a port of that repo's tools/split_trials.py and tools/add_trial.py. The
CSV columns, metrics keys, README row and index.csv columns are kept byte-compatible so
the existing trials stay comparable. The one behavioural change: a raw log keeps growing
during a session and is pushed repeatedly, so split() recognises trials it already
published (by ARMING time) and keeps their numbers and hand-written summaries.

FC roll/pitch/yaw are Gibbs/Rodrigues components tan(theta/2), not radians; degrees are
2*atan(g). st bits: 1 ARMED, 2 ESTOP, 4 ARMING, 8 CALDONE. Stdlib only.
"""

from __future__ import annotations

import bisect
import csv
import datetime
import json
import math
import os
import re
import shutil
import subprocess
from pathlib import Path
from typing import Any, Callable

from .boards import ROOT


DEFAULT_REPO = Path(os.environ["RB_FLIGHT_LOGS"]) if os.environ.get("RB_FLIGHT_LOGS") \
    else Path.home() / "flight-logs"
DEFAULT_RAW_DIR = ROOT / "results" / "riskybird-logs"

ARMED, ESTOP, ARMING, CALDONE = 1, 2, 4, 8

# The same regexes as split_trials.py. KV deliberately needs a word boundary after the
# number, so "dt=1ms" does not parse and fc.csv's dt column stays empty, as it always has.
KV = re.compile(r"\b(\w+)=(-?[0-9]+(?:\.[0-9]+)?)\b")
TILT = re.compile(r"tilt=est(-?\d+)deg/acc(-?\d+)deg")
AMAG = re.compile(r"\|a\|=(-?[0-9.]+)")
DUTY = re.compile(r"duty=\[([^\]]*)\]")
U = re.compile(r"u=\[([^\]]*)\]")
FL = re.compile(r"fl=0x([0-9a-fA-F]+)\[([^\]]*)\]")
ML = re.compile(r"ml=(\w+)")
TRIAL_NUMBER = re.compile(r"_T(\d+)_")
RAW_DATE = re.compile(r"raw_(\d{4})(\d{2})(\d{2})_")

FC_COLS = ["host_ts", "t", "it", "dt", "st", "roll_g", "pitch_g", "yaw_g", "z", "tofv",
           "tofh", "x", "y", "vx", "vy", "vz", "zsp", "vbat", "tilt_est", "tilt_acc",
           "amag", "u0", "u1", "u2", "u3", "duty0", "duty1", "duty2", "duty3"]
RB_COLS = ["host_ts", "t", "ml", "fl_hex", "fl", "duty0", "duty1", "duty2", "duty3", "up",
           "seq", "fr", "gap", "lost", "stale", "crc", "cuts", "pwmerr", "kill", "cl"]

README_ANCHOR = "\n`index.csv` has the same list"
# Two ARMING times closer than this are the same RESET press seen in two versions of a log.
# Real RESETs are >= ~0.3 s apart only when mashed (T36/T37), and those share a folder second
# but differ by far more than the log's own 1 ms timestamp jitter.
MATCH_S = 0.5

Row = tuple[float, str, dict[str, Any]]


class FlightLogError(Exception):
    """Concise failure while splitting, indexing or publishing flight logs."""


# --------------------------------------------------------------------------- parsing

def _lines(path: Path) -> list[str]:
    """Raw log lines. A trailing line with no newline is a datagram still being written by the
    monitor (the log is copied while it grows), so it is dropped rather than half-parsed."""
    lines = path.read_text(errors="replace").splitlines(keepends=True)
    if lines and not lines[-1].endswith("\n"):
        lines.pop()
    return lines


def _parse(lines: list[str]) -> tuple[list[Row], list[Row]]:
    fc: list[Row] = []
    rb: list[Row] = []
    for raw in lines:
        head, _, text = raw.rstrip("\n").partition(" ")
        try:
            ts = float(head)
        except ValueError:
            continue
        if "RBLINK" in text:
            d: dict[str, Any] = dict((k, float(v)) for k, v in KV.findall(text))
            m = FL.search(text)
            if m:
                d["fl_hex"], d["fl"] = "0x" + m.group(1), m.group(2).strip()
            m = ML.search(text)
            d["ml"] = m.group(1) if m else ""
            m = DUTY.search(text)
            if m:
                for i, v in enumerate(m.group(1).split()[:4]):
                    d["duty%d" % i] = int(v)
            d["host_ts"] = ts
            rb.append((ts, raw, d))
        elif "flight_controller" in text and " it=" in text and " st=" in text:
            d = dict((k, float(v)) for k, v in KV.findall(text))
            d["roll_g"], d["pitch_g"], d["yaw_g"] = d.get("roll"), d.get("pitch"), d.get("yaw")
            m = TILT.search(text)
            if m:
                d["tilt_est"], d["tilt_acc"] = int(m.group(1)), int(m.group(2))
            m = AMAG.search(text)
            if m:
                d["amag"] = float(m.group(1))
            for name, rx in (("u", U), ("duty", DUTY)):
                m = rx.search(text)
                if m:
                    for i, v in enumerate(m.group(1).split()[:4]):
                        d["%s%d" % (name, i)] = float(v)
            d["host_ts"] = ts
            fc.append((ts, raw, d))
    return fc, rb


def _find_starts(fc: list[Row]) -> list[float]:
    starts, prev = [], 0
    for ts, _, d in fc:
        st = int(d.get("st", 0))
        # A new RESET shows as ARMING appearing. The state before it can still carry
        # ARMED+ESTOP (st=11) from the previous attempt, so only ARMING is compared.
        if st & ARMING and not prev & ARMING:
            starts.append(ts)
        prev = st
    return starts


def _trial_end(fc: list[Row], t0: float) -> float:
    last_active = t0
    for ts, _, d in fc:
        if ts < t0:
            continue
        if int(d.get("st", 0)) & (ARMING | ARMED):
            last_active = ts
        if ts - last_active > 8 or ts - t0 > 60:
            break
    return min(last_active + 8, t0 + 60)


def _metrics(fc: list[Row], fcw: list[Row], rbw: list[Row], t0: float) -> dict[str, Any]:
    m: dict[str, Any] = {}
    # "armed" = actually flying: ARMED and not yet ESTOP (st=11 rows are the aftermath)
    armed = [d for ts, _, d in fcw
             if int(d.get("st", 0)) & ARMED and not int(d.get("st", 0)) & ESTOP]
    t_arm = next((ts for ts, _, d in fcw if int(d.get("st", 0)) & ARMED), None)
    t_estop = next((ts for ts, _, d in fcw if ts >= t0 and int(d.get("st", 0)) & ESTOP), None)
    m["arming_to_armed_s"] = round(t_arm - t0, 2) if t_arm else None
    t_stop = t_estop or (fcw[-1][0] if fcw else None)
    m["armed_s_before_estop_or_end"] = round(t_stop - t_arm, 2) if t_arm and t_stop else 0
    m["fc_estop_at_s"] = round(t_estop - t0, 2) if t_estop else None
    pre = [d for ts, _, d in fc if t0 - 1 <= ts < t0]   # full log: pre-roll may belong to the previous trial
    at_arm = [d for ts, _, d in fcw if t_arm and 0 <= ts - t_arm < 0.06]
    if pre:
        m["pre_arm_tilt_est_acc"] = [pre[-1].get("tilt_est"), pre[-1].get("tilt_acc")]
        m["pre_arm_vbat"] = pre[-1].get("vbat")
    if at_arm:
        m["spinup_duty"] = [at_arm[0].get("duty%d" % i) for i in range(4)]
    if armed:
        m["max_tilt_est_deg_armed"] = max(d.get("tilt_est", 0) for d in armed)  # the FC's own tilt=, what its guard sees
        m["max_tofh_armed_m"] = max(d.get("tofh", 0) for d in armed)
        m["max_z_armed_m"] = max(d.get("z", 0) for d in armed)
        m["min_vbat_armed"] = min(d.get("vbat", 9) for d in armed)
        m["last_armed_duty"] = [armed[-1].get("duty%d" % i) for i in range(4)]
        m["max_duty_armed"] = [max(d.get("duty%d" % i, 0) for d in armed) for i in range(4)]
    if rbw:
        m["esp_flags_seen"] = sorted({d.get("fl", "") for _, _, d in rbw})
        m["esp_kill_seen"] = sorted({int(d.get("kill", 0)) for _, _, d in rbw})
        m["esp_max_duty"] = [max(d.get("duty%d" % i, 0) for _, _, d in rbw) for i in range(4)]
        m["esp_max_gap_ms"] = max(d.get("gap", 0) for _, _, d in rbw)
        m["esp_cuts_start_end"] = [rbw[0][2].get("cuts"), rbw[-1][2].get("cuts")]
        m["esp_crc_start_end"] = [rbw[0][2].get("crc"), rbw[-1][2].get("crc")]
    if t_estop is not None and rbw:
        # The ESP latches kill=1 the moment a dashboard ESTOP arrives; the FC's st ESTOP bit
        # follows one telemetry line later. So kill=1 at or before (t_estop + 0.2) = dashboard.
        near = [d for ts, _, d in rbw if t_estop - 1.0 <= ts <= t_estop + 0.2]
        m["estop_source_hint"] = ("ESP kill latch (dashboard ESTOP)"
                                  if any(int(d.get("kill", 0)) for d in near)
                                  else "FC-side ESTOP (ESP kill=0), e.g. tilt guard")
    hts = [ts for ts, _, _ in fcw]
    gaps = [b - a for a, b in zip(hts, hts[1:])]
    m["fc_lines"] = len(fcw)
    m["rblink_lines"] = len(rbw)
    m["fc_max_host_gap_ms"] = round(max(gaps) * 1000) if gaps else None
    return m


def _write_csv(path: Path, cols: list[str], rows: list[Row], t0: float) -> None:
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for ts, _, d in rows:
            d = dict(d, t=round(ts - t0, 3))
            w.writerow(["" if d.get(c) is None else d.get(c) for c in cols])


# --------------------------------------------------------------------------- repo helpers

def latest_raw(raw_dir: Path = DEFAULT_RAW_DIR) -> Path:
    """The newest raw_*.log by mtime: the session the operator just flew."""
    logs = sorted(Path(raw_dir).glob("raw_*.log"), key=lambda p: p.stat().st_mtime)
    if not logs:
        raise FlightLogError(f"no raw_*.log in {raw_dir}")
    return logs[-1]


def _trial_dirs(repo: Path) -> list[Path]:
    trials = Path(repo) / "trials"
    if not trials.is_dir():
        return []
    return sorted(p for p in trials.iterdir() if p.is_dir() and TRIAL_NUMBER.search(p.name))


def _number(trial: Path) -> int:
    m = TRIAL_NUMBER.search(trial.name)
    if not m:
        raise FlightLogError(f"{trial.name} is not a <date>_T<nn>_<HHMMSS> trial folder")
    return int(m.group(1))


def resolve_trial(repo: Path, name: str) -> Path:
    """Accept "T27", "27", a full folder name, or a path to a trial folder. Trial numbers
    are unique across dates in this repo, so the number alone is the natural handle."""
    direct = Path(repo) / "trials" / name
    if name and direct.is_dir():
        return direct
    if name and Path(name).is_dir() and (Path(name) / "fc.csv").exists():
        return Path(name)
    key = name[1:] if name[:1] in ("T", "t") else name
    if not key.isdigit():
        raise FlightLogError(f"no trial {name!r} in {repo}/trials (use T27, 27 or a folder name)")
    hits = [p for p in _trial_dirs(repo) if _number(p) == int(key)]
    if not hits:
        raise FlightLogError(f"no trial T{int(key):02d} in {repo}/trials")
    if len(hits) > 1:
        raise FlightLogError(f"T{int(key):02d} is ambiguous: " + ", ".join(p.name for p in hits))
    return hits[0]


def next_trial_number(repo: Path) -> int:
    """1 + the highest T number already published, so numbering never restarts per session."""
    return max((_number(p) for p in _trial_dirs(repo)), default=0) + 1


def _arming_times(repo: Path) -> dict[Path, float]:
    """Published trials by ARMING host time. Folders without metrics.json (hand-built early
    trials) cannot be matched and are simply skipped."""
    out: dict[Path, float] = {}
    for d in _trial_dirs(repo):
        try:
            m = json.loads((d / "metrics.json").read_text())
            out[d] = datetime.datetime.fromisoformat(m["arming_host_time"]).timestamp()
        except (OSError, ValueError, KeyError, TypeError):
            continue
    return out


# --------------------------------------------------------------------------- split

def split(raw: Path, date: str, repo: Path, first: int | None = None) -> list[tuple[Path, bool]]:
    """Cut `raw` into trial folders under repo/trials. Returns (folder, created) in time order.

    A trial starts where st gains ARMING, ends 8 s after the drone last had ARMED or ARMING
    (capped at 60 s and at the next trial's start), with a 10 s pre-roll that never overlaps
    the previous trial. A start whose ARMING time matches an already published trial
    (within MATCH_S) rewrites that folder's data files and leaves summary.md alone, because
    the same growing log is pushed several times per session and hand notes must survive.
    """
    raw = Path(raw)
    lines = _lines(raw)
    fc, rb = _parse(lines)
    starts = _find_starts(fc)
    known = _arming_times(repo)
    claimed: set[Path] = set()
    number = first if first is not None else next_trial_number(repo)
    out: list[tuple[Path, bool]] = []
    prev_end: float | None = None
    stamped = []
    for ln in lines:
        try:
            stamped.append((float(ln.split(" ", 1)[0]), ln))
        except ValueError:
            continue

    for i, t0 in enumerate(starts):
        nxt = starts[i + 1] if i + 1 < len(starts) else None
        t_end = _trial_end(fc, t0)
        if nxt is not None:
            t_end = min(t_end, nxt - 0.001)
        lo = t0 - 10 if prev_end is None else max(t0 - 10, prev_end + 0.001)
        prev_end = t_end

        match = min(((abs(ts - t0), d) for d, ts in known.items()
                     if d not in claimed and abs(ts - t0) <= MATCH_S), default=None)
        if match is not None:
            d, created = match[1], False
            claimed.add(d)
        else:
            hhmmss = datetime.datetime.fromtimestamp(t0).strftime("%H%M%S")
            d = Path(repo) / "trials" / ("%s_T%02d_%s" % (date, number, hhmmss))
            number += 1
            created = True
        d.mkdir(parents=True, exist_ok=True)

        with open(d / "raw.log", "w") as fh:
            fh.writelines(ln for ts, ln in stamped if lo <= ts <= t_end)
        fcw = [r for r in fc if lo <= r[0] <= t_end]
        rbw = [r for r in rb if lo <= r[0] <= t_end]
        _write_csv(d / "fc.csv", FC_COLS, fcw, t0)
        _write_csv(d / "rblink.csv", RB_COLS, rbw, t0)
        m = _metrics(fc, fcw, rbw, t0)
        m["trial"] = d.name
        m["arming_host_time"] = datetime.datetime.fromtimestamp(t0).isoformat(timespec="milliseconds")
        m["source_log"] = raw.name
        with open(d / "metrics.json", "w") as fh:
            json.dump(m, fh, indent=2)
        out.append((d, created))
    return out


# --------------------------------------------------------------------------- summarize

def _f(v: Any) -> float:
    return float(v) if v not in ("", None) else float("nan")


def _deg(g: Any) -> float:
    return math.degrees(2 * math.atan(_f(g)))


def _st(r: dict[str, str]) -> int:
    v = _f(r.get("st"))
    return 0 if math.isnan(v) else int(v)


def _read(trial: Path) -> tuple[list[dict[str, str]], list[dict[str, str]], dict[str, Any]]:
    trial = Path(trial)
    try:
        with open(trial / "fc.csv", newline="") as fh:
            fc = list(csv.DictReader(fh))
        m = json.loads((trial / "metrics.json").read_text())
    except OSError as exc:
        raise FlightLogError(f"{trial.name}: {exc.strerror}: {exc.filename}") from exc
    rb: list[dict[str, str]] = []
    if (trial / "rblink.csv").exists():
        with open(trial / "rblink.csv", newline="") as fh:
            rb = list(csv.DictReader(fh))
    return fc, rb, m


def _airborne(fc: list[dict[str, str]]) -> list[dict[str, str]]:
    # Airborne = armed, not ESTOP, ToF above 2 cm.
    return [r for r in fc if _st(r) & ARMED and not _st(r) & ESTOP and _f(r["tofh"]) > 0.02]


def _flight_line(fc: list[dict[str, str]], m: dict[str, Any]) -> str:
    air = _airborne(fc)
    if not air:
        return ("never airborne (tofh never > 2 cm while armed); armed %s s; flags %s; pre-arm vbat %s"
                % (m.get("armed_s_before_estop_or_end"), m.get("esp_flags_seen"), m.get("pre_arm_vbat")))
    n = len(air)
    t0, t1 = _f(air[0]["t"]), _f(air[-1]["t"])
    hov = [r for r in air if abs(_f(r["z"]) - _f(r["zsp"])) < 0.05 and _f(r["zsp"]) > 0.15]
    rolls = [_deg(r["roll_g"]) for r in air]
    pitches = [_deg(r["pitch_g"]) for r in air]
    duty = [sum(_f(r["duty%d" % i]) for r in air) / n for i in range(4)]
    return (
        "airborne %.1f s (t %.2f..%.2f); z max %.2f m (zsp max %.2f); max tilt %s deg; "
        "roll %.1f..%.1f (mean %.1f), pitch %.1f..%.1f (mean %.1f) deg; "
        "drift x %+.2f m, y %+.2f m (|vx| max %.2f, |vy| max %.2f m/s); mean duty %.3f %.3f %.3f %.3f; "
        "vbat %.2f -> %.2f; within 5 cm of zsp for %.1f s; ESTOP: %s"
        % (t1 - t0, t0, t1, max(_f(r["z"]) for r in air), max(_f(r["zsp"]) for r in air),
           m.get("max_tilt_est_deg_armed"), min(rolls), max(rolls), sum(rolls) / n,
           min(pitches), max(pitches), sum(pitches) / n,
           _f(air[-1]["x"]) - _f(air[0]["x"]), _f(air[-1]["y"]) - _f(air[0]["y"]),
           max(abs(_f(r["vx"])) for r in air), max(abs(_f(r["vy"])) for r in air),
           duty[0], duty[1], duty[2], duty[3],
           _f(m.get("pre_arm_vbat")), _f(m.get("min_vbat_armed")),
           (_f(hov[-1]["t"]) - _f(hov[0]["t"])) if hov else 0,
           m.get("estop_source_hint") or "none recorded"))


def summarize(trial: Path, repo: Path, build: str = "", tag: str = "", note: str = "") -> str:
    """Write summary.md for one trial and add it to README.md and index.csv; return its line.

    summary.md is overwritten (it is the generated starting point for hand notes, so push()
    only calls this for new trials). The README row and index.csv line are added only if the
    trial is not listed yet, so re-running never duplicates them.
    """
    trial, repo = Path(trial), Path(repo)
    name = trial.name
    fc, _, m = _read(trial)
    line = _flight_line(fc, m)
    body = "# %s\n\n" % name
    if note:
        body += "Operator: \"%s\".\n\n" % note
    body += "- %s\n- spin-up duty %s\n- ESP flags seen: %s; ESP kill seen: %s\n" % (
        line, m.get("spinup_duty"), m.get("esp_flags_seen"), m.get("esp_kill_seen"))
    if build:
        body += "\n## Build (reported by the controller session)\n%s\n" % build
    (trial / "summary.md").write_text(body)

    short = line.split("; mean duty")[0]
    label = " ".join(x for x in ((("**%s**." % tag) if tag else ""),
                                 (("operator: %s." % note) if note else ""), short) if x)
    readme_path = repo / "README.md"
    readme = readme_path.read_text()
    if "trials/%s/" % name not in readme:
        i = readme.find(README_ANCHOR)
        if i < 0:
            raise FlightLogError(f"{readme_path} has no {README_ANCHOR.strip()!r} line to insert before")
        row = "| [%s](trials/%s/summary.md) | %s | %s | raw.log, fc.csv, rblink.csv, metrics.json |" % (
            name, name, m["arming_host_time"][11:19], label)
        readme_path.write_text(readme[:i] + "\n" + row + readme[i:])
    index_path = repo / "index.csv"
    index = index_path.read_text() if index_path.exists() else ""
    if not any(ln.startswith(name + ",") for ln in index.splitlines()):
        notes = "; ".join(x for x in (tag, ("operator: " + note) if note else "",
                                      line.split(";")[0]) if x).replace(",", " ")
        with open(index_path, "a") as fh:
            if index and not index.endswith("\n"):
                fh.write("\n")
            fh.write("%s,%s,%s,%s,%s,%s,yes,%s\n" % (
                name, m["arming_host_time"], m.get("armed_s_before_estop_or_end"),
                m.get("max_tilt_est_deg_armed"), m.get("max_tofh_armed_m"),
                m.get("estop_source_hint") or "none recorded", notes))
    return line


# --------------------------------------------------------------------------- push

def _git(run: Callable[..., Any], repo: Path, *args: str) -> str:
    cmd = ["git", "-C", str(repo), *args]
    result = run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        err = (result.stderr or result.stdout or "").strip()
        raise FlightLogError(f"git {' '.join(args)} failed in {repo}: {err}")
    return result.stdout or ""


def _raw_date(raw: Path) -> str:
    m = RAW_DATE.search(raw.name)
    if not m:
        raise FlightLogError(f"cannot read a date from {raw.name}; pass date=YYYY-MM-DD")
    return "%s-%s-%s" % m.groups()


def push(repo: Path = DEFAULT_REPO, raw: Path | None = None, date: str | None = None,
         build: str = "", tag: str = "", note: str = "", dry_run: bool = False,
         run: Callable[..., Any] = subprocess.run) -> list[Path]:
    """Publish a session: pull, copy the raw log, split, summarize new trials, commit, push.

    Safe to repeat as the raw log grows: known trials keep their number and summary, and
    nothing is committed when the data did not change. dry_run skips pull, commit and push
    (and staging), leaving the repo's working tree updated for inspection.
    Returns the newly created trial folders.
    """
    repo = Path(repo)
    if not repo.is_dir():
        raise FlightLogError(f"no flight-logs checkout at {repo} (set RB_FLIGHT_LOGS or clone it there)")
    if not dry_run:
        _git(run, repo, "pull", "--ff-only")
    raw = Path(raw) if raw is not None else latest_raw()
    if not raw.is_file():
        raise FlightLogError(f"no raw log {raw}")
    date = date or _raw_date(raw)

    dest = repo / "sessions" / date / "raw"
    dest.mkdir(parents=True, exist_ok=True)
    try:
        shutil.copyfile(raw, dest / raw.name)
    except shutil.SameFileError:
        pass

    results = split(raw, date, repo)
    new = [d for d, created in results if created]
    for d in new:
        summarize(d, repo, build=build, tag=tag, note=note)

    if not _git(run, repo, "status", "--porcelain").strip():
        return []
    if dry_run:
        return new
    nums = [_number(d) for d in new]
    if not nums:
        what = "refresh"
    elif len(nums) == 1:
        what = "T%02d" % nums[0]
    else:
        what = "T%02d-T%02d" % (min(nums), max(nums))
    _git(run, repo, "add", "-A")
    _git(run, repo, "commit", "-m", f"Flight logs {date}: {what}")
    _git(run, repo, "push")
    return new


# --------------------------------------------------------------------------- reading

def _table(headers: list[str], rows: list[list[str]], left: tuple[int, ...] = ()) -> str:
    widths = [max([len(h)] + [len(r[i]) for r in rows]) for i, h in enumerate(headers)]

    def fmt(cells: list[str]) -> str:
        out = []
        for i, c in enumerate(cells):
            last = i == len(cells) - 1
            if i in left:
                out.append(c if last else c.ljust(widths[i]))
            else:
                out.append(c.rjust(widths[i]))
        return "  ".join(out).rstrip()

    return "\n".join([fmt(headers), fmt(["-" * w for w in widths])] + [fmt(r) for r in rows])


def _num(v: Any, spec: str) -> str:
    x = _f(v)
    return "-" if math.isnan(x) else format(x, spec)


def show(trial: Path, step: float = 0.2, t_from: float | None = None,
         t_to: float | None = None) -> str:
    """A timeline of one trial, one row per `step` seconds of t (seconds from ARMING).

    Rows are the nearest FC line to each grid time, joined with the nearest RBLINK line so the
    ESP's view (frame flags, kill latch) sits next to the FC's. The default window is the part
    worth reading: 0.3 s before the first ARMED line to 1 s after the flight ended (the first
    ESTOP line after ARMED, else the last ARMED line).
    """
    trial = Path(trial)
    if step <= 0:
        raise FlightLogError("step must be positive")
    fc, rb, _ = _read(trial)
    fc = [r for r in fc if not math.isnan(_f(r.get("t")))]
    if not fc:
        raise FlightLogError(f"{trial.name}: fc.csv has no rows")
    ts = [_f(r["t"]) for r in fc]
    armed = [t for t, r in zip(ts, fc) if _st(r) & ARMED]
    if armed:
        # ESTOP is latched (st 11 then 10 for the rest of the log), so "the last ESTOP row" would
        # be the end of the file. The flight ends at the first ESTOP after ARMED, or at the last
        # ARMED row if it never ESTOPped.
        estop = next((t for t, r in zip(ts, fc) if t >= armed[0] and _st(r) & ESTOP), None)
        lo, hi = armed[0] - 0.3, (estop if estop is not None else armed[-1]) + 1.0
    else:
        # Never armed: show the RESET attempt itself, ARMING until it gave up.
        arming = [t for t, r in zip(ts, fc) if _st(r) & ARMING]
        lo, hi = (min(0.0, arming[0]) - 0.3, arming[-1] + 1.0) if arming else (ts[0], ts[-1])
    lo = lo if t_from is None else t_from
    hi = hi if t_to is None else t_to
    rb = [r for r in rb if not math.isnan(_f(r.get("t")))]
    rts = [_f(r["t"]) for r in rb]

    def nearest(times: list[float], t: float) -> int:
        i = bisect.bisect_left(times, t)
        if i == 0:
            return 0
        if i == len(times):
            return len(times) - 1
        return i if times[i] - t < t - times[i - 1] else i - 1

    rows: list[list[str]] = []
    n = 0
    last = -1
    while True:
        t = lo + n * step
        if t > hi + 1e-9:
            break
        n += 1
        i = nearest(ts, t)
        if i == last or abs(ts[i] - t) > max(step, 0.25):
            continue   # no FC line near this grid time (telemetry gap or outside the data)
        last = i
        r = fc[i]
        e = rb[nearest(rts, ts[i])] if rb else {}
        rows.append([
            "%.2f" % ts[i], str(_st(r)), _num(r.get("zsp"), ".2f"), _num(r.get("z"), ".2f"),
            _num(r.get("tofh"), ".3f"), _num(r.get("vz"), "+.2f"), _num(r.get("tilt_est"), ".0f"),
            "%+.1f" % _deg(r.get("roll_g")), "%+.1f" % _deg(r.get("pitch_g")),
            "%+.1f" % _deg(r.get("yaw_g")),
            *(_num(r.get("duty%d" % k), ".3f") for k in range(4)),
            _num(r.get("vbat"), ".3f"), e.get("fl", "") or "-", _num(e.get("kill"), ".0f"),
        ])
    headers = ["t", "st", "zsp", "z", "tofh", "vz", "tilt", "roll", "pitch", "yaw",
               "d0", "d1", "d2", "d3", "vbat", "esp fl", "kill"]
    title = "%s  t %.2f..%.2f s every %.2f s  (st 1 ARMED 2 ESTOP 4 ARMING 8 CALDONE; angles deg)" % (
        trial.name, lo, hi, step)
    if not rows:
        return title + "\n(no FC lines in this window)"
    return title + "\n" + _table(headers, rows, left=(15,))


def compare(trials: list[Path]) -> str:
    """One row per trial with the numbers that separate a good flight from a bad one.

    The yaw-pair differential |(d0+d2)-(d1+d3)| is there because every upset so far (T02,
    T10, T27, T31) showed the diagonal pair saturating; drift and mean roll/pitch expose trim.
    """
    headers = ["trial", "airborne", "z max", "zsp max", "tilt max", "roll", "pitch",
               "drift x", "drift y", "|vx|", "|vy|", "yaw diff", "vbat", "estop"]
    rows: list[list[str]] = []
    for trial in trials:
        trial = Path(trial)
        fc, _, m = _read(trial)
        label = "T%02d %s" % (_number(trial), trial.name[:10]) if TRIAL_NUMBER.search(trial.name) \
            else trial.name
        air = _airborne(fc)
        if not air:
            rows.append([label, "not airborne"] + ["-"] * (len(headers) - 2))
            continue
        armed = [r for r in fc if _st(r) & ARMED and not _st(r) & ESTOP]
        n = len(air)
        tilt = max((_f(r["tilt_est"]) for r in armed if not math.isnan(_f(r["tilt_est"]))), default=float("nan"))
        ydiff = max((abs(_f(r["duty0"]) + _f(r["duty2"]) - _f(r["duty1"]) - _f(r["duty3"]))
                     for r in armed), default=float("nan"))
        vmin = _f(m.get("min_vbat_armed"))
        vpre = _f(m.get("pre_arm_vbat"))
        rows.append([
            label,
            "%.1f s" % (_f(air[-1]["t"]) - _f(air[0]["t"])),
            "%.2f" % max(_f(r["z"]) for r in air),
            "%.2f" % max(_f(r["zsp"]) for r in air),
            "-" if math.isnan(tilt) else "%.0f" % tilt,
            "%+.1f" % (sum(_deg(r["roll_g"]) for r in air) / n),
            "%+.1f" % (sum(_deg(r["pitch_g"]) for r in air) / n),
            "%+.2f" % (_f(air[-1]["x"]) - _f(air[0]["x"])),
            "%+.2f" % (_f(air[-1]["y"]) - _f(air[0]["y"])),
            "%.2f" % max(abs(_f(r["vx"])) for r in air),
            "%.2f" % max(abs(_f(r["vy"])) for r in air),
            "-" if math.isnan(ydiff) else "%.2f" % ydiff,
            "%s->%s" % (_num(vpre, ".2f"), _num(vmin, ".2f")),
            m.get("estop_source_hint") or "none recorded",
        ])
    note = ("airborne = ARMED, not ESTOP, tofh > 2 cm; z/roll/pitch/drift/|v| over airborne rows; "
            "tilt max and yaw diff |(d0+d2)-(d1+d3)| over armed rows; roll/pitch are means (deg)")
    return _table(headers, rows, left=(0, 13)) + "\n" + note
