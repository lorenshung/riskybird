#!/usr/bin/env python3
"""
Roll-harness twin driver: builds tools/rig/sim/rig_twin.cpp against the REAL flight-controller rig
code, runs scenarios, sweeps gains across a plant-uncertainty box and does the linearised margins.

    PY=backends/zephyr-chipyard-sw/tools/miniforge3/envs/zephyr-new/bin/python   # numpy + scipy
    $PY tools/rig/sim/rig_sim.py mixer-check          # twin mixer == controller_pid.cpp, bit for bit
    $PY tools/rig/sim/rig_sim.py run --csv out.csv [--set I=3e-5 ...] [--gain RATE_P=30]
    $PY tools/rig/sim/rig_sim.py run --axis pitch --coll 600 --step-mdeg 52 --csv out.csv
    $PY tools/rig/sim/rig_sim.py margins [--coll 1000]
    $PY tools/rig/sim/rig_sim.py map [--coll 1000] [--grid quick|full]
    $PY tools/rig/sim/rig_sim.py liftoff
    $PY tools/rig/sim/rig_sim.py liftoff-assist [--axis pitch]   # LIFT_I / LIFT_I_LIM vs a steady torque to the stop
    $PY tools/rig/sim/rig_sim.py report               # everything above -> results/rig-sim/<stamp>/

`run` writes rec.csv in EXACTLY the recorder's rig columns (flightrec_fetch.RIG_COLUMNS), at
every 4th loop (FLIGHTREC_DIV 4: ~89 Hz at the measured ~355 Hz loop, whose dt_us sequence
is replayed from a real capture, see DT_CSV), so tools/rb/rig_analyze.py reads it unchanged; the
metrics here use rig_analyze.trial_metrics when it imports and fall back to an equivalent local
implementation (same keys) when it does not.

THE TWIN (rig_twin.cpp). FC side, all REAL code: fc_rig.cpp (verbs, state machine, excitations,
roll law), fc_shared_math.h (fc_actuator_duty, dwell/guard helpers), compiled with the rig-roll
mode's -D values read from tools/rb/boards.py RIG_MODES. controller_pid.cpp's mixer is
transcribed (rig_mixer.h; equality asserted by mixer_check.cpp against the real file under
-DROSE_RIG=1). main.cpp is emulated: tick order, RESET/ESTOP/DISARM, the rig envelope guard
(rate 10 rad/s, impact 3 g/40 ms, battery 3.2 V), the battery EMA + sag compensation, 200 Hz
send_control frames (immediately on a flag change) and the recorder's held, as-sent duty.
PLANT: theta'' = [arm*(T_left - T_right) - m g (h sin th + y cos th) - c w - tau_c tanh(w/0.01)] / I
with I = I_cm (1 + cradle) + m (h^2 + y^2); per-motor first-order thrust lag towards
thrust_scale * (1 +- imbalance/2) * Bitcraze45-35(duty * V/v_ref) * 1.33; inelastic stops with a
restitution; gyro white noise + bias; frames applied delay_ms after they are sent.

WHERE THE DEFAULTS COME FROM (/scratch2/loren/sim-keep/evidence/plant-model-EVIDENCE.txt, sec. 3):
  v_ref 3.37 V: the evidence model's out-of-ground-effect hover duties for 55 g (0.779/0.757/0.736/
    0.716 at 3.2/3.3/3.4/3.5 V) all map to the same curve point d*V/3.37 = 0.74 (+-0.002) with
    thrust_scale 1. So thrust_scale 1 = the flight-calibrated thrust. On the stiff 4.2 V PSU the
    small-signal actuator gain d(true moment)/d(commanded u[1]) is ~1.26 (0.9 * 4.2/3.37 *
    curve-slope ratio, actuator_gain()), vs ~1.08 in flight on a 3.3 V pack with sag compensation:
    the PSU rig runs ~17 % more roll-loop gain than the flown gains saw (COLL 1000 = ~1.10x weight).
  LiPo on the rig: rig-roll keeps BATT_CUTOFF_V 3.2 (t21 flew 3.0); the twin's pack sags ~0.7 V at
    ~3.5 A, so a pack below ~3.9 V OCV trips the battery guard at COLL 1000 (3.7 V: also at 800).
  tau: the evidence priors (J_r 4e-8, k_e 6.5e-4, R_m 0.663 + D*0.07, k_Q/k_T 0.005 m, k_T 2.2e-8
    at 55 g) give a hover spin-up tau of ~35 ms (electrical 5.9e-7 + aero 5.4e-7 N m s damping,
    aero share 0.48) and a no-regeneration coast-down of ~73 ms; both slow down at low thrust
    (tau_speed=1). Default 70 ms (the brief's prior); box 35-100 ms.
  LiPo: R_0 0.180 ohm, R_p 0.049 ohm, tau_p 8.9 s, I_base 0.36 A, hover battery current 3.5-4.0 A;
    FC vbat = EMA(alpha 0.2) every BATT_CHECK_DIV = 200 iterations (evidence 2a).
PITCH RIG (twin axis=1, `run --axis pitch`): the drone turned 90 deg on the pivot (rig-pitch).
theta is pitch, I is Iyy (the same prior: controller_pid.cpp's J = diag(16, 16, 29)e-6 has
Iyy = Ixx), the moment is M's pitch column through the real permutation (rear pair duty[1]/duty[2]
minus front duty[0]/duty[3]), gravity uses h_mm and cg_x_mm (CoM forward = nose-down torque),
imbalance splits rear/front, and ext_mNm (the USB cable) defaults to 0: it now runs along the pivot.
The FC side is the real pitch build: fc_rig.cpp compiled with rig-pitch's defines (RIG_AXIS=1),
fed the physical roll/pitch/gyro (rig_twin.cpp TWIN_FW_AXIS_AWARE). build() makes one twin per
axis, rig_twin (rig-roll) and rig_twin_pitch (rig-pitch), and run_twin / run_batch pick the one
matching the run's `axis` parameter (twin_for). The CSV's ang_deg / ang_tgt_deg / ang_rate_dps
are the controlled axis either way.
NOT MODELLED: the attitude estimator's dynamics (roll = truth + bias/noise; on a pivot the angular
acceleration also corrupts an accelerometer tilt reference), rotor gyroscopics, the brushed motors'
low-duty dead band, the stop's compliance (so the impact guard's 40 ms dwell is not meaningful).
"""

from __future__ import annotations

import argparse
import ast
import concurrent.futures as cf
import csv
import hashlib
import itertools
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SRC = ROOT / "workloads/flight_controller/src"
RB = ROOT / "tools/rb"

# The rig-roll mode's compile definitions; rig_defines() reads them from tools/rb/boards.py and
# tests/test_rig_sim.py asserts this fallback still matches it on TWIN_KEYS, the values that change
# the twin's behaviour. rig-pitch is the same plus RIG_AXIS=1 (RIG_PITCH_EXTRA; checked the same way).
RIG_DEFINES_FALLBACK = {
    "ROSE_RIG": "1", "ROSE_AUTOFLIGHT": "1", "ROSE_UART_CMD": "1", "ROSE_FLIGHT_COMMIT": "1",
    "ROSE_COMMIT_MAX_MS": "900000", "ROSE_ARM_NO_GESTURE": "1", "PID_MASS_KG": "0.050f",
    "AUTOFLIGHT_MAX_DUTY": "0.95f", "ROSE_BATT_REPORT_ONLY": "0", "ROSE_TELEM_DIV": "50",
    "ROSE_FLIGHTREC": "1", "ROSE_FLOW": "0", "ROSE_VEL_LOOP": "0", "ROSE_TELEM_CONSOLE": "0",
    "FLIGHTREC_DIV": "4", "ATT_GAIN": "0.85f", "MOTOR_TRIM_LEFT": "1.0f", "IMU_OFFSET_Y": "0.016f",
    "RIG_MAX_TILT_RAD": "1.05f", "RIG_TGT_MAX_RAD": "0.35f", "BATT_CUTOFF_V": "3.0f",
    "RIG_LIFTOFF_TIMEOUT_MS": "8000", "ROSE_BATT_SAG_COMP": "0",
}

RIG_PITCH_EXTRA = {"RIG_AXIS": "1"}
RIG_MODE_BY_AXIS = {0: "rig-roll", 1: "rig-pitch"}

TWIN_KEYS = ("ROSE_RIG", "ROSE_VEL_LOOP", "PID_MASS_KG", "ATT_GAIN", "MOTOR_TRIM_LEFT",
             "AUTOFLIGHT_MAX_DUTY", "FLIGHTREC_DIV", "ROSE_BATT_REPORT_ONLY", "TAU_ROLL",
             "TAU_ROLLRATE", "ROLL_TRIM_RAD", "PITCH_TRIM_RAD", "BATT_CUTOFF_V", "BATT_NOMINAL_V",
             "SAFE_MAX_RATE_RADPS", "RIG_SP_SLEW_RADPS", "RIG_RAMP_MS", "RIG_MAX_TILT_RAD",
             "RIG_TGT_MAX_RAD", "RIG_LIFTOFF_TIMEOUT_MS", "RIG_AXIS")

# controller_pid.cpp / fc_rig.cpp constants the linear model needs
J0 = 16e-6
L_ARM_FC = 0.033
THRUST_A, THRUST_B, PROP_GAIN, GPN = 26.919633, 35.754861, 1.33, 101.9368
G = 9.81

FLOWN = {"ATT_P": 5.0, "ATT_I": 0.0, "RATE_P": 40.0, "RATE_I": 0.0, "RATE_D": 0.0,
         "ATT_GAIN": 0.85, "D_LPF_HZ": 30.0, "GYRO_LPF_HZ": 0.0}

# Plant nominal = rig_twin.cpp's defaults (twin parameters), and the uncertainty box: corners + centre.
NOMINAL = {"mass": 0.055, "arm": 0.033, "I": 2.5e-5, "cradle_frac": 0.0, "h_mm": 1.0,
           "tau_up": 0.07, "tau_down": 0.07, "delay_ms": 2.0, "frame_ms": 5.0, "viscous": 2e-6,
           "coulomb": 5e-6, "thrust_scale": 1.0, "imbalance": 0.0, "v_ref": 3.37, "v_psu": 4.2,
           "lipo": 0.0, "ocv": 3.9, "loop_hz": 355.0, "tau_speed": 0.0}
# NOTE: keep NOMINAL equal to rig_twin.cpp's PARAMS defaults (tests/test_rig_sim.py checks it).
BOX = {  # name -> (lo, hi); "tau" sets tau_up and tau_down together
    "I": (1.6e-5, 4.4e-5),       # code J .. 4e-5 + 10 % cradle
    "tau": (0.035, 0.10),
    "delay_ms": (1.0, 5.0),      # + the 5 ms frame hold: 3.5..7.5 ms mean, 6..10 ms worst
    "h_mm": (-2.0, 5.0),
    "thrust_scale": (0.7, 1.3),
    "imbalance": (-0.05, 0.05),
}


# =============================================================================================
# Build
# =============================================================================================

def rig_defines(axis: int = 0) -> dict:
    """The rig-roll (axis 0) or rig-pitch (axis 1) mode's -D values from tools/rb/boards.py (CMake +
    preprocessor tuples)."""
    try:
        sys.path.insert(0, str(ROOT))
        from tools.rb import boards  # noqa: PLC0415
        mode = next(m for m in boards.RIG_MODES if m[0] == RIG_MODE_BY_AXIS[axis])
        # CONFIG_* entries are Kconfig, not preprocessor macros: not for the host compile
        return {k: v for k, v in (*mode[2], *mode[3]) if not k.startswith("CONFIG_")}
    except Exception:  # noqa: BLE001 - boards.py mid-edit elsewhere: the asserted fallback
        return dict(RIG_DEFINES_FALLBACK, **(RIG_PITCH_EXTRA if axis else {}))
    finally:
        if sys.path and sys.path[0] == str(ROOT):
            sys.path.pop(0)


def _compiler() -> str | None:
    return shutil.which("g++") or shutil.which("c++")


def build(extra: dict | None = None, out_dir: str | os.PathLike | None = None,
          axes: tuple | None = None) -> dict:
    """Compile rig_twin and mixer_check per axis (cached by source+flag hash): axis 0 with the
    rig-roll mode's defines -> {twin, mixer_check}, axis 1 with rig-pitch's (RIG_AXIS=1) ->
    {twin_pitch, mixer_check_pitch}. Both by default; only roll when extra sets
    TWIN_FW_AXIS_AWARE=0 (the transcription, which remaps axis=1 onto a roll build)."""
    cxx = _compiler()
    if cxx is None:
        raise RuntimeError("no host C++ compiler (g++)")
    aware = str((extra or {}).get("TWIN_FW_AXIS_AWARE", "1")) != "0"
    if axes is None:
        axes = (0, 1) if aware else (0,)
    if not aware and 1 in axes:
        raise ValueError("TWIN_FW_AXIS_AWARE=0 builds the roll twin only (axis=1 is remapped onto it)")
    flags_by_axis = {}
    for axis in axes:
        defs = rig_defines(axis)
        defs.update(extra or {})
        flags_by_axis[axis] = ["-std=c++17", "-O2", "-Wall", "-Wextra", "-ffp-contract=off",
                               *[f"-D{k}={v}" for k, v in sorted(defs.items())], f"-I{SRC}", f"-I{HERE}"]
    srcs = [HERE / "rig_twin.cpp", HERE / "mixer_check.cpp", HERE / "rig_mixer.h",
            SRC / "fc_rig.cpp", SRC / "fc_rig.h", SRC / "fc_shared_math.h",
            SRC / "controller_pid.cpp", SRC / "controller_pid.hpp"]
    h = hashlib.sha1(" ".join([cxx, *(f for a in axes for f in [str(a), *flags_by_axis[a]])]).encode())
    for s in srcs:
        h.update(s.read_bytes())
    d = Path(out_dir) if out_dir else Path(tempfile.gettempdir()) / f"rig_twin-{h.hexdigest()[:12]}"
    d.mkdir(parents=True, exist_ok=True)
    out = {}
    jobs = []
    for axis in axes:
        sfx = "_pitch" if axis else ""
        out["twin" + sfx] = d / ("rig_twin" + sfx)
        out["mixer_check" + sfx] = d / ("mixer_check" + sfx)
        jobs += [
            (out["twin" + sfx], flags_by_axis[axis], [HERE / "rig_twin.cpp", SRC / "fc_rig.cpp"]),
            (out["mixer_check" + sfx], flags_by_axis[axis],
             [HERE / "mixer_check.cpp", SRC / "fc_rig.cpp", SRC / "controller_pid.cpp"]),
        ]
    for exe, flags, files in jobs:
        if exe.exists():
            continue
        tmp = exe.with_suffix(f".tmp{os.getpid()}")
        r = subprocess.run([cxx, *flags, *map(str, files), "-o", str(tmp), "-lm"],
                           capture_output=True, text=True)
        if r.returncode != 0:
            raise RuntimeError(f"build of {exe.name} failed:\n{r.stderr}")
        os.replace(tmp, exe)
    return out


def _axis_of(params: dict | None):
    """The run's `axis` parameter as 0/1, None when it does not set one."""
    v = (params or {}).get("axis")
    if v is None:
        return None
    return 1 if float(v) != 0.0 else 0


def twin_for(twin, axis):
    """The twin binary for `axis` (0 roll, 1 pitch). Each binary is ONE firmware axis (fc_rig.cpp's
    RIG_AXIS) and refuses the other: rig_twin is rig-roll, rig_twin_pitch (next to it) rig-pitch.
    `twin` may be either path or build()'s dict; axis None keeps it (the binary's own axis)."""
    if isinstance(twin, dict):
        return Path(twin["twin_pitch" if axis == 1 else "twin"])
    p = Path(twin)
    if axis is None:
        return p
    base = p.name[:-len("_pitch")] if p.name.endswith("_pitch") else p.name
    want = p.with_name(base + ("_pitch" if axis == 1 else ""))
    return want if want.exists() else p


def mixer_check(exes: dict | None = None) -> str:
    """mixer_check of every axis built (roll, and pitch: RIG_AXIS=1, u[2] through M's column 2)."""
    exes = exes or build()
    outs = []
    for key in ("mixer_check", "mixer_check_pitch"):
        if key not in exes:
            continue
        r = subprocess.run([str(exes[key])], capture_output=True, text=True, timeout=120)
        if r.returncode != 0:
            raise AssertionError(r.stdout + r.stderr)
        outs.append(r.stdout.strip())
    return "\n".join(outs)


# =============================================================================================
# Scenarios and runs
# =============================================================================================

def scenario_text(coll_permille: int = 1000, step_mdeg: int = 87, step_ms: int = 1500,
                  hold_ms: int = 2500, after_ms: int = 2500, rsp_mdeg: int = 0,
                  extra: list[str] | None = None) -> str:
    """Arm, ramp, hold, a `step att` (it returns to 0 when it ends), hold, STOP."""
    lines = ["0 RESET", "0 HBON", "0 RUNMS 30000", f"0 RSP {rsp_mdeg}", f"0 COLL {coll_permille}"]
    lines += list(extra or [])
    t = hold_ms
    if step_mdeg:
        lines.append(f"run+{t} EXC step att {step_mdeg} {step_ms}")
        t += step_ms + after_ms
    lines.append(f"run+{t} STOP")
    return "\n".join(lines) + "\n"


def _write_tmp(text: str, d: Path, name: str) -> Path:
    p = d / name
    p.write_text(text)
    return p


# Measured loop periods: dt_us of a real recorder capture (2026-11-04, ~355 Hz, stalls to 18 ms).
# Every twin run replays them (rig_twin dt_file=) unless the run sets dt_file itself; set_dt_csv()
# or --dt-csv picks another capture, --dt-csv none the fixed loop_hz.
DT_CSV = ROOT / "results/rig/20261104-133716-first_hold_250/spins/1/rec.csv"
_DT_FILE: list = [None, False]   # [path, resolved]


def set_dt_csv(path):
    global DT_CSV
    DT_CSV = None if path in (None, "", "none") else Path(path)
    _DT_FILE[:] = [None, False]


def dt_file() -> str | None:
    """DT_CSV's dt_us column as a one-per-line text file (cached), or None if unavailable."""
    if not _DT_FILE[1]:
        _DT_FILE[1] = True
        if DT_CSV is not None and Path(DT_CSV).exists():
            vals = [row["dt_us"] for row in csv.DictReader(open(DT_CSV, newline=""))]
            p = Path(tempfile.gettempdir()) / f"rig_dt-{hashlib.sha1(''.join(vals).encode()).hexdigest()[:10]}.txt"
            p.write_text("\n".join(vals) + "\n")
            _DT_FILE[0] = str(p)
    return _DT_FILE[0]


def _kv(params: dict) -> list[str]:
    out = []
    if "dt_file" not in params and dt_file():
        out.append(f"dt_file={dt_file()}")
    for k, v in params.items():
        if k == "tau":
            out += [f"tau_up={v}", f"tau_down={v}"]
        elif k == "gains":
            out += [f"gain.{n}={g}" for n, g in v.items()]
        else:
            out.append(f"{k}={v}")
    return out


def run_twin(twin, scenario: Path, params: dict | None = None, csv_path=None, truth=None) -> dict:
    """One run. The binary follows params' `axis` (twin_for): rig_twin for roll, rig_twin_pitch for
    pitch."""
    twin = twin_for(twin, _axis_of(params))
    args = [str(twin), f"scenario={scenario}", *_kv(params or {})]
    if csv_path:
        args.append(f"csv={csv_path}")
    if truth:
        args.append(f"truth={truth}")
    r = subprocess.run(args, capture_output=True, text=True, timeout=120)
    if r.returncode != 0:
        raise RuntimeError(r.stdout + r.stderr)
    return json.loads(r.stdout.strip().splitlines()[-1])


def run_batch(twin, scenario: Path, param_list: list[dict], jobs: int | None = None,
              workdir: Path | None = None) -> list[dict]:
    """Many runs of one scenario, split over `jobs` twin processes. Results in input order. All
    runs share one axis (the binary follows it, twin_for)."""
    axes = {_axis_of(p) for p in param_list} - {None}
    if len(axes) > 1:
        raise ValueError("run_batch: one axis per batch (the twin binary is per axis)")
    twin = twin_for(twin, axes.pop() if axes else None)
    jobs = jobs or min(os.cpu_count() or 4, 48)
    tmp = tempfile.TemporaryDirectory(prefix="rig-batch-", dir=workdir)
    chunks = [list(range(i, len(param_list), jobs)) for i in range(min(jobs, len(param_list)))]

    def go(ci):
        idx = chunks[ci]
        bf = Path(tmp.name) / f"b{ci}.txt"
        bf.write_text("".join(" ".join(_kv(param_list[i]) + [f"id={i}"]) + "\n" for i in idx))
        r = subprocess.run([str(twin), f"scenario={scenario}", f"batch={bf}"],
                           capture_output=True, text=True, timeout=3600)
        res = [json.loads(line) for line in r.stdout.splitlines() if line.strip()]
        if r.returncode != 0 or len(res) != len(idx):
            raise RuntimeError(f"batch chunk {ci} failed: {r.stdout[-500:]}{r.stderr[-500:]}")
        return res

    out: list[dict | None] = [None] * len(param_list)
    try:
        with cf.ThreadPoolExecutor(max_workers=len(chunks)) as ex:
            for res in ex.map(go, range(len(chunks))):
                for js in res:
                    out[int(js["id"])] = js
    finally:
        tmp.cleanup()
    return out  # type: ignore[return-value]


def box_points(box: dict = BOX, centre: dict | None = None) -> list[dict]:
    """2^n corners + the centre of the box (each a dict of twin parameters)."""
    names = list(box)
    pts = [dict(zip(names, c)) for c in itertools.product(*[box[n] for n in names])]
    mid = {n: (centre or {}).get(n, 0.5 * (box[n][0] + box[n][1])) for n in names}
    return pts + [mid]


# =============================================================================================
# Metrics (rig_analyze.trial_metrics when importable, else the same definitions here)
# =============================================================================================

JOURNAL_KEYS = ("overshoot_pct", "rise_ms", "settle_ms", "rms_err_deg", "steady_err_deg",
                "peak_duty", "sat_frac", "osc_hz", "osc_amp_deg", "settled", "pm_deg", "gm_db",
                "bw_hz", "margin_penalty", "score")


def load_csv(path) -> dict:
    with open(path, newline="") as f:
        rd = csv.reader(f)
        head = next(rd)
        rows = [r for r in rd if r]
    out = {}
    for i, n in enumerate(head):
        try:
            out[n] = [float(r[i]) for r in rows]
        except ValueError:
            out[n] = [r[i] for r in rows]
    return out


def analyze_csv(path, exc: dict) -> dict:
    """Trial metrics of a twin (or recorder) CSV; `exc` in the batch.json spec shape."""
    try:
        sys.path.insert(0, str(RB))
        import rig_analyze as A  # noqa: PLC0415
        m = A.trial_metrics(A.load_rec(str(path)), exc)
        m.pop("_frf", None)
        m["_by"] = "rig_analyze"
        return m
    except Exception:  # noqa: BLE001 - mid-edit elsewhere or no numpy: local fallback
        m = _local_metrics(load_csv(path), exc)
        m["_by"] = "rig_sim fallback"
        return m
    finally:
        if sys.path and sys.path[0] == str(RB):
            sys.path.pop(0)


def _local_metrics(d: dict, exc: dict) -> dict:
    """rig_analyze's step definitions (att steps; others get rms/sat/peak only), stdlib only."""
    t, x = d["t_s"], d["exc_val"]
    # the controlled axis: ang_* (roll or pitch), roll_* in a CSV from before those columns
    roll, tgt = d.get("ang_deg") or d["roll_deg"], d.get("ang_tgt_deg") or d["roll_tgt_deg"]
    n = len(t)
    i0 = next((i for i, v in enumerate(x) if v != 0), 0)
    err = [a - b for a, b in zip(tgt, roll)]
    m = {k: None for k in JOURNAL_KEYS}
    m.update(type=exc["type"], target=exc["target"], samples=n)
    m["rms_err_deg"] = math.sqrt(sum(e * e for e in err[i0:]) / max(1, n - i0))
    duties = [d[f"duty{i}"][i0:] for i in range(4)]
    m["peak_duty"] = max(max(c) for c in duties)
    sat = [any(c[k] >= 0.94 or c[k] <= 0.01 for c in duties) for k in range(n - i0)]
    m["sat_frac"] = sum(sat) / max(1, len(sat))
    if exc["type"] in ("step", "doublet") and exc["target"] == "att":
        xs = [math.degrees(v) for v in x]
        amp = max(abs(v) for v in xs) or 1.0
        edges = [i for i in range(1, n) if abs(xs[i] - xs[i - 1]) > 0.4 * amp and i >= i0 - 2]
        em = []
        for j, e in enumerate(edges):
            e1 = edges[j + 1] if j + 1 < len(edges) else n
            seg = roll[e:e1]
            if len(seg) < 10:
                continue
            y0, k20 = roll[e - 1], int(0.8 * len(seg))
            yss = sum(seg[k20:]) / len(seg[k20:])
            delta = xs[e] - xs[e - 1]
            sgn = 1 if delta > 0 else -1
            span = abs(yss - y0) or 1e-9
            ov = 100.0 * max(0.0, max(sgn * (y - yss) for y in seg)) / span
            band = max(0.05 * abs(delta), 1.0)
            out = [k for k, y in enumerate(seg) if abs(y - yss) > band]
            settled = not out or out[-1] < len(seg) - 1
            st = 0.0 if not out else ((t[e + out[-1] + 1] - t[e]) * 1000 if settled else None)
            fr = [(y - y0) / (yss - y0) if yss != y0 else 0 for y in seg]
            k10 = next((k for k, f in enumerate(fr) if f >= 0.1), None)
            k90 = next((k for k, f in enumerate(fr) if f >= 0.9), None)
            rise = (t[e + k90] - t[e + k10]) * 1000 if k10 is not None and k90 is not None else None
            em.append(dict(ov=ov, st=st, settled=settled, rise=rise, ss=tgt[e1 - 1] - yss))
        if em:
            m["overshoot_pct"] = max(e["ov"] for e in em)
            m["settled"] = all(e["settled"] for e in em)
            m["settle_ms"] = max(e["st"] for e in em) if m["settled"] else None
            rises = [e["rise"] for e in em if e["rise"] is not None]
            m["rise_ms"] = max(rises) if rises else None
            m["steady_err_deg"] = max((e["ss"] for e in em), key=abs)
    return m


def stable(js: dict) -> bool:
    """Map verdict for one twin run: no estop / ramp-down fault, no stop hit after lift-off, no
    sustained oscillation (sqrt2*std of the roll error over the quiet windows < 1 deg)."""
    return (js["status"] == "ok" and js["stop_hits_live"] == 0
            and (js["tail_osc_deg"] or 0) < 1.0 and (js["quiet_osc_deg"] or 0) < 1.0)


# =============================================================================================
# Linearised loop (numpy / scipy-free polynomial algebra on numpy)
# =============================================================================================

def curve_g(d):
    """Bitcraze 45-35 total grams x PROP_GAIN, and its slope, at duty d."""
    return (THRUST_A * d * d + THRUST_B * d) * PROP_GAIN, (2 * THRUST_A * d + THRUST_B) * PROP_GAIN


def force_to_duty(f_n):
    t = f_n * GPN * 4.0 / PROP_GAIN
    return max(0.0, min(1.0, (-THRUST_B + math.sqrt(THRUST_B ** 2 + 4 * THRUST_A * t)) / (2 * THRUST_A)))


def actuator_gain(plant: dict, coll: float, mass_model: float = 0.050) -> dict:
    """Small-signal d(true roll moment)/d(commanded u[1]) at a collective (fraction of MASS*g):
    0.9 * scale * batt_scale * V/v_ref * curve'(d_eff)/curve'(d0) * arm/0.033."""
    v = plant.get("v_psu", 4.2) if not plant.get("lipo") else plant.get("ocv", 3.9) - 0.6
    bs = min(1.3, max(1.0, 3.8 / v))
    d0 = force_to_duty(0.9 * 0.25 * coll * mass_model * G)
    d = min(0.95, d0 * bs)
    deff = d * v / plant.get("v_ref", 3.37)
    k = 0.9 * plant.get("thrust_scale", 1.0) * bs * v / plant.get("v_ref", 3.37) \
        * curve_g(deff)[1] / max(curve_g(d0)[1], 1e-9) * plant.get("arm", 0.033) / L_ARM_FC
    true_n = plant.get("thrust_scale", 1.0) * curve_g(deff)[0] / GPN   # curve is 4-motor total
    return {"G_act": k, "duty": d, "d_eff": deff, "batt_scale": bs, "V": v,
            "true_thrust_over_weight": true_n / (plant.get("mass", 0.055) * G)}


def _tf(gains: dict, plant: dict, coll: float, outer: bool = True):
    """Open loop L(s) = N/D (numpy poly1d), broken at the moment command, delay as Pade(3), plus
    the exact delay for frequency-domain use. Returns (N, D, Td)."""
    import numpy as np  # noqa: PLC0415
    P = np.poly1d
    g = dict(FLOWN, **gains)
    if not outer:
        g["ATT_P"] = g["ATT_I"] = 0.0
    I = plant.get("I", 2.5e-5) * (1 + plant.get("cradle_frac", 0.0))
    m, h = plant.get("mass", 0.055), plant.get("h_mm", 1.0) * 1e-3
    I += m * h * h
    tau = plant.get("tau", plant.get("tau_up", 0.07))
    if plant.get("tau_speed"):
        # as rig_twin.cpp tau_speed=1: spin-up damping = electrical + aero (~ rotor speed ~ sqrt(T))
        s_ = math.sqrt(max(actuator_gain(plant, coll)["true_thrust_over_weight"], 0.02))
        a_ = plant.get("tau_aero_frac", 0.48)
        tau = tau / ((1 - a_) + a_ * s_)
    # transport + mean frame hold (a frame goes out on the first loop tick >= 5 ms after the last,
    # so the period is ~frame_ms + one loop period) + the sensor sample's mean age (half a period)
    T_loop = 1.0 / plant.get("loop_hz", 355.0)
    Td = plant.get("delay_ms", 2.0) * 1e-3 + 0.5 * (plant.get("frame_ms", 5.0) * 1e-3 + T_loop) \
        + 0.5 * T_loop
    K = J0 * g["ATT_GAIN"] * actuator_gain(plant, coll)["G_act"]
    s = P([1, 0])
    # gyro LPF and D LPF as fractions
    if g["GYRO_LPF_HZ"] > 0:
        wg = 2 * math.pi * g["GYRO_LPF_HZ"]
        Fg_n, Fg_d = P([wg]), P([1, wg])
    else:
        Fg_n, Fg_d = P([1]), P([1])
    wd = 2 * math.pi * g["D_LPF_HZ"]
    Fd_n, Fd_d = P([wd]), P([1, wd])
    # K(s) = (RP s + RI)/s * [ (AP s + AI)/s + s Fg ] + RD s^2 Fg Fd
    rate_n, rate_d = P([g["RATE_P"], g["RATE_I"]]), s
    att_n, att_d = P([g["ATT_P"], g["ATT_I"]]), s
    inner_n = att_n * Fg_d + s * Fg_n * att_d          # over att_d * Fg_d
    inner_d = att_d * Fg_d
    a_n, a_d = rate_n * inner_n, rate_d * inner_d
    b_n, b_d = g["RATE_D"] * s * s * Fg_n * Fd_n, Fg_d * Fd_d
    C_n, C_d = a_n * b_d + b_n * a_d, a_d * b_d
    # Pade(3) of exp(-s Td)
    c = [1, 0.5, 0.1, 1 / 120]
    pd_n = P([(-1) ** 3 * c[3] * Td ** 3, c[2] * Td ** 2, -c[1] * Td, c[0]])
    pd_d = P([c[3] * Td ** 3, c[2] * Td ** 2, c[1] * Td, c[0]])
    plant_d = P([I, plant.get("viscous", 2e-6), m * G * h]) * P([tau, 1])
    N = K * C_n * pd_n
    D = C_d * pd_d * plant_d
    # cancel the s-factors the zero I gains leave in both (pole-zero pairs at the origin)
    while len(N.c) > 1 and len(D.c) > 1 and abs(N.c[-1]) < 1e-30 and abs(D.c[-1]) < 1e-30:
        N, D = P(N.c[:-1]), P(D.c[:-1])
    return N, D, Td, K, C_n, C_d, plant_d


def loop_freq(gains: dict, plant: dict, coll: float, w, outer: bool = True):
    """L(jw) with the exact delay and the 5 ms frame-hold sinc."""
    import numpy as np  # noqa: PLC0415
    N, D, Td, K, C_n, C_d, plant_d = _tf(gains, plant, coll, outer)
    jw = 1j * np.asarray(w)
    Tf = plant.get("frame_ms", 5.0) * 1e-3 + 1.0 / plant.get("loop_hz", 355.0)
    sinc = np.sinc(np.asarray(w) * Tf / (2 * math.pi))
    return K * C_n(jw) / C_d(jw) / plant_d(jw) * np.exp(-jw * Td) * sinc


def margins(gains: dict, plant: dict, coll: float = 1.0, outer: bool = True) -> dict:
    """Phase margin / crossover, upper and lower gain margins (frequency response), and
    closed-loop stability + slowest/least-damped pole from the Pade characteristic polynomial."""
    import numpy as np  # noqa: PLC0415
    w = np.logspace(-1, 3.3, 6000)
    L = loop_freq(gains, plant, coll, w, outer)
    mag = np.abs(L)
    ph = np.degrees(np.unwrap(np.angle(L)))
    out = {"pm_deg": None, "wc_radps": None, "fc_hz": None, "gm_db": None, "gm_low_db": None,
           "w180_radps": None}
    ix = np.nonzero(np.diff(np.sign(mag - 1.0)))[0]
    pms = []
    for i in ix:
        f = (1 - mag[i]) / (mag[i + 1] - mag[i])
        wc = w[i] + f * (w[i + 1] - w[i])
        p = ph[i] + f * (ph[i + 1] - ph[i])
        wrapped = (p + 180.0) % 360.0 - 180.0      # phase in [-180, 180)
        pms.append((180.0 - abs(wrapped), wc))      # angular distance from -1 (cl_stable says which side)
    if pms:
        pm, wc = min(pms)
        out.update(pm_deg=float(pm), wc_radps=float(wc), fc_hz=float(wc / 2 / math.pi))
    # phase crossings of -180 - 360 k
    k = np.floor((ph + 180.0) / 360.0)
    jx = np.nonzero(np.diff(k))[0]
    up, low = [], []
    for i in jx:
        gdb = -20 * math.log10(max(mag[i], 1e-12))
        (up if mag[i] < 1 else low).append((gdb, w[i]))
    if up:
        gm, w180 = min(up)
        out.update(gm_db=float(gm), w180_radps=float(w180))
    if low:
        out["gm_low_db"] = float(-max(low)[0])   # gain may DROP this many dB before instability
    N, D, *_ = _tf(gains, plant, coll, outer)
    roots = (D + N).r
    roots = roots[np.abs(roots) < 1e5]           # drop Pade/filter far poles' numerical noise
    out["cl_stable"] = bool(np.all(roots.real < 0))
    rr = max(roots, key=lambda r: r.real)
    out["cl_max_real"] = float(rr.real)
    osc = [r for r in roots if abs(r.imag) > 1e-3]
    if osc:
        r = min(osc, key=lambda r: -r.real / abs(r))
        out["cl_min_zeta"] = float(-r.real / abs(r))
        out["cl_min_zeta_hz"] = float(abs(r) / 2 / math.pi)
    out["G_act"] = actuator_gain(plant, coll)["G_act"]
    gg = dict(FLOWN, **gains)
    out["stiffness_Nm_per_rad"] = float(J0 * gg["ATT_GAIN"] * out["G_act"] * gg["RATE_P"] * gg["ATT_P"])
    a = plant.get("mass", 0.055) * G * plant.get("h_mm", 1.0) * 1e-3
    if a < 0 and outer:
        # CoM above the axis: the loop's static stiffness must beat m g |h| (lower gain margin)
        out["gm_low_db"] = 20 * math.log10(max(out["stiffness_Nm_per_rad"] / -a, 1e-9))
    return out


def box_margins(gains: dict, coll: float, box: dict = BOX) -> dict:
    """Worst case of `margins` over the box corners + centre (linear: imbalance has no effect).
    PM/GM minima are over the closed-loop-stable points; n_unstable counts the rest. Below half
    hover collective the rotors are slower (tau_speed, as the nonlinear map)."""
    res = []
    for pt in box_points(box):
        p = dict(NOMINAL, **pt)
        if coll < 0.5:
            p["tau_speed"] = 1
        res.append((pt, margins(gains, p, coll)))
    pms = [r["pm_deg"] for _, r in res if r["pm_deg"] is not None and r["cl_stable"]]
    gms = [r["gm_db"] for _, r in res if r["gm_db"] is not None and r["cl_stable"]]
    worst = min(res, key=lambda x: (x[1]["cl_stable"], x[1]["pm_deg"] if x[1]["pm_deg"] is not None else -999))
    return {"all_stable": all(r["cl_stable"] for _, r in res), "n": len(res),
            "n_unstable": sum(not r["cl_stable"] for _, r in res),
            "pm_min": min(pms) if pms else None, "gm_min": min(gms) if gms else None,
            "worst_point": worst[0], "worst": worst[1]}


# =============================================================================================
# Studies
# =============================================================================================

def _min_none(it):
    """min, where None (never reached RUN) counts as the worst value, -inf."""
    v = [(-math.inf if x is None else x) for x in it]
    return min(v) if v else None


def _fmt(v, nd=1):
    if v is None:
        return "-"
    if isinstance(v, bool):
        return "y" if v else "n"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def study_margins(colls=(1.0, 0.25), out=print) -> dict:
    res = {}
    for coll in colls:
        plant = dict(NOMINAL, tau_speed=1) if coll < 0.5 else NOMINAL
        nom = margins({}, plant, coll)
        inner = margins({}, plant, coll, outer=False)
        bm = box_margins({}, coll)
        res[f"coll_{coll}"] = {"nominal": nom, "inner_only": inner, "box": bm,
                               "actuator": actuator_gain(NOMINAL, coll)}
        out(f"\n== linear margins, flown defaults, coll {coll} ==")
        out(f"  actuator gain G_act {nom['G_act']:.3f} (true thrust / weight "
            f"{actuator_gain(NOMINAL, coll)['true_thrust_over_weight']:.2f})")
        out(f"  nominal  : PM {_fmt(nom['pm_deg'])} deg @ {_fmt(nom['fc_hz'], 2)} Hz, GM {_fmt(nom['gm_db'])} dB"
            f", CL stable {nom['cl_stable']}, min zeta {_fmt(nom.get('cl_min_zeta'), 2)}")
        out(f"  inner    : PM {_fmt(inner['pm_deg'])} deg @ {_fmt(inner['fc_hz'], 2)} Hz, GM {_fmt(inner['gm_db'])} dB")
        out(f"  box ({bm['n']} pts): unstable {bm['n_unstable']}, PM min {_fmt(bm['pm_min'])}, "
            f"GM min {_fmt(bm['gm_min'])}; worst {bm['worst_point']}")
    # sensitivity of the flown defaults to the two least-known plant numbers
    out("\n== flown defaults, coll 1.0: PM deg (CL-unstable marked !) vs I and tau ==")
    sens = []
    out("  tau \\ I    " + "".join(f"{i:>9.1e}" for i in (1.6e-5, 2.5e-5, 3.5e-5, 4.4e-5)))
    for tau in (0.035, 0.05, 0.07, 0.10):
        cells = []
        for i in (1.6e-5, 2.5e-5, 3.5e-5, 4.4e-5):
            mm = margins({}, dict(NOMINAL, I=i, tau=tau), 1.0)
            sens.append({"I": i, "tau": tau, **mm})
            cells.append(f"{_fmt(mm['pm_deg'], 0)}{'' if mm['cl_stable'] else '!'}".rjust(9))
        out(f"  {tau:<10}" + "".join(cells))
    res["sensitivity"] = sens
    # small grid at coll 1.0
    grid = []
    out("\n== linear grid (coll 1.0): nominal PM / box-worst PM / box unstable count ==")
    for rp in (20, 30, 40, 55):
        row = []
        for ap in (2.5, 3.5, 5.0, 7.0):
            g = {"RATE_P": rp, "ATT_P": ap}
            nm, bm = margins(g, NOMINAL, 1.0), box_margins(g, 1.0)
            grid.append({"gains": g, "nominal": nm, "box": {k: bm[k] for k in ("all_stable", "n_unstable", "pm_min", "gm_min", "worst_point")}})
            row.append(f"AP{ap:<4} {_fmt(nm['pm_deg'], 0):>3}/{_fmt(bm['pm_min'], 0):>3}/{bm['n_unstable']:>2}")
        out(f"  RATE_P {rp:<3}: " + "   ".join(row))
    res["grid"] = grid
    return res


def study_map(exes, coll_permille=1000, grid="quick", out=print, workdir=None, extra_plant=None) -> list:
    """Nonlinear robustness map: every gain point x (box corners + centre)."""
    if grid == "full":
        axes = {"RATE_P": [20, 30, 40, 55], "ATT_P": [2.5, 3.5, 5.0, 7.0],
                "RATE_D": [0.0, 0.5, 1.0, 1.5], "ATT_I": [0.0, 1.0, 2.0, 4.0]}
    elif grid == "quick":
        axes = {"RATE_P": [25, 40], "ATT_P": [3.5, 5.0], "RATE_D": [0.0, 0.5], "ATT_I": [0.0, 2.0]}
    else:
        axes = json.loads(grid)
    names = list(axes)
    gpts = [dict(zip(names, c)) for c in itertools.product(*axes.values())]
    pts = box_points()
    tmp = Path(tempfile.mkdtemp(prefix="rig-map-", dir=workdir))
    scen = _write_tmp(scenario_text(coll_permille, hold_ms=4000, after_ms=3000), tmp, "scen.txt")
    plist = []
    for g in gpts:
        for p in pts:
            q = dict(p, gains=g, theta0_deg=0.5)
            if coll_permille < 500:
                q["tau_speed"] = 1        # low collective: slower rotors (evidence model)
            q.update(extra_plant or {})
            plist.append(q)
    t0 = time.time()
    res = run_batch(exes["twin"], scen, plist)
    shutil.rmtree(tmp, ignore_errors=True)
    rows = []
    k = 0
    for g in gpts:
        rs = res[k:k + len(pts)]
        k += len(pts)
        ok = [stable(r) for r in rs]
        ovs = [r["overshoot_pct"] for r in rs if r["overshoot_pct"] is not None]
        bad = [dict(pts[i], status=rs[i]["status"], hits=rs[i]["stop_hits_live"],
                    osc=rs[i]["tail_osc_deg"]) for i in range(len(rs)) if not ok[i]]
        rows.append({
            "gains": g, "n": len(rs), "n_stable": sum(ok),
            "overshoot_max": max(ovs) if ovs else None,
            "overshoot_nom": rs[-1]["overshoot_pct"],
            "settle_nom_ms": rs[-1]["settle_ms"],
            "sat_frac_max": max((r["sat_frac"] or 0) for r in rs),
            "steady_err_max_deg": max(abs(r["steady_err_deg"] or 0) for r in rs),
            "guard_margin_min_deg": _min_none(r["guard_margin_deg"] for r in rs),
            "lin": box_margins(g, coll_permille / 1000.0),
            "unstable_points": bad[:6],
        })
    out(f"\n== nonlinear map, coll {coll_permille}, {len(gpts)} gain points x {len(pts)} plants "
        f"({len(plist)} runs, {time.time() - t0:.0f} s) ==")
    out("  RATE_P ATT_P RATE_D ATT_I | stable  ovs_nom ovs_max  sat_max  |err|max guard_min | lin PMmin GMmin")
    for r in sorted(rows, key=lambda r: (-r["n_stable"], r["overshoot_max"] or 1e9)):
        g = r["gains"]
        out(f"  {g.get('RATE_P', 40):>6} {g.get('ATT_P', 5):>5} {g.get('RATE_D', 0):>6} {g.get('ATT_I', 0):>5} |"
            f" {r['n_stable']:>2}/{r['n']:<3} {_fmt(r['overshoot_nom']):>7} {_fmt(r['overshoot_max']):>7}"
            f" {_fmt(r['sat_frac_max'], 3):>8} {_fmt(r['steady_err_max_deg']):>8} {_fmt(r['guard_margin_min_deg']):>8} |"
            f" {_fmt(r['lin']['pm_min']):>9} {_fmt(r['lin']['gm_min']):>5}")
    return rows


def study_liftoff(exes, gains=None, coll_permille=1000, out=print, workdir=None, stop_deg=30.0) -> dict:
    """Arm resting on the -30 deg stop, default slew/ramp; h <= 0 (or a lateral CoM offset) is what
    keeps it on the stop. Every other box axis at its corners."""
    box = {k: v for k, v in BOX.items() if k != "h_mm"}
    cases = [("h=-2", {"h_mm": -2.0}), ("h=-1", {"h_mm": -1.0}), ("h=0,y=+0.5", {"h_mm": 0.0, "cg_y_mm": 0.5}),
             ("h=+2,y=+2", {"h_mm": 2.0, "cg_y_mm": 2.0})]
    tmp = Path(tempfile.mkdtemp(prefix="rig-lift-", dir=workdir))
    scen = _write_tmp(scenario_text(coll_permille, step_mdeg=0, hold_ms=4000), tmp, "lift.txt")
    plist, tags = [], []
    for name, extra in cases:
        for p in box_points(box):
            plist.append(dict(p, theta0_deg=-stop_deg, stop_deg=stop_deg, gains=gains or {}, **extra))
            tags.append(name)
    res = run_batch(exes["twin"], scen, plist)
    # also: h = +2 mm without a lateral offset (it does not rest on the stop at all)
    free = run_twin(exes["twin"], scen, dict(theta0_deg=-stop_deg, stop_deg=stop_deg, h_mm=2.0,
                                             gains=gains or {}))
    shutil.rmtree(tmp, ignore_errors=True)
    summ = {}
    out(f"\n== lift-off from the -{stop_deg} deg stop, coll {coll_permille}, gains {gains or 'flown'} ==")
    for name, _ in cases:
        rs = [r for r, t in zip(res, tags) if t == name]
        ps = [p for p, t in zip(plist, tags) if t == name]
        st = {}
        for r in rs:
            st[r["status"]] = st.get(r["status"], 0) + 1
        lo = [r["liftoff_ms"] for r in rs if r["liftoff_ms"] is not None]
        summ[name] = {"n": len(rs), "status": st,
                      "liftoff_ms": [min(lo), max(lo)] if lo else None,
                      "max_live_deg": max(r["max_live_deg"] for r in rs),
                      # started on the -30 deg stop: how far past level it swings the other way
                      "overshoot_past_level_deg": max((r["roll_max_live_deg"] or 0) for r in rs),
                      # split by the imbalance sign: +imbalance leans toward +roll (a steady lean
                      # without ATT_I, not a lift-off overshoot); 0 isolates the lift-off itself
                      "past_level_by_imbalance": {
                          str(v): max((r["roll_max_live_deg"] or 0) for r, p in zip(rs, ps)
                                      if p["imbalance"] == v)
                          for v in sorted({p["imbalance"] for p in ps})},
                      "guard_margin_min_deg": _min_none(r["guard_margin_deg"] for r in rs),
                      "stop_hits_live": sum(r["stop_hits_live"] for r in rs),
                      "max_impact_radps": max(r["max_impact_radps"] for r in rs)}
        s = summ[name]
        lo = s['liftoff_ms']
        out(f"  {name:<11} {s['status']}  roll guard live {lo[0]:.0f}-{lo[1]:.0f} ms after arm; max swing "
            f"past level {s['overshoot_past_level_deg']:+.1f} deg (by imbalance "
            f"{ {k: round(v, 1) for k, v in s['past_level_by_imbalance'].items()} }; stop +{stop_deg}, guard +25.2); guard margin "
            f">= {_fmt(s['guard_margin_min_deg'])} deg in RUN; "
            f"stop hits after lift-off {s['stop_hits_live']}"
            if lo else f"  {name:<11} {s['status']}  never lifted off")
    summ["h=+2 no offset"] = {"status": free["status"], "arm_s": free["arm_s"], "events": free["events"][:4]}
    out(f"  h=+2,y=0   : swings off the stop before arming; arms at t={free['arm_s']} s, status {free['status']}")
    return summ


# Gains of the 2026-11-04 hardware hold (results/rig/manual/20261104-144131-hold600long.gains.json),
# with ATT_I_LIM raised to the new default: at its 1.0 there the assist's clamp min(LIFT_I_LIM,
# ATT_I_LIM) is 1 rad/s (~0.8 mN m) and it cannot lift off.
HW_1104 = {"ATT_P": 5.0, "ATT_I": 1.0, "RATE_P": 40.0, "RATE_D": 1.0, "ATT_GAIN": 1.2,
           "MOTOR_TRIM_LEFT": 0.85, "ATT_I_LIM": 8.0}
# Steady torque toward +roll (right wing down): the lateral CoM offset (cg_y_mm < 0 = right of the
# axis; m g y = 2.7 mN m at 55 g, 5 mm) plus a tethered cable (ext_mNm), resting on the +30 stop.
ASSIST_PLANT = {"theta0_deg": 30.0, "stop_deg": 30.0, "cg_y_mm": -5.0, "ext_mNm": 0.8,
                "lipo": 1, "ocv": 4.15}
# The same torque on the pitch rig (rig-pitch, the drone turned 90 deg): the CoM 5 mm FORWARD of the
# pivot (cg_x_mm > 0, nose-down = toward +pitch; the twin's offset is -cg_x) plus the same 0.8 mN m,
# resting on the +30 deg stop. (The cable runs along the pivot there, so 0.8 is a pessimistic stand-in
# for whatever else loads it.)
ASSIST_PLANT_PITCH = {"theta0_deg": 30.0, "stop_deg": 30.0, "cg_x_mm": 5.0, "ext_mNm": 0.8,
                      "lipo": 1, "ocv": 4.15, "axis": 1}


def study_liftoff_assist(exes, out=print, workdir=None, sweep=True, axis=0) -> dict:
    """The lift-off assist (fc_rig.cpp LIFT-OFF ASSIST) against a steady 3-4 mN m torque toward
    the +30 deg stop: without it (LIFT_I 0) it never leaves the stop, as on hardware; with the
    defaults it lifts off at COLL 600 and 800. Then LIFT_I x LIFT_I_LIM at the nominal plant, and
    the defaults over the plant box (thrust 0.7-1.3, tau, I, delay, h, imbalance; LiPo and PSU).
    axis=1: the same on the pitch rig (rig-pitch twin, ASSIST_PLANT_PITCH)."""
    plant = ASSIST_PLANT_PITCH if axis else ASSIST_PLANT
    ax = "pitch" if axis else "roll"
    scen_path = HERE / "scenarios/liftoff_assist.txt"
    base = scen_path.read_text()
    tmp = Path(tempfile.mkdtemp(prefix="rig-assist-", dir=workdir))
    scens = {c: _write_tmp(base.replace("COLL 600", f"COLL {c}"), tmp, f"assist{c}.txt") for c in (600, 800)}
    keys = ("status", "liftoff_ms", "liftoff_att_int", "att_int_peak", "guard_margin_live_deg",
            "roll_min_live_deg", "settle_level_ms", "peak_duty")
    rep = {"nominal": {}, "sweep": [], "box": {}}
    out(f"\n== lift-off assist, {ax.upper()} rig: +30 deg stop, 3.5 mN m toward it "
        f"({'cg_x +5' if axis else 'cg_y -5'} mm + cable 0.8), LiPo ==")
    for coll in (600, 800):
        for name, g in (("no assist", dict(HW_1104, LIFT_I=0)), ("assist", dict(HW_1104)),
                        ("assist ATT_I 2", dict(HW_1104, ATT_I=2.0)),
                        ("ATT_I_LIM 1", dict(HW_1104, ATT_I_LIM=1.0))):
            js = run_twin(exes["twin"], scens[coll], dict(plant, gains=g))
            r = {k: js.get(k) for k in keys}
            rep["nominal"][f"{coll} {name}"] = r
            out(f"  COLL {coll} {name:<15} {r['status']:<30} lift-off {_fmt(r['liftoff_ms'], 0)} ms, "
                f"att_int {_fmt(r['liftoff_att_int'], 2)} -> {_fmt(r['att_int_peak'], 2)} rad/s, past level "
                f"{_fmt(r['roll_min_live_deg'])} deg (min {ax}), guard margin {_fmt(r['guard_margin_live_deg'])} "
                f"deg, |{ax}| < 2 deg {_fmt(r['settle_level_ms'], 0)} ms after lift-off")
    if sweep:
        grid = [(li, ll, coll, tq) for li in (1, 2, 4, 6, 8, 10) for ll in (4, 6, 8)
                for coll in (600, 800) for tq in (0.3, 1.3)]
        plist = [dict(plant, ext_mNm=tq, gains=dict(HW_1104, LIFT_I=li, LIFT_I_LIM=ll))
                 for li, ll, coll, tq in grid]
        res = []
        for coll in (600, 800):
            idx = [i for i, gr in enumerate(grid) if gr[2] == coll]
            for i, js in zip(idx, run_batch(exes["twin"], scens[coll], [plist[i] for i in idx])):
                res.append((grid[i], js))
        out("  sweep (hw gains, 3.0 and 4.0 mN m, COLL 600/800): LIFT_I LIFT_I_LIM -> ok/n, worst "
            f"lift-off ms, min {ax} after lift-off, min guard margin")
        for li in (1, 2, 4, 6, 8, 10):
            for ll in (4, 6, 8):
                rs = [js for gr, js in res if gr[0] == li and gr[1] == ll]
                lo = [js["liftoff_ms"] for js in rs if js["status"] == "ok"]
                row = {"LIFT_I": li, "LIFT_I_LIM": ll, "ok": sum(js["status"] == "ok" for js in rs),
                       "n": len(rs), "liftoff_ms_max": max(lo) if lo else None,
                       "roll_min_live_deg": _min_none(js["roll_min_live_deg"] for js in rs
                                                      if js["status"] == "ok"),
                       "guard_margin_live_deg": _min_none(js["guard_margin_live_deg"] for js in rs
                                                          if js["status"] == "ok")}
                rep["sweep"].append(row)
                out(f"    {li:>4} {ll:>3}: {row['ok']}/{row['n']}  {_fmt(row['liftoff_ms_max'], 0):>6}  "
                    f"{_fmt(row['roll_min_live_deg']):>6}  {_fmt(row['guard_margin_live_deg']):>5}")
        pts = box_points()
        for coll in (600, 800):
            plist = [dict(p, **plant, gains=dict(HW_1104)) for p in pts]
            rs = run_batch(exes["twin"], scens[coll], plist)
            ok = [js for js in rs if js["status"] == "ok"]
            bad = [p for p, js in zip(pts, rs) if js["status"] != "ok"]
            rep["box"][coll] = {"ok": len(ok), "n": len(rs),
                                "liftoff_ms": [min(j["liftoff_ms"] for j in ok), max(j["liftoff_ms"] for j in ok)] if ok else None,
                                "roll_min_live_deg": _min_none(j["roll_min_live_deg"] for j in ok),
                                "guard_margin_live_deg": _min_none(j["guard_margin_live_deg"] for j in ok),
                                "not_ok_thrust_scale": sorted({p["thrust_scale"] for p in bad})}
            b = rep["box"][coll]
            out(f"  box, defaults, COLL {coll}: {b['ok']}/{b['n']} ok, lift-off {b['liftoff_ms']} ms, min {ax} "
                f"after lift-off {_fmt(b['roll_min_live_deg'])} deg, guard margin >= "
                f"{_fmt(b['guard_margin_live_deg'])} deg; not ok at thrust_scale {b['not_ok_thrust_scale']}")
    shutil.rmtree(tmp, ignore_errors=True)
    return rep


def study_unstable_h(exes, gains=None, coll_permille=1000, out=print, workdir=None) -> dict:
    """CoM above the axis: lean, stiffness and divergence vs h (nominal plant otherwise), plus
    the linear static-divergence limit m g |h| = stiffness."""
    tmp = Path(tempfile.mkdtemp(prefix="rig-h-", dir=workdir))
    scen = _write_tmp(scenario_text(coll_permille), tmp, "h.txt")
    hs = [-1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -8.0]
    plist = []
    for h in hs:
        for imb in (0.0, 0.05):
            plist.append(dict(h_mm=h, imbalance=imb, theta0_deg=0.5, gains=gains or {}))
    res = run_batch(exes["twin"], scen, plist)
    shutil.rmtree(tmp, ignore_errors=True)
    out(f"\n== CoM above the axis (h < 0), coll {coll_permille}, nominal plant ==")
    rows = []
    for i, h in enumerate(hs):
        a, b = res[2 * i], res[2 * i + 1]
        lin = margins(gains or {}, dict(NOMINAL, h_mm=h), coll_permille / 1000)
        rows.append({"h_mm": h, "balanced": a, "imb5": b, "lin": lin})
        out(f"  h {h:+.0f} mm: {a['status']:<22} step ovs {_fmt(a['overshoot_pct'])}%  | with +5% imbalance:"
            f" {b['status']:<16} lean {_fmt(b['steady_err_deg'])} deg | lin CL stable {lin['cl_stable']}, "
            f"PM {_fmt(lin['pm_deg'])}, GM_low {_fmt(lin['gm_low_db'])} dB")
    nom_k = margins(gains or {}, NOMINAL, coll_permille / 1000)["stiffness_Nm_per_rad"]
    hcrit = nom_k / (NOMINAL["mass"] * G) * 1e3
    out(f"  static divergence when m g |h| > loop stiffness {nom_k * 1e3:.2f} mN m/rad: h < -{hcrit:.1f} mm "
        f"(nominal plant; scales with ATT_GAIN*RATE_P*ATT_P*G_act)")
    return {"rows": rows, "h_crit_mm": -hcrit}


def study_noise(exes, out=print, workdir=None) -> list:
    """RATE_D vs gyro noise: duty jitter (std of duty0 over 1 s of quiet RUN) at the sensor-only
    0.005 rad/s and a prop-vibration-like 0.05 rad/s."""
    tmp = Path(tempfile.mkdtemp(prefix="rig-noise-", dir=workdir))
    scen = _write_tmp(scenario_text(1000, step_mdeg=0, hold_ms=3000), tmp, "n.txt")
    rows = []
    out(f"\n== gyro noise -> duty jitter (std of duty0 in RUN, flown P gains, D_LPF 30 Hz, "
        f"dt replay {'on' if dt_file() else 'off'}) ==")
    for rd in (0.0, 0.5, 1.0, 1.5):
        cells = []
        for gn in (0.005, 0.05):
            c = tmp / f"n{rd}{gn}.csv"
            run_twin(exes["twin"], scen, {"gyro_noise": gn, "gains": {"RATE_D": rd}}, csv_path=c)
            d = load_csv(c)
            v = [x for x, st in zip(d["duty0"], d["rs_name"]) if st == "RUN"][-315:]
            mu = sum(v) / len(v)
            sd = math.sqrt(sum((x - mu) ** 2 for x in v) / len(v))
            rows.append({"RATE_D": rd, "gyro_noise": gn, "duty_std": sd})
            cells.append(f"noise {gn}: {sd:.4f}")
        out(f"  RATE_D {rd:<4} " + "   ".join(cells))
    shutil.rmtree(tmp, ignore_errors=True)
    return rows


FLOWN_SET = {"RATE_P": 40, "ATT_P": 5.0, "RATE_D": 0.0, "ATT_I": 0.0}
CONSERVATIVE = {"RATE_P": 40, "ATT_P": 5.0, "RATE_D": 1.0, "ATT_I": 1.0}
SWEEP_GRID = [  # (RATE_P, ATT_P, RATE_D, ATT_I), the first sweep: safe across the box at 1260 Hz
    (40, 5.0, 1.0, 1.0), (40, 5.0, 1.5, 1.0), (40, 5.0, 1.5, 2.0), (55, 3.5, 0.5, 1.0),
    (55, 3.5, 1.0, 1.0), (55, 3.5, 1.0, 2.0), (55, 3.5, 1.5, 1.0), (55, 3.5, 1.5, 2.0)]


def study_candidates(exes, cands=None, colls=(1000, 800, 250), out=print, extra_plant=None) -> list:
    """Named gain sets x collectives: nonlinear box map row + linear box margins, with the failing
    box points spelled out."""
    cands = cands or [("flown", FLOWN_SET)] + [
        (f"grid{i}", dict(zip(("RATE_P", "ATT_P", "RATE_D", "ATT_I"), g))) for i, g in enumerate(SWEEP_GRID)]
    rows = []
    out(f"\n== candidates across the box (dt replay: {dt_file() or 'off, fixed loop_hz'}) ==")
    for name, g in cands:
        for coll in colls:
            r = study_map(exes, coll, json.dumps({k: [v] for k, v in g.items()}), out=lambda s: None,
                          extra_plant=extra_plant)[0]
            nom = margins(g, dict(NOMINAL, tau_speed=1) if coll < 500 else NOMINAL, coll / 1000)
            lb = r["lin"]
            rows.append({"name": name, "gains": g, "coll": coll, "map": r, "nominal": nom})
            out(f"  {name:<6} {tuple(g.values())} coll {coll:>4}: nonlin {r['n_stable']}/{r['n']}, "
                f"lean<= {_fmt(r['steady_err_max_deg'])}, guard>= {_fmt(r['guard_margin_min_deg'])} | nominal PM "
                f"{_fmt(nom['pm_deg'])} @ {_fmt(nom['fc_hz'], 2)} Hz GM {_fmt(nom['gm_db'])} | box lin unstable "
                f"{lb['n_unstable']}, PM>= {_fmt(lb['pm_min'])}, GM>= {_fmt(lb['gm_min'])}")
            for u in r["unstable_points"][:3]:
                out(f"      fails: {u}")
    return rows


def recommend(map_rows: list, map_low: list | None = None, out=print) -> dict:
    """Rank the map's gain points: stable at every box point (nonlinear and linear) first, then the
    worst-case linear PM, then the guard margin. Also the subset that is safe everywhere."""
    low_ok = {json.dumps(r["gains"], sort_keys=True): r["n_stable"] == r["n"] for r in (map_low or [])}

    def key(r):
        k = json.dumps(r["gains"], sort_keys=True)
        return (r["n_stable"] == r["n"], r["lin"]["all_stable"], low_ok.get(k, True),
                r["lin"]["pm_min"] or -99, r["guard_margin_min_deg"] or -99)
    ranked = sorted(map_rows, key=key, reverse=True)
    low_guard = {json.dumps(r["gains"], sort_keys=True): (r["guard_margin_min_deg"] or -99) for r in (map_low or [])}
    safe_rows = [r for r in ranked if key(r)[:3] == (True, True, True)
                 and (r["guard_margin_min_deg"] or -99) >= 3.0
                 and low_guard.get(json.dumps(r["gains"], sort_keys=True), 99) >= 3.0]
    safe = [r["gains"] for r in safe_rows]

    def novelty(r):
        # conservative = closest to the flown P gains, then the least added D and I, then PM
        g = dict(FLOWN, **r["gains"])
        return (abs(math.log(g["RATE_P"] / 40.0)) + abs(math.log(g["ATT_P"] / 5.0)),
                g["RATE_D"] + 0.25 * g["ATT_I"], -(r["lin"]["pm_min"] or 0))
    conservative = min(safe_rows, key=novelty)["gains"] if safe_rows else None
    out(f"\n== ranking (stable at all {map_rows[0]['n']} box points, nonlinear and linear; then the box-worst "
        f"linear PM; then the guard margin) ==")
    for r in ranked[:8]:
        out(f"  {r['gains']}  stable {r['n_stable']}/{r['n']}, lin stable {r['lin']['all_stable']}, "
            f"PM box-min {_fmt(r['lin']['pm_min'])}, GM box-min {_fmt(r['lin']['gm_min'])} dB, "
            f"overshoot nom/max {_fmt(r['overshoot_nom'])}/{_fmt(r['overshoot_max'])} %, "
            f"guard margin {_fmt(r['guard_margin_min_deg'])} deg")
    out(f"  safe everywhere (also guard margin >= 3 deg in RUN at both collectives): {len(safe)} of "
        f"{len(map_rows)} gain points: {[tuple(g.values()) for g in safe]}")
    out(f"  conservative pick (safe, nearest the flown RATE_P/ATT_P, least added D/I): {conservative}")
    return {"ranked": [{"gains": r["gains"], "key": key(r)} for r in ranked[:20]], "safe": safe,
            "conservative": conservative}


def run_one(args, exes):
    tmp = Path(tempfile.mkdtemp(prefix="rig-run-"))
    scen = Path(args.scenario) if args.scenario else _write_tmp(
        scenario_text(args.coll, args.step_mdeg), tmp, "scen.txt")
    params = {}
    for kv in args.set or []:
        k, v = kv.split("=", 1)
        params[k] = v
    params["gains"] = dict(kv.split("=", 1) for kv in (args.gain or []))
    if getattr(args, "axis", "roll") == "pitch":
        params.setdefault("axis", 1)
    js = run_twin(exes, scen, params, csv_path=args.csv, truth=args.truth)   # twin per axis
    print(json.dumps(js, indent=1))
    if args.csv and args.step_mdeg:
        m = analyze_csv(args.csv, {"type": "step", "target": "att", "amp": args.step_mdeg,
                                   "dur_ms": 1500})
        print("metrics:", json.dumps({k: m.get(k) for k in (*JOURNAL_KEYS, "_by")}))
    shutil.rmtree(tmp, ignore_errors=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--dt-csv", default=None,
                    help="recorder CSV whose dt_us loop periods are replayed (default DT_CSV; 'none' = fixed loop_hz)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("build")
    sub.add_parser("mixer-check")
    r = sub.add_parser("run")
    r.add_argument("--scenario")
    r.add_argument("--coll", type=int, default=1000)
    r.add_argument("--step-mdeg", type=int, default=87)
    r.add_argument("--set", action="append", help="twin parameter key=value")
    r.add_argument("--gain", action="append", help="NAME=value (GAIN at t=0)")
    r.add_argument("--axis", choices=("roll", "pitch"), default="roll",
                   help="the rig's axis (pitch = rig-pitch, the drone turned 90 deg; twin axis=1)")
    r.add_argument("--csv")
    r.add_argument("--truth")
    m = sub.add_parser("margins")
    m.add_argument("--coll", type=float, action="append")
    mp = sub.add_parser("map")
    mp.add_argument("--coll", type=int, default=1000)
    mp.add_argument("--grid", default="quick", help="quick | full | JSON {NAME: [values]}")
    sub.add_parser("liftoff")
    la = sub.add_parser("liftoff-assist")
    la.add_argument("--axis", choices=("roll", "pitch"), default="roll",
                    help="pitch: the rig-pitch twin, CoM forward of the pivot (ASSIST_PLANT_PITCH)")
    la.add_argument("--no-sweep", action="store_true", help="nominal runs only (no LIFT_I sweep / box)")
    sub.add_parser("unstable-h")
    sub.add_parser("noise")
    rp = sub.add_parser("report")
    rp.add_argument("--grid", default="full")
    rp.add_argument("--out", default=None)
    sub.add_parser("candidates")
    args = ap.parse_args(argv)
    if args.dt_csv is not None:
        set_dt_csv(args.dt_csv)

    exes = build()
    if args.cmd == "build":
        print(exes)
    elif args.cmd == "mixer-check":
        print(mixer_check(exes))
    elif args.cmd == "run":
        run_one(args, exes)
    elif args.cmd == "margins":
        study_margins(tuple(c / 1000 if c > 10 else c for c in (args.coll or [1.0, 0.25])))
    elif args.cmd == "map":
        study_map(exes, args.coll, args.grid)
    elif args.cmd == "liftoff":
        study_liftoff(exes)
    elif args.cmd == "liftoff-assist":
        study_liftoff_assist(exes, sweep=not args.no_sweep, axis=1 if args.axis == "pitch" else 0)
    elif args.cmd == "unstable-h":
        study_unstable_h(exes)
    elif args.cmd == "noise":
        study_noise(exes)
    elif args.cmd == "candidates":
        study_candidates(exes)
    elif args.cmd == "report":
        outd = Path(args.out or ROOT / "results/rig-sim" / time.strftime("%Y%m%d-%H%M%S"))
        outd.mkdir(parents=True, exist_ok=True)
        log = open(outd / "report.txt", "w")

        def out(s):
            print(s)
            log.write(s + "\n")
            log.flush()
        out(mixer_check(exes))
        rep = {"margins": study_margins(out=out)}
        rep["map_1000"] = study_map(exes, 1000, args.grid, out=out)
        rep["map_250"] = study_map(exes, 250, args.grid, out=out)
        rep["liftoff"] = study_liftoff(exes, out=out)
        rep["unstable_h"] = study_unstable_h(exes, out=out)
        rep["noise"] = study_noise(exes, out=out)
        rep["recommend"] = recommend(rep["map_1000"], rep["map_250"], out=out)
        best = rep["recommend"]["conservative"] or {}
        rep["liftoff_conservative_800"] = study_liftoff(exes, gains=best, coll_permille=800, out=out)
        rep["map_800_conservative"] = study_map(exes, 800, json.dumps({k: [v] for k, v in best.items()}),
                                                out=out) if best else None
        (outd / "report.json").write_text(json.dumps(rep, indent=1, default=str))
        out(f"\nwrote {outd}/report.json, report.txt")
    return 0


if __name__ == "__main__":
    sys.exit(main())
