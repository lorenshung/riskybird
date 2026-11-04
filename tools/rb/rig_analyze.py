#!/usr/bin/env python3
"""
RiskyBird roll-harness ("rig") analysis: metrics per trial, ranking, a next grid.

    ./rb rig analyze results/rig/<batch>          # zephyr env: numpy, scipy, matplotlib

Reads a batch directory written by tools/rb/rig_run.py (batch.json, journal.jsonl,
trials/<n>/rec.csv + trial.json), recomputes every trial's metrics offline, writes
trials/<n>/metrics.json (+ plot.png / frf.png), appends a journal `metrics` event where the
values changed, prints a ranked table and writes summary.md and next_grid.json.

STEP / DOUBLET METRICS (per edge of the excitation, worst edge reported)
  The response y and reference r are the roll angle and roll target (deg) for `att`
  excitations, gyro and rate target (deg/s) for `rate`. Each edge runs to the next edge (or
  the end of the trial). y0 = y just before the edge, y_ss = mean of the window's last 20 %.
  overshoot_pct  100 * max(sign * (y - y_ss)) / |y_ss - y0|   (>= 0)
  rise_ms        10 % -> 90 % of y0 -> y_ss
  settle_ms      last time |y - y_ss| > band, band = max(5 % of the edge's step, 1 deg)
                 (rate: 5 deg/s); None and settled=false if it is still outside at the end
  steady_err_deg r - y_ss at the end of the edge window (worst edge, signed)
  `moment` excitations are disturbances: no overshoot/rise; settle_ms is the recovery after
  the last edge to within 1 deg of the target.
  For every trial: rms_err_deg (roll target - roll from the EXC on), peak_duty, sat_frac
  (fraction of samples with any duty >= 0.94 or <= 0.01), osc_hz / osc_amp_deg (FFT peak and
  sqrt(2)*std of the detrended roll error in the quiet tail after the last edge has settled;
  osc_hz only when osc_amp_deg >= 0.3).

CHIRP / PRBS / SINE (frequency response, Welch cross-spectra with the excitation as the
instrument, so loop noise does not bias the estimate)
  frf roll/exc and the closed loop T = y/r (att: roll/roll_tgt, rate: gyro/rate_tgt), the loop
  L = T/(1-T) (moment: L = -u_ctrl/u_total); coherence(exc, .) > 0.6 masks every estimate.
  pm_deg/pm_hz at |L| = 1, gm_db/gm_hz at phase(L) = -180, bw_hz where |T| < -3 dB.

AXIS
  A rig-pitch batch (the drone turned 90 deg on the pivot) is analysed exactly like a roll one, on
  the controlled axis: the recorder's generic columns ang_deg / ang_tgt_deg / ang_rate_dps (the
  axis column says R or P) stand for roll_deg / roll_tgt_deg / gyro_p everywhere above. A CSV from
  before those columns existed is a roll recording and uses roll_deg / roll_tgt_deg / gyro_p.

SCORE (lower is better; None for trials whose status is not ok: estop/aborted/timeout)
  step-like:  rms_err_deg + 0.05*overshoot_pct + 0.3*settle_s + 20*sat_frac
              + 10*max(0, osc_amp_deg - 0.2) + 3*(not settled)
              i.e. 1 deg rms = 20 % overshoot = 3.3 s settling = 5 % of samples saturated =
              0.1 deg of sustained oscillation above the noise floor.
  frf trials get NO `score` (a chirp's tracking error is not comparable with a step's);
  they get margin_penalty = max(0, 45 - pm_deg)/10 + max(0, 6 - gm_db)/2 + 20*sat_frac
              + 10*max(0, osc_amp_deg - 0.2)   (5 when no phase margin could be found).
  A gain point's score = mean(score of its step-like trials) + mean(its margin_penalty).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
if not __package__:
    sys.path.insert(0, str(_HERE))
    import rig_tui as T  # noqa: E402
    import rig_run as RR  # noqa: E402
    import rig_axis as AX  # noqa: E402
else:
    from . import rig_tui as T
    from . import rig_run as RR
    from . import rig_axis as AX

JOURNAL_KEYS = ("overshoot_pct", "rise_ms", "settle_ms", "rms_err_deg", "steady_err_deg",
                "peak_duty", "sat_frac", "osc_hz", "osc_amp_deg", "settled", "pm_deg", "gm_db",
                "bw_hz", "margin_penalty", "score")
SAT_HI, SAT_LO = 0.94, 0.01
BAND_FRAC = 0.05
BAND_ABS = {"att": 1.0, "rate": 5.0, "moment": 1.0}
OSC_MIN_DEG = 0.3
OSC_FLOOR_DEG = 0.2
COH_MIN = 0.6
FRF_TYPES = ("chirp", "prbs", "sine")


# =============================================================================
# Loading
# =============================================================================

def load_rec(path):
    """rec.csv -> dict of float arrays (missing columns absent)."""
    with open(path, "r", encoding="utf-8", errors="replace", newline="") as f:
        rd = csv.reader(f)
        header = next(rd, [])
        rows = [r for r in rd if r]
    out = {}
    for i, name in enumerate(header):
        try:
            out[name] = np.array([float(r[i]) for r in rows], dtype=float)
        except ValueError:
            continue
    out["_n"] = len(rows)
    # the controlled axis: the axis column (R/P) if the recorder wrote it, else roll (old CSVs)
    ai = header.index("axis") if "axis" in header else None
    letters = {r[ai] for r in rows if ai is not None and ai < len(r) and r[ai]}
    out["_axis"] = AX.parse_axis(letters.pop()) if len(letters) == 1 else AX.ROLL
    if out["_axis"] is None:
        out["_axis"] = AX.ROLL
    return out


def ctl_angle(d):
    """(angle, target) of the controlled axis in deg: the generic ang_deg / ang_tgt_deg columns,
    or roll_deg / roll_tgt_deg in a CSV from before them (always a roll recording)."""
    if "ang_deg" in d and "ang_tgt_deg" in d:
        return d["ang_deg"], d["ang_tgt_deg"]
    return d["roll_deg"], d["roll_tgt_deg"]


def ctl_rate_dps(d):
    """The controlled axis' body rate in deg/s: ang_rate_dps (already deg/s), else gyro_p (rad/s)."""
    if "ang_rate_dps" in d:
        return d["ang_rate_dps"]
    return np.degrees(d["gyro_p"])


def _col(d, *names):
    for n in names:
        if n in d:
            return d[n]
    return None


# =============================================================================
# Step metrics
# =============================================================================

def find_edges(x, rel=0.4):
    """Indices where the excitation x jumps by more than rel * max|x| between samples."""
    x = np.asarray(x, dtype=float)
    if len(x) < 2:
        return []
    amp = float(np.max(np.abs(x)))
    if amp <= 0:
        return []
    d = np.diff(x)
    idx = list(np.nonzero(np.abs(d) > rel * amp)[0] + 1)
    out = []
    for i in idx:
        if out and i - out[-1] <= 1:
            out[-1] = i
        else:
            out.append(int(i))
    return out


def edge_metrics(t, y, r, i0, i1, delta, band_abs):
    """Metrics of one edge: y responds from index i0 (the first sample after the jump) to i1."""
    n = i1 - i0
    if n < 5:
        return None
    pre = y[max(0, i0 - 3):i0]
    y0 = float(np.mean(pre)) if len(pre) else float(y[i0])
    yw = y[i0:i1]
    tw = t[i0:i1] - t[i0]          # i0: the first sample carrying the new excitation value
    k = max(3, n // 5)
    y_ss = float(np.mean(yw[-k:]))
    span = y_ss - y0
    if np.sign(span) != np.sign(delta) or abs(span) < 0.3 * abs(delta):
        span = delta          # it did not follow: normalise by the commanded step
    s = 1.0 if span > 0 else -1.0
    os_pct = max(0.0, float(np.max(s * (yw - y_ss)))) / abs(span) * 100.0
    prog = s * (yw - y0) / abs(span)
    i10 = np.nonzero(prog >= 0.1)[0]
    i90 = np.nonzero(prog >= 0.9)[0]
    rise = float(tw[i90[0]] - tw[i10[0]]) * 1000.0 if len(i10) and len(i90) else None
    band = max(BAND_FRAC * abs(delta), band_abs)
    out_idx = np.nonzero(np.abs(yw - y_ss) > band)[0]
    if len(out_idx) == 0:
        settle, settled = 0.0, True
    elif out_idx[-1] >= n - 1:
        settle, settled = None, False
    else:
        settle, settled = float(tw[out_idx[-1] + 1]) * 1000.0, True
    r_end = float(np.mean(r[i0:i1][-k:])) if r is not None else y_ss
    return {"overshoot_pct": os_pct, "rise_ms": rise, "settle_ms": settle, "settled": settled,
            "steady_err": r_end - y_ss, "y0": y0, "y_ss": y_ss, "delta": float(delta)}


def osc_metrics(t, e, fs):
    """(osc_hz or None, osc_amp_deg) of the error tail e."""
    if len(e) < 8:
        return None, 0.0
    tt = t - t[0]
    p = np.polyfit(tt, e, 1)
    d = e - np.polyval(p, tt)
    amp = float(np.sqrt(2.0) * np.std(d))
    if amp < OSC_MIN_DEG:
        return None, amp
    nfft = max(1024, 1 << int(math.ceil(math.log2(len(d)))))
    spec = np.abs(np.fft.rfft(d * np.hanning(len(d)), nfft))
    f = np.fft.rfftfreq(nfft, 1.0 / fs)
    m = (f >= 0.5) & (f <= fs / 2.5)
    if not np.any(m):
        return None, amp
    return float(f[m][np.argmax(spec[m])]), amp


# =============================================================================
# Frequency response
# =============================================================================

def _nperseg(n):
    k = 1 << int(math.floor(math.log2(max(64, n // 4))))
    return int(max(64, min(2048, k)))


def frf(u, y, x, fs):
    """H = S_uy / S_ux (x None: S_uu) and coherence(u, y), Welch, u the excitation."""
    from scipy import signal
    nps = _nperseg(len(u))
    f, s_uy = signal.csd(u, y, fs=fs, nperseg=nps, detrend="constant")
    if x is None:
        _, s_ux = signal.welch(u, fs=fs, nperseg=nps, detrend="constant")
    else:
        _, s_ux = signal.csd(u, x, fs=fs, nperseg=nps, detrend="constant")
    _, coh = signal.coherence(u, y, fs=fs, nperseg=nps, detrend="constant")
    with np.errstate(divide="ignore", invalid="ignore"):
        h = s_uy / s_ux
    return f, h, coh


def margins(f, L, valid):
    """Phase margin at |L| = 1 and gain margin at phase -180 deg over the valid bins."""
    out = {"pm_deg": None, "pm_hz": None, "gm_db": None, "gm_hz": None}
    idx = np.nonzero(valid & (f > 0) & np.isfinite(L))[0]
    if len(idx) < 3:
        return out
    ff = f[idx]
    mag = np.abs(L[idx])
    ph = np.degrees(np.unwrap(np.angle(L)))[idx]
    ph = ph - 360.0 * np.ceil((ph[0]) / 360.0) if ph[0] > 0 else ph   # start in (-360, 0]
    lm = 20 * np.log10(np.maximum(mag, 1e-12))
    for j in range(len(idx) - 1):
        if lm[j] >= 0 > lm[j + 1]:
            a = lm[j] / (lm[j] - lm[j + 1])
            fc = math.exp(math.log(ff[j]) + a * (math.log(ff[j + 1]) - math.log(ff[j])))
            pc = ph[j] + a * (ph[j + 1] - ph[j])
            out["pm_deg"], out["pm_hz"] = 180.0 + pc, fc
            break
    for j in range(len(idx) - 1):
        if (ph[j] + 180.0) * (ph[j + 1] + 180.0) <= 0 and ph[j] != ph[j + 1]:
            a = (-180.0 - ph[j]) / (ph[j + 1] - ph[j])
            out["gm_db"] = -(lm[j] + a * (lm[j + 1] - lm[j]))
            out["gm_hz"] = math.exp(math.log(ff[j]) + a * (math.log(ff[j + 1]) - math.log(ff[j])))
            break
    return out


def bandwidth(f, Tcl, valid):
    """First -3 dB crossing of |T| over the valid bins, interpolated in log f (not a bin edge)."""
    idx = np.nonzero(valid & (f > 0))[0]
    lim = -3.0103
    prev = None
    for j in idx:
        db = 20 * math.log10(max(abs(Tcl[j]), 1e-12))
        if db < lim:
            if prev is None:
                return float(f[j])
            pj, pdb = prev
            a = (pdb - lim) / (pdb - db)
            return float(math.exp(math.log(f[pj]) + a * (math.log(f[j]) - math.log(f[pj]))))
        prev = (j, db)
    return None


def frf_metrics(d, exc, i0, fs):
    """FRF of one chirp/prbs/sine trial from index i0 (the excitation start) on."""
    tgt = exc["target"]
    u = d["exc_val"][i0:]
    out = {}
    ang, ang_tgt = ctl_angle(d)
    roll = ang[i0:]          # the controlled axis (roll or pitch); key names kept for metrics.json
    f, h_roll, coh_roll = frf(u, roll, None, fs)
    out["frf_roll_exc"] = (f, h_roll, coh_roll)
    if tgt == "att":
        f, Tcl, coh = frf(u, roll, ang_tgt[i0:], fs)
        L = Tcl / (1.0 - Tcl)
        _, _, coh_x = frf(u, ang_tgt[i0:], None, fs)
    elif tgt == "rate":
        gy = ctl_rate_dps(d)[i0:]
        f, Tcl, coh = frf(u, gy, d["rate_tgt_dps"][i0:], fs)
        L = Tcl / (1.0 - Tcl)
        _, _, coh_x = frf(u, d["rate_tgt_dps"][i0:], None, fs)
    else:
        mom = d["moment_mNm"][i0:]
        uctrl = mom - u * 1e3
        f, Lneg, coh = frf(u, uctrl, mom, fs)
        L = -Lneg
        Tcl = L / (1.0 + L)
        _, _, coh_x = frf(u, mom, None, fs)
    valid = (coh > COH_MIN) & (coh_x > COH_MIN)
    lo = exc.get("f0_mHz") or 0
    hi = exc.get("f1_mHz") if exc["type"] == "chirp" else (exc.get("f0_mHz") or 0) * (
        0.4 if exc["type"] == "prbs" else 1.0)
    if exc["type"] == "chirp":
        lo, hi = min(lo, hi), max(lo, hi)
        valid &= (f >= 0.8 * lo / 1000.0) & (f <= 1.2 * hi / 1000.0)
    m = margins(f, L, valid)
    out.update(m)
    out["bw_hz"] = bandwidth(f, Tcl, valid) if tgt != "moment" else m["pm_hz"]
    out["coh_valid_bins"] = int(np.sum(valid))
    out["frf_loop"] = (f, L, coh, valid)
    out["frf_closed"] = (f, Tcl, coh, valid)
    return out


def _frf_table(f, h, coh, valid=None, n=40):
    """Log-spaced summary rows [f, mag_db, phase_deg, coh] for metrics.json."""
    pos = np.nonzero(f > 0)[0]
    if len(pos) == 0:
        return []
    picks = np.unique(np.geomspace(pos[0], pos[-1], n).astype(int))
    rows = []
    for j in picks:
        if not np.isfinite(h[j]):
            continue
        rows.append([round(float(f[j]), 4), round(20 * math.log10(max(abs(h[j]), 1e-12)), 2),
                     round(math.degrees(np.angle(h[j])), 1), round(float(coh[j]), 3)]
                    + ([bool(valid[j])] if valid is not None else []))
    return rows


# =============================================================================
# One trial
# =============================================================================

def trial_metrics(d, exc, t_exc=None, status="ok"):
    """Every metric for one trial's samples d (load_rec). Returns a JSON-able dict (+ '_frf' with
    numpy arrays for plotting, stripped before writing)."""
    t = _col(d, "t_s")
    if t is None or len(t) < 10:
        raise ValueError("too few samples (%d)" % (0 if t is None else len(t)))
    n = len(t)
    dt = float(np.median(np.diff(t)))
    fs = 1.0 / dt if dt > 0 else 250.0
    x = d["exc_val"]
    nz = np.nonzero(x != 0)[0]
    if t_exc is not None:
        i0 = int(np.searchsorted(t, t_exc - 1e-9))
    else:
        i0 = int(nz[0]) if len(nz) else 0
    tgt, typ = exc["target"], exc["type"]
    roll, rtgt = ctl_angle(d)      # the controlled axis: roll, or pitch in a rig-pitch recording
    err = rtgt - roll
    m = {"type": typ, "target": tgt, "samples": n, "fs_hz": round(fs, 1),
         "axis": AX.name(d.get("_axis", AX.ROLL))}
    m["rms_err_deg"] = float(np.sqrt(np.mean(err[i0:] ** 2))) if n > i0 else None
    duty = np.vstack([d[c] for c in ("duty0", "duty1", "duty2", "duty3") if c in d]) \
        if "duty0" in d else None
    if duty is not None:
        dw = duty[:, i0:]
        m["peak_duty"] = float(np.max(dw)) if dw.size else None
        sat = np.any((dw >= SAT_HI) | (dw <= SAT_LO), axis=0)
        m["sat_frac"] = float(np.mean(sat)) if sat.size else None
    else:
        m["peak_duty"] = m["sat_frac"] = None
    for k in ("overshoot_pct", "rise_ms", "settle_ms", "steady_err_deg", "pm_deg", "gm_db",
              "bw_hz"):
        m[k] = None
    m["settled"] = None
    dur = exc["dur_ms"] / 1000.0
    tail_from = t[i0] + dur
    if typ in ("step", "doublet"):
        if tgt == "att":
            xs, y, r = np.degrees(x), roll, rtgt
        elif tgt == "rate":
            xs, y, r = np.degrees(x), ctl_rate_dps(d), d["rate_tgt_dps"]
        else:
            xs, y, r = x, roll, rtgt
        edges = [e for e in find_edges(xs) if e >= max(1, i0 - 2)]
        if tgt != "moment":
            em = []
            for j, e in enumerate(edges):
                e1 = edges[j + 1] if j + 1 < len(edges) else n
                delta = float(xs[e] - xs[e - 1])
                r_m = edge_metrics(t, y, r, e, e1, delta, BAND_ABS[tgt])
                if r_m is not None:
                    em.append(r_m)
            if em:
                m["overshoot_pct"] = max(e["overshoot_pct"] for e in em)
                rises = [e["rise_ms"] for e in em if e["rise_ms"] is not None]
                m["rise_ms"] = max(rises) if rises else None
                m["settled"] = all(e["settled"] for e in em)
                m["settle_ms"] = max(e["settle_ms"] for e in em) if m["settled"] else None
                worst = max(em, key=lambda e: abs(e["steady_err"]))
                m["steady_err_deg"] = worst["steady_err"] if tgt == "att" else \
                    float(np.mean(err[-max(3, n // 10):]))
                if tgt == "rate":
                    m["rate_steady_err_dps"] = worst["steady_err"]
                m["edges"] = [{k: (round(v, 3) if isinstance(v, float) else v) for k, v in e.items()}
                              for e in em]
                last = edges[-1]
                settle_s = (m["settle_ms"] or 0.0) / 1000.0 if m["settled"] else 0.0
                tail_from = max(t[last] + settle_s, t[last] + 0.3)
        else:
            if edges:
                last = edges[-1]
                ew = np.abs(err[last:])
                out_idx = np.nonzero(ew > BAND_ABS["moment"])[0]
                if len(out_idx) == 0:
                    m["settle_ms"], m["settled"] = 0.0, True
                elif out_idx[-1] >= len(ew) - 1:
                    m["settled"] = False
                else:
                    m["settle_ms"] = float(t[last + out_idx[-1] + 1] - t[last]) * 1000.0
                    m["settled"] = True
                m["peak_dev_deg"] = float(np.max(np.abs(err[i0:])))
                tail_from = t[last] + (m["settle_ms"] or 300.0) / 1000.0
            m["steady_err_deg"] = float(np.mean(err[-max(3, n // 10):]))
    tail = t >= tail_from
    if np.sum(tail) < int(0.4 * fs):
        tail = t >= t[-1] - 0.4
    m["osc_hz"], m["osc_amp_deg"] = osc_metrics(t[tail], err[tail], fs)
    frf_plot = None
    if typ in FRF_TYPES:
        try:
            fm = frf_metrics(d, exc, i0, fs)
        except Exception as exc_:  # noqa: BLE001 - an FRF that cannot be estimated is not fatal
            m["frf_error"] = str(exc_)
        else:
            for k in ("pm_deg", "pm_hz", "gm_db", "gm_hz", "bw_hz", "coh_valid_bins"):
                m[k] = fm.get(k)
            f, h, coh = fm["frf_roll_exc"]
            m["frf_roll_exc"] = _frf_table(f, h, coh)
            f, L, coh, valid = fm["frf_loop"]
            m["frf_loop"] = _frf_table(f, L, coh, valid)
            frf_plot = fm
    m["status"] = status
    ok = status in ("ok", None)
    m["score"] = score(m) if ok else None
    m["margin_penalty"] = margin_penalty(m) if ok else None
    for k, v in list(m.items()):
        if isinstance(v, (np.floating, np.integer)):
            m[k] = v.item()
        if isinstance(m[k], float):
            m[k] = round(m[k], 4) if math.isfinite(m[k]) else None
    m["_frf"] = frf_plot
    return m


def score(m):
    """The documented scalar (module docstring); lower is better."""
    sat = m.get("sat_frac") or 0.0
    osc = max(0.0, (m.get("osc_amp_deg") or 0.0) - OSC_FLOOR_DEG)
    if m.get("type") in FRF_TYPES:
        return None
    rms = m.get("rms_err_deg")
    if rms is None:
        return None
    s = rms + 0.05 * (m.get("overshoot_pct") or 0.0) + 20.0 * sat + 10.0 * osc
    if m.get("settled") is False:
        s += 3.0
    elif m.get("settle_ms") is not None:
        s += 0.3 * m["settle_ms"] / 1000.0
    return round(s, 4)


def margin_penalty(m):
    """FRF trials: how far the loop is from PM 45 deg / GM 6 dB (module docstring)."""
    if m.get("type") not in FRF_TYPES:
        return None
    sat = m.get("sat_frac") or 0.0
    osc = max(0.0, (m.get("osc_amp_deg") or 0.0) - OSC_FLOOR_DEG)
    pm, gm = m.get("pm_deg"), m.get("gm_db")
    s = 5.0 if pm is None else max(0.0, 45.0 - pm) / 10.0
    if gm is not None:
        s += max(0.0, 6.0 - gm) / 2.0
    return round(s + 20.0 * sat + 10.0 * osc, 4)


def analyze_trial(tdir, meta, plots=False):
    """Compute and write trials/<n>/metrics.json (and plots). Returns the metrics dict."""
    tdir = Path(tdir)
    d = load_rec(tdir / "rec.csv")
    m = trial_metrics(d, meta["exc"], meta.get("t_exc"), meta.get("status") or "ok")
    frf_plot = m.pop("_frf", None)
    m["trial"] = meta.get("trial")
    m["gains"] = meta.get("gains")
    m["exc"] = meta.get("exc")
    T.write_json_atomic(tdir / "metrics.json", m)
    if plots:
        try:
            plot_trial(tdir, d, meta, m, frf_plot)
        except Exception as exc:  # noqa: BLE001 - plots are a convenience
            print("rig analyze: trial %s plot failed: %s" % (meta.get("trial"), exc), file=sys.stderr)
    return m


# =============================================================================
# Plots (matplotlib if present)
# =============================================================================

def _plt():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return plt
    except Exception:  # noqa: BLE001
        return None


def plot_trial(tdir, d, meta, m, frf_plot):
    plt = _plt()
    if plt is None:
        return
    t = d["t_s"]
    axn = AX.name(d.get("_axis", AX.ROLL))
    ang, ang_tgt = ctl_angle(d)
    fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    ax[0].plot(t, ang_tgt, label="%s target" % axn, color="#888888", lw=1)
    ax[0].plot(t, ang, label=axn, color="#1f5fbf", lw=1.2)
    ax[0].set_ylabel("%s (deg)" % axn)
    ax[0].legend(loc="upper right", fontsize=8)
    ax[0].set_title("%s trial %s  %s  score %s" % (axn.upper(), meta.get("trial"),
                                                   RR.exc_line(meta["exc"]), m.get("score")),
                    fontsize=10)
    ax[1].plot(t, d["exc_val"], color="#b05f00", lw=1)
    ax[1].set_ylabel("exc (SI)")
    for i in range(4):
        c = "duty%d" % i
        if c in d:
            ax[2].plot(t, d[c], lw=0.8, label=c)
    ax[2].axhline(SAT_HI, color="#c00000", lw=0.6, ls="--")
    ax[2].set_ylabel("duty")
    ax[2].set_xlabel("t (s)")
    ax[2].legend(loc="upper right", fontsize=7, ncol=4)
    fig.tight_layout()
    fig.savefig(tdir / "plot.png", dpi=90)
    plt.close(fig)
    if frf_plot:
        f, L, coh, valid = frf_plot["frf_loop"]
        pos = f > 0
        fig, ax = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
        ax[0].semilogx(f[pos], 20 * np.log10(np.abs(L[pos]) + 1e-12), color="#cccccc", lw=0.8)
        ax[0].semilogx(f[pos & valid], 20 * np.log10(np.abs(L[pos & valid]) + 1e-12), "o-",
                       color="#1f5fbf", ms=3, lw=1)
        ax[0].axhline(0, color="#888888", lw=0.6)
        ax[0].set_ylabel("|L| dB")
        ax[0].set_title("%s loop L  PM %s deg @ %s Hz  GM %s dB" % (
            axn, m.get("pm_deg"), m.get("pm_hz"), m.get("gm_db")), fontsize=10)
        ph = np.degrees(np.unwrap(np.angle(L)))
        ax[1].semilogx(f[pos], ph[pos], color="#cccccc", lw=0.8)
        ax[1].semilogx(f[pos & valid], ph[pos & valid], "o-", color="#1f5fbf", ms=3, lw=1)
        ax[1].axhline(-180, color="#888888", lw=0.6)
        ax[1].set_ylabel("phase deg")
        ax[2].semilogx(f[pos], coh[pos], color="#b05f00", lw=1)
        ax[2].axhline(COH_MIN, color="#c00000", lw=0.6, ls="--")
        ax[2].set_ylabel("coherence")
        ax[2].set_xlabel("Hz")
        fig.tight_layout()
        fig.savefig(tdir / "frf.png", dpi=90)
        plt.close(fig)


def plot_summary(bdir, points, varied):
    plt = _plt()
    if plt is None or not varied:
        return None
    fig, axs = plt.subplots(1, len(varied), figsize=(4 * len(varied), 3.2), squeeze=False)
    for ax, name in zip(axs[0], varied):
        xs = [p["gains"].get(name) for p in points if p["score"] is not None]
        ys = [p["score"] for p in points if p["score"] is not None]
        ax.plot(xs, ys, "o", color="#1f5fbf")
        ax.set_xlabel(name)
        ax.set_ylabel("score (lower is better)")
    fig.tight_layout()
    out = Path(bdir) / "summary.png"
    fig.savefig(out, dpi=90)
    plt.close(fig)
    return out


# =============================================================================
# The batch
# =============================================================================

def _gkey(g):
    return json.dumps({k: g[k] for k in sorted(g)}, sort_keys=True)


def _fmt(v, nd=2):
    if v is None:
        return "-"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return "%.*f" % (nd, v)
    return str(v)


def next_grid(points, best, varied):
    """Halve the step around the best point, per varied gain, clipped to the firmware ranges."""
    if best is None:
        return {}
    out = {}
    for name in varied:
        vals = sorted({p["gains"][name] for p in points if name in p["gains"]})
        b = best["gains"].get(name)
        others = [v for v in vals if v != b]
        if b is None or not others:
            continue
        step = min(abs(v - b) for v in others)
        lo, hi = RR.GAIN_RANGE[name]
        cand = []
        for v in (b - step / 2.0, b, b + step / 2.0):
            v = min(hi, max(lo, v))
            v = float("%.4g" % v)
            if v not in cand:
                cand.append(v)
        out[name] = cand
    return out


def batch_axis(meta):
    """The batch's axis name from batch.json (top level, else spec), roll for older batches."""
    for v in (meta.get("axis"), (meta.get("spec") or {}).get("axis")):
        a = AX.parse_axis(v)
        if a is not None:
            return AX.name(a)
    return AX.name(AX.ROLL)


def analyze_batch(bdir, plots=True, write_journal=True, quiet=False):
    bdir = Path(bdir)
    meta = T.read_json(bdir / "batch.json", {}) or {}
    axis = batch_axis(meta)
    events, _ = T.load_journal(bdir / "journal.jsonl")
    status, starts, last_metrics = {}, {}, {}
    for e in events:
        if e.get("ev") == "trial_start":
            starts[e.get("trial")] = e
        elif e.get("ev") == "trial_end":
            status[e.get("trial")] = e.get("status")
        elif e.get("ev") == "metrics":
            last_metrics[e.get("trial")] = e
    trials = []
    tdirs = sorted((p for p in (bdir / "trials").iterdir() if p.name.isdigit()),
                   key=lambda p: int(p.name)) if (bdir / "trials").is_dir() else []
    for tdir in tdirs:
        n = int(tdir.name)
        tm = T.read_json(tdir / "trial.json", None)
        if tm is None:
            st = starts.get(n) or {}
            tm = {"trial": n, "gains": st.get("gains") or {}, "exc": st.get("exc") or {},
                  "matched": True}
        tm["status"] = status.get(n, tm.get("status"))
        row = {"trial": n, "gains": tm.get("gains") or {}, "exc": tm.get("exc") or {},
               "status": tm["status"], "metrics": None, "note": None}
        if not tm.get("matched", True):
            row["note"] = tm.get("reason")
        elif not (tdir / "rec.csv").is_file():
            row["note"] = "no rec.csv"
        else:
            try:
                m = analyze_trial(tdir, tm, plots=plots)
                row["metrics"] = m
                if m.get("axis") and m["axis"] != axis:
                    row["note"] = "rec.csv is a %s recording in a %s batch" % (m["axis"], axis)
                if write_journal:
                    ev = {k: m.get(k) for k in JOURNAL_KEYS}
                    old = last_metrics.get(n) or {}
                    if any(old.get(k) != ev[k] for k in JOURNAL_KEYS):
                        T.append_journal(bdir / "journal.jsonl",
                                         dict({"t": round(__import__("time").time(), 3),
                                               "ev": "metrics", "trial": n, "recomputed": True}, **ev))
            except Exception as exc:  # noqa: BLE001
                row["note"] = "metrics failed: %s" % exc
        trials.append(row)
    # gain points
    pts = {}
    for r in trials:
        k = _gkey(r["gains"])
        p = pts.setdefault(k, {"gains": r["gains"], "trials": [], "scores": [], "penalties": []})
        p["trials"].append(r["trial"])
        s = (r["metrics"] or {}).get("score")
        if s is not None:
            p["scores"].append(s)
        mp = (r["metrics"] or {}).get("margin_penalty")
        if mp is not None:
            p["penalties"].append(mp)
    points = []
    for p in pts.values():
        if p["scores"] or p["penalties"]:
            p["score"] = (float(np.mean(p["scores"])) if p["scores"] else 0.0) + \
                (float(np.mean(p["penalties"])) if p["penalties"] else 0.0)
        else:
            p["score"] = None
        rows = [r for r in trials if r["trial"] in p["trials"] and r["metrics"]]

        def mean(key):
            v = [r["metrics"].get(key) for r in rows if r["metrics"].get(key) is not None]
            return float(np.mean(v)) if v else None
        p.update({k: mean(k) for k in ("overshoot_pct", "rise_ms", "settle_ms", "rms_err_deg",
                                        "sat_frac", "osc_amp_deg", "pm_deg", "gm_db")})
        points.append(p)
    points.sort(key=lambda p: (p["score"] is None, p["score"] if p["score"] is not None else 0))
    allg = [p["gains"] for p in points]
    names = sorted({k for g in allg for k in g}, key=lambda n: RR.GAIN_NAMES.index(n)
                   if n in RR.GAIN_NAMES else 99)
    varied = [n for n in names if len({g.get(n) for g in allg}) > 1]
    best = next((p for p in points if p["score"] is not None), None)
    ng = next_grid(points, best, varied)
    summary_png = plot_summary(bdir, points, varied) if plots else None
    text = render_summary(bdir, meta, trials, points, varied, best, ng, summary_png, axis)
    (bdir / "summary.md").write_text(text, encoding="utf-8")
    nxt = None
    if ng:
        spec = dict(meta.get("spec") or {})
        nxt = {"name": (meta.get("name") or "rig") + "-refine", "axis": axis,
               "collective_permille": spec.get("collective_permille"),
               "roll_sp_mdeg": spec.get("roll_sp_mdeg", 0),
               "base_gains": {k: v for k, v in (best["gains"] if best else {}).items() if k not in ng},
               "grid_product": ng, "excitations": spec.get("excitations"),
               "repeats": spec.get("repeats", 1),
               "arrangement": spec.get("arrangement") or {"per_spin": 1},
               "limits": dict(spec.get("limits") or {}, settle_ms=spec.get("settle_ms", 1500))}
        if spec.get("mass_kg"):
            nxt["mass_kg"] = spec["mass_kg"]
        T.write_json_atomic(bdir / "next_grid.json", nxt)
    if not quiet:
        print("axis: %s" % axis.upper())
        print(render_table(points, varied, names))
        if ng:
            print("\nnext grid (halved step around the best point): %s" % json.dumps(ng))
            print("  spec written to %s" % (bdir / "next_grid.json"))
        print("summary: %s" % (bdir / "summary.md"))
    return {"trials": trials, "points": points, "best": best, "next_grid": ng, "next_spec": nxt,
            "varied": varied, "axis": axis}


def render_table(points, varied, names):
    cols = varied or names[:3]
    head = ["#"] + cols + ["n", "score", "OS%", "rise", "settle", "rms", "sat%", "osc", "PM"]
    lines = []
    for i, p in enumerate(points, 1):
        lines.append([str(i)] + [_fmt(p["gains"].get(c), 3) for c in cols] + [
            str(len(p["trials"])), _fmt(p["score"], 3), _fmt(p["overshoot_pct"], 1),
            _fmt(p["rise_ms"], 0), _fmt(p["settle_ms"], 0), _fmt(p["rms_err_deg"], 2),
            _fmt(None if p["sat_frac"] is None else p["sat_frac"] * 100, 1),
            _fmt(p["osc_amp_deg"], 2), _fmt(p["pm_deg"], 0)])
    w = [max(len(h), *(len(r[i]) for r in lines)) if lines else len(h) for i, h in enumerate(head)]
    out = ["  ".join(h.rjust(w[i]) for i, h in enumerate(head))]
    out += ["  ".join(c.rjust(w[i]) for i, c in enumerate(r)) for r in lines]
    return "\n".join(out)


def render_summary(bdir, meta, trials, points, varied, best, ng, summary_png, axis="roll"):
    spec = meta.get("spec") or {}
    L = ["# Rig batch %s (%s)" % (meta.get("name") or Path(bdir).name, axis.upper()), ""]
    L.append("- axis: **%s**%s" % (axis.upper(), " (rig-pitch: the drone turned 90 deg on the pivot; "
                                   "roll is the locked axis)" if axis == "pitch" else ""))
    L.append("- directory: `%s`" % bdir)
    L.append("- ELF: `%s` (sha1 %s), mode %s%s" % (meta.get("elf"), meta.get("elf_sha1"),
                                                  meta.get("mode"),
                                                  ", DRY RUN (fake drone)" if meta.get("dry_run") else ""))
    L.append("- approval: %s" % meta.get("approval_note"))
    L.append("- collective %s permille, %s setpoint %s mdeg, %d trial(s), %d gain point(s)"
             % (spec.get("collective_permille"), axis, spec.get("roll_sp_mdeg"), len(trials),
                len(points)))
    L.append("")
    L.append("## Ranking (mean score per gain point; lower is better)")
    L.append("")
    L.append("```")
    names = sorted({k for p in points for k in p["gains"]})
    L.append(render_table(points, varied, names))
    L.append("```")
    L.append("")
    if best:
        L.append("Best: `%s` (score %.3f over %d scored trial(s), %d FRF penalty term(s))."
                 % (json.dumps(best["gains"], sort_keys=True), best["score"], len(best["scores"]),
                    len(best["penalties"])))
        L.append("")
        L.append("Back to a flight preset (fc_rig.cpp): TAU_ROLL = 1/(2*ATT_P), TAU_ROLLRATE = 1/RATE_P; "
                 "the flown PID has no roll I/D or gyro LPF.")
        if axis == "pitch":
            L.append("")
            L.append("PITCH: controller_pid.cpp flies pitch with the ROLL constants (tau_pitch = "
                     "TAU_ROLL, tau_pitchRate = TAU_ROLLRATE, J[1] = J[0]); a pitch tune that differs "
                     "from the roll one needs separate pitch presets before it can fly.")
        L.append("")
    L.append("## Recommended next grid")
    L.append("")
    if ng:
        L.append("Halve the step around the best point (per varied gain, clipped to the firmware range):")
        L.append("")
        L.append("```json")
        L.append(json.dumps({"grid_product": ng}, indent=1))
        L.append("```")
        L.append("")
        L.append("A complete spec is in `next_grid.json`: check it, then "
                 "`./rb rig run %s/next_grid.json --i-have-approval ...`." % bdir)
    else:
        L.append("No refinement: fewer than two values of any gain were scored.")
    L.append("")
    L.append("## Trials")
    L.append("")
    L.append("| # | gains | exc | status | OS% | rise ms | settle ms | rms deg | sat % | osc Hz | PM deg | score | note |")
    L.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for r in trials:
        m = r["metrics"] or {}
        L.append("| %s | `%s` | `%s` | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
            r["trial"], " ".join("%s=%s" % (k, RR.format_gain(v)) for k, v in sorted(r["gains"].items())),
            RR.exc_line(r["exc"]) if r["exc"].get("type") else "-", r["status"] or "-",
            _fmt(m.get("overshoot_pct"), 1), _fmt(m.get("rise_ms"), 0), _fmt(m.get("settle_ms"), 0),
            _fmt(m.get("rms_err_deg"), 2),
            _fmt(None if m.get("sat_frac") is None else m["sat_frac"] * 100, 1),
            _fmt(m.get("osc_hz"), 1), _fmt(m.get("pm_deg"), 0), _fmt(m.get("score"), 3),
            r["note"] or ""))
    L.append("")
    if summary_png:
        L.append("![score vs gains](summary.png)")
        L.append("")
    L.append("Plots: `trials/<n>/plot.png` (%s, excitation, duty) and `trials/<n>/frf.png` (loop "
             "Bode + coherence for chirp/PRBS/sine)." % axis)
    L.append("")
    L.append("## Score")
    L.append("")
    L.append("```")
    L.append("step/doublet: rms_err_deg + 0.05*OS% + 0.3*settle_s + 20*sat_frac "
             "+ 10*max(0, osc_amp_deg-0.2) + 3*(not settled)")
    L.append("chirp/prbs/sine: no score; margin_penalty = max(0,45-PM)/10 + max(0,6-GM_dB)/2 "
             "+ 20*sat_frac + 10*max(0, osc_amp_deg-0.2)  (5 if no PM)")
    L.append("gain point: mean(step-like scores) + mean(margin penalties)")
    L.append("no score for estop / aborted / timeout trials")
    L.append("```")
    return "\n".join(L) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description="rig batch analysis (metrics, ranking, next grid)")
    ap.add_argument("batch", help="results/rig/<stamp>-<name>")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--no-journal", action="store_true", help="do not append metrics events")
    a = ap.parse_args(argv)
    if not Path(a.batch).is_dir():
        print("rig analyze: no such batch directory: %s" % a.batch, file=sys.stderr)
        return 2
    analyze_batch(a.batch, plots=not a.no_plots, write_journal=not a.no_journal)
    return 0


if __name__ == "__main__":
    sys.exit(main())
