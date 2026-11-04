#!/usr/bin/env python3
"""Solve FLOW_RAD_PER_COUNT from PMW3901 slide captures, or validate one.

Reads the `FLOWCAL RUN ...` lines that workloads/pmw3901_test prints, applies
the run-rejection rules from docs/flight-calibration-procedures.md, and solves

    K = D / (h * |sum(counts)|)

for each surviving run, where D is the measured slide distance in metres and h
the measured sensor height in metres. Both are held fixed across a batch, which
is what makes one --distance and one --height enough for the whole log.

WHY A SUM OF COUNTS AND NOT A VELOCITY
--------------------------------------
flow.c computes `ang = -dx * K / dt` and main.cpp turns that into `v = ang * h`,
so `v = -dx * K * h / dt` and the integral over a slide is `D = K * h * sum(dx)`.

    dt CANCELS.

That is the whole trick: the constant falls out of a hand slide with no motion
rig, no timing accuracy and no sensitivity to the sensor's poll rate, as long as
every count between the start and the end of the slide is summed. The motion
burst returns the deltas accumulated since the last read and clears them, so a
slower poll returns BIGGER deltas rather than fewer counts.

Per sample the small-angle model is exact enough by construction: at a hand
slide of ~0.2 m/s, 10 ms and h = 0.15 m each sample subtends ~0.013 rad. The
slide's TOTAL angle is irrelevant because the sensor re-acquires features
continuously and the tool integrates increments -- which is why a 1 m slide at
0.15 m height is a legitimate measurement and not a wild extrapolation.

USAGE
-----
    ./tools/flow-cal.py --distance 1.000 --height 0.150 --axis x capture.log
    ./tools/flow-cal.py --distance 0.600 --height 0.250 --axis x \\
        --validate 0.00198 validation.log
    ./tools/flow-cal.py --self-test        # no hardware, no log: proves the maths

`-` reads the log from stdin. --axis picks which count sum is the measurement;
the other axis is checked for leakage and a run that leaks too much is rejected,
because a slide that was not straight measures the wrong distance.
"""

from __future__ import annotations

import argparse
import random
import re
import statistics
import sys
from dataclasses import dataclass, field

RUN_RE = re.compile(r"^.*?FLOWCAL\s+RUN\s+(.*)$")
FIELD_RE = re.compile(r"([A-Za-z_]+)=(-?\d+)")

# Every field the rejection rules and the arithmetic below read. A line missing
# one of these is a truncated console capture, not a run: it is reported and
# skipped rather than raising a KeyError halfway through a batch.
REQUIRED = (
    "id", "ms", "n", "err", "sumdx", "sumdy", "peakdx", "peakdy",
    "n_ok", "okdx", "okdy", "sq_min", "sq_max", "sq_mean",
    "sh_min", "sh_max", "sh_mean",
)


@dataclass
class Thresholds:
    """Each one is a reason a run did not measure what it was supposed to
    measure -- NOT a reason the sensor is bad. All are overridable from the
    command line, because none of them has been checked on this airframe."""

    min_counts: int = 200       # below this, one count of quantisation shows
    max_leak: float = 0.15      # cross-axis sum as a fraction of the measured axis
    min_squal_mean: int = 25    # comfortably above flow.c's floor of 19
    max_squal_spread: int = 60  # sq_max - sq_min: the surface changed mid-slide
    max_shutter_ratio: float = 4.0   # sh_max / sh_min: the LIGHTING changed
    saturation_peak: int = 8000      # per-sample |count| approaching int16
    min_run_ms: int = 500       # a shorter "slide" was a flick
    outlier_sigma: float = 3.0


MAD_K = 1.4826   # MAD -> sigma for a normal distribution


@dataclass
class Run:
    fields: dict[str, int] = field(default_factory=dict)

    def __getitem__(self, key: str) -> int:
        return self.fields[key]

    @property
    def id(self) -> int:
        return self.fields.get("id", 0)


def parse(text: str) -> tuple[list[Run], list[str]]:
    """Return the usable runs and one complaint per malformed FLOWCAL RUN line."""
    runs: list[Run] = []
    complaints: list[str] = []
    for lineno, line in enumerate(text.splitlines(), 1):
        m = RUN_RE.match(line)
        if not m:
            continue
        fields = {k: int(v) for k, v in FIELD_RE.findall(m.group(1))}
        missing = [k for k in REQUIRED if k not in fields]
        if missing:
            complaints.append(
                f"line {lineno}: FLOWCAL RUN line is missing {', '.join(missing)} "
                "-- truncated capture, skipped"
            )
            continue
        runs.append(Run(fields))
    return runs, complaints


def reject(run: Run, axis: str, thr: Thresholds) -> str | None:
    """Why this run cannot be used, or None if it can."""
    measured = run["okdx"] if axis == "x" else run["okdy"]
    other = run["okdy"] if axis == "x" else run["okdx"]
    peak = run["peakdx"] if axis == "x" else run["peakdy"]

    if run["n"] == 0:
        return "no samples"
    if run["err"] != 0:
        return f"{run['err']} read error(s): counts were lost"
    if run["n_ok"] != run["n"]:
        return (
            f"{run['n'] - run['n_ok']} of {run['n']} samples below the SQUAL "
            "floor: flight would have discarded them"
        )
    if run["ms"] < thr.min_run_ms:
        return (
            f"the run lasted {run['ms']} ms (need {thr.min_run_ms}): that was a "
            "flick, not a controlled slide"
        )
    if abs(measured) < thr.min_counts:
        return (f"only {abs(measured)} counts on the {axis} axis "
                f"(need {thr.min_counts})")
    if abs(other) > thr.max_leak * abs(measured):
        return (
            f"cross-axis leak {abs(other)}/{abs(measured)} = "
            f"{abs(other) / abs(measured):.0%}: the slide was not straight"
        )
    if run["sq_mean"] < thr.min_squal_mean:
        return f"mean SQUAL {run['sq_mean']} below {thr.min_squal_mean}"
    if run["sq_max"] - run["sq_min"] > thr.max_squal_spread:
        return (
            f"SQUAL swung {run['sq_min']}..{run['sq_max']}: the surface "
            "changed mid-slide"
        )
    if run["sh_min"] > 0 and run["sh_max"] > thr.max_shutter_ratio * run["sh_min"]:
        return (
            f"shutter swung {run['sh_min']}..{run['sh_max']} "
            f"({run['sh_max'] / run['sh_min']:.1f}x): the LIGHTING or the height "
            "changed mid-slide, so h was not the constant the solve assumes"
        )
    if peak >= thr.saturation_peak:
        return f"peak per-sample count {peak} near int16 saturation"
    return None


def solve(runs, axis: str, distance: float, height: float, thr: Thresholds):
    kept, dropped = [], []
    for run in runs:
        why = reject(run, axis, thr)
        counts = run["okdx"] if axis == "x" else run["okdy"]
        if why is not None:
            dropped.append((run, why))
            continue
        kept.append((run, counts, distance / (height * abs(counts))))
    return kept, dropped


def robust_outliers(kept, thr: Thresholds):
    """Flag runs whose K is far from the median, by MAD rather than by sigma.

    A mean and a standard deviation are both dragged by the very outlier they
    are meant to find; the median and the MAD are not, which matters when a
    batch is six runs long and one of them was a stumble.
    """
    if len(kept) < 4:
        return kept, []
    ks = [k for _, _, k in kept]
    med = statistics.median(ks)
    mad = statistics.median([abs(k - med) for k in ks])
    if mad == 0.0:
        return kept, []
    sigma = MAD_K * mad
    inliers, outliers = [], []
    for row in kept:
        if abs(row[2] - med) > thr.outlier_sigma * sigma:
            outliers.append((row, abs(row[2] - med) / sigma))
        else:
            inliers.append(row)
    return inliers, outliers


def report_batch(runs, complaints, kept, dropped) -> None:
    for c in complaints:
        print(f"  SKIP {c}")
    print(f"{len(runs)} run(s) parsed, {len(kept)} usable, {len(dropped)} rejected")
    for run, why in dropped:
        print(f"  REJECT id={run.id}: {why}")


def do_fit(runs, complaints, kept, dropped, args, thr) -> int:
    inliers, outliers = robust_outliers(kept, thr)
    for row, dev in outliers:
        run, _, k = row
        print(f"  OUTLIER id={run.id}: K={k:.6f} is {dev:.1f} robust sigma "
              "from the median -- dropped from the fit")

    ks = [k for _, _, k in inliers]
    counts = [c for _, c, _ in inliers]
    med = statistics.median(ks)
    mean = statistics.fmean(ks)
    spread = statistics.pstdev(ks) if len(ks) > 1 else 0.0
    sign = "negative" if statistics.fmean([float(c) for c in counts]) < 0 else "positive"

    print()
    print(f"{'id':>4} {'counts':>9} {'SQUAL':>6} {'shutter':>8} {'K (rad/count)':>15}")
    for run, c, k in inliers:
        print(f"{run.id:>4} {c:>9} {run['sq_mean']:>6} {run['sh_mean']:>8} {k:>15.6f}")
    print()
    print(f"median  K = {med:.6f} rad/count   <-- use this")
    print(f"mean    K = {mean:.6f}")
    if med:
        print(f"spread    = {spread:.6f} ({spread / med:.1%} of the median)")
    print(f"count sign on the {args.axis} axis: {sign}")
    print()
    print("Set it with, e.g.:")
    print(f'  RB_EXTRA_CPPFLAGS="-DFLOW_RAD_PER_COUNT={med:.6f}f"')
    print("or change the #ifndef default in")
    print("  workloads/flight_controller/src/flow.c")
    if len(inliers) < 5:
        print()
        print(f"WARNING: only {len(inliers)} run(s) survived. The procedure asks for "
              "at least 5 usable runs; below that the median is not a median.")
    if spread and med and spread / med > 0.10:
        print()
        print("WARNING: the runs disagree by more than 10%. That is the slide or "
              "the surface, not the sensor -- redo the batch before trusting it.")
    return 0


def do_validate(kept, args, thr) -> int:
    """Check a constant against a batch the fit never saw.

    Deliberately over EVERY physically-valid run, with no statistical outlier
    removal. The fit may drop an outlier -- a fit is allowed to choose its data.
    A validation may not: dropping the runs that disagree is exactly how a
    constant passes a test it should have failed. Outliers are named, and then
    counted anyway.
    """
    k = args.validate
    print(f"Validating K = {k:.6f} rad/count against "
          f"D = {args.distance} m at h = {args.height} m")
    print("(every usable run counts; nothing is dropped for disagreeing)")
    print()
    print(f"{'id':>4} {'counts':>9} {'SQUAL':>6} {'predicted D':>13} {'error':>9}")
    errs = []
    for run, counts, _ in kept:
        pred = k * args.height * abs(counts)
        err = (pred - args.distance) / args.distance
        errs.append(err)
        print(f"{run.id:>4} {counts:>9} {run['sq_mean']:>6} {pred:>11.4f} m {err:>8.1%}")
    bias = statistics.fmean(errs)
    spread = statistics.pstdev(errs) if len(errs) > 1 else 0.0
    worst = max(abs(e) for e in errs)
    print()
    print(f"mean error {bias:+.1%}, spread {spread:.1%}, worst {worst:.1%} "
          f"over {len(errs)} run(s)")
    # 10% is the band the doc's validation section argues for: a 30 s hover's
    # flow-driven position drift then stays inside the tether.
    ok = abs(bias) <= args.tolerance and spread <= args.tolerance
    print(f"tolerance: |mean| and spread both <= {args.tolerance:.0%}")
    print("VERDICT:", "PASS" if ok else "FAIL -- recalibrate")
    if len(errs) < 3:
        print("WARNING: fewer than 3 runs. A PASS here means very little.")
    return 0 if ok else 1


def synth(k_true: float, distance: float, height: float, n_runs: int,
          scatter: float, seed: int) -> str:
    """A synthetic capture log, in the exact format pmw3901_test prints.

    This exists so the solver can be exercised end to end on a machine with no
    drone attached -- which is every machine this tool was written on.
    """
    rng = random.Random(seed)
    lines = ["(synthetic capture -- no hardware was involved)"]
    for i in range(1, n_runs + 1):
        ideal = distance / (k_true * height)
        counts = -int(round(ideal * (1.0 + rng.gauss(0.0, scatter))))
        n = 600 + rng.randrange(0, 200)
        leak = int(round(counts * rng.gauss(0.0, 0.02)))
        peak = max(1, int(abs(counts) / n) + 3)
        sq = 55 + rng.randrange(-6, 6)
        sh = 1800 + rng.randrange(-200, 200)
        lines.append(
            f"FLOWCAL RUN id={i} ms={n * 10} n={n} err=0 "
            f"sumdx={counts} sumdy={leak} absdx={abs(counts)} absdy={abs(leak)} "
            f"peakdx={peak} peakdy={max(1, abs(leak) // n + 1)} "
            f"n_ok={n} okdx={counts} okdy={leak} "
            f"sq_min={sq - 4} sq_max={sq + 4} sq_mean={sq} "
            f"sh_min={sh - 120} sh_max={sh + 120} sh_mean={sh}"
        )
    return "\n".join(lines) + "\n"


def self_test() -> int:
    """Recover a known constant from synthetic counts, then validate it.

    Proves the arithmetic and the parser, and NOTHING about the sensor.
    """
    k_true, thr = 0.00198, Thresholds()
    fit_log = synth(k_true, 1.000, 0.150, 6, 0.015, seed=1)
    val_log = synth(k_true, 0.600, 0.250, 4, 0.015, seed=2)

    runs, _ = parse(fit_log)
    kept, dropped = solve(runs, "x", 1.000, 0.150, thr)
    if dropped:
        print("SELF-TEST FAIL: clean synthetic runs were rejected:")
        for run, why in dropped:
            print(f"  id={run.id}: {why}")
        return 1
    inliers, _ = robust_outliers(kept, thr)
    med = statistics.median([k for _, _, k in inliers])
    fit_err = (med - k_true) / k_true
    print(f"fit      : {len(inliers)} synthetic runs at D=1.000 m h=0.150 m")
    print(f"           K_true = {k_true:.6f}, K_solved = {med:.6f} "
          f"({fit_err:+.2%})")

    runs, _ = parse(val_log)
    kept, _ = solve(runs, "x", 0.600, 0.250, thr)
    errs = [(med * 0.250 * abs(c) - 0.600) / 0.600 for _, c, _ in kept]
    bias = statistics.fmean(errs)
    print(f"validate : {len(kept)} synthetic runs at D=0.600 m h=0.250 m, "
          f"mean error {bias:+.2%}")

    # A 1.5% per-run scatter over 6 runs cannot honestly leave more than a few
    # percent on the median; anything larger means the arithmetic changed.
    ok = abs(fit_err) < 0.03 and abs(bias) < 0.03
    print("SELF-TEST:", "PASS" if ok else "FAIL")
    print("This exercises the solver only. It says nothing about the sensor, "
          "the surface or the airframe.")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", nargs="?",
                    help="console log with FLOWCAL RUN lines, or - for stdin")
    ap.add_argument("--distance", type=float,
                    help="measured slide distance, metres")
    ap.add_argument("--height", type=float,
                    help="measured sensor height above the surface, metres")
    ap.add_argument("--axis", choices=("x", "y"), default="x",
                    help="which count sum is the measurement (default x)")
    ap.add_argument("--validate", type=float, metavar="K",
                    help="check an existing constant instead of solving for one")
    ap.add_argument("--tolerance", type=float, default=0.10,
                    help="validation pass band on mean error and spread "
                         "(default 0.10 = 10%%)")
    ap.add_argument("--self-test", action="store_true",
                    help="run the solver against synthetic counts and exit")
    ap.add_argument("--min-counts", type=int, default=Thresholds.min_counts)
    ap.add_argument("--min-squal", type=int, default=Thresholds.min_squal_mean)
    ap.add_argument("--max-leak", type=float, default=Thresholds.max_leak)
    args = ap.parse_args()

    if args.self_test:
        return self_test()
    missing = [n for n in ("log", "distance", "height") if getattr(args, n) is None]
    if missing:
        ap.error("missing required argument(s): " + ", ".join(missing))
    if args.height <= 0.0 or args.distance <= 0.0:
        ap.error("--distance and --height must both be positive metres")

    thr = Thresholds(min_counts=args.min_counts, max_leak=args.max_leak,
                     min_squal_mean=args.min_squal)

    text = sys.stdin.read() if args.log == "-" else open(args.log).read()
    runs, complaints = parse(text)
    if not runs:
        for c in complaints:
            print(f"  SKIP {c}", file=sys.stderr)
        print("no usable FLOWCAL RUN lines found", file=sys.stderr)
        return 2

    kept, dropped = solve(runs, args.axis, args.distance, args.height, thr)
    report_batch(runs, complaints, kept, dropped)
    if not kept:
        print("nothing usable: fix the surface or the slide and capture again",
              file=sys.stderr)
        return 1

    if args.validate is not None:
        print()
        return do_validate(kept, args, thr)
    return do_fit(runs, complaints, kept, dropped, args, thr)


if __name__ == "__main__":
    raise SystemExit(main())
