"""Host checks for the roll-harness twin (tools/rig/sim: rig_twin.cpp, rig_mixer.h, rig_sim.py).

Run with ``python3 -m unittest discover -s tests -p test_rig_sim.py -v``. Stdlib only (the numpy
parts of rig_sim.py are imported lazily and not used here). Builds the twin and the mixer check
with the host g++ (skipped without one). Covered: the transcribed mixer equals the real
controller_pid.cpp rig branch bit for bit; the CSV has exactly the recorder's rig columns, rate
and time base (fixed rate and replayed measured dt); a stable closed loop settles level; stop collisions stay bounded; lift-off from a
stop; the rig-roll -D fallback and the plant defaults stay in sync.
"""

from __future__ import annotations

import ast
import csv
import json
import math
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / "tools/rig/sim"
sys.path.insert(0, str(SIM))
import rig_sim  # noqa: E402


def recorder_rig_columns() -> list[str]:
    """RIG_COLUMNS from tools/rb/flightrec_fetch.py without importing it (it needs pyserial):
    evaluate just its FLAGS and RIG_COLUMNS assignments."""
    tree = ast.parse((ROOT / "tools/rb/flightrec_fetch.py").read_text())
    ns: dict = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id in ("FLAGS", "RIG_COLUMNS") for t in node.targets):
            exec(compile(ast.Module(body=[node], type_ignores=[]), "flightrec_fetch.py", "exec"), ns)
    return list(ns["RIG_COLUMNS"])


def read_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    return rows[0], rows[1:]


class RigSim(unittest.TestCase):
    exes = None
    _tmp = None

    @classmethod
    def setUpClass(cls):
        if shutil.which("g++") is None and shutil.which("c++") is None:
            raise unittest.SkipTest("host C++ compiler (g++) unavailable")
        cls._tmp = tempfile.TemporaryDirectory(prefix="rig-sim-test-")
        cls.d = Path(cls._tmp.name)
        cls.exes = rig_sim.build(out_dir=cls.d / "build")
        rig_sim.set_dt_csv(None)   # hermetic: fixed loop_hz, not the (gitignored) measured capture
        cls.level = cls.d / "level.txt"
        cls.level.write_text(rig_sim.scenario_text(1000, step_mdeg=87, hold_ms=2000, after_ms=2500))

    @classmethod
    def tearDownClass(cls):
        if cls._tmp is not None:
            cls._tmp.cleanup()

    def run_twin(self, scenario, csv_path=None, truth=None, **params):
        return rig_sim.run_twin(self.exes["twin"], scenario, params, csv_path=csv_path, truth=truth)

    # ---- the transcription -------------------------------------------------------------------
    def test_mixer_matches_controller_pid(self):
        out = rig_sim.mixer_check(self.exes)
        self.assertIn("ok mixer roll", out)
        self.assertIn("ok mixer pitch", out)   # rig-pitch: u[2] through M's column 2, rear pair up

    # ---- the recorder contract ----------------------------------------------------------------
    def test_csv_columns_match_recorder(self):
        path = self.d / "cols.csv"
        js = self.run_twin(self.level, csv_path=path)
        self.assertEqual(js["status"], "ok", js)
        head, rows = read_csv(path)
        self.assertEqual(head, recorder_rig_columns())
        col = {n: i for i, n in enumerate(head)}
        t = [float(r[col["t_s"]]) for r in rows]
        dts = sorted(b - a for a, b in zip(t, t[1:]))
        self.assertAlmostEqual(1.0 / dts[len(dts) // 2], rig_sim.NOMINAL["loop_hz"] / 4, delta=1.0)  # FLIGHTREC_DIV 4
        seq = [int(r[col["seq"]]) for r in rows]
        self.assertTrue(all((b - a) & 0xFF == 4 for a, b in zip(seq, seq[1:])))
        first = next(i for i, r in enumerate(rows)
                     if any(float(r[col[f"duty{k}"]]) > 0 for k in range(4)))
        self.assertEqual(float(rows[first][col["t_s"]]), 0.0)                 # t_s = 0 at the first motor tick
        self.assertLess(t[0], 0.0)                                            # preroll before it
        names = {r[col["rs_name"]] for r in rows}
        self.assertLessEqual(names, {"IDLE", "ARMING", "RAMP_UP", "RUN", "RAMP_DOWN", "DONE", "ESTOP"})
        self.assertTrue({"RAMP_UP", "RUN", "RAMP_DOWN", "DONE"} <= names)
        for r in rows:   # duty is the 1/10000 wire quantum, inside the ceiling
            for k in range(4):
                d = float(r[col[f"duty{k}"]])
                self.assertAlmostEqual(d * 10000, round(d * 10000), places=6)
                self.assertLessEqual(d, 0.95)
        # the excitation is logged in SI (rad) and reaches the target
        exc = [float(r[col["exc_val"]]) for r in rows]
        self.assertAlmostEqual(max(exc), 0.087, places=4)

    # ---- closed loop ---------------------------------------------------------------------------
    def test_stable_closed_loop_settles_level(self):
        path = self.d / "level.csv"
        js = self.run_twin(self.level, csv_path=path)
        self.assertEqual(js["status"], "ok", js)
        self.assertEqual(js["naks"], 0, js["nak_lines"])
        self.assertEqual(js["stop_hits_live"], 0)
        self.assertTrue(js["settled"])
        self.assertLess(js["overshoot_pct"], 20.0)
        self.assertLess(abs(js["steady_err_deg"]), 0.5)
        self.assertLess(js["tail_osc_deg"], 0.5)
        head, rows = read_csv(path)
        col = {n: i for i, n in enumerate(head)}
        run = [float(r[col["roll_deg"]]) for r in rows if r[col["rs_name"]] == "RUN"]
        self.assertLess(max(abs(x) for x in run[-150:]), 0.5)                 # level again at the end
        # the swappable metrics (rig_analyze or the local fallback) agree with the twin's own
        m = rig_sim._local_metrics(rig_sim.load_csv(path), {"type": "step", "target": "att",
                                                             "amp": 87, "dur_ms": 1500})
        self.assertAlmostEqual(m["overshoot_pct"], js["overshoot_pct"], delta=1.0)

    def test_unstable_gains_are_flagged(self):
        """A gain far past the stability edge on a slow, light plant must not look stable."""
        js = self.run_twin(self.level, I=1.6e-5, tau_up=0.1, tau_down=0.1, delay_ms=5,
                           thrust_scale=1.3, gains={"RATE_P": 150, "ATT_P": 10})
        self.assertFalse(rig_sim.stable(js), js)

    # ---- stops ---------------------------------------------------------------------------------
    def test_stop_collision_bounded(self):
        # CoM 3 mm above the axis, motors never armed: it falls onto a stop and stays there
        scen = self.d / "idle.txt"
        scen.write_text("0 HBON\n8000 END\n")
        truth = self.d / "fall.csv"
        js = self.run_twin(scen, truth=truth, h_mm=-3.0, theta0_deg=1.0, restitution=0.5)
        self.assertEqual(js["status"], "never_armed")
        self.assertGreaterEqual(js["stop_hits_total"], 1)
        head, rows = read_csv(truth)
        th = [float(r[head.index("theta_deg")]) for r in rows]
        self.assertLessEqual(max(abs(x) for x in th), 30.0 + 1e-9)
        self.assertAlmostEqual(th[-1], 30.0, places=3)
        # each rebound is smaller than the last (restitution < 1): peaks away from the stop shrink
        lows, falling = [], False
        for a, b in zip(th, th[1:]):
            if b < a:
                falling = True
            elif falling and b > a:
                lows.append(a)
                falling = False
        self.assertTrue(all(y >= x - 1e-6 for x, y in zip(lows, lows[1:])), lows)

    def test_liftoff_from_stop(self):
        scen = self.d / "lift.txt"
        scen.write_text(rig_sim.scenario_text(1000, step_mdeg=0, hold_ms=3000))
        js = self.run_twin(scen, theta0_deg=-30.0, h_mm=-1.0)
        self.assertEqual(js["status"], "ok", js)
        self.assertIsNotNone(js["liftoff_ms"])
        self.assertLess(js["liftoff_ms"], 1500 + 2000)          # inside RIG_LIFTOFF_TIMEOUT_MS
        self.assertEqual(js["stop_hits_live"], 0)
        self.assertLess(js["roll_max_live_deg"], 5.0)           # no swing into the opposite side
        self.assertTrue(any("liftoff" in e for e in js["events"]))

    def test_liftoff_assist(self):
        """3.5 mN m toward the +30 deg stop (right wing down), the 2026-11-04 hardware gains: without
        the lift-off assist it never leaves the stop (as on hardware); with the defaults it lifts off
        at COLL 600 and 800, never trips the guard and barely swings past level."""
        base = (rig_sim.HERE / "scenarios/liftoff_assist.txt").read_text()
        for coll in (600, 800):
            scen = self.d / f"assist{coll}.txt"
            scen.write_text(base.replace("COLL 600", f"COLL {coll}"))
            off = self.run_twin(scen, **rig_sim.ASSIST_PLANT, gains=dict(rig_sim.HW_1104, LIFT_I=0))
            self.assertEqual(off["status"], "rampdown:RIG liftoff timeout", off)
            self.assertIsNone(off["liftoff_ms"])
            on = self.run_twin(scen, **rig_sim.ASSIST_PLANT, gains=rig_sim.HW_1104)
            self.assertEqual(on["status"], "ok", on)
            self.assertLess(on["liftoff_ms"], 1500 + 8000)      # inside rig-roll's 8 s timeout
            self.assertLess(on["liftoff_att_int"], 0.0)         # pushes off the + stop
            self.assertGreater(on["guard_margin_live_deg"], 2.0)
            self.assertGreater(on["roll_min_live_deg"], -3.0)   # swing past level
            self.assertEqual(on["stop_hits_live"], 0)

    # ---- keeping the copies honest -------------------------------------------------------------
    def test_rig_defines_fallback_matches_boards(self):
        live, fallback = rig_sim.rig_defines(), rig_sim.RIG_DEFINES_FALLBACK
        self.assertEqual({k: live.get(k) for k in rig_sim.TWIN_KEYS},
                         {k: fallback.get(k) for k in rig_sim.TWIN_KEYS})
        self.assertFalse([k for k in live if k.startswith("CONFIG_")])
        # rig-pitch = rig-roll + RIG_AXIS=1, in boards.py and in the fallback
        pitch = rig_sim.rig_defines(1)
        self.assertEqual(pitch, dict(live, **rig_sim.RIG_PITCH_EXTRA))
        self.assertEqual({k: pitch.get(k) for k in rig_sim.TWIN_KEYS},
                         {k: dict(fallback, **rig_sim.RIG_PITCH_EXTRA).get(k) for k in rig_sim.TWIN_KEYS})

    def test_plant_defaults_match_twin(self):
        out = subprocess.run([str(self.exes["twin"]), "help"], capture_output=True, text=True).stdout
        twin = {}
        for line in out.splitlines()[1:]:
            parts = line.split()
            if len(parts) >= 2:
                twin[parts[0]] = float(parts[1])
        for k, v in rig_sim.NOMINAL.items():
            self.assertIn(k, twin)
            self.assertTrue(math.isclose(twin[k], v, rel_tol=1e-9), f"{k}: twin {twin[k]} vs {v}")

    def test_dt_replay(self):
        """dt_file= replays measured loop periods (stalls included); the recorder keeps every 4th."""
        periods = [2000, 2860, 2000, 9220, 2000, 3140, 2000, 18080] * 4
        f = self.d / "dt.txt"
        f.write_text("\n".join(map(str, periods)) + "\n")
        path = self.d / "dt.csv"
        js = self.run_twin(self.level, csv_path=path, dt_file=f, seed=3)
        self.assertEqual(js["status"], "ok", js)
        head, rows = read_csv(path)
        col = {n: i for i, n in enumerate(head)}
        self.assertLessEqual({int(r[col["dt_us"]]) for r in rows}, set(periods))
        t = [float(r[col["t_s"]]) for r in rows]
        mean = sum(periods) / len(periods)
        self.assertAlmostEqual((t[-1] - t[0]) / (len(t) - 1), 4 * mean * 1e-6, delta=0.15 * 4 * mean * 1e-6)

    def test_batch_matches_single(self):
        pts = [{"I": 3e-5, "gains": {"RATE_P": 30}}, {"h_mm": -1.0, "theta0_deg": 0.5}]
        batch = rig_sim.run_batch(self.exes["twin"], self.level, pts, jobs=2)
        for p, b in zip(pts, batch):
            single = self.run_twin(self.level, **p)
            b = dict(b)
            b.pop("id")
            self.assertEqual(json.dumps(b, sort_keys=True), json.dumps(single, sort_keys=True))


if __name__ == "__main__":
    unittest.main()
