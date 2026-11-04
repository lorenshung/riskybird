"""tools/rb/rig_analyze.py on responses with known answers: an exact second-order step (overshoot,
rise, settle), oscillation and saturation, and the frequency response / margins of a known loop.

Needs numpy and scipy (the zephyr env: `rb` runs analysis there); skips cleanly without them.
"""

from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

try:
    import numpy as np
    from scipy import signal
    from tools.rb import rig_analyze as A
except ImportError as exc:  # pragma: no cover - system python without numpy
    A = None
    _WHY = str(exc)
else:
    _WHY = ""

from tools.rb import rig_run as R
from tools.rb import rig_tui as T

FS = 250.0


def second_order_step(t, zeta, wn):
    wd = wn * math.sqrt(1 - zeta ** 2)
    phi = math.acos(zeta)
    y = 1 - np.exp(-zeta * wn * t) / math.sqrt(1 - zeta ** 2) * np.sin(wd * t + phi)
    return np.where(t >= 0, y, 0.0)


def trial(t, roll_deg, tgt_deg, exc_val, duty=0.5, gyro=None):
    n = len(t)
    d = {"t_s": t, "roll_deg": roll_deg, "roll_tgt_deg": tgt_deg, "exc_val": exc_val,
         "gyro_p": np.zeros(n) if gyro is None else gyro, "rate_tgt_dps": np.zeros(n),
         "moment_mNm": np.zeros(n)}
    for i in range(4):
        d["duty%d" % i] = np.full(n, duty) if np.isscalar(duty) else duty[i]
    return d


@unittest.skipIf(A is None, "numpy/scipy not available: %s" % _WHY)
class StepMetricsTest(unittest.TestCase):
    ZETA, FN, AMP, T0 = 0.4, 2.0, 10.0, 0.3

    def make(self, extra=None, duty=0.5):
        t = np.arange(0, 3.3, 1 / FS)
        wn = 2 * math.pi * self.FN
        y = self.AMP * second_order_step(t - self.T0, self.ZETA, wn)
        if extra is not None:
            y = y + extra(t)
        step = np.where(t >= self.T0 - 1e-9, 1.0, 0.0)
        return t, trial(t, y, self.AMP * step, math.radians(self.AMP) * step, duty=duty)

    def reference(self):
        """The same definitions on the analytic curve at 100 kHz."""
        wn = 2 * math.pi * self.FN
        tt = np.arange(0, 3.0, 1e-5)
        y = second_order_step(tt, self.ZETA, wn)
        os_pct = 100 * math.exp(-self.ZETA * math.pi / math.sqrt(1 - self.ZETA ** 2))
        rise = (tt[np.argmax(y >= 0.9)] - tt[np.argmax(y >= 0.1)]) * 1000
        band = max(0.05 * self.AMP, 1.0) / self.AMP
        settle = tt[np.nonzero(np.abs(y - 1) > band)[0][-1]] * 1000
        return os_pct, rise, settle

    def test_exact_second_order_step(self):
        t, d = self.make()
        m = A.trial_metrics(d, {"type": "step", "target": "att", "amp": 175, "dur_ms": 5000},
                            t_exc=self.T0)
        os_pct, rise, settle = self.reference()
        self.assertAlmostEqual(m["overshoot_pct"], os_pct, delta=0.3)
        self.assertAlmostEqual(m["rise_ms"], rise, delta=1000 / FS + 0.5)
        self.assertAlmostEqual(m["settle_ms"], settle, delta=1000 / FS + 0.5)
        self.assertTrue(m["settled"])
        self.assertAlmostEqual(m["steady_err_deg"], 0.0, delta=0.05)
        self.assertEqual(m["sat_frac"], 0.0)
        self.assertIsNone(m["osc_hz"])
        self.assertLess(m["osc_amp_deg"], A.OSC_MIN_DEG)
        want = round(m["rms_err_deg"] + 0.05 * m["overshoot_pct"] + 0.3 * m["settle_ms"] / 1000
                     + 10 * max(0.0, m["osc_amp_deg"] - 0.2), 4)
        self.assertAlmostEqual(m["score"], want, delta=0.002)

    def test_sustained_oscillation_and_saturation(self):
        osc = lambda t: np.where(t > 1.8, 0.8 * np.sin(2 * math.pi * 3.0 * t), 0.0)  # noqa: E731
        t = np.arange(0, 3.3, 1 / FS)
        duty = [np.where(t > 1.0, 0.95, 0.5)] + [np.full(len(t), 0.5)] * 3
        _, d = self.make(extra=osc, duty=duty)
        m = A.trial_metrics(d, {"type": "step", "target": "att", "amp": 175, "dur_ms": 5000},
                            t_exc=self.T0)
        self.assertAlmostEqual(m["osc_hz"], 3.0, delta=0.25)
        self.assertGreater(m["osc_amp_deg"], 0.5)
        self.assertAlmostEqual(m["sat_frac"], (3.3 - 1.0) / (3.3 - self.T0), delta=0.02)
        self.assertGreater(m["score"], 10 * (m["osc_amp_deg"] - 0.2) + 20 * m["sat_frac"])

    def test_doublet_edges_and_unsettled(self):
        t = np.arange(0, 3.0, 1 / FS)
        wn = 2 * math.pi * 2.0
        x = np.where((t >= 0.3) & (t < 0.9), 1.0, np.where((t >= 0.9) & (t < 1.5), -1.0, 0.0))
        y = 5 * (second_order_step(t - 0.3, 0.6, wn) - 2 * second_order_step(t - 0.9, 0.6, wn)
                 + second_order_step(t - 1.5, 0.6, wn))
        d = trial(t, y, 5 * x, math.radians(5) * x)
        m = A.trial_metrics(d, {"type": "doublet", "target": "att", "amp": 87, "dur_ms": 1200},
                            t_exc=0.3)
        self.assertEqual(len(m["edges"]), 3)
        self.assertAlmostEqual(m["edges"][1]["delta"], -10.0, delta=0.01)
        self.assertAlmostEqual(m["overshoot_pct"], 100 * math.exp(-0.6 * math.pi / 0.8), delta=1.0)
        # never comes back: the last edge does not settle
        d2 = trial(t, np.where(t > 1.5, 4.0, y), 5 * x, math.radians(5) * x)
        m2 = A.trial_metrics(d2, {"type": "doublet", "target": "att", "amp": 87, "dur_ms": 1200},
                             t_exc=0.3)
        self.assertIsNotNone(m2["score"])

    def test_no_score_unless_ok(self):
        t, d = self.make()
        for st in ("estop", "aborted", "timeout"):
            m = A.trial_metrics(d, {"type": "step", "target": "att", "amp": 175, "dur_ms": 5000},
                                t_exc=self.T0, status=st)
            self.assertIsNone(m["score"], st)
            self.assertIsNotNone(m["rms_err_deg"])


@unittest.skipIf(A is None, "numpy/scipy not available: %s" % _WHY)
class FrequencyResponseTest(unittest.TestCase):
    """Closed loop T = L/(1+L) with L = K/(s(t1 s+1)(t2 s+1)); excite its reference with a chirp."""

    K, T1, T2 = 8.0, 0.05, 0.02

    def L(self, w):
        s = 1j * w
        return self.K / (s * (self.T1 * s + 1) * (self.T2 * s + 1))

    def analytic(self):
        ws = np.logspace(-1, 3, 200000)
        mag = np.abs(self.L(ws))
        wc = ws[np.argmin(np.abs(mag - 1))]
        pm = 180 + math.degrees(np.angle(self.L(wc)) - 2 * math.pi
                                if np.angle(self.L(wc)) > 0 else np.angle(self.L(wc)))
        w180 = 1 / math.sqrt(self.T1 * self.T2)
        gm = -20 * math.log10(abs(self.L(w180)))
        return pm, wc / (2 * math.pi), gm, w180 / (2 * math.pi)

    def test_margins_of_a_known_loop(self):
        a1, a2 = self.T1 * self.T2, self.T1 + self.T2
        sys_t = signal.lti([self.K], [a1, a2, 1.0, self.K])
        dur = 24.0
        t = np.arange(0, dur, 1 / FS)
        f0, f1 = 0.2, 10.0
        lk = math.log(f1 / f0)
        r = 0.05 * np.sin(2 * math.pi * f0 * dur / lk * (np.exp(lk * t / dur) - 1))
        _, y, _ = signal.lsim(sys_t, r, t)
        rng = np.random.default_rng(1)
        roll = np.degrees(y) + rng.normal(0, 0.01, len(t))
        d = trial(t, roll, np.degrees(r), r)
        m = A.trial_metrics(d, {"type": "chirp", "target": "att", "amp": 50, "dur_ms": int(dur * 1000),
                                "f0_mHz": 200, "f1_mHz": 10000}, t_exc=0.0)
        pm, fc, gm, f180 = self.analytic()
        self.assertIsNotNone(m["pm_deg"], m)
        self.assertAlmostEqual(m["pm_deg"], pm, delta=4.0)
        self.assertAlmostEqual(m["pm_hz"], fc, delta=0.08 * fc)
        self.assertAlmostEqual(m["gm_db"], gm, delta=2.0)
        self.assertGreater(m["bw_hz"], fc * 0.8)
        self.assertIsNone(m["score"])                       # FRF trials: margin_penalty instead
        self.assertIsNotNone(m["margin_penalty"])
        # |roll/exc| (deg per rad) against |T| where coherent
        for f, mag_db, ph, coh in m["frf_roll_exc"]:
            if 0.3 < f < 8 and coh > 0.9:
                want = 20 * math.log10(abs(self.L(2 * math.pi * f) / (1 + self.L(2 * math.pi * f)))
                                       * 180 / math.pi)
                self.assertAlmostEqual(mag_db, want, delta=1.0, msg="f=%.2f" % f)


@unittest.skipIf(A is None, "numpy/scipy not available: %s" % _WHY)
class BatchAnalysisTest(unittest.TestCase):
    def test_dry_run_batch_ranking_and_next_grid(self):
        try:
            from tests.test_rig_run import Batch, small_spec
        except ImportError:          # unittest discover -s tests puts tests/ itself on the path
            from test_rig_run import Batch, small_spec
        b = Batch(small_spec(grid_product={"RATE_P": [30, 45, 60]}, arrangement={"per_spin": 3}),
                  analyze=True).run()
        self.addCleanup(b.close)
        self.assertEqual(b.status, "ok")
        n_metrics = len(b.evs("metrics"))
        res = A.analyze_batch(b.dir, plots=False, quiet=True)
        self.assertEqual(len(res["points"]), 3)
        self.assertEqual(res["varied"], ["RATE_P"])
        best = res["best"]["gains"]["RATE_P"]
        self.assertIn(res["next_grid"]["RATE_P"][1], (30.0, 45.0, 60.0))
        self.assertEqual(res["next_grid"]["RATE_P"][1], best)
        self.assertEqual(len(res["next_grid"]["RATE_P"]), 3)
        nxt = json.loads((b.dir / "next_grid.json").read_text())
        R.validate_spec(nxt)                                  # the suggestion is a runnable spec
        self.assertTrue((b.dir / "summary.md").is_file())
        # recomputing the same data appends nothing
        self.assertEqual(len(b.evs("metrics")), n_metrics)
        A.analyze_batch(b.dir, plots=False, quiet=True)
        self.assertEqual(len(b.evs("metrics")), n_metrics)

    def test_next_grid_halves_the_step(self):
        pts = [{"gains": {"RATE_P": v, "ATT_P": 5.0}} for v in (20.0, 40.0, 60.0)]
        self.assertEqual(A.next_grid(pts, pts[2], ["RATE_P"]), {"RATE_P": [50.0, 60.0, 70.0]})
        pts = [{"gains": {"ATT_P": v}} for v in (38.0, 40.0)]
        self.assertEqual(A.next_grid(pts, pts[1], ["ATT_P"]), {"ATT_P": [39.0, 40.0]})  # clipped


if __name__ == "__main__":
    unittest.main()
