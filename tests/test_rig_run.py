"""tools/rb/rig_run.py: spec validation, the firmware mirror, segmentation, and whole dry-run
batches against the in-process fake drone (FakeDrone + VirtualClock).

Nothing here opens a socket to the drone: the dry-run tests run under a socket guard that
fails any send or connect to 192.168.4.1, and RealLink is never constructed. Metrics need numpy
(the zephyr env); without it the runner notes that metrics were skipped and the rest is tested.
"""

from __future__ import annotations

import contextlib
import io
import json
import math
import os
import re
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

from tools.rb import rig_run as R
from tools.rb import rig_tui as T

ROOT = Path(__file__).resolve().parents[1]
FC_RIG_CPP = ROOT / "workloads" / "flight_controller" / "src" / "fc_rig.cpp"
EXAMPLE = ROOT / "tools" / "rig" / "examples" / "rate_p_sweep.json"
SPEED = 10.0
_DEVNULL = open(os.devnull, "w")  # noqa: SIM115 - module lifetime

try:
    import numpy  # noqa: F401
    import scipy  # noqa: F401
    HAVE_NUMPY = True
except ImportError:
    HAVE_NUMPY = False


def small_spec(**over):
    raw = {
        "name": "t",
        "collective_permille": 1000,
        "roll_sp_mdeg": 0,
        "base_gains": {"ATT_P": 5.0},
        "grid_product": {"RATE_P": [30, 45]},
        "excitations": [{"type": "step", "target": "att", "amp": 87, "dur_ms": 600},
                        {"type": "doublet", "target": "att", "amp": 70, "dur_ms": 600}],
        "repeats": 1,
        "arrangement": {"per_spin": 2},
        "limits": {"max_trials": 20, "max_coll_permille": 1100, "max_run_ms": 20000,
                   "motor_budget_s": 120, "settle_ms": 400},
    }
    for k, v in over.items():
        if k == "limits":
            raw["limits"] = dict(raw["limits"], **v)
        else:
            raw[k] = v
    return raw


class _GuardSocket(socket.socket):
    def sendto(self, data, *args):
        addr = args[-1]
        if isinstance(addr, tuple) and addr[0] == R.DEFAULT_DRONE:
            raise AssertionError("dry run sent a datagram to the drone")
        return super().sendto(data, *args)

    def connect(self, addr):
        if isinstance(addr, tuple) and addr[0] == R.DEFAULT_DRONE:
            raise AssertionError("dry run connected to the drone")
        return super().connect(addr)


@contextlib.contextmanager
def no_drone():
    def boom(*a, **k):
        raise AssertionError("RealLink constructed in a dry run")
    with mock.patch.object(socket, "socket", _GuardSocket), \
            mock.patch.object(R.RealLink, "__init__", boom):
        yield


class Batch:
    """One dry-run batch in a temp dir."""

    def __init__(self, raw, committed=True, analyze=None, fake_kw=None, speed=SPEED, **runner_kw):
        self.tmp = tempfile.TemporaryDirectory()
        self.spec = R.validate_spec(raw)
        self.clock = R.VirtualClock(speed)
        self.fake = R.FakeDrone(self.clock, committed=committed, **(fake_kw or {}))
        self.dir = R.make_batch_dir(self.tmp.name, self.spec["name"])
        self.runner = R.Runner(self.spec, self.dir, self.fake, self.clock, approval="unit test",
                               dry_run=True, analyze=HAVE_NUMPY if analyze is None else analyze,
                               log=lambda s: None, **runner_kw)
        self.status = None
        self.exc = None

    def run(self):
        with no_drone():
            try:
                self.status = self.runner.run()
            except BaseException as exc:  # noqa: BLE001 - the test inspects it
                self.exc = exc
                self.status = self.runner.result
        return self

    def journal(self):
        return T.load_journal(self.dir / "journal.jsonl")[0]

    def evs(self, kind):
        return [e for e in self.journal() if e.get("ev") == kind]

    def cmds(self):
        return [c[1] for c in self.fake.commands]

    def close(self):
        self.tmp.cleanup()


def ramp_down_windows(fake):
    """[(t STOP delivered while armed, t motors off)] from the fake's command log and FC events
    (the disarmed setup's STOP and the exit path's STOP after motors-off are not ramp-downs)."""
    out = []
    stops = [t for t, text, reply in fake.commands if text == "STOP" and reply.startswith("ACK")]
    arms = [t for t, e in fake.events if e.startswith("armed")]
    offs = [t for t, e in fake.events if "ramp-down complete" in e or "on the stop" in e
            or e.startswith("estop")]
    for t0 in stops:
        armed = [t for t in arms if t <= t0]
        if not armed or any(armed[-1] < t <= t0 for t in offs):
            continue
        after = [t for t in offs if t > t0]
        if after:
            out.append((t0, after[0]))
    return out


def max_hb_gap(fake, t0, t1):
    ts = [t for t in fake.hb_times if t0 - 0.2 <= t <= t1 + 0.1]
    if len(ts) < 2:
        return float("inf")
    return max(b - a for a, b in zip(ts, ts[1:]))


# =============================================================================

class FirmwareMirrorTest(unittest.TestCase):
    """The host must refuse exactly what the FC would NAK or clamp: keep the mirror honest."""

    def setUp(self):
        self.src = FC_RIG_CPP.read_text()

    def test_gain_table_matches_fc_rig_cpp(self):
        rows = re.findall(r'\{"(\w+)",\s*(-?[\d.]+)f,\s*(-?[\d.]+)f\}', self.src)
        self.assertEqual([(n, float(lo), float(hi)) for n, lo, hi in rows], list(R.GAIN_TABLE))

    def test_defines_match_fc_rig_cpp(self):
        want = {"RIG_ARM_SETTLE_MS": R.ARM_SETTLE_MS, "RIG_RAMP_MS": R.RAMP_MS,
                "RIG_SP_SLEW_RADPS": R.SP_SLEW_RADPS, "RIG_HB_RAMP_MS": R.HB_RAMP_MS,
                "RIG_HB_ESTOP_MS": R.HB_ESTOP_MS, "RIG_MAX_RUN_CAP_MS": R.MAX_RUN_CAP_MS,
                "RIG_RUNMS_MIN": R.RUNMS_MIN, "RIG_COLL_MAX_PERMILLE": R.COLL_MAX_PERMILLE,
                "RIG_SP_MAX_MDEG": R.SP_MAX_MDEG, "RIG_EXC_MOMENT_FRAC": R.EXC_MOMENT_FRAC,
                "RIG_EXC_ATT_MAX_RAD": R.EXC_ATT_MAX_MRAD / 1000.0,
                "RIG_EXC_RATE_MAX_RADPS": R.EXC_RATE_MAX_MRADPS / 1000.0,
                "RIG_EXC_F_MIN_MHZ": R.EXC_F_MIN_MHZ, "RIG_EXC_F_MAX_MHZ": R.EXC_F_MAX_MHZ,
                "RIG_ARM_MAX_TILT_RAD": R.ARM_MAX_TILT_RAD, "RIG_MAX_TILT_RAD": R.MAX_TILT_RAD,
                "RIG_LIFTOFF_TIMEOUT_MS": R.LIFTOFF_TIMEOUT_MS, "RIG_COLL_SLEW": R.COLL_SLEW,
                "RIG_EXC_TAPER_FRAC": R.EXC_TAPER_FRAC}
        for name, v in want.items():
            m = re.search(r"#define %s\s+(-?[\d.]+)f?" % name, self.src)
            self.assertIsNotNone(m, name)
            self.assertAlmostEqual(float(m.group(1)), v, places=6, msg=name)
        self.assertIn("rig_l_arm    = 33e-3f", self.src)
        self.assertIn("rig_J[3]     = {16e-6f", self.src)

    def test_parse_decimal_follows_the_fc_parser(self):
        self.assertEqual(R.parse_decimal("40"), 40.0)
        self.assertEqual(R.parse_decimal("-0.0125"), R.f32(-0.0125))
        for bad in ("", "-", ".5", "5.", "1e3", "inf", "nan", " 1", "1 ", "1234567890",
                    "0.1234567890123", "+-1", "١"):
            self.assertIsNone(R.parse_decimal(bad), bad)

    def test_format_gain_round_trips_through_the_parser(self):
        import random
        rng = random.Random(3)
        for name, lo, hi in R.GAIN_TABLE:
            for _ in range(200):
                v = rng.uniform(lo, hi)
                s = R.format_gain(v)
                self.assertIsNotNone(s)
                self.assertNotIn("e", s.lower())
                self.assertEqual(R.parse_decimal(s), R.f32(v), (name, v, s))
        self.assertEqual(R.format_gain(0.85), "0.85")
        self.assertEqual(R.format_gain(40), "40")
        self.assertEqual(R.format_gain(1e-5), "0.00001")
        self.assertIsNone(R.format_gain(float("nan")))


class SpecValidationTest(unittest.TestCase):
    def problems(self, raw):
        with self.assertRaises(R.SpecError) as cm:
            R.validate_spec(raw)
        return "\n".join(cm.exception.problems)

    def test_example_spec_is_valid(self):
        spec = R.load_spec(EXAMPLE)
        self.assertEqual(len(spec["grid"]), 3)
        self.assertEqual(len(R.plan_trials(spec)), 9)

    def test_small_spec_is_valid_and_ordered(self):
        spec = R.validate_spec(small_spec(repeats=2))
        trials = R.plan_trials(spec)
        self.assertEqual([t["trial"] for t in trials], list(range(1, 9)))
        # repeat > grid point > excitation
        self.assertEqual([(t["rep"], t["point"], t["exc_index"]) for t in trials[:4]],
                         [(0, 0, 0), (0, 0, 1), (0, 1, 0), (0, 1, 1)])
        self.assertEqual(trials[0]["gains"], {"ATT_P": 5.0, "RATE_P": 30.0})
        spec = R.validate_spec(small_spec(limits={"max_trials": 3}))
        self.assertEqual(len(R.plan_trials(spec)), 3)

    def test_firmware_ranges(self):
        self.assertIn("unknown gain 'RATE_X'", self.problems(small_spec(base_gains={"RATE_X": 1})))
        self.assertIn("outside the firmware range", self.problems(small_spec(base_gains={"ATT_P": 41})))
        self.assertIn("outside the firmware range", self.problems(small_spec(base_gains={"RATE_P": 0.5})))
        self.assertIn("not a finite number", self.problems(small_spec(base_gains={"ATT_P": "5"})))
        self.assertIn("collective_permille", self.problems(small_spec(collective_permille=1200)))
        self.assertIn("collective_permille", self.problems(small_spec(collective_permille=0)))
        self.assertIn("limits.max_coll_permille",
                      self.problems(small_spec(limits={"max_coll_permille": 1400})))
        self.assertIn("roll_sp_mdeg", self.problems(small_spec(roll_sp_mdeg=20001)))
        self.assertIn("limits.max_run_ms", self.problems(small_spec(limits={"max_run_ms": 70000})))
        self.assertIn("unknown key 'grid_prodcut'", self.problems(dict(small_spec(), grid_prodcut={})))

    def test_excitation_checks(self):
        def exc(**e):
            base = {"type": "step", "target": "att", "amp": 50, "dur_ms": 500}
            base.update(e)
            return small_spec(excitations=[base])
        self.assertIn("over the cap 260", self.problems(exc(amp=261)))
        self.assertIn("integer", self.problems(exc(amp=50.5)))
        self.assertIn("amp 0", self.problems(exc(amp=0)))
        self.assertIn("f1_mHz", self.problems(exc(type="chirp", f0_mHz=200)))
        self.assertIn("f0_mHz", self.problems(exc(type="prbs")))
        self.assertIn("f0_mHz", self.problems(exc(type="sine", f0_mHz=5)))
        self.assertIn("type 'ramp'", self.problems(exc(type="ramp")))
        self.assertIn("dur_ms", self.problems(exc(dur_ms=0)))
        cap = R.moment_cap_unm(1000, 0.050)
        self.assertAlmostEqual(cap, 0.3 * 0.5 * 0.4905 * 0.033 * 1e6, places=3)
        self.assertIn("moment amplitude", self.problems(exc(target="moment", amp=int(cap) + 1)))
        # tilt budget: 15 deg setpoint + 5 deg step > 18 deg
        self.assertIn("exceeds 18 deg", self.problems(dict(exc(amp=87), roll_sp_mdeg=15000)))
        # a 2 rad/s rate step at ATT_P 5 holds 0.4 rad = 23 deg
        self.assertIn("steady offset", self.problems(exc(target="rate", amp=2000)))
        self.assertIn("steady offset", self.problems(exc(target="moment", amp=2000)))

    def test_trial_must_fit_max_run_and_budget(self):
        raw = small_spec(excitations=[{"type": "step", "target": "att", "amp": 50, "dur_ms": 9000}],
                         limits={"max_run_ms": 10000})
        self.assertIn("more than limits.max_run_ms", self.problems(raw))
        self.assertIn("does not fit even one trial",
                      self.problems(small_spec(limits={"motor_budget_s": 2})))

    def test_spin_grouping(self):
        spec = R.validate_spec(small_spec(grid_product={"RATE_P": [30, 40, 50]}))
        q = R.plan_trials(spec)
        s1, why = R.take_spin(spec, q, 1e9, 1e9)
        self.assertEqual([t["trial"] for t in s1], [1, 2, 3, 4])
        self.assertEqual(why, "per_spin")
        s, why = R.take_spin(spec, q, R.spin_estimate(spec, q[:2])["motor_s"] + 0.01, 1e9)
        self.assertEqual(([t["trial"] for t in s], why), ([1, 2], "budget"))
        est = R.spin_estimate(spec, q[:2])
        self.assertGreater(est["fetch_s"], 0)
        self.assertGreater(est["wall_s"], est["motor_s"] + est["fetch_s"])


class SegmentTest(unittest.TestCase):
    def rows(self, plan):
        """plan: list of (n_samples, exc_id, gain_seq, exc_on)."""
        header = ["t_s", "rig_state", "exc_id", "gain_seq", "exc_val"]
        rows, t = [], 0.0
        for n, eid, gs, on in plan:
            for i in range(n):
                rows.append(["%.3f" % t, "3", str(eid & 0xFF), str(gs),
                             "0.1" if on and i < n // 2 else "0"])
                t += 0.004
        return header, rows

    def test_maps_exc_ids_in_order_with_u8_wrap(self):
        header, rows = self.rows([(100, 254, 5, False), (250, 255, 5, True), (250, 0, 5, True)])
        sent = [{"trial": 7, "gains": {}, "exc": {"dur_ms": 500}},
                {"trial": 8, "gains": {}, "exc": {"dur_ms": 500}}]
        segs = R.segment_spin(header, rows, sent)
        self.assertEqual([s["trial"] for s in segs], [7, 8])
        self.assertTrue(all(s["matched"] for s in segs), segs)
        self.assertEqual(segs[0]["exc_id"], 255)
        self.assertEqual(segs[1]["exc_id"], 0)
        self.assertEqual(segs[0]["hi"], 350)                 # the next exc_id starts there
        self.assertEqual(segs[1]["lo"], 350 - 75)            # 0.3 s of pre-window before it

    def test_missing_exc_and_gain_mismatch_are_flagged(self):
        header, rows = self.rows([(100, 3, 5, False), (250, 4, 5, True), (100, 4, 6, False)])
        gj = {"snapshot": {"gain_seq": 5, "gains": {"RATE_P": 40.0}},
              "changes": [{"gain_seq": 6, "name": "RATE_P", "value": 50.0, "t_us": 0}]}
        sent = [{"trial": 1, "gains": {"RATE_P": 45.0}, "exc": {"dur_ms": 500}},
                {"trial": 2, "gains": {"RATE_P": 50.0}, "exc": {"dur_ms": 500}}]
        segs = R.segment_spin(header, rows, sent, gj)
        self.assertFalse(segs[0]["matched"])
        self.assertIn("RATE_P=40.0 (plan 45.0)", segs[0]["reason"])
        self.assertLess(segs[0]["hi"], len(rows))        # cut at the gain_seq change
        self.assertFalse(segs[1]["matched"])
        self.assertIn("NAKed or lost", segs[1]["reason"])
        self.assertEqual(R.gains_at(gj, 6), {"RATE_P": 50.0})
        self.assertIsNone(R.gains_at(gj, 7))                 # change fell out of the ring


# =============================================================================

class DryRunBatchTest(unittest.TestCase):
    def test_full_batch(self):
        b = Batch(small_spec(), analyze=True).run()
        self.addCleanup(b.close)
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "ok")
        j = b.journal()
        self.assertEqual(j[0]["ev"], "batch_start")
        self.assertEqual(j[-1]["ev"], "batch_end")
        self.assertEqual(j[-1]["status"], "ok")
        self.assertTrue(all("t" in e and "ev" in e for e in j))
        ends = {e["trial"]: e["status"] for e in b.evs("trial_end")}
        self.assertEqual(ends, {1: "ok", 2: "ok", 3: "ok", 4: "ok"})
        fetch = {e["trial"]: e for e in b.evs("fetch")}
        self.assertEqual(sorted(fetch), [1, 2, 3, 4])
        for n in range(1, 5):
            tdir = b.dir / "trials" / str(n)
            self.assertTrue((tdir / "rec.csv").is_file())
            self.assertTrue((tdir / "rec.bin").is_file())
            tj = T.read_json(tdir / "trial.json")
            self.assertTrue(tj["matched"], tj)
            self.assertGreater(fetch[n]["samples"], 100)
            rec = T.load_rec_csv(tdir / "rec.csv")       # the TUI reads it
            self.assertGreater(len(rec["roll"]), 100)
        # files per contract
        meta = T.read_json(b.dir / "batch.json")
        for k in ("name", "created", "mode", "elf", "elf_sha1", "spec", "approval_note"):
            self.assertIn(k, meta)
        for k in ("collective_permille", "roll_sp_mdeg", "grid", "excitations", "repeats", "limits"):
            self.assertIn(k, meta["spec"])
        self.assertEqual(meta["approval_note"], "unit test")
        st = T.read_json(b.dir / "status.json")
        for k in ("phase", "trial", "n_trials", "current", "motor_ms", "motor_budget_ms",
                  "commit_left_ms", "paused", "updated"):
            self.assertIn(k, st)
        self.assertEqual(st["phase"], "done")
        self.assertEqual(T.read_control(b.dir), {"pause": False, "abort": False, "seq": 0})
        # one spin (per_spin 2 covers both points), gains changed live in between
        self.assertEqual(b.runner.spin_n, 1)
        cmds = b.cmds()
        i_coll = cmds.index("COLL 1000")
        # STOP, COLL 0, gains | RUNMS, RSP | COLL <c> | RESET: COLL is proven before the RESET
        self.assertEqual(cmds[:2], ["STOP", "COLL 0"])
        self.assertEqual(cmds[i_coll - 2:i_coll + 2], ["RUNMS %d" % T.read_json(
            b.dir / "spins" / "1" / "spin.json")["runms"], "RSP 0", "COLL 1000", "RESET"])
        self.assertIn("GAIN RATE_P 45", cmds[i_coll:])
        self.assertNotIn("GAINS", cmds)
        self.assertEqual(b.fake.naks, [])
        self.assertEqual(b.fake.estops, 0)
        events = [e for _, e in b.fake.events]
        self.assertIn("RIG stop", events)
        self.assertFalse([e for e in events if "heartbeat" in e or e.startswith("estop")], events)
        # the heartbeat ran through the whole ramp-down
        win = ramp_down_windows(b.fake)
        self.assertEqual(len(win), 1)
        t0, t1 = win[0]
        self.assertGreater(t1 - t0, R.RAMP_MS / 1000.0 - 0.05)
        self.assertLess(max_hb_gap(b.fake, t0, t1), R.HB_RAMP_MS / 1000.0 * 0.8)
        self.assertTrue([t for t in b.fake.hb_times if t >= t1])   # still beating at DONE
        if HAVE_NUMPY:
            m = {e["trial"]: e for e in b.evs("metrics")}
            self.assertEqual(sorted(m), [1, 2, 3, 4])
            self.assertTrue(all(e["score"] is not None for e in m.values()))
            self.assertTrue((b.dir / "trials" / "1" / "metrics.json").is_file())
        else:
            self.assertTrue([e for e in b.evs("note") if "metrics skipped" in e["text"]])

    def test_tui_renders_the_dry_run_batch(self):
        b = Batch(small_spec(), analyze=False).run()
        self.addCleanup(b.close)
        out = subprocess.run([sys.executable, str(ROOT / "tools" / "rb" / "rig_tui.py"), "--replay",
                              str(b.dir), "--snapshot", "120x40", "--ascii"],
                             capture_output=True, text=True, timeout=60)
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertIn("DONE OK", out.stdout)
        self.assertIn("4 trials", out.stdout)

    def test_abort_via_control_json_stops_with_heartbeat_through_ramp_down(self):
        b = Batch(small_spec())
        self.addCleanup(b.close)

        def hook(text):
            if text.startswith("EXC") and not getattr(hook, "done", False):
                hook.done = True
                T.update_control(b.dir, abort=True)
        b.fake.on_command.append(hook)
        b.run()
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "aborted")
        self.assertEqual({e["trial"]: e["status"] for e in b.evs("trial_end")}, {1: "aborted"})
        self.assertEqual([e["trial"] for e in b.evs("trial_start")], [1])
        self.assertEqual(b.evs("batch_end")[-1]["status"], "aborted")
        self.assertIn("EXCSTOP", b.cmds())
        self.assertEqual(b.fake.estops, 0)
        events = [e for _, e in b.fake.events]
        self.assertIn("RIG stop", events)
        self.assertFalse([e for e in events if "heartbeat" in e or e.startswith("estop")], events)
        (t0, t1), = ramp_down_windows(b.fake)
        self.assertLess(max_hb_gap(b.fake, t0, t1), R.HB_RAMP_MS / 1000.0 * 0.8)
        self.assertEqual(b.cmds().count("RESET"), 1)

    def test_pause_ends_the_spin_and_resume_continues(self):
        b = Batch(small_spec())
        self.addCleanup(b.close)
        seen = {}

        def hook(text):
            if text.startswith("EXC") and "paused" not in seen:
                seen["paused"] = True
                T.update_control(b.dir, pause=True)

        def resumer():
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline:
                st = T.read_json(b.dir / "status.json", {}) or {}
                if st.get("paused"):
                    seen["status_paused"] = True
                    T.update_control(b.dir, pause=False)
                    return
                time.sleep(0.02)
        b.fake.on_command.append(hook)
        th = threading.Thread(target=resumer, daemon=True)
        th.start()
        b.run()
        th.join(5)
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "ok")
        self.assertTrue(seen.get("status_paused"))
        self.assertEqual({e["trial"]: e["status"] for e in b.evs("trial_end")},
                         {1: "ok", 2: "ok", 3: "ok", 4: "ok"})
        self.assertEqual(b.runner.spin_n, 2)                 # trial 1 alone, then 2-4
        self.assertEqual(b.cmds().count("RESET"), 2)
        self.assertEqual(b.fake.estops, 0)
        self.assertEqual(len(ramp_down_windows(b.fake)), 2)

    def _estop_on_exception(self, exc_type):
        b = Batch(small_spec(), analyze=False)
        self.addCleanup(b.close)
        count = {"exc": 0}

        def hook(text):
            if text.startswith("EXC"):
                count["exc"] += 1
                if count["exc"] == 2:
                    raise exc_type("injected")
        b.fake.on_command.append(hook)
        b.run()
        self.assertIsInstance(b.exc, exc_type)
        self.assertEqual(b.fake.estops, 3)
        self.assertEqual(b.cmds().count("ESTOP"), 3)
        i_estop = b.cmds().index("ESTOP")
        self.assertIn("STOP", b.cmds()[i_estop:])            # STOP on the way out too
        self.assertNotIn("RESET", b.cmds()[i_estop:])
        est = b.evs("estop")
        self.assertEqual(est[0]["source"], "host")
        self.assertEqual(b.evs("trial_end")[-1], dict(b.evs("trial_end")[-1], trial=2, status="estop"))
        return b

    def test_exception_sends_estop_three_times(self):
        b = self._estop_on_exception(RuntimeError)
        self.assertEqual(b.evs("batch_end")[-1]["status"], "error")

    def test_ctrl_c_sends_estop_three_times(self):
        b = self._estop_on_exception(KeyboardInterrupt)
        self.assertEqual(b.evs("batch_end")[-1]["status"], "interrupted")

    def test_motor_budget_stops_before_the_next_spin(self):
        spec = R.validate_spec(small_spec(arrangement={"per_spin": 1}))
        q = R.plan_trials(spec)
        one = R.spin_estimate(spec, q[:2])["motor_s"]
        b = Batch(small_spec(arrangement={"per_spin": 1},
                             limits={"motor_budget_s": round(one * 1.5, 1)}), analyze=False).run()
        self.addCleanup(b.close)
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "budget")
        self.assertEqual(sorted(e["trial"] for e in b.evs("trial_end")), [1, 2])
        self.assertTrue([e for e in b.evs("note") if "motor budget" in e["text"]])
        self.assertEqual(b.cmds().count("RESET"), 1)
        self.assertLessEqual(b.runner.motor_s, round(one * 1.5, 1))

    def test_tui_estop_ends_the_batch_without_reset(self):
        b = Batch(small_spec(), analyze=False)
        self.addCleanup(b.close)

        def hook(text):
            if text.startswith("EXC") and not getattr(hook, "done", False):
                hook.done = True
                # what rig_tui.do_estop does: UDP ESTOP first, then the journal, then control
                threading.Thread(target=lambda: (b.fake.estop_burst(3), T.do_estop(
                    mock.Mock(estop=lambda: (3, [])), b.dir, reason="operator key"))).start()
        b.fake.on_command.append(hook)
        b.run()
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "estop")
        self.assertEqual(b.evs("trial_end")[-1]["status"], "estop")
        i = b.cmds().index("ESTOP")
        self.assertNotIn("RESET", b.cmds()[i:])
        self.assertEqual(len(b.evs("trial_start")), 1)

    def test_refuses_an_uncommitted_drone(self):
        b = Batch(small_spec(), committed=False, analyze=False).run()
        self.addCleanup(b.close)
        self.assertIsInstance(b.exc, R.ArmFailed)
        self.assertIn("not COMMITTED", str(b.exc))
        self.assertNotIn("RESET", b.cmds())
        self.assertNotIn("COLL 1000", b.cmds())


class MoreSafetyTest(unittest.TestCase):
    def test_commit_window_stops_before_a_spin_that_would_overrun_it(self):
        b = Batch(small_spec(), analyze=False, commit_used_s=880.0).run()
        self.addCleanup(b.close)
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "commit_window")
        self.assertNotIn("RESET", b.cmds())
        self.assertFalse(b.evs("trial_start"))

    def test_verify_reads_g_rig_back_at_every_stage(self):
        b = Batch(small_spec(), analyze=False, verify_poke=True).run()
        self.addCleanup(b.close)
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "ok")
        reads = b.evs("rig_read")
        self.assertEqual([e["stage"] for e in reads], ["baseline", "gains", "params", "coll", "rx_after"])
        g = reads[1]
        self.assertEqual(g["gains"]["RATE_P"], 30.0)
        self.assertEqual(g["gain_seq"] - reads[0]["gain_seq"], 2)      # ATT_P and RATE_P
        self.assertEqual((g["rs_name"], g["coll_permille"]), ("DONE", 0))
        self.assertEqual(reads[3]["coll_permille"], 1000)
        self.assertFalse(b.evs("uplink_mismatch"))
        self.assertFalse([r for _, _, r in b.fake.commands if r.startswith("BUSY")])

    def test_verify_refuses_a_value_mismatch(self):
        b = Batch(small_spec(), analyze=False)
        self.addCleanup(b.close)
        real = b.fake._memory

        def wrong():                          # RATE_P reads 31 whatever is sent
            mem = real()
            base, rig = mem[0]
            rig = bytearray(rig)
            R.struct.pack_into("<f", rig, 4 * R.GAIN_NAMES.index("RATE_P"), 31.0)
            return [(base, bytes(rig))] + mem[1:]
        b.fake._memory = wrong
        b.run()
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "uplink_unverified")
        self.assertIn("RATE_P=31, 30 expected", b.evs("uplink_mismatch")[-1]["problems"][0])
        self.assertNotIn("RESET", b.cmds())
        self.assertNotIn("COLL 1000", b.cmds())
        self.assertEqual(b.fake.estops, 0)

    def test_tui_estop_while_paused(self):
        b = Batch(small_spec(), analyze=False)
        self.addCleanup(b.close)

        def hook(text):
            if text.startswith("EXC") and not getattr(hook, "done", False):
                hook.done = True
                T.update_control(b.dir, pause=True)

        def presser():
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline:
                if (T.read_json(b.dir / "status.json", {}) or {}).get("paused"):
                    b.fake.estop_burst(3)
                    T.do_estop(mock.Mock(estop=lambda: (3, [])), b.dir, reason="operator key")
                    return
                time.sleep(0.02)
        b.fake.on_command.append(hook)
        th = threading.Thread(target=presser, daemon=True)
        th.start()
        b.run()
        th.join(5)
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "estop")
        self.assertEqual(b.cmds().count("RESET"), 1)
        self.assertEqual(b.evs("batch_end")[-1]["status"], "estop")


class RecorderContractTest(unittest.TestCase):
    def test_fake_csv_columns_are_flightrec_fetch_rig_columns(self):
        try:
            from tools.rb import flightrec_fetch as ff
        except ImportError as exc:
            self.skipTest("flightrec_fetch needs pyserial/pyelftools (zephyr env): %s" % exc)
        self.assertEqual(list(R.REC_COLUMNS), list(ff.RIG_COLUMNS))
        self.assertEqual(tuple(R.REC_FLAGS), tuple(ff.FLAGS))


class CliTest(unittest.TestCase):
    def test_hardware_run_needs_approval_and_elf(self):
        with tempfile.TemporaryDirectory() as tmp:
            spec = Path(tmp) / "s.json"
            spec.write_text(json.dumps(small_spec()))
            with no_drone(), contextlib.redirect_stderr(_DEVNULL):
                self.assertEqual(R.main(["run", str(spec), "--out-dir", tmp]), 2)
                self.assertEqual(R.main(["run", str(spec), "--out-dir", tmp,
                                         "--i-have-approval", "Loren, today, unit test"]), 2)
            self.assertEqual([p.name for p in Path(tmp).iterdir()], ["s.json"])

    def test_bad_spec_is_refused_before_anything(self):
        with tempfile.TemporaryDirectory() as tmp:
            spec = Path(tmp) / "s.json"
            spec.write_text(json.dumps(small_spec(collective_permille=5000)))
            with no_drone(), contextlib.redirect_stderr(_DEVNULL):
                self.assertEqual(R.main(["run", str(spec), "--dry-run", "--out-dir", tmp]), 2)

    def test_dry_run_cli_end_to_end(self):
        with tempfile.TemporaryDirectory() as tmp:
            spec = Path(tmp) / "s.json"
            spec.write_text(json.dumps(small_spec(grid_product={"RATE_P": [40]})))
            with no_drone(), contextlib.redirect_stdout(_DEVNULL):
                rc = R.main(["run", str(spec), "--dry-run", "--speed", str(SPEED), "--out-dir", tmp])
            self.assertEqual(rc, 0)
            (bdir,) = [p for p in Path(tmp).iterdir() if p.is_dir()]
            self.assertTrue(bdir.name.endswith("-t-dryrun"))
            meta = T.read_json(bdir / "batch.json")
            self.assertTrue(meta["dry_run"])


# =============================================================================
# The uplink: one paced sender, delivery proven over RBPOKE, arming observed (11-04 failure)
# =============================================================================

def drop(fake, line, times=None):
    """Make the fake FC lose `line` (the ESP still ACKs it) `times` times, or always."""
    left = {"n": times}

    def hook(text):
        if text == line and (left["n"] is None or left["n"] > 0):
            if left["n"] is not None:
                left["n"] -= 1
            return "drop"
        return None
    fake.rx_hooks.append(hook)


def ev_index(journal, pred):
    return next(i for i, e in enumerate(journal) if pred(e))


class PacingTest(unittest.TestCase):
    def test_no_two_datagrams_closer_than_the_gap(self):
        b = Batch(small_spec(), analyze=False).run()
        self.addCleanup(b.close)
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "ok")
        gap = R.TX_GAP_MS / 1000.0
        ts = [t for t, _, _, bypass in b.fake.rx_log if not bypass]
        self.assertGreater(len(ts), 100)                  # HB included
        self.assertGreaterEqual(min(y - x for x, y in zip(ts, ts[1:])), gap - 1e-9)
        self.assertEqual({o for _, _, o, _ in b.fake.rx_log}, {"ok"})
        self.assertEqual(b.fake.naks, [])
        # the heartbeat still holds 20 Hz on average and never lapses near HB_RAMP_MS
        (t0, t1), = ramp_down_windows(b.fake)
        arm = [t for t, e in b.fake.events if e.startswith("armed")][0]
        hbs = [t for t in b.fake.hb_times if arm <= t <= t1]
        # (virtual time at 10x: a loaded host stretches it, so only loose bounds here; the
        # pacer's own guarantees are tested with a manual clock below)
        self.assertAlmostEqual((hbs[-1] - hbs[0]) / (len(hbs) - 1), R.HB_PERIOD_S, delta=0.010)
        self.assertFalse([e for _, e in b.fake.events if "heartbeat" in e])
        self.assertEqual(b.runner.tx_gap_ms, R.TX_GAP_MS)
        self.assertEqual(T.read_json(b.dir / "batch.json")["tx_gap_ms"], R.TX_GAP_MS)

    def test_pacer_gap_priority_and_bypass(self):
        class Manual:
            def __init__(self):
                self.t, self.lock = 0.0, threading.Lock()

            def now(self):
                with self.lock:
                    return self.t

            def sleep(self, s):
                with self.lock:
                    self.t += max(0.0, s)
        clock = Manual()
        p = R.TxPacer(clock, 0.030)
        stamps = []
        for _ in range(3):
            with p.slot():
                stamps.append(clock.now())
        self.assertEqual([round(t, 6) for t in stamps], [0.0, 0.03, 0.06])
        clock.sleep(0.5)
        p.bypass()                                   # an ESTOP burst at t=0.56
        with p.slot():
            self.assertAlmostEqual(clock.now(), 0.59)
        # a priority sender (HB, STOP) overtakes a command that was waiting first
        order, hold, go = [], threading.Event(), threading.Event()

        def holder():
            with p.slot():
                hold.set()
                go.wait(5)

        def sender(name, prio):
            with p.slot(prio):
                order.append(name)
        th = [threading.Thread(target=holder)]
        th[0].start()
        hold.wait(5)
        th.append(threading.Thread(target=sender, args=("cmd", False)))
        th[1].start()
        time.sleep(0.05)
        th.append(threading.Thread(target=sender, args=("hb", True)))
        th[2].start()
        time.sleep(0.05)
        go.set()
        for t in th:
            t.join(5)
        self.assertEqual(order, ["hb", "cmd"])

    def test_estop_bypasses_the_gap(self):
        b = Batch(small_spec(), analyze=False)
        self.addCleanup(b.close)

        def hook(text):
            if text.startswith("EXC"):
                raise RuntimeError("injected")
        b.fake.on_command.append(hook)
        b.run()
        self.assertIsInstance(b.exc, RuntimeError)
        est = [t for t, text, _, bypass in b.fake.rx_log if text == "ESTOP"]
        self.assertEqual(len(est), 3)
        self.assertTrue(all(bp for _, text, _, bp in b.fake.rx_log if text == "ESTOP"))
        self.assertLess(est[-1] - est[0], R.TX_GAP_MS / 1000.0)    # back to back, not paced
        after = [t for t, text, _, bp in b.fake.rx_log if not bp and t > est[-1]]
        self.assertGreaterEqual(after[0] - est[-1], R.TX_GAP_MS / 1000.0 - 1e-9)

    def test_unpaced_uplink_reproduces_11_04_and_is_refused(self):
        # first_hold_250's 7 GAINs, back to back in real time (at 10x the host's own overhead
        # would space them out): the first line after each RBPOKE lands, the rest are lost
        gains = {"ATT_P": 5.0, "RATE_P": 40.0, "ATT_GAIN": 0.85, "ATT_I": 0.0, "RATE_I": 0.0,
                 "RATE_D": 0.0, "MOTOR_TRIM_LEFT": 1.0}
        b = Batch(small_spec(base_gains=gains, grid_product={"RATE_P": [40]}), analyze=False,
                  tx_gap_ms=0, speed=1.0, fake_kw={"rx_window_ms": 400.0}).run()
        self.addCleanup(b.close)
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "uplink_unverified")
        self.assertTrue([o for _, _, o, _ in b.fake.rx_log if o != "ok"])   # lines were lost
        self.assertTrue(all(r.startswith("ACK") for _, _, r in b.fake.commands))   # yet ACKed
        self.assertEqual(len([e for e in b.evs("uplink_mismatch") if e["stage"] == "gains"]), 3)
        self.assertNotIn("RESET", b.cmds())
        self.assertEqual(b.fake.estops, 0)
        self.assertEqual(b.evs("batch_end")[-1]["status"], "uplink_unverified")

    def test_gap_validation(self):
        with tempfile.TemporaryDirectory() as tmp:
            spec = Path(tmp) / "s.json"
            spec.write_text(json.dumps(small_spec()))
            with no_drone(), contextlib.redirect_stderr(_DEVNULL):
                self.assertEqual(R.main(["run", str(spec), "--dry-run", "--tx-gap-ms", "50",
                                         "--out-dir", tmp]), 2)


class DeliveryTest(unittest.TestCase):
    def test_compare_catches_a_lost_gain_equal_to_the_old_value(self):
        g = {n: R.f32(v) for n, v in R.RIG_ROLL_DEFAULTS.items()}
        before = {"gains": g, "gain_seq": 7, "rs_name": "DONE", "coll_permille": 0}
        snap = dict(before, gain_seq=8)                  # 2 lines sent, 1 applied, values all right
        sent = ["GAIN ATT_P 5", "GAIN RATE_P 40"]
        problems, resend, fatal = R.compare_rig(before, sent, snap,
                                                {"gains": g, "coll": 0, "rs": ("DONE",)})
        self.assertEqual(problems, ["gain_seq advanced by 1 for 2 GAIN line(s)"])
        self.assertEqual((resend, fatal), (sent, False))
        p, _, fatal = R.compare_rig(before, sent, dict(before, gain_seq=10),
                                    {"gains": g, "coll": 0, "rs": ("DONE",)})
        self.assertTrue(fatal)                           # more applied than sent

    def test_dropped_gain_with_an_equal_value_is_caught_and_resent(self):
        b = Batch(small_spec(), analyze=False)          # ATT_P 5 is the fake's default too
        self.addCleanup(b.close)
        drop(b.fake, "GAIN ATT_P 5", times=1)
        b.run()
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "ok")
        mm, = b.evs("uplink_mismatch")
        self.assertEqual(mm["problems"], ["gain_seq advanced by 1 for 2 GAIN line(s)"])
        self.assertEqual(mm["resend"], ["GAIN ATT_P 5"])
        self.assertEqual(b.cmds().count("GAIN ATT_P 5"), 2)
        self.assertTrue([e for e in b.evs("rig_read") if e["stage"] == "gains/resend1"])

    def test_lost_ack_of_a_delivered_gain_is_not_resent(self):
        b = Batch(small_spec(), analyze=False)
        self.addCleanup(b.close)
        real, left = b.fake.await_reply, {"n": 1}

        def await_reply(text, timeout=0.5):
            r = real(text, timeout)
            if text == "GAIN RATE_P 30" and left["n"]:
                left["n"] -= 1
                return None                     # delivered, the ACK lost on WiFi
            return r
        b.fake.await_reply = await_reply
        b.run()
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "ok")
        self.assertEqual(b.cmds().count("GAIN RATE_P 30"), 1)
        self.assertFalse(b.evs("uplink_mismatch"))

    def test_resend_recovers_a_lost_gain(self):
        b = Batch(small_spec(), analyze=False)
        self.addCleanup(b.close)
        drop(b.fake, "GAIN RATE_P 30", times=2)
        b.run()
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "ok")
        self.assertEqual(b.cmds().count("GAIN RATE_P 30"), 3)
        self.assertIn("RATE_P=40, 30 expected", b.evs("uplink_mismatch")[0]["problems"])
        self.assertEqual(R.f32(b.fake.gains["RATE_P"]), 45.0)          # then live to point 2

    def test_refused_after_two_failed_resends(self):
        b = Batch(small_spec(), analyze=False)
        self.addCleanup(b.close)
        drop(b.fake, "GAIN RATE_P 30")
        b.run()
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "uplink_unverified")
        self.assertEqual(b.cmds().count("GAIN RATE_P 30"), 1 + R.RESEND_TRIES)
        self.assertNotIn("RESET", b.cmds())
        self.assertNotIn("COLL 1000", b.cmds())
        self.assertEqual(b.fake.estops, 0)
        self.assertFalse(b.fake.hb_times)                 # never got as far as the heartbeat
        self.assertTrue([e for e in b.evs("note") if "refused before RESET" in e["text"]])

    def test_coll_is_verified_before_the_reset(self):
        b = Batch(small_spec(), analyze=False)
        self.addCleanup(b.close)
        drop(b.fake, "COLL 1000", times=1)
        b.run()
        self.assertEqual(b.status, "ok")
        j = b.journal()
        i_coll = ev_index(j, lambda e: e.get("ev") == "cmd" and e["text"] == "COLL 1000")
        i_read = ev_index(j, lambda e: e.get("ev") == "rig_read" and e["stage"] == "coll/resend1")
        i_reset = ev_index(j, lambda e: e.get("ev") == "cmd" and e["text"] == "RESET")
        self.assertLess(i_coll, i_read)
        self.assertLess(i_read, i_reset)
        self.assertEqual(j[i_read]["coll_permille"], 1000)
        self.assertEqual(j[i_read]["rs_name"], "DONE")     # cannot arm before the RESET
        cmds = b.cmds()
        self.assertEqual(cmds.index("RESET"), len(cmds) - 1 - cmds[::-1].index("COLL 1000") + 1)
        arm = [t for t, e in b.fake.events if e.startswith("armed")][0]
        reset_t = [t for t, text, _ in b.fake.commands if text == "RESET"][0]
        self.assertGreater(arm, reset_t)

    def test_coll_never_landing_refuses_without_reset(self):
        b = Batch(small_spec(), analyze=False)
        self.addCleanup(b.close)
        drop(b.fake, "COLL 1000")
        b.run()
        self.assertEqual(b.status, "uplink_unverified")
        self.assertNotIn("RESET", b.cmds())
        self.assertEqual(b.fake.coll_cmd, 0.0)

    def test_reset_first_order_when_the_reset_edge_clears_coll(self):
        with mock.patch.object(R, "COLL_SURVIVES_RESET", False):
            b = Batch(small_spec(), analyze=False).run()
        self.addCleanup(b.close)
        self.assertEqual(b.status, "ok")
        cmds = b.cmds()
        self.assertEqual(cmds[cmds.index("RESET") + 1], "COLL 1000")
        self.assertFalse([e for e in b.evs("rig_read") if e["stage"] == "coll"])


class PreResetTest(unittest.TestCase):
    def _latch_at_heartbeat(self, b, **flags):
        real = b.fake.tx_hb

        def tx_hb():
            for k, v in flags.items():
                setattr(b.fake, k, v)
            return real()
        b.fake.tx_hb = tx_hb

    def test_kill_latch_before_reset_refuses(self):
        b = Batch(small_spec(), analyze=False)
        self.addCleanup(b.close)
        self._latch_at_heartbeat(b, kill=True)
        b.run()
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "estop")
        self.assertNotIn("RESET", b.cmds())
        self.assertIn("kill=1", b.evs("estop")[-1]["reason"])
        self.assertFalse([t for t, e in b.fake.events if e.startswith("armed")])

    def test_fc_estop_before_reset_refuses_even_with_clear_estop(self):
        b = Batch(small_spec(), analyze=False, clear_estop=True)
        self.addCleanup(b.close)
        self._latch_at_heartbeat(b, estop=True)
        b.run()
        self.assertEqual(b.status, "estop")
        self.assertNotIn("RESET", b.cmds())

    def test_clear_estop_clears_first_then_runs_the_normal_order(self):
        b = Batch(small_spec(), analyze=False, clear_estop=True)
        self.addCleanup(b.close)
        b.fake.estop = b.fake.kill = True
        b.run()
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "ok")
        cmds = b.cmds()
        self.assertEqual(cmds[:3], ["COLL 0", "RESET", "STOP"])
        self.assertEqual(cmds.count("RESET"), 2)
        self.assertLess(cmds.index("COLL 1000"), len(cmds) - 1 - cmds[::-1].index("RESET"))

    def test_latched_estop_without_the_flag_refuses(self):
        b = Batch(small_spec(), analyze=False)
        self.addCleanup(b.close)
        b.fake.estop = b.fake.kill = True
        b.run()
        self.assertEqual(b.status, "estop")
        self.assertNotIn("RESET", b.cmds())


class ArmDiagnosisTest(unittest.TestCase):
    def test_arm_failure_is_diagnosed_before_the_stop(self):
        b = Batch(small_spec(), analyze=False)
        self.addCleanup(b.close)
        b.fake.gyro_cal_done = False
        b.run()
        self.assertIsNone(b.exc, repr(b.exc))
        self.assertEqual(b.status, "arm_failed")
        j = b.journal()
        i_diag = ev_index(j, lambda e: e.get("ev") == "arm_diag")
        d = j[i_diag]
        self.assertIn("gyro calibration not done", d["reasons"])
        self.assertEqual(d["coll_permille"], 1000)        # read before the STOP that zeroes it
        self.assertTrue(d["arm_enabled"])
        self.assertIn("HB fresh until the host stopped it", d["notes"])
        i_reset = ev_index(j, lambda e: e.get("ev") == "cmd" and e["text"] == "RESET")
        stops = [i for i, e in enumerate(j) if e.get("ev") == "cmd" and e["text"] == "STOP"]
        self.assertFalse([i for i in stops if i_reset < i < i_diag])
        self.assertTrue([i for i in stops if i > i_diag])
        self.assertTrue([e for e in b.evs("note") if "gyro calibration" in e["text"]])
        self.assertTrue([e for e in b.evs("note") if "never armed" in e["text"]])
        self.assertEqual(b.fake.estops, 0)

    def test_a_lost_reset_is_named(self):
        b = Batch(small_spec(), analyze=False)
        self.addCleanup(b.close)
        drop(b.fake, "RESET")
        b.run()
        self.assertEqual(b.status, "arm_failed")
        d, = b.evs("arm_diag")
        self.assertTrue([r for r in d["reasons"] if r.startswith("RESET never reached the FC")])
        self.assertTrue([r for r in d["reasons"] if r.startswith("arming not enabled")])

    def test_armed_state_comes_from_rbstat(self):
        b = Batch(small_spec(), analyze=False).run()
        self.addCleanup(b.close)
        st = [e for e in b.evs("state") if e.get("src") == "rbstat"]
        self.assertEqual([e["rs"] for e in st], ["RAMP_UP", "RUN"])
        self.assertTrue(int(st[0]["fl"], 16) & 0x01)
        self.assertTrue(any(st[1]["duty"]))
        self.assertTrue([e for e in b.evs("event") if "motors confirmed by RBSTAT" in e["text"]])


class LiftoffTest(unittest.TestCase):
    def test_first_exc_waits_out_the_liftoff_window(self):
        b = Batch(small_spec(), analyze=False).run()
        self.addCleanup(b.close)
        self.assertEqual(b.status, "ok")
        run_t = [t for t, e in b.fake.events if e == "run"][0]
        exc_t = [t for t, text, _ in b.fake.commands if text.startswith("EXC")][0]
        self.assertGreaterEqual(exc_t - run_t, R.LIFTOFF_TIMEOUT_MS / 1000.0)
        self.assertFalse([n for n in b.fake.naks if "lifted" in n[2]])

    def test_fake_refuses_exc_before_liftoff(self):
        clock = R.SimClock(SPEED)
        f = R.FakeDrone(clock)
        f.rs, f.guard = "RUN", False
        f._fc_command("EXC step att 50 500")
        self.assertEqual(f.naks[-1][2], "EXC: not lifted off yet")


class RxHealthTest(unittest.TestCase):
    def test_rx_counters_are_journaled_per_spin(self):
        b = Batch(small_spec(), analyze=False).run()
        self.addCleanup(b.close)
        rx, = b.evs("uplink_rx")
        self.assertEqual(rx["spin"], 1)
        self.assertEqual(set(rx["before"]), {"g_uart1_rx_overruns", "g_uart1_lines_ok",
                                             "g_uart1_lines_bad"})
        self.assertGreater(rx["delta"]["g_uart1_lines_ok"], 100)
        self.assertEqual(rx["delta"]["g_uart1_lines_bad"], 0)

    def test_absent_counters_are_skipped(self):
        b = Batch(small_spec(), analyze=False, fake_kw={"rx_counters": False}).run()
        self.addCleanup(b.close)
        self.assertEqual(b.status, "ok")
        self.assertFalse(b.evs("uplink_rx"))
        self.assertTrue([e for e in b.evs("note") if "RX counters" in e["text"]])


class FirmwareDependencyTest(unittest.TestCase):
    def setUp(self):
        self.src = FC_RIG_CPP.read_text()

    def test_reset_edge_keeps_coll_cmd(self):
        """COLL_SURVIVES_RESET must say what fc_rig_step's reset_gen edge does to coll_cmd."""
        i = self.src.index("if (in->reset_gen != s->reset_seen) {")
        depth, j = 0, self.src.index("{", i)
        while True:
            c = self.src[j]
            depth += c == "{"
            depth -= c == "}"
            j += 1
            if depth == 0:
                break
        block = self.src[i:j]
        self.assertEqual("coll_cmd" not in block, R.COLL_SURVIVES_RESET, block)
        self.assertIn("s->coll_cmd = 0.0f", self.src[self.src.index("static void to_done"):])

    def test_limit_defaults_match_fc_rig_cpp(self):
        for name, v in R.FC_LIMIT_DEFAULTS:
            if v is None:
                continue
            m = re.search(r"#define %s\s+(-?[\d.]+)f?" % name, self.src)
            self.assertIsNotNone(m, name)
            self.assertAlmostEqual(float(m.group(1)), v, places=6, msg=name)
        m = re.search(r"#define RIG_TGT_MAX_RAD\s+\(RIG_MAX_TILT_RAD - ([\d.]+)f\)", self.src)
        self.assertIsNotNone(m)
        self.assertAlmostEqual(float(m.group(1)), R.TGT_MARGIN_RAD)

    def test_dwarf_layout_of_a_rig_build(self):
        elf = R.default_elf("rig-roll")
        if not elf.is_file():
            self.skipTest("no rig-roll build at %s" % elf)
        try:
            import elftools  # noqa: F401
        except ImportError:
            self.skipTest("pyelftools (zephyr env)")
        lay = R.RigLayout.from_elf(str(elf))
        self.assertEqual(lay.source, "dwarf")
        # gain_seq follows the float gain table, whose length is the firmware's gain count
        self.assertEqual((lay.offsets["gains"], lay.offsets["gain_seq"]), (0, 4 * len(R.GAIN_NAMES)))
        for p in R.RIG_REQUIRED + ("reset_seen", "hb_seen", "last_hb_ms", "dbg.roll"):
            self.assertIn(p, lay.offsets)
        self.assertIn("arm_enabled", lay.fc_globals)      # a file static: DWARF, not .symtab
        self.assertIn("fc_tick", lay.fc_globals)
        regions = lay.regions()
        snap = lay.decode(regions, [bytes(n) for _, n in regions])
        self.assertEqual(snap["rs_name"], "IDLE")


class FcLimitsTest(unittest.TestCase):
    def _build(self, tmp, cppflags):
        d = Path(tmp) / "rig-roll"
        (d / "zephyr").mkdir(parents=True)
        elf = d / "zephyr" / "zephyr.elf"
        elf.write_bytes(b"\x7fELF")
        (d / "rb-build.txt").write_text(
            "workload: flight_controller\ncmake:        -DROSE_RIG=1 -DPID_MASS_KG=0.050f\n"
            "cppflags:     %s\n" % cppflags)
        return elf

    def test_effective_limits_from_the_build_record(self):
        with tempfile.TemporaryDirectory() as tmp:
            elf = self._build(tmp, "-DATT_GAIN=0.85f -DRIG_MAX_TILT_RAD=0.227f -DRIG_TGT_MAX_RAD=0.14f")
            lim = R.fc_limits(elf)
            self.assertAlmostEqual(lim["RIG_MAX_TILT_RAD"], 0.227)
            self.assertAlmostEqual(lim["RIG_TGT_MAX_RAD"], 0.14)
            self.assertEqual(lim["RIG_SP_MAX_MDEG"], R.SP_MAX_MDEG)
            self.assertAlmostEqual(lim["PID_MASS_KG"], 0.05)
            self.assertIn("rb-build.txt", lim["source"])
            # tilt only: the target follows it
            elf2 = self._build(Path(tmp) / "b", "-DRIG_MAX_TILT_RAD=0.30f")
            self.assertAlmostEqual(R.fc_limits(elf2)["RIG_TGT_MAX_RAD"], 0.30 - R.TGT_MARGIN_RAD)
        d = R.fc_limits(None)
        self.assertAlmostEqual(d["RIG_TGT_MAX_RAD"], R.MAX_TILT_RAD - R.TGT_MARGIN_RAD)

    def test_compile_commands_fallback(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp)
            (d / "zephyr").mkdir()
            (d / "compile_commands.json").write_text(json.dumps([
                {"file": "/x/src/fc_rig.cpp", "command": "g++ -DROSE_RIG=1 -DRIG_TGT_MAX_RAD=0.2f -c x"}]))
            lim = R.fc_limits(d / "zephyr" / "zephyr.elf")
            self.assertAlmostEqual(lim["RIG_TGT_MAX_RAD"], 0.2)
            self.assertIn("compile_commands.json", lim["source"])

    def test_spec_checked_against_the_roll_target_limit(self):
        with tempfile.TemporaryDirectory() as tmp:
            lim = R.fc_limits(self._build(tmp, "-DRIG_MAX_TILT_RAD=0.227f -DRIG_TGT_MAX_RAD=0.14f"))
        ok = small_spec(excitations=[{"type": "step", "target": "att", "amp": 52, "dur_ms": 600}])
        spec = R.validate_spec(ok, lim)
        self.assertAlmostEqual(spec["fc_limits"]["RIG_TGT_MAX_RAD"], 0.14)
        # 5 deg setpoint + 0.05 rad trim + 3 deg step = 0.1873 rad > 0.14
        bad = dict(ok, roll_sp_mdeg=5000, base_gains={"ATT_P": 5.0, "ROLL_TRIM": 0.05})
        with self.assertRaises(R.SpecError) as cm:
            R.validate_spec(bad, lim)
        self.assertTrue([p for p in cm.exception.problems if "RIG_TGT_MAX_RAD 8.0 deg" in p])
        # the same spec is fine against the default 20 deg target
        R.validate_spec(bad)
        # and the tilt budget follows the 13 deg guard (6 deg left)
        with self.assertRaises(R.SpecError) as cm:
            R.validate_spec(small_spec(excitations=[{"type": "step", "target": "rate", "amp": 600,
                                                     "dur_ms": 600}]), lim)
        self.assertTrue([p for p in cm.exception.problems if "roll estop is at 13 deg" in p])
        with self.assertRaises(R.SpecError) as cm:
            R.validate_spec(dict(ok, mass_kg=0.06), lim)
        self.assertTrue([p for p in cm.exception.problems if "PID_MASS_KG" in p])

    def test_check_prints_and_batch_records_the_limits(self):
        with tempfile.TemporaryDirectory() as tmp:
            elf = self._build(tmp, "-DRIG_MAX_TILT_RAD=0.227f -DRIG_TGT_MAX_RAD=0.14f")
            spec = Path(tmp) / "s.json"
            spec.write_text(json.dumps(small_spec(
                excitations=[{"type": "step", "target": "att", "amp": 52, "dur_ms": 600}])))
            out = io.StringIO()
            with no_drone(), contextlib.redirect_stdout(out):
                self.assertEqual(R.main(["run", str(spec), "--check", "--elf", str(elf)]), 0)
            self.assertIn("roll target 8.0 deg", out.getvalue())
            self.assertIn("tilt guard 13.0 deg", out.getvalue())
            with no_drone(), contextlib.redirect_stdout(_DEVNULL):
                self.assertEqual(R.main(["run", str(spec), "--dry-run", "--speed", str(SPEED),
                                         "--elf", str(elf), "--out-dir", tmp]), 0)
            (bdir,) = [p for p in Path(tmp).iterdir() if p.is_dir() and p.name.endswith("dryrun")]
            meta = T.read_json(bdir / "batch.json")
            self.assertAlmostEqual(meta["fc_limits"]["RIG_TGT_MAX_RAD"], 0.14)
            self.assertTrue(meta["coll_survives_reset"])


if __name__ == "__main__":
    unittest.main()
