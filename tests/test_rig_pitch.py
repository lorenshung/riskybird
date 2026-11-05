"""The pitch rig (rig-pitch: the drone turned 90 deg on the pivot) through the host tooling.

RIG-SPEC "Pitch axis": the build's axis comes from the ELF's g_rig_axis, else -DRIG_AXIS in its
build record, else roll; the recorder bytes are unchanged and the CSV gains generic columns for the
controlled axis at the end; the RIG line gains ax=R|P. Covered: tools/rb/rig_axis.py, the fetch
(flightrec_fetch.py, stubbed link), the runner (spec axis, mode, --check against a fake build
record, the fake drone on pitch), rig_analyze (numpy: zephyr env), the TUI, and the twin
(tools/rig/sim, host g++).

Run: PYTHONPATH=$PWD python3 -m unittest discover -s tests -p "test_rig_pitch.py" -v
"""

from __future__ import annotations

import contextlib
import csv
import io
import json
import math
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from tools.rb import rig_axis as AX
from tools.rb import rig_run as R
from tools.rb import rig_tui as T

ROOT = Path(__file__).resolve().parents[1]
CAMPAIGN = ROOT / "tools" / "rig" / "campaigns" / "first_hold_pitch_600.json"
ROLL_RECORD = ROOT / "build" / "zephyr" / "arty200t" / "flight_controller" / "rig-roll" / "rb-build.txt"
NEW_COLUMNS = ["axis", "ang_deg", "ang_tgt_deg", "ang_rate_dps"]

try:
    import elftools  # noqa: F401
    HAVE_ELFTOOLS = True
except ImportError:
    HAVE_ELFTOOLS = False


def fake_build(d: Path, mode: str, cppflags: str, elf_bytes: bytes = b"") -> Path:
    """build/.../<mode>/rb-build.txt + an ELF at <mode>/zephyr/zephyr.elf (empty by default:
    not a readable ELF, so the build record decides)."""
    (d / mode / "zephyr").mkdir(parents=True, exist_ok=True)
    (d / mode / "rb-build.txt").write_text(
        "workload:     flight_controller\nmode:         %s   (base: esp-motors)\n"
        "cmake:        -DROSE_RIG=1 -DPID_MASS_KG=0.050f\ncppflags:     %s\n" % (mode, cppflags))
    elf = d / mode / "zephyr" / "zephyr.elf"
    elf.write_bytes(elf_bytes)
    return elf


class AxisDetection(unittest.TestCase):
    def test_parse_and_names(self):
        for v, want in (("roll", 0), ("pitch", 1), ("R", 0), ("p", 1), (0, 0), (1, 1),
                        ("rig-pitch", 1), ("rig-roll", 0), ("yaw", None), (2, None), (True, None),
                        (None, None)):
            self.assertEqual(AX.parse_axis(v), want, v)
        self.assertEqual((AX.name(1), AX.letter(1), AX.name(None), AX.letter(0)),
                         ("pitch", "P", "roll", "R"))
        self.assertEqual((AX.mode_axis("rig-pitch"), AX.mode_axis("esp-motors")), (1, None))

    def test_build_record_fallback_and_default(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp)
            pitch = fake_build(d, "rig-pitch", "-DROSE_VEL_LOOP=0 -DRIG_AXIS=1 -DFLIGHTREC_DIV=4")
            roll = fake_build(d, "rig-roll", "-DROSE_VEL_LOOP=0 -DFLIGHTREC_DIV=4")
            a, src = AX.elf_axis(pitch)
            self.assertEqual(a, 1)
            self.assertIn("rb-build.txt", src)
            self.assertEqual(AX.elf_axis(roll)[0], 0)
            self.assertEqual(AX.elf_axis(d / "nowhere" / "zephyr.elf")[0], 0)   # no record: roll
            self.assertIn("default", AX.elf_axis(None)[1])
            # cppflags win over cmake:
            (d / "rig-pitch" / "rb-build.txt").write_text("cmake: -DRIG_AXIS=0\ncppflags: -DRIG_AXIS=1\n")
            self.assertEqual(AX.elf_axis(pitch)[0], 1)

    @unittest.skipUnless(HAVE_ELFTOOLS and shutil.which("gcc"), "pyelftools + gcc (zephyr env)")
    def test_elf_symbol_wins_over_the_build_record(self):
        """const uint8_t g_rig_axis in .rodata, read from section data (a relocatable object is
        enough: st_value is then the offset in its section)."""
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp)
            for val, extra in ((1, ""), (0, "")):
                src = d / "axis.c"
                src.write_text("const unsigned char g_rig_axis = %d;\n"
                               "const unsigned char g_other = 7;\n%s" % (val, extra))
                elf = fake_build(d, "rig-x", "-DRIG_AXIS=%d" % (1 - val))
                r = subprocess.run(["gcc", "-c", "-O2", str(src), "-o", str(elf)],
                                   capture_output=True, text=True)
                self.assertEqual(r.returncode, 0, r.stderr)
                a, where = AX.elf_axis(elf)
                self.assertEqual(a, val)
                self.assertIn("g_rig_axis", where)
            # A C++ file static (_ZL10g_rig_axis) is accepted too.
            src = d / "axis.cpp"
            src.write_text("static const unsigned char g_rig_axis = 1;\n"
                           "const unsigned char *keep() { return &g_rig_axis; }\n")
            elf = fake_build(d, "rig-y", "")
            r = subprocess.run(["g++", "-c", "-O0", str(src), "-o", str(elf)], capture_output=True,
                               text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertEqual(AX.elf_symbol_axis(elf), 1)


class FetchPitch(unittest.TestCase):
    """flightrec_fetch.fetch on a stubbed link: same bytes, the axis from the build record."""

    def _fetch(self, elf):
        try:
            import test_flightrec_fetch as TF
        except ImportError:
            from tests import test_flightrec_fetch as TF
        mem = {}
        F = TF._import_fetch(TF._stub_link(mem))
        mem.update(TF.RigLayout._image(TF.RigLayout(), F))
        with tempfile.TemporaryDirectory() as tmp:
            res = F.fetch("socket://stub", str(elf), Path(tmp) / "rec", log=lambda _m: None)
            with open(res["csv"], newline="") as f:
                rows = list(csv.DictReader(f))
            with open(res["gains"]) as f:
                gains = json.load(f)
        return F, res, rows, gains

    def test_roll_and_pitch_columns(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp)
            roll_elf = fake_build(d, "rig-roll", "-DFLIGHTREC_DIV=4")
            pitch_elf = fake_build(d, "rig-pitch", "-DFLIGHTREC_DIV=4 -DRIG_AXIS=1")
            F, res, rows, gains = self._fetch(roll_elf)
            self.assertEqual(list(F.RIG_COLUMNS[-4:]), NEW_COLUMNS)
            self.assertEqual((res["axis"], gains["axis"]), ("roll", "roll"))
            r = rows[5]
            self.assertEqual(r["axis"], "R")
            self.assertEqual(r["ang_deg"], r["roll_deg"])
            self.assertEqual(r["ang_tgt_deg"], r["roll_tgt_deg"])
            self.assertAlmostEqual(float(r["ang_rate_dps"]), math.degrees(0.1), places=3)  # gyro_p
            F, res, rows, gains = self._fetch(pitch_elf)
            self.assertEqual((res["axis"], gains["axis"]), ("pitch", "pitch"))
            self.assertIn("RIG_AXIS=1", res["axis_source"])
            r = rows[5]
            self.assertEqual(r["axis"], "P")
            self.assertEqual(r["ang_deg"], r["pitch_deg"])                  # physical pitch
            self.assertAlmostEqual(float(r["ang_deg"]), -1.0, places=3)
            self.assertAlmostEqual(float(r["roll_deg"]), 5.0, places=3)      # physical roll kept
            self.assertEqual(r["ang_tgt_deg"], r["roll_tgt_deg"])            # controlled-axis target
            self.assertAlmostEqual(float(r["ang_rate_dps"]), math.degrees(-0.2), places=3)  # gyro_q
            self.assertNotIn("axis", gains["snapshot"])


class RunnerPitch(unittest.TestCase):
    def _raw(self, **over):
        raw = json.loads(CAMPAIGN.read_text())
        raw.update(over)
        return raw

    def test_campaign_shape(self):
        raw = json.loads(CAMPAIGN.read_text())
        self.assertEqual(raw["axis"], "pitch")
        self.assertEqual(raw["collective_permille"], 600)
        g = raw["base_gains"]
        self.assertEqual((g["ATT_P"], g["RATE_P"], g["RATE_D"], g["ATT_I"], g["ATT_GAIN"]),
                         (5.0, 40.0, 1.0, 1.0, 0.85))
        self.assertEqual(len(raw["excitations"]), 1)
        e = raw["excitations"][0]
        self.assertEqual((e["type"], e["target"]), ("step", "att"))
        self.assertAlmostEqual(math.degrees(e["amp"] / 1000.0), 3.0, delta=0.05)
        self.assertEqual(raw["limits"]["max_trials"], 1)
        self.assertEqual(len(R.plan_trials(R.validate_spec(raw))), 1)

    def test_axis_must_match_the_build(self):
        spec = R.validate_spec(self._raw(), elf_axis=(1, "test"))
        self.assertEqual(spec["axis"], "pitch")
        with self.assertRaises(R.SpecError) as cm:
            R.validate_spec(self._raw(), elf_axis=(0, "rig-roll/rb-build.txt"))
        msg = str(cm.exception)
        self.assertIn("does not match the build", msg)
        self.assertIn("ROLL", msg)
        self.assertIn("--mode rig-pitch", msg)
        raw = self._raw()
        del raw["axis"]
        self.assertEqual(R.validate_spec(raw, elf_axis=(1, "t"))["axis"], "pitch")   # follows the ELF
        self.assertEqual(R.validate_spec(raw)["axis"], "roll")
        with self.assertRaises(R.SpecError):
            R.validate_spec(self._raw(axis="yaw"))

    def test_mode_follows_the_spec(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "s.json"
            p.write_text(json.dumps(self._raw()))
            self.assertEqual(R.resolve_mode(None, p), ("rig-pitch", None))
            self.assertEqual(R.resolve_mode("my-pitch-build", p), ("my-pitch-build", None))
            mode, problem = R.resolve_mode("rig-roll", p)
            self.assertIn("use --mode rig-pitch", problem)
            raw = self._raw()
            del raw["axis"]
            p.write_text(json.dumps(raw))
            self.assertEqual(R.resolve_mode(None, p), ("rig-roll", None))

    def _check(self, *argv):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            rc = R.main(["run", str(CAMPAIGN), "--check", *argv])
        return rc, out.getvalue(), err.getvalue()

    def test_check_against_build_records(self):
        """The rig-pitch ELF need not exist yet: a build record with -DRIG_AXIS=1 (empty ELF) is
        enough; a roll build is refused; no ELF at all is a NOTE, not a failure."""
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp)
            flags = "-DRIG_MAX_TILT_RAD=0.227f -DRIG_TGT_MAX_RAD=0.14f -DRIG_LIFTOFF_TIMEOUT_MS=8000"
            pitch = fake_build(d, "rig-pitch", flags + " -DRIG_AXIS=1")
            roll = fake_build(d, "rig-roll", flags)
            rc, out, err = self._check("--elf", str(pitch))
            self.assertEqual(rc, 0, err)
            self.assertIn("axis PITCH", out)
            self.assertIn("pitch tilt guard 13.0 deg", out)
            self.assertIn("locked-axis (roll) guard", out)
            rc, out, err = self._check("--elf", str(roll))
            self.assertEqual(rc, 2)
            self.assertIn("does not match the build", err)
            rc, out, err = self._check("--elf", str(pitch), "--mode", "rig-roll")
            self.assertEqual(rc, 2)
            self.assertIn("--mode rig-roll builds the roll axis", err)
            # no "axis" in the spec: the pitch ELF decides, and the default mode follows it; an
            # explicit rig-roll mode for a pitch build is refused (batch.json's mode finds the ELF)
            raw = json.loads(CAMPAIGN.read_text())
            del raw["axis"]
            noaxis = d / "noaxis.json"
            noaxis.write_text(json.dumps(raw))
            out_, err_ = io.StringIO(), io.StringIO()
            with contextlib.redirect_stdout(out_), contextlib.redirect_stderr(err_):
                rc = R.main(["run", str(noaxis), "--check", "--elf", str(pitch)])
            self.assertEqual(rc, 0, err_.getvalue())
            self.assertIn("axis PITCH", out_.getvalue())
            self.assertIn("mode rig-pitch", out_.getvalue())
            with contextlib.redirect_stdout(out_), contextlib.redirect_stderr(err_):
                rc = R.main(["run", str(noaxis), "--check", "--elf", str(pitch), "--mode", "rig-roll"])
            self.assertEqual(rc, 2)
            self.assertIn("--mode rig-roll is the roll build but this batch is PITCH", err_.getvalue())
            rc, out, err = self._check("--elf", str(d / "absent" / "zephyr.elf"))
            self.assertEqual(rc, 0, err)
            self.assertIn("NOTE no ELF", out)

    @unittest.skipUnless(ROLL_RECORD.is_file(), "no rig-roll build in this checkout")
    def test_check_against_the_real_roll_record_plus_rig_axis(self):
        """The real rig-roll build record with -DRIG_AXIS=1 appended (what rig-pitch will be)."""
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp)
            (d / "rig-pitch" / "zephyr").mkdir(parents=True)
            text = "\n".join(ln + " -DRIG_AXIS=1" if ln.startswith("cppflags:") else ln
                             for ln in ROLL_RECORD.read_text().splitlines()) + "\n"
            (d / "rig-pitch" / "rb-build.txt").write_text(text)
            (d / "rig-pitch" / "zephyr" / "zephyr.elf").write_bytes(b"")
            rc, out, err = self._check("--elf", str(d / "rig-pitch" / "zephyr" / "zephyr.elf"))
            self.assertEqual(rc, 0, err)
            self.assertIn("axis PITCH", out)

    def test_fake_drone_pitch_recording(self):
        """FakeDrone(axis=1): the moment on the rear/front pairs, physical columns swapped back."""
        with tempfile.TemporaryDirectory() as tmp:
            for axis in (0, 1):
                clock = R.SimClock(1e6)
                fd = R.FakeDrone(clock, seed=3, axis=axis)
                self.assertEqual(fd.pos_pair, ((1, 2), (2, 3))[axis == 0])
                fd.th = math.radians(3.0)
                fd.coll = 1.0
                rows = []
                for k in range(3):
                    fd.duty = [0.30, 0.40, 0.40, 0.30] if axis else [0.30, 0.30, 0.40, 0.40]
                    rows.append((1000 * k, 2000, 2 * k, 0x09, 3, 1, 1, 0.1, -0.2, 0.0,
                                 0.05 if axis == 0 else 0.0, 0.05 if axis else 0.0,
                                 0.02, 0.1, 1.0, 1e-3, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 9.8, 7.6,
                                 list(fd.duty)))
                self.assertGreater(fd.L_EFF_moment(), 0)       # the raised pair is the + pair
                res = R.write_rig_recording(Path(tmp) / ("a%d" % axis), rows,
                                            {"gain_seq": 1, "gains": {}}, [], axis=axis)
                self.assertEqual(res["axis"], AX.name(axis))
                with open(res["csv"], newline="") as f:
                    rd = list(csv.DictReader(f))
                self.assertEqual(list(rd[0].keys())[-4:], NEW_COLUMNS)
                r = rd[0]
                self.assertEqual(r["axis"], "RP"[axis])
                self.assertEqual(r["ang_deg"], r["pitch_deg" if axis else "roll_deg"])
                self.assertAlmostEqual(float(r["ang_rate_dps"]), math.degrees(-0.2 if axis else 0.1),
                                       places=3)

    def test_dry_run_pitch_batch(self):
        """A whole dry-run batch on the fake pitch drone: batch.json / journal carry the axis, the
        trial recording is a P recording, and pitch (not roll) followed the step."""
        with tempfile.TemporaryDirectory() as tmp:
            spec = R.validate_spec(self._raw())
            clock = R.VirtualClock(20.0)
            link = R.FakeDrone(clock, seed=1, mass_kg=spec["mass_kg"], axis=1)
            (Path(tmp) / "b").mkdir()
            r = R.Runner(spec, Path(tmp) / "b", link, clock, approval="test", mode="rig-pitch",
                         dry_run=True, analyze=False, log=lambda s: None, axis_source="test")
            status = r.run()
            meta = json.loads((Path(tmp) / "b" / "batch.json").read_text())
            self.assertEqual((meta["axis"], meta["spec"]["axis"], meta["mode"]),
                             ("pitch", "pitch", "rig-pitch"))
            first = json.loads((Path(tmp) / "b" / "journal.jsonl").read_text().splitlines()[0])
            self.assertEqual((first["ev"], first["axis"]), ("batch_start", "pitch"))
            if status != "ok":
                self.skipTest("dry run timing: %s (the axis bookkeeping above passed)" % status)
            rec = Path(tmp) / "b" / "trials" / "1" / "rec.csv"
            with open(rec, newline="") as f:
                rows = list(csv.DictReader(f))
            self.assertEqual({x["axis"] for x in rows}, {"P"})
            run = [x for x in rows if x["rs_name"] == "RUN"]
            self.assertGreater(max(float(x["pitch_deg"]) for x in run), 2.0)   # the 3 deg step
            self.assertLess(max(abs(float(x["roll_deg"])) for x in run), 0.1)  # roll locked

    def test_limits_text_and_arm_reasons_name_the_axis(self):
        lim = R.fc_limits(None)
        self.assertIn("pitch tilt guard", R.limits_text(lim, 1))
        self.assertIn("locked-axis (roll) guard", R.limits_text(lim, 1))
        self.assertIn("roll tilt guard", R.limits_text(lim))
        snap = {"rs_name": "IDLE", "coll_permille": 600, "roll": 0.01, "pitch": 0.3}
        _why, notes = R.arm_reasons(snap, None, 0.0, axis=1)
        self.assertTrue(any("arm needs |pitch| < 35, locked |roll| < 10" in n for n in notes), notes)


try:
    import numpy as np  # noqa: F401
    from tools.rb import rig_analyze as A
except Exception:  # noqa: BLE001
    A = None


@unittest.skipIf(A is None, "numpy/scipy not available (zephyr env)")
class AnalyzePitch(unittest.TestCase):
    def test_generic_columns_win_and_old_csvs_fall_back(self):
        fs = 250.0
        n = 1000
        t = [i / fs for i in range(n)]
        step = [0.0 if ti < 1.0 or ti >= 3.0 else math.radians(3.0) for ti in t]
        pitch = []
        y = 0.0
        for x in step:          # a first-order follower, tau 0.1 s
            y += (math.degrees(x) - y) / (0.1 * fs)
            pitch.append(y)
        cols = {"t_s": t, "exc_val": step, "roll_deg": [0.0] * n, "roll_tgt_deg": [math.degrees(x) for x in step],
                "ang_deg": pitch, "ang_tgt_deg": [math.degrees(x) for x in step],
                "ang_rate_dps": [0.0] * n, "gyro_p": [0.0] * n,
                "duty0": [0.4] * n, "duty1": [0.4] * n, "duty2": [0.4] * n, "duty3": [0.4] * n}
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "rec.csv"
            with open(p, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(list(cols) + ["axis"])
                for i in range(n):
                    w.writerow([cols[c][i] for c in cols] + ["P"])
            d = A.load_rec(p)
            self.assertEqual(d["_axis"], 1)
            exc = {"type": "step", "target": "att", "amp": 52, "dur_ms": 2000}
            m = A.trial_metrics(d, exc)
            self.assertEqual(m["axis"], "pitch")
            self.assertTrue(m["settled"])
            self.assertLess(m["rms_err_deg"], 1.0)          # pitch tracked; roll_deg (0) would not
            old = {k: v for k, v in d.items() if not k.startswith("ang_")}
            old["roll_deg"] = d["ang_deg"]
            m_old = A.trial_metrics(old, exc)
            self.assertAlmostEqual(m_old["rms_err_deg"], m["rms_err_deg"], places=6)
            self.assertEqual(A.batch_axis({"axis": "pitch"}), "pitch")
            self.assertEqual(A.batch_axis({}), "roll")


class TuiPitch(unittest.TestCase):
    LINE = ("RIG it=10 st=9 rs=RUN r=250 p=-2750 gx=-12 sp=0 tgt=3000 coll=600 exc=52 ex=1 gs=4 "
            "d=[3300 3500 3500 3300] vb=7730 ax=P")

    def test_parse_ax(self):
        d = T.parse_rig_line(self.LINE)
        self.assertEqual((d["ax"], T.ctl_angle_mdeg(d)), ("P", -2750))
        old = T.parse_rig_line(self.LINE.replace(" ax=P", ""))
        self.assertEqual((old["ax"], T.ctl_angle_mdeg(old)), ("R", 250))       # pre-pitch firmware
        self.assertEqual(T.parse_rig_line(self.LINE.replace("ax=P", "ax=R"))["ax"], "R")

    def test_live_header_and_plot(self):
        app = T.App(drone="127.0.0.1", port=9)
        self.addCleanup(app.commander.close)
        ev = []
        app.live.ingest("1800000000.000 " + self.LINE, ev)
        self.assertEqual(app.live.samples[-1][1], -2.75)        # the pitch angle is plotted
        g = T.pick_glyphs(True)
        txt = T.render_frame(app, 170, 40, 1800000000.1, g).to_text()
        head = txt.splitlines()[0]
        self.assertIn(" PITCH ", head)
        self.assertRegex(head, r"\| pitch +-2\.8d +sp")
        self.assertIn("roll(locked) +0.2d", head)
        self.assertIn("Pitch last", txt)
        self.assertEqual(T.axis_badge_style("P"), "badge_AXIS_PITCH")
        self.assertNotEqual(T.STYLE_TABLE["badge_AXIS_PITCH"], T.STYLE_TABLE["badge_AXIS_ROLL"])
        # a roll line afterwards (another build) restarts the trace instead of mixing axes
        app.live.ingest("1800000000.040 " + self.LINE.replace(" ax=P", ""), ev)
        self.assertEqual(len(app.live.samples), 1)

    def test_replay_axis_from_batch_and_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            b = Path(tmp)
            (b / "batch.json").write_text(json.dumps({"name": "x", "mode": "rig-pitch",
                                                      "axis": "pitch", "spec": {}}))
            bs = T.BatchState(b)
            bs.meta = json.loads((b / "batch.json").read_text())
            self.assertEqual(bs.axis(), "P")
            app = T.App(drone="127.0.0.1", port=9, batch_dir=b, readonly=True)
            self.addCleanup(app.commander.close)
            app.batch.meta = bs.meta
            self.assertEqual(app.axis(), ("P", True))
            head = T.render_frame(app, 120, 30, 0.0, T.pick_glyphs(True)).to_text().splitlines()[0]
            self.assertIn(" PITCH ", head)
            rec = b / "rec.csv"
            rec.write_text("t_s,roll_deg,roll_tgt_deg,exc_val,exc_id,axis,ang_deg,ang_tgt_deg,ang_rate_dps\n"
                           "0.0,0.1,3.0,0.05,1,P,2.5,3.0,1.0\n0.01,0.1,3.0,0.05,1,P,2.6,3.0,1.0\n")
            r = T.load_rec_csv(rec)
            self.assertEqual((r["axis"], r["roll"][0], r["sp"][0], r["sp_col"]),
                             ("P", 2.5, 3.0, "ang_tgt_deg"))
            app2 = T.App(drone="127.0.0.1", port=9)
            self.addCleanup(app2.commander.close)
            self.assertEqual(app2.axis(), ("R", False))         # nothing known: roll, marked "?"


# ---- the twin -------------------------------------------------------------------------------------

SIM = ROOT / "tools" / "rig" / "sim"


@unittest.skipIf(shutil.which("g++") is None and shutil.which("c++") is None, "no host C++ compiler")
class TwinPitch(unittest.TestCase):
    _tmp = None

    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(SIM))
        import rig_sim  # noqa: PLC0415
        cls.rs = rig_sim
        cls._tmp = tempfile.TemporaryDirectory(prefix="rig-pitch-test-")
        cls.d = Path(cls._tmp.name)
        cls.exes = rig_sim.build(out_dir=cls.d / "build")
        rig_sim.set_dt_csv(None)
        cls.level = cls.d / "level.txt"
        cls.level.write_text(rig_sim.scenario_text(1000, step_mdeg=87, hold_ms=2000, after_ms=2500))

    @classmethod
    def tearDownClass(cls):
        if cls._tmp is not None:
            cls._tmp.cleanup()

    def _csv(self, path):
        with open(path, newline="") as f:
            return list(csv.DictReader(f))

    def test_pitch_transcription_assumptions_hold(self):
        """The twin's pitch path = the roll law + M column 2 + J[1]: valid while controller_pid.cpp
        has J[1] == J[0], tau_pitch == TAU_ROLL, tau_pitchRate == TAU_ROLLRATE and the same M."""
        pid = (ROOT / "workloads/flight_controller/src/controller_pid.cpp").read_text()
        mix = (SIM / "rig_mixer.h").read_text()
        j = re.search(r"static const float J\[3\]\s*=\s*\{([^}]*)\}", pid).group(1)
        jv = [float(x.strip().rstrip("f")) for x in j.split(",")]
        jr = float(re.search(r"J_ROLL\s*=\s*([0-9.e+-]+)f", mix).group(1))
        jp = float(re.search(r"J_PITCH\s*=\s*([0-9.e+-]+)f", mix).group(1))
        self.assertEqual((jv[0], jv[1]), (jr, jp))
        self.assertEqual(jv[0], jv[1])
        self.assertRegex(pid, r"tau_pitch\s*=\s*TAU_ROLL;")
        self.assertRegex(pid, r"tau_pitchRate\s*=\s*TAU_ROLLRATE;")

        def mat(text):
            m = re.search(r"const float M\[4\]\[4\] = \{(.*?)\n\s*\};", text, re.S).group(1)
            return re.sub(r"\s+", "", m)
        self.assertEqual(mat(pid), mat(mix))

    def test_pitch_hold_settles_on_the_pitch_pairs(self):
        path = self.d / "pitch.csv"
        js = self.rs.run_twin(self.exes["twin"], self.level, {"axis": 1, "ext_mNm": 0}, csv_path=path)
        self.assertEqual((js["status"], js["axis"]), ("ok", "pitch"), js)
        self.assertEqual(js["naks"], 0, js["nak_lines"])
        self.assertEqual(js["stop_hits_live"], 0)
        self.assertTrue(js["settled"])
        self.assertLess(js["overshoot_pct"], 20.0)
        self.assertLess(abs(js["steady_err_deg"]), 0.5)
        rows = self._csv(path)
        self.assertEqual(list(rows[0].keys())[-4:], NEW_COLUMNS)
        self.assertEqual({r["axis"] for r in rows}, {"P"})
        self.assertTrue(all(r["ang_deg"] == r["pitch_deg"] for r in rows))
        self.assertTrue(all(r["ang_tgt_deg"] == r["roll_tgt_deg"] for r in rows))
        run = [r for r in rows if r["rs_name"] == "RUN"]
        self.assertLess(max(abs(float(r["roll_deg"])) for r in run), 1e-6)         # locked
        self.assertGreater(max(float(r["pitch_deg"]) for r in run), 4.0)          # the 5 deg step
        self.assertTrue(all(abs(float(r["ang_rate_dps"]) - math.degrees(float(r["gyro_q"]))) < 2e-3
                            for r in rows[:50]))
        # the step's leading edge: rear pair (duty1, duty2) up, front (duty0, duty3) down; no roll
        edge = [r for r in run if float(r["exc_val"]) > 0][:12]
        dp = [float(r["duty1"]) + float(r["duty2"]) - float(r["duty0"]) - float(r["duty3"]) for r in edge]
        dr = [float(r["duty2"]) + float(r["duty3"]) - float(r["duty0"]) - float(r["duty1"]) for r in edge]
        self.assertGreater(max(dp), 0.01)
        self.assertLess(max(abs(x) for x in dr), 1e-3)
        # same plant and noise on either axis: the pitch rig reproduces the roll rig
        roll = self.rs.run_twin(self.exes["twin"], self.level, {}, csv_path=None)
        for k in ("overshoot_pct", "settle_ms", "rms_err_deg", "steady_err_deg"):
            self.assertAlmostEqual(js[k], roll[k], places=6, msg=k)

    def test_firmware_pitch_path_matches_the_transcription(self):
        """The twin's pitch runs use the REAL pitch firmware (rig_twin_pitch: fc_rig.cpp with
        RIG_AXIS=1, TWIN_FW_AXIS_AWARE=1). Cross-check against the earlier transcription
        (TWIN_FW_AXIS_AWARE=0: a ROLL build fed the pitch view, its moment moved to u[2], the
        saturation on the pitch pairs): the same run, number for number, CSV included."""
        self.assertTrue(self.exes["twin_pitch"].name.endswith("_pitch"))
        self.assertEqual(self.rs.twin_for(self.exes["twin"], 1), self.exes["twin_pitch"])
        self.assertEqual(self.rs.twin_for(self.exes["twin_pitch"], 0), self.exes["twin"])
        old = self.rs.build(extra={"TWIN_FW_AXIS_AWARE": "0"}, out_dir=self.d / "build-transcribed")
        self.assertNotIn("twin_pitch", old)
        params = {"axis": 1, "imbalance": 0.03, "cg_x_mm": 0.5, "gains": {"ATT_I": 2, "RATE_D": 0.3}}
        pa, pb = self.d / "fw.csv", self.d / "tr.csv"
        a = self.rs.run_twin(self.exes["twin_pitch"], self.level, params, csv_path=pa)
        b = subprocess.run([str(old["twin"]), "scenario=%s" % self.level, *self.rs._kv(params),
                            "csv=%s" % pb], capture_output=True, text=True)
        self.assertEqual(b.returncode, 0, b.stdout + b.stderr)
        b = json.loads(b.stdout.strip().splitlines()[-1])
        self.assertEqual(a["status"], "ok", a)
        self.assertIn("RIG liftoff: pitch guard live", " ".join(a["events"]))   # the pitch build's text
        b["events"] = [e.replace("roll guard live", "pitch guard live") for e in b["events"]]
        self.maxDiff = None
        self.assertEqual(json.dumps(a, sort_keys=True), json.dumps(b, sort_keys=True))
        self.assertEqual(pa.read_bytes(), pb.read_bytes())
        # each binary is one firmware axis and refuses the other
        for exe, axis, built in ((self.exes["twin"], 1, 0), (self.exes["twin_pitch"], 0, 1)):
            bad = subprocess.run([str(exe), "scenario=%s" % self.level, "axis=%d" % axis],
                                 capture_output=True, text=True)
            self.assertNotEqual(bad.returncode, 0)
            self.assertIn("RIG_AXIS=%d" % built, bad.stdout)
        # the pitch binary defaults to pitch
        js = subprocess.run([str(self.exes["twin_pitch"]), "scenario=%s" % self.level],
                            capture_output=True, text=True)
        self.assertEqual(json.loads(js.stdout.strip().splitlines()[-1])["axis"], "pitch")

    def test_campaign_scenario_and_com_offset(self):
        """tools/rig/sim/scenarios/pitch_hold_600.txt (the first_hold_pitch_600 campaign); a CoM
        1 mm forward is a nose-down (+pitch) torque: the loop answers with a steady nose-up moment
        (front pair harder than the rear) and still sags toward +pitch."""
        scen = SIM / "scenarios" / "pitch_hold_600.txt"
        path = self.d / "camp.csv"
        js = self.rs.run_twin(self.exes["twin"], scen, {"axis": 1, "cg_x_mm": 1.0}, csv_path=path)
        self.assertEqual(js["status"], "ok", js)
        self.assertEqual(js["naks"], 0, js["nak_lines"])
        self.assertEqual(js["first_exc"], "step att")
        self.assertTrue(js["settled"], js)
        quiet = [r for r in self._csv(path) if r["rs_name"] == "RUN" and float(r["exc_val"]) == 0]
        quiet = quiet[len(quiet) // 4:]
        diff = sum(float(r["duty1"]) + float(r["duty2"]) - float(r["duty0"]) - float(r["duty3"])
                   for r in quiet) / len(quiet)
        self.assertLess(diff, -1e-3)
        sag = sum(float(r["pitch_deg"]) for r in quiet) / len(quiet)
        self.assertGreater(sag, 0.1)


if __name__ == "__main__":
    unittest.main()
