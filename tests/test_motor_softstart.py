"""Host checks for the ESP motor spin-up gate and the PWM phase-interleave arithmetic.

Run with ``python3 -m unittest discover -s tests -p test_motor_softstart.py -v``.
Builds tests/motor_softstart_check.c against
workloads/lib/motor_link/include/riskybird/motor_softstart.h with the host compiler, as C and
as C++, and drives ss_step() with timed command sequences. The driver itself fails any step whose
output exceeds its command (rule 1). No Zephyr and no LEDC: this checks the logic and the hpoint
arithmetic, not the register writes in workloads/esp_bridge/src/main.c.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
INC = ROOT / "workloads/lib/motor_link/include"
DRIVER = Path(__file__).resolve().parent / "motor_softstart_check.c"
WARN = ["-Wall", "-Wextra", "-Werror"]

IDLE, HOLD_US, RATE, STAGGER_US, MAX_US = 1000, 80_000, 20_000, 25_000, 700_000
COLD, WAIT, HOLD, SLEW, RUN = range(5)
TICK = 5_000   # link_thread worst-case wake period, us


def _build(compiler, out, name, lang):
    exe = out / name
    cmd = [compiler, *(["-x", "c++", "-std=c++17"] if lang == "c++" else ["-std=c11"]),
           "-O2", *WARN, f"-I{INC}", str(DRIVER), "-o", str(exe)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise AssertionError(f"{' '.join(cmd)}\n{result.stderr}")
    return exe


class Run:
    """Outputs of one sim: rows of (now, tgt, out, phases, lim)."""

    def __init__(self, rows):
        self.rows = rows

    def at(self, now):
        for row in self.rows:
            if row[0] == now:
                return row
        raise KeyError(now)

    def first_reach(self, motor, value, after=0):
        for now, _tgt, out, _ph, _lim in self.rows:
            if now >= after and out[motor] >= value:
                return now
        return None


class MotorSoftstart(unittest.TestCase):
    _tmp = None

    @classmethod
    def setUpClass(cls):
        cc = shutil.which("gcc") or shutil.which("cc")
        cxx = shutil.which("g++") or shutil.which("c++")
        if cc is None or cxx is None:
            raise unittest.SkipTest("host C/C++ compiler unavailable")
        cls._tmp = tempfile.TemporaryDirectory(prefix="motor-softstart-test-")
        out = Path(cls._tmp.name)
        cls.exes = [_build(cc, out, "check_c", "c"), _build(cxx, out, "check_cxx", "c++")]

    @classmethod
    def tearDownClass(cls):
        if cls._tmp is not None:
            cls._tmp.cleanup()

    def sim(self, script, exe=None):
        """script: list of ("R",), ("C", now_us), ("U", now_us) or (now_us, [t0, t1, t2, t3]).
        R = ss_reset (disarm/kill/load), C = ss_cut (failsafe), U = ss_resume. Returns a Run."""
        lines, steps = [], []
        for item in script:
            if item[0] == "R":
                lines.append("R")
            elif item[0] in ("C", "U"):
                lines.append(f"{item[0]} {item[1] % 2**32}")
            else:
                now, tgt = item
                lines.append(f"S {now % 2**32} {' '.join(map(str, tgt))}")
                steps.append((now, list(tgt)))
        result = subprocess.run([str(exe or self.exes[0]), "sim"], input="\n".join(lines) + "\n",
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        outs = [r.split() for r in result.stdout.splitlines() if r.startswith("O ")]
        self.assertEqual(len(outs), len(steps))
        rows = []
        for (now, tgt), o in zip(steps, outs):
            word = int(o[5], 16)
            phases = [(word >> (12 - 4 * i)) & 0xF for i in range(4)]
            rows.append((now, tgt, [int(x) for x in o[1:5]], phases, int(o[6])))
        return Run(rows)

    @staticmethod
    def ticks(t0, t1, tgt, dt=TICK):
        return [(t, tgt) for t in range(t0, t1, dt)]

    def assert_slew_bounded(self, run, motor):
        """After the idle step, no tick raises the output faster than SS_RATE (+1 count)."""
        prev = None
        for now, _tgt, out, ph, _lim in run.rows:
            if prev is not None and ph[motor] in (SLEW, RUN) and prev[1] >= IDLE:
                allowed = RATE * (now - prev[0]) / 1e6 + 1
                if ph[motor] == SLEW:
                    self.assertLessEqual(out[motor] - prev[1], allowed, (now, motor))
            prev = (now, out[motor])

    # -- the contract -----------------------------------------------------------------------

    def test_step_to_0_6_is_gated_and_arrives_within_700ms(self):
        for exe in self.exes:
            run = self.sim(self.ticks(0, 1_000_000, [6000] * 4), exe=exe)
            first = run.at(0)
            self.assertEqual(first[2], [IDLE, 0, 0, 0])            # rank 0 holds, others wait
            self.assertEqual(first[3], [HOLD, WAIT, WAIT, WAIT])
            for m in range(4):
                reach = run.first_reach(m, 6000)
                self.assertIsNotNone(reach)
                self.assertLessEqual(reach, MAX_US)
                # rank m: stagger + hold + (0.6 - 0.1) / 2.0 s = 330 ms + 25 ms * m
                expect = m * STAGGER_US + HOLD_US + (6000 - IDLE) * 1_000_000 // RATE
                self.assertAlmostEqual(reach, expect, delta=TICK)
                self.assert_slew_bounded(run, m)
            # Released to RUN only when the envelope reaches full scale (S1, 2026-11-05):
            # go + 80 ms + 0.9 / 2.0 s = 530 ms + 25 ms * rank, so all four by ~605 ms.
            self.assertEqual(run.at(500_000)[3], [SLEW] * 4)
            self.assertEqual(run.at(500_000)[2], [6000] * 4)       # out == cmd, still gated
            self.assertEqual(run.at(610_000)[3], [RUN] * 4)
            self.assertGreater(run.rows[-1][4], 0)                 # sslim counted
            lim_at_run = run.at(500_000)[4]
            self.assertEqual(run.rows[-1][4], lim_at_run)          # and stops counting in RUN

    def test_hold_lasts_80ms_at_idle(self):
        run = self.sim(self.ticks(0, 200_000, [9800, 0, 0, 0]))
        for now in range(0, HOLD_US, TICK):
            self.assertEqual(run.at(now)[2][0], IDLE, now)
        self.assertGreater(run.at(HOLD_US + 10_000)[2][0], IDLE)

    def test_decreases_are_instant(self):
        script = self.ticks(0, 600_000, [6000] * 4)
        script += self.ticks(600_000, 700_000, [2000, 6000, 0, 3000])
        run = self.sim(script)
        self.assertEqual(run.at(600_000)[2], [2000, 6000, 0, 3000])
        # mid-slew decrease also instant, and below the envelope
        script = self.ticks(0, 150_000, [8000] * 4) + [(150_000, [1500, 1500, 300, 1500])]
        run = self.sim(script)
        self.assertEqual(run.at(150_000)[2][2], 300)
        before = run.at(145_000)[2]
        after = run.at(150_000)[2]
        for m, tgt in enumerate([1500, 1500, 300, 1500]):
            self.assertLessEqual(after[m], tgt)
            if tgt <= before[m]:
                self.assertEqual(after[m], tgt, m)   # any drop lands in the same tick

    def test_running_motor_is_not_limited(self):
        script = self.ticks(0, 600_000, [3000] * 4)
        script += self.ticks(600_000, 700_000, [9800, 3000, 9800, 3000])
        run = self.sim(script)
        self.assertEqual(run.at(600_000)[2], [9800, 3000, 9800, 3000])
        lim = run.at(595_000)[4]
        self.assertEqual(run.rows[-1][4], lim)

    def test_stagger_is_by_rank_among_starters(self):
        # motors 3 and 4 start together: motor 3 is rank 0 (no wait), motor 4 waits 25 ms
        run = self.sim(self.ticks(0, 200_000, [0, 0, 5000, 5000]))
        self.assertEqual(run.at(0)[2], [0, 0, IDLE, 0])
        self.assertEqual(run.at(20_000)[2][3], 0)
        self.assertEqual(run.at(25_000)[2][3], IDLE)
        # a lone start later (motor 1) waits nothing even though others are running
        script = self.ticks(0, 600_000, [0, 0, 5000, 5000]) + self.ticks(600_000, 700_000,
                                                                       [5000, 0, 5000, 5000])
        run = self.sim(script)
        self.assertEqual(run.at(600_000)[2][0], IDLE)
        self.assertEqual(run.at(600_000)[3][0], HOLD)
        # all four: order 1, 2, 3, 4 at 0 / 25 / 50 / 75 ms
        run = self.sim(self.ticks(0, 200_000, [5000] * 4))
        for m in range(4):
            self.assertEqual(run.at(m * STAGGER_US)[2][m], IDLE)
            if m:
                self.assertEqual(run.at(m * STAGGER_US - TICK)[2][m], 0)

    def test_frame_rate_independent(self):
        fast = self.sim(self.ticks(0, 800_000, [7000] * 4, dt=1_000))
        slow = self.sim(self.ticks(0, 800_000, [7000] * 4, dt=20_000))
        for now in range(0, 800_000, 20_000):
            self.assertEqual(fast.at(now)[2], slow.at(now)[2], now)

    def test_disarm_reset_makes_every_motor_cold_at_once(self):
        # Disarm (not-ARMED frame), kill and load sessions: motors_off() -> ss_reset().
        script = self.ticks(0, 600_000, [6000] * 4) + [("R",)]
        script += self.ticks(605_000, 700_000, [6000] * 4)
        run = self.sim(script)
        self.assertEqual(run.at(595_000)[2], [6000] * 4)
        self.assertEqual(run.at(605_000)[2], [IDLE, 0, 0, 0])    # spin-up again, 5 ms later
        self.assertEqual(run.at(605_000)[3], [HOLD, WAIT, WAIT, WAIT])

    # -- the failsafe cut (ss_cut / ss_resume) ------------------------------------------------

    def cut_script(self, cut_us, resume_tgt=(9000, 5000, 9000, 5000), zero_after_resume=0):
        """Run at 5000 x4 until 600 ms, cut, zeros for cut_us, resume (U), then resume_tgt."""
        t_cut = 600_000
        t_res = t_cut + cut_us
        script = self.ticks(0, t_cut, [5000] * 4) + [("C", t_cut)]
        script += self.ticks(t_cut, t_res, [0] * 4)
        script += [("U", t_res)]
        script += self.ticks(t_res, t_res + zero_after_resume, [0] * 4)
        t_go = t_res + zero_after_resume
        script += self.ticks(t_go, t_go + 100_000, list(resume_tgt))
        return script, t_go

    def test_cut_120ms_then_resume_is_unlimited(self):
        script, t_go = self.cut_script(120_000)
        run = self.sim(script)
        self.assertEqual(run.at(715_000)[2], [0] * 4)                 # cut: outputs 0
        self.assertEqual(run.at(715_000)[3], [RUN] * 4)               # but still warm
        self.assertEqual(run.at(t_go)[2], [9000, 5000, 9000, 5000])   # no hold, no stagger
        self.assertEqual(run.at(t_go)[3], [RUN] * 4)
        self.assertEqual(run.rows[-1][4], run.at(595_000)[4])         # sslim unchanged

    def test_cut_500ms_then_resume_is_gated(self):
        script, t_go = self.cut_script(500_000)
        run = self.sim(script)
        self.assertEqual(run.at(895_000)[3], [RUN] * 4)               # warm until 300 ms
        self.assertEqual(run.at(905_000)[3], [COLD] * 4)              # cold from 300 ms on
        # in-flight recovery: hold + slew, but all four together (no stagger, S2)
        self.assertEqual(run.at(t_go)[2], [IDLE] * 4)
        self.assertEqual(run.at(t_go)[3], [HOLD] * 4)
        self.assertGreater(run.rows[-1][4], run.at(595_000)[4])         # sslim counts again

    def test_cut_500ms_resume_with_no_steps_during_the_cut_is_gated(self):
        # The cold rule is a function of time, not of how many service ticks ran.
        script = self.ticks(0, 600_000, [5000] * 4) + [("C", 600_000), ("U", 1_100_000),
                                                         (1_100_000, [5000] * 4)]
        run = self.sim(script)
        self.assertEqual(run.at(1_100_000)[2], [IDLE] * 4)
        # same without the explicit resume: a nonzero command reaching ss_step ends the cut
        script = self.ticks(0, 600_000, [5000] * 4) + [("C", 600_000), (1_100_000, [5000] * 4)]
        self.assertEqual(self.sim(script).at(1_100_000)[2], [IDLE] * 4)
        script = self.ticks(0, 600_000, [5000] * 4) + [("C", 600_000), (700_000, [5000] * 4)]
        self.assertEqual(self.sim(script).at(700_000)[2], [5000] * 4)

    def test_cut_just_under_and_at_the_cold_threshold(self):
        run = self.sim(self.cut_script(295_000)[0])
        self.assertEqual(run.at(895_000)[2], [9000, 5000, 9000, 5000])
        script, t_go = self.cut_script(300_000)
        self.assertEqual(self.sim(script).at(t_go)[2], [IDLE] * 4)

    def test_short_cut_then_zero_commands_then_drive_is_unlimited(self):
        # The link came back (resume) with zero duties for 400 ms: a commanded 0 (rule 3),
        # not a cut, so the cold timer does not apply.
        script, t_go = self.cut_script(120_000, zero_after_resume=400_000)
        run = self.sim(script)
        self.assertEqual(run.at(t_go - TICK)[3], [RUN] * 4)
        self.assertEqual(run.at(t_go)[2], [9000, 5000, 9000, 5000])

    def test_disarm_during_a_short_cut_is_cold(self):
        script = (self.ticks(0, 600_000, [5000] * 4) + [("C", 600_000)]
                  + self.ticks(600_000, 650_000, [0] * 4) + [("R",), ("U", 650_000)]
                  + self.ticks(650_000, 700_000, [5000] * 4))
        run = self.sim(script)
        self.assertEqual(run.at(650_000)[2], [IDLE, 0, 0, 0])

    def test_cut_during_spinup_keeps_the_timeline(self):
        script = (self.ticks(0, 150_000, [8000] * 4) + [("C", 150_000)]
                  + self.ticks(150_000, 250_000, [0] * 4) + [("U", 250_000)]
                  + self.ticks(250_000, 900_000, [8000] * 4))
        run = self.sim(script)
        env = IDLE + RATE * (250_000 - HOLD_US) // 1_000_000
        self.assertLessEqual(run.at(250_000)[2][0], env + 1)          # not a free step to 8000
        self.assertGreater(run.at(250_000)[2][0], IDLE)               # nor a fresh hold
        self.assertLessEqual(run.first_reach(0, 8000), MAX_US)

    def test_repeated_cut_keeps_the_first_start(self):
        # Link up again with zero duties (no U: e.g. a not-yet-drivable frame), second cut:
        # the rotors have been unpowered since the first one.
        script = (self.ticks(0, 600_000, [5000] * 4) + [("C", 600_000)]
                  + self.ticks(600_000, 800_000, [0] * 4) + [("C", 800_000)]
                  + self.ticks(800_000, 950_000, [0] * 4) + [("U", 950_000)]
                  + self.ticks(950_000, 1_000_000, [5000] * 4))
        self.assertEqual(self.sim(script).at(950_000)[2], [IDLE] * 4)

    def test_command_below_idle_passes_through_but_does_not_release(self):
        script = self.ticks(0, 200_000, [500] * 4) + self.ticks(200_000, 1_000_000, [6000] * 4)
        run = self.sim(script)
        self.assertEqual(run.at(0)[2], [500, 0, 0, 0])
        self.assertEqual(run.at(100_000)[2], [500] * 4)           # passes through unchanged
        self.assertEqual(run.at(100_000)[3][0], SLEW)             # still gated
        out = run.at(200_000)[2][0]
        env = IDLE + RATE * (200_000 - HOLD_US) // 1_000_000
        self.assertLess(out, 6000)
        self.assertLessEqual(out, env + 1)
        self.assertLessEqual(run.first_reach(0, 6000), MAX_US)
        # held below idle past SS_MAX: released, the next step is unlimited
        script = self.ticks(0, 800_000, [500] * 4) + [(800_000, [9000] * 4)]
        run = self.sim(script)
        self.assertEqual(run.at(800_000)[2], [9000] * 4)

    def test_armed_motor_at_zero_for_2s_restarts_unlimited(self):
        # Mixer saturation / throttle chop in flight: no rest timer re-limits a started motor.
        script = (self.ticks(0, 600_000, [5000] * 4) + self.ticks(600_000, 2_600_000, [0] * 4)
                  + self.ticks(2_600_000, 2_700_000, [9000, 5000, 9000, 5000]))
        run = self.sim(script)
        self.assertEqual(run.at(2_595_000)[3], [RUN] * 4)            # not COLD after 2 s at 0
        self.assertEqual(run.at(2_600_000)[2], [9000, 5000, 9000, 5000])   # no hold, no stagger
        lim = run.at(595_000)[4]
        self.assertEqual(run.rows[-1][4], lim)
        # a 50 ms blip is likewise unlimited
        blip = (self.ticks(0, 600_000, [5000] * 4) + self.ticks(600_000, 650_000, [0] * 4)
                + self.ticks(650_000, 700_000, [5000] * 4))
        self.assertEqual(self.sim(blip).at(650_000)[2], [5000] * 4)

    def test_reset_after_a_cold_cut_restores_the_stagger(self):
        # A long cut starts the next spin-up unstaggered; a disarm/kill (ss_reset) after it
        # brings the ground-start stagger back.
        script = (self.ticks(0, 600_000, [5000] * 4) + [("C", 600_000)]
                  + self.ticks(600_000, 1_000_000, [0] * 4) + [("U", 1_000_000)]
                  + self.ticks(1_000_000, 1_100_000, [5000] * 4) + [("R",)]
                  + self.ticks(1_100_000, 1_200_000, [5000] * 4))
        run = self.sim(script)
        self.assertEqual(run.at(1_000_000)[2], [IDLE] * 4)
        self.assertEqual(run.at(1_100_000)[2], [IDLE, 0, 0, 0])
        self.assertEqual(run.at(1_100_000)[3], [HOLD, WAIT, WAIT, WAIT])

    def test_inside_envelope_frame_does_not_release_the_gate(self):
        # S1: a frame that sits inside the envelope (here 0.11 at 90 ms, exactly idle at 80)
        # must not let the next frame step unlimited to 0.9.
        for blip in ((80_000, 1000), (90_000, 1100)):
            script = (self.ticks(0, blip[0], [9000, 0, 0, 0]) + [(blip[0], [blip[1], 0, 0, 0])]
                      + self.ticks(blip[0] + TICK, 400_000, [9000, 0, 0, 0]))
            run = self.sim(script)
            now = blip[0] + TICK
            env = IDLE + RATE * (now - HOLD_US) // 1_000_000
            self.assertLessEqual(run.at(now)[2][0], env + 1, blip)
            self.assertEqual(run.at(now)[3][0], SLEW, blip)

    def test_stale_timestamp_does_not_release_the_gate(self):
        # S3: a sample older than the cold start reads as 0 elapsed, not ~71 min.
        script = [(1_000_000, [5000] * 4), (999_999, [5000] * 4), (1_005_000, [5000] * 4)]
        run = self.sim(script)
        for _now, _tgt, out, ph, _lim in run.rows:
            self.assertNotIn(RUN, ph)
            self.assertLessEqual(max(out), IDLE)

    def test_after_reset_it_is_gated_again(self):
        script = (self.ticks(0, 600_000, [5000] * 4) + self.ticks(600_000, 2_600_000, [0] * 4)
                  + [("R",)] + self.ticks(2_600_000, 3_600_000, [5000] * 4))
        run = self.sim(script)
        self.assertEqual(run.at(2_600_000)[2], [IDLE, 0, 0, 0])
        self.assertEqual(run.at(2_600_000)[3], [HOLD, WAIT, WAIT, WAIT])
        self.assertLessEqual(run.first_reach(3, 5000, after=2_600_000) - 2_600_000, MAX_US)

    def test_never_started_motor_stays_cold_and_gets_its_own_spinup(self):
        # The rig: right pair starts at arm, left pair stays 0 for 2 s, then starts.
        script = (self.ticks(0, 2_000_000, [0, 500, 0, 500])
                  + self.ticks(2_000_000, 3_000_000, [6000, 6000, 6000, 6000]))
        run = self.sim(script)
        self.assertEqual(run.at(1_995_000)[3], [COLD, RUN, COLD, RUN])   # 700 ms bound released m2/m4
        start = run.at(2_000_000)
        self.assertEqual(start[2], [IDLE, 6000, 0, 6000])   # m1 rank 0, m3 rank 1; runners free
        self.assertEqual(start[3], [HOLD, RUN, WAIT, RUN])
        self.assertEqual(run.at(2_025_000)[2][2], IDLE)
        for m in (0, 2):
            reach = run.first_reach(m, 6000, after=2_000_000) - 2_000_000
            self.assertLessEqual(reach, MAX_US)
            self.assertGreater(reach, HOLD_US)
        self.assertEqual(run.rows[-1][3], [RUN] * 4)

    def test_zero_blip_during_spinup_keeps_the_envelope(self):
        script = (self.ticks(0, 100_000, [8000] * 4) + self.ticks(100_000, 120_000, [0] * 4)
                  + self.ticks(120_000, 900_000, [8000] * 4))
        run = self.sim(script)
        env = IDLE + RATE * (120_000 - HOLD_US) // 1_000_000
        self.assertLessEqual(run.at(120_000)[2][0], env + 1)      # not a free step to 8000
        self.assertLessEqual(run.first_reach(0, 8000), MAX_US)

    def test_u32_clock_wrap(self):
        start = 2**32 - 100_000
        run = self.sim(self.ticks(start, start + 900_000, [6000] * 4))
        reach = run.first_reach(0, 6000)
        self.assertAlmostEqual(reach - start, HOLD_US + 250_000, delta=TICK)
        self.assertEqual(run.rows[-1][2], [6000] * 4)

    # -- phase interleave -------------------------------------------------------------------

    def test_hpoint_never_overflows_and_packs(self):
        for P in (256, 2048, 4096):
            result = subprocess.run([str(self.exes[0]), "hpoint", str(P)], capture_output=True,
                                    text=True, timeout=120)
            self.assertEqual(result.returncode, 0, result.stderr)
            peaks = {}
            for row in result.stdout.splitlines():
                _, pct, _d, peak, _n = row.split()
                peaks[int(pct)] = int(peak)
            # <= 24 %: one FET on at a time; 30-49 %: two (the quarter-wrap ideal, where the
            # old per-channel spread had three on from ~40 %); 60 / 98 %: all four overlap
            # mid-period, unavoidable without wraparound
            self.assertEqual([peaks[p] for p in (10, 20, 24)], [1, 1, 1], P)
            self.assertEqual([peaks[p] for p in (30, 40, 45, 49)], [2, 2, 2, 2], P)
            self.assertEqual(peaks[60], 4)
            self.assertEqual(peaks[98], 4)

    def test_equal_duty_peak_is_the_no_wrap_optimum(self):
        # peak == ceil(4d / (P-1)) for every d <= (P-1)/2; above it the 4-on window is exactly
        # 2d - (P-1) counts (every pulse must cover [P-1-d, d))
        for P in (256, 2048):
            result = subprocess.run([str(self.exes[0]), "packpeak", str(P)],
                                    capture_output=True, text=True, timeout=300)
            self.assertEqual(result.returncode, 0, result.stderr)
            last = result.stdout.splitlines()[-1].split()
            self.assertEqual(last[0], "OK", result.stdout[-2000:])
            self.assertEqual(int(last[1]), P - 2)
            self.assertEqual(int(last[3]), 0, result.stdout[:2000])


if __name__ == "__main__":
    unittest.main()
