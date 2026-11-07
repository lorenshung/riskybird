"""Randomized stress tests for the ESP motor spin-up gate and the PWM phase interleave.

Run with ``python3 -m unittest tests.test_motor_softstart_fuzz -v`` (or pytest).

Builds tests/motor_softstart_fuzz_check.c against
workloads/lib/motor_link/include/riskybird/motor_softstart.h, as C and as C++, and runs it once
per scenario. The driver does all the work -- a fixed-seed PRNG, long random command/cut/reset
sequences, a shadow model of the header's documented rules, and one violation counter per
invariant -- so a run is deterministic. Each test below asserts one invariant (counter == 0)
and, on failure, prints the driver's first counterexample with the events that led to it.

Knobs: RB_FUZZ_SEED (default 0x5EED) and RB_FUZZ_SCALE (iteration multiplier, default 1).
No Zephyr and no LEDC hardware: the register-sequence test models the write order of
pwm_led_esp32.c + main.c motor_phase_apply(); the order itself is pinned against the sources.
"""

from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
INC = ROOT / "workloads/lib/motor_link/include"
DRIVER = Path(__file__).resolve().parent / "motor_softstart_fuzz_check.c"
MAIN_C = ROOT / "workloads/esp_bridge/src/main.c"
LEDC_DRIVER = ROOT / "backends/zephyr-chipyard-sw/zephyr_ws/zephyr/drivers/pwm/pwm_led_esp32.c"
WARN = ["-Wall", "-Wextra", "-Werror"]

SEED = int(os.environ.get("RB_FUZZ_SEED", "0x5EED"), 0)
SCALE = max(1, int(os.environ.get("RB_FUZZ_SCALE", "1")))
GATE_STEPS = 2_000_000 * SCALE
HPOINT_VECTORS = 400_000 * SCALE
REG_UPDATES = 2_000_000 * SCALE


def _build(compiler, out, name, lang):
    exe = out / name
    cmd = [compiler, *(["-x", "c++", "-std=c++17"] if lang == "c++" else ["-std=c11"]),
           "-O2", *WARN, f"-I{INC}", str(DRIVER), "-o", str(exe)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise AssertionError(f"{' '.join(cmd)}\n{result.stderr}")
    return exe


class Result:
    def __init__(self, stdout):
        self.stdout = stdout
        self.viol, self.first, self.stat = {}, {}, {}
        for line in stdout.splitlines():
            kind, _, rest = line.partition(" ")
            name, _, value = rest.partition(" ")
            if kind == "VIOL":
                self.viol[name] = int(value)
            elif kind == "FIRST":
                self.first[name] = value
            elif kind == "STAT":
                self.stat[name] = int(value)


def _run(exe, *args):
    result = subprocess.run([str(exe), *map(str, args)], capture_output=True, text=True,
                            timeout=300)
    if result.returncode != 0:
        raise AssertionError(f"{exe} {' '.join(map(str, args))} -> {result.returncode}\n"
                             f"{result.stderr}")
    return Result(result.stdout)


class _Fuzz(unittest.TestCase):
    _tmp = None
    runs = {}

    @classmethod
    def setUpClass(cls):
        cc = shutil.which("gcc") or shutil.which("cc")
        cxx = shutil.which("g++") or shutil.which("c++")
        if cc is None or cxx is None:
            raise unittest.SkipTest("host C/C++ compiler unavailable")
        cls._tmp = tempfile.TemporaryDirectory(prefix="motor-softstart-fuzz-")
        out = Path(cls._tmp.name)
        cls.exe_c = _build(cc, out, "fuzz_c", "c")
        cls.exe_cxx = _build(cxx, out, "fuzz_cxx", "c++")
        cls.runs = cls.make_runs()

    @classmethod
    def tearDownClass(cls):
        if cls._tmp is not None:
            cls._tmp.cleanup()

    @classmethod
    def make_runs(cls):
        return {}

    def clean(self, run_name, invariant):
        run = self.runs[run_name]
        self.assertIn(invariant, run.viol, f"driver did not report {invariant}")
        self.assertEqual(run.viol[invariant], 0,
                         f"[{run_name}, seed {SEED:#x}] {invariant}: {run.viol[invariant]} "
                         f"violations; first: {run.first.get(invariant)}")


class SpinupGateFuzz(_Fuzz):
    """ss_step/ss_cut/ss_resume/ss_reset against a shadow model of the documented rules."""

    @classmethod
    def make_runs(cls):
        return {mode: _run(cls.exe_c, "gate", SEED, GATE_STEPS, mode)
                for mode in ("mono", "quant", "back")}

    # -- safety invariants that must hold in every scenario, incl. time going backwards ------

    def test_rule1_output_never_exceeds_command(self):
        for mode in ("mono", "quant", "back"):
            self.clean(mode, "rule1_out_le_cmd")

    def test_zero_command_is_zero_output(self):
        for mode in ("mono", "quant", "back"):
            self.clean(mode, "zero_cmd_zero_out")

    def test_lim_ticks_counts_exactly_the_limited_steps(self):
        for mode in ("mono", "quant", "back"):
            self.clean(mode, "lim_ticks")

    def test_a_running_motor_passes_through_incl_warm_resume(self):
        # RUN before and after the step -> out == cmd. Covers the < SS_CUT_COLD_MS resume.
        for mode in ("mono", "quant", "back"):
            self.clean(mode, "run_passthrough")

    # -- the documented rules, monotonic time -----------------------------------------------

    def test_decreases_are_instant(self):
        for mode in ("mono", "quant"):
            self.clean(mode, "decrease_instant")

    def test_spinup_output_is_min_of_command_and_envelope(self):
        # WAIT/HOLD/SLEW: out == min(cmd, env(now - t_start - rank * stagger)), phase matches
        for mode in ("mono", "quant"):
            self.clean(mode, "spinup_exact")
        for mode in ("mono", "quant", "back"):
            self.clean(mode, "envelope_other")

    def test_spinup_is_bounded_by_max(self):
        for mode in ("mono", "quant"):
            self.clean(mode, "rule5_bounded")

    def test_cold_only_at_boot_reset_or_long_cut(self):
        # COLD <=> boot, ss_reset(), or a cut of >= 300 ms counted from the FIRST ss_cut.
        for mode in ("mono", "quant"):
            self.clean(mode, "cold_only_documented")

    # -- the strict envelope: no unlimited step within SS_MAX of a cold start ----------------

    def test_no_unlimited_step_within_max_of_a_cold_start(self):
        """FAILS (finding S1): in SLEW, a single command with IDLE <= cmd <= env releases the
        motor to RUN (motor_softstart.h:266), and the very next frame steps unlimited -- e.g.
        1000 -> 9999 at 90 ms after a cold start. The task's invariant allows only the
        warm-resume exception."""
        for mode in ("mono", "quant"):
            self.clean(mode, "envelope_after_reach_release")

    def test_time_going_backwards_never_releases_the_gate(self):
        """FAILS (finding S3): a timestamp 1 us older than t_start makes
        (uint32_t)(now - t_start) wrap to ~2^32 >= SS_MAX_US (motor_softstart.h:239/251), so
        rule 5 releases the motor to RUN and it steps straight to its command."""
        self.clean("back", "envelope_after_rule5_release")

    # -- the fuzz is not vacuous ---------------------------------------------------------------

    def test_fuzz_coverage(self):
        st = self.runs["mono"].stat
        for key, floor in (("steps", GATE_STEPS), ("cold_starts", 1000), ("rank3", 100),
                           ("cut_299", 50), ("cut_300", 50), ("cut_301", 50),
                           ("cut_lt299", 50), ("cut_long", 50), ("cut_repeated", 20),
                           ("warm_resumes", 100), ("cold_resumes", 100),
                           ("direct_resumes", 20), ("resets", 100), ("wraps", 10),
                           ("big_gaps", 20), ("max_releases", 50), ("reach_releases", 50),
                           ("zero_blip_spinup", 100)):
            self.assertGreaterEqual(st.get(key, 0), floor, key)
        self.assertGreater(self.runs["quant"].stat["cut_300"], 50)

    def test_c_and_cxx_builds_agree(self):
        cxx = _run(self.exe_cxx, "gate", SEED, GATE_STEPS // 4, "mono")
        c = _run(self.exe_c, "gate", SEED, GATE_STEPS // 4, "mono")
        self.assertEqual(c.stdout, cxx.stdout)


class InterleaveFuzz(_Fuzz):
    """ss_phase_hpoints() (packing) with an independent random duty per channel, P in
    {256..4096}."""

    @classmethod
    def make_runs(cls):
        return {"hpoint": _run(cls.exe_c, "hpoint", SEED, HPOINT_VECTORS)}

    def test_lpoint_never_reaches_the_period(self):
        # hpoint + duty <= P - 1 for every channel and every duty 0..P (exhaustive per P)
        self.clean("hpoint", "hpoint_lpoint_le_P_minus_1")
        self.clean("hpoint", "hpoint_in_range")

    def test_pair_overlap_never_exceeds_in_phase(self):
        self.clean("hpoint", "pair_overlap_le_inphase")

    def test_peak_never_exceeds_in_phase(self):
        self.clean("hpoint", "peak_le_inphase")

    def test_excess_on_time_never_exceeds_in_phase(self):
        self.clean("hpoint", "excess_on_le_inphase")

    def test_equal_duties_strictly_reduce_overlap(self):
        self.clean("hpoint", "equal_duty_overlap_strictly_reduced")

    def test_flight_case_all_four_running_is_reduced(self):
        st = self.runs["hpoint"].stat
        self.assertGreater(st["all4_flight_cases"], 1000)
        self.assertEqual(st["all4_flight_not_reduced"], 0)

    def test_unequal_duties_strictly_reduce_overlap(self):
        """Finding P1 (2026-11-05): the old per-channel spread let a short pulse land wholly
        inside a long one. Packing places all four from all four duties (a pulse whose
        partner is idle is edge-aligned), so with two or more channels that have room the
        summed pair overlap is strictly below in-phase."""
        self.clean("hpoint", "random_duty_overlap_strictly_reduced")


class LedcRegisterSequence(_Fuzz):
    """The LEDC writes of pwm_led_esp32_set_cycles() + motor_phase_apply_all(), with a timer
    overflow allowed between any two writes: no latched (hpoint, duty) may overflow."""

    @classmethod
    def make_runs(cls):
        return {"regs": _run(cls.exe_c, "regs", SEED, REG_UPDATES),
                "duty-first": _run(cls.exe_c, "regs", SEED, REG_UPDATES // 4, "duty-first")}

    def test_latched_at_para_up(self):
        self.clean("regs", "regs_latched_at_para_up")

    def test_live_registers(self):
        self.clean("regs", "regs_live")

    def test_checker_catches_a_reordered_driver(self):
        # Not vacuous: had the driver written DUTY before hpoint 0, the old hpoint would run
        # with the new duty and the model would see Lpoint >= P.
        run = self.runs["duty-first"]
        self.assertGreater(run.viol["regs_latched_at_para_up"], 0)
        self.assertGreater(run.viol["regs_live"], 0)

    def test_driver_writes_hpoint_zero_before_duty(self):
        # The safety of the sequence rests on this order; pin it against the sources.
        if not LEDC_DRIVER.exists():
            self.skipTest("zephyr backend not checked out")
        src = LEDC_DRIVER.read_text()
        body = src[src.index("static void pwm_led_esp32_duty_set("):]
        body = body[:body.index("\n}\n")]
        self.assertLess(body.index("ledc_hal_set_hpoint("), body.index("ledc_hal_set_duty_int_part("))
        main = MAIN_C.read_text()
        apply = main[main.index("static void motor_phase_apply_all(void)"):]
        apply = apply[:apply.index("\n}\n")]
        # every hpoint computed from the DUTY already in each register (stopped channels as
        # idle), all four placed together, then hpoint, duty_start + para_up per channel
        self.assertRegex(apply, r"hw->channel_group\[LEDC_LOW_SPEED_MODE\]\.channel\[ch\]"
                                r"\.duty\.duty >> 4")
        self.assertLess(apply.index("sig_out_en"), apply.index("ss_phase_hpoints(d, period, h)"))
        self.assertLess(apply.index("ss_phase_hpoints(d, period, h)"),
                        apply.index("ledc_ll_set_hpoint("))
        self.assertLess(apply.index("ledc_ll_set_hpoint("), apply.index("ledc_ll_ls_channel_update("))
        apply_call = main[main.index("static void motors_apply("):]
        apply_call = apply_call[:apply_call.index("\n}\n")]
        self.assertLess(apply_call.index("rc = pwm_set_dt("), apply_call.index("motor_phase_apply_all()"))
        self.assertTrue(re.search(r"\} else \{\s*g_written\[i\] = \(uint16_t\)d;", apply_call))


if __name__ == "__main__":
    unittest.main()
