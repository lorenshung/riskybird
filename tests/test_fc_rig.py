"""Host checks for the roll-harness ("rig") mode logic in the flight controller.

Run with ``python3 -m unittest discover -s tests -p test_fc_rig.py -v``.
Builds workloads/flight_controller/src/fc_rig.cpp with the host g++, together with the C++
driver tests/fc_rig_check.cpp and the firmware's own controller_pid.cpp (ROSE_VEL_LOOP=0), and
runs one driver group per test. Covered: the strict number parser and printer, the gain
whitelist and ranges, every uplink verb, the arm/ramp/run/ramp-down state machine and its
guards, the excitation shapes, and the roll law against the flown PID. No Zephyr SDK or
hardware: this does not check main.cpp's hooks, timing or the motors.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "workloads/flight_controller/src"
DRIVER = Path(__file__).resolve().parent / "fc_rig_check.cpp"

# The second build moves every default the rig derives from the build, the way a flight preset
# does, to prove the defaults follow the macros instead of hard-coding the stock numbers.
# RIG_TGT_MAX_RAD=0.27f is the lower roll-target bound a stand with ~15.6 deg stops would preset.
TUNED = ["-DATT_GAIN=0.85f", "-DTAU_ROLL=0.12f", "-DTAU_ROLLRATE=0.03f",
         "-DMOTOR_TRIM_LEFT=1.0f", "-DPID_MASS_KG=0.050f", "-DRIG_TGT_MAX_RAD=0.27f"]
WARN = ["-Wall", "-Wextra", "-Werror"]


def _build(compiler, out_dir, name, defines):
    """Compile fc_rig.cpp (with the target's -Wdouble-promotion), then link the driver."""
    exe = out_dir / name
    obj = out_dir / f"{name}-fc_rig.o"
    common = ["-std=c++17", "-O1", *WARN, "-DROSE_VEL_LOOP=0", *defines, f"-I{SRC}"]
    steps = [
        [compiler, *common, "-Wdouble-promotion", "-c", str(SRC / "fc_rig.cpp"), "-o", str(obj)],
        [compiler, *common, str(DRIVER), str(obj), str(SRC / "controller_pid.cpp"),
         "-o", str(exe), "-lm"],
    ]
    for cmd in steps:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            raise AssertionError(f"{' '.join(cmd)}\n{result.stderr}")
    return exe


class FcRig(unittest.TestCase):
    exe = None
    exe_tuned = None
    _tmp = None

    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("g++") or shutil.which("c++")
        if compiler is None:
            raise unittest.SkipTest("host C++ compiler (g++) unavailable")
        cls._tmp = tempfile.TemporaryDirectory(prefix="fc-rig-test-")
        out = Path(cls._tmp.name)
        cls.exe = _build(compiler, out, "check", [])
        cls.exe_tuned = _build(compiler, out, "check_tuned", TUNED)

    @classmethod
    def tearDownClass(cls):
        if cls._tmp is not None:
            cls._tmp.cleanup()

    def group(self, name, exe=None):
        result = subprocess.run([str(exe or self.exe), name], capture_output=True, text=True,
                                timeout=60)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn(f"ok {name}", result.stdout)
        return result.stdout

    # numbers and gains
    def test_parser(self):
        self.group("parser")

    def test_format_round_trip(self):
        self.group("format")

    def test_gain_whitelist_ranges_and_defaults(self):
        out = self.group("gains")
        self.assertIn("defaults ATT_P=5 RATE_P=40 ATT_GAIN=1 MOTOR_TRIM_LEFT=0.87", out)

    def test_gain_defaults_follow_build_macros(self):
        out = self.group("gains", self.exe_tuned)
        self.assertIn("defaults ATT_P=4.16667 RATE_P=33.3333 ATT_GAIN=0.85 MOTOR_TRIM_LEFT=1", out)

    def test_header_is_c(self):
        cc = shutil.which("gcc") or shutil.which("cc")
        if cc is None:
            self.skipTest("host C compiler unavailable")
        result = subprocess.run([cc, "-std=c11", *WARN, "-fsyntax-only", "-x", "c",
                                 str(SRC / "fc_rig.h")], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    # uplink verbs
    def test_commands(self):
        self.group("commands")

    def test_exc_commands(self):
        self.group("exc_commands")
        self.group("exc_commands", self.exe_tuned)   # moment cap at a different mass

    # state machine
    def test_normal_sequence_and_arm_refusals(self):
        self.group("sm_normal")

    def test_arm_on_stop_guard_waits_for_liftoff(self):
        self.group("sm_stop_liftoff")

    def test_liftoff_timeout_ramps_down(self):
        self.group("sm_liftoff_timeout")

    def test_heartbeat_ramp_then_estop(self):
        self.group("sm_heartbeat")

    def test_pitch_guard(self):
        self.group("sm_pitch_guard")

    def test_estop_reset_rearm_disarm(self):
        self.group("sm_reset_rearm")

    def test_max_run_ms(self):
        self.group("sm_max_run")

    def test_rampdown_onto_stop_is_not_estop(self):
        self.group("sm_rampdown_on_stop")

    def test_setpoint_and_collective_slew(self):
        self.group("sm_slews")

    # excitation
    def test_excitation_shapes(self):
        self.group("exc_shapes")

    # control law
    def test_law_matches_flown_formula(self):
        for exe in (self.exe, self.exe_tuned):
            out = self.group("law_matches_flown", exe)
            self.assertIn("worst relative error vs flown formula", out)

    def test_law_matches_controller_pid_end_to_end(self):
        for exe in (self.exe, self.exe_tuned):
            self.group("law_matches_controller_pid", exe)

    def test_law_integrators_and_antiwindup(self):
        self.group("law_integrators")

    def test_law_input_from_shared_state(self):
        self.group("law_input")

    def test_law_filters_and_excitation_targets(self):
        self.group("law_filters_and_exc")

    # safety-review fixes
    def test_ramp_target_latched_at_arm(self):
        """M1: a COLL during RAMP_UP does not step the collective; RAMP_UP -> RUN is continuous."""
        for exe in (self.exe, self.exe_tuned):
            self.group("ramp_latch", exe)

    def test_roll_target_bounded(self):
        """M2: roll_tgt clamped to RIG_TGT_MAX_RAD; RSP/ROLL_TRIM/EXC att over it are NAKed."""
        self.assertIn("RIG_TGT_MAX_RAD=0.35", self.group("tgt_bound"))
        self.assertIn("RIG_TGT_MAX_RAD=0.27", self.group("tgt_bound", self.exe_tuned))

    def test_nan_inf_inputs(self):
        """M3: NaN/Inf on roll, pitch or gyro estops while armed, never arms, never latches."""
        self.group("nan")

    def test_integration_and_exc_wait_for_liftoff(self):
        self.group("liftoff_gate")

    def test_reset_then_stop_same_tick(self):
        self.group("reset_stop")

    def test_bumpless_gains_and_trim_slew(self):
        for exe in (self.exe, self.exe_tuned):
            self.group("bumpless", exe)

    def test_lift_assist(self):
        """Before lift-off the angle integrator builds at LIFT_I (moment-clamped, anti-windup on the
        clamp), is kept at lift-off, unwinds on a swing past level; LIFT_I 0 is the old law."""
        for exe in (self.exe, self.exe_tuned):
            self.group("lift_assist", exe)

    def test_rate_exc_cap_scales_with_collective(self):
        for exe in (self.exe, self.exe_tuned):
            self.group("rate_cap", exe)


if __name__ == "__main__":
    unittest.main()
