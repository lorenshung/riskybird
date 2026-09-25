"""rb fly and the flight presets: pure logic, no hardware."""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from rb import boards, fly  # noqa: E402


class FlightPresets(unittest.TestCase):
    def test_every_preset_file_is_a_flight_controller_mode(self):
        wl = boards.WORKLOADS["flight_controller"]
        presets = boards.flight_presets()
        self.assertGreaterEqual(len(presets), 4)
        for name in presets:
            self.assertIn(name, wl.modes)
            self.assertEqual(wl.layout_mode(name), "esp-motors")
            self.assertTrue(wl.mode_note(name), name)

    def test_preset_inherits_esp_motors_and_overrides_it(self):
        wl = boards.WORKLOADS["flight_controller"]
        defines = dict(wl.defines_for("t21-hover-1.0m"))
        # from esp-motors (the base)
        self.assertEqual(defines["ROSE_ESP_MOTORS"], "1")
        self.assertEqual(defines["ROSE_USE_PID"], "1")
        self.assertEqual(defines["CONFIG_UART_SIFIVE_PORT_1"], "y")
        # the preset's own values win over the base
        self.assertEqual(defines["PID_MASS_KG"], "0.050f")
        self.assertEqual(defines["ROSE_BATT_REPORT_ONLY"], "0")
        self.assertEqual(defines["HOVER_Z_M"], "1.00f")

    def test_preset_preprocessor_knobs_reach_cppflags_only(self):
        wl = boards.WORKLOADS["flight_controller"]
        cpp = dict(wl.cppflags_for("t27-hover-1.6m"))
        self.assertEqual(cpp["TOF_STEP_REJECT_M"], "0.25f")
        self.assertEqual(cpp["ALT_INT_BAND"], "0.10f")
        self.assertNotIn("TOF_STEP_REJECT_M", dict(wl.defines_for("t27-hover-1.6m")))
        # T16 predates the altitude band: the preset must not invent it
        self.assertNotIn("ALT_INT_BAND", dict(wl.cppflags_for("t16-hover-0.6m")))

    def test_base_modes_are_unchanged(self):
        wl = boards.WORKLOADS["flight_controller"]
        self.assertEqual(wl.layout_mode("esp-motors"), "esp-motors")
        self.assertEqual(wl.cppflags_for("esp-motors"), ())
        self.assertNotIn("ROSE_AUTOFLIGHT", dict(wl.defines_for("default")))

    def test_imu_test_mode_is_motor_incapable(self):
        wl = boards.WORKLOADS["flight_controller"]
        self.assertEqual(wl.layout_mode("imu-test"), "default")
        self.assertEqual(dict(wl.cppflags_for("imu-test"))["ROSE_IMU_DEBUG"], "1")
        self.assertNotIn("ROSE_ESP_MOTORS", dict(wl.defines_for("imu-test")))

    def test_parser_rejects_non_define_tokens(self):
        with tempfile.TemporaryDirectory() as d:
            bad = Path(d) / "bad.env"
            bad.write_text('export RB_CMAKE_ARGS="-DOK=1 --oops"\n')
            with self.assertRaises(ValueError):
                boards._parse_preset(bad)

    def test_esp_bridge_motors_mode_and_motor_duty(self):
        esp = boards.WORKLOADS["esp_bridge"]
        self.assertIn("motors", esp.modes)
        self.assertEqual(dict(esp.defines_for("motors"))["ESP_LINK_MAX_DUTY_PCT"], "98")
        self.assertNotIn("ESP_LINK_MAX_DUTY_PCT", dict(esp.defines_for("default")))
        self.assertIn("motor_duty", boards.WORKLOADS)


RB_LINES = [
    "RBLINK v1 ml=up up=15375 seq=58 fl=0x20[COMMITTED ] duty=[0 0 0 0] fr=729 gap=34 lost=0 stale=0 crc=0 noise=49700 cuts=2 pwmerr=0 kill=0 txt=49700 cl=1",
    "RBLINK v1 ml=up up=15576 seq=70 fl=0x20[COMMITTED ] duty=[0 0 0 0] fr=779 gap=25 lost=0 stale=0 crc=0 noise=50284 cuts=2 pwmerr=0 kill=0 txt=50284 cl=1",
    "RBLINK v1 ml=up up=15777 seq=81 fl=0x21[ARMED COMMITTED ] duty=[7500 7441 6650 6744] fr=829 gap=5 lost=0 stale=0 crc=0 noise=51092 cuts=2 pwmerr=0 kill=0 txt=51092 cl=1",
]


class RblinkParsing(unittest.TestCase):
    def test_parse_one_line(self):
        d = fly.parse_rblink(RB_LINES[2])
        self.assertEqual(d["ml"], "up")
        self.assertEqual(d["duty"], [7500, 7441, 6650, 6744])
        self.assertEqual(d["flags"], "ARMED COMMITTED")
        self.assertEqual(d["kill"], 0)
        self.assertIsNone(fly.parse_rblink("flight_controller: it=1"))

    def test_link_summary_rate_and_gap(self):
        s = fly.link_summary(RB_LINES)
        self.assertTrue(s["ok"])
        self.assertAlmostEqual(s["frames_per_s"], 100 * 1000 / 402, places=1)
        self.assertEqual(s["gap_max_ms"], 34)

    def test_link_summary_detects_a_reboot(self):
        rebooted = [RB_LINES[1], RB_LINES[0].replace("up=15375", "up=300")]
        self.assertFalse(fly.link_summary(rebooted)["ok"])


def _imu(a, g):
    return (0, list(a), list(g))


class ImuVerdicts(unittest.TestCase):
    """Signs as measured on the FPGA carrier 2026-09-24 (all six channels correct)."""

    def test_level(self):
        ok, _ = fly.imu_verdict("level", [_imu((-0.19, 0.23, 9.72), (0, 0, 0))] * 6)
        self.assertTrue(ok)

    def test_nose_up_needs_ax_positive_and_gy_negative(self):
        good = [_imu((2.6, 0.1, 9.3), (0.0, -1.15, 0.0))] * 6
        self.assertTrue(fly.imu_verdict("nose-up", good)[0])
        flipped = [_imu((-2.6, 0.1, 9.3), (0.0, 1.15, 0.0))] * 6
        self.assertFalse(fly.imu_verdict("nose-up", flipped)[0])

    def test_right_wing_down_and_nose_left(self):
        self.assertTrue(fly.imu_verdict("right-wing-down", [_imu((0, 2.9, 9.3), (0.66, 0, 0))] * 6)[0])
        self.assertTrue(fly.imu_verdict("nose-left", [_imu((0, 0, 9.7), (0, 0, 1.14))] * 6)[0])
        self.assertFalse(fly.imu_verdict("nose-left", [_imu((0, 0, 9.7), (0, 0, -1.14))] * 6)[0])

    def test_parses_the_debug_line(self):
        s = fly._imu_samples(["flight_controller: iter=6400 a=[1.039 0.107 9.617] g=[-0.019 -1.239 0.063]"])
        self.assertEqual(s[0][1], [1.039, 0.107, 9.617])


if __name__ == "__main__":
    unittest.main()
