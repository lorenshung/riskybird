"""rb fly and the flight presets: pure logic, no hardware."""

import argparse
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

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


def _adapter(location, manufacturer="FTDI", product="Dual RS232-HS"):
    return {"location": location, "pid": "6010", "manufacturer": manufacturer,
            "product": product, "serial": "(none)", "bus": "7", "device": "1", "ttys": ""}


BENCH = [_adapter("7-1.1.1", "Digilent", "Digilent Adept USB Device"),
         _adapter("7-1.1.4.1"),
         _adapter("7-1.1.4.2", "Digilent", "Digilent Adept USB Device"),
         _adapter("7-1.1.4.3"),
         _adapter("5-2", "Xilinx", "A-U250-A64G")]


def _args(**kw):
    base = dict(debug_location=None, bitstream_location=None, console="/dev/null", esp_port="x")
    base.update(kw)
    return argparse.Namespace(**base)


class Discovery(unittest.TestCase):
    """The probe detaches an adapter's JTAG, so it must never reach someone else's board."""

    def _discover(self, args, idcodes):
        from rb import cli
        probed = []

        def probe(location):
            probed.append(location)
            return idcodes.get(location, "all ones (nothing driving TDO)")

        with mock.patch.object(cli, "ftdi_adapters", return_value=BENCH), \
             mock.patch.object(cli, "probe_idcode", side_effect=probe), \
             mock.patch.object(fly, "_holders", return_value=[]):
            return fly.discover(args), probed

    def test_vendor_boards_are_never_probed(self):
        bench, probed = self._discover(_args(), {
            "7-1.1.4.1": "0x13636093 xc7a200t config TAP -- bitstream link",
            "7-1.1.4.3": "0x00000001 Rocket debug TAP"})
        self.assertEqual(sorted(probed), ["7-1.1.4.1", "7-1.1.4.3"])
        self.assertEqual(bench.bitstream["location"], "7-1.1.4.1")
        self.assertEqual(bench.debug["location"], "7-1.1.4.3")

    def test_halted_debug_adapter_resolves_to_the_twin_not_the_digilent_on_the_same_hub(self):
        bench, _ = self._discover(_args(), {"7-1.1.4.1": "0x13636093 xc7a200t config TAP -- bitstream link"})
        self.assertEqual(bench.debug["location"], "7-1.1.4.3")

    def test_both_locations_given_probes_nothing(self):
        bench, probed = self._discover(_args(debug_location="7-1.1.4.3", bitstream_location="7-1.1.4.1"), {})
        self.assertEqual(probed, [])
        self.assertEqual(bench.debug["location"], "7-1.1.4.3")

    def test_a_named_location_that_is_not_a_drone_adapter_is_refused(self):
        with self.assertRaises(fly.FlyError):
            self._discover(_args(debug_location="7-1.1.4.2", bitstream_location="7-1.1.4.1"), {})


class StopScript(unittest.TestCase):
    def _stop(self, last):
        calls = []
        with tempfile.TemporaryDirectory() as d, \
             mock.patch.object(fly, "LAST_MODE", Path(d) / "last-mode"), \
             mock.patch.object(fly, "discover", return_value=fly.Bench(None, {"location": "x"}, None, None)), \
             mock.patch.object(fly, "rb_debug", side_effect=lambda *a, **k: calls.append((a[3], k))):
            fly._remember_mode(*last)
            fly.gdb_step(argparse.Namespace(mode=None), fly.STOP)
        return calls[0]

    def test_flight_controller_gets_the_estop_script(self):
        script, kw = self._stop(("t21-hover-1.0m", "flight_controller"))
        self.assertEqual(script, fly.STOP)

    def test_an_image_without_g_estop_gets_a_plain_halt(self):
        # stop.gdb aborts on the missing g_estop before `monitor halt`
        script, kw = self._stop(("default", "motor_duty"))
        self.assertEqual(script, fly.HALT)
        self.assertEqual(kw["workload"], "motor_duty")
        self.assertIn("monitor halt", fly.HALT.read_text())
        self.assertNotIn("g_estop =", fly.HALT.read_text())

    def test_motor_step_length_is_bounded_by_the_firmware_dwell(self):
        args = argparse.Namespace(duty=20, ms=fly.MOTOR_DUTY_DWELL_MS + 1, yes=True)
        with self.assertRaises(fly.FlyError):
            fly.motors(args)


if __name__ == "__main__":
    unittest.main()
