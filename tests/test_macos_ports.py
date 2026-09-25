"""macOS serial-node discovery: the names measured on the drone bench Mac, and failing closed."""

from __future__ import annotations

import unittest
from pathlib import Path
from unittest import mock

from tools.rb import artifacts, cli, fly


class LocationPrefixTests(unittest.TestCase):
    def test_measured_bench_names(self) -> None:
        # ioreg locationID 0x01130000 / 0x02100000 / 0x00100000 on the bench Mac.
        self.assertEqual(artifacts.macos_location_prefix(1, (1, 3)), "113")
        self.assertEqual(artifacts.macos_location_prefix(2, (1,)), "21")
        self.assertEqual(artifacts.macos_location_prefix(0, (1,)), "1")

    def test_deeper_chain(self) -> None:
        self.assertEqual(artifacts.macos_location_prefix(1, (1, 3, 2)), "1132")


class SerialNodeTests(unittest.TestCase):
    def test_blank_eeprom_ft2232h_channels(self) -> None:
        present = {"/dev/cu.usbserial-11300", "/dev/cu.usbserial-11301"}
        node = lambda i: cli.macos_serial_node(1, (1, 3), i, "", exists=present.__contains__)
        self.assertEqual(node(0), "/dev/cu.usbserial-11300")
        self.assertEqual(node(1), "/dev/cu.usbserial-11301")

    def test_serial_number_scheme_wins_when_present(self) -> None:
        present = {"/dev/cu.usbserial-FT1234B", "/dev/cu.usbserial-11301"}
        self.assertEqual(
            cli.macos_serial_node(1, (1, 3), 1, "FT1234", exists=present.__contains__),
            "/dev/cu.usbserial-FT1234B")

    def test_missing_node_fails_closed(self) -> None:
        self.assertIsNone(cli.macos_serial_node(1, (1, 3), 1, "", exists=lambda p: False))

    def test_unknown_tty_is_not_a_dev_path(self) -> None:
        # "?" must never become "/dev/?" -- that passes startswith("/dev/") and gets opened.
        adapter = {"ttys": "if0=? if1=?"}
        self.assertEqual(cli._tty_for(adapter, 1), "-")
        adapter = {"ttys": "if0=cu.usbserial-2100 if1=cu.usbserial-2101"}
        self.assertEqual(cli._tty_for(adapter, 1), "/dev/cu.usbserial-2101")


class EspSelectionTests(unittest.TestCase):
    def test_esp_chosen_by_usb_vendor_when_names_say_nothing(self) -> None:
        devices = [Path("/dev/cu.usbmodem101"), Path("/dev/cu.usbserial-11300"),
                   Path("/dev/cu.usbserial-11301")]
        with mock.patch.dict("os.environ", {}, clear=False), \
             mock.patch.object(artifacts, "serial_devices", return_value=devices), \
             mock.patch.object(artifacts, "macos_espressif_ports",
                               return_value=[Path("/dev/cu.usbmodem101")]):
            import os
            os.environ.pop("RB_ESP_PORT", None)
            self.assertEqual(artifacts.select_serial_port("esp"), Path("/dev/cu.usbmodem101"))

    def test_ambiguous_without_espressif_still_refuses(self) -> None:
        devices = [Path("/dev/cu.usbmodem101"), Path("/dev/cu.usbmodem201")]
        with mock.patch.object(artifacts, "serial_devices", return_value=devices), \
             mock.patch.object(artifacts, "macos_espressif_ports", return_value=[]):
            import os
            os.environ.pop("RB_ESP_PORT", None)
            with self.assertRaises(RuntimeError):
                artifacts.select_serial_port("esp")


class HoldersTests(unittest.TestCase):
    def test_lsof_output_parsed(self) -> None:
        out = "p4242\ncpython3\np99999\ncscreen\n"
        with mock.patch.object(fly.subprocess, "run",
                               return_value=mock.Mock(stdout=out)):
            self.assertEqual(fly._holders_lsof("/dev/cu.usbserial-11301"),
                             ["pid 4242: python3", "pid 99999: screen"])


if __name__ == "__main__":
    unittest.main()
