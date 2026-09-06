"""Regression tests for reading achieved frequency out of Vivado reports.

Uses the real report text where it exists, because a guessed fixture label has
already hidden one parsing bug in this repository.
"""

from __future__ import annotations

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from tools.rb import area, timing
from tools.rb.boards import get_board


CLOCKS = """\
| Tool Version      : Vivado v.2023.1 (lin64)
| Date              : Fri Jul 24 09:28:01 2026
| Design State      : Physopt postRoute
Clock                       Period(ns)  Waveform(ns)    Attributes  Sources
JTCK                        100.000     {0.000 50.000}  P           {jtag_TCK}
ospi_pclk                   27.778      {0.000 13.889}  P           {ospi_pclk}
sys_clock                   5.000       {0.000 2.500}   P           {sys_clock_p}
clk_out1_harnessSysPLLNode  20.000      {0.000 10.000}  P,G,A       {mmcme3/CLKOUT0}
"""

TIMING = """\
| Date              : Fri Jul 24 09:27:53 2026
| Design State      : Physopt postRoute
Clock                             WNS(ns)      TNS(ns)  TNS Failing Endpoints  TNS Total Endpoints
JTCK                               46.268        0.000                      0                  560
ospi_pclk                          24.393        0.000                      0                 2019
  clk_out1_harnessSysPLLNode        5.619        0.000                      0                54771
"""


def _reports(tmp: Path, timing_text: str = TIMING, clocks_text: str = CLOCKS) -> Path:
    out = tmp / "report"
    out.mkdir(parents=True, exist_ok=True)
    (out / "timing.txt").write_text(timing_text)
    (out / "clocks.txt").write_text(clocks_text)
    return out


class ParseTests(unittest.TestCase):
    def test_the_design_clock_is_the_one_with_the_endpoints(self) -> None:
        """JTAG reports 46 ns of slack and decides nothing."""
        with TemporaryDirectory() as tmp:
            report = timing.read_reports(_reports(Path(tmp)), "C")
        design = report.design_clock
        self.assertEqual(design.name, "clk_out1_harnessSysPLLNode")
        self.assertEqual(design.endpoints, 54771)

    def test_achieved_frequency_comes_from_the_critical_path(self) -> None:
        with TemporaryDirectory() as tmp:
            report = timing.read_reports(_reports(Path(tmp)), "C")
        design = report.design_clock
        self.assertAlmostEqual(design.period_ns, 20.0)
        self.assertAlmostEqual(design.wns_ns, 5.619)
        self.assertAlmostEqual(design.critical_path_ns, 14.381, places=3)
        self.assertAlmostEqual(design.achieved_mhz, 69.54, places=1)
        self.assertAlmostEqual(design.declared_mhz, 50.0)
        self.assertAlmostEqual(design.headroom, 1.391, places=2)

    def test_timing_met_when_nothing_fails(self) -> None:
        with TemporaryDirectory() as tmp:
            self.assertTrue(timing.read_reports(_reports(Path(tmp)), "C").met)

    def test_a_negative_slack_is_reported_as_violated(self) -> None:
        bad = TIMING.replace("        5.619        0.000                      0",
                             "       -2.500     -100.000                     42")
        with TemporaryDirectory() as tmp:
            report = timing.read_reports(_reports(Path(tmp), timing_text=bad), "C")
        self.assertFalse(report.met)
        design = report.design_clock
        self.assertLess(design.wns_ns, 0)
        # Slower than declared, which is the point of reporting it.
        self.assertLess(design.achieved_mhz, design.declared_mhz)

    def test_design_state_and_date_are_captured(self) -> None:
        with TemporaryDirectory() as tmp:
            report = timing.read_reports(_reports(Path(tmp)), "C")
        self.assertEqual(report.design_state, "Physopt postRoute")
        self.assertIn("Jul 24", report.reported_at)

    def test_reports_older_than_the_synthesis_are_flagged(self) -> None:
        """A timing report on disk need not belong to the record beside it."""
        with TemporaryDirectory() as tmp:
            report = timing.read_reports(
                _reports(Path(tmp)), "C", synthesized_at="2026-07-27T22:57:53-0700"
            )
        self.assertTrue(report.stale_against_synthesis)

    def test_reports_newer_than_the_synthesis_are_not_flagged(self) -> None:
        with TemporaryDirectory() as tmp:
            report = timing.read_reports(
                _reports(Path(tmp)), "C", synthesized_at="2026-07-01T00:00:00-0700"
            )
        self.assertFalse(report.stale_against_synthesis)

    def test_a_missing_timing_report_says_why(self) -> None:
        with TemporaryDirectory() as tmp:
            with self.assertRaises(timing.TimingError) as caught:
                timing.read_reports(Path(tmp), "C")
        self.assertIn("synthesis only", str(caught.exception))

    def test_clocks_without_a_period_are_skipped_not_guessed(self) -> None:
        with TemporaryDirectory() as tmp:
            report = timing.read_reports(
                _reports(Path(tmp), clocks_text=CLOCKS.replace(
                    "ospi_pclk                   27.778", "")), "C")
        self.assertNotIn("ospi_pclk", [c.name for c in report.clocks])


class RealReportTests(unittest.TestCase):
    def test_the_ku040_rocket_config_closes_above_its_declared_clock(self) -> None:
        """Against the reports actually in the tree."""
        board = get_board("ku040").with_config("RocketKU040OspiConfig")
        if not (board.generated_dir / "obj" / "report" / "timing.txt").is_file():
            self.skipTest("no post-route reports for RocketKU040OspiConfig")
        report = timing.for_board(board)
        design = report.design_clock
        self.assertEqual(design.period_ns, 20.0)
        self.assertTrue(report.met)
        # Declared 50 MHz; the routed design has real headroom above it.
        self.assertGreater(design.achieved_mhz, 60.0)

    def test_accelerator_configs_have_no_timing_yet(self) -> None:
        """rb area is synthesis-only, so these carry no frequency at all."""
        for config in ("GemminiKU040Config", "Q31Ws32x32AccGemminiKU040Config"):
            board = get_board("ku040").with_config(config)
            with self.subTest(config=config):
                with self.assertRaises(timing.TimingError):
                    timing.for_board(board)


if __name__ == "__main__":
    unittest.main()
