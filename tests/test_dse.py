"""Regression tests for the join between the two halves.

A design point is only worth having if it cannot quietly mix a measured number
with a borrowed one, so most of these pin the verdict and the frontier rather
than the plumbing.
"""

from __future__ import annotations

import json
import unittest
from dataclasses import dataclass, field
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any

from tools.rb import dse, plotting


@dataclass
class _Record:
    """Stand-in for an AreaRecord, carrying only what build_point reads."""

    run_id: str = "run-1"
    board: str = "ku040"
    config: str = "GemminiKU040Config"
    attrs: str = "none"
    evidence: str = "synthesized"
    flow: str = "synth-only"
    device: dict = field(default_factory=lambda: {"part": "xcku040-sfva784-1-c"})
    totals_are_upper_bound: bool = False
    fingerprint: dict | None = field(default_factory=lambda: {"identity_id": "abc123"})
    totals: dict = field(default_factory=lambda: {
        "luts": {"used": 224263, "available": 242400},
        "dsps": {"used": 228, "available": 1920},
    })


def _summary(makespan=57.0, latency=5.0, deadline_missed=False, agreement="exact"):
    return {
        "makespan": makespan,
        "makespan_cycles": int(makespan * 1000),
        "networks": {
            "dronet": {
                "worst_latency": 93.0 if deadline_missed else latency,
                "worst_latency_cycles": 1000,
                "instances": 1,
            },
        },
        "timing": {"clock_mhz": 50.0, "clock_source": "DTS", "capture_clock_mhz": 1000.0},
        "fingerprint": {"agreement": agreement, "divergences": []},
        "profile_hw": {"CPU_P": "gemmini_q31"},
        "machines": ["CPU_P#0"],
        "schedule": "/tmp/s.json",
    }


def _point(**kwargs):
    defaults = dict(
        board="ku040", config="C", attrs="none", mix="flight2",
        area_record=_Record(), summary=_summary(), deadlines={"dronet": 50.0},
    )
    defaults.update(kwargs)
    return dse.build_point(**defaults)


class VerdictTests(unittest.TestCase):
    def test_a_clean_point_is_ok(self) -> None:
        self.assertEqual(_point().verdict, "ok")

    def test_a_borrowed_profile_is_never_ok(self) -> None:
        """Extrapolated latency looks authoritative; the verdict must say it isn't."""
        point = _point(summary=_summary(agreement="extrapolated"))
        self.assertEqual(point.verdict, "extrapolated")

    def test_a_missed_deadline_outranks_extrapolation(self) -> None:
        point = _point(summary=_summary(deadline_missed=True, agreement="extrapolated"))
        self.assertEqual(point.verdict, "misses-deadline")
        self.assertEqual(point.missed, ["dronet"])

    def test_not_fitting_outranks_everything(self) -> None:
        record = _Record(totals={"luts": {"used": 300000, "available": 242400}})
        point = _point(area_record=record, summary=_summary(deadline_missed=True))
        self.assertEqual(point.verdict, "infeasible")
        self.assertFalse(point.fits)

    def test_deadline_comparison_uses_the_declared_deadline(self) -> None:
        self.assertEqual(_point(deadlines={"dronet": 200.0},
                                summary=_summary(deadline_missed=True)).missed, [])


class RecordTests(unittest.TestCase):
    def test_the_area_record_is_referenced_not_duplicated(self) -> None:
        """Deleting every point must lose nothing recorded elsewhere."""
        point = _point()
        self.assertEqual(point.resource["area_run_id"], "run-1")
        self.assertIn("results/area/run-1", point.provenance["area_record"])

    def test_utilization_percentages_are_computed(self) -> None:
        point = _point()
        self.assertAlmostEqual(
            point.resource["utilization"]["luts"]["util_pct"], 92.52, places=1
        )
        self.assertAlmostEqual(point.worst_utilization, 92.52, places=1)

    def test_identity_id_is_the_join_key(self) -> None:
        self.assertEqual(_point().identity_id, "abc123")

    def test_clock_and_its_source_survive_into_the_point(self) -> None:
        point = _point()
        self.assertEqual(point.performance["clock_mhz"], 50.0)
        self.assertEqual(point.performance["capture_clock_mhz"], 1000.0)

    def test_round_trips_through_disk(self) -> None:
        with TemporaryDirectory() as tmp:
            point = _point()
            path = dse.write_point(point, Path(tmp))
            loaded = dse.load_points(Path(tmp))
            self.assertEqual(json.loads(path.read_text())["schema"], dse.SCHEMA)
        self.assertEqual(len(loaded), 1)
        self.assertEqual(loaded[0].point_id, point.point_id)
        self.assertEqual(loaded[0].verdict, point.verdict)

    def test_a_foreign_schema_is_refused(self) -> None:
        with TemporaryDirectory() as tmp:
            out = Path(tmp) / "p"
            out.mkdir()
            (out / "point.json").write_text('{"schema": "something.else", "point_id": "x"}')
            with self.assertRaises(dse.DseError):
                dse.load_points(Path(tmp))


class ParetoTests(unittest.TestCase):
    def _at(self, luts, makespan, missed=False, available=242400):
        record = _Record(totals={"luts": {"used": luts, "available": available}})
        return _point(
            area_record=record,
            summary=_summary(makespan=makespan, deadline_missed=missed),
        )

    def test_a_point_beaten_on_both_axes_is_dropped(self) -> None:
        small_fast = self._at(100_000, 50.0)
        big_slow = self._at(200_000, 90.0)
        frontier = dse.pareto([small_fast, big_slow])
        self.assertEqual([p.resource_value("luts") for p in frontier], [100_000])

    def test_a_genuine_tradeoff_keeps_both(self) -> None:
        small_slow = self._at(100_000, 90.0)
        big_fast = self._at(200_000, 50.0)
        frontier = dse.pareto([small_slow, big_fast])
        self.assertEqual(len(frontier), 2)

    def test_infeasible_points_are_excluded_entirely(self) -> None:
        """A configuration that does not fit is not a trade-off."""
        over = self._at(300_000, 1.0)
        fits = self._at(100_000, 50.0)
        frontier = dse.pareto([over, fits])
        self.assertEqual([p.resource_value("luts") for p in frontier], [100_000])

    def test_the_frontier_is_ordered_by_resource(self) -> None:
        points = [self._at(200_000, 50.0), self._at(100_000, 90.0)]
        frontier = dse.pareto(points)
        self.assertEqual(
            [p.resource_value("luts") for p in frontier], [100_000, 200_000]
        )

    def test_points_missing_either_axis_are_skipped(self) -> None:
        blank = _point(area_record=_Record(totals={}))
        self.assertEqual(dse.pareto([blank]), [])


class FigureTests(unittest.TestCase):
    def _points(self):
        return [
            _point(config="A", area_record=_Record(
                config="A", totals={"luts": {"used": 100_000, "available": 242400},
                                    "dsps": {"used": 200, "available": 1920}})),
            _point(config="B", area_record=_Record(
                config="B", totals={"luts": {"used": 200_000, "available": 242400},
                                    "dsps": {"used": 900, "available": 1920}}),
                summary=_summary(makespan=30.0, agreement="extrapolated")),
        ]

    def test_tradeoff_chart_renders_valid_svg(self) -> None:
        svg = plotting.tradeoff_chart(self._points())
        self.assertTrue(svg.startswith("<svg"))
        self.assertTrue(svg.rstrip().endswith("</svg>"))

    def test_tradeoff_chart_states_the_clock(self) -> None:
        """A latency in milliseconds without its frequency is not a result."""
        self.assertIn("50 MHz", plotting.tradeoff_chart(self._points()))

    def test_tradeoff_chart_marks_device_capacity(self) -> None:
        self.assertIn("device capacity", plotting.tradeoff_chart(self._points()))

    def test_extrapolated_points_are_visibly_distinct(self) -> None:
        svg = plotting.tradeoff_chart(self._points())
        self.assertIn(plotting.VERDICT_COLORS["extrapolated"], svg)
        self.assertIn("extrapolated", svg)

    def test_deadline_chart_shows_the_deadline_as_the_track(self) -> None:
        points = [_point(summary=_summary(deadline_missed=True))]
        svg = plotting.deadline_chart(points)
        self.assertIn("MISS", svg)
        self.assertIn("deadline", svg)

    def test_deadline_chart_says_the_number_is_not_measured(self) -> None:
        svg = plotting.deadline_chart(self._points())
        self.assertIn("Not a measurement on hardware", svg)

    def test_empty_input_raises_rather_than_drawing_nothing(self) -> None:
        with self.assertRaises(plotting.PlotError):
            plotting.tradeoff_chart([])

    def test_no_mark_is_drawn_outside_the_plot_area(self) -> None:
        """The axis has to reach the largest value, capacity included.

        It used to stop below it, and because geometry is scaled by the last
        tick, the longest KU040 bars rendered ~110px past their frame.
        """
        import re
        svg = plotting.tradeoff_chart(self._points())
        # Data marks carry a white stroke; legend swatches do not.
        marks = re.findall(
            r'<circle cx="(-?[\d.]+)" cy="(-?[\d.]+)"[^>]*stroke="#ffffff"', svg
        )
        self.assertEqual(len(marks), 2, marks)
        for x, y in ((float(a), float(b)) for a, b in marks):
            self.assertTrue(78 <= x <= 78 + 560, f"x={x} outside plot")
            self.assertTrue(84 <= y <= 84 + 360, f"y={y} outside plot")


    def test_write_dse_figures_produces_files(self) -> None:
        with TemporaryDirectory() as tmp:
            written = plotting.write_dse_figures(self._points(), Path(tmp))
            self.assertTrue(written)
            for path in written:
                with self.subTest(path=path.name):
                    self.assertTrue(path.is_file())
                    self.assertTrue(path.read_text().startswith("<svg"))


class TickTests(unittest.TestCase):
    def test_ticks_always_reach_the_maximum(self) -> None:
        for maximum in (242_400, 224_263, 1_920, 100, 57.024, 1):
            with self.subTest(maximum=maximum):
                self.assertGreaterEqual(plotting._nice_ticks(maximum)[-1], maximum)

    def test_ticks_start_at_zero_and_ascend(self) -> None:
        ticks = plotting._nice_ticks(242_400)
        self.assertEqual(ticks[0], 0.0)
        self.assertEqual(ticks, sorted(ticks))

    def test_a_bar_never_exceeds_the_axis(self) -> None:
        """KU040 LUT capacity against its largest measured design."""
        ticks = plotting._nice_ticks(max(242_400, 224_263))
        self.assertGreaterEqual(ticks[-1], 242_400)


if __name__ == "__main__":
    unittest.main()
