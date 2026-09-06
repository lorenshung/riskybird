"""Unit tests for the RoSE run contract and artifact renderer."""

from __future__ import annotations

import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from tools.rb import rose


class TimingTests(unittest.TestCase):
    def test_twenty_ms_grant_scales_with_clock(self) -> None:
        self.assertEqual(rose.step_cycles(1000, 20), 20_000_000)
        self.assertEqual(rose.step_cycles(100, 20), 2_000_000)
        self.assertEqual(rose.step_cycles(10, 20), 200_000)

    def test_invalid_clock_is_rejected(self) -> None:
        with self.assertRaises(rose.RoseError):
            rose.step_cycles(0, 20)


class ArtifactTests(unittest.TestCase):
    def test_smoke_summary_and_svg_are_derived_from_trace(self) -> None:
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            trace = root / "trace.jsonl"
            events = []
            for step in range(1, 6):
                events.extend(
                    [
                        {"event": "physics_step", "step": step,
                         "sim_time_s": step * 0.02, "pos": [0, 0, step / 5],
                         "z_error_m": step / 5 - 1,
                         "action": [-0.1, 0.1, -0.05, 0.05]},
                        {"event": "bridge_packet", "cmd": 0x12},
                        {"event": "bridge_packet", "cmd": 0x20},
                    ]
                )
            trace.write_text("".join(json.dumps(event) + "\n" for event in events))
            guest = root / "guest.log"
            guest.write_text(
                "RB_MPC_ROSE ready states=12 actions=4 horizon=10\n"
                "RB_MPC_ROSE iter=1 status=0 z_milli=-800 solve_ns=1000\n"
            )
            summary = rose.summarize(trace, guest, 5, 20, 0, False)
            self.assertEqual(summary["verdict"], "smoke-pass")
            self.assertEqual(summary["state_requests"], 5)
            self.assertEqual(summary["control_updates"], 5)
            self.assertEqual(summary["nominal_control_bound_violation_steps"], 0)
            plot = rose.render_plot(trace, summary, root / "plot.svg")
            self.assertIn("RoSE TinyMPC", plot.read_text())


if __name__ == "__main__":
    unittest.main()
