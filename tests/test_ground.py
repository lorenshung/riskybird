"""Unit tests for the ground-station status line and the gonogo exit code."""

from __future__ import annotations

import json
import os
import time
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from tools.rb import ground


GO = {
    "t": "2026-09-24T17:12:37", "verdict": "GO", "link": "ok", "fc_hz": 23.3, "rb_hz": 5.0,
    "ml": "up", "fl": "COMMITTED", "kill": 0.0, "cl": 1.0, "tilt": 3, "tilt_acc": 3,
    "tofv": 1.0, "tofh": 0.001, "z": 0.0, "st": 8.0, "gap_max": 7.0, "cuts": 2.0,
    "vbat": 4.156, "reasons": [], "warnings": ["tofh=0.001"],
}


class FormatTests(unittest.TestCase):
    def test_go_line_has_checklist_fields_and_integer_flags(self) -> None:
        line = ground.format_status(GO, age_s=0.5)
        self.assertIn("GO", line)
        self.assertIn("fl=[COMMITTED]", line)
        self.assertIn("kill=0", line)       # 0.0 printed as an integer flag
        self.assertIn("st=8", line)
        self.assertIn("warn:  tofh=0.001", line)
        self.assertNotIn("STALE", line)

    def test_no_go_lists_reasons_and_missing_fields_print_dash(self) -> None:
        s = {"t": "2026-09-24T18:13:17", "verdict": "NO-GO", "link": "silent", "fc_hz": 0.0,
             "reasons": ["FC lines 0.0 Hz", "commit WINDOW-EXPIRED (re-run flight.sh commit)"]}
        line = ground.format_status(s, age_s=1.0)
        self.assertIn("NO-GO: FC lines 0.0 Hz; commit WINDOW-EXPIRED", line)
        self.assertIn("tilt=-/-", line)

    def test_old_status_is_flagged_stale(self) -> None:
        self.assertIn("STALE", ground.format_status(GO, age_s=60))


class GonogoTests(unittest.TestCase):
    def _write(self, d: Path, status: dict, age_s: float = 0.0) -> Path:
        p = d / "ready-status.json"
        p.write_text(json.dumps(status), encoding="utf-8")
        if age_s:
            t = time.time() - age_s
            os.utime(p, (t, t))
        return p

    def test_exit_zero_only_on_fresh_go(self) -> None:
        with TemporaryDirectory() as tmp:
            d = Path(tmp)
            self.assertEqual(ground.gonogo(path=self._write(d, GO)), 0)
            self.assertEqual(ground.gonogo(path=self._write(d, GO, age_s=60)), 1)
            self.assertEqual(ground.gonogo(path=self._write(d, dict(GO, verdict="NO-GO"))), 1)

    def test_missing_status_is_an_operator_error(self) -> None:
        with TemporaryDirectory() as tmp:
            with self.assertRaises(ground.GroundError):
                ground.gonogo(path=Path(tmp) / "nope.json")


if __name__ == "__main__":
    unittest.main()
