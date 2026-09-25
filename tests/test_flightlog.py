"""Unit tests for the flight-log splitter, indexer and publisher (tools/rb/flightlog.py)."""

from __future__ import annotations

import csv
import datetime
import json
import subprocess
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from tools.rb import flightlog


BASE = 1790298533.0          # 2026-09-24, local-time folder names are computed from it
README = ("# RiskyBird flight logs\n\n## Index\n| trial | ARMING | outcome | data |\n|---|---|---|---|\n"
          "\n`index.csv` has the same list, one row per trial.\n\n## Formats\n")
INDEX = "trial,arming_time,armed_s,max_tilt_est_deg,max_tofh_m,estop_source,rblink,notes\n"


def fc_line(ts: float, st: int, z: float = 0.0, tofh: float = 0.002, zsp: float = 0.0,
            roll: float = 0.0, duty: tuple[float, ...] = (0, 0, 0, 0), x: float = 0.0,
            vbat: float = 4.0, tilt: int = 1) -> str:
    d = " ".join("%.3f" % v for v in duty)
    return ("%.3f flight_controller: it=%d dt=1ms roll=%.3f pitch=0.000 yaw=0.000 z=%.3f tofv=1 "
            "tofh=%.3f u=[0.4 0.4 0.4 0.4] duty=[%s] x=%.3f y=0.000 vx=0.100 vy=-0.050 vz=0.000 "
            "zsp=%.3f vbat=%.3f st=%d tilt=est%ddeg/acc0deg |a|=9.800 batt=meas\n"
            % (ts, int(ts * 1000) % 100000, roll, z, tofh, d, x, zsp, vbat, st, tilt))


def rb_line(ts: float, fl: str, kill: int, duty: tuple[int, ...] = (0, 0, 0, 0)) -> str:
    return ("%.3f RBLINK v1 ml=up up=1000 seq=1 fl=0x20[%s ] duty=[%s] fr=10 gap=4 lost=0 "
            "stale=0 crc=0 noise=5 cuts=2 pwmerr=0 kill=%d txt=5 cl=1\n"
            % (ts, fl, " ".join(str(v) for v in duty), kill))


def attempt(t_arming: float, seconds: float = 8.0, kill_at_estop: bool = True) -> list[str]:
    """One RESET press relative to t_arming: 1 s idle, ARMING 1 s, ARMED 2 s (airborne for
    1 s of it), then a dashboard ESTOP (ESP kill=1) and the latched aftermath."""
    out = []
    start = t_arming - 1.0
    for k in range(int(seconds * 25)):
        t = start + k * 0.04
        rel = t - t_arming
        if rel < 0:
            out.append(fc_line(t, 8))
        elif rel < 1.0:
            out.append(fc_line(t, 12))
        elif rel < 3.0:
            flying = 1.5 <= rel < 2.5
            out.append(fc_line(t, 9, z=0.30 if flying else 0.0, tofh=0.30 if flying else 0.002,
                               zsp=0.30, roll=0.05 if flying else 0.0, x=rel,
                               duty=(0.7, 0.6, 0.7, 0.6), vbat=3.6, tilt=6))
        else:
            out.append(fc_line(t, 11))
        if k % 5 == 0:
            if rel < 1.0:
                out.append(rb_line(t + 0.001, "COMMITTED", 0))
            elif rel < 2.9:
                out.append(rb_line(t + 0.001, "ARMED COMMITTED", 0, (7000, 6000, 7000, 6000)))
            else:
                out.append(rb_line(t + 0.001, "COMMITTED ESTOP", 1 if kill_at_estop else 0))
    return out


def make_repo(root: Path) -> Path:
    repo = root / "flight-logs"
    (repo / "trials").mkdir(parents=True)
    (repo / "README.md").write_text(README)
    (repo / "index.csv").write_text(INDEX)
    return repo


def hhmmss(ts: float) -> str:
    return datetime.datetime.fromtimestamp(ts).strftime("%H%M%S")


class Fixture(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = TemporaryDirectory()
        self.root = Path(self._tmp.name)
        self.repo = make_repo(self.root)
        self.raw_dir = self.root / "results"
        self.raw_dir.mkdir()
        self.raw = self.raw_dir / "raw_20260924_180000.log"
        self.first = attempt(BASE + 2.0)
        self.second = attempt(BASE + 30.0)
        self.raw.write_text("".join(self.first))

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def grow(self) -> None:
        """The same session log after a second RESET press was flown and logged."""
        self.raw.write_text("".join(self.first + self.second))


class SplitTests(Fixture):
    def test_split_creates_trial_with_files_and_arming_time(self) -> None:
        out = flightlog.split(self.raw, "2026-09-24", self.repo)
        self.assertEqual(len(out), 1)
        trial, created = out[0]
        self.assertTrue(created)
        self.assertEqual(trial.name, "2026-09-24_T01_%s" % hhmmss(BASE + 2.0))
        for name in ("raw.log", "fc.csv", "rblink.csv", "metrics.json"):
            self.assertTrue((trial / name).is_file(), name)
        m = json.loads((trial / "metrics.json").read_text())
        arming = datetime.datetime.fromisoformat(m["arming_host_time"]).timestamp()
        self.assertAlmostEqual(arming, BASE + 2.0, places=2)
        self.assertEqual(m["trial"], trial.name)
        self.assertEqual(m["source_log"], self.raw.name)
        self.assertEqual(m["arming_to_armed_s"], 1.0)
        self.assertEqual(m["estop_source_hint"], "ESP kill latch (dashboard ESTOP)")
        self.assertEqual(m["max_tofh_armed_m"], 0.3)
        with open(trial / "fc.csv", newline="") as fh:
            rows = list(csv.DictReader(fh))
        self.assertEqual(list(rows[0]), flightlog.FC_COLS)
        self.assertEqual(rows[0]["dt"], "")            # "dt=1ms" never parsed, kept for compatibility
        self.assertEqual(min(float(r["t"]) for r in rows), -1.0)
        self.assertTrue(any(r["t"] == "0.0" and r["st"] == "12.0" for r in rows))

    def test_resplit_of_grown_log_keeps_numbers_and_summary(self) -> None:
        # First push: the log was copied mid-session, a few seconds after the ESTOP.
        self.raw.write_text("".join(self.first[: len(self.first) * 7 // 10]))
        (t1, _), = flightlog.split(self.raw, "2026-09-24", self.repo)
        (t1 / "summary.md").write_text("hand notes\n")
        before = (t1 / "raw.log").read_text()
        self.grow()
        out = flightlog.split(self.raw, "2026-09-24", self.repo)
        self.assertEqual([(p.name, c) for p, c in out], [
            (t1.name, False),
            ("2026-09-24_T02_%s" % hhmmss(BASE + 30.0), True),
        ])
        self.assertEqual((t1 / "summary.md").read_text(), "hand notes\n")
        self.assertGreater(len((t1 / "raw.log").read_text()), len(before))   # grew into the aftermath
        self.assertEqual(flightlog.next_trial_number(self.repo), 3)

    def test_first_number_and_partial_last_line(self) -> None:
        self.raw.write_text("".join(self.first) + "%.3f flight_controller: it=1 st=" % (BASE + 9))
        out = flightlog.split(self.raw, "2026-09-24", self.repo, first=41)
        self.assertEqual(out[0][0].name[:14], "2026-09-24_T41")
        self.assertNotIn("it=1 st=\n", (out[0][0] / "raw.log").read_text())

    def test_next_trial_number_and_resolve(self) -> None:
        self.assertEqual(flightlog.next_trial_number(self.repo), 1)
        (trial, _), = flightlog.split(self.raw, "2026-09-24", self.repo)
        (self.repo / "trials" / "2026-09-23_T07_101010").mkdir()
        self.assertEqual(flightlog.next_trial_number(self.repo), 8)
        self.assertEqual(flightlog.resolve_trial(self.repo, "T1"), trial)
        self.assertEqual(flightlog.resolve_trial(self.repo, "1"), trial)
        self.assertEqual(flightlog.resolve_trial(self.repo, trial.name), trial)
        with self.assertRaises(flightlog.FlightLogError):
            flightlog.resolve_trial(self.repo, "T9")
        (self.repo / "trials" / "2026-09-25_T07_111111").mkdir()
        with self.assertRaisesRegex(flightlog.FlightLogError, "ambiguous"):
            flightlog.resolve_trial(self.repo, "T07")

    def test_latest_raw(self) -> None:
        self.assertEqual(flightlog.latest_raw(self.raw_dir), self.raw)
        with self.assertRaises(flightlog.FlightLogError):
            flightlog.latest_raw(self.root / "trials-nowhere")


class SummarizeTests(Fixture):
    def test_summarize_is_idempotent_on_readme_and_index(self) -> None:
        (trial, _), = flightlog.split(self.raw, "2026-09-24", self.repo)
        line = flightlog.summarize(trial, self.repo, build="B1", tag="zsp 0.3", note="ok")
        again = flightlog.summarize(trial, self.repo, build="B1", tag="zsp 0.3", note="ok")
        self.assertEqual(line, again)
        self.assertTrue(line.startswith("airborne 1.0 s"), line)
        self.assertIn("ESTOP: ESP kill latch (dashboard ESTOP)", line)
        readme = (self.repo / "README.md").read_text()
        self.assertEqual(readme.count("trials/%s/" % trial.name), 1)
        self.assertIn("| [%s](trials/%s/summary.md) | %s | **zsp 0.3**. operator: ok. airborne 1.0 s"
                      % (trial.name, trial.name, hhmmss(BASE + 2.0)[:2] + ":"
                         + hhmmss(BASE + 2.0)[2:4] + ":" + hhmmss(BASE + 2.0)[4:]), readme)
        self.assertLess(readme.index(trial.name), readme.index("`index.csv` has the same list"))
        index = (self.repo / "index.csv").read_text().splitlines()
        self.assertEqual(len(index), 2)
        self.assertTrue(index[1].startswith(trial.name + ","))
        self.assertTrue(index[1].endswith(",yes,zsp 0.3; operator: ok; airborne 1.0 s (t 1.52..2.48)"))
        summary = (trial / "summary.md").read_text()
        self.assertIn('Operator: "ok".', summary)
        self.assertIn("## Build (reported by the controller session)\nB1\n", summary)

    def test_never_airborne_line(self) -> None:
        lines = [l.replace("tofh=0.300", "tofh=0.010") for l in self.first]
        self.raw.write_text("".join(lines))
        (trial, _), = flightlog.split(self.raw, "2026-09-24", self.repo)
        line = flightlog.summarize(trial, self.repo)
        self.assertTrue(line.startswith("never airborne (tofh never > 2 cm while armed); armed 2.0 s"), line)
        self.assertIn("not airborne", flightlog.compare([trial]))


class FakeGit:
    def __init__(self, status: str = " M trials/x\n", fail: str | None = None) -> None:
        self.calls: list[list[str]] = []
        self.status = status
        self.fail = fail

    def __call__(self, cmd: list[str], **kwargs: object) -> subprocess.CompletedProcess:
        self.calls.append(cmd[3:])
        if self.fail and cmd[3] == self.fail:
            return subprocess.CompletedProcess(cmd, 1, "", "fatal: rejected\n")
        out = self.status if cmd[3] == "status" else ""
        return subprocess.CompletedProcess(cmd, 0, out, "")


class PushTests(Fixture):
    def test_dry_run_copies_raw_and_skips_pull_commit_push(self) -> None:
        git = FakeGit()
        new = flightlog.push(self.repo, raw=self.raw, dry_run=True, tag="t", run=git)
        self.assertEqual([p.name[:14] for p in new], ["2026-09-24_T01"])
        self.assertEqual(git.calls, [["status", "--porcelain"]])
        copy = self.repo / "sessions" / "2026-09-24" / "raw" / self.raw.name
        self.assertEqual(copy.read_text(), self.raw.read_text())
        self.assertTrue((new[0] / "summary.md").is_file())

    def test_push_commits_new_trials_then_refreshes(self) -> None:
        git = FakeGit()
        new = flightlog.push(self.repo, raw=self.raw, run=git)
        self.assertEqual(git.calls, [
            ["pull", "--ff-only"], ["status", "--porcelain"], ["add", "-A"],
            ["commit", "-m", "Flight logs 2026-09-24: T01"], ["push"],
        ])
        (new[0] / "summary.md").write_text("hand notes\n")
        self.grow()
        self.raw.write_text(self.raw.read_text() + "".join(attempt(BASE + 60.0)))
        git = FakeGit()
        new2 = flightlog.push(self.repo, raw=self.raw, run=git)
        self.assertEqual([_n(p) for p in new2], [2, 3])
        self.assertIn(["commit", "-m", "Flight logs 2026-09-24: T02-T03"], git.calls)
        self.assertEqual((new[0] / "summary.md").read_text(), "hand notes\n")
        readme = (self.repo / "README.md").read_text()
        self.assertEqual(readme.count("trials/%s/" % new[0].name), 1)
        git = FakeGit()
        self.assertEqual(flightlog.push(self.repo, raw=self.raw, run=git), [])
        self.assertIn(["commit", "-m", "Flight logs 2026-09-24: refresh"], git.calls)

    def test_nothing_changed_and_git_failure(self) -> None:
        git = FakeGit(status="")
        self.assertEqual(flightlog.push(self.repo, raw=self.raw, run=git), [])
        self.assertEqual(git.calls[-1], ["status", "--porcelain"])
        with self.assertRaisesRegex(flightlog.FlightLogError, "fatal: rejected"):
            flightlog.push(self.repo, raw=self.raw, run=FakeGit(fail="push"))
        with self.assertRaisesRegex(flightlog.FlightLogError, "pull"):
            flightlog.push(self.repo, raw=self.raw, run=FakeGit(fail="pull"))


def _n(p: Path) -> int:
    return int(p.name.split("_T")[1].split("_")[0])


class ReadingTests(Fixture):
    def test_show_and_compare(self) -> None:
        self.grow()
        trials = [p for p, _ in flightlog.split(self.raw, "2026-09-24", self.repo)]
        text = flightlog.show(trials[0])
        lines = text.splitlines()
        self.assertIn(trials[0].name, lines[0])
        self.assertIn("t 0.70..4.00", lines[0])       # 0.3 s before ARMED .. 1 s after ESTOP
        self.assertIn("esp fl", lines[1])
        self.assertTrue(any("ARMED COMMITTED" in l and "0.300" in l for l in lines))
        self.assertTrue(any("COMMITTED ESTOP" in l and l.rstrip().endswith("1") for l in lines))
        roll = "%+.1f" % flightlog._deg(0.05)          # Gibbs 0.05 -> 5.7 deg, not 2.9
        self.assertEqual(roll, "+5.7")
        self.assertTrue(any(roll in l for l in lines))
        narrow = flightlog.show(trials[0], step=0.5, t_from=0.0, t_to=1.0).splitlines()
        self.assertEqual(len(narrow), 3 + 3)     # title, header, rule, t 0.0/0.5/1.0

        table = flightlog.compare(trials)
        rows = table.splitlines()
        self.assertIn("airborne", rows[0])
        self.assertIn("yaw diff", rows[0])
        t1 = next(r for r in rows if r.startswith("T01"))
        for want in ("1.0 s", "0.30", "+5.7", "0.20", "4.00->3.60", "ESP kill latch (dashboard ESTOP)"):
            self.assertIn(want, t1)
        self.assertTrue(any(r.startswith("T02") for r in rows))


if __name__ == "__main__":
    unittest.main()
