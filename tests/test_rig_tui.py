"""Unit tests for tools/rb/rig_tui.py: parsers, batch files, ESTOP, rasteriser, rendering.

None of these need a TTY except the pty smoke test, which skips itself without one. Nothing
here sends to the real drone: every socket is 127.0.0.1.
"""

from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import threading
import time
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from tools.rb import rig_tui as R

ROOT = Path(__file__).resolve().parents[1]

RIG_OK = ("RIG it=1234 st=9 rs=RUN r=-1250 p=40 gx=-12 sp=0 tgt=3500 coll=950 exc=140 ex=3 "
          "gs=7 d=[6100 6000 5000 4900] vb=7730")


def selftest_app(tmp, t_end=23.0, listen=False):
    t0 = 1_800_000_000.0
    w = R.SelftestWorld(tmp, t0, listen=listen)
    w.step(t0 + t_end)
    app = R.App(drone="127.0.0.1", port=w.cmd_port or 9, batch_dir=w.batch_dir,
                selftest=w if listen else None)
    ing = R.Ingestor(app, log=R.LogFollower(path=w.log_path, backlog=1 << 30),
                     batch_dir=w.batch_dir)
    for _ in range(30):
        ing.poll_once(now=w.t)
    app.drain_notices()
    return w, app, ing


def close_all(w, app, ing):
    w.close()
    app.commander.close()
    ing.log.close()


class RigLineTests(unittest.TestCase):
    def test_telemetry(self) -> None:
        d = R.parse_rig_line(RIG_OK)
        self.assertEqual(d["kind"], "rig")
        self.assertEqual((d["it"], d["st"], d["rs"], d["r"], d["tgt"]), (1234, 9, "RUN", -1250, 3500))
        self.assertEqual(d["d"], [6100, 6000, 5000, 4900])
        self.assertEqual((d["ex"], d["gs"], d["vb"], d["coll"]), (3, 7, 7730, 950))

    def test_uart_rx_trailing_fields_tolerated(self) -> None:
        # Rig firmware appends the uart1 RX-ring overruns and dropped uplink lines.
        d = R.parse_rig_line(RIG_OK + " rxo=0 rxd=2")
        self.assertEqual(d, R.parse_rig_line(RIG_OK))

    def test_leading_junk_and_missing_optional_fields(self) -> None:
        d = R.parse_rig_line("\x00garbage>RIG it=1 st=0 rs=IDLE r=5")
        self.assertEqual((d["kind"], d["rs"], d["r"]), ("rig", "IDLE", 5))
        self.assertIsNone(d["d"])
        self.assertIsNone(d["vb"])

    def test_malformed_telemetry(self) -> None:
        for bad in ("RIG it=1 st=9 rs=RUN",                     # no r=
                    "RIG it=x st=9 rs=RUN r=1",                 # non-int
                    "RIG it=1 st=9 rs=RUN r=1 d=[1 2 3",        # truncated duty
                    "RIG it=1 st=9 rs=RUN r=1 d=[1 2 3]",       # 3 duties
                    "RIG it=1 st=9 rs=R-UN r=1",
                    "RIGX it=1 st=9 rs=RUN r=1",
                    "BRIG it=1 st=9 rs=RUN r=1",
                    "", "   ", "flight_controller: it=3 dt=4ms roll=0.000"):
            self.assertIsNone(R.parse_rig_line(bad), bad)

    def test_gain_lines_do_not_collide(self) -> None:
        g = R.parse_rig_line("RIGGAIN ATT_P=8.5")
        self.assertEqual(g, {"kind": "gain", "name": "ATT_P", "value": 8.5})
        gs = R.parse_rig_line("RIGGAINS gs=12 ATT_P=5 RATE_P=40 ROLL_TRIM=-0.01")
        self.assertEqual(gs["kind"], "gains")
        self.assertEqual(gs["gs"], 12)
        self.assertEqual(gs["gains"], {"ATT_P": 5.0, "RATE_P": 40.0, "ROLL_TRIM": -0.01})
        for bad in ("RIGGAIN ATT_P", "RIGGAIN ATT_P=abc", "RIGGAIN =3", "RIGGAINS",
                    "RIGGAINS gs=1 ATT_P=nan!"):
            self.assertIsNone(R.parse_rig_line(bad), bad)

    def test_event_and_acks(self) -> None:
        self.assertEqual(R.parse_rig_line("RIG EVENT estop: tilt 25.6deg"),
                         {"kind": "event", "text": "estop: tilt 25.6deg"})
        self.assertIsNone(R.parse_rig_line("RIG EVENT   "))
        self.assertEqual(R.parse_rig_line("FCACK GAIN")["kind"], "ack")
        n = R.parse_rig_line("FCNAK EXC out of range")
        self.assertEqual((n["kind"], n["verb"]), ("nak", "EXC"))
        self.assertIsNone(R.parse_rig_line("FCACK"))

    def test_split_log_line(self) -> None:
        self.assertEqual(R.split_log_line("1800000000.123 RIG it=1\n"), (1800000000.123, "RIG it=1"))
        self.assertEqual(R.split_log_line("RIG it=1", 5.0), (5.0, "RIG it=1"))
        self.assertEqual(R.split_log_line("12 RIG"), (None, "12 RIG"))

    def test_rblink(self) -> None:
        line = ("RBLINK v1 ml=up up=41000 seq=3 fl=0x21[ARMED COMMITTED ] duty=[1 2 3 4] "
                "fr=10 gap=20 lost=1 stale=0 crc=2 noise=0 cuts=0 pwmerr=0 kill=1 txt=9 cl=1")
        d = R.parse_rblink("1800000000.000 " + line)
        self.assertEqual((d["ml"], d["gap_ms"], d["lost"], d["crc"], d["kill"]), ("up", 20, 1, 2, True))
        self.assertEqual(d["duty"], [1, 2, 3, 4])
        self.assertIsNone(R.parse_rblink(line.replace("crc=2", "crc=x")))
        self.assertIsNone(R.parse_rblink(""))
        self.assertIsNone(R.parse_rblink(None))


class JournalTests(unittest.TestCase):
    def test_truncated_last_line(self) -> None:
        with TemporaryDirectory() as tmp:
            p = Path(tmp) / "journal.jsonl"
            p.write_text('{"t":1,"ev":"batch_start"}\n{"t":2,"ev":"trial_start","trial":1}\n'
                         '{"t":3,"ev":"trial_e', encoding="utf-8")
            evs, bad = R.load_journal(p)
            self.assertEqual([e["ev"] for e in evs], ["batch_start", "trial_start"])
            self.assertEqual(bad, 0)

    def test_bad_middle_line_counted(self) -> None:
        evs, bad = R.parse_journal_text('{"t":1,"ev":"a"}\nnot json\n[1,2]\n{"t":2,"ev":"b"}\n')
        self.assertEqual([e["ev"] for e in evs], ["a", "b"])
        self.assertEqual(bad, 2)

    def test_missing_journal(self) -> None:
        self.assertEqual(R.load_journal("/nonexistent/journal.jsonl"), ([], 0))

    def test_follower_holds_back_partial_line(self) -> None:
        with TemporaryDirectory() as tmp:
            p = Path(tmp) / "journal.jsonl"
            f = R.JsonlFollower(p)
            self.assertEqual(f.poll(), [])
            with open(p, "a") as fh:
                fh.write('{"t":1,"ev":"a"}\n{"t":2,"ev"')
            self.assertEqual([e["ev"] for e in f.poll()], ["a"])
            with open(p, "a") as fh:
                fh.write(':"b"}\n')
            self.assertEqual([e["ev"] for e in f.poll()], ["b"])
            self.assertEqual(f.bad, 0)

    def test_append_after_truncated_line_starts_fresh(self) -> None:
        with TemporaryDirectory() as tmp:
            p = Path(tmp) / "journal.jsonl"
            p.write_text('{"t":1,"ev":"a"}\n{"t":2,"ev":', encoding="utf-8")
            R.append_journal(p, {"t": 3, "ev": "estop", "source": "tui"})
            evs, bad = R.load_journal(p)
            self.assertEqual([e["ev"] for e in evs], ["a", "estop"])
            self.assertEqual(bad, 1)            # the dead writer's line, isolated


class ControlTests(unittest.TestCase):
    def test_pause_and_abort_are_independent_and_seq_grows(self) -> None:
        with TemporaryDirectory() as tmp:
            self.assertEqual(R.read_control(tmp), {"pause": False, "abort": False, "seq": 0})
            c1 = R.update_control(tmp, pause=True)
            c2 = R.update_control(tmp, abort=True, own_seq=0)
            self.assertEqual(c2, {"pause": True, "abort": True, "seq": c1["seq"] + 1})
            c3 = R.update_control(tmp, pause=False, own_seq=10)
            self.assertEqual(c3, {"pause": False, "abort": True, "seq": 11})
            self.assertEqual(json.loads((Path(tmp) / "control.json").read_text()), c3)
            self.assertEqual(sorted(os.listdir(tmp)), ["control.json"])   # no temp leftovers

    def test_atomic_against_concurrent_reader(self) -> None:
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "control.json"
            R.update_control(tmp, pause=False)
            errors, reads = [], [0]
            done = threading.Event()

            def reader():
                while not done.is_set():
                    try:
                        with open(path) as f:
                            obj = json.load(f)
                        assert set(obj) == {"pause", "abort", "seq"}
                        reads[0] += 1
                    except FileNotFoundError as exc:
                        errors.append(exc)
                    except ValueError as exc:
                        errors.append(exc)

            th = threading.Thread(target=reader)
            th.start()
            try:
                for i in range(150):
                    R.update_control(tmp, pause=bool(i % 2))
            finally:
                done.set()
                th.join()
            self.assertEqual(errors, [])
            self.assertGreater(reads[0], 0)
            self.assertEqual(R.read_control(tmp)["seq"], 151)


class EstopTests(unittest.TestCase):
    def _listener(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 0))
        s.settimeout(1.0)
        self.addCleanup(s.close)
        return s, s.getsockname()[1]

    def test_three_estop_datagrams(self) -> None:
        s, port = self._listener()
        cmd = R.UdpCommander("127.0.0.1", port)
        self.addCleanup(cmd.close)
        sent, errs = cmd.estop()
        self.assertEqual((sent, errs), (3, []))
        got = [s.recvfrom(64)[0] for _ in range(3)]
        self.assertEqual(got, [b"ESTOP\n"] * 3)

    def test_do_estop_writes_journal_and_control(self) -> None:
        s, port = self._listener()
        cmd = R.UdpCommander("127.0.0.1", port)
        self.addCleanup(cmd.close)
        with TemporaryDirectory() as tmp:
            R.update_control(tmp, pause=True)
            res = R.do_estop(cmd, tmp, now=1800000000.0)
            self.assertEqual((res["sent"], res["journal"], res["control"]), (3, True, True))
            evs, _ = R.load_journal(Path(tmp) / "journal.jsonl")
            self.assertEqual(evs, [{"t": 1800000000.0, "ev": "estop", "reason": "operator key",
                                    "source": "tui"}])
            self.assertEqual(R.read_control(tmp), {"pause": True, "abort": True, "seq": 2})
        self.assertEqual([s.recvfrom(64)[0] for _ in range(3)], [b"ESTOP\n"] * 3)

    def test_estop_without_batch_and_with_broken_batch_dir(self) -> None:
        s, port = self._listener()
        cmd = R.UdpCommander("127.0.0.1", port)
        self.addCleanup(cmd.close)
        res = R.do_estop(cmd, None)
        self.assertEqual((res["sent"], res["journal"], res["control"]), (3, None, None))
        res = R.do_estop(cmd, "/nonexistent/batch/dir")     # files fail, UDP still went out
        self.assertEqual(res["sent"], 3)
        self.assertNotEqual(res["journal"], True)
        self.assertEqual(len([s.recvfrom(64)[0] for _ in range(6)]), 6)

    def test_replay_estop_sends_but_leaves_files(self) -> None:
        s, port = self._listener()
        with TemporaryDirectory() as tmp:
            app = R.App(drone="127.0.0.1", port=port, batch_dir=tmp, readonly=True)
            self.addCleanup(app.commander.close)
            app.estop()
            self.assertEqual(os.listdir(tmp), [])
        self.assertEqual([s.recvfrom(64)[0] for _ in range(3)], [b"ESTOP\n"] * 3)


class ModelTests(unittest.TestCase):
    def test_compute_eta(self) -> None:
        self.assertIsNone(R.compute_eta([], 8, 0))
        self.assertAlmostEqual(R.compute_eta([10.0, 20.0], 8, 2), 90.0)
        self.assertAlmostEqual(R.compute_eta([10.0, 20.0], 8, 2, current_elapsed=5.0), 85.0)
        self.assertEqual(R.compute_eta([10.0], 3, 3), 0.0)
        self.assertEqual(R.compute_eta([10.0], 3, 2, current_elapsed=50.0), 0.0)

    def test_best_trial_skips_failed(self) -> None:
        rows = [{"trial": 1, "status": "ok", "metrics": {"score": 2.0}},
                {"trial": 2, "status": "estop", "estop": True, "metrics": {"score": 0.1}},
                {"trial": 3, "status": "ok", "metrics": {"score": 1.5}},
                {"trial": 4, "status": "aborted", "metrics": {"score": 0.2}},
                {"trial": 5, "status": "ok", "metrics": {"score": None}}]
        self.assertEqual(R.best_trial(rows), 3)
        self.assertEqual(R.best_trial(rows, lower_is_better=False), 1)
        self.assertIsNone(R.best_trial([]))

    def test_gains_diff(self) -> None:
        self.assertEqual(R.gains_diff({"ATT_P": 5.0, "RATE_P": 60}, {"ATT_P": 5, "RATE_P": 40}),
                         "RATE_P=60")
        self.assertEqual(R.gains_diff({"ATT_P": 5.0}, {"ATT_P": 5}), "=base")

    def test_batch_state_from_journal(self) -> None:
        b = R.BatchState("/x")
        for e in [{"t": 0, "ev": "batch_start"},
                  {"t": 1, "ev": "trial_start", "trial": 1, "gains": {"ATT_P": 5}, "exc": {}},
                  {"t": 6, "ev": "trial_end", "trial": 1, "status": "ok"},
                  {"t": 6.5, "ev": "metrics", "trial": 1, "score": 1.0},
                  {"t": 7, "ev": "trial_start", "trial": 2, "gains": {"ATT_P": 8}, "exc": {}},
                  {"t": 8, "ev": "estop", "source": "fc", "reason": "tilt"},
                  {"t": 8, "ev": "trial_end", "trial": 2, "status": "estop"}]:
            b.apply(e)
        self.assertEqual(b.n_done(), 2)
        self.assertEqual(b.durations(), [5, 1])
        self.assertTrue(b.trials[2]["estop"])
        self.assertEqual(R.best_trial(b.rows()), 1)

    def test_rec_csv_and_marks(self) -> None:
        with TemporaryDirectory() as tmp:
            p = Path(tmp) / "rec.csv"
            p.write_text("t_us,exc_id,roll,roll_tgt,exc_val\n"
                         "1000000,0,0.0,0.0,0\n1010000,1,0.01,0.1,0.1\n1020000,1,0.05,0.1,0.1\n"
                         "1030000,1,0.08,0.0,0\n", encoding="utf-8")
            rec = R.load_rec_csv(p)
            self.assertAlmostEqual(rec["t"][1], 0.01)
            self.assertAlmostEqual(rec["roll"][1], 0.5729577951308232)
            self.assertAlmostEqual(rec["sp"][1], 5.729577951308232)
            self.assertEqual(len(R.exc_marks(rec["t"], rec["exc"], rec["exc_id"])), 2)
            p.write_text("a,b\n1,2\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                R.load_rec_csv(p)


class RasteriserTests(unittest.TestCase):
    def test_dimensions(self) -> None:
        pts = [(i * 0.1, 10 * ((i % 20) - 10)) for i in range(100)]
        for mode in ("braille", "block", "ascii"):
            for w, h in ((1, 1), (7, 3), (80, 20), (0, 5), (5, 0)):
                rows, styles = R.rasterise([(pts, "roll", "*")], w, h, (0, 10), (-50, 50), mode,
                                           hlines=[(0, "dim")], vmarks=[(5, "exc")])
                self.assertEqual(len(rows), h, (mode, w, h))
                self.assertEqual(len(styles), h)
                for r, s in zip(rows, styles):
                    self.assertEqual(len(r), w)
                    self.assertEqual(len(s), w)

    def test_out_of_range_is_clipped_and_nan_skipped(self) -> None:
        rows, styles = R.rasterise([([(0, 1e9), (1, float("nan")), (2, -1e9), (99, 0)], "roll", "*")],
                                   10, 4, (0, 2), (-1, 1), "ascii")
        self.assertIn("*", rows[0])
        self.assertIn("*", rows[3])

    def test_braille_draws_dots_and_later_series_owns_colour(self) -> None:
        line = [(0, 0), (1, 0)]
        rows, styles = R.rasterise([(line, "sp", "-"), (line, "roll", "*")], 4, 2, (0, 1), (-1, 1))
        drawn = [ch for r in rows for ch in r if ch != " "]
        self.assertTrue(drawn and all(0x2800 < ord(c) <= 0x28FF for c in drawn))
        self.assertTrue(all(s == "roll" for row in styles for s in row if s))

    def test_progress_bar(self) -> None:
        for g in (R.GLYPHS_UNICODE, R.GLYPHS_ASCII):
            for frac in (0, 0.01, 0.5, 0.999, 1.0, 2.0):
                self.assertEqual(len(R._progress_bar(frac, 17, g)), 17)


class RenderTests(unittest.TestCase):
    def test_frames_at_many_sizes_and_both_glyph_sets(self) -> None:
        with TemporaryDirectory() as tmp:
            w, app, ing = selftest_app(tmp)
            for g in (R.GLYPHS_UNICODE, R.GLYPHS_ASCII):
                for cols, lines in ((120, 40), (80, 24), (100, 30), (60, 16), (40, 12), (20, 4),
                                    (12, 2), (1, 1), (0, 0)):
                    c = R.render_frame(app, cols, lines, w.t, g)
                    text = c.to_text()
                    rows = text.split("\n") if (lines and cols) else []
                    self.assertEqual(len(rows), lines if cols else 0)
                    for r in rows:
                        self.assertEqual(len(r), cols, (cols, lines))
                    if cols >= 7 and lines >= 1:
                        self.assertIn("ESTOP", rows[0], (cols, lines))
                    if g is R.GLYPHS_ASCII:
                        self.assertTrue(all(ord(ch) < 128 for ch in text), (cols, lines))
            big = R.render_frame(app, 120, 40, w.t, R.GLYPHS_UNICODE).to_text()
            for word in ("Roll", "Gains", "Batch", "Results", "Events", "trial", "RUN",
                         "quits the viewer only"):
                self.assertIn(word, big)
            self.assertIn(R.GLYPHS_UNICODE["star"], big)
            close_all(w, app, ing)

    def test_trial_view_and_estop_banner(self) -> None:
        with TemporaryDirectory() as tmp:
            w, app, ing = selftest_app(tmp, t_end=40.0, listen=True)
            app.sel, app.view = 3, "trial"
            text = R.render_frame(app, 100, 26, w.t, R.GLYPHS_UNICODE).to_text()
            self.assertIn("Trial 3", text)
            self.assertIn("OS ", text)
            app.sel = 99
            app.batch.row(99)
            self.assertIn("not fetched", R.render_frame(app, 100, 26, w.t, R.GLYPHS_UNICODE).to_text())
            app.view = "main"
            app.estop(now=w.t)
            text = R.render_frame(app, 120, 40, w.t + 0.1, R.GLYPHS_UNICODE).to_text()
            self.assertIn("ESTOP SENT x3 -> 127.0.0.1:%d" % w.cmd_port, text)
            time.sleep(0.05)
            w.step(w.t + 0.2)
            self.assertEqual(w.counts["ESTOP"], 3)
            self.assertTrue(R.read_control(w.batch_dir)["abort"])
            html = R.render_frame(app, 120, 40, w.t, R.GLYPHS_UNICODE).to_html()
            self.assertIn("<pre>", html)
            ansi = R.render_frame(app, 80, 24, w.t, R.GLYPHS_UNICODE).to_ansi()
            self.assertIn("\x1b[", ansi)
            close_all(w, app, ing)

    def test_control_chars_and_non_ascii_never_reach_the_screen(self) -> None:
        with TemporaryDirectory() as tmp:
            w, app, ing = selftest_app(tmp)
            app.ingest_log(["%.3f RIG EVENT x\x00\x1b[2J\r\t\ufffd\x9b y" % w.t])
            app.ingest_journal([{"t": w.t, "ev": "note", "text": "tilt 25\u00b0 \x07bell"}])
            for g in (R.GLYPHS_UNICODE, R.GLYPHS_ASCII):
                text = R.render_frame(app, 120, 40, w.t, g).to_text()
                self.assertIn("RIG EVENT x", text)
                bad = [ch for ch in text if ch != "\n" and (ord(ch) < 32 or 127 <= ord(ch) < 160)]
                self.assertEqual(bad, [])
                if g is R.GLYPHS_ASCII:
                    self.assertTrue(all(ord(ch) < 128 for ch in text))
            close_all(w, app, ing)

    def test_failed_estop_never_reads_as_sent(self) -> None:
        with TemporaryDirectory() as tmp:
            w, app, ing = selftest_app(tmp)
            app.estops.append((w.t, {"sent": 0, "errors": ["[Errno 101] Network is unreachable"],
                                     "journal": None, "control": None, "seq": 0}))
            text = R.render_frame(app, 120, 40, w.t + 0.1, R.GLYPHS_UNICODE).to_text()
            self.assertIn("ESTOP SEND FAILED 0/3", text)
            self.assertNotIn("ESTOP SENT", text)
            close_all(w, app, ing)

    def test_estop_key_survives_render_exceptions(self) -> None:
        import curses
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 0))
        s.settimeout(1.0)
        self.addCleanup(s.close)
        app = R.App(drone="127.0.0.1", port=s.getsockname()[1])
        self.addCleanup(app.commander.close)

        class FakeScreen:
            keys = [ord("E"), -1, ord("j"), -1, ord("q")]

            def __init__(self):
                self.calls = 0

            def getch(self):
                self.calls += 1
                if self.calls > 1000:
                    raise AssertionError("key loop did not quit")
                return self.keys.pop(0) if self.keys else -1

            def getmaxyx(self):
                return (40, 120)

            def __getattr__(self, name):          # keypad, timeout, erase, addstr, ...
                return lambda *a, **k: None

        def boom(*a, **k):
            raise RuntimeError("render bug")

        orig_render, orig_painter, orig_doupdate = R.render_frame, R.CursesPainter, curses.doupdate
        R.render_frame = boom
        R.CursesPainter = lambda cu, scr: None
        curses.doupdate = lambda: None
        try:
            rc = R._curses_main(FakeScreen(), app, R.GLYPHS_UNICODE, threading.Event())
        finally:
            R.render_frame, R.CursesPainter, curses.doupdate = orig_render, orig_painter, orig_doupdate
        self.assertEqual(rc, 0)
        self.assertEqual([s.recvfrom(64)[0] for _ in range(3)], [b"ESTOP\n"] * 3)
        self.assertTrue(any("render bug" in e for e in app.errors))

    def test_stack_alloc(self) -> None:
        self.assertEqual(R.stack_alloc(21, [("plot", 7, 3), ("gb", 6, 0), ("results", 4, 3),
                                            ("events", 5, 1)]),
                         [("plot", 9), ("gb", 6), ("results", 6)])


class LogFollowerTests(unittest.TestCase):
    def test_rotation_to_newer_file(self) -> None:
        with TemporaryDirectory() as tmp:
            a = Path(tmp) / "raw_20261104_100000.log"
            a.write_text("1.000 old\n", encoding="utf-8")
            f = R.LogFollower(directory=tmp)
            self.assertEqual(f.poll(now=0.0), ["1.000 old"])
            with open(a, "a") as fh:
                fh.write("2.000 more\n3.000 part")
            self.assertEqual(f.poll(now=0.5), ["2.000 more"])
            b = Path(tmp) / "raw_20261104_110000.log"
            b.write_text("4.000 new\n", encoding="utf-8")
            os.utime(b, (time.time() + 5, time.time() + 5))
            self.assertEqual(f.poll(now=2.0), ["3.000 part", "4.000 new"])
            self.assertEqual(f.cur, b)
            f.close()

    def test_explicit_path_truncation_and_backlog(self) -> None:
        with TemporaryDirectory() as tmp:
            p = Path(tmp) / "x.log"
            p.write_text("".join("%d.000 line%d\n" % (i, i) for i in range(1000)), encoding="utf-8")
            f = R.LogFollower(path=p, backlog=100)
            got = f.poll(now=0.0)
            self.assertTrue(got and got[-1] == "999.000 line999" and len(got) < 10)
            self.assertTrue(all(ln.split()[0].endswith(".000") for ln in got))   # no partial line
            p.write_text("5.000 fresh\n", encoding="utf-8")
            self.assertEqual(f.poll(now=2.0), ["5.000 fresh"])
            f.close()


def _pty_ok():
    try:
        import _curses  # noqa: F401
        import pty
        m, s = pty.openpty()
        os.close(m)
        os.close(s)
        return True
    except Exception:  # noqa: BLE001
        return False


@unittest.skipUnless(sys.platform.startswith(("linux", "darwin")) and _pty_ok(),
                     "needs a pty and _curses")
class PtySmokeTests(unittest.TestCase):
    """Run the real curses UI in --selftest under a pseudo-terminal, press E then q."""

    def _run(self, cols, rows, lang="C.UTF-8", term="xterm-256color"):
        import fcntl
        import pty
        import select
        import struct
        import termios
        pid, fd = pty.fork()
        if pid == 0:                                       # child
            os.environ.update(TERM=term, LANG=lang, LC_ALL=lang)
            os.environ.pop("NOTION_TOKEN", None)
            os.execv(sys.executable, [sys.executable, str(ROOT / "tools" / "rb" / "rig_tui.py"),
                                      "--selftest"])
        fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", rows, cols, 0, 0))
        out = bytearray()

        def pump(seconds):
            end = time.time() + seconds
            while time.time() < end:
                r, _, _ = select.select([fd], [], [], 0.05)
                if r:
                    try:
                        chunk = os.read(fd, 65536)
                    except OSError:
                        return False
                    if not chunk:
                        return False
                    out.extend(chunk)
            return True

        try:
            pump(1.5)
            os.write(fd, b"E")
            pump(1.0)
            os.write(fd, b"q")
            pump(2.0)
            deadline = time.time() + 5
            status = None
            while time.time() < deadline:
                wpid, status = os.waitpid(pid, os.WNOHANG)
                if wpid:
                    break
                pump(0.1)
            else:
                os.kill(pid, 9)
                os.waitpid(pid, 0)
                self.fail("viewer did not exit on q")
        finally:
            os.close(fd)
        return os.WEXITSTATUS(status), out.decode("utf-8", "replace")

    def test_selftest_120x40_and_80x24(self) -> None:
        for cols, rows in ((120, 40), (80, 24)):
            code, text = self._run(cols, rows)
            self.assertEqual(code, 0, text[-2000:])
            self.assertIn("ESTOP", text)
            self.assertIn("received ESTOP x3", text)
            self.assertIn("(nothing sent to 192.168.4.1)", text)

    def test_selftest_c_locale(self) -> None:
        code, text = self._run(80, 24, lang="C")
        self.assertEqual(code, 0, text[-2000:])
        self.assertIn("received ESTOP x3", text)

    def test_selftest_8_colour_and_no_colour_terminals(self) -> None:
        for term in ("xterm", "vt100"):
            code, text = self._run(80, 24, term=term)
            self.assertEqual(code, 0, (term, text[-2000:]))
            self.assertIn("received ESTOP x3", text)
            self.assertNotIn("38;5;", text)          # no 256-colour escapes on these


if __name__ == "__main__":
    unittest.main()
