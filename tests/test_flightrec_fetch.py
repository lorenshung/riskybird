"""rb flightlog fetch's reader (tools/rb/flightrec_fetch.py) against a synthetic recorder image.

No hardware: esp_tsi_load (the ESP link) is replaced by a stub whose Tsi.read serves a
header, a main ring and two preroll rings packed in memory, as the FC would hold them.
Both layouts: v2 (flight) and v3/rig (ROSE_RIG=1, with the gain snapshot and change ring).
FlightrecHostSim also compiles integration/zephyr/flightrec/src/flightrec.c on the host,
runs a scenario through it and reads its memory back through fetch().
"""

from __future__ import annotations

import contextlib
import csv
import importlib.util
import io
import math
import json
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
HDR_ADDR, SMP_ADDR, PRE_ADDR = 0x8000_1000, 0x8001_0000, 0x8008_0000


class _Ser:
    def close(self) -> None:
        pass


def _stub_link(mem: dict[int, bytes]) -> types.ModuleType:
    E = types.ModuleType("esp_tsi_load")

    class LoadError(RuntimeError):
        pass

    class Tsi:
        def __init__(self, *_a) -> None:
            pass

        def read(self, addr: int, n: int) -> bytes:
            for base, blob in mem.items():
                if base <= addr and addr + n <= base + len(blob):
                    return blob[addr - base:addr - base + n]
            raise LoadError(f"read outside the image: {addr:#x}+{n}")

        def end(self) -> None:
            pass

    syms = {"g_flightrec_hdr": HDR_ADDR, "g_flightrec_samples": SMP_ADDR,
            "g_flightrec_pre": PRE_ADDR}
    E.LoadError = LoadError
    E.Tsi = Tsi
    E.elf_symbol = lambda _elf, name: syms.get(name)
    E.open_link = lambda _a: _Ser()
    E.handshake = lambda *_a: "stub"
    E.text_samples = lambda _elf: []
    return E


def _import_fetch(E: types.ModuleType):
    """A fresh copy of the script bound to this stub (not the cached tools.rb attribute)."""
    spec = importlib.util.spec_from_file_location(
        "flightrec_fetch_under_test", ROOT / "tools" / "rb" / "flightrec_fetch.py")
    mod = importlib.util.module_from_spec(spec)
    # The script puts tools/rb on sys.path at import; keep that out of later tests.
    with mock.patch.dict(sys.modules, {"esp_tsi_load": E}), \
            mock.patch.object(sys, "path", list(sys.path)):
        spec.loader.exec_module(mod)
    return mod


class FlightrecFetch(unittest.TestCase):
    CAP, PREROLL, DIV = 16, 4, 2
    ROLL_G = math.tan(math.radians(2.5))   # Gibbs for 5 degrees of roll

    def _sample(self, F, k: int, motor: bool) -> bytes:
        duty = [0.5, 0.5, 0.5, 0.5] if motor else [0.0] * 4
        return struct.pack(F.SAMPLE_FMT, 1000 * k, 1000, 0x09, (self.DIV * k) & 0xFF,
                           0.1, 0.2, 0.3, self.ROLL_G, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.5, 1.0, 0.0, 0.0, 0.0, 0.0, *duty)

    def _image(self, F, magic: int | None = None) -> dict[int, bytes]:
        """Segment 2, frozen: 3 preroll samples in ring 1 (wrapping), then 12 ring samples
        10..21 (wrapping at 16); the motors start at the third ring sample."""
        cap, pre, size = self.CAP, self.PREROLL, F.SAMPLE_SIZE
        seg_start, seg_end, pre_buf, pre_head, pre_count = 10, 22, 1, 2, 3
        hdr = struct.pack(F.HDR_FMT, F.MAGIC if magic is None else magic, F.VERSION, size,
                          cap, self.DIV, 2000, pre, seg_end, seg_start, seg_end, 2, 1,
                          SMP_ADDR, PRE_ADDR, pre_buf, pre_head, pre_count)
        k = 0
        pre_ring = bytearray(2 * pre * size)
        for j in range(pre_count):
            slot = (pre_head + j) % pre
            off = (pre_buf * pre + slot) * size
            pre_ring[off:off + size] = self._sample(F, k, False)
            k += 1
        ring = bytearray(cap * size)
        for i in range(seg_start, seg_end):
            slot = i % cap
            ring[slot * size:(slot + 1) * size] = self._sample(F, k, i >= seg_start + 2)
            k += 1
        return {HDR_ADDR: hdr, SMP_ADDR: bytes(ring), PRE_ADDR: bytes(pre_ring)}

    def _run(self, F, out: Path) -> tuple[int, str, str]:
        argv = ["flightrec_fetch.py", "--url", "socket://stub", "--elf", "stub.elf",
                "--out", str(out)]
        so, se = io.StringIO(), io.StringIO()
        with mock.patch.object(sys, "argv", argv), contextlib.redirect_stdout(so), \
                contextlib.redirect_stderr(se):
            rc = F.main()
        return rc, so.getvalue(), se.getvalue()

    def test_layout_matches_the_c_header(self) -> None:
        F = _import_fetch(_stub_link({}))
        self.assertEqual(F.HDR_SIZE, 64)       # BUILD_ASSERTs in flightrec.c
        self.assertEqual(F.SAMPLE_SIZE, 84)
        h = (ROOT / "integration/zephyr/flightrec/include/riskybird/flightrec.h").read_text()
        self.assertEqual(int(re.search(r"FLIGHTREC_MAGIC\s+(0x[0-9a-fA-F]+)u", h).group(1), 16),
                         F.MAGIC)
        self.assertEqual(int(re.search(r"FLIGHTREC_VERSION\s+(\d+)u", h).group(1)), F.VERSION)
        for bit, name in F.FLAGS:
            self.assertRegex(h, rf"FLIGHTREC_F_{name.upper()}\s+{bit:#04x}u")

    def test_frozen_segment_reads_preroll_then_wrapped_ring(self) -> None:
        mem = {}
        F = _import_fetch(_stub_link(mem))
        mem.update(self._image(F))
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "rec"
            rc, so, se = self._run(F, out)
            self.assertEqual(rc, 0, se)
            with open(f"{out}.csv", newline="") as f:
                rows = list(csv.DictReader(f))
            blob = Path(f"{out}.bin").read_bytes()
        self.assertEqual(len(rows), 3 + 12)
        self.assertEqual(len(blob), F.HDR_SIZE + 15 * F.SAMPLE_SIZE)
        self.assertEqual([int(r["t_us"]) for r in rows], [1000 * k for k in range(15)])
        first_motor = next(r for r in rows if float(r["duty0"]) > 0)
        self.assertEqual(first_motor["t_us"], "5000")
        self.assertEqual(float(first_motor["t_s"]), 0.0)
        self.assertEqual(float(rows[0]["t_s"]), -0.005)
        self.assertEqual(rows[0]["roll_deg"], "5.000")
        self.assertEqual((rows[0]["armed"], rows[0]["caldone"], rows[0]["estop"]), ("1", "1", "0"))
        self.assertIn("0 seq gap(s)", so)
        self.assertIn("10 with motors on", so)

    def test_bad_magic_is_refused(self) -> None:
        mem = {}
        F = _import_fetch(_stub_link(mem))
        mem.update(self._image(F, magic=0xDEADBEEF))
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "rec"
            rc, _so, se = self._run(F, out)
            self.assertEqual(rc, 1)
            self.assertIn("magic", se)
            self.assertFalse(Path(f"{out}.csv").exists())


RIG_CONTRACT_COLUMNS = (  # RIG-SPEC "Recorder v3 (rig) <-> runner contract", verbatim
    "t_s,t_us,dt_us,seq,flags,armed,estop,arming,caldone,tof_valid,flow_valid,committed,"
    "rig_state,rs_name,exc_id,gain_seq,gyro_p,gyro_q,gyro_r,roll_deg,pitch_deg,"
    "roll_tgt_deg,rate_tgt_dps,rate_cmd,moment_mNm,exc_val,coll_thrust_N,att_int,rate_int,"
    "pitch_cmd,yaw_cmd,accel_y,accel_z,vbat,duty0,duty1,duty2,duty3,"
    "axis,ang_deg,ang_tgt_deg,ang_rate_dps").split(",")   # + RIG-SPEC "Pitch axis", appended
FC_SRC = ROOT / "workloads" / "flight_controller" / "src"
FR_DIR = ROOT / "integration" / "zephyr" / "flightrec"


def _rig_header(F, *, cap, div, preroll, count, seg_start, seg_end, segments, frozen,
                pre_buf, pre_head, pre_count, snap_seq, snap, changes, layout=None) -> bytes:
    """struct flightrec_rig_header: the v2 header (version 3), the extension, the ring.
    changes: (gain_seq, index, value, t_us), oldest first; the last 64 are kept in the ring
    at their count % 64 slots, like flightrec_rig_gain() writes them."""
    hdr = struct.pack(F.HDR_FMT, F.MAGIC, F.VERSION_RIG, F.RIG_SAMPLE_SIZE, cap, div, 2000,
                      preroll, count, seg_start, seg_end, segments, frozen, SMP_ADDR, PRE_ADDR,
                      pre_buf, pre_head, pre_count)
    ext = struct.pack(F.RIG_EXT_FMT, F.LAYOUT_RIG if layout is None else layout,
                      F.RIG_N_GAINS, F.GAIN_RING, snap_seq, segments, *snap, len(changes))
    ring = bytearray(F.GAIN_RING * F.GAIN_CHANGE_SIZE)
    for k, (seq, idx, val, t_us) in enumerate(changes):
        off = (k % F.GAIN_RING) * F.GAIN_CHANGE_SIZE
        ring[off:off + F.GAIN_CHANGE_SIZE] = struct.pack(F.GAIN_CHANGE_FMT, seq, idx, 0, val, t_us)
    return hdr + ext + bytes(ring)


class RigLayout(unittest.TestCase):
    """v3 / layout 2: the roll-harness sample, the gain snapshot and the change ring."""
    CAP, PREROLL, DIV = 16, 4, 4
    SNAP = [5.0, 0.0, 40.0, 0.0, 0.0, 0.85, 30.0, 0.0, 0.0, 1.0, 1.0, 30.0]

    def _sample(self, F, k: int, motor: bool, *, rs: int = 3, flags: int = 0x09,
                gseq: int = 3) -> bytes:
        duty = [5000, 5100, 4900, 5000] if motor else [0] * 4
        return struct.pack(F.RIG_SAMPLE_FMT, 1000 * k, 1000, (self.DIV * k) & 0xFF, flags,
                           rs if motor else 0, 7, gseq,
                           0.1, -0.2, 0.3,                       # gyro
                           math.radians(5.0), math.radians(-1.0),  # roll, pitch
                           math.radians(2.0), math.radians(10.0),  # roll_tgt, rate_tgt
                           1.5, 0.0025, 0.001, 0.49,             # rate_cmd moment exc coll
                           0.01, 0.02, 0.3, -0.4, 0.5, -9.8, 7.9, *duty)

    def _image(self, F, changes=None, last_flags: int = 0x09):
        """Segment 2, frozen: 3 preroll samples in ring 1 (wrapping), then 12 ring samples
        10..21 (wrapping at 16); the motors start at the third ring sample."""
        cap, pre, size = self.CAP, self.PREROLL, F.RIG_SAMPLE_SIZE
        seg_start, seg_end, pre_buf, pre_head, pre_count = 10, 22, 1, 2, 3
        if changes is None:
            changes = [(1, 0, 6.0, 100), (2, 2, 45.0, 200), (3, 5, 0.85, 300),
                       (4, 1, 0.5, 400)]
        hdr = _rig_header(F, cap=cap, div=self.DIV, preroll=pre, count=seg_end,
                          seg_start=seg_start, seg_end=seg_end, segments=2, frozen=1,
                          pre_buf=pre_buf, pre_head=pre_head, pre_count=pre_count,
                          snap_seq=3, snap=self.SNAP, changes=changes)
        k = 0
        pre_ring = bytearray(2 * pre * size)
        for j in range(pre_count):
            off = (pre_buf * pre + (pre_head + j) % pre) * size
            pre_ring[off:off + size] = self._sample(F, k, False, gseq=1 if j == 0 else 3)
            k += 1
        ring = bytearray(cap * size)
        for i in range(seg_start, seg_end):
            slot = i % cap
            last = i == seg_end - 1
            ring[slot * size:(slot + 1) * size] = self._sample(
                F, k, i >= seg_start + 2, gseq=4 if i >= seg_end - 3 else 3,
                flags=last_flags if last else 0x09)
            k += 1
        return {HDR_ADDR: hdr, SMP_ADDR: bytes(ring), PRE_ADDR: bytes(pre_ring)}

    def _fetch(self, F, tmp: str) -> tuple[dict, list[dict], dict, bytes]:
        logs: list[str] = []
        res = F.fetch("socket://stub", "stub.elf", Path(tmp) / "rec", log=logs.append)
        with open(res["csv"], newline="") as f:
            rd = csv.reader(f)
            header = next(rd)
            rows = [dict(zip(header, r)) for r in rd]
        self.assertEqual(header, RIG_CONTRACT_COLUMNS)
        with open(res["gains"]) as f:
            gains = json.load(f)
        res["log"] = "\n".join(logs)
        return res, rows, gains, Path(res["bin"]).read_bytes()

    def test_layout_matches_the_c_header(self) -> None:
        F = _import_fetch(_stub_link({}))
        self.assertEqual(F.RIG_SAMPLE_SIZE, 92)     # BUILD_ASSERTs in flightrec.c
        self.assertEqual(F.GAIN_CHANGE_SIZE, 12)
        self.assertEqual(F.RIG_EXT_SIZE, 68)
        self.assertEqual(F.RIG_HDR_SIZE, 64 + 68 + 12 * 64)
        h = (FR_DIR / "include/riskybird/flightrec.h").read_text()
        c = (FR_DIR / "src/flightrec.c").read_text()
        for name, want in (("FLIGHTREC_VERSION_RIG", F.VERSION_RIG),
                           ("FLIGHTREC_LAYOUT_FLIGHT", F.LAYOUT_FLIGHT),
                           ("FLIGHTREC_LAYOUT_RIG", F.LAYOUT_RIG),
                           ("FLIGHTREC_RIG_GAINS", F.RIG_N_GAINS),
                           ("FLIGHTREC_GAIN_RING", F.GAIN_RING)):
            self.assertEqual(int(re.search(rf"#define {name}\s+(\d+)u", h).group(1)), want, name)
        self.assertIn("sizeof(struct flightrec_rig_sample) == 92", c)
        self.assertIn("== 64 + 68 + 12 * FLIGHTREC_GAIN_RING", c)

    def test_gain_and_state_names_match_fc_rig(self) -> None:
        """No hand-kept list may drift from the C: the enum, fc_rig.cpp's gain_table and
        RIG_GAIN_NAMES must agree index for index (and the run states with their enum)."""
        F = _import_fetch(_stub_link({}))
        hdr = (FC_SRC / "fc_rig.h").read_text()
        cpp = (FC_SRC / "fc_rig.cpp").read_text()
        enum = re.search(r"enum fc_rig_gain_id \{(.*?)\};", hdr, re.S).group(1)
        ids = re.findall(r"^\s*FC_RIG_(\w+?)\s*(?:=\s*0\s*)?,?\s*(?:/\*.*)?$", enum, re.M)
        self.assertEqual(ids[-1], "GAIN_COUNT")
        ids = ids[:-1]
        table = re.search(r"gain_table\[FC_RIG_GAIN_COUNT\] = \{(.*?)\n\};", cpp, re.S).group(1)
        rows = re.findall(r"/\*\s*FC_RIG_(\w+?)\s*\*/\s*\{\s*\"(\w+)\"", table)
        self.assertEqual(tuple(ids), F.RIG_GAIN_NAMES)
        self.assertEqual(tuple(r[0] for r in rows), F.RIG_GAIN_NAMES)   # table comment = enum
        self.assertEqual(tuple(r[1] for r in rows), F.RIG_GAIN_NAMES)   # wire names
        rs = re.search(r"enum fc_rig_run_state \{(.*?)\};", hdr, re.S).group(1)
        self.assertEqual(tuple(re.findall(r"^\s*FC_RIG_(\w+)", rs, re.M)), F.RIG_STATE_NAMES)

    def test_columns_are_the_contract(self) -> None:
        F = _import_fetch(_stub_link({}))
        self.assertEqual(F.RIG_COLUMNS, RIG_CONTRACT_COLUMNS)

    def test_frozen_rig_segment(self) -> None:
        mem = {}
        F = _import_fetch(_stub_link(mem))
        mem.update(self._image(F, last_flags=0x0B))
        with tempfile.TemporaryDirectory() as tmp:
            res, rows, gains, blob = self._fetch(F, tmp)
        self.assertEqual(res["layout"], F.LAYOUT_RIG)
        self.assertEqual((res["samples"], res["gaps"]), (15, 0))
        self.assertEqual(len(blob), F.RIG_HDR_SIZE + 15 * F.RIG_SAMPLE_SIZE)
        self.assertEqual(blob[:F.RIG_HDR_SIZE], mem[HDR_ADDR])
        self.assertEqual([int(r["t_us"]) for r in rows], [1000 * k for k in range(15)])
        self.assertEqual(float(rows[5]["t_s"]), 0.0)          # first motor tick
        self.assertEqual(float(rows[0]["t_s"]), -0.005)
        r = rows[5]
        self.assertEqual((r["rig_state"], r["rs_name"], r["exc_id"]), ("3", "RUN", "7"))
        self.assertEqual(rows[0]["rs_name"], "IDLE")
        self.assertEqual(rows[-1]["rs_name"], "ESTOP")          # flags 0x0B: estop latched
        self.assertEqual((r["armed"], r["caldone"], r["estop"]), ("1", "1", "0"))
        self.assertAlmostEqual(float(r["roll_deg"]), 5.0, places=3)
        self.assertAlmostEqual(float(r["pitch_deg"]), -1.0, places=3)
        self.assertAlmostEqual(float(r["roll_tgt_deg"]), 2.0, places=3)
        self.assertAlmostEqual(float(r["rate_tgt_dps"]), 10.0, places=3)
        self.assertAlmostEqual(float(r["moment_mNm"]), 2.5, places=4)
        self.assertAlmostEqual(float(r["exc_val"]), 0.001, places=7)
        self.assertAlmostEqual(float(r["coll_thrust_N"]), 0.49, places=5)
        self.assertAlmostEqual(float(r["vbat"]), 7.9, places=3)
        self.assertEqual([r[f"duty{i}"] for i in range(4)], ["0.5000", "0.5100", "0.4900", "0.5000"])
        self.assertEqual(rows[0]["duty0"], "0.0000")
        self.assertIn("0 seq gap(s)", res["log"])
        self.assertIn("10 with motors on", res["log"])
        # gains.json: the snapshot at segment start, then every change, oldest first.
        self.assertEqual(gains["snapshot"]["gain_seq"], 3)
        self.assertEqual(gains["snapshot"]["gains"],
                         dict(zip(F.RIG_GAIN_NAMES, [F.f32(v) for v in self.SNAP])))
        self.assertEqual(gains["snapshot"]["gains"]["ATT_GAIN"], 0.85)   # shortest float32
        self.assertEqual([(c["gain_seq"], c["name"], c["value"], c["t_us"])
                          for c in gains["changes"]],
                         [(1, "ATT_P", 6.0, 100), (2, "RATE_P", 45.0, 200),
                          (3, "ATT_GAIN", 0.85, 300), (4, "ATT_I", 0.5, 400)])
        self.assertEqual(gains["changes_lost"], 0)
        # Gains in force at each row's gain_seq.
        last = F.gains_at(gains, int(rows[-1]["gain_seq"]))
        self.assertEqual((last["ATT_I"], last["RATE_P"], last["ATT_P"]), (0.5, 40.0, 5.0))
        first = F.gains_at(gains, int(rows[0]["gain_seq"]))     # preroll, before seq 2 and 3
        self.assertEqual(first["ATT_P"], 5.0)        # not changed after seq 1
        self.assertIsNone(first["RATE_P"])           # its value before seq 2 was never logged
        self.assertEqual(first["ATT_I"], 0.0)

    def test_gain_ring_wraps(self) -> None:
        """70 changes: the ring keeps the last 64, oldest first; gains before them are unknown."""
        mem = {}
        F = _import_fetch(_stub_link(mem))
        changes = [(s, s % 12, float(s), 10 * s) for s in range(1, 71)]
        mem.update(self._image(F, changes=changes))
        info = F.rig_gains(mem[HDR_ADDR][F.HDR_SIZE:])
        self.assertEqual(info["changes_lost"], 6)
        self.assertEqual([c["gain_seq"] for c in info["changes"]], list(range(7, 71)))
        self.assertEqual(info["changes"][0]["name"], F.RIG_GAIN_NAMES[7])
        info["snapshot"]["gain_seq"] = 3
        self.assertIsNone(F.gains_at(info, 10)["ATT_P"])      # seq 4..6 fell out of the ring
        info["snapshot"]["gain_seq"] = 6
        g = F.gains_at(info, 70)
        self.assertEqual(g[F.RIG_GAIN_NAMES[70 % 12]], 70.0)
        self.assertEqual(g[F.RIG_GAIN_NAMES[59 % 12]], 59.0)

    def test_appended_gains_only_in_the_change_ring(self) -> None:
        """LIFT_I / LIFT_I_LIM (enum 12, 13) are not in the v3 snapshot: the snapshot keeps the
        recorded 12 and their GAINs are named from the ring; gains_at() then has them."""
        mem = {}
        F = _import_fetch(_stub_link(mem))
        self.assertEqual(F.RIG_GAIN_NAMES[F.RIG_N_GAINS:], ("LIFT_I", "LIFT_I_LIM"))
        mem.update(self._image(F, changes=[(1, 0, 6.0, 100), (2, 12, 3.5, 200), (3, 13, 7.0, 300)]))
        info = F.rig_gains(mem[HDR_ADDR][F.HDR_SIZE:])
        self.assertEqual(tuple(info["snapshot"]["gains"]), F.RIG_GAIN_NAMES[:F.RIG_N_GAINS])
        self.assertEqual([c["name"] for c in info["changes"]], ["ATT_P", "LIFT_I", "LIFT_I_LIM"])
        info["snapshot"]["gain_seq"] = 1
        g = F.gains_at(info, 3)
        self.assertEqual((g["LIFT_I"], g["LIFT_I_LIM"]), (3.5, 7.0))

    def test_unknown_gain_count_is_refused(self) -> None:
        mem = {}
        F = _import_fetch(_stub_link(mem))
        img = self._image(F)
        hdr = bytearray(img[HDR_ADDR])
        struct.pack_into("<H", hdr, F.HDR_SIZE + 4, 13)       # n_gains
        img[HDR_ADDR] = bytes(hdr)
        mem.update(img)
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(F.FetchError, "13 gains"):
                F.fetch("socket://stub", "stub.elf", Path(tmp) / "rec", log=lambda _m: None)
            self.assertFalse((Path(tmp) / "rec.csv").exists())

    def test_link_failure_is_a_fetch_error(self) -> None:
        """The runner catches FetchError: a refused/busy ESP must not escape as OSError."""
        E = _stub_link({})

        def refuse(_a):
            raise ConnectionRefusedError("Connection refused")
        E.open_link = refuse
        F = _import_fetch(E)
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(F.FetchError, "refused"):
                F.fetch("socket://stub", "stub.elf", Path(tmp) / "rec", log=lambda _m: None)
            self.assertEqual(list(Path(tmp).iterdir()), [])

    def test_cli_reads_rig(self) -> None:
        mem = {}
        F = _import_fetch(_stub_link(mem))
        mem.update(self._image(F))
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "rec"
            argv = ["flightrec_fetch.py", "--url", "socket://stub", "--elf", "stub.elf",
                    "--out", str(out)]
            so, se = io.StringIO(), io.StringIO()
            with mock.patch.object(sys, "argv", argv), contextlib.redirect_stdout(so), \
                    contextlib.redirect_stderr(se):
                rc = F.main()
            self.assertEqual(rc, 0, se.getvalue())
            for ext in (".csv", ".bin", ".gains.json"):
                self.assertTrue(Path(f"{out}{ext}").exists(), ext)
        self.assertIn("[rig layout]", so.getvalue())


class FlightV2Api(unittest.TestCase):
    """The v2 image through the Python API: layout 1, no gains file."""
    CAP, PREROLL, DIV, ROLL_G = (FlightrecFetch.CAP, FlightrecFetch.PREROLL,
                                 FlightrecFetch.DIV, FlightrecFetch.ROLL_G)
    _sample = FlightrecFetch._sample
    _image = FlightrecFetch._image

    def test_api(self) -> None:
        mem = {}
        F = _import_fetch(_stub_link(mem))
        mem.update(self._image(F))
        with tempfile.TemporaryDirectory() as tmp:
            res = F.fetch("socket://stub", "stub.elf", Path(tmp) / "rec", log=lambda _m: None)
            self.assertEqual((res["layout"], res["gains"], res["samples"], res["gaps"]),
                             (F.LAYOUT_FLIGHT, None, 15, 0))
            self.assertFalse(Path(f"{tmp}/rec.gains.json").exists())
            with open(res["csv"], newline="") as f:
                self.assertEqual(next(csv.reader(f)), F.COLUMNS)


# ---- flightrec.c itself, on the host ----------------------------------------------------------

SIM_C = r"""
#include <stdio.h>
#include <string.h>
#include <riskybird/flightrec.h>

int64_t fake_ms;
extern flightrec_rec_t g_flightrec_samples[FLIGHTREC_CAPACITY];
extern flightrec_rec_t g_flightrec_pre[2][FLIGHTREC_PREROLL];
#if FLIGHTREC_RIG
extern struct flightrec_rig_header g_flightrec_hdr;
static float gains[FLIGHTREC_RIG_GAINS];
static volatile uint32_t gseq;
#else
extern struct flightrec_header g_flightrec_hdr;
#endif
static uint32_t tick;

static void run(int n, int active)
{
	for (int i = 0; i < n; i++) {
		flightrec_rec_t s;

		memset(&s, 0, sizeof(s));
		s.t_us = tick * 1000u;           /* 1 ms ticks */
		s.seq = (uint8_t)tick;
#if FLIGHTREC_RIG
		s.rig_state = active ? 3 : 0;
		s.gain_seq = (uint16_t)gseq;
		s.roll = 0.001f * (float)(tick % 200);
		s.duty[0] = active ? 5000 : 0;
#else
		s.duty[0] = active ? 0.5f : 0.0f;
#endif
		fake_ms = tick++;
		flightrec_tick(&s, active);
	}
}

#if FLIGHTREC_RIG
static void gain(unsigned idx, float v)
{
	gains[idx] = v;
	gseq++;
	flightrec_rig_gain(gseq, idx, v, tick * 1000u);
}
#endif

static void dump(const char *dir, const char *name, const void *p, size_t n)
{
	char path[512];
	FILE *f;

	snprintf(path, sizeof(path), "%s/%s", dir, name);
	f = fopen(path, "wb");
	fwrite(p, 1, n, f);
	fclose(f);
}

int main(int argc, char **argv)
{
	(void)argc;
	flightrec_init();
#if FLIGHTREC_RIG
	for (unsigned i = 0; i < FLIGHTREC_RIG_GAINS; i++) {
		gains[i] = (float)(i + 1);
	}
	flightrec_rig_bind(gains, &gseq);
	run(3000, 0); gain(0, 6.5f); run(2000, 1); run(3000, 0);    /* segment 1 */
	run(5000, 0); gain(2, 45.0f); gain(5, 0.9f); run(1000, 0);   /* gains set before arming */
	run(2000, 1); gain(1, 0.5f); run(1000, 1); run(3000, 0);     /* segment 2, one change in it */
#else
	run(3000, 0); run(2000, 1); run(3000, 0);
	run(6000, 0); run(3000, 1); run(3000, 0);
#endif
	/* The addresses the host stub serves, as the ELF would give them. */
	g_flightrec_hdr_addrs:
	{
		struct flightrec_header *h = (struct flightrec_header *)&g_flightrec_hdr;

		h->samples_addr = SMP_ADDR;
		h->pre_addr = PRE_ADDR;
	}
	dump(argv[1], "hdr.bin", &g_flightrec_hdr, sizeof(g_flightrec_hdr));
	dump(argv[1], "smp.bin", g_flightrec_samples, sizeof(g_flightrec_samples));
	dump(argv[1], "pre.bin", g_flightrec_pre, sizeof(g_flightrec_pre));
	return 0;
}
"""

SIM_KERNEL_H = """#include <stdint.h>
extern int64_t fake_ms;
static inline int64_t k_uptime_get(void) { return fake_ms; }
#define compiler_barrier() __asm__ volatile("" ::: "memory")
#define BUILD_ASSERT(c, ...) _Static_assert(c, "" __VA_ARGS__)
"""


class FlightrecHostSim(unittest.TestCase):
    """flightrec.c built for the host with stub Zephyr headers, both layouts: two segments,
    the second's preroll from the idle ring, then the dump read back through fetch()."""
    CAP, PREROLL, DIV = 4096, 500, 4

    def _sim(self, tmp: Path, rig: bool) -> dict[int, bytes]:
        cc = shutil.which("gcc") or shutil.which("cc")
        if cc is None:
            self.skipTest("host C compiler unavailable")
        (tmp / "zephyr/sys").mkdir(parents=True)
        (tmp / "zephyr/kernel.h").write_text(SIM_KERNEL_H)
        (tmp / "zephyr/sys/util.h").write_text("#define MIN(a, b) ((a) < (b) ? (a) : (b))\n")
        (tmp / "sim.c").write_text(SIM_C.replace("g_flightrec_hdr_addrs:\n", ""))
        exe = tmp / "sim"
        cmd = [cc, "-std=c11", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
               f"-I{tmp}", f"-I{FR_DIR / 'include'}", f"-DROSE_RIG={int(rig)}",
               f"-DFLIGHTREC_CAPACITY={self.CAP}u", f"-DFLIGHTREC_PREROLL={self.PREROLL}u",
               f"-DFLIGHTREC_DIV={self.DIV}u", f"-DSMP_ADDR={SMP_ADDR:#x}u",
               f"-DPRE_ADDR={PRE_ADDR:#x}u", str(tmp / "sim.c"),
               str(FR_DIR / "src/flightrec.c"), "-o", str(exe)]
        r = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        r = subprocess.run([str(exe), str(tmp)], capture_output=True, text=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        return {HDR_ADDR: (tmp / "hdr.bin").read_bytes(),
                SMP_ADDR: (tmp / "smp.bin").read_bytes(),
                PRE_ADDR: (tmp / "pre.bin").read_bytes()}

    def _check_segment(self, F, rows: list[dict], res: dict) -> None:
        self.assertEqual(res["gaps"], 0)
        t = [int(r["t_us"]) for r in rows]
        self.assertEqual(sorted(set(b - a for a, b in zip(t, t[1:]))), [1000 * self.DIV])
        motor = [i for i, r in enumerate(rows) if float(r["duty0"]) > 0]
        self.assertEqual(float(rows[motor[0]]["t_s"]), 0.0)
        self.assertEqual(motor[0], self.PREROLL)       # a full preroll from the idle ring
        self.assertEqual(len(motor), 3000 // self.DIV)
        self.assertTrue(res["frozen"])

    def test_v2_scenario(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            mem = self._sim(Path(d), rig=False)
            F = _import_fetch(_stub_link(mem))
            res = F.fetch("socket://stub", "stub.elf", Path(d) / "rec", log=lambda _m: None)
            with open(res["csv"], newline="") as f:
                rows = list(csv.DictReader(f))
        self.assertEqual(len(mem[HDR_ADDR]), 64)
        self.assertEqual((res["layout"], res["gains"]), (F.LAYOUT_FLIGHT, None))
        self._check_segment(F, rows, res)

    def test_rig_scenario(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            mem = self._sim(Path(d), rig=True)
            F = _import_fetch(_stub_link(mem))
            res = F.fetch("socket://stub", "stub.elf", Path(d) / "rec", log=lambda _m: None)
            with open(res["csv"], newline="") as f:
                rows = list(csv.DictReader(f))
            with open(res["gains"]) as f:
                gains = json.load(f)
            blob = Path(res["bin"]).read_bytes()
        self.assertEqual(len(mem[HDR_ADDR]), F.RIG_HDR_SIZE)
        self.assertEqual(res["layout"], F.LAYOUT_RIG)
        self.assertEqual(len(blob), F.RIG_HDR_SIZE + res["samples"] * F.RIG_SAMPLE_SIZE)
        self._check_segment(F, rows, res)
        defaults = {n: float(i + 1) for i, n in enumerate(F.RIG_GAIN_NAMES[:F.RIG_N_GAINS])}
        snap = dict(defaults, ATT_P=6.5, RATE_P=45.0, ATT_GAIN=0.9)
        self.assertEqual(gains["snapshot"], {"gain_seq": 3, "segment": 2, "gains": snap})
        self.assertEqual([(c["gain_seq"], c["name"], c["value"]) for c in gains["changes"]],
                         [(1, "ATT_P", 6.5), (2, "RATE_P", 45.0), (3, "ATT_GAIN", 0.9),
                          (4, "ATT_I", 0.5)])
        motor = [r for r in rows if float(r["duty0"]) > 0]
        self.assertEqual({r["rs_name"] for r in motor}, {"RUN"})
        self.assertEqual(rows[0]["gain_seq"], "1")       # preroll: before the pre-arm changes
        self.assertEqual(motor[0]["gain_seq"], "3")
        self.assertEqual(rows[-1]["gain_seq"], "4")
        g = F.gains_at(gains, 4)
        self.assertEqual(g, dict(snap, ATT_I=0.5))
        g1 = F.gains_at(gains, 1)
        self.assertEqual(g1["ATT_P"], 6.5)
        self.assertIsNone(g1["RATE_P"])                  # its default was never a change
        # The roll the sim wrote (0.001 rad * tick % 200) came through as degrees.
        t0 = int(rows[0]["t_us"]) // 1000
        self.assertAlmostEqual(float(rows[0]["roll_deg"]),
                               math.degrees(0.001 * (t0 % 200)), places=3)


if __name__ == "__main__":
    unittest.main()
