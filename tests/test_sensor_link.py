"""Host checks for the ESP -> FPGA sensor relay wire format.

Run with ``python3 -m unittest discover -s tests -p test_sensor_link.py -v``.
Builds tests/sensor_link_check.c against workloads/lib/motor_link/include/riskybird/sensor_link.h
and workloads/flight_controller/src/fc_uart_rx.h with the host compiler, as C and as C++.

Covered: the frame layout and CRC against an independent Python encoder; frames and command
lines interleaved on one stream; that a frame broken by a bad CRC, a bad type or lost bytes never
leaks a byte into the line assembler (even when its payload holds printable text and newlines);
resynchronisation on the next frame; sequence-gap accounting; payload scaling and clamping; and a
randomised stream where every line handed out must be one that was sent. No Zephyr: the ISR, the
drivers and the ESP's sensor reads are not exercised here.
"""

from pathlib import Path
import random
import shutil
import struct
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
FC_SRC = ROOT / "workloads/flight_controller/src"
LINK_INC = ROOT / "workloads/lib/motor_link/include"
DRIVER = Path(__file__).resolve().parent / "sensor_link_check.c"
WARN = ["-Wall", "-Wextra", "-Werror"]

MAGIC = 0xA6
LENS = {1: 12, 2: 6, 3: 3, 4: 2}


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(ftype, seq, payload):
    assert len(payload) == LENS[ftype]
    body = bytes([ftype, seq & 0xFF]) + payload
    return bytes([MAGIC]) + body + struct.pack("<H", crc16(body))


def _build(compiler, out, name, lang):
    exe = out / name
    cmd = [compiler, *(["-x", "c++", "-std=c++17"] if lang == "c++" else ["-std=c11"]),
           "-O2", *WARN, f"-I{FC_SRC}", f"-I{LINK_INC}", str(DRIVER), "-o", str(exe)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise AssertionError(f"{' '.join(cmd)}\n{result.stderr}")
    return exe


class SensorLink(unittest.TestCase):
    _tmp = None

    @classmethod
    def setUpClass(cls):
        cc = shutil.which("gcc") or shutil.which("cc")
        cxx = shutil.which("g++") or shutil.which("c++")
        if cc is None or cxx is None:
            raise unittest.SkipTest("host C/C++ compiler unavailable")
        cls._tmp = tempfile.TemporaryDirectory(prefix="sensor-link-test-")
        out = Path(cls._tmp.name)
        cls.exe_c = _build(cc, out, "check_c", "c")
        cls.exe_cxx = _build(cxx, out, "check_cxx", "c++")

    @classmethod
    def tearDownClass(cls):
        if cls._tmp is not None:
            cls._tmp.cleanup()

    def run_exe(self, *args, exe=None):
        result = subprocess.run([str(exe or self.exe_c), *args], capture_output=True, text=True,
                                timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        return result.stdout

    def demux(self, *tokens, exe=None):
        args = [t if isinstance(t, str) else t.hex() for t in tokens]
        out = self.run_exe("demux", *args, exe=exe)
        frames, lines, counts = [], [], None
        for row in out.splitlines():
            if row.startswith("F "):
                _, t, s, p = (row.split(" ") + [""])[:4]
                frames.append((int(t), int(s), bytes.fromhex(p)))
            elif row.startswith("L "):
                lines.append(row[2:])
            elif row.startswith("C "):
                counts = {k: int(v) for k, v in (kv.split("=") for kv in row[2:].split())}
        return frames, lines, counts

    def test_crc_check_value(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)   # CRC-16/CCITT-FALSE catalogue value

    def test_encode_matches_reference(self):
        rng = random.Random(1112)
        for exe in (self.exe_c, self.exe_cxx):
            for t, n in LENS.items():
                for _ in range(20):
                    seq, payload = rng.randrange(256), bytes(rng.randrange(256) for _ in range(n))
                    out = self.run_exe("encode", str(t), str(seq), payload.hex(), exe=exe)
                    self.assertEqual(out.strip(), "E " + frame(t, seq, payload).hex())
            self.assertEqual(self.run_exe("encode", "9", "0", "00", exe=exe).strip(), "E -")

    def test_frames_and_lines_interleave(self):
        imu = frame(1, 7, bytes(range(12)))
        tof = frame(3, 1, b"\x2c\x01\x00")
        stream = b"RESET\n" + imu + b"HB\n" + tof + imu + b"COLL 250\n"
        for exe in (self.exe_c, self.exe_cxx):
            frames, lines, c = self.demux(stream, exe=exe)
            self.assertEqual(lines, ["RESET", "HB", "COLL 250"])
            self.assertEqual([f[0] for f in frames], [1, 3, 1])
            self.assertEqual(frames[0][2], bytes(range(12)))
            self.assertEqual((c["frames"], c["crc"], c["bad"]), (3, 0, 0))

    def test_payload_text_never_reaches_line_assembler(self):
        # A payload made of "ESTOP\n" bytes: whole, it is a frame; broken, it must not be a command.
        payload = b"ESTOP\nHB\nABC"
        good = frame(1, 0, payload)
        frames, lines, _ = self.demux(good + b"HB\n")
        self.assertEqual((len(frames), lines), (1, ["HB"]))
        bad = bytearray(good)
        bad[-1] ^= 0xFF                                   # CRC fails
        frames, lines, c = self.demux(bytes(bad) + b"HB\n" + frame(2, 0, b"\x01" * 6) + b"HB\n")
        self.assertNotIn("ESTOP", lines)
        self.assertEqual(lines, ["HB"])                   # the line inside the hunt is dropped
        self.assertEqual((c["crc"], c["frames"]), (1, 1))

    def test_lost_bytes_mid_frame(self):
        good = frame(1, 3, b"ESTOP\nESTOP\n")
        frames, lines, c = self.demux(good[:5], "LOST", good[9:], b"HB\n",
                                      frame(4, 0, b"\x00\x08"), b"PING\n")
        self.assertEqual(lines, ["PING"])
        self.assertEqual(frames, [(4, 0, b"\x00\x08")])
        self.assertEqual(c["lost"], 1)

    def test_bad_type_resyncs(self):
        frames, lines, c = self.demux(bytes([MAGIC, 9, 0, 0x41, 0x0A]) + b"XY\n"
                                      + frame(3, 2, b"\x10\x00\x00") + b"HB\n")
        self.assertEqual((lines, len(frames), c["type"]), (["HB"], 1, 1))

    def test_magic_inside_payload_is_data(self):
        payload = bytes([MAGIC] * 12)
        frames, _, c = self.demux(frame(1, 0, payload), frame(1, 1, payload))
        self.assertEqual([f[2] for f in frames], [payload, payload])
        self.assertEqual(c["gaps"], 0)

    def test_sequence_gaps(self):
        p = bytes(12)
        _, _, c = self.demux(frame(1, 250, p), frame(1, 251, p), frame(1, 255, p), frame(1, 2, p),
                             frame(3, 9, b"\x00\x00\x00"), frame(3, 10, b"\x00\x00\x00"))
        self.assertEqual(c["gaps"], 3 + 2)                # 252-254, then 0-1 across the wrap

    def test_scaling_round_trip_and_clamp(self):
        out = self.run_exe("pack", "9.81", "-0.5", "0.001", "1.234", "-17.5", "0.0004",
                           "101.325", "23.45").split()
        vals = [float(v) for v in out[1:]]
        self.assertAlmostEqual(vals[0], 9.81, delta=0.002)
        self.assertAlmostEqual(vals[1], -0.5, delta=0.002)
        self.assertAlmostEqual(vals[3], 1.234, delta=0.001)
        self.assertAlmostEqual(vals[4], -17.5, delta=0.001)
        self.assertAlmostEqual(vals[6], 101.325, delta=0.00002)
        self.assertAlmostEqual(vals[7], 23.45, delta=0.01)
        out = self.run_exe("pack", "200", "-200", "0", "99", "-99", "0", "0", "0").split()
        vals = [float(v) for v in out[1:]]
        self.assertAlmostEqual(vals[0], 32767 * 0.002, places=3)   # clamped, not wrapped
        self.assertAlmostEqual(vals[1], -32768 * 0.002, places=3)
        self.assertAlmostEqual(vals[3], 32.767, places=3)

    def test_random_streams_never_forge_a_line(self):
        rng = random.Random(4242)
        verbs = [b"HB", b"RESET", b"PING", b"COLL 300", b"GAIN ATT_P 1.25", b"ESTOP"]
        for trial in range(150):
            parts, sent_lines, sent_frames = [], set(), []
            for _ in range(rng.randrange(1, 40)):
                if rng.random() < 0.4:
                    v = rng.choice(verbs)
                    sent_lines.add(v.decode())
                    parts.append(v + b"\n")
                else:
                    t = rng.choice(list(LENS))
                    f = frame(t, rng.randrange(256),
                              bytes(rng.choice(b"\x0aABESTOP \x00\xa6") for _ in range(LENS[t])))
                    sent_frames.append(f)
                    if rng.random() < 0.15:              # corrupt some frames
                        f = bytearray(f)
                        f[rng.randrange(1, len(f))] ^= 1 << rng.randrange(8)
                        f = bytes(f)
                    parts.append(f)
            tokens = []
            for p in parts:
                if rng.random() < 0.05 and len(p) > 3:   # occasionally lose bytes mid-part
                    cut = rng.randrange(1, len(p) - 1)
                    tokens += [p[:cut], "LOST", p[cut + 1:]]
                else:
                    tokens.append(p)
            frames, lines, _ = self.demux(*[t for t in tokens if t != b""])
            for line in lines:
                self.assertIn(line, sent_lines, f"trial {trial}: forged line {line!r}")
            good = {f for f in sent_frames}
            for t, s, p in frames:
                self.assertIn(frame(t, s, p), good, f"trial {trial}: forged frame")


if __name__ == "__main__":
    unittest.main()
