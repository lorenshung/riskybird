"""Host checks for the rig build's interrupt-driven uart1 receive path.

Run with ``python3 -m unittest discover -s tests -p test_fc_uart_rx.py -v``.
Builds tests/fc_uart_rx_check.c against workloads/flight_controller/src/fc_uart_rx.h with the host
compiler, as C and as C++, and once with a 16-entry ring to force overruns. Covered: the line rules
uart_cmd_poll() applied to polled bytes (compared against a reference of the old loop), the
back-to-back burst that was lost on hardware, overrun accounting and the rule that the two sides
of a gap are never joined, and a two-thread producer/consumer stress run. No Zephyr: this does not
check the ISR, the driver or the IRQ wiring.
"""

from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "workloads/flight_controller/src"
DRIVER = Path(__file__).resolve().parent / "fc_uart_rx_check.c"
WARN = ["-Wall", "-Wextra", "-Werror"]


def _build(compiler, out, name, lang, defines=()):
    exe = out / name
    cmd = [compiler, *(["-x", "c++", "-std=c++17"] if lang == "c++" else ["-std=c11"]),
           "-O2", *WARN, *defines, f"-I{SRC}", str(DRIVER), "-o", str(exe), "-pthread"]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise AssertionError(f"{' '.join(cmd)}\n{result.stderr}")
    return exe


def _old_polled_reader(data):
    """The pre-ISR uart_cmd_poll() line rules, byte for byte."""
    out, buf, discarding = [], bytearray(), False
    for ch in data:
        if ch in (0x0A, 0x0D):
            if buf and not discarding:
                out.append(buf.decode())
            buf, discarding = bytearray(), False
        elif ch < 0x20 or ch > 0x7E:
            buf, discarding = bytearray(), False
        elif discarding:
            pass
        elif len(buf) < 79:
            buf.append(ch)
        else:
            buf, discarding = bytearray(), True
    return out


class FcUartRx(unittest.TestCase):
    _tmp = None

    @classmethod
    def setUpClass(cls):
        cc = shutil.which("gcc") or shutil.which("cc")
        cxx = shutil.which("g++") or shutil.which("c++")
        if cc is None or cxx is None:
            raise unittest.SkipTest("host C/C++ compiler unavailable")
        cls._tmp = tempfile.TemporaryDirectory(prefix="fc-uart-rx-test-")
        out = Path(cls._tmp.name)
        cls.exe_c = _build(cc, out, "check_c", "c")
        cls.exe_cxx = _build(cxx, out, "check_cxx", "c++")
        cls.exe_small = _build(cc, out, "check_small", "c", ["-DFC_RX_RING_SIZE=16u"])

    @classmethod
    def tearDownClass(cls):
        if cls._tmp is not None:
            cls._tmp.cleanup()

    def feed(self, *chunks, exe=None):
        args = [str(exe or self.exe_c), "feed", *(c.hex() for c in chunks)]
        result = subprocess.run(args, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        lines, counts = [], None
        for row in result.stdout.splitlines():
            if row.startswith("L "):
                lines.append(row[2:])
            elif row.startswith("C "):
                counts = {k: int(v) for k, v in (kv.split("=") for kv in row[2:].split())}
        return lines, counts

    def test_back_to_back_gain_burst_all_delivered(self):
        # The hardware failure: 7 GAIN lines a few ms apart, one applied. Here all 7 land in the
        # ring before the loop drains it once.
        burst = b"".join(b"GAIN ATT_P %d.%02d\n" % (i, 10 * i) for i in range(7))
        burst += b"RESET\nCOLL 250\n"
        for exe in (self.exe_c, self.exe_cxx):
            lines, c = self.feed(burst, exe=exe)
            self.assertEqual(lines, _old_polled_reader(burst))
            self.assertEqual(len(lines), 9)
            self.assertEqual(c, {"ok": 9, "bad": 0, "ovr": 0, "bytes": len(burst)})

    def test_matches_old_polled_rules(self):
        rng = random.Random(1104)
        alphabet = b"ABCZ019 .-_\n\r\x00\x01\x1f\x7f\x80\xff"
        for _ in range(300):
            data = bytes(rng.choice(alphabet) for _ in range(rng.randrange(0, 300)))
            if rng.random() < 0.3:   # some overlong lines
                data += b"G" * rng.randrange(70, 90) + b"\n" + data
            chunks, i = [], 0
            while i < len(data):     # arbitrary drain points must not matter
                step = rng.randrange(1, 40)
                chunks.append(data[i:i + step])
                i += step
            lines, c = self.feed(*chunks) if chunks else self.feed(b"")
            self.assertEqual(lines, _old_polled_reader(data), data)
            self.assertEqual(c["ovr"], 0)
            self.assertEqual(c["ok"], len(lines))

    def test_noise_resyncs_and_counts_cut_lines(self):
        lines, c = self.feed(b"\x00\xffESTOP\n")
        self.assertEqual((lines, c["bad"]), (["ESTOP"], 0))   # junk alone drops no line
        lines, c = self.feed(b"GA\x01ESTOP\r\n")
        self.assertEqual((lines, c["ok"], c["bad"]), (["ESTOP"], 1, 1))

    def test_overlong_line_dropped_whole(self):
        ok79, long80 = b"A" * 79, b"B" * 80
        lines, c = self.feed(ok79 + b"\n" + long80 + b"TAIL\nHB\n")
        self.assertEqual(lines, ["A" * 79, "HB"])
        self.assertEqual((c["ok"], c["bad"]), (2, 1))

    def test_overrun_never_joins_head_and_tail(self):
        # 16-entry ring: "HB\n" x5 + "E" fit, "XC" are dropped; "STOP\n" arrives later. The old
        # rules would read "ESTOP" -- or, had the head been longer, "EXCSTOP" -> "STOP".
        lines, c = self.feed(b"HB\n" * 5 + b"EXC", b"STOP\n", b"HB\n", exe=self.exe_small)
        self.assertEqual(lines, ["HB"] * 6)
        self.assertEqual((c["ovr"], c["bad"]), (2, 1))

    def test_overrun_that_ate_the_newline_drops_the_next_line_too(self):
        lines, c = self.feed(b"HB\n" * 5 + b"X\n", b"STOP\nHB\n", exe=self.exe_small)
        self.assertEqual(lines, ["HB"] * 6)   # "X" and "STOP" both dropped: the safe error
        self.assertEqual((c["ovr"], c["bad"]), (1, 1))

    def test_two_thread_stress(self):
        n = 2_000_000
        for exe in (self.exe_c, self.exe_cxx, self.exe_small):
            result = subprocess.run([str(exe), "stress", str(n)], capture_output=True,
                                    text=True, timeout=120)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            row = dict(kv.split("=") for kv in result.stdout.split()[1:])
            self.assertEqual(int(row["kept"]) + int(row["ovr"]), n)
            self.assertEqual(int(row["bytes"]), n)


if __name__ == "__main__":
    unittest.main()
