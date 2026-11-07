"""Host checks for the esp_bridge crash-safety logic that has no hardware-free test otherwise.

Run with ``python3 -m unittest discover -s tests -p test_esp_safety.py -v``.

1. Builds tests/esp_safety_check.c against workloads/esp_bridge/src/esp_safety_core.h (the
   re-arm gate and the RTC record format) with the host compiler, as C and as C++, and runs it.
2. Checks that RBLINK lines carrying the reset/safety tail (appended after ``cl=``) still parse
   in every host parser -- tools/rb/fly.py, tools/rb/rig_tui.py (which rig_run.py uses),
   tools/rb/flightlog.py and tools/bench/riskybird-dash.py -- and that the new fields are read.

Not covered (needs the board): the fatal handler, the watchdog, LP SRAM retention, pad states.
"""

from pathlib import Path
import importlib.util
import shutil
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / "workloads/esp_bridge/src"
LINK = ROOT / "workloads/lib/motor_link/include"
DRIVER = Path(__file__).resolve().parent / "esp_safety_check.c"
WARN = ["-Wall", "-Wextra", "-Werror"]

sys.path.insert(0, str(ROOT))
from tools.rb import fly, flightlog, rig_tui  # noqa: E402

V1 = ("RBLINK v1 ml=up up=15777 seq=81 fl=0x21[ARMED COMMITTED ] duty=[0 0 0 0] fr=829 gap=5 "
      "lost=0 stale=0 crc=0 noise=51092 cuts=2 pwmerr=0 kill=0 txt=51092 cl=1")
TAIL = (" rearm=boot rablk=37 wdt=250ms rst=0x0c(sw-cpu) hw=0x40 bootn=7 rtc=kept"
        " prev_fault=wdt/motorlin/up91234 prev_up=91230 prev_duty=[7500 7441 6650 6744]"
        " prev_fl=0x21 prev_kill=0 prev_ml=up prev_hold=0")
# What the motors image sends: the v1 line, the PWM/spin-up fields, then the reset/safety tail.
LINE = V1 + " ph=1 ss=4444 sslim=12" + TAIL


def _load_dash():
    spec = importlib.util.spec_from_file_location("riskybird_dash",
                                                  ROOT / "tools/bench/riskybird-dash.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class EspSafetyCore(unittest.TestCase):
    _tmp = None

    @classmethod
    def setUpClass(cls):
        cls.cc = shutil.which("gcc") or shutil.which("cc")
        cls.cxx = shutil.which("g++") or shutil.which("c++")
        cls._tmp = tempfile.TemporaryDirectory()

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def _run(self, compiler, lang):
        if compiler is None:
            self.skipTest("no host compiler")
        exe = Path(self._tmp.name) / f"esp_safety_{lang}"
        cmd = [compiler, *(["-x", "c++", "-std=c++17"] if lang == "c++" else ["-std=c11"]),
               "-O2", *WARN, f"-I{CORE}", f"-I{LINK}", str(DRIVER), "-o", str(exe)]
        built = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(built.returncode, 0, f"{' '.join(cmd)}\n{built.stderr}")
        ran = subprocess.run([str(exe)], capture_output=True, text=True, timeout=60)
        self.assertEqual(ran.returncode, 0, ran.stderr)
        self.assertIn("ok boot_needs_disarm_arm_edge", ran.stdout)
        self.assertIn("ok kill_fc_never_disarmed", ran.stdout)
        self.assertIn("ok records", ran.stdout)
        self.assertIn("ok failsafe_cut_resumes_by_default", ran.stdout)
        self.assertIn("ok failsafe_cut_needs_edge_when_enabled", ran.stdout)

    def test_c(self):
        self._run(self.cc, "c")

    def test_cxx(self):
        self._run(self.cxx, "c++")


class FirmwareSettings(unittest.TestCase):
    """Source-level pins on choices the C test cannot see (no board, no Zephyr)."""

    SRC = ROOT / "workloads/esp_bridge"

    def test_rearm_after_cut_defaults_off(self):
        cmake = (self.SRC / "CMakeLists.txt").read_text()
        self.assertRegex(cmake, r"set\(ESP_LINK_REARM_AFTER_CUT 0 CACHE")
        main = (self.SRC / "src/main.c").read_text()
        self.assertIn("#define ESP_LINK_REARM_AFTER_CUT 0", main)
        self.assertIn("esp_rearm_on_cut(&g_rearm, ESP_LINK_REARM_AFTER_CUT)", main)
        self.assertNotIn("esp_rearm_hold(&g_rearm, ESP_REARM_CUT)", main)

    def test_watchdog_starts_at_first_drivable_frame(self):
        main = (self.SRC / "src/main.c").read_text()
        body = main[main.index("static bool handle_frame("):main.index("static void link_thread(")]
        self.assertIn("esp_safety_wdt_start()", body)
        self.assertEqual(main.count("= esp_safety_wdt_start();"), 1)
        # after the re-arm gate and the ARMED/BLOCK refusal, i.e. only when a gate may be driven
        self.assertLess(body.index("esp_rearm_frame("), body.index("esp_safety_wdt_start()"))
        self.assertLess(body.index("MOTOR_LINK_FLAGS_BLOCK) != 0u"), body.index("esp_safety_wdt_start()"))

    def test_persist_sessions_refused_while_motors_run(self):
        src = (self.SRC / "src/load_link.c").read_text()
        body = src[src.index("static void session("):]
        self.assertLess(body.index("esp_bridge_motors_recently_on()"), body.index("persist_session("))

    def test_flash_erase_only_under_a_granted_hold(self):
        # persist.c is the only ESP flash writer (no NVS/settings/PHY-cal storage in the build).
        src = (self.SRC / "src/persist.c").read_text()
        self.assertEqual(src.count("flash_area_erase("), 1)
        self.assertEqual(src.count("flash_area_write("), 1)
        erase = src[src.index("static int erase_guarded("):src.index("static int write_guarded(")]
        # the erase is inside the branch that needs the 1 s rule AND the granted hold
        self.assertRegex(erase, r"if \(!esp_bridge_motors_recently_on\(\) && esp_safety_wdt_hold\(true\)\)"
                                r" \{\s*rc = flash_area_erase\(")
        self.assertIn("esp_safety_wdt_hold(false);", erase)
        write = src[src.index("static int write_guarded("):]
        write = write[:write.index("\n}\n")]
        self.assertLess(write.index("esp_bridge_motors_recently_on()"), write.index("flash_area_write("))
        safety = (self.SRC / "src/esp_safety.c").read_text()
        start = safety[safety.index("int esp_safety_wdt_start(void)\n{"):]
        self.assertLess(start.index("return -EAGAIN;"), start.index("wdt_setup("))
        service = safety[safety.index("void esp_safety_wdt_service(bool motors_on)\n{"):]
        service = service[:service.index("\n}\n")]
        self.assertNotIn("wdt_started)) {\n\t\treturn;", service)   # grants before the start too
        main = (self.SRC / "src/main.c").read_text()
        svc = main[main.index("static void motors_service("):]
        self.assertIn("esp_safety_wdt_held()", svc[:svc.index("\n}\n")])

    def test_failsafe_cut_does_not_reset_the_spinup_gate(self):
        main = (self.SRC / "src/main.c").read_text()
        link = main[main.index("static void link_thread("):]
        cut = link[link.index("if (g_link_up && now >= deadline) {"):]
        cut = cut[:cut.index("}")]
        self.assertIn("motors_cut(", cut)
        self.assertNotIn("motors_off()", cut)
        body = main[main.index("static bool handle_frame("):main.index("static void link_thread(")]
        self.assertLess(body.index("ss_resume(&g_ss"), body.index("motors_set_target(cmd->duty)"))
        off = main[main.index("static void motors_off(void)"):]
        self.assertIn("ss_reset(&g_ss);", off[:off.index("\n}\n")])

    def test_motors_conf_drops_exception_dump(self):
        conf = (self.SRC / "conf/motors.conf").read_text()
        self.assertRegex(conf, r"(?m)^CONFIG_EXCEPTION_DEBUG=n$")
        self.assertRegex(conf, r"(?m)^CONFIG_WDT_DISABLE_AT_BOOT=y$")


class RblinkTail(unittest.TestCase):
    """The tail is additive: every parser that read the v1 line reads it unchanged."""

    def test_fly(self):
        base, full = fly.parse_rblink(V1), fly.parse_rblink(LINE)
        for key, value in base.items():
            self.assertEqual(full[key], value, key)
        self.assertEqual(full["up"], 15777)        # not prev_up
        self.assertEqual(full["kill"], 0)          # not prev_kill
        self.assertEqual(full["duty"], [0, 0, 0, 0])   # not prev_duty
        self.assertEqual(full["flags"], "ARMED COMMITTED")
        self.assertEqual((full["rearm"], full["rst"], full["bootn"], full["rtc"]),
                         ("boot", 0x0C, 7, "kept"))
        self.assertEqual(full["prev_up"], 91230)
        self.assertNotIn("rearm", base)

    def test_rig_tui(self):
        base, full = rig_tui.parse_rblink(V1), rig_tui.parse_rblink(LINE)
        self.assertIsNotNone(full)
        for key, value in base.items():
            self.assertEqual(full[key], value, key)
        self.assertEqual((full["rearm"], full["rst"], full["bootn"]), ("boot", 0x0C, 7))
        self.assertNotIn("rearm", base)

    def test_dash_regex(self):
        dash = _load_dash()
        self.assertEqual(dash.RBLINK_RE.search(LINE).groups(), dash.RBLINK_RE.search(V1).groups())

    def test_flightlog(self):
        _, rb = flightlog._parse(["1800000000.000 " + LINE + "\n"])
        d = rb[0][2]
        self.assertEqual((d["up"], d["kill"], d["duty0"], d["fl"]), (15777, 0, 0, "ARMED COMMITTED"))
        self.assertEqual((d["bootn"], d["prev_up"], d["prev_kill"]), (7, 91230, 0))
        self.assertNotIn("rst", d)    # "rst=0x0c" is not a number; never misread as 0

    def test_default_build_line_is_not_v1(self):
        line = "RBLINK none (no motor path in this image) rst=0x01(por/bod) hw=0x8 bootn=1 rtc=cold"
        self.assertIsNone(fly.parse_rblink(line))
        self.assertIsNone(rig_tui.parse_rblink(line))


if __name__ == "__main__":
    unittest.main()
