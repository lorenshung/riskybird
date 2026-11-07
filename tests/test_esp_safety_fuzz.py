"""Randomized stress tests for workloads/esp_bridge/src/esp_safety_core.h.

Run with ``python3 -m unittest tests.test_esp_safety_fuzz -v`` (or pytest).

Builds tests/esp_safety_fuzz_check.c as C and as C++ and runs it once per area:

* rearm    esp_rearm_* against an independent, position-based statement of the re-arm rule,
           over random flag bytes (every bit), kill toggles, holds and failsafe cuts with
           ESP_LINK_REARM_AFTER_CUT 0 and 1; plus a pass-level model of main.c link_thread()
           (kill epoch at the top of each pass, ESTOP/RESET only between passes, ESTOP+RESET
           in one gap) checking the user-level rules.
* records  esp_rec_seal/esp_rec_valid for the boot/snap/fault/kill layouts: single-bit and
           single-word corruption, random multi-word corruption, the same high bit flipped in
           two words, torn writes in the firmware's store order, esp_snap_state() packing.

One test per invariant; a failure prints the driver's first counterexample. Knobs:
RB_FUZZ_SEED (default 0x5EED) and RB_FUZZ_SCALE (iteration multiplier, default 1).
"""

from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / "workloads/esp_bridge/src"
LINK = ROOT / "workloads/lib/motor_link/include"
DRIVER = Path(__file__).resolve().parent / "esp_safety_fuzz_check.c"
WARN = ["-Wall", "-Wextra", "-Werror"]

SEED = int(os.environ.get("RB_FUZZ_SEED", "0x5EED"), 0)
SCALE = max(1, int(os.environ.get("RB_FUZZ_SCALE", "1")))
REARM_SEQUENCES = 60_000 * SCALE
RECORD_TRIALS = 1_000_000 * SCALE


def _build(compiler, out, name, lang):
    exe = out / name
    cmd = [compiler, *(["-x", "c++", "-std=c++17"] if lang == "c++" else ["-std=c11"]),
           "-O2", *WARN, f"-I{CORE}", f"-I{LINK}", str(DRIVER), "-o", str(exe)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise AssertionError(f"{' '.join(cmd)}\n{result.stderr}")
    return exe


def _run(exe, *args):
    result = subprocess.run([str(exe), *map(str, args)], capture_output=True, text=True,
                            timeout=300)
    if result.returncode != 0:
        raise AssertionError(f"{exe} {' '.join(map(str, args))} -> {result.returncode}\n"
                             f"{result.stderr}")
    viol, first, stat = {}, {}, {}
    for line in result.stdout.splitlines():
        kind, _, rest = line.partition(" ")
        name, _, value = rest.partition(" ")
        if kind == "VIOL":
            viol[name] = int(value)
        elif kind == "FIRST":
            first[name] = value
        elif kind == "STAT":
            stat[name] = int(value)
    return result.stdout, viol, first, stat


class EspSafetyCoreFuzz(unittest.TestCase):
    _tmp = None

    @classmethod
    def setUpClass(cls):
        cc = shutil.which("gcc") or shutil.which("cc")
        cxx = shutil.which("g++") or shutil.which("c++")
        if cc is None or cxx is None:
            raise unittest.SkipTest("host C/C++ compiler unavailable")
        cls._tmp = tempfile.TemporaryDirectory(prefix="esp-safety-fuzz-")
        out = Path(cls._tmp.name)
        cls.exe_c = _build(cc, out, "fuzz_c", "c")
        cls.exe_cxx = _build(cxx, out, "fuzz_cxx", "c++")
        cls.rearm = _run(cls.exe_c, "rearm", SEED, REARM_SEQUENCES)
        cls.records = _run(cls.exe_c, "records", SEED, RECORD_TRIALS)

    @classmethod
    def tearDownClass(cls):
        if cls._tmp is not None:
            cls._tmp.cleanup()

    def clean(self, run, invariant):
        _out, viol, first, _stat = run
        self.assertIn(invariant, viol, f"driver did not report {invariant}")
        self.assertEqual(viol[invariant], 0,
                         f"[seed {SEED:#x}] {invariant}: {viol[invariant]} violations; "
                         f"first: {first.get(invariant)}")

    # -- re-arm gate --------------------------------------------------------------------------

    def test_rearm_matches_the_rule(self):
        self.clean(self.rearm, "rearm_matches_rule")

    def test_rearm_never_drives_killed_or_unarmed(self):
        self.clean(self.rearm, "rearm_never_drives_killed_or_unarmed")

    def test_rearm_blocked_counter(self):
        self.clean(self.rearm, "rearm_blocked_counter")

    def test_link_passes_never_drive_while_latched(self):
        self.clean(self.rearm, "pass_no_drive_while_latched")

    def test_link_passes_estop_needs_a_disarm(self):
        # incl. ESTOP + RESET inside one 5 ms gap (the kill epoch)
        self.clean(self.rearm, "pass_estop_needs_disarm")

    def test_link_passes_boot_needs_a_disarm(self):
        self.clean(self.rearm, "pass_boot_needs_disarm")

    # -- RTC records ----------------------------------------------------------------------

    def test_sealed_record_is_valid(self):
        self.clean(self.records, "rec_sealed_is_valid")

    def test_every_single_bit_flip_is_rejected(self):
        self.clean(self.records, "rec_single_bit_flip_rejected")

    def test_every_single_word_replacement_is_rejected(self):
        self.clean(self.records, "rec_single_word_replace_rejected")

    def test_random_multiword_corruption_is_rejected(self):
        self.clean(self.records, "rec_random_multiword_rejected")

    def test_never_valid_as_another_record_type(self):
        self.clean(self.records, "rec_never_valid_as_other_type")

    def test_torn_writes_validate_only_as_old_or_new(self):
        self.clean(self.records, "rec_torn_write_old_new_or_invalid")

    def test_snap_state_packing_roundtrips(self):
        self.clean(self.records, "snap_state_roundtrip")

    def test_same_high_bit_in_two_words_is_rejected(self):
        """FAILS (finding R1): esp_rec_check() is word-wise FNV-1a with no final mix. A
        multiply by an odd constant only carries upward, so bit 31 flipped in any two
        non-magic words (or in one payload word and the check word) always cancels, and bit
        30 about half the time. Example: kill record KILLED 0 -> 0x80000000 plus check
        bit 31 validates (a spurious kill: fails safe); in the snap record the same pattern
        turns duty[1]/duty[3] high bits."""
        self.clean(self.records, "rec_highbit_multiword_rejected")

    def test_c_and_cxx_builds_agree(self):
        self.assertEqual(_run(self.exe_cxx, "rearm", SEED, REARM_SEQUENCES // 4)[0],
                         _run(self.exe_c, "rearm", SEED, REARM_SEQUENCES // 4)[0])
        self.assertEqual(_run(self.exe_cxx, "records", SEED, RECORD_TRIALS // 4)[0],
                         _run(self.exe_c, "records", SEED, RECORD_TRIALS // 4)[0])


if __name__ == "__main__":
    unittest.main()
