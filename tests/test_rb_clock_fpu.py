"""The clock and FPU Kconfig rb derives for a shell, and the post-link check.

hardware/zephyr/fpga.conf hardcodes the 50 MHz shells' 50 kHz CLINT timebase
and CONFIG_FPU=y. The accelerator shells run at 35-40 MHz, and Fp16At35's
scalar FPU implements only Zfh, so rb appends generated fragments after
fpga.conf and then disassembles the linked image. These tests pin the
derivation, its call site in zephyr_build, and the wrapper around
isa_contract_offences() (whose classification tests/test_accel_socs.py covers).

Pure host tests: the build is never run, and objdump is replaced by fixture
listings. RealShellTests reads collected DTS files when RB_DTS_DIR provides
them and skips otherwise.
"""

from __future__ import annotations

import contextlib
import io
import os
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

from tools.rb import cli
from tools.rb.boards import BOARDS, TARGETS, WORKLOADS
from tools.rb.cli import isa_contract_offences

FP16 = "RocketArty200TDroneGemminiSaturnFp16At35Config"
INT_NOPERM = "RocketArty200TDroneGemminiSaturnIntAt35Config"
GEMMINI40 = "RocketArty200TDroneGemmini16At40Config"
FULLDDR = "RocketArty200TDroneFullDDRConfig"


def _dts(timebase: int | None = 50_000, isa: str = "rv64imafdc_zicsr") -> str:
    """A devicetree carrying just what the derivation reads."""
    timebase_line = (
        f"\t\ttimebase-frequency = <{timebase}>;\n" if timebase is not None else ""
    )
    return (
        "/dts-v1/;\n"
        "/ {\n"
        "\tcpus {\n"
        f"{timebase_line}"
        "\t\tcpu@0 {\n"
        "\t\t\tdevice_type = \"cpu\";\n"
        f"\t\t\triscv,isa = \"{isa}\";\n"
        "\t\t};\n"
        "\t};\n"
        "};\n"
    )


def _settings(path: Path) -> list[str]:
    return [
        line for line in path.read_text().splitlines()
        if line and not line.startswith("#")
    ]


class _Scratch(unittest.TestCase):
    """A temp DTS and a temp GENERATED_CONF_DIR, so nothing touches .rb/."""

    def setUp(self) -> None:
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.dts = self.root / "shell.dts"
        conf_dir = patch.object(cli, "GENERATED_CONF_DIR", self.root / "generated")
        conf_dir.start()
        self.addCleanup(conf_dir.stop)

    def board(self, config: str = GEMMINI40):
        return SimpleNamespace(name="arty200t", config=config, generated_dts=self.dts)


class TimebaseTests(_Scratch):
    def test_reads_timebase_frequency(self) -> None:
        self.dts.write_text(_dts(35_000))
        self.assertEqual(cli.dts_timebase_hz(self.board()), 35_000)

    def test_missing_dts_is_none_not_a_guess(self) -> None:
        self.assertIsNone(cli.dts_timebase_hz(self.board()))

    def test_dts_without_the_property_is_none(self) -> None:
        self.dts.write_text(_dts(None))
        self.assertIsNone(cli.dts_timebase_hz(self.board()))


class ClockConfTests(_Scratch):
    def test_fifty_mhz_shell_gets_no_fragment(self) -> None:
        # fpga.conf is already right there; a fragment would only change the
        # build-input stamp and force a pristine rebuild for nothing.
        self.dts.write_text(_dts(cli.FPGA_CONF_TIMEBASE_HZ))
        self.assertIsNone(cli.generated_clock_conf(self.board()))

    def test_no_dts_gets_no_fragment(self) -> None:
        self.assertIsNone(cli.generated_clock_conf(self.board()))

    def test_slow_shell_overrides_the_cycle_rate(self) -> None:
        for timebase in (35_000, 40_000):
            with self.subTest(timebase=timebase):
                self.dts.write_text(_dts(timebase))
                path = cli.generated_clock_conf(self.board())
                assert path is not None
                self.assertEqual(path.name, f"arty200t.{GEMMINI40}.clock.conf")
                self.assertEqual(
                    _settings(path), [f"CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC={timebase}"]
                )

    def test_unchanged_fragment_is_not_rewritten(self) -> None:
        # Rewriting identical content would bump the mtime and invalidate
        # Zephyr's Kconfig cache on every build.
        self.dts.write_text(_dts(40_000))
        path = cli.generated_clock_conf(self.board())
        assert path is not None
        os.utime(path, (1_000_000_000, 1_000_000_000))
        self.assertEqual(cli.generated_clock_conf(self.board()), path)
        self.assertEqual(path.stat().st_mtime, 1_000_000_000)


class FpuConfTests(_Scratch):
    def test_full_fpu_shell_gets_no_fragment(self) -> None:
        self.dts.write_text(_dts(40_000, "rv64imafdcbvzicsr_zve64d_zvl128b"))
        self.assertIsNone(cli.generated_fpu_conf(self.board("RocketArty200TDroneSaturn40Config")))

    def test_fp16_shell_turns_the_fpu_off(self) -> None:
        self.dts.write_text(_dts(35_000, "rv64imafczicsr_zifencei_zfh"))
        path = cli.generated_fpu_conf(self.board(FP16))
        assert path is not None
        self.assertEqual(path.name, f"arty200t.{FP16}.fpu.conf")
        self.assertEqual(_settings(path), ["CONFIG_FPU=n"])

    def test_fp16_fragment_never_enables_the_vector_extension(self) -> None:
        # CONFIG_RISCV_ISA_EXT_V puts a global "v" in -march, which GCC expands
        # to zve64d and so to d and f: hardware float again. Workloads that
        # want Saturn opt in themselves with CONFIG_RISCV_V_KERNEL_ONLY=y.
        self.dts.write_text(_dts(35_000, "rv64imafcbvzicsr_zifencei_zvl128b_zve64d_zvfh_zfh"))
        path = cli.generated_fpu_conf(self.board(FP16))
        assert path is not None
        self.assertEqual(_settings(path), ["CONFIG_FPU=n"])

    def test_fp16_shell_without_its_dts_still_turns_the_fpu_off(self) -> None:
        # The FPU fact comes from the config name, not the DTS (rocket-chip
        # writes "f" for any FPU), so a missing DTS must not re-enable fp32.
        path = cli.generated_fpu_conf(self.board(FP16))
        assert path is not None
        self.assertEqual(_settings(path), ["CONFIG_FPU=n"])


class ZephyrBuildWiringTests(_Scratch):
    """The fragments reach EXTRA_CONF_FILE after fpga.conf, and only when due.

    The periphery and PLIC checks are stubbed: they are exercised against real
    shells elsewhere, and here the DTS carries only the timebase.
    """

    def conf_files(self, config: str, timebase: int) -> list[str]:
        self.dts.write_text(_dts(timebase))
        board = BOARDS["arty200t"].with_config(config)
        periphery = cli.ShellPeriphery(has_i2c=True, uart_irq=2, has_drone=True)
        captured: list[list[str]] = []
        with (
            patch.object(type(board), "generated_dts", new=property(lambda _self: self.dts)),
            patch.object(cli, "elaborated_periphery", return_value=periphery),
            patch.object(cli, "validate_hardware_contract", return_value=[]),
            patch.object(cli, "zephyr_build_dir", return_value=self.root / "build"),
            # The real ELF path would make this pass or fail on whatever an
            # earlier real build left behind.
            patch.object(cli, "zephyr_elf", return_value=self.dts),
            patch.object(cli, "_check_isa_contract") as check,
            patch.object(cli, "run", side_effect=lambda command, **_: captured.append(
                [str(part) for part in command]
            )),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            cli.zephyr_build(
                TARGETS["arty200t"], WORKLOADS["bootup_check"], "default", "always",
                fpga_board=board,
            )
        self.checked = check.called
        (extra,) = [arg for arg in captured[0] if arg.startswith("-DEXTRA_CONF_FILE=")]
        return extra.removeprefix("-DEXTRA_CONF_FILE=").split(";")

    def test_fifty_mhz_build_inputs_are_unchanged(self) -> None:
        confs = self.conf_files(FULLDDR, 50_000)
        self.assertFalse(any(".rb" in conf for conf in confs), confs)
        self.assertIn(str(cli.FPGA_CONF), confs)
        self.assertFalse(self.checked)

    def test_slow_fp16_build_layers_both_fragments_over_fpga_conf(self) -> None:
        confs = self.conf_files(FP16, 35_000)
        fpga = confs.index(str(cli.FPGA_CONF))
        clock = next(i for i, c in enumerate(confs) if c.endswith(".clock.conf"))
        fpu = next(i for i, c in enumerate(confs) if c.endswith(".fpu.conf"))
        # Later fragments win in Zephyr's Kconfig merge.
        self.assertLess(fpga, clock)
        self.assertLess(fpga, fpu)
        self.assertIn("CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC=35000", Path(confs[clock]).read_text())
        # And the linked image is checked against the FPU it will run on.
        self.assertTrue(self.checked)

    def test_nopermute_build_gets_the_clock_and_the_check_but_keeps_its_fpu(self) -> None:
        confs = self.conf_files(INT_NOPERM, 35_000)
        self.assertTrue(any(c.endswith(".clock.conf") for c in confs), confs)
        self.assertFalse(any(c.endswith(".fpu.conf") for c in confs), confs)
        self.assertTrue(self.checked)

    def test_forty_mhz_gemmini_build_gets_only_the_clock(self) -> None:
        confs = self.conf_files(GEMMINI40, 40_000)
        self.assertTrue(any(c.endswith(".clock.conf") for c in confs), confs)
        self.assertFalse(any(c.endswith(".fpu.conf") for c in confs), confs)
        self.assertFalse(self.checked)


class RealShellTests(unittest.TestCase):
    """Against the collected or elaborated DTS files, when they are present."""

    EXPECTED_HZ = {
        FULLDDR: 50_000,
        GEMMINI40: 40_000,
        FP16: 35_000,
        INT_NOPERM: 35_000,
    }

    def test_timebase_matches_the_config_name(self) -> None:
        checked = 0
        for config, hz in self.EXPECTED_HZ.items():
            board = BOARDS["arty200t"].with_config(config)
            if not board.generated_dts.is_file():
                continue
            checked += 1
            self.assertEqual(cli.dts_timebase_hz(board), hz, config)
        if not checked:
            self.skipTest("no arty200t drone shell DTS present (set RB_DTS_DIR)")


# Trimmed `objdump -d --no-show-raw-insn` output in the layout a real Zephyr
# image produces: file header, section headers, and a call whose target is
# printed as <symbol>, which must not be taken for a function header.
CLEAN_LISTING = """
build/zephyr/zephyr.elf:     file format elf64-littleriscv


Disassembly of section rom_start:

0000000080000000 <__start>:
    80000000:	auipc	t0,0x0
    80000004:	addi	t0,t0,64
    80000008:	fence.i

0000000080001234 <rb_tof_array_init>:
    800018c6:	vsetivli	zero,9,e64,m8,ta,ma
    800018ca:	vle64.v	v8,(s1)
    800018cc:	vfirst.m	a0,v8
    800018d2:	vse64.v	v8,(s0)
    800018d6:	call	80004000 <__mulsf3>
    800018da:	fence.tso
"""

BAD_LISTING = CLEAN_LISTING + """
0000000080002000 <estimator_step>:
    80002000:	flw	fa5,0(a0)
    80002004:	fmadd.s	fa5,fa4,fa3,fa5
    80002008:	fmv.x.w	a0,fa5
    8000200c:	fcvt.s.h	fa0,fa1
    80002010:	flw	fa4,4(a0)

0000000080003000 <z_riscv_switch>:
    80003000:	frcsr	t0
    80003004:	fdiv.d	fa5,fa5,fa4
    80003008:	vfadd.vv	v1,v2,v3
    8000300c:	vmflt.vf	v0,v4,fa0
    80003010:	c.fldsp	fs0,8(sp)
"""


def _one(insn: str) -> str:
    return f"\n0000000080001000 <f>:\n    80001000:\t{insn}\n"


class Fp16ListingTests(unittest.TestCase):
    """isa_contract_offences() on real objdump layout and the wider mnemonic set."""

    def test_real_layout_clean_image_passes(self) -> None:
        self.assertEqual(isa_contract_offences(CLEAN_LISTING, fp16=True, noperm=True), {})

    def test_real_layout_offences_are_grouped_by_function(self) -> None:
        found = isa_contract_offences(BAD_LISTING, fp16=True, noperm=False)
        self.assertEqual(found["scalar floating point"], {
            "estimator_step: flw": 2,
            "estimator_step: fmadd.s": 1,
            "estimator_step: fmv.x.w": 1,
            "estimator_step: fcvt.s.h": 1,
            "z_riscv_switch: frcsr": 1,
            "z_riscv_switch: fdiv.d": 1,
            "z_riscv_switch: c.fldsp": 1,
        })
        self.assertEqual(found["vector floating point"], {
            "z_riscv_switch: vfadd.vv": 1,
            "z_riscv_switch: vmflt.vf": 1,
        })

    def test_every_scalar_float_form_is_refused_on_an_fp16_shell(self) -> None:
        # Zfh included: CONFIG_FPU=n leaves mstatus.FS Off, and with FS Off
        # even the half-precision instructions this FPU implements trap.
        for mnemonic in ("fsd\tfa0,0(sp)", "c.fldsp\tfs0,8(sp)", "fsqrt.q\tfa0,fa1",
                         "fcvt.s.w\tfa0,a0", "fcvt.s.h\tfa0,fa1", "fmv.x.w\ta0,fa0",
                         "fmv.d.x\tfa0,a0", "fneg.s\tfa0,fa1", "feq.d\ta0,fa0,fa1",
                         "fclass.s\ta0,fa0", "fsflags\ta0", "fsrmi\t1",
                         "flh\tfa5,0(a0)", "fsh\tfa5,0(a0)", "fadd.h\tfa0,fa1,fa2",
                         "fcvt.w.h\ta0,fa0", "fmv.x.h\ta0,fa0", "fmv.h.x\tfa0,a0"):
            with self.subTest(mnemonic=mnemonic):
                found = isa_contract_offences(_one(mnemonic), fp16=True, noperm=False)
                self.assertEqual(list(found), ["scalar floating point"])

    def test_every_vector_float_form_is_refused_on_an_fp16_shell(self) -> None:
        for mnemonic in ("vfncvt.x.f.w\tv1,v2", "vfmv.f.s\tfa0,v1", "vmflt.vf\tv0,v4,fa0"):
            with self.subTest(mnemonic=mnemonic):
                found = isa_contract_offences(_one(mnemonic), fp16=True, noperm=False)
                self.assertEqual(list(found), ["vector floating point"])

    def test_integer_and_memory_ordering_are_not_float(self) -> None:
        for mnemonic in ("fence.tso", "vle64.v\tv8,(s1)", "vsetivli\tzero,9,e64,m8,ta,ma",
                         "call\t80004000 <__mulsf3>"):
            with self.subTest(mnemonic=mnemonic):
                self.assertEqual(isa_contract_offences(_one(mnemonic), fp16=True, noperm=True), {})


class IsaContractCheckTests(_Scratch):
    """_check_isa_contract, with objdump replaced by a fixture listing."""

    def check(self, config: str, listing: str) -> str:
        gnu = self.root / "gnu"
        gnu.mkdir(exist_ok=True)
        (gnu / "riscv64-zephyr-elf-objdump").write_text("")
        elf = self.root / "zephyr.elf"
        elf.write_bytes(b"\x7fELF")
        out = io.StringIO()
        with (
            patch.object(cli, "ZEPHYR_GNU_BIN", gnu),
            patch.object(cli.subprocess, "run", return_value=SimpleNamespace(stdout=listing)) as run,
            contextlib.redirect_stdout(out),
        ):
            try:
                cli._check_isa_contract(self.board(config), elf)
            finally:
                self.command = run.call_args.args[0]
        return out.getvalue()

    def test_clean_image_passes_and_says_what_was_checked(self) -> None:
        out = self.check(FP16, CLEAN_LISTING)
        self.assertIn("-d", self.command)
        self.assertIn(f"no instructions {FP16} decodes as illegal", out)
        self.assertIn("scalar and vector FP checked", out)

    def test_bad_image_is_refused_naming_the_shell_and_the_functions(self) -> None:
        with self.assertRaises(cli.RbError) as caught:
            self.check(FP16, BAD_LISTING)
        message = str(caught.exception)
        self.assertIn(FP16, message)
        self.assertIn("FP16-only FPU", message)
        self.assertIn("8 scalar floating point:", message)
        self.assertIn("2 vector floating point:", message)
        self.assertIn("estimator_step: flw", message)
        self.assertIn("z_riscv_switch: vmflt.vf", message)

    def test_nopermute_shell_accepts_the_scalar_fpu_it_has(self) -> None:
        listing = CLEAN_LISTING + _one("fadd.d\tfa0,fa0,fa1")
        out = self.check(INT_NOPERM, listing)
        self.assertIn("vector FP, permutes, vmv<n>r checked", out)


if __name__ == "__main__":
    unittest.main()
