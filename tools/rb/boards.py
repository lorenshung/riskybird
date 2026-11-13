"""RiskyBird physical-board, Zephyr-target, and workload registries."""

from __future__ import annotations

import os
import re
from dataclasses import dataclass, replace
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
# Config-name markers for Chipyard configs whose main memory is the 32 KiB mbus scratchpad rather
# than the board's external memory. Matched as substrings, which is the same idiom the DroneLogic
# periphery check in cli.py uses. A new Arty config elaborated with
# WithArty100TTweaks/WithArty200TTweaks(ddr = false) must be named to contain one of these.
SCRATCHPAD_CONFIG_MARKERS = ("Spad", "CameraBringup")
# The KU040 configs that cannot follow that rule. WithKU040Tweaks defaults to ddr = false, the
# reverse of the Arty tweaks, so on this board the *unmarked* names are the scratchpad ones and
# their DDR4 counterparts carry the marker. A substring cannot express that -- "RocketKU040Config"
# is a prefix of "RocketKU040DDRConfig" -- so these are matched by exact name. Kept in sync with
# WithKU040Tweaks call sites in backends/chipyard/fpga/src/main/scala/ku040/Configs.scala; the
# comment in hardware/zephyr/ku040.overlay names the same set.
SCRATCHPAD_CONFIG_NAMES = frozenset({
    "RocketKU040Config",
    "RocketKU040OspiConfig",
    "RocketKU040DroneLogicConfig",
    "Q31Ws32x32AccGemminiSaturnV128D128Fp16NoMvinScaleNoLoopConvKU040Config",
    "NoCoresKU040Config",
})
SCRATCHPAD_MEMORY_SIZE = 0x8000
# Configs whose Rocket scalar FPU is FP16-only (WithRocketFPU16: fLen = 16).
#
# Listed by name for the same reason SCRATCHPAD_CONFIG_NAMES is: the fact cannot
# be read off anything else. The elaborated devicetree does NOT distinguish
# them -- rocket-chip emits "f" in riscv,isa for any FPU, so
# RocketArty200TDroneGemminiSaturnFp16At35Config advertises rv64imafc while its
# FPU implements only Zfh. A shell that claims FP32 it cannot execute is a trap
# rather than a limitation: Zephyr's RISC-V CMake puts "f" in -march whenever
# CONFIG_FPU=y (cmake/compiler/gcc/target_riscv.cmake), independent of
# CONFIG_FLOAT_HARD, so any fp32 arithmetic compiles to fadd.s and faults at
# run time with nothing in the build to warn about it.
#
# Keep in sync with the WithRocketFPU16 configs, direct or inherited through
# `new <Parent>`, in backends/chipyard/fpga/src/main/scala/arty200t/*.scala;
# tests/test_rb_clock_fpu.py checks it whenever Chipyard is checked out.
# ku040/Configs.scala also uses WithRocketFPU16; those shells are not listed
# here, so rb still builds them with CONFIG_FPU=y.
FP16_SCALAR_FPU_CONFIGS = frozenset({
    "RocketArty200TDroneGemminiSaturnFp16Config",
    "RocketArty200TDroneGemminiSaturnFp16At35Config",
    # Area and trace variants of the same shell (RbAreaExperiments*.scala,
    # TacitAccelConfigs.scala, TacitDropConfigs.scala). None is benched, but
    # each has the FP16-only FPU, so an image built for one needs the same
    # fragment and the same post-link check.
    "RocketArty200TDroneGemminiSaturnFp16NoLoopConvConfig",
    "RocketArty200TDroneGemminiSaturnFp16At35TacitConfig",
    "RocketArty200TDroneGemminiSaturnFp16At35Tacit256Config",
    "RocketArty200TDroneGemminiSaturnFp16At35TacitFullNoLoopConvConfig",
    "RocketArty200TDroneGemminiSaturnFp16TraceBaseConfig",
    "RocketArty200TDroneGemminiSaturnFp16At35TacitTrimConfig",
    "RocketArty200TDroneGemminiSaturnFp16At35TacitTrim1024Config",
})
# Shells whose Saturn is VectorParams.intOnlyParams.copy(noPermute = true):
# integer-only vector (zve64x -- no vector FP) with the PermuteUnit removed, so
# vslide*/vfslide*/vrgather*/vcompress and the whole-register moves vmv<n>r.v are
# not in Saturn's supported_ex_insns and decode as illegal instructions. GCC
# emits vmv<n>r.v on its own for register copies, so a kernel written without a
# single permute intrinsic can still contain one. rb scans every image built for
# these shells. Keep in sync with AccelConfigs.scala.
INT_NOPERMUTE_VECTOR_CONFIGS = frozenset({
    "RocketArty200TDroneGemminiSaturnIntConfig",
    "RocketArty200TDroneGemminiSaturnIntAt40Config",
    "RocketArty200TDroneGemminiSaturnIntAt35Config",
})
CHIPYARD = ROOT / "backends" / "chipyard"
# RiskyBird's Chipyard sources, compiled into the plain backend by the sbt
# plugin hardware/chipyard/project/RiskyBird.scala (linked in by
# tools/backend-patches).
CHIPYARD_OVERLAY = ROOT / "hardware" / "chipyard"
# Every Chipyard FPGA make goes through this wrapper around fpga/Makefile: it
# adds the arty200t/ku040 sub-projects and the rb-area/rb-impl flow.
CHIPYARD_FPGA_MAKEFILE = CHIPYARD_OVERLAY / "fpga" / "rb-fpga.mk"
ZEPHYR_BACKEND = ROOT / "backends" / "zephyr-chipyard-sw"
# The PyTorch -> quantized -> Zephyr/RISC-V compilation pipeline behind
# `rb modelblaster`. A backend in its own right rather than a submodule of the
# Zephyr one: nothing in that backend builds against it, and the only consumer
# of both is this repository.
MODELBLASTER = ROOT / "backends" / "modelblaster"
FPGA_ZEPHYR_BOARD = "chipyard_riscv64/rocketchip_virt_riscv64"
CONFIG_NAME = re.compile(r"[A-Za-z_][A-Za-z0-9_]*\Z")


@dataclass(frozen=True)
class Board:
    """A physical FPGA board and its default Chipyard configuration."""

    name: str
    sub_project: str
    config: str
    package: str
    harness: str
    memory_base: int
    memory_size: int
    physical_memory_size: int | None
    physical_memory_type: str | None
    physical_memory_evidence: str
    openfpgaloader_board: str | None
    vivado_device_pattern: str
    # Sizes a non-default config on this board may legally present at
    # memory_base. memory_size is what the default config presents and is what
    # the workload RAM check and `rb doctor` report; a --config override can
    # elaborate a different memory arrangement, and the DTS contract accepts
    # any of these rather than only the default.
    alternate_memory_sizes: tuple[int, ...] = ()

    @property
    def legal_memory_sizes(self) -> tuple[int, ...]:
        return (self.memory_size, *self.alternate_memory_sizes)

    def with_config(self, config: str | None) -> Board:
        """Return this physical board with a selected Chipyard config."""
        if config is None:
            return self
        if CONFIG_NAME.fullmatch(config) is None:
            raise ValueError(
                f"invalid Chipyard config {config!r}; expected a Scala class name"
            )
        return replace(self, config=config)

    @property
    def generated_name(self) -> str:
        return f"{self.package}.{self.harness}.{self.config}"

    @property
    def generated_dir(self) -> Path:
        return CHIPYARD / "fpga" / "generated-src" / self.generated_name

    @property
    def generated_dts(self) -> Path:
        """The elaborated devicetree for this board+config.

        Chipyard's own output is the source of truth, but building a workload
        does not require Chipyard -- only the DTS it produced. `rb` reads it to
        derive PLIC sources, the periphery a shell elaborated, and the memory
        map, none of which need a toolchain to consult.

        That distinction is what lets the bench tier exist at all: Vivado has no
        macOS release, so a Mac cannot elaborate, but it can build workloads and
        program the board against a DTS collected elsewhere. Prefer the local
        Chipyard output when it is present, otherwise fall back to a plain
        directory of collected DTS files laid out like the bitstream and ELF
        side-repositories -- <board>/<board>.<Config>.dts.

        Set RB_DTS_DIR to point that fallback somewhere other than <root>/dts.
        """
        generated = self.generated_dir / f"{self.generated_name}.dts"
        if generated.is_file():
            return generated
        collected = self.collected_dts
        if collected.is_file():
            return collected
        return generated

    @property
    def collected_dts(self) -> Path:
        """Where a DTS collected from another machine is looked for."""
        root = Path(os.environ.get("RB_DTS_DIR", ROOT / "dts"))
        return root / self.name / f"{self.name}.{self.config}.dts"

    @property
    def bitstream(self) -> Path:
        """The bitstream for this board+config.

        Same split as generated_dts, for the same reason: Chipyard's build
        output is the source of truth, but PROGRAMMING a board needs the file
        rather than the toolchain that made it. A host that cannot synthesise --
        a Mac, since Vivado has no macOS release -- programs from bitstreams
        built elsewhere.

        Prefer a local Chipyard build when one is present, otherwise fall back
        to a collected directory laid out the way tools/collect-bitstreams.py
        publishes it: <board>/<board>.<Config>.bit. Note that layout is what
        makes the fallback usable at all -- Chipyard names every bitstream after
        its harness, so a directory of Arty200THarness.bit files would be
        indistinguishable once moved.

        Set RB_BITSTREAM_DIR to point it somewhere other than <root>/bitstreams.

        rb_impl.bit COMES FIRST, and that ordering is the whole point. Two
        different flows in this repository produce a bitstream for the same
        config:

          make bitstream   Chipyard's own flow. Re-elaborates, so it discards
                           any gen-collateral attribute injection, and writes
                           <Harness>.bit.
          make rb-impl     riskybird/area.mk. Routes obj/post_synth.dcp -- the
                           netlist `rb area run --attrs dsp` measured, with
                           use_dsp already baked in -- and writes rb_impl.bit.

        The accelerator configs only fit with DSP injection, so rb_impl.bit is
        the real artifact for them and <Harness>.bit, if one is lying around
        from an earlier plain build, is a DIFFERENT DESIGN. Preferring the
        harness name meant `rb program` could silently load the uninjected build
        of a config whose recorded area and timing came from the injected one --
        a wrong bitstream that programs perfectly and is measured against the
        wrong numbers.
        """
        obj = self.generated_dir / "obj"
        for built in (obj / "rb_impl.bit", obj / f"{self.harness}.bit"):
            if built.is_file():
                return built
        collected = self.collected_bitstream
        if collected.is_file():
            return collected
        return obj / f"{self.harness}.bit"

    @property
    def collected_bitstream(self) -> Path:
        """Where a bitstream built on another machine is looked for."""
        root = Path(os.environ.get("RB_BITSTREAM_DIR", ROOT / "bitstreams"))
        return root / self.name / f"{self.name}.{self.config}.bit"

    @property
    def overlay(self) -> Path:
        """The devicetree overlay describing this board's memory to Zephyr.

        The size has to follow the selected config, not just the board: a
        scratchpad config presents 32 KiB where the DDR default presents the
        board's full external memory, and linking against the wrong one is the
        difference between a workload that fails at link time and one that runs
        off the end of the scratchpad on hardware.
        """
        if self.is_scratchpad_config:
            return ROOT / "hardware" / "zephyr" / f"{self.name}-scratchpad.overlay"
        return ROOT / "hardware" / "zephyr" / f"{self.name}.overlay"

    @property
    def is_scratchpad_config(self) -> bool:
        if self.config in SCRATCHPAD_CONFIG_NAMES:
            return True
        return any(marker in self.config for marker in SCRATCHPAD_CONFIG_MARKERS)

    @property
    def config_memory_size(self) -> int:
        """The memory size the selected config actually presents."""
        return SCRATCHPAD_MEMORY_SIZE if self.is_scratchpad_config else self.memory_size

    @property
    def openocd_target(self) -> Path:
        return ROOT / "hardware" / "openocd" / f"{self.name}.cfg"


# These remain the default RiskyBird FPGA configurations. Low-level Chipyard
# commands intentionally continue to use physical board names.
BOARDS = {
    "arty100t": Board(
        name="arty100t",
        sub_project="arty100t",
        config="RocketArty100TOspiConfig",
        package="chipyard.fpga.arty100t",
        harness="Arty100THarness",
        memory_base=0x80000000,
        memory_size=0x40000000,
        physical_memory_size=None,
        physical_memory_type=None,
        physical_memory_evidence="not recorded in the RiskyBird board registry",
        openfpgaloader_board="arty_a7_100t",
        vivado_device_pattern="xc7a100t",
        # The 32 KiB mbus scratchpad presented by the ...SpadConfig family
        # (WithArty100TTweaks ddr = false), which places no MIG at all.
        alternate_memory_sizes=(0x8000,),
    ),
    # Trenz TE0712 carrying the XC7A200T. Same carrier, same 484-ball footprint,
    # and all 484 package balls verified identical to the 100T on bank,
    # PIN_FUNC, clock-capability, VREF/VRP and diff-pair type -- so the pin
    # mapping, memory map and Zephyr overlays are shared with arty100t.
    "arty200t": Board(
        name="arty200t",
        sub_project="arty200t",
        config="RocketArty200TOspiConfig",
        package="chipyard.fpga.arty200t",
        harness="Arty200THarness",
        memory_base=0x80000000,
        memory_size=0x40000000,
        physical_memory_size=None,
        physical_memory_type=None,
        physical_memory_evidence="not recorded in the RiskyBird board registry",
        # openFPGALoader's te0712_8 profile does cover the 200T variant, which
        # was the open question here. `openFPGALoader -b te0712_8 --detect`
        # against this module reports `family artix a7 200t`, `model xc7a200`,
        # idcode 0x3636093 -- the same IDCODE the JTAG chain shifts out. The
        # 0.12.0 install also ships spiOverJtag_xc7a200tfbg484.bit.gz, the
        # bridge `-f` needs to reach the SPI flash (see
        # docs/ku040-openfpgaloader-spioverjtag.md, which records the KU040
        # case where that bridge is the thing that is missing).
        openfpgaloader_board="te0712_8",
        vivado_device_pattern="xc7a200t",
        alternate_memory_sizes=(0x8000,),
    ),
    "ku040": Board(
        name="ku040",
        sub_project="ku040",
        config="RocketKU040OspiDDRConfig",
        package="chipyard.fpga.ku040",
        harness="KU040Harness",
        memory_base=0x80000000,
        memory_size=0x80000000,
        physical_memory_size=0x80000000,
        physical_memory_type="DDR4",
        physical_memory_evidence=(
            "2 GiB wired as two 1 GiB DDR4 controllers (HP banks 44 and 46), "
            "joined into one contiguous region; place-and-route verified, "
            "never exercised on hardware"
        ),
        openfpgaloader_board=None,
        vivado_device_pattern="xcku040",
        # One DDR4 controller (WithKU040Tweaks ddrControllers = 1) and the
        # pre-DDR 32 KiB mbus scratchpad (ddr = false).
        alternate_memory_sizes=(0x40000000, 0x8000),
    ),
}


@dataclass(frozen=True)
class ZephyrTarget:
    """A Zephyr build/deploy target, independent of workload source."""

    name: str
    kind: str
    zephyr_board: str
    fpga_board_name: str | None = None
    console_baudrate: int = 115200
    evidence: str = "declared"

    @property
    def fpga_board(self) -> Board | None:
        if self.fpga_board_name is None:
            return None
        return BOARDS[self.fpga_board_name]

    @property
    def target_dir(self) -> Path:
        return ROOT / "hardware" / "zephyr" / "targets" / self.name

    @property
    def common_overlay(self) -> Path:
        return self.target_dir / "target.overlay"

    @property
    def common_conf(self) -> Path:
        return self.target_dir / "target.conf"

    def workload_overlay(self, workload: str, mode: str) -> Path | None:
        candidates = (
            self.target_dir / "workloads" / f"{workload}-{mode}.overlay",
            self.target_dir / "workloads" / f"{workload}.overlay",
            ROOT
            / "hardware"
            / "zephyr"
            / "targets"
            / self.kind
            / "workloads"
            / f"{workload}-{mode}.overlay",
            ROOT
            / "hardware"
            / "zephyr"
            / "targets"
            / self.kind
            / "workloads"
            / f"{workload}.overlay",
        )
        return next((path for path in candidates if path.is_file()), None)

    def workload_confs(self, workload: str, mode: str) -> tuple[Path, ...]:
        """The workload's base config, then the mode's, in that order.

        A mode config EXTENDS the base rather than replacing it. This used to
        return a single path with `<workload>-<mode>.conf` ahead of
        `<workload>.conf`, so adding one symbol for one mode silently dropped
        everything the base carried -- libc, C++, sensor and bus settings --
        and the failure surfaced as an undefined reference at link time.
        """
        roots = (
            self.target_dir / "workloads",
            ROOT / "hardware" / "zephyr" / "targets" / self.kind / "workloads",
        )
        found = []
        for name in (f"{workload}.conf", f"{workload}-{mode}.conf"):
            path = next((r / name for r in roots if (r / name).is_file()), None)
            if path is not None:
                found.append(path)
        return tuple(found)


TARGETS = {
    "arty100t": ZephyrTarget(
        name="arty100t",
        kind="fpga",
        zephyr_board=FPGA_ZEPHYR_BOARD,
        fpga_board_name="arty100t",
        evidence="generated and synthesized",
    ),
    "arty200t": ZephyrTarget(
        name="arty200t",
        kind="fpga",
        zephyr_board=FPGA_ZEPHYR_BOARD,
        fpga_board_name="arty200t",
        evidence="generated",
    ),
    "ku040": ZephyrTarget(
        name="ku040",
        kind="fpga",
        zephyr_board=FPGA_ZEPHYR_BOARD,
        fpga_board_name="ku040",
        evidence="generated and synthesized",
    ),
    "esp32c6": ZephyrTarget(
        name="esp32c6",
        kind="esp",
        zephyr_board="esp32c6_devkitc/esp32c6/hpcore",
        evidence="deployed outside this integration",
    ),
    # FireSim on the U250. Same Zephyr board as the FPGA targets -- the base
    # chipyard DTS already places ram0 at 0x80000000 and selects the HTIF
    # console, which is what FireSim's TSI bridge presents -- so this target
    # needs no devicetree overlay, only the Kconfig fragment at
    # hardware/zephyr/targets/firesim/target.conf.
    #
    # fpga_board_name is deliberately None. FireSim is not a physical board: it
    # has no Vivado part, no openFPGALoader name, and no resource budget, and
    # RiskyBird results stay constrained to KU040. Leaving it None also skips
    # zephyr_build's board block, so no KU040 memory-size check or
    # validate_hardware_contract call is applied to a machine they do not
    # describe.
    #
    # Note the hart count in target.conf must match the bitstream being run;
    # it is currently 1, for rb_u250_rocket.
    "firesim": ZephyrTarget(
        name="firesim",
        kind="firesim",
        zephyr_board=FPGA_ZEPHYR_BOARD,
        fpga_board_name=None,
        evidence="booted on the U250 via rb_u250_rocket",
    ),
    # Host-only RoSE co-simulation through the custom cycle-budgeted Spike
    # harness.  This executor does not use FireSim or an FPGA.
    "rose-spike": ZephyrTarget(
        name="rose-spike",
        kind="rose",
        zephyr_board="spike_riscv64",
        fpga_board_name=None,
        evidence="RoSE lockstep Spike ISA simulator (no FPGA)",
    ),
}


@dataclass(frozen=True)
class Workload:
    """A board-agnostic Zephyr application plus its supported execution modes."""

    name: str
    source_dir: Path
    modes: tuple[str, ...] = ("default",)
    default_mode: str = "default"
    targets: tuple[str, ...] = ("esp32c6",)
    summary: str = ""
    safety: str = "normal"
    blocked_reason: str | None = None
    minimum_ram_by_mode: tuple[tuple[str, int], ...] = ()
    # -D definitions this workload always builds with. Modes select overlays and
    # Kconfig fragments, which cannot reach a CMake cache variable -- so a build
    # knob like ROSE_FLOW or ROSE_ESTIMATOR_ONLY previously had to be passed by
    # hand through RB_CMAKE_ARGS, and a workload that needed one was a command
    # to remember rather than a thing you could ask for by name.
    cmake_defines: tuple[tuple[str, str], ...] = ()
    # Same idea, but selected by mode. A mode is the unit an operator actually
    # asks for ("--mode autoflight"), and the knobs that make a mode what it is
    # are CMake cache variables, not Kconfig -- so without this, a mode can
    # select overlays and .conf fragments but cannot set the one thing that
    # decides whether the arming gate is even compiled in. That left flight
    # builds as a pile of -D flags to remember, which is precisely how a build
    # ends up flying with a safety knob silently absent.
    cmake_defines_by_mode: tuple[tuple[str, tuple[tuple[str, str], ...]], ...] = ()
    # Preprocessor definitions by mode, appended to Zephyr's EXTRA_CPPFLAGS.
    # The counterpart of cmake_defines_by_mode for knobs that are a bare
    # `#ifndef X / #define X` in the source rather than a CMake cache variable
    # (the flight controller's T_CLIMB_MS, ATT_GAIN, TOF_STEP_REJECT_M, ...):
    # passed as -DX on the CMake line those do nothing at all.
    cppflags_by_mode: tuple[tuple[str, tuple[tuple[str, str], ...]], ...] = ()
    # A mode that is another mode plus knobs. It builds with its base's
    # overlays, .conf fragments and definitions, then its own on top -- so a
    # flight preset is `esp-motors` exactly, plus the values that flew.
    mode_bases: tuple[tuple[str, str], ...] = ()
    # One line per mode saying where its values come from (a preset file, the
    # trials it flew), printed at build time and recorded with the build.
    mode_notes: tuple[tuple[str, str], ...] = ()
    # What superseded this workload, if something did. A legacy workload still
    # builds -- main has images and docs that name it -- but `rb workloads`
    # marks it and every build prints the note as a warning, so a stale
    # bring-up image is never mistaken for the path that flies.
    legacy: str = ""
    # The same, for one mode of a workload that is otherwise current.
    mode_legacy: tuple[tuple[str, str], ...] = ()
    # Modes that build on fewer targets than the workload does. A mode listed
    # here builds only on the targets it names; unlisted modes build on all of
    # self.targets.
    mode_targets: tuple[tuple[str, tuple[str, ...]], ...] = ()
    # The config a board builds this workload against when --config is not
    # given, by board name. Every drone workload needs a drone shell, and the
    # board default (the OSPI config) is not one, so without this each of them
    # failed unless the operator remembered --config.
    default_configs: tuple[tuple[str, str], ...] = ()

    def base_mode(self, mode: str) -> str | None:
        return dict(self.mode_bases).get(mode)

    def layout_mode(self, mode: str) -> str:
        """The mode whose overlays and .conf fragments this mode builds with."""
        seen = {mode}
        while (base := self.base_mode(mode)) is not None and base not in seen:
            seen.add(base)
            mode = base
        return mode

    def _chain(self, mode: str) -> list[str]:
        chain = [mode]
        while (base := self.base_mode(chain[-1])) is not None and base not in chain:
            chain.append(base)
        return list(reversed(chain))   # base first, the mode itself last

    def defines_for(self, mode: str) -> tuple[tuple[str, str], ...]:
        """Workload-wide definitions, then the base mode's, then this mode's."""
        merged = dict(self.cmake_defines)
        for link in self._chain(mode):
            for selected_mode, defines in self.cmake_defines_by_mode:
                if selected_mode == link:
                    merged.update(defines)
        return tuple(merged.items())

    def cppflags_for(self, mode: str) -> tuple[tuple[str, str], ...]:
        merged: dict[str, str] = {}
        for link in self._chain(mode):
            for selected_mode, defines in self.cppflags_by_mode:
                if selected_mode == link:
                    merged.update(defines)
        return tuple(merged.items())

    def mode_note(self, mode: str) -> str:
        return dict(self.mode_notes).get(mode, "")

    def legacy_note(self, mode: str) -> str:
        """The mode's own legacy note, else the workload's, else ""."""
        return dict(self.mode_legacy).get(mode, "") or self.legacy

    def default_config(self, board_name: str) -> str | None:
        return dict(self.default_configs).get(board_name)

    def board_for(self, board: Board, config: str | None) -> Board:
        """The board with --config applied, or this workload's default for it."""
        return board.with_config(config if config is not None else self.default_config(board.name))

    def mode_conf(self, mode: str) -> Path | None:
        path = self.source_dir / "conf" / f"{self.layout_mode(mode)}.conf"
        return path if path.is_file() else None

    def mode_target_names(self, mode: str) -> tuple[str, ...]:
        return dict(self.mode_targets).get(mode, self.targets)

    def supports(self, target: ZephyrTarget, mode: str | None = None) -> bool:
        if target.name not in self.targets:
            return False
        return mode is None or target.name in self.mode_target_names(mode)

    def minimum_ram(self, mode: str) -> int:
        return next(
            (
                minimum
                for selected_mode, minimum in self.minimum_ram_by_mode
                if selected_mode == mode
            ),
            0,
        )


def _workload_dir(name: str) -> Path:
    """workloads/<name>, or workloads/bringup|probes/<name> for the grouped ones."""
    for group in ("", "bringup", "probes"):
        path = ROOT / "workloads" / group / name
        if path.is_dir():
            return path
    return ROOT / "workloads" / name


def _workload(
    name: str,
    *,
    modes: tuple[str, ...] = ("default",),
    default_mode: str = "default",
    targets: tuple[str, ...] = ("esp32c6",),
    summary: str,
    safety: str = "normal",
    blocked_reason: str | None = None,
    minimum_ram_by_mode: tuple[tuple[str, int], ...] = (),
    source_dir: Path | None = None,
    cmake_defines: tuple[tuple[str, str], ...] = (),
    cmake_defines_by_mode: tuple[
        tuple[str, tuple[tuple[str, str], ...]], ...
    ] = (),
    cppflags_by_mode: tuple[tuple[str, tuple[tuple[str, str], ...]], ...] = (),
    mode_bases: tuple[tuple[str, str], ...] = (),
    mode_notes: tuple[tuple[str, str], ...] = (),
    legacy: str = "",
    mode_legacy: tuple[tuple[str, str], ...] = (),
    mode_targets: tuple[tuple[str, tuple[str, ...]], ...] = (),
    default_configs: tuple[tuple[str, str], ...] = (),
) -> Workload:
    return Workload(
        name=name,
        source_dir=source_dir if source_dir is not None else _workload_dir(name),
        modes=modes,
        default_mode=default_mode,
        targets=targets,
        summary=summary,
        safety=safety,
        blocked_reason=blocked_reason,
        minimum_ram_by_mode=minimum_ram_by_mode,
        cmake_defines=cmake_defines,
        cmake_defines_by_mode=cmake_defines_by_mode,
        cppflags_by_mode=cppflags_by_mode,
        mode_bases=mode_bases,
        mode_notes=mode_notes,
        legacy=legacy,
        mode_legacy=mode_legacy,
        mode_targets=mode_targets,
        default_configs=default_configs,
    )


# General board diagnostics do not make sense on the RoSE bridge target.  The
# MPC workload opts into that target explicitly below.
ALL_TARGETS = tuple(name for name in TARGETS if name != "rose-spike")
# FireSim runs the base chipyard devicetree: a console, the CLINT and main
# memory, and no I2C, SPI, GPIO or PWM. A workload that touches a peripheral
# cannot even compile there -- its devicetree alias does not exist -- so only
# console- and memory-only workloads are registered for it.
PERIPHERAL_TARGETS = tuple(name for name in ALL_TARGETS if name != "firesim")
I2C_TARGETS = PERIPHERAL_TARGETS
# The drone carrier's shell, for workloads that need its periphery (the second
# UART, the sensor bus, the camera OSPI). RocketArty200TOspiConfig, the board
# default, has none of the drone periphery, so a drone workload built without
# --config failed on every arty200t build.
# The TinyMPC library the flight controller links. It stays in the Zephyr
# backend (samples/drone_control/tinympc, a submodule of zephyr-chipyard-sw).
FLIGHT_TINYMPC = ZEPHYR_BACKEND / "samples" / "drone_control" / "tinympc"
DRONE_DEFAULT_CONFIGS = (("arty200t", "RocketArty200TDroneFullDDRConfig"),)
# The same sentence for every mode that drives the motor pins from the FPGA.
FPGA_PWM_LEGACY = (
    "drives the FPGA PWM pins: motor 4's pin F13 is dead, and this fights the "
    "ESP for the motor gates when the ESP runs esp_bridge motors; the flying "
    "path is esp-motors + esp_bridge --mode motors"
)
MOTOR_N_LEGACY = (
    "superseded by flight_controller --mode esp-motors with esp_bridge --mode "
    "motors on the ESP (and motor_duty for a bench spin); it " + FPGA_PWM_LEGACY
)

# What `--mode flow` is for. The overlay alone names the bus (flow-spi = &spi0);
# the flow path itself is compiled in by ROSE_FLOW, and the PMW3901's chip-select
# on this carrier is gpio0 pin 0 rather than the ESP32-C6's 19 (see
# pmw3901_test.overlay). Without both, main.cpp prints "flow-spi alias present
# but ROSE_FLOW=0" and the mode builds an image with no flow in it -- which is
# what it did until these were declared here.
FLOW_DEFINES = (("ROSE_FLOW", "1"), ("CS_GPIO_PIN", "0"))


WORKLOADS = {
    # Console-only bring-up probe. Enabled for every target because it touches
    # no peripheral but the console UART, which is exactly what makes it the
    # first thing to run on a shell whose UART has never emitted a character.
    "hello_world": _workload(
        "hello_world",
        targets=ALL_TARGETS,
        summary=(
            "Console bring-up probe: banner, timed tick loop, and a "
            "completion marker a debugger can break on"
        ),
    ),
    "state": _workload(
        "state",
        modes=("replay", "bmi088"),
        default_mode="replay",
        targets=ALL_TARGETS,
        # Replay touches no peripheral; the BMI088 is on the I2C bus.
        mode_targets=(("bmi088", PERIPHERAL_TARGETS),),
        summary="Madgwick state estimator with replay or BMI088 input",
        legacy=(
            "superseded by flight_controller (and flight_estimator for the "
            "estimator alone); on a stationary board this Madgwick settles to "
            "roll -178.7 deg where the flight estimator reads 0.10"
        ),
    ),
    "mpc": _workload(
        "mpc",
        modes=("replay", "hil", "rose"),
        default_mode="replay",
        targets=tuple(TARGETS),
        # hil exchanges frames over a UART the FireSim console does not
        # provide.
        mode_targets=(("hil", tuple(name for name in TARGETS if name != "firesim")),),
        summary="TinyMPC replay, UART HIL, or RoSE lockstep controller",
        legacy=(
            "superseded by flight_controller, whose esp-motors mode flies the "
            "PID cascade; TinyMPC is unsafe on this airframe (see the "
            "esp-motors notes in tools/rb/boards.py)"
        ),
    ),
    "ads7128_test": _workload(
        "ads7128_test",
        targets=I2C_TARGETS,
        summary="ADS7128 register and GPIO diagnostic",
    ),
    "bmi088_test": _workload(
        "bmi088_test",
        targets=I2C_TARGETS,
        summary="BMI088 accelerometer and gyroscope diagnostic",
    ),
    # Whole-board sensor check plus a go/no-go for the state estimator. Needs the
    # drone periphery (I2C for the sensor bus, SPI + GPIO for the flow sensor)
    # and the camera OSPI, i.e. one of the full drone shells.
    "sensor_check": _workload(
        "sensor_check",
        targets=("arty200t",),
        summary="All on-board sensors in one pass, ending in a state-estimator go/no-go",
        minimum_ram_by_mode=(("default", 0x8000),),
        default_configs=DRONE_DEFAULT_CONFIGS,
        legacy="superseded by bootup_check, which covers every sensor here in boot order",
    ),
    # One pass over every peripheral a flight needs, in boot order, ending in a
    # board-ready verdict. Needs the drone periphery for the same reason
    # sensor_check does; `motors` additionally needs the battery, the ESP link
    # (serial@10021000) and esp_bridge --mode motors on the ESP: the sweep sends
    # motor_link duty frames, as the flight controller does, and drives no
    # FPGA pin.
    "bootup_check": _workload(
        "bootup_check",
        modes=("default", "motors"),
        targets=("arty200t",),
        summary="Every peripheral in one boot-order pass, ending in a board-ready verdict",
        # A frame is 78892 bytes and the image carries it as a static buffer, so
        # a 32 KB scratchpad shell cannot run this at all. 256 KB excludes them
        # by contract rather than at link time.
        minimum_ram_by_mode=(("default", 0x40000), ("motors", 0x40000)),
        default_configs=DRONE_DEFAULT_CONFIGS,
        cppflags_by_mode=(("motors", (("BOOTUP_MOTORS", "1"),)),),
    ),
    # The ESP link lives only in the full drone shells, which are arty200t-only
    # today. Requires a shell that elaborates serial@10021000; rb appends
    # fpga-esp-uart.overlay when it detects one.
    # The ESP side of the same link esp_uart proves from the FPGA side. Deliberately
    # a dumb pipe: the flight controller's telemetry line already matches the ground
    # station's v1.1 parser (verified, 500/500 captured lines), so the bridge has no
    # format to translate and does not need changing when telemetry gains fields.
    "esp_bridge": _workload(
        "esp_bridge",
        # `default` has no motor code at all (the four motor pins are hogged as
        # pulled-down inputs). `motors` is the image that flies: it also owns
        # the four gates, decodes the FPGA's duty frames, runs the 120 ms
        # failsafe and executes a dashboard ESTOP locally. See
        # docs/wireless-telemetry-bringup.md.
        modes=("default", "motors"),
        targets=("esp32c6",),
        summary=(
            "WiFi modem for the FPGA flight controller: UART <-> UDP "
            "telemetry :14550 and commands :14551 (--mode motors also drives "
            "the four motor gates)"
        ),
        # The collective duty ceiling applied at the gate, after everything the
        # FPGA did. 98 for flight: just above the flight controller's own 0.95
        # cap, so it backstops a mis-built FPGA image without ever being the
        # constraint that binds (T05-T27 flew with 90-98).
        cmake_defines_by_mode=(("motors", (("ESP_LINK_MAX_DUTY_PCT", "98"),)),),
    ),
    # The FPGA half of a thrust / motor-identification bench test: holds a
    # fixed duty on chosen motors by sending the same motor_link frames the
    # flight controller sends. Driven by `rb fly motors`.
    "motor_duty": _workload(
        "motor_duty",
        targets=("arty200t",),
        summary=(
            "Hold a fixed duty on chosen motors by sending motor_link frames "
            "to the ESP32-C6 (motor identification / thrust bench test)"
        ),
        safety="props-on-RESTRAINED (docs/flight-calibration-procedures.md)",
        default_configs=DRONE_DEFAULT_CONFIGS,
    ),
    "esp_uart": _workload(
        "esp_uart",
        targets=("arty200t",),
        summary="Second UART (ESP32 link on E13/F14): TX proof and RX rate sweep",
        minimum_ram_by_mode=(("default", 0x8000),),
        default_configs=DRONE_DEFAULT_CONFIGS,
    ),
    # The HM01B0 target exists on both Arty parts. The 200T entry is the drone
    # carrier's own DVP pinout (bank-13 B2B pins), which is a different set of
    # balls from the 100T's -- the config selects them, not this table.
    "camera_bringup": _workload(
        "camera_bringup",
        targets=("arty100t", "arty200t"),
        summary="HM01B0 I2C control and pixel-capture bring-up",
        minimum_ram_by_mode=(("default", 0x8000),),
        legacy=(
            "superseded by bootup_check (its camera stage) and camera_photo "
            "(one full frame) on the drone shell"
        ),
    ),
    # One photograph, for a shell where the camera is already proven. The RAM
    # floor is the point: a frame is 326*242 = 78892 bytes plus the image, so
    # this cannot be built against a 32 KiB scratchpad config, and the check
    # says so at build time rather than at the link error.
    # The flight stack that actually flew on the ESP32-C6, built here for the
    # FPGA. Its source_dir is workloads/flight_controller, this repository's own
    # fork of Dima's rose_flight_controller sample. The upstream original stays
    # in the Zephyr backend (samples/rose_flight_controller) as the RoSE sample;
    # rb never builds that copy, so RiskyBird changes land here and the backend
    # stays a clean upstream checkout.
    #
    # The application selects its subsystems from device-tree aliases
    # (DT_NODE_EXISTS(DT_ALIAS(flow/tof/baro))), so which sensors are compiled
    # in is decided by the FPGA overlay, not here -- which is what makes a
    # sensor-at-a-time bring-up possible. See docs/flight-on-fpga.md.
    # Dima's estimator, on its own. Same sources as flight_controller -- this is
    # a build variant, not a second copy -- with the controller compiled out, so a bench
    # run measures the estimator and nothing else. The stale workloads/state
    # Madgwick target is NOT this: on the same stationary board it settles to
    # roll -178.7 deg where this reports roll 0.10, so the two disagree by half a
    # turn and only one of them flies.
    "flight_estimator": _workload(
        "flight_estimator",
        source_dir=ROOT / "workloads" / "flight_controller",
        modes=("default", "flow"),
        targets=("arty200t",),
        summary="RoSE estimator alone: attitude, altitude and sensor validity, no controller",
        minimum_ram_by_mode=(("default", 0x80000),),
        # CONFIG_UART_SIFIVE_PORT_1 for the same reason flight_controller sets
        # it: on a shell with the ESP UART, main.cpp's uplink reader
        # (ROSE_UART_CMD, on by default whenever the esp-uart alias exists)
        # references uart1, and without the port the link fails with an
        # undefined __device_dts_ord_* in uart_cmd_reply.
        cmake_defines=(
            ("ROSE_ESTIMATOR_ONLY", "1"),
            ("CONFIG_UART_SIFIVE_PORT_1", "y"),
            ("TINYMPC", str(FLIGHT_TINYMPC)),
        ),
        cmake_defines_by_mode=(("flow", FLOW_DEFINES),),
        default_configs=DRONE_DEFAULT_CONFIGS,
    ),
    "flight_controller": _workload(
        "flight_controller",
        source_dir=ROOT / "workloads" / "flight_controller",
        # One mode per bring-up phase, each selecting a device-tree overlay that
        # adds one subsystem to the previous. The overlays #include each other
        # rather than restating the IMU rates and bus speed, so the phases
        # cannot drift apart. What a mode declares is what the application
        # compiles in -- `default` has no `motors` alias and therefore contains
        # no code path to a motor at all.
        modes=("default", "telem", "flow", "motors", "esp-motors", "autoflight"),
        targets=("arty200t",),
        summary="RoSE flight controller: estimator + control cascade + sensors",
        # Every full-drone-shell mode gets the esp-uart alias, and main.cpp's
        # guarded telemetry writer opens it while leaving the console on uart0.
        # The SiFive driver does not instantiate DT instance 1 from devicetree
        # alone, so omitting this makes non-telem modes fail at link time with
        # an undefined __device_dts_ord_* reference. flight_controller-telem.conf
        # repeats the setting so the console-moving mode remains self-contained.
        # TINYMPC: the top-level copy has no ../drone_control beside it, so it
        # is pointed at the Zephyr backend's TinyMPC submodule (generic, still
        # upstream there).
        cmake_defines=(("CONFIG_UART_SIFIVE_PORT_1", "y"), ("TINYMPC", str(FLIGHT_TINYMPC))),
        # Per-mode, because these are CMake cache variables: a .conf fragment
        # cannot reach them, and a mode that cannot set them is a mode whose
        # defining feature has to be remembered as a -D on the command line.
        # ROSE_AUTOFLIGHT is exactly that kind of knob -- passed the wrong way
        # it is silently ignored and the build falls through to an
        # always-armed bench path, which is the trap FLIGHT_BUILD.md warns
        # about and the reason this is declarative rather than documented.
        cmake_defines_by_mode=(
            ("flow", FLOW_DEFINES),
            (
                "telem",
                (
                    # CMake defaults to TinyMPC, which is not what flies and
                    # measured ~34 ms of loop time here; select the flight stack.
                    ("ROSE_USE_PID", "1"),
                    ("ROSE_USE_EKF", "1"),
                    # The flight build used ROSE_TELEM=0 because an undrained
                    # 115200 console busy-waits and stalls the control loop.
                    # This mode deliberately turns it on: the ESP bridge drains
                    # uart1 continuously and forwards each complete line.
                    ("ROSE_TELEM", "1"),
                    # Sized against the WIRE, not the loop. With the v1.1 tail
                    # (x/y/vx/vy/vz/zsp/vbat/st) the line is ~240 bytes rather
                    # than ~150, and printk busy-waits on this carrier because
                    # CONFIG_UART_INTERRUPT_DRIVEN is off -- so every byte over
                    # the 11.5 kB/s link is charged to the control loop. At the
                    # ~770 Hz the PID cascade actually achieves, a divisor of 30
                    # is ~25 Hz: about 6 kB/s, a bit over half the link. A
                    # divisor of 20 would be ~77% and would start stealing loop
                    # time to no benefit -- 25 Hz is already twice what a
                    # dashboard can usefully show.
                    ("ROSE_TELEM_DIV", "30"),
                    # Run until stopped. The sample defaults to 5000
                    # iterations, which was minutes of telemetry while the
                    # loop crawled at 29 Hz under TinyMPC and is SIX SECONDS
                    # at the ~770 Hz the PID cascade actually achieves -- the
                    # stream simply stopped, main() returned, and the core sat
                    # in the idle thread looking like a dead link. A bench
                    # telemetry mode has no flight log to flush, so the
                    # finite-iteration argument that applies to autoflight
                    # does not apply here.
                    ("CTRL_ITERS", "0"),
                ),
            ),
            (
                # Motors offloaded to the ESP32-C6. FPGA ball F13 -- motor4's
                # gate -- is dead as an OUTPUT: a bare-metal bitstream driving
                # the four pads at 4/8/12/16 % reads back M1=40 M2=80 M3=120
                # M4=0 per mille at the gates, while the same ball as an INPUT
                # reads the carrier's 10k pulldown. Net intact, driver dead, no
                # RTL repair. So the FPGA stops driving gates entirely and sends
                # duty frames to the ESP, which owns all four.
                #
                # It used to match `--mode motors` in every other respect, so
                # that the only variable under test was where the PWM is
                # generated. That A/B is finished: the link is proven -- 1212
                # frames, zero CRC/noise/loss, and all four motors cut 120 ms
                # after the core halts. What is left is the mode that actually
                # commands the drone, so it now declares the flight stack
                # instead of inheriting the sample's CMake defaults, which are
                # TinyMPC, no optical flow and a Crazyflie's mass. See the
                # per-knob notes below. `--mode motors` is deliberately left on
                # the defaults: it is a PWM-pin bring-up, not a flight mode.
                "esp-motors",
                (
                    ("ROSE_ESP_MOTORS", "1"),
                    # 200 Hz, not the CMake default of 50.
                    #
                    # 50 Hz is a 20 ms zero-order hold in the actuator. The
                    # inner rate loop's TAU_ROLLRATE is 0.025 s and
                    # FLIGHT_TUNING_LOG.md entries 6-7 put that loop ON its
                    # stability edge already (0.015 oscillates; 0.02 is "the
                    # known-stable inner loop"). Adding ~10 ms of effective
                    # delay to a 25 ms time constant is the most likely reason
                    # a faithful reproduction of the 2026-08-13 position-hold
                    # milestone would still fail to hold -- that flight
                    # actuated EVERY control iteration, with the FPGA driving
                    # the pins directly.
                    #
                    # 200 Hz aligns with the receiver's own 5 ms tick and costs
                    # 14 B x 200 = 2800 B/s, 24% of the 115200 link. 1000/200
                    # divides exactly, so the period is a clean 5 ms.
                    ("ROSE_ESP_MOTORS_HZ", "200"),
                    # ---- the one fact a REMOTE operator cannot otherwise get ----
                    #
                    # This mode is the one that commands the drone, and it is
                    # run over ssh with nobody in the room. The single question
                    # that decides whether it is safe to touch anything is "can
                    # a motor physically turn", and on this airframe that is
                    # exactly "is the pack connected": the motors are brushed,
                    # switched by low-side SI2302 FETs off +BATT, so with no
                    # pack the FETs gate a weakly cap-charged rail and nothing
                    # turns. There are no ESCs and no second energy source.
                    #
                    # vbat= has been in the telemetry line all along -- it is a
                    # field of the SHARED TELEM_TAIL_FMT that feeds both the
                    # console printk and the esp-uart radio mirror. It was
                    # simply always 0.000, because g_vbat is only ever written
                    # by battery_poll() and ROSE_BATT_SENSE defaults to 0 and
                    # no mode but autoflight set it. So this one line is the
                    # whole fix for observability: nothing needed adding to the
                    # format string.
                    ("ROSE_BATT_SENSE", "1"),
                    # ...but NOT as a control input, which is what
                    # ROSE_BATT_SENSE alone would mean. On its own it also
                    # scales every motor duty by BATT_NOMINAL_V/g_vbat and
                    # latches the emergency estop below BATT_CUTOFF_V. Both are
                    # right for a flight and wrong here, because the AIN5
                    # channel and the 200k/100k scaling have NEVER been checked
                    # against a meter on this carrier. main.cpp's guards reject
                    # a garbage reading (battery_poll() drops anything outside
                    # [0.5, 6.0] V; the estop needs >= 1.0 V; the scale needs
                    # [1.0, 5.0] V) -- but they do not and cannot reject a
                    # PLAUSIBLE WRONG one. A mis-scaled result in [1.0, 3.8) V
                    # boosts commanded duty by up to 30 % while telemetry still
                    # shows the duty that was asked for, and one in [1.0, 3.2) V
                    # estops a healthy bench run in a way that reads, remotely,
                    # exactly like a real fault.
                    #
                    # So: observability now, actuation after verification. Drop
                    # this line -- and only this line -- once someone has put a
                    # meter on the pack and confirmed it against vbat=. The boot
                    # banner says so on every run, so the promotion cannot be
                    # forgotten. `--mode autoflight` is unaffected: it does not
                    # set this, and keeps the full sag comp + cutoff + arm gate
                    # it deliberately turned on.
                    ("ROSE_BATT_REPORT_ONLY", "1"),
                    # The wire format is one header shared with
                    # workloads/esp_motors. The sample is built in place from a
                    # submodule and cannot guess where this checkout is, so the
                    # path is passed rather than assumed; its CMakeLists fails
                    # the build outright if it is missing.
                    (
                        "ROSE_MOTOR_LINK_DIR",
                        str(ROOT / "workloads" / "lib" / "motor_link"),
                    ),
                    # ---- the stack that has actually flown this airframe ----
                    #
                    # docs/FLIGHT_BUILD.md in the sample records the build used
                    # for the untethered hops as PID + EKF + flow, and `--mode
                    # telem` already declares the first two for the same reason.
                    # This mode inherited neither. Verified by symbol on the
                    # image this replaces: tiny_solve 1, HierarchicalPid 0.
                    #
                    # TinyMPC here is not merely "not what flies", it is unsafe.
                    # Its gains are baked for a ~30 g Crazyflie at 50 Hz while
                    # this airframe is ~60 g at ~71 % hover duty, and its solver
                    # gets 10 ADMM iterations -- nowhere near enough to satisfy
                    # its own input box constraint from a bench-sized altitude
                    # error. Measured on the host with this exact problem data:
                    # the returned command is 1.28 against a bound of 0.417. It
                    # also runs at ~35 ms/iteration against the PID cascade's
                    # ~1.3 ms, and the offload run that tilted to 90 degrees
                    # with nothing cutting the motors was the slow one (see
                    # docs/history/flight-safety-bench-procedure.md section 2.3).
                    ("ROSE_USE_PID", "1"),
                    # Already the CMakeLists default; stated anyway because the
                    # comment sitting directly above that default still claims
                    # the complementary filter is what gets selected. A default
                    # whose own documentation disagrees with it is one edit away
                    # from silently changing the estimator under a mode that
                    # commands motors.
                    #
                    # COMPLEMENTARY, NOT EKF, AND THAT IS NOT A PREFERENCE.
                    # The 2026-08-13 position-hold milestone (72b391a, ~1 m
                    # drift) records it as a requirement in tools/build.sh:
                    #
                    #   "NOTE: ROSE_USE_EKF=0 is REQUIRED -- the EKF's tof_gate
                    #    decouples altitude and climbs away."
                    #
                    # The gate rejects a ToF sample too far from the predicted
                    # state; once altitude has diverged, every subsequent GOOD
                    # sample is rejected too, so altitude decouples and the
                    # drone climbs away. This mode previously set 1 on the
                    # strength of docs/FLIGHT_BUILD.md -- but that file
                    # documents the EARLIER untethered-hop build, and its own
                    # "Known caveat" section describes precisely that
                    # climb-away as unfixed. tools/build.sh is the record of
                    # what actually held position.
                    ("ROSE_USE_EKF", "0"),
                    # Optical flow -> horizontal velocity into the estimator.
                    # FLIGHT_TUNING_LOG.md entry 15: the build that flew before
                    # this one had ROSE_FLOW=0, so horizontal velocity was pure
                    # accel dead-reckoning and drifted by construction.
                    #
                    # This takes THREE things that are easy to get individually
                    # right and collectively wrong, so all three are asserted
                    # here rather than left to the operator:
                    #
                    #  1. the bus, from devicetree --
                    #     flight_controller-esp-motors.overlay includes
                    #     flight_controller-flow.overlay, which sets
                    #     `flow-spi = &spi0` and enables spi0/gpio0.
                    #     flight_controller.conf already carries CONFIG_SPI and
                    #     CONFIG_GPIO for every mode.
                    #  2. the code, from this knob. Without it main.cpp emits
                    #     "flow-spi alias present but ROSE_FLOW=0" and compiles
                    #     no flow path at all.
                    #  3. the chip-select, below.
                    ("ROSE_FLOW", "1"),
                    # gpio0 pin 0, not the ESP32-C6's 19.
                    #
                    # The FPGA carrier's DroneLogic GPIO block is a different
                    # controller with its own numbering; pmw3901_test.overlay
                    # wires the flow CS to index 0, and that is the wiring the
                    # standalone PMW3901 bring-up reads the chip ID through. Get
                    # this wrong and every other part of the flow path is
                    # correct and useless: flow_init() drives the wrong pin, the
                    # chip-ID read fails, and after five 50 ms retries it prints
                    # "PMW3901 not detected -- flow disabled" and the estimator
                    # silently falls back to the dead-reckoning that entry 15
                    # was about. ~250 ms of boot spent proving nothing.
                    #
                    # Reachable only as of this change, and it took TWO fixes:
                    # flow.c's default is now #ifndef'd (a bare #define quietly
                    # won the argument against the -D that
                    # flight_controller-flow.overlay and docs/flight-on-fpga.md
                    # both tell the operator to pass), and CMakeLists.txt now
                    # declares CS_GPIO_PIN and lists it in
                    # target_compile_definitions, without which a -D of this
                    # name sets a cache variable nothing reads.
                    ("CS_GPIO_PIN", "0"),
                    # Mass feedforward for the PID altitude loop, which has no
                    # integrator -- so this MUST match the real weight or the
                    # loop cannot reach hover thrust. 0.060 kg is riskybird v3,
                    # measured; the source default is 0.032 kg, a Crazyflie.
                    #
                    # Reachable only because CMakeLists.txt now declares
                    # PID_MASS_KG as a cache variable AND lists it in
                    # target_compile_definitions. It previously existed solely
                    # as an #ifndef inside controller_pid.cpp, so a -D of this
                    # name was silently ignored and FLIGHT_BUILD.md had to smuggle
                    # it through EXTRA_CPPFLAGS -- exactly the remembered-flag
                    # trap this table exists to remove.
                    ("PID_MASS_KG", "0.060f"),
                    # Run until stopped. NOT an independent choice: it is forced
                    # by ROSE_USE_PID above. The sample's 5000-iteration default
                    # was ~175 s of bench time at TinyMPC's 35 ms/iter and is
                    # ~6.5 s at the PID cascade's 1.3 ms. Every step of
                    # docs/history/flight-safety-bench-procedure.md part A -- tilt the
                    # frame past 40 degrees by hand, drop-test free fall, halt
                    # the core and watch the ESP failsafe cut within 125 ms,
                    # wait out the 5 s commitment window -- needs the loop alive
                    # for minutes, and a committed flight test is given a 60 s
                    # motor budget that a 6.5 s run would end for it. What
                    # bounds motor time in this mode is the commitment gate (5 s
                    # uncommitted, 60 s committed) and the envelope guard,
                    # neither of which is an iteration count. Same reasoning
                    # `--mode telem` records for the same cause.
                    ("CTRL_ITERS", "0"),
                ),
            ),
            (
                "autoflight",
                (
                    # Compiles in the lift-and-place arm gate, the altitude
                    # profile and the FLIGHT_MAX_MS cap. Without it none of
                    # those exist and the build cannot arm at all.
                    ("ROSE_AUTOFLIGHT", "1"),
                    # Off by default upstream, which silently removes the
                    # BATT_ARM_MIN_V pre-flight gate, the BATT_CUTOFF_V
                    # in-flight estop and thrust sag compensation. On a 1S
                    # LiPo that is the difference between a controlled
                    # descent and a brownout mid-hover, so a flight mode
                    # turns it on rather than leaving it to the operator.
                    ("ROSE_BATT_SENSE", "1"),
                    # The only observability that survives an untethered
                    # flight: a background thread into RAM, flushed at the
                    # end, costing the control loop nothing.
                    ("ROSE_FLIGHTLOG", "1"),
                    # ~5 Hz instead of ~100. This console is a 115200 FTDI
                    # link in polling mode, so printk busy-waits; see the
                    # note in flight_controller-autoflight.conf.
                    ("ROSE_TELEM_DIV", "200"),
                    # Finite on purpose. CTRL_ITERS=0 runs forever, which
                    # suits a tethered estimator session but means the flight
                    # log is never flushed. ~60 s at the measured 2.97 ms
                    # loop: comfortably longer than FLIGHT_MAX_MS (10 s), so
                    # the cap that ends a flight is the flight timer and not
                    # an arbitrary iteration count.
                    ("CTRL_ITERS", "20000"),
                    # 50 mm, not the stock 20 mm. Measured on this board: a
                    # drone sitting flat on the bench reads 0.022 m on the
                    # down-ToF, so the stock threshold fails the "on the
                    # ground" test by 2 mm and the lift-and-place gesture can
                    # never complete -- the gate was set tighter than the
                    # sensor's own resting value. 50 mm keeps the check
                    # meaningful (it still refuses to arm while the drone is
                    # held at any real height) with margin for near-field
                    # jitter, which main.cpp already notes is unreliable when
                    # the sensor sits flush.
                    ("ARM_MAX_HEIGHT_M", "0.05f"),
                ),
            ),
        ),
        safety="propellers-off",
        minimum_ram_by_mode=(("default", 0x80000),),
        default_configs=DRONE_DEFAULT_CONFIGS,
        mode_legacy=(("motors", FPGA_PWM_LEGACY), ("autoflight", FPGA_PWM_LEGACY)),
    ),
    "camera_photo": _workload(
        "camera_photo",
        targets=("arty200t",),
        summary="Capture one full HM01B0 frame into RAM for JTAG readout",
        minimum_ram_by_mode=(("default", 0x20000),),
        default_configs=DRONE_DEFAULT_CONFIGS,
    ),
    # Main-memory stress. Registered for the FPGA targets only: it walks a
    # window at 0x8200_0000, which exists on a DDR-backed shell and on nothing
    # else -- a scratchpad config presents 32 KiB at 0x8000_0000 and the ESP32
    # is not in this address space at all. The 96 MiB floor is the 32 MiB the
    # window skips plus the 64 MiB default window; a larger --extra-cflags
    # window needs correspondingly more.
    "ddr_stress": _workload(
        "ddr_stress",
        targets=("arty100t", "arty200t", "ku040", "firesim"),
        summary="Main-memory stress: four patterns over a configurable window",
        minimum_ram_by_mode=(("default", 0x6000000),),
    ),
    # DroNet through ModelBlaster's harness. Built in place from the
    # ModelBlaster backend: the harness is maintained upstream and takes the generated
    # model as -DMODEL_DIR, so there is nothing for a workloads/dronet/
    # directory to hold that would not be a copy.
    #
    # `rb build --workload dronet` alone is not enough -- MODEL_DIR has to point
    # at codegen output that matches this board's accelerators. `rb modelblaster
    # run` is the entry point; it runs stages 1-3, then this.
    "dronet": _workload(
        "dronet",
        source_dir=MODELBLASTER / "harness",
        targets=("arty200t",),
        summary="DroNet inference through the ModelBlaster harness, with a per-op profile",
        # The harness alone asks for a 4 MB main stack
        # (modelblaster/harness/prj.conf), before DroNet's weights, activations
        # and the baked test io. No scratchpad shell can host this, and the
        # floor says so at build time instead of letting it surface as a linker
        # "region RAM overflowed" against a 32 KiB region.
        minimum_ram_by_mode=(("default", 0x800000),),
    ),
    "i2c_scanner": _workload(
        "i2c_scanner",
        targets=I2C_TARGETS,
        summary="I2C address scan",
    ),
    "motor_1": _workload(
        "motor_1",
        targets=PERIPHERAL_TARGETS,
        summary="One-motor PWM diagnostic",
        safety="propellers-off",
        legacy=MOTOR_N_LEGACY,
    ),
    "motor_2": _workload(
        "motor_2",
        targets=PERIPHERAL_TARGETS,
        summary="Two-motor PWM diagnostic",
        safety="propellers-off",
        legacy=MOTOR_N_LEGACY,
    ),
    "motor_3": _workload(
        "motor_3",
        targets=PERIPHERAL_TARGETS,
        summary="Three-motor PWM diagnostic",
        safety="propellers-off",
        legacy=MOTOR_N_LEGACY,
    ),
    "motor_4": _workload(
        "motor_4",
        targets=PERIPHERAL_TARGETS,
        summary="Four-motor interactive PWM diagnostic",
        safety="propellers-off",
        legacy=MOTOR_N_LEGACY,
    ),
    "motor_safe": _workload(
        "motor_safe",
        targets=("esp32c6",),
        summary=(
            "Park the shared ESP32-C6 motor pins as high-impedance inputs "
            "with pull-downs"
        ),
    ),
    # The other half of `flight_controller --mode esp-motors`. FPGA ball F13 is
    # dead as an OUTPUT -- a bare-metal bitstream driving the four pads at
    # 4/8/12/16 % reads back M1=40 M2=80 M3=120 M4=0 per mille, and F13 as an
    # input reads the carrier's 10k pulldown, so the net is intact and the
    # driver is not. Motor4 therefore cannot be driven from the FPGA at all.
    #
    # This moves ALL FOUR motors onto the ESP, which reaches every gate through
    # its own 47R, and leaves the FPGA driving none -- which also removes the
    # dual-driver contention on the other three gates rather than working around
    # one pin.
    #
    # Directly opposed to motor_safe, which parks the same four pins as
    # pulled-down inputs: flashing this REPLACES the gate probe that every motor
    # measurement in this investigation was made with.
    "esp_motors": _workload(
        "esp_motors",
        targets=("esp32c6",),
        summary=(
            "Drive all four motor gates from the FPGA's duty frames, with a "
            "receive-timeout failsafe"
        ),
        safety="propellers-off",
        legacy=(
            "superseded by esp_bridge --mode motors, which drives the same four "
            "gates with the same failsafe and also carries telemetry and the "
            "dashboard ESTOP"
        ),
    ),
    "pmw3901_test": _workload(
        "pmw3901_test",
        targets=PERIPHERAL_TARGETS,
        summary="PMW3901 SPI optical-flow diagnostic",
    ),
    "vl35l5cx_test": _workload(
        "vl35l5cx_test",
        modes=("address", "ranging"),
        default_mode="address",
        targets=I2C_TARGETS,
        summary=(
            "Four VL53L5CX sensors through ADS7128 GPIO1-4; portable "
            "address validation or full grid ranging"
        ),
        minimum_ram_by_mode=(("ranging", 0x30000),),
    ),
}


# ---- flight presets: the builds that flew, as data ------------------------------------------
#
# Every flight in the 2026-09-23/24 campaign was built as
#   RB_CMAKE_ARGS="..." RB_EXTRA_CPPFLAGS="..." rb build flight_controller --mode esp-motors
# and a flag silently dropped or sent down the wrong channel was the commonest failure (rb debug
# rebuilding without them; a -D a file never reads). Each file in tools/flight/presets/ records
# the exact values of one configuration that flew, and becomes a mode of flight_controller here:
# `esp-motors` plus those values. So the build is asked for by name --
#   rb build flight_controller --target arty200t --config RocketArty200TDroneFullDDRConfig \
#       --mode t21-hover-1.0m
# -- and the same file is what `rb flightlog push --build-file` records.
FLIGHT_PRESETS_DIR = ROOT / "tools" / "flight" / "presets"


def _parse_preset(path: Path) -> tuple[tuple[tuple[str, str], ...], tuple[tuple[str, str], ...], str]:
    """(CMake -D definitions, preprocessor definitions, one-line note) from a preset file."""
    import shlex

    cmake: list[tuple[str, str]] = []
    cpp: list[tuple[str, str]] = []
    note = ""
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if line.startswith("#"):
            if not note and "flew:" in line:
                note = line.split("flew:", 1)[1].strip()
            continue
        match = re.match(r"^(?:export\s+)?(RB_CMAKE_ARGS|RB_EXTRA_CPPFLAGS)=(.*)$", line)
        if not match:
            continue
        target = cmake if match.group(1) == "RB_CMAKE_ARGS" else cpp
        value = " ".join(shlex.split(match.group(2)))   # unquote the shell assignment
        for token in shlex.split(value):
            if not token.startswith("-D") or "=" not in token:
                raise ValueError(f"{path.name}: unsupported token {token!r} (only -DNAME=VALUE)")
            name, value = token[2:].split("=", 1)
            target.append((name, value))
    if not cmake and not cpp:
        raise ValueError(f"{path.name}: no RB_CMAKE_ARGS / RB_EXTRA_CPPFLAGS line")
    return tuple(cmake), tuple(cpp), note


def flight_presets() -> dict[str, tuple[tuple[tuple[str, str], ...], tuple[tuple[str, str], ...], str]]:
    if not FLIGHT_PRESETS_DIR.is_dir():
        return {}
    return {path.stem: _parse_preset(path) for path in sorted(FLIGHT_PRESETS_DIR.glob("*.env"))}


# ---- camera photo at the top of a flight ----------------------------------------------------
#
# A preset plus the camera, as a mode of its own, so the photo build is asked for by name and
# its knobs are recorded in rb-build.txt like every other flight's. Built on a flown preset
# through mode_bases rather than as a copied .env: the flight knobs stay exactly t21's, and a
# change to t21-hover-1.0m.env reaches the photo build instead of silently forking from it.
# The firmware side is workloads/lib/camera (ROSE_CAMERA; the sample's CMakeLists points
# ROSE_CAMERA_DIR there by default). `rb fly photo` reads the frame back over JTAG after landing
# (tools/flight/gdb/photo.gdb).
PHOTO_CPPFLAGS = (
    ("RB_CAMERA_STATUS_LED", "0"),     # the LED pin belongs to the flight image
    ("RB_CAMERA_FRAME_LEN", "1000"),
    ("RB_CAMERA_MAX_INTG", "900"),
)
# How long after boot a bench image captures on its own while disarmed (then drains at once).
PHOTO_BENCH_AFTER_MS = 15000
# (mode, base mode, CMake -D, preprocessor -D, note). The note is what rb-build.txt records on
# its `flew:` line, so it says plainly that these have NOT flown.
PHOTO_MODES = (
    (
        "t21-hover-1.0m-photo", "t21-hover-1.0m",
        (("ROSE_CAMERA", "1"),),
        PHOTO_CPPFLAGS,
        "NOT FLOWN as a photo build -- t21-hover-1.0m (which flew T20 T21 T22) plus one camera "
        "frame captured mid-hover and read back after landing (rb fly photo); "
        "the photo variant itself has never flown",
    ),
    (
        "t33-hover-1.5m-photo", "t33-hover-1.5m",
        (("ROSE_CAMERA", "1"),),
        PHOTO_CPPFLAGS,
        "NOT FLOWN as a photo build -- t33-hover-1.5m (T21's control at 1.5 m; T122 flew the full "
        "profile to 1.53 m and landed itself) plus one camera frame captured mid-hover and read "
        "back after landing (rb fly photo); the photo variant itself has never flown",
    ),
    (
        "photo-bench", "t21-hover-1.0m-photo",
        (),
        (("ROSE_PHOTO_BENCH_AFTER_MS", str(PHOTO_BENCH_AFTER_MS)),),
        "NOT A FLIGHT BUILD -- bench test of prepare->arm->drain->dump, pack OUT: "
        f"t21-hover-1.0m-photo that captures {PHOTO_BENCH_AFTER_MS // 1000} s after boot while "
        "disarmed and drains at once; then rb fly photo",
    ),
)


def _with_photo_modes(workload: Workload, table=PHOTO_MODES) -> Workload:
    known = set(workload.modes) | {m for m, *_ in table}
    if any(base not in known for _m, base, *_ in table):
        # A photo mode on a preset that is gone would build as esp-motors plus the camera --
        # a flight image with none of the flown values. Leave them out rather than that.
        return workload
    return replace(
        workload,
        modes=tuple(workload.modes) + tuple(m for m, *_ in table),
        cmake_defines_by_mode=tuple(workload.cmake_defines_by_mode)
        + tuple((m, cmake) for m, _b, cmake, _c, _n in table if cmake),
        cppflags_by_mode=tuple(workload.cppflags_by_mode)
        + tuple((m, cpp) for m, _b, _cm, cpp, _n in table if cpp),
        mode_bases=tuple(workload.mode_bases) + tuple((m, b) for m, b, *_ in table),
        mode_notes=tuple(workload.mode_notes) + tuple((m, n) for m, _b, _cm, _c, n in table),
    )


# ---- roll harness ("rig") -------------------------------------------------------------------
#
# The drone on a 1-DOF roll pivot (hard stops at ~+-30 deg), held level at hover collective while
# the roll loops are tuned over WiFi (workloads/flight_controller/src/fc_rig.h; RIG-SPEC). Same
# (mode, base, CMake -D, preprocessor -D, note) shape as PHOTO_MODES and built the same way, on
# esp-motors -- NOT a tools/flight/presets/*.env, which would be recorded as a build that flew.
# ROSE_TELEM_DIV 50 is ~25 Hz of RIG lines at the measured ~1260 Hz loop. ROSE_FLOW=0: the rig
# uses no flow, and the build only notes that the flow-spi alias is unused. The rig gains default
# to the build's ATT_GAIN / MOTOR_TRIM_LEFT, i.e. esp-motors' 1.0 / 0.87, not t21's 0.85 / 1.0:
# set them with GAIN before the first run.
# (CMake -D, preprocessor -D) shared by rig-roll and rig-pitch: one copy, so the two cannot drift.
_RIG_CMAKE = (
    ("ROSE_RIG", "1"),
    ("ROSE_AUTOFLIGHT", "1"),
    ("ROSE_UART_CMD", "1"),
    ("ROSE_FLIGHT_COMMIT", "1"),
    ("ROSE_COMMIT_MAX_MS", "900000"),   # 15 min from the first spin: one batch per load
    ("ROSE_ARM_NO_GESTURE", "1"),
    ("PID_MASS_KG", "0.050f"),
    ("AUTOFLIGHT_MAX_DUTY", "0.95f"),
    ("ROSE_BATT_REPORT_ONLY", "0"),
    ("ROSE_TELEM_DIV", "50"),
    ("ROSE_FLIGHTREC", "1"),
    ("ROSE_FLOW", "0"),
    # uart1 RX from its interrupt (main.cpp uart1_rx_isr, fc_uart_rx.h): polled once a
    # tick, the 8-byte RX FIFO overflowed and uplink lines were lost. A -DCONFIG_ here, like
    # the workload's CONFIG_UART_SIFIVE_PORT_1, because a derived mode builds with its base's
    # .conf fragments (layout_mode) and a flight_controller-rig-roll.conf would never be
    # read. TX stays polled; uart0 (console) only gains an unused, disabled IRQ hookup.
    ("CONFIG_UART_INTERRUPT_DRIVEN", "y"),
)
_RIG_CPPFLAGS = (
    ("ROSE_VEL_LOOP", "0"),
    ("ROSE_TELEM_CONSOLE", "0"),
    ("FLIGHTREC_DIV", "4"),
    # Start from the gains t21-hover-1.0m flew (its preset), not esp-motors' bench
    # defaults: MOTOR_TRIM_LEFT 0.87 there was measured as a -6 deg bank.
    ("ATT_GAIN", "0.85f"),
    ("MOTOR_TRIM_LEFT", "1.0f"),
    ("IMU_OFFSET_Y", "0.016f"),
    # v3 stand (2026-11-05): no blockers, the pivot turns a full 360 deg, so the tilt guard is
    # only for an extreme excursion. fc_rig.cpp's own defaults (0.44f / tilt - 0.09f) were sized
    # for a 30 deg stop, and the v2 stand's 0.227f / 0.14f for ~15 deg blockers.
    ("RIG_MAX_TILT_RAD", "1.05f"),    # 60 deg tilt guard: extreme only
    ("RIG_TGT_MAX_RAD", "0.35f"),     # 20 deg roll-target clamp
    # LiPo on the rig: t21's cutoff. The 3.2 V default estops on normal sag at hover
    # collective (sim twin, R0 ~0.18 ohm); the bench PSU browns the ESP out above ~0.45 duty.
    ("BATT_CUTOFF_V", "3.0f"),
    # Keep the cutoff and arm gate, drop the sag boost: on the bench PSU the ADC sees the
    # in-phase PWM troughs and the boost raised duty as the rail sagged (0.50 -> 0.58).
    ("ROSE_BATT_SAG_COMP", "0"),
    # Time allowed from the end of RAMP_UP to come off the stop before ramping down. 8 s
    # while a tethered USB cable loads one side; the 2 s default is for an untethered drone.
    ("RIG_LIFTOFF_TIMEOUT_MS", "8000"),
)
_RIG_NOTE = (
    "NOT A FLIGHT BUILD -- %s harness only: the drone on the 1-DOF %s pivot%s, arming over the "
    "uplink (RESET + HB + COLL), %s gains set live; it has no altitude, velocity or free-fall "
    "protection and must NEVER be flown free"
)
# rig-pitch: the same build with RIG_AXIS=1 -- the drone turned 90 deg in the cradle, so the pivot
# is PITCH. fc_rig.cpp then controls pitch (gyro y, u[2], the rear/front pairs), guards roll as the
# locked axis, and its RIG line ends ax=P (RIG-SPEC "Pitch axis"). A cppflag, so it lands on the
# build record's cppflags: line, which tools/rb/rig_axis.py falls back to when the ELF has no
# g_rig_axis symbol. The roll-named tuning (RIG_MAX_TILT_RAD, RIG_TGT_MAX_RAD, ROLL_TRIM) applies to
# the controlled axis, so the v2 stand's values carry over to the turned drone unchanged.
RIG_MODES = (
    ("rig-roll", "esp-motors", _RIG_CMAKE, _RIG_CPPFLAGS, _RIG_NOTE % ("roll", "roll", "", "roll")),
    ("rig-pitch", "esp-motors", _RIG_CMAKE, _RIG_CPPFLAGS + (("RIG_AXIS", "1"),),
     _RIG_NOTE % ("pitch", "pitch", " (turned 90 deg in the cradle)", "pitch")),
)


def _with_flight_presets(workload: Workload) -> Workload:
    presets = flight_presets()
    imu = (
        # Motor-incapable IMU sign test (`rb fly imu`): `default` has no motors
        # alias at all, and ROSE_IMU_DEBUG replaces the telemetry line with raw
        # body-frame accel/gyro so each axis sign can be checked by hand.
        "imu-test",
        (("ROSE_USE_PID", "1"), ("ROSE_USE_EKF", "0"), ("CTRL_ITERS", "0"), ("ROSE_TELEM_DIV", "100")),
        (("ROSE_IMU_DEBUG", "1"),),
        "motor-incapable IMU sign test (default mode + ROSE_IMU_DEBUG)",
    )
    modes = tuple(workload.modes) + tuple(presets) + (imu[0],)
    return replace(
        workload,
        modes=modes,
        cmake_defines_by_mode=tuple(workload.cmake_defines_by_mode)
        + tuple((name, cmake) for name, (cmake, _cpp, _note) in presets.items())
        + ((imu[0], imu[1]),),
        cppflags_by_mode=tuple(workload.cppflags_by_mode)
        + tuple((name, cpp) for name, (_cmake, cpp, _note) in presets.items())
        + ((imu[0], imu[2]),),
        mode_bases=tuple(workload.mode_bases)
        + tuple((name, "esp-motors") for name in presets)
        + ((imu[0], "default"),),
        mode_notes=tuple(workload.mode_notes)
        + tuple((name, note) for name, (_cmake, _cpp, note) in presets.items())
        + ((imu[0], imu[3]),),
    )


# ---- sensor relay (*-relay) ----------------------------------------------------------------
#
# The ESP reads the IMU, the down ToF and the ADS7128 battery channel and sends them over uart1
# (workloads/lib/motor_link/include/riskybird/sensor_link.h); the FPGA never touches its own I2C.
# For a drone whose FPGA cannot reach the sensor bus (2026-11-12: bus SDA reaches no FPGA pin).
# Each mode is its base plus four switches, nothing else:
#   RB_SENSOR_RELAY      CMakeLists.txt appends hardware/zephyr/fpga-sensor-relay.overlay last:
#                        i2c0 and its sensors disabled, the aliases on riskybird,relay-sensor nodes.
#   CONFIG_BMI08X/VL53L1X=n  their nodes are disabled; say so, rather than leave Kconfig to drop a
#                        requested =y.
#   CONFIG_UART_INTERRUPT_DRIVEN  frames arrive at ~7.6 kB/s; the polled 8-byte FIFO would drop them.
#   ROSE_SENSOR_RELAY    main.cpp: demux in uart_cmd_poll, keepalive, battery from the relay.
#   FC_RX_RING_SIZE 2048 ~270 ms of relay traffic, so a 30-60 ms polled-console stall cannot
#                        overrun the ring.
# Needs the esp_bridge motors image from the same commit (it answers the keepalive).
_RELAY_CMAKE = (
    ("RB_SENSOR_RELAY", "1"),
    ("CONFIG_BMI08X", "n"),
    ("CONFIG_VL53L1X", "n"),
    ("CONFIG_UART_INTERRUPT_DRIVEN", "y"),
)
_RELAY_CPPFLAGS = (
    ("ROSE_SENSOR_RELAY", "1"),
    ("FC_RX_RING_SIZE", "2048u"),
)
_RELAY_NOTE = (
    "%s, with the IMU, down ToF and battery relayed by the ESP over uart1 (FPGA I2C off); needs "
    "the esp_bridge motors image from the same commit"
)
RELAY_MODES = (
    ("esp-motors-relay", "esp-motors", _RELAY_CMAKE, _RELAY_CPPFLAGS, _RELAY_NOTE % "esp-motors"),
    ("rig-roll-relay", "rig-roll", _RELAY_CMAKE, _RELAY_CPPFLAGS, _RELAY_NOTE % "rig-roll"),
    ("rig-pitch-relay", "rig-pitch", _RELAY_CMAKE, _RELAY_CPPFLAGS, _RELAY_NOTE % "rig-pitch"),
)

WORKLOADS["flight_controller"] = _with_photo_modes(
    _with_photo_modes(
        _with_photo_modes(_with_flight_presets(WORKLOADS["flight_controller"])), RIG_MODES
    ),
    RELAY_MODES,
)


def get_board(name: str) -> Board:
    try:
        return BOARDS[name]
    except KeyError as exc:
        raise ValueError(f"unknown board {name!r}; choose: {', '.join(BOARDS)}") from exc


def get_target(name: str) -> ZephyrTarget:
    try:
        return TARGETS[name]
    except KeyError as exc:
        raise ValueError(
            f"unknown target {name!r}; choose: {', '.join(TARGETS)}"
        ) from exc


def get_workload(name: str) -> Workload:
    try:
        return WORKLOADS[name]
    except KeyError as exc:
        raise ValueError(
            f"unknown workload {name!r}; choose: {', '.join(WORKLOADS)}"
        ) from exc
