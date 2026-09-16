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
# Keep in sync with WithRocketFPU16 call sites in
# backends/chipyard/fpga/src/main/scala/arty200t/AccelConfigs.scala.
FP16_SCALAR_FPU_CONFIGS = frozenset({
    "RocketArty200TDroneGemminiSaturnFp16Config",
    "RocketArty200TDroneGemminiSaturnFp16At35Config",
})
CHIPYARD = ROOT / "backends" / "chipyard"
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

    def defines_for(self, mode: str) -> tuple[tuple[str, str], ...]:
        """Workload-wide definitions, then this mode's, which may override."""
        merged = dict(self.cmake_defines)
        for selected_mode, defines in self.cmake_defines_by_mode:
            if selected_mode == mode:
                merged.update(defines)
        return tuple(merged.items())

    def mode_conf(self, mode: str) -> Path | None:
        path = self.source_dir / "conf" / f"{mode}.conf"
        return path if path.is_file() else None

    def supports(self, target: ZephyrTarget) -> bool:
        return target.name in self.targets

    def minimum_ram(self, mode: str) -> int:
        return next(
            (
                minimum
                for selected_mode, minimum in self.minimum_ram_by_mode
                if selected_mode == mode
            ),
            0,
        )


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
) -> Workload:
    return Workload(
        name=name,
        source_dir=source_dir if source_dir is not None else ROOT / "workloads" / name,
        modes=modes,
        default_mode=default_mode,
        targets=targets,
        summary=summary,
        safety=safety,
        blocked_reason=blocked_reason,
        minimum_ram_by_mode=minimum_ram_by_mode,
        cmake_defines=cmake_defines,
        cmake_defines_by_mode=cmake_defines_by_mode,
    )


# General board diagnostics do not make sense on the RoSE bridge target.  The
# MPC workload opts into that target explicitly below.
ALL_TARGETS = tuple(name for name in TARGETS if name != "rose-spike")
I2C_TARGETS = ALL_TARGETS
LEGACY_VL53L0X_TARGETS = ("arty100t", "esp32c6")

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
        summary="Madgwick state estimator with replay or BMI088 input",
    ),
    "mpc": _workload(
        "mpc",
        modes=("replay", "hil", "rose"),
        default_mode="replay",
        targets=tuple(TARGETS),
        summary="TinyMPC replay, UART HIL, or RoSE lockstep controller",
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
    "i2c_expander": _workload(
        "i2c_expander",
        targets=LEGACY_VL53L0X_TARGETS,
        summary="Legacy one-ToF ADS7128 GPIO6 diagnostic",
    ),
    # Whole-board sensor check plus a go/no-go for the state estimator. Needs the
    # drone periphery (I2C for the sensor bus, SPI + GPIO for the flow sensor)
    # and the camera OSPI, i.e. one of the full drone shells.
    "sensor_check": _workload(
        "sensor_check",
        targets=("arty200t",),
        summary="All on-board sensors in one pass, ending in a state-estimator go/no-go",
        minimum_ram_by_mode=(("default", 0x8000),),
    ),
    # One pass over every peripheral a flight needs, in boot order, ending in a
    # board-ready verdict. Needs the drone periphery for the same reason
    # sensor_check does; `motors` additionally needs the battery and a shell
    # that carries pwm0/pwm1.
    "bootup_check": _workload(
        "bootup_check",
        modes=("default", "motors"),
        targets=("arty200t",),
        summary="Every peripheral in one boot-order pass, ending in a board-ready verdict",
        # A frame is 78892 bytes and the image carries it as a static buffer, so
        # a 32 KB scratchpad shell cannot run this at all. 256 KB excludes them
        # by contract rather than at link time.
        minimum_ram_by_mode=(("default", 0x40000), ("motors", 0x40000)),
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
        targets=("esp32c6",),
        summary=(
            "WiFi modem for the FPGA flight controller: UART <-> UDP "
            "telemetry :14550 and commands :14551"
        ),
    ),
    "esp_uart": _workload(
        "esp_uart",
        targets=("arty200t",),
        summary="Second UART (ESP32 link on E13/F14): TX proof and RX rate sweep",
        minimum_ram_by_mode=(("default", 0x8000),),
    ),
    # The HM01B0 target exists on both Arty parts. The 200T entry is the drone
    # carrier's own DVP pinout (bank-13 B2B pins), which is a different set of
    # balls from the 100T's -- the config selects them, not this table.
    "camera_bringup": _workload(
        "camera_bringup",
        targets=("arty100t", "arty200t"),
        summary="HM01B0 I2C control and pixel-capture bring-up",
        minimum_ram_by_mode=(("default", 0x8000),),
    ),
    # One photograph, for a shell where the camera is already proven. The RAM
    # floor is the point: a frame is 326*242 = 78892 bytes plus the image, so
    # this cannot be built against a 32 KiB scratchpad config, and the check
    # says so at build time rather than at the link error.
    # The flight stack that actually flew on the ESP32-C6, built here for the
    # FPGA. Its source_dir points INTO the Zephyr backend rather than into
    # workloads/: the sample is maintained upstream on
    # ucb-bar/zephyr-chipyard-sw, and copying it into this repository would fork
    # a 4,500-line application that is still being developed there. Building it
    # in place means `rb` adds a target instead of a branch.
    #
    # The application selects its subsystems from device-tree aliases
    # (DT_NODE_EXISTS(DT_ALIAS(flow/tof/baro))), so which sensors are compiled
    # in is decided by the FPGA overlay, not here -- which is what makes a
    # sensor-at-a-time bring-up possible. See docs/flight-on-fpga.md.
    # Dima's estimator, on its own. Same sources as flight_controller -- this is
    # a build variant, not a fork -- with the controller compiled out, so a bench
    # run measures the estimator and nothing else. The stale workloads/state
    # Madgwick target is NOT this: on the same stationary board it settles to
    # roll -178.7 deg where this reports roll 0.10, so the two disagree by half a
    # turn and only one of them flies.
    "flight_estimator": _workload(
        "flight_estimator",
        source_dir=ZEPHYR_BACKEND / "samples" / "rose_flight_controller",
        modes=("default", "flow"),
        targets=("arty200t",),
        summary="RoSE estimator alone: attitude, altitude and sensor validity, no controller",
        minimum_ram_by_mode=(("default", 0x80000),),
        cmake_defines=(("ROSE_ESTIMATOR_ONLY", "1"),),
    ),
    "flight_controller": _workload(
        "flight_controller",
        source_dir=ZEPHYR_BACKEND / "samples" / "rose_flight_controller",
        # One mode per bring-up phase, each selecting a device-tree overlay that
        # adds one subsystem to the previous. The overlays #include each other
        # rather than restating the IMU rates and bus speed, so the phases
        # cannot drift apart. What a mode declares is what the application
        # compiles in -- `default` has no `motors` alias and therefore contains
        # no code path to a motor at all.
        modes=("default", "flow", "motors", "autoflight"),
        targets=("arty200t",),
        summary="RoSE flight controller: estimator + control cascade + sensors",
        # Per-mode, because these are CMake cache variables: a .conf fragment
        # cannot reach them, and a mode that cannot set them is a mode whose
        # defining feature has to be remembered as a -D on the command line.
        # ROSE_AUTOFLIGHT is exactly that kind of knob -- passed the wrong way
        # it is silently ignored and the build falls through to an
        # always-armed bench path, which is the trap FLIGHT_BUILD.md warns
        # about and the reason this is declarative rather than documented.
        cmake_defines_by_mode=(
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
    ),
    "camera_photo": _workload(
        "camera_photo",
        targets=("arty200t",),
        summary="Capture one full HM01B0 frame into RAM for JTAG readout",
        minimum_ram_by_mode=(("default", 0x20000),),
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
    # ModelBlaster backend, the way flight_controller is built in place from the
    # Zephyr one: the harness is maintained upstream and takes the generated
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
        targets=ALL_TARGETS,
        summary="One-motor PWM diagnostic",
        safety="propellers-off",
    ),
    "motor_2": _workload(
        "motor_2",
        targets=ALL_TARGETS,
        summary="Two-motor PWM diagnostic",
        safety="propellers-off",
    ),
    "motor_3": _workload(
        "motor_3",
        targets=ALL_TARGETS,
        summary="Three-motor PWM diagnostic",
        safety="propellers-off",
    ),
    "motor_4": _workload(
        "motor_4",
        targets=ALL_TARGETS,
        summary="Four-motor interactive PWM diagnostic",
        safety="propellers-off",
    ),
    "motor_safe": _workload(
        "motor_safe",
        targets=("esp32c6",),
        summary=(
            "Park the shared ESP32-C6 motor pins as high-impedance inputs "
            "with pull-downs"
        ),
    ),
    "pmw3901_test": _workload(
        "pmw3901_test",
        targets=ALL_TARGETS,
        summary="PMW3901 SPI optical-flow diagnostic",
    ),
    "sensor_bringup": _workload(
        "sensor_bringup",
        targets=("esp32c6",),
        summary="Legacy PMW3901 plus one VL53L1X/ADS7128 GPIO6 diagnostic",
    ),
    "tof_sensor": _workload(
        "tof_sensor",
        targets=LEGACY_VL53L0X_TARGETS,
        summary="Legacy single VL53L0X diagnostic",
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
