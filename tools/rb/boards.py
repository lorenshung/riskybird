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
CHIPYARD = ROOT / "backends" / "chipyard"
ZEPHYR_BACKEND = ROOT / "backends" / "zephyr-chipyard-sw"
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
        """
        built = self.generated_dir / "obj" / f"{self.harness}.bit"
        if built.is_file():
            return built
        collected = self.collected_bitstream
        if collected.is_file():
            return collected
        return built

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

    def workload_conf(self, workload: str, mode: str) -> Path | None:
        candidates = (
            self.target_dir / "workloads" / f"{workload}-{mode}.conf",
            self.target_dir / "workloads" / f"{workload}.conf",
            ROOT
            / "hardware"
            / "zephyr"
            / "targets"
            / self.kind
            / "workloads"
            / f"{workload}-{mode}.conf",
            ROOT
            / "hardware"
            / "zephyr"
            / "targets"
            / self.kind
            / "workloads"
            / f"{workload}.conf",
        )
        return next((path for path in candidates if path.is_file()), None)


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
    # The ESP link lives only in the full drone shells, which are arty200t-only
    # today. Requires a shell that elaborates serial@10021000; rb appends
    # fpga-esp-uart.overlay when it detects one.
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
    "flight_controller": _workload(
        "flight_controller",
        source_dir=ZEPHYR_BACKEND / "samples" / "rose_flight_controller",
        # One mode per bring-up phase, each selecting a device-tree overlay that
        # adds one subsystem to the previous. The overlays #include each other
        # rather than restating the IMU rates and bus speed, so the phases
        # cannot drift apart. What a mode declares is what the application
        # compiles in -- `default` has no `motors` alias and therefore contains
        # no code path to a motor at all.
        modes=("default", "flow", "motors"),
        targets=("arty200t",),
        summary="RoSE flight controller: estimator + control cascade + sensors",
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
