"""Command line interface for RiskyBird development flows."""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import shlex
import shutil
import re
import signal
import socket
import subprocess
import termios
from typing import NamedTuple, Sequence
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from . import (
    __version__,
    area,
    artifacts,
    dse,
    fingerprint,
    flightlog,
    fly,
    ground,
    modelblaster,
    plotting,
    rose,
    soc,
    timing,
    xpurt,
)
from .area import AreaRecord
from .artifacts import (
    select_serial_port,
    serial_devices,
    zephyr_build_dir,
    zephyr_elf,
)
from .boards import (
    BOARDS,
    CHIPYARD,
    FP16_SCALAR_FPU_CONFIGS,
    INT_NOPERMUTE_VECTOR_CONFIGS,
    FPGA_ZEPHYR_BOARD,
    MODELBLASTER,
    ROOT,
    TARGETS,
    WORKLOADS,
    ZEPHYR_BACKEND,
    Board,
    Workload,
    ZephyrTarget,
    get_board,
    get_target,
    get_workload,
)
from .environments import (
    CHIPYARD_PREFIX,
    VIVADO_BIN,
    ZEPHYR_BASE,
    ZEPHYR_GNU_BIN,
    ZEPHYR_HOST_BIN,
    ZEPHYR_HOST_LOADER,
    ZEPHYR_MINIFORGE,
    ZEPHYR_PREFIX,
    ZEPHYR_SDK,
    executable,
    host_env,
    openocd_command,
    run,
    select_zephyr_prefix,
    shell,
    zephyr_env,
)


STUDIES_DIR = ROOT / "experiments" / "studies"
FPGA_CONF = ROOT / "hardware" / "zephyr" / "fpga.conf"
COMMON_OVERLAY = ROOT / "hardware" / "zephyr" / "fpga-common.overlay"
DRONE_OVERLAY = ROOT / "hardware" / "zephyr" / "fpga-drone.overlay"
NO_I2C_OVERLAY = ROOT / "hardware" / "zephyr" / "fpga-no-i2c.overlay"
ESP_UART_OVERLAY = ROOT / "hardware" / "zephyr" / "fpga-esp-uart.overlay"
# PLIC source of serial@10021000 in the full drone shells. Asserted against the
# generated DTS at build time rather than trusted.
ESP_UART_IRQ = 11
# gpio@10010000 and spi@10031000 with that UART elaborated. fpga-drone.overlay
# declares the pre-uart1 numbering (11-13 and 14); fpga-esp-uart.overlay
# overrides it to these. Asserted against the generated DTS at build time.
ESP_SHIFTED_GPIO_IRQS = (12, 13, 14)
ESP_SHIFTED_SPI_IRQ = 15
RAM_CONSOLE_CONF = ROOT / "hardware" / "zephyr" / "ram-console.conf"
FT_LINK_CONFIG = ROOT / "hardware" / "openocd" / "ft-link.cfg"
PROGRAM_TCL = ROOT / "tools" / "rb" / "program_fpga.tcl"
ESPRESSIF_HAL = (
    ZEPHYR_BACKEND / "zephyr_ws" / "modules" / "hal" / "espressif"
)
ST_HAL = ZEPHYR_BACKEND / "zephyr_ws" / "modules" / "hal" / "st"
VL53L5CX_HEADER = (
    ZEPHYR_BASE / "include" / "zephyr" / "drivers" / "sensor" / "vl53l5cx.h"
)
ZEPHYR_TARGET_REQUIREMENTS = ROOT / "tools" / "rb" / "requirements-zephyr-targets.txt"
CAPTURE_SERIAL = ROOT / "tools" / "rb" / "capture_serial.py"
DRONE_FPGA_WORKLOADS = {
    "motor_1",
    "motor_2",
    "motor_3",
    "motor_4",
    "pmw3901_test",
}


class RbError(RuntimeError):
    """A concise, user-facing command failure."""


def _populated(path: Path, *markers: str) -> bool:
    """True when a directory exists and looks actually installed.

    Every path in the doctor table can exist while not being usable, and each
    one means "not installed" when it is: an uninitialised submodule, an
    interrupted conda install, a half-extracted SDK, a west module that was
    never fetched. Reporting those as ok is worse than reporting nothing,
    because the setup guide's checkpoints are exactly where someone looks to
    decide whether the previous step worked.

    A non-empty test is not enough, which is the trap this used to fall into: a
    submodule init that is interrupted after writing its `.git` file and before
    checking anything out leaves a directory with exactly one entry in it. So
    callers name something the install is *supposed* to contain, and a directory
    holding only debris fails.

    Falls back to a non-empty test when no marker is given, for paths with no
    single stable member.
    """
    try:
        if not path.is_dir():
            return False
        if markers:
            return any((path / m).exists() for m in markers)
        return any(path.iterdir())
    except OSError:
        return False


def _yes_no(value: bool) -> str:
    return "ok" if value else "MISSING"


def _probe(command: list[str], environment: str) -> str:
    env = {
        "zephyr": zephyr_env,
        "host": host_env,
    }[environment]()
    try:
        result = subprocess.run(
            command,
            env=env,
            cwd=ROOT,
            capture_output=True,
            check=False,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return f"unavailable ({exc})"
    text = (result.stdout or result.stderr).strip().splitlines()
    return text[0] if text else f"exit {result.returncode}"


class ShellPeriphery(NamedTuple):
    """What a shell actually elaborated, read from its generated DTS."""

    has_i2c: bool
    uart_irq: int | None
    has_drone: bool
    # PLIC source of the second UART (0x1002_1000), the ESP32 link on E13/F14,
    # or None when the shell elaborates only the console UART. Read from the
    # DTS rather than the config name for the same reason as the rest of this
    # struct: the periphery is what decides, not what the class is called.
    esp_uart_irq: int | None = None
    # PLIC sources of gpio@10010000 (three) and spi@10031000 (one). Both shift
    # up by one in a shell that elaborates the ESP UART at source 11, so the
    # overlay that declares them has to be chosen against these, not guessed.
    gpio_irqs: tuple[int, ...] = ()
    spi_irq: int | None = None


def elaborated_periphery(board: Board) -> ShellPeriphery:
    """Whether the selected config elaborated I2C, and the UART's PLIC source.

    Read from the generated DTS rather than guessed from the config name. The
    name is not sufficient: CnnNavKU040Config carries the I2C controller
    without "Ospi" anywhere in it, and the OSPI periphery is what shifts the
    UART from source 1 to source 2. Across every config in
    backends/chipyard/fpga/generated-src the two facts agree -- I2C present
    means UART on 2, I2C absent means UART on 1 -- but the DTS is the one that
    is true by construction.

    Returns (False, None) when the DTS has not been generated; the caller's
    validate_hardware_contract raises the actionable error for that.
    """

    path = board.generated_dts
    if not path.is_file():
        return ShellPeriphery(False, None, False, None)
    text = path.read_text(encoding="utf-8")
    has_i2c = re.search(r"i2c@10040000\s*\{", text) is not None
    uart_irq = re.search(
        r"serial@10020000\s*\{.*?interrupts\s*=\s*<(\d+)>;",
        text,
        re.DOTALL,
    )
    # The drone periphery is the SPI + GPIO pair the flow sensor needs, and it
    # is what hardware/zephyr/fpga-drone.overlay describes. Detected here rather
    # than from a "DroneLogic" substring so a config that carries the periphery
    # under another name still gets the right overlay.
    has_drone = (
        re.search(r"spi@10031000\s*\{", text) is not None
        and re.search(r"gpio@10010000\s*\{", text) is not None
    )
    esp_uart_irq = re.search(
        r"serial@10021000\s*\{.*?interrupts\s*=\s*<(\d+)>;",
        text,
        re.DOTALL,
    )
    gpio_irqs = re.search(
        r"gpio@10010000\s*\{.*?interrupts\s*=\s*<([\d\s]+)>;",
        text,
        re.DOTALL,
    )
    spi_irq = re.search(
        r"spi@10031000\s*\{.*?interrupts\s*=\s*<(\d+)>;",
        text,
        re.DOTALL,
    )
    return ShellPeriphery(
        has_i2c,
        int(uart_irq.group(1)) if uart_irq is not None else None,
        has_drone,
        int(esp_uart_irq.group(1)) if esp_uart_irq is not None else None,
        tuple(int(n) for n in gpio_irqs.group(1).split()) if gpio_irqs else (),
        int(spi_irq.group(1)) if spi_irq is not None else None,
    )


def validate_hardware_contract(
    board: Board,
    require_dts: bool = True,
    require_i2c: bool = True,
    require_uart_irq: int | None = 2,
) -> list[str]:
    path = board.generated_dts
    if not path.is_file():
        if require_dts:
            raise RbError(
                f"elaborated DTS is missing for {board.name} "
                f"({board.config})\n"
                f"  looked in: {path}\n"
                f"  and:       {board.collected_dts}\n"
                "Either elaborate it here (needs Chipyard and a Linux host):\n"
                f"  rb chipyard elaborate --board {board.name}\n"
                "or collect one from a machine that has: the DTS is a plain "
                "file, so building a\nworkload needs it but not the toolchain "
                "that produced it. Point RB_DTS_DIR at a\ndirectory laid out "
                "as <board>/<board>.<Config>.dts."
            )
        return ["generated DTS not present"]

    text = path.read_text(encoding="utf-8")
    drone_logic = "DroneLogic" in board.config
    if drone_logic:
        require_i2c = True
        require_uart_irq = 10
    required = {
        "UART address": r"serial@10020000\s*\{",
    }
    if require_uart_irq is not None:
        required[f"UART interrupt {require_uart_irq}"] = (
            rf"serial@10020000\s*\{{.*?interrupts\s*=\s*<{require_uart_irq}>;"
        )
    if require_i2c:
        required.update(
            {
                "I2C address": r"i2c@10040000\s*\{",
                "I2C interrupt 1": (
                    r"i2c@10040000\s*\{.*?interrupts\s*=\s*<1>;"
                ),
            }
        )
    if drone_logic:
        required.update(
            {
                "GPIO address": r"gpio@10010000\s*\{",
                "GPIO interrupts 11..13": (
                    r"gpio@10010000\s*\{.*?interrupts\s*=\s*<11 12 13>;"
                ),
                "PWM0 address": r"pwm@10050000\s*\{",
                "PWM0 interrupts 2..5": (
                    r"pwm@10050000\s*\{.*?interrupts\s*=\s*<2 3 4 5>;"
                ),
                "PWM1 address": r"pwm@10051000\s*\{",
                "PWM1 interrupts 6..9": (
                    r"pwm@10051000\s*\{.*?interrupts\s*=\s*<6 7 8 9>;"
                ),
                "SPI address": r"spi@10031000\s*\{",
                "SPI interrupt 14": (
                    r"spi@10031000\s*\{.*?interrupts\s*=\s*<14>;"
                ),
                "PLIC source count 15": (
                    r"interrupt-controller@c000000\s*\{.*?"
                    r"riscv,ndev\s*=\s*<15>;"
                ),
            }
        )
    failures = [
        description
        for description, pattern in required.items()
        if re.search(pattern, text, re.DOTALL) is None
    ]
    # A --config override can elaborate any memory arrangement the board
    # supports, so match the size the DTS actually declares against the legal
    # set and report that one rather than the board default.
    elaborated_memory_size = next(
        (
            size
            for size in board.legal_memory_sizes
            if re.search(
                rf"memory@{board.memory_base:x}\s*\{{.*?"
                rf"reg\s*=\s*<0x{board.memory_base:x}\s+0x{size:x}>;",
                text,
                re.DOTALL,
            )
            is not None
        ),
        None,
    )
    if elaborated_memory_size is None:
        failures.append(
            f"memory 0x{board.memory_base:x}+"
            + "|".join(f"0x{size:x}" for size in board.legal_memory_sizes)
        )
    if failures:
        # Elaboration SUCCEEDED if we got here -- this is a check on what the
        # shell contains, not on whether it built. Say so: the old message read
        # as a build failure, which sent people looking for a Chisel error that
        # does not exist.
        #
        # A config with no cores is the honest case. NoCores*Config elaborates
        # cleanly and has neither a CPU nor a UART by design, so it fails a
        # contract whose whole purpose is to describe a deployable RiskyBird
        # target. That is a correct rejection of an inapplicable config, not a
        # broken one.
        raise RbError(
            f"{board.name} elaborated successfully, but the generated DTS is "
            f"not a deployable RiskyBird target.\n"
            f"  config:  {board.config}\n"
            f"  missing: " + ", ".join(failures) + "\n"
            "The RTL is fine -- this config simply does not present the "
            "periphery a\nRiskyBird workload needs. Configs with no cores, or "
            "built for another\npurpose, are expected to fail here."
        )
    uart_irq = re.search(
        r"serial@10020000\s*\{.*?interrupts\s*=\s*<(\d+)>;",
        text,
        re.DOTALL,
    )
    details = [
        f"config {board.config}",
        f"RAM 0x{board.memory_base:x}+0x{elaborated_memory_size:x}",
        "UART 0x10020000"
        + (f" irq {uart_irq.group(1)}" if uart_irq is not None else ""),
    ]
    if re.search(r"i2c@10040000\s*\{", text) is not None:
        details.append("I2C 0x10040000 irq 1")
    if drone_logic:
        details.extend(
            (
                "PWM 0x10050000/0x10051000 irq 2..9",
                "GPIO 0x10010000 irq 11..13",
                "SPI 0x10031000 irq 14",
            )
        )
    return details


def cmd_doctor(args: argparse.Namespace) -> int:
    print(f"RiskyBird rb {__version__}")
    print(f"repository       {ROOT}")
    checks = (
        ("Chipyard backend", _populated(CHIPYARD, "build.sbt", "build-setup.sh"), CHIPYARD),
        ("Chipyard env", _populated(CHIPYARD_PREFIX, "bin"), CHIPYARD_PREFIX),
        # Populated, not merely present -- same reason as Zephyr base below.
        # An uninitialised submodule leaves an EMPTY directory here, and
        # reporting that as ok contradicted the setup guide's own first
        # checkpoint on a clone where nothing was installed yet.
        (
            "Zephyr backend",
            _populated(ZEPHYR_BACKEND, "scripts"),
            ZEPHYR_BACKEND,
        ),
        ("Zephyr env", _populated(ZEPHYR_PREFIX, "bin"), ZEPHYR_PREFIX),
        # Its own tier, and MISSING is a normal state: every workload but dronet
        # builds without it. Reported anyway, because the alternative is that
        # `rb modelblaster` fails with a codegen traceback from inside a conda
        # environment rather than with "that backend is not initialised".
        (
            "ModelBlaster backend",
            _populated(MODELBLASTER, "pipeline", "harness"),
            MODELBLASTER,
        ),
        # Populated, not merely present. `git submodule update` and a failed
        # west workspace both leave an EMPTY directory here, and reporting that
        # as ok made the setup guide's own checkpoint pass on an install whose
        # very next command (rb build) could not run.
        ("Zephyr base", (ZEPHYR_BASE / "VERSION").is_file(), ZEPHYR_BASE),
        ("Zephyr SDK", _populated(ZEPHYR_SDK, "cmake"), ZEPHYR_SDK),
        ("Espressif HAL", _populated(ESPRESSIF_HAL, "components"), ESPRESSIF_HAL),
        ("ST HAL", _populated(ST_HAL, "CMakeLists.txt", "sensor"), ST_HAL),
        ("west", executable("zephyr", "west") is not None, executable("zephyr", "west")),
        # dtc is a hard requirement of `rb build` -- CMake needs an absolute
        # path to it -- but it is usually already installed, so its absence
        # only ever surfaced as a CMake error deep inside a west build.
        (
            "dtc",
            shutil.which("dtc", path=zephyr_env()["PATH"]) is not None,
            shutil.which("dtc", path=zephyr_env()["PATH"]),
        ),
        (
            "RISC-V compiler",
            (ZEPHYR_GNU_BIN / "riscv64-zephyr-elf-gcc").is_file(),
            ZEPHYR_GNU_BIN / "riscv64-zephyr-elf-gcc",
        ),
        (
            "Espresso",
            executable("chipyard", "espresso") is not None,
            executable("chipyard", "espresso"),
        ),
        ("Vivado", (VIVADO_BIN / "vivado").is_file(), VIVADO_BIN / "vivado"),
        # Mirror openocd_command(): the SDK's own binary, or one on PATH. The
        # ELF loader is NOT required -- it is a workaround for a relocated SDK,
        # and demanding it reported MISSING on any host that does not need one,
        # macOS included, while rb ran OpenOCD perfectly well.
        (
            "OpenOCD",
            (ZEPHYR_HOST_BIN / "openocd").is_file()
            or shutil.which("openocd") is not None,
            ZEPHYR_HOST_BIN / "openocd"
            if (ZEPHYR_HOST_BIN / "openocd").is_file()
            else (shutil.which("openocd") or ZEPHYR_HOST_BIN / "openocd"),
        ),
        (
            "GDB",
            (ZEPHYR_GNU_BIN / "riscv64-zephyr-elf-gdb").is_file(),
            ZEPHYR_GNU_BIN / "riscv64-zephyr-elf-gdb",
        ),
    )
    failed = False
    # Width from the labels themselves rather than a constant: adding a check
    # whose name is longer than the constant used to knock every column after it
    # out of line, which is a silly way for a diagnostic to become harder to read.
    label_width = max(17, max(len(label) for label, _, _ in checks))
    for label, passed, value in checks:
        print(f"{label:{label_width}} {_yes_no(passed):7} {value or ''}")
        failed |= not passed

    if executable("zephyr", "west"):
        print(f"{'west version':17}         {_probe(['west', '--version'], 'zephyr')}")
    if (ZEPHYR_HOST_BIN / "openocd").is_file() or shutil.which("openocd"):
        print(
            f"{'OpenOCD version':17}         "
            f"{_probe(openocd_command('--version'), 'zephyr')}"
        )
    if (VIVADO_BIN / "vivado").is_file():
        print(
            f"{'Vivado version':17}         "
            f"{_probe([str(VIVADO_BIN / 'vivado'), '-version'], 'host')}"
        )

    print("hardware contracts")
    for board in BOARDS.values():
        try:
            details = validate_hardware_contract(board, require_dts=False)
            state = "ok" if details != ["generated DTS not present"] else "pending"
            print(f"  {board.name:12} {state:7} {', '.join(details)}")
        except RbError as exc:
            failed = True
            print(f"  {board.name:12} ERROR   {exc}")

    devices = serial_devices()
    print("serial devices")
    if devices:
        for device in devices:
            print(f"  {device}")
    else:
        print("  none attached (build-only commands remain usable)")
        if args.hardware:
            failed = True

    activation_script = ZEPHYR_BACKEND / "scripts" / "activate_conda.sh"
    if activation_script.is_file():
        match = re.search(
            r"\bconda\s+activate\s+([A-Za-z0-9_.-]+)",
            activation_script.read_text(encoding="utf-8"),
        )
        if match is not None and match.group(1) != ZEPHYR_PREFIX.name:
            print(
                "warning: backend activate_conda.sh names "
                f"{match.group(1)!r}, but rb selected the installed "
                f"{ZEPHYR_PREFIX.name!r} prefix"
            )
    return 1 if failed else 0


def cmd_env(args: argparse.Namespace) -> int:
    if args.env_action == "status":
        for name, prefix in (
            ("chipyard", CHIPYARD_PREFIX),
            ("zephyr", ZEPHYR_PREFIX),
            ("rose", rose.ROSE_ENV),
        ):
            # `bin` rather than bare existence: 4b creates the prefix early and
            # populates it for minutes afterwards, so is_dir() reported ok on a
            # half-finished install. Same fix the doctor table carries.
            print(f"{name:9} {_yes_no(_populated(prefix, 'bin')):7} {prefix}")
        return 0

    if args.env_action == "shell":
        if args.environment == "rose":
            return rose.environment_shell()
        return shell(args.environment)

    if args.env_action == "setup":
        if args.environment == "rose":
            python = rose.setup_environment()
            print(f"RoSE host environment installed: {python.parent.parent}")
            return 0
        if args.environment == "zephyr":
            if ZEPHYR_PREFIX.is_dir():
                print(f"Zephyr environment already exists: {ZEPHYR_PREFIX}")
                return 0
            command = (
                'source scripts/install_conda.sh && '
                'bash scripts/install_submodules.sh'
            )
            status = subprocess.call(
                ["/bin/bash", "-c", command], cwd=ZEPHYR_BACKEND
            )
            if status:
                return status
            installed_prefix = select_zephyr_prefix()
            if not installed_prefix.is_dir():
                raise RbError(
                    "Zephyr setup completed without creating a supported "
                    f"environment under {installed_prefix.parent}"
                )
            print(f"Zephyr environment installed: {installed_prefix}")
            return 0
        if CHIPYARD_PREFIX.is_dir():
            print(f"Chipyard environment already exists: {CHIPYARD_PREFIX}")
            return 0
        require_chipyard("rb env setup chipyard")
        print("Bootstrapping Chipyard's own lean, locked environment.", flush=True)
        return subprocess.call(
            ["/bin/bash", "build-setup.sh", "--use-lean-conda"], cwd=CHIPYARD
        )
    raise RbError(f"unsupported env action: {args.env_action}")


def chipyard_build(board: Board, target: str) -> None:
    if board.config.startswith("Saturn") and executable("chipyard", "espresso") is None:
        raise RbError(
            f"{board.config} requires the Espresso logic minimizer, but it is "
            "missing from the Chipyard environment. Re-run Chipyard's "
            "toolchain-extra setup or install its pinned Constellation "
            "Espresso submodule into $RISCV/bin."
        )
    make_target = "bitstream" if target == "bitstream" else "verilog"
    run(
        [
            "make",
            "-C",
            CHIPYARD / "fpga",
            f"SUB_PROJECT={board.sub_project}",
            f"CONFIG={board.config}",
            make_target,
        ],
        environment="chipyard",
        cwd=ROOT,
    )


def _configured_board(args: argparse.Namespace) -> Board:
    return get_board(args.board).with_config(getattr(args, "config", None))


def require_chipyard(command: str) -> None:
    """Fail with a tier explanation rather than a make/bash error.

    An uninitialised submodule is an empty directory, so the underlying tools
    report a missing file and give the reader nothing to act on.
    """
    if _populated(CHIPYARD, "build.sbt", "build-setup.sh"):
        return
    raise RbError(
        f"'{command}' is a silicon-tier command and needs the Chipyard backend, "
        f"which is not installed at {CHIPYARD}.\n"
        "  The bench tier -- building Zephyr workloads and programming a board "
        "with bitstreams\n"
        "  built elsewhere -- does not need it, and on macOS it cannot be "
        "installed at all\n"
        "  because it requires Vivado, which is Linux-only.\n"
        "  On Linux, install it with:\n"
        "    git submodule update --init backends/chipyard\n"
        "    rb env setup chipyard\n"
        "  Otherwise supply prebuilt bitstreams via RB_BITSTREAM_DIR and an "
        "elaborated\n"
        "  devicetree via RB_DTS_DIR; see docs/setup.md steps 2d and 3c."
    )


def cmd_chipyard(args: argparse.Namespace) -> int:
    board = _configured_board(args)
    # 'path' is exempt: it resolves against RB_BITSTREAM_DIR too, so it is
    # answerable on the bench tier.
    if args.chipyard_action != "path":
        require_chipyard(f"rb chipyard {args.chipyard_action}")
    if args.chipyard_action == "path":
        # stdout stays the bare path so this remains scriptable, but a
        # non-zero exit with an empty stderr is indistinguishable from a
        # broken install -- the path printed is simply where the bitstream
        # WOULD be. Say which of the two situations this is.
        print(board.bitstream)
        if board.bitstream.is_file():
            return 0
        print(
            f"note: no bitstream at that path for {board.config}.\n"
            "      A config can be elaborated without being synthesised; only\n"
            "      a synthesised one has a bitstream. Build it with:\n"
            f"        rb chipyard build --board {board.name} "
            f"--config {board.config}\n"
            "      or supply one via RB_BITSTREAM_DIR / <root>/bitstreams.",
            file=sys.stderr,
        )
        return 1
    if args.chipyard_action == "elaborate":
        chipyard_build(board, "verilog")
        ospi_config = "Ospi" in board.config
        validate_hardware_contract(
            board,
            require_i2c=ospi_config,
            require_uart_irq=2 if ospi_config else None,
        )
        print(board.generated_dts)
        return 0
    if args.chipyard_action == "build":
        chipyard_build(board, "bitstream")
        ospi_config = "Ospi" in board.config
        validate_hardware_contract(
            board,
            require_i2c=ospi_config,
            require_uart_irq=2 if ospi_config else None,
        )
        if not board.bitstream.is_file():
            raise RbError(f"build completed but bitstream was not found: {board.bitstream}")
        print(board.bitstream)
        return 0
    if args.chipyard_action == "program":
        program_fpga(board, args.tool, getattr(args, "usb_location", "") or "")
        return 0
    raise RbError(f"unsupported chipyard action: {args.chipyard_action}")


def openfpgaloader_probe_selector(location: str) -> list[str]:
    """openFPGALoader arguments pinning it to the adapter at this USB location.

    openFPGALoader has no `adapter usb location` equivalent: it selects a probe
    by `--busdev-num BUS:DEV`, by `--cable-index`, or by `--ftdi-serial`. The
    serial is useless here because both dev-board adapters ship with a blank
    EEPROM, and bus:device is not stable across replugs -- the drone bench has
    seen the same adapter as 7:12, 7:17 and 7:18 in one session. So the stable
    bus-port path stays the user-facing knob and is resolved to whatever
    bus:device it currently holds, at the moment the command runs.
    """

    if not location:
        return []
    for adapter in ftdi_adapters():
        if adapter["location"] == location:
            return ["--busdev-num", f"{int(adapter['bus'])}:{int(adapter['device'])}"]
    attached = ", ".join(a["location"] for a in ftdi_adapters()) or "none"
    raise RbError(
        f"no FTDI adapter at USB location {location}; attached: {attached}. "
        "Run `rb probe-ftdi` -- the location changes when an adapter moves to "
        "a different port or hub."
    )


def program_fpga(board: Board, tool: str = "auto", usb_location: str = "") -> None:
    bitstream = board.bitstream
    if not bitstream.is_file():
        # Mirror the missing-DTS error: name BOTH places that were searched and
        # BOTH ways to satisfy them. Telling a bench-tier host to run
        # `rb chipyard build` is telling it to do the one thing it cannot --
        # Vivado has no macOS release, and that host is expected to consume a
        # bitstream built elsewhere.
        raise RbError(
            f"no bitstream for {board.name} ({board.config})\n"
            f"  looked in: {board.generated_dir / 'obj' / (board.harness + '.bit')}\n"
            f"  and:       {board.collected_bitstream}\n"
            "Either build it here (needs Chipyard and Vivado, Linux only):\n"
            f"  rb chipyard build --board {board.name} --config {board.config}\n"
            "or collect one from a machine that has: a bitstream is a plain "
            "file, so programming a\nboard needs it but not the toolchain that "
            "produced it. Point RB_BITSTREAM_DIR at a\ndirectory laid out as "
            "<board>/<board>.<Config>.bit.\n"
            "Note a config can be elaborated without being synthesised; only a "
            "synthesised one\nhas a bitstream."
        )
    selected = tool
    if selected == "auto":
        selected = (
            "openfpgaloader"
            if board.openfpgaloader_board and executable("host", "openFPGALoader")
            else "vivado"
        )
    if selected == "openfpgaloader":
        if not board.openfpgaloader_board:
            raise RbError(f"openFPGALoader has no known board definition for {board.name}")
        location = usb_location or os.environ.get("RB_PROGRAM_FTDI_LOCATION", "")
        # Same coin flip as the OpenOCD path, with a worse failure: unpinned,
        # openFPGALoader takes the first FTDI libusb enumerates, which on this
        # bench may be the debug adapter, the Trenz module or the Alveo card.
        # Refuse to guess rather than write a bitstream at whatever answers.
        if not location:
            candidates = [a for a in ftdi_adapters() if a["pid"] in MPSSE_PRODUCT_IDS]
            if len(candidates) > 1:
                listed = ", ".join(a["location"] for a in candidates)
                raise RbError(
                    f"{len(candidates)} FTDI adapters are attached ({listed}), so "
                    "openFPGALoader would pick one at random. Pass "
                    "--usb-location (or set RB_PROGRAM_FTDI_LOCATION) with the "
                    "location of the *bitstream* adapter -- the one whose "
                    "IDCODE is a config TAP, not the Rocket debug TAP. "
                    "Run: rb probe-ftdi --idcode"
                )
        loader = [
            "openFPGALoader",
            "--board",
            board.openfpgaloader_board,
            *openfpgaloader_probe_selector(location),
        ]
        # Write the SPI flash, not just the volatile SRAM config, so the part
        # holds this bitstream across a power cycle and the loaded shell cannot
        # silently drift from the image built against it.
        run(
            [*loader, "--write-flash", "--bitstream", bitstream],
            environment="host",
            cwd=ROOT,
        )
        # Then load the same image into SRAM, because on the TE0712 flash boot
        # does not survive the PROGRAM_B pulse openFPGALoader issues at the end
        # of a flash write. Measured on the drone bench: the part comes back
        # DONE=0 after every one of nine PROGRAM_B resets, while an SRAM load
        # of the byte-identical image reaches DONE=1 every time. Whether the
        # flash copy boots at all depends on the bitstream: only images built
        # with fpga-shells 80831d8 (2026-09-23, the TE0712 SPI_32BIT_ADDR boot
        # properties) can -- a 40 MHz shell built that day came back from a
        # cold power cycle on its own. The FullDDR image flown so far predates
        # it and never configures from flash. Either way, without this second
        # load `rb program` returns success and leaves an unconfigured part,
        # which is what makes `rb debug` report an all-ones scan that reads
        # like a broken cable.
        run(
            [*loader, "--bitstream", bitstream],
            environment="host",
            cwd=ROOT,
        )
        # A programming step that reports success says nothing about whether
        # the part actually configured. Ask the config TAP rather than assume.
        if location:
            require_fpga_configured(location)
        return
    if selected == "vivado":
        run(
            [
                VIVADO_BIN / "vivado",
                "-nojournal",
                "-nolog",
                "-mode",
                "batch",
                "-source",
                PROGRAM_TCL,
                "-tclargs",
                bitstream,
                board.vivado_device_pattern,
            ],
            environment="host",
            cwd=ROOT,
        )
        return
    raise RbError(f"unknown programming tool: {selected}")


# What hardware/zephyr/fpga.conf hardcodes: the 50 MHz shells' CLINT timebase,
# in Hz. Chipyard divides the fabric clock by CONFIG_RTC_CLOCK_DIVIDER_VALUE
# (1000) to make mtime, so this is fabric_hz / 1000 and the generated DTS
# reports exactly the same number as timebase-frequency.
FPGA_CONF_TIMEBASE_HZ = 50_000

# Generated Kconfig fragments live outside the build directory so their path is
# stable before the build directory exists, and under .rb/ because they are
# derived artifacts -- regenerating them must never look like an edit.
GENERATED_CONF_DIR = ROOT / ".rb" / "generated"


def generated_fpu_conf(board: Board) -> Path | None:
    """Kconfig for a shell whose scalar FPU cannot run fp32, or None.

    Zephyr puts "f" into -march whenever CONFIG_FPU=y, independently of
    CONFIG_FLOAT_HARD (cmake/compiler/gcc/target_riscv.cmake). On a shell built
    with WithRocketFPU16 the FPU implements only Zfh, so that permission lets
    gcc emit fp32 the hardware faults on -- and the devicetree agrees with gcc
    rather than with the silicon, because rocket-chip writes "f" into riscv,isa
    for any FPU at all.

    Turning CONFIG_FPU off is what makes the ISA gcc targets match the ISA the
    shell implements.

    THIS FRAGMENT MUST NOT ALSO SET CONFIG_RISCV_ISA_EXT_V. It used to, so that
    Saturn would be "reachable", and that silently undid the line above:
    Zephyr appends a bare "v" to the global -march for RISCV_ISA_EXT_V, and GCC
    expands "v" to zve64d, which implies "d" and "f". The image was built
    -march=rv64imacv_zicsr_zifencei -mabi=lp64, i.e. soft-float CALLING
    CONVENTION with hardware float ARITHMETIC: the DroNet gemmini_q31_softfp
    image for RocketArty200TDroneGemminiSaturnFp16At35Config carried 80
    fadd/fcvt/flw/fsw in main() and three kernels, every one of which traps on a
    shell with misa.F = 0. A workload that wants the vector unit asks for it
    itself, as the ModelBlaster harness's gemmini_q31_rvv_softfp overlay does:
    CONFIG_RISCV_ISA_EXT_V=y together with CONFIG_RISCV_V_KERNEL_ONLY=y, which
    keeps "v" out of the global -march, while kernels.c gets
    -march=rv64imac_zve64x -- integer vector, nothing floating-point implied.

    Float still works, in software. picolibc's soft-float is what the harness's
    %g output and any fp32 left in a model go through, which costs cycles and
    changes no answers.
    """
    if board.config not in FP16_SCALAR_FPU_CONFIGS:
        return None
    path = GENERATED_CONF_DIR / f"{board.name}.{board.config}.fpu.conf"
    body = (
        "# GENERATED by rb from the shell's scalar FPU width. Do not edit.\n"
        f"# {board.name}/{board.config} was elaborated with WithRocketFPU16:\n"
        "# the FPU implements Zfh and nothing else.\n"
        "#\n"
        "# CONFIG_FPU=y would put \"f\" into -march regardless of\n"
        "# CONFIG_FLOAT_HARD, letting gcc emit fp32 this hardware cannot\n"
        "# execute. Off, the compiler targets the ISA that actually exists and\n"
        "# float goes through picolibc's soft-float instead.\n"
        "#\n"
        "# Deliberately NOT CONFIG_RISCV_ISA_EXT_V=y: Zephyr turns it into a\n"
        "# global \"v\" in -march, which GCC expands to imply f and d -- hardware\n"
        "# float again. Workloads that use the vector unit enable it themselves\n"
        "# with CONFIG_RISCV_V_KERNEL_ONLY=y (see tools/rb/cli.py).\n"
        "CONFIG_FPU=n\n"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.is_file() or path.read_text() != body:
        path.write_text(body)
    return path


def dts_timebase_hz(board: Board) -> int | None:
    """The CLINT timebase the shell actually elaborated, from its DTS.

    Reads the devicetree rather than the generated-source tree so it works on
    the bench tier too, where Chipyard is absent and the DTS was collected from
    the Linux host (see Board.generated_dts).
    """
    dts = board.generated_dts
    if not dts.is_file():
        return None
    tree = fingerprint.parse_dts(dts.read_text())
    for node in tree.walk():
        value = node.cell("timebase-frequency")
        if value:
            return int(value)
    return None


def generated_clock_conf(board: Board) -> Path | None:
    """A Kconfig fragment carrying this shell's real clock, or None at 50 MHz.

    One line does the whole job, which is why this is worth generating rather
    than maintaining by hand per config. The rocketchip SoC defines
    SIFIVE_PERIPHERAL_CLOCK_FREQUENCY as
    CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC * CONFIG_RTC_CLOCK_DIVIDER_VALUE
    (soc/rocketchip/virt_riscv/common/soc.h), and every SiFive driver takes its
    clock from that macro. So the single constant below sets the kernel tick
    rate, the UART baud divisor, and the I2C, SPI and PWM dividers together.

    Getting it wrong is not a subtle failure. At 35 MHz with fpga.conf's 50 kHz
    value the SiFive UART divides for 50 MHz and transmits 115200 * 35/50 =
    80640 baud, which on a 115200 host reads as line noise -- indistinguishable
    from the dead console this bench has already chased once
    (docs/drone-initial-bringup.md). The timer is wrong by the same ratio, so
    k_cycle_get_64 and the harness's MODELBLASTER_WALL_CYCLES are inflated 43%
    and every derived FPS number with them.
    """
    timebase = dts_timebase_hz(board)
    if timebase is None or timebase == FPGA_CONF_TIMEBASE_HZ:
        return None
    path = GENERATED_CONF_DIR / f"{board.name}.{board.config}.clock.conf"
    body = (
        "# GENERATED by rb from the elaborated devicetree. Do not edit.\n"
        f"# {board.name}/{board.config}\n"
        f"# timebase-frequency = {timebase} Hz "
        f"({timebase * 1000 / 1e6:.6g} MHz fabric clock)\n"
        "#\n"
        "# Overrides hardware/zephyr/fpga.conf, which assumes the 50 MHz shells.\n"
        "# Sets the kernel tick rate and, through\n"
        "# SIFIVE_PERIPHERAL_CLOCK_FREQUENCY, the UART baud divisor and the\n"
        "# I2C/SPI/PWM dividers.\n"
        f"CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC={timebase}\n"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    # Rewrite only on change, so a rebuild does not invalidate Zephyr's cache
    # for a fragment whose content is identical.
    if not path.is_file() or path.read_text() != body:
        path.write_text(body)
    return path


def _git_describe(path: Path) -> str:
    try:
        rev = subprocess.run(["git", "-C", str(path), "rev-parse", "--short=12", "HEAD"],
                             capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", str(path), "status", "--porcelain", "--", "."],
                               capture_output=True, text=True, check=True).stdout.strip()
        return rev + ("-dirty" if dirty else "")
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def write_build_record(target, workload, mode: str, board, elf: Path) -> Path:
    """What this build was, next to its ELF: rb-build.txt in the build directory.

    One file that answers "what flew?" without the shell history: the mode and
    the preset it came from, every CMake and preprocessor knob the mode set, any
    RB_* environment override on top, the source revisions, and the image hash.
    `rb fly commit` prints its path and `rb flightlog push --build-file` takes it.
    """
    import hashlib

    lines = [
        f"workload:     {workload.name}",
        f"mode:         {mode}" + (f"   (base: {workload.layout_mode(mode)})" if workload.layout_mode(mode) != mode else ""),
    ]
    if workload.mode_note(mode):
        lines.append(f"flew:         {workload.mode_note(mode)}")
    lines += [
        f"target:       {target.name}" + (f"  config: {board.config}" if board is not None else ""),
        f"setup rev:    {_git_describe(ROOT)}",
        f"firmware rev: {_git_describe(ZEPHYR_BACKEND)}",
        f"elf sha1:     {hashlib.sha1(elf.read_bytes()).hexdigest()[:16]}",
        "cmake:        " + " ".join(f"-D{n}={v}" for n, v in workload.defines_for(mode)),
        "cppflags:     " + " ".join(f"-D{n}={v}" for n, v in workload.cppflags_for(mode)),
    ]
    for var in ("RB_CMAKE_ARGS", "RB_EXTRA_CPPFLAGS", "RB_EXTRA_CFLAGS"):
        if os.environ.get(var):
            lines.append(f"{var}: {os.environ[var]}")
    record = elf.parent.parent / "rb-build.txt"
    record.write_text("\n".join(lines) + "\n")
    return record


def _warn_legacy(workload: Workload, mode: str) -> None:
    """Print a legacy workload's note loudly enough to survive west's output."""
    note = workload.legacy_note(mode)
    if not note:
        return
    what = workload.name if not dict(workload.mode_legacy).get(mode) else f"{workload.name} --mode {mode}"
    rule = "rb: " + "!" * 72
    print(f"{rule}\nrb: LEGACY: {what} -- {note}\n{rule}", file=sys.stderr, flush=True)


def _workload_board(board: Board, workload: Workload, config: str | None) -> Board:
    """--config if given, else the workload's default config for this board."""
    selected = workload.board_for(board, config)
    if config is None and selected.config != board.config:
        print(
            f"rb: {workload.name} builds against {selected.config} on "
            f"{board.name} by default (--config overrides)",
            flush=True,
        )
    return selected


def zephyr_build(
    target: ZephyrTarget,
    workload: Workload,
    mode: str,
    pristine: str,
    fpga_board: Board | None = None,
    ram_console: bool = False,
    extra_defines: tuple[tuple[str, str], ...] = (),
) -> Path:
    if workload.blocked_reason is not None:
        raise RbError(f"{workload.name} is blocked: {workload.blocked_reason}")
    if mode not in workload.modes:
        raise RbError(
            f"{workload.name} mode {mode!r} is invalid; choose: "
            + ", ".join(workload.modes)
        )
    if not workload.supports(target):
        supported = ", ".join(workload.targets)
        raise RbError(
            f"{workload.name} is not enabled for target {target.name}; "
            f"currently enabled for: {supported}"
        )
    if not workload.supports(target, mode):
        raise RbError(
            f"{workload.name} --mode {mode} is not enabled for target "
            f"{target.name}; that mode builds for: "
            + ", ".join(workload.mode_target_names(mode))
        )
    if target.kind == "rose" or (workload.name == "mpc" and mode == "rose"):
        # Refused here rather than left to CMake, which reports the missing
        # module as a zephyr_module.cmake error that names neither RoSE nor
        # the branch it lives on.
        _require_main_backend(
            rose.ROSE_MODULE / "zephyr" / "module.yml",
            "the RoSE Zephyr module",
            f"{workload.name} --mode {mode} on {target.name}",
        )
    _warn_legacy(workload, mode)
    if (
        workload.name == "vl35l5cx_test"
        and mode == "ranging"
        and not VL53L5CX_HEADER.is_file()
    ):
        raise RbError(
            "vl35l5cx_test ranging mode requires the VL53L5CX "
            "Zephyr driver, which is absent from the pinned backend; use "
            "--mode address to validate all four XSHUT lines, addresses, "
            "and sensor identities"
        )

    overlays: list[Path] = []
    conf_files: list[Path] = []
    board = fpga_board or target.fpga_board
    if board is not None:
        minimum_ram = workload.minimum_ram(mode)
        # The selected config decides this, not the board default: a scratchpad
        # config exposes 32 KiB where the DDR default exposes the full region.
        available_ram = board.config_memory_size
        if minimum_ram > available_ram:
            raise RbError(
                f"{workload.name} mode {mode} requires at least "
                f"{minimum_ram // 1024} KiB addressable RAM on FPGA targets; "
                f"{board.name}/{board.config} currently exposes "
                f"{available_ram // 1024} KiB.\n"
                + (
                    # Only suggest a mode this workload actually has. The
                    # message used to name "address" unconditionally, which is
                    # a mode of vl35l5cx_test and not of most workloads that
                    # hit this check.
                    f"Try a smaller mode ({', '.join(smaller)}) or select "
                    "a configuration with external memory."
                    if (smaller := [
                        m for m in workload.modes
                        if m != mode and workload.minimum_ram(m) <= available_ram
                    ])
                    else "Select a configuration with external memory: this "
                    "workload has no mode that fits."
                )
            )
        if (
            workload.name in DRONE_FPGA_WORKLOADS
            and not elaborated_periphery(board).has_drone
        ):
            # Derived rather than tabulated: a table silently KeyErrors on a
            # board added later, which turns a clear diagnostic into a crash.
            example_config = {
                "arty100t": "RocketArty100TDroneLogicConfig",
                "arty200t": "RocketArty200TDroneLogicConfig",
                "ku040": "RocketKU040DroneLogicConfig",
            }.get(board.name)
            hint = (
                f", for example {example_config}" if example_config else ""
            )
            raise RbError(
                f"{workload.name} needs the SPI and GPIO periphery, and "
                f"{board.name}/{board.config} elaborates neither; "
                f"select a configuration with it using --config{hint}"
            )
        # What the shell actually elaborated decides both the contract and the
        # overlay set. A workload that needs I2C on a shell without it is still
        # an error; a workload that does not may run on either.
        shell = elaborated_periphery(board)
        shell_has_i2c, shell_uart_irq = shell.has_i2c, shell.uart_irq
        # ddr_stress joins mpc and hello_world here: it touches main memory and
        # the console and no peripheral at all, which is what makes it the right
        # first workload on a DDR shell built without the OSPI periphery.
        needs_i2c = workload.name not in {"mpc", "hello_world", "ddr_stress"} and not (
            workload.name == "state" and mode == "replay"
        )
        validate_hardware_contract(
            board,
            require_i2c=needs_i2c,
            require_uart_irq=shell_uart_irq,
        )
        if needs_i2c and not shell_has_i2c:
            raise RbError(
                f"{workload.name} needs the I2C controller, and "
                f"{board.name}/{board.config} elaborates none. Select a "
                "configuration with the OSPI periphery, for example "
                f"{board.name.replace('arty', 'RocketArty').replace('ku040', 'RocketKU040')}OspiConfig"
            )
        overlays.extend(
            (DRONE_OVERLAY if shell.has_drone else COMMON_OVERLAY, board.overlay)
        )
        # fpga-common.overlay describes the OSPI shells, where the UART sits on
        # PLIC source 2 behind I2C on source 1. Without I2C the UART moves down
        # to source 1, and a console bound to the wrong source never drains.
        if not shell_has_i2c and not shell.has_drone:
            overlays.append(NO_I2C_OVERLAY)
        # The second UART exists only in the full drone shells, and Zephyr's
        # chipyard-riscv.dtsi declares only uart0, so the node has to be added
        # for the ESP link to be addressable at all. Gate it on the shell
        # actually having elaborated the controller: binding a Zephyr device to
        # an MMIO window that no peripheral answers reads back garbage.
        if shell.esp_uart_irq is not None:
            if shell.esp_uart_irq != ESP_UART_IRQ:
                raise RbError(
                    f"{board.config} puts the ESP UART on PLIC source "
                    f"{shell.esp_uart_irq}, but "
                    f"{ESP_UART_OVERLAY.name} hardcodes {ESP_UART_IRQ}. Update "
                    "the overlay to match the generated DTS."
                )
            # The same overlay overrides gpio0/spi0, because elaborating this
            # UART at 11 pushes every later PLIC source up by one. Check the
            # DTS agrees rather than assuming the shift: a shell that numbers
            # differently would otherwise point the optical-flow SPI at the
            # wrong source and fail as "no flow", never naming an interrupt.
            if shell.gpio_irqs and shell.gpio_irqs != ESP_SHIFTED_GPIO_IRQS:
                raise RbError(
                    f"{board.config} puts gpio@10010000 on PLIC sources "
                    f"{list(shell.gpio_irqs)}, but {ESP_UART_OVERLAY.name} "
                    f"overrides them to {list(ESP_SHIFTED_GPIO_IRQS)}. Update "
                    "the overlay to match the generated DTS."
                )
            if shell.spi_irq is not None and shell.spi_irq != ESP_SHIFTED_SPI_IRQ:
                raise RbError(
                    f"{board.config} puts spi@10031000 on PLIC source "
                    f"{shell.spi_irq}, but {ESP_UART_OVERLAY.name} overrides "
                    f"it to {ESP_SHIFTED_SPI_IRQ}. Update the overlay to match "
                    "the generated DTS."
                )
            overlays.append(ESP_UART_OVERLAY)
        conf_files.append(FPGA_CONF)
        # fpga.conf hardcodes the 50 MHz shells' 50 kHz CLINT timebase. The
        # accelerator configs deliberately do not run at 50 MHz -- the combined
        # Gemmini+Saturn point closes at 35 and the split ones at 40 -- and on
        # those shells that constant is simply wrong. Nothing warns: Zephyr
        # converts cycles to time with it, so k_cycle_get_64 and therefore the
        # harness's MODELBLASTER_WALL_CYCLES come out scaled by 50/35, and an
        # FPS figure derived from them is 43% too high while looking perfectly
        # plausible. The elaborated DTS already carries the real value, so take
        # it from there rather than asking an operator to remember.
        clock_conf = generated_clock_conf(board)
        if clock_conf is not None:
            conf_files.append(clock_conf)
        fpu_conf = generated_fpu_conf(board)
        if fpu_conf is not None:
            conf_files.append(fpu_conf)

    if target.common_overlay.is_file():
        overlays.append(target.common_overlay)
    if target.common_conf.is_file():
        conf_files.append(target.common_conf)

    # A preset mode builds with its base mode's overlays and .conf fragments.
    layout = workload.layout_mode(mode)
    workload_overlay = target.workload_overlay(workload.name, layout)
    if workload_overlay is not None:
        overlays.append(workload_overlay)
    conf_files.extend(target.workload_confs(workload.name, layout))
    mode_conf = workload.mode_conf(mode)
    if mode_conf is not None:
        conf_files.append(mode_conf)

    for required in (
        *overlays,
        *conf_files,
        workload.source_dir / "CMakeLists.txt",
        workload.source_dir / "prj.conf",
    ):
        if not required.is_file():
            raise RbError(f"required build input is missing: {required}")

    # Last, so its CONFIG_UART_CONSOLE=n overrides hardware/zephyr/fpga.conf.
    if ram_console:
        conf_files.append(RAM_CONSOLE_CONF)

    build_dir = zephyr_build_dir(target, workload, mode, ram_console)
    build_dir.mkdir(parents=True, exist_ok=True)
    cmake_args = [
        f"-DRB_ROOT={ROOT}",
        f"-DRB_TARGET={target.name}",
        # Resolved against the build environment's PATH, not the caller's: on a
        # Mac dtc is Homebrew's, and 2b may install it into the conda env. A bare
        # "-DDTC=dtc" does not work -- CMake resolves it relative to the build
        # directory. /usr/bin is SIP-protected, so a symlink is not a way out.
        f"-DDTC={shutil.which('dtc', path=zephyr_env()['PATH']) or '/usr/bin/dtc'}",
        f"-DDTS_ROOT={ROOT / 'hardware' / 'zephyr'}",
    ]
    # The workload's own definitions, before RB_CMAKE_ARGS so an operator can
    # still override one from the command line.
    cmake_args.extend(
        f"-D{name}={value}" for name, value in workload.defines_for(mode)
    )
    # Definitions a caller computed rather than declared. The ModelBlaster
    # harness needs MODEL_DIR -- the directory codegen just wrote -- which
    # depends on the model, quantization and kernel target and so cannot be a
    # static entry in the workload registry. Placed after the workload's own
    # definitions and still before RB_CMAKE_ARGS, so the precedence is
    # registry < caller < operator.
    # The ModelBlaster harness cannot build without MODEL_DIR, and the commands
    # that make up the deploy path -- rb build, debug, run, capture -- all
    # rebuild and none of them has a --model to pass. Left alone they fail at
    # CMake with "MODEL_DIR not set", which made `rb modelblaster build` the only
    # way to build this workload and the JTAG load path unusable. Fill the
    # definitions in from the defaults when the caller has not supplied them; an
    # explicit extra_defines still wins, which is how `rb modelblaster build`
    # selects a non-default quantization or kernel target.
    if workload.source_dir == MODELBLASTER / "harness" and not any(
        name == "MODEL_DIR" for name, _ in extra_defines
    ):
        # Reuse whatever the existing build was made from, so a rebuild by a
        # command with no --model of its own (rb debug, rb run, rb build)
        # cannot silently swap the kernel backend underneath an image someone
        # deliberately built with `rb modelblaster build --target ...`.
        marker = modelblaster.read_build_marker(
            zephyr_build_dir(target, workload, mode, ram_console)
        )
        if marker is not None:
            # Name the DIRECTORY, not just the three names. The earlier
            # version printed a correct-looking target while silently building
            # from a different directory, so the line read as reassurance at
            # the exact moment it should have been alarming.
            print(
                f"rb: reusing this build's ModelBlaster selection\n"
                f"      model={marker['model']} quant={marker['quant']} "
                f"target={marker['target']}\n"
                f"      sources={marker.get('gen_dir', '(default path)')}\n"
                f"    `rb modelblaster build` changes it"
            )
        defaults = modelblaster.harness_defines(
            marker["model"] if marker else workload.name,
            quant=marker["quant"] if marker else modelblaster.DEFAULT_QUANT,
            target=marker["target"] if marker else modelblaster.DEFAULT_TARGET,
            # The DIRECTORY, not just the three names it is usually derived
            # from. Round-tripping (model, quant, target) and rebuilding the
            # default path from them looks right and is not: a build made with
            # --gen-dir pointing at published, verified sources was rebuilt
            # against whatever local codegen happened to sit at the default
            # path, while the reassuring "reusing this build's selection" line
            # named the correct target. That swapped a verified conv2d for one
            # the gate rejects at max_abs_err 56, and only a diff of
            # compile_commands.json caught it.
            gen_dir=Path(marker["gen_dir"]) if marker and marker.get("gen_dir")
            else None,
            gemmini_config=board.config if board is not None else None,
        )
        if board is not None:
            # The same refusal `rb modelblaster build` makes, applied here so it
            # cannot be walked around by using a different rb command to build
            # the identical image.
            _check_float_contract(
                board,
                dict(defaults)["MODELBLASTER_KERNEL_CFLAGS"].split(";"),
            )
        extra_defines = defaults + tuple(extra_defines)
    cmake_args.extend(f"-D{name}={value}" for name, value in extra_defines)
    if not (workload.source_dir / "Kconfig").is_file():
        cmake_args.append(
            f"-DKCONFIG_ROOT={ROOT / 'hardware' / 'zephyr' / 'Kconfig.workload'}"
        )
    if overlays:
        cmake_args.append(
            "-DDTC_OVERLAY_FILE=" + ";".join(str(path) for path in overlays)
        )
    if conf_files:
        cmake_args.append(
            "-DEXTRA_CONF_FILE=" + ";".join(str(path) for path in conf_files)
        )
    if board is not None:
        cmake_args.append(f"-DRB_BOARD={board.name}")
        # RB_EXTRA_CFLAGS appends rather than replaces: EXTRA_CFLAGS already
        # carries the RiskyBird include path, so a second -DEXTRA_CFLAGS= would
        # silently drop it. Used for the per-run knobs the workloads document,
        # e.g. RB_EXTRA_CFLAGS=-DMOTOR_MAX_PCT=80 for a propellers-off bench
        # test. The build tree is shared, so a build made with one of these
        # stays until the next build without it.
        extra_cflags = f"-I{ROOT / 'hardware' / 'zephyr' / 'include'}"
        if os.environ.get("RB_EXTRA_CFLAGS"):
            extra_cflags += " " + os.environ["RB_EXTRA_CFLAGS"]
        cmake_args.append(f"-DEXTRA_CFLAGS={extra_cflags}")
        # EXTRA_CFLAGS reaches C only. RB_EXTRA_CPPFLAGS becomes Zephyr's
        # EXTRA_CPPFLAGS, which is applied to C, C++ and assembly alike, and is
        # therefore the only one of the two that reaches a C++ application --
        # the flight controller's main.cpp among them. Its own docs are explicit
        # that defines like ROSE_AUTOFLIGHT must go here: passed any other way
        # they are silently ignored and the build quietly falls through to a
        # different flight mode.
        # The mode's own preprocessor knobs first (a flight preset's T_CLIMB_MS,
        # ATT_GAIN, ...), then RB_EXTRA_CPPFLAGS, so an explicit override wins.
        cppflags = " ".join(f"-D{name}={value}" for name, value in workload.cppflags_for(mode))
        if os.environ.get("RB_EXTRA_CPPFLAGS"):
            cppflags = (cppflags + " " + os.environ["RB_EXTRA_CPPFLAGS"]).strip()
        if cppflags:
            cmake_args.append(f"-DEXTRA_CPPFLAGS={cppflags}")
    # Application CMake cache knobs, appended last so they win. Distinct from
    # RB_EXTRA_CPPFLAGS: these are cache variables an application's CMakeLists
    # reads with set(... CACHE ...) and then turns into compile definitions
    # itself -- ROSE_USE_PID, ROSE_USE_EKF, CTRL_ITERS, ROSE_FLOW. Passing one
    # of those as a bare preprocessor define instead does nothing at all,
    # because the CMakeLists overwrites it from the cache default.
    if os.environ.get("RB_CMAKE_ARGS"):
        cmake_args.extend(shlex.split(os.environ["RB_CMAKE_ARGS"]))

    if workload.name == "mpc":
        cmake_args.append(f"-DRB_BOARD={target.name}")
        if mode == "rose":
            cmake_args.append(
                f"-DZEPHYR_EXTRA_MODULES={rose.ROSE_MODULE}"
            )
    # west's `-p auto` re-runs a pristine build when the board or toolchain
    # changes, but not when the devicetree changes underneath it, and the build
    # directory does not carry the config in its path. Switching --config
    # between a scratchpad shell and a DDR one therefore reuses the cached
    # .config: `rb debug hello_world --config RocketArty200TDroneDDRConfig`
    # produced an image linked against **32 KiB** -- the previously built
    # scratchpad config's memory -- while its own generated DTS said 1 GiB and
    # CONFIG_SRAM_SIZE said 32. Nothing errors, because a small image still
    # links and runs; it is simply built for the wrong machine, and a workload
    # sized against the DDR region would silently overrun. Stamp the inputs that
    # decide the memory map and rebuild pristine when they move.
    stamp = build_dir / ".rb-build-inputs"
    fingerprint = json.dumps(
        {
            "config": board.config if board is not None else None,
            "overlays": [str(path) for path in overlays],
            "conf_files": [str(path) for path in conf_files],
            "cmake_args": cmake_args,
        },
        sort_keys=True,
        indent=1,
    )
    effective_pristine = pristine
    if pristine == "auto" and stamp.is_file():
        if stamp.read_text() != fingerprint:
            effective_pristine = "always"
            print(
                "rb: build inputs changed since the last build in this "
                "directory (config, overlays or Kconfig fragments); "
                "rebuilding pristine so the memory map cannot go stale",
                flush=True,
            )
    run(
        [
            "west",
            "build",
            "-b",
            target.zephyr_board,
            "-d",
            build_dir,
            "-p",
            effective_pristine,
            workload.source_dir,
            "--",
            *cmake_args,
        ],
        environment="zephyr",
        cwd=ZEPHYR_BACKEND / "zephyr_ws",
    )
    stamp.write_text(fingerprint)
    elf = zephyr_elf(target, workload, mode, ram_console)
    if not elf.is_file():
        raise RbError(f"west completed but ELF was not found: {elf}")
    record = write_build_record(target, workload, mode, board, elf)
    if workload.mode_note(mode):
        print(f"rb: {workload.name} --mode {mode}: {workload.mode_note(mode)}")
    _warn_legacy(workload, mode)
    print(f"rb: build record {record}")
    if board is not None and (
        board.config in FP16_SCALAR_FPU_CONFIGS
        or board.config in INT_NOPERMUTE_VECTOR_CONFIGS
    ):
        _check_isa_contract(board, elf)
    print(elf)
    return elf


_PERMUTE_MNEMONICS = ("vslide", "vfslide", "vrgather", "vcompress",
                      "vmv1r", "vmv2r", "vmv4r", "vmv8r")


def _is_vector_fp(mnemonic: str) -> bool:
    """Vector floating point: every vf* except vfirst.m, plus the vmf* compares.

    vfirst.m is an integer mask instruction, and vmfeq/vmflt/... are FP compares
    that do not start with "vf". Both were misclassified by a plain "vf" prefix.
    """
    return (mnemonic.startswith("vf") and not mnemonic.startswith("vfirst")) \
        or mnemonic.startswith("vmf")


def isa_contract_offences(listing: str, *, fp16: bool, noperm: bool) -> dict[str, dict[str, int]]:
    """Instructions an FP16 or noPermute shell decodes as illegal, per kind.

    `listing` is `objdump -d --no-show-raw-insn` output. Returns
    {kind: {"<function>: <mnemonic>": count}}; empty means the image is clean.

    FP16 shells run CONFIG_FPU=n, so mstatus.FS stays Off and EVERY floating
    point instruction traps: scalar F/D/Q/Zfh, fcsr access, and vector FP too
    (the vector spec requires FS != Off for any vector FP operation).
    """
    offences: dict[str, dict[str, int]] = {}
    function = "?"
    for line in listing.splitlines():
        header = re.match(r"^[0-9a-f]+ <(.+)>:$", line)
        if header:
            function = header.group(1)
            continue
        fields = line.split("\t")
        if len(fields) < 2 or not fields[0].strip().endswith(":"):
            continue
        mnemonic = fields[1].split()[0] if fields[1].split() else ""
        kind = None
        if fp16 and mnemonic.startswith(("f", "c.f")) and not mnemonic.startswith("fence"):
            kind = "scalar floating point"
        elif (fp16 or noperm) and _is_vector_fp(mnemonic):
            kind = "vector floating point"
        elif noperm and mnemonic.startswith(_PERMUTE_MNEMONICS):
            kind = "vector permute / whole-register move"
        if kind:
            per = offences.setdefault(kind, {})
            key = f"{function}: {mnemonic}"
            per[key] = per.get(key, 0) + 1
    return offences


def _check_isa_contract(board: Board, elf: Path) -> None:
    """Refuse an image containing instructions this shell decodes as illegal.

    FP16 shells (FP16_SCALAR_FPU_CONFIGS): no floating-point opcode at all,
    scalar or vector. CONFIG_FPU=n is necessary and not sufficient. Anything
    that puts "f" or "d" back into a translation unit's -march -- a global "v"
    (GCC expands it to zve64d, which implies d and f), a per-file -march from a
    kernel flag table, a prebuilt object -- produces hardware float that links
    cleanly and traps on the first execution, because Zephyr leaves mstatus.FS
    off on a no-FPU image and the shell's misa has neither F nor D.

    Integer no-permute shells (INT_NOPERMUTE_VECTOR_CONFIGS): no vector FP and
    no vslide*/vfslide*/vrgather*/vcompress/vmv<n>r.v. The last is the one that
    bites: GCC emits vmv4r.v for an ordinary register copy, and DroNet's RVV
    linear_s8 kernel carried one on its main path.

    The only evidence that settles either is the binary, so read the binary.
    """
    fp16 = board.config in FP16_SCALAR_FPU_CONFIGS
    noperm = board.config in INT_NOPERMUTE_VECTOR_CONFIGS
    objdump = ZEPHYR_GNU_BIN / "riscv64-zephyr-elf-objdump"
    if not objdump.is_file():
        print(f"rb: WARNING cannot check {elf.name} against {board.config}: {objdump} missing")
        return
    listing = subprocess.run(
        [str(objdump), "-d", "--no-show-raw-insn", str(elf)],
        capture_output=True, text=True, check=True,
    ).stdout
    offences = isa_contract_offences(listing, fp16=fp16, noperm=noperm)
    if offences:
        lines = []
        for kind, per in offences.items():
            lines.append(f"{sum(per.values())} {kind}:")
            for key, count in sorted(per.items(), key=lambda kv: -kv[1])[:12]:
                lines.append(f"  {count:5d}  {key}")
        raise RbError(
            f"{elf} contains instructions {board.config} decodes as ILLEGAL"
            + (" (FP16-only FPU, mstatus.FS off on a CONFIG_FPU=n image)" if fp16 else "")
            + (" (Saturn intOnlyParams + noPermute)" if noperm else "")
            + ":\n" + "\n".join(lines) + "\n"
            "Find what put them there (compile_commands.json): a global \"v\" from "
            "CONFIG_RISCV_ISA_EXT_V without CONFIG_RISCV_V_KERNEL_ONLY, a kernel "
            "flag table naming v/f/d (use zve64x for integer vector), or a "
            "kernel whose codegen needs a permute -- drop that kernel for this "
            "shell rather than hoping the path is never taken."
        )
    print(f"rb: {elf.name}: no instructions {board.config} decodes as illegal "
          f"({'scalar and vector FP' if fp16 else ''}{' / ' if fp16 and noperm else ''}"
          f"{'vector FP, permutes, vmv<n>r' if noperm else ''} checked)")


def cmd_zephyr(args: argparse.Namespace) -> int:
    target = get_target(args.target)
    workload = get_workload(args.workload)
    mode = args.mode or workload.default_mode
    if args.zephyr_action == "path":
        print(zephyr_elf(target, workload, mode))
        return 0 if zephyr_elf(target, workload, mode).is_file() else 1
    board = target.fpga_board
    if board is not None:
        board = _workload_board(board, workload, args.config)
    elif args.config is not None:
        raise RbError("--config applies only to FPGA targets")
    zephyr_build(target, workload, mode, args.pristine, board)
    return 0


def cmd_zephyr_setup_targets(args: argparse.Namespace) -> int:
    """Enable and fetch the HALs required by all registered Zephyr targets."""
    run(
        ["west", "config", "manifest.file", "west.yml"],
        environment="zephyr",
        cwd=ZEPHYR_BACKEND / "zephyr_ws",
    )
    run(
        ["west", "update", "hal_espressif", "hal_st"],
        environment="zephyr",
        cwd=ZEPHYR_BACKEND / "zephyr_ws",
    )
    run(
        [
            ZEPHYR_PREFIX / "bin" / "python",
            "-m",
            "pip",
            "install",
            "--requirement",
            ZEPHYR_TARGET_REQUIREMENTS,
        ],
        environment="zephyr",
        cwd=ROOT,
    )
    # After the pip install, not before: the fetcher below needs `requests`,
    # which requirements-zephyr-targets.txt supplies.
    #
    # The ESP32 WiFi driver links Espressif binary blobs that the manifest does
    # not carry, so `west update` alone leaves an ESP WiFi build failing at
    # CMake time with "Blob for path .../libble_app.a missing". Fetching them
    # here makes `rb zephyr setup-targets` sufficient for every registered
    # target rather than every target except the one that needs a radio.
    run(
        ["west", "blobs", "fetch", "hal_espressif"],
        environment="zephyr",
        cwd=ZEPHYR_BACKEND / "zephyr_ws",
    )
    print("Zephyr target HALs and Espressif blobs are installed.")
    return 0


def load_elf(
    board: Board,
    workload: Workload,
    mode: str,
    port_arg: str | None,
    baudrate: int,
    ram_console: bool = False,
) -> None:
    if workload.blocked_reason is not None:
        raise RbError(f"{workload.name} is blocked: {workload.blocked_reason}")
    target = get_target(board.name)
    elf = zephyr_elf(target, workload, mode, ram_console)
    if not elf.is_file():
        raise RbError(
            f"Zephyr ELF is missing: {elf}\n"
            f"Run: rb build {workload.name} --target {target.name} --mode {mode}"
            + (" --ram-console" if ram_console else "")
        )
    port_arg = port_arg or os.environ.get("RB_TSI_PORT")
    if port_arg and "://" in port_arg:
        # Through the ESP32 (WiFi, or its USB console as socket:// via a
        # proxy): the FPGA end is EspLinkMux, which needs the esp_bridge
        # load-service handshake that plain pyuartsi does not speak. Only
        # shells built with the ESP loader answer it (e.g.
        # RocketArty200TDroneFullDDRLoaderConfig).
        # esp-usb:///dev/ttyACM0 is the same service through the ESP's USB
        # console -- the bench test that needs no WiFi association.
        link = (
            ["--usb", port_arg[len("esp-usb://"):]]
            if port_arg.startswith("esp-usb://")
            else ["--url", port_arg]
        )
        run(
            [
                ZEPHYR_PREFIX / "bin" / "python",
                ROOT / "tools" / "rb" / "esp_tsi_load.py",
                *link,
                "--elf",
                elf,
            ],
            environment="zephyr",
            cwd=ROOT,
        )
        return
    try:
        port = select_serial_port("tsi", port_arg)
    except (FileNotFoundError, RuntimeError) as exc:
        raise RbError(str(exc)) from exc
    run(
        [
            ZEPHYR_PREFIX / "bin" / "python",
            "-m",
            "pyuartsi",
            "--port",
            port,
            "--baudrate",
            str(baudrate),
            "--elf",
            elf,
            "--load",
            "--selfcheck",
            "--hart0_msip",
        ],
        environment="zephyr",
        cwd=ROOT,
    )


def cmd_load(args: argparse.Namespace) -> int:
    workload = get_workload(args.workload)
    load_elf(
        get_board(args.board),
        workload,
        args.mode or workload.default_mode,
        args.port,
        args.baudrate,
        args.ram_console,
    )
    return 0


def _capture_output_dir(
    target: ZephyrTarget,
    workload: Workload,
    mode: str,
    explicit: str | None,
) -> Path:
    if explicit:
        return Path(explicit)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return (
        ROOT
        / "results"
        / "runs"
        / f"{timestamp}_{target.name}_{workload.name}_{mode}"
    )


def capture_serial(
    target: ZephyrTarget,
    workload: Workload,
    mode: str,
    port_arg: str | None,
    baudrate: int,
    duration: float,
    output_dir: str | None,
) -> None:
    if workload.blocked_reason is not None:
        raise RbError(f"{workload.name} is blocked: {workload.blocked_reason}")
    try:
        role = "esp" if target.kind == "esp" else "console"
        port = select_serial_port(role, port_arg)
    except (FileNotFoundError, RuntimeError) as exc:
        raise RbError(str(exc)) from exc
    elf = zephyr_elf(target, workload, mode)
    if not elf.is_file():
        raise RbError(
            f"Zephyr ELF is missing: {elf}\n"
            f"Run: rb build {workload.name} --target {target.name} --mode {mode}"
        )
    destination = _capture_output_dir(target, workload, mode, output_dir)
    if destination.exists():
        raise RbError(f"capture output already exists: {destination}")
    command = [
        ZEPHYR_PREFIX / "bin" / "python",
        CAPTURE_SERIAL,
        "--root",
        ROOT,
        "--target",
        target.name,
        "--workload",
        workload.name,
        "--mode",
        mode,
        "--port",
        port,
        "--baudrate",
        str(baudrate),
        "--duration",
        str(duration),
        "--output-dir",
        destination,
        "--elf",
        elf,
    ]
    if target.fpga_board_name is not None:
        command.extend(("--board", target.fpga_board_name))
    run(command, environment="zephyr", cwd=ROOT)


def cmd_capture(args: argparse.Namespace) -> int:
    target = get_target(args.target)
    workload = get_workload(args.workload)
    mode = args.mode or workload.default_mode
    if not workload.supports(target):
        raise RbError(
            f"{workload.name} is not enabled for target {target.name}"
        )
    capture_serial(
        target,
        workload,
        mode,
        args.port,
        args.baudrate or target.console_baudrate,
        args.duration,
        args.output_dir,
    )
    return 0


def cmd_console(args: argparse.Namespace) -> int:
    try:
        port = select_serial_port("console", args.port)
    except (FileNotFoundError, RuntimeError) as exc:
        raise RbError(str(exc)) from exc
    # miniterm puts the terminal in raw mode, so it needs a real one. Without
    # this check a piped or agent-driven run died inside the terminal library
    # with `termios.error: (25, 'Inappropriate ioctl for device')`, which names
    # neither the command that failed nor the thing to do about it.
    if not sys.stdin.isatty():
        raise RbError(
            "rb console needs an interactive terminal: it hands the port to "
            "miniterm,\n  which puts the terminal into raw mode.\n"
            f"  To capture {port} from a script or an agent instead:\n"
            f"    stty -F {port} {args.baudrate} raw -echo && cat {port} | tee console.log\n"
            "  Or run this command inside a pseudo-terminal (script -qc ... /dev/null)."
        )
    run(
        [
            ZEPHYR_PREFIX / "bin" / "python",
            "-m",
            "serial.tools.miniterm",
            port,
            str(args.baudrate),
        ],
        environment="zephyr",
        cwd=ROOT,
    )
    return 0


def _wait_for_port(port: int, process: subprocess.Popen[str], timeout: float = 10) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RbError(f"OpenOCD exited early with status {process.returncode}")
        with socket.socket() as sock:
            sock.settimeout(0.2)
            if sock.connect_ex(("127.0.0.1", port)) == 0:
                return
        time.sleep(0.1)
    raise RbError(f"OpenOCD did not open GDB port {port} within {timeout:g}s")


def cmd_debug(args: argparse.Namespace) -> int:
    target = get_target(args.target)
    if target.fpga_board is None:
        raise RbError("OpenOCD/GDB debug is currently available only for FPGA targets")
    workload = get_workload(args.workload)
    board = _workload_board(target.fpga_board, workload, args.config)
    mode = args.mode or workload.default_mode
    elf = zephyr_build(
        target, workload, mode, args.pristine, board,
        ram_console=args.ram_console,
    )
    if args.program:
        # Deliberately not args.usb_location: that pins the *debug* adapter,
        # and programming has to go to the config TAP on the other one.
        program_fpga(board, args.tool, args.program_usb_location or "")

    adapter = Path(args.adapter_config) if args.adapter_config else FT_LINK_CONFIG
    if not adapter.is_file():
        raise RbError(f"OpenOCD adapter configuration is missing: {adapter}")

    # Selecting the adapter by VID:PID alone is a coin flip on a bench with
    # more than one FTDI, and losing it is indistinguishable from a wiring
    # fault. Refuse to guess rather than emit a confusing all-ones scan.
    location = args.usb_location or os.environ.get("RB_FTDI_LOCATION", "")
    candidates = [a for a in ftdi_adapters() if a["pid"] in MPSSE_PRODUCT_IDS]
    if not location and len(candidates) > 1:
        listed = ", ".join(a["location"] for a in candidates)
        raise RbError(
            f"{len(candidates)} FTDI adapters are attached ({listed}), so "
            "matching on VID:PID alone would pick one at random. Pass "
            "--usb-location (or set RB_FTDI_LOCATION); run `rb probe-ftdi "
            "--idcode` to see which one reaches the debug chain."
        )

    environment = zephyr_env()
    if location:
        environment["RB_FTDI_LOCATION"] = location

    gdb = ZEPHYR_GNU_BIN / "riscv64-zephyr-elf-gdb"
    command = openocd_command(
        "-f",
        str(adapter),
        # Must precede the target config: that file ends in `init`, and
        # gdb_port is only settable beforehand. Without this, --gdb-port moved
        # the port rb waits on but not the one OpenOCD listens on.
        "-c",
        f"gdb_port {args.gdb_port}",
        "-f",
        str(board.openocd_target),
    )
    print("+", " ".join(command), flush=True)
    if location:
        print(f"+ RB_FTDI_LOCATION={location}", flush=True)
    process = subprocess.Popen(
        command,
        cwd=ROOT,
        env=environment,
        text=True,
        # OpenOCD reads commands from stdin, and so does GDB. With both
        # inheriting the same descriptor a piped command script is consumed by
        # whichever gets there first: a batch readback fed on stdin had its
        # `break` executed twice and every later line silently swallowed.
        # OpenOCD needs no input here.
        stdin=subprocess.DEVNULL,
    )
    try:
        _wait_for_port(args.gdb_port, process)
        gdb_command = [
            str(gdb),
            str(elf),
            "-ex",
            f"target extended-remote localhost:{args.gdb_port}",
            "-ex",
            "monitor halt",
        ]
        if not args.no_load:
            gdb_command.extend(["-ex", "load"])
        # Start the workload rather than parking at the prompt.
        #
        # `load` alone leaves the core halted at the entry point, so an
        # interactive session showed nothing until the user typed something --
        # and the obvious thing to type, `run`, is wrong on a remote target:
        # it asks "Start it from the beginning?" and needs a `y`. `continue` is
        # the right verb and there is no reason to make anyone type it.
        #
        # Skipped when a script is supplied: a script sets its own breakpoints
        # before continuing, and resuming underneath it would run the program
        # past them.
        if args.gdb_script is None and not args.no_run and not args.no_load:
            # The program starts HERE, the moment this GDB runs -- and the
            # console is a different channel, opened by a different command in
            # a different shell. Anything the workload prints before someone
            # attaches to channel B is gone.
            #
            # That is a race the operator loses on a fast workload and wins on
            # a slow one, which is exactly the pattern the bench saw and read
            # as a flaky UART: a 15.79 s DroNet run yielded 978 lossy bytes, and
            # the same image at 188 ms yielded 15 -- the reproducible reset
            # glitch, and none of the program's output. The console was fine
            # both times; nobody was listening yet the second time.
            if not args.ram_console:
                print(
                    "rb: the workload starts now, and its console output goes "
                    "to channel B as it is produced.\n"
                    "    Anything printed before you attach is LOST. A short "
                    "workload can finish before you get there.\n"
                    "    Either attach first --\n"
                    "        rb debug ... --no-run   (then `continue` at the "
                    "gdb prompt once the console is up)\n"
                    "    or use --ram-console, which buffers output in RAM and "
                    "is read back over this same JTAG link."
                )
            gdb_command.extend(["-ex", "continue"])
        # -x rather than stdin: a script on stdin competes with GDB's own
        # prompt handling, and `--batch` guarantees the session ends rather
        # than dropping to an interactive prompt no one is watching.
        if args.gdb_script:
            script = Path(args.gdb_script)
            if not script.is_file():
                raise RbError(f"GDB script is missing: {script}")
            gdb_command.extend(["-x", str(script), "--batch"])
        return subprocess.call(gdb_command, cwd=ROOT, env=zephyr_env())
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def cmd_build(args: argparse.Namespace) -> int:
    target = get_target(args.target)
    workload = get_workload(args.workload)
    board = target.fpga_board
    if board is not None:
        board = _workload_board(board, workload, args.config)
    elif args.config is not None:
        raise RbError("--config applies only to FPGA targets")
    zephyr_build(
        target,
        workload,
        args.mode or workload.default_mode,
        args.pristine,
        board,
        ram_console=args.ram_console,
    )
    return 0


# FTDI adapters on a drone bench are not distinguishable by descriptor: the
# dev board's two FT2232Hs both ship with a blank EEPROM and report identical
# manufacturer, product and (empty) serial strings. USB topology is the only
# discriminator, and the sysfs directory name is already in the bus-port.port
# form that OpenOCD's `adapter usb location` expects.
FTDI_VENDOR = "0403"
USB_DEVICES = Path("/sys/bus/usb/devices")

# The MPSSE-capable FTDI parts the benches use on channel A: 0x6010 is the
# FT2232H ("Dual RS232-HS"), 0x6011 the FT4232H ("Quad RS232-HS"). Both appear
# on the drone bench and the two are interchangeable for JTAG, so every place
# that counts or probes adapters has to accept either. Matching one PID makes
# the other invisible to the ambiguity guard below, which is worse than not
# guarding at all: it lets OpenOCD pick a different device and report the
# all-ones scan that reads as a wiring fault.
MPSSE_PRODUCT_IDS = ("6010", "6011")


def _sysfs_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return ""


def macos_serial_node(bus: int, ports: Sequence[int], interface: int, serial: str = "",
                      exists=os.path.exists) -> str | None:
    """The /dev/cu.* node macOS creates for one USB serial interface, or None.

    Two naming schemes, both measured on the drone bench Mac:
      * with a USB serial number:    cu.usbserial-<serial><A|B...>  (A = interface 0)
      * with none (the drone's blank-EEPROM FT2232H pair, and the ESP32-C6's CDC port):
        cu.usbserial-<L><NN> / cu.usbmodem-<L><NN>, where <L> is the IORegistry locationID
        in hex with its trailing zeros dropped and <NN> the interface number, two digits.
        locationID packs the libusb bus into the top byte and one port per nibble below it:
        bus 1 ports 1.3 -> 0x01130000 -> "113" -> cu.usbserial-11300 / -11301.
    Only a node that exists is returned: a scheme that does not match this host must fail
    closed (None), never hand back a plausible path to the wrong device.
    """
    prefix = artifacts.macos_location_prefix(bus, ports)
    candidates = []
    if serial and serial != "(none)":
        candidates.append(f"/dev/cu.usbserial-{serial}{chr(ord('A') + interface)}")
    candidates += [f"/dev/cu.usbserial-{prefix}{interface:02d}",
                   f"/dev/cu.usbmodem{prefix}{interface:02d}"]
    return next((c for c in candidates if exists(c)), None)


def _ftdi_adapters_libusb() -> list[dict[str, str]]:
    """FTDI discovery for hosts with no sysfs -- macOS, principally.

    OpenOCD's `adapter usb location` takes a bus-and-port path like 7-1.1.4.1.
    On Linux that string is the sysfs directory name; elsewhere it has to be
    rebuilt. libusb reports the same bus number and port chain OpenOCD's own
    libusb backend sees, so the location computed here is the one OpenOCD will
    match -- which is the whole reason for using libusb rather than parsing
    system_profiler, whose Location ID is a different numbering.

    Interface-to-tty mapping has no kernel link on macOS (that is a Linux
    ftdi_sio property), but the driver's node NAMES are deterministic -- see
    macos_serial_node(). Each interface gets the node that name predicts only if
    that node exists; otherwise it stays "?" and callers treat it as unknown.
    """
    try:
        import usb.core  # type: ignore
    except ImportError:
        raise RbError(
            "FTDI discovery on this host needs pyusb (no /sys/bus/usb).\n"
            "Install it, plus libusb:\n"
            "  brew install libusb && pip install pyusb"
        )

    found = []
    for device in usb.core.find(find_all=True, idVendor=int(FTDI_VENDOR, 16)):
        ports = getattr(device, "port_numbers", None) or ()
        location = f"{device.bus}-{'.'.join(str(p) for p in ports)}" if ports else str(device.bus)

        def _describe(index: int) -> str:
            # Reading string descriptors needs the device to be claimable; a
            # kernel driver already holding it makes this raise rather than
            # return, and an unreadable name must not abort the whole listing.
            try:
                return usb.util.get_string(device, index) or ""
            except Exception:
                return ""

        serial = _describe(device.iSerialNumber)
        channels = []
        try:
            for interface in device[0]:
                number = interface.bInterfaceNumber
                node = macos_serial_node(device.bus, ports, number, serial) if ports else None
                channels.append(f"if{number}={node[len('/dev/'):] if node else '?'}")
        except Exception:
            pass

        found.append(
            {
                "location": location,
                "bus": str(device.bus),
                "device": str(device.address),
                "pid": f"{device.idProduct:04x}",
                "manufacturer": _describe(device.iManufacturer),
                "product": _describe(device.iProduct),
                "serial": serial or "(none)",
                "ttys": " ".join(sorted(set(channels))) or "(none)",
                "detached": False,
            }
        )
    return sorted(found, key=lambda a: a["location"])


def ftdi_adapters() -> list[dict[str, str]]:
    """Every attached FTDI device, keyed by the USB location OpenOCD wants."""

    # sysfs is richer where it exists -- it is the only source that ties a USB
    # interface to its ttyUSB node, which is what makes the "my console
    # vanished" asymmetry visible. Fall back only when there is no sysfs.
    if not USB_DEVICES.is_dir():
        return _ftdi_adapters_libusb()

    found = []
    for entry in sorted(USB_DEVICES.glob("*")):
        if _sysfs_text(entry / "idVendor") != FTDI_VENDOR:
            continue
        # Per interface, not per device: OpenOCD drives JTAG on channel A by
        # detaching ftdi_sio from interface 0, which makes that ttyUSB node
        # disappear while the console on interface 1 keeps working. Reporting
        # the whole device as one line hides that asymmetry, and "my console
        # vanished" is the most common way it gets noticed.
        channels = []
        for interface in sorted(entry.glob(f"{entry.name}:*")):
            number = _sysfs_text(interface / "bInterfaceNumber").lstrip("0") or "0"
            tty = next((t.name for t in interface.glob("ttyUSB*")), None)
            if tty is not None:
                channels.append(f"if{number}={tty}")
            elif (interface / "driver").exists():
                channels.append(f"if{number}=bound")
            else:
                channels.append(f"if{number}=detached")
        ttys = channels
        found.append(
            {
                "location": entry.name,
                "bus": _sysfs_text(entry / "busnum"),
                "device": _sysfs_text(entry / "devnum"),
                "pid": _sysfs_text(entry / "idProduct"),
                "manufacturer": _sysfs_text(entry / "manufacturer"),
                "product": _sysfs_text(entry / "product"),
                "serial": _sysfs_text(entry / "serial") or "(none)",
                "ttys": " ".join(ttys) or "(none)",
                "detached": any(c.endswith("=detached") for c in ttys),
            }
        )
    return found


def cmd_probe_ftdi(args: argparse.Namespace) -> int:
    adapters = ftdi_adapters()
    if not adapters:
        print("no FTDI devices attached")
        return 1
    # Include the header, or a short location like "5-2" leaves the column
    # narrower than the word LOCATION and every row shifts left of its heading.
    width = max(len("LOCATION"), *(len(a["location"]) for a in adapters))
    print(f"{'LOCATION'.ljust(width)}  BUS:DEV  VID:PID    SERIAL        INTERFACES")
    for adapter in adapters:
        print(
            f"{adapter['location'].ljust(width)}  "
            f"{adapter['bus']}:{adapter['device']:<5}  "
            f"{FTDI_VENDOR}:{adapter['pid']}  "
            f"{adapter['serial'][:12]:<12}  {adapter['ttys']}"
        )
        print(f"{' ' * width}  {adapter['manufacturer']} / {adapter['product']}")
    if any(a["detached"] for a in adapters):
        print()
        print("'detached' means ftdi_sio is not bound to that interface, which is")
        print("what OpenOCD leaves behind on the JTAG channel. Its ttyUSB node is")
        print("gone until the adapter is replugged, or rebound with:")
        print("  echo <location>:1.<n> | sudo tee /sys/bus/usb/drivers/ftdi_sio/bind")
    if not args.idcode:
        print()
        print("Pass --idcode to shift a JTAG IDCODE out of each adapter, which")
        print("is what tells the bitstream chain apart from the debug chain.")
        return 0

    wanted = set(args.location or ())
    if wanted:
        unknown = wanted - {a["location"] for a in adapters}
        if unknown:
            raise RbError(
                "no attached adapter at "
                + ", ".join(sorted(unknown))
                + ".\n  Locations change on every replug -- take them from the "
                "table above,\n  not from a previous run."
            )

    print()
    print("Probing IDCODE. This detaches ftdi_sio, so the ttyUSB nodes above")
    print("disappear until the adapter is replugged or rebound.")
    unreadable = 0
    if not wanted:
        # A scan is not read-only from the point of view of whoever else is on
        # this host: it drives TCK/TMS on every adapter it touches. Say which
        # ones, so an unexpected name is noticed before the scan and not after.
        print("Scanning every MPSSE adapter: "
              + ", ".join(a["location"] for a in adapters
                          if a["pid"] in MPSSE_PRODUCT_IDS))
        print("Pass --location to restrict this on a shared host.")
    for adapter in adapters:
        if adapter["pid"] not in MPSSE_PRODUCT_IDS:
            continue
        if wanted and adapter["location"] not in wanted:
            print(f"{adapter['location'].ljust(width)}  (skipped, not requested)")
            continue
        idcode = probe_idcode(adapter["location"])
        print(f"{adapter['location'].ljust(width)}  {idcode}")
        if idcode.startswith(("cannot open", "no IDCODE", "probe failed")):
            unreadable += 1
    if unreadable:
        print()
        print(f"{unreadable} adapter(s) could not be probed. Exiting non-zero: a "
              "scan that\nnever reached the hardware is not the same as a chain "
              "that answered nothing.")
        return 1
    return 0


def _tty_for(adapter: dict, interface: int) -> str:
    """Pull `ifN=ttyUSBn` out of the adapter's interface summary.

    Returns the /dev path, or the literal 'detached' when OpenOCD has taken
    that interface, which is normal for the JTAG channel and not a fault.
    """

    match = re.search(rf"if{interface}=(\S+)", adapter.get("ttys", ""))
    if match is None or match.group(1) == "?":
        # "?" is libusb discovery saying "no node found": returning /dev/? would pass
        # every startswith("/dev/") check downstream and get opened as a console.
        return "-"
    node = match.group(1)
    return node if node == "detached" else f"/dev/{node}"


def cmd_bench(args: argparse.Namespace) -> int:
    """Print the numbers every bench command needs, with roles resolved.

    `rb probe-ftdi` lists adapters; this answers the question actually being
    asked at the prompt -- which location and bus:device go in which flag, and
    which ttyUSB is the console. Roles cannot be inferred from topology: the
    bitstream and debug locations have swapped between sessions on this bench,
    so they are resolved by shifting an IDCODE unless --no-probe says otherwise.
    """

    adapters = [a for a in ftdi_adapters() if a["pid"] in MPSSE_PRODUCT_IDS]
    if not adapters:
        print("no MPSSE-capable FTDI adapters attached")
        return 1

    rows = []
    for adapter in adapters:
        if args.no_probe:
            idcode = "(not probed)"
        elif is_foreign_adapter(adapter) and not args.all:
            idcode = f"(not probed: {adapter['manufacturer']} board; --all probes it)"
        else:
            idcode = probe_idcode(adapter["location"])
        if "config TAP" in idcode and "Alveo" not in idcode:
            role = "bitstream"
        elif "Rocket debug TAP" in idcode:
            role = "debug"
        elif "Alveo" in idcode or idcode.startswith("(not probed:"):
            role = "other"
        elif "all ones" in idcode:
            # The debug adapter reads all ones whenever no design is running,
            # which is a state, not an identity. Say so rather than guess.
            role = "debug?"
        else:
            role = "?"
        rows.append((role, adapter, idcode))

    width = max([len(a["location"]) for a in adapters] + [len("LOCATION")])
    print(f"{'ROLE':<10} {'LOCATION'.ljust(width)}  {'BUS:DEV':<8} {'JTAG chA':<14} {'CONSOLE chB':<14} IDCODE")
    for role, a, idcode in rows:
        print(
            f"{role:<10} {a['location'].ljust(width)}  "
            f"{a['bus'] + ':' + a['device']:<8} "
            f"{_tty_for(a, 0):<14} {_tty_for(a, 1):<14} {idcode}"
        )

    bit = next((r for r in rows if r[0] == "bitstream"), None)
    dbg = next((r for r in rows if r[0].startswith("debug")), None)
    print()
    print("Ready to paste:")
    if bit:
        b = bit[1]
        print(f"  openFPGALoader -b te0712_8 --busdev-num {b['bus']}:{b['device']} \\")
        print(f"      --write-sram --bitstream <file.bit>")
    else:
        print("  (no bitstream adapter found -- nothing can program the FPGA)")
    if dbg:
        d = dbg[1]
        print(f"  rb debug <workload> --target arty200t --config <Config> \\")
        print(f"      --usb-location {d['location']}")
        console = _tty_for(d, 1)
        if console not in ("-", "detached"):
            print(f"  rb console --port {console} --baudrate 115200")
    else:
        print("  (no debug adapter found)")
    print()
    print("bus:device changes on every replug or USB reset; the location does not.")
    return 0


# FTDI boards whose EEPROM names them as someone else's: Digilent Arty/Nexys boards and the Alveo
# card's own FT4232H. The drone's programmer board carries two blank-EEPROM FT2232Hs (manufacturer
# "FTDI", product "Dual RS232-HS", no serial). An IDCODE probe detaches ftdi_sio from channel A,
# so shifting a scan through a board that is not the drone's takes its JTAG away from whoever is
# using it -- on a shared host that is other people's hardware.
FOREIGN_FTDI_MANUFACTURERS = ("Digilent", "Xilinx")


def is_foreign_adapter(adapter: dict[str, str]) -> bool:
    """True for an adapter whose EEPROM says it belongs to a vendor board, not the drone."""

    return adapter.get("manufacturer", "") in FOREIGN_FTDI_MANUFACTURERS


def probe_idcode(location: str) -> str:
    """Shift one IDCODE out of the adapter at this USB location.

    0x13636093 is the xc7a200t configuration TAP, so that adapter is the
    bitstream link. 0x00000001 is Rocket's debug transport module, whose
    IDCODE is all-default (JtagDTMKeyDefault sets version, part and manufacturer
    to zero), so that adapter reaches the RISC-V debug chain. All-ones means
    nothing is driving TDO.
    """

    command = openocd_command(
        "-c", "adapter driver ftdi",
        "-c", "ftdi vid_pid 0x0403 0x6010 0x0403 0x6011",
        "-c", f"adapter usb location {location}",
        "-c", "ftdi channel 0",
        "-c", "ftdi layout_init 0x0B00 0x0B0B",
        "-c", "adapter speed 1000",
        "-c", "transport select jtag",
        "-c", "reset_config none",
        "-c", "jtag newtap probe tap -irlen 5 -expected-id 0",
        "-c", "init",
        "-c", "shutdown",
    )
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            env=zephyr_env(),
            cwd=ROOT,
            timeout=30,
        )
    except (subprocess.TimeoutExpired, OSError) as exc:
        return f"probe failed: {exc}"
    match = re.search(r"tap/device found: (0x[0-9a-f]+)", result.stderr)
    if match is None:
        if "all ones" in result.stderr:
            return "all ones (nothing driving TDO)"
        # Distinguish "the scan ran and the chain was silent" from "the scan
        # never happened". Reporting both as `no IDCODE returned` and exiting 0
        # made a permissions failure look like a probed-and-empty chain: the
        # command printed a line per adapter and succeeded, having touched
        # nothing. OpenOCD names the cause; pass it through.
        if "LIBUSB_ERROR_ACCESS" in result.stderr or "permission" in result.stderr.lower():
            return ("cannot open the adapter (permission denied) -- this is not "
                    "a JTAG result; see setup.md 3c-bis")
        if "unable to open ftdi device" in result.stderr:
            return ("cannot open the adapter -- in use by another process, or no "
                    "device access; this is not a JTAG result")
        return "no IDCODE returned (the scan produced no tap/device line)"
    idcode = match.group(1)
    known = {
        "0x13636093": "xc7a200t config TAP -- bitstream link",
        "0x13631093": "xc7a100t config TAP -- bitstream link",
        "0x13822093": "xcku040 config TAP -- bitstream link",
        # From OpenOCD's own tcl/cpld/xilinx-xcu.cfg, which lists
        # `XCU250 {0x04b57093 24}`. This is the Alveo card, not part of the
        # drone bench, but --idcode reaches it and an unlabelled IDCODE reads
        # as a fault.
        "0x04b57093": "xcu250 config TAP -- Alveo card, not the drone bench",
        "0x00000001": "Rocket debug TAP -- Zephyr/GDB link",
    }
    return f"{idcode}  {known.get(idcode, 'unrecognised')}"


# Xilinx 7-series configuration STATUS register, UG470 Table 5-27. The bit
# positions matter more than they look:
#
#   bit  3  DONE        the DONE pin -- configuration finished
#   bit  4  RELEASE_DONE
#   bit 13  EOS         startup sequencer reached end-of-startup
#   bit 14  DCI_MATCH   every DCI-using bank matched its reference impedance
#
# These were previously read as DONE=bit 14 and EOS=bit 4, which is DCI_MATCH
# and RELEASE_DONE. That passed a scratchpad shell by luck -- it happens to
# report DCI_MATCH=1 -- and rejected every MIG shell, because the MIG drives
# DDR3 banks with DCI and a board whose DCI reference does not match reports
# DCI_MATCH=0 while still being fully configured. `RocketArty200TDroneDDRConfig`
# reads 0x5000190c: DONE=1, EOS=0, DCI_MATCH=0. That part is configured and its
# debug TAP answers; the old decode called it "never finished configuring" and
# aborted `rb program` before `rb debug` could reach it.
#
# DONE is the gate. EOS and DCI_MATCH are reported for diagnosis only, because a
# design can be legitimately up with either of them low.
CONFIG_STATUS_DONE = 1 << 3
CONFIG_STATUS_EOS = 1 << 13
CONFIG_STATUS_DCI_MATCH = 1 << 14


def fpga_config_status(location: str) -> int | None:
    """Read the Xilinx configuration STATUS register over the config TAP.

    This goes to the *bitstream* adapter, not the debug one. The config TAP is
    alive whenever the part has power, configured or not, which is exactly what
    makes it the right place to ask whether configuration succeeded. Returns
    None when the register cannot be read at all -- an unpowered board or a
    disconnected JTAG link, which is a different failure from DONE=0.
    """

    command = openocd_command(
        "-c", "adapter driver ftdi",
        "-c", "ftdi vid_pid 0x0403 0x6010 0x0403 0x6011",
        "-c", f"adapter usb location {location}",
        "-c", "ftdi channel 0",
        "-c", "ftdi layout_init 0x0B00 0x0B0B",
        "-c", "adapter speed 1000",
        "-c", "transport select jtag",
        "-c", "reset_config none",
        # -expected-id 0 accepts any IDCODE, so this works unchanged on the
        # 100T and the 200T rather than pinning one part number.
        "-c", "jtag newtap xc7 tap -irlen 6 -expected-id 0",
        "-c", "pld device virtex2 xc7.tap 1",
        "-c", "init",
        "-c", "virtex2 read_stat 0",
        "-c", "shutdown",
    )
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            env=zephyr_env(),
            cwd=ROOT,
            timeout=30,
        )
    except (subprocess.TimeoutExpired, OSError):
        return None
    match = re.search(
        r"status register: (0x[0-9a-f]+)", result.stderr + result.stdout
    )
    if match is None:
        return None
    status = int(match.group(1), 16)
    # All-ones is what an unpowered or unreachable target shifts out, not a
    # status word. Reporting it as "DONE=1" would be exactly backwards.
    if status == 0xFFFFFFFF:
        return None
    return status


def require_fpga_configured(location: str, timeout: float = 40.0) -> None:
    """Fail unless the part finishes configuring, rather than later and opaquely.

    Writing the flash succeeding says nothing about whether the FPGA can boot
    from it: the image reads back byte-identical while DONE stays low. Left
    unchecked that surfaces much later as an `all ones` scan on the debug
    adapter, which reads like a broken cable. Checking here attributes it.
    """

    deadline = time.monotonic() + timeout
    status = None
    while True:
        status = fpga_config_status(location)
        if status is not None and status & CONFIG_STATUS_DONE:
            return
        if time.monotonic() >= deadline:
            break
        # Each read spawns an OpenOCD run, and back-to-back runs on this bench
        # leave the MPSSE engine dirty enough that the second and later reads
        # return garbage. Spacing them keeps a retry meaningful instead of
        # turning a slow-but-fine part into a wall of bad reads.
        time.sleep(1.0)
    if status is None:
        raise RbError(
            "the FPGA configuration status register could not be read after "
            f"programming (adapter {location}). The config TAP answers "
            "whenever the part has power, so this points at board power or "
            "the JTAG link, not at the bitstream. Check that the FPGA board "
            "is powered and the ribbon is seated."
        )
    raise RbError(
        f"the FPGA never finished configuring: STATUS=0x{status:08x} "
        f"(DONE={(status >> 3) & 1}, EOS={(status >> 13) & 1}, "
        f"DCI_MATCH={(status >> 14) & 1}). The bitstream was written, but the "
        "part cannot boot from it, so the Rocket debug TAP does not exist and "
        "`rb debug` would report an all-ones scan. This is what a missing "
        "CONFIG_MODE/SPI_BUSWIDTH/SPI_32BIT_ADDR set looks like -- see "
        "fpga-shells/xilinx/arty_a7_200/constraints/arty-config.xdc. Note that "
        "only DONE gates this check: a MIG shell reporting DCI_MATCH=0 or "
        "EOS=0 with DONE=1 is configured and is not what this error is about."
    )


def cmd_program(args: argparse.Namespace) -> int:
    program_fpga(
        _configured_board(args), args.tool, getattr(args, "usb_location", "") or ""
    )
    return 0


def flash_esp(
    target: ZephyrTarget,
    workload: Workload,
    mode: str,
    port_arg: str | None,
) -> None:
    if workload.blocked_reason is not None:
        raise RbError(f"{workload.name} is blocked: {workload.blocked_reason}")
    if target.kind != "esp":
        raise RbError(f"target {target.name} does not use the ESP flash runner")
    build_dir = zephyr_build_dir(target, workload, mode)
    elf = zephyr_elf(target, workload, mode)
    if not elf.is_file():
        raise RbError(
            f"Zephyr ELF is missing: {elf}\n"
            f"Run: rb build {workload.name} --target {target.name} --mode {mode}"
        )
    try:
        port = select_serial_port("esp", port_arg)
    except (FileNotFoundError, RuntimeError) as exc:
        raise RbError(str(exc)) from exc
    run(
        [
            "west",
            "flash",
            "-d",
            build_dir,
            "--",
            "--esp-device",
            port,
        ],
        environment="zephyr",
        cwd=ZEPHYR_BACKEND / "zephyr_ws",
    )


def _load_study(name: str) -> dict:
    path = STUDIES_DIR / f"{name}.json"
    if not path.is_file():
        available = sorted(p.stem for p in STUDIES_DIR.glob("*.json"))
        raise RbError(
            f"unknown study {name!r}; available: {', '.join(available) or 'none'}"
        )
    study = json.loads(path.read_text())
    blocked = study.get("blocked_reason")
    if blocked:
        raise RbError(
            f"study {name!r} cannot run yet: {blocked}\n"
            f"See {path.relative_to(ROOT)} for what it needs."
        )
    configs = study.get("configs") or []
    if not configs:
        raise RbError(f"study {name!r} defines no configs")
    return study


def area_run_one(board: Board, attrs: str, force: bool) -> AreaRecord:
    """Synthesize one config, then record its utilization with provenance."""
    command = area.make_command(board, attrs, force)
    run(command, environment="chipyard", cwd=ROOT)
    record = area.collect_record(board, attrs, command)
    out = area.write_record(record)
    print(f"recorded {record.run_id}")
    print(f"  {out.relative_to(ROOT)}/record.json")
    if record.fingerprint:
        got = fingerprint.Fingerprint.from_dict(record.fingerprint)
        print(f"  SoC: {got.summary()}")
    for line in area.describe_mutation(record):
        print(f"  {line}")
    luts = record.luts
    if luts is not None:
        entry = record.totals.get("luts") or {}
        available = entry.get("available")
        suffix = f" of {available:,} ({luts / available * 100:.1f}%)" if available else ""
        print(f"  LUTs: {luts:,}{suffix}")
    return record


# The pairing the shipped flight profiles were captured under, used only when
# no --config is given and there is therefore nothing to derive a machine from.
LEGACY_HW = ("gemmini_q31", "V256D128_rvv")


DSE_SCRIPTS = ROOT / "dse" / "scripts"
DSE_PLOTS = ROOT / "dse" / "plots"


def _require_main_backend(path: Path, what: str, command: str) -> None:
    """One line, not a traceback, when an experimental backend is not here.

    The research facets -- XPU-RT scheduling, the DSE scripts and data, RoSE --
    live on main. The setup branch carries the drone stack only, so on it these
    commands can only say where the rest is.
    """
    if path.exists():
        return
    raise RbError(
        f"{command} needs {what} at {path.relative_to(ROOT)}, which is not "
        "here: it lives on main, not on the setup branch"
    )


def _run_plot_script(script: str, argv: list[str]) -> int:
    """Invoke a facet-local plotting script with the interpreter running us.

    The scripts stay under ``dse/scripts`` rather than moving into ``tools/rb``:
    they are specific to this DSE facet, and shelling out keeps one source of
    truth instead of a second copy of the rendering code.
    """
    path = DSE_SCRIPTS / script
    _require_main_backend(DSE_SCRIPTS, "the DSE plotting scripts", "this command")
    if not path.is_file():
        raise RbError(f"plotting script missing: {path}")
    return subprocess.call([sys.executable, str(path), *argv], cwd=ROOT)


def _resolve_profile(target: str, workload: str) -> xpurt.Profile:
    """The one discovered profile for a (target, workload) pair."""
    matches = [
        p for p in xpurt.discover_profiles()
        if p.target == target and p.model == workload
    ]
    if not matches:
        known = sorted({(p.target, p.model) for p in xpurt.discover_profiles()})
        listing = "\n".join(f"    {t} / {m}" for t, m in known)
        raise RbError(
            f"no profile for {workload} on target {target}. Discovered:\n{listing}"
        )
    if len(matches) > 1:
        raise RbError(
            f"{len(matches)} profiles match {workload} on {target}; "
            f"hw values: {', '.join(sorted(m.hw for m in matches))}"
        )
    return matches[0]


def cmd_xpurt(args: argparse.Namespace) -> int:
    """Schedule a workload mix on an XPU-RT hardware pairing and plot it."""
    action = args.xpurt_action

    if action == "doctor":
        return xpurt.forest_doctor(
            ready=args.ready,
            python=args.python,
            dronet=args.dronet_weights,
            checkpoint=args.checkpoint,
            schedule=args.schedule,
        )

    if action in {"summarize", "plot"}:
        run_dir = args.run_dir.resolve()
        try:
            summary = xpurt.refresh_forest_summary(run_dir, getattr(args, "out", None))
        except (OSError, ValueError, KeyError, xpurt.XpurtError) as exc:
            raise RbError(f"cannot refresh XPU-RT forest run {run_dir}: {exc}") from exc
        if action == "summarize":
            print(json.dumps(summary, indent=2, sort_keys=True))
        else:
            print(args.out or run_dir / "plot.svg")
        return 0

    if action == "ops":
        primary = _resolve_profile(args.target, args.workload)
        argv = ["--profile", f"{args.label or args.target}={primary.results}"]
        if args.compare:
            other = _resolve_profile(args.compare, args.workload)
            label = args.compare_label or args.compare
            argv += ["--profile", f"{label}={other.results}"]
        out = args.out or DSE_PLOTS / f"{args.workload}_{args.target}_ops.svg"
        argv += [
            "--out", str(out),
            "--title", args.title or f"{args.workload} per-op cycles",
        ]
        return _run_plot_script("plot_op_cycles.py", argv)

    if action == "gantt":
        schedule = args.schedule
        if schedule is None:
            if not (args.config and args.workload):
                raise RbError(
                    "rb xpurt gantt needs either --schedule, or --config and "
                    "--workload to locate one under dse/results/xpurt/schedules"
                )
            stem = f"networks_{args.workload}_{args.config}"
            found = sorted(
                (xpurt.WORKSPACE / "schedules").glob(
                    f"scheduled_{stem}_{args.solver}*.json"
                )
            )
            if not found:
                raise RbError(
                    f"no schedule matching {stem} with solver {args.solver}. Run: "
                    f"rb xpurt run --workloads {args.workload} "
                    f"--config {args.config} --solver {args.solver}"
                )
            schedule = found[-1]
        stem = schedule.stem.replace("scheduled_", "")
        out = args.out or DSE_PLOTS / f"{stem}_gantt.svg"
        argv = ["--schedule", str(schedule), "--out", str(out)]
        if args.title:
            argv += ["--title", args.title]
        if args.deadline_ms is not None:
            argv += ["--deadline-ms", str(args.deadline_ms)]
        return _run_plot_script("plot_gantt.py", argv)

    if action == "profiles":
        profiles = sorted(xpurt.discover_profiles(), key=lambda p: p.key)
        graphs = sorted(xpurt.discover_graphs(), key=lambda g: (g.model, g.target))
        print(f"per-op profiles ({len(profiles)}):")
        print(f"  {'hw':22s} {'target':24s} {'model':16s} precision")
        for p in profiles:
            print(f"  {p.hw:22s} {p.target:24s} {p.model:16s} {p.basename}")
        print(f"\ndispatch graphs ({len(graphs)}):")
        for g in graphs:
            print(f"  {g.model:16s} {g.target:24s} {g.backend:10s} {g.basename}")
        available = sorted(p.stem for p in xpurt.WORKLOAD_DIR.glob("*.json"))
        print(f"\nworkload descriptors in {xpurt.WORKLOAD_DIR.relative_to(ROOT)}:")
        print(f"  {', '.join(available) or 'none'}")

        # A profile is only reusable on a machine whose fingerprint matches the
        # one it was captured on, so say what that machine is.
        targets = sorted({p.target for p in profiles})
        print(f"\nprofile targets ({len(targets)}):")
        for name in targets:
            known = xpurt.load_profile_target(name)
            if known is None:
                print(f"  {name:24s} no fingerprint declared "
                      f"(add {xpurt.PROFILE_TARGET_DIR.relative_to(ROOT)}/{name}.json)")
                continue
            captured = next(
                (c for c in (xpurt.derive_capture_clock(p) for p in profiles
                             if p.target == name) if c), None
            )
            clock = f", {captured:g} MHz captured" if captured else ""
            print(f"  {name:24s} {known.summary()}{clock}")
        return 0

    if action == "run":
        workloads = [w.strip() for w in args.workloads.split(",") if w.strip()]
        mix = args.mix or "_".join(workloads)
        name = f"{mix}_{args.config or args.hw_p or LEGACY_HW[0]}"

        _require_main_backend(xpurt.XPURT, "the XPU-RT backend", "rb xpurt run")
        try:
            ws = xpurt.build_workspace(force=args.rebuild_workspace)
        except xpurt.XpurtError as exc:
            raise RbError(str(exc)) from exc
        relevant = [p for p in xpurt.discover_profiles() if p.target == args.target]

        # What machine is this schedule claiming to describe? A config name is
        # not evidence, so the answer comes from its elaborated DTS and
        # gen-collateral, and the clock comes with it.
        config_fp = None
        clock_mhz = args.clock_mhz
        clock_source = "--clock-mhz" if clock_mhz else None
        if args.config:
            board = get_board(args.board).with_config(args.config)
            try:
                config_fp = fingerprint.for_board(board)
            except fingerprint.FingerprintError as exc:
                raise RbError(str(exc)) from exc
            if clock_mhz is None:
                clock_mhz = config_fp.clock_mhz
                clock_source = f"{args.config} DTS (declared)"
                if clock_mhz is None:
                    raise RbError(
                        f"{args.config} declares more than one bus clock "
                        f"({config_fp.clocks_mhz}); pass --clock-mhz to pick one"
                    )

        capture = next(
            (c for c in (xpurt.derive_capture_clock(p) for p in relevant) if c), None
        )
        if clock_mhz is None:
            linked = xpurt.link_profiles(ws, relevant)
            print(f"workspace {ws.relative_to(ROOT)} ({linked} profile(s) linked)")
            clock_mhz, clock_source = capture, "profile capture (derived)"
        else:
            reports, unconverted = xpurt.retime_profiles(ws, relevant, clock_mhz)
            converted = sum(r.converted for r in reports)
            passthrough = sum(r.passthrough for r in reports)
            print(
                f"workspace {ws.relative_to(ROOT)} "
                f"({len(reports)} profile(s) retimed, {converted} op(s) from cycles)"
            )
            if capture:
                print(
                    f"  clock   {capture:g} MHz captured -> {clock_mhz:g} MHz "
                    f"reported ({capture / clock_mhz:.4g}x), source: {clock_source}"
                )
            if passthrough or unconverted:
                names = ", ".join(sorted(p.model for p in unconverted)) or "-"
                print(
                    f"  warning: {passthrough} op(s) and {len(unconverted)} profile(s) "
                    f"carry no cycle count and stay at their capture clock ({names})",
                    file=sys.stderr,
                )

        loaded = xpurt.load_workloads(workloads)

        # Bound to a config, the machine comes off the elaboration rather than
        # off the command line: hart count, core kinds, and the profile each
        # kind reads. Unbound, the caller asserts a topology nothing checks.
        soc_plan = None
        machines = profile_hw = None
        if config_fp is not None:
            try:
                soc_plan = soc.plan(config_fp, args.config)
            except soc.SocError as exc:
                raise RbError(str(exc)) from exc
            machines, profile_hw = soc_plan.machines, dict(soc_plan.profile_hw)
            print(f"\nSoC derived from {args.config}:")
            for line in soc_plan.describe():
                print(line)
            # An explicit --hw-p/--hw-e overrides the derived slug so a profile
            # captured on some other machine can be reused deliberately. The
            # fingerprint comparison still labels the result.
            for label, override in (("cpu_p", args.hw_p), ("cpu_e", args.hw_e)):
                if override and label in profile_hw:
                    profile_hw[label] = override
            rejected = soc.check_backends(soc_plan, profile_hw)
            if rejected:
                raise RbError(
                    f"{args.config} cannot run this pairing:\n"
                    + "\n".join(f"  - {r}" for r in rejected)
                )

        if profile_hw is None:
            # Unbound run: no config to derive from, so fall back to the pairing
            # the flight profiles were captured under.
            hw_p = args.hw_p or LEGACY_HW[0]
            hw_e = args.hw_e or LEGACY_HW[1]
            requested_hw = [hw_p, hw_e]
        else:
            hw_p = hw_e = None
            requested_hw = sorted(set(profile_hw.values()))
        # A profiled schedule needs a profile for every network on every core kind
        # the scheduler may choose, or it falls back to synthetic times.
        want_topo = xpurt.topo_tag_for_cores(max(machines.values())) if machines else "topo_0"
        for hw in requested_hw:
            have = xpurt.profiles_for(args.target, hw)
            # Profiled, but not at this core count, needs a different remedy from
            # never profiled at all: another sweep, not another bitstream.
            wrong_topo = [
                (w.name, sorted(xpurt.available_topos(have[w.name])))
                for w in loaded
                if w.name in have and want_topo not in xpurt.available_topos(have[w.name])
            ]
            if wrong_topo:
                detail = "\n".join(
                    f"    {name}: has {', '.join(topos)}" for name, topos in wrong_topo
                )
                raise RbError(
                    f"{hw} is profiled, but not for {max(machines.values())} core(s) "
                    f"({want_topo}).\n{detail}\n"
                    "Profile that backend at this core count, or schedule a "
                    "topology it already covers."
                )
            missing = [w.name for w in loaded if w.name not in have]
            if missing:
                hint = (
                    f"\nThat slug names the machine {args.config} elaborates to. No "
                    "profile exists for it, which means it has never been run: it "
                    "needs a FireSim bitstream and a profiling pass.\nTo schedule "
                    "against an existing profile instead, pass --hw-p/--hw-e "
                    "explicitly; the result will be labelled extrapolated."
                    if soc_plan is not None and hw in soc_plan.slugs else
                    "\nRun the modelblaster sweep for that backend, or pick a pairing "
                    "with full coverage (rb xpurt profiles)."
                )
                raise RbError(
                    f"no profile for {', '.join(missing)} on {hw} / {args.target}."
                    + hint
                )

        spec = xpurt.write_spec(
            ws, name, loaded,
            target=args.target, hw_p=hw_p, hw_e=hw_e,
            n_p=args.cores_p, n_e=args.cores_e, time_limit=args.time_limit,
            machines=machines, profile_hw=profile_hw,
        )
        print(f"spec {spec.relative_to(ROOT)}")

        if soc_plan is not None:
            # The harness resolves CPU_P#n to a hart through this at codegen
            # time, so it has to name the harts the bitstream actually has.
            ops = sorted({op for wl in loaded for op in xpurt.workload_ops(args.target, wl.name)})
            registry_path = ws / "data" / f"cores_{args.config}.json"
            registry_path.write_text(
                json.dumps(soc.cores_registry(soc_plan, ops), indent=2) + "\n"
            )
            print(f"cores   {registry_path.relative_to(ROOT)} ({len(ops)} ops)")

        result = xpurt.run_schedule(ws, spec, solver=args.solver, profiled=True,
                                    time_limit=args.time_limit)
        if result.returncode != 0:
            tail = "\n".join((result.stderr or result.stdout or "").splitlines()[-12:])
            raise RbError(f"scheduler failed (exit {result.returncode}):\n{tail}")

        out = xpurt.schedule_outputs(ws, spec, args.solver, profiled=True)
        if not out["schedule"].is_file():
            raise RbError(f"scheduler reported success but wrote no schedule at {out['schedule']}")

        summary = xpurt.summarize(out["schedule"], clock_mhz=clock_mhz)

        # Both halves of the DSE have to name the same machine, and the join is
        # only exact when the profiles were captured on it. Say which it is.
        profile_fp = xpurt.load_profile_target(args.target)
        agreement = None
        divergences = []
        if profile_fp is not None and config_fp is not None:
            divergences = fingerprint.compare(profile_fp, config_fp)
            agreement = fingerprint.agreement(divergences)
        summary["timing"] = {
            "clock_mhz": clock_mhz,
            "clock_source": clock_source,
            "capture_clock_mhz": capture,
            "unit": "ms",
        }
        summary["profiles"] = {
            hw: {
                model: xpurt.profile_provenance(prof)
                for model, prof in sorted(xpurt.profiles_for(args.target, hw).items())
                if any(w.name == model for w in loaded)
            }
            for hw in requested_hw
        }
        summary["fingerprint"] = {
            "agreement": agreement,
            "config": config_fp.as_dict() if config_fp else None,
            "profile_target": profile_fp.as_dict() if profile_fp else None,
            "divergences": [
                {"field": d.field, "profile": d.profile, "config": d.config, "affects": d.affects}
                for d in divergences
            ],
        }

        clock_note = f" at {clock_mhz:g} MHz" if clock_mhz else ""
        makespan_cycles = summary.get("makespan_cycles")
        cycle_note = f" ({makespan_cycles:,} cycles)" if makespan_cycles else ""
        print(
            f"\nmakespan {summary['makespan']:.3f} ms{clock_note}{cycle_note} "
            f"over {summary['num_operations']} ops"
        )
        print(f"machines {summary['machines']}  profile_hw {summary['profile_hw']}")

        if config_fp is not None:
            print(f"\nconfig  {args.config}: {config_fp.summary()}")
        if profile_fp is not None:
            print(f"profile {args.target}: {profile_fp.summary()}")
        if agreement == "extrapolated":
            print(
                f"\nEXTRAPOLATED -- these profiles were not captured on {args.config}. "
                f"{len([d for d in divergences if d.affects == 'cycles'])} "
                "cycle-affecting divergence(s):",
                file=sys.stderr,
            )
            for d in divergences:
                print(f"  - {d}", file=sys.stderr)
        elif agreement == "exact":
            print("\nfingerprints match: cycles transfer exactly")

        print(f"\n  {'network':16s} {'inst':>4s} {'worst latency':>14s} {'deadline':>9s} {'verdict':>8s}")
        deadlines = {w.name: w.window_duration for w in loaded}
        missed = 0
        for net, v in summary["networks"].items():
            dl = deadlines.get(net)
            verdict = "-"
            if dl:
                ok = v["worst_latency"] <= dl
                verdict = "ok" if ok else "MISS"
                missed += 0 if ok else 1
            dl_s = f"{dl:.0f}" if dl else "-"
            print(f"  {net:16s} {v['instances']:>4} {v['worst_latency']:>14.3f} {dl_s:>9s} {verdict:>8s}")

        # Not `DSE_PLOTS = xpurt.DSE_PLOTS`: assigning the name anywhere in
        # this function made it local to the whole of it, so `rb xpurt ops`
        # and `gantt` died with UnboundLocalError before reaching this branch.
        DSE_PLOTS.mkdir(parents=True, exist_ok=True)
        if out["gantt"].is_file():
            dest = DSE_PLOTS / out["gantt"].name
            dest.write_bytes(out["gantt"].read_bytes())
            print(f"\ngantt   {dest.relative_to(ROOT)}")
        summary_path = DSE_PLOTS / f"{spec.stem}_{args.solver}_summary.json"
        summary_path.write_text(json.dumps(summary, indent=2) + "\n")
        print(f"summary {summary_path.relative_to(ROOT)}")

        if missed:
            print(f"\n{missed} network(s) missed their deadline", file=sys.stderr)
            return 1
        return 0

    raise RbError(f"unsupported xpurt action: {action}")


def _rose_guest_build(pristine: str = "auto") -> Path:
    return zephyr_build(
        get_target("rose-spike"),
        get_workload("mpc"),
        "rose",
        pristine,
    )


def cmd_rose(args: argparse.Namespace) -> int:
    """Low-level inspection/build/plot operations for the RoSE executor."""
    try:
        if args.rose_action == "doctor":
            return rose.doctor(args.ready, args.python)
        if args.rose_action == "build":
            _require_main_backend(rose.ROSE, "the RoSE backend", "rb rose build")
            if args.component in ("bridge", "all"):
                rose.build_bridge(force=args.force)
            if args.component in ("guest", "all"):
                _rose_guest_build("always" if args.force else "auto")
            return 0
        if args.rose_action == "plot":
            run_dir = args.run_dir.resolve()
            summary_path = run_dir / "summary.json"
            trace_path = run_dir / "trace.jsonl"
            if not summary_path.is_file() or not trace_path.is_file():
                raise RbError(f"run is missing summary.json or trace.jsonl: {run_dir}")
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            output = args.out or run_dir / "plot.svg"
            print(rose.render_plot(trace_path, summary, output))
            return 0
        if args.rose_action == "summarize":
            run_dir = args.run_dir.resolve()
            summary = rose.refresh_summary(run_dir)
            print(json.dumps(summary, indent=2, sort_keys=True))
            return 0
    except rose.RoseError as exc:
        raise RbError(str(exc)) from exc
    raise RbError(f"unsupported rose action: {args.rose_action}")


def cmd_ground(args: argparse.Namespace) -> int:
    """The flight-test ground station: SoftAP link, panel + monitor, and the live GO / NO-GO."""
    try:
        if args.ground_action == "ready":
            return ground.ready(no_join=args.no_join, auto_rejoin=args.auto_rejoin)
        if args.ground_action in ("up", "down", "status", "verify"):
            return ground.wifi(args.ground_action, getattr(args, "ssid", None))
        if args.ground_action == "gonogo":
            return ground.gonogo(watch=args.watch)
        if args.ground_action == "monitor":
            return ground.monitor(args.extra)
    except ground.GroundError as exc:
        raise RbError(str(exc)) from exc
    raise RbError(f"unsupported ground action: {args.ground_action}")


def _flightlog_build_text(args: argparse.Namespace) -> str:
    # The controller session hands over its build as a pasted block; a file keeps its
    # line breaks and avoids shell quoting of `->` and parentheses.
    if getattr(args, "build_file", None):
        return args.build_file.read_text(encoding="utf-8").strip()
    return getattr(args, "build", "") or ""


def cmd_flightlog(args: argparse.Namespace) -> int:
    """Publish and inspect flight trials in the flight-logs repo (RB_FLIGHT_LOGS, default ~/flight-logs)."""
    repo = args.repo or flightlog.DEFAULT_REPO
    try:
        if args.flightlog_action == "push":
            new = flightlog.push(repo=repo, raw=args.raw, date=args.date,
                                 build=_flightlog_build_text(args), tag=args.tag or "",
                                 note=args.note or "", dry_run=args.dry_run)
            if not new:
                print("no new trials (data refreshed if anything changed)")
            for trial in new:
                print(trial.name)
            return 0
        if args.flightlog_action == "show":
            print(flightlog.show(flightlog.resolve_trial(repo, args.trial), step=args.step,
                                 t_from=args.t_from, t_to=args.t_to))
            return 0
        if args.flightlog_action == "compare":
            print(flightlog.compare([flightlog.resolve_trial(repo, t) for t in args.trials]))
            return 0
    except flightlog.FlightLogError as exc:
        raise RbError(str(exc)) from exc
    raise RbError(f"unsupported flightlog action: {args.flightlog_action}")


def cmd_fly(args: argparse.Namespace) -> int:
    """The flight bench: program, load, commit, status, stop, photo (see tools/rb/fly.py)."""
    try:
        action = args.fly_action
        if action in ("load", "commit"):
            if getattr(args, "remote", None):
                return fly.remote_load_and_check(args, commit=(action == "commit"))
            return fly.load_and_check(args, commit=(action == "commit"))
        if action == "program":
            fly._confirm_pack_out(args)
            fly.program_sram(args, fly.discover(args))
            return 0
        if action == "status":
            return fly.status(args)
        if action == "stop":
            return fly.gdb_step(args, fly.FLIGHT_GDB / "stop.gdb")
        if action == "where":
            return fly.gdb_step(args, fly.FLIGHT_GDB / "where.gdb")
        if action == "photo":
            return fly.photo(args)
        if action == "imu":
            return fly.imu(args)
        if action == "motors":
            return fly.motors(args)
    except fly.FlyError as exc:
        raise RbError(str(exc)) from exc
    raise RbError(f"unsupported fly action: {action}")


def cmd_hil(args: argparse.Namespace) -> int:
    """Run a closed-loop HIL/co-simulation scenario under one supervisor."""
    if args.hil_action != "replay" or args.scenario != "mpc-hover":
        raise RbError("the implemented HIL scenario is `replay mpc-hover`")
    if args.executor != "rose-spike":
        raise RbError("mpc-hover currently requires --executor rose-spike")
    _require_main_backend(rose.ROSE, "the RoSE backend", "rb hil replay mpc-hover")
    try:
        if args.rebuild:
            rose.build_bridge(force=True)
            guest = _rose_guest_build("always")
        elif args.no_build:
            guest = args.elf or rose.DEFAULT_GUEST
        else:
            if not rose.BRIDGE_BINARY.is_file():
                rose.build_bridge()
            guest = args.elf or rose.DEFAULT_GUEST
            if not guest.is_file():
                guest = _rose_guest_build("auto")
        rose.run_mpc_hover(
            guest=guest,
            clock_mhz=args.clock_mhz,
            period_ms=args.period_ms,
            steps=args.steps,
            timeout_s=args.timeout,
            output=args.output_dir,
            python=args.python,
        )
        return 0
    except rose.RoseError as exc:
        raise RbError(str(exc)) from exc


def cmd_schedule(args: argparse.Namespace) -> int:
    """Replay a generated scheduling policy inside an application scenario."""
    if args.schedule_action != "replay" or args.scenario != "forest-trail":
        raise RbError("the implemented schedule replay is `forest-trail`")
    if args.policy != "xpurt":
        raise RbError("forest-trail currently requires --policy xpurt")
    _require_main_backend(xpurt.XPURT, "the XPU-RT backend", "rb schedule replay forest-trail")
    try:
        xpurt.run_forest_trail(
            dronet=args.dronet_weights,
            checkpoint=args.checkpoint,
            schedule=args.schedule,
            python=args.python,
            periods=args.periods,
            trail=args.trail,
            seed=args.seed,
            yolo=args.yolo,
            humans=args.humans,
            headless=args.headless,
            video=args.video,
            timeout_s=args.timeout,
            output=args.output_dir,
        )
        return 0
    except xpurt.XpurtError as exc:
        raise RbError(str(exc)) from exc


def _display(path: Path) -> str:
    """Path relative to the repo root when it is inside it, else absolute."""
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def _summaries(plots_dir: Path) -> list[tuple[Path, dict]]:
    """Every xpurt summary that recorded which config it described."""
    found = []
    for path in sorted(plots_dir.glob("*_summary.json")):
        try:
            data = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        if isinstance(data, dict) and data.get("fingerprint"):
            found.append((path, data))
    return found


def cmd_dse(args: argparse.Namespace) -> int:
    """Join the resource and performance halves into a rankable dataset."""
    action = args.dse_action

    if action == "join":
        records = {
            r.identity_id: r
            for r in area.latest_per_config(area.load_records())
            if r.identity_id
        }
        if not records:
            raise RbError(
                "no area record carries a fingerprint. Re-run rb area run so the "
                "record can be joined; records written before fingerprinting "
                "existed have nothing to match on."
            )
        pairs = _summaries(xpurt.DSE_PLOTS)
        if not pairs:
            raise RbError(
                f"no xpurt summary under {xpurt.DSE_PLOTS.relative_to(ROOT)}. "
                "Run rb xpurt run --config <config> first."
            )

        written, skipped = [], []
        for path, summary in pairs:
            config_fp = (summary.get("fingerprint") or {}).get("config") or {}
            identity = config_fp.get("identity_id")
            record = records.get(identity)
            if record is None:
                skipped.append((path.name, identity))
                continue
            mix = path.stem.replace("networks_", "").replace("_summary", "")
            deadlines = {
                name: (value or {}).get("deadline_ms")
                for name, value in (summary.get("networks") or {}).items()
            }
            # The summary knows latency but not each network's deadline, so
            # recover it from the workload descriptors the run used.
            for wl_path in xpurt.WORKLOAD_DIR.glob("*.json"):
                wl = xpurt.Workload.from_json(wl_path)
                if wl.name in deadlines and deadlines[wl.name] is None:
                    deadlines[wl.name] = wl.window_duration
            point = dse.build_point(
                board=record.board, config=record.config, attrs=record.attrs,
                mix=mix, area_record=record, summary=summary, deadlines=deadlines,
            )
            out = dse.write_point(point)
            written.append(point)
            print(f"  {point.verdict:16s} {point.config:34s} -> {out.parent.name}")

        print(f"\njoined {len(written)} point(s) into {dse.RESULTS_DIR.relative_to(ROOT)}")
        for name, identity in skipped:
            print(
                f"  skipped {name}: no area record with fingerprint {identity}. "
                "Synthesize that config (rb area run --config ...) to complete the join.",
                file=sys.stderr,
            )
        return 0

    points = dse.latest_per_config(dse.load_points()) if args.latest else dse.load_points()

    if action == "list":
        if not points:
            print("no design points recorded yet; try: rb dse join")
            return 0
        print(f"design points ({len(points)}) in {dse.RESULTS_DIR.relative_to(ROOT)}:")
        print(f"  {'verdict':16s} {'config':34s} {'attrs':10s} {'mix':22s} identity")
        for p in sorted(points, key=lambda p: (dse.VERDICTS.index(p.verdict), p.config)):
            print(f"  {p.verdict:16s} {p.config:34s} {p.attrs:10s} {p.mix:22s} {p.identity_id}")
        return 0

    if action == "report":
        if not points:
            raise RbError("no design points recorded yet; try: rb dse join")
        print(
            f"  {'config':32s} {'attrs':6s} {'LUTs':>9s} {'%':>6s} {'DSPs':>6s} "
            f"{'makespan':>10s} {'clock':>8s} {'verdict':>16s}"
        )
        def cell(value: Any, spec: str, suffix: str = "", width: int = 0) -> str:
            if value is None:
                return f"{'-':>{width}s}"
            return f"{format(value, spec)}{suffix}"

        for p in sorted(points, key=lambda p: (dse.VERDICTS.index(p.verdict), p.config)):
            util = (p.resource.get("utilization") or {}).get("luts", {}).get("util_pct")
            print(
                f"  {p.config[:32]:32s} {p.attrs:6s} "
                f"{cell(p.resource_value('luts'), '>9,.0f', width=9)} "
                f"{cell(util, '>5.1f', '%', width=6)} "
                f"{cell(p.resource_value('dsps'), '>6,.0f', width=6)} "
                f"{cell(p.performance.get('makespan_ms'), '>8.1f', 'ms', width=10)} "
                f"{cell(p.performance.get('clock_mhz'), '>5g', 'MHz', width=8)} "
                f"{p.verdict:>16s}"
            )

        frontier = dse.pareto(points)
        if frontier:
            print(f"\nPareto frontier on (LUTs, makespan), {len(frontier)} point(s):")
            for p in frontier:
                note = " [extrapolated]" if p.agreement == "extrapolated" else ""
                print(f"  {p.config}{note}")
        extrapolated = [p for p in points if p.agreement == "extrapolated"]
        if extrapolated:
            print(
                f"\n{len(extrapolated)} of {len(points)} point(s) use a profile captured "
                "on another machine. Their latency is an estimate, not a measurement.",
                file=sys.stderr,
            )
        return 0

    if action == "plot":
        if not points:
            raise RbError("no design points recorded yet; try: rb dse join")
        # A relative --out is not under ROOT until resolved, and relative_to
        # raises rather than falling back.
        out = args.out if args.out.is_absolute() else (Path.cwd() / args.out).resolve()
        written = plotting.write_dse_figures(points, out)
        for path in written:
            print(f"  {_display(path)}")
        print(f"\n{len(written)} figure(s) in {_display(out)}")
        return 0

    raise RbError(f"unsupported dse action: {action}")


def cmd_area(args: argparse.Namespace) -> int:
    action = args.area_action

    if action == "list":
        records = area.load_records()
        print(f"recorded runs ({len(records)}) in {area.RESULTS_DIR.relative_to(ROOT)}:")
        for record in records:
            flag = " [upper bound]" if record.totals_are_upper_bound else ""
            print(
                f"  {record.run_id}  {record.board:9s} {record.config:38s} "
                f"luts={record.luts}{flag}"
            )
        studies = sorted(STUDIES_DIR.glob("*.json"))
        print(f"\nstudies ({len(studies)}) in {STUDIES_DIR.relative_to(ROOT)}:")
        for path in studies:
            study = json.loads(path.read_text())
            state = "BLOCKED" if study.get("blocked_reason") else "runnable"
            count = len(study.get("configs") or [])
            print(f"  {path.stem:22s} {state:9s} {count} config(s)  {study.get('description','')}")
        return 0

    if action == "report":
        records = area.load_records()
        if args.run_id:
            records = [r for r in records if r.run_id == args.run_id]
            if not records:
                raise RbError(f"no recorded run with id {args.run_id!r}")
        elif not args.all:
            records = area.latest_per_config(records)
        if not records:
            print("no area runs recorded yet; try: rb area run --board ku040")
            return 0
        for key, bucket in sorted(area.group_comparable(records).items()):
            flow, attrs, part, board = key
            print(f"\n{board} / {part} / flow={flow} / attrs={attrs}")
            if bucket[0].totals_are_upper_bound:
                print("  NOTE: keep_hierarchy injected; totals are an upper bound")
            print(f"  {'config':40s} {'LUTs':>10s} {'Regs':>10s} {'BRAM':>7s} {'DSP':>6s}")
            for record in bucket:
                def used(key: str) -> str:
                    entry = record.totals.get(key) or {}
                    value = entry.get("used")
                    return "-" if value is None else f"{value:,}"

                print(
                    f"  {record.config:40s} {used('luts'):>10s} {used('registers'):>10s} "
                    f"{used('brams'):>7s} {used('dsps'):>6s}"
                )
        return 0

    if action == "plot":
        records = area.load_records()
        if not args.all:
            records = area.latest_per_config(records)
        if not records:
            raise RbError("no area runs recorded yet; nothing to plot")
        area.write_summary_csv(records, args.out / "summary.csv")
        written = plotting.write_figures(records, args.out)
        print(f"wrote {len(written)} figure(s) and summary.csv to {args.out}")
        for path in written:
            print(f"  {path.name}")
        return 0

    if action == "restore":
        board = _configured_board(args)
        run(area.restore_command(board), environment="chipyard", cwd=ROOT)
        return 0

    if action == "run":
        board = _configured_board(args)
        area_run_one(board, args.attrs, args.force)
        return 0

    if action == "sweep":
        if bool(args.study) == bool(args.configs):
            raise RbError("pass exactly one of --study or --configs")
        base = get_board(args.board)
        if args.configs:
            configs = [c.strip() for c in args.configs.split(",") if c.strip()]
            attrs = args.attrs
        else:
            study = _load_study(args.study)
            study_board = study.get("board")
            if study_board and study_board != base.name:
                raise RbError(
                    f"study {args.study!r} targets board {study_board!r}, "
                    f"but --board {base.name!r} was given"
                )
            configs = list(study["configs"])
            attrs = args.attrs if args.attrs != "none" else study.get("attrs", "none")

        records: list[AreaRecord] = []
        failures: list[tuple[str, str]] = []
        for index, config in enumerate(configs, start=1):
            print(f"\n=== [{index}/{len(configs)}] {config} (attrs={attrs}) ===")
            try:
                records.append(area_run_one(base.with_config(config), attrs, args.force))
            except (RbError, area.AreaError, subprocess.CalledProcessError) as exc:
                if not args.keep_going:
                    raise
                failures.append((config, str(exc)))
                print(f"rb: area sweep: {config} failed: {exc}", file=sys.stderr)

        if records:
            summary = area.RESULTS_DIR / "sweeps"
            summary.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            name = args.study or "adhoc"
            path = area.write_summary_csv(records, summary / f"{stamp}-{name}.csv")
            print(f"\nsweep summary: {path.relative_to(ROOT)}")
        if failures:
            print(f"\n{len(failures)} config(s) failed:", file=sys.stderr)
            for config, message in failures:
                print(f"  {config}: {message}", file=sys.stderr)
            return 1
        return 0

    if action == "timing":
        board = get_board(args.board).with_config(getattr(args, "config", None))
        records = {r.config: r for r in area.latest_per_config(area.load_records())}
        record = records.get(board.config)
        synthesized = (record.design or {}).get("synthesized_at") if record else None
        try:
            report = timing.for_board(board, synthesized)
        except timing.TimingError as exc:
            raise RbError(str(exc)) from exc

        print(f"{board.config}: {report.summary()}")
        print(f"  state    {report.design_state}, reported {report.reported_at}")
        print(f"\n  {'clock':32s} {'period':>9s} {'WNS':>9s} {'critical':>9s} {'Fmax':>9s} {'endpoints':>10s}")
        for clock in report.clocks:
            fmax = f"{clock.achieved_mhz:.1f}" if clock.achieved_mhz else "-"
            print(
                f"  {clock.name[:32]:32s} {clock.period_ns:>8.3f}n {clock.wns_ns:>+9.3f} "
                f"{clock.critical_path_ns:>8.3f}n {fmax:>8s}M {clock.endpoints:>10,}"
            )
        if report.stale_against_synthesis:
            print(
                "\nThese reports predate the synthesis recorded for this config, so "
                "they describe an earlier implementation. Treat the frequency as "
                "belonging to that run, not to the current area record.",
                file=sys.stderr,
            )
        if not report.met:
            print("\ntiming NOT met", file=sys.stderr)
            return 1
        return 0

    raise RbError(f"unsupported area action: {action}")


# Chipyard divides the fabric clock by this to make mtime, and the Zephyr SoC
# restates it as CONFIG_RTC_CLOCK_DIVIDER_VALUE
# (soc/rocketchip/virt_riscv/Kconfig.soc, default 1000). It is what separates
# the two clocks the ModelBlaster harness reports in: rdcycle counts at the
# fabric clock, k_cycle_get_64 at the fabric clock over this.
MTIME_DIVIDER = 1000


def _modelblaster_clocks(board: Board) -> tuple[float, float, str]:
    """(cpu_hz, tick_hz, evidence) for a board+config, from its elaborated DTS.

    Taken from the devicetree rather than the config name because the name is
    not evidence: RocketArty200TDroneGemminiSaturnFp16At35Config says 35 in it,
    but what the shell was actually elaborated at is a property of the build.
    """
    timebase = dts_timebase_hz(board)
    if timebase is None:
        raise RbError(
            f"no timebase-frequency in the devicetree for {board.name}/"
            f"{board.config}. Elaborate it (rb area run --board "
            f"{board.name} --config {board.config}) or publish its DTS "
            "(tools/collect-dts.py) before reporting timings against it: "
            "without the clock, cycles cannot become milliseconds and the "
            "FPS figure would be invented."
        )
    return (
        float(timebase * MTIME_DIVIDER),
        float(timebase),
        f"{board.name}/{board.config} devicetree (timebase-frequency = {timebase} Hz)",
    )


def _check_float_contract(board: Board, cflags: Sequence[str]) -> None:
    """Refuse a hard-float kernel build on a shell whose FPU cannot run it.

    The failure this prevents is silent at build time and fatal at run time.
    modelblaster's gemmini_q31 backend compiles kernels.c with
    -march=rv64imafdc -mabi=lp64d, i.e. hardware fp32 AND fp64. On
    RocketArty200TDroneGemminiSaturnFp16At35Config the scalar FPU is FP16-only,
    and nothing downstream notices: rocket-chip still writes "f" into
    riscv,isa, so the devicetree, Zephyr's -march, and these flags all agree
    with each other and disagree with the silicon. The image links, boots, and
    faults on the first fadd.s inside a kernel.
    """
    if board.config not in FP16_SCALAR_FPU_CONFIGS:
        return
    demanded = [flag for flag in cflags if flag.startswith("-march=") or flag.startswith("-mabi=")]
    wants_fp = any(
        ("d" in flag.split("=", 1)[1] or "f" in flag.split("=", 1)[1])
        for flag in demanded
    )
    if not wants_fp:
        return
    raise RbError(
        f"{board.config} has an FP16-only scalar FPU (WithRocketFPU16: "
        f"fLen = 16), and these kernel flags ask for hardware fp32/fp64: "
        f"{' '.join(demanded)}.\n"
        "\n"
        "Nothing downstream would catch this. rocket-chip writes \"f\" into "
        "riscv,isa for any FPU, so the DTS advertises rv64imafc while the "
        "hardware implements only Zfh; the build succeeds and the first "
        "fp32 instruction inside a kernel faults.\n"
        "\n"
        "Options, in the order worth trying:\n"
        "  --config RocketArty200TDroneGemmini16At40Config   Gemmini 16x16, no "
        "Saturn, FULL scalar FPU, closes at 40 MHz with wide margin. The "
        "int8 DroNet path uses Gemmini, not vector FP, so this is the "
        "deployable target for this model.\n"
        "  --config RocketArty200TDroneSaturn40Config        Saturn V128D128, "
        "full scalar FPU, 40 MHz.\n"
        "  a soft-float kernel target (TARGET=scalar), which keeps this shell "
        "but gives up Gemmini.\n"
        "\n"
        f"{board.config} remains the right target for AREA and TIMING work; "
        "it is running a stock fp32 image on it that does not work."
    )


def cmd_modelblaster(args: argparse.Namespace) -> int:
    action = args.modelblaster_action

    if action == "report":
        capture = Path(args.capture)
        if not capture.is_file():
            raise RbError(f"no capture at {capture}")
        if args.cpu_mhz is not None:
            cpu_hz = args.cpu_mhz * 1e6
            # The mtime rate is a property of the machine, not of the CPU clock,
            # and it is NOT the same divider everywhere: the Chipyard shells use
            # 1000, and the measured ratio on a spike capture of this harness is
            # 100. Defaulting to the FPGA value and letting it be overridden
            # keeps a spike or FireSim capture reportable without silently
            # scaling its frame time by 10x.
            tick_hz = args.tick_hz if args.tick_hz else cpu_hz / MTIME_DIVIDER
            evidence = (
                f"--cpu-mhz {args.cpu_mhz}"
                + (f" --tick-hz {args.tick_hz:g}" if args.tick_hz else "")
                + " (asserted, not read from a devicetree)"
            )
        else:
            board = BOARDS[args.board].with_config(args.config)
            cpu_hz, tick_hz, evidence = _modelblaster_clocks(board)
            if args.tick_hz:
                tick_hz = args.tick_hz
                evidence += f", tick rate overridden to {args.tick_hz:g} Hz"
        gen_dir = Path(args.gen_dir).resolve() if args.gen_dir else (
            modelblaster.generated_dir(args.model, args.quant, args.target)
        )
        run = modelblaster.parse_capture(
            capture.read_text(errors="replace"),
            model=args.model,
            cpu_hz=cpu_hz,
            tick_hz=tick_hz,
            clock_evidence=evidence,
            picks=modelblaster.load_kernel_picks(gen_dir),
        )
        out_dir = Path(args.out_dir) if args.out_dir else capture.parent
        csv_path = modelblaster.write_profile_csv(run, out_dir / "profile.csv")
        svg_path = out_dir / "gantt.svg"
        modelblaster.gantt_svg(run, svg_path)
        print("\n".join(modelblaster.summary_lines(run)))
        print()
        print(f"per-op table  {csv_path}")
        print(f"timeline      {svg_path}")
        return 0

    if action == "generate":
        out = modelblaster.generate(
            args.model, quant=args.quant, target=args.target,
            drift_atol=args.drift_atol,
        )
        print(out)
        return 0

    if action == "build":
        board = BOARDS[args.board].with_config(args.config)
        target = get_target(args.board)
        workload = get_workload("dronet")
        # resolve(): CMake is given this verbatim and resolves a relative path
        # against the harness source directory, not the caller's cwd, so
        # `--gen-dir artifacts/...` became
        # backends/modelblaster/harness/artifacts/... and failed as "not a
        # directory". Every documented invocation of this flag was relative.
        gen_dir = Path(args.gen_dir).resolve() if args.gen_dir else (
            modelblaster.generated_dir(args.model, args.quant, args.target)
        )
        if not gen_dir.is_dir():
            raise RbError(
                f"no generated sources at {gen_dir}; run "
                f"`rb modelblaster generate --model {args.model} "
                f"--quant {args.quant} --target {args.target}` first"
            )
        # Select the Gemmini header elaborated for THIS config. Without it the
        # build takes whatever per_config/default16x16 holds, which describes a
        # different mvin-scale type than these shells elaborate -- and a
        # gemmini_params.h that disagrees with the hardware is not a build
        # error, it is a binary that drives the accelerator wrongly.
        # A --gen-dir knows which backend it was generated for; believe it over
        # the default, so `--gen-dir <softfp dir>` is not silently built with
        # gemmini_q31's hard-float flags.
        # kernel_target, not `target` -- that name is already the ZephyrTarget
        # in this scope, and shadowing it turns the next workload.supports()
        # into an AttributeError on a str.
        kernel_target = args.target
        if args.gen_dir:
            recorded = modelblaster.target_of(gen_dir)
            if recorded and recorded != kernel_target:
                print(
                    f"rb: {gen_dir.name} was generated for target "
                    f"{recorded!r}; using that instead of {kernel_target!r}"
                )
                kernel_target = recorded
        cflags = modelblaster.kernel_cflags(
            kernel_target, gemmini_config=board.config)
        _check_float_contract(board, cflags)
        elf = zephyr_build(
            target,
            workload,
            workload.default_mode,
            args.pristine,
            board,
            ram_console=args.ram_console,
            extra_defines=modelblaster.harness_defines(
                args.model,
                quant=args.quant,
                target=kernel_target,
                gemmini_config=board.config,
                gen_dir=gen_dir,
            ),
        )
        # So a later rb debug / run / build rebuilds THIS selection rather than
        # the defaults.
        modelblaster.write_build_marker(
            elf.parent.parent,
            model=args.model, quant=args.quant, target=kernel_target,
            gen_dir=gen_dir,
        )
        print(elf)
        return 0

    raise RbError(f"unknown modelblaster action {action!r}")


def cmd_flash(args: argparse.Namespace) -> int:
    target = get_target(args.target)
    workload = get_workload(args.workload)
    mode = args.mode or workload.default_mode
    if args.build:
        zephyr_build(target, workload, mode, args.pristine)
    flash_esp(target, workload, mode, args.port)
    return 0


def cmd_run(args: argparse.Namespace) -> int:
    target = get_target(args.target)
    workload = get_workload(args.workload)
    mode = args.mode or workload.default_mode

    if target.kind == "esp":
        if args.config:
            raise RbError("--config selects Chipyard FPGA hardware and is invalid for ESP")
        zephyr_build(target, workload, mode, args.pristine)
        if not args.no_flash:
            flash_esp(target, workload, mode, args.port)
        if not args.no_console:
            if args.capture_dir is not None or args.capture_duration:
                capture_serial(
                    target,
                    workload,
                    mode,
                    args.port,
                    args.console_baudrate,
                    args.capture_duration,
                    args.capture_dir,
                )
                return 0
            return cmd_console(
                argparse.Namespace(
                    port=args.port,
                    baudrate=args.console_baudrate,
                )
            )
        return 0

    board = target.fpga_board
    if board is None:
        raise RbError(f"FPGA target {target.name} has no physical board definition")
    board = _workload_board(board, workload, args.config)
    if workload.name in DRONE_FPGA_WORKLOADS:
        if "DroneLogic" not in board.config:
            raise RbError(
                f"{workload.name} requires a DroneLogic FPGA configuration"
            )
        raise RbError(
            f"{workload.name} software builds against {board.config}, but "
            f"{target.name} deployment remains blocked: SPI/GPIO/PWM signals "
            "have no verified package-pin and I/O-voltage mapping yet"
        )
    if args.rebuild_fpga or not board.bitstream.is_file():
        chipyard_build(board, "bitstream")
    zephyr_build(target, workload, mode, args.pristine, board)
    if not args.no_program:
        program_fpga(
            board, args.tool, getattr(args, "program_usb_location", "") or ""
        )
    if not args.no_load:
        load_elf(board, workload, mode, args.tsi_port, args.tsi_baudrate)
    if not args.no_console:
        if args.capture_dir is not None or args.capture_duration:
            capture_serial(
                target,
                workload,
                mode,
                args.console_port,
                args.console_baudrate,
                args.capture_duration,
                args.capture_dir,
            )
            return 0
        return cmd_console(
            argparse.Namespace(
                port=args.console_port,
                baudrate=args.console_baudrate,
            )
        )
    return 0


def cmd_targets(args: argparse.Namespace) -> int:
    print("TARGET       KIND  ZEPHYR BOARD                                  EVIDENCE")
    for target in TARGETS.values():
        print(
            f"{target.name:12} {target.kind:5} "
            f"{target.zephyr_board:45} {target.evidence}"
        )
        board = target.fpga_board
        if board is not None:
            active_mib = board.memory_size / (1 << 20)
            active = (
                f"{active_mib:g} MiB"
                if active_mib >= 1
                else f"{board.memory_size // 1024} KiB"
            )
            if board.physical_memory_size is None:
                physical = "not recorded"
            else:
                physical = (
                    f"{board.physical_memory_size / (1 << 30):g} GiB "
                    f"{board.physical_memory_type}"
                )
            print(
                f"  active SoC RAM: {active}; physical memory: {physical}; "
                f"{board.physical_memory_evidence}"
            )
    return 0


def cmd_workloads(args: argparse.Namespace) -> int:
    print("WORKLOAD          MODES          TARGETS                  STATUS/SAFETY")
    for workload in WORKLOADS.values():
        status = "blocked" if workload.blocked_reason else workload.safety
        if workload.legacy:
            status = f"LEGACY, {status}"
        targets = ",".join(workload.targets) or "none"
        print(
            f"{workload.name:17} {','.join(workload.modes):14} "
            f"{targets:24} {status}"
        )
        print(f"  {workload.summary}")
        if workload.legacy:
            print(f"  legacy: {workload.legacy}")
        for mode, note in workload.mode_legacy:
            print(f"  legacy --mode {mode}: {note}")
        if workload.blocked_reason is not None:
            print(f"  blocked: {workload.blocked_reason}")
        for mode, names in workload.mode_targets:
            print(f"  --mode {mode} targets: {','.join(names)}")
        for board_name, config in workload.default_configs:
            print(f"  {board_name} default config: {config}")
        for mode, minimum in workload.minimum_ram_by_mode:
            print(
                f"  {mode} requirement: at least {minimum // 1024} KiB "
                "addressable RAM on FPGA targets"
            )
    return 0


def _add_board(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--board", choices=BOARDS, default="arty100t")


def _add_target(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--target",
        "--board",
        dest="target",
        choices=TARGETS,
        default="arty100t",
        help="Zephyr deployment target (legacy spelling: --board)",
    )


def _add_fpga_config(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--config",
        metavar="CLASS",
        help=(
            "Chipyard FPGA config class (default: the drone shell for drone "
            "workloads, else the board's RiskyBird OSPI config; see: rb workloads)"
        ),
    )


def _add_attrs(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--attrs",
        choices=area.ATTR_MODES,
        default="none",
        help=(
            "RTL attribute injection before synthesis: none measures pristine "
            "generated RTL; hierarchy adds keep_hierarchy for faithful "
            "per-module attribution but makes totals an upper bound; dsp adds "
            "use_dsp to MAC/OPU multipliers; all does both"
        ),
    )


def _add_workload(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("workload", choices=WORKLOADS)


def _add_build_options(parser: argparse.ArgumentParser) -> None:
    _add_target(parser)
    parser.add_argument(
        "--mode",
        help="workload execution mode (see: rb workloads)",
    )
    parser.add_argument(
        "--pristine",
        choices=("auto", "always", "never"),
        default="auto",
    )


def _add_program_tool(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--tool",
        choices=("auto", "vivado", "openfpgaloader"),
        default="auto",
    )


def _add_program_usb_location(
    parser: argparse.ArgumentParser, option: str = "--usb-location"
) -> None:
    """Pin the adapter openFPGALoader programs through.

    A separate option from `rb debug --usb-location` on purpose: that one pins
    the RISC-V debug TAP and this one pins the FPGA configuration TAP, and on
    the drone bench they are two different physical adapters. Passing one where
    the other belongs writes a bitstream at a chain that has no config TAP.
    """

    parser.add_argument(
        option,
        dest="program_usb_location" if option != "--usb-location" else "usb_location",
        help="USB topology path of the *bitstream* FTDI, e.g. 7-1.1.4.3 "
        "(or RB_PROGRAM_FTDI_LOCATION). Required when more than one FTDI is "
        "attached; see: rb probe-ftdi --idcode",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rb",
        description="One entry point for RiskyBird FPGA and Zephyr flows.",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    sub = parser.add_subparsers(dest="command", required=True)

    doctor = sub.add_parser("doctor", help="check tools, environments, and hardware")
    doctor.add_argument("--hardware", action="store_true", help="require an attached serial device")
    doctor.set_defaults(func=cmd_doctor)

    targets = sub.add_parser("targets", help="list Zephyr build/deploy targets")
    targets.set_defaults(func=cmd_targets)

    workloads = sub.add_parser("workloads", help="list workloads and target support")
    workloads.set_defaults(func=cmd_workloads)

    env = sub.add_parser("env", help="inspect or enter backend environments")
    env_sub = env.add_subparsers(dest="env_action", required=True)
    env_status = env_sub.add_parser("status")
    env_status.set_defaults(func=cmd_env)
    for action in ("setup", "shell"):
        item = env_sub.add_parser(action)
        item.add_argument("environment", choices=("chipyard", "zephyr", "rose"))
        item.set_defaults(func=cmd_env)

    chipyard = sub.add_parser("chipyard", help="low-level Chipyard operations")
    cy_sub = chipyard.add_subparsers(dest="chipyard_action", required=True)
    for action in ("elaborate", "build", "path"):
        item = cy_sub.add_parser(action)
        _add_board(item)
        _add_fpga_config(item)
        item.set_defaults(func=cmd_chipyard)
    cy_program = cy_sub.add_parser("program")
    _add_board(cy_program)
    _add_fpga_config(cy_program)
    _add_program_tool(cy_program)
    _add_program_usb_location(cy_program)
    cy_program.set_defaults(func=cmd_chipyard)

    zephyr = sub.add_parser("zephyr", help="low-level Zephyr operations")
    z_sub = zephyr.add_subparsers(dest="zephyr_action", required=True)
    z_setup = z_sub.add_parser(
        "setup-targets",
        help="install the pinned ESP and ST HAL projects",
    )
    z_setup.set_defaults(func=cmd_zephyr_setup_targets)
    z_build = z_sub.add_parser("build")
    _add_workload(z_build)
    _add_build_options(z_build)
    _add_fpga_config(z_build)
    z_build.set_defaults(func=cmd_zephyr)
    z_path = z_sub.add_parser("path")
    _add_workload(z_path)
    _add_target(z_path)
    z_path.add_argument("--mode")
    z_path.set_defaults(func=cmd_zephyr)

    build = sub.add_parser("build", help="build a top-level Zephyr workload")
    _add_workload(build)
    _add_build_options(build)
    _add_fpga_config(build)
    build.add_argument("--ram-console", action="store_true", help="console into a RAM buffer (ram_console_buf) instead of the UART, readable over JTAG; for boards whose UART is unproven")
    build.set_defaults(func=cmd_build)

    flash = sub.add_parser("flash", help="flash a built ESP workload")
    _add_workload(flash)
    _add_target(flash)
    flash.add_argument("--mode")
    flash.add_argument("--port", help="ESP serial device (or RB_ESP_PORT)")
    flash.add_argument(
        "--build",
        action="store_true",
        help="build before flashing",
    )
    flash.add_argument(
        "--pristine",
        choices=("auto", "always", "never"),
        default="auto",
    )
    flash.set_defaults(func=cmd_flash)

    probe_ftdi = sub.add_parser(
        "probe-ftdi",
        help="list attached FTDI adapters and their OpenOCD USB locations",
    )
    probe_ftdi.add_argument(
        "--idcode",
        action="store_true",
        help="shift a JTAG IDCODE out of each adapter to tell them apart "
        "(detaches ftdi_sio, so ttyUSB nodes drop until replug)",
    )
    probe_ftdi.add_argument(
        "--location",
        action="append",
        metavar="LOC",
        help="restrict --idcode to these USB locations (repeatable). Use this on "
        "a shared host: without it, --idcode shifts a scan through EVERY MPSSE "
        "adapter present, including cards that belong to someone else.",
    )

    bench = sub.add_parser(
        "bench",
        help="print the bench's FTDI numbers with roles resolved: which "
        "location, bus:device and ttyUSB go in which flag",
    )
    bench.add_argument(
        "--no-probe",
        action="store_true",
        help="skip the IDCODE probe (faster, keeps ttyUSB nodes bound, but "
        "cannot tell the bitstream adapter from the debug one)",
    )
    bench.add_argument(
        "--all",
        action="store_true",
        help="also probe adapters whose EEPROM names a vendor board (Digilent, "
        "Xilinx). Off by default: a probe detaches that board's JTAG from "
        "whoever is using it",
    )
    bench.set_defaults(func=cmd_bench)
    probe_ftdi.set_defaults(func=cmd_probe_ftdi)

    program = sub.add_parser("program", help="program the FPGA bitstream")
    _add_board(program)
    _add_fpga_config(program)
    _add_program_tool(program)
    _add_program_usb_location(program)
    program.set_defaults(func=cmd_program)

    load = sub.add_parser("load", help="load a built Zephyr ELF through UARTTSI")
    _add_workload(load)
    _add_board(load)
    load.add_argument("--mode")
    load.add_argument(
        "--port",
        help="UARTTSI device (or RB_TSI_PORT); socket://HOST:14552 loads through the "
        "ESP over WiFi, esp-usb:///dev/ttyACM0 through its USB console (shells "
        "built with the ESP loader only)",
    )
    load.add_argument("--baudrate", type=int, default=921600)
    load.add_argument("--ram-console", action="store_true", help="load the --ram-console build (console in ram_console_buf, readable over JTAG or an ESP RBPOKE1 session) -- for a drone with no programmer board attached")
    load.set_defaults(func=cmd_load)

    console = sub.add_parser("console", help="open the FPGA UART console")
    console.add_argument("--port", help="console device (or RB_CONSOLE_PORT)")
    console.add_argument("--baudrate", type=int, default=115200)
    console.set_defaults(func=cmd_console)

    capture = sub.add_parser(
        "capture",
        help="capture an interactive serial trace plus run metadata",
    )
    _add_workload(capture)
    _add_target(capture)
    capture.add_argument("--mode")
    capture.add_argument("--port", help="serial device for the selected target")
    capture.add_argument("--baudrate", type=int)
    capture.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="capture seconds; zero runs until Ctrl-]",
    )
    capture.add_argument(
        "--output-dir",
        help="new results directory (default: timestamped results/runs path)",
    )
    capture.set_defaults(func=cmd_capture)

    debug = sub.add_parser("debug", help="build and launch OpenOCD plus GDB")
    _add_workload(debug)
    _add_build_options(debug)
    _add_fpga_config(debug)
    _add_program_tool(debug)
    _add_program_usb_location(debug, "--program-usb-location")
    debug.add_argument("--program", action="store_true", help="program the FPGA first")
    debug.add_argument("--adapter-config", help="override the default FT-LINK OpenOCD config")
    debug.add_argument(
        "--usb-location",
        help="USB topology path of the JTAG adapter, e.g. 7-1.1.4.1 "
        "(or RB_FTDI_LOCATION). Required when more than one FTDI is "
        "attached; see: rb probe-ftdi",
    )
    debug.add_argument("--gdb-port", type=int, default=3333)
    debug.add_argument("--no-load", action="store_true", help="attach without loading the ELF")
    debug.add_argument(
        "--no-run",
        action="store_true",
        help="stop at the GDB prompt instead of starting the workload. The default loads "
        "and runs; use this to set breakpoints before the program executes",
    )
    debug.add_argument(
        "--gdb-script",
        help="run a GDB command file and exit, instead of an interactive session "
        "(passed as -x --batch, not on stdin: OpenOCD reads stdin too)",
    )
    debug.add_argument("--ram-console", action="store_true", help="console into a RAM buffer (ram_console_buf) instead of the UART, readable over JTAG; for boards whose UART is unproven")
    debug.set_defaults(func=cmd_debug)

    full = sub.add_parser("run", help="build, program, load, and open the console")
    _add_workload(full)
    _add_build_options(full)
    _add_fpga_config(full)
    _add_program_tool(full)
    _add_program_usb_location(full, "--program-usb-location")
    full.add_argument("--rebuild-fpga", action="store_true")
    full.add_argument("--no-program", action="store_true")
    full.add_argument("--no-load", action="store_true")
    full.add_argument("--no-flash", action="store_true")
    full.add_argument("--no-console", action="store_true")
    full.add_argument("--port", help="ESP flash/console device (or RB_ESP_PORT)")
    full.add_argument("--tsi-port", help="UARTTSI device (or RB_TSI_PORT)")
    full.add_argument("--console-port", help="console device (or RB_CONSOLE_PORT)")
    full.add_argument("--tsi-baudrate", type=int, default=921600)
    full.add_argument("--console-baudrate", type=int, default=115200)
    full.add_argument(
        "--capture-dir",
        help="capture the console and metadata into this new directory",
    )
    full.add_argument(
        "--capture-duration",
        type=float,
        default=0.0,
        help="capture seconds; zero runs until Ctrl-]",
    )
    full.set_defaults(func=cmd_run)

    rose_cli = sub.add_parser(
        "rose",
        help="[experimental] inspect and build the FPGA-free RoSE Spike executor",
    )
    rose_sub = rose_cli.add_subparsers(dest="rose_action", required=True)
    rose_doctor = rose_sub.add_parser("doctor", help="check RoSE host and simulator prerequisites")
    rose_doctor.add_argument("--ready", action="store_true", help="also require built bridge and guest artifacts")
    rose_doctor.add_argument("--python", type=Path, help="host Python interpreter to probe")
    rose_doctor.set_defaults(func=cmd_rose)
    rose_build = rose_sub.add_parser("build", help="build the Spike bridge and/or MPC guest")
    rose_build.add_argument("component", choices=("bridge", "guest", "all"), nargs="?", default="all")
    rose_build.add_argument("--force", action="store_true", help="rebuild even when inputs are older")
    rose_build.set_defaults(func=cmd_rose)
    rose_plot = rose_sub.add_parser("plot", help="render an SVG from a recorded RoSE run")
    rose_plot.add_argument("run_dir", type=Path)
    rose_plot.add_argument("--out", type=Path)
    rose_plot.set_defaults(func=cmd_rose)
    rose_summary = rose_sub.add_parser(
        "summarize",
        help="re-derive summary.json and plot.svg from a recorded trace",
    )
    rose_summary.add_argument("run_dir", type=Path)
    rose_summary.set_defaults(func=cmd_rose)

    hil = sub.add_parser("hil", help="[experimental] supervise closed-loop hardware-in-the-loop replays")
    hil_sub = hil.add_subparsers(dest="hil_action", required=True)
    hil_replay = hil_sub.add_parser("replay", help="run a recorded or simulated closed-loop scenario")
    hil_replay.add_argument("scenario", choices=("mpc-hover",))
    hil_replay.add_argument("--executor", choices=("rose-spike",), default="rose-spike")
    hil_replay.add_argument("--mode", choices=("lockstep",), default="lockstep")
    hil_replay.add_argument("--clock-mhz", type=float, default=1000.0)
    hil_replay.add_argument("--period-ms", type=float, default=20.0)
    hil_replay.add_argument("--steps", type=int, default=300)
    hil_replay.add_argument("--timeout", type=float, default=600.0, help="wall-clock timeout in seconds")
    hil_replay.add_argument("--output-dir", type=Path)
    hil_replay.add_argument("--python", type=Path, help="host Python interpreter override")
    hil_replay.add_argument("--elf", type=Path, help="prebuilt MPC guest ELF override")
    build_group = hil_replay.add_mutually_exclusive_group()
    build_group.add_argument("--no-build", action="store_true", help="require existing bridge and guest artifacts")
    build_group.add_argument("--rebuild", action="store_true", help="force both bridge and guest rebuilds")
    hil_replay.set_defaults(func=cmd_hil)

    gnd = sub.add_parser(
        "ground",
        help="flight-test ground station: drone SoftAP, panel + monitor, live GO / NO-GO",
    )
    gnd_sub = gnd.add_subparsers(dest="ground_action", required=True)
    gnd_ready = gnd_sub.add_parser(
        "ready", help="caffeinate, join the SoftAP once, start the panel and the monitor"
    )
    gnd_ready.add_argument("--no-join", action="store_true", help="start everything, do not touch Wi-Fi")
    gnd_ready.add_argument(
        "--auto-rejoin", action="store_true",
        help="also recover stale leases and rejoin by itself (switches the Mac's Wi-Fi on each retry)",
    )
    gnd_ready.set_defaults(func=cmd_ground)
    gnd_up = gnd_sub.add_parser("up", help="join the drone SoftAP (fresh lease; needs non-Wi-Fi internet)")
    gnd_up.add_argument("--ssid", help="drone SSID (default: the script's riskybird-5674)")
    gnd_up.set_defaults(func=cmd_ground)
    for action, text in (("down", "leave the SoftAP and restore the previous network"),
                         ("status", "show the Wi-Fi and internet-path state"),
                         ("verify", "PING the bridge and listen 10 s on :14550 (stop the monitor first)")):
        gnd_sub.add_parser(action, help=text).set_defaults(func=cmd_ground)
    gnd_go = gnd_sub.add_parser("gonogo", help="print the running monitor's verdict (exit 0 only on GO)")
    gnd_go.add_argument("--watch", action="store_true", help="print once a second until Ctrl-C")
    gnd_go.set_defaults(func=cmd_ground)
    gnd_mon = gnd_sub.add_parser("monitor", help="run only the monitor (no panel, no Wi-Fi changes)")
    gnd_mon.add_argument("extra", nargs=argparse.REMAINDER,
                         help="passed to riskybird-flightready.py (e.g. --relay 0)")
    gnd_mon.set_defaults(func=cmd_ground)

    fly_p = sub.add_parser(
        "fly",
        help="flight bench: program the shell, load + verify a flight preset, commit, stop",
        description="Build, program, load, verify and commit a flight image, or stop one. "
        "Presets are flight_controller modes from tools/flight/presets/ (rb workloads).",
    )
    fly_sub = fly_p.add_subparsers(dest="fly_action", required=True)

    def _fly_common(p, *, mode_required=False, program=False):
        p.add_argument("--target", default=fly.DEFAULT_TARGET)
        p.add_argument("--config", default=fly.DEFAULT_CONFIG)
        p.add_argument("--debug-location", help="USB location of the debug adapter (default: by IDCODE)")
        p.add_argument("--bitstream-location", help="USB location of the bitstream adapter (default: by IDCODE)")
        p.add_argument("--console", help="FPGA console serial port (default: the debug adapter's channel B)")
        p.add_argument("--esp-port", help="ESP32-C6 USB serial port (default: auto)")
        if mode_required is not None:
            p.add_argument("--mode", required=mode_required,
                           help="flight_controller mode / flight preset, e.g. t21-hover-1.0m")
        if program:
            p.add_argument("--pack-out", action="store_true",
                           help="confirm the LiPo is disconnected (re-programming the shell with it in is unsafe)")
            p.add_argument("--no-program", action="store_true",
                           help="skip re-programming the FullDDR shell (it is already fresh)")
            p.add_argument("--pristine", choices=("auto", "always", "never"), default="auto")
            p.add_argument("--remote", metavar="URL",
                           help="no cable to the drone: load (and commit) through the ESP, e.g. "
                                "socket://192.168.4.1:14552 or esp-usb:///dev/ttyACM0. Needs a loader "
                                "shell (RocketArty200TDroneFullDDRLoaderConfig) already running")
        p.set_defaults(func=cmd_fly)
        return p

    _fly_common(fly_sub.add_parser("commit", help="program SRAM, load, verify banner + ESP link, write the commit key"),
                mode_required=True, program=True)
    _fly_common(fly_sub.add_parser("load", help="as commit, without the commit key (dry bench)"),
                mode_required=True, program=True)
    prog = _fly_common(fly_sub.add_parser("program", help="the FullDDR shell into SRAM"), mode_required=None)
    prog.add_argument("--pack-out", action="store_true")
    prog.add_argument("--flash", action="store_true", help="write the SPI flash first (slow), then SRAM")
    st = _fly_common(fly_sub.add_parser("status", help="ESP link health from its console (RESETS the ESP)"),
                     mode_required=None)
    st.add_argument("--seconds", type=float, default=2.5)
    st.add_argument("--yes", action="store_true", help="skip the 'nothing is flying?' prompt")
    _fly_common(fly_sub.add_parser("stop", help="halt the core; the ESP cuts all motors within 120 ms"))
    _fly_common(fly_sub.add_parser("where", help="halt, backtrace, resume"))
    _fly_common(fly_sub.add_parser(
        "photo", help="halt the photo image and read its camera frame out over JTAG -> PNG "
        "(the core is LEFT HALTED; run after landing)"))
    im = _fly_common(fly_sub.add_parser("imu", help="guided IMU sign test on a motor-incapable image"),
                     mode_required=None, program=True)
    mo = _fly_common(fly_sub.add_parser("motors", help="spin each motor alone (motor identification)"),
                     mode_required=None)
    mo.add_argument("--duty", type=int, default=20, help="percent, 5..25 (default 20)")
    mo.add_argument("--ms", type=int, default=1500, help="spin time per motor (default 1500)")
    mo.add_argument("--motor", type=int, nargs="+", default=[1, 2, 3, 4], choices=[1, 2, 3, 4])
    mo.add_argument("--yes", action="store_true", help="skip the typed confirmation")

    flog = sub.add_parser(
        "flightlog",
        help="publish flight trials to the flight-logs repo and inspect them",
    )
    flog.add_argument("--repo", type=Path, help="flight-logs checkout (default: $RB_FLIGHT_LOGS or ~/flight-logs)")
    flog_sub = flog.add_subparsers(dest="flightlog_action", required=True)
    flog_push = flog_sub.add_parser(
        "push",
        help="split the latest raw log into trials (existing trials keep their numbers), "
        "summarize the new ones, commit and push",
    )
    flog_push.add_argument("--raw", type=Path, help="raw log (default: newest results/riskybird-logs/raw_*.log)")
    flog_push.add_argument("--date", help="session date YYYY-MM-DD (default: from the raw log's name)")
    build_src = flog_push.add_mutually_exclusive_group()
    build_src.add_argument("--build", help="the controller session's build text, recorded verbatim")
    build_src.add_argument("--build-file", type=Path, help="read the build text from a file")
    flog_push.add_argument("--tag", help="short build label for the README row")
    flog_push.add_argument("--note", help="operator comment, e.g. \"good hover, still drifting\"")
    flog_push.add_argument("--dry-run", action="store_true", help="write into the checkout but do not commit or push")
    flog_push.set_defaults(func=cmd_flightlog)
    flog_show = flog_sub.add_parser("show", help="timeline of one trial (T27, 27, or the folder name)")
    flog_show.add_argument("trial")
    flog_show.add_argument("--step", type=float, default=0.2, help="seconds between rows (default 0.2)")
    flog_show.add_argument("--from", dest="t_from", type=float, help="start, seconds from ARMING")
    flog_show.add_argument("--to", dest="t_to", type=float, help="end, seconds from ARMING")
    flog_show.set_defaults(func=cmd_flightlog)
    flog_cmp = flog_sub.add_parser("compare", help="side-by-side flight numbers for several trials")
    flog_cmp.add_argument("trials", nargs="+")
    flog_cmp.set_defaults(func=cmd_flightlog)

    xp = sub.add_parser(
        "xpurt",
        help="[experimental] schedule a workload mix on an XPU-RT hardware pairing and plot it",
    )
    xp_sub = xp.add_subparsers(dest="xpurt_action", required=True)

    xp_doctor = xp_sub.add_parser(
        "doctor",
        help="check the Isaac forest-pilot environment and its model inputs",
    )
    xp_doctor.add_argument("--ready", action="store_true", help="require Python and model checkpoints")
    xp_doctor.add_argument("--python", type=Path, help="Isaac-capable Python interpreter")
    xp_doctor.add_argument("--dronet-weights", type=Path)
    xp_doctor.add_argument("--checkpoint", type=Path, help="RL velocity-tracker checkpoint")
    xp_doctor.add_argument("--schedule", type=Path, default=xpurt.FOREST_SCHEDULE)
    xp_doctor.set_defaults(func=cmd_xpurt)

    xp_summary = xp_sub.add_parser(
        "summarize",
        help="re-derive summary.json and plot.svg from a forest replay",
    )
    xp_summary.add_argument("run_dir", type=Path)
    xp_summary.set_defaults(func=cmd_xpurt)

    xp_plot = xp_sub.add_parser("plot", help="re-render plot.svg from a forest replay")
    xp_plot.add_argument("run_dir", type=Path)
    xp_plot.add_argument("--out", type=Path)
    xp_plot.set_defaults(func=cmd_xpurt)

    xp_prof = xp_sub.add_parser(
        "profiles",
        help="list discovered per-op profiles, dispatch graphs, and workloads",
    )
    xp_prof.set_defaults(func=cmd_xpurt)

    xp_ops = xp_sub.add_parser(
        "ops",
        help="per-operation cycle breakdown for one workload, optionally comparing targets",
    )
    xp_ops.add_argument("workload", help="workload/model name, e.g. mlp_control")
    xp_ops.add_argument(
        "--target",
        required=True,
        help="profiling target to plot, e.g. firesim_rocket_scalar",
    )
    xp_ops.add_argument("--compare", help="second target to plot alongside")
    xp_ops.add_argument("--label", help="legend label for --target")
    xp_ops.add_argument("--compare-label", help="legend label for --compare")
    xp_ops.add_argument("--title")
    xp_ops.add_argument("--out", type=Path)
    xp_ops.set_defaults(func=cmd_xpurt)

    xp_gantt = xp_sub.add_parser(
        "gantt",
        help="legible schedule timeline: colour per op, deadline marker, detail table",
    )
    xp_gantt.add_argument("--schedule", type=Path, help="scheduled_*.json to render")
    xp_gantt.add_argument("--config", help="with --workload, locates the schedule")
    xp_gantt.add_argument("--workload")
    xp_gantt.add_argument("--solver", choices=xpurt.SOLVERS, default="greedy")
    xp_gantt.add_argument(
        "--deadline-ms",
        type=float,
        help="draw a deadline marker at this latency",
    )
    xp_gantt.add_argument("--title")
    xp_gantt.add_argument("--out", type=Path)
    xp_gantt.set_defaults(func=cmd_xpurt)

    xp_run = xp_sub.add_parser("run", help="generate a spec, schedule it, and plot the Gantt")
    xp_run.add_argument(
        "--workloads",
        default="mlp_control,dronet,yolov8_nano",
        help="comma-separated workload descriptor names (see rb xpurt profiles)",
    )
    xp_run.add_argument("--mix", help="name for this mix (default: joined workload names)")
    xp_run.add_argument(
        "--target",
        default="firesim_rocket_saturn",
        help="profiling target the profiles were captured on",
    )
    # No default: with --config these are derived from the elaboration, and a
    # default here would silently overwrite the derived value with a guess.
    # The unbound path falls back to LEGACY_HW below.
    xp_run.add_argument(
        "--hw-p",
        help="override the performance core's profile (default: derived from --config)",
    )
    xp_run.add_argument(
        "--hw-e",
        help="override the efficiency core's profile (default: derived from --config)",
    )
    xp_run.add_argument("--cores-p", type=int, default=1)
    xp_run.add_argument("--cores-e", type=int, default=1)
    xp_run.add_argument("--solver", choices=xpurt.SOLVERS, default="decomposed")
    xp_run.add_argument("--time-limit", type=int, default=60)
    # Unlike the low-level Chipyard commands, this defaults to the deployment
    # target: KU040 is where RiskyBird results have to be admissible.
    xp_run.add_argument("--board", choices=BOARDS, default="ku040")
    xp_run.add_argument(
        "--config",
        help=(
            "Chipyard config to bind this schedule to. Its elaborated DTS supplies "
            "the clock the result is reported at, and its fingerprint is compared "
            "against the machine the profiles were captured on"
        ),
    )
    xp_run.add_argument(
        "--clock-mhz",
        type=float,
        help=(
            "report at this clock instead of the config's declared one; per-op "
            "cycles are converted to wall time once, here"
        ),
    )
    xp_run.add_argument(
        "--rebuild-workspace",
        action="store_true",
        help="discard and rebuild the symlinked XPU-RT mirror",
    )
    xp_run.set_defaults(func=cmd_xpurt)

    schedule_cli = sub.add_parser(
        "schedule",
        help="[experimental] replay a scheduling policy inside an application scenario",
    )
    schedule_sub = schedule_cli.add_subparsers(dest="schedule_action", required=True)
    schedule_replay = schedule_sub.add_parser(
        "replay",
        help="run a scheduled application scenario and archive its evidence",
    )
    schedule_replay.add_argument("scenario", choices=("forest-trail",))
    schedule_replay.add_argument("--policy", choices=("xpurt",), default="xpurt")
    schedule_replay.add_argument("--schedule", type=Path, default=xpurt.FOREST_SCHEDULE)
    schedule_replay.add_argument("--dronet-weights", type=Path)
    schedule_replay.add_argument("--checkpoint", type=Path, help="RL velocity-tracker checkpoint")
    schedule_replay.add_argument("--python", type=Path, help="Isaac-capable Python interpreter")
    schedule_replay.add_argument("--periods", type=int, default=200)
    schedule_replay.add_argument("--trail", choices=("straight", "curved"), default="curved")
    schedule_replay.add_argument("--seed", type=int, default=12)
    schedule_replay.add_argument("--no-yolo", dest="yolo", action="store_false")
    schedule_replay.add_argument("--no-humans", dest="humans", action="store_false")
    schedule_replay.add_argument(
        "--headless",
        action="store_true",
        help="disable UI rendering; video remains available through offscreen Xvfb capture",
    )
    schedule_replay.add_argument("--no-video", dest="video", action="store_false")
    schedule_replay.add_argument("--timeout", type=float, default=3600.0)
    schedule_replay.add_argument("--output-dir", type=Path)
    schedule_replay.set_defaults(func=cmd_schedule, yolo=True, humans=True, video=True)

    ds = sub.add_parser(
        "dse",
        help="[experimental] join resource and performance results into a rankable dataset",
    )
    ds_sub = ds.add_subparsers(dest="dse_action", required=True)

    ds_join = ds_sub.add_parser(
        "join",
        help="pair area records with schedules by fingerprint and record the result",
    )
    ds_join.set_defaults(func=cmd_dse, latest=True)

    for name, helptext in (
        ("list", "list recorded design points"),
        ("report", "table of utilization against latency, plus the Pareto frontier"),
        ("plot", "render design-space figures"),
    ):
        item = ds_sub.add_parser(name, help=helptext)
        item.add_argument(
            "--all",
            dest="latest",
            action="store_false",
            help="include superseded points, not just the newest per configuration",
        )
        if name == "plot":
            item.add_argument("--out", type=Path, default=ROOT / "dse" / "plots")
        item.set_defaults(func=cmd_dse, latest=True)

    area = sub.add_parser(
        "area",
        help="[experimental] characterize FPGA resource utilization per Chipyard config",
    )
    area_sub = area.add_subparsers(dest="area_action", required=True)

    area_timing = area_sub.add_parser(
        "timing",
        help="report achieved Fmax from post-route reports already on disk",
    )
    _add_board(area_timing)
    _add_fpga_config(area_timing)
    area_timing.set_defaults(func=cmd_area)

    area_run = area_sub.add_parser("run", help="synthesize one config and record its area")
    _add_board(area_run)
    _add_fpga_config(area_run)
    _add_attrs(area_run)
    area_run.add_argument(
        "--force",
        action="store_true",
        help="re-synthesize even if the report is up to date",
    )
    area_run.set_defaults(func=cmd_area)

    area_sweep = area_sub.add_parser("sweep", help="run a study's configs in sequence")
    _add_board(area_sweep)
    _add_attrs(area_sweep)
    area_sweep.add_argument("--study", help=f"study name under {STUDIES_DIR}")
    area_sweep.add_argument(
        "--configs",
        help="comma-separated Chipyard config classes, instead of --study",
    )
    area_sweep.add_argument("--force", action="store_true")
    area_sweep.add_argument(
        "--keep-going",
        action="store_true",
        help="continue the sweep when one config fails",
    )
    area_sweep.set_defaults(func=cmd_area)

    area_list = area_sub.add_parser("list", help="list recorded area runs and studies")
    area_list.set_defaults(func=cmd_area)

    area_report = area_sub.add_parser("report", help="print a summary table of recorded runs")
    area_report.add_argument("--run-id", help="report one run instead of all")
    area_report.add_argument(
        "--all",
        action="store_true",
        help="include superseded runs, not just the newest per config",
    )
    area_report.set_defaults(func=cmd_area)

    area_plot = area_sub.add_parser("plot", help="render SVG figures from recorded runs")
    area_plot.add_argument(
        "--out",
        type=Path,
        default=ROOT / "results" / "area" / "figures",
        help="output directory for figures",
    )
    area_plot.add_argument(
        "--all",
        action="store_true",
        help="include superseded runs",
    )
    area_plot.set_defaults(func=cmd_area)

    area_restore = area_sub.add_parser(
        "restore",
        help="undo RTL attribute injection in a config's gen-collateral",
    )
    _add_board(area_restore)
    _add_fpga_config(area_restore)
    area_restore.set_defaults(func=cmd_area)

    mb = sub.add_parser(
        "modelblaster",
        help="run a ModelBlaster model on a RiskyBird shell and report its timing",
    )
    mb_sub = mb.add_subparsers(dest="modelblaster_action", required=True)

    def _add_model(target_parser: argparse.ArgumentParser) -> None:
        target_parser.add_argument(
            "--model", default="dronet",
            help="ModelBlaster example name (default: dronet)",
        )
        target_parser.add_argument(
            "--quant", default="int8", choices=("int8", "fp16", "fp32"),
            help="quantization the codegen emits (default: int8)",
        )
        target_parser.add_argument(
            "--target", default="gemmini_q31",
            help="ModelBlaster kernel backend, e.g. gemmini_q31, rvv, rvv_f16, "
            "scalar (default: gemmini_q31)",
        )
        target_parser.add_argument(
            "--gen-dir",
            help="use these generated sources instead of the codegen default",
        )

    mb_generate = mb_sub.add_parser(
        "generate",
        help="run ModelBlaster stages 1-3 (extract, skeleton, kernels); no simulator",
    )
    _add_model(mb_generate)
    mb_generate.add_argument(
        "--drift-atol", type=int, metavar="LSB",
        help="accept up to LSB of MEASURED numeric drift when verifying a "
        "curated kernel, instead of requiring a bit-exact match. Not a way to "
        "skip verification -- the gate still runs and still measures. On "
        "dronet/gemmini_q31, --drift-atol 2 is what lets conv2d_s8 onto Gemmini "
        "at a bounded 2 LSB, worth 460,351,223 -> 1,989,282 cycles",
    )
    mb_generate.set_defaults(func=cmd_modelblaster)

    mb_build = mb_sub.add_parser(
        "build", help="build the ModelBlaster harness for an FPGA shell",
    )
    _add_model(mb_build)
    _add_board(mb_build)
    _add_fpga_config(mb_build)
    mb_build.add_argument("--pristine", default="auto", choices=("auto", "always", "never"))
    mb_build.add_argument(
        "--ram-console", action="store_true",
        help="console into a RAM buffer readable over JTAG, for shells whose UART is unproven",
    )
    mb_build.set_defaults(func=cmd_modelblaster)

    mb_report = mb_sub.add_parser(
        "report",
        help="turn a console capture into a per-op table, an FPS figure and a timeline",
    )
    mb_report.add_argument("capture", help="console capture containing the harness markers")
    _add_model(mb_report)
    _add_board(mb_report)
    _add_fpga_config(mb_report)
    mb_report.add_argument(
        "--cpu-mhz", type=float,
        help="assert the CPU clock instead of reading it from the config's "
        "devicetree. Recorded in the output as asserted, because a wrong value "
        "here silently scales every millisecond and the FPS figure",
    )
    mb_report.add_argument(
        "--tick-hz", type=float,
        help="the machine's mtime rate, for a capture that did not come from an "
        "FPGA shell. WALL_CYCLES is counted in these, not in CPU cycles; the "
        "Chipyard shells run mtime at the fabric clock over 1000, a spike "
        "capture of this harness measures 100",
    )
    mb_report.add_argument(
        "--out-dir", help="where profile.csv and gantt.svg go (default: beside the capture)",
    )
    mb_report.set_defaults(func=cmd_modelblaster)

    return parser


@contextlib.contextmanager
def restored_terminal():
    """Put the controlling terminal back the way we found it, whatever happens.

    Several subcommands hand the tty to something that turns off echo and
    canonical mode: `rb console` runs `serial.tools.miniterm`, which clears
    ICANON, ECHO *and* ISIG, and `rb capture` uses `tty.setcbreak`. Both restore
    on a clean exit -- miniterm through `atexit`, capture through a `finally` --
    and neither runs on SIGTERM.

    That gap is easy to fall into rather than exotic, because clearing ISIG means
    Ctrl-C no longer interrupts miniterm (it is forwarded to the target; only
    Ctrl-] exits). The natural next move is to kill it from another terminal,
    which is precisely the case `atexit` misses. The shell is then left with echo
    off: typing works but shows nothing, and it survives until `stty sane`.

    Snapshotting here costs nothing and covers every subcommand, including any
    added later.
    """
    if not sys.stdin.isatty():
        yield
        return
    fd = sys.stdin.fileno()
    try:
        saved = termios.tcgetattr(fd)
    except termios.error:
        yield
        return

    def restore(*_):
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        except (termios.error, OSError):
            pass

    previous = {}
    for sig in (signal.SIGTERM, signal.SIGHUP):
        def handler(signum, frame, _sig=sig):
            restore()
            # Re-raise through the default action so the exit status still
            # reports the signal rather than a plain 0.
            signal.signal(_sig, previous.get(_sig, signal.SIG_DFL))
            os.kill(os.getpid(), signum)
        try:
            previous[sig] = signal.signal(sig, handler)
        except (ValueError, OSError):
            pass
    try:
        yield
    finally:
        restore()


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        with restored_terminal():
            return int(args.func(args) or 0)
    except (
        RbError,
        modelblaster.ModelBlasterError,
        # The facet modules' own operator-facing errors. Most commands already
        # translate them to RbError; this is the backstop for any path that
        # does not, which otherwise ended in a raw traceback (`rb xpurt run`
        # did, with XPU-RT absent).
        xpurt.XpurtError,
        rose.RoseError,
        dse.DseError,
        area.AreaError,
        fingerprint.FingerprintError,
        soc.SocError,
        timing.TimingError,
        plotting.PlotError,
        ValueError,
    ) as exc:
        # ModelBlasterError is an operator-facing diagnostic like RbError -- a
        # missing backend, an unverified kernel set, a codegen dependency that
        # is not installed. Printing a traceback for those buried the sentence
        # that says what to do under twenty lines of stack.
        print(f"rb: error: {exc}", file=sys.stderr)
        return 2
    except subprocess.CalledProcessError as exc:
        print(
            f"rb: command failed with exit status {exc.returncode}",
            file=sys.stderr,
        )
        return exc.returncode or 1
    except OSError as exc:
        print(f"rb: operating-system error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nrb: interrupted", file=sys.stderr)
        return 130
