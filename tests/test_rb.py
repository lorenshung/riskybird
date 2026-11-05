"""Regression tests for the top-level RiskyBird flow contract."""

from __future__ import annotations

import dataclasses
import re
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from tools.rb.artifacts import zephyr_build_dir
from tools.rb import boards
from tools.rb.boards import BOARDS, ROOT, TARGETS, WORKLOADS
from tools.rb.cli import (
    RbError,
    build_parser,
    chipyard_build,
    validate_hardware_contract,
    zephyr_build,
)
from tools.rb.environments import (
    ZEPHYR_HOST_BIN,
    ZEPHYR_HOST_LOADER,
    ZEPHYR_PREFIX,
    chipyard_env,
    openocd_command,
    select_zephyr_prefix,
    zephyr_env,
)


class RegistryTests(unittest.TestCase):
    def test_supported_targets_are_deliberately_small(self) -> None:
        self.assertEqual(set(BOARDS), {"arty100t", "arty200t", "ku040"})
        self.assertEqual(
            set(TARGETS),
            {"arty100t", "arty200t", "ku040", "esp32c6", "firesim", "rose-spike"},
        )
        self.assertIn("state", WORKLOADS)
        self.assertIn("vl35l5cx_test", WORKLOADS)
        # Removed: the VL53L0X diagnostics (the drone carries VL53L1X and
        # VL53L5CX) and the ESP-hosted sensor_bringup.
        for name in ("i2c_expander", "tof_sensor", "sensor_bringup"):
            self.assertNotIn(name, WORKLOADS)
        self.assertEqual(WORKLOADS["motor_safe"].targets, ("esp32c6",))

    def test_separate_drone_diagnostics_are_registered_for_all_targets(self) -> None:
        # Every target with a peripheral bus. FireSim's base devicetree has no
        # I2C, SPI or PWM, so none of these can compile there.
        expected_targets = set(TARGETS) - {"rose-spike", "firesim"}
        for name in (
            "ads7128_test",
            "bmi088_test",
            "i2c_scanner",
            "motor_1",
            "motor_2",
            "motor_3",
            "motor_4",
            "pmw3901_test",
            "vl35l5cx_test",
        ):
            self.assertEqual(set(WORKLOADS[name].targets), expected_targets)
        self.assertEqual(
            WORKLOADS["vl35l5cx_test"].modes,
            ("address", "ranging"),
        )
        self.assertEqual(WORKLOADS["vl35l5cx_test"].default_mode, "address")

    def test_firesim_carries_only_peripheral_free_workloads(self) -> None:
        firesim = TARGETS["firesim"]
        on_firesim = {
            name for name, workload in WORKLOADS.items() if workload.supports(firesim)
        }
        self.assertEqual(on_firesim, {"hello_world", "state", "mpc", "ddr_stress"})
        self.assertTrue(WORKLOADS["state"].supports(firesim, "replay"))
        self.assertFalse(WORKLOADS["state"].supports(firesim, "bmi088"))
        self.assertTrue(WORKLOADS["mpc"].supports(firesim, "replay"))
        self.assertFalse(WORKLOADS["mpc"].supports(firesim, "hil"))

    def test_drone_workloads_default_to_the_drone_shell_on_arty200t(self) -> None:
        arty200t = BOARDS["arty200t"]
        for name in (
            "bootup_check", "sensor_check", "flight_controller", "flight_estimator",
            "esp_uart", "motor_duty", "camera_photo",
        ):
            workload = WORKLOADS[name]
            self.assertEqual(
                workload.board_for(arty200t, None).config,
                "RocketArty200TDroneFullDDRConfig",
                name,
            )
            # An explicit --config still wins.
            self.assertEqual(
                workload.board_for(arty200t, "RocketArty200TDroneDDRConfig").config,
                "RocketArty200TDroneDDRConfig",
            )
        # Non-drone workloads keep the board default.
        for name in ("hello_world", "ddr_stress"):
            self.assertEqual(
                WORKLOADS[name].board_for(arty200t, None).config, arty200t.config
            )

    def test_legacy_workloads_and_modes_are_marked(self) -> None:
        for name in (
            "motor_1", "motor_2", "motor_3", "motor_4", "esp_motors",
            "sensor_check", "state", "mpc", "camera_bringup",
        ):
            self.assertTrue(WORKLOADS[name].legacy, name)
            self.assertTrue(WORKLOADS[name].legacy_note(WORKLOADS[name].default_mode))
        fc = WORKLOADS["flight_controller"]
        self.assertFalse(fc.legacy)
        for mode in ("motors", "autoflight"):
            self.assertIn("F13", fc.legacy_note(mode))
        for mode in ("default", "flow", "esp-motors", "telem"):
            self.assertEqual(fc.legacy_note(mode), "", mode)
        # bootup_check's sweep moved to the ESP link (2026-11-03): neither mode
        # drives an FPGA PWM pin, so neither is legacy.
        for mode in ("default", "motors"):
            self.assertEqual(WORKLOADS["bootup_check"].legacy_note(mode), "", mode)
        self.assertEqual(
            dict(WORKLOADS["bootup_check"].cppflags_for("motors")).get("BOOTUP_MOTORS"), "1")
        self.assertEqual(dict(WORKLOADS["bootup_check"].cppflags_for("default")), {})

    def test_flow_modes_compile_flow_in(self) -> None:
        for name in ("flight_controller", "flight_estimator"):
            defines = dict(WORKLOADS[name].defines_for("flow"))
            self.assertEqual(defines.get("ROSE_FLOW"), "1", name)
            self.assertEqual(defines.get("CS_GPIO_PIN"), "0", name)
        # The estimator's uplink reader needs uart1 on the drone shell.
        self.assertEqual(
            dict(WORKLOADS["flight_estimator"].defines_for("default")).get(
                "CONFIG_UART_SIFIVE_PORT_1"
            ),
            "y",
        )

    def test_esp_is_a_zephyr_target_not_an_fpga_board(self) -> None:
        self.assertIsNone(TARGETS["esp32c6"].fpga_board)
        self.assertEqual(TARGETS["arty100t"].fpga_board, BOARDS["arty100t"])
        self.assertEqual(
            TARGETS["esp32c6"].zephyr_board,
            "esp32c6_devkitc/esp32c6/hpcore",
        )

    def test_build_artifacts_are_target_workload_and_mode_specific(self) -> None:
        path = zephyr_build_dir(
            TARGETS["esp32c6"],
            WORKLOADS["state"],
            "bmi088",
        )
        self.assertEqual(
            path,
            ROOT / "build" / "zephyr" / "esp32c6" / "state" / "bmi088",
        )

    def test_physical_memory_contract(self) -> None:
        self.assertEqual(BOARDS["arty100t"].memory_size, 1 << 30)
        self.assertEqual(BOARDS["ku040"].memory_size, 2 << 30)
        self.assertEqual(BOARDS["ku040"].physical_memory_size, 2 << 30)
        self.assertEqual(BOARDS["ku040"].physical_memory_type, "DDR4")
        self.assertIn(
            "place-and-route verified",
            BOARDS["ku040"].physical_memory_evidence,
        )

    def test_tof_ranging_requires_an_fpga_memory_capable_config(self) -> None:
        workload = WORKLOADS["vl35l5cx_test"]
        self.assertEqual(workload.minimum_ram("address"), 0)
        self.assertEqual(workload.minimum_ram("ranging"), 192 * 1024)

        # ku040 now backs main memory with 2 GiB of DDR4, so the guard no longer
        # rejects it. Checked against the registry rather than by invoking
        # zephyr_build, which would run a real Zephyr build.
        self.assertLessEqual(
            workload.minimum_ram("ranging"), BOARDS["ku040"].memory_size
        )

        # The guard itself still has to reject a scratchpad-only configuration.
        scratchpad = dataclasses.replace(
            BOARDS["ku040"],
            config="RocketKU040OspiConfig",
            memory_size=32 * 1024,
        )
        with self.assertRaisesRegex(RbError, "requires at least 192 KiB"):
            zephyr_build(
                TARGETS["ku040"],
                workload,
                "ranging",
                "auto",
                fpga_board=scratchpad,
            )

    def test_sensor_capable_shells_are_selected(self) -> None:
        self.assertEqual(BOARDS["arty100t"].config, "RocketArty100TOspiConfig")
        self.assertEqual(BOARDS["ku040"].config, "RocketKU040OspiDDRConfig")

    def test_chipyard_config_selects_matching_artifacts(self) -> None:
        board = BOARDS["ku040"].with_config("SaturnKU040OspiConfig")
        self.assertEqual(board.config, "SaturnKU040OspiConfig")
        self.assertEqual(
            board.generated_dir.name,
            "chipyard.fpga.ku040.KU040Harness.SaturnKU040OspiConfig",
        )
        self.assertEqual(board.bitstream.name, "KU040Harness.bit")
        self.assertEqual(BOARDS["ku040"].config, "RocketKU040OspiDDRConfig")

    def test_chipyard_config_rejects_make_syntax(self) -> None:
        with self.assertRaisesRegex(ValueError, "expected a Scala class name"):
            BOARDS["ku040"].with_config("RocketKU040Config OTHER=value")

    def test_ku040_scratchpad_configs_present_32_kib(self) -> None:
        """WithKU040Tweaks defaults to ddr = false, so the unmarked KU040 names
        are the scratchpad ones and the marker convention cannot catch them."""
        board = BOARDS["ku040"]
        for config in (
            "RocketKU040Config",
            "RocketKU040OspiConfig",
            "RocketKU040DroneLogicConfig",
            "NoCoresKU040Config",
        ):
            selected = board.with_config(config)
            self.assertTrue(selected.is_scratchpad_config, config)
            self.assertEqual(selected.config_memory_size, 0x8000, config)
            self.assertEqual(selected.overlay.name, "ku040-scratchpad.overlay")
            self.assertTrue(selected.overlay.is_file(), config)

        # The DDR counterparts must not be swept up by a prefix match.
        for config in ("RocketKU040DDRConfig", "RocketKU040OspiDDRConfig",
                       "RocketKU040DroneLogicDDRConfig", "CnnNavKU040Config"):
            selected = board.with_config(config)
            self.assertFalse(selected.is_scratchpad_config, config)
            self.assertEqual(selected.config_memory_size, board.memory_size, config)

    def test_registry_scratchpad_flags_match_the_chipyard_configs(self) -> None:
        """Every KU040 config's registry classification must follow the ddr
        argument it actually elaborates with, not just its name."""
        source = (
            boards.CHIPYARD_OVERLAY
            / "fpga"
            / "src"
            / "main"
            / "scala"
            / "ku040"
            / "Configs.scala"
        ).read_text()
        board = BOARDS["ku040"]
        checked = 0
        for match in re.finditer(
            r"^class (\w+) extends Config\(\n((?:.*\n)*?)^\s*new [^\n]*\)\n",
            source,
            re.M,
        ):
            tweaks = re.search(r"new WithKU040Tweaks(\([^)]*\))?", match.group(0))
            if tweaks is None:
                continue
            checked += 1
            name = match.group(1)
            elaborates_ddr = "ddr = true" in (tweaks.group(1) or "")
            self.assertEqual(
                board.with_config(name).is_scratchpad_config,
                not elaborates_ddr,
                f"{name} elaborates ddr = {elaborates_ddr} but the registry "
                "classifies it the other way; update SCRATCHPAD_CONFIG_NAMES",
            )
        self.assertGreater(checked, 20)

    def test_top_level_hardware_inputs_exist(self) -> None:
        for board in BOARDS.values():
            self.assertTrue(board.overlay.is_file())
            self.assertTrue(board.openocd_target.is_file())
        self.assertTrue(
            (ROOT / "hardware" / "zephyr" / "fpga-drone.overlay").is_file()
        )
        for target in TARGETS.values():
            if target.kind in {"rose", "firesim"}:
                continue
            for workload_name in ("motor_4", "pmw3901_test"):
                self.assertIsNotNone(
                    target.workload_overlay(workload_name, "default")
                )


class ContractTests(unittest.TestCase):
    def test_generated_chipyard_dts_contracts(self) -> None:
        for board in BOARDS.values():
            if not board.generated_dts.is_file():
                self.skipTest(f"generated DTS not initialized: {board.generated_dts}")
            details = validate_hardware_contract(board)
            self.assertIn("UART 0x10020000 irq 2", details)
            self.assertIn("I2C 0x10040000 irq 1", details)

    def test_base_config_does_not_require_optional_i2c(self) -> None:
        board = BOARDS["arty100t"].with_config("RocketArty100TConfig")
        if not board.generated_dts.is_file():
            self.skipTest(f"generated DTS not initialized: {board.generated_dts}")
        details = validate_hardware_contract(
            board,
            require_i2c=False,
            require_uart_irq=None,
        )
        self.assertIn("UART 0x10020000 irq 1", details)
        self.assertNotIn("I2C 0x10040000 irq 1", details)

    def test_generated_drone_logic_contracts(self) -> None:
        configs = {
            "arty100t": "RocketArty100TDroneLogicConfig",
            "ku040": "RocketKU040DroneLogicDDRConfig",
        }
        for name, config in configs.items():
            board = BOARDS[name].with_config(config)
            if not board.generated_dts.is_file():
                self.skipTest(
                    f"generated DroneLogic DTS not initialized: {board.generated_dts}"
                )
            details = validate_hardware_contract(board)
            self.assertIn("UART 0x10020000 irq 10", details)
            self.assertIn("I2C 0x10040000 irq 1", details)
            self.assertIn("PWM 0x10050000/0x10051000 irq 2..9", details)
            self.assertIn("GPIO 0x10010000 irq 11..13", details)
            self.assertIn("SPI 0x10031000 irq 14", details)

    def test_target_overlays_own_pins_and_logical_channels(self) -> None:
        target_root = ROOT / "hardware" / "zephyr" / "targets"
        esp_contract = (target_root / "esp32c6" / "target.overlay").read_text()
        self.assertIn("I2C0_SDA_GPIO15", esp_contract)
        self.assertIn("I2C0_SCL_GPIO14", esp_contract)
        self.assertIn("reg = <0x0 DT_SIZE_M(4)>", esp_contract)
        self.assertIn("gyroscope@68", esp_contract)
        self.assertNotIn("gyroscope@69", esp_contract)

        for path in (target_root / "esp32c6" / "workloads").glob("*.overlay"):
            text = path.read_text()
            self.assertNotIn(
                "i2c0_default:",
                text,
                f"{path.name} duplicates the ESP base-board I2C pin contract",
            )
            self.assertNotIn(
                "gyroscope@69",
                text,
                f"{path.name} conflicts with the ESP base-board BMI088 address",
            )

        for name in ("fpga-common.overlay", "fpga-drone.overlay"):
            text = (ROOT / "hardware" / "zephyr" / name).read_text()
            self.assertIn("gyroscope@68", text)
            self.assertNotIn("gyroscope@69", text)

        esp_motor = (
            target_root / "esp32c6" / "workloads" / "motor_4.overlay"
        ).read_text()
        for pin in ("GPIO21", "GPIO20", "GPIO23", "GPIO22"):
            self.assertIn(pin, esp_motor)

        esp_motor_safe = (
            target_root / "esp32c6" / "workloads" / "motor_safe.overlay"
        ).read_text()
        for pin in ("21", "20", "23", "22"):
            self.assertRegex(esp_motor_safe, rf"<{pin}\s+GPIO_PULL_DOWN>")
        self.assertIn("gpio-hog;", esp_motor_safe)
        self.assertIn("input;", esp_motor_safe)
        self.assertIn("zephyr,console = &usb_serial", esp_motor_safe)
        self.assertRegex(
            esp_motor_safe,
            r"&ledc0\s*\{\s*status = \"disabled\";",
        )

        fpga_motor = (
            target_root / "fpga" / "workloads" / "motor_4.overlay"
        ).read_text()
        for mapping in (
            "<&pwm0 1 50000>",
            "<&pwm0 2 50000>",
            "<&pwm0 3 50000>",
            "<&pwm1 1 50000>",
        ):
            self.assertIn(mapping, fpga_motor)

        tof_source = (
            ROOT / "workloads" / "vl35l5cx_test" / "src" / "main.c"
        ).read_text()
        # Channels 1, 2, 3 and 6 -- not 1..4. Measured with rb_tof_scan_xshut,
        # which releases each unclaimed ADS7128 channel on its own and probes
        # the default address; channel 4 gates nothing on this board.
        self.assertIn("{ 1, 2, 3, 6, RB_TOF_XSHUT_DIRECT }", tof_source)
        self.assertIn("{ 0x31, 0x32, 0x33, 0x34, 0x35 }", tof_source)
        # And every ToF is behind the expander: rb_tof_hold_all drives all
        # channels low and no VL53 part answers, so sensor_count is the four
        # that exist rather than a fifth that does not.
        self.assertIn(".sensor_count = 4", tof_source)

        tof_api = (
            ROOT
            / "workloads"
            / "lib"
            / "tof"
            / "include"
            / "riskybird"
            / "tof.h"
        ).read_text()
        self.assertIn("struct rb_tof_frame", tof_api)
        self.assertIn("host_timestamp_us", tof_api)
        self.assertIn("target_status[RB_TOF_MAX_ZONES]", tof_api)


class EnvironmentTests(unittest.TestCase):
    def test_zephyr_prefix_supports_clean_and_existing_installations(self) -> None:
        with TemporaryDirectory() as directory:
            envs_dir = Path(directory)
            self.assertEqual(select_zephyr_prefix(envs_dir), envs_dir / "zephyr")

            (envs_dir / "zephyr").mkdir()
            self.assertEqual(select_zephyr_prefix(envs_dir), envs_dir / "zephyr")

            (envs_dir / "zephyr-new").mkdir()
            self.assertEqual(
                select_zephyr_prefix(envs_dir),
                envs_dir / "zephyr-new",
            )

    def test_zephyr_prefix_wins_over_parent_environment(self) -> None:
        environment = zephyr_env()
        self.assertEqual(environment["RB_ENV"], "zephyr")
        self.assertEqual(environment["CONDA_DEFAULT_ENV"], ZEPHYR_PREFIX.name)
        self.assertEqual(environment["PATH"].split(":")[0], str(ZEPHYR_PREFIX / "bin"))
        self.assertNotIn("VIRTUAL_ENV", environment)

    def test_openocd_command_matches_what_is_installed(self) -> None:
        """The loader is a workaround, not a requirement.

        openocd_command() prefers the SDK's own binary launched through the ELF
        loader -- which only exists for an SDK that was MOVED after install and
        kept a stale interpreter path -- then the SDK binary directly, then
        whatever is on PATH. A macOS host has no ELF loader at all and must not
        be forced down the first branch.

        Asserted against what is actually present so this passes on a checkout
        with no backend installed as well as on a full one.
        """
        command = openocd_command("--version")
        self.assertIn("--version", command)
        sdk_openocd = ZEPHYR_HOST_BIN / "openocd"
        if ZEPHYR_HOST_LOADER.is_file() and sdk_openocd.is_file():
            self.assertEqual(command[0], str(ZEPHYR_HOST_LOADER))
            self.assertIn(str(sdk_openocd), command)
        elif sdk_openocd.is_file():
            self.assertEqual(command[0], str(sdk_openocd))
        else:
            self.assertEqual(command[0], "openocd")

    def test_chipyard_runtime_is_repository_local(self) -> None:
        environment = chipyard_env()
        runtime = Path(environment["XDG_RUNTIME_DIR"])
        self.assertTrue(runtime.is_relative_to(ROOT / ".rb"))
        self.assertEqual(runtime.stat().st_mode & 0o777, 0o700)
        self.assertIn("-Dsbt.server.forcestart=true", environment["SBT_OPTS"])

    def test_fpga_irq_count_is_a_hardware_kconfig_default(self) -> None:
        hardware_kconfig = ROOT / "hardware" / "zephyr" / "Kconfig.fpga"
        self.assertIn("default 27", hardware_kconfig.read_text())
        self.assertNotIn(
            "CONFIG_NUM_IRQS=",
            (ROOT / "hardware" / "zephyr" / "fpga.conf").read_text(),
        )
        for workload in (WORKLOADS["state"], WORKLOADS["mpc"]):
            self.assertIn(
                'rsource "../../hardware/zephyr/Kconfig.fpga"',
                (workload.source_dir / "Kconfig").read_text(),
            )


class ParserTests(unittest.TestCase):
    def test_high_level_build(self) -> None:
        args = build_parser().parse_args(
            ["build", "state", "--target", "ku040", "--mode", "bmi088"]
        )
        self.assertEqual(args.workload, "state")
        self.assertEqual(args.target, "ku040")
        self.assertEqual(args.mode, "bmi088")

    def test_high_level_build_accepts_drone_logic_config(self) -> None:
        args = build_parser().parse_args(
            [
                "build",
                "motor_4",
                "--target",
                "arty100t",
                "--config",
                "RocketArty100TDroneLogicConfig",
            ]
        )
        self.assertEqual(args.target, "arty100t")
        self.assertEqual(args.config, "RocketArty100TDroneLogicConfig")

    def test_legacy_board_spelling_selects_a_zephyr_target(self) -> None:
        args = build_parser().parse_args(
            ["build", "i2c_scanner", "--board", "esp32c6"]
        )
        self.assertEqual(args.target, "esp32c6")

    def test_full_flow(self) -> None:
        args = build_parser().parse_args(
            [
                "run",
                "mpc",
                "--target",
                "ku040",
                "--config",
                "SaturnKU040OspiConfig",
                "--no-program",
            ]
        )
        self.assertEqual(args.workload, "mpc")
        self.assertEqual(args.config, "SaturnKU040OspiConfig")
        self.assertTrue(args.no_program)

    def test_rose_hil_flow_is_explicitly_fpga_free(self) -> None:
        args = build_parser().parse_args(
            [
                "hil",
                "replay",
                "mpc-hover",
                "--executor",
                "rose-spike",
                "--clock-mhz",
                "100",
                "--steps",
                "25",
                "--no-build",
            ]
        )
        self.assertEqual(args.executor, "rose-spike")
        self.assertEqual(args.clock_mhz, 100.0)
        self.assertEqual(args.steps, 25)
        self.assertTrue(args.no_build)

    def test_xpurt_forest_schedule_replay_is_host_cuda(self) -> None:
        args = build_parser().parse_args(
            [
                "schedule",
                "replay",
                "forest-trail",
                "--policy",
                "xpurt",
                "--periods",
                "3",
                "--trail",
                "straight",
                "--headless",
                "--no-video",
            ]
        )
        self.assertEqual(args.policy, "xpurt")
        self.assertEqual(args.periods, 3)
        self.assertEqual(args.trail, "straight")
        self.assertTrue(args.headless)
        self.assertFalse(args.video)

    def test_capture_records_a_target_specific_session(self) -> None:
        args = build_parser().parse_args(
            [
                "capture",
                "bmi088_test",
                "--target",
                "esp32c6",
                "--duration",
                "15",
                "--output-dir",
                "results/manual-bmi088",
            ]
        )
        self.assertEqual(args.target, "esp32c6")
        self.assertEqual(args.duration, 15)
        self.assertEqual(args.output_dir, "results/manual-bmi088")

    def test_low_level_chipyard_config(self) -> None:
        args = build_parser().parse_args(
            [
                "chipyard",
                "build",
                "--board",
                "ku040",
                "--config",
                "SaturnKU040OspiConfig",
            ]
        )
        self.assertEqual(args.config, "SaturnKU040OspiConfig")

    @patch("tools.rb.cli.executable", return_value=Path("/tools/bin/espresso"))
    @patch("tools.rb.cli.run")
    def test_chipyard_config_is_forwarded_to_make(
        self,
        mock_run,
        _mock_executable,
    ) -> None:
        chipyard_build(
            BOARDS["ku040"].with_config("SaturnKU040OspiConfig"),
            "bitstream",
        )
        command = mock_run.call_args.args[0]
        self.assertIn("SUB_PROJECT=ku040", command)
        self.assertIn("CONFIG=SaturnKU040OspiConfig", command)
        self.assertEqual(command[-1], "bitstream")

    @patch("tools.rb.cli.executable", return_value=None)
    def test_saturn_config_reports_missing_espresso(self, _mock_executable) -> None:
        with self.assertRaisesRegex(RbError, "requires the Espresso"):
            chipyard_build(
                BOARDS["ku040"].with_config("SaturnKU040OspiConfig"),
                "verilog",
            )


if __name__ == "__main__":
    unittest.main()
