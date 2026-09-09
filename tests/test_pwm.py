"""The motor PWM contract: driver, RTL, package pins and devicetree.

Four independent descriptions of the same four signals have to agree, and none
of them can see the others:

  * Zephyr's sifive driver, whose register offsets and pwmcfg bit positions are
    plain #defines,
  * the elaborated RTL, whose register map chipyard emits as JSON,
  * the harness binder, which decides which comparator reaches which package
    pin and with what polarity,
  * the devicetree overlay, which tells the driver which comparator is which
    motor.

A disagreement between any two is silent on the bench -- the registers read
back exactly what was written either way -- so it is checked here instead.
"""

from __future__ import annotations

import json
import re
import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

DRIVER = (
    ROOT / "backends/zephyr-chipyard-sw/zephyr_ws/zephyr/drivers/pwm/pwm_sifive.c"
)
BINDER = ROOT / "backends/chipyard/fpga/src/main/scala/arty200t/HarnessBinders.scala"
CONFIGS = ROOT / "backends/chipyard/fpga/src/main/scala/arty200t/Configs.scala"
OVERLAY = ROOT / "hardware/zephyr/targets/fpga/workloads/motor_4.overlay"
WORKLOAD = ROOT / "workloads/motor_4/src/main.c"
BOOTUP_WORKLOAD = ROOT / "workloads/bootup_check/src/main.c"
DRONE_OVERLAY = ROOT / "hardware/zephyr/fpga-drone.overlay"
BOARD_PINS = (
    ROOT / "arty_files/te0712-rd/board_files/TE0712_200_2C/1.0/part0_pins.xml"
)
MOTOR_CONFIG = "RocketArty200TMotorSpadConfig"
GENERATED = (
    ROOT / "backends/chipyard/fpga/generated-src"
    / f"chipyard.fpga.arty200t.Arty200THarness.{MOTOR_CONFIG}"
)

# The mapping this repository commits to, in one place. Everything below checks
# some other file against it.
#   motor -> (pwm block, comparator, package pin, TE0712 J1 position)
MOTORS = {
    "motor1": (0, 1, "E22", "J1_31"),
    "motor2": (0, 2, "B20", "J1_36"),
    "motor3": (0, 3, "R17", "J1_99"),
    "motor4": (1, 1, "F13", "J1_100"),
}


def driver_defines() -> dict[str, str]:
    text = DRIVER.read_text(encoding="utf-8")
    out = {}
    for name, value in re.findall(
        r"^#define\s+(\w+(?:\([^)]*\))?)\s+(.+?)$", text, re.MULTILINE
    ):
        out[name] = value.strip()
    return out


class DriverAgainstRegmapTests(unittest.TestCase):
    """Zephyr's #defines against the register map the RTL actually emitted."""

    @classmethod
    def setUpClass(cls) -> None:
        candidates = sorted(GENERATED.glob("*.0x10050000.0.regmap.json"))
        if not candidates:
            raise unittest.SkipTest(
                f"{MOTOR_CONFIG} has not been elaborated; run "
                f"./rb chipyard elaborate --board arty200t --config {MOTOR_CONFIG}"
            )
        peripheral = json.loads(candidates[0].read_text())["peripheral"]
        cls.fields = {}
        for entry in peripheral["regfields"]:
            for name, field in entry.items():
                if name == "reserved":
                    continue
                cls.fields[name] = field

    def test_register_offsets_match_the_driver(self) -> None:
        defines = driver_defines()
        for field, define in (
            ("pwmscale", "REG_PWMCFG"),
            ("pwmcount", "REG_PWMCOUNT"),
            ("pwms", "REG_PWMS"),
            ("pwmcmp0", "REG_PWMCMP0"),
        ):
            self.assertEqual(
                int(self.fields[field]["byteOffset"], 16),
                int(defines[define], 0),
                f"{define} does not match the elaborated {field} offset",
            )

    def test_comparator_stride_is_four_bytes(self) -> None:
        base = int(self.fields["pwmcmp0"]["byteOffset"], 16)
        for channel in range(4):
            self.assertEqual(
                int(self.fields[f"pwmcmp{channel}"]["byteOffset"], 16),
                base + 4 * channel,
                "REG_PWMCMP(_channel) assumes a 4-byte stride",
            )

    def test_pwmcfg_bit_positions_match_the_driver(self) -> None:
        defines = driver_defines()
        for field, define in (
            ("pwmsticky", "SF_PWMSTICKY"),
            ("pwmzerocmp", "SF_PWMZEROCMP"),
            ("pwmdeglitch", "SF_PWMDEGLITCH"),
            ("pwmenalways", "SF_PWMENALWAYS"),
            ("pwmoneshot", "SF_PWMENONESHOT"),
        ):
            self.assertEqual(
                self.fields[field]["bitOffset"],
                int(defines[define], 0),
                f"{define} does not match the elaborated {field} bit",
            )
        for channel in range(4):
            self.assertEqual(
                self.fields[f"pwmcmp{channel}center"]["bitOffset"], 16 + channel
            )
            self.assertEqual(self.fields[f"pwmgang{channel}"]["bitOffset"], 24 + channel)
            self.assertEqual(self.fields[f"pwmip{channel}"]["bitOffset"], 28 + channel)

    def test_scale_field_is_four_bits(self) -> None:
        # The driver's SF_PWMSCALEMASK is 0xF and it refuses a period needing
        # more, so a narrower field would silently truncate the scale instead.
        self.assertEqual(self.fields["pwmscale"]["bitWidth"], 4)
        self.assertEqual(int(driver_defines()["SF_PWMSCALEMASK"], 0), 0xF)

    def test_invert_bits_exist_and_the_driver_leaves_them_alone(self) -> None:
        """The harness inverts in hardware, so software must not invert again.

        pwmcfg[20+n] does the same job from software. WithArty200TPWM does not
        rely on it -- it cannot, because those bits are zero until software
        writes them, which leaves the whole power-on window uncovered -- but a
        driver that started setting them would invert a second time and put the
        motors back at full throttle on a stop command.
        """
        for channel in range(4):
            self.assertEqual(
                self.fields[f"pwminvert{channel}"]["bitOffset"], 20 + channel
            )
        text = DRIVER.read_text(encoding="utf-8")
        for bit in range(20, 24):
            self.assertNotIn(
                f"({bit})", text.replace(" ", ""),
                "the sifive driver appears to write a pwminvert bit; "
                "WithArty200TPWM already inverts at the pin",
            )
        self.assertNotIn("PWMINVERT", text.upper())


class PackagePinTests(unittest.TestCase):
    """The four pins against the TE0712 board file."""

    @classmethod
    def setUpClass(cls) -> None:
        if not BOARD_PINS.is_file():
            raise unittest.SkipTest(f"{BOARD_PINS} is not present")
        cls.by_loc = {
            pin.get("loc"): pin.get("name")
            for pin in ET.parse(BOARD_PINS).getroot().iter("pin")
        }

    def test_every_motor_pin_is_a_real_user_io_on_the_module(self) -> None:
        for motor, (_, _, pin, connector) in MOTORS.items():
            self.assertIn(
                pin, self.by_loc, f"{motor}: {pin} is not a TE0712 user I/O"
            )
            self.assertEqual(
                self.by_loc[pin], connector,
                f"{motor}: {pin} is {self.by_loc[pin]}, not {connector}",
            )

    def test_motor_pins_do_not_collide_with_other_signals(self) -> None:
        binder = BINDER.read_text(encoding="utf-8")
        used = set(re.findall(r'"([A-Z]{1,2}\d{1,2})"', binder))
        motor_pins = {pin for _, _, pin, _ in MOTORS.values()}
        for pin in motor_pins:
            # Each motor pin must appear exactly once across the whole binder
            # file: twice would mean two peripherals constrained to one ball.
            self.assertEqual(
                binder.count(f'"{pin}"'), 1,
                f"{pin} is named more than once in HarnessBinders.scala",
            )
        self.assertEqual(len(motor_pins), 4, "two motors share a package pin")
        self.assertTrue(motor_pins <= used)


class BinderTests(unittest.TestCase):
    def setUp(self) -> None:
        self.binder = BINDER.read_text(encoding="utf-8")

    def test_binder_maps_the_committed_block_comparator_and_pin(self) -> None:
        body = self.binder.split("class WithArty200TPWM", 1)[1]
        for motor, (block, cmp_index, pin, _) in MOTORS.items():
            self.assertRegex(
                body,
                rf'\("{motor}",\s*{block},\s*{cmp_index},\s*"{pin}"\)',
                f"{motor} is not bound to block {block} comparator {cmp_index} on {pin}",
            )

    def test_no_motor_is_bound_to_comparator_zero(self) -> None:
        # Comparator 0 sets the shared period; Zephyr refuses to drive it.
        for motor, (_, cmp_index, _, _) in MOTORS.items():
            self.assertNotEqual(cmp_index, 0, motor)

    def test_outputs_are_inverted_by_default(self) -> None:
        """An uninverted build is full throttle at power-on. See the binder."""
        body = self.binder.split("class WithArty200TPWM", 1)[1]
        self.assertIn("invert: Boolean = true", body)
        self.assertIn("~port.io.gpio(cmp)", body)

    def test_motor_pins_are_pulled_down(self) -> None:
        body = self.binder.split("class WithArty200TPWM", 1)[1].split("\nclass ", 1)[0]
        self.assertIn("addPulldown", body)


class ConfigTests(unittest.TestCase):
    def setUp(self) -> None:
        self.configs = CONFIGS.read_text(encoding="utf-8")

    def test_motor_config_exists_and_punches_the_pwm_ports_through(self) -> None:
        body = self.configs.split(f"class {MOTOR_CONFIG}", 1)
        self.assertEqual(len(body), 2, f"{MOTOR_CONFIG} is missing")
        body = body[1].split("new chipyard.RocketConfig)", 1)[0]
        self.assertIn("WithArty200TPWM", body)
        self.assertIn("WithPWMPunchthrough", body)

    def test_motor_config_is_a_scratchpad_name(self) -> None:
        from tools.rb.boards import BOARDS

        board = BOARDS["arty200t"].with_config(MOTOR_CONFIG)
        self.assertTrue(board.is_scratchpad_config)


class OverlayTests(unittest.TestCase):
    def setUp(self) -> None:
        self.overlay = OVERLAY.read_text(encoding="utf-8")

    def test_overlay_channels_match_the_binder(self) -> None:
        """The overlay picks the comparator; the binder picks the pin.

        If these drift, every motor still runs -- just not the one the caller
        named. Nothing on the bench reports that.
        """
        for index, (motor, (block, cmp_index, _, _)) in enumerate(MOTORS.items(), 1):
            self.assertRegex(
                self.overlay,
                rf"motor{index}_pwm:\s*motor{index}-pwm\s*\{{\s*"
                rf"pwms\s*=\s*<&pwm{block}\s+{cmp_index}\s+50000>;",
                f"{motor} overlay node does not name pwm{block} comparator {cmp_index}",
            )

    def test_overlay_aliases_cover_all_four_motors(self) -> None:
        for index in range(4):
            self.assertRegex(
                self.overlay, rf"pwm-motor{index}\s*=\s*&motor{index + 1}_pwm;"
            )

    def test_both_pwm_blocks_are_declared_at_the_elaborated_addresses(self) -> None:
        drone = DRONE_OVERLAY.read_text(encoding="utf-8")
        self.assertIn("pwm0: pwm@10050000", drone)
        self.assertIn("pwm1: pwm@10051000", drone)
        self.assertEqual(drone.count('compatible = "sifive,pwm0"'), 2)

    def test_declared_compare_width_matches_the_driver_assumption(self) -> None:
        drone = DRONE_OVERLAY.read_text(encoding="utf-8")
        self.assertEqual(drone.count("sifive,compare-width = <16>"), 2)


class WorkloadTests(unittest.TestCase):
    """The on-chip probe reads the peripheral directly, so it needs its own map.

    motor_4 samples pwmcfg's live comparator bits to measure the waveform, which
    means it hardcodes which block and comparator each motor is on rather than
    going through the devicetree. A third copy of the mapping is a third thing
    that can drift, so it is checked against the same table as the other two.
    """

    def setUp(self) -> None:
        self.source = WORKLOAD.read_text(encoding="utf-8")

    def test_probe_block_and_comparator_map_matches(self) -> None:
        blocks = re.search(r"motor_block\[4\]\s*=\s*\{([^}]*)\}", self.source)
        comparators = re.search(r"motor_cmp\[4\]\s*=\s*\{([^}]*)\}", self.source)
        self.assertIsNotNone(blocks)
        self.assertIsNotNone(comparators)
        parsed_blocks = [int(v) for v in blocks.group(1).replace(" ", "").split(",") if v]
        parsed_cmps = [int(v) for v in comparators.group(1).replace(" ", "").split(",") if v]
        self.assertEqual(parsed_blocks, [m[0] for m in MOTORS.values()])
        self.assertEqual(parsed_cmps, [m[1] for m in MOTORS.values()])

    def test_probe_reads_the_registers_the_regmap_declares(self) -> None:
        for define, value in (
            ("PWM0_BASE", "0x10050000"),
            ("PWM1_BASE", "0x10051000"),
            ("PWM_CFG", "0x00"),
            ("PWM_COUNT", "0x08"),
            ("PWM_S", "0x10"),
        ):
            self.assertRegex(self.source, rf"#define\s+{define}\s+{value}")
        self.assertIn("#define PWM_IP_BIT(n)   (28U + (n))", self.source)

    def test_probe_unwraps_rather_than_dropping_samples(self) -> None:
        """Dropping wrapped samples biases the measured duty low.

        The midpoint of a straddled read sits half a read-window ahead of the
        first phase sample, so the phases just after zero are only reachable
        from `before` values just before the wrap. Discarding those halves bin
        0's population, and bin 0 is where the pin is high at low duty.
        Simulation measured 10% as 86 per mille that way and 98 with the fix.
        """
        self.assertIn("(after + period[block] - before)", self.source)

    def test_probe_averages_per_bin_rather_than_pooling(self) -> None:
        """Pooling samples across unevenly populated bins biases the duty.

        Measured on hardware: a fixed-length sampling loop aliased onto a
        near-integer fraction of the PWM period, leaving consecutive bins with
        615 and 1 samples and three bins never visited. The pooled estimator
        reported 153 per mille for a 100 per mille request; the per-bin mean
        reports 101.
        """
        self.assertIn("permille_sum += (1000U * (total - high)) / total;", self.source)
        self.assertIn("used ? permille_sum / used : 0U", self.source)

    def test_probe_jitters_the_sampling_interval(self) -> None:
        self.assertRegex(self.source, r"rng = rng \* 1103515245u \+ 12345u;")
        self.assertRegex(self.source, r"for \(volatile uint32_t spin")

    def test_arming_needs_the_magic_word_and_expires(self) -> None:
        self.assertIn("#define MOTOR_ARM_MAGIC   0xA5A5C0DEu", self.source)
        self.assertRegex(self.source, r"request == MOTOR_ARM_MAGIC")
        self.assertRegex(self.source, r"k_uptime_get\(\) > arm_deadline")

    def test_arm_request_is_consumed_not_renewed(self) -> None:
        """A standing request word is not a watchdog, it is the opposite of one.

        The request lives in plain RAM and outlives whatever wrote it, so
        treating "still present" as a renewal means a detached or dead debugger
        pushes the deadline out every loop, forever. Measured on the board:
        armed at 10%, still armed 12 seconds later, zero expiries. Consuming it
        makes a renewal an actual write.
        """
        self.assertIn("motor_arm_request = MOTOR_ARM_CONSUMED;", self.source)
        self.assertIn("request == MOTOR_ARM_CONSUMED", self.source)
        self.assertNotRegex(
            self.source, r"else \{\s*\n\s*arm_deadline = k_uptime_get\(\)"
        )

    def test_arm_window_override_is_bounded(self) -> None:
        self.assertIn("#define MOTOR_ARM_WINDOW_MAX_MS", self.source)
        self.assertRegex(self.source, r"window > MOTOR_ARM_WINDOW_MAX_MS")

    def test_spin_script_resumes_the_core_before_detaching(self) -> None:
        """Detach alone leaves the core halted while the PWM keeps driving.

        The PWM block is independent hardware: a halted CPU stops the main loop
        that enforces the arm window without stopping the motors.
        """
        # Compare command lines, not raw offsets: "detach" also appears in the
        # script's own header prose, which is earlier in the file.
        commands = [
            line.strip()
            for line in (ROOT / "hardware/pwm/spin.gdb")
            .read_text(encoding="utf-8")
            .splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        ]
        self.assertIn("monitor resume", commands)
        self.assertIn("detach", commands)
        self.assertLess(
            commands.index("monitor resume"), commands.index("detach")
        )

    def test_throttle_is_clamped(self) -> None:
        self.assertRegex(self.source, r"#define MOTOR_MAX_PCT \d+U")
        self.assertIn("clamp_pct", self.source)


class BootupMotorSafetyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.source = BOOTUP_WORKLOAD.read_text(encoding="utf-8")

    def test_all_motors_are_parked_before_main(self) -> None:
        self.assertIn(
            "SYS_INIT(park_motors_before_main, APPLICATION, 0);",
            self.source,
        )
        init_body = re.search(
            r"static int park_motors_before_main\(void\)\s*\{(.*?)\n\}",
            self.source,
            re.DOTALL,
        )
        self.assertIsNotNone(init_body)
        self.assertIn(
            "pwm_set_dt(&motors[i], MOTOR_PERIOD_NSEC, 0)",
            init_body.group(1),
        )
        self.assertIn(
            "bootup_motor_early_park_mask = parked",
            init_body.group(1),
        )

    def test_stage_refuses_sweep_if_early_park_failed(self) -> None:
        self.assertRegex(
            self.source,
            r"bootup_motor_early_park_mask != MOTOR_ALL_PARKED_MASK",
        )


class ElaboratedShellTests(unittest.TestCase):
    """What the generator actually produced, when it has been run."""

    @classmethod
    def setUpClass(cls) -> None:
        if not GENERATED.is_dir():
            raise unittest.SkipTest(
                f"{MOTOR_CONFIG} has not been elaborated; run "
                f"./rb chipyard elaborate --board arty200t --config {MOTOR_CONFIG}"
            )

    def test_xdc_constrains_every_motor_pin(self) -> None:
        xdc = next(GENERATED.glob("*.shell.xdc")).read_text(encoding="utf-8")
        for motor, (_, _, pin, _) in MOTORS.items():
            self.assertIn(
                f"set_property PACKAGE_PIN {{{pin}}} [get_ports {{{motor}}}]", xdc
            )
            self.assertIn(
                f"set_property IOSTANDARD {{LVCMOS33}} [get_ports {{{motor}}}]", xdc
            )
            self.assertIn(
                f"set_property PULLDOWN {{TRUE}} [get_ports {{{motor}}}]", xdc
            )

    def test_harness_inverts_each_comparator_at_the_top_level(self) -> None:
        harness = (GENERATED / "gen-collateral" / "Arty200THarness.sv").read_text(
            encoding="utf-8"
        )
        for motor, (block, cmp_index, _, _) in MOTORS.items():
            self.assertIn(
                f"assign {motor} = ~_chiptop0_pwm_{block}_gpio_{cmp_index};",
                harness,
                f"{motor} is not the inverted comparator output",
            )

    def test_the_pwm_output_path_survived_elaboration(self) -> None:
        """Without an IOBinder firtool deletes it, and nothing else complains.

        A shell with PeripheryPWMKey set but no binder still elaborates, still
        maps the registers, and still reads back everything software writes --
        it simply has no gpio port anywhere. This is that check.
        """
        timer = (GENERATED / "gen-collateral" / "PWMTimer.sv").read_text(
            encoding="utf-8"
        )
        header = timer.split("module PWMTimer(", 1)[1].split(");", 1)[0]
        for channel in range(4):
            self.assertIn(f"io_gpio_{channel}", header)

    def test_pwm_interrupts_are_where_the_overlay_says(self) -> None:
        """The PLIC numbering the drone overlay hardcodes.

        Adding or dropping a controller shifts every source above it, which is
        how this bench lost its console UART once already.
        """
        dts = next(GENERATED.glob("*.dts")).read_text(encoding="utf-8")
        for address, expected in (
            ("10050000", "2 3 4 5"),
            ("10051000", "6 7 8 9"),
        ):
            self.assertRegex(
                dts,
                rf"(?s)pwm@{address}\s*\{{.*?interrupts = <{expected}>;",
                f"pwm@{address} is not on PLIC sources {expected}",
            )


class WaveformTests(unittest.TestCase):
    """Run the RTL and measure the pin. See hardware/pwm/tb_motor_pwm.cpp."""

    @classmethod
    def setUpClass(cls) -> None:
        verilator = ROOT / "backends/chipyard/.conda-env/bin/verilator"
        timer = GENERATED / "gen-collateral" / "PWMTimer.sv"
        if not verilator.is_file() or not timer.is_file():
            raise unittest.SkipTest("verilator or the elaborated PWMTimer is missing")
        cls.result = subprocess.run(
            [str(ROOT / "hardware/pwm/run.sh"), MOTOR_CONFIG],
            capture_output=True,
            text=True,
            cwd=ROOT,
            timeout=900,
        )

    def test_the_measured_waveform_matches_the_request(self) -> None:
        self.assertEqual(
            self.result.returncode, 0,
            f"{self.result.stdout}\n{self.result.stderr}",
        )
        self.assertIn("OK: 0 failure(s)", self.result.stdout)
        self.assertNotIn("FAIL", self.result.stdout)
