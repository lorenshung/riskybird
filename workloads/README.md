# RiskyBird workloads

This directory owns board-independent Zephyr applications. Target-specific
pins, buses, addresses, and peripheral instances live under
`hardware/zephyr/targets/`; `tools/rb/boards.py` is the machine-readable
workload/target registry.

The sensor and actuator diagnostics were imported from
`backends/zephyr-chipyard-sw` remote ref `origin/riskybird-bringup` at commit
`2bcbd635bffdb9dba6596aaf326e0cc2be3d8ff6`. They are now top-level
applications and are selected with the same `rb build` and `rb run` interface
as the state-estimation and MPC workloads.

## What is available

| Workload | Function | Targets | Current qualification |
| --- | --- | --- | --- |
| `state` | Madgwick estimator with deterministic replay or BMI088 input | all four | Software builds |
| `mpc` | TinyMPC deterministic replay or binary UART HIL | all four | Software builds |
| `ads7128_test` | ADS7128 identity, register, and GPIO diagnostic | all four | Software builds |
| `bmi088_test` | BMI088 accelerometer, gyroscope, and temperature stream | all four | Software builds |
| `i2c_scanner` | I2C address scan | all four | Software builds |
| `i2c_expander` | Legacy single-VL53L0X test using ADS7128 GPIO6 | ESP, Arty100T | Too large for a 32 KiB scratchpad config |
| `tof_sensor` | Legacy single-VL53L0X ranging test | ESP, Arty100T | Too large for a 32 KiB scratchpad config |
| `vl35l5cx_test` | Four VL53L5CX sensors using ADS7128 GPIO1–GPIO4 | all four | `address` everywhere; `ranging` needs ≥192 KiB, so not on a scratchpad config |
| `pmw3901_test` | PMW3901 SPI optical-flow stream | all four | ESP and FPGA software builds; FPGA physical I/O pending |
| `motor_1` … `motor_4` | Explicitly armed 20 kHz PWM diagnostics | all four | ESP and FPGA software builds; FPGA physical I/O pending |
| `sensor_bringup` | Imported PMW3901 plus one VL53L1X integration test | ESP | Legacy application builds; retained for comparison rather than as the combined-sensor architecture |

“Software builds” means Zephyr configured and linked the image — not a
hardware result. These applications ran on the ESP drone before this
integration, but no serial capture from that hardware is recorded here.

The directory name `vl35l5cx_test` is retained for continuity with the source
branch; the sensor implemented by the workload is the VL53L5CX.

The pinned Zephyr tree now contains the public VL53L1X reinitialization API
needed by `sensor_bringup` and the VL53L5CX grid API used by full ranging.
Those backend additions are kept at the Zephyr driver boundary; RiskyBird
workloads consume the stable top-level contract under
`integration/zephyr/tof/`.

## Build and run

Install the ESP and ST HAL projects required by the additional targets once:

```console
./rb zephyr setup-targets
```

Discover the registered interface and build individual diagnostics:

```console
./rb targets
./rb workloads

./rb build i2c_scanner --target esp32c6
./rb build ads7128_test --target esp32c6
./rb build bmi088_test --target esp32c6
./rb build pmw3901_test --target esp32c6
./rb build vl35l5cx_test --target esp32c6 --mode address
```

With the ESP drone connected, `run` builds, flashes, and opens the console:

```console
./rb run bmi088_test --target esp32c6 --port /dev/ttyACM0
./rb run vl35l5cx_test --target esp32c6 --mode address \
  --port /dev/ttyACM0 --capture-duration 30
```

`rb capture` can record an already-running image. It stores the serial stream,
operator input, ELF hash, target/workload/mode, and repository revisions in a
new results directory:

```console
./rb capture bmi088_test --target esp32c6 --port /dev/ttyACM0 \
  --duration 30
```

FPGA I2C diagnostics use the current OSPI configurations:

```console
./rb build bmi088_test --target arty100t
./rb build vl35l5cx_test --target ku040 --mode address
```

FPGA SPI/PWM software requires the logical DroneLogic SoC:

```console
./rb build pmw3901_test --target arty100t \
  --config RocketArty100TDroneLogicConfig
./rb build motor_4 --target ku040 \
  --config RocketKU040DroneLogicConfig
```

Those last two commands validate software and the generated MMIO/interrupt
contract only. `rb run` intentionally refuses FPGA PMW3901 and motor
deployment until package pins, voltage banks, harness binders, constraints,
synthesis, and timing are verified for the selected physical board.

## Four-ToF sequence

The four sensors share the default VL53L5CX address `0x29`. The
`vl35l5cx_test` `address` mode:

1. configures ADS7128 GPIO1–GPIO4 as push-pull XSHUT outputs;
2. holds all four sensors in reset;
3. releases one additional sensor at a time;
4. changes its volatile address to `0x31`, `0x32`, `0x33`, or `0x34`; and
5. leaves earlier sensors awake while bringing up the next sensor.

Keeping earlier sensors awake is required because asserting XSHUT again would
discard their volatile address and return them to `0x29`. Address mode then
probes each device identity continuously without depending on a private
VL53L5CX Zephyr driver.

Full ranging is a separate `ranging` mode: same XSHUT and address sequence,
then the Zephyr VL53L5CX devices at their final addresses, emitting target
count, status, and signed millimeter distance per row-major zone. The linked
Arty image is ~137 KiB, so `rb` requires ≥192 KiB addressable RAM on an FPGA
target. Scratchpad-only configs take `address` mode but not `ranging`.

## Motor safety

All motor workloads force every PWM output to zero at boot. They require an
explicit `s` command before applying the default 10% diagnostic duty cycle;
`x`, space, or `q` stops and disarms the outputs. Remove propellers and verify
the electrical motor-driver interface before using these tests.

A four-motor connector normally exposes one PWM signal per motor, but that does
not mean an FPGA pin can drive the motor stage: ground reference, voltage
levels, polarity, pull state, ESC input requirements, and power sequencing must
all be verified against the base-board schematic.

The SiFive PWM peripheral has four comparators, but comparator 0 establishes
the period; usable output channels are 1–3. The logical FPGA design therefore
uses channels 1–3 of `pwm0` for motors 1–3 and channel 1 of `pwm1` for motor
4.

## Next combined workload

`sensor_bringup` is a historical one-off, not the base for state estimation.
The combined autonomy workload should use the same devicetree contracts as the
separate diagnostics, timestamp BMI088/PMW3901/four-ToF samples, report validity
per sensor, and emit a replayable machine-readable frame. Keep initialization
and health checks separate per sensor so a missing one cannot hide the others.
