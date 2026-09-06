# Zephyr target contracts

Zephyr application source is board-independent. A target supplies the
physical or logical buses and signals through devicetree overlays:

```text
workloads/<name>/                     application and execution modes
hardware/zephyr/targets/esp32c6/      ESP drone PCB wiring
  target.overlay                     shared I2C and BMI088 contract
  target.conf                        architecture-specific Kconfig
hardware/zephyr/targets/fpga/         common logical FPGA workload wiring
hardware/zephyr/targets/firesim/      FireSim hart count and HTIF console
hardware/zephyr/fpga-drone.overlay    generated FPGA MMIO/interrupt contract
hardware/zephyr/{arty100t,ku040}.overlay
                                      board memory contract
```

## ESP32-C6 drone PCB

The ESP target overlay preserves the base-board wiring from the previously
deployed drone sources. Workload overlays only add devices or operating
settings specific to that workload:

| Function | ESP32-C6 connection |
| --- | --- |
| I2C0 SCL / SDA | GPIO14 / GPIO15 |
| PMW3901 SCLK / MOSI / MISO | GPIO6 / GPIO7 / GPIO18 |
| PMW3901 software CS / reset / LED_N | GPIO19 / GPIO2 / GPIO3 |
| motor 1 / 2 / 3 / 4 PWM | GPIO21 / GPIO20 / GPIO23 / GPIO22 |
| four VL53L5CX XSHUT controls | ADS7128 GPIO1 / GPIO2 / GPIO3 / GPIO4 |

The four ToF sensors share I2C with the ADS7128 at `0x17`. They start at
`0x29` and are assigned `0x31`–`0x34` at each boot. The one ESP base-board
contract defines the BMI088 accelerometer at `0x18` and gyroscope at `0x68`.
The FPGA overlays use those same sensor addresses because the intent is to
connect the same base board.

The board-independent sequencing and grid contract lives in
`integration/zephyr/tof/`. Address/identity mode builds for all targets.
Full VL53L5CX ranging needs at least 192 KiB addressable RAM on an FPGA target,
which the DDR4-backed KU040 configs supply. Only the retained 32 KiB scratchpad
variants are too small for it.

This table is a declared PCB wiring contract derived from the previously
deployed BMI088 overlay. A new I2C scan and identity capture in this repository
is still required to confirm the physical address straps.

## FPGA DroneLogic SoC

The following values are derived from elaborated
`RocketArty100TDroneLogicConfig` and `RocketKU040DroneLogicConfig` DTS output:

| Peripheral | MMIO address | PLIC sources |
| --- | ---: | --- |
| I2C | `0x10040000` | 1 |
| PWM block 0 | `0x10050000` | 2–5 |
| PWM block 1 | `0x10051000` | 6–9 |
| UART | `0x10020000` | 10 |
| 3-bit GPIO | `0x10010000` | 11–13 |
| SPI | `0x10031000` | 14 |
| OSPI | generated shell address | 15 |

Zephyr consequently uses 27 interrupt slots: 12 primary RISC-V slots followed
by 15 second-level PLIC sources.

The shared workload mapping is:

| Function | Logical FPGA mapping |
| --- | --- |
| PMW3901 CS / reset / LED_N | GPIO indices 0 / 1 / 2 |
| PMW3901 data bus | SPI0 |
| motors 1–3 | PWM0 channels 1–3 |
| motor 4 | PWM1 channel 1 |
| ADS7128, BMI088, and ToF sensors | I2C0 |

This is not a verified FPGA connector map. The current harness ties SPI
inputs off and does not expose GPIO/PWM/SPI through physical package pins.
The next FPGA hardware step is to add explicit I/O binders and XDC constraints
for each board, then elaborate, synthesize, route, inspect DRC/timing, and
bench-test the pins before enabling `rb run`.

For KU040, use only constraints proven for the custom
`XCKU040-SFVA784-1-C` board. Connector constraints from commercial KU040
boards with different packages are not interchangeable. The known candidate
bank is a 1.8 V domain, but the base-connector/package-pin mapping is still
unknown and no constraint is assigned here.

The custom KU040 board has 2 GiB of populated DDR4. The Rocket, Saturn, and
accelerator KU040 configurations instantiate both 1 GiB controllers (HP banks 44
and 46) as one contiguous region and expose it in their generated DTS. The
32 KiB scratchpad variants are retained only to keep that memory map buildable
for area comparisons.
