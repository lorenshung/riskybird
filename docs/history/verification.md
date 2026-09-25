# Verification record — 2026-07-24 through 2026-07-26

> **Historical record (2026-07-24 to 2026-07-26), not instructions.** Measured on
> `main`, before `setup` existed in its current form. Superseded by "Checking the tooling itself"
> in [`../setup.md`](../setup.md). Known-wrong or stale here: every test count and per-module
> pass/fail below, and the note that `test_dse` and `test_rose` pass here (part of the suite
> needs data and backends that live only on `main`); the KU040 rows used the 32 KiB scratchpad map.


> **Running the suite.** `python3 -m unittest discover -s tests` needs the
> backends installed -- it exercises environment construction, the board and
> workload registries, and the Chipyard hook, all of which resolve real paths.
> On a bare checkout most of it fails for that reason alone; run it after
> setup.md step 2, and after step 4 for the Chipyard-dependent modules.
>
> On the `setup` branch, with both backends present, `test_rb`, `test_soc`,
> `test_timing`, `test_pwm`, `test_dse` and `test_rose` pass. `test_xpurt` and
> `test_area` need profile data and Vivado reports that live on `main`, and one
> `test_fingerprint` case wants a FireSim descriptor that is not carried here.
> The counts quoted on this page were measured on `main`.


The 2026-07-24 sections retain the original OSPI-only evidence. The
2026-07-25 section records the expanded ESP and DroneLogic workload contract.

Results are recorded as measured on those dates and are not restated when the
tree moves. In particular every KU040 row below was taken against the 32 KiB
scratchpad memory map; KU040 configs have since become DDR4-backed.

## Chipyard and Vivado

| Command/check | Result |
| --- | --- |
| `./rb chipyard build --board arty100t` | PASS: Chipyard elaboration, FIRRTL/SystemVerilog generation, Vivado synthesis, placement, routing, timing analysis, and bitstream generation |
| Arty100T route status | PASS: 0 failed, unrouted, partially routed, or overlapping nets |
| Arty100T timing | PASS: WNS `+0.185 ns`, WHS `+0.036 ns`, 0 setup/hold failing endpoints |
| Arty100T DRC | Completed with 155 warnings and no error-severity violations; warnings include inherited shell IOB constraints, missing CFGBVS/CONFIG_VOLTAGE, and I/O/DSP recommendations |
| Arty100T artifact | `backends/chipyard/fpga/generated-src/chipyard.fpga.arty100t.Arty100THarness.RocketArty100TOspiConfig/obj/Arty100THarness.bit` |
| Arty100T bitstream | 3,825,900 bytes; SHA-256 `4217bd9c46cf2359fdbeb28f8a4c9c439be57bdad4d6508467052333635aca80` |
| `./rb chipyard build --board ku040` | PASS outside the managed sandbox: Chipyard elaboration, RTL generation, Vivado synthesis, placement, routing, timing analysis, and bitstream generation |
| KU040 route status | PASS: 0 failed, unrouted, partially routed, or overlapping nets |
| KU040 timing | PASS: WNS `+5.619 ns`, WHS `+0.030 ns`, 0 setup/hold failing endpoints |
| KU040 DRC | Completed with 147 warning-severity findings and no error-severity violations; warnings are missing CFGBVS/CONFIG_VOLTAGE, DSP pipeline recommendations, and inherited broad UART IOB constraints |
| KU040 utilization | 38,908 LUTs, 17,514 FFs, 57 RAMB36, 80 RAMB18, and 25 DSP blocks; 97 of 600 equivalent BlockRAM tiles (16.17%) |
| KU040 artifact | `backends/chipyard/fpga/generated-src/chipyard.fpga.ku040.KU040Harness.RocketKU040OspiConfig/obj/KU040Harness.bit` |
| KU040 bitstream | 16,007,024 bytes; SHA-256 `bfcb9d3ad4b254ab84cfcc540fe0368ee49a5a3f4130e31045fe094b9b2d2693` |
| `./rb chipyard elaborate --board ku040 --config SaturnKU040OspiConfig` | PASS after installing Chipyard's pinned Espresso minimizer: config override reached Make/Chisel, FIRRTL and split SystemVerilog were generated, and the config-specific DTS passed the KU040 OSPI contract |
| Saturn KU040 DTS | PASS: RV64 vector ISA with `zvl256b`, 32 KiB RAM at `0x80000000`, UART IRQ 2, I2C IRQ 1, and OSPI IRQ 3 |
| Noninteractive SBT launch through `rb` environment | PASS: exit 0 when the sandbox rejects SBT's Unix-domain boot socket |
| Generated DTS contract validation | PASS on both boards: UART `0x10020000`/IRQ 2, I2C `0x10040000`/IRQ 1, and board memory ranges |

The same KU040 command cannot reach the external Xilinx license service from
the managed sandbox. Running it with network access checks out both Synthesis
and Implementation licenses and completes successfully; no RTL change was
needed.

No FPGA was programmed during this verification. Hardware-manager connection,
UARTTSI loading, serial I/O, sensor I/O, and OpenOCD/GDB remain hardware tests.

## Zephyr clean-build matrix

Every row below was rebuilt with `--pristine always` through `./rb build`.
These are linked RAM-image sizes, not host executable sizes.

| Board | Workload | Mode | Linked RAM | Board RAM | Result |
| --- | --- | --- | ---: | ---: | --- |
| Arty100T | state | replay | 19,168 B | 1 GiB | PASS |
| Arty100T | state | BMI088 | 23,824 B | 1 GiB | PASS |
| Arty100T | MPC | replay | 38,160 B | 1 GiB | PASS |
| Arty100T | MPC | HIL | 34,320 B | 1 GiB | PASS |
| KU040 | state | replay | 19,168 B | 32 KiB | PASS, 58.50% |
| KU040 | state | BMI088 | 23,824 B | 32 KiB | PASS, 72.71% |
| KU040 | MPC | replay | 32,000 B | 32 KiB | PASS, 97.66% |
| KU040 | MPC | HIL | 28,160 B | 32 KiB | PASS, 85.94% |

The KU040 MPC replay image has 768 bytes of link-time headroom — not a
comfortable fit. All rows retain warnings inherited from the backend
board/source (inactive `PRIVILEGED_STACK_SIZE`, disabled HTIF vendor prefix,
leading-zero memory unit address, existing compiler warnings); none stopped
configuration or linking.

Each image has a real RISC-V load segment spanning the reported footprint, so
these are not empty wrappers: the BMI088 state ELF carries `main`,
`rb_madgwick_update_imu`, `rb_madgwick_euler`, `rb_imu_init`, `rb_imu_read` and
the Zephyr BMI08x driver symbols; the MPC HIL ELF carries `main`, `tiny_init`,
`tiny_solve`, and `update_linear_cost`.

### Why the backend hello-world sample does not fit

A clean KU040 build of Zephyr's backend `samples/hello_world` with the FPGA
overlay and common configuration fails by 2,272 bytes: its map ends at 35,040
bytes for a 32,768-byte region.

| Setting/section | Backend hello world | RiskyBird workloads |
| --- | ---: | ---: |
| `CONFIG_NUM_IRQS` | 1,036 | 16 at the time of this measurement |
| software ISR table | 16,576 B (`0x40c0`) | 256 B at the time of this measurement |
| `CONFIG_MAIN_STACK_SIZE` | 4,096 B | 2,048 B |

The generic backend board models a QEMU-sized PLIC. The OSPI shells used in
this measurement exposed sources 1 through 3, so the first RiskyBird default
used 16 table entries. The later DroneLogic shell exposes sources 1 through
15. The shared default is now 27 entries: 12 primary RISC-V slots plus 15
second-level PLIC sources. This remains far smaller than the backend's 1,036
entries while covering both shell variants.

## Standalone OSPI/HM01B0 RTL

| Check | Result |
| --- | --- |
| `cd hardware/ospi && ./mill -i ospi.compile` | PASS |
| `cd hardware/ospi && ./mill -i ospi.test` | PASS: 17 tests, 5 suites |
| Six shared RTL files compared with Chipyard copies using `cmp` | PASS: all byte-identical |

## Top-level CLI tests

| Check | Result |
| --- | --- |
| `./rb doctor` | PASS: both environments, west, compiler, Vivado, OpenOCD, GDB, and both DTS contracts found; no serial devices attached |
| `./rb --help` | PASS |
| `python3 -m unittest discover -s tests -v` | PASS: 69 tests |
| Physical FPGA board registry | Arty100T and custom KU040 |
| Zephyr target registry | Arty100T, custom KU040, and ESP32-C6 |
| Zephyr environment selection | PASS: clean setup defaults to `zephyr`, an existing `zephyr` prefix is accepted, and an existing `zephyr-new` prefix is preferred |
| Supported-workload registry | state, MPC, and 12 sensor/actuator bring-up applications |
| Environment isolation checks | Zephyr/Chipyard prefixes, repository-local SBT runtime, relocated OpenOCD loader, and SBT socket fallback |

## ESP and DroneLogic integration — 2026-07-25

### Generated FPGA hardware contract

Both new Rocket DroneLogic configurations elaborated successfully. Their
generated DTS files were checked by `rb` and by the unit tests:

| Config | Derived contract | Result |
| --- | --- | --- |
| `RocketArty100TDroneLogicConfig` | I2C IRQ 1; PWM IRQs 2–9; UART IRQ 10; GPIO IRQs 11–13; SPI IRQ 14; PLIC `ndev=15` | PASS |
| `RocketKU040DroneLogicConfig` | Same MMIO/interrupt topology; 32 KiB memory contract | PASS |

The logical MMIO addresses are PWM0 `0x10050000`, PWM1 `0x10051000`, UART
`0x10020000`, GPIO `0x10010000`, SPI `0x10031000`, and I2C `0x10040000`.

This is elaboration evidence, not synthesis or physical-I/O evidence. No
DroneLogic bitstream was built. SPI is tied off by the current harness and
GPIO/PWM/SPI package pins and voltage constraints have not been assigned.

### Representative Zephyr builds

All rows below were linked through `./rb build`. FPGA motor and PMW3901 rows
used the matching Rocket DroneLogic config.

| Target | Workload/mode | Linked memory | Result |
| --- | --- | ---: | --- |
| ESP32-C6 | `pmw3901_test` | 50,864 B SRAM; 134,624 B flash | PASS |
| ESP32-C6 | `bmi088_test` | not recorded | PASS |
| ESP32-C6 | `vl35l5cx_test/address` | 54,304 B SRAM; 134,344 B flash | PASS |
| ESP32-C6 | `vl35l5cx_test/ranging` | 71,648 B SRAM; 223,600 B flash | PASS |
| ESP32-C6 | `sensor_bringup` | 53,904 B SRAM; 139,552 B flash | PASS |
| Arty100T | `motor_4` | 21,472 B | PASS |
| Arty100T | `pmw3901_test` | 25,952 B | PASS |
| Arty100T | `vl35l5cx_test/address` | 23,264 B | PASS |
| Arty100T | `vl35l5cx_test/ranging` | 140,016 B | PASS |
| KU040 | `ads7128_test` | 23,264 B of 32 KiB | PASS, 71.00% |
| KU040 | `bmi088_test` | 24,816 B of 32 KiB | PASS, 75.73% |
| KU040 | `motor_4` | 21,472 B of 32 KiB | PASS, 65.53% |
| KU040 | `pmw3901_test` | 25,952 B of 32 KiB | PASS, 79.20% |
| KU040 | `vl35l5cx_test/address` | 22,544 B of 32 KiB | PASS, 68.80% |

The four-ToF `address` mode compiles the ADS7128 XSHUT sequence and sensor
identity path using public I2C. The `ranging` mode compiles the same stable
RiskyBird API against the pinned Zephyr VL53L5CX grid driver. Its Arty image
uses 140,016 bytes, so the registry enforces a conservative 192 KiB
addressable-RAM minimum for FPGA ranging. KU040 ranging is rejected before
West because the current SoC exposes only 32 KiB; the custom board's declared
2 GiB DDR4 is not yet part of that hardware or Zephyr memory contract.

The legacy `i2c_expander` VL53L0X image failed the KU040 link by 28,288 bytes
and is therefore enabled only for ESP32-C6 and Arty100T. It is not the
four-ToF path; the portable four-VL53L5CX address/XSHUT workload fits KU040.

No board was attached during this verification. ESP flashing, current serial
capture, FPGA programming, sensor traffic, PWM waveforms, motor response, and
all custom KU040 connector behavior remain physical tests.

## Shared ESP base-board contract — 2026-07-26

The ESP target now owns the common drone-PCB I2C wiring and BMI088 device
contract. Workload overlays add only workload-specific devices, rates, or SPI
wiring. The declared BMI088 addresses are accelerometer `0x18` and gyroscope
`0x68`; the FPGA overlays use the same addresses because the same base board is
intended to connect to all three targets. This address remains derived from the
previously deployed ESP source rather than a new measurement.

Clean `--pristine always` builds passed for ESP32-C6 `i2c_scanner`,
`bmi088_test`, `state/bmi088`, `pmw3901_test`, `ads7128_test`,
`vl35l5cx_test/address`, and `motor_4`, plus KU040 `bmi088_test`. The generated
ESP PMW3901 DTS has both the RiskyBird I2C controller and SPI2 enabled, and the
generated ESP and KU040 DTS files both place the BMI088 gyro at `0x68`.

The historical `sensor_bringup` now compiles for ESP against a public
VL53L1X reinitialization header. The four-ToF `address` and `ranging` modes
compile against the stable RiskyBird ToF contract. These changes establish
software integration only: no attached sensor was available to validate the
XSHUT sequence, volatile addresses, ULD firmware transfer, data-ready timing,
or range values.

The custom KU040 board has 2 GiB of populated DDR4 by board-owner declaration.
The current KU040 SoC configurations still expose only their implemented
32 KiB memory window in generated hardware and Zephyr DTS. The CLI reports both
facts separately; no DDR controller, pin, calibration, timing, or physical
memory-access result is claimed.

Validation for this correction was build-only: 69 unit tests and `./rb doctor`
passed, and `git diff --check` was clean. No serial device was attached.

## FPGA area characterization — 2026-07-26

First execution of the `rb area` flow. Three synthesis runs of the same config on
the same part, differing only in RTL attribute injection. Vivado v.2023.1,
`xcku040-sfva784-1-c`, `flow=synth-only`, `evidence=synthesized`.

| Command | Result |
| --- | --- |
| `rb area run --board ku040 --attrs none` | PASS: synthesis, hierarchical and flat utilization, provenance manifest, record |
| `rb area run --board ku040 --attrs hierarchy --force` | PASS: `keep_hierarchy` applied to `ChipTop.sv` and `RocketTile.sv`; 6 targets correctly reported `file-absent` |
| `rb area run --board ku040 --attrs dsp --force` | PASS: `use_dsp` applied to all three `MulAddRecFNPipe_*.sv`; `MacUnit.sv` and `OuterProductCell.sv` correctly reported `file-absent` |
| `rb area restore --board ku040` | PASS both times: every injected file restored from its `.rb-orig`, zero backups and zero injected attributes left |
| `rb area report` | PASS: the three runs partitioned into separate comparison groups by attribute mode, with the upper-bound note on the injected group |
| `rb area plot` | PASS: 12 well-formed SVGs plus `summary.csv`, percentages taken from Vivado's own `Available` column |

### `RocketKU040OspiConfig` device totals

| Resource | `attrs=none` | `attrs=dsp` | Δ | `attrs=hierarchy` | Δ |
| --- | ---: | ---: | ---: | ---: | ---: |
| CLB LUTs | 39,289 | 39,064 | −225 (−0.57%) | 40,235 | +946 (+2.41%) |
| Logic LUTs | 36,984 | 36,759 | −225 (−0.61%) | 37,758 | +774 (+2.09%) |
| LUT as Memory | 2,305 | 2,305 | 0 | 2,477 | +172 (+7.46%) |
| CLB Registers | 17,536 | 17,519 | −17 (−0.10%) | 18,956 | +1,420 (+8.10%) |
| Block RAM Tile | 97 | 97 | 0 | 97 | 0 |
| DSPs | 25 | **30** | **+5 (+20.0%)** | 25 | 0 |

Device capacity as reported by Vivado: 242,400 LUTs, 484,800 registers, 600 BRAM
tiles, 1,920 DSPs. `attrs=none` uses 16.21% of LUTs, 3.62% of registers, 16.17%
of BRAM tiles, and 1.30% of DSPs.

### Findings

- **`keep_hierarchy` inflates totals, confirming the recorded upper-bound
  caveat.** +2.41% LUTs and +8.10% registers, with hard macros unchanged. The
  upstream flow applies this attribute unconditionally and does not record the
  effect, so its totals are systematically high by about this much.
- **`keep_hierarchy` bought little attribution here.** The annotated
  `RocketTile` moved +0.29% while the design total moved +2.41%, so the
  inflation is mostly outside the annotated boundary. Without an accelerator to
  separate from its host, `attrs=none` is the better default.
- **`use_dsp` on hardfloat FMAs is not a no-op here.** Five additional DSP48E2
  slices absorbed 225 LUTs and 17 registers. The upstream comment recording this
  injection as a no-op was measured on `V128D128 robotMpc` and does not
  generalize. Spending 5 of 1,895 idle DSPs to recover 225 LUTs is favourable on
  a part where LUTs are the scarce resource.
- **Synthesis and post-route totals differ systematically.** 39,289 LUTs
  post-synthesis against 38,908 post-route for the same config: +0.98%. This is
  why `flow` is part of a record's identity.

### Defect found and fixed by running the flow

Vivado labels the LUT total row `CLB LUTs*` and the RAMB36 row `RAMB36/FIFO*`,
with footnote asterisks that appear depending on design state. The alias table
matched only the unmarked spellings, so `record.luts` was `None`: `rb area
report` printed `-` for LUTs and a LUT chart would have raised. `RAMB36/FIFO*`
fell through to the nested `RAMB36E2 only` row, which carries no device capacity.
The fixture used a guessed label, so the tests passed while the real path was
broken.

Fixed by making alias matching ignore footnote markers while `totals_raw` stays
faithful to Vivado's exact labels. A regression test built from the real report
text now covers both rows. Because `totals_raw` preserved every row verbatim,
the two affected records were re-normalized without re-synthesizing.

### Integrity

The `RocketKU040OspiConfig` build directory was reused, so injection wrote into
its `gen-collateral`. After both restores the tree carries zero injected
attributes and zero `.rb-orig` backups, and the previously verified bitstream is
byte-identical: SHA-256 `bfcb9d3ad4b254ab84cfcc540fe0368ee49a5a3f4130e31045fe094b9b2d2693`,
unchanged from the Chipyard section above.

`PowerOnResetFPGAOnly.v` carries a `keep_hierarchy` attribute in generated
collateral. This is pre-existing fpga-shells output from a Chisel `BlackBox`
`setInline`, present in the Arty100T and `SaturnKU040OspiConfig` collateral that
this flow never touched, and is not injection residue.

### Not established

No Gemmini or Saturn OPU configuration was measured, so the two largest claimed
injection wins — roughly 31K LUTs for a 16×16 Gemmini `MacUnit` mesh and 43K for
a 256-cell `OuterProductCell` mesh — remain untested. Both require submodule
revisions that have not been adopted. No FP-stripping variant was measured for
the same reason. No place-and-route, timing, or power result was produced by this
flow.
