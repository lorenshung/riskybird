# HM01B0 capture + I2C FPGA integration

This is the Chipyard-side build and implementation record. The detailed architecture, register
map, software sequence, and electrical notes are in
`hardware/ospi/docs/CHIPYARD_INTEGRATION.md`.

## Implemented

- Shared capture RTL: `generators/chipyard/src/main/scala/ospi/`
- TileLink register wrapper and PLIC source: `ospi/OspiChipyard.scala`
- SoC attachment: `DigitalTop.scala`
- Chip I/O: `iobinders/Ports.scala` and `iobinders/IOBinders.scala`
- Arty100T I2C/camera harness binders and opt-in config
- KU040 I2C/camera harness binders and Rocket/Saturn opt-in configs
- Synchronously reset elastic isolation between the async CDC FIFO and BRAM controls
- Complete-frame BRAM queue (`104977 x 11`) between capture CDC and software

The base `RocketArty100TConfig`, `RocketKU040Config`, and `SaturnKU040Config` do not contain camera
hardware. Select `RocketArty100TOspiConfig`, `RocketKU040OspiConfig`, or
`SaturnKU040OspiConfig` to add OSPI plus the I2C controller used to configure the sensor.

## Reproduce verification

Standalone RTL tests:

```bash
cd hardware/ospi
./mill -i ospi.compile
./mill -i ospi.test
```

Chipyard compile/elaboration, from `backends/chipyard` after repository initialization:

```bash
source env.sh

make -C fpga SUB_PROJECT=arty100t launch-sbt \
  SBT_COMMAND=';project chipyard_fpga; compile'

make -C fpga SUB_PROJECT=arty100t CONFIG=RocketArty100TOspiConfig firrtl
make -C fpga SUB_PROJECT=ku040   CONFIG=RocketKU040OspiConfig firrtl
```

Full Vivado builds:

```bash
make -C fpga SUB_PROJECT=arty100t CONFIG=RocketArty100TOspiConfig bitstream
make -C fpga SUB_PROJECT=ku040   CONFIG=RocketKU040OspiConfig bitstream
```

Expected output paths:

```text
backends/chipyard/fpga/generated-src/chipyard.fpga.arty100t.Arty100THarness.RocketArty100TOspiConfig/obj/Arty100THarness.bit
backends/chipyard/fpga/generated-src/chipyard.fpga.ku040.KU040Harness.RocketKU040OspiConfig/obj/KU040Harness.bit
```

## Verified results (2026-07-24)

| Check | Result |
|---|---|
| Standalone Mill suite | PASS — 17 tests, 5 suites |
| `chipyard_fpga` Scala compile | PASS |
| Arty100T OSPI FIRRTL/device tree/XDC | PASS — OSPI at `0x10080000`, PLIC source 3 |
| KU040 OSPI FIRRTL/device tree/XDC | PASS — OSPI at `0x10080000`, PLIC source 3 |
| Base-config optionality | PASS — Rocket Arty100T/KU040 elaborate without OSPI, HM01B0, or I2C collateral |
| Arty100T BRAM inference | PASS — frame queue inferred as `104977 x 11` block RAM |
| Arty100T OSPI bitstream | PASS — fully routed, 0 unrouted nets, WNS `+0.185 ns`, WHS `+0.036 ns`, bitgen complete |
| Arty100T async-BRAM DRC | PASS — the earlier `REQP-1839` violation is absent after adding the elastic stage |
| KU040 static/package review | PASS — all camera/I2C pins are unique SFVA784 bank-68 user I/O; PCLK C3 is GC |
| KU040 OSPI bitstream | PASS outside the managed sandbox — fully routed, 0 unrouted nets, WNS `+5.619 ns`, WHS `+0.030 ns`, bitgen complete |
| KU040 DRC | PASS with 147 warning-severity findings and no error-severity violations |
| KU040 BRAM inference | PASS — frame queue remains inferred as block RAM; complete design uses 97 of 600 equivalent BlockRAM tiles |

The latest final Arty artifact is
`backends/chipyard/fpga/generated-src/chipyard.fpga.arty100t.Arty100THarness.RocketArty100TOspiConfig/obj/Arty100THarness.bit`
(SHA-256 `4217bd9c46cf2359fdbeb28f8a4c9c439be57bdad4d6508467052333635aca80`).

The latest final KU040 artifact is
`backends/chipyard/fpga/generated-src/chipyard.fpga.ku040.KU040Harness.RocketKU040OspiConfig/obj/KU040Harness.bit`
(16,007,024 bytes; SHA-256
`bfcb9d3ad4b254ab84cfcc540fe0368ee49a5a3f4130e31045fe094b9b2d2693`).

The managed sandbox cannot reach the external Xilinx license service. The identical `rb` command
outside that network boundary checks out Synthesis and Implementation licenses and completes; no
RTL change or alternate build command is required.

The shell helper's port-level `IOB=TRUE` query follows fan-in/fan-out past the intended boundary
register — on PCLK it tagged the clock domain; on FLVD and data, multiple clock-domain endpoints
and inferred FIFO memory. The OSPI binder therefore emits no broad IOB hints, retaining only
package, I/O-standard, clock, asynchronous-group, and (on Arty) dedicated-route constraints. If
I/O-cell placement is needed later, instantiate explicit registers in the harness.

## Build caveats

The Makefile contains two pre-existing optional-copy recipes that can print `Error 1 (ignored)`
when no custom overlay exists. They are not build failures. Evaluate the final Vivado exit status,
the `.bit` file, DRC, utilization, and timing reports.

The current Arty and KU040 pin maps are package-legal build defaults, not proof of physical
camera-connector routing. Do not attach the HM01B0 until the carrier schematic and voltage domains
have been checked. Arty's `LVCMOS33` interface requires level shifting because HM01B0 IOVDD is at
most 3.0 V; KU040 bank 68 uses `LVCMOS18`.
