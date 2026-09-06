# OSPI — HM01B0 camera capture

This is the FPGA-side receiver for the HM01B0's 8-bit parallel video interface. Despite the
historical `ospi` name, it is not an Octal-SPI controller: it samples `D[7:0]`, `PCLK`, `FLVD`, and
`LVLD`, crosses the pixel stream into the system clock domain, and preserves frame markers for
software. The physical HM01B0 remains an external I2C-configured sensor.

The directory is a small standalone Mill project for fast RTL verification. The same six shared
source files are kept in Chipyard under
`../../backends/chipyard/generators/chipyard/src/main/scala/ospi/`; Chipyard adds
`OspiChipyard.scala` for
TileLink MMIO, PLIC, buffering, configuration, and shell wiring. See
[`CHIPYARD_INTEGRATION.md`](CHIPYARD_INTEGRATION.md) for that integration.

## Run verification

From `hardware/ospi`:

```bash
./mill -i ospi.compile
./mill -i ospi.test
```

`-i` keeps Mill in interactive/no-server mode, which also works in environments where local
server sockets are unavailable. The suite currently contains 17 tests across five suites. It
checks pixel and marker ordering, frame reconstruction, geometry reporting, enable/clear and
sideband behavior, FIFO wraparound/backpressure/overflow, and complete-frame buffering/flush.

To emit standalone SystemVerilog, put Chipyard's `firtool` on `PATH`:

```bash
PATH=../../backends/chipyard/.conda-env/riscv-tools/bin:$PATH \
  ./mill -i ospi.runMain ospi.Elaborate
```

The output is `generated/HM01B0Capture.sv`. Generated output and Mill's `out/` directory are
ignored; neither is source collateral.

## Interface

The sensor-facing directions below are from the capture core's point of view:

| Signal | Direction | Meaning |
|---|---:|---|
| `pclk` | input clock | HM01B0 source-synchronous pixel clock |
| `fvld` | input | HM01B0 `FLVD` frame-valid signal; the bundle retains the old spelling |
| `lvld` | input | line valid |
| `d[7:0]` | input | 8-bit RAW pixel |
| `intr` | input | sensor motion interrupt |
| `mclk` | output clock | sensor master clock, `sysclk / (2 * (mclkDiv + 1))` |
| `trig` | output | one-cycle software trigger in snapshot mode |

I2C is deliberately separate. Chipyard's `TLI2C` controller configures the HM01B0 at its default
7-bit address `0x24`; one physical two-wire bus may also address other slaves.

## Datasheet conformance boundary

The implementation follows every explicit digital-interface requirement in the supplied
preliminary HM01B0 module datasheet that applies to the FPGA receiver:

| Datasheet requirement | Implementation |
|---|---|
| `D[7:0]`, `PCLK`, `FLVD`, `LVLD`, and `INT` are sensor outputs | FPGA inputs |
| `MCLK` and active-high `TRIG` are sensor inputs | FPGA outputs |
| 324x244 QVGA and 324x324 maximum active geometry | supported and checked |
| 3–36 MHz MCLK with 45–55% duty cycle | 3.125–25 MHz for legal divider values 0–7, 50% duty |
| PCLK maximum 36 MHz | constrained to 36 MHz |
| I2C maximum 400 kHz, default 7-bit address `0x24` | software-configurable general I2C master |
| module IOVDD is 1.8 V | KU040 uses LVCMOS18; Arty requires level translation |

That PDF does not define the PCLK sampling edge, output setup/hold or clock-to-output timing, a
minimum TRIG pulse width, or the complete sensor initialization register sequence. Consequently,
the RTL currently samples pixels on PCLK's rising edge, emits a one-system-clock TRIG pulse, and
leaves power sequencing and register initialization to board/software integration. Production
timing certification needs a complete timing reference or a hardware measurement; the preliminary
datasheet cannot establish it.

Each accepted pixel becomes a `PixelFlit(data, sof, eol, eof=false)`. A separate in-band EOF flit
follows the last pixel. Keeping EOF in the same FIFO prevents a frame-completion event from racing
the data it describes.

## Datapath

```text
D/FLVD/LVLD -- PCLK --> CaptureFrontend --> AsyncFifo --> frame measurement --> pixels
                                      CDC                 system clock
```

`CaptureFrontend` cannot backpressure the sensor. If the CDC FIFO fills, incoming beats are
dropped and a sticky overflow flag is set. The Chipyard wrapper therefore drains that FIFO
continuously into a block-RAM queue sized for one maximum 324x324 frame plus EOF. Vivado confirms
that the `104977 x 11` queue is inferred as block RAM.

The functional tests tie PCLK to the test clock. They exercise the complete logic and the
Gray-pointer FIFO state machine, but they do not replace FPGA CDC analysis or hardware testing
with independently running clocks.

## Source layout

```text
src/ospi/   parameters, sensor/control IO, capture frontend, top-level capture, frame buffer
src/infra/  Gray-pointer asynchronous FIFO and standalone Verilog elaborator
test/src/   five ScalaTest/chiseltest suites
docs/       integration state and runbook
```

The supported capture mode is currently the HM01B0 8-bit parallel output. QVGA is 324x244 in the
sensor's convention; the maximum active array is 324x324. DMA, packed-word reads, and a production
camera driver are follow-on work rather than hidden requirements of the RTL.
