# Chipyard integration — HM01B0 capture and I2C

## State of the integration

Implemented on Chipyard's `trenz` branch:

- a TileLink peripheral at `0x1008_0000`;
- autonomous PCLK-to-system-clock capture followed by a complete-frame block-RAM queue;
- nonblocking MMIO pixel reads, geometry/error status, and a level PLIC interrupt;
- the existing SiFive I2C controller at `0x1004_0000` for HM01B0 configuration;
- opt-in Rocket configs for the Arty100T and KU040 shells, plus a Saturn KU040 variant;
- shell binders and timing constraints for both boards.

Scala compilation, all 17 standalone tests, both FPGA elaborations, device-tree generation, and
block-RAM inference have been checked. The final Arty100T and KU040 implementations are fully
routed, meet timing, and produce bitstreams. Exact build outcomes are recorded in
`hardware/ospi/docs/OSPI_I2C_INTEGRATION.md`.

## Where the implementation lives

Shared RTL is present in both of these locations:

```text
hardware/ospi/src/{ospi,infra}/
backends/chipyard/generators/chipyard/src/main/scala/ospi/
```

The shared files are `Params.scala`, `CapturePort.scala`, `CaptureFrontend.scala`,
`AsyncFifo.scala`, `HM01B0Capture.scala`, and `FrameBuffer.scala`. Chipyard-only integration is in
`OspiChipyard.scala`, `DigitalTop.scala`, the I/O binders, and the FPGA configs/binders. The
standalone tree intentionally contains no copied `chipyard/fpga`, `chipyard/generators`, generated
Verilog, or Mill output collateral.

Before changing integration behavior, run the standalone tests, copy the changed shared file to
the Chipyard location, and compare the six files. The two copies are currently byte-identical.

## Data path and the deadlock that was removed

```text
HM01B0 pins                    Chipyard system clock
--------------                ------------------------------------------
D/FLVD/LVLD -> frontend -> CDC FIFO -> elastic stage -> full-frame BRAM -> MMIO DATA
                                                           |              (pop)
                                                           +-> EOF -> frame IRQ -> PLIC
```

Software must be able to wait for frame completion before reading pixels. Connecting MMIO DATA
directly to the capture core did not allow that: `frameCount` only advances when the in-band EOF
flit is consumed, while software was waiting for `frameCount` before consuming it. The wrapper now
drains continuously into a queue with `324 * 324 + 1 = 104977` entries, enough for the maximum
frame and EOF marker. A completed frame can therefore be reported before software starts reading.
The small synchronously reset elastic stage also prevents an async-reset FIFO flag from directly
driving a BRAM write enable, which keeps the RAM controls statically analyzable by Vivado.

If software does not drain before another frame arrives, backpressure eventually reaches the
unstallable sensor and sets `overflow`; there is no second full-frame bank. DMA or ping-pong frame
buffers are appropriate follow-on work for sustained capture.

## Opt-in FPGA configurations

Camera support is not part of either board's base config. Use:

| Board/core | Base config | Camera-enabled config |
|---|---|---|
| Arty100T Rocket | `RocketArty100TConfig` | `RocketArty100TOspiConfig` |
| KU040 Rocket | `RocketKU040Config` | `RocketKU040OspiConfig`, or `RocketKU040OspiDDRConfig` for the 2 GiB DDR4 default |
| KU040 Saturn | `SaturnKU040Config` | `SaturnKU040OspiConfig` |

The camera fragments add `TLI2C`, the open-drain I2C harness binder, the OSPI TileLink/PLIC
peripheral, the chip I/O punch-through, and the board pin binder. The generated device tree for
the Rocket configs assigns I2C interrupt source 1, UART source 2, and OSPI source 3.

From `backends/chipyard`:

```bash
source env.sh

make -C fpga SUB_PROJECT=arty100t CONFIG=RocketArty100TOspiConfig firrtl
make -C fpga SUB_PROJECT=ku040   CONFIG=RocketKU040OspiConfig firrtl

make -C fpga SUB_PROJECT=arty100t CONFIG=RocketArty100TOspiConfig bitstream
make -C fpga SUB_PROJECT=ku040   CONFIG=RocketKU040OspiConfig bitstream
```

The generated directories are named
`fpga/generated-src/chipyard.fpga.<board>.<Harness>.<Config>/`; each bitstream is under its `obj/`
directory.

## MMIO registers

Base address: `0x1008_0000`; all accesses below are 32-bit words.

| Offset | Name | Access | Definition |
|---:|---|---|---|
| `0x00` | `CTRL` | RW/W1P | `[0] enable`, `[1] continuous`, `[2] irqEnable`; write-one pulses `[3] trigger`, `[4] clear`, `[5] flush` |
| `0x04` | `GEOM` | RW | expected width `[15:0]`, expected height `[31:16]`; reset 324x244 |
| `0x08` | `MCLKDIV` | RW | `MCLK = sysclk / (2 * (value + 1))` |
| `0x0c` | `FIFOCOUNT` | RO | buffered pixel/EOF flits |
| `0x10` | `FRAMECNT` | RO | completed frames accepted into the frame queue |
| `0x14` | `LASTWIDTH` | RO | measured width of the last completed frame |
| `0x18` | `LASTHEIGHT` | RO | measured height of the last completed frame |
| `0x1c` | `FLAGS` | RO | `[0] dataValid`, `[1] overflow`, `[2] geomErr`, `[3] sensorInt`, `[4] busy`, `[5] irqPending`, `[6] frameBufferFull` |
| `0x20` | `DATA` | RO/pop | `[31] valid`, `[10] eof`, `[9] eol`, `[8] sof`, `[7:0] data`; empty reads return zero |
| `0x24` | `CAPACITY` | RO | queue capacity in flits; reset configuration reports 104977 |

`clear` clears sticky geometry/CDC overflow state and the PLIC pending latch. `flush` clears only
the system-domain frame queue; it does not reset pixels already in the CDC FIFO or stop a live
sensor frame. Disable capture before flushing if software needs a clean frame boundary.

The pulse bits share a word with persistent control bits. Preserve bits `[2:0]` whenever issuing
`trigger`, `clear`, or `flush`; a write containing only a pulse bit would also write zeros into
`enable`, `continuous`, and `irqEnable`.

## Minimal software sequence

1. Keep capture disabled. Configure the HM01B0 over I2C address `0x24`, including its output mode
   and window. The FPGA is the I2C master and video-stream receiver; the camera is the I2C slave
   and parallel-video source.
2. Write `GEOM`. QVGA is `(244 << 16) | 324 = 0x00f4_0144`.
3. Write `MCLKDIV`. At the 50 MHz system clock, value 3 produces 6.25 MHz MCLK.
4. Clear stale status and flush while preserving any desired persistent CTRL bits.
5. Set `enable`; set `irqEnable` for PLIC operation. `continuous` only suppresses the software TRIG
   pulse—the sensor's free-running/snapshot mode is selected over I2C.
6. Wait for PLIC source 3 or poll `FLAGS.irqPending`/`FRAMECNT`.
7. Read `DATA` while bit 31 is set. Store pixel bytes and honor SOF/EOL until the EOF flit.
8. Acknowledge with `CTRL.clear`, preserving persistent CTRL bits. Check `overflow`, `geomErr`, and
   measured geometry before using the image.

## Board bindings and electrical caveats

The repository's `arty100t` target is a Trenz-modified `xc7a100t-fgg484-2` shell, not the stock
Digilent CSG324 package. Its current camera pins are build defaults whose carrier/header wiring
still must be checked:

| Signal | Arty100T/Trenz FGG484 (`LVCMOS33`) | KU040 SFVA784 bank 68 (`LVCMOS18`) |
|---|---|---|
| `D[0..7]` | B17 B18 A14 A13 D16 E16 E17 F16 | H3 H4 G2 H2 E2 F2 D1 E1 |
| `PCLK` | B15 (non-CC; route override) | C3 (GC) |
| `FLVD` | A16 | E3 |
| `LVLD` | B16 | F3 |
| `INT` | F19 | F4 |
| `MCLK` | A15 | G4 |
| `TRIG` | F20 | B1 |
| `I2C SCL/SDA` | A18 / A19 | A1 / A2 |

The HM01B0 I/O supply is at most 3.0 V. The Arty shell's 3.3 V signals therefore require proper
bidirectional level translation, including open-drain I2C pull-ups referenced to safe rails. The
KU040 bank-68 1.8 V standard is directly compatible when the sensor IOVDD is 1.8 V. Weak FPGA
pull-ups are emitted for bring-up; external I2C pull-ups sized for the bus are recommended.

Arty PCLK currently uses a non-clock-capable package pin and `CLOCK_DEDICATED_ROUTE=FALSE`; use a
carrier-connected MRCC/SRCC pin when the hardware permits. Both shells constrain PCLK to the
HM01B0's 36 MHz maximum and place it in an asynchronous timing group relative to system clocks.

Input delays are unconstrained because the preliminary datasheet defines no PCLK sampling edge or
source-output timing — see the conformance boundary in [`README.md`](README.md). Board-specific
values: at 50 MHz, MCLK divider 0–7 covers 25–3.125 MHz at 50% duty; program I2C no faster than
400 kHz; the TRIG pulse is one system-clock cycle; power-rail sequencing is external.

## Remaining hardware work

The RTL/SoC/shell integration has passing Arty100T and KU040 bitstreams. Receiving a real image
still requires:

- confirm both default pin maps against the actual carrier schematics and camera connector;
- provide the Arty voltage translation and external I2C pull-ups;
- add or adapt an HM01B0 initialization sequence in bare-metal or Zephyr software;
- perform a hardware smoke test that captures a known scene/test pattern and checks geometry and
  overflow;
- add DMA or ping-pong buffering only if sustained frame rate, rather than single-frame bring-up,
  is required.
