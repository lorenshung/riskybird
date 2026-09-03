# FPGA LUT / Resource Utilization — RocketArty200TDroneFullDDRDmaConfig

**Device:** Xilinx Artix-7 **XC7A200T**-fbg484-1 (Trenz TE0712 SoM)
**Config:** `chipyard.fpga.arty200t.Arty200THarness.RocketArty200TDroneFullDDRDmaConfig`
(RV64GC Rocket SoC + DDR3/MIG + I2C + SPI + 2×PWM + GPIO + 2nd UART/ESP link + OSPI HM01B0 camera-capture core with a TileLink-master DMA)
**Toolchain:** Vivado v.2023.1, build on `garden` (Ubuntu 24.04), 2026-09-03
**Numbers are POST-ROUTE (post-implementation, real placed+routed)** — `Design State: Physopt postRoute`, not a synthesis estimate.
**Source report:** `report_utilization -hierarchical` →
`/scratch/dima/riskybird_chipyard/fpga/generated-src/chipyard.fpga.arty200t.Arty200THarness.RocketArty200TDroneFullDDRDmaConfig/obj/report/utilization.txt` (on garden)

## Whole-design totals

| Resource | Used | Available (XC7A200T) | % of device |
|---|---:|---:|---:|
| LUTs        | 50,471 | 133,800 | **37.7 %** |
| Flip-flops  | 29,138 | 267,600 | 10.9 % |
| RAMB36      | 60     | 365     | 16.4 % |
| RAMB18      | 96     | 730     | 13.2 % |
| BRAM (36-eq)| 108    | 365     | **29.6 %** |
| DSP48       | 25     | 740     | 3.4 % |

The design occupies a bit over a third of the XC7A200T's LUTs. BRAM is the second-tightest resource (~30 % in 36 Kb-equivalents), driven mostly by the camera frame buffer and the caches; DSPs are nearly free.

## LUT utilization by component (ranked)

Rows follow the natural Vivado hierarchy boundary: the DDR3 MIG (a top-level sibling of the SoC) plus the `DigitalTop` sub-domains. "% design" is of the 50,471 total; "% chip" is of the 133,800 device LUTs.

| # | Component | LUTs | % design | % chip | FFs | RAMB36 | RAMB18 | DSP |
|---:|---|---:|---:|---:|---:|---:|---:|---:|
| 1 | **Rocket RV64GC core tile** (core + FPU + I$/D$ + PTW) | **29,169** | **57.8 %** | **21.8 %** | 13,280 | 0 | 96 | 25 |
| 2 | **DDR3 memory controller** (Xilinx MIG + AXI adapters) | **10,474** | **20.8 %** | **7.8 %** | 10,378 | 0 | 0 | 0 |
| 3 | cbus — control/periphery TL bus + crossbar | 2,676 | 5.3 % | 2.0 % | 443 | 0 | 0 | 0 |
| 4 | pbus — peripheral TL bus + crossbar | 1,769 | 3.5 % | 1.3 % | 584 | 0 | 0 | 0 |
| 5 | **OSPI camera-capture core + TileLink DMA (HM01B0)** | **1,275** | **2.5 %** | **1.0 %** | 1,084 | 44 | 0 | 0 |
| 6 | tlDM — RISC-V debug module (DMI / abstract cmds) | 1,047 | 2.1 % | 0.8 % | 1,199 | 0 | 0 | 0 |
| 7 | bootrom | 667 | 1.3 % | 0.5 % | 0 | 0 | 0 | 0 |
| 8 | mbus — memory TL bus | 469 | 0.9 % | 0.4 % | 45 | 0 | 0 | 0 |
| 9 | PLIC — external interrupt controller | 422 | 0.8 % | 0.3 % | 200 | 0 | 0 | 0 |
| 10 | sbus — system TL bus | 363 | 0.7 % | 0.3 % | 52 | 0 | 0 | 0 |
| 11 | fbus — front TL bus (DMA / serial-TL / debug ingress) | 361 | 0.7 % | 0.3 % | 160 | 0 | 0 | 0 |
| 12 | coh_wrapper — coherence manager (broadcast hub) | 330 | 0.7 % | 0.2 % | 276 | 0 | 0 | 0 |
| 13 | Scratchpad bank (TL RAM, 16× RAMB36) | 245 | 0.5 % | 0.2 % | 123 | 16 | 0 | 0 |
| 14 | SPI controller | 212 | 0.4 % | 0.2 % | 187 | 0 | 0 | 0 |
| 15 | DigitalTop glue / top interconnect | 200 | 0.4 % | 0.1 % | 10 | 0 | 0 | 0 |
| 16 | I2C controller | 115 | 0.2 % | 0.1 % | 119 | 0 | 0 | 0 |
| 17 | PWM0 controller | 103 | 0.2 % | 0.1 % | 125 | 0 | 0 | 0 |
| 18 | PWM1 controller | 100 | 0.2 % | 0.1 % | 125 | 0 | 0 | 0 |
| 19 | UART0 controller | 93 | 0.2 % | 0.1 % | 108 | 0 | 0 | 0 |
| 20 | UART1 controller (ESP link) | 93 | 0.2 % | 0.1 % | 108 | 0 | 0 | 0 |
| 21 | PRCI / clock + reset control | 91 | 0.2 % | 0.1 % | 62 | 0 | 0 | 0 |
| 22 | JTAG debug transport module (DTM) | 84 | 0.2 % | 0.1 % | 167 | 0 | 0 | 0 |
| 23 | serial-TL (TSI) domain | 44 | 0.1 % | 0.0 % | 31 | 0 | 0 | 0 |
| 24 | CLINT — timer / soft interrupts | 36 | 0.1 % | 0.0 % | 131 | 0 | 0 | 0 |
| 25 | GPIO controller | 35 | 0.1 % | 0.0 % | 57 | 0 | 0 | 0 |
| — | ChipTop + harness glue (resets / POR / PLL / clk-wrangler) | ≈0¹ | — | — | 84 | 0 | 0 | 0 |
| | **TOTAL (Arty200THarness)** | **50,471** | 100 % | 37.7 % | 29,138 | 60 | 96 | 25 |

¹ The harness/glue row nets to ≈0 (−2) LUTs because Vivado's hierarchical report distributes a handful of shared LUTs across boundaries; component rows can therefore be off by a few LUTs versus a strict tree sum.

## What dominates, and why

- **The Rocket RV64GC tile is the design — 57.8 % of all LUTs (21.8 % of the chip).** Everything else combined is smaller than the core. Inside the tile, the **double-precision FPU alone is 13,945 LUTs — 48 % of the tile and 27.6 % of the entire design.** This is the cost of the `G` (= IMAFD) extension: full hardware single+double float. If FPU area ever needs to be reclaimed, dropping to `RV64IMAC` (soft-float) would free ~14 k LUTs (~10 % of the chip) in one move.

| Rocket tile internal | LUTs | % of tile | % of design | FFs | RAMB18 | DSP |
|---|---:|---:|---:|---:|---:|---:|
| **FPU** (double-precision) | 13,945 | 48 % | 27.6 % | 3,971 | 0 | 12 |
| Rocket core (integer pipeline) | 6,434 | 22 % | 12.7 % | 2,031 | 0 | 13 |
| Frontend (I-fetch + I$) | 4,758 | 16 % | 9.4 % | 4,317 | 24 | 0 |
| DCache (D$) | 3,081 | 11 % | 6.1 % | 2,392 | 72 | 0 |
| PTW (page-table walker) | 569 | 2 % | 1.1 % | 539 | 0 | 0 |
| TL buffers / xbar / arbiter | 382 | 1 % | 0.8 % | 30 | 0 | 0 |

- **DDR3 MIG is the #2 block — 10,474 LUTs (7.8 % of chip).** ~8,100 LUTs are the Xilinx `arty200tmig` hard-controller soft logic (calibration/PHY glue), plus ~1,340 for the AXI4 deinterleaver and ~500 for the AXI async clock-crossing. It also owns almost all of the design's flip-flops after the core (10.4 k FFs).
- **The TileLink interconnect is a real cost.** cbus (2,676) + pbus (1,769) + mbus (469) + sbus (363) + fbus (361) + coherence hub (330) ≈ **5,968 LUTs (~12 % of the design)** — more than the camera, debug, and every peripheral put together. cbus/pbus carry the address decoders, width/protocol adapters and crossbars fanning out to the many MMIO devices.
- **The OSPI HM01B0 camera-capture core + DMA is modest in LUTs (1,275, 1.0 % of chip) but is the design's single biggest BRAM consumer: 44 of the 60 RAMB36 (12 % of the chip's block RAM).** Those 44 RAMB36 are the on-chip frame buffer (`FrameBuffer`); the `HM01B0Capture` DVP front-end + async pixel FIFO is 579 LUTs, and the TileLink-master DMA path (the `s11`-source TL buffers that push frames to DDR) is the rest. LUT-cheap, BRAM-hungry.
- **Debug (tlDM 1,047 + DTM 84 = 1,131 LUTs)** is non-trivial — a fixed bring-up/JTAG tax that could be trimmed in a production bitstream.
- **All the flight-relevant peripherals are nearly free:** I2C + SPI + 2×PWM + 2×UART + GPIO together are **751 LUTs (0.6 % of the chip).** The sensor/motor/ESP I/O is not what fills the part — the CPU is.

## Files

- `LUT_utilization_DmaConfig.md` — this report
- `LUT_utilization_DmaConfig.csv` — the ranked component table (machine-readable)
- `lut_by_component.png` — horizontal bar chart, LUTs by component (sorted)
- `lut_share_donut.png` — donut of the top-8 LUT contributors + "other"
- `rocket_tile_breakdown.png` — Rocket tile internals (FPU / core / frontend / D$ / PTW)

Raw source copied for reference: hierarchical util report on garden at the path above.
