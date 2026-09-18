# DroNet on the KU040 dual-core SoC — reproduction (2026-09-18)

End-to-end grayscale int8 DroNet running **across both harts** of the KU040 dual-core
SoC on real silicon: hart0 = Rocket + Saturn V128 (RVV) + FcRoCC, hart1 = Rocket +
Q0.31 **32×32** Gemmini. **~18–20 fps**, gemmini-bound, int8-mvout accuracy
`max_abs_err = 2` (2 LSB) vs the model's own reference.

Deployable ELF: `lorenshung_elf/ku040/ku040.dronet_gray_dual_2hart.elf`.

## Config / bitstream
- Chisel config `RocketKU040DroneDualConfig` (`Q31Ws32x32AccGemminiConfig`, DIM=32,
  WS-only, Q0.31 acc-scale, 128 KB accumulator; Saturn V128 int-only on hart0; TACIT
  both tiles, DSC branch predictor off; all carrier peripherals). Timing-closed
  (WNS +0.128 ns, 89% LUT).
- Bitstream: `bitstreams/KU040_RocketDroneDual_32x32.bit` (SRAM-load only; the FPGA
  does not auto-boot from flash — MODE = JTAG).

## Model + kernel split (the "what runs where")
Grayscale (IC=1) fully-integer DroNet, conv0+maxpool **fused**. 20-op graph, split by
capability across the two harts (no solver needed — capability-driven):
- **hart1 / Gemmini (`gemmini_q31`), 10 ops:** conv0 = fused `conv2d_pool_s8` via
  **`gemmini_tiled_conv_pool`** (path A: HW conv + HW-pool tail, int8-mvout, **bit-exact**
  for conv0's params); conv1–9 = `conv2d_s8` via **`gemmini_tiled_conv`** (HW im2col,
  int8-mvout). **No SW im2col (`im2col_full_C`) anywhere.**
- **hart0 / Saturn-RVV (`rvv`), 10 ops:** batchnorm/relu/linear → `direct`, add →
  `rvv_frm_rmm`, sigmoid → `rvv_memo_lut_gather`. Every V-using op stays on hart0.

## Measured (rdcycle @ 50 MHz, on-board)
| op | cycles |
|---|---:|
| conv0 (fused conv+pool, IC=1) | 934K (37% of gemmini time) |
| conv1–9 (tiled_conv int8-mvout) | 72K–404K each |
| **gemmini total (hart1)** | **2.52M ≈ 50.4 ms** — the bottleneck |
| rvv total (hart0) | 0.53M ≈ 10.6 ms (mostly idle, waiting on gemmini) |

`MODELBLASTER_VERIFY max_abs_err=2` (int8-mvout drift; conv0 path A is exact),
output `[115, 127]`.

## Deploy steps
1. SRAM-load the bitstream (FPGA config JTAG):
   `openFPGALoader -c ft2232 --busdev-num <bus:dev> bitstreams/KU040_RocketDroneDual_32x32.bit`
2. Load + run the ELF over the Rocket user JTAG, **resuming BOTH harts** (SMP: hart0
   boots, hart1 is the secondary): `scripts/openocd/ku040_dual_smp.cfg`
   (`init; reset halt; load_image <elf>`; set both harts `pc=0x80000000`; `resume`).
3. Console: uart0 @ 115200 (bench: `/dev/ttyUSB3`, mapping re-derives after any replug —
   probe: Rocket DTM `0x00000001` = user JTAG, Xilinx `0x13822093` = config JTAG).
4. Expect `MODELBLASTER_PROFILE` per-op cycles + `MODELBLASTER_VERIFY max_abs_err`.

## Key fixes this brought together (see also memory: riskybird-gemmini-dim-mismatch)
- **Gemmini "wrong int32 output" was a DIM=16-vs-32 `gemmini_params.h` mismatch**, not a
  HW fault (spike=0 / HW≠0 / RVV=0 is the fingerprint). Fixed via the DIM=32
  `q31ws_32x32_acc` per-config header. Now bit-exact.
- **conv0 int8-mvout path A** (`tiled_conv_auto` + HW pool) is bit-exact + 586K standalone;
  6× faster than conv0+maxpool on RVV. The old "gemmini conv0 = 7.4M" was the SW-im2col
  path (avoid it). **Load-once conv0 does NOT fit the 32×32 scratchpad** (one pixel/row
  needs 12,996 rows > the DIM=32 mesh's 8,192) — deferred; needs a pixel-packing rewrite.
- Deploy is a **two-backend compile** (`generated/gemmini_q31` + `generated/rvv`), not the
  single hetero `gemmini_q31_rvv` — the xpurt runner dispatches per worker-kind, so op
  impls must be `gemmini_q31`/`rvv`.

## Software reproduction (ModelBlaster)
Branch `riskybird-fps-repro` on `ucb-bar/ModelBlaster` holds the pipeline fixes
(`profile_kernel.py` MODEL_DIR-abspath, `scripts/run_xpurt_scheduler.py` CSV parser),
the `q31ws_32x32_acc` per-config headers, the config descriptor
`cores/chipyard_ku040_dronedual_q31.json`, the validation harnesses
(`harness_conv0_lo`, `harness_gemmini_mm_sanity`), the 2-hart schedule/main, the
`build_ku040_dual.sh` recipe, and `notes/DRONET_KU040_DUAL_REPRODUCTION.md` with the
full env recipe (DIM=32 / `MODELBLASTER_GEMMINI_CONFIG=q31ws_32x32_acc`,
`MODELBLASTER_CURATED_VERIFY=0`, grayscale `MODELBLASTER_DRONET_CHANNELS=1`,
`MB_ENABLE_FUSION=1`, two-backend `--core-kinds rvv,gemmini_q31`, eager-V +
`RISCV_V_KERNEL_ONLY`, soft-float, and the `ZEPHYR_TOOLCHAIN_VARIANT`/SDK/`ZEPHYR_BASE`
env gotchas).

## Next optimization (track a)
conv0 is 934K here but 586K standalone in NHWC — the deploy graph is NCHW so the gemmini
kernels pay a layout-conversion penalty. Restore the NHWC island (`assign_layouts`, with
relayouts on the gemmini hart) to pull conv0 toward 586K and lift fps; the load-once
DIM=32 rewrite (→456K) is a further step.
