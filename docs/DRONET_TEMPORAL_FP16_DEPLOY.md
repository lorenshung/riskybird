# Temporal DroNet + FP16-hybrid KU040 deploy — reproduction

End-to-end recipe for the **N=3 frame-stacked (temporal) DroNet** running on the KU040
dual-core SoC at **~27 fps, bit-exact int8**, plus the **FP16-Saturn** bitstream that lets
the ReLU→FC tail run in fp16 (optional turn-accuracy lever).

Scripts referenced here live in `scripts/dronet_temporal/`. Heavyweight code stays on the
garden (chipyard bitstream builds, `modelblaster`, `XPU-RT`); this doc pins the exact
commands + the source changes made in those repos.

## TL;DR results (on real KU040 silicon, 2026-09-20)

| Model | float EVA | int8 EVA (val) | turn-EVA | on-board VERIFY | fps @50MHz |
|---|---|---|---|---|---|
| single-frame grayscale | 0.115 | 0.012 (collapse) | 0.14 | err=3 | ~22 |
| **N=3 temporal** | **0.433** | **0.300** | **0.463** | **err=0 (bit-exact)** | **~26.8** |

Temporal frame-stacking is the accuracy win (25× the single-frame int8); the int8 deploy needs
no fp16. FP16 is an optional turn-accuracy lever (see §4).

> **Accuracy caveat:** the PULP-DroNet v3 official split is per-*frame*, not per-time-block, so
> valid stacks sit ~40 ms from train frames (temporal-adjacency leakage). The *direction*
> (temporal ≫ single-frame) is unambiguous; to trust the absolute 0.433/0.300, re-split by whole
> acquisition / time-block and re-eval both N=1 and N=3. Deployment (bit-exact, fps) is solid regardless.

## 0. Hosts / envs
- **garden** (`ssh dima-garden`): training + `modelblaster` + chipyard. Python env `xpurt`
  (`/scratch2/dima/miniforge3/envs/xpurt/bin/python`, torch 2.7+cu128, TITAN RTX). Zephyr build
  env `zephyr` conda env. Vivado 2023.1 at `/ecad/tools/xilinx`.
- **cobble** (local, the bench): FPGA on two FT2232H (config JTAG + Rocket JTAG/console),
  `openFPGALoader` + `openocd`.

## 1. Dataset — real HM01B0 (PULP-DroNet v3)
Zenodo 13348430, on garden at `/scratch2/dima/misc_sw/XPU-RT/datasets/pulp_dronet_himax/raw/Dataset_PULP_Dronet_v3/`.
Layout `<collector>/dataset-sessionN/acquisitionM/{images/<frame>.jpeg, labels_partitioned.csv}`
(cols `filename,partition,label_yaw_rate,label_collision`). `label_yaw_rate` is **deg/s ±90** →
normalize `/90`. Frames are ms-timestamped (int filenames, ~22 Hz). Download endpoint gotcha: use
`https://zenodo.org/records/13348430/files/<name>?download=1` (the API `/content` 504s).

## 2. Train the temporal model — `scripts/dronet_temporal/train_stack.py`
Only change from single-frame DroNet: **conv0 input depth 1→N** (`DronetTorch(img_channels=N)`),
so N consecutive grayscale frames are stacked as N channels. Zero new operators → stays in the
int8-Gemmini envelope. Correctness rules (all enforced in the trainer):
- frames time-ordered by `int(filename)` within an acquisition; stack = `[i-(N-1)…i]` oldest→newest, label = frame i.
- never stack across acquisition boundaries; require all N frames share the target's `partition` (no leakage); drop filenames >1e12 (uint64-overflow anomalies).
- hflip flips all N channels together and negates the label; RandomCrop identical across channels.
- MSE loss, Adam lr 1e-3 wd 1e-4, CosineAnnealing, 45 epochs, batch 64.

```bash
ssh dima-garden
PY=/scratch2/dima/miniforge3/envs/xpurt/bin/python
for N in 1 3 5; do N=$N EPOCHS=45 $PY scripts/dronet_temporal/train_stack.py --N $N; done
# → /scratch2/dima/misc_sw/XPU-RT/logs/dronet_himax_stack{1,3,5}/best.pt
# N=1 val EVA 0.090 (reproduces single-frame baseline), N=3 0.433 (BEST), N=5 0.322
```

## 3. int8 / fp16 characterization (host sim) — `hybrid_sim.py`, `fp16_char.py`
Faithful int8 = `modelblaster.pipeline.extract_graph.extract_int8(model, sample, name, out,
calibration_samples=, fold_conv_bn=False, fp16_op_names={...})` + the golden `int8_sim`/`hybrid_sim`
(bit-exact, verified max|lsb|=0). `hybrid_sim.py` runs a per-op int8/fp16 mix (fp16 ops use
full-precision weights, fp32 accumulate) with auto int8↔fp16 casts. `fp16_char.py` sweeps per-op
fp16 promotion and reports EVA recovery. Finding: for the temporal model the int8 loss is in the
**convs** (only_convs recovers ~49%), NOT the ReLU→FC tail; promoting the tail is a turn-vs-straight
tradeoff (turn 0.463→0.621, overall 0.300→0.172). all_fp16 ≈ float (pipeline faithful).

## 4. FP16-Saturn bitstream (optional, for the fp16 tail) — garden chipyard
The deployed `RocketKU040DroneDualConfig` uses `VectorParams.intOnlyParams` (Saturn is
integer-only, **cannot run fp16**). Added FP16 ("robotMpc") Saturn variants to
`fpga/src/main/scala/ku040/Configs.scala` (reference copies in `chipyard_configs/`):

| config | Saturn | Gemmini | post-route LUT | verdict |
|---|---|---|---|---|
| `RocketKU040DroneDualFp16Config` | robotMpc **D128** | 32×32 | 99.86% | won't route |
| `RocketKU040DroneDual16Fp16Config` | robotMpc D128 | 16×16 | 83.4% | fits (slower conv, DIM=16) |
| **`RocketKU040DroneDualFp16D64Config`** | robotMpc **D64** | 32×32 | **83.8%** | **WINNER** |

The int/float MAC split is **99.98% int8 / 0.018% fp16** (float = only the final ReLU + steering
FC, 2048 MACs), so a full-width fp16 vector unit is overkill — halving DLEN 128→64 (VLEN stays 128,
so kernels/model are unchanged) fits the fp16 unit *and* keeps the 32×32 Gemmini. Build:
```bash
ssh dima-garden; cd /scratch/dima/riskybird_chipyard; source env.sh; export PATH=/scratch/dima/espresso-install/bin:$PATH
make -C fpga SUB_PROJECT=ku040 CONFIG=RocketKU040DroneDualFp16D64Config RB_ATTRS=dsp rb-area   # synth/area
make -C fpga SUB_PROJECT=ku040 CONFIG=RocketKU040DroneDualFp16D64Config RB_ATTRS=dsp rb-impl   # P&R → rb_impl.bit
# timing MET (0 failing endpoints); bitstream → .../RocketKU040DroneDualFp16D64Config/obj/rb_impl.bit
```
On silicon the int8 DroNet runs unchanged and the DLEN=64 RVV penalty is hidden behind the
Gemmini critical path → zero fps cost.

## 5. Build the 2-hart deploy ELF — `scripts/dronet_temporal/stack3_build.sh`
Adapts the single-frame NHWC build: `MODELBLASTER_DRONET_CHANNELS=3` +
`MODELBLASTER_DRONET_CKPT=…/dronet_himax_stack3/best.pt` + separate `generated_stack3`/build dirs.
Two-backend compile (`generated/gemmini_q31` + `generated/rvv`), harness_xpurt, SMP boot hart0,
eager-V + `RISCV_V_KERNEL_ONLY`. Kernel picks: conv0→`gemmini_tiled_conv_pool_nhwc` (fused),
convs→`gemmini_tiled_conv_nhwc` (HW im2col, int8-mvout), bn/relu/linear→rvv `direct`,
relayouts→`gemmini_blocked_tb32`; 14 gemmini_q31 + 14 rvv ops.
```bash
ssh dima-garden; setsid bash -c '/tmp/stack3_build.sh > /tmp/stack3_build.log 2>&1; echo done > /tmp/stack3_build.done'
# → examples/xpurt_demo/int8/build/ku040_dronet_stack3/zephyr/zephyr.elf (2.36 MB)
```
Env gotchas (all in the script): set `ZEPHYR_BASE`/`ZEPHYR_SDK_INSTALL_DIR`/`ZEPHYR_TOOLCHAIN_VARIANT=zephyr`
explicitly (do NOT `source set_envvars_sdk.sh` under set -e — its `find|head` SIGPIPEs); DIM=32
`per_config/q31ws_32x32_acc` headers; `MODELBLASTER_CURATED_VERIFY=0`; `-DMODEL_DIR` absolute.

## 6. Flash + run on the board (cobble)
The two FT2232H are identical `0403:6010`; distinguish by JTAG probe (config JTAG reads the xilinx
IDCODE). This session: config JTAG = busdev `3:122`, Rocket JTAG = `3:6` (openocd `FTDI_LOCATION`),
console = the Rocket-JTAG FTDI's UART interface (**ttyUSB1 this session — the tty mapping
re-derives per replug; find it via `readlink /sys/class/tty/ttyUSBn/device` → the `…-6:1.1` node**).
```bash
cd /home/cobble/Tools/riskybird
# 1. flash (volatile JTAG config; power-cycle reverts). Any int8 bitstream works; here the fp16 D64:
openFPGALoader -c ft2232 --busdev-num 3:122 bitstreams/KU040_RocketDroneDualFp16D64.bit
# 2. start console reader on the mapped tty (from a script file; fuser -k to clear stale readers)
# 3. load + resume both harts:
FTDI_LOCATION=3-6 openocd -f scripts/openocd/ku040_dual_smp.cfg \
  -c init -c "reset halt" -c "load_image lorenshung_elf/ku040/ku040.dronet_stack3_2hart.elf" \
  -c "targets riscv.cpu0" -c "reg pc 0x80000000" -c "targets riscv.cpu1" -c "reg pc 0x80000000" \
  -c resume -c exit
```
Bench gotchas: openocd claims the Rocket-JTAG FTDI → its UART re-enumerates (start the reader from
a script, read after openocd settles). **NEVER `pkill -f "<pattern in your own command>"`** (self-match
kills your shell); use `fuser -k /dev/ttyUSBn`.

Expected console: `MODELBLASTER_VERIFY [dronet] max_abs_err=0`, 14 rvv + 14 gemmini ops,
`WALL_CYCLES 1866` (KILOCYCLES → 1.866M cyc @50MHz = 37.3 ms = **~26.8 fps**). The `_us`/WALL
profiler columns are kilocycles, not microseconds.

## 7. Performance / further optimization
Gemmini is the bottleneck (~1.13M cyc); the biggest remaining lever is the 8 NCHW↔NHWC relayouts
(~285K cyc on the critical path) — fuse bn/add into the NHWC island so residual blocks stay one
layout. Not needed for the 20 fps target (already ~27). Note: if the Gemmini path is cut, the
DLEN=64 RVV (~0.61M cyc) becomes the new bottleneck → revisit DLEN then.

## Source changes committed alongside this doc
- `scripts/dronet_temporal/` — trainer, deploy build, int8/fp16 sim tooling, chipyard config refs.
- **garden `riskybird_chipyard`**: `fpga/src/main/scala/ku040/Configs.scala` — 3 FP16 Saturn configs.
- **garden `xpurt_repro_wt/modelblaster`**: `pipeline/extract_graph.py` — added `relu_f16`/`add_f16`/
  `batchnorm2d_f16` branches to the golden verifier (the fp16-tail deploy blocker).
