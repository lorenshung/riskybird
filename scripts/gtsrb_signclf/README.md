# GTSRB street-sign classifier — modelblaster int8 setup

Small **standard-conv** CNN (Gemmini-friendly, no depthwise) for GTSRB traffic-sign
classification, quantized through modelblaster's faithful int8 flow. Built for the KU040
SoC to detect STOP/YIELD (and the full 43-class GTSRB set) for demos.

## Results (validated 2026-09-20, full 12,630-img GTSRB test set)
| Model | input | float top-1 | int8 top-1 (faithful) | STOP recall | notes |
|---|---|---|---|---|---|
| **heavy grayscale (deploy)** | 48×48×1 | **99.0%** | **98.97%** (−0.06 pp) | **100%** | int8 verify max\|diff\|=0; int8↔float agree 99.71% |
| heavy RGB | 48×48×3 | 98.6% | ~98.6% (bit-exact) | 100% | *lower* than grayscale |
| lite grayscale/RGB | 48×48 | ~95–99% val / 95.3% test | — | 99.3% | smaller/faster tradeoff |

**Grayscale is the deploy choice** — it beats RGB (signs are shape-distinct), so the
HM01B0 grayscale camera suffices; no color camera needed. Classification argmax is robust
to int8 (near-lossless), unlike the DroNet regression case.

## Files
- `model.py` — `SignNet` (conv backbone) + `SignNetSoftmax` (int8-eval wrapper).
- `data.py` / `dl_gtsrb.py` — GTSRB loader + download (`torchvision.datasets.GTSRB` / archive). Consts: `N_CLASSES`, `INPUT`, `STOP`(=14), `YIELD`(=13).
- `train.py` / `train_lite.py` — float trainers (heavy / lite), grayscale + RGB via `--channels`.
- `eval_int8.py` / `eval_lite.py` — faithful int8 top-1: `extract_int8(model, sample, name, out, calibration_samples=, fold_conv_bn=False)` + the golden `int8_sim` (bit-exact, verify max|diff|=0) + `fast_int8.py` (fast bit-exact conv, cross-checked vs golden). Reports int8 vs float top-1 + STOP/YIELD per-class.
- `run_*.sh` — garden launchers (setsid + log + `.done` sentinel).
- `probe.py` / `smoke_int8.py` — sanity checks.

## Reproduce (on garden)
```bash
PY=/scratch2/dima/miniforge3/envs/xpurt/bin/python   # torch 2.7, TITAN RTX
export MODELBLASTER_GEMMINI_CONFIG=q31ws_32x32_acc MODELBLASTER_CURATED_VERIFY=0
bash run_dl.sh        # download GTSRB -> ./data (not committed, ~690MB)
bash run_train.sh     # -> ./ckpt (not committed)
bash run_eval.sh      # faithful int8 eval -> ./logs, ./int8_out (not committed)
```
**Dependencies** (garden paths, as the scripts set on `sys.path`): modelblaster at
`/scratch2/dima/misc_sw/xpurt_repro_wt` (provides `modelblaster.pipeline.extract_graph`),
and the shared golden `int8_sim.py` at `/scratch2/dima/misc_sw/dronet_int8_eval/`
(also committed here at `../dronet_temporal/int8_sim.py`).

## Not committed (regenerable): `data/` (GTSRB), `ckpt/` (weights), `int8_out/` (graphs), `logs/`.

## Deploy caveat
GTSRB is photographs of real signs; 3D-printed plastic signs are a domain shift. The 99%
validates the architecture + int8 flow; finetune on a few HM01B0 shots of the actual
printed signs before the demo. Deploy = the same 2-hart modelblaster flow as DroNet;
int8-only (no fp16 tail — classification is robust to int8).
