#!/usr/bin/env bash
# Provision the generated DroNet model files the FC+DroNet build needs
# (model.c/kernels.c/weights.c/buffers.c/model.h/kernels.h/test_io.{S,h}/
#  test_input.bin/test_golden.bin) for the 13.31 fps grayscale integrated build.
#
# The optimized inference == modelblaster branch riskybird-fps-integrated @ 10b50ba
# (NHWC transpose-delete + NHWC conv0->maxpool1 fusion + rvv_seg boundary
# relayouts, grayscale, bit-exact; 2,629 kcyc). Its generated C is the
# gemmini_q31_rvv target of examples/dronet/int8/generated_gray/hetero_tiled_rvvadd.
#
# Two modes:
#   (A) COPY from an existing modelblaster checkout that already carries the
#       generated dir (fast; the default). Set MB_SRC to that checkout.
#   (B) REGENERATE from scratch via the modelblaster pipeline (needs the conda
#       env + torch). Shown below; run it inside a standalone MB_SRC @ 10b50ba
#       if the generated dir is absent -- never inside backends/modelblaster,
#       which is a clean upstream pin.
# Either way the files land in DST, under this repository's (git-ignored)
# build/ directory, which is where scripts/build_fc_dronet.sh and
# ../CMakeLists.txt look for them by default.
set -uo pipefail

RB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
MB_SRC="${MB_SRC:-${RB_ROOT}/backends/modelblaster}"
REL="examples/dronet/int8/generated_gray/hetero_tiled_rvvadd"
SRC="${MB_SRC}/${REL}"
DST="${DST:-${RB_ROOT}/build/fc_dronet/hetero_tiled_rvvadd}"

need=(model.c kernels.c weights.c buffers.c model.h kernels.h test_io.S test_io.h test_input.bin test_golden.bin)

have_all() { local d="$1"; for f in "${need[@]}"; do [ -f "$d/$f" ] || return 1; done; return 0; }

if have_all "$SRC"; then
  echo "OK: generated DroNet model present at ${SRC}"
  if [ "$DST" != "$SRC" ]; then
    mkdir -p "$DST"; cp -v "${SRC}"/* "$DST"/ 2>/dev/null || true
    echo "copied -> ${DST}"
  fi
  echo ">> build with:  MODEL_DIR=${DST} workloads/fc_dronet/scripts/build_fc_dronet.sh"
  exit 0
fi

cat <<EOF
MISSING: ${SRC} is incomplete.

REGENERATE (mode B) inside a standalone modelblaster checkout @ 10b50ba
(not backends/modelblaster, which stays a clean pin):

  cd <that checkout>
  source ${RB_ROOT}/backends/zephyr-chipyard-sw/scripts/activate_conda.sh
  # grayscale (1ch), fused Gemmini+Saturn(RVV) target, NHWC islands + conv0-pool
  # fusion + rvv_seg relayouts (the integrated stack):
  MODELBLASTER_DRONET_CHANNELS=1 \\
  TARGET=gemmini_q31_rvv BACKEND=reference \\
  MB_ENABLE_FUSION=1 \\
    bash examples/dronet/run.sh --enable-fusion --assign-layouts islands
  # -> writes ${REL%/*}/hetero_tiled_rvvadd/  (model.c/kernels.c/weights.c/...)

Then re-run this script with MB_SRC=<that checkout> to copy it to ${DST}
(or pass MODEL_DIR to scripts/build_fc_dronet.sh).

The same recipe's committed, verify-gated output for this shell is
${RB_ROOT}/artifacts/modelblaster/dronet/int8/gemmini_q31_rvv_softfp-gray-nhwc
(tools/modelblaster/dronet_gray_nhwc.py); MODEL_DIR may name it directly.
EOF
exit 1
