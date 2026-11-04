#!/usr/bin/env bash
# Build the integrated FC + DroNet ELF for the riskybird v3 FPGA Rocket combined
# core RocketArty200TDroneGemminiSaturnFp16At35Config (Gemmini + Saturn + full
# drone periphery). Off-bench; produces build_fc_dronet/zephyr/zephyr.elf.
#
# Env overrides:
#   MB_REPO   modelblaster checkout to take the gemmini runtime headers from
#             (default: this checkout's backends/modelblaster; only read).
#   MODEL_DIR the generated DroNet model dir (default: the 13.31 fps grayscale
#             integrated build, generated_gray/hetero_tiled_rvvadd, as
#             scripts/gen_dronet_model.sh provisions it under build/fc_dronet/).
#   IRQ_GATE  1 => build the mid-V-preemption-safe fallback (DRONET_IRQ_GATE=1).
set -uo pipefail

WT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"          # workloads/fc_dronet
RB_ROOT="$(cd "${WT}/../.." && pwd)"
MAIN="${MAIN:-${RB_ROOT}/backends/zephyr-chipyard-sw}"          # conda + SDK + west workspace live here
MB_REPO="${MB_REPO:-${RB_ROOT}/backends/modelblaster}"
MODEL_DIR="${MODEL_DIR:-${RB_ROOT}/build/fc_dronet/hetero_tiled_rvvadd}"
BUILDDIR="${BUILDDIR:-${WT}/build_fc_dronet}"
IRQ_GATE="${IRQ_GATE:-0}"
# ROSE_TELEM=0 quiets the FC per-iter it= console spam so the DRONET lines + PROFILE
# loop timing come through the 115200 console cleanly; DRONET_REPORT_EVERY=1 prints
# every inference. Override EXTRA_CPP to change.
EXTRA_CPP="${EXTRA_CPP:--DROSE_MOTORS_INHIBIT=1 -DROSE_PROFILE=1 -DROSE_TELEM=0 -DDRONET_REPORT_EVERY=1 -DDRONET_IRQ_GATE=${IRQ_GATE}}"

# Toolchain + west workspace come from the Zephyr backend checkout (a valid west workspace
# whose Zephyr submodule is the same riskybirdv3-bringup tip, bfd185f, with the
# FC driver fixes + the fp16 boot fix). The build reads it read-only; the sample,
# FC sources and build dir all live in this worktree.
source "${MAIN}/scripts/activate_conda.sh"
source "${MAIN}/scripts/set_envvars_sdk.sh"
export PATH="/usr/bin:${PATH}"   # avoid stale Vitis cmake

KCFLAGS="-march=rv64imac_zve64x;-mabi=lp64"
# The shell's Gemmini parameters, first on the include path (see GEMMINI_PARAMS_DIR in ../CMakeLists.txt).
GPARAMS="${GEMMINI_PARAMS_DIR:-${RB_ROOT}/hardware/modelblaster/cores/gemmini/include/per_config/RocketArty200TDroneGemminiSaturnFp16At35Config}"
KCFLAGS="${KCFLAGS};-isystem${GPARAMS}"
KCFLAGS="${KCFLAGS};-isystem${MB_REPO}/cores/gemmini/include"
KCFLAGS="${KCFLAGS};-isystem${MB_REPO}/cores/gemmini"
KCFLAGS="${KCFLAGS};-DGEMMINI_ROCC;-DBAREMETAL"
KCFLAGS="${KCFLAGS};-DMODELBLASTER_GEMMINI_HWIO_WEIGHTS=1;-DMODELBLASTER_GEMMINI_Q31_ACC_SCALE=1"

echo "=== FC+DroNet build ==="
echo "  worktree = ${WT}"
echo "  MB_REPO  = ${MB_REPO}"
echo "  MODEL_DIR= ${MODEL_DIR}"
echo "  builddir = ${BUILDDIR}  irq_gate=${IRQ_GATE}"

cd "${MAIN}"
west build -p -b chipyard_riscv64 "${WT}" --build-dir "${BUILDDIR}" -- \
  -DMB_REPO="${MB_REPO}" \
  -DMODEL_DIR="${MODEL_DIR}" \
  -DMODELBLASTER_KERNEL_CFLAGS="${KCFLAGS}" \
  -DEXTRA_CPPFLAGS="${EXTRA_CPP}"
rc=$?
echo "=== BUILD EXIT ${rc} ==="
ls -la "${BUILDDIR}/zephyr/zephyr.elf" 2>&1
exit ${rc}
