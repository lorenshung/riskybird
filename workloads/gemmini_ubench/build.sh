#!/usr/bin/env bash
# Build the Gemmini conv microbenchmark ELF for the riskybird v3 FPGA Rocket
# combined config. Self-contained: board conf + DT overlay auto-apply from
# boards/chipyard_riscv64.{conf,overlay}. Usage: ./build.sh
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RB_ROOT="$(cd "$HERE/../.." && pwd)"
REPO="${ZCS:-$RB_ROOT/backends/zephyr-chipyard-sw}"   # west workspace + toolchain scripts
BUILDDIR="$HERE/build_gemmini_ubench"
cd "$REPO"
source scripts/activate_conda.sh
source scripts/set_envvars_sdk.sh
export PATH="/usr/bin:${PATH}"   # avoid stale Vitis cmake
echo "=== west build gemmini_ubench ==="
west build -p -b chipyard_riscv64 "$HERE" --build-dir "$BUILDDIR" 2>&1
echo "=== BUILD EXIT $? ==="
ls -la "$BUILDDIR/zephyr/zephyr.elf" 2>&1
