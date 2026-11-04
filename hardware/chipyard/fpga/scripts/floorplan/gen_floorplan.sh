#!/usr/bin/env bash
# gen_floorplan.sh -- one-shot generator for the FPGA PAR floorplan diagram.
#
# Opens a POST-ROUTE checkpoint in Vivado, re-derives the per-block partition
# (Gemmini / Saturn / Rocket / MIG / periphery) and each block's LUT/FF/BRAM/DSP
# via `report_utilization -cells`, then renders a labeled PNG with matplotlib.
# Nothing is hardcoded: the part / speed-grade and every tally come from the DCP,
# so it works on any config or grade (e.g. the -2 DCPs board.tcl now produces).
#
# Requirements (run on garden): Vivado on PATH (or --vivado-settings), and
# python3 with numpy + matplotlib.
#
# Usage:
#   gen_floorplan.sh --config <ConfigName> [--out <png>] [--outdir <dir>]
#   gen_floorplan.sh --dcp <post_route.dcp> [--out <png>] [--outdir <dir>]
#
# Options:
#   -c, --config NAME    Resolve the DCP from the chipyard build tree by config
#                        name: <chipyard>/fpga/generated-src/*<NAME>*/obj/post_route.dcp
#   -d, --dcp PATH       Use an explicit post_route.dcp (overrides --config).
#   -o, --out PATH       Output PNG (default: <outdir>/floorplan.png).
#       --outdir DIR     Intermediate/output dir (default: <dcp_dir>/floorplan,
#                        which lives under generated-src and is gitignored).
#       --chipyard DIR   Chipyard root (default: three levels up from this script).
#       --vivado-settings PATH  settingsNN.sh to source if vivado is not on PATH
#                        (default: /ecad/tools/xilinx/Vivado/2023.1/settings64.sh).
#       --note TEXT      Extra note appended to the plot subtitle.
#   -h, --help           Show this help.
#
# Example (the flashed combined core):
#   ./gen_floorplan.sh --config RocketArty200TDroneGemminiSaturnFp16At35Config \
#       --out ~/at35_floorplan.png
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHIPYARD="$(cd "$SCRIPT_DIR/../../.." && pwd)"
VIVADO_SETTINGS="/ecad/tools/xilinx/Vivado/2023.1/settings64.sh"
CONFIG=""; DCP=""; OUT=""; OUTDIR=""; NOTE=""

usage() { sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    -c|--config) CONFIG="$2"; shift 2;;
    -d|--dcp) DCP="$2"; shift 2;;
    -o|--out) OUT="$2"; shift 2;;
    --outdir) OUTDIR="$2"; shift 2;;
    --chipyard) CHIPYARD="$2"; shift 2;;
    --vivado-settings) VIVADO_SETTINGS="$2"; shift 2;;
    --note) NOTE="$2"; shift 2;;
    -h|--help) usage 0;;
    *) echo "unknown arg: $1" >&2; usage 1;;
  esac
done

# ---- resolve the DCP ----
if [[ -z "$DCP" && -n "$CONFIG" ]]; then
  mapfile -t HITS < <(ls -1 "$CHIPYARD"/fpga/generated-src/*"$CONFIG"*/obj/post_route.dcp 2>/dev/null || true)
  if [[ ${#HITS[@]} -eq 0 ]]; then
    echo "ERROR: no post_route.dcp for config '*$CONFIG*' under $CHIPYARD/fpga/generated-src" >&2; exit 1
  elif [[ ${#HITS[@]} -gt 1 ]]; then
    echo "ERROR: multiple DCPs match '*$CONFIG*' -- pass --dcp explicitly:" >&2
    printf '  %s\n' "${HITS[@]}" >&2; exit 1
  fi
  DCP="${HITS[0]}"
fi
if [[ -z "$DCP" ]]; then echo "ERROR: give --config NAME or --dcp PATH (see --help)" >&2; exit 1; fi
if [[ ! -f "$DCP" ]]; then echo "ERROR: DCP not found: $DCP" >&2; exit 1; fi

[[ -z "$OUTDIR" ]] && OUTDIR="$(dirname "$DCP")/floorplan"
[[ -z "$OUT" ]] && OUT="$OUTDIR/floorplan.png"
mkdir -p "$OUTDIR"

echo "== DCP:    $DCP"
echo "== outdir: $OUTDIR"
echo "== out:    $OUT"

# ---- Vivado on PATH? ----
if ! command -v vivado >/dev/null 2>&1; then
  if [[ -f "$VIVADO_SETTINGS" ]]; then
    # shellcheck disable=SC1090
    source "$VIVADO_SETTINGS"
  fi
fi
command -v vivado >/dev/null 2>&1 || { echo "ERROR: vivado not on PATH (try --vivado-settings)" >&2; exit 1; }

# ---- stage 1: extract from the checkpoint ----
vivado -mode batch -notrace -source "$SCRIPT_DIR/extract_floorplan.tcl" \
       -tclargs "$DCP" "$OUTDIR"

# ---- stage 2: plot ----
python3 "$SCRIPT_DIR/plot_floorplan.py" --indir "$OUTDIR" --out "$OUT" \
        ${NOTE:+--title-note "$NOTE"}

echo "== floorplan PNG: $OUT"
