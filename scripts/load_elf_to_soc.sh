#!/usr/bin/env bash
#
# load_elf_to_soc.sh — load + run a RISC-V ELF on the riskybird FPGA Rocket SoC over OpenOCD/JTAG.
#
# Prereq: the FPGA is already configured with a Rocket shell whose memory matches the ELF
#   (32 KiB scratchpad vs 1 GiB DDR — see the elf repo manifest / FPGA_BRINGUP_LOG.md) and the
#   "binary" FTDI (0403:6010) is connected. The console is that FTDI's channel 1 = /dev/ttyUSB1.
#
# USAGE
#   scripts/load_elf_to_soc.sh [options] <image.elf>
#   scripts/load_elf_to_soc.sh --halt              # just check the core is alive (prints misa/pc)
#
# OPTIONS
#   --entry ADDR     PC to resume at (default 0x80000000 — the RAM/scratchpad/DDR base)
#   --speed KHZ      JTAG clock (default 2000)
#   --location B:P   USB location to select one FTDI when both are plugged (see `lsusb -t`)
#   --halt           halt + print misa/pc only; no load/resume ("is the Rocket alive?")
#   --reset          `reset halt` before loading (else just `halt`)
#   --console        after resume, stream the SoC console until Ctrl-C (opened BEFORE resume so the
#                    boot banner is captured)
#   --console-dev D  console device (default /dev/ttyUSB1);  --baud N (default 115200)
#   --cfg FILE       OpenOCD config (default: scripts/openocd/arty200t_rocket.cfg next to this)
# ENV: OPENOCD (binary name/path), FTDI_LOCATION, FTDI_SPEED.
#
# EXAMPLES
#   scripts/load_elf_to_soc.sh --console .../elf/arty200t/arty200t.ddr_stress.default.elf
#   scripts/load_elf_to_soc.sh --halt
#
set -euo pipefail
HERE="$(cd "$(dirname "$(realpath "$0")")" && pwd)"
CFG="$HERE/openocd/arty200t_rocket.cfg"
OPENOCD="${OPENOCD:-openocd}"
ENTRY=0x80000000; SPEED=2000; LOCATION=""; MODE=load; DO_CONSOLE=0
CONSOLE_DEV="${CONSOLE_DEV:-/dev/ttyUSB1}"; CONSOLE_BAUD="${CONSOLE_BAUD:-115200}"
ELF=""

while [[ $# -gt 0 ]]; do case "$1" in
  --entry)       ENTRY="$2"; shift 2;;
  --speed)       SPEED="$2"; shift 2;;
  --location)    LOCATION="$2"; shift 2;;
  --halt)        MODE=halt; shift;;
  --reset)       MODE=reset; shift;;
  --console)     DO_CONSOLE=1; shift;;
  --console-dev) CONSOLE_DEV="$2"; shift 2;;
  --baud)        CONSOLE_BAUD="$2"; shift 2;;
  --cfg)         CFG="$2"; shift 2;;
  -h|--help)     awk 'NR>1{if(/^#/){sub(/^# ?/,"");print}else exit}' "$0"; exit 0;;
  -*)            echo "unknown option: $1 (try --help)" >&2; exit 2;;
  *)             ELF="$1"; shift;;
esac; done

command -v "$OPENOCD" >/dev/null 2>&1 || { echo "ERROR: openocd not found (set \$OPENOCD)."; exit 1; }
[[ -f "$CFG" ]] || { echo "ERROR: OpenOCD config not found: $CFG"; exit 1; }
export FTDI_SPEED="$SPEED"
[[ -n "$LOCATION" ]] && export FTDI_LOCATION="$LOCATION"

# quick liveness check: halt + dump misa/pc, no load
if [[ "$MODE" == halt ]]; then
  exec "$OPENOCD" -f "$CFG" -c "init; halt; echo {--- Rocket halted ---}; reg pc; reg misa; shutdown"
fi

[[ -n "$ELF" ]] || { echo "ERROR: no ELF given (see --help)."; exit 2; }
[[ -f "$ELF" ]] || { echo "ERROR: ELF not found: $ELF"; exit 1; }
ELF="$(realpath "$ELF")"

PRE="init; halt"
[[ "$MODE" == reset ]] && PRE="init; reset halt"
OOCMD="$PRE; load_image {$ELF}; echo {--- loaded, resuming @ $ENTRY ---}; resume $ENTRY; shutdown"

echo ">> loading $ELF -> Rocket (entry $ENTRY, JTAG ${SPEED}kHz)"
if (( DO_CONSOLE )); then
  # open the console BEFORE resume so the boot banner is captured; Ctrl-C to stop
  stty -F "$CONSOLE_DEV" "$CONSOLE_BAUD" raw -echo 2>/dev/null || echo "(warn: could not stty $CONSOLE_DEV)"
  cat "$CONSOLE_DEV" & CATPID=$!
  trap 'kill "$CATPID" 2>/dev/null || true' EXIT
  "$OPENOCD" -f "$CFG" -c "$OOCMD"
  echo ">> running — streaming $CONSOLE_DEV @ $CONSOLE_BAUD (Ctrl-C to exit)"
  wait "$CATPID"
else
  "$OPENOCD" -f "$CFG" -c "$OOCMD"
  echo ">> loaded + resumed. Watch the console:  cat $CONSOLE_DEV   (or re-run with --console)"
fi
