#!/usr/bin/env bash
# The hardware acceptance check from docs/setup.md, as one command.
#
#   tools/bench/acceptance.sh <bitstream-location> <debug-location>
#   tools/bench/acceptance.sh 7-1.1.4.3 7-1.1.4.1
#
# Get the two locations from `rb probe-ftdi --idcode`. They change on every
# replug, so read them rather than trusting a previous note.
#
# Runs the six steps in order and stops at the first failure, because each one
# is a precondition for the next: there is no point testing DDR3 on a shell that
# did not configure. Every step prints what it proved or what its failure means.
#
# NEEDS NO BATTERY AND NO PROPELLERS. Nothing here drives a motor. The full
# drone shell does drive the motor pins from configuration onward, but at 0%
# duty -- see docs/motor-pwm-bringup.md.
#
# NOT YET RUN AGAINST HARDWARE. Written from the documented sequence while the
# bench was unavailable; treat the first run as testing this script too.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

if [ $# -lt 2 ]; then
    sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit 2
fi
BIT="$1"; DBG="$2"
CFG="${RB_ACCEPT_CONFIG:-RocketArty200TDroneFullDDRConfig}"
LOG="${RB_ACCEPT_LOG:-$ROOT/.rb/acceptance}"
mkdir -p "$LOG"

pass=0; fail=0
step() {   # step <name> <why-it-matters> <command...>
    local name="$1" why="$2"; shift 2
    printf '\n=== %s ===\n' "$name"
    if "$@" > "$LOG/${name// /-}.log" 2>&1; then
        printf '  PASS  %s\n' "$why"; pass=$((pass+1)); return 0
    fi
    printf '  FAIL  %s\n' "$why"
    printf '        log: %s\n' "$LOG/${name// /-}.log"
    tail -5 "$LOG/${name// /-}.log" | sed 's/^/        /'
    fail=$((fail+1)); return 1
}

debug() { ./rb debug "$1" --target arty200t --config "$CFG" \
            --program --program-usb-location "$BIT" --usb-location "$DBG" "${@:2}"; }

echo "RiskyBird hardware acceptance check"
echo "  config    $CFG"
echo "  bitstream adapter $BIT   debug adapter $DBG"
echo "  logs      $LOG"

step "adapters" "both TAPs answer; 0x13636093 is the config TAP, 0x00000001 Rocket's" \
     ./rb probe-ftdi --idcode || exit 1

step "configure" "the shell configures; a MIG stall here is input voltage, not the cable" \
     ./rb program --board arty200t --config "$CFG" --usb-location "$BIT" || exit 1

step "hello_world" "code runs and the console talks -- touches nothing else" \
     timeout 120 debug hello_world || exit 1

step "ddr_stress" "the DDR3 is real, not merely that the shell configured" \
     timeout 900 debug ddr_stress

step "sensor_check" "the I2C bus and the on-board sensors answer" \
     timeout 300 debug sensor_check

step "camera_photo" "a workload drives real hardware end to end" \
     timeout 300 debug camera_photo --gdb-script hardware/ospi/capture-frame.gdb

printf '\n=== summary ===\n  %d passed, %d failed\n' "$pass" "$fail"
if [ -f camera-frame.png ]; then
    printf '  camera-frame.png written -- open it, that is the end-to-end proof\n'
fi
[ "$fail" -eq 0 ] || exit 1
