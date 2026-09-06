#!/bin/bash
# Controlled A/B: does a MIG-bearing bitstream finish configuration on this carrier?
#
#   tools/bench/drone-ddr-check.sh <CONFIG_ADAPTER_LOC> <DEBUG_ADAPTER_LOC> [configs...]
#   tools/bench/drone-ddr-check.sh 7-1.1.4.1 7-1.1.4.3
#
# Get the two locations from `rb probe-ftdi --idcode`. They are NOT stable across
# sessions -- the bitstream and debug adapters have swapped between 7-1.1.4.1 and
# 7-1.1.4.3, so always re-probe rather than trusting a previous note.
set -u
ROOT=/scratch2/loren/riskybird-dev
CFG="$1"; DBG="$2"; shift 2
CONFIGS=("$@")
[ ${#CONFIGS[@]} -eq 0 ] && CONFIGS=(RocketArty200TSpadConfig RocketArty200TDroneDDRConfig \
                                     RocketArty200TSpadConfig RocketArty200TConfig \
                                     RocketArty200TSpadConfig RocketArty200TFtdiDDRConfig \
                                     RocketArty200TSpadConfig)
cd "$ROOT"
busdev() { echo "$(($(cat /sys/bus/usb/devices/$1/busnum))):$(($(cat /sys/bus/usb/devices/$1/devnum)))"; }

printf '%-32s %-8s %-12s %s\n' CONFIG LOAD STATUS "DEBUG TAP"
for c in "${CONFIGS[@]}"; do
    # A USB reset before every read. Without it the config-TAP read degrades
    # after the first OpenOCD run and even the known-good control returns
    # 0x5000190c, which makes any A/B worthless.
    python3 "$ROOT/tools/bench/usbreset.py" "$CFG" >/dev/null 2>&1
    python3 "$ROOT/tools/bench/usbreset.py" "$DBG" >/dev/null 2>&1
    sleep 4

    bit="$ROOT/backends/chipyard/fpga/generated-src/chipyard.fpga.arty200t.Arty200THarness.$c/obj/Arty200THarness.bit"
    if [ ! -f "$bit" ]; then printf '%-32s %s\n' "$c" "MISSING BITSTREAM"; continue; fi
    out=$(timeout 200 openFPGALoader --board te0712_8 --busdev-num "$(busdev "$CFG")" \
              --write-sram --bitstream "$bit" 2>&1 | tr -d '\0' | tr '\r' '\n')
    echo "$out" | grep -qa '^Done' && load=OK || load=FAIL

    status=$(timeout 40 openocd \
        -c "adapter driver ftdi" -c "ftdi vid_pid 0x0403 0x6010 0x0403 0x6011" \
        -c "adapter usb location $CFG" -c "ftdi channel 0" \
        -c "ftdi layout_init 0x0B00 0x0B0B" -c "adapter speed 1000" \
        -c "transport select jtag" -c "reset_config none" \
        -c "jtag newtap xc7 tap -irlen 6 -expected-id 0" \
        -c "pld device virtex2 xc7.tap 1" \
        -c "init" -c "virtex2 read_stat 0" -c "shutdown" 2>&1 \
        | grep -oE '0x[0-9a-f]{8}' | tail -1)

    tap=$(RB_FTDI_LOCATION="$DBG" timeout 60 openocd \
            -f hardware/openocd/ft-link.cfg -f hardware/openocd/arty200t.cfg \
            -c shutdown 2>&1 | grep -qE 'hart 0: XLEN=64' && echo ROCKET-OK || echo no-TAP)

    printf '%-32s %-8s %-12s %s\n' "$c" "$load" "${status:-(none)}" "$tap"
done
