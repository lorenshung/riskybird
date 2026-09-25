#!/usr/bin/env bash
# One-shot bring-up check for a freshly swapped drone board.
#
# Programs the BARE-METAL pad test (no Rocket, no PWM IP, no Zephyr) to SRAM and
# reads the four motor gate nodes through the ESP32-C6 probe. That is the
# shortest path from "board is powered" to "is F13 alive on this board".
#
# SRAM only: nothing on the new board's SPI flash is touched.
# PROPELLERS OFF -- this drives all four gates continuously at 4/8/12/16%.
set -u
cd "$(dirname "$0")/../.." || exit 1

BIT=hardware/padtest/out/padtest.bit
say() { printf '\n\033[36m== %s ==\033[0m\n' "$*"; }

say "1. adapters"
./rb bench 2>&1 | head -8
BUSDEV=$(./rb bench 2>&1 | awk '$1=="bitstream"{print $3; exit}')
if [ -z "${BUSDEV:-}" ]; then
    echo "!! no bitstream-role FTDI found. Is the board powered and cabled?"
    exit 1
fi
echo "bitstream link -> $BUSDEV"

say "2. ESP firmware check (the probe only works if motor_safe is running)"
ESP=$(ls /dev/serial/by-id/*Espressif* 2>/dev/null | head -1)
if [ -z "${ESP:-}" ]; then
    ESP=/dev/ttyACM0
fi
echo "listening on $ESP for 6s ..."
stty -F "$ESP" 115200 raw -echo 2>/dev/null
ESPTXT=$(timeout 6 cat "$ESP" 2>/dev/null | tr -d '\r')
if echo "$ESPTXT" | grep -q "MOTOR GATE"; then
    echo "OK: motor_safe is running -- gate probe is usable"
    echo "$ESPTXT" | grep "MOTOR GATE" | tail -2
else
    echo "!! no 'MOTOR GATE' lines seen. Either the ESP is running other firmware,"
    echo "   or it is driving the gates itself -- readings below would be confounded."
    echo "   Flash motor_safe before trusting this:"
    echo "     cd $PWD && ./rb build motor_safe --target esp32c6 && ./rb flash motor_safe --target esp32c6"
    echo "   captured instead:"; echo "$ESPTXT" | tail -5
fi

say "3. programming bare-metal padtest to SRAM (flash untouched)"
openFPGALoader -b te0712_8 --busdev-num "$BUSDEV" --write-sram --bitstream "$BIT" 2>&1 \
    | grep -v 'Load SRAM: \[' | tail -4
sleep 4

say "4. gate probe  (expected on a healthy board: M1=40 M2=80 M3=120 M4=160)"
timeout 12 cat "$ESP" 2>/dev/null | tr -d '\r' | grep "MOTOR GATE" | tail -8

say "5. verdict"
LAST=$(timeout 8 cat "$ESP" 2>/dev/null | tr -d '\r' | grep "MOTOR GATE" | tail -1)
echo "$LAST"
case "$LAST" in
  *M4=0*)   echo ">> F13 DEAD on this board too. Two boards failing identically means this is"
            echo "   NOT a one-off defect -- reopen whether F13 is bonded to J1_100 on this"
            echo "   TE0712 module revision." ;;
  *M4=1[0-9][0-9]*)
            echo ">> F13 ALIVE on this board. The old board has a physical defect on the"
            echo "   /MOTOR4 net. The FPGA design is vindicated; run f13probe.bit on the OLD"
            echo "   board to localise what to repair." ;;
  *)        echo ">> inconclusive -- check M1=40 first; if M1 is not 40 the bitstream is not running." ;;
esac
