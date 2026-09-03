#!/usr/bin/env bash
#
# program_bitstream_ftdi_eeprom.sh
#   Reproducibly write the riskybird identity into the bitstream-programmer
#   FTDI's config EEPROM (the 93LC66C on riskybirdv3_connector) so the chip is
#   self-identifying: Manufacturer / Product / a unique Serial. Once serials
#   differ it is also tellable apart from the "binary" FTDI on the same board.
#
# USAGE
#   scripts/program_bitstream_ftdi_eeprom.sh [--detect] [--erase] [--yes]
#       [--mfg NAME] [--product STR] [--serial STR] [--device SPEC]
#
#   --detect     Locate the FTDI and print it; make NO changes (safe dry run).
#   --erase      Erase the EEPROM (recovery); reverts the chip to its blank
#                FT4232H default (0403:6011) after a power-cycle.
#   --yes        Skip the confirmation prompt (for automation).
#   --mfg/--product/--serial   Override the identity strings.
#   --device SPEC  ftdi_eeprom device selector when >1 FTDI is present, e.g.
#                  d:/dev/bus/usb/003/012   (see `lsusb` for Bus/Device).
#   Defaults also read from $RB_FTDI_MFG / $RB_FTDI_PRODUCT / $RB_FTDI_SERIAL.
#
# HARD-WON BACKGROUND (see FPGA_BRINGUP_LOG.md / memory riskybird-v3-fpga-bringup)
#   * The chip is FT4232H silicon REMARKED as FT2232HL. While the EEPROM is
#     blank it enumerates 0403:6011 (FT4232H); once programmed, libftdi builds a
#     2232H image and it enumerates 0403:6010 (FT2232H "Dual RS232-HS"). This is
#     EXPECTED -> afterwards use `openFPGALoader -c ft2232` (not ft4232).
#   * libftdi 1.5 mis-sizes a blank EEPROM at 128 B, so keep the strings short
#     (manufacturer+product+serial well under ~40 chars total).
#   * A blank (6011) chip usually needs the write applied twice: the first write
#     flips it to 6010, the second lands a valid image. This script auto-retries.
#   * After a good write, POWER-CYCLE the board (a USB replug may not be enough)
#     for the host to re-read the descriptors.
#
# SAFETY: this modifies a hardware EEPROM. It refuses to run when more than one
#   FTDI is present (so it can't program the wrong one) unless you pass --device.
#
set -euo pipefail

MFG="${RB_FTDI_MFG:-riskybird}"
PRODUCT="${RB_FTDI_PRODUCT:-riskybird bitstream}"
SERIAL="${RB_FTDI_SERIAL:-RBV3-BIT-01}"
MODE="program"; ASSUME_YES=0; OPEN_DEV=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --detect)  MODE="detect"; shift;;
    --erase)   MODE="erase";  shift;;
    --yes|-y)  ASSUME_YES=1;  shift;;
    --mfg)     MFG="$2";      shift 2;;
    --product) PRODUCT="$2";  shift 2;;
    --serial)  SERIAL="$2";   shift 2;;
    --device)  OPEN_DEV="$2"; shift 2;;
    -h|--help) sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'; exit 0;;
    *) echo "unknown arg: $1 (try --help)" >&2; exit 2;;
  esac
done

command -v ftdi_eeprom >/dev/null 2>&1 || { echo "ERROR: ftdi_eeprom not found (install libftdi)."; exit 1; }

# ---- locate the target FTDI ------------------------------------------------
lines() { { lsusb -d 0403:6010 2>/dev/null; lsusb -d 0403:6011 2>/dev/null; } | grep . || true; }
FTDI_LINES="$(lines)"; N="$(printf '%s\n' "$FTDI_LINES" | grep -c . || true)"

if [[ -z "$OPEN_DEV" ]]; then
  (( N == 0 )) && { echo "ERROR: no FTDI (0403:6010/6011) found. Plug in the bitstream programmer."; exit 1; }
  if (( N > 1 )); then
    echo "ERROR: $N FTDIs present - refusing to guess which is the bitstream one:"; echo "$FTDI_LINES"
    echo "Unplug the other FTDI, or pass --device d:/dev/bus/usb/BUS/DEV (Bus/Device from lsusb)."; exit 1
  fi
fi

PID="$(printf '%s\n' "$FTDI_LINES" | grep -oE '0403:60(10|11)' | head -1 | cut -d: -f2)"
echo ">> target FTDI:"; echo "$FTDI_LINES" | sed 's/^/     /'
echo ">> currently enumerates as 0403:${PID}  (6011=blank FT4232H, 6010=programmed FT2232H)"

# device selector for ftdi_eeprom
if [[ -n "$OPEN_DEV" ]]; then DEVSEL=(--device "$OPEN_DEV"); else DEVSEL=(--device "i:0x0403:0x${PID}"); fi

[[ "$MODE" == "detect" ]] && { echo ">> --detect: no changes made."; exit 0; }

WORK="$(mktemp -d)"; trap 'rm -rf "$WORK"' EXIT

if [[ "$MODE" == "erase" ]]; then
  (( ASSUME_YES )) || { read -r -p ">> ERASE this FTDI's EEPROM? [y/N] " a; [[ "$a" == [yY]* ]] || { echo "aborted."; exit 1; }; }
  printf 'vendor_id=0x0403\nproduct_id=0x%s\nfilename="%s/e.bin"\n' "$PID" "$WORK" > "$WORK/erase.conf"
  ftdi_eeprom "${DEVSEL[@]}" --erase-eeprom "$WORK/erase.conf"
  echo ">> erased. Power-cycle; the chip reverts to the FT4232H default (0403:6011)."; exit 0
fi

# ---- program ---------------------------------------------------------------
echo ">> identity:  manufacturer='$MFG'  product='$PRODUCT'  serial='$SERIAL'"
(( ASSUME_YES )) || { read -r -p ">> Write this identity to the FTDI above? [y/N] " a; [[ "$a" == [yY]* ]] || { echo "aborted."; exit 1; }; }

cat > "$WORK/prog.conf" <<EOF
vendor_id=0x0403
product_id=0x6010
max_power=500
manufacturer="$MFG"
product="$PRODUCT"
serial="$SERIAL"
use_serial=true
self_powered=false
remote_wakeup=false
filename="$WORK/eeprom.bin"
EOF

verify() {  # read the EEPROM back and confirm the serial string is physically present
  printf 'vendor_id=0x0403\nproduct_id=0x6010\nfilename="%s/rb.bin"\n' "$WORK" > "$WORK/rd.conf"
  ftdi_eeprom --device "i:0x0403:0x6010" --read-eeprom "$WORK/rd.conf" >/dev/null 2>&1 || return 1
  strings -el "$WORK/rb.bin" 2>/dev/null | grep -qF "$SERIAL"
}

ok=0
for attempt in 1 2; do
  echo ">> write attempt $attempt ..."
  ftdi_eeprom "${DEVSEL[@]}" --build-eeprom "$WORK/prog.conf" 2>&1 | grep -iE 'only contain|used eeprom' || true
  ftdi_eeprom "${DEVSEL[@]}" --flash-eeprom "$WORK/prog.conf" 2>&1 | grep -iE 'write eeprom' || true
  sleep 2
  DEVSEL=(--device "i:0x0403:0x6010")   # after any write the chip re-enumerates to 6010
  if verify; then echo ">> VERIFIED: '$SERIAL' present in the EEPROM image."; ok=1; break; fi
  echo ">> not verified yet; retrying..."
done
(( ok )) || { echo "!! EEPROM did not verify after 2 attempts. Try: $0 --erase, then re-run."; exit 1; }

cat <<EOF

DONE. The bitstream FTDI now carries the riskybird identity and enumerates as
FT2232H (0403:6010).
  1) POWER-CYCLE the board so the host re-reads the descriptors, then check:
       lsusb -v -d 0403:6010 | grep -E 'iManufacturer|iProduct|iSerial'
       cat /sys/bus/usb/devices/*/{manufacturer,product,serial}   # for 0403:6010
  2) Program bitstreams with:  openFPGALoader -c ft2232 ...
  3) Recover a bad EEPROM with: $0 --erase
EOF
