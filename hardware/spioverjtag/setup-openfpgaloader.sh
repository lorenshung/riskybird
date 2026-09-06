#!/usr/bin/env bash
# Set up openFPGALoader with the KU040 (xcku040-sfva784) flash support in this
# directory, on a machine that has neither.
#
# Nothing in openFPGALoader is patched. The only custom artifact is the
# spiOverJtag bridge bitstream sitting next to this script, so this is a build
# of upstream at a pinned commit plus a `-B` pointing at that file.
#
#   ./setup-openfpgaloader.sh            # deps check, clone, build
#   ./setup-openfpgaloader.sh --check-hw # also read the flash ID off the board
#
# Override the install location with OFL_PREFIX, the board with KU040_FTDI_SERIAL.
set -euo pipefail

# Default under the repository rather than one particular user's home, so a
# fresh clone installs somewhere that exists. Override with OFL_PREFIX.
_ofl_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PREFIX="${OFL_PREFIX:-${_ofl_root}/.rb/openFPGALoader}"
PIN="${OFL_COMMIT:-63c519c3c57ded2d8cc412197d8b781115b1922a}"
REPO=https://github.com/trabucayre/openFPGALoader.git
SERIAL="${KU040_FTDI_SERIAL:-TE0000991948}"
PART=xcku040-sfva784

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE="$HERE/spiOverJtag_${PART}.bit.gz"
OFL="$PREFIX/build/openFPGALoader"

step() { printf '\n== %s ==\n' "$*"; }
die()  { printf 'error: %s\n' "$*" >&2; exit 1; }

step "dependencies"
# Use /usr/bin/cmake explicitly: Vivado and Vitis both put a cmake 3.3.2 ahead
# of it on PATH which dies with "libidn.so.11: cannot open shared object file".
CMAKE=/usr/bin/cmake
missing=()
for pc in libftdi1 libusb-1.0 zlib; do
	pkg-config --exists "$pc" 2>/dev/null || missing+=("$pc")
done
[ -x "$CMAKE" ] || missing+=(cmake)
command -v g++ >/dev/null || missing+=(g++)
command -v git >/dev/null || missing+=(git)
if [ ${#missing[@]} -ne 0 ]; then
	echo "missing: ${missing[*]}"
	echo "  sudo apt-get install libftdi1-dev libhidapi-dev libusb-1.0-0-dev zlib1g-dev cmake g++ git"
	die "install the above and re-run"
fi
echo "ok: $($CMAKE --version | head -1), libftdi1 $(pkg-config --modversion libftdi1)"

[ -f "$BRIDGE" ] || die "bridge bitstream missing: $BRIDGE"
echo "ok: bridge $(basename "$BRIDGE") ($(stat -c%s "$BRIDGE") bytes)"

step "source at $PREFIX"
if [ -d "$PREFIX/.git" ]; then
	echo "checkout exists; fetching"
	git -C "$PREFIX" fetch -q --depth 200 origin || true
else
	mkdir -p "$(dirname "$PREFIX")"
	git clone -q "$REPO" "$PREFIX"
fi
git -C "$PREFIX" checkout -q "$PIN" 2>/dev/null \
	|| die "cannot check out pinned commit $PIN (fetch may be too shallow)"
echo "pinned at $(git -C "$PREFIX" log -1 --format='%h %cs')"

step "build"
# A CMakeCache.txt records the absolute path it was generated in, so a build
# tree that has been copied or moved fails until the cache is discarded.
if [ -f "$PREFIX/build/CMakeCache.txt" ] &&
   ! grep -qx "CMAKE_HOME_DIRECTORY:INTERNAL=$PREFIX" "$PREFIX/build/CMakeCache.txt"; then
	echo "stale cmake cache from another path; wiping build dir"
	rm -rf "$PREFIX/build"
fi
"$CMAKE" -S "$PREFIX" -B "$PREFIX/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
"$CMAKE" --build "$PREFIX/build" -j"$(nproc)" >/dev/null
[ -x "$OFL" ] || die "build produced no binary at $OFL"
echo "built: $("$OFL" --Version)"

# 0.12.0 loads the bridge but its UltraScale SPI path does not work: every
# JEDEC read comes back 0xff. Anything at or below that is not usable here.
case "$("$OFL" --Version | grep -oE 'v[0-9.]+')" in
	v0.*) die "built $("$OFL" --Version); need > v0.12.0 for UltraScale flash" ;;
esac

step "usage"
cat <<USAGE
Read the flash ID (safe, read-only):
  sudo $OFL -c ft2232 --ftdi-serial $SERIAL \\
    --fpga-part $PART -B $BRIDGE \\
    --dump-flash --file-size 4096 --bitstream /tmp/head.bin

Write a bitstream to flash (destructive, always with --verify):
  sudo $OFL -c ft2232 --ftdi-serial $SERIAL \\
    --fpga-part $PART -B $BRIDGE \\
    --verify -f --bitstream <path-to.bit>

Load to SRAM instead (volatile, no flash touched):
  sudo $OFL -c ft2232 --ftdi-serial $SERIAL -m --bitstream <path-to.bit>

sudo is needed unless you are in the plugdev group: the udev rule tags these
devices TAG+="uaccess", which only grants an ACL on a local seat, so over SSH
group membership is the only path. Fix with:
  sudo usermod -aG plugdev \$USER   # then log in again
USAGE

if [ "${1:-}" = "--check-hw" ]; then
	step "reading flash ID from the board"
	out=$(sudo "$OFL" -c ft2232 --ftdi-serial "$SERIAL" --fpga-part "$PART" \
		-B "$BRIDGE" --dump-flash --file-size 4096 \
		--bitstream /tmp/ku040-flash-head.bin 2>&1 | tr -d '\000' | tr '\r' '\n')
	echo "$out" | grep -E 'SOJ version|JEDEC ID|Detected' || true
	if echo "$out" | grep -q 'JEDEC ID: 0xff'; then
		die "JEDEC ID reads 0xff -- flash not responding"
	fi
	echo "$out" | grep -q 'JEDEC ID' || die "no JEDEC ID in output"
	echo "PASS: flash responds"
fi
