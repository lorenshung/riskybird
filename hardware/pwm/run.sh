#!/usr/bin/env bash
# Verilate and run the motor PWM check against the RTL a given config elaborated.
#
# Takes the PWMTimer straight out of that build's gen-collateral, so what is
# measured is the RTL that goes into the bitstream rather than a copy of it.
#
#   hardware/pwm/run.sh [CONFIG]
#
# CONFIG defaults to RocketArty200TMotorSpadConfig. Elaborate it first with:
#   ./rb chipyard elaborate --board arty200t --config <CONFIG>
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CONFIG="${1:-RocketArty200TMotorSpadConfig}"
GEN="$ROOT/backends/chipyard/fpga/generated-src/chipyard.fpga.arty200t.Arty200THarness.$CONFIG/gen-collateral"
VERILATOR="${VERILATOR:-$ROOT/backends/chipyard/.conda-env/bin/verilator}"
OBJ="$ROOT/hardware/pwm/obj_dir"

if [[ ! -f "$GEN/PWMTimer.sv" ]]; then
    echo "no PWMTimer.sv under $GEN" >&2
    echo "run: ./rb chipyard elaborate --board arty200t --config $CONFIG" >&2
    exit 1
fi
if ! grep -q "io_gpio" "$GEN/PWMTimer.sv"; then
    echo "$CONFIG elaborated a PWMTimer with no gpio outputs." >&2
    echo "That shell has no PWM IOBinder, so firtool removed the output path." >&2
    exit 1
fi
if [[ ! -x "$VERILATOR" ]]; then
    echo "verilator not found at $VERILATOR (override with VERILATOR=)" >&2
    exit 1
fi

rm -rf "$OBJ"
"$VERILATOR" --cc --exe --build -j 4 \
    -Wno-fatal -Mdir "$OBJ" \
    --top-module PWMTimer \
    -o tb_motor_pwm \
    "$GEN/PWMTimer.sv" "$ROOT/hardware/pwm/tb_motor_pwm.cpp" >/dev/null

exec "$OBJ/tb_motor_pwm" "${@:2}"
