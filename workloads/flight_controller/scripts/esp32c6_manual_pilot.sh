#!/bin/bash
# Build / flash the standalone ESP32-C6 manual-pilot flight controller (riskybird v3, no FPGA) with the
# configuration flown 2026-11-21. See docs/ESP32C6_MANUAL_PILOT_TUNING.md for how each flag was found.
#
#   usage: esp32c6_manual_pilot.sh build|flash|both
#
# Environment (defaults are the original bench machine's layout):
#   ZCS        zephyr-chipyard-sw checkout providing west + SDK env (scripts/activate_conda.sh,
#              scripts/set_envvars_sdk.sh) and samples/drone_control/tinympc
#   BUILD_DIR  west build directory
#   PANEL_URL  ground-station panel, used by the flash guard (default http://127.0.0.1:8080)
#   FORCE_FLASH=1  skip the flight guard (only when you know the drone is not flying)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
RB_ROOT="$(cd "$HERE/../../.." && pwd)"
ZCS="${ZCS:-$HOME/Tools/zephyr-chipyard-sw}"
BUILD_DIR="${BUILD_DIR:-$HOME/Tools/rb_relay_build/tree1_esp32c6_mp}"
PANEL_URL="${PANEL_URL:-http://127.0.0.1:8080}"

# ---- flight configuration (plain #defines -> EXTRA_CPPFLAGS) -------------------------------------
FLAGS=(
  # build mode: optical flow, manual pilot from the panel / Deck, ANGLE stick + ALTITUDE throttle
  -DROSE_FLOW=1 -DROSE_MANUAL_PILOT=1 -DROSE_VEL_HEADING_FRAME=1
  -DROSE_PILOT_ANGLE=1 -DPILOT_ANGLE_MAX_RAD=0.70f   # full stick = 40 deg bank
  -DROSE_PILOT_ALT=1
  # watchdog relaxed to "complete flip only" (deadman, B / throttle-down ESTOP are the real cuts).
  # SAFE_IMPACT/FREEFALL off: raw-|a| checks tripped on full-throttle motor vibration.
  -DSAFE_MAX_TILT_RAD=2.8f -DSAFE_ACC_TILT_RAD=2.8f -DSAFE_MAX_RATE_RADPS=1000.0f
  -DSAFE_MAX_IMU_MISS=1000000 -DSAFE_MAX_VEL_MPS=1000.0f -DSAFE_MAX_HEIGHT_M=1000.0f
  -DSAFE_IMPACT_MPS2=1000.0f -DSAFE_FREEFALL_MPS2=0.0f
  # motors: per-motor duty feed-forward (FR, BR, BL, FL) from the measured steady-hover duty shares;
  # the 0.87 left-pair trim of the original airframe is disabled. Yaw authority clamp 30%.
  -DMOTOR_TRIM_LEFT=1.0f
  -DMOTOR_DUTY_TRIM_0=1.061f -DMOTOR_DUTY_TRIM_1=0.882f -DMOTOR_DUTY_TRIM_2=1.090f -DMOTOR_DUTY_TRIM_3=0.967f
  -DYAW_AUTH_FRAC=0.3f
  # attitude integral (learns the moment trim with the stick centred, kept across landings)
  -DKI_ATT=1.0f -DATT_INT_STICK=0.035f -DATT_INT_MAX=0.15f -DATT_INT_KEEP=1
  # velocity hold: stick centred and above 8 cm -> the flow velocity loop brakes to zero
  -DROSE_PILOT_VEL_HOLD=1 -DPILOT_HOLD_MIN_H=0.08f
  -DKI_VEL=0.8f -DVEL_TC=1.0f -DVEL_TILT_MAX=0.13f
  -DFLOW_LP_TAU_X=0.05f -DFLOW_LP_TAU_Y=0.15f -DFLOW_GYRO_LP_MATCH=1
  # altitude: damping on the filtered height derivative (the estimator vz is unusable in flight)
  -DPID_MASS_KG=0.045f -DALT_NATFREQ=2.5f -DALT_DAMPING=0.8f -DKI_HEIGHT=0.6f
  -DALT_VZ_FROM_Z=1 -DALT_ACC_MAX_G=1.15f
)

env_setup() {
  cd "$ZCS" || { echo "ZCS=$ZCS not found"; exit 2; }
  # shellcheck disable=SC1091
  source scripts/activate_conda.sh >/dev/null 2>&1; source scripts/set_envvars_sdk.sh >/dev/null 2>&1
  export PATH=/usr/bin:$PATH
}

do_build() {
  env_setup
  # MOTOR_MAX_DUTY / MOTOR_DUTY_CEILING / ROSE_MOTOR_WINDOW_MS are CMake CACHE vars (CMakeLists
  # set(... CACHE) -> target_compile_definitions): pass them as -D CMake args, NOT in EXTRA_CPPFLAGS,
  # or they double-define against their defaults. 86400000 ms = the motor-live window never closes
  # within a session (it re-arms on every board reset). -p always so the cache takes fresh values.
  west build -b esp32c6_devkitc/esp32c6/hpcore -d "$BUILD_DIR" -p always \
    "$RB_ROOT/workloads/flight_controller" -- \
    -DRB_ROOT="$RB_ROOT" -DDTS_ROOT="$RB_ROOT/hardware/zephyr" \
    -DTINYMPC="$ZCS/samples/drone_control/tinympc" \
    -DROSE_USE_EKF=0 -DROSE_BARO=1 -DROSE_USE_PID=1 -DCTRL_ITERS=0 \
    -DMOTOR_MAX_DUTY=1.0f -DMOTOR_DUTY_CEILING=0.75f -DROSE_MOTOR_WINDOW_MS=86400000 \
    -DEXTRA_CONF_FILE="telem.conf" -DEXTRA_DTC_OVERLAY_FILE="telem.overlay" \
    -DEXTRA_CPPFLAGS="${FLAGS[*]}" 2>&1 | tail -20
}

do_flash() {
  # FLIGHT GUARD: flashing reboots the FC. Refuse if the panel shows motors above idle or live stick
  # commands (2026-11-17 a flash reset the drone mid-takeoff).
  if [ "${FORCE_FLASH:-0}" != 1 ]; then
    busy=$(curl -s --max-time 4 "$PANEL_URL/stream" 2>/dev/null | grep -m1 '"link"' | python3 -c '
import sys,json
try: d=json.loads(sys.stdin.read().replace("data: ",""))
except Exception: print(""); sys.exit()
t=d.get("t") or {}; p=d.get("pilot") or {}
u=t.get("u") or [-0.583]*4
why=[]
if max(u)+0.583 > 0.03: why.append("motors running (duty %.2f)"%(max(u)+0.583))
if p.get("engaged") and (p.get("age_ms") or 1e9) < 10000: why.append("stick commands live (%d ms ago)"%p["age_ms"])
print("; ".join(why))' 2>/dev/null)
    if [ -n "$busy" ]; then
      echo "!! NOT FLASHING: $busy -- the pilot may be flying. Land/disengage, or FORCE_FLASH=1."; exit 3
    fi
  fi
  env_setup
  NODE=$(ls /dev/ttyACM* 2>/dev/null | head -1)
  [ -n "$NODE" ] || { echo "no /dev/ttyACM* (is USB connected?)"; exit 4; }
  # esptool needs the port: stop anything holding it (e.g. a console reader). Kills by PID only.
  for p in $(fuser "$NODE" 2>/dev/null); do kill "$p" 2>/dev/null; done; sleep 1
  echo "flashing standalone ESP FC on $NODE ..."
  west flash -d "$BUILD_DIR" --esp-device "$NODE" 2>&1 | tail -6
}

case "${1:-}" in
  build) do_build ;;
  flash) do_flash ;;
  both)  do_build && do_flash ;;
  *) echo "usage: $0 build|flash|both"; exit 1 ;;
esac
