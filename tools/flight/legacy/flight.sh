#!/usr/bin/env bash
# RiskyBird gentle-hover prep. Run from the Linux bench, LiPo OUT until told otherwise.
#   flight.sh sram     program FullDDR into SRAM only (after any power cycle: the flash may hold another shell)
#   flight.sh load     load the flight image fresh and print what it booted as
#   flight.sh dryrun   PACK OUT: enable arming over JTAG, let the profile run on the wire, report
#   flight.sh commit   load fresh again, then write the commit key -> ready for the dashboard
set -u
SP=/tmp/claude-2629/-scratch2-loren-riskybird-dev/f44ab62f-6a4c-49ac-99d7-4370b563a1bb/scratchpad
R=/scratch2/loren/riskybird-dev-setup
PY=$R/backends/zephyr-chipyard-sw/tools/miniforge3/envs/zephyr/bin/python3
DBG=7-1.1.4.3
FC=$SP/fpga-console.log
ESP=$SP/esp-console.log

cd "$R" && source ./rb-env.sh >/dev/null 2>&1
export RB_CMAKE_ARGS="-DROSE_AUTOFLIGHT=1 -DROSE_ARM_NO_GESTURE=1 -DROSE_UART_CMD=1 -DROSE_FLIGHT_COMMIT=1 -DROSE_COMMIT_MAX_MS=60000 -DAUTOFLIGHT_MAX_DUTY=0.95f -DPID_MASS_KG=0.050f -DARM_MAX_HEIGHT_M=0.05f -DHOVER_Z_M=1.60f -DROSE_TELEM_DIV=40 -DROSE_BATT_REPORT_ONLY=0"
export RB_EXTRA_CPPFLAGS="-DT_CLIMB_MS=5000 -DT_HOVER_MS=1000 -DT_DESCEND_MS=5000 -DFLIGHT_MAX_MS=15000 -DARM_MAX_TILT_RAD=0.087f -DROSE_TELEM_CONSOLE=0 -DFLOW_LP_TAU_X=0.05f -DFLOW_LP_TAU_Y=0.15f -DVL53L1X_TIMING_BUDGET_US=100000 -DIMU_OFFSET_Y=0.016f -DSAFE_MAX_HEIGHT_M=2.2f -DTOF_STEP_REJECT_M=0.25f -DGYRO_STEP_MAX=5.0f -DPITCH_TRIM_RAD=0.045f -DBATT_NOMINAL_V=3.8f -DBATT_ARM_MIN_V=3.95f -DATT_GAIN=0.85f -DALT_INT_BAND=0.10f -DSAFE_MAX_VEL_MPS=2.0f -DBATT_CUTOFF_V=3.0f -DYAW_AUTH_FRAC=0.15f -DROSE_VEL_LOOP=1 -DVEL_TILT_MAX=0.08f -DMOTOR_TRIM_LEFT=1.0f"

loggers() {   # (re)start both console loggers; opening the ESP port resets the ESP once
	pkill -f "$SP/fpga-console.py" 2>/dev/null; pkill -f "$SP/esp-console.py" 2>/dev/null; sleep 0.5
	: > "$FC"; : > "$ESP"
	nohup "$PY" "$SP/fpga-console.py" "$FC" >/dev/null 2>&1 &
	nohup "$PY" "$SP/esp-console.py" "$ESP" >/dev/null 2>&1 &
	sleep 3
}
clean() { tr -d '\r' < "$1" | tr -cd '\11\12\40-\176'; }

ELF=$R/build/zephyr/arty200t/flight_controller/esp-motors/zephyr/zephyr.elf
elfsum() { sha1sum "$ELF" | cut -c1-12; }

rbdebug() {   # $1 = gdb script, rest = extra rb flags; log kept per script
	local s=$1; shift
	local log="$SP/rbdebug-$(basename "$s" .gdb).log"
	timeout 300 ./rb debug flight_controller --target arty200t \
		--config RocketArty200TDroneFullDDRConfig --mode esp-motors \
		--usb-location "$DBG" --gdb-script "$s" "$@" > "$log" 2>&1
	local rc=$?
	grep -a "=== \|g_\|detached\|rror\|load size\|Start address" "$log" | grep -v "^Info" | head -20
	echo "rb debug exit=$rc"
	return $rc
}

fresh_boot() {   # THIS load printed its banner, and the ESP is receiving frames from it
	# (ROSE_TELEM_CONSOLE=0: telemetry no longer goes to the console, only to the ESP)
	clean "$FC" | grep -aq "COMMIT GATE -- build is COMMIT-CAPABLE" || return 1
	clean "$ESP" | grep -a RBLINK | tail -1 | grep -q "ml=up"
}

report_boot() {
	if ! fresh_boot; then
		echo; echo "!!!! NO-GO: no boot banner at 115200, or the ESP is not getting frames (ml!=up). Wrong shell/clock (the SPI flash may"
		echo "!!!! hold another config) or the core is not running. Run: flight.sh sram, then commit."
	fi
	echo; echo "---- flight controller banners ----"
	clean "$FC" | grep -a "PROFILE/\|COMMIT GATE\|DUTY CHAIN\|AUTOFLIGHT\|REPORT-ONLY\|BELOW THE\|VL53L1X\|uplink\|UART_CMD" | cut -c1-170
	echo; echo "---- last telemetry line ----"
	clean "$FC" | grep -a "^flight_controller: it=" | tail -1 | cut -c1-420
	echo; echo "---- ESP link (want ml=up, fr climbing, crc/noise ~flat, kill=0) ----"
	clean "$ESP" | grep -a "duty ceiling\|RBLINK" | tail -3
}


esp_flash() {   # the 98% esp_bridge motors image; opening the port afterwards resets the ESP once
	pkill -f "$SP/esp-console.py" 2>/dev/null; sleep 0.5
	( export PATH=$R/backends/zephyr-chipyard-sw/tools/miniforge3/envs/zephyr/bin:$PATH \
	    ZEPHYR_BASE=$R/backends/zephyr-chipyard-sw/zephyr_ws/zephyr \
	    ZEPHYR_SDK_INSTALL_DIR=$R/backends/zephyr-chipyard-sw/tools-manual/zephyr-sdk-1.0.0-beta1
	  cd $R/backends/zephyr-chipyard-sw/zephyr_ws && \
	  west flash -d $R/build/zephyr/esp32c6/esp_bridge/motors-cap98 -- --esp-device /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_98:A3:16:B8:56:74-if00 ) \
		> "$SP/esp-flash98.log" 2>&1
	local rc=$?
	grep -a "verified\|rror" "$SP/esp-flash98.log" | tail -2; echo "esp flash exit=$rc"
	return $rc
}

shell_up() {   # the Rocket debug TAP answers only when FullDDR is configured and out of reset
	timeout 120 ./rb bench 2>/dev/null | awk -v d="$DBG" '$2==d{print}' | grep -q "Rocket debug TAP"
}

link_rate() {   # motor frames per second the ESP is receiving, over ~2 s of RBLINK lines
	clean "$ESP" | grep -a RBLINK | tail -11 | sed -n 's/.* up=\([0-9]*\) .* fr=\([0-9]*\) .*/\1 \2/p' | \
		awk 'NR==1{u0=$1;f0=$2} {u=$1;f=$2} END{ if (u>u0) printf "motor frames: %.0f /s (was ~57 before the fix; target ~200)\n", (f-f0)*1000/(u-u0); else print "motor frames: no data" }'
	clean "$ESP" | grep -a RBLINK | tail -1
}

case "${1:-}" in
sram)
	# FullDDR into SRAM ONLY -- leaves whatever another session put in the SPI flash alone.
	loc=$(timeout 120 ./rb bench 2>/dev/null | awk '$1=="bitstream"{print $3}')
	[ -n "$loc" ] || { echo "no bitstream adapter found by rb bench"; exit 1; }
	echo "bitstream adapter bus:dev = $loc"
	openFPGALoader -b te0712_8 --busdev-num "$loc" --write-sram \
		--bitstream "$HOME/riskybird-bitstreams/arty200t/arty200t.RocketArty200TDroneFullDDRConfig.bit" \
		2>&1 | grep -a "Done\|rror\|DONE" | tail -3
	;;
go)
	echo "PACK MUST BE OUT."
	esp_flash || exit 1
	if shell_up; then echo "FullDDR shell is up"; else
		echo "FullDDR shell not answering -> programming SRAM"
		"$0" sram
		sleep 3
		shell_up || { echo "NO-GO: FullDDR still not answering after SRAM program"; exit 1; }
	fi
	loggers
	rbdebug "$SP/freerun.gdb" || exit 1
	sleep 8
	report_boot
	echo; echo "---- ESP ceiling ----"; clean "$ESP" | grep -a "duty ceiling" | tail -1
	link_rate
	;;
imu)
	# IMU sign test on THIS path (FPGA reads the BMI088). --mode default: no motor code at all.
	# Expect, per the 2026-08-06 ESP-build check: level a~(0,0,+9.6); NOSE-UP ax>0 and gy<0 while
	# moving; RIGHT-WING-DOWN ay>0 and gx>0 while moving; NOSE-LEFT (yaw) gz>0 while turning.
	"$0" sram || exit 1
	sleep 3
	loggers
	RB_CMAKE_ARGS="-DROSE_USE_PID=1 -DROSE_USE_EKF=0 -DCTRL_ITERS=0 -DROSE_TELEM_DIV=100" \
	RB_EXTRA_CPPFLAGS="-DROSE_IMU_DEBUG=1" \
	timeout 300 ./rb debug flight_controller --target arty200t \
		--config RocketArty200TDroneFullDDRConfig --mode default \
		--usb-location "$DBG" --gdb-script "$SP/freerun.gdb" \
		> "$SP/rbdebug-imu.log" 2>&1
	echo "rb debug exit=$?"
	sleep 5
	clean "$FC" | grep -a "iter=" | tail -3
	echo "now tilt it; watch with:  tail -f $FC | grep --line-buffered iter="
	;;
load)
	loggers
	rbdebug "$SP/freerun.gdb" || exit 1
	sleep 8
	report_boot
	;;
dryrun)
	echo "PACK MUST BE OUT. Enabling arming; it arms ~3 s after it is level+still, then runs"
	echo "the profile on the wire for ~7 s. Nothing can turn without the pack."
	rbdebug "$R/hardware/flight/arm-enable.gdb" --no-load || exit 1
	sleep 16
	echo; echo "---- autoflight / gate events ----"
	clean "$FC" | grep -a "AUTOFLIGHT\|MOTOR GATE\|ENVELOPE\|ESTOP\|estop" | grep -v "ENVELOPE GUARD --" | cut -c1-200
	echo; echo "---- duty seen by the FC (every telemetry line while nonzero) ----"
	clean "$FC" | grep -a "^flight_controller: it=" | grep -ao "duty=\[[^]]*\].*zsp=[^ ]*\|st=[0-9]*" | paste - - | grep -v "duty=\[0.000 0.000 0.000 0.000\]" | head -40
	echo; echo "---- ESP (max duty applied, cuts, kill) ----"
	clean "$ESP" | grep -a RBLINK | awk '{print $3,$6,$7,$8,$9,$15,$16,$18}' | grep -v "duty=\[0 0 0 0\]" | head -30
	clean "$ESP" | grep -a RBLINK | tail -1
	;;
commit)
	# a plain reload leaves the SoC peripherals as the old image left them; measured 2026-09-23 it
	# hung the new image in its first uart_poll_out. Always start from a fresh SRAM program.
	"$0" sram || exit 1
	sleep 3
	loggers
	loaded=$(elfsum); echo "loading ELF $loaded"
	rbdebug "$SP/freerun.gdb" || exit 1
	[ "$(elfsum)" = "$loaded" ] || { echo "ELF was rebuilt during the load ($loaded -> $(elfsum)); rerun."; exit 1; }
	sleep 8
	report_boot
	link_rate
	for _i in 1 2 3 4 5 6 7 8 9 10; do fresh_boot && break; sleep 1; done   # the ESP may still be re-enumerating
	fresh_boot || { echo "NOT committing: no banner/fresh telemetry from THIS load (see $SP/rbdebug-freerun.log)."; exit 1; }
	[ "$(elfsum)" = "$loaded" ] || { echo "NOT committing: ELF changed since it was loaded ($loaded -> $(elfsum))."; exit 1; }
	echo; echo "---- committing ----"
	rbdebug "$R/hardware/flight/commit.gdb" --no-load || exit 1
	sleep 2
	clean "$FC" | grep -a "MOTOR GATE" | tail -3
	echo; echo "If that says *** COMMITTED TO A FLIGHT TEST ***: plug the pack in, then hand over to the Mac."
	;;
*) sed -n 2,6p "$0"; exit 2 ;;
esac
