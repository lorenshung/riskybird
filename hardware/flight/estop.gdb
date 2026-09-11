# Tethered emergency stop: latch the estop and hand the core straight back.
#
#   ./rb debug flight_controller --target arty200t \
#       --config RocketArty200TDroneFullDDRConfig --mode autoflight \
#       --usb-location <debug> --no-load --no-run \
#       --gdb-script hardware/flight/estop.gdb
#
# g_estop is the same latch the safety limits and the IMU-loss watchdog set.
# Once latched, send_control() forces every motor to zero and the flight
# profile will not restart until the board is reset.
#
# READ THIS BEFORE RELYING ON IT.
#
# Halting the core is NOT a stop. The sifive PWM block is separate hardware:
# the comparators keep driving the pins at whatever duty was last written, so a
# halted core leaves the motors running with no software able to intervene
# (measured during motor bring-up -- outputs stayed at 1 and probes read 0
# after a plain detach). Stopping the motors requires SOFTWARE to run, which is
# why this script sets the latch and then RESUMES rather than parking at a
# prompt. Never Ctrl-C out of a GDB session with motors armed.
#
# It is also not instant: attaching OpenOCD, halting and resuming takes a
# second or two over JTAG. Treat it as a backstop behind FLIGHT_MAX_MS and the
# safety limits, not as the primary kill. The primary kill on a tether is
# pulling the battery.

set pagination off
set confirm off

set var g_estop = 1
printf "\n=== ESTOP LATCHED -- motors commanded to zero on the next control tick ===\n"
printf "  g_estop = %d  (reset the board to clear)\n", g_estop

# Must run. See above: without it the core stays halted and the PWM block keeps
# driving the outputs, which is the opposite of what this script is for.
monitor resume
detach
quit
