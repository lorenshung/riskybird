# Enable the arming gate on a tethered board, then release the core.
#
#   ./rb debug flight_controller --target arty200t \
#       --config RocketArty200TDroneFullDDRConfig --mode autoflight \
#       --usb-location <debug> --no-load --no-run \
#       --gdb-script tools/flight/gdb/arm-enable.gdb
#
# PROPELLERS OFF unless the bench thrust test has already passed.
#
# g_arm_enabled is false on boot and the ONLY writer is rose_cmd_reset(), which
# the groundstation calls over WiFi. With CONFIG_WIFI off nothing references
# that function, so --gc-sections drops it and the symbol is not in the image
# (verified with nm). The variable itself survives because it is volatile and
# the control loop reads it, so a tethered debugger can set it directly.
#
# This is deliberately a separate, explicit step rather than a build default.
# The interlock is the point: an image that boots armed is one power-cycle away
# from spinning motors nobody asked for. Loading the image does nothing; this
# script is the act of enabling, and the lift-and-place gesture is still
# required afterwards to actually arm.
#
# --no-load matters: without it rb reloads the ELF and restarts the program,
# which clears the very flag this script sets.

set pagination off
set confirm off

printf "\n=== before ===\n"
printf "  g_arm_enabled = %d\n", g_arm_enabled
printf "  g_estop       = %d\n", g_estop

set var g_arm_enabled = 1

printf "\n=== after ===\n"
printf "  g_arm_enabled = %d  (lift above LIFT_ARM_THRESHOLD_M, then set down level and still)\n", g_arm_enabled

# Load-bearing. `rb debug` shuts OpenOCD down after GDB exits, and a core left
# HALTED does not stop the motors: the PWM block is separate hardware and keeps
# driving whatever the comparators were last set to. Resuming is what leaves
# software in control of the outputs.
monitor resume
detach
quit
