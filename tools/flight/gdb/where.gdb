# Where is the core, without disturbing it?
#
#   ./rb debug flight_controller --target arty200t \
#       --config RocketArty200TDroneFullDDRConfig --usb-location 7-1.1.4.3 \
#       --no-load --gdb-script tools/flight/gdb/where.gdb
#
# Attaching halts the hart, so this reports and then detaches (which resumes).
# Use it to tell "the image is running and just not printing" from "the image
# is not running at all" -- those look identical from a silent console.

set pagination off
set confirm off

printf "\n=== where is the core? ===\n"
info registers pc
printf "\n--- backtrace ---\n"
bt 12
printf "\n--- did it reach the control loop? ---\n"
printf "  g_reset_gen   = %u\n", g_reset_gen
printf "  g_estop       = %d\n", g_estop
printf "  g_arm_enabled = %d\n", g_arm_enabled

detach
quit
