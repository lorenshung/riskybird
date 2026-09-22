# COMMIT TO A FLIGHT TEST. This is the run-time half of a two-part interlock.
#
#   RB_CMAKE_ARGS="-DROSE_FLIGHT_COMMIT=1" ./rb build flight_controller \
#       --target arty200t --config RocketArty200TDroneFullDDRConfig --mode esp-motors
#
#   ./rb debug flight_controller --target arty200t \
#       --config RocketArty200TDroneFullDDRConfig --mode esp-motors \
#       --usb-location <debug> --no-load --no-run \
#       --gdb-script hardware/flight/commit.gdb
#
# RUN THIS ONLY WHEN YOU ARE STANDING AT THE DRONE AND READY TO FLY.
#
# WHAT IT DOES. Without a commitment the flight controller latches all four
# motors OFF ROSE_MOTOR_WINDOW_MS (5 s) after they first spin, whatever the
# controller is asking for, and will not un-latch until the board is reset. This
# writes the key that widens that budget to ROSE_COMMIT_MAX_MS (60 s).
#
# WHY IT CANNOT HAPPEN BY ACCIDENT. Two independent acts are required and
# NEITHER ONE ALONE DOES ANYTHING:
#
#   build time   -DROSE_FLIGHT_COMMIT=1, which no --mode sets. On a build
#                without it, g_flight_commit_key IS NOT IN THE IMAGE and this
#                script fails with "No symbol \"g_flight_commit_key\"". That
#                failure is the interlock working, not a broken script --
#                verified with nm: the symbol is absent from an uncommitted
#                build and present in a committed one.
#   run time     this script, writing one specific 32-bit constant. Not 1, not
#                anything uninitialised RAM or a stray store can produce.
#
# It is polled, never latched at boot, so it cannot survive into a later session:
# a power cycle puts the board back to uncommitted.
#
# COMMIT BEFORE THE MOTORS RUN. The gate refuses a commit once the window has
# already latched -- by then the honest answer is to reset and start over.
#
# --no-load matters: without it rb reloads the ELF and restarts the program,
# which throws away the very write this script makes.

set pagination off
set confirm off

printf "\n=== before ===\n"
printf "  g_flight_commit_key = 0x%08x\n", g_flight_commit_key
printf "  g_motor_latched     = %d  (1 = too late, reset the board)\n", g_motor_latched
printf "  g_motor_committed   = %d\n", g_motor_committed
printf "  g_estop             = %d\n", g_estop

set var g_flight_commit_key = 0x464C5921

printf "\n=== COMMITTED ===\n"
printf "  g_flight_commit_key = 0x%08x\n", g_flight_commit_key
printf "  watch the console for: MOTOR GATE: *** COMMITTED TO A FLIGHT TEST ***\n"
printf "  the gate polls this on the next control tick; it is not accepted until\n"
printf "  that line prints. PROPS ARE LIVE FROM THAT MOMENT.\n"

# Load-bearing, for the same reason estop.gdb resumes. A core left HALTED stops
# the control loop. On the PWM path that leaves the SiFive comparators driving
# whatever they hold; on the ESP-offload path it stops the frame stream and the
# ESP cuts after its 120 ms failsafe. Neither is a state to leave the board in
# while a human is reaching for it.
monitor resume
detach
quit
