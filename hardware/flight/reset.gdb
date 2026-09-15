# Soft reset between flights: clear the latches so the drone can fly again,
# WITHOUT re-enabling arming.
#
#   ./rb debug flight_controller --target arty200t \
#       --config RocketArty200TDroneFullDDRConfig --mode autoflight \
#       --usb-location <debug> --no-load --no-run \
#       --gdb-script hardware/flight/reset.gdb
#
# After a completed flight the controller latches flight_done ("one-shot: don't
# auto re-arm") and, if anything tripped, g_estop stays set until reset. Neither
# clears on its own, so a second flight needs this.
#
# Bumping g_reset_gen is what the ground station's RESET command does. The
# control loop sees the change and clears g_estop, g_armed, g_arming,
# flight_done and arm_since, and re-inits the estimator and controller
# (dropping all integrators). The boot gyro-bias cal is deliberately KEPT --
# recalibrating in the rushed post-landing window contaminated the bias and
# made each successive flight worse.
#
# DELIBERATELY NOT rose_cmd_reset(). That also sets g_arm_enabled, which would
# couple "clear the latches" to "allow takeoff" in one step. Worse, `lifted` is
# never cleared, so the lift-and-place gate is already half-satisfied after a
# flight: with arming enabled the drone RE-ARMS ON ITS OWN after
# PLACE_CONFIRM_MS (1.5 s) of sitting level and still -- no gesture needed,
# just sitting on the bench. Keeping arming off here means re-enabling it stays
# a separate, deliberate act via arm-enable.gdb, and you choose the moment.
#
# Writes happen while the core is HALTED. `rb debug --no-load --no-run` halts on
# attach, and that is required here: `set var` against a RUNNING target fails on
# this build with "Could not read registers; remote failure reply '0E'".
# Resuming first looks safer and instead makes every write silently impossible.
#
# The trade is that an error before the resume below leaves the core halted.
# That is the safe direction for this script: it only ever disarms.

set pagination off
set confirm off

printf "\n=== before ===\n"
printf "  g_estop = %d   g_arm_enabled = %d\n", g_estop, g_arm_enabled

set var g_arm_enabled = 0
set var g_reset_gen = g_reset_gen + 1

printf "\n=== soft reset issued ===\n"
printf "  latches cleared on the next control iteration; watch for:\n"
printf "    flight_controller: SOFT RESET -- estop cleared, disarmed\n"
printf "  arming left DISABLED -- run hardware/flight/arm-enable.gdb when ready,\n"
printf "  and expect it to arm ~1.5 s later without a new lift.\n"

monitor resume
detach
quit
