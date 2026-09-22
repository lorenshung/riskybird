# Read the envelope guard's OWN VIEW on a running board, without touching it.
#
#   ./rb debug flight_controller --target arty200t \
#       --config RocketArty200TDroneFullDDRConfig --mode esp-motors \
#       --usb-location <debug> --no-load --no-run \
#       --gdb-script hardware/flight/guard-check.gdb
#
# THIS IS THE ANSWER TO "did it not trip, or is it not looking?" -- the question
# that could not be answered on 2026-09-21, when a hand rotation to ~90 degrees
# produced no cut and nothing on the console said why.
#
# g_guard_tilt_rad  tilt from vertical measured STRAIGHT OUT OF GRAVITY, in
#                   radians. No estimator in that path. Tilt the drone by hand
#                   with the battery disconnected and watch this follow: at 45
#                   degrees it reads ~0.785, at 90 it reads ~1.571. If it does
#                   not move, the guard is not looking and nothing else here
#                   matters. 3.14159 means |a| is outside the trust band
#                   (0.6 g .. 1.5 g, i.e. 5.89 .. 14.72 m/s2), so the DIRECTION
#                   test is suspended for that sample -- the frame is being
#                   shaken. Free-fall, impact, est-tilt, rate, velocity and
#                   height keep running throughout; only this one sits out.
# g_guard_acc_skips how many samples the direction test has sat out since boot.
#                   0 with props off and the frame still. A number that CLIMBS
#                   while motors run means prop vibration is pushing |a| out of
#                   the band and the estimator's tilt is carrying the attitude
#                   envelope alone. Compare it against the run length: a few
#                   hundred out of tens of thousands is noise, a count that
#                   tracks the iteration count is the accelerometer test being
#                   effectively off.
# g_guard_est_rad   the ESTIMATOR's tilt, converted out of Rodrigues. Compare
#                   the two: they should agree within a few degrees at rest.
# g_guard_amag      |accel| in m/s^2. ~9.81 at rest. Below 3.92 is free fall,
#                   above 29.43 is an impact.
#
# The trip points are printed at boot by the ENVELOPE GUARD banner lines.

set pagination off
set confirm off

printf "\n=== envelope guard, live ===\n"
printf "  g_guard_tilt_rad = %f rad   (accelerometer, no estimator)\n", g_guard_tilt_rad
printf "  g_guard_est_rad  = %f rad   (estimator, Gibbs converted)\n", g_guard_est_rad
printf "  g_guard_amag     = %f m/s2  (~9.81 at rest)\n", g_guard_amag
printf "  g_guard_acc_skips= %u        (direction test sat out; 0 when still)\n", g_guard_acc_skips

printf "\n=== latches ===\n"
printf "  g_estop          = %d\n", g_estop
printf "  g_motor_latched  = %d  (commitment window expired)\n", g_motor_latched
printf "  g_motor_committed= %d\n", g_motor_committed

monitor resume
detach
quit
