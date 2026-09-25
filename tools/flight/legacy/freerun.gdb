# Load the flight controller and let it RUN, with no breakpoint and no motors.
#
#   ./rb debug flight_controller --target arty200t \
#       --config RocketArty200TDroneFullDDRConfig --usb-location 7-1.1.4.3 \
#       --gdb-script hardware/flight/freerun.gdb
#
# WHAT THIS IS FOR. The ground stations need a live telemetry stream to show
# anything, and the only thing that emits one is the flight controller itself.
# Every other way of getting the image onto the part either halts at a
# breakpoint (the capture scripts) or never resumes. `detach` resumes the
# inferior, which is the entire trick: GDB lets go and the core keeps running,
# printing its telemetry line to the console the bench logger is already
# tailing.
#
# WHAT THIS IS NOT. It commands no duty and asks for no flight. On an
# uncommitted build `g_flight_commit_key` is not even in the image, so the
# motor gate latches everything off 5 s after the first non-zero duty would
# have been written -- and nothing writes one, because arming needs the
# lift-and-place gesture and `g_arm_enabled` is false at boot.
#
# STOP: hardware/flight/stop.gdb over the same JTAG link, or pull USB.

set pagination off
set confirm off

# No `load` here -- rb debug already passes `-ex load`.

printf "\n=== free-run ===\n"

# `monitor resume`, not `continue`, and not `detach` alone. Three traps here,
# all hit in order:
#
#  1. rb debug appends `-ex continue` ONLY when no --gdb-script was given
#     (tools/rb/cli.py), so a script has to start the core itself. `detach`
#     alone does NOT resume a core halted at the load point -- measured: the
#     console stayed silent and the core sat at __start with .bss never zeroed,
#     so every global read uninitialised garbage.
#  2. plain `continue` blocks the script forever.
#  3. `continue &` does resume, but then `detach` fails with "Cannot execute
#     this command while the target is running", and GDB exits without
#     detaching cleanly.
#
# `monitor resume` tells OpenOCD to resume the hart directly. GDB's own idea of
# the target is untouched -- it still believes it is halted -- so the `detach`
# below succeeds, and the core is already running by then.
monitor resume

printf "  core resumed; telemetry now goes to the console\n"
printf "  stop with hardware/flight/stop.gdb, or pull USB\n\n"

detach
quit
