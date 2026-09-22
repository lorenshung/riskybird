# Stop a workload that is running standalone on the board.
#
#   ./rb debug flight_controller --target arty200t \
#       --config RocketArty200TDroneFullDDRConfig --mode autoflight \
#       --usb-location <debug> --no-load --no-run \
#       --gdb-script hardware/flight/stop.gdb
#
# `rb debug` with boot.gdb deliberately resumes the core and detaches, so the
# drone keeps flying after GDB exits -- that is the point of it, and it means
# quitting GDB is NOT a way to stop anything.
#
# THIS SCRIPT COVERS TWO DIFFERENT MACHINES, AND THEY NEED OPPOSITE THINGS.
#
#   FPGA drives the PWM (--mode motors / autoflight, the `motors` DT alias
#   present). The SiFive PWM block is separate hardware: the comparators keep
#   driving the pins at whatever they hold, so HALTING THE CPU DOES NOT STOP THE
#   MOTORS. The comparators must be zeroed BEFORE the core is halted, never
#   after. That is what the mww writes below are.
#
#   ESP owns the PWM (--mode esp-motors). The FPGA drives no gate at all --
#   CONFIG_PWM is off, pwm_sifive_init() never runs, and the four comparator
#   addresses below belong to a block that is not connected to anything. Writing
#   them is a NO-OP THAT LOOKS LIKE IT WORKED. Do not read "zeroing motor
#   comparators" below as evidence that anything stopped on that path. What
#   stops motors there is the ESTOP frame the running core transmits, and behind
#   it the ESP's 120 ms receive failsafe once the core is halted.
#
# So the order is: zero the comparators (the only thing that helps the PWM path,
# harmless on the offload path), latch g_estop, LET THE CORE RUN briefly so the
# offload path actually transmits an explicit zero-duty ESTOP frame, and only
# then halt. Halting first would leave the offload path relying on a 120 ms
# timeout when an explicit "off" was available in one control tick.
#
# Addresses are the four bound motor outputs on the PWM path:
#   pwm0 0x10050000  cmp1/2/3 = motor1 (E22/J1_31), motor2 (B20/J1_36), motor3 (R17/J1_99)
#   pwm1 0x10051000  cmp1     = motor4 (F13/J1_100)

set pagination off
set confirm off

printf "\n=== zeroing motor comparators (PWM path only -- a NO-OP under --mode esp-motors) ===\n"
monitor mww 0x10050024 0
monitor mww 0x10050028 0
monitor mww 0x1005002c 0
monitor mww 0x10051024 0
monitor mdw 0x10050024
monitor mdw 0x10051024

# Best effort, and deliberately after the comparators: if this workload has no
# g_estop the batch aborts here, and by then the PWM path is already off.
set var g_estop = 1
printf "=== g_estop latched ===\n"

# Run for a moment. On the offload path this is what puts an explicit zero-duty
# ESTOP frame on the wire -- a flags change bypasses the frame rate limiter, so
# it goes out on the very next control tick rather than waiting for the 50 Hz
# slot. On the PWM path send_control() writes zeros again, which is harmless.
# "Commanded off" and "link dead" are different things and the receiver should
# be told which one this is.
monitor resume
shell sleep 0.3

# Now halting is safe on both paths, and on the offload path it is a second,
# independent stop: no frames -> the ESP cuts all four within 120 ms (+5 ms
# tick). Halting is also the point of this script: it stops the control loop,
# the ToF thread, the status LED and everything else the image is running.
monitor halt
printf "=== core halted -- program stopped ===\n"
printf "=== on --mode esp-motors, confirm on the ESP console: cuts=1 and\n"
printf "    'link=DOWN -- MOTORS OFF (failsafe)' within ~125 ms ===\n"

detach
quit
