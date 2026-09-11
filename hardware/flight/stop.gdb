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
# Order matters here. The motor comparators are zeroed FIRST, through OpenOCD's
# own memory writes, because that path depends on nothing: not on the symbol
# table, not on which workload is loaded, not on the control loop still being
# sane. Halting the CPU does NOT stop the motors -- the SiFive PWM block is
# separate hardware and keeps driving whatever the comparators hold -- so the
# comparators have to go to zero before the core is halted, never after.
#
# Addresses are the four bound motor outputs:
#   pwm0 0x10050000  cmp1/2/3 = motor1 (E22/J1_31), motor2 (B20/J1_36), motor3 (R17/J1_99)
#   pwm1 0x10051000  cmp1     = motor4 (F13/J1_100)

set pagination off
set confirm off

printf "\n=== zeroing motor comparators (hardware level) ===\n"
monitor mww 0x10050024 0
monitor mww 0x10050028 0
monitor mww 0x1005002c 0
monitor mww 0x10051024 0
monitor mdw 0x10050024
monitor mdw 0x10051024

# Best effort, and deliberately after the comparators: if this workload has no
# g_estop the batch aborts here, and by then the motors are already off.
set var g_estop = 1
printf "=== g_estop latched ===\n"

# No `monitor resume`. Halting is the point: it stops the control loop, the
# ToF thread, the status LED and everything else the image is running.
monitor halt
printf "=== core halted -- program stopped ===\n"

detach
quit
