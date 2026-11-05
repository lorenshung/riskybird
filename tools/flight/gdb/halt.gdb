# Stop an image that has no g_estop -- motor_duty, the bring-up workloads.
#
# stop.gdb latches g_estop before it halts, and a GDB batch aborts at the first
# error: against an image without that symbol it never reaches `monitor halt`,
# so the core keeps running. `rb fly stop` picks this script for anything that
# is not flight_controller.
#
# On the ESP offload path (motor_duty, esp-motors) halting the core stops the
# frame stream, and the ESP cuts all four motors within 120 ms (+5 ms tick).
# The comparator writes come first for the FPGA-PWM path, where halting alone
# leaves the pins at their last duty; on the offload path they are a no-op.

set pagination off
set confirm off

printf "\n=== zeroing motor comparators (PWM path only -- a NO-OP on the ESP offload path) ===\n"
monitor mww 0x10050024 0
monitor mww 0x10050028 0
monitor mww 0x1005002c 0
monitor mww 0x10051024 0

monitor halt
printf "=== core halted -- program stopped ===\n"
printf "=== on the ESP offload path, confirm on the ESP console: link=DOWN -- MOTORS OFF ===\n"

detach
quit
