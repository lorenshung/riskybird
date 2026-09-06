# Load the image, start it, and get out of the way.
#
#   ./rb debug motor_4 --board arty200t \
#       --config RocketArty200TMotorSpadConfig --usb-location 7-1.1.4.1 \
#       --gdb-script hardware/pwm/boot.gdb
#
# This is all a working console needs. The other scripts here (probe, spin,
# readback) exist because this carrier's console UART was dead for the whole of
# the initial bring-up, so JTAG was the only way in or out; with the UART alive,
# arming, stopping and status all come from the console instead.
#
# Two lines are load-bearing:
#
#   set $pc   `load` puts the image in RAM but leaves the core wherever it was
#             halted, which is not the reset vector.
#   monitor resume
#             GDB detaches and rb then shuts OpenOCD down; without an explicit
#             resume the core is left HALTED. That stops the main loop -- and
#             so the arm-window timeout -- while the PWM block, being separate
#             hardware, keeps driving the pins.
#
# Leaves the board running standalone with the outputs stopped and nothing
# attached to the JTAG adapter.

set pagination off
set confirm off

# Start the image under GDB's own control, not with `set $pc` + `monitor
# resume`. A raw register write sits in GDB's cache until GDB itself resumes
# the target; `monitor` goes straight to OpenOCD and can resume from the stale
# PC -- the bootrom, in this case -- so the program either does not start or
# starts by a route nobody tested. A temporary breakpoint plus `continue` is a
# GDB-managed resume, which flushes the write, and stopping at main proves the
# image actually entered rather than assuming it.
tbreak main
jump *&__start

printf "\nreached main; releasing the core\n"

# Only now hand it to OpenOCD to run free. Without this the core is left HALTED
# when rb shuts OpenOCD down, which stops the main loop -- and the arm-window
# timeout with it -- while the PWM block keeps driving the pins.
monitor resume
detach
quit
