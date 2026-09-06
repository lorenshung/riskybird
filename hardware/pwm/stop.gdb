# Force every motor pin low, without relying on the CPU.
#
#   ./rb debug motor_4 --board arty200t --config RocketArty200TMotorSpadConfig \
#       --usb-location 7-1.1.4.1 --no-load --gdb-script hardware/pwm/stop.gdb
#
# Writes the PWM comparators directly, so it works even when the workload has
# crashed -- which is the case that matters. The arm-window timeout lives in the
# main loop, and a wedged CPU cannot run it while the PWM block, being separate
# hardware, keeps driving whatever was last written.
set pagination off
set confirm off
printf "before: cmp1=%u cmp2=%u cmp3=%u (block0)  cmp1=%u (block1)  period=%u\n", \
    *(unsigned int*)0x10050024, *(unsigned int*)0x10050028, \
    *(unsigned int*)0x1005002c, *(unsigned int*)0x10051024, \
    *(unsigned int*)0x10050020
set *(unsigned int*)0x10050024 = 0
set *(unsigned int*)0x10050028 = 0
set *(unsigned int*)0x1005002c = 0
set *(unsigned int*)0x10051024 = 0
printf "after:  cmp1=%u cmp2=%u cmp3=%u (block0)  cmp1=%u (block1)  -- all pins low\n", \
    *(unsigned int*)0x10050024, *(unsigned int*)0x10050028, \
    *(unsigned int*)0x1005002c, *(unsigned int*)0x10051024
detach
quit
