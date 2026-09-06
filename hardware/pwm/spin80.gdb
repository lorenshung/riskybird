# Full-throttle bench spin. PROPELLERS OFF.
#
#   RB_EXTRA_CFLAGS=-DMOTOR_MAX_PCT=80 ./rb debug motor_4 --board arty200t \
#       --config RocketArty200TMotorSpadConfig --usb-location 7-1.1.4.1 \
#       --gdb-script hardware/pwm/spin80.gdb
#
# The RB_EXTRA_CFLAGS is not optional and belongs on THIS command, not on an
# earlier `rb zephyr build`: rb debug rebuilds before it loads, and a rebuild
# without the flag reconfigures CMake back to the 25% default and silently
# overwrites the image you thought you were running. That happened -- a run
# billed as 80% put 625 of 2500 on the comparators, which is 25%.
set pagination off
set confirm off
tbreak main
jump *&__start
break motor_probe_done
continue
delete breakpoints
printf "\n--- stopped state ---\n"
printf "duty at pin: %u %u %u %u per mille (expect 0)   period: %u counts\n", \
    motor_duty_permille[0], motor_duty_permille[1], motor_duty_permille[2], \
    motor_duty_permille[3], motor_probe_period[0]
printf "\n--- arming all four at 80%% for 10 s ---\n"
set var motor_arm_window_ms = 10000
set var motor_throttle_pct = 80
set var motor_arm_request = 0xA5A5C0DE
monitor resume
detach
quit
