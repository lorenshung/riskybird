# Read the workload's state without loading or resetting anything.
#
#   ./rb debug motor_4 --board arty200t \
#       --config RocketArty200TMotorSpadConfig --usb-location 7-1.1.4.1 \
#       --no-load --gdb-script hardware/pwm/readback.gdb
#
# Answers "what did it do while I was not attached", which is how the arm
# window is confirmed to have expired on its own: motor_arm_expiries counts the
# times the workload stopped the motors by itself, with no debugger present.

set pagination off
set confirm off

printf "\n--- state ---\n"
printf "motor_state=0x%08x  applied=%u%%  last_error=%d\n", \
    motor_state, motor_applied_pct, motor_last_error
printf "output changes: %u   probes run: %u   arm windows expired: %u\n", \
    motor_updates, motor_probe_runs, motor_arm_expiries
printf "duty at the pin: %u %u %u %u per mille\n", \
    motor_duty_permille[0], motor_duty_permille[1], \
    motor_duty_permille[2], motor_duty_permille[3]
printf "arm request word: 0x%08x  (0 = disarmed)\n", motor_arm_request

# rb debug halts the core to attach, so this has to put it back. Leaving it
# halted would stop the main loop that enforces the arm window while the PWM
# block kept driving the pins -- i.e. checking the status of a spinning motor
# would disable the thing that stops it.
monitor resume
detach
quit
