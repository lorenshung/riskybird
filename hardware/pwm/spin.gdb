# Arm the motors for a visual check, then get out of the way.
#
#   ./rb debug motor_4 --board arty200t \
#       --config RocketArty200TMotorSpadConfig --usb-location 7-1.1.4.1 \
#       --gdb-script hardware/pwm/spin.gdb
#
# Use this one, not probe.gdb, once motors are installed.
#
# probe.gdb stops on a breakpoint to read the reconstructed waveform, and that
# is exactly wrong here: the PWM block is independent hardware and keeps
# toggling while the CPU is halted, so the motors keep spinning while the arm
# window -- which is checked by the main loop -- can never expire. A halted CPU
# disables the safety timeout without stopping the motors.
#
# This arms and detaches instead. The CPU runs free, MOTOR_ARM_TIMEOUT_MS
# elapses, and the workload stops the outputs on its own with no debugger
# attached. Re-run for another spin.
#
# SAFETY: this spins the motors at MOTOR_MAX_PCT-clamped 10% for the length of
# the arm window. Propellers off.

set pagination off
set confirm off

set $pc = &__start

# Let it boot and take its own stopped-state measurement before arming.
break motor_probe_done
continue
delete breakpoints

printf "\n--- before arming ---\n"
printf "duty at the pin: %u %u %u %u per mille (expect 0)\n", \
    motor_duty_permille[0], motor_duty_permille[1], \
    motor_duty_permille[2], motor_duty_permille[3]
printf "phase span: %u/%u, wraps: %u/%u\n", \
    motor_probe_phase_span[0], motor_probe_phase_span[1], \
    motor_probe_wraps[0], motor_probe_wraps[1]

printf "\n--- arming at 10%% and detaching; watch the motors ---\n"
set var motor_arm_window_ms = 8000
set var motor_throttle_pct = 10
set var motor_arm_request = 0xA5A5C0DE
# `monitor resume` is not optional. GDB detaches and rb then shuts OpenOCD
# down, and without this the core is left HALTED -- which stops the main loop
# while the PWM block, being independent hardware, keeps driving the pins. That
# is motors spinning with nothing able to stop them. Measured: detach alone
# left output changes at 1 and probes run at 0, i.e. the CPU never moved.
monitor resume
detach
quit

# The motors spin for motor_arm_window_ms and then the workload stops them by
# itself, with no debugger attached. Confirm with:
#   ./rb debug motor_4 --board arty200t --config RocketArty200TMotorSpadConfig \
#       --usb-location 7-1.1.4.1 --no-load --gdb-script hardware/pwm/readback.gdb
# "arm windows expired" should have gone up by one and the duty back to 0.
