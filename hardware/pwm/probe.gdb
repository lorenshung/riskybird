# Batch readback of the on-chip PWM waveform probe. See docs/motor-pwm-bringup.md.
#
#   ./rb debug motor_4 --board arty200t \
#       --config RocketArty200TMotorSpadConfig --usb-location 7-1.1.4.1 \
#       < hardware/pwm/probe.gdb
#
# motor_4 runs forever, so this stops on motor_probe_done() rather than trying
# to interrupt at an arbitrary point: the results are settled exactly there.
#
# SAFETY: this arms the motors at 10% for one measurement. Run it only with
# propellers removed. It disarms before detaching, and the workload's own arm
# window expires on its own if this script dies partway through.

set pagination off
set confirm off
set print elements 0
set print repeats 0

break motor_probe_done

# rb debug leaves the core halted wherever it was after `load`, which is not
# necessarily the reset vector. Enter at the image's own entry point so the run
# is the same one a cold boot would give.
set $pc = &__start

printf "\n===== running to the first probe (outputs stopped) =====\n"
continue

printf "\n--- is the peripheral alive at all ---\n"
printf "phase span over 2000 reads: block0=%u block1=%u (0 = counter stopped)\n", \
    motor_probe_phase_span[0], motor_probe_phase_span[1]
printf "observed wraps:              block0=%u block1=%u (0 = counter stopped)\n", \
    motor_probe_wraps[0], motor_probe_wraps[1]
printf "phase range:                 block0=%u..%u block1=%u..%u\n", \
    motor_probe_phase_min[0], motor_probe_phase_max[0], \
    motor_probe_phase_min[1], motor_probe_phase_max[1]
printf "pwmcfg:  block0=0x%08x block1=0x%08x\n", motor_probe_cfg[0], motor_probe_cfg[1]
printf "pwmcmp0: block0=%u block1=%u (period in counts)\n", \
    motor_probe_period[0], motor_probe_period[1]
printf "mean phase advance across one comparator read: %u counts\n", \
    motor_probe_read_lag

printf "\n--- measured duty at the pin, stopped (expect 0) ---\n"
printf "motor1=%u  motor2=%u  motor3=%u  motor4=%u  per mille\n", \
    motor_duty_permille[0], motor_duty_permille[1], \
    motor_duty_permille[2], motor_duty_permille[3]
printf "cmp readback: %u %u %u %u\n", \
    motor_probe_cmp_readback[0], motor_probe_cmp_readback[1], \
    motor_probe_cmp_readback[2], motor_probe_cmp_readback[3]

printf "\n===== arming at 10%% and remeasuring =====\n"
set var motor_throttle_pct = 10
set var motor_arm_request = 0xA5A5C0DE
# The probe request matters as much as the arm: motor_probe_done is the only
# breakpoint, so without asking for a measurement the target simply runs on and
# every later command fails with "Cannot execute this command while the target
# is running". The main loop arms before it probes, so one pass does both.
set var motor_probe_request = 1
continue

printf "\n--- armed at 10%% ---\n"
printf "motor_state=0x%08x  applied=%u%%  last_error=%d  updates=%u\n", \
    motor_state, motor_applied_pct, motor_last_error, motor_updates
printf "measured duty at the pin: %u %u %u %u per mille (expect ~100)\n", \
    motor_duty_permille[0], motor_duty_permille[1], \
    motor_duty_permille[2], motor_duty_permille[3]
printf "comparator rises at bin: %u %u %u %u of 32 (expect ~3)\n", \
    motor_probe_edge_bin[0], motor_probe_edge_bin[1], \
    motor_probe_edge_bin[2], motor_probe_edge_bin[3]
printf "phase bins sampled:      %u %u %u %u of 32\n", \
    motor_probe_bins_used[0], motor_probe_bins_used[1], \
    motor_probe_bins_used[2], motor_probe_bins_used[3]
printf "cmp readback: %u %u %u %u (expect ~250)\n", \
    motor_probe_cmp_readback[0], motor_probe_cmp_readback[1], \
    motor_probe_cmp_readback[2], motor_probe_cmp_readback[3]

printf "\n--- reconstructed waveform, motor1, by phase bin ---\n"
printf "bin  samples  comparator-high  pin-high-permille\n"
set $i = 0
while $i < 32
  set $t = motor_probe_bin_total[0][$i]
  set $h = motor_probe_bin_high[0][$i]
  printf "%3d  %7u  %15u  %17u\n", $i, $t, $h, ($t > 0) ? (1000 * ($t - $h) / $t) : 0
  set $i = $i + 1
end

printf "\n===== disarming =====\n"
set var motor_arm_request = 0
set var motor_probe_request = 1
continue
printf "after disarm, duty: %u %u %u %u per mille (expect 0)\n", \
    motor_duty_permille[0], motor_duty_permille[1], \
    motor_duty_permille[2], motor_duty_permille[3]
printf "motor_state=0x%08x  applied=%u%%\n", motor_state, motor_applied_pct

detach
quit
