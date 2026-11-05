# Read the battery sense chain on a running board, then release the core.
#
#   ./rb debug flight_controller --target arty200t \
#       --config RocketArty200TDroneFullDDRConfig --mode autoflight \
#       --usb-location <debug> --no-load --no-run \
#       --gdb-script tools/flight/gdb/battery.gdb
#
# g_vbat is the smoothed pack voltage in volts, derived from ADS7128 AIN5
# through the R30/R31 divider (Vbat = Vadc * 3, VREF = AVDD = 3V3, 12-bit).
#
# CHECK IT AGAINST A METER BEFORE TRUSTING EITHER GATE. The divider ratio, the
# reference and the channel are all assumptions baked into constants; if any of
# them is wrong the number still looks plausible while BATT_ARM_MIN_V (3.4 V)
# and BATT_CUTOFF_V (3.2 V) protect nothing. A reading within ~0.05 V of the
# meter is the pass; anything further means BATT_DIVIDER or BATT_VREF_V is
# wrong for this board revision.
#
# --no-load: a reload restarts the program and throws away the EMA.

set pagination off
set confirm off

printf "\n=== battery sense ===\n"
printf "  g_vbat        = %f V   (smoothed, EMA alpha 0.20)\n", g_vbat
printf "  arm floor     = 3.40 V (BATT_ARM_MIN_V -- refuses to ARM below)\n"
printf "  in-flight cut = 3.20 V (BATT_CUTOFF_V  -- latches estop below)\n"

printf "\n=== arming state ===\n"
printf "  g_arm_enabled = %d  (0 = locked; tools/flight/gdb/arm-enable.gdb sets it)\n", g_arm_enabled
printf "  g_estop       = %d\n", g_estop

monitor resume
detach
quit
