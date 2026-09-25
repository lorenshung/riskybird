# Read-only VL53L1X ranging and calibration state from a halted flight_controller.
#
#   ./rb debug flight_controller --target arty200t --config <Config> \
#       --mode default --usb-location <debug-usb> --no-load --no-run \
#       --gdb-script hardware/flight/tof-cal.gdb
#
# Stop the image first with hardware/flight/stop.gdb, which zeroes the PWM
# comparators before it halts the hart. Prints the control loop's cached height
# (g_tof_h / g_tof_valid, which exist in hardware builds with the down-ToF, i.e.
# the `tof` alias of flight_controller.overlay) and the ST driver's own state:
# the last ranging result, the NVM part-to-part offset, the offset/xtalk
# calibration, the optical centre and the I2C address.
#
# Use it to tell a sensor whose calibration was lost (offsets zeroed) from one
# that is ranging correctly and seeing something: a RangeMilliMeter of 0 with
# RangeStatus 0, a high signal rate and an ambient rate of 0 is a covered
# aperture, not a driver fault. Writes nothing and leaves the hart halted.

set pagination off
set confirm off
printf "\n=== tof cal ===\n"
info registers pc
print g_tof_h
print g_tof_valid
print vl53l1x_data_0.data
print vl53l1x_data_0.distance_mode
print vl53l1x_data_0.vl53l1x.Data.LLData.preset_mode
print vl53l1x_data_0.vl53l1x.Data.LLData.offset_calibration_mode
print vl53l1x_data_0.vl53l1x.Data.LLData.offset_correction_mode
print vl53l1x_data_0.vl53l1x.Data.LLData.customer
print vl53l1x_data_0.vl53l1x.Data.LLData.stat_nvm.i2c_slave__device_address
print vl53l1x_data_0.vl53l1x.Data.LLData.xtalk_cfg
print vl53l1x_data_0.vl53l1x.Data.LLData.add_off_cal_data
print vl53l1x_data_0.vl53l1x.Data.LLData.optical_centre
print *vl53l1x_data_0.vl53l1x.i2c
monitor halt
quit
