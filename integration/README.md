# RiskyBird integration contracts

Top-level integration code defines stable interfaces between RiskyBird
workloads and backend-specific implementations.

`zephyr/tof/` is the ToF-array interface. The drone carrier is meant to carry
three sensors behind the ADS7128 -- two side VL53L5CX and the down-facing
VL53L1X on GPIO6 -- and on the current board one side sensor does not answer.
The contract owns XSHUT sequencing, per-sensor part detection, volatile
address assignment, identity validation, grid layout, units, validity fields,
and trace-facing frame metadata. Address-only mode depends only on Zephyr
I2C; ranging mode is adapted to the pinned backend driver and ranges only the
slots that came up as a VL53L5CX.

`zephyr/motor_link/` is the wire format of the FPGA-to-ESP motor link: the
FPGA (`flight_controller --mode esp-motors`, `motor_duty`) sends duty frames
over uart1 and the ESP (`esp_bridge --mode motors`) drives all four motor
gates. `zephyr/camera/` holds the HM01B0 capture, and `zephyr/safety/` the
attitude watchdog that cuts the motors when a bench sweep's airframe stops
sitting still.

Board buses and pins remain under `hardware/zephyr/targets/`, while workload
applications remain under `workloads/`.
