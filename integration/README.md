# RiskyBird integration contracts

Top-level integration code defines stable interfaces between RiskyBird
workloads and backend-specific implementations.

The first Zephyr contract is the four-sensor VL53L5CX interface in
`zephyr/tof/`. It owns XSHUT sequencing, volatile address assignment,
identity validation, grid layout, units, validity fields, and trace-facing
frame metadata. Address-only mode depends only on Zephyr I2C; ranging mode is
adapted to the pinned backend driver.

Board buses and pins remain under `hardware/zephyr/targets/`, while workload
applications remain under `workloads/`.
