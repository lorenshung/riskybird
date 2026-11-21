/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * flight_controller relay builds (ROSE_SENSOR_RELAY): see sensor_relay_fc.c.
 */
#ifndef FC_SENSOR_RELAY_FC_H
#define FC_SENSOR_RELAY_FC_H

#include <stdbool.h>
#include <stdint.h>

#include "riskybird/sensor_link.h"

/* An IMU sample older than this is a miss (read_sensor_frame() fails; SAFE_MAX_IMU_MISS of them
 * while armed is the IMU-lost ESTOP). The ESP sends 400 samples/s (2.5 ms apart); 50 ms is 20
 * frame periods. NOT YET MEASURED against the ESP's worst gap: read RELAY gap_max= on the ESP
 * console with the down-ToF ranging before trusting it in flight. */
#ifndef ROSE_RELAY_IMU_STALE_MS
#define ROSE_RELAY_IMU_STALE_MS 50
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* A CRC-good frame from the uart1 demultiplexer. Control-loop thread. */
void sensor_relay_ingest(const struct sensor_link_frame *f);

/* The newest ADS7128 AIN5 code (12-bit) if under 1 s old. */
bool sensor_relay_batt(uint16_t *code);

/* The newest relayed barometer sample (pressure kPa, temperature degC) if under ~0.5 s old; *seq
 * is a per-sample counter for once-each gating. Used by the ROSE_BARO altitude path on relay HW
 * (the FPGA cannot reach the sensor bus, so the baro only arrives via SL_BARO). */
bool sensor_relay_baro(float *kpa, float *tc, uint32_t *seq);

extern volatile uint32_t g_relay_imu_frames, g_relay_tof_frames, g_relay_batt_frames,
	g_relay_baro_frames;
extern volatile uint32_t g_relay_imu_stale;

#ifdef __cplusplus
}
#endif

#endif /* FC_SENSOR_RELAY_FC_H */
