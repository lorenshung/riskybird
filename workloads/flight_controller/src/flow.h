/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * PMW3901 optical-flow front-end for the flight controller. Reads the sensor's motion burst on a
 * dedicated background thread (blocking SPI off the control loop, like side_tof / the down-ToF),
 * applies the validated body-frame remap, gates on SQUAL, and caches the latest body-frame ANGULAR
 * flow rate (rad/s). The control loop reads the cache non-blocking via flow_get() and multiplies by
 * the ToF height to get body-frame velocity (m/s) for the estimator's flow update.
 *
 * Axis remap (validated on HW via tools/live_flow_plot.py): the sensor is mounted so
 *   drone +x (forward) = -deltaX,   drone +y (left) = +deltaY.
 */
#ifndef ROSE_FLOW_H
#define ROSE_FLOW_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bring up the PMW3901 (SPI2, software CS) and start the background reader thread.
 * Returns 0 if the sensor answered (chip ID ok), <0 otherwise (flow then stays invalid). */
int flow_init(void);

/* Non-blocking snapshot of the latest flow sample.
 *   ang_x, ang_y : body-frame optical-flow ANGULAR rate (rad/s), remapped to the drone frame.
 *                  Body velocity (m/s) = ang * height_m.  (+x fwd, +y left)
 *   squal        : surface quality of the sample (0..~128; higher = more trustworthy)
 *   valid        : true iff the sample is fresh AND squal >= the floor (caller should also
 *                  require a valid ToF height before using it as velocity).
 */
void flow_get(float *ang_x, float *ang_y, int *squal, bool *valid);

/* k_uptime_get() of the latest sample in the cache (what flow_get() returns); very negative before
 * the first one. flow_get()'s `valid` spans FLOW_STALE_MS after it, so an age budget must use this. */
int64_t flow_sample_ms(void);

/* Raw PMW3901 counts for calibration (ground station FLOW CAL mode / tools/flow-cal.py).
 * cx/cy/n/nbad/nerr are running totals since boot (the host differences two snapshots);
 * pkx/pky are the largest per-read |delta| since the previous flow_counts() call; squal/shutter
 * are the latest read's. Sensor axes, NOT the drone remap (drone +x = -cx, +y = +cy). */
struct flow_counts_snap {
	int32_t  cx, cy;
	uint32_t n, nbad, nerr;
	int32_t  pkx, pky;
	int32_t  squal, shutter;
	int32_t  rawsum, rawmax, rawmin;   /* latest burst's pixel stats (RawData_Sum/Max/Min) */
};
void flow_counts(struct flow_counts_snap *out);

/* Raw 35x35 pixel frame (PMW3901 frame capture), for focus / exposure / texture checks on the bench.
 * flow_frame_request() asks the flow thread to grab one (motion tracking pauses ~0.3 s, then the
 * sensor is re-initialised). flow_frame_get() copies the latest frame if its sequence number is newer
 * than *seq (updating *seq) and returns its status (0 ok, <0 capture error), or 1 if nothing new. */
#define FLOW_FRAME_SIDE   35
#define FLOW_FRAME_PIXELS (FLOW_FRAME_SIDE * FLOW_FRAME_SIDE)
int  flow_frame_request(bool slow);   /* slow: conservative register timing (A/B) */
int  flow_frame_get(uint8_t out[FLOW_FRAME_PIXELS], uint32_t *seq);
/* Re-initialise the sensor with register-init variant 0 (Crazyflie) or 1 (PixArt v3.20 / PX4). Bench A/B. */
int  flow_reinit_request(int variant);

/* RoSE co-sim only (spike_riscv64 + ROSE_FLOW=1): issue the motion-burst request alongside the IMU
 * requests, then collect it before the down-ToF request goes out on the same reqrsp channel.
 * No-ops on every other target. See flow.c. */
void flow_rose_request(void);
void flow_rose_collect(void);

#ifdef __cplusplus
}
#endif

#endif /* ROSE_FLOW_H */
