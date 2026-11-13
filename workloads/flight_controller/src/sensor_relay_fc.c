/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Relay sensors for flight_controller's *-relay builds: the ESP reads the BMI088, the down
 * VL53L1X and the ADS7128 battery channel and sends them over uart1
 * (riskybird/sensor_link.h). uart_cmd_poll() demultiplexes each frame and hands it to
 * sensor_relay_ingest(); the flight code keeps reading through the sensor API, because
 * fpga-sensor-relay.overlay points its aliases at the riskybird,relay-sensor nodes defined here.
 *
 * SEMANTICS, chosen so the flight code's existing guards do the right thing:
 *   accel/gyro  sample_fetch() latches the newest IMU sample if it is younger than
 *               ROSE_RELAY_IMU_STALE_MS, else returns -EAGAIN. read_sensor_frame() turns that
 *               into a miss, and SAFE_MAX_IMU_MISS consecutive misses is the existing IMU-lost
 *               ESTOP -- so a stopped relay stops the motors the same way a dead BMI088 does.
 *   distance    sample_fetch() blocks until a NEW ToF sample arrives (at most 200 ms, then
 *               -EAGAIN), like the ST driver's blocking fetch the down-ToF thread was written for.
 *   battery     sensor_relay_batt() returns the newest ADS7128 code if it is under 1 s old;
 *               battery_poll() applies its unchanged divider math and EMA.
 *
 * Producer and consumers: the control-loop thread ingests (uart_cmd_poll); the control loop, the
 * down-ToF thread and battery_poll() read. A spinlock guards the shared copy; the ToF semaphore
 * wakes the ToF thread.
 */
#define DT_DRV_COMPAT riskybird_relay_sensor

#include "sensor_relay_fc.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

#include "riskybird/sensor_link.h"

#define RELAY_TOF_WAIT_MS 200
#define RELAY_BATT_STALE_MS 1000

enum relay_kind { RELAY_ACCEL = 0, RELAY_GYRO = 1, RELAY_DISTANCE = 2 };   /* binding enum order */

struct relay_cfg {
	enum relay_kind kind;
};

struct relay_data {
	float v[3];
};

static struct k_spinlock lock;
static struct {
	float accel[3], gyro[3];
	int64_t imu_ms;
	bool imu_valid;
	uint16_t tof_mm;
	uint16_t batt_code;
	int64_t batt_ms;
	bool batt_valid;
} latest;
static K_SEM_DEFINE(tof_new, 0, 1);

/* Counters, readable over JTAG (and printed on the console at boot by the caller). */
volatile uint32_t g_relay_imu_frames, g_relay_tof_frames, g_relay_batt_frames;
volatile uint32_t g_relay_imu_stale;

void sensor_relay_ingest(const struct sensor_link_frame *f)
{
	const int64_t now = k_uptime_get();
	k_spinlock_key_t key;

	switch (f->type) {
	case SL_IMU: {
		float a[3], g[3];

		sl_imu_unpack(f->payload, a, g);
		key = k_spin_lock(&lock);
		for (int i = 0; i < 3; i++) {
			latest.accel[i] = a[i];
			latest.gyro[i] = g[i];
		}
		latest.imu_ms = now;
		latest.imu_valid = true;
		k_spin_unlock(&lock, key);
		g_relay_imu_frames++;
		break;
	}
	case SL_TOF:
		key = k_spin_lock(&lock);
		latest.tof_mm = sl_get_le16(f->payload);
		k_spin_unlock(&lock, key);
		g_relay_tof_frames++;
		k_sem_give(&tof_new);
		break;
	case SL_BATT:
		key = k_spin_lock(&lock);
		latest.batt_code = (uint16_t)(sl_get_le16(f->payload) & 0x0FFFu);
		latest.batt_ms = now;
		latest.batt_valid = true;
		k_spin_unlock(&lock, key);
		g_relay_batt_frames++;
		break;
	default:
		break;   /* SL_BARO: not consumed by this build (the FPGA devicetree has no baro) */
	}
}

bool sensor_relay_batt(uint16_t *code)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	const bool ok = latest.batt_valid && (k_uptime_get() - latest.batt_ms) < RELAY_BATT_STALE_MS;

	if (ok) {
		*code = latest.batt_code;
	}
	k_spin_unlock(&lock, key);
	return ok;
}

static int relay_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct relay_cfg *cfg = dev->config;
	struct relay_data *d = dev->data;

	ARG_UNUSED(chan);
	if (cfg->kind == RELAY_DISTANCE) {
		if (k_sem_take(&tof_new, K_MSEC(RELAY_TOF_WAIT_MS)) != 0) {
			return -EAGAIN;
		}
		k_spinlock_key_t key = k_spin_lock(&lock);

		d->v[0] = (float)latest.tof_mm / 1000.0f;
		k_spin_unlock(&lock, key);
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);
	const bool fresh = latest.imu_valid && (k_uptime_get() - latest.imu_ms) <= ROSE_RELAY_IMU_STALE_MS;

	if (fresh) {
		const float *src = cfg->kind == RELAY_ACCEL ? latest.accel : latest.gyro;

		for (int i = 0; i < 3; i++) {
			d->v[i] = src[i];
		}
	}
	k_spin_unlock(&lock, key);
	if (!fresh) {
		g_relay_imu_stale++;
		return -EAGAIN;
	}
	return 0;
}

static int relay_channel_get(const struct device *dev, enum sensor_channel chan,
			     struct sensor_value *val)
{
	const struct relay_cfg *cfg = dev->config;
	const struct relay_data *d = dev->data;
	int n;

	switch (cfg->kind) {
	case RELAY_ACCEL:
		n = (chan == SENSOR_CHAN_ACCEL_XYZ) ? 3 : 0;
		break;
	case RELAY_GYRO:
		n = (chan == SENSOR_CHAN_GYRO_XYZ) ? 3 : 0;
		break;
	default:
		n = (chan == SENSOR_CHAN_DISTANCE) ? 1 : 0;
		break;
	}
	if (n == 0) {
		return -ENOTSUP;
	}
	for (int i = 0; i < n; i++) {
		sensor_value_from_float(&val[i], d->v[i]);
	}
	return 0;
}

static DEVICE_API(sensor, relay_api) = {
	.sample_fetch = relay_sample_fetch,
	.channel_get = relay_channel_get,
};

static int relay_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

#define RELAY_DEFINE(inst)                                                                         \
	static struct relay_data relay_data_##inst;                                                \
	static const struct relay_cfg relay_cfg_##inst = {                                         \
		.kind = (enum relay_kind)DT_INST_ENUM_IDX(inst, kind),                              \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, relay_init, NULL, &relay_data_##inst,                   \
				     &relay_cfg_##inst, POST_KERNEL,                               \
				     CONFIG_SENSOR_INIT_PRIORITY, &relay_api);

DT_INST_FOREACH_STATUS_OKAY(RELAY_DEFINE)
