/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Sensor relay, motors build: the ESP reads the drone's sensors on its own I2C and sends them to
 * the FPGA flight controller over uart1 (riskybird/sensor_link.h). For a drone whose FPGA cannot
 * reach the sensor bus itself (2026-11-12: bus SDA reaches no FPGA pin).
 *
 * WHAT IS RELAYED. Exactly what the FPGA flight controller reads over I2C today: the BMI088
 * (accel + gyro, the same Zephyr driver and devicetree rates the FPGA uses), the down VL53L1X,
 * and the ADS7128 battery channel (AIN5, raw 12-bit code). The barometer is not on the FPGA's
 * devicetree, so it is not relayed; the wire format reserves its frame type.
 *
 * OPT-IN FROM THE FC. Nothing here touches the bus until a flight controller asks: a relay build
 * writes RELAY_KEEPALIVE_LINE about once a second, the downlink thread hands it to
 * sensor_relay_keepalive(), and frames flow while the last keepalive is under RELAY_TIMEOUT_MS
 * old. A non-relay FC never asks, so on that system the ESP stays off the bus entirely and the
 * FPGA remains the only I2C master.
 *
 * BOOT CANNOT HANG ON THE BUS. The sensor devices carry zephyr,deferred-init, so no driver talks
 * to the bus before main(); device_init() runs here, on this preemptible thread, after the motor
 * link, the radio and the loader are already up. A stuck bus stalls only this thread.
 *
 * SCHEDULING. Both relay threads are PREEMPTIBLE and below the cooperative motor-link thread, so
 * they can never delay a failsafe decision or the watchdog feed (sched_audit() still sees one
 * cooperative thread). The IMU thread (K_PRIO_PREEMPT(4)) sits above the radio's downlink/uplink;
 * the ToF thread (K_PRIO_PREEMPT(8)) sits below everything, because a VL53L1X fetch blocks for a
 * full ranging budget. Neither thread prints: the USB console can block when nobody drains it,
 * so the counters are printed by the status thread through sensor_relay_status().
 *
 * NEVER STALE. A frame is sent only after a fresh, successful read; a failed read sends nothing,
 * and the FC's own IMU-miss counter turns a silence into its existing ESTOP.
 */
#include "sensor_relay.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/vl53l1x.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <errno.h>
#include <stdio.h>

#include "riskybird/sensor_link.h"

#ifndef ESP_RELAY_IMU_HZ
#define ESP_RELAY_IMU_HZ 400      /* 17 B frames: 59 % of the ESP -> FPGA direction at 115200 */
#endif
#ifndef ESP_RELAY_TOF_PERIOD_MS
#define ESP_RELAY_TOF_PERIOD_MS 33  /* ~30 Hz, the FC's own TOF_FETCH_PERIOD_MS */
#endif
#ifndef ESP_RELAY_BATT_PERIOD_MS
#define ESP_RELAY_BATT_PERIOD_MS 200
#endif
#define RELAY_TIMEOUT_MS 3000       /* keepalive is ~1 s; three missed and the stream stops */

#define ADS7128_ADDR          0x17
#define ADS7128_WR            0x08
#define ADS7128_RD            0x10
#define ADS7128_PIN_CFG       0x05
#define ADS7128_GPIO_CFG      0x07
#define ADS7128_GPO_DRIVE_CFG 0x09
#define ADS7128_GPO_VALUE     0x0B
#define ADS7128_CHANNEL_SEL   0x11
#define SIDE_LPN_MASK         0x1Eu  /* GPIO1-4: side VL53L5CX LPn, held low (comms off) */
#define DOWN_XSHUT_BIT        0x40u  /* GPIO6: down VL53L1X XSHUT */
#define BATT_ADC_CH           5      /* AIN5: +BATT divider tap */

static const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
static const struct device *const accel = DEVICE_DT_GET(DT_NODELABEL(bmi088_accel));
static const struct device *const gyro = DEVICE_DT_GET(DT_NODELABEL(bmi088_gyro));
static const struct device *const tof = DEVICE_DT_GET(DT_NODELABEL(relay_vl53l1x));

static atomic_t keepalive_ms;       /* k_uptime_get_32() of the last RELAY ON; 0 = never */
static K_SEM_DEFINE(first_keepalive, 0, 1);
static K_MUTEX_DEFINE(bus_setup);   /* the ToF thread waits for the IMU thread's bring-up */
static volatile bool board_ready;

static struct {
	uint32_t imu_sent, imu_fail, tof_sent, tof_fail, batt_sent, batt_fail, tx_refused;
	uint32_t imu_gap_max_us;        /* worst interval between successful IMU reads */
	int init_rc_accel, init_rc_gyro, init_rc_tof, init_rc_board;
	int imu_last_rc, tof_last_rc;   /* the most recent failure's errno, 0 if none yet */
	bool imu_up, tof_up;
} st = { .init_rc_accel = 1, .init_rc_gyro = 1, .init_rc_tof = 1, .init_rc_board = 1 };

static uint8_t seq[SL_TYPE_COUNT];

void sensor_relay_keepalive(void)
{
	uint32_t now = k_uptime_get_32();

	atomic_set(&keepalive_ms, (atomic_val_t)(now ? now : 1u));
	k_sem_give(&first_keepalive);
}

static bool streaming(void)
{
	const uint32_t last = (uint32_t)atomic_get(&keepalive_ms);

	return last != 0u && (uint32_t)(k_uptime_get_32() - last) < RELAY_TIMEOUT_MS;
}

static void send(uint8_t type, const uint8_t *payload, uint32_t *sent)
{
	uint8_t frame[SL_FRAME_MAX];
	const size_t n = sensor_link_encode(frame, type, seq[type], payload);

	if (esp_bridge_fpga_write((const char *)frame, n)) {
		seq[type]++;
		(*sent)++;
	} else {
		st.tx_refused++;   /* a load session owns the wire: drop, never queue (it would be stale) */
	}
}

static int ads_wr(uint8_t reg, uint8_t val)
{
	uint8_t tx[3] = { ADS7128_WR, reg, val };

	return i2c_write(i2c, tx, sizeof(tx), ADS7128_ADDR);
}

/* The FC's board_sensor_init(), on the ESP: side ToFs held off the bus, the down VL53L1X
 * pulsed through reset, AIN5 left analog and selected for the battery conversion. */
static int board_setup(void)
{
	int rc = ads_wr(ADS7128_GPO_VALUE, 0x00)
	       | ads_wr(ADS7128_PIN_CFG, SIDE_LPN_MASK | DOWN_XSHUT_BIT)
	       | ads_wr(ADS7128_GPIO_CFG, SIDE_LPN_MASK | DOWN_XSHUT_BIT)
	       | ads_wr(ADS7128_GPO_DRIVE_CFG, SIDE_LPN_MASK | DOWN_XSHUT_BIT);

	k_msleep(5);
	rc |= ads_wr(ADS7128_GPO_VALUE, DOWN_XSHUT_BIT);   /* XSHUT high: the VL53L1X boots */
	rc |= ads_wr(ADS7128_CHANNEL_SEL, BATT_ADC_CH);
	k_msleep(10);
	return rc;
}

static int batt_read(uint16_t *code)
{
	uint8_t rx[2];
	int rc = i2c_read(i2c, rx, sizeof(rx), ADS7128_ADDR);

	if (rc == 0) {
		*code = (uint16_t)(((uint16_t)rx[0] << 4) | (rx[1] >> 4));   /* 12-bit, left-justified */
	}
	return rc;
}

static int dev_bringup(const struct device *dev)
{
	if (device_is_ready(dev)) {
		return 0;
	}
	return device_init(dev);
}

/* The ST driver's init only registers the device and defers the sensor bring-up; the FPGA path
 * calls vl53l1x_reinit() explicitly after powering the part, and so does this. */
static int tof_bringup(void)
{
	int rc = dev_bringup(tof);

	return rc != 0 ? rc : vl53l1x_reinit(tof);
}

static void imu_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	k_sem_take(&first_keepalive, K_FOREVER);

	/* Bring-up, retried once a second until all of it answers. */
	k_mutex_lock(&bus_setup, K_FOREVER);
	while (!(st.init_rc_board == 0 && st.init_rc_accel == 0 && st.init_rc_gyro == 0)) {
		if (st.init_rc_board != 0) {
			st.init_rc_board = board_setup();
		}
		if (st.init_rc_board == 0) {
			board_ready = true;
			st.init_rc_accel = dev_bringup(accel);
			st.init_rc_gyro = dev_bringup(gyro);
		}
		if (st.init_rc_board != 0 || st.init_rc_accel != 0 || st.init_rc_gyro != 0) {
			k_mutex_unlock(&bus_setup);
			k_msleep(1000);
			k_mutex_lock(&bus_setup, K_FOREVER);
		}
	}
	k_mutex_unlock(&bus_setup);
	st.imu_up = true;

	const int64_t period_us = 1000000 / ESP_RELAY_IMU_HZ;
	int64_t next_us = k_ticks_to_us_floor64(k_uptime_ticks());
	int64_t last_ok_us = 0;
	int64_t next_batt_ms = 0;

	for (;;) {
		next_us += period_us;
		const int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());

		if (next_us > now_us) {
			k_usleep((int32_t)(next_us - now_us));
		} else if (now_us - next_us > 10 * period_us) {
			next_us = now_us;   /* fell far behind (a load session, a long ISR): don't burst */
		}
		if (!streaming()) {
			last_ok_us = 0;
			k_msleep(20);
			next_us = k_ticks_to_us_floor64(k_uptime_ticks());
			continue;
		}

		struct sensor_value av[3], gv[3];
		const int rc_a = sensor_sample_fetch(accel);
		const int rc_g = rc_a == 0 ? sensor_sample_fetch(gyro) : rc_a;
		const int rc = rc_g;

		if (rc == 0 &&
		    sensor_channel_get(accel, SENSOR_CHAN_ACCEL_XYZ, av) == 0 &&
		    sensor_channel_get(gyro, SENSOR_CHAN_GYRO_XYZ, gv) == 0) {
			float a3[3], g3[3];
			uint8_t p[SL_IMU_LEN];

			for (int i = 0; i < 3; i++) {
				a3[i] = (float)sensor_value_to_double(&av[i]);
				g3[i] = (float)sensor_value_to_double(&gv[i]);
			}
			sl_imu_pack(p, a3, g3);
			send(SL_IMU, p, &st.imu_sent);

			const int64_t t = k_ticks_to_us_floor64(k_uptime_ticks());

			if (last_ok_us != 0 && (uint32_t)(t - last_ok_us) > st.imu_gap_max_us) {
				st.imu_gap_max_us = (uint32_t)(t - last_ok_us);
			}
			last_ok_us = t;
		} else {
			st.imu_fail++;   /* send nothing: the FC counts the silence */
			st.imu_last_rc = rc != 0 ? rc : -EIO;
		}

		if (k_uptime_get() >= next_batt_ms) {
			uint16_t code;

			next_batt_ms = k_uptime_get() + ESP_RELAY_BATT_PERIOD_MS;
			if (batt_read(&code) == 0) {
				uint8_t p[SL_BATT_LEN];

				sl_batt_pack(p, code);
				send(SL_BATT, p, &st.batt_sent);
			} else {
				st.batt_fail++;
			}
		}
	}
}

static void tof_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	/* After the IMU thread has raised XSHUT; the ST driver then talks to a booted part. */
	while (!board_ready) {
		k_msleep(100);
	}
	k_mutex_lock(&bus_setup, K_FOREVER);
	st.init_rc_tof = tof_bringup();
	k_mutex_unlock(&bus_setup);
	while (st.init_rc_tof != 0) {
		k_msleep(1000);
		k_mutex_lock(&bus_setup, K_FOREVER);
		st.init_rc_tof = tof_bringup();
		k_mutex_unlock(&bus_setup);
	}
	st.tof_up = true;

	for (;;) {
		if (!streaming()) {
			k_msleep(50);
			continue;
		}
		struct sensor_value d;

		/* Blocks about one ranging budget, on this lowest-priority thread only. */
		const int frc = sensor_sample_fetch(tof);
		const int grc = frc == 0 ? sensor_channel_get(tof, SENSOR_CHAN_DISTANCE, &d) : frc;

		if (grc == 0) {
			const double m = sensor_value_to_double(&d);
			const uint32_t mm = m <= 0.0 ? 0u : (m >= 65.535 ? 65535u : (uint32_t)(m * 1000.0 + 0.5));
			uint8_t p[SL_TOF_LEN];

			sl_tof_pack(p, (uint16_t)mm, 0u);
			send(SL_TOF, p, &st.tof_sent);
			k_msleep(ESP_RELAY_TOF_PERIOD_MS);
		} else {
			st.tof_fail++;
			st.tof_last_rc = grc;
			k_msleep(50);
		}
	}
}

K_THREAD_STACK_DEFINE(relay_imu_stack, 2048);
K_THREAD_STACK_DEFINE(relay_tof_stack, 2048);
static struct k_thread relay_imu_data, relay_tof_data;

void sensor_relay_start(void)
{
	k_thread_create(&relay_imu_data, relay_imu_stack, K_THREAD_STACK_SIZEOF(relay_imu_stack),
			imu_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(4), 0, K_NO_WAIT);
	k_thread_name_set(&relay_imu_data, "relay_imu");
	k_thread_create(&relay_tof_data, relay_tof_stack, K_THREAD_STACK_SIZEOF(relay_tof_stack),
			tof_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(8), 0, K_NO_WAIT);
	k_thread_name_set(&relay_tof_data, "relay_tof");
}

int sensor_relay_status(char *buf, size_t len)
{
	if ((uint32_t)atomic_get(&keepalive_ms) == 0u) {
		return snprintf(buf, len, "RELAY idle (no FC has asked)\n");
	}
	return snprintf(buf, len,
			"RELAY %s imu=%u/%u fail(rc %d) gap_max=%uus tof=%u/%u fail(rc %d) batt=%u/%u fail "
			"txrefused=%u init=b%d a%d g%d t%d\n",
			streaming() ? "on" : "stale", (unsigned)st.imu_sent, (unsigned)st.imu_fail,
			st.imu_last_rc, (unsigned)st.imu_gap_max_us, (unsigned)st.tof_sent,
			(unsigned)st.tof_fail, st.tof_last_rc,
			(unsigned)st.batt_sent, (unsigned)st.batt_fail, (unsigned)st.tx_refused,
			st.init_rc_board, st.init_rc_accel, st.init_rc_gyro, st.init_rc_tof);
}
