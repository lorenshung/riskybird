/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Sensor relay wire format: ESP32-C6 -> FPGA flight controller, on the same uart1 that carries
 * the motor duty frames the other way (riskybird/motor_link.h).
 *
 * WHY. The FPGA reads the drone's sensors over its own I2C. When that path is down (2026-11-12:
 * the bus SDA reaches no FPGA pin), the ESP still reads every sensor on the same bus, and the
 * ESP -> FPGA direction of the link is nearly idle. The ESP reads the sensors and sends them here.
 *
 * WHO SENDS WHAT.
 *   - A relay flight controller writes RELAY_KEEPALIVE_LINE on its TX (FPGA -> ESP) about once a
 *     second. The ESP streams frames only while the last keepalive is recent. An older FC image
 *     never asks, so it never receives a binary byte it does not understand.
 *   - The ESP sends one frame per fresh sensor read, never a repeat. A frame is written whole,
 *     under the same TX lock as the command lines it forwards, so a frame never lands inside a line.
 *
 * FRAME.  MAGIC  TYPE  SEQ  PAYLOAD[len(TYPE)]  CRC16-LE
 *   MAGIC 0xA6 is not printable ASCII, so it can never begin, or occur inside, a command line.
 *   SEQ counts per type and wraps at 256, so a receiver sees both gaps and staleness.
 *   CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over TYPE, SEQ and PAYLOAD.
 *
 * PAYLOADS (little-endian, sensor frame = the chip's own axes; the FC applies IMU_REMAP as before)
 *   SL_IMU   12 B  ax ay az int16 (SL_ACCEL_LSB m/s^2), gx gy gz int16 (SL_GYRO_LSB rad/s)
 *   SL_BARO   6 B  pressure uint32 (Pa/100, i.e. 0.01 Pa), temperature int16 (0.01 degC)
 *   SL_TOF    3 B  distance uint16 (mm), status uint8 (0 = valid range)
 *   SL_BATT   2 B  ADS7128 AIN5 code uint16, 12-bit (the FC's battery_poll() math unchanged)
 *
 * Header-only and free of Zephyr so the host tests (tests/test_sensor_link.py) compile it.
 */
#ifndef RISKYBIRD_SENSOR_LINK_H
#define RISKYBIRD_SENSOR_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SENSOR_LINK_MAGIC 0xA6u

enum sensor_link_type {
	SL_IMU  = 1,
	SL_BARO = 2,
	SL_TOF  = 3,
	SL_BATT = 4,
	SL_TYPE_COUNT = 5,   /* one past the last type; sizes per-type arrays */
};

#define SL_IMU_LEN   12u
#define SL_BARO_LEN   6u
#define SL_TOF_LEN    3u
#define SL_BATT_LEN   2u
#define SL_HDR_LEN    3u                 /* magic, type, seq */
#define SL_CRC_LEN    2u
#define SL_PAYLOAD_MAX SL_IMU_LEN
#define SL_FRAME_MAX  (SL_HDR_LEN + SL_PAYLOAD_MAX + SL_CRC_LEN)

/* +-65.5 m/s^2 (6.7 g) and +-32.7 rad/s (1875 deg/s): above the BMI088 ranges both ends
 * configure (6 g, 1000 deg/s), so a clamp means the sensor saturated first. */
#define SL_ACCEL_LSB 0.002f
#define SL_GYRO_LSB  0.001f

/* The FC -> ESP request. One line, so it travels the existing text path untouched. */
#define RELAY_KEEPALIVE_LINE "RELAY ON\n"
#define RELAY_KEEPALIVE_VERB "RELAY ON"

static inline unsigned sensor_link_payload_len(uint8_t type)
{
	switch (type) {
	case SL_IMU:  return SL_IMU_LEN;
	case SL_BARO: return SL_BARO_LEN;
	case SL_TOF:  return SL_TOF_LEN;
	case SL_BATT: return SL_BATT_LEN;
	default:      return 0u;
	}
}

static inline uint16_t sensor_link_crc16(const uint8_t *p, size_t n)
{
	uint16_t crc = 0xFFFFu;

	for (size_t i = 0; i < n; i++) {
		crc ^= (uint16_t)((uint16_t)p[i] << 8);
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

static inline void sl_put_le16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

static inline void sl_put_le32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static inline uint16_t sl_get_le16(const uint8_t *p)
{
	return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t sl_get_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Float -> int16 at a fixed LSB, rounded, clamped (never wrapped). */
static inline int16_t sl_q16(float v, float lsb)
{
	float q = v / lsb;

	if (q >= 32767.0f) {
		return 32767;
	}
	if (q <= -32768.0f) {
		return -32768;
	}
	return (int16_t)(q >= 0.0f ? q + 0.5f : q - 0.5f);
}

/* Write one frame to out (at least SL_FRAME_MAX bytes). Returns its length, 0 on a bad type. */
static inline size_t sensor_link_encode(uint8_t *out, uint8_t type, uint8_t seq,
					const uint8_t *payload)
{
	const unsigned n = sensor_link_payload_len(type);

	if (n == 0u) {
		return 0u;
	}
	out[0] = (uint8_t)SENSOR_LINK_MAGIC;
	out[1] = type;
	out[2] = seq;
	for (unsigned i = 0; i < n; i++) {
		out[SL_HDR_LEN + i] = payload[i];
	}
	sl_put_le16(&out[SL_HDR_LEN + n], sensor_link_crc16(&out[1], 2u + n));
	return SL_HDR_LEN + n + SL_CRC_LEN;
}

/* ---- receive demultiplexer ------------------------------------------------------------------
 * Feeds every entry of the uart1 RX stream. Bytes outside a frame are command text and come back
 * as SL_RX_TEXT for the caller's line assembler; frame bytes are consumed; a completed, CRC-good
 * frame comes back as SL_RX_FRAME. Entries are ints so an ISR ring's out-of-band loss flag
 * (bit 8, carried on the byte after the gap) passes through with its byte.
 *
 * A frame that fails (bad type, bad CRC, lost bytes inside it), or a loss flag on a byte this
 * parser consumes, returns SL_RX_ABORT: the caller should then make its line assembler drop any
 * partial line (the FC feeds it FC_RX_LOST | '\n'). The demux then HUNTS --
 * discards everything up to the next MAGIC -- because the rest of a broken frame is binary and
 * must not reach the line assembler, where it could form a garbage command. At 400 frames/s the
 * hunt lasts at most one IMU frame period (2.5 ms); a command line caught inside it is dropped
 * whole, which the ESP's own ESTOP latch and the host's resend already cover.
 */
enum sensor_link_rx_result {
	SL_RX_TEXT  = 0,   /* entry belongs to the text path (or is a lost-bytes marker) */
	SL_RX_EATEN = 1,   /* consumed by the frame parser */
	SL_RX_FRAME = 2,   /* consumed; *out holds a complete, CRC-good frame */
	SL_RX_ABORT = 3,   /* consumed; a frame failed: drop any partial command line */
};

struct sensor_link_frame {
	uint8_t type;
	uint8_t seq;
	uint8_t payload[SL_PAYLOAD_MAX];
};

struct sensor_link_rx {
	uint8_t buf[SL_FRAME_MAX];   /* type, seq, payload, crc (no magic) */
	uint8_t have;
	uint8_t need;
	uint8_t in_frame;
	uint8_t hunting;
	uint8_t seq_valid[SL_TYPE_COUNT];
	uint8_t last_seq[SL_TYPE_COUNT];
	uint32_t frames_ok;
	uint32_t crc_errors;
	uint32_t type_errors;
	uint32_t lost_in_frame;
	uint32_t seq_gaps;           /* frames missing between consecutive good frames of a type */
	uint32_t hunted;             /* bytes discarded while resynchronising */
};

static inline void sensor_link_rx_init(struct sensor_link_rx *rx)
{
	uint8_t *p = (uint8_t *)rx;

	for (size_t i = 0; i < sizeof(*rx); i++) {
		p[i] = 0u;
	}
}

static inline enum sensor_link_rx_result sensor_link_rx_feed(struct sensor_link_rx *rx, int entry,
							      struct sensor_link_frame *out)
{
	/* An ISR ring flags the byte that FOLLOWS a loss (fc_uart_rx.h: entry = byte | FC_RX_LOST).
	 * The loss ends any frame in progress; the byte itself is then parsed like any other. If the
	 * byte goes to the text path, the caller hands the flagged entry to its line assembler, which
	 * handles the loss itself. If the byte is consumed here, the caller hears SL_RX_ABORT so the
	 * line assembler still drops whatever partial line the loss cut. */
	const bool lost = entry > 0xFF;
	bool aborted = false;

	if (lost && rx->in_frame) {
		rx->in_frame = 0u;
		rx->hunting = 1u;
		rx->lost_in_frame++;
		aborted = true;
	}
	const uint8_t b = (uint8_t)(entry & 0xFF);

	if (!rx->in_frame) {
		if (b == SENSOR_LINK_MAGIC) {
			rx->in_frame = 1u;
			rx->hunting = 0u;
			rx->have = 0u;
			rx->need = 0u;
			return (lost || aborted) ? SL_RX_ABORT : SL_RX_EATEN;
		}
		if (rx->hunting) {
			rx->hunted++;
			return (lost || aborted) ? SL_RX_ABORT : SL_RX_EATEN;
		}
		return SL_RX_TEXT;
	}

	rx->buf[rx->have++] = b;
	if (rx->have == 1u) {
		const unsigned n = sensor_link_payload_len(b);

		if (n == 0u) {
			rx->in_frame = 0u;
			rx->hunting = 1u;
			rx->type_errors++;
			return SL_RX_ABORT;
		}
		rx->need = (uint8_t)(2u + n + SL_CRC_LEN);   /* type, seq, payload, crc */
		return SL_RX_EATEN;
	}
	if (rx->have < rx->need) {
		return SL_RX_EATEN;
	}

	rx->in_frame = 0u;
	const unsigned n = (unsigned)rx->need - 2u - SL_CRC_LEN;
	const uint16_t want = sl_get_le16(&rx->buf[2u + n]);

	if (sensor_link_crc16(rx->buf, 2u + n) != want) {
		rx->hunting = 1u;
		rx->crc_errors++;
		return SL_RX_ABORT;
	}
	const uint8_t type = rx->buf[0], seq = rx->buf[1];

	if (rx->seq_valid[type]) {
		rx->seq_gaps += (uint8_t)(seq - rx->last_seq[type] - 1u);
	}
	rx->seq_valid[type] = 1u;
	rx->last_seq[type] = seq;
	rx->frames_ok++;
	if (out != NULL) {
		out->type = type;
		out->seq = seq;
		for (unsigned i = 0; i < n; i++) {
			out->payload[i] = rx->buf[2u + i];
		}
	}
	return SL_RX_FRAME;
}

/* ---- payload helpers (both ends use these, so the scaling cannot drift) -------------------- */
static inline void sl_imu_pack(uint8_t *p, const float accel[3], const float gyro[3])
{
	for (int i = 0; i < 3; i++) {
		sl_put_le16(&p[2 * i], (uint16_t)sl_q16(accel[i], SL_ACCEL_LSB));
		sl_put_le16(&p[6 + 2 * i], (uint16_t)sl_q16(gyro[i], SL_GYRO_LSB));
	}
}

static inline void sl_imu_unpack(const uint8_t *p, float accel[3], float gyro[3])
{
	for (int i = 0; i < 3; i++) {
		accel[i] = (float)(int16_t)sl_get_le16(&p[2 * i]) * SL_ACCEL_LSB;
		gyro[i] = (float)(int16_t)sl_get_le16(&p[6 + 2 * i]) * SL_GYRO_LSB;
	}
}

/* Pressure in kPa (Zephyr's SENSOR_CHAN_PRESS unit) <-> 0.01 Pa counts. */
static inline void sl_baro_pack(uint8_t *p, float kpa, float degc)
{
	const float c = kpa * 100000.0f;

	sl_put_le32(p, c <= 0.0f ? 0u : (c >= 4294967040.0f ? 0xFFFFFFFFu : (uint32_t)(c + 0.5f)));
	sl_put_le16(&p[4], (uint16_t)sl_q16(degc, 0.01f));
}

static inline void sl_baro_unpack(const uint8_t *p, float *kpa, float *degc)
{
	*kpa = (float)sl_get_le32(p) / 100000.0f;
	*degc = (float)(int16_t)sl_get_le16(&p[4]) * 0.01f;
}

static inline void sl_tof_pack(uint8_t *p, uint16_t mm, uint8_t status)
{
	sl_put_le16(p, mm);
	p[2] = status;
}

static inline void sl_batt_pack(uint8_t *p, uint16_t code12)
{
	sl_put_le16(p, (uint16_t)(code12 & 0x0FFFu));
}

#endif /* RISKYBIRD_SENSOR_LINK_H */
