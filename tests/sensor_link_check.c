/*
 * Host driver for workloads/lib/motor_link/include/riskybird/sensor_link.h
 * (tests/test_sensor_link.py). Compiled as C and as C++ (the FC includes it from main.cpp).
 *
 *   sensor_link_check encode <type> <seq> <payload-hex>
 *       Prints the encoded frame as hex ("E <hex>"), or "E -" for a bad type.
 *   sensor_link_check demux <token> [<token> ...]
 *       Each token is hex bytes, or LOST: bytes were lost here, so the NEXT byte carries
 *       FC_RX_LOST, as fc_rx_ring_put() marks it. Every entry goes through sensor_link_rx_feed()
 *       and then, exactly as the flight controller chains them, through fc_line_feed() (text
 *       entries as-is, an abort as FC_RX_LOST | '\n'). Prints
 *       "F <type> <seq> <payload-hex>" per frame, "L <line>" per command line, then the counters.
 *   sensor_link_check pack <ax> <ay> <az> <gx> <gy> <gz> <kpa> <degc>
 *       Packs and unpacks an IMU and a baro payload; prints the round-tripped values.
 */
#include "fc_uart_rx.h"
#include "riskybird/sensor_link.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int hexval(char c)
{
	if (c >= '0' && c <= '9') {
		return c - '0';
	}
	if (c >= 'a' && c <= 'f') {
		return c - 'a' + 10;
	}
	if (c >= 'A' && c <= 'F') {
		return c - 'A' + 10;
	}
	return -1;
}

static size_t unhex(const char *s, uint8_t *out, size_t cap)
{
	size_t n = 0;

	while (s[0] && s[1] && n < cap) {
		const int hi = hexval(s[0]), lo = hexval(s[1]);

		if (hi < 0 || lo < 0) {
			fprintf(stderr, "bad hex: %s\n", s);
			exit(2);
		}
		out[n++] = (uint8_t)(hi << 4 | lo);
		s += 2;
	}
	return n;
}

static struct sensor_link_rx rx;
static struct fc_line_asm lines;

static void feed_entry(int entry)
{
	struct sensor_link_frame f;
	const enum sensor_link_rx_result r = sensor_link_rx_feed(&rx, entry, &f);
	const char *line = NULL;

	if (r == SL_RX_TEXT) {
		line = fc_line_feed(&lines, entry);
	} else if (r == SL_RX_ABORT) {
		line = fc_line_feed(&lines, (int)(FC_RX_LOST | '\n'));
	} else if (r == SL_RX_FRAME) {
		printf("F %u %u ", (unsigned)f.type, (unsigned)f.seq);
		for (unsigned i = 0; i < sensor_link_payload_len(f.type); i++) {
			printf("%02x", f.payload[i]);
		}
		printf("\n");
	}
	if (line != NULL) {
		printf("L %s\n", line);
	}
}

int main(int argc, char **argv)
{
	if (argc >= 5 && strcmp(argv[1], "encode") == 0) {
		uint8_t payload[64] = { 0 }, frame[SL_FRAME_MAX];

		(void)unhex(argv[4], payload, sizeof(payload));
		const size_t n = sensor_link_encode(frame, (uint8_t)atoi(argv[2]), (uint8_t)atoi(argv[3]),
						    payload);
		printf("E ");
		if (n == 0) {
			printf("-");
		}
		for (size_t i = 0; i < n; i++) {
			printf("%02x", frame[i]);
		}
		printf("\n");
		return 0;
	}
	if (argc >= 2 && strcmp(argv[1], "demux") == 0) {
		sensor_link_rx_init(&rx);
		memset(&lines, 0, sizeof(lines));
		unsigned pending = 0u;

		for (int a = 2; a < argc; a++) {
			if (strcmp(argv[a], "LOST") == 0) {
				pending = FC_RX_LOST;
				continue;
			}
			uint8_t buf[4096];
			const size_t n = unhex(argv[a], buf, sizeof(buf));

			for (size_t i = 0; i < n; i++) {
				feed_entry((int)(buf[i] | pending));
				pending = 0u;
			}
		}
		printf("C frames=%u crc=%u type=%u lost=%u gaps=%u hunted=%u ok=%u bad=%u\n",
		       (unsigned)rx.frames_ok, (unsigned)rx.crc_errors, (unsigned)rx.type_errors,
		       (unsigned)rx.lost_in_frame, (unsigned)rx.seq_gaps, (unsigned)rx.hunted,
		       (unsigned)lines.lines_ok, (unsigned)lines.lines_bad);
		return 0;
	}
	if (argc >= 10 && strcmp(argv[1], "pack") == 0) {
		float a[3], g[3], a2[3], g2[3], kpa, degc;
		uint8_t imu[SL_IMU_LEN], baro[SL_BARO_LEN];

		for (int i = 0; i < 3; i++) {
			a[i] = (float)atof(argv[2 + i]);
			g[i] = (float)atof(argv[5 + i]);
		}
		sl_imu_pack(imu, a, g);
		sl_imu_unpack(imu, a2, g2);
		sl_baro_pack(baro, (float)atof(argv[8]), (float)atof(argv[9]));
		sl_baro_unpack(baro, &kpa, &degc);
		printf("P %.4f %.4f %.4f %.4f %.4f %.4f %.5f %.2f\n", a2[0], a2[1], a2[2], g2[0], g2[1],
		       g2[2], kpa, degc);
		return 0;
	}
	fprintf(stderr, "usage: see the header comment\n");
	return 2;
}
