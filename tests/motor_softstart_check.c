/*
 * Host driver for workloads/lib/motor_link/include/riskybird/motor_softstart.h
 * (tests/test_motor_softstart.py). Compiled as C and as C++.
 *
 *   motor_softstart_check sim
 *       Reads commands from stdin, one per line:
 *         R                      ss_reset() (motors_off: disarm, kill, load)
 *         C <now_us>             ss_cut() (the 120 ms link failsafe)
 *         U <now_us>             ss_resume() (first frame allowed to drive)
 *         S <now_us> <t0..t3>    ss_step(); prints "O o0 o1 o2 o3 <phaseword hex> <lim_ticks>"
 *       Every step also checks rule 1 (out <= tgt) and exits 3 on a violation.
 *   motor_softstart_check hpoint <P>
 *       For every channel 0..3 and every duty 0..P, whatever the other three duties:
 *       hpoint + duty <= P - 1 (ss_phase_hpoints, packing)
 *   motor_softstart_check packpeak <P>
 *       Equal duties: peak FETs on == ceil(4d/(P-1)) up to d = (P-1)/2, and above it a 4-on
 *       window of exactly 2d-(P-1) counts (the no-wrap optimum)
 *       whenever 0 < duty < P - 1. Prints the worst-case number of channels on
 *       at once for a few duties, as "D <pct> <duty> <peak> <counts_at_peak>".
 */
#include <riskybird/motor_softstart.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int sim(void)
{
	static struct ss s;
	char line[256];

	ss_init(&s);
	while (fgets(line, sizeof(line), stdin) != NULL) {
		if (line[0] == 'R') {
			ss_reset(&s);
			continue;
		}
		if (line[0] == 'C' || line[0] == 'U') {
			unsigned long now;

			if (sscanf(line + 1, "%lu", &now) != 1) {
				fprintf(stderr, "bad line: %s", line);
				return 2;
			}
			if (line[0] == 'C') {
				ss_cut(&s, (uint32_t)now);
			} else {
				ss_resume(&s, (uint32_t)now);
			}
			continue;
		}
		if (line[0] == 'S') {
			unsigned long now, t[SS_NMOTORS];
			uint16_t tgt[SS_NMOTORS], out[SS_NMOTORS];

			if (sscanf(line + 1, "%lu %lu %lu %lu %lu", &now, &t[0], &t[1], &t[2],
				   &t[3]) != 5) {
				fprintf(stderr, "bad line: %s", line);
				return 2;
			}
			for (unsigned i = 0; i < SS_NMOTORS; i++) {
				tgt[i] = (uint16_t)t[i];
			}
			ss_step(&s, tgt, out, (uint32_t)now);
			for (unsigned i = 0; i < SS_NMOTORS; i++) {
				if (out[i] > tgt[i]) {
					fprintf(stderr, "rule 1 violated: m%u out %u > tgt %u\n", i,
						out[i], tgt[i]);
					return 3;
				}
			}
			printf("O %u %u %u %u %04x %lu\n", out[0], out[1], out[2], out[3],
			       ss_phase_word(&s), (unsigned long)s.lim_ticks);
		}
	}
	return 0;
}

static void equal_hpoints(uint32_t d, uint32_t P, uint32_t h[4])
{
	const uint32_t dv[4] = { d, d, d, d };

	ss_phase_hpoints(dv, P, h);
}

/* channels on at counter c, and the number of counters at the peak */
static unsigned peak_on(uint32_t d, uint32_t P, unsigned *at_peak, uint32_t *width4)
{
	uint32_t h[4];
	unsigned peak = 0;

	equal_hpoints(d, P, h);
	*at_peak = 0;
	*width4 = 0;
	for (uint32_t c = 0; c < P; c++) {
		unsigned on = 0;

		for (unsigned i = 0; i < 4; i++) {
			/* high from counter == h until counter == h + d */
			on += (c >= h[i] && c < h[i] + d) ? 1u : 0u;
		}
		if (on > peak) {
			peak = on;
			*at_peak = 0;
		}
		*at_peak += (on == peak) ? 1u : 0u;
		*width4 += (on == 4u) ? 1u : 0u;
	}
	return peak;
}

static int hpoint(uint32_t P)
{
	static const uint32_t probe_pct[] = { 10, 20, 24, 30, 40, 45, 49, 50, 60, 98 };
	static const uint32_t others[] = { 0u, 1u, 7u, 100u, 0x7FFFFu };

	for (uint32_t d = 0; d <= P; d++) {
		for (unsigned i = 0; i < 4; i++) {
			for (unsigned o = 0; o < 5; o++) {
				uint32_t dv[4], h[4];
				const uint32_t ov = others[o] % (P + 1u);

				for (unsigned j = 0; j < 4; j++) {
					dv[j] = j == i ? d : (o == 4u ? (d * 7u + j) % (P + 1u) : ov);
				}
				ss_phase_hpoints(dv, P, h);
				if (h[i] >= P) {
					fprintf(stderr, "hpoint %u out of range (d=%u i=%u)\n", h[i], d, i);
					return 3;
				}
				if (d > 0 && d < P - 1 && h[i] + d > P - 1) {
					fprintf(stderr, "Lpoint overflow: i=%u d=%u h=%u P=%u\n", i, d,
						h[i], P);
					return 3;
				}
			}
		}
	}
	for (unsigned k = 0; k < sizeof(probe_pct) / sizeof(probe_pct[0]); k++) {
		const uint32_t d = P * probe_pct[k] / 100u;
		unsigned at_peak;
		uint32_t w4;
		const unsigned peak = peak_on(d, P, &at_peak, &w4);

		printf("D %u %u %u %u\n", probe_pct[k], d, peak, at_peak);
	}
	return 0;
}

/* Packing optimality with equal duties: peak == ceil(4d / (P-1)) for 0 < d <= (P-1)/2;
 * above that every pulse covers [P-1-d, d), the peak is 4 and the 4-on window is exactly
 * 2d - (P-1) counts wide. Prints "BAD ..." per failure and "OK <n>" at the end. */
static int packpeak(uint32_t P)
{
	unsigned bad = 0, n = 0;

	for (uint32_t d = 1; d + 1u < P; d++) {
		unsigned at_peak;
		uint32_t w4;
		const unsigned peak = peak_on(d, P, &at_peak, &w4);
		unsigned want;

		if (2u * d <= P - 1u) {
			want = (unsigned)((4u * d + (P - 1u) - 1u) / (P - 1u));
			if (peak != want) {
				printf("BAD P=%u d=%u peak %u want %u\n", P, d, peak, want);
				bad++;
			}
		} else if (peak != 4u || w4 != 2u * d - (P - 1u)) {
			printf("BAD P=%u d=%u peak %u 4-on width %u want 4 / %u\n", P, d, peak, w4,
			       2u * d - (P - 1u));
			bad++;
		}
		n++;
	}
	printf("OK %u bad %u\n", n, bad);
	return 0;
}

int main(int argc, char **argv)
{
	if (argc >= 2 && strcmp(argv[1], "sim") == 0) {
		return sim();
	}
	if (argc >= 3 && strcmp(argv[1], "hpoint") == 0) {
		return hpoint((uint32_t)strtoul(argv[2], NULL, 0));
	}
	if (argc >= 3 && strcmp(argv[1], "packpeak") == 0) {
		return packpeak((uint32_t)strtoul(argv[2], NULL, 0));
	}
	fprintf(stderr, "usage: %s sim | hpoint <P> | packpeak <P>\n", argv[0]);
	return 2;
}
