/*
 * Randomized stress driver for workloads/esp_bridge/src/esp_safety_core.h (the re-arm gate
 * and the RTC record format), run by tests/test_esp_safety_fuzz.py. Compiled as C11 and as
 * C++17. Deterministic for a given seed; one violation counter per invariant, the first
 * counterexample of each printed.
 *
 *   esp_safety_fuzz_check rearm <seed> <sequences>
 *       esp_rearm_* against an independent, position-based statement of the rule, plus a
 *       pass-level model of main.c link_thread() (kill epoch at the top of each pass, ESTOP /
 *       RESET only between passes, ESTOP+RESET inside one gap).
 *   esp_safety_fuzz_check records <seed> <trials>
 *       esp_rec_seal / esp_rec_valid for all four record layouts: single-bit and single-word
 *       corruption, random multi-word corruption, structured high-bit corruption, torn
 *       writes in the firmware's real store order, and the esp_snap_state() packing.
 *
 * Output: "VIOL <name> <count>", "FIRST <name> <text>", "STAT <name> <value>". Exit 0 if it ran.
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_safety_core.h"
#include <riskybird/motor_link.h>   /* the real flag values, as main.c passes them */

/* ---- PRNG + bookkeeping (same scheme as motor_softstart_fuzz_check.c) ------------------- */

static uint64_t g_rng = 1u;

static uint64_t rnd(void)
{
	g_rng ^= g_rng >> 12;
	g_rng ^= g_rng << 25;
	g_rng ^= g_rng >> 27;
	return g_rng * 0x2545F4914F6CDD1Dull;
}

static uint32_t below(uint32_t n)
{
	return n != 0u ? (uint32_t)(rnd() % n) : 0u;
}

static uint32_t rnd32(void)
{
	return (uint32_t)(rnd() >> 32);
}

#define MAXV 24
#define HIST 14

static const char *g_vname[MAXV];
static unsigned long g_viol[MAXV];
static char g_first[MAXV][1400];
static unsigned g_nv;
static char g_hist[HIST][80];
static unsigned long g_hist_n;

static unsigned vdef(const char *name)
{
	g_vname[g_nv] = name;
	return g_nv++;
}

static void hist(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(g_hist[g_hist_n % HIST], sizeof(g_hist[0]), fmt, ap);
	va_end(ap);
	g_hist_n++;
}

static void viol(unsigned v, const char *fmt, ...)
{
	if (g_viol[v]++ != 0u) {
		return;
	}
	char *p = g_first[v];
	size_t left = sizeof(g_first[0]);
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintf(p, left, fmt, ap);
	va_end(ap);
	if (n < 0 || (size_t)n >= left) {
		return;
	}
	p += n;
	left -= (size_t)n;
	unsigned long start = g_hist_n > HIST ? g_hist_n - HIST : 0u;

	for (unsigned long k = start; k < g_hist_n && left > 4u; k++) {
		n = snprintf(p, left, " [%s]", g_hist[k % HIST]);
		if (n < 0 || (size_t)n >= left) {
			break;
		}
		p += n;
		left -= (size_t)n;
	}
}

static void report(void)
{
	for (unsigned v = 0; v < g_nv; v++) {
		printf("VIOL %s %lu\n", g_vname[v], g_viol[v]);
	}
	for (unsigned v = 0; v < g_nv; v++) {
		if (g_viol[v] != 0u) {
			printf("FIRST %s %s\n", g_vname[v], g_first[v]);
		}
	}
}

/* ==== 1. the re-arm gate ================================================================== */

static uint8_t pick_flags(void)
{
	static const uint8_t common[] = {
		0u,
		MOTOR_LINK_FLAG_ARMED,
		MOTOR_LINK_FLAG_ARMED | MOTOR_LINK_FLAG_COMMIT,
		MOTOR_LINK_FLAG_COMMIT,
		MOTOR_LINK_FLAG_ARMED | MOTOR_LINK_FLAG_ESTOP,
		MOTOR_LINK_FLAG_ARMED | MOTOR_LINK_FLAG_WINDOW,
		MOTOR_LINK_FLAG_ESTOP,
	};

	if (below(4u) == 0u) {
		return (uint8_t)rnd32();   /* every bit pattern, incl. unknown/future bits */
	}
	return common[below(sizeof(common))];
}

static bool wants(uint8_t f)
{
	return (f & MOTOR_LINK_FLAG_ARMED) != 0u && (f & MOTOR_LINK_FLAGS_BLOCK) == 0u;
}

/*
 * The rule, stated by event positions rather than by the gate's own two variables:
 * a frame at position k drives iff it wants to drive, the kill latch is clear, and either no
 * hold is outstanding (none set since the last frame that drove), or a not-armed frame came
 * after the latest hold and after the latest ARMED-while-killed frame.
 */
struct oracle {
	long hold, last_true, disarm, armed_killed;
	uint32_t blocked;
};

static unsigned V_REARM_MATCH, V_REARM_DRIVE_KILLED, V_REARM_BLOCKED, V_PASS_KILL,
	V_PASS_ESTOP_EDGE, V_PASS_BOOT_EDGE;

static void oracle_init(struct oracle *o, long pos)
{
	o->hold = pos;
	o->last_true = pos - 1;
	o->disarm = pos - 2;
	o->armed_killed = pos - 2;
	o->blocked = 0u;
}

static bool oracle_frame(struct oracle *o, long k, uint8_t f, bool killed)
{
	if (!wants(f)) {
		o->disarm = k;
		return false;
	}
	if (killed) {
		o->armed_killed = k;
		return false;
	}
	if (o->hold <= o->last_true) {
		o->last_true = k;
		return true;
	}
	if (o->disarm > o->hold && o->disarm > o->armed_killed) {
		o->last_true = k;
		return true;
	}
	o->blocked++;
	return false;
}

static void rearm_sequences(unsigned long sequences)
{
	for (unsigned long q = 0; q < sequences; q++) {
		struct esp_rearm r;
		struct oracle o;
		bool killed = below(4u) == 0u;
		const bool rearm_after_cut = below(2u) != 0u;
		const unsigned len = 1u + below(80u);
		long pos = 0;

		esp_rearm_init(&r);
		oracle_init(&o, pos++);
		hist("init cut=%d", (int)rearm_after_cut);
		for (unsigned e = 0; e < len; e++, pos++) {
			const unsigned what = below(20u);

			if (what == 0u) {
				esp_rearm_init(&r);
				oracle_init(&o, pos);
				hist("init");
			} else if (what == 1u) {
				const uint8_t why = (uint8_t)(1u + below(3u));

				esp_rearm_hold(&r, why);
				o.hold = pos;
				hist("hold %s", esp_rearm_hold_name(why));
			} else if (what == 2u) {
				esp_rearm_on_cut(&r, rearm_after_cut);
				if (rearm_after_cut) {
					o.hold = pos;
				}
				hist("cut");
			} else if (what == 3u) {
				killed = !killed;
				hist("killed=%d", (int)killed);
			} else {
				const uint8_t f = pick_flags();
				const bool got = esp_rearm_frame(&r, f, MOTOR_LINK_FLAG_ARMED,
								 MOTOR_LINK_FLAGS_BLOCK, killed);
				const bool exp = oracle_frame(&o, pos, f, killed);

				hist("frame 0x%02x killed=%d -> %d", f, (int)killed, (int)got);
				if (got != exp) {
					viol(V_REARM_MATCH, "sequence %lu: esp_rearm_frame returned %d, "
					     "the rule says %d:", q, (int)got, (int)exp);
				}
				if (got && (!wants(f) || killed)) {
					viol(V_REARM_DRIVE_KILLED, "drove a frame 0x%02x killed=%d:", f,
					     (int)killed);
				}
				if (r.blocked != o.blocked) {
					viol(V_REARM_BLOCKED, "blocked %u, rule %u:", r.blocked, o.blocked);
					o.blocked = r.blocked;
				}
			}
		}
	}
}

/*
 * main.c link_thread(), at the pass level, with the real gate:
 *   top of pass: kill epoch changed -> esp_rearm_hold(KILL)
 *   each queued frame: esp_rearm_frame(.., g_kill) and the ARMED/BLOCK refusal
 *   end of pass, sometimes: the silence failsafe -> esp_rearm_on_cut()
 * between passes (the uplink thread, preemptible, only runs while the link thread
 * blocks): ESTOP (g_kill = 1, epoch++), RESET (g_kill = 0), or both in one gap.
 * User-level properties: no frame drives while the latch is set; after an ESTOP, and after
 * boot, no frame drives until a not-armed frame has been processed since.
 */
static void link_passes(unsigned long runs)
{
	for (unsigned long q = 0; q < runs; q++) {
		struct esp_rearm r;
		bool g_kill = below(8u) == 0u;   /* restored from RTC at boot */
		unsigned epoch = 0, seen = 0;
		bool need_disarm = true;          /* boot */
		bool estop_pending_edge = false;
		const bool rearm_after_cut = below(2u) != 0u;
		const unsigned passes = 1u + below(60u);

		esp_rearm_init(&r);
		hist("boot kill=%d", (int)g_kill);
		for (unsigned p = 0; p < passes; p++) {
			/* gap: uplink commands */
			const unsigned u = below(12u);

			if (u == 0u) {
				g_kill = true;
				epoch++;
				estop_pending_edge = true;
				hist("ESTOP");
			} else if (u == 1u) {
				g_kill = false;
				hist("RESET");
			} else if (u == 2u) {
				g_kill = true;
				epoch++;
				g_kill = false;
				estop_pending_edge = true;
				hist("ESTOP+RESET");
			}
			/* pass */
			if (epoch != seen) {
				seen = epoch;
				esp_rearm_hold(&r, ESP_REARM_KILL);
			}
			if (estop_pending_edge) {
				need_disarm = true;   /* the user-level rule from this pass on */
				estop_pending_edge = false;
			}
			const unsigned nframes = below(5u);

			for (unsigned k = 0; k < nframes; k++) {
				const uint8_t f = pick_flags();
				bool drive = esp_rearm_frame(&r, f, MOTOR_LINK_FLAG_ARMED,
							     MOTOR_LINK_FLAGS_BLOCK, g_kill);

				if (drive && !wants(f)) {
					drive = false;   /* main.c's own ARMED/BLOCK refusal */
				}
				hist("frame 0x%02x kill=%d -> %d", f, (int)g_kill, (int)drive);
				if (!wants(f)) {
					need_disarm = false;
				}
				if (drive && g_kill) {
					viol(V_PASS_KILL, "run %lu pass %u: drove while the latch is set:",
					     q, p);
				}
				if (drive && need_disarm) {
					viol(epoch ? V_PASS_ESTOP_EDGE : V_PASS_BOOT_EDGE,
					     "run %lu pass %u: drove with no not-armed frame since "
					     "the %s:", q, p, epoch ? "ESTOP" : "boot");
				}
			}
			if (below(10u) == 0u) {
				esp_rearm_on_cut(&r, rearm_after_cut);
				if (rearm_after_cut) {
					need_disarm = true;
				}
				hist("failsafe cut");
			}
		}
	}
}

static int run_rearm(uint64_t seed, unsigned long sequences)
{
	V_REARM_MATCH = vdef("rearm_matches_rule");
	V_REARM_DRIVE_KILLED = vdef("rearm_never_drives_killed_or_unarmed");
	V_REARM_BLOCKED = vdef("rearm_blocked_counter");
	V_PASS_KILL = vdef("pass_no_drive_while_latched");
	V_PASS_ESTOP_EDGE = vdef("pass_estop_needs_disarm");
	V_PASS_BOOT_EDGE = vdef("pass_boot_needs_disarm");
	g_rng = seed ? seed : 1u;
	rearm_sequences(sequences);
	link_passes(sequences);
	report();
	printf("STAT sequences %lu\n", sequences);
	return 0;
}

/* ==== 2. RTC records ======================================================================= */

struct layout {
	const char *name;
	unsigned nw;
	uint32_t magic;
};

static const struct layout LAYOUTS[] = {
	{ "boot", BOOT_NW, ESP_REC_BOOT_MAGIC },
	{ "snap", SNAP_NW, ESP_REC_SNAP_MAGIC },
	{ "fault", FAULT_NW, ESP_REC_FAULT_MAGIC },
	{ "kill", KILL_NW, ESP_REC_KILL_MAGIC },
};
#define NLAYOUT (sizeof(LAYOUTS) / sizeof(LAYOUTS[0]))

static void make_record(uint32_t *w, const struct layout *L)
{
	for (unsigned i = 0; i < L->nw; i++) {
		w[i] = below(3u) == 0u ? below(4u) : rnd32();   /* small values are common */
	}
	esp_rec_seal(w, L->nw, L->magic);
}

static bool same(const uint32_t *a, const uint32_t *b, unsigned n)
{
	return memcmp(a, b, n * sizeof(uint32_t)) == 0;
}

static int run_records(uint64_t seed, unsigned long trials)
{
	const unsigned V_BIT = vdef("rec_single_bit_flip_rejected");
	const unsigned V_WORD = vdef("rec_single_word_replace_rejected");
	const unsigned V_MULTI = vdef("rec_random_multiword_rejected");
	const unsigned V_HIGH = vdef("rec_highbit_multiword_rejected");
	const unsigned V_XTYPE = vdef("rec_never_valid_as_other_type");
	const unsigned V_TORN = vdef("rec_torn_write_old_new_or_invalid");
	const unsigned V_SEAL = vdef("rec_sealed_is_valid");
	const unsigned V_STATE = vdef("snap_state_roundtrip");
	unsigned long high_trials = 0, high_acc[32];
	uint32_t w[FAULT_NW], c[FAULT_NW];

	memset(high_acc, 0, sizeof(high_acc));
	g_rng = seed ? seed : 1u;

	for (unsigned long t = 0; t < trials; t++) {
		const struct layout *L = &LAYOUTS[below(NLAYOUT)];

		make_record(w, L);
		if (!esp_rec_valid(w, L->nw, L->magic)) {
			viol(V_SEAL, "%s: sealed record does not validate", L->name);
		}
		for (unsigned j = 0; j < NLAYOUT; j++) {
			if (&LAYOUTS[j] != L && LAYOUTS[j].nw == L->nw &&
			    esp_rec_valid(w, L->nw, LAYOUTS[j].magic)) {
				viol(V_XTYPE, "%s record validates as %s", L->name, LAYOUTS[j].name);
			}
		}
		/* every single-bit flip of every word */
		if (t < 20000u) {
			for (unsigned i = 0; i < L->nw; i++) {
				for (unsigned b = 0; b < 32u; b++) {
					memcpy(c, w, L->nw * sizeof(uint32_t));
					c[i] ^= 1u << b;
					if (esp_rec_valid(c, L->nw, L->magic)) {
						viol(V_BIT, "%s word %u bit %u flip accepted", L->name, i, b);
					}
				}
			}
		}
		/* one word replaced by anything */
		memcpy(c, w, L->nw * sizeof(uint32_t));
		{
			const unsigned i = below(L->nw);

			do {
				c[i] = below(2u) ? rnd32() : w[i] ^ (1u + below(0xFFFFu));
			} while (c[i] == w[i]);
			if (esp_rec_valid(c, L->nw, L->magic)) {
				viol(V_WORD, "%s word %u 0x%08x -> 0x%08x accepted", L->name, i,
				     w[i], c[i]);
			}
		}
		/* random corruption of 2..nw words (magic kept: the hard case) */
		memcpy(c, w, L->nw * sizeof(uint32_t));
		{
			const unsigned k = 2u + below(L->nw - 1u);

			for (unsigned m = 0; m < k; m++) {
				c[1u + below(L->nw - 1u)] = rnd32();
			}
			if (!same(c, w, L->nw) && esp_rec_valid(c, L->nw, L->magic)) {
				viol(V_MULTI, "%s random multi-word corruption accepted", L->name);
			}
		}
		/* structured: the same high bit flipped in two non-magic words (a correlated
		 * LP SRAM upset, or two half-word stores torn the same way) */
		if (L->nw >= 3u) {
			const unsigned b = 24u + below(8u);
			const unsigned i = 1u + below(L->nw - 1u);
			unsigned j;

			do {
				j = 1u + below(L->nw - 1u);
			} while (j == i);
			memcpy(c, w, L->nw * sizeof(uint32_t));
			c[i] ^= 1u << b;
			c[j] ^= 1u << b;
			high_trials++;
			if (esp_rec_valid(c, L->nw, L->magic)) {
				high_acc[b]++;
				viol(V_HIGH, "%s: bit %u flipped in words %u and %u -> still valid "
				     "(0x%08x -> 0x%08x, 0x%08x -> 0x%08x)", L->name, b, i, j,
				     w[i], c[i], w[j], c[j]);
			}
		}
	}

	/* torn writes in the firmware's store order, from a valid OLD record to NEW:
	 *   snap  esp_safety_snap_store: UP, DUTY01, DUTY23, STATE, then seal (magic, check)
	 *   kill  esp_safety_kill_store: KILLED, then seal
	 *   boot  esp_safety_boot: COUNT, then seal
	 *   fault fault_record: KIND..NAME1, then seal
	 * cut after every prefix: the result must validate only as OLD or as NEW. */
	for (unsigned long t = 0; t < trials / 4u; t++) {
		const struct layout *L = &LAYOUTS[below(NLAYOUT)];
		uint32_t old_r[FAULT_NW], new_r[FAULT_NW];

		make_record(old_r, L);
		make_record(new_r, L);
		/* the write order: payload words 1..nw-2, then magic, then check */
		for (unsigned cut = 0; cut <= L->nw; cut++) {
			memcpy(c, old_r, L->nw * sizeof(uint32_t));
			for (unsigned s = 0; s < cut; s++) {
				const unsigned idx = s < L->nw - 2u ? 1u + s :
						     s == L->nw - 2u ? 0u : L->nw - 1u;

				c[idx] = new_r[idx];
			}
			if (esp_rec_valid(c, L->nw, L->magic) && !same(c, old_r, L->nw) &&
			    !same(c, new_r, L->nw)) {
				viol(V_TORN, "%s torn after %u of %u stores validates as a mix",
				     L->name, cut, L->nw);
			}
		}
	}

	/* esp_snap_state packing, decoded as esp_safety_prev_fields() does */
	for (unsigned long t = 0; t < trials / 4u; t++) {
		const uint8_t flags = (uint8_t)rnd32();
		const bool kill = below(2u) != 0u, link = below(2u) != 0u;
		const uint8_t hold = (uint8_t)below(8u);
		const uint32_t st = esp_snap_state(flags, kill, link, hold);

		if ((st & 0xffu) != flags || ((st >> 8) & 1u) != (uint32_t)kill ||
		    ((st >> 9) & 1u) != (uint32_t)link || ((st >> 12) & 0x7u) != hold ||
		    (st & ~0x73ffu) != 0u) {
			viol(V_STATE, "flags 0x%02x kill %d link %d hold %u -> 0x%08x", flags,
			     (int)kill, (int)link, hold, st);
		}
	}

	report();
	printf("STAT trials %lu\n", trials);
	printf("STAT highbit_trials %lu\n", high_trials);
	for (unsigned b = 24; b < 32u; b++) {
		printf("STAT highbit_accepted_bit%u %lu\n", b, high_acc[b]);
	}
	return 0;
}

int main(int argc, char **argv)
{
	if (argc >= 4 && strcmp(argv[1], "rearm") == 0) {
		return run_rearm(strtoull(argv[2], NULL, 0), strtoul(argv[3], NULL, 0));
	}
	if (argc >= 4 && strcmp(argv[1], "records") == 0) {
		return run_records(strtoull(argv[2], NULL, 0), strtoul(argv[3], NULL, 0));
	}
	fprintf(stderr, "usage: %s rearm <seed> <n> | records <seed> <n>\n", argv[0]);
	return 2;
}
