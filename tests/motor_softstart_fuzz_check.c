/*
 * Randomized stress driver for workloads/lib/motor_link/include/riskybird/motor_softstart.h,
 * run by tests/test_motor_softstart_fuzz.py. Compiled as C11 and as C++17.
 *
 * Everything -- the PRNG, the scenario generator, the shadow model and every check -- is in
 * here, so a run is deterministic for a given seed. Each invariant has its own violation
 * counter; the first counterexample of each is printed with the last few events.
 *
 *   motor_softstart_fuzz_check gate <seed> <steps> <mode>
 *       mode mono  monotonic time, jitter 0..6 ms, gaps up to 2^31-1 us, the u32 wrap
 *            quant the same, quantized to 1000 us (the device clock is 1 ms)
 *            back  like mono, but the time handed to ss_step()/ss_cut()/ss_resume() can
 *                  step BACKWARDS by up to 5 ms; the model uses the high-water mark
 *   motor_softstart_fuzz_check hpoint <seed> <vectors>
 *       ss_phase_hpoints() (packing) with an independent random duty per channel
 *   motor_softstart_fuzz_check regs <seed> <updates> [duty-first]
 *       the LEDC register write sequence of pwm_led_esp32 + main.c motor_phase_apply_all(), with
 *       a timer overflow allowed between any two writes, under three latch semantics.
 *       duty-first models a driver that wrote DUTY before hpoint (to show why the
 *       order is pinned by tests/test_motor_softstart_fuzz.py)
 *
 * Output: "VIOL <name> <count>" for every invariant (0 included), "FIRST <name> <text>" for
 * the first counterexample, "STAT <name> <value>" for coverage. Exit 0 if it ran.
 */
#include <riskybird/motor_softstart.h>

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- PRNG (xorshift64*) ------------------------------------------------------------------ */

static uint64_t g_rng = 0x9E3779B97F4A7C15ull;

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

static bool chance(unsigned per_mille)
{
	return below(1000u) < per_mille;
}

/* ---- violation bookkeeping -------------------------------------------------------------- */

#define MAXV 32
#define HIST 12

static const char *g_vname[MAXV];
static unsigned long g_viol[MAXV];
static char g_first[MAXV][1600];
static unsigned g_nv;
static char g_hist[HIST][120];
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
	n = snprintf(p, left, " | last events:");
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

/* ==== 1. the spin-up gate ================================================================ */

/* Shadow model of the documented rules, on a 64-bit clock. */
struct model {
	bool     cold[SS_NMOTORS];
	uint64_t ts[SS_NMOTORS];      /* cold start (rule 5 clock) */
	uint64_t tgo[SS_NMOTORS];     /* ts + rank * stagger */
	bool     cut;
	uint64_t tcut;
	bool     stagger;             /* set by reset; a cut-cold leaves starts unstaggered (S2) */
};

static unsigned V_RULE1, V_ZERO, V_ENV_AFTER_REACH, V_ENV_AFTER_MAX, V_ENV_OTHER, V_EXACT,
	V_RULE5, V_RUN_PASS, V_COLD, V_DECREASE, V_LIM;

static struct ss g_s;
static struct model g_m;
static uint64_t g_T;           /* time handed to the gate, 64-bit (low 32 bits are "now") */
static uint64_t g_Thw;         /* high-water mark of g_T (== g_T unless mode back) */
static bool g_back;
static uint16_t g_prev_out[SS_NMOTORS];
static uint8_t g_release[SS_NMOTORS];   /* how the gate last went to RUN: 1 reach, 2 rule 5 */
static unsigned long g_steps;

/* coverage */
static unsigned long st_cut_lt299, st_cut_299, st_cut_300, st_cut_301, st_cut_long, st_cut_rep;
static unsigned long st_cold_starts, st_rank[SS_NMOTORS], st_reach_rel, st_max_rel;
static unsigned long st_warm_resume, st_cold_resume, st_direct_resume, st_wraps, st_big_gaps;
static unsigned long st_resets, st_spurious_cold_back, st_zero_blip_spinup;
static unsigned long st_close_starts;   /* cold starts within SS_STAGGER of another's HOLD entry */
static uint64_t st_min_go_spacing = UINT64_MAX;

static uint32_t env_at(unsigned i, uint64_t T)
{
	if (T < g_m.tgo[i]) {
		return 0u;
	}
	uint64_t x = T - g_m.tgo[i];

	if (x < SS_HOLD_US) {
		return SS_IDLE;
	}
	uint64_t slew = (uint64_t)SS_RATE_PER_S * (x - SS_HOLD_US) / 1000000u;

	if (slew > 0xFFFFu) {
		slew = 0xFFFFu;
	}
	return SS_IDLE + (uint32_t)slew;
}

static void model_all_cold(void)
{
	for (unsigned i = 0; i < SS_NMOTORS; i++) {
		g_m.cold[i] = true;
	}
	g_m.cut = false;
}

static void check_cold(const char *op)
{
	for (unsigned i = 0; i < SS_NMOTORS; i++) {
		const bool dev_cold = g_s.m[i].ph == SS_COLD;

		if (dev_cold == g_m.cold[i]) {
			continue;
		}
		if (g_back) {
			/* time went backwards past a latched cut (the gate reads it as
			 * a long cut), or the device clock lags the model's high-water
			 * mark at the 300 ms threshold. Mode back checks only the
			 * envelope; resync and count. */
			st_spurious_cold_back++;
		} else {
			viol(V_COLD, "after %s at T=%llu: m%u device %s, model %s", op,
			     (unsigned long long)g_T, i, dev_cold ? "COLD" : "warm",
			     g_m.cold[i] ? "COLD" : "warm");
		}
		g_m.cold[i] = dev_cold;   /* resync so later checks stay meaningful */
		g_m.stagger = g_s.stagger;
		if (!dev_cold) {
			g_m.ts[i] = g_m.tgo[i] = 0;   /* unknown; treat as long started */
		}
		g_prev_out[i] = 0;
	}
}

static void op_reset(void)
{
	hist("R T=%llu", (unsigned long long)g_T);
	ss_reset(&g_s);
	model_all_cold();
	g_m.stagger = true;
	memset(g_prev_out, 0, sizeof(g_prev_out));
	st_resets++;
	check_cold("R");
}

static void op_cut(void)
{
	hist("C T=%llu", (unsigned long long)g_T);
	ss_cut(&g_s, (uint32_t)g_T);
	if (!g_m.cut) {
		g_m.cut = true;
		g_m.tcut = g_Thw;
	} else {
		st_cut_rep++;
	}
	memset(g_prev_out, 0, sizeof(g_prev_out));
	check_cold("C");
}

static void note_cut_length(uint64_t len)
{
	if (len < 299000u) {
		st_cut_lt299++;
	} else if (len == 299000u) {
		st_cut_299++;
	} else if (len == 300000u) {
		st_cut_300++;
	} else if (len == 301000u) {
		st_cut_301++;
	} else {
		st_cut_long++;
	}
}

static void op_resume(void)
{
	hist("U T=%llu", (unsigned long long)g_T);
	if (g_m.cut) {
		const uint64_t len = g_Thw - g_m.tcut;

		note_cut_length(len);
		if (len >= SS_CUT_COLD_US) {
			model_all_cold();
			g_m.stagger = false;
			st_cold_resume++;
		} else {
			st_warm_resume++;
		}
	}
	g_m.cut = false;
	ss_resume(&g_s, (uint32_t)g_T);
	check_cold("U");
}

static void op_step(const uint16_t tgt[SS_NMOTORS])
{
	uint16_t out[SS_NMOTORS];
	uint8_t ph_before[SS_NMOTORS];
	const uint32_t lim_before = g_s.lim_ticks;
	const uint64_t T = g_Thw;   /* the model's clock (== g_T when monotonic) */

	hist("S T=%llu now=%lu tgt=%u %u %u %u", (unsigned long long)g_T,
	     (unsigned long)(uint32_t)g_T, tgt[0], tgt[1], tgt[2], tgt[3]);

	/* model: the cut rules, then cold starts by rank among this call's starters */
	if (g_m.cut && T - g_m.tcut >= SS_CUT_COLD_US) {
		note_cut_length(T - g_m.tcut);
		model_all_cold();
		g_m.stagger = false;
		memset(g_prev_out, 0, sizeof(g_prev_out));
	}
	if (g_m.cut) {
		for (unsigned i = 0; i < SS_NMOTORS; i++) {
			if (tgt[i] != 0u) {
				note_cut_length(T - g_m.tcut);
				g_m.cut = false;
				st_direct_resume++;
				break;
			}
		}
	}
	unsigned rank = 0;

	for (unsigned i = 0; i < SS_NMOTORS; i++) {
		if (tgt[i] != 0u && g_m.cold[i]) {
			g_m.cold[i] = false;
			g_m.ts[i] = T;
			g_m.tgo[i] = T + (uint64_t)rank * SS_STAGGER_US;
			st_rank[rank]++;
			if (g_m.stagger) {
				rank++;
			}
			st_cold_starts++;
			g_release[i] = 0;
			/* how close is this HOLD entry to another motor's that is still
			 * in its HOLD? (the stagger only spaces starters of one call) */
			for (unsigned j = 0; j < SS_NMOTORS; j++) {
				if (j == i || g_m.cold[j] || T - g_m.ts[j] >= SS_MAX_US) {
					continue;
				}
				const uint64_t a = g_m.tgo[i], b = g_m.tgo[j];
				const uint64_t d = a > b ? a - b : b - a;

				if (d < st_min_go_spacing) {
					st_min_go_spacing = d;
				}
				if (d < SS_STAGGER_US && T >= b && T - b < SS_HOLD_US) {
					st_close_starts++;
				}
			}
		}
	}

	for (unsigned i = 0; i < SS_NMOTORS; i++) {
		ph_before[i] = g_s.m[i].ph;
	}
	bool started[SS_NMOTORS];

	for (unsigned i = 0; i < SS_NMOTORS; i++) {
		started[i] = g_s.m[i].ph == SS_COLD && tgt[i] != 0u;
	}
	ss_step(&g_s, tgt, out, (uint32_t)g_T);
	g_steps++;
	check_cold("S");
	if (g_back) {
		/* mode back: whether a cut expired (and so whether this call's starters are
		 * staggered) can differ from the model at the 300 ms boundary, since the
		 * device clock lags the high-water mark; take the rank from the device */
		for (unsigned i = 0; i < SS_NMOTORS; i++) {
			if (started[i] && !g_m.cold[i]) {
				g_m.tgo[i] = g_m.ts[i] + (uint32_t)(g_s.m[i].t_go - g_s.m[i].t_start);
			}
		}
		g_m.stagger = g_s.stagger;
	}

	bool any_lim = false;

	for (unsigned i = 0; i < SS_NMOTORS; i++) {
		const uint8_t ph = g_s.m[i].ph;

		if (ph_before[i] != SS_RUN && ph == SS_RUN) {
			if ((uint32_t)((uint32_t)g_T - g_s.m[i].t_start) >= SS_MAX_US) {
				g_release[i] = 2;
				st_max_rel++;
			} else {
				g_release[i] = 1;
				st_reach_rel++;
			}
		}
		if (out[i] < tgt[i]) {
			any_lim = true;
		}
		if (out[i] > tgt[i]) {
			viol(V_RULE1, "T=%llu m%u out %u > tgt %u", (unsigned long long)g_T, i,
			     out[i], tgt[i]);
		}
		if (tgt[i] == 0u && out[i] != 0u) {
			viol(V_ZERO, "T=%llu m%u tgt 0 out %u", (unsigned long long)g_T, i, out[i]);
		}
		if (!g_back && tgt[i] <= g_prev_out[i] && out[i] != tgt[i]) {
			viol(V_DECREASE, "T=%llu m%u tgt %u <= previous out %u but out %u",
			     (unsigned long long)g_T, i, tgt[i], g_prev_out[i], out[i]);
		}
		if (g_m.cold[i]) {
			g_prev_out[i] = out[i];
			continue;
		}
		const uint64_t el = T - g_m.ts[i];

		if (el < SS_MAX_US) {
			/* mode back: the device clock may lag the high-water mark by up to
			 * 5 ms, so allow the envelope that far ahead */
			const uint32_t env = env_at(i, g_back ? T + 5000u : T);

			if (tgt[i] == 0u && ph != SS_RUN) {
				st_zero_blip_spinup++;
			}
			if (out[i] > env) {
				if (ph_before[i] == SS_RUN && g_release[i] == 1) {
					viol(V_ENV_AFTER_REACH,
					     "T=%llu m%u %llu us after its cold start: out %u > "
					     "envelope %u (released to RUN earlier with the "
					     "envelope below full scale)",
					     (unsigned long long)g_T, i, (unsigned long long)el,
					     out[i], env);
				} else if (g_release[i] == 2) {
					viol(V_ENV_AFTER_MAX,
					     "T=%llu (now=%lu, t_start=%lu) m%u only %llu us after its "
					     "cold start: out %u > envelope %u (rule 5 fired "
					     "early: elapsed now - t_start >= SS_MAX_US)",
					     (unsigned long long)g_T, (unsigned long)(uint32_t)g_T,
					     (unsigned long)g_s.m[i].t_start, i,
					     (unsigned long long)el, out[i], env);
				} else {
					viol(V_ENV_OTHER, "T=%llu m%u el %llu out %u > envelope %u ph %u->%u",
					     (unsigned long long)g_T, i, (unsigned long long)el,
					     out[i], env, ph_before[i], ph);
				}
			}
			if (tgt[i] != 0u && (ph == SS_WAIT || ph == SS_HOLD || ph == SS_SLEW)) {
				const uint32_t want = tgt[i] < env ? tgt[i] : env;
				uint8_t want_ph;

				if (T < g_m.tgo[i]) {
					want_ph = SS_WAIT;
				} else if (T - g_m.tgo[i] < SS_HOLD_US) {
					want_ph = SS_HOLD;
				} else {
					want_ph = SS_SLEW;
				}
				if (!g_back && (out[i] != want || ph != want_ph)) {
					viol(V_EXACT, "T=%llu m%u out %u ph %u, model min(tgt %u, env %u) "
					     "= %u ph %u", (unsigned long long)g_T, i, out[i], ph,
					     tgt[i], env, want, want_ph);
				}
			}
		} else if (ph != SS_RUN && !g_back) {
			viol(V_RULE5, "T=%llu m%u %llu us after its cold start, phase %u, not RUN",
			     (unsigned long long)g_T, i, (unsigned long long)el, ph);
		}
		if (ph_before[i] == SS_RUN && ph == SS_RUN && out[i] != tgt[i]) {
			viol(V_RUN_PASS, "T=%llu m%u RUN but out %u != tgt %u",
			     (unsigned long long)g_T, i, out[i], tgt[i]);
		}
		g_prev_out[i] = out[i];
	}
	if (g_s.lim_ticks != lim_before + (any_lim ? 1u : 0u)) {
		viol(V_LIM, "T=%llu lim_ticks %lu -> %lu, any_lim %d", (unsigned long long)g_T,
		     (unsigned long)lim_before, (unsigned long)g_s.lim_ticks, (int)any_lim);
	}
}

/* ---- scenario generator ---- */

static int g_mode_quant;

static void advance(uint64_t dt)
{
	const uint64_t before = g_T;

	g_T += dt;
	if ((g_T >> 32) != (before >> 32)) {
		st_wraps++;
	}
	if (g_T > g_Thw) {
		g_Thw = g_T;
	}
}

static uint64_t jitter(void)
{
	uint64_t dt;

	if (chance(1)) {
		dt = chance(100) ? 0x7FFFFFFFull : 1000000ull + below(2000000u);  /* stall/huge gap */
		st_big_gaps++;
	} else if (chance(20)) {
		dt = 20000u + below(200000u);    /* a slow pass */
	} else {
		dt = below(6001u);                /* 0 .. ESP_LINK_TICK_MS + 1, incl. same-us calls */
	}
	if (g_mode_quant) {
		dt = (dt + 500u) / 1000u * 1000u;
	}
	return dt;
}

/* one step of the clock, possibly backwards in mode back */
static void tick(void)
{
	advance(jitter());
	if (g_back && chance(150)) {
		const uint64_t back = 1u + below(5000u);

		g_T -= back;   /* g_Thw keeps the high-water mark */
		if (g_T + 5000u < g_Thw) {
			g_T = g_Thw - 5000u;   /* lag at most 5 ms behind the high-water mark */
		}
	} else if (g_back) {
		g_T = g_Thw;
	}
}

static uint16_t pick_value(void)
{
	switch (below(10u)) {
	case 0: return 0u;
	case 1: return (uint16_t)(1u + below(999u));     /* below idle */
	case 2: return (uint16_t)SS_IDLE;                /* exactly idle */
	case 3: return 2500u;                            /* the default ceiling */
	case 4: return 9999u;
	case 5: return 10000u;                           /* MOTOR_LINK_DUTY_FULL */
	case 6: return (uint16_t)(SS_IDLE + 1u + below(400u));   /* just above idle */
	default: return (uint16_t)below(10001u);
	}
}

static void drive_episode(void)
{
	uint16_t tgt[SS_NMOTORS];
	const unsigned steps = 1u + below(chance(100) ? 400u : 60u);
	const unsigned pattern = below(6u);
	const unsigned mask = 1u + below(15u);
	uint16_t base = pick_value();
	const uint16_t hi = pick_value();

	for (unsigned k = 0; k < steps; k++) {
		for (unsigned i = 0; i < SS_NMOTORS; i++) {
			switch (pattern) {
			case 0:   /* all four the same */
				tgt[i] = base;
				break;
			case 1:   /* independent random each step */
				tgt[i] = pick_value();
				break;
			case 2:   /* a subset, the rest 0 */
				tgt[i] = (mask >> i) & 1u ? base : 0u;
				break;
			case 3: { /* a ramp from base to hi over the episode */
				const int32_t a = base, b = hi;

				tgt[i] = (uint16_t)(a + (b - a) * (int32_t)k / (int32_t)steps);
				break;
			}
			case 4: { /* mixer-like: base +- 15 %, clamped */
				int32_t v = (int32_t)base + (int32_t)below(3001u) - 1500;

				tgt[i] = (uint16_t)(v < 0 ? 0 : v > 10000 ? 10000 : v);
				break;
			}
			default:  /* each motor its own constant */
				tgt[i] = (uint16_t)(((uint32_t)base * (i + 1u) * 7919u) % 10001u);
				break;
			}
			if (chance(40)) {
				tgt[i] = 0u;   /* a zero blip */
			}
		}
		if (pattern == 0 && chance(30)) {
			base = pick_value();
		}
		op_step(tgt);
		tick();
	}
}

static void cut_episode(void)
{
	static const uint16_t zero[SS_NMOTORS] = { 0, 0, 0, 0 };
	uint64_t len;

	switch (below(6u)) {
	case 0: len = 299000u; break;
	case 1: len = 300000u; break;
	case 2: len = 301000u; break;
	case 3: len = below(299000u); break;
	case 4: len = 120000u + below(60000u); break;
	default: len = below(1200000u); break;
	}
	op_cut();
	const uint64_t t_end = g_Thw + len;
	bool recut = chance(200);

	/* the caller feeds zeros for the cut; the link thread services every <= 5 ms,
	 * sometimes not at all (a cut with no step in it) */
	if (!chance(100)) {
		while (g_Thw + 5000u < t_end) {
			op_step(zero);
			advance(1u + below(5000u));
			if (g_back) {
				g_T = g_Thw;
			}
			if (recut && chance(100)) {
				op_cut();   /* a second cut keeps the first start */
				recut = false;
			}
		}
	}
	/* land exactly on the intended length */
	if (t_end > g_Thw) {
		advance(t_end - g_Thw);
	}
	if (g_back) {
		g_T = g_Thw;
	}
	switch (below(10u)) {
	case 0:
		op_reset();           /* a disarm during the cut */
		op_resume();
		break;
	case 1: case 2:
		break;                /* no U: a nonzero command reaching ss_step ends it */
	default:
		op_resume();          /* main.c handle_frame(): ss_resume, then the step */
		break;
	}
	drive_episode();
}

static int run_gate(uint64_t seed, unsigned long steps, const char *mode)
{
	V_RULE1 = vdef("rule1_out_le_cmd");
	V_ZERO = vdef("zero_cmd_zero_out");
	V_DECREASE = vdef("decrease_instant");
	V_ENV_AFTER_REACH = vdef("envelope_after_reach_release");
	V_ENV_AFTER_MAX = vdef("envelope_after_rule5_release");
	V_ENV_OTHER = vdef("envelope_other");
	V_EXACT = vdef("spinup_exact");
	V_RULE5 = vdef("rule5_bounded");
	V_RUN_PASS = vdef("run_passthrough");
	V_COLD = vdef("cold_only_documented");
	V_LIM = vdef("lim_ticks");

	g_mode_quant = strcmp(mode, "quant") == 0;
	g_back = strcmp(mode, "back") == 0;
	if (!g_mode_quant && !g_back && strcmp(mode, "mono") != 0) {
		fprintf(stderr, "unknown mode %s\n", mode);
		return 2;
	}
	g_rng = seed ? seed : 1u;
	ss_init(&g_s);
	model_all_cold();
	g_m.stagger = true;
	/* start a little before the u32 wrap of the microsecond clock */
	g_T = g_Thw = (1ull << 32) * (1u + below(3u)) - below(3000000u);
	if (g_mode_quant) {
		g_T = g_Thw = g_T / 1000u * 1000u;
	}

	while (g_steps < steps) {
		const uint32_t r = below(100u);

		if (r < 70u) {
			drive_episode();
		} else if (r < 92u) {
			drive_episode();
			cut_episode();
		} else {
			op_reset();
		}
		if (chance(5)) {
			/* re-seat near a wrap boundary */
			uint64_t next = ((g_Thw >> 32) + 1u) << 32;

			advance(next - g_Thw - below(400000u));
			if (g_back) {
				g_T = g_Thw;
			}
		}
	}
	report();
	printf("STAT steps %lu\n", g_steps);
	printf("STAT cold_starts %lu\n", st_cold_starts);
	for (unsigned r = 0; r < SS_NMOTORS; r++) {
		printf("STAT rank%u %lu\n", r, st_rank[r]);
	}
	printf("STAT reach_releases %lu\n", st_reach_rel);
	printf("STAT max_releases %lu\n", st_max_rel);
	printf("STAT cut_lt299 %lu\n", st_cut_lt299);
	printf("STAT cut_299 %lu\n", st_cut_299);
	printf("STAT cut_300 %lu\n", st_cut_300);
	printf("STAT cut_301 %lu\n", st_cut_301);
	printf("STAT cut_long %lu\n", st_cut_long);
	printf("STAT cut_repeated %lu\n", st_cut_rep);
	printf("STAT warm_resumes %lu\n", st_warm_resume);
	printf("STAT cold_resumes %lu\n", st_cold_resume);
	printf("STAT direct_resumes %lu\n", st_direct_resume);
	printf("STAT resets %lu\n", st_resets);
	printf("STAT wraps %lu\n", st_wraps);
	printf("STAT big_gaps %lu\n", st_big_gaps);
	printf("STAT zero_blip_spinup %lu\n", st_zero_blip_spinup);
	printf("STAT spurious_cold_backwards %lu\n", st_spurious_cold_back);
	printf("STAT close_cold_starts %lu\n", st_close_starts);
	printf("STAT min_hold_entry_spacing_us %llu\n",
	       (unsigned long long)(st_min_go_spacing == UINT64_MAX ? 0u : st_min_go_spacing));
	return 0;
}

/* ==== 2. phase interleave arithmetic ====================================================== */

/* The on-intervals [lo, hi) of the four channels in one period, as the hardware would run
 * them: ss_phase_hpoints() places all four from all four duties. */
static void on_intervals(const uint32_t d[SS_NMOTORS], uint32_t P, bool interleave,
			 uint32_t lo[SS_NMOTORS], uint32_t hi[SS_NMOTORS])
{
	uint32_t h[SS_NMOTORS];

	ss_phase_hpoints(d, P, h);
	for (unsigned i = 0; i < SS_NMOTORS; i++) {
		if (d[i] == 0u) {
			lo[i] = hi[i] = 0u;
		} else if (d[i] >= P) {   /* the driver's 100 % path: stopped at idle-high */
			lo[i] = 0u;
			hi[i] = P;
		} else {
			lo[i] = interleave ? h[i] : 0u;
			hi[i] = lo[i] + d[i];
		}
	}
}

static uint32_t ov(uint32_t a0, uint32_t a1, uint32_t b0, uint32_t b1)
{
	const uint32_t lo = a0 > b0 ? a0 : b0, hi = a1 < b1 ? a1 : b1;

	return hi > lo ? hi - lo : 0u;
}

static unsigned peak_on(const uint32_t lo[], const uint32_t hi[])
{
	unsigned peak = 0;

	for (unsigned i = 0; i < SS_NMOTORS; i++) {
		if (hi[i] == lo[i]) {
			continue;
		}
		unsigned n = 0;

		for (unsigned j = 0; j < SS_NMOTORS; j++) {
			n += (hi[j] > lo[j] && lo[j] <= lo[i] && lo[i] < hi[j]) ? 1u : 0u;
		}
		if (n > peak) {
			peak = n;
		}
	}
	return peak;
}

/* sum over the period of max(0, channels on - 1) */
static uint64_t excess_on(const uint32_t lo[], const uint32_t hi[], uint32_t P)
{
	uint32_t pts[2 * SS_NMOTORS + 2];
	unsigned n = 0;
	uint64_t total = 0;

	for (unsigned i = 0; i < SS_NMOTORS; i++) {
		pts[n++] = lo[i];
		pts[n++] = hi[i];
	}
	pts[n++] = 0u;
	pts[n++] = P;
	for (unsigned a = 1; a < n; a++) {   /* insertion sort, n <= 10 */
		uint32_t v = pts[a];
		unsigned b = a;

		while (b > 0u && pts[b - 1u] > v) {
			pts[b] = pts[b - 1u];
			b--;
		}
		pts[b] = v;
	}
	for (unsigned a = 0; a + 1u < n; a++) {
		const uint32_t x = pts[a], w = pts[a + 1u] - pts[a];
		unsigned on = 0;

		if (w == 0u) {
			continue;
		}
		for (unsigned j = 0; j < SS_NMOTORS; j++) {
			on += (lo[j] <= x && x < hi[j]) ? 1u : 0u;
		}
		if (on > 1u) {
			total += (uint64_t)(on - 1u) * w;
		}
	}
	return total;
}

static unsigned long st_all4, st_all4_fail;

static int run_hpoint(uint64_t seed, unsigned long vectors)
{
	static const uint32_t Ps[] = { 256u, 1024u, 2048u, 4096u };
	const unsigned V_BOUND = vdef("hpoint_lpoint_le_P_minus_1");
	const unsigned V_RANGE = vdef("hpoint_in_range");
	const unsigned V_PAIR = vdef("pair_overlap_le_inphase");
	const unsigned V_PEAK = vdef("peak_le_inphase");
	const unsigned V_EXCESS = vdef("excess_on_le_inphase");
	const unsigned V_STRICT_EQ = vdef("equal_duty_overlap_strictly_reduced");
	const unsigned V_STRICT = vdef("random_duty_overlap_strictly_reduced");
	unsigned long strict_cases = 0, eq_cases = 0;
	uint64_t red_num = 0, red_den = 0;

	g_rng = seed ? seed : 1u;
	/* exhaustive: every channel, every duty 0..P+1, for each P, with the other three at
	 * each of a set of duties (the bound must hold whatever the neighbours are) */
	for (unsigned k = 0; k < sizeof(Ps) / sizeof(Ps[0]); k++) {
		const uint32_t P = Ps[k];
		const uint32_t others[] = { 0u, 1u, P / 4u, P / 2u - 1u, P / 2u, P - 2u, P - 1u, P };

		for (uint32_t d = 0; d <= P + 1u; d++) {
			for (unsigned i = 0; i < SS_NMOTORS; i++) {
				for (unsigned o = 0; o < sizeof(others) / sizeof(others[0]); o++) {
					uint32_t dv[SS_NMOTORS], hv[SS_NMOTORS];

					for (unsigned j = 0; j < SS_NMOTORS; j++) {
						dv[j] = j == i ? d : others[o];
					}
					ss_phase_hpoints(dv, P, hv);
					const uint32_t h = hv[i];

					if (h >= P) {
						viol(V_RANGE, "P=%u i=%u d=%u h=%u", P, i, d, h);
					}
					if (d >= 1u && d <= P - 1u && h + d > P - 1u) {
						viol(V_BOUND, "P=%u i=%u d=%u others=%u h=%u: Lpoint "
						     "%u > P-1", P, i, d, others[o], h, h + d);
					}
					if ((d == 0u || d >= P - 1u) && h != 0u) {
						viol(V_BOUND, "P=%u i=%u d=%u: driver path but h=%u",
						     P, i, d, h);
					}
				}
			}
		}
	}
	for (unsigned long v = 0; v < vectors; v++) {
		const uint32_t P = Ps[below(4u)];
		uint32_t d[SS_NMOTORS], lo[SS_NMOTORS], hi[SS_NMOTORS], l0[SS_NMOTORS],
			 h0[SS_NMOTORS];
		const unsigned kind = below(4u);
		const uint32_t common = 1u + below(P - 4u);
		unsigned roomy = 0, active = 0;

		for (unsigned i = 0; i < SS_NMOTORS; i++) {
			switch (kind) {
			case 0:  d[i] = common; break;                               /* equal */
			case 1:  d[i] = below(5u) == 0u ? 0u : common; break;        /* equal subset */
			case 2: {                                                    /* near-equal */
				int32_t x = (int32_t)common + (int32_t)below(P / 8u) - (int32_t)(P / 16u);

				d[i] = (uint32_t)(x < 0 ? 0 : x > (int32_t)P ? (int32_t)P : x);
				break;
			}
			default: /* independent, incl. 0, P-1, P */
				switch (below(8u)) {
				case 0: d[i] = 0u; break;
				case 1: d[i] = P - 1u; break;
				case 2: d[i] = P; break;
				default: d[i] = below(P + 1u); break;
				}
				break;
			}
			active += (d[i] != 0u) ? 1u : 0u;
			roomy += (d[i] >= 1u && d[i] + 4u <= P) ? 1u : 0u;
		}
		on_intervals(d, P, true, lo, hi);
		on_intervals(d, P, false, l0, h0);
		for (unsigned i = 0; i < SS_NMOTORS; i++) {
			if (d[i] >= 1u && d[i] <= P - 1u && lo[i] + d[i] > P - 1u) {
				viol(V_BOUND, "P=%u i=%u d=%u h=%u", P, i, d[i], lo[i]);
			}
		}
		uint64_t pi = 0, p0 = 0;

		for (unsigned i = 0; i < SS_NMOTORS; i++) {
			for (unsigned j = i + 1u; j < SS_NMOTORS; j++) {
				const uint32_t a = ov(lo[i], hi[i], lo[j], hi[j]);
				const uint32_t b = ov(l0[i], h0[i], l0[j], h0[j]);

				if (a > b) {
					viol(V_PAIR, "P=%u d=[%u %u %u %u] pair %u,%u overlap %u > in-phase %u",
					     P, d[0], d[1], d[2], d[3], i, j, a, b);
				}
				pi += a;
				p0 += b;
			}
		}
		if (peak_on(lo, hi) > peak_on(l0, h0)) {
			viol(V_PEAK, "P=%u d=[%u %u %u %u] peak %u > in-phase %u", P, d[0], d[1],
			     d[2], d[3], peak_on(lo, hi), peak_on(l0, h0));
		}
		if (excess_on(lo, hi, P) > excess_on(l0, h0, P)) {
			viol(V_EXCESS, "P=%u d=[%u %u %u %u] excess %llu > in-phase %llu", P, d[0],
			     d[1], d[2], d[3], (unsigned long long)excess_on(lo, hi, P),
			     (unsigned long long)excess_on(l0, h0, P));
		}
		if ((kind == 0 || kind == 1) && roomy >= 2u) {
			eq_cases++;
			if (pi >= p0) {
				viol(V_STRICT_EQ, "P=%u d=[%u %u %u %u] summed pair overlap %llu, "
				     "in-phase %llu", P, d[0], d[1], d[2], d[3],
				     (unsigned long long)pi, (unsigned long long)p0);
			}
		}
		if (roomy >= 2u) {
			strict_cases++;
			if (pi >= p0) {
				viol(V_STRICT, "P=%u d=[%u %u %u %u] summed pair overlap %llu, "
				     "in-phase %llu (a short pulse lands wholly inside a long one: "
				     "each hpoint depends only on its own duty)", P, d[0], d[1],
				     d[2], d[3], (unsigned long long)pi, (unsigned long long)p0);
			}
		}
		if (active >= 2u) {
			red_num += pi;
			red_den += p0;
		}
		/* the flight case: all four running, none at the driver's 0/100 % paths, under
		 * a 98 % ceiling (2007 counts at P = 2048) */
		if (P == 2048u && d[0] && d[1] && d[2] && d[3] && d[0] <= 2007u && d[1] <= 2007u &&
		    d[2] <= 2007u && d[3] <= 2007u) {
			st_all4++;
			st_all4_fail += (pi >= p0) ? 1u : 0u;
		}
	}
	report();
	printf("STAT vectors %lu\n", vectors);
	printf("STAT strict_cases %lu\n", strict_cases);
	printf("STAT equal_cases %lu\n", eq_cases);
	printf("STAT all4_flight_cases %lu\n", st_all4);
	printf("STAT all4_flight_not_reduced %lu\n", st_all4_fail);
	printf("STAT mean_overlap_ratio_pct %llu\n",
	       (unsigned long long)(red_den ? red_num * 100u / red_den : 0u));
	return 0;
}

/* ==== 3. the LEDC register sequence ======================================================== */

/*
 * One LEDC channel: the software-visible registers and what the comparator is running
 * on. pwm_led_esp32_set_cycles() (backends/.../drivers/pwm/pwm_led_esp32.c) for
 * 0 < pulse < period writes hpoint = 0, then DUTY, then sig_out_en, duty_start, para_up;
 * for pulse 0 or == period it stops the channel (sig_out_en 0, duty_start 0, para_up).
 * main.c motor_phase_apply_all() then (for each channel with sig_out_en and 0 < DUTY < P-1)
 * writes the packed hpoint
 * for the DUTY in the register, duty_start, para_up.
 *
 * Latch semantics, applied at every timer overflow (which may fall between ANY two writes):
 *   A  TRM: HPOINT/DUTY/sig_out_en latched at the overflow after para_up
 *   B  "live": whatever the registers hold at each overflow
 *   C  pessimistic, unverified: hpoint/sig_out_en latch at the para_up overflow, DUTY one
 *      overflow later (reported, not asserted -- it is not what the TRM says)
 */
struct ledc_ch {
	uint32_t hp, duty;
	bool en, para;
	uint32_t hpA, dA; bool enA;
	uint32_t hpB, dB; bool enB;
	uint32_t hpC, dC; bool enC; bool pendC; uint32_t dCnext;
};

static unsigned V_REG_A, V_REG_B;
static bool g_reg_reversed;
static unsigned long st_regC_overflows, st_reg_latches, st_reg_writes;
static const uint32_t REG_P = 2048u;

static void reg_check(unsigned i, const struct ledc_ch *c)
{
	if (c->enA && c->dA != 0u && c->hpA + c->dA > REG_P - 1u) {
		viol(V_REG_A, "ch%u latched (A) hpoint %u + duty %u > %u", i, c->hpA, c->dA,
		     REG_P - 1u);
	}
	if (c->enB && c->dB != 0u && c->hpB + c->dB > REG_P - 1u) {
		viol(V_REG_B, "ch%u latched (B) hpoint %u + duty %u > %u", i, c->hpB, c->dB,
		     REG_P - 1u);
	}
	if (c->enC && c->dC != 0u && c->hpC + c->dC > REG_P - 1u) {
		st_regC_overflows++;
	}
}

static void reg_overflow(struct ledc_ch ch[], unsigned n)
{
	for (unsigned i = 0; i < n; i++) {
		struct ledc_ch *c = &ch[i];

		/* C: last overflow's pending duty lands now */
		if (c->pendC) {
			c->dC = c->dCnext;
			c->pendC = false;
		}
		if (c->para) {
			c->hpA = c->hp; c->dA = c->duty; c->enA = c->en;
			c->hpC = c->hp; c->enC = c->en; c->dCnext = c->duty; c->pendC = true;
			c->para = false;
		}
		c->hpB = c->hp; c->dB = c->duty; c->enB = c->en;
		st_reg_latches++;
		reg_check(i, c);
	}
}

/* every register write passes through here: an overflow may precede it */
static void reg_maybe_overflow(struct ledc_ch ch[], unsigned n)
{
	st_reg_writes++;
	if (chance(250)) {
		reg_overflow(ch, n);
	}
}

static int run_regs(uint64_t seed, unsigned long updates)
{
	struct ledc_ch ch[SS_NMOTORS];
	uint16_t written[SS_NMOTORS] = { 0xFFFFu, 0xFFFFu, 0xFFFFu, 0xFFFFu };

	V_REG_A = vdef("regs_latched_at_para_up");
	V_REG_B = vdef("regs_live");
	g_rng = seed ? seed : 1u;
	memset(ch, 0, sizeof(ch));

	for (unsigned long u = 0; u < updates; u++) {
		const unsigned i = below(SS_NMOTORS);
		uint16_t d;

		switch (below(6u)) {
		case 0: d = 0u; break;
		case 1: d = 10000u; break;
		case 2: d = (uint16_t)(9990u + below(11u)); break;
		case 3: d = (uint16_t)(1u + below(3u)); break;
		default: d = (uint16_t)below(10001u); break;
		}
		if (d == written[i]) {
			continue;   /* main.c motors_apply(): write on change only */
		}
		written[i] = d;
		/* main.c: pulse in ns; Zephyr pwm_set: ns -> cycles at 80 MHz; period 4000 */
		const uint32_t pulse_ns = (uint32_t)((uint64_t)50000u * d / 10000u);
		const uint32_t pulse_cyc = (uint32_t)((uint64_t)pulse_ns * 80000000u / 1000000000u);
		struct ledc_ch *c = &ch[i];

		if (pulse_cyc == 0u || pulse_cyc == 4000u) {
			/* pwm_led_esp32_stop(): idle level, sig_out_en 0, duty_start 0, para_up */
			reg_maybe_overflow(ch, SS_NMOTORS);
			c->en = false;
			reg_maybe_overflow(ch, SS_NMOTORS);
			c->para = true;
		} else {
			const uint32_t dv = (uint32_t)((double)(1u << 11) *
						       ((double)pulse_cyc / (double)4000u));

			/* pwm_led_esp32_duty_set(): hpoint 0 FIRST, then the duty.
			 * g_reg_reversed models a driver that wrote them the other way
			 * round (reported as a STAT: why the order is pinned). */
			if (!g_reg_reversed) {
				reg_maybe_overflow(ch, SS_NMOTORS);
				c->hp = 0u;
				reg_maybe_overflow(ch, SS_NMOTORS);
				c->duty = dv;
			} else {
				reg_maybe_overflow(ch, SS_NMOTORS);
				c->duty = dv;
				reg_maybe_overflow(ch, SS_NMOTORS);
				c->hp = 0u;
			}
			/* pwm_led_esp32_start(): sig_out_en, duty_start, para_up */
			reg_maybe_overflow(ch, SS_NMOTORS);
			c->en = true;
			reg_maybe_overflow(ch, SS_NMOTORS);
			c->para = true;
		}
		/* main.c motors_apply(): any rewrite -> motor_phase_apply_all(): read every
		 * channel's sig_out_en and DUTY, place all four, and rewrite the hpoint (then
		 * duty_start + para_up) of each running channel whose hpoint differs */
		(void)pulse_ns;
		{
			uint32_t rd[SS_NMOTORS], hv[SS_NMOTORS];
			bool run[SS_NMOTORS];

			for (unsigned j = 0; j < SS_NMOTORS; j++) {
				run[j] = ch[j].en;
				rd[j] = run[j] ? ch[j].duty : 0u;
			}
			ss_phase_hpoints(rd, REG_P, hv);
			for (unsigned j = 0; j < SS_NMOTORS; j++) {
				if (!run[j] || rd[j] == 0u || rd[j] >= REG_P - 1u || ch[j].hp == hv[j]) {
					continue;
				}
				reg_maybe_overflow(ch, SS_NMOTORS);
				ch[j].hp = hv[j];
				reg_maybe_overflow(ch, SS_NMOTORS);
				ch[j].para = true;
			}
		}
		if (chance(400)) {
			reg_overflow(ch, SS_NMOTORS);   /* a full period passes */
		}
	}
	report();
	printf("STAT reg_writes %lu\n", st_reg_writes);
	printf("STAT reg_latches %lu\n", st_reg_latches);
	printf("STAT regC_split_latch_overflows %lu\n", st_regC_overflows);
	return 0;
}

int main(int argc, char **argv)
{
	if (argc >= 5 && strcmp(argv[1], "gate") == 0) {
		return run_gate(strtoull(argv[2], NULL, 0), strtoul(argv[3], NULL, 0), argv[4]);
	}
	if (argc >= 4 && strcmp(argv[1], "hpoint") == 0) {
		return run_hpoint(strtoull(argv[2], NULL, 0), strtoul(argv[3], NULL, 0));
	}
	if (argc >= 4 && strcmp(argv[1], "regs") == 0) {
		g_reg_reversed = argc >= 5 && strcmp(argv[4], "duty-first") == 0;
		return run_regs(strtoull(argv[2], NULL, 0), strtoul(argv[3], NULL, 0));
	}
	fprintf(stderr, "usage: %s gate <seed> <steps> mono|quant|back | hpoint <seed> <n> | "
		"regs <seed> <n>\n", argv[0]);
	return 2;
}
