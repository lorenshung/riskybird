/*
 * Motor spin-up gate ("soft start") and PWM phase interleave arithmetic.
 *
 * The last-stage authority on the ESP32-C6 that drives the four brushed motors
 * (workloads/esp_bridge --mode motors). Pure C, no Zephyr: tests/motor_softstart_check.c
 * compiles this on the host, so every rule below is a tested contract.
 *
 * Units: duty in 1/10000 (MOTOR_LINK_DUTY_FULL), time in microseconds as a
 * free-running uint32_t. All time comparisons use unsigned differences, so the
 * 71-minute wrap of a u32 microsecond clock is harmless as long as ss_step()
 * runs at least every few seconds (the link thread runs it every <= 5 ms).
 *
 * RULES
 *  1. Output <= command, always. The gate never raises a duty, and a command
 *     below the idle duty passes through unchanged.
 *  2. Decreases are never limited. ss_reset() (motors_off: disarm, kill,
 *     load session) puts every motor back to COLD at once. ss_cut() (the
 *     120 ms link failsafe) zeroes every output but does NOT reset the gate:
 *     a rotor 120 ms after a cut is still spinning, and re-limiting it to
 *     idle and staggering the four would cost authority and kick attitude
 *     in flight. Only a cut that lasts SS_CUT_COLD_MS -- by then the rotors
 *     have spun down and inrush matters again -- turns every motor COLD.
 *  3. Only cold rotors are limited. A motor is COLD at boot, after
 *     ss_reset(), and after a cut of >= SS_CUT_COLD_MS -- nothing else.
 *     While armed with the link up, a motor that
 *     has started once and then drops to 0 (mixer saturation, throttle chop)
 *     restarts unlimited however long it sat at 0: re-limiting it to idle, or
 *     staggering several such motors, would kick attitude in flight. A COLD
 *     motor whose command stays 0 stays COLD (no timer runs), so it gets its
 *     own full spin-up on its first nonzero command, however late after arming
 *     that is (e.g. the rig's left pair starting after the right pair).
 *  4. Time based, not frame based: the envelope is a function of time since
 *     the start, so producer frame rate and thread wake pattern do not matter.
 *  5. Bounded: a motor leaves spin-up after SS_MAX_US from ITS OWN cold start
 *     (its first nonzero command), whatever happened in between.
 *  6. Observable: per-motor phase and a count of limited ticks.
 *
 * SPIN-UP of a cold motor whose command becomes nonzero:
 *   WAIT  stagger: output 0 for rank * SS_STAGGER_US, where rank is the
 *         motor's order among the motors that cold-started in the SAME call
 *         (a lone start waits 0). Only after ss_reset() (boot, disarm, kill,
 *         load): a spin-up after a long failsafe cut starts every motor at
 *         rank 0, because a vehicle that has just lost >= SS_CUT_COLD_MS of
 *         control in the air must not also get 75 ms of lopsided thrust.
 *   HOLD  output min(cmd, SS_IDLE) for SS_HOLD_US.
 *   SLEW  output min(cmd, SS_IDLE + SS_RATE_PER_S * t_slew). Released to RUN
 *         only when the envelope itself reaches SS_FULL (full scale), or by
 *         rule 5. Reaching the command does NOT release it: a frame that
 *         happens to sit inside the envelope (e.g. 0.11 at 90 ms) must not let
 *         the next frame step unlimited to 0.9. For a producer that ramps no
 *         faster than the envelope this changes nothing (out == cmd, and
 *         lim_ticks only counts out < cmd).
 *   RUN   output = cmd, including after any number of drops to 0, until
 *         ss_reset() or a long cut.
 * Time never runs backwards inside the gate: every entry point clamps a `now`
 * that is up to SS_BACK_TOL_MS behind the latest time it has seen to that
 * time. A stale or reordered sample therefore counts as no time passing,
 * never as the ~71 min an unsigned now - t_start would read -- which used to
 * fire rule 5 and release a motor in one unlimited step. Anything further
 * "behind" is taken as a forward step across the u32 wrap, as before.
 * A 0 command during WAIT/HOLD/SLEW outputs 0 but keeps the spin-up timeline,
 * so a 0 blip cannot be used to skip the envelope. A short cut does the same.
 *
 * FAILSAFE CUT. ss_cut(now) latches the cut start (a second cut before the
 * resume keeps the first start: the rotors have been unpowered since then).
 * The caller zeroes its commands for the duration. The cut ends at
 * ss_resume(now), which the caller makes at the first frame that may drive
 * again, whatever its duties: if the cut lasted < SS_CUT_COLD_MS every motor
 * keeps its phase (RUN stays unlimited; a spin-up keeps its timeline), else
 * every motor is COLD. ss_step() applies the same rule on its own, so the
 * outcome does not depend on how often it ran during the cut, and a nonzero
 * command reaching ss_step() while a cut is latched ends the cut as
 * ss_resume() would. After the resume a motor at 0 is a commanded 0 (rule 3),
 * not a cut.
 *
 * Every number is #ifndef-overridable. In esp_bridge they are reachable from
 * the build line as CMake cache variables (see its CMakeLists.txt).
 */
#ifndef RISKYBIRD_MOTOR_SOFTSTART_H_
#define RISKYBIRD_MOTOR_SOFTSTART_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SS_NMOTORS 4

/* Idle (hold) duty, 1/10000. 10 %: the duty Dima chirps and flies at. */
#ifndef SS_IDLE
#define SS_IDLE 1000u
#endif
/* Hold at idle before the slew, ms. Lets back-EMF build. */
#ifndef SS_HOLD_MS
#define SS_HOLD_MS 80u
#endif
/* Slew rate in 1/10000 duty per second. 20000 = 2.0 duty/s. */
#ifndef SS_RATE_PER_S
#define SS_RATE_PER_S 20000u
#endif
/* Stagger between motors cold-starting in the same tick, ms (by rank). */
#ifndef SS_STAGGER_MS
#define SS_STAGGER_MS 25u
#endif
/* Hard bound on spin-up from the cold start (stagger included), ms. */
#ifndef SS_MAX_MS
#define SS_MAX_MS 700u
#endif
/* A failsafe cut that lasts this long turns every motor COLD (rule 2), ms.
 * Shorter, and the rotors are still spinning: they resume unlimited. */
#ifndef SS_CUT_COLD_MS
#define SS_CUT_COLD_MS 300u
#endif

/* Full scale in the same 1/10000 units (MOTOR_LINK_DUTY_FULL). SLEW releases to
 * RUN when the envelope reaches it. */
#ifndef SS_FULL
#define SS_FULL 10000u
#endif

/* How far behind the gate's clock a `now` may be and still be read as stale
 * (clamped) rather than as a forward step across the u32 wrap, ms. */
#ifndef SS_BACK_TOL_MS
#define SS_BACK_TOL_MS 1000u
#endif

#define SS_HOLD_US    ((uint32_t)SS_HOLD_MS * 1000u)
#define SS_BACK_TOL_US ((uint32_t)SS_BACK_TOL_MS * 1000u)
#define SS_STAGGER_US ((uint32_t)SS_STAGGER_MS * 1000u)
#define SS_MAX_US     ((uint32_t)SS_MAX_MS * 1000u)
#define SS_CUT_COLD_US ((uint32_t)SS_CUT_COLD_MS * 1000u)

/* Every *_US is compared as a signed 32-bit difference. */
#define SS_ASSERT_(c, m) typedef char ss_assert_##m[(c) ? 1 : -1]
SS_ASSERT_(SS_MAX_MS < 2000000u && SS_CUT_COLD_MS < 2000000u && SS_BACK_TOL_MS < 2000000u,
	   us_fit_int32);
SS_ASSERT_(SS_IDLE < SS_FULL && SS_RATE_PER_S > 0u, idle_and_rate);
/* Rule 5 must not release a rank-3 motor before its envelope reaches full scale:
 * that release would be one unlimited step. Defaults: 75 + 80 + 450 = 605 <= 700. */
SS_ASSERT_((uint64_t)SS_MAX_MS * 1000u >=
	   (uint64_t)3u * SS_STAGGER_MS * 1000u + (uint64_t)SS_HOLD_MS * 1000u +
	   ((uint64_t)(SS_FULL - SS_IDLE) * 1000000u + SS_RATE_PER_S - 1u) / SS_RATE_PER_S,
	   max_covers_full_envelope);

enum ss_phase {
	SS_COLD = 0,
	SS_WAIT = 1,
	SS_HOLD = 2,
	SS_SLEW = 3,
	SS_RUN  = 4,
};

struct ss_motor {
	uint8_t  ph;
	uint16_t out;
	uint32_t t_start;   /* cold start (rule 5 clock) */
	uint32_t t_go;      /* end of the stagger wait = start of HOLD */
};

struct ss {
	struct ss_motor m[SS_NMOTORS];
	uint32_t lim_ticks;   /* ss_step calls in which any output < its command */
	bool     cut;         /* a failsafe cut is latched (ss_cut .. ss_resume) */
	uint32_t t_cut;       /* when it started */
	bool     stagger;     /* next cold starts are staggered: set by ss_reset only */
	bool     t_seen;      /* t_hw is valid */
	uint32_t t_hw;        /* latest `now` seen: the gate's clock never runs backwards */
};

static inline uint32_t ss_min_u32(uint32_t a, uint32_t b)
{
	return a < b ? a : b;
}

/* The gate's clock: `now`, or the latest time already seen if `now` is up to
 * SS_BACK_TOL_US older (a stale or reordered sample, S3). */
static inline uint32_t ss_clock(struct ss *s, uint32_t now)
{
	if (s->t_seen && now != s->t_hw && (uint32_t)(s->t_hw - now) <= SS_BACK_TOL_US) {
		return s->t_hw;
	}
	s->t_hw = now;
	s->t_seen = true;
	return now;
}

static inline void ss_cold_all_(struct ss *s)
{
	for (unsigned i = 0; i < SS_NMOTORS; i++) {
		s->m[i].ph = SS_COLD;
		s->m[i].out = 0;
		s->m[i].t_start = 0;
		s->m[i].t_go = 0;
	}
	s->cut = false;
	s->t_cut = 0;
}

/* Everything COLD, outputs 0, no cut, next starts staggered. lim_ticks is a
 * lifetime counter and survives. Disarm, kill, load session (main.c motors_off()). */
static inline void ss_reset(struct ss *s)
{
	ss_cold_all_(s);
	s->stagger = true;
}

/* The failsafe cut the gates at `now` (rule 2). Outputs 0, phases kept; the
 * caller feeds 0 commands until ss_resume(). Latched once. */
static inline void ss_cut(struct ss *s, uint32_t now)
{
	now = ss_clock(s, now);
	for (unsigned i = 0; i < SS_NMOTORS; i++) {
		s->m[i].out = 0;
	}
	if (!s->cut) {
		s->cut = true;
		s->t_cut = now;
	}
}

/* Ends a latched cut that has lasted SS_CUT_COLD_US: every motor COLD, and
 * the spin-up that follows starts all four together (no stagger, rule S2).
 * `now` is already on the gate's clock (ss_clock). */
static inline void ss_cut_expire(struct ss *s, uint32_t now)
{
	if (s->cut && (uint32_t)(now - s->t_cut) >= SS_CUT_COLD_US) {
		ss_cold_all_(s);
		s->stagger = false;
	}
}

/* A frame that may drive arrived at `now`: the cut, if any, is over. Short:
 * phases kept. Long: every motor COLD. No-op without a cut. */
static inline void ss_resume(struct ss *s, uint32_t now)
{
	ss_cut_expire(s, ss_clock(s, now));
	s->cut = false;
}

static inline void ss_init(struct ss *s)
{
	ss_reset(s);
	s->lim_ticks = 0;
	s->t_seen = false;
	s->t_hw = 0;
}

/* The spin-up envelope of motor m at time now (only meaningful in WAIT/HOLD/SLEW). */
static inline uint32_t ss_envelope(const struct ss_motor *m, uint32_t now)
{
	uint32_t since_go;
	uint64_t slew;

	if ((int32_t)(now - m->t_go) < 0) {
		return 0;                         /* still waiting its turn */
	}
	since_go = now - m->t_go;
	if (since_go < SS_HOLD_US) {
		return SS_IDLE;
	}
	slew = (uint64_t)SS_RATE_PER_S * (uint64_t)(since_go - SS_HOLD_US) / 1000000u;
	if (slew > 0xFFFFu) {
		slew = 0xFFFFu;
	}
	return SS_IDLE + (uint32_t)slew;
}

/*
 * One service tick. tgt[] is the command (post ceiling, post kill: whatever
 * may actually reach a gate); out[] receives the gated duties. Never
 * out[i] > tgt[i].
 */
static inline void ss_step(struct ss *s, const uint16_t tgt[SS_NMOTORS],
			   uint16_t out[SS_NMOTORS], uint32_t now)
{
	unsigned rank = 0;   /* cold starters in this call */
	bool limited = false;

	now = ss_clock(s, now);   /* S3: never backwards */

	/* Rule 2: a cut that has lasted SS_CUT_COLD_US is COLD, however few
	 * steps ran during it. A nonzero command while still latched ends it. */
	ss_cut_expire(s, now);
	if (s->cut) {
		for (unsigned i = 0; i < SS_NMOTORS; i++) {
			if (tgt[i] != 0u) {
				s->cut = false;
				break;
			}
		}
	}

	for (unsigned i = 0; i < SS_NMOTORS; i++) {
		struct ss_motor *m = &s->m[i];
		const uint32_t t = tgt[i];

		if (t == 0u) {                                /* rule 2 */
			m->out = 0;
			out[i] = 0;
			/* rule 3: no rest timer -- COLD only via ss_reset() or a
			 * long cut. rule 5 still runs during a 0 blip in spin-up. */
			if (m->ph != SS_COLD && m->ph != SS_RUN &&
			    (uint32_t)(now - m->t_start) >= SS_MAX_US) {
				m->ph = SS_RUN;
			}
			continue;
		}

		if (m->ph == SS_COLD) {                       /* rule 3: only cold */
			m->ph = SS_WAIT;
			m->t_start = now;
			m->t_go = now + (uint32_t)rank * SS_STAGGER_US;
			if (s->stagger) {
				rank++;
			}
		}
		if (m->ph != SS_RUN && (uint32_t)(now - m->t_start) >= SS_MAX_US) {
			m->ph = SS_RUN;                       /* rule 5 */
		}

		if (m->ph == SS_RUN) {
			m->out = (uint16_t)t;
		} else {
			const uint32_t env = ss_envelope(m, now);
			const uint32_t since_go = now - m->t_go;

			m->out = (uint16_t)ss_min_u32(t, env);   /* rule 1 */
			if ((int32_t)(now - m->t_go) < 0) {
				m->ph = SS_WAIT;
			} else if (since_go < SS_HOLD_US) {
				m->ph = SS_HOLD;
			} else if (env >= SS_FULL) {
				m->ph = SS_RUN;               /* envelope at full scale */
			} else {
				m->ph = SS_SLEW;
			}
		}
		if (m->out < t) {
			limited = true;
		}
		out[i] = m->out;
	}
	if (limited) {
		s->lim_ticks++;
	}
}

/* Phase nibbles, motor 1 in the top nibble: printed %04x reads "m1 m2 m3 m4". */
static inline uint16_t ss_phase_word(const struct ss *s)
{
	uint16_t w = 0;

	for (unsigned i = 0; i < SS_NMOTORS; i++) {
		w = (uint16_t)((w << 4) | (s->m[i].ph & 0xFu));
	}
	return w;
}

/*
 * PWM PHASE INTERLEAVE: the hpoint of each of the four channels, given all four
 * duties d[] in timer counts and the timer period P = 2^duty_res counts.
 *
 * ESP32-C6 LEDC (TRM v1.2 sec. 35.3.3): output goes high when the timer
 * counter == Hpoint and low when it == Lpoint, Lpoint = DUTY[24:4] + HPOINT,
 * both latched at counter overflow; the counter runs 0 .. P-1 (sec. 35.3.2).
 * Nothing says Lpoint is reduced mod P, so Lpoint >= P would never match and
 * the output would stay HIGH -- a motor full on. Hence the hard rule here:
 * h[i] + d[i] <= P - 1 for every channel, and it depends on d[i] ALONE (each
 * term below is bounded by P-1-d[i] whatever the other duties are), so a
 * channel's (hpoint, duty) pair is valid even when a neighbour's duty in the
 * computation is stale or changes in the next period.
 *
 * PACKING (no wraparound). Two channels fill the period from the left, two
 * from the right:
 *   h0 = 0                       ch0 [0, d0)
 *   h1 = min(d0, P-1-d1)         ch1 starts where ch0 ends
 *   h2 = P-1-d2                  ch2 ends at P-1
 *   h3 = max(0, P-1-d2-d3)       ch3 ends where ch2 starts
 * With equal duties d the peak number of channels on at once is
 * ceil(4d / (P-1)) up to d = (P-1)/2 -- one FET to ~25 %, two all the way to
 * ~50 % -- which is what an ideal quarter-period wrap gives. Above (P-1)/2
 * every pulse must cover [P-1-d, d), so all four overlap there whatever the
 * placement; packing makes that the only 4-on window (2d - P + 1 counts) and
 * leaves two on elsewhere. Unequal duties, or channels at 0 / 100 %, still get
 * a sensible layout: a channel whose left (right) partner is idle is aligned
 * to the left (right) edge, so a short pulse never lands inside a long one
 * the way it could with a per-channel spread.
 * A channel with d == 0 or d >= P - 1 gets 0 (the driver's own path: the
 * caller does not touch those channels) and counts as idle for placement.
 */
static inline void ss_phase_hpoints(const uint32_t d[SS_NMOTORS], uint32_t P,
				    uint32_t h[SS_NMOTORS])
{
	uint32_t e[SS_NMOTORS];

	for (unsigned i = 0; i < SS_NMOTORS; i++) {
		e[i] = (P >= 2u && d[i] != 0u && d[i] < P - 1u) ? d[i] : 0u;
		h[i] = 0u;
	}
	if (P < 2u) {
		return;
	}
	if (e[1] != 0u) {
		h[1] = ss_min_u32(e[0], P - 1u - e[1]);
	}
	if (e[2] != 0u) {
		h[2] = P - 1u - e[2];
	}
	if (e[3] != 0u) {
		h[3] = (e[2] + e[3] <= P - 1u) ? P - 1u - e[2] - e[3] : 0u;
	}
}

#ifdef __cplusplus
}
#endif

#endif /* RISKYBIRD_MOTOR_SOFTSTART_H_ */
