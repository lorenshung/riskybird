/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Roll-harness ("rig") mode: gains, uplink commands, run state machine, excitation generator and
 * the roll control law. Interface and contracts: fc_rig.h. Compiled only when ROSE_RIG=1, and on
 * the host by tests/test_fc_rig.py -- so: no Zephyr, no heap, no exceptions, float math (double
 * only where a float would lose the answer: number parsing/printing and excitation phase).
 *
 * HOW THE DEFAULTS REPRODUCE THE FLOWN PID (controller_pid.cpp, ROSE_VEL_LOOP=0 so desRoll = 0):
 *
 *   flown:  rollRate_tgt = -(1/(PID_FLOWN_SCALE*TAU_ROLL)) * (estRoll - roll_tgt)
 *           rollRate_cmd = -(1/TAU_ROLLRATE) * (gyroX - rollRate_tgt)
 *           u[1]         = rollRate_cmd * J[0] * ATT_GAIN
 *   rig:    rate_tgt     = ATT_P  * (roll_tgt - roll) + ATT_I*int(e)
 *           rate_cmd     = RATE_P * (rate_tgt - gx_f) + RATE_I*int(.) - RATE_D * d(gx_f)/dt
 *           u[1]         = J[0] * ATT_GAIN * rate_cmd
 *
 *   so with ATT_P = 1/(PID_FLOWN_SCALE*TAU_ROLL) (= 5.0 at the default 0.10 s), RATE_P =
 *   1/TAU_ROLLRATE (= 40 at 0.025 s), ATT_GAIN = the build's ATT_GAIN, ROLL_TRIM = the build's
 *   PID_ANGLE(ROLL_TRIM_RAD), I = D = 0 and GYRO_LPF_HZ = 0 the two are the same expression, sign
 *   for sign. tests/test_fc_rig.py checks that against the formula above.
 *
 *   Back to a flight preset (default PID_FLOWN_SCALE = 2; under PID_TILT_TIGHT/PID_GIBBS_LEGACY it
 *   is 1 and the factor 2 below drops out):
 *       TAU_ROLL     = 1 / (2 * ATT_P)        TAU_ROLLRATE = 1 / RATE_P
 *       ATT_GAIN, MOTOR_TRIM_LEFT as they are; ROLL_TRIM_RAD = tan(ROLL_TRIM / 2) (Gibbs units).
 *   The flown PID has no roll I or D term and no gyro LPF: a tune that needs them does not
 *   transfer without adding them to controller_pid.cpp too.
 *
 * LIFT-OFF ASSIST (LIFT_I, LIFT_I_LIM; written for "the controlled axis": angle, rate, moment).
 *   A steady torque toward the stop the drone rests on (CoM off the pivot, a tethered cable) can
 *   exceed what the P path commands at the rest angle, and the integrators are gated on lift-off
 *   (integrate = RUN && guard_active) so they cannot build the trim that would lift it. In RUN
 *   BEFORE lift-off, with LIFT_I > 0 (fc_rig_outputs.lift_assist):
 *     - the ANGLE integrator (att_int; the rate integrator stays at 0) builds at LIFT_I * e * dt,
 *       clamped to +-min(LIFT_I_LIM, ATT_I_LIM), with conditional integration against the mixer's
 *       saturation AND the clamp below (assist_sat_*: the mixer is not saturated when only the
 *       clamp binds, so without them it would wind straight up to the limit);
 *     - the commanded moment is clamped to +-RIG_LIFT_MOMENT_FRAC * fc_rig_moment_authority(u0),
 *       well short of the 2x that takes one side to zero thrust.
 *   HAND-OFF at lift-off (guard goes live): att_int is KEPT as the initial ATT_I state; it is the
 *   trim the steady torque needs. ATT_I then continues from it under ATT_I_LIM (ATT_I = 0 holds it
 *   where it is). The assist's clamp is min(LIFT_I_LIM, ATT_I_LIM), so the value handed over is
 *   always inside ATT_I_LIM and the hand-off never steps. The moment clamp ends at the hand-off;
 *   it can only bind when P is large (far from level), and lift-off is near level. GAIN keeps its
 *   bumpless rebase in both phases (rebase_integrators clamps to the limit of the active phase).
 *   After lift-off nothing assist-specific runs: a swing past level reverses e, so ATT_I unwinds
 *   it, and the tilt guard catches a big overshoot. No lift-off by RIG_LIFTOFF_TIMEOUT_MS ramps
 *   down as before (the assist at a limit says so once: "RIG lift assist: at limit").
 *   att_int (dbg, recorder column att_int, already recorded) shows the build-up.
 */
#include "fc_rig.h"
#include "fc_shared_math.h"   /* fc_arm_dwell, fc_guard_dwell; PID_GIBBS_LEGACY / PID_TILT_TIGHT */

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

/* ---- the build's constants, with controller_pid.cpp's and main.cpp's own fallbacks ---------- */
#if PID_GIBBS_LEGACY
/* The rig compares TRUE radians (fc_pid_attitude) against radian limits; the legacy controller
 * hands out Gibbs parameters instead, and every limit below would be off by ~2x. */
#error "ROSE_RIG needs true-radian attitude: do not combine it with PID_GIBBS_LEGACY"
#endif
#if PID_TILT_TIGHT
#define RIG_FLOWN_SCALE 1.0f
#define RIG_ANGLE(x) (x)
#else
#define RIG_FLOWN_SCALE 2.0f            /* controller_pid.cpp's PID_FLOWN_SCALE */
#define RIG_ANGLE(x) (2.0f * atanf(x))  /* controller_pid.cpp's PID_ANGLE */
#endif
#ifndef TAU_ROLL
#define TAU_ROLL 0.10f
#endif
#ifndef TAU_ROLLRATE
#define TAU_ROLLRATE 0.025f
#endif
#ifndef ATT_GAIN
#define ATT_GAIN 1.0f
#endif
#ifndef MOTOR_TRIM_LEFT
#define MOTOR_TRIM_LEFT 0.87f
#endif
#ifndef PID_MASS_KG
#define PID_MASS_KG 0.032f
#endif
#ifndef ROLL_TRIM_RAD
#define ROLL_TRIM_RAD 0.0f
#endif
#ifndef PITCH_TRIM_RAD
#define PITCH_TRIM_RAD 0.0f
#endif
#ifndef YAW_MIX_SIGN
#define YAW_MIX_SIGN 1.0f
#endif
#ifndef ARM_MAX_RATE_RADPS
#define ARM_MAX_RATE_RADPS 0.30f        /* main.cpp: must be still to arm */
#endif
#ifndef SAFE_DEBOUNCE_MS
#define SAFE_DEBOUNCE_MS 120
#endif
#ifndef SAFE_DEBOUNCE_MIN_SAMPLES
#define SAFE_DEBOUNCE_MIN_SAMPLES 2
#endif

/* controller_pid.cpp's rigid-body constants (static there, so copied; same values). */
static const float rig_gravity  = 9.81f;
static const float rig_J[3]     = {16e-6f, 16e-6f, 29e-6f};
static const float rig_l_arm    = 33e-3f;
static const float rig_tau_yaw  = 0.25f;
static const float rig_tau_yawrate = 0.05f;

/* ---- the controlled axis (RIG_AXIS) ----------------------------------------------------------
 * 0 = roll (rig-roll, the default), 1 = pitch (rig-pitch: -DRIG_AXIS=1, the drone turned 90 deg in
 * the cradle). Compile-time only, so a roll build compiles exactly the roll code it always had. The
 * interfaces stay PHYSICAL either way (fc_rig_inputs roll_rad/pitch_rad/gyro[3], fc_rig_law_in
 * roll/pitch/gyro[3], dbg.roll/dbg.pitch); only what the law, the guards and the saturation flags
 * read and write moves. The names that say "roll" (sp_roll*, roll_tgt, the ROLL_TRIM gain,
 * fc_rig_roll_law, the guard_active "roll guard") are kept for the host's RBPOKE / recorder / wire
 * contracts and mean "the controlled axis". In a pitch build:
 *   - law: angle = pitch, rate = gyro[1], target = setpoint + ROLL_TRIM (default PITCH_TRIM_RAD) +
 *     exc(att); moment = J[1] * ATT_GAIN * rate_cmd into u[2], u[1] = 0. That is controller_pid.cpp's
 *     pitch path sign for sign: pitchRate_tgt = -(1/(S*tau_pitch)) * (estPitch - pitch_tgt),
 *     pitchRate_cmd = -(1/tau_pitchRate) * (gyroY - pitchRate_tgt), u[2] = pitchRate_cmd*J[1]*ATT_GAIN,
 *     with tau_pitch = TAU_ROLL, tau_pitchRate = TAU_ROLLRATE (so ATT_P/RATE_P defaults are unchanged);
 *   - dbg.pitch_cmd is the would-be command on the LOCKED axis (roll there: the flown roll path);
 *   - +u[2] raises duty[1]/duty[2] (rear) and lowers duty[0]/duty[3] (front): M's column 2 is
 *     {-, -, +, +} on ctrl[0..3] and duty[0..3] <- ctrl[1], ctrl[2], ctrl[3], ctrl[0]. The
 *     saturation flags (anti-windup) use those pairs;
 *   - the tilt guard, lift-off band, lift-off timeout and lift-off assist act on pitch;
 *     RIG_PITCH_MAX_RAD (name kept) is the LOCKED-axis guard, i.e. it guards ROLL;
 *   - MOTOR_TRIM_LEFT still scales the LEFT pair (duty[2]/duty[3]): one rear and one front motor, so
 *     on the pitch rig it trims the locked roll axis and adds no pitch moment. */
#ifndef RIG_AXIS
#define RIG_AXIS 0
#endif
#if RIG_AXIS == 1
#define RIG_CTL            1            /* gyro[] / rig_J[] index of the controlled axis */
#define RIG_LOCK           0            /* ... and of the locked one                     */
#define RIG_CTL_ANG(x)     ((x)->pitch)
#define RIG_LOCK_ANG(x)    ((x)->roll)
#define RIG_CTL_RAD(x)     ((x)->pitch_rad)
#define RIG_LOCK_RAD(x)    ((x)->roll_rad)
#define RIG_CTL_TRIM_RAD   PITCH_TRIM_RAD
#define RIG_LOCK_TRIM_RAD  ROLL_TRIM_RAD
#define RIG_CTL_NAME       "pitch"
#define RIG_LOCK_NAME      "roll"
/* duty[] indices of the pair the controlled moment raises (P) and lowers (M) */
#define RIG_SAT_P0 1
#define RIG_SAT_P1 2
#define RIG_SAT_M0 0
#define RIG_SAT_M1 3
#elif RIG_AXIS == 0
#define RIG_CTL            0
#define RIG_LOCK           1
#define RIG_CTL_ANG(x)     ((x)->roll)
#define RIG_LOCK_ANG(x)    ((x)->pitch)
#define RIG_CTL_RAD(x)     ((x)->roll_rad)
#define RIG_LOCK_RAD(x)    ((x)->pitch_rad)
#define RIG_CTL_TRIM_RAD   ROLL_TRIM_RAD
#define RIG_LOCK_TRIM_RAD  PITCH_TRIM_RAD
#define RIG_CTL_NAME       "roll"
#define RIG_LOCK_NAME      "pitch"
#define RIG_SAT_P0 2
#define RIG_SAT_P1 3
#define RIG_SAT_M0 0
#define RIG_SAT_M1 1
#else
#error "RIG_AXIS must be 0 (roll) or 1 (pitch)"
#endif

/* The build's axis for the host (tools/rb/rig_axis.py reads it from the ELF): external linkage, in
 * read-only data, and read at run time by main.cpp's RIG line (ax=) so --gc-sections keeps it. */
extern "C" const uint8_t g_rig_axis = RIG_AXIS;

/* ---- rig knobs (spec defaults; all overridable with -D) ------------------------------------ */
#ifndef RIG_ARM_MAX_TILT_RAD
#define RIG_ARM_MAX_TILT_RAD   0.61f    /* 35 deg: may arm resting on a ~30 deg stop */
#endif
#ifndef RIG_PITCH_MAX_RAD
#define RIG_PITCH_MAX_RAD      0.17f    /* 10 deg: cradle/rig failure */
#endif
#ifndef RIG_MAX_TILT_RAD
#define RIG_MAX_TILT_RAD       0.44f    /* 25 deg, inside the 30 deg stop */
#endif
/* The roll ANGLE TARGET (setpoint + ROLL_TRIM + att excitation) never leaves +-this: the law clamps
 * it, and RSP / ROLL_TRIM / EXC att are refused when their worst case would need more. Default
 * 0.35 rad (20 deg), 5 deg inside the tilt guard. A stand whose stops sit lower (rigid PLA blockers
 * at ~15.6 deg) wants a preset value below its stop angle, e.g. -DRIG_TGT_MAX_RAD=0.22f. */
#ifndef RIG_TGT_MAX_RAD
#define RIG_TGT_MAX_RAD        (RIG_MAX_TILT_RAD - 0.09f)
#endif
static_assert(RIG_TGT_MAX_RAD > 0.0f && RIG_TGT_MAX_RAD < RIG_MAX_TILT_RAD,
	      "RIG_TGT_MAX_RAD must lie inside the tilt guard RIG_MAX_TILT_RAD");
#ifndef RIG_LIFTOFF_MARGIN_RAD
#define RIG_LIFTOFF_MARGIN_RAD 0.05f    /* guard goes live once |roll| < MAX_TILT - this */
#endif
#ifndef RIG_LIFTOFF_TIMEOUT_MS
#define RIG_LIFTOFF_TIMEOUT_MS 2000     /* after the end of RAMP_UP */
#endif
#ifndef RIG_ARM_SETTLE_MS
#define RIG_ARM_SETTLE_MS      1000
#endif
#ifndef RIG_RAMP_MS
#define RIG_RAMP_MS            1500
#endif
#ifndef RIG_SP_SLEW_RADPS
#define RIG_SP_SLEW_RADPS      0.5f
#endif
#ifndef RIG_COLL_SLEW
#define RIG_COLL_SLEW          0.5f     /* collective fraction per second, in RUN */
#endif
#ifndef RIG_TRIM_SLEW
#define RIG_TRIM_SLEW          0.5f     /* MOTOR_TRIM_LEFT applied value, per second, motors live */
#endif
#ifndef RIG_DT_MAX_S
#define RIG_DT_MAX_S           0.05f    /* a longer (or non-finite) tick is taken as this for slews */
#endif
#ifndef RIG_HB_RAMP_MS
#define RIG_HB_RAMP_MS         300
#endif
#ifndef RIG_HB_ESTOP_MS
#define RIG_HB_ESTOP_MS        1000
#endif
#ifndef RIG_MAX_RUN_CAP_MS
#define RIG_MAX_RUN_CAP_MS     60000
#endif
#ifndef RIG_MAX_RUN_MS
#define RIG_MAX_RUN_MS         30000    /* default max_run_ms, from the arm tick */
#endif
#ifndef RIG_RUNMS_MIN
#define RIG_RUNMS_MIN          1000
#endif
#ifndef RIG_COLL_MAX_PERMILLE
#define RIG_COLL_MAX_PERMILLE  1300
#endif
#ifndef RIG_SP_MAX_MDEG
#define RIG_SP_MAX_MDEG        20000
#endif
#ifndef RIG_EXC_MOMENT_FRAC
#define RIG_EXC_MOMENT_FRAC    0.3f
#endif
#ifndef RIG_EXC_ATT_MAX_RAD
#define RIG_EXC_ATT_MAX_RAD    0.26f
#endif
#ifndef RIG_EXC_RATE_MAX_RADPS
#define RIG_EXC_RATE_MAX_RADPS 3.0f
#endif
#ifndef RIG_EXC_F_MIN_MHZ
#define RIG_EXC_F_MIN_MHZ      10       /* 0.01 Hz */
#endif
#ifndef RIG_EXC_F_MAX_MHZ
#define RIG_EXC_F_MAX_MHZ      50000    /* 50 Hz: well under the ~1 kHz loop */
#endif
#ifndef RIG_LIFT_MOMENT_FRAC
#define RIG_LIFT_MOMENT_FRAC   1.0f     /* lift assist: |moment| <= this * fc_rig_moment_authority */
#endif
#ifndef RIG_EXC_TAPER_FRAC
#define RIG_EXC_TAPER_FRAC     0.1f     /* chirp/sine cosine taper, each end, fraction of dur */
#endif

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
/* Finite: false for NaN and +-Inf. Written as a comparison so NaN fails it (no -ffast-math here). */
static bool fin(float x) { return fabsf(x) <= FLT_MAX; }

/* ==== gains ================================================================================== */
struct gain_info { const char *name; float min, max; };

/* Ranges: wide enough to tune in, narrow enough that a typo (an extra digit, a missing point)
 * NAKs instead of flying. Every default must lie inside its range (a test checks it). */
static const struct gain_info gain_table[FC_RIG_GAIN_COUNT] = {
	/* FC_RIG_ATT_P          */ {"ATT_P",           0.0f,   40.0f},
	/* FC_RIG_ATT_I          */ {"ATT_I",           0.0f,   50.0f},
	/* FC_RIG_RATE_P         */ {"RATE_P",          1.0f,  200.0f},
	/* FC_RIG_RATE_I         */ {"RATE_I",          0.0f,  500.0f},
	/* FC_RIG_RATE_D         */ {"RATE_D",          0.0f,    2.0f},
	/* FC_RIG_ATT_GAIN       */ {"ATT_GAIN",        0.1f,    3.0f},
	/* FC_RIG_D_LPF_HZ       */ {"D_LPF_HZ",        1.0f,  200.0f},
	/* FC_RIG_GYRO_LPF_HZ    */ {"GYRO_LPF_HZ",     0.0f,  200.0f},
	/* FC_RIG_ROLL_TRIM      */ {"ROLL_TRIM",      -0.175f,  0.175f},
	/* FC_RIG_MOTOR_TRIM_LEFT*/ {"MOTOR_TRIM_LEFT", 0.7f,    1.3f},
	/* FC_RIG_ATT_I_LIM      */ {"ATT_I_LIM",       0.0f,   10.0f},
	/* FC_RIG_RATE_I_LIM     */ {"RATE_I_LIM",      0.0f,  100.0f},
	/* FC_RIG_LIFT_I         */ {"LIFT_I",          0.0f,   10.0f},   /* twin: 12 swings ~13 deg past level */
	/* FC_RIG_LIFT_I_LIM     */ {"LIFT_I_LIM",      0.0f,   10.0f},
};

const char *fc_rig_gain_name(int id)
{
	return (id >= 0 && id < FC_RIG_GAIN_COUNT) ? gain_table[id].name : NULL;
}

int fc_rig_gain_find(const char *name)
{
	for (int i = 0; i < FC_RIG_GAIN_COUNT; i++) {
		if (strcmp(name, gain_table[i].name) == 0) {
			return i;
		}
	}
	return -1;
}

void fc_rig_gain_range(int id, float *min, float *max)
{
	if (id < 0 || id >= FC_RIG_GAIN_COUNT) {
		*min = 0.0f; *max = -1.0f;   /* empty */
		return;
	}
	*min = gain_table[id].min;
	*max = gain_table[id].max;
}

void fc_rig_gains_default(struct fc_rig_gains *g)
{
	g->v[FC_RIG_ATT_P]           = 1.0f / (RIG_FLOWN_SCALE * (float)(TAU_ROLL));
	g->v[FC_RIG_ATT_I]           = 0.0f;
	g->v[FC_RIG_RATE_P]          = 1.0f / (float)(TAU_ROLLRATE);
	g->v[FC_RIG_RATE_I]          = 0.0f;
	g->v[FC_RIG_RATE_D]          = 0.0f;
	g->v[FC_RIG_ATT_GAIN]        = (float)(ATT_GAIN);
	g->v[FC_RIG_D_LPF_HZ]        = 30.0f;
	g->v[FC_RIG_GYRO_LPF_HZ]     = 0.0f;
	g->v[FC_RIG_ROLL_TRIM]       = RIG_ANGLE((float)(RIG_CTL_TRIM_RAD));   /* the controlled axis' trim */
	g->v[FC_RIG_MOTOR_TRIM_LEFT] = (float)(MOTOR_TRIM_LEFT);
	/* ATT_I_LIM >= LIFT_I_LIM, or the assist's clamp is ATT_I_LIM. Lift-assist defaults from the
	 * twin (tools/rig/sim rig_sim.py liftoff-assist): 3-4 mN*m toward the stop needs ~4-7 rad/s
	 * of att_int (ATT_GAIN 1.2-0.85 at RATE_P 40); LIFT_I 4 lifts off in 2-7 s from arming over
	 * the plant box with <~1 deg swing past level, LIFT_I >= 12 swings 10+ deg past it. */
	g->v[FC_RIG_ATT_I_LIM]       = 8.0f;
	g->v[FC_RIG_RATE_I_LIM]      = 30.0f;
	g->v[FC_RIG_LIFT_I]          = 4.0f;
	g->v[FC_RIG_LIFT_I_LIM]      = 8.0f;
}

int fc_rig_gain_set(struct fc_rig_gains *g, const char *name, float value)
{
	const int id = fc_rig_gain_find(name);
	if (id < 0) {
		return -1;
	}
	/* written so NaN fails too */
	if (!(value >= gain_table[id].min && value <= gain_table[id].max)) {
		return -2;
	}
	g->v[id] = value;
	return id;
}

/* ==== numbers ================================================================================ */
static bool is_digit(char c) { return c >= '0' && c <= '9'; }

bool fc_rig_parse_decimal(const char *s, float *out)
{
	if (s == NULL) {
		return false;
	}
	bool neg = false;
	if (*s == '+' || *s == '-') {
		neg = (*s == '-');
		s++;
	}
	/* The whole number is read as one integer mantissa and a power of ten, then divided once in
	 * double (exact for these sizes, correctly rounded), then rounded once to float. Summing
	 * digit by digit in float would drift. At most 15 digits keep the mantissa exact in a double. */
	int64_t mant = 0;
	int idig = 0, fdig = 0;
	while (is_digit(*s)) {
		if (++idig > 9) {
			return false;
		}
		mant = mant * 10 + (*s - '0');
		s++;
	}
	if (idig == 0) {
		return false;       /* "", "-", ".5" */
	}
	if (*s == '.') {
		s++;
		while (is_digit(*s)) {
			if (++fdig > 12 || idig + fdig > 15) {
				return false;   /* mantissa stays exact in a double */
			}
			mant = mant * 10 + (*s - '0');
			s++;
		}
		if (fdig == 0) {
			return false;   /* "5." */
		}
	}
	if (*s != '\0') {
		return false;       /* exponent, junk, blanks */
	}
	double p = 1.0;
	for (int i = 0; i < fdig; i++) {
		p *= 10.0;
	}
	double v = (double)mant / p;
	*out = (float)(neg ? -v : v);
	return true;
}

bool fc_rig_parse_int(const char *s, int32_t *out)
{
	if (s == NULL) {
		return false;
	}
	bool neg = false;
	if (*s == '+' || *s == '-') {
		neg = (*s == '-');
		s++;
	}
	int32_t v = 0;
	int n = 0;
	while (is_digit(*s)) {
		if (++n > 9) {
			return false;   /* keeps v inside int32 */
		}
		v = v * 10 + (*s - '0');
		s++;
	}
	if (n == 0 || *s != '\0') {
		return false;
	}
	*out = neg ? -v : v;
	return true;
}

/* Append-only string builder: never overruns, remembers whether anything was cut. */
struct sbuf { char *p; size_t n, len; bool overflow; };

static void sb_init(struct sbuf *b, char *p, size_t n)
{
	b->p = p; b->n = n; b->len = 0; b->overflow = (n == 0);
	if (n > 0) {
		p[0] = '\0';
	}
}

static void sb_putc(struct sbuf *b, char c)
{
	if (b->len + 1 < b->n) {
		b->p[b->len++] = c;
		b->p[b->len] = '\0';
	} else {
		b->overflow = true;
	}
}

static void sb_puts(struct sbuf *b, const char *s)
{
	while (*s) {
		sb_putc(b, *s++);
	}
}

static void sb_putu(struct sbuf *b, uint64_t v)
{
	char tmp[24];
	int k = 0;
	do {
		tmp[k++] = (char)('0' + (int)(v % 10u));
		v /= 10u;
	} while (v != 0u);
	while (k > 0) {
		sb_putc(b, tmp[--k]);
	}
}

/* v as fixed-point with `sig` significant digits (at most 12 decimals). Integer digits only, no %f. */
static void fmt_sig(float v, int sig, char *out, size_t n)
{
	struct sbuf b;
	sb_init(&b, out, n);
	double a = fabs((double)v);
	if (!(a < 999999999.0)) {
		a = 999999999.0;      /* not a gain; also catches inf/NaN */
	}
	int dec = 0;
	if (a > 0.0) {
		dec = sig - 1 - (int)floor(log10(a));
		dec = dec < 0 ? 0 : (dec > 12 ? 12 : dec);
	}
	double p = 1.0;
	for (int i = 0; i < dec; i++) {
		p *= 10.0;
	}
	const uint64_t scaled = (uint64_t)floor(a * p + 0.5);
	const uint64_t ip = (uint64_t)p;
	uint64_t whole = scaled / ip, frac = scaled % ip;
	if (v < 0.0f && scaled != 0u) {
		sb_putc(&b, '-');
	}
	sb_putu(&b, whole);
	if (frac == 0u) {
		return;
	}
	char digs[12];
	for (int i = dec - 1; i >= 0; i--) {
		digs[i] = (char)('0' + (int)(frac % 10u));
		frac /= 10u;
	}
	int last = dec - 1;
	while (last >= 0 && digs[last] == '0') {
		last--;
	}
	sb_putc(&b, '.');
	for (int i = 0; i <= last; i++) {
		sb_putc(&b, digs[i]);
	}
}

/* The SHORTEST decimal that parses back to exactly v: 0.85f prints "0.85", not "0.850000024".
 * Nine significant digits always round-trip a float, so the search ends by then. */
static void sb_putdec(struct sbuf *b, float v)
{
	char tmp[32];
	for (int sig = 1; sig <= 9; sig++) {
		float back;
		fmt_sig(v, sig, tmp, sizeof(tmp));
		if (fc_rig_parse_decimal(tmp, &back) && back == v) {
			break;
		}
	}
	sb_puts(b, tmp);
}

int fc_rig_format_decimal(float v, char *out, size_t n)
{
	struct sbuf b;
	sb_init(&b, out, n);
	sb_putdec(&b, v);
	if (b.overflow) {
		if (n > 0) {
			out[0] = '\0';
		}
		return -1;
	}
	return (int)b.len;
}

int fc_rig_gains_format(const struct fc_rig_gains *g, uint32_t gain_seq, char *out, size_t n)
{
	struct sbuf b;
	sb_init(&b, out, n);
	sb_puts(&b, "RIGGAINS gs=");   /* gs= : RIG-SPEC host contract */
	sb_putu(&b, gain_seq);
	for (int i = 0; i < FC_RIG_GAIN_COUNT; i++) {
		sb_putc(&b, ' ');
		sb_puts(&b, gain_table[i].name);
		sb_putc(&b, '=');
		sb_putdec(&b, g->v[i]);
	}
	sb_putc(&b, '\n');
	if (b.overflow) {
		if (n > 0) {
			out[0] = '\0';
		}
		return -1;
	}
	return (int)b.len;
}

/* ==== excitation ============================================================================= */
static const double rig_two_pi = 6.283185307179586;

/* Cosine (Tukey) taper: 0 at both ends, 1 in the middle 1 - 2*RIG_EXC_TAPER_FRAC of the run. */
static float taper(int64_t t_ms, int32_t dur_ms)
{
	const float tt = RIG_EXC_TAPER_FRAC * (float)dur_ms;
	const float t = (float)t_ms;
	const float tail = (float)dur_ms - t;
	if (tt <= 0.0f) {
		return 1.0f;
	}
	if (t < tt) {
		return 0.5f - 0.5f * cosf(3.14159265f * t / tt);
	}
	if (tail < tt) {
		return 0.5f - 0.5f * cosf(3.14159265f * tail / tt);
	}
	return 1.0f;
}

/* 9-bit maximal LFSR, x^9 + x^5 + 1 (period 511). Fixed seed: every PRBS run is the same sequence. */
#define RIG_PRBS_SEED 0x1FFu
static uint16_t prbs_next(uint16_t r)
{
	const uint16_t bit = (uint16_t)(((r >> 8) ^ (r >> 4)) & 1u);
	return (uint16_t)(((r << 1) | bit) & 0x1FFu);
}

float fc_rig_exc_shape(struct fc_rig_exc *e, int64_t t_ms)
{
	if (e->type == FC_RIG_EXC_NONE || t_ms < 0 || t_ms >= (int64_t)e->dur_ms) {
		return 0.0f;
	}
	const double t = (double)t_ms * 1e-3;
	const double T = (double)e->dur_ms * 1e-3;
	switch (e->type) {
	case FC_RIG_EXC_STEP:
		return e->amp;
	case FC_RIG_EXC_DOUBLET:
		/* +amp then -amp, half and half: net zero area, so the angle comes back. */
		return (2 * t_ms < (int64_t)e->dur_ms) ? e->amp : -e->amp;
	case FC_RIG_EXC_SINE: {
		/* Phase in cycles, reduced before the float sin so 60 s at 50 Hz keeps its precision. */
		double cyc = t * (double)e->f0_hz;
		cyc -= floor(cyc);
		return e->amp * taper(t_ms, e->dur_ms) * (float)sin(rig_two_pi * cyc);
	}
	case FC_RIG_EXC_CHIRP: {
		/* Exponential (log) sweep f(t) = f0 * k^(t/T), k = f1/f0. Phase in cycles is the integral:
		 * f0*T/ln(k) * (k^(t/T) - 1); a linear phase when f0 == f1. */
		const double f0 = (double)e->f0_hz, f1 = (double)e->f1_hz;
		double cyc;
		if (fabs(f1 - f0) < 1e-9) {
			cyc = f0 * t;
		} else {
			const double lk = log(f1 / f0);
			cyc = f0 * T / lk * (exp(lk * t / T) - 1.0);
		}
		cyc -= floor(cyc);
		return e->amp * taper(t_ms, e->dur_ms) * (float)sin(rig_two_pi * cyc);
	}
	case FC_RIG_EXC_PRBS: {
		/* Bit period = 1/f0. The register advances to the bit that holds at t; t only moves
		 * forward within one excitation, so this is O(1) per tick. */
		/* bit index = floor(t * f0), nudged so t = an exact multiple of the bit period lands on
		 * the new bit despite 1/f0 not being exact in binary */
		const int32_t idx = (int32_t)floor(t * (double)e->f0_hz + 1e-6);
		if (idx < e->prbs_bit) {
			/* time went backwards (only a caller bug can do this): restart deterministically */
			e->prbs_reg = RIG_PRBS_SEED;
			e->prbs_bit = 0;
		}
		while (e->prbs_bit < idx) {
			e->prbs_reg = prbs_next(e->prbs_reg);
			e->prbs_bit++;
		}
		return (e->prbs_reg & 1u) ? e->amp : -e->amp;
	}
	default:
		return 0.0f;
	}
}

float fc_rig_hover_thrust(void)
{
	return (float)(PID_MASS_KG) * rig_gravity;
}

float fc_rig_moment_authority(float coll_thrust)
{
	/* The spec's 2 * 0.25*u0 * l_arm. NOTE: with controller_pid.cpp's mixer one side reaches zero
	 * thrust at u1 = u0 * l_arm, twice this; the spec's (smaller, conservative) figure is kept. */
	const float u0 = coll_thrust > 0.0f ? coll_thrust : 0.0f;
	return 2.0f * 0.25f * u0 * rig_l_arm;
}

/* Largest |value| an excitation on this target may inject right now. The moment cap scales with
 * the collective; so does the rate cap: a rate-target excitation reaches the mixer as
 * RATE_P * amp * J0 * ATT_GAIN, which may not exceed the moment cap either (and never 3 rad/s). */
static float exc_cap(uint8_t target, float coll_thrust, const struct fc_rig_gains *g)
{
	const float m_cap = RIG_EXC_MOMENT_FRAC * fc_rig_moment_authority(coll_thrust);
	switch (target) {
	case FC_RIG_TGT_ATT:    return RIG_EXC_ATT_MAX_RAD;
	case FC_RIG_TGT_RATE: {
		const float per = g->v[FC_RIG_RATE_P] * rig_J[RIG_CTL] * g->v[FC_RIG_ATT_GAIN];   /* > 0 by range */
		const float c = m_cap / per;
		return c < RIG_EXC_RATE_MAX_RADPS ? c : RIG_EXC_RATE_MAX_RADPS;
	}
	case FC_RIG_TGT_MOMENT: return m_cap;
	default:                return 0.0f;
	}
}

/* ==== roll control law ======================================================================= */
/* First-order low-pass coefficient for cutoff f at step dt; 1 (= pass-through) when f <= 0. */
static float lpf_alpha(float dt, float f_hz)
{
	if (f_hz <= 0.0f || dt <= 0.0f) {
		return 1.0f;
	}
	const float rc = 1.0f / (2.0f * 3.14159265f * f_hz);
	return dt / (dt + rc);
}

void fc_rig_law_reset(struct fc_rig_law_state *ls)
{
	ls->att_int = 0.0f;
	ls->rate_int = 0.0f;
	ls->gx_f = 0.0f;
	ls->gx_prev = 0.0f;
	ls->dgx_f = 0.0f;
	ls->seeded = false;
	ls->nonfinite = false;
	ls->assist_sat_pos = false;
	ls->assist_sat_neg = false;
}

/* The angle integrator's clamp: ATT_I_LIM after lift-off, min(LIFT_I_LIM, ATT_I_LIM) while the
 * lift assist builds it (so what it hands over is always inside ATT_I_LIM). */
static float att_int_lim(const struct fc_rig_gains *g, bool lift_assist)
{
	const float al = g->v[FC_RIG_ATT_I_LIM], ll = g->v[FC_RIG_LIFT_I_LIM];
	return (lift_assist && ll < al) ? ll : al;
}

/* Conditional integration: skip an increment that pushes further into a saturated direction.
 * Both integrators raise the roll moment when they grow (positive gains), so the same test
 * serves both. */
static float integrate_clamped(float acc, float inc, float lim, bool sat_pos, bool sat_neg)
{
	if (!((inc > 0.0f && sat_pos) || (inc < 0.0f && sat_neg))) {
		acc += inc;
	}
	return clampf(acc, -lim, lim);
}

void fc_rig_roll_law(struct fc_rig_law_state *ls, const struct fc_rig_gains *g,
		     const struct fc_rig_law_in *in, float u[4], struct fc_rig_dbg *dbg)
{
	const float *k = g->v;
	/* Non-finite input (a NaN gyro sample, an Inf attitude): command NO roll moment and drop the
	 * filter and integrator state, so the NaN cannot latch in gx_f / dgx_f / the integrators and
	 * the next clean tick starts fresh. fc_rig_step estops on the same inputs while armed. */
	if (!(fin(in->roll) && fin(in->pitch) && fin(in->yaw_err) && fin(in->gyro[0]) && fin(in->gyro[1]) &&
	      fin(in->gyro[2]) && fin(in->dt) && fin(in->sp_roll) && fin(in->exc_val) && fin(in->coll_thrust))) {
		fc_rig_law_reset(ls);
		ls->nonfinite = true;          /* after the reset: fc_rig_step turns it into an estop */
		u[0] = fin(in->coll_thrust) ? in->coll_thrust : 0.0f;
		u[1] = u[2] = u[3] = 0.0f;
		if (dbg != NULL) {
			memset(dbg, 0, sizeof(*dbg));
			dbg->roll = in->roll;          /* kept as given: the recorder shows what tripped it */
			dbg->pitch = in->pitch;
			dbg->coll_thrust = u[0];
		}
		return;
	}
	const float dt = in->dt > 0.0f ? in->dt : 1e-3f;
	const float exc_att    = in->exc_target == FC_RIG_TGT_ATT    ? in->exc_val : 0.0f;
	const float exc_rate   = in->exc_target == FC_RIG_TGT_RATE   ? in->exc_val : 0.0f;
	const float exc_moment = in->exc_target == FC_RIG_TGT_MOMENT ? in->exc_val : 0.0f;

	/* Gyro into the rate loop: optional first-order LPF; its derivative for D, low-passed again. */
	const float gx = in->gyro[RIG_CTL];   /* the controlled axis' rate (gyro y on the pitch rig) */
	if (!ls->seeded) {
		ls->gx_f = gx;
		ls->gx_prev = gx;
		ls->dgx_f = 0.0f;
		ls->seeded = true;
	} else {
		ls->gx_f += lpf_alpha(dt, k[FC_RIG_GYRO_LPF_HZ]) * (gx - ls->gx_f);
		const float d = (ls->gx_f - ls->gx_prev) / dt;
		ls->dgx_f += lpf_alpha(dt, k[FC_RIG_D_LPF_HZ]) * (d - ls->dgx_f);
		ls->gx_prev = ls->gx_f;
	}

	/* Angle loop. controller_pid.cpp: rollRate_tgt = -(1/(S*tau)) * (est - tgt) == ATT_P*(tgt - est). */
	/* Clamped to +-RIG_TGT_MAX_RAD, inside the tilt guard and the stops, whatever RSP, ROLL_TRIM and an
	 * att excitation add up to. Right after arming on a stop beyond it (sp_roll starts at the rest
	 * angle) this steps the target in to the bound at ~zero collective; the setpoint itself still
	 * slews down from the rest angle (fc_rig_step), so dbg.roll_tgt and out.sp_roll differ then. */
	const float roll_tgt = clampf(in->sp_roll + k[FC_RIG_ROLL_TRIM] + exc_att, -RIG_TGT_MAX_RAD, RIG_TGT_MAX_RAD);
	const float e = roll_tgt - RIG_CTL_ANG(in);
	if (in->int_zero) {
		ls->att_int = 0.0f;
		ls->rate_int = 0.0f;
	} else if (in->integrate) {
		ls->att_int = integrate_clamped(ls->att_int, k[FC_RIG_ATT_I] * e * dt,
						k[FC_RIG_ATT_I_LIM], in->sat_pos, in->sat_neg);
	} else if (in->lift_assist) {
		/* Before lift-off: build the trim at LIFT_I, kept at the hand-off (LIFT-OFF ASSIST). */
		ls->att_int = integrate_clamped(ls->att_int, k[FC_RIG_LIFT_I] * e * dt, att_int_lim(g, true),
						in->sat_pos || ls->assist_sat_pos,
						in->sat_neg || ls->assist_sat_neg);
	}
	const float rate_tgt = k[FC_RIG_ATT_P] * e + ls->att_int + exc_rate;

	/* Rate loop. controller_pid.cpp: rollRate_cmd = -(1/tau_rr) * (gyro - tgt) == RATE_P*(tgt - gyro).
	 * D acts on the measurement (not the error), so a setpoint or excitation step does not kick. */
	const float rate_err = rate_tgt - ls->gx_f;
	if (!in->int_zero && in->integrate) {
		ls->rate_int = integrate_clamped(ls->rate_int, k[FC_RIG_RATE_I] * rate_err * dt,
						 k[FC_RIG_RATE_I_LIM], in->sat_pos, in->sat_neg);
	}
	const float rate_cmd = k[FC_RIG_RATE_P] * rate_err + ls->rate_int - k[FC_RIG_RATE_D] * ls->dgx_f;

	/* controller_pid.cpp: u[1] = rollRate_cmd * J[0] * ATT_GAIN (pitch: u[2] = pitchRate_cmd * J[1] * ATT_GAIN). */
	float moment = rig_J[RIG_CTL] * k[FC_RIG_ATT_GAIN] * rate_cmd + exc_moment;
	/* Finite inputs can still overflow (a 1e37 gyro times RATE_P): never hand the mixer a NaN/Inf. */
	if (!(fin(moment) && fin(rate_cmd) && fin(ls->att_int) && fin(ls->rate_int) && fin(ls->dgx_f))) {
		fc_rig_law_reset(ls);
		ls->nonfinite = true;
		moment = 0.0f;
	}
	/* Lift assist: the moment stays well inside the authority at this collective, and the clamp
	 * binding counts as saturation for the next tick's integration. */
	if (in->lift_assist) {
		const float cap = RIG_LIFT_MOMENT_FRAC * fc_rig_moment_authority(in->coll_thrust);
		ls->assist_sat_pos = moment >= cap;
		ls->assist_sat_neg = moment <= -cap;
		moment = clampf(moment, -cap, cap);
	} else {
		ls->assist_sat_pos = false;
		ls->assist_sat_neg = false;
	}

	/* The locked axis (pitch on the roll rig, roll on the pitch rig) and yaw: what the flown cascade
	 * WOULD command (build constants, level target), for the log only (dbg.pitch_cmd is the locked
	 * axis'). The rig locks those axes; feeding them would just fight the cradle. */
	const float pitch_tgt = RIG_ANGLE((float)(RIG_LOCK_TRIM_RAD));
	const float pitchRate_tgt = (-1.0f / (RIG_FLOWN_SCALE * (float)(TAU_ROLL))) * (RIG_LOCK_ANG(in) - pitch_tgt);
	const float pitch_cmd = (-1.0f / (float)(TAU_ROLLRATE)) * (in->gyro[RIG_LOCK] - pitchRate_tgt);
	const float yawRate_tgt = (-1.0f / (RIG_FLOWN_SCALE * rig_tau_yaw)) * in->yaw_err;
	const float yaw_cmd = (float)(YAW_MIX_SIGN) * (-1.0f / rig_tau_yawrate) * (in->gyro[2] - yawRate_tgt);

	u[0] = in->coll_thrust;
#if RIG_AXIS == 1
	u[1] = 0.0f;
	u[2] = moment;      /* controller_pid.cpp's pitch-moment slot */
#else
	u[1] = moment;
	u[2] = 0.0f;
#endif
	u[3] = 0.0f;

	if (dbg != NULL) {
		dbg->roll = in->roll;
		dbg->pitch = in->pitch;
		dbg->roll_tgt = roll_tgt;
		dbg->rate_tgt = rate_tgt;
		dbg->rate_cmd = rate_cmd;
		dbg->moment = moment;
		dbg->exc_val = in->exc_val;
		dbg->coll_thrust = in->coll_thrust;
		dbg->att_int = ls->att_int;
		dbg->rate_int = ls->rate_int;
		dbg->pitch_cmd = pitch_cmd;
		dbg->yaw_cmd = yaw_cmd;
	}
}

void fc_rig_moment_saturation(const float duty[4], float ceiling, bool *sat_pos, bool *sat_neg)
{
	/* controller_pid.cpp's mapping: +u[1] raises ctrl[0]/ctrl[3] -> duty[3]/duty[2] (left pair)
	 * and lowers ctrl[1]/ctrl[2] -> duty[0]/duty[1] (right pair). Pitch rig: +u[2] raises
	 * ctrl[2]/ctrl[3] -> duty[1]/duty[2] (rear pair) and lowers ctrl[0]/ctrl[1] -> duty[3]/duty[0]
	 * (front pair). "left"/"right" below are those "+"/"-" pairs (RIG_SAT_*). */
	const float eps = 1e-4f;
	const bool left_hi  = duty[RIG_SAT_P0] >= ceiling - eps || duty[RIG_SAT_P1] >= ceiling - eps;
	const bool left_lo  = duty[RIG_SAT_P0] <= eps || duty[RIG_SAT_P1] <= eps;
	const bool right_hi = duty[RIG_SAT_M0] >= ceiling - eps || duty[RIG_SAT_M1] >= ceiling - eps;
	const bool right_lo = duty[RIG_SAT_M0] <= eps || duty[RIG_SAT_M1] <= eps;
	*sat_pos = left_hi || right_lo;
	*sat_neg = right_hi || left_lo;
}

/* ==== state ================================================================================== */
static void exc_clear(struct fc_rig_exc *e)
{
	memset(e, 0, sizeof(*e));
	e->type = FC_RIG_EXC_NONE;
}

void fc_rig_init(struct fc_rig_state *s)
{
	memset(s, 0, sizeof(*s));
	fc_rig_gains_default(&s->gains);
	s->last_gain = -1;
	s->coll_cmd = 0.0f;              /* nothing to ramp to until a COLL arrives */
	s->max_run_ms = clampi(RIG_MAX_RUN_MS, RIG_RUNMS_MIN, RIG_MAX_RUN_CAP_MS);
	s->trim_left_applied = s->gains.v[FC_RIG_MOTOR_TRIM_LEFT];
	exc_clear(&s->exc);
	s->rs = FC_RIG_IDLE;
	fc_rig_law_reset(&s->law);
}

const char *fc_rig_run_state_name(int rs)
{
	static const char *const names[] = {"IDLE", "ARMING", "RAMP_UP", "RUN", "RAMP_DOWN", "DONE"};
	return (rs >= 0 && rs <= FC_RIG_DONE) ? names[rs] : "?";
}

static bool motors_live(uint8_t rs)
{
	return rs == FC_RIG_RAMP_UP || rs == FC_RIG_RUN || rs == FC_RIG_RAMP_DOWN;
}

static void to_done(struct fc_rig_state *s)
{
	s->rs = FC_RIG_DONE;
	s->coll_frac = 0.0f;
	/* A finished, stopped or estopped run forgets its collective: RESET clears the estop latches
	 * (FC and ESP) and re-enables arming, so a kept COLL would re-arm and ramp straight back up
	 * ~1 s after a RESET with nobody asking. Arming again needs a fresh COLL. */
	s->coll_cmd = 0.0f;
	s->coll_ramp_to = 0.0f;
	s->arm_since = 0;
	exc_clear(&s->exc);
}

static void to_ramp_down(struct fc_rig_state *s, int64_t now, const char *why)
{
	s->rs = FC_RIG_RAMP_DOWN;
	s->phase_ms = now;
	s->coll_ramp_from = s->coll_frac;
	s->rd_reason = why;
	exc_clear(&s->exc);
}

void fc_rig_disarm(struct fc_rig_state *s)
{
	to_done(s);
}

static float slew_toward(float v, float target, float max_step)
{
	if (target > v + max_step) {
		return v + max_step;
	}
	if (target < v - max_step) {
		return v - max_step;
	}
	return target;
}

void fc_rig_step(struct fc_rig_state *s, const struct fc_rig_inputs *in, struct fc_rig_outputs *out)
{
	const int64_t now = in->now_ms;
	/* NaN -> 0 (no slew this tick); Inf or a long stall -> RIG_DT_MAX_S, so no slew jumps. */
	const float dt = in->dt > 0.0f ? (in->dt < RIG_DT_MAX_S ? in->dt : RIG_DT_MAX_S) : 0.0f;
	memset(out, 0, sizeof(*out));

	/* Soft RESET: back to IDLE (arming possible again), nothing carried over -- except a pending
	 * STOP. stop_req is consumed every tick, so one still pending here arrived since the last
	 * tick, i.e. in the same tick as the RESET, and the order of the two is not known here
	 * (rose_cmd_reset() is also reachable without a RESET line). Keeping it is the conservative
	 * reading: RESET-then-STOP is obeyed; STOP-then-RESET cancels the arming ("RIG stop: arming
	 * cancelled") and needs one more RESET. */
	if (in->reset_gen != s->reset_seen) {
		s->reset_seen = in->reset_gen;
		s->rs = FC_RIG_IDLE;
		s->arm_since = 0;
		s->coll_frac = 0.0f;
		s->coll_ramp_to = 0.0f;
		s->rd_reason = NULL;
		s->guard_active = false;
		s->tilt_since = s->pitch_since = 0;
		s->tilt_count = s->pitch_count = 0;
		exc_clear(&s->exc);
		fc_rig_law_reset(&s->law);
		out->event = "RIG reset: idle";
	}

	/* Any latched estop, in ANY state (also IDLE/ARMING), drops the commanded collective, so the
	 * RESET that clears the latch cannot re-arm on a COLL sent before the estop. */
	if (in->estop) {
		s->coll_cmd = 0.0f;
	}
	/* A latched estop from any source (guards here or in main.cpp, the ESTOP command), or arming
	 * withdrawn, ends a run at once: no ramp. */
	if (motors_live(s->rs) && (in->estop || !in->arm_enabled)) {
		to_done(s);
		out->event = in->estop ? "RIG estop: motors off" : "RIG arming disabled: motors off";
	}

	const bool stop = s->stop_req;
	s->stop_req = false;
	/* aroll: the CONTROLLED axis (pitch on the pitch rig), apitch: the LOCKED one (roll there) */
	const float aroll = fabsf(RIG_CTL_RAD(in)), apitch = fabsf(RIG_LOCK_RAD(in));
	/* NaN/Inf blinds every comparison below (fabsf(NaN) > lim is false): checked on its own. The
	 * arm test (`<` comparisons) already refuses them; the armed guards are written as
	 * !(x <= lim) so a NaN trips them too. */
	const bool sensors_finite = fin(in->roll_rad) && fin(in->pitch_rad) && fin(in->gyro[0]) &&
				    fin(in->gyro[1]) && fin(in->gyro[2]);
	/* The law zeroes the roll moment on a non-finite input it sees and fc_rig_step does not (the
	 * yaw error, the estimator's gyro, an overflow): collective with no roll authority is not a
	 * state to sit in, so that is an estop too, one tick later (main.cpp steps before the law). */
	const bool law_nonfinite = s->law.nonfinite;
	s->law.nonfinite = false;
	const bool hb_fresh = s->hb_seen && (now - s->last_hb_ms) <= (int64_t)RIG_HB_RAMP_MS;

	switch (s->rs) {
	case FC_RIG_IDLE:
	case FC_RIG_ARMING: {
		if (stop) {
			to_done(s);      /* STOP cancels a pending arm; RESET to try again */
			out->event = "RIG stop: arming cancelled";
			break;
		}
		const bool still = fabsf(in->gyro[0]) < ARM_MAX_RATE_RADPS &&
				   fabsf(in->gyro[1]) < ARM_MAX_RATE_RADPS &&
				   fabsf(in->gyro[2]) < ARM_MAX_RATE_RADPS;
		/* NO ToF / on-ground / level requirement: it may arm resting on a ~30 deg stop. */
		const bool ready = in->arm_enabled && in->gyro_cal_done && !in->estop && in->batt_ok &&
				   still && aroll < RIG_ARM_MAX_TILT_RAD && apitch < RIG_PITCH_MAX_RAD &&
				   hb_fresh && s->coll_cmd > 0.0f;
		if (fc_arm_dwell(ready, now, RIG_ARM_SETTLE_MS, &s->arm_since)) {
			s->rs = FC_RIG_RAMP_UP;
			s->arm_ms = s->phase_ms = now;
			s->arm_since = 0;
			s->coll_frac = 0.0f;
			/* The ramp's target is latched here: a COLL during RAMP_UP must not rescale the
			 * ramp (a step); it is followed from RUN on, through the RIG_COLL_SLEW slew. */
			s->coll_ramp_to = s->coll_cmd;
			s->sp_roll = RIG_CTL_RAD(in);  /* start where it rests; slew to the command */
			s->guard_active = false;
			s->rd_reason = NULL;
			s->tilt_since = s->pitch_since = 0;
			s->tilt_count = s->pitch_count = 0;
			s->lift_lim_seen = false;
			exc_clear(&s->exc);
			fc_rig_law_reset(&s->law);
			out->send_gains = true;
			out->event = "RIG armed: ramping up";
		} else {
			s->rs = s->arm_since != 0 ? FC_RIG_ARMING : FC_RIG_IDLE;
		}
		break;
	}
	case FC_RIG_RAMP_UP:
	case FC_RIG_RUN:
	case FC_RIG_RAMP_DOWN: {
		const int64_t hb_age = now - s->last_hb_ms;
		const char *estop_why = NULL;

		/* Guards (debounced like main.cpp's envelope guard). A non-finite attitude or gyro is
		 * first and immediate: every other guard is blind to it. */
		if (!sensors_finite || law_nonfinite) {
			estop_why = "rig nan";
		}
		if (hb_age > (int64_t)RIG_HB_ESTOP_MS && estop_why == NULL) {
			estop_why = "rig heartbeat";
		}
		if (fc_guard_dwell(!(apitch <= RIG_PITCH_MAX_RAD), now, SAFE_DEBOUNCE_MS,
				   SAFE_DEBOUNCE_MIN_SAMPLES, &s->pitch_since, &s->pitch_count) &&
		    estop_why == NULL) {
			estop_why = "rig " RIG_LOCK_NAME;   /* "rig pitch" on the roll rig */
		}
		/* The roll guard only goes live once roll has come inside the band since arming: it may
		 * have armed lying on a stop beyond RIG_MAX_TILT_RAD. */
		if (!s->guard_active && aroll < RIG_MAX_TILT_RAD - RIG_LIFTOFF_MARGIN_RAD) {
			s->guard_active = true;
			out->event = "RIG liftoff: " RIG_CTL_NAME " guard live";
		}
		const bool tilt_trip = s->guard_active &&
			fc_guard_dwell(!(aroll <= RIG_MAX_TILT_RAD), now, SAFE_DEBOUNCE_MS,
				       SAFE_DEBOUNCE_MIN_SAMPLES, &s->tilt_since, &s->tilt_count);
		if (tilt_trip && estop_why == NULL) {
			if (s->rs == FC_RIG_RAMP_DOWN) {
				/* Settling back onto a stop while ramping down is the expected end of a run,
				 * not a fault: finish the disarm without an estop latch. */
				to_done(s);
				out->event = "RIG ramp-down: on the stop, motors off";
				break;
			}
			estop_why = "rig tilt";
		}
		if (estop_why != NULL) {
			to_done(s);
			out->estop_req = true;
			out->estop_reason = estop_why;
			out->event = "RIG estop";
			break;
		}

		/* Ramp-down triggers, RAMP_UP and RUN only. */
		if (s->rs != FC_RIG_RAMP_DOWN) {
			const char *why = NULL;
			if (stop) {
				why = "RIG stop";
			} else if (hb_age > (int64_t)RIG_HB_RAMP_MS) {
				why = "RIG heartbeat stale";
			} else if (now - s->arm_ms > (int64_t)s->max_run_ms) {
				why = "RIG max run time";
			} else if (!s->guard_active &&
				   now - s->arm_ms > (int64_t)RIG_RAMP_MS + (int64_t)RIG_LIFTOFF_TIMEOUT_MS) {
				why = "RIG liftoff timeout";
			}
			if (why != NULL) {
				to_ramp_down(s, now, why);
				out->event = why;
			}
		}

		const int64_t t = now - s->phase_ms;
		if (s->rs == FC_RIG_RAMP_UP) {
			if (t >= (int64_t)RIG_RAMP_MS) {
				/* Handover: the ramp ends exactly at its latched target (continuous); RUN
				 * then slews to coll_cmd, whatever COLL arrived in the meantime. */
				s->rs = FC_RIG_RUN;
				s->phase_ms = now;
				s->coll_frac = s->coll_ramp_to;
				if (out->event == NULL) {
					out->event = "RIG run";
				}
			} else {
				s->coll_frac = s->coll_ramp_to * (float)t / (float)RIG_RAMP_MS;
			}
		} else if (s->rs == FC_RIG_RUN) {
			s->coll_frac = slew_toward(s->coll_frac, s->coll_cmd, RIG_COLL_SLEW * dt);
		} else {   /* RAMP_DOWN */
			if (t >= (int64_t)RIG_RAMP_MS) {
				to_done(s);
				out->event = "RIG ramp-down complete: motors off";
				break;
			}
			s->coll_frac = s->coll_ramp_from * (1.0f - (float)t / (float)RIG_RAMP_MS);
		}
		s->sp_roll = slew_toward(s->sp_roll, s->sp_roll_cmd, RIG_SP_SLEW_RADPS * dt);
		break;
	}
	case FC_RIG_DONE:
	default:
		s->coll_frac = 0.0f;
		break;
	}

	/* Excitation: RUN only, ends by itself after dur_ms, and never above its cap at the
	 * collective of this tick (COLL may have changed since it was accepted). */
	float exc_val = 0.0f;
	if (s->rs == FC_RIG_RUN && s->exc.type != FC_RIG_EXC_NONE) {
		const int64_t te = now - s->exc.start_ms;
		if (te >= (int64_t)s->exc.dur_ms) {
			exc_clear(&s->exc);
		} else {
			const float cap = exc_cap(s->exc.target, s->coll_frac * fc_rig_hover_thrust(), &s->gains);
			exc_val = clampf(fc_rig_exc_shape(&s->exc, te), -cap, cap);
			out->exc_target = s->exc.target;
		}
	} else if (s->rs != FC_RIG_RUN) {
		exc_clear(&s->exc);
	}

	/* MOTOR_TRIM_LEFT: gains.v[] holds the commanded value (echoed, logged, read back over
	 * RBPOKE); the mixer should apply trim_left_applied, slewed while the motors run so a GAIN
	 * does not step the left pair's thrust, and equal to the command while they are off. */
	const float trim_cmd = s->gains.v[FC_RIG_MOTOR_TRIM_LEFT];
	s->trim_left_applied = motors_live(s->rs)
		? slew_toward(s->trim_left_applied, trim_cmd, RIG_TRIM_SLEW * dt) : trim_cmd;

	out->rs = s->rs;
	out->armed = motors_live(s->rs);
	out->arming = s->rs == FC_RIG_ARMING;
	out->coll_frac = out->armed ? s->coll_frac : 0.0f;
	out->sp_roll = s->sp_roll;
	out->exc_val = exc_val;
	/* Integrate only once lifted off (roll guard live): RUN starts after a fixed ramp, possibly
	 * still lying on a stop, and integrating against the stop only winds up. */
	out->integrate = s->rs == FC_RIG_RUN && s->guard_active;
	out->int_zero = s->rs == FC_RIG_IDLE || s->rs == FC_RIG_ARMING || s->rs == FC_RIG_RAMP_UP;
	/* ... except the lift assist (LIFT-OFF ASSIST at the top): RUN before lift-off, which builds
	 * the angle integrator at LIFT_I under a moment clamp; its value is kept at lift-off. */
	out->lift_assist = s->rs == FC_RIG_RUN && !s->guard_active && s->gains.v[FC_RIG_LIFT_I] > 0.0f;
	/* Once per run: the assist can build no further (its integrator clamp, or the moment clamp /
	 * mixer saturation stopping the integration); no lift-off by the timeout then ramps down. */
	if (out->lift_assist && !s->lift_lim_seen && out->event == NULL) {
		const float lim = att_int_lim(&s->gains, true);
		if (fabsf(s->law.att_int) >= lim || s->law.assist_sat_pos || s->law.assist_sat_neg) {
			s->lift_lim_seen = true;
			out->event = "RIG lift assist: at limit";
		}
	}
	out->guard_active = s->guard_active;
	out->rampdown_reason = s->rd_reason;
	s->out = *out;
}

void fc_rig_law_input(const struct fc_rig_state *s, float roll, float pitch, float yaw_err,
		      const float gyro[3], float dt, struct fc_rig_law_in *in)
{
	memset(in, 0, sizeof(*in));
	in->roll = roll;
	in->pitch = pitch;
	in->yaw_err = yaw_err;
	in->gyro[0] = gyro[0];
	in->gyro[1] = gyro[1];
	in->gyro[2] = gyro[2];
	in->dt = dt;
	in->sp_roll = s->out.sp_roll;
	in->exc_target = s->out.exc_target;
	in->exc_val = s->out.exc_val;
	in->coll_thrust = s->out.coll_frac * fc_rig_hover_thrust();
	in->integrate = s->out.integrate;
	in->int_zero = s->out.int_zero;
	in->lift_assist = s->out.lift_assist;
	in->sat_pos = s->sat_pos;
	in->sat_neg = s->sat_neg;
}

/* ==== uplink commands ======================================================================== */
#define RIG_MAX_TOK 8

static int tokenize(char *buf, char *tok[RIG_MAX_TOK])
{
	int n = 0;
	char *p = buf;
	while (*p) {
		while (*p == ' ' || *p == '\t') {
			*p++ = '\0';
		}
		if (*p == '\0') {
			break;
		}
		if (n == RIG_MAX_TOK) {
			return RIG_MAX_TOK + 1;   /* too many: callers NAK */
		}
		tok[n++] = p;
		while (*p && *p != ' ' && *p != '\t') {
			p++;
		}
	}
	return n;
}

static bool reply_ack(struct sbuf *b, const char *verb, bool ok)
{
	sb_puts(b, ok ? "FCACK " : "FCNAK ");
	sb_puts(b, verb);
	sb_putc(b, '\n');
	return true;
}

static bool nak(struct fc_rig_state *s, struct sbuf *b, const char *verb, const char *why)
{
	s->nak_reason = why;
	b->len = 0;                      /* drop any half-written echo */
	if (b->n > 0) {
		b->p[0] = '\0';
	}
	return reply_ack(b, verb, false);
}

/* |amp| of a running att-target excitation (0 otherwise): it adds to the roll target. */
static float att_exc_amp(const struct fc_rig_state *s)
{
	return (s->exc.type != FC_RIG_EXC_NONE && s->exc.target == FC_RIG_TGT_ATT) ? fabsf(s->exc.amp) : 0.0f;
}

/* Worst case of the roll target (setpoint + ROLL_TRIM + att excitation) inside RIG_TGT_MAX_RAD?
 * The law clamps anyway; this refuses a command that could only be obeyed clamped. The 1e-6 lets
 * RSP 20000 (0.349066 rad) through against the default 0.35. */
static bool tgt_within(float sp, float trim, float amp)
{
	return fabsf(sp) + fabsf(trim) + fabsf(amp) <= RIG_TGT_MAX_RAD * (1.0f + 1e-6f);
}

/* Bumpless GAIN: the integrators hold CONTRIBUTIONS (att_int in rad/s of rate target, rate_int in
 * rad/s^2 of accel command; each increment is already multiplied by ATT_I / RATE_I), so a new
 * ATT_I or RATE_I acts on future increments only and the output does not step. What still scales
 * a held contribution is rebased so the integral part of the moment,
 *     J0 * ATT_GAIN * (RATE_P * att_int + rate_int),
 * is unchanged: a new ATT_GAIN rescales both by old/new, a new RATE_P rescales att_int by old/new.
 * A new *_I_LIM (LIFT_I_LIM during the lift assist) clamps to the new limit (the one case that can
 * step, and only if the limit binds).
 * ATT_I, LIFT_I or RATE_I set to 0 freezes that contribution where it is (no step; LIFT_I = 0
 * also ends the assist's moment clamp); the next arm clears it. Everything else (ATT_P, RATE_D, the LPFs, ROLL_TRIM, MOTOR_TRIM_LEFT) leaves them alone. */
static void rebase_integrators(struct fc_rig_law_state *ls, const struct fc_rig_gains *old_g,
			       const struct fc_rig_gains *new_g, int id, bool lift_assist)
{
	const float o = old_g->v[id], n = new_g->v[id];
	switch (id) {
	case FC_RIG_ATT_GAIN:     /* both ranges' minimums are > 0: no division by zero */
		ls->att_int *= o / n;
		ls->rate_int *= o / n;
		break;
	case FC_RIG_RATE_P:
		ls->att_int *= o / n;
		break;
	default:
		break;
	}
	/* the active phase's angle clamp: min(LIFT_I_LIM, ATT_I_LIM) during the lift assist */
	const float al = att_int_lim(new_g, lift_assist), rl = new_g->v[FC_RIG_RATE_I_LIM];
	ls->att_int = clampf(ls->att_int, -al, al);
	ls->rate_int = clampf(ls->rate_int, -rl, rl);
}

static const char *const exc_type_names[] = {"none", "step", "doublet", "chirp", "sine", "prbs"};
static const char *const exc_target_names[] = {"att", "rate", "moment"};

static int find_name(const char *s, const char *const *names, int count)
{
	for (int i = 0; i < count; i++) {
		if (strcmp(s, names[i]) == 0) {
			return i;
		}
	}
	return -1;
}

static bool cmd_exc(struct fc_rig_state *s, char **tok, int ntok, int64_t now, struct sbuf *b)
{
	static const char verb[] = "EXC";
	if (ntok < 5 || ntok > 7) {
		return nak(s, b, verb, "EXC: <type> <target> <amp> <dur_ms> [<f0_mHz> [<f1_mHz>]]");
	}
	const int type = find_name(tok[1], exc_type_names, 6);
	const int target = find_name(tok[2], exc_target_names, 3);
	if (type <= FC_RIG_EXC_NONE) {
		return nak(s, b, verb, "EXC: unknown type");
	}
	if (target < 0) {
		return nak(s, b, verb, "EXC: unknown target");
	}
	int32_t amp, dur, f0 = 0, f1 = 0;
	if (!fc_rig_parse_int(tok[3], &amp) || !fc_rig_parse_int(tok[4], &dur) ||
	    (ntok > 5 && !fc_rig_parse_int(tok[5], &f0)) ||
	    (ntok > 6 && !fc_rig_parse_int(tok[6], &f1))) {
		return nak(s, b, verb, "EXC: arguments must be integers");
	}
	const bool need_f0 = type == FC_RIG_EXC_CHIRP || type == FC_RIG_EXC_SINE || type == FC_RIG_EXC_PRBS;
	if ((need_f0 && ntok < 6) || (type == FC_RIG_EXC_CHIRP && ntok < 7)) {
		return nak(s, b, verb, "EXC: missing frequency");
	}
	if ((ntok > 5 && (f0 < RIG_EXC_F_MIN_MHZ || f0 > RIG_EXC_F_MAX_MHZ)) ||
	    (ntok > 6 && (f1 < RIG_EXC_F_MIN_MHZ || f1 > RIG_EXC_F_MAX_MHZ))) {
		return nak(s, b, verb, "EXC: frequency out of range");
	}
	if (dur < 1 || dur > RIG_MAX_RUN_CAP_MS) {
		return nak(s, b, verb, "EXC: duration out of range");
	}
	if (amp == 0) {
		return nak(s, b, verb, "EXC: zero amplitude");
	}
	/* mrad, mrad/s, uN*m -> SI */
	const float amp_si = (float)amp * (target == FC_RIG_TGT_MOMENT ? 1e-6f : 1e-3f);
	if (s->rs != FC_RIG_RUN) {
		return nak(s, b, verb, "EXC: only in RUN");
	}
	if (!s->guard_active) {
		return nak(s, b, verb, "EXC: not lifted off yet");
	}
	/* Over the cap is refused, not clamped: a silently smaller excitation is a bad data set. */
	if (fabsf(amp_si) > exc_cap((uint8_t)target, s->coll_frac * fc_rig_hover_thrust(), &s->gains) *
			    (1.0f + 1e-6f)) {
		return nak(s, b, verb, "EXC: amplitude over cap");
	}
	if (target == FC_RIG_TGT_ATT) {
		const float sp = fabsf(s->sp_roll_cmd) > fabsf(s->sp_roll) ? s->sp_roll_cmd : s->sp_roll;
		if (!tgt_within(sp, s->gains.v[FC_RIG_ROLL_TRIM], amp_si)) {
			return nak(s, b, verb, "EXC: setpoint + trim + amp over RIG_TGT_MAX_RAD");
		}
	}
	exc_clear(&s->exc);
	s->exc.type = (uint8_t)type;
	s->exc.target = (uint8_t)target;
	s->exc.amp = amp_si;
	s->exc.dur_ms = dur;
	s->exc.f0_hz = (float)f0 * 1e-3f;
	s->exc.f1_hz = (float)(type == FC_RIG_EXC_CHIRP ? f1 : f0) * 1e-3f;
	s->exc.start_ms = now;
	s->exc.prbs_reg = RIG_PRBS_SEED;
	s->exc.prbs_bit = 0;
	s->exc_id++;
	sb_puts(b, "RIGEXC id=");
	sb_putu(b, s->exc_id);
	for (int i = 1; i < ntok; i++) {
		sb_putc(b, ' ');
		sb_puts(b, tok[i]);
	}
	sb_putc(b, '\n');
	return reply_ack(b, verb, true);
}

bool fc_rig_handle_command(struct fc_rig_state *s, const char *line, int64_t now_ms,
			   char *reply, size_t n)
{
	struct sbuf b;
	sb_init(&b, reply, n);
	if (line == NULL) {
		return false;
	}
	/* Work on a copy: the caller's line is const. main.cpp's reader caps a line at 79 bytes. */
	char buf[96];
	const size_t len = strlen(line);
	const bool too_long = len >= sizeof(buf);
	memcpy(buf, line, too_long ? sizeof(buf) - 1 : len);
	buf[too_long ? sizeof(buf) - 1 : len] = '\0';

	char *tok[RIG_MAX_TOK];
	const int ntok = tokenize(buf, tok);
	if (ntok == 0) {
		return false;
	}
	static const char *const verbs[] = {"HB", "COLL", "RSP", "GAIN", "GAINS", "EXC",
					    "EXCSTOP", "RUNMS", "STOP"};
	const char *verb = NULL;
	for (size_t i = 0; i < sizeof(verbs) / sizeof(verbs[0]); i++) {
		if (strcmp(tok[0], verbs[i]) == 0) {   /* whole token: GAINS is not GAIN */
			verb = verbs[i];
		}
	}
	if (verb == NULL) {
		return false;                         /* ESTOP, RESET, PING, ... -> main.cpp */
	}
	s->nak_reason = NULL;
	if (too_long || ntok > RIG_MAX_TOK) {
		return nak(s, &b, verb, "line too long");
	}

	if (strcmp(verb, "HB") == 0) {
		if (ntok != 1) {
			return nak(s, &b, verb, "HB takes no arguments");
		}
		s->last_hb_ms = now_ms;
		s->hb_seen = true;
		return true;                          /* no reply: 20 Hz of ACKs would crowd the link */
	}
	if (strcmp(verb, "COLL") == 0 || strcmp(verb, "RSP") == 0 || strcmp(verb, "RUNMS") == 0) {
		int32_t v;
		if (ntok != 2 || !fc_rig_parse_int(tok[1], &v)) {
			return nak(s, &b, verb, "expects one integer");
		}
		/* COLL and RSP out of range are refused, not clamped: "COLL 999999999" is a typo, not a
		 * request for the maximum. (RUNMS is a time cap: clamping it is always the safe side.) */
		if (strcmp(verb, "COLL") == 0) {
			/* per-mille of MASS*g */
			if (v < 0 || v > RIG_COLL_MAX_PERMILLE) {
				return nak(s, &b, verb, "COLL: out of range [0, RIG_COLL_MAX_PERMILLE]");
			}
			s->coll_cmd = (float)v * 1e-3f;
		} else if (strcmp(verb, "RSP") == 0) {
			/* millidegrees */
			if (v < -RIG_SP_MAX_MDEG || v > RIG_SP_MAX_MDEG) {
				return nak(s, &b, verb, "RSP: out of range +-RIG_SP_MAX_MDEG");
			}
			const float sp = (float)v * (3.14159265f / 180000.0f);
			if (!tgt_within(sp, s->gains.v[FC_RIG_ROLL_TRIM], att_exc_amp(s))) {
				return nak(s, &b, verb, "RSP: setpoint + trim + att exc over RIG_TGT_MAX_RAD");
			}
			s->sp_roll_cmd = sp;
		} else {   /* RUNMS */
			s->max_run_ms = clampi(v, RIG_RUNMS_MIN, RIG_MAX_RUN_CAP_MS);
		}
		return reply_ack(&b, verb, true);
	}
	if (strcmp(verb, "GAIN") == 0) {
		float v;
		if (ntok != 3) {
			return nak(s, &b, verb, "GAIN <NAME> <value>");
		}
		if (!fc_rig_parse_decimal(tok[2], &v)) {
			return nak(s, &b, verb, "GAIN: bad number");
		}
		/* On a copy first: whitelist and range, then the cross-checks, then commit. */
		struct fc_rig_gains ng = s->gains;
		const int id = fc_rig_gain_set(&ng, tok[1], v);
		if (id == -1) {
			return nak(s, &b, verb, "GAIN: unknown name");
		}
		if (id < 0) {
			return nak(s, &b, verb, "GAIN: out of range");
		}
		if (id == FC_RIG_ROLL_TRIM && !tgt_within(s->sp_roll_cmd, ng.v[id], att_exc_amp(s))) {
			return nak(s, &b, verb, "GAIN: ROLL_TRIM + setpoint + att exc over RIG_TGT_MAX_RAD");
		}
		rebase_integrators(&s->law, &s->gains, &ng, id, s->out.lift_assist);
		s->gains = ng;
		s->gain_seq++;
		s->last_gain = (int8_t)id;
		sb_puts(&b, "RIGGAIN ");
		sb_puts(&b, fc_rig_gain_name(id));
		sb_putc(&b, '=');
		sb_putdec(&b, s->gains.v[id]);
		sb_putc(&b, '\n');
		return reply_ack(&b, verb, true);
	}
	if (strcmp(verb, "GAINS") == 0) {
		if (ntok != 1) {
			return nak(s, &b, verb, "GAINS takes no arguments");
		}
		if (fc_rig_gains_format(&s->gains, s->gain_seq, reply, n) < 0) {
			return nak(s, &b, verb, "reply buffer too small");
		}
		b.len = strlen(reply);
		return reply_ack(&b, verb, true);
	}
	if (strcmp(verb, "EXC") == 0) {
		return cmd_exc(s, tok, ntok, now_ms, &b);
	}
	if (strcmp(verb, "EXCSTOP") == 0) {
		if (ntok != 1) {
			return nak(s, &b, verb, "EXCSTOP takes no arguments");
		}
		exc_clear(&s->exc);
		return reply_ack(&b, verb, true);
	}
	/* STOP: acted on at the next fc_rig_step, the only place that changes run state. */
	if (ntok != 1) {
		return nak(s, &b, verb, "STOP takes no arguments");
	}
	s->stop_req = true;
	return reply_ack(&b, verb, true);
}
