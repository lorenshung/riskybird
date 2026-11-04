/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Roll-harness ("rig") mode: the drone sits on a 1-DOF roll pivot (hard stops at about +-30 deg)
 * and is held LEVEL at hover-level collective while the roll loops are tuned over WiFi. This header
 * is the platform-free half: plain structs and pure functions, no Zephyr, so the whole thing also
 * compiles and is unit-tested on the host (tests/test_fc_rig.py). main.cpp owns the one instance
 * (g_rig), feeds it uplink lines and one fc_rig_step() per control tick; controller_pid.cpp calls
 * fc_rig_roll_law() in place of its cascade. fc_rig.cpp is compiled only when ROSE_RIG=1.
 *
 * Nothing build-configurable lives here. The defaults that must follow the build (ATT_GAIN,
 * TAU_ROLL, TAU_ROLLRATE, MOTOR_TRIM_LEFT, PID_MASS_KG, ROLL_TRIM_RAD, SAFE_DEBOUNCE_MS) are read
 * in fc_rig.cpp with the same #ifndef fallbacks controller_pid.cpp and main.cpp use, so including
 * this header can never redefine one of their macros.
 *
 * Units on this side are honest: radians from fc_pid_attitude(), rad/s, N, N*m. See fc_rig.cpp for
 * how the default gains reproduce the flown PID and how to convert a tuned rig back to a preset.
 */
#ifndef ROSE_FC_RIG_H
#define ROSE_FC_RIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- gains -----------------------------------------------------------------------------------
 * Index-based so the flight recorder can log a change as (gain_seq, index, value) and the host
 * can read the whole array back over RBPOKE. The order is part of that log format: append only. */
enum fc_rig_gain_id {
	FC_RIG_ATT_P = 0,        /* [1/s]     rate target per rad of angle error           */
	FC_RIG_ATT_I,            /* [1/s^2]   integral of angle error into the rate target  */
	FC_RIG_RATE_P,           /* [1/s]     angular-accel command per rad/s of rate error */
	FC_RIG_RATE_I,           /* [1/s^2]   integral of rate error into the accel command */
	FC_RIG_RATE_D,           /* [-]       on the filtered measured angular acceleration */
	FC_RIG_ATT_GAIN,         /* [-]       moment scale (the mixer's J*ATT_GAIN)          */
	FC_RIG_D_LPF_HZ,         /* [Hz]      low-pass on the D term's derivative            */
	FC_RIG_GYRO_LPF_HZ,      /* [Hz]      first-order LPF on the rate-loop gyro, 0 = off */
	FC_RIG_ROLL_TRIM,        /* [rad]     added to the roll angle target                 */
	FC_RIG_MOTOR_TRIM_LEFT,  /* [-]       left-pair thrust factor (commanded; the mixer applies
				  *           fc_rig_state.trim_left_applied, slewed toward it)  */
	FC_RIG_ATT_I_LIM,        /* [rad/s]   clamp on the ATT_I contribution                */
	FC_RIG_RATE_I_LIM,       /* [rad/s^2] clamp on the RATE_I contribution               */
	/* Lift-off assist (RUN before lift-off; see fc_rig.cpp "LIFT-OFF ASSIST"). Appended: the
	 * recorder's v3 gain snapshot holds the first FLIGHTREC_RIG_GAINS (12) only; changes to these
	 * are still logged in its change ring. */
	FC_RIG_LIFT_I,           /* [1/s^2]   angle-integrator rate before lift-off, 0 = assist off */
	FC_RIG_LIFT_I_LIM,       /* [rad/s]   clamp on it before lift-off (min'd with ATT_I_LIM)  */
	FC_RIG_GAIN_COUNT
};

struct fc_rig_gains {
	float v[FC_RIG_GAIN_COUNT];
};

/* Wire name ("ATT_P") of a gain, NULL when out of range. */
const char *fc_rig_gain_name(int id);
/* Index of a wire name, -1 if not on the whitelist. Exact, case-sensitive match. */
int fc_rig_gain_find(const char *name);
/* The accepted [min, max] of a gain. */
void fc_rig_gain_range(int id, float *min, float *max);
/* Fill with the build's defaults (fc_rig.cpp). */
void fc_rig_gains_default(struct fc_rig_gains *g);
/* Set one gain by wire name. Returns the gain's index, or -1 (unknown name) / -2 (out of range or
 * not finite); the table is unchanged on failure. Does NOT bump gain_seq, cross-check ROLL_TRIM
 * against the setpoint or rebase the integrators -- the GAIN command does that (bumpless: see
 * rebase_integrators in fc_rig.cpp); this is the bare whitelist. */
int fc_rig_gain_set(struct fc_rig_gains *g, const char *name, float value);
/* "RIGGAINS gs=<n> ATT_P=5 ATT_I=0 ... RATE_I_LIM=30\n" into out. Returns the length written, or
 * -1 if it did not fit (out is then an empty string). FC_RIG_REPLY_MAX always fits. */
int fc_rig_gains_format(const struct fc_rig_gains *g, uint32_t gain_seq, char *out, size_t n);

/* ---- number parsing and printing (no scanf, no %f) -------------------------------------------
 * Decimal: optional sign, at least one digit, optional '.' followed by at least one digit. No
 * exponent, no inf/nan, no surrounding blanks, no leading '.', no trailing '.'; at most 9 integer
 * and 12 fraction digits, 15 in all. The digits are accumulated as one integer and scaled once. */
bool fc_rig_parse_decimal(const char *s, float *out);
/* Integer: optional sign, 1..9 digits, nothing else. */
bool fc_rig_parse_int(const char *s, int32_t *out);
/* The shortest decimal that fc_rig_parse_decimal reads back as exactly v ("40", "0.85",
 * "-0.0125"); no exponent, so it is exact for |v| >= 1e-3 and rounds below that. -1 if it does not fit. */
int fc_rig_format_decimal(float v, char *out, size_t n);

/* ---- run state machine --------------------------------------------------------------------- */
enum fc_rig_run_state {
	FC_RIG_IDLE = 0,    /* disarmed, arm conditions not (yet) held                         */
	FC_RIG_ARMING,      /* disarmed, conditions held, counting RIG_ARM_SETTLE_MS          */
	FC_RIG_RAMP_UP,     /* armed: collective 0 -> coll_cmd over RIG_RAMP_MS               */
	FC_RIG_RUN,         /* armed: collective follows coll_cmd, excitations allowed        */
	FC_RIG_RAMP_DOWN,   /* armed: collective -> 0 over RIG_RAMP_MS, then DONE             */
	FC_RIG_DONE         /* disarmed; RESET (reset_gen change) to go back to IDLE           */
};
const char *fc_rig_run_state_name(int rs);

/* ---- excitation ---------------------------------------------------------------------------- */
enum fc_rig_exc_type { FC_RIG_EXC_NONE = 0, FC_RIG_EXC_STEP, FC_RIG_EXC_DOUBLET, FC_RIG_EXC_CHIRP,
		       FC_RIG_EXC_SINE, FC_RIG_EXC_PRBS };
/* Where the value is added: the roll angle target (rad), the rate target (rad/s) or the roll
 * moment (N*m). */
enum fc_rig_exc_target { FC_RIG_TGT_ATT = 0, FC_RIG_TGT_RATE, FC_RIG_TGT_MOMENT };

struct fc_rig_exc {
	uint8_t  type;        /* fc_rig_exc_type; NONE = inactive                   */
	uint8_t  target;      /* fc_rig_exc_target                                   */
	float    amp;         /* SI units of the target, signed                      */
	int32_t  dur_ms;
	float    f0_hz, f1_hz;
	int64_t  start_ms;
	/* PRBS generator: the register after `prbs_bit` advances. */
	uint16_t prbs_reg;
	int32_t  prbs_bit;
};

/* Value of an excitation `t_ms` after its start, before any clamping. 0 for t < 0 and t >= dur.
 * Pure for every shape but PRBS, which advances its LFSR in `e` (deterministically: the same
 * excitation always yields the same bit sequence, and t may only move forward between calls). */
float fc_rig_exc_shape(struct fc_rig_exc *e, int64_t t_ms);

/* ---- roll control law ---------------------------------------------------------------------- */
struct fc_rig_law_state {
	float att_int;     /* angle-integrator contribution to the rate target (rad/s), clamped:
			    * built by LIFT_I before lift-off, kept and continued by ATT_I after */
	float rate_int;    /* RATE_I contribution to the accel command (rad/s^2), clamped   */
	float gx_f;        /* gyro x after GYRO_LPF_HZ                                       */
	float gx_prev;     /* previous gx_f, for the derivative                              */
	float dgx_f;       /* d(gx_f)/dt after D_LPF_HZ                                      */
	bool  seeded;      /* false until the first tick after a reset                       */
	bool  nonfinite;   /* set when the law zeroed a non-finite input/output; fc_rig_step
			    * estops on it ("rig nan") while armed and clears it            */
	bool  assist_sat_pos, assist_sat_neg;  /* the lift-assist moment clamp bound last tick in
				    * that direction: the assist's anti-windup (the mixer is NOT
				    * saturated when only the clamp binds)                          */
};

/* What the law publishes for telemetry and the recorder. */
struct fc_rig_dbg {
	float roll, pitch;          /* rad, as used                                              */
	float roll_tgt;             /* rad: setpoint + trim + exc(att)                           */
	float rate_tgt;             /* rad/s                                                     */
	float rate_cmd;             /* rad/s^2                                                   */
	float moment;               /* N*m, what goes to the mixer                               */
	float exc_val;              /* the injected value, in its target's unit                  */
	float coll_thrust;          /* N                                                         */
	float att_int, rate_int;    /* integrator contributions                                  */
	float pitch_cmd, yaw_cmd;   /* rad/s^2 the flown PID would command; NOT sent to motors   */
};

struct fc_rig_law_in {
	float roll, pitch;      /* rad (fc_pid_attitude: true angles)                              */
	float yaw_err;          /* rad, yaw - yaw target (wrapped); only for the would-be yaw_cmd  */
	float gyro[3];          /* rad/s                                                           */
	float dt;               /* s                                                               */
	float sp_roll;          /* rad: the slewed setpoint (fc_rig_outputs.sp_roll)               */
	uint8_t exc_target;     /* fc_rig_exc_target                                               */
	float exc_val;          /* 0 when no excitation                                            */
	float coll_thrust;      /* N: coll_frac * MASS * g                                         */
	bool  integrate;        /* integrators run (RUN after liftoff, i.e. roll guard live)      */
	bool  int_zero;         /* integrators held at 0 (IDLE/ARMING/RAMP_UP/DONE)                */
	bool  lift_assist;      /* RUN before lift-off with LIFT_I > 0: the angle integrator builds at
				 * LIFT_I (clamp min(LIFT_I_LIM, ATT_I_LIM)) and the moment is clamped
				 * to RIG_LIFT_MOMENT_FRAC of the authority; never with integrate      */
	/* Anti-windup. sat_pos: the mixer cannot give MORE positive roll moment (a left motor,
	 * duty[2]/duty[3], is at the ceiling or a right motor, duty[0]/duty[1], is at 0); sat_neg the
	 * mirror. Usually last tick's duties via fc_rig_moment_saturation(). */
	bool  sat_pos, sat_neg;
};

/* u[0] = collective thrust (N), u[1] = roll moment (N*m), u[2] = u[3] = 0: the [thrust, moments]
 * vector controller_pid.cpp feeds its mixer matrix M directly (already scaled; do NOT multiply by
 * J*ATT_GAIN again). Sign conventions are controller_pid.cpp's. */
void fc_rig_roll_law(struct fc_rig_law_state *ls, const struct fc_rig_gains *g,
		     const struct fc_rig_law_in *in, float u[4], struct fc_rig_dbg *dbg);
void fc_rig_law_reset(struct fc_rig_law_state *ls);
/* Directional saturation from the duties controller_pid.cpp produced (index = its duty[] order). */
void fc_rig_moment_saturation(const float duty[4], float ceiling, bool *sat_pos, bool *sat_neg);
/* The spec's roll-moment authority at a collective thrust (N): 2 * 0.25 * u0 * l_arm. */
float fc_rig_moment_authority(float coll_thrust);
/* MASS * g of this build (N): the thrust of a collective of 1.0. */
float fc_rig_hover_thrust(void);

/* One control tick. */
struct fc_rig_inputs {
	int64_t  now_ms;
	float    dt;
	bool     arm_enabled;       /* g_arm_enabled (RESET command)                            */
	bool     gyro_cal_done;
	float    gyro[3];           /* rad/s                                                    */
	float    roll_rad, pitch_rad;
	bool     batt_ok;           /* batt_ok_to_arm()                                         */
	bool     estop;             /* g_estop (any source)                                     */
	uint32_t reset_gen;         /* g_reset_gen                                              */
};

struct fc_rig_outputs {
	uint8_t  rs;                /* fc_rig_run_state after this tick                         */
	bool     armed;             /* motors may run: RAMP_UP, RUN, RAMP_DOWN                  */
	bool     arming;            /* ARMING (status bit 4)                                    */
	float    coll_frac;         /* collective, fraction of MASS*g                           */
	float    sp_roll;           /* slewed roll setpoint (rad)                               */
	uint8_t  exc_target;
	float    exc_val;           /* already clamped to its cap; 0 outside RUN                */
	bool     integrate, int_zero;   /* for fc_rig_law_in; integrate = RUN && guard_active    */
	bool     lift_assist;       /* for fc_rig_law_in: RUN && !guard_active && LIFT_I > 0     */
	bool     guard_active;
	/* Edge-triggered, valid for THIS tick only: */
	bool     estop_req;         /* latch g_estop now and command zero                       */
	const char *estop_reason;
	const char *event;          /* human-readable transition note ("RIG armed", ...)        */
	bool     send_gains;        /* emit the GAINS line (once, on the arm tick)              */
	/* Persistent: */
	const char *rampdown_reason;
};

/* ---- shared state -------------------------------------------------------------------------- */
struct fc_rig_state {
	/* commanded over the uplink */
	struct fc_rig_gains gains;
	uint32_t gain_seq;          /* bumped per accepted GAIN                                 */
	int8_t   last_gain;         /* index of the last accepted GAIN, -1 none                 */
	float    coll_cmd;          /* fraction of MASS*g, [0, RIG_COLL_MAX_PERMILLE/1000]; zeroed
				     * by any estop and at DONE, so a RESET needs a fresh COLL   */
	float    sp_roll_cmd;       /* rad                                                     */
	int32_t  max_run_ms;        /* motor-live time cap, from the arm tick                   */
	int64_t  last_hb_ms;        /* time of the last HB                                      */
	bool     hb_seen;           /* any HB yet                                               */
	bool     stop_req;          /* STOP received; consumed by fc_rig_step                   */
	struct fc_rig_exc exc;
	uint32_t exc_id;            /* bumped per accepted EXC                                  */

	/* run state */
	uint8_t  rs;                /* fc_rig_run_state                                         */
	uint32_t reset_seen;
	int64_t  arm_since;         /* fc_arm_dwell sentinel                                    */
	int64_t  arm_ms;            /* tick that armed (RAMP_UP entry)                          */
	int64_t  phase_ms;          /* entry tick of the current armed state                    */
	float    coll_frac;         /* current collective (fraction)                            */
	float    coll_ramp_from;    /* collective at RAMP_DOWN entry                            */
	float    sp_roll;           /* slewed setpoint (rad)                                    */
	bool     guard_active;      /* roll has been inside the liftoff band since arming       */
	const char *rd_reason;      /* why the last RAMP_DOWN began (NULL none)                 */
	int64_t  tilt_since, pitch_since;    /* fc_guard_dwell state                            */
	int      tilt_count, pitch_count;

	/* control law */
	struct fc_rig_law_state law;
	struct fc_rig_dbg dbg;
	bool     sat_pos, sat_neg;  /* written by the mixer hook (fc_rig_moment_saturation on the
				     * final duties), read by the next tick's fc_rig_law_input  */
	struct fc_rig_outputs out;  /* copy of the last fc_rig_step result, for controller_pid.cpp */

	const char *nak_reason;     /* why the last command was NAKed (for a console printk)    */

	/* Appended (keep gains + gain_seq first: the host reads them at &g_rig over RBPOKE). */
	float    coll_ramp_to;      /* RAMP_UP's target, coll_cmd latched at the arm tick       */
	float    trim_left_applied; /* MOTOR_TRIM_LEFT the mixer should use: gains.v[] slewed at
				     * RIG_TRIM_SLEW while motors run, equal to it otherwise    */
	bool     lift_lim_seen;     /* the "lift assist at its limit" event went out this run   */
};

/* The one instance, defined in main.cpp (rig builds only); controller_pid.cpp reads it. */
extern struct fc_rig_state g_rig;

void fc_rig_init(struct fc_rig_state *s);

void fc_rig_step(struct fc_rig_state *s, const struct fc_rig_inputs *in, struct fc_rig_outputs *out);

/* Assemble the law's inputs from the last fc_rig_step (s->out), the saturation flags and the
 * live attitude, so the controller_pid.cpp hook is: fc_rig_law_input(); fc_rig_roll_law(). */
void fc_rig_law_input(const struct fc_rig_state *s, float roll, float pitch, float yaw_err,
		      const float gyro[3], float dt, struct fc_rig_law_in *in);

/* DISARM command: motors off now, no estop; RESET to re-arm. */
void fc_rig_disarm(struct fc_rig_state *s);

/* ---- uplink commands ------------------------------------------------------------------------
 * Handles HB, COLL, RSP, GAIN, GAINS, EXC, EXCSTOP, RUNMS, STOP (whole-token verb match). Returns
 * false for anything else -- ESTOP, RESET, DISARM, PING, unknown -- with reply = "", so main.cpp
 * falls through to its existing verbs. When true, reply holds zero or more '\n'-terminated lines
 * to write to the ESP verbatim: HB -> "" (no reply, keeps the link quiet); GAIN -> "RIGGAIN
 * NAME=value\n" then "FCACK GAIN\n"; GAINS -> the RIGGAINS line then "FCACK GAINS\n"; EXC ->
 * "RIGEXC id=<n> <args>\n" then "FCACK EXC\n"; every other ACK is the one line "FCACK <VERB>\n";
 * every NAK is the one line "FCNAK <VERB>\n", with the reason in s->nak_reason (not on the wire,
 * so host parsers of "FCNAK <verb>" are unaffected). COLL and RSP out of range are NAKed, not
 * clamped; RSP, EXC att and GAIN ROLL_TRIM are NAKed when |setpoint| + |ROLL_TRIM| + |att exc|
 * could exceed RIG_TGT_MAX_RAD; EXC needs RUN after liftoff. Size the reply buffer
 * FC_RIG_REPLY_MAX: the GAINS reply is ~300 B (the ESP bridge forwards lines up to 512 B). */
#define FC_RIG_REPLY_MAX 384
bool fc_rig_handle_command(struct fc_rig_state *s, const char *line, int64_t now_ms,
			   char *reply, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* ROSE_FC_RIG_H */
