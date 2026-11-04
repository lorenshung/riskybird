/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host twin of the roll harness: a TRANSCRIPTION of the rig path of
 * workloads/flight_controller/src/controller_pid.cpp after fc_rig_roll_law() -- the mixer matrix M,
 * the ctrl[] -> duty[] motor permutation, the 0.9 factor, MOTOR_TRIM_LEFT and forceToVoltage() --
 * because those are `static` there and cannot be linked. Everything that is already platform-free
 * (fc_rig.cpp, fc_shared_math.h's fc_actuator_duty) is used as is, not copied.
 *
 * Kept in float with controller_pid.cpp's operation order so the result is bit-identical;
 * tools/rig/sim/mixer_check.cpp (run by tests/test_rig_sim.py) compiles the REAL controller_pid.cpp
 * with -DROSE_RIG=1 and asserts that. If that test fails, controller_pid.cpp changed: update this.
 */
#ifndef RIG_TWIN_MIXER_H
#define RIG_TWIN_MIXER_H

#include <math.h>

namespace rigtwin {

/* controller_pid.cpp constants (static there). */
static const float L_ARM            = 33e-3f;
static const float K_DRAG           = 0.01f;
static const float THRUST_A         = 26.919633f;   /* g per duty^2 (45-35 TOTAL fit) */
static const float THRUST_B         = 35.754861f;   /* g per duty   (45-35 TOTAL fit) */
static const float PROP_GAIN        = 1.33f;
static const float GRAMS_PER_NEWTON = 101.9368f;
static const float MOTORS           = 4.0f;
static const float MIX_FACTOR       = 0.9f;         /* the "0.9f * ctrl[i]" in compute() */

/* controller_pid.cpp forceToVoltage(), verbatim. */
static inline float force_to_duty(float forceNewtons)
{
	if (forceNewtons <= 0.0f) {
		return 0.0f;
	}
	float t = (forceNewtons * GRAMS_PER_NEWTON * MOTORS) / PROP_GAIN;
	float disc = THRUST_B * THRUST_B + 4.0f * THRUST_A * t;
	if (disc < 0.0f) {
		disc = 0.0f;
	}
	float duty = (-THRUST_B + sqrtf(disc)) / (2.0f * THRUST_A);
	if (duty < 0.0f) {
		duty = 0.0f;
	} else if (duty > 1.0f) {
		duty = 1.0f;
	}
	return duty;
}

/* controller_pid.cpp compute() from `const float M[4][4]` to u_out (the rig branch: u from
 * fc_rig_roll_law, trim_left = g_rig.trim_left_applied). u_out is duty - 0.583. */
static inline void rig_mix(const float u[4], float trim_left, float u_out[4])
{
	const float l_arm = L_ARM, k_drag = K_DRAG;
	const float M[4][4] = {
		{0.25f,  0.25f / l_arm, -0.25f / l_arm,  0.25f / k_drag},
		{0.25f, -0.25f / l_arm, -0.25f / l_arm, -0.25f / k_drag},
		{0.25f, -0.25f / l_arm,  0.25f / l_arm,  0.25f / k_drag},
		{0.25f,  0.25f / l_arm,  0.25f / l_arm, -0.25f / k_drag},
	};
	float ctrl[4] = {0, 0, 0, 0};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			ctrl[i] += M[i][j] * u[j];
		}
	}
	float duty[4];
	duty[0] = force_to_duty(0.9f * ctrl[1]);
	duty[1] = force_to_duty(0.9f * ctrl[2]);
	duty[2] = force_to_duty(0.9f * ctrl[3] * (float)(trim_left));
	duty[3] = force_to_duty(0.9f * ctrl[0] * (float)(trim_left));
	for (int i = 0; i < 4; i++) {
		u_out[i] = duty[i] - 0.583f;
	}
}

/* The Bitcraze 45-35 curve x PROP_GAIN as PER-MOTOR thrust (N) at duty d: the forward map
 * force_to_duty() inverts (up to its clamps). The plant uses it with an effective duty and a
 * "true thrust scale"; double, since this is the physics side. */
static inline double curve_thrust_n(double d)
{
	if (d <= 0.0) {
		return 0.0;
	}
	return ((double)THRUST_A * d * d + (double)THRUST_B * d) * (double)PROP_GAIN /
	       ((double)GRAMS_PER_NEWTON * (double)MOTORS);
}

/* Which physical side each duty[] index drives (controller_pid.cpp's permutation):
 * duty[0] <- ctrl[1], duty[1] <- ctrl[2] are the RIGHT pair (lowered by +u[1]);
 * duty[2] <- ctrl[3], duty[3] <- ctrl[0] are the LEFT pair (raised by +u[1]). */
static const int SIDE_LEFT[4] = {0, 0, 1, 1};

/* The pitch rig (axis 1, the drone turned 90 deg on the pivot): M's column 2 (u[2], pitch moment)
 * is {-, -, +, +} on ctrl[0..3], so through the same permutation +u[2] raises duty[1] <- ctrl[2]
 * and duty[2] <- ctrl[3] (the REAR pair: +pitch is nose down about +y, FLU) and lowers duty[0] <-
 * ctrl[1] and duty[3] <- ctrl[0] (the FRONT pair). MOTOR_TRIM_LEFT still scales duty[2]/duty[3],
 * one motor of each pitch pair, so on the pitch rig it is a roll trim on the locked axis. */
static const int SIDE_REAR[4] = {0, 1, 1, 0};

/* controller_pid.cpp `J[3]` (static there): the roll / pitch inertias the moments are scaled by.
 * The twin's pitch path reuses the roll law's moment (rig_J[0] * ATT_GAIN * rate_cmd), which is
 * the flight PID's pitch moment only while J[1] == J[0]; tests/test_rig_sim.py checks both values
 * (and tau_pitch == TAU_ROLL, tau_pitchRate == TAU_ROLLRATE) against controller_pid.cpp. */
static const float J_ROLL  = 16e-6f;
static const float J_PITCH = 16e-6f;

} /* namespace rigtwin */

#endif /* RIG_TWIN_MIXER_H */
