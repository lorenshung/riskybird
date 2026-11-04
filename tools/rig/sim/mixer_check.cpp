// Equality check for tools/rig/sim/rig_mixer.h, built and run by tests/test_rig_sim.py (and
// `rig_sim.py mixer-check`). Compiled with the REAL workloads/flight_controller/src/controller_pid.cpp
// and fc_rig.cpp under -DROSE_RIG=1 and the rig-roll build's -D values, so it exercises the rig
// branch of HierarchicalPidController::compute (fc_rig_law_input -> fc_rig_roll_law -> M ->
// forceToVoltage with g_rig.trim_left_applied). The twin's path is the same law followed by
// rigtwin::rig_mix(); every u_out must match bit for bit, across trims, collectives, saturated and
// zero-duty cases. Prints "ok mixer <n> points (<clipped> clipped)" and exits 0 on success.
#include "fc_rig.h"
#include "fc_shared_math.h"
#include "controller_pid.hpp"
#include "rig_mixer.h"

#include <cmath>
#include <cstdio>
#include <cstring>

struct fc_rig_state g_rig;   // controller_pid.cpp's rig branch reads this (main.cpp defines it on target)

int main()
{
	HierarchicalPidController pid;
	int n = 0, fails = 0, clipped_lo = 0, clipped_hi = 0;
	const float trims[] = {1.0f, 0.87f, 0.7f, 1.3f};
	const float colls[] = {0.0f, 0.05f, 0.25f, 0.9f, 1.0f, 1.3f};
	const float rolls[] = {-0.52f, -0.1f, 0.0f, 0.03f, 0.3f};
	const float gyros[] = {-6.0f, -0.4f, 0.0f, 0.2f, 3.0f};
	const float sps[] = {-0.2f, 0.0f, 0.1f};
	const float excs[] = {0.0f, 2e-3f, -5e-3f};   // N*m moment excitation: drives clipping
	for (float trim : trims) {
		for (float coll : colls) {
			for (float exc : excs) {
				fc_rig_init(&g_rig);
				g_rig.gains.v[FC_RIG_MOTOR_TRIM_LEFT] = trim;
				g_rig.trim_left_applied = trim;   // what controller_pid.cpp's mixer reads (slewed by fc_rig_step)
				g_rig.gains.v[FC_RIG_RATE_D] = 0.02f;   // exercise the D filter state too
				g_rig.gains.v[FC_RIG_ATT_I] = 3.0f;
				g_rig.gains.v[FC_RIG_RATE_I] = 20.0f;
				std::memset(&g_rig.out, 0, sizeof g_rig.out);
				g_rig.out.coll_frac = coll;
				g_rig.out.exc_target = FC_RIG_TGT_MOMENT;
				g_rig.out.exc_val = exc;
				g_rig.out.integrate = true;
				pid.init();
				for (float sp : sps) {
					g_rig.out.sp_roll = sp;
					for (float roll : rolls) {
						for (float gx : gyros) {
							float state[12] = {0}, setp[12] = {0}, uo[4], u2[4], u[4];
							state[3] = std::tan(0.5f * roll);   // Gibbs, pure roll
							state[4] = 0.01f;                   // a little pitch for the would-be cmds
							state[9] = gx;
							state[10] = 0.05f;
							state[11] = -0.02f;
							g_rig.sat_pos = (n % 3) == 1;
							g_rig.sat_neg = (n % 5) == 2;
							// the twin's path, on a copy of the law state taken BEFORE compute()
							struct fc_rig_law_state ls = g_rig.law;
							float att[3], yaw_tgt;
							fc_pid_attitude(state, setp, att, &yaw_tgt);
							struct fc_rig_law_in in;
							fc_rig_law_input(&g_rig, att[0], att[1], att[2] - yaw_tgt, &state[9],
									 0.0008f, &in);
							fc_rig_roll_law(&ls, &g_rig.gains, &in, u, nullptr);
							rigtwin::rig_mix(u, g_rig.trim_left_applied, u2);
							// the firmware's path
							pid.compute(state, setp, uo, 0.0008f);
							for (int i = 0; i < 4; i++) {
								if (std::memcmp(&uo[i], &u2[i], sizeof(float)) != 0) {
									if (fails < 10) {
										std::printf("FAIL trim=%g coll=%g exc=%g sp=%g roll=%g gx=%g "
											    "i=%d controller_pid=%.9g twin=%.9g\n",
											    trim, coll, exc, sp, roll, gx, i,
											    (double)uo[i], (double)u2[i]);
									}
									fails++;
								}
								const float d = uo[i] + 0.583f;
								clipped_lo += d <= 0.0f;
								clipped_hi += d >= 1.0f;
							}
							if (std::memcmp(&ls, &g_rig.law, sizeof ls) != 0) {
								std::printf("FAIL law state diverged at trim=%g coll=%g\n", trim, coll);
								fails++;
							}
							n++;
						}
					}
				}
			}
		}
	}
	// the plant-side forward curve inverts force_to_duty inside (0, 1)
	for (int k = 1; k < 100; k++) {
		const double d = k / 100.0;
		const float back = rigtwin::force_to_duty((float)rigtwin::curve_thrust_n(d));
		if (std::fabs(back - d) > 2e-5) {
			std::printf("FAIL curve inverse d=%g back=%g\n", d, (double)back);
			fails++;
		}
	}
	if (clipped_lo == 0 || clipped_hi == 0) {
		std::printf("FAIL coverage: %d zero-duty and %d full-duty motor values\n", clipped_lo, clipped_hi);
		fails++;
	}
	if (fails) {
		std::printf("FAILED mixer: %d mismatches over %d points\n", fails, n);
		return 1;
	}
	std::printf("ok mixer %d points (%d zero-duty, %d full-duty motor values)\n", n, clipped_lo, clipped_hi);
	return 0;
}
