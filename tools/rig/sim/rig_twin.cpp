// Copyright (c) 2026 UC Berkeley
// SPDX-License-Identifier: Apache-2.0
//
// rig_twin: host simulation "twin" of the roll harness. Runs the REAL flight-controller rig code
// (workloads/flight_controller/src/fc_rig.cpp: uplink verbs, run state machine, excitations, roll
// law) and fc_shared_math.h's actuator duty chain, plus a bit-exact transcription of
// controller_pid.cpp's mixer (rig_mixer.h, checked by mixer_check.cpp), against a 1-DOF roll plant
// on a pivot. Built and driven by tools/rig/sim/rig_sim.py; see that file for the model notes.
//
// Per control tick, in main.cpp's order:
//   uplink lines (RESET/ESTOP/DISARM as main.cpp, the rest via fc_rig_handle_command)
//   -> sensors -> envelope guard (rig_safety_violation: rate, impact, battery)
//   -> fc_rig_step -> fc_rig_law_input/fc_rig_roll_law -> mixer -> fc_actuator_duty
//   -> fc_rig_moment_saturation -> send_control (200 Hz frames, or at once on a flag change)
//   -> recorder (every FLIGHTREC_DIV ticks; duty = the last frame SENT, as g_last_duty is)
// and between ticks the ESP applies each frame `delay_ms` after it was sent; the plant integrates
// first-order thrust lags, the pendulum, friction and the stops in `substeps` Euler sub-steps.
//
// Usage:
//   rig_twin scenario=FILE [key=value ...] [csv=OUT.csv] [truth=OUT.csv]   -> one JSON summary line
//   rig_twin batch=FILE [key=value ...]  -> one JSON line per batch line (each: key=value overrides)
//   rig_twin help                        -> the parameter table
// Scenario lines: "<when> <uplink line>", when = ms since boot, or <anchor>+<ms> with anchor in
// arm | run | liftoff | done (first RAMP_UP entry, first RUN entry, roll guard live, first DONE
// after arming). Extra verbs: HBON / HBOFF (host heartbeat at hb_hz), END (stop the simulation).
// key gain.NAME=v, coll=permille, rsp=mdeg prepend "0 GAIN NAME v" / "0 COLL" / "0 RSP".
//
// axis=1 is the PITCH rig (the rig-pitch build, the drone turned 90 deg on the pivot): theta/omega
// are pitch, the plant moment is the mixer's pitch column (rig_mixer.h SIDE_REAR: +u[2] raises
// duty[1]/duty[2]), gravity acts through h_mm and cg_x_mm (CoM forward = nose-down torque), and
// `imbalance` splits rear/front. ext_mNm (the tethered USB cable) acts about the pivot whatever the
// axis; with the cable routed along the pitch pivot it is ~0, its default. The FC side is
// the REAL pitch build (fc_rig.cpp compiled with -DRIG_AXIS=1): see TWIN_FW_AXIS_AWARE below.
#include "fc_rig.h"
#include "fc_shared_math.h"
#include "rig_mixer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#ifndef AUTOFLIGHT_MAX_DUTY
#define AUTOFLIGHT_MAX_DUTY 0.45f
#endif
#ifndef FLIGHTREC_DIV
#define FLIGHTREC_DIV 4
#endif
// main.cpp's guard / battery constants (its defaults; the rig-roll mode does not override them)
#ifndef SAFE_MAX_RATE_RADPS
#define SAFE_MAX_RATE_RADPS 10.0f
#endif
#ifndef SAFE_IMPACT_MPS2
#define SAFE_IMPACT_MPS2 29.43f
#endif
#ifndef SAFE_DEBOUNCE_MS
#define SAFE_DEBOUNCE_MS 120
#endif
#ifndef SAFE_SHOCK_DEBOUNCE_MS
#define SAFE_SHOCK_DEBOUNCE_MS 40
#endif
#ifndef SAFE_DEBOUNCE_MIN_SAMPLES
#define SAFE_DEBOUNCE_MIN_SAMPLES 2
#endif
#ifndef BATT_NOMINAL_V
#define BATT_NOMINAL_V 3.8f
#endif
#ifndef BATT_SCALE_MAX
#define BATT_SCALE_MAX 1.30f
#endif
#ifndef BATT_ARM_MIN_V
#define BATT_ARM_MIN_V 3.4f
#endif
#ifndef BATT_CUTOFF_V
#define BATT_CUTOFF_V 3.2f
#endif
#ifndef BATT_CHECK_DIV
#define BATT_CHECK_DIV 200
#endif
#ifndef BATT_SMOOTH_ALPHA
#define BATT_SMOOTH_ALPHA 0.20f
#endif
#ifndef RIG_MAX_TILT_RAD
#define RIG_MAX_TILT_RAD 0.44f
#endif
#ifndef RIG_AXIS
#define RIG_AXIS 0   // the build's controlled axis (0 roll, 1 pitch): -DRIG_AXIS=1 in rig-pitch
#endif

// ---- the controlled axis on the FC side ---------------------------------------------------------
// TWIN_FW_AXIS_AWARE=1 (the default): fc_rig.cpp selects its axis itself (-DRIG_AXIS, rig-pitch),
// so the twin hands the REAL fc_rig_step / fc_rig_roll_law / fc_rig_moment_saturation the PHYSICAL
// roll, pitch and gyro[3] exactly as main.cpp and controller_pid.cpp do, and the firmware puts the
// pitch moment in u[2], guards roll as the locked axis and takes the saturation on the rear/front
// pairs. One axis per binary: rig_sim.py builds rig_twin (rig-roll's defines) and rig_twin_pitch
// (rig-pitch's: RIG_AXIS=1), and `axis` defaults to the binary's RIG_AXIS.
// TWIN_FW_AXIS_AWARE=0 is the earlier transcription, kept as a cross-check of the firmware's pitch
// path: a ROLL build (RIG_AXIS 0) fed the pitch build's view for axis=1 -- the controlled angle and
// rate in the roll slots, the locked roll in the pitch slot, the law's moment moved to u[2] (the
// flight PID's pitch path is the roll path with J[1] == J[0], rig_mixer.h J_PITCH) and the
// saturation on the pitch pairs. tests/test_rig_pitch.py runs both on one scenario and requires the
// same numbers.
#ifndef TWIN_FW_AXIS_AWARE
#define TWIN_FW_AXIS_AWARE 1
#endif
#if !TWIN_FW_AXIS_AWARE && RIG_AXIS != 0
#error "TWIN_FW_AXIS_AWARE=0 remaps axis=1 onto a ROLL build: compile it with RIG_AXIS 0"
#endif

// fc_rig_step inputs: (angle into roll_rad, angle into pitch_rad, gyro[]) for the build's axis.
static void rig_step_view(int axis, float ctl, float locked, const float gphys[3], float *roll_rad,
			  float *pitch_rad, float g[3])
{
#if TWIN_FW_AXIS_AWARE
	*roll_rad = axis ? locked : ctl;
	*pitch_rad = axis ? ctl : locked;
	for (int i = 0; i < 3; i++) g[i] = gphys[i];
#else
	*roll_rad = ctl;
	*pitch_rad = locked;
	g[0] = gphys[axis ? 1 : 0];
	g[1] = gphys[axis ? 0 : 1];
	g[2] = gphys[2];
#endif
}

// controller_pid.cpp's rig branch up to the mixer input u[]: fc_rig_law_input + fc_rig_roll_law.
// dbg.roll/dbg.pitch come out PHYSICAL (the recorder contract); dbg's other fields are the
// controlled axis'.
static void rig_law(int axis, float ctl, float locked, const float gphys[3], float dt, float u[4])
{
	struct fc_rig_law_in lin;
#if TWIN_FW_AXIS_AWARE
	(void)axis;
	fc_rig_law_input(&g_rig, axis ? locked : ctl, axis ? ctl : locked, 0.0f, gphys, dt, &lin);
	fc_rig_roll_law(&g_rig.law, &g_rig.gains, &lin, u, &g_rig.dbg);
#else
	float r, pch, g[3];
	rig_step_view(axis, ctl, locked, gphys, &r, &pch, g);
	fc_rig_law_input(&g_rig, r, pch, 0.0f, g, dt, &lin);
	fc_rig_roll_law(&g_rig.law, &g_rig.gains, &lin, u, &g_rig.dbg);
	if (axis == 1) {
		u[2] = u[1] * (rigtwin::J_PITCH / rigtwin::J_ROLL);   // == u[1] while J[1] == J[0]
		u[1] = 0.0f;
		g_rig.dbg.roll = locked;
		g_rig.dbg.pitch = ctl;
	}
#endif
}

// fc_rig_moment_saturation on the pair the controlled moment moves: left/right for roll; for pitch
// the rear pair {duty[1], duty[2]} goes into its "+" slots [2],[3] and the front {duty[0], duty[3]}
// into its "-" slots [0],[1].
static void rig_saturation(int axis, const float duty[4], float ceiling, bool *sp, bool *sn)
{
#if TWIN_FW_AXIS_AWARE
	(void)axis;
	fc_rig_moment_saturation(duty, ceiling, sp, sn);
#else
	if (axis == 1) {
		const float perm[4] = {duty[0], duty[3], duty[1], duty[2]};
		fc_rig_moment_saturation(perm, ceiling, sp, sn);
	} else {
		fc_rig_moment_saturation(duty, ceiling, sp, sn);
	}
#endif
}

struct fc_rig_state g_rig;   // the one instance, as main.cpp

static const double G = 9.81;
static const double PI = 3.14159265358979323846;

// ---- parameters ------------------------------------------------------------------------------
struct ParamInfo { const char *key; double def; const char *help; };
static const ParamInfo PARAMS[] = {
	{"mass", 0.055, "kg, vehicle mass on the pivot (prior 0.050-0.060)"},
	{"arm", 0.033, "m, TRUE lateral moment arm of each motor (FC assumes 0.033)"},
	{"I", 2.5e-5, "kg m^2, roll inertia about the CoM (code 16e-6, scaling 3-4e-5)"},
	{"cradle_frac", 0.0, "extra inertia of the cradle, fraction of I (<= 0.10)"},
	{"h_mm", 1.0, "mm, CoM below the pivot axis (negative = above: unstable)"},
	{"cg_y_mm", 0.0, "mm, CoM lateral offset (+ = left: a torque toward -roll)"},
	{"ext_mNm", 0.0, "mN m, constant external torque, + = toward +roll (right wing down): a tethered cable"},
	{"axis", (double)RIG_AXIS, "0 = roll rig (rig-roll), 1 = pitch rig (rig-pitch: pivot turned 90 deg; I = Iyy, same prior); default: the build's RIG_AXIS"},
	{"cg_x_mm", 0.0, "mm, CoM front/back offset (+ = forward: a torque toward +pitch, nose down); pitch rig"},
	{"tau_up", 0.070, "s, thrust lag time constant, rising"},
	{"tau_down", 0.070, "s, thrust lag time constant, falling"},
	{"tau_speed", 0.0, "1 = scale tau with rotor speed (evidence model), 0 = constant"},
	{"tau_aero_frac", 0.48, "aero share of the spin-up damping at hover (evidence priors)"},
	{"delay_ms", 2.0, "ms, FC frame sent -> ESP applies (UART 14 B @115200 + ISR)"},
	{"frame_ms", 5.0, "ms, ESP_MOTORS_PERIOD_MS (200 Hz)"},
	{"esp_max_duty", 0.98, "ESP_LINK_MAX_DUTY_PCT/100 of the esp_bridge motors image"},
	{"coulomb", 5e-6, "N m, pivot Coulomb friction (tanh-regularised; ball bearing ~2e-6, bushing ~1e-4)"},
	{"viscous", 2e-6, "N m s/rad, pivot viscous friction"},
	{"stop_deg", 30.0, "deg, hard stops at +-stop_deg"},
	{"restitution", 0.2, "stop bounce coefficient (0 = fully inelastic)"},
	{"gyro_noise", 0.005, "rad/s RMS white noise per sample"},
	{"gyro_bias", 0.0, "rad/s residual roll-gyro bias"},
	{"roll_bias_deg", 0.0, "deg, attitude-estimate offset"},
	{"roll_noise_deg", 0.0, "deg RMS attitude-estimate noise"},
	{"imu_r_mm", 20.0, "mm, IMU distance from the pivot (tangential accel)"},
	{"imbalance", 0.0, "left/right thrust ratio - 1 (left x(1+i/2), right x(1-i/2))"},
	{"thrust_scale", 1.0, "true/curve thrust multiplier (0.7-1.3)"},
	{"v_ref", 3.37, "V, bus voltage at which the curve x scale is true (evidence hover duty)"},
	{"lipo", 0.0, "0 = stiff PSU at v_psu, 1 = LiPo Thevenin model"},
	{"v_psu", 4.2, "V, bench supply"},
	{"ocv", 3.9, "V, LiPo open-circuit voltage"},
	{"r0", 0.180, "ohm, LiPo series resistance (evidence fit)"},
	{"rp", 0.049, "ohm, LiPo polarisation resistance (evidence fit)"},
	{"tau_p", 8.9, "s, LiPo polarisation time constant (evidence fit)"},
	{"i_base", 0.36, "A, board current (evidence fit)"},
	{"i_motor_hover", 0.84, "A, battery current per motor at hover thrust (evidence ~3.5-4 A total)"},
	{"loop_hz", 355.0, "Hz, control loop rate when no dt_file (measured mean 2821 us, 2026-11-04)"},
	{"jitter", 0.0, "fractional uniform jitter of the loop period"},
	{"substeps", 8.0, "plant sub-steps per control tick"},
	{"cmd_latency_ms", 15.0, "ms, host -> WiFi -> ESP -> FC uplink latency"},
	{"hb_hz", 20.0, "Hz, host heartbeat rate while HBON"},
	{"theta0_deg", 0.0, "deg, initial roll (clamped to the stops)"},
	{"omega0", 0.0, "rad/s, initial roll rate"},
	{"t_max_s", 40.0, "s, hard end of the simulation"},
	{"tail_ms", 2000.0, "ms simulated after DONE (FLIGHTREC_TAIL_MS)"},
	{"boot_ms", 10000.0, "ms of uptime at t = 0 (gyro cal done)"},
	{"seed", 1.0, "noise seed"},
	{"quiet_ms", 1000.0, "ms window before the first EXC used for steady-state metrics"},
};

struct Params {
	std::map<std::string, double> v;
	std::string scenario, csv, truth, id, dt_file;   // dt_file: one loop period (us) per line
	std::vector<std::string> pre;   // prepended uplink lines at t = 0
	Params() { for (const auto &p : PARAMS) v[p.key] = p.def; }
	double operator[](const char *k) const { return v.at(k); }
	bool set(const std::string &kv, std::string *err)
	{
		const size_t eq = kv.find('=');
		if (eq == std::string::npos) { *err = "expected key=value: " + kv; return false; }
		const std::string k = kv.substr(0, eq), val = kv.substr(eq + 1);
		if (k == "scenario") { scenario = val; return true; }
		if (k == "csv") { csv = val; return true; }
		if (k == "truth") { truth = val; return true; }
		if (k == "id") { id = val; return true; }
		if (k == "dt_file") { dt_file = val; return true; }
		if (k.rfind("gain.", 0) == 0) { pre.push_back("GAIN " + k.substr(5) + " " + val); return true; }
		if (k == "coll") { pre.push_back("COLL " + val); return true; }
		if (k == "rsp") { pre.push_back("RSP " + val); return true; }
		auto it = v.find(k);
		if (it == v.end()) { *err = "unknown parameter " + k; return false; }
		char *end = nullptr;
		const double d = std::strtod(val.c_str(), &end);
		if (end == val.c_str() || *end != '\0') { *err = "bad number for " + k + ": " + val; return false; }
		it->second = d;
		return true;
	}
};

// ---- scenario ----------------------------------------------------------------------------------
enum Anchor { A_ABS = 0, A_ARM, A_RUN, A_LIFTOFF, A_DONE, A_COUNT };
struct Cmd { Anchor anchor; double ms; std::string line; bool sent = false; };

static bool parse_scenario(const std::string &path, const std::vector<std::string> &pre,
			   std::vector<Cmd> *out, std::string *err)
{
	for (const auto &p : pre) {
		out->push_back(Cmd{A_ABS, 0.0, p});
	}
	if (path.empty()) {
		return true;
	}
	std::ifstream f(path);
	if (!f) { *err = "cannot open scenario " + path; return false; }
	std::string line;
	int ln = 0;
	while (std::getline(f, line)) {
		ln++;
		const size_t hash = line.find('#');
		if (hash != std::string::npos) line = line.substr(0, hash);
		std::istringstream is(line);
		std::string when;
		if (!(is >> when)) continue;
		std::string rest;
		std::getline(is, rest);
		rest.erase(0, rest.find_first_not_of(" \t"));
		while (!rest.empty() && (rest.back() == ' ' || rest.back() == '\t' || rest.back() == '\r')) rest.pop_back();
		Cmd c{A_ABS, 0.0, rest};
		const size_t plus = when.find('+');
		std::string num = when;
		if (plus != std::string::npos) {
			const std::string a = when.substr(0, plus);
			num = when.substr(plus + 1);
			if (a == "arm") c.anchor = A_ARM;
			else if (a == "run") c.anchor = A_RUN;
			else if (a == "liftoff") c.anchor = A_LIFTOFF;
			else if (a == "done") c.anchor = A_DONE;
			else { *err = path + ":" + std::to_string(ln) + ": unknown anchor " + a; return false; }
		}
		char *end = nullptr;
		c.ms = std::strtod(num.c_str(), &end);
		if (end == num.c_str() || *end != '\0' || rest.empty()) {
			*err = path + ":" + std::to_string(ln) + ": expected '<when> <command>'";
			return false;
		}
		out->push_back(c);
	}
	return true;
}

// ---- small helpers -----------------------------------------------------------------------------
static std::string jstr(const std::string &s)
{
	std::string o = "\"";
	for (char c : s) {
		if (c == '"' || c == '\\') { o += '\\'; o += c; }
		else if (c == '\n') o += "\\n";
		else if ((unsigned char)c < 0x20) o += ' ';
		else o += c;
	}
	return o + "\"";
}
static std::string jnum(double v)
{
	if (!std::isfinite(v)) return "null";
	char b[48];
	std::snprintf(b, sizeof b, "%.6g", v);
	return b;
}

struct Frame { double t_apply; float duty[4]; };

struct Row {   // one recorder sample (struct flightrec_rig_sample, before CSV formatting)
	uint32_t t_us; uint16_t dt_us; uint8_t seq, flags, rs, exc_id; uint16_t gseq;
	float gyro[3], roll, pitch, roll_tgt, rate_tgt, rate_cmd, moment, exc_val, coll, att_int, rate_int,
	      pitch_cmd, yaw_cmd, ay, az, vbat;
	uint16_t duty[4];
};

struct TruthRow { double t, theta, omega, moment_true, vbus, batt_scale; float applied[4]; double thrust[4]; int rs; };

// ---- one simulation ----------------------------------------------------------------------------
static int simulate(const Params &P, std::string *json)
{
	std::string err;
	std::vector<Cmd> cmds;
	if (!parse_scenario(P.scenario, P.pre, &cmds, &err)) {
		*json = "{\"error\":" + jstr(err) + "}";
		return 2;
	}
	std::mt19937_64 rng((uint64_t)P["seed"]);
	std::normal_distribution<double> nrm(0.0, 1.0);
	std::uniform_real_distribution<double> uni(-1.0, 1.0);

	// plant
	const int axis = P["axis"] != 0.0 ? 1 : 0;
	const double m = P["mass"], arm = P["arm"], h = P["h_mm"] * 1e-3, cgy = P["cg_y_mm"] * 1e-3;
	const double cgx = P["cg_x_mm"] * 1e-3;
#if TWIN_FW_AXIS_AWARE
	if (axis != RIG_AXIS) {
		*json = "{\"error\":" + jstr("axis=" + std::to_string(axis) + " but this twin was built with RIG_AXIS=" +
					      std::to_string(RIG_AXIS)) + "}";
		return 2;
	}
#endif
	// the offset across the pivot: lateral (y) on the roll rig, fore/aft (x) on the pitch rig; the
	// torque sign follows the axis (+y CoM rolls left-down = -roll, +x CoM pitches nose-down = +pitch)
	const double cg_off = axis ? -cgx : cgy;
	const double Itot = P["I"] * (1.0 + P["cradle_frac"]) + m * (h * h + cg_off * cg_off);
	const double ext_torque = P["ext_mNm"] * 1e-3;
	const double stop = P["stop_deg"] * PI / 180.0;
	const double T_hover = m * G / 4.0;   // true per-motor hover thrust (aero reference)
	double theta = std::max(-stop, std::min(stop, P["theta0_deg"] * PI / 180.0));
	double omega = P["omega0"];
	double thrust[4] = {0, 0, 0, 0};
	float applied[4] = {0, 0, 0, 0};
	double side_k[4];
	for (int i = 0; i < 4; i++) {
		const int side = axis ? rigtwin::SIDE_REAR[i] : rigtwin::SIDE_LEFT[i];   // the "+moment" pair
		side_k[i] = P["thrust_scale"] * (side ? 1.0 + 0.5 * P["imbalance"] : 1.0 - 0.5 * P["imbalance"]);
	}
	const bool lipo = P["lipo"] != 0.0;
	double vbus = lipo ? P["ocv"] : P["v_psu"], vp = 0.0;
	float g_vbat = 0.0f;   // FC's EMA (battery_poll)

	// FC side
	fc_rig_init(&g_rig);
	bool arm_enabled = false, estop = false, armed = false, arming = false, committed = false;
	uint32_t reset_gen = 0, reset_seen_main = 0;
	int64_t viol_since = 0; int viol_count = 0;
	const char *estop_reason = nullptr;
	const float ceiling = ((float)(AUTOFLIGHT_MAX_DUTY) < 1.0f) ? (float)(AUTOFLIGHT_MAX_DUTY) : 1.0f;
	float last_sent[4] = {0, 0, 0, 0};
	int64_t next_ms = 0;
	int last_flags = -1;
	std::deque<Frame> link;
	std::deque<std::pair<double, std::string>> uplink;   // (deliver time s, line)
	bool hb_on = false;
	double next_hb = 0.0;
	double prev_omega_acc = omega;

	// anchors
	double anchor_t[A_COUNT];
	bool anchor_set[A_COUNT] = {true, false, false, false, false};
	for (double &a : anchor_t) a = 0.0;
	bool have_armed = false;

	// metrics
	int naks = 0, acks = 0;
	std::vector<std::string> events, nak_lines;
	double liftoff_ms = -1, max_live_deg = 0.0, max_est_after_liftoff = 0.0, max_impact = 0.0;
	// lift-off assist: angle integrator at the hand-off and its peak; the last RUN tick after
	// lift-off with |true roll| > 2 deg (settle-to-level time = that - lift-off)
	double liftoff_att_int = NAN, att_int_peak = 0.0, last_off_level_t = -1.0, liftoff_t = -1.0;
	bool settle_seen = false;
	double max_est_live = -1.0;   // max |roll estimate| over RUN after lift-off (guard margin there)
	double roll_hi_live = -1e9, roll_lo_live = 1e9;   // true roll extremes after lift-off, motors live
	bool near_level = false;   // reached RUN: the guard margin counts from here
	int stop_hits_live = 0, stop_hits_total = 0;
	long run_ticks = 0, sat_ticks = 0;
	double peak_duty = 0.0, sum_err2 = 0.0;
	double first_exc_t = -1.0, end_t = -1.0;
	std::string first_exc;
	std::vector<double> rt, rerr, rroll, rtgt;   // RUN ticks: time, err, roll, target (deg)
	std::string status;
	double done_t = -1.0;
	double run_enter_t = -1.0;
	std::string rd_reason;

	std::vector<Row> rows;
	std::vector<TruthRow> truth;
	const bool want_truth = !P.truth.empty();

	const double dt_nom = 1.0 / P["loop_hz"];
	// Loop periods: replayed from a measured dt_us sequence (recorder column), from a seed-chosen
	// offset and wrapping, so the stalls and their clustering come along; else loop_hz +- jitter.
	std::vector<double> dts;
	if (!P.dt_file.empty()) {
		std::ifstream df(P.dt_file);
		double v;
		while (df >> v) if (v > 0) dts.push_back(v * 1e-6);
		if (dts.empty()) { *json = "{\"error\":" + jstr("no periods in " + P.dt_file) + "}"; return 2; }
	}
	size_t dt_idx = dts.empty() ? 0 : (size_t)(rng() % dts.size());
	const int nsub = std::max(1, (int)P["substeps"]);
	double t = 0.0;
	uint32_t iter = 0;
	uint32_t rec_calls = 0;
	const double boot = P["boot_ms"] * 1e-3;
	const double t_max = P["t_max_s"];
	bool ended = false;
	char reply[FC_RIG_REPLY_MAX];

	while (!ended && t < t_max) {
		const double dt = !dts.empty() ? dts[dt_idx++ % dts.size()] : dt_nom * (1.0 + P["jitter"] * uni(rng));
		const int64_t now_ms = (int64_t)std::floor((t + boot) * 1000.0);

		// ---- host: scenario commands and heartbeat onto the uplink ------------------------------
		for (auto &c : cmds) {
			if (c.sent || !anchor_set[c.anchor]) continue;
			if (t * 1000.0 + 1e-9 >= anchor_t[c.anchor] * 1000.0 + c.ms) {
				c.sent = true;
				if (c.line == "HBON") { hb_on = true; next_hb = t; continue; }
				if (c.line == "HBOFF") { hb_on = false; continue; }
				if (c.line == "END") { ended = true; continue; }
				uplink.emplace_back(t + P["cmd_latency_ms"] * 1e-3, c.line);
			}
		}
		if (ended) break;
		if (hb_on && t >= next_hb) {
			uplink.emplace_back(t + P["cmd_latency_ms"] * 1e-3, "HB");
			next_hb += 1.0 / P["hb_hz"];
		}
		// ---- FC: uart_cmd_poll -------------------------------------------------------------------
		bool reset_now = false;
		while (!uplink.empty() && uplink.front().first <= t) {
			const std::string line = uplink.front().second;
			uplink.pop_front();
			if (line == "RESET") {
				arm_enabled = true; reset_gen++; reset_now = true;
			} else if (line == "ESTOP") {
				estop = true; armed = false;
				if (!estop_reason) estop_reason = "ESTOP command";
			} else if (line == "DISARM") {
				fc_rig_disarm(&g_rig);
			} else if (!fc_rig_handle_command(&g_rig, line.c_str(), now_ms, reply, sizeof reply)) {
				naks++;
				nak_lines.push_back(line + " -> not a rig verb");
			} else if (std::strncmp(reply, "FCNAK", 5) == 0 || std::strstr(reply, "\nFCNAK")) {
				naks++;
				nak_lines.push_back(line + " -> " + (g_rig.nak_reason ? g_rig.nak_reason : "?"));
			} else if (line != "HB") {
				acks++;
			}
		}
		// soft-RESET block (main.cpp): clears estop, disarms, clears the saturation flags
		if (reset_now || reset_gen != reset_seen_main) {
			reset_seen_main = reset_gen;
			estop = false; armed = false; arming = false; estop_reason = nullptr;
			g_rig.sat_pos = g_rig.sat_neg = false;
		}

		// ---- sensors -----------------------------------------------------------------------------
		// physical body rates (p, q, r); theta/omega are the controlled axis (roll, or pitch: axis=1).
		// roll_est is the CONTROLLED angle estimate (roll_bias_deg / roll_noise_deg apply to it);
		// the locked axis reads 0.
		float gyro[3];
		gyro[0] = (float)(omega + P["gyro_bias"] + P["gyro_noise"] * nrm(rng));
		gyro[1] = (float)(P["gyro_noise"] * nrm(rng));
		gyro[2] = (float)(P["gyro_noise"] * nrm(rng));
		if (axis) std::swap(gyro[0], gyro[1]);
		const float roll_est = (float)(theta + (P["roll_bias_deg"] + P["roll_noise_deg"] * nrm(rng)) * PI / 180.0);
		const float locked_est = 0.0f;
		// specific force at the IMU, body frame: gravity + tangential (r*alpha) + centripetal
		const double alpha_est = (omega - prev_omega_acc) / dt;
		prev_omega_acc = omega;
		const double r_imu = P["imu_r_mm"] * 1e-3;
		float accel[3];
		accel[0] = 0.0f;
		accel[1] = (float)(G * std::sin(theta) + r_imu * alpha_est);
		if (axis) {   // pitch about +y: gravity's body-x component is -g sin(theta)
			accel[0] = -accel[1];
			accel[1] = 0.0f;
		}
		accel[2] = (float)(G * std::cos(theta) + r_imu * omega * omega);
		// FC battery_poll: EMA of the bus every BATT_CHECK_DIV iterations
		if (iter % BATT_CHECK_DIV == 0) {
			const float v = (float)vbus;
			g_vbat = g_vbat <= 0.0f ? v : g_vbat + BATT_SMOOTH_ALPHA * (v - g_vbat);
		}
		float batt_scale = 1.0f;
		if (g_vbat >= 1.0f && g_vbat <= 5.0f) {
			batt_scale = std::min(BATT_SCALE_MAX, std::max(1.0f, BATT_NOMINAL_V / g_vbat));
		}
		const bool batt_ok = !(g_vbat >= 1.0f && g_vbat <= 5.0f) || g_vbat >= BATT_ARM_MIN_V;

		// ---- envelope guard (rig_safety_violation) -------------------------------------------------
		{
			const char *why = nullptr;
			int need_ms = SAFE_DEBOUNCE_MS;
			const float amag = std::sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
			float rate = std::max(std::fabs(gyro[0]), std::max(std::fabs(gyro[1]), std::fabs(gyro[2])));
			if (amag > SAFE_IMPACT_MPS2) { why = "impact"; need_ms = SAFE_SHOCK_DEBOUNCE_MS; }
			else if (rate > SAFE_MAX_RATE_RADPS) { why = "rate"; }
			else if (g_vbat >= 1.0f && g_vbat < BATT_CUTOFF_V) { why = "battery"; }
			if (!armed || estop) why = nullptr;
			if (fc_guard_dwell(why != nullptr, now_ms, need_ms, SAFE_DEBOUNCE_MIN_SAMPLES, &viol_since, &viol_count)) {
				estop = true; armed = false;
				if (!estop_reason) estop_reason = why;
				events.push_back(std::to_string((long)(t * 1000)) + " envelope estop: " + why);
			}
		}

		// ---- fc_rig_step -------------------------------------------------------------------------
		struct fc_rig_inputs ri;
		struct fc_rig_outputs ro;
		ri.now_ms = now_ms;
		ri.dt = (float)dt;
		ri.arm_enabled = arm_enabled;
		ri.gyro_cal_done = true;
		rig_step_view(axis, roll_est, locked_est, gyro, &ri.roll_rad, &ri.pitch_rad, ri.gyro);
		ri.batt_ok = batt_ok;
		ri.estop = estop;
		ri.reset_gen = reset_gen;
		const uint8_t rs_before = g_rig.rs;
		fc_rig_step(&g_rig, &ri, &ro);
		if (ro.estop_req && !estop) {
			estop = true;
			if (!estop_reason) estop_reason = ro.estop_reason;
		}
		armed = ro.armed && !estop;
		arming = ro.arming;
		if (ro.event) {
			std::string ev = std::to_string((long)(t * 1000)) + " " + ro.event;
			if (ro.estop_req && ro.estop_reason) ev += std::string(": ") + ro.estop_reason;
			events.push_back(ev);
		}
		if (g_rig.rs == FC_RIG_RAMP_UP && rs_before != FC_RIG_RAMP_UP && !anchor_set[A_ARM]) {
			anchor_set[A_ARM] = true; anchor_t[A_ARM] = t; have_armed = true;
		}
		if (g_rig.rs == FC_RIG_RUN && !anchor_set[A_RUN]) {
			anchor_set[A_RUN] = true; anchor_t[A_RUN] = t; run_enter_t = t;
		}
		if (have_armed && g_rig.guard_active && !anchor_set[A_LIFTOFF]) {
			anchor_set[A_LIFTOFF] = true; anchor_t[A_LIFTOFF] = t;
			liftoff_ms = (t - anchor_t[A_ARM]) * 1000.0;
			liftoff_att_int = g_rig.law.att_int;   // the law has not run yet this tick: the hand-off value
			liftoff_t = t;
		}
		if (have_armed && g_rig.rs == FC_RIG_DONE && !anchor_set[A_DONE]) {
			anchor_set[A_DONE] = true; anchor_t[A_DONE] = t; done_t = t;
			if (g_rig.rd_reason) rd_reason = g_rig.rd_reason;
		}
		if (g_rig.rs == FC_RIG_RAMP_DOWN && g_rig.rd_reason) rd_reason = g_rig.rd_reason;
		if (g_rig.exc.type != FC_RIG_EXC_NONE && first_exc_t < 0) {
			first_exc_t = t;
			static const char *tn[] = {"none", "step", "doublet", "chirp", "sine", "prbs"};
			static const char *gn[] = {"att", "rate", "moment"};
			first_exc = std::string(tn[g_rig.exc.type]) + " " + gn[g_rig.exc.target];
		}

		// ---- controller_pid.cpp rig branch + send_control/actuator_duty -----------------------------
		float u[4], uo[4], duty[4] = {0, 0, 0, 0};
		{
			rig_law(axis, roll_est, locked_est, gyro, (float)dt, u);
			rigtwin::rig_mix(u, g_rig.trim_left_applied, uo);   // as controller_pid.cpp's rig MOTOR_TRIM_LEFT
		}
		bool armed_now = false;
		if (armed && !estop) {
			armed_now = true;
			fc_actuator_duty(uo, duty, batt_scale, ceiling, true, 1.0f);
			rig_saturation(axis, duty, ceiling, &g_rig.sat_pos, &g_rig.sat_neg);
			committed = true;
		}
		const int flags_wire = (estop ? 1 : 0) | (armed_now ? 2 : 0);
		if (!(flags_wire == last_flags && now_ms < next_ms)) {
			last_flags = flags_wire;
			next_ms = now_ms + (int64_t)P["frame_ms"];
			Frame fr;
			fr.t_apply = t + P["delay_ms"] * 1e-3;
			for (int i = 0; i < 4; i++) { fr.duty[i] = duty[i]; last_sent[i] = duty[i]; }
			link.push_back(fr);
		}

		// ---- metrics on this tick ----------------------------------------------------------------
		const double est_deg = std::fabs(roll_est) * 180.0 / PI;
		// RAMP_UP and RUN only: in RAMP_DOWN settling back onto a stop is the expected end of a run
		const bool holding = armed && (g_rig.rs == FC_RIG_RAMP_UP || g_rig.rs == FC_RIG_RUN);
		// guard margin: over RUN (the setpoint slew from a stop is over by then)
		if (armed && g_rig.rs == FC_RIG_RUN) {
			near_level = true;
			max_est_after_liftoff = std::max(max_est_after_liftoff, est_deg);
		}
		if (holding && g_rig.guard_active) {
			max_live_deg = std::max(max_live_deg, std::fabs(theta) * 180.0 / PI);
			roll_hi_live = std::max(roll_hi_live, theta * 180.0 / PI);
			roll_lo_live = std::min(roll_lo_live, theta * 180.0 / PI);
		}
		if (armed) att_int_peak = std::max(att_int_peak, (double)std::fabs(g_rig.law.att_int));
		if (g_rig.rs == FC_RIG_RUN && armed && liftoff_t >= 0) {
			settle_seen = true;
			if (std::fabs(theta) * 180.0 / PI > 2.0) last_off_level_t = t;
			max_est_live = std::max(max_est_live, est_deg);
		}
		if (g_rig.rs == FC_RIG_RUN && armed) {
			run_ticks++;
			bool sat = false;
			for (int i = 0; i < 4; i++) {
				peak_duty = std::max(peak_duty, (double)duty[i]);
				sat = sat || duty[i] >= 0.94f || duty[i] <= 0.01f;
			}
			sat_ticks += sat;
			const double ctl_used = axis ? g_rig.dbg.pitch : g_rig.dbg.roll;   // the controlled angle
			const double e = (g_rig.dbg.roll_tgt - ctl_used) * 180.0 / PI;
			sum_err2 += e * e;
			rt.push_back(t); rerr.push_back(e);
			rroll.push_back(ctl_used * 180.0 / PI); rtgt.push_back(g_rig.dbg.roll_tgt * 180.0 / PI);
		}

		// ---- recorder ----------------------------------------------------------------------------
		if ((rec_calls++ % FLIGHTREC_DIV) == 0u) {
			Row r;
			const double tu = (t + boot) * 1e6;
			r.t_us = (uint32_t)(uint64_t)tu;
			r.dt_us = (uint16_t)std::min(dt * 1e6 + 0.5, 65535.0);   // rounded: dt may be a replayed us value
			r.seq = (uint8_t)iter;
			r.flags = (uint8_t)((armed ? 0x01 : 0) | (estop ? 0x02 : 0) | (arming ? 0x04 : 0) | 0x08 |
					    (committed ? 0x40 : 0));
			r.rs = g_rig.rs; r.exc_id = (uint8_t)g_rig.exc_id; r.gseq = (uint16_t)g_rig.gain_seq;
			for (int i = 0; i < 3; i++) r.gyro[i] = gyro[i];
			const fc_rig_dbg &d = g_rig.dbg;
			r.roll = d.roll; r.pitch = d.pitch; r.roll_tgt = d.roll_tgt; r.rate_tgt = d.rate_tgt;
			r.rate_cmd = d.rate_cmd; r.moment = d.moment; r.exc_val = d.exc_val; r.coll = d.coll_thrust;
			r.att_int = d.att_int; r.rate_int = d.rate_int; r.pitch_cmd = d.pitch_cmd; r.yaw_cmd = d.yaw_cmd;
			r.ay = accel[1]; r.az = accel[2]; r.vbat = g_vbat;
			for (int i = 0; i < 4; i++) {
				const float du = std::min(std::max(last_sent[i], 0.0f), 1.0f);
				r.duty[i] = (uint16_t)(du * 10000.0f + 0.5f);
			}
			if (!P.csv.empty()) rows.push_back(r);
		}

		// ---- plant: integrate over this tick ------------------------------------------------------
		const double hs = dt / nsub;
		for (int k = 0; k < nsub; k++) {
			const double ts = t + k * hs;
			while (!link.empty() && link.front().t_apply <= ts) {
				for (int i = 0; i < 4; i++) applied[i] = std::min(link.front().duty[i], (float)P["esp_max_duty"]);
				link.pop_front();
			}
			double ibatt = P["i_base"];
			for (int i = 0; i < 4; i++) {
				const double deff = (double)applied[i] * vbus / P["v_ref"];
				const double tss = side_k[i] * rigtwin::curve_thrust_n(deff);
				double tau = tss >= thrust[i] ? P["tau_up"] : P["tau_down"];
				if (P["tau_speed"] != 0.0) {
					// spin-up damping = electrical (constant) + aero (prop. rotor speed ~ sqrt(T));
					// coast-down (no regeneration) is aero only. tau scales accordingly vs hover.
					const double s = std::sqrt(std::max(std::max(thrust[i], tss), 0.02 * T_hover) / T_hover);
					const double a = P["tau_aero_frac"];
					tau = tss >= thrust[i] ? P["tau_up"] / ((1.0 - a) + a * s) : P["tau_down"] / s;
					tau = std::min(tau, 0.5);
				}
				thrust[i] += (tss - thrust[i]) * (1.0 - std::exp(-hs / tau));
				if (applied[i] > 0.0f && lipo) {
					ibatt += P["i_motor_hover"] * std::pow(std::max(thrust[i], 0.0) / T_hover, 1.5);
				}
			}
			// roll: left pair - right pair; pitch (M column 2): rear {1, 2} - front {0, 3}
			const double M = axis ? arm * ((thrust[1] + thrust[2]) - (thrust[0] + thrust[3]))
					      : arm * ((thrust[2] + thrust[3]) - (thrust[0] + thrust[1]));
			const double tg = -m * G * (h * std::sin(theta) + cg_off * std::cos(theta));
			const double tf = -P["viscous"] * omega - P["coulomb"] * std::tanh(omega / 0.01);
			omega += hs * (M + tg + tf + ext_torque) / Itot;
			theta += hs * omega;
			if (theta > stop || theta < -stop) {
				const double imp = std::fabs(omega);
				theta = theta > 0 ? stop : -stop;
				if ((theta > 0 && omega > 0) || (theta < 0 && omega < 0)) {
					if (imp > 0.05) {
						stop_hits_total++;
						if (armed && g_rig.guard_active && g_rig.rs != FC_RIG_RAMP_DOWN) {
							stop_hits_live++;
							max_impact = std::max(max_impact, imp);
						}
					}
					omega = -P["restitution"] * omega;
				}
			}
			if (lipo) {
				vp += hs * (ibatt * P["rp"] - vp) / P["tau_p"];
				vbus = P["ocv"] - ibatt * P["r0"] - vp;
			}
			if (want_truth && k == 0 && (iter % FLIGHTREC_DIV) == 0) {
				TruthRow tr;
				tr.t = t; tr.theta = theta; tr.omega = omega; tr.moment_true = M; tr.vbus = vbus;
				tr.batt_scale = batt_scale; tr.rs = g_rig.rs;
				for (int i = 0; i < 4; i++) { tr.applied[i] = applied[i]; tr.thrust[i] = thrust[i]; }
				truth.push_back(tr);
			}
		}
		t += dt;
		iter++;
		if (done_t >= 0 && t - done_t >= P["tail_ms"] * 1e-3) break;
	}
	end_t = t;

	// ---- status ------------------------------------------------------------------------------------
	if (!have_armed) status = "never_armed";
	else if (estop) status = std::string("estop:") + (estop_reason ? estop_reason : "?");
	else if (done_t < 0) status = "running";
	else if (rd_reason.empty() || rd_reason == "RIG stop" || rd_reason == "RIG max run time") status = "ok";
	else status = "rampdown:" + rd_reason;

	// ---- step metrics (first att step, rig_analyze.py's definitions) ------------------------------------
	double overshoot = NAN, settle_ms = NAN, rise_ms = NAN, quiet_mean = NAN, quiet_osc = NAN, tail_osc = NAN;
	bool settled = false;
	// mean of the roll error over [t0, t1), and sqrt(2) * the std of its residual about a straight
	// line (a slow drift, e.g. an integrator still converging, is not an oscillation)
	auto window_stats = [&](double t0, double t1, double *mean, double *osc) {
		double st = 0, se = 0, stt = 0, ste = 0; int n = 0;
		for (size_t i = 0; i < rt.size(); i++) {
			if (rt[i] < t0 || rt[i] >= t1) continue;
			const double x = rt[i] - t0;
			st += x; se += rerr[i]; stt += x * x; ste += x * rerr[i]; n++;
		}
		if (n < 10) return;
		*mean = se / n;
		const double den = n * stt - st * st;
		const double b = den > 0 ? (n * ste - st * se) / den : 0.0, a = (se - b * st) / n;
		double r2 = 0;
		for (size_t i = 0; i < rt.size(); i++) {
			if (rt[i] < t0 || rt[i] >= t1) continue;
			const double r = rerr[i] - (a + b * (rt[i] - t0));
			r2 += r * r;
		}
		*osc = std::sqrt(2.0) * std::sqrt(r2 / n);
	};
	const double qwin = P["quiet_ms"] * 1e-3;
	if (first_exc_t > 0) window_stats(first_exc_t - qwin, first_exc_t, &quiet_mean, &quiet_osc);
	else if (!rt.empty()) window_stats(rt.back() - qwin, rt.back() + 1, &quiet_mean, &quiet_osc);
	if (!rt.empty()) { double dummy; window_stats(rt.back() - qwin, rt.back() + 1, &dummy, &tail_osc); }
	if (first_exc_t > 0 && first_exc == "step att") {
		// the step's leading edge: target jumps at first_exc_t; window until the target jumps back
		size_t i0 = 0;
		while (i0 < rt.size() && rt[i0] < first_exc_t) i0++;
		if (i0 > 0 && i0 + 10 < rt.size()) {
			const double delta = rtgt[i0] - rtgt[i0 - 1];
			size_t i1 = i0 + 1;
			while (i1 < rt.size() && std::fabs(rtgt[i1] - rtgt[i1 - 1]) < 0.4 * std::fabs(delta)) i1++;
			if (std::fabs(delta) > 1e-3 && i1 - i0 > 20) {
				const double y0 = rroll[i0 - 1];
				const size_t n = i1 - i0, k20 = i0 + (size_t)(0.8 * n);
				double yss = 0; for (size_t i = k20; i < i1; i++) yss += rroll[i]; yss /= (double)(i1 - k20);
				const double sgn = delta > 0 ? 1 : -1;
				double mx = 0; for (size_t i = i0; i < i1; i++) mx = std::max(mx, sgn * (rroll[i] - yss));
				const double span = std::fabs(yss - y0);
				overshoot = span > 1e-6 ? 100.0 * mx / span : NAN;
				double t10 = NAN, t90 = NAN;
				for (size_t i = i0; i < i1; i++) {
					const double fr = (rroll[i] - y0) / (yss - y0);
					if (std::isnan(t10) && fr >= 0.1) t10 = rt[i];
					if (std::isnan(t90) && fr >= 0.9) { t90 = rt[i]; break; }
				}
				rise_ms = (t90 - t10) * 1000.0;
				const double band = std::max(0.05 * std::fabs(delta), 1.0);
				size_t last_out = i0; bool any = false;
				for (size_t i = i0; i < i1; i++) if (std::fabs(rroll[i] - yss) > band) { last_out = i; any = true; }
				settled = !any || last_out + 1 < i1;
				settle_ms = !any ? 0.0 : (settled ? (rt[last_out + 1] - rt[i0]) * 1000.0 : NAN);
			}
		}
	}

	// ---- CSV (flightrec_fetch.py _write_rig_csv, byte for byte in format) -------------------------------
	if (!P.csv.empty()) {
		FILE *f = std::fopen(P.csv.c_str(), "w");
		if (!f) { *json = "{\"error\":" + jstr("cannot write " + P.csv) + "}"; return 2; }
		std::fprintf(f, "t_s,t_us,dt_us,seq,flags,armed,estop,arming,caldone,tof_valid,flow_valid,committed,"
			     "rig_state,rs_name,exc_id,gain_seq,gyro_p,gyro_q,gyro_r,roll_deg,pitch_deg,roll_tgt_deg,"
			     "rate_tgt_dps,rate_cmd,moment_mNm,exc_val,coll_thrust_N,att_int,rate_int,pitch_cmd,yaw_cmd,"
			     "accel_y,accel_z,vbat,duty0,duty1,duty2,duty3,axis,ang_deg,ang_tgt_deg,ang_rate_dps\r\n");
		uint32_t ref = rows.empty() ? 0 : rows[0].t_us;
		for (const Row &r : rows) {
			if (r.duty[0] || r.duty[1] || r.duty[2] || r.duty[3]) { ref = r.t_us; break; }
		}
		static const char *names[] = {"IDLE", "ARMING", "RAMP_UP", "RUN", "RAMP_DOWN", "DONE"};
		const int bits[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40};
		auto deg = [](float x) { return (double)x * 180.0 / PI; };
		for (const Row &r : rows) {
			double ts = (double)((uint32_t)(r.t_us - ref)) / 1e6;
			if (ts > 2147.483648) ts -= 4294.967296;
			std::fprintf(f, "%.6f,%u,%u,%u,%u", ts, r.t_us, r.dt_us, r.seq, r.flags);
			for (int b : bits) std::fprintf(f, ",%d", (r.flags & b) ? 1 : 0);
			const char *rn = (r.flags & 0x02) ? "ESTOP" : (r.rs <= 5 ? names[r.rs] : "?");
			std::fprintf(f, ",%u,%s,%u,%u", r.rs, rn, r.exc_id, r.gseq);
			std::fprintf(f, ",%.5f,%.5f,%.5f", r.gyro[0], r.gyro[1], r.gyro[2]);
			std::fprintf(f, ",%.4f,%.4f,%.4f,%.4f", deg(r.roll), deg(r.pitch), deg(r.roll_tgt), deg(r.rate_tgt));
			std::fprintf(f, ",%.6g,%.6g,%.6g,%.5f", r.rate_cmd, r.moment * 1e3, r.exc_val, r.coll);
			std::fprintf(f, ",%.6g,%.6g,%.6g,%.6g", r.att_int, r.rate_int, r.pitch_cmd, r.yaw_cmd);
			std::fprintf(f, ",%.4f,%.4f,%.3f", r.ay, r.az, r.vbat);
			for (int i = 0; i < 4; i++) std::fprintf(f, ",%.4f", r.duty[i] / 10000.0);
			std::fprintf(f, ",%s,%.4f,%.4f,%.4f", axis ? "P" : "R", deg(axis ? r.pitch : r.roll),
				     deg(r.roll_tgt), deg(r.gyro[axis ? 1 : 0]));
			std::fprintf(f, "\r\n");
		}
		std::fclose(f);
	}
	if (want_truth) {
		FILE *f = std::fopen(P.truth.c_str(), "w");
		if (f) {
			std::fprintf(f, "t,theta_deg,omega,moment_true_mNm,vbus,batt_scale,rig_state,"
				     "applied0,applied1,applied2,applied3,thrust0,thrust1,thrust2,thrust3\n");
			for (const TruthRow &r : truth) {
				std::fprintf(f, "%.5f,%.4f,%.5f,%.6g,%.4f,%.4f,%d", r.t, r.theta * 180 / PI, r.omega,
					     r.moment_true * 1e3, r.vbus, r.batt_scale, r.rs);
				for (int i = 0; i < 4; i++) std::fprintf(f, ",%.4f", r.applied[i]);
				for (int i = 0; i < 4; i++) std::fprintf(f, ",%.6g", r.thrust[i]);
				std::fprintf(f, "\n");
			}
			std::fclose(f);
		}
	}

	// ---- JSON summary ----------------------------------------------------------------------------------
	std::ostringstream o;
	o << "{";
	if (!P.id.empty()) o << "\"id\":" << jstr(P.id) << ",";
	o << "\"status\":" << jstr(status)
	  << ",\"axis\":" << jstr(axis ? "pitch" : "roll")
	  << ",\"t_end_s\":" << jnum(end_t)
	  << ",\"arm_s\":" << (anchor_set[A_ARM] ? jnum(anchor_t[A_ARM]) : "null")
	  << ",\"run_s\":" << (run_enter_t >= 0 ? jnum(run_enter_t) : "null")
	  << ",\"liftoff_ms\":" << (liftoff_ms >= 0 ? jnum(liftoff_ms) : "null")
	  << ",\"liftoff_att_int\":" << jnum(liftoff_att_int)
	  << ",\"att_int_peak\":" << jnum(att_int_peak)
	  << ",\"guard_margin_live_deg\":" << (max_est_live >= 0 ? jnum(RIG_MAX_TILT_RAD * 180.0 / PI - max_est_live) : "null")
	  << ",\"settle_level_ms\":" << (settle_seen ? jnum(last_off_level_t < 0 ? 0.0 : (last_off_level_t - liftoff_t) * 1000.0) : "null")
	  << ",\"max_live_deg\":" << jnum(max_live_deg)
	  << ",\"roll_max_live_deg\":" << (roll_hi_live > -1e8 ? jnum(roll_hi_live) : "null")
	  << ",\"roll_min_live_deg\":" << (roll_lo_live < 1e8 ? jnum(roll_lo_live) : "null")
	  << ",\"guard_margin_deg\":" << (near_level ? jnum(RIG_MAX_TILT_RAD * 180.0 / PI - max_est_after_liftoff) : "null")
	  << ",\"stop_hits_live\":" << stop_hits_live
	  << ",\"stop_hits_total\":" << stop_hits_total
	  << ",\"max_impact_radps\":" << jnum(max_impact)
	  << ",\"sat_frac\":" << jnum(run_ticks ? (double)sat_ticks / run_ticks : NAN)
	  << ",\"peak_duty\":" << jnum(peak_duty)
	  << ",\"rms_err_deg\":" << jnum(run_ticks ? std::sqrt(sum_err2 / run_ticks) : NAN)
	  << ",\"steady_err_deg\":" << jnum(quiet_mean)
	  << ",\"quiet_osc_deg\":" << jnum(quiet_osc)
	  << ",\"tail_osc_deg\":" << jnum(tail_osc)
	  << ",\"first_exc\":" << jstr(first_exc)
	  << ",\"overshoot_pct\":" << jnum(overshoot)
	  << ",\"rise_ms\":" << jnum(rise_ms)
	  << ",\"settle_ms\":" << jnum(settle_ms)
	  << ",\"settled\":" << (std::isnan(overshoot) ? "null" : (settled ? "true" : "false"))
	  << ",\"acks\":" << acks << ",\"naks\":" << naks
	  << ",\"vbat_end\":" << jnum(g_vbat)
	  << ",\"I_total\":" << jnum(Itot)
	  << ",\"nak_lines\":[";
	for (size_t i = 0; i < nak_lines.size() && i < 20; i++) o << (i ? "," : "") << jstr(nak_lines[i]);
	o << "],\"events\":[";
	for (size_t i = 0; i < events.size() && i < 40; i++) o << (i ? "," : "") << jstr(events[i]);
	o << "]}";
	*json = o.str();
	return 0;
}

int main(int argc, char **argv)
{
	Params base;
	std::string batch, err;
	for (int i = 1; i < argc; i++) {
		const std::string a = argv[i];
		if (a == "help" || a == "--help" || a == "-h") {
			std::printf("rig_twin key=value ... (see the header of rig_twin.cpp)\n");
			for (const auto &p : PARAMS) std::printf("  %-16s %-10g %s\n", p.key, p.def, p.help);
			return 0;
		}
		if (a.rfind("batch=", 0) == 0) { batch = a.substr(6); continue; }
		if (!base.set(a, &err)) { std::fprintf(stderr, "rig_twin: %s\n", err.c_str()); return 2; }
	}
	if (batch.empty()) {
		std::string js;
		const int rc = simulate(base, &js);
		std::printf("%s\n", js.c_str());
		return rc;
	}
	std::ifstream f(batch);
	if (!f) { std::fprintf(stderr, "rig_twin: cannot open %s\n", batch.c_str()); return 2; }
	std::string line;
	int rc = 0;
	while (std::getline(f, line)) {
		if (line.empty() || line[0] == '#') continue;
		Params p = base;
		std::istringstream is(line);
		std::string kv;
		bool ok = true;
		while (is >> kv) {
			if (!p.set(kv, &err)) { std::printf("{\"error\":%s}\n", jstr(err).c_str()); ok = false; rc = 2; break; }
		}
		if (!ok) continue;
		std::string js;
		if (simulate(p, &js) != 0) rc = 2;
		std::printf("%s\n", js.c_str());
		std::fflush(stdout);
	}
	return rc;
}
