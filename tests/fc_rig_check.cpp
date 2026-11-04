// Host test driver for workloads/flight_controller/src/fc_rig.cpp, built and run by test_fc_rig.py.
// `fc_rig_check <name>` runs one group and exits non-zero on any failed CHECK. Compiled together
// with fc_rig.cpp and controller_pid.cpp (ROSE_VEL_LOOP=0), with the same -D tuning macros, so the
// flown-PID comparisons see the build constants the rig defaults are derived from.
#include "fc_rig.h"
#include "controller_pid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

// controller_pid.cpp's fallbacks, verbatim, for the reference formula.
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
// fc_rig.cpp's fallbacks for the knobs the driver checks against.
#ifndef RIG_MAX_TILT_RAD
#define RIG_MAX_TILT_RAD 0.44f
#endif
#ifndef RIG_TGT_MAX_RAD
#define RIG_TGT_MAX_RAD (RIG_MAX_TILT_RAD - 0.09f)
#endif
static const float PID_FLOWN_SCALE = 2.0f;   // default build (no PID_TILT_TIGHT / PID_GIBBS_LEGACY)
static const float J0 = 16e-6f, L_ARM = 33e-3f, GRAV = 9.81f;

static int g_fail;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)
#define CHECK_NEAR(a, b, tol) do { double _a = (a), _b = (b); if (!(std::fabs(_a - _b) <= (double)(tol))) { \
	std::printf("FAIL %s:%d: %s=%.9g vs %s=%.9g (tol %g)\n", __FILE__, __LINE__, #a, _a, #b, _b, (double)(tol)); g_fail++; } } while (0)

static bool same_float(float a, float b) { return a == b; }

// ---------------------------------------------------------------------------------------------
static void t_parser()
{
	struct { const char *s; float v; } good[] = {
		{"0", 0.0f}, {"5", 5.0f}, {"-5", -5.0f}, {"+5", 5.0f}, {"40.5", 40.5f}, {"-0.0125", -0.0125f},
		{"0.1", 0.1f}, {"0.85", 0.85f}, {"123456789", 123456789.0f}, {"007", 7.0f},
		{"1.000000000001", 1.0f}, {"0.333333333333", 0.333333333333f}, {"-0", 0.0f},
	};
	for (auto &g : good) {
		float v = -999.0f;
		bool ok = fc_rig_parse_decimal(g.s, &v);
		if (!ok || !same_float(v, g.v)) {
			std::printf("FAIL parse_decimal(\"%s\") ok=%d v=%.9g want %.9g\n", g.s, ok, (double)v, (double)g.v);
			g_fail++;
		}
	}
	const char *bad[] = {"", "-", "+", ".5", "5.", "-.5", "1e3", "1E3", "1.5e-2", "inf", "nan", " 5", "5 ",
			     "1.2.3", "0x10", "1,5", "--5", "+-5", "1234567890", "1.0000000000001",
			     "12345.12345678901", "5a", "a5", "\t5"};
	for (const char *s : bad) {
		float v = 42.0f;
		if (fc_rig_parse_decimal(s, &v) || v != 42.0f) {
			std::printf("FAIL parse_decimal accepted or wrote on \"%s\"\n", s);
			g_fail++;
		}
	}
	CHECK(!fc_rig_parse_decimal(nullptr, nullptr));

	int32_t i = 0;
	CHECK(fc_rig_parse_int("0", &i) && i == 0);
	CHECK(fc_rig_parse_int("-20000", &i) && i == -20000);
	CHECK(fc_rig_parse_int("+7", &i) && i == 7);
	CHECK(fc_rig_parse_int("999999999", &i) && i == 999999999);
	const char *bad_int[] = {"", "-", "1.5", "12abc", "1e3", "1234567890", " 1", "1 ", "0x1"};
	for (const char *s : bad_int) {
		i = 42;
		if (fc_rig_parse_int(s, &i) || i != 42) {
			std::printf("FAIL parse_int accepted \"%s\"\n", s);
			g_fail++;
		}
	}
}

static void t_format()
{
	char buf[40];
	struct { float v; const char *s; } exact[] = {
		{40.0f, "40"}, {0.85f, "0.85"}, {0.87f, "0.87"}, {-0.0125f, "-0.0125"}, {0.0f, "0"},
		{5.0f, "5"}, {1.0f / 0.03f, "33.333336"}, {-0.0f, "0"}, {0.175f, "0.175"},
	};
	for (auto &e : exact) {
		fc_rig_format_decimal(e.v, buf, sizeof buf);
		if (std::strcmp(buf, e.s) != 0) {
			std::printf("FAIL format(%.9g) = \"%s\", want \"%s\"\n", (double)e.v, buf, e.s);
			g_fail++;
		}
	}
	// Round trip for every gain range: endpoints, defaults, and a deterministic sweep.
	fc_rig_gains d;
	fc_rig_gains_default(&d);
	uint32_t lcg = 12345u;
	for (int id = 0; id < FC_RIG_GAIN_COUNT; id++) {
		float lo, hi;
		fc_rig_gain_range(id, &lo, &hi);
		std::vector<float> vals = {lo, hi, d.v[id], 0.5f * (lo + hi)};
		for (int k = 0; k < 2000; k++) {
			lcg = lcg * 1664525u + 1013904223u;
			vals.push_back(lo + (hi - lo) * (float)(lcg >> 8) / 16777216.0f);
		}
		for (float v : vals) {
			if (v != 0.0f && std::fabs(v) < 1e-3f) {
				continue;   // documented: exact only for |v| >= 1e-3
			}
			float back = NAN;
			CHECK(fc_rig_format_decimal(v, buf, sizeof buf) > 0);
			if (!fc_rig_parse_decimal(buf, &back) || back != v) {
				std::printf("FAIL round trip %.9g -> \"%s\" -> %.9g\n", (double)v, buf, (double)back);
				g_fail++;
			}
		}
	}
	CHECK(fc_rig_format_decimal(33.3333f, buf, 3) == -1 && buf[0] == '\0');
}

static void t_gains()
{
	fc_rig_gains g;
	fc_rig_gains_default(&g);
	for (int id = 0; id < FC_RIG_GAIN_COUNT; id++) {
		float lo, hi;
		fc_rig_gain_range(id, &lo, &hi);
		CHECK(fc_rig_gain_name(id) != nullptr);
		CHECK(fc_rig_gain_find(fc_rig_gain_name(id)) == id);
		if (!(g.v[id] >= lo && g.v[id] <= hi)) {
			std::printf("FAIL default %s=%g outside [%g, %g]\n", fc_rig_gain_name(id), (double)g.v[id],
				    (double)lo, (double)hi);
			g_fail++;
		}
		// boundaries accepted, just outside refused, table unchanged on refusal
		const float before = g.v[id];
		CHECK(fc_rig_gain_set(&g, fc_rig_gain_name(id), std::nextafter(lo, -1e9f)) == -2);
		CHECK(fc_rig_gain_set(&g, fc_rig_gain_name(id), std::nextafter(hi, 1e9f)) == -2);
		CHECK(fc_rig_gain_set(&g, fc_rig_gain_name(id), NAN) == -2);
		CHECK(g.v[id] == before);
		CHECK(fc_rig_gain_set(&g, fc_rig_gain_name(id), lo) == id && g.v[id] == lo);
		CHECK(fc_rig_gain_set(&g, fc_rig_gain_name(id), hi) == id && g.v[id] == hi);
	}
	CHECK(fc_rig_gain_name(-1) == nullptr && fc_rig_gain_name(FC_RIG_GAIN_COUNT) == nullptr);
	CHECK(fc_rig_gain_set(&g, "FOO", 1.0f) == -1);
	CHECK(fc_rig_gain_set(&g, "att_p", 1.0f) == -1);     // case-sensitive
	CHECK(fc_rig_gain_set(&g, "ATT_P ", 1.0f) == -1);
	CHECK(fc_rig_gain_set(&g, "", 1.0f) == -1);

	// The defaults follow the build's macros (the second test variant sets them).
	fc_rig_gains_default(&g);
	CHECK_NEAR(g.v[FC_RIG_ATT_P], 1.0f / (PID_FLOWN_SCALE * (float)(TAU_ROLL)), 1e-6);
	CHECK_NEAR(g.v[FC_RIG_RATE_P], 1.0f / (float)(TAU_ROLLRATE), 1e-5);
	CHECK(g.v[FC_RIG_ATT_GAIN] == (float)(ATT_GAIN));
	CHECK(g.v[FC_RIG_MOTOR_TRIM_LEFT] == (float)(MOTOR_TRIM_LEFT));
	CHECK(g.v[FC_RIG_ATT_I] == 0.0f && g.v[FC_RIG_RATE_I] == 0.0f && g.v[FC_RIG_RATE_D] == 0.0f);
	CHECK(g.v[FC_RIG_GYRO_LPF_HZ] == 0.0f && g.v[FC_RIG_D_LPF_HZ] == 30.0f);
	std::printf("defaults ATT_P=%g RATE_P=%g ATT_GAIN=%g MOTOR_TRIM_LEFT=%g\n", (double)g.v[FC_RIG_ATT_P],
		    (double)g.v[FC_RIG_RATE_P], (double)g.v[FC_RIG_ATT_GAIN], (double)g.v[FC_RIG_MOTOR_TRIM_LEFT]);

	char line[FC_RIG_REPLY_MAX];
	const int n = fc_rig_gains_format(&g, 4294967295u, line, sizeof line);
	CHECK(n > 0 && n < (int)sizeof line && line[n - 1] == '\n');
	CHECK(std::strncmp(line, "RIGGAINS gs=4294967295 ", 23) == 0);
	for (int id = 0; id < FC_RIG_GAIN_COUNT; id++) {
		std::string key = std::string(" ") + fc_rig_gain_name(id) + "=";
		CHECK(std::strstr(line, key.c_str()) != nullptr);
	}
	// worst case: every gain at its longest printable value still fits
	for (int id = 0; id < FC_RIG_GAIN_COUNT; id++) {
		float lo, hi;
		fc_rig_gain_range(id, &lo, &hi);
		g.v[id] = lo < 0.0f ? std::nextafter(lo, 0.0f) : std::nextafter(hi, 0.0f);
	}
	CHECK(fc_rig_gains_format(&g, 4294967295u, line, sizeof line) > 0);
	std::printf("worst-case GAINS line %zu B\n", std::strlen(line));
	CHECK(fc_rig_gains_format(&g, 1u, line, 20) == -1 && line[0] == '\0');
}

// ---------------------------------------------------------------------------------------------
// Commands
struct Cmd {
	fc_rig_state s;
	char reply[FC_RIG_REPLY_MAX];
	bool handled = false;
	Cmd() { fc_rig_init(&s); }
	bool send(const char *line, int64_t now = 5000)
	{
		std::memset(reply, 'x', sizeof reply);
		handled = fc_rig_handle_command(&s, line, now, reply, sizeof reply);
		return handled;
	}
	bool acked(const char *verb) const { return handled && std::string(reply) == std::string("FCACK ") + verb + "\n"; }
	bool naked(const char *verb) const { return handled && std::string(reply) == std::string("FCNAK ") + verb + "\n"; }
};

static void t_commands()
{
	Cmd c;
	// not ours -> main.cpp's verbs
	for (const char *l : {"ESTOP", "RESET", "DISARM", "PING", "HOVER_Z 500", "GAINSX", "HBX", "STOPX",
			      "EXCSTOPX", "coll 500", "", "   "}) {
		if (c.send(l) || c.reply[0] != '\0') {
			std::printf("FAIL \"%s\" was claimed\n", l);
			g_fail++;
		}
	}
	// HB: no reply
	CHECK(c.send("HB", 7000) && c.reply[0] == '\0' && c.s.hb_seen && c.s.last_hb_ms == 7000);
	CHECK(c.send("HB 1") && c.naked("HB"));
	// COLL per-mille; out of range is NAKed (review: "COLL 999999999" was ACKed as 1.3x hover)
	CHECK(c.send("COLL 500") && c.acked("COLL") && std::fabs(c.s.coll_cmd - 0.5f) < 1e-6f);
	CHECK(c.send("COLL 1300") && c.acked("COLL") && std::fabs(c.s.coll_cmd - 1.3f) < 1e-6f);
	CHECK(c.send("COLL 0") && c.acked("COLL") && c.s.coll_cmd == 0.0f);
	for (const char *l : {"COLL 1301", "COLL 2000", "COLL -5", "COLL -1", "COLL 999999999", "COLL -999999999"}) {
		if (!(c.send(l) && c.naked("COLL") && c.s.nak_reason != nullptr && c.s.coll_cmd == 0.0f)) {
			std::printf("FAIL \"%s\" -> \"%s\" coll_cmd=%g\n", l, c.reply, (double)c.s.coll_cmd);
			g_fail++;
		}
	}
	CHECK(c.send("COLL 1000 ") && c.acked("COLL") && c.s.coll_cmd == 1.0f);   // trailing blank
	for (const char *l : {"COLL", "COLL 5abc", "COLL 0.5", "COLL 1 2", "COLL x"}) {
		CHECK(c.send(l) && c.naked("COLL"));
	}
	CHECK(c.s.coll_cmd == 1.0f);   // NAKs changed nothing
	// RSP millideg; out of range is NAKed, not clamped
	CHECK(c.send("RSP 5000") && c.acked("RSP") && std::fabs(c.s.sp_roll_cmd - 0.0872665f) < 1e-6f);
	for (const char *l : {"RSP 20001", "RSP -20001", "RSP 99999", "RSP -99999", "RSP 999999999"}) {
		if (!(c.send(l) && c.naked("RSP") && c.s.nak_reason != nullptr &&
		      std::fabs(c.s.sp_roll_cmd - 0.0872665f) < 1e-6f)) {
			std::printf("FAIL \"%s\" -> \"%s\"\n", l, c.reply);
			g_fail++;
		}
	}
	if (RIG_TGT_MAX_RAD >= 0.3491f) {        // the default bound admits the full +-20 deg
		CHECK(c.send("RSP 20000") && c.acked("RSP") && std::fabs(c.s.sp_roll_cmd - 0.3490659f) < 1e-6f);
		CHECK(c.send("RSP -20000") && c.acked("RSP") && std::fabs(c.s.sp_roll_cmd + 0.3490659f) < 1e-6f);
	}
	CHECK(c.send("RSP 5000") && c.acked("RSP"));
	CHECK(c.send("RSP 1.5") && c.naked("RSP"));
	// RUNMS clamped to [1000, 60000]
	CHECK(c.send("RUNMS 20000") && c.acked("RUNMS") && c.s.max_run_ms == 20000);
	CHECK(c.send("RUNMS 10") && c.s.max_run_ms == 1000);
	CHECK(c.send("RUNMS 999999") && c.s.max_run_ms == 60000);
	CHECK(c.send("RUNMS") && c.naked("RUNMS"));
	// GAIN: whitelist, range, echo, seq; bumpless: ATT_P leaves the integrators alone (t_bumpless)
	c.s.law.att_int = 0.3f;
	c.s.law.rate_int = 4.0f;
	CHECK(c.send("GAIN ATT_P 6.5") && std::string(c.reply) == "RIGGAIN ATT_P=6.5\nFCACK GAIN\n");
	CHECK(c.s.gains.v[FC_RIG_ATT_P] == 6.5f && c.s.gain_seq == 1 && c.s.last_gain == FC_RIG_ATT_P);
	CHECK(c.s.law.att_int == 0.3f && c.s.law.rate_int == 4.0f);
	CHECK(c.send("GAIN MOTOR_TRIM_LEFT 0.87") && std::string(c.reply) == "RIGGAIN MOTOR_TRIM_LEFT=0.87\nFCACK GAIN\n");
	CHECK(c.s.gain_seq == 2);
	for (const char *l : {"GAIN FOO 1", "GAIN ATT_P 1e3", "GAIN ATT_P 41", "GAIN ATT_P -1", "GAIN ATT_P",
			      "GAIN", "GAIN ATT_P 1 2", "GAIN att_p 5", "GAIN RATE_P 0", "GAIN ATT_P .5"}) {
		if (!(c.send(l) && c.naked("GAIN"))) {
			std::printf("FAIL \"%s\" -> \"%s\"\n", l, c.reply);
			g_fail++;
		}
		CHECK(c.s.nak_reason != nullptr);
	}
	CHECK(c.s.gain_seq == 2 && c.s.gains.v[FC_RIG_ATT_P] == 6.5f);
	// GAINS: the line then the ACK; "GAINS" is not "GAIN"
	CHECK(c.send("GAINS"));
	std::string r = c.reply;
	CHECK(r.rfind("RIGGAINS gs=2 ATT_P=6.5 ", 0) == 0);
	CHECK(r.size() > 12 && r.substr(r.size() - 12) == "FCACK GAINS\n");
	CHECK(std::count(r.begin(), r.end(), '\n') == 2);
	CHECK(c.send("GAINS 1") && c.naked("GAINS"));
	// EXC refused outside RUN (state is IDLE here); EXCSTOP and STOP always ACK
	CHECK(c.send("EXC step att 100 1000") && c.naked("EXC"));
	CHECK(std::strcmp(c.s.nak_reason, "EXC: only in RUN") == 0);
	CHECK(c.s.exc_id == 0);
	CHECK(c.send("EXCSTOP") && c.acked("EXCSTOP"));
	CHECK(c.send("STOP") && c.acked("STOP") && c.s.stop_req);
	CHECK(c.send("STOP now") && c.naked("STOP"));
	// over-long line: ours -> NAK, not truncated-and-obeyed
	std::string longline = "COLL " + std::string(120, '1');
	CHECK(c.send(longline.c_str()) && c.naked("COLL"));
	CHECK(c.send("EXC a b c d e f g h i") && c.naked("EXC"));
	// a small reply buffer is never overrun
	char small[8];
	std::memset(small, 'x', sizeof small);
	CHECK(fc_rig_handle_command(&c.s, "GAINS", 0, small, sizeof small));
	CHECK(std::memchr(small, '\0', sizeof small) != nullptr);
}

// ---------------------------------------------------------------------------------------------
// State machine
struct Sim {
	fc_rig_state s;
	fc_rig_inputs in;
	fc_rig_outputs out;
	int64_t t = 10000;          // ms; well clear of the dwell helpers' 0 sentinel
	bool hb = true;             // host sends HB at 20 Hz
	int64_t last_hb = 0;
	std::vector<std::string> events;
	bool estop_seen = false;
	const char *estop_reason = nullptr;
	char reply[FC_RIG_REPLY_MAX];

	Sim()
	{
		fc_rig_init(&s);
		std::memset(&in, 0, sizeof in);
		in.arm_enabled = true;
		in.gyro_cal_done = true;
		in.batt_ok = true;
		in.dt = 0.001f;
	}
	bool cmd(const char *l) { return fc_rig_handle_command(&s, l, t, reply, sizeof reply); }
	void tick()
	{
		t += 1;
		if (hb && t - last_hb >= 50) {
			cmd("HB");
			last_hb = t;
		}
		in.now_ms = t;
		fc_rig_step(&s, &in, &out);
		if (out.event) events.push_back(out.event);
		if (out.estop_req) {
			estop_seen = true;
			estop_reason = out.estop_reason;
			in.estop = true;       // main.cpp latches g_estop
		}
	}
	void run(int ms) { for (int i = 0; i < ms; i++) tick(); }
	// ticks until the state is rs; returns the ms it took, -1 if not within max
	int until(int rs, int max_ms)
	{
		for (int i = 1; i <= max_ms; i++) {
			tick();
			if (out.rs == rs) return i;
		}
		return -1;
	}
	bool saw(const char *ev) const
	{
		for (auto &e : events) if (e == ev) return true;
		return false;
	}
	void reset()
	{
		in.reset_gen++;
		in.estop = false;          // main.cpp's soft reset clears g_estop
	}
	// arm and get to RUN at the current inputs
	void to_run()
	{
		cmd("COLL 1000");
		until(FC_RIG_RUN, 4000);
	}
};

static void t_sm_normal()
{
	Sim m;
	// no COLL yet -> nothing to ramp to -> never arms
	m.run(2000);
	CHECK(m.out.rs == FC_RIG_IDLE && !m.out.armed);
	m.cmd("COLL 1000");
	m.tick();
	CHECK(m.out.rs == FC_RIG_ARMING && m.out.arming && !m.out.armed);
	int gains_ticks = 0;
	int k = 0;
	for (; k < 1100 && m.out.rs != FC_RIG_RAMP_UP; k++) {
		m.tick();
		gains_ticks += m.out.send_gains;
	}
	CHECK(m.out.rs == FC_RIG_RAMP_UP && m.out.armed && gains_ticks == 1);
	CHECK(k >= 999 && k <= 1001);          // RIG_ARM_SETTLE_MS
	CHECK(m.out.int_zero && !m.out.integrate);
	CHECK(m.out.coll_frac == 0.0f);
	m.run(750);
	CHECK_NEAR(m.out.coll_frac, 0.5f, 0.01f);
	m.run(10);
	m.tick();
	CHECK(m.out.send_gains == false);
	const int to_run = m.until(FC_RIG_RUN, 1000);
	CHECK(to_run > 0 && to_run <= 742);
	CHECK(m.out.coll_frac == 1.0f && m.out.integrate && !m.out.int_zero);
	m.run(2000);
	CHECK(m.out.rs == FC_RIG_RUN && !m.estop_seen);
	m.cmd("STOP");
	m.tick();
	CHECK(m.out.rs == FC_RIG_RAMP_DOWN && m.out.armed);
	CHECK(m.out.rampdown_reason && std::strcmp(m.out.rampdown_reason, "RIG stop") == 0);
	CHECK(!m.out.integrate && !m.out.int_zero);   // frozen
	m.run(750);
	CHECK_NEAR(m.out.coll_frac, 0.5f, 0.01f);
	CHECK(m.until(FC_RIG_DONE, 800) > 0);
	CHECK(!m.out.armed && m.out.coll_frac == 0.0f && !m.estop_seen);
	m.run(3000);                            // stays down: RESET to go again
	CHECK(m.out.rs == FC_RIG_DONE);

	// arming refusals
	struct { const char *what; void (*set)(fc_rig_inputs &); } refusals[] = {
		{"moving", [](fc_rig_inputs &i) { i.gyro[0] = 0.5f; }},
		{"pitch", [](fc_rig_inputs &i) { i.pitch_rad = 0.2f; }},
		{"roll 40", [](fc_rig_inputs &i) { i.roll_rad = 0.70f; }},
		{"battery", [](fc_rig_inputs &i) { i.batt_ok = false; }},
		{"estop", [](fc_rig_inputs &i) { i.estop = true; }},
		{"no cal", [](fc_rig_inputs &i) { i.gyro_cal_done = false; }},
		{"not enabled", [](fc_rig_inputs &i) { i.arm_enabled = false; }},
	};
	for (auto &r : refusals) {
		Sim n;
		r.set(n.in);
		n.cmd("COLL 1000");
		n.run(3000);
		if (n.out.armed || n.out.rs != FC_RIG_IDLE) {
			std::printf("FAIL armed despite %s\n", r.what);
			g_fail++;
		}
	}
	Sim q;
	q.hb = false;                            // no heartbeat: no arm, and no estop while idle
	q.cmd("COLL 1000");
	q.run(5000);
	CHECK(q.out.rs == FC_RIG_IDLE && !q.estop_seen);
}

static void t_sm_stop_liftoff()
{
	Sim m;
	m.in.roll_rad = 0.5236f;                 // resting on the 30 deg stop
	m.cmd("COLL 1000");
	CHECK(m.until(FC_RIG_RAMP_UP, 1100) > 0);
	CHECK(std::fabs(m.out.sp_roll - 0.5236f) < 1e-6f);   // setpoint starts at the measured roll
	CHECK(!m.out.guard_active);
	m.run(1500);                             // whole ramp at 30 deg: guard must stay off
	CHECK(m.out.rs == FC_RIG_RUN && !m.estop_seen && !m.out.guard_active);
	CHECK(m.out.sp_roll == 0.0f);            // slewed 0.52 -> 0 at 0.5 rad/s within the ramp
	m.in.roll_rad = 0.40f;                   // inside 0.44 but not inside the 0.39 band
	m.run(50);
	CHECK(!m.out.guard_active);
	m.in.roll_rad = 0.30f;                   // lifted off
	m.tick();
	CHECK(m.out.guard_active && m.saw("RIG liftoff: roll guard live"));
	m.in.roll_rad = 0.50f;                   // falls back beyond 25 deg: estop, debounced
	m.run(100);
	CHECK(!m.estop_seen);
	m.run(60);
	CHECK(m.estop_seen && m.estop_reason && std::strcmp(m.estop_reason, "rig tilt") == 0);
	CHECK(m.out.rs == FC_RIG_DONE && !m.out.armed);
}

static void t_sm_liftoff_timeout()
{
	Sim m;
	m.in.roll_rad = -0.52f;
	m.cmd("COLL 1000");
	CHECK(m.until(FC_RIG_RAMP_UP, 1100) > 0);
	const int64_t arm_t = m.t;
	CHECK(m.until(FC_RIG_RAMP_DOWN, 4000) > 0);
	CHECK(m.t - arm_t >= 3500 && m.t - arm_t <= 3502);   // RAMP_MS + LIFTOFF_TIMEOUT_MS
	CHECK(std::strcmp(m.out.rampdown_reason, "RIG liftoff timeout") == 0);
	CHECK(m.until(FC_RIG_DONE, 1600) > 0 && !m.estop_seen);
}

static void t_sm_heartbeat()
{
	Sim m;
	m.to_run();
	CHECK(m.out.rs == FC_RIG_RUN);
	m.hb = false;
	const int64_t silent_from = m.last_hb;
	CHECK(m.until(FC_RIG_RAMP_DOWN, 400) > 0);
	CHECK(m.t - silent_from == 301);
	CHECK(std::strcmp(m.out.rampdown_reason, "RIG heartbeat stale") == 0 && !m.estop_seen);
	m.run(1000);
	CHECK(m.estop_seen && std::strcmp(m.estop_reason, "rig heartbeat") == 0);
	CHECK(m.out.rs == FC_RIG_DONE);

	// HB back within 300 ms: nothing happens
	Sim n;
	n.to_run();
	n.hb = false;
	n.run(250);
	n.hb = true;
	n.last_hb = 0;
	n.run(2000);
	CHECK(n.out.rs == FC_RIG_RUN && !n.estop_seen);
}

static void t_sm_pitch_guard()
{
	Sim m;
	m.to_run();
	m.in.pitch_rad = -0.2f;
	m.run(100);
	CHECK(!m.estop_seen);
	m.run(60);
	CHECK(m.estop_seen && std::strcmp(m.estop_reason, "rig pitch") == 0);
	// a short spike is ridden out
	Sim n;
	n.to_run();
	n.in.pitch_rad = 0.2f;
	n.run(50);
	n.in.pitch_rad = 0.0f;
	n.run(500);
	CHECK(!n.estop_seen && n.out.rs == FC_RIG_RUN);
}

static void t_sm_reset_rearm()
{
	Sim m;
	m.to_run();
	m.in.estop = true;                       // external ESTOP command
	m.tick();
	CHECK(m.out.rs == FC_RIG_DONE && !m.out.armed && m.out.coll_frac == 0.0f);
	m.run(3000);
	CHECK(m.out.rs == FC_RIG_DONE);
	m.reset();
	m.tick();
	CHECK(m.out.rs == FC_RIG_IDLE);
	// Review B1: the estop dropped the commanded collective, so RESET (which clears the FC and
	// ESP latches and re-enables arming) must NOT re-arm on the old COLL, even with HB live.
	m.run(3000);
	CHECK(m.out.rs == FC_RIG_IDLE && !m.out.armed && m.s.coll_cmd == 0.0f);
	CHECK(m.cmd("COLL 1000"));
	CHECK(m.until(FC_RIG_RAMP_UP, 1100) > 0);
	CHECK(m.until(FC_RIG_RUN, 1600) > 0);
	// Review B1: STOP -> RAMP_DOWN -> DONE, then RESET with HB live and no new COLL: stays IDLE.
	{
		Sim q;
		q.to_run();
		CHECK(q.cmd("STOP"));
		CHECK(q.until(FC_RIG_DONE, 3000) > 0);
		q.reset();
		q.run(3000);
		CHECK(q.out.rs == FC_RIG_IDLE && !q.out.armed);
	}
	// DISARM: straight down, no estop
	fc_rig_disarm(&m.s);
	m.tick();
	CHECK(m.out.rs == FC_RIG_DONE && !m.out.armed && !m.estop_seen);
	// STOP during ARMING cancels it
	Sim n;
	n.cmd("COLL 1000");
	n.run(500);
	CHECK(n.out.rs == FC_RIG_ARMING);
	n.cmd("STOP");
	n.run(2000);
	CHECK(n.out.rs == FC_RIG_DONE && !n.out.armed);
}

static void t_sm_max_run()
{
	Sim m;
	m.cmd("RUNMS 5000");
	m.cmd("COLL 1000");
	CHECK(m.until(FC_RIG_RAMP_UP, 1100) > 0);
	const int64_t arm_t = m.t;
	CHECK(m.until(FC_RIG_RAMP_DOWN, 6000) > 0);
	CHECK(m.t - arm_t == 5001);
	CHECK(std::strcmp(m.out.rampdown_reason, "RIG max run time") == 0);
	CHECK(m.until(FC_RIG_DONE, 1600) > 0 && !m.estop_seen);
}

static void t_sm_rampdown_on_stop()
{
	// After a normal STOP the drone sinks back onto a stop: that ends the ramp, not an estop.
	Sim m;
	m.to_run();
	CHECK(m.out.guard_active);
	m.cmd("STOP");
	m.run(500);
	CHECK(m.out.rs == FC_RIG_RAMP_DOWN);
	m.in.roll_rad = 0.52f;
	CHECK(m.until(FC_RIG_DONE, 300) > 0);
	CHECK(!m.estop_seen);
}

static void t_sm_slews()
{
	Sim m;
	m.to_run();
	m.cmd("RSP 10000");                      // 10 deg
	m.run(100);
	CHECK_NEAR(m.out.sp_roll, 0.05f, 0.001f);   // 0.5 rad/s
	m.run(1000);
	CHECK_NEAR(m.out.sp_roll, 0.1745329f, 1e-5f);
	m.cmd("COLL 1300");
	m.run(300);
	CHECK_NEAR(m.out.coll_frac, 1.15f, 0.002f);  // RIG_COLL_SLEW 0.5 /s
	m.run(400);
	CHECK_NEAR(m.out.coll_frac, 1.3f, 1e-6f);
}

// ---------------------------------------------------------------------------------------------
// Excitation
static fc_rig_exc make_exc(int type, float amp, int dur, float f0, float f1)
{
	fc_rig_exc e;
	std::memset(&e, 0, sizeof e);
	e.type = (uint8_t)type;
	e.target = FC_RIG_TGT_ATT;
	e.amp = amp;
	e.dur_ms = dur;
	e.f0_hz = f0;
	e.f1_hz = f1;
	e.prbs_reg = 0x1FF;
	return e;
}

static void t_exc_shapes()
{
	const int types[] = {FC_RIG_EXC_STEP, FC_RIG_EXC_DOUBLET, FC_RIG_EXC_CHIRP, FC_RIG_EXC_SINE, FC_RIG_EXC_PRBS};
	for (int type : types) {
		for (float amp : {0.2f, -0.15f}) {
			fc_rig_exc e = make_exc(type, amp, 10000, 0.5f, 12.0f);
			CHECK(fc_rig_exc_shape(&e, -1) == 0.0f);
			double area = 0.0;
			for (int t = 0; t < 10000; t++) {
				const float v = fc_rig_exc_shape(&e, t);
				if (std::fabs(v) > std::fabs(amp) * (1.0f + 1e-6f)) {
					std::printf("FAIL type %d |v(%d)|=%g > amp %g\n", type, t, (double)v, (double)amp);
					g_fail++;
					break;
				}
				area += v;
			}
			CHECK(fc_rig_exc_shape(&e, 10000) == 0.0f && fc_rig_exc_shape(&e, 20000) == 0.0f);
			if (type == FC_RIG_EXC_CHIRP || type == FC_RIG_EXC_SINE) {
				fc_rig_exc f = make_exc(type, amp, 10000, 0.5f, 12.0f);
				CHECK(fc_rig_exc_shape(&f, 0) == 0.0f);                       // starts at 0
				CHECK(std::fabs(fc_rig_exc_shape(&f, 9999)) < 1e-4f * std::fabs(amp));   // ends at 0
				CHECK(std::fabs(fc_rig_exc_shape(&f, 1)) < 1e-4f * std::fabs(amp));
			}
			if (type == FC_RIG_EXC_DOUBLET) {
				CHECK(fc_rig_exc_shape(&e, 0) == amp && fc_rig_exc_shape(&e, 4999) == amp);
				CHECK(fc_rig_exc_shape(&e, 5000) == -amp && fc_rig_exc_shape(&e, 9999) == -amp);
				CHECK(std::fabs(area) < 1e-6);
			}
			if (type == FC_RIG_EXC_STEP) {
				CHECK(fc_rig_exc_shape(&e, 0) == amp && fc_rig_exc_shape(&e, 9999) == amp);
			}
		}
	}

	// chirp: zero-crossing intervals shrink (up-sweep) / grow (down-sweep) monotonically
	for (int dir = 0; dir < 2; dir++) {
		const float f0 = dir == 0 ? 0.5f : 10.0f, f1 = dir == 0 ? 10.0f : 0.5f;
		fc_rig_exc e = make_exc(FC_RIG_EXC_CHIRP, 0.1f, 20000, f0, f1);
		std::vector<double> zc;
		float prev = fc_rig_exc_shape(&e, 1);
		for (int t = 2; t < 20000; t++) {
			const float v = fc_rig_exc_shape(&e, t);
			if ((prev < 0.0f && v >= 0.0f) || (prev > 0.0f && v <= 0.0f)) {
				zc.push_back(t - 1 + prev / (prev - v));   // linear interpolation
			}
			prev = v;
		}
		CHECK(zc.size() > 40);
		int bad = 0;
		for (size_t i = 2; i < zc.size(); i++) {
			const double a = zc[i - 1] - zc[i - 2], b = zc[i] - zc[i - 1];
			if (dir == 0 ? b > a + 0.2 : b < a - 0.2) bad++;
		}
		CHECK(bad == 0);
		// every half-period matches the log sweep's instantaneous frequency f0*(f1/f0)^(t/T)
		int off = 0;
		for (size_t i = 1; i < zc.size(); i++) {
			const double tm = 0.5 * (zc[i] + zc[i - 1]) * 1e-3;
			const double f = f0 * std::pow((double)f1 / f0, tm / 20.0);
			if (std::fabs((zc[i] - zc[i - 1]) - 500.0 / f) > 0.03 * 500.0 / f + 1.0) off++;
		}
		CHECK(off == 0);
	}

	// PRBS: deterministic, binary, bit-aligned, both levels, not stuck
	fc_rig_exc a = make_exc(FC_RIG_EXC_PRBS, 0.1f, 30000, 20.0f, 0.0f);   // 50 ms bits
	fc_rig_exc b = make_exc(FC_RIG_EXC_PRBS, 0.1f, 30000, 20.0f, 0.0f);
	std::vector<float> sa, sb;
	for (int t = 0; t < 30000; t++) sa.push_back(fc_rig_exc_shape(&a, t));
	for (int t = 0; t < 30000; t += 3) sb.push_back(fc_rig_exc_shape(&b, t));   // coarser ticks
	int pos = 0, changes = 0, misaligned = 0, mismatch = 0;
	for (int t = 0; t < 30000; t++) {
		CHECK(sa[t] == 0.1f || sa[t] == -0.1f);
		pos += sa[t] > 0.0f;
		if (t > 0 && sa[t] != sa[t - 1]) {
			changes++;
			if (t % 50 != 0) misaligned++;
		}
		if (t % 3 == 0 && sb[t / 3] != sa[t]) mismatch++;
	}
	CHECK(misaligned == 0 && mismatch == 0);
	CHECK(changes > 100 && pos > 12000 && pos < 18000);
	std::vector<float> first;                                  // the same EXC again -> same bits
	fc_rig_exc c2 = make_exc(FC_RIG_EXC_PRBS, 0.1f, 30000, 20.0f, 0.0f);
	for (int t = 0; t < 30000; t += 7) CHECK(fc_rig_exc_shape(&c2, t) == sa[t]);
	// period 511 bits (9-bit maximal LFSR), and not shorter
	fc_rig_exc p = make_exc(FC_RIG_EXC_PRBS, 1.0f, 60000, 100.0f, 0.0f);    // 10 ms bits
	std::vector<float> bits;
	for (int bit = 0; bit < 1100; bit++) bits.push_back(fc_rig_exc_shape(&p, bit * 10 + 5));
	int diff511 = 0, diff_short = 0;
	for (int i = 0; i + 511 < 1100; i++) diff511 += bits[i] != bits[i + 511];
	for (int i = 0; i + 255 < 1100; i++) diff_short += bits[i] != bits[i + 255];
	CHECK(diff511 == 0 && diff_short > 0);
	(void)first;
}

static void t_exc_commands()
{
	Sim m;
	m.cmd("COLL 1000");
	CHECK(m.until(FC_RIG_RAMP_UP, 1100) > 0);
	CHECK(m.cmd("EXC step att 100 1000") && std::string(m.reply) == "FCNAK EXC\n");   // RAMP_UP
	CHECK(m.until(FC_RIG_RUN, 1600) > 0);

	// caps: att 0.26 rad, rate 3 rad/s, moment 0.3 * 0.5*u0*l_arm
	CHECK(m.cmd("EXC step att 261 1000") && std::string(m.reply) == "FCNAK EXC\n");
	CHECK(m.cmd("EXC step rate 3001 1000") && std::string(m.reply) == "FCNAK EXC\n");
	const float u0 = (float)(PID_MASS_KG) * GRAV;
	CHECK_NEAR(fc_rig_hover_thrust(), u0, 1e-7);
	CHECK_NEAR(fc_rig_moment_authority(u0), 0.5f * u0 * L_ARM, 1e-9);
	const int cap_uNm = (int)(0.3f * 0.5f * u0 * L_ARM * 1e6f);
	char l[96];
	std::snprintf(l, sizeof l, "EXC doublet moment %d 1000", cap_uNm + 2);
	CHECK(m.cmd(l) && std::string(m.reply) == "FCNAK EXC\n");
	for (const char *bad : {"EXC wobble att 100 1000", "EXC step yaw 100 1000", "EXC step att 0 1000",
				"EXC step att 100 0", "EXC step att 100 70000", "EXC chirp att 100 1000 500",
				"EXC sine att 100 1000", "EXC sine att 100 1000 5", "EXC sine att 100 1000 60000",
				"EXC step att 1.5 1000", "EXC step att", "EXC STEP att 100 1000"}) {
		if (!(m.cmd(bad) && std::string(m.reply) == "FCNAK EXC\n")) {
			std::printf("FAIL \"%s\" -> \"%s\"\n", bad, m.reply);
			g_fail++;
		}
	}
	CHECK(m.s.exc_id == 0);

	CHECK(m.cmd("EXC step att 260 1000"));
	CHECK(std::string(m.reply) == "RIGEXC id=1 step att 260 1000\nFCACK EXC\n");
	m.tick();
	CHECK(m.out.exc_target == FC_RIG_TGT_ATT && std::fabs(m.out.exc_val - 0.26f) < 1e-6f);
	m.run(1100);
	CHECK(m.out.exc_val == 0.0f && m.s.exc.type == FC_RIG_EXC_NONE);   // ended by itself

	std::snprintf(l, sizeof l, "EXC chirp moment %d 8000 1000 20000", cap_uNm - 1);
	CHECK(m.cmd(l) && m.s.exc_id == 2);
	m.run(2000);
	CHECK(m.out.exc_target == FC_RIG_TGT_MOMENT);
	// collective drops: the live cap follows it
	m.cmd("COLL 300");
	float worst = 0.0f;
	bool over = false;
	for (int i = 0; i < 3000; i++) {
		m.tick();
		const float cap = 0.3f * fc_rig_moment_authority(m.out.coll_frac * u0);
		if (std::fabs(m.out.exc_val) > cap * (1.0f + 1e-6f)) over = true;
		if (std::fabs(m.out.exc_val) > worst) worst = std::fabs(m.out.exc_val);
	}
	CHECK(!over && worst > 0.0f);
	CHECK(m.cmd("EXCSTOP") && std::string(m.reply) == "FCACK EXCSTOP\n");
	m.tick();
	CHECK(m.out.exc_val == 0.0f);

	// leaving RUN stops it (500 mrad/s: inside the collective-scaled rate cap at COLL 300)
	CHECK(m.cmd("EXC sine rate 500 5000 2000") && std::string(m.reply) == "RIGEXC id=3 sine rate 500 5000 2000\nFCACK EXC\n");
	m.run(10);
	m.cmd("STOP");
	m.tick();
	CHECK(m.out.rs == FC_RIG_RAMP_DOWN && m.out.exc_val == 0.0f && m.s.exc.type == FC_RIG_EXC_NONE);
	CHECK(m.cmd("EXC step att 100 1000") && std::string(m.reply) == "FCNAK EXC\n");
}

// ---------------------------------------------------------------------------------------------
// Control law
static fc_rig_law_in law_in(float roll, float gx)
{
	fc_rig_law_in in;
	std::memset(&in, 0, sizeof in);
	in.roll = roll;
	in.gyro[0] = gx;
	in.dt = 0.0012f;
	in.coll_thrust = (float)(PID_MASS_KG) * GRAV;
	return in;
}

// controller_pid.cpp's roll path, transcribed (ROSE_VEL_LOOP=0: desRoll = 0, trim 0).
static float flown_roll_moment(float estRoll, float gyroX, float roll_tgt)
{
	const float rollRate_tgt = (-1.0f / (PID_FLOWN_SCALE * (float)(TAU_ROLL))) * (estRoll - roll_tgt);
	const float rollRate_cmd = (-1.0f / (float)(TAU_ROLLRATE)) * (gyroX - rollRate_tgt);
	return rollRate_cmd * J0 * (float)(ATT_GAIN);
}

static void t_law_matches_flown()
{
	fc_rig_gains g;
	fc_rig_gains_default(&g);
	uint32_t lcg = 777u;
	int worst_i = -1;
	double worst = 0.0;
	for (int i = 0; i < 2000; i++) {
		lcg = lcg * 1664525u + 1013904223u;
		const float roll = -0.4f + 0.8f * (float)(lcg >> 8) / 16777216.0f;
		lcg = lcg * 1664525u + 1013904223u;
		const float gx = -2.0f + 4.0f * (float)(lcg >> 8) / 16777216.0f;
		fc_rig_law_state ls;
		fc_rig_law_reset(&ls);
		fc_rig_law_in in = law_in(roll, gx);
		in.pitch = 0.05f; in.gyro[1] = 0.3f; in.gyro[2] = -0.2f; in.yaw_err = 0.3f;   // ignored axes
		in.integrate = true;                                                  // I = 0 by default
		float u[4];
		fc_rig_dbg dbg;
		fc_rig_roll_law(&ls, &g, &in, u, &dbg);
		const float ref = flown_roll_moment(roll, gx, 0.0f);
		const double rel = std::fabs((double)u[1] - ref) / std::fmax(std::fabs((double)ref), 1e-9);
		if (rel > worst) { worst = rel; worst_i = i; }
		CHECK(u[0] == in.coll_thrust && u[2] == 0.0f && u[3] == 0.0f);
		CHECK(dbg.pitch_cmd != 0.0f && dbg.yaw_cmd != 0.0f);   // computed, not applied
		CHECK(dbg.moment == u[1] && dbg.exc_val == 0.0f);
	}
	std::printf("worst relative error vs flown formula %.3g (sample %d)\n", worst, worst_i);
	CHECK(worst < 1e-5);

	// sign: a positive roll error (target above the estimate) gives a positive moment, both laws
	fc_rig_law_state ls;
	fc_rig_law_reset(&ls);
	fc_rig_law_in in = law_in(-0.1f, 0.0f);
	float u[4];
	fc_rig_dbg dbg;
	fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK(u[1] > 0.0f && flown_roll_moment(-0.1f, 0.0f, 0.0f) > 0.0f);
	// and a positive roll rate is damped
	fc_rig_law_reset(&ls);
	in = law_in(0.0f, 1.0f);
	fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK(u[1] < 0.0f && flown_roll_moment(0.0f, 1.0f, 0.0f) < 0.0f);
	// the setpoint is a target like the flown roll_tgt
	fc_rig_law_reset(&ls);
	in = law_in(0.05f, 0.3f);
	in.sp_roll = 0.12f;
	fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK_NEAR(u[1], flown_roll_moment(0.05f, 0.3f, 0.12f), 1e-5 * std::fabs(flown_roll_moment(0.05f, 0.3f, 0.12f)));
}

// End to end through the real controller_pid.cpp: run HierarchicalPidController::compute at the
// same state, invert its duties back through forceToVoltage and the mixer, and recover u[1].
static void t_law_matches_controller_pid()
{
	const float A = 26.919633f, B = 35.754861f, PROP = 1.33f, GPN = 101.9368f;
	auto force = [&](float duty) { return (A * duty * duty + B * duty) * PROP / (GPN * 4.0f); };
	fc_rig_gains g;
	fc_rig_gains_default(&g);
	HierarchicalPidController pid;
	int checked = 0;
	for (float roll : {-0.3f, -0.1f, 0.0f, 0.07f, 0.25f}) {
		for (float gx : {-1.0f, -0.2f, 0.0f, 0.4f, 1.5f}) {
			float state[12] = {0}, sp[12] = {0}, uo[4];
			state[3] = std::tan(0.5f * roll);   // Gibbs, pure roll
			state[9] = gx;
			pid.init();
			pid.compute(state, sp, uo, 0.0012f);
			float duty[4];
			bool clipped = false;
			for (int i = 0; i < 4; i++) {
				duty[i] = uo[i] + 0.583f;
				clipped |= duty[i] <= 0.0f || duty[i] >= 1.0f;
			}
			if (clipped) continue;
			const float trim = (float)(MOTOR_TRIM_LEFT);
			const float c0 = force(duty[3]) / (0.9f * trim), c1 = force(duty[0]) / 0.9f;
			const float c2 = force(duty[1]) / 0.9f, c3 = force(duty[2]) / (0.9f * trim);
			const float u1_flown = L_ARM * (c0 - c1 - c2 + c3);
			const float u0_flown = c0 + c1 + c2 + c3;

			fc_rig_law_state ls;
			fc_rig_law_reset(&ls);
			fc_rig_law_in in = law_in(roll, gx);
			in.coll_thrust = (float)(PID_MASS_KG) * GRAV / std::cos(roll);   // the flown tilt-compensated u0
			float u[4];
			fc_rig_roll_law(&ls, &g, &in, u, nullptr);
			CHECK_NEAR(u[0], u0_flown, 1e-4 * u0_flown);
			CHECK_NEAR(u[1], u1_flown, 2e-3 * std::fabs(u1_flown) + 2e-9);
			checked++;
		}
	}
	std::printf("controller_pid.cpp end-to-end points checked: %d\n", checked);
	CHECK(checked >= 20);
}

static void t_law_integrators()
{
	fc_rig_gains g;
	fc_rig_gains_default(&g);
	CHECK(fc_rig_gain_set(&g, "ATT_I", 50.0f) >= 0);
	CHECK(fc_rig_gain_set(&g, "RATE_I", 500.0f) >= 0);
	fc_rig_law_state ls;
	fc_rig_law_reset(&ls);
	fc_rig_law_in in = law_in(-0.3f, 0.0f);
	float u[4];
	fc_rig_dbg dbg;
	// held at zero (RAMP_UP)
	in.int_zero = true;
	in.integrate = true;
	for (int i = 0; i < 100; i++) fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK(ls.att_int == 0.0f && ls.rate_int == 0.0f);
	// running: grows, then clamps at the limits
	in.int_zero = false;
	fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK(ls.att_int > 0.0f && ls.rate_int > 0.0f);
	for (int i = 0; i < 20000; i++) fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK(ls.att_int == g.v[FC_RIG_ATT_I_LIM] && ls.rate_int == g.v[FC_RIG_RATE_I_LIM]);
	CHECK(dbg.att_int == ls.att_int && dbg.rate_int == ls.rate_int);
	// frozen (integrate = false): no change even with error
	in = law_in(0.3f, 0.0f);
	const float a0 = ls.att_int, r0 = ls.rate_int;
	for (int i = 0; i < 100; i++) fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK(ls.att_int == a0 && ls.rate_int == r0);
	// anti-windup: positive increments are skipped while positive moment is saturated
	fc_rig_law_reset(&ls);
	in = law_in(-0.3f, 0.0f);
	in.integrate = true;
	in.sat_pos = true;
	for (int i = 0; i < 100; i++) fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK(ls.att_int == 0.0f && ls.rate_int == 0.0f);
	in.sat_pos = false;
	in.sat_neg = true;                    // the other side saturated: winding up is fine
	fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK(ls.att_int > 0.0f);
	// negative limit
	fc_rig_law_reset(&ls);
	in = law_in(0.3f, 0.0f);
	in.integrate = true;
	for (int i = 0; i < 20000; i++) fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK(ls.att_int == -g.v[FC_RIG_ATT_I_LIM] && ls.rate_int == -g.v[FC_RIG_RATE_I_LIM]);

	// saturation helper: duty[2]/[3] left pair raises +moment
	bool sp, sn;
	const float d_left_hi[4] = {0.5f, 0.5f, 0.95f, 0.5f};
	fc_rig_moment_saturation(d_left_hi, 0.95f, &sp, &sn);
	CHECK(sp && !sn);
	const float d_right_zero[4] = {0.0f, 0.5f, 0.5f, 0.5f};
	fc_rig_moment_saturation(d_right_zero, 0.95f, &sp, &sn);
	CHECK(sp && !sn);
	const float d_right_hi[4] = {0.5f, 0.95f, 0.5f, 0.5f};
	fc_rig_moment_saturation(d_right_hi, 0.95f, &sp, &sn);
	CHECK(!sp && sn);
	const float d_ok[4] = {0.5f, 0.5f, 0.5f, 0.5f};
	fc_rig_moment_saturation(d_ok, 0.95f, &sp, &sn);
	CHECK(!sp && !sn);
}

static void t_law_filters_and_exc()
{
	fc_rig_gains g;
	fc_rig_gains_default(&g);
	float u[4];
	fc_rig_dbg dbg;
	fc_rig_law_state ls;
	// excitation targets
	fc_rig_law_reset(&ls);
	fc_rig_law_in in = law_in(0.0f, 0.0f);
	in.exc_target = FC_RIG_TGT_ATT;
	in.exc_val = 0.1f;
	fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK_NEAR(dbg.roll_tgt, 0.1f, 1e-7);
	CHECK_NEAR(u[1], flown_roll_moment(0.0f, 0.0f, 0.1f), 1e-5 * std::fabs(flown_roll_moment(0.0f, 0.0f, 0.1f)));
	fc_rig_law_reset(&ls);
	in.exc_target = FC_RIG_TGT_RATE;
	fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK_NEAR(dbg.rate_tgt, 0.1f, 1e-7);
	CHECK(dbg.roll_tgt == 0.0f);
	fc_rig_law_reset(&ls);
	in.exc_target = FC_RIG_TGT_MOMENT;
	in.exc_val = 1e-4f;
	fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK(u[1] == 1e-4f && dbg.rate_cmd == 0.0f && dbg.exc_val == 1e-4f);

	// gyro LPF off: rate loop sees the raw gyro; on: a step is smoothed
	fc_rig_law_reset(&ls);
	in = law_in(0.0f, 0.0f);
	fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	in.gyro[0] = 1.0f;
	fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK_NEAR(ls.gx_f, 1.0f, 0.0);
	CHECK(fc_rig_gain_set(&g, "GYRO_LPF_HZ", 10.0f) >= 0);
	fc_rig_law_reset(&ls);
	in.gyro[0] = 0.0f;
	fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	in.gyro[0] = 1.0f;
	fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK(ls.gx_f > 0.0f && ls.gx_f < 0.1f);
	for (int i = 0; i < 2000; i++) fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK_NEAR(ls.gx_f, 1.0f, 1e-3);

	// D: opposes angular acceleration, vanishes at constant rate, no kick on a setpoint step
	fc_rig_gains_default(&g);
	CHECK(fc_rig_gain_set(&g, "RATE_D", 0.5f) >= 0);
	fc_rig_law_reset(&ls);
	in = law_in(0.0f, 0.0f);
	float base = 0.0f;
	for (int i = 0; i < 200; i++) {
		in.gyro[0] = 0.001f * (float)i;      // accelerating at 1/0.0012 rad/s^2
		fc_rig_roll_law(&ls, &g, &in, u, &dbg);
		base = -g.v[FC_RIG_RATE_P] * in.gyro[0] * J0 * g.v[FC_RIG_ATT_GAIN];
	}
	CHECK(std::fabs(ls.dgx_f - 0.001f / 0.0012f) < 0.01f && u[1] < base);   // d/dt of the ramp
	for (int i = 0; i < 2000; i++) fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK(std::fabs(ls.dgx_f) < 1e-3f);
	in.sp_roll = 0.2f;
	const float d_before = ls.dgx_f;
	fc_rig_roll_law(&ls, &g, &in, u, &dbg);
	CHECK(ls.dgx_f == d_before || std::fabs(ls.dgx_f) < 1e-3f);
}

// The controller_pid.cpp hook's view: g_rig.out mirrors fc_rig_step's result, and
// fc_rig_law_input builds the same law inputs one would fill by hand.
static void t_law_input_from_state()
{
	Sim m;
	m.to_run();
	CHECK(m.cmd("EXC sine rate 500 4000 2000"));
	m.run(700);
	CHECK(std::memcmp(&m.s.out, &m.out, sizeof m.out) == 0);
	CHECK(m.s.out.exc_val != 0.0f && m.s.out.integrate);
	m.s.sat_pos = true;
	const float gyro[3] = {0.2f, -0.1f, 0.05f};
	fc_rig_law_in a, b;
	fc_rig_law_input(&m.s, 0.04f, 0.01f, 0.02f, gyro, 0.0012f, &a);
	std::memset(&b, 0, sizeof b);
	b.roll = 0.04f; b.pitch = 0.01f; b.yaw_err = 0.02f;
	b.gyro[0] = gyro[0]; b.gyro[1] = gyro[1]; b.gyro[2] = gyro[2];
	b.dt = 0.0012f;
	b.sp_roll = m.out.sp_roll;
	b.exc_target = m.out.exc_target;
	b.exc_val = m.out.exc_val;
	b.coll_thrust = m.out.coll_frac * (float)(PID_MASS_KG) * GRAV;
	b.integrate = m.out.integrate;
	b.int_zero = m.out.int_zero;
	b.lift_assist = m.out.lift_assist;
	b.sat_pos = true;
	CHECK(std::memcmp(&a, &b, sizeof a) == 0);
	fc_rig_law_state la, lb;
	fc_rig_law_reset(&la);
	fc_rig_law_reset(&lb);
	float ua[4], ub[4];
	fc_rig_roll_law(&la, &m.s.gains, &a, ua, nullptr);
	fc_rig_roll_law(&lb, &m.s.gains, &b, ub, nullptr);
	CHECK(ua[0] == ub[0] && ua[1] == ub[1] && ua[0] > 0.0f);
	// idle: zero collective, integrators held
	Sim n;
	n.run(10);
	fc_rig_law_input(&n.s, 0.0f, 0.0f, 0.0f, gyro, 0.001f, &a);
	CHECK(a.coll_thrust == 0.0f && a.int_zero && !a.integrate && a.exc_val == 0.0f);
}


// ---------------------------------------------------------------------------------------------
// Review fixes (M1 M2 M3 and the minors)

// M1: a COLL during RAMP_UP must not step the collective, and RAMP_UP -> RUN is continuous.
static void t_ramp_latch()
{
	const float ramp_ms = 1500.0f;                 // RIG_RAMP_MS
	for (int new_coll : {1300, 0, 500}) {
		Sim m;
		m.cmd("COLL 100");
		CHECK(m.until(FC_RIG_RAMP_UP, 1100) > 0);
		m.run(700);                                  // mid-ramp, coll ~0.047
		const float before = m.out.coll_frac;
		char l[32];
		std::snprintf(l, sizeof l, "COLL %d", new_coll);
		CHECK(m.cmd(l));
		float prev = before, worst_up = 0.0f, worst_run = 0.0f;
		bool saw_run = false;
		for (int i = 0; i < 4000 && m.out.rs != FC_RIG_DONE; i++) {
			m.tick();
			const float d = std::fabs(m.out.coll_frac - prev);
			if (m.out.rs == FC_RIG_RAMP_UP) worst_up = std::max(worst_up, d);
			else if (m.out.rs == FC_RIG_RUN) { worst_run = std::max(worst_run, d); saw_run = true; }
			prev = m.out.coll_frac;
		}
		// the ramp keeps its latched target (0.1 over 1.5 s) and RUN slews at 0.5/s, handover included
		CHECK(saw_run && !m.estop_seen);
		CHECK(worst_up <= 0.1f / ramp_ms + 1e-6f);
		CHECK(worst_run <= 0.5f * 0.001f + 1e-6f);
		CHECK_NEAR(m.out.coll_frac, new_coll * 1e-3f, 1e-5);
		std::printf("COLL %d mid-ramp: worst step ramp %.6f run %.6f\n", new_coll, (double)worst_up, (double)worst_run);
	}
	// the review's P1: COLL 100, then COLL 1300 in RAMP_UP -> no 0.093 -> 1.212 jump
	Sim p;
	p.cmd("COLL 100");
	CHECK(p.until(FC_RIG_RAMP_UP, 1100) > 0);
	p.run(1400);
	const float c0 = p.out.coll_frac;
	p.cmd("COLL 1300");
	p.tick();
	CHECK(std::fabs(p.out.coll_frac - c0) < 1e-3f && p.s.coll_ramp_to == 0.1f);
	// latched target cleared at DONE
	CHECK(p.cmd("STOP"));
	CHECK(p.until(FC_RIG_DONE, 3000) > 0);
	CHECK(p.s.coll_ramp_to == 0.0f);
}

// M2: the roll target is bounded in the law and at the commands.
static void t_tgt_bound()
{
	const float B = RIG_TGT_MAX_RAD;
	std::printf("RIG_TGT_MAX_RAD=%g\n", (double)B);
	CHECK(B > 0.0f && B < RIG_MAX_TILT_RAD);
	fc_rig_gains g;
	fc_rig_gains_default(&g);
	g.v[FC_RIG_ROLL_TRIM] = 0.175f;
	for (float sign : {1.0f, -1.0f}) {
		fc_rig_law_state ls;
		fc_rig_law_reset(&ls);
		fc_rig_law_in in = law_in(0.0f, 0.0f);
		g.v[FC_RIG_ROLL_TRIM] = sign * 0.175f;
		in.sp_roll = sign * 0.349f;
		in.exc_target = FC_RIG_TGT_ATT;
		in.exc_val = sign * 0.26f;                    // sum ~45 deg
		float u[4];
		fc_rig_dbg dbg;
		fc_rig_roll_law(&ls, &g, &in, u, &dbg);
		CHECK(dbg.roll_tgt == sign * B);
		fc_rig_gains g0;
		fc_rig_gains_default(&g0);
		CHECK_NEAR(u[1], flown_roll_moment(0.0f, 0.0f, sign * B), 1e-5 * std::fabs(flown_roll_moment(0.0f, 0.0f, B)));
	}

	// commands: RSP, GAIN ROLL_TRIM, EXC att refused when the worst case exceeds B
	Sim m;
	m.to_run();
	CHECK(m.out.rs == FC_RIG_RUN && m.out.guard_active);
	const int sp_md = (int)(B * 0.6f * 57295.78f);      // 60% of the bound
	char l[96];
	std::snprintf(l, sizeof l, "RSP %d", sp_md);
	CHECK(m.cmd(l) && std::string(m.reply) == "FCACK RSP\n");
	const float sp = m.s.sp_roll_cmd;
	m.run(2000);
	// trim that would push past B: refused, nothing changed
	const uint32_t gs = m.s.gain_seq;
	std::snprintf(l, sizeof l, "GAIN ROLL_TRIM %.4f", (double)std::fmin(0.175f, B - sp + 0.01f));
	CHECK(m.cmd(l) && std::string(m.reply) == "FCNAK GAIN\n" && m.s.gain_seq == gs);
	CHECK(m.s.nak_reason && std::strstr(m.s.nak_reason, "ROLL_TRIM"));
	CHECK(m.s.gains.v[FC_RIG_ROLL_TRIM] == 0.0f);
	// a smaller one fits
	CHECK(m.cmd("GAIN ROLL_TRIM 0.02") && m.s.gain_seq == gs + 1);
	// EXC att over the remaining room: NAK; inside: ACK
	const int room_mrad = (int)((B - sp - 0.02f) * 1000.0f);
	std::snprintf(l, sizeof l, "EXC step att %d 1000", room_mrad + 2);
	CHECK(m.cmd(l) && std::string(m.reply) == "FCNAK EXC\n");
	CHECK(m.s.nak_reason && std::strstr(m.s.nak_reason, "RIG_TGT_MAX_RAD"));
	std::snprintf(l, sizeof l, "EXC step att %d 1000", room_mrad - 2);
	CHECK(m.cmd(l) && std::strstr(m.reply, "FCACK EXC"));
	// with the att excitation running, an RSP that would overflow is refused
	std::snprintf(l, sizeof l, "RSP %d", sp_md + 3000);
	CHECK(m.cmd(l) && std::string(m.reply) == "FCNAK RSP\n" && m.s.sp_roll_cmd == sp);
	// and a negative setpoint counts by magnitude
	std::snprintf(l, sizeof l, "RSP %d", -sp_md - 3000);
	CHECK(m.cmd(l) && std::string(m.reply) == "FCNAK RSP\n");
	m.run(1100);                                   // excitation over
	std::snprintf(l, sizeof l, "RSP %d", -sp_md);
	CHECK(m.cmd(l) && std::string(m.reply) == "FCACK RSP\n");
	// the review's P4 (RSP 20000 + ROLL_TRIM 0.175 + EXC att 260 = ~45 deg) cannot be assembled
	Sim p;
	p.to_run();
	p.cmd("RSP 20000");
	p.cmd("GAIN ROLL_TRIM 0.175");
	p.run(1500);
	p.cmd("EXC step att 260 2000");
	p.tick();
	const float gyro[3] = {0, 0, 0};
	fc_rig_law_in li;
	fc_rig_law_input(&p.s, 0.0f, 0.0f, 0.0f, gyro, 0.001f, &li);
	float u[4];
	fc_rig_dbg d;
	fc_rig_roll_law(&p.s.law, &p.s.gains, &li, u, &d);
	std::printf("P4 roll_tgt %.4f rad (sp_cmd %.4f trim %.4f exc %.4f)\n", (double)d.roll_tgt,
		    (double)p.s.sp_roll_cmd, (double)p.s.gains.v[FC_RIG_ROLL_TRIM], (double)p.out.exc_val);
	CHECK(std::fabs(d.roll_tgt) <= B && std::fabs(p.s.sp_roll_cmd) + std::fabs(p.s.gains.v[FC_RIG_ROLL_TRIM]) +
	      std::fabs(p.out.exc_val) <= B * (1.0f + 1e-6f));
}

// M3: NaN / Inf on roll, pitch or any gyro.
static void t_nan()
{
	const float bad[] = {NAN, INFINITY, -INFINITY};
	const char *where[] = {"roll", "pitch", "gx", "gy", "gz"};
	auto poison = [](fc_rig_inputs &i, int w, float v) {
		if (w == 0) i.roll_rad = v; else if (w == 1) i.pitch_rad = v; else i.gyro[w - 2] = v;
	};
	for (int w = 0; w < 5; w++) {
		for (float v : bad) {
			// RUN: immediate estop, motors off, reason "rig nan"
			Sim r;
			r.to_run();
			CHECK(r.out.rs == FC_RIG_RUN);
			poison(r.in, w, v);
			r.tick();
			if (!(r.estop_seen && r.estop_reason && std::strcmp(r.estop_reason, "rig nan") == 0 &&
			      r.out.rs == FC_RIG_DONE && !r.out.armed && r.out.coll_frac == 0.0f)) {
				std::printf("FAIL RUN %s=%g: no immediate rig nan estop\n", where[w], (double)v);
				g_fail++;
			}
			// RAMP_UP: same
			Sim u;
			u.cmd("COLL 1000");
			CHECK(u.until(FC_RIG_RAMP_UP, 1100) > 0);
			u.run(200);
			poison(u.in, w, v);
			u.tick();
			if (!(u.estop_seen && std::strcmp(u.estop_reason, "rig nan") == 0 && !u.out.armed)) {
				std::printf("FAIL RAMP_UP %s=%g: no rig nan estop\n", where[w], (double)v);
				g_fail++;
			}
			// RAMP_DOWN: same
			Sim dn;
			dn.to_run();
			dn.cmd("STOP");
			dn.run(100);
			CHECK(dn.out.rs == FC_RIG_RAMP_DOWN);
			poison(dn.in, w, v);
			dn.tick();
			CHECK(dn.estop_seen && !dn.out.armed);
			// IDLE: never arms (and never estops: motors are off), arms once it is clean again
			Sim i;
			i.cmd("COLL 1000");
			poison(i.in, w, v);
			i.run(3000);
			if (!(i.out.rs == FC_RIG_IDLE && !i.out.armed && !i.estop_seen)) {
				std::printf("FAIL IDLE %s=%g: rs=%d armed=%d\n", where[w], (double)v, i.out.rs, i.out.armed);
				g_fail++;
			}
			poison(i.in, w, 0.0f);
			CHECK(i.until(FC_RIG_RAMP_UP, 1100) > 0);
		}
	}

	// the law: zero moment, state dropped, nothing latched; never a NaN out
	fc_rig_gains g;
	fc_rig_gains_default(&g);
	g.v[FC_RIG_ATT_I] = 20.0f;
	g.v[FC_RIG_RATE_I] = 100.0f;
	g.v[FC_RIG_RATE_D] = 0.3f;
	g.v[FC_RIG_GYRO_LPF_HZ] = 20.0f;
	for (int w = 0; w < 5; w++) {
		for (float v : bad) {
			fc_rig_law_state ls;
			fc_rig_law_reset(&ls);
			fc_rig_law_in in = law_in(-0.1f, 0.2f);
			in.integrate = true;
			float u[4];
			fc_rig_dbg dbg;
			for (int k = 0; k < 50; k++) fc_rig_roll_law(&ls, &g, &in, u, &dbg);
			CHECK(ls.att_int != 0.0f && ls.rate_int != 0.0f);
			fc_rig_law_in bad_in = in;
			if (w == 0) bad_in.roll = v; else if (w == 1) bad_in.pitch = v; else bad_in.gyro[w - 2] = v;
			fc_rig_roll_law(&ls, &g, &bad_in, u, &dbg);
			if (!(u[1] == 0.0f && u[2] == 0.0f && u[3] == 0.0f && u[0] == in.coll_thrust && !ls.seeded &&
			      ls.att_int == 0.0f && ls.rate_int == 0.0f && dbg.moment == 0.0f)) {
				std::printf("FAIL law %s=%g: u1=%g seeded=%d\n", where[w], (double)v, (double)u[1], ls.seeded);
				g_fail++;
			}
			// next clean tick: same as a freshly reset law (the NaN did not stick in gx_f)
			fc_rig_law_state fresh;
			fc_rig_law_reset(&fresh);
			float uf[4];
			fc_rig_roll_law(&fresh, &g, &in, uf, nullptr);
			fc_rig_roll_law(&ls, &g, &in, u, &dbg);
			CHECK(std::isfinite(u[1]) && u[1] == uf[1]);
		}
	}
	// a non-finite input only the law sees (yaw error, the estimator gyro): the law zeroes the
	// moment AND the next fc_rig_step estops, so the rig does not sit with no roll authority
	for (float v : bad) {
		for (int which = 0; which < 2; which++) {
			Sim r;
			r.to_run();
			const float gy[3] = {which == 1 ? v : 0.0f, 0.0f, 0.0f};
			for (int k = 0; k < 2 && !r.estop_seen; k++) {
				fc_rig_law_in li;
				fc_rig_law_input(&r.s, 0.0f, 0.0f, which == 0 ? v : 0.0f, gy, 0.001f, &li);
				float u[4];
				fc_rig_roll_law(&r.s.law, &r.s.gains, &li, u, &r.s.dbg);
				CHECK(u[1] == 0.0f);
				r.tick();
			}
			CHECK(r.estop_seen && std::strcmp(r.estop_reason, "rig nan") == 0 && !r.out.armed);
		}
		// while disarmed it is just cleared (no estop), and arming clears it too
		Sim i;
		const float gy[3] = {0, 0, 0};
		fc_rig_law_in li;
		fc_rig_law_input(&i.s, 0.0f, 0.0f, v, gy, 0.001f, &li);
		float u[4];
		fc_rig_roll_law(&i.s.law, &i.s.gains, &li, u, nullptr);
		CHECK(i.s.law.nonfinite);
		i.tick();
		CHECK(!i.s.law.nonfinite && !i.estop_seen);
	}
	// other non-finite law inputs, and finite inputs that overflow
	{
		fc_rig_law_state ls;
		float u[4];
		for (int which = 0; which < 5; which++) {
			fc_rig_law_reset(&ls);
			fc_rig_law_in in = law_in(0.0f, 0.0f);
			if (which == 0) in.sp_roll = NAN;
			if (which == 1) in.exc_val = INFINITY;
			if (which == 2) in.dt = NAN;
			if (which == 3) { in.coll_thrust = NAN; }
			if (which == 4) in.gyro[0] = 1e37f;      // finite, but RATE_P * it overflows
			fc_rig_roll_law(&ls, &g, &in, u, nullptr);
			CHECK(u[1] == 0.0f && std::isfinite(u[0]) && ls.nonfinite);
		}
	}
}

// Minor: integration and EXC wait for liftoff (guard_active), not just RUN.
static void t_liftoff_gate()
{
	Sim m;
	m.in.roll_rad = 0.506f;                      // review P3: resting at 29 deg
	m.cmd("COLL 1000");
	CHECK(m.until(FC_RIG_RUN, 3000) > 0);
	CHECK(!m.out.guard_active && !m.out.integrate && !m.out.int_zero);
	// the law is fed a large error but must not integrate (lift assist off: LIFT_I 0; with it on,
	// only the angle integrator builds, at LIFT_I -- t_lift_assist)
	fc_rig_gains_default(&m.s.gains);
	m.s.gains.v[FC_RIG_ATT_I] = 20.0f;
	m.s.gains.v[FC_RIG_RATE_I] = 100.0f;
	m.s.gains.v[FC_RIG_LIFT_I] = 0.0f;
	const float gyro[3] = {0, 0, 0};
	for (int i = 0; i < 300; i++) {
		m.tick();
		fc_rig_law_in li;
		fc_rig_law_input(&m.s, m.in.roll_rad, 0.0f, 0.0f, gyro, 0.001f, &li);
		float u[4];
		fc_rig_roll_law(&m.s.law, &m.s.gains, &li, u, &m.s.dbg);
	}
	CHECK(m.s.law.att_int == 0.0f && m.s.law.rate_int == 0.0f);
	CHECK(m.cmd("EXC step moment 100 1000") && std::string(m.reply) == "FCNAK EXC\n");
	CHECK(m.s.nak_reason && std::strcmp(m.s.nak_reason, "EXC: not lifted off yet") == 0);
	CHECK(m.s.exc_id == 0);
	// lifts off: integration and EXC allowed
	m.in.roll_rad = 0.2f;
	m.tick();
	CHECK(m.out.guard_active && m.out.integrate);
	fc_rig_law_in li;
	fc_rig_law_input(&m.s, m.in.roll_rad, 0.0f, 0.0f, gyro, 0.001f, &li);
	float u[4];
	fc_rig_roll_law(&m.s.law, &m.s.gains, &li, u, &m.s.dbg);
	CHECK(m.s.law.att_int != 0.0f);
	CHECK(m.cmd("EXC step moment 100 1000") && std::strstr(m.reply, "FCACK EXC"));
}

// Lift-off assist (fc_rig.cpp LIFT-OFF ASSIST): RUN before lift-off builds the angle integrator at
// LIFT_I under min(LIFT_I_LIM, ATT_I_LIM) and a moment clamp; the value is kept at lift-off.
struct AssistSim : Sim {
	float u[4] = {0, 0, 0, 0};
	void law(float gx = 0.0f)
	{
		const float gyro[3] = {gx, 0.0f, 0.0f};
		fc_rig_law_in li;
		fc_rig_law_input(&s, in.roll_rad, 0.0f, 0.0f, gyro, 0.001f, &li);
		fc_rig_roll_law(&s.law, &s.gains, &li, u, &s.dbg);
	}
	void step(int n) { for (int i = 0; i < n; i++) { tick(); law(); } }
};

static void t_lift_assist()
{
	// on the +29 deg stop, right wing down: assist on by default, builds NEGATIVE (push off +roll)
	{
		AssistSim m;
		fc_rig_gains g;
		fc_rig_gains_default(&g);
		CHECK(g.v[FC_RIG_LIFT_I] > 0.0f && g.v[FC_RIG_LIFT_I_LIM] <= g.v[FC_RIG_ATT_I_LIM]);
		float lo, hi, alo, ahi;
		fc_rig_gain_range(FC_RIG_LIFT_I_LIM, &lo, &hi);
		fc_rig_gain_range(FC_RIG_ATT_I_LIM, &alo, &ahi);
		CHECK(hi <= ahi);                                // the hand-off fits ATT_I_LIM's range
		m.in.roll_rad = 0.506f;
		m.cmd("COLL 1000");
		CHECK(m.until(FC_RIG_RUN, 3000) > 0);
		CHECK(m.out.lift_assist && !m.out.integrate && !m.out.guard_active);
		m.s.gains.v[FC_RIG_RATE_I] = 100.0f;             // must stay unused before lift-off
		m.step(1);
		const float a1 = m.s.law.att_int;
		m.step(500);
		CHECK(a1 < 0.0f && m.s.law.att_int < a1);         // grows, toward pushing off the + stop
		CHECK(m.s.law.rate_int == 0.0f);
		CHECK(m.s.dbg.att_int == m.s.law.att_int);       // recorded as att_int
		// the moment never exceeds RIG_LIFT_MOMENT_FRAC (1.0) x the authority
		const float cap = fc_rig_moment_authority(m.out.coll_frac * (float)(PID_MASS_KG) * GRAV);
		CHECK(std::fabs(m.u[1]) <= cap * (1.0f + 1e-6f));
	}
	// limits: ATT_I_LIM below LIFT_I_LIM bounds the assist; lowering LIFT_I_LIM clamps at once
	{
		AssistSim m;
		m.in.roll_rad = 0.506f;
		m.cmd("COLL 1000");
		CHECK(m.cmd("GAIN LIFT_I 10") && m.cmd("GAIN ATT_I_LIM 0.5"));
		CHECK(m.until(FC_RIG_RUN, 3000) > 0);
		m.step(1000);                                    // (RIG_LIFTOFF_TIMEOUT_MS 2000 here)
		CHECK(m.s.law.att_int == -0.5f && m.saw("RIG lift assist: at limit"));
		CHECK(m.cmd("GAIN ATT_I_LIM 5") && m.cmd("GAIN LIFT_I_LIM 0.25"));
		CHECK(m.s.law.att_int == -0.25f);                // rebase clamps to the assist's limit
		m.step(10);
		CHECK(m.s.law.att_int == -0.25f);
		// no lift-off by the timeout: ramps down as before, the assist ends with RUN
		CHECK(m.until(FC_RIG_RAMP_DOWN, 9000) > 0);
		CHECK(std::strcmp(m.out.rampdown_reason, "RIG liftoff timeout") == 0 && !m.out.lift_assist);
	}
	// the moment clamp is anti-windup too: mixer unsaturated, clamp binding -> integration stops
	{
		AssistSim m;
		m.in.roll_rad = 0.506f;
		m.cmd("COLL 300");                               // small collective: small authority
		CHECK(m.cmd("GAIN LIFT_I 10") && m.cmd("GAIN ATT_P 1"));   // P alone well inside it
		CHECK(m.until(FC_RIG_RUN, 3000) > 0);
		m.step(1000);
		const float cap = fc_rig_moment_authority(m.out.coll_frac * (float)(PID_MASS_KG) * GRAV);
		CHECK(m.u[1] == -cap && m.s.law.assist_sat_neg && !m.s.law.assist_sat_pos);
		CHECK(!m.s.sat_pos && !m.s.sat_neg);             // (no mixer in this driver)
		const float held = m.s.law.att_int;
		CHECK(held > -m.s.gains.v[FC_RIG_LIFT_I_LIM]);   // stopped short of the limit
		m.step(500);
		CHECK(m.s.law.att_int == held);
		CHECK(m.saw("RIG lift assist: at limit"));
	}
	// hand-off: the value is kept as ATT_I's initial state; ATT_I 0 holds it; no moment clamp after
	{
		AssistSim m;
		m.in.roll_rad = 0.506f;
		m.cmd("COLL 1000");
		CHECK(m.until(FC_RIG_RUN, 3000) > 0);
		m.step(1500);
		m.in.roll_rad = 0.18f;                           // inside the lift-off band (0.39)
		m.tick();
		CHECK(m.out.guard_active && m.out.integrate && !m.out.lift_assist);
		const float trim = m.s.law.att_int;
		CHECK(trim < 0.0f);
		m.law();
		CHECK(m.s.law.att_int == trim);                  // ATT_I 0 (default): held, not dumped
		CHECK(!m.s.law.assist_sat_pos && !m.s.law.assist_sat_neg);
		// moment continuity across the hand-off: the same law tick with and without the flag
		fc_rig_law_state a = m.s.law, b = m.s.law;
		const float gyro[3] = {0, 0, 0};
		fc_rig_law_in li;
		fc_rig_law_input(&m.s, 0.18f, 0.0f, 0.0f, gyro, 0.001f, &li);
		float ua[4], ub[4];
		fc_rig_roll_law(&a, &m.s.gains, &li, ua, nullptr);
		li.integrate = false; li.lift_assist = true;
		fc_rig_roll_law(&b, &m.s.gains, &li, ub, nullptr);
		CHECK(std::fabs(ua[1] - ub[1]) < 1e-3f * std::fabs(ua[1]));   // one tick's LIFT_I increment
		// overshoot: swings past level to the other side -> ATT_I unwinds, never winds further
		m.s.gains.v[FC_RIG_ATT_I] = 2.0f;
		m.in.roll_rad = -0.10f;
		float prev = m.s.law.att_int;
		for (int i = 0; i < 300; i++) {
			m.tick();
			m.law();
			CHECK(m.s.law.att_int >= prev);              // moves toward + (less push to -roll)
			prev = m.s.law.att_int;
		}
		CHECK(m.s.law.att_int > trim);
	}
	// LIFT_I 0: off, exactly the old law (no clamp, no build-up)
	{
		AssistSim m;
		m.in.roll_rad = 0.506f;
		m.cmd("COLL 100");
		CHECK(m.cmd("GAIN LIFT_I 0"));
		CHECK(m.until(FC_RIG_RUN, 3000) > 0);
		CHECK(!m.out.lift_assist);
		m.step(500);
		const float cap = fc_rig_moment_authority(m.out.coll_frac * (float)(PID_MASS_KG) * GRAV);
		CHECK(m.s.law.att_int == 0.0f && m.u[1] < -cap);   // P alone, unclamped
	}
}

// Minor: RESET then STOP in the same tick must not lose the STOP.
static void t_reset_stop()
{
	Sim m;
	m.cmd("COLL 1000");
	m.run(200);
	CHECK(m.out.rs == FC_RIG_ARMING);
	m.reset();                                   // RESET ...
	m.cmd("STOP");                               // ... then STOP, before the next tick
	m.tick();
	CHECK(m.out.rs == FC_RIG_DONE && m.saw("RIG stop: arming cancelled"));
	m.run(3000);
	CHECK(m.out.rs == FC_RIG_DONE && !m.out.armed);
	// a STOP from an earlier tick is consumed by then: a later RESET still re-enables arming
	Sim n;
	n.cmd("STOP");
	n.tick();
	n.reset();
	n.cmd("COLL 1000");
	CHECK(n.until(FC_RIG_RAMP_UP, 1200) > 0);
}

// Minor: bumpless GAIN, and MOTOR_TRIM_LEFT slewed.
static void t_bumpless()
{
	Cmd c;
	c.send("GAIN ATT_I 10");
	c.send("GAIN RATE_I 50");
	c.s.law.att_int = 0.2f;
	c.s.law.rate_int = 5.0f;
	// integral part of the moment at e = 0, gx = 0, D = 0: J0 * ATT_GAIN * (RATE_P * att_int + rate_int)
	auto moment = [&]() {
		fc_rig_law_state ls = c.s.law;
		fc_rig_law_in in = law_in(0.0f, 0.0f);   // frozen (integrate false), so the state is read only
		float u[4];
		fc_rig_roll_law(&ls, &c.s.gains, &in, u, nullptr);
		return u[1];
	};
	const float m0 = moment();
	CHECK(m0 != 0.0f);
	for (const char *l : {"GAIN ATT_GAIN 2", "GAIN RATE_P 80", "GAIN ATT_GAIN 0.5", "GAIN RATE_P 25",
			      "GAIN ATT_I 30", "GAIN RATE_I 200", "GAIN ATT_P 8", "GAIN RATE_D 0.5",
			      "GAIN D_LPF_HZ 50", "GAIN GYRO_LPF_HZ 20", "GAIN ATT_I 0", "GAIN RATE_I 0"}) {
		CHECK(c.send(l) && std::strstr(c.reply, "FCACK GAIN"));
		const float m1 = moment();
		if (!(std::fabs(m1 - m0) <= 1e-5f * std::fabs(m0))) {
			std::printf("FAIL \"%s\" stepped the integral moment %g -> %g\n", l, (double)m0, (double)m1);
			g_fail++;
		}
	}
	// ROLL_TRIM / MOTOR_TRIM_LEFT leave the integrators alone
	const float a = c.s.law.att_int, r = c.s.law.rate_int;
	CHECK(c.send("GAIN ROLL_TRIM 0.01") && c.send("GAIN MOTOR_TRIM_LEFT 0.9"));
	CHECK(c.s.law.att_int == a && c.s.law.rate_int == r);
	// a lower limit clamps (the one gain that may step the output); the other integrator is kept
	CHECK(c.send("GAIN RATE_I_LIM 1") && std::fabs(c.s.law.rate_int) <= 1.0f && c.s.law.att_int == a);
	CHECK(c.send("GAIN ATT_I_LIM 0") && c.s.law.att_int == 0.0f && c.s.law.rate_int != 0.0f);

	// MOTOR_TRIM_LEFT: commanded value in gains.v at once (echo, recorder, RBPOKE), applied value
	// slewed at <= 0.5/s while the motors run
	Sim m;
	const float def = m.s.gains.v[FC_RIG_MOTOR_TRIM_LEFT];
	CHECK(m.s.trim_left_applied == def);
	m.to_run();
	const float tgt = def > 1.0f ? def - 0.13f : def + 0.13f;
	char l[64];
	std::snprintf(l, sizeof l, "GAIN MOTOR_TRIM_LEFT %.2f", (double)tgt);
	CHECK(m.cmd(l) && std::strstr(m.reply, "FCACK GAIN"));
	CHECK_NEAR(m.s.gains.v[FC_RIG_MOTOR_TRIM_LEFT], tgt, 1e-6);
	float prev = m.s.trim_left_applied, worst = 0.0f;
	int ticks = 0;
	while (m.s.trim_left_applied != m.s.gains.v[FC_RIG_MOTOR_TRIM_LEFT] && ticks < 1000) {
		m.tick();
		worst = std::max(worst, std::fabs(m.s.trim_left_applied - prev));
		prev = m.s.trim_left_applied;
		ticks++;
	}
	CHECK(worst <= 0.5f * 0.001f + 1e-6f && ticks >= 255 && ticks <= 265);
	// disarmed: follows at once (no motors to bump)
	m.cmd("STOP");
	CHECK(m.until(FC_RIG_DONE, 3000) > 0);
	CHECK(m.cmd("GAIN MOTOR_TRIM_LEFT 0.8"));
	m.tick();
	CHECK(m.s.trim_left_applied == 0.8f);
}

// Minor: the rate-excitation cap scales with collective (and the live gains) like the moment cap.
static void t_rate_cap()
{
	const float u0 = (float)(PID_MASS_KG) * GRAV;
	Sim m;
	m.to_run();
	auto cap = [&](float coll) {
		const float mc = 0.3f * 0.5f * coll * u0 * L_ARM;
		const float c = mc / (m.s.gains.v[FC_RIG_RATE_P] * J0 * m.s.gains.v[FC_RIG_ATT_GAIN]);
		return std::fmin(c, 3.0f);
	};
	const int cap_mrad = (int)(cap(1.0f) * 1000.0f);
	std::printf("rate cap at hover %d mrad/s\n", cap_mrad);
	char l[96];
	std::snprintf(l, sizeof l, "EXC step rate %d 1000", cap_mrad + 2);
	CHECK(m.cmd(l) && std::string(m.reply) == "FCNAK EXC\n");
	std::snprintf(l, sizeof l, "EXC sine rate %d 8000 2000", cap_mrad - 2);
	CHECK(m.cmd(l) && std::strstr(m.reply, "FCACK EXC"));
	// RATE_P x4 and COLL 300 while it runs: the live value stays under the new cap, and its moment
	// (RATE_P * amp * J0 * ATT_GAIN) under the moment cap
	m.cmd("GAIN RATE_P 160");
	m.cmd("COLL 300");
	bool over = false;
	float worst = 0.0f;
	for (int i = 0; i < 4000; i++) {
		m.tick();
		const float c = cap(m.out.coll_frac);
		const float mom = m.s.gains.v[FC_RIG_RATE_P] * std::fabs(m.out.exc_val) * J0 * m.s.gains.v[FC_RIG_ATT_GAIN];
		if (std::fabs(m.out.exc_val) > c * (1.0f + 1e-5f) ||
		    mom > 0.3f * 0.5f * m.out.coll_frac * u0 * L_ARM * (1.0f + 1e-5f)) over = true;
		worst = std::max(worst, std::fabs(m.out.exc_val));
	}
	CHECK(!over && worst > 0.0f);
}

// ---------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
	struct { const char *name; void (*fn)(); } groups[] = {
		{"parser", t_parser}, {"format", t_format}, {"gains", t_gains}, {"commands", t_commands},
		{"sm_normal", t_sm_normal}, {"sm_stop_liftoff", t_sm_stop_liftoff},
		{"sm_liftoff_timeout", t_sm_liftoff_timeout}, {"sm_heartbeat", t_sm_heartbeat},
		{"sm_pitch_guard", t_sm_pitch_guard}, {"sm_reset_rearm", t_sm_reset_rearm},
		{"sm_max_run", t_sm_max_run}, {"sm_rampdown_on_stop", t_sm_rampdown_on_stop},
		{"sm_slews", t_sm_slews}, {"exc_shapes", t_exc_shapes}, {"exc_commands", t_exc_commands},
		{"law_matches_flown", t_law_matches_flown}, {"law_matches_controller_pid", t_law_matches_controller_pid},
		{"law_integrators", t_law_integrators}, {"law_input", t_law_input_from_state}, {"law_filters_and_exc", t_law_filters_and_exc},
		{"ramp_latch", t_ramp_latch}, {"tgt_bound", t_tgt_bound}, {"nan", t_nan}, {"liftoff_gate", t_liftoff_gate},
		{"reset_stop", t_reset_stop}, {"bumpless", t_bumpless}, {"rate_cap", t_rate_cap},
		{"lift_assist", t_lift_assist},
	};
	const char *want = argc > 1 ? argv[1] : "all";
	bool found = false;
	for (auto &grp : groups) {
		if (std::strcmp(want, "all") == 0 || std::strcmp(want, grp.name) == 0) {
			found = true;
			const int before = g_fail;
			grp.fn();
			std::printf("%s %s\n", g_fail == before ? "ok" : "FAILED", grp.name);
		}
	}
	if (!found) {
		std::printf("no group %s\n", want);
		return 2;
	}
	return g_fail ? 1 : 0;
}
