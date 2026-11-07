/*
 * Host check for workloads/esp_bridge/src/esp_safety_core.h: the re-arm gate
 * and the RTC record format. Driven by tests/test_esp_safety.py; prints one
 * "ok <name>" line per case and exits nonzero on the first failure.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_safety_core.h"
#include <riskybird/motor_link.h>   /* the real flag values, as main.c passes them */

#define ARMED  MOTOR_LINK_FLAG_ARMED
#define ESTOP  MOTOR_LINK_FLAG_ESTOP
#define WINDOW MOTOR_LINK_FLAG_WINDOW
#define COMMIT MOTOR_LINK_FLAG_COMMIT
#define BLOCK  MOTOR_LINK_FLAGS_BLOCK

static int failures;

#define CHECK(cond)                                                            \
	do {                                                                   \
		if (!(cond)) {                                                 \
			fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
			failures++;                                            \
		}                                                              \
	} while (0)

static bool frame(struct esp_rearm *r, unsigned flags, bool killed)
{
	return esp_rearm_frame(r, (uint8_t)flags, ARMED, BLOCK, killed);
}

static void boot_needs_disarm_arm_edge(void)
{
	struct esp_rearm r;

	esp_rearm_init(&r);
	CHECK(r.hold == ESP_REARM_BOOT);
	/* The FC kept sending ARMED across an ESP-only reset: refused. */
	CHECK(!frame(&r, ARMED | COMMIT, false));
	CHECK(!frame(&r, ARMED | COMMIT, false));
	CHECK(r.blocked == 2u);
	CHECK(r.hold == ESP_REARM_BOOT);
	/* Disarm, then arm: drives, and the hold is gone. */
	CHECK(!frame(&r, COMMIT, false));
	CHECK(frame(&r, ARMED | COMMIT, false));
	CHECK(r.hold == ESP_REARM_NONE);
	CHECK(frame(&r, ARMED, false));
	CHECK(r.blocked == 2u);
	puts("ok boot_needs_disarm_arm_edge");
}

static void normal_boot_disarmed_first(void)
{
	struct esp_rearm r;

	/* The usual bench order: FC boots disarmed, frames say so, then RESET
	 * arms it. No extra step for the operator. */
	esp_rearm_init(&r);
	CHECK(!frame(&r, 0x00, false));
	CHECK(!frame(&r, COMMIT, false));
	CHECK(frame(&r, ARMED | COMMIT, false));
	CHECK(r.blocked == 0u);
	puts("ok normal_boot_disarmed_first");
}

/* ESP_LINK_REARM_AFTER_CUT=0 (the default): the FC keeps sending ARMED across
 * a 120 ms link gap and never disarms; the next ARMED frame must drive. */
static void failsafe_cut_resumes_by_default(void)
{
	struct esp_rearm r;

	esp_rearm_init(&r);
	CHECK(!frame(&r, 0, false));
	CHECK(frame(&r, ARMED, false));
	esp_rearm_on_cut(&r, false);
	CHECK(r.hold == ESP_REARM_NONE);
	CHECK(frame(&r, ARMED, false));
	CHECK(r.blocked == 0u);
	/* ... and does not disturb a hold that is already pending. */
	esp_rearm_init(&r);
	esp_rearm_on_cut(&r, false);
	CHECK(r.hold == ESP_REARM_BOOT);
	CHECK(!frame(&r, ARMED, false));
	puts("ok failsafe_cut_resumes_by_default");
}

/* ESP_LINK_REARM_AFTER_CUT=1: the cut needs a disarm -> arm edge. */
static void failsafe_cut_needs_edge_when_enabled(void)
{
	struct esp_rearm r;

	esp_rearm_init(&r);
	CHECK(!frame(&r, 0, false));
	CHECK(frame(&r, ARMED, false));
	esp_rearm_on_cut(&r, true);
	CHECK(!frame(&r, ARMED, false));
	CHECK(r.hold == ESP_REARM_CUT);
	CHECK(!frame(&r, 0, false));
	CHECK(frame(&r, ARMED, false));
	puts("ok failsafe_cut_needs_edge_when_enabled");
}

static void disarm_before_cut_does_not_count(void)
{
	struct esp_rearm r;

	esp_rearm_init(&r);
	CHECK(!frame(&r, 0, false));    /* evidence from BEFORE the hold ... */
	esp_rearm_on_cut(&r, true);
	CHECK(!frame(&r, ARMED, false)); /* ... does not satisfy it */
	puts("ok disarm_before_cut_does_not_count");
}

static void block_flags_count_as_disarm(void)
{
	struct esp_rearm r;

	esp_rearm_init(&r);
	CHECK(!frame(&r, ARMED | ESTOP, false));    /* ARMED but blocked: off */
	CHECK(r.blocked == 0u);                     /* not an ARMED refusal */
	CHECK(frame(&r, ARMED, false));
	esp_rearm_hold(&r, ESP_REARM_CUT);
	CHECK(!frame(&r, ARMED | WINDOW, false));
	CHECK(frame(&r, ARMED, false));
	puts("ok block_flags_count_as_disarm");
}

static void kill_fc_disarmed_during_latch(void)
{
	struct esp_rearm r;

	/* ESTOP reached both the ESP and the FC: the FC's frames go not-armed
	 * while the latch is set, then RESET clears both and the FC re-arms. */
	esp_rearm_init(&r);
	CHECK(!frame(&r, 0, false));
	CHECK(frame(&r, ARMED, false));
	esp_rearm_hold(&r, ESP_REARM_KILL);
	CHECK(!frame(&r, ESTOP, true));
	CHECK(!frame(&r, 0, true));
	/* RESET: latch clear (killed=false), FC arms. */
	CHECK(frame(&r, ARMED, false));
	puts("ok kill_fc_disarmed_during_latch");
}

static void kill_fc_never_disarmed(void)
{
	struct esp_rearm r;

	/* ESTOP reached only the ESP: the FC kept sending ARMED all through the
	 * latch. After RESET the ESP still refuses until the FC disarms. */
	esp_rearm_init(&r);
	CHECK(!frame(&r, 0, false));
	CHECK(frame(&r, ARMED, false));
	esp_rearm_hold(&r, ESP_REARM_KILL);
	CHECK(!frame(&r, ARMED, true));
	CHECK(!frame(&r, ARMED, false));   /* RESET sent, FC still ARMED */
	CHECK(r.hold == ESP_REARM_KILL);
	CHECK(!frame(&r, 0, false));
	CHECK(frame(&r, ARMED, false));
	puts("ok kill_fc_never_disarmed");
}

static void armed_during_kill_erases_disarm(void)
{
	struct esp_rearm r;

	esp_rearm_init(&r);
	esp_rearm_hold(&r, ESP_REARM_KILL);
	CHECK(!frame(&r, 0, true));       /* disarm while killed ... */
	CHECK(!frame(&r, ARMED, true));   /* ... then armed again while killed */
	CHECK(!frame(&r, ARMED, false));  /* not an edge after the kill */
	puts("ok armed_during_kill_erases_disarm");
}

static void plain_reset_changes_nothing(void)
{
	struct esp_rearm r;

	/* No latch, no hold: the gate is transparent (RESET is the rig's
	 * routine arm command and adds no hold of its own). */
	esp_rearm_init(&r);
	CHECK(!frame(&r, 0, false));
	CHECK(frame(&r, ARMED, false));
	CHECK(frame(&r, ARMED, false));
	CHECK(!frame(&r, 0, false));
	CHECK(frame(&r, ARMED, false));
	CHECK(r.blocked == 0u);
	puts("ok plain_reset_changes_nothing");
}

static void hold_names(void)
{
	CHECK(strcmp(esp_rearm_hold_name(ESP_REARM_NONE), "0") == 0);
	CHECK(strcmp(esp_rearm_hold_name(ESP_REARM_BOOT), "boot") == 0);
	CHECK(strcmp(esp_rearm_hold_name(ESP_REARM_CUT), "cut") == 0);
	CHECK(strcmp(esp_rearm_hold_name(ESP_REARM_KILL), "kill") == 0);
	puts("ok hold_names");
}

static void records(void)
{
	uint32_t snap[SNAP_NW];
	uint32_t kill[KILL_NW];

	memset(snap, 0, sizeof(snap));
	CHECK(!esp_rec_valid(snap, SNAP_NW, ESP_REC_SNAP_MAGIC));   /* zeroed RAM */
	snap[SNAP_W_UP_MS] = 123456u;
	snap[SNAP_W_DUTY01] = 1000u | (2000u << 16);
	snap[SNAP_W_DUTY23] = 3000u | (4000u << 16);
	snap[SNAP_W_STATE] = esp_snap_state(0x21, true, false, ESP_REARM_CUT);
	esp_rec_seal(snap, SNAP_NW, ESP_REC_SNAP_MAGIC);
	CHECK(esp_rec_valid(snap, SNAP_NW, ESP_REC_SNAP_MAGIC));
	CHECK(snap[SNAP_W_STATE] == (0x21u | (1u << 8) | (2u << 12)));

	/* Any single torn word is caught. */
	for (unsigned i = 1; i + 1 < SNAP_NW; i++) {
		uint32_t keep = snap[i];

		snap[i] ^= 0x00010000u;
		CHECK(!esp_rec_valid(snap, SNAP_NW, ESP_REC_SNAP_MAGIC));
		snap[i] = keep;
	}
	CHECK(esp_rec_valid(snap, SNAP_NW, ESP_REC_SNAP_MAGIC));

	/* A record never validates as another type, even with equal payload. */
	memset(kill, 0, sizeof(kill));
	kill[KILL_W_KILLED] = 1u;
	esp_rec_seal(kill, KILL_NW, ESP_REC_KILL_MAGIC);
	CHECK(esp_rec_valid(kill, KILL_NW, ESP_REC_KILL_MAGIC));
	CHECK(!esp_rec_valid(kill, KILL_NW, ESP_REC_BOOT_MAGIC));
	kill[KILL_W_MAGIC] = ESP_REC_BOOT_MAGIC;
	CHECK(!esp_rec_valid(kill, KILL_NW, ESP_REC_BOOT_MAGIC));

	/* Power-on garbage: random words essentially never validate. */
	srand(1);
	unsigned accepted = 0;

	for (unsigned t = 0; t < 100000u; t++) {
		uint32_t g[BOOT_NW];

		for (unsigned i = 0; i < BOOT_NW; i++) {
			g[i] = ((uint32_t)rand() << 16) ^ (uint32_t)rand();
		}
		g[0] = ESP_REC_BOOT_MAGIC;   /* even with the magic right */
		accepted += esp_rec_valid(g, BOOT_NW, ESP_REC_BOOT_MAGIC);
	}
	CHECK(accepted == 0u);
	puts("ok records");
}

int main(void)
{
	boot_needs_disarm_arm_edge();
	normal_boot_disarmed_first();
	failsafe_cut_resumes_by_default();
	failsafe_cut_needs_edge_when_enabled();
	disarm_before_cut_does_not_count();
	block_flags_count_as_disarm();
	kill_fc_disarmed_during_latch();
	kill_fc_never_disarmed();
	armed_during_kill_erases_disarm();
	plain_reset_changes_nothing();
	hold_names();
	records();
	return failures ? 1 : 0;
}
