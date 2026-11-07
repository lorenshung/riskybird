/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * The pure half of esp_safety.c: the re-arm gate and the RTC record format.
 *
 * Freestanding on purpose -- stdint/stdbool and nothing else -- so the host
 * test (tests/test_esp_safety.py, tests/esp_safety_check.c) compiles exactly
 * this file and exercises the rules that cannot be checked without a board.
 * Everything here is static inline and touches no hardware.
 */
#ifndef RISKYBIRD_ESP_SAFETY_CORE_H
#define RISKYBIRD_ESP_SAFETY_CORE_H

#include <stdbool.h>
#include <stdint.h>

/* ---- the re-arm gate ------------------------------------------------------
 *
 * The rule: after any event that cut the gates without the sender asking, an
 * ARMED frame is not enough to drive again. The sender has to be seen NOT
 * armed at least once first, and then armed: a disarm -> arm edge, i.e. a
 * human (or the FC's own logic) deciding to arm again. Without it, an ESP
 * reset or a kill-clear while the FC is still sending ARMED frames steps the
 * gates straight back to the commanded duty.
 *
 * The events that set the hold:
 *   BOOT  every boot, whatever the reset cause (an ESP reboot already means
 *         >1 s without thrust, so requiring a re-arm loses nothing)
 *   KILL  the ESP kill latch was SET (UDP ESTOP). Clearing it (RESET) does
 *         not add a hold of its own: a plain RESET with no latch is the rig's
 *         routine arm command and must change nothing.
 *   CUT   the 120 ms silence failsafe fired -- ONLY when the build sets
 *         ESP_LINK_REARM_AFTER_CUT=1 (esp_rearm_on_cut). Off by default: in
 *         flight the FC keeps sending ARMED across a link gap and never
 *         disarms, so a CUT hold would keep the motors off for good after a
 *         120 ms hiccup. By default the cut only zeroes the gates and the next
 *         valid ARMED frame resumes (unlimited if the cut was shorter than
 *         SS_CUT_COLD_MS, through a cold spin-up if longer: motor_softstart.h).
 *
 * What counts as "not armed": a valid frame with ARMED clear, or with any
 * MOTOR_LINK_FLAGS_BLOCK bit set (the sender then forces ARMED clear anyway).
 * An ARMED frame received while the kill latch is set CLEARS the disarm
 * evidence, so a sender that never disarmed across the whole kill has to
 * disarm once after RESET before it can drive.
 */
enum esp_rearm_hold {
	ESP_REARM_NONE = 0,
	ESP_REARM_BOOT = 1,
	ESP_REARM_CUT  = 2,
	ESP_REARM_KILL = 3,
};

struct esp_rearm {
	uint8_t  hold;          /* enum esp_rearm_hold; NONE = frames may drive */
	bool     disarm_seen;   /* a not-armed frame arrived since the hold was set */
	uint32_t blocked;       /* ARMED frames refused by the hold, since boot */
};

static inline void esp_rearm_init(struct esp_rearm *r)
{
	r->hold = ESP_REARM_BOOT;
	r->disarm_seen = false;
	r->blocked = 0u;
}

static inline void esp_rearm_hold(struct esp_rearm *r, uint8_t why)
{
	r->hold = why;
	r->disarm_seen = false;
}

/* The silence failsafe just cut the gates. `rearm_after_cut` is the build's
 * ESP_LINK_REARM_AFTER_CUT: false leaves the gate as it was (frames resume),
 * true sets the CUT hold. */
static inline void esp_rearm_on_cut(struct esp_rearm *r, bool rearm_after_cut)
{
	if (rearm_after_cut) {
		esp_rearm_hold(r, ESP_REARM_CUT);
	}
}

/* One valid, in-sequence frame. `armed_bit` and `block_mask` are the
 * MOTOR_LINK_FLAG_ARMED / MOTOR_LINK_FLAGS_BLOCK values (passed in so this
 * header does not depend on motor_link.h). Returns true only if this frame
 * may drive the gates. */
static inline bool esp_rearm_frame(struct esp_rearm *r, uint8_t flags,
				   uint8_t armed_bit, uint8_t block_mask,
				   bool killed)
{
	bool wants_drive = (flags & armed_bit) != 0u && (flags & block_mask) == 0u;

	if (!wants_drive) {
		r->disarm_seen = true;
		return false;
	}
	if (killed) {
		r->disarm_seen = false;
		return false;
	}
	if (r->hold != ESP_REARM_NONE) {
		if (!r->disarm_seen) {
			r->blocked++;
			return false;
		}
		r->hold = ESP_REARM_NONE;   /* the disarm -> arm edge */
	}
	return true;
}

static inline const char *esp_rearm_hold_name(uint8_t hold)
{
	switch (hold) {
	case ESP_REARM_NONE: return "0";
	case ESP_REARM_BOOT: return "boot";
	case ESP_REARM_CUT:  return "cut";
	case ESP_REARM_KILL: return "kill";
	default:             return "?";
	}
}

/* ---- RTC records ------------------------------------------------------------
 *
 * Every record is an array of 32-bit words: word 0 a per-record magic, the
 * last word a check over all the others. Word-only so every store to LP SRAM
 * is a single aligned 32-bit write, and so a write torn by a reset is caught
 * by the check rather than half-trusted.
 *
 * On a true power-on LP SRAM holds whatever the cells settled to, so the magic
 * plus a 32-bit check rejects it with overwhelming probability -- which is
 * also exactly how "the rail collapsed far enough to lose RTC RAM" shows up
 * (rtc=cold), as opposed to a dip that reset the chip but kept it (rtc=kept).
 */
#define ESP_REC_BOOT_MAGIC  0x52424254u   /* "RBBT" */
#define ESP_REC_SNAP_MAGIC  0x52425350u   /* "RBSP" */
#define ESP_REC_FAULT_MAGIC 0x52424654u   /* "RBFT" */
#define ESP_REC_KILL_MAGIC  0x52424b4cu   /* "RBKL" */

/* FNV-1a over 32-bit words, seeded with the magic so two record types with
 * equal payloads never validate as each other, with a right-shift fold after
 * every multiply and a murmur3 fmix32 at the end. Plain FNV over whole words
 * only carries upward, so bit 31 of the hash depended on bit 31 of each word
 * alone: the same high bit flipped in two words validated (~100 % for bit 31,
 * 41 % for bit 30; tests/esp_safety_fuzz_check.c). The folds make every bit
 * depend on every bit. Changing the function invalidates the records written
 * by an older image once (that boot reads rtc=cold). always_inline, no calls,
 * no tables: the fatal handler runs it from IRAM and must not touch flash. */
static inline __attribute__((always_inline))
uint32_t esp_rec_check(const volatile uint32_t *w, unsigned nwords)
{
	uint32_t h = 2166136261u ^ w[0];

	for (unsigned i = 1; i + 1u < nwords; i++) {
		h = (h ^ w[i]) * 16777619u;
		h ^= h >> 15;
	}
	h ^= h >> 16;
	h *= 0x85ebca6bu;
	h ^= h >> 13;
	h *= 0xc2b2ae35u;
	h ^= h >> 16;
	return ~h;
}

static inline __attribute__((always_inline))
void esp_rec_seal(volatile uint32_t *w, unsigned nwords, uint32_t magic)
{
	w[0] = magic;
	w[nwords - 1u] = esp_rec_check(w, nwords);
}

static inline bool esp_rec_valid(const volatile uint32_t *w, unsigned nwords,
				 uint32_t magic)
{
	return w[0] == magic && w[nwords - 1u] == esp_rec_check(w, nwords);
}

/* Record layouts (word indices). */
enum { BOOT_W_MAGIC, BOOT_W_COUNT, BOOT_W_CHECK, BOOT_NW };

enum {
	SNAP_W_MAGIC,
	SNAP_W_UP_MS,     /* k_uptime at the store */
	SNAP_W_DUTY01,    /* duty[0] | duty[1] << 16, per 10000, as applied */
	SNAP_W_DUTY23,
	SNAP_W_STATE,     /* see esp_snap_state() */
	SNAP_W_CHECK,
	SNAP_NW
};

enum {
	FAULT_W_MAGIC,
	FAULT_W_KIND,     /* ESP_FAULT_* */
	FAULT_W_REASON,   /* k_fatal reason code (FATAL), 0 for WDT */
	FAULT_W_PC,       /* mepc of the faulting context, 0 if unknown */
	FAULT_W_UP_MS,
	FAULT_W_NAME0,    /* first 8 chars of the running thread's name */
	FAULT_W_NAME1,
	FAULT_W_CHECK,
	FAULT_NW
};

enum { KILL_W_MAGIC, KILL_W_KILLED, KILL_W_CHECK, KILL_NW };

#define ESP_FAULT_FATAL 1u
#define ESP_FAULT_WDT   2u

/* SNAP_W_STATE: the sender's flag byte, then one bit each for the kill latch
 * and link-up, then the re-arm hold. */
static inline uint32_t esp_snap_state(uint8_t flags, bool kill, bool link_up,
				      uint8_t hold)
{
	return (uint32_t)flags | ((uint32_t)kill << 8) | ((uint32_t)link_up << 9) |
	       ((uint32_t)(hold & 0x7u) << 12);
}

#endif /* RISKYBIRD_ESP_SAFETY_CORE_H */
