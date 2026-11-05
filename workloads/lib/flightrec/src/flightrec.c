/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * DRAM flight recorder. See include/riskybird/flightrec.h.
 */

#include <riskybird/flightrec.h>

#include <stddef.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

/* Ring size in samples. 32768 x 84 B = 2.6 MiB of DDR; at div 2 on a ~1.3 kHz
 * loop that is ~50 s. Readout over the ESP costs ~7 ms per sample (115200 baud),
 * which is why the host reads only the last segment, not the ring.
 * Plus 2 x PREROLL samples of preroll rings (168 KiB at the defaults).
 * Rig layout: 92 B samples, 2.9 MiB + 180 KiB. */
#ifndef FLIGHTREC_CAPACITY
#define FLIGHTREC_CAPACITY 32768u
#endif
#ifndef FLIGHTREC_DIV
#define FLIGHTREC_DIV 2u
#endif
#ifndef FLIGHTREC_TAIL_MS
#define FLIGHTREC_TAIL_MS 2000u
#endif
/* Samples before the first motor tick kept with each segment (~1.5 s at 650 Hz). */
#ifndef FLIGHTREC_PREROLL
#define FLIGHTREC_PREROLL 1000u
#endif

BUILD_ASSERT(sizeof(struct flightrec_sample) == 84,
	     "flightrec_sample layout is a contract with tools/rb/flightrec_fetch.py; "
	     "bump FLIGHTREC_VERSION and update the host when it changes");
BUILD_ASSERT(sizeof(struct flightrec_header) == 64,
	     "flightrec_header layout is a contract with tools/rb/flightrec_fetch.py");
BUILD_ASSERT(FLIGHTREC_DIV >= 1, "FLIGHTREC_DIV must be >= 1");
BUILD_ASSERT(FLIGHTREC_PREROLL < FLIGHTREC_CAPACITY, "preroll must fit in the ring");

/* Exported names so the host can find them in the ELF. g_flightrec_hdr starts
 * with struct flightrec_header in both layouts; HDR points at that part. */
#if FLIGHTREC_RIG
BUILD_ASSERT(sizeof(struct flightrec_rig_sample) == 92,
	     "flightrec_rig_sample layout is a contract with tools/rb/flightrec_fetch.py");
BUILD_ASSERT(sizeof(struct flightrec_gain_change) == 12,
	     "flightrec_gain_change layout is a contract with tools/rb/flightrec_fetch.py");
BUILD_ASSERT(sizeof(struct flightrec_rig_header) == 64 + 68 + 12 * FLIGHTREC_GAIN_RING,
	     "flightrec_rig_header layout is a contract with tools/rb/flightrec_fetch.py");
BUILD_ASSERT(offsetof(struct flightrec_rig_header, rig) == sizeof(struct flightrec_header),
	     "the v3 extension follows the v2 header directly");

struct flightrec_rig_header g_flightrec_hdr;
#define HDR (&g_flightrec_hdr.h)
#define HDR_VERSION FLIGHTREC_VERSION_RIG

static const float *rig_gains;
static const volatile uint32_t *rig_gain_seq;

/* The live gains into the header snapshot, tagged with the segment they open. */
static void rig_snapshot(void)
{
	struct flightrec_rig_ext *x = &g_flightrec_hdr.rig;

	if (rig_gains == NULL) {
		return;
	}
	for (uint32_t i = 0; i < FLIGHTREC_RIG_GAINS; i++) {
		x->snap_gains[i] = rig_gains[i];
	}
	x->snap_gain_seq = *rig_gain_seq;
	x->snap_segment = HDR->segments;
}

void flightrec_rig_bind(const float *gains, const volatile uint32_t *gain_seq)
{
	rig_gains = gains;
	rig_gain_seq = gain_seq;
	rig_snapshot();
}

void flightrec_rig_gain(uint32_t gain_seq, unsigned index, float value, uint32_t t_us)
{
	struct flightrec_rig_ext *x = &g_flightrec_hdr.rig;
	struct flightrec_gain_change *c = &x->changes[x->gain_count % FLIGHTREC_GAIN_RING];

	c->gain_seq = (uint16_t)gain_seq;
	c->index = (uint8_t)index;
	c->pad = 0;
	c->value = value;
	c->t_us = t_us;
	/* The entry is written before the count that publishes it. */
	compiler_barrier();
	x->gain_count++;
}
#else
struct flightrec_header g_flightrec_hdr;
#define HDR (&g_flightrec_hdr)
#define HDR_VERSION FLIGHTREC_VERSION
#endif
flightrec_rec_t g_flightrec_samples[FLIGHTREC_CAPACITY];
flightrec_rec_t g_flightrec_pre[2][FLIGHTREC_PREROLL];

static uint32_t calls;
static bool was_active;
static int64_t idle_since_ms;
static uint32_t idle_buf;      /* preroll ring frozen idle ticks go to */
static uint32_t idle_written;  /* samples written to it this frozen spell */
static uint32_t main_since;    /* count at the last unfreeze: the main ring is
				* contiguous only from here */

void flightrec_init(void)
{
	struct flightrec_header *h = HDR;

	h->magic = FLIGHTREC_MAGIC;
	h->version = HDR_VERSION;
	h->sample_size = (uint16_t)sizeof(flightrec_rec_t);
	h->capacity = FLIGHTREC_CAPACITY;
	h->div = FLIGHTREC_DIV;
	h->tail_ms = FLIGHTREC_TAIL_MS;
	h->preroll = FLIGHTREC_PREROLL;
	h->samples_addr = (uint32_t)(uintptr_t)&g_flightrec_samples[0];
	h->pre_addr = (uint32_t)(uintptr_t)&g_flightrec_pre[0][0];
	h->pre_buf = 0;
	h->pre_head = 0;
	h->pre_count = 0;
	h->count = 0;
	h->seg_start = 0;
	h->seg_end = 0;
	h->segments = 0;
	h->frozen = 0;
	calls = 0;
	was_active = false;
	idle_since_ms = 0;
	idle_buf = 0;
	idle_written = 0;
	main_since = 0;
#if FLIGHTREC_RIG
	g_flightrec_hdr.rig.layout = FLIGHTREC_LAYOUT_RIG;
	g_flightrec_hdr.rig.n_gains = FLIGHTREC_RIG_GAINS;
	g_flightrec_hdr.rig.gain_ring = FLIGHTREC_GAIN_RING;
	g_flightrec_hdr.rig.gain_count = 0;
	rig_snapshot();
#endif
}

void flightrec_tick(const flightrec_rec_t *s, bool motors_active)
{
	struct flightrec_header *h = HDR;

	if (motors_active && !was_active) {
		if (h->frozen) {
			/* The main ring stopped at the last freeze: the preroll is
			 * the idle ring. Hand it to this segment; the next frozen
			 * spell writes the other one. */
			uint32_t n = MIN(idle_written, (uint32_t)FLIGHTREC_PREROLL);

			h->pre_buf = idle_buf;
			h->pre_count = n;
			h->pre_head = (idle_written - n) % FLIGHTREC_PREROLL;
			h->seg_start = h->count;
			main_since = h->count;
			idle_buf ^= 1u;
		} else {
			/* Still recording (first segment, or a restart inside the
			 * tail): the preroll is in the main ring, but only back to
			 * the last unfreeze -- before that it holds an older flight. */
			uint32_t pre = MIN(h->count - main_since,
					   (uint32_t)FLIGHTREC_PREROLL);

			h->seg_start = h->count - pre;
			if (h->seg_start != main_since) {
				h->pre_count = 0;
			}
			/* else the previous segment's preroll ring still runs
			 * straight into main_since; keep it. */
		}
		h->seg_end = 0;
		h->segments++;
		h->frozen = 0;
#if FLIGHTREC_RIG
		rig_snapshot();
#endif
	}
	if (!motors_active && was_active) {
		idle_since_ms = k_uptime_get();
	}
	was_active = motors_active;

	if (!h->frozen && !motors_active && h->segments > 0 &&
	    k_uptime_get() - idle_since_ms >= (int64_t)FLIGHTREC_TAIL_MS) {
		h->seg_end = h->count;
		h->frozen = 1;
		idle_written = 0;
	}
	if ((calls++ % FLIGHTREC_DIV) != 0u) {
		return;
	}
	if (h->frozen) {
		g_flightrec_pre[idle_buf][idle_written % FLIGHTREC_PREROLL] = *s;
		idle_written++;
		return;
	}
	g_flightrec_samples[h->count % FLIGHTREC_CAPACITY] = *s;
	/* The sample is written before the count that publishes it. */
	compiler_barrier();
	h->count++;
}

const struct flightrec_header *flightrec_header(void)
{
	return HDR;
}
