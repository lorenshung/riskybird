/*
 * uart1 (ESP uplink) receive path for the roll-harness build: an ISR-fed byte ring and the line
 * assembler that turns it into command lines.
 *
 * WHY. uart_cmd_poll() used to read the SiFive RX FIFO with uart_poll_in() once per control-loop
 * iteration. That FIFO is 8 entries (rocket-chip-blocks UARTParams.nRxEntries), 115200 baud fills
 * it in ~0.7 ms, and the rig loop measured 2.8 ms mean / 18 ms worst between polls. The RTL
 * enqueues every received byte with no ready check and has no overrun flag, so the bytes past the
 * eighth were dropped without trace: of 7 back-to-back GAIN lines one was applied, and a COLL sent
 * 5 ms after RESET never took (2026-11-04). The ISR now empties the FIFO within one byte time.
 *
 * Header-only and free of Zephyr so tests/test_fc_uart_rx.py can compile it on the host.
 *
 * Concurrency: exactly one producer (the uart1 RX ISR) and one consumer (the control loop).
 * head is written only by the producer, tail only by the consumer; each publishes with a release
 * store and reads the other's index with an acquire load. No lock, nothing blocks.
 */
#ifndef FC_UART_RX_H
#define FC_UART_RX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef FC_RX_RING_SIZE
#define FC_RX_RING_SIZE 512u   /* entries; a power of two. 512 B is ~44 ms of the link at 115200 */
#endif
#if (FC_RX_RING_SIZE & (FC_RX_RING_SIZE - 1u)) != 0
#error "FC_RX_RING_SIZE must be a power of two"
#endif

/* An entry is the received byte, plus this flag when bytes were lost IMMEDIATELY BEFORE it (the
 * ring was full). Out of band on purpose: no byte value can mean "loss", or line noise could
 * forge one. */
#define FC_RX_LOST 0x100u

#define FC_LINE_MAX 80u   /* line buffer incl. NUL: 79 characters fit, 80+ drop the line */

struct fc_rx_ring {
	uint32_t head;       /* next entry to write; producer only */
	uint32_t tail;       /* next entry to read; consumer only */
	uint32_t bytes;      /* bytes received, kept or not; producer only */
	uint32_t overruns;   /* bytes dropped because the ring was full; producer only */
	bool lost;           /* producer: a byte was dropped and the next entry must say so */
	uint16_t buf[FC_RX_RING_SIZE];
};

/* Producer (ISR). Never blocks: a full ring drops the byte and counts it. */
static inline void fc_rx_ring_put(struct fc_rx_ring *r, uint8_t byte)
{
	const uint32_t head = r->head;
	const uint32_t tail = __atomic_load_n(&r->tail, __ATOMIC_ACQUIRE);

	__atomic_store_n(&r->bytes, r->bytes + 1u, __ATOMIC_RELAXED);
	if (head - tail >= FC_RX_RING_SIZE) {
		__atomic_store_n(&r->overruns, r->overruns + 1u, __ATOMIC_RELAXED);
		r->lost = true;
		return;
	}
	r->buf[head & (FC_RX_RING_SIZE - 1u)] = (uint16_t)(byte | (r->lost ? FC_RX_LOST : 0u));
	r->lost = false;
	__atomic_store_n(&r->head, head + 1u, __ATOMIC_RELEASE);
}

/* Consumer. Returns an entry (byte | FC_RX_LOST), or -1 when the ring is empty. */
static inline int fc_rx_ring_get(struct fc_rx_ring *r)
{
	const uint32_t tail = r->tail;
	const uint32_t head = __atomic_load_n(&r->head, __ATOMIC_ACQUIRE);
	int entry;

	if (head == tail) {
		return -1;
	}
	entry = r->buf[tail & (FC_RX_RING_SIZE - 1u)];
	__atomic_store_n(&r->tail, tail + 1u, __ATOMIC_RELEASE);
	return entry;
}

/* Counters, readable from either side. */
static inline uint32_t fc_rx_ring_overruns(const struct fc_rx_ring *r)
{
	return __atomic_load_n(&r->overruns, __ATOMIC_RELAXED);
}

static inline uint32_t fc_rx_ring_bytes(const struct fc_rx_ring *r)
{
	return __atomic_load_n(&r->bytes, __ATOMIC_RELAXED);
}

/*
 * Line assembly: the rules uart_cmd_poll() has always applied to uart_poll_in() bytes.
 *   \n or \r ends a line; an empty line is ignored (so CRLF is one terminator).
 *   A non-printable byte is line noise: drop what was gathered and resync, so "<junk>ESTOP" is
 *   read as ESTOP (measured 2026-09-23: an ESP reset glitched its TX ahead of an ESTOP).
 *   A line longer than FC_LINE_MAX - 1 is dropped whole, through its newline.
 * One rule is new, for the ring: an entry flagged FC_RX_LOST drops the line in progress and
 * discards through the next newline, so the head and tail of a line whose middle was lost are
 * never joined ("EXCSTOP" must not become "STOP", nor "GAIN ATT_P 1.25" become "... 15"). If
 * the lost bytes held that line's newline too, the following line is dropped as well: losing a
 * good line is the safe error; the host sees it in lines_bad and resends.
 */
struct fc_line_asm {
	char buf[FC_LINE_MAX];
	size_t len;
	bool discarding;      /* inside an overlong or damaged line: wait for its newline */
	uint32_t lines_ok;    /* complete lines handed out (whether or not the verb is then known) */
	uint32_t lines_bad;   /* lines dropped: overlong, cut by noise, or damaged by a ring overrun */
};

/* Feed one ring entry. Returns the completed, NUL-terminated line (in a->buf, valid until the
 * next call), or NULL. */
static inline char *fc_line_feed(struct fc_line_asm *a, int entry)
{
	const unsigned ch = (unsigned)entry & 0xffu;

	if ((unsigned)entry & FC_RX_LOST) {
		if (a->len > 0 || !a->discarding) {
			a->lines_bad++;   /* the line the lost bytes belonged to */
		}
		a->len = 0;
		a->discarding = true;
	}
	if (ch == '\n' || ch == '\r') {
		const bool complete = a->len > 0 && !a->discarding;

		a->len = 0;
		a->discarding = false;
		if (complete) {
			a->lines_ok++;
			return a->buf;
		}
	} else if (ch < 0x20u || ch > 0x7eu) {
		if (a->len > 0) {
			a->lines_bad++;
		}
		a->len = 0;
		a->discarding = false;
	} else if (a->discarding) {
		/* rest of an overlong or damaged line: never dispatch its tail */
	} else if (a->len < FC_LINE_MAX - 1u) {
		a->buf[a->len++] = (char)ch;
		a->buf[a->len] = '\0';
	} else {
		a->lines_bad++;
		a->len = 0;
		a->discarding = true;
	}
	return NULL;
}

#endif /* FC_UART_RX_H */
