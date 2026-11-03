/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Remote program loading: host <-> (WiFi TCP | USB tunnel) <-> this ESP <->
 * FPGA link <-> UART-TSI client in the SoC.
 *
 * The FPGA side is EspLinkMux (backends/chipyard/generators/chipyard/src/main/
 * scala/esplink/EspLinkMux.scala, built into RocketArty200TDroneFullDDRLoaderConfig).
 * It gives the two link wires to uart1 normally, and to a hardware TSI loader
 * when this ESP sends a >= 2 ms break followed by an 8-byte key. It answers
 * "RBLK", holds the SoC in reset for 10 ms (parking hart 0 in the boot ROM),
 * then routes the wires to TSI until 2 s pass with no traffic either way.
 *
 * SESSION PROTOCOL (identical on both transports). The host sends one line:
 *
 *   "RBLOAD1"            reset the SoC, then TSI (program loading)
 *   "RBPOKE1"            TSI with NO reset: read/write memory under the running
 *                        program (the remote commit). Allowed only while this
 *                        ESP knows TSI's word assembly is at a command boundary.
 *   "RBPING1"            which bitstream is answering; touches nothing
 *   "RBFLASH1 [FAST]"    configuration-flash bridge (EspLinkFlash framing),
 *                        FAST = 921600 baud on the wire after the ack
 *   "RBBOOT1 <hex addr> [mode]" reconfigure the FPGA from that flash address
 *                       (IPROG). mode 1-3 are ICAP diagnostics (EspLinkIprog):
 *                       1 IPROG without writing WBSTAR, 2 write WBSTAR only
 *                       (the FPGA keeps running), 3 Vivado's packed layout.
 *                        Never run a FLASH session in the same breath first:
 *                        on 2026-11-01 an IPROG issued ~0.3 s after one was
 *                        ignored (the running program survived), while the
 *                        same IPROG seconds after a FLASH session worked.
 *   "RBSTAT1"            the ESP's RBLINK line (motor-frame flags, e.g.
 *                        COMMITTED); never touches the FPGA link
 *
 *   ESP   -> "RB<KIND> OK id=<8 hex>\n"    the FPGA answered; id is its
 *                                           USR_ACCESS build timestamp
 *         or "RB<KIND> ERR <reason>\n"     and the session is over
 *   then, for LOAD/POKE: TSI commands and responses (below); for FLASH: flash
 *   frames and replies; PING and BOOT end after the reply.
 *
 * Every session is refused while, or for 1 s after, any motor gate carried a
 * nonzero duty: a session takes the wire the motor frames use.
 *
 * THE ESP PARSES THE TSI COMMAND STREAM, and forwards only well-formed
 * commands. A TSI header is 20 bytes, little-endian: u32 cmd, u64 addr,
 * u64 len (32-bit words minus one); a write carries (len + 1) * 4 payload
 * bytes. Commands other than read (0) and write (1) are never forwarded:
 * TSI is a bus master, and a stray byte reaching it -- a restarted host
 * re-sending its handshake mid-session, a half-dead connection -- would
 * misalign every following word into writes at addresses nobody chose. So an
 * unknown command ends the session instead. The one local command:
 *
 *   cmd 0xE5D0E5D0  END: stop forwarding; nothing more is sent to TSI
 *
 * Header boundaries are also what makes END possible at all on the USB tunnel,
 * which has no connection to close.
 *
 * FLASH frames are parsed the same way and forwarded only whole: 0x01 XFER
 * (u16 wlen <= 512, u16 rlen, wlen bytes), 0x02 PROG (u32 addr, 256 bytes),
 * 0x03 ERASE (u32 addr), and the local 0xE5 END. The FPGA refuses to program
 * or erase below the golden-image guard whatever arrives here.
 *
 * TSI ALIGNMENT. TSI assembles 32-bit words with no timeout, and only a LOAD
 * (SoC reset) restarts it. A POKE session does not reset, so it is allowed
 * only when every byte this ESP has sent TSI since the last LOAD formed whole
 * commands: tsi_synced. POKE commands are buffered whole (<= 1 KiB payload)
 * before the first byte is forwarded, so even a host that vanishes mid-command
 * leaves TSI aligned; a LOAD that ends mid-command clears tsi_synced. (On
 * bitstreams before EspLinkRxFilter the closing break itself leaked into TSI;
 * those do not answer the POKE key at all.)
 *
 * WHILE A SESSION IS ACTIVE (load_link_active()) the motors read zero, the
 * uplink refuses to write the FPGA UART, and every FPGA byte goes to the
 * session. A session ends with a bare break (no key), which takes the mux out
 * of TSI; it stays active until the mux is back on uart1 (DRAIN_MS) before
 * anything else writes. The break's first 2 ms -- before the mux has
 * recognised it -- still reach TSI as one framing-error byte. That cannot
 * complete a command: no further byte reaches TSI until the next session,
 * whose handshake resets the SoC and with it TSI.
 *
 * USB TUNNEL. The USB serial/JTAG unit is also the console. During a USB
 * session the log backends are switched off and this file's own prints are
 * deferred, but anything that bypasses logging (a ROM print, a panic) would
 * land in the FPGA->host stream. That direction carries only read responses,
 * so corruption cannot write anything; the host verifies every byte it loaded
 * by reading it back, which is where it would show up.
 */

#include "load_link.h"
#include "load_link_internal.h"

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/net/socket.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log_backend.h>
#include <hal/uart_ll.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdlib.h>

/* The break is made by inverting the UART0 TX signal, so the FPGA link must be
 * UART0. (GPIO16 is UART0's TX; see esp_bridge.overlay.) */
BUILD_ASSERT(DT_SAME_NODE(DT_ALIAS(fpga_uart), DT_NODELABEL(uart0)),
	     "load_link drives the break through UART0's TX inversion");
#define FPGA_UART_HW (&UART0)

#define BREAK_MS          20    /* FPGA threshold is 2 ms */
#define ACK_TIMEOUT_MS    100
#define RESET_SETTLE_MS   25    /* FPGA holds SoC reset 10 ms after the ack */
#define FPGA_IDLE_EXIT_MS 2000  /* EspLinkMux idleMs: its fallback exit */
#define FPGA_KEY_WINDOW_MS 200  /* EspLinkMux keyWindowMs */
/* After a session the ESP sends a bare break: from TSI the mux goes to KEY,
 * hears no key, and is back on uart1 when the key window expires -- 220 ms
 * instead of the 2 s idle exit, which matters because the program just started
 * talks on uart1 from its first instruction (telemetry, motor frames) and the
 * pins are not uart1's until the mux leaves TSI. The idle exit stays as the
 * fallback; DRAIN_MS covers whichever applies. */
#define DRAIN_MS          (BREAK_MS + FPGA_KEY_WINDOW_MS + 80)
#define FALLBACK_DRAIN_MS (FPGA_IDLE_EXIT_MS + 500)
#define STALL_ABORT_MS    5000  /* no traffic either way: the host is gone */
#define HELLO_TIMEOUT_MS  2000
#define HELLO_LINE_MAX          32

#define TSI_CMD_READ  0u
#define TSI_CMD_WRITE 1u
#define TSI_CMD_END   0xE5D0E5D0u
#define TSI_HDR_LEN   20
/* Largest write accepted in one command: 4 MiB. The host sends 1 KiB chunks;
 * anything near this is a corrupt header, and (len + 1) * 4 must not wrap. */
#define TSI_MAX_LEN_WORDS (1u << 20)

static const uint8_t KEY_PREFIX[7] = { 0x52, 0x42, 0x4c, 0x44, 0xa5, 0x5a, 0xc3 };

struct kind_info {
	const char *hello;     /* line prefix */
	const char *name;      /* reply prefix */
	uint8_t sel;           /* last key byte */
	char ack[4];
};
static const struct kind_info KINDS[] = {
	[K_LOAD]  = { "RBLOAD1",  "RBLOAD",  0x3c, { 'R', 'B', 'L', 'K' } },
	[K_POKE]  = { "RBPOKE1",  "RBPOKE",  0x3d, { 'R', 'B', 'P', 'K' } },
	[K_PING]  = { "RBPING1",  "RBPING",  0x3d, { 'R', 'B', 'P', 'K' } },
	[K_FLASH] = { "RBFLASH1", "RBFLASH", 0x3e, { 'R', 'B', 'F', 'L' } },
	[K_BOOT]  = { "RBBOOT1",  "RBBOOT",  0x3f, { 'R', 'B', 'B', 'T' } },
	[K_STAT]  = { "RBSTAT1",  "RBSTAT",  0x00, { 0 } },   /* ESP only: no FPGA */
	[K_STASH] = { "RBSTASH1", "RBSTASH", 0x00, { 0 } },   /* persist.c */
	[K_POLICY] = { "RBPOLICY1", "RBPOLICY", 0x00, { 0 } },
	[K_AUTO]  = { "RBAUTO1",  "RBAUTO",  0x00, { 0 } },
};
#define SEL_FLASH_FAST 0x3b
#define FLASH_FAST_BAUD 921600
#define LINK_BAUD 115200


#define FL_XFER  0x01
#define FL_PROG  0x02
#define FL_ERASE 0x03
#define FL_CRC   0x04
#define FL_END   0xE5
#define FL_MAX_WLEN 512
#define POKE_MAX_PAYLOAD 1024

static bool tsi_synced;

static const struct device *fpga;
static const struct device *usb;
static atomic_t active;
static K_MUTEX_DEFINE(tx_lock);

/* 16 KiB: a WiFi retransmit stall of ~1.4 s at the link rate before a byte drops. */
static uint8_t from_fpga_buf[16384];
static struct ring_buf from_fpga;
static atomic_t from_fpga_dropped;

bool load_link_active(void)
{
	return atomic_get(&active) != 0;
}

void load_link_rx_from_fpga(const uint8_t *buf, size_t n)
{
	if (ring_buf_put(&from_fpga, buf, n) != n) {
		atomic_inc(&from_fpga_dropped);
	}
}

bool load_link_tx_trylock(void)
{
	/* A session holds the lock for its whole length, so other writers must
	 * not wait on it: 50 ms covers one in-flight uplink line (~11 ms for 128
	 * bytes at 115200), and a timeout means a session has the wire. */
	if (load_link_active()) {
		return false;
	}
	if (k_mutex_lock(&tx_lock, K_MSEC(50)) != 0) {
		return false;
	}
	if (load_link_active()) {
		k_mutex_unlock(&tx_lock);
		return false;
	}
	return true;
}

void load_link_tx_unlock(void)
{
	k_mutex_unlock(&tx_lock);
}

/* The UART ISR puts into from_fpga whenever a session is active; Zephyr's
 * ring buffer is lock-free for one producer and one consumer, and a reset is
 * neither, so it happens with interrupts masked. */
static void from_fpga_reset(void)
{
	unsigned int key = irq_lock();

	ring_buf_reset(&from_fpga);
	irq_unlock(key);
}

static void usb_flush(void);

/* ---- transports ---------------------------------------------------------- */

struct transport {
	const char *name;
	int sock;          /* TCP, or -1 */
	bool is_usb;
};

/* > 0 bytes, 0 nothing available now, < 0 the host is gone. */
static int t_recv(struct transport *t, uint8_t *buf, size_t max)
{
	if (t->is_usb) {
		size_t n = 0;
		unsigned char c;

		while (n < max && uart_poll_in(usb, &c) == 0) {
			buf[n++] = c;
		}
		return (int)n;
	}
	int r = zsock_recv(t->sock, buf, max, ZSOCK_MSG_DONTWAIT);

	if (r > 0) {
		return r;
	}
	if (r == 0) {
		return -1;     /* orderly close */
	}
	return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
}

static int last_send_errno;

static int t_send(struct transport *t, const uint8_t *buf, size_t n)
{
	if (t->is_usb) {
		for (size_t i = 0; i < n; i++) {
			uart_poll_out(usb, buf[i]);
		}
		return 0;
	}
	/* Never block indefinitely on a half-dead connection: the session holds
	 * the FPGA wire (telemetry diverted, uplink refused) for as long as it runs. */
	int64_t give_up = k_uptime_get() + STALL_ABORT_MS;

	while (n > 0) {
		int r = zsock_send(t->sock, buf, n, ZSOCK_MSG_DONTWAIT);

		if (r > 0) {
			buf += r;
			n -= (size_t)r;
			continue;
		}
		if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
			last_send_errno = errno;
			return -1;
		}
		if (k_uptime_get() > give_up) {
			return -1;
		}
		struct zsock_pollfd pfd = { .fd = t->sock, .events = ZSOCK_POLLOUT };

		(void)zsock_poll(&pfd, 1, 100);
	}
	return 0;
}

static void t_say(struct transport *t, const char *s)
{
	(void)t_send(t, (const uint8_t *)s, strlen(s));
}

/* ---- console quieting for the USB tunnel --------------------------------- */

#define MAX_BACKENDS 4
static bool backend_was_active[MAX_BACKENDS];

static void logs_quiet(bool quiet)
{
	int n = log_backend_count_get();

	for (int i = 0; i < n && i < MAX_BACKENDS; i++) {
		const struct log_backend *b = log_backend_get(i);

		if (quiet) {
			backend_was_active[i] = log_backend_is_active(b);
			if (backend_was_active[i]) {
				log_backend_deactivate(b);
			}
		} else if (backend_was_active[i]) {
			log_backend_activate(b, b->cb->ctx);
		}
	}
	if (quiet) {
		k_msleep(20);  /* let a line already being written finish */
	}
}

/* ---- FPGA handshake ------------------------------------------------------- */

/* One byte to the FPGA, SLEEPING while the 128-byte TX FIFO is full. The
 * driver's uart_poll_out() spins instead, and this thread outranks the WiFi
 * task: a load spinning for minutes starved it until the SoftAP stopped
 * beaconing, TCP stalled for more than the mux's 2 s idle exit, and the rest
 * of the program went to uart1 instead of TSI. A full FIFO is 11 ms of
 * transmission, so a 1 ms sleep never lets the line go idle. */
static void fpga_putc(uint8_t c)
{
	while (uart_ll_get_txfifo_len(FPGA_UART_HW) == 0) {
		k_msleep(1);
	}
	uart_poll_out(fpga, c);
}

static bool send_break(void)
{
	int64_t give_up = k_uptime_get() + 50;

	/* Never cut a byte in half: wait until the shifter is empty. */
	while (!uart_ll_is_tx_idle(FPGA_UART_HW)) {
		if (k_uptime_get() > give_up) {
			return false;
		}
		k_busy_wait(50);
	}
	uart_ll_inverse_signal(FPGA_UART_HW, UART_SIGNAL_TXD_INV);
	k_msleep(BREAK_MS);
	uart_ll_inverse_signal(FPGA_UART_HW, 0);
	k_msleep(1);   /* idle high before the key's first start bit */
	return true;
}

/* The ack, then (bitstreams with EspLinkRxFilter onwards) four id bytes MSB
 * first. *id_valid is false for an older loader, which sends the ack alone. */
static bool wait_ack(const char ack[4], uint32_t *id, bool *id_valid)
{
	int64_t deadline = k_uptime_get() + ACK_TIMEOUT_MS;
	size_t matched = 0;

	*id_valid = false;
	while (k_uptime_get() < deadline) {
		uint8_t c;

		if (ring_buf_get(&from_fpga, &c, 1) != 1) {
			k_msleep(1);
			continue;
		}
		matched = (c == (uint8_t)ack[matched]) ? matched + 1 :
			  (c == (uint8_t)ack[0] ? 1 : 0);
		if (matched == 4) {
			break;
		}
	}
	if (matched != 4) {
		return false;
	}
	/* Four more bytes take 350 us at 115200; allow 5 ms. */
	uint8_t idb[4];
	size_t n = 0;
	int64_t id_deadline = k_uptime_get() + 5;

	while (n < 4 && k_uptime_get() <= id_deadline) {
		n += ring_buf_get(&from_fpga, &idb[n], 4 - n);
		if (n < 4) {
			k_busy_wait(200);
		}
	}
	if (n == 4) {
		*id = sys_get_be32(idb);
		*id_valid = true;
	}
	return true;
}

static void set_link_baud(uint32_t baud)
{
	struct uart_config cfg;

	if (uart_config_get(fpga, &cfg) == 0 && cfg.baudrate != baud) {
		cfg.baudrate = baud;
		(void)uart_configure(fpga, &cfg);
	}
}

/* ---- the session ----------------------------------------------------------- */

/* A LOAD pump that stops with a write's payload half sent leaves TSI misaligned. */
static bool pump_tsi_mid_command;

struct session_stats {
	uint32_t cmds;
	uint32_t to_fpga;
	uint32_t to_host;
	const char *end;
};

static void fpga_write(const uint8_t *b, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		fpga_putc(b[i]);
	}
}

/* Reply accounting for a session. (A keepalive that injected its own
 * commands while idle was tried and removed: replies came back shifted. The
 * mux's 2 s idle exit therefore ends a session if the host goes quiet that
 * long; with the WiFi heap sized properly, a load does not.) */
struct wire {
	uint32_t outstanding;   /* reply bytes the host is still owed */
	uint32_t swallow;       /* keepalive reply bytes to drop */
	int64_t last;           /* last byte either way */
};

/* Forward FPGA -> host; false if the host is gone. */
static bool drain_to_host(struct transport *t, struct session_stats *st, bool *progress,
			  struct wire *w)
{
	static uint8_t out[1024];   /* larger sends: fewer, fuller TCP segments */
	uint32_t m = ring_buf_get(&from_fpga, out, sizeof(out));
	uint32_t skip = 0;

	if (m == 0u) {
		return true;
	}
	*progress = true;
	w->last = k_uptime_get();
	if (w->swallow > 0u) {
		skip = MIN(w->swallow, m);
		w->swallow -= skip;
	}
	if (m > skip) {
		st->to_host += m - skip;
		w->outstanding -= MIN(w->outstanding, m - skip);
		if (t_send(t, &out[skip], m - skip) < 0) {
			static char why[48];

			snprintk(why, sizeof(why), "host send failed (errno %d)", last_send_errno);
			st->end = why;
			return false;
		}
	}
	return true;
}

/* POKE: each command is collected whole (header + payload) and only then
 * forwarded, so nothing the host does can leave TSI mid-command. */
static void pump_poke(struct transport *t, struct session_stats *st)
{
	static uint8_t cmdbuf[TSI_HDR_LEN + POKE_MAX_PAYLOAD];
	size_t have = 0, need = TSI_HDR_LEN;
	int64_t last_traffic = k_uptime_get();
	struct wire w = { .last = k_uptime_get() };

	for (;;) {
		bool progress = false;
		int n = t_recv(t, &cmdbuf[have], need - have);

		if (n < 0) {
			st->end = have == 0 ? "host closed" : "host closed mid-command (nothing of it sent)";
			return;
		}
		if (n > 0) {
			progress = true;
			have += (size_t)n;
		}
		if (have == TSI_HDR_LEN && need == TSI_HDR_LEN) {
			uint32_t cmd = sys_get_le32(&cmdbuf[0]);
			uint64_t len = sys_get_le64(&cmdbuf[12]);

			if (cmd == TSI_CMD_END) {
				st->end = "END";
				return;
			}
			if ((cmd != TSI_CMD_READ && cmd != TSI_CMD_WRITE) ||
			    (cmd == TSI_CMD_WRITE && (len + 1u) * 4u > POKE_MAX_PAYLOAD) ||
			    len >= TSI_MAX_LEN_WORDS) {
				st->end = "REJECTED (poke takes reads, and writes <= 1 KiB)";
				return;
			}
			need = TSI_HDR_LEN + (cmd == TSI_CMD_WRITE ? (size_t)(len + 1u) * 4u : 0u);
		}
		if (have == need && have >= TSI_HDR_LEN) {
			fpga_write(cmdbuf, have);
			if (sys_get_le32(&cmdbuf[0]) == TSI_CMD_READ) {
				w.outstanding += (uint32_t)(sys_get_le64(&cmdbuf[12]) + 1u) * 4u;
			}
			w.last = k_uptime_get();
			st->to_fpga += have;
			st->cmds++;
			have = 0;
			need = TSI_HDR_LEN;
		}
		if (!drain_to_host(t, st, &progress, &w)) {
			return;
		}

		if (progress) {
			last_traffic = k_uptime_get();
		} else if (k_uptime_get() - last_traffic > STALL_ABORT_MS) {
			st->end = "stalled (no traffic for 5 s)";
			return;
		} else {
			k_msleep(1);
		}
	}
}

/* FLASH: whole frames only, as for POKE. */
static void pump_flash(struct transport *t, struct session_stats *st)
{
	static uint8_t fr[5 + FL_MAX_WLEN + 256];
	size_t have = 0, need = 1;
	int64_t last_traffic = k_uptime_get();
	struct wire w = { .last = k_uptime_get() };

	for (;;) {
		bool progress = false;
		int n = t_recv(t, &fr[have], need - have);

		if (n < 0) {
			st->end = have == 0 ? "host closed" : "host closed mid-frame (nothing of it sent)";
			return;
		}
		if (n > 0) {
			progress = true;
			have += (size_t)n;
		}
		if (have == 1 && need == 1) {
			switch (fr[0]) {
			case FL_END:   st->end = "END"; return;
			case FL_XFER:  need = 5; break;
			case FL_PROG:  need = 5 + 256; break;
			case FL_ERASE: need = 5; break;
			case FL_CRC:   need = 9; break;
			default:       st->end = "REJECTED an unknown flash frame"; return;
			}
		}
		if (have == 5 && need == 5 && fr[0] == FL_XFER) {
			uint16_t wlen = sys_get_le16(&fr[1]);

			if (wlen == 0 || wlen > FL_MAX_WLEN) {
				st->end = "REJECTED an XFER length";
				return;
			}
			need = 5 + wlen;
		}
		if (have == need && have >= 5) {
			fpga_write(fr, have);
			w.outstanding += fr[0] == FL_XFER ? (uint32_t)sys_get_le16(&fr[3]) + 1u :
					 fr[0] == FL_CRC ? 5u : 1u;
			w.last = k_uptime_get();
			st->to_fpga += have;
			st->cmds++;
			have = 0;
			need = 1;
		}
		if (!drain_to_host(t, st, &progress, &w)) {
			return;
		}

		if (progress) {
			last_traffic = k_uptime_get();
		} else if (k_uptime_get() - last_traffic > STALL_ABORT_MS) {
			st->end = "stalled (no traffic for 5 s)";
			return;
		} else {
			k_msleep(1);
		}
	}
}

static void pump(struct transport *t, struct session_stats *st)
{
	uint8_t hdr[TSI_HDR_LEN];
	size_t hn = 0;
	uint64_t payload_left = 0;
	int64_t last_traffic = k_uptime_get();
	struct wire w = { .last = k_uptime_get() };

	pump_tsi_mid_command = false;

	for (;;) {
		uint8_t in[128];
		bool progress = false;
		int n = t_recv(t, in, sizeof(in));

		if (n < 0) {
			st->end = (hn == 0 && payload_left == 0) ? "host closed" :
				  "host closed MID-COMMAND";
			pump_tsi_mid_command = payload_left > 0;
			return;
		}
		for (int i = 0; i < n; i++) {
			uint8_t b = in[i];

			progress = true;
			if (payload_left > 0) {
				fpga_putc(b);
				payload_left--;
				st->to_fpga++;
				w.last = k_uptime_get();
				continue;
			}
			hdr[hn++] = b;
			if (hn < TSI_HDR_LEN) {
				continue;
			}
			hn = 0;

			uint32_t cmd = sys_get_le32(&hdr[0]);
			uint64_t len = sys_get_le64(&hdr[12]);

			if (cmd == TSI_CMD_END) {
				st->end = "END";
				return;
			}
			if (cmd != TSI_CMD_READ && cmd != TSI_CMD_WRITE) {
				st->end = "REJECTED a non-TSI command";
				return;
			}
			if (len >= TSI_MAX_LEN_WORDS) {
				st->end = "REJECTED an oversized TSI command";
				return;
			}
			for (size_t k = 0; k < TSI_HDR_LEN; k++) {
				fpga_putc(hdr[k]);
			}
			st->to_fpga += TSI_HDR_LEN;
			st->cmds++;
			payload_left = (cmd == TSI_CMD_WRITE) ? (len + 1u) * 4u : 0u;
			if (cmd == TSI_CMD_READ) {
				w.outstanding += (uint32_t)(len + 1u) * 4u;
			}
			w.last = k_uptime_get();
		}

		if (!drain_to_host(t, st, &progress, &w)) {
			pump_tsi_mid_command = payload_left > 0;
			return;
		}

		if (progress) {
			last_traffic = k_uptime_get();
		} else if (k_uptime_get() - last_traffic > STALL_ABORT_MS) {
			st->end = "stalled (no traffic for 5 s)";
			pump_tsi_mid_command = payload_left > 0;
			return;
		} else {
			k_msleep(1);
		}
	}
}

static void say_kind(struct transport *t, const struct request *rq, const char *fmt, ...)
{
	char buf[200];
	int n = snprintk(buf, sizeof(buf), "%s ", KINDS[rq->kind].name);
	va_list ap;

	va_start(ap, fmt);
	n += vsnprintk(&buf[n], sizeof(buf) - n, fmt, ap);
	va_end(ap);
	t_say(t, buf);
}

static void session(struct transport *t, const struct request *rq)
{
	struct session_stats st = { .end = "?" };
	const struct kind_info *k = &KINDS[rq->kind];
	const char *err = NULL;
	int64_t t0 = k_uptime_get();
	uint32_t id = 0;
	bool id_valid = false;

	if (rq->kind == K_STASH || rq->kind == K_POLICY || rq->kind == K_AUTO) {
		persist_session(t->is_usb ? NULL : &t->sock, t->is_usb, rq);
		return;
	}
	if (rq->kind == K_STAT) {
		char line[256];

		(void)esp_bridge_status(line, sizeof(line));
		say_kind(t, rq, "OK %s", line);
		return;
	}
	if (esp_bridge_motors_recently_on()) {
		say_kind(t, rq, "ERR motors-running: a gate carried a nonzero duty in the last "
			 "second; a session would cut the motor link. Land and disarm first.\n");
		printk("load_link: %s %s refused: motors running\n", t->name, k->name);
		return;
	}
	if (rq->kind == K_POKE && !tsi_synced) {
		say_kind(t, rq, "ERR tsi-unsynced: TSI may be mid-command since the last load "
			 "(or this ESP restarted); poke needs a fresh load first\n");
		return;
	}

	k_mutex_lock(&tx_lock, K_FOREVER);   /* waits out an in-flight uplink line */
	from_fpga_reset();
	atomic_set(&from_fpga_dropped, 0);
	atomic_set(&active, 1);     /* motors 0, uplink refused, RX diverted */
	if (t->is_usb) {
		logs_quiet(true);
	}
	k_msleep(10);               /* > one motor-link tick: gates are at 0 */


	if (!send_break()) {
		err = "ERR esp-uart-busy\n";
	} else {
		from_fpga_reset();
		fpga_write(KEY_PREFIX, sizeof(KEY_PREFIX));
		uint8_t sel = (rq->kind == K_FLASH && rq->fast) ? SEL_FLASH_FAST : k->sel;

		fpga_putc(sel);
		if (rq->kind == K_BOOT) {
			uint8_t arg[8];

			sys_put_le32(rq->addr, &arg[0]);
			sys_put_le32(~rq->addr ^ (0x01010101u * rq->mode), &arg[4]);
			fpga_write(arg, sizeof(arg));
		}
		if (!wait_ack(k->ack, &id, &id_valid)) {
			err = rq->kind == K_LOAD ?
				"ERR no-ack: the FPGA did not answer the loader key "
				"(bitstream without the ESP loader, or link down)\n" :
				"ERR no-ack: no answer to this session's key (a bitstream "
				"without the ESP loader, or a loader older than ping/poke/flash/boot)\n";
		}
	}

	char idtxt[16];

	if (id_valid) {
		snprintk(idtxt, sizeof(idtxt), "%08x", id);
	} else {
		strcpy(idtxt, "unknown");
	}

	if (err != NULL) {
		say_kind(t, rq, "%s", err);
		st.end = "handshake failed";
		/* A non-loader bitstream's uart1 just saw a break and junk bytes; a
		 * loader bitstream that missed the key is back on uart1 already. */
		k_msleep(300);
	} else {
		switch (rq->kind) {
		case K_LOAD:
			k_msleep(RESET_SETTLE_MS);
			from_fpga_reset();
			tsi_synced = true;            /* the SoC reset restarted TSI */
			say_kind(t, rq, "OK id=%s\n", idtxt);
			pump(t, &st);
			if (pump_tsi_mid_command) {
				tsi_synced = false;
			}
			break;
		case K_POKE:
			k_msleep(2);
			from_fpga_reset();
			say_kind(t, rq, "OK id=%s\n", idtxt);
			pump_poke(t, &st);
			break;
		case K_PING:
			say_kind(t, rq, "OK id=%s\n", idtxt);
			st.end = "ping";
			break;
		case K_FLASH:
			if (rq->fast) {
				set_link_baud(FLASH_FAST_BAUD);
			}
			k_msleep(2);
			from_fpga_reset();
			say_kind(t, rq, "OK id=%s%s\n", idtxt, rq->fast ? " baud=921600" : "");
			pump_flash(t, &st);
			break;
		case K_BOOT:
			if (rq->mode == 2) {
				say_kind(t, rq, "OK id=%s; WBSTAR <- 0x%08x (mode 2, no IPROG)\n",
					 idtxt, rq->addr);
			} else {
				say_kind(t, rq, "OK id=%s; reconfiguring from 0x%08x (mode %u)\n",
					 idtxt, rq->addr, rq->mode);
			}
			st.end = "IPROG";
			/* The FPGA reconfigures now: its pins float, then a new image
			 * starts on uart1. Anything heard meanwhile is noise. A load or
			 * poke after this needs a fresh LOAD first. */
			tsi_synced = false;
			k_msleep(500);
			break;
		default:
			break;   /* answered before the handshake */
		}
		if (rq->kind == K_LOAD || rq->kind == K_POKE || rq->kind == K_FLASH) {
			/* Why it ended, for a host that is still reading (after the
			 * binary stream; tools print it on failure). */
			say_kind(t, rq, "END %s\n", st.end);
		}
		if (rq->kind != K_BOOT) {
			/* Hand the pins back to uart1: bare break, then wait out the
			 * key window. Without the break, fall back to the idle exit. */
			if (send_break()) {
				k_msleep(DRAIN_MS);
			} else {
				k_msleep(FALLBACK_DRAIN_MS);
			}
		}
		set_link_baud(LINK_BAUD);
	}

	from_fpga_reset();
	atomic_set(&active, 0);
	if (t->is_usb) {
		logs_quiet(false);
	}
	k_mutex_unlock(&tx_lock);
	usb_flush();   /* a hello typed or retried during this session is stale */

	printk("load_link: %s %s session: %s; id=%s; %u cmds, %u B to FPGA, %u B to host, "
	       "%d dropped, %lld ms\n", t->name, k->name, err ? "NO ACK" : st.end, idtxt,
	       st.cmds, st.to_fpga, st.to_host, (int)atomic_get(&from_fpga_dropped),
	       k_uptime_get() - t0);
}

/* Parse a hello line into a request. */
static bool parse_hello(const char *line, struct request *rq)
{
	for (size_t i = 0; i < ARRAY_SIZE(KINDS); i++) {
		size_t n = strlen(KINDS[i].hello);

		if (strncmp(line, KINDS[i].hello, n) != 0) {
			continue;
		}
		const char *rest = &line[n];

		rq->kind = (enum kind)i;
		rq->fast = false;
		rq->addr = 0;
		rq->mode = 0;
		rq->args[0] = '\0';
		if (rq->kind == K_FLASH) {
			if (*rest == '\0') {
				return true;
			}
			if (strcmp(rest, " FAST") == 0) {
				rq->fast = true;
				return true;
			}
			return false;
		}
		if (rq->kind == K_STASH || rq->kind == K_POLICY) {
			strncpy(rq->args, rest, sizeof(rq->args) - 1);
			rq->args[sizeof(rq->args) - 1] = '\0';
			return true;
		}
		if (rq->kind == K_BOOT) {
			char *endp;

			if (*rest != ' ') {
				return false;
			}
			unsigned long a = strtoul(rest + 1, &endp, 16);

			if (endp == rest + 1) {
				return false;
			}
			/* Optional " <mode>" 0..3: the ICAP sequence (EspLinkIprog). */
			if (endp[0] == ' ' && endp[1] >= '0' && endp[1] <= '3' && endp[2] == '\0') {
				rq->mode = (uint8_t)(endp[1] - '0');
			} else if (*endp != '\0') {
				return false;
			}
			rq->addr = (uint32_t)a;
			return true;
		}
		return *rest == '\0';
	}
	return false;
}

/* ---- listeners --------------------------------------------------------------- */

/* Read one hello line on a fresh TCP connection. */
static bool tcp_hello(int sock, struct request *rq)
{
	char line[HELLO_LINE_MAX];
	size_t n = 0;
	int64_t deadline = k_uptime_get() + HELLO_TIMEOUT_MS;

	while (k_uptime_get() < deadline) {
		struct zsock_pollfd pfd = { .fd = sock, .events = ZSOCK_POLLIN };
		char c;

		if (zsock_poll(&pfd, 1, 100) <= 0) {
			continue;
		}
		if (zsock_recv(sock, &c, 1, 0) != 1) {
			return false;
		}
		if (c == '\n') {
			line[n] = '\0';
			return parse_hello(line, rq);
		}
		if (c != '\r' && n < sizeof(line) - 1) {
			line[n++] = c;
		}
	}
	return false;
}

static char usb_line[HELLO_LINE_MAX];
static size_t usb_line_n;

/* Feed USB console input; true when a hello line has arrived. */
static bool usb_hello(struct request *rq)
{
	char *line = usb_line;
	size_t n = usb_line_n;
	bool hit = false;
	unsigned char c;

	while (!hit && usb != NULL && uart_poll_in(usb, &c) == 0) {
		if (c == '\n') {
			if (n < sizeof(usb_line)) {
				line[n] = '\0';
				hit = parse_hello(line, rq);
			}
			n = 0;
		} else if (c != '\r') {
			if (n < sizeof(usb_line) - 1) {
				line[n++] = (char)c;
			} else {
				n = sizeof(usb_line);   /* overlong: never matches */
			}
		}
	}
	usb_line_n = n;
	return hit;
}

static void usb_flush(void)
{
	unsigned char c;

	while (usb != NULL && uart_poll_in(usb, &c) == 0) {
	}
	usb_line_n = 0;
}

static int listen_sock = -1;

static void open_listener(void)
{
	struct sockaddr_in addr = {
		.sin_family = AF_INET,
		.sin_port = htons(LOAD_LINK_TCP_PORT),
		.sin_addr.s_addr = htonl(INADDR_ANY),
	};
	int s = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	if (s < 0) {
		return;
	}
	if (zsock_bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
	    zsock_listen(s, 4) < 0) {
		zsock_close(s);
		return;
	}
	listen_sock = s;
	printk("load_link: program loading on TCP :%d%s (RBLOAD1/RBPOKE1/RBPING1/"
	       "RBFLASH1/RBBOOT1)\n", LOAD_LINK_TCP_PORT, usb ? " and the USB console" : "");
}

static void load_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	int64_t next_listen_try = 0;

	/* If this ESP restarted in the middle of a session, the mux may still be
	 * in TSI or FLASH (or, for FLASH, waiting out its idle timer): a bare
	 * break returns it to uart1 within the key window. */
	k_mutex_lock(&tx_lock, K_FOREVER);
	atomic_set(&active, 1);
	(void)send_break();
	k_msleep(DRAIN_MS);
	from_fpga_reset();
	atomic_set(&active, 0);
	k_mutex_unlock(&tx_lock);

	persist_start();   /* power-on only: boot policy and the stashed program */

	for (;;) {
		if (listen_sock < 0 && k_uptime_get() >= next_listen_try) {
			open_listener();
			if (listen_sock < 0) {
				next_listen_try = k_uptime_get() + 2000;
			}
		}
		if (listen_sock >= 0) {
			struct zsock_pollfd pfd = { .fd = listen_sock, .events = ZSOCK_POLLIN };

			if (zsock_poll(&pfd, 1, 20) > 0) {
				int s = zsock_accept(listen_sock, NULL, NULL);

				if (s >= 0) {
					int one = 1;

					(void)zsock_setsockopt(s, IPPROTO_TCP, TCP_NODELAY,
							       &one, sizeof(one));
					char peek;
					struct request rq;

					if (tcp_hello(s, &rq) &&
					    /* A request that gave up while another session ran
					     * still has its hello buffered: do not reset the SoC
					     * for a peer that has already closed. */
					    zsock_recv(s, &peek, 1, ZSOCK_MSG_PEEK | ZSOCK_MSG_DONTWAIT) != 0) {
						struct transport t = {
							.name = "WiFi", .sock = s, .is_usb = false,
						};
						session(&t, &rq);
					} else {
						static const char bad[] = "RB ERR expected RBLOAD1, RBPOKE1, "
							"RBPING1, RBFLASH1 [FAST], RBBOOT1 <hex> [mode] or RBSTAT1\n";

						(void)zsock_send(s, bad, sizeof(bad) - 1, 0);
					}
					zsock_close(s);
				}
			}
		} else {
			k_msleep(20);
		}
		struct request urq;

		if (usb_hello(&urq)) {
			struct transport t = { .name = "USB", .sock = -1, .is_usb = true };

			session(&t, &urq);
		}
	}
}

K_THREAD_STACK_DEFINE(load_stack, 4096);
static struct k_thread load_thread_data;

int load_link_start(const struct device *fpga_uart, const struct device *usb_dev)
{
	fpga = fpga_uart;
	usb = (usb_dev != NULL && device_is_ready(usb_dev)) ? usb_dev : NULL;
	ring_buf_init(&from_fpga, sizeof(from_fpga_buf), from_fpga_buf);
	k_thread_create(&load_thread_data, load_stack, K_THREAD_STACK_SIZEOF(load_stack),
			load_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(4), 0, K_NO_WAIT);
	k_thread_name_set(&load_thread_data, "loadlink");
	return 0;
}

/* ---- internal sessions (persist.c: power-on boot policy, stashed program) ---- */

void ll_claim(void)
{
	k_mutex_lock(&tx_lock, K_FOREVER);
	from_fpga_reset();
	atomic_set(&active, 1);
	k_msleep(10);
}

void ll_release(bool end_break)
{
	if (end_break) {
		if (send_break()) {
			k_msleep(DRAIN_MS);
		} else {
			k_msleep(FALLBACK_DRAIN_MS);
		}
	}
	set_link_baud(LINK_BAUD);
	from_fpga_reset();
	atomic_set(&active, 0);
	k_mutex_unlock(&tx_lock);
}

bool ll_handshake(enum kind kind, uint32_t addr, uint32_t *id, bool *id_valid)
{
	const struct kind_info *k = &KINDS[kind];

	if (!send_break()) {
		return false;
	}
	from_fpga_reset();
	fpga_write(KEY_PREFIX, sizeof(KEY_PREFIX));
	fpga_putc(k->sel);
	if (kind == K_BOOT) {
		uint8_t arg[8];

		sys_put_le32(addr, &arg[0]);
		sys_put_le32(~addr, &arg[4]);
		fpga_write(arg, sizeof(arg));
	}
	if (!wait_ack(k->ack, id, id_valid)) {
		return false;
	}
	if (kind == K_LOAD) {
		k_msleep(RESET_SETTLE_MS);
		tsi_synced = true;
	} else if (kind == K_BOOT) {
		tsi_synced = false;
	} else {
		k_msleep(2);
	}
	from_fpga_reset();
	return true;
}

void ll_write(const uint8_t *b, size_t n)
{
	fpga_write(b, n);
}

size_t ll_read(uint8_t *b, size_t n, int timeout_ms)
{
	size_t got = 0;
	int64_t deadline = k_uptime_get() + timeout_ms;

	while (got < n && k_uptime_get() < deadline) {
		uint32_t m = ring_buf_get(&from_fpga, &b[got], n - got);

		got += m;
		if (m == 0) {
			k_msleep(1);
		}
	}
	return got;
}

int ll_send(int *sock, bool is_usb, const char *s)
{
	struct transport t = { .name = "", .sock = sock ? *sock : -1, .is_usb = is_usb };

	return t_send(&t, (const uint8_t *)s, strlen(s));
}

int ll_recv(int *sock, bool is_usb, uint8_t *buf, size_t max)
{
	struct transport t = { .name = "", .sock = sock ? *sock : -1, .is_usb = is_usb };

	return t_recv(&t, buf, max);
}
