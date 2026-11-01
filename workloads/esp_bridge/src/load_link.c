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
 * SESSION PROTOCOL (identical on both transports)
 *
 *   host  -> "RBLOAD1\n"
 *   ESP   -> "RBLOAD OK\n"                  handshake with the FPGA done
 *         or "RBLOAD ERR <reason>\n"        and the session is over
 *   host  -> TSI commands, FPGA -> host TSI responses, raw
 *   host  -> END command (below), or closes the TCP connection
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

#define TSI_CMD_READ  0u
#define TSI_CMD_WRITE 1u
#define TSI_CMD_END   0xE5D0E5D0u
#define TSI_HDR_LEN   20
/* Largest write accepted in one command: 4 MiB. The host sends 1 KiB chunks;
 * anything near this is a corrupt header, and (len + 1) * 4 must not wrap. */
#define TSI_MAX_LEN_WORDS (1u << 20)

static const uint8_t KEY[8] = { 0x52, 0x42, 0x4c, 0x44, 0xa5, 0x5a, 0xc3, 0x3c };
static const uint8_t ACK[4] = { 'R', 'B', 'L', 'K' };
static const char HELLO[] = "RBLOAD1";

static const struct device *fpga;
static const struct device *usb;
static atomic_t active;
static K_MUTEX_DEFINE(tx_lock);

static uint8_t from_fpga_buf[4096];
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

static bool wait_ack(void)
{
	int64_t deadline = k_uptime_get() + ACK_TIMEOUT_MS;
	size_t matched = 0;

	while (k_uptime_get() < deadline) {
		uint8_t c;

		if (ring_buf_get(&from_fpga, &c, 1) != 1) {
			k_msleep(1);
			continue;
		}
		matched = (c == ACK[matched]) ? matched + 1 : (c == ACK[0] ? 1 : 0);
		if (matched == sizeof(ACK)) {
			return true;
		}
	}
	return false;
}

/* ---- the session ----------------------------------------------------------- */

struct session_stats {
	uint32_t cmds;
	uint32_t to_fpga;
	uint32_t to_host;
	const char *end;
};

static void pump(struct transport *t, struct session_stats *st)
{
	uint8_t hdr[TSI_HDR_LEN];
	size_t hn = 0;
	uint64_t payload_left = 0;
	int64_t last_traffic = k_uptime_get();

	for (;;) {
		uint8_t in[128];
		uint8_t out[128];
		bool progress = false;
		int n = t_recv(t, in, sizeof(in));

		if (n < 0) {
			st->end = (hn == 0 && payload_left == 0) ? "host closed" :
				  "host closed MID-COMMAND";
			return;
		}
		for (int i = 0; i < n; i++) {
			uint8_t b = in[i];

			progress = true;
			if (payload_left > 0) {
				uart_poll_out(fpga, b);
				payload_left--;
				st->to_fpga++;
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
				uart_poll_out(fpga, hdr[k]);
			}
			st->to_fpga += TSI_HDR_LEN;
			st->cmds++;
			payload_left = (cmd == TSI_CMD_WRITE) ? (len + 1u) * 4u : 0u;
		}

		uint32_t m = ring_buf_get(&from_fpga, out, sizeof(out));

		if (m > 0u) {
			progress = true;
			st->to_host += m;
			if (t_send(t, out, m) < 0) {
				st->end = "host send failed";
				return;
			}
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

static void session(struct transport *t)
{
	struct session_stats st = { .end = "?" };
	const char *err = NULL;
	int64_t t0 = k_uptime_get();

	k_mutex_lock(&tx_lock, K_FOREVER);   /* waits out an in-flight uplink line */
	from_fpga_reset();
	atomic_set(&from_fpga_dropped, 0);
	atomic_set(&active, 1);     /* motors 0, uplink refused, RX diverted */
	if (t->is_usb) {
		logs_quiet(true);
	}
	k_msleep(10);               /* > one motor-link tick: gates are at 0 */

	if (!send_break()) {
		err = "RBLOAD ERR esp-uart-busy\n";
	} else {
		from_fpga_reset();
		for (size_t i = 0; i < sizeof(KEY); i++) {
			uart_poll_out(fpga, KEY[i]);
		}
		if (!wait_ack()) {
			err = "RBLOAD ERR no-ack: the FPGA did not answer the loader key "
			      "(bitstream without the ESP loader, or link down)\n";
		}
	}

	if (err != NULL) {
		t_say(t, err);
		st.end = "handshake failed";
		/* A non-loader bitstream's uart1 just saw a break and 8 junk bytes;
		 * a loader bitstream that missed the key is back on uart1 already. */
		k_msleep(300);
	} else {
		k_msleep(RESET_SETTLE_MS);
		from_fpga_reset();
		t_say(t, "RBLOAD OK\n");
		pump(t, &st);
		/* Hand the pins back to uart1: bare break, then wait out the key
		 * window. If the break cannot be sent, fall back to the idle exit. */
		if (send_break()) {
			k_msleep(DRAIN_MS);
		} else {
			k_msleep(FALLBACK_DRAIN_MS);
		}
	}

	from_fpga_reset();
	atomic_set(&active, 0);
	if (t->is_usb) {
		logs_quiet(false);
	}
	k_mutex_unlock(&tx_lock);
	usb_flush();   /* a hello typed or retried during this session is stale */

	printk("load_link: %s session: %s; %u cmds, %u B to FPGA, %u B to host, "
	       "%d dropped, %lld ms\n", t->name, err ? "NO ACK" : st.end,
	       st.cmds, st.to_fpga, st.to_host, (int)atomic_get(&from_fpga_dropped),
	       k_uptime_get() - t0);
}

/* ---- listeners --------------------------------------------------------------- */

/* Read one hello line on a fresh TCP connection. */
static bool tcp_hello(int sock)
{
	char line[16];
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
			return strcmp(line, HELLO) == 0;
		}
		if (c != '\r' && n < sizeof(line) - 1) {
			line[n++] = c;
		}
	}
	return false;
}

static char usb_line[16];
static size_t usb_line_n;

/* Feed USB console input; true when a hello line has arrived. */
static bool usb_hello(void)
{
	char *line = usb_line;
	size_t n = usb_line_n;
	bool hit = false;
	unsigned char c;

	while (!hit && usb != NULL && uart_poll_in(usb, &c) == 0) {
		if (c == '\n') {
			hit = (n == strlen(HELLO)) && memcmp(line, HELLO, n) == 0;
			n = 0;
		} else if (c != '\r') {
			if (n < sizeof(usb_line)) {
				line[n++] = (char)c;
			} else {
				n = sizeof(usb_line) + 1;   /* overlong: never matches */
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
	    zsock_listen(s, 1) < 0) {
		zsock_close(s);
		return;
	}
	listen_sock = s;
	printk("load_link: program loading on TCP :%d%s\n", LOAD_LINK_TCP_PORT,
	       usb ? " and the USB console (\"RBLOAD1\")" : "");
}

static void load_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	int64_t next_listen_try = 0;

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

					if (tcp_hello(s) &&
					    /* A request that gave up while another session ran
					     * still has its hello buffered: do not reset the SoC
					     * for a peer that has already closed. */
					    zsock_recv(s, &peek, 1, ZSOCK_MSG_PEEK | ZSOCK_MSG_DONTWAIT) != 0) {
						struct transport t = {
							.name = "WiFi", .sock = s, .is_usb = false,
						};
						session(&t);
					} else {
						static const char bad[] = "RBLOAD ERR expected RBLOAD1\n";

						(void)zsock_send(s, bad, sizeof(bad) - 1, 0);
					}
					zsock_close(s);
				}
			}
		} else {
			k_msleep(20);
		}
		if (usb_hello()) {
			struct transport t = { .name = "USB", .sock = -1, .is_usb = true };

			session(&t);
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
