/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * WiFi SoftAP UDP telemetry downlink (see telem_wifi.h + docs/TELEMETRY_PLAN.md).
 *
 * Bring-up follows the proven Zephyr apsta_mode sample
 * (zephyr_ws/zephyr/samples/net/wifi/apsta_mode): set a static IP + netmask + gw on the WiFi
 * interface, start the DHCPv4 server, then NET_REQUEST_WIFI_AP_ENABLE. On the esp32c6 the single
 * WiFi interface is placed into AP mode by the driver's ap_enable (drivers/wifi/esp32); no AP-STA
 * co-existence is needed for a telemetry-only link, so we keep the lighter SoftAP-only path.
 *
 * Telemetry is BROADCAST on the SoftAP subnet (192.168.4.255:14550), so any laptop that joins the
 * AP receives it without the firmware having to learn the client's DHCP-assigned address. UDP +
 * loss-tolerant: each datagram carries the latest state, so a dropped packet just means "wait for
 * the next one" (docs/TELEMETRY_PLAN.md sec 1).
 */

/* Entire module gated on CONFIG_WIFI (defense-in-depth; CMakeLists also only compiles it when
 * CONFIG_WIFI, so the normal flight build never sees this file). */
#if defined(CONFIG_WIFI)

#include "telem_wifi.h"
#include "flow.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/dhcpv4_server.h>
#include <zephyr/net/socket.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

/* ---- tunables (override via -DEXTRA_CPPFLAGS like the other ROSE_* knobs) --------------------- */
#ifndef ROSE_TELEM_AP_SSID_PREFIX
#define ROSE_TELEM_AP_SSID_PREFIX "riskybird-"  /* SSID = prefix + last 2 MAC bytes -> riskybird-<id> */
#endif
#ifndef ROSE_TELEM_AP_PSK
#define ROSE_TELEM_AP_PSK ""            /* empty => OPEN network (join with no password); else WPA2-PSK */
#endif
#ifndef ROSE_TELEM_AP_IP
#define ROSE_TELEM_AP_IP "192.168.4.1"          /* the drone's SoftAP / gateway address */
#endif
#ifndef ROSE_TELEM_AP_NETMASK
#define ROSE_TELEM_AP_NETMASK "255.255.255.0"
#endif
#ifndef ROSE_TELEM_AP_BROADCAST
#define ROSE_TELEM_AP_BROADCAST "192.168.4.255" /* directed broadcast for the /24 above */
#endif
#ifndef ROSE_TELEM_AP_CHANNEL
#define ROSE_TELEM_AP_CHANNEL 6                 /* 2.4 GHz channel; WIFI_CHANNEL_ANY == 0 */
#endif
#ifndef ROSE_TELEM_PORT
#define ROSE_TELEM_PORT 14550                   /* downlink UDP port (docs/TELEMETRY_PLAN.md sec 1) */
#endif
#ifndef ROSE_TELEM_HZ
#define ROSE_TELEM_HZ 50                        /* fixed downlink rate */
#endif
#ifndef ROSE_CMD_PORT
#define ROSE_CMD_PORT 14551                     /* uplink command UDP port (docs/TELEMETRY_PLAN.md sec 1) */
#endif

/* Split a signed float into ("-"/"" , int, milli) for a printk/snprintk %s%d.%03d field -- the
 * exact same decomposition as main.cpp's FP3, so the UDP line is byte-for-byte the ROSE_TELEM line. */
#define FP3(x) ((x) < 0 ? "-" : ""), (int)fabsf(x), ((int)(fabsf(x) * 1000.0f)) % 1000

/* ---- mutex-protected snapshot the control loop fills each iteration --------------------------- */
static K_MUTEX_DEFINE(telem_mtx);
static struct telem_snapshot g_snap;
static bool g_ready;   /* set once the AP + socket are up; publish is a no-op before this */

/* ---- WiFi mgmt event plumbing (log AP-up + station join; signal the TX thread to start) ------- */
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define TELEM_WIFI_EVENTS                                                                          \
	(NET_EVENT_WIFI_AP_ENABLE_RESULT | NET_EVENT_WIFI_AP_DISABLE_RESULT |                       \
	 NET_EVENT_WIFI_AP_STA_CONNECTED | NET_EVENT_WIFI_AP_STA_DISCONNECTED)

static struct net_if *g_ap_iface;
static struct net_mgmt_event_callback g_wifi_cb;
static K_SEM_DEFINE(g_ap_up, 0, 1);

static void wifi_event_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
			       struct net_if *iface)
{
	ARG_UNUSED(iface);
	switch (mgmt_event) {
	case NET_EVENT_WIFI_AP_ENABLE_RESULT:
		printk("telem_wifi: SoftAP enabled -- broadcasting telemetry on %s:%d @ %d Hz\n",
		       ROSE_TELEM_AP_BROADCAST, ROSE_TELEM_PORT, ROSE_TELEM_HZ);
		k_sem_give(&g_ap_up);
		break;
	case NET_EVENT_WIFI_AP_DISABLE_RESULT:
		printk("telem_wifi: SoftAP disabled\n");
		break;
	case NET_EVENT_WIFI_AP_STA_CONNECTED: {
		/* cb->info is only populated when CONFIG_NET_MGMT_EVENT_INFO=y; guard against NULL so a
		 * client association can NEVER crash the firmware (this NULL-deref was the original
		 * "AP appears but can't connect" failure -- the join event killed the AP). */
		struct wifi_ap_sta_info *s = (struct wifi_ap_sta_info *)cb->info;
		if (s != NULL) {
			printk("telem_wifi: client joined " MACSTR "\n",
			       s->mac[0], s->mac[1], s->mac[2], s->mac[3], s->mac[4], s->mac[5]);
		} else {
			printk("telem_wifi: client joined\n");
		}
		break;
	}
	case NET_EVENT_WIFI_AP_STA_DISCONNECTED: {
		struct wifi_ap_sta_info *s = (struct wifi_ap_sta_info *)cb->info;
		if (s != NULL) {
			printk("telem_wifi: client left " MACSTR "\n",
			       s->mac[0], s->mac[1], s->mac[2], s->mac[3], s->mac[4], s->mac[5]);
		} else {
			printk("telem_wifi: client left\n");
		}
		break;
	}
	default:
		break;
	}
}

/* ---- v1 wire format: the EXACT ROSE_TELEM text line, so tools/*.py parse it unchanged ---------- */
static int telem_format_v1(char *buf, size_t cap, const struct telem_snapshot *s)
{
	/* Raw flow counts for the panel's FLOW CAL mode (running totals; see flow.h). Read here, in the
	 * downlink thread, so the control loop's snapshot path is untouched. */
	struct flow_counts_snap fc;
	flow_counts(&fc);
	return snprintk(buf, cap,
		"flight_controller: it=%d dt=%dms roll=%s%d.%03d pitch=%s%d.%03d yaw=%s%d.%03d "
		"z=%s%d.%03d tofv=%d tofh=%s%d.%03d u=[%s%d.%03d %s%d.%03d %s%d.%03d %s%d.%03d] "
		"vx=%s%d.%03d vy=%s%d.%03d vz=%s%d.%03d zsp=%s%d.%03d vbat=%s%d.%03d st=%u "
		"x=%s%d.%03d y=%s%d.%03d "
		"amag=%s%d.%03d apk=%s%d.%03d why=%s wm=%s%d.%03d tfl=%d trc=%d trv=%u "
		"fcx=%d fcy=%d fn=%u fbad=%u ferr=%u fpk=%d,%d fsq=%d fsh=%d fraw=%d,%d,%d\n",
		s->iter, s->dt_ms,
		FP3(s->roll), FP3(s->pitch), FP3(s->yaw), FP3(s->z),
		s->tof_valid, FP3(s->height),
		FP3(s->u[0]), FP3(s->u[1]), FP3(s->u[2]), FP3(s->u[3]),
		FP3(s->vx), FP3(s->vy), FP3(s->vz), FP3(s->zsp), FP3(s->vbat),
		(unsigned)s->flags, FP3(s->x), FP3(s->y),
		FP3(s->amag), FP3(s->apk), (s->why ? s->why : "-"), FP3(s->why_meas),
		(int)s->tof_fails, (int)s->tof_rc, (unsigned)s->tof_recov,
		(int)fc.cx, (int)fc.cy, (unsigned)fc.n, (unsigned)fc.nbad, (unsigned)fc.nerr,
		(int)fc.pkx, (int)fc.pky, (int)fc.squal, (int)fc.shutter,
		(int)fc.rawsum, (int)fc.rawmax, (int)fc.rawmin);
}

/* ---- unicast telemetry to each DHCP-leased station (RELIABILITY) -----------------------------
 * Broadcast/multicast on a WiFi SoftAP is buffered at the AP and released only at DTIM/beacon
 * boundaries (bursty ~100 ms delivery) and is never MAC-ACKed or retried -> high jitter, loss, and
 * an effective rate well below 50 Hz. UNICAST is ACKed + retried + delivered immediately, so it
 * holds a smooth 50 Hz. The firmware runs the DHCP server, so it already knows every client's
 * address: unicast the telemetry line to each ALLOCATED lease (the host stays a passive UDP
 * listener -- no subscribe needed). Fall back to broadcast only when nobody holds a lease yet, so a
 * listener still sees traffic during the brief window before its lease is registered. */
#define TELEM_MAX_CLIENTS 4

/* IMPORTANT: net_dhcpv4_server_foreach_lease() invokes this callback while holding the DHCP server's
 * internal mutex. So the callback MUST be fast and non-blocking -- it only COPIES the leased address
 * into the caller's array. The actual (blocking) zsock_sendto() is done later, OUTSIDE the lock. An
 * earlier version sent from inside the callback: that ran a deep, blocking net-TX path while holding
 * the DHCP server_lock -> priority inversion + memory/stack pressure that corrupted the scheduler
 * (load-access fault in z_riscv_switch) ~1 s after a client's DHCP completed. */
struct telem_tx_ctx {
	struct in_addr addrs[TELEM_MAX_CLIENTS];
	int count;
};

static void telem_collect_lease(struct net_if *iface, struct dhcpv4_addr_slot *lease, void *ud)
{
	ARG_UNUSED(iface);
	struct telem_tx_ctx *ctx = (struct telem_tx_ctx *)ud;
	if (lease->state != DHCPV4_SERVER_ADDR_ALLOCATED) {
		return;
	}
	if (ctx->count < TELEM_MAX_CLIENTS) {
		ctx->addrs[ctx->count++] = lease->addr;
	}
}

/* ---- the downlink thread: copy-under-mutex, format, send. LOW priority (below tof/flow @ 8) so a
 *      blocking WiFi TX can never delay the control loop or the sensor threads. ------------------ */
/* 6 KB: the send path does snprintk() WITH float fields (FP3) plus net_dhcpv4_server_foreach_lease()
 * + zsock_sendto(); with CONFIG_CBPRINTF_FP_SUPPORT this is stack-hungry, so keep generous margin. */
K_THREAD_STACK_DEFINE(telem_stack, 6144);
static struct k_thread telem_thread;

static void telem_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	/* Wait for the AP to come up (async NET_REQUEST result), but don't hang forever if the event
	 * is missed -- the send loop tolerates a not-yet-ready interface (sendto just errors). */
	(void)k_sem_take(&g_ap_up, K_SECONDS(10));

	int sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		printk("telem_wifi: socket() failed (%d) -- downlink disabled\n", errno);
		return;
	}
	int bcast = 1;
	(void)zsock_setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));

	/* Broadcast destination -- used ONLY as a fallback when no client holds a DHCP lease yet. */
	struct sockaddr_in bdst;
	memset(&bdst, 0, sizeof(bdst));
	bdst.sin_family = AF_INET;
	bdst.sin_port = htons(ROSE_TELEM_PORT);
	(void)zsock_inet_pton(AF_INET, ROSE_TELEM_AP_BROADCAST, &bdst.sin_addr);

	const int period_ms = (ROSE_TELEM_HZ > 0) ? (1000 / ROSE_TELEM_HZ) : 20;
	char line[768];   /* also holds 7 FIMG rows (~600 B); v1.2 tail (amag/apk/why/wm) ~50 chars + the flow-cal tail (fcx..fsh) ~80 */
	int64_t next = k_uptime_get();
	for (;;) {
		struct telem_snapshot s;
		k_mutex_lock(&telem_mtx, K_FOREVER);
		s = g_snap;
		k_mutex_unlock(&telem_mtx);

		int n = telem_format_v1(line, sizeof(line), &s);
		/* snprintk returns the WOULD-BE length; on truncation that exceeds the buffer and the
		 * sendto below would read past it. Clamp to what was actually written. */
		if (n >= (int)sizeof(line)) {
			n = (int)sizeof(line) - 1;
		}
		if (n > 0) {
			/* Snapshot the current DHCP leases under the server lock (fast copy only), then
			 * unicast OUTSIDE the lock (reliable: MAC-ACKed + retried). Loss-tolerant: send
			 * errors (client gone, TX buffer full, ...) are ignored -- the next datagram carries
			 * the latest state. */
			struct telem_tx_ctx ctx;
			ctx.count = 0;
			(void)net_dhcpv4_server_foreach_lease(g_ap_iface, telem_collect_lease, &ctx);

			if (ctx.count > 0) {
				struct sockaddr_in d;
				memset(&d, 0, sizeof(d));
				d.sin_family = AF_INET;
				d.sin_port = htons(ROSE_TELEM_PORT);
				for (int i = 0; i < ctx.count; i++) {
					d.sin_addr = ctx.addrs[i];
					(void)zsock_sendto(sock, line, (size_t)n, 0,
							   (struct sockaddr *)&d, sizeof(d));
				}
			} else {
				/* No leased client yet -> broadcast fallback so a passive listener still
				 * sees traffic in the join->lease window. */
				(void)zsock_sendto(sock, line, (size_t)n, 0,
						   (struct sockaddr *)&bdst, sizeof(bdst));
			}
		}

		/* A fresh raw flow frame (FLOWIMG command) goes out as datagrams of 7 hex rows:
		 *   "FIMG <seq> <rc> <row> <70 hex chars>\n" x7   (35x35 pixels, 8-bit, row-major)
		 * ONE datagram per telemetry tick, the whole frame cycled 12x (60 ticks, ~1.2 s): bursts of
		 * back-to-back datagrams were being dropped (WiFi TX queue), and the host merges rows per seq,
		 * so any chunk lost once is filled by a later cycle. */
		static uint8_t fimg[FLOW_FRAME_PIXELS];
		static uint32_t fimg_seq;
		static int fimg_rc, fimg_left;
		{
			int frc = flow_frame_get(fimg, &fimg_seq);
			if (frc != 1) {
				fimg_rc = frc;
				fimg_left = 12 * ((FLOW_FRAME_SIDE + 6) / 7);   /* ~65% datagram loss seen on the bench */
			}
		}
		if (fimg_left > 0) {
			const int nchunk = (FLOW_FRAME_SIDE + 6) / 7;
			const int r0 = 7 * (--fimg_left % nchunk);
			int fn = 0;
			for (int r = r0; r < r0 + 7 && r < FLOW_FRAME_SIDE; r++) {
				fn += snprintk(line + fn, sizeof(line) - fn, "FIMG %u %d %d ",
					       (unsigned)fimg_seq, fimg_rc, r);
				for (int c = 0; c < FLOW_FRAME_SIDE; c++) {
					static const char hx[] = "0123456789abcdef";
					const uint8_t v = fimg[r * FLOW_FRAME_SIDE + c];
					line[fn++] = hx[v >> 4];
					line[fn++] = hx[v & 15];
				}
				line[fn++] = '\n';
			}
			struct telem_tx_ctx fctx;
			fctx.count = 0;
			(void)net_dhcpv4_server_foreach_lease(g_ap_iface, telem_collect_lease, &fctx);
			struct sockaddr_in d;
			memset(&d, 0, sizeof(d));
			d.sin_family = AF_INET;
			d.sin_port = htons(ROSE_TELEM_PORT);
			if (fctx.count == 0) {
				(void)zsock_sendto(sock, line, (size_t)fn, 0, (struct sockaddr *)&bdst, sizeof(bdst));
			}
			for (int i = 0; i < fctx.count; i++) {
				d.sin_addr = fctx.addrs[i];
				(void)zsock_sendto(sock, line, (size_t)fn, 0, (struct sockaddr *)&d, sizeof(d));
			}
		}

		/* Fixed-cadence 50 Hz: sleep until the next tick rather than "work + sleep(period)", so
		 * per-iteration send time doesn't drag the rate down. Never accumulate drift or spin if we
		 * fell behind (reset the schedule to now). */
		next += period_ms;
		int64_t now = k_uptime_get();
		int64_t slp = next - now;
		if (slp < 0) {
			next = now;
			slp = 0;
		}
		k_msleep((int32_t)slp);
	}
}

/* ---- static IP + DHCPv4 server on the AP interface (apsta_mode pattern) ----------------------- */
static void configure_ap_ipv4(struct net_if *iface)
{
	struct in_addr addr, netmask, pool_start;

	if (zsock_inet_pton(AF_INET, ROSE_TELEM_AP_IP, &addr) != 1 ||
	    zsock_inet_pton(AF_INET, ROSE_TELEM_AP_NETMASK, &netmask) != 1) {
		printk("telem_wifi: bad AP IP/netmask literal\n");
		return;
	}

	net_if_ipv4_set_gw(iface, &addr);
	if (net_if_ipv4_addr_add(iface, &addr, NET_ADDR_MANUAL, 0) == NULL) {
		printk("telem_wifi: failed to set AP IPv4 address\n");
	}
	if (!net_if_ipv4_set_netmask_by_addr(iface, &addr, &netmask)) {
		printk("telem_wifi: failed to set AP netmask\n");
	}

	/* DHCPv4 pool starts at .10 on the AP subnet (so the laptop gets e.g. 192.168.4.10). */
	pool_start = addr;
	pool_start.s4_addr[3] += 10;
	if (net_dhcpv4_server_start(iface, &pool_start) != 0) {
		printk("telem_wifi: DHCPv4 server did not start\n");
	} else {
		printk("telem_wifi: DHCPv4 server up (pool from %d.%d.%d.%d)\n",
		       pool_start.s4_addr[0], pool_start.s4_addr[1],
		       pool_start.s4_addr[2], pool_start.s4_addr[3]);
	}
}

/* ---- uplink command thread: recv UDP on ROSE_CMD_PORT, parse, ACK to sender ------------------
 * SEPARATE low-priority thread (like the TX thread) so the blocking recvfrom / net path never runs
 * in the control loop. One command per datagram, UPPERCASE, trailing whitespace tolerated:
 *     ESTOP          -> rose_cmd_estop()          (latched remote kill; clears only on reset)
 *     DISARM         -> rose_cmd_disarm()
 *     HOVER_Z <mm>   -> rose_cmd_set_hover_z(mm/1000)
 *     PING           -> liveness only
 * Each is ACKed back to the sender ("ACK <cmd>\n" / "PONG\n" / "ERR unknown\n"). There is NO
 * remote force-ARM -- arming stays on the on-board gate (safety). */
K_THREAD_STACK_DEFINE(cmd_stack, 4096);
static struct k_thread cmd_thread;

static void cmd_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	(void)k_sem_take(&g_ap_up, K_SECONDS(10));

	int sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		printk("telem_wifi: cmd socket() failed (%d) -- uplink disabled\n", errno);
		return;
	}
	struct sockaddr_in me;
	memset(&me, 0, sizeof(me));
	me.sin_family = AF_INET;
	me.sin_port = htons(ROSE_CMD_PORT);
	me.sin_addr.s_addr = htonl(INADDR_ANY);
	if (zsock_bind(sock, (struct sockaddr *)&me, sizeof(me)) < 0) {
		printk("telem_wifi: cmd bind(:%d) failed (%d) -- uplink disabled\n", ROSE_CMD_PORT, errno);
		(void)zsock_close(sock);
		return;
	}
	printk("telem_wifi: command uplink on UDP :%d (ESTOP / DISARM / RESET / HOVER_Z / PROFILE / PING)\n",
	       ROSE_CMD_PORT);

	char rx[64];
	for (;;) {
		struct sockaddr_in src;
		socklen_t slen = sizeof(src);
		int n = zsock_recvfrom(sock, rx, sizeof(rx) - 1, 0, (struct sockaddr *)&src, &slen);
		if (n <= 0) {
			continue;
		}
		rx[n] = '\0';
		while (n > 0 && (rx[n-1] == '\n' || rx[n-1] == '\r' || rx[n-1] == ' ' || rx[n-1] == '\t')) {
			rx[--n] = '\0';
		}
		char ack[64];
		int an;
		if (strncmp(rx, "ESTOP", 5) == 0) {
			rose_cmd_estop();
			printk("telem_wifi: CMD ESTOP -- remote kill latched\n");
			an = snprintk(ack, sizeof(ack), "ACK ESTOP\n");
		} else if (strncmp(rx, "DISARM", 6) == 0) {
			rose_cmd_disarm();
			printk("telem_wifi: CMD DISARM\n");
			an = snprintk(ack, sizeof(ack), "ACK DISARM\n");
		} else if (strncmp(rx, "HOVER_Z", 7) == 0) {
			int mm = atoi(rx + 7);
			rose_cmd_set_hover_z((float)mm / 1000.0f);
			printk("telem_wifi: CMD HOVER_Z %d mm\n", mm);
			an = snprintk(ack, sizeof(ack), "ACK HOVER_Z %d\n", mm);
		} else if (strncmp(rx, "PROFILE", 7) == 0) {
			/* PROFILE <climb_ms> <hover_ms> <descend_ms> <max_ms>  (ms; -1/absent = keep) */
			char *p = rx + 7;
			int v[4] = { -1, -1, -1, -1 };
			for (int vi = 0; vi < 4; vi++) {
				while (*p == ' ') { p++; }
				if (*p == '\0') { break; }
				v[vi] = (int)strtol(p, &p, 10);
			}
			rose_cmd_set_profile(v[0], v[1], v[2], v[3]);
			printk("telem_wifi: CMD PROFILE %d %d %d %d\n", v[0], v[1], v[2], v[3]);
			an = snprintk(ack, sizeof(ack), "ACK PROFILE %d %d %d %d\n", v[0], v[1], v[2], v[3]);
		} else if (strncmp(rx, "RESET", 5) == 0) {
			rose_cmd_reset();
			printk("telem_wifi: CMD RESET -- soft reset requested\n");
			an = snprintk(ack, sizeof(ack), "ACK RESET\n");
		} else if (strncmp(rx, "PILOT", 5) == 0) {
#if defined(ROSE_MANUAL_PILOT) && ROSE_MANUAL_PILOT
			/* PILOT <vx_mm> <vy_mm> <yawrate_mrad> <throttle_milli> -- manual pilot (Deck). Same
			 * wire format as the FPGA uart1 path; fields are milli-units so the parse stays integer. */
			char *p = rx + 5;
			long v[4] = { 0, 0, 0, 0 };
			for (int vi = 0; vi < 4; vi++) {
				while (*p == ' ') { p++; }
				if (*p == '\0') { break; }
				v[vi] = strtol(p, &p, 10);
			}
			rose_cmd_pilot((float)v[0] / 1000.0f, (float)v[1] / 1000.0f,
				       (float)v[2] / 1000.0f, (float)v[3] / 1000.0f);
#endif
			an = 0;   /* no ACK: a reply per 30 Hz packet would flood the uplink */
		} else if (strncmp(rx, "FLOWIMG", 7) == 0) {
			/* raw PMW3901 frame -> FIMG lines on the telemetry downlink. Bench only: the capture
			 * pauses flow tracking ~0.5 s, so it is refused while armed / arming. */
			/* "armed" alone is the idle-ready state in the manual-pilot build, so gate on what
			 * matters: the arm countdown, or any motor above idle (u ~ -0.583 at rest; same
			 * threshold as the host flash guard). */
			k_mutex_lock(&telem_mtx, K_FOREVER);
			const uint32_t fl = g_snap.flags;
			float umax = g_snap.u[0];
			for (int k = 1; k < 4; k++) { umax = g_snap.u[k] > umax ? g_snap.u[k] : umax; }
			k_mutex_unlock(&telem_mtx);
			/* a latched ESTOP means the motors are cut regardless of what u reads */
			if (!(fl & TELEM_FLAG_ESTOP) &&
			    ((fl & TELEM_FLAG_ARMING) || ((fl & TELEM_FLAG_ARMED) && umax > -0.55f))) {
				an = snprintk(ack, sizeof(ack), "ERR FLOWIMG refused: motors running (ESTOP first)\n");
			} else {
				int frc = flow_frame_request(strstr(rx, "SLOW") != NULL);
				an = snprintk(ack, sizeof(ack), frc == 0 ? "ACK FLOWIMG\n" : "ERR FLOWIMG no flow\n");
			}
		} else if (strncmp(rx, "FLOWINIT", 8) == 0) {
			/* "FLOWINIT 0-3": re-init the PMW3901: Crazyflie (0), PixArt v3.20/PX4 (1), PAW3902 low-light (2),
			 * PAW3902 super-low-light (3)
			 * register sequence. Same motor gate as FLOWIMG (tracking pauses during the re-init). */
			k_mutex_lock(&telem_mtx, K_FOREVER);
			const uint32_t fl = g_snap.flags;
			float umax = g_snap.u[0];
			for (int k = 1; k < 4; k++) { umax = g_snap.u[k] > umax ? g_snap.u[k] : umax; }
			k_mutex_unlock(&telem_mtx);
			const int var = (rx[8] == ' ' && rx[9] >= '0' && rx[9] <= '3') ? rx[9] - '0' : 0;
			if (!(fl & TELEM_FLAG_ESTOP) &&
			    ((fl & TELEM_FLAG_ARMING) || ((fl & TELEM_FLAG_ARMED) && umax > -0.55f))) {
				an = snprintk(ack, sizeof(ack), "ERR FLOWINIT refused: motors running (ESTOP first)\n");
			} else {
				int frc = flow_reinit_request(var);
				an = snprintk(ack, sizeof(ack), frc == 0 ? "ACK FLOWINIT %d\n" : "ERR FLOWINIT no flow\n", var);
			}
		} else if (strncmp(rx, "TOFRESET", 8) == 0) {
			rose_cmd_tof_reset();   /* safe in flight too: height goes stale ~30 ms, estimator predicts */
			an = snprintk(ack, sizeof(ack), "ACK TOFRESET\n");
		} else if (strncmp(rx, "PING", 4) == 0) {
			an = snprintk(ack, sizeof(ack), "PONG\n");
		} else {
			an = snprintk(ack, sizeof(ack), "ERR unknown\n");
		}
		if (an > 0) {
			(void)zsock_sendto(sock, ack, (size_t)an, 0, (struct sockaddr *)&src, slen);
		}
	}
}

int telem_wifi_init(void)
{
	static struct wifi_connect_req_params ap_cfg;
	static char ssid[32];

	net_mgmt_init_event_callback(&g_wifi_cb, wifi_event_handler, TELEM_WIFI_EVENTS);
	net_mgmt_add_event_callback(&g_wifi_cb);

	/* Prefer the NM-registered SoftAP interface (present only in AP+STA co-existence mode,
	 * CONFIG_ESP32_WIFI_AP_STA_MODE=y). In the lighter SoftAP-only / single-interface mode the esp32
	 * driver registers NEITHER a WIFI_TYPE_SAP nor a WIFI_TYPE_STA managed interface, so
	 * net_if_get_wifi_sap() returns NULL -- fall back to the first WiFi interface (the one the driver
	 * drives into AP mode via NET_REQUEST_WIFI_AP_ENABLE). Without this fallback telem_wifi_init()
	 * bailed out and the SoftAP never started -- a root cause of the earlier connect failure. */
	g_ap_iface = net_if_get_wifi_sap();
	if (g_ap_iface == NULL) {
		g_ap_iface = net_if_get_first_wifi();
	}
	if (g_ap_iface == NULL) {
		printk("telem_wifi: no WiFi interface -- downlink disabled\n");
		return -ENODEV;
	}
	printk("telem_wifi: using WiFi iface %p (idx %d)\n", g_ap_iface,
	       net_if_get_by_iface(g_ap_iface));

	/* SSID = riskybird-<id>, <id> = last 2 bytes of the SoftAP MAC (stable per board). */
	struct net_linkaddr *la = net_if_get_link_addr(g_ap_iface);
	if (la != NULL && la->len >= 6) {
		snprintk(ssid, sizeof(ssid), ROSE_TELEM_AP_SSID_PREFIX "%02x%02x",
			 la->addr[4], la->addr[5]);
	} else {
		snprintk(ssid, sizeof(ssid), ROSE_TELEM_AP_SSID_PREFIX "0000");
	}

	/* Static IP + DHCP BEFORE enabling the AP (matches the apsta_mode ordering). */
	configure_ap_ipv4(g_ap_iface);

	memset(&ap_cfg, 0, sizeof(ap_cfg));
	ap_cfg.ssid = (const uint8_t *)ssid;
	ap_cfg.ssid_length = strlen(ssid);
	ap_cfg.channel = ROSE_TELEM_AP_CHANNEL;
	ap_cfg.band = WIFI_FREQ_BAND_2_4_GHZ;
	if (sizeof(ROSE_TELEM_AP_PSK) <= 1) {   /* "" -> sizeof 1 -> OPEN */
		ap_cfg.security = WIFI_SECURITY_TYPE_NONE;
	} else {
		ap_cfg.psk = (const uint8_t *)ROSE_TELEM_AP_PSK;
		ap_cfg.psk_length = sizeof(ROSE_TELEM_AP_PSK) - 1;
		ap_cfg.security = WIFI_SECURITY_TYPE_PSK;
	}

	int ret = net_mgmt(NET_REQUEST_WIFI_AP_ENABLE, g_ap_iface, &ap_cfg,
			   sizeof(struct wifi_connect_req_params));
	if (ret) {
		printk("telem_wifi: NET_REQUEST_WIFI_AP_ENABLE failed (%d) -- downlink disabled\n", ret);
		return ret;
	}
	printk("telem_wifi: SoftAP requested -- SSID '%s' (%s), IP %s, UDP :%d @ %d Hz\n",
	       ssid, (ap_cfg.security == WIFI_SECURITY_TYPE_NONE) ? "open" : "wpa2",
	       ROSE_TELEM_AP_IP, ROSE_TELEM_PORT, ROSE_TELEM_HZ);

	g_ready = true;   /* publish() now records snapshots; the thread starts sending once AP is up */
	k_thread_create(&telem_thread, telem_stack, K_THREAD_STACK_SIZEOF(telem_stack),
			telem_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
	k_thread_name_set(&telem_thread, "telem");
	k_thread_create(&cmd_thread, cmd_stack, K_THREAD_STACK_SIZEOF(cmd_stack),
			cmd_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
	k_thread_name_set(&cmd_thread, "telem_cmd");
	return 0;
}

void telem_wifi_publish(const struct telem_snapshot *s)
{
	if (!g_ready || s == NULL) {
		return;
	}
	k_mutex_lock(&telem_mtx, K_FOREVER);
	g_snap = *s;
	k_mutex_unlock(&telem_mtx);
}

#endif /* CONFIG_WIFI */
