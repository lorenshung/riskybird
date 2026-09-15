/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * RiskyBird ESP32-C6 telemetry bridge: FPGA UART <-> WiFi UDP.
 *
 * On the ESP board the flight controller and the radio are the same chip. On
 * the FPGA carrier they are not: Rocket runs the flight controller and this
 * chip is a WiFi modem reached over a serial link. So this firmware is
 * deliberately a DUMB PIPE -- it does not parse telemetry, does not know the
 * field names, and has no opinion about the protocol. Both ends already agree:
 *
 *   downlink  FPGA uart1 -> here -> UDP unicast :14550 -> riskybird_panel.py
 *   uplink    panel -> UDP :14551 -> here -> FPGA uart1
 *
 * Verified before writing any of this: the flight controller's existing
 * telemetry line already matches the ground station's v1.1 parser exactly
 * (500/500 real captured lines matched CORE_RE), so there is no format to
 * translate and nothing here needs to change when the telemetry gains fields.
 *
 * UNICAST, NOT BROADCAST. WiFi broadcast/multicast is buffered at the AP and
 * released only on DTIM/beacon boundaries, and is never MAC-ACKed or retried:
 * docs/TELEMETRY_BRINGUP.md measured that as bursty ~37 Hz with 100 ms+ gaps.
 * Unicast to each DHCP-leased client is ACKed, retried and delivered
 * immediately -- a smooth ~50 Hz at <0.5% loss. We run the DHCP server, so the
 * leases are exactly the list of clients to send to.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/dhcpv4_server.h>
#include <zephyr/net/socket.h>
#include <string.h>
#include <stdio.h>

#define FPGA_UART_NODE DT_ALIAS(fpga_uart)
#if !DT_NODE_EXISTS(FPGA_UART_NODE)
#error "esp_bridge needs an fpga-uart alias (see esp_bridge.overlay)"
#endif

#define TELEM_PORT 14550
#define CMD_PORT   14551

/* A telemetry line is ~140 bytes today. 256 leaves room for the enriched tail
 * (vx/vy/vz/zsp/vbat/st) without a second thought; anything longer is
 * truncated at a line boundary rather than being allowed to desynchronise the
 * stream. */
#define LINE_MAX   256

/* Sized for roughly 8 lines in flight. The UART ISR must never block, so the
 * only backpressure available is dropping -- and dropping whole lines at the
 * ring is far better than delivering half of one. */
#define RB_SIZE    2048

#define AP_CHANNEL 6
#define MAX_CLIENTS 4

static const struct device *const fpga_uart = DEVICE_DT_GET(FPGA_UART_NODE);

static uint8_t rb_buf[RB_SIZE];
static struct ring_buf uart_rb;

static K_SEM_DEFINE(line_ready, 0, 1);
static struct net_mgmt_event_callback wifi_cb;

/* ---- UART receive ---------------------------------------------------------
 * Interrupt-driven, because polling a 115200 link from a thread either burns a
 * core or drops bytes, and this is the only copy of the flight data. The ISR
 * does nothing but move bytes into the ring. */
static void uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	uint8_t buf[64];

	if (!uart_irq_update(dev)) {
		return;
	}
	while (uart_irq_rx_ready(dev)) {
		int n = uart_fifo_read(dev, buf, sizeof(buf));

		if (n <= 0) {
			break;
		}
		/* Short write = ring full. Nothing useful to do in an ISR
		 * except keep draining the FIFO, which is what this loop is. */
		(void)ring_buf_put(&uart_rb, buf, (uint32_t)n);
		k_sem_give(&line_ready);
	}
}

static void fpga_uart_write(const char *s, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(fpga_uart, s[i]);
	}
}

/* ---- client list ----------------------------------------------------------
 * Rebuilt from the DHCP server's leases on each send rather than cached: a
 * laptop that reconnects gets a new lease, and a cached address would keep
 * unicasting into the void while looking perfectly healthy. */
struct client_list {
	struct in_addr addr[MAX_CLIENTS];
	int count;
};

static void lease_cb(struct net_if *iface, struct dhcpv4_addr_slot *lease,
		     void *user_data)
{
	struct client_list *cl = user_data;

	ARG_UNUSED(iface);
	if (lease == NULL || cl->count >= MAX_CLIENTS) {
		return;
	}
	if (lease->state != DHCPV4_SERVER_ADDR_ALLOCATED) {
		return;
	}
	cl->addr[cl->count++] = lease->addr;
}

/* ---- downlink -------------------------------------------------------------
 * Assemble complete lines and send one UDP datagram per line to each leased
 * client. One line per packet keeps the ground station's parser trivial and
 * means a lost packet costs exactly one sample. */
static void downlink_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	static char line[LINE_MAX];
	size_t len = 0;
	bool overlong = false;
	int sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	if (sock < 0) {
		printk("esp_bridge: telemetry socket failed (%d)\n", errno);
		return;
	}

	for (;;) {
		uint8_t ch;

		if (ring_buf_get(&uart_rb, &ch, 1) != 1) {
			k_sem_take(&line_ready, K_MSEC(200));
			continue;
		}
		if (ch == '\r') {
			continue;
		}
		if (ch != '\n') {
			if (len < sizeof(line) - 1) {
				line[len++] = (char)ch;
			} else {
				/* Mark rather than flush: emitting the first 255
				 * bytes of a longer line would look like a valid
				 * short line to the parser. */
				overlong = true;
			}
			continue;
		}
		if (overlong || len == 0) {
			len = 0;
			overlong = false;
			continue;
		}
		line[len++] = '\n';

		struct client_list cl = { .count = 0 };

		(void)net_dhcpv4_server_foreach_lease(net_if_get_wifi_sap(),
						      lease_cb, &cl);

		/* Diagnostic, rate-limited to the first few lines and then one a
		 * second. Without it a silent downlink is ambiguous between three
		 * very different faults: no bytes arriving on the UART at all, a
		 * line assembled but no DHCP lease to unicast it to, or the send
		 * itself failing. Each needs a different fix, and the console
		 * cannot distinguish them after the fact. */
		{
			static uint32_t seen;
			static int64_t next_report;

			seen++;
			if (seen <= 3 || k_uptime_get() >= next_report) {
				next_report = k_uptime_get() + 1000;
				printk("esp_bridge: line #%u (%u B), %d client(s): %.*s\n",
				       seen, (unsigned)len, cl.count,
				       (int)(len > 60 ? 60 : len - 1), line);
			}
		}
		for (int i = 0; i < cl.count; i++) {
			struct sockaddr_in dst = {
				.sin_family = AF_INET,
				.sin_port = htons(TELEM_PORT),
			};

			dst.sin_addr = cl.addr[i];
			(void)zsock_sendto(sock, line, len, 0,
					   (struct sockaddr *)&dst, sizeof(dst));
		}
		len = 0;
	}
}

/* ---- uplink ---------------------------------------------------------------
 * Commands in, straight out of the UART. The ACK is issued by the BRIDGE, not
 * relayed from the flight controller: the panel does a blocking recvfrom after
 * each send, and making that wait on a round trip through a 115200 link plus a
 * control loop would stall the dashboard on the one path that must never
 * stall -- the kill switch. The bridge ACK means "delivered to the FPGA";
 * whether the command took effect is visible in the next telemetry line's
 * state flags, which is the honest place to look for it. */
static void uplink_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	int sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	struct sockaddr_in bind_addr = {
		.sin_family = AF_INET,
		.sin_port = htons(CMD_PORT),
		.sin_addr.s_addr = htonl(INADDR_ANY),
	};

	if (sock < 0 ||
	    zsock_bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
		printk("esp_bridge: command socket failed (%d)\n", errno);
		return;
	}
	printk("esp_bridge: commands on UDP :%d\n", CMD_PORT);

	for (;;) {
		char cmd[128];
		struct sockaddr_in src;
		socklen_t srclen = sizeof(src);
		int n = zsock_recvfrom(sock, cmd, sizeof(cmd) - 2, 0,
				       (struct sockaddr *)&src, &srclen);

		if (n <= 0) {
			continue;
		}
		while (n > 0 && (cmd[n - 1] == '\n' || cmd[n - 1] == '\r')) {
			n--;
		}
		if (n == 0) {
			continue;
		}
		cmd[n] = '\n';
		fpga_uart_write(cmd, (size_t)n + 1);

		char ack[160];
		int alen = snprintf(ack, sizeof(ack), "ACK %.*s\n", n, cmd);

		if (alen > 0) {
			(void)zsock_sendto(sock, ack, (size_t)alen, 0,
					   (struct sockaddr *)&src, srclen);
		}
		cmd[n] = '\0';
		printk("esp_bridge: cmd -> FPGA: %s\n", cmd);
	}
}

K_THREAD_STACK_DEFINE(downlink_stack, 6144);
K_THREAD_STACK_DEFINE(uplink_stack, 4096);
static struct k_thread downlink_thread_data;
static struct k_thread uplink_thread_data;

/* ---- WiFi -----------------------------------------------------------------  */
static void wifi_event(struct net_mgmt_event_callback *cb, uint64_t event,
		       struct net_if *iface)
{
	ARG_UNUSED(iface);
	switch (event) {
	case NET_EVENT_WIFI_AP_ENABLE_RESULT:
		printk("esp_bridge: SoftAP up\n");
		break;
	case NET_EVENT_WIFI_AP_STA_CONNECTED:
		/* cb->info is NULL unless CONFIG_NET_MGMT_EVENT_INFO=y. It is
		 * set in prj.conf; this guard is the second half of that fix,
		 * because the failure mode was a fault at association time that
		 * presented as a radio problem. */
		printk("esp_bridge: client associated\n");
		break;
	case NET_EVENT_WIFI_AP_STA_DISCONNECTED:
		printk("esp_bridge: client left\n");
		break;
	default:
		break;
	}
}

static int softap_start(void)
{
	struct net_if *iface = net_if_get_wifi_sap();
	static char ssid[32];
	struct net_linkaddr *mac;
	struct wifi_connect_req_params p = { 0 };
	struct in_addr base, gw, netmask;

	if (iface == NULL) {
		printk("esp_bridge: no SoftAP interface\n");
		return -ENODEV;
	}

	/* SSID carries the MAC tail so two drones in a room are tellable apart
	 * without reflashing either -- the ground station auto-detects
	 * riskybird-*. */
	mac = net_if_get_link_addr(iface);
	snprintf(ssid, sizeof(ssid), "riskybird-%02x%02x",
		 mac->addr[4], mac->addr[5]);

	p.ssid = (const uint8_t *)ssid;
	p.ssid_length = (uint8_t)strlen(ssid);
	p.channel = AP_CHANNEL;
	p.band = WIFI_FREQ_BAND_2_4_GHZ;
	/* Open. A PSK buys nothing here: the link carries telemetry and a kill
	 * switch over a dedicated AP nobody routes, and a wrong-password failure
	 * during a flight is a worse outcome than an open network in a lab. */
	p.security = WIFI_SECURITY_TYPE_NONE;

	if (net_mgmt(NET_REQUEST_WIFI_AP_ENABLE, iface, &p, sizeof(p))) {
		printk("esp_bridge: AP enable failed\n");
		return -EIO;
	}

	/*
	 * Give the AP interface its own address BEFORE starting the DHCP server.
	 *
	 * Starting the server alone is not enough and fails silently in the worst
	 * way: the client associates fine, sees no DHCP offer, falls back to an
	 * IPv4LL 169.254.x address and cannot reach the drone at all. Measured
	 * exactly that -- wpa_state=COMPLETED, then `using IPv4LL address
	 * 169.254.150.200` and 100% packet loss to 192.168.4.1. The radio was
	 * never the problem; the interface simply had no address to serve from.
	 *
	 * 192.168.4.1/24 with the pool from .11 is what the ground station
	 * defaults to (riskybird_panel.py --drone 192.168.4.1).
	 */
	if (net_addr_pton(AF_INET, "192.168.4.1", &gw) != 0 ||
	    net_addr_pton(AF_INET, "255.255.255.0", &netmask) != 0 ||
	    net_addr_pton(AF_INET, "192.168.4.11", &base) != 0) {
		printk("esp_bridge: bad address literal\n");
		return -EINVAL;
	}
	if (net_if_ipv4_addr_add(iface, &gw, NET_ADDR_MANUAL, 0) == NULL) {
		printk("esp_bridge: could not set 192.168.4.1 on the AP iface\n");
		return -EIO;
	}
	(void)net_if_ipv4_set_netmask_by_addr(iface, &gw, &netmask);

	{
		int rc = net_dhcpv4_server_start(iface, &base);

		if (rc != 0 && rc != -EALREADY) {
			printk("esp_bridge: DHCP server failed (%d)\n", rc);
			return rc;
		}
	}
	printk("esp_bridge: SSID %s, ch %d, open; telemetry -> UDP :%d\n",
	       ssid, AP_CHANNEL, TELEM_PORT);
	return 0;
}

int main(void)
{
	printk("\n=== RiskyBird ESP bridge: FPGA uart <-> WiFi UDP ===\n");

	if (!device_is_ready(fpga_uart)) {
		printk("esp_bridge: fpga-uart not ready\n");
		return -ENODEV;
	}
	ring_buf_init(&uart_rb, sizeof(rb_buf), rb_buf);

	uart_irq_callback_user_data_set(fpga_uart, uart_isr, NULL);
	uart_irq_rx_enable(fpga_uart);
	printk("esp_bridge: FPGA link up on %s\n", fpga_uart->name);

	net_mgmt_init_event_callback(&wifi_cb, wifi_event,
				     NET_EVENT_WIFI_AP_ENABLE_RESULT |
				     NET_EVENT_WIFI_AP_STA_CONNECTED |
				     NET_EVENT_WIFI_AP_STA_DISCONNECTED);
	net_mgmt_add_event_callback(&wifi_cb);

	if (softap_start() != 0) {
		printk("esp_bridge: radio down -- UART still logged below\n");
	}

	k_thread_create(&downlink_thread_data, downlink_stack,
			K_THREAD_STACK_SIZEOF(downlink_stack),
			downlink_thread, NULL, NULL, NULL,
			K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	k_thread_name_set(&downlink_thread_data, "downlink");

	k_thread_create(&uplink_thread_data, uplink_stack,
			K_THREAD_STACK_SIZEOF(uplink_stack),
			uplink_thread, NULL, NULL, NULL,
			K_PRIO_PREEMPT(6), 0, K_NO_WAIT);
	k_thread_name_set(&uplink_thread_data, "uplink");

	return 0;
}
