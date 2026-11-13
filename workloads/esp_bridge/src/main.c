/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * RiskyBird ESP32-C6 link workload: FPGA UART <-> WiFi UDP, and -- when the
 * devicetree says so -- the four motor gates as well.
 *
 * On the ESP board the flight controller and the radio are the same chip. On
 * the FPGA carrier they are not: Rocket runs the flight controller and this
 * chip is a WiFi modem reached over a serial link. So the radio half of this
 * firmware is deliberately a DUMB PIPE -- it does not parse telemetry, does not
 * know the field names, and has no opinion about the ASCII protocol. Both ends
 * already agree:
 *
 *   downlink  FPGA uart1 -> here -> UDP unicast :14550 -> ground station
 *   uplink    panel -> UDP :14551 -> here -> FPGA uart1 (+ local ESTOP, below)
 *
 * Verified before writing any of this: the flight controller's existing
 * telemetry line already matches the ground station's v1.1 parser exactly
 * (500/500 real captured lines matched CORE_RE), so there is no format to
 * translate and nothing here needs to change when telemetry gains fields.
 *
 * UNICAST, NOT BROADCAST. WiFi broadcast/multicast is buffered at the AP and
 * released only on DTIM/beacon boundaries, and is never MAC-ACKed or retried:
 * docs/TELEMETRY_BRINGUP.md measured that as bursty ~37 Hz with 100 ms+ gaps.
 * Unicast to each DHCP-leased client is ACKed, retried and delivered
 * immediately -- a smooth ~50 Hz at <0.5% loss. We run the DHCP server, so the
 * leases are exactly the list of clients to send to.
 *
 *
 * ===========================================================================
 * TWO BUILDS OUT OF ONE SOURCE, SELECTED BY DEVICETREE
 * ===========================================================================
 *
 * default   esp_bridge.overlay: ledc0 disabled, the four motor pins hogged as
 *           pulled-down INPUTS, no pwm-motorN aliases. This file then contains
 *           no code path to a motor gate at all -- HAVE_MOTORS is 0 and every
 *           motor block below is compiled out. Behaviourally what the bridge
 *           has always been, and the image every telemetry bring-up step that
 *           must not be able to spin a motor uses.
 *
 * motors    esp_bridge-motors.overlay: LEDC channels on GPIO21/20/23/22 and the
 *           pwm-motorN aliases. This image OWNS ALL FOUR GATES and additionally
 *           decodes the FPGA's 14-byte duty frames off the same uart1.
 *
 * WHY THEY HAVE TO BE ONE IMAGE. FPGA ball F13 is dead as an output, so motor
 * PWM moved to this chip (workloads/esp_motors). There is one ESP, it runs one
 * image, and there is exactly one wire between the FPGA and it. A drone that is
 * flying has its motors on this chip and its telemetry on that wire; wanting
 * both at once is not a preference, it is the only arrangement in which an
 * untethered flight has any telemetry at all. See
 * docs/wireless-telemetry-bringup.md.
 *
 * WHY THE TWO PROTOCOLS CAN SHARE THE WIRE. The duty frame opens with 0xA5 0x5A
 * -- two bytes that cannot occur in the ASCII telemetry line -- and is validated
 * by CRC, not by the magic. The receiver is a 14-byte sliding window, so it
 * resynchronises within one frame from any amount of interleaved text. What the
 * window does NOT do on its own is keep frame bytes out of the text stream, so
 * demux_feed() below holds the text back by exactly 14 bytes and retracts them
 * when a frame is accepted. Without that, every accepted frame would inject 14
 * binary bytes into the middle of a telemetry line -- including 0x0A bytes out
 * of the duty fields, which would split it.
 *
 *
 * ===========================================================================
 * SCHEDULING -- READ THIS BEFORE CHANGING ANY PRIORITY IN THE motors BUILD
 * ===========================================================================
 *
 * The motor failsafe is the last thing standing between a software fault and
 * four spinning props. In the motors build it shares a CPU with the ESP-IDF
 * WiFi blob and the Zephyr networking stack, so "can anything hold the CPU
 * against the failsafe?" has to be answered from the scheduler, not from hope.
 *
 * A NOTE RECORDED FROM EARLIER ANALYSIS SAID: "CONFIG_NET_TC_THREAD_TYPE has no
 * explicit default so NET_TC_THREAD_COOPERATIVE wins and the networking RX/TX
 * threads land at K_PRIO_COOP(0) = -16, the SAME priority as the motor thread.
 * Merging REQUIRES CONFIG_NET_TC_THREAD_PRIO_CUSTOM with the TC base priorities
 * pushed down." The first half is right; the conclusion is WRONG IN A WAY THAT
 * MAKES THINGS WORSE. Read out of this checkout's Zephyr 4.2.99 and out of this
 * workload's own generated .config:
 *
 *  1. NET_TC_THREAD_COOPERATIVE does win (.config: CONFIG_NET_TC_THREAD_COOPERATIVE=y).
 *     But WITHOUT NET_TC_THREAD_PRIO_CUSTOM, subsys/net/ip/net_tc.c:163 sets
 *     BASE_PRIO_RX = CONFIG_NET_TC_NUM_PRIORITIES - 1 = 15, and the RX thread is
 *     created at K_PRIO_COOP(15) = -(16 - 15) = -1. NOT -16. net_tc.c's own
 *     comment spells the series out: "-1, -2, -3, ...".
 *  2. CONFIG_NET_TC_TX_COUNT is 0 in this build, so there is no traffic-class TX
 *     thread at all -- TX runs in the caller's context, which here is the
 *     preemptible downlink thread.
 *  3. Turning on CONFIG_NET_TC_THREAD_PRIO_CUSTOM *without also setting the base
 *     priorities* defaults NET_TC_RX/TX_THREAD_BASE_PRIO to 0
 *     (subsys/net/ip/Kconfig:297-312), i.e. K_PRIO_COOP(0) = -16 -- it CREATES
 *     the exact tie the note was written to prevent. Do not apply that advice
 *     literally.
 *  4. And neither knob fixes the real hazard, because in Zephyr A COOPERATIVE
 *     THREAD IS NEVER PREEMPTED BY ANOTHER COOPERATIVE THREAD, whatever their
 *     priorities: kernel/include/kthread.h:227 should_preempt() returns false
 *     unless the running thread is preemptible or the incoming one is metairq.
 *     A coop net RX thread at -1 therefore still holds the CPU until it blocks,
 *     and a -16 motor thread waits. Same for the net_mgmt work queue
 *     (subsys/net/ip/net_mgmt.c:419, also K_PRIO_COOP(15) while the TC threads
 *     are cooperative) and for the system work queue, which this workload's
 *     .config puts at CONFIG_SYSTEM_WORKQUEUE_PRIORITY=-1 -- and which is where
 *     DHCP server work runs.
 *
 * So what conf/motors.conf actually does is make those threads PREEMPTIBLE
 * (CONFIG_NET_TC_THREAD_PREEMPTIVE=y, which also demotes net_mgmt by the #elif
 * above, plus a preemptible system work queue). Then the cooperative link
 * thread is the only cooperative thread in the system and nothing but an ISR
 * can take the CPU from it. THIS IMAGE CHECKS THAT AT BOOT rather than asking
 * anyone to believe it: sched_audit() below walks every thread and prints a
 * verdict line naming any other cooperative thread it finds. Read that line on
 * the bench before the props go on.
 *
 * The escape hatch if the audit ever reports a cooperative thread that cannot
 * be demoted: CONFIG_NUM_METAIRQ_PRIORITIES=1 turns priority -16 into a metairq
 * priority (kernel/include/kthread.h:72 -- the predicate is simply
 * `prio - K_HIGHEST_THREAD_PRIO < CONFIG_NUM_METAIRQ_PRIORITIES`), which makes
 * this thread preempt cooperative threads too, with no code change. It is not
 * on by default because Zephyr's own Kconfig says metairq "probably shouldn't
 * be used from application code", and because an unproven scheduler change on a
 * motor bench is its own hazard.
 *
 * What NO Kconfig fixes: the WiFi MAC does substantial work in interrupt
 * context, and ISRs preempt everything including this thread. That is the
 * realistic source of failsafe jitter in the motors build, it is why the
 * failsafe timeout keeps 3.4 nominal frame intervals of margin, and it is the
 * first thing to measure on the bench.
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

#include "load_link.h"
#include "esp_safety.h"

#define FPGA_UART_NODE DT_ALIAS(fpga_uart)
#if !DT_NODE_EXISTS(FPGA_UART_NODE)
#error "esp_bridge needs an fpga-uart alias (see esp_bridge.overlay)"
#endif

/* The one switch. Devicetree, not a -D: the motor path needs LEDC channels and
 * pinctrl that only the motors overlay provides, so a build that could enable
 * the code without them would link and then drive nothing. */
#define HAVE_MOTORS DT_NODE_EXISTS(DT_ALIAS(pwm_motor0))

/* Sensor relay (sensor_relay.c): the motors overlay declares the relay's down-ToF node. It
 * does nothing until a relay flight controller sends its keepalive. */
#define HAVE_SENSOR_RELAY (HAVE_MOTORS && DT_NODE_HAS_STATUS(DT_NODELABEL(relay_vl53l1x), okay))
#if HAVE_SENSOR_RELAY
#include "sensor_relay.h"
#include <riskybird/sensor_link.h>
#endif

#if HAVE_MOTORS
#include <zephyr/drivers/pwm.h>
#include <riskybird/motor_link.h>
#include <riskybird/motor_softstart.h>

/* Phase interleave of the four LEDC channels (motor_phase_apply()). 1 = on,
 * 0 = Zephyr's in-phase hpoint 0, for bench A/B. A CMake cache variable in
 * CMakeLists.txt, like ESP_LINK_MAX_DUTY_PCT. */
#ifndef ESP_LINK_PWM_INTERLEAVE
#define ESP_LINK_PWM_INTERLEAVE 1
#endif
/* The spin-up gate in motors_service(). 1 = on, 0 = commands applied as
 * received (the pre-gate behaviour), for bench A/B. Same mechanism. */
#ifndef ESP_LINK_SOFTSTART
#define ESP_LINK_SOFTSTART 1
#endif
#if ESP_LINK_PWM_INTERLEAVE
#include <hal/ledc_ll.h>
#endif
#endif

#define TELEM_PORT 14550
#define CMD_PORT   14551

/* Must hold the LONGEST line the flight controller mirrors, not a typical one:
 * anything longer is dropped whole at the line boundary, silently. It was 256,
 * sized for the ~240-byte v1.1 line; the FPGA build's line is 281 bytes
 * (measured, 2026-09-23) out of a 384-byte esp_line[] buffer in main.cpp, so
 * EVERY telemetry line was dropped and only short replies like "FCACK PING"
 * reached the ground station. 512 covers the sender's whole buffer. */
#define TELEM_LINE_MAX 512

/* Sized for roughly 8 lines in flight at that length. The UART ISR must never
 * block, so the only backpressure available is dropping -- and dropping whole
 * lines at the ring is far better than delivering half of one. */
#define RB_SIZE    4096

#define AP_CHANNEL 6
#define MAX_CLIENTS 4

static const struct device *const fpga_uart = DEVICE_DT_GET(FPGA_UART_NODE);

static uint8_t rb_buf[RB_SIZE];
static struct ring_buf uart_rb;

/* Given by whoever puts bytes where the downlink thread reads them: the ISR in
 * the default build, the link thread in the motors build. */
static K_SEM_DEFINE(line_ready, 0, 1);
static struct net_mgmt_event_callback wifi_cb;

#if HAVE_MOTORS
K_SEM_DEFINE(rx_sem, 0, 1);   /* ISR -> link thread */

/* Second stage: ASCII only, after the demux has taken the duty frames out. */
static uint8_t line_rb_buf[RB_SIZE];
static struct ring_buf line_rb;
#define DOWNLINK_RB (&line_rb)
#else
#define DOWNLINK_RB (&uart_rb)
#endif

/* ---- UART receive ---------------------------------------------------------
 * Interrupt-driven, because polling a 115200 link from a thread either burns a
 * core or drops bytes, and this is the only copy of the flight data. The ISR
 * does nothing but move bytes into the ring. */
/* Bytes uart1 delivered outside load sessions since boot: persist.c reads it
 * to tell "a program is already running" from "the FPGA just powered up". */
static atomic_t g_uart1_rx_total = ATOMIC_INIT(0);

uint32_t esp_bridge_uart1_rx_bytes(void)
{
	return (uint32_t)atomic_get(&g_uart1_rx_total);
}

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
		/* A program-load session owns the wire: these bytes are TSI
		 * responses, not telemetry or duty frames. */
		if (load_link_active()) {
			load_link_rx_from_fpga(buf, (size_t)n);
			continue;
		}
		atomic_add(&g_uart1_rx_total, n);
		/* Short write = ring full. Nothing useful to do in an ISR
		 * except keep draining the FIFO, which is what this loop is. */
		(void)ring_buf_put(&uart_rb, buf, (uint32_t)n);
#if HAVE_MOTORS
		k_sem_give(&rx_sem);
#else
		k_sem_give(&line_ready);
#endif
	}
}

/* Returns false, writing nothing, while a program-load session owns the wire:
 * the FPGA end is a bus master then, not the flight controller's uart1. */
static bool fpga_uart_write(const char *s, size_t len)
{
	if (!load_link_tx_trylock()) {
		return false;
	}
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(fpga_uart, s[i]);
	}
	load_link_tx_unlock();
	return true;
}

#if HAVE_SENSOR_RELAY
/* For sensor_relay.c: the same writer, so a frame and a forwarded command line never interleave. */
bool esp_bridge_fpga_write(const char *s, size_t len)
{
	return fpga_uart_write(s, len);
}
#endif

/* ==========================================================================
 * MOTOR HALF -- compiled only when the devicetree declares the gates
 * ========================================================================== */
#if HAVE_MOTORS

/* 20 kHz, matching the FPGA build and workloads/esp_motors. Above audible, and
 * well inside what the LEDC resolves. */
#define MOTOR_PERIOD_NSEC 50000U

/*
 * Failsafe timeout. The same number workloads/esp_motors derives and for the
 * same reasons -- worst benign gap ~95 ms (a ~35 ms control loop plus a 30-60 ms
 * stall on the FPGA's polled console), airframe attitude time constant ~150 ms
 * -- so 120 ms sits above the first and below the second. Overridable for bench
 * work with RB_EXTRA_CFLAGS=-DESP_LINK_TIMEOUT_MS=<n>.
 *
 * NOTE FOR THE MOTORS BUILD SPECIFICALLY: putting the ASCII telemetry on this
 * same wire ADDS to the worst benign gap, because the FPGA writes it with a
 * polled uart_poll_out and cannot emit a duty frame while it does. A ~240-byte
 * line is 20.8 ms of wire time, so the worst frame interval grows by about that
 * much: 20 + 35 + 60 = 115 ms, uncomfortably close to 120. That is the single
 * strongest reason to keep the telemetry divisor high in an offloaded flight
 * build -- see the bandwidth table in docs/wireless-telemetry-bringup.md -- and
 * the reason this constant has to be re-derived if the telemetry rate changes.
 */
#ifndef ESP_LINK_TIMEOUT_MS
#define ESP_LINK_TIMEOUT_MS 120
#endif

/* How often the link thread wakes when no byte arrives. Bounds the failsafe's
 * own granularity: worst-case cut latency is TIMEOUT + TICK. */
#ifndef ESP_LINK_TICK_MS
#define ESP_LINK_TICK_MS 5
#endif

/*
 * Independent duty ceiling, applied here, after everything the FPGA did. Same
 * value and same argument as workloads/esp_motors: a flight-configured FPGA
 * build must not be able to produce flight thrust on a bench that was not
 * expecting it. Raise deliberately, and only for a flight test:
 *   RB_CMAKE_ARGS="-DESP_LINK_MAX_DUTY_PCT=80"   (a cache var in CMakeLists.txt;
 *   RB_EXTRA_CFLAGS is dropped on this target and silently leaves it at 25)
 */
#ifndef ESP_LINK_MAX_DUTY_PCT
#define ESP_LINK_MAX_DUTY_PCT 25
#endif

/* How often the ESP's own link-health line is emitted, to the console and to
 * every leased client. 5 Hz: enough for an operator to watch, small enough
 * (~160 B) that it costs the radio nothing and never touches the FPGA wire. */
#ifndef ESP_LINK_STATUS_MS
#define ESP_LINK_STATUS_MS 200
#endif

static const struct pwm_dt_spec motors[MOTOR_LINK_NMOTORS] = {
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor0)),   /* MOTOR1 GPIO21 */
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor1)),   /* MOTOR2 GPIO20 */
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor2)),   /* MOTOR3 GPIO23 */
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor3)),   /* MOTOR4 GPIO22 */
};

/* Written by the link thread, read by the status and uplink threads. Plain
 * volatile scalars rather than a lock: the status line is diagnostics, a torn
 * read costs one cosmetically odd line, and taking a mutex in the link thread
 * would let a printing thread delay the failsafe. */
static volatile uint16_t g_duty[MOTOR_LINK_NMOTORS];   /* per 10000, as applied */
static volatile uint32_t g_frames;
static volatile uint32_t g_gaps;
static volatile uint32_t g_stale;
static volatile uint32_t g_failsafe_cuts;
static volatile uint32_t g_pwm_errs;
static volatile uint32_t g_text_bytes;     /* ASCII bytes handed to the radio half */
static volatile uint8_t  g_flags;
static volatile uint8_t  g_seq;
static volatile bool     g_link_up;
static volatile int32_t  g_last_gap_ms;

/*
 * REMOTE KILL, AND WHY IT ONLY EXISTS IN THIS BUILD.
 *
 * The bridge writes uplink command bytes into the FPGA's uart1 receiver. Until
 * 2026-09-23 the flight controller did not read them, so the panel's ESTOP did
 * nothing on this carrier while the bridge still printed "ACK ESTOP". The FC's
 * ROSE_UART_CMD reader (on in every flight preset) now acts on them; an image
 * built without it still drops them.
 *
 * Once this image owns the gates there is a second, independent kill: it can be
 * executed HERE, one component before the MOSFET, with no dependency on the
 * FPGA reading anything or even still running. That is the strongest single
 * argument for merging the two images, and it is latched -- cleared only by an
 * explicit RESET or a power cycle, never by the next frame saying ARMED.
 *
 * The latch also survives an ESP reset: it is mirrored into RTC memory
 * (esp_safety_kill_store) and restored in main() before the link thread runs.
 * Only the explicit RESET, or a power-on that loses LP SRAM, clears it.
 */
static volatile bool g_kill;

/* Bumped by the uplink thread on every ESTOP, so the link thread sees a latch
 * even if a RESET cleared g_kill again within the same 5 ms tick. */
static atomic_t g_kill_epoch = ATOMIC_INIT(0);

/* The re-arm gate (esp_safety_core.h): after a boot or a kill, ARMED frames are
 * refused until the sender has been seen NOT armed and then armed again. Link
 * thread only; the status thread reads it for RBLINK. */
static struct esp_rearm g_rearm = { .hold = ESP_REARM_BOOT };

/*
 * Whether the 120 ms silence failsafe ALSO sets the re-arm hold. 0 (default):
 * no -- the cut zeroes the gates and the next valid ARMED frame resumes
 * (unlimited if the cut was shorter than SS_CUT_COLD_MS, through a cold
 * spin-up if longer: motors_cut()). In flight the FC keeps sending ARMED across a
 * link gap and never disarms, so a hold here would turn a 120 ms UART hiccup
 * into "motors off until someone re-arms", i.e. a crash. 1: the cut needs a
 * disarm -> arm edge like a boot does (bench policy). A CMake cache variable
 * in CMakeLists.txt, read back from the boot banner.
 */
#ifndef ESP_LINK_REARM_AFTER_CUT
#define ESP_LINK_REARM_AFTER_CUT 0
#endif

/* esp_safety_wdt_start()'s result, for the RBLINK wdt= field: -EAGAIN until
 * the first accepted ARMED frame starts the watchdog. */
static volatile int g_wdt_rc = -EAGAIN;

/* ---- actuation ----------------------------------------------------------- */

/*
 * Apply one duty vector, in units of 1/10000. Everything that reaches a gate
 * goes through here, so neither the ceiling nor the kill latch can be bypassed
 * by a code path.
 *
 * Thread context only, and only the link thread. pwm_led_esp32_set_cycles()
 * takes a per-device semaphore with K_FOREVER, so it is not callable from an
 * ISR or a k_timer expiry -- which is why the failsafe is a deadline this
 * thread checks rather than a timer that cuts the motors itself. Keeping this
 * the sole caller also means that semaphore is never contended, so the
 * cooperative link thread can never block on another thread's PWM write.
 */
/* When a gate last carried a nonzero duty: load_link refuses every session
 * for a second after that, because a session takes the wire the motor frames
 * travel on (and resets the SoC, for a load). Mid-flight, either drops the
 * drone. */
static atomic_t g_last_spin_ms = ATOMIC_INIT(0);
static atomic_t g_ever_spun = ATOMIC_INIT(0);

bool esp_bridge_motors_recently_on(void)
{
	if (!atomic_get(&g_ever_spun)) {
		return false;
	}
	return (uint32_t)(k_uptime_get_32() - (uint32_t)atomic_get(&g_last_spin_ms)) < 1000u;
}

#if ESP_LINK_PWM_INTERLEAVE
/*
 * PWM PHASE INTERLEAVE. All four channels share LEDC timer 0 and Zephyr's
 * driver writes hpoint = 0 on every pwm_set (pwm_led_esp32.c:103), so all four
 * low-side FETs turn on at the same counter value every 50 us and +BATT sees
 * 4x the per-motor current in one pulse. Here, after the driver has set the
 * channels up, the hpoints are moved so the four pulses are PACKED into the
 * period, two from the left and two from the right (ss_phase_hpoints() in
 * motor_softstart.h): at most two FETs on at once up to ~50 % duty, where the
 * old per-channel spread had three on from ~40 % (2026-11-05 power analysis:
 * COLL 600 = 0.43 duty sat at the logic-dropout edge on the bench leads).
 * Average current is unchanged; the peak, and the on-phase notch on +BATT, drop.
 *
 * NO WRAPAROUND. ESP32-C6 TRM v1.2 sec. 35.3.3: the output goes high at
 * counter == Hpoint and low at counter == Lpoint = DUTY[24:4] + HPOINT, both
 * latched at overflow, and the counter only runs 0 .. 2^res - 1. An Lpoint
 * >= 2^res never matches and would leave the gate HIGH, so hpoint + duty is
 * kept <= 2^res - 1 for every duty. Consequence: above 50 % duty every pulse
 * covers mid-period and all four still overlap there (for 2d - P + 1 counts).
 *
 * Sequence, after any channel was rewritten: read every channel's running
 * state and the DUTY[24:4] in its register (the driver's double rounding can
 * differ from ours by a count), compute all four hpoints, and for each running
 * channel whose hpoint register differs write hpoint, then duty_start +
 * para_up as IDF's ledc_update_duty() does. Channels the driver stopped (0 %
 * and 100 % -- sig_out_en 0, idle level) are never touched, so duty 0 stays
 * genuinely off, and count as idle for placement. Every (hpoint, duty) pair
 * the hardware can latch is valid: the driver writes hpoint 0 BEFORE a new
 * duty, and each hpoint is bounded by P-1 minus the duty already in ITS OWN
 * register -- a neighbour's duty only moves it within that bound -- so a
 * channel whose duty did not change can be re-placed with no new duty at all.
 * If an overflow falls between the driver's para_up and ours, one 50 us period
 * runs with a pulse at hpoint 0 -- harmless.
 *
 * Link thread only, after pwm_set_dt() returned: the driver's semaphore is
 * free and nothing else touches these channels.
 */
#define MOTOR_LEDC_HW ((ledc_dev_t *)DT_REG_ADDR(DT_PWMS_CTLR(DT_ALIAS(pwm_motor0))))

static void motor_phase_apply_all(void)
{
	ledc_dev_t *const hw = MOTOR_LEDC_HW;
	uint32_t d[MOTOR_LINK_NMOTORS], h[MOTOR_LINK_NMOTORS], period = 0;
	bool run[MOTOR_LINK_NMOTORS];

	for (unsigned i = 0; i < MOTOR_LINK_NMOTORS; i++) {
		const ledc_channel_t ch = (ledc_channel_t)motors[i].channel;
		const uint32_t tsel =
			hw->channel_group[LEDC_LOW_SPEED_MODE].channel[ch].conf0.timer_sel;
		const uint32_t p =
			1u << hw->timer_group[LEDC_LOW_SPEED_MODE].timer[tsel].conf.duty_res;

		if (i == 0u) {
			period = p;
		} else if (p != period) {
			return;   /* not one shared timer: leave every hpoint as it is */
		}
		/* sig_out_en is the driver's own running/stopped state
		 * (pwm_led_esp32_start/stop). A pulse under one 80 MHz cycle
		 * (duty 1-2/10000) truncates to 0 cycles inside pwm_set_dt and
		 * takes the stop path without writing DUTY, so the register can
		 * still hold the previous nonzero duty: never trust it alone. */
		run[i] = hw->channel_group[LEDC_LOW_SPEED_MODE].channel[ch].conf0.sig_out_en;
		d[i] = run[i] ? hw->channel_group[LEDC_LOW_SPEED_MODE].channel[ch].duty.duty >> 4
			      : 0u;
	}
	ss_phase_hpoints(d, period, h);
	for (unsigned i = 0; i < MOTOR_LINK_NMOTORS; i++) {
		const ledc_channel_t ch = (ledc_channel_t)motors[i].channel;
		uint32_t cur;

		if (!run[i] || d[i] == 0u || d[i] >= period - 1u) {
			continue;   /* leave the driver's hpoint 0 */
		}
		ledc_ll_get_hpoint(hw, LEDC_LOW_SPEED_MODE, ch, &cur);
		if (cur == h[i]) {
			continue;
		}
		ledc_ll_set_hpoint(hw, LEDC_LOW_SPEED_MODE, ch, h[i]);
		ledc_ll_set_duty_start(hw, LEDC_LOW_SPEED_MODE, ch, true);
		ledc_ll_ls_channel_update(hw, LEDC_LOW_SPEED_MODE, ch);
	}
}
#endif

/* Last duty successfully written per gate; 0xFFFF forces the first write.
 * The link thread wakes on every UART ISR, so rewriting unchanged duties
 * would reprogram the LEDC (and reopen the in-phase window above) many times
 * per frame. */
static uint16_t g_written[MOTOR_LINK_NMOTORS] = { 0xFFFFu, 0xFFFFu, 0xFFFFu, 0xFFFFu };

static void motors_apply(const uint16_t duty[MOTOR_LINK_NMOTORS])
{
	const uint32_t ceiling = (MOTOR_LINK_DUTY_FULL * ESP_LINK_MAX_DUTY_PCT) / 100U;
	bool rewrote = false;

	for (unsigned i = 0; i < MOTOR_LINK_NMOTORS; i++) {
		/* A program load resets the SoC and starts new code on it: no
		 * gate is driven from the moment it claims the wire. */
		uint32_t d = (g_kill || load_link_active()) ? 0u : duty[i];
		uint32_t pulse;
		int rc;

		if (d > ceiling) {
			d = ceiling;
		}
		if (d != g_written[i]) {
			/* Active-high into the SI2302: pulse 0 is genuinely off. */
			pulse = (uint32_t)((uint64_t)MOTOR_PERIOD_NSEC * d /
					   MOTOR_LINK_DUTY_FULL);
			rc = pwm_set_dt(&motors[i], MOTOR_PERIOD_NSEC, pulse);
			if (rc != 0) {
				g_pwm_errs++;
				g_written[i] = 0xFFFFu;   /* retry next call */
			} else {
				g_written[i] = (uint16_t)d;
				rewrote = true;
			}
		}
		g_duty[i] = (uint16_t)d;
		if (d != 0u) {
			atomic_set(&g_last_spin_ms, (atomic_val_t)k_uptime_get_32());
			atomic_set(&g_ever_spun, 1);
		}
	}
#if ESP_LINK_PWM_INTERLEAVE
	/* A changed duty moves its neighbours too (packing): re-place all four. */
	if (rewrote) {
		motor_phase_apply_all();
	}
#else
	(void)rewrote;
#endif
}

/* ---- spin-up gate ---------------------------------------------------------
 * The ESP's last-stage soft start (rules in riskybird/motor_softstart.h):
 * handle_frame() stores the commanded duties; motors_service() runs on every
 * link-thread wake (<= ESP_LINK_TICK_MS) and writes the gated duties. Link
 * thread only; the status thread reads g_ss for the RBLINK line, torn reads
 * being cosmetic as for g_duty. */
static uint16_t g_tgt[MOTOR_LINK_NMOTORS];   /* post-ceiling commands */
static struct ss g_ss;

static uint32_t motors_now_us(void)
{
	return (uint32_t)k_ticks_to_us_floor64((uint64_t)k_uptime_ticks());
}

static void motors_set_target(const uint16_t duty[MOTOR_LINK_NMOTORS])
{
	const uint32_t ceiling = (MOTOR_LINK_DUTY_FULL * ESP_LINK_MAX_DUTY_PCT) / 100U;

	/* Ceiling first, so the gate's "never above the command" is about what
	 * can actually reach a gate. */
	for (unsigned i = 0; i < MOTOR_LINK_NMOTORS; i++) {
		g_tgt[i] = (uint16_t)(duty[i] > ceiling ? ceiling : duty[i]);
	}
}

static void motors_service(uint32_t now_us)
{
	uint16_t tgt[MOTOR_LINK_NMOTORS];
	/* esp_safety_wdt_held(): persist.c is erasing ESP flash with the
	 * watchdog paused (granted only after 1 s with no duty), so nothing may
	 * drive until it is released. */
	const bool blocked = g_kill || load_link_active() || esp_safety_wdt_held();

	for (unsigned i = 0; i < MOTOR_LINK_NMOTORS; i++) {
		tgt[i] = blocked ? 0u : g_tgt[i];
	}
#if ESP_LINK_SOFTSTART
	uint16_t out[MOTOR_LINK_NMOTORS];

	ss_step(&g_ss, tgt, out, now_us);
	motors_apply(out);
#else
	ARG_UNUSED(now_us);
	motors_apply(tgt);
#endif
}

/* Disarm (not-ARMED frame), kill, load session, flash hold, re-arm refusal:
 * gates to 0 and every motor COLD, so the next arm always spins up. */
static void motors_off(void)
{
	static const uint16_t zero[MOTOR_LINK_NMOTORS] = { 0, 0, 0, 0 };

	ss_reset(&g_ss);
	memset(g_tgt, 0, sizeof(g_tgt));
	motors_apply(zero);
}

/*
 * The 120 ms silence failsafe: gates to 0 at once, but the spin-up gate is NOT
 * reset (ss_cut, rule 2 in motor_softstart.h). In flight the FC keeps sending
 * ARMED across a link gap, and a rotor 120 ms after the cut is still spinning:
 * resuming it through hold + stagger would cost authority and kick attitude.
 * The motors go COLD only if the cut lasts SS_CUT_COLD_MS (rotors spun down,
 * inrush matters again); handle_frame() ends the cut with ss_resume().
 */
static void motors_cut(uint32_t now_us)
{
	memset(g_tgt, 0, sizeof(g_tgt));
	ss_cut(&g_ss, now_us);
	motors_service(now_us);
}

/* ---- frame handling ------------------------------------------------------
 *
 * The failsafe rules, identical to workloads/esp_motors because they are the
 * contract and not a local choice:
 *
 *   silence       No valid frame for ESP_LINK_TIMEOUT_MS -> all four to 0.
 *   CRC failure   Frame discarded, duties unchanged, deadline NOT advanced. A
 *                 stream of corrupt frames therefore ends in the timeout, which
 *                 is right: corrupt is indistinguishable from absent.
 *   sequence gap  Frame IS applied and the deadline IS advanced. A gap means
 *                 bytes were lost, not that this command is wrong -- and this
 *                 command is the newest one.
 *   stale seq     Dropped, and does NOT advance the deadline. A UART cannot
 *                 reorder, so this is a replay or the tail of a resync.
 *                 handle_frame() returns false for it and link_thread() only
 *                 moves the deadline on true, so a sender stuck re-emitting
 *                 one frame (same seq) ends in the timeout like silence.
 *   not ARMED     Every motor to 0, whatever the duty fields say.
 *   kill latched  Every motor to 0, whatever the frame says, until RESET.
 */
static struct motor_link_rx g_rx;   /* link thread only */

/* Returns whether the frame counts as link activity (feeds the failsafe
 * deadline): false only for a stale/replayed sequence number. */
static bool handle_frame(const struct motor_link_cmd *cmd, int64_t now)
{
	static uint8_t last_seq;
	static int64_t last_ms;

	if (g_link_up) {
		int delta = motor_link_seq_delta(cmd->seq, last_seq);

		if (delta <= 0) {
			g_stale++;
			return false;   /* deliberately does NOT feed the failsafe */
		}
		if (delta > 1) {
			g_gaps += (uint32_t)(delta - 1);
		}
		g_last_gap_ms = (int32_t)(now - last_ms);
	}
	last_seq = cmd->seq;
	last_ms = now;
	g_seq = cmd->seq;
	g_flags = cmd->flags;
	g_frames++;
	g_link_up = true;

	/*
	 * ARMED is the sender's permission to drive. Anything else -- estop, the
	 * sender's own bench actuation timeout, its commitment window having
	 * expired, a build with motors inhibited -- is off, and is checked here as
	 * well as by the sender so that a single mistake on one side of the link
	 * cannot spin a motor. The blocking set is MOTOR_LINK_FLAGS_BLOCK, which
	 * lives in the shared header so a sender that gains a new reason to stop
	 * cannot diverge from a receiver that has never heard of it.
	 *
	 * The re-arm gate goes first: it has to SEE the not-armed frames (that
	 * is its evidence of a disarm), and it refuses the first ARMED frames
	 * after a boot or a kill (and a failsafe cut, if
	 * ESP_LINK_REARM_AFTER_CUT) until that disarm -> arm edge.
	 */
	if (!esp_rearm_frame(&g_rearm, cmd->flags, MOTOR_LINK_FLAG_ARMED,
			     MOTOR_LINK_FLAGS_BLOCK, g_kill)) {
		motors_off();
		return true;
	}
	if ((cmd->flags & MOTOR_LINK_FLAG_ARMED) == 0u ||
	    (cmd->flags & MOTOR_LINK_FLAGS_BLOCK) != 0u) {
		motors_off();
		return true;
	}
	/* The hardware watchdog starts at the first ARMED frame that may drive,
	 * not at link-thread start: the boot runs WiFi bring-up and PHY
	 * calibration asynchronously, and the dog only has to guard the time
	 * the gates can be driven. Once started it runs until the next reset. */
	if (g_wdt_rc == -EAGAIN) {
		/* -EAGAIN again while persist.c holds the dog for a flash erase:
		 * retried on the next frame. */
		g_wdt_rc = esp_safety_wdt_start();
	}
	/* Fail closed: a watchdog that could not start (no &wdt0, a build
	 * without CONFIG_WATCHDOG, an install/setup error) means no backstop for
	 * a hung link thread, so nothing drives. -EAGAIN is the flash-erase hold
	 * (the link thread cannot run during the IRQ-off erase anyway) and is
	 * retried. RBLINK shows wdt=err. */
	if (g_wdt_rc != 0 && g_wdt_rc != -EAGAIN) {
		motors_off();
		return true;
	}
	/* A failsafe cut, if one is latched, ends here: short -> the motors
	 * resume unlimited, >= SS_CUT_COLD_MS -> cold spin-up. Then store, and
	 * apply through the spin-up gate at once (the link thread also services
	 * it on every wake). */
	const uint32_t now_us = motors_now_us();

	ss_resume(&g_ss, now_us);
	motors_set_target(cmd->duty);
	motors_service(now_us);
	return true;
}

/* ---- the demux ------------------------------------------------------------
 *
 * One byte in; zero or one ASCII byte out, and duty frames consumed on the way.
 *
 * The sliding-window decoder already ignores everything that is not a frame, so
 * the frames are safe from the text. The text is NOT safe from the frames: a
 * duty field can contain 0x0A, so simply copying every byte into the line
 * assembler would both corrupt lines and split them. Hence the delay line --
 * the text is held back by exactly MOTOR_LINK_FRAME_LEN bytes, which is exactly
 * what the decoder needs in order to retract a frame it has just accepted.
 *
 * Cost: telemetry reaches the radio 14 byte-times (1.2 ms at 115200) later than
 * it otherwise would. Nothing downstream can tell.
 *
 * Correctness rests on one invariant from motor_link.h: an accepted frame IS
 * the last MOTOR_LINK_FRAME_LEN bytes fed to the window. The hold holds exactly
 * those same bytes, because a byte is only emitted (and thus lost from the
 * hold) when a 15th byte arrives -- i.e. strictly before the byte that could
 * complete the current frame.
 *
 * The one thing it gives up, stated rather than hidden: when the stream stops,
 * up to 14 bytes stay in the hold forever, so the final partial line before
 * silence is never delivered. A telemetry line is 190-286 bytes, so a line is
 * only ever DELAYED by the next line's arrival -- and a link that has gone quiet
 * has a much larger problem than a lost tail.
 */
struct demux {
	uint8_t hold[MOTOR_LINK_FRAME_LEN];
	uint8_t n;
};

static struct demux g_demux;   /* link thread only */

/* Returns true and sets *out when a byte has aged out of the hold and is
 * therefore known not to be part of a frame. *got_frame is set when this byte
 * completed a valid frame, in which case *cmd is filled. */
static bool demux_feed(struct demux *d, uint8_t byte, uint8_t *out,
		       struct motor_link_cmd *cmd, bool *got_frame)
{
	bool emit = false;

	*got_frame = false;

	if (d->n == MOTOR_LINK_FRAME_LEN) {
		*out = d->hold[0];
		memmove(&d->hold[0], &d->hold[1], MOTOR_LINK_FRAME_LEN - 1u);
		d->n--;
		emit = true;
	}
	d->hold[d->n++] = byte;

	if (motor_link_rx_feed(&g_rx, byte, cmd)) {
		d->n = 0;   /* the hold IS the frame: none of it is text */
		*got_frame = true;
	}
	return emit;
}

/* ---- the link thread -----------------------------------------------------
 * Everything safety-critical, in one thread that nothing but an ISR can
 * interrupt. It never calls printk (the console is a USB CDC endpoint that
 * blocks when no host drains it) and never touches a socket. */
static void link_thread(void *a, void *b, void *c)
{
	/* Starts already expired: nothing drives a gate before the first frame. */
	int64_t deadline = 0;
	bool kill_applied = false;
	bool load_applied = false;
	bool hold_applied = false;
	atomic_val_t kill_epoch = atomic_get(&g_kill_epoch);

	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	/* The hardware watchdog (esp_safety.c) is fed only from this loop, so a
	 * link thread that stops cycling resets the chip with the gates forced
	 * low. It is started by handle_frame() at the first ARMED frame that may
	 * drive (see there), and esp_safety_wdt_service() is a no-op until then. */

	for (;;) {
		uint8_t buf[64];
		uint32_t n;
		int64_t now;
		bool woke_downlink = false;

		/* Every ESTOP since the last pass, even one a RESET already cleared
		 * again, sets the re-arm hold -- BEFORE this pass's frames, so none
		 * of them can drive past it. */
		if (atomic_get(&g_kill_epoch) != kill_epoch) {
			kill_epoch = atomic_get(&g_kill_epoch);
			esp_rearm_hold(&g_rearm, ESP_REARM_KILL);
			motors_off();
		}

		while ((n = ring_buf_get(&uart_rb, buf, sizeof(buf))) > 0u) {
			for (uint32_t i = 0; i < n; i++) {
				struct motor_link_cmd cmd;
				uint8_t text;
				bool got_frame;

				if (demux_feed(&g_demux, buf[i], &text, &cmd,
					       &got_frame)) {
					if (ring_buf_put(&line_rb, &text, 1) == 1u) {
						g_text_bytes++;
					}
					woke_downlink = true;
				}
				if (got_frame) {
					now = k_uptime_get();
					if (handle_frame(&cmd, now) && g_link_up) {
						deadline = now + ESP_LINK_TIMEOUT_MS;
					}
				}
			}
		}
		if (woke_downlink) {
			k_sem_give(&line_ready);
		}

		now = k_uptime_get();
		if (g_link_up && now >= deadline) {
			motors_cut(motors_now_us());   /* zero, NOT cold: see there */
			g_link_up = false;
			g_failsafe_cuts++;
			esp_rearm_on_cut(&g_rearm, ESP_LINK_REARM_AFTER_CUT);
		}
		/* A kill that arrives between frames must not wait for the next
		 * one. motors_apply() already forces zero while latched; this
		 * edge check is what makes it take effect within one tick. */
		if (g_kill != kill_applied) {
			kill_applied = g_kill;
			if (kill_applied) {
				motors_off();
			}
		}
		/* Same for a load session starting between frames. */
		if (load_link_active() != load_applied) {
			load_applied = load_link_active();
			if (load_applied) {
				motors_off();
			}
		}
		/* And for a flash-erase hold granted at the end of the last pass
		 * (esp_safety_wdt_service below; motors_service blocks meanwhile). */
		if (esp_safety_wdt_held() != hold_applied) {
			hold_applied = esp_safety_wdt_held();
			if (hold_applied) {
				motors_off();
			}
		}

		/* Advance the spin-up gate. After the failsafe, kill and load
		 * checks, so a stale command is never written past the deadline
		 * (motors_off() or motors_cut() above has already zeroed the targets). */
		motors_service(motors_now_us());

		/* Watchdog feed (and a persist.c flash-erase hold request: granted
		 * here only after 1 s with no duty, which also blocks the gates
		 * until released), and the previous-run snapshot for the next
		 * boot's RBLINK prev_* fields (rate-limited inside). */
		esp_safety_wdt_service(esp_bridge_motors_recently_on());
		esp_safety_snap_store(g_duty, g_flags, g_kill, g_link_up, g_rearm.hold);

		/* Yield. A cooperative thread that never blocks starves the
		 * system; blocking with a timeout is what makes the failsafe
		 * independent of any byte ever arriving again. */
		(void)k_sem_take(&rx_sem, K_MSEC(ESP_LINK_TICK_MS));
	}
}

/* Decode the sender's flags into something readable. A hex byte is fine for a
 * log and useless to an operator deciding whether the thing in front of them is
 * about to spin. */
static const char *flags_str(uint8_t f, char *buf, size_t len)
{
	size_t n = 0;

	buf[0] = '\0';
	if (f & MOTOR_LINK_FLAG_ARMED)   { n += snprintk(buf + n, len - n, "ARMED "); }
	if (f & MOTOR_LINK_FLAG_COMMIT)  { n += snprintk(buf + n, len - n, "COMMITTED "); }
	if (f & MOTOR_LINK_FLAG_ESTOP)   { n += snprintk(buf + n, len - n, "ESTOP "); }
	if (f & MOTOR_LINK_FLAG_TIMEOUT) { n += snprintk(buf + n, len - n, "TIMEOUT "); }
	if (f & MOTOR_LINK_FLAG_INHIBIT) { n += snprintk(buf + n, len - n, "INHIBIT "); }
	if (f & MOTOR_LINK_FLAG_WINDOW)  { n += snprintk(buf + n, len - n, "WINDOW-EXPIRED "); }
	if (buf[0] == '\0') { (void)snprintk(buf, len, "off"); }
	return buf;
}

/*
 * The ESP's own health line, emitted to the console AND to every leased client.
 *
 * It answers the one question the FPGA's telemetry line structurally cannot:
 * "are the motors off because they were commanded off, or because the link
 * died?" The FPGA reports what it SENT; this reports what ARRIVED and what was
 * actually written to the gates after the local ceiling and the kill latch. On
 * a dashboard those are different colours.
 *
 *   RBLINK v1 ml=up up=12345 seq=17 fl=0x21[ARMED COMMITTED ]
 *          duty=[1000 1100 1050 1020] fr=1234 gap=20 lost=0 stale=0
 *          crc=0 noise=0 cuts=0 pwmerr=0 kill=0 txt=98765 cl=1
 *
 *   ml     up | down   -- down means the failsafe has cut, i.e. LINK DEAD
 *   fl     the sender's flag byte; ARMED clear with ml=up means COMMANDED OFF
 *   duty   per 10000, AS APPLIED here (post ceiling, post kill)
 *   crc    frames rejected by CRC; noise  bytes that never belonged to a frame
 *   kill   1 = this ESP has latched a remote kill; frames are being ignored
 *   txt    ASCII bytes forwarded to the radio, i.e. proof the demux is working
 *   ph     1 = PWM phase interleave built in, 0 = in-phase (hpoint 0)
 *   ss     spin-up gate phase per motor, motor 1 first: 0 COLD 1 WAIT 2 HOLD
 *          3 SLEW 4 RUN; ffff = gate compiled out (ESP_LINK_SOFTSTART=0)
 *   sslim  service ticks in which the gate held some output below its command
 */
static int status_line(char *buf, size_t len, int clients)
{
	char fbuf[80];

	return snprintk(buf, len,
			"RBLINK v1 ml=%s up=%lld seq=%u fl=0x%02x[%s] "
			"duty=[%u %u %u %u] fr=%u gap=%d lost=%u stale=%u "
			"crc=%u noise=%u cuts=%u pwmerr=%u kill=%u txt=%u cl=%d "
			"ph=%u ss=%04x sslim=%u\n",
			g_link_up ? "up" : "down", k_uptime_get(),
			(unsigned)g_seq, (unsigned)g_flags,
			flags_str(g_flags, fbuf, sizeof(fbuf)),
			(unsigned)g_duty[0], (unsigned)g_duty[1],
			(unsigned)g_duty[2], (unsigned)g_duty[3],
			(unsigned)g_frames, (int)g_last_gap_ms,
			(unsigned)g_gaps, (unsigned)g_stale,
			(unsigned)g_rx.bad_crc, (unsigned)g_rx.noise,
			(unsigned)g_failsafe_cuts, (unsigned)g_pwm_errs,
			g_kill ? 1u : 0u, (unsigned)g_text_bytes, clients,
			(unsigned)ESP_LINK_PWM_INTERLEAVE,
			ESP_LINK_SOFTSTART ? (unsigned)ss_phase_word(&g_ss) : 0xffffu,
			(unsigned)g_ss.lim_ticks);
}

/*
 * Reset and safety fields, appended AFTER cl= so every existing RBLINK parser
 * (rig_tui / riskybird-dash regexes anchor on "... txt=N cl=N") still matches:
 *
 *   rearm   the re-arm hold: 0 | boot | kill, and cut only in a build with
 *           ESP_LINK_REARM_AFTER_CUT=1 (by default a failsafe cut resumes on
 *           the next ARMED frame). Anything but 0 with fl=ARMED means the ESP
 *           is refusing ARMED frames until the sender disarms and arms again
 *           (esp_safety_core.h)
 *   rablk   ARMED frames refused by that hold since boot
 *   wdt     hardware watchdog: off (not started: it starts at the first ARMED
 *           frame allowed to drive) | <ms>ms (running) | held (paused for a
 *           flash erase; the gates are held at 0 meanwhile) | err (start
 *           failed). After an rb debug session this is not proof
 *           the dog is live: an OpenOCD halt likely disables the MWDTs
 *           (unverified) and the firmware cannot see that.
 *   rst     raw ROM reset reason of THIS boot, with a name; 0x01 = power-on
 *           OR chip brownout (the C6 cannot tell them apart)
 *   hw      Zephyr hwinfo RESET_* mask (what persist.c acts on)
 *   bootn   boots since RTC memory was last lost; rtc kept|cold says whether
 *           it survived this reset (cold = true power loss, or garbage)
 *   prev_fault  none | wdt/<thread>/up<ms> | fatal<reason>/pc<hex>/<thread>/up<ms>
 *   prev_*  the previous run's last snapshot (<= 10 ms old when it died):
 *           uptime, applied duties, flag byte, kill, link, re-arm hold
 */
static int status_tail(char *buf, size_t len, int n)
{
	if (n <= 0 || (size_t)n >= len) {
		return n;
	}
	if (buf[n - 1] == '\n') {
		n--;
	}
	n += snprintk(buf + n, len - n, " rearm=%s rablk=%u wdt=%s",
		      esp_rearm_hold_name(g_rearm.hold), (unsigned)g_rearm.blocked,
		      g_wdt_rc == 0 ? esp_safety_wdt_state() :
		      g_wdt_rc == -EAGAIN ? "off" : "err");
	if ((size_t)n < len) {
		n += esp_safety_boot_fields(buf + n, len - n);
	}
	if ((size_t)n < len) {
		n += esp_safety_prev_fields(buf + n, len - n);
	}
	if ((size_t)n < len) {
		n += snprintk(buf + n, len - n, "\n");
	}
	return n;
}

int esp_bridge_status(char *buf, size_t len)
{
	return status_tail(buf, len, status_line(buf, len, -1));
}

#else
bool esp_bridge_motors_recently_on(void)
{
	return false;   /* no gate in this image */
}

int esp_bridge_status(char *buf, size_t len)
{
	int n = snprintk(buf, len, "RBLINK none (no motor path in this image)");

	if (n > 0 && (size_t)n < len) {
		n += esp_safety_boot_fields(buf + n, len - n);
	}
	if (n > 0 && (size_t)n < len) {
		n += snprintk(buf + n, len - n, "\n");
	}
	return n;
}
#endif /* HAVE_MOTORS */

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

static void clients_now(struct client_list *cl)
{
	cl->count = 0;
	(void)net_dhcpv4_server_foreach_lease(net_if_get_wifi_sap(), lease_cb, cl);
}

static void send_to_clients(int sock, const struct client_list *cl,
			    const char *data, size_t len)
{
	for (int i = 0; i < cl->count; i++) {
		struct sockaddr_in dst = {
			.sin_family = AF_INET,
			.sin_port = htons(TELEM_PORT),
		};

		dst.sin_addr = cl->addr[i];
		(void)zsock_sendto(sock, data, len, 0,
				   (struct sockaddr *)&dst, sizeof(dst));
	}
}

/* ---- downlink -------------------------------------------------------------
 * Assemble complete lines and send one UDP datagram per line to each leased
 * client. One line per packet keeps the ground station's parser trivial and
 * means a lost packet costs exactly one sample. */
static void downlink_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	static char line[TELEM_LINE_MAX];
	size_t len = 0;
	bool overlong = false;
	int sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	if (sock < 0) {
		printk("esp_bridge: telemetry socket failed (%d)\n", errno);
		return;
	}

	for (;;) {
		uint8_t ch;

		if (ring_buf_get(DOWNLINK_RB, &ch, 1) != 1) {
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
#if HAVE_SENSOR_RELAY
		/* A relay flight controller asking for sensor frames: consumed here, not sent to the
		 * radio (the FC writes it once a second; the ground station has no use for it). */
		if (len == sizeof(RELAY_KEEPALIVE_VERB) - 1 &&
		    memcmp(line, RELAY_KEEPALIVE_VERB, len) == 0) {
			sensor_relay_keepalive();
			len = 0;
			continue;
		}
#endif
		line[len++] = '\n';

		struct client_list cl;

		clients_now(&cl);

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
		send_to_clients(sock, &cl, line, len);
		len = 0;
	}
}

#if HAVE_MOTORS
/* ---- status ---------------------------------------------------------------
 * Preemptible on purpose. It prints to a USB CDC console that can block when no
 * host is draining it, and it opens its own socket so a stalled send cannot sit
 * in front of the telemetry stream. */
static void status_thread(void *a, void *b, void *c)
{
	/* 640: the v1 line is ~230 B; the reset/safety tail (status_tail) adds
	 * up to ~260 more. A truncated line loses its newline and the tail. */
	char buf[640];
	int sock = -1;
	bool sock_err_said = false;

	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	for (;;) {
		struct client_list cl;
		int n;

		k_msleep(ESP_LINK_STATUS_MS);
		/* Opened here and retried, not once at thread start: this is the last
		 * socket the image opens, and with CONFIG_ZVFS_OPEN_MAX=4 the open
		 * failed with EMFILE -- after which every RBLINK was silently never
		 * sent, while the console copy below kept printing and looked fine. */
		if (sock < 0) {
			sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			if (sock < 0 && !sock_err_said) {
				sock_err_said = true;
				printk("esp_bridge: STATUS SOCKET FAILED (errno %d) -- RBLINK is "
				       "console-only, NOT reaching the ground station\n", errno);
			}
		}
		clients_now(&cl);
		n = status_tail(buf, sizeof(buf), status_line(buf, sizeof(buf), cl.count));
		if (n <= 0) {
			continue;
		}
		if (n > (int)sizeof(buf) - 1) {
			/* snprintk truncated: it wrote sizeof(buf) - 1 chars and the
			 * NUL, which must not go out in the datagram. */
			n = (int)sizeof(buf) - 1;
		}
		if (sock >= 0) {
			send_to_clients(sock, &cl, buf, (size_t)n);
		}
		if (!load_link_active()) {   /* the USB tunnel shares the console */
			printk("%s", buf);
		}
#if HAVE_SENSOR_RELAY
		/* Its own line, after RBLINK, so the RBLINK parsers see an unchanged line; every
		 * tenth tick (~2 s), so it does not crowd the console or the radio. */
		static unsigned relay_tick;

		n = (++relay_tick % 10u == 0u) ? sensor_relay_status(buf, sizeof(buf)) : 0;
		if (n > 0 && n < (int)sizeof(buf)) {
			if (sock >= 0) {
				send_to_clients(sock, &cl, buf, (size_t)n);
			}
			if (!load_link_active()) {
				printk("%s", buf);
			}
		}
#endif
	}
}
#endif /* HAVE_MOTORS */

/* ---- uplink ---------------------------------------------------------------
 * Commands in, straight out of the UART. The ACK is issued by the BRIDGE, not
 * relayed from the flight controller: the panel does a blocking recvfrom after
 * each send, and making that wait on a round trip through a 115200 link plus a
 * control loop would stall the dashboard on the one path that must never
 * stall -- the kill switch.
 *
 * READ THE ACK FOR WHAT IT IS. "ACK <cmd>" means DELIVERED TO THE FPGA's UART
 * and nothing more. Whether the flight controller acts on it depends on its
 * build: with ROSE_UART_CMD (every flight preset) it does; without it every
 * forwarded command is dropped while the ACK is still printed. ESTOP in the
 * motors build is also executed HERE, at the gates, whatever the FPGA runs, and
 * its ACK says so in different words so the two cannot be confused.
 */
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
		const char *local = "";

		if (n <= 0) {
			continue;
		}
		while (n > 0 && (cmd[n - 1] == '\n' || cmd[n - 1] == '\r')) {
			n--;
		}
		if (n == 0) {
			continue;
		}
#if HAVE_MOTORS
		/* Acted on HERE, before forwarding, because this is the only
		 * component in the chain that can actually stop a motor. */
		if (n == 5 && memcmp(cmd, "ESTOP", 5) == 0) {
			g_kill = true;
			atomic_inc(&g_kill_epoch);
			esp_safety_kill_store(true);   /* survives an ESP reset */
			local = " -- ESP GATES CUT AND LATCHED";
		} else if (n == 5 && memcmp(cmd, "RESET", 5) == 0) {
			/* No extra step is implied: the FC's own ESTOP-flagged (not
			 * armed) frames during the latch already satisfy the re-arm
			 * gate, so the FC's next ARMED frame applies. Only an FC that
			 * stayed ARMED through the whole latch has to disarm first,
			 * and RBLINK says so (rearm=kill, rablk counting). */
			g_kill = false;
			esp_safety_kill_store(false);
			local = " -- ESP kill latch CLEARED, frames apply again";
		}
#endif
		cmd[n] = '\n';
		bool delivered = fpga_uart_write(cmd, (size_t)n + 1);

		char ack[192];
		int alen = delivered ?
			snprintf(ack, sizeof(ack), "ACK %.*s%s\n", n, cmd, local) :
			snprintf(ack, sizeof(ack), "BUSY %.*s -- program load in progress, "
				 "NOT delivered to the FPGA%s\n", n, cmd, local);

		if (alen > 0) {
			if (alen > (int)sizeof(ack)) {
				alen = (int)sizeof(ack);
			}
			(void)zsock_sendto(sock, ack, (size_t)alen, 0,
					   (struct sockaddr *)&src, srclen);
		}
		cmd[n] = '\0';
		if (delivered) {
			printk("esp_bridge: cmd -> FPGA: %s%s\n", cmd, local);
		}
	}
}

K_THREAD_STACK_DEFINE(downlink_stack, 6144);
K_THREAD_STACK_DEFINE(uplink_stack, 4096);
static struct k_thread downlink_thread_data;
static struct k_thread uplink_thread_data;
#if HAVE_MOTORS
K_THREAD_STACK_DEFINE(link_stack, 2048);
K_THREAD_STACK_DEFINE(status_stack, 4096);
static struct k_thread link_thread_data;
static struct k_thread status_thread_data;
#endif

/* ---- WiFi -----------------------------------------------------------------  */
static void wifi_event(struct net_mgmt_event_callback *cb, uint64_t event,
		       struct net_if *iface)
{
	ARG_UNUSED(cb);
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
	 * defaults to (--drone 192.168.4.1).
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

/* ---- scheduler audit ------------------------------------------------------
 *
 * Prints, once, every thread in the system with its priority, and a verdict on
 * the one property the motors build's safety argument rests on: that the link
 * thread is the ONLY cooperative thread, so nothing but an ISR can hold the CPU
 * against the failsafe.
 *
 * This exists because that argument is otherwise unfalsifiable from the bench.
 * The networking stack creates its threads from Kconfig-derived priorities that
 * depend on options set three layers away (see the header comment), a Zephyr
 * bump can change any of them silently, and the symptom of getting it wrong is
 * a failsafe that is late exactly when the radio is busy -- which is exactly
 * when nobody is watching the console. A printed list is cheap and checkable by
 * eye in one second.
 *
 * k_thread_foreach_unlocked, not k_thread_foreach: the locked variant holds a
 * spinlock across the callback, and printing from there to a USB CDC console
 * that may not be drained is a way to hang at boot. Rows are collected under
 * the walk and printed after it.
 */
#define AUDIT_MAX 24
struct audit_row {
	const char *name;
	int prio;
};
static struct audit_row audit_rows[AUDIT_MAX];
static int audit_n;

static void audit_cb(const struct k_thread *thread, void *user_data)
{
	ARG_UNUSED(user_data);

	if (audit_n >= AUDIT_MAX) {
		return;
	}
	audit_rows[audit_n].name = k_thread_name_get((k_tid_t)thread);
	audit_rows[audit_n].prio = k_thread_priority_get((k_tid_t)thread);
	audit_n++;
}

static void sched_audit(const char *critical)
{
	int coop_others = 0;

	audit_n = 0;
	k_thread_foreach_unlocked(audit_cb, NULL);

	printk("esp_bridge: SCHED AUDIT -- %d thread(s)\n", audit_n);
	for (int i = 0; i < audit_n; i++) {
		const char *name = audit_rows[i].name;
		int prio = audit_rows[i].prio;
		bool coop = prio < 0;
		bool is_critical = (critical != NULL) && (name != NULL) &&
				   (strcmp(name, critical) == 0);

		printk("  %-16s prio %3d %s%s\n", name ? name : "(unnamed)",
		       prio, coop ? "COOP" : "preempt",
		       is_critical ? "   <- failsafe thread" : "");
		if (coop && !is_critical) {
			coop_others++;
		}
	}
	if (critical == NULL) {
		printk("esp_bridge: no motor path in this image -- scheduling is "
		       "not safety-critical here\n");
	} else if (coop_others == 0) {
		printk("esp_bridge: VERDICT OK -- '%s' is the only cooperative "
		       "thread; nothing but an ISR can delay the failsafe\n",
		       critical);
	} else {
		printk("esp_bridge: VERDICT WARN -- %d other cooperative "
		       "thread(s). A cooperative thread is NEVER preempted, not "
		       "even by a higher-priority cooperative one, so each of "
		       "those CAN delay the failsafe. Demote them, or build with "
		       "CONFIG_NUM_METAIRQ_PRIORITIES=1.\n",
		       coop_others);
	}
}

int main(void)
{
	printk("\n=== RiskyBird ESP link: FPGA uart <-> WiFi UDP%s ===\n",
	       HAVE_MOTORS ? " + 4x motor gates" : "");
	/* Reset cause, boot counter and the previous run's RTC records, before
	 * anything else can overwrite them. */
	esp_safety_boot();

#if HAVE_MOTORS
	printk("esp_bridge: MOTORS BUILD -- this image DRIVES the gates. "
	       "MOTOR1=GPIO21 MOTOR2=GPIO20 MOTOR3=GPIO23 MOTOR4=GPIO22, "
	       "%u ns period (%u kHz), active-high\n",
	       MOTOR_PERIOD_NSEC, 1000000U / MOTOR_PERIOD_NSEC);
	printk("esp_bridge: failsafe %d ms, tick %d ms, duty ceiling %u%%\n",
	       ESP_LINK_TIMEOUT_MS, ESP_LINK_TICK_MS, ESP_LINK_MAX_DUTY_PCT);
	printk("esp_bridge: PWM interleave %s\n", ESP_LINK_PWM_INTERLEAVE ?
	       "ON (hpoint_i = i*(P-1-duty)/3, no Lpoint wrap)" :
	       "OFF (all four in phase, hpoint 0)");
	if (ESP_LINK_SOFTSTART) {
		printk("esp_bridge: spin-up gate ON -- idle %u/10000 hold %u ms, "
		       "slew %u/10000 per s, stagger %u ms, max %u ms; cold at "
		       "boot, disarm, kill and load, and after a failsafe cut "
		       ">= %u ms (a shorter cut resumes unlimited)\n",
		       (unsigned)SS_IDLE, (unsigned)SS_HOLD_MS,
		       (unsigned)SS_RATE_PER_S, (unsigned)SS_STAGGER_MS,
		       (unsigned)SS_MAX_MS, (unsigned)SS_CUT_COLD_MS);
	} else {
		printk("esp_bridge: spin-up gate OFF -- commands applied as received\n");
	}
	printk("esp_bridge: workloads/esp_motors and workloads/motor_safe claim "
	       "these same pins -- only one image can be flashed\n");

	for (unsigned i = 0; i < MOTOR_LINK_NMOTORS; i++) {
		if (!device_is_ready(motors[i].dev)) {
			printk("esp_bridge: PWM device for motor%u not ready -- "
			       "REFUSING to run\n", i + 1U);
			return -ENODEV;
		}
	}
	/*
	 * Zero the outputs before anything else can command them and before the
	 * UART is enabled. The window before this line is already safe and not by
	 * luck: pwm_led_esp32_init() configures every DT-declared channel and
	 * stops it (sig_out_en = 0, idle level = the `inverted` property, which
	 * none of these four set) BEFORE applying pinctrl, so each pad is driven
	 * to a static LOW from the moment the LEDC signal reaches it, over the
	 * carrier's 10k gate pulldown.
	 */
	motors_off();

	/* Re-arm hold from boot, and an ESTOP latched before this reset stays
	 * latched. Both before the link thread exists. */
	esp_rearm_init(&g_rearm);
	if (esp_safety_kill_restored()) {
		g_kill = true;
		printk("esp_bridge: ESP KILL LATCH RESTORED from RTC memory (an ESTOP "
		       "before this reset was never cleared) -- send RESET to clear\n");
	}
	printk("esp_bridge: re-arm gate ON -- after boot or ESTOP%s, ARMED applies "
	       "only after a disarm -> arm edge%s; watchdog %d ms (hardware backstop "
	       "%d ms), started at the first ARMED frame allowed to drive\n",
	       ESP_LINK_REARM_AFTER_CUT ? " or a failsafe cut" : "",
	       ESP_LINK_REARM_AFTER_CUT ? "" : " (a failsafe cut resumes on the next "
	       "ARMED frame: ESP_LINK_REARM_AFTER_CUT=0)",
	       ESP_LINK_WDT_MS, 2 * ESP_LINK_WDT_MS);
#endif

	if (!device_is_ready(fpga_uart)) {
		printk("esp_bridge: fpga-uart not ready\n");
		return -ENODEV;
	}
	ring_buf_init(&uart_rb, sizeof(rb_buf), rb_buf);
#if HAVE_MOTORS
	ring_buf_init(&line_rb, sizeof(line_rb_buf), line_rb_buf);
#endif

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

	/* Program loading into the FPGA SoC: TCP for WiFi, plus the USB console
	 * as a tunnel for bench tests without joining the drone's network. */
	(void)load_link_start(fpga_uart, DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_console)));

#if HAVE_MOTORS
	/* Highest priority in the application, and created BEFORE the radio
	 * threads so it is already running by the time anything can queue
	 * network work.
	 *
	 * The UART IRQ has been filling uart_rb since before softap_start(),
	 * which can take seconds; on a full ring the NEWEST bytes are dropped.
	 * Frames in that backlog would be stamped with the drain time and treated
	 * as fresh, so start the link thread on an empty ring: the next frame
	 * the FC sends is the first one acted on. */
	{
		const unsigned int key = irq_lock();

		ring_buf_reset(&uart_rb);
		irq_unlock(key);
	}
	k_thread_create(&link_thread_data, link_stack,
			K_THREAD_STACK_SIZEOF(link_stack),
			link_thread, NULL, NULL, NULL,
			K_PRIO_COOP(0), 0, K_NO_WAIT);
	k_thread_name_set(&link_thread_data, "motorlink");
#endif

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

#if HAVE_MOTORS
	k_thread_create(&status_thread_data, status_stack,
			K_THREAD_STACK_SIZEOF(status_stack),
			status_thread, NULL, NULL, NULL,
			K_PRIO_PREEMPT(7), 0, K_NO_WAIT);
	k_thread_name_set(&status_thread_data, "status");
#endif

#if HAVE_SENSOR_RELAY
	/* Preemptible, below the motor link; idle (and off the bus) until a keepalive arrives. */
	sensor_relay_start();
#endif

	/* After every thread exists, so the list is the steady-state one. */
	k_msleep(100);
	sched_audit(HAVE_MOTORS ? "motorlink" : NULL);

	return 0;
}
