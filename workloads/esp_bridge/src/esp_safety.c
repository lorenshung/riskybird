/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Crash safety and reset diagnostics for esp_bridge.
 *
 * ---------------------------------------------------------------------------
 * 1. FATAL ERROR -> GATES LOW -> RESET   (both builds; gates only in motors)
 * ---------------------------------------------------------------------------
 * Zephyr's default k_sys_fatal_error_handler halts. On this chip that is the
 * worst possible outcome: LEDC is free-running hardware, so a halted CPU leaves
 * all four gates at the last duty (up to the build's ceiling) while the 120 ms
 * failsafe, being a thread deadline, has died with the CPU. The override below
 *
 *   a. routes GPIO20-23 through the GPIO matrix to the plain GPIO_OUT bit, set
 *      LOW and output-enabled, then stops LEDC channels 0-3 at idle-low --
 *      register writes only, no kernel call, no driver semaphore, always_inline
 *      into an IRAM function so it works with the flash cache off;
 *   b. records the fault in RTC memory (kind, reason, mepc, uptime, thread);
 *   c. sets the IDF reset hint ESP_RST_PANIC and calls esp_restart_noos().
 *
 * esp_restart_noos() is a CPU-only reset (ROM reason 0x0C, RESET_REASON_CPU0_SW).
 * Its esp_system_reset_modules_on_exit() resets MSPI, UART0/1, SYSTIMER, GDMA,
 * SDIO slave, MCPWM, ETM and the modem -- NOT LEDC and NOT the GPIO/IO_MUX
 * block. So the GPIO-matrix override from (a) stays in force through the ROM
 * and the boot code, until pwm_led_esp32_init() applies pinctrl again -- with
 * every channel already stopped at idle-low (main.c's comment above
 * motors_off()). That property is read out of the hal source, not measured.
 * The hint makes the next boot's esp_reset_reason() ESP_RST_PANIC, i.e. hwinfo
 * RESET_CPU_LOCKUP, so persist.c does NOT treat it as a power-on.
 *
 * THE BOUND, stated honestly. The gates are forced low INSIDE this handler, so
 * the guarantee covers only faults that reach it. Zephyr runs its own code
 * first (exception entry, z_riscv_fatal_error, z_fatal_error -- flash
 * resident; conf/motors.conf sets CONFIG_EXCEPTION_DEBUG=n to drop the
 * register/stack dump from that path). If that path itself faults or hangs and
 * the handler is never reached, the only backstop is the watchdog's hardware
 * stage-1 reset at 2 x ESP_LINK_WDT_MS = 500 ms -- and only once the watchdog
 * is armed, i.e. after the first ARMED frame allowed to drive (section 2).
 * Before that, nothing has driven a gate since boot.
 *
 * No esp_rom_printf anywhere in here: the ROM console is UART0, and UART0 IS
 * the FPGA link (GPIO16 -> FPGA uart1 RX). No LOG_PANIC either: it can stall on
 * a USB CDC console nobody drains.
 *
 * ---------------------------------------------------------------------------
 * 2. HARDWARE WATCHDOG   (motors build: &wdt0 okay + CONFIG_WATCHDOG)
 * ---------------------------------------------------------------------------
 * TG0 MWDT, fed only from the motor-link thread. Started by the link thread at
 * the first ARMED frame the re-arm gate lets drive -- not at boot, where the
 * asynchronous WiFi bring-up and PHY calibration run -- and never stopped after
 * that (except the flash-erase hold below). RBLINK wdt=off until then.
 * Zephyr's wdt_esp32 programs
 * stage 0 = interrupt and stage 1 = the reset action, both at window.max, and
 * its ISR calls wdt_hal_handle_intr(), which FEEDS the dog -- so with a NULL
 * callback a stuck link thread would merely get fed every timeout, forever.
 * The callback here therefore does exactly what the fatal handler does (gates
 * low, record, hint ESP_RST_TASK_WDT, esp_restart_noos()). Timing:
 *
 *   ESP_LINK_WDT_MS (250)     stage 0: ISR -> gates low + reset. Covers a link
 *                             thread that is stuck, spinning or deadlocked
 *                             while interrupts still run.
 *   2 x ESP_LINK_WDT_MS       stage 1: hardware SYSTEM reset (ROM 0x07,
 *                             CORE_MWDT0), only reached if interrupts are
 *                             masked the whole time. A system reset DOES reset
 *                             LEDC and GPIO, so the pads fall back to their
 *                             IO_MUX reset state (see below), not forced-low.
 *
 * Units: the driver passes window.max straight through as MWDT ticks with
 * prescaler 40000. TG0's WDT clock select (PCR tg0_wdt_clk_sel) resets to 0 =
 * XTAL 40 MHz and nothing in this tree changes it, so 1 tick = 1 ms. The boot
 * line prints the register so the bench can confirm it ("wdtclk=0").
 *
 * Flash erases run with interrupts and cache off for a whole sector/64 KiB
 * block (esp_flash_erase_region brackets each op with spi1_start/end, which
 * irq_lock()s). A slow block erase could outlast the timeout, so persist.c
 * brackets every erase with esp_safety_wdt_hold() and does NOT erase unless
 * it is granted. The link thread grants it only after 1 s with no duty on any
 * gate -- the same window as load_link's motors-running refusal -- and, while
 * it is held, (a) the dog is disabled (or, if it was never started, cannot
 * be: esp_safety_wdt_start() returns -EAGAIN) and (b) motors_service() holds
 * every gate at 0 (esp_safety_wdt_held()). Single core + cooperative link
 * thread: between the grant and the release no duty can appear, so no erase
 * ever runs with the dog armed, nor with a gate driven while the failsafe is
 * stalled. Flash WRITES (page programs, ~1 ms with IRQs off each) need no
 * hold; persist.c refuses them while motors run.
 *
 * A JTAG halt (rb debug) also stops the feeding. OpenOCD's esp32c6 target
 * likely disables the MWDTs on halt (UNVERIFIED). If so, the dog may stay
 * disabled after the session resumes, and RBLINK wdt=<ms>ms -- which only
 * reports that this firmware started it -- cannot prove it is still live.
 * Power-cycle or reset after an rb debug session before trusting it.
 *
 * GPIO20-23 at reset: they are the SDIO_DATA0-3 pads (IO_MUX function 0 =
 * SDIO, function 1 = GPIO). The hal headers in this tree do not record the
 * IO_MUX reset pull configuration; whether a weak pull-up is on during reset is
 * UNVERIFIED here -- check the C6 datasheet IO MUX table and scope a gate
 * through a stage-1 / USB / power-on reset. With the carrier's 10 k pulldown a
 * ~45 k WPU would sit near 0.6 V at the SI2302 gate.
 *
 * ---------------------------------------------------------------------------
 * 3. RTC NOINIT RECORDS
 * ---------------------------------------------------------------------------
 * Four records in .rtc_noinit (LP SRAM), format in esp_safety_core.h:
 *   boot   boot counter
 *   snap   the running state, saved by the link thread every ESP_SAFETY_SNAP_MS
 *   fault  written by the fatal handler / watchdog callback
 *   kill   the ESP kill latch, written only on ESTOP and on the explicit RESET
 * They survive every reset that leaves LP SRAM powered (CPU/system/WDT/USB). On
 * a true power-on they are garbage and fail their check: "rtc=cold". Whether
 * they survive the C6's chip-level brownout (ROM 0x01, indistinguishable from
 * power-on by reason code) depends on how far the rail fell -- that is exactly
 * the evidence rtc=kept vs rtc=cold on an rst=0x01 boot provides. Not measured.
 */

#include "esp_safety.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/fatal.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#include <esp_attr.h>
#include <esp_system.h>
#include <esp_rom_sys.h>
#include <esp_private/system_internal.h>
#include <soc/soc.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_sig_map.h>
#include <soc/ledc_struct.h>
#include <soc/pcr_struct.h>

#define HAVE_MOTORS DT_NODE_EXISTS(DT_ALIAS(pwm_motor0))

#if HAVE_MOTORS && defined(CONFIG_WATCHDOG) && DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(wdt0))
#define HAVE_WDT 1
#include <zephyr/drivers/watchdog.h>
#else
#define HAVE_WDT 0
#endif

#ifndef ESP_SAFETY_SNAP_MS
#define ESP_SAFETY_SNAP_MS 10
#endif

/* ---- RTC records ----------------------------------------------------------- */

static RTC_NOINIT_ATTR volatile uint32_t rtc_boot[BOOT_NW];
static RTC_NOINIT_ATTR volatile uint32_t rtc_snap[SNAP_NW];
static RTC_NOINIT_ATTR volatile uint32_t rtc_fault[FAULT_NW];
static RTC_NOINIT_ATTR volatile uint32_t rtc_kill[KILL_NW];

/* What esp_safety_boot() captured. Read-only after boot. */
static uint32_t s_raw;           /* soc_reset_reason_t */
static uint32_t s_hw;            /* hwinfo RESET_* */
static uint32_t s_boots;
static bool     s_rtc_kept;
static bool     s_kill_restored;
static bool     s_prev_snap_ok;
static uint32_t s_prev_snap[SNAP_NW];
static bool     s_prev_fault_ok;
static uint32_t s_prev_fault[FAULT_NW];
static uint32_t s_wdt_clk_sel;

/* ---- the gates, with no kernel underneath -------------------------------- */

#if HAVE_MOTORS
#include <riskybird/motor_link.h>

/* Pins are the routed PCB's (esp_bridge-motors.overlay); channels are read
 * from the same devicetree the driver uses. Immediates only: this runs from
 * IRAM with the cache possibly off, so no lookup tables in flash rodata. */
#define GATE_PIN_LOW(pin)                                                       \
	do {                                                                    \
		REG_WRITE(GPIO_OUT_W1TC_REG, 1u << (pin));                      \
		REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (pin) * 4u, SIG_GPIO_OUT_IDX); \
		REG_WRITE(GPIO_ENABLE_W1TS_REG, 1u << (pin));                   \
	} while (0)

#define LEDC_STOP(ch)                                                           \
	do {                                                                    \
		LEDC.channel_group[0].channel[(ch)].conf0.idle_lv = 0;          \
		LEDC.channel_group[0].channel[(ch)].conf0.sig_out_en = 0;       \
		LEDC.channel_group[0].channel[(ch)].conf0.para_up = 1;          \
	} while (0)

static inline __attribute__((always_inline)) void gates_force_low(void)
{
	GATE_PIN_LOW(21);   /* MOTOR1 */
	GATE_PIN_LOW(20);   /* MOTOR2 */
	GATE_PIN_LOW(23);   /* MOTOR3 */
	GATE_PIN_LOW(22);   /* MOTOR4 */
	LEDC_STOP(DT_PWMS_CHANNEL(DT_ALIAS(pwm_motor0)));
	LEDC_STOP(DT_PWMS_CHANNEL(DT_ALIAS(pwm_motor1)));
	LEDC_STOP(DT_PWMS_CHANNEL(DT_ALIAS(pwm_motor2)));
	LEDC_STOP(DT_PWMS_CHANNEL(DT_ALIAS(pwm_motor3)));
}
#else
/* Default build: the gates belong to the FPGA (47 R in parallel on the
 * carrier) and this image hogs the pins as inputs. Driving them here would
 * fight the FPGA, so a fatal error only resets. */
static inline __attribute__((always_inline)) void gates_force_low(void)
{
}
#endif

/* ---- fault record + reset ---------------------------------------------------- */

static volatile uint32_t s_fault_depth;

static void fault_record(uint32_t kind, uint32_t reason, uint32_t pc)
{
	const char *name = k_thread_name_get(k_current_get());
	uint32_t nm[2] = { 0, 0 };

	if (name != NULL) {
		strncpy((char *)nm, name, sizeof(nm));
	}
	rtc_fault[FAULT_W_KIND] = kind;
	rtc_fault[FAULT_W_REASON] = reason;
	rtc_fault[FAULT_W_PC] = pc;
	rtc_fault[FAULT_W_UP_MS] = k_uptime_get_32();
	rtc_fault[FAULT_W_NAME0] = nm[0];
	rtc_fault[FAULT_W_NAME1] = nm[1];
	esp_rec_seal(rtc_fault, FAULT_NW, ESP_REC_FAULT_MAGIC);
}

/* Gates first, then the (flash-resident, possibly faulting) diagnostics, then
 * the reset. A fault inside fault_record() re-enters here with depth != 0 and
 * goes straight to the reset. */
static void IRAM_ATTR __attribute__((noreturn))
stop_and_restart(uint32_t kind, uint32_t reason, uint32_t pc, esp_reset_reason_t hint)
{
	gates_force_low();
	if (s_fault_depth++ == 0u) {
		fault_record(kind, reason, pc);
	}
	esp_reset_reason_set_hint(hint);
	esp_restart_noos();
	for (;;) {
	}
}

void IRAM_ATTR k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	stop_and_restart(ESP_FAULT_FATAL, reason,
			 esf != NULL ? (uint32_t)esf->mepc : 0u, ESP_RST_PANIC);
}

/* ---- boot ------------------------------------------------------------------- */

const char *esp_safety_reset_name(uint32_t raw)
{
	switch (raw) {
	case 0x01: return "por/bod";    /* RESET_REASON_CHIP_POWER_ON == CHIP_BROWN_OUT */
	case 0x03: return "sw-sys";
	case 0x05: return "deepsleep";
	case 0x06: return "sdio";
	case 0x07: return "mwdt0-sys";
	case 0x08: return "mwdt1-sys";
	case 0x09: return "rtcwdt-sys";
	case 0x0B: return "mwdt0-cpu";
	case 0x0C: return "sw-cpu";
	case 0x0D: return "rtcwdt-cpu";
	case 0x0F: return "bod-sys";
	case 0x10: return "rtcwdt-rtc";
	case 0x11: return "mwdt1-cpu";
	case 0x12: return "superwdt";
	case 0x14: return "efuse-crc";
	case 0x15: return "usb-uart";
	case 0x16: return "usb-jtag";
	case 0x18: return "jtag-cpu";
	default:   return "unknown";
	}
}

static void rec_copy(uint32_t *dst, const volatile uint32_t *src, unsigned n)
{
	for (unsigned i = 0; i < n; i++) {
		dst[i] = src[i];
	}
}

void esp_safety_boot(void)
{
	s_raw = (uint32_t)esp_rom_get_reset_reason(0);
	(void)hwinfo_get_reset_cause(&s_hw);
	s_wdt_clk_sel = PCR.timergroup0_wdt_clk_conf.tg0_wdt_clk_sel;

	s_rtc_kept = esp_rec_valid(rtc_boot, BOOT_NW, ESP_REC_BOOT_MAGIC);
	s_boots = s_rtc_kept ? rtc_boot[BOOT_W_COUNT] + 1u : 1u;
	rtc_boot[BOOT_W_COUNT] = s_boots;
	esp_rec_seal(rtc_boot, BOOT_NW, ESP_REC_BOOT_MAGIC);

	/* Previous run's records: copy to RAM, then invalidate, so a record is
	 * only ever reported by the boot right after the run that wrote it. */
	s_prev_snap_ok = esp_rec_valid(rtc_snap, SNAP_NW, ESP_REC_SNAP_MAGIC);
	if (s_prev_snap_ok) {
		rec_copy(s_prev_snap, rtc_snap, SNAP_NW);
	}
	rtc_snap[SNAP_W_MAGIC] = 0u;
	s_prev_fault_ok = esp_rec_valid(rtc_fault, FAULT_NW, ESP_REC_FAULT_MAGIC);
	if (s_prev_fault_ok) {
		rec_copy(s_prev_fault, rtc_fault, FAULT_NW);
	}
	rtc_fault[FAULT_W_MAGIC] = 0u;

	/* The kill latch is NOT invalidated: it holds until the explicit RESET
	 * or a power-on that loses LP SRAM. Every boot rewrites it, so with LP
	 * SRAM kept (valid boot record) an invalid kill record means a store
	 * torn by a reset -- and the store most likely to be torn is the ESTOP's
	 * KILLED=1, made from the preemptible uplink thread. Fail closed: restore
	 * killed. Only a cold RTC (power-on garbage) defaults to not killed. */
	if (esp_rec_valid(rtc_kill, KILL_NW, ESP_REC_KILL_MAGIC)) {
		s_kill_restored = rtc_kill[KILL_W_KILLED] != 0u;
	} else if (s_rtc_kept) {
		s_kill_restored = true;
		esp_safety_kill_store(true);
		printk("esp_safety: kill record torn with RTC kept -> killed (RESET clears)\n");
	} else {
		esp_safety_kill_store(false);
	}

	char line[160];

	(void)esp_safety_boot_fields(line, sizeof(line));
	printk("esp_safety: boot%s kill=%u wdtclk=%u\n", line,
	       s_kill_restored ? 1u : 0u, (unsigned)s_wdt_clk_sel);
}

bool esp_safety_kill_restored(void)
{
	return s_kill_restored;
}

void esp_safety_kill_store(bool killed)
{
	rtc_kill[KILL_W_KILLED] = killed ? 1u : 0u;
	esp_rec_seal(rtc_kill, KILL_NW, ESP_REC_KILL_MAGIC);
}

uint32_t esp_safety_reset_raw(void)
{
	return s_raw;
}

uint32_t esp_safety_reset_hwinfo(void)
{
	return s_hw;
}

static int fault_str(char *buf, size_t len)
{
	char name[9];

	if (!s_prev_fault_ok) {
		return snprintk(buf, len, "none");
	}
	memcpy(name, &s_prev_fault[FAULT_W_NAME0], 8);
	name[8] = '\0';
	for (char *p = name; *p != '\0'; p++) {
		if (*p == ' ' || *p == '=') {
			*p = '_';
		}
	}
	if (s_prev_fault[FAULT_W_KIND] == ESP_FAULT_WDT) {
		return snprintk(buf, len, "wdt/%s/up%u", name[0] ? name : "?",
				(unsigned)s_prev_fault[FAULT_W_UP_MS]);
	}
	return snprintk(buf, len, "fatal%u/pc%08x/%s/up%u",
			(unsigned)s_prev_fault[FAULT_W_REASON],
			(unsigned)s_prev_fault[FAULT_W_PC], name[0] ? name : "?",
			(unsigned)s_prev_fault[FAULT_W_UP_MS]);
}

int esp_safety_boot_fields(char *buf, size_t len)
{
	char f[64];

	(void)fault_str(f, sizeof(f));
	return snprintk(buf, len, " rst=0x%02x(%s) hw=0x%x bootn=%u rtc=%s prev_fault=%s",
			(unsigned)s_raw, esp_safety_reset_name(s_raw), (unsigned)s_hw,
			(unsigned)s_boots, s_rtc_kept ? "kept" : "cold", f);
}

/* ---- motors build: snapshot and watchdog ------------------------------------ */

#if HAVE_MOTORS
void esp_safety_snap_store(const volatile uint16_t duty[4], uint8_t flags,
			   bool kill, bool link_up, uint8_t hold)
{
	static uint32_t last;
	uint32_t now = k_uptime_get_32();

	if (last != 0u && (uint32_t)(now - last) < ESP_SAFETY_SNAP_MS) {
		return;
	}
	last = now != 0u ? now : 1u;
	rtc_snap[SNAP_W_UP_MS] = now;
	rtc_snap[SNAP_W_DUTY01] = (uint32_t)duty[0] | ((uint32_t)duty[1] << 16);
	rtc_snap[SNAP_W_DUTY23] = (uint32_t)duty[2] | ((uint32_t)duty[3] << 16);
	rtc_snap[SNAP_W_STATE] = esp_snap_state(flags, kill, link_up, hold);
	esp_rec_seal(rtc_snap, SNAP_NW, ESP_REC_SNAP_MAGIC);
}

int esp_safety_prev_fields(char *buf, size_t len)
{
	if (!s_prev_snap_ok) {
		return snprintk(buf, len, " prev_up=none");
	}
	uint32_t st = s_prev_snap[SNAP_W_STATE];

	return snprintk(buf, len,
			" prev_up=%u prev_duty=[%u %u %u %u] prev_fl=0x%02x prev_kill=%u"
			" prev_ml=%s prev_hold=%s",
			(unsigned)s_prev_snap[SNAP_W_UP_MS],
			(unsigned)(s_prev_snap[SNAP_W_DUTY01] & 0xffffu),
			(unsigned)(s_prev_snap[SNAP_W_DUTY01] >> 16),
			(unsigned)(s_prev_snap[SNAP_W_DUTY23] & 0xffffu),
			(unsigned)(s_prev_snap[SNAP_W_DUTY23] >> 16),
			(unsigned)(st & 0xffu), (unsigned)((st >> 8) & 1u),
			((st >> 9) & 1u) ? "up" : "down",
			esp_rearm_hold_name((uint8_t)((st >> 12) & 0x7u)));
}
#else
void esp_safety_snap_store(const volatile uint16_t duty[4], uint8_t flags,
			   bool kill, bool link_up, uint8_t hold)
{
	ARG_UNUSED(duty); ARG_UNUSED(flags); ARG_UNUSED(kill);
	ARG_UNUSED(link_up); ARG_UNUSED(hold);
}

int esp_safety_prev_fields(char *buf, size_t len)
{
	return snprintk(buf, len, " prev_up=none");
}
#endif

#if HAVE_WDT
static const struct device *const wdt_dev = DEVICE_DT_GET(DT_NODELABEL(wdt0));
static atomic_t wdt_hold_req = ATOMIC_INIT(0);
static atomic_t wdt_held = ATOMIC_INIT(0);
static bool wdt_running;   /* link thread only */
static atomic_t wdt_started = ATOMIC_INIT(0);

/* Stage 0 expired: the link thread has not fed for ESP_LINK_WDT_MS. ISR
 * context (the driver's TG0 WDT interrupt). Never returns. */
static void wdt_expired(const struct device *dev, int channel_id)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);
	stop_and_restart(ESP_FAULT_WDT, 0u, 0u, ESP_RST_TASK_WDT);
}

int esp_safety_wdt_start(void)
{
	/* Not under a flash-erase hold: the caller retries on the next frame. */
	if (atomic_get(&wdt_hold_req) != 0 || atomic_get(&wdt_held) != 0) {
		return -EAGAIN;
	}
	struct wdt_timeout_cfg cfg = {
		.window = { .min = 0u, .max = ESP_LINK_WDT_MS },
		.callback = wdt_expired,
		.flags = WDT_FLAG_RESET_SOC,
	};
	int rc;

	if (!device_is_ready(wdt_dev)) {
		return -ENODEV;
	}
	rc = wdt_install_timeout(wdt_dev, &cfg);
	if (rc < 0) {
		return rc;
	}
	rc = wdt_setup(wdt_dev, 0);
	if (rc < 0) {
		return rc;
	}
	wdt_running = true;
	atomic_set(&wdt_started, 1);
	return 0;
}

/* Runs whether or not the dog has started: a hold also has to keep an
 * unstarted dog from starting (and the gates at 0) for the erase. */
void esp_safety_wdt_service(bool motors_on)
{
	bool req = atomic_get(&wdt_hold_req) != 0;
	bool held = atomic_get(&wdt_held) != 0;

	if (req && !held && !motors_on) {
		if (wdt_running) {
			(void)wdt_disable(wdt_dev);
			wdt_running = false;
		}
		atomic_set(&wdt_held, 1);
	} else if (held && (!req || motors_on)) {
		/* Released. (motors_on cannot become true while held --
		 * motors_service() blocks every gate -- but if it ever did, the
		 * dog comes back at once.) */
		if (atomic_get(&wdt_started)) {
			(void)wdt_setup(wdt_dev, 0);
			wdt_running = true;
		}
		atomic_set(&wdt_held, 0);
	}
	if (wdt_running) {
		(void)wdt_feed(wdt_dev, 0);
	}
}

bool esp_safety_wdt_hold(bool on)
{
	atomic_set(&wdt_hold_req, on ? 1 : 0);
	/* The link thread services the request within one tick (5 ms). */
	for (int i = 0; i < 20; i++) {
		if ((atomic_get(&wdt_held) != 0) == on) {
			break;
		}
		k_msleep(5);
	}
	return atomic_get(&wdt_held) != 0;
}

bool esp_safety_wdt_held(void)
{
	return atomic_get(&wdt_held) != 0;
}

const char *esp_safety_wdt_state(void)
{
	if (atomic_get(&wdt_held)) {
		return "held";
	}
	return atomic_get(&wdt_started) ? STRINGIFY(ESP_LINK_WDT_MS) "ms" : "off";
}
#else
int esp_safety_wdt_start(void)
{
	return -ENOTSUP;
}

void esp_safety_wdt_service(bool motors_on)
{
	ARG_UNUSED(motors_on);
}

/* No watchdog and (in the default build) no gates: an erase is always safe
 * as far as this file is concerned; persist.c still refuses while motors run. */
bool esp_safety_wdt_hold(bool on)
{
	return on;
}

bool esp_safety_wdt_held(void)
{
	return false;
}

const char *esp_safety_wdt_state(void)
{
	return "off";
}
#endif
