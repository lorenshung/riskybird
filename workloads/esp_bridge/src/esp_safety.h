/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Crash safety and reset diagnostics for esp_bridge. See esp_safety.c.
 */
#ifndef RISKYBIRD_ESP_BRIDGE_ESP_SAFETY_H
#define RISKYBIRD_ESP_BRIDGE_ESP_SAFETY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_safety_core.h"

/* Motors build: hardware watchdog stage-0 timeout (interrupt -> gates low ->
 * reset). The hardware-only reset backstop is twice this. 1 MWDT tick = 1 ms
 * (TG0 WDT clock = XTAL, prescaler 40000 in wdt_esp32.c). */
#ifndef ESP_LINK_WDT_MS
#define ESP_LINK_WDT_MS 250
#endif

/* Once, first thing in main(): reads the ROM reset reason, the hwinfo cause and
 * the RTC records left by the previous run, bumps the boot counter and prints
 * one summary line. Everything below reports what this captured. */
void esp_safety_boot(void);

/* True if the previous run left the ESP kill latch SET in RTC memory (an
 * ESTOP that no RESET has cleared). main() restores g_kill from it. */
bool esp_safety_kill_restored(void);

/* Record the kill latch in RTC memory. From the uplink thread, on ESTOP (true)
 * and on the explicit RESET (false). */
void esp_safety_kill_store(bool killed);

/* Raw ROM reset reason (soc_reset_reason_t) and the hwinfo RESET_* mask. */
uint32_t esp_safety_reset_raw(void);
uint32_t esp_safety_reset_hwinfo(void);
const char *esp_safety_reset_name(uint32_t raw);

/* Appends " rst=0x.. (name) hw=0x.. bootn=N rtc=kept|cold prev_fault=..."
 * (both builds) to buf. Returns the snprintk count. */
int esp_safety_boot_fields(char *buf, size_t len);

/* ---- motors build only (no-ops / zeros in the default build) ------------- */

/* Link thread only, every loop: saves the previous-run snapshot (uptime,
 * applied duties, flags, kill, link, re-arm hold) to RTC memory, at most once
 * every ESP_SAFETY_SNAP_MS. */
void esp_safety_snap_store(const volatile uint16_t duty[4], uint8_t flags,
			   bool kill, bool link_up, uint8_t hold);

/* Appends " prev_up=.. prev_duty=[....] prev_fl=0x.. prev_kill=. prev_ml=.."
 * describing the snapshot the previous run left, or " prev_up=none". */
int esp_safety_prev_fields(char *buf, size_t len);

/* Hardware watchdog on TG0 MWDT. Link thread only: start once, at the first
 * ARMED frame allowed to drive (main.c handle_frame; -EAGAIN while a hold is
 * requested or granted, retried on the next frame), then service on every
 * loop pass (feeds, and grants/releases a hold request -- see below; that
 * part runs before the start too).
 * `motors_on` is esp_bridge_motors_recently_on(). */
int esp_safety_wdt_start(void);
void esp_safety_wdt_service(bool motors_on);

/* From any thread EXCEPT the link thread: ask for the watchdog to be paused
 * around a long interrupts-off operation (a flash erase), started or not.
 * The link thread grants it only after 1 s with no duty on any gate; while
 * held, the dog is disabled or cannot start, and the gates are held at 0.
 * Waits <= 100 ms. on=true returns true = granted, i.e. safe to erase (false:
 * do NOT erase); always release with on=false, granted or not. Without a
 * watchdog (default build) it returns `on` at once. */
bool esp_safety_wdt_hold(bool on);

/* True while a hold is granted: motors_service() holds every gate at 0. */
bool esp_safety_wdt_held(void);

/* "off" | "<ms>ms" | "held" -- for the RBLINK line. */
const char *esp_safety_wdt_state(void);

#endif
