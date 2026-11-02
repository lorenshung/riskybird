/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Remote program loading through the FPGA link. See load_link.c.
 */
#ifndef RISKYBIRD_ESP_BRIDGE_LOAD_LINK_H
#define RISKYBIRD_ESP_BRIDGE_LOAD_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>

/* TCP port of the WiFi load service. */
#define LOAD_LINK_TCP_PORT 14552

/* True from the moment a load session claims the FPGA wire until the FPGA's
 * mux has idled back to uart1. While true: every byte from the FPGA belongs to
 * the session (hand it to load_link_rx_from_fpga, not the telemetry path),
 * nothing else may write the FPGA UART, and the motor gates must read zero.
 * ISR-safe. */
bool load_link_active(void);

/* From the FPGA UART ISR while load_link_active(). */
void load_link_rx_from_fpga(const uint8_t *buf, size_t n);

/* Every writer of the FPGA UART outside a session holds this lock while it
 * writes, so a session never starts its break in the middle of someone else's
 * line. Never blocks for a session: false means one has the wire (or is taking
 * it), and the caller must drop what it wanted to send, not queue it -- queued
 * bytes would reach whatever program the session just loaded. */
bool load_link_tx_trylock(void);
void load_link_tx_unlock(void);

/* Provided by main.c: true while, or within 1 s after, any motor gate carried
 * a nonzero duty. Every session is refused then. */
bool esp_bridge_motors_recently_on(void);

/* Provided by main.c: the current RBLINK health line, newline-terminated
 * (what the motor frames last said, e.g. fl=0x20 COMMITTED). */
int esp_bridge_status(char *buf, size_t len);

/* Provided by main.c: uart1 bytes received outside sessions since boot. */
uint32_t esp_bridge_uart1_rx_bytes(void);

/* Starts the session thread: TCP :LOAD_LINK_TCP_PORT and the USB tunnel.
 * `usb` may be NULL (no tunnel). */
int load_link_start(const struct device *fpga_uart, const struct device *usb);

#endif
