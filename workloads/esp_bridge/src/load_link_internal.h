/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared between load_link.c (sessions on the FPGA link) and persist.c
 * (power-on boot policy and the stashed program). Not a public API.
 */
#ifndef RISKYBIRD_ESP_BRIDGE_LOAD_LINK_INTERNAL_H
#define RISKYBIRD_ESP_BRIDGE_LOAD_LINK_INTERNAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum kind { K_LOAD, K_POKE, K_PING, K_FLASH, K_BOOT, K_STAT, K_STASH, K_POLICY, K_AUTO };

struct request {
	enum kind kind;
	bool fast;
	uint32_t addr;
	uint8_t mode;     /* BOOT: ICAP sequence, see EspLinkIprog (0 = IPROG) */
	char args[48];
};

/* Claim the FPGA wire (motors read zero, uplink refused, RX diverted). */
void ll_claim(void);
/* Give it back; end_break: hand the mux back to uart1 with a bare break. */
void ll_release(bool end_break);
/* Break + key (+ boot address) + ack. LOAD also waits out the SoC reset. */
bool ll_handshake(enum kind kind, uint32_t addr, uint32_t *id, bool *id_valid);
void ll_write(const uint8_t *b, size_t n);
size_t ll_read(uint8_t *b, size_t n, int timeout_ms);

/* The host transport of the current session (sock NULL for USB). */
int ll_send(int *sock, bool is_usb, const char *s);
int ll_recv(int *sock, bool is_usb, uint8_t *buf, size_t max);

/* persist.c */
void persist_session(int *sock, bool is_usb, const struct request *rq);
void persist_start(void);

#endif
