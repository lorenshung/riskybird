/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Sensor relay (motors build, ESP_SENSOR_RELAY): see sensor_relay.c.
 */
#ifndef ESP_BRIDGE_SENSOR_RELAY_H
#define ESP_BRIDGE_SENSOR_RELAY_H

#include <stdbool.h>
#include <stddef.h>

/* Start the relay threads. They idle until the first keepalive; call once from main(). */
void sensor_relay_start(void);

/* The FC asked for frames (its RELAY_KEEPALIVE_LINE arrived). Any thread. */
void sensor_relay_keepalive(void);

/* One status line (with its newline) for the status thread to print and send. */
int sensor_relay_status(char *buf, size_t len);

/* main.c's TX path to the FPGA: whole buffer under the TX lock, false while a load session
 * owns the wire. */
bool esp_bridge_fpga_write(const char *s, size_t len);

#endif /* ESP_BRIDGE_SENSOR_RELAY_H */
