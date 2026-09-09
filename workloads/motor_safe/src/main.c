/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Passive owner for the ESP32-C6 side of RiskyBird's shared motor nets.
 */

#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define MOTOR_SAFE_HOG DT_NODELABEL(riskybird_motor_safe_hog)

#define MOTOR_SAFE_SPEC(index)                                            \
	{                                                                  \
		.port = DEVICE_DT_GET(DT_PARENT(MOTOR_SAFE_HOG)),            \
		.pin = DT_GPIO_HOG_PIN_BY_IDX(MOTOR_SAFE_HOG, index),        \
		.dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(MOTOR_SAFE_HOG, index), \
	}

static const struct gpio_dt_spec motor_pins[] = {
	MOTOR_SAFE_SPEC(0),
	MOTOR_SAFE_SPEC(1),
	MOTOR_SAFE_SPEC(2),
	MOTOR_SAFE_SPEC(3),
};

static unsigned int park_esp_motor_pins(bool verbose)
{
	unsigned int safe = 0U;

	for (size_t i = 0; i < ARRAY_SIZE(motor_pins); i++) {
		int err = gpio_pin_configure_dt(&motor_pins[i], GPIO_INPUT);

		if (verbose || err != 0) {
			printk("motor%u ESP GPIO%u input+pulldown: %s (%d)\n",
			       (unsigned)i + 1U, (unsigned)motor_pins[i].pin,
			       err == 0 ? "OK" : "FAIL", err);
		}
		if (err == 0) {
			safe++;
		}
	}
	return safe;
}

int main(void)
{
	unsigned int safe = park_esp_motor_pins(true);

	printk("MOTOR SAFE: %u/4 ESP drivers high-impedance with pull-downs\n", safe);
	if (safe != ARRAY_SIZE(motor_pins)) {
		printk("MOTOR SAFE FAILED -- do not arm the FPGA sweep\n");
	}

	for (;;) {
		k_sleep(K_SECONDS(5));
		safe = park_esp_motor_pins(false);
		printk("MOTOR SAFE heartbeat: %u/4 inputs+pulldowns\n", safe);
	}

	return 0;
}
