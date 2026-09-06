/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * PMW3901 Optical Flow Sensor Test
 *
 * This sample initializes and tests the PMW3901 optical flow sensor
 * connected via SPI.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#include "pmw3901.h"
#include <zephyr/devicetree.h>

#define PMW3901_WIRING_NODE DT_NODELABEL(riskybird_pmw3901)

BUILD_ASSERT(DT_NODE_HAS_COMPAT(PMW3901_WIRING_NODE, riskybird_pmw3901),
	     "target overlay must define riskybird_pmw3901 wiring");

static const struct gpio_dt_spec pmw3901_cs =
	GPIO_DT_SPEC_GET(PMW3901_WIRING_NODE, cs_gpios);
static const struct gpio_dt_spec pmw3901_reset =
	GPIO_DT_SPEC_GET(PMW3901_WIRING_NODE, reset_gpios);
static const struct gpio_dt_spec pmw3901_led =
	GPIO_DT_SPEC_GET(PMW3901_WIRING_NODE, led_gpios);

/* Device configuration and data structures - initialized at runtime */
static struct pmw3901_config pmw3901_cfg;
static struct pmw3901_data pmw3901_data;

/* Create a simple device structure */
static const struct device pmw3901_device = {
    .name = "PMW3901",
    .config = &pmw3901_cfg,
    .data = &pmw3901_data,
};

#define PMW3901_DEV (&pmw3901_device)

/* Initialize PMW3901 configuration at runtime */
static int pmw3901_init_config(void)
{
    pmw3901_cfg.spi.bus =
	    DEVICE_DT_GET(DT_PHANDLE(PMW3901_WIRING_NODE, spi_bus));
    /* PMW3901 uses SPI mode 3 (CPOL=1, CPHA=1) */
    pmw3901_cfg.spi.config.operation =
        SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB;
    pmw3901_cfg.spi.config.frequency = 2000000;  /* 2MHz */
    pmw3901_cfg.spi.config.slave = 0;

    pmw3901_cfg.cs_gpio = pmw3901_cs;
    pmw3901_cfg.reset_gpio = pmw3901_reset;
    pmw3901_cfg.led_gpio = pmw3901_led;

    return 0;
}

int main(void)
{
    int ret;
    motionBurst_t motion;
    int16_t deltaX, deltaY;

    printf("PMW3901 Optical Flow Sensor Test\n");
    printf("================================\n\n");

    /* Initialize PMW3901 configuration */
    ret = pmw3901_init_config();
    if (ret != 0) {
        printf("ERROR: Failed to initialize PMW3901 configuration (ret: %d)\n", ret);
        return 1;
    }

    /* Check if SPI and GPIO devices are ready */
    if (!device_is_ready(pmw3901_cfg.spi.bus)) {
        printf("ERROR: SPI device is not ready\n");
        return 1;
    }
    if (!device_is_ready(pmw3901_cfg.cs_gpio.port)) {
        printf("ERROR: GPIO device is not ready\n");
        return 1;
    }

    printf("SPI and GPIO devices ready\n\n");

    /* Initialize the PMW3901 sensor */
    printf("Initializing PMW3901...\n");
    ret = pmw3901_init(PMW3901_DEV);
    if (ret != 0) {
        printf("ERROR: Failed to initialize PMW3901 (ret: %d)\n", ret);
        return 1;
    }
    printf("PMW3901 initialized successfully.\n\n");

    /* Continuously read motion data */
    printf("Entering continuous motion read loop...\n");
    printf("Format: deltaX, deltaY, SQUAL, Shutter\n\n");

    while (1) {
        /* Read motion burst */
        ret = pmw3901_read_motion_burst(PMW3901_DEV, &motion);
        if (ret != 0) {
            printf("ERROR: Failed to read motion burst (ret: %d)\n", ret);
        } else {
            printf("Motion: deltaX=%5d, deltaY=%5d, SQUAL=%3u, Shutter=%5u",
                   motion.deltaX, motion.deltaY, motion.squal, motion.shutter);

            if (motion.motionOccured) {
                printf(" [MOTION]");
            }
            printf("\n");
        }

        /* Also read motion count for comparison */
        ret = pmw3901_read_motion_count(PMW3901_DEV, &deltaX, &deltaY);
        if (ret == 0) {
            printf("  Count: deltaX=%5d, deltaY=%5d\n", deltaX, deltaY);
        }

        k_msleep(100); /* Poll every 100ms */
    }

    return 0;
}
