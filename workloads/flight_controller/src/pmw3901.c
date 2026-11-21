/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * PMW3901 Optical Flow Sensor Driver Implementation
 * Ported from ESP-IDF to Zephyr
 */

#include "pmw3901.h"
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(pmw3901, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * gpio_dt_spec values are LOGICAL, not raw electrical levels:
 * - value=1 means "active" (asserted)
 * - value=0 means "inactive" (deasserted)
 *
 * Since our CS is configured GPIO_ACTIVE_LOW, asserting CS drives the pin low.
 */
static void cs_deassert(const struct device *dev)
{
    const struct pmw3901_config *config = dev->config;
    gpio_pin_set_dt(&config->cs_gpio, 0);
}

static void cs_assert(const struct device *dev)
{
    const struct pmw3901_config *config = dev->config;
    gpio_pin_set_dt(&config->cs_gpio, 1);
}

static void busy_wait_us(int microseconds)
{
    k_busy_wait(microseconds);
}

uint8_t pmw3901_register_read(const struct device *dev, uint8_t reg)
{
    const struct pmw3901_config *config = dev->config;
    uint8_t data = 0;
    uint8_t tx_data = reg & ~0x80u;  /* Set MSB to 0 for read */
    uint8_t dummy = 0;

    struct spi_buf tx_buf = {
        .buf = &tx_data,
        .len = 1,
    };
    struct spi_buf_set tx_bufs = {
        .buffers = &tx_buf,
        .count = 1,
    };

    struct spi_buf rx_buf = {
        .buf = &data,
        .len = 1,
    };
    struct spi_buf_set rx_bufs = {
        .buffers = &rx_buf,
        .count = 1,
    };

    cs_assert(dev);
    busy_wait_us(50);

    /* Send register address */
    spi_write_dt(&config->spi, &tx_bufs);
    busy_wait_us(500);  /* Longer delay after sending register address */

    /* Read data (send dummy byte) */
    tx_buf.buf = &dummy;
    spi_transceive_dt(&config->spi, &tx_bufs, &rx_bufs);

    busy_wait_us(50);
    cs_deassert(dev);
    k_msleep(1);

    return data;
}

int pmw3901_register_write(const struct device *dev, uint8_t reg, uint8_t value)
{
    const struct pmw3901_config *config = dev->config;
    uint8_t data[2] = { (uint8_t)(reg | 0x80u), value };  /* Set MSB to 1 for write */

    struct spi_buf tx_buf = {
        .buf = data,
        .len = 2,
    };
    struct spi_buf_set tx_bufs = {
        .buffers = &tx_buf,
        .count = 1,
    };

    cs_assert(dev);
    busy_wait_us(50);

    spi_write_dt(&config->spi, &tx_bufs);

    busy_wait_us(50);
    cs_deassert(dev);
    k_msleep(1);

    return 0;
}

/* Register-init variant (bench A/B): 0 = Bitcraze / Crazyflie firmware (default), 1 = PixArt PMW3901MB
 * demo-kit v3.20 as used by PX4 (adds a per-chip trim preamble, 0x74=0x21, and no 0x5A/0x54 tail),
 * 2 = PAW3902 Low-Light mode table, 3 = PAW3902 Super-Low-Light (50 fps) table. */
#define PMW_DELAY 0xFF
/* PAW3902 mode tables, verbatim from PX4 src/drivers/optical_flow/paw3902/PAW3902.cpp (BSD-3), LED write omitted.
 * {PMW_DELAY, ms} entries are sleeps. Bench A/B on the PMW3901 (same Product_ID 0x49). */
static const uint8_t paw3902_low_light[][2] = {
    {0x7F, 0x00}, {0x55, 0x01}, {0x50, 0x07}, {0x7F, 0x0E}, {0x43, 0x10}, {0x48, 0x02},
    {0x7F, 0x00}, {0x51, 0x7B}, {0x50, 0x00}, {0x55, 0x00}, {0x7F, 0x00}, {0x61, 0xAD},
    {0x7F, 0x03}, {0x40, 0x00}, {0x7F, 0x05}, {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14},
    {0x5F, 0x34}, {0x7B, 0x08}, {0x5E, 0x34}, {0x5B, 0x65}, {0x6D, 0x65}, {0x45, 0x17},
    {0x70, 0xE5}, {0x71, 0xE5}, {0x7F, 0x06}, {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F},
    {0x7F, 0x08}, {0x66, 0x44}, {0x65, 0x20}, {0x6A, 0x3A}, {0x61, 0x05}, {0x62, 0x05},
    {0x7F, 0x09}, {0x4F, 0xAF}, {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77}, {0x5F, 0x40},
    {0x60, 0x78}, {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50}, {0x7F, 0x0A}, {0x45, 0x60},
    {0x7F, 0x00}, {0x4D, 0x11}, {0x55, 0x80}, {0x74, 0x21}, {0x75, 0x1F}, {0x4A, 0x78},
    {0x4B, 0x78}, {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xFE}, {0x65, 0x1F}, {0x72, 0x0A},
    {0x73, 0x00}, {0x7F, 0x14}, {0x44, 0x84}, {0x65, 0x67}, {0x66, 0x18}, {0x63, 0x70},
    {0x6F, 0x2C}, {0x7F, 0x15}, {0x48, 0x48}, {0x7F, 0x07}, {0x41, 0x0D}, {0x43, 0x14},
    {0x4B, 0x0E}, {0x45, 0x0F}, {0x44, 0x42}, {0x4C, 0x80}, {0x7F, 0x10}, {0x5B, 0x03},
    {0x7F, 0x07}, {0x40, 0x41}, {PMW_DELAY, 10}, {0x7F, 0x00}, {0x32, 0x00}, {0x7F, 0x07},
    {0x40, 0x40}, {0x7F, 0x06}, {0x68, 0x70}, {0x69, 0x01}, {0x7F, 0x0D}, {0x48, 0xC0},
    {0x6F, 0xD5}, {0x7F, 0x00}, {0x5B, 0xA0}, {0x4E, 0xA8}, {0x5A, 0x50}, {0x40, 0x80},
    {0x73, 0x1F}, {PMW_DELAY, 10}, {0x73, 0x00},
};
static const uint8_t paw3902_super_low_light[][2] = {
    {0x7F, 0x00}, {0x55, 0x01}, {0x50, 0x07}, {0x7F, 0x0E}, {0x43, 0x10}, {0x48, 0x04},
    {0x7F, 0x00}, {0x51, 0x7B}, {0x50, 0x00}, {0x55, 0x00}, {0x7F, 0x00}, {0x61, 0xAD},
    {0x7F, 0x03}, {0x40, 0x00}, {0x7F, 0x05}, {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14},
    {0x5F, 0x34}, {0x7B, 0x08}, {0x5E, 0x34}, {0x5B, 0x32}, {0x6D, 0x32}, {0x45, 0x17},
    {0x70, 0xE5}, {0x71, 0xE5}, {0x7F, 0x06}, {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F},
    {0x7F, 0x08}, {0x66, 0x44}, {0x65, 0x20}, {0x6A, 0x3A}, {0x61, 0x05}, {0x62, 0x05},
    {0x7F, 0x09}, {0x4F, 0xAF}, {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77}, {0x5F, 0x40},
    {0x60, 0x78}, {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50}, {0x7F, 0x0A}, {0x45, 0x60},
    {0x7F, 0x00}, {0x4D, 0x11}, {0x55, 0x80}, {0x74, 0x21}, {0x75, 0x1F}, {0x4A, 0x78},
    {0x4B, 0x78}, {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xCE}, {0x65, 0x0B}, {0x72, 0x0A},
    {0x73, 0x00}, {0x7F, 0x14}, {0x44, 0x84}, {0x65, 0x67}, {0x66, 0x18}, {0x63, 0x70},
    {0x6F, 0x2C}, {0x7F, 0x15}, {0x48, 0x48}, {0x7F, 0x07}, {0x41, 0x0D}, {0x43, 0x14},
    {0x4B, 0x0E}, {0x45, 0x0F}, {0x44, 0x42}, {0x4C, 0x80}, {0x7F, 0x10}, {0x5B, 0x02},
    {0x7F, 0x07}, {0x40, 0x41}, {PMW_DELAY, 25}, {0x7F, 0x00}, {0x32, 0x44}, {0x7F, 0x07},
    {0x40, 0x40}, {0x7F, 0x06}, {0x68, 0x40}, {0x69, 0x02}, {0x7F, 0x0D}, {0x48, 0xC0},
    {0x6F, 0xD5}, {0x7F, 0x00}, {0x5B, 0xA0}, {0x4E, 0xA8}, {0x5A, 0x50}, {0x40, 0x80},
    {0x73, 0x0B}, {PMW_DELAY, 25}, {0x73, 0x00},
};

static void write_table(const struct device *dev, const uint8_t (*t)[2], size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if (t[i][0] == PMW_DELAY) { k_msleep(t[i][1]); }
        else { pmw3901_register_write(dev, t[i][0], t[i][1]); }
    }
}

static int g_init_variant;
void pmw3901_set_init_variant(int v) { g_init_variant = v; }
int  pmw3901_get_init_variant(void) { return g_init_variant; }

static void init_preamble_v320(const struct device *dev)
{
    pmw3901_register_write(dev, 0x7F, 0x00);
    pmw3901_register_write(dev, 0x55, 0x01);
    pmw3901_register_write(dev, 0x50, 0x07);
    pmw3901_register_write(dev, 0x7F, 0x0E);
    pmw3901_register_write(dev, 0x43, 0x10);
    uint8_t v = pmw3901_register_read(dev, 0x67);
    pmw3901_register_write(dev, 0x48, (v & 0x80) ? 0x04 : 0x02);
    pmw3901_register_write(dev, 0x7F, 0x00);
    pmw3901_register_write(dev, 0x51, 0x7B);
    pmw3901_register_write(dev, 0x50, 0x00);
    pmw3901_register_write(dev, 0x55, 0x00);
    pmw3901_register_write(dev, 0x7F, 0x0E);
    uint8_t c0 = pmw3901_register_read(dev, 0x73);
    uint8_t c1 = 0, c2 = 0;
    if (c0 == 0) {
        c1 = pmw3901_register_read(dev, 0x70);
        c1 = (c1 <= 28) ? c1 + 14 : c1 + 11;
        if (c1 > 0x3F) { c1 = 0x3F; }
        c2 = pmw3901_register_read(dev, 0x71);
        c2 = (uint8_t)(((unsigned)c2 * 45) / 100);
        pmw3901_register_write(dev, 0x7F, 0x00);
        pmw3901_register_write(dev, 0x61, 0xAD);
        pmw3901_register_write(dev, 0x51, 0x70);
        pmw3901_register_write(dev, 0x7F, 0x0E);
        pmw3901_register_write(dev, 0x70, c1);
        pmw3901_register_write(dev, 0x71, c2);
    }
    printk("PMW3901 v3.20 preamble: reg67=0x%02X reg73=0x%02X c1=%u c2=%u\n", v, c0, c1, c2);
}

static void init_registers(const struct device *dev)
{
    if (g_init_variant == 2) {
        write_table(dev, paw3902_low_light, ARRAY_SIZE(paw3902_low_light));
        return;
    }
    if (g_init_variant == 3) {
        write_table(dev, paw3902_super_low_light, ARRAY_SIZE(paw3902_super_low_light));
        return;
    }
    if (g_init_variant == 1) {
        init_preamble_v320(dev);
    }
    pmw3901_register_write(dev, 0x7F, 0x00);
    pmw3901_register_write(dev, 0x61, 0xAD);
    pmw3901_register_write(dev, 0x7F, 0x03);
    pmw3901_register_write(dev, 0x40, 0x00);
    pmw3901_register_write(dev, 0x7F, 0x05);
    pmw3901_register_write(dev, 0x41, 0xB3);
    pmw3901_register_write(dev, 0x43, 0xF1);
    pmw3901_register_write(dev, 0x45, 0x14);
    pmw3901_register_write(dev, 0x5B, 0x32);
    pmw3901_register_write(dev, 0x5F, 0x34);
    pmw3901_register_write(dev, 0x7B, 0x08);
    pmw3901_register_write(dev, 0x7F, 0x06);
    pmw3901_register_write(dev, 0x44, 0x1B);
    pmw3901_register_write(dev, 0x40, 0xBF);
    pmw3901_register_write(dev, 0x4E, 0x3F);
    pmw3901_register_write(dev, 0x7F, 0x08);
    pmw3901_register_write(dev, 0x65, 0x20);
    pmw3901_register_write(dev, 0x6A, 0x18);
    pmw3901_register_write(dev, 0x7F, 0x09);
    pmw3901_register_write(dev, 0x4F, 0xAF);
    pmw3901_register_write(dev, 0x5F, 0x40);
    pmw3901_register_write(dev, 0x48, 0x80);
    pmw3901_register_write(dev, 0x49, 0x80);
    pmw3901_register_write(dev, 0x57, 0x77);
    pmw3901_register_write(dev, 0x60, 0x78);
    pmw3901_register_write(dev, 0x61, 0x78);
    pmw3901_register_write(dev, 0x62, 0x08);
    pmw3901_register_write(dev, 0x63, 0x50);
    pmw3901_register_write(dev, 0x7F, 0x0A);
    pmw3901_register_write(dev, 0x45, 0x60);
    pmw3901_register_write(dev, 0x7F, 0x00);
    pmw3901_register_write(dev, 0x4D, 0x11);
    pmw3901_register_write(dev, 0x55, 0x80);
    pmw3901_register_write(dev, 0x74, g_init_variant == 1 ? 0x21 : 0x1F);
    pmw3901_register_write(dev, 0x75, 0x1F);
    pmw3901_register_write(dev, 0x4A, 0x78);
    pmw3901_register_write(dev, 0x4B, 0x78);
    pmw3901_register_write(dev, 0x44, 0x08);
    pmw3901_register_write(dev, 0x45, 0x50);
    pmw3901_register_write(dev, 0x64, 0xFF);
    pmw3901_register_write(dev, 0x65, 0x1F);
    pmw3901_register_write(dev, 0x7F, 0x14);
    pmw3901_register_write(dev, 0x65, 0x67);
    pmw3901_register_write(dev, 0x66, 0x08);
    pmw3901_register_write(dev, 0x63, 0x70);
    pmw3901_register_write(dev, 0x7F, 0x15);
    pmw3901_register_write(dev, 0x48, 0x48);
    pmw3901_register_write(dev, 0x7F, 0x07);
    pmw3901_register_write(dev, 0x41, 0x0D);
    pmw3901_register_write(dev, 0x43, 0x14);
    pmw3901_register_write(dev, 0x4B, 0x0E);
    pmw3901_register_write(dev, 0x45, 0x0F);
    pmw3901_register_write(dev, 0x44, 0x42);
    pmw3901_register_write(dev, 0x4C, 0x80);
    pmw3901_register_write(dev, 0x7F, 0x10);
    pmw3901_register_write(dev, 0x5B, 0x02);
    pmw3901_register_write(dev, 0x7F, 0x07);
    pmw3901_register_write(dev, 0x40, 0x41);
    pmw3901_register_write(dev, 0x70, 0x00);

    k_msleep(10);

    pmw3901_register_write(dev, 0x32, 0x44);
    pmw3901_register_write(dev, 0x7F, 0x07);
    pmw3901_register_write(dev, 0x40, 0x40);
    pmw3901_register_write(dev, 0x7F, 0x06);
    pmw3901_register_write(dev, 0x62, 0xF0);
    pmw3901_register_write(dev, 0x63, 0x00);
    pmw3901_register_write(dev, 0x7F, 0x0D);
    pmw3901_register_write(dev, 0x48, 0xC0);
    pmw3901_register_write(dev, 0x6F, 0xD5);
    pmw3901_register_write(dev, 0x7F, 0x00);
    pmw3901_register_write(dev, 0x5B, 0xA0);
    pmw3901_register_write(dev, 0x4E, 0xA8);
    pmw3901_register_write(dev, 0x5A, 0x50);
    pmw3901_register_write(dev, 0x40, 0x80);

    if (g_init_variant == 1) {
        return;
    }
    pmw3901_register_write(dev, 0x7F, 0x00);
    pmw3901_register_write(dev, 0x5A, 0x10);
    pmw3901_register_write(dev, 0x54, 0x00);
}

int pmw3901_init(const struct device *dev)
{
    const struct pmw3901_config *config = dev->config;
    struct pmw3901_data *data = dev->data;
    uint8_t chip_id, inv_chip_id;

    if (!spi_is_ready_dt(&config->spi)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&config->cs_gpio)) {
        LOG_ERR("CS GPIO not ready");
        return -ENODEV;
    }

    /* Configure CS pin */
    gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);

    /* Configure optional reset pin if present */
    if (config->reset_gpio.port) {
        if (!gpio_is_ready_dt(&config->reset_gpio)) {
            LOG_ERR("Reset GPIO not ready");
            return -ENODEV;
        }
        gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
        /* Reset sequence: assert (active-low), then deassert */
        gpio_pin_set_dt(&config->reset_gpio, 1);
        k_msleep(10);
        gpio_pin_set_dt(&config->reset_gpio, 0);
        k_msleep(10);
    }

    /* Configure optional LED pin if present */
    if (config->led_gpio.port) {
        if (gpio_is_ready_dt(&config->led_gpio)) {
            gpio_pin_configure_dt(&config->led_gpio, GPIO_OUTPUT_INACTIVE);
        }
    }

    k_msleep(40);

    /* CS toggle sequence */
    cs_deassert(dev);
    k_msleep(2);
    cs_assert(dev);
    k_msleep(2);
    cs_deassert(dev);
    k_msleep(2);
    cs_assert(dev);
    k_msleep(2);

    /*
     * Discard the first read. On this bench the first register read after the
     * CS dance above comes back bit-reversed -- 0x92 where 0x49 is expected,
     * which is exactly 0x49 with its bits in the other order -- while the very
     * next read (0x5F) returns a correct 0xB6. A global LSB-first mismatch
     * would reverse both, and 0xB6 reversed would be 0x6D, so it is the first
     * transaction that is misaligned rather than the bus.
     *
     * Ported from workloads/bringup/pmw3901_test/src/pmw3901.c, which is the copy of
     * this driver the FPGA bring-up used and where the effect was measured.
     */
    (void)pmw3901_register_read(dev, 0x00);

    /* Read chip ID */
    chip_id = pmw3901_register_read(dev, 0x00);
    inv_chip_id = pmw3901_register_read(dev, 0x5F);

    /* Use printk so this is visible even when CONFIG_LOG is disabled */
    printk("PMW3901 chip ID: 0x%02X, inverted: 0x%02X\n", chip_id, inv_chip_id);

    /*
     * BOTH, not either. This was `||`, which passes when one of the two
     * registers is right and the other is garbage -- the exact case seen on
     * this bench, chip_id=0x92 with inv_chip_id=0xB6. The pair is a complement
     * check; accepting half of it accepts a sensor whose reads cannot be
     * trusted and reports it as healthy, which for optical flow means a
     * silently wrong velocity rather than an absent one.
     */
    if (chip_id == 0x49 && inv_chip_id == 0xB6) {
        /* Power on reset */
        pmw3901_register_write(dev, 0x3A, 0x5A);
        k_msleep(5);

        /* Read motion registers once */
        pmw3901_register_read(dev, 0x02);
        pmw3901_register_read(dev, 0x03);
        pmw3901_register_read(dev, 0x04);
        pmw3901_register_read(dev, 0x05);
        pmw3901_register_read(dev, 0x06);
        k_msleep(1);

        /* Initialize registers */
        init_registers(dev);

        /* Verify chip ID again */
        chip_id = pmw3901_register_read(dev, 0x00);
        inv_chip_id = pmw3901_register_read(dev, 0x5F);
        LOG_INF("PMW3901 chip ID after init: 0x%02X, inverted: 0x%02X", chip_id, inv_chip_id);

        data->last_read_time_us = k_uptime_get() * 1000;
        return 0;
    }

    printk("PMW3901: invalid chip ID (check pinmux MISO/MOSI/SCK, CS wiring, SPI mode)\n");
    return -ENODEV;
}

int pmw3901_read_motion_burst(const struct device *dev, motionBurst_t *motion)
{
    const struct pmw3901_config *config = dev->config;
    uint8_t address = 0x16;
    uint8_t rx_buffer[sizeof(motionBurst_t)] = {0};

    struct spi_buf tx_buf = {
        .buf = &address,
        .len = 1,
    };
    struct spi_buf_set tx_bufs = {
        .buffers = &tx_buf,
        .count = 1,
    };

    struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = sizeof(motionBurst_t),
    };
    struct spi_buf_set rx_bufs = {
        .buffers = &rx_buf,
        .count = 1,
    };

    cs_assert(dev);
    busy_wait_us(50);

    /* Send burst-read address */
    spi_write_dt(&config->spi, &tx_bufs);
    busy_wait_us(100);   /* t_SRAD: wait for motion-burst data ready (datasheet ~35 us) before read.
                          * Was k_msleep(1) x3 (~6 ms/read, 167 Hz cap) -- busy_wait_us gives ~4 kHz
                          * and a precise, jitter-free dt for the flow velocity integration. */

    /* Read motion burst data */
    spi_read_dt(&config->spi, &rx_bufs);

    cs_deassert(dev);
    busy_wait_us(50);

    memcpy(motion, rx_buffer, sizeof(motionBurst_t));

    /* Fix endianness for shutter value */
    uint16_t real_shutter = (motion->shutter >> 8) & 0x0FF;
    real_shutter |= (motion->shutter & 0x0FF) << 8;
    motion->shutter = real_shutter;

    return 0;
}

/* Register read with datasheet timing only (tSRAD 35 us, tSRR 20 us) instead of the ~1.6 ms of the
 * general-purpose read above: frame capture is ~2500 reads, 4 s at the slow rate vs ~0.3 s here. */
static uint8_t reg_read_fast(const struct device *dev, uint8_t reg)
{
    const struct pmw3901_config *config = dev->config;
    uint8_t tx = reg & ~0x80u, rx = 0;
    struct spi_buf tb = { .buf = &tx, .len = 1 };
    struct spi_buf_set tbs = { .buffers = &tb, .count = 1 };
    struct spi_buf rb = { .buf = &rx, .len = 1 };
    struct spi_buf_set rbs = { .buffers = &rb, .count = 1 };

    cs_assert(dev);
    busy_wait_us(2);
    spi_write_dt(&config->spi, &tbs);
    busy_wait_us(40);
    spi_read_dt(&config->spi, &rbs);
    busy_wait_us(2);
    cs_deassert(dev);
    busy_wait_us(25);
    return rx;
}

/* Raw 35x35 frame grab (PixArt "frame capture" / Bitcraze enableFrameBuffer + readFrameBuffer).
 * Each pixel arrives in two reads of RAWDATA_GRAB (0x58): status 01 carries bits 7:2 in 5:0, status 10
 * carries bits 1:0 in 3:2. Motion tracking is suspended; the caller must re-run pmw3901_init()
 * afterwards. Returns 0, or -ETIMEDOUT (out is then partially filled). */
/* slow = use the general-purpose register read (50 us / 500 us / 50 us + 1 ms gap, more conservative
 * than every reference driver) instead of the datasheet-minimum one -- a readout-timing A/B test. */
int pmw3901_frame_capture(const struct device *dev, uint8_t out[PMW3901_FRAME_PIXELS], bool slow)
{
    uint8_t (*rd)(const struct device *, uint8_t) = slow ? pmw3901_register_read : reg_read_fast;
    static const uint8_t seq[][2] = {
        {0x7F, 0x07}, {0x41, 0x1D}, {0x4C, 0x00}, {0x7F, 0x08}, {0x6A, 0x38},
        {0x7F, 0x00}, {0x55, 0x04}, {0x40, 0x80}, {0x4D, 0x11},
    };
    for (size_t i = 0; i < ARRAY_SIZE(seq); i++) {
        pmw3901_register_write(dev, seq[i][0], seq[i][1]);
    }
    k_msleep(10);
    pmw3901_register_write(dev, 0x70, 0x00);
    pmw3901_register_write(dev, 0x58, 0xFF);

    /* wait for the grab to arm: RAWDATA_GRAB_STATUS (0x59) bits 7:6 */
    int64_t t_end = k_uptime_get() + 500;
    while ((rd(dev, 0x59) & 0xC0) == 0) {
        if (k_uptime_get() > t_end) {
            return -ETIMEDOUT;
        }
    }
    pmw3901_register_write(dev, 0x58, 0x00);

    t_end = k_uptime_get() + (slow ? 60000 : 20000);   /* pixels trickle out at ~7 ms each on this bus */
    int x = 0;
    uint8_t hi = 0;
    bool have_hi = false;
    while (x < PMW3901_FRAME_PIXELS) {
        const uint8_t d = rd(dev, 0x58);
        const uint8_t st = d >> 6;
        if (st == 0x01) {
            hi = (uint8_t)((d & 0x3F) << 2);
            have_hi = true;
        } else if (st == 0x02 && have_hi) {
            out[x++] = hi | (uint8_t)((d & 0x0C) >> 2);
            have_hi = false;
        }
        if (k_uptime_get() > t_end) {
            return -ETIMEDOUT;
        }
    }
    return 0;
}

int pmw3901_read_motion_count(const struct device *dev, int16_t *deltaX, int16_t *deltaY)
{
    pmw3901_register_read(dev, 0x02);
    *deltaY = -1 * (((int16_t)pmw3901_register_read(dev, 0x04) << 8) | pmw3901_register_read(dev, 0x03));
    *deltaX = -1 * (((int16_t)pmw3901_register_read(dev, 0x06) << 8) | pmw3901_register_read(dev, 0x05));
    return 0;
}
