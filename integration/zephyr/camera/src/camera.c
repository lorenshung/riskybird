/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * See include/riskybird/camera.h. Extracted from workloads/camera_photo, whose
 * comments explain why each step is where it is; the reasoning is kept here
 * because it is the part that is expensive to rediscover.
 */

#include <riskybird/camera.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>

#include "hm01b0.h"

/* Capture peripheral. Base and layout: ospi/OspiChipyard.scala. */
#define CAM_BASE        0x10080000UL
#define CAM_CTRL        (CAM_BASE + 0x00)
#define CAM_GEOM        (CAM_BASE + 0x04)
#define CAM_MCLKDIV     (CAM_BASE + 0x08)
#define CAM_LASTWIDTH   (CAM_BASE + 0x14)
#define CAM_LASTHEIGHT  (CAM_BASE + 0x18)
#define CAM_FLAGS       (CAM_BASE + 0x1c)
#define CAM_DATA        (CAM_BASE + 0x20)
#define CAM_CAPACITY    (CAM_BASE + 0x24)
#define CAM_PIXTARGET   (CAM_BASE + 0x28)
#define CAM_CAPCOUNT    (CAM_BASE + 0x2c)
#define CAM_PCLKCNT     (CAM_BASE + 0x30)
#define CAM_FVLDCNT     (CAM_BASE + 0x34)
#define CAM_LVLDCNT     (CAM_BASE + 0x38)
#define CAM_CAPSTAT     (CAM_BASE + 0x40)

#define CTRL_ENABLE     (1U << 0)
#define CTRL_CLEAR      (1U << 4)
#define CTRL_FLUSH      (1U << 5)
#define CTRL_ARM        (1U << 6)

#define FLAG_OVERFLOW   (1U << 1)
#define DATA_VALID      (1U << 31)
#define DATA_EOF        (1U << 10)
#define DATA_PIXEL_MASK 0xffU
#define CAPSTAT_DONE    (1U << 1)

/*
 * MCLK = sysclk / (2 * (div + 1)), sysclk = 50 MHz, so div = 1 is 12.5 MHz.
 * 12.5 MHz because that is the rate this sensor has actually answered at on
 * this hardware; 6.25 MHz, equally inside the datasheet's 3-36 MHz range, got
 * no ACK at all.
 */
#define CAM_MCLK_DIV       1U
#define CAPTURE_TIMEOUT_MS 4000

static const struct device *i2c_dev;

/*
 * SCCB needs a STOP between the register address and the read. i2c_write_read()
 * emits a repeated START and a healthy HM01B0 NAKs it, which is
 * indistinguishable from an absent sensor.
 */
static int cam_reg_read(uint16_t reg, uint8_t *out)
{
	uint8_t addr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff) };
	int rc = i2c_write(i2c_dev, addr, sizeof(addr), HM01B0_I2C_ADDR);

	if (rc != 0) {
		return rc;
	}
	return i2c_read(i2c_dev, out, 1, HM01B0_I2C_ADDR);
}

static int cam_reg_write(uint16_t reg, uint8_t val)
{
	uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff), val };

	return i2c_write(i2c_dev, buf, sizeof(buf), HM01B0_I2C_ADDR);
}

uint32_t rb_camera_capacity(void)
{
	return sys_read32(CAM_CAPACITY);
}

static int stop(const char **stage, const char **detail,
		const char *s, const char *d)
{
	*stage = s;
	*detail = d;
	return -1;
}

int rb_camera_capture_frame(uint8_t *dst, uint32_t want,
			    struct rb_camera_capture *out,
			    const char **stage, const char **detail)
{
	uint8_t id_h, id_l;
	uint32_t captured, n = 0, word;
	int64_t deadline;

	*stage = "none";
	*detail = "";

	if (sys_read32(CAM_CAPACITY) < want + 1U) {
		return stop(stage, detail, "capacity",
			    "capture buffer is smaller than one frame");
	}

	/* MCLK first: the sensor cannot answer on SCCB without its master clock. */
	sys_write32(CAM_MCLK_DIV, CAM_MCLKDIV);
	sys_write32(CTRL_ENABLE, CAM_CTRL);
	k_msleep(10);

	i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(i2c_dev)) {
		return stop(stage, detail, "i2c-dev", "I2C controller not ready");
	}
	if (cam_reg_read(HM01B0_REG_MODEL_ID_H, &id_h) != 0 ||
	    cam_reg_read(HM01B0_REG_MODEL_ID_L, &id_l) != 0) {
		return stop(stage, detail, "i2c-nack",
			    "no ACK from the sensor at 0x24 with MCLK running");
	}
	out->model_id = (uint16_t)((id_h << 8) | id_l);
	if (out->model_id != HM01B0_MODEL_ID) {
		return stop(stage, detail, "chip-id",
			    "I2C works but this is not an HM01B0");
	}

	/* A photograph, not a test pattern: cheap insurance against a previous
	 * run leaving a walking-1s pattern enabled. */
	if (cam_reg_write(HM01B0_REG_TEST_PATTERN, HM01B0_TESTPAT_OFF) != 0) {
		return stop(stage, detail, "reg-write",
			    "could not disable TEST_PATTERN_MODE");
	}
	if (cam_reg_write(HM01B0_REG_MODE_SELECT, HM01B0_MODE_STREAMING) != 0) {
		return stop(stage, detail, "stream",
			    "MODE_SELECT write was not acknowledged");
	}
	k_msleep(100);

	/* PCLK is only meaningful after the sensor leaves standby -- the part
	 * powers up in standby and a healthy camera drives no pixel clock until
	 * now, so checking earlier fails a working sensor. */
	out->pclk_before = sys_read32(CAM_PCLKCNT);
	k_msleep(50);
	out->pclk_after = sys_read32(CAM_PCLKCNT);
	if (out->pclk_after == out->pclk_before) {
		return stop(stage, detail, "pclk",
			    "no pixel clock after MODE_SELECT=streaming");
	}
	if (sys_read32(CAM_FVLDCNT) == 0U || sys_read32(CAM_LVLDCNT) == 0U) {
		return stop(stage, detail, "sync",
			    "sensor is clocked but FVLD/LVLD never asserted");
	}

	/* Arm on a frame boundary so the capture is one picture, not a slice of two. */
	sys_write32(CTRL_ENABLE | CTRL_CLEAR, CAM_CTRL);
	sys_write32(CTRL_ENABLE | CTRL_FLUSH, CAM_CTRL);
	sys_write32(want, CAM_PIXTARGET);
	sys_write32(CTRL_ENABLE | CTRL_ARM, CAM_CTRL);

	deadline = k_uptime_get() + CAPTURE_TIMEOUT_MS;
	do {
		captured = sys_read32(CAM_CAPCOUNT);
	} while (captured < want &&
		 (sys_read32(CAM_CAPSTAT) & CAPSTAT_DONE) == 0U &&
		 k_uptime_get() < deadline);
	out->captured = captured;

	/*
	 * Stop the sensor the moment the frame is satisfied. The CDC FIFO is
	 * 1024 beats and the sensor streams at several MHz, so it refills faster
	 * than a polling CPU drains it; leaving it running means OVERFLOW is set
	 * by surplus pixels arriving after PIXTARGET. That is data nobody asked
	 * for, not data that was lost -- which is why overflow is only fatal
	 * below when the count also came up short.
	 */
	(void)cam_reg_write(HM01B0_REG_MODE_SELECT, HM01B0_MODE_STANDBY);
	sys_write32(0U, CAM_CTRL);
	out->flags = sys_read32(CAM_FLAGS);

	if (captured < want) {
		return stop(stage, detail,
			    (out->flags & FLAG_OVERFLOW) ? "overflow" : "timeout",
			    "the frame did not complete");
	}

	while (n < want) {
		word = sys_read32(CAM_DATA);
		if ((word & DATA_VALID) == 0U) {
			break;
		}
		if (word & DATA_EOF) {
			continue;   /* in-band end-of-frame marker, not a pixel */
		}
		dst[n++] = (uint8_t)(word & DATA_PIXEL_MASK);
	}
	out->drained = n;
	out->measured_width = sys_read32(CAM_LASTWIDTH);
	out->measured_height = sys_read32(CAM_LASTHEIGHT);
	out->configured_width = sys_read32(CAM_GEOM) & 0xffffU;
	out->configured_height = (sys_read32(CAM_GEOM) >> 16) & 0xffffU;

	out->vmin = 0xff;
	out->vmax = 0x00;
	for (uint32_t i = 0; i < n; i++) {
		if (dst[i] < out->vmin) { out->vmin = dst[i]; }
		if (dst[i] > out->vmax) { out->vmax = dst[i]; }
	}
	if (n == 0U) {
		return stop(stage, detail, "drain",
			    "capture reported done but the buffer returned no pixels");
	}
	if (out->vmin == out->vmax) {
		return stop(stage, detail, "constant-data",
			    "every pixel identical: the data bus is stuck");
	}
	return 0;
}
