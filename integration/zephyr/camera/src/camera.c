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
#include <zephyr/sys/util.h>

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

/*
 * OPTIONAL, OFF BY DEFAULT: program the sensor for full-resolution, unbinned,
 * unsubsampled readout instead of trusting its power-up defaults.
 *
 *   ./rb build ... with RB_EXTRA_CPPFLAGS=-DRB_CAMERA_FULL_READOUT=1
 *
 * It is a deliberate experiment and not the default because of what is already
 * known. Nothing in this repository has ever written a resolution, window,
 * binning or subsample register -- the whole init is TEST_PATTERN_MODE and
 * MODE_SELECT -- and the pictures that came out were sharp and correctly
 * exposed. A build that starts writing mode registers whose encodings are NOT
 * established on this machine (see the provenance note in hm01b0.h: the only
 * Himax datasheet here is a module datasheet that names exactly one of them)
 * can turn a working camera into a broken one, and the failure would look like
 * a hardware fault. Keeping it behind a flag means the bad outcome is always
 * one rebuild away from being undone.
 *
 * What it is FOR: the sensor demonstrably is not in the mode the software
 * assumes -- GEOM is configured 324x244 and the core measures 324 active
 * columns by roughly 324 lines -- so this pins the mode down instead of
 * inferring it. It should be a no-op on a sensor that is already in full
 * readout, and the read-back below is what proves that either way.
 *
 * GRP_PARAM_HOLD brackets the writes so the sensor cannot latch a half-written
 * configuration mid-frame. 0x0104 = 1 to hold, 0 to commit. UNVERIFIED: the
 * address and that protocol are datasheet recall, not established here. If the
 * hold write is NAKed the sequence gives up rather than writing the mode
 * registers unbracketed, because a torn configuration is harder to diagnose
 * than a refusal.
 *
 * Order is deliberate: QVGA_WIN_EN last, because it is the only one of the four
 * whose address and polarity this machine can actually confirm, so if the
 * sequence dies partway the register that is most likely to have taken effect
 * is the one whose effect is understood.
 */
#ifndef RB_CAMERA_FULL_READOUT
#define RB_CAMERA_FULL_READOUT 0
#endif

/*
 * Raise the auto-exposure target.
 *
 * MEASURED 2026-09-22, reading the live part for the first time: AE is ENABLED
 * (ae_ctrl 0x2100 = 0x01) and working correctly -- it is simply converging to
 * the target it was given, AE_TARGET 0x2101 = 0x3c = 60 DN. With black level
 * correction on and BLC_TARGET = 32, a mean of 60 leaves about 28 DN of actual
 * signal, and every frame ever captured here spans 5-49 DN above black. The
 * sensor was never underexposed by accident; nobody had ever set the target.
 *
 * There is plenty of headroom to spend: integration was 67 lines out of a
 * 562-line frame (12%), and analog gain was at its 1x minimum, so AE can reach
 * a higher target on integration time alone without adding gain noise.
 *
 * Why this is the prerequisite for the colour question rather than a cosmetic
 * improvement: a per-channel difference on a Bayer sensor is a RATIO of
 * signals above black. At 28 DN that difference lives in a handful of codes and
 * sits at the quantisation floor; at ~96 DN it is four times larger while the
 * quantisation step is unchanged. Every colour statistic computed on this
 * bench so far was measured on almost no signal.
 *
 * Default 128 is mid-scale for 8 bits, leaving ~96 DN above black and headroom
 * before clipping. Override with RB_CAMERA_AE_TARGET.
 */
#ifndef RB_CAMERA_AE_SETTLE_MS
#define RB_CAMERA_AE_SETTLE_MS 600
#endif

#ifndef RB_CAMERA_AE_TARGET
#define RB_CAMERA_AE_TARGET 128
#endif

static int cam_set_ae_target(uint8_t target)
{
	int rc = cam_reg_write(HM01B0_REG_GRP_PARAM_HOLD, HM01B0_GRP_HOLD);

	if (rc != 0) {
		return rc;
	}
	rc = cam_reg_write(HM01B0_REG_AE_TARGET, target);
	/* Commit regardless: a sensor left held may never stream again, and
	 * that presents as a dead camera rather than as a failed write. */
	(void)cam_reg_write(HM01B0_REG_GRP_PARAM_HOLD, HM01B0_GRP_COMMIT);
	return rc;
}

#if RB_CAMERA_FULL_READOUT
static int cam_program_full_readout(void)
{
	static const struct { uint16_t reg; uint8_t val; } seq[] = {
		{ HM01B0_REG_X_ODD_INC,    HM01B0_X_ODD_INC_EVERY },
		{ HM01B0_REG_Y_ODD_INC,    HM01B0_Y_ODD_INC_EVERY },
		{ HM01B0_REG_BINNING_MODE, HM01B0_BINNING_OFF },
		{ HM01B0_REG_QVGA_WIN_EN,  HM01B0_QVGA_WIN_OFF },
	};
	int rc = cam_reg_write(HM01B0_REG_GRP_PARAM_HOLD, HM01B0_GRP_HOLD);

	if (rc != 0) {
		return rc;
	}
	for (int i = 0; i < (int)ARRAY_SIZE(seq); i++) {
		rc = cam_reg_write(seq[i].reg, seq[i].val);
		if (rc != 0) {
			/* Commit anyway: leaving the sensor held is worse than
			 * committing a partial write, because a held sensor may
			 * never stream again and that reads as a dead camera. */
			(void)cam_reg_write(HM01B0_REG_GRP_PARAM_HOLD,
					    HM01B0_GRP_COMMIT);
			return rc;
		}
	}
	return cam_reg_write(HM01B0_REG_GRP_PARAM_HOLD, HM01B0_GRP_COMMIT);
}
#endif

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

	/*
	 * Mark the diagnostics "not reached" FIRST, before any early return can
	 * fire. A zeroed struct would otherwise report readout_ok = 0, which
	 * means "all four registers read cleanly", on a capture that never got
	 * as far as opening the I2C bus -- a lie that reads as evidence.
	 */
	out->readout_ok = RB_CAMERA_READOUT_NOT_READ;
	out->x_odd_inc = out->y_odd_inc = 0xffU;
	out->binning_mode = out->qvga_win_en = 0xffU;
	out->program_ok = RB_CAMERA_PROGRAM_OFF;

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

	/*
	 * Record the readout mode before streaming starts. Read-only in the
	 * default build -- nothing here programs these, so the sensor runs on
	 * its power-up defaults and this is the only way to find out what they
	 * are. With RB_CAMERA_FULL_READOUT the write above has just happened and
	 * this becomes the read-back that says whether it took.
	 *
	 * The reason it is worth four I2C reads: the mono and Bayer-colour
	 * HM01B0 variants share model id 0x01B0, so a colour part that powers up
	 * subsampling or binning averages adjacent CFA sites and yields a frame
	 * with no recoverable mosaic -- byte-for-byte the shape of mono data.
	 * Without these values, "the picture is not colour" and "the sensor is
	 * throwing the colour away before it leaves the die" are indistinguishable.
	 *
	 * A failed read is reported, never fatal: these are diagnostics, and a
	 * sensor that will not answer them can still take a perfectly good
	 * picture.
	 */
#if RB_CAMERA_FULL_READOUT
	/*
	 * Before the probe, so the values reported below are what the sensor is
	 * ACTUALLY in after being told, not what it was told. A write that is
	 * acknowledged and then ignored is a real failure mode for an
	 * unverified register address, and only the read-back catches it.
	 */
	out->program_ok = cam_program_full_readout();
#endif

	/*
	 * EXPOSURE CHAIN DUMP. Read-only, and printed rather than returned,
	 * because this is a bring-up question rather than something a caller
	 * acts on: nothing in this tree has ever written or read any of these,
	 * so the sensor has run on power-up defaults throughout. Every frame
	 * captured on this bench spans 5-49 DN above a ~36 DN black floor,
	 * where a correctly exposed 8-bit frame spans 90-150, and at 20 DN of
	 * signal a colour difference is below the quantisation noise. So this
	 * is the first thing to look at before any further colour analysis.
	 *
	 * A NAK or an unchanged value means the ADDRESS is wrong -- these come
	 * from the published Himax reference init, not from a register map in
	 * this repository. Report, never act.
	 */
	{
		static const struct { uint16_t reg; const char *name; } dump[] = {
			{ HM01B0_REG_AE_CTRL,        "ae_ctrl      " },
			{ HM01B0_REG_AE_TARGET,      "ae_target    " },
			{ HM01B0_REG_AE_MIN_MEAN,    "ae_min_mean  " },
			{ HM01B0_REG_INTEGRATION_H,  "integration_h" },
			{ HM01B0_REG_INTEGRATION_L,  "integration_l" },
			{ HM01B0_REG_ANALOG_GAIN,    "analog_gain  " },
			{ HM01B0_REG_DIGITAL_GAIN_H, "dig_gain_h   " },
			{ HM01B0_REG_DIGITAL_GAIN_L, "dig_gain_l   " },
			{ HM01B0_REG_FRAME_LEN_H,    "frame_len_h  " },
			{ HM01B0_REG_FRAME_LEN_L,    "frame_len_l  " },
			{ HM01B0_REG_LINE_LEN_H,     "line_len_h   " },
			{ HM01B0_REG_LINE_LEN_L,     "line_len_l   " },
			{ HM01B0_REG_BLC_CFG,        "blc_cfg      " },
			{ HM01B0_REG_BLC_TARGET,     "blc_target   " },
		};

		uint8_t ag = 0, dgh = 0, dgl = 0;

		printk("         --- HM01B0 exposure chain ---\n");
		for (unsigned i = 0; i < ARRAY_SIZE(dump); i++) {
			uint8_t v = 0;
			int rc = cam_reg_read(dump[i].reg, &v);

			if (rc == 0) {
				printk("         %s 0x%04x = 0x%02x (%u)\n",
				       dump[i].name, dump[i].reg, v, v);
				if (dump[i].reg == HM01B0_REG_ANALOG_GAIN)    { ag = v; }
				if (dump[i].reg == HM01B0_REG_DIGITAL_GAIN_H) { dgh = v; }
				if (dump[i].reg == HM01B0_REG_DIGITAL_GAIN_L) { dgl = v; }
			} else {
				printk("         %s 0x%04x = NAK (%d) -- wrong address?\n",
				       dump[i].name, dump[i].reg, rc);
			}
		}

		/*
		 * AE SATURATION IS A LIGHTING REPORT, NOT A FAULT, and saying so
		 * here is the whole point of dumping these.
		 *
		 * With the gain chain pegged the sensor emits amplified black,
		 * which arrives as a nearly constant frame with a handful of
		 * distinct values. That looks exactly like a dead sensor, a
		 * broken capture path or a "synthetic pattern", and it has
		 * already been misread as all three on this bench. It is none of
		 * them: it means no photons.
		 *
		 * Measured on this hardware for contrast -- lens with a view:
		 * analog_gain 0x00, integration 67 of 562 lines. Lens dark:
		 * analog_gain 0x30 (max), digital gain 3x, and the frame comes
		 * back as 99.99% one value.
		 *
		 * No colour question can be answered in this state: a per-channel
		 * ratio needs signal above black, and there is none.
		 */
		/*
		 * Analog gain pegged is sufficient on its own. Do NOT also
		 * require the digital gain to read > 1: DIGITAL_GAIN_H is the
		 * INTEGER part and the fraction lives in DIGITAL_GAIN_L, so a
		 * genuine 1.92x reads as h=0x01, l=0xec. Testing h > 1 silently
		 * missed a fully saturated sensor on this bench.
		 */
		if (ag >= HM01B0_ANALOG_GAIN_MAX) {
			printk("         *** AE SATURATED -- analog gain at max (0x%02x), "
			       "digital gain %u+%u/256. The sensor has NO LIGHT.\n",
			       ag, dgh, dgl);
			printk("         *** This is a lighting problem, not a sensor, capture "
			       "or colour problem. Point the camera at a lit scene.\n");
		}
	}

	out->readout_ok = 0;
	{
		static const uint16_t regs[4] = {
			HM01B0_REG_X_ODD_INC, HM01B0_REG_Y_ODD_INC,
			HM01B0_REG_BINNING_MODE, HM01B0_REG_QVGA_WIN_EN,
		};
		uint8_t *dst[4] = {
			&out->x_odd_inc, &out->y_odd_inc,
			&out->binning_mode, &out->qvga_win_en,
		};

		for (int i = 0; i < 4; i++) {
			int rc = cam_reg_read(regs[i], dst[i]);

			if (rc != 0) {
				out->readout_ok = rc;
				*dst[i] = 0xffU;
			}
		}
		/* Bit 0 is the whole of the documented field -- hm01b0.h
		 * quotes the datasheet. The other seven bits are undocumented
		 * here and reporting them invites reading a reserved bit as a
		 * mode change. */
		if (out->readout_ok == 0) {
			out->qvga_win_en &= HM01B0_QVGA_WIN_EN_MASK;
		}
	}

	/*
	 * A photograph, not a test pattern: cheap insurance against a previous
	 * run leaving a walking-1s pattern enabled.
	 *
	 * RB_CAMERA_TEST_PATTERN=1 turns it back on deliberately. The pattern is
	 * generated after the analog column path, so it separates a defect in
	 * the capture datapath from one in the sensor's analog readout -- an
	 * artifact present in a photograph and absent in the pattern is
	 * upstream of the bus, and cannot be a capture-core or wiring fault.
	 *
	 * RESULT, 2026-09-22: it is absent. The pattern arrives bit-exact --
	 * three distinct byte values in the whole frame, all three present on
	 * all four 2x2 phases, and every full-scale adjacent-column step lands
	 * on its exact code. So the one-phase attenuation in the photographs is
	 * UPSTREAM of the bus and no capture-core change will remove it.
	 *
	 * Do NOT read the per-2x2-phase means of a pattern frame: the pattern is
	 * a one-pixel-period column stripe, so those means measure its geometry.
	 * hm01b0.h carries the measurement.
	 */
#ifndef RB_CAMERA_TEST_PATTERN
#define RB_CAMERA_TEST_PATTERN 0
#endif
	{
		int aerc = cam_set_ae_target(RB_CAMERA_AE_TARGET);

		if (aerc == 0) {
			printk("         AE target raised to %u DN (was 60 at power-up; "
			       "black level is 32, so this is ~%u DN of signal)\n",
			       (unsigned)RB_CAMERA_AE_TARGET,
			       (unsigned)(RB_CAMERA_AE_TARGET > 32
					  ? RB_CAMERA_AE_TARGET - 32 : 0));
		} else {
			printk("         AE target write FAILED (%d) -- exposure "
			       "unchanged, frames will stay dark\n", aerc);
		}
	}

	if (cam_reg_write(HM01B0_REG_TEST_PATTERN,
			  RB_CAMERA_TEST_PATTERN ? HM01B0_TESTPAT_WALKING1
						 : HM01B0_TESTPAT_OFF) != 0) {
		return stop(stage, detail, "reg-write",
			    "could not set TEST_PATTERN_MODE");
	}
	if (cam_reg_write(HM01B0_REG_MODE_SELECT, HM01B0_MODE_STREAMING) != 0) {
		return stop(stage, detail, "stream",
			    "MODE_SELECT write was not acknowledged");
	}
	/*
	 * AE settle. 100 ms was enough when the target was never changed --
	 * the sensor simply streamed at whatever it powered up with. Having
	 * just moved the target, the loop has to re-converge, and it does so
	 * over frames rather than instantly: at ~30 fps, 600 ms is roughly 18
	 * frames, which is ample for an AE loop that only has to walk
	 * integration time up. Capturing before it converges yields a frame
	 * part-way through the ramp, which looks exactly like a target that
	 * did not take.
	 */
	k_msleep(RB_CAMERA_AE_SETTLE_MS);

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
