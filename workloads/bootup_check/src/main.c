/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Whole-board bootup sequence: every peripheral a flight needs, brought up in
 * the order it would be brought up for a flight, each stage reporting one
 * verdict, ending in a single summary table.
 *
 * This is a liveness check, not a characterization. Depth belongs to the
 * per-peripheral workloads -- flight_estimator for sensor values, ddr_stress
 * for memory, vl35l5cx_test --mode ranging for the side ToFs, camera_photo for
 * a frame, motor_duty for one motor at a time. What this one adds is that
 * they are all exercised in one pass, in boot order, so a board is either
 * ready or it names what is not.
 *
 * Order matters and is not arbitrary:
 *
 *   1. memory     -- nothing below is trustworthy if DDR3 is not.
 *   2. inventory  -- who answers on the bus, before anything is configured.
 *   3. expander   -- the ADS7128 gates the ToF array; it comes up first.
 *   4. tof        -- XSHUT bring-up. Every ToF on this board is behind the
 *                    expander, so this cannot be done without stage 3.
 *   5. imu        -- the estimator's only attitude source.
 *   6. camera     -- shares the I2C bus with everything above.
 *   7. flow       -- SPI, so independent of the bus; last of the sensors.
 *   8. motors     -- only under --mode motors: duty frames to the ESP32,
 *                    which owns the gates. This core drives no motor pin.
 */

#include <riskybird/camera.h>
#include <riskybird/motor_guard.h>
#include <riskybird/tof.h>
#include "pmw3901.h"

#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#define PMW3901_NODE DT_NODELABEL(riskybird_pmw3901)

/* Set by `rb build bootup_check --mode motors` (cppflags_by_mode in
 * tools/rb/boards.py). The sweep goes through the ESP link; see stage 8. */
#ifndef BOOTUP_MOTORS
#define BOOTUP_MOTORS 0
#endif

/* ---------------------------------------------------------------- addresses */

#define ADDR_ADS7128   0x17U   /* 8-channel ADC / GPIO expander; gates the ToFs */
#define ADDR_ACCEL     0x18U   /* BMI088 accelerometer */
#define ADDR_TOF       0x29U   /* VL53Lxx default; freed by the XSHUT bring-up */
#define ADDR_CAMERA    0x24U   /* HM01B0 */
#define ADDR_GYRO      0x68U   /* BMI088 gyroscope */
#define ADDR_BARO      0x76U   /* BMP280 / BME280 class barometer */

#define ADS7128_CMD_WRITE     0x08U
#define ADS7128_CMD_READ      0x10U
#define ADS7128_SYSTEM_STATUS 0x00U

/*
 * Skip the first 32 MiB, as ddr_stress does: image, stacks and heap live down
 * there. 4 MiB is enough to prove the controller is real and keeps the stage
 * under a second; ddr_stress is the one that walks the part.
 */
#define MEM_BASE  0x82000000UL
#define MEM_WORDS ((4UL * 1024UL * 1024UL) / sizeof(uint64_t))

/* ------------------------------------------------------------------ verdicts */

enum verdict { OK = 0, WARN, BAD, SKIP };

static const char *const VERDICT[] = {
	[OK] = "  OK  ", [WARN] = " WARN ", [BAD] = " FAIL ", [SKIP] = " SKIP ",
};

enum stage {
	S_MEMORY = 0, S_INVENTORY, S_EXPANDER, S_TOF, S_IMU, S_CAMERA, S_FLOW,
	S_MOTORS, S_COUNT,
};

static const char *const STAGE_NAME[S_COUNT] = {
	[S_MEMORY] = "main memory (DDR3)",
	[S_INVENTORY] = "I2C inventory",
	[S_EXPANDER] = "ADS7128 expander",
	[S_TOF] = "ToF array (XSHUT)",
	[S_IMU] = "BMI088 IMU",
	[S_CAMERA] = "HM01B0 camera",
	[S_FLOW] = "PMW3901 flow",
	[S_MOTORS] = "motors (ESP link)",
};

static enum verdict outcome[S_COUNT];
static char detail[S_COUNT][96];

static void finish(enum stage s, enum verdict v, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(detail[s], sizeof(detail[s]), fmt, ap);
	va_end(ap);
	outcome[s] = v;
	printf("[%s] %-20s %s\n", VERDICT[v], STAGE_NAME[s], detail[s]);
}

/* --------------------------------------------------------------------- I2C */

static const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));

static bool acks(uint8_t addr)
{
	uint8_t byte;

	/* A zero-length write is not enough on this controller; a one-byte read
	 * is what i2c_scanner and sensor_check both use to decide presence. */
	return i2c_read(i2c, &byte, sizeof(byte), addr) == 0;
}

static int reg16_read(uint8_t addr, uint16_t reg, uint8_t *out)
{
	uint8_t tx[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };

	return i2c_write_read(i2c, addr, tx, sizeof(tx), out, 1);
}

static int ads7128_read(uint8_t reg, uint8_t *out)
{
	uint8_t tx[2] = { ADS7128_CMD_READ, reg };

	return i2c_write_read(i2c, ADDR_ADS7128, tx, sizeof(tx), out, 1);
}

/* --------------------------------------------------------- 1. main memory */

static void stage_memory(void)
{
	volatile uint64_t *const mem = (volatile uint64_t *)MEM_BASE;

	/* Address-in-address: catches a stuck or shorted address line, which a
	 * constant pattern cannot -- every cell would still read back its own
	 * (wrong) neighbour's value and compare equal. */
	for (size_t i = 0; i < MEM_WORDS; i++) {
		mem[i] = (uint64_t)(uintptr_t)&mem[i];
	}
	for (size_t i = 0; i < MEM_WORDS; i++) {
		if (mem[i] != (uint64_t)(uintptr_t)&mem[i]) {
			finish(S_MEMORY, BAD,
			       "mismatch at 0x%08x: got 0x%016llx",
			       (unsigned)(uintptr_t)&mem[i],
			       (unsigned long long)mem[i]);
			return;
		}
	}
	finish(S_MEMORY, OK, "%lu MiB at 0x%08lx verified address-in-address",
	       (unsigned long)(MEM_WORDS * sizeof(uint64_t) / (1024UL * 1024UL)),
	       (unsigned long)MEM_BASE);
}

/* ------------------------------------------------------------ 2. inventory */

static void stage_inventory(void)
{
	static const struct { uint8_t addr; const char *what; } expected[] = {
		{ ADDR_ADS7128, "ADS7128 expander" },
		{ ADDR_ACCEL,   "BMI088 accel"     },
		{ ADDR_GYRO,    "BMI088 gyro"      },
		{ ADDR_CAMERA,  "HM01B0 camera"    },
		{ ADDR_BARO,    "barometer"        },
		{ ADDR_TOF,     "VL53Lxx at default" },
	};
	unsigned int found = 0, missing = 0;

	for (size_t i = 0; i < ARRAY_SIZE(expected); i++) {
		bool present = acks(expected[i].addr);

		printf("         0x%02x  %-20s %s\n", expected[i].addr,
		       expected[i].what, present ? "ACK" : "--");
		if (present) {
			found++;
		} else if (expected[i].addr != ADDR_TOF) {
			/* 0x29 is expected to be absent once a previous run has
			 * readdressed the array, so it is not counted missing. */
			missing++;
		}
	}
	if (missing == 0U) {
		finish(S_INVENTORY, OK, "%u answered; every fixed part present", found);
	} else {
		finish(S_INVENTORY, BAD, "%u answered, %u fixed part(s) missing",
		       found, missing);
	}
}

/* ------------------------------------------------------------- 3. expander */

static void stage_expander(void)
{
	uint8_t status;

	if (ads7128_read(ADS7128_SYSTEM_STATUS, &status) != 0) {
		finish(S_EXPANDER, BAD, "SYSTEM_STATUS read failed at 0x%02x",
		       ADDR_ADS7128);
		return;
	}
	/* Reading a register at all distinguishes an ADS7128 from something
	 * merely ACKing 0x17: the opcode framing is device-specific. */
	finish(S_EXPANDER, OK, "SYSTEM_STATUS=0x%02x (register framing accepted)",
	       status);
}

/* ------------------------------------------------------------------ 4. ToF */

static struct rb_tof_array tof;

/*
 * Channels 1, 2, 3 and 6, measured rather than assumed -- see
 * workloads/vl35l5cx_test: rb_tof_scan_xshut found those four and only those,
 * and rb_tof_hold_all confirmed no VL53 part is wired straight to the bus.
 * Channel 6 is the down-facing VL53L1X; 1..3 are the side headers J9, J2, J11.
 * Slots that hold nothing report absent rather than failing, which is the whole
 * point of a per-sensor status: one empty header must not hide the other three.
 *
 * sensors[] is left NULL: this stage proves the parts come up and answer at
 * their new addresses, which needs no driver. Ranging is vl35l5cx_test's job.
 */
static const struct rb_tof_array_config tof_config = {
	.i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0)),
/*
 * BOOTUP_TOF_SCAN=1 claims only the down sensor, which leaves every side
 * channel unclaimed so rb_tof_scan_xshut probes all of them and reports where
 * a sensor actually answers. Use it when the number of side sensors found
 * disagrees with the number fitted: the default config scans channels 1, 2, 3
 * and 6, and the schematic puts four side headers on GPIO1-4 (J9, J2, J11,
 * J10), so a sensor on channel 4 is invisible to the default and looks exactly
 * like an empty header.
 */
#ifndef BOOTUP_TOF_SCAN
#define BOOTUP_TOF_SCAN 0
#endif
#if BOOTUP_TOF_SCAN
	.sensor_count = 1,
	.xshut_channels = { 6 },
	.addresses = { 0x34 },
#else
	.sensor_count = 4,
	.xshut_channels = { 1, 2, 3, 6 },
	.addresses = { 0x31, 0x32, 0x33, 0x34 },
#endif
};

static void stage_tof(void)
{
	unsigned int up = 0, absent = 0;
	uint8_t unclaimed = 0U;

	/* Every ToF starts at 0x29, so they collide until each is moved. Hold
	 * them all down first: a previous run may have left one live. */
	(void)rb_tof_hold_all(i2c);

	if (rb_tof_array_init(&tof, &tof_config) != 0 && tof.initialized_sensors == 0U) {
		finish(S_TOF, BAD, "no sensor came up; stopped at %s (err %d)",
		       rb_tof_step_name(tof.last_step), tof.last_error);
		return;
	}
	for (uint8_t i = 0; i < tof_config.sensor_count; i++) {
		const char *where = (tof_config.xshut_channels[i] == 6U) ? "down" : "side";

		if (tof.status[i] == 0) {
			up++;
			printf("         ch%u %-5s %-10s at 0x%02x (id 0x%04x)\n",
			       tof_config.xshut_channels[i], where,
			       rb_tof_kind_name(tof.kinds[i]),
			       tof_config.addresses[i], tof.identities[i]);
		} else {
			absent++;
			printf("         ch%u %-5s -- absent (err %d)\n",
			       tof_config.xshut_channels[i], where, tof.status[i]);
		}
	}
	/* Anything gated by a channel this config does not claim. On a board
	 * whose population changed, that is the difference between "one sensor
	 * is broken" and "it is on a header nobody declared". */
	int scan = rb_tof_scan_xshut(&tof, &unclaimed);

	if (scan != 0) {
		printf("         unclaimed-channel scan failed (%d)\n", scan);
	} else if (unclaimed != 0U) {
		printf("         unclaimed channels answering at 0x29: mask 0x%02x ->",
		       unclaimed);
		for (unsigned ch = 0; ch < 8; ch++) {
			if (unclaimed & (1U << ch)) { printf(" ch%u", ch); }
		}
		printf("\n");
	} else {
		printf("         no unclaimed channel answers at 0x29\n");
	}

	bool down_up = (tof.status[3] == 0);

	if (up == tof_config.sensor_count) {
		finish(S_TOF, OK, "%u/%u up (down + %u side)", up,
		       tof_config.sensor_count, up - 1U);
	} else if (down_up && up >= 2U) {
		/* The down sensor plus at least one side is the minimum a
		 * height-holding flight needs; a missing side header is a
		 * population question, not a fault. */
		finish(S_TOF, WARN, "%u up, %u absent -- down sensor present",
		       up, absent);
	} else {
		finish(S_TOF, BAD, "%u up, %u absent -- down sensor %s", up, absent,
		       down_up ? "present" : "MISSING");
	}
}

/* ------------------------------------------------------------------ 5. IMU */

static void stage_imu(void)
{
	const struct device *accel = DEVICE_DT_GET(DT_ALIAS(bmi088_accel));
	const struct device *gyro = DEVICE_DT_GET(DT_ALIAS(bmi088_gyro));
	struct sensor_value v[3];
	int64_t mag_milli;

	if (!device_is_ready(accel) || !device_is_ready(gyro)) {
		finish(S_IMU, BAD, "accel or gyro device not ready");
		return;
	}
	if (sensor_sample_fetch(accel) != 0 ||
	    sensor_channel_get(accel, SENSOR_CHAN_ACCEL_XYZ, v) != 0) {
		finish(S_IMU, BAD, "accelerometer sample fetch/get failed");
		return;
	}

	/* Gravity is the one reading whose correct value is known without a
	 * reference instrument, so it is what proves the part is really being
	 * read rather than returning a plausible constant. Milli-units
	 * throughout: CBPRINTF_FP_SUPPORT is off. */
	int64_t sum = 0;

	for (int i = 0; i < 3; i++) {
		int64_t milli = (int64_t)v[i].val1 * 1000 + v[i].val2 / 1000;

		sum += milli * milli;
	}
	mag_milli = 0;
	for (int64_t g = 1; g * g <= sum; g++) {
		mag_milli = g; /* integer sqrt; sum is at most a few 10^8 here */
	}
	printf("         |a| = %lld mm/s^2 (expect 9807 at rest)\n",
	       (long long)mag_milli);

	if (sensor_sample_fetch(gyro) != 0 ||
	    sensor_channel_get(gyro, SENSOR_CHAN_GYRO_XYZ, v) != 0) {
		finish(S_IMU, BAD, "accel reads but gyro sample fetch/get failed");
		return;
	}
	/* 5% of g. Tighter than this fails on a bench that is being leaned on;
	 * looser stops distinguishing a real reading from a stuck register. */
	if (mag_milli < 9317 || mag_milli > 10297) {
		finish(S_IMU, WARN,
		       "|a| = %lld mm/s^2 is not 1 g -- board moving, or axis fault",
		       (long long)mag_milli);
		return;
	}
	finish(S_IMU, OK, "accel |a| = %lld mm/s^2 within 5%% of g; gyro reads",
	       (long long)mag_milli);
}

/* --------------------------------------------------------------- 6. camera */

/*
 * One frame at the width the core MEASURES, not the width GEOM is configured
 * for: the sensor puts two dummy pixels ahead of each active line, so lines are
 * 326 wide where GEOM says 324. Reshaping at 324 shears the picture one pixel
 * per row, which looks like broken hardware and is not.
 */
#ifndef BOOTUP_CAMERA_WIDTH
#define BOOTUP_CAMERA_WIDTH  326U
#endif
/*
 * 324 rows, measured, not the 242 this started with and not the 244 GEOM is
 * configured for. The core's own counters settle it: 1296 line-valid
 * assertions over 4 frames is 324 lines per frame, and LASTHEIGHT reports 324
 * independently. Asking for 242 captured the top 75% of the picture and cut 82
 * rows off the bottom, which reads as a broken sensor and is just a short
 * PIXTARGET.
 */
#ifndef BOOTUP_CAMERA_HEIGHT
#define BOOTUP_CAMERA_HEIGHT 324U
#endif
#define BOOTUP_CAMERA_PIXELS (BOOTUP_CAMERA_WIDTH * BOOTUP_CAMERA_HEIGHT)

/*
 * Pixels left free above PIXTARGET so the surplus arriving during the sensor's
 * SCCB stop has somewhere to go. 2048 is ~6 rows and ~1.6x the ~1250 pixels a
 * 100 us stop admits at 12.5 MHz PCLK.
 */
#ifndef BOOTUP_CAMERA_STOP_MARGIN
#define BOOTUP_CAMERA_STOP_MARGIN 2048U
#endif

/*
 * Debugger handles, deliberately the same names camera_photo and camera_bringup
 * export so that one GDB script serves all three. pixels[] is a file static and
 * survives main returning; camera_frame_pixels is how many leading bytes are
 * valid; camera_frame_ready() is the breakpoint anchor.
 */
static uint8_t pixels[BOOTUP_CAMERA_PIXELS];
volatile uint32_t camera_frame_pixels;

__attribute__((noinline)) void camera_frame_ready(void)
{
	/*
	 * A side effect the optimiser cannot discard. noinline alone is not
	 * enough: with an empty body GCC deduces the function is const and
	 * deletes the call site, so the anchor disappears from the image even
	 * though the symbol still resolves.
	 */
	__asm__ volatile("" ::: "memory");
}

/*
 * The readout mode, printed on BOTH the pass and the fail path.
 *
 * On the fail path especially: a capture that dies at "sync" or "timeout" is
 * exactly when you most want to know what mode the sensor is in, and that is
 * exactly when the old code returned before printing it. The probe runs before
 * streaming starts, so by the time any capture-stage failure fires these four
 * values have already been read and are the most useful thing on the screen.
 *
 * What the flag means: subsampling or binning averages or skips adjacent CFA
 * sites, so a colour part in either mode returns pixels indistinguishable from
 * a mono part. The QVGA window is NOT in that list -- the datasheet is explicit
 * that it is a crop at (0,0), and a crop cannot destroy a mosaic.
 */
static void report_readout(const struct rb_camera_capture *cap)
{
	if (cap->program_ok == RB_CAMERA_PROGRAM_OK) {
		printf("         full-readout config: written and committed "
		       "(RB_CAMERA_FULL_READOUT)\n");
	} else if (cap->program_ok != RB_CAMERA_PROGRAM_OFF) {
		printf("         full-readout config: FAILED (%d) -- the sensor did "
		       "not accept it; the values below are whatever survived\n",
		       cap->program_ok);
	}
	if (cap->readout_ok == RB_CAMERA_READOUT_NOT_READ) {
		printf("         readout: not probed -- the capture stopped before "
		       "the sensor was asked\n");
		return;
	}
	if (cap->readout_ok != 0) {
		printf("         readout: registers unreadable (%d)\n",
		       cap->readout_ok);
		return;
	}
	printf("         readout: x_odd_inc=0x%02x y_odd_inc=0x%02x "
	       "binning=0x%02x qvga_win=%u%s\n",
	       cap->x_odd_inc, cap->y_odd_inc, cap->binning_mode,
	       cap->qvga_win_en,
	       (cap->binning_mode != 0x00U || cap->x_odd_inc != 0x01U ||
		cap->y_odd_inc != 0x01U)
		       ? "  <- MOSAIC-DESTROYING"
		       : "  (full readout; a mosaic would survive)");
}

static void stage_camera(void)
{
	static struct rb_camera_capture cap;
	const char *stage = "none", *why = "";
	uint32_t capacity = rb_camera_capacity();
	uint32_t want = BOOTUP_CAMERA_PIXELS;

	/*
	 * Clamp to whole rows the core can hold, keeping a stop-latency margin.
	 *
	 * On this shell CAPACITY is 104977 pixels against a 105624-pixel frame,
	 * so the full height fails the capacity check outright and captures
	 * nothing. But filling the buffer to the brim fails differently and
	 * worse: the sensor cannot be stopped instantly -- stopping it is an
	 * SCCB write, ~100 us, while pixels keep arriving at 12.5 MHz -- so
	 * roughly a thousand surplus pixels land after PIXTARGET is satisfied.
	 * With 5 pixels of headroom that overflows immediately and the drain
	 * comes back empty: measured, capcount=104972 of 104972 and pixels[]
	 * still zero. The margin is what makes the difference between a
	 * photograph and a successful capture you cannot read.
	 *
	 * Whole rows, always. A partial row shifts every row after it and shears
	 * the picture, which looks like broken silicon and is not.
	 */
	if (want + BOOTUP_CAMERA_STOP_MARGIN > capacity) {
		uint32_t rows = (capacity - BOOTUP_CAMERA_STOP_MARGIN) /
				BOOTUP_CAMERA_WIDTH;

		want = rows * BOOTUP_CAMERA_WIDTH;
		printf("         core holds %u px, frame is %u -- capturing %u of "
		       "%u rows (%u px stop margin)\n", capacity,
		       (unsigned)BOOTUP_CAMERA_PIXELS, rows, BOOTUP_CAMERA_HEIGHT,
		       (unsigned)BOOTUP_CAMERA_STOP_MARGIN);
	}
	if (rb_camera_capture_frame(pixels, want, &cap, &stage, &why) != 0) {
		/* Publish whatever was drained anyway: a partial frame is worth
		 * reading out, and the anchor below is what lets the debugger
		 * do it. */
		camera_frame_pixels = cap.drained;
		/* Before finish(), which prints the stage verdict: the readout
		 * lines belong with the failure they explain, not after it. */
		report_readout(&cap);
		finish(S_CAMERA, BAD, "%s -- %s", stage, why);
		return;
	}
	camera_frame_pixels = cap.drained;
	/*
	 * READ THE WIDTH, NOT THE HEIGHT.
	 *
	 * LASTWIDTH is trustworthy: it is the length of the last complete line
	 * before end-of-frame, and it reads 326 in every capture -- 324 active
	 * pixels behind two 0x00 dummies, confirmed by the dummies themselves
	 * appearing at exactly one offset mod 326 in all 315 lines of four
	 * separate captures.
	 *
	 * LASTHEIGHT is not. The core counts lines only while a capture is
	 * storing, and ARM lands wherever the free-running sensor happens to be,
	 * so the count is taken across a frame boundary from an arbitrary
	 * starting line. The same firmware and the same core have reported 242,
	 * 272 and 296 for a sensor whose line count did not change. Do not infer
	 * a readout mode from it; the sensor's own registers, above, are what
	 * answers that.
	 */
	printf("         model 0x%04x, %ux%u measured (%ux%u configured), "
	       "min=0x%02x max=0x%02x\n",
	       cap.model_id, cap.measured_width, cap.measured_height,
	       cap.configured_width, cap.configured_height, cap.vmin, cap.vmax);
	report_readout(&cap);
	finish(S_CAMERA, OK, "%u-byte frame captured; reshape rows at %u",
	       cap.drained, cap.measured_width);
}

/* ----------------------------------------------------------------- 7. flow */

#if DT_NODE_EXISTS(PMW3901_NODE)

static void stage_flow(void)
{
	/* The driver takes a struct device, and this one is assembled by hand
	 * rather than instantiated: the wiring lives in a riskybird,pmw3901
	 * node with no driver of its own, because the sifive controller's
	 * automatic chip-select does not hold across the driver's multi-byte
	 * register transactions. Same construction as sensor_check. */
	static struct pmw3901_config cfg;
	static struct pmw3901_data data;
	static const struct device dev = {
		.name = "PMW3901", .config = &cfg, .data = &data,
	};
	motionBurst_t burst;

	cfg.spi.bus = DEVICE_DT_GET(DT_PHANDLE(PMW3901_NODE, spi_bus));
	cfg.spi.config.operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER |
				   SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB;
	cfg.spi.config.frequency = 2000000;
	cfg.spi.config.slave = 0;
	cfg.cs_gpio = (struct gpio_dt_spec)GPIO_DT_SPEC_GET(PMW3901_NODE, cs_gpios);
	cfg.reset_gpio = (struct gpio_dt_spec)GPIO_DT_SPEC_GET(PMW3901_NODE, reset_gpios);
	cfg.led_gpio = (struct gpio_dt_spec)GPIO_DT_SPEC_GET(PMW3901_NODE, led_gpios);

	if (!device_is_ready(cfg.spi.bus) || !device_is_ready(cfg.cs_gpio.port)) {
		finish(S_FLOW, BAD, "SPI bus or chip-select GPIO not ready");
		return;
	}
	if (pmw3901_init(&dev) != 0) {
		finish(S_FLOW, BAD, "init failed -- chip id did not read back 0x49");
		return;
	}
	if (pmw3901_read_motion_burst(&dev, &burst) != 0) {
		finish(S_FLOW, BAD, "motion burst read failed after a good init");
		return;
	}
	/*
	 * Read the identity rather than asserting it. This line used to quote a
	 * literal "chip id 0x49" in its own success message, which reported a
	 * value it had never read -- and did so on a run whose driver print
	 * showed 0x92.
	 */
	uint8_t id = pmw3901_register_read(&dev, 0x00);
	uint8_t inv = pmw3901_register_read(&dev, 0x5F);
	/* SQUAL is surface quality. A stuck bus reads a rail value; a real
	 * surface lands well inside the range, which "the register was
	 * readable" does not distinguish. */
	if (burst.squal == 0U || burst.squal == 0xFFU) {
		finish(S_FLOW, WARN,
		       "SQUAL=%u is a rail value -- init passed, surface reads stuck",
		       burst.squal);
		return;
	}
	if (id != 0x49U || inv != 0xB6U) {
		finish(S_FLOW, WARN,
		       "burst reads but id=0x%02x/inv=0x%02x, expected 0x49/0xb6",
		       id, inv);
		return;
	}
	finish(S_FLOW, OK, "id 0x%02x/inv 0x%02x; dx=%+d dy=%+d SQUAL=%u shutter=%u",
	       id, inv, burst.deltaX, burst.deltaY, burst.squal, burst.shutter);
}

#else

static void stage_flow(void)
{
	finish(S_FLOW, SKIP, "no riskybird,pmw3901 wiring node in this build");
}

#endif /* PMW3901_NODE */

/* --------------------------------------------------------------- 8. motors */

#if BOOTUP_MOTORS
/*
 * The motors are driven by the ESP32-C6, not by this core. The ESP (esp_bridge
 * --mode motors) owns all four gates and applies the 14-byte motor_link duty
 * frames this core sends it on uart1 -- the same frames, wire and rate the
 * flight controller uses, so this sweep exercises the flight path, gate FET
 * and all. This image enables no PWM driver and drives no motor pin.
 *
 * Until 2026-11-03 this stage wrote the FPGA's own PWM comparators instead.
 * That path no longer reaches a flying motor: motor 4's pad (F13) is dead, the
 * ESP drives the same four gates with no arbitration, and a halted core left
 * the comparators driving. Here a halted core stops sending frames and the ESP
 * cuts all four within 120 ms.
 *
 * The sweep is a liveness test for the four motors, so it has to be long
 * enough and strong enough to SEE. 300 ms at 10% is inside the stiction of a
 * loaded motor -- it twitches and stops, which is indistinguishable from dead.
 *
 * Overridable at build time with
 *   RB_EXTRA_CPPFLAGS="-DBOOTUP_MOTOR_DUTY_PCT=n -DBOOTUP_MOTOR_PULSE_MS=n"
 * kept plain defines rather than Kconfig for the reason ddr_stress gives. The
 * ESP applies its own ceiling (ESP_LINK_MAX_DUTY_PCT, 25 by default) on top.
 */
#include <zephyr/drivers/uart.h>
#include <riskybird/motor_link.h>

#if !DT_NODE_EXISTS(DT_ALIAS(esp_uart))
#error "bootup_check --mode motors drives the motors through the ESP link: build \
against a shell that elaborates serial@10021000 so rb appends fpga-esp-uart.overlay."
#endif

#ifndef BOOTUP_MOTOR_DUTY_PCT
#define BOOTUP_MOTOR_DUTY_PCT 15
#endif
#ifndef BOOTUP_MOTOR_PULSE_MS
#define BOOTUP_MOTOR_PULSE_MS 1000
#endif
/*
 * Seconds of warning before the sweep starts.
 *
 * There is deliberately no keypress gate. It could not be answered through a
 * piped GDB session -- the console is a separate serial node -- so a run driven
 * by rb debug sat at the prompt until it timed out and reported nothing, and
 * needing an extra -D flag to get past it made the documented command wrong.
 * Choosing --mode motors is the opt-in.
 */
#ifndef BOOTUP_MOTOR_WARN_SEC
#define BOOTUP_MOTOR_WARN_SEC 3
#endif
/* 50 Hz, as the flight controller and motor_duty send: 6x margin on the
 * ESP's 120 ms frame timeout. */
#define BOOTUP_MOTOR_FRAME_MS 20
#define BOOTUP_MOTOR_GAP_MS   700
#define BOOTUP_MOTOR_COUNT    MOTOR_LINK_NMOTORS
#define BOOTUP_MOTOR_NONE     MOTOR_LINK_NMOTORS

BUILD_ASSERT(BOOTUP_MOTOR_DUTY_PCT > 0 && BOOTUP_MOTOR_DUTY_PCT <= 25,
	     "BOOTUP_MOTOR_DUTY_PCT is a liveness sweep, not thrust: 1..25 "
	     "(the ESP's default ceiling is 25)");

static const struct device *const esp_link = DEVICE_DT_GET(DT_ALIAS(esp_uart));
static uint32_t motor_frames;

/* One frame: `motor` at `duty` (1/10000) and ARMED, or all four off and
 * disarmed when motor is BOOTUP_MOTOR_NONE. */
static void motor_frame(unsigned int motor, uint16_t duty)
{
	static uint8_t seq;
	uint16_t d[MOTOR_LINK_NMOTORS] = { 0, 0, 0, 0 };
	uint8_t frame[MOTOR_LINK_FRAME_LEN];
	uint8_t flags = 0u;

	if (motor < BOOTUP_MOTOR_COUNT && duty > 0u) {
		d[motor] = duty;
		flags = MOTOR_LINK_FLAG_ARMED;
	}
	motor_link_encode(frame, ++seq, flags, d);
	for (unsigned int i = 0; i < MOTOR_LINK_FRAME_LEN; i++) {
		uart_poll_out(esp_link, frame[i]);
	}
	motor_frames++;
}

/*
 * Hold one command for `ms`, a frame every BOOTUP_MOTOR_FRAME_MS. The watchdog
 * latch is checked before EVERY frame: on a trip this sends an explicit
 * disarmed all-zero frame at once and returns false, rather than relying on
 * the ESP noticing that frames stopped.
 */
static bool motor_hold(unsigned int motor, uint16_t duty, uint32_t ms)
{
	int64_t end = k_uptime_get() + ms;

	while (k_uptime_get() < end) {
		if (rb_motor_guard_tripped()) {
			motor_frame(BOOTUP_MOTOR_NONE, 0u);
			return false;
		}
		motor_frame(motor, duty);
		k_msleep(BOOTUP_MOTOR_FRAME_MS);
	}
	return true;
}

static void stage_motors(void)
{
	const uint16_t duty = (uint16_t)(BOOTUP_MOTOR_DUTY_PCT * 100U);
	unsigned int commanded = 0;

	if (!device_is_ready(esp_link)) {
		finish(S_MOTORS, BAD, "esp-uart not ready -- no frames can be sent");
		return;
	}
	printf("         motors are driven by the ESP32 from motor_link frames on %s;\n"
	       "         this image drives no motor pin. The ESP needs esp_bridge --mode\n"
	       "         motors, applies its own duty ceiling, and cuts all four if\n"
	       "         frames stop for 120 ms.\n", esp_link->name);
	printf("\n"
	       "  Motors run from +BATT (battery port); keep the propellers OFF for\n"
	       "  this sweep. This stage checks that each frame was sent, not that\n"
	       "  anything turned: watch the motors, or read the ESP's RBLINK duty.\n");

	/*
	 * Attitude watchdog, armed BEFORE the countdown so it is already
	 * sampling when the first motor turns. It latches on excess tilt, on free
	 * fall, or on impact, and motor_hold() stops on the next frame.
	 *
	 * This stage had no supervision of any kind until 2026-09-21, when the
	 * drone was knocked off the bench mid-sweep and kept driving all the way
	 * down and through what it hit. Free fall is the condition that catches
	 * that: a frame dropped flat never tilts, so a tilt-only guard watches it
	 * fall and does nothing.
	 *
	 * A guard that will not start is a FAILED stage, not a warning.
	 */
	{
		int grc = rb_motor_guard_start_watch(NULL);

		if (grc != 0) {
			motor_frame(BOOTUP_MOTOR_NONE, 0u);
			finish(S_MOTORS, BAD,
			       "attitude watchdog would not start (%d) -- refusing to sweep",
			       grc);
			return;
		}
	}
	printf("         attitude watchdog armed: tilt>45deg, free fall, or impact "
	       "stops the sweep\n");

	/*
	 * Opt-in guard self-test, motors OFF. Build with
	 *   RB_EXTRA_CPPFLAGS="-DBOOTUP_GUARD_SELFTEST=1"
	 * and the guard runs for BOOTUP_GUARD_SELFTEST_SEC seconds, printing what
	 * it can see, before anything is allowed to spin. Showing the live tilt
	 * makes "it did not trip" a measurement instead of an absence.
	 */
#ifndef BOOTUP_GUARD_SELFTEST
#define BOOTUP_GUARD_SELFTEST 0
#endif
#ifndef BOOTUP_GUARD_SELFTEST_SEC
#define BOOTUP_GUARD_SELFTEST_SEC 25
#endif
#if BOOTUP_GUARD_SELFTEST
	printf("\n         GUARD SELF-TEST: %d s, motors stay OFF.\n",
	       (int)BOOTUP_GUARD_SELFTEST_SEC);
	printf("         Tilt the drone past 45 deg, or lift and drop it onto\n"
	       "         something soft. tilt%% is sin^2(angle); it trips at 50.\n\n");
	for (int t = 0; t < BOOTUP_GUARD_SELFTEST_SEC * 2; t++) {
		int32_t tp = 0, am = 0;

		rb_motor_guard_live(&tp, &am);
		printf("         tilt%%=%3d (trip at 50)   |a|=%5d mm/s^2   %s\n",
		       (int)tp, (int)am,
		       rb_motor_guard_tripped() ? "<<< TRIPPED" : "");
		if (rb_motor_guard_tripped()) {
			finish(S_MOTORS, BAD,
			       "GUARD SELF-TEST TRIPPED: %s (|a|=%d mm/s^2) -- the watchdog works; no sweep run",
			       rb_motor_guard_reason_str(),
			       (int)rb_motor_guard_trip_accel());
			rb_motor_guard_stop();
			return;
		}
		k_msleep(500);
	}
	printf("         self-test finished without tripping.\n\n");
#endif

	/* Disarmed all-zero frames through the countdown: the ESP sees a live,
	 * explicitly-off link before the first armed frame arrives. */
	for (int left = BOOTUP_MOTOR_WARN_SEC; left > 0; left--) {
		printf("  sweeping in %d ...\n", left);
		if (!motor_hold(BOOTUP_MOTOR_NONE, 0u, 1000U)) {
			rb_motor_guard_stop();
			finish(S_MOTORS, BAD, "watchdog tripped before the sweep: %s",
			       rb_motor_guard_reason_str());
			return;
		}
	}
	printf("\n");

	/* One at a time, with a gap, announced before and after, so an operator
	 * can tie what they see to one motor. */
	printf("         observe each motor as it is commanded:\n");
	for (unsigned int i = 0; i < BOOTUP_MOTOR_COUNT; i++) {
		uint32_t before = motor_frames;
		bool ok;

		printf("         motor%u -> %u%% for %u ms ... ", i + 1U,
		       (unsigned)BOOTUP_MOTOR_DUTY_PCT, (unsigned)BOOTUP_MOTOR_PULSE_MS);
		ok = motor_hold(i, duty, BOOTUP_MOTOR_PULSE_MS);
		/* Explicit off before anything is printed: console output on a
		 * polled UART takes milliseconds. */
		motor_frame(BOOTUP_MOTOR_NONE, 0u);
		printf("%u frames sent, then off\n", (unsigned)(motor_frames - before));
		if (!ok) {
			int32_t a = rb_motor_guard_trip_accel();

			rb_motor_guard_stop();
			finish(S_MOTORS, BAD,
			       "WATCHDOG STOPPED THE SWEEP at motor%u: %s (|a|=%d mm/s^2)",
			       i + 1U, rb_motor_guard_reason_str(), (int)a);
			return;
		}
		commanded++;
		if (!motor_hold(BOOTUP_MOTOR_NONE, 0u, BOOTUP_MOTOR_GAP_MS)) {
			rb_motor_guard_stop();
			finish(S_MOTORS, BAD,
			       "WATCHDOG STOPPED THE SWEEP after motor%u: %s",
			       i + 1U, rb_motor_guard_reason_str());
			return;
		}
	}

	{
		uint32_t seen = rb_motor_guard_samples();

		rb_motor_guard_stop();
		motor_frame(BOOTUP_MOTOR_NONE, 0u);
		/* Two things are reported and neither is rotation: what was sent,
		 * and how many samples the watchdog took -- zero would mean the
		 * guard thread never ran, which no "passed" message can show. */
		finish(S_MOTORS, OK,
		       "%u/%u motors commanded at %u%% via the ESP link (%u frames); "
		       "rotation not verified; watchdog clean over %u samples",
		       commanded, (unsigned)BOOTUP_MOTOR_COUNT,
		       (unsigned)BOOTUP_MOTOR_DUTY_PCT, (unsigned)motor_frames,
		       (unsigned)seen);
	}
}

#else

static void stage_motors(void)
{
	finish(S_MOTORS, SKIP, "not built for motors -- use --mode motors");
}

#endif /* BOOTUP_MOTORS */

/* ------------------------------------------------------------------- main */

int main(void)
{
	unsigned int failed = 0, warned = 0, skipped = 0;

	printf("\n=== RiskyBird bootup check -- " __DATE__ " " __TIME__ " ===\n");
	printf("One pass over every peripheral, in boot order.\n\n");

	if (!device_is_ready(i2c)) {
		printf("[%s] I2C controller not ready -- nothing below can run\n",
		       VERDICT[BAD]);
		return 1;
	}

	stage_memory();
	stage_inventory();
	stage_expander();
	stage_tof();
	stage_imu();
	stage_camera();
	stage_flow();
	stage_motors();

	printf("\n--- summary ---\n");
	for (int s = 0; s < S_COUNT; s++) {
		printf("[%s] %-20s %s\n", VERDICT[outcome[s]], STAGE_NAME[s],
		       detail[s]);
		if (outcome[s] == BAD) {
			failed++;
		} else if (outcome[s] == WARN) {
			warned++;
		} else if (outcome[s] == SKIP) {
			skipped++;
		}
	}
	printf("\n%u stage(s) failed, %u warned, %u skipped, %u passed.\n",
	       failed, warned, skipped, S_COUNT - failed - warned - skipped);
	printf("%s\n", failed == 0U ? "BOARD READY" : "BOARD NOT READY");

	/*
	 * Last, not inside stage_camera: the frame is ~79 KB and cannot be
	 * printed, so it is read out of pixels[] over JTAG by
	 * hardware/ospi/bootup-capture.gdb. Stopping here rather than mid-run
	 * means the whole console report is already out before the core halts,
	 * so a capture readout never costs you the rest of the check.
	 */
	if (camera_frame_pixels > 0U) {
		printf("\n%u bytes in pixels[] -- see hardware/ospi/bootup-capture.gdb\n",
		       camera_frame_pixels);
	}
	camera_frame_ready();
	return failed == 0U ? 0 : 1;
}
