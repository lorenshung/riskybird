/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * PMW3901 optical-flow front-end (see flow.h). Reuses the standalone pmw3901.c driver
 * (workloads/pmw3901_test) with the optimized busy_wait_us read path (~285 us/read).
 */
#include "flow.h"
#include "fc_shared_math.h"
#include "pmw3901.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <errno.h>
#include <math.h>

/*
 * Which SPI controller the PMW3901 hangs off.
 *
 * This used to be a bare DT_NODELABEL(spi2) -- the ESP32-C6's controller -- resolved in the
 * preprocessor. Any board without a node labelled exactly `spi2` therefore failed to BUILD the
 * whole flight controller, even with the flow feature off, because CMakeLists compiles this file
 * unconditionally. The RiskyBird FPGA carrier's flow sensor is on spi@10031000, labelled spi0.
 *
 * A board now points at its own controller with a `flow-spi` alias; boards that do not are
 * unchanged, because the fallback is the original spi2 label. The ESP build therefore selects
 * exactly the same node as before.
 *
 * NOTE this is deliberately NOT the `flow` alias. That alias means the RoSE *virtual* flow
 * device and drives HAVE_FLOW in main.cpp (spike co-sim only); the real PMW3901 path is selected
 * by the ROSE_FLOW CMake knob and talks to the bus directly, with no Zephyr driver and no `flow`
 * node. The ESP flies with ROSE_FLOW=1 and no `flow` alias at all, so guarding this file on that
 * alias would silently disable optical flow on the configuration that actually flew.
 */
/*
 * RoSE co-sim transport (spike_riscv64 + ROSE_FLOW=1). There is no SPI bus in the co-sim, so the
 * motion burst comes from the RoSE bridge instead: the env models the PMW3901 itself
 * (riskybird-sim riskybird_sim/sensors/pmw3901.py) and serves its burst registers on command
 * ROSE_FLOW_BURST_CMD. Everything from motionBurst_t on -- the count scaling, the low-pass, the
 * SQUAL gate, the staleness window, and main.cpp's gyro compensation and fc_flow_velocity -- is the
 * code the drone runs. Only the bytes' origin differs.
 *
 * The env cannot know when the firmware reads, so it serves the counts as RUNNING TOTALS and the
 * read below differences them: the delta since this read's predecessor, which is exactly the
 * sensor's accumulate-and-clear-on-read register behaviour. Words (float32): cumulative dX,
 * cumulative dY, SQUAL, Shutter_Upper, spare.
 *
 * The request is issued with the IMU's (flow_rose_request) and collected in the same grant
 * (flow_rose_collect), on reqrsp channel ROSE_FLOW_BURST_CH, BEFORE the down-ToF request goes out
 * on that channel, so the two responses can never interleave. No thread.
 */
#if DT_HAS_COMPAT_STATUS_OKAY(ucbbar_roseadapter) && !DT_NODE_EXISTS(DT_ALIAS(flow_spi))
#define FLOW_ROSE       1
#define HAVE_FLOW_HW    1
#include <rose/rose_proto.h>
#include <string.h>
#ifndef ROSE_FLOW_BURST_CMD
#define ROSE_FLOW_BURST_CMD 0x16u
#endif
#ifndef ROSE_FLOW_BURST_CH
#define ROSE_FLOW_BURST_CH 1u
#endif
#define ROSE_FLOW_BURST_WORDS 5
#elif DT_NODE_EXISTS(DT_ALIAS(flow_spi))
#define FLOW_SPI_NODE   DT_ALIAS(flow_spi)
#define HAVE_FLOW_HW    1
#elif DT_NODE_EXISTS(DT_NODELABEL(spi2))
#define FLOW_SPI_NODE   DT_NODELABEL(spi2)
#define HAVE_FLOW_HW    1
#else
#define HAVE_FLOW_HW    0
#endif
#ifndef FLOW_ROSE
#define FLOW_ROSE 0
#endif

#if !HAVE_FLOW_HW

int flow_init(void)
{
	printk("flow: no flow-spi alias and no spi2 node -- optical flow unavailable\n");
	return -ENODEV;
}

void flow_get(float *ang_x, float *ang_y, int *squal, bool *valid)
{
	if (ang_x) { *ang_x = 0.0f; }
	if (ang_y) { *ang_y = 0.0f; }
	if (squal) { *squal = 0; }
	if (valid) { *valid = false; }
}

int64_t flow_sample_ms(void)
{
	return INT64_MIN / 2;
}

void flow_rose_request(void) { }
void flow_rose_collect(void) { }

#else

/* --- tunables (override via -D) --- */
#ifndef FLOW_RAD_PER_COUNT
/* rad of ground-feature angle per PMW3901 motion count, taken from the Crazyflie flow deck (same
 * sensor + same Bitcraze register init, field-proven). Their measurement model is
 *   pixels = (Npix/thetapix) * angular_flow * dt,   measured_pixels = deltaX * FLOW_RESOLUTION
 * with Npix=35, thetapix=2*sin(42deg/2)=0.71674 rad, and FLOW_RESOLUTION=0.10 (the sensor reports
 * motion in 10x pixels). So rad/count = FLOW_RESOLUTION * thetapix / Npix = 0.10*0.71674/35 = 0.00205.
 * (An earlier 0.021 here omitted the 0.10 "10x pixels" factor -> ~10x too large; bench gyro-ratio and
 * slide-integration data bracketed 0.001-0.0035, consistent with 0.00205.) */
#define FLOW_RAD_PER_COUNT 0.00205f
#endif
#ifndef FLOW_MIN_SQUAL
#define FLOW_MIN_SQUAL 19        /* drop low-quality samples (surface-quality floor) */
#endif
#ifndef FLOW_PERIOD_MS
#define FLOW_PERIOD_MS 10        /* ~100 Hz background read; each read is ~285 us so it mostly idles */
#endif
#ifndef FLOW_STALE_MS
#define FLOW_STALE_MS 50         /* flow marked invalid if no fresh sample within this window */
#endif
/* Per-axis low-pass on the angular flow, applied HERE in the ~100 Hz sensor thread (NOT in the 1 kHz
 * estimator, where re-fusing the same stale sample every control tick converges right back to the raw
 * value and defeats any per-tick gain). Time-constant based (uses the measured dt) so it's independent
 * of the read period. The PMW3901's left (y) axis is far noisier (std ~1.3 m/s, railing to +/-3), yet a
 * real drift survives underneath (2:1 sign-imbalanced) -- a heavy y low-pass collapses the noise while
 * keeping that drift; x is usually cleaner so it needs only a light touch. Tune via -DFLOW_LP_TAU_{X,Y}. */
#ifndef FLOW_LP_TAU_X
#define FLOW_LP_TAU_X 0.0f       /* x low-pass time constant (s); 0 = off (pass raw) */
#endif
#ifndef FLOW_LP_TAU_Y
#define FLOW_LP_TAU_Y 0.0f       /* y low-pass time constant (s); 0 = off */
#endif

/*
 * Software chip-select pin, on the controller named by `gpio0`.
 *
 * 19 is the ESP32-C6's (v3: SCLK=6 MOSI=7 MISO=18 CS=19). The FPGA carrier's
 * DroneLogic GPIO block is a DIFFERENT controller with its own numbering, and
 * the flow sensor's CS is logical index 0 there -- which is what
 * hardware/zephyr/targets/fpga/workloads/pmw3901_test.overlay declares and what
 * workloads/bootup_check reads the chip ID through successfully.
 *
 * THIS WAS NOT OVERRIDABLE. Both flight_controller-flow.overlay and
 * docs/flight-on-fpga.md tell the operator to pass -DCS_GPIO_PIN=0 for the FPGA
 * build, and a bare `#define` here silently won that argument: the command-line
 * definition is redefined by this line (a warning, not an error), so an FPGA
 * flow build drove gpio0 pin 19 as chip select and the sensor never answered.
 * The #ifndef makes the documented override actually work.
 */
#ifndef CS_GPIO_PIN
#if defined(CONFIG_BOARD_CHIPYARD_RISCV64)
#define CS_GPIO_PIN 0
#else
#define CS_GPIO_PIN 19
#endif
#endif

#if !FLOW_ROSE
/* Manually-constructed pmw3901 device (the driver takes cfg/data via a struct device, same as the
 * pmw3901_test bring-up sample). */
static struct pmw3901_config pmw3901_cfg;
static struct pmw3901_data   pmw3901_data;
static const struct device   pmw3901_device = {
	.name = "FLOW", .config = &pmw3901_cfg, .data = &pmw3901_data,
};
#define PMW3901_DEV (&pmw3901_device)
#endif

/* --- cached flow snapshot --- */
static K_MUTEX_DEFINE(flow_mtx);
static float   g_ax, g_ay;       /* body-frame angular flow rate (rad/s), remapped */
static int     g_squal;
static bool    g_squal_ok;
static uint32_t g_seq;
static int64_t g_last_ms = -100000;

/* One motion burst -> the cached angular flow. Shared by the SPI thread and the RoSE transport, so
 * both run the same conversion, filter and gate. dt = time since the previous burst read. */
static void flow_ingest(const motionBurst_t *m, float dt)
{
	if (dt > 1e-4f && dt < 0.5f) {
		static struct fc_flow_filter filtered;
		fc_flow_counts(&filtered, m->deltaX, m->deltaY, dt, FLOW_RAD_PER_COUNT,
			       FLOW_LP_TAU_X, FLOW_LP_TAU_Y);
		k_mutex_lock(&flow_mtx, K_FOREVER);
		g_ax = filtered.x; g_ay = filtered.y;
		g_squal = m->squal; g_squal_ok = (m->squal >= FLOW_MIN_SQUAL);
		g_last_ms = k_uptime_get();
		g_seq++;
		k_mutex_unlock(&flow_mtx);
	}
}

#if FLOW_ROSE
static const struct device *flow_rose = DEVICE_DT_GET_ONE(ucbbar_roseadapter);
static bool     flow_rose_pending;
static bool     flow_rose_seeded;
static float    flow_rose_cum[2];
static uint32_t flow_rose_t_prev;

void flow_rose_request(void)
{
	rose_request(flow_rose, ROSE_FLOW_BURST_CMD, 0U);
	flow_rose_pending = true;
}

void flow_rose_collect(void)
{
	if (!flow_rose_pending) {
		return;
	}
	flow_rose_pending = false;
	uint32_t raw[ROSE_FLOW_BURST_WORDS];
	float w[ROSE_FLOW_BURST_WORDS];
	int n = rose_recv_reqrsp(flow_rose, ROSE_FLOW_BURST_CH, raw, ROSE_FLOW_BURST_WORDS);
	const uint32_t t_now = k_cycle_get_32();
	if (n < ROSE_FLOW_BURST_WORDS) {
		printk("flow: short RoSE burst (%d/%d words)\n", n, ROSE_FLOW_BURST_WORDS);
		return;
	}
	memcpy(w, raw, sizeof(w));
	if (!flow_rose_seeded) {   /* first read only establishes the counters' origin */
		flow_rose_cum[0] = w[0]; flow_rose_cum[1] = w[1];
		flow_rose_t_prev = t_now; flow_rose_seeded = true;
		return;
	}
	motionBurst_t m;
	memset(&m, 0, sizeof(m));
	const float dx = w[0] - flow_rose_cum[0], dy = w[1] - flow_rose_cum[1];
	flow_rose_cum[0] = w[0]; flow_rose_cum[1] = w[1];
	m.deltaX = (int16_t)(dx > 32767.0f ? 32767.0f : (dx < -32768.0f ? -32768.0f : dx));
	m.deltaY = (int16_t)(dy > 32767.0f ? 32767.0f : (dy < -32768.0f ? -32768.0f : dy));
	m.squal = (uint8_t)w[2];
	m.shutter = (uint16_t)((uint16_t)w[3] << 8);
	m.motionOccured = (m.deltaX != 0 || m.deltaY != 0);
	const float dt = (float)k_cyc_to_us_floor32(t_now - flow_rose_t_prev) * 1e-6f;
	flow_rose_t_prev = t_now;
	flow_ingest(&m, dt);
}

int flow_init(void)
{
	if (!device_is_ready(flow_rose)) {
		printk("flow: RoSE adapter not ready -- flow disabled\n");
		return -ENODEV;
	}
	printk("flow: PMW3901 via RoSE burst cmd 0x%x ch%u (counts path; remap +x=-dX +y=+dY)\n",
	       (unsigned)ROSE_FLOW_BURST_CMD, (unsigned)ROSE_FLOW_BURST_CH);
	return 0;
}

#else /* SPI */
K_THREAD_STACK_DEFINE(flow_stack, 2048);
static struct k_thread flow_thread;

static void flow_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	uint32_t t_prev = k_cycle_get_32();
	for (;;) {
		motionBurst_t m;
		int rc = pmw3901_read_motion_burst(PMW3901_DEV, &m);
		uint32_t t_now = k_cycle_get_32();
		float dt = (float)k_cyc_to_us_floor32(t_now - t_prev) * 1e-6f;
		t_prev = t_now;

		if (rc == 0) {
			flow_ingest(&m, dt);
		}
		k_msleep(FLOW_PERIOD_MS);
	}
}

void flow_rose_request(void) { }
void flow_rose_collect(void) { }

int flow_init(void)
{
	const struct device *spi_dev  = DEVICE_DT_GET(FLOW_SPI_NODE);
	const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	if (!device_is_ready(spi_dev) || !device_is_ready(gpio_dev)) {
		printk("flow: SPI/GPIO not ready -- flow disabled\n");
		return -ENODEV;
	}

	pmw3901_cfg.spi.bus = spi_dev;
	pmw3901_cfg.spi.config.operation =
		SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB;
	pmw3901_cfg.spi.config.frequency = 2000000;   /* PMW3901 SPI max */
	pmw3901_cfg.spi.config.slave = 0;
	pmw3901_cfg.cs_gpio.port = gpio_dev;
	pmw3901_cfg.cs_gpio.pin = CS_GPIO_PIN;
	pmw3901_cfg.cs_gpio.dt_flags = GPIO_ACTIVE_LOW;
	/* v3: NRESET tied high (R37), MOTION -> FPGA -- no GPIO reset/LED here (driver does a soft POR) */
	pmw3901_cfg.reset_gpio.port = NULL;
	pmw3901_cfg.led_gpio.port = NULL;

	int rc = -1;
	for (int i = 0; i < 5 && rc != 0; i++) {
		rc = pmw3901_init(PMW3901_DEV);
		if (rc != 0) {
			k_msleep(50);
		}
	}
	if (rc != 0) {
		printk("flow: PMW3901 not detected (rc=%d) -- flow disabled\n", rc);
		return rc;
	}

	k_thread_create(&flow_thread, flow_stack, K_THREAD_STACK_SIZEOF(flow_stack),
			flow_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(8), 0, K_NO_WAIT);
	k_thread_name_set(&flow_thread, "flow");
	printk("flow: PMW3901 up -- background read ~%d Hz (remap +x=-dX +y=+dY)\n", 1000 / FLOW_PERIOD_MS);
	return 0;
}
#endif /* FLOW_ROSE */

void flow_get(float *ang_x, float *ang_y, int *squal, bool *valid)
{
	k_mutex_lock(&flow_mtx, K_FOREVER);
	*ang_x = g_ax; *ang_y = g_ay; *squal = g_squal;
	bool fresh = (k_uptime_get() - g_last_ms) < FLOW_STALE_MS;
	*valid = g_squal_ok && fresh && (g_seq != 0);
	k_mutex_unlock(&flow_mtx);
}

int64_t flow_sample_ms(void)
{
	k_mutex_lock(&flow_mtx, K_FOREVER);
	const int64_t t = g_last_ms;
	k_mutex_unlock(&flow_mtx);
	return t;
}

#endif /* HAVE_FLOW_HW */
