/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * WiFi SoftAP UDP telemetry DOWNLINK (see docs/TELEMETRY_PLAN.md, roadmap phase 2).
 *
 * The flight controller hosts a 2.4 GHz SoftAP ("riskybird-<id>"); a laptop joins it directly and
 * receives a UDP telemetry datagram at a fixed rate (default 50 Hz) on port 14550. The whole path
 * is DECOUPLED from the ~1 kHz control loop: the control loop only does a non-blocking, mutex-guarded
 * copy into a `telem_snapshot` (telem_wifi_publish); a dedicated LOW-priority thread formats + sends
 * (WiFi TX can block -- it must NEVER run inside the control loop, the same rule that put the
 * down-ToF / optical-flow reads on their own threads).
 *
 * ENTIRELY gated on CONFIG_WIFI: the normal flight build (no telem.conf) never compiles this file
 * (see CMakeLists.txt) and never references these symbols (all call sites in main.cpp are
 * `#if defined(CONFIG_WIFI)`), so the flight binary stays byte-identical.
 */
#ifndef ROSE_TELEM_WIFI_H
#define ROSE_TELEM_WIFI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Latest control-loop state the downlink publishes. v1 = the exact ROSE_TELEM text line
 * (main.cpp: "it=... roll=... u=[...]") so the existing Python parsers work unchanged; keep the
 * fields flat so a v2 compact-binary format can drop in behind the same publish/format seam. */
/* Bits for telem_snapshot.flags (also emitted on the wire as `st=<n>`). */
#define TELEM_FLAG_ARMED   0x1u   /* motors live */
#define TELEM_FLAG_ESTOP   0x2u   /* emergency-stop latched */
#define TELEM_FLAG_ARMING  0x4u   /* autoflight arm-settle countdown in progress */
#define TELEM_FLAG_CALDONE 0x8u   /* gyro-bias cal finished */

struct telem_snapshot {
	int32_t  iter;      /* control-loop iteration counter */
	int32_t  dt_ms;     /* measured loop period, ms */
	float    roll;      /* state[3], rad */
	float    pitch;     /* state[4], rad */
	float    yaw;       /* state[5], rad */
	float    z;         /* state[2], m (estimated height) */
	int32_t  tof_valid; /* down-ToF sample valid flag */
	float    height;    /* f.height, m (raw down-ToF, tilt-corrected) */
	float    u[4];      /* motor commands */
	/* v1.1 additions (appended to the wire line, so existing parsers ignore them): */
	float    x;         /* state[0], m (dead-reckoned horizontal position) */
	float    y;         /* state[1], m */
	float    vx;        /* state[6], m/s (est horizontal velocity, flow-fed) */
	float    vy;        /* state[7], m/s */
	float    vz;        /* state[8], m/s (climb rate) */
	float    zsp;       /* g_setpoint[2], m (altitude target) */
	float    vbat;      /* smoothed pack voltage, V (0 if battery sense off) */
	uint32_t flags;     /* TELEM_FLAG_* */
	/* v1.2 additions -- guard diagnostics, so a cut explains itself on the panel instead of only on
	 * the (unreadable-in-flight) console. Appended to the wire line; older parsers ignore them. */
	float    amag;      /* |accel| at the last envelope-guard eval, m/s^2 */
	float    apk;       /* decaying peak |accel| (~0.7 s), m/s^2 -- the vibration level */
	const char *why;    /* reason the estop latched, "-" if none (string literal: impact, free-fall,
	                     * accel-tilt, tilt, rate, yaw-spin, velocity, height, battery, imu-lost, cmd) */
	float    why_meas;  /* the value measured at the latch, in that reason's unit */
	/* down-ToF health (it has been failing in flight): consecutive failed fetches, last error rc
	 * (-5 EIO = bus/wiring/power/address, -16 EBUSY / -116 ETIMEDOUT = ranging), recoveries */
	int32_t  tof_fails;
	int32_t  tof_rc;
	uint32_t tof_recov;
};

/* ---- Uplink command hooks (port 14551) -- IMPLEMENTED in main.cpp -----------------------------
 * The command-RX thread (telem_wifi.c) parses uplink datagrams and calls these; each just pokes a
 * volatile shared flag the control loop consumes next iteration (the plan's shared-flag design).
 * SAFETY: rose_cmd_estop() is the remote kill (latched, cleared only by reset). There is
 * deliberately NO remote force-ARM -- arming stays on the on-board gate. */
void rose_cmd_tof_reset(void);      /* pulse the down-ToF XSHUT + readdress + re-init (on the ToF thread) */
void rose_cmd_estop(void);          /* latch emergency stop -> all motors off (unrecoverable w/o reset) */
void rose_cmd_disarm(void);         /* clear armed -> motors off (autoflight can re-arm via the gate) */
void rose_cmd_set_hover_z(float m); /* set hover altitude setpoint, m (clamped to [0, SAFE_MAX_HEIGHT]) */
void rose_cmd_reset(void);          /* SOFT reset: back to just-booted (clears estop, disarms, re-cals gyro) -- no chip reset */
void rose_cmd_set_profile(int climb_ms, int hover_ms, int descend_ms, int max_ms); /* autoflight profile durations */
#if defined(ROSE_MANUAL_PILOT) && ROSE_MANUAL_PILOT
void rose_cmd_pilot(float vx, float vy, float yawrate, float thr);  /* manual pilot: body vel (m/s) + yaw rate (rad/s) + throttle [0..1] */
#endif

/* Bring up the SoftAP (SSID riskybird-<id>) + DHCPv4 server + start the UDP telemetry thread.
 * Call ONCE at boot, after sensors are up, before the control loop. Returns 0 on success. */
int telem_wifi_init(void);

/* Publish the latest state for the downlink. Non-blocking: takes a mutex only for the struct copy
 * (a few us), so it is safe to call every control-loop iteration. No-op until telem_wifi_init(). */
void telem_wifi_publish(const struct telem_snapshot *s);

#ifdef __cplusplus
}
#endif

#endif /* ROSE_TELEM_WIFI_H */
