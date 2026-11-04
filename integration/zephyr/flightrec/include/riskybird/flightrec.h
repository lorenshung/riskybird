/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * DRAM flight recorder: every control tick (or every Nth), in full precision,
 * into a ring buffer in DDR, read out over WiFi after the flight.
 *
 * WHY. The live telemetry line is ~300 bytes on a 115200 wire it shares with
 * the motor frames, so it tops out near 30 Hz -- too coarse to tune an
 * attitude loop. This records at control rate and costs nothing on the wire
 * during the flight. Afterwards the host reads it through an ESP RBPOKE1
 * session (TSI reads under the running program, no reset):
 *
 *   rb flightlog fetch --port socket://192.168.4.1:14552 --mode <preset>
 *
 * WHAT IS KEPT. The ring is written continuously, so the seconds before the
 * motors start are already in it. When the motors stop (every duty zero) for
 * FLIGHTREC_TAIL_MS the recorder FREEZES: the flight and its tail stay put no
 * matter how long the drone then sits idle. The next time any motor is
 * commanded it unfreezes and a new segment starts. The header says where the
 * last segment begins and ends, so the host reads only that.
 *
 * PREROLL. While frozen the main ring is not written, so the samples just
 * before a later segment are not in it. Frozen idle ticks go to one of two
 * small preroll rings instead; when a segment starts from frozen, that ring
 * becomes the segment's preroll (pre_buf/pre_head/pre_count) and the next
 * frozen idle writes the OTHER ring, so a fetch of a frozen segment always
 * finds its own preroll intact. A segment that starts while the main ring is
 * still recording (the first one, or a restart inside the tail) takes its
 * preroll from the main ring, back to the last unfreeze at most; if that is
 * not a full preroll it keeps the previous segment's preroll ring as well.
 *
 * LAYOUT IS A CONTRACT with tools/rb/flightrec_fetch.py: bump FLIGHTREC_VERSION on
 * any change to either struct.
 *
 * TWO SAMPLE LAYOUTS, chosen at build time. Flight builds record struct
 * flightrec_sample under header version 2 (the 64-byte struct flightrec_header,
 * no layout field). Roll-harness builds (ROSE_RIG=1, workloads/flight_controller
 * fc_rig.h) record struct flightrec_rig_sample under header version 3: the same
 * 64 bytes, then struct flightrec_rig_ext with layout = FLIGHTREC_LAYOUT_RIG, a
 * snapshot of every rig gain taken at segment start and a ring of the last
 * FLIGHTREC_GAIN_RING accepted gain changes, so a trial's gains can be rebuilt
 * from the fetched log alone. The ring/freeze/preroll logic is the same code for
 * both; only flightrec_rec_t differs.
 */
#ifndef RISKYBIRD_FLIGHTREC_H
#define RISKYBIRD_FLIGHTREC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FLIGHTREC_MAGIC   0x31435246u   /* "FRC1" little-endian */
#define FLIGHTREC_VERSION 2u
#define FLIGHTREC_VERSION_RIG 3u        /* header + struct flightrec_rig_ext */

/* Sample layouts (struct flightrec_rig_ext.layout; version 2 implies FLIGHT). */
#define FLIGHTREC_LAYOUT_FLIGHT 1u
#define FLIGHTREC_LAYOUT_RIG    2u

#if defined(ROSE_RIG) && ROSE_RIG
#define FLIGHTREC_RIG 1
#else
#define FLIGHTREC_RIG 0
#endif

/* Flag bits in flightrec_sample.flags. */
#define FLIGHTREC_F_ARMED      0x01u
#define FLIGHTREC_F_ESTOP      0x02u
#define FLIGHTREC_F_ARMING     0x04u
#define FLIGHTREC_F_CALDONE    0x08u
#define FLIGHTREC_F_TOF_VALID  0x10u
#define FLIGHTREC_F_FLOW_VALID 0x20u
#define FLIGHTREC_F_COMMITTED  0x40u

/* One sample, 84 bytes. Units are the controller's own: attitude is the
 * estimator's Gibbs/Rodrigues vector (tan(theta/2) per axis, NOT radians),
 * rates rad/s, duty and u as the controller produced them. */
struct flightrec_sample {
	uint32_t t_us;        /* uptime, microseconds (wraps after ~71 min) */
	uint16_t dt_us;       /* this tick's measured loop period */
	uint8_t  flags;       /* FLIGHTREC_F_* */
	uint8_t  seq;         /* low byte of the tick counter: gaps show dropped ticks */
	float    gyro[3];     /* body rates p q r, rad/s, as the estimator received them */
	float    att[3];      /* state[3..5]: roll pitch yaw, Gibbs */
	float    sp_att[3];   /* setpoint[3..5] */
	float    z;           /* state[2], m */
	float    zsp;         /* setpoint[2], m */
	float    u[4];        /* controller output, pre-mixing offset and caps */
	float    duty[4];     /* duty actually sent to the motors, 0..1 */
};

/* Placed first in the recorder's storage; the host finds it by symbol. */
struct flightrec_header {
	uint32_t magic;       /* FLIGHTREC_MAGIC */
	uint16_t version;     /* FLIGHTREC_VERSION */
	uint16_t sample_size; /* sizeof(struct flightrec_sample) */
	uint32_t capacity;    /* samples in the ring */
	uint32_t div;         /* recorded every div-th call */
	uint32_t tail_ms;     /* idle time after the motors stop before freezing */
	uint32_t preroll;     /* samples kept before a segment's first motor tick */
	volatile uint32_t count;      /* samples ever written (ring index = n % capacity) */
	volatile uint32_t seg_start;  /* count at the current/last segment start (incl. preroll) */
	volatile uint32_t seg_end;    /* count when that segment froze; 0 while recording */
	volatile uint32_t segments;   /* segments started since boot */
	volatile uint32_t frozen;     /* 1 while frozen */
	uint32_t samples_addr;        /* low 32 bits of &samples[0] (DDR is below 4 GiB) */
	uint32_t pre_addr;            /* low 32 bits of the two preroll rings, [2][preroll] */
	volatile uint32_t pre_buf;    /* which preroll ring belongs to the last segment */
	volatile uint32_t pre_head;   /* its oldest sample's index */
	volatile uint32_t pre_count;  /* samples in it (0: the preroll is in the main ring) */
};

#if FLIGHTREC_RIG
/* Rig gains in the snapshot: the first 12 of fc_rig.h's enum fc_rig_gain_id, in its order.
 * Gains appended to the enum after them (LIFT_I, LIFT_I_LIM) are NOT in the snapshot, so the
 * layout stays v3; their accepted changes are still logged in changes[] (index >= 12). */
#define FLIGHTREC_RIG_GAINS 12u
/* Accepted gain changes kept in the header area. */
#define FLIGHTREC_GAIN_RING 64u

/* One sample, 92 bytes, roll-harness build. Angles are TRUE radians (fc_rig's
 * dbg, from fc_pid_attitude), not Gibbs. Every float after gyro is the value of
 * struct fc_rig_dbg the roll law published this tick. */
struct flightrec_rig_sample {
	uint32_t t_us;        /* uptime, microseconds (wraps after ~71 min) */
	uint16_t dt_us;       /* this tick's measured loop period */
	uint8_t  seq;         /* low byte of the tick counter */
	uint8_t  flags;       /* FLIGHTREC_F_* */
	uint8_t  rig_state;   /* enum fc_rig_run_state */
	uint8_t  exc_id;      /* low byte of g_rig.exc_id (bumped per EXC) */
	uint16_t gain_seq;    /* low 16 bits of g_rig.gain_seq (bumped per GAIN) */
	float    gyro[3];     /* body rates p q r the rate loop used, rad/s */
	float    roll, pitch; /* rad */
	float    roll_tgt;    /* rad: setpoint + trim + exc(att) */
	float    rate_tgt;    /* rad/s */
	float    rate_cmd;    /* rad/s^2 */
	float    moment;      /* N*m into the mixer */
	float    exc_val;     /* injected value, in its target's SI unit */
	float    coll_thrust; /* N */
	float    att_int;     /* integrator contributions */
	float    rate_int;
	float    pitch_cmd;   /* would-be commands, rad/s^2, NOT sent to the motors */
	float    yaw_cmd;
	float    accel_y;     /* m/s^2, body */
	float    accel_z;
	float    vbat;        /* V; -1 battery sense not built, -2 no valid sample yet */
	uint16_t duty[4];     /* duty sent to the motors, 1/10000 */
};

struct flightrec_gain_change {
	uint16_t gain_seq;    /* g_rig.gain_seq after the change (low 16 bits) */
	uint8_t  index;       /* enum fc_rig_gain_id */
	uint8_t  pad;
	float    value;
	uint32_t t_us;        /* same clock as flightrec_rig_sample.t_us */
};

/* Follows struct flightrec_header (version FLIGHTREC_VERSION_RIG) in storage. */
struct flightrec_rig_ext {
	uint32_t layout;              /* FLIGHTREC_LAYOUT_RIG */
	uint16_t n_gains;             /* FLIGHTREC_RIG_GAINS */
	uint16_t gain_ring;           /* FLIGHTREC_GAIN_RING */
	volatile uint32_t snap_gain_seq;  /* gain_seq of the snapshot */
	volatile uint32_t snap_segment;   /* header.segments when taken (0 = at init) */
	volatile float    snap_gains[FLIGHTREC_RIG_GAINS];
	volatile uint32_t gain_count; /* changes ever logged (ring index = n % FLIGHTREC_GAIN_RING) */
	struct flightrec_gain_change changes[FLIGHTREC_GAIN_RING];
};

struct flightrec_rig_header {
	struct flightrec_header h;
	struct flightrec_rig_ext rig;
};

typedef struct flightrec_rig_sample flightrec_rec_t;
#else
typedef struct flightrec_sample flightrec_rec_t;
#endif

/* Set up the header. Safe to call more than once. */
void flightrec_init(void);

/*
 * Offer one control tick. Records every div-th call while not frozen.
 * motors_active: any duty is nonzero this tick (drives segment start/freeze).
 */
void flightrec_tick(const flightrec_rec_t *s, bool motors_active);

#if FLIGHTREC_RIG
/*
 * Where the live gains are (g_rig.gains.v, &g_rig.gain_seq). The recorder copies
 * them into the header snapshot now and at every segment start. Call after
 * flightrec_init() and after the gains have their defaults.
 */
void flightrec_rig_bind(const float *gains, const volatile uint32_t *gain_seq);
/* Log one accepted gain change (call once per accepted GAIN). */
void flightrec_rig_gain(uint32_t gain_seq, unsigned index, float value, uint32_t t_us);
#endif

/* The header, for a banner or a debugger. */
const struct flightrec_header *flightrec_header(void);

#ifdef __cplusplus
}
#endif

#endif /* RISKYBIRD_FLIGHTREC_H */
