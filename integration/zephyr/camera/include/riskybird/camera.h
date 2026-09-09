/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * One HM01B0 frame, captured into a caller-supplied buffer.
 *
 * The sequence -- MCLK before SCCB, streaming before PCLK is meaningful, arm on
 * a frame boundary, stop the sensor the moment PIXTARGET is satisfied -- is
 * board knowledge, and every step of it is a way a capture can look broken
 * while the hardware is fine. It lives here so a workload that wants a picture
 * does not have to re-derive it.
 *
 * This reports rather than prints: the caller owns its own output format, and
 * bootup_check's one-line-per-stage table and camera_photo's nine stages of
 * evidence want different things from the same capture.
 *
 * NOTE ON DUPLICATION: workloads/camera_photo predates this module and still
 * carries its own copy of the sequence. It is the reference implementation the
 * camera docs quote verbatim, so it was left alone rather than refactored
 * mid-bench-session; migrating it here is the outstanding follow-up, and until
 * that happens a fix to the sequence belongs in both places.
 */

#ifndef RISKYBIRD_CAMERA_H_
#define RISKYBIRD_CAMERA_H_

#include <stdint.h>

/* What the capture observed. Populated as far as the sequence got, so a failed
 * capture still says how far it got and what the core saw. */
struct rb_camera_capture {
	uint16_t model_id;
	uint32_t pclk_before;
	uint32_t pclk_after;
	uint32_t captured;        /* pixels the core counted */
	uint32_t drained;         /* pixels actually read into the buffer */
	uint32_t measured_width;  /* LASTWIDTH -- reshape rows at THIS, not GEOM */
	uint32_t measured_height;
	uint32_t configured_width;
	uint32_t configured_height;
	uint32_t flags;
	uint8_t vmin;
	uint8_t vmax;
};

/*
 * Pixels the capture core can hold in one capture -- the hard ceiling on a
 * frame, and lower than a full frame on at least one shipped shell. A caller
 * that wants "as much of the picture as fits" clamps to whole rows against
 * this rather than hardcoding a height, so a core with a bigger buffer needs
 * no code change.
 */
uint32_t rb_camera_capacity(void);

/*
 * Capture `want` pixels into `dst`. Returns 0 on success, or -1 with *stage and
 * *detail set to static strings naming where it stopped and why.
 *
 * Needs a DDR-backed shell: a full frame is ~79 KB and a scratchpad holds 32 KB.
 */
int rb_camera_capture_frame(uint8_t *dst, uint32_t want,
			    struct rb_camera_capture *out,
			    const char **stage, const char **detail);

#endif /* RISKYBIRD_CAMERA_H_ */
