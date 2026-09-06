/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Himax HM01B0 register definitions used by the camera bring-up program.
 *
 * PROVENANCE. Two different levels of confidence live in this file and they are
 * labelled, because bring-up debugging is impossible if a wrong constant is
 * indistinguishable from a wrong wire.
 *
 *  VERIFIED IN THIS REPOSITORY
 *    - 7-bit I2C address 0x24. Recorded in hardware/ospi/docs/README.md
 *      ("Chipyard's TLI2C controller configures the HM01B0 at its default 7-bit
 *      address 0x24") and repeated in the WithArty100TI2C binder comment.
 *    - 16-bit register address, 8-bit register data; I2C at most 400 kHz.
 *      Same source, "Datasheet conformance boundary" table.
 *    - 8-bit parallel output with PCLK/FVLD/LVLD, maximum PCLK 36 MHz,
 *      MCLK 3-36 MHz. Same table.
 *
 *  FROM THE HM01B0 DATASHEET, NOT INDEPENDENTLY CONFIRMED AGAINST A COPY IN
 *  THIS REPOSITORY. These are the standard Himax addresses also used by the
 *  public SparkFun Edge / TensorFlow Lite Micro himax drivers. Confirm against
 *  the datasheet before trusting a failure that implicates one of them.
 */

#ifndef HM01B0_H_
#define HM01B0_H_

/* --- verified in this repository --- */
#define HM01B0_I2C_ADDR          0x24U

/* --- datasheet values, see the provenance note above --- */
#define HM01B0_REG_MODEL_ID_H    0x0000U /* expect 0x01 */
#define HM01B0_REG_MODEL_ID_L    0x0001U /* expect 0xB0 */
#define HM01B0_MODEL_ID          0x01B0U

#define HM01B0_REG_MODE_SELECT   0x0100U /* 0 = standby, 1 = streaming */
#define HM01B0_MODE_STANDBY      0x00U
#define HM01B0_MODE_STREAMING    0x01U

#define HM01B0_REG_SW_RESET      0x0103U /* write any value to reset */
#define HM01B0_SW_RESET_VALUE    0x00U

#define HM01B0_REG_GRP_PARAM_HOLD 0x0104U /* 1 = hold, 0 = commit */

#define HM01B0_REG_TEST_PATTERN  0x0601U
#define HM01B0_TESTPAT_OFF       0x00U
#define HM01B0_TESTPAT_WALKING1  0x01U

/*
 * UNVERIFIED EXPECTATION. The "walking 1s" pattern is understood to emit
 * 0x01, 0x02, 0x04 ... 0x80 and repeat. This program reports whether the
 * captured data matches that, but does NOT gate its PASS verdict on it,
 * because the exact pattern semantics are not established by any source in
 * this repository. Treat a mismatch as "confirm the datasheet", not as a
 * proven hardware fault.
 */
#define HM01B0_WALKING1_FIRST    0x01U

/*
 * Data-width mode is deliberately NOT programmed. The sensor's reset default
 * is the 8-bit parallel output this capture core requires, so bring-up relies
 * on the reset state rather than writing a mode register whose encoding is not
 * established here. If the sensor were to come up in 1-bit or 4-bit mode the
 * pixel values would be wrong while the sync counters still looked healthy --
 * which is exactly what the per-pixel dump below is for.
 */

#endif /* HM01B0_H_ */
