#!/usr/bin/env python3
"""FcRoCC disassembler + source-recovery tool.

  fcr_disasm.py                 annotated listing of ../vsrc/fc_rocc_rom.hex (register names)
  fcr_disasm.py --init          decode ../vsrc/fc_rocc_init.hex into named constants
  fcr_disasm.py --analyze       dead writes, redundant ops, NR-block inventory
  fcr_disasm.py --recover OUT   emit the symbolic source (how fc_rocc.fca was first produced)

fc_rocc.fca is the maintained source; --recover is the one-time recovery path from the
checked-in ROM and is kept so the derivation can be re-run and audited.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fcr_isa import OPS, decode, load_hex, s48  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROM = os.path.join(HERE, "../vsrc/fc_rocc_rom.hex")
INIT = os.path.join(HERE, "../vsrc/fc_rocc_init.hex")
EST, CTRL = 0, 537

# --------------------------------------------------------------------------------------
# compile-time parameters (C names, values as in the C sources / microcode)
SETS = """\
; ---- physical / tuning parameters (C names; f = float, as in the C sources) ----
.set GRAVITY        = 9.81f
.set IMU_OFFSET_Y   = 0.016f      ; IMU lever arm, body y (m; +y = left, FLU). C default is 0; PCB value +0.016
.set IMU_ALPHA_TAU  = 0.02f       ; gyro-derivative LP time constant (s)
.set KP             = 0.5f        ; Mahony gravity-trim gain
.set Z_GAIN         = 0.5f        ; ToF observer, z
.set VZ_GAIN        = 0.3f        ; ToF observer, vz
.set LEAD           = 1.0f        ; get_state translation lead (x dt)
.set LEAD_ATT       = 0.5f        ; get_state attitude lead (x dt)
.set TOF_MIN_COS    = 0.05f       ; R[8] > this => down-ToF is vertical enough
.set ZUPT_HEIGHT_M  = 0.05f
.set VMAX           = 5.0f        ; velocity backstop (m/s)
.set NAT_FREQ_H     = 2.0f
.set DAMP_H         = 0.7f
.set KI_HEIGHT      = 1.5f
.set ALT_INT_MAX    = 3.0f
.set ALT_INT_GROUND_M = 0.05f
.set KI_VEL         = 0.8f
.set VEL_INT_MAX    = 2.0f
.set VEL_TC         = 0.5f
.set VEL_TILT_MAX   = 0.26f
.set TILT_SLEW_RADPS = 1.0f
.set TAU_ROLL       = 0.10f       ; = tau_pitch
.set TAU_YAW        = 0.25f
.set TAU_ROLLRATE   = 0.025f      ; = tau_pitchRate
.set TAU_YAWRATE    = 0.05f
.set MASS           = 0.06f       ; kg (controller_pid.cpp default PID_MASS_KG is 0.032f)
.set J0             = 16e-6f
.set J1             = 16e-6f
.set J2             = 29e-6f
.set L_ARM          = 33e-3f
.set K_DRAG         = 0.01f
.set MOTOR_SCALE    = 0.9f
.set MOTOR_TRIM_LEFT = 0.87f
.set DUTY_OFFSET    = 0.583f
.set THRUST_A       = 26.919633f
.set THRUST_B       = 35.754861f
.set PROP_GAIN      = 1.33f
.set GRAMS_PER_NEWTON = 101.9368f
.set MOTORS         = 4.0f
"""

# register -> list of (name, expr, fmt).  First entry is the primary name; later entries
# are extra meanings of the same 48-bit pattern (the assembler checks the bits agree).
CONSTS = {
    0: [("ONE", "1.0f", "q24"), ("LEAD_H", "LEAD", "q24"), ("TILT_SLEW", "TILT_SLEW_RADPS", "q24"),
        ("DUTY_MAX", "1.0f", "q24"), ("COS_C0", "1.0f", "q24")],
    1: [("TWO", "2.0f", "q24"), ("INV_VEL_TC", "1.0f/VEL_TC", "q24")],
    2: [("G", "GRAVITY", "q24")],
    3: [("IMU_RY", "IMU_OFFSET_Y", "q24")],
    4: [("HALF", "0.5f", "q24"), ("KP_GAIN", "KP", "q24"), ("Z_GAIN_K", "Z_GAIN", "q24"),
        ("LEAD_A", "LEAD_ATT", "q24"), ("COS_C2", "0.5f", "q24")],
    5: [("NHALF", "-0.5f", "q24")],
    60: [("I24", "24", "int")],
    61: [("I6", "6", "int"), ("Q24_TO_Q30", "6", "int")],
    62: [("HALF_Q30", "0.5", "q30")],
    63: [("THREEHALF_Q30", "1.5", "q30"), ("ALT_INT_MAX_Q29", "ALT_INT_MAX", "q29")],
    64: [("TWO_Q30", "2.0", "q30"), ("VEL_INT_MAX_Q30", "VEL_INT_MAX", "q30")],
    65: [("I30", "30", "int")],
    66: [("I16", "16", "int")],
    67: [("ATAU", "IMU_ALPHA_TAU", "q24")],
    68: [("SHR1", "-1", "int"), ("Q25_TO_Q24", "-1", "int")],
    69: [("SHL1", "1", "int"), ("Q24_TO_Q25", "1", "int"), ("Q27_TO_Q28", "1", "int"),
         ("TRUE", "1", "int")],
    70: [("RSQ_SEED_A", "0.14", "q30")],
    71: [("RSQ_SEED_B", "1.05", "q30")],
    72: [("GATE_SLOPE", "1.0f/(0.5f*GRAVITY)", "q24")],
    73: [("ZERO", "0", "q24")],
    74: [("Q30_TO_Q24", "-6", "int")],
    75: [("TOF_COS_MIN", "TOF_MIN_COS", "q24"), ("ZUPT_H", "ZUPT_HEIGHT_M", "q24"),
         ("ALT_INT_GROUND", "ALT_INT_GROUND_M", "q24")],
    76: [("Q24_TO_Q28", "4", "int")],
    77: [("VMAX_Q28", "VMAX", "q28")],
    78: [("Q24_TO_Q27", "3", "int")],
    79: [("VZ_GAIN_K", "VZ_GAIN", "q24")],
    80: [("Q28_TO_Q24", "-4", "int")],
    81: [("Q27_TO_Q24", "-3", "int")],
    82: [("KI_H", "KI_HEIGHT", "q24")],
    83: [("Q24_TO_Q29", "5", "int")],
    84: [("Q29_TO_Q24", "-5", "int")],
    85: [("NF2", "NAT_FREQ_H*NAT_FREQ_H", "q24"), ("FOUR", "4.0f", "q24")],
    86: [("NEG_2DN", "-2.0f*DAMP_H*NAT_FREQ_H", "q24")],
    87: [("COS_C4", "1.0f/24.0f", "q24")],
    88: [("KI_V", "KI_VEL", "q24")],
    89: [("INV_G", "1.0f/GRAVITY", "q24")],
    90: [("TILT_MAX", "VEL_TILT_MAX", "q24")],
    91: [("Q31_TO_Q24", "-7", "int")],
    92: [("Q24_TO_Q31", "7", "int")],
    93: [("NEG_INV_TR", "-1.0f/TAU_ROLL", "q24")],
    94: [("NEG_INV_TY", "-1.0f/TAU_YAW", "q24")],
    95: [("NEG_INV_TRR", "-1.0f/TAU_ROLLRATE", "q24")],
    96: [("NEG_INV_TYR", "-1.0f/TAU_YAWRATE", "q24")],
    97: [("MASS_K", "MASS", "q24")],
    98: [("J0_K", "J0", "q24"), ("J1_K", "J1", "q24")],
    99: [("J2_K", "J2", "q24")],
    100: [("MIX_L", "0.25/L_ARM", "q24")],
    101: [("MIX_K", "0.25/K_DRAG", "q24")],
    102: [("MIX_T", "0.25f", "q24")],
    103: [("MSCALE", "MOTOR_SCALE", "q24")],
    104: [("TRIM_L", "MOTOR_TRIM_LEFT", "q24")],
    105: [("DUTY_OFS", "DUTY_OFFSET", "q24")],
    106: [("MOT_PROP", "MOTORS/PROP_GAIN", "q24")],
    107: [("GPN", "GRAMS_PER_NEWTON", "q24")],
    108: [("TA", "THRUST_A", "q24")],
    109: [("TB", "THRUST_B", "q24")],
    110: [("INV_2TA", "1.0f/(2.0f*THRUST_A)", "q24")],
}
CONST_COMMENT = {
    0: "1.0; also LEAD x dt, slew rate x dt, duty ceiling, cos-poly 1",
    1: "2.0; also 1/VEL_TC",
    4: "0.5; also KP, Z_GAIN, LEAD_ATT, cos-poly x^2/2",
    60: "NR: Q24 binary point (msb - 24 = exponent)",
    61: "NR: 30-24; also Q24->Q30 shift",
    63: "NR 1.5; same bits = 3.0 in Q29 (alt_int clamp)",
    64: "NR 2.0; same bits = VEL_INT_MAX in Q30",
    68: "VSH -1: NR exponent halving; Q25->Q24",
    69: "VSH +1: NR re-double; Q24->Q25, Q27->Q28; flag 1",
    70: "rsqrt seed y0 = 1.05 - 0.14*x on x in [1,4) (Q30)",
    73: "0 (selects, clamps)",
    75: "0.05: ToF tilt gate, ZUPT height, ground band",
    85: "4.0: NF^2 and the 4 in 4*TA",
    98: "J0 == J1",
}

INPUTS = [("AX", 6), ("AY", 7), ("AZ", 8), ("GX", 9), ("GY", 10), ("GZ", 11), ("F0", 12),
          ("F1", 13), ("H", 14), ("DT", 15), ("FV", 16), ("TV", 17), ("DESH", 18),
          ("DESV1", 19), ("DESV2", 20), ("YAWT", 21)]
INPUT_COMMENT = {"AX": "accel (m/s^2, Q24)", "GX": "gyro (rad/s, Q24)", "F0": "flow velocity (m/s, Q24)",
                 "H": "down-ToF slant range (m, Q24)", "DT": "step (s, Q24)", "FV": "flow valid (0/1)",
                 "TV": "ToF valid (0/1)", "DESH": "controller: height setpoint (m)",
                 "DESV1": "controller: vx setpoint", "DESV2": "controller: vy setpoint",
                 "YAWT": "controller: yaw target"}
STATES = [("Q_W", 22, "1.0", "q30"), ("Q_X", 23, "0", "q30"), ("Q_Y", 24, "0", "q30"),
          ("Q_Z", 25, "0", "q30"), ("POS_X", 26, "0", "q27"), ("POS_Y", 27, "0", "q27"),
          ("POS_Z", 28, "0", "q27"), ("VEL_X", 29, "0", "q28"), ("VEL_Y", 30, "0", "q28"),
          ("VEL_Z", 31, "0", "q28"), ("ALPHA_X", 32, "0", "q25"), ("WPREV_X", 33, "0", "q25"),
          ("ALPHA_Y", 34, "0", "q25"), ("WPREV_Y", 35, "0", "q25"), ("ALPHA_Z", 36, "0", "q25"),
          ("WPREV_Z", 37, "0", "q25"), ("HAVE_WPREV", 38, "0", "int"), ("ALT_INT", 39, "0", "q29"),
          ("VEL_INT_1", 40, "0", "q30"), ("VEL_INT_2", 41, "0", "q30"),
          ("ROLL_PREV", 42, "0", "q31"), ("PITCH_PREV", 43, "0", "q31")]
SOUTS = [("EST_X", 44), ("EST_Y", 45), ("EST_Z", 46), ("EST_R1", 47), ("EST_R2", 48),
         ("EST_R3", 49), ("EST_VX", 50), ("EST_VY", 51), ("EST_VZ", 52), ("EST_WX", 53),
         ("EST_WY", 54), ("EST_WZ", 55)]
OUTPUTS = [("U0", 56), ("U1", 57), ("U2", 58), ("U3", 59)]

FIXED = {}
for r, lst in CONSTS.items():
    FIXED[r] = lst[0][0]
for n, r in INPUTS + SOUTS + OUTPUTS:
    FIXED[r] = n
for n, r, _, _ in STATES:
    FIXED[r] = n

RECIP = {0: ("inv_dt", 16), 25: ("inv_tau", 24), 468: ("inv_pw", 24), 570: ("inv_crcp", 24)}
RSQRT = {102: "inv_amag", 234: "inv_qn", 423: "inv_pn", 699: "rs", 754: "rs", 810: "rs", 866: "rs"}
NR_PC = set()
for s in RECIP:
    NR_PC.update(range(s, s + 24))
for s in RSQRT:
    NR_PC.update(range(s, s + 41))


def nr_name(pc):
    for s, (out, _) in RECIP.items():
        if s <= pc < s + 24:
            k = pc - s
            if k == 23:
                return out
            return {0: "nr_m", 1: "nr_e", 2: "nr_sh", 3: "nr_x", 4: "nr_t", 5: "nr_y",
                    21: "nr_t", 22: "nr_sh"}.get(k) or ("nr_t", "nr_c", "nr_y")[(k - 6) % 3]
    for s, out in RSQRT.items():
        if s <= pc < s + 41:
            k = pc - s
            if k == 40:
                return out
            return {0: "nr_m", 1: "nr_e", 2: "nr_h", 3: "nr_e2", 4: "nr_sh", 5: "nr_x",
                    6: "nr_t", 7: "nr_y", 38: "nr_t", 39: "nr_sh"}.get(k) or \
                ("nr_y2", "nr_t", "nr_t", "nr_c", "nr_y")[(k - 8) % 5]
    return None


# value names for every non-NR scratch definition:  "pc name"
NAMES_TXT = """
24 tau_dt
49 k_alpha
50 wp_x 51 dw_x 52 deriv_x 53 al_x 54 t 55 t 56 t 57 al_x_upd 58 alpha_x_new 59 wprev_x_new
60 wp_y 61 dw_y 62 deriv_y 63 al_y 64 t 65 t 66 t 67 al_y_upd 68 alpha_y_new 69 wprev_y_new
70 wp_z 71 dw_z 72 deriv_z 73 al_z 74 t 75 t 76 t 77 al_z_upd 78 alpha_z_new 79 wprev_z_new
80 alx 81 alz 82 gxgy 83 t 84 t 85 lev_x 86 gxgx 87 gzgz 88 t 89 t 90 lev_y 91 gygz 92 t 93 lev_z
94 acx 95 acy 96 acz
97 acz2 98 acy2 99 acx2 100 t 101 a2
143 amag
144 dead_d 145 dead_nd 146 dead_lt 147 dead_d2 148 dead_abs
149 g_m_a 150 a_m_g 151 amag_err 152 t 153 t 154 gate 155 kgate
156 anx 157 any 158 anz
159 qw 160 qx 161 qy 162 qz
163 qwqy 164 qxqz 165 t 166 vux 167 qwqx 168 qyqz 169 t 170 vuy 171 qyqy 172 qxqx 173 t 174 t 175 vuz
176 az_vuy 177 ay_vuz 178 ex 179 ax_vuz 180 az_vux 181 ey 182 ay_vux 183 ax_vuy 184 ez
185 t 186 wx 187 t 188 wy 189 t 190 wz
191 qz_wz 192 qy_wy 193 qx_wx 194 t 195 t 196 dqw
197 qy_wz 198 qz_wy 199 qw_wx 200 t 201 t 202 dqx
203 qz_wx 204 qx_wz 205 qw_wy 206 t 207 t 208 dqy
209 qx_wy 210 qy_wx 211 qw_wz 212 t 213 t 214 dqz
215 t 216 qiw 217 t 218 qix 219 t 220 qiy 221 t 222 qiz
223 qiw24 224 qix24 225 qiy24 226 qiz24
227 qiz2 228 qiy2 229 s_zy 230 qix2 231 qiw2 232 s_wx 233 qn2
275 qnw 276 qnx 277 qny 278 qnz
279 qw 280 qx 281 qy 282 qz
283 pa 284 pb 285 t 286 t 287 R0
288 pa 289 pb 290 t 291 R1
292 pa 293 pb 294 t 295 R2
296 pa 297 pb 298 t 299 R3
300 pa 301 pb 302 t 303 t 304 R4
305 pa 306 pb 307 t 308 R5
309 pa 310 pb 311 t 312 R6
313 pa 314 pb 315 t 316 R7
317 pa 318 pb 319 t 320 t 321 R8
322 tilt_ok 323 tof_vert 324 h_vert
325 R2_az 326 R1_ay 327 R0_ax 328 t 329 awx
330 R5_az 331 R4_ay 332 R3_ax 333 t 334 awy
335 R8_az 336 R7_ay 337 R6_ax 338 t 339 t 340 awz
341 t 342 vx_p 343 t 344 vy_p 345 t 346 vz_p
347 R1_f1 348 R0_f0 349 t 350 vfx 351 R4_f1 352 R3_f0 353 t 354 vfy
355 hv_low 356 zupt 357 vx_z 358 vy_z 359 vx_f 360 vy_f
361 nvmax 362 t 363 vx_c 364 nvmax 365 t 366 vy_c
367 t 368 px_p 369 t 370 py_p 371 t 372 pz_p
373 h_q27 374 rz 375 dz 376 rz_q28 377 dvz 378 dz_g 379 pz_new 380 dvz_g 381 vz_new
382 h_lead 383 ha
384 qz_gz 385 qy_gy 386 qx_gx 387 t 388 t 389 dpw
390 qy_gz 391 qz_gy 392 qw_gx 393 t 394 t 395 dpx
396 qz_gx 397 qx_gz 398 qw_gy 399 t 400 t 401 dpy
402 qx_gy 403 qy_gx 404 qw_gz 405 t 406 t 407 dpz
408 t 409 pw 410 t 411 px 412 t 413 py 414 t 415 pz
416 pzpz 417 pypy 418 s1 419 pxpx 420 pwpw 421 s2 422 pn2
464 pnw 465 pnx 466 pny 467 pnz
492 rod1 493 rod2 494 rod3
495 t 496 out_px 497 t 498 out_py 499 t 500 out_pz
501 t 502 out_vx 503 t 504 out_vy 505 t 506 out_vz
537 vx 538 vy 539 vz 540 z
541 ground
542 h_err 543 t 544 t 545 t 546 ai 547 nmax 548 t 549 ai 550 alt_int_new 551 ai24
552 dz_err 553 t1 554 t2 555 t 556 acc_z
557 x2 558 x4 559 c4 560 c2 561 t 562 cr
563 x2 564 x4 565 c4 566 c2 567 t 568 cp
569 crcp
594 t 595 des_norm
596 verr1 597 verr2
598 t 599 t 600 t 601 vi1 602 t 603 t 604 t 605 vi2
606 nmax 607 t 608 vi1 609 nmax 610 t 611 vi2
612 vel_int1_new 613 vel_int2_new
614 vi1_24 615 t 616 acc1 617 vi2_24 618 t 619 acc2
620 nacc2 621 des_roll 622 des_pitch
623 ntilt 624 t 625 des_roll 626 ntilt 627 t 628 des_pitch
629 dmax
630 t 631 roll_prev 632 t 633 pitch_prev
634 hi 635 lo 636 t 637 des_roll 638 hi 639 lo 640 t 641 des_pitch
642 roll_prev_new 643 pitch_prev_new
644 t 645 rate_r 646 t 647 rate_p 648 t 649 rate_y
650 t 651 cmd_r 652 t 653 cmd_p 654 t 655 cmd_y
656 u_thr 657 u_roll 658 u_pitch 659 u_yaw
660 nmix_l 661 nmix_k
662 t3 663 t2 664 s 665 t1 666 t0 667 s2 668 ctrl0
669 t3 670 t2 671 s 672 t1 673 t0 674 s2 675 ctrl1
676 t3 677 t2 678 s 679 t1 680 t0 681 s2 682 ctrl2
683 t3 684 t2 685 s 686 t1 687 t0 688 s2 689 ctrl3
690 f 691 fin 692 t 693 thr 694 t 695 t 696 tb2 697 t 698 disc
740 sq 741 t 742 d 743 t 744 duty0
745 f 746 fin 747 t 748 thr 749 t 750 t 751 tb2 752 t 753 disc
795 sq 796 t 797 d 798 t 799 duty1
800 f 801 f 802 fin 803 t 804 thr 805 t 806 t 807 tb2 808 t 809 disc
851 sq 852 t 853 d 854 t 855 duty2
856 f 857 f 858 fin 859 t 860 thr 861 t 862 t 863 tb2 864 t 865 disc
907 sq 908 t 909 d 910 t 911 duty3
912 out0 913 out1 914 out2 915 out3
"""
NAMES = {}
_tok = NAMES_TXT.split()
for i in range(0, len(_tok), 2):
    NAMES[int(_tok[i])] = _tok[i + 1]

# const operands whose meaning differs from the register's primary name: {pc: {reg: name}}
SITE = {382: {0: "LEAD_H"}, 629: {0: "TILT_SLEW"}, 561: {0: "COS_C0"}, 567: {0: "COS_C0"},
        615: {1: "INV_VEL_TC"}, 618: {1: "INV_VEL_TC"},
        155: {4: "KP_GAIN"}, 375: {4: "Z_GAIN_K"}, 383: {4: "LEAD_A"}, 560: {4: "COS_C2"}, 566: {4: "COS_C2"},
        600: {61: "Q24_TO_Q30"}, 604: {61: "Q24_TO_Q30"},
        547: {63: "ALT_INT_MAX_Q29"}, 548: {63: "ALT_INT_MAX_Q29"},
        376: {69: "Q27_TO_Q28"}, 513: {69: "TRUE"},
        322: {75: "TOF_COS_MIN"}, 355: {75: "ZUPT_H"}, 541: {75: "ALT_INT_GROUND"},
        553: {85: "NF2"}, 658: {98: "J1_K"}}
for pc in (743, 798, 854, 910):
    SITE[pc] = {0: "DUTY_MAX"}
for pc in (606, 607, 609, 610):
    SITE[pc] = {64: "VEL_INT_MAX_Q30"}
for pc in (694, 749, 805, 861):
    SITE[pc] = {85: "FOUR"}
DATA_NAME = {68: "Q25_TO_Q24", 69: "Q24_TO_Q25", 75: "TOF_COS_MIN"}  # outside NR blocks

SECTIONS = {
    0: ("EST 1. reciprocal 1/dt  (Newton-Raphson, 5 iterations, result Q16)", None),
    24: ("EST 1. lever-arm compensation: k = dt/(IMU_ALPHA_TAU+dt)  (NR reciprocal, Q24)", None),
    50: ("EST 1. angular-accel LP  alpha += k*((g - w_prev)/dt - alpha); w_prev = g  (state Q25)", None),
    80: ("EST 1. a = accel - (alpha x r + w x (w x r)),  r = (0, IMU_OFFSET_Y, 0)", None),
    97: ("EST 2. Mahony trim: |a| = a2 * rsqrt(a2)  (rsqrt: NR, 6 iterations)", None),
    144: ("EST 2. gate = max(0, 1 - |(|a| - g)| / (g/2)),  k = KP*gate", None),
    156: ("EST 2. a_hat = a/|a|;  vu = R^T (0,0,1) from q;  e = a_hat x vu;  w = gyro + k*e", None),
    191: ("EST 3. quaternion integrate  dq = 1/2 q (x) (0,w);  q += dq*dt  (q in Q30)", None),
    223: ("EST 3. normalise q /= |q|  (rsqrt)", None),
    279: ("EST 4. rotation matrix R(q), row-major R0..R8", None),
    322: ("EST 5. ToF tilt gate + vertical height;  world accel a_w = R a - g", None),
    341: ("EST 5. velocity integrate v += a_w*dt  (vel Q28)", None),
    347: ("EST 6. flow fuse v_xy = R[0..1, 3..4] * flow  /  ZUPT on the ground  / clamp |v| <= VMAX", None),
    367: ("EST 6. position integrate p += v*dt  (pos Q27)", None),
    373: ("EST 6. ToF altitude observer  z += Z_GAIN*rz,  vz += VZ_GAIN*rz  (when tof_vert)", None),
    382: ("EST 7. get_state lead: quaternion predicted by LEAD_ATT*dt with the raw gyro", None),
    416: ("EST 7. normalise the lead quaternion (rsqrt)", None),
    468: ("EST 7. Rodrigues r = q_xyz / q_w  (NR reciprocal of q_w)", None),
    495: ("EST 7. translation lead  p + v*h,  v + a_w*h  (h = LEAD*dt)", None),
    507: ("EST    commit state + state-out", None),
    537: ("CTRL 8. altitude loop: alt_int += KI_H*(desH - z)*dt (Q29, clamp, ground reset)", None),
    552: ("CTRL 8. desAcc = -2*DH*NF*vz - NF^2*(z - desH) + alt_int", None),
    557: ("CTRL 8. cos(r1), cos(r2) by 1 - x^2/2 + x^4/24;  desNorm = (g + desAcc)/(cr*cp)", None),
    596: ("CTRL 9. horizontal velocity PI: vel_int += KI_V*e*dt (Q30, clamp, ground reset)", None),
    614: ("CTRL 9. desAcc = e/VEL_TC + vel_int;  desRoll = -dA2/g, desPitch = dA1/g, clamp", None),
    629: ("CTRL 9. tilt slew limit |d(des)/dt| <= TILT_SLEW_RADPS (prev in Q31, ground reset)", None),
    644: ("CTRL 10. attitude loop rate_tgt = (est - tgt)*(-1/tau);  rate loop cmd = (gyro - rate_tgt)*(-1/tau_r)", None),
    656: ("CTRL 11. u = [desNorm*MASS, cmd_r*J0, cmd_p*J1, cmd_y*J2];  ctrl = M u  (4x4 mixer)", None),
    690: ("CTRL 12. motor 0 = force->duty(0.9*ctrl1):  t = max(f,0)*GPN*MOT/PROP;  disc = 4*TA*t + TB^2;"
          "  d = (sqrt(disc) - TB)/(2*TA), clamp [0,1]", None),
    745: ("CTRL 12. motor 1 = force->duty(0.9*ctrl2)", None),
    800: ("CTRL 12. motor 2 = force->duty(0.9*ctrl3*TRIM_L)", None),
    856: ("CTRL 12. motor 3 = force->duty(0.9*ctrl0*TRIM_L)", None),
    912: ("CTRL    u_out = duty - 0.583;  commit state + outputs", None),
}
COMMENTS = {
    144: "dead: |a|-g via SEL, overwritten unread (pc 148)", 149: "|a|-g recomputed as max(a-b, b-a)",
    323: "tof_vert = TV && tilt_ok (mul.0 = AND of 0/1)", 356: "zupt = tof_vert && h_vert < ZUPT",
    361: "-VMAX (recomputed below)", 364: "-VMAX again (redundant)", 696: "TB^2 at run time (per motor)",
    694: "4*TA at run time (per motor)", 58: "first step: keep alpha (no w_prev yet)",
}


def srcs(u):
    o = u["op"]
    if o == 13:
        return []
    if o in (6, 7, 10):
        return [u["fa"]]
    if o == 12:
        return [u["fa"], u["fb"], u["fc"]]
    return [u["fa"], u["fb"]]


def reg_name(r, pc):
    if pc in SITE and r in SITE[pc]:
        return SITE[pc][r]
    if pc not in NR_PC and r in DATA_NAME:
        return DATA_NAME[r]
    return FIXED.get(r)


def fmt_ins(u, names):
    """names(r) -> operand text."""
    o = OPS[u["op"]]
    if o == "HALT":
        return "halt", ""
    m = "mul.%d" % u["sh"] if o == "MUL" else o.lower()
    d = names("d", u["fd"])
    if o == "SEL":
        ops = [d, names("s", u["fc"]), names("s", u["fa"]), names("s", u["fb"])]
    elif o in ("MOV", "NEG", "MSB"):
        ops = [d, names("s", u["fa"])]
    else:
        ops = [d, names("s", u["fa"]), names("s", u["fb"])]
    return m, ", ".join(ops)


def listing(rom):
    out = []
    for pc, w in enumerate(rom):
        if pc == EST:
            out.append("; ===== EST program (start %d) =====" % EST)
        if pc == CTRL:
            out.append("; ===== CTRL program (start %d) =====" % CTRL)
        if pc in SECTIONS:
            out.append(";  -- " + SECTIONS[pc][0])
        u = decode(w)
        m, ops = fmt_ins(u, lambda k, r, pc=pc: (reg_name(r, pc) or "r%d" % r) + ("" if (reg_name(r, pc) is None) else "(r%d)" % r))
        out.append("%4d  %016x  %-7s %s" % (pc, w, m, ops))
    return "\n".join(out)


def init_table(init):
    rows = []
    for r in range(256):
        v = s48(init[r])
        lab = FIXED.get(r)
        if lab is None and v == 0:
            continue
        fmt = "q24"
        if r in CONSTS:
            fmt = CONSTS[r][0][2]
        elif any(s[1] == r for s in STATES):
            fmt = [s[3] for s in STATES if s[1] == r][0]
        if fmt == "int":
            val = "%d" % v
        else:
            val = "%.9g" % (v / 2.0 ** int(fmt[1:]))
        extra = ""
        if r in CONSTS:
            extra = "  = " + "; ".join("%s:%s" % (n, e) for n, e, _ in CONSTS[r])
        rows.append("r%-3d %-16s %012x  %-4s %14s%s" % (r, lab or "-", init[r], fmt, val, extra))
    return "\n".join(rows)


def analyze(rom):
    us = [decode(w) for w in rom]
    lines = []
    for pc, u in enumerate(us):
        if u["op"] == 13:
            continue
        end = CTRL if pc < CTRL else len(us)
        r = u["fd"]
        read = False
        for q in range(pc + 1, end):
            if r in srcs(us[q]):
                read = True
                break
            if us[q]["op"] != 13 and us[q]["fd"] == r:
                break
        else:
            continue  # live-out (state / outputs)
        if not read:
            lines.append("dead write: pc %d writes r%d, overwritten before any read" % (pc, r))
    # repeated identical computations (same op/sh/operand regs, operands unchanged in between)
    for pc, u in enumerate(us):
        if u["op"] == 13:
            continue
        for q in range(pc + 1, min(len(us), pc + 40)):
            v = us[q]
            if (v["op"], v["sh"], v["fa"], v["fb"], v["fc"]) == (u["op"], u["sh"], u["fa"], u["fb"], u["fc"]):
                clob = any(us[k]["op"] != 13 and us[k]["fd"] in srcs(u) for k in range(pc, q))
                if not clob and (pc < CTRL) == (q < CTRL):
                    lines.append("recomputed: pc %d repeats pc %d (%s)" % (q, pc, OPS[u["op"]]))
                break
    lines.append("recip blocks (24 words, 5 NR iterations): " + ", ".join("%d..%d" % (s, s + 23) for s in RECIP))
    lines.append("rsqrt blocks (41 words, 6 NR iterations): " + ", ".join("%d..%d" % (s, s + 40) for s in RSQRT))
    return "\n".join(lines)


def recover(rom, init, path):
    us = [decode(w) for w in rom]
    out = []
    w = out.append
    w("; fc_rocc.fca -- FcRoCC flight-controller microcode (estimator + hierarchical PID), Q24.24.")
    w(";")
    w("; Assemble:  fc_rocc_asm -o build fc_rocc.fca   (see README in this directory)")
    w("; Recovered 2026-11 from the checked-in vsrc/fc_rocc_rom.hex + fc_rocc_init.hex; assembling")
    w("; this file reproduces both, and workloads/flight_controller/src/fc_rocc_fw.h, byte for byte.")
    w(";")
    w("; Fixed-point formats: Q24 by default.  q (Q30), pos (Q27), vel (Q28), alpha/w_prev (Q25),")
    w("; alt_int (Q29), vel_int (Q30) and the slew 'prev' tilt (Q31) are kept wider for drift.")
    w("; mul.S d,a,b = sat((a*b) >> S): Q24*Q24 -> mul.24, Q30*Q30 -> mul.30, Q24*Q24 -> Q28 is mul.20 ...")
    w("; vsh d,a,K shifts by the signed integer in register K (the Qx_TO_Qy constants below).")
    w(";")
    w("; Value names: `name:rN` binds a name to the register it is written into; later operands")
    w("; use the name.  The assembler rejects a name whose register has since been overwritten, so")
    w("; renaming or reordering cannot silently read the wrong value.  New code can take a fresh")
    w("; register with `name:new`, and an inline constant with `=EXPR` (e.g. `=0.9f`, `=1.5@q30`).")
    w(";")
    w("; NR blocks: reciprocal 1/x (24 words) normalises x to [1,2) in Q30 by its MSB, seeds")
    w("; y0 = 1.5 - x/2, iterates y = y*(2 - x*y) 5x, then shifts back (result Q24, or Q16 for 1/dt).")
    w("; rsqrt (41 words) normalises by an even shift to [1,4), seeds y0 = 1.05 - 0.14x, iterates")
    w("; y = y*(1.5 - x*y^2/2) 6x.  Neither guards x <= 0 (the C code's 1e-9 / 1e-3 checks are absent).")
    w("")
    w(SETS)
    w("; ---- firmware map exports (fc_rocc_fw.h) ----")
    w(".export QA 24")
    w(".export Q_POS 27")
    w(".export Q_VEL 28")
    w("")
    w("; ---- constants (boot image).  Several registers carry more than one meaning; each extra")
    w("; name re-derives the same bits from its own parameter and the assembler checks they agree.")
    for r in sorted(CONSTS):
        c = CONST_COMMENT.get(r)
        for i, (n, e, f) in enumerate(CONSTS[r]):
            tail = (" " + f) if f != "q24" else ""
            txt = ".const %-16s r%-3d = %s%s" % (n, r, e, tail)
            if i == 0 and c:
                txt = "%-58s ; %s" % (txt, c)
            w(txt)
    w("")
    w("; ---- inputs (PUSH, Q24; flags 0/1) ----")
    for n, r in INPUTS:
        c = INPUT_COMMENT.get(n)
        w(("%-26s ; %s" % (".input %-6s r%d" % (n, r), c)) if c else ".input %-6s r%d" % (n, r))
    w("")
    w("; ---- persistent state (boot values) ----")
    for n, r, e, f in STATES:
        w(".state %-12s r%-3d = %s %s" % (n, r, e, f))
    w("")
    w("; ---- estimator state-out (POP; EST_X..Z Q27, EST_R* Q24 Rodrigues, EST_V* Q28, EST_W* Q24) ----")
    for n, r in SOUTS:
        w(".sout %-7s r%d" % (n, r))
    w("")
    w("; ---- controller outputs (POP; duty - 0.583, Q24) ----")
    for n, r in OUTPUTS:
        w(".output %s r%d" % (n, r))

    cur = {}  # reg -> current value name (scratch)
    for pc, u in enumerate(us):
        if pc == EST:
            w("")
            w("; " + "=" * 92)
            w(".program EST")
        if pc == CTRL:
            w("")
            w("; " + "=" * 92)
            w(".program CTRL")
        if pc in SECTIONS:
            w("")
            w("; ---- " + SECTIONS[pc][0])

        def names(kind, r, pc=pc, u=u):
            fixed = reg_name(r, pc)
            if fixed is not None:
                return fixed
            if kind == "s":
                if r not in cur:
                    raise SystemExit("pc %d reads r%d with no name" % (pc, r))
                return cur[r]
            nm = NAMES.get(pc) or nr_name(pc)
            if nm is None:
                raise SystemExit("pc %d: no name for scratch dest r%d" % (pc, r))
            return "%s:r%d" % (nm, r)

        m, ops = fmt_ins(u, names)
        line = "    %-8s %s" % (m, ops)
        if pc in COMMENTS:
            line = "%-52s ; %s" % (line, COMMENTS[pc])
        w(line.rstrip())
        if u["op"] != 13 and u["fd"] not in FIXED:
            cur[u["fd"]] = NAMES.get(pc) or nr_name(pc)
    open(path, "w").write("\n".join(out) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rom", default=ROM)
    ap.add_argument("--init-hex", default=INIT)
    ap.add_argument("--init", action="store_true")
    ap.add_argument("--analyze", action="store_true")
    ap.add_argument("--recover")
    a = ap.parse_args()
    rom, init = load_hex(a.rom), load_hex(a.init_hex)
    if a.init:
        print(init_table(init))
    elif a.analyze:
        print(analyze(rom))
    elif a.recover:
        recover(rom, init, a.recover)
    else:
        print(listing(rom))


if __name__ == "__main__":
    main()
