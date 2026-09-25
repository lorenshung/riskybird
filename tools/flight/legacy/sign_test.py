#!/usr/bin/env python3
"""
Actuation sign test: does the vehicle move the way the controller commands?

    python3 tools/sign_test.py trials/<trial> [trials/<trial> ...]

For each ARMED telemetry line, decompose the FC's duty change from the first armed line
into roll / pitch / yaw moment commands using controller_pid.cpp's mixer as seen at the
duty outputs:
    roll  -> duty [-,-,+,+]     pitch -> [-,+,+,-]     yaw -> [-,+,-,+]
(derived from M[][] and duty[0]=ctrl[1], duty[1]=ctrl[2], duty[2]=ctrl[3]*.87,
duty[3]=ctrl[0]*.87), and compare each command's sign with the attitude change over the
next line (roll/pitch/yaw from the FC's Gibbs components, in degrees). A healthy loop
that is merely losing also commands AGAINST the motion, so read this with the magnitudes:
a large yaw command followed by yaw ACCELERATING the same way is only possible if the
yaw actuation is reversed -- nothing but the motors makes yaw torque.
"""
import csv, math, sys


def deg(g):
    return 2.0 * math.degrees(math.atan(float(g)))


def main(dirs):
    for d in dirs:
        rows = list(csv.DictReader(open(f"{d}/fc.csv")))
        armed = [x for x in rows if int(float(x["st"])) & 1 and not int(float(x["st"])) & 2]
        if len(armed) < 3:
            print(f"== {d}: fewer than 3 armed lines"); continue
        base = [float(armed[0][f"duty{i}"]) for i in range(4)]
        tally = {k: [0, 0] for k in ("roll", "pitch", "yaw")}
        print(f"== {d}  ({len(armed)} armed lines)")
        print("     t   roll  pitch    yaw |  c_roll c_pitch  c_yaw | d_roll d_pitch  d_yaw")
        for a, n in zip(armed, armed[1:]):
            du = [float(a[f"duty{i}"]) - base[i] for i in range(4)]
            c = {"roll": (-du[0] - du[1] + du[2] + du[3]) / 4,
                 "pitch": (-du[0] + du[1] + du[2] - du[3]) / 4,
                 "yaw": (-du[0] + du[1] - du[2] + du[3]) / 4}
            A = {k: deg(a[f"{k}_g"]) for k in c}
            N = {k: deg(n[f"{k}_g"]) for k in c}
            dd = {k: N[k] - A[k] for k in c}
            if abs(A["roll"]) < 80 and abs(A["pitch"]) < 80:
                for k in c:
                    if abs(c[k]) > 0.01 and abs(dd[k]) > 0.3:
                        tally[k][0 if c[k] * dd[k] > 0 else 1] += 1
            print(f"{float(a['t']):6.2f} {A['roll']:6.1f} {A['pitch']:6.1f} {A['yaw']:6.1f} | "
                  f"{c['roll']:+.3f}  {c['pitch']:+.3f} {c['yaw']:+.3f} | "
                  f"{dd['roll']:+6.1f} {dd['pitch']:+6.1f} {dd['yaw']:+6.1f}")
        print("   [moved WITH command, AGAINST command]:", tally)


if __name__ == "__main__":
    main(sys.argv[1:] or [])
