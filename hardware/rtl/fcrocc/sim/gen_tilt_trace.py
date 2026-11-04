#!/usr/bin/env python3
"""Synthesise a TILTED sensor trace in sim/sensors.csv format, for the flow body-z check.

The vehicle holds 1.0 m and flies horizontally while banked 15-25 deg along its direction of
motion (the drag-balanced attitude of steady flight), which is where the flow update's body-z term
(fc_flow_to_world(), FC_FLOW_BODYZ) is large: bz = (R^T v)_z ~ v*sin(tilt).

Kinematically exact, noise-free sensors:
  accel   specific force at the IMU, R^T (a_w + g z) + alpha x r + w x (w x r), r = (0, -0.016, 0)
  gyro    body rate w, from the quaternion derivative
  flow    body x/y velocity (R^T v)_xy -- what fc_flow_velocity() delivers (angular flow x slant)
  height  down-ToF SLANT range to the floor along body -z, z / R[8]
  fvalid  1 every step; tofvalid every 3rd step (as sim/sensors.csv)

Usage: gen_tilt_trace.py OUT.csv [--truth TRUTH.csv]
"""
import math
import sys

DT = 0.002
T_END = 12.0
G = 9.81
IMU_R = (0.0, -0.016, 0.0)
Z0 = 1.0
D = math.radians


def smooth(t, t0, t1, a, b):
    """C2 smoothstep from a (t <= t0) to b (t >= t1)."""
    if t <= t0:
        return a
    if t >= t1:
        return b
    s = (t - t0) / (t1 - t0)
    return a + (b - a) * s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)


def env(t, t0, t1):
    """1 on [t0+0.5, t1-0.5], smoothly to 0 at t0 and t1: keeps rates and accels continuous."""
    return smooth(t, t0, t0 + 0.5, 0.0, 1.0) * smooth(t, t1 - 0.5, t1, 1.0, 0.0)


def profile(t):
    """(vx, vy, roll, pitch, yaw) at time t. Positive pitch tilts body z toward +x.

    0-1 s hover level; 1-2 s pitch to +20 deg while accelerating to +1.5 m/s; 2-6 s cruise with a
    +-4 deg pitch and +-8 deg roll wobble, a 0.3 m/s y weave and a 0.3 rad yaw turn; 6-7 s reverse to
    -22 deg / -1.2 m/s; 7-10 s cruise with -10 deg roll and +0.4 m/s y; 10-11 s back to level, 11-12 s
    hover. Every segment joins with continuous rate and acceleration."""
    vx = smooth(t, 1.0, 2.0, 0.0, 1.5)
    if t > 6.0:
        vx = smooth(t, 6.0, 7.0, 1.5, -1.2)
    if t > 10.0:
        vx = smooth(t, 10.0, 11.0, -1.2, 0.0)
    vy = 0.3 * math.sin(2 * math.pi * 0.25 * (t - 2.0)) * env(t, 2.0, 6.0) if 2.0 < t < 6.0 else 0.0
    if t > 7.0:
        vy = smooth(t, 7.0, 7.5, 0.0, 0.4) if t <= 10.0 else smooth(t, 10.0, 11.0, 0.4, 0.0)

    pitch = smooth(t, 1.0, 2.0, 0.0, D(20.0))
    if 2.0 < t < 6.0:
        pitch += D(4.0) * math.sin(2 * math.pi * 0.5 * (t - 2.0)) * env(t, 2.0, 6.0)
    if t > 6.0:
        pitch = smooth(t, 6.0, 7.0, D(20.0), D(-22.0))
    if t > 10.0:
        pitch = smooth(t, 10.0, 11.0, D(-22.0), 0.0)

    roll = D(8.0) * math.sin(2 * math.pi * 0.3 * (t - 2.0)) * env(t, 2.0, 6.0) if 2.0 < t < 6.0 else 0.0
    if t > 7.0:
        roll = smooth(t, 7.0, 7.5, 0.0, D(-10.0)) if t <= 10.0 else smooth(t, 10.0, 11.0, D(-10.0), 0.0)

    yaw = smooth(t, 2.0, 6.0, 0.0, 0.3)
    if t > 7.0:
        yaw = smooth(t, 7.0, 10.0, 0.3, 0.1)
    return vx, vy, roll, pitch, yaw


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat(t):
    _, _, r, p, y = profile(t)
    qz = (math.cos(y / 2), 0.0, 0.0, math.sin(y / 2))
    qy = (math.cos(p / 2), 0.0, math.sin(p / 2), 0.0)
    qx = (math.cos(r / 2), math.sin(r / 2), 0.0, 0.0)
    return qmul(qz, qmul(qy, qx))      # body -> world, ZYX


def rot(q):
    w, x, y, z = q
    return [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
            2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
            2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]


def rate(t, h=1e-5):
    """body angular velocity w = 2 * conj(q) (x) dq/dt (vector part)"""
    qa, qb = quat(t - h), quat(t + h)
    q = quat(t)
    dq = tuple((b - a) / (2 * h) for a, b in zip(qa, qb))
    qc = (q[0], -q[1], -q[2], -q[3])
    m = qmul(qc, dq)
    return (2 * m[1], 2 * m[2], 2 * m[3])


def accel_world(t, h=1e-5):
    a, b = profile(t - h), profile(t + h)
    return ((b[0] - a[0]) / (2 * h), (b[1] - a[1]) / (2 * h), 0.0)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def mt_v(R, v):
    return [R[0] * v[0] + R[3] * v[1] + R[6] * v[2],
            R[1] * v[0] + R[4] * v[1] + R[7] * v[2],
            R[2] * v[0] + R[5] * v[1] + R[8] * v[2]]


def main():
    out = open(sys.argv[1], "w")
    truth = open(sys.argv[sys.argv.index("--truth") + 1], "w") if "--truth" in sys.argv else None
    out.write("dt,ax,ay,az,gx,gy,gz,fx,fy,fvalid,height,tofvalid,zsp\n")
    if truth:
        truth.write("step,t,vx,vy,vz,bx,by,bz,tilt_deg\n")
    n = int(round(T_END / DT))
    for k in range(n):
        t = k * DT
        vx, vy, _, _, _ = profile(t)
        R = rot(quat(t))
        w = rate(t)
        wa, wb = rate(t - 1e-4), rate(t + 1e-4)
        alpha = tuple((b - a) / 2e-4 for a, b in zip(wa, wb))
        aw = accel_world(t)
        f = mt_v(R, (aw[0], aw[1], aw[2] + G))
        lev = [p + q for p, q in zip(cross(alpha, IMU_R), cross(w, cross(w, IMU_R)))]
        f = [p + q for p, q in zip(f, lev)]
        b = mt_v(R, (vx, vy, 0.0))
        height = Z0 / R[8]
        tv = 1 if k % 3 == 0 else 0
        out.write("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,1,%.6f,%d,%.4f\n" %
                  (DT, f[0], f[1], f[2], w[0], w[1], w[2], b[0], b[1], height, tv, Z0))
        if truth:
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, R[8]))))
            truth.write("%d,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f\n" %
                        (k, t, vx, vy, 0.0, b[0], b[1], b[2], tilt))
    out.close()
    if truth:
        truth.close()


if __name__ == "__main__":
    main()
