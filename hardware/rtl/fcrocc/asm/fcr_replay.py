#!/usr/bin/env python3
"""Replay a sensor trace through the FcRoCC interpreter exactly as sim/tb_fc_rocc.sv does.

Usage: fcr_replay.py [--rom R] [--init I] [--map M] [--sensors S] [--out O]
Defaults use ../vsrc/*.hex and ../sim/sensors.csv; output defaults to stdout.
The program entry points and register map come from --map (an fc_rocc_map.svh), else from an
fc_rocc_map.svh next to --rom (as fc_rocc_asm writes them), else the checked-in values below.
A ROM assembled with options that move CTRL_START (e.g. -D FLOW_BODYZ=1) needs its own map.
"""
import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fcr_isa import Core, load_hex  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
# register map (fc_rocc_fw.h / fc_rocc_map.svh)
IN = dict(AX=6, AY=7, AZ=8, GX=9, GY=10, GZ=11, F0=12, F1=13, H=14, DT=15, FV=16, TV=17,
          DESH=18, DESV1=19, DESV2=20, YAWT=21)
U = [56, 57, 58, 59]
SOUT = list(range(44, 56))
EST_START, CTRL_START = 0, 537
Q_POS, Q_ATT, Q_VEL, Q_U = 27, 24, 28, 24


def toq(v, q):
    """tb toq(): $rtoi(v*2^q + (v>=0 ? 0.5 : -0.5)) -- round half away from zero."""
    s = v * (2.0 ** q)
    return int(s + (0.5 if v >= 0.0 else -0.5))   # int() truncates toward zero like $rtoi


def load_map(path):
    """Read the localparams of an fc_rocc_map.svh into the module-level map."""
    global EST_START, CTRL_START, U, SOUT
    p = dict((k, int(v)) for k, v in re.findall(r"\b(\w+)\s*=\s*(\d+)", open(path).read()))
    EST_START, CTRL_START = p["EST_START"], p["CTRL_START"]
    for k in IN:
        IN[k] = p["IN_" + k]
    U = [p["U%d" % i] for i in range(4)]
    m = re.search(r"SOUT\s*\[[^]]*\]\s*=\s*'\{([^}]*)\}", open(path).read())
    SOUT = [int(x) for x in m.group(1).split(",")]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rom", default=os.path.join(HERE, "../vsrc/fc_rocc_rom.hex"))
    ap.add_argument("--init", default=os.path.join(HERE, "../vsrc/fc_rocc_init.hex"))
    ap.add_argument("--map", default=None)
    ap.add_argument("--sensors", default=os.path.join(HERE, "../sim/sensors.csv"))
    ap.add_argument("--out", default="-")
    a = ap.parse_args()
    mp = a.map or os.path.join(os.path.dirname(os.path.abspath(a.rom)), "fc_rocc_map.svh")
    if a.map or os.path.exists(mp):
        load_map(mp)

    core = Core(load_hex(a.rom), load_hex(a.init))
    out = sys.stdout if a.out == "-" else open(a.out, "w")
    out.write("# impl=fcr_replay.py interpreter (assembled ROM)\n")
    out.write("step,ex,ey,ez,er1,er2,er3,evx,evy,evz,u0,u1,u2,u3\n")
    lines = open(a.sensors).read().splitlines()[1:]
    step = 0
    ec = cc = 0
    for ln in lines:
        c = ln.split(",")
        if len(c) != 13:
            continue
        dt, ax, ay, az, gx, gy, gz, fx, fy, fv, hh, tv, zsp = map(float, c)
        for k, v in (("AX", ax), ("AY", ay), ("AZ", az), ("GX", gx), ("GY", gy), ("GZ", gz),
                     ("F0", fx), ("F1", fy), ("H", hh), ("DT", dt)):
            core.write(IN[k], toq(v, Q_ATT))
        core.write(IN["FV"], 1 if fv > 0.5 else 0)
        core.write(IN["TV"], 1 if tv > 0.5 else 0)
        c0 = core.cycles
        core.run(EST_START)
        ec = core.cycles - c0
        core.write(IN["DESH"], toq(zsp, Q_ATT))
        core.write(IN["DESV1"], 0)
        core.write(IN["DESV2"], 0)
        core.write(IN["YAWT"], 0)
        core.write(IN["DT"], toq(dt, Q_ATT))
        c0 = core.cycles
        core.run(CTRL_START)
        cc = core.cycles - c0
        qs = [Q_POS] * 3 + [Q_ATT] * 3 + [Q_VEL] * 3
        vals = [core.read(r) / 2.0 ** q for r, q in zip(SOUT[:9], qs)]
        vals += [core.read(r) / 2.0 ** Q_U for r in U]
        out.write("%d," % step + ",".join("%.5f" % v for v in vals) + "\n")
        step += 1
    sys.stderr.write("replay: %d steps, est %d cycles/run, ctrl %d cycles/run\n" % (step, ec, cc))


if __name__ == "__main__":
    main()
