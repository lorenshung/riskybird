#!/bin/sh
# check_flow_bodyz.sh -- check the EST microcode's flow update against the C estimator, with and
# without the body-z term (fc_rocc.fca FLOW_BODYZ / fc_shared_math.h FC_FLOW_BODYZ).
#
#   sh check_flow_bodyz.sh [WORKDIR]      (default: a fresh mktemp -d; nothing is written here)
#
# Builds fc_rocc_asm and ref_complementary.cpp for both settings, synthesises the tilted trace
# (gen_tilt_trace.py), replays it through the assembled ROMs (fc_rocc_asm --replay) and the C,
# and prints max/rms differences of the velocity estimate for each pairing. Same-setting pairs
# should agree to ~1e-5 m/s (Q24.24 vs float); the cross pairs show the size of the term. The
# same-setting pairs are also run on sim/sensors.csv, whose on-ground start (no flow, ZUPT)
# covers the flow-invalid branches that the tilted trace never takes.
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
ASM=$HERE/../asm
FC=$HERE/../../../../workloads/flight_controller/src
W=${1:-$(mktemp -d)}
mkdir -p "$W"
cc -O2 -o "$W/fc_rocc_asm" "$ASM/fc_rocc_asm.c" -lm
for b in 0 1; do
    c++ -O2 -I"$FC" -DIMU_OFFSET_Y=-0.016f -DFC_FLOW_BODYZ=$b "$HERE/ref_complementary.cpp" \
        -o "$W/ref$b" -lm
done
python3 -I "$HERE/gen_tilt_trace.py" "$W/tilt.csv" --truth "$W/truth.csv"
for b in 0 1; do
    "$W/fc_rocc_asm" -o "$W/rom$b" -D FLOW_BODYZ=$b "$ASM/fc_rocc.fca" --replay "$W/tilt.csv" "$W/rom$b.csv"
    "$W/ref$b" "$W/tilt.csv" "$W/c$b.csv"
    "$W/fc_rocc_asm" -o "$W/rom$b" -D FLOW_BODYZ=$b "$ASM/fc_rocc.fca" --replay "$HERE/sensors.csv" "$W/rom${b}_ground.csv"
    "$W/ref$b" "$HERE/sensors.csv" "$W/c${b}_ground.csv"
done
python3 -I - "$W" <<'EOF'
import math, os, sys
w = sys.argv[1]
def load(name):
    rows, hdr = [], None
    for ln in open(os.path.join(w, name)):
        ln = ln.strip()
        if not ln or ln[0] == "#":
            continue
        if hdr is None:
            hdr = ln.split(",")
            continue
        rows.append(dict(zip(hdr, map(float, ln.split(",")))))
    return rows
tr = load("truth.csv")
tilted = [k for k, r in enumerate(tr) if 15.0 <= r["tilt_deg"] <= 25.5 and math.hypot(r["vx"], r["vy"]) > 0.8]
def stats(d):
    return max(abs(x) for x in d), math.sqrt(sum(x * x for x in d) / len(d))
def row(label, a, b, ks):
    s = []
    for sig in ("evx", "evy", "evz"):
        m, r = stats([b[k][sig] - a[k][sig] for k in ks])
        s.append("%s max %.2e rms %.2e" % (sig, m, r))
    print("  %-26s %s" % (label, "   ".join(s)))
R = {n: load(n + ".csv") for n in ("rom0", "rom1", "c0", "c1")}
for title, ks in (("all %d steps" % len(tr), range(len(tr))),
                  ("%d steps at 15-25 deg tilt, |v| > 0.8 m/s" % len(tilted), tilted)):
    print(title)
    row("ROM FLOW_BODYZ=0 vs C =0", R["c0"], R["rom0"], ks)
    row("ROM FLOW_BODYZ=1 vs C =1", R["c1"], R["rom1"], ks)
    row("ROM =0 vs C =1", R["c1"], R["rom0"], ks)
    row("C =0 vs C =1", R["c0"], R["c1"], ks)
G = {n: load(n + "_ground.csv") for n in ("rom0", "rom1", "c0", "c1")}
print("sim/sensors.csv, %d steps (ground start, flow invalid, ZUPT)" % len(G["c0"]))
row("ROM FLOW_BODYZ=0 vs C =0", G["c0"], G["rom0"], range(len(G["c0"])))
row("ROM FLOW_BODYZ=1 vs C =1", G["c1"], G["rom1"], range(len(G["c1"])))
print("velocity error vs truth, tilted steps")
for n in ("c0", "c1", "rom0", "rom1"):
    ex = stats([R[n][k]["evx"] - tr[k]["vx"] for k in tilted])
    ey = stats([R[n][k]["evy"] - tr[k]["vy"] for k in tilted])
    print("  %-5s evx max %.4f rms %.4f   evy max %.4f rms %.4f" % (n, ex[0], ex[1], ey[0], ey[1]))
EOF
echo "outputs in $W"
