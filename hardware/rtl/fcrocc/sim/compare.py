#!/usr/bin/env python3
"""Compare two FC output CSVs (replay or closed-loop) and report per-signal error.
Usage: compare.py golden.csv test.csv [--label NAME]"""
import sys, csv, math

def load(path):
    rows=[]; hdr=None
    for line in open(path):
        line=line.strip()
        if not line or line.startswith("#"): continue
        if hdr is None:
            hdr=line.split(","); continue
        c=line.split(",")
        if len(c)!=len(hdr): continue
        try: rows.append([float(x) for x in c])
        except ValueError: continue
    return hdr,rows

def main():
    g,t=sys.argv[1],sys.argv[2]
    label=sys.argv[4] if len(sys.argv)>4 else ""
    hg,rg=load(g); ht,rt=load(t)
    n=min(len(rg),len(rt))
    cols={name:i for i,name in enumerate(hg)}
    # signal groups (present in both replay and closed-loop est/u columns)
    groups={
      "pos(m)":["ex","ey","ez"], "att(rad)":["er1","er2","er3"],
      "vel(m/s)":["evx","evy","evz"], "u(norm)":["u0","u1","u2","u3"],
    }
    print(f"== compare {label}: {g.split('/')[-1]} (golden) vs {t.split('/')[-1]}  rows={n} ==")
    for gname,sigs in groups.items():
        rms=0.0; mx=0.0; cnt=0; scale=0.0
        for s in sigs:
            if s not in cols: continue
            i=cols[s]
            for k in range(n):
                d=rt[k][i]-rg[k][i]
                rms+=d*d; mx=max(mx,abs(d)); scale=max(scale,abs(rg[k][i])); cnt+=1
        if cnt==0: continue
        rms=math.sqrt(rms/cnt)
        print(f"  {gname:10s} rms={rms:.6g}  max={mx:.6g}  (peak|golden|={scale:.4g})")

if __name__=="__main__": main()
