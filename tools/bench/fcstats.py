#!/usr/bin/env python3
"""Loop-rate and telemetry summary of a timestamped flight_controller log.

    python3 tools/bench/fcstats.py <out-prefix>.ts.log

Input is the .ts.log that tools/bench/tsreader.py or tsfollow.py writes
(<epoch seconds> <line>). Only complete telemetry lines count: a line cut off
or interleaved with another printk is skipped rather than parsed half-way.

Reports:
  - the loop rate from HOST timestamps, from the second telemetry line on so
    boot is excluded. This is the number to quote as the loop period: it
    includes the telemetry printk and cannot be skewed by a wrong kernel tick.
  - the image's own dt= field for it >= 60. Host and image agreeing is what
    shows the tick is the rate rb derived from the shell's DTS.
  - the tofv=1 fraction and the heights seen while valid.
  - every distinct duty vector. A no-motor run must show only zeros.
  - the largest host gap between telemetry lines (a stall shows here), and
    the z, |a| and st= ranges.
"""
import re
import statistics
import sys
from collections import Counter

TELEMETRY = re.compile(r"it=(\d+) dt=(\d+)ms")
FIELDS = {
    "tofv": re.compile(r" tofv=(\d)"),
    "tofh": re.compile(r" tofh=([-\d.]+)"),
    "duty": re.compile(r" duty=\[([^\]]*)\]"),
    "z": re.compile(r" z=([-\d.]+)"),
    "amag": re.compile(r" \|a\|=([\d.]+)"),
}
STATE = re.compile(r" st=(\d+)")


def telemetry(path):
    """[(host_t, it, dt_ms, fields, line)] for each complete telemetry line."""
    rows = []
    with open(path, errors="replace") as log:
        for raw in log:
            stamp, _, line = raw.rstrip("\n").partition(" ")
            match = TELEMETRY.search(line)
            if not match:
                continue
            found = {k: p.search(line) for k, p in FIELDS.items()}
            if not all(found.values()):
                continue
            try:
                host_t = float(stamp)
            except ValueError:
                continue
            fields = {k: m.group(1) for k, m in found.items()}
            rows.append((host_t, int(match.group(1)), int(match.group(2)), fields, line))
    return rows


def main(argv):
    if len(argv) != 2:
        sys.exit(__doc__)
    rows = telemetry(argv[1])
    print("telemetry lines", len(rows))
    if len(rows) < 3:
        sys.exit("fcstats: need at least 3 telemetry lines for a loop rate")

    (t0, i0), (t1, i1) = rows[1][:2], rows[-1][:2]
    print("it %d..%d over %.2f s (host timestamps, it>=%d) -> %.2f it/s, %.1f ms/it"
          % (i0, i1, t1 - t0, i0, (i1 - i0) / (t1 - t0), 1000 * (t1 - t0) / (i1 - i0)))

    dts = [dt for _, it, dt, _, _ in rows if it >= 60]
    if dts:
        print("dt= (it>=60): n=%d min %d max %d mean %.1f median %d"
              % (len(dts), min(dts), max(dts), statistics.mean(dts), statistics.median(dts)))
        print("dt histogram", sorted(Counter(dts).items()))

    fields = [f for _, _, _, f, _ in rows]
    valid = [f["tofh"] for f in fields if f["tofv"] == "1"]
    print("tofv=1: %d/%d; tofh values on tofv=1: %s" % (len(valid), len(fields), sorted(set(valid))))
    print("duty values", sorted({f["duty"] for f in fields}))

    gaps = [(rows[k + 1][0] - rows[k][0], rows[k][1]) for k in range(1, len(rows) - 1)]
    gap, at = max(gaps)
    print("max host gap between telemetry lines %.3f s at it=%d; mean %.3f"
          % (gap, at, statistics.mean(g for g, _ in gaps)))

    z = [float(f["z"]) for (_, it, _, f, _) in rows if it >= 20]
    if z:
        print("z range (it>=20) %.3f..%.3f" % (min(z), max(z)))
    amag = [float(f["amag"]) for f in fields]
    print("|a| %.3f..%.3f" % (min(amag), max(amag)))
    print("st values", sorted({m.group(1) for *_, line in rows for m in [STATE.search(line)] if m}))


if __name__ == "__main__":
    main(sys.argv)
