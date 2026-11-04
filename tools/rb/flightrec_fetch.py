#!/usr/bin/env python3
"""Read the DRAM flight recorder out of a running flight controller over the ESP.

    rb flightlog fetch --port socket://192.168.4.1:14552 --mode <preset>

(runs this in the zephyr env). The recorder is integration/zephyr/flightrec: a
ring of samples in DDR, one per control tick (or every div-th), frozen
FLIGHTREC_TAIL_MS after the motors stop so the last flight stays put. This
opens an RBPOKE1 session -- TSI reads under the running program, no reset --
checks that the running code is this ELF, reads the header, then only the last
segment (with its preroll, which after the first segment lives in a separate
preroll ring), and writes it as .bin (exact bytes) and .csv.

Two layouts, told apart by the header:
  version 2            flight builds, 84-byte struct flightrec_sample (attitude
                       as Gibbs vectors); PREFIX.csv uses COLUMNS.
  version 3, layout 2  roll-harness (ROSE_RIG=1) builds, 92-byte struct
                       flightrec_rig_sample (true radians); PREFIX.csv uses
                       RIG_COLUMNS, and PREFIX.gains.json holds the gain
                       snapshot taken at segment start plus the ring of the last
                       64 accepted GAIN changes (gains_at() rebuilds the gains
                       in force at any sample's gain_seq).

Rig builds control ROLL (rig-roll) or PITCH (rig-pitch, the drone turned 90 deg on the pivot).
The sample bytes are the same; in a pitch build roll_tgt..rate_int are the PITCH loop's and
pitch_cmd is the would-be command on the locked roll axis, while roll/pitch/gyro stay physical.
The axis comes from the ELF (g_rig_axis) or its build record (-DRIG_AXIS=1), see rig_axis.py, and
the CSV ends with generic columns for the controlled axis: axis (R/P), ang_deg, ang_tgt_deg,
ang_rate_dps.

Python API for the rig runner: fetch(url, elf, out_prefix) -> dict(csv=, bin=,
gains=, samples=, gaps=, layout=, axis=); it raises FetchError instead of printing.

A POKE session takes the FPGA's uart1, so the motor frames stop while it runs
and the ESP cuts all four motors within 120 ms. Fetch after landing.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import struct
import sys
import time
import types
from typing import Callable

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import esp_tsi_load as E  # noqa: E402
import rig_axis  # noqa: E402

MAGIC = 0x31435246
VERSION = 2                         # flight layout, 64-byte header
VERSION_RIG = 3                     # header + struct flightrec_rig_ext
LAYOUT_FLIGHT = 1
LAYOUT_RIG = 2

HDR_FMT = "<IHH" + "I" * 14         # struct flightrec_header
SAMPLE_FMT = "<IHBB3f3f3fff4f4f"    # struct flightrec_sample
HDR_SIZE = struct.calcsize(HDR_FMT)
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)

RIG_SAMPLE_FMT = "<IHBBBBH18f4H"    # struct flightrec_rig_sample
RIG_EXT_FMT = "<IHHII12fI"          # struct flightrec_rig_ext up to changes[]
GAIN_CHANGE_FMT = "<HBBfI"          # struct flightrec_gain_change
RIG_SAMPLE_SIZE = struct.calcsize(RIG_SAMPLE_FMT)
RIG_EXT_SIZE = struct.calcsize(RIG_EXT_FMT)
GAIN_CHANGE_SIZE = struct.calcsize(GAIN_CHANGE_FMT)
RIG_N_GAINS = 12                    # FLIGHTREC_RIG_GAINS: gains in the v3 header snapshot
GAIN_RING = 64
RIG_HDR_SIZE = HDR_SIZE + RIG_EXT_SIZE + GAIN_RING * GAIN_CHANGE_SIZE

# fc_rig.h enum fc_rig_gain_id order (append only); tests/test_flightrec_fetch.py
# checks this against both the enum and fc_rig.cpp's gain_table. Only the first RIG_N_GAINS are
# in the header snapshot (layout v3 unchanged); the ones after (the lift-off assist's LIFT_I,
# LIFT_I_LIM) appear in gains.json only through the change ring, i.e. once a GAIN set them.
RIG_GAIN_NAMES = ("ATT_P", "ATT_I", "RATE_P", "RATE_I", "RATE_D", "ATT_GAIN", "D_LPF_HZ",
                  "GYRO_LPF_HZ", "ROLL_TRIM", "MOTOR_TRIM_LEFT", "ATT_I_LIM", "RATE_I_LIM",
                  "LIFT_I", "LIFT_I_LIM")
# fc_rig.h enum fc_rig_run_state (also checked by the tests).
RIG_STATE_NAMES = ("IDLE", "ARMING", "RAMP_UP", "RUN", "RAMP_DOWN", "DONE")

CHUNK = 4096
FLAGS = ((0x01, "armed"), (0x02, "estop"), (0x04, "arming"), (0x08, "caldone"),
         (0x10, "tof_valid"), (0x20, "flow_valid"), (0x40, "committed"))

COLUMNS = (["t_s", "t_us", "dt_us", "seq", "flags"] + [n for _, n in FLAGS]
           + ["gyro_p", "gyro_q", "gyro_r",
              "roll_g", "pitch_g", "yaw_g", "roll_deg", "pitch_deg", "yaw_deg",
              "sp_roll_g", "sp_pitch_g", "sp_yaw_g", "sp_roll_deg", "sp_pitch_deg",
              "z", "zsp", "u0", "u1", "u2", "u3", "duty0", "duty1", "duty2", "duty3"])

# RIG-SPEC "Recorder v3 (rig) <-> runner contract". rs_name is ESTOP while the
# estop flag is set (as the live RIG line's rs=), else the run state's name. The last four are the
# controlled axis (RIG-SPEC "Pitch axis"): axis R|P, ang_deg = roll_deg|pitch_deg, ang_tgt_deg =
# roll_tgt_deg (the firmware's controlled-axis target), ang_rate_dps = gyro_p|gyro_q in deg/s.
RIG_COLUMNS = (["t_s", "t_us", "dt_us", "seq", "flags"] + [n for _, n in FLAGS]
               + ["rig_state", "rs_name", "exc_id", "gain_seq", "gyro_p", "gyro_q", "gyro_r",
                  "roll_deg", "pitch_deg", "roll_tgt_deg", "rate_tgt_dps", "rate_cmd",
                  "moment_mNm", "exc_val", "coll_thrust_N", "att_int", "rate_int",
                  "pitch_cmd", "yaw_cmd", "accel_y", "accel_z", "vbat",
                  "duty0", "duty1", "duty2", "duty3",
                  "axis", "ang_deg", "ang_tgt_deg", "ang_rate_dps"])


class FetchError(RuntimeError):
    """The recorder could not be read or interpreted; nothing was written."""


def gibbs_deg(g: float) -> float:
    """Gibbs/Rodrigues component tan(theta/2) -> degrees (state[3..5] are NOT radians)."""
    return math.degrees(2.0 * math.atan(g))


def f32(x: float) -> float:
    """The shortest decimal that is the same float32 (0.85, not 0.8500000238418579)."""
    want = struct.pack("<f", x)
    for p in range(1, 10):
        y = float(f"{x:.{p}g}")
        if struct.pack("<f", y) == want:
            return y
    return x


def read_span(tsi: E.Tsi, addr: int, nbytes: int, label: str,
              log: Callable[[str], None] = print) -> bytes:
    out = bytearray()
    t0 = time.monotonic()
    while len(out) < nbytes:
        k = min(CHUNK, nbytes - len(out))
        out += tsi.read(addr + len(out), k)
        if nbytes > 8 * CHUNK and len(out) % (16 * CHUNK) < CHUNK:
            rate = len(out) / max(time.monotonic() - t0, 1e-3)
            log(f"  {label}: {len(out)}/{nbytes} B "
                f"({(nbytes - len(out)) / rate:.0f} s left)")
    return bytes(out)


# ---- the rig header extension --------------------------------------------------------------

def rig_gains(ext: bytes) -> dict:
    """struct flightrec_rig_ext (with its change ring) -> the PREFIX.gains.json object.
    changes are the ring's valid entries, oldest first."""
    (_layout, n_gains, ring, snap_seq, snap_seg, *rest) = struct.unpack_from(RIG_EXT_FMT, ext)
    snap, count = rest[:RIG_N_GAINS], rest[RIG_N_GAINS]
    if n_gains != RIG_N_GAINS or ring != GAIN_RING:
        raise FetchError(f"rig header has {n_gains} gains / a {ring}-entry change ring; this "
                         f"tool knows {RIG_N_GAINS} / {GAIN_RING} (update RIG_N_GAINS / "
                         "RIG_GAIN_NAMES from flightrec.h / fc_rig.h)")
    changes = []
    for k in range(max(0, count - ring), count):
        seq, idx, _pad, value, t_us = struct.unpack_from(
            GAIN_CHANGE_FMT, ext, RIG_EXT_SIZE + (k % ring) * GAIN_CHANGE_SIZE)
        changes.append({"gain_seq": seq,
                        "name": RIG_GAIN_NAMES[idx] if idx < len(RIG_GAIN_NAMES) else f"#{idx}",
                        "value": f32(value), "t_us": t_us})
    return {"snapshot": {"gain_seq": snap_seq, "segment": snap_seg,
                         "gains": {n: f32(v) for n, v in zip(RIG_GAIN_NAMES[:RIG_N_GAINS], snap)}},
            "changes": changes, "changes_total": count,
            "changes_lost": max(0, count - ring)}


def gains_at(info: dict, gain_seq: int) -> dict[str, float | None]:
    """The gains in force at a sample's gain_seq, from a gains.json object. A gain the log
    cannot pin down (its change fell out of the 64-entry ring) is None."""
    snap = info["snapshot"]
    s0 = snap["gain_seq"]
    by_seq = {c["gain_seq"]: c for c in info["changes"]}
    out: dict[str, float | None] = dict(snap["gains"])
    if gain_seq >= s0:
        for s in range(s0 + 1, gain_seq + 1):
            c = by_seq.get(s)
            if c is None:
                return {n: None for n in out}      # a change between is not in the ring
            out[c["name"]] = c["value"]
        return out
    # Before the snapshot (preroll): undo the changes in (gain_seq, s0].
    undone = set()
    for s in range(gain_seq + 1, s0 + 1):
        c = by_seq.get(s)
        if c is None:
            return {n: None for n in out}
        undone.add(c["name"])
    for name in undone:
        prior = [c for c in info["changes"] if c["name"] == name and c["gain_seq"] <= gain_seq]
        out[name] = max(prior, key=lambda c: c["gain_seq"])["value"] if prior else None
    return out


# ---- CSV writers ----------------------------------------------------------------------------

def _t_rel(t_us: int, ref: int) -> float:
    t = ((t_us - ref) & 0xFFFFFFFF) / 1e6
    return t - 2**32 / 1e6 if t > 2**31 / 1e6 else t


def _write_flight_csv(path: str, blob: bytes, n: int, div: int) -> dict:
    rows = [struct.unpack_from(SAMPLE_FMT, blob, k * SAMPLE_SIZE) for k in range(n)]
    first_motor = next((r[0] for r in rows if any(d > 0 for d in r[-4:])), None)
    ref = first_motor if first_motor is not None else rows[0][0]
    gaps = 0
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        prev = None
        for r in rows:
            t_us, dt_us, flags, seq = r[0:4]
            g = r[4:7]
            att = r[7:10]
            sp = r[10:13]
            z, zsp = r[13:15]
            u = r[15:19]
            duty = r[19:23]
            if prev is not None and ((seq - prev) & 0xFF) != (div & 0xFF):
                gaps += 1
            prev = seq
            w.writerow([f"{_t_rel(t_us, ref):.6f}", t_us, dt_us, seq, flags]
                       + [1 if flags & bit else 0 for bit, _ in FLAGS]
                       + [f"{x:.5f}" for x in g]
                       + [f"{x:.5f}" for x in att] + [f"{gibbs_deg(x):.3f}" for x in att]
                       + [f"{x:.5f}" for x in sp] + [f"{gibbs_deg(x):.3f}" for x in sp[:2]]
                       + [f"{z:.4f}", f"{zsp:.4f}"]
                       + [f"{x:.5f}" for x in u] + [f"{x:.4f}" for x in duty])
    return {"rows": rows, "gaps": gaps, "first_motor": first_motor,
            "motor_n": sum(1 for r in rows if any(d > 0 for d in r[-4:])),
            "max_roll_deg": max(abs(gibbs_deg(r[7])) for r in rows)}


def rig_state_name(rig_state: int, flags: int) -> str:
    if flags & 0x02:
        return "ESTOP"
    return RIG_STATE_NAMES[rig_state] if rig_state < len(RIG_STATE_NAMES) else str(rig_state)


def _write_rig_csv(path: str, blob: bytes, n: int, div: int, axis: int = rig_axis.ROLL) -> dict:
    letter = rig_axis.letter(axis)
    rows = [struct.unpack_from(RIG_SAMPLE_FMT, blob, k * RIG_SAMPLE_SIZE) for k in range(n)]
    first_motor = next((r[0] for r in rows if any(r[-4:])), None)
    ref = first_motor if first_motor is not None else rows[0][0]
    gaps = 0
    g6 = "{:.6g}".format
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(RIG_COLUMNS)
        prev = None
        for r in rows:
            t_us, dt_us, seq, flags, rs, exc_id, gseq = r[0:7]
            gyro = r[7:10]
            (roll, pitch, roll_tgt, rate_tgt, rate_cmd, moment, exc_val, coll,
             att_int, rate_int, pitch_cmd, yaw_cmd, ay, az, vbat) = r[10:25]
            duty = r[25:29]
            if prev is not None and ((seq - prev) & 0xFF) != (div & 0xFF):
                gaps += 1
            prev = seq
            ang, ang_rate = (pitch, gyro[1]) if axis == rig_axis.PITCH else (roll, gyro[0])
            w.writerow([f"{_t_rel(t_us, ref):.6f}", t_us, dt_us, seq, flags]
                       + [1 if flags & bit else 0 for bit, _ in FLAGS]
                       + [rs, rig_state_name(rs, flags), exc_id, gseq]
                       + [f"{x:.5f}" for x in gyro]
                       + [f"{math.degrees(roll):.4f}", f"{math.degrees(pitch):.4f}",
                          f"{math.degrees(roll_tgt):.4f}", f"{math.degrees(rate_tgt):.4f}",
                          g6(rate_cmd), g6(moment * 1e3), g6(exc_val), f"{coll:.5f}",
                          g6(att_int), g6(rate_int), g6(pitch_cmd), g6(yaw_cmd),
                          f"{ay:.4f}", f"{az:.4f}", f"{vbat:.3f}"]
                       + [f"{d / 10000:.4f}" for d in duty]
                       + [letter, f"{math.degrees(ang):.4f}", f"{math.degrees(roll_tgt):.4f}",
                          f"{math.degrees(ang_rate):.4f}"])
    k = 11 if axis == rig_axis.PITCH else 10
    return {"rows": rows, "gaps": gaps, "first_motor": first_motor,
            "motor_n": sum(1 for r in rows if any(r[-4:])),
            "max_roll_deg": max(abs(math.degrees(r[k])) for r in rows)}


# ---- the fetch ------------------------------------------------------------------------------

def fetch(url: str | None, elf: str, out_prefix: str | os.PathLike, *, usb: str | None = None,
          last: int = 4096, timeout: float = 8.0,
          log: Callable[[str], None] = lambda m: print(m, flush=True)) -> dict:
    """Read the last segment and write PREFIX.bin, PREFIX.csv (and PREFIX.gains.json for a rig
    build). Returns dict(csv, bin, gains (None unless rig), samples, gaps, layout, frozen,
    motor_samples, axis ("roll"/"pitch" for a rig layout, else None), axis_source). Raises
    FetchError (nothing written) when it cannot."""
    out_prefix = os.fspath(out_prefix)
    hdr_addr = E.elf_symbol(elf, "g_flightrec_hdr")
    smp_addr = E.elf_symbol(elf, "g_flightrec_samples")
    pre_sym = E.elf_symbol(elf, "g_flightrec_pre")
    if hdr_addr is None or smp_addr is None or pre_sym is None:
        raise FetchError(f"{elf} has no flight recorder (build with "
                         "RB_CMAKE_ARGS=-DROSE_FLIGHTREC=1, or a preset that sets it)")

    ser = None
    in_session = False
    gains = None
    try:
        ser = E.open_link(types.SimpleNamespace(url=url, usb=usb))
        tsi = E.Tsi(ser, timeout)
        ident = E.handshake(ser, timeout, "RBPOKE1", "RBPOKE")
        in_session = True
        log(f"ESP: RBPOKE OK {ident} -- no reset; motor frames pause during the read")
        for paddr, want in E.text_samples(elf):
            if tsi.read(paddr, len(want)) != want:
                raise FetchError(f"the running image is NOT {elf} (code at {paddr:#x} "
                                 "differs) -- refusing to interpret its memory")
        raw_hdr = tsi.read(hdr_addr, HDR_SIZE)
        h = struct.unpack(HDR_FMT, raw_hdr)
        (magic, version, size, cap, div, tail_ms, preroll, count,
         seg_start, seg_end, segments, frozen, samples_lo,
         pre_lo, pre_buf, pre_head, pre_count) = h
        if magic != MAGIC:
            raise FetchError(f"header magic {magic:#010x} (want {MAGIC:#010x}) -- "
                             "recorder not initialised yet?")
        if version == VERSION:
            layout = LAYOUT_FLIGHT
        elif version == VERSION_RIG:
            raw_hdr += tsi.read(hdr_addr + HDR_SIZE, RIG_HDR_SIZE - HDR_SIZE)
            layout = struct.unpack_from("<I", raw_hdr, HDR_SIZE)[0]
        else:
            raise FetchError(f"recorder v{version}; this tool reads v{VERSION} and "
                             f"v{VERSION_RIG}")
        want_size = {LAYOUT_FLIGHT: SAMPLE_SIZE, LAYOUT_RIG: RIG_SAMPLE_SIZE}.get(layout)
        if want_size is None or size != want_size:
            raise FetchError(f"recorder v{version} layout {layout} with {size}-byte samples; "
                             f"this tool reads layout {LAYOUT_FLIGHT} ({SAMPLE_SIZE} B) and "
                             f"{LAYOUT_RIG} ({RIG_SAMPLE_SIZE} B)")
        if version == VERSION_RIG:
            gains = rig_gains(raw_hdr[HDR_SIZE:])
        if samples_lo != (smp_addr & 0xFFFFFFFF) or pre_lo != (pre_sym & 0xFFFFFFFF):
            raise FetchError(f"header says samples at {samples_lo:#x} / preroll at "
                             f"{pre_lo:#x}, ELF says {smp_addr:#x} / {pre_sym:#x}")
        log(f"recorder: {count} samples written, ring {cap} x {size} B, every {div} "
            f"ticks; {segments} motor segment(s); {'FROZEN' if frozen else 'recording'}"
            + (" [rig layout]" if layout == LAYOUT_RIG else ""))
        if segments == 0:
            end = count
            start = max(0, count - last)
            log(f"no motor segment yet -- reading the last {end - start} samples")
        else:
            end = seg_end if frozen else count
            start = seg_start
            if not frozen:
                log("WARNING: the segment is still recording (motors on, or inside the "
                    f"{tail_ms} ms tail) -- reading what is there now")
        if segments and pre_count > preroll or pre_buf > 1:
            raise FetchError(f"bad preroll ring state buf={pre_buf} count={pre_count}")
        npre = pre_count if segments else 0
        if end - start > cap:
            log(f"segment is longer than the ring: keeping the last {cap} samples")
            start = end - cap
            npre = 0          # no longer contiguous with what is kept
        n = end - start
        if n <= 0:
            raise FetchError("nothing to read")
        log(f"reading {npre} preroll + samples {start}..{end - 1} "
            f"({(npre + n) * size} B, ~{(npre + n) * size / (115200 / 10):.0f} s over the link)")
        blob = bytearray()
        ring = pre_sym + pre_buf * preroll * size
        j = 0
        while j < npre:
            slot = (pre_head + j) % preroll
            run = min(npre - j, preroll - slot)
            blob += read_span(tsi, ring + slot * size, run * size, "preroll", log)
            j += run
        i = start
        while i < end:
            slot = i % cap
            run = min(end - i, cap - slot)
            blob += read_span(tsi, smp_addr + slot * size, run * size, "samples", log)
            i += run
    except (E.LoadError, OSError) as exc:
        raise FetchError(str(exc)) from exc
    finally:
        if in_session:
            try:
                tsi.end()
            except Exception:
                pass
        if ser is not None:
            ser.close()

    n += npre
    out = {"csv": out_prefix + ".csv", "bin": out_prefix + ".bin", "gains": None,
           "samples": n, "layout": layout, "frozen": bool(frozen), "axis": None,
           "axis_source": None}
    with open(out["bin"], "wb") as f:
        f.write(raw_hdr)
        f.write(blob)
    if layout == LAYOUT_RIG:
        axis, out["axis_source"] = rig_axis.elf_axis(elf)
        out["axis"] = rig_axis.name(axis)
        log(f"rig axis: {out['axis'].upper()} ({out['axis_source']})")
        st = _write_rig_csv(out["csv"], bytes(blob), n, div, axis)
        gains["axis"] = out["axis"]
        gains["axis_source"] = out["axis_source"]
    else:
        st = _write_flight_csv(out["csv"], bytes(blob), n, div)
    if gains is not None:
        out["gains"] = out_prefix + ".gains.json"
        with open(out["gains"], "w") as f:
            json.dump(gains, f, indent=1)
            f.write("\n")
    rows = st["rows"]
    out["gaps"] = st["gaps"]
    out["motor_samples"] = st["motor_n"]
    span = ((rows[-1][0] - rows[0][0]) & 0xFFFFFFFF) / 1e6
    written = f"{out_prefix}.csv{', .gains.json' if gains is not None else ''} and .bin"
    log(f"wrote {written}: {n} samples over {span:.2f} s "
        f"({n / span if span else 0:.0f} Hz), {st['motor_n']} with motors on, "
        f"{st['gaps']} seq gap(s), max |{out['axis'] or 'roll'}| {st['max_roll_deg']:.1f} deg; "
        "t_s = 0 at the first motor tick"
        if st["first_motor"] is not None else
        f"wrote {written}: {n} samples over {span:.2f} s "
        f"({n / span if span else 0:.0f} Hz), no motor ticks, {st['gaps']} seq gap(s)")
    if gains is not None and gains["changes_lost"]:
        log(f"WARNING: {gains['changes_lost']} older gain change(s) fell out of the "
            f"{GAIN_RING}-entry ring")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    where = ap.add_mutually_exclusive_group(required=True)
    where.add_argument("--url", help="socket://192.168.4.1:14552")
    where.add_argument("--usb", help="ESP USB console, e.g. /dev/ttyACM0")
    ap.add_argument("--elf", required=True, help="the running flight controller's ELF")
    ap.add_argument("--out", required=True,
                    help="output prefix (writes PREFIX.bin, PREFIX.csv; rig builds also "
                         "PREFIX.gains.json)")
    ap.add_argument("--last", type=int, default=4096,
                    help="samples to read when no motor segment exists yet (default 4096)")
    ap.add_argument("--timeout", type=float, default=8.0)
    a = ap.parse_args()
    try:
        fetch(a.url, a.elf, a.out, usb=a.usb, last=a.last, timeout=a.timeout)
    except FetchError as exc:
        print(f"flightrec: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
