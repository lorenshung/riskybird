#!/usr/bin/env python3
"""
RiskyBird roll-harness ("rig") batch runner.

    ./rb rig run SPEC.json --i-have-approval "Loren 11-04: RATE_P sweep, 1000 permille"
    ./rb rig run SPEC.json --dry-run                 # in-process fake drone, no sockets
    ./rb rig fetch results/rig/<batch>               # re-fetch the last spin of a batch
    ./rb rig analyze results/rig/<batch>             # tools/rb/rig_analyze.py

Runs in the zephyr env (pyserial + pyelftools for the ESP link, numpy/scipy for metrics); the
module itself imports with the standard library alone, so a dry run and the unit tests work
with any python3 (metrics are then skipped with a journal note).

SPEC (JSON; tools/rig/examples/rate_p_sweep.json is a complete one)
  name                  [A-Za-z0-9_-]+, names the batch directory
  axis                  "roll" (default) | "pitch": the axis the build controls (rig-roll /
                        rig-pitch, the drone turned 90 deg on the pivot). It must match the ELF's
                        (g_rig_axis, else -DRIG_AXIS in its build record, else roll); the default
                        --mode follows it. In a pitch build roll_sp_mdeg, ROLL_TRIM, the excitations
                        and every limit below act on PITCH; RIG_PITCH_MAX_RAD guards the locked ROLL
  collective_permille   COLL: per-mille of MASS*g. 0 < c <= limits.max_coll_permille <= 1300
  roll_sp_mdeg          RSP: roll setpoint, |sp| <= 20000
  mass_kg               the build's PID_MASS_KG (default 0.050, the rig-roll preset): moment caps
  base_gains            {NAME: value} merged under every grid point
  grid                  [{"gains": {NAME: value}}, ...]       explicit points, and/or
  grid_product          {NAME: [v, v, ...], ...}             their cartesian product
  excitations           [{type, target, amp, dur_ms, f0_mHz, f1_mHz}]   wire units: amp in mrad
                        (att), mrad/s (rate) or uN*m (moment); integers only
  repeats               the whole (grid x excitations) sequence runs this many times
  arrangement           {"per_spin": n}: n gain points share one armed run, GAIN lines between
  limits                {max_trials, max_coll_permille, max_run_ms, motor_budget_s, settle_ms,
                         commit_window_s (900 = ROSE_COMMIT_MAX_MS of rig-roll)}

  A trial is one (gain point, excitation, repeat); trials run in the order
  repeat > grid point > excitation, grouped into spins of at most per_spin consecutive points
  (fewer when max_run_ms, the motor budget or the commit window says so).

ONE SPIN (no FC replies are relied on: the host has no DHCP lease, so only the ESP's
`ACK <cmd>` on :14551 and its RBSTAT1 line are seen. An ACK means the ESP took the datagram, not
that the FC got it: the FC's UART loses lines sent close together, so every datagram -- HB too --
goes through one paced sender, --tx-gap-ms apart, and what matters is proven over RBPOKE)
  1. RBSTAT: link up, not ARMED, duty 0, no ESTOP/kill/WINDOW-EXPIRED, COMMITTED.
  2. RBPOKE baseline read of g_rig (+ the FC's uart1 RX counters where the ELF has them).
  3. STOP, COLL 0 and every gain of the point -> RBPOKE: values, rs DONE, coll_cmd 0, and gain_seq
     advanced by exactly the number of GAIN lines (a lost GAIN equal to the old value shows only
     there). RUNMS, RSP -> RBPOKE. COLL <c> -> RBPOKE coll_cmd. What did not land is resent (paced)
     up to 2 times, then the spin is refused. COLL goes before RESET because fc_rig.cpp's reset
     edge keeps coll_cmd (COLL_SURVIVES_RESET; Runner.arm_sequence), and in DONE it cannot arm.
  4. Heartbeat (20 Hz, priority in the sender), RBSTAT re-read (refused on ESTOP / kill unless
     --clear-estop on the first spin), RESET. ARMED is taken from RBSTAT's fl= within 5 s, nonzero
     duty journaled when seen (an RBSTAT session is ESP-local and does NOT cut the motors). No arm:
     HB stops, then RBPOKE says why (coll_cmd, arm_enabled, RESET delivered, HB age, estop, cal)
     BEFORE the STOP that would erase it.
  5. RUN by timing; the first EXC waits out the lift-off window (the FC refuses EXC before its roll
     guard is live, which cannot be read while motors run). GAIN lines between gain points.
  6. STOP; HB keeps going through RAMP_DOWN until RBSTAT shows not ARMED and zero duty; COLL 0.
  7. Wait out the ESP's 1 s post-motor session refusal and the recorder's 2 s tail, fetch the
     segment (flightrec_fetch.fetch), cut it into trials by exc_id / gain_seq, compute metrics;
     RBPOKE the RX counters again.

SAFETY
  * --i-have-approval "<text>" is required for a hardware run and recorded in batch.json.
  * The spec is validated against the firmware's ranges (fc_rig.cpp, mirrored below and checked
    against the source by tests/test_rig_run.py) and the build's EFFECTIVE limits (the -D
    overrides in rb-build.txt / compile_commands.json next to the ELF: tilt guard, roll target
    |RSP| + |ROLL_TRIM| + |att exc| <= RIG_TGT_MAX_RAD) before anything is sent; the firmware NAKs
    silently and the NAK never reaches this host.
  * Every exit path sends STOP; an exception, Ctrl-C, SIGTERM or SIGHUP sends ESTOP x3 first
    from a dedicated socket. After any estop the runner never sends RESET (which would clear it).
  * control.json from the TUI: pause finishes the current trial and ends the spin; abort sends
    STOP now (HB through the ramp-down) and ends the batch. An `estop` journal event from the
    TUI ends the batch.
  * Motor time and the commit window (wall clock from the first spin) are tracked; a spin that
    would overrun either is not started.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import hashlib
import importlib
import itertools
import json
import math
import os
import random
import re
import signal
import socket
import struct
import sys
import threading
import time
import traceback
from collections import deque
from pathlib import Path

_HERE = Path(__file__).resolve().parent
if not __package__:
    sys.path.insert(0, str(_HERE))
    import rig_tui as T  # noqa: E402
    import rig_axis as AX  # noqa: E402
else:
    from . import rig_tui as T
    from . import rig_axis as AX


def _sibling(name):
    """Import tools/rb/<name> lazily, as a package module or a script neighbour."""
    if __package__:
        return importlib.import_module("." + name, __package__)
    return importlib.import_module(name)


ROOT = _HERE.parents[1]
RIG_DIR = ROOT / "results" / "rig"
DEFAULT_DRONE = "192.168.4.1"
CMD_PORT = 14551
TSI_PORT = 14552

# =============================================================================
# Firmware mirror (workloads/flight_controller/src/fc_rig.cpp; tests check it against the source)
# =============================================================================

GAIN_TABLE = (           # fc_rig_gain_id order: also the order of g_rig.gains.v[] and the recorder
    ("ATT_P", 0.0, 40.0),
    ("ATT_I", 0.0, 50.0),
    ("RATE_P", 1.0, 200.0),
    ("RATE_I", 0.0, 500.0),
    ("RATE_D", 0.0, 2.0),
    ("ATT_GAIN", 0.1, 3.0),
    ("D_LPF_HZ", 1.0, 200.0),
    ("GYRO_LPF_HZ", 0.0, 200.0),
    ("ROLL_TRIM", -0.175, 0.175),
    ("MOTOR_TRIM_LEFT", 0.7, 1.3),
    ("ATT_I_LIM", 0.0, 10.0),
    ("RATE_I_LIM", 0.0, 100.0),
    ("LIFT_I", 0.0, 10.0),         # lift-off assist (appended; NOT in the recorder's gain snapshot)
    ("LIFT_I_LIM", 0.0, 10.0),
)
GAIN_NAMES = tuple(n for n, _, _ in GAIN_TABLE)
GAIN_RANGE = {n: (lo, hi) for n, lo, hi in GAIN_TABLE}
# The rig-roll build's defaults (TAU_ROLL 0.10 -> ATT_P 5, TAU_ROLLRATE 0.025 -> RATE_P 40,
# ATT_GAIN 0.85f and MOTOR_TRIM_LEFT 1.0f from boards.py). Only the fake drone uses these.
RIG_ROLL_DEFAULTS = {"ATT_P": 5.0, "ATT_I": 0.0, "RATE_P": 40.0, "RATE_I": 0.0, "RATE_D": 0.0,
                     "ATT_GAIN": 0.85, "D_LPF_HZ": 30.0, "GYRO_LPF_HZ": 0.0, "ROLL_TRIM": 0.0,
                     "MOTOR_TRIM_LEFT": 1.0, "ATT_I_LIM": 8.0, "RATE_I_LIM": 30.0,
                     "LIFT_I": 4.0, "LIFT_I_LIM": 8.0}

ARM_SETTLE_MS = 1000          # RIG_ARM_SETTLE_MS
RAMP_MS = 1500                # RIG_RAMP_MS (up and down)
SP_SLEW_RADPS = 0.5           # RIG_SP_SLEW_RADPS
COLL_SLEW = 0.5               # RIG_COLL_SLEW, fraction per second in RUN
HB_RAMP_MS = 300              # RIG_HB_RAMP_MS: HB silent this long -> RAMP_DOWN
HB_ESTOP_MS = 1000            # RIG_HB_ESTOP_MS: -> ESTOP
MAX_RUN_CAP_MS = 60000        # RIG_MAX_RUN_CAP_MS
RUNMS_MIN = 1000              # RIG_RUNMS_MIN
COLL_MAX_PERMILLE = 1300      # RIG_COLL_MAX_PERMILLE
SP_MAX_MDEG = 20000           # RIG_SP_MAX_MDEG
EXC_MOMENT_FRAC = 0.3         # RIG_EXC_MOMENT_FRAC
EXC_ATT_MAX_MRAD = 260        # RIG_EXC_ATT_MAX_RAD 0.26
EXC_RATE_MAX_MRADPS = 3000    # RIG_EXC_RATE_MAX_RADPS 3.0
EXC_F_MIN_MHZ = 10            # RIG_EXC_F_MIN_MHZ
EXC_F_MAX_MHZ = 50000         # RIG_EXC_F_MAX_MHZ
EXC_TAPER_FRAC = 0.1          # RIG_EXC_TAPER_FRAC
ARM_MAX_TILT_RAD = 0.61       # RIG_ARM_MAX_TILT_RAD: worst-case resting angle at arm
MAX_TILT_RAD = 0.44           # RIG_MAX_TILT_RAD: the roll estop
LIFTOFF_MARGIN_RAD = 0.05     # RIG_LIFTOFF_MARGIN_RAD
LIFTOFF_TIMEOUT_MS = 2000     # RIG_LIFTOFF_TIMEOUT_MS
ARM_MAX_RATE_RADPS = 0.30     # main.cpp ARM_MAX_RATE_RADPS
PITCH_MAX_RAD = 0.17          # RIG_PITCH_MAX_RAD
J_ROLL = 16e-6                # controller_pid.cpp J[0]
L_ARM = 33e-3                 # controller_pid.cpp l_arm
GRAVITY = 9.81
DEFAULT_MASS_KG = 0.050       # PID_MASS_KG of the rig-roll preset
LINE_MAX = 78                 # main.cpp's uplink reader keeps 79 bytes; leave one spare
EXC_TYPES = ("step", "doublet", "chirp", "sine", "prbs")
EXC_TARGETS = ("att", "rate", "moment")
RS_NAMES = ("IDLE", "ARMING", "RAMP_UP", "RUN", "RAMP_DOWN", "DONE")

# Host-side margins (not firmware).
TILT_BUDGET_DEG = 18.0        # |sp| + worst steady excitation offset; the estop is at 25 deg
EXC_CAP_MARGIN = 0.98         # stay under the moment cap: float rounding on the FC side
FLIGHTREC_TAIL_S = 2.0        # FLIGHTREC_TAIL_MS: the segment freezes this long after motors stop
ESP_HOLDOFF_S = 1.0           # load_link.c: sessions refused for 1 s after a nonzero duty
POKE_HANDBACK_S = 0.6         # the ESP hands uart1 back ~0.3 s after a session's END
HB_PERIOD_S = 0.05            # 20 Hz
HB_ACK_LOST_S = 1.0           # no ESP ACK to HB this long while armed -> link lost
ARM_SLACK_S = 4.0             # extra time allowed for the arm conditions to hold
RUN_MARGIN_S = 3.0            # RUNMS = planned live time + this
TRIAL_OVERHEAD_S = 0.3        # per trial: commands, stat check
GAIN_CMD_S = 0.1              # per live GAIN line
STOP_CONFIRM_S = 3.0          # after the ramp-down: RBSTAT must show motors off within this
FETCH_TRIES = 4
REC_HZ = 315.0                # recorder samples/s at FLIGHTREC_DIV 4 (measured loop rate)
FETCH_S_PER_SAMPLE = 0.008    # flightrec_fetch over WiFi: ~8 ms per sample
PREROLL_S = 3.2               # FLIGHTREC_PREROLL 1000 samples at 315 Hz, ahead of the segment
REC_CAPACITY = 32768          # FLIGHTREC_CAPACITY: a longer segment loses its START on fetch
COMMIT_MARGIN_S = 10.0
PRE_WINDOW_S = 0.3            # trial slice: this much before the EXC
# The uplink. The FC polls the FPGA UART every ~2.8 ms (up to 18 ms) and its RX FIFO holds ~8
# bytes: lines that arrive close together, or long ones, get corrupted (11-04: 7 GAINs 4-11 ms
# apart, 1 applied; COLL 5 ms after RESET never arrived). The ESP ACKs a datagram, not delivery.
TX_GAP_MS = 30.0              # default minimum gap between two datagrams to :14551
UART_RX_FIFO = 8              # bytes (the fake drone's loss model)
FAKE_RX_WINDOW_MS = 20.0      # fake drone: a long line this soon after the previous one is lost
RESEND_TRIES = 2              # a line that did not land is sent again at most this often
POKE_READ_S = 3.0             # one RBPOKE verification read over WiFi, incl. the uart1 handback
SETUP_VERIFY_S = 5 * POKE_READ_S   # per spin: baseline, gains, params, COLL, RX counters after
HB_LEAD_S = 0.3               # HB runs this long before the RESET
# fc_rig.cpp: the reset_gen edge in fc_rig_step() does NOT clear coll_cmd (only to_done() -- STOP,
# DONE, DISARM -- and a latched estop do), so COLL is sent and proven over RBPOKE BEFORE the RESET
# that arms with it (Runner.arm_sequence). tests/test_rig_run.py checks this against the source.
# If the reset edge ever clears coll_cmd, set this False: RESET first, then COLL, confirmed only
# by RBSTAT showing ARMED.
COLL_SURVIVES_RESET = True

# =============================================================================
# Numbers on the wire (fc_rig_parse_decimal / fc_rig_format_decimal, ported)
# =============================================================================

_DEC_RE = re.compile(r"^([+-]?)([0-9]+)(?:\.([0-9]+))?$")


def f32(v):
    return struct.unpack("<f", struct.pack("<f", float(v)))[0]


def parse_decimal(s):
    """fc_rig_parse_decimal: the float32 the FC will hold, or None where it NAKs."""
    m = _DEC_RE.match(s or "")
    if not m:
        return None
    ip, fp = m.group(2), m.group(3) or ""
    if len(ip) > 9 or len(fp) > 12 or len(ip) + len(fp) > 15:
        return None
    v = int(ip + fp) / (10 ** len(fp))
    return f32(-v if m.group(1) == "-" else v)


def format_gain(v):
    """Shortest decimal (no exponent) that the FC parses back to exactly float32(v); None if
    there is none (inf, nan, out of the parser's digit limits)."""
    try:
        v32 = f32(v)
    except (OverflowError, ValueError, TypeError, struct.error):
        return None
    if not math.isfinite(v32):
        return None
    a = abs(v32)
    for sig in range(1, 10):
        dec = 0 if a == 0 else min(12, max(0, sig - 1 - int(math.floor(math.log10(a)))))
        s = "%.*f" % (dec, v32)
        if "." in s:
            s = s.rstrip("0").rstrip(".")
        if s in ("-0", ""):
            s = "0"
        if parse_decimal(s) == v32:
            return s
    return None


# =============================================================================
# g_rig over RBPOKE: what the FC actually holds (no FC reply reaches this host)
# =============================================================================

# (path in struct fc_rig_state, struct format). On hardware EVERY offset comes from the running
# ELF's DWARF (fc_rig.h keeps changing; only gains + gain_seq are promised to stay first), and an
# ELF without DWARF for fc_rig_state is refused. Fields after RIG_REQUIRED are read if present.
RIG_FIELDS = (("gains", "<%df" % len(GAIN_NAMES)), ("gain_seq", "<I"), ("last_gain", "<b"),
              ("coll_cmd", "<f"), ("sp_roll_cmd", "<f"), ("max_run_ms", "<i"),
              ("last_hb_ms", "<q"), ("hb_seen", "<?"), ("exc_id", "<I"), ("rs", "<B"),
              ("reset_seen", "<I"), ("arm_since", "<q"), ("guard_active", "<?"),
              ("dbg.roll", "<f"), ("dbg.pitch", "<f"), ("coll_ramp_to", "<f"))
RIG_REQUIRED = ("gains", "gain_seq", "coll_cmd", "sp_roll_cmd", "max_run_ms", "rs")
# The fake drone's own g_rig layout (its memory image; never used for hardware).
FAKE_RIG_OFFSETS = {"gains": 0, "gain_seq": 56, "last_gain": 60, "coll_cmd": 64, "sp_roll_cmd": 68,
                    "max_run_ms": 72, "last_hb_ms": 80, "hb_seen": 88, "exc_id": 136, "rs": 140,
                    "reset_seen": 144, "arm_since": 152, "guard_active": 188, "dbg.roll": 248,
                    "dbg.pitch": 252, "coll_ramp_to": 368}
RIG_STATE_SIZE = 376
# main.cpp / kernel statics the arm diagnosis reads where the ELF has them: (key, symbol, format).
# They are file statics, so .symtab often lacks them; DWARF has their DW_OP_addr.
FC_GLOBALS = (("arm_enabled", "g_arm_enabled", "<?"), ("estop", "g_estop", "<?"),
              ("gyro_cal_done", "g_gyro_cal_done", "<?"), ("reset_gen", "g_reset_gen", "<I"),
              ("vbat", "g_vbat", "<f"), ("fc_tick", "curr_tick", "<q"))
# The FC's uplink RX health counters (interrupt-driven uart1 RX, being added 11-04), e.g.
# g_uart1_rx_overruns, g_uart1_lines_ok, g_uart1_lines_bad: every 4- or 8-byte object with this
# prefix is read; none is required.
RX_COUNTER_PREFIX = "g_uart1_"
_ZL_RE = re.compile(r"^_ZL(\d+)(.+)$")


def _uleb(data):
    v = shift = 0
    for b in data:
        v |= (b & 0x7F) << shift
        shift += 7
        if not b & 0x80:
            break
    return v


def _die_name(die):
    a = die.attributes.get("DW_AT_name")
    if a is None:
        return None
    v = a.value
    return v.decode("utf-8", "replace") if isinstance(v, bytes) else str(v)


def _die_type(die):
    """The DIE's type with typedef / const / volatile peeled off, or None."""
    seen = 0
    while die is not None and "DW_AT_type" in die.attributes and seen < 16:
        die = die.get_DIE_from_attribute("DW_AT_type")
        seen += 1
        if die.tag not in ("DW_TAG_typedef", "DW_TAG_const_type", "DW_TAG_volatile_type"):
            return die
    return None


def _member_offsets(struct_die, prefix=""):
    out = {}
    for m in struct_die.iter_children():
        if m.tag != "DW_TAG_member":
            continue
        name, loc = _die_name(m), m.attributes.get("DW_AT_data_member_location")
        if name is None or loc is None:
            continue
        off = loc.value
        if isinstance(off, list):                       # DW_OP_plus_uconst <uleb>
            off = _uleb(off[1:]) if off and off[0] == 0x23 else None
        if not isinstance(off, int):
            continue
        out[prefix + name] = off
        if name == "dbg" and not prefix:
            t = _die_type(m)
            if t is not None and t.tag == "DW_TAG_structure_type":
                for k, v in _member_offsets(t, "dbg.").items():
                    out[k] = off + v
    return out


def _wanted_symbol(name):
    return name == "g_rig" or name.startswith(RX_COUNTER_PREFIX) or \
        any(name == s for _, s, _ in FC_GLOBALS)


def elf_rig_symbols(elf):
    """pyelftools ELFFile -> ({name: (addr, size)}, fc_rig_state offsets or None, its size or None).
    Names from .symtab (plain, or the _ZL<n><name> of a C++ file static), then DWARF variables
    whose location is a plain DW_OP_addr; the struct layout from DWARF."""
    syms = {}
    tab = elf.get_section_by_name(".symtab")
    if tab is not None:
        for s in tab.iter_symbols():
            name = s.name
            m = _ZL_RE.match(name)
            if m and len(m.group(2)) == int(m.group(1)):
                name = m.group(2)
            if _wanted_symbol(name) and s["st_info"]["type"] == "STT_OBJECT" and name not in syms:
                syms[name] = (s["st_value"], s["st_size"])
    offsets = size = None
    if elf.has_dwarf_info():
        asz = 8 if elf.elfclass == 64 else 4
        for cu in elf.get_dwarf_info().iter_CUs():
            for die in cu.get_top_DIE().iter_children():
                if die.tag == "DW_TAG_structure_type":
                    if offsets is None and _die_name(die) == "fc_rig_state" and \
                            "DW_AT_byte_size" in die.attributes:
                        offsets = _member_offsets(die)
                        size = die.attributes["DW_AT_byte_size"].value
                    continue
                if die.tag != "DW_TAG_variable" or "DW_AT_location" not in die.attributes:
                    continue
                decl = die
                name = _die_name(die)
                if name is None and "DW_AT_specification" in die.attributes:
                    decl = die.get_DIE_from_attribute("DW_AT_specification")
                    name = _die_name(decl)
                if name is None or name in syms or not _wanted_symbol(name):
                    continue
                loc = die.attributes["DW_AT_location"].value
                if isinstance(loc, list) and len(loc) == 1 + asz and loc[0] == 0x03:
                    t = _die_type(decl)
                    bs = t.attributes.get("DW_AT_byte_size") if t is not None else None
                    syms[name] = (int.from_bytes(bytes(loc[1:]), "little"), bs.value if bs else 0)
    return syms, offsets, size


class RigLayout:
    """Where one RBPOKE verification reads: g_rig's fields, the FC globals the arm diagnosis wants
    and the uplink RX counters (the last two only where the ELF has them), and how to decode them."""

    def __init__(self, rig_addr, offsets=None, size=RIG_STATE_SIZE, fc_globals=None, rx=None,
                 source="fake", ticks_per_s=1000):
        self.rig_addr = rig_addr
        self.offsets = dict(FAKE_RIG_OFFSETS if offsets is None else offsets)
        self.size = size
        self.fc_globals = dict(fc_globals or {})        # key -> (addr, fmt)
        self.rx = dict(rx or {})                        # symbol -> (addr, fmt)
        self.source, self.ticks_per_s = source, ticks_per_s
        missing = [p for p in RIG_REQUIRED if p not in self.offsets]
        if missing:
            raise RuntimeError("struct fc_rig_state (%s) lacks %s: rig_run.py's RIG_FIELDS need an "
                               "update" % (source, ", ".join(missing)))

    @classmethod
    def from_elf(cls, elf_path):
        from elftools.elf.elffile import ELFFile      # the zephyr env; a dry run never gets here
        with open(elf_path, "rb") as f:
            syms, offsets, size = elf_rig_symbols(ELFFile(f))
        if "g_rig" not in syms:
            raise RuntimeError("%s has no g_rig: not a ROSE_RIG (rig-roll) build" % elf_path)
        rig_addr, rig_size = syms["g_rig"]
        if not offsets:
            raise RuntimeError("%s has no DWARF for struct fc_rig_state: RBPOKE verification needs "
                               "it (build with debug info)" % elf_path)
        source = "dwarf"
        fc = {k: (syms[s][0], fmt) for k, s, fmt in FC_GLOBALS if s in syms}
        rx = {n: (a, "<I" if sz == 4 else "<Q") for n, (a, sz) in syms.items()
              if n.startswith(RX_COUNTER_PREFIX) and sz in (4, 8)}
        tps = 1000
        try:
            m = re.search(r"^CONFIG_SYS_CLOCK_TICKS_PER_SEC=(\d+)$",
                          (Path(elf_path).parent / ".config").read_text(), re.M)
            tps = int(m.group(1)) if m else tps
        except OSError:
            pass
        return cls(rig_addr, offsets, size or rig_size or RIG_STATE_SIZE, fc, rx, source, tps)

    def items(self):
        """[(space, key, addr, fmt)] of everything one read covers."""
        out = [("rig", p, self.rig_addr + self.offsets[p], fmt)
               for p, fmt in RIG_FIELDS if p in self.offsets]
        out += [("fc", k, a, fmt) for k, (a, fmt) in sorted(self.fc_globals.items())]
        out += [("rx", k, a, fmt) for k, (a, fmt) in sorted(self.rx.items())]
        return out

    def regions(self):
        """Word-aligned, coalesced (addr, nbytes) TSI reads covering items()."""
        spans = sorted((a & ~3, (a + struct.calcsize(fmt) + 3) & ~3) for _, _, a, fmt in self.items())
        out = []
        for lo, hi in spans:
            if out and lo <= out[-1][1] + 64:
                out[-1][1] = max(out[-1][1], hi)
            else:
                out.append([lo, hi])
        return [(lo, hi - lo) for lo, hi in out]

    def decode(self, regions, blobs):
        def get(addr, n):
            for (lo, ln), b in zip(regions, blobs):
                if lo <= addr and addr + n <= lo + min(ln, len(b)):
                    return b[addr - lo:addr - lo + n]
            raise RuntimeError("internal: %#x+%d was not read" % (addr, n))
        snap = {"rx": {}}
        for space, key, addr, fmt in self.items():
            vals = struct.unpack(fmt, get(addr, struct.calcsize(fmt)))
            v = vals if len(vals) > 1 else vals[0]
            if space == "rx":
                snap["rx"][key] = v
            else:
                snap[key.replace("dbg.", "")] = v
        snap["gains"] = dict(zip(GAIN_NAMES, snap["gains"]))
        snap["coll_permille"] = int(round(snap["coll_cmd"] * 1000.0))
        snap["rsp_mdeg"] = int(round(math.degrees(snap["sp_roll_cmd"]) * 1000.0))
        rs = snap["rs"]
        snap["rs_name"] = RS_NAMES[rs] if 0 <= rs < len(RS_NAMES) else "?%d" % rs
        if "fc_tick" in snap:
            snap["fc_ms"] = int(snap["fc_tick"] * 1000 // max(1, self.ticks_per_s))
            if snap.get("hb_seen") and "last_hb_ms" in snap:
                snap["hb_age_ms"] = snap["fc_ms"] - snap["last_hb_ms"]
        return snap


def rig_summary(snap):
    """The JSON-able part of a decoded g_rig read, for the journal."""
    out = {"gains": {n: format_gain(v) and float(format_gain(v)) for n, v in snap["gains"].items()}}
    for k in ("gain_seq", "coll_permille", "rsp_mdeg", "max_run_ms", "rs_name", "reset_seen",
              "hb_seen", "last_hb_ms", "arm_since", "exc_id", "arm_enabled", "estop",
              "gyro_cal_done", "reset_gen", "fc_ms", "hb_age_ms"):
        if k in snap:
            out[k] = snap[k]
    for k in ("roll", "pitch", "vbat"):
        if k in snap and math.isfinite(snap[k]):
            out[k] = round(snap[k], 4)
    if snap["rx"]:
        out["rx"] = dict(snap["rx"])
    return out


def runms_applied(v):
    return min(max(int(v), RUNMS_MIN), MAX_RUN_CAP_MS)


def rsp_applied(v):
    return min(max(int(v), -SP_MAX_MDEG), SP_MAX_MDEG)


def coll_applied(v):
    return min(max(int(v), 0), COLL_MAX_PERMILLE)


def compare_rig(before, sent, snap, exp):
    """What did not land: (problems, lines to resend, fatal). `before` is the g_rig read before
    `sent` went out, `exp` what the FC must now hold: gains (all 14), coll, rsp, runms (None = not
    checked), rs (allowed run states) and stop_ok (a stray rs may be fixed with a STOP).

    gain_seq is the delivery check: it must advance by exactly the number of GAIN lines sent. A
    value compare alone cannot tell "applied" from "unchanged" (11-04: GAINs equal to the defaults
    were lost and --verify-poke passed). When it falls short and no value differs, the lost line is
    one whose value equals what the FC already held -- which one is unknowable, so all of those are
    sent again, and the next round must again account for every one of them."""
    problems, resend, fatal = [], [], False
    rs = snap["rs_name"]
    if rs not in exp["rs"]:
        if rs in ("RAMP_UP", "RUN", "RAMP_DOWN"):
            return ["g_rig.rs is %s: the FC is ARMED during the disarmed setup" % rs], [], True
        problems.append("g_rig.rs is %s, %s expected" % (rs, "/".join(exp["rs"])))
        if exp.get("stop_ok"):
            resend.append("STOP")
        else:
            fatal = True
    if exp.get("coll") is not None and snap["coll_permille"] != coll_applied(exp["coll"]):
        problems.append("coll_cmd is %d permille, %d sent" % (snap["coll_permille"], exp["coll"]))
        resend.append("COLL %d" % exp["coll"])
    gain_sent = [ln for ln in sent if ln.startswith("GAIN ")]
    wrong = []
    for n in GAIN_NAMES:
        if snap["gains"][n] != f32(exp["gains"][n]):
            problems.append("%s=%s, %s expected" % (n, format_gain(snap["gains"][n]),
                                                   format_gain(exp["gains"][n])))
            wrong.append(gain_line(n, exp["gains"][n]))
    resend += wrong
    delta = (snap["gain_seq"] - before["gain_seq"]) & 0xFFFFFFFF
    if delta != len(gain_sent):
        problems.append("gain_seq advanced by %d for %d GAIN line(s)" % (delta, len(gain_sent)))
        if delta > len(gain_sent):
            fatal = True          # a GAIN we did not send (or a corrupted one) was applied
        elif len(gain_sent) - delta > sum(1 for ln in gain_sent if ln in wrong):
            same = [ln for ln in gain_sent if ln not in wrong
                    and before["gains"][ln.split()[1]] == parse_decimal(ln.split()[2])]
            if not same:
                fatal = True
            resend += same
    if exp.get("runms") is not None and snap["max_run_ms"] != runms_applied(exp["runms"]):
        problems.append("max_run_ms is %d, %d sent" % (snap["max_run_ms"], exp["runms"]))
        resend.append("RUNMS %d" % exp["runms"])
    if exp.get("rsp") is not None and abs(snap["rsp_mdeg"] - rsp_applied(exp["rsp"])) > 1:
        problems.append("sp_roll_cmd is %d mdeg, %d sent" % (snap["rsp_mdeg"], exp["rsp"]))
        resend.append("RSP %d" % exp["rsp"])
    return problems, resend, fatal


def arm_reasons(snap, base, hb_off_s, axis=0):
    """(definite reasons, state notes) for an FC that did not arm, from a g_rig read taken after the
    heartbeat was stopped and BEFORE any STOP (STOP's to_done() zeroes coll_cmd and the dwell).
    `base` is the read before the spin's RESET, `hb_off_s` how long ago the host stopped HB."""
    why, notes = [], ["rs=%s" % snap["rs_name"]]
    if base is not None and snap.get("reset_gen") is not None and \
            base.get("reset_gen") == snap["reset_gen"]:
        why.append("RESET never reached the FC (g_reset_gen still %d)" % snap["reset_gen"])
    if snap.get("arm_enabled") is False:
        why.append("arming not enabled (g_arm_enabled=0: no RESET since boot)")
    if snap["coll_permille"] <= 0:
        why.append("coll_cmd is 0 (COLL lost, or zeroed by a STOP / estop)")
    else:
        notes.append("coll_cmd=%d permille" % snap["coll_permille"])
    if snap.get("estop"):
        why.append("the FC estop is latched (g_estop=1)")
    if snap.get("gyro_cal_done") is False:
        why.append("gyro calibration not done")
    if not snap.get("hb_seen", True):
        why.append("no HB ever reached the FC")
    elif snap.get("hb_age_ms") is not None:
        late = snap["hb_age_ms"] - hb_off_s * 1000.0
        if late > HB_RAMP_MS:
            why.append("HB stale: the last HB reached the FC ~%d ms before the host stopped "
                       "sending (> %d ms)" % (late, HB_RAMP_MS))
        else:
            notes.append("HB fresh until the host stopped it")
    if "arm_enabled" in snap:
        notes.append("arm_enabled=%d" % snap["arm_enabled"])
    if "roll" in snap and math.isfinite(snap["roll"]):
        ctl, lck = AX.name(axis), AX.name(1 - axis)
        notes.append("roll %.1f / pitch %.1f deg (last law tick; arm needs |%s| < %.0f, locked "
                     "|%s| < %.0f)" % (math.degrees(snap["roll"]), math.degrees(snap.get("pitch", 0.0)),
                                       ctl, math.degrees(ARM_MAX_TILT_RAD), lck,
                                       math.degrees(PITCH_MAX_RAD)))
    if "vbat" in snap and math.isfinite(snap["vbat"]) and snap["vbat"] > 0:
        notes.append("vbat %.2f V" % snap["vbat"])
    return why, notes


# =============================================================================
# Spec: load, validate, expand
# =============================================================================

class SpecError(ValueError):
    def __init__(self, problems):
        self.problems = list(problems)
        super().__init__("batch spec refused:\n  - " + "\n  - ".join(self.problems))


def _is_int(v):
    return isinstance(v, int) and not isinstance(v, bool)


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def moment_cap_unm(coll_permille, mass_kg):
    """fc_rig.cpp exc_cap(MOMENT) at the collective, in uN*m: 0.3 * 2*0.25*u0*l_arm."""
    u0 = coll_permille / 1000.0 * mass_kg * GRAVITY
    return EXC_MOMENT_FRAC * 2.0 * 0.25 * u0 * L_ARM * 1e6


def exc_line(e):
    parts = ["EXC", e["type"], e["target"], str(e["amp"]), str(e["dur_ms"])]
    if e["type"] in ("chirp", "sine", "prbs"):
        parts.append(str(e["f0_mHz"]))
    if e["type"] == "chirp":
        parts.append(str(e["f1_mHz"]))
    return " ".join(parts)


def gain_line(name, value):
    return "GAIN %s %s" % (name, format_gain(value))


def _check_gains(where, gains, problems):
    if not isinstance(gains, dict):
        problems.append("%s: gains must be an object {NAME: value}" % where)
        return {}
    out = {}
    for k, v in gains.items():
        if k not in GAIN_RANGE:
            problems.append("%s: unknown gain %r (whitelist: %s)" % (where, k, ", ".join(GAIN_NAMES)))
            continue
        if not _is_num(v):
            problems.append("%s: %s=%r is not a finite number" % (where, k, v))
            continue
        lo, hi = GAIN_RANGE[k]
        v32 = f32(v)
        if not (f32(lo) <= v32 <= f32(hi)):
            problems.append("%s: %s=%s outside the firmware range [%s, %s] (the FC would NAK)"
                            % (where, k, v, lo, hi))
            continue
        if format_gain(v) is None or len(gain_line(k, v)) > LINE_MAX:
            problems.append("%s: %s=%r cannot be sent as a plain decimal" % (where, k, v))
            continue
        out[k] = float(v)
    return out


def expand_grid(raw, problems):
    """base_gains + grid + grid_product -> [{"gains": {...}}] (at least one point)."""
    base = _check_gains("base_gains", raw.get("base_gains") or {}, problems)
    points = []
    grid = raw.get("grid")
    if grid is not None:
        if not isinstance(grid, list):
            problems.append("grid must be a list of {\"gains\": {...}}")
            grid = []
        for i, p in enumerate(grid):
            g = p.get("gains") if isinstance(p, dict) and "gains" in p else p
            points.append(_check_gains("grid[%d]" % i, g, problems))
    prod = raw.get("grid_product")
    if prod is not None:
        if not isinstance(prod, dict) or not prod:
            problems.append("grid_product must be a non-empty {NAME: [values]}")
        else:
            names, lists = [], []
            for k, vals in prod.items():
                if not isinstance(vals, list) or not vals:
                    problems.append("grid_product.%s must be a non-empty list" % k)
                    continue
                names.append(k)
                lists.append(vals)
            for combo in itertools.product(*lists):
                points.append(_check_gains("grid_product", dict(zip(names, combo)), problems))
    if not points:
        points = [{}]
    out = []
    for p in points:
        g = dict(base)
        g.update(p)
        out.append({"gains": g})
    return out


def _check_exc(i, e, coll, mass, problems, att_max_mrad=EXC_ATT_MAX_MRAD):
    where = "excitations[%d]" % i
    if not isinstance(e, dict):
        problems.append("%s must be an object" % where)
        return None
    t, tgt = e.get("type"), e.get("target")
    if t not in EXC_TYPES:
        problems.append("%s: type %r not in %s" % (where, t, EXC_TYPES))
        return None
    if tgt not in EXC_TARGETS:
        problems.append("%s: target %r not in %s" % (where, tgt, EXC_TARGETS))
        return None
    out = {"type": t, "target": tgt}
    for k in ("amp", "dur_ms"):
        if not _is_int(e.get(k)):
            problems.append("%s: %s must be an integer (wire units), got %r" % (where, k, e.get(k)))
            return None
        out[k] = e[k]
    if out["amp"] == 0:
        problems.append("%s: amp 0 (the FC NAKs a zero amplitude)" % where)
    if not 1 <= out["dur_ms"] <= MAX_RUN_CAP_MS:
        problems.append("%s: dur_ms %d outside [1, %d]" % (where, out["dur_ms"], MAX_RUN_CAP_MS))
    need = ("f0_mHz",) if t in ("sine", "prbs") else (("f0_mHz", "f1_mHz") if t == "chirp" else ())
    for k in ("f0_mHz", "f1_mHz"):
        v = e.get(k)
        if k in need:
            if not _is_int(v) or not EXC_F_MIN_MHZ <= v <= EXC_F_MAX_MHZ:
                problems.append("%s: %s must be an integer in [%d, %d] mHz for %s, got %r"
                                % (where, k, EXC_F_MIN_MHZ, EXC_F_MAX_MHZ, t, v))
            out[k] = v
        else:
            out[k] = None
    a = abs(out["amp"])
    if tgt == "att" and a > att_max_mrad:
        problems.append("%s: att amplitude %d mrad over the cap %d (the FC NAKs it)"
                        % (where, a, att_max_mrad))
    if tgt == "rate" and a > EXC_RATE_MAX_MRADPS:
        problems.append("%s: rate amplitude %d mrad/s over the cap %d" % (where, a, EXC_RATE_MAX_MRADPS))
    if tgt == "moment":
        cap = moment_cap_unm(coll, mass) * EXC_CAP_MARGIN
        if a > cap:
            problems.append("%s: moment amplitude %d uN*m over the cap %.0f at %d permille "
                            "(0.3 x the roll authority, mass %.3f kg, with a 2%% margin)"
                            % (where, a, cap, coll, mass))
    if len(exc_line(out)) > LINE_MAX:
        problems.append("%s: command line too long" % where)
    return out


def exc_offset_deg(exc, gains):
    """Steady roll offset (deg) that holding this excitation at full amplitude would cause with
    the P cascade (I terms would remove it later; P alone is the worst case)."""
    a = abs(exc["amp"])
    if exc["target"] == "att":
        return math.degrees(a * 1e-3)
    att_p = gains.get("ATT_P", RIG_ROLL_DEFAULTS["ATT_P"])
    if att_p <= 0:
        return float("inf")
    if exc["target"] == "rate":
        return math.degrees(a * 1e-3 / att_p)
    rate_p = gains.get("RATE_P", RIG_ROLL_DEFAULTS["RATE_P"])
    ag = gains.get("ATT_GAIN", RIG_ROLL_DEFAULTS["ATT_GAIN"])
    return math.degrees(a * 1e-6 / (J_ROLL * ag * rate_p * att_p))


# fc_rig.cpp knobs a preset may override with -D, and their source defaults (tests check these
# against fc_rig.cpp). RIG_TGT_MAX_RAD defaults to RIG_MAX_TILT_RAD - TGT_MARGIN_RAD.
FC_LIMIT_DEFAULTS = (("RIG_MAX_TILT_RAD", MAX_TILT_RAD), ("RIG_TGT_MAX_RAD", None),
                     ("RIG_ARM_MAX_TILT_RAD", ARM_MAX_TILT_RAD), ("RIG_PITCH_MAX_RAD", PITCH_MAX_RAD),
                     ("RIG_LIFTOFF_MARGIN_RAD", LIFTOFF_MARGIN_RAD),
                     ("RIG_LIFTOFF_TIMEOUT_MS", LIFTOFF_TIMEOUT_MS), ("RIG_SP_MAX_MDEG", SP_MAX_MDEG),
                     ("RIG_COLL_MAX_PERMILLE", COLL_MAX_PERMILLE),
                     ("RIG_EXC_ATT_MAX_RAD", EXC_ATT_MAX_MRAD / 1000.0))
TGT_MARGIN_RAD = 0.09
TILT_BUDGET_MARGIN_DEG = math.degrees(MAX_TILT_RAD) - TILT_BUDGET_DEG   # 7 deg under the guard
_DEFINE_RE = re.compile(r"(?:^|\s)-D([A-Za-z_]\w*)(?:=(\S+))?")


def _define_value(s):
    try:
        return float(s.rstrip("fFlLuU"))
    except (ValueError, AttributeError):
        return None


def build_defines(elf_path):
    """({NAME: value string} of the -D flags the ELF was built with, where they came from), from
    the build record rb-build.txt (its cmake: and cppflags: lines) or compile_commands.json's
    fc_rig.cpp entry, next to the ELF or one directory up. ({}, None) when neither is there."""
    elf = Path(elf_path)
    for d in (elf.parent, elf.parent.parent):
        rec = d / "rb-build.txt"
        try:
            text = rec.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        out = {}
        for key in ("cmake:", "cppflags:"):           # cppflags last: it wins
            for line in text.splitlines():
                if line.startswith(key):
                    out.update({m.group(1): m.group(2) or "1" for m in _DEFINE_RE.finditer(line)})
        return out, str(rec)
    for d in (elf.parent, elf.parent.parent):
        cc = d / "compile_commands.json"
        try:
            entries = json.loads(cc.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        for e in entries:
            if str(e.get("file", "")).endswith("fc_rig.cpp"):
                cmd = e.get("command") or " ".join(e.get("arguments") or [])
                return {m.group(1): m.group(2) or "1" for m in _DEFINE_RE.finditer(cmd)}, str(cc)
    return {}, None


def fc_limits(elf_path=None):
    """The EFFECTIVE fc_rig.cpp limits of a build: its -D overrides over the source defaults.
    -> {NAME: number, ..., "PID_MASS_KG": number or None, "source": text}."""
    defs, src = build_defines(elf_path) if elf_path else ({}, None)
    out = {}
    for name, dflt in FC_LIMIT_DEFAULTS:
        v = _define_value(defs.get(name)) if name in defs else None
        out[name] = v if v is not None else dflt
    if out["RIG_TGT_MAX_RAD"] is None:
        out["RIG_TGT_MAX_RAD"] = out["RIG_MAX_TILT_RAD"] - TGT_MARGIN_RAD
    for k in ("RIG_LIFTOFF_TIMEOUT_MS", "RIG_SP_MAX_MDEG", "RIG_COLL_MAX_PERMILLE"):
        out[k] = int(out[k])
    out["PID_MASS_KG"] = _define_value(defs.get("PID_MASS_KG"))
    over = sorted(n for n, _ in FC_LIMIT_DEFAULTS if n in defs)
    out["source"] = ("%s (-D %s)" % (src, ", ".join(over) if over else "none of the limits")
                     if src else "fc_rig.cpp defaults (no build record)")
    return out


def limits_text(lim, axis=0):
    """The effective limits in words. In a pitch build (axis 1) the tilt guard, the target clamp,
    RSP and ROLL_TRIM act on PITCH, and RIG_PITCH_MAX_RAD is the locked-axis guard on ROLL."""
    ax = AX.name(axis)
    return ("%s tilt guard %.1f deg, %s target %.1f deg (|RSP| + |ROLL_TRIM| + |att exc|), RSP %d "
            "mdeg, locked-axis (%s) guard %.1f deg (RIG_PITCH_MAX_RAD), COLL %d permille, att exc "
            "%.0f mrad, liftoff timeout %d ms; from %s"
            % (ax, math.degrees(lim["RIG_MAX_TILT_RAD"]), ax, math.degrees(lim["RIG_TGT_MAX_RAD"]),
               lim["RIG_SP_MAX_MDEG"], AX.name(1 - axis), math.degrees(lim["RIG_PITCH_MAX_RAD"]),
               lim["RIG_COLL_MAX_PERMILLE"], lim["RIG_EXC_ATT_MAX_RAD"] * 1000,
               lim["RIG_LIFTOFF_TIMEOUT_MS"], lim["source"]))


def tgt_within(sp_rad, trim, amp_rad, tgt_max):
    """fc_rig.cpp tgt_within(): the worst-case roll target inside RIG_TGT_MAX_RAD (float32)."""
    return abs(f32(sp_rad)) + abs(f32(trim)) + abs(f32(amp_rad)) <= f32(tgt_max) * (1.0 + 1e-6)


def rsp_rad(mdeg):
    return f32(f32(mdeg) * f32(f32(3.14159265) / f32(180000.0)))


def validate_spec(raw, limits=None, elf_axis=None):
    """-> the normalised spec, or SpecError listing every problem. `limits` are the build's
    effective fc_rig.cpp limits (fc_limits(elf)); the source defaults without one. `elf_axis` is
    (axis, source) of the build the spec will run on (rig_axis.elf_axis) or None when unknown; the
    spec's optional "axis" must match it, and defaults to it (else roll)."""
    fcl = limits or fc_limits(None)
    p = []
    if not isinstance(raw, dict):
        raise SpecError(["the spec must be a JSON object"])
    known = {"name", "collective_permille", "roll_sp_mdeg", "mass_kg", "base_gains", "grid",
             "grid_product", "excitations", "repeats", "arrangement", "limits", "notes", "comment",
             "axis"}
    axis = elf_axis[0] if elf_axis else AX.ROLL
    if "axis" in raw:
        want = raw["axis"] if raw["axis"] in AX.AXIS_NAMES else None
        if want is None:
            p.append("axis must be \"roll\" or \"pitch\", got %r" % (raw["axis"],))
        else:
            axis = AX.AXIS_NAMES.index(want)
            if elf_axis and elf_axis[0] != axis:
                p.append("axis %r does not match the build: it controls %s (%s); build and load "
                         "the %s mode (./rb build flight_controller --target arty200t --mode %s) or "
                         "fix the spec" % (want, AX.name(elf_axis[0]).upper(), elf_axis[1],
                                           AX.AXIS_MODES[axis], AX.AXIS_MODES[axis]))
    axn = AX.name(axis)
    for k in raw:
        if k not in known:
            p.append("unknown key %r (typo?)" % k)
    name = raw.get("name")
    if not isinstance(name, str) or not re.match(r"^[A-Za-z0-9_-]{1,40}$", name):
        p.append("name must be 1-40 of [A-Za-z0-9_-], got %r" % (name,))
    lim = raw.get("limits")
    if not isinstance(lim, dict):
        p.append("limits is required: {max_trials, max_coll_permille, max_run_ms, motor_budget_s, "
                 "settle_ms}")
        lim = {}
    limits = {}
    for k, lo, hi, need_int in (("max_trials", 1, 1000, True),
                                ("max_coll_permille", 1, fcl["RIG_COLL_MAX_PERMILLE"], True),
                                ("max_run_ms", RUNMS_MIN, MAX_RUN_CAP_MS, True),
                                ("motor_budget_s", 1, 900, False),
                                ("settle_ms", 0, 20000, True),
                                ("commit_window_s", 30, 900, False)):
        v = lim.get(k, 900 if k == "commit_window_s" else None)
        ok = _is_int(v) if need_int else _is_num(v)
        if not ok or not lo <= v <= hi:
            p.append("limits.%s must be %s in [%s, %s], got %r"
                     % (k, "an integer" if need_int else "a number", lo, hi, v))
        else:
            limits[k] = v
    for k in lim:
        if k not in ("max_trials", "max_coll_permille", "max_run_ms", "motor_budget_s",
                     "settle_ms", "commit_window_s"):
            p.append("unknown limits key %r" % k)
    coll = raw.get("collective_permille")
    cmax = limits.get("max_coll_permille", fcl["RIG_COLL_MAX_PERMILLE"])
    if not _is_int(coll) or not 0 < coll <= cmax:
        p.append("collective_permille must be an integer in (0, %d] (limits.max_coll_permille; "
                 "firmware cap %d), got %r" % (cmax, fcl["RIG_COLL_MAX_PERMILLE"], coll))
        coll = None
    sp = raw.get("roll_sp_mdeg", 0)
    if not _is_int(sp) or abs(sp) > fcl["RIG_SP_MAX_MDEG"]:
        p.append("roll_sp_mdeg (the %s setpoint) must be an integer with |sp| <= %d, got %r"
                 % (axn, fcl["RIG_SP_MAX_MDEG"], sp))
        sp = 0
    mass = raw.get("mass_kg", DEFAULT_MASS_KG)
    if not _is_num(mass) or not 0.01 <= mass <= 0.5:
        p.append("mass_kg must be the build's PID_MASS_KG in [0.01, 0.5], got %r" % (mass,))
        mass = DEFAULT_MASS_KG
    if fcl.get("PID_MASS_KG") and abs(mass - fcl["PID_MASS_KG"]) > 1e-6:
        p.append("mass_kg %g is not the build's PID_MASS_KG %g (the moment caps follow it)"
                 % (mass, fcl["PID_MASS_KG"]))
    grid = expand_grid(raw, p)
    excs_raw = raw.get("excitations")
    excs = []
    if not isinstance(excs_raw, list) or not excs_raw:
        p.append("excitations must be a non-empty list")
    else:
        for i, e in enumerate(excs_raw):
            x = _check_exc(i, e, coll or 1, mass, p, int(round(fcl["RIG_EXC_ATT_MAX_RAD"] * 1000)))
            if x is not None:
                excs.append(x)
    reps = raw.get("repeats", 1)
    if not _is_int(reps) or not 1 <= reps <= 100:
        p.append("repeats must be an integer in [1, 100], got %r" % (reps,))
        reps = 1
    arr = raw.get("arrangement") or {}
    per_spin = arr.get("per_spin", 1) if isinstance(arr, dict) else None
    if not _is_int(per_spin) or not 1 <= per_spin <= 50:
        p.append("arrangement.per_spin must be an integer in [1, 50], got %r" % (per_spin,))
        per_spin = 1
    # Excitation size against the roll estop (the build's RIG_MAX_TILT_RAD): |sp| + steady offset
    # under P must stay TILT_BUDGET_MARGIN_DEG inside it.
    sp_deg = abs(sp) / 1000.0
    guard_deg = math.degrees(fcl["RIG_MAX_TILT_RAD"])
    budget = guard_deg - TILT_BUDGET_MARGIN_DEG
    for i, x in enumerate(excs):
        worst = max(exc_offset_deg(x, g["gains"]) for g in grid)
        if sp_deg + worst > budget:
            p.append("excitations[%d] (%s): |setpoint| %.1f deg + steady offset %.1f deg exceeds "
                     "%.0f deg (the %s estop is at %.0f deg)" % (
                         i, exc_line(x), sp_deg, worst, budget, axn, guard_deg))
    # The roll ANGLE target: the FC refuses RSP / GAIN ROLL_TRIM / EXC att whose worst case
    # |RSP| + |ROLL_TRIM| + |att exc amp| is over RIG_TGT_MAX_RAD (an absent ROLL_TRIM counts as 0).
    tmax = fcl["RIG_TGT_MAX_RAD"]
    for gi, g in enumerate(grid):
        trim = g["gains"].get("ROLL_TRIM", 0.0)
        amps = [abs(x["amp"]) * 1e-3 for x in excs if x["target"] == "att"] or [0.0]
        for a in sorted(set([0.0] + amps)):
            if not tgt_within(rsp_rad(sp), trim, a, tmax):
                p.append("grid point %d: |RSP| %.1f deg + |ROLL_TRIM| %.1f deg + |att exc| %.1f deg "
                         "is over the build's %s target limit RIG_TGT_MAX_RAD %.1f deg (the FC "
                         "refuses it)" % (gi, abs(sp) / 1000.0, math.degrees(abs(trim)),
                                          math.degrees(a), axn, math.degrees(tmax)))
                break
    spec = {"name": name, "axis": axn, "collective_permille": coll, "roll_sp_mdeg": sp, "mass_kg": mass,
            "base_gains": dict(raw.get("base_gains") or {}), "grid": grid, "excitations": excs,
            "repeats": reps, "arrangement": {"per_spin": per_spin}, "limits": limits,
            "fc_limits": fcl}
    if not p and "grid_product" in raw:
        spec["grid_product"] = raw["grid_product"]
    if not p:
        # Every trial must fit one spin on its own.
        for x in excs:
            est = spin_estimate(spec, [{"exc": x, "point": 0, "gains": grid[0]["gains"]}])
            if est["live_s"] + RUN_MARGIN_S > limits["max_run_ms"] / 1000.0:
                p.append("%s needs %.1f s armed (ramp, settle, excitation, margin): more than "
                         "limits.max_run_ms %d" % (exc_line(x), est["live_s"] + RUN_MARGIN_S,
                                                   limits["max_run_ms"]))
        first = spin_estimate(spec, plan_trials(spec)[:1])
        if first["motor_s"] > limits["motor_budget_s"]:
            p.append("limits.motor_budget_s %.0f does not fit even one trial (%.1f s of motor time)"
                     % (limits["motor_budget_s"], first["motor_s"]))
    if p:
        raise SpecError(p)
    return spec


def load_spec(path, limits=None, elf_axis=None):
    try:
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)
    except OSError as exc:
        raise SpecError(["cannot read %s: %s" % (path, exc)])
    except ValueError as exc:
        raise SpecError(["%s is not valid JSON: %s" % (path, exc)])
    return validate_spec(raw, limits, elf_axis)


def plan_trials(spec):
    """[{trial, point, gains, exc, exc_index, rep}] in run order, max_trials applied."""
    out = []
    for rep in range(spec["repeats"]):
        for gi, g in enumerate(spec["grid"]):
            for ei, e in enumerate(spec["excitations"]):
                out.append({"trial": len(out) + 1, "point": gi, "gains": dict(g["gains"]),
                            "exc": dict(e), "exc_index": ei, "rep": rep})
    return out[:spec["limits"]["max_trials"]]


LIFTOFF_WAIT_MARGIN_S = 0.3


def liftoff_wait_s(spec):
    """From RUN until the FC has certainly lifted off or given up. The FC refuses EXC (and holds
    its integrators) until its roll guard goes live (guard_active: |roll| inside the liftoff band),
    which this host cannot see while motors run (an RBPOKE session would cut them). Past
    RIG_LIFTOFF_TIMEOUT_MS after RAMP_UP the FC has either lifted off or begun its own ramp-down,
    which RBSTAT then shows as the motors stopping."""
    lim = spec.get("fc_limits") or {}
    return lim.get("RIG_LIFTOFF_TIMEOUT_MS", LIFTOFF_TIMEOUT_MS) / 1000.0 + LIFTOFF_WAIT_MARGIN_S


def pre_settle_s(spec):
    """From RUN to the first excitation: the lift-off window (liftoff_wait_s), or the setpoint slew
    that RAMP_UP did not cover (it starts at the arm-time angle, worst case the 35 deg arm limit)
    if that is longer, plus settle_ms."""
    slew = (ARM_MAX_TILT_RAD + abs(math.radians(spec["roll_sp_mdeg"] / 1000.0))) / SP_SLEW_RADPS
    return max(liftoff_wait_s(spec), slew - RAMP_MS / 1000.0) + spec["limits"]["settle_ms"] / 1000.0


def point_runs(trials):
    """Split consecutive trials into runs of the same gain point."""
    runs = []
    for t in trials:
        if runs and runs[-1][0]["point"] == t["point"] and runs[-1][0]["gains"] == t["gains"]:
            runs[-1].append(t)
        else:
            runs.append([t])
    return runs


def spin_estimate(spec, trials):
    """Seconds: live_s (arm -> STOP sent), motor_s (arm -> motors off) and wall_s (disarmed setup
    and its RBPOKE verification -> fetched)."""
    settle = spec["limits"]["settle_ms"] / 1000.0
    live = RAMP_MS / 1000.0 + pre_settle_s(spec)
    prev = None
    for run in point_runs(trials):
        if prev is not None:
            changed = sum(1 for k, v in run[0]["gains"].items() if prev.get(k) != v)
            live += settle + GAIN_CMD_S * max(1, changed)
        for t in run:
            live += t["exc"]["dur_ms"] / 1000.0 + settle + TRIAL_OVERHEAD_S
        prev = run[0]["gains"]
    motor = live + RAMP_MS / 1000.0 + 0.5
    fetch = fetch_estimate_s(motor)
    wall = SETUP_VERIFY_S + ARM_SETTLE_MS / 1000.0 + ARM_SLACK_S + motor + STOP_CONFIRM_S + \
        max(ESP_HOLDOFF_S, FLIGHTREC_TAIL_S) + fetch + 5.0
    return {"live_s": live, "motor_s": motor, "fetch_s": fetch, "wall_s": wall}


def rec_samples(motor_s):
    """Recorder samples in one spin's segment: preroll + motors + tail."""
    return (PREROLL_S + motor_s + FLIGHTREC_TAIL_S) * REC_HZ


def fetch_estimate_s(motor_s):
    """flightrec_fetch reads the whole segment at ~8 ms a sample."""
    return rec_samples(motor_s) * FETCH_S_PER_SAMPLE


def take_spin(spec, queue, motor_left_s, commit_left_s):
    """The trials of the next spin (a prefix of queue), and why it stopped growing."""
    chosen, why = [], "queue"
    for t in queue:
        cand = chosen + [t]
        if len(point_runs(cand)) > spec["arrangement"]["per_spin"]:
            why = "per_spin"
            break
        est = spin_estimate(spec, cand)
        if est["live_s"] + RUN_MARGIN_S > spec["limits"]["max_run_ms"] / 1000.0:
            why = "max_run_ms"
            break
        if rec_samples(est["motor_s"]) > 0.9 * REC_CAPACITY:
            why = "recorder"       # unreachable at the 60 s RUNMS cap; kept so it stays true
            break
        if est["motor_s"] > motor_left_s:
            why = "budget"
            break
        if est["wall_s"] > commit_left_s:
            why = "commit_window"
            break
        chosen = cand
    return chosen, why


# =============================================================================
# Clocks
# =============================================================================

class RealClock:
    speed = 1.0

    def now(self):
        return time.monotonic()

    def wall(self):
        return time.time()

    def sleep(self, s):
        if s > 0:
            time.sleep(s)

    def wait(self, event, s):
        return event.wait(max(0.0, s))


class SimClock:
    """Virtual seconds running `speed` times faster than real time (dry runs, tests)."""

    def __init__(self, speed=10.0, wall0=None):
        self.speed = float(speed)
        self.r0 = time.monotonic()
        self.w0 = time.time() if wall0 is None else wall0

    def now(self):
        return (time.monotonic() - self.r0) * self.speed

    def wall(self):
        return self.w0 + self.now()

    def sleep(self, s):
        if s > 0:
            time.sleep(s / self.speed)

    def wait(self, event, s):
        return event.wait(max(0.0, s) / self.speed)


# =============================================================================
# Links: the real ESP over WiFi, and an in-process fake drone
# =============================================================================

class EspRefused(RuntimeError):
    """The ESP refused a session (motors-running: try again later; tsi-unsynced: do not)."""


class TxPacer:
    """The one gate every datagram to :14551 passes: at least `gap` seconds of clock time from the
    END of one datagram to the start of the next. Priority senders (HB, STOP) go ahead of waiting
    commands, so a command delays an HB by at most one gap and the drift-free heartbeat catches up.
    ESTOP never waits (bypass) but still pushes the next datagram back by a gap."""

    def __init__(self, clock, gap_s):
        self.clock, self.gap = clock, max(0.0, float(gap_s))
        self.cv = threading.Condition()
        self.busy = False
        self.prio_waiting = 0
        self.last = None
        self.count = 0

    @contextlib.contextmanager
    def slot(self, priority=False):
        with self.cv:
            if priority:
                self.prio_waiting += 1
            try:
                while self.busy or (not priority and self.prio_waiting):
                    self.cv.wait(0.05)
                self.busy = True
            finally:
                if priority:
                    self.prio_waiting -= 1
        try:
            while True:
                with self.cv:
                    due = None if self.last is None else self.last + self.gap
                left = 0.0 if due is None else due - self.clock.now()
                if left <= 0:
                    break
                self.clock.sleep(left)
            yield
        finally:
            with self.cv:
                self.last = self.clock.now()
                self.count += 1
                self.busy = False
                self.cv.notify_all()

    def bypass(self):
        with self.cv:
            self.last = self.clock.now()


class Uplink:
    """The single sender to :14551: commands, HB and STOP through one TxPacer (the link's
    tx / await_reply split keeps the ACK wait outside the gate); ESTOP x3 immediately."""

    def __init__(self, link, clock, gap_s=TX_GAP_MS / 1000.0):
        self.link, self.clock = link, clock
        self.pacer = TxPacer(clock, gap_s)

    def cmd(self, text, timeout=0.5, priority=False):
        with self.pacer.slot(priority):
            self.link.tx(text)
        return self.link.await_reply(text, timeout)

    def hb(self):
        with self.pacer.slot(True):
            return self.link.tx_hb()

    def hb_ack_age(self):
        return self.link.hb_ack_age()

    def estop_burst(self, n=3):
        try:
            return self.link.estop_burst(n)
        finally:
            self.pacer.bypass()


class RealLink:
    """UDP :14551 (commands and HB on separate sockets; ESTOP on a third) and the ESP's TCP
    load service :14552 for RBSTAT1 / RBPOKE1 / the recorder fetch. The runner sends through an
    Uplink (the pacing); tx / tx_hb / await_reply are its raw halves."""

    def __init__(self, host=DEFAULT_DRONE, port=CMD_PORT, tsi_port=TSI_PORT, elf=None, clock=None,
                 layout=None):
        self.host, self.port = host, int(port)
        self.url = "socket://%s:%d" % (host, tsi_port)
        self.elf = str(elf) if elf else None
        self.clock = clock or RealClock()
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.hb_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.hb_sock.setblocking(False)
        self.estop_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.estop_sock.setblocking(False)
        self.hb_last_ack = None
        self.fetch_log = lambda m: print("  fetch: " + m, flush=True)
        self._E = None
        self._layout = layout
        self._image_ok = False

    def _esp(self):
        if self._E is None:
            self._E = _sibling("esp_tsi_load")
        return self._E

    def _args(self, timeout=3.0):
        return argparse.Namespace(url=self.url, usb=None, timeout=timeout)

    def cmd(self, text, timeout=0.5):
        """Unpaced send + reply, for tools; the runner goes through an Uplink."""
        self.tx(text)
        return self.await_reply(text, timeout)

    def tx(self, text):
        """Send one line, no wait."""
        self.cmd_sock.setblocking(False)
        try:
            while True:
                self.cmd_sock.recvfrom(512)   # stale replies from an earlier, timed-out send
        except (BlockingIOError, InterruptedError):
            pass
        except OSError:
            pass
        self.cmd_sock.sendto((text + "\n").encode("ascii"), (self.host, self.port))

    def await_reply(self, text, timeout=0.5):
        """The ESP's reply to the line just sent ("ACK <text>..." or "BUSY <text> ..."), or None if
        nothing came back in time. An ACK says the ESP took the datagram, NOT that the FC got it."""
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            self.cmd_sock.settimeout(left)
            try:
                data = self.cmd_sock.recvfrom(512)[0]
            except socket.timeout:
                return None
            except OSError:
                return None
            reply = data.decode("ascii", "replace").strip()
            if _reply_matches(reply, text):
                return reply

    def tx_hb(self):
        try:
            self.hb_sock.sendto(b"HB\n", (self.host, self.port))
        except OSError:
            return False
        try:
            while True:
                data = self.hb_sock.recvfrom(256)[0]
                if data.startswith(b"ACK HB"):
                    self.hb_last_ack = self.clock.now()
        except (BlockingIOError, InterruptedError):
            pass
        except OSError:
            pass
        return True

    hb = tx_hb

    def hb_ack_age(self):
        return None if self.hb_last_ack is None else self.clock.now() - self.hb_last_ack

    def estop_burst(self, n=3):
        sent = 0
        for _ in range(n):
            try:
                self.estop_sock.sendto(b"ESTOP\n", (self.host, self.port))
                sent += 1
            except OSError:
                pass
        return sent

    def stat(self):
        E = self._esp()
        try:
            return E.stat_line(self._args(2.0))
        except Exception as exc:  # noqa: BLE001 - LoadError, SerialException, OSError
            raise RuntimeError("RBSTAT1: %s" % exc)

    def rig_layout(self):
        if self._layout is None:
            if not self.elf:
                raise RuntimeError("RBPOKE verification needs the running ELF (--elf)")
            self._layout = RigLayout.from_elf(self.elf)
        return self._layout

    def peek(self, regions):
        """One RBPOKE session reading each (addr, nbytes) of the running FC. Disarmed only: the
        session holds uart1 (no HB reaches the FC) and the ESP refuses it while motors run or for
        1 s after. The first session checks 1 KiB of every code segment against the ELF, later
        ones 64 B."""
        E = self._esp()
        if not self.elf:
            raise RuntimeError("RBPOKE verification needs the running ELF (--elf)")
        ser = E.open_link(self._args(5.0))
        tsi = E.Tsi(ser, 5.0)
        in_session = False
        try:
            try:
                E.handshake(ser, 5.0, "RBPOKE1", "RBPOKE")
            except E.LoadError as exc:
                raise EspRefused(str(exc))
            in_session = True
            for paddr, want in E.text_samples(self.elf, 64 if self._image_ok else 1024):
                if tsi.read(paddr, len(want)) != want:
                    raise RuntimeError("the running image is not %s" % self.elf)
            self._image_ok = True
            return [tsi.read(addr, n) for addr, n in regions]
        finally:
            if in_session:
                try:
                    tsi.end()
                except Exception:  # noqa: BLE001
                    pass
            ser.close()

    def fetch(self, out_prefix):
        ff = _sibling("flightrec_fetch")
        if not hasattr(ff, "fetch"):
            raise RuntimeError("tools/rb/flightrec_fetch.py has no fetch() yet (recorder v3 API)")
        try:
            return ff.fetch(self.url, self.elf, str(out_prefix), log=self.fetch_log)
        except Exception as exc:  # noqa: BLE001
            text = str(exc)
            if "motors-running" in text or "tsi-unsynced" in text:
                raise EspRefused(text)
            raise

    def close(self):
        for s in (self.cmd_sock, self.hb_sock, self.estop_sock):
            try:
                s.close()
            except OSError:
                pass


def _reply_matches(reply, text):
    for head in ("ACK ", "BUSY "):
        if reply.startswith(head + text):
            rest = reply[len(head) + len(text):]
            if rest == "" or rest[0] == " ":
                return True
    return False


# ---- the fake drone (dry runs and tests) -------------------------------------------------------

def _lpf_alpha(dt, f):
    if f <= 0 or dt <= 0:
        return 1.0
    rc = 1.0 / (2.0 * math.pi * f)
    return dt / (dt + rc)


class FakeDrone:
    """The FC's rig state machine (fc_rig_step), its uplink verbs, the ESP's ACK / kill latch /
    RBSTAT / session refusal, and a roll plant: J*th'' = M(t - delay) - c*th' + k_ub*sin(th),
    with first-order motor lag, duty saturation [0, 0.95], a 1.5 % weaker left pair, gyro noise and
    hard stops at +-30 deg. Advanced lazily in DT steps up to clock.now() on every call.

    axis=1 is a rig-pitch build: the same plant on the PITCH axis (the pivot turned 90 deg), the
    moment through the mixer's pitch column (+moment raises duty[1]/duty[2], the rear pair, and
    lowers duty[0]/duty[3]; the rear pair is the 1.5 % weak one), saturation on those pairs, and
    the recorder's physical roll/pitch and gyro p/q swapped back (roll is the locked axis).

    The uplink into the FC is lossy like the real one (an ~8-byte UART RX FIFO polled every few
    ms): a line longer than rx_fifo bytes that arrives within rx_window_ms of the previous one is
    corrupted ("mangle": a byte past the FIFO is lost, so it usually NAKs and may parse as a wrong
    value; or "drop"). The ESP side still ACKs it. rx_hooks (callables(text) -> "drop" / "mangle" /
    None) inject losses deterministically. g_rig and the FC globals are exposed as a memory image
    for RBPOKE reads (peek / rig_layout), with g_uart1_* RX counters when rx_counters is true."""

    FAKE_GLOBALS = {"arm_enabled": 0x80018e22, "estop": 0x80018e24, "gyro_cal_done": 0x80018e1a,
                    "reset_gen": 0x80018df0, "vbat": 0x80018df8, "fc_tick": 0x80018dc0}
    FAKE_RX = {"g_uart1_rx_overruns": 0x80018e40, "g_uart1_lines_ok": 0x80018e44,
               "g_uart1_lines_bad": 0x80018e48}
    FAKE_RIG_ADDR = 0x80019870

    DT = 0.002
    REC_DIV = 2                # one recorder sample per 2 ticks: 250 Hz
    J_PLANT = 32e-6            # the rig's cradle doubles the controller's J
    C_DAMP = 4e-5
    K_UB = 2.0e-4              # CoM above the pivot: it falls onto a stop when unpowered
    LEFT_EFF = 0.985
    MOTOR_TAU = 0.03
    DELAY_TICKS = 4            # 8 ms transport delay
    T_MAX = 0.2229             # N per motor at duty 1: hover (coll 1.0) is duty ~0.55
    STOP_RAD = math.radians(30.0)
    CEILING = 0.95
    TGT_MAX = MAX_TILT_RAD - 0.09           # RIG_TGT_MAX_RAD's default
    PREROLL_S = 0.5

    def __init__(self, clock, seed=1, committed=True, gains=None, mass_kg=DEFAULT_MASS_KG,
                 rx_window_ms=FAKE_RX_WINDOW_MS, rx_fifo=UART_RX_FIFO, rx_corrupt="mangle",
                 rx_counters=True, axis=0):
        self.clock = clock
        self.axis = AX.PITCH if AX.parse_axis(axis) == AX.PITCH else AX.ROLL
        # duty[] indices of the pair +moment raises / lowers (controller_pid.cpp M column 1 / 2)
        self.pos_pair, self.neg_pair = ((1, 2), (0, 3)) if self.axis == AX.PITCH else ((2, 3), (0, 1))
        self.lock = threading.RLock()
        self.rng = random.Random(seed)
        self.mass = mass_kg
        self.committed = committed
        self.t = clock.now()
        self.boot = self.t
        self.gains = dict(RIG_ROLL_DEFAULTS)
        if gains:
            self.gains.update(gains)
        self.gain_seq = 0
        self.gain_log = deque(maxlen=64)
        self.exc_id = 0
        self.coll_cmd = 0.0
        self.sp_cmd = 0.0
        self.max_run_ms = 30000
        self.last_hb = None
        self.stop_req = False
        self.arm_enabled = False
        self.estop = False
        self.kill = False
        self.rs = "IDLE"
        self.arm_since = None
        self.arm_t = self.phase_t = None
        self.coll = 0.0
        self.coll_from = 0.0
        self.sp = 0.0
        self.guard = False
        self.tilt_since = None
        self.rd_reason = None
        self.exc = None
        self.law = None
        self.sat = (False, False)
        self.th, self.om = self.STOP_RAD, 0.0
        self.m_act = 0.0
        self.delay = deque([0.0] * self.DELAY_TICKS)
        self.duty = [0.0] * 4
        self.dbg = {}
        self.tick = 0
        self.last_nonzero = None
        self.motor_live_since = None
        self.window_expired = False
        self.session_end = None
        # recorder
        self.pre = deque(maxlen=int(self.PREROLL_S / (self.DT * self.REC_DIV)) + 1)
        self.seg = None
        self.seg_snapshot = None
        self.seg_frozen = False
        self.idle_since = None
        # test hooks and observations
        self.on_command = []       # callables(text), run before a command is handled
        self.commands = []         # (t, text, reply)
        self.hb_times = []
        self.events = []           # (t, text): FC "RIG EVENT"s
        self.naks = []             # (t, text, reason)
        self.estops = 0
        self.stats = 0
        # the uplink UART and the FC globals the RBPOKE reads see
        self.rx_window = rx_window_ms / 1000.0
        self.rx_fifo, self.rx_corrupt = rx_fifo, rx_corrupt
        self.rx_rng = random.Random(seed + 7)
        self.rx_last = None
        self.rx_hooks = []         # callables(text) -> "drop" / "mangle" / None
        self.rx_log = []           # (clock t, text, outcome, bypass): every datagram the FC side saw
        self.rx_ok = self.rx_bad = self.rx_overruns = 0
        self.rx_counters = rx_counters
        self.reset_gen = 0
        self.gyro_cal_done = True
        self.vbat = 7.6
        self.peeks = 0
        self._reply = None

    # ---- ESP side ----
    def tx(self, text, bypass=False):
        for hook in list(self.on_command):
            hook(text)
        with self.lock:
            self._advance()
            if self.session_end is not None and self.t < self.session_end + 0.3:
                reply = "BUSY %s -- program load in progress, NOT delivered to the FPGA" % text
                self.commands.append((self.t, text, reply))
                self._reply = (text, reply)
                return
            local = ""
            if text == "ESTOP":
                self.kill = True
                self.estops += 1
                local = " -- ESP GATES CUT AND LATCHED"
            elif text == "RESET":
                self.kill = False
                local = " -- ESP kill latch CLEARED, frames apply again"
            self._uart(text, bypass)
            reply = "ACK %s%s" % (text, local)       # the ESP ACKs whatever the UART did with it
            self.commands.append((self.t, text, reply))
            self._reply = (text, reply)

    def await_reply(self, text, timeout=0.5):
        with self.lock:
            r, self._reply = self._reply, None
        return r[1] if r is not None and r[0] == text else None

    def cmd(self, text, timeout=0.5):
        """Unpaced, like RealLink.cmd."""
        self.tx(text)
        return self.await_reply(text, timeout)

    def tx_hb(self):
        with self.lock:
            self._advance()
            self.hb_times.append(self.t)
            if self.session_end is None or self.t >= self.session_end + 0.3:
                self._uart("HB")
            return True

    hb = tx_hb

    def hb_ack_age(self):
        with self.lock:
            return None if not self.hb_times else self.clock.now() - self.hb_times[-1]

    def estop_burst(self, n=3):
        for _ in range(n):
            self.tx("ESTOP", bypass=True)
        return n

    def _uart(self, text, bypass=False):
        """The FPGA UART into the FC (see the class doc)."""
        now = self.clock.now()
        prev, self.rx_last = self.rx_last, now
        outcome = "ok"
        for hook in list(self.rx_hooks):
            o = hook(text)
            if o:
                outcome = o
                break
        if outcome == "ok" and prev is not None and now - prev < self.rx_window \
                and len(text) + 1 > self.rx_fifo:
            outcome = self.rx_corrupt
        self.rx_log.append((now, text, outcome, bypass))
        if outcome == "drop":
            self.rx_bad += 1
            self.rx_overruns += 1
            return
        line = text
        if outcome == "mangle":
            self.rx_bad += 1
            self.rx_overruns += 1
            i = self.rx_rng.randrange(min(self.rx_fifo, len(text) - 1), len(text))
            line = text[:i] + text[i + 1:]
        else:
            self.rx_ok += 1
        self._fc_command(line)

    def stat(self):
        with self.lock:
            self._advance()
            self.stats += 1
            armed = self.rs in ("RAMP_UP", "RUN", "RAMP_DOWN") and not self.estop
            fl = 0
            names = []
            gate_duty = [0 if self.kill else int(round(d * 10000)) for d in self.duty]
            if armed:
                fl |= 0x01
                names.append("ARMED")
            if self.committed:
                fl |= 0x20
                names.append("COMMITTED")
            if self.estop:
                fl |= 0x02
                names.append("ESTOP")
            if self.window_expired:
                fl |= 0x10
                names.append("WINDOW-EXPIRED")
            ftext = (" ".join(names) + " ") if names else "off"
            return ("RBLINK v1 ml=up up=%d seq=%d fl=0x%02x[%s] duty=[%d %d %d %d] fr=%d gap=5 "
                    "lost=0 stale=0 crc=0 noise=0 cuts=0 pwmerr=0 kill=%d txt=0 cl=-1"
                    % (int((self.t - self.boot) * 1000), self.tick & 0xFF, fl, ftext,
                       gate_duty[0], gate_duty[1], gate_duty[2], gate_duty[3], self.tick // 5,
                       1 if self.kill else 0))

    def _session_check(self, kind):
        if self.last_nonzero is not None and self.t - self.last_nonzero < ESP_HOLDOFF_S:
            raise EspRefused("ESP refused: %s ERR motors-running: a gate carried a nonzero duty "
                             "in the last second" % kind)
        self.session_end = self.t

    def rig_layout(self):
        g = {k: (a, fmt) for k, _, fmt in FC_GLOBALS for kk, a in self.FAKE_GLOBALS.items() if kk == k}
        rx = {n: (a, "<I") for n, a in self.FAKE_RX.items()} if self.rx_counters else {}
        return RigLayout(self.FAKE_RIG_ADDR, FAKE_RIG_OFFSETS, RIG_STATE_SIZE, g, rx, "fake")

    def _memory(self):
        """[(addr, bytes)]: g_rig in FAKE_RIG_OFFSETS layout, the FC globals, the RX counters."""
        rig = bytearray(RIG_STATE_SIZE)
        ms = lambda t: 0 if t is None else int(round((t - self.boot) * 1000))  # noqa: E731
        vals = {"gains": [f32(self.gains[n]) for n in GAIN_NAMES], "gain_seq": self.gain_seq,
                "last_gain": -1, "coll_cmd": f32(self.coll_cmd), "sp_roll_cmd": f32(self.sp_cmd),
                "max_run_ms": self.max_run_ms, "last_hb_ms": ms(self.last_hb),
                "hb_seen": self.last_hb is not None, "exc_id": self.exc_id,
                "rs": RS_NAMES.index(self.rs), "reset_seen": self.reset_gen,
                "arm_since": ms(self.arm_since), "guard_active": self.guard,
                "dbg.roll": self.th if self.axis == AX.ROLL else 0.0,
                "dbg.pitch": self.th if self.axis == AX.PITCH else 0.0,
                "coll_ramp_to": f32(self.coll_cmd)}
        for path, fmt in RIG_FIELDS:
            v = vals[path]
            struct.pack_into(fmt, rig, FAKE_RIG_OFFSETS[path], *(v if isinstance(v, list) else [v]))
        out = [(self.FAKE_RIG_ADDR, bytes(rig))]
        g = {"arm_enabled": self.arm_enabled, "estop": self.estop,
             "gyro_cal_done": self.gyro_cal_done, "reset_gen": self.reset_gen, "vbat": self.vbat,
             "fc_tick": ms(self.t)}
        for k, _, fmt in FC_GLOBALS:
            out.append((self.FAKE_GLOBALS[k], struct.pack(fmt, g[k])))
        if self.rx_counters:
            for n, v in (("g_uart1_rx_overruns", self.rx_overruns), ("g_uart1_lines_ok", self.rx_ok),
                         ("g_uart1_lines_bad", self.rx_bad)):
                out.append((self.FAKE_RX[n], struct.pack("<I", v & 0xFFFFFFFF)))
        return out

    def peek(self, regions):
        with self.lock:
            self._advance()
            self._session_check("RBPOKE")
            self.peeks += 1
            mem = self._memory()
        res = []
        for addr, n in regions:
            buf = bytearray(n)
            for base, data in mem:
                lo, hi = max(addr, base), min(addr + n, base + len(data))
                if lo < hi:
                    buf[lo - addr:hi - addr] = data[lo - base:hi - base]
            res.append(bytes(buf))
        return res

    def fetch(self, out_prefix):
        with self.lock:
            self._advance()
            self._session_check("RBPOKE")
            rows = list(self.seg or [])
            snap = self.seg_snapshot or {"gain_seq": self.gain_seq, "gains": dict(self.gains)}
            changes = list(self.gain_log)
        return write_rig_recording(out_prefix, rows, snap, changes, axis=self.axis)

    def close(self):
        pass

    # ---- FC uplink ----
    def _nak(self, text, why):
        self.naks.append((self.t, text, why))

    def _fc_command(self, text):
        tok = text.split()
        if not tok:
            return
        v = tok[0]
        if v == "HB":
            self.last_hb = self.t
        elif v in ("COLL", "RSP", "RUNMS"):
            n = int(tok[1]) if len(tok) == 2 and re.match(r"^[+-]?[0-9]{1,9}$", tok[1]) else None
            if n is None:
                return self._nak(text, "%s: expects one integer" % v)
            if v == "COLL":
                if not 0 <= n <= COLL_MAX_PERMILLE:           # refused, not clamped
                    return self._nak(text, "COLL: out of range")
                self.coll_cmd = f32(n * 1e-3)
            elif v == "RSP":
                if abs(n) > SP_MAX_MDEG:
                    return self._nak(text, "RSP: out of range")
                if not tgt_within(rsp_rad(n), self.gains["ROLL_TRIM"], self._att_amp(), self.TGT_MAX):
                    return self._nak(text, "RSP: over RIG_TGT_MAX_RAD")
                self.sp_cmd = rsp_rad(n)
            else:
                self.max_run_ms = runms_applied(n)
        elif v == "GAIN":
            val = parse_decimal(tok[2]) if len(tok) == 3 else None
            if len(tok) != 3 or tok[1] not in GAIN_RANGE or val is None:
                self._nak(text, "GAIN: bad")
                return
            lo, hi = GAIN_RANGE[tok[1]]
            if not f32(lo) <= val <= f32(hi):
                self._nak(text, "GAIN: out of range")
                return
            if tok[1] == "ROLL_TRIM" and not tgt_within(self.sp_cmd, val, self._att_amp(), self.TGT_MAX):
                self._nak(text, "GAIN: ROLL_TRIM over RIG_TGT_MAX_RAD")
                return
            self.gains[tok[1]] = val
            self.gain_seq += 1
            self.gain_log.append({"gain_seq": self.gain_seq, "name": tok[1], "value": val,
                                  "t_us": self._t_us()})
            if self.law:
                self.law["att_int"] = self.law["rate_int"] = 0.0
        elif v == "EXC":
            self._fc_exc(text, tok)
        elif v == "EXCSTOP":
            self.exc = None
        elif v == "STOP":
            self.stop_req = True
        elif v == "ESTOP":
            self.estop = True
        elif v == "RESET":
            # main.cpp rose_cmd_reset + the soft reset + fc_rig_step's reset_gen edge (which keeps
            # coll_cmd, gains, RSP, RUNMS and, since 11-04, a pending STOP)
            self.reset_gen += 1
            self.arm_enabled = True
            self.estop = False
            self.rs, self.arm_since, self.coll = "IDLE", None, 0.0
            self.rd_reason, self.guard, self.exc = None, False, None
            self.law = None
            self.window_expired = False if not self.motor_live_since else self.window_expired
        elif v == "DISARM":
            self._to_done()
        else:
            self._nak(text, "unknown")

    def _fc_exc(self, text, tok):
        if len(tok) < 5 or tok[1] not in EXC_TYPES or tok[2] not in EXC_TARGETS:
            return self._nak(text, "EXC: syntax")
        try:
            nums = [int(x) for x in tok[3:]]
        except ValueError:
            return self._nak(text, "EXC: integers")
        amp, dur = nums[0], nums[1]
        f0 = nums[2] if len(nums) > 2 else 0
        f1 = nums[3] if len(nums) > 3 else f0
        if self.rs != "RUN":
            return self._nak(text, "EXC: only in RUN")
        if not self.guard:
            return self._nak(text, "EXC: not lifted off yet")
        si = amp * (1e-6 if tok[2] == "moment" else 1e-3)
        if abs(si) > self._exc_cap(tok[2]) * (1 + 1e-6):
            return self._nak(text, "EXC: amplitude over cap")
        if tok[2] == "att" and not tgt_within(max(self.sp_cmd, self.sp, key=abs), self.gains["ROLL_TRIM"],
                                              si, self.TGT_MAX):
            return self._nak(text, "EXC: over RIG_TGT_MAX_RAD")
        self.exc_id = (self.exc_id + 1) & 0xFFFFFFFF
        self.exc = {"type": tok[1], "target": tok[2], "amp": si, "dur": dur / 1000.0,
                    "f0": f0 / 1000.0, "f1": (f1 if tok[1] == "chirp" else f0) / 1000.0,
                    "t0": self.t, "reg": 0x1FF, "bit": 0}

    def _att_amp(self):
        return abs(self.exc["amp"]) if self.exc is not None and self.exc["target"] == "att" else 0.0

    def _exc_cap(self, target):
        if target == "att":
            return EXC_ATT_MAX_MRAD * 1e-3
        if target == "rate":
            return EXC_RATE_MAX_MRADPS * 1e-3
        return EXC_MOMENT_FRAC * 2 * 0.25 * self.coll * self.mass * GRAVITY * L_ARM

    def _exc_value(self):
        e = self.exc
        te = self.t - e["t0"]
        if te < 0 or te >= e["dur"]:
            return None
        T, A = e["dur"], e["amp"]

        def taper():
            tt = EXC_TAPER_FRAC * T
            if te < tt:
                return 0.5 - 0.5 * math.cos(math.pi * te / tt)
            if T - te < tt:
                return 0.5 - 0.5 * math.cos(math.pi * (T - te) / tt)
            return 1.0
        if e["type"] == "step":
            return A
        if e["type"] == "doublet":
            return A if te < T / 2 else -A
        if e["type"] == "sine":
            return A * taper() * math.sin(2 * math.pi * e["f0"] * te)
        if e["type"] == "chirp":
            f0, f1 = e["f0"], e["f1"]
            if abs(f1 - f0) < 1e-9:
                cyc = f0 * te
            else:
                lk = math.log(f1 / f0)
                cyc = f0 * T / lk * (math.exp(lk * te / T) - 1.0)
            return A * taper() * math.sin(2 * math.pi * cyc)
        idx = int(math.floor(te * e["f0"] + 1e-6))
        while e["bit"] < idx:
            r = e["reg"]
            e["reg"] = ((r << 1) | (((r >> 8) ^ (r >> 4)) & 1)) & 0x1FF
            e["bit"] += 1
        return A if e["reg"] & 1 else -A

    # ---- simulation ----
    def _t_us(self):
        return int((self.t - self.boot) * 1e6) & 0xFFFFFFFF

    def _to_done(self):
        self.rs = "DONE"
        self.coll = 0.0
        self.coll_cmd = 0.0          # fc_rig.cpp to_done(): arming again needs a fresh COLL
        self.arm_since = None
        self.exc = None

    def _event(self, text):
        self.events.append((self.t, text))

    def _advance(self):
        target = self.clock.now()
        while self.t + self.DT <= target:
            self.t += self.DT
            self._tick()

    def _tick(self):
        dt = self.DT
        self.tick += 1
        g = self.gains
        gyro = self.om + self.rng.gauss(0.0, 0.008)
        roll = self.th + self.rng.gauss(0.0, 0.0005)
        now = self.t
        armed_states = ("RAMP_UP", "RUN", "RAMP_DOWN")
        if self.estop:
            self.coll_cmd = 0.0      # fc_rig_step: a latched estop drops the commanded collective
        if self.rs in armed_states and (self.estop or not self.arm_enabled):
            self._to_done()
            self._event("estop: motors off")
        stop, self.stop_req = self.stop_req, False
        hb_age = (now - self.last_hb) if self.last_hb is not None else 1e9
        if self.rs in ("IDLE", "ARMING", "DONE"):
            if self.rs != "DONE":
                if stop:
                    self._to_done()
                else:
                    ready = (self.arm_enabled and self.gyro_cal_done and not self.estop
                             and abs(gyro) < ARM_MAX_RATE_RADPS
                             and abs(roll) < ARM_MAX_TILT_RAD and hb_age * 1000 <= HB_RAMP_MS
                             and self.coll_cmd > 0)
                    if not ready:
                        self.arm_since = None
                        self.rs = "IDLE"
                    else:
                        if self.arm_since is None:
                            self.arm_since = now
                        self.rs = "ARMING"
                        if now - self.arm_since >= ARM_SETTLE_MS / 1000.0:
                            self.rs = "RAMP_UP"
                            self.arm_t = self.phase_t = now
                            self.coll = 0.0
                            self.sp = roll
                            self.guard = False
                            self.tilt_since = None
                            self.rd_reason = None
                            self.law = {"att_int": 0.0, "rate_int": 0.0, "gx_f": gyro,
                                        "gx_prev": gyro, "dgx_f": 0.0}
                            self._event("armed: ramping up")
                            self._seg_start()
        else:
            why = None
            if hb_age * 1000 > HB_ESTOP_MS:
                why = "rig heartbeat"
            if not self.guard and abs(roll) < MAX_TILT_RAD - LIFTOFF_MARGIN_RAD:
                self.guard = True
                self._event("liftoff: roll guard live")
            if self.guard and abs(roll) > MAX_TILT_RAD:
                if self.tilt_since is None:
                    self.tilt_since = now
                if now - self.tilt_since >= 0.12 and why is None:
                    if self.rs == "RAMP_DOWN":
                        self._to_done()
                        self._event("ramp-down: on the stop, motors off")
                    else:
                        why = "rig tilt"
            else:
                self.tilt_since = None
            if why is not None:
                self._to_done()
                self.estop = True
                self._event("estop: " + why)
            elif self.rs != "DONE":
                if self.rs != "RAMP_DOWN":
                    rd = None
                    if stop:
                        rd = "RIG stop"
                    elif hb_age * 1000 > HB_RAMP_MS:
                        rd = "RIG heartbeat stale"
                    elif (now - self.arm_t) * 1000 > self.max_run_ms:
                        rd = "RIG max run time"
                    elif not self.guard and (now - self.arm_t) * 1000 > RAMP_MS + LIFTOFF_TIMEOUT_MS:
                        rd = "RIG liftoff timeout"
                    if rd:
                        self.rs, self.phase_t, self.coll_from = "RAMP_DOWN", now, self.coll
                        self.rd_reason, self.exc = rd, None
                        self._event(rd)
                tp = now - self.phase_t
                if self.rs == "RAMP_UP":
                    if tp >= RAMP_MS / 1000.0:
                        self.rs, self.phase_t, self.coll = "RUN", now, self.coll_cmd
                        self._event("run")
                    else:
                        self.coll = self.coll_cmd * tp / (RAMP_MS / 1000.0)
                elif self.rs == "RUN":
                    step = COLL_SLEW * dt
                    self.coll += max(-step, min(step, self.coll_cmd - self.coll))
                elif self.rs == "RAMP_DOWN":
                    if tp >= RAMP_MS / 1000.0:
                        self._to_done()
                        self._event("ramp-down complete: motors off")
                    else:
                        self.coll = self.coll_from * (1 - tp / (RAMP_MS / 1000.0))
                if self.rs != "DONE":
                    step = SP_SLEW_RADPS * dt
                    self.sp += max(-step, min(step, self.sp_cmd - self.sp))
        exc_val, exc_tgt = 0.0, None
        if self.rs == "RUN" and self.exc is not None:
            v = self._exc_value()
            if v is None:
                self.exc = None
            else:
                cap = self._exc_cap(self.exc["target"])
                exc_val, exc_tgt = max(-cap, min(cap, v)), self.exc["target"]
        elif self.rs != "RUN":
            self.exc = None
        armed = self.rs in armed_states
        u0 = self.coll * self.mass * GRAVITY if armed else 0.0
        moment = 0.0
        d = {"roll": roll, "roll_tgt": 0.0, "rate_tgt": 0.0, "rate_cmd": 0.0, "moment": 0.0,
             "att_int": 0.0, "rate_int": 0.0}
        if armed and self.law is not None:
            L = self.law
            L["gx_f"] += _lpf_alpha(dt, g["GYRO_LPF_HZ"]) * (gyro - L["gx_f"])
            dd = (L["gx_f"] - L["gx_prev"]) / dt
            L["dgx_f"] += _lpf_alpha(dt, g["D_LPF_HZ"]) * (dd - L["dgx_f"])
            L["gx_prev"] = L["gx_f"]
            tgt = self.sp + g["ROLL_TRIM"] + (exc_val if exc_tgt == "att" else 0.0)
            e = tgt - roll
            integ = self.rs == "RUN" and self.guard
            # fc_rig.cpp LIFT-OFF ASSIST: RUN before lift-off builds att_int at LIFT_I (kept at
            # lift-off) under a moment clamp that also counts as saturation
            assist = self.rs == "RUN" and not self.guard and g["LIFT_I"] > 0
            if self.rs == "RAMP_UP":
                L["att_int"] = L["rate_int"] = 0.0
            sat_pos, sat_neg = self.sat
            a_pos, a_neg = L.get("assist_sat", (False, False))

            def integrate(acc, inc, lim, sp=sat_pos, sn=sat_neg):
                if not ((inc > 0 and sp) or (inc < 0 and sn)):
                    acc += inc
                return max(-lim, min(lim, acc))
            if integ:
                L["att_int"] = integrate(L["att_int"], g["ATT_I"] * e * dt, g["ATT_I_LIM"])
            elif assist:
                L["att_int"] = integrate(L["att_int"], g["LIFT_I"] * e * dt,
                                         min(g["LIFT_I_LIM"], g["ATT_I_LIM"]),
                                         sat_pos or a_pos, sat_neg or a_neg)
            rate_tgt = g["ATT_P"] * e + L["att_int"] + (exc_val if exc_tgt == "rate" else 0.0)
            rerr = rate_tgt - L["gx_f"]
            if integ:
                L["rate_int"] = integrate(L["rate_int"], g["RATE_I"] * rerr * dt, g["RATE_I_LIM"])
            rate_cmd = g["RATE_P"] * rerr + L["rate_int"] - g["RATE_D"] * L["dgx_f"]
            moment = J_ROLL * g["ATT_GAIN"] * rate_cmd + (exc_val if exc_tgt == "moment" else 0.0)
            if assist:   # RIG_LIFT_MOMENT_FRAC 1.0 x fc_rig_moment_authority(u0)
                cap = 1.0 * 2 * 0.25 * u0 * L_ARM
                L["assist_sat"] = (moment >= cap, moment <= -cap)
                moment = max(-cap, min(cap, moment))
            else:
                L["assist_sat"] = (False, False)
            d.update(roll_tgt=tgt, rate_tgt=rate_tgt, rate_cmd=rate_cmd, moment=moment,
                     att_int=L["att_int"], rate_int=L["rate_int"])
        # mixer: +moment raises pos_pair (roll: the left pair duty[2], duty[3]; pitch: the rear
        # pair duty[1], duty[2]) and lowers neg_pair; MOTOR_TRIM_LEFT scales duty[2], duty[3]
        if armed and not self.kill and not self.window_expired:
            duty = [0.0] * 4
            for pair, sgn in ((self.pos_pair, 1.0), (self.neg_pair, -1.0)):
                for i in pair:
                    f = u0 / 4.0 + sgn * moment / (4.0 * L_ARM)
                    trim = g["MOTOR_TRIM_LEFT"] if i in (2, 3) else 1.0
                    duty[i] = min(self.CEILING, max(0.0, f / self.T_MAX * trim))
            self.duty = duty
        else:
            self.duty = [0.0] * 4
        eps = 1e-4
        hi = lambda pair: any(self.duty[i] >= self.CEILING - eps for i in pair)  # noqa: E731
        lo = lambda pair: any(self.duty[i] <= eps for i in pair)                 # noqa: E731
        self.sat = (hi(self.pos_pair) or lo(self.neg_pair), hi(self.neg_pair) or lo(self.pos_pair))
        if any(x > 0 for x in self.duty):
            self.last_nonzero = now
            if self.motor_live_since is None:
                self.motor_live_since = now
        if self.motor_live_since is not None and now - self.motor_live_since > 900.0:
            self.window_expired = True
        # plant
        m_cmd = self.L_EFF_moment()
        self.delay.append(m_cmd)
        m_in = self.delay.popleft()
        self.m_act += (m_in - self.m_act) * dt / self.MOTOR_TAU
        acc = (self.m_act - self.C_DAMP * self.om + self.K_UB * math.sin(self.th)) / self.J_PLANT
        self.om += acc * dt
        self.th += self.om * dt
        if self.th > self.STOP_RAD:
            self.th, self.om = self.STOP_RAD, min(0.0, self.om)
        elif self.th < -self.STOP_RAD:
            self.th, self.om = -self.STOP_RAD, max(0.0, self.om)
        # recorder
        if self.tick % self.REC_DIV == 0:
            self._record(gyro, roll, d, exc_val, u0)

    def L_EFF_moment(self):
        """Moment about the pivot the motors actually make from the duties (the pair +moment
        raises is 1.5 % weak: the left pair on roll, the rear pair on pitch)."""
        tl = sum(self.duty[i] for i in self.pos_pair) * self.T_MAX * self.LEFT_EFF
        tr = sum(self.duty[i] for i in self.neg_pair) * self.T_MAX
        return (tl - tr) * L_ARM

    def _seg_start(self):
        self.seg = list(self.pre)
        self.seg_snapshot = {"gain_seq": self.gain_seq, "gains": dict(self.gains)}
        self.seg_frozen = False
        self.idle_since = None

    def _record(self, gyro, roll, d, exc_val, u0):
        armed = self.rs in ("RAMP_UP", "RUN", "RAMP_DOWN")
        flags = (1 if armed else 0) | (2 if self.estop else 0) | \
            (4 if self.rs == "ARMING" else 0) | 8 | (0x40 if self.committed else 0)
        # `roll`/`gyro` are the controlled axis (the law's roll slots); the recorder's roll, pitch
        # and gyro p/q are physical, so a pitch build logs them swapped (roll locked near 0)
        other, other_rate = self.rng.gauss(0.0, 0.0005), self.rng.gauss(0, 0.005)
        if self.axis == AX.PITCH:
            r_phys, p_phys, gp, gq = other, roll, other_rate, gyro
            ay = 0.0
        else:
            r_phys, p_phys, gp, gq = roll, 0.0, gyro, other_rate
            ay = GRAVITY * math.sin(self.th)
        row = (self._t_us(), int(self.DT * self.REC_DIV * 1e6), (self.tick // self.REC_DIV * self.REC_DIV) & 0xFF,
               flags, RS_NAMES.index(self.rs), self.exc_id & 0xFF, self.gain_seq & 0xFFFF,
               gp, gq, self.rng.gauss(0, 0.005), r_phys, p_phys,
               d["roll_tgt"], d["rate_tgt"], d["rate_cmd"], d["moment"], exc_val, u0,
               d["att_int"], d["rate_int"], 0.0, 0.0, ay, GRAVITY * math.cos(self.th),
               7.6, [x for x in self.duty])
        self.pre.append(row)
        if self.seg is not None and not self.seg_frozen:
            self.seg.append(row)
            if armed:
                self.idle_since = None
            elif self.idle_since is None:
                self.idle_since = self.t
            elif self.t - self.idle_since >= FLIGHTREC_TAIL_S:
                self.seg_frozen = True


# ---- recorder v3 (rig) CSV, as flightrec_fetch.fetch writes it -----------------------------------

REC_FLAGS = ((0x01, "armed"), (0x02, "estop"), (0x04, "arming"), (0x08, "caldone"),
             (0x10, "tof_valid"), (0x20, "flow_valid"), (0x40, "committed"))
REC_COLUMNS = (["t_s", "t_us", "dt_us", "seq", "flags"] + [n for _, n in REC_FLAGS]
               + ["rig_state", "rs_name", "exc_id", "gain_seq", "gyro_p", "gyro_q", "gyro_r",
                  "roll_deg", "pitch_deg", "roll_tgt_deg", "rate_tgt_dps", "rate_cmd", "moment_mNm",
                  "exc_val", "coll_thrust_N", "att_int", "rate_int", "pitch_cmd", "yaw_cmd",
                  "accel_y", "accel_z", "vbat", "duty0", "duty1", "duty2", "duty3",
                  "axis", "ang_deg", "ang_tgt_deg", "ang_rate_dps"])
_REC_PACK = struct.Struct("<IHBBBBH18f4H")


def write_rig_recording(out_prefix, rows, snapshot, changes, axis=0):
    """Write PREFIX.csv / .gains.json / .bin from fake recorder rows (FakeDrone._record tuples).
    axis: the build's (0 roll, 1 pitch) -- the generic ang_* columns follow it."""
    axis = AX.PITCH if AX.parse_axis(axis) == AX.PITCH else AX.ROLL
    out_prefix = str(out_prefix)
    Path(out_prefix).parent.mkdir(parents=True, exist_ok=True)
    first_motor = next((r[0] for r in rows if any(x > 0 for x in r[-1])), None)
    ref = first_motor if first_motor is not None else (rows[0][0] if rows else 0)
    gaps = 0
    with open(out_prefix + ".bin", "wb") as fb, \
            open(out_prefix + ".csv", "w", newline="", encoding="utf-8") as fc:
        w = csv.writer(fc)
        w.writerow(REC_COLUMNS)
        prev = None
        for r in rows:
            (t_us, dt_us, seq, flags, rs, exc_id, gs, gp, gq, gr, roll, pitch, rtgt, rate_tgt,
             rate_cmd, moment, exc_val, coll, att_int, rate_int, pcmd, ycmd, ay, az, vb, duty) = r
            if prev is not None and ((seq - prev) & 0xFF) != FakeDrone.REC_DIV:
                gaps += 1
            prev = seq
            fb.write(_REC_PACK.pack(t_us, dt_us, seq, flags, rs, exc_id, gs, gp, gq, gr, roll, pitch,
                                    rtgt, rate_tgt, rate_cmd, moment, exc_val, coll, att_int,
                                    rate_int, pcmd, ycmd, ay, az, vb,
                                    *[int(round(x * 10000)) for x in duty]))
            t_rel = (((t_us - ref) & 0xFFFFFFFF) ^ 0x80000000) - 0x80000000
            w.writerow(["%.6f" % (t_rel / 1e6), t_us, dt_us, seq, flags]
                       + [1 if flags & b else 0 for b, _ in REC_FLAGS]
                       + [rs, RS_NAMES[rs], exc_id, gs, "%.5f" % gp, "%.5f" % gq, "%.5f" % gr,
                          "%.4f" % math.degrees(roll), "%.4f" % math.degrees(pitch),
                          "%.4f" % math.degrees(rtgt), "%.3f" % math.degrees(rate_tgt),
                          "%.5f" % rate_cmd, "%.5f" % (moment * 1e3), "%.6g" % exc_val,
                          "%.5f" % coll, "%.5f" % att_int, "%.5f" % rate_int, "%.5f" % pcmd,
                          "%.5f" % ycmd, "%.4f" % ay, "%.4f" % az, "%.3f" % vb]
                       + ["%.4f" % x for x in duty]
                       + [AX.letter(axis),
                          "%.4f" % math.degrees(pitch if axis == AX.PITCH else roll),
                          "%.4f" % math.degrees(rtgt),
                          "%.4f" % math.degrees(gq if axis == AX.PITCH else gp)])
    gj = {"snapshot": {"gain_seq": snapshot["gain_seq"], "gains": dict(snapshot["gains"])},
          "changes": [dict(c) for c in changes], "axis": AX.name(axis)}
    with open(out_prefix + ".gains.json", "w", encoding="utf-8") as f:
        json.dump(gj, f, indent=1, sort_keys=True)
    return {"csv": out_prefix + ".csv", "bin": out_prefix + ".bin",
            "gains": out_prefix + ".gains.json", "samples": len(rows), "gaps": gaps, "layout": 2,
            "axis": AX.name(axis), "axis_source": "fake drone"}


# =============================================================================
# Recording -> trials
# =============================================================================

def read_rec_csv(path):
    """-> (header, rows as lists of strings)."""
    with open(path, "r", encoding="utf-8", errors="replace", newline="") as f:
        rd = csv.reader(f)
        header = next(rd, [])
        rows = [r for r in rd if r]
    return header, rows


def gains_at(gains_json, gain_seq):
    """The full gain table in effect at gain_seq, from PREFIX.gains.json (None if unknown)."""
    if not gains_json:
        return None
    snap = gains_json.get("snapshot") or {}
    g = dict(snap.get("gains") or {})
    s0 = snap.get("gain_seq", 0)
    if gain_seq < s0:
        return None
    by_seq = {c.get("gain_seq"): c for c in gains_json.get("changes") or []}
    for s in range(s0 + 1, gain_seq + 1):
        c = by_seq.get(s)
        if c is None:
            return None          # that change fell out of the 64-entry ring
        g[c.get("name")] = c.get("value")
    return g


def _u8_after(a, b):
    """exc_id is a u8 in the recorder: is a later than b (by < 128)?"""
    return 0 < ((a - b) & 0xFF) < 128


def segment_spin(header, rows, sent, gains_json=None):
    """Cut one spin's recording into trials.

    sent: the trials whose EXC went out (ESP ACK or not), in send order, each {"trial", "gains",
    "exc"}. The k-th new exc_id seen in the recording is the k-th accepted EXC; each mapping is
    then checked (gains at that gain_seq, excitation length), so a NAKed or lost EXC shows up as a
    mismatch instead of shifting every metric onto the wrong gains.

    -> [{"trial", "lo", "hi", "exc_id", "gain_seq", "t_exc", "matched", "reason"}]
    """
    col = {n: i for i, n in enumerate(header)}
    need = ("t_s", "exc_id", "gain_seq", "exc_val")
    if any(n not in col for n in need) or not rows:
        return [{"trial": s["trial"], "lo": 0, "hi": 0, "exc_id": None, "gain_seq": None,
                 "t_exc": None, "matched": False, "reason": "recording empty or columns missing"}
                for s in sent]
    rs_col = col.get("rig_state")
    t = [float(r[col["t_s"]]) for r in rows]
    eid = [int(float(r[col["exc_id"]])) for r in rows]
    gsq = [int(float(r[col["gain_seq"]])) for r in rows]
    xv = [float(r[col["exc_val"]]) for r in rows]
    run = [int(float(r[rs_col])) == 3 for r in rows] if rs_col is not None else [True] * len(rows)
    starts = []
    for i in range(1, len(rows)):
        if eid[i] != eid[i - 1] and _u8_after(eid[i], eid[i - 1]):
            starts.append(i)
    dt = (t[-1] - t[0]) / max(1, len(t) - 1) if len(t) > 1 else 0.004
    pre_n = int(round(PRE_WINDOW_S / dt)) if dt > 0 else 0
    out = []
    for k, s in enumerate(sent):
        if k >= len(starts):
            out.append({"trial": s["trial"], "lo": 0, "hi": 0, "exc_id": None, "gain_seq": None,
                        "t_exc": None, "matched": False,
                        "reason": "no exc_id change for this EXC in the recording (NAKed or lost)"})
            continue
        i0 = starts[k]
        nxt = starts[k + 1] if k + 1 < len(starts) else len(rows)
        hi = i0
        while hi < nxt and run[hi] and gsq[hi] == gsq[i0]:
            hi += 1
        lo = i0
        while lo > 0 and i0 - lo < pre_n and gsq[lo - 1] == gsq[i0] and eid[lo - 1] == eid[i0 - 1] \
                and run[lo - 1]:
            lo -= 1
        reason = None
        g_rec = gains_at(gains_json, gsq[i0])
        if g_rec is not None:
            bad = [n for n, v in s["gains"].items()
                   if g_rec.get(n) is not None and not math.isclose(f32(v), f32(g_rec[n]), rel_tol=1e-6, abs_tol=1e-7)]
            if bad:
                reason = "gains in the recording differ from the plan: %s" % ", ".join(
                    "%s=%s (plan %s)" % (n, g_rec[n], s["gains"][n]) for n in bad)
        dur = s["exc"]["dur_ms"] / 1000.0
        on = [t[i] for i in range(i0, hi) if xv[i] != 0.0]
        if reason is None and dur >= 0.1:
            span = (on[-1] - on[0] + dt) if on else 0.0
            if span < 0.5 * dur or span > dur + 0.25:
                reason = "excitation lasted %.2f s in the recording, planned %.2f s" % (span, dur)
        if reason is None and hi - i0 < 2:
            reason = "the run left RUN at the excitation"
        out.append({"trial": s["trial"], "lo": lo, "hi": hi, "exc_id": eid[i0], "gain_seq": gsq[i0],
                    "t_exc": t[i0], "matched": reason is None, "reason": reason})
    return out


# =============================================================================
# Threads: heartbeat, RBSTAT poller, status.json writer
# =============================================================================

class Heartbeat(threading.Thread):
    """HB at 20 Hz, drift-free, no file I/O, sent through the Uplink with priority (so a command
    delays it by at most one --tx-gap-ms). Runs from just before RESET until the motors are
    confirmed off (RBSTAT), including all of RAMP_DOWN."""

    def __init__(self, link, clock):
        super().__init__(name="rig-hb", daemon=True)
        self.link, self.clock = link, clock
        self.stop_evt = threading.Event()
        self.count = 0
        self.errors = 0
        self.max_gap = 0.0
        self.last = None

    def run(self):
        nxt = self.clock.now()
        while not self.stop_evt.is_set():
            try:
                if self.link.hb():
                    self.count += 1
                    now = self.clock.now()
                    if self.last is not None:
                        self.max_gap = max(self.max_gap, now - self.last)
                    self.last = now
                else:
                    self.errors += 1
            except Exception:  # noqa: BLE001 - keep beating whatever happened
                self.errors += 1
            nxt += HB_PERIOD_S
            delay = nxt - self.clock.now()
            if delay < -0.5:
                nxt = self.clock.now()
                delay = 0.0
            self.clock.wait(self.stop_evt, delay)

    def stop(self):
        self.stop_evt.set()
        if self.is_alive() and threading.current_thread() is not self:
            self.join(timeout=2.0)


class StatPoller(threading.Thread):
    """The only caller of RBSTAT1 while a batch runs (the ESP's TCP service takes one connection
    at a time). Paused around POKE/fetch sessions via `session_lock`."""

    def __init__(self, link, clock, interval=1.0):
        super().__init__(name="rig-stat", daemon=True)
        self.link, self.clock = link, clock
        self.interval = interval
        self.stop_evt = threading.Event()
        self.kick = threading.Event()
        self.session_lock = threading.Lock()
        self.cond = threading.Condition()
        self.latest = None        # (t_obs, line, parsed)
        self.errors = 0
        self.last_error = None
        self.enabled = threading.Event()
        self.enabled.set()

    def run(self):
        while not self.stop_evt.is_set():
            if self.enabled.is_set():
                with self.session_lock:
                    try:
                        line = self.link.stat()
                        parsed = T.parse_rblink(line)
                        with self.cond:
                            self.latest = (self.clock.now(), line, parsed)
                            self.cond.notify_all()
                    except Exception as exc:  # noqa: BLE001
                        self.errors += 1
                        self.last_error = str(exc)
            self.kick.clear()
            # wake early on a kick (a caller wants a fresh line)
            deadline = self.clock.now() + self.interval
            while not self.stop_evt.is_set() and not self.kick.is_set():
                left = deadline - self.clock.now()
                if left <= 0:
                    break
                self.clock.wait(self.kick, min(left, 0.25))

    def fresh(self, after, timeout):
        """A stat line observed after clock time `after`, or None within `timeout` (clock s)."""
        end = self.clock.now() + timeout
        self.kick.set()
        while True:
            with self.cond:
                if self.latest is not None and self.latest[0] > after:
                    return self.latest
            if self.clock.now() >= end or self.stop_evt.is_set():
                return None
            self.kick.set()
            self.clock.sleep(0.05)

    def stop(self):
        self.stop_evt.set()
        self.kick.set()
        if self.is_alive():
            self.join(timeout=5.0)


class StatusWriter(threading.Thread):
    def __init__(self, path, snapshot_fn, period=0.5):
        super().__init__(name="rig-status", daemon=True)
        self.path, self.snapshot_fn, self.period = path, snapshot_fn, period
        self.stop_evt = threading.Event()

    def write(self):
        try:
            T.write_json_atomic(self.path, self.snapshot_fn())
        except Exception:  # noqa: BLE001 - status is advisory
            pass

    def run(self):
        while not self.stop_evt.is_set():
            self.write()
            self.stop_evt.wait(self.period)

    def stop(self):
        self.stop_evt.set()
        if self.is_alive():
            self.join(timeout=2.0)
        self.write()


# =============================================================================
# The runner
# =============================================================================

class AbortRequested(Exception):
    pass


class EstopSeen(Exception):
    def __init__(self, reason, source):
        super().__init__(reason)
        self.reason, self.source = reason, source


class FcStopped(Exception):
    """The motors stopped without a STOP from us and without an estop flag (max run, liftoff
    timeout, heartbeat stale, tilt on the ramp-down)."""


class LinkLost(Exception):
    pass


class ArmFailed(Exception):
    pass


class UplinkUnverified(Exception):
    """RBPOKE shows the FC did not apply what was sent, after the resends: the spin is refused
    before any RESET (no motors, no ESTOP needed)."""


class SpinRefused(Exception):
    """A check right before arming failed; the spin is refused before RESET."""


class SignalExit(BaseException):
    """SIGTERM/SIGHUP: handled exactly like Ctrl-C."""


def sha1_file(path):
    h = hashlib.sha1()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def make_batch_dir(base, name, now=None):
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(now or time.time()))
    d = Path(base) / ("%s-%s" % (stamp, name))
    n = 1
    while d.exists():
        n += 1
        d = Path(base) / ("%s-%s-%d" % (stamp, name, n))
    (d / "trials").mkdir(parents=True)
    (d / "spins").mkdir()
    return d


class Runner:
    def __init__(self, spec, batch_dir, link, clock, *, approval, mode="rig-roll", elf=None,
                 dry_run=False, verify_poke=True, commit_used_s=0.0, clear_estop=False,
                 analyze=True, log=None, tx_gap_ms=TX_GAP_MS, limits=None, axis_source=None):
        self.spec = spec
        self.axis = AX.parse_axis(spec.get("axis", "roll")) or AX.ROLL
        self.axis_source = axis_source
        self.dir = Path(batch_dir)
        self.link, self.clock = link, clock
        self.approval, self.mode, self.elf = approval, mode, elf
        self.dry_run = dry_run
        self.verify_poke = True        # always: the old flag is accepted and ignored
        self.tx_gap_ms = float(tx_gap_ms)
        self.up = Uplink(link, clock, self.tx_gap_ms / 1000.0)
        self.fc_limits = limits or spec.get("fc_limits")
        self.layout = None          # RigLayout, from the link on the first RBPOKE read
        self.rig = None             # the last g_rig read the FC was proven to hold
        self.exp = None
        self.rx_before = None
        self.spin_armed = False
        self.clear_estop = clear_estop
        self.analyze = analyze
        self.log = log or (lambda s: print(s, flush=True))
        self.journal = self.dir / "journal.jsonl"
        self.lock = threading.Lock()
        self.trials = plan_trials(spec)
        self.queue = list(self.trials)
        self.n_trials = len(self.trials)
        self.phase = "starting"
        self.cur_trial = None
        self.cur = {}
        self.paused = False
        self.pause_req = False
        self.abort_req = False
        self.ctl_seq = 0
        self.ctl_mtime = None
        self.ctl_next = 0.0
        self.j_pos = 0
        self.motor_s = 0.0
        self.commit_used_s = float(commit_used_s)
        self.first_spin = None
        self.spin_n = 0
        self.hb = None
        self.poller = None
        self.status = None
        self.armed_from = None
        self.estopped = False
        self.burst_sent = False
        self.fc_gains = None        # what we believe the FC holds
        self.result = None
        self.fetch_eta = None

    # ---- journal / status ----
    def j(self, ev, **kw):
        obj = {"t": round(self.clock.wall(), 3), "ev": ev}
        obj.update(kw)
        T.append_journal(self.journal, obj)
        return obj

    def note(self, text):
        self.log("rig run: " + text)
        self.j("note", text=text)

    def commit_left_s(self):
        win = self.spec["limits"]["commit_window_s"]
        used = self.commit_used_s + (self.clock.now() - self.first_spin if self.first_spin else 0.0)
        return win - used

    def snapshot(self):
        with self.lock:
            budget = self.spec["limits"]["motor_budget_s"]
            motor = self.motor_s
            if self.armed_from is not None:
                motor += max(0.0, self.clock.now() - self.armed_from)
            return {"phase": self.phase, "trial": self.cur_trial, "n_trials": self.n_trials,
                    "current": dict(self.cur), "motor_ms": int(motor * 1000),
                    "motor_budget_ms": int(budget * 1000),
                    "commit_left_ms": int(max(0.0, self.commit_left_s()) * 1000),
                    "paused": self.paused, "updated": round(self.clock.wall(), 3),
                    "spin": self.spin_n, "dry_run": self.dry_run,
                    "fetch_eta_s": self.fetch_eta,
                    "hb": self.hb.count if self.hb else 0}

    def set_phase(self, phase):
        with self.lock:
            self.phase = phase

    def state(self, rs):
        self.j("state", rs=rs, src="host-timing")

    # ---- commands ----
    def cmd(self, text, retries=2, journal=True, priority=False, busy_only=False):
        """Send a line through the paced uplink; confirm with the ESP's ACK (the ESP took the
        datagram -- whether the FC applied it only RBPOKE can tell). Retries only where repeating
        is harmless; busy_only: only on BUSY (= not delivered), never on silence, which may be a
        lost ACK of a delivered line (a repeated GAIN would bump gain_seq twice)."""
        if len(text) > LINE_MAX:
            raise RuntimeError("internal: command too long: %r" % text)
        reply = None
        for _ in range(retries + 1):
            reply = self.up.cmd(text, priority=priority)
            if reply is not None and reply.startswith("ACK"):
                break
            if reply is None and busy_only:
                break
            if reply is not None and reply.startswith("BUSY"):
                self.clock.sleep(0.4)
        if journal:
            self.j("cmd", text=text, reply=reply or "(no ack)")
        return reply is not None and reply.startswith("ACK")

    def must(self, text, retries=2, priority=False):
        if not self.cmd(text, retries=retries, priority=priority):
            raise LinkLost("no ESP ACK for %r" % text)

    def emergency(self, reason):
        """ESTOP x3 first (dedicated socket), then the bookkeeping. Never raises. One burst per
        failure: the spin's handler and the batch's both call this on the way out."""
        if self.burst_sent:
            return
        self.burst_sent = True
        try:
            self.up.estop_burst(3)
        except Exception:  # noqa: BLE001
            pass
        self.estopped = True
        try:
            self.j("estop", reason=reason, source="host")
        except Exception:  # noqa: BLE001
            pass

    # ---- waiting: the one place control.json, the TUI's estops and the drone are watched ----
    def poll_control(self):
        now = time.monotonic()
        if now < self.ctl_next:
            return
        self.ctl_next = now + 0.1
        c = T.read_control(self.dir)
        if c["seq"] > self.ctl_seq:
            self.ctl_seq = c["seq"]
            if c["abort"] and not self.abort_req:
                self.abort_req = True
                self.note("abort requested via control.json (seq %d)" % c["seq"])
            if c["pause"] != self.pause_req:
                self.pause_req = c["pause"]
                self.note("%s requested via control.json (seq %d)"
                          % ("pause" if c["pause"] else "resume", c["seq"]))
        # estop events someone else (the TUI) appended to the journal
        try:
            size = os.path.getsize(self.journal)
        except OSError:
            return
        if size > self.j_pos:
            with open(self.journal, "rb") as f:
                f.seek(self.j_pos)
                data = f.read(size - self.j_pos)
            self.j_pos = size
            for line in data.split(b"\n"):
                try:
                    ev = json.loads(line)
                except ValueError:
                    continue
                if isinstance(ev, dict) and ev.get("ev") == "estop" and ev.get("source") not in ("host", "fc"):
                    raise EstopSeen(str(ev.get("reason", "operator")), ev.get("source", "tui"))

    def wait(self, seconds, armed=False, abortable=True, until=None, watch=True):
        """Sleep `seconds` of clock time (or until clock time `until`), watching everything."""
        end = until if until is not None else self.clock.now() + seconds
        checked = self.clock.now()
        while True:
            if watch:
                self.poll_control()
            if abortable and self.abort_req:
                raise AbortRequested()
            if armed:
                age = self.link.hb_ack_age()
                if self.hb is None or not self.hb.is_alive():
                    raise LinkLost("heartbeat thread died")
                if age is not None and age > HB_ACK_LOST_S:
                    raise LinkLost("no ESP ACK to HB for %.1f s" % age)
                lat = self.poller.latest if self.poller else None
                if lat is not None and lat[0] > checked and lat[2] is not None:
                    checked = lat[0]
                    self.check_stat(lat[2], expect_armed=True)
            if self.clock.now() >= end:
                return
            self.clock.sleep(min(0.05, max(0.0, end - self.clock.now())))

    def check_stat(self, st, expect_armed):
        fl = st["flags"]
        if fl & 0x02 or st["kill"]:
            raise EstopSeen("RBSTAT shows %s" % ("ESTOP" if fl & 0x02 else "the ESP kill latch"), "fc")
        if fl & 0x10:
            raise EstopSeen("RBSTAT shows WINDOW-EXPIRED: the commit window is used up", "fc")
        if st["ml"] != "up":
            raise LinkLost("RBSTAT: motor link %s" % st["ml"])
        if expect_armed and not fl & 0x01:
            raise FcStopped("RBSTAT shows not ARMED (duty %s) before STOP" % st["duty"])

    # ---- the batch ----
    def write_batch_json(self):
        spec = self.spec
        meta = {"name": spec["name"], "created": round(self.clock.wall(), 3), "mode": self.mode,
                "axis": AX.name(self.axis), "axis_source": self.axis_source,
                "elf": str(self.elf) if self.elf else None,
                "elf_sha1": sha1_file(self.elf) if self.elf and Path(self.elf).is_file() else None,
                "spec": {"axis": AX.name(self.axis), "collective_permille": spec["collective_permille"],
                         "roll_sp_mdeg": spec["roll_sp_mdeg"], "grid": spec["grid"],
                         "excitations": spec["excitations"], "repeats": spec["repeats"],
                         "limits": {k: spec["limits"][k] for k in
                                    ("max_trials", "max_coll_permille", "max_run_ms", "motor_budget_s")},
                         "arrangement": spec["arrangement"], "settle_ms": spec["limits"]["settle_ms"],
                         "commit_window_s": spec["limits"]["commit_window_s"],
                         "mass_kg": spec["mass_kg"], "base_gains": spec["base_gains"]},
                "approval_note": self.approval, "dry_run": self.dry_run,
                "verify": "rbpoke g_rig: baseline, gains, params, COLL before RESET",
                "tx_gap_ms": self.tx_gap_ms, "fc_limits": self.fc_limits,
                "coll_survives_reset": COLL_SURVIVES_RESET, "host": socket.gethostname(),
                "trials": [{"trial": t["trial"], "point": t["point"], "exc_index": t["exc_index"],
                            "rep": t["rep"]} for t in self.trials]}
        if "grid_product" in spec:
            meta["spec"]["grid_product"] = spec["grid_product"]
        T.write_json_atomic(self.dir / "batch.json", meta)
        T.write_json_atomic(self.dir / "control.json", {"pause": False, "abort": False, "seq": 0})

    def run(self):
        """-> batch status string. Exceptions are recorded, ESTOPped and re-raised."""
        self.write_batch_json()
        self.j("batch_start", n_trials=self.n_trials, dry_run=self.dry_run, axis=AX.name(self.axis),
               mode=self.mode)
        self.status = StatusWriter(self.dir / "status.json", self.snapshot)
        self.status.start()
        self.poller = StatPoller(self.link, self.clock)
        self.poller.start()
        status = "error"
        try:
            status = self._run()
        except (KeyboardInterrupt, SignalExit) as exc:
            self.emergency("operator interrupt (%s)" % type(exc).__name__)
            self._end_open_trial("estop")
            status = "interrupted"
            raise
        except BaseException as exc:
            self.emergency("runner exception: %s: %s" % (type(exc).__name__, exc))
            self._end_open_trial("estop")
            try:
                self.j("note", text="runner exception:\n" + traceback.format_exc()[-1500:])
            except Exception:  # noqa: BLE001
                pass
            status = "error"
            raise
        finally:
            self._shutdown(status)
        return status

    def _end_open_trial(self, status):
        if self.cur_trial is not None:
            try:
                self.j("trial_end", trial=self.cur_trial, status=status)
            except Exception:  # noqa: BLE001
                pass
            self.cur_trial = None

    def _shutdown(self, status):
        # STOP on every exit path: harmless when disarmed (IDLE -> DONE, DONE stays DONE).
        try:
            self.up.cmd("STOP", priority=True)
        except Exception:  # noqa: BLE001
            pass
        if self.hb is not None:
            if status in ("error", "interrupted") or self.estopped:
                self.hb.stop()           # ESTOP already cut the motors
            else:
                # a clean path reaches here with the motors confirmed off; belt and braces
                self.hb.stop()
        try:
            self.up.cmd("COLL 0")
        except Exception:  # noqa: BLE001
            pass
        self.set_phase("done")
        self.result = status
        try:
            self.j("batch_end", status=status, motor_ms=int(self.motor_s * 1000),
                   trials_done=self.n_trials - len(self.queue))
        except Exception:  # noqa: BLE001
            pass
        if self.poller:
            self.poller.stop()
        if self.status:
            self.status.stop()

    def _run(self):
        try:
            return self._run_batch()
        except EstopSeen as exc:
            # outside a spin's handler: while paused, between spins, in a precheck
            if exc.source == "fc":
                self.j("estop", reason=exc.reason, source="fc")
            self.estopped = True
            self.note("estop (%s): %s; the batch ends without RESET" % (exc.source, exc.reason))
            return "estop"

    def _run_batch(self):
        self.set_phase("precheck")
        st = self.poller.fresh(-1e18, 15.0)
        if st is None or st[2] is None:
            raise RuntimeError("no RBSTAT1 answer from the ESP (%s): is the drone's SoftAP joined "
                               "and esp_bridge running?" % (self.poller.last_error or "timeout"))
        self.precheck(st[2], first=True)
        self.note("precheck OK: %s" % st[1].strip()[:160])
        while self.queue:
            self.poll_control()
            if self.abort_req:
                return "aborted"
            if self.pause_req:
                self.set_phase("paused")
                with self.lock:
                    self.paused = True
                while self.pause_req and not self.abort_req:
                    self.wait(0.25, abortable=False)
                    if self.first_spin and self.commit_left_s() < COMMIT_MARGIN_S:
                        break
                with self.lock:
                    self.paused = False
                continue
            motor_left = self.spec["limits"]["motor_budget_s"] - self.motor_s
            spin, why = take_spin(self.spec, self.queue, motor_left,
                                  self.commit_left_s() - COMMIT_MARGIN_S)
            if not spin:
                if why == "budget":
                    self.note("motor budget: %.1f s used of %.0f s; the next spin needs more. Stopping."
                              % (self.motor_s, self.spec["limits"]["motor_budget_s"]))
                    return "budget"
                if why == "commit_window":
                    self.note("commit window: %.0f s left, the next spin needs more. Stopping."
                              % self.commit_left_s())
                    return "commit_window"
                raise RuntimeError("internal: an empty spin (%s)" % why)
            res = self.do_spin(spin)
            if res != "ok":
                return res
        return "ok"

    def precheck(self, st, first=False):
        problems = []
        if st["ml"] != "up":
            problems.append("motor link is %s (ml=up needed)" % st["ml"])
        if st["flags"] & 0x01 or any(st["duty"]):
            problems.append("motors are ARMED or carry duty %s" % st["duty"])
        if st["flags"] & 0x10:
            problems.append("WINDOW-EXPIRED: the commit window of this load is used up; reload")
        if not st["flags"] & 0x20:
            problems.append("not COMMITTED: commit the running rig ELF first (esp_tsi_load --commit "
                            "--elf ...); uncommitted motors latch off 5 s after the first spin")
        if (st["flags"] & 0x02 or st["kill"]) and not (first and self.clear_estop):
            why = "an ESTOP is latched (%s). Find out why; --clear-estop lets the first RESET " \
                "of a new batch clear it" % ("FC" if st["flags"] & 0x02 else "ESP kill")
            if not first or not problems:
                raise EstopSeen(why, "fc")
            problems.append(why)
        if problems:
            raise ArmFailed("refusing to spin: " + "; ".join(problems))

    # ---- one spin ----
    def do_spin(self, trials):
        self.spin_n += 1
        k = self.spin_n
        sdir = self.dir / "spins" / str(k)
        sdir.mkdir(parents=True, exist_ok=True)
        est = spin_estimate(self.spec, trials)
        runms = int(min(self.spec["limits"]["max_run_ms"],
                        max(RUNMS_MIN, math.ceil((est["live_s"] + RUN_MARGIN_S) * 1000))))
        T.write_json_atomic(sdir / "spin.json", {
            "spin": k, "trials": [t["trial"] for t in trials], "runms": runms,
            "estimate": est, "plan": [{"trial": t["trial"], "gains": t["gains"], "exc": t["exc"]}
                                      for t in trials]})
        self.note("spin %d: trials %s, est. %.1f s motor time, RUNMS %d"
                  % (k, ",".join(str(t["trial"]) for t in trials), est["motor_s"], runms))
        sent = []
        status = "ok"
        runs = point_runs(trials)
        self.spin_armed = False
        # 1. disarmed: fresh stat; STOP, COLL 0, the gain point, RUNMS, RSP, each set proven
        #    applied over RBPOKE (no FC reply reaches this host; the ESP ACKs datagrams only)
        self.set_phase("gains")
        st = self.poller.fresh(self.clock.now(), 10.0)
        if st is None or st[2] is None:
            raise LinkLost("no RBSTAT before spin %d" % k)
        self.precheck(st[2], first=(k == 1))
        self.hb = None
        try:
            try:
                if k == 1 and self.clear_estop and (st[2]["flags"] & 0x02 or st[2]["kill"]):
                    self.clear_latched_estop()
                self.setup_disarmed(runs[0][0]["gains"], runms)
                # 2. COLL proven, heartbeat, RBSTAT re-read, RESET (arm_sequence has the order)
                t_arm = self.arm_sequence(k)
                self.set_phase("arming")
                self.state("ARMING")
                status = self._spin_body(runs, sent, t_arm)
            except (UplinkUnverified, SpinRefused) as exc:
                status = "uplink_unverified" if isinstance(exc, UplinkUnverified) else "refused"
                self.note("spin %d refused before RESET: %s" % (k, exc))
                self.cmd("STOP", retries=2, priority=True)    # DONE: zeroes coll_cmd too
            except AbortRequested:
                self._end_open_trial("aborted")
                self.cmd("EXCSTOP", retries=1)
                status = "aborted"
            except FcStopped as exc:
                self._end_open_trial("timeout")
                self.note("FC stopped the motors by itself: %s (max run, liftoff timeout, stale HB "
                          "or the tilt guard)" % exc)
                status = "fc_stopped"
            except ArmFailed as exc:
                self.note(str(exc))
                status = "arm_failed"
            except LinkLost as exc:
                self.emergency("link lost: %s" % exc)
                self._end_open_trial("estop")
                status = "link_lost"
            except EstopSeen as exc:
                if exc.source == "fc":
                    self.j("estop", reason=exc.reason, source="fc")
                self.estopped = True
                self._end_open_trial("estop")
                status = "estop"
            # 3. ramp down with HB, confirm motors off
            if status not in ("estop", "link_lost", "uplink_unverified", "refused"):
                try:
                    self.stop_and_confirm()
                except EstopSeen as exc:
                    if exc.source == "fc":
                        self.j("estop", reason=exc.reason, source="fc")
                    self.estopped = True
                    status = "estop"
        except BaseException as exc:
            # ESTOP while the heartbeat still runs, never after it stopped
            self.emergency("%s during spin %d: %s" % (type(exc).__name__, k, exc))
            raise
        finally:
            if self.hb is not None:
                self.hb.stop()
            if self.armed_from is not None:
                with self.lock:
                    self.motor_s += max(0.0, self.clock.now() - self.armed_from)
                    self.armed_from = None
        self.cmd("COLL 0", retries=1)
        self.state("DONE")
        # 4. fetch and cut into trials (motors are off: nothing here may ESTOP)
        late = None
        if self.spin_armed:
            late = self.fetch_and_segment(k, sent)
        else:
            self.note("spin %d: the FC never armed; no recording to fetch" % k)
        self.rx_health(k)
        if late is not None and status == "ok":
            if late.source == "fc":
                self.j("estop", reason=late.reason, source="fc")
            self.estopped = True
            status = "estop"
        return status

    # ---- the disarmed setup and its proof ----
    def read_rig(self, stage):
        """One RBPOKE read of g_rig (+ FC globals, RX counters). Disarmed only."""
        if self.layout is None:
            self.layout = self.link.rig_layout()
            self.j("rig_layout", source=self.layout.source, g_rig="%#x" % self.layout.rig_addr,
                   fields=sorted(self.layout.offsets), fc_globals=sorted(self.layout.fc_globals),
                   rx=sorted(self.layout.rx))
            if not self.layout.rx:
                self.note("this ELF has no %s* RX counters: uplink RX health is not journaled"
                          % RX_COUNTER_PREFIX)
        regions = self.layout.regions()
        prev = self.phase
        self.set_phase("poke")
        with self.poller.session_lock:
            blobs = None
            for _ in range(3):
                try:
                    blobs = self.link.peek(regions)
                    break
                except EspRefused as exc:
                    if "motors-running" not in str(exc):
                        raise
                    self.clock.sleep(ESP_HOLDOFF_S)
            if blobs is None:
                raise RuntimeError("RBPOKE refused three times (motors-running)")
        self.clock.sleep(POKE_HANDBACK_S)
        self.set_phase(prev)
        snap = self.layout.decode(regions, blobs)
        self.j("rig_read", stage=stage, spin=self.spin_n, **rig_summary(snap))
        return snap

    def deliver(self, stage, lines, exp):
        """Send `lines` (paced), then prove over RBPOKE that the FC holds `exp` (compare_rig, with
        gain_seq as the delivery count). What did not land is sent again, paced, up to
        RESEND_TRIES times; then UplinkUnverified refuses the spin."""
        before, todo = self.rig, list(lines)
        for attempt in range(RESEND_TRIES + 1):
            for line in todo:
                if line.startswith("GAIN "):     # silence is "unknown": the RBPOKE read decides
                    self.cmd(line, busy_only=True)
                else:
                    self.must(line, priority=(line == "STOP"))
            snap = self.read_rig(stage if attempt == 0 else "%s/resend%d" % (stage, attempt))
            problems, resend, fatal = compare_rig(before, todo, snap, exp)
            if not problems:
                self.rig = snap
                if attempt:
                    self.note("%s: applied after %d resend round(s)" % (stage, attempt))
                return snap
            self.j("uplink_mismatch", stage=stage, attempt=attempt, spin=self.spin_n,
                   problems=problems, resend=resend, fatal=fatal)
            if fatal or not resend or attempt == RESEND_TRIES:
                raise UplinkUnverified("%s: the FC did not apply what was sent%s: %s" % (
                    stage, "" if fatal else " (after %d resend round(s))" % attempt,
                    "; ".join(problems)))
            self.note("%s: not applied (%s); resending %s"
                      % (stage, "; ".join(problems), ", ".join(resend)))
            before, todo = snap, resend
        raise AssertionError("unreachable")

    def setup_disarmed(self, gains, runms):
        """(STOP, COLL 0, the gain point) -> verify -> (RUNMS, RSP) -> verify. The STOP puts the
        FC in DONE, which only a RESET leaves: COLL <target> can then go first (arm_sequence)."""
        self.set_phase("verify")
        base = self.read_rig("baseline")
        if base["rs_name"] in ("RAMP_UP", "RUN", "RAMP_DOWN"):
            raise SpinRefused("g_rig.rs is %s before the spin: the FC is armed" % base["rs_name"])
        self.rig = base
        self.rx_before = dict(base["rx"]) if base["rx"] else None
        exp = {"gains": dict(base["gains"]), "coll": 0, "rs": ("DONE",), "stop_ok": True}
        exp["gains"].update({n: f32(v) for n, v in gains.items()})
        lines = ["STOP", "COLL 0"]
        sp = self.spec["roll_sp_mdeg"]
        tmax = (self.fc_limits or {}).get("RIG_TGT_MAX_RAD", MAX_TILT_RAD - 0.09)
        if not tgt_within(base["sp_roll_cmd"], exp["gains"]["ROLL_TRIM"], 0.0, tmax):
            lines.append("RSP 0")      # the FC checks a new ROLL_TRIM against the setpoint it holds
            exp["rsp"] = 0
        # every gain of the point, also those equal to what the FC holds: gain_seq counts them
        lines += [gain_line(n, v) for n, v in gains.items()]
        self.deliver("gains", lines, exp)
        self.fc_gains = dict(exp["gains"])
        exp.update(runms=runms, rsp=sp)
        self.deliver("params", ["RUNMS %d" % runms, "RSP %d" % sp], exp)
        self.exp = exp

    def clear_latched_estop(self):
        """--clear-estop, first spin: a latched FC estop zeroes coll_cmd on every tick, so a COLL
        sent before the arming RESET could not survive it. Clear the latch first with a RESET sent
        while coll_cmd is 0 and no HB runs (the FC cannot arm), and check RBSTAT."""
        self.note("--clear-estop: clearing the latched ESTOP with a RESET (COLL 0, no heartbeat)")
        self.must("COLL 0")
        self.must("RESET")
        self.wait(0.5, abortable=False)
        st = self.poller.fresh(self.clock.now(), 5.0)
        if st is None or st[2] is None:
            raise SpinRefused("no RBSTAT after the clearing RESET")
        if st[2]["flags"] & 0x02 or st[2]["kill"]:
            raise EstopSeen("the ESTOP is still latched after the clearing RESET: %s"
                            % st[1].strip()[:120], "fc")

    def start_heartbeat(self):
        self.hb = Heartbeat(self.up, self.clock)
        self.hb.start()

    def arm_sequence(self, k):
        """COLL <target>, the heartbeat and the RESET, in the order fc_rig.cpp needs; -> the clock
        time the arm timeout runs from. This is the one place that depends on COLL_SURVIVES_RESET:

        True (fc_rig.cpp as of 11-04): the reset_gen edge leaves coll_cmd alone, so COLL goes out
        while the FC is in DONE (cannot arm; only the RESET edge leaves DONE) and no HB runs, is
        proven over RBPOKE, and the RESET then arms with it.
        False: RESET first, then COLL, which only RBSTAT showing ARMED can confirm."""
        c = self.spec["collective_permille"]
        if COLL_SURVIVES_RESET:
            self.set_phase("verify")
            self.deliver("coll", ["COLL %d" % c], dict(self.exp, coll=c, rs=("DONE",), stop_ok=False))
        self.start_heartbeat()
        self.wait(HB_LEAD_S, abortable=False)
        self.pre_reset_check(k)
        self.must("RESET")
        t0 = self.clock.now()
        if self.first_spin is None:
            self.first_spin = t0
        if not COLL_SURVIVES_RESET:
            self.must("COLL %d" % c)
            t0 = self.clock.now()
        return t0

    def pre_reset_check(self, k):
        """RBSTAT right before the RESET (which clears both estop latches): refuse on a latched
        ESTOP or ESP kill unless --clear-estop covers it (first spin; and with COLL sent first, an
        FC estop has already zeroed coll_cmd, so only the ESP kill alone can be cleared here)."""
        st = self.poller.fresh(self.clock.now(), 5.0)
        if st is None or st[2] is None:
            raise SpinRefused("no RBSTAT right before RESET (%s)" % (self.poller.last_error or "timeout"))
        s = st[2]
        self.j("event", text="host: RBSTAT before RESET: %s" % st[1].strip()[:160])
        fc_estop = bool(s["flags"] & 0x02)
        if fc_estop or s["kill"]:
            ok = k == 1 and self.clear_estop and not (fc_estop and COLL_SURVIVES_RESET)
            if not ok:
                raise EstopSeen("RBSTAT right before RESET shows %s; not sending RESET (it would "
                                "clear it)%s" % ("ESTOP" if fc_estop else "the ESP kill latch (kill=1)",
                                                 "" if k > 1 or self.clear_estop else
                                                 "; --clear-estop lets the first spin clear a latch "
                                                 "from before the batch"), "fc")
            self.note("RBSTAT shows the ESP kill latch; --clear-estop: the RESET clears it")
        if s["flags"] & 0x10:
            raise EstopSeen("RBSTAT shows WINDOW-EXPIRED: the commit window is used up", "fc")
        if s["ml"] != "up":
            raise SpinRefused("RBSTAT before RESET: motor link %s" % s["ml"])
        if s["flags"] & 0x01 or any(s["duty"]):
            raise SpinRefused("RBSTAT before RESET: ARMED or duty %s already" % s["duty"])

    def _spin_body(self, runs, sent, t0):
        # arming: ARMED in RBSTAT's fl= within ARM_SETTLE + ARM_SLACK of the RESET
        deadline = t0 + ARM_SETTLE_MS / 1000.0 + ARM_SLACK_S
        self.poller.interval = 0.25
        armed_at = None
        prev_obs = t0
        lat = None
        while armed_at is None:
            self.wait(0.05, abortable=True)
            lat = self.poller.latest
            if lat is not None and lat[0] > prev_obs and lat[2] is not None:
                self.check_stat(lat[2], expect_armed=False)
                if lat[2]["flags"] & 0x01:
                    armed_at = lat[0]
                    with self.lock:
                        self.armed_from = prev_obs     # earliest it can have armed: motor time
                    self.spin_armed = True
                else:
                    prev_obs = lat[0]
            if armed_at is None and self.clock.now() > deadline:
                self.poller.interval = 1.0
                raise ArmFailed(self.diagnose_arm(deadline - t0))
        self.j("state", rs="RAMP_UP", src="rbstat", fl="0x%02x" % lat[2]["flags"], duty=lat[2]["duty"],
               after_reset_s=round(armed_at - t0, 2))
        self.set_phase("ramp_up")
        # RAMP_UP starts at zero collective: nonzero duty is confirmed separately
        duty_seen = any(lat[2]["duty"])
        if duty_seen:
            self.j("event", text="host: motors confirmed by RBSTAT, duty %s" % lat[2]["duty"])
        t_run = armed_at + RAMP_MS / 1000.0 + 0.05
        while self.clock.now() < t_run:
            self.wait(0, armed=True, until=min(t_run, self.clock.now() + 0.1))
            lat = self.poller.latest
            if not duty_seen and lat is not None and lat[2] is not None and any(lat[2]["duty"]):
                duty_seen = True
                self.j("event", text="host: motors confirmed by RBSTAT, duty %s" % lat[2]["duty"])
        self.poller.interval = 1.0
        self.state("RUN")
        # lift-off: the FC refuses EXC until its roll guard is live, invisible from here
        self.set_phase("settle")
        self.wait(pre_settle_s(self.spec), armed=True)
        st = self.poller.fresh(armed_at, 3.0)
        if st is None or st[2] is None:
            raise LinkLost("no RBSTAT during RUN")
        self.check_stat(st[2], expect_armed=True)
        if not any(st[2]["duty"]):
            raise FcStopped("RBSTAT: ARMED but every duty is 0 in RUN")
        self.j("state", rs="RUN", src="rbstat", fl="0x%02x" % st[2]["flags"], duty=st[2]["duty"])
        self.j("event", text="host: RUN confirmed by RBSTAT, duty %s" % st[2]["duty"])
        settle = self.spec["limits"]["settle_ms"] / 1000.0
        for ri, run in enumerate(runs):
            if ri > 0:
                if self.pause_req:
                    self.note("pausing: ending spin %d after trial %d" % (self.spin_n, sent[-1]["trial"]))
                    return "ok"
                self.set_phase("gain_change")
                self.send_gains(run[0]["gains"], live=True)
                self.wait(settle, armed=True)
            for t in run:
                if self.pause_req and sent:
                    self.note("pausing: ending spin %d after trial %d" % (self.spin_n, sent[-1]["trial"]))
                    return "ok"
                self.run_trial(t, sent)
        return "ok"

    def diagnose_arm(self, waited):
        """The FC did not arm. Stop the heartbeat (it cannot arm without one), make sure the
        motors are off, then read WHY over RBPOKE -- before any STOP, whose to_done() would zero
        coll_cmd and the arm dwell. -> the ArmFailed message."""
        self.set_phase("arm_diag")
        self.hb.stop()
        t_off = self.clock.now()
        self.wait(HB_RAMP_MS / 1000.0 + 0.2, abortable=False)
        st = self.poller.fresh(self.clock.now(), 3.0)
        if st is None or st[2] is None:
            return "the FC did not arm within %.1f s of RESET (no RBSTAT for a diagnosis)" % waited
        if st[2]["flags"] & 0x01 or any(st[2]["duty"]):
            # armed just after the timeout: heartbeat back on, the ramp-down runs as usual
            with self.lock:
                self.armed_from = self.clock.now()
            self.spin_armed = True
            self.start_heartbeat()
            return "the FC armed only after the %.1f s timeout (RBSTAT %s); stopping it" % (
                waited, st[1].strip()[:120])
        try:
            snap = self.read_rig("arm_diag")
        except Exception as exc:  # noqa: BLE001 - the diagnosis is advisory
            return "the FC did not arm within %.1f s of RESET; the RBPOKE diagnosis failed: %s: %s" % (
                waited, type(exc).__name__, exc)
        why, notes = arm_reasons(snap, self.rig, self.clock.now() - t_off, self.axis)
        self.j("arm_diag", spin=self.spin_n, reasons=why, notes=notes, **rig_summary(snap))
        return "the FC did not arm within %.1f s of RESET: %s [%s]" % (
            waited, "; ".join(why) if why else "no arm condition visibly failed (still? tilt? "
            "battery?)", ", ".join(notes))

    def rx_health(self, k):
        """The FC's uplink RX counters before / after the spin (only if the ELF has them)."""
        if self.layout is None or not self.layout.rx or self.rx_before is None:
            return
        try:
            after = self.read_rig("rx_after")["rx"]
        except Exception as exc:  # noqa: BLE001 - advisory
            self.note("spin %d: RX counters after the spin not read: %s" % (k, exc))
            return
        self.j("uplink_rx", spin=k, before=self.rx_before, after=after,
               delta={n: (after[n] - self.rx_before.get(n, 0)) & 0xFFFFFFFF for n in after})
        self.rx_before = None

    def run_trial(self, t, sent):
        with self.lock:
            self.cur_trial = t["trial"]
            self.cur = {"gains": dict(t["gains"]), "exc": dict(t["exc"])}
            self.phase = "trial"
        self.j("trial_start", trial=t["trial"], gains=t["gains"], exc=t["exc"], spin=self.spin_n,
               rep=t["rep"])
        line = exc_line(t["exc"])
        ok = self.cmd(line, retries=0)      # never repeated: a lost ACK may still have started it
        sent.append(t)
        self.queue.remove(t)
        self.wait((t["exc"]["dur_ms"] + self.spec["limits"]["settle_ms"]) / 1000.0, armed=True)
        status = "ok" if ok else "timeout"
        if not ok:
            self.note("trial %d: no ESP ACK for %r; the recording decides" % (t["trial"], line))
        self.j("trial_end", trial=t["trial"], status=status)
        with self.lock:
            self.cur_trial = None

    def send_gains(self, gains, live):
        """Live (armed) GAIN lines for every gain that differs from what the FC holds; paced, not
        verifiable while motors run (the recorder's gain log and the next spin's baseline are)."""
        changed = [(n, v) for n, v in gains.items()
                   if self.fc_gains is None or n not in self.fc_gains or f32(self.fc_gains[n]) != f32(v)]
        for n, v in changed:
            self.must(gain_line(n, v))
            if self.fc_gains is None:
                self.fc_gains = {}
            self.fc_gains[n] = v
        if changed:
            self.j("event", text="host: %d GAIN line(s) %s" % (len(changed), "live" if live else "disarmed"))

    def stop_and_confirm(self):
        self.set_phase("ramp_down")
        self.cmd("STOP", retries=3, priority=True)
        t_stop = self.clock.now()
        self.state("RAMP_DOWN")
        try:
            self.wait(RAMP_MS / 1000.0 + 0.3, abortable=False)
        except EstopSeen:
            raise
        end = self.clock.now() + STOP_CONFIRM_S
        while True:
            st = self.poller.fresh(t_stop + RAMP_MS / 1000.0, max(0.1, end - self.clock.now()))
            if st is not None and st[2] is not None and not st[2]["flags"] & 0x01 \
                    and not any(st[2]["duty"]):
                return
            if self.clock.now() >= end:
                self.emergency("motors still on %.1f s after STOP (RBSTAT %s)"
                               % (self.clock.now() - t_stop, st[1].strip() if st else "none"))
                return
            self.wait(0.1, abortable=False)

    # ---- fetch + trials ----
    def fetch_and_segment(self, k, sent):
        self.set_phase("fetch")
        prefix = self.dir / "spins" / str(k) / "rec"
        late = None
        try:
            self.wait(max(ESP_HOLDOFF_S, FLIGHTREC_TAIL_S) + 0.5, abortable=False)
        except EstopSeen as exc:     # motors already off: fetch anyway, end the batch after
            late = exc
            self.wait(max(ESP_HOLDOFF_S, FLIGHTREC_TAIL_S) + 0.5, abortable=False, watch=False)
        motor = T.read_json(self.dir / "spins" / str(k) / "spin.json", {}).get("estimate", {})
        self.fetch_eta = round(fetch_estimate_s(motor.get("motor_s", 10.0)), 1)
        self.note("spin %d: fetching the recorder (~%.0f s over WiFi)" % (k, self.fetch_eta))
        res = None
        err = None
        for attempt in range(FETCH_TRIES):
            try:
                with self.poller.session_lock:
                    res = self.link.fetch(prefix)
                break
            except EspRefused as exc:
                err = str(exc)
                if "motors-running" not in err:
                    break
                self.clock.sleep(ESP_HOLDOFF_S)
            except Exception as exc:  # noqa: BLE001 - a failed fetch must not kill the batch
                err = "%s: %s" % (type(exc).__name__, exc)
                break
        self.fetch_eta = None
        if res is None:
            self.note("spin %d fetch failed: %s (retry later: ./rb rig fetch %s --spin %d)"
                      % (k, err, self.dir, k))
            return late
        self.clock.sleep(POKE_HANDBACK_S)
        if res.get("layout") not in (2, None):
            self.note("spin %d: recorder layout %r is not the rig layout (2)" % (k, res.get("layout")))
        if res.get("axis") not in (None, AX.name(self.axis)):
            self.note("spin %d: the recording says axis %s (%s) but this batch is %s -- wrong ELF?"
                      % (k, res.get("axis"), res.get("axis_source"), AX.name(self.axis)))
        try:
            process_spin(self.dir, k, sent, res, journal=self.j, note=self.note, analyze=self.analyze)
        except Exception as exc:  # noqa: BLE001 - the raw files are saved; never ESTOP over this
            self.note("spin %d: cutting the recording into trials failed (%s: %s); the raw fetch is "
                      "in spins/%d -- ./rb rig fetch / analyze can redo it"
                      % (k, type(exc).__name__, exc, k))
        return late


def process_spin(batch_dir, k, sent, res, journal, note, analyze=True):
    """Cut spins/<k>/rec.csv into trials/<n>/rec.csv (+ rec.bin link, trial.json), emit fetch
    events, compute metrics. Shared by the runner and `rig fetch`."""
    batch_dir = Path(batch_dir)
    header, rows = read_rec_csv(res["csv"])
    gj = None
    if res.get("gains") and Path(res["gains"]).is_file():
        try:
            with open(res["gains"], "r", encoding="utf-8") as f:
                gj = json.load(f)
        except (OSError, ValueError):
            gj = None
    segs = segment_spin(header, rows, sent, gj)
    statuses = trial_statuses(batch_dir)
    ana = None
    if analyze:
        try:
            ana = _sibling("rig_analyze")
        except ImportError as exc:
            note("metrics skipped (%s); run ./rb rig analyze %s in the zephyr env" % (exc, batch_dir))
    for s, seg in zip(sent, segs):
        n = s["trial"]
        tdir = batch_dir / "trials" / str(n)
        tdir.mkdir(parents=True, exist_ok=True)
        tmp = tdir / ".rec.csv.tmp"
        with open(tmp, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(rows[seg["lo"]:seg["hi"]])
        os.replace(str(tmp), str(tdir / "rec.csv"))
        binp = tdir / "rec.bin"
        try:
            if binp.exists():
                binp.unlink()
            os.link(res["bin"], binp)
        except OSError:
            try:
                import shutil
                shutil.copyfile(res["bin"], binp)
            except OSError:
                pass
        meta = {"trial": n, "spin": k, "gains": s["gains"], "exc": s["exc"],
                "rows": [seg["lo"], seg["hi"]], "exc_id": seg["exc_id"], "gain_seq": seg["gain_seq"],
                "t_exc": seg["t_exc"], "matched": seg["matched"], "reason": seg["reason"],
                "status": statuses.get(n), "spin_csv": os.path.relpath(res["csv"], batch_dir)}
        if gj is not None and seg["gain_seq"] is not None:
            meta["gains_recorded"] = gains_at(gj, seg["gain_seq"])
        T.write_json_atomic(tdir / "trial.json", meta)
        journal("fetch", trial=n, csv="trials/%d/rec.csv" % n, bin="trials/%d/rec.bin" % n,
                samples=max(0, seg["hi"] - seg["lo"]), gaps=res.get("gaps", 0), spin=k)
        if not seg["matched"]:
            note("trial %d: %s; no metrics" % (n, seg["reason"]))
            continue
        if ana is not None:
            try:
                m = ana.analyze_trial(tdir, meta)
            except Exception as exc:  # noqa: BLE001
                note("trial %d: metrics failed: %s: %s" % (n, type(exc).__name__, exc))
                continue
            ev = {k2: m.get(k2) for k2 in ana.JOURNAL_KEYS}
            journal("metrics", trial=n, **ev)


def trial_statuses(batch_dir):
    evs, _ = T.load_journal(Path(batch_dir) / "journal.jsonl")
    out = {}
    for e in evs:
        if e.get("ev") == "trial_end" and "trial" in e:
            out[e["trial"]] = e.get("status")
    return out


# =============================================================================
# CLI
# =============================================================================

def _install_signals():
    def handler(signum, frame):
        raise SignalExit("signal %d" % signum)
    old = {}
    for sig in (signal.SIGTERM, getattr(signal, "SIGHUP", None)):
        if sig is not None:
            try:
                old[sig] = signal.signal(sig, handler)
            except (ValueError, OSError):
                pass
    return old


def default_elf(mode):
    return ROOT / "build" / "zephyr" / "arty200t" / "flight_controller" / mode / "zephyr" / "zephyr.elf"


def spec_axis_hint(path):
    """The raw spec's "axis" as 0/1 (None if absent, unreadable or invalid; validate_spec reports
    those). Read before the ELF is resolved, so the default --mode can follow it."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)
    except (OSError, ValueError):
        return None
    return AX.parse_axis(raw.get("axis")) if isinstance(raw, dict) and "axis" in raw else None


def resolve_mode(mode, spec_path):
    """(mode, problem or None). No --mode: rig-pitch for an "axis": "pitch" spec, else rig-roll.
    A rig-roll / rig-pitch --mode that contradicts the spec's axis is a problem; any other mode
    name makes no claim (the ELF's own axis is checked against the spec)."""
    hint = spec_axis_hint(spec_path)
    if not mode:
        return AX.AXIS_MODES[hint if hint is not None else AX.ROLL], None
    m = AX.mode_axis(mode)
    if hint is not None and m is not None and m != hint:
        return mode, ("--mode %s builds the %s axis but the spec says \"axis\": \"%s\" (use --mode %s)"
                      % (mode, AX.name(m), AX.name(hint), AX.AXIS_MODES[hint]))
    return mode, None


def cmd_run(a):
    if not 0.0 <= a.tx_gap_ms < HB_PERIOD_S * 1000.0:
        print("rig run: --tx-gap-ms must be in [0, %.0f): at or above the heartbeat period HB "
              "cannot keep 20 Hz" % (HB_PERIOD_S * 1000.0), file=sys.stderr)
        return 2
    mode_explicit = a.mode is not None
    a.mode, problem = resolve_mode(a.mode, a.spec)
    if problem:
        print("rig run: %s" % problem, file=sys.stderr)
        return 2
    # the build's EFFECTIVE limits (its -D overrides) and its axis; --check / --dry-run without
    # --elf use the mode's build if there is one, else the fc_rig.cpp defaults and the spec's axis
    lim_elf = a.elf if a.elf else (default_elf(a.mode) if (a.check or a.dry_run) else None)
    have_elf = bool(lim_elf) and Path(lim_elf).is_file()
    limits = fc_limits(str(lim_elf) if have_elf else None)
    elf_axis = AX.elf_axis(str(lim_elf)) if have_elf else None
    lim_axis = elf_axis[0] if elf_axis else (spec_axis_hint(a.spec) or AX.ROLL)
    try:
        spec = load_spec(a.spec, limits, elf_axis)
    except SpecError as exc:
        print("rig run: FC limits: %s" % limits_text(limits, lim_axis), file=sys.stderr)
        print("rig run: %s" % exc, file=sys.stderr)
        return 2
    axis = AX.parse_axis(spec["axis"])
    axis_source = elf_axis[1] if elf_axis else "spec (no ELF: %s not built)" % (lim_elf or "--elf")
    m_axis = AX.mode_axis(a.mode)
    if m_axis is not None and m_axis != axis:
        # e.g. a spec without "axis" run with --elf on a pitch build: batch.json's mode is what
        # `rig fetch` resolves the ELF from later, so it must not say the other axis
        if mode_explicit:
            print("rig run: --mode %s is the %s build but this batch is %s (%s); use --mode %s"
                  % (a.mode, AX.name(m_axis), spec["axis"].upper(), axis_source,
                     AX.AXIS_MODES[axis]), file=sys.stderr)
            return 2
        a.mode = AX.AXIS_MODES[axis]
    print("rig run: FC limits: %s" % limits_text(limits, axis))
    print("rig run: axis %s (%s), mode %s" % (spec["axis"].upper(), axis_source, a.mode))
    if not elf_axis and (a.check or a.dry_run):
        print("rig run: NOTE no ELF at %s: the axis and the -D limits are NOT checked against a "
              "build" % lim_elf)
    layout = None
    if not a.dry_run and not a.check:
        if not a.i_have_approval or len(a.i_have_approval.strip()) < 8:
            print("rig run: refusing to spin motors without --i-have-approval \"<who, when, what>\" "
                  "(Loren approves each campaign explicitly; the text goes into batch.json)",
                  file=sys.stderr)
            return 2
        if not a.elf or not Path(a.elf).is_file():
            print("rig run: the running rig ELF is required (--elf / --mode): %s" % a.elf,
                  file=sys.stderr)
            return 2
        try:
            _sibling("esp_tsi_load")
            layout = RigLayout.from_elf(str(a.elf))
        except ImportError as exc:
            print("rig run: %s (run it in the zephyr env: ./rb rig run)" % exc, file=sys.stderr)
            return 2
        except (RuntimeError, OSError) as exc:
            print("rig run: %s" % exc, file=sys.stderr)
            return 2
        print("rig run: g_rig at %#x (%s); FC globals %s; RX counters %s"
              % (layout.rig_addr, layout.source, ",".join(sorted(layout.fc_globals)) or "none",
                 ",".join(sorted(layout.rx)) or "none (not in this ELF)"))
    trials = plan_trials(spec)
    print("rig run: %d trial(s), %d gain point(s) x %d excitation(s) x %d repeat(s)%s, per_spin %d"
          % (len(trials), len(spec["grid"]), len(spec["excitations"]), spec["repeats"],
             " (capped by max_trials)" if len(trials) < len(spec["grid"]) * len(spec["excitations"])
             * spec["repeats"] else "", spec["arrangement"]["per_spin"]))
    if a.check:
        q, n = list(trials), 0
        while q:
            s, why = take_spin(spec, q, 1e9, 1e9)
            n += 1
            e = spin_estimate(spec, s)
            print("  spin %d: trials %s  ~%.1f s motors" % (n, ",".join(str(t["trial"]) for t in s),
                                                            e["motor_s"]))
            q = q[len(s):]
        print("rig run: spec OK (--check: nothing sent)")
        return 0
    name = spec["name"] + ("-dryrun" if a.dry_run else "")
    bdir = make_batch_dir(a.out_dir, name)
    if a.dry_run:
        clock = SimClock(a.speed)
        link = FakeDrone(clock, seed=a.seed, mass_kg=spec["mass_kg"], axis=axis)
        approval = a.i_have_approval or "dry run: fake drone, no hardware"
    else:
        clock = RealClock()
        link = RealLink(a.drone, a.port, elf=a.elf, clock=clock, layout=layout)
        approval = a.i_have_approval
    print("rig run: batch %s%s" % (bdir, "  (DRY RUN at %gx)" % a.speed if a.dry_run else ""))
    print("rig run: watch it with  ./rb rig tui --batch %s" % bdir)
    runner = Runner(spec, bdir, link, clock, approval=approval, mode=a.mode,
                    elf=str(a.elf) if a.elf else None, dry_run=a.dry_run,
                    commit_used_s=a.commit_used_s, clear_estop=a.clear_estop,
                    tx_gap_ms=a.tx_gap_ms, limits=limits, axis_source=axis_source)
    old = _install_signals()
    try:
        status = runner.run()
    except (KeyboardInterrupt, SignalExit):
        print("\nrig run: interrupted -- ESTOP x3 sent", file=sys.stderr)
        return 130
    except Exception as exc:  # noqa: BLE001
        print("rig run: %s: %s -- ESTOP x3 sent" % (type(exc).__name__, exc), file=sys.stderr)
        return 1
    finally:
        for sig, h in old.items():
            signal.signal(sig, h)
        link.close()
    print("rig run: batch_end %s; motors %.1f s; %s" % (status, runner.motor_s, bdir))
    if status == "ok":
        print("rig run: ./rb rig analyze %s" % bdir)
    return 0 if status == "ok" else 3


def cmd_fetch(a):
    if a.out:
        link = RealLink(a.drone, a.port, elf=a.elf)
        try:
            st = T.parse_rblink(link.stat())
            if st and (st["flags"] & 0x01 or any(st["duty"])):
                print("rig fetch: motors are running (RBSTAT %s); a POKE session would cut them"
                      % st["duty"], file=sys.stderr)
                return 2
            res = link.fetch(a.out)
        finally:
            link.close()
        print("rig fetch: %s" % json.dumps(res))
        return 0
    bdir = Path(a.batch)
    spins = sorted((int(p.name) for p in (bdir / "spins").iterdir() if p.name.isdigit()),
                   key=int) if (bdir / "spins").is_dir() else []
    k = a.spin or (spins[-1] if spins else None)
    if k is None:
        print("rig fetch: %s has no spins/<k>/spin.json" % bdir, file=sys.stderr)
        return 2
    plan = T.read_json(bdir / "spins" / str(k) / "spin.json", {})
    started = {e["trial"] for e in T.load_journal(bdir / "journal.jsonl")[0]
               if e.get("ev") == "trial_start"}
    sent = [p for p in plan.get("plan", []) if p["trial"] in started]
    for p in sent:
        p.setdefault("rep", 0)
    link = RealLink(a.drone, a.port, elf=a.elf)
    try:
        st = T.parse_rblink(link.stat())
        if st and (st["flags"] & 0x01 or any(st["duty"])):
            print("rig fetch: motors are running; not opening a session", file=sys.stderr)
            return 2
        res = link.fetch(bdir / "spins" / str(k) / "rec")
    finally:
        link.close()

    def jrnl(ev, **kw):
        obj = {"t": round(time.time(), 3), "ev": ev}
        obj.update(kw)
        T.append_journal(bdir / "journal.jsonl", obj)

    def note(text):
        print("rig fetch: " + text)
        jrnl("note", text=text)
    process_spin(bdir, k, sent, res, journal=jrnl, note=note)
    print("rig fetch: spin %d -> %d trial(s) in %s" % (k, len(sent), bdir / "trials"))
    return 0


def build_parser():
    ap = argparse.ArgumentParser(prog="rig_run.py", description=__doc__.strip().splitlines()[0])
    sub = ap.add_subparsers(dest="action", required=True)
    add_run_arguments(sub.add_parser("run", help="run a batch spec"))
    add_fetch_arguments(sub.add_parser("fetch", help="(re-)fetch a spin's recording"))
    return ap


def add_run_arguments(p):
    p.add_argument("spec", help="batch spec JSON (see tools/rig/examples/rate_p_sweep.json)")
    p.add_argument("--i-have-approval", metavar="TEXT",
                   help="required for hardware: who approved this campaign, recorded in batch.json")
    p.add_argument("--dry-run", action="store_true",
                   help="fake drone in-process (state machine, timing, roll model); no sockets")
    p.add_argument("--speed", type=float, default=5.0, help="dry run: virtual time speed-up")
    p.add_argument("--seed", type=int, default=1, help="dry run: noise seed")
    p.add_argument("--check", action="store_true", help="validate and print the spin plan; send nothing")
    p.add_argument("--elf", help="the running rig ELF (fetch, --verify-poke, batch.json sha1)")
    p.add_argument("--mode", default=None,
                   help="flight_controller mode (recorded; finds the ELF via ./rb). Default: "
                   "rig-pitch for a spec with \"axis\": \"pitch\", else rig-roll")
    p.add_argument("--verify-poke", action="store_true",
                   help="(always on now; accepted for old command lines) every disarmed command is "
                   "proven applied over RBPOKE before RESET")
    p.add_argument("--tx-gap-ms", type=float, default=TX_GAP_MS,
                   help="minimum gap between two datagrams to the drone, HB included (default "
                   "%(default)g; the FC's UART loses lines sent closer). ESTOP is never delayed")
    p.add_argument("--drone", default=DEFAULT_DRONE)
    p.add_argument("--port", type=int, default=CMD_PORT)
    p.add_argument("--out-dir", default=str(RIG_DIR), help="parent of the batch directory")
    p.add_argument("--commit-used-s", type=float, default=0.0,
                   help="seconds of this load's commit window already used by an earlier batch")
    p.add_argument("--clear-estop", action="store_true",
                   help="allow the first RESET to clear an ESTOP latched before this batch")
    p.set_defaults(func=cmd_run)


def add_fetch_arguments(p):
    p.add_argument("batch", nargs="?", help="batch directory (re-fetch its last or --spin K)")
    p.add_argument("--spin", type=int, help="spin number to (re)fill")
    p.add_argument("--out", help="plain fetch to this prefix instead of a batch")
    p.add_argument("--elf", required=True, help="the running rig ELF")
    p.add_argument("--mode", default="rig-roll", help="(used by ./rb to find the ELF)")
    p.add_argument("--drone", default=DEFAULT_DRONE)
    p.add_argument("--port", type=int, default=CMD_PORT)
    p.set_defaults(func=cmd_fetch)


def main(argv=None):
    a = build_parser().parse_args(argv)
    if a.action == "fetch" and not a.out and not a.batch:
        print("rig fetch: give a batch directory or --out PREFIX", file=sys.stderr)
        return 2
    return a.func(a)


if __name__ == "__main__":
    sys.exit(main())
