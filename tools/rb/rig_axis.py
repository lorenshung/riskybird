"""Which axis a roll-harness ("rig") build controls: 0 = roll (rig-roll), 1 = pitch (rig-pitch).

The drone sits on a 1-DOF pivot; turned 90 deg in the cradle the pivot is the PITCH axis. A rig
build says which one it controls in two places (RIG-SPEC "Pitch axis"):
  * the ELF: `const uint8_t g_rig_axis` (0 roll, 1 pitch) in its read-only data;
  * the build record rb-build.txt next to the ELF (or one directory up): `-DRIG_AXIS=1` on its
    cppflags: (or cmake:) line; absent means 0.
elf_axis() reads the ELF symbol (pyelftools, imported lazily: plain python3 without it still
works), falls back to the build record, then to 0 (roll) for older builds that have neither.

Stdlib only at import, shared by flightrec_fetch.py, rig_run.py, rig_analyze.py and rig_tui.py.
"""

from __future__ import annotations

import re
from pathlib import Path

ROLL, PITCH = 0, 1
AXIS_NAMES = ("roll", "pitch")            # index = RIG_AXIS / g_rig_axis
AXIS_LETTERS = ("R", "P")                 # the CSV `axis` column and the RIG line's ax=
AXIS_MODES = ("rig-roll", "rig-pitch")    # tools/rb/boards.py modes
SYMBOL = "g_rig_axis"
_ZL_RE = re.compile(r"^_ZL(\d+)(.+)$")
_AXIS_DEF_RE = re.compile(r"(?:^|\s)-DRIG_AXIS(?:=(\S+))?(?=\s|$)")


def parse_axis(v) -> int | None:
    """0/1, "roll"/"pitch", "R"/"P", "rig-roll"/"rig-pitch" -> 0/1; anything else None."""
    if isinstance(v, bool):
        return None
    if isinstance(v, int):
        return v if v in (ROLL, PITCH) else None
    if not isinstance(v, str):
        return None
    s = v.strip().lower()
    for i in (ROLL, PITCH):
        if s in (AXIS_NAMES[i], AXIS_LETTERS[i].lower(), AXIS_MODES[i], str(i)):
            return i
    return None


def name(axis: int | None) -> str:
    return AXIS_NAMES[axis if axis in (ROLL, PITCH) else ROLL]


def letter(axis: int | None) -> str:
    return AXIS_LETTERS[axis if axis in (ROLL, PITCH) else ROLL]


def mode_axis(mode) -> int | None:
    """rig-roll -> 0, rig-pitch -> 1, any other mode name -> None (no claim)."""
    return AXIS_MODES.index(mode) if mode in AXIS_MODES else None


def elf_symbol_axis(elf_path) -> int | None:
    """g_rig_axis read from the ELF's section data (plain name or a C++ file static's
    _ZL<n>g_rig_axis). None when pyelftools is missing, the file is not a readable ELF, the
    symbol is absent or not in file-backed data, or its value is not 0/1."""
    try:
        from elftools.elf.elffile import ELFFile  # noqa: PLC0415 - zephyr env only
    except ImportError:
        return None
    try:
        with open(elf_path, "rb") as f:
            elf = ELFFile(f)
            tab = elf.get_section_by_name(".symtab")
            if tab is None:
                return None
            for s in tab.iter_symbols():
                n = s.name
                m = _ZL_RE.match(n)
                if m and len(m.group(2)) == int(m.group(1)):
                    n = m.group(2)
                if n != SYMBOL or not isinstance(s["st_shndx"], int):
                    continue
                sec = elf.get_section(s["st_shndx"])
                if sec["sh_type"] == "SHT_NOBITS":
                    return None
                off = s["st_value"] - sec["sh_addr"]
                data = sec.data()
                if not 0 <= off < len(data):
                    return None
                v = data[off]
                return v if v in (ROLL, PITCH) else None
    except Exception:  # noqa: BLE001 - a missing/empty/foreign file: fall back, never fail here
        return None
    return None


def build_record(elf_path):
    """The rb-build.txt next to the ELF or one directory up (build/.../<mode>/rb-build.txt with
    the ELF at <mode>/zephyr/zephyr.elf), or None."""
    elf = Path(elf_path)
    for d in (elf.parent, elf.parent.parent):
        rec = d / "rb-build.txt"
        if rec.is_file():
            return rec
    return None


def build_record_axis(elf_path) -> tuple[int | None, str | None]:
    """(RIG_AXIS from the build record's cmake:/cppflags: -D flags (cppflags win), its path).
    (0, path) when the record exists without RIG_AXIS; (None, None) without a record."""
    rec = build_record(elf_path)
    if rec is None:
        return None, None
    try:
        text = rec.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None, None
    val = None
    for key in ("cmake:", "cppflags:"):
        for line in text.splitlines():
            if line.startswith(key):
                for m in _AXIS_DEF_RE.finditer(line):
                    val = m.group(1) or "1"
    if val is None:
        return ROLL, str(rec)
    try:
        v = int(val.rstrip("uUlL"), 0)
    except ValueError:
        return None, str(rec)
    return (v if v in (ROLL, PITCH) else None), str(rec)


def elf_axis(elf_path) -> tuple[int, str]:
    """(axis, where it came from). ELF symbol > build record > 0 (an old roll-only build)."""
    if elf_path:
        v = elf_symbol_axis(elf_path)
        if v is not None:
            return v, "%s in %s" % (SYMBOL, elf_path)
        v, rec = build_record_axis(elf_path)
        if v is not None:
            return v, ("%s (%s)" % (rec, "-DRIG_AXIS=%d" % v if v else "no RIG_AXIS: roll"))
    return ROLL, "default (no %s, no build record: a roll build)" % SYMBOL
