"""Machine-readable SoC fingerprints, extracted rather than trusted.

Two halves of the design-space exploration measure a machine: `rb area`
synthesizes it, and `rb xpurt` schedules a workload against per-op profiles
captured on it. A join between them is only sound if both describe the *same*
machine, and a configuration name is not evidence of that. `gemmini_q31` and
`V256D128_rvv` are profile *directory* names; `Q31Ws32x32AccGemminiKU040Config`
elaborates to a single hart whose ISA carries no ``zvl*`` at all, so pairing it
with an RVV profile silently describes hardware that does not exist. Name
matching cannot catch that. Extraction can.

Everything needed is already emitted by elaboration:

===================  ==========================================================
harts                ``cpu@N`` nodes in the DTS
ISA, vector length   each hart's ``riscv,isa``
clock                the bus ``clock-frequency`` nodes
memory map           ``memory@BASE`` nodes, including ``status``
accelerator shape    module names and instance counts in ``gen-collateral``
===================  ==========================================================

The fingerprint separates two kinds of fact, because they fail differently:

* **Identity** — harts, ISA, accelerators, memory map. These change *cycle
  counts*. Two machines sharing an ``identity_id`` produce the same per-op
  cycles, so a profile captured on one transfers exactly to the other.
* **Clock** — changes *wall time only*. Cycles are invariant to it, so it is
  deliberately excluded from ``identity_id`` and applied once, at the end, by
  the consumer that needs milliseconds.

That split is what lets a profile captured on FireSim's modeled 1 GHz be
reported honestly at a KU040 clock: the cycles carry over, the milliseconds are
recomputed, and nothing is silently pinned to the wrong frequency.
"""

from __future__ import annotations

import hashlib
import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator, Sequence


SCHEMA = "riskybird.soc-fingerprint.v1"

# Single-letter RISC-V base extensions. Chipyard glues the first multi-letter
# extension straight onto this run without a separator ("rv64imafdcbzicsr"),
# so the run ends at the first character outside this set.
_SINGLE_LETTER_EXTENSIONS = "imafdqcbvhgjptn"

_ISA_XLEN = re.compile(r"\Arv(\d+)", re.I)
_ZVL = re.compile(r"\Azvl(\d+)b\Z", re.I)
_STRING = re.compile(r'"([^"]*)"')
_CELLS = re.compile(r"<([^>]*)>")

# A leaf cell whose instance count is the accelerator's shape. Each entry is
# (accelerator, marker modules that must all be present, leaf cell, unit,
#  optional features keyed to the module that appears when the feature is on).
#
# The marker modules alone do not pin the machine down. Gemmini's `has_loop_conv`
# and `mvin_scale_args` both strip whole datapaths without touching the mesh, so
# two configs that differ by either one land on the same `mesh_cells` and the
# same marker list -- and, before these features were recorded, on the same
# `identity_id`. That let a latency measurement taken on a LoopConv-free machine
# join to an area record that had LoopConv, and be labelled one machine.
#
# The feature modules below were chosen by diffing `gen-collateral` between the
# two elaborated probes rather than by reading the generator: `LoopConv` heads
# the six LoopConv* modules, and `ScalePipe` heads the float mvin-scale datapath
# (MulAddRecFN_e8_s24 and friends). The `RRArbiter_*`/`ram_*`/`Pipeline_*` names
# also differ between the two, but only by index renumbering, so they are
# deliberately not used as markers.
_ACCELERATOR_MARKERS: tuple[
    tuple[str, tuple[str, ...], str | None, str, tuple[tuple[str, str], ...]], ...
] = (
    (
        "gemmini",
        ("Mesh", "MeshWithDelays", "MacUnit"),
        "MacUnit",
        "mesh_cells",
        (("loop_conv", "LoopConv"), ("mvin_scale", "ScalePipe")),
    ),
    (
        "saturn-opu",
        ("OuterProductCell", "OuterProductUnit"),
        "OuterProductCell",
        "opu_cells",
        (),
    ),
)

# Verilog keywords that can open a statement shaped like an instantiation.
_NOT_A_MODULE = frozenset(
    """
    module endmodule input output inout wire reg logic assign always initial
    parameter localparam generate endgenerate begin end if else for while case
    endcase function endfunction task endtask genvar integer real time signed
    unsigned automatic return default posedge negedge and or not nand nor xor
    xnor buf bufif0 bufif1 notif0 notif1 pullup pulldown supply0 supply1 tri
    wand wor typedef struct union enum package endpackage import export
    """.split()
)

_INSTANCE = re.compile(r"^[ \t]*(?:\(\*.*?\*\)[ \t]*)?([A-Za-z_]\w*)[ \t]+([A-Za-z_]\w*)[ \t]*\(")
_MODULE_DECL = re.compile(r"^[ \t]*module[ \t]+([A-Za-z_]\w*)", re.M)


class FingerprintError(RuntimeError):
    """The elaboration output needed for a fingerprint is missing or unreadable."""


# ---------------------------------------------------------------------------
# Device-tree parsing
# ---------------------------------------------------------------------------


@dataclass
class DtsNode:
    """One device-tree node: a name, an optional label, properties, children."""

    name: str
    label: str | None = None
    properties: dict[str, str] = field(default_factory=dict)
    children: list["DtsNode"] = field(default_factory=list)

    def walk(self) -> Iterator["DtsNode"]:
        yield self
        for child in self.children:
            yield from child.walk()

    def find(self, prefix: str) -> list["DtsNode"]:
        """Every descendant whose node name starts with `prefix`."""
        return [n for n in self.walk() if n.name.startswith(prefix)]

    def strings(self, prop: str) -> list[str]:
        return _STRING.findall(self.properties.get(prop, ""))

    def string(self, prop: str) -> str | None:
        found = self.strings(prop)
        return found[0] if found else None

    def cells(self, prop: str) -> list[int]:
        return _prop_cells(self.properties.get(prop, ""))

    def cell(self, prop: str) -> int | None:
        found = self.cells(prop)
        return found[0] if found else None


def _prop_cells(value: str) -> list[int]:
    """Integers from every ``<...>`` cell group, skipping ``&label`` references."""
    out: list[int] = []
    for group in _CELLS.findall(value):
        for token in group.replace(",", " ").split():
            if token.startswith("&"):
                continue
            try:
                out.append(int(token, 0))
            except ValueError:
                continue
    return out


def _inherited_widths(node: "DtsNode", inherited: tuple[int, int]) -> tuple[int, int]:
    """This node's ``#address-cells``/``#size-cells``, falling back to the parent's.

    These govern how a *child's* ``reg`` is laid out, so they are threaded down
    the tree rather than read off the node owning the ``reg``.
    """
    address_cells = node.cell("#address-cells")
    size_cells = node.cell("#size-cells")
    return (
        address_cells if address_cells is not None else inherited[0],
        size_cells if size_cells is not None else inherited[1],
    )


def _decode_reg(
    cells: list[int], address_cells: int, size_cells: int
) -> list[tuple[int, int]]:
    """Decode a ``reg`` cell list into ``(base, size)`` pairs.

    A cell is 32 bits, so a 64-bit address occupies two of them, most
    significant first. Assuming one cell each -- which holds for the KU040 and
    Arty harnesses, whose roots declare ``#address-cells = <1>`` -- silently
    truncates any tree that needs two. Chipyard's own TestHarness declares
    ``<2>`` as soon as the address space exceeds 32 bits, which is exactly the
    case for a FireSim target carrying 16 GiB of DRAM: ``reg = <0x0 0x80000000
    0x4 0x0>`` is base 0x80000000 size 16 GiB, but read a cell at a time it
    looks like base 0x0 size 2 GiB.
    """
    stride = address_cells + size_cells
    if stride <= 0:
        return []
    def join(words: list[int]) -> int:
        value = 0
        for word in words:
            value = (value << 32) | word
        return value
    return [
        (
            join(cells[i : i + address_cells]),
            join(cells[i + address_cells : i + stride]),
        )
        for i in range(0, len(cells) - stride + 1, stride)
    ]


def _memory_regions(root: "DtsNode") -> list["MemoryRegion"]:
    """Every ``memory@`` node, decoded with the cell widths in force where it sits."""
    out: list[MemoryRegion] = []

    def visit(node: "DtsNode", inherited: tuple[int, int]) -> None:
        widths = _inherited_widths(node, inherited)
        for child in node.children:
            if child.name.startswith("memory@"):
                for base, size in _decode_reg(child.cells("reg"), *widths):
                    out.append(
                        MemoryRegion(
                            base=base,
                            size=size,
                            status=child.string("status") or "okay",
                        )
                    )
            visit(child, widths)

    # The device-tree spec's default when a parent says nothing is 2/2.
    visit(root, (2, 2))
    out.sort(key=lambda m: m.base)
    return out


def _end_of_string(text: str, start: int) -> int:
    """Index just past the closing quote of the string opening at `start`."""
    i = start + 1
    while i < len(text):
        if text[i] == "\\":
            i += 2
            continue
        if text[i] == '"':
            return i + 1
        i += 1
    return len(text)


def _strip_comments(text: str) -> str:
    """Remove /* */ and // comments without touching quoted strings."""
    out: list[str] = []
    i, n = 0, len(text)
    while i < n:
        char = text[i]
        if char == '"':
            end = _end_of_string(text, i)
            out.append(text[i:end])
            i = end
            continue
        if char == "/" and i + 1 < n and text[i + 1] == "*":
            end = text.find("*/", i + 2)
            i = n if end < 0 else end + 2
            out.append(" ")
            continue
        if char == "/" and i + 1 < n and text[i + 1] == "/":
            end = text.find("\n", i)
            i = n if end < 0 else end
            continue
        out.append(char)
        i += 1
    return "".join(out)


def _node_from_header(header: str) -> DtsNode:
    label: str | None = None
    if ":" in header:
        raw_label, _, header = header.partition(":")
        label = raw_label.strip() or None
    return DtsNode(name=header.strip(), label=label)


def _parse_body(text: str, i: int, parent: DtsNode) -> int:
    """Consume one node body starting at `i`; return the index past its close."""
    buffer: list[str] = []
    n = len(text)
    while i < n:
        char = text[i]
        if char == '"':
            end = _end_of_string(text, i)
            buffer.append(text[i:end])
            i = end
            continue
        if char == "{":
            child = _node_from_header("".join(buffer))
            buffer = []
            parent.children.append(child)
            i = _parse_body(text, i + 1, child)
            continue
        if char == "}":
            i += 1
            while i < n and text[i].isspace():
                i += 1
            if i < n and text[i] == ";":
                i += 1
            return i
        if char == ";":
            statement = "".join(buffer).strip()
            buffer = []
            i += 1
            if not statement or statement.startswith("/dts-"):
                continue
            name, sep, value = statement.partition("=")
            parent.properties[name.strip()] = value.strip() if sep else ""
            continue
        buffer.append(char)
        i += 1
    return i


def parse_dts(text: str) -> DtsNode:
    """Parse a flattened device tree into a node tree.

    This is deliberately a scanner rather than a full DTS implementation: the
    fingerprint needs node names, labels, and property text, and nothing about
    ``/include/``, overlays, or ``/delete-node/`` appears in the files Chipyard
    emits.
    """
    root = DtsNode(name="")
    _parse_body(_strip_comments(text), 0, root)
    # A well-formed DTS wraps everything in a single "/" node.
    for child in root.children:
        if child.name == "/":
            return child
    return root


# ---------------------------------------------------------------------------
# ISA strings
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Isa:
    """The parts of a ``riscv,isa`` string that change how code executes."""

    raw: str
    xlen: int | None
    base: tuple[str, ...]
    extensions: tuple[str, ...]

    @property
    def vlen_bits(self) -> int | None:
        """Vector register length in bits, from ``zvl<N>b``."""
        for ext in self.extensions:
            match = _ZVL.match(ext)
            if match:
                return int(match.group(1))
        return None

    @property
    def has_vector(self) -> bool:
        """Whether this hart has a vector unit at all.

        The ``v`` base letter implies the full V extension; an embedded profile
        advertises ``zve*`` instead, with ``zvl*`` fixing the register width.
        A hart with none of these cannot run RVV code, whatever the profile
        directory it is paired with happens to be called.
        """
        return "v" in self.base or any(
            e.startswith("zve") or e.startswith("zvl") for e in self.extensions
        )


def parse_isa(raw: str) -> Isa:
    """Split a ``riscv,isa`` string into base letters and multi-letter extensions."""
    text = (raw or "").strip().lower()
    match = _ISA_XLEN.match(text)
    xlen = int(match.group(1)) if match else None
    body = text[match.end():] if match else text

    tokens = [t for t in body.split("_") if t]
    base: list[str] = []
    extensions: list[str] = []
    if tokens:
        head = tokens[0]
        cut = 0
        while cut < len(head) and head[cut] in _SINGLE_LETTER_EXTENSIONS:
            cut += 1
        base = list(head[:cut])
        # Chipyard appends the first multi-letter extension with no separator.
        if head[cut:]:
            extensions.append(head[cut:])
        extensions.extend(tokens[1:])
    return Isa(raw=raw, xlen=xlen, base=tuple(base), extensions=tuple(extensions))


# ---------------------------------------------------------------------------
# Accelerator shape, from generated Verilog
# ---------------------------------------------------------------------------


def _module_bodies(collateral: Path) -> dict[str, list[str]]:
    """Map module name -> its body lines, across every generated source file."""
    bodies: dict[str, list[str]] = {}
    for path in sorted(collateral.iterdir()):
        if path.suffix not in (".sv", ".v") or not path.is_file():
            continue
        try:
            text = path.read_text(errors="replace")
        except OSError:
            continue
        starts = [(m.start(), m.group(1)) for m in _MODULE_DECL.finditer(text)]
        for index, (offset, name) in enumerate(starts):
            end = starts[index + 1][0] if index + 1 < len(starts) else len(text)
            bodies.setdefault(name, text[offset:end].splitlines())
    return bodies


def _instantiation_counts(bodies: dict[str, list[str]]) -> dict[str, dict[str, int]]:
    """For each module, how many times it instantiates each other module."""
    known = set(bodies)
    counts: dict[str, dict[str, int]] = {}
    for name, lines in bodies.items():
        local: dict[str, int] = {}
        for line in lines:
            match = _INSTANCE.match(line)
            if not match:
                continue
            child = match.group(1)
            if child in _NOT_A_MODULE or child not in known or child == name:
                continue
            local[child] = local.get(child, 0) + 1
        counts[name] = local
    return counts


def count_instances(collateral: Path, top: str, leaf: str) -> int | None:
    """Total instances of `leaf` beneath `top`, or None if either is absent.

    Walks the instantiation graph with memoization. Chipyard emits one flat
    directory of modules with no generate-block collapsing left to do, so a
    per-line instantiation match is exact — verified against the measured DSP
    law, where a Gemmini mesh costs one DSP48E2 per cell: 256 counted cells for
    ``GemminiKU040Config`` and 1024 for ``Q31Ws32x32AccGemminiKU040Config``
    reproduce the +261 and +1029 deltas (cells plus five hardfloat FMAs).
    """
    bodies = _module_bodies(collateral)
    if top not in bodies or leaf not in bodies:
        return None
    counts = _instantiation_counts(bodies)

    memo: dict[str, int] = {}
    visiting: set[str] = set()

    def total(module: str) -> int:
        if module == leaf:
            return 1
        if module in memo:
            return memo[module]
        if module in visiting:  # defensive: the graph is a DAG in practice
            return 0
        visiting.add(module)
        found = sum(n * total(child) for child, n in counts.get(module, {}).items())
        visiting.discard(module)
        memo[module] = found
        return found

    return total(top)


def detect_accelerators(collateral: Path, top: str | None = None) -> dict[str, dict[str, Any]]:
    """Which accelerators the generated Verilog contains, and their shape."""
    if not collateral.is_dir():
        return {}
    present = {p.stem for p in collateral.iterdir() if p.suffix in (".sv", ".v")}
    found: dict[str, dict[str, Any]] = {}
    for name, markers, leaf, unit, features in _ACCELERATOR_MARKERS:
        if not all(m in present for m in markers):
            continue
        shape: dict[str, Any] = {"modules": list(markers)}
        if leaf and top:
            cells = count_instances(collateral, top, leaf)
            if cells:
                shape[unit] = cells
        shape.update(disabled_features(present, features))
        found[name] = shape
    return found


def disabled_features(
    present: set[str], features: tuple[tuple[str, str], ...]
) -> dict[str, list[str]]:
    """The stock accelerator features this design is missing, if any.

    Only deviations are recorded, and the key is omitted entirely when nothing is
    missing. That keeps the shape dict -- and so `identity_id`, which hashes it
    whole -- byte-identical for every fully-featured machine already on record,
    while still separating a stripped variant from the stock one.

    Shared with `soc.py`, so a per-tile accelerator shape carries the same
    features as the whole-design one and the two stay comparable.
    """
    missing = sorted(key for key, module in features if module not in present)
    return {"disabled_features": missing} if missing else {}


# ---------------------------------------------------------------------------
# The fingerprint
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class MemoryRegion:
    base: int
    size: int
    status: str

    def as_dict(self) -> dict[str, Any]:
        return {"base": hex(self.base), "size": self.size, "status": self.status}


@dataclass(frozen=True)
class Fingerprint:
    """What a machine is, in the terms that decide whether results transfer."""

    evidence: str
    harts: tuple[dict[str, Any], ...]
    accelerators: dict[str, dict[str, Any]]
    memory: tuple[MemoryRegion, ...]
    clocks_mhz: dict[str, float]
    timebase_hz: int | None = None
    source: dict[str, Any] = field(default_factory=dict)

    # -- derived ------------------------------------------------------------

    @property
    def hart_count(self) -> int:
        return len(self.harts)

    @property
    def isa_by_hart(self) -> tuple[str, ...]:
        return tuple(str(h.get("isa", "")) for h in self.harts)

    @property
    def has_vector(self) -> bool:
        return any(bool(h.get("has_vector")) for h in self.harts)

    @property
    def vlen_bits(self) -> int | None:
        widths = [h.get("vlen_bits") for h in self.harts if h.get("vlen_bits")]
        return max(widths) if widths else None

    @property
    def clock_mhz(self) -> float | None:
        """The single bus clock, or None when the buses disagree.

        Returning None rather than a guess is deliberate: a multi-clock SoC has
        no one frequency to convert cycles at, and picking one silently would
        reintroduce exactly the error this module exists to prevent.
        """
        distinct = set(self.clocks_mhz.values())
        if len(distinct) == 1:
            return distinct.pop()
        return None

    @property
    def identity(self) -> dict[str, Any]:
        """The cycle-affecting facts. Clock is excluded — cycles are invariant."""
        return {
            "harts": [
                {
                    "isa": h.get("isa"),
                    "has_vector": h.get("has_vector"),
                    "vlen_bits": h.get("vlen_bits"),
                }
                for h in self.harts
            ],
            "accelerators": self.accelerators,
            "memory": [m.as_dict() for m in self.memory],
        }

    @property
    def identity_id(self) -> str:
        blob = json.dumps(self.identity, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(blob.encode()).hexdigest()[:12]

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema": SCHEMA,
            "evidence": self.evidence,
            "identity_id": self.identity_id,
            "harts": list(self.harts),
            "hart_count": self.hart_count,
            "accelerators": self.accelerators,
            "memory": [m.as_dict() for m in self.memory],
            "clocks_mhz": self.clocks_mhz,
            "clock_mhz": self.clock_mhz,
            "timebase_hz": self.timebase_hz,
            "source": self.source,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Fingerprint":
        # Descriptors cite their sources inline with `_`-prefixed keys. Those are
        # commentary, not facts about the machine, so they are dropped before
        # anything hashes or compares an accelerator shape.
        accelerators = {
            name: {k: v for k, v in shape.items() if not k.startswith("_")}
            for name, shape in data.get("accelerators", {}).items()
        }
        return cls(
            evidence=data.get("evidence", "declared"),
            harts=tuple(data.get("harts", ())),
            accelerators=accelerators,
            memory=tuple(
                MemoryRegion(
                    base=int(str(m["base"]), 0), size=int(m["size"]), status=m.get("status", "okay")
                )
                for m in data.get("memory", ())
            ),
            clocks_mhz=data.get("clocks_mhz", {}),
            timebase_hz=data.get("timebase_hz"),
            source=data.get("source", {}),
        )

    def summary(self) -> str:
        """One line, for a terminal."""
        accelerators = ", ".join(
            f"{name}({shape.get('mesh_cells') or shape.get('opu_cells') or '?'})"
            for name, shape in sorted(self.accelerators.items())
        )
        vector = f"VLEN={self.vlen_bits}" if self.has_vector else "no vector"
        clock = f"{self.clock_mhz:g} MHz" if self.clock_mhz else "mixed clocks"
        return (
            f"{self.hart_count} hart(s), {vector}, "
            f"{accelerators or 'no accelerator'}, {clock} [{self.identity_id}]"
        )


def from_generated_src(generated_dir: Path, top: str | None = None) -> Fingerprint:
    """Extract a fingerprint from one Chipyard elaboration directory."""
    generated_dir = Path(generated_dir)
    candidates = sorted(generated_dir.glob("*.dts"))
    if not candidates:
        raise FingerprintError(
            f"no .dts under {generated_dir}; elaborate the config first "
            "(rb area run, or make -C fpga SUB_PROJECT=... CONFIG=...)"
        )
    dts_path = candidates[0]
    tree = parse_dts(dts_path.read_text())

    harts: list[dict[str, Any]] = []
    for node in tree.find("cpu@"):
        if node.string("device_type") != "cpu":
            continue
        isa = parse_isa(node.string("riscv,isa") or "")
        harts.append(
            {
                "index": node.cell("reg"),
                "isa": isa.raw,
                "xlen": isa.xlen,
                "extensions": list(isa.extensions),
                "has_vector": isa.has_vector,
                "vlen_bits": isa.vlen_bits,
                "status": node.string("status") or "okay",
                "compatible": node.strings("compatible"),
            }
        )
    harts.sort(key=lambda h: (h["index"] is None, h["index"]))

    memory = _memory_regions(tree)

    clocks: dict[str, float] = {}
    for node in tree.walk():
        if not node.name.endswith("_clock"):
            continue
        hz = node.cell("clock-frequency")
        if hz:
            clocks[node.name] = hz / 1e6

    timebase = None
    for node in tree.walk():
        value = node.cell("timebase-frequency")
        if value:
            timebase = value
            break

    collateral = generated_dir / "gen-collateral"
    accelerators = detect_accelerators(collateral, top=top)

    return Fingerprint(
        evidence="elaborated",
        harts=tuple(harts),
        accelerators=accelerators,
        memory=tuple(memory),
        clocks_mhz=clocks,
        timebase_hz=timebase,
        source={
            "dts": str(dts_path),
            "gen_collateral": str(collateral) if collateral.is_dir() else None,
            "top": top,
        },
    )


def for_board(board: Any) -> Fingerprint:
    """Extract the fingerprint of a `boards.Board` with a config selected."""
    return from_generated_src(board.generated_dir, top=board.harness)


# ---------------------------------------------------------------------------
# Comparison
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Divergence:
    """One way two machines differ, and what that difference invalidates."""

    field: str
    profile: Any
    config: Any
    affects: str  # "cycles" or "wall-time"

    def __str__(self) -> str:
        return f"{self.field}: profile={self.profile!r} config={self.config!r} (affects {self.affects})"


# G is shorthand for IMAFD plus Zicsr and Zifencei. Expanding it keeps a
# registry shorthand ("rv64gcv") comparable with an elaborated ISA string
# ("rv64imafdcb...zicsr..."), which would otherwise appear to disagree about
# every base extension the two actually share.
_G_EXPANSION = frozenset({"i", "m", "a", "f", "d", "zicsr", "zifencei"})


def _extension_set(fp: Fingerprint) -> frozenset[str]:
    """Every ISA extension any hart advertises, normalized for comparison."""
    out: set[str] = set()
    for hart in fp.harts:
        isa = parse_isa(str(hart.get("isa", "")))
        out.update(isa.base)
        out.update(isa.extensions)
        # A declared fingerprint may list extensions the shorthand omits.
        out.update(str(e).lower() for e in hart.get("extensions", ()) or ())
    if "g" in out:
        out.discard("g")
        out |= _G_EXPANSION
    return frozenset(out)


def compare(profile: Fingerprint, config: Fingerprint) -> list[Divergence]:
    """Every way the machine a profile was captured on differs from a config.

    Ordered so cycle-affecting divergences come first: those invalidate the
    numbers, while a clock difference only changes the unit they are reported
    in.

    ISA is compared as a set of extensions rather than as a raw string, because
    the two sides do not always come from the same kind of source — an
    elaborated config yields a canonical ``riscv,isa``, whereas a profiling
    machine may only be described by a registry shorthand like ``rv64gcv``.
    Comparing sets keeps the report to what actually differs instead of two
    incomparable strings.
    """
    out: list[Divergence] = []
    if profile.hart_count != config.hart_count:
        out.append(Divergence("harts", profile.hart_count, config.hart_count, "cycles"))
    if profile.has_vector != config.has_vector:
        out.append(Divergence("vector", profile.has_vector, config.has_vector, "cycles"))
    elif profile.vlen_bits != config.vlen_bits:
        out.append(Divergence("vlen_bits", profile.vlen_bits, config.vlen_bits, "cycles"))

    profile_extensions, config_extensions = _extension_set(profile), _extension_set(config)
    if profile_extensions != config_extensions:
        out.append(
            Divergence(
                "isa_extensions",
                sorted(profile_extensions - config_extensions),
                sorted(config_extensions - profile_extensions),
                "cycles",
            )
        )
    if profile.accelerators != config.accelerators:
        out.append(
            Divergence(
                "accelerators",
                {k: v for k, v in sorted(profile.accelerators.items())},
                {k: v for k, v in sorted(config.accelerators.items())},
                "cycles",
            )
        )
    profile_memory = [m.as_dict() for m in profile.memory]
    config_memory = [m.as_dict() for m in config.memory]
    if profile_memory != config_memory:
        out.append(Divergence("memory", profile_memory, config_memory, "cycles"))
    if profile.clock_mhz != config.clock_mhz:
        out.append(Divergence("clock_mhz", profile.clock_mhz, config.clock_mhz, "wall-time"))
    return out


def agreement(divergences: Sequence[Divergence]) -> str:
    """``exact`` when cycles transfer unchanged, ``extrapolated`` when they do not.

    A clock-only difference is still ``exact``: cycles are invariant to
    frequency, and the conversion to wall time happens once, downstream.
    """
    return "extrapolated" if any(d.affects == "cycles" for d in divergences) else "exact"
