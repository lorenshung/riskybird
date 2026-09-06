"""Turn an extracted fingerprint into the descriptors the software flow needs.

The scheduler and the modelblaster harness each need to be told what machine
they are targeting. Today that is written by hand in three places -- the xpurt
spec's ``hardware`` block, a ``modelblaster/cores/*.json`` registry, and the
profile directory names -- and none of them is checked against the Chipyard
config they claim to describe. This module derives all three from the
elaboration output instead, so they cannot disagree with the bitstream.

**Nothing here is keyed on a config name.** There is no table mapping
``Q31Ws32x32AccGemminiKU040Config`` to anything. Every fact comes from the DTS
and ``gen-collateral``, so a config written tomorrow works without touching this
file. Adding a new accelerator *family* is one entry in
``fingerprint._ACCELERATOR_MARKERS``; adding a new *config* is nothing.

Accelerators attach to harts, they do not create them. Saturn is a vector unit
inside a Rocket tile, Gemmini is a RoCC coprocessor on that tile's port, and the
Saturn OPU sits inside Saturn's own vector backend -- so scalar, vector, and
matrix can all live on one hart, and every KU040 config measured so far is
exactly that. Hart count comes from the DTS and nothing else.

A **machine class** is a group of harts that are the same hardware. Classes come
from tile modules: Chipyard emits one module per distinct tile and instantiates
it once per hart of that kind, so two identical Rocket+Saturn tiles are one
class with two cores, while an asymmetric pair emits ``RocketTile`` and
``RocketTile_1`` and becomes two classes.

Each class names itself with a **slug** derived from its own hardware, e.g.
``novec-gemmini1024``. That is what a profile directory is called. It replaces
the old scheme where ``gemmini_q31`` and ``V256D128_rvv`` were names a person
chose, unverifiable against any config: a slug is computed, so a machine we have
already profiled is recognised automatically and one we have not is obviously
absent. Profiles captured before this existed keep their names through an
explicit alias in the profile-target descriptor, not through a rule here.
"""

from __future__ import annotations

import hashlib
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

from . import fingerprint as fingerprint_mod
from .fingerprint import Fingerprint, count_instances, parse_isa


# Downstream tooling recognises exactly two core labels: `ingest_xpurt_schedule.py`
# resolves "CPU_P#<n>"/"CPU_E#<n>" and nothing else, and `profile_loader` takes a
# `cpu_p_profile_hw` and a `cpu_e_profile_hw`. A machine with three distinct hart
# types cannot be expressed, so it is refused rather than silently truncated.
CORE_LABELS = ("CPU_P", "CPU_E")

# A tile is the unit a hart is built from. Chipyard names them <Core>Tile, and
# emits one module per *distinct* tile configuration.
_TILE_MODULE = re.compile(r"\A([A-Z]\w*?)Tile(_\d+)?\Z")

# Tile-like modules that are infrastructure rather than a core.
_NOT_A_TILE = frozenset({"TileClockGater", "TilePRCIDomain", "TileResetSetter"})


class SocError(RuntimeError):
    """The elaborated machine cannot be expressed to the software flow."""


# ---------------------------------------------------------------------------
# Machine classes
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class MachineClass:
    """One kind of hart, and how many of them the SoC has."""

    slug: str
    cores: int
    harts: tuple[int, ...]
    isa: str
    has_vector: bool
    vlen_bits: int | None
    accelerators: dict[str, dict[str, Any]]
    tile_module: str | None = None

    @property
    def is_matrix(self) -> bool:
        """Whether this class carries a matrix engine.

        Used only to decide which class is the performance core when there are
        two; it is a labelling choice, not a capability claim.
        """
        return any(
            "mesh_cells" in shape or "opu_cells" in shape
            for shape in self.accelerators.values()
        )

    def describe(self) -> str:
        vector = f"vec{self.vlen_bits}" if self.has_vector else "no vector"
        accel = ", ".join(
            f"{name} {shape.get('mesh_cells') or shape.get('opu_cells') or '?'} cells"
            for name, shape in sorted(self.accelerators.items())
        )
        return f"{self.cores}x [{vector}, {accel or 'no accelerator'}] -> {self.slug}"


def machine_slug(
    has_vector: bool,
    vlen_bits: int | None,
    accelerators: dict[str, dict[str, Any]],
    isa: str = "",
) -> str:
    """A stable, readable name for one kind of hart.

    Readable because it becomes a profile directory someone has to recognise;
    hashed at the tail because the readable part deliberately omits ISA detail
    that still changes cycle counts (bitmanip, zfh, and so on). Two harts with
    the same slug are the same machine, so a profile captured on one is valid on
    the other.
    """
    parts = [f"vec{vlen_bits}" if has_vector and vlen_bits else ("vec" if has_vector else "novec")]
    for name, shape in sorted(accelerators.items()):
        cells = shape.get("mesh_cells") or shape.get("opu_cells")
        parts.append(f"{name.replace('-', '')}{cells}" if cells else name.replace("-", ""))
    digest = hashlib.sha256(
        "|".join([isa, repr(sorted(accelerators.items()))]).encode()
    ).hexdigest()[:6]
    return "-".join(parts + [digest])


def _tile_modules(collateral: Path) -> dict[str, int]:
    """Map tile module name -> how many times the design instantiates it."""
    from .fingerprint import _instantiation_counts, _module_bodies  # noqa: PLC0415

    bodies = _module_bodies(collateral)
    counts = _instantiation_counts(bodies)
    tiles = {
        name for name in bodies
        if _TILE_MODULE.match(name) and name not in _NOT_A_TILE
    }
    totals: dict[str, int] = {}
    for tile in tiles:
        # A tile may be instantiated inside a PRCI domain rather than at top
        # level, so total across every parent rather than assuming one site.
        totals[tile] = sum(local.get(tile, 0) for local in counts.values()) or 1
    return totals


def _accelerators_in(collateral: Path, tile: str) -> dict[str, dict[str, Any]]:
    """Accelerator shapes reachable from one tile module."""
    found: dict[str, dict[str, Any]] = {}
    present = {p.stem for p in collateral.iterdir() if p.suffix in (".sv", ".v")}
    for name, markers, leaf, unit, features in fingerprint_mod._ACCELERATOR_MARKERS:
        if not leaf:
            continue
        cells = count_instances(collateral, tile, leaf)
        if cells:
            shape: dict[str, Any] = {"modules": list(markers), unit: cells}
            # Same feature record the whole-design fingerprint carries, so a
            # per-tile shape and `fp.accelerators` remain comparable.
            shape.update(fingerprint_mod.disabled_features(present, features))
            found[name] = shape
    return found


def machine_classes(fp: Fingerprint) -> list[MachineClass]:
    """Group a fingerprint's harts into distinct machine classes.

    For a single-hart SoC this is one class carrying every accelerator, which is
    exact: there is nowhere else for them to be. For several harts the split
    comes from tile modules, and if their instantiation counts do not add up to
    the DTS hart count the attribution is refused rather than guessed -- an
    incorrect topology produces a benchmark for a machine that was never built.
    """
    if not fp.harts:
        raise SocError("fingerprint has no harts; was the config elaborated?")

    collateral = fp.source.get("gen_collateral")
    top = fp.source.get("top")

    if len(fp.harts) == 1:
        hart = fp.harts[0]
        isa = parse_isa(str(hart.get("isa", "")))
        return [
            MachineClass(
                slug=machine_slug(isa.has_vector, isa.vlen_bits, fp.accelerators, isa.raw),
                cores=1,
                harts=(int(hart.get("index") or 0),),
                isa=isa.raw,
                has_vector=isa.has_vector,
                vlen_bits=isa.vlen_bits,
                accelerators=fp.accelerators,
            )
        ]

    if not collateral or not top:
        raise SocError(
            f"{len(fp.harts)} harts but no gen-collateral recorded, so accelerators "
            "cannot be attributed to harts. Re-extract the fingerprint from the "
            "elaboration directory."
        )

    tiles = _tile_modules(Path(collateral))
    if not tiles:
        raise SocError(
            f"{len(fp.harts)} harts but no tile modules found under {collateral}. "
            "The tile naming convention may have changed; attribution is refused "
            "rather than guessed."
        )
    if sum(tiles.values()) != len(fp.harts):
        raise SocError(
            f"tile instantiations {tiles} sum to {sum(tiles.values())} but the DTS "
            f"declares {len(fp.harts)} harts. Refusing to guess which hart carries "
            "which accelerator -- state the topology explicitly instead."
        )

    # Harts appear in the DTS in reg order; tiles in module order. Pair them by
    # walking both in a stable order.
    classes: list[MachineClass] = []
    remaining = list(fp.harts)
    for tile, count in sorted(tiles.items()):
        group, remaining = remaining[:count], remaining[count:]
        isa = parse_isa(str(group[0].get("isa", "")))
        accelerators = _accelerators_in(Path(collateral), tile)
        classes.append(
            MachineClass(
                slug=machine_slug(isa.has_vector, isa.vlen_bits, accelerators, isa.raw),
                cores=count,
                harts=tuple(int(h.get("index") or i) for i, h in enumerate(group)),
                isa=isa.raw,
                has_vector=isa.has_vector,
                vlen_bits=isa.vlen_bits,
                accelerators=accelerators,
                tile_module=tile,
            )
        )

    # Identical tiles that Chipyard happened to emit separately collapse here.
    merged: dict[str, MachineClass] = {}
    for cls in classes:
        if cls.slug in merged:
            was = merged[cls.slug]
            merged[cls.slug] = MachineClass(
                slug=was.slug, cores=was.cores + cls.cores,
                harts=was.harts + cls.harts, isa=was.isa,
                has_vector=was.has_vector, vlen_bits=was.vlen_bits,
                accelerators=was.accelerators, tile_module=was.tile_module,
            )
        else:
            merged[cls.slug] = cls
    return list(merged.values())


# ---------------------------------------------------------------------------
# The descriptors the software flow consumes
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SocPlan:
    """Everything the scheduler and the harness need, derived from one config."""

    config: str
    classes: tuple[MachineClass, ...]
    labels: dict[str, MachineClass]  # CPU_P / CPU_E -> class

    @property
    def machines(self) -> dict[str, int]:
        """The xpurt ``machines`` block: how many cores of each label."""
        return {label: cls.cores for label, cls in self.labels.items()}

    @property
    def profile_hw(self) -> dict[str, str]:
        """The xpurt ``profile_hw`` block: which profile each label reads."""
        return {label.lower(): cls.slug for label, cls in self.labels.items()}

    @property
    def slugs(self) -> tuple[str, ...]:
        return tuple(cls.slug for cls in self.classes)

    def describe(self) -> list[str]:
        return [f"  {label}: {cls.describe()}" for label, cls in self.labels.items()]


def plan(fp: Fingerprint, config: str) -> SocPlan:
    """Assign machine classes to the two core labels the toolchain understands.

    When there are two classes the matrix-bearing one becomes ``CPU_P``, which
    matches how the existing hand-written registries were laid out. With one
    class every core is ``CPU_P`` and there is no ``CPU_E`` -- a single-hart SoC
    genuinely has nowhere else to put work, and saying so is the point.
    """
    classes = machine_classes(fp)
    if len(classes) > len(CORE_LABELS):
        raise SocError(
            f"{config} has {len(classes)} distinct hart types "
            f"({', '.join(c.slug for c in classes)}), but the schedule ingest and "
            f"profile loader recognise only {', '.join(CORE_LABELS)}. Expressing "
            "this machine needs an upstream change, not a workaround here."
        )
    ordered = sorted(classes, key=lambda c: (not c.is_matrix, c.slug))
    labels = {CORE_LABELS[i]: cls for i, cls in enumerate(ordered)}
    return SocPlan(config=config, classes=tuple(ordered), labels=labels)


def cores_registry(soc: SocPlan, capabilities: Sequence[str]) -> dict[str, Any]:
    """A modelblaster cores registry describing this SoC.

    `ingest_xpurt_schedule.py` resolves ``CPU_P#<n>`` to the n-th core of a kind
    and pins the runtime worker to that core's hart, so these hart ids must be
    the ones the bitstream actually has -- which is why they come from the DTS.

    Every core is given the same capability set. The shipped registries list
    only an accelerator's fast paths (``gemmini_q31_0`` claims just
    ``linear_s8`` and ``conv2d_s8``), but each of these harts is a Rocket that
    executes anything in software; the accelerator only makes some ops faster.
    Encoding routing preference as capability would make a single-hart SoC look
    incapable of running a workload it runs fine, so preference is left where it
    belongs -- in the measured per-op times.
    """
    cores = []
    for label, cls in soc.labels.items():
        for index, hart in enumerate(cls.harts):
            cores.append({
                "name": f"{cls.slug}_{index}",
                "kind": cls.slug,
                "isa": cls.isa,
                "harts": [hart],
                "capabilities": list(capabilities),
                "description": f"{label}#{index}: {cls.describe()}",
            })
    return {
        "system": soc.config,
        "description": (
            f"Derived from the elaborated {soc.config}: "
            + "; ".join(c.describe() for c in soc.classes)
        ),
        "default_dispatch_target": "any",
        "cores": cores,
    }


# ---------------------------------------------------------------------------
# Gating
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Rejection:
    """A reason this machine cannot run this workload as configured."""

    reason: str
    detail: str

    def __str__(self) -> str:
        return f"{self.reason}: {self.detail}"


def check_backends(soc: SocPlan, requested: dict[str, str]) -> list[Rejection]:
    """Refuse a profile backend the hardware cannot execute.

    The case this exists for: pairing a vector profile with a hart whose ISA
    carries no ``zvl*``. Half such a profile describes instructions that will
    trap, and no amount of retiming makes the resulting number mean anything.
    Name matching cannot catch it because the profile is only a directory name.
    """
    out: list[Rejection] = []
    for label, backend in requested.items():
        cls = soc.labels.get(label.upper())
        if cls is None:
            out.append(Rejection(
                "no such core",
                f"{backend!r} was requested for {label.upper()}, but {soc.config} "
                f"has only {', '.join(soc.labels)}",
            ))
            continue
        if _looks_vector(backend) and not cls.has_vector:
            out.append(Rejection(
                "no vector unit",
                f"{label.upper()} wants {backend!r} but its ISA carries no zvl* "
                f"({cls.isa})",
            ))
    return out


def _looks_vector(backend: str) -> bool:
    """Whether a profile directory name denotes a vector backend.

    A heuristic over a human-chosen string, which is exactly why derived slugs
    are better: for those the answer is structural, not spelled.
    """
    lowered = backend.lower()
    return "rvv" in lowered or lowered.startswith("vec") or "vector" in lowered
