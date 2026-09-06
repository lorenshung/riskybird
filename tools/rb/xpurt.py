"""Drive the XPU-RT schedule-and-plot flow from `rb`.

The XPU-RT scheduler derives its repository base from its own script location
(``abspath(__file__)/../..``) and then looks for dispatch graphs and per-op
profiles beneath it. Nothing overrides that base, and the profiles we need are
split across two backends at two different layouts.

Rather than patch a backend or write into its tracked ``gen/`` tree, this module
assembles a **symlinked mirror**: a RiskyBird-owned directory whose ``scripts``
and ``xpu-rt`` entries point at the backend, and whose ``gen``, ``data``,
``schedules``, and ``plots`` we own outright. ``abspath`` does not resolve
symlinks, so running the backend's script through the mirror makes the mirror the
repository base. XPU-RT stays byte-clean.

The scheduler needs only numpy and matplotlib for both the ``greedy`` and
``decomposed`` solvers -- cvxpy and mosek are confined to the MILP path, which
``decomposed_schedule`` does not use. The Chipyard Conda environment already
carries numpy 1.26 (the ``<2`` the scheduler wants) and matplotlib, so no new
environment is required.
"""

from __future__ import annotations

import csv
import hashlib
import html
import json
import os
import re
import shutil
import signal
import subprocess
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Sequence

from .boards import ROOT


XPURT = ROOT / "backends" / "XPU-RT"
ZEPHYR_GEN = ROOT / "backends" / "zephyr-chipyard-sw" / "gen"
WORKSPACE = ROOT / "dse" / "results" / "xpurt"
DSE_PLOTS = ROOT / "dse" / "plots"
WORKLOAD_DIR = ROOT / "dse" / "workloads"
PROFILE_TARGET_DIR = ROOT / "dse" / "profile-targets"
FOREST_PILOT = XPURT / "sims" / "scripts" / "pilot" / "pilot_forest_with_dronet_scheduled.py"
FOREST_LAUNCHER = ROOT / "tools" / "rb" / "xpurt_pilot_launcher.py"
FOREST_SCHEDULE = (
    XPURT / "schedules"
    / "scheduled_networks_periodic_dronet50ms_yolov8_firesim_greedy_profiled.json"
)
FOREST_ISAACLAB = XPURT / "sims" / "IsaacLab"
FOREST_RUNS = ROOT / "results" / "runs"
XPURT_ENV = ROOT / ".rb" / "envs" / "xpurt"

SOLVERS = ("greedy", "greedy_periodic", "decomposed")

# Roots that may contain a gen/profile tree, in precedence order. The chipyard
# FireSim sweeps landed under zephyr-chipyard-sw with an extra sweep_* level.
PROFILE_ROOTS = (XPURT / "gen" / "profile", ZEPHYR_GEN / "profile")
GRAPH_ROOTS = (XPURT / "gen" / "vmfb", ZEPHYR_GEN / "vmfb", XPURT / "gen" / "qnn_vmfb")


class XpurtError(RuntimeError):
    """A recoverable failure in the XPU-RT flow."""


@dataclass(frozen=True)
class Profile:
    """One per-op timing profile: (hw, target, model, basename) -> results.csv."""

    hw: str
    target: str
    model: str
    basename: str
    results: Path

    @property
    def key(self) -> tuple[str, str, str, str]:
        return (self.hw, self.target, self.model, self.basename)


@dataclass(frozen=True)
class Graph:
    """A dispatch graph for one (model, target, backend, precision)."""

    model: str
    target: str
    backend: str
    basename: str
    path: Path


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------


def _profile_results(basename_dir: Path) -> Path | None:
    """Locate results.csv beneath a <basename> directory, either layout.

    The loader globs two shapes and prefers the newest match:
        <basename>/<input_tag>/<topo>/results.csv    (current)
        <basename>/<topo>/results.csv                (back-compat)
    """
    matches = sorted(
        list(basename_dir.glob("*/*/results.csv")) + list(basename_dir.glob("*/results.csv")),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    return matches[0] if matches else None


def discover_profiles() -> list[Profile]:
    """Find every per-op profile under any known root.

    Rather than parse paths from the tail -- which the optional sweep-tag and
    input-tag levels make ambiguous -- this walks candidate
    ``<hw>/<target>/<model>/<basename>`` directories at both the root itself and
    one level below it (the chipyard FireSim sweeps sit under a ``sweep_*`` tag),
    and keeps those with a results.csv underneath.
    """
    found: dict[tuple[str, str, str, str], Profile] = {}
    for root in PROFILE_ROOTS:
        if not root.is_dir():
            continue
        # Descend a level only for a sweep tag. Descending into every directory
        # also re-reads the untagged tree one level too deep, where
        # <hw>/<target>/<model>/<basename>/<input_tag>/<topo>/results.csv
        # re-parses as <target>/<model>/<basename>/<input_tag>, inventing
        # targets like "mlp_control" that no profile was ever captured on.
        bases = [root] + [
            d for d in sorted(root.iterdir()) if d.is_dir() and d.name.startswith("sweep")
        ]
        for base in bases:
            for hw_dir in sorted(p for p in base.iterdir() if p.is_dir()):
                for target_dir in sorted(p for p in hw_dir.iterdir() if p.is_dir()):
                    for model_dir in sorted(p for p in target_dir.iterdir() if p.is_dir()):
                        for basename_dir in sorted(p for p in model_dir.iterdir() if p.is_dir()):
                            results = _profile_results(basename_dir)
                            if results is None:
                                continue
                            key = (hw_dir.name, target_dir.name, model_dir.name, basename_dir.name)
                            found.setdefault(key, Profile(
                                hw=key[0], target=key[1], model=key[2], basename=key[3],
                                results=results,
                            ))
    return list(found.values())


def discover_graphs() -> list[Graph]:
    graphs: dict[str, Graph] = {}
    for root in GRAPH_ROOTS:
        if not root.is_dir():
            continue
        for path in sorted(root.rglob("*_dispatch_graph.json")):
            rel = path.relative_to(root).parts
            if len(rel) < 4:
                continue
            model, target, backend, basename = rel[0], rel[1], rel[2], rel[3]
            graphs.setdefault(str(path), Graph(model, target, backend, basename, path))
    return list(graphs.values())


def load_profile_target(target: str) -> "fingerprint.Fingerprint | None":
    """The declared fingerprint of the machine a profile target was captured on.

    These cannot be extracted: the profiling machine is a FireSim target that
    this tree never elaborates. The descriptors under ``dse/profile-targets/``
    therefore cite a source for every field and leave the rest null, so a
    comparison against an elaborated config reports what is genuinely unknown
    instead of asserting a match.
    """
    from . import fingerprint

    path = PROFILE_TARGET_DIR / f"{target}.json"
    if not path.is_file():
        return None
    return fingerprint.Fingerprint.from_dict(json.loads(path.read_text()))


def topo_tag_for_cores(cores: int) -> str:
    """The profile topo tag matching a core count: 2 cores -> ``topo_0_1``.

    Profiles are captured per core count because a parallel-for workload does
    not take the same cycles on one core as on four -- the recorded sweeps run
    275, 162, 120, 83 ms across one to four cores for the same network. Reading
    a ``topo_0`` profile for a two-core machine silently reports single-core
    timings for a machine that has two.
    """
    return "topo_" + "_".join(str(i) for i in range(max(cores, 1)))


def available_topos(prof: Profile) -> set[str]:
    """Every topo tag this profile actually has data for."""
    basename_dir = prof.results.parent.parent.parent
    found = set()
    for candidate in basename_dir.rglob("results.csv"):
        found.add(candidate.parent.name)
    return found


def profile_provenance(prof: Profile) -> dict[str, Any]:
    """Where a profile came from, and what shape it is.

    Two families of the same network live in this tree and disagree by ~4.6x --
    a 30-op modelblaster QNN capture carrying cycles, and a 15-op IREE capture
    without them. Only the first is discovered, but nothing said so, and a
    disagreement that leaves no trace in the output is the kind that gets
    noticed a long way downstream.
    """
    try:
        relative = prof.results.relative_to(ROOT)
    except ValueError:
        relative = prof.results
    parts = relative.parts
    with prof.results.open() as handle:
        rows = list(csv.DictReader(handle))
    return {
        "path": str(relative),
        "backend_root": "zephyr-chipyard-sw" if "zephyr-chipyard-sw" in parts else "XPU-RT",
        "sweep_tag": next((p for p in parts if p.startswith("sweep")), None),
        "topo": prof.results.parent.name,
        "available_topos": sorted(available_topos(prof)),
        "rows": len(rows),
        "ops": sorted({r["op"] for r in rows if r.get("op")}),
        "has_cycles": bool(rows and rows[0].get("cycles")),
    }


def workload_ops(target: str, model: str) -> set[str]:
    """Which operations a workload actually executes, from its profile.

    The dispatch graph carries only ids and dependencies, so the `op` column of
    a profile is the record of what the network really ran. Used to populate a
    derived cores registry with the ops the harness has to be able to place.
    """
    found: set[str] = set()
    for prof in discover_profiles():
        if prof.target != target or prof.model != model:
            continue
        with prof.results.open() as handle:
            for row in csv.DictReader(handle):
                if row.get("op"):
                    found.add(row["op"])
    return found


def profiles_for(target: str, hw: str) -> dict[str, Profile]:
    """Profiles for one (target, hw), keyed by model."""
    return {
        p.model: p
        for p in discover_profiles()
        if p.target == target and p.hw == hw
    }


# ---------------------------------------------------------------------------
# Workspace
# ---------------------------------------------------------------------------


def build_workspace(force: bool = False) -> Path:
    """Assemble the symlinked mirror that becomes XPU-RT's repository base."""
    if not (XPURT / "scripts" / "run_xpurt_schedule.py").is_file():
        raise XpurtError(f"XPU-RT backend not found under {XPURT}")

    ws = WORKSPACE
    if force and ws.exists():
        shutil.rmtree(ws)
    for sub in ("data/toplevel", "gen/profile", "schedules", "plots"):
        (ws / sub).mkdir(parents=True, exist_ok=True)

    # Point at the backend's code; own everything else.
    for name in ("scripts", "xpu-rt"):
        link = ws / name
        target = XPURT / name
        if link.is_symlink():
            if Path(os.readlink(link)) == target:
                continue
            link.unlink()
        elif link.exists():
            raise XpurtError(f"{link} exists and is not a symlink; remove it")
        link.symlink_to(target)
    return ws


def link_profiles(ws: Path, profiles: Iterable[Profile]) -> int:
    """Expose profiles at the flat layout the loader expects, via symlinks.

    The loader globs ``<base>/gen/profile/<hw>/<target>/<model>/<basename>/*/<topo>``,
    so each profile's parent directory is linked into place. Discovery already
    normalized away any sweep tag.

    This is the pass-through path: per-op times reach the scheduler at whatever
    clock they were captured on. Use :func:`retime_profiles` to report them at a
    different one.
    """
    linked = 0
    for prof in profiles:
        dest = ws / "gen" / "profile" / prof.hw / prof.target / prof.model / prof.basename
        dest.parent.mkdir(parents=True, exist_ok=True)
        src = prof.results.parent.parent  # the input-tag level, holding <topo>/results.csv
        if dest.is_symlink():
            if Path(os.readlink(dest)) == src:
                linked += 1
                continue
            dest.unlink()
        elif dest.is_dir():
            # A real directory here is retimed output from an earlier run at some
            # other clock. Leaving it would let those times masquerade as the
            # capture-clock ones this call is asking for, so it is discarded.
            shutil.rmtree(dest)
        elif dest.exists():
            dest.unlink()
        dest.symlink_to(src)
        linked += 1
    return linked


# ---------------------------------------------------------------------------
# Cycles
# ---------------------------------------------------------------------------


@dataclass
class Retime:
    """What happened when one profile's per-op times were moved to a new clock."""

    profile: Profile
    rows: int = 0
    converted: int = 0
    passthrough: int = 0
    capture_mhz: float | None = None
    capture_consistent: bool = True

    @property
    def complete(self) -> bool:
        return self.rows > 0 and self.passthrough == 0


def _capture_clock_mhz(cycles: float, nanoseconds: float) -> float | None:
    """The clock a row was captured at, derived from its own two columns.

    ``cycles`` and ``mean_time_ns`` are recorded independently by the harness,
    so their ratio *is* the capture frequency. Deriving it beats asserting
    "FireSim models 1 GHz" — the number comes from the data, and a profile
    captured at some other rate cannot be silently mistimed.
    """
    if cycles <= 0 or nanoseconds <= 0:
        return None
    return cycles / nanoseconds * 1000.0


def _row_float(row: dict[str, str], column: str) -> float | None:
    raw = (row.get(column) or "").strip()
    if not raw:
        return None
    try:
        return float(raw)
    except ValueError:
        return None


def retime_results_csv(source: Path, dest: Path, clock_mhz: float) -> Retime | None:
    """Rewrite one results.csv with ``mean_time`` recomputed at `clock_mhz`.

    The scheduler reads wall time (``mean_time``/``mean_unit``) and discards the
    ``cycles`` column the harness also records. Rather than patch XPU-RT — whose
    ``gen/`` tree is tracked and whose loader we do not own — this recomputes
    the wall-time columns *from* cycles into the mirror the scheduler actually
    reads. Cycles are the hardware-independent quantity; the conversion to
    milliseconds happens exactly once, here, at a stated frequency.

    Rows without a usable ``cycles`` value keep their original time and are
    counted as ``passthrough`` so a partial conversion can never pass for a
    complete one.
    """
    if clock_mhz <= 0:
        raise XpurtError(f"clock must be positive, got {clock_mhz}")
    with source.open() as handle:
        reader = csv.DictReader(handle)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)
    if "cycles" not in fieldnames:
        return None

    report = Retime(profile=None, rows=len(rows))  # type: ignore[arg-type]
    captures: list[float] = []
    for row in rows:
        cycles = _row_float(row, "cycles")
        if cycles is None or cycles < 0:
            report.passthrough += 1
            continue
        nanoseconds = _row_float(row, "mean_time_ns")
        if nanoseconds:
            derived = _capture_clock_mhz(cycles, nanoseconds)
            if derived:
                captures.append(derived)
        row["mean_time"] = f"{cycles / (clock_mhz * 1000.0):.6f}"
        row["mean_unit"] = "ms"
        if "mean_time_ns" in fieldnames:
            row["mean_time_ns"] = f"{cycles * 1000.0 / clock_mhz:.6f}"
        report.converted += 1

    if captures:
        report.capture_mhz = round(sum(captures) / len(captures), 6)
        spread = max(captures) - min(captures)
        report.capture_consistent = spread <= max(captures) * 1e-6

    dest.parent.mkdir(parents=True, exist_ok=True)
    with dest.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return report


def derive_capture_clock(profile: Profile) -> float | None:
    """The clock a profile was captured at, read out of the profile itself.

    Averaged over every row that records both ``cycles`` and ``mean_time_ns``.
    Returns None when the profile carries no cycle counts, which is also the
    signal that it cannot be retimed.
    """
    with profile.results.open() as handle:
        rows = list(csv.DictReader(handle))
    derived = []
    for row in rows:
        cycles = _row_float(row, "cycles")
        nanoseconds = _row_float(row, "mean_time_ns")
        if cycles and nanoseconds:
            value = _capture_clock_mhz(cycles, nanoseconds)
            if value:
                derived.append(value)
    if not derived:
        return None
    return round(sum(derived) / len(derived), 6)


def retime_profiles(
    ws: Path, profiles: Iterable[Profile], clock_mhz: float
) -> tuple[list[Retime], list[Profile]]:
    """Materialize every profile into the mirror, retimed to `clock_mhz`.

    Returns the per-profile reports and any profiles that had to be symlinked
    through unchanged because they carry no ``cycles`` column. Those are the
    ones whose numbers remain pinned to their capture clock, and the caller is
    expected to say so rather than fold them in silently.
    """
    reports: list[Retime] = []
    unconverted: list[Profile] = []
    for prof in profiles:
        # Preserve the <basename>/<input_tag>/<topo>/results.csv shape the
        # loader globs for; discovery already stripped any sweep tag above it.
        topo = prof.results.parent.name
        input_tag = prof.results.parent.parent.name
        dest = (
            ws / "gen" / "profile" / prof.hw / prof.target / prof.model / prof.basename
            / input_tag / topo / "results.csv"
        )
        # A previous run may have symlinked this basename straight at the
        # backend; a real directory has to replace the link before we write.
        basename_dir = ws / "gen" / "profile" / prof.hw / prof.target / prof.model / prof.basename
        if basename_dir.is_symlink():
            basename_dir.unlink()
        report = retime_results_csv(prof.results, dest, clock_mhz)
        if report is None:
            unconverted.append(prof)
            continue
        report.profile = prof
        reports.append(report)
    if unconverted:
        link_profiles(ws, unconverted)
    return reports, unconverted


# ---------------------------------------------------------------------------
# Workload descriptors and spec generation
# ---------------------------------------------------------------------------


@dataclass
class Workload:
    name: str
    graph: Path
    period: float | None = None
    window_duration: float | None = None
    preferred_hw: str | None = None
    criticality: str = "best-effort"

    @classmethod
    def from_json(cls, path: Path) -> "Workload":
        data = json.loads(path.read_text())
        graph = Path(data["dispatch_graph"])
        if not graph.is_absolute():
            graph = ROOT / graph
        return cls(
            name=data["name"],
            graph=graph,
            period=data.get("period_ms"),
            window_duration=data.get("deadline_ms") or data.get("period_ms"),
            preferred_hw=data.get("preferred_hw"),
            criticality=data.get("criticality", "best-effort"),
        )


def load_workloads(names: Sequence[str]) -> list[Workload]:
    out = []
    for name in names:
        path = WORKLOAD_DIR / f"{name}.json"
        if not path.is_file():
            available = sorted(p.stem for p in WORKLOAD_DIR.glob("*.json"))
            raise XpurtError(
                f"unknown workload {name!r}; available: {', '.join(available) or 'none'}"
            )
        out.append(Workload.from_json(path))
    return out


def write_spec(
    ws: Path,
    name: str,
    workloads: Sequence[Workload],
    target: str,
    hw_p: str | None = None,
    hw_e: str | None = None,
    n_p: int = 1,
    n_e: int = 1,
    topo_tag: str | None = None,
    time_limit: int = 60,
    seed: int = 42,
    machines: dict[str, int] | None = None,
    profile_hw: dict[str, str] | None = None,
) -> Path:
    """Emit a networks_*.json with absolute dispatch-graph paths.

    Absolute paths matter: the backend resolver accepts them verbatim, whereas a
    relative path is joined onto the repository base and the shipped specs assume
    a layout this tree does not have.

    `machines` and `profile_hw` come from :func:`soc.plan` when the run is bound
    to a Chipyard config, so the core count and kinds are read off the
    elaboration rather than typed at the command line. Passing `hw_p`/`hw_e`/
    `n_p`/`n_e` instead keeps the unbound path, where the caller is asserting a
    topology nothing has checked.

    A single-hart SoC yields ``{"CPU_P": 1}`` with no ``CPU_E`` at all. That is
    the honest shape -- there is nowhere else to put work -- and the scheduler
    handles it, since it expands whatever labels the dict carries.
    """
    if not workloads:
        raise XpurtError("no workloads given")

    if machines is None:
        machines = {"CPU_P": n_p, "CPU_E": n_e}
    if profile_hw is None:
        profile_hw = {"cpu_p": hw_p or "", "cpu_e": hw_e or ""}
    if not machines:
        raise XpurtError("the SoC has no cores to schedule onto")
    # Per-core-count profiles: a two-core machine reads topo_0_1, not topo_0.
    if topo_tag is None:
        topo_tag = topo_tag_for_cores(max(machines.values()))

    networks: dict[str, Any] = {}
    for index, wl in enumerate(workloads):
        if not wl.graph.is_file():
            raise XpurtError(f"dispatch graph missing for {wl.name}: {wl.graph}")
        entry: dict[str, Any] = {
            "id": index,
            "identifier": wl.name,
            "dispatch_deps_path": str(wl.graph),
        }
        if wl.period is not None:
            entry["period"] = wl.period
        if wl.window_duration is not None:
            entry["window_duration"] = wl.window_duration
        if wl.preferred_hw:
            entry["preferred_hw"] = wl.preferred_hw
        networks[wl.name] = entry

    spec = {
        "_comment": (
            f"Generated by rb xpurt for mix '{name}'. Dispatch paths are absolute "
            "so they resolve against this tree rather than the upstream layout."
        ),
        "hardware": {
            "machines": {label.lower(): count for label, count in machines.items()},
            "profile_hw": dict(profile_hw),
            "profile": {
                "target": target,
                "topo_tag": topo_tag,
                "topo_tag_override": True,
                "gen_root": "gen",
            },
            "p_core_speedup": 1.0,
        },
        "scheduler": {
            "random_seed": seed,
            "solver_verbosity": 1,
            "time_limit": time_limit,
            "use_profiled": True,
            "prune_periodic": True,
            "restrict_makespan_to_nonperiodic": True,
        },
        "networks": networks,
        "edges": [],
    }
    path = ws / "data" / "toplevel" / f"networks_{name}.json"
    path.write_text(json.dumps(spec, indent=2) + "\n")
    return path


# ---------------------------------------------------------------------------
# Invocation
# ---------------------------------------------------------------------------


def interpreter() -> Path:
    """Python that satisfies the scheduler (numpy<2 + matplotlib)."""
    from .environments import CHIPYARD_PREFIX

    python = CHIPYARD_PREFIX / "bin" / "python"
    if not python.is_file():
        raise XpurtError(
            f"no interpreter at {python}; the Chipyard Conda environment supplies "
            "the numpy<2 and matplotlib the scheduler needs (rb env setup chipyard)"
        )
    return python


def run_schedule(
    ws: Path,
    spec: Path,
    solver: str = "decomposed",
    profiled: bool = True,
    time_limit: int | None = None,
) -> subprocess.CompletedProcess[str]:
    if solver not in SOLVERS:
        raise XpurtError(f"--solver must be one of {', '.join(SOLVERS)}")
    command = [
        str(interpreter()),
        str(ws / "scripts" / "run_xpurt_schedule.py"),
        "--networks-json",
        str(spec),
        "--solver",
        solver,
    ]
    command.append("--profiled" if profiled else "--no-profiled")
    if time_limit is not None:
        command += ["--time-limit", str(time_limit)]
    print("+", " ".join(command), flush=True)
    return subprocess.run(command, cwd=ws, text=True, capture_output=True, check=False)


def schedule_outputs(ws: Path, spec: Path, solver: str, profiled: bool) -> dict[str, Path]:
    stem = spec.stem  # networks_<name>
    suffix = "_profiled" if profiled else ""
    return {
        "schedule": ws / "schedules" / f"scheduled_{stem}_{solver}{suffix}.json",
        "gantt": ws / "plots" / f"{stem}_{solver}{suffix}.png",
    }


def summarize(schedule_json: Path, clock_mhz: float | None = None) -> dict[str, Any]:
    """Pull the headline latency numbers out of a produced schedule.

    The scheduler emits ``{dot_file, dispatches, metadata}``, where ``dispatches``
    maps a dispatch key to a record carrying ``job_name``, ``hardware_target``,
    ``start_time``, and ``duration``. A job name is the network identifier with a
    periodic-instance index appended (``dronet0``, ``dronet1``), so per-instance
    latency is recovered by grouping on the job name and the network by stripping
    the trailing digits.

    When `clock_mhz` is given it is the frequency the input profiles were
    retimed to, so every millisecond here is also reported as a cycle count.
    Cycles are the quantity that survives a change of clock; milliseconds are
    only meaningful alongside the frequency that produced them.
    """
    data = json.loads(schedule_json.read_text())
    meta = data.get("metadata", {})
    dispatches = data.get("dispatches", {})
    records = dispatches.values() if isinstance(dispatches, dict) else dispatches

    jobs: dict[str, dict[str, Any]] = {}
    for item in records:
        job = str(item.get("job_name", "?"))
        start = float(item.get("start_time", 0.0) or 0.0)
        end = start + float(item.get("duration", 0.0) or 0.0)
        slot = jobs.setdefault(
            job, {"start": start, "end": end, "dispatches": 0, "targets": set()}
        )
        slot["start"] = min(slot["start"], start)
        slot["end"] = max(slot["end"], end)
        slot["dispatches"] += 1
        if item.get("hardware_target"):
            slot["targets"].add(str(item["hardware_target"]))

    def cycles(milliseconds: float) -> int | None:
        if clock_mhz is None:
            return None
        return round(milliseconds * clock_mhz * 1000.0)

    per_job = {
        job: {
            "start": round(v["start"], 3),
            "end": round(v["end"], 3),
            "latency": round(v["end"] - v["start"], 3),
            "latency_cycles": cycles(v["end"] - v["start"]),
            "dispatches": v["dispatches"],
            "targets": sorted(v["targets"]),
        }
        for job, v in sorted(jobs.items())
    }

    per_net: dict[str, dict[str, Any]] = {}
    for job, v in per_job.items():
        net = job.rstrip("0123456789") or job
        slot = per_net.setdefault(
            net, {"instances": 0, "worst_latency": 0.0, "dispatches": 0}
        )
        slot["instances"] += 1
        slot["dispatches"] += v["dispatches"]
        slot["worst_latency"] = max(slot["worst_latency"], v["latency"])
    for slot in per_net.values():
        slot["worst_latency_cycles"] = cycles(slot["worst_latency"])

    makespan = meta.get("makespan")
    return {
        "makespan": makespan,
        "makespan_cycles": cycles(makespan) if isinstance(makespan, (int, float)) else None,
        "num_operations": meta.get("num_operations"),
        "machines": meta.get("machines"),
        "profile_hw": meta.get("profile_hw"),
        "networks": per_net,
        "jobs": per_job,
        "schedule": str(schedule_json),
    }


# ---------------------------------------------------------------------------
# Forest-trail schedule replay
# ---------------------------------------------------------------------------


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _git_revision(path: Path) -> str | None:
    try:
        return subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=path, capture_output=True,
            text=True, check=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _latest(pattern: str) -> Path | None:
    paths = [path for path in XPURT.glob(pattern) if path.is_file()]
    return max(paths, key=lambda path: path.stat().st_mtime) if paths else None


def forest_dronet_weights(explicit: Path | None = None) -> Path | None:
    return explicit or _latest("logs/dronet/*/best.pt")


def forest_checkpoint(explicit: Path | None = None) -> Path | None:
    return explicit or _latest(
        "logs/rsl_rl/crazyflie_steering_tracking/*/model_*.pt"
    )


def forest_python_candidates(explicit: Path | None = None) -> list[Path]:
    env_python = Path(os.environ["RB_XPURT_PYTHON"]) if os.environ.get("RB_XPURT_PYTHON") else None
    active = Path(os.environ["CONDA_PREFIX"]) / "bin" / "python" if os.environ.get("CONDA_PREFIX") else None
    candidates = [explicit, env_python, XPURT_ENV / "bin" / "python", active]
    unique: list[Path] = []
    for path in candidates:
        if path is not None and path not in unique:
            unique.append(path)
    return unique


def forest_environment() -> dict[str, str]:
    env = dict(os.environ)
    python_path = [
        str(XPURT),
        *(str(FOREST_ISAACLAB / "source" / name) for name in (
            "isaaclab", "isaaclab_assets", "isaaclab_rl", "isaaclab_contrib",
        )),
    ]
    if env.get("PYTHONPATH"):
        python_path.append(env["PYTHONPATH"])
    runtime = ROOT / ".rb" / "xpurt-runtime"
    (runtime / "config").mkdir(parents=True, exist_ok=True)
    (runtime / "cache").mkdir(parents=True, exist_ok=True)
    env.update({
        "OMNI_KIT_ACCEPT_EULA": "Y",
        "MPLBACKEND": "Agg",
        "PYTHONPATH": os.pathsep.join(python_path),
        "PYTHONUNBUFFERED": "1",
        "XDG_CONFIG_HOME": str(runtime / "config"),
        "XDG_CACHE_HOME": str(runtime / "cache"),
    })
    return env


def _forest_python_works(path: Path) -> tuple[bool, str]:
    if not path.is_file():
        return False, "not installed"
    code = (
        "import torch,isaacsim,isaaclab,rsl_rl,gymnasium,matplotlib,imageio,ultralytics; "
        "print('torch=' + torch.__version__ + ' cuda=' + str(torch.cuda.is_available()))"
    )
    try:
        result = subprocess.run(
            [str(path), "-c", code], cwd=ROOT, env=forest_environment(),
            capture_output=True, text=True, timeout=30, check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)
    lines = (result.stdout or result.stderr).strip().splitlines()
    return result.returncode == 0, lines[-1] if lines else f"exit {result.returncode}"


def select_forest_python(explicit: Path | None = None) -> Path:
    failures = []
    for candidate in forest_python_candidates(explicit):
        ok, detail = _forest_python_works(candidate)
        if ok:
            return candidate
        failures.append(f"{candidate}: {detail}")
    raise XpurtError(
        "no Python environment can run the Isaac forest pilot; pass --python, "
        "set RB_XPURT_PYTHON, or follow backends/XPU-RT/docs/xpurt_env_setup.md"
        + ("\n  " + "\n  ".join(failures) if failures else "")
    )


def forest_doctor(*, ready: bool = False, python: Path | None = None,
                  dronet: Path | None = None, checkpoint: Path | None = None,
                  schedule: Path = FOREST_SCHEDULE) -> int:
    selected = None
    py_ok = False
    py_detail = "no candidate"
    for candidate in forest_python_candidates(python):
        py_ok, py_detail = _forest_python_works(candidate)
        if py_ok:
            selected = candidate
            break
    dronet = forest_dronet_weights(dronet)
    checkpoint = forest_checkpoint(checkpoint)
    isaac_sources = (FOREST_ISAACLAB / "source" / "isaaclab" / "isaaclab").is_dir()
    cuda_ok = py_ok and "cuda=True" in py_detail
    checks = [
        ("XPU-RT backend", XPURT.is_dir(), str(XPURT), True),
        ("Forest pilot", FOREST_PILOT.is_file(), str(FOREST_PILOT), True),
        ("Pilot adapter", FOREST_LAUNCHER.is_file(), str(FOREST_LAUNCHER), True),
        ("Schedule", schedule.is_file(), str(schedule), True),
        ("IsaacLab submodule", isaac_sources, str(FOREST_ISAACLAB), False),
        ("Isaac Python", py_ok, f"{selected or ''} ({py_detail})", ready),
        ("NVIDIA CUDA", cuda_ok, py_detail, ready),
        ("DroNet weights", bool(dronet and dronet.is_file()), str(dronet or "not found"), ready),
        ("RL checkpoint", bool(checkpoint and checkpoint.is_file()), str(checkpoint or "not found"), ready),
        ("ffmpeg", shutil.which("ffmpeg") is not None, shutil.which("ffmpeg") or "not found", ready),
        ("virtual display", bool(os.environ.get("DISPLAY") or shutil.which("xvfb-run")),
         os.environ.get("DISPLAY") or shutil.which("xvfb-run") or "not found", ready),
    ]
    print("XPU-RT forest executor: Isaac Sim + host CUDA; schedule timing is replayed")
    failed = False
    for label, passed, detail, required in checks:
        state = "ok" if passed else ("MISSING" if required else "optional")
        print(f"{label:20} {state:8} {detail}")
        failed |= required and not passed
    print("FPGA / FireSim / U250 unused   pilot inference runs on the NVIDIA GPU")
    return 1 if failed else 0


def forest_schedule_summary(schedule: Path, time_unit: str = "ms") -> dict[str, Any]:
    if time_unit not in {"ms", "s"}:
        raise XpurtError(f"unsupported schedule time unit: {time_unit}")
    if not schedule.is_file():
        raise XpurtError(f"schedule is missing: {schedule}")
    try:
        data = json.loads(schedule.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise XpurtError(f"cannot read schedule {schedule}: {exc}") from exc
    dispatches = data.get("dispatches")
    if not isinstance(dispatches, dict) or not dispatches:
        raise XpurtError(f"schedule has no dispatches object: {schedule}")
    scale = 1000.0 if time_unit == "s" else 1.0
    jobs: dict[str, dict[str, Any]] = {}
    hardware: dict[str, dict[str, Any]] = {}
    operations: list[dict[str, Any]] = []
    max_end = 0.0
    for key, item in dispatches.items():
        start = float(item.get("start_time", 0.0)) * scale
        duration = float(item.get("duration", 0.0)) * scale
        end = start + duration
        job = str(item.get("job_name") or str(key).split("_dispatch", 1)[0])
        model = re.sub(r"\d+$", "", job) or job
        hw = str(item.get("hardware_target") or "unspecified")
        operations.append({"key": str(key), "model": model, "job": job, "hardware": hw,
                           "start_ms": start, "duration_ms": duration, "end_ms": end})
        slot = jobs.setdefault(job, {"model": model, "start_ms": start, "end_ms": end,
                                     "dispatches": 0, "hardware": set()})
        slot["start_ms"] = min(slot["start_ms"], start)
        slot["end_ms"] = max(slot["end_ms"], end)
        slot["dispatches"] += 1
        slot["hardware"].add(hw)
        hw_slot = hardware.setdefault(hw, {"dispatches": 0, "busy_ms": 0.0, "models": set()})
        hw_slot["dispatches"] += 1
        hw_slot["busy_ms"] += duration
        hw_slot["models"].add(model)
        max_end = max(max_end, end)
    makespan = float((data.get("metadata") or {}).get("makespan", max_end)) * scale
    period = max(makespan, max_end)
    per_model: dict[str, dict[str, Any]] = {}
    for job in jobs.values():
        latency = job["end_ms"] - job["start_ms"]
        model = job["model"]
        slot = per_model.setdefault(model, {"jobs": 0, "dispatches": 0, "latencies_ms": [],
                                             "completion_times_ms": []})
        slot["jobs"] += 1
        slot["dispatches"] += job["dispatches"]
        slot["latencies_ms"].append(latency)
        slot["completion_times_ms"].append(job["end_ms"])
    for slot in per_model.values():
        completions = sorted(slot.pop("completion_times_ms"))
        gaps = [b - a for a, b in zip(completions, completions[1:])]
        if completions:
            gaps.append(period - completions[-1] + completions[0])
        latencies = slot.pop("latencies_ms")
        slot["worst_job_latency_ms"] = max(latencies)
        slot["mean_job_latency_ms"] = sum(latencies) / len(latencies)
        slot["max_output_hold_ms"] = max(gaps, default=period)
    for slot in hardware.values():
        slot["models"] = sorted(slot["models"])
        slot["utilization"] = slot["busy_ms"] / period if period else 0.0
    return {
        "period_ms": period,
        "operations": len(operations),
        "models": dict(sorted(per_model.items())),
        "hardware": dict(sorted(hardware.items())),
        "dispatches": sorted(operations, key=lambda item: (item["start_ms"], item["hardware"])),
        "schedule": str(schedule),
        "schedule_sha256": _sha256(schedule),
        "timing_semantics": "input sampled at job start; output applied at job completion; cached between completions",
        "timing_evidence": "replayed XPU-RT schedule metadata; inference itself runs on host CUDA",
    }


def _svg_escape(value: Any) -> str:
    return html.escape(str(value), quote=True)


def render_forest_plot(schedule_summary: dict[str, Any], output: Path,
                       telemetry: Sequence[dict[str, float]] = (),
                       verdict: str = "schedule-ready") -> Path:
    width, height = 960, 650 if telemetry else 430
    left, right = 120.0, 910.0
    plot_width = right - left
    hardware = list(schedule_summary["hardware"])
    row_h = 34
    timeline_y = 105
    period = float(schedule_summary["period_ms"]) or 1.0
    colors = {"dronet": "#e79027", "mlp_control": "#4d8ecb", "mlp": "#4d8ecb",
              "yolov8_nano": "#d9534f", "yolo": "#d9534f"}
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        f'<rect width="{width}" height="{height}" fill="#fbfbf8"/>',
        f'<text x="42" y="38" font-family="sans-serif" font-size="20" font-weight="600">XPU-RT forest-trail replay · {_svg_escape(verdict)}</text>',
        f'<text x="42" y="61" font-family="sans-serif" font-size="12" fill="#555">{period:.3f} ms period · {schedule_summary["operations"]} scheduled operations · host-CUDA inference · no FPGA/FireSim at runtime</text>',
    ]
    for row, hw in enumerate(hardware):
        y = timeline_y + row * row_h
        parts += [
            f'<text x="112" y="{y + 16}" text-anchor="end" font-family="sans-serif" font-size="11">{_svg_escape(hw)}</text>',
            f'<rect x="{left}" y="{y}" width="{plot_width}" height="22" fill="#fff" stroke="#ddd"/>',
        ]
    for item in schedule_summary["dispatches"]:
        row = hardware.index(item["hardware"])
        x = left + plot_width * item["start_ms"] / period
        bar_width = max(0.7, plot_width * item["duration_ms"] / period)
        y = timeline_y + row * row_h
        color = colors.get(item["model"], "#8b8b83")
        parts.append(
            f'<rect x="{x:.2f}" y="{y + 2}" width="{bar_width:.2f}" height="18" fill="{color}" opacity="0.9"><title>{_svg_escape(item["job"])} · {_svg_escape(item["model"])} · {item["duration_ms"]:.3f} ms</title></rect>'
        )
    timeline_bottom = timeline_y + len(hardware) * row_h
    parts += [
        f'<line x1="{left}" y1="{timeline_bottom}" x2="{right}" y2="{timeline_bottom}" stroke="#555"/>',
        f'<text x="{(left + right) / 2}" y="{timeline_bottom + 30}" text-anchor="middle" font-family="sans-serif" font-size="12">time within repeating schedule period (ms)</text>',
    ]
    legend_y = timeline_bottom + 52
    for index, model in enumerate(schedule_summary["models"]):
        x = left + index * 165
        color = colors.get(model, "#8b8b83")
        parts += [f'<rect x="{x}" y="{legend_y}" width="13" height="13" fill="{color}"/>',
                  f'<text x="{x + 19}" y="{legend_y + 11}" font-family="sans-serif" font-size="11">{_svg_escape(model)}</text>']
    if telemetry:
        path_y, path_h = legend_y + 45, 185
        xs = [float(item["x_m"]) for item in telemetry]
        ys = [float(item["y_m"]) for item in telemetry]
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)
        xspan, yspan = xmax - xmin or 1.0, ymax - ymin or 1.0
        points = " ".join(
            f'{left + plot_width * (x - xmin) / xspan:.1f},{path_y + path_h * (ymax - y) / yspan:.1f}'
            for x, y in zip(xs, ys)
        )
        parts += [
            f'<text x="{left}" y="{path_y - 10}" font-family="sans-serif" font-size="12">Pilot position samples from logs/pilot.log</text>',
            f'<rect x="{left}" y="{path_y}" width="{plot_width}" height="{path_h}" fill="#fff" stroke="#ddd"/>',
            f'<polyline points="{points}" fill="none" stroke="#176b87" stroke-width="2.2"/>',
            f'<circle cx="{points.split()[0].split(",")[0]}" cy="{points.split()[0].split(",")[1]}" r="4" fill="#3a9d5d"/>',
            f'<circle cx="{points.split()[-1].split(",")[0]}" cy="{points.split()[-1].split(",")[1]}" r="4" fill="#d9534f"/>',
        ]
    parts += [
        f'<text x="42" y="{height - 24}" font-family="sans-serif" font-size="10" fill="#666">Bars are predicted/profile-derived dispatch windows replayed by the simulator; they are not measurements of the CUDA inference calls.</text>',
        '</svg>',
    ]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(parts) + "\n", encoding="utf-8")
    return output


_TELEMETRY = re.compile(
    r"step\s+(?P<step>\d+).*?pos=\((?P<x>[+-]?\d+\.\d+),(?P<y>[+-]?\d+\.\d+)\).*?"
    r"cmd ω=(?P<cmd>[+-]?\d+\.\d+).*?act ω=(?P<yaw>[+-]?\d+\.\d+).*?v=(?P<velocity>[+-]?\d+\.\d+)"
)


def summarize_forest_run(run_dir: Path, *, returncode: int | None = None,
                         timed_out: bool = False) -> dict[str, Any]:
    config = json.loads((run_dir / "rendered-config.json").read_text(encoding="utf-8"))
    schedule_summary = forest_schedule_summary(run_dir / "schedule.json", config["schedule_time_unit"])
    log_path = run_dir / "logs" / "pilot.log"
    text = log_path.read_text(encoding="utf-8", errors="replace") if log_path.is_file() else ""
    counts = {}
    for label in ("DroNet", "MLP", "YOLO"):
        match = re.search(rf"\[done\] {label} executions:\s+(\d+)", text)
        counts[label.lower()] = int(match.group(1)) if match else None
    steps_match = re.search(r"\[done\] total control steps:\s+(\d+)", text)
    telemetry = [
        {"step": int(match["step"]), "x_m": float(match["x"]), "y_m": float(match["y"]),
         "command_yaw_rad_s": float(match["cmd"]), "actual_yaw_rad_s": float(match["yaw"]),
         "velocity_m_s": float(match["velocity"])}
        for match in _TELEMETRY.finditer(text)
    ]
    videos = []
    for path in sorted(run_dir.glob("video_p*.*")):
        valid = path.stat().st_size > 0
        duration = None
        ffprobe = shutil.which("ffprobe")
        if valid and ffprobe:
            probe = subprocess.run(
                [ffprobe, "-v", "error", "-show_entries", "format=duration",
                 "-of", "default=noprint_wrappers=1:nokey=1", str(path)],
                capture_output=True, text=True, check=False,
            )
            valid = probe.returncode == 0
            if valid:
                try:
                    duration = float(probe.stdout.strip())
                except ValueError:
                    valid = False
        videos.append({"path": path.name, "bytes": path.stat().st_size,
                       "sha256": _sha256(path), "valid": valid,
                       "duration_s": duration})
    resets = len(re.findall(r"\[reset\]", text))
    video_ok = not config.get("video", False) or any(item["valid"] for item in videos)
    execution_ok = returncode == 0 and bool(steps_match) and video_ok
    verdict = "complete" if execution_ok else ("timeout" if timed_out else "error")
    return {
        "schema": "riskybird.xpurt-forest-summary.v1",
        "verdict": verdict,
        "execution_ok": execution_ok,
        "pilot_exit": returncode,
        "timed_out": timed_out,
        "control_steps": int(steps_match.group(1)) if steps_match else None,
        "model_executions": counts,
        "resets": resets,
        "telemetry_samples": telemetry,
        "final_position_m": ([telemetry[-1]["x_m"], telemetry[-1]["y_m"]] if telemetry else None),
        "videos": videos,
        "schedule": {key: value for key, value in schedule_summary.items() if key != "dispatches"},
        "runtime_claim": "models executed on host CUDA; XPU-RT dispatch timing was replayed",
        "uses_fpga": False,
        "uses_firesim": False,
        "uses_u250": False,
    }


def refresh_forest_summary(run_dir: Path, output: Path | None = None) -> dict[str, Any]:
    previous_path = run_dir / "summary.json"
    previous = json.loads(previous_path.read_text(encoding="utf-8")) if previous_path.is_file() else {}
    summary = summarize_forest_run(
        run_dir, returncode=previous.get("pilot_exit", 0),
        timed_out=bool(previous.get("timed_out", False)),
    )
    _json(previous_path, summary)
    schedule_summary = forest_schedule_summary(
        run_dir / "schedule.json",
        json.loads((run_dir / "rendered-config.json").read_text())["schedule_time_unit"],
    )
    render_forest_plot(schedule_summary, output or run_dir / "plot.svg",
                       summary["telemetry_samples"], summary["verdict"])
    return summary


def run_forest_trail(*, dronet: Path | None, checkpoint: Path | None,
                     schedule: Path = FOREST_SCHEDULE, python: Path | None = None,
                     periods: int = 200, trail: str = "curved", seed: int = 12,
                     yolo: bool = True, humans: bool = True, headless: bool = False,
                     video: bool = True, timeout_s: float = 3600.0,
                     output: Path | None = None) -> tuple[Path, dict[str, Any]]:
    if periods < 1:
        raise XpurtError("periods must be at least one")
    dronet = forest_dronet_weights(dronet)
    checkpoint = forest_checkpoint(checkpoint)
    missing = [label for label, path in (("DroNet weights", dronet), ("RL checkpoint", checkpoint),
                                          ("schedule", schedule), ("forest pilot", FOREST_PILOT),
                                          ("pilot adapter", FOREST_LAUNCHER))
               if path is None or not path.is_file()]
    if missing:
        raise XpurtError("missing forest-pilot input(s): " + ", ".join(missing))
    host_python = select_forest_python(python)
    if output is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output = FOREST_RUNS / f"{stamp}_xpurt_forest_trail"
    elif not output.is_absolute():
        output = ROOT / output
    if output.exists() and any(output.iterdir()):
        raise XpurtError(f"output directory is not empty: {output}")
    (output / "logs").mkdir(parents=True, exist_ok=True)
    shutil.copy2(schedule, output / "schedule.json")
    config = {
        "schema": "riskybird.xpurt-forest-config.v1", "scenario": "forest-trail",
        "policy": "xpurt", "inference_executor": "host-cuda", "periods": periods,
        "trail": trail, "curvature_seed": seed, "yolo": yolo, "humans": humans,
        "headless": headless, "video": video, "schedule_time_unit": "ms",
    }
    _json(output / "rendered-config.json", config)
    command = [
        str(host_python), str(FOREST_LAUNCHER), str(FOREST_PILOT),
        "--dronet_weights", str(dronet),
        "--checkpoint", str(checkpoint), "--schedule_json", str(output / "schedule.json"),
        "--schedule_time_unit", "ms", "--num_periods", str(periods),
        "--trail", trail, "--curvature_seed", str(seed),
    ]
    if yolo:
        command.append("--yolo")
    if not humans:
        command.append("--no_humans")
    if headless and not video:
        command += ["--headless", "--no_fpv_plot"]
    if video:
        # The compatibility launcher lets Isaac consume --headless, then
        # re-enables only the pilot's matplotlib panel for offscreen capture.
        command += ["--headless", "--save_video", str(output / "video.mp4")]
    launch_command = command
    if video and not os.environ.get("DISPLAY"):
        xvfb = shutil.which("xvfb-run")
        if xvfb is None:
            raise XpurtError("video capture needs DISPLAY or xvfb-run; use --headless --no-video")
        launch_command = [xvfb, "-a", *command]
    _json(output / "meta.json", {
        "schema": "riskybird.run-meta.v1", "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": launch_command, "pilot_command": command,
        "executor": "isaac-host-cuda", "uses_fpga": False,
        "uses_firesim": False, "uses_u250": False, "repository_commit": _git_revision(ROOT),
        "xpurt_commit": _git_revision(XPURT), "host_python": str(host_python),
        "inputs": {
            "dronet": {"path": str(dronet), "sha256": _sha256(dronet)},
            "checkpoint": {"path": str(checkpoint), "sha256": _sha256(checkpoint)},
            "schedule": {"path": str(schedule), "sha256": _sha256(schedule)},
        },
    })
    schedule_summary = forest_schedule_summary(output / "schedule.json")
    render_forest_plot(schedule_summary, output / "plot.svg")
    log_path = output / "logs" / "pilot.log"
    timed_out = False
    returncode: int | None = None
    started = time.monotonic()
    print("+", " ".join(str(item) for item in launch_command), flush=True)
    stdout = stderr = ""
    try:
        process = subprocess.Popen(
            launch_command, cwd=XPURT, env=forest_environment(),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            start_new_session=True,
        )
        try:
            stdout, stderr = process.communicate(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            timed_out = True
            os.killpg(process.pid, signal.SIGTERM)
            try:
                stdout, stderr = process.communicate(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                stdout, stderr = process.communicate()
        returncode = process.returncode
    except OSError as exc:
        stderr = f"failed to launch forest pilot: {exc}\n"
    log_path.write_text((stdout or "") + (stderr or ""), encoding="utf-8")
    if stdout:
        print(stdout, end="")
    if stderr:
        print(stderr, end="")
    summary = summarize_forest_run(output, returncode=returncode, timed_out=timed_out)
    summary["wall_time_s"] = time.monotonic() - started
    _json(output / "summary.json", summary)
    render_forest_plot(schedule_summary, output / "plot.svg",
                       summary["telemetry_samples"], summary["verdict"])
    print(f"run      {output}")
    print(f"verdict  {summary['verdict']}")
    print(f"steps    {summary['control_steps']}")
    print(f"models   {summary['model_executions']}")
    print(f"plot     {output / 'plot.svg'}")
    if not summary["execution_ok"]:
        reason = "timed out" if timed_out else f"did not produce a complete evidence bundle (exit {returncode})"
        raise XpurtError(f"forest replay {reason}; partial artifacts are in {output}")
    return output, summary
