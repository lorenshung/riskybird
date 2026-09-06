"""The join: one record per SoC config carrying both halves.

`rb area` answers "does it fit" and writes a record under ``results/area/``.
`rb xpurt` answers "is it fast enough" and writes a summary under
``dse/plots/``. Both work, and nothing connects them -- to ask which
configuration is best you would open two directories and match them up by hand.

A DSE point is that join, made once and stored. It carries the resource
measurement, the latency result, and the fingerprint of the machine both
describe, so ranking configurations is a sort over a directory rather than a
manual correlation.

It does not replace either half. The area record stays where it was, with its
own report and figures; a point references it by ``run_id`` and copies only the
few numbers a ranking needs. Deleting every point loses nothing that was not
already recorded elsewhere.

**The join key is the fingerprint, not the config name.** Two configs differing
only in ``use_dsp`` elaborate to the same machine and share an ``identity_id``,
so a latency measured for one is valid for the other -- which is what keeps the
expensive half of the flow at a handful of machines rather than one per config.
When the latency came from a profile captured on some *other* machine, the point
records that as ``extrapolated`` and keeps the divergences, so a ranking can
exclude it or flag it rather than silently mixing measured and borrowed numbers.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Sequence

from .boards import ROOT


SCHEMA = "riskybird.dse-point.v1"
RESULTS_DIR = ROOT / "dse" / "results" / "points"
PLOTS_DIR = ROOT / "dse" / "plots"

# Ranked worst-first: a point that cannot be built is worse than one that merely
# misses a deadline, and a borrowed number is worse than one nobody measured
# because it looks authoritative.
VERDICTS = ("infeasible", "misses-deadline", "extrapolated", "ok")


class DseError(RuntimeError):
    """A design point could not be assembled."""


def make_point_id(board: str, config: str, attrs: str, mix: str) -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    safe = re.sub(r"[^A-Za-z0-9_.-]", "-", f"{board}-{config}-attrs_{attrs}-{mix}")
    return f"{stamp}-{safe}"


@dataclass
class DsePoint:
    """One configuration, measured on both axes."""

    point_id: str
    recorded_at: str
    board: str
    config: str
    attrs: str
    mix: str
    identity_id: str | None
    fingerprint: dict[str, Any] | None
    resource: dict[str, Any]
    performance: dict[str, Any]
    provenance: dict[str, Any] = field(default_factory=dict)

    # -- derived ------------------------------------------------------------

    @property
    def fits(self) -> bool | None:
        """Whether every measured resource is within device capacity."""
        used = self.resource.get("utilization") or {}
        if not used:
            return None
        percentages = [v.get("util_pct") for v in used.values() if v.get("util_pct") is not None]
        return max(percentages) <= 100.0 if percentages else None

    @property
    def worst_utilization(self) -> float | None:
        used = self.resource.get("utilization") or {}
        percentages = [v.get("util_pct") for v in used.values() if v.get("util_pct") is not None]
        return max(percentages) if percentages else None

    @property
    def missed(self) -> list[str]:
        return list(self.performance.get("missed_deadlines") or [])

    @property
    def agreement(self) -> str | None:
        return self.performance.get("agreement")

    @property
    def verdict(self) -> str:
        """The single worst thing true about this point."""
        if self.fits is False:
            return "infeasible"
        if self.missed:
            return "misses-deadline"
        if self.agreement == "extrapolated":
            return "extrapolated"
        return "ok"

    def resource_value(self, key: str) -> float | None:
        entry = (self.resource.get("utilization") or {}).get(key) or {}
        used = entry.get("used")
        return float(used) if used is not None else None

    def to_json(self) -> dict[str, Any]:
        return {
            "schema": SCHEMA,
            "point_id": self.point_id,
            "recorded_at": self.recorded_at,
            "board": self.board,
            "config": self.config,
            "attrs": self.attrs,
            "mix": self.mix,
            "identity_id": self.identity_id,
            "verdict": self.verdict,
            "fingerprint": self.fingerprint,
            "resource": self.resource,
            "performance": self.performance,
            "provenance": self.provenance,
        }

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> "DsePoint":
        return cls(
            point_id=data["point_id"],
            recorded_at=data["recorded_at"],
            board=data["board"],
            config=data["config"],
            attrs=data.get("attrs", "none"),
            mix=data.get("mix", ""),
            identity_id=data.get("identity_id"),
            fingerprint=data.get("fingerprint"),
            resource=data.get("resource", {}),
            performance=data.get("performance", {}),
            provenance=data.get("provenance", {}),
        )


def build_point(
    *,
    board: str,
    config: str,
    attrs: str,
    mix: str,
    area_record: Any,
    summary: dict[str, Any],
    deadlines: dict[str, float | None],
) -> DsePoint:
    """Assemble a point from an area record and an xpurt summary.

    Only the fields a ranking needs are copied; the area record keeps
    everything else and is referenced by ``run_id`` so nothing is duplicated
    into a second source of truth.
    """
    utilization = {
        key: {
            "used": entry.get("used"),
            "available": entry.get("available"),
            "util_pct": (
                round(entry["used"] / entry["available"] * 100.0, 2)
                if entry.get("used") is not None and entry.get("available")
                else None
            ),
        }
        for key, entry in (area_record.totals or {}).items()
    }

    networks = summary.get("networks") or {}
    missed = [
        name
        for name, value in networks.items()
        if deadlines.get(name) and value.get("worst_latency", 0.0) > deadlines[name]
    ]
    timing = summary.get("timing") or {}
    fingerprint = summary.get("fingerprint") or {}

    return DsePoint(
        point_id=make_point_id(board, config, attrs, mix),
        recorded_at=datetime.now(timezone.utc).isoformat(timespec="seconds"),
        board=board,
        config=config,
        attrs=attrs,
        mix=mix,
        identity_id=(area_record.fingerprint or {}).get("identity_id"),
        fingerprint=area_record.fingerprint,
        resource={
            "evidence": area_record.evidence,
            "flow": area_record.flow,
            "part": (area_record.device or {}).get("part"),
            "totals_are_upper_bound": area_record.totals_are_upper_bound,
            "utilization": utilization,
            "area_run_id": area_record.run_id,
        },
        performance={
            "evidence": "scheduled",
            "agreement": fingerprint.get("agreement"),
            "clock_mhz": timing.get("clock_mhz"),
            "clock_source": timing.get("clock_source"),
            "capture_clock_mhz": timing.get("capture_clock_mhz"),
            "makespan_ms": summary.get("makespan"),
            "makespan_cycles": summary.get("makespan_cycles"),
            "networks": {
                name: {
                    "worst_latency_ms": value.get("worst_latency"),
                    "worst_latency_cycles": value.get("worst_latency_cycles"),
                    "deadline_ms": deadlines.get(name),
                    "instances": value.get("instances"),
                }
                for name, value in networks.items()
            },
            "missed_deadlines": missed,
            "divergences": fingerprint.get("divergences") or [],
            "profile_hw": summary.get("profile_hw"),
            "machines": summary.get("machines"),
        },
        provenance={
            "area_record": f"results/area/{area_record.run_id}/record.json",
            "schedule": summary.get("schedule"),
        },
    )


def write_point(point: DsePoint, results_dir: Path = RESULTS_DIR) -> Path:
    out = results_dir / point.point_id
    out.mkdir(parents=True, exist_ok=True)
    path = out / "point.json"
    path.write_text(json.dumps(point.to_json(), indent=2) + "\n")
    return path


def load_points(results_dir: Path = RESULTS_DIR) -> list[DsePoint]:
    if not results_dir.is_dir():
        return []
    points = []
    for candidate in sorted(results_dir.glob("*/point.json")):
        data = json.loads(candidate.read_text())
        if data.get("schema") != SCHEMA:
            raise DseError(f"{candidate}: expected schema {SCHEMA}, found {data.get('schema')!r}")
        points.append(DsePoint.from_json(data))
    return points


def latest_per_config(points: Iterable[DsePoint]) -> list[DsePoint]:
    """Keep the newest point for each (board, config, attrs, mix)."""
    newest: dict[tuple[str, str, str, str], DsePoint] = {}
    for point in sorted(points, key=lambda p: p.recorded_at):
        newest[(point.board, point.config, point.attrs, point.mix)] = point
    return list(newest.values())


def pareto(
    points: Sequence[DsePoint],
    resource: str = "luts",
    objective: str = "makespan_ms",
) -> list[DsePoint]:
    """Points not beaten on both axes at once.

    Smaller is better on each: fewer LUTs and less latency. A point is
    dominated when another uses no more of the resource *and* is no slower,
    while being strictly better on at least one.

    Infeasible points are excluded outright -- a configuration that does not fit
    is not a trade-off, it is not a configuration. Extrapolated points are kept,
    because excluding them silently would hide most of the space; they carry
    their label so a reader can see which frontier members are borrowed.
    """
    usable = [
        p for p in points
        if p.fits is not False
        and p.resource_value(resource) is not None
        and p.performance.get(objective) is not None
    ]
    frontier: list[DsePoint] = []
    for point in usable:
        area = point.resource_value(resource)
        latency = float(point.performance[objective])
        dominated = any(
            other is not point
            and other.resource_value(resource) <= area
            and float(other.performance[objective]) <= latency
            and (
                other.resource_value(resource) < area
                or float(other.performance[objective]) < latency
            )
            for other in usable
        )
        if not dominated:
            frontier.append(point)
    return sorted(frontier, key=lambda p: p.resource_value(resource) or 0.0)
