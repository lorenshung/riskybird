"""FPGA resource-utilization characterization and reporting.

This module owns the top-level half of RiskyBird's area-characterization flow.
The Vivado-side half lives in ``backends/chipyard/riskybird/`` and is invoked
through Chipyard's FPGA Makefile; everything here turns its text reports into
records with provenance, and turns records into tables and figures.

Design rules this module enforces:

* **Nothing is hardcoded that Vivado already reports.** Device capacities come
  from the ``Available`` column of Vivado's own utilization report, not from a
  transcribed datasheet table, so a part substitution cannot silently produce
  wrong percentages.
* **Every record carries provenance**: part, board part, tool version, config
  class, RiskyBird and Chipyard commits with dirty flags, the pinned revision of
  every Chipyard submodule that affects generated RTL, the exact command, and
  the RTL-mutation state of the run.
* **Incomparable runs cannot share an axis.** A record's ``flow`` and
  ``rtl_mutation.mode`` are part of its identity. ``group_comparable`` refuses to
  mix them, because ``keep_hierarchy`` inhibits cross-boundary optimization and
  makes device totals an upper bound.
"""

from __future__ import annotations

import csv
import json
import re
import subprocess
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Sequence

from . import fingerprint
from .boards import BOARDS, CHIPYARD, ROOT, Board


RESULTS_DIR = ROOT / "results" / "area"
SCHEMA = "riskybird.area-record.v1"

# Attribute-injection modes accepted by backends/chipyard/riskybird/area.mk.
ATTR_MODES = ("none", "hierarchy", "dsp", "all")

# Chipyard submodules whose revision changes generated RTL, and therefore area.
# Recorded per run so a utilization number can be traced to the exact generator
# revisions that produced it.
RTL_SUBMODULES = (
    "generators/rocket-chip",
    "generators/saturn",
    "generators/gemmini",
    "generators/shuttle",
    "generators/testchipip",
    "fpga/fpga-shells",
)

# Resource rows worth promoting into the flat summary, in report order. Vivado
# names differ between UltraScale ("CLB LUTs") and 7-series ("Slice LUTs"), so
# both spellings are accepted and normalized to a common key.
RESOURCE_ALIASES = {
    "luts": ("CLB LUTs", "Slice LUTs"),
    "lut_logic": ("LUT as Logic",),
    "lut_memory": ("LUT as Memory",),
    "registers": ("CLB Registers", "Slice Registers"),
    "brams": ("Block RAM Tile",),
    "ramb36": ("RAMB36/FIFO", "RAMB36E2 only", "RAMB36E1 only"),
    "ramb18": ("RAMB18", "RAMB18E2 only", "RAMB18E1 only"),
    "dsps": ("DSPs",),
    "urams": ("URAM",),
}


class AreaError(RuntimeError):
    """A recoverable failure in the area-characterization flow."""


# ---------------------------------------------------------------------------
# Vivado report parsing
# ---------------------------------------------------------------------------


def _row_blocks(text: str) -> list[list[str]]:
    """Group a Vivado report's table rows into runs separated by ``+--+`` borders.

    A Vivado table is border / header row / border / data rows / border, so the
    header and the data land in consecutive blocks. Report preamble lines such as
    ``| Tool Version : ...`` open with ``|`` but do not close with one, which is
    what distinguishes them from table rows.
    """
    blocks: list[list[str]] = []
    current: list[str] = []
    for line in text.splitlines():
        stripped = line.rstrip()
        if stripped.startswith("|") and stripped.endswith("|"):
            current.append(stripped)
            continue
        if current:
            blocks.append(current)
            current = []
    if current:
        blocks.append(current)
    return blocks


def _tables(text: str) -> list[tuple[list[str], list[str]]]:
    """Return ``(header_cells, data_row_lines)`` for every table in the report."""
    blocks = _row_blocks(text)
    tables: list[tuple[list[str], list[str]]] = []
    index = 0
    while index < len(blocks):
        block = blocks[index]
        if len(block) == 1 and index + 1 < len(blocks):
            tables.append(([c.strip() for c in _raw_cells(block[0])], blocks[index + 1]))
            index += 2
            continue
        index += 1
    return tables


def _raw_cells(line: str) -> list[str]:
    """Split a ``|``-delimited row, preserving each cell's whitespace."""
    cells = line.split("|")
    if cells and cells[0].strip() == "":
        cells = cells[1:]
    if cells and cells[-1].strip() == "":
        cells = cells[:-1]
    return cells


def _row_cells(line: str) -> list[str]:
    return [cell.strip() for cell in _raw_cells(line)]


def _to_number(value: str) -> float | int | None:
    text = value.replace(",", "").strip()
    if text in {"", "-", "<0.01"}:
        return 0 if text == "<0.01" else None
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        return None


def parse_flat_utilization(text: str) -> dict[str, dict[str, Any]]:
    """Parse device totals out of ``report_utilization`` output.

    Returns ``{site_type: {"used": int, "available": int, "util_pct": float}}``
    for every site-type row found, keyed by Vivado's own row label. Availability
    and percentage come from Vivado, never from a local device table.
    """
    totals: dict[str, dict[str, Any]] = {}
    for header_cells, rows in _tables(text):
        header = [cell.lower() for cell in header_cells]
        if not header or "site type" not in header[0]:
            continue
        try:
            i_used = header.index("used")
        except ValueError:
            continue
        i_avail = header.index("available") if "available" in header else None
        i_util = next(
            (i for i, cell in enumerate(header) if cell.startswith("util")), None
        )
        for line in rows:
            cells = _row_cells(line)
            if len(cells) <= i_used:
                continue
            name = cells[0].strip()
            if not name:
                continue
            used = _to_number(cells[i_used])
            if used is None:
                continue
            entry: dict[str, Any] = {"used": used}
            if i_avail is not None and len(cells) > i_avail:
                entry["available"] = _to_number(cells[i_avail])
            if i_util is not None and len(cells) > i_util:
                entry["util_pct"] = _to_number(cells[i_util])
            # Site-type labels are unique within a report, including the nested
            # breakdown rows ("CLB LUTs" vs "LUT as Logic"), so the stripped
            # label is a safe key. First occurrence wins.
            totals.setdefault(name, entry)
    return totals


def normalize_totals(totals: dict[str, dict[str, Any]]) -> dict[str, dict[str, Any]]:
    """Map Vivado's device-specific row labels onto stable RiskyBird keys.

    Vivado appends a footnote marker to some rows depending on design state --
    ``CLB LUTs*`` and ``RAMB36/FIFO*`` both appear that way in a post-synthesis
    report -- so matching ignores trailing asterisks. ``totals_raw`` keeps the
    label exactly as Vivado wrote it; ``vivado_row`` here records which raw label
    a normalized entry came from.
    """
    lookup: dict[str, tuple[str, dict[str, Any]]] = {}
    for label, entry in totals.items():
        lookup.setdefault(label.rstrip("*").strip(), (label, entry))

    out: dict[str, dict[str, Any]] = {}
    for key, aliases in RESOURCE_ALIASES.items():
        for alias in aliases:
            found = lookup.get(alias.rstrip("*").strip())
            if found is not None:
                raw_label, entry = found
                out[key] = dict(entry, vivado_row=raw_label)
                break
    return out


# Columns of the hierarchical report's top row, used when a flat device report
# is unavailable. These give usage but not device capacity, so no percentages.
HIERARCHY_TOTAL_COLUMNS = {
    "luts": "Total LUTs",
    "lut_logic": "Logic LUTs",
    "lut_memory": "LUTRAMs",
    "registers": "FFs",
    "ramb36": "RAMB36",
    "ramb18": "RAMB18",
    "dsps": "DSP Blocks",
    "urams": "URAM",
}


def totals_from_hierarchy(rows: Sequence[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    """Derive device usage from the top row of a hierarchical utilization report.

    A fallback for when ``report_utilization`` (flat) was not produced. Yields
    ``used`` only: the hierarchical report does not state device capacity, so
    callers must not compute utilization percentages from this.
    """
    top = next((row for row in rows if row["depth"] == min(r["depth"] for r in rows)), None)
    if top is None:
        return {}
    out: dict[str, dict[str, Any]] = {}
    for key, column in HIERARCHY_TOTAL_COLUMNS.items():
        if column in top["metrics"]:
            out[key] = {"used": top["metrics"][column], "vivado_row": column}
    return out


def parse_hierarchical_utilization(text: str) -> list[dict[str, Any]]:
    """Parse ``report_utilization -hierarchical`` into per-instance rows.

    Column names are read from the table header rather than assumed, so the
    parser tolerates the column-set differences between Vivado versions and
    between UltraScale and 7-series parts.
    """
    rows: list[dict[str, Any]] = []
    for header, block in _tables(text):
        lowered = [cell.lower() for cell in header]
        if not lowered or lowered[0] != "instance":
            continue
        table_rows: list[dict[str, Any]] = []
        for line in block:
            raw = _raw_cells(line)
            if len(raw) != len(header):
                continue
            cells = [cell.strip() for cell in raw]
            # Hierarchy depth is encoded as leading spaces in the Instance cell,
            # so it must be read before stripping.
            indent = len(raw[0]) - len(raw[0].lstrip())
            name = cells[0]
            metrics = {
                header[i]: _to_number(cells[i])
                for i in range(2, len(header))
                if _to_number(cells[i]) is not None
            }
            table_rows.append(
                {
                    "instance": name,
                    "module": cells[1],
                    "indent": indent,
                    # Vivado emits a parenthesized row per module holding the
                    # logic that lives directly in it rather than in a child.
                    "self_only": name.startswith("(") and name.endswith(")"),
                    "metrics": metrics,
                }
            )

        # Vivado pads cells and indents two spaces per level, so raw indent runs
        # 1, 3, 5, ... Rank the distinct indents instead of assuming the padding,
        # giving 0-based hierarchy levels that survive a formatting change.
        levels = {indent: i for i, indent in enumerate(sorted({r["indent"] for r in table_rows}))}
        for row in table_rows:
            row["depth"] = levels[row["indent"]]
        rows.extend(table_rows)
    return rows


# ---------------------------------------------------------------------------
# Provenance
# ---------------------------------------------------------------------------


def _git(args: Sequence[str], cwd: Path) -> str | None:
    try:
        result = subprocess.run(
            ["git", *args],
            cwd=cwd,
            check=True,
            capture_output=True,
            text=True,
        )
    except (subprocess.CalledProcessError, FileNotFoundError, NotADirectoryError):
        return None
    return result.stdout.strip() or None


def _repo_state(path: Path) -> dict[str, Any]:
    commit = _git(["rev-parse", "HEAD"], path)
    status = _git(["status", "--porcelain"], path)
    return {
        "commit": commit,
        "dirty": bool(status),
        "describe": _git(["describe", "--always", "--dirty"], path),
    }


def _submodule_revisions() -> dict[str, str | None]:
    revisions: dict[str, str | None] = {}
    for sub in RTL_SUBMODULES:
        path = CHIPYARD / sub
        if not path.exists():
            revisions[sub] = None
            continue
        revisions[sub] = _git(["rev-parse", "HEAD"], path)
    return revisions


def collect_provenance(command: Sequence[str]) -> dict[str, Any]:
    """Everything needed to reproduce or invalidate a measurement."""
    return {
        "riskybird": _repo_state(ROOT),
        "chipyard": _repo_state(CHIPYARD),
        "chipyard_rtl_submodules": _submodule_revisions(),
        "command": [str(part) for part in command],
    }


# ---------------------------------------------------------------------------
# Records
# ---------------------------------------------------------------------------


@dataclass
class AreaRecord:
    run_id: str
    board: str
    config: str
    attrs: str
    flow: str
    evidence: str
    recorded_at: str
    device: dict[str, Any]
    design: dict[str, Any]
    provenance: dict[str, Any]
    rtl_mutation: dict[str, Any]
    totals: dict[str, dict[str, Any]]
    totals_source: str = "report_utilization"
    totals_raw: dict[str, dict[str, Any]] = field(default_factory=dict)
    hierarchy: list[dict[str, Any]] = field(default_factory=list)
    reports: dict[str, str] = field(default_factory=dict)
    # What machine these totals describe, extracted from the same elaboration
    # that Vivado synthesized. The performance half records the same structure,
    # so a join between the two halves is a comparison of identity_id rather
    # than of config names. Optional: records written before fingerprinting
    # existed simply carry None.
    fingerprint: dict[str, Any] | None = None

    def to_json(self) -> dict[str, Any]:
        return {
            "schema": SCHEMA,
            "run_id": self.run_id,
            "recorded_at": self.recorded_at,
            "evidence": self.evidence,
            "flow": self.flow,
            "board": self.board,
            "config": self.config,
            "device": self.device,
            "design": self.design,
            "provenance": self.provenance,
            "rtl_mutation": self.rtl_mutation,
            "fingerprint": self.fingerprint,
            "totals": self.totals,
            "totals_source": self.totals_source,
            "totals_raw": self.totals_raw,
            "hierarchy": self.hierarchy,
            "reports": self.reports,
        }

    @property
    def identity_id(self) -> str | None:
        """The join key against a performance record, when one was recorded."""
        return (self.fingerprint or {}).get("identity_id")

    @property
    def comparison_key(self) -> tuple[str, str, str, str]:
        """Records may only be compared within one of these groups.

        The discriminator is whether the totals are an upper bound, not which
        attribute mode produced them. ``keep_hierarchy`` inhibits cross-boundary
        optimization and inflates totals, so it cannot share an axis with
        anything. ``use_dsp`` only changes which primitive the mapper picks, and
        measuring the resulting shift is the entire point -- so pristine and
        ``dsp`` runs are legitimately comparable and must be plottable together.
        """
        part = str(self.device.get("part", "unknown"))
        bound = "upper-bound" if self.totals_are_upper_bound else "exact"
        return (self.flow, bound, part, self.board)

    @property
    def luts(self) -> int | None:
        entry = self.totals.get("luts")
        return None if entry is None else entry.get("used")

    @property
    def totals_are_upper_bound(self) -> bool:
        return bool(self.rtl_mutation.get("totals_are_upper_bound"))


def make_run_id(board: str, config: str, attrs: str, when: datetime | None = None) -> str:
    stamp = (when or datetime.now(timezone.utc)).strftime("%Y%m%dT%H%M%SZ")
    return f"{stamp}-{board}-{config}-attrs_{attrs}"


def _read_json(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text())
    except FileNotFoundError as exc:
        raise AreaError(f"expected Vivado manifest is missing: {path}") from exc
    except json.JSONDecodeError as exc:
        raise AreaError(f"malformed Vivado manifest {path}: {exc}") from exc


def collect_record(
    board: Board,
    attrs: str,
    command: Sequence[str],
    run_id: str | None = None,
    report_dir: Path | None = None,
) -> AreaRecord:
    """Build a record from the reports a completed ``rb-area`` run left behind.

    ``report_dir`` defaults to the config-specific Vivado output directory and
    exists so the collection path can be exercised against a fixture.
    """
    if report_dir is None:
        report_dir = board.generated_dir / "obj" / "report"
    manifest = _read_json(report_dir / "rb_manifest.json")

    hier_path = report_dir / "rb_utilization_hier.txt"
    flat_path = report_dir / "rb_utilization_flat.txt"
    if not hier_path.is_file():
        raise AreaError(
            f"hierarchical utilization report is missing: {hier_path}\n"
            "The synthesis run did not complete."
        )

    totals_raw = parse_flat_utilization(flat_path.read_text()) if flat_path.is_file() else {}
    totals = normalize_totals(totals_raw)
    hierarchy = parse_hierarchical_utilization(hier_path.read_text())
    totals_source = "report_utilization"

    if not totals:
        # No flat device report, or its format was not recognized. The top row of
        # the hierarchical report still gives usage, just without capacity.
        totals = totals_from_hierarchy(hierarchy)
        totals_source = "hierarchy-top-row"

    if not totals:
        raise AreaError(
            f"no device totals could be parsed from {flat_path} or {hier_path}. "
            "The Vivado report format may have changed; parse_flat_utilization "
            "and parse_hierarchical_utilization need updating."
        )

    mutation = dict(manifest.get("rtl_mutation", {}))
    if mutation.get("mode") != attrs:
        raise AreaError(
            f"requested attribute mode {attrs!r} but the Vivado manifest records "
            f"{mutation.get('mode')!r}. The report directory is stale — re-run with --force."
        )

    # Extracted from the same generated-src Vivado read, so the fingerprint and
    # the utilization numbers cannot drift apart. A missing or unparseable DTS
    # must not lose a completed synthesis run, so this degrades to None.
    try:
        soc_fingerprint = fingerprint.for_board(board).as_dict()
    except (fingerprint.FingerprintError, OSError, ValueError):
        soc_fingerprint = None

    return AreaRecord(
        run_id=run_id or make_run_id(board.name, board.config, attrs),
        board=board.name,
        config=board.config,
        attrs=attrs,
        flow=str(manifest.get("flow", "synth-only")),
        evidence="synthesized",
        recorded_at=datetime.now(timezone.utc).isoformat(timespec="seconds"),
        device=dict(manifest.get("device", {}), tool=manifest.get("tool", {})),
        design=dict(
            manifest.get("design", {}),
            harness=board.harness,
            package=board.package,
            synth_options=manifest.get("synth_options"),
            synthesized_at=manifest.get("synthesized_at"),
        ),
        provenance=collect_provenance(command),
        rtl_mutation=mutation,
        fingerprint=soc_fingerprint,
        totals=totals,
        totals_source=totals_source,
        totals_raw=totals_raw,
        hierarchy=hierarchy,
        reports={name: str(report_dir / name) for name in manifest.get("reports", [])},
    )


def write_record(record: AreaRecord, results_dir: Path = RESULTS_DIR) -> Path:
    out = results_dir / record.run_id
    out.mkdir(parents=True, exist_ok=True)
    (out / "record.json").write_text(json.dumps(record.to_json(), indent=2) + "\n")
    write_summary_csv([record], out / "summary.csv")
    write_hierarchy_csv(record, out / "hierarchy.csv")
    return out


def write_summary_csv(records: Sequence[AreaRecord], path: Path) -> Path:
    keys = list(RESOURCE_ALIASES)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "run_id",
                "board",
                "part",
                "config",
                "flow",
                "attrs",
                "totals_are_upper_bound",
                *[f"{k}_used" for k in keys],
                *[f"{k}_available" for k in keys],
                *[f"{k}_util_pct" for k in keys],
            ]
        )
        for record in records:
            row: list[Any] = [
                record.run_id,
                record.board,
                record.device.get("part"),
                record.config,
                record.flow,
                record.attrs,
                record.totals_are_upper_bound,
            ]
            for field_name in ("used", "available", "util_pct"):
                for key in keys:
                    entry = record.totals.get(key) or {}
                    row.append(entry.get(field_name))
            writer.writerow(row)
    return path


def write_hierarchy_csv(record: AreaRecord, path: Path) -> Path:
    columns: list[str] = []
    for row in record.hierarchy:
        for name in row["metrics"]:
            if name not in columns:
                columns.append(name)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["instance", "module", "depth", *columns])
        for row in record.hierarchy:
            writer.writerow(
                [
                    row["instance"],
                    row["module"],
                    row["depth"],
                    *[row["metrics"].get(name) for name in columns],
                ]
            )
    return path


def load_record(path: Path) -> AreaRecord:
    """Load a record written by :func:`write_record`."""
    if path.is_dir():
        path = path / "record.json"
    data = json.loads(path.read_text())
    schema = data.get("schema")
    if schema != SCHEMA:
        raise AreaError(f"{path}: expected schema {SCHEMA}, found {schema!r}")
    return AreaRecord(
        run_id=data["run_id"],
        board=data["board"],
        config=data["config"],
        attrs=data.get("rtl_mutation", {}).get("mode", "unknown"),
        flow=data["flow"],
        evidence=data.get("evidence", "synthesized"),
        recorded_at=data["recorded_at"],
        device=data.get("device", {}),
        design=data.get("design", {}),
        provenance=data.get("provenance", {}),
        rtl_mutation=data.get("rtl_mutation", {}),
        totals=data.get("totals", {}),
        totals_source=data.get("totals_source", "report_utilization"),
        totals_raw=data.get("totals_raw", {}),
        hierarchy=data.get("hierarchy", []),
        reports=data.get("reports", {}),
        fingerprint=data.get("fingerprint"),
    )


def load_records(results_dir: Path = RESULTS_DIR) -> list[AreaRecord]:
    if not results_dir.is_dir():
        return []
    records = []
    for candidate in sorted(results_dir.glob("*/record.json")):
        records.append(load_record(candidate))
    return records


def latest_per_config(records: Iterable[AreaRecord]) -> list[AreaRecord]:
    """Keep only the newest record for each (board, config, flow, attrs)."""
    newest: dict[tuple[str, str, str, str], AreaRecord] = {}
    for record in sorted(records, key=lambda r: r.recorded_at):
        newest[(record.board, record.config, record.flow, record.attrs)] = record
    return list(newest.values())


def group_comparable(records: Iterable[AreaRecord]) -> dict[tuple[str, str, str, str], list[AreaRecord]]:
    """Partition records into sets that may legitimately share a plot axis.

    Records differing in flow, attribute-injection mode, device part, or board
    are never comparable: synth-only totals are not post-route totals, and
    keep_hierarchy makes totals an upper bound.
    """
    groups: dict[tuple[str, str, str, str], list[AreaRecord]] = {}
    for record in records:
        groups.setdefault(record.comparison_key, []).append(record)
    for bucket in groups.values():
        bucket.sort(key=lambda r: (r.luts or 0))
    return groups


# ---------------------------------------------------------------------------
# Running a measurement
# ---------------------------------------------------------------------------


def make_command(board: Board, attrs: str, force: bool) -> list[str]:
    if attrs not in ATTR_MODES:
        raise AreaError(f"--attrs must be one of {', '.join(ATTR_MODES)}, got {attrs!r}")
    return [
        "make",
        "-C",
        str(CHIPYARD / "fpga"),
        f"SUB_PROJECT={board.sub_project}",
        f"CONFIG={board.config}",
        f"RB_ATTRS={attrs}",
        "rb-area-force" if force else "rb-area",
    ]


def restore_command(board: Board) -> list[str]:
    return [
        "make",
        "-C",
        str(CHIPYARD / "fpga"),
        f"SUB_PROJECT={board.sub_project}",
        f"CONFIG={board.config}",
        "rb-area-restore",
    ]


def describe_mutation(record: AreaRecord) -> list[str]:
    """Human-readable lines describing what was done to the RTL, if anything."""
    mutation = record.rtl_mutation
    if mutation.get("pristine"):
        return ["RTL: pristine generated output, no attributes injected"]
    lines = [f"RTL: attribute injection mode '{mutation.get('mode')}'"]
    if mutation.get("totals_are_upper_bound"):
        lines.append(
            "     device totals are an UPPER BOUND (keep_hierarchy inhibits "
            "cross-boundary optimization)"
        )
    applied = [
        inj
        for inj in mutation.get("injections", [])
        if inj.get("module_status") == "applied" or inj.get("wire_status") == "applied"
    ]
    missing = [
        inj for inj in mutation.get("injections", []) if inj.get("wire_status") == "not-found"
    ]
    lines.append(f"     {len(applied)} injection(s) applied")
    for inj in missing:
        lines.append(
            f"     WARNING: {inj.get('file')}: expected wire pattern not found — "
            "generated RTL changed shape, DSP hint incomplete"
        )
    return lines
