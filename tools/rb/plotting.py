"""Dependency-free SVG rendering for RiskyBird resource-utilization figures.

Plotting deliberately has no third-party dependency. Three reasons:

* A figure must be reproducible from an archived record years later, without
  resolving a Python environment.
* Coupling plotting to the Chipyard or Zephyr Conda environments would make
  ``rb plot`` unusable outside a backend shell.
* SVG is text, so a committed figure diffs meaningfully and converts to PDF with
  any tool for publication.

Every figure is emitted alongside the CSV it was rendered from, so the numbers
behind a plotted bar are always inspectable.

Colours are chosen to stay legible in light and dark viewers and to remain
distinguishable in grayscale; the palette is ordered by luminance so a
stacked bar reads as a ramp rather than a jumble.
"""

from __future__ import annotations

import html
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

from .area import AreaRecord


# Ordered by increasing luminance so stacks read as a ramp and survive grayscale.
SERIES_COLORS = (
    "#1f4e79",  # deep blue
    "#2e7d9a",  # teal
    "#3f9e6f",  # green
    "#c88a2e",  # amber
    "#b5533c",  # terracotta
    "#7b5aa6",  # violet
    "#6b7280",  # slate
)

# Approximate advance width of the 11px sans label face, used to size gutters
# so text is never clipped. Slightly generous: overshooting adds whitespace,
# undershooting cuts characters off.
CHAR_W = 6.1

AXIS = "#6b7280"
TEXT = "#111827"
GRID = "#d1d5db"
OVER = "#b5533c"

# Resource keys rendered by the breakdown chart, with display labels.
BREAKDOWN_RESOURCES = (
    ("luts", "LUTs"),
    ("registers", "Registers"),
    ("brams", "BRAM tiles"),
    ("dsps", "DSPs"),
)


class PlotError(RuntimeError):
    """A figure could not be produced from the given records."""


@dataclass
class _Bar:
    label: str
    value: float
    capacity: float | None
    note: str = ""


def _elide(text: str, limit: int) -> str:
    """Shorten from the middle, so both ends stay readable.

    Instance names disambiguate on the left and module names on the right, so
    cutting the tail (the old behaviour) threw away the half that identifies
    what the row actually is.
    """
    if len(text) <= limit:
        return text
    keep = (limit - 1) // 2
    return f"{text[:keep]}…{text[-(limit - keep - 1):]}"


def _short_config(config: str) -> str:
    """A legend-sized name that keeps what distinguishes one config from another."""
    name = re.sub(r"KU040Config$|Config$", "", config)
    return _elide(name, 30)


def _esc(text: str) -> str:
    return html.escape(str(text), quote=True)


def _svg_open(width: int, height: int, title: str, desc: str) -> list[str]:
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
        f'width="{width}" height="{height}" role="img" '
        f'aria-labelledby="title desc" font-family="Inter, Helvetica, Arial, sans-serif">',
        f"<title id=\"title\">{_esc(title)}</title>",
        f"<desc id=\"desc\">{_esc(desc)}</desc>",
        # Light/dark aware without JavaScript: media query plus an explicit
        # data-theme override for viewers that stamp the root element.
        "<style>"
        f".bg{{fill:#ffffff}} .tx{{fill:{TEXT}}} .ax{{stroke:{AXIS}}} .gr{{stroke:{GRID}}}"
        "@media (prefers-color-scheme: dark){"
        ".bg{fill:#0b0f19} .tx{fill:#e5e7eb} .ax{stroke:#9ca3af} .gr{stroke:#374151}}"
        ':root[data-theme="dark"] .bg{fill:#0b0f19}'
        ':root[data-theme="dark"] .tx{fill:#e5e7eb}'
        ':root[data-theme="dark"] .ax{stroke:#9ca3af}'
        ':root[data-theme="dark"] .gr{stroke:#374151}'
        ':root[data-theme="light"] .bg{fill:#ffffff}'
        f':root[data-theme="light"] .tx{{fill:{TEXT}}}'
        "</style>",
        f'<rect class="bg" x="0" y="0" width="{width}" height="{height}"/>',
    ]


def _nice_ticks(maximum: float, count: int = 5) -> list[float]:
    """Round tick values that always reach at least `maximum`.

    Covering the maximum is not cosmetic. Callers scale their geometry by
    ``ticks[-1]``, so an axis that stops short renders anything above it outside
    the plot area: with the KU040's 242,400 LUT capacity the axis used to end at
    200,000 and the longest bars overran their frame by 110px, straight through
    the value labels.
    """
    if maximum <= 0:
        return [0.0]
    raw = maximum / count
    magnitude = 10 ** (len(str(int(raw))) - 1) if raw >= 1 else 1
    for factor in (1, 2, 2.5, 5, 10):
        step = magnitude * factor
        if step >= raw:
            break
    ticks = []
    value = 0.0
    while value <= maximum + step * 0.001:
        ticks.append(value)
        value += step
    # The loop stops below `maximum` whenever it is not a multiple of the step.
    if ticks[-1] < maximum:
        ticks.append(ticks[-1] + step)
    return ticks


def _fmt(value: float) -> str:
    if value >= 1_000_000:
        return f"{value / 1_000_000:.1f}M"
    if value >= 1_000:
        return f"{value / 1_000:.0f}K"
    return f"{value:.0f}"


def _horizontal_bars(
    bars: Sequence[_Bar],
    title: str,
    subtitle: str,
    unit: str,
    footnote: str,
) -> str:
    if not bars:
        raise PlotError("no data to plot")

    # Size the label gutter and the annotation margin to the content. Fixed
    # widths truncated instance names and pushed the longest ones off the left
    # edge -- a hierarchy label can run to 60 characters, and a bar at full
    # scale puts its value label hard against the right border.
    def annotation_of(bar: _Bar) -> str:
        text = f"{bar.value:,.0f}"
        if bar.capacity:
            text += f"  ({bar.value / bar.capacity * 100:.1f}%)"
        if bar.note:
            text += f"  {bar.note}"
        return text

    label_w = max(150, min(420, int(max(len(b.label) for b in bars) * CHAR_W) + 22))
    right_w = int(max(len(annotation_of(b)) for b in bars) * CHAR_W) + 26
    plot_w = 520
    row_h = 30
    pad_top = 78
    pad_bottom = 34 + (18 if footnote else 0)
    width = label_w + plot_w + right_w
    height = pad_top + row_h * len(bars) + pad_bottom

    # Scale to capacity when every bar has one, so bars are comparable to the
    # device limit rather than only to each other.
    capacities = [b.capacity for b in bars if b.capacity]
    scale_max = max(
        max(b.value for b in bars),
        max(capacities) if len(capacities) == len(bars) else 0,
    )
    ticks = _nice_ticks(scale_max)
    axis_max = ticks[-1] if ticks else 1.0

    out = _svg_open(width, height, title, subtitle or title)
    out.append(
        f'<text class="tx" x="16" y="28" font-size="15" font-weight="600">{_esc(title)}</text>'
    )
    if subtitle:
        out.append(
            f'<text class="tx" x="16" y="47" font-size="11" opacity="0.75">{_esc(subtitle)}</text>'
        )

    x0 = label_w
    for tick in ticks:
        x = x0 + (tick / axis_max) * plot_w
        out.append(
            f'<line class="gr" x1="{x:.1f}" y1="{pad_top - 12}" x2="{x:.1f}" '
            f'y2="{pad_top + row_h * len(bars)}" stroke-width="1"/>'
        )
        out.append(
            f'<text class="tx" x="{x:.1f}" y="{pad_top - 18}" font-size="10" '
            f'text-anchor="middle" opacity="0.7">{_fmt(tick)}</text>'
        )
    out.append(
        f'<text class="tx" x="{x0 + plot_w / 2:.1f}" y="{pad_top - 34}" font-size="10" '
        f'text-anchor="middle" opacity="0.6">{_esc(unit)}</text>'
    )

    for index, bar in enumerate(bars):
        y = pad_top + index * row_h
        bar_h = row_h - 11
        out.append(
            f'<text class="tx" x="{label_w - 10}" y="{y + bar_h - 1}" font-size="11" '
            f'text-anchor="end">{_esc(bar.label)}</text>'
        )
        # Capacity track, when known.
        if bar.capacity:
            cap_w = (bar.capacity / axis_max) * plot_w
            out.append(
                f'<rect x="{x0}" y="{y}" width="{cap_w:.1f}" height="{bar_h}" '
                f'fill="none" stroke="{GRID}" stroke-width="1" stroke-dasharray="2 2"/>'
            )
        value_w = (bar.value / axis_max) * plot_w if axis_max else 0
        over = bool(bar.capacity and bar.value > bar.capacity)
        # Cycle the palette so adjacent rows separate; red stays reserved for
        # over-capacity, which must never be mistaken for a series colour.
        color = OVER if over else SERIES_COLORS[index % len(SERIES_COLORS)]
        out.append(
            f'<rect x="{x0}" y="{y}" width="{max(value_w, 1):.1f}" height="{bar_h}" '
            f'fill="{color}" rx="2"><title>{_esc(bar.label)}: {bar.value:,.0f}</title></rect>'
        )
        annotation = annotation_of(bar)
        out.append(
            f'<text class="tx" x="{x0 + value_w + 8:.1f}" y="{y + bar_h - 1}" '
            f'font-size="10" opacity="0.85">{_esc(annotation)}</text>'
        )

    if footnote:
        out.append(
            f'<text class="tx" x="16" y="{height - 12}" font-size="9.5" '
            f'opacity="0.65">{_esc(footnote)}</text>'
        )
    out.append("</svg>")
    return "\n".join(out) + "\n"


def _require_one_group(records: Sequence[AreaRecord]) -> None:
    keys = {r.comparison_key for r in records}
    if len(keys) > 1:
        rendered = "\n  ".join(
            f"flow={k[0]} attrs={k[1]} part={k[2]} board={k[3]}" for k in sorted(keys)
        )
        raise PlotError(
            "refusing to plot records from different comparison groups on one "
            f"axis:\n  {rendered}\n"
            "synth-only and post-route totals are not comparable, and "
            "keep_hierarchy makes totals an upper bound. Filter to one group."
        )


def resource_chart(
    records: Sequence[AreaRecord],
    resource: str = "luts",
    path: Path | None = None,
) -> str:
    """One bar per config for a single resource, against device capacity."""
    if not records:
        raise PlotError("no records to plot")
    _require_one_group(records)

    label = dict(BREAKDOWN_RESOURCES).get(resource, resource)
    bars: list[_Bar] = []
    for record in sorted(records, key=lambda r: (r.totals.get(resource) or {}).get("used") or 0):
        entry = record.totals.get(resource)
        if entry is None or entry.get("used") is None:
            continue
        bars.append(
            _Bar(
                label=record.config,
                value=float(entry["used"]),
                capacity=float(entry["available"]) if entry.get("available") else None,
                note="upper bound" if record.totals_are_upper_bound else "",
            )
        )
    if not bars:
        raise PlotError(f"no record reports the resource {resource!r}")

    sample = records[0]
    part = sample.device.get("part", "unknown part")
    subtitle = (
        f"{part} · {sample.flow} · attrs={sample.attrs} · "
        f"vivado {sample.device.get('tool', {}).get('version', '?')}"
    )
    footnote = f"totals from {sample.totals_source}"
    if sample.totals_source == "hierarchy-top-row":
        footnote += " (device capacity unavailable, percentages omitted)"
    if sample.totals_are_upper_bound:
        footnote += " · keep_hierarchy injected: totals are an UPPER BOUND"

    svg = _horizontal_bars(bars, f"{label} by configuration", subtitle, label, footnote)
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(svg)
    return svg


def hierarchy_chart(
    record: AreaRecord,
    resource: str = "Total LUTs",
    depth: int = 2,
    top: int = 14,
    path: Path | None = None,
) -> str:
    """Where one build's resource goes, by instance at a chosen hierarchy level.

    This is the figure that answers "what is actually consuming the device" — the
    per-module breakdown the upstream work produced by hand.
    """
    rows = [r for r in record.hierarchy if r["metrics"].get(resource)]
    if not rows:
        raise PlotError(f"record {record.run_id} has no hierarchy rows with {resource!r}")

    depths = sorted({r["depth"] for r in rows})
    if depth >= len(depths):
        depth = len(depths) - 1
    wanted = depths[depth]
    selected = [r for r in rows if r["depth"] == wanted and not r["self_only"]]
    if not selected:
        selected = [r for r in rows if r["depth"] == wanted]
    selected.sort(key=lambda r: r["metrics"][resource], reverse=True)
    selected = selected[:top]

    bars = [
        _Bar(
            label=_elide(f"{r['instance']}  ({r['module']})", 62),
            value=float(r["metrics"][resource]),
            capacity=None,
        )
        for r in selected
    ]
    total = next(
        (r["metrics"].get(resource) for r in record.hierarchy if r["depth"] == depths[0]),
        None,
    )
    subtitle = (
        f"{record.config} · {record.device.get('part', '?')} · {record.flow} · "
        f"attrs={record.attrs}"
    )
    footnote = f"hierarchy level {depth}, top {len(bars)} instances"
    if total:
        footnote += f" · design total {total:,.0f} {resource}"
    if record.totals_are_upper_bound:
        footnote += " · keep_hierarchy injected"
    else:
        footnote += " · no keep_hierarchy: boundary optimization may shift attribution"

    svg = _horizontal_bars(bars, f"{resource} by instance", subtitle, resource, footnote)
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(svg)
    return svg


def utilization_chart(record: AreaRecord, path: Path | None = None) -> str:
    """One build against its device limits across every headline resource."""
    bars: list[_Bar] = []
    for key, label in BREAKDOWN_RESOURCES:
        entry = record.totals.get(key)
        if entry is None or entry.get("used") is None:
            continue
        capacity = entry.get("available")
        if not capacity:
            continue
        bars.append(
            _Bar(
                label=f"{label}  ({entry['vivado_row']})",
                value=float(entry["used"]) / float(capacity) * 100.0,
                capacity=100.0,
            )
        )
    if not bars:
        raise PlotError(
            f"record {record.run_id} has no resource with a device capacity "
            f"(totals came from {record.totals_source}); a flat "
            "report_utilization is required for a utilization chart"
        )
    subtitle = f"{record.config} · {record.device.get('part', '?')} · {record.flow}"
    footnote = "percentage of device capacity, as reported by Vivado"
    if record.totals_are_upper_bound:
        footnote += " · keep_hierarchy injected: UPPER BOUND"
    svg = _horizontal_bars(bars, "Device utilization", subtitle, "% of device", footnote)
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(svg)
    return svg


def injection_chart(
    records: Sequence[AreaRecord],
    resource: str = "luts",
    path: Path | None = None,
) -> str:
    """Pristine vs `use_dsp` for every config measured both ways.

    Two bars per configuration, so the resource shift the injection causes is
    read directly. Only valid because `use_dsp` changes primitive selection
    rather than inhibiting optimization -- a `keep_hierarchy` record would be an
    upper bound and is excluded by the comparison-group check.
    """
    pairs: dict[str, dict[str, AreaRecord]] = {}
    for record in records:
        if record.attrs in ("none", "dsp"):
            pairs.setdefault(record.config, {})[record.attrs] = record
    complete = {c: p for c, p in pairs.items() if "none" in p and "dsp" in p}
    if not complete:
        raise PlotError("no configuration has both a pristine and a dsp record")
    _require_one_group([r for p in complete.values() for r in p.values()])

    bars: list[_Bar] = []
    order = sorted(
        complete.items(),
        key=lambda kv: -((kv[1]["none"].totals.get(resource) or {}).get("used") or 0),
    )
    capacity = None
    for config, pair in order:
        for mode in ("none", "dsp"):
            entry = pair[mode].totals.get(resource)
            if entry is None or entry.get("used") is None:
                continue
            capacity = capacity or (float(entry["available"]) if entry.get("available") else None)
            base = (pair["none"].totals.get(resource) or {}).get("used")
            used = entry["used"]
            note = ""
            if mode == "dsp" and base:
                note = f"{(used - base) / base * 100:+.1f}%"
            bars.append(
                _Bar(
                    label=f"{config[:34]}  [{'pristine' if mode == 'none' else 'use_dsp'}]",
                    value=float(used),
                    capacity=capacity,
                    note=note,
                )
            )

    label = dict(BREAKDOWN_RESOURCES).get(resource, resource)
    sample = next(iter(complete.values()))["none"]
    subtitle = (
        f"{sample.device.get('part', '?')} · {sample.flow} · "
        f"vivado {sample.device.get('tool', {}).get('version', '?')} · "
        "dashed track = device capacity"
    )
    svg = _horizontal_bars(
        bars,
        f"{label}: pristine vs use_dsp injection",
        subtitle,
        label,
        "use_dsp maps small signed multipliers to DSP48E2 instead of LUTs; "
        "totals are exact, not an upper bound",
    )
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(svg)
    return svg


# ---------------------------------------------------------------------------
# Design-space figures: the two halves against each other
# ---------------------------------------------------------------------------

# Verdict drives colour, so the constraint reads before the position does.
VERDICT_COLORS = {
    "ok": "#3f9e6f",
    "extrapolated": "#c88a2e",
    "misses-deadline": "#b5533c",
    "infeasible": "#6b7280",
}


def tradeoff_chart(
    points: Sequence[Any],
    resource: str = "luts",
    objective: str = "makespan_ms",
    path: Path | None = None,
) -> str:
    """Resource against latency, one mark per configuration.

    The question the design space actually asks -- what does this configuration
    cost, and is it fast enough -- so both axes are "smaller is better" and the
    frontier runs along the bottom-left. Device capacity is drawn as a hard
    vertical line because exceeding it is not a trade-off, and points are
    coloured by verdict so a borrowed number never reads as a measured one.
    """
    from .dse import pareto  # local: dse imports plotting for its own figures

    usable = [
        p for p in points
        if p.resource_value(resource) is not None
        and p.performance.get(objective) is not None
    ]
    if not usable:
        raise PlotError("no points carry both a resource measurement and a latency")

    plot_w, plot_h = 560, 360
    # Right gutter sized to the legend so entries are never clipped.
    legend_w = int(max((len(_short_config(p.config)) for p in points), default=10) * CHAR_W) + 46
    pad_l, pad_t, pad_r, pad_b = 78, 84, max(210, legend_w), 66
    width, height = pad_l + plot_w + pad_r, pad_t + plot_h + pad_b

    capacity = next(
        (
            (p.resource.get("utilization") or {}).get(resource, {}).get("available")
            for p in usable
            if (p.resource.get("utilization") or {}).get(resource, {}).get("available")
        ),
        None,
    )
    x_max = max([p.resource_value(resource) for p in usable] + ([capacity] if capacity else []))
    y_max = max(float(p.performance[objective]) for p in usable)
    x_ticks, y_ticks = _nice_ticks(x_max), _nice_ticks(y_max)
    x_axis = x_ticks[-1] or 1.0
    y_axis = y_ticks[-1] or 1.0

    label = dict(BREAKDOWN_RESOURCES).get(resource, resource)
    clocks = {p.performance.get("clock_mhz") for p in usable}
    clock_note = (
        f"at {clocks.pop():g} MHz" if len(clocks) == 1 and None not in clocks
        else "at mixed clocks"
    )
    title = f"{label} against schedule latency"
    subtitle = f"one mark per configuration; {clock_note}; lower-left is better"

    out = _svg_open(width, height, title, subtitle)
    out.append(f'<text class="tx" x="16" y="28" font-size="15" font-weight="600">{_esc(title)}</text>')
    out.append(f'<text class="tx" x="16" y="47" font-size="11" opacity="0.75">{_esc(subtitle)}</text>')

    def sx(value: float) -> float:
        return pad_l + (value / x_axis) * plot_w

    def sy(value: float) -> float:
        return pad_t + plot_h - (value / y_axis) * plot_h

    for tick in x_ticks:
        x = sx(tick)
        out.append(f'<line class="gr" x1="{x:.1f}" y1="{pad_t}" x2="{x:.1f}" y2="{pad_t + plot_h}" stroke-width="1"/>')
        out.append(
            f'<text class="tx" x="{x:.1f}" y="{pad_t + plot_h + 18}" font-size="10" '
            f'text-anchor="middle" opacity="0.7">{_fmt(tick)}</text>'
        )
    for tick in y_ticks:
        y = sy(tick)
        out.append(f'<line class="gr" x1="{pad_l}" y1="{y:.1f}" x2="{pad_l + plot_w}" y2="{y:.1f}" stroke-width="1"/>')
        out.append(
            f'<text class="tx" x="{pad_l - 10}" y="{y + 3:.1f}" font-size="10" '
            f'text-anchor="end" opacity="0.7">{_fmt(tick)}</text>'
        )
    out.append(
        f'<text class="tx" x="{pad_l + plot_w / 2:.1f}" y="{height - 30}" font-size="11" '
        f'text-anchor="middle" opacity="0.8">{_esc(label)}</text>'
    )
    out.append(
        f'<text class="tx" x="18" y="{pad_t + plot_h / 2:.1f}" font-size="11" opacity="0.8" '
        f'text-anchor="middle" transform="rotate(-90 18 {pad_t + plot_h / 2:.1f})">latency (ms)</text>'
    )

    # Capacity: everything right of this line does not exist.
    if capacity and capacity <= x_axis:
        x = sx(capacity)
        out.append(
            f'<line x1="{x:.1f}" y1="{pad_t}" x2="{x:.1f}" y2="{pad_t + plot_h}" '
            f'stroke="{OVER}" stroke-width="1.5" stroke-dasharray="5 3"/>'
        )
        out.append(
            f'<text x="{x - 6:.1f}" y="{pad_t - 8}" font-size="10" fill="{OVER}" '
            f'text-anchor="end">device capacity</text>'
        )

    # Frontier first, so marks sit on top of it.
    try:
        frontier = pareto(usable, resource=resource, objective=objective)
    except Exception:  # a frontier is a nicety; the marks are the data
        frontier = []
    if len(frontier) > 1:
        path_points = " ".join(
            f"{sx(p.resource_value(resource)):.1f},{sy(float(p.performance[objective])):.1f}"
            for p in frontier
        )
        out.append(
            f'<polyline points="{path_points}" fill="none" stroke="{SERIES_COLORS[0]}" '
            f'stroke-width="1.2" stroke-dasharray="4 3" opacity="0.55"/>'
        )

    on_frontier = {id(p) for p in frontier}
    ordered = sorted(usable, key=lambda p: p.resource_value(resource) or 0.0)
    for point in ordered:
        x = sx(point.resource_value(resource))
        y = sy(float(point.performance[objective]))
        color = VERDICT_COLORS.get(point.verdict, SERIES_COLORS[0])
        radius = 6.5 if id(point) in on_frontier else 4.5
        latency = float(point.performance[objective])
        detail = (
            f"{point.config} [{point.attrs}] — "
            f"{point.resource_value(resource):,.0f} {label}, {latency:,.1f} ms, {point.verdict}"
        )
        out.append(
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{radius}" fill="{color}" '
            f'stroke="#ffffff" stroke-width="1.2"><title>{_esc(detail)}</title></circle>'
        )

    # Legend, in the right-hand gutter so it never overlaps a mark.
    lx = pad_l + plot_w + 22
    out.append(f'<text class="tx" x="{lx}" y="{pad_t + 4}" font-size="10" font-weight="600">configurations</text>')
    for index, point in enumerate(ordered):
        ly = pad_t + 22 + index * 16
        color = VERDICT_COLORS.get(point.verdict, SERIES_COLORS[0])
        out.append(f'<circle cx="{lx + 5}" cy="{ly - 4}" r="4" fill="{color}"/>')
        out.append(
            f'<text class="tx" x="{lx + 15}" y="{ly}" font-size="9.5" opacity="0.9">'
            f'{_esc(_short_config(point.config))}</text>'
        )

    verdicts = [v for v in ("ok", "extrapolated", "misses-deadline", "infeasible")
                if any(p.verdict == v for p in usable)]
    out.append(
        f'<text class="tx" x="16" y="{height - 12}" font-size="9.5" opacity="0.65">'
        + _esc("verdict: " + " · ".join(verdicts)
               + ". Dashed line is the Pareto frontier; extrapolated points use a "
                 "profile captured on another machine.")
        + "</text>"
    )
    out.append("</svg>")
    svg = "\n".join(out) + "\n"
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(svg)
    return svg


def deadline_chart(points: Sequence[Any], path: Path | None = None) -> str:
    """Per-network latency against its deadline, one bar per configuration.

    Slack is the quantity that decides admissibility, so it is drawn directly
    rather than left to be inferred from two numbers.
    """
    bars: list[_Bar] = []
    for point in sorted(points, key=lambda p: p.config):
        for name, value in sorted((point.performance.get("networks") or {}).items()):
            deadline = value.get("deadline_ms")
            latency = value.get("worst_latency_ms")
            if deadline is None or latency is None:
                continue
            short = point.config.replace("KU040Config", "").replace("Config", "")
            note = "MISS" if latency > deadline else ""
            bars.append(_Bar(
                label=f"{name} · {short[:18]}",
                value=float(latency),
                capacity=float(deadline),
                note=note,
            ))
    if not bars:
        raise PlotError("no point carries a network with a deadline")
    clocks = {p.performance.get("clock_mhz") for p in points}
    clock_note = f"at {clocks.pop():g} MHz" if len(clocks) == 1 and None not in clocks else "at mixed clocks"
    svg = _horizontal_bars(
        bars,
        title="Latency against deadline",
        subtitle=f"dashed track is the deadline; {clock_note}",
        unit="milliseconds",
        footnote=(
            "Scheduled from measured per-op cycles, converted once at the stated "
            "clock. Not a measurement on hardware."
        ),
    )
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(svg)
    return svg


def write_dse_figures(points: Sequence[Any], out_dir: Path) -> list[Path]:
    """Every design-space figure the given points can support."""
    out_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    for resource, _label in BREAKDOWN_RESOURCES:
        target = out_dir / f"dse-tradeoff-{resource}.svg"
        try:
            tradeoff_chart(points, resource=resource, path=target)
        except PlotError:
            continue
        written.append(target)
    try:
        target = out_dir / "dse-deadlines.svg"
        deadline_chart(points, path=target)
        written.append(target)
    except PlotError:
        pass
    return written


def write_figures(records: Sequence[AreaRecord], out_dir: Path) -> list[Path]:
    """Render every figure that the given records can support."""
    out_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []

    from .area import group_comparable

    for key, bucket in sorted(group_comparable(records).items()):
        flow, bound, part, board = key
        stem = f"{board}-{part}-{flow}-{bound}".replace("/", "_")
        # Pristine vs use_dsp, the injection effect itself.
        for resource, _label in BREAKDOWN_RESOURCES:
            try:
                target = out_dir / f"{stem}-injection-{resource}.svg"
                injection_chart(bucket, resource, target)
                written.append(target)
            except PlotError:
                pass
        for resource, _label in BREAKDOWN_RESOURCES:
            if not any((r.totals.get(resource) or {}).get("used") is not None for r in bucket):
                continue
            target = out_dir / f"{stem}-{resource}.svg"
            try:
                resource_chart(bucket, resource, target)
            except PlotError:
                continue
            written.append(target)
        for record in bucket:
            # Level 2 is where the whole design still sits inside one instance,
            # so also emit the levels that actually separate the subsystems --
            # accelerators from core from caches, and then inside each.
            for label, level in (("hierarchy", 2), ("subsystems", 5), ("units", 7)):
                try:
                    target = out_dir / f"{record.run_id}-{label}.svg"
                    hierarchy_chart(record, depth=level, path=target)
                    written.append(target)
                except PlotError:
                    pass
            try:
                target = out_dir / f"{record.run_id}-utilization.svg"
                utilization_chart(record, path=target)
                written.append(target)
            except PlotError:
                pass
    return written


# --------------------------------------------------------------------------- #
#  Co-design pair figures
#
#  These answer a different question from the charts above. Those plot one build;
#  these compare (hardware, software) *pairs*, because on this platform the two
#  are not independent: has_loop_conv decides whether conv2d can use
#  tiled_conv_auto at all, so a hardware choice selects a software path and the
#  pair is the unit that has a latency.
#
#  Two departures from the older figures in this module, both deliberate:
#
#  * Series colours are re-stepped for the dark surface rather than left fixed.
#    dse/scripts/plot_op_cycles.py went light-only because this module flipped the
#    background to near-black without restepping the fills, which is the right
#    complaint about the wrong half -- the fix is to stepping the fills, not to
#    drop dark mode.
#  * A single-series chart uses ONE colour for every bar. _horizontal_bars cycles
#    SERIES_COLORS by row index, which reads as though each bar were a different
#    series; here colour means "over the guideline" and nothing else.
#
#  Palette: slots 1-5 of the validated categorical order, light and dark steps.
#  Checked against this module's own surfaces (#ffffff / #0b0f19): lightness band,
#  chroma floor, adjacent CVD separation, normal-vision floor and contrast all
#  pass in both modes. Worst adjacent normal-vision dE is 19.6 light / 19.3 dark.
#  Three light steps fall under 3:1 against white, so every segment carries a
#  direct label or a legend entry and each figure is emitted with its CSV --
#  colour is never the only channel.
# --------------------------------------------------------------------------- #

CODESIGN_COLORS = (
    ("#2a78d6", "#3987e5"),  # blue
    ("#eb6834", "#d95926"),  # orange
    ("#1baf7a", "#199e70"),  # aqua
    ("#eda100", "#c98500"),  # yellow
    ("#e87ba4", "#d55181"),  # magenta
)

# One fill for a single-series magnitude chart, and the reserved status fill.
FILL_NEUTRAL = ("#2a78d6", "#3987e5")
FILL_OVER = ("#b5533c", "#e66767")

# Gap between stacked segments, in px. The surface shows through, so touching
# fills never blend into one another.
SEG_GAP = 2.0


def _fill_style(pairs: Sequence[tuple[str, str]], prefix: str = "s") -> str:
    """A <style> block giving each slot a light fill and a dark-surface fill."""
    light = " ".join(f".{prefix}{i}{{fill:{lo}}}" for i, (lo, _) in enumerate(pairs))
    dark = " ".join(f".{prefix}{i}{{fill:{hi}}}" for i, (_, hi) in enumerate(pairs))
    root = " ".join(
        f':root[data-theme="dark"] .{prefix}{i}{{fill:{hi}}}' for i, (_, hi) in enumerate(pairs)
    )
    back = " ".join(
        f':root[data-theme="light"] .{prefix}{i}{{fill:{lo}}}' for i, (lo, _) in enumerate(pairs)
    )
    return (
        f"<style>{light}"
        f"@media (prefers-color-scheme: dark){{{dark}}}"
        f"{root} {back}</style>"
    )


def _guideline(x: float, y0: float, y1: float, label: str) -> list[str]:
    """A dashed reference rule, labelled *below* the plot.

    The label used to sit above the rule, where it landed in the tick row and
    overprinted whichever tick it happened to fall near -- "80% guideline" on top
    of "200K". Below the last bar is empty in both of these figures.
    """
    return [
        f'<line class="ax" x1="{x:.1f}" y1="{y0:.1f}" x2="{x:.1f}" y2="{y1:.1f}" '
        f'stroke-width="1.5" stroke-dasharray="5 3" opacity="0.9"/>',
        f'<text class="tx" x="{x:.1f}" y="{y1 + 13:.1f}" font-size="9.5" '
        f'text-anchor="middle" opacity="0.8">{_esc(label)}</text>',
    ]


def codesign_area_chart(
    rows: Sequence[tuple[str, float, float]],
    guideline_frac: float = 0.80,
    title: str = "LUT utilization by configuration",
    subtitle: str = "",
    footnote: str = "",
    path: Path | None = None,
) -> str:
    """LUTs per configuration against device capacity and a routability guideline.

    `rows` is (label, used, available). One series, so no legend: the title names
    the measure. A bar past the guideline is drawn in the reserved status fill and
    also says so in its annotation, because colour alone is not an encoding.
    """
    if not rows:
        raise PlotError("no rows to plot")

    frac_txt = f"{guideline_frac * 100:.0f}%"
    annotations = [
        f"{used:,.0f}  ({used / avail * 100:.1f}%)"
        + ("  over " + frac_txt if used > avail * guideline_frac else "")
        for _, used, avail in rows
    ]
    label_w = max(150, min(420, int(max(len(r[0]) for r in rows) * CHAR_W) + 22))
    right_w = int(max(len(a) for a in annotations) * CHAR_W) + 26
    plot_w = 520
    row_h = 32
    pad_top = 84
    pad_bottom = 34 + (18 if footnote else 0)
    width = label_w + plot_w + right_w
    height = pad_top + row_h * len(rows) + pad_bottom

    capacity = max(r[2] for r in rows)
    ticks = _nice_ticks(max(max(r[1] for r in rows), capacity))
    axis_max = ticks[-1] if ticks else 1.0

    out = _svg_open(width, height, title, subtitle or title)
    out.append(_fill_style((FILL_NEUTRAL, FILL_OVER), prefix="f"))
    out.append(
        f'<text class="tx" x="16" y="28" font-size="15" font-weight="600">{_esc(title)}</text>'
    )
    if subtitle:
        out.append(
            f'<text class="tx" x="16" y="47" font-size="11" opacity="0.75">{_esc(subtitle)}</text>'
        )

    x0 = label_w
    y_top, y_bot = pad_top - 12, pad_top + row_h * len(rows)
    for tick in ticks:
        x = x0 + (tick / axis_max) * plot_w
        out.append(
            f'<line class="gr" x1="{x:.1f}" y1="{y_top}" x2="{x:.1f}" y2="{y_bot}" '
            f'stroke-width="1"/>'
        )
        out.append(
            f'<text class="tx" x="{x:.1f}" y="{pad_top - 18}" font-size="10" '
            f'text-anchor="middle" opacity="0.7">{_fmt(tick)}</text>'
        )
    # No axis unit label: the title already says LUTs, and at this pad_top it
    # printed straight through the subtitle.

    for index, (label, used, avail) in enumerate(rows):
        y = pad_top + index * row_h
        bar_h = row_h - 12
        out.append(
            f'<text class="tx" x="{label_w - 10}" y="{y + bar_h - 1}" font-size="11" '
            f'text-anchor="end">{_esc(label)}</text>'
        )
        cap_w = (avail / axis_max) * plot_w
        out.append(
            f'<rect x="{x0}" y="{y}" width="{cap_w:.1f}" height="{bar_h}" fill="none" '
            f'stroke="{GRID}" stroke-width="1" stroke-dasharray="2 2"/>'
        )
        value_w = (used / axis_max) * plot_w if axis_max else 0
        cls = "f1" if used > avail * guideline_frac else "f0"
        out.append(
            f'<rect class="{cls}" x="{x0}" y="{y}" width="{max(value_w, 1):.1f}" '
            f'height="{bar_h}" rx="4"><title>{_esc(label)}: {used:,.0f} of '
            f'{avail:,.0f} LUTs</title></rect>'
        )
        out.append(
            f'<text class="tx" x="{x0 + value_w + 8:.1f}" y="{y + bar_h - 1}" '
            f'font-size="10" opacity="0.85">{_esc(annotations[index])}</text>'
        )

    out += _guideline(
        x0 + (capacity * guideline_frac / axis_max) * plot_w,
        y_top,
        y_bot,
        f"{frac_txt} guideline",
    )
    if footnote:
        out.append(
            f'<text class="tx" x="16" y="{height - 12}" font-size="9.5" '
            f'opacity="0.65">{_esc(footnote)}</text>'
        )
    out.append("</svg>")
    svg = "\n".join(out) + "\n"
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(svg)
    return svg


def codesign_cycles_chart(
    pairs: Sequence[tuple[str, Sequence[tuple[str, float]]]],
    deadline_cycles: float | None = None,
    deadline_label: str = "",
    title: str = "Cycles by operation, per hardware/software pair",
    subtitle: str = "",
    footnote: str = "",
    path: Path | None = None,
) -> str:
    """Stacked cycle breakdown for each (hardware, software) pair.

    `pairs` is (label, [(op, cycles), ...]). Operation -> colour is fixed by the
    order operations first appear, never by their rank within a bar, so the same
    operation is the same colour in every bar and a pair that reorders its costs
    does not repaint them. Operations beyond the palette fold into "other" rather
    than generating a hue.
    """
    if not pairs:
        raise PlotError("no pairs to plot")

    # Fixed op -> slot assignment, by first appearance across all pairs.
    order: list[str] = []
    for _, ops in pairs:
        for op, _cycles in ops:
            if op not in order:
                order.append(op)
    limit = len(CODESIGN_COLORS) - 1
    keep = order[:limit]
    slot = {op: i for i, op in enumerate(keep)}
    folded = len(order) > limit
    legend = list(keep) + (["other"] if folded else [])

    def segments(ops: Sequence[tuple[str, float]]) -> list[tuple[int, str, float]]:
        acc: dict[int, float] = {}
        names: dict[int, str] = {}
        for op, cycles in ops:
            index = slot.get(op, limit)
            acc[index] = acc.get(index, 0.0) + cycles
            names[index] = op if op in slot else "other"
        return [(i, names[i], acc[i]) for i in sorted(acc)]

    totals = [sum(c for _, c in ops) for _, ops in pairs]
    annotations = [f"{t:,.0f} cyc" for t in totals]
    label_w = max(170, min(420, int(max(len(p[0]) for p in pairs) * CHAR_W) + 22))
    right_w = int(max(len(a) for a in annotations) * CHAR_W) + 26
    plot_w = 560
    row_h = 46
    pad_top = 92
    # Room for the per-segment % labels, then the guideline label, then the legend.
    legend_h = 44
    pad_bottom = 30 + legend_h + (18 if footnote else 0)
    width = label_w + plot_w + right_w
    height = pad_top + row_h * len(pairs) + pad_bottom

    ticks = _nice_ticks(max(totals + ([deadline_cycles] if deadline_cycles else [])))
    axis_max = ticks[-1] if ticks else 1.0

    out = _svg_open(width, height, title, subtitle or title)
    out.append(_fill_style(CODESIGN_COLORS))
    out.append(
        f'<text class="tx" x="16" y="28" font-size="15" font-weight="600">{_esc(title)}</text>'
    )
    if subtitle:
        out.append(
            f'<text class="tx" x="16" y="47" font-size="11" opacity="0.75">{_esc(subtitle)}</text>'
        )

    x0 = label_w
    y_top, y_bot = pad_top - 12, pad_top + row_h * len(pairs)
    for tick in ticks:
        x = x0 + (tick / axis_max) * plot_w
        out.append(
            f'<line class="gr" x1="{x:.1f}" y1="{y_top}" x2="{x:.1f}" y2="{y_bot}" '
            f'stroke-width="1"/>'
        )
        out.append(
            f'<text class="tx" x="{x:.1f}" y="{pad_top - 18}" font-size="10" '
            f'text-anchor="middle" opacity="0.7">{_fmt(tick)}</text>'
        )
    # No axis unit label: the title says cycles, and it collided with the subtitle.

    for index, (label, ops) in enumerate(pairs):
        y = pad_top + index * row_h
        bar_h = row_h - 20
        out.append(
            f'<text class="tx" x="{label_w - 10}" y="{y + bar_h / 2 + 4:.1f}" font-size="11" '
            f'text-anchor="end">{_esc(label)}</text>'
        )
        cursor = 0.0
        for slot_index, name, cycles in segments(ops):
            seg_w = (cycles / axis_max) * plot_w
            draw_w = max(seg_w - SEG_GAP, 1.0)
            share = cycles / totals[index] * 100 if totals[index] else 0
            out.append(
                f'<rect class="s{slot_index}" x="{x0 + cursor:.1f}" y="{y}" '
                f'width="{draw_w:.1f}" height="{bar_h}" rx="2">'
                f"<title>{_esc(name)}: {cycles:,.0f} cycles ({share:.1f}%)</title></rect>"
            )
            # Direct-label a segment only when its own width can hold the text;
            # the legend and the tooltip carry the rest. A number on every
            # segment is unreadable at this scale.
            if draw_w > 46:
                out.append(
                    f'<text class="tx" x="{x0 + cursor + draw_w / 2:.1f}" '
                    f'y="{y + bar_h + 11}" font-size="9" text-anchor="middle" '
                    f'opacity="0.8">{share:.0f}%</text>'
                )
            cursor += seg_w
        out.append(
            f'<text class="tx" x="{x0 + cursor + 8:.1f}" y="{y + bar_h / 2 + 4:.1f}" '
            f'font-size="10" opacity="0.85">{_esc(annotations[index])}</text>'
        )

    if deadline_cycles:
        out += _guideline(
            x0 + (deadline_cycles / axis_max) * plot_w, y_top, y_bot, deadline_label
        )

    # Legend: always present, because there is more than one series.
    ly = y_bot + 40
    lx = x0
    for slot_index, name in enumerate(legend):
        out.append(
            f'<rect class="s{slot_index}" x="{lx}" y="{ly - 8}" width="10" height="10" rx="2"/>'
        )
        out.append(
            f'<text class="tx" x="{lx + 15}" y="{ly + 1}" font-size="9.5" '
            f'opacity="0.85">{_esc(name)}</text>'
        )
        lx += 15 + int(len(name) * CHAR_W) + 18

    if footnote:
        out.append(
            f'<text class="tx" x="16" y="{height - 12}" font-size="9.5" '
            f'opacity="0.65">{_esc(footnote)}</text>'
        )
    out.append("</svg>")
    svg = "\n".join(out) + "\n"
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(svg)
    return svg
