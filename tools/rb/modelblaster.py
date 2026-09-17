"""ModelBlaster on RiskyBird hardware: codegen, capture analysis, and figures.

ModelBlaster (``backends/modelblaster``) takes a PyTorch
model through quantization, per-target kernel generation, a Zephyr build, and a
run on spike, Verilator or FireSim. What it has never had is a *board*: its
stage-5 runners are all simulators, so a model that has been measured has never
been measured on the thing that flies.

This module is the missing half, and it is deliberately only a half. The split
follows the rule in docs/repository-layout.md -- integration code lives at the
top level, backends are inputs:

    stages 1-3 (extract -> skeleton -> kernels)   ModelBlaster's, unchanged.
                                                  Driven here with STAGE_ONLY=1
                                                  so no simulator is involved.
    stage 4    (west build)                       rb's, because the image needs
                                                  RiskyBird's devicetree
                                                  overlays and Kconfig for the
                                                  shell it will run on.
    stage 5    (run)                              rb's: program, load, capture.
    analysis                                      here.

Everything below stage 3 is therefore ordinary rb machinery, and this file adds
only what is genuinely new: the console-marker contract, the per-op timeline,
and frames per second.

DEPENDENCY-FREE, for the same reason tools/rb/plotting.py is. An archived
capture must still render years later without resolving a Conda environment, and
the bench tier (macOS, no Chipyard) must be able to read a capture collected on
the Linux host. Nothing here imports numpy, matplotlib, or ModelBlaster itself.

THE MARKER CONTRACT IS SHARED, NOT COPIED LIGHTLY. The regexes below match what
modelblaster/harness/src/main.c prints and what
modelblaster/validation/runner_common.py parses. If that harness changes its
markers, this breaks, and it should: a silently mis-parsed profile is worse than
a failed parse. ``rb modelblaster report`` fails loudly on a capture with no
PROFILE block rather than reporting zero ops.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

from .boards import CHIPYARD, MODELBLASTER, ROOT
from .environments import zephyr_env


# Chipyard's RISC-V tools, which is where spike and libgemmini.so live.
#
# Codegen runs in the ZEPHYR environment and spike ships with the CHIPYARD one,
# so the per-op verification gate had no simulator to run and silently did not
# gate anything -- on the bench Mac, and on the Linux box too. Both tools are
# right here; nothing was missing but the PATH between two backends.
#
# Absent on the bench tier, which has no Chipyard. That is what the unverified
# refusal in harness_defines is for.
# RB_RISCV_TOOLS overrides the location, for a checkout whose own Chipyard is
# not initialised but which can reach one elsewhere -- a worktree, or a bench
# machine sharing a build host's filesystem.
CHIPYARD_RISCV_TOOLS = Path(
    os.environ.get("RB_RISCV_TOOLS", CHIPYARD / ".conda-env" / "riscv-tools")
)


def _with_chipyard_tools(environment: dict[str, str]) -> dict[str, str]:
    """Put Chipyard's spike and libgemmini on the codegen environment's path."""
    bindir = CHIPYARD_RISCV_TOOLS / "bin"
    libdir = CHIPYARD_RISCV_TOOLS / "lib"
    if not (bindir / "spike").is_file():
        return environment
    environment["PATH"] = os.pathsep.join(
        [str(bindir), environment.get("PATH", "")]
    ).rstrip(os.pathsep)
    environment["LD_LIBRARY_PATH"] = os.pathsep.join(
        [str(libdir), environment.get("LD_LIBRARY_PATH", "")]
    ).rstrip(os.pathsep)
    return environment



class ModelBlasterError(RuntimeError):
    """A ModelBlaster flow could not be carried out."""


# ---------------------------------------------------------------------------
# The console-marker contract.
#
# harness/src/main.c prints, in this order:
#
#   === MODELBLASTER_VERIFY === max_abs_err=<g> max_rel_err=<g> n=<int>
#   === MODELBLASTER_OUTPUT_BEGIN ===            (only when the output is small)
#   ...
#   === MODELBLASTER_OUTPUT_END ===
#   === MODELBLASTER_PROFILE_BEGIN ===
#   dispatch_id,name,op,shape,cycles
#   ...
#   === MODELBLASTER_PROFILE_END ===
#   === MODELBLASTER_WALL_CYCLES === <int>
#
# The multi-model harnesses tag each block as ``[<model>]``; the single-model
# harness this flow uses does not, so both spellings are accepted.
# ---------------------------------------------------------------------------

_PROFILE_BLOCK = re.compile(
    r"=== MODELBLASTER_PROFILE_BEGIN(?: \[([^\]]+)\])? ===\n(.*?)\n"
    r"=== MODELBLASTER_PROFILE_END(?: \[[^\]]*\])? ===",
    re.S,
)
_WALL = re.compile(r"=== MODELBLASTER_WALL_CYCLES(?: \[([^\]]+)\])? === (\d+)")
_VERIFY = re.compile(
    r"=== MODELBLASTER_VERIFY === max_abs_err=(\S+) max_rel_err=(\S+) n=(\d+)"
)


# ---------------------------------------------------------------------------
# Engine attribution.
#
# A per-op timeline is only worth rendering if each bar says which piece of
# silicon ran it -- "conv2d took 12 ms" is a number, "conv2d took 12 ms on
# Gemmini" is a finding. ModelBlaster already records the answer: the kernel
# picker writes kernel_picks.json next to the generated C, naming the
# implementation chosen per op. That file is the source of truth here, and the
# name-based table below is only the fallback for a capture whose GEN_DIR is
# gone -- an archived log, or one collected on another machine.
#
# Attribution is reported with its provenance ("picks" or "inferred") so a
# figure never implies more certainty than it has.
# ---------------------------------------------------------------------------

ENGINES = ("gemmini", "vector", "scalar")

ENGINE_COLORS = {
    "gemmini": "#1f4e79",   # deep blue
    "vector": "#3f9e6f",    # green
    "scalar": "#6b7280",    # slate
    "unknown": "#c88a2e",   # amber -- visible on purpose
}

ENGINE_LABELS = {
    "gemmini": "Gemmini (RoCC systolic)",
    "vector": "Saturn (RVV vector)",
    "scalar": "Rocket scalar",
    "unknown": "unattributed",
}

# Substring -> engine, applied to the chosen kernel's algorithm or path. Ordered
# most specific first: "gemmini_q31_conv2d" must not fall through to the plain
# "rvv" test on a path that happens to contain both.
_ENGINE_MARKERS = (
    ("gemmini", "gemmini"),
    ("vector", "rvv"),
    ("vector", "saturn"),
    ("vector", "opu"),
    ("scalar", "reference"),
    ("scalar", "scalar"),
)


def _engine_from_text(text: str) -> str | None:
    lowered = text.lower()
    for engine, marker in _ENGINE_MARKERS:
        if marker in lowered:
            return engine
    return None


def load_kernel_picks(gen_dir: Path | None) -> dict[str, str]:
    """Map op name -> engine, from ModelBlaster's own kernel_picks.json.

    Returns an empty mapping when the file is absent, which is not an error:
    the caller falls back to name inference and says so.
    """
    if gen_dir is None:
        return {}
    path = Path(gen_dir) / "kernel_picks.json"
    if not path.is_file():
        return {}
    try:
        data = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ModelBlasterError(f"{path} is not readable JSON: {exc}") from exc
    picks = data.get("picks")
    if not isinstance(picks, dict):
        return {}
    out: dict[str, str] = {}
    for op, pick in picks.items():
        if not isinstance(pick, dict):
            continue
        # algorithm and path both name the implementation; either identifies
        # the engine, and a curated kernel may carry only one of them.
        for key in ("algorithm", "path", "source"):
            value = pick.get(key)
            if isinstance(value, str) and (engine := _engine_from_text(value)):
                out[op] = engine
                break
    return out


@dataclass(frozen=True)
class Op:
    """One profiled kernel dispatch. `cycles` is an rdcycle delta: CPU clock."""

    dispatch_id: int
    name: str
    op: str
    shape: str
    cycles: int
    engine: str

    def ms(self, cpu_hz: float) -> float:
        return 1000.0 * self.cycles / cpu_hz


@dataclass(frozen=True)
class Run:
    """One model inference measured on one machine.

    TWO CLOCKS, AND THEY ARE NOT THE SAME ONE. The harness reports time in two
    units and a reader that divides both by the same number is wrong by three
    orders of magnitude:

      per-op `cycles`   rdcycle(), the mcycle CSR -> CPU clock (cpu_hz).
                        On the 35 MHz shell, 35,000,000 ticks per second.
      WALL_CYCLES       k_cycle_get_64(), which on this SoC is mtime ->
                        CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC (tick_hz). Chipyard
                        divides the fabric clock by 1000 to make mtime, so
                        35,000 ticks per second.

    modelblaster/pipeline/generate_skeleton.py says so at both call sites:
    rdcycle is "high-res", k_cycle_get_64 is "lower resolution ... but
    cross-hart-correct". The consequence here is that FPS comes from the wall
    figure over tick_hz, and the per-op bars from cycles over cpu_hz, and the
    two must never be added together.

    One mtime tick is 1/tick_hz -- 28.6 microseconds at 35 MHz -- so a frame
    time is quantized to that. It is immaterial for a 100 ms inference and would
    not be for a 1 ms one; `tick_quantum_ms` reports it so a reader can tell.
    """

    model: str
    ops: tuple[Op, ...]
    wall_ticks: int | None
    cpu_hz: float
    tick_hz: float
    clock_evidence: str
    attribution: str
    verify: tuple[float, float, int] | None = None

    @property
    def op_cycles(self) -> int:
        return sum(op.cycles for op in self.ops)

    @property
    def op_ms(self) -> float:
        """Time inside profiled kernels, from rdcycle deltas at the CPU clock."""
        return 1000.0 * self.op_cycles / self.cpu_hz

    @property
    def tick_quantum_ms(self) -> float:
        return 1000.0 / self.tick_hz

    @property
    def wall_ms(self) -> float:
        """The frame time.

        From WALL_CYCLES when the harness reported it, because that is the only
        figure that includes the gaps between kernels -- argument marshalling,
        buffer zeroing, misses on the next weight tile. Falling back to the
        kernel sum understates the frame and therefore OVERSTATES FPS, which is
        why `summary_lines` says so out loud when it happens.
        """
        if self.wall_ticks is None:
            return self.op_ms
        return 1000.0 * self.wall_ticks / self.tick_hz

    @property
    def fps(self) -> float:
        return 1000.0 / self.wall_ms if self.wall_ms > 0 else 0.0

    @property
    def unattributed_ms(self) -> float:
        """Frame time not inside any profiled kernel."""
        if self.wall_ticks is None:
            return 0.0
        return max(0.0, self.wall_ms - self.op_ms)

    def by_engine(self) -> list[tuple[str, int]]:
        totals: dict[str, int] = {}
        for op in self.ops:
            totals[op.engine] = totals.get(op.engine, 0) + op.cycles
        order = {name: i for i, name in enumerate((*ENGINES, "unknown"))}
        return sorted(totals.items(), key=lambda kv: order.get(kv[0], 99))


def parse_capture(
    text: str,
    *,
    model: str,
    cpu_hz: float,
    tick_hz: float,
    clock_evidence: str,
    picks: dict[str, str] | None = None,
) -> Run:
    """Turn a console capture into a Run.

    Raises rather than returning an empty Run when the PROFILE block is absent:
    a capture with no profile is a run that did not finish, and reporting 0 ops
    and infinite FPS would be worse than saying so.
    """
    block = _PROFILE_BLOCK.search(text)
    if block is None:
        raise ModelBlasterError(
            "no MODELBLASTER_PROFILE block in this capture.\n"
            "The harness prints it only after the model runs to completion, so "
            "the usual causes are a run that faulted, a console that dropped "
            "output, or a capture window that closed early. Check the capture "
            "for a Zephyr fatal-error banner first."
        )
    lines = [line for line in block.group(2).splitlines() if line.strip()]
    if not lines:
        raise ModelBlasterError("the MODELBLASTER_PROFILE block is empty")

    header = [cell.strip() for cell in lines[0].split(",")]
    required = {"dispatch_id", "name", "op", "shape", "cycles"}
    if not required.issubset(header):
        raise ModelBlasterError(
            f"unexpected profile header {header!r}; expected at least "
            f"{sorted(required)}. The harness changed its format, and this "
            "parser must be updated rather than guessed at."
        )

    picks = picks or {}
    attribution = "picks" if picks else "inferred"
    ops: list[Op] = []
    for line in lines[1:]:
        cells = [cell.strip() for cell in line.split(",")]
        if len(cells) != len(header):
            # A truncated row is a dropped console byte, not a parse bug. Say
            # which row, because the rest of the capture is still usable.
            raise ModelBlasterError(
                f"malformed profile row (got {len(cells)} fields, expected "
                f"{len(header)}): {line!r}. This is what a dropped console byte "
                "looks like; re-run the capture."
            )
        record = dict(zip(header, cells))
        op_name = record["op"]
        engine = picks.get(op_name) or _engine_from_text(op_name) or "unknown"
        ops.append(
            Op(
                dispatch_id=int(record["dispatch_id"]),
                name=record["name"],
                op=op_name,
                shape=record["shape"],
                cycles=int(record["cycles"]),
                engine=engine,
            )
        )

    wall = _WALL.search(text)
    verify_match = _VERIFY.search(text)
    verify = None
    if verify_match:
        verify = (
            float(verify_match.group(1)),
            float(verify_match.group(2)),
            int(verify_match.group(3)),
        )

    return Run(
        model=model,
        ops=tuple(ops),
        wall_ticks=int(wall.group(2)) if wall else None,
        cpu_hz=cpu_hz,
        tick_hz=tick_hz,
        clock_evidence=clock_evidence,
        attribution=attribution,
        verify=verify,
    )


def write_profile_csv(run: Run, path: Path) -> Path:
    """The per-op table, in ModelBlaster's own profile.csv columns plus engine."""
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = ["dispatch_id,name,op,shape,cycles,engine,ms"]
    for op in run.ops:
        rows.append(
            f"{op.dispatch_id},{op.name},{op.op},{op.shape},{op.cycles},"
            f"{op.engine},{op.ms(run.cpu_hz):.4f}"
        )
    path.write_text("\n".join(rows) + "\n")
    return path


def summary_lines(run: Run) -> list[str]:
    """The terminal report. Every number carries how it was obtained."""
    out = [
        f"model            {run.model}",
        f"cpu clock        {run.cpu_hz / 1e6:.6g} MHz  (rdcycle, per-op)",
        f"tick clock       {run.tick_hz / 1e3:.6g} kHz  (mtime, wall; "
        f"{run.tick_quantum_ms:.4f} ms quantum)",
        f"clock evidence   {run.clock_evidence}",
        f"ops profiled     {len(run.ops)}  (engine attribution: {run.attribution})",
    ]
    if run.wall_ticks is None:
        out.append(
            "wall ticks       NOT REPORTED -- falling back to the per-op sum, "
            "which excludes time between kernels, so FPS is an OVER-estimate"
        )
    else:
        out.append(f"wall ticks       {run.wall_ticks:,} mtime ticks")
    out.append(f"kernel time      {run.op_ms:.3f} ms ({run.op_cycles:,} cpu cycles)")
    out.append(f"inference        {run.wall_ms:.3f} ms")
    out.append(f"frames/s         {run.fps:.2f}")
    if run.wall_ticks is not None:
        share = 100.0 * run.unattributed_ms / run.wall_ms if run.wall_ms else 0.0
        out.append(
            f"outside kernels  {run.unattributed_ms:.3f} ms ({share:.1f}% of the "
            "frame: marshalling, buffer setup, misses between dispatches)"
        )
        # The two clocks are independent inputs, and getting one wrong scales
        # the frame time without producing anything that looks like an error.
        # These two shapes are what a mismatch looks like, and both are far
        # outside what a real run produces: a frame that is mostly not in any
        # kernel, or a kernel sum that exceeds the frame it sits inside.
        if share > 50.0:
            out.append(
                "  WARNING: over half the frame is outside every profiled "
                "kernel. That is usually a CLOCK MISMATCH rather than a real "
                "gap -- check that --cpu-mhz/--tick-hz (or the --config whose "
                "devicetree was read) describe the machine this capture came "
                "from. A spike capture reported against an FPGA shell's clocks "
                "lands here."
            )
        elif run.op_ms > run.wall_ms * 1.02:
            out.append(
                f"  WARNING: the kernel sum ({run.op_ms:.3f} ms) exceeds the "
                f"frame it sits inside ({run.wall_ms:.3f} ms), which is "
                "impossible. The CPU and mtime rates do not describe the same "
                "machine."
            )
    if run.verify is not None:
        abs_err, rel_err, n = run.verify
        out.append(
            f"verify           max_abs_err={abs_err:g} max_rel_err={rel_err:g} n={n}"
        )
    else:
        out.append("verify           NOT REPORTED -- outputs were not checked")
    out.append("")
    total = run.op_cycles or 1
    out.append(f"{'engine':<26}{'cycles':>14}{'ms':>10}{'% of kernels':>14}")
    out.append("-" * 64)
    for engine, cycles in run.by_engine():
        out.append(
            f"{ENGINE_LABELS.get(engine, engine):<26}{cycles:>14,}"
            f"{1000.0 * cycles / run.cpu_hz:>10.2f}{100.0 * cycles / total:>13.1f}%"
        )
    return out


# ---------------------------------------------------------------------------
# The Gantt figure.
#
# One hart runs these kernels in sequence, so the timeline is a waterfall: each
# dispatch starts where the previous ended. Laying the bars out on one lane per
# engine rather than one lane per op is what makes the picture worth having --
# the eye reads "how much of the frame was Gemmini" directly, which is the
# question a co-design change is trying to move.
#
# The gap between the kernel sum and WALL_CYCLES is drawn as its own lane rather
# than dropped. On the KU040 flight3 profile that gap was a fifth of the frame,
# and a Gantt that silently omits it invites optimizing the wrong thing.
# ---------------------------------------------------------------------------

_AXIS = "#6b7280"
_TEXT = "#111827"
_GRID = "#d1d5db"


def _esc(text: str) -> str:
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def _ticks(maximum: float, count: int = 6) -> list[float]:
    if maximum <= 0:
        return [0.0]
    raw = maximum / count
    magnitude = 10.0 ** (len(str(int(raw))) - 1) if raw >= 1 else 0.001
    step = magnitude
    for factor in (1, 2, 2.5, 5, 10):
        step = magnitude * factor
        if step >= raw:
            break
    ticks: list[float] = []
    value = 0.0
    while value <= maximum + step * 0.001:
        ticks.append(value)
        value += step
    if ticks[-1] < maximum:
        ticks.append(ticks[-1] + step)
    return ticks


def gantt_svg(run: Run, path: Path | None = None, *, subtitle: str = "") -> str:
    """Render the per-op timeline as a standalone SVG."""
    lanes = [engine for engine, _ in run.by_engine()]
    if run.wall_ticks is not None and run.unattributed_ms > 0:
        lanes.append("gap")

    lane_h = 34
    top = 96
    left = 190
    right = 40
    width = 1180
    height = top + lane_h * len(lanes) + 96
    span_ms = run.wall_ms if run.wall_ms > 0 else 1.0
    ticks = _ticks(span_ms)
    plot_w = width - left - right
    scale = plot_w / ticks[-1]

    title = f"{run.model}: per-op timeline, one inference"
    caption = subtitle or (
        f"{run.wall_ms:.2f} ms/frame = {run.fps:.1f} FPS at "
        f"{run.cpu_hz / 1e6:.6g} MHz"
    )

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
        f'width="{width}" height="{height}" role="img" aria-labelledby="t d" '
        'font-family="Inter, Helvetica, Arial, sans-serif">',
        f'<title id="t">{_esc(title)}</title>',
        f'<desc id="d">{_esc(caption)}</desc>',
        "<style>"
        ".bg{fill:#ffffff} .tx{fill:#111827} .ax{stroke:#6b7280} .gr{stroke:#d1d5db}"
        "@media (prefers-color-scheme: dark){"
        ".bg{fill:#0b0f19} .tx{fill:#e5e7eb} .ax{stroke:#9ca3af} .gr{stroke:#374151}}"
        ':root[data-theme="dark"] .bg{fill:#0b0f19}'
        ':root[data-theme="dark"] .tx{fill:#e5e7eb}'
        ':root[data-theme="dark"] .gr{stroke:#374151}'
        ':root[data-theme="light"] .bg{fill:#ffffff}'
        ':root[data-theme="light"] .tx{fill:#111827}'
        "</style>",
        f'<rect class="bg" x="0" y="0" width="{width}" height="{height}"/>',
        f'<text class="tx" x="24" y="34" font-size="17" font-weight="600">{_esc(title)}</text>',
        f'<text class="tx" x="24" y="56" font-size="12" opacity="0.75">{_esc(caption)}</text>',
        f'<text class="tx" x="24" y="74" font-size="11" opacity="0.6">'
        f'{_esc("engine attribution: " + run.attribution + "  |  clock: " + run.clock_evidence)}</text>',
    ]

    # Gridlines and the millisecond axis.
    for tick in ticks:
        x = left + tick * scale
        svg.append(
            f'<line class="gr" x1="{x:.1f}" y1="{top - 10}" x2="{x:.1f}" '
            f'y2="{top + lane_h * len(lanes)}" stroke-width="1"/>'
        )
        svg.append(
            f'<text class="tx" x="{x:.1f}" y="{top + lane_h * len(lanes) + 18}" '
            f'font-size="10" text-anchor="middle" opacity="0.7">{tick:g}</text>'
        )
    svg.append(
        f'<text class="tx" x="{left + plot_w / 2:.1f}" '
        f'y="{top + lane_h * len(lanes) + 36}" font-size="11" '
        'text-anchor="middle" opacity="0.7">milliseconds</text>'
    )

    lane_y = {engine: top + i * lane_h for i, engine in enumerate(lanes)}
    for engine in lanes:
        label = "between kernels" if engine == "gap" else ENGINE_LABELS.get(engine, engine)
        svg.append(
            f'<text class="tx" x="{left - 10}" y="{lane_y[engine] + 20}" '
            f'font-size="11" text-anchor="end">{_esc(label)}</text>'
        )

    # The waterfall. Ops are sequential on one hart, so each starts where the
    # previous ended; the lane only says which engine ran it.
    cursor_ms = 0.0
    for op in run.ops:
        duration = op.ms(run.cpu_hz)
        x = left + cursor_ms * scale
        w = max(1.0, duration * scale)
        y = lane_y[op.engine]
        colour = ENGINE_COLORS.get(op.engine, ENGINE_COLORS["unknown"])
        svg.append(
            f'<rect x="{x:.2f}" y="{y + 6}" width="{w:.2f}" height="{lane_h - 14}" '
            f'fill="{colour}" rx="2"><title>{_esc(op.name)} ({_esc(op.op)}) '
            f'{_esc(op.shape)} — {op.cycles:,} cycles, {duration:.3f} ms</title></rect>'
        )
        # Only label a bar wide enough to hold text; the rest are readable from
        # the CSV emitted beside this figure.
        if w > 46:
            svg.append(
                f'<text x="{x + 4:.2f}" y="{y + lane_h - 13}" font-size="9.5" '
                f'fill="#ffffff">{_esc(op.name[:14])}</text>'
            )
        cursor_ms += duration

    if "gap" in lane_y:
        x = left + cursor_ms * scale
        w = max(1.0, run.unattributed_ms * scale)
        svg.append(
            f'<rect x="{x:.2f}" y="{lane_y["gap"] + 6}" width="{w:.2f}" '
            f'height="{lane_h - 14}" fill="{ENGINE_COLORS["unknown"]}" rx="2" '
            f'opacity="0.55"><title>time outside profiled kernels — '
            f'{run.unattributed_ms:.3f} ms</title></rect>'
        )

    svg.append("</svg>")
    out = "\n".join(svg)
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(out)
    return out


# ---------------------------------------------------------------------------
# Stage 1-3: ModelBlaster's own codegen, driven with the simulator left out.
# ---------------------------------------------------------------------------


def generated_dir(model: str, quant: str, target: str) -> Path:
    """Where ModelBlaster's codegen lands for this (model, quant, target).

    Mirrors examples/_run_lib.sh: IR_DIR is <example>/<quant>/generated and
    GEN_DIR is IR_DIR/<TARGET>. Stated here rather than discovered, so a layout
    change upstream fails loudly in `generate` instead of silently building the
    previous model's sources.
    """
    return MODELBLASTER / "examples" / model / quant / "generated" / target


def generate(
    model: str,
    *,
    quant: str,
    target: str,
    curated: bool = True,
    env: dict[str, str] | None = None,
) -> Path:
    """Run ModelBlaster stages 1-3 and return the generated directory.

    STAGE_ONLY=1 is what keeps this honest: it stops the flow after codegen, so
    nothing here ever runs spike. The image that goes on the board is built by
    rb, from RiskyBird's overlays, against exactly these sources.
    """
    run_sh = MODELBLASTER / "examples" / model / "run.sh"
    if not run_sh.is_file():
        available = sorted(
            p.name for p in (MODELBLASTER / "examples").iterdir()
            if (p / "run.sh").is_file()
        ) if (MODELBLASTER / "examples").is_dir() else []
        raise ModelBlasterError(
            f"no ModelBlaster example named {model!r}"
            + (f"; available: {', '.join(available)}" if available else
               f"; {MODELBLASTER} is empty -- initialise the backend "
               "(git submodule update --init backends/modelblaster)")
        )

    # The zephyr conda environment, not the caller's: codegen needs torch,
    # numpy and ModelBlaster's own package, none of which are on a plain PATH.
    environment = dict(env or zephyr_env())
    environment.update(
        {
            "STAGE_ONLY": "1",
            "QUANT": quant,
            "TARGET": target,
            "BACKEND": "reference",
            # examples/_run_lib.sh does `cd "${REPO_ROOT}"` into modelblaster/
            # and then runs `python -m modelblaster.pipeline.*`, which only
            # resolves if the package's PARENT is importable. Upstream gets that
            # from `pip install -e`; this backend's conda env has no such
            # install, and doing one here would mutate a shared environment to
            # make one command work. Putting the parent on PYTHONPATH is the
            # same resolution with no side effect -- modelblaster/ is a
            # namespace package, so it imports from the checkout directly.
            "PYTHONPATH": os.pathsep.join(
                [str(MODELBLASTER.parent), environment.get("PYTHONPATH", "")]
            ).rstrip(os.pathsep),
        }
    )
    if curated:
        environment["GLOBAL_CURATED_DIR"] = str(MODELBLASTER / "kernels")
    _with_chipyard_tools(environment)

    result = subprocess.run(
        ["bash", str(run_sh)],
        cwd=str(MODELBLASTER.parent),
        env=environment,
        text=True,
        capture_output=True,
    )
    if result.returncode != 0:
        combined = result.stdout + result.stderr
        # ModelBlaster's own dependencies -- torch above all -- are declared in
        # its pyproject.toml, not in the Zephyr backend's environment spec. So
        # `rb env setup zephyr` does not install them, and on a machine where
        # nobody has installed them by hand this is the first place it shows,
        # as a ModuleNotFoundError buried in a codegen traceback. Say what to
        # do instead of making someone read the stack.
        hint = ""
        if "ModuleNotFoundError" in combined:
            missing = re.findall(r"No module named '([^']+)'", combined)
            hint = (
                "\n\nThis is a missing codegen dependency"
                + (f" ({', '.join(sorted(set(missing)))})" if missing else "")
                + ". ModelBlaster declares its own in "
                "backends/modelblaster/pyproject.toml -- torch, numpy, pillow, "
                "ultralytics, pyyaml, requests -- and the Zephyr backend's "
                "environment does not carry them. Install them into it:\n"
                "    rb env shell zephyr\n"
                "    pip install -e backends/modelblaster\n"
                "Only the models tier needs this; nothing else in the "
                "repository imports torch."
            )
        raise ModelBlasterError(
            f"ModelBlaster codegen failed for {model} "
            f"(quant={quant}, target={target}):\n"
            f"{result.stdout[-4000:]}\n{result.stderr[-4000:]}{hint}"
        )
    out = generated_dir(model, quant, target)
    if not out.is_dir():
        raise ModelBlasterError(
            f"codegen reported success but {out} does not exist; "
            "the example's directory layout has changed"
        )
    _record_verification(out, environment)
    return out


# Whether a codegen run's kernel selection was verify-gated, recorded beside the
# sources it produced.
#
# ModelBlaster selects a curated kernel only if it passes a per-op check against
# the scalar reference (examples/_run_lib.sh: "Defaulting to curated is safe
# because selection is verify-gated"). That check runs the kernel, which for a
# RISC-V accelerator target means spike. No spike, no gate -- and a curated
# kernel is then taken on its own declared accuracy class, which is metadata the
# kernel author wrote, not a measurement.
#
# This is not hypothetical. A bench run with the gate off produced DroNet
# outputs wrong by up to 13 LSB while every checkpoint passed, and the same run
# under MAX_ACCURACY_CLASS=bit_exact was still wrong by 3 -- because restricting
# by declared class cannot exclude a kernel whose declaration is what is wrong.
CODEGEN_MARKER = ".rb-codegen.json"


def _record_verification(gen_dir: Path, environment: dict[str, str]) -> None:
    spike = shutil.which("spike", path=environment.get("PATH", ""))
    disabled = [
        name for name in sorted(environment)
        if ("VERIF" in name.upper() or "NO_VERIFY" in name.upper())
        and name.startswith(("MB_", "MODELBLASTER_"))
        and environment[name] not in ("", "0")
    ]
    (gen_dir / CODEGEN_MARKER).write_text(json.dumps({
        "verified": bool(spike) and not disabled,
        "spike": spike,
        "verification_disabled_by": disabled,
    }, indent=2) + "\n")


def verification_status(gen_dir: Path) -> dict | None:
    """What `generate` recorded about this directory's kernel selection."""
    path = Path(gen_dir) / CODEGEN_MARKER
    if not path.is_file():
        return None
    try:
        data = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) else None


def per_config_header_dir(config: str) -> Path:
    """Where the Gemmini header elaborated for `config` lives, if it is captured."""
    return MODELBLASTER / "cores" / "gemmini" / "include" / "per_config" / config


# The ModelBlaster harness's defaults, for the rb commands that build a workload
# without being told about models. `rb debug`, `rb run`, `rb capture` and
# `rb build` all rebuild before they do their real job, and they have no
# --model/--quant/--target of their own -- so without this the whole deploy path
# fails at CMake with "MODEL_DIR not set", and only `rb modelblaster build`
# works. These are the same defaults that command takes.
DEFAULT_QUANT = "int8"
DEFAULT_TARGET = "gemmini_q31"


# What a built image was made from. rb debug / run / build rebuild before they do
# their real job and have no --model of their own, so without a record they fall
# back to the defaults below and silently replace a build made with different
# ones -- a `--target scalar` image becomes a gemmini_q31 image on the next
# `rb debug`, with nothing in the output saying so. That cost a real comparison
# run on the bench.
BUILD_MARKER = ".rb-modelblaster.json"


def read_build_marker(build_dir: Path) -> dict | None:
    """What the existing build in `build_dir` was generated from, if known."""
    path = Path(build_dir) / BUILD_MARKER
    if not path.is_file():
        return None
    try:
        data = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) and "target" in data else None


def write_build_marker(
    build_dir: Path, *, model: str, quant: str, target: str, gen_dir: Path
) -> None:
    path = Path(build_dir) / BUILD_MARKER
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(
        {"model": model, "quant": quant, "target": target,
         "gen_dir": str(gen_dir)},
        indent=2,
    ) + "\n")


def harness_defines(
    model: str,
    *,
    quant: str = DEFAULT_QUANT,
    target: str = DEFAULT_TARGET,
    gemmini_config: str | None = None,
    gen_dir: Path | None = None,
) -> tuple[tuple[str, str], ...]:
    """The three -D definitions the ModelBlaster harness cannot build without.

    MODEL_DIR is where codegen wrote the model; MODELBLASTER_BACKEND picks the
    harness's Kconfig overlay; MODELBLASTER_KERNEL_CFLAGS is what makes
    kernels.c a Gemmini kernel rather than a scalar one. The last is resolved
    from ModelBlaster's own table rather than restated, and carries the
    per-config Gemmini header selection with it.
    """
    resolved = Path(gen_dir) if gen_dir else generated_dir(model, quant, target)
    if not resolved.is_dir():
        raise ModelBlasterError(
            f"no generated sources at {resolved}.\n"
            f"Run: rb modelblaster generate --model {model} --quant {quant} "
            f"--target {target}"
        )

    # Refuse, loudly and with a named override, rather than building an image
    # whose kernels were never checked against the reference. The override is
    # deliberately ugly to type and greppable afterwards, because the failure it
    # guards against is a run that passes every checkpoint and returns wrong
    # numbers -- which is exactly what happened on the bench.
    status = verification_status(resolved)
    if status is not None and not status.get("verified"):
        if os.environ.get("RB_ALLOW_UNVERIFIED_KERNELS") != "1":
            why = (
                "verification was disabled by "
                + ", ".join(status["verification_disabled_by"])
                if status.get("verification_disabled_by")
                else "spike is not on PATH in the Zephyr environment, so the "
                "per-op check could not run"
            )
            raise ModelBlasterError(
                f"the kernels in {resolved} were selected WITHOUT verification "
                f"({why}).\n"
                "\n"
                "ModelBlaster only takes a curated kernel if it passes a per-op "
                "check against the scalar reference. Without that gate a kernel "
                "is trusted on its declared accuracy class, which is something "
                "its author wrote rather than something anyone measured. A bench "
                "run in this state produced DroNet outputs wrong by 13 LSB with "
                "every checkpoint green, and 3 LSB even under "
                "MAX_ACCURACY_CLASS=bit_exact.\n"
                "\n"
                "Fix it (preferred): make spike reachable in the Zephyr "
                "environment and re-run `rb modelblaster generate`.\n"
                "Or use a scalar target, whose reference kernels need no gate:\n"
                f"    rb modelblaster generate --model {model} --quant {quant} "
                "--target scalar\n"
                "\n"
                "To proceed anyway, knowing the numbers may be wrong:\n"
                "    RB_ALLOW_UNVERIFIED_KERNELS=1 rb ..."
            )
        print(
            "rb: WARNING building from UNVERIFIED kernels "
            "(RB_ALLOW_UNVERIFIED_KERNELS=1). Results may be numerically wrong."
        )
    return (
        ("MODEL_DIR", str(resolved)),
        ("MODELBLASTER_BACKEND", target),
        ("MODELBLASTER_KERNEL_CFLAGS",
         ";".join(kernel_cflags(target, gemmini_config=gemmini_config))),
    )


def kernel_cflags(target: str, *, gemmini_config: str | None = None) -> list[str]:
    """The per-backend flags ModelBlaster applies to kernels.c, from its own table.

    Asked of modelblaster.pipeline.backends rather than restated here. These
    flags are what turn a generated kernel into a Gemmini or RVV one -- getting
    them from a copy that has drifted would produce an image that builds, runs,
    and quietly uses the scalar path while the report claims otherwise.

    `gemmini_config` selects cores/gemmini/include/per_config/<name>/, which is
    how a build gets the gemmini_params.h that MATCHES THE ELABORATED HARDWARE
    rather than whichever one is checked in. Pass the Chipyard config name. The
    mesh width, the scratchpad geometry and whether a mvin scale unit exists at
    all live in that header, and a mismatch is not a build error -- it is a
    binary that drives Gemmini with the wrong parameters.

    Every include path that comes back is checked for existence, because the
    failure mode this guards against is silent: GCC ignores a -isystem naming a
    directory that is not there, so a path that stops resolving takes the header
    selection with it and says nothing.
    """
    environment = zephyr_env()
    if gemmini_config is not None:
        environment["MODELBLASTER_GEMMINI_CONFIG"] = gemmini_config
    environment["PYTHONPATH"] = os.pathsep.join(
        [str(MODELBLASTER.parent), environment.get("PYTHONPATH", "")]
    ).rstrip(os.pathsep)
    result = subprocess.run(
        ["python", "-c",
         "from modelblaster.pipeline.backends import get;"
         f"print(';'.join(get({target!r}).resolved_kernel_cflags({str(MODELBLASTER)!r})))"],
        cwd=str(MODELBLASTER.parent),
        env=environment,
        text=True,
        capture_output=True,
    )
    if result.returncode != 0:
        raise ModelBlasterError(
            f"could not resolve kernel cflags for target {target!r}:\n"
            f"{result.stderr[-2000:]}"
        )
    flags = [flag for flag in result.stdout.strip().split(";") if flag]

    def _include_path(flag: str) -> str | None:
        for prefix in ("-isystem", "-I"):
            if flag.startswith(prefix) and len(flag) > len(prefix):
                return flag[len(prefix):]
        return None

    missing = [
        flag for flag in flags
        if (path := _include_path(flag)) is not None and not Path(path).is_dir()
    ]
    if missing:
        raise ModelBlasterError(
            f"ModelBlaster's flag table for target {target!r} names include "
            f"directories that do not exist:\n  " + "\n  ".join(missing) + "\n\n"
            "GCC ignores these without a word, so the build would succeed with "
            "the wrong headers rather than fail. Either the flag table's "
            "<repo_root> convention moved, or "
            + (f"per_config/{gemmini_config}/ has not been captured "
               "(elaborate that config and copy its generated "
               "gemmini_params_q31.h there)"
               if gemmini_config else "a vendored core is missing")
            + "."
        )
    return flags
