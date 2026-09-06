"""Achieved clock frequency, read out of Vivado's post-route reports.

Area at an unstated frequency is not a result. The 50 MHz every KU040 config
carries is *declared* -- it is what the shell asks the PLL for, and says nothing
about whether the design closes at it, let alone how much faster it could go.

`rb area` runs synthesis only, so it never produces timing. But a full
`rb build` does, and leaves `report_timing_summary` and `report_clocks` behind
in the config's build directory. This module reads them.

It is deliberately kept out of the area record. A timing report found on disk
belongs to whichever implementation run last wrote it, which is not necessarily
the synthesis that produced the record sitting next to it -- so it carries its
own timestamp and design state, and the caller is told when the two disagree
rather than shown one number implying both.

The number that matters is worst negative slack on the clock that carries the
design, not the worst across all clocks. A JTAG clock at 100 ns will happily
report tens of nanoseconds of slack while the SoC clock is the one about to
fail, so picking the wrong row overstates F_max by a wide margin.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


# "clk_out1_harnessSysPLLNode    20.000    {0.000 10.000}   P,G,A   {...}"
_CLOCK_ROW = re.compile(
    r"^(?P<name>[A-Za-z_][\w/\[\]\.]*)\s+(?P<period>\d+\.\d+)\s+\{", re.M
)
# "  clk_out1_harnessSysPLLNode   5.619   0.000   0   54771   ..."
_SLACK_ROW = re.compile(
    r"^\s*(?P<name>[A-Za-z_][\w/\[\]\.]*)\s+(?P<wns>-?\d+\.\d+)\s+(?P<tns>-?\d+\.\d+)\s+"
    r"(?P<failing>\d+)\s+(?P<total>\d+)\b",
    re.M,
)
_DESIGN_STATE = re.compile(r"^\|\s*Design State\s*:\s*(?P<state>.+?)\s*$", re.M)
_REPORT_DATE = re.compile(r"^\|\s*Date\s*:\s*(?P<date>.+?)\s*$", re.M)


class TimingError(RuntimeError):
    """Timing could not be read from the reports on disk."""


@dataclass(frozen=True)
class ClockTiming:
    """One clock domain's constraint and how much room it had left."""

    name: str
    period_ns: float
    wns_ns: float
    endpoints: int
    failing: int

    @property
    def critical_path_ns(self) -> float:
        return self.period_ns - self.wns_ns

    @property
    def declared_mhz(self) -> float:
        return 1000.0 / self.period_ns

    @property
    def achieved_mhz(self) -> float | None:
        """F_max implied by the critical path, or None if it is non-positive."""
        path = self.critical_path_ns
        return 1000.0 / path if path > 0 else None

    @property
    def headroom(self) -> float | None:
        if self.achieved_mhz is None:
            return None
        return self.achieved_mhz / self.declared_mhz

    def as_dict(self) -> dict[str, Any]:
        return {
            "clock": self.name,
            "period_ns": self.period_ns,
            "declared_mhz": round(self.declared_mhz, 3),
            "wns_ns": self.wns_ns,
            "critical_path_ns": round(self.critical_path_ns, 3),
            "achieved_mhz": round(self.achieved_mhz, 3) if self.achieved_mhz else None,
            "endpoints": self.endpoints,
            "failing_endpoints": self.failing,
            "met": self.failing == 0 and self.wns_ns >= 0,
        }


@dataclass(frozen=True)
class TimingReport:
    """What a config's post-route reports say, and when they said it."""

    config: str
    design_state: str
    reported_at: str
    clocks: tuple[ClockTiming, ...]
    sources: dict[str, str]
    synthesized_at: str | None = None

    @property
    def design_clock(self) -> ClockTiming | None:
        """The clock carrying the design, by endpoint count.

        Not the worst clock and not the first: a JTAG domain with 560 endpoints
        and 46 ns of slack is not the one that decides F_max.
        """
        candidates = [c for c in self.clocks if c.endpoints > 0]
        return max(candidates, key=lambda c: c.endpoints) if candidates else None

    @property
    def met(self) -> bool:
        return all(c.failing == 0 and c.wns_ns >= 0 for c in self.clocks)

    @property
    def stale_against_synthesis(self) -> bool | None:
        """Whether these reports predate the synthesis recorded beside them."""
        if not self.synthesized_at:
            return None
        try:
            synth = datetime.fromisoformat(self.synthesized_at)
            report = _parse_vivado_date(self.reported_at)
        except (ValueError, TypeError):
            return None
        if report is None:
            return None
        if synth.tzinfo and not report.tzinfo:
            report = report.replace(tzinfo=synth.tzinfo)
        return report < synth

    def as_dict(self) -> dict[str, Any]:
        design = self.design_clock
        return {
            "evidence": "post-route",
            "config": self.config,
            "design_state": self.design_state,
            "reported_at": self.reported_at,
            "timing_met": self.met,
            "design_clock": design.as_dict() if design else None,
            "clocks": [c.as_dict() for c in self.clocks],
            "stale_against_synthesis": self.stale_against_synthesis,
            "sources": self.sources,
        }

    def summary(self) -> str:
        design = self.design_clock
        if design is None:
            return f"{self.config}: no clock with endpoints in the report"
        achieved = (
            f"{design.achieved_mhz:.1f} MHz achieved" if design.achieved_mhz
            else "no positive critical path"
        )
        verdict = "met" if self.met else "VIOLATED"
        head = f" ({design.headroom:.2f}x)" if design.headroom else ""
        return (
            f"{design.declared_mhz:.0f} MHz declared -> {achieved}{head}, "
            f"WNS {design.wns_ns:+.3f} ns on {design.name}, timing {verdict}"
        )


def _parse_vivado_date(text: str) -> datetime | None:
    for fmt in ("%a %b %d %H:%M:%S %Y", "%Y-%m-%d %H:%M:%S"):
        try:
            return datetime.strptime(text.strip(), fmt)
        except ValueError:
            continue
    return None


def parse_clocks(text: str) -> dict[str, float]:
    """Clock name -> constrained period in nanoseconds, from report_clocks."""
    periods: dict[str, float] = {}
    for match in _CLOCK_ROW.finditer(text):
        name = match.group("name")
        if name in ("Clock", "Generated", "Attributes"):
            continue
        periods.setdefault(name, float(match.group("period")))
    return periods


def parse_slack(text: str) -> dict[str, tuple[float, int, int]]:
    """Clock name -> (WNS ns, failing endpoints, total endpoints)."""
    found: dict[str, tuple[float, int, int]] = {}
    for match in _SLACK_ROW.finditer(text):
        name = match.group("name")
        if name.lower() in ("clock", "from", "path", "setup", "hold"):
            continue
        found.setdefault(
            name,
            (float(match.group("wns")), int(match.group("failing")), int(match.group("total"))),
        )
    return found


def read_reports(report_dir: Path, config: str, synthesized_at: str | None = None) -> TimingReport:
    """Assemble a timing report from a config's Vivado report directory."""
    timing_path = report_dir / "timing.txt"
    clocks_path = report_dir / "clocks.txt"
    if not timing_path.is_file():
        raise TimingError(
            f"no timing.txt under {report_dir}. rb area runs synthesis only; a "
            "full implementation (rb build) is what writes timing reports."
        )
    timing_text = timing_path.read_text(errors="replace")
    clocks_text = clocks_path.read_text(errors="replace") if clocks_path.is_file() else ""

    periods = parse_clocks(clocks_text)
    slack = parse_slack(timing_text)
    clocks = [
        ClockTiming(
            name=name,
            period_ns=periods[name],
            wns_ns=wns,
            endpoints=total,
            failing=failing,
        )
        for name, (wns, failing, total) in slack.items()
        if name in periods
    ]
    if not clocks:
        raise TimingError(
            f"no clock in {timing_path} could be matched to a period in "
            f"{clocks_path}. The Vivado report format may have changed."
        )

    state = _DESIGN_STATE.search(timing_text)
    date = _REPORT_DATE.search(timing_text)
    return TimingReport(
        config=config,
        design_state=state.group("state") if state else "unknown",
        reported_at=date.group("date") if date else "unknown",
        clocks=tuple(sorted(clocks, key=lambda c: -c.endpoints)),
        sources={"timing": str(timing_path), "clocks": str(clocks_path)},
        synthesized_at=synthesized_at,
    )


def for_board(board: Any, synthesized_at: str | None = None) -> TimingReport:
    """Read whatever post-route reports exist for a board's selected config."""
    return read_reports(
        board.generated_dir / "obj" / "report", board.config, synthesized_at
    )
