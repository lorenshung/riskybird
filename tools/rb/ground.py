"""RiskyBird ground station: the flight-test side of `rb`, next to build/debug/console.

`rb ground` wraps the bench tools that a flight session actually runs, so a whole
session -- build, load, fly, publish the logs -- is one tool on one machine:

    rb ground ready [--no-join] [--auto-rejoin]   caffeinate, join once, panel, live GO / NO-GO
    rb ground up | down | status                  the drone SoftAP (macOS script; Linux: see below)
    rb ground gonogo [--watch]                    one line from the running monitor

The tools themselves stay in tools/bench/ as plain scripts. They are stdlib-only
and runnable without the rb environment, which is what a laptop joined to the
drone's SoftAP, with no route to anything else, needs. This module only finds them
and passes flags through.

Nothing here sends to the drone's command port. The only uplink traffic is the
Wi-Fi script's own PING during `up`. ESTOP / RESET / ... stay on the panel's buttons.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

from .boards import ROOT


class GroundError(Exception):
    """An operator-facing ground-station problem (missing tool, no monitor running)."""


BENCH = ROOT / "tools" / "bench"
READY_SH = BENCH / "riskybird-ready.sh"
WIFI_MACOS = BENCH / "riskybird-wifi-macos.sh"
MONITOR = BENCH / "riskybird-flightready.py"
WIFI_LINUX = BENCH / "groundstation" / "riskybird_wifi_setup.sh"
LOG_DIR = ROOT / "results" / "riskybird-logs"
STATUS = LOG_DIR / "ready-status.json"

# The monitor rewrites STATUS once a second. Older than this, nothing is running.
STATUS_STALE_S = 5.0


def _require(path: Path) -> Path:
    if not path.is_file():
        raise GroundError(f"missing {path.relative_to(ROOT)} -- is the checkout complete?")
    return path


def ready(no_join: bool = False, auto_rejoin: bool = False) -> int:
    """Run riskybird-ready.sh in the foreground (Ctrl-C stops the panel and monitor with it)."""
    cmd = [str(_require(READY_SH))]
    if no_join:
        cmd.append("--no-join")
    if auto_rejoin:
        cmd.append("--auto-rejoin")
    return subprocess.call(cmd, cwd=ROOT)


def wifi(action: str, ssid: str | None = None) -> int:
    """up / down / status / verify for the drone SoftAP.

    macOS: riskybird-wifi-macos.sh, which refuses `up` unless a non-Wi-Fi internet path exists
    (iPhone USB, Ethernet). Linux has a different model -- a second Wi-Fi adapter with a
    NetworkManager profile that auto-joins -- so there is no up/down to wrap. Point at it
    instead of pretending.
    """
    if sys.platform == "darwin":
        cmd = [str(_require(WIFI_MACOS)), action]
        if ssid and action == "up":
            cmd += ["--ssid", ssid]
        return subprocess.call(cmd, cwd=ROOT)
    raise GroundError(
        "`rb ground up/down/status` wraps the macOS script. On Linux, set up the drone SoftAP once "
        f"on a second Wi-Fi adapter with {WIFI_LINUX.relative_to(ROOT)} [ssid] [iface]; "
        "NetworkManager then joins it automatically."
    )


def read_status(path: Path = STATUS) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise GroundError("no monitor status yet -- start one with `rb ground ready`") from None
    except (OSError, ValueError) as exc:
        raise GroundError(f"unreadable monitor status {path}: {exc}") from None


def format_status(s: dict, age_s: float | None = None) -> str:
    """One line: verdict, link, the fields the pre-flight checklist looks at, then the reasons."""
    def g(key: str, fmt: str = "{}") -> str:
        v = s.get(key)
        if v is None:
            return "-"
        if isinstance(v, float) and v.is_integer() and fmt == "{}":
            v = int(v)
        return fmt.format(v)

    line = (f"{g('t')[11:] or '-'} {g('verdict'):5} link={g('link')} "
            f"fc={g('fc_hz')}Hz rb={g('rb_hz')}Hz ml={g('ml')} fl=[{g('fl')}] kill={g('kill')} "
            f"cl={g('cl')} tilt={g('tilt')}/{g('tilt_acc')} tofv={g('tofv')} tofh={g('tofh')} "
            f"z={g('z')} st={g('st')} gap={g('gap_max')} cuts={g('cuts')} vbat={g('vbat')}")
    if age_s is not None and age_s > STATUS_STALE_S:
        line += f"  (STALE: {age_s:.0f} s old -- is the monitor running?)"
    reasons = s.get("reasons") or []
    warnings = s.get("warnings") or []
    if reasons:
        line += "\n  NO-GO: " + "; ".join(reasons)
    if warnings:
        line += "\n  warn:  " + "; ".join(warnings)
    return line


def gonogo(watch: bool = False, path: Path = STATUS) -> int:
    """Print the monitor's current verdict. Exit 0 on GO, 1 otherwise, so scripts can gate on it."""
    while True:
        s = read_status(path)
        age = time.time() - path.stat().st_mtime
        print(format_status(s, age), flush=True)
        if not watch:
            return 0 if s.get("verdict") == "GO" and age <= STATUS_STALE_S else 1
        time.sleep(1.0)


def monitor(extra: list[str]) -> int:
    """Run the monitor alone (no panel, no Wi-Fi), e.g. on a machine that already has the link."""
    return subprocess.call([sys.executable, "-u", str(_require(MONITOR)),
                            "--log-dir", str(LOG_DIR), *extra], cwd=ROOT)
