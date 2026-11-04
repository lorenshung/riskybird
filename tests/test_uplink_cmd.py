"""The uplink command contract: ground station, ESP bridge, flight controller.

Three independent descriptions of the same small vocabulary, none of which can
see the others:

  * the ground stations' POST /cmd whitelists, which decide what a browser is
    allowed to send at all,
  * workloads/esp_bridge, which forwards the line verbatim over uart1 and
    answers with an ACK it builds ITSELF,
  * rose_flight_controller's uart_cmd_dispatch(), which is the only thing that
    actually acts on a command.

A disagreement between any two is INVISIBLE from the browser. That is the whole
reason this file exists: because the bridge fabricates its own ACK, a verb the
flight controller does not recognise still turns the dashboard green. A typo
that renames HOVER_Z to HOVERZ in one of the three would never surface as an
error anywhere -- the command would simply stop working, silently, forever.

Until 2026-09-22 the flight controller had no reader at all: every rose_cmd_*
hook sat behind `#if defined(CONFIG_WIFI)`, which is never set in any FPGA
build, so the entire vocabulary was accepted by the dashboards, forwarded by the
ESP, acknowledged, and then dropped on the floor. This test would have failed
loudly on day one.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

FC_MAIN = (
    ROOT
    / "workloads/flight_controller/src/main.cpp"
)
PANEL = (
    ROOT
    / "tools/bench/groundstation/riskybird_panel.py"
)

# The newer dashboard lives in the setup tree, which is a sibling checkout and
# may not be present. Checked when it is, skipped when it is not, rather than
# hard-failing a repo that legitimately does not contain it.
DASH_CANDIDATES = (
    ROOT / "tools/bench/riskybird-dash.py",
    ROOT.parent / "riskybird-dev-setup/tools/bench/riskybird-dash.py",
)

WHITELIST_RE = re.compile(r"head not in \(([^)]*)\)")
STRING_RE = re.compile(r'"([A-Z_0-9]+)"')


def whitelist_from(path: Path) -> set[str]:
    """The verbs a ground station will forward, read from its own source."""
    m = WHITELIST_RE.search(path.read_text())
    if not m:
        raise AssertionError(f"{path.name}: no `head not in (...)` whitelist found")
    return set(STRING_RE.findall(m.group(1)))


def fc_vocabulary() -> set[str]:
    """The verbs uart_cmd_dispatch() compares against.

    Read from the source rather than from a list kept here, so that adding a
    command to the firmware and forgetting the ground station -- or the reverse
    -- is what fails, instead of this file quietly describing neither.
    """
    text = FC_MAIN.read_text()
    start = text.find("static void uart_cmd_dispatch(")
    if start < 0:
        raise AssertionError("uart_cmd_dispatch() not found in main.cpp")
    end = text.find("\nstatic void uart_cmd_poll(", start)
    body = text[start:end if end > 0 else len(text)]
    verbs = set(re.findall(r'strcmp\(line, "([A-Z_0-9]+)"\)', body))
    verbs |= set(re.findall(r'strncmp\(line, "([A-Z_0-9]+)"', body))
    return verbs


class UplinkCommandContract(unittest.TestCase):
    def test_flight_controller_has_a_reader_at_all(self):
        """The regression that made every button on every dashboard a no-op."""
        text = FC_MAIN.read_text()
        self.assertIn(
            "uart_cmd_dispatch", text,
            "the flight controller has no uplink command reader: every "
            "rose_cmd_* hook is unreachable and the ground stations are "
            "commanding nothing",
        )
        # The hooks must not be WiFi-only again. This carrier has no radio, so
        # a guard of `#if defined(CONFIG_WIFI)` alone compiles them out.
        self.assertRegex(
            text, r"#if defined\(CONFIG_WIFI\) \|\| ROSE_UART_CMD",
            "rose_cmd_* is gated on CONFIG_WIFI alone again -- that is never "
            "set in an FPGA build, so the hooks are absent from the image",
        )

    def test_fc_understands_everything_dimas_panel_can_send(self):
        panel = whitelist_from(PANEL)
        fc = fc_vocabulary()
        missing = panel - fc
        self.assertFalse(
            missing,
            f"riskybird_panel.py can send {sorted(missing)}, which "
            f"uart_cmd_dispatch() does not recognise. The ESP bridge will ACK "
            f"them anyway, so this fails silently on the bench.",
        )

    def test_fc_understands_everything_the_dashboard_can_send(self):
        for path in DASH_CANDIDATES:
            if path.is_file():
                break
        else:
            self.skipTest("riskybird-dash.py is not in this checkout")
        dash = whitelist_from(path)
        fc = fc_vocabulary()
        missing = dash - fc
        self.assertFalse(
            missing,
            f"{path.name} can send {sorted(missing)}, which "
            f"uart_cmd_dispatch() does not recognise.",
        )

    def test_every_verb_the_fc_accepts_is_reachable_from_a_ground_station(self):
        """The other direction: a command nothing can send is dead code."""
        fc = fc_vocabulary()
        reachable = whitelist_from(PANEL)
        for path in DASH_CANDIDATES:
            if path.is_file():
                reachable |= whitelist_from(path)
        stranded = fc - reachable
        self.assertFalse(
            stranded,
            f"uart_cmd_dispatch() accepts {sorted(stranded)}, but no ground "
            f"station whitelist will forward it -- either add it to a "
            f"whitelist or drop it from the firmware.",
        )

    def test_the_reply_is_the_flight_controllers_own(self):
        """esp_bridge's ACK means 'delivered to a UART', never 'acted on'.

        The FC therefore answers with its own distinct token. If these ever
        collapse into the same string, a delivered-but-ignored command becomes
        indistinguishable from an executed one again.
        """
        text = FC_MAIN.read_text()
        self.assertIn('"FCACK"', text)
        self.assertIn('"FCNAK"', text)

    def test_arming_is_not_reachable_from_a_browser(self):
        """A deliberate design property, stated by the groundstation README as
        'There is no remote force-ARM by design -- arming stays on the on-board
        gate.' Nothing in the uplink vocabulary may arm."""
        fc = fc_vocabulary()
        for verb in ("ARM", "TAKEOFF", "FLY", "LAUNCH"):
            self.assertNotIn(
                verb, fc,
                f"{verb} is reachable over the uplink: arming must stay behind "
                f"the on-board lift-and-place gate",
            )


if __name__ == "__main__":
    unittest.main()
