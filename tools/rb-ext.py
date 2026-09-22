#!/usr/bin/env python3
"""`rb`, plus the bench workloads that are not in the registry yet.

WHY THIS FILE EXISTS
--------------------
`tools/rb/boards.py` is the single registry of workloads, and a workload that is
not in it cannot be named on an `rb` command line at all. That file was being
edited by someone else while `workloads/motor_duty` was written, and a registry
is exactly the wrong file for two people to be holding at once -- so the entry
lives here instead, added to the same dictionary at run time.

This is a SHIM, not a second entry point. It adds registry rows and then hands
the whole command line to `rb`'s own `main()`, so every subcommand, every flag
and every safety check behaves identically:

    ./tools/rb-ext.py build motor_duty --target arty200t \\
        --config RocketArty200TDroneFullDDRConfig

The permanent form is the same rows pasted into `WORKLOADS` in boards.py, after
which this file can be deleted and `./rb` used directly. Nothing else depends on
it.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.rb import boards  # noqa: E402  (path setup must come first)
from tools.rb.cli import main  # noqa: E402


# Rows to fold into boards.py's WORKLOADS when that file is free again.
#
# `targets` is arty200t only: the workload writes to uart@10021000, which only
# the full drone shells elaborate, and rb refuses the build on a shell without
# it rather than producing an image whose link goes nowhere.
EXTRA_WORKLOADS = {
    "motor_duty": boards.Workload(
        name="motor_duty",
        source_dir=boards.ROOT / "workloads" / "motor_duty",
        targets=("arty200t",),
        summary=(
            "Hold a fixed duty on chosen motors by sending motor_link frames "
            "to the ESP32-C6, for a thrust-curve bench test"
        ),
        # `safety` is a free-form label that rb only ever prints (cli.py's
        # workload listing). Propellers ON is the whole point of a thrust test,
        # so labelling this "propellers-off" like every other motor workload
        # would put a false statement in the one place an operator reads before
        # deciding what to fit to the airframe. It says what it is instead, and
        # the procedure it names is the one that makes it safe.
        safety="props-on-RESTRAINED (docs/flight-calibration-procedures.md)",
    ),
}


def install() -> None:
    """Add the rows, refusing to overwrite anything already registered."""
    for name, workload in EXTRA_WORKLOADS.items():
        if name in boards.WORKLOADS:
            # Someone finished the boards.py edit. Theirs wins: a shim that
            # silently replaced a real registry row would hide the fact that
            # this file is no longer needed.
            continue
        boards.WORKLOADS[name] = workload


if __name__ == "__main__":
    install()
    raise SystemExit(main())
