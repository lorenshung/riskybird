#!/usr/bin/env python3
"""Collect elaborated devicetrees into the layout the side repositories use.

The DTS Chipyard emits is what `rb` reads to derive a shell's PLIC sources, its
periphery and its memory map. None of that needs a toolchain to consult -- only
the file. Collecting them lets a host that cannot elaborate (a Mac: Vivado has
no macOS release) still build workloads and program a board.

Laid out like tools/collect-bitstreams.py's output so the three side
repositories are interchangeable to a reader:

    <board>/<board>.<Config>.dts

Companion to that script. Where it publishes what the board runs, this
publishes what the build needs to know about the board.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GENERATED = ROOT / "backends" / "chipyard" / "fpga" / "generated-src"
BOARD_OF_PACKAGE = {
    "chipyard.fpga.arty100t": "arty100t",
    "chipyard.fpga.arty200t": "arty200t",
    "chipyard.fpga.ku040": "ku040",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def git_rev(path: Path) -> str:
    try:
        return subprocess.run(
            ["git", "-C", str(path), "rev-parse", "HEAD"],
            capture_output=True, text=True, check=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def summarise(text: str) -> dict:
    """The facts rb actually reads out of a DTS, recorded for the manifest.

    Not a parse of the whole tree: these are the fields that decide whether a
    workload can be built against this shell, so a reader can tell two configs
    apart without opening either file.
    """
    nodes = {}
    for match in re.finditer(r"(\w+)@([0-9a-f]+)\s*\{(.*?)\n\t\t\};", text, re.S):
        kind, addr, body = match.groups()
        irq = re.search(r"interrupts = <([^>]*)>", body)
        nodes[f"{kind}@{addr}"] = irq.group(1).strip() if irq else None
    memory = re.search(r"memory@([0-9a-f]+)\s*\{.*?reg = <([^>]*)>", text, re.S)
    return {
        "peripherals": dict(sorted(nodes.items())),
        "memory": memory.group(0).split("reg = <")[-1].rstrip(">") if memory else None,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=str(ROOT / "dts"))
    args = ap.parse_args()
    out_dir = Path(args.out)

    entries = []
    for dts in sorted(GENERATED.glob("*/*.dts")):
        gen = dts.parent.name
        match = re.match(r"^(chipyard\.fpga\.[a-z0-9]+)\.([A-Za-z0-9]+)\.(.+)$", gen)
        if not match:
            print(f"skip (unrecognised name): {gen}")
            continue
        package, harness, config = match.groups()
        board = BOARD_OF_PACKAGE.get(package, package.split(".")[-1])

        dest = out_dir / board / f"{board}.{config}.dts"
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(dts, dest)

        entries.append({
            "board": board,
            "config": config,
            "harness": harness,
            "file": f"{board}/{dest.name}",
            "source": str(dts.relative_to(ROOT)),
            "bytes": dest.stat().st_size,
            "sha256": sha256(dest),
            **summarise(dts.read_text(encoding="utf-8", errors="replace")),
        })
        print(f"{dest.relative_to(out_dir)}  <-  {gen}")

    manifest = {
        "generated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "source_revisions": {
            "riskybird-dev": git_rev(ROOT),
            "chipyard": git_rev(ROOT / "backends" / "chipyard"),
        },
        "entries": entries,
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    readme = [
        "# RiskyBird elaborated devicetrees",
        "",
        "Collected by `tools/collect-dts.py` from a Chipyard checkout. One file",
        "per board and config:",
        "",
        "    <board>/<board>.<Config>.dts",
        "",
        "`rb` reads these to derive a shell's PLIC sources, the periphery it",
        "elaborated and its memory map. None of that needs Chipyard installed --",
        "only the file -- so a host that cannot elaborate can still build",
        "workloads and program a board:",
        "",
        "    export RB_DTS_DIR=/path/to/this/checkout",
        "    rb build hello_world --target arty200t --config <Config>",
        "",
        "`rb` prefers a local Chipyard elaboration when one is present, so on a",
        "machine that has Chipyard this directory is ignored.",
        "",
        "`manifest.json` records the source revisions and, per entry, the",
        "peripherals and their PLIC sources -- enough to tell two configs apart",
        "without opening either file.",
        "",
        f"{len(entries)} devicetree(s).",
        "",
    ]
    (out_dir / "README.md").write_text("\n".join(readme))
    print(f"\n{len(entries)} devicetree(s) -> {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
