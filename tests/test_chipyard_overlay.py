"""The Chipyard overlay: hardware/chipyard/project/RiskyBird.scala and friends.

RiskyBird's Chipyard code lives in this repository and is compiled into a plain
Chipyard checkout by an sbt plugin that tools/backend-patches links in. These
checks keep the pieces that cannot be compiled here consistent with each other
and, when Chipyard is checked out, with the pinned Chipyard.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PLUGIN = ROOT / "hardware/chipyard/project/RiskyBird.scala"
RB_DIGITAL_TOP = ROOT / "hardware/chipyard/generators/chipyard/src/main/scala/RbDigitalTop.scala"
DIGITAL_TOP = ROOT / "backends/chipyard/generators/chipyard/src/main/scala/DigitalTop.scala"
OVERLAYS = ROOT / "backends" / "overlays"
QUEUE = ROOT / "hardware/chipyard/patches"
RISKYBIRD_TRAITS = ["ospi.CanHavePeripheryOspi", "uartdma.CanHavePeripheryUartTxDma"]


def traits(path: Path, cls: str) -> list[str]:
    text = path.read_text()
    body = text.split(f"class {cls}(implicit p: Parameters) extends ChipyardSystem", 1)[1]
    body = body.split("{", 1)[0]
    return re.findall(r"^\s*with\s+([\w.]+)", body, re.M)


class OverlayTests(unittest.TestCase):
    def test_plugin_overlay_roots_exist(self) -> None:
        roots = re.findall(r'"(hardware/[^"]+)"', PLUGIN.read_text())
        self.assertTrue(roots)
        for root in roots:
            with self.subTest(root=root):
                self.assertTrue((ROOT / root).exists(), root)

    def test_overlay_link_targets_exist(self) -> None:
        for line in OVERLAYS.read_text().splitlines():
            line = line.split("#", 1)[0].split()
            if not line:
                continue
            link, target = line
            with self.subTest(link=link):
                self.assertTrue(link.startswith("backends/"), link)
                self.assertTrue((ROOT / target).exists(), target)

    def test_every_queued_patch_explains_itself(self) -> None:
        """A queue patch carries a description before its first diff: what it
        changes and why it is generic (or why it is an open decision)."""
        patches = sorted(QUEUE.rglob("*.patch"))
        self.assertTrue(patches)
        for patch in patches:
            with self.subTest(patch=str(patch.relative_to(ROOT))):
                head = patch.read_text().split("diff --git", 1)[0].strip()
                self.assertGreater(len(head.splitlines()), 1)

    def test_rb_digital_top_adds_only_the_riskybird_traits(self) -> None:
        mine = traits(RB_DIGITAL_TOP, "RbDigitalTop")
        for trait in RISKYBIRD_TRAITS:
            self.assertIn(trait, mine)
        gcd = mine.index("chipyard.example.CanHavePeripheryGCD")
        self.assertEqual(mine[gcd + 1 : gcd + 3], RISKYBIRD_TRAITS)

    @unittest.skipUnless(DIGITAL_TOP.is_file(), "backends/chipyard is not initialised")
    def test_rb_digital_top_follows_the_pinned_digital_top(self) -> None:
        """RbDigitalTop restates DigitalTop's trait list (order fixes the
        emitted Verilog). When a Chipyard bump changes DigitalTop, this fails
        until RbDigitalTop is updated to match."""
        stock = traits(DIGITAL_TOP, "DigitalTop")
        mine = [t for t in traits(RB_DIGITAL_TOP, "RbDigitalTop") if t not in RISKYBIRD_TRAITS]
        self.assertEqual(mine, stock)


if __name__ == "__main__":
    unittest.main()
