"""Check that every row of prune/moved-to-top.tsv names a blob that exists at
the named source path in the named commit of the pre-prune clone.

The clones are not part of this checkout, so the test skips when they are
absent. Point RB_PRUNE_CLONES at a directory holding zcs-rb-clean and
mb-rb-clean to override the default location.
"""
import csv
import os
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CLONES = Path(os.environ.get("RB_PRUNE_CLONES", "/scratch2/loren/cleanup"))
REPOS = {
    "zephyr-chipyard-sw": CLONES / "zcs-rb-clean",
    "modelblaster": CLONES / "mb-rb-clean",
    # The other backends are not pruned clones; their source commits are
    # checked in the backend checkout when it is present.
    "chipyard": Path(os.environ.get("RB_CHIPYARD", ROOT / "backends" / "chipyard")),
}
# Rows whose blob is "extract" name a symbol cut out of a larger file.
EXTRACT = "extract"
# Copies that must still be byte-identical to the manifest blob.
UNMODIFIED = ("hardware/rtl/fcrocc/prototype/", "hardware/rtl/fcrocc/sim/",
              "hardware/rtl/fcrocc/vsrc/")
# Deliberate changes since the move (see prune/pin-clean-proof.txt, section 9): the IMU lever-arm
# sign in the boot image (IMU_OFFSET_Y -0.016 -> +0.016). The ROM itself is unchanged.
CHANGED_ON_PURPOSE = ("hardware/rtl/fcrocc/vsrc/fc_rocc_init.hex",)


def rows():
    with open(ROOT / "prune" / "moved-to-top.tsv", newline="") as f:
        return list(csv.DictReader(f, delimiter="\t"))


class ManifestTest(unittest.TestCase):
    def test_columns_and_destinations(self):
        seen = set()
        for r in rows():
            self.assertTrue(len(r["blob"]) == 40 or r["blob"] == EXTRACT, r)
            self.assertNotIn(r["destination"], seen, "duplicate destination")
            seen.add(r["destination"])

    def test_blobs_exist_in_source_commits(self):
        checked = 0
        for r in rows():
            clone = REPOS.get(r["backend"])
            if clone is None or not clone.is_dir() or r["blob"] == EXTRACT:
                continue
            if subprocess.run(["git", "-C", str(clone), "cat-file", "-e", r["commit"]],
                              capture_output=True).returncode:
                continue  # commit not available in this checkout
            out = subprocess.run(
                ["git", "-C", str(clone), "rev-parse", f"{r['commit']}:{r['source']}"],
                capture_output=True, text=True)
            self.assertEqual(out.stdout.strip(), r["blob"],
                             f"{r['backend']} {r['commit']}:{r['source']} {out.stderr.strip()}")
            checked += 1
        if not checked:
            self.skipTest("no source clones available")

    def test_unmodified_copies_match(self):
        for r in rows():
            if (r["destination"].startswith(UNMODIFIED) and r["blob"] != EXTRACT
                    and r["destination"] not in CHANGED_ON_PURPOSE):
                out = subprocess.run(["git", "-C", str(ROOT), "rev-parse", f"HEAD:{r['destination']}"],
                                     capture_output=True, text=True)
                if out.returncode == 0:
                    self.assertEqual(out.stdout.strip(), r["blob"], r["destination"])


if __name__ == "__main__":
    unittest.main()
