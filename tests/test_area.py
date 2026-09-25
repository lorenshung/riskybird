"""Regression tests for the FPGA area-characterization flow.

The Vivado half of this flow cannot run in a unit test, so these tests pin the
two things that would otherwise break silently: the report parsers, and the
guardrails that stop incomparable measurements from being plotted together.
"""

from __future__ import annotations

import json
import unittest
import xml.dom.minidom
from datetime import datetime, timezone
from pathlib import Path
from tempfile import TemporaryDirectory

from tools.rb import area, plotting
from tools.rb.boards import BOARDS, CHIPYARD, ROOT

# The setup branch carries neither the experiments/ tree nor an initialised
# Chipyard; tests over either skip there rather than fail.
CHIPYARD_READY = (CHIPYARD / "build.sbt").is_file()
EXPERIMENTS = ROOT / "experiments"

# A faithful excerpt of `report_utilization` on an UltraScale part. The numbers
# are the recorded RocketKU040OspiConfig post-route result from
# docs/history/verification.md.
ULTRASCALE_FLAT = """Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
----------------------------------------------------------------------
| Tool Version : Vivado v.2023.1 (lin64) Build 3865809
| Date         : Fri Jul 24 09:27:48 2026
| Device       : xcku040-sfva784-1-c
| Design State : Synthesized
----------------------------------------------------------------------

Utilization Design Information

1. CLB Logic
------------

+----------------------------+-------+-------+------------+-----------+-------+
|          Site Type         |  Used | Fixed | Prohibited | Available | Util% |
+----------------------------+-------+-------+------------+-----------+-------+
| CLB LUTs                   | 38908 |     0 |          0 |    242400 | 16.05 |
|   LUT as Logic             | 36613 |     0 |          0 |    242400 | 15.10 |
|   LUT as Memory            |  2295 |     0 |          0 |    112800 |  2.03 |
| CLB Registers              | 17514 |     0 |          0 |    484800 |  3.61 |
+----------------------------+-------+-------+------------+-----------+-------+

2. BLOCKRAM
-----------

+-------------------+------+-------+------------+-----------+-------+
|     Site Type     | Used | Fixed | Prohibited | Available | Util% |
+-------------------+------+-------+------------+-----------+-------+
| Block RAM Tile    |   97 |     0 |          0 |       600 | 16.17 |
|   RAMB36/FIFO     |   57 |     0 |          0 |       600 |  9.50 |
|   RAMB18          |   80 |     0 |          0 |      1200 |  6.67 |
+-------------------+------+-------+------------+-----------+-------+

3. ARITHMETIC
-------------

+-----------+------+-------+------------+-----------+-------+
| Site Type | Used | Fixed | Prohibited | Available | Util% |
+-----------+------+-------+------------+-----------+-------+
| DSPs      |   25 |     0 |          0 |      1920 |  1.30 |
+-----------+------+-------+------------+-----------+-------+
"""

# The same report on a 7-series part, where Vivado renames the LUT and register
# rows. The parser must not depend on the UltraScale spelling.
SERIES7_FLAT = """1. Slice Logic
--------------

+-------------------------+-------+-------+-----------+-------+
|        Site Type        |  Used | Fixed | Available | Util% |
+-------------------------+-------+-------+-----------+-------+
| Slice LUTs              | 49256 |     0 |     63400 | 77.69 |
| Slice Registers         | 28333 |     0 |    126800 | 22.35 |
+-------------------------+-------+-------+-----------+-------+
"""

# Real `report_utilization -hierarchical` shape, including the parenthesized
# self-only rows and two-space-per-level indentation.
HIERARCHICAL = """1. Utilization by Hierarchy
---------------------------

+-----------------+------------------+------------+------------+---------+-------+--------+--------+------------+
|     Instance    |      Module      | Total LUTs | Logic LUTs | LUTRAMs |  FFs  | RAMB36 | RAMB18 | DSP Blocks |
+-----------------+------------------+------------+------------+---------+-------+--------+--------+------------+
| KU040Harness    |            (top) |      38908 |      36613 |    2294 | 17514 |     57 |     80 |         25 |
|   (KU040Harness)|            (top) |          0 |          0 |       0 |    34 |      0 |      0 |          0 |
|   chiptop0      |          ChipTop |      38904 |      36609 |    2294 | 17462 |     57 |     80 |         25 |
|     system      |       DigitalTop |      38901 |      36606 |    2294 | 17451 |     57 |     80 |         25 |
|       bank      |   ScratchpadBank |        229 |        113 |     116 |   123 |     16 |      0 |          0 |
+-----------------+------------------+------------+------------+---------+-------+--------+--------+------------+
"""


def _record(
    board: str = "ku040",
    config: str = "RocketKU040OspiConfig",
    part: str = "xcku040-sfva784-1-c",
    attrs: str = "none",
    flow: str = "synth-only",
    totals: dict | None = None,
    upper_bound: bool = False,
) -> area.AreaRecord:
    if totals is None:
        totals = area.normalize_totals(area.parse_flat_utilization(ULTRASCALE_FLAT))
    return area.AreaRecord(
        run_id=f"test-{board}-{config}-{attrs}",
        board=board,
        config=config,
        attrs=attrs,
        flow=flow,
        evidence="synthesized",
        recorded_at=datetime.now(timezone.utc).isoformat(timespec="seconds"),
        device={"part": part, "tool": {"version": "v.2023.1"}},
        design={},
        provenance={},
        rtl_mutation={
            "mode": attrs,
            "pristine": attrs == "none",
            "totals_are_upper_bound": upper_bound,
            "injections": [],
        },
        totals=totals,
        hierarchy=area.parse_hierarchical_utilization(HIERARCHICAL),
    )


class FlatReportParsingTests(unittest.TestCase):
    def test_ultrascale_totals_match_the_report(self) -> None:
        totals = area.normalize_totals(area.parse_flat_utilization(ULTRASCALE_FLAT))
        self.assertEqual(totals["luts"]["used"], 38908)
        self.assertEqual(totals["luts"]["available"], 242400)
        self.assertAlmostEqual(totals["luts"]["util_pct"], 16.05)
        self.assertEqual(totals["registers"]["used"], 17514)
        self.assertEqual(totals["brams"]["used"], 97)
        self.assertEqual(totals["ramb36"]["used"], 57)
        self.assertEqual(totals["ramb18"]["used"], 80)
        self.assertEqual(totals["dsps"]["used"], 25)
        self.assertEqual(totals["dsps"]["available"], 1920)

    def test_device_capacity_is_never_hardcoded(self) -> None:
        """Capacity must come from Vivado, so a part swap cannot go unnoticed."""
        totals = area.normalize_totals(area.parse_flat_utilization(ULTRASCALE_FLAT))
        for entry in totals.values():
            if "available" in entry:
                self.assertIsNotNone(entry["available"])
        source = (ROOT / "tools" / "rb" / "area.py").read_text()
        self.assertNotIn("242400", source)
        self.assertNotIn("1920", source)

    def test_series7_row_names_are_normalized(self) -> None:
        totals = area.normalize_totals(area.parse_flat_utilization(SERIES7_FLAT))
        self.assertEqual(totals["luts"]["used"], 49256)
        self.assertEqual(totals["luts"]["vivado_row"], "Slice LUTs")
        self.assertEqual(totals["registers"]["used"], 28333)

    def test_footnote_asterisks_do_not_defeat_normalization(self) -> None:
        """Vivado marks some rows with a footnote asterisk depending on design state.

        A real post-synthesis report of RocketKU040OspiConfig labels the rows
        'CLB LUTs*' and 'RAMB36/FIFO*'. Matching must ignore the marker, while
        totals_raw keeps the label exactly as Vivado wrote it.
        """
        report = """1. CLB Logic
------------

+----------------------------+-------+-------+------------+-----------+-------+
|          Site Type         |  Used | Fixed | Prohibited | Available | Util% |
+----------------------------+-------+-------+------------+-----------+-------+
| CLB LUTs*                  | 39289 |     0 |          0 |    242400 | 16.21 |
| CLB Registers              | 17536 |     0 |          0 |    484800 |  3.62 |
+----------------------------+-------+-------+------------+-----------+-------+

2. BLOCKRAM
-----------

+-------------------+------+-------+------------+-----------+-------+
|     Site Type     | Used | Fixed | Prohibited | Available | Util% |
+-------------------+------+-------+------------+-----------+-------+
| Block RAM Tile    |   97 |     0 |          0 |       600 | 16.17 |
|   RAMB36/FIFO*    |   57 |     0 |          0 |       600 |  9.50 |
|     RAMB36E2 only |   57 |       |            |           |       |
|   RAMB18          |   80 |     0 |          0 |      1200 |  6.67 |
+-------------------+------+-------+------------+-----------+-------+
"""
        raw = area.parse_flat_utilization(report)
        # Faithful to Vivado, marker included.
        self.assertIn("CLB LUTs*", raw)
        self.assertIn("RAMB36/FIFO*", raw)

        totals = area.normalize_totals(raw)
        self.assertEqual(totals["luts"]["used"], 39289)
        self.assertEqual(totals["luts"]["available"], 242400)
        self.assertEqual(totals["luts"]["vivado_row"], "CLB LUTs*")
        # The asterisked row carries device capacity; the nested 'RAMB36E2 only'
        # row does not, so the marker-tolerant match must win.
        self.assertEqual(totals["ramb36"]["used"], 57)
        self.assertEqual(totals["ramb36"]["available"], 600)
        self.assertEqual(totals["ramb36"]["vivado_row"], "RAMB36/FIFO*")

    def test_report_preamble_is_not_mistaken_for_a_table(self) -> None:
        totals = area.parse_flat_utilization(ULTRASCALE_FLAT)
        self.assertNotIn("Tool Version : Vivado v.2023.1 (lin64) Build 3865809", totals)
        self.assertNotIn("Device", totals)


class HierarchicalReportParsingTests(unittest.TestCase):
    def test_rows_and_depth_are_recovered(self) -> None:
        rows = area.parse_hierarchical_utilization(HIERARCHICAL)
        self.assertEqual(len(rows), 5)
        top = rows[0]
        self.assertEqual(top["instance"], "KU040Harness")
        self.assertEqual(top["depth"], 0)
        self.assertEqual(top["metrics"]["Total LUTs"], 38908)
        self.assertEqual(top["metrics"]["DSP Blocks"], 25)
        # Indentation must survive into depth; a stripped cell would flatten it.
        self.assertGreater(rows[2]["depth"], rows[0]["depth"])
        self.assertGreater(rows[3]["depth"], rows[2]["depth"])

    def test_self_only_rows_are_flagged(self) -> None:
        rows = area.parse_hierarchical_utilization(HIERARCHICAL)
        self.assertTrue(rows[1]["self_only"])
        self.assertFalse(rows[0]["self_only"])

    def test_totals_fall_back_to_the_hierarchy_top_row(self) -> None:
        rows = area.parse_hierarchical_utilization(HIERARCHICAL)
        totals = area.totals_from_hierarchy(rows)
        self.assertEqual(totals["luts"]["used"], 38908)
        self.assertEqual(totals["dsps"]["used"], 25)
        # No capacity is available from a hierarchical report, so no percentages
        # may be derived from it.
        self.assertNotIn("available", totals["luts"])
        self.assertNotIn("util_pct", totals["luts"])


class ComparabilityTests(unittest.TestCase):
    def test_flow_and_attrs_are_part_of_a_record_identity(self) -> None:
        pristine = _record(attrs="none")
        injected = _record(attrs="hierarchy", upper_bound=True)
        self.assertNotEqual(pristine.comparison_key, injected.comparison_key)

    def test_pristine_and_dsp_are_comparable(self) -> None:
        """use_dsp changes primitive selection; it does not inflate totals.

        Measuring the LUT-to-DSP shift is the point of the injection, so a
        pristine record and a dsp record must land in the same comparison group
        and be plottable on one axis. Only keep_hierarchy, which inhibits
        cross-boundary optimization, is an upper bound.
        """
        pristine = _record(attrs="none")
        injected = _record(attrs="dsp", upper_bound=False)
        self.assertEqual(pristine.comparison_key, injected.comparison_key)
        svg = plotting.resource_chart([pristine, injected], "luts")
        self.assertIn("<svg", svg)

    def test_keep_hierarchy_still_cannot_be_compared(self) -> None:
        pristine = _record(attrs="none")
        hier = _record(attrs="hierarchy", upper_bound=True)
        self.assertNotEqual(pristine.comparison_key, hier.comparison_key)
        with self.assertRaises(plotting.PlotError):
            plotting.resource_chart([pristine, hier], "luts")

    def test_injection_chart_pairs_pristine_against_dsp(self) -> None:
        records = [_record(attrs="none"), _record(attrs="dsp", upper_bound=False)]
        svg = plotting.injection_chart(records, "luts")
        self.assertIn("pristine", svg)
        self.assertIn("use_dsp", svg)

    def test_injection_chart_needs_both_halves(self) -> None:
        with self.assertRaises(plotting.PlotError):
            plotting.injection_chart([_record(attrs="none")], "luts")

    def test_grouping_separates_incomparable_records(self) -> None:
        records = [
            _record(attrs="none"),
            _record(attrs="hierarchy", upper_bound=True),
            _record(board="arty100t", part="xc7a100tfgg484-2"),
            _record(flow="post-route"),
        ]
        self.assertEqual(len(area.group_comparable(records)), 4)

    def test_plotting_refuses_to_mix_comparison_groups(self) -> None:
        records = [_record(attrs="none"), _record(attrs="hierarchy", upper_bound=True)]
        with self.assertRaises(plotting.PlotError) as caught:
            plotting.resource_chart(records, "luts")
        self.assertIn("comparison group", str(caught.exception))

    def test_plotting_accepts_one_group(self) -> None:
        svg = plotting.resource_chart([_record()], "luts")
        self.assertIn("<svg", svg)

    def test_upper_bound_is_surfaced_in_the_figure(self) -> None:
        svg = plotting.resource_chart(
            [_record(attrs="hierarchy", upper_bound=True)], "luts"
        )
        self.assertIn("UPPER BOUND", svg)

    def test_latest_per_config_supersedes_older_runs(self) -> None:
        old = _record()
        old.recorded_at = "2026-01-01T00:00:00+00:00"
        old.run_id = "old"
        new = _record()
        new.recorded_at = "2026-07-01T00:00:00+00:00"
        new.run_id = "new"
        kept = area.latest_per_config([old, new])
        self.assertEqual([r.run_id for r in kept], ["new"])


class FigureTests(unittest.TestCase):
    def test_every_figure_is_well_formed_xml(self) -> None:
        record = _record()
        with TemporaryDirectory() as tmp:
            out = Path(tmp)
            written = plotting.write_figures([record], out)
            self.assertTrue(written)
            for path in written:
                xml.dom.minidom.parse(str(path))

    def test_utilization_chart_requires_device_capacity(self) -> None:
        rows = area.parse_hierarchical_utilization(HIERARCHICAL)
        record = _record(totals=area.totals_from_hierarchy(rows))
        record.totals_source = "hierarchy-top-row"
        with self.assertRaises(plotting.PlotError):
            plotting.utilization_chart(record)

    def test_figures_render_without_third_party_imports(self) -> None:
        source = (ROOT / "tools" / "rb" / "plotting.py").read_text()
        for banned in ("matplotlib", "numpy", "pandas", "plotly"):
            self.assertNotIn(banned, source)


class RecordRoundTripTests(unittest.TestCase):
    def test_record_survives_write_and_load(self) -> None:
        record = _record()
        with TemporaryDirectory() as tmp:
            out = area.write_record(record, Path(tmp))
            self.assertTrue((out / "record.json").is_file())
            self.assertTrue((out / "summary.csv").is_file())
            self.assertTrue((out / "hierarchy.csv").is_file())
            loaded = area.load_record(out)
        self.assertEqual(loaded.run_id, record.run_id)
        self.assertEqual(loaded.luts, record.luts)
        self.assertEqual(loaded.comparison_key, record.comparison_key)

    def test_schema_mismatch_is_rejected(self) -> None:
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "record.json"
            path.write_text(json.dumps({"schema": "something.else.v9"}))
            with self.assertRaises(area.AreaError):
                area.load_record(path)


class CollectionTests(unittest.TestCase):
    """Exercise the path from Vivado's outputs to a stored record."""

    MANIFEST = {
        "schema": "riskybird.area-run.v1",
        "flow": "synth-only",
        "synthesized_at": "2026-07-25T10:00:00-0700",
        "tool": {"name": "vivado", "version": "v.2023.1"},
        "device": {
            "part": "xcku040-sfva784-1-c",
            "board_part": "",
            "fpga_shells_board": "ku040",
        },
        "design": {
            "sub_project": "ku040",
            "config": "RocketKU040OspiConfig",
            "top_module": "KU040Harness",
        },
        "synth_options": "-flatten_hierarchy rebuilt",
        "rtl_mutation": {
            "mode": "none",
            "pristine": True,
            "totals_are_upper_bound": False,
            "injections": [],
        },
        "reports": ["rb_utilization_hier.txt", "rb_utilization_flat.txt"],
    }

    def _fixture(self, tmp: Path, manifest: dict | None = None) -> Path:
        report = tmp / "report"
        report.mkdir(parents=True, exist_ok=True)
        (report / "rb_manifest.json").write_text(json.dumps(manifest or self.MANIFEST))
        (report / "rb_utilization_hier.txt").write_text(HIERARCHICAL)
        (report / "rb_utilization_flat.txt").write_text(ULTRASCALE_FLAT)
        return report

    def test_record_is_built_from_vivado_outputs(self) -> None:
        board = BOARDS["ku040"]
        with TemporaryDirectory() as tmp:
            report = self._fixture(Path(tmp))
            record = area.collect_record(
                board, "none", ["make", "rb-area"], report_dir=report
            )
        self.assertEqual(record.luts, 38908)
        self.assertEqual(record.totals["dsps"]["used"], 25)
        self.assertEqual(record.totals["luts"]["available"], 242400)
        self.assertEqual(record.totals_source, "report_utilization")
        self.assertEqual(record.device["part"], "xcku040-sfva784-1-c")
        self.assertEqual(record.evidence, "synthesized")
        self.assertFalse(record.totals_are_upper_bound)
        self.assertTrue(record.hierarchy)

    def test_record_carries_the_soc_fingerprint(self) -> None:
        """The join key against a performance record.

        `BOARDS["ku040"]` defaults to RocketKU040OspiConfig, which is elaborated
        in this tree, so the fingerprint comes from the same generated-src that
        Vivado read.
        """
        board = BOARDS["ku040"]
        if not any(board.generated_dir.glob("*.dts")):   # a collected DTS is not elaboration
            self.skipTest("RocketKU040OspiConfig has not been elaborated")
        with TemporaryDirectory() as tmp:
            report = self._fixture(Path(tmp))
            record = area.collect_record(
                board, "none", ["make", "rb-area"], report_dir=report
            )
        self.assertIsNotNone(record.fingerprint)
        self.assertEqual(record.fingerprint["evidence"], "elaborated")
        self.assertEqual(record.fingerprint["hart_count"], 1)
        self.assertEqual(record.fingerprint["clock_mhz"], 50.0)
        self.assertEqual(record.identity_id, record.fingerprint["identity_id"])

    def test_an_unelaborated_config_still_records_its_area(self) -> None:
        """A missing DTS must not throw away a completed synthesis run."""
        board = BOARDS["ku040"].with_config("NeverElaboratedConfig")
        with TemporaryDirectory() as tmp:
            report = self._fixture(Path(tmp))
            record = area.collect_record(board, "none", [], report_dir=report)
        self.assertIsNone(record.fingerprint)
        self.assertIsNone(record.identity_id)
        self.assertEqual(record.luts, 38908)

    def test_records_written_before_fingerprinting_still_load(self) -> None:
        board = BOARDS["ku040"]
        with TemporaryDirectory() as tmp:
            report = self._fixture(Path(tmp))
            record = area.collect_record(board, "none", [], report_dir=report)
            payload = record.to_json()
            payload.pop("fingerprint")
            path = Path(tmp) / "record.json"
            path.write_text(json.dumps(payload))
            loaded = area.load_record(path)
        self.assertIsNone(loaded.fingerprint)
        self.assertEqual(loaded.luts, 38908)

    def test_provenance_records_the_generator_revisions(self) -> None:
        board = BOARDS["ku040"]
        with TemporaryDirectory() as tmp:
            report = self._fixture(Path(tmp))
            record = area.collect_record(
                board, "none", ["make", "rb-area"], report_dir=report
            )
        provenance = record.provenance
        self.assertIn("riskybird", provenance)
        self.assertIn("chipyard", provenance)
        self.assertIn("chipyard_rtl_submodules", provenance)
        self.assertEqual(provenance["command"], ["make", "rb-area"])

    def test_stale_report_directory_is_refused(self) -> None:
        """A report left by a different attribute mode must not be misattributed."""
        board = BOARDS["ku040"]
        with TemporaryDirectory() as tmp:
            report = self._fixture(Path(tmp))
            with self.assertRaises(area.AreaError) as caught:
                area.collect_record(
                    board, "hierarchy", ["make", "rb-area"], report_dir=report
                )
        self.assertIn("stale", str(caught.exception))

    def test_missing_hierarchical_report_is_an_error(self) -> None:
        board = BOARDS["ku040"]
        with TemporaryDirectory() as tmp:
            report = self._fixture(Path(tmp))
            (report / "rb_utilization_hier.txt").unlink()
            with self.assertRaises(area.AreaError):
                area.collect_record(board, "none", [], report_dir=report)

    def test_missing_flat_report_falls_back_and_says_so(self) -> None:
        board = BOARDS["ku040"]
        with TemporaryDirectory() as tmp:
            report = self._fixture(Path(tmp))
            (report / "rb_utilization_flat.txt").unlink()
            record = area.collect_record(board, "none", [], report_dir=report)
        self.assertEqual(record.totals_source, "hierarchy-top-row")
        self.assertEqual(record.luts, 38908)
        self.assertNotIn("available", record.totals["luts"])

    def test_injected_run_is_marked_as_an_upper_bound(self) -> None:
        manifest = dict(self.MANIFEST)
        manifest["rtl_mutation"] = {
            "mode": "hierarchy",
            "pristine": False,
            "totals_are_upper_bound": True,
            "injections": [
                {
                    "file": "MacUnit.sv",
                    "module": "MacUnit",
                    "attribute": 'use_dsp = "yes"',
                    "module_status": "applied",
                    "wire_status": "not-found",
                    "why": "test",
                }
            ],
        }
        board = BOARDS["ku040"]
        with TemporaryDirectory() as tmp:
            report = self._fixture(Path(tmp), manifest)
            record = area.collect_record(board, "hierarchy", [], report_dir=report)
        self.assertTrue(record.totals_are_upper_bound)
        described = "\n".join(area.describe_mutation(record))
        self.assertIn("UPPER BOUND", described)
        # A wire pattern that stopped matching means the RTL changed shape; that
        # must be surfaced, not swallowed.
        self.assertIn("WARNING", described)
        self.assertIn("MacUnit.sv", described)

    def test_pristine_run_says_so_plainly(self) -> None:
        board = BOARDS["ku040"]
        with TemporaryDirectory() as tmp:
            report = self._fixture(Path(tmp))
            record = area.collect_record(board, "none", [], report_dir=report)
        self.assertIn("pristine", "\n".join(area.describe_mutation(record)))


class MakeCommandTests(unittest.TestCase):
    def test_command_targets_the_riskybird_area_rule(self) -> None:
        board = BOARDS["ku040"]
        command = area.make_command(board, "none", force=False)
        self.assertIn("SUB_PROJECT=ku040", command)
        self.assertIn("CONFIG=RocketKU040OspiDDRConfig", command)
        self.assertIn("RB_ATTRS=none", command)
        self.assertEqual(command[-1], "rb-area")

    def test_force_selects_the_unconditional_rule(self) -> None:
        command = area.make_command(BOARDS["ku040"], "all", force=True)
        self.assertEqual(command[-1], "rb-area-force")
        self.assertIn("RB_ATTRS=all", command)

    def test_selected_config_reaches_the_command(self) -> None:
        board = BOARDS["ku040"].with_config("SaturnKU040OspiConfig")
        self.assertIn("CONFIG=SaturnKU040OspiConfig", area.make_command(board, "none", False))

    def test_invalid_attribute_mode_is_rejected(self) -> None:
        with self.assertRaises(area.AreaError):
            area.make_command(BOARDS["ku040"], "everything", force=False)


@unittest.skipUnless(CHIPYARD_READY, "backends/chipyard is not initialised")
class BackendIntegrationTests(unittest.TestCase):
    """The Chipyard-side hook must stay present, visible, and self-contained."""

    def test_riskybird_directory_holds_the_whole_flow(self) -> None:
        rb_dir = CHIPYARD / "riskybird"
        for name in ("README.md", "area.mk", "tcl/rb_synth_area.tcl", "tcl/rb_attributes.tcl"):
            self.assertTrue((rb_dir / name).is_file(), f"missing {name}")

    def test_chipyard_is_hooked_exactly_once_and_optionally(self) -> None:
        makefile = (CHIPYARD / "fpga" / "Makefile").read_text()
        hooks = [
            line
            for line in makefile.splitlines()
            if "riskybird" in line and "include" in line
        ]
        self.assertEqual(len(hooks), 1, f"expected one include hook, found {hooks}")
        # `-include` keeps Chipyard buildable if the directory is deleted.
        self.assertTrue(hooks[0].strip().startswith("-include"))

    def test_area_mk_defines_the_expected_targets(self) -> None:
        text = (CHIPYARD / "riskybird" / "area.mk").read_text()
        for target in ("rb-area:", "rb-area-force:", "rb-area-restore:"):
            self.assertIn(target, text)

    def test_injection_defaults_to_pristine_rtl(self) -> None:
        self.assertIn("RB_ATTRS ?= none", (CHIPYARD / "riskybird" / "area.mk").read_text())

    def test_riskybird_does_not_patch_the_shared_bitstream_flow(self) -> None:
        """The upstream DRC downgrade must not be adopted: it hides pin errors."""
        impl = CHIPYARD / "fpga" / "scripts" / "run_impl_bitstream.tcl"
        if impl.is_file():
            text = impl.read_text()
            self.assertNotIn("NSTD-1", text)
            self.assertNotIn("UCIO-1", text)


@unittest.skipUnless(
    (EXPERIMENTS / "studies").is_dir(),
    "experiments/studies lives on main; not on the setup branch",
)
class StudyDefinitionTests(unittest.TestCase):
    STUDIES = EXPERIMENTS / "studies"

    def test_every_study_is_valid_and_self_describing(self) -> None:
        studies = sorted(self.STUDIES.glob("*.json"))
        self.assertTrue(studies, "no studies defined")
        for path in studies:
            with self.subTest(study=path.stem):
                study = json.loads(path.read_text())
                self.assertEqual(study["schema"], "riskybird.area-study.v1")
                self.assertEqual(study["name"], path.stem)
                self.assertIn(study["board"], BOARDS)
                self.assertTrue(study["configs"])
                self.assertTrue(study.get("question"))
                if study.get("attrs"):
                    self.assertIn(study["attrs"], area.ATTR_MODES)

    def test_a_blocked_study_states_its_prerequisites(self) -> None:
        for path in self.STUDIES.glob("*.json"):
            study = json.loads(path.read_text())
            if study.get("blocked_reason"):
                with self.subTest(study=path.stem):
                    self.assertTrue(study.get("prerequisites"))

    def test_runnable_studies_only_reference_defined_configs(self) -> None:
        """A runnable study must not name a config that cannot elaborate."""
        configs_file = (
            CHIPYARD / "fpga" / "src" / "main" / "scala" / "ku040" / "Configs.scala"
        )
        if not configs_file.is_file():
            self.skipTest("Chipyard KU040 config source is unavailable")
        defined = configs_file.read_text()
        for path in self.STUDIES.glob("*.json"):
            study = json.loads(path.read_text())
            if study.get("blocked_reason") or study["board"] != "ku040":
                continue
            for config in study["configs"]:
                with self.subTest(study=path.stem, config=config):
                    self.assertIn(f"class {config}", defined)


@unittest.skipUnless(
    (EXPERIMENTS / "fpga-configs" / "manifest.json").is_file(),
    "experiments/fpga-configs lives on main; not on the setup branch",
)
class ImportArchiveTests(unittest.TestCase):
    ARCHIVE = EXPERIMENTS / "fpga-configs"

    def test_archive_is_marked_reference_only(self) -> None:
        manifest = json.loads((self.ARCHIVE / "manifest.json").read_text())
        self.assertEqual(manifest["status"], "reference-only")
        self.assertFalse(manifest["adopted"])
        self.assertFalse(manifest["riskybird_adopted"]["submodule_pins_moved"])
        self.assertFalse(manifest["riskybird_adopted"]["gitmodules_urls_changed"])

    def test_archived_files_exist_and_record_their_origin(self) -> None:
        manifest = json.loads((self.ARCHIVE / "manifest.json").read_text())
        self.assertTrue(manifest["archived_files"])
        for entry in manifest["archived_files"]:
            with self.subTest(path=entry["path"]):
                self.assertTrue((self.ARCHIVE / entry["path"]).is_file())
                self.assertTrue(entry["source_commit"])
                self.assertTrue(entry["git_blob"])

    def test_known_defects_are_recorded_rather_than_forgotten(self) -> None:
        manifest = json.loads((self.ARCHIVE / "manifest.json").read_text())
        ids = {defect["id"] for defect in manifest["known_defects"]}
        for expected in (
            "vrf-mask-coarsening",
            "q31-saturation-width",
            "q31-identity-is-half",
            "rocket-fpu16-non-spec",
        ):
            self.assertIn(expected, ids)
        for defect in manifest["known_defects"]:
            self.assertFalse(defect["fixed_in_archive"])

    def test_submodule_pins_have_not_silently_moved(self) -> None:
        """The archive documents pins it needs; adopting them is a live decision."""
        manifest = json.loads((self.ARCHIVE / "manifest.json").read_text())
        for entry in manifest["required_submodule_revisions"]:
            with self.subTest(path=entry["path"]):
                self.assertNotEqual(entry["riskybird_pin"], entry["required"])
                self.assertFalse(entry["exists_on_canonical_upstream"])


if __name__ == "__main__":
    unittest.main()
