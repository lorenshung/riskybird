"""Regression tests for SoC fingerprint extraction.

The point of the fingerprint is that a configuration name is not evidence, so
these tests lean on real elaboration output wherever it exists: the DTS and
gen-collateral under ``backends/chipyard/fpga/generated-src`` are the actual
artifacts the extractor has to survive. Synthetic fixtures are used only for
parser edge cases that the tree does not happen to contain.

One landmine worth restating: a guessed fixture once hid a real bug in the area
parser, so where a claim can be checked against a measurement it is. The mesh
cell counts below are cross-checked against the measured DSP law (one DSP48E2
per mesh cell), which is independent evidence that the instantiation walk is
counting the right thing.
"""

from __future__ import annotations

import unittest
from pathlib import Path

from tools.rb import fingerprint as fp
from tools.rb.boards import get_board


GENERATED = (
    Path(__file__).resolve().parents[1]
    / "backends" / "chipyard" / "fpga" / "generated-src"
)


def _elaborated(config: str, board: str = "ku040"):
    b = get_board(board).with_config(config)
    if not b.generated_dir.is_dir():
        raise unittest.SkipTest(f"{config} has not been elaborated in this tree")
    return fp.for_board(b)


class DtsParserTests(unittest.TestCase):
    def test_nodes_properties_and_labels(self) -> None:
        tree = fp.parse_dts(
            """
            /dts-v1/;
            / {
                compatible = "ucb-bar,chipyard-dev";
                L2: cpus {
                    timebase-frequency = <50000>;
                    L12: cpu@0 {
                        device_type = "cpu";
                        riscv,isa = "rv64imac";
                        reg = <0x0>;
                    };
                };
            };
            """
        )
        self.assertEqual(tree.name, "/")
        self.assertEqual(tree.string("compatible"), "ucb-bar,chipyard-dev")
        cpus = tree.find("cpu@")
        self.assertEqual(len(cpus), 1)
        self.assertEqual(cpus[0].label, "L12")
        self.assertEqual(cpus[0].cell("reg"), 0)

    def test_comments_are_stripped_but_strings_survive(self) -> None:
        tree = fp.parse_dts(
            """
            / {
                /* a block comment { with braces } and a ; */
                model = "a/*b*/c";   // trailing
                path = "/soc/serial@10020000";
            };
            """
        )
        self.assertEqual(tree.string("model"), "a/*b*/c")
        self.assertEqual(tree.string("path"), "/soc/serial@10020000")

    def test_cell_lists_skip_phandle_references(self) -> None:
        tree = fp.parse_dts('/ { interrupts-extended = <&L10 3 &L10 7>; };')
        self.assertEqual(tree.cells("interrupts-extended"), [3, 7])

    def test_multiple_string_values(self) -> None:
        tree = fp.parse_dts('/ { compatible = "sifive,rocket0", "riscv"; };')
        self.assertEqual(tree.strings("compatible"), ["sifive,rocket0", "riscv"])


class IsaTests(unittest.TestCase):
    def test_chipyard_glues_the_first_multi_letter_extension_on(self) -> None:
        """``rv64imafdcbzicsr_...`` has no separator before ``zicsr``."""
        isa = fp.parse_isa("rv64imafdcbzicsr_zifencei_zihpm_zfh_zba_zbb_zbs_xrocket")
        self.assertEqual(isa.xlen, 64)
        self.assertEqual(isa.base, tuple("imafdcb"))
        self.assertIn("zicsr", isa.extensions)
        self.assertIn("xrocket", isa.extensions)

    def test_a_config_without_zvl_has_no_vector_unit(self) -> None:
        """The whole reason this module exists."""
        isa = fp.parse_isa("rv64imafdcbzicsr_zifencei_zihpm_zfh_zba_zbb_zbs_xrocket")
        self.assertFalse(isa.has_vector)
        self.assertIsNone(isa.vlen_bits)

    def test_vector_length_comes_from_zvl(self) -> None:
        isa = fp.parse_isa("rv64imafdcbvzicsr_zifencei_zvl128b_zve64d_zvfh")
        self.assertTrue(isa.has_vector)
        self.assertEqual(isa.vlen_bits, 128)

    def test_zve_alone_still_counts_as_vector(self) -> None:
        self.assertTrue(fp.parse_isa("rv64imac_zve32x").has_vector)

    def test_v_base_letter_counts_as_vector(self) -> None:
        self.assertTrue(fp.parse_isa("rv64gcv").has_vector)


class ElaboratedConfigTests(unittest.TestCase):
    """Against the real generated-src tree."""

    def test_ku040_configs_are_single_hart_at_50_mhz(self) -> None:
        for config in (
            "RocketKU040OspiConfig",
            "GemminiKU040Config",
            "Q31Ws32x32AccGemminiKU040Config",
            "SaturnOPUV128D128KU040Config",
        ):
            with self.subTest(config=config):
                got = _elaborated(config)
                self.assertEqual(got.hart_count, 1)
                self.assertEqual(got.clock_mhz, 50.0)
                self.assertEqual(got.evidence, "elaborated")

    def test_q31_gemmini_has_no_vector_unit(self) -> None:
        """It is nevertheless paired with a V256D128_rvv profile today.

        Name matching cannot catch that; this is the fact that makes the
        pairing wrong, read straight out of the elaborated ISA string.
        """
        got = _elaborated("Q31Ws32x32AccGemminiKU040Config")
        self.assertFalse(got.has_vector)
        self.assertIsNone(got.vlen_bits)
        self.assertIn("gemmini", got.accelerators)

    def test_mesh_cell_counts_match_the_measured_dsp_law(self) -> None:
        """One DSP48E2 per mesh cell, measured as +261 and +1029 with the five
        hardfloat FMAs in Rocket's FPU. Counting instantiations has to agree."""
        self.assertEqual(
            _elaborated("GemminiKU040Config").accelerators["gemmini"]["mesh_cells"], 256
        )
        self.assertEqual(
            _elaborated("Q31Ws32x32AccGemminiKU040Config")
            .accelerators["gemmini"]["mesh_cells"],
            1024,
        )

    def test_saturn_opu_is_detected_separately_from_gemmini(self) -> None:
        got = _elaborated("SaturnOPUV128D128KU040Config")
        self.assertIn("saturn-opu", got.accelerators)
        self.assertNotIn("gemmini", got.accelerators)
        self.assertEqual(got.vlen_bits, 128)

    def test_opu_mesh_is_256_cells_not_the_512_the_dsp_delta_suggested(self) -> None:
        """One OuterProductUnit x 16 clusters x 16 cells.

        The +517 DSP delta was read as 512 cells plus five FMAs, on the
        assumption of one DSP per cell. The recorded hierarchy.csv for that run
        shows two DSP Blocks per OuterProductCell, so 256 cells is the count and
        the per-cell LUT figure derived from 512 was half what it should be.
        See docs/findings.md section 2.
        """
        got = _elaborated("SaturnOPUV128D128KU040Config")
        self.assertEqual(got.accelerators["saturn-opu"]["opu_cells"], 256)

    def test_a_plain_rocket_has_no_accelerator(self) -> None:
        self.assertEqual(_elaborated("RocketKU040OspiConfig").accelerators, {})

    def test_scratchpad_memory_is_recorded_as_disabled(self) -> None:
        """The KU040 shell wires no DRAM, and the record has to say so."""
        got = _elaborated("Q31Ws32x32AccGemminiKU040Config")
        self.assertTrue(got.memory)
        self.assertTrue(all(m.status == "disabled" for m in got.memory))

    def test_identity_is_stable_and_distinguishes_configs(self) -> None:
        first = _elaborated("GemminiKU040Config")
        again = _elaborated("GemminiKU040Config")
        other = _elaborated("Q31Ws32x32AccGemminiKU040Config")
        self.assertEqual(first.identity_id, again.identity_id)
        self.assertNotEqual(first.identity_id, other.identity_id)

    def test_round_trips_through_json(self) -> None:
        got = _elaborated("Q31Ws32x32AccGemminiKU040Config")
        again = fp.Fingerprint.from_dict(got.as_dict())
        self.assertEqual(got.identity_id, again.identity_id)
        self.assertEqual(got.clock_mhz, again.clock_mhz)


class IdentityTests(unittest.TestCase):
    def _fingerprint(self, **overrides):
        base = dict(
            evidence="declared",
            harts=({"isa": "rv64gc", "has_vector": False, "vlen_bits": None},),
            accelerators={},
            memory=(),
            clocks_mhz={"sbus_clock": 50.0},
        )
        base.update(overrides)
        return fp.Fingerprint(**base)

    def test_clock_is_excluded_from_identity(self) -> None:
        """Cycles are invariant to frequency; only wall time is not."""
        slow = self._fingerprint(clocks_mhz={"sbus_clock": 50.0})
        fast = self._fingerprint(clocks_mhz={"sbus_clock": 1000.0})
        self.assertEqual(slow.identity_id, fast.identity_id)

    def test_a_clock_only_difference_still_counts_as_exact(self) -> None:
        slow = self._fingerprint(clocks_mhz={"sbus_clock": 50.0})
        fast = self._fingerprint(clocks_mhz={"sbus_clock": 1000.0})
        divergences = fp.compare(fast, slow)
        self.assertEqual([d.field for d in divergences], ["clock_mhz"])
        self.assertEqual(divergences[0].affects, "wall-time")
        self.assertEqual(fp.agreement(divergences), "exact")

    def test_mixed_bus_clocks_refuse_to_name_one_frequency(self) -> None:
        mixed = self._fingerprint(clocks_mhz={"sbus_clock": 50.0, "mbus_clock": 100.0})
        self.assertIsNone(mixed.clock_mhz)

    def test_hart_count_difference_is_cycle_affecting(self) -> None:
        one = self._fingerprint()
        two = self._fingerprint(
            harts=(
                {"isa": "rv64gc", "has_vector": False, "vlen_bits": None},
                {"isa": "rv64gc", "has_vector": False, "vlen_bits": None},
            )
        )
        divergences = fp.compare(two, one)
        self.assertIn("harts", [d.field for d in divergences])
        self.assertEqual(fp.agreement(divergences), "extrapolated")

    def test_g_expands_so_a_shorthand_isa_stays_comparable(self) -> None:
        """``rv64gcv`` and ``rv64imafdcv...`` should not look like they disagree
        about every base extension."""
        shorthand = self._fingerprint(
            harts=({"isa": "rv64gc", "has_vector": False, "vlen_bits": None},)
        )
        spelled = self._fingerprint(
            harts=(
                {"isa": "rv64imafdc_zicsr_zifencei", "has_vector": False, "vlen_bits": None},
            )
        )
        self.assertEqual(fp.compare(shorthand, spelled), [])

    @unittest.skipUnless(
        (Path(__file__).resolve().parents[1] / "dse" / "profile-targets").is_dir(),
        "dse/profile-targets lives on main; not on the setup branch",
    )
    def test_declared_profile_target_is_loadable_and_diverges_from_the_config(self) -> None:
        from tools.rb import xpurt

        profile = xpurt.load_profile_target("firesim_rocket_saturn")
        self.assertIsNotNone(profile, "the FireSim profile-target descriptor is missing")
        self.assertEqual(profile.hart_count, 2)
        self.assertEqual(profile.clock_mhz, 1000.0)

        config = _elaborated("Q31Ws32x32AccGemminiKU040Config")
        divergences = fp.compare(profile, config)
        fields = {d.field for d in divergences}
        # The four divergences the flow has to account for, plus the clock.
        self.assertIn("harts", fields)        # dual-core profile, single-hart config
        self.assertIn("vector", fields)       # RVV profile, no vector unit at all
        self.assertIn("accelerators", fields)  # mesh shape unverified on the profile side
        self.assertIn("memory", fields)       # L2+DDR vs a disabled scratchpad
        self.assertIn("clock_mhz", fields)
        self.assertEqual(fp.agreement(divergences), "extrapolated")


class InstanceCountTests(unittest.TestCase):
    def test_missing_modules_return_none_rather_than_zero(self) -> None:
        directory = GENERATED / "chipyard.fpga.ku040.KU040Harness.RocketKU040OspiConfig"
        if not directory.is_dir():
            self.skipTest("RocketKU040OspiConfig has not been elaborated")
        self.assertIsNone(
            fp.count_instances(directory / "gen-collateral", "KU040Harness", "MacUnit")
        )


VERILATOR_GENERATED = (
    Path(__file__).resolve().parents[1]
    / "backends" / "chipyard" / "sims" / "verilator" / "generated-src"
)


class AcceleratorFeatureTests(unittest.TestCase):
    """Stripping a Gemmini feature has to move `identity_id`.

    `has_loop_conv` and `mvin_scale_args` remove whole datapaths without
    touching the mesh, so before these were recorded the LoopConv-free FireSim
    probe and the stock one both hashed to 4ddc46a5aae3 -- which would let
    `rb dse join` pair a latency measurement taken on one machine with an area
    record from the other and call the result exact.
    """

    def _collateral(self, name: str) -> Path:
        directory = VERILATOR_GENERATED / f"chipyard.harness.TestHarness.{name}" / "gen-collateral"
        if not directory.is_dir():
            self.skipTest(f"{name} has not been elaborated in this tree")
        return directory

    def test_stock_gemmini_records_no_disabled_features(self) -> None:
        got = fp.detect_accelerators(
            self._collateral("FireSimRiskyBirdGemminiQ31SaturnV128D128Fp16FullTargetConfig"),
            top="ChipTop",
        )
        self.assertEqual(got["gemmini"]["mesh_cells"], 1024)
        # Absent, not empty: the key has to stay off the stock shape or every
        # identity_id already on record would move.
        self.assertNotIn("disabled_features", got["gemmini"])

    def test_stripped_gemmini_records_both_missing_features(self) -> None:
        got = fp.detect_accelerators(
            self._collateral(
                "FireSimRiskyBirdGemminiQ31SaturnV128D128Fp16NoMvinScaleNoLoopConvTargetConfig"
            ),
            top="ChipTop",
        )
        # Same mesh, which is exactly why mesh_cells alone cannot separate them.
        self.assertEqual(got["gemmini"]["mesh_cells"], 1024)
        self.assertEqual(got["gemmini"]["disabled_features"], ["loop_conv", "mvin_scale"])

    def test_the_two_probes_no_longer_share_an_identity(self) -> None:
        stock = self._collateral(
            "FireSimRiskyBirdGemminiQ31SaturnV128D128Fp16FullTargetConfig"
        ).parent
        stripped = self._collateral(
            "FireSimRiskyBirdGemminiQ31SaturnV128D128Fp16NoMvinScaleNoLoopConvTargetConfig"
        ).parent
        a = fp.from_generated_src(stock, top="ChipTop")
        b = fp.from_generated_src(stripped, top="ChipTop")
        self.assertNotEqual(a.identity_id, b.identity_id)
        # The stock machine keeps the id already cited in
        # docs/firesim_u250_benchmarking.md and in dse/ descriptors.
        self.assertEqual(a.identity_id, "4ddc46a5aae3")

    def test_a_feature_key_is_omitted_when_the_module_is_present(self) -> None:
        """Hermetic, so the invariant is checked even without an elaborated tree."""
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            for module in ("Mesh", "MeshWithDelays", "MacUnit", "LoopConv", "ScalePipe"):
                (directory / f"{module}.sv").write_text("")
            self.assertEqual(fp.disabled_features({p.stem for p in directory.iterdir()},
                                                  (("loop_conv", "LoopConv"),
                                                   ("mvin_scale", "ScalePipe"))), {})
            (directory / "LoopConv.sv").unlink()
            self.assertEqual(fp.disabled_features({p.stem for p in directory.iterdir()},
                                                  (("loop_conv", "LoopConv"),
                                                   ("mvin_scale", "ScalePipe"))),
                             {"disabled_features": ["loop_conv"]})


if __name__ == "__main__":
    unittest.main()
