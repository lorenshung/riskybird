"""Regression tests for deriving software-flow descriptors from a config.

The property under test throughout is that nothing is keyed on a config name.
Each case either runs against real elaboration output or constructs a
fingerprint directly, so a config written tomorrow is covered by the same rules
as the ones that exist today.
"""

from __future__ import annotations

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from tools.rb import fingerprint as fp
from tools.rb import soc
from tools.rb.boards import get_board


def _elaborated(config: str):
    board = get_board("ku040").with_config(config)
    if not board.generated_dts.is_file():
        raise unittest.SkipTest(f"{config} has not been elaborated in this tree")
    return fp.for_board(board)


def _fingerprint(harts, accelerators=None, **overrides):
    base = dict(
        evidence="elaborated",
        harts=tuple(harts),
        accelerators=accelerators or {},
        memory=(),
        clocks_mhz={"sbus_clock": 50.0},
        source={},
    )
    base.update(overrides)
    return fp.Fingerprint(**base)


def _hart(index=0, isa="rv64imafdc_zicsr", vector=False, vlen=None):
    return {"index": index, "isa": isa, "has_vector": vector, "vlen_bits": vlen}


class SingleHartTests(unittest.TestCase):
    """Accelerators attach to harts; they do not create them."""

    CONFIGS = (
        "RocketKU040OspiConfig",
        "SaturnKU040OspiConfig",
        "GemminiKU040Config",
        "Q31Ws32x32AccGemminiKU040Config",
        "SaturnOPUV128D128KU040Config",
    )

    def test_every_ku040_config_is_one_class_of_one_core(self) -> None:
        for config in self.CONFIGS:
            with self.subTest(config=config):
                got = soc.plan(_elaborated(config), config)
                self.assertEqual(len(got.classes), 1)
                self.assertEqual(got.machines, {"CPU_P": 1})
                self.assertNotIn("CPU_E", got.labels)

    def test_scalar_vector_and_matrix_coexist_on_one_hart(self) -> None:
        """SaturnOPU is a vector unit and a matrix engine on a single Rocket."""
        got = soc.plan(_elaborated("SaturnOPUV128D128KU040Config"), "x")
        only = got.classes[0]
        self.assertEqual(only.cores, 1)
        self.assertTrue(only.has_vector)
        self.assertEqual(only.vlen_bits, 128)
        self.assertIn("saturn-opu", only.accelerators)

    def test_slug_distinguishes_mesh_size_but_not_config_name(self) -> None:
        small = soc.plan(_elaborated("GemminiKU040Config"), "a").slugs[0]
        large = soc.plan(_elaborated("Q31Ws32x32AccGemminiKU040Config"), "b").slugs[0]
        self.assertNotEqual(small, large)
        self.assertIn("gemmini256", small)
        self.assertIn("gemmini1024", large)

    def test_slug_is_stable_across_calls(self) -> None:
        first = soc.plan(_elaborated("GemminiKU040Config"), "a").slugs[0]
        again = soc.plan(_elaborated("GemminiKU040Config"), "a").slugs[0]
        self.assertEqual(first, again)


class BackendGateTests(unittest.TestCase):
    def test_a_vector_profile_on_a_hart_with_no_vector_unit_is_refused(self) -> None:
        """The failure name matching cannot catch.

        Q31Ws32x32AccGemminiKU040Config elaborates without any zvl*, yet the
        shipped pairing hands its E-core a V256D128_rvv profile.
        """
        got = soc.plan(_elaborated("Q31Ws32x32AccGemminiKU040Config"), "q31")
        rejected = soc.check_backends(got, {"cpu_p": "V256D128_rvv"})
        self.assertTrue(rejected)
        self.assertEqual(rejected[0].reason, "no vector unit")

    def test_a_vector_profile_on_a_vector_hart_is_accepted(self) -> None:
        got = soc.plan(_elaborated("SaturnKU040OspiConfig"), "saturn")
        self.assertEqual(soc.check_backends(got, {"cpu_p": "V256D128_rvv"}), [])

    def test_requesting_a_core_the_soc_lacks_is_refused(self) -> None:
        got = soc.plan(_elaborated("GemminiKU040Config"), "gemmini")
        rejected = soc.check_backends(got, {"cpu_e": "anything"})
        self.assertEqual(rejected[0].reason, "no such core")


class MultiHartTests(unittest.TestCase):
    """Only reachable with constructed fingerprints until a dual config exists."""

    def _collateral(self, tmp: Path, modules: dict[str, str]) -> Path:
        """Write a minimal gen-collateral tree."""
        out = tmp / "gen-collateral"
        out.mkdir(parents=True)
        for name, body in modules.items():
            (out / f"{name}.sv").write_text(f"module {name}(\n);\n{body}\nendmodule\n")
        return out

    def test_symmetric_harts_collapse_to_one_class_with_two_cores(self) -> None:
        """Dual Saturn: two Rockets each with its own vector unit, one class."""
        with TemporaryDirectory() as tmp:
            collateral = self._collateral(Path(tmp), {
                "Top": "  RocketTile tile_0 (\n  );\n  RocketTile tile_1 (\n  );",
                "RocketTile": "",
            })
            classes = soc.machine_classes(_fingerprint(
                [_hart(0, "rv64imafdcv_zvl256b", True, 256),
                 _hart(1, "rv64imafdcv_zvl256b", True, 256)],
                source={"gen_collateral": str(collateral), "top": "Top"},
            ))
        self.assertEqual(len(classes), 1)
        self.assertEqual(classes[0].cores, 2)
        self.assertEqual(classes[0].harts, (0, 1))
        self.assertTrue(classes[0].has_vector)

    def test_asymmetric_harts_attribute_the_accelerator_to_its_own_tile(self) -> None:
        """Gemmini on hart 0 only: two tile modules, two classes."""
        with TemporaryDirectory() as tmp:
            collateral = self._collateral(Path(tmp), {
                "Top": "  RocketTile tile_0 (\n  );\n  RocketTile_1 tile_1 (\n  );",
                "RocketTile": "  MeshWithDelays mesh (\n  );",
                "RocketTile_1": "",
                "MeshWithDelays": "  Mesh mesh (\n  );",
                "Mesh": "  MacUnit mac_0 (\n  );\n  MacUnit mac_1 (\n  );",
                "MacUnit": "",
            })
            classes = soc.machine_classes(_fingerprint(
                [_hart(0), _hart(1)],
                source={"gen_collateral": str(collateral), "top": "Top"},
            ))
        self.assertEqual(len(classes), 2)
        by_hart = {c.harts: c for c in classes}
        self.assertEqual(by_hart[(0,)].accelerators["gemmini"]["mesh_cells"], 2)
        self.assertEqual(by_hart[(1,)].accelerators, {})

    def test_tile_count_disagreeing_with_the_dts_is_refused(self) -> None:
        """A wrong topology yields a benchmark for a machine nobody built."""
        with TemporaryDirectory() as tmp:
            collateral = self._collateral(Path(tmp), {
                "Top": "  RocketTile tile_0 (\n  );",
                "RocketTile": "",
            })
            with self.assertRaises(soc.SocError) as caught:
                soc.machine_classes(_fingerprint(
                    [_hart(0), _hart(1), _hart(2)],
                    source={"gen_collateral": str(collateral), "top": "Top"},
                ))
        self.assertIn("Refusing to guess", str(caught.exception))

    def test_multi_hart_without_collateral_is_refused(self) -> None:
        with self.assertRaises(soc.SocError) as caught:
            soc.machine_classes(_fingerprint(
                [_hart(0), _hart(1)], source={}
            ))
        self.assertIn("attributed", str(caught.exception))

    def test_more_than_two_classes_is_refused_not_truncated(self) -> None:
        classes = [
            soc.MachineClass(slug=f"k{i}", cores=1, harts=(i,), isa="rv64imafdc",
                             has_vector=False, vlen_bits=None, accelerators={})
            for i in range(3)
        ]
        original = soc.machine_classes
        soc.machine_classes = lambda _fp: classes  # type: ignore[assignment]
        try:
            with self.assertRaises(soc.SocError) as caught:
                soc.plan(_fingerprint([_hart(0)]), "three-kinds")
        finally:
            soc.machine_classes = original  # type: ignore[assignment]
        self.assertIn("CPU_P, CPU_E", str(caught.exception))

    def test_no_harts_is_refused(self) -> None:
        with self.assertRaises(soc.SocError):
            soc.machine_classes(_fingerprint([]))


class RegistryTests(unittest.TestCase):
    OPS = ("conv2d_s8", "maxpool2d_s8", "silu_s8")

    def test_hart_ids_come_from_the_dts(self) -> None:
        """They pin the runtime worker, so they must match the bitstream."""
        got = soc.plan(_elaborated("Q31Ws32x32AccGemminiKU040Config"), "q31")
        registry = soc.cores_registry(got, self.OPS)
        self.assertEqual(len(registry["cores"]), 1)
        self.assertEqual(registry["cores"][0]["harts"], [0])

    def test_every_core_can_run_every_op(self) -> None:
        """A Rocket executes anything; the accelerator only makes some ops fast.

        The shipped registries list an accelerator's fast paths as its whole
        capability set, which would make a single-hart Gemmini SoC look unable
        to run a workload it runs correctly, just slowly.
        """
        got = soc.plan(_elaborated("GemminiKU040Config"), "gemmini")
        registry = soc.cores_registry(got, self.OPS)
        for core in registry["cores"]:
            with self.subTest(core=core["name"]):
                self.assertEqual(set(core["capabilities"]), set(self.OPS))

    def test_kind_is_the_slug_so_it_matches_the_profile_directory(self) -> None:
        got = soc.plan(_elaborated("GemminiKU040Config"), "gemmini")
        registry = soc.cores_registry(got, self.OPS)
        self.assertEqual(registry["cores"][0]["kind"], got.profile_hw["cpu_p"])


class SlugTests(unittest.TestCase):
    def test_accelerator_shape_changes_the_slug(self) -> None:
        a = soc.machine_slug(False, None, {"gemmini": {"mesh_cells": 256}}, "rv64imafdc")
        b = soc.machine_slug(False, None, {"gemmini": {"mesh_cells": 1024}}, "rv64imafdc")
        self.assertNotEqual(a, b)

    def test_isa_detail_changes_the_slug_even_when_the_label_matches(self) -> None:
        """Bitmanip and zfh change cycles without changing the readable part."""
        a = soc.machine_slug(False, None, {}, "rv64imafdc_zicsr")
        b = soc.machine_slug(False, None, {}, "rv64imafdcb_zicsr_zba_zbb")
        self.assertTrue(a.startswith("novec-"))
        self.assertTrue(b.startswith("novec-"))
        self.assertNotEqual(a, b)

    def test_vector_width_is_visible_in_the_name(self) -> None:
        self.assertTrue(soc.machine_slug(True, 256, {}, "x").startswith("vec256-"))
        self.assertTrue(soc.machine_slug(False, None, {}, "x").startswith("novec-"))


if __name__ == "__main__":
    unittest.main()
