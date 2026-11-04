"""The accelerator-shell contracts: the post-link ISA scan, the published
DroNet codegen for the Gemmini+Saturn shells, where `rb modelblaster generate`
writes, and the shells' Gemmini header layout.

Pure host tests. Nothing here needs Chipyard, a toolchain or a board.
"""

from __future__ import annotations

import json
import os
import subprocess
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from tools.rb import modelblaster as mb
from tools.rb.boards import FP16_SCALAR_FPU_CONFIGS, INT_NOPERMUTE_VECTOR_CONFIGS, ROOT
from tools.rb.cli import isa_contract_offences


def _listing(*insns: str, function: str = "kernel_x") -> str:
    """A minimal `objdump -d --no-show-raw-insn` body."""
    lines = ["", f"0000000080001000 <{function}>:"]
    for i, insn in enumerate(insns):
        lines.append(f"    80001{i:03x}:\t{insn}")
    return "\n".join(lines) + "\n"


class IsaContractScanTests(unittest.TestCase):
    def test_clean_integer_code_passes_both_contracts(self) -> None:
        text = _listing("addi\ta0,a0,1", "vsetvli\ta4,a4,e8,m1,ta,ma", "vle8.v\tv1,(a0)",
                        "fence\tiorw,iorw", "fence.i")
        self.assertEqual(isa_contract_offences(text, fp16=True, noperm=True), {})

    def test_fp16_shell_refuses_scalar_float(self) -> None:
        text = _listing("flw\tfa5,0(a5)", "fadd.s\tfa0,fa0,fa1", "frcsr\ta0", "c.fld\tfa0,0(a0)")
        found = isa_contract_offences(text, fp16=True, noperm=False)
        self.assertEqual(sum(found["scalar floating point"].values()), 4)

    def test_fp16_shell_refuses_vector_float(self) -> None:
        # FS is Off on a CONFIG_FPU=n image, and vector FP needs FS != Off.
        text = _listing("vfadd.vv\tv1,v2,v3", "vmfeq.vv\tv0,v1,v2")
        found = isa_contract_offences(text, fp16=True, noperm=False)
        self.assertEqual(sum(found["vector floating point"].values()), 2)

    def test_vfirst_is_an_integer_mask_instruction(self) -> None:
        text = _listing("vfirst.m\ta0,v0")
        self.assertEqual(isa_contract_offences(text, fp16=True, noperm=True), {})

    def test_nopermute_shell_refuses_permutes_and_whole_register_moves(self) -> None:
        text = _listing("vmv4r.v\tv4,v16", "vslidedown.vi\tv1,v2,1", "vrgather.vv\tv1,v2,v3")
        found = isa_contract_offences(text, fp16=False, noperm=True)
        self.assertEqual(sum(found["vector permute / whole-register move"].values()), 3)

    def test_nopermute_shell_keeps_its_scalar_fpu(self) -> None:
        text = _listing("fadd.d\tfa0,fa0,fa1")
        self.assertEqual(isa_contract_offences(text, fp16=False, noperm=True), {})

    def test_offences_name_their_function(self) -> None:
        found = isa_contract_offences(_listing("vmv4r.v\tv4,v16", function="kernel_linear_s8"),
                                      fp16=False, noperm=True)
        self.assertIn("kernel_linear_s8: vmv4r.v",
                      found["vector permute / whole-register move"])

    def test_the_benched_shells_are_listed(self) -> None:
        self.assertIn("RocketArty200TDroneGemminiSaturnFp16At35Config", FP16_SCALAR_FPU_CONFIGS)
        self.assertIn("RocketArty200TDroneGemminiSaturnIntAt35Config", INT_NOPERMUTE_VECTOR_CONFIGS)


class KernelPathTests(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = TemporaryDirectory()
        self.mb = Path(self._tmp.name) / "modelblaster"
        kernel = self.mb / "kernels" / "gemmini_q31_rvv" / "k_rvv.c"
        kernel.parent.mkdir(parents=True)
        kernel.write_text("#include <riscv_vector.h>\nvoid k(void) { __riscv_vle8_v_i8m1(0, 1); }\n")
        patcher = patch.object(mb, "MODELBLASTER", self.mb)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.addCleanup(self._tmp.cleanup)

    def test_relative_path_resolves_inside_the_backend(self) -> None:
        self.assertEqual(mb.resolve_kernel_path("kernels/gemmini_q31_rvv/k_rvv.c"),
                         self.mb / "kernels" / "gemmini_q31_rvv" / "k_rvv.c")

    def test_another_hosts_absolute_path_is_retried_from_kernels(self) -> None:
        foreign = "/elsewhere/checkout/backends/modelblaster/kernels/gemmini_q31_rvv/k_rvv.c"
        self.assertEqual(mb._engine_from_source(foreign), "vector")

    def test_a_path_that_resolves_nowhere_is_none(self) -> None:
        self.assertIsNone(mb.resolve_kernel_path("/nowhere/at/all.c"))


PUBLISHED = {
    # directory: (target, board config it was benched on)
    "gemmini_q31_rvv_softfp-gray-nhwc":
        ("gemmini_q31_rvv_softfp", "RocketArty200TDroneGemminiSaturnFp16At35Config"),
    "gemmini_q31_rvv_zve64x-gray-nhwc":
        ("gemmini_q31_rvv_zve64x", "RocketArty200TDroneGemminiSaturnIntAt35Config"),
}


class PublishedCodegenTests(unittest.TestCase):
    """The committed DroNet directories must build without a simulator."""

    def test_each_directory_is_complete_and_verified(self) -> None:
        for name, (target, _config) in PUBLISHED.items():
            with self.subTest(name=name):
                gen = ROOT / "artifacts" / "modelblaster" / "dronet" / "int8" / name
                for required in ("model.c", "kernels.c", "weights.c", "kernel_picks.json",
                                 mb.CODEGEN_MARKER):
                    self.assertTrue((gen / required).is_file(), required)
                self.assertEqual(mb.target_of(gen), target)
                status = mb.verification_status(gen)
                self.assertTrue(status and status.get("verified"))
                self.assertEqual(status.get("drift_atol"), 3)

    def test_picked_kernels_are_named_inside_the_backend(self) -> None:
        # Paths from the codegen host would not resolve on a clean install.
        for name in PUBLISHED:
            with self.subTest(name=name):
                gen = ROOT / "artifacts" / "modelblaster" / "dronet" / "int8" / name
                picks = json.loads((gen / "kernel_picks.json").read_text())["picks"]
                for op, pick in picks.items():
                    path = pick.get("path")
                    if path is not None:
                        self.assertTrue(path.startswith("kernels/"), f"{op}: {path}")

    def test_nopermute_directory_keeps_linear_on_the_scalar_reference(self) -> None:
        gen = ROOT / "artifacts/modelblaster/dronet/int8/gemmini_q31_rvv_zve64x-gray-nhwc"
        picks = json.loads((gen / "kernel_picks.json").read_text())["picks"]
        self.assertEqual(picks["linear_s8"]["source"], "reference")


class CodegenRootTests(unittest.TestCase):
    """`rb modelblaster generate` runs ModelBlaster's runner without writing into it."""

    def setUp(self) -> None:
        self._tmp = TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        tmp = Path(self._tmp.name)
        self.mb = tmp / "backends" / "modelblaster"
        self.root = tmp / "build" / "modelblaster"
        files = {
            ".git/HEAD": "ref: refs/heads/main\n",
            "pipeline/extract_graph.py": "",
            "kernels/gemmini_q31/k.c": "kernel\n",
            "examples/_run_lib.sh": "",
            "examples/lenet/run.sh": "",
            "examples/dronet/run.sh": "",
            "examples/dronet/int8/fq.csv": "",
            "examples/dronet/int8/cache/gemmini_q31/cached.c": "cached\n",
            "examples/dronet/int8/generated/stale.c": "",
            "examples/dronet/int8/build/stale.o": "",
            "examples/dronet/fp32/cache/rvv/a.c": "",
        }
        for rel, text in files.items():
            (self.mb / rel).parent.mkdir(parents=True, exist_ok=True)
            (self.mb / rel).write_text(text)
        for name, value in (("MODELBLASTER", self.mb), ("CODEGEN_ROOT", self.root)):
            patcher = patch.object(mb, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)

    def _backend_snapshot(self) -> dict[str, str]:
        return {str(p.relative_to(self.mb)): p.read_text()
                for p in sorted(self.mb.rglob("*")) if p.is_file()}

    def test_generated_dir_is_under_the_codegen_root(self) -> None:
        self.assertEqual(mb.generated_dir("dronet", "int8", "gemmini_q31"),
                         self.root / "examples/dronet/int8/generated/gemmini_q31")

    def test_runner_sees_the_codegen_root_as_its_repo_root(self) -> None:
        run_sh = mb._prepare_codegen_root("dronet", "int8", "gemmini_q31")
        self.assertEqual(run_sh, self.root / "examples/dronet/run.sh")
        # run.sh's own REPO_ROOT computation, which does not resolve links.
        repo_root = subprocess.run(
            ["bash", "-c", 'cd "$(dirname "$1")/../.." && pwd', "_", str(run_sh)],
            capture_output=True, text=True, check=True,
        ).stdout.strip()
        self.assertEqual(Path(repo_root), self.root)

    def test_layout_links_everything_but_the_runner_outputs(self) -> None:
        mb._prepare_codegen_root("dronet", "int8", "gemmini_q31")
        links = {
            "pipeline": "pipeline", "kernels": "kernels",
            "examples/_run_lib.sh": "examples/_run_lib.sh",
            "examples/lenet": "examples/lenet",
            "examples/dronet/run.sh": "examples/dronet/run.sh",
            "examples/dronet/int8/fq.csv": "examples/dronet/int8/fq.csv",
        }
        for rel, target in links.items():
            with self.subTest(rel=rel):
                self.assertTrue((self.root / rel).is_symlink())
                self.assertEqual(Path(os.readlink(self.root / rel)), self.mb / target)
        self.assertFalse((self.root / ".git").exists())
        for rel in ("examples", "examples/dronet", "examples/dronet/int8",
                    "examples/dronet/fp32", "examples/dronet/int8/cache/gemmini_q31"):
            with self.subTest(rel=rel):
                path = self.root / rel
                self.assertTrue(path.is_dir() and not path.is_symlink())
        for rel in ("examples/dronet/int8/generated", "examples/dronet/int8/build",
                    "examples/dronet/fp32/cache"):
            self.assertFalse((self.root / rel).exists(), rel)

    def test_kernel_cache_is_reseeded_from_the_pin_every_run(self) -> None:
        before = self._backend_snapshot()
        mb._prepare_codegen_root("dronet", "int8", "gemmini_q31")
        cache = self.root / "examples/dronet/int8/cache/gemmini_q31"
        (cache / "cached.c").write_text("rewritten by a run\n")
        (cache / "new.c").write_text("added by a run\n")
        mb._prepare_codegen_root("dronet", "int8", "gemmini_q31")
        self.assertEqual(sorted(p.name for p in cache.iterdir()), ["cached.c"])
        self.assertFalse((cache / "cached.c").is_symlink())
        self.assertEqual((cache / "cached.c").read_text(), "cached\n")
        self.assertEqual(self._backend_snapshot(), before)

    def test_a_target_without_a_tracked_cache_starts_empty(self) -> None:
        mb._prepare_codegen_root("dronet", "fp16", "rvv_f16")
        cache = self.root / "examples/dronet/fp16/cache/rvv_f16"
        self.assertTrue(cache.is_dir() and not any(cache.iterdir()))

    def test_links_follow_the_pin(self) -> None:
        (self.mb / "examples/added").mkdir()
        mb._prepare_codegen_root("dronet", "int8", "gemmini_q31")
        self.assertTrue((self.root / "examples/added").is_symlink())
        (self.mb / "examples/added").rmdir()
        mb._prepare_codegen_root("dronet", "int8", "gemmini_q31")
        self.assertFalse(os.path.lexists(self.root / "examples/added"))

    def test_a_real_file_in_the_way_is_not_deleted(self) -> None:
        (self.root / "examples/dronet").mkdir(parents=True)
        (self.root / "examples/dronet/run.sh").write_text("mine\n")
        with self.assertRaises(mb.ModelBlasterError):
            mb._prepare_codegen_root("dronet", "int8", "gemmini_q31")
        self.assertEqual((self.root / "examples/dronet/run.sh").read_text(), "mine\n")


class PerConfigHeaderTests(unittest.TestCase):
    """The shells' Gemmini headers use ModelBlaster's per_config/<name>/ layout.

    gemmini.h includes "include/gemmini_params.h" and the kernels include
    <gemmini_params.h>; with -isystem per_config/<name> first, both must find
    this shell's header. A directory with only the second spelling sends
    gemmini.h to ModelBlaster's default16x16 header without a word.
    """

    def test_both_spellings_name_the_same_header(self) -> None:
        base = ROOT / "hardware/modelblaster/cores/gemmini/include/per_config"
        configs = sorted(p for p in base.iterdir() if p.is_dir())
        self.assertTrue(configs)
        for config in configs:
            with self.subTest(config=config.name):
                header = config / "gemmini_params.h"
                self.assertTrue(header.is_file())
                self.assertTrue((config / "include" / "gemmini_params.h").samefile(header))


if __name__ == "__main__":
    unittest.main()
