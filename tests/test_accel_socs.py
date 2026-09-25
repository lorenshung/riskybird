"""The accelerator-shell contracts: the post-link ISA scan, and where a picked
kernel's source is found.

Pure host tests. Nothing here needs Chipyard, a toolchain or a board.
"""

from __future__ import annotations

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from tools.rb import modelblaster as mb
from tools.rb.boards import FP16_SCALAR_FPU_CONFIGS, INT_NOPERMUTE_VECTOR_CONFIGS
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


if __name__ == "__main__":
    unittest.main()
