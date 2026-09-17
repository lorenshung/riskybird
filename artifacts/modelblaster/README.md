# Verified ModelBlaster codegen, published for machines that cannot verify

ModelBlaster picks a curated kernel only if it passes a per-op check against the
scalar reference. That check *runs* the kernel, which for a RISC-V accelerator
target means spike — and spike ships with the **Chipyard** backend while codegen
runs in the **Zephyr** one. A bench-tier machine has no Chipyard at all, so it
cannot run the gate, and `rb modelblaster build` refuses rather than trusting a
kernel's self-declared accuracy class.

This directory is the way out: codegen run on a silicon-tier host with the gate
working, committed so a Mac can build the same image without a simulator.

    rb modelblaster build --board arty200t \
        --config RocketArty200TDroneGemmini16At40Config \
        --gen-dir artifacts/modelblaster/dronet/int8/gemmini_q31

`.rb-codegen.json` records that verification ran and where the spike that ran it
came from. Regenerating on a machine that cannot verify overwrites the marker
and the refusal returns, which is the intent.

## What the gate actually rejected, for dronet / int8 / gemmini_q31

Measured, not assumed — these are the per-op results from the run that produced
this directory:

| op | Gemmini algorithm | result |
| --- | --- | --- |
| `conv2d_s8` | `gemmini_tiled_conv` | **FAIL** max_abs_err 2 |
| `conv2d_s8` | `gemmini_im2col_full_C` | **FAIL** max_abs_err 56 |
| `linear_s8` | `gemmini_tiled_matmul` | **FAIL** max_abs_err 56 |
| `relu_s8` | `gemmini_resadd_relu` | **FAIL** max_abs_err 56 |
| `add_s8` | `gemmini_resadd` | PASS, err 0 |
| `batchnorm2d_s8` | `direct` | PASS, err 0 |
| `maxpool2d_s8` | `gemmini_tiled_conv_pool` | PASS, err 0 |
| `sigmoid_s8` | `direct` | PASS, err 0 |

So **conv2d_s8 and linear_s8 have no verified Gemmini kernel on this
configuration** and fall back to scalar reference. Read that before quoting a
speedup: conv2d dominates DroNet, so a *correct* run is much closer to the
scalar timing than to the 296 ms an ungated run reported. The ungated run was
fast because it was running the kernel that fails by 56 LSB.
