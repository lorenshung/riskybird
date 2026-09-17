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

## Two variants, and the choice between them is yours

| directory | accepted at | conv2d_s8 | measured end-to-end error |
| --- | --- | --- | --- |
| `dronet/int8/gemmini_q31` | bit-exact | scalar reference | **0** |
| `dronet/int8/gemmini_q31-drift2` | 2 LSB (`--drift-atol 2`) | **Gemmini** `gemmini_tiled_conv` | **2** |

The gate's per-op results for dronet / int8 / gemmini_q31, measured on spike
against an all-reference baseline — each kernel swapped in alone, so these are
per-kernel facts and not collateral from one bad neighbour:

| op | algorithm | result |
| --- | --- | --- |
| `conv2d_s8` | `gemmini_tiled_conv` | misses exact by **2 LSB** |
| `conv2d_s8` | `gemmini_im2col_full_C` | **broken**, 56 LSB |
| `linear_s8` | `gemmini_tiled_matmul` | **broken**, 56 LSB |
| `relu_s8` | `gemmini_resadd_relu` | **broken**, 56 LSB (archived; `swar` used) |
| `add_s8`, `batchnorm2d_s8`, `maxpool2d_s8`, `sigmoid_s8` | curated | exact, 0 |

### Why the 2 LSB is worth having

`conv2d_s8` dominates DroNet, and the difference between rejecting and accepting
that drift is not marginal:

```
conv2d_s8   460,351,223 cycles   scalar reference   (bit-exact variant)
conv2d_s8     1,989,282 cycles   gemmini_tiled_conv (drift2 variant)
```

231x on the op that is ~99% of the network. The first hardware run of the
bit-exact variant took 15.79 s; the drift2 variant should land near a fifth of a
second.

**This is not the mistake that produced wrong numbers earlier.** That run had no
gate at all and used `gemmini_im2col_full_C`, the 56-LSB kernel, with nothing
measuring it. Here the gate runs, measures the error, and accepts a bound that
is recorded in `.rb-codegen.json` and printed at build time. The difference is
between a known 2 and an unknown anything.

Whether 2 LSB is acceptable is a judgement about the airframe, not about the
tooling. DroNet's outputs are a steering angle and a collision probability; 2 of
255 is ~0.8% on each. `linear_s8`, which produces those two numbers, stays on
scalar reference in both variants, so the drift enters through the convolution
stack rather than the final layer.

`linear_s8` and `gemmini_im2col_full_C` are broken by 56 LSB and no tolerance
setting should be used to admit them. Fixing those is kernel work.
