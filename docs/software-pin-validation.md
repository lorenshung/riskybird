# Additional software pin validation

Tested on 2026-09-28, from parent `integration` at `fe871f8`:

| Component | Exact revision |
| --- | --- |
| ModelBlaster, root and nested | `80cec139799e6ecf3d8625b14cdaa78ba3bf596a` |
| zephyr-chipyard-sw | `35bf7c9becb88171304ced50ee487ac5adedfc36` |
| Zephyr RTOS | `5241922dd100c98dd953f6a6478f1e2d408de8aa` |
| ModelBlaster comparison baseline | `1293334703cd99fa80871448126bd33b17827551` |

No backend source, pin, existing checkout or upstream ref was changed. The
controlled diagnostic variations below used disposable checkouts or build
arguments. They are not fixes applied to the integration branches.

## Findings affecting integration

### Fresh code generation exposes the Gemmini header mismatch

Ran setup's existing `tools/modelblaster/dronet_gray_nhwc.py` recipe for
`gemmini_q31_rvv_softfp`, from empty output/cache directories. It extracts the
grayscale graph, generates C, compiles the full Zephyr harness and executes
candidate kernel selections in Spike against the model's golden outputs.
The tolerance remains the recipe's documented three integer LSBs.

| Experiment | Result |
| --- | --- |
| Original fork `1293334`, unchanged recipe | 8 numerical gates passed; max absolute error 3 |
| Merged pin `80cec13`, unchanged recipe | 0 passed, 9 candidate failures; errors 79, 10 or 29; generation exits nonzero |
| Merged pin, only fused conv+pool source restored to `1293334` in a disposable checkout | 8 passed; max absolute error 3 |
| Unchanged merged pin, simulator compiled against the header actually selected by the kernels | 8 passed; max absolute error 3 |

The original recipe compiles `ucb-bar/libgemmini` at
`5b1254a7dc72f757442280e7f28ef30542c78533` against
`per_config/default16x16/gemmini_params.h`. That header enables float input
scaling with `MVIN_SCALE_IDENTITY=1.0`. But `gemmini.h` includes
`include/gemmini_params.h`, bypassing that flat per-config header and selecting
the root fallback: no input scaling, `MVIN_SCALE_IDENTITY=0`.

The newly merged load-once conv+pool path exposes this inconsistency. Restoring
its previous source isolates the trigger; changing only the simulator's
parameter header restores all eight gates without changing the merged source.
The bad run fails closed with an activation-layout error after rejecting the
NHWC kernels. It does not emit a successfully verified replacement model.

This establishes a reproducibility defect in the default integration recipe,
not a demonstrated FPGA arithmetic defect. Fix parameter selection consistently
for hardware, generated kernels and the simulator before considering fresh
generation qualified. Do not simply raise the verification tolerance or revert
Dima's optimization based on the mismatched simulator result.

Compiler preprocessing also reconfirmed the existing DIM32 selection defect:

| Requested config | Effective DIM | Effective input scale |
| --- | ---: | --- |
| `default16x16` | 16 | disabled, identity 0 |
| `Q31Ws32x32AccGemminiSaturnV128D128Fp16FullKU040Config` | **16** | disabled, identity 0 |
| `q31ws_32x32_acc` | 32 | float, identity 1.0 |

### FC+DroNet defaults omit a newly required kernel include path

Built `samples/rose_fc_dronet` against the fresh model that passed with the
matched simulator. Its unchanged default kernel flags fail compilation:

```text
fatal error: conv2d_pool_loadonce.h: No such file or directory
```

The header lives in `modelblaster/kernels/gemmini_q31_rvv`. The existing
committed fixture contains the old fused convolution, so earlier successful
builds did not exercise this new dependency. A diagnostic build adds that
directory to the otherwise unchanged `MODELBLASTER_KERNEL_CFLAGS` and **passes**
(RAM 1,953,152 bytes). The sample defaults and its helper script both need review
when applying the actual fix.

## Checks that passed

- Both committed grayscale DroNet artifacts (`gemmini_q31_rvv_softfp` and
  `gemmini_q31_rvv_zve64x`) built with the merged ModelBlaster harness and merged
  RTOS, then executed in Spike with max absolute error **3**. These are numerical
  checks on seeded grayscale performance fixtures, not trained-model accuracy
  measurements or cycle-accurate performance tests.
- Regenerating with the original ModelBlaster pin produced byte-identical
  `model.c`, `kernels.c`, `weights.c`, `buffers.c`, `test_io.h` and `test_io.S`
  to the committed soft-float artifact.
- Four additional FC+DroNet firmware builds passed with the committed artifact:
  RoCC/preemptible, RoCC/atomic + camera DMA/raw telemetry, FP16 controller, and
  optical flow enabled. All were compiled with motors inhibited; none ran on
  hardware. The raw-UART clock mismatch remains unresolved.
- The actual command parser and camera/UART ownership host tests both passed.
- Placement opt-in built with `MB_CONV_PIN=1`, `MB_CONV_PIN_OFFSET=128`.
  ELF inspection found the nonempty `kernel_conv2d_s8_dronet` at `0x80001080`,
  exactly 128 bytes into its 4096-byte page. This validates linking, not an
  FPGA speed improvement.
- Setup's no-floating-point ISA scan passed on both RoCC images, the optical
  flow image and the placement image.

The FP16 controller image contains **125 vector floating-point instructions**
and therefore fails setup's current no-floating-point ISA guard. It compiles,
but is outside the qualified setup path. Its FP/vector enablement and target
compatibility need separate runtime validation; compilation is not sufficient.

## Expanded host suite

Ran the configured ModelBlaster tests plus `benchmarks/tests`, with the real
RISC-V compiler enabled: **227 passed, 5 failed, 8 skipped**.

- Four failures are the previously recorded `rvv_x60` conv/depthwise weight
  layout mismatch, unchanged from the original fork.
- The newly exercised failure is
  `IngestQueryTests.test_hetero_runs_are_skipped`: profile ingestion includes a
  heterogeneous run where the test expects exclusion. Repeating that file on
  `1293334` gives the same failure (six other cases pass).
- Eight skips require absent generated graph/schedule fixtures. The cross
  compiler cases ran rather than skipping.
- The separately invoked synthetic benchmark dashboard/aggregation harness
  passed, including 90 populated metric rows. It uses fabricated benchmark
  records and makes no model-performance claim.

## Reproduction and evidence

Validation reused the installed `zephyr-new` environment, Zephyr SDK
1.0.0-beta1 and Chipyard's installed Spike/GCC. Exact west dependency revisions
remain those recorded in the merged backend's
`docs/integration-20260928-dependencies.tsv`; this is not a clean network-only
bootstrap test.

```sh
# In the pinned ModelBlaster checkout, with its src directory first on PYTHONPATH:
PYTHONPATH=src CROSS=<riscv-tools>/bin/riscv64-unknown-elf- \
  <zephyr-new>/bin/python -m pytest tests pipeline/tests benchmarks/tests -q -ra
PYTHONPATH=src <zephyr-new>/bin/python -m modelblaster.benchmarks.tests.test_harness_e2e

# In the setup-derived parent, using a fresh scratch directory:
RB_RISCV_TOOLS=<chipyard-riscv-tools> python3 tools/modelblaster/dronet_gray_nhwc.py \
  --target gemmini_q31_rvv_softfp --work <scratch>

# In the pinned software backend:
python3 -m unittest discover -s tests -p test_fc_merge.py -v
```

Host-local evidence is in `/tmp/rb-pin-validation/`: `modelblaster-tests.log`,
`baseline-profile-db.log`, `benchmark-harness.log`, `build-results.json`,
`image-checks.json`, `header-checks.json`, `numerical-results.json`,
`committed-model-results.json`, the per-build and Spike logs, and diagnostic
scripts recording the exact invocations. The failed and successful generator
runs have separate directories and logs. These scratch files are not clone
dependencies.

Before promoting these pins as a working upstream integration, resolve the
header/simulator selection and fresh-model include path, then repeat those
checks without diagnostic overrides. FPGA numerical output, sustained
FC/inference concurrency, vector-context stability, sensors, camera and flight
remain untested with these exact pins. Publication to ucb-bar remains Loren's
decision.
