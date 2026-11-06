# Developing the backends from riskybird-dev

How to change Chipyard, Zephyr, zephyr-chipyard-sw, ModelBlaster and the simulation
pieces from the riskybird-dev top level without moving a backend pin. If a pin does
have to move, section 3 covers it.


## 0. Rules

1. RiskyBird code lives in riskybird-dev (`hardware/`, `workloads/`, `tools/`). It never
   lives inside a backend checkout.
2. Backends are pinned at plain branch tips. riskybird-dev wraps them locally and commits
   nothing inside a backend checkout.
3. Upstream PRs are not required. A RiskyBird change to a backend goes on that repo's
   `rb-clean` branch. A **separate** riskybird-dev PR then bumps the pin. Never use a
   shared `dev` branch.
4. riskybird-dev `main` changes only through PRs to ucb-bar/riskybird.
5. Build and run the backend tools in a **standalone clone**, never in a linked worktree
   (see section 4).

## 1. The model

| Mechanism | What it is | Declared in | Applied / checked by |
|---|---|---|---|
| **pin** | gitlink in riskybird-dev: the exact backend commit | `.gitmodules` + the index | `git submodule update --init <path>` (never `--recursive`) |
| **overlay link** | symlink *inside* a backend that points back into riskybird-dev | `backends/overlays` (`LINK TARGET`) | `tools/backend-patches apply\|check\|revert` |
| **sbt overlay** | AutoPlugin that adds `hardware/**` source roots to Chipyard's sbt projects. It can also *shadow* a stock file that has the same relative path | `hardware/chipyard/project/RiskyBird.scala` (itself an overlay link) | sbt, when you build through `rb-fpga.mk` |
| **make wrapper** | wraps stock `fpga/Makefile`: adds the RiskyBird sub-projects and the area/impl flow | `hardware/chipyard/fpga/rb-fpga.mk` | `rb chipyard …`, `rb area` |
| **patch queue** | working-tree edits to a backend's own files, for generic fixes only | `hardware/chipyard/patches/<path inside backends/chipyard>/NNNN-*.patch` | `tools/backend-patches apply\|check\|revert` |
| **redirect** | nested duplicate submodule replaced by a symlink to the top-level backend | `backends/redirects` (`OWNER PATH TARGET`) | `tools/submodule-redirects apply\|check\|undo` |
| **build-time injection** | rb passes RiskyBird inputs on the CMake line: `-DRB_ROOT`, `-DDTS_ROOT=hardware/zephyr`, `-DDTC_OVERLAY_FILE`, `-DEXTRA_CONF_FILE`, `-DKCONFIG_ROOT`, `EXTRA_CFLAGS=-Ihardware/zephyr/include`, `-DTINYMPC=…`, `-DZEPHYR_EXTRA_MODULES=…` | `tools/rb/cli.py` (zephyr_build), `tools/rb/boards.py` | `rb build / debug / run` |
| **mirror root** | a RiskyBird-owned directory of links into a backend. The backend's script runs from there, so its outputs land outside the backend | `tools/rb/modelblaster.py` (`build/modelblaster`), `tools/rb/xpurt.py`, `tools/rb/rose.py` (`.rb/generated/rose-spike`) | the rb subcommand |

```
riskybird-dev (gitlinks = pins)
├── hardware/chipyard/{project,fpga,generators,patches}  ──links, sbt roots, queue──▶  backends/chipyard (+ nested rocket-chip, fpga-shells, gemmini, tacit, sims/firesim)
├── hardware/zephyr, workloads/, workloads/lib           ──CMake args at build──▶      backends/zephyr-chipyard-sw/zephyr_ws/zephyr
├── hardware/modelblaster, build/modelblaster (mirror)   ──links, -isystem──▶          backends/modelblaster
└── backends/redirects:  RoSE/soc/sim/chipyard, RoSE/…/xpu-rt/{hw/chipyard,zephyr-chipyard-sw},
                         zephyr-chipyard-sw/modelblaster, chipyard/generators/saturn ──▶ the top-level copy;
                         chipyard/software/zephyrproject/zephyr ──▶ left empty
```

**The fact everything else depends on:** the patch queue reaches only `backends/chipyard`
and the repos nested in it (`QUEUE_TARGET` is hard-coded in `tools/backend-patches`).
Zephyr, zephyr-chipyard-sw, ModelBlaster, RoSE and Saturn have **no queue**. A change to
their own source is a commit on a branch plus a pin bump. Overlay links could in
principle point into any backend, but today they point only into Chipyard.

Pins as of 2026-11-05 (check `git ls-tree HEAD backends/` for the current ones):

| Path | Repo / branch | Pin | Branch tip |
|---|---|---|---|
| `backends/chipyard` | ucb-bar/chipyard `main` + queue | 157dd69 | 157dd69 |
| `backends/zephyr-chipyard-sw` | ucb-bar/zephyr-chipyard-sw `rb-clean` | a0f2484 | a0f2484 |
| ↳ `zephyr_ws/zephyr` | ucb-bar/zephyr `rb-clean` | 5241922 | 5241922 |
| ↳ `samples/drone_control/tinympc` | ucb-bar/Accelerated-TinyMPC `zephyr` | (zcs pins it) | |
| `backends/modelblaster` | ucb-bar/ModelBlaster `rb-clean` | 5b78467 | 5b78467 |
| `backends/saturn` | CobbledSteel/saturn-vectors `saturn-vmem-be-gate` | e73d9bc | e73d9bc |
| `backends/RoSE` | ucb-bar/RoSE `rose-2-dev` | b6ac3e4 | b6ac3e4 |
| `hardware/pcb` | lorenshung/riskybird `riskybirdv3-layout` | e1a5a50 | e0322a3 |

## 2. "I want to change X." Where does it go?

### 2.1 Chipyard: configs, harnesses, IOBinders, RTL

| Change | Where |
|---|---|
| SoC or FPGA config, harness, binder, fragment | Scala under `hardware/chipyard/fpga/src/main/scala/<board>/` (sbt project `chipyard_fpga`), `hardware/chipyard/generators/chipyard/src/main/scala/` (`chipyard`), `hardware/chipyard/generators/firechip/chip/src/main/scala/` (`firechip`), `hardware/fpga-shells/src/main/scala/` (`fpga_shells`), `hardware/gemmini/{chipyard,src/main/scala}` (`chipyard` / `gemmini`). Keep the existing packages (`chipyard.fpga.arty200t…`): config names are fully qualified. |
| RiskyBird RTL / peripherals | `hardware/rtl/{esplink,uartdma,ospi,fcrocc}`. These are already roots of the `chipyard` project. FcRoCC blackbox resources go in `hardware/rtl/fcrocc/vsrc/` and appear on the classpath as `/vsrc/*`. |
| A new source root | Add it to `overlays` in `hardware/chipyard/project/RiskyBird.scala`. If it sits outside `hardware/{chipyard,rtl,fpga-shells/src,gemmini}`, also add it to the `find` in `rb_overlay_sources` in `rb-fpga.mk`, or the generator jar will not rebuild when it changes. |
| Rework a stock Scala file wholesale | **Shadow** it: put a file at the same path relative to an overlay root. Today the only shadows are the Arty100T/TE0712 harness, binders and configs, plus fpga-shells' Arty100T shell and MIG. The sbt task `riskybirdShadows` lists them. Every shadow is a manual merge on each Chipyard bump. |
| `DigitalTop` traits | `RbDigitalTop.scala` restates stock `DigitalTop`'s trait list and adds the OSPI and UART-DMA traits. `test_rb_digital_top_follows_the_pinned_digital_top` fails after a bump until you re-sync it. |
| New Vivado board | Four places: `hardware/fpga-shells/xilinx/<board>/`, a row in `backends/overlays` that links it under `backends/chipyard/fpga/fpga-shells/xilinx/`, a `SUB_PROJECT` block in `rb-fpga.mk`, and a `Board` in `tools/rb/boards.py`. |
| Area / impl flow | `hardware/chipyard/riskybird/{area.mk,tcl/}`, included by `rb-fpga.mk`. The stock Makefile carries **no** hook (`test_stock_makefile_has_no_riskybird_hook`). |
| A generic fix to a backend's own file that an overlay or shadow cannot express | Patch queue (below). RiskyBird-specific code never goes in a patch. |

Build: `rb chipyard elaborate|build --board arty200t --config <Class>`. This is the same as
`make -C backends/chipyard/fpga -f $PWD/hardware/chipyard/fpga/rb-fpga.mk SUB_PROJECT=… CONFIG=… verilog`.

**Patch queue.** Patches live at `hardware/chipyard/patches/<subpath>/NNNN-Title.patch`.
`<subpath>` mirrors the repo inside `backends/chipyard`: for example
`patches/generators/rocket-chip/` targets `backends/chipyard/generators/rocket-chip`, and
top-level files target Chipyard itself. Each patch is a plain description followed by a
`git diff` (no mbox headers). `test_every_queued_patch_explains_itself` requires more
than one line before the first `diff --git`. Say what the patch changes and why it is
generic. `git apply` applies the patches in name order. Delete a patch once the pinned
commit contains the fix.

Add a patch without editing the applied backend by hand. Work in a scratch clone of the
nested repo:

```sh
R=backends/chipyard/generators/rocket-chip
Q=$PWD/hardware/chipyard/patches/generators/rocket-chip
W=$(mktemp -d)/repo
git clone -q "$R" "$W" && git -C "$W" checkout -q --detach "$(git -C "$R" rev-parse HEAD)"
# one commit per queued patch, each carrying its own description as the message
for p in "$Q"/*.patch; do
  sed '/^diff --git/,$d' "$p" > "$W.msg"
  git -C "$W" apply "$p" && git -C "$W" add -A && git -C "$W" commit -q --cleanup=whitespace -F "$W.msg"
done
# ...edit in $W, write the description (title, blank line, why it is generic) to $W.msg, then:
git -C "$W" add -A && git -C "$W" commit -q --cleanup=whitespace -F "$W.msg"
{ git -C "$W" log -1 --format=%B; git -C "$W" diff HEAD~1 HEAD; } > "$Q/000N-Title.patch"
tools/backend-patches apply && tools/backend-patches check
```

To re-export any queue commit `C`, use the same pattern:
`{ git log -1 --format=%B C; git diff C~1 C; }`. A dry run of this round trip over the
five rocket-chip patches at their pin (ece7b9a) gave back every description verbatim.
Only `index` lines and one hunk offset changed, because the old patches were cut against
another base. Expect that noise the first time you refresh.

`apply` notices that the first k patches are already on the working tree and applies only
the new tail. That works only when the new file sorts **last**. To insert, edit or remove
an existing patch, run `tools/backend-patches revert` **first, while the old queue files
are still in place**. Then change the queue and run `apply`.

### 2.2 Zephyr: drivers, devicetree, Kconfig

| Change | Where |
|---|---|
| Devicetree for a board, shell or workload | `hardware/zephyr/<board>[-scratchpad].overlay`, `fpga-*.overlay`, `targets/<target>/target.{overlay,conf}`, `targets/<target or kind>/workloads/<workload>[-<mode>].{overlay,conf}`. For the overlay, the first match wins in this order: target `<w>-<mode>`, target `<w>`, kind `<w>-<mode>`, kind `<w>`. So a mode overlay *replaces* the base overlay, which is why the flight_controller overlays `#include` each other. For `.conf`, the base is applied and then the mode's (`boards.py: workload_overlay / workload_confs`). |
| DT bindings, vendor prefixes | `hardware/zephyr/dts/bindings/`. rb passes `-DDTS_ROOT=hardware/zephyr`. A workload built outside rb must append it itself, as `workloads/fc_dronet/CMakeLists.txt` does. |
| Kconfig symbols | `hardware/zephyr/Kconfig.fpga` (sourced by `Kconfig.workload`, which rb passes as `KCONFIG_ROOT` when the workload has no `Kconfig` of its own); fragments in `fpga.conf` and `ram-console.conf` |
| SoC headers (`pinctrl_soc.h`) | `hardware/zephyr/include/` (added through `EXTRA_CFLAGS -I`) |
| Shared driver-like code (camera, ToF, motor link, motor guard, flight recorder) | `workloads/lib/<lib>/{src,include/riskybird}`. Workloads compile it with `target_sources(... ${RB_ROOT}/workloads/lib/...)`; rb passes `-DRB_ROOT`. It is not a Zephyr module. |
| A workload / app | `workloads/<name>` or `workloads/{bringup,probes}/<name>`, registered in `WORKLOADS` in `tools/rb/boards.py`. Cache knobs reach the build through `RB_CMAKE_ARGS`; preprocessor knobs through `RB_EXTRA_CPPFLAGS`. |
| **Must** be a Zephyr commit | Anything in Zephyr's own tree: `arch/riscv` (for example `core/v.c`), in-tree drivers (`drivers/i2c/i2c_sifive.c`, `drivers/serial/uart_sifive.c`, `drivers/sensor/st/vl53l5x/…`), and the `boards/chipyard/riscv64` board (`chipyard_riscv64`). This goes on ucb-bar/zephyr `rb-clean`, and then takes the two-level bump in section 3. |

There is no queue and no `ignore=dirty` for Zephyr. To try a Zephyr edit before
committing it, edit `backends/zephyr-chipyard-sw/zephyr_ws/zephyr` **in a standalone
clone only**. riskybird-dev's `git status` shows the result as `M
backends/zephyr-chipyard-sw`. Never verify or PR from that state.

### 2.3 zephyr-chipyard-sw: samples, scripts, TinyMPC

- rb builds **no** zcs sample. It takes the toolchain, the SDK and the west workspace
  from zcs (`scripts/install_conda.sh`, `install_submodules.sh`, `activate_conda.sh`,
  `tools/miniforge3`, `tools-manual/`, `zephyr_ws/`), plus TinyMPC from
  `samples/drone_control/tinympc` (`FLIGHT_TINYMPC`, passed as `-DTINYMPC`).
- The flight controller is riskybird-dev's `workloads/flight_controller`, a fork of the
  RoSE sample `samples/rose_flight_controller`, which stays in zcs. FC changes go there.
  The zcs copy is changed only for RoSE/upstream purposes.
- A change to zcs scripts, samples or nested pins goes on ucb-bar/zephyr-chipyard-sw
  `rb-clean`, followed by a riskybird-dev bump. A TinyMPC change is one level deeper:
  Accelerated-TinyMPC, then zcs, then riskybird-dev.

### 2.4 ModelBlaster: codegen, kernels, harness

| Change | Where |
|---|---|
| Gemmini params for a shell | `hardware/modelblaster/cores/gemmini/include/per_config/<Config>/{,include/}gemmini_params.h` (copied from that config's elaboration). rb looks there before the backend. |
| RiskyBird core descriptions, examples, harness variants, models, recipes | `hardware/modelblaster/{cores,examples,harness*,models,riskybird}/`. `rb modelblaster generate` **cannot** run an example that exists only here (it refuses). |
| Generate sources | `rb modelblaster generate --model dronet --quant int8 --target gemmini_q31 [--drift-atol N] [--config <Class>]`. This runs the backend's `examples/<model>/run.sh` (`STAGE_ONLY=1`) from the mirror root `build/modelblaster`. Output goes to `build/modelblaster/examples/<model>/<quant>/generated/<target>/` with `.rb-codegen.json`. Nothing is written into `backends/modelblaster`. The gate needs Chipyard's spike (`backends/chipyard/.conda-env/riscv-tools`, or `RB_RISCV_TOOLS`) and torch in the Zephyr env. |
| Committed, verified sources | `workloads/fc_dronet/generated/<name>/` (with `.rb-codegen.json`). Build from them with `rb modelblaster build --model dronet --board arty200t --config <Class> --gen-dir workloads/fc_dronet/generated/<name>`. The gray NHWC sets are regenerated by `tools/modelblaster/dronet_gray_nhwc.py`. |
| Pipeline, kernels or harness behaviour | A commit on ucb-bar/ModelBlaster `rb-clean`, then a bump of `backends/modelblaster`. zcs also nests ModelBlaster, but that copy is redirected to `backends/modelblaster`, so rb never builds it. |

### 2.5 Simulation

- **In-repo RoSE flow (`rb rose doctor|build|…`, target `rose-spike`, `mpc --mode rose`).**
  It uses `backends/RoSE`, initialised **non-recursively**. RoSE's nested Chipyard and
  xpu-rt's Chipyard/zcs are redirected to the top-level backends, so Spike is built under
  `backends/chipyard/toolchains/riscv-tools/riscv-isa-sim`. rb puts the RoSE Zephyr module
  on the build with `-DZEPHYR_EXTRA_MODULES=backends/RoSE/soc/sw/zephyr-rose` and writes
  its modified Spike `sim.h` to `.rb/generated/rose-spike` rather than editing Spike.
  Target files: `hardware/zephyr/targets/rose-spike/`. RiskyBird glue: `tools/rb/rose.py`
  and `tools/rb/rose_sync.py`.
- **Digital twin (`lorenshung/riskybird-sim`, private).** RoSE is used unmodified at
  `ROSE_PIN` (b6ac3e4, the same as `backends/RoSE`). Airframe, envs, sensor models and
  scenarios live there. Its `scripts/build-guest.sh` builds riskybird-dev's
  `workloads/flight_controller` (`RISKYBIRD_DEV`, optionally `FC_REV`). It takes Zephyr
  and TinyMPC from its own zcs checkout at `GUEST_REV` f16695d (`ZCS_TOOLCHAIN`,
  `ZEPHYR_ROSE`), **not** from riskybird-dev's pin.
- Where changes go: FC behaviour goes in riskybird-dev `workloads/flight_controller`. Sim
  models and envs go in riskybird-sim. A generic RoSE change goes in RoSE (no `rb-clean`
  exists; ask first). When `backends/RoSE` moves, update riskybird-sim `ROSE_PIN` too.

### 2.6 Saturn and the PCB

`backends/saturn` is Dima's branch, redirected into `chipyard/generators/saturn`. Do not
commit to it; ask. `hardware/pcb` is not read by any build.

## 3. When a pin bump is needed, and how

A bump is needed when the change is to a backend's own source and the Chipyard queue
cannot carry it. That means any Zephyr, zcs, ModelBlaster, RoSE or Saturn source change,
or a newer upstream Chipyard.

1. **Land the backend commit** on its `rb-clean` branch (never `dev`).
   - Zephyr is a chain: a ucb-bar/zephyr `rb-clean` commit, then a ucb-bar/zephyr-chipyard-sw
     `rb-clean` commit that bumps `zephyr_ws/zephyr`, then the riskybird-dev bump of
     `backends/zephyr-chipyard-sw`.
   - Chipyard has no `rb-clean`. Bump to a newer ucb-bar/chipyard `main` and rebase the
     queue onto it.
2. **Move the checkout in a standalone clone**, on a topic branch:
   ```sh
   tools/backend-patches revert                          # Chipyard only, before moving it
   git -C backends/<b> fetch origin && git -C backends/<b> checkout --detach <sha>
   git add backends/<b>                                  # backend-patches reads the pin from the index
   ```
   - Update only the nested submodules you need, never with `--recursive`. For Chipyard,
     use `bash scripts/init-submodules-no-riscv-tools.sh --gemmini --tacit`, run from
     `backends/chipyard`.
   - Then run `tools/submodule-redirects apply`. Every nested `submodule update` or init
     script re-arms the redirects.
   - For Chipyard, rebase the queue commits from section 2.1 onto the new nested pins,
     re-export them under their existing names, re-sync `RbDigitalTop` and the shadowed
     files, and then run `tools/backend-patches apply`.
3. **Verify** (section 5). For a bump meant to be behaviour-neutral, the bar is
   byte-identical images.
4. **Open a riskybird-dev PR that only bumps the pin.** The exception is what the bump
   forces: a Chipyard queue refresh, the `RbDigitalTop` sync, or a shadow merge.
   `git diff --cached --submodule=log` must show only the intended gitlink(s). Put the
   check outputs and the image comparison in the PR body.

## 4. Setup for an agent

**Use a standalone clone, not a linked worktree.** Linked worktrees share `.git/config`.
`backend-patches apply` writes `submodule.<name>.ignore=dirty` there, and
`git submodule sync` rewrites submodule URLs there (it has repointed a fork before). Never
run `tools/backend-patches apply` or `git submodule sync` from a linked worktree. A
worktree is fine for edits that need no backend checkout (`tools/`, `tests/`, docs), but
build in a clone.

```sh
git clone git@github.com:ucb-bar/riskybird.git /scratch2/$USER/rb-<task> && cd $_
git switch -c <topic>
# Tiers are cumulative. Initialise only what you need, never --recursive.
git submodule update --init backends/zephyr-chipyard-sw backends/modelblaster   # bench (modelblaster is required, see the flags)
git submodule update --init backends/chipyard backends/saturn                   # silicon (Linux, Vivado)
git submodule update --init backends/RoSE                                       # sim (needs chipyard + zcs for its redirects)
source ./rb-env.sh                 # rb on PATH, RB_DTS_DIR / RB_BITSTREAM_DIR if ~/riskybird-{dts,bitstreams} exist
rb env setup zephyr                # zcs installers, then submodule-redirects apply
rb env setup chipyard              # redirects, Chipyard init (--gemmini --tacit), redirects, backend-patches apply, build-setup --skip-submodules
tools/submodule-redirects check && tools/backend-patches check
rb doctor
```

- `rb env setup <env>` returns immediately when the env already exists. After a bump, a
  branch switch, or any nested submodule command, run `tools/submodule-redirects apply`
  and then `tools/backend-patches apply` yourself, in that order.
- To avoid reinstalling toolchains, earlier clones symlinked them from an existing
  checkout. riskybird-sim's `build-guest.sh` links zcs `tools/miniforge3` and
  `tools-manual`, and copies `zephyr_ws/{bootloader,modules,tools,.west}`. These paths are
  ignored in zcs. `RB_RISCV_TOOLS` points ModelBlaster's gate at another Chipyard's spike.
  Treat linked envs as read-only.

## 5. Verification checklist before a PR

- [ ] `tools/submodule-redirects check` and `tools/backend-patches check` both exit 0.
- [ ] `PYTHONPATH=. python3 -m unittest discover -s tests -t tests` passes. Record
      `Ran N … skipped=M` and compare it with the base commit; the skip count depends on
      which backends and artifacts are present.
- [ ] If you touched `hardware/modelblaster`: from `hardware/modelblaster`, run
      `python -m unittest discover -s riskybird/tests -v` (needs torch/numpy/Pillow/torchvision).
- [ ] If you touched Chipyard: `rb chipyard elaborate --board arty200t --config <Class>`
      for the shells you affect. For a bump or refactor, compare `generated-src` before and
      after (method: pin-clean proof, section 1).
- [ ] For behaviour-neutral changes, images must be byte-identical. Build at the base and
      at your branch with the same flags, and copy each result out before the next build.
      The build directory is keyed on target/workload/mode, **not** on `--config`.
      ```sh
      rb build flight_controller --target arty200t --mode esp-motors \
         --config RocketArty200TDroneFullDDRLoaderConfig
      B=build/zephyr/arty200t/flight_controller/esp-motors
      OBJCOPY=$(sed -n 's/^CMAKE_OBJCOPY:FILEPATH=//p' $B/CMakeCache.txt)
      $OBJCOPY -O binary $B/zephyr/zephyr.elf /tmp/after.bin
      cmp /tmp/before.bin /tmp/after.bin && echo identical
      ```
      The `.elf` files may differ in debug sections; compare the binary. The config's DTS
      must be reachable (`RB_DTS_DIR` or Chipyard `generated-src`).
- [ ] `git status` in riskybird-dev shows no unintended `M backends/…`, and
      `git diff --cached --submodule` shows only the gitlinks you meant to move.
- [ ] The PR targets ucb-bar/riskybird `main`, and a pin-bump PR contains nothing else.

## 6. Don'ts

- Don't commit inside a backend checkout, and don't edit backend files except through the
  queue or a scratch clone.
- Don't run `git submodule update --init --recursive`, Chipyard's full `build-setup.sh`
  (its step 2 breaks the Saturn redirect), or `git submodule update --checkout <path>`
  inside a redirect OWNER.
- Don't run `tools/backend-patches apply` or `git submodule sync` from a linked worktree.
- Don't stage broadly in riskybird-dev: no `git add -A`, `git add .`, `git commit -a`, or a
  bare `git add backends`. `ignore=dirty` hides working-tree dirt, not moved HEADs, so
  these stage pin bumps silently. The only backend staging is the deliberate
  `git add backends/<b>` in a pin-bump PR (section 3).
- Don't commit or push unless the user asks.
- Don't trust `git status` or `git submodule status` for backends. Redirected paths print
  the OWNER's pin. Use the two `check` commands.
- Don't push to a shared `dev` branch, don't push to riskybird-dev `main`, and don't mix a
  pin bump into a feature PR.
- Don't put RiskyBird-specific code in a queue patch. Don't write generated output into
  `backends/modelblaster`.

## 7. Known gaps (open; fix or decide before relying on them)

- **Bench tier needs ModelBlaster.** With only `backends/zephyr-chipyard-sw` initialised,
  `tools/submodule-redirects apply|check` fails ("target backends/modelblaster is not checked
  out"), because the zcs → ModelBlaster redirect checks the owner, not the target. Initialise
  `backends/modelblaster` too until that is fixed.
- **`workloads/fc_dronet/patches/*.patch` are applied by nothing.** Six touch Zephyr
  (`arch/riscv/core/v.c`, `i2c_sifive`) and one the DroNet kernels; the `v.c` save-fence fix is
  not in the pinned Zephyr. They need to land on zephyr `rb-clean` or be retired.
- **Only Chipyard is guarded.** `backend-patches check` does not notice a dirty Zephyr, zcs or
  ModelBlaster tree; check those with `git -C backends/<b> status` before building images.
- **No `rb-clean` for Chipyard, RoSE, Saturn or TinyMPC.** Chipyard uses upstream `main` plus the
  queue; for the others, ask before committing anywhere.
- **riskybird-sim does not follow these pins.** Its guest builds against its own zcs
  (`GUEST_REV`), and `ROSE_PIN` must be bumped by hand alongside `backends/RoSE`.
