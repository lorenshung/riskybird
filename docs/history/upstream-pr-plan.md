# Proposing our work back to `ucb-bar/zephyr-chipyard-sw`

> **Historical record (2026-09-22), not instructions.** The plan behind the patch
> series in [`../upstream-pr/`](../upstream-pr/README.md), which supersedes it. Known-wrong claims:
> the uplink command reader (`ROSE_UART_CMD`) is "unverified on hardware" (every flight T01-T42,
> 2026-09-23/24, was built with it and flown with the dashboard ESTOP live); the submodule
> revisions quoted (`4af12b9`) predate the current pin (`834b9c6`).


Prepared 2026-09-22. **Nothing here has been executed.** No branch has been
created, no commit made, nothing pushed, no PR opened. This is the plan and the
exact commands; running them is a separate decision.

## The situation, measured

The histories were never divergent. They are strictly linear:

```
ucb/riskybirdv3-bringup  f33d768        upstream tip, 2026-09-02
   ⊂ riskybird-dev        e564842  (+32)
      ⊂ riskybird-dev-setup 4af12b9  (+10)
```

`git rev-list --count dev..ucb/riskybirdv3-bringup` is **0**, and the merge base
of the two working trees' submodules *is* the dev-tree commit. So there is no
history to reconcile — only a stale pointer, now fast-forwarded.

Of the 42 commits ahead of upstream, 31 are ours and 11 are Dima's. Dima's 11
(modelblaster / executorch / KernelBench) are already reachable from `ucb/dev`,
`ucb/rose-2-dev`, `ucb/et-sizing-knobs` and
`ucb/feat/firesim-bitexact-profile-recalibration`. They appear "ahead" only
because `riskybirdv3-bringup` is a side branch that never took the rose-2-dev
line. **A PR must not contain them.**

## Target the right branch

Target **`ucb/dev`**, not `riskybirdv3-bringup`. Dima's rose-2-dev line then
drops out of the range for free, instead of having to be excluded by hand.

## What to propose: 4–6 commits, not 32

The net tree delta `f33d768..dev` is 33 files / 1207 insertions, and most of it
does not belong upstream.

### Take

| commit | why |
|---|---|
| `1add285a5ad` | **The strongest item.** Compile-time presence guards so the FC builds on a board that lacks some sensors. Fixes four separate latent build breakages in Dima's own files — `side_tof.c`, `flow.c`, `flightlog.c`, `main.cpp` — each of which fails the *whole* FC build on a board missing one alias. Every fallback preserves the ESP32-C6 build bit-for-bit. Fold `e05b85e791d` in; split the unrelated `ROSE_TELEM_DIV` hunk out as its own commit, since console bandwidth is a different concern from devicetree presence. |
| `e564842b14c` | `ROSE_ESTIMATOR_ONLY`. Default-off CMake knob, one guarded branch in `solve_control()`. Zero risk. |
| `8d74f2e3b8a` | Installer reported "CMake package registered" even when registration failed, because `setup.sh` exits 0 on that path and `set -e` misses it. Pure bug fix to upstream's own script. |
| `de6c500adc6` + `45be01c08e7` + `6cc84ebd3d7`, squashed | "Stop assuming Linux/x86_64" — host detection for the Zephyr SDK tarball and the Miniforge installer, plus the macOS `usermod` guard. All default to existing Linux behaviour with an env-var escape hatch. |
| *(new one-liner)* | `install_conda.sh:6` literally contains the text `scripts/install_conda.sh#` where a bare `#` belongs — a stray paste in upstream's file. Currently buried inside the host-detection commit; pull it out so a reviewer actually sees it. |

Optionally the `.gitignore` `/yolov8n.pt` hunk from `b29368ebc52`, detached from
the data captures it is bundled with.

### Drop

- **Dima's 11** — already on `ucb/dev`.
- **`ab9d9a4915d`** — `git diff --stat ab9d9a4915d f33d768 -- samples/rose_flight_controller/` is **empty**. This squash-imported Dima's FC onto `dev` three days before the branch was properly merged. Including it would re-propose 7907 lines of Dima's own code back to Dima.
- **`986c1d27556`, `3dc6a5f8250`** — repoint the `modelblaster` submodule at a personal fork. Upstream cannot take that.
- **`7fcc53a9d51`, `7850a848fd7`** — net no-ops. `zephyr_ws/zephyr` is `df8e50e2` at *both* endpoints; the 2026-09-09 merge already resolved to upstream's pin. **Note the cost of that resolution**: `7850a848` was the sifive-UART-instance-1 pinctrl guard, and losing it is why `CONFIG_UART_SIFIVE_PORT_1=y` could not build until the nested submodule was re-synced. Worth saying so in the PR description even though the commit itself is dropped.
- **`73d0828c0d2` + `bbb4cb3c0b7`** — a wrong theory and its revert. Keep at most the surviving comment about macOS minimal tarball sizes.
- **`caf0e8bf39b`** + the conda-env rename inside `986c1d27556` — net zero on `install_submodules.sh`.

### Rework before proposing

**`6e98a6811e7` (`west.yml`) — our diagnosis is right, Dima's mechanism is better.**

- Ours: `revision: rose-2-dev` (a branch) for both `zephyr` and `zephyr-rose`.
- Dima's: `revision: 4329bf61…` (a SHA) for `zephyr`.

Our reasoning is correct and specific — the old pin predates the bmi08x/vl53l1x
ESP32-C6 I2C fixes and the VL53L1X metre conversion the estimator consumes, and
`samples/rose_nav_controller` selects `ROSE_CAMERA`/`ROSE_VISION`, which do not
exist on `zephyr-rose@main`. But a branch name in a west manifest means every
`west update` can produce a different tree; the manifest stops describing a
buildable state. Dima pinned a SHA deliberately, and his tree is the one that
has flown.

**Propose a SHA pin to the current tip of `rose-2-dev`, keeping our comment block
verbatim as the justification.** Do not propose `6e98a6811e7` as written.

### Not a conflict, but say so in the PR

`1add285a5ad` introduces a `flow-spi` alias rather than reusing Dima's `flow`
alias. That is deliberate: `DT_ALIAS(flow)` means the RoSE *virtual* flow device
(spike co-sim), while the real PMW3901 path is gated on the `ROSE_FLOW` CMake
knob and touches the bus directly. Reusing `flow` would have silently killed
optical flow on the ESP config that actually flew. A reviewer's first instinct
will be "why not reuse `flow`?", so answer it up front.

## How to build it

Do **not** rebase 32 commits. Branch from `ucb/dev` and cherry-pick:

```sh
cd backends/zephyr-chipyard-sw
git fetch ucb
git switch -c riskybird-fpga-portability ucb/dev

git cherry-pick 1add285a5ad e05b85e791d      # squash these two
git cherry-pick e564842b14c
git cherry-pick 8d74f2e3b8a
git cherry-pick de6c500adc6 45be01c08e7 6cc84ebd3d7   # squash these three

# then, by hand:
#   - split the ROSE_TELEM_DIV hunk out of the first commit
#   - split the install_conda.sh:6 stray-line fix into its own commit
#   - write the west.yml SHA pin (do NOT cherry-pick 6e98a6811e7)
```

That yields the full 1207-line net delta minus the parts that don't belong, in
~6 reviewable commits.

## What is NOT in this PR, and why

The uplink command reader (`ROSE_UART_CMD`), the `fpga-esp-uart.conf` pairing
and `tools/bench/console-to-udp.py` are all **new work from 2026-09-22 and
unverified on hardware**. They are the pieces that let a ground station command
the FPGA carrier at all, and they should be proposed only after the end-to-end
path has been exercised on a board — the uplink has so far been verified by
symbol inspection and one injected telemetry packet, not by a command actually
moving anything.
