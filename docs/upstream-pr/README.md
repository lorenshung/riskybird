# Historical upstream patch proposal

This proposal predates the merged backend selected by `integration`. Its source
commits are already in that backend's history; do not reapply or squash this
series onto it. The patches and original proposal below are retained as a
historical record. Current pins and validation are in
[the integration record](../integration.md).

## Original proposal

Seven patches generated with `git format-patch` from commits that **already
exist** on our `dev`. No branch was created, no commit made, nothing pushed, no
PR opened. Applying them is a separate decision.

## Target `riskybirdv3-bringup`, NOT `ucb/dev`

This was checked by applying the series in throwaway worktrees, not assumed —
and the first plan was wrong.

```
samples/rose_flight_controller on ucb/dev                  -> DOES NOT EXIST
samples/rose_flight_controller on ucb/riskybirdv3-bringup  -> exists
```

Result, measured:

| patch | vs `ucb/dev` | vs `riskybirdv3-bringup` |
|---|---|---|
| 0001 FC builds without every sensor | FAILS | clean |
| 0002 warn on declared-but-disabled flow | FAILS | clean *(after 0001)* |
| 0003 `ROSE_ESTIMATOR_ONLY` | FAILS | clean |
| 0004 don't report a failed CMake registration | clean | clean |
| 0005 Zephyr SDK tarball per host | clean | clean |
| 0006 Miniforge installer per host | clean | clean |
| 0007 no Linux group advice on macOS | clean | clean |

The three FC patches fail against `ucb/dev` with *"No such file or directory"* —
the flight controller lives only on the bring-up branch. All seven apply cleanly
in order onto `riskybirdv3-bringup`, touching five files:

```
samples/rose_flight_controller/CMakeLists.txt
samples/rose_flight_controller/src/{flightlog.c, flow.c, main.cpp, side_tof.c}
```

0002 must follow 0001 — it edits a file 0001 creates hunks in, so it fails
standalone and succeeds in sequence. Squash them.

## Apply

```sh
cd backends/zephyr-chipyard-sw
git switch -c riskybird-fpga-portability ucb/riskybirdv3-bringup
git am ../../docs/upstream-pr/*.patch    # or `git apply` to stage without commits
```

The patches live in this repository, two levels above the submodule, so the
path is relative to `backends/zephyr-chipyard-sw`.

Then, before proposing:

- **squash 0001 + 0002** — one is a follow-up to the other
- **split the `ROSE_TELEM_DIV` hunk out of 0001** — console bandwidth is a
  different concern from devicetree presence guards
- **squash 0005 + 0006 + 0007** — all three are "stop assuming Linux/x86_64"
- **split out the `install_conda.sh:6` one-liner**: upstream's file literally
  contains the text `scripts/install_conda.sh#` where a bare `#` belongs. It is
  currently buried inside 0006; a reviewer should see it on its own.

Seven patches become about four commits.

## Not in this series, deliberately

- **Dima's 11 commits** (modelblaster / executorch / KernelBench) — already
  reachable from `ucb/dev`, `ucb/rose-2-dev` and two other upstream branches.
  They only look "ahead" because `riskybirdv3-bringup` never took the rose-2-dev
  line. Proposing them back would be noise.
- **`ab9d9a4915d`** — `git diff --stat ab9d9a4915d f33d768 -- samples/rose_flight_controller/`
  is **empty**. It squash-imported Dima's own FC onto our `dev` three days before
  the branch was properly merged. Including it re-proposes 7907 lines of his code
  to him.
- **`986c1d27556`, `3dc6a5f8250`** — repoint the `modelblaster` submodule at a
  personal fork.
- **`7fcc53a9d51`, `7850a848fd7`** — net no-ops at these endpoints. Worth a line
  in the PR description anyway: `7850a848` was the sifive-UART-instance-1 pinctrl
  guard, and the 2026-09-09 merge resolving `zephyr_ws/zephyr` back to upstream's
  pin is why `CONFIG_UART_SIFIVE_PORT_1=y` could not build until the nested
  submodule was re-synced.
- **`6e98a6811e7` (`west.yml`)** — needs rework, not cherry-picking. Our
  diagnosis is right (the old pin predates the bmi08x/vl53l1x I2C fixes and the
  VL53L1X metre conversion the estimator consumes, and `rose_nav_controller`
  selects `ROSE_CAMERA`/`ROSE_VISION` which do not exist on `zephyr-rose@main`),
  but our mechanism is worse: a branch name in a west manifest means every
  `west update` can produce a different tree. Dima's SHA pin was deliberate and
  his tree is the one that has flown. Propose a **SHA pin to the current tip of
  `rose-2-dev`**, keeping our comment block as the justification.
- **`ROSE_UART_CMD`, `fpga-esp-uart.conf`, `console-to-udp.py`** — new work from
  2026-09-22, and not in these patches. `ROSE_UART_CMD` is no longer unverified:
  every flight T01-T42 (2026-09-23/24) was built with it, and the dashboard
  RESET that launches each flight reaches the FPGA through it. It belongs in a later PR of its
  own, generated from the current pin rather than from this series.

## Worth saying in the PR description

`0001` adds a `flow-spi` alias rather than reusing the existing `flow` alias.
That is deliberate: `DT_ALIAS(flow)` is the RoSE *virtual* flow device (spike
co-sim), while the real PMW3901 path is gated on the `ROSE_FLOW` CMake knob and
touches the bus directly. Reusing `flow` would have silently killed optical flow
on the ESP config that actually flew. A reviewer's first question will be "why
not reuse `flow`?" — answer it up front.
