# Handoff: RiskyBird drone stack → the accelerator SoCs (DroNet)

> **Historical record (2026-09-24), not instructions.** A session prompt for moving
> the drone stack onto the accelerator shells. Superseded by [`../accel-shells-bringup.md`](../accel-shells-bringup.md)
> and [`../dronet-on-accel-socs.md`](../dronet-on-accel-socs.md). Known-wrong claims: the dashboard
> ESTOP "does not stop the drone" (real with `esp_bridge --mode motors` + `ROSE_UART_CMD`, used in
> T01-T42); `esp_bridge --mode motors` "cannot be selected and has never run on hardware" (it is an
> `rb` mode and is the flying ESP image); fork submodules commit on `trenz` (the chipyard pin is on
> `arty200t-dual-uart`).


You are picking up the RiskyBird drone project: a quadrotor whose flight computer is a
Chipyard RISC-V SoC on an FPGA. The goal of this session is to build out DroNet (and related
vision workloads) on the more advanced SoCs — Gemmini and Saturn configs — **but first to verify
that the drone-specific stack, which has only been proven on the plain Rocket shell, still works
on those cores.** Everything below is context from the previous session. Where something is
verified it says how; where it is not, it says so. Treat unverified items as unverified.

Read the memory index (`MEMORY.md` in your memory directory) as well — it carries the standing
rules and several bench traps in more detail.

---

## Standing rules (these override convenience)

- **Motors: no test that can turn a motor without explicit, per-test approval.** State the
  purpose, the duty, the duration, and the stop path, then wait. One approval covers one test; it
  does not carry across a rebuild or reflash. **Battery presence is NOT remotely observable** —
  the charger floats +BATT to 4.2 V with no pack fitted, indistinguishable from a full pack — so
  "the pack is out" is the user's word, and the rule still applies. A motor-capable image means
  anything with PWM or ESP-motor support loaded and the core resumed.
- **The FullDDR-family bitstreams drive the motor gates directly from FPGA pins** (motor1–4 on
  E22/B20/R17/F13, via `WithArty200TPWM`). An unexplained motor spin-up happened on 2026-09-22
  while running *camera-only* images on `RocketArty200TDroneFullDDRConfig`; the ESP log proved the
  ESP did not command it. A similar event is recorded from 2026-09-04. **Root cause unknown.** Every
  accelerator config below carries the same PWM periphery. Do first boots of any new shell with the
  pack out.
- **Never raise the 3.3 V rail** to fix the DDR/MIG shells browning out — an ESP32 was destroyed that
  way. The TE0712 MIG shells need a supply that can deliver real current; 3.3 V @ 1 A is not enough.
- **Do not commit or push unless asked**, and a previous "yes" does not carry forward. No AI
  attribution in commits, PRs or docs. Commit only to `lorenshung/*` forks. In `riskybird-dev`,
  commits go on `main`; fork submodules commit on `trenz`.
- The dashboard's ESTOP button **does not stop the drone** (see Flight stack). Pulling the pack is
  the only stop that does not depend on software.

---

## The hardware

| | |
|---|---|
| FPGA module | Trenz **TE0712**, Xilinx **XC7A200T-FBG484** |
| Carrier | RiskyBird v3 drone carrier (real layout: branch `riskybirdv3-layout` of `lorenshung/riskybird`; `hardware/pcb` in the repo is a stale July v2 snapshot) |
| Baseline SoC | `RocketArty200TDroneFullDDRConfig` — one Rocket core, **50 MHz**, DDR3 |
| Motors | four **brushed** motors on low-side SI2302 FETs. **No ESCs.** |
| Motor PWM | offloaded to an **ESP32-C6** (FPGA pin F13's output driver is dead — proven with bare-metal Verilog; the net is intact). FPGA→ESP over uart1 (E13/F14) at 115200 |
| Power | a 1S LiPo for the motors; USB-C powers the logic. Motor tests do nothing on USB alone |

Bench USB (locations are stable; bus:device and ttyUSB numbers renumber on every replug — run
`./rb bench` to resolve them):

| Role | USB location | Notes |
|---|---|---|
| bitstream link | `7-1.1.4.1` | xc7a200t config TAP |
| debug link | `7-1.1.4.3` | Rocket debug TAP; its channel B is the FPGA console |
| ESP32-C6 | `ttyACM0` | runs `esp_motors` (PWM only, **no networking**) |

The two FT2232Hs sit three hubs deep and drop off in pairs; `dmesg` distinguishes a bus drop from a
dirty MPSSE state.

---

## What is verified on the baseline SoC

On `RocketArty200TDroneFullDDRConfig`, 50 MHz, from `workloads/bootup_check` and follow-up runs:

| Subsystem | State | How known |
|---|---|---|
| DDR3 | OK | 4 MiB address-in-address test |
| I2C inventory | ADS7128 `0x17`, BMI088 accel `0x18` / gyro `0x68`, HM01B0 `0x24`, barometer `0x76` | bootup_check |
| IMU (BMI088) | OK, \|a\| ≈ 9.7–9.8 m/s² at rest | bootup_check |
| ToF | **2 of 3 answer.** ch1 side VL53L5CX at `0x31`, ch6 down VL53L1X at `0x34`; ch2/ch3 side absent. 3 are intended, so the WARN is a **real fault** | bootup_check + XSHUT scan |
| Optical flow (PMW3901) | OK, chip id `0x49` / inverted `0xB6` | bootup_check |
| Camera (HM01B0) | **colour photographs**, 324×315 | captured and rendered 2026-09-22 |
| ESP motor link | works; ESP applies duty, 120 ms receive-timeout failsafe cuts all four; halt-core failsafe verified | earlier sessions |
| Flight controller | runs, streams telemetry | free-run on 2026-09-22, no motors |

---

## Flight stack

`backends/zephyr-chipyard-sw/samples/rose_flight_controller/` (Dima Nikiforov's, with RiskyBird
additions). The `zephyr-chipyard-sw` history is **strictly linear** with upstream
`ucb-bar/zephyr-chipyard-sw` `riskybirdv3-bringup` — there is no divergence to reconcile. The dev
tree's submodule is at `4af12b9`.

- **Attitude state is Gibbs parameters, `tan(θ/2)`, not radians.** Every tilt limit written as if it
  were radians was ~2× looser than its comment. The telemetry's `tilt=est…deg/acc…deg` field is the
  honest angle.
- **`ROSE_USE_EKF=0` is required** — the EKF's ToF gate decouples altitude and it climbs away. Use the
  complementary estimator.
- **Arming is Dima's lift-and-place gesture** (`ROSE_AUTOFLIGHT`): lift >15 cm on the down ToF, set
  down level (<11.5°), on the ground, still (<0.3 rad/s), hold 1.5 s. It is layer 2 of four
  interlocks: `g_arm_enabled` false at boot → the gesture → the motor/commit gate (5 s uncommitted; 60
  s needs both a build flag and a debugger writing `0x464C5921`) → duty ceilings and the envelope
  guard. **Trap:** stock `ARM_MAX_HEIGHT_M` is 0.020 but a drone flat on the bench reads 0.022 m, so
  the gesture can never complete; the setup tree's autoflight mode overrides it to 0.05.
- **The dev tree's `flight_controller` has no altitude sensor.** `CONFIG_VL53L1X=y` exists only in the
  setup tree's `flight_controller.conf`. Without it `tofv=0`, `z` is a dead estimate (it read −3.1 m),
  vertical velocity runs away, and the envelope guard latches ESTOP within seconds
  (`EMERGENCY CUTOFF … velocity limit exceeded`). That guard behaving correctly is not a fault; the
  build is.
- **Uplink commands (new, `ROSE_UART_CMD`).** The FC now reads `ESTOP/DISARM/RESET/HOVER_Z/PROFILE/
  PING` from uart1 and replies with its own `FCACK`/`FCNAK`. Built, in-image, and contract-tested
  (`tests/test_uplink_cmd.py`), but **never exercised end to end**: that needs `esp_bridge` on the
  ESP, and the ESP runs `esp_motors`.
- **The dashboard ESTOP is still not a kill.** Of three breaks, one is closed (the FC reader above).
  The ESP still fabricates its own ACK, and the gate-cutting ESTOP lives in `esp_bridge --mode
  motors`, which `boards.py` cannot select and which has never run on hardware.
- Ground stations: Dima's `tools/groundstation/riskybird_panel.py` (3D view) and the setup tree's
  `tools/bench/riskybird-dash.py`. Both ran on live FPGA telemetry on 2026-09-22 through
  `tools/bench/console-to-udp.py`, which forwards the wired console to UDP 14550 — the FPGA has no
  radio, and that was the only reason Dima's panel never showed FPGA data.

Hover-demo prerequisites still open: a thrust curve, `ROSE_ESP_MOTORS_HZ=200` verified on hardware
(watch the ESP's `gap=`), autoflight on, `FLOW_LP_TAU_Y=0.15`, `ROSE_TELEM=0`.

---

## Camera

Now packaged standalone at **`github.com/lorenshung/ospi-camera`** (RTL, Chipyard integration,
Zephyr driver, host tools, a real example frame; `make check` verifies it with no hardware). Read its
`docs/bringup-lessons.md` before touching the camera. The short version — each of these looked like
broken hardware:

- The sensor won't ACK I2C until **MCLK** runs. **12.5 MHz answered; 6.25 MHz did not.**
- "No light" was an **exposure ceiling**: use `FRAME_LEN=4000`, `MAX_INTG=3900`, and let AE settle
  **~50 frames** (6 s), not a fixed short delay.
- A frame with **no noise** is the black-level clamp: `BLC_CFG=0x00`.
- `AE_TARGET` is a **linear** mean: 64, not 128.
- Lines are **326** wide (two dummy pixels), a frame is **324** rows — reshape at `LASTWIDTH`.
- Under warm light the colour sensor looks mono to any global statistic; the Bayer phase is found by
  which diagonal pair agrees, per frame.

Capture on the bench: `./rb debug bootup_check … --gdb-script hardware/ospi/bootup-capture.gdb
--ram-console`, then `python3 hardware/ospi/frame-to-colour.py bootup-frame.raw 326`.

---

## The accelerator SoCs and the integer DroNet

Two branches carry this work. **Both are already checked out and contain nothing you lack** —
verified with `git rev-list --left-right`, so there is nothing to fetch or merge:

| Branch | Where it is | Relationship |
|---|---|---|
| [`lorenshung/chipyard` `ku040-codesign-cnn`](https://github.com/lorenshung/chipyard/tree/ku040-codesign-cnn) (tip `8dd1b34c`, Dima, 09-05) | `backends/chipyard`, checked out at `arty200t-dual-uart` | fully contained; the checkout has 6 more commits on top, including the merge (`eebb8c47`) |
| [`ucb-bar/ModelBlaster` `riskybird-integer-dronet`](https://github.com/ucb-bar/ModelBlaster/tree/riskybird-integer-dronet) (tip `10b50ba`, Dima, 09-05) | the **setup** tree's `backends/modelblaster`, at `riskybird-integer-dronet` | fully contained; the checkout has 10 more commits (Loren's). Its `origin` is `lorenshung/ModelBlaster`; an `ucbbar` remote was added for the comparison |

The dev tree has no `backends/modelblaster`; the orphan at `backends/zephyr-chipyard-sw/modelblaster`
is a stale `main` checkout — do not use it.

### The FPGA configs — what closes, measured

Configs are in `backends/chipyard/fpga/src/main/scala/arty200t/`: `AccelConfigs.scala` (the signed-off
set), `RbAreaExperiments.scala` (no-LoopConv), and `TacitAccelConfigs.scala` / `TacitDropConfigs.scala`
(TACIT hardware trace). All share **`WithArty200TDroneFullDmaUartBase`** — periphery byte-for-byte
`RocketArty200TDroneFullDDRDmaUartConfig`: DMA camera, uart1 TX-DMA, PWM, flow SPI, GPIO, I2C.

Post-route, read from each build's `obj/report/{timing.txt,rb_impl_utilization_flat.txt}` in
`backends/chipyard/fpga/generated-src/chipyard.fpga.arty200t.Arty200THarness.<config>/`:

| Config | Accelerators | Scalar FPU | Clock | LUT | DSP | BRAM | WNS | Timing |
|---|---|---|---|---|---|---|---|---|
| `RocketArty200TDroneGemminiSaturnFp16At35Config` | Gemmini 16×16 + Saturn V128D64 fp16 | **FP16 only** | 35 MHz | **93.11%** | 75.0% | 66.9% | +0.289 ns | **met** — the combined shell, and the core Dima flashed |
| `RocketArty200TDroneGemminiSaturnIntAt40Config` | Gemmini 16×16 + Saturn int-only | **full** | 40 MHz | 90.58% | 76.6% | 66.9% | **−0.067 ns** | **NOT met** — bitstream exists, not signed off |
| `RocketArty200TDroneGemmini16At40Config` | Gemmini 16×16 | full | 40 MHz | 70.34% | 69.1% | 75.6% | 0.000 ns | met (needed `RB_IMPL_DIRECTIVE=explore`) |
| `RocketArty200TDroneSaturn40Config` | Saturn V128D128 fp16 | full | 40 MHz | 90.1% | | | +0.422 ns | met per Dima's commit `c30dc63d`; no build in the dev tree |

Also on the branch: `…Fp16At35TacitFullNoLoopConvConfig` fits the combined design plus a TACIT trace
sink at **92.4%** with the full periphery (dropping LoopConv frees 4,841 LUT / 130 DSP; convs then run
via im2col → LoopMatmul), and the combined design at 50 MHz routes but misses by **−5.29 ns**. Gemmini
needs a power-of-2 mesh; 32×32 is infeasible on the 200T.

**The integer shell is 67 ps short, and the remedy is known.** `Gemmini16At40` first routed at −0.091
ns and `RB_IMPL_DIRECTIVE=explore` closed it (commit `25fbe99a`). Try that on `IntAt40` before anything
else; an `At35` variant is the one-line fallback its commit (`4aeeae37`) names.

**Why the integer shell exists, and why it matters for this work.** On the FP16 shell,
`WithRocketFPU16` implements only Zfh, so the image must be built `CONFIG_FPU=n`, which forces a
soft-float ABI — and the soft-float ABI is what blocks the RVV kernels. In `4aeeae37`'s words: *"the
accelerator is switched off by its own floating-point unit."* The integer shell keeps the full FPU:

```
rv64imafdc ... zve64x ... zfh ...        IntAt40: hard-float, integer vector -- FC and RVV both run
rv64imafc  ... zve64d ... zvfh_zfh ...   Fp16At35: no "d", fp16 vector
```

So `IntAt40` is the shell that can carry **both** the fp32 flight controller and Gemmini + RVV — once
it closes timing.

Build a shell (synthesis with DSP-attribute injection, then place-and-route from the injected
checkpoint — separate steps, because re-elaborating discards the injection):

```sh
./rb area run --board arty200t --config <Config> --attrs dsp
make -C backends/chipyard/fpga SUB_PROJECT=arty200t CONFIG=<Config> rb-impl        # add RB_IMPL_DIRECTIVE=explore if it misses
backends/chipyard/fpga/scripts/floorplan/gen_floorplan.sh --config <Config>        # post-route floorplan PNG, coloured by block
```

`rb-impl` writes `obj/rb_post_route.dcp`, `obj/rb_impl.bit`, and the reports. **Read the reports in
`obj/report/` before writing any TCL or reopening a checkpoint.**

### The integer DroNet

Dima's branch takes DroNet to integer-only int8 end to end — integer batchnorm, integer fixed-point
requantise, no fp32 anywhere in the hot path, so the kernels run with `misa.F=0`. His tip commit
(`10b50ba`) stacks three accuracy-free layout levers on the flashed At35 core — **grayscale, JTAG-load
only, one Rocket hart, `gemmini_q31_rvv` backend**:

| Step | kcycles / frame | ms | fps |
|---|---|---|---|
| base (NCHW, requant tree) | 4,536 | 129.6 | 7.72 |
| + NHWC convs and maxpool | 3,446 | 98.5 | 10.16 |
| + conv0 + maxpool1 fused into one Gemmini call | 2,952 | 84.3 | 11.86 |
| + RVV segment-store boundary relayouts | **2,629** | **75.1** | **13.31** |

Output `[114,127]`, max_abs_err 3 — identical to the NCHW anchor — across 5 runs. The commit gives the
regeneration recipe (`assign_layouts … --policy islands --hint examples/dronet/int8/
dronet_layout_hint.json`, then `generate_skeleton --backend gemmini_q31_rvv` with `MB_REQUIRE_NHWC=1`).

**An unresolved contradiction — settle it, do not pick a side.** Twelve days later, Loren's `02079fc`
concluded that `gemmini_q31_rvv` (`-march=rv64imafdcv -mabi=lp64d`) **cannot run** on the FP16 shell:
the RVV kernels need `v` in `-march`, the shell's image is `CONFIG_FPU=n`, the variant that would fit
(`rv64imacv` + `lp64`) cannot be verified because the verify harness builds `spike_riscv64` as `lp64d`,
and so *"the combined shell runs today via `gemmini_q31_softfp`: Gemmini working, Saturn idle."* Both
records are specific and both are dated. The likeliest reconciliation is that Dima's run was built
outside `rb`'s Zephyr `CONFIG_FPU=n` flow — find out how, from his commits and scripts, before
reproducing the 13.31 fps.

Also measured, through `rb`'s own flow: **5.31 fps** on `Gemmini16At40` (`conv2d_s8` on Gemmini at
max_abs_err 2, `--drift-atol 2`), 0.06 fps bit-exact — see the setup tree's
`docs/modelblaster-dronet-on-fpga.md`.

### The Gantt plots — two different things

- **A per-op timeline of one inference on one hart.** `rb modelblaster report` renders it
  (`gantt_svg()` in the setup tree's `tools/rb/modelblaster.py`) from a captured run: a waterfall, since
  one hart runs the kernels in sequence. Verified against a real spike capture; example in the setup
  tree at `docs/figures/dronet_gemmini_q31_spike_gantt.svg`. The on-board flow is
  `rb modelblaster generate` → `build` → `rb debug`/`rb capture` → `report` — **setup tree only**.
- **A scheduled program across accelerators** — the "well scheduled" kind. ModelBlaster profiles each
  (network, backend) into `gen/profile/…/results.csv`; the XPU-RT scheduler (MILP or greedy;
  `scripts/run_xpurt_scheduler.py`) emits `schedules/scheduled_<workload>.json` and a predicted timeline;
  the dispatch binary is built from that with `XPURT_TRACE=1` (`examples/xpurt_demo/run.sh`), and
  `scripts/plot_xpurt_trace.py` renders measured against predicted. HEFT / CP-SAT / MOSEK schedules for
  dronet + mlp_control + yolov8_nano are in `notes/figures/gantt_3way_*.png`, with period windows
  (10 ms mlp_control, 20 ms dronet) and deadline markers. **ModelBlaster documents this flow on FireSim
  (`RUNNER=firesim`); running it on the board is not recorded anywhere.** `rb xpurt` and `rb schedule`
  (in both trees) are the RiskyBird-side entry points; `rb modelblaster` exists only in the setup tree.
- `MB_TACIT_HW=1` (`5d2f69f`) brackets one inference with the TACIT hardware trace (controller
  `0x3000000`, sink `0x3010000`) on the TACIT configs — the hardware source for a measured timeline.

None of the drone peripherals has been checked on any of these shells.

---

## What will differ on the new cores — the verification checklist

These are the specific ways the proven stack can break on the accelerator shells. Check each; do not
assume.

1. **Timebase — the one that breaks everything.** `hardware/zephyr/fpga.conf` hardcodes
   `CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC=50000` for the 50 MHz shells. Every SiFive driver derives its
   clock from `SIFIVE_PERIPHERAL_CLOCK_FREQUENCY`, which is computed from that value — so one wrong
   constant sets the kernel tick, **the UART baud divisor**, and the I2C/SPI/PWM dividers all wrong
   together. At 35 MHz the UART transmits 80640 baud and the console reads as line noise.
   **The setup tree's `rb` generates the right value from the elaborated DTS
   (`generated_clock_conf`, `dts_timebase_hz`); the dev tree's `rb` does not.** Use the setup tree's
   `rb` for these shells, or port that function first. (The `clock-frequency = <50000000>` in
   `fpga-esp-uart.overlay` is *not* used by the SiFive UART driver and is not the problem.)
2. **FPU.** `…Fp16At35Config` has an FP16-only scalar FPU (`WithRocketFPU16`, non-spec Zfh-without-F).
   The flight controller is built hard-float `rv64imafdc`/`lp64d`; F/D instructions will trap there.
   Setup's `rb` (`generated_fpu_conf`) switches such shells to `CONFIG_FPU=n` and picolibc
   soft-float — which runs, but makes the estimator and controller far slower. For the flight stack
   prefer the full-FPU shells (`Gemmini16At40`, `SaturnIntAt40`, `Saturn40`).
3. **Slower clock → slower control loop.** The default controller already runs 25–35 ms per iteration
   at 50 MHz. At 35–40 MHz check the loop period against the ESP's 120 ms receive failsafe and the
   motor-frame rate.
4. **Camera MCLK.** The driver's divider is fixed at 1 (`sysclk/4`): 12.5 MHz at 50 MHz, but **10 MHz
   at 40 and 8.75 MHz at 35 — neither has been tried**, and 6.25 MHz is known not to answer. The
   divider is overridable (`OSPI_CAMERA_MCLK_DIV`); the formula is `sysclk / (2 × (div + 1))`, so
   12.5 MHz is not exactly reachable at 35 or 40 MHz.
5. **Camera DMA periphery.** These shells use `WithArty200TOspiDmaPeriphery`. The photographs were
   taken on the non-DMA periphery with the CPU popping `DATA`. The register map says `DATA` still
   works while `DMA_CTRL.enable` is 0 (the reset value) — **unverified on hardware.** The DMA path
   itself is Dima's (chipyard `ecc51d57`).
6. **uart1 TX DMA.** A `WithUartTxDma` block sits on uart1 (`0x10022000`). The FC writes motor frames
   and telemetry to uart1 by polling, and the new uplink reads it — coexistence with the DMA block is
   unverified.
7. **Interrupt numbering.** `rb` asserts the ESP UART at PLIC source 11 and the shifted GPIO/SPI
   sources against the generated DTS and refuses to build on a mismatch. That check is your friend;
   don't disable it.
8. **Saturn `noPermute`** (`SaturnIntAt40`) removes vrgather/vcompress/vslide; a kernel that lowers to
   one traps illegal-instruction at run time. Check emitted kernels.
9. **Motor gates.** The PWM periphery is present on every one of these — the unexplained spin-up
   applies. Pack out.

A sensible order per shell: `bootup_check` (sensors + camera, no PWM in the image) → one camera
capture → a no-motor `flight_controller` telemetry run with the pack out → the ESP link.

---

## Repository state and tooling

- **Two working trees of the same repo, on unrelated histories** (`git merge-base` finds nothing):
  - the `main` checkout (`riskybird-dev`) — branch `main`: the research surface (DSE, experiments, RoSE,
    XPU-RT, KU040 work) and **this session's uncommitted work**.
  - the `setup` checkout (`riskybird-dev-setup`) — branch `setup`: **ahead on capability** — the `rb` with
    `telem/esp-motors/autoflight` modes, the clock/FPU derivation above, `esp_bridge`/`esp_motors`/
    `motor_safe` workloads, `hardware/flight/*.gdb`, the bench dashboard/watch scripts, and
    ModelBlaster/DroNet (`rb modelblaster`, `backends/modelblaster`).
  - For accelerator work, **the setup tree's tooling is the one that handles these shells.** Porting
    setup → main is planned but not done (~60 files; the survey is in the previous session).
- **Uncommitted in `riskybird-dev`** (nothing staged): the submodule fast-forward to `4af12b9`; the
  uplink reader; `hardware/zephyr/fpga-esp-uart.conf` (the Kconfig that must travel with the ESP-UART
  overlay, or builds link against a non-existent device); `tools/bench/console-to-udp.py`;
  `tests/test_uplink_cmd.py`; `hardware/flight/{freerun,where}.gdb`; the camera pipeline; three
  `frame-to-png.py` fixes; ToF and `bootup_check` merges from setup; `docs/upstream-pr/` (seven patches
  verified to apply to `riskybirdv3-bringup`).
- `zephyr_ws/zephyr` must match its recorded pin after any submodule move — `git submodule update
  zephyr_ws/zephyr`. A stale checkout lost a pinctrl guard and broke `CONFIG_UART_SIFIVE_PORT_1`.

Bench traps that cost real time:

- **`rb debug --gdb-script` never starts the core.** `rb` adds `-ex continue` only without a script.
  `detach` alone leaves the core at `__start` with `.bss` unzeroed — silent, and it looks like success.
  Use `monitor resume` then `detach` (`hardware/flight/freerun.gdb`). `where.gdb` tells "running but
  silent" from "not running".
- **`RB_CMAKE_ARGS` sets CMake cache variables only.** A bare `#define` knob needs `RB_EXTRA_CFLAGS`
  (C) or `RB_EXTRA_CPPFLAGS` (C++). The wrong one fails silently — always print the value back.
  Cache values also stick across builds; use `--pristine always` when in doubt.
- **The bench console logger dies half-way on a replug**: its `cat /dev/ttyUSBn` keeps reading (the
  pane shows data) but stops writing `fpga-console.log`. Anything reading the log sees nothing. Restart
  `rb-watch.sh`, or `tmux pipe-pane` the pane. Never open the tty a second time — two readers corrupt
  both streams.
- Short workloads print before you can attach: use `--ram-console`.
- `CTRL_ITERS <= 0` runs the flight controller forever; the default 5000 stops it after ~2 minutes.
- Stop a running image with the setup tree's `hardware/flight/stop.gdb` — it zeroes the PWM comparators
  *before* halting, which matters because the PWM block keeps driving pins after the CPU stops.

---

## First: dispatch a subagent to restore the altitude sensor

Before any flight work on the new cores, the flight controller needs its down-facing ToF back.
It has worked before; this is a configuration regression in the dev tree, not a hardware fault.
Dispatch a subagent with the prompt below and keep working on other things while it runs. When it
reports, check its evidence yourself before relying on it.

```text
Restore the down-facing altitude sensor (VL53L1X) in the RiskyBird flight controller build.

THE SYMPTOM. In the main checkout, `./rb build flight_controller --target arty200t
--config RocketArty200TDroneFullDDRConfig` produces an image with no altitude sensor at all: the
generated devicetree has no `tof` alias and CONFIG_VL53L1X is not set, so HAVE_TOF is 0. On
hardware (2026-09-22) the telemetry read tofv=0 on every one of 1086 lines, z sat at a dead -3.1 m,
vertical velocity ran away, and the envelope guard latched EMERGENCY CUTOFF within seconds. The
guard was right; the build was wrong.

IT HAS WORKED. The user is certain, and there is evidence: earlier flight-controller telemetry on
this bench read `tofv=1 tofh=0.030`. The down VL53L1X is physically present and answers -- the
bootup_check workload brings it up on ADS7128 XSHUT channel 6 and reads id 0xeacc.

THE STRONG LEAD -- verify it, do not just trust it. The sibling setup
checkout already carries a fix, and the dev tree lacks every piece of it:
  - hardware/zephyr/targets/fpga/workloads/flight_controller.overlay declares `vl53l1x@30`
    (compatible "st,vl53l1x") and a `tof = &vl53l1x;` alias. Its own comment describes this exact
    failure: "Without the node the estimator ran with tofv=0 on every sample and z dead-reckoned".
  - hardware/zephyr/targets/fpga/workloads/flight_controller.conf sets CONFIG_VL53L1X=y and
    CONFIG_VL53L1X_INTERRUPT_MODE=n (polling -- read the comment for why).
  - main.cpp (in the shared submodule backends/zephyr-chipyard-sw, same commit in both trees)
    readdresses the part from its power-on 0x29 to VL53L1X_DOWN_ADDR (0x30) via vl53l1x_readdress_down()
    before the ST driver's deferred init runs.
Diff the dev and setup copies of those files, understand WHY each line is there, and port the
proven fix. Prefer porting setup's working configuration over writing a new one.

THINGS TO CHECK WHILE YOU ARE THERE:
  - Addresses. The setup overlay declares the down part at 0x30; bootup_check's ToF library puts
    it at 0x34, and the side VL53L5CX sits at 0x31. Make sure nothing collides on the shared I2C bus
    in the flight-controller image, and that the devicetree address matches what main.cpp readdresses
    to. Dima's commit e51f10f in zephyr-chipyard-sw is titled "fix down-ToF 0x29 collision" -- read it.
  - hal_st. CONFIG_VL53L1X needs the ST HAL in the west workspace. `./rb zephyr setup-targets`
    installs the pinned ESP and ST HAL projects; confirm hal_st is present under
    backends/zephyr-chipyard-sw/zephyr_ws/modules/hal/st before blaming anything else.
  - Every mode. The dev tree's rb has flight_controller modes default, flow and motors. Make sure the
    sensor is present in each, not just default.
  - The accelerator shells. The same overlay/conf chain must give the sensor on
    RocketArty200TDroneGemmini16At40Config and the other configs in
    backends/chipyard/fpga/src/main/scala/arty200t/AccelConfigs.scala. Build at least one.
  - Gotchas from the last session: a bare #define knob needs RB_EXTRA_CFLAGS / RB_EXTRA_CPPFLAGS, not
    RB_CMAKE_ARGS (the wrong one is a silent no-op), and CMake cache values stick -- use
    `--pristine always` when checking a config change actually took.

VERIFY BY EVIDENCE, NOT BY THE BUILD SUCCEEDING. A build that links proves nothing about the sensor.
For each build, show: `tof` alias present in the generated devicetree
(build/.../zephyr/include/generated/zephyr/devicetree_generated.h), CONFIG_VL53L1X=y in the build's
.config, and the VL53L1X driver symbols in the ELF (nm). Run the repo's test suite
(`python3 -m unittest discover -s tests`) and confirm it still passes.

HARD CONSTRAINTS:
  - Do NOT touch hardware -- no rb debug/program/load/run, no serial devices, no openocd. The
    flight controller is a motor-capable image and needs the user's explicit per-test approval to
    run. Builds, file reads and host-side tests are fine.
  - Do NOT commit, push, or stage. Leave changes in the working tree and list every file you changed.
  - Do not disable, loosen or bypass the envelope guard or any safety check to make telemetry look
    sane. The guard firing on a dead altitude estimate is correct behaviour.

REPORT: what was missing and why; each file changed with a one-line reason; the build evidence
above per mode and per config; anything you found that you could not resolve; and the exact
hardware test you would ask the user to approve to confirm tofv=1 on the board (purpose, image,
that no duty is commanded, how it is stopped, and that the pack should be out).
```

## Your tasks

<!-- add tasks below -->
