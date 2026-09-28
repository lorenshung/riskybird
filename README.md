# RiskyBird

A RISC-V soft-core flight stack for a custom drone: Chipyard/Rocket FPGA shells
on a Trenz TE0712 carrier, Zephyr workloads that drive the board's sensors,
camera and motors, and one command — `rb` — that builds and runs all of it.

## Start here

This `integration` branch is based on the curated `setup` history. Its merged
backend pins are integration candidates: the flight and bench results below
describe the original setup versions until repeated with these pins. See the
[integration record](docs/integration.md) for the baseline, changes and checks.

**To fly the drone: [`docs/tutorial-flight.md`](docs/tutorial-flight.md).** One
machine, no Chipyard or Vivado: `rb fly` programs the shell, builds and loads a
flight preset, checks the ESP motor link and prints `READY`.

**To install: [`docs/setup.md`](docs/setup.md).** Built up one step at a time,
each ending in a command whose real output is quoted, so you can tell whether
the step worked rather than assuming it did.

Three tiers, and you probably want the first:

| Tier | What it does | Steps | macOS |
| --- | --- | --- | --- |
| **Bench** | build workloads, program the FPGA, debug, run the drone | 1–3 | **yes** |
| **Silicon** | elaborate Chipyard, build bitstreams | 1, then 4 | no — Vivado is Linux only |
| **Models** | compile a PyTorch model onto an accelerated shell | 1–2, then 5 | untested |

Because Vivado has no macOS release, the common arrangement is two machines: one
Linux host runs step 4 and publishes its bitstreams and devicetrees; every other
machine — Macs included — runs steps 1–3 against that published output. Neither
side needs the other's toolchain.

## What is here

| Path | |
| --- | --- |
| `rb`, `tools/rb/` | the entry point; `rb --help` lists every flow |
| `rb-env.sh` | `source` it to get a shell that can run `rb` — for scripts and CI |
| `tests/` | the test suite; needs the backends installed |
| `workloads/` | Zephyr applications — sensors, camera, motors, DDR stress. (`flight_controller` is the exception: it is maintained upstream and lives in the Zephyr backend, built in place.) |
| `hardware/zephyr/` | device-tree overlays and Kconfig per board, target and workload |
| `hardware/openocd/` | JTAG adapter and target configs |
| `tools/bench/` | bench helpers, including the FTDI reset the JTAG link needs, and the ground-station monitor |
| `tools/flight/` | the flight presets `rb fly` builds (`presets/*.env`) and the campaign scripts it replaced (`legacy/`) |
| `hardware/flight/`, `hardware/padtest/` | GDB scripts for the flight image (arm, commit, stop, where); the bare-metal motor-pad test |
| `tools/collect-*.py` | publish bitstreams and devicetrees for hosts that cannot build them |
| `backends/` | Chipyard, Zephyr and ModelBlaster, as submodules, initialised per tier |
| `artifacts/modelblaster/` | the committed DroNet kernels the accelerator shells run |

## Documentation

| | |
| --- | --- |
| [`tutorial-flight.md`](docs/tutorial-flight.md) | **fly the drone**: `rb fly`, the presets that flew, the ground station, known issues |
| [`setup.md`](docs/setup.md) | install, all three tiers, and the board acceptance and bootup checks |
| [`drone-initial-bringup.md`](docs/drone-initial-bringup.md) | first contact with the drone bench: adapters, console, what a dead link looks like |
| [`te0712-drone-carrier-ddr-bringup.md`](docs/te0712-drone-carrier-ddr-bringup.md) | DDR3 on the carrier, and the power findings — **read before concluding a board is broken** |
| [`esp-motor-offload.md`](docs/esp-motor-offload.md) | how the motors are driven now (from the ESP32-C6), and why: FPGA ball F13 is dead |
| [`wireless-telemetry-bringup.md`](docs/wireless-telemetry-bringup.md) | the ESP Wi-Fi bridge, the `RBLINK` line and the dashboard ESTOP |
| [`remote-bench-operation.md`](docs/remote-bench-operation.md) | running the bench when nobody is in the room |
| [`flight-calibration-procedures.md`](docs/flight-calibration-procedures.md) | the optical-flow scale and thrust-curve procedures |
| [`hm01b0-camera-bringup.md`](docs/hm01b0-camera-bringup.md) | the camera, end to end |
| [`hm01b0-colour-investigation.md`](docs/hm01b0-colour-investigation.md) | the camera is Bayer colour: the exposure and black-level fixes, and how to render colour |
| [`motor-pwm-bringup.md`](docs/motor-pwm-bringup.md) | FPGA motor PWM, the legacy path (motor 4 cannot be driven from it) |
| [`flight-on-fpga.md`](docs/flight-on-fpga.md) | porting the flight controller onto Rocket; phase plan and what was verified |
| [`te0712-ftdi-carrier-ddr-bringup.md`](docs/te0712-ftdi-carrier-ddr-bringup.md) | the same DDR3 work on the FTDI carrier |
| [`accel-shells-bringup.md`](docs/accel-shells-bringup.md) | the drone stack on the Gemmini/Saturn shells: which ones this branch supports, their bitstreams and devicetrees, bench results, and why the flight controller is not among them |
| [`dronet-on-accel-socs.md`](docs/dronet-on-accel-socs.md) | DroNet on those shells from committed sources: 14.57 fps on IntAt35, the recipe that regenerates it, and the Q31 spike model its gate needs |
| [`modelblaster-dronet-on-fpga.md`](docs/modelblaster-dronet-on-fpga.md) | the earlier single-shell DroNet write-up: the Gemmini/Saturn configs, the per-op timeline, and the two clock/FPU traps |
| [`upstream-pr/`](docs/upstream-pr/README.md) | the patch series proposed to `ucb-bar/zephyr-chipyard-sw` |
| [`repository-layout.md`](docs/repository-layout.md) | orientation and the curated integration scope |
| [`merging-into-main.md`](docs/merging-into-main.md) | integration policy and branch histories |
| [`history/`](docs/history/) | handoffs, plans and one-off records, kept for provenance; each opens with what it got wrong |

Docs distinguish **verified** (run here, output quoted) from **recorded**,
**declared** and **blocked**. That vocabulary is load-bearing — if a page says a
thing is verified, someone ran it and pasted what it printed.

## Scope

The supported surface is Chipyard, Zephyr and the drone. `rb --help` also lists
commands marked `[experimental]` — `rose`, `hil`, `xpurt`, `schedule`, `dse`,
`area`. Their backends and data (`backends/RoSE`, `backends/XPU-RT`, `dse/`,
`experiments/`) live on the `main` branch, not here, so on this branch they
cannot run (most of `rb area` included). See the
note at the top of `docs/setup.md`.
