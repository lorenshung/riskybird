# RiskyBird

A RISC-V soft-core flight stack for a custom drone: Chipyard/Rocket FPGA shells
on a Trenz TE0712 carrier, Zephyr workloads that drive the board's sensors,
camera and motors, and one command — `rb` — that builds and runs all of it.

## Start here

**[`docs/setup.md`](docs/setup.md)** — the install guide. Built up one step at a
time, each ending in a command whose real output is quoted, so you can tell
whether the step worked rather than assuming it did.

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
| `tools/bench/` | bench helpers, including the FTDI reset the JTAG link needs |
| `tools/collect-*.py` | publish bitstreams and devicetrees for hosts that cannot build them |
| `backends/` | Chipyard, Zephyr and ModelBlaster, as submodules, initialised per tier |

## Documentation

| | |
| --- | --- |
| [`setup.md`](docs/setup.md) | install, both tiers |
| [`drone-initial-bringup.md`](docs/drone-initial-bringup.md) | first contact with the drone bench: adapters, console, what a dead link looks like |
| [`te0712-drone-carrier-ddr-bringup.md`](docs/te0712-drone-carrier-ddr-bringup.md) | DDR3 on the carrier, and the power findings — **read before concluding a board is broken** |
| [`hm01b0-camera-bringup.md`](docs/hm01b0-camera-bringup.md) | the camera, end to end |
| [`motor-pwm-bringup.md`](docs/motor-pwm-bringup.md) | motor PWM |
| [`flight-on-fpga.md`](docs/flight-on-fpga.md) | porting the flight controller onto Rocket; phase plan and what is verified |
| [`te0712-ftdi-carrier-ddr-bringup.md`](docs/te0712-ftdi-carrier-ddr-bringup.md) | the same DDR3 work on the FTDI carrier |
| [`modelblaster-dronet-on-fpga.md`](docs/modelblaster-dronet-on-fpga.md) | a PyTorch model to an accelerated XC7A200T shell: the Gemmini/Saturn configs, the per-op timeline and FPS, and the two clock/FPU traps |
| [`verification.md`](docs/verification.md) | what "verified" means here, and which tests pass where |
| [`repository-layout.md`](docs/repository-layout.md) | orientation |
| [`merging-into-main.md`](docs/merging-into-main.md) | what this branch changed, and how to fold it back |

Docs distinguish **verified** (run here, output quoted) from **recorded**,
**declared** and **blocked**. That vocabulary is load-bearing — if a page says a
thing is verified, someone ran it and pasted what it printed.

## Scope

The supported surface is Chipyard and Zephyr. `rb --help` also lists six
commands marked `[experimental]` — `rose`, `hil`, `xpurt`, `schedule`, `dse`,
`area`. They work and are used in this repository's own experiments, but their
setup is undocumented and their interfaces are expected to change. See the note
at the top of `docs/setup.md`.
