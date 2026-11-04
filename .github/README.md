# RiskyBird

RiskyBird is a palm-sized quadrotor whose flight computer is a Chipyard RISC-V
SoC on an FPGA module. This repository holds everything RiskyBird-specific:
the board, the SoC configurations and RTL, the flight software and bring-up
workloads, the bench and ground-station tools, and `rb`, the command that ties
them together.

The shared projects RiskyBird builds on stay generic and are pulled in as
submodules under `backends/`.

## Start here

- **Fly the drone:** [`tutorial-flight`](https://app.notion.com/p/3ea06b97ede4819a8827dddf36b95e48).
  `rb fly` programs the shell, builds and loads a flight preset, checks the ESP
  motor link and prints `READY`. No Chipyard or Vivado needed.
- **Install:** [`setup`](https://app.notion.com/p/3ea06b97ede48144ad05c3d1c426a09b).
  Three tiers: **Bench** (build workloads, program the FPGA, debug, fly; works
  on macOS), **Silicon** (elaborate Chipyard and build bitstreams; Linux, needs
  Vivado), **Models** (compile a PyTorch model onto an accelerated shell). A
  common split is one Linux host that builds and publishes bitstreams and
  devicetrees, and any number of bench machines that use them.

## Layout

| Path | What it holds |
|---|---|
| `rb`, `tools/rb/` | the `rb` command: build, program, load, console, debug, fly |
| `workloads/` | Zephyr workloads: `flight_controller`, `fc_dronet`, sensor, motor, camera and ESP bring-up |
| `hardware/chipyard/` | RiskyBird Chipyard configs, IO binders and the FPGA build wrapper |
| `hardware/rtl/` | RiskyBird's own RTL: `fcrocc` (flight-control RoCC and its microcode assembler), `ospi`, `uartdma`, `esplink` |
| `hardware/zephyr/` | devicetree overlays and Kconfig fragments per FPGA target |
| `hardware/{gemmini,fpga-shells,firesim,modelblaster}/` | RiskyBird configs and files for those backends |
| `hardware/pcb/` | the board designs (v2 boards and the v3 base), their docs and bring-up log, and riskybird-academy |
| `hardware/{openocd,spioverjtag}/` | OpenOCD configs and the SPI-over-JTAG flash loader |
| `hardware/{flight,bench}/` | GDB scripts for flight-bench and bench debugging |
| `hardware/padtest/` | bare-metal FPGA pad tests |
| `tools/flight/`, `tools/bench/` | flight recording, ESP32-C6 link firmware, ground station |
| `integration/zephyr/` | shared Zephyr modules: camera, ToF, motor link, motor guard |
| `backends/` | submodules: chipyard, zephyr-chipyard-sw, modelblaster |
| `prune/moved-to-top.tsv` | where each file copied out of a backend came from |
| `tests/` | repository tests |

## Getting started

```sh
git submodule update --init backends/zephyr-chipyard-sw backends/chipyard
source ./rb-env.sh
rb doctor          # tools, environments and attached hardware
rb targets         # FPGA and Zephyr targets
rb workloads       # workloads and which targets they support
rb run --help      # build, program, load and open the console in one step
```

## Documentation

Setup, flight, hardware and workload guides are in the
[RiskyBird Docs notebook](https://www.notion.so/3ea06b97ede480799eb4d570d4d4d4be)
in Notion; the [integration branch section](https://www.notion.so/3ea06b97ede4817fb997d1c5f607c577)
covers this branch. Pages follow the repository's directories. To read or edit
them from a checkout, see [`tools/shynote.md`](../tools/shynote.md).
