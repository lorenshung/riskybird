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
| `workloads/` | Zephyr workloads: `flight_controller`, `fc_dronet`, `bootup_check`, motor and ESP applications |
| `workloads/bringup/` | sensor, camera and actuator bring-up and test workloads |
| `workloads/probes/` | RVV (vector unit) probe workloads |
| `workloads/lib/` | shared Zephyr libraries the workloads compile in: camera, flightrec, motor link, motor guard (`safety`), ToF |
| `hardware/chipyard/` | RiskyBird Chipyard configs, IO binders, the FPGA build wrapper, and `patches/`, the patch queue `tools/backend-patches` applies to `backends/chipyard` |
| `hardware/rtl/` | RiskyBird's own RTL: `fcrocc` (flight-control RoCC and its microcode assembler), `ospi`, `uartdma`, `esplink` |
| `hardware/zephyr/` | devicetree overlays and Kconfig fragments per FPGA target |
| `hardware/{gemmini,fpga-shells,firesim,modelblaster}/` | RiskyBird configs and files for those backends |
| `hardware/pcb/` | submodule (lorenshung/riskybird `riskybirdv3-layout`; `git submodule update --init hardware/pcb`): the board designs (v2 boards and the v3 base), their docs and bring-up log, and riskybird-academy |
| `hardware/{openocd,spioverjtag}/` | OpenOCD configs and the SPI-over-JTAG flash loader |
| `hardware/padtest/` | bare-metal FPGA pad tests |
| `tools/flight/` | flight recording, ESP32-C6 link firmware, flight-bench GDB scripts (`gdb/`) |
| `tools/bench/` | ground station, bench scripts, bench GDB scripts (`gdb/`) |
| `backends/` | submodules: chipyard, zephyr-chipyard-sw, modelblaster and RoSE at plain upstream pins; saturn at the Saturn commit RiskyBird's accelerator shells need |
| `backends/README.md` | how to develop Chipyard, Zephyr, ModelBlaster and the sim from here without moving a pin |
| `backends/overlays`, `backends/redirects` | what `tools/backend-patches` and `tools/submodule-redirects` put into the backends |
| `tests/` | repository tests; `tests/data/provenance.tsv` records where each file copied out of a backend came from |

## Getting started

```sh
git submodule update --init backends/zephyr-chipyard-sw backends/modelblaster backends/chipyard backends/saturn
source ./rb-env.sh
rb env setup zephyr && rb env setup chipyard   # also applies the redirects, overlay and patch queue
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
them from a checkout, see [`tools/shynote.md`](tools/shynote.md).
