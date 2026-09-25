# Repository layout and flow boundaries

What is on the `setup` branch, directory by directory, read from `git ls-tree`
on 2026-09-25. For what lives only on `main`, see
[`merging-into-main.md`](merging-into-main.md).

```text
riskybird-dev/  (setup)
├── rb, rb-env.sh              the entry point; rb-env.sh puts it on PATH for scripts and CI
├── tools/
│   ├── rb/                    every rb flow: build, program, debug, fly, ground, flightlog, ...
│   ├── bench/                 bench helpers: FTDI reset, acceptance check, console logging,
│   │                          the ground-station monitor, dashboard and macOS Wi-Fi script
│   ├── flight/presets/        the flight presets rb fly builds, one .env per configuration that flew
│   ├── flight/legacy/         the campaign scripts rb fly replaced, kept as reference
│   ├── modelblaster/          DroNet model preparation for rb modelblaster
│   ├── collect-bitstreams.py, collect-dts.py
│   │                          publish bitstreams and devicetrees for hosts that cannot build them
│   └── flow-cal.py            optical-flow calibration solver
├── workloads/                 Zephyr applications: sensors, camera, DDR stress, the ESP images
│                              (esp_bridge, esp_motors, motor_safe) and motor diagnostics.
│                              flight_controller is maintained upstream and built in place from
│                              backends/zephyr-chipyard-sw.
├── integration/zephyr/        code shared by workloads: camera driver, motor_link wire format,
│                              safety, ToF
├── hardware/
│   ├── zephyr/                device-tree overlays and Kconfig per board, target and workload
│   ├── openocd/               FT-LINK adapter and per-board target configs
│   ├── flight/                GDB scripts for the flight image: arm, commit, stop, halt, where
│   ├── bench/, modelblaster/  GDB scripts for accelerator and DroNet debugging
│   ├── ospi/                  standalone HM01B0 capture RTL, tests, capture scripts, frame tools
│   ├── pwm/                   FPGA PWM simulation and bench scripts (the legacy motor path)
│   ├── padtest/               bare-metal Verilog motor-pad test and the F13 input probe
│   ├── spioverjtag/           KU040 flash bridge and openFPGALoader setup
│   └── pcb/                   submodule: the board's KiCad files (riskybirdv3-layout branch)
├── artifacts/modelblaster/    committed DroNet kernels for the accelerator shells
├── tests/                     unittest suite for the tooling; see setup.md, "Checking the tooling itself"
├── docs/                      these pages; docs/history/ holds handoffs and one-off records
└── backends/                  submodules, initialised per tier (see .gitmodules)
    ├── zephyr-chipyard-sw/    Zephyr, SDK and the flight controller sample (bench tier)
    ├── chipyard/              hardware generator and FPGA projects (silicon tier, Linux)
    └── modelblaster/          PyTorch -> quantized -> Zephyr pipeline (models tier)
```

## Generated, untracked

```text
build/zephyr/<target>/<workload>/<mode>/zephyr/zephyr.elf   rb build
results/fly/<timestamp>/                                    rb fly session logs
results/riskybird-logs/raw_*.log                            ground-station monitor
.rb/                                                        caches and environments rb creates
bitstreams/, dts/                                           collected shells (or $RB_BITSTREAM_DIR, $RB_DTS_DIR)
```

Chipyard's own bitstreams stay where its build puts them, under
`backends/chipyard/fpga/generated-src/`; `rb chipyard path` prints the selected
one. The build directory is keyed on target, workload and mode but not on
`--config`, so building a second config for the same workload overwrites the
ELF.

## Ownership rule

RiskyBird integration belongs at the repository top level. Backend trees are
inputs: `rb` invokes their build systems and consumes what they generate, but
does not rewrite backend source.

The one bounded exception is `backends/chipyard/riskybird/`. FPGA area
characterisation has to run inside Chipyard's Vivado build, so its collateral
lives in that one directory and hooks in through a single line at the end of
`fpga/Makefile`:

```make
-include $(base_dir)/riskybird/area.mk
```

`-include`, so deleting the directory leaves Chipyard fully functional;
`grep -rn riskybird backends/chipyard/fpga/Makefile` shows the whole footprint,
and `tests/test_area.py` asserts it stays a single hook. The RiskyBird FPGA board
configs in `backends/chipyard/fpga/src/main/scala/` are a separate,
pre-existing divergence, because Chipyard resolves config classes from its own
source tree.
