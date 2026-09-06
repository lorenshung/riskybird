# Repository layout and flow boundaries

```text
riskybird-dev/
├── rb                         single user entry point
├── tools/rb/                  flow orchestration and environment adapters
├── workloads/
│   ├── state/                 Madgwick state estimation: replay or BMI088
│   ├── mpc/                   scalar TinyMPC: replay or binary UART HIL
│   └── */                     board-independent sensor/motor diagnostics
├── benchmarks/
│   ├── schemas/               versioned record and study schemas
│   ├── state/                 RiskyBird estimator inputs and expectations
│   └── mpc/                   RiskyBird control inputs and expectations
├── experiments/
│   ├── studies/               named config sweeps, one question each
│   └── fpga-configs/          reference archive of imported area work
├── dse/                       FPGA design-space exploration: docs, descriptors, scripts
├── integration/               stable contracts between workloads and backend drivers
├── tests/                     host-only unit tests, no backend environment needed
├── hardware/
│   ├── zephyr/                target wiring, DTS bindings, and FPGA Kconfig
│   ├── openocd/               FT-LINK and one-hart Rocket debug targets
│   └── ospi/                  standalone HM01B0 capture RTL and tests
├── results/                   generated run output (untracked)
├── docs/                      integration documentation
└── backends/
    ├── chipyard/              pinned hardware generator and FPGA projects
    │   └── riskybird/         the ONLY RiskyBird code inside this backend
    ├── zephyr-chipyard-sw/    pinned Zephyr tree, SDK, and dependencies
    ├── RoSE/                  deferred simulation backend
    └── XPU-RT/                deferred simulation/runtime backend
```

## Ownership rule

RiskyBird integration belongs at the repository top level. Backend trees are
inputs, not places to accumulate project glue. The CLI may invoke their build
systems and consume generated artifacts, but does not rewrite backend source.
The standalone HM01B0 receiver is hardware collateral, so it lives under
`hardware/ospi` rather than being presented as a fifth backend. Chipyard keeps
the SoC-attached copy required by its own generator.

### The one bounded exception: `backends/chipyard/riskybird/`

FPGA area characterization has to run *inside* Chipyard's Vivado build, because
it needs Chipyard's elaborated RTL, generated IP TCL, source manifest, and board
tables. Reimplementing that plumbing at the top level would duplicate backend
logic — worse than one contained exception.

So all RiskyBird collateral in that backend lives in one clearly named
directory, and hooks in through exactly one line at the end of `fpga/Makefile`:

```make
-include $(base_dir)/riskybird/area.mk
```

`-include` rather than `include`, so deleting the directory leaves Chipyard fully
functional. The complete footprint is discoverable with:

```console
grep -rn riskybird backends/chipyard/fpga/Makefile
```

`tests/test_area.py` asserts that this hook stays present, stays a single
`-include`, and that RiskyBird has not patched Chipyard's shared bitstream flow.
The RiskyBird-owned FPGA board configs in
`backends/chipyard/fpga/src/main/scala/{arty100t,ku040}/Configs.scala` remain a
separate, pre-existing divergence on the `trenz` branch: a board shell is
backend-shaped, and Chipyard's FPGA build resolves config classes from its own
source tree.

RoSE, XPU-RT, Isaac Sim, and FireSim adapters should consume stable top-level
contracts rather than moving or rewriting backend-owned infrastructure. See the
[full-stack platform roadmap](README.md).

## Hardware contract

The default RiskyBird OSPI configs use a single 50 MHz Rocket hart, the SiFive
UART at `0x10020000` (PLIC source 2), and I2C at `0x10040000` (PLIC source 1).
DroneLogic configs additionally instantiate two PWM blocks, a three-bit GPIO
block, and SPI. `rb` selects the matching Zephyr overlay and validates the
addresses, interrupts, and memory range against Chipyard's generated DTS
before a build.

The backend Zephyr board describes a generic eight-hart virtual machine.
Top-level overlays disable the extra harts, correct the physical interrupts,
select the SiFive UART, add BMI088 nodes, and provide the board-specific memory
size. The DroneLogic PLIC has sources 1 through 15, so the Zephyr software ISR
table contains 27 entries: 12 primary interrupt slots plus 15 second-level
PLIC sources.

Every KU040 config now carries the board's 2 GiB of DDR4; the 32 KiB scratchpad
variants are retained only to keep that memory map buildable. MPC preserves the
12-state model and controller matrices but uses a three-step horizon on KU040
against Arty100T's ten-step, a holdover from the scratchpad era that the DDR4
configs make reviewable.

## Artifact layout

Generated top-level artifacts are untracked:

```text
build/zephyr/<target>/<workload>/<mode>/zephyr/zephyr.elf
results/runs/<timestamp>_<target>_<workload>_<mode>/
.rb/ccache/
```

Chipyard bitstreams remain where its build system produces them under
`backends/chipyard/fpga/generated-src/`. `rb chipyard path` and
`rb zephyr path` print the exact selected artifacts. Chipyard commands accept
`--config CLASS`; the selected class is part of the generated directory name,
so build, path lookup, programming, debug, and full-run flows all resolve the
same config-specific bitstream. Without the option, the board's RiskyBird OSPI
config remains selected.

## Flow composition

- `rb chipyard elaborate` generates config-specific RTL and checks the DTS
  contract.
- `rb chipyard build` produces the FPGA bitstream.
- `rb program` configures the FPGA (openFPGALoader for Arty by default,
  Vivado Hardware Manager for KU040).
- `rb build` invokes `west build` with the exact overlays and mode config.
- `rb flash` writes an existing ESP image; ESP `rb run` composes build, flash,
  and console.
- `rb load` writes the ELF to RAM over UARTTSI.
- `rb console` opens the SiFive UART monitor.
- `rb capture` records serial data, operator input, firmware identity, and
  repository revisions.
- `rb debug` supervises OpenOCD, connects GDB, and optionally loads the ELF.
- `rb run` composes build, FPGA programming, UARTTSI load, and console.

FPGA PMW3901 and motor `rb run` flows remain disabled until the logical
SPI/GPIO/PWM ports are routed through board-specific harness binders and
verified package constraints. See
`hardware/zephyr/targets/README.md` for the logical and physical distinction.

OpenOCD is launched through the SDK's current dynamic loader because the SDK
in this checkout was relocated and its binary retains its original absolute
interpreter path. This workaround is confined to the environment adapter.
