# RiskyBird setup

Built up one piece at a time. Each step adds exactly one dependency and ends in a command you
can run to prove that step worked -- a step is not done because it installed, it is done because
its verification command printed what this page says it prints.

Two tiers, and you almost certainly want the first:

| Tier | What it does | Steps | Needs | macOS |
| --- | --- | --- | --- | --- |
| **Bench** | build workloads, program the FPGA, debug, run the drone | 1-3 | Python, Zephyr SDK, openFPGALoader, OpenOCD | **yes** |
| **Silicon** | elaborate Chipyard, build bitstreams | 1, then 4 | Vivado, Chipyard | **no -- Linux only** |

Vivado has no macOS release, so a Mac cannot build bitstreams. It can do everything else,
consuming bitstreams and devicetrees built elsewhere. Nothing in the bench tier needs Chipyard
installed at all; an uninitialised submodule is simply an empty directory.

**The two-machine split** this supports, which is the common case: one Linux machine runs step 4
and publishes its output with `tools/collect-dts.py` and `tools/collect-bitstreams.py`; every
other machine -- Macs included -- runs steps 1-3 and points `RB_DTS_DIR` and `RB_BITSTREAM_DIR`
at that output. Neither side needs the other's toolchain.

### What is supported, and what is not yet

The supported surface is **Chipyard and Zephyr**: elaborating a shell, building a workload,
programming a board, and debugging what runs on it. That is what this guide covers and what the
verification at the end of each step exercises.

`rb --help` also lists six commands marked **`[experimental]`**:

| | |
| --- | --- |
| `rose`, `hil` | RoSE co-simulation and hardware-in-the-loop replay |
| `xpurt`, `schedule` | workload-mix scheduling and policy replay |
| `dse`, `area` | design-space exploration and FPGA resource characterisation |

They work here and are used in this repository's own experiments, but they are in progress:
their setup is not documented, and their interfaces are expected to change. Note that step 2b
*does* pull several gigabytes of their dependencies -- `gymnasium`, `torch`, CUDA runtime
libraries, `stable-baselines3` -- into the Zephyr environment as a side effect. None of it is
needed for the supported surface; it is simply part of the environment the backend defines, and
is most of why that environment is ~7 GB. Nothing in the supported surface depends
on them -- their modules are imported lazily, so an absent dependency costs nothing until you
invoke one. Treat them as a preview rather than part of the install.

---

## Step 1 -- `rb`, the entry point

Every flow in this repository goes through one command. On its own it needs **nothing but a
Python interpreter**: the CLI, the board registry and the workload registry are pure standard
library.

### Prerequisites

- **Python 3.9 or newer.** Verified here on 3.13.
  macOS ships an old system Python; install a current one rather than fighting it:
  ```console
  brew install python
  ```
- Nothing else. No pip packages, no virtualenv, no toolchain.

### Get it

```console
git clone --branch setup git@github.com:lorenshung/riskybird-dev.git
cd riskybird-dev
```

`--branch setup` matters: `main` carries an older, shorter version of this same
file, so cloning without it gives you a different page from the one you are
reading.

> **These are private repositories.** This one, and the two side repositories in
> steps 2d and 3c, are cloned over SSH and are not publicly readable. You need a
> GitHub account with access and an SSH key configured, or the very first
> command fails. (`lorenshung/zephyr-chipyard-sw`, fetched in step 2a, is public
> and uses HTTPS.)

### Put `rb` on your PATH

**Do this now.** Every command on this page from here on is written as `rb`, so
without this they fail with `rb: command not found` -- and this page is meant to
be copy-pasted.

```console
export PATH="$PWD:$PATH"
```

Confirm it took, before anything depends on it:

```console
$ rb --version
rb 0.1.0
```

If that says `rb: command not found`, the export did not apply to this shell --
re-run it from the repository root. Put the same line in your shell profile so
later sessions have it too.

`./rb` also works, but only when your working directory is the repository root.

**Do not use `--recursive`.** The backends carry their own nested submodules -- recursing pulls
several gigabytes, including three separate copies of Chipyard, none of which the bench tier
uses. Later steps initialise exactly the submodules they need.

### Verify

Three commands, no hardware and no toolchain required.

```console
$ rb --help
usage: rb [-h] [--version]
          {doctor,targets,workloads,env,chipyard,zephyr,build,flash,probe-ftdi,...}

One entry point for RiskyBird FPGA and Zephyr flows.
```

```console
$ rb targets
TARGET       KIND  ZEPHYR BOARD                                  EVIDENCE
arty100t     fpga  chipyard_riscv64/rocketchip_virt_riscv64      generated and synthesized
  active SoC RAM: 1024 MiB; physical memory: not recorded; ...
arty200t     fpga  chipyard_riscv64/rocketchip_virt_riscv64      generated
  ...
ku040        fpga  chipyard_riscv64/rocketchip_virt_riscv64      generated and synthesized
  ...
(esp32c6, firesim and rose-spike also listed)
```

Every row is followed by an indented detail line; they are elided above.

```console
$ rb workloads
WORKLOAD          MODES          TARGETS                  STATUS/SAFETY
hello_world       default        arty100t,arty200t,ku040,esp32c6,firesim normal
  Console bring-up probe: banner, timed tick loop, and a completion marker a debugger can break on
...
```

### What `rb doctor` should say right now

`rb doctor` is the running scorecard for the whole install. After step 1 it is **supposed** to be
almost entirely `MISSING` -- that is the correct output, not a failure:

```console
$ rb doctor
RiskyBird rb 0.1.0
repository       /path/to/riskybird-dev
Chipyard backend  MISSING .../backends/chipyard
Chipyard env      MISSING .../backends/chipyard/.conda-env
Zephyr backend    MISSING .../backends/zephyr-chipyard-sw
Zephyr env        MISSING .../tools/miniforge3/envs/zephyr
Zephyr base       MISSING .../zephyr_ws/zephyr
Zephyr SDK        MISSING .../tools-manual/zephyr-sdk-1.0.0-beta1
Espressif HAL     MISSING .../zephyr_ws/modules/hal/espressif
ST HAL            MISSING .../zephyr_ws/modules/hal/st
west              MISSING
RISC-V compiler   MISSING .../riscv64-zephyr-elf-gcc
Espresso          MISSING
Vivado            ok      /ecad/tools/xilinx/Vivado/2023.1/bin/vivado
OpenOCD           MISSING .../hosttools/.../usr/bin/openocd
GDB               MISSING .../riscv64-zephyr-elf-gdb
hardware contracts
  arty200t     pending generated DTS not present
serial devices
  ...
```

Each later step turns one of those lines green. What matters here is only that the command runs,
prints your repository path, and does not raise.

**`rb doctor` exits non-zero while anything is missing**, so after step 1 it returns 1. That is
the scorecard reporting an incomplete install, not the command failing -- do not wrap it in a
shell test yet. (It currently has no way to say "complete *for the bench tier*", which is why a
Mac install will keep reporting the Vivado and Chipyard lines forever. That is a known rough
edge, not a problem with your install.)

**Most `MISSING` lines name the path that was expected**, so they double as the answer to "where
is it supposed to go". Two do not: `west` and `Espresso` are looked up on a `PATH` rather than at
a fixed location, so they print `MISSING` with nothing after it. The SDK line naming `x86_64-pokysdk-linux` on a Mac is not a bug -- with no SDK
installed there is nothing to detect yet, so it reports the Linux default; once the SDK is
installed, step 2 finds the right sysroot for your host automatically.

### Optional: FTDI discovery on macOS

`rb probe-ftdi` and `rb bench` report which USB adapter is the bitstream link and which is the
debug link. On Linux they read `/sys/bus/usb`, which needs nothing installed.

macOS has no sysfs, so those two commands need libusb:

```console
brew install libusb
pip install pyusb
```

`rb` computes the same bus-and-port location string OpenOCD's own libusb backend uses
(`7-1.1.4.1`), so the value it prints is the one to pass to `--usb-location`.

**Verified**, rather than assumed: the libusb path was run against a real FT4232H and compared
to the sysfs path on the same device. Identical location, bus:device, PID, serial and product
string; only the interface column differs.

It does **not** report which serial node belongs to which channel -- that is a Linux `ftdi_sio`
property, so the libusb path prints `if0=?` rather than guessing. On macOS match them by the
trailing letter of the device name: `/dev/cu.usbserial-<serial>A` is channel A (JTAG), `...B` is
channel B (console).

You can skip this until you have hardware attached.

---

## Step 2 -- the Zephyr backend, ending in a built ELF

Adds the toolchain. After this step `rb build` produces a real ELF, with no FPGA
attached and -- on the bench tier -- no Chipyard installed.

**Budget ~16 GB of disk and around half an hour**, most of it unattended
downloads. The bulk lands in `backends/zephyr-chipyard-sw`: ~4 GB SDK, ~7 GB
conda environment, ~3 GB Zephyr workspace.

### Host prerequisites

Both installers hard-require these, and neither checks usefully:

- **`wget`**
- **`cmake`** -- and it must be a *working* one. See the warning below.

> **Check what `cmake` you have before starting:**
> ```console
> $ which cmake && cmake --version
> ```
> If it resolves to something under `/ecad`, Vitis or another vendor toolchain,
> put a real one first: `export PATH=/usr/bin:$PATH`. A Xilinx-shipped cmake
> satisfies `command -v cmake` and then dies with
> `cmake: error while loading shared libraries: libidn.so.11`, failing SDK
> installation at the *"Register CMake package"* step with exit 40.
>
> Fixing `PATH` is the whole remedy -- do it before step 2c and the installer
> runs clean.
>
> Note that step 2b installs a pinned `cmake<4` into the Zephyr *environment*,
> and that is the cmake `west build` uses later. It does **not** protect step 2c:
> 2c is a plain shell script that never activates that environment, so whatever
> `cmake` your shell resolves is what it gets.

### 2a. Initialise only the backend you need

```console
git submodule update --init backends/zephyr-chipyard-sw
```

**Not `--recursive`.** The backend has its own nested submodules; 2b brings up
exactly the ones it needs.

The bench tier does not initialise `backends/chipyard` at all. An uninitialised
submodule is just an empty directory -- nothing is deleted, nothing is
special-cased.

### 2b. Build the Zephyr environment

```console
rb env setup zephyr
```

This runs the backend's `install_conda.sh` and `install_submodules.sh`: it
creates a conda environment under `backends/zephyr-chipyard-sw/tools/miniforge3`,
installs **`west`** and a pinned `cmake<4` plus `ninja`, and checks out the
nested submodules including the Zephyr tree itself. Takes several minutes.

Nothing later works without this. `rb build` shells out to `west`, so skipping
it fails with a bare `No such file or directory: 'west'` several steps
downstream.

> **This step edits `~/.condarc`**, which is user-global: it adds the
> `conda-forge` channel, sets `channel_priority: strict` and switches the solver
> to `libmamba`. That changes solver behaviour for every other conda environment
> you own, not just this one. Back the file up first if you care about it.
>
> It also appends the new environment to `~/.conda/environments.txt` and touches
> `~/.wget-hsts`. Both are harmless; listed so that nothing this install does
> outside the checkout is a surprise.

> **If you have a conda environment active, deactivate it first.** The
> installer only bootstraps its own Miniforge when `CONDA_DEFAULT_ENV` is unset;
> with one active it builds into *that* environment instead -- putting ~7 GB
> into your base install rather than into the checkout.
> ```console
> conda deactivate          # repeat until `echo $CONDA_DEFAULT_ENV` is empty
> ```
> If that returns `CondaError: Run 'conda init' before 'conda deactivate'`, the
> shell has the environment variables but not the shell function -- usual in a
> non-interactive shell. Load it first, then deactivate:
> ```console
> source "$(dirname "$(dirname "$(command -v conda)")")/etc/profile.d/conda.sh"
> ```
> Check the path it reports: it should be
> `backends/zephyr-chipyard-sw/tools/miniforge3`.

### 2c. Install the toolchain

```console
cd backends/zephyr-chipyard-sw
./scripts/install_toolchain_sdk.sh
cd -
```

The installer picks the right prebuilt tarball for your host:

| host | tarball |
| --- | --- |
| Linux x86_64 | `zephyr-sdk-1.0.0-beta1_linux-x86_64_minimal.tar.xz` |
| **macOS Apple Silicon** | `zephyr-sdk-1.0.0-beta1_macos-aarch64_minimal.tar.xz` |
| macOS Intel | `zephyr-sdk-1.0.0-beta1_macos-x86_64_minimal.tar.xz` |

`RB_SDK_HOST` overrides for a host it does not know. Getting this wrong is worth
avoiding: the Linux tarball extracts perfectly well on macOS and then every tool
fails with *"bad CPU type in executable"*, which reads like a corrupt download
rather than the wrong package.

> **This step writes outside the checkout.** It registers the SDK in the
> *user-global* `~/.cmake/packages/Zephyr-sdk/`, one file per SDK, named for the
> md5 of that SDK's path. So a second checkout **adds** a second entry rather
> than replacing the first, and the directory accumulates one per checkout you
> ever install.
>
> None of this affects `rb`, which passes `ZEPHYR_SDK_INSTALL_DIR` explicitly and
> never consults the registry. It matters only if you run `west build` by hand
> outside `rb`, where CMake may then have several registered SDKs to choose
> between. Old entries pointing at deleted checkouts are safe to delete.

`rb` finds the SDK's host tools by looking inside the SDK, so no path needs
editing afterwards.

### 2d. Supply an elaborated devicetree

**This is the step that lets a Mac build at all**, and it has no equivalent in a
Linux silicon-tier install.

`rb` reads the elaborated DTS to derive a shell's PLIC sources, the periphery it
carries and its memory map. Chipyard produces that file -- but consulting it
needs only the file, not the toolchain. So a host that cannot elaborate can use
one collected from a host that can:

```console
git clone git@github.com:lorenshung/dts.git ~/riskybird-dts
export RB_DTS_DIR=~/riskybird-dts
```

Layout is `<board>/<board>.<Config>.dts`. `rb` prefers a local Chipyard
elaboration when one exists, so on a machine with Chipyard this is ignored.

> `RB_DTS_DIR` must be set in **every** shell that runs `rb`, not just this one.
> Put it in your shell profile. Without it `rb doctor` reports
> `pending  generated DTS not present` and `rb build` refuses to start.

Regenerate the collection from a machine that has Chipyard with
`tools/collect-dts.py`.

### Verify

`rb doctor` should now report the Zephyr lines `ok`. **Check the `west` line
specifically** -- it is the one whose absence stops `rb build`:

```console
$ rb doctor
Zephyr backend    ok      .../backends/zephyr-chipyard-sw
Zephyr env        ok      .../tools/miniforge3/envs/zephyr
Zephyr base       ok      .../zephyr_ws/zephyr
Zephyr SDK        ok      .../tools-manual/zephyr-sdk-1.0.0-beta1
west              ok      .../envs/zephyr/bin/west
RISC-V compiler   ok      .../riscv64-zephyr-elf-gcc
hardware contracts
  arty200t     ok      config RocketArty200TOspiConfig, RAM 0x80000000+0x40000000, UART 0x10020000 irq 2, I2C 0x10040000 irq 1
```

`Chipyard backend` and `Chipyard env` stay `MISSING` on the bench tier, and
`rb doctor` still exits non-zero because of them. That is correct. `Vivado` may
be `ok` or `MISSING` depending on the machine; the bench tier does not use it.

Then build something:

```console
$ rb build hello_world --target arty200t --config RocketArty200TDroneFullDDRConfig
...
Memory region         Used Size  Region Size  %age Used
             RAM:       19424 B         1 GB      0.00%
```

An ELF at `build/zephyr/arty200t/hello_world/default/zephyr/zephyr.elf` is the
end of step 2. Nothing so far has needed hardware.

> **Verified 2026-09-05, twice, from a clean clone of this branch into an empty
> directory**, each time by someone following only this page with no access to
> any other checkout. Steps 1 and 2 completed both times with no undocumented
> workarounds beyond the `PATH` and `conda deactivate` remedies prescribed
> above. Every figure quoted on this page -- the doctor lines, the
> `19424 B / 1 GB`, the `32 KB / 59.28%`, the ~16 GB footprint -- was reproduced
> exactly.
>
> Step 3 has **not** been verified that way and cannot be without an FPGA board
> attached; see the note at the end of it.

That `1 GB` region is the proof step 2d worked: it comes from the
`RocketArty200TDroneFullDDRConfig` devicetree. Build
`--config RocketArty200TSpadConfig` instead and the same workload reports
`32 KB` and `59.28%`, because that shell presents a 32 KiB scratchpad rather
than DDR.

> **The build directory is keyed on target, workload and mode -- but not on
> config.** Building a second config for the same workload overwrites the ELF at
> that path. If you build two, note which one is currently there.

---

## Step 3 -- program a board and open a console

Adds the two host tools that talk to hardware, and the bitstream to load. This
is the first step that needs the board attached.

### 3a. openFPGALoader

Not vendored -- it is a normal package:

```console
brew install openfpgaloader          # macOS
sudo apt install openfpgaloader      # Debian/Ubuntu
```

`hardware/spioverjtag/setup-openfpgaloader.sh` builds it from source if your
distribution's package is too old; set `OFL_PREFIX` to choose where it lands.

### 3b. OpenOCD

The Zephyr SDK from step 2 ships one, and `rb` uses it automatically. If your
SDK has none, install OpenOCD yourself (`brew install open-ocd`) -- `rb` falls
back to whatever is on `PATH`.

There is nothing to configure. `rb` looks inside the SDK for the host-tools
sysroot belonging to *your* machine, so the same command works on Linux and
macOS.

### 3c. Supply a bitstream

Same split as the devicetree in step 2d, and for the same reason: **programming
a board needs the bitstream file, not the toolchain that synthesised it.**

```console
git clone git@github.com:lorenshung/bitstream.git ~/riskybird-bitstreams
export RB_BITSTREAM_DIR=~/riskybird-bitstreams
```

Layout is `<board>/<board>.<Config>.bit`. That naming is what makes the fallback
work at all: Chipyard names every bitstream after its harness, so a directory of
`Arty200THarness.bit` files would be indistinguishable once moved.

`rb` prefers a local Chipyard build when one exists. Regenerate the collection
from a machine that has Chipyard with `tools/collect-bitstreams.py`.

> Like `RB_DTS_DIR`, this must be set in **every** shell that runs `rb`. Put both
> in your shell profile.

Not every config has a bitstream. A shell has to have been *synthesised* for one
to exist, and several are only ever elaborated -- enough for `rb build`, not
enough to program. Check before relying on one:

```console
$ ls ~/riskybird-bitstreams/arty200t/
arty200t.CameraBringup200TConfig.bit
arty200t.DualRocketArty200TOspiConfig.bit
arty200t.RocketArty200TConfig.bit
arty200t.RocketArty200TDroneDDRConfig.bit
arty200t.RocketArty200TDroneFullDDRConfig.bit
...
```

> **On arty200t you must pass `--config`.** `rb doctor`'s contract line names
> each board's *default* config, which is what `rb program --board <board>` uses
> when you give no `--config` -- and arty200t's default,
> `RocketArty200TOspiConfig`, has been elaborated but never synthesised, so it
> has no bitstream in the collection. arty100t and ku040 do have theirs.
>
> Use a config you have confirmed above, e.g. the one step 2 builds for:
> ```console
> rb program --board arty200t --config RocketArty200TDroneFullDDRConfig --usb-location <loc>
> ```

### 3c-bis. USB and serial access (Linux)

Step 3 talks to FTDI adapters and serial ports. On Linux that needs group
membership the installer mentions in passing and this guide previously did not:

```console
sudo usermod -aG plugdev,dialout "$USER"
```

Log out and back in for it to take effect. Without it `rb probe-ftdi` and
`rb console` fail on permissions, which looks like absent hardware.

macOS needs no equivalent.

### 3d. Find the adapters

The drone bench has two FT2232H adapters: one reaches the FPGA's configuration
TAP (bitstream), the other the Rocket debug TAP (ELF loading, and the console on
its channel B). Which is which is **not stable across replugs**, so read it
rather than remember it:

```console
$ rb bench
ROLE       LOCATION   BUS:DEV  JTAG chA       CONSOLE chB    IDCODE
debug      7-1.1.4.1  7:22     detached       /dev/ttyUSB5   0x00000001  Rocket debug TAP
bitstream  7-1.1.4.3  7:21     detached       /dev/ttyUSB0   0x13636093  xc7a200t config TAP
```

On Linux this needs nothing installed. **On macOS it needs pyusb** (step 1's
optional section): there is no sysfs, so `rb` asks libusb instead. It reports
the same bus-and-port location string OpenOCD matches, so the value is
interchangeable.

Note the debug TAP reads `0x00000001` only **after** the part is configured.
Before that it scans all ones, which is correct behaviour and not a fault --
probe with a bitstream already loaded or the debug link looks dead.

> `rb bench` runs a JTAG IDCODE scan on **every** FTDI device attached to the
> host, not just the drone bench's, in order to work out which is which. It
> labels foreign adapters clearly (an Alveo card shows up as
> `xcu250 config TAP -- Alveo card, not the drone bench`), but on a machine with
> other FPGA hardware attached, be aware it touches those adapters too -- and
> see 3e on why the first scan after enumeration is the reliable one.

### 3e. Recovering a wedged adapter

Only the first OpenOCD run after an adapter enumerates is reliable. Afterwards
the MPSSE engine can be left in a state where the JTAG scan returns all zeroes
or all ones -- which reads exactly like a broken cable or an unconfigured part.

```console
python3 tools/bench/usbreset.py 7-1.1.4.1
```

Linux uses the `USBDEVFS_RESET` ioctl; macOS has no usbfs, so the same script
uses libusb through pyusb. The location argument is identical on both.

If a scan returns **all zeroes**, suspect that the part lost its configuration
(power cycled, or a programming run was interrupted) before suspecting the
cable.

### Verify

```console
rb program --board arty200t --config <Config> --usb-location <bitstream-location>
rb console --port /dev/ttyUSB5 --baudrate 115200
```

> **Status of this section: recorded, not verified.** Tool discovery, bitstream
> resolution and adapter probing have been exercised on this checkout. The
> `rb program` and `rb console` commands are transcribed from a working bench
> and have **not** been re-run here, because no board was attached when this was
> written. Steps 1 and 2 have been run from a clean clone; step 3 has not.

---

## Step 4 -- build a bitstream (silicon tier, Linux only)

Everything above *consumes* bitstreams built elsewhere. This step builds them.

**Linux only.** Vivado has no macOS release, so a Mac cannot do this step at all.
On a Mac, someone else runs step 4 and you consume the output through steps 2d
and 3c.

### 4a. Initialise Chipyard

```console
git submodule update --init backends/chipyard
```

Not `--recursive`: Chipyard initialises its own nested submodules during 4b, and
recursing here pulls a large set its setup deliberately skips.

This checkout is far larger than the bench tier's, and 4b is a long install.
Measured on the machine these figures come from:

| | |
| --- | --- |
| `backends/chipyard` after 4b, before any build | **~50 GB** (of which ~8 GB is its conda environment) |
| each config you elaborate or build | **~0.5 GB** under `fpga/generated-src/` |

So budget ~50 GB for the silicon tier on top of step 2's ~16 GB, and expect
`generated-src/` to grow with every config you touch -- it is a per-config
directory, not a shared one, and nothing prunes it.

### 4b. Chipyard's environment

```console
rb env setup chipyard
```

Runs Chipyard's own `build-setup.sh --use-lean-conda`, building a locked conda
environment at `backends/chipyard/.conda-env`. Check it:

```console
$ rb env status
chipyard  ok      .../backends/chipyard/.conda-env
zephyr    ok      .../tools/miniforge3/envs/zephyr
```

### 4c. Vivado

Not vendored and not installed by anything here -- it is a licensed Xilinx tool
you install yourself. `rb` defaults to `/ecad/tools/xilinx/Vivado/2023.1/bin`,
which is one particular bench's path and almost certainly not yours:

```console
export RB_VIVADO_BIN=/path/to/Vivado/2023.1/bin
```

Confirm `rb` sees it:

```console
$ rb doctor
Vivado            ok      /path/to/Vivado/2023.1/bin/vivado
Vivado version            vivado v2023.1 (64-bit)
```

### 4d. Elaborate, then build

Elaboration is minutes and catches configuration errors without spending an hour
in Vivado, so do it first on a config you have not built before:

```console
rb chipyard elaborate --board arty200t --config RocketArty200TDroneFullDDRConfig
```

It writes the `.dts` that step 2d distributes. Then the bitstream:

```console
rb chipyard build --board arty200t --config RocketArty200TDroneFullDDRConfig
```

This runs `make bitstream` inside Chipyard's environment. A DDR-backed config
takes on the order of an hour -- the MIG dominates -- and a scratchpad config is
considerably faster. The result lands at

```
backends/chipyard/fpga/generated-src/<generated-name>/obj/<Harness>.bit
```

which is exactly where `rb program` looks first.

> **A config that "fails" the contract check has still built.** `NoCores*Config`
> elaborates cleanly and then reports *"elaborated successfully, but the
> generated DTS is not a deployable RiskyBird target"* -- it has no cores and no
> UART by design. That is a correct rejection of a config that was never a
> deployment target, not a broken build. The same config exists for all three
> boards and behaves the same way.

### 4e. Publish for hosts that cannot build

This is what makes the bench tier work on a Mac.

```console
python3 tools/collect-dts.py         --out ~/riskybird-dts
python3 tools/collect-bitstreams.py  --out ~/riskybird-bitstreams
```

Each writes `<board>/<board>.<Config>.<ext>` plus a `manifest.json` recording
the source revisions -- and for bitstreams, resource and timing figures, with
any config that missed timing flagged.

The renaming is the point. Chipyard names every bitstream after its *harness*,
so a directory of `Arty200THarness.bit` files is indistinguishable once moved;
naming by board and config is what lets `RB_BITSTREAM_DIR` and `RB_DTS_DIR`
resolve a specific shell on a machine that never built it.

Commit and push those two directories to their side repositories, and a Mac
picks them up through steps 2d and 3c.

Only *synthesised* configs produce bitstreams. A config that was elaborated but
never built appears in the DTS collection and not the bitstream one -- which is
precisely the situation step 3c warns about for `arty200t`'s default config.
