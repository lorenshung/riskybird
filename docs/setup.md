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

### Set up your shell

**Do this now.** Every command on this page from here on is written as `rb`, so
without it they fail with `rb: command not found`.

```console
source ./rb-env.sh
```

That is the whole of it *for now*. `rb-env.sh` applies everything this page
would otherwise ask you to export one at a time, is safe to source repeatedly,
sets nothing you have already set, and warns you about both of step 2's
environment hazards before you meet them:

```console
$ source ./rb-env.sh
rb-env: cmake resolves to /ecad/tools/xilinx/Vitis/2023.1/tps/lnx64/cmake-3.3.2/bin/cmake
rb-env: that one is likely broken; putting /usr/bin first
rb-env: conda environment 'base' is active.
rb-env: run 'conda deactivate' before setup.md step 2b.
rb-env: rb on PATH
```

Confirm it took:

```console
$ rb --version
rb 0.1.0
```

**Source it in every shell that runs `rb`** -- including each command if you are
driving this from a script, a Makefile, CI or an agent, since those get a fresh
environment every time. For an interactive shell you can put it in your profile
instead.

**Re-source it after steps 2d and 3c.** `RB_DTS_DIR` and `RB_BITSTREAM_DIR` are
exported only if those directories already exist, so sourcing here -- before the
side repositories are cloned -- correctly leaves both unset:

```console
$ source ./rb-env.sh
$ echo "[$RB_DTS_DIR] [$RB_BITSTREAM_DIR]"
[] []
```

Source it again once each clone lands and the variables appear.

<details><summary>Doing it by hand instead</summary>

Everything `rb-env.sh` does is a few exports, if you would rather see them:

```console
export PATH="$PWD:$PATH"                 # rb itself
export RB_DTS_DIR=~/riskybird-dts        # step 2d, once you have it
export RB_BITSTREAM_DIR=~/riskybird-bitstreams   # step 3c, once you have it
```

plus the `PATH=/usr/bin:$PATH` cmake fix and the `conda deactivate` in step 2.
`./rb` also works in place of `rb`, but only from the repository root.

</details>

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
dtc               ok      /usr/bin/dtc
RISC-V compiler   MISSING .../riscv64-zephyr-elf-gcc
Espresso          MISSING
Vivado            ok      /ecad/tools/xilinx/Vivado/2023.1/bin/vivado
OpenOCD           ok      /usr/bin/openocd
GDB               MISSING .../riscv64-zephyr-elf-gdb
Vivado version            vivado v2023.1 (64-bit)
OpenOCD version           Open On-Chip Debugger 0.12.0
hardware contracts
  arty100t     pending generated DTS not present
  arty200t     pending generated DTS not present
  ku040        pending generated DTS not present
serial devices
  ...
```

Three of those lines depend on your machine rather than on this step. `Vivado`,
`OpenOCD` and `dtc` report `ok` if you happen to have them installed
system-wide and `MISSING` if you do not. The first two do not matter yet.
**`dtc` does** -- it is a hard requirement of step 2's build, so if it says
`MISSING`, install it now (see the prerequisites at the top of step 2). The
`version` lines print only for whatever was found.

Each later step turns one of those lines green. What matters here is only that the command runs,
prints your repository path, and does not raise.

**`rb doctor` exits non-zero while anything is missing**, so after step 1 it returns 1. That is
the scorecard reporting an incomplete install, not the command failing -- do not wrap it in a
shell test yet. (It currently has no way to say "complete *for the bench tier*", which is why a
Mac install will keep reporting the Vivado and Chipyard lines forever. That is a known rough
edge, not a problem with your install.)

**Most `MISSING` lines name the path that was expected**, so they double as the answer to "where
is it supposed to go". Two do not: `west` and `Espresso` are looked up on a `PATH` rather than at
a fixed location, so they print `MISSING` with nothing after it.

**`west` may already say `ok` here, and if it does it is a false green.** If you
have ever `brew install`ed or `pip install`ed west, the doctor finds *that* one
-- on a Mac typically `ok /opt/homebrew/bin/west`. It is not the west this
project builds with, so step 2's "check the `west` line specifically"
verification would pass for the wrong reason. Step 2b installs west into the
project's conda environment and the line re-resolves there; the check is only
meaningful after 2b.

### Optional: FTDI discovery on macOS

`rb probe-ftdi` and `rb bench` report which USB adapter is the bitstream link and which is the
debug link. On Linux they read `/sys/bus/usb`, which needs nothing installed.

macOS has no sysfs, so those two commands need libusb:

```console
brew install libusb
python3 -m pip install --break-system-packages --user pyusb
```

Two traps here, both of which bite if you follow this page top to bottom:

- A Homebrew python is *externally managed*, so a plain `pip install pyusb`
  stops with `error: externally-managed-environment`. Hence the flags above.
  (There may also be no bare `pip` on `PATH` at all -- use `python3 -m pip`.)
- **Install it into the interpreter `rb` will actually run under.** `rb` starts
  with `#!/usr/bin/env python3`. If you run this step while a conda environment
  is active -- which is the state `rb-env.sh` warns you about -- pyusb lands in
  the conda python, and then step 2b tells you to `conda deactivate`, after
  which `rb` resolves to a *different* python and reports
  `ModuleNotFoundError: No module named 'usb'`. Either `conda deactivate` before
  installing pyusb, or reinstall it afterwards.

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

**Budget ~16 GB of disk on Linux, ~9 GB on macOS, and around half an hour**,
most of it unattended downloads. The bulk lands in
`backends/zephyr-chipyard-sw`. Measured, on each platform:

| | Linux | macOS (Apple Silicon) |
| --- | --- | --- |
| whole checkout after step 2 | ~16 GB | 8.3 GB |
| SDK (`tools-manual`) | ~4 GB | 3.3 GB |
| conda environment | ~7 GB | 851 MB |
| Zephyr workspace (`zephyr_ws`) | ~3 GB | 2.9 GB |

macOS is much smaller for one reason: the torch/CUDA/pybullet stack in the
conda environment is Linux-only and does not install (see 2b below). If you are
on a Mac and your conda environment is 7 GB, something unexpected happened.

Steps 2d and 3c clone two further repositories into your **home** directory
rather than the checkout -- **~400 MB**, nearly all of it bitstreams (the DTS
repository is under 1 MB). This grows as bitstreams are added.

### Host prerequisites

Both installers hard-require these, and neither checks usefully:

- **`wget`**, **`tar`**, **`git`** -- the installers check for these and stop if
  any is missing.
- **`cmake`** -- and it must be a *working* one. See the warning below.
- **`dtc`**, the devicetree compiler (`brew install dtc`,
  `apt install device-tree-compiler`). 2b checks for it, and `rb build` cannot
  run without it. It is often already present, which is why it is easy to miss
  that it is a real dependency.
- **Java** is needed only by `hardware/ospi`'s mill test suite, not by the
  install itself.

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
> `command -v conda` will still point at *your* conda -- expected, and exactly
> why the deactivate is needed. What matters is where 2b then installs: watch its
> output for
> `Installing Miniforge to .../backends/zephyr-chipyard-sw/tools/miniforge3`.
> That directory does not exist until 2b creates it, so do not go looking for it
> beforehand.

> **On macOS, 2b ends with one failure line and still exits 0.** This is
> expected and safe to ignore:
> ```
> [hh:mm:ss] ERROR: gym-pybullet-drones installation failed - continuing with other installations
>   - gym-pybullet-drones installation failed (pybullet build error).
> [hh:mm:ss] Installation completed with 1 warning(s)
> ```
> `gym-pybullet-drones` is part of the experimental simulation surface, not the
> build path, and its `pybullet` dependency does not build on Apple Silicon.
> Nothing in steps 2c through 3 uses it. This is also why a macOS conda
> environment is ~850 MB rather than the ~7 GB it is on Linux.
>
> The installer also prints Linux-only advice on macOS:
> `sudo usermod -aG plugdev,dialout "$USER"`. There is no `usermod` on macOS and
> none of it is needed there -- see 3c-bis.

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
| Linux aarch64 | `zephyr-sdk-1.0.0-beta1_linux-aarch64_minimal.tar.xz` |
| **macOS Apple Silicon** | `zephyr-sdk-1.0.0-beta1_macos-aarch64_minimal.tar.xz` |
| macOS Intel | `zephyr-sdk-1.0.0-beta1_macos-x86_64_minimal.tar.xz` |

`RB_SDK_HOST` overrides the host; `RB_SDK_VERSION` overrides the version.

> **The macOS tarball is a few KB and that is correct.** `linux-x86_64` is about
> 70 MB and `macos-aarch64` about 6 KB, which looks like a broken asset and is
> not: *minimal* means the cmake scaffolding and `setup.sh`, and the installer's
> `setup.sh -t riscv64-zephyr-elf -l -h -c` downloads the toolchains afterwards.
> Do not "fix" this by selecting a different SDK version -- that would give macOS
> a toolchain which does not match the one Linux pins.
>
> `rb` locates whichever SDK version is installed rather than assuming the
> pinned one, so `RB_SDK_VERSION` works without any other change. Getting this wrong is worth
avoiding: the Linux tarball extracts perfectly well on macOS and then every tool
fails with *"bad CPU type in executable"*, which reads like a corrupt download
rather than the wrong package.

> **This step writes outside the checkout.** It registers the SDK in the
> *user-global* `~/.cmake/packages/Zephyr-sdk/`, one file per SDK, named for the
> md5 of that SDK's path. So a second checkout **adds** a second entry rather
> than replacing the first, and two live checkouts each keep their own.
>
> It does not grow without bound: CMake prunes entries whose target directory no
> longer exists, on the next `find_package` that reads the registry. You need not
> clean up after a deleted checkout.
>
> None of this affects `rb`, which passes `ZEPHYR_SDK_INSTALL_DIR` explicitly and
> never consults the registry -- verified by building with `HOME` pointed at an
> empty directory. It matters only if you run `west build` by hand outside `rb`,
> where CMake may then have several registered SDKs to choose between.

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

> **You can skip the environment variable.** With no `RB_DTS_DIR` set, `rb`
> looks in `<repo>/dts` -- so unpacking the collection there works with no
> configuration at all. Same for `<repo>/bitstreams` and `RB_BITSTREAM_DIR`.
> Both directories are gitignored. The environment variables exist for keeping
> the collections outside the checkout, and are the better choice if you have
> several checkouts sharing one collection.

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

> **Verified on Linux 2026-09-05, twice, from a clean clone of this branch into
> an empty directory**, each time by someone following only this page with no
> access to any other checkout. Steps 1 and 2 completed both times with no
> undocumented workarounds beyond the `PATH` and `conda deactivate` remedies
> prescribed above. Every figure quoted on this page -- the doctor lines, the
> `19424 B / 1 GB`, the `32 KB / 59.28%`, the ~16 GB footprint -- was reproduced
> exactly **on Linux**.
>
> **Verified on macOS (Apple Silicon) 2026-09-06**, once, the same way. That run
> found fourteen deviations from this page, all now folded in above; the one
> that stopped the build outright was `rb build` passing a hardcoded
> `-DDTC=/usr/bin/dtc`, which does not exist on a Mac and cannot be symlinked
> into place because `/usr/bin` is SIP-protected. `rb` now resolves `dtc` from
> the build environment's `PATH`. After that fix both memory figures reproduce
> exactly on macOS too. **The footprint figures do not** -- see the table in
> "Budget" above.
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

**Verify.** `rb doctor` has no line for openFPGALoader, so the only way to prove
`rb` found it is to run the real command. With no board attached it should reach
openFPGALoader and fail *there*:

```console
$ rb program --board arty200t --config RocketArty200TDroneFullDDRConfig
+ openFPGALoader --board te0712_8 --write-flash --bitstream .../arty200t.RocketArty200TDroneFullDDRConfig.bit
JTAG init failed with: unable to open ftdi device
```

That error is the pass condition here -- it means the binary was found and ran.
`rb: command not found`-style output, or a complaint about the bitstream path,
is not. (This needs step 3c's bitstream first; come back to it if you are
reading in order.)

### 3b. OpenOCD

On **Linux**, the Zephyr SDK from step 2 ships one and `rb` uses it
automatically; there is nothing to do here.

On **macOS this step is mandatory**:

```console
brew install open-ocd
```

The macOS SDK tarball is the `_minimal` one and carries no OpenOCD and no
`sysroots/` directory at all, even though its installer logs a `- Host tools`
line. After a clean, successful step 2c:

```console
$ ls .../zephyr-sdk-1.0.0-beta1/
cmake  gnu  llvm  sdk_gnu_toolchains  sdk_version  setup.sh
$ find .../zephyr-sdk-1.0.0-beta1 -iname "*openocd*"
$
```

So the `PATH` fallback is not a fallback on a Mac, it is the only path. `rb`
then reports `OpenOCD ok /opt/homebrew/bin/openocd`. `rb` looks inside the SDK
for a host-tools sysroot first and falls through to `PATH` when there is none,
so the same `rb` command works on both platforms -- but only after you have
installed OpenOCD yourself here.

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

### The hardware acceptance check

Everything above can be verified with no board. This is the sequence that closes
the remaining gap, in order, each step meaningful on its own. Run it the first
time a board is attached.

All six in one command, stopping at the first failure:

```console
tools/bench/acceptance.sh <bitstream-loc> <debug-loc>
```

It prints what each step proved, or what its failure means, and leaves per-step
logs under `.rb/acceptance/`. Its **control flow is verified** -- clean run, mid-sequence failure and a bad
adapter location were each exercised against a stubbed `rb` -- but it has **not
been run against hardware**, so treat the first real run as testing the script's
hardware path too. The steps individually, which is what
it runs:

```console
# 0. Which adapter is which. Locations change on every replug -- read, don't remember.
rb probe-ftdi --idcode
#    expect: one 0x13636093 (xc7a200t config TAP -> bitstream link)
#            one 0x00000001 (Rocket debug TAP -> ELF + console). All-ones before
#            the part is configured is correct, not a fault.

# 1. Does the shell configure at all? The single most diagnostic read on this board.
rb program --board arty200t --config RocketArty200TDroneFullDDRConfig \
    --usb-location <bitstream-loc>
#    If a MIG shell stalls, read the config STATUS register before suspecting
#    the cable: EOS=0 is an input-voltage problem. See
#    te0712-drone-carrier-ddr-bringup.md.

# 2. Does anything run? hello_world touches only the console, so it separates
#    "the board is alive" from "my workload is wrong".
rb debug hello_world --target arty200t --config RocketArty200TDroneFullDDRConfig \
    --program --program-usb-location <bitstream-loc> --usb-location <debug-loc>
#    expect on the console: "hello world from RiskyBird", then ticks 250 ms apart.

# 3. Is the DDR3 real? Only this proves the MIG, not just that it configured.
rb debug ddr_stress --target arty200t --config RocketArty200TDroneFullDDRConfig \
    --program --program-usb-location <bitstream-loc> --usb-location <debug-loc>

# 4. Are the sensors alive?
rb debug sensor_check --target arty200t --config RocketArty200TDroneFullDDRConfig \
    --program --program-usb-location <bitstream-loc> --usb-location <debug-loc>

# 5. Take a picture -- the end-to-end proof that a workload drives real hardware.
rb debug camera_photo --target arty200t --config RocketArty200TDroneFullDDRConfig \
    --program --program-usb-location <bitstream-loc> --usb-location <debug-loc> \
    --gdb-script hardware/ospi/capture-frame.gdb
#    writes camera-frame.png next to you. See hm01b0-camera-bringup.md.
```

**What is already checked, so a failure here means something specific.** The
memory contract between shell and workload is verified statically on every
build: this shell declares `ram0` at `0x8000_0000 + 0x4000_0000` (1 GiB DDR3),
`zephyr,sram` points at it, and the ELF's single LOAD segment sits at
`0x8000_0000` -- the same address the image that has run on this board loads at.
So if step 2 loads and then prints nothing, the linker and the memory map are
not the cause; look at the console wiring and the shell's UART instead.

The JTAG chain is likewise resolved end to end before you plug anything in: an
OpenOCD binary (the SDK's on Linux, Homebrew's on macOS -- see 3b),
`hardware/openocd/ft-link.cfg`, the board's target config
and `riscv64-zephyr-elf-gdb` all exist and are what `rb debug` invokes, and the
adapter config matches both parts this bench uses
(`ftdi vid_pid 0x0403 0x6010 0x0403 0x6011` -- FT2232H and FT4232H). A failure at
step 0 or 1 is therefore about the cable, the adapter's state or the board's
power, not about a missing or mis-pathed tool.

Steps 0-5 need **no battery and no propellers**. Anything that drives a motor
does: see `motor-pwm-bringup.md` and `flight-on-fpga.md`, and note that the full
drone shell drives the motor pins from configuration onward, before any code
runs.

> **Status of this section: recorded, not verified.** Tool discovery, bitstream
> resolution and adapter probing have been exercised on this checkout. The
> `rb program` and `rb console` commands are transcribed from a working bench
> and have **not** been re-run here, because no board was attached when this was
> written. Steps 1 and 2 have been run from a clean clone; step 3 has not.


### 3f. Loading and running a workload

Programming the FPGA puts a *shell* on the board. Getting your ELF into it is a
separate step, and **which mechanism works depends on the shell**:

| Shell | Load path | Command |
| --- | --- | --- |
| Scratchpad shells (`*Spad*`, `CameraBringup200T`) | UART-TSI | `rb run` — builds, programs, loads and opens the console in one go |
| **The drone shells** (`RocketArty200TDroneDDRConfig`, `RocketArty200TDroneFullDDRConfig`) | **JTAG** | `rb debug` |

The drone shells are elaborated with `uartTsi = false` and their devicetree
contains no TSI node at all, so `rb run` and `rb load` have nothing to talk to
on them. Use `rb debug`, which loads the ELF over the same JTAG link that
attaches the debugger:

```console
rb debug hello_world --target arty200t --config RocketArty200TDroneFullDDRConfig \
    --program --program-usb-location <bitstream-loc> --usb-location <debug-loc>
```

`--program` loads the bitstream first, so this one command covers programming
and loading. It loads and then **runs** the workload; add `--no-run` to stop at
a GDB prompt instead, and `--gdb-script <file>` to drive a scripted session and
exit.

Console output appears on channel B of the *debug* adapter, so one cable carries
both JTAG and the console.

### Checking the tooling itself

Optional, and it needs step 2 finished -- the suite resolves real backend paths:

```console
python3 -m unittest discover -s tests
```

**This suite is a silicon-tier check.** It resolves paths into
`backends/chipyard`, so on the bench tier a large minority of it fails for that
reason alone and nothing is wrong with your install:

| | result |
| --- | --- |
| silicon tier, Linux, both backends present | `Ran 245 tests ... OK` |
| bench tier, Linux, no Chipyard | `Ran 233 tests ... failures=18, errors=27` |
| bench tier, macOS, no Chipyard | `Ran 233 tests ... failures=7, errors=40` |

The failing modules are the same on both bench-tier platforms -- `test_xpurt`,
`test_area`, `test_pwm`, `test_rb`, `test_fingerprint` (and `test_soc` on
macOS) -- though how a given case lands on the failure/error line varies by
platform, so do not read anything into the split. Every one of them traces back
to Chipyard being absent: `fingerprint.py` raises `FingerprintError`, and
`test_pwm` reads the pin binder from a path under `backends/chipyard`.

If you are on the bench tier, this command tells you nothing about your install
and can be skipped. `verification.md` lists which modules pass where.

### Where to go next

The install is done; these are the pages that use it.

| Want to | Read |
| --- | --- |
| Make first contact with the drone bench | [`drone-initial-bringup.md`](drone-initial-bringup.md) -- adapters, console, and what a dead JTAG link actually looks like |
| Understand a board that will not configure | [`te0712-drone-carrier-ddr-bringup.md`](te0712-drone-carrier-ddr-bringup.md) -- **read this before concluding a board is broken** |
| Take a picture | [`hm01b0-camera-bringup.md`](hm01b0-camera-bringup.md) |
| Drive motors | [`motor-pwm-bringup.md`](motor-pwm-bringup.md) -- propellers off, battery connected |
| Run the flight controller | [`flight-on-fpga.md`](flight-on-fpga.md) |

Two things that will otherwise cost you a bench session:

- **A MIG shell that stalls at `EOS=0` is usually an input-voltage problem, not
  a broken board or a broken cable.** The DDR3 rails need headroom above 3.3 V.
  The DDR page has the measurements.
- **Motors are powered from the battery, not from USB-C.** A motor test on USB
  alone does nothing useful, and the full drone shell drives the motor pins from
  the moment it is configured -- before any code runs.

Start with `hello_world`. It touches nothing but the console, so it separates
"the board is alive" from "my workload is wrong".

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
| `backends/chipyard` after 4b, before any build | **~11 GB** (of which ~6 GB is its conda environment) |
| each config you **elaborate** | ~45 MB under `fpga/generated-src/` |
| each config you **build** | ~450-650 MB under `fpga/generated-src/`, depending on the design |

So budget ~11 GB for the silicon tier on top of step 2's ~16 GB, and expect
`generated-src/` to grow with every config you touch -- it is a per-config
directory, not a shared one, and nothing prunes it. A long-lived checkout that
has built dozens of configs can carry tens of gigabytes there alone.

### 4b. Chipyard's environment

```console
rb env setup chipyard
```

Runs Chipyard's own `build-setup.sh --use-lean-conda`, building a locked conda
environment at `backends/chipyard/.conda-env`. This is also what supplies
**Espresso**, the one `rb doctor` line that no earlier step turns green -- so
after 4b `rb doctor` can reach a fully-`ok` scorecard and exit 0. Check it:

```console
$ rb env status
chipyard  ok      .../backends/chipyard/.conda-env
zephyr    ok      .../tools/miniforge3/envs/zephyr
rose      MISSING .../.rb/envs/rose
```

`rose` is the experimental co-simulation environment and stays `MISSING` unless
you want it. Wait for 4b to exit before trusting this: it is a probe of the
prefix, not of whether the install finished.

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

> That `ok` means the binary exists and runs -- **not** that it is licensed.
> `rb doctor` cannot tell the difference, so an unlicensed install passes here
> and then fails deep inside 4d. If your site uses a floating licence, make sure
> `XILINXD_LICENSE_FILE` is set before starting.

### 4d. Elaborate, then build

Elaboration is minutes and catches configuration errors without spending an hour
in Vivado, so do it first on a config you have not built before:

```console
rb chipyard elaborate --board arty200t --config RocketArty200TDroneFullDDRConfig
```

It writes the `.dts` that step 2d distributes.

> A successful elaboration prints
> `make: [common.mk:283 ...top.mems.conf] Error 1 (ignored)`. That line is
> expected -- the `(ignored)` is the point -- and does not mean the run failed.

Then the bitstream:

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

> **Verified from cold clones by readers other than its author**, on four
> separate runs. The most recent reached this step with the environment, SDK,
> devicetree and a built ELF all in place; the two before it finished bitstreams
> -- one `RocketArty200TSpadConfig`, one the MIG drone shell.
>
> 2026-09-06, from a cold clone of this branch by a reader
> following only this page: `RocketArty200TDroneFullDDRConfig` -- the MIG drone
> config -- elaborated and built, reporting *"All user specified timing
> constraints are met."* and 9,730,764 bytes. The same reader also elaborated
> `NoCoresArty200TConfig` and saw the contract message described above.
>
> And earlier the same day by building `RocketArty200TDroneFullDDRDmaConfig` --
> a config that had never been synthesised, so nothing could no-op -- exactly as
> written above. `write_bitstream completed successfully`, timing met
> (`WNS=0.153  TNS=0.000  WHS=0.055  THS=0.000`), 9,730,764 bytes, which is the
> fixed bitstream size for this part. Elapsed roughly an hour, dominated by
> routing.

> **A config that "fails" the contract check has still built.** `NoCores*Config`
> elaborates cleanly and then reports *"elaborated successfully, but the
> generated DTS is not a deployable RiskyBird target"* -- it has no cores and no
> UART by design. That is a correct rejection of a config that was never a
> deployment target, not a broken build. The same config exists for all three
> boards and behaves the same way.

### 4e. Publish for hosts that cannot build

This is what makes the bench tier work on a Mac.

```console
python3 tools/collect-dts.py         --out ~/riskybird-dts-out
python3 tools/collect-bitstreams.py  --out ~/riskybird-bitstreams-out
```

> **Do not point `--out` at the checkouts you cloned in steps 2d and 3c.** The
> collectors copy files in without deleting, but they **overwrite
> `manifest.json` and `README.md`** with only what *this* machine has
> elaborated or built. Aim them somewhere else and copy what you want across,
> or the collection ends up with fifty devicetrees beside a manifest claiming
> one.
>
> Omitting `--out` writes to `<repo>/dts` and `<repo>/bitstreams`, which is
> often what you want -- see below.

Each writes `<board>/<board>.<Config>.<ext>` plus a `manifest.json` recording
the source revisions -- and for bitstreams, per-config LUT/FF/BRAM/DSP counts,
`wns_ns`, `whs_ns` and a `timing_met` flag, with any config that missed timing
called out in the README too.

> The two manifests are **not** the same schema: the devicetree one keys its
> list as `entries`, the bitstream one as `bitstreams`. Anything parsing both
> has to handle each separately.

The renaming is the point. Chipyard names every bitstream after its *harness*,
so a directory of `Arty200THarness.bit` files is indistinguishable once moved;
naming by board and config is what lets `RB_BITSTREAM_DIR` and `RB_DTS_DIR`
resolve a specific shell on a machine that never built it.

Commit and push those two directories to their side repositories, and a Mac
picks them up through steps 2d and 3c.

**Verified**: cloning both published side repositories and pointing `RB_DTS_DIR`
and `RB_BITSTREAM_DIR` at them, with no Chipyard installed, `rb build` produced
a correct ELF for `RocketArty200TDroneFullDDRConfig` -- the 1 GB memory region
proves it read the published devicetree -- and `rb` resolved the published
bitstream for `rb program`. That is the whole cross-machine path, exercised on
Linux; only macOS itself is untested.

Only *synthesised* configs produce bitstreams. A config that was elaborated but
never built appears in the DTS collection and not the bitstream one -- which is
precisely the situation step 3c warns about for `arty200t`'s default config.
