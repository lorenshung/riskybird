# RiskyBird setup

Built up one piece at a time. Each step adds exactly one dependency and ends in a command you
can run to prove that step worked -- a step is not done because it installed, it is done because
its verification command printed what this page says it prints.

Two tiers, and you almost certainly want the first:

| Tier | What it does | Needs | macOS |
| --- | --- | --- | --- |
| **Bench** | build workloads, program the FPGA, debug, run the drone | Python, Zephyr SDK, openFPGALoader, OpenOCD | **yes** |
| **Silicon** | elaborate Chipyard, build bitstreams, FireSim | Vivado, sbt/Verilator | **no -- Linux only** |

Vivado has no macOS release, so a Mac cannot build bitstreams. It can do everything else,
consuming bitstreams built elsewhere. Nothing in the bench tier needs Chipyard installed at all;
an uninitialised submodule is simply an empty directory.

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

They work here and are used in this repository's own experiments, but they are in progress: they
carry dependencies this guide does not install (`gymnasium`, `isaaclab`), their setup is not
documented, and their interfaces are expected to change. Nothing in the supported surface depends
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
git clone git@github.com:lorenshung/riskybird-dev.git
cd riskybird-dev
```

**Do not use `--recursive`.** The backends carry their own nested submodules -- recursing pulls
several gigabytes, including three separate copies of Chipyard, none of which the bench tier
uses. Later steps initialise exactly the submodules they need.

### Verify

Three commands, no hardware and no toolchain required.

```console
$ ./rb --help
usage: rb [-h] [--version]
          {doctor,targets,workloads,env,chipyard,zephyr,build,flash,probe-ftdi,...}

One entry point for RiskyBird FPGA and Zephyr flows.
```

```console
$ ./rb targets
TARGET       KIND  ZEPHYR BOARD                                  EVIDENCE
arty100t     fpga  chipyard_riscv64/rocketchip_virt_riscv64      generated and synthesized
arty200t     fpga  chipyard_riscv64/rocketchip_virt_riscv64      generated
ku040        fpga  chipyard_riscv64/rocketchip_virt_riscv64      generated and synthesized
```

```console
$ ./rb workloads
WORKLOAD          MODES          TARGETS                  STATUS/SAFETY
hello_world       default        arty100t,arty200t,ku040,esp32c6,firesim normal
  Console bring-up probe: banner, timed tick loop, and a completion marker a debugger can break on
...
```

### What `rb doctor` should say right now

`rb doctor` is the running scorecard for the whole install. After step 1 it is **supposed** to be
almost entirely `MISSING` -- that is the correct output, not a failure:

```console
$ ./rb doctor
RiskyBird rb 0.1.0
repository       /path/to/riskybird-dev
Chipyard backend  MISSING .../backends/chipyard
Zephyr backend    MISSING .../backends/zephyr-chipyard-sw
Zephyr SDK        MISSING .../backends/zephyr-chipyard-sw/tools-manual/zephyr-sdk-1.0.0-beta1
west              MISSING
RISC-V compiler   MISSING .../riscv64-zephyr-elf-gcc
OpenOCD           MISSING .../hosttools/sysroots/x86_64-pokysdk-linux/usr/bin/openocd
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

**`MISSING` lines name the path that was expected**, so they double as the answer to "where is it
supposed to go". The SDK line naming `x86_64-pokysdk-linux` on a Mac is not a bug -- with no SDK
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
(`7-1.1.4.1`), so the value it prints is the one to pass to `--usb-location`. It does **not**
report which serial node belongs to which channel -- that is a Linux `ftdi_sio` property. On
macOS match them by the trailing letter of the device name: `/dev/cu.usbserial-<serial>A` is
channel A (JTAG), `...B` is channel B (console).

You can skip this until you have hardware attached.

---

## Step 2 -- the Zephyr backend, ending in a built ELF

Adds the toolchain. After this step `rb build` produces a real ELF, with no FPGA
attached and -- on the bench tier -- no Chipyard installed.

### 2a. Initialise only the backend you need

```console
git submodule update --init backends/zephyr-chipyard-sw
```

Again, **not `--recursive`**. The backend initialises its own nested submodules
during its setup; recursing here pulls a large set it deliberately skips.

The bench tier does not initialise `backends/chipyard` at all. An uninitialised
submodule is just an empty directory -- nothing is deleted and nothing is
special-cased.

### 2b. Install the toolchain

Follow the backend's own README, which owns this process. The SDK installer
picks the right prebuilt tarball for your host automatically:

```console
cd backends/zephyr-chipyard-sw
./scripts/install_toolchain_sdk.sh
```

| host | tarball |
| --- | --- |
| Linux x86_64 | `zephyr-sdk-1.0.0-beta1_linux-x86_64_minimal.tar.xz` |
| **macOS Apple Silicon** | `zephyr-sdk-1.0.0-beta1_macos-aarch64_minimal.tar.xz` |
| macOS Intel | `zephyr-sdk-1.0.0-beta1_macos-x86_64_minimal.tar.xz` |

Set `RB_SDK_HOST` to override for a host the installer does not know. Getting
this wrong is worth avoiding: the Linux tarball extracts perfectly well on
macOS and then every tool fails with *"bad CPU type in executable"*, which reads
like a corrupt download rather than the wrong package.

`rb` finds the SDK's host tools by looking inside the SDK, so no path needs
editing after this.

### 2c. Supply an elaborated devicetree

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

The layout matches the bitstream and ELF side-repositories:
`<board>/<board>.<Config>.dts`. `rb` prefers a local Chipyard elaboration when
one exists, so on a machine with Chipyard this is ignored entirely.

Regenerate it from a machine that has Chipyard with `tools/collect-dts.py`.

### Verify

`rb doctor` should now report the Zephyr lines `ok` and the hardware contracts
resolved. The contract lines are the ones that prove step 2c worked -- they are
read out of the DTS:

```console
$ rb doctor
Zephyr backend    ok      .../backends/zephyr-chipyard-sw
Zephyr SDK        ok      .../tools-manual/zephyr-sdk-1.0.0-beta1
RISC-V compiler   ok      .../riscv64-zephyr-elf-gcc
hardware contracts
  arty200t     ok      config RocketArty200TOspiConfig, RAM 0x80000000+0x40000000, UART 0x10020000 irq 2, I2C 0x10040000 irq 1
```

Chipyard and Vivado stay `MISSING` on the bench tier. That is correct.

Then build something:

```console
$ rb build hello_world --target arty200t --config RocketArty200TDroneFullDDRConfig
...
Memory region         Used Size  Region Size  %age Used
             RAM:       19424 B         1 GB      0.00%
```

An ELF at `build/zephyr/arty200t/hello_world/default/zephyr/zephyr.elf` is the
end of step 2. Nothing so far has needed hardware.

That `1 GB` region is itself the proof that step 2c worked: it comes from the
`RocketArty200TDroneFullDDRConfig` devicetree, so the build read the collected
DTS. A scratchpad config would report 32 KB here.

**Verified** on 2026-09-05 exactly as written above, with `backends/chipyard`
absent and the devicetree supplied through `RB_DTS_DIR` -- `rb doctor` reported
every Zephyr line `ok`, Chipyard and its conda environment `MISSING`, and the
build produced a 409 KB ELF.

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

Same split as the devicetree in step 2c, and for the same reason: **programming
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

> **Not yet verified end to end.** Steps 1 and 2 were run exactly as written.
> Step 3's tool discovery, bitstream resolution and adapter probing are
> verified; the programming and console commands are recorded from a working
> bench but have not been re-run against this checkout, because the board was
> detached while it was written. Treat this section as recorded, not verified,
> until it is.
