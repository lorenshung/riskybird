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

## Step 2 -- (not written yet)

The Zephyr backend and SDK, ending in a built `hello_world` ELF.
