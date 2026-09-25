# Drone initial bring-up

Getting from "two USB cables plugged into a dev board" to "a Zephyr program
running on Rocket and printing", on the TE0712 drone carrier.

This is the first-contact document for the drone bench. It assumes nothing is
known to work and establishes each link in an order where a failure at step *n*
cannot be blamed on step *n+1*: tell the two FTDIs apart, reach the debug TAP,
find the memory, load an image, verify every byte of it, run it, read what it
printed.

Both I2C and SPI work: six devices answer on the bus and the optical-flow sensor
streams live motion.

The console UART did not come up during this bring-up, and a long investigation
concluded the pair was never landed. **That conclusion is superseded: the console
works.** Confirmed 2026-09-03, reading Zephyr's output off channel B of the debug
FT2232H at 115200. The investigation is kept in full under
[Resolved: the console UART](#resolved-the-console-uart), because the evidence
and the two retracted readings inside it are worth more than the conclusion was.
Nothing that depended on the workaround is invalidated -- results can still be
read straight out of memory over JTAG, and that path is what proved the console
was silent rather than misconfigured.

## The bench

```
host ── USB-C ──┐
                ├── dev board ── FT2232H #1  "bitstream"  → FPGA configuration JTAG
host ── USB-C ──┘               └ FT2232H #2  "zephyr"     → RISC-V debug JTAG + UART pins
                                        │
                                   ribbon cable
                                        │
                                   drone board
```

The FPGA is a TE0712 SOM carrying an `xc7a200t-fbg484`. Both FT2232Hs are
dual-channel: channel A drives JTAG in MPSSE mode, channel B is a plain UART.

## Telling the two adapters apart

**This is the step that blocks everything else, and it is not obvious.**

Both dev-board adapters ship with a blank EEPROM, so neither carries a serial
number and neither can be addressed by one. When the two are the same part --
which is how this bench started out, two FT2232Hs -- every USB descriptor field
is identical as well: manufacturer `FTDI`, product `Dual RS232-HS`, empty
serial. `lsusb -v` for the two devices then differs only in the device-number
line, and there is no descriptor, EEPROM field, or udev property that separates
them.

**Do not lean on the descriptors even when they do differ.** The adapters get
swapped, and the parts are not fixed:

| | Session start | After a swap |
| --- | --- | --- |
| bitstream | FT4232H `0403:6011` "Quad RS232-HS" | unchanged |
| zephyr | FT4232H `0403:6011` "Quad RS232-HS" | FT2232H `0403:6010` "Dual RS232-HS" |

Both parts drive JTAG identically -- channel A is the MPSSE channel on each --
so which one is in the port carries no information about which link it is.
Anything that matches on the product string, or on one PID, breaks the moment a
part changes. `ft-link.cfg` therefore lists both PIDs and filters on no
descriptor at all:

```tcl
ftdi vid_pid 0x0403 0x6010 0x0403 0x6011
```

A single-PID match is worse than no filter, because `rb debug`'s ambiguity
guard counts the same set: an adapter excluded by PID is invisible to the
guard, so instead of an error you get a silent wrong-adapter pick and the
all-ones scan below.

Matching on VID:PID alone hits four devices on this host, so OpenOCD takes
whichever libusb enumerates first. Losing that coin flip produces:

```console
Error: JTAG scan chain interrogation failed: all ones
Error: riscv.cpu: IR capture error; saw 0x1f not 0x01
Error: Unsupported DTM version: 15
```

which reads exactly like an open circuit and is not one. All three lines are one
symptom -- TDO stuck high -- and `15` is `0xf`, all ones in the four-bit version
field of `dtmcs`.

USB topology is the only discriminator. `rb probe-ftdi` reports it:

```console
$ rb probe-ftdi
LOCATION   BUS:DEV  VID:PID    SERIAL        INTERFACES
5-2        5:2      0403:6011  21320733400T  if0=detached if1=ttyUSB1 if2=ttyUSB2 if3=ttyUSB3
           Xilinx / A-U250-A64G
7-1.1.4.1  7:18     0403:6010  (none)        if0=ttyUSB0 if1=ttyUSB4
           FTDI / Dual RS232-HS
7-1.1.4.3  7:16     0403:6011  (none)        if0=detached if1=ttyUSB6 if2=ttyUSB10 if3=ttyUSB11
           FTDI / Quad RS232-HS
7-1.3      7:8      0403:6010  TE0000991948  if0=detached if1=ttyUSB7
           Xilinx / Trenz USB Device
```

`--idcode` then shifts one IDCODE out of each and says which chain it reached,
which is the only way to learn which physical adapter is which:

```console
$ rb probe-ftdi --idcode
5-2        0x04b57093  xcu250 config TAP -- Alveo card, not the drone bench
7-1.1.4.1  0x00000001  Rocket debug TAP -- Zephyr/GDB link
7-1.1.4.3  0x13636093  xc7a200t config TAP -- bitstream link
7-1.3      0x13822093  xcku040 config TAP -- bitstream link
```

So, on this bench, as of the session this document records:

| Location | Link | Reaches |
| --- | --- | --- |
| **`7-1.1.4.1`** | **zephyr** | Rocket debug TAP -- `rb debug`, GDB, load and run |
| **`7-1.1.4.3`** | **bitstream** | xc7a200t config TAP -- FPGA configuration only |

**These values are stale, and that is the point of the section.** On 2026-09-03
the adapters were one hub deep instead of three and the roles had moved with
them: `7-1.1` was the zephyr link and `5-5.3` the bitstream link. Every command
in this document quotes the older pair. Re-derive with `rb probe-ftdi --idcode`
rather than copying either set -- the location is a bus-port path, and the roles
do not follow the part.

The bitstream adapter cannot reach the debug TAP and the zephyr adapter cannot
configure the FPGA, so the order is fixed: configure the FPGA over the bitstream
link first, then load Zephyr over the zephyr link. A `RocketArty200T*` bitstream
has to be in the part before the debug TAP answers at all.

**The cheapest confirmation is the unplug test**, and it needs no JTAG: pull one
adapter, re-run `rb probe-ftdi`, and see which location disappears. Done here,
removing the zephyr adapter left `7-1.1.4.3` alone -- and `7-1.1.4.3` is exactly
the location that had shifted out `0x13636093`. Two independent methods, same
answer. Worth doing whenever a cable has been touched, because it costs one
command and the IDCODE probe costs the `ttyUSB` nodes (below).

Only the two dev-board adapters answer repeatably. `5-2` and `7-1.3` return an
IDCODE on a first probe and `no IDCODE returned` on every probe after, until
they are replugged -- the same ftdi_sio-detach residue described below. Neither
is part of this bench, so this is recorded rather than chased.

**`0x00000001` is not a failure.** It is Rocket's debug transport module:
`JtagDTMKeyDefault` sets `idcodeVersion`, `idcodePartNum` and `idcodeManufId`
all to zero, so the IDCODE is `0<<28 | 0<<12 | 0<<1 | 1`
(`rocket-chip/src/main/scala/devices/debug/DebugTransport.scala:26`). Reading it
as a fault and going hunting for a broken wire is the trap this section exists
to prevent.

`0x13822093` is the KU040 on the same host, not part of this bench.

Once known, the location is passed to every command that opens the adapter:

```bash
rb debug ... --usb-location 7-1.1.4.1     # or export RB_FTDI_LOCATION
```

`ft-link.cfg` reads `RB_FTDI_LOCATION` and applies `adapter usb location`. It
also now sets `ftdi device_desc "Dual RS232-HS"`, which at least excludes the
Trenz module. `rb debug` refuses to guess: with more than one FT2232H attached
and no location given, it errors instead of producing a misleading all-ones
scan.

### Two things about USB locations

- The location is a **bus-port path**, not a `bus:device` pair. It survives
  replugging into the same physical port and changes if the adapter moves to a
  different port or hub. Re-run `rb probe-ftdi` after moving a cable.
- `ttyUSB` numbers are **not stable**. Across this session `7-1.1.4.1:1.0`
  was `ttyUSB4`, then `ttyUSB0`; `7-1.1.4.3:1.0` was `ttyUSB10`, then `ttyUSB4`.
  Address serial ports through `/dev/serial/by-path/` instead:

  ```
  /dev/serial/by-path/pci-0000:29:00.3-usb-0:1.1.4.1:1.1-port0   # zephyr FTDI, channel B
  ```

### Why ttyUSB nodes disappear

OpenOCD drives JTAG by detaching `ftdi_sio` from channel A, and the kernel does
not rebind it on exit. That channel's `ttyUSB` node vanishes until the adapter
is replugged. `rb probe-ftdi` reports it per interface as `if0=detached`, and
the console on channel B is unaffected. To rebind without replugging:

```bash
echo 7-1.1.4.1:1.0 | sudo tee /sys/bus/usb/drivers/ftdi_sio/bind
```

Killing OpenOCD mid-transaction is worse than a lost tty: it leaves the MPSSE
engine dirty, and the next run fails with `dmstatus=0x0` /
`Debug Module version 0` plus an `mpsse_flush` assertion. That is not a hardware
fault. Let OpenOCD shut down cleanly, or replug.

## The configuration

`RocketArty200TSpadConfig`, which is what these bitstreams were built with and
what the drone carrier's pinout matches.

| | |
| --- | --- |
| Config | `RocketArty200TSpadConfig` |
| Core | one huge Rocket, `misa=0x800000000094112d` -- RV64 IMAFDC, S and U modes, FPU and MMU |
| L2 | none (`WithBroadcastManager`) |
| Main memory | **32 KiB mbus scratchpad at `0x8000_0000`**, no MIG, no DDR3 |
| Periphery | sifive UART, JTAG, UART-TSI. **No I2C** |
| Clock | 50 MHz on every bus |
| Part | `xc7a200tfbg484-2` |

The memory bound is worth confirming on a new board, because it identifies the
bitstream without reading back the bitstream. Writing and reading walks cleanly
up to `0x80007ffc` and fails at `0x80008000`:

```console
(gdb) monitor mww 0x80007ff0 0xa5a57ff0
(gdb) monitor mdw 0x80007ff0
0x80007ff0: a5a57ff0
(gdb) monitor mww 0x80008000 0xa5a58000
Error: Target riscv.cpu0: Failed to write memory (addr=0x80008000)
  progbuf=failed, sysbus=failed, abstract=failed
```

Exactly 32 KiB. A DDR config would have kept going.

## Pins

From the generated `*.shell.xdc` of the loaded bitstream, so this is what is in
the part rather than what the binders intend. All `LVCMOS33`.

| Port | Pin | J1 | Bank | Direction | Extra |
| --- | --- | --- | --- | --- | --- |
| `jtag_TCK` | A19 | J1_46 | 16 | in | `PULLUP`, `CLOCK_DEDICATED_ROUTE FALSE` |
| `jtag_TMS` | F19 | J1_52 | 16 | in | `PULLUP` |
| `jtag_TDI` | A18 | J1_48 | 16 | in | `PULLUP` |
| `jtag_TDO` | F20 | J1_50 | 16 | out | `PULLUP` |
| `uart_rxd` | E21 | J1_41 | 16 | in | `IOB TRUE`, **no pullup** |
| `uart_txd` | D21 | J1_43 | 16 | out | `IOB TRUE` |
| `uart_tsi_rxd` | P16 | J1_92 | 14 | in | `IOB TRUE` |
| `uart_tsi_txd` | U18 | J1_85 | 14 | out | `IOB TRUE` |

J1 numbers are from Trenz's own board file,
`arty_files/te0712-rd/board_files/TE0712_200_1I/2.0/part0_pins.xml`.

`uart_rxd` is the FPGA **input** -- the FTDI's TX drives J1_41 -- and `uart_txd`
is the FPGA **output**, read by the FTDI's RX on J1_43.

Plus the on-module 50 MHz LVDS clock pair on H4/G4.

The four JTAG pins are bank 16, on the TE0712's **J1** header, and they are
confirmed working -- the debug TAP answers on them. Note this is a *different*
pinout from `docs/te0712-ftdi-carrier-ddr-bringup.md`, which puts JTAG and UART
on **J2** in bank 14 for a different carrier. Do not mix the two: loading
`RocketArty200TFtdiDDRConfig` on this bench moves JTAG to pins nothing is
connected to, and the all-ones scan that follows is genuine.

There are **two** UART ports, on physically separate pin pairs: the console
(`uart_rxd`/`uart_txd`, E21/D21) which is the DTS device `serial@10020000`, and
the TSI program loader (`uart_tsi_rxd`/`uart_tsi_txd`, P16/U18) which is
harness-only and appears in no devicetree.

## Building

`hello_world` is the smallest workload that proves the console path, and it
touches no peripheral except the console -- which is what makes it correct for a
shell with no I2C, SPI or PWM at all.

```bash
rb build hello_world --target arty200t --config RocketArty200TSpadConfig
```

```console
Memory region         Used Size  Region Size  %age Used
             RAM:       19424 B        32 KB     59.28%
```

### The UART interrupt moves when there is no I2C

`RocketArty200TSpadConfig` has no I2C controller, and that shifts the UART's
PLIC source from 2 down to 1. `hardware/zephyr/fpga-common.overlay` describes
the OSPI shells and hardcodes source 2, so a no-I2C shell built against it alone
gets a console interrupt wired to a source nothing drives.

`hardware/zephyr/fpga-no-i2c.overlay` corrects this and `rb` applies it
automatically. The selection reads the generated DTS rather than pattern-matching
the config name, because the name is not sufficient -- `CnnNavKU040Config`
carries I2C without `Ospi` anywhere in it. Across all 39 elaborated configs in
`backends/chipyard/fpga/generated-src` the two facts agree exactly: I2C present
means UART on source 2, I2C absent means UART on source 1, DroneLogic means 10.

Confirm it landed:

```console
$ grep -A8 'uart0:' build/zephyr/arty200t/hello_world/default/zephyr/zephyr.dts
	interrupts = < 0x1 0x1 >;     /* in ../../../hardware/zephyr/fpga-no-i2c.overlay:17 */
```

## Loading and running over GDB

```bash
rb debug hello_world --target arty200t \
    --config RocketArty200TSpadConfig \
    --usb-location 7-1.1.4.1
```

That starts OpenOCD against the pinned adapter, waits for the GDB port, and
attaches `riscv64-zephyr-elf-gdb` with the ELF loaded. OpenOCD reports:

```console
Info : JTAG tap: riscv.cpu tap/device found: 0x00000001 (mfg: 0x000 (<invalid>), part: 0x0000, ver: 0x0)
Info : datacount=8 progbufsize=16
Info : Examined RISC-V core; found 1 harts
Info :  hart 0: XLEN=64, misa=0x800000000094112d
RiskyBird Arty200T is ready for GDB
```

Inside GDB, load and then **verify** -- `compare-sections` reads the whole image
back over JTAG and checks it against the host copy, which is a bulk memory test
with a known-good reference:

```gdb
(gdb) monitor halt
(gdb) load
(gdb) compare-sections
(gdb) monitor reg pc 0x80000000
(gdb) monitor resume
```

```console
Start address 0x0000000080000000, load size 11354
Transfer rate: 51 KB/sec, 946 bytes/write.
Section rom_start, range 0x80000000 -- 0x80000018: matched.
Section reset, range 0x80000018 -- 0x8000001e: matched.
Section exceptions, range 0x80000020 -- 0x80000232: matched.
Section text, range 0x80000234 -- 0x80002186: matched.
...
Section device_states, range 0x80002c60 -- 0x80002c64: matched.
```

All twelve sections matched. Use `monitor reg pc 0x80000000` rather than
`set $pc = ...`: `monitor resume` bypasses GDB's resume path, so a `$pc` written
through GDB is not necessarily flushed to the target first, and the core resumes
from wherever it already was.

Re-attaching later and landing in `arch_cpu_idle()` is the cheapest proof the
program ran: `main()` reached its `k_msleep` loop and the kernel went idle.

## Reading the output

Read it on the console UART, channel B of the zephyr FT2232H, at 115200:

```bash
rb console --port /dev/ttyUSB6 --baudrate 115200
```

Pick the node that is interface 1 of the zephyr adapter's USB location; the
numbers move across replugs, so re-derive rather than reusing one from here.

```bash
rb build hello_world --target arty200t --config RocketArty200TSpadConfig
rb debug hello_world --target arty200t --config RocketArty200TSpadConfig \
    --usb-location 7-1.1.4.1
```

```gdb
(gdb) monitor halt
(gdb) load
(gdb) monitor reg pc 0x80000000
(gdb) monitor resume
```

```console
hello world from RiskyBird
board   chipyard_riscv64
arch    riscv, 64-bit
tick 0  uptime 0 ms
tick 1  uptime 251 ms
tick 2  uptime 502 ms
tick 3  uptime 753 ms
tick 4  uptime 1004 ms
...
tick 15  uptime 3765 ms
```

That is the whole bring-up in one output. The image loaded and entered, the C
runtime formats, the kernel schedules, and the CLINT timebase is right -- the
deltas are 251 ms against a `k_msleep(250)`.

## I2C: the drone board answers

The console is not the only way to get an answer off the board, and I2C turned
out to be the better one. A bus scan reports which addresses ACK, and that
single list proves the I2C controller, the carrier wiring, the ribbon cable and
the drone's sensors all at once -- with no serial path involved.

**Configuration.** `RocketArty200TOspiSpadConfig`: the same huge Rocket and the
same 32 KiB scratchpad as `RocketArty200TSpadConfig`, plus the OSPI periphery
that brings in the I2C controller. Main memory stays on the path already proven,
so a failure here is about I2C and nothing else. The JTAG pins are unchanged, so
the debugger still works.

| Signal | Pin | J1 | Bank |
| --- | --- | --- | --- |
| `i2c_scl` | A15 | J1_58 | 16 |
| `i2c_sda` | A16 | J1_56 | 16 |

Both are in bank 16, the bank the working JTAG pins already prove is live.

```bash
rb chipyard build --board arty200t --config RocketArty200TOspiSpadConfig
rb build i2c_scanner --target arty200t --config RocketArty200TOspiSpadConfig
rb debug i2c_scanner --target arty200t --config RocketArty200TOspiSpadConfig \
    --usb-location 7-1.1.4.1
```

```gdb
(gdb) break i2c_scan_done
(gdb) continue
(gdb) p i2c_scan_count
(gdb) p/x i2c_scan_found[0]@16
```

### Result

```console
$1 = 117                                    <- addresses probed
$2 = 6                                      <- devices found
$3 = -5                                     <- last error, -EIO: a clean NAK
$4 = {0x17, 0x18, 0x29, 0x2b, 0x68, 0x76, 0x0 ...}
```

```console
Starting I2C scan on i2c@10040000...
Found device at 0x17
Found device at 0x18
Found device at 0x29
Found device at 0x2B
Found device at 0x68
Found device at 0x76
I2C scan complete: 6 device(s) found in 117 probes
```

Byte-identical across two runs.

Four of the six match devices this repository already describes:

| Address | Device | Where it is declared |
| --- | --- | --- |
| 0x17 | ADS7128 ADC / GPIO expander | `hardware/zephyr/targets/esp32c6/workloads/ads7128_test.overlay:16` |
| 0x18 | BMI088 accelerometer | `hardware/zephyr/fpga-common.overlay:72` |
| 0x29 | VL53L1X time-of-flight (down), power-on address; the flight controller moves it to 0x30 | `hardware/zephyr/targets/fpga/workloads/flight_controller.overlay:51` |
| 0x68 | BMI088 gyroscope | `hardware/zephyr/fpga-common.overlay:79` |
| 0x2B | **unidentified** | not referenced anywhere in this repository |
| 0x76 | **unidentified** | not referenced anywhere in this repository |

0x76 is the conventional BMP280/BME280 barometer address and 0x2B is plausibly a
second ADS7128 strap, but neither is asserted here: nothing in this repository
declares them, and the scan proves only that something ACKs.

`i2c_scan_last_error` is `-EIO` rather than a bus fault, which is what a healthy
bus returns for the 111 addresses with nothing at them. A floating or
un-pulled-up bus fails every address identically and finds nothing; this one
discriminates.

### What this establishes

This is the first evidence that anything beyond the FPGA itself works:

- The sifive I2C controller at `0x10040000` is functional.
- A15/A16 reach the carrier, so the J1 signal block is wired for more than JTAG.
- **The ribbon cable to the drone board carries signal**, which nothing before
  this had shown.
- The drone's sensors are powered and responding.

It also settles the method question the UART left open: results can be read off
this board with no serial path at all, straight out of `i2c_scan_found` in
memory over JTAG. That agreed with the console output.

## IMU: the BMI088 reads gravity

The IMU works, and needed no new code -- the drone overlay already declares both
halves and the aliases the workload expects.

```bash
rb build bmi088_test --target arty200t --config RocketArty200TFlowSpadConfig
rb debug bmi088_test --target arty200t --config RocketArty200TFlowSpadConfig \
    --usb-location 7-1.1.4.1
```

```console
Accelerometer - X: -0.170525 m/s^2, Y: 0.218990 m/s^2, Z: -9.694795 m/s^2
Gyroscope     - X: 0.002660 rad/s, Y: 0.004256 rad/s, Z: -0.003192 rad/s
Temperature    - 23.009750 C
```

**The check that matters is the magnitude, not the presence of numbers.** A
misconfigured sensor, a wrong scale factor or a stuck bus all produce plausible
readings; only the vector length says the data is physically real:

| Sample | \|a\| | vs 9.80665 |
| --- | --- | --- |
| 1 | 9.7211 | -0.87% |
| 2 | 9.6784 | -1.31% |
| 3 | 9.6828 | -1.26% |

That is gravity, on an uncalibrated consumer part sitting still. The two
horizontal axes read a few hundredths of g, consistent with a board resting not
quite level; the gyro sits at single-digit millirad/s, which is rest plus noise;
and 23.0 C is a believable room temperature rather than a default.

The BMI088 is two I2C devices, not one -- accelerometer at 0x18 and gyroscope at
0x68, both of which answered the bus scan. `bosch,bmi08x-accel` and
`bosch,bmi08x-gyro` in `hardware/zephyr/fpga-common.overlay` and
`fpga-drone.overlay` already describe them at 200 Hz, +/-6 g and +/-1000 dps.

## ToF array: three sensors intended, two answer

**The drone is meant to carry three time-of-flight sensors** (confirmed by the
board owner, 2026-09-10): two side VL53L5CX and one down-facing VL53L1X, each
held in reset through its own ADS7128 XSHUT channel. On the current board only
two answer -- one side sensor and the down sensor -- measured 2026-09-10 with
`bootup_check`:

```console
ch1 side  VL53L5CX   at 0x31 (id 0xf002)
ch2 side  -- absent (err -116)
ch3 side  -- absent (err -116)
ch6 down  VL53L1X    at 0x34 (id 0xeacc)
no unclaimed channel answers at 0x29
[ WARN ] ToF array (XSHUT)    2 up, 2 absent -- down sensor present
```

The missing side sensor is not on an undeclared channel (the all-channel scan
finds nothing), so it is dead, unpopulated or its XSHUT line is broken. **That
WARN is a real fault**, not a population note; do not trim the declared
channels to make it pass. The down sensor, which altitude uses, is unaffected.

The rest of this section is the earlier record, kept for its bug fixes. It was
taken before the three-sensor intent was confirmed, and it expected five. At the
time four sensors initialized and identified, all behind the ADS7128 expander at
0x17:

```console
RB_TOF init code=0 initialized=4 of 4
RB_TOF sensor=0 xshut  addr=0x31 part=VL53L5CX identity=0xf002 valid=1
RB_TOF sensor=1 xshut  addr=0x32 part=VL53L5CX identity=0xf002 valid=1
RB_TOF sensor=2 xshut  addr=0x33 part=VL53L5CX identity=0xf002 valid=1
RB_TOF sensor=3 xshut  addr=0x34 part=VL53L1X  identity=0xeacc valid=1
```

Three things came out of this that are worth keeping:

- **The parts are mixed**, and cannot be assumed. A VL53L1X read through the
  VL53L5CX's registers returns a plausible `0x0129` rather than failing, so the
  library detects the part per sensor and dispatches the address-change register
  accordingly -- 0x0001 unpaged for the L1X, 0x0004 behind a page select for the
  L5CX.
- **A real bug**: `prepare_xshut` used to drive only the *configured* channels
  low and preserve the rest. A channel left high from a previous run releases an
  untracked sensor onto the default address, the first configured sensor
  collides with it, and the sequence readdresses whichever answers first -- so
  sensors land at the wrong indices and an empty channel looks populated. It now
  holds all eight down.
- **Initialization is tolerant.** One absent sensor used to abort the whole
  array; per-sensor `status[]` plus step-level error reporting turns an opaque
  `-EIO` into `step=direct-probe-0x29 sensor=4`.

**Superseded expectation.** That session expected one directly-wired VL53L1X
plus four ADS7128-gated VL53L5CX; the real topology is the three above, with
the down sensor gated too. What it measured: no sensor survives with all eight
channels held low (so nothing is directly wired), and only three L5CX appear.
Worse, the two discovery methods disagree -- releasing channels *cumulatively*
finds L5CX on 1, 2, 3 and nothing on 4, while releasing each channel *alone*
from all-low finds one on 4 and nothing on 1, 2, 3. No single-channel-per-XSHUT
wiring explains both, so the model of the expander is wrong somewhere; a shared
enable or two channels on one net would produce exactly this. Ranging is
untouched: both drivers exist in the backend, but the VL53L5CX ULD wants 192 KiB
against this config's 32 KiB.

## SPI: the PMW3901 optical-flow sensor streams

The flow sensor works. It reports its identity and streams live motion over SPI:

```console
PMW3901 Optical Flow Sensor Test
================================

SPI and GPIO devices ready

Initializing PMW3901...
PMW3901 chip ID: 0x49, inverted: 0xB6
PMW3901 initialized successfully.

Entering continuous motion read loop...
Format: deltaX, deltaY, SQUAL, Shutter

Motion: deltaX=    0, deltaY=    1, SQUAL= 55, Shutter= 8191 [MOTION]
Motion: deltaX=   -4, deltaY=    0, SQUAL= 48, Shutter= 8191 [MOTION]
Motion: deltaX=    1, deltaY=    0, SQUAL= 58, Shutter= 8191 [MOTION]
```

`0x49` with its inverted twin `0xB6` is the PMW3901's product ID, and SQUAL in
the 48-65 range is genuine surface-quality data rather than a stuck value.

### The SPI signals were routed to no pin at all

Before this, the flow sensor was unreachable for a structural reason rather than
a subtle one. The DroneLogic shell's generated XDC constrained `i2c`, `ospi`,
`uart`, `jtag` and the clock -- and nothing else -- while its DTS carried
`spi@10031000`, `gpio@10010000` and both PWM blocks. The controllers were in the
SoC with their ports bound to nothing.
`hardware/zephyr/fpga-drone.overlay` says so in its header: "They do not assign
FPGA package pins."

`WithArty200TSPI` and `WithArty200TGPIO` in
`backends/chipyard/fpga/src/main/scala/arty200t/HarnessBinders.scala` add that
routing, and `RocketArty200TFlowSpadConfig` layers them onto the drone periphery
over the 32 KiB scratchpad, so main memory stays on the path already proven and
a failure is about SPI and nothing else.

### Pins

| Signal | Pin | J1 | Notes |
| --- | --- | --- | --- |
| `spi_sck` | F16 | J1_82 | |
| `spi_mosi` | E17 | J1_80 | |
| `spi_miso` | E16 | J1_78 | pulldown, see below |
| CS (`gpio_0`) | D16 | J1_76 | the sensor's `NCS` |
| `MOTION` | B17 | J1_66 | interrupt; not bound, the driver polls |
| reset (`gpio_1`) | A20 | J1_38 | placeholder -- see below |
| LED (`gpio_2`) | A21 | J1_57 | placeholder -- see below |

All five real signals are on **J1**, and all on its even row -- the same
connector and row as the JTAG and I2C signals that work here.

`NRESET` and `LED_N` have no net on either the FPGA or the ESP32, and the sensor
is flown without them. Their pins above are arbitrary placeholders, present only
because the GPIO block is three bits wide and `pmw3901_test`'s devicetree
contract requires all three properties. Nothing depends on them: the driver
treats reset as optional (`if (config->reset_gpio.port)`) and the LED is
cosmetic.

Motors remain deliberately unbound. Nothing in this configuration can drive a
propeller.

### How the wrong pins were caught

The first attempt used `SCLK` L21, `MISO` M21, `MOSI` N20, `NCS` M20, taken from
the bank survey in `docs/hm01b0-camera-bringup.md`. Those are J2_62/64/66/68 --
**the wrong connector** -- and the sensor read back `0xFF`.

`0xFF` is exactly what an unconnected input reads, because MISO had no pull and
a floating input drifts high. It is also what a connected-but-silent sensor
would produce, and that ambiguity is what made the reading useless. Adding a
weak pulldown removed it:

```console
PMW3901 chip ID: 0x00, inverted: 0x00
```

A weak internal pulldown is trivially overpowered by a real device asserting the
line, so a sensor connected to that pin could not read back a clean `0x00` once
CS is asserted -- not even a mis-clocked one, which would return some other
nonzero pattern. That measurement proved the pin was unconnected and sent the
question back to the wiring, where it belonged.

`misoPulldown` is now the default for exactly that reason: it makes "nothing is
connected here" and "connected but silent" stop looking alike. It needed an
`addPulldown` in fpga-shells' `XilinxShell.scala`, added beside the existing
`addPullup`.

### Implementation notes

Three details each cost a build cycle:

- The sifive SPI port is a quad interface. `dq(0)` is MOSI and `dq(1)` is MISO
  in single mode; `dq(2)`/`dq(3)` are unused. This is confirmed against the
  controller's RTL rather than assumed -- `SPIPhysical.scala:138` reads
  `val samples = Seq(rxd_fin(1), rxd_fin(1, 0), rxd_fin)`, so single-protocol
  receive samples `dq(1)`.
- The hard chip-select is **not** bound. The overlay drives CS from `gpio0` bit
  0, because the driver holds CS low across a multi-byte register transaction
  and the controller's automatic CS does not. Upstream's ESP32 sample makes the
  same choice: "CS: GPIO19 (handled as a normal GPIO in software; not hardware
  CSEL)".
- Chipyard punches **one `GPIOPort` per pin**, not one per controller, so the
  binder fires once per bit and `port.pinId` selects the pin. A `Bundle`
  declared inside a `HarnessBinder` is also invisible to the Chisel compiler
  plugin and fails at FIRRTL emission rather than at compile time, which is why
  the SPI binder uses three separate IOs.

### Controller settings, read back from hardware

| Register | Value | Meaning |
| --- | --- | --- |
| `sckdiv` | `0x0b` | 50 MHz / 24 = 2.08 MHz, the 2 MHz the driver asked for |
| `sckmode` | `0x03` | CPOL=1, CPHA=1 -- SPI mode 3, what the PMW3901 requires |
| `csmode` | `0x03` | CS off: software chip-select, matching the GPIO design |
| `fmt` | `0x00080000` | 8-bit words, single protocol |
| GPIO `output_en` / `port` | `0x7` / `0x7` | all three bits outputs, CS high at rest |

### Running it

```bash
rb chipyard build --board arty200t --config RocketArty200TFlowSpadConfig
rb build pmw3901_test --target arty200t --config RocketArty200TFlowSpadConfig
rb debug pmw3901_test --target arty200t --config RocketArty200TFlowSpadConfig \
    --usb-location 7-1.1.4.1
```


## Resolved: the console UART

**The console works.** Confirmed 2026-09-03: `hello_world` on
`RocketArty200TDroneDDRConfig` printed its whole run to channel B of the debug
FT2232H at 115200, read through
`/dev/serial/by-path/pci-0000:29:00.3-usb-0:1.1:1.1-port0`.

```console
hello world from RiskyBird
board   chipyard_riscv64
arch    riscv, 64-bit
tick 0  uptime 6 ms
...
hello world complete after 40 ticks
```

**What changed on the bench is not recorded here.** This section's measurements
were made on 2026-09-02 and were accurate then: the line was genuinely silent in
both directions, and the reasoning below narrowed that to the wire rather than to
the FPGA, Zephyr or the pin assignment. The predicted fix was continuity from
J1_41/J1_43 to the FTDI's channel B, and the console answering on exactly that
pair without a bitstream change is consistent with that having been done -- but
this session did not observe the bench being changed and does not assert it.

Everything from here down is the original investigation, kept because the method
is the transferable part: two conclusions were drawn inside it without a control
and both were wrong, and the section says how each failed.

### The state it was in

Nothing the SoC transmitted reached any serial port on this host, and nothing
sent from the host reached the SoC. Only the four JTAG wires appeared to be
connected.

### The design side is correct

Post-route `report_io` for the loaded bitstream, in
`obj/report/io.txt`:

```
| D21 | uart_txd | IO_L23N_T3_16 | OUTPUT | LVCMOS33 | bank 16 | drive 12 | SLOW |
| E21 | uart_rxd | IO_L23P_T3_16 | INPUT  | LVCMOS33 | bank 16 |          |      |
| A18 | jtag_TDI | IO_L17P_T2_16 | INPUT  | LVCMOS33 | bank 16 |          | PULLUP |
| F20 | jtag_TDO | IO_L18N_T2_16 | OUTPUT | LVCMOS33 | bank 16 | drive 12 | SLOW |
```

`uart_txd` is in **bank 16, the same bank as the JTAG pins that demonstrably
work**. So `VCCO_16` is powered at 3.3 V and its output drivers function. The
FPGA is driving D21.

Note that `uart_rxd` has **no `PULLUP`** where the JTAG inputs do. Disconnected,
it floats rather than resting high, which is consistent with the low reading
below.

The loaded bitstream is confirmed to be `RocketArty200TSpadConfig` rather than
assumed, by probing the periphery over JTAG:

```console
I2C  0x10040000 : ABSENT
UART 0x10020000 : PRESENT
GPIO 0x10010000 : ABSENT
SPI  0x10031000 : ABSENT
PWM  0x10050000 : ABSENT
```

Exactly that config: one UART, nothing else.

### The UART side is correct

Writing the divisor and reading it back returns `0x1b1` = 433, which is
50 MHz / 434 = 115.2 kBaud. `txctrl` reads `0x00010001`: TX enabled, watermark 1.

```console
--- txctrl/div after setup ---
0x10020008: 00010001
0x10020018: 000001b1
```

### The measurements

Zephyr was taken out of the picture entirely and 400 bytes were written straight
into `txdata` at `0x10020000` over JTAG, while every `ttyUSB` on the machine
listened at 115200:

```console
listening on: /dev/ttyUSB1 /dev/ttyUSB11 /dev/ttyUSB2 /dev/ttyUSB3 /dev/ttyUSB4
              /dev/ttyUSB5 /dev/ttyUSB7 /dev/ttyUSB8 /dev/ttyUSB9
/dev/ttyUSB1         0 bytes
...
/dev/ttyUSB9         0 bytes
```

Zero bytes on all nine, including the two `dialout`-only ports re-tested under
`sudo`.

**Baud mismatch cannot explain this.** A UART receiver at the wrong rate still
produces garbage bytes from a real signal; only a line with no edges at all
yields exactly zero. Nothing is reaching any FTDI RX electrically.

In the reverse direction `rxdata` reads a continuous `0x00000000` -- the empty
bit clear with a data byte of zero, which is a line held low rather than an idle
one -- and none of three distinct probe bytes driven from three different
channels ever appeared in it.

D21 and E21 are also not shorted **to each other**: after transmitting `0x5A`
and draining the RX FIFO, the only distinct byte ever read back is `0x00`.

### The header pins

The console pair sits on the same connector as the working JTAG block:

| Signal | FPGA pin | J1 pin | Status |
| --- | --- | --- | --- |
| `jtag_TCK` | A19 | J1_46 | works |
| `jtag_TDI` | A18 | J1_48 | works |
| `jtag_TDO` | F20 | J1_50 | works |
| `jtag_TMS` | F19 | J1_52 | works |
| `uart_rxd` | E21 | **J1_41** | dead |
| `uart_txd` | D21 | **J1_43** | dead |
| `uart_tsi_rxd` | P16 | J1_92 | dead |
| `uart_tsi_txd` | U18 | J1_85 | untested |

Every working pin is even and both dead console pins are odd, which on a
dual-row header raises the obvious hypothesis: one row landed and the other not.

That hypothesis was tested and **the test came back against it, but not
cleanly.** `uart_tsi_rxd` on J1_92 is even, the same row as the working JTAG
pins, and it does not work either: a UART-TSI write of `0x12345678` to
`0x8000_1000`, sent on each candidate channel, left the word at its JTAG-written
sentinel `0xaaaa5555` both times.

```console
  before: 0x80001000: aaaa5555
  pyuartsi: W: 0x80001000 <= 0x12345678
  after : 0x80001000: aaaa5555
```

Two caveats keep this from being a clean refutation, and both matter if the row
theory is worth pursuing:

- **The TSI pins are in bank 14, not bank 16.** Bank 16 is proven live because
  JTAG works on it. Nothing here proves bank 14 is powered at all -- the design
  *requires* `VCCO_14` = 3.30 V, which is a constraint, not a measurement. A dead
  bank 14 would produce the same result as a dead wire.
- The test assumes the TSI adapter accepts a write without a handshake. That is
  true of the protocol, but only the inbound direction was exercised.

So the row split is not disproven, merely unsupported. The simpler reading that
fits everything measured is that **only the four JTAG wires were ever landed**,
and either way the console pair on J1_41/J1_43 -- in the bank already proven
live -- is the thing to wire next.

### A false lead worth recording

`pyuartsi --init_read 0x02000000` appears to succeed on two of the channels:

```console
zephyr-chB     /dev/ttyUSB5    921600  R: 0x2000000 => 0xfa00fa
zephyr-chB     /dev/ttyUSB5    115200  R: 0x2000000 => 0xfa00fa
bitstream-chB  /dev/ttyUSB11   921600  TIMEOUT (no TSI response)
bitstream-chA  /dev/ttyUSB4    921600  R: 0x2000000 => 0xfa00fa
```

This is **not** a working TSI link. A real UART cannot work at 921600 and 115200
simultaneously, and `0xfa00fa` is not a plausible value for a CLINT `msip`
register. Writing a known pattern and reading the same port back shows what is
really happening:

```console
zephyr-chB     /dev/ttyUSB5   sent 16 got  16  b'\xfaa\xff\xfad\xfae\xff\xfah\xfai\xff\xfam\xff'
bitstream-chB  /dev/ttyUSB11  sent 16 got   0  silent
bitstream-chA  /dev/ttyUSB4   sent 16 got  16  b'\xfaa\xbf\xfad\xfae\xff\xfah\xfai\xff\xfam\xf7'
```

Sixteen bytes out, sixteen back, with `a`, `d`, `e`, `h`, `i`, `m` from the
transmitted `a`..`p` still recognisable among the framing noise. Those two
channels echo the host's own transmissions, so any tool that writes and then
reads will invent a plausible-looking answer on them. This is why the TSI test
above is checked by reading memory over JTAG rather than by believing the tool.

Unrelated but worth knowing: `pyuartsi --help` documents `--init_write
addr:value`, but the code splits on `=`. Use `--init_write 0x80001000=0x12345678`.

### The pin-swap experiment, and what it ruled out

If the ribbon were crossed -- the FTDI's TX landing on J1_43 and its RX on
J1_41 -- every measurement above would follow from one mistake. So
`RocketArty200TSpadSwapUartConfig` was built and loaded: identical to
`RocketArty200TSpadConfig` except `uart_rxd` on D21 and `uart_txd` on E21.

**It failed identically.** No console bytes in either pin arrangement. A crossed
pair is therefore ruled out, and with it the pin assignment as a whole: both
orderings were tried on hardware and neither works.

Two intermediate readings during that experiment suggested the swap had found a
signal. Both are **retracted**, and they are recorded here because the way they
failed is the useful part.

- *Polling `rxdata` over JTAG* appeared to show the RX FIFO filling only while
  `/dev/ttyUSB5` transmitted, with an A/B/A/B control that looked clean. A
  CPU-side probe using `uart_poll_in` -- the same register, read at full speed
  by the hart instead of through the debugger -- saw **zero** bytes under the
  same conditions. The readings were also unstable between runs (`49/254`,
  `0xff`, `0xf1`, `241`), and no divisor from 104 to 867 decoded any of them.
  The CPU-side measurement is the trustworthy one.
- *A static pin-level probe* (`ftdi get_signal` on the FTDI's RX pin, the
  technique `hardware/openocd/tdo-sense.cfg` uses) reported the pin never going
  low while the FPGA held `uart_txd` low. **Its control failed**: driving the
  FTDI's own TX pin low, high, and high-Z produced the same reading every time,
  so the probe was not reading real levels and proves nothing either way.

The lesson generalises: on this bench, any measurement that reaches the UART
through the debugger needs a control run that can distinguish a real signal from
an artifact, and a probe needs a positive control proving it can detect the
thing it is looking for. Two conclusions were drawn here without one and both
were wrong.

### What is actually established about the console

Positively, and with controls:

- **The UART transmits.** `uart_sifive_poll_out` spins on the FIFO-full bit, so
  a UART that never drained would hang the program. The program runs to
  completion, so bytes are leaving the shift register.
- **The driver is correct and is the sifive one.** `CONFIG_UART_HTIF` is unset,
  the ELF contains no HTIF symbols, and the `htif` node is disabled. `div` reads
  `434` before Zephyr -- Chipyard's hardware init value -- and `433` after, with
  `txctrl`/`rxctrl` going from zero to enabled. That transition is the driver
  running.
- **The divisor arithmetic is right, and confirmed against hardware.** The
  peripheral clock is computed, not read from devicetree:
  `SIFIVE_PERIPHERAL_CLOCK_FREQUENCY = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC *
  CONFIG_RTC_CLOCK_DIVIDER_VALUE` = `50000 * 1000` = 50 MHz, giving
  `50e6/115200 - 1 = 433`. An independent CLINT measurement agrees: `mtime`
  advances 100337 ticks in 2.0 s, a 50168 Hz timebase against the expected
  50 kHz.
- **Nothing arrives, and not because of baud.** Zero bytes on all nine serial
  ports, listener verified open, with and without OpenOCD attached. A receiver
  at the wrong rate still emits garbage from a real signal; exactly zero means
  no edges at all on the FTDI's RX pin.

The one reproducible electrical oddity: writing 16 bytes to the zephyr FTDI's
channel B returns exactly 16 bytes, every one `0xff`, on **both** bitstreams --
including the swapped one, where the FPGA pin is an input and cannot contend.
Byte count preserved, every data bit high. That is what a floating RX pin does
when it capacitively picks up the start edge from its neighbouring TX line and
then drifts back high.

Assessment, not proof: the FPGA transmits and nothing reaches the FTDI's RX pin,
which points at the `uart_txd` -> FTDI-RX wire rather than at the FPGA, Zephyr,
or the pin assignment. Settling it needs a meter.

### What the investigation predicted, and how it landed

The single measurement it asked for was **continuity from J1_41 and J1_43 to the
zephyr FT2232H's channel B**, with the FTDI's TX landing on J1_41 (the FPGA
input) and its RX on J1_43. That was named as "the single change that would make
the console work; everything else is verified." The console now works on that
pair, with no change to the bitstream, which is the outcome that prediction
implies.

Two items from that list are **still open** and are not closed by the console
working:

- **The `0xff` echo on channel B**, present on both bitstreams including the
  swapped one where the FPGA pin is an input and cannot contend. Whatever
  explains it invalidates every write-then-read test on that channel, so it is
  worth understanding before trusting any such test again.
- **`uart_tsi_txd` on J1_85** is still the one pin never exercised in any
  direction, and the DDR configs carry no TSI client at all.

The pin ordering is settled by the working console: `RocketArty200TSpadConfig`'s
E21=rxd / D21=txd is correct, and `RocketArty200TSpadSwapUartConfig` -- built
only to test a crossed ribbon -- has served its purpose. Its own doc comment says
"Delete this once the wiring is settled."

## What this does establish

- The two FT2232Hs can be addressed independently and identified by IDCODE.
- The RISC-V debug TAP answers on A19/F19/A18/F20 at 2000 kHz.
- The core is a full RV64GC Rocket with FPU, MMU, S and U modes.
- Main memory is 32 KiB at `0x8000_0000`, bounded by measurement.
- A Zephyr image builds, loads over JTAG, and verifies byte-for-byte.
- It runs: the kernel boots, schedules, formats output, and keeps time
  accurately, and reaches the end of `main`.
- **I2C works, and the ribbon to the drone board carries signal.** Six addresses
  ACK, four of them devices this repository already describes.
- **SPI works and the PMW3901 streams.** Chip ID `0x49`, live motion bursts with
  real SQUAL values. The SPI and GPIO signals had never been routed to a pin on
  this board before.
- **The console UART works** (2026-09-03), on E21/D21 through J1_41/J1_43. This
  supersedes the conclusion the bulk of this document argues for; see
  [Resolved: the console UART](#resolved-the-console-uart).

## What this does not establish

- **Any I2C sensor's data.** The scan proves six devices ACK their address; not
  one register has been read from any of them. The two unidentified addresses
  are unidentified.
- **Sustained or accurate flow.** The sensor identifies itself and reports
  motion, but the readings have not been checked against controlled movement,
  and `MOTION` (B17) is unbound so the driver polls rather than taking the
  interrupt.
- **Motors and PWM.** Still unbound to any pin, deliberately.
- **Anything about DDR3.** These configs have no MIG; the scratchpad at
  `0x8000_0000` is on-die. DDR3 on this carrier is a separate document, and it
  now passes 960 MiB -- see
  [`te0712-drone-carrier-ddr-bringup.md`](te0712-drone-carrier-ddr-bringup.md#measured-on-hardware).
- **Booting without a debugger.** Every run here was loaded over JTAG. Nothing
  has been written to flash and no power-cycle boot has been attempted.
- **Why the console came up.** It works now and did not before; what changed on
  the bench between the two sessions is not recorded here.
