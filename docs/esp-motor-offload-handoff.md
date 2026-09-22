# Handoff: the motor PWM offload to the ESP32-C6

**Written 2026-09-21 on the Linux build host, for a Mac with the drone attached
and no memory of the session that produced it.** Everything you need is restated
here. Where a number came from a hardware run it says **measured**; where
something was never run it says so in as many words. The unverified list in
section 5 is the most valuable part of this document — do not skim it.

Evidence vocabulary, as used elsewhere in this repo: **measured** (run on this
hardware, output quoted), **verified** (read out of the tree or proven by a
build), **declared** (asserted by code or docs, never run), **unverified**.

---

## 0. If you read nothing else

1. **Motor 4 was never a software bug.** FPGA ball **F13** — motor4's gate — has
   a **dead output driver**. The net to the FET is intact. There is no RTL fix.
2. The original premise ("motor 4 works on `main`, not on `setup`") was **false**:
   both trees produce a byte-identical bitstream, DTS, overlays and driver.
   Motor 4 was dead on both.
3. **The fix is an offload.** The FPGA now drives **no motor gate at all** and
   sends 14-byte duty frames over the existing uart1 to the ESP32-C6, which owns
   all four gates. `flight_controller --mode esp-motors` + `esp_motors`.
4. **Motor 4 turns again**, measured, from an FPGA command through the ESP.
5. **You do not need a new bitstream.** `RocketArty200TDroneFullDDRConfig` is
   unchanged; sha256 `8b21446ac76266e2305ac3bc6ab0132e8e28fc925af36229ba6318908c19b1a3`.
6. **ASSUME THE BATTERY IS LIVE.** A ~40 s powered motor run happened during this
   session because the session believed the motors were inert. They were not.
7. Fastest stop is **pulling the battery**. Second is stopping the frame stream —
   the ESP cuts all four within **120 ms** (measured, `cuts=1`).
8. Motors **latch off 5 s** after they first spin unless you commit with **both**
   `RB_CMAKE_ARGS="-DROSE_FLIGHT_COMMIT=1"` **and** `hardware/flight/commit.gdb`.
9. `state[3..5]` are **Rodrigues/Gibbs** parameters, `tan(theta/2)` — **not
   radians**. Misreading them put a "57 degree" tilt guard at a real 90 degrees.
   Never reintroduce that.
10. **Nothing about hover has been measured.** `MOTOR_MAX_DUTY` 0.10 is below
    this airframe's 0.15 break-away; the flow scale is uncalibrated. Do not fly.

---

## 1. What happened, and what is proven

### 1.1 The false premise

The session opened with "motor 4 doesn't spin on the `setup` branch but works on
`main`". That is **not true**. The two trees were compared file by file: same
bitstream sha256, same elaborated DTS, same overlays, same `pwm_sifive` driver,
same duty constants. Motor 4 was dead in both. Do not go looking for a branch
difference; there isn't one.

### 1.2 F13 is dead as an output — measured, twice, with no software in the path

The path under test is `F13 → J1_100 → JB1 pad 99 → 47R (R26) → Q4 gate`. The
ball-to-J1 half of that is verified in this tree (`docs/motor-pwm-bringup.md`
quotes the board file: `<pin index="88" name="J1_100" loc="F13"/>`); the
carrier-side designators come from the **v3 KiCad netlist on the
`riskybirdv3-layout` branch of the PCB repo**, which is not checked out here, so
take `JB1 99 / R26 / Q4` as read from there rather than confirmed in this tree.
**Nothing below depends on those designators** — the measurement is made at the
gate node itself.

Two bare-metal Verilog bitstreams were built for this. **No Rocket, no TileLink,
no Chipyard, no Zephyr** — just `IBUFDS`, a counter and output buffers.

**Test 1, `hardware/padtest/`.** Drives the four motor pads at four distinct
duties so the ESP's high-impedance gate probe (`workloads/motor_safe`) can
identify each one in a single reading:

| pad | ball | driven | read at the gate |
|---|---|---|---|
| motor1 | E22 | 4 % | **40** per mille |
| motor2 | B20 | 8 % | **80** per mille |
| motor3 | R17 | 12 % | **120** per mille |
| motor4 | **F13** | 16 % | **0** |

Three exact. F13 nothing.

**Test 2, `hardware/padtest/f13probe/`.** This is the one that closes it. F13 is
configured as an **INPUT with the FPGA's internal pullup**, and what it reads is
echoed onto a known-good pad. Every gate node on the carrier has a 10k pulldown:

- F13 **open** → the internal pullup wins outright → reads a solid **1**
- F13 **wired** → the pullup fights the 10k → the node sits at or below mid-rail,
  well under LVCMOS33 VIH(min)=2.0 V → reads **0**

It read a **solid, stable 0**. The carrier's 10k gate pulldown is reachable from
the ball, so the wire, the 47R, the board-to-board pin and the FET are all fine
and the IOB is not. A connected net cannot present a valid logic 1 against that
10k; a clean 1 would have meant an open. It read 0.

### 1.3 Everything else was eliminated

- **The RTL and the constraints are correct.** In the FullDDR build's own
  `report_io` and generated `*.shell.xdc`, motor4 is `F13 / OUTPUT / LVCMOS33 /
  bank 16 / drive 12 / SLOW / FIXED / PULLDOWN` — **character-for-character the
  same attribute set as motor1 and motor2**, which both work.
- **It is not a dead bank.** F13 is in **bank 16 together with motor1 (E22) and
  motor2 (B20)**, both of which work. `VCCO_16` is powered and its drivers work.
- **Post-route structure is identical** to the working three. F13 places at
  `IOB_X0Y248` — confirmed for the bare-metal builds in
  `hardware/padtest/verify.out:106` (as `IOB33.OUTBUF`) and
  `hardware/padtest/f13probe/verify.out:97` (as `IOB33.INBUF_EN`). In the full
  design the same OBUF is driven by a LUT4 out of the second PWM clock-domain
  wrapper, the same shape as motors 1–3. **That full-design post-route check was
  made during the session and its report is not in either tree**, so treat it as
  recorded rather than reproducible from here; the XDC and `report_io` evidence
  above is, and it is checkable in the Linux tree at
  `backends/chipyard/fpga/generated-src/chipyard.fpga.arty200t.Arty200THarness.RocketArty200TDroneFullDDRConfig/obj/report/io.txt`.
  > Caveat, recorded in `docs/motor-pwm-bringup.md`: do **not** read a
  > post-synthesis LUT INIT for *intent*. Synthesis folds the harness inverter
  > into the LUT, and an earlier attempt to read `LUT4 INIT=16'h6A55` that way
  > reached the opposite, wrong conclusion. The load-bearing claim here is
  > *structural identity with the working motors*, nothing more.
- **The probe is not lying.** `workloads/motor_safe` self-tests each pin by
  driving it high and reading it back through its own input buffer; all four
  input paths, GPIO22 included, reported `input path OK`.
- **Reseating the TE0712 changed nothing.**
- **It used to work.** `setup-vs-actual.md` (run date 2026-09-09) records an
  operator watching all four motors turn. This is a fault that **developed**.
  That file lives at the root of the **Linux `riskybird-dev` checkout only**; it
  is not on this branch.

### 1.4 The fix

`flight_controller --mode esp-motors` on the FPGA plus `esp_motors` on the
ESP32-C6. **The FPGA runs all the flight logic and only COMMANDS. The ESP
computes nothing and is purely a PWM driver** — verified by reading
`workloads/esp_motors/src/main.c`: there is no PID, no estimator, no IMU code in
that image at all, and its whole frame handler is `motors_apply(cmd->duty)` →
`pwm_set_dt()`.

The ESP takes **all four** gates, not just motor 4. Each gate is a wired-OR of an
FPGA 47R and an ESP 47R into a 10k pulldown; leaving three motors dual-driven
would keep the contention that parks a contested gate near half rail. One owner
per gate.

**The link** is the second UART that was already routed and already described:

```
uart1_rxd  E13  J1_96   FPGA input   <- ESP TX   (ESP GPIO16 / U0TXD)
uart1_txd  F14  J1_98   FPGA output  -> ESP RX   (ESP GPIO17 / U0RXD)
```

Bank 16, unrelated to the dead pad.

**The wire format** is 14 bytes: magic `0xA5 0x5A`, seq, flags, 4x uint16 duty in
units of 1/10000, CRC-16/CCITT-FALSE over bytes 0..11, with a 14-byte sliding
window on the receiver so resynchronisation is guaranteed within one frame.
There is **one shared copy** of it, at
`integration/zephyr/motor_link/include/riskybird/motor_link.h`, and both ends
compile that same header — the FPGA side reaches it through the
`ROSE_MOTOR_LINK_DIR` CMake define set in `tools/rb/boards.py`.

---

## 2. Current state of the tree

You are on branch `setup`. The offload work is here:

| path | what |
|---|---|
| `integration/zephyr/motor_link/include/riskybird/motor_link.h` | the wire contract, shared by both ends |
| `workloads/esp_motors/` | the ESP32-C6 PWM driver image |
| `hardware/zephyr/targets/esp32c6/workloads/esp_motors.overlay` | ESP pinmux; console moved to USB Serial/JTAG |
| `hardware/zephyr/targets/fpga/workloads/flight_controller-esp-motors.{conf,overlay}` | the FPGA mode |
| `tools/rb/boards.py` | `--mode esp-motors` and the `esp_motors` workload row |
| `hardware/flight/commit.gdb` | the run-time half of the flight-commit interlock (new) |
| `hardware/flight/guard-check.gdb` | read the envelope guard's own view over JTAG (new) |
| `hardware/flight/stop.gdb`, `estop.gdb` | rewritten for the offload path |
| `docs/flight-safety-bench-procedure.md` | the full bench procedure |
| `docs/motor4-remote-investigation.md` | the original work order, for background |
| `backends/zephyr-chipyard-sw` | submodule `dev`, **with uncommitted changes** |

### 2.1 It builds ONLY in this tree — verified

The Zephyr in `backends/zephyr-chipyard-sw/zephyr_ws/zephyr` in **this** tree
carries a guard that the `main` tree's does not. In `drivers/serial/uart_sifive.c`:

```c
/* Guarded to match instance 0 above: without CONFIG_PINCTRL the device config
 * struct has no pcfg member, so an unconditional pinctrl definition here makes
 * ... */
#ifdef CONFIG_PINCTRL
PINCTRL_DT_INST_DEFINE(1);
```

The `main` tree has `PINCTRL_DT_INST_DEFINE(1);` **unguarded** at the same place,
so `uart_sifive.c` does not compile with uart1 present at all. The offload needs
uart1. Do not try to build this mode from the `main` tree.

### 2.2 Things this branch does NOT have — read this before hunting for them

Several artefacts referenced in this document and in the source comments live
**only in the Linux `riskybird-dev` (`main`) checkout** and were never ported to
`setup`:

- **`hardware/padtest/` and `hardware/padtest/f13probe/`** — the two bare-metal
  bitstreams, their Vivado `verify.out` reports and their `.bit` files. This is
  the entire physical proof for section 1.2. You cannot rebuild them on a Mac
  (no Vivado, and `backends/chipyard` is an **empty directory** on this branch).
- **`integration/zephyr/safety/motor_guard.{h,c}`** — `main.cpp`'s
  `accel_envelope()` cites it as the origin of its thresholds. It is a dangling
  reference on this branch. Nothing links against it, so nothing is broken; the
  thresholds are restated inline in `main.cpp`.
- **`docs/hm01b0-colour-investigation.md`** and the repaired
  `hardware/ospi/frame-to-png.py` (see section 5.4). This branch still has the
  **old 139-line** `frame-to-png.py` — five arguments, no `--no-repair`, none of
  the repair, line-anchoring or mosaic-detection work. The Linux tree's version
  is 1180 lines.

If you need any of them, they have to be fetched from the Linux host.

### 2.3 Parallel work in flight

While this was written, other agents were editing
`tools/rb/boards.py`, `controller_tinympc.cpp`, `src/main.cpp`, `flow.c`,
`pmw3901.c`, `workloads/pmw3901_test/`, `workloads/motor_duty/`,
`tools/flow-cal.py`, `tools/rb-ext.py` and `docs/flight-calibration-procedures.md`.
Expect `git status` to show them dirty, and expect
`docs/flight-calibration-procedures.md` to exist by the time you read this — it
did not exist when this was written. `workloads/motor_duty` is registered through
the `tools/rb-ext.py` shim rather than through `boards.py`, precisely because two
people were holding the registry at once; invoke it as
`./tools/rb-ext.py build motor_duty ...`.

---

## 3. How to build and run it, on a Mac

### 3.1 The Mac cannot build bitstreams, and does not need to

`backends/chipyard` is empty on this branch and there is no Vivado. `rb` was made
to work without either: `Board.generated_dts` and `Board.bitstream` fall back to
collected directories pointed at by `RB_DTS_DIR` and `RB_BITSTREAM_DIR`. That
fallback is what makes a Mac usable at all (see `docs/merging-into-main.md`).

**The FullDDR bitstream is UNCHANGED by this work.** The offload is entirely a
software change: same shell, same PWM blocks (now simply unused), same DTS. If
`~/riskybird-bitstreams/arty200t/arty200t.RocketArty200TDroneFullDDRConfig.bit`
has sha256

```
8b21446ac76266e2305ac3bc6ab0132e8e28fc925af36229ba6318908c19b1a3
```

you have the right one. (9,730,764 bytes, part `xc7a200tfbg484-1`, built
2026-09-03, timing met, WNS 0.023 ns.) Nothing in this handoff asks you to
build or program a new one.

Every shell that runs `rb` needs both variables, so source the helper:

```bash
cd ~/riskybird-dev            # your setup-branch checkout
source ./rb-env.sh
echo "[$RB_DTS_DIR] [$RB_BITSTREAM_DIR]"    # both must be non-empty
```

`rb-env.sh` defaults them to `~/riskybird-dts` and `~/riskybird-bitstreams`.
`docs/setup.md` steps 2d and 3c say how to populate those.

### 3.2 Build both halves

```bash
# FPGA side: flight logic, no motor pin driven, duty frames on uart1
./rb build flight_controller --target arty200t \
    --config RocketArty200TDroneFullDDRConfig --mode esp-motors

# ESP32-C6 side: the PWM driver
./rb build esp_motors --target esp32c6
```

Build outputs land at `build/zephyr/<target>/<workload>/<mode>/zephyr/zephyr.elf`,
so the FPGA ELF is
`build/zephyr/arty200t/flight_controller/esp-motors/zephyr/zephyr.elf`.

### 3.3 Prove the build is uncommitted before you plug anything in

The portable check, which needs nothing but `grep` — this must print **`0`**:

```bash
grep -c g_flight_commit_key \
  build/zephyr/arty200t/flight_controller/esp-motors/zephyr/zephyr.map
```

Verified on this host against the uncommitted build in this tree: it prints `0`.

The authoritative form is `riscv64-zephyr-elf-nm` on the ELF, which must print
nothing (the committed build shows `B g_flight_commit_key`; that pair was
verified with `nm` earlier in the session). Note that `rb` **discovers** the SDK
itself rather than reading `ZEPHYR_SDK_INSTALL_DIR` out of your shell, so do not
assume that variable is set — print the path `rb` will use with

```bash
python3 -c 'import sys; sys.path.insert(0,"tools"); \
  from rb.environments import ZEPHYR_SDK; print(ZEPHYR_SDK)'
```

and look for `riscv64-zephyr-elf/bin/riscv64-zephyr-elf-nm` under it. On this
Linux host that directory is not where the SDK is actually installed, so treat
the answer as a starting point and fall back to the map-file check above.

If the symbol is absent, the image **cannot** be committed at run time and the
motors will latch off 5 s after they first spin. That absence *is* the interlock.

### 3.4 Flash the ESP, then run the FPGA

```bash
./rb flash esp_motors --target esp32c6 --port "$RB_ESP_PORT"

./rb run flight_controller --target arty200t \
    --config RocketArty200TDroneFullDDRConfig --mode esp-motors
```

`rb run` programs the bitstream, loads the ELF over UARTTSI and opens the
console. `--no-program` skips the bitstream if it is already loaded.

> **On macOS**, the FT2232H presents two devices distinguished by the trailing
> letter: `/dev/cu.usbserial-<serial>A` is channel A (JTAG),
> `...B` is channel B. See `docs/setup.md`.

### 3.5 Flashing `esp_motors` DESTROYS your instrument

`workloads/motor_safe` parks the same four ESP pins as pulled-down inputs and
reports what the FPGA delivers to each gate. **That probe is where every motor
measurement in section 1.2 came from.** `esp_motors` drives those same four pins.
They cannot coexist; flashing one replaces the other. Reflash `motor_safe` to get
the instrument back:

```bash
./rb flash motor_safe --target esp32c6 --port "$RB_ESP_PORT"
```

To keep some observability, `esp_motors` prints a deliberately parallel line —
`MOTOR CMD duty per mille` against `motor_safe`'s `MOTOR GATE duty per mille`.
**They are different claims.** GATE is what was measured at the pin; CMD is only
what was commanded.

### 3.6 ESP flashing fails when the part is several USB hubs deep

**Measured this session:** with the ESP32-C6 three hubs deep at 12 Mbps, every
write died mid-erase and left the part blank. Reseat to a shallower port before
concluding the part is dead. The USB-JTAG is in silicon on the C6, so a blank
part still enumerates and is always recoverable.

---

## 4. What is MEASURED

Everything in this section was run on this hardware today, 2026-09-21. The
console text is quoted as it was seen.

### 4.1 The link is clean

```
693 frames, lost=0 stale=0 crcerr=0 noise=0
```

with the inter-frame gap between **25 and 39 ms**. `noise=0` after the first
frame is the signature of a correct baud rate with nothing else on the wire;
a rising `noise` count would mean the wrong rate, and a rising `crcerr` would
mean corruption — different faults needing different fixes.

**115200 is therefore MEASURED.** Three places in the tree still say otherwise
and are now **stale**, and should be corrected in a follow-up commit:

- `hardware/zephyr/fpga-esp-uart.overlay` — "a starting point, not a measured
  value"
- `hardware/zephyr/targets/esp32c6/workloads/esp_motors.overlay` — "treat this as
  UNVERIFIED until the bench says otherwise"
- `integration/zephyr/motor_link/include/riskybird/motor_link.h`, LATENCY BUDGET
  section — "Until that is settled on the bench, treat 115200 as unverified"

If you ever change the rate, change it in **both** overlays in the same commit,
and confirm it with `workloads/esp_uart`, which sweeps candidate rates and
reports per-rate byte counts.

> A separate, earlier run of the same link is recorded in `tools/rb/boards.py` as
> **1212 frames, zero CRC/noise/loss**. Two runs, same result.

### 4.2 The ESP boots safe

Before any frame arrives, the ESP reports all four motors at 0 and

```
link=DOWN -- MOTORS OFF (failsafe)
```

The failsafe deadline starts **already expired**, and the motors are written to 0
before the UART is even enabled. Nothing can drive a gate until a valid frame has
arrived.

### 4.3 Motor 4 turns again

**Measured, with the battery connected:** motor 4 spins from an FPGA command
delivered through the ESP. This is the whole point of the exercise.

### 4.4 The failsafe cuts

Halting the Rocket core stopped the frame stream, and the ESP cut all four
motors with

```
cuts=1
```

**This is a real safety gain, not a workaround.** Today, on the PWM path, the
SiFive comparators are free-running hardware: halting the core — which `rb debug`
does routinely — leaves them driving the gates at the last programmed duty
**forever**, until a reconfigure or a power cycle. On the offload path a halted
core, a crash, a reset or a pulled ribbon all cut the motors in **≤120 ms**
(+5 ms tick granularity). "Forever" became "≤120 ms".

### 4.5 The tilt guard fires

```
*** EMERGENCY CUTOFF *** tilt limit exceeded: measured 47.457 deg est-tilt, held 141 ms over 5 samples; motors OFF (reset to clear)
```

That is the fixed guard doing exactly what its constant now says: 40 degrees,
debounced in **milliseconds** rather than iterations, and reported in degrees
rather than in Gibbs units.

### 4.6 Attitude authority is restored

With **props ON** and the drone rocking in its restraint:

```
duty=[0.046 0.051 0.100 0.088]
```

A real 2:1 correction across the motors. **Before the anti-saturation fix all
four duties were identical on every single sample** — see bug 3.

### 4.7 The 0.6–1.5 g accelerometer trust band survives real vibration

The accelerometer tilt test is skipped whenever `|a|` falls outside 0.6 g … 1.5 g,
because the direction of a vector that is not gravity says nothing about
attitude. The open question was whether a spinning airframe is noisier than that
band. **Measured, props ON, restrained:**

| statistic | value |
|---|---|
| accel magnitude min / mean / max | **7.85 / 9.90 / 12.46 m/s²** |
| samples outside the 0.6–1.5 g trust band | **0 of 44** |
| accel-tilt mean | **10.7 deg** |

One sample read **41 deg**, and it was **real tilt, not vibration** — the
estimator independently said 33 deg on the same sample. The trust band holds at
these duties.

> Scope: this is at bench duties with the airframe restrained. It is **not** a
> measurement of free-flight vibration at hover thrust.

### 4.8 The commitment gate latches

The window counted from `0` to **`5009 ms`**, then latched all four motors off.
The ESP saw

```
flags=0x10[WINDOW-EXPIRED]
```

> `docs/flight-safety-bench-procedure.md` predicted `flags=0x11`. **The
> prediction was wrong and the measurement is right**: when the gate refuses, the
> sender clears ARMED *and* sets WINDOW, so `0x10` is correct. That doc was
> written before any of this ran; its section 5 ("Not verified — nothing here has
> run on hardware") is now partly superseded by this section.

---

## 5. What is NOT verified

Stated plainly, because these are the ways this could still hurt someone.

### 5.1 The controller stack switch has never been built-and-run on hardware

`--mode esp-motors` now declares `ROSE_USE_PID=1`, `ROSE_USE_EKF=1`,
`ROSE_FLOW=1` and `PID_MASS_KG=0.060f` in `tools/rb/boards.py`. Before this
session it inherited the sample's CMake defaults, which are **TinyMPC, no optical
flow, and a 32 g Crazyflie's mass**. The new combination has **never been built
and run on this hardware**. Build it and read the banner before trusting it.

Related and also unverified: `ROSE_FLOW=1` is **inert on this target today**.
`flow.c` hard-codes `#define CS_GPIO_PIN 19` — the ESP32-C6's chip-select — with
no `#ifndef` around it, while this carrier wires CS to `gpio0` pin 0. An
unguarded `#define` cannot be overridden from a `-D` or from devicetree, so the
PMW3901 chip-ID read fails and `flow_init()` prints `PMW3901 not detected --
flow disabled` after five 50 ms retries, costing ~250 ms of boot. The knob is
declared so the mode *states* the flight stack, and it starts working the moment
`flow.c` takes its CS pin from the devicetree.

### 5.2 Thrust and hover duty are completely unmeasured

- `MOTOR_MAX_DUTY` defaults to **0.10**. This airframe's measured break-away duty
  is **0.15** (`MOTOR_BREAKAWAY_DUTY` in `CMakeLists.txt`: 15 % starts all four,
  10 % starts three). **0.10 is below break-away AND it saturates** — it cannot
  reliably start a motor, and the controller pins against it.
- A recommendation of **0.30** exists. It is **calculated, not measured**: roughly
  30 % of the airframe's weight, chosen to be visibly responsive and *unable to
  lift*. Nobody has put a scale under this drone.
- **No thrust curve exists.** `workloads/motor_duty` was written this session to
  produce one (it holds a fixed duty on chosen motors by sending `motor_link`
  frames), but the curve has not been taken.

### 5.3 `FLOW_RAD_PER_COUNT` is uncalibrated

`flow.c` carries `FLOW_RAD_PER_COUNT 0.00205f` as a default. It has not been
calibrated for this lens, this sensor or this height. **This is the main risk to
a 30-second hover**: an uncalibrated flow scale means horizontal drift, and the
drone will walk off its spot. `tools/flow-cal.py` solves the constant from hand
slide captures (`K = D / (h * |sum(counts)|)`; `dt` cancels, so no motion rig is
needed); run it before any hover attempt. Its `--self-test` proves the maths with
no hardware.

### 5.4 The camera does not deliver colour, and the mechanism is contested

**No Bayer mosaic arrives in any captured frame.** That much the data settles.

The mechanism is an **under-settled sample** on **one of the four 2x2 phases** —
always the same one, even row and even active column on the line-anchored grid —
read before it has settled and left part of the way from the pedestal to the real
signal.

> **Correction to what the session believed.** An earlier framing of this called
> it a "column sample-and-hold **inside the sensor**". **The repository says the
> opposite, in three places**: `hardware/ospi/frame-to-png.py:41-43` and
> `:368-369` note the defect is present in captures taken *before the camera
> module was swapped*, "so this is the capture path and not the sensor", and
> `:98-101` states "the fix belongs in the capture RTL, not in a sensor register
> table". Treat it as a **capture-core / capture-path defect**. The phrase
> "sample-and-hold" appears nowhere in the tree, and it is a *phase* defect, not
> a *column* defect.
>
> A second claim from the session — an "output-code-lattice" argument with
> "25596/25596 on-lattice" excluding downstream arithmetic — **could not be found
> anywhere in the repository**. Do not cite it. The real argument in the doc is
> that the effect **is a function of the code value, not of the scene**: the
> damaged phase's ratio falls from 0.98 to 0.72 as the *digital number* rises,
> identically across three exposures, which no dye can do. Separately, the
> capture is a `dump binary memory` of the core's own bytes with no host-side
> processing in between.

**A software repair ships** in the Linux tree's `hardware/ospi/frame-to-png.py`:
it fits alpha and beta per frame and applies `true = (v - beta) / alpha` to the
damaged phase on the way to the PNG. Measured 2x2 phase spread on frame C:
**21.69 % → 1.13 %**. Two caveats: that pair is for **frame C only** (frames A
and B start at 15.29 % and 15.55 %), and the doc gives **two inconsistent tables**
for the after-repair numbers on A, B and CTRL — only frame C agrees with itself.

The board owner believes the fitted part is a colour sensor. The data says no
chroma arrives. **Unresolved.** The definitive bench test is in section 8 of
`docs/hm01b0-colour-investigation.md`: a matte white card filling the frame,
captured twice — once lit by a phone screen at full-screen pure red (255,0,0),
once at pure blue (0,0,255), at ~10 cm. Phone primaries are narrow-band and carry
essentially no IR; do **not** use incandescent, halogen or sunlight. The statistic
is `D = r(RED) - r(BLUE)`: `|D| < 0.02` means mono, `|D| > 0.30` means colour.
The doc also notes that simply reading the part number off the module takes 30
seconds and would settle it faster.

**None of this is on the `setup` branch.** The doc and the repaired script exist
only in the Linux `riskybird-dev` checkout; this branch has the old 139-line
`frame-to-png.py`. The capture artefacts also differ: the `bootup-frame.raw` at
the root of **this** tree is the doc's **frame B**, while the Linux tree's
same-named file is **frame C** — same byte count, different capture, different
md5. Do not assume a `bootup-frame.raw` is the one a number was quoted from.

---

## 6. Bugs found. Do not reintroduce any of these.

### Bug 1 — `state[3..5]` are Gibbs parameters, not radians

`state[3..5]` are **Rodrigues (Gibbs) parameters**, `r = q_xyz / qw` — see
`estimator.hpp` and both `get_state()` implementations, which divide the
quaternion vector part by `qw`. For a rotation of θ about one axis that is
`r = tan(θ/2)`.

`safety_violation()` compared that number against a constant named
`SAFE_MAX_TILT_RAD` **as though it were radians**. `1.0` in Gibbs units is
`tan(θ/2) = 1`, i.e. **exactly 90.0 degrees** — not the "~57 deg" its own comment
claimed. On the bench a hand rotation reached 87 degrees and **missed the trip
point by three degrees**, and nothing cut the motors. The same misreading was in
the arm gate: `ARM_MAX_TILT_RAD` `0.10` commented "~5.7 deg" was really
`2·atan(0.10) = 11.4 deg`.

**The estimator was CORRECT.** An earlier diagnosis in this session claimed a
35-degree estimator shortfall. **That diagnosis was WRONG. Do not repeat it.**
The disproof was already in the codebase: `read_sensor_frame()`'s flow tilt
compensation derives `cos(tilt)` from the Gibbs state as
`(1 - ga² - gb² + gc²)/(1 + ga² + gb² + gc²)`; put `ga = 0.949` through it and it
returns `0.052`, which is `cos(87.0°)`. Two independent routes, same answer.

Fixed in three parts, because one would not have been enough:
1. `tilt_rad_from_gibbs()` converts **once**, and every attitude limit now
   compares radians against radians. `SAFE_MAX_TILT_RAD` is now `0.70f` = 40 deg
   and genuinely 40 deg; `ARM_MAX_TILT_RAD` is `0.20f` = 11.5 deg, which is the
   angle the arm gate was already enforcing — only the name changed, so arming
   does not silently get harder.
2. `accel_envelope()` reads the accelerometer directly, with **no estimator in
   the path**, and adds **free fall** (`|a| < 3.92 m/s²`) and **impact**
   (`|a| > 29.43 m/s²`), neither of which existed anywhere before. Tilt
   structurally cannot catch a drop: a drone dropped flat stays flat all the way
   down.
3. Debounce is in **milliseconds** (`SAFE_DEBOUNCE_MS` 120, 40 for free fall and
   impact, plus a 2-sample floor). The old `SAFE_DEBOUNCE_ITERS` of 15 iterations
   was 20 ms under the PID cascade and **525 ms under TinyMPC** — a factor of 26
   between two builds of the same guard, and the run that failed was the slow one.

### Bug 2 — `ROSE_ACTUATE_TIMEOUT_MS` defaults to 0, i.e. DISABLED

The bench actuation cutoff is guarded by `#if ROSE_ACTUATE_TIMEOUT_MS > 0`, and
its **only** definition is `set(ROSE_ACTUATE_TIMEOUT_MS 0 CACHE STRING ...)` in
`CMakeLists.txt`. There is no `#ifndef` fallback in the source and no `--mode`
overrides it. With the default, the entire cutoff is **preprocessed out of the
image**. In every non-autoflight mode `g_armed` is a compile-time `true`, so the
board is armed from its first control tick and asks for thrust immediately.
**The default build had no time bound on motor output whatsoever.**

The commitment gate (section 7.3) is what now bounds it.

### Bug 3 — the autoflight anti-saturation never worked in the case it was written for

`actuator_duty()` clamped each motor to the ceiling **before** applying the
collective cut. Under gross saturation — which is every iteration of a bench run
that regulates to `TARGET_Z` from the floor — all four raw duties exceed the
ceiling, all four clamp to exactly the same number, and four identical values stay
identical through the cut, which then subtracts the same amount from four equal
values. **Measured on the bench before the fix:**

```
u = [1.082 0.977 1.429 1.541] -> duty = [0.100 0.100 0.100 0.100]   (1200 iterations)
```

The per-motor clamp is now gone and the collective cut runs on the unclamped
peak, so the **differential survives and attitude authority is preserved** —
sacrifice a little altitude, never attitude. In the unsaturated region the new
code is bit-identical to the old. See section 4.6 for the measured result.

### Bug 4 — `stop.gdb` was a no-op on the offload path

It zeroed the four SiFive PWM comparators. Under `--mode esp-motors`,
`CONFIG_PWM` is off, `pwm_sifive_init()` never runs, and those four addresses
belong to a block connected to nothing: **a no-op that looked like it worked**.
It now zeroes the comparators (still the only thing that helps the PWM path,
harmless here), latches `g_estop`, **resumes for 300 ms so the offload path
actually transmits an explicit zero-duty ESTOP frame**, and only then halts —
which is itself the second, independent stop via the ESP failsafe.

`estop.gdb`'s "halting the core is NOT a stop" warning is now correctly marked
**PWM-path-only**; on the offload path halting *is* a stop, just slower and less
explicit.

> Residual, unfixed: `stop.gdb` uses `shell sleep 0.3` and therefore assumes the
> GDB host has a POSIX `sleep`. Fine on a Mac.

### Bug 5 — TinyMPC returns an unconstrained command

`init()` tells the solver `u ∈ [-0.583, 0.417]`, but ADMM enforces that on the
**slack** only; the **primal** `u` is what gets returned. Measured on the host
with this exact problem data:

```
u = [1.260 1.284 1.247 1.218]   against a bound of 0.417   -- a 3.1x violation
```

and on the bench the returned command ranged up to ~1.6. TinyMPC is also baked
for a ~30 g Crazyflie at 50 Hz while this airframe is ~60 g, gets 10 ADMM
iterations, and runs at **~35 ms/iteration** against the PID cascade's **~1.3 ms**.
`--mode esp-motors` now selects PID explicitly. Do not let a mode that commands
motors fall through to the CMake defaults.

### Bug 6 — there are NO ESCs on this drone

Four brushed motors switched by **low-side SI2302 N-FETs**, gate active-high,
20 kHz PWM, 47R series, 10k pulldown. The FET gate *is* the motor control signal.

**Any diagnosis involving arming, calibration or locked-rotor latching describes
hardware this drone does not have.** The old "motor 4 stalled and its ESC latched
locked-rotor protection until power was pulled" story is **wrong**. That text is
still present in two places that were owned by other agents this session and
could not be edited here:

- `backends/zephyr-chipyard-sw/samples/rose_flight_controller/src/main.cpp`,
  in the `MOTOR_BREAKAWAY_DUTY` comment block
- `.../CMakeLists.txt`, in the comment above `set(MOTOR_BREAKAWAY_DUTY 0.15f ...)`

The **break-away number itself is good** (15 % starts all four, 10 % starts
three). Only the ESC explanation is wrong. Fix the comments in a follow-up.

---

## 7. Safety procedures

### 7.1 Assume the battery is live

A **~40 second powered motor run** happened during this session because the
session believed the motors were inert and they were not. Check the pack before
every step, not once at the start.

The converse also holds and is in the memory of this bench: **motor and flight
tests do nothing on USB-C alone.** A silent motor with no battery connected means
the battery, not a fault.

### 7.2 Every way to stop the motors, fastest first

| # | mechanism | how fast | works on `esp-motors`? |
|---|---|---|---|
| 1 | **Pull the battery** | instant | **yes — the only real kill** |
| 2 | Stop sending frames (halt the core, unplug the ribbon, crash) | **≤120 ms** + 5 ms tick | **yes — measured, `cuts=1`** |
| 3 | ESP duty ceiling, 25 %, independent of the FPGA | per frame | yes |
| 4 | Envelope guard → `g_estop` (tilt 40°, free fall, impact, rate, velocity, height, battery) | 40–120 ms dwell + one tick | **yes — measured** |
| 5 | Commitment window, 5 s uncommitted | one tick | **yes — measured, 5009 ms** |
| 6 | `hardware/flight/estop.gdb` | 1–2 s (JTAG attach) | yes |
| 7 | `hardware/flight/stop.gdb` | 1–2 s + 0.3 s + 120 ms | yes (see bug 4) |
| 8 | `ROSE_MOTORS_INHIBIT=1` | build time | yes |
| 9 | `ROSE_ACTUATE_TIMEOUT_MS` | one tick | yes — **but defaults to 0 = DISABLED** |

Both GDB scripts **resume the core and detach** rather than parking at a prompt,
deliberately. **Never Ctrl-C out of a GDB session with motors armed.**

```bash
./rb debug flight_controller --target arty200t \
    --config RocketArty200TDroneFullDDRConfig --mode esp-motors \
    --usb-location <jtag-location> --no-load --no-run \
    --gdb-script hardware/flight/stop.gdb
```

`--no-load` matters: without it `rb` reloads the ELF and restarts the program.
Find `<jtag-location>` with `./rb probe-ftdi`.

### 7.3 The commitment interlock

**Default, with no special flags: motors may run for 5 s from the first instant a
non-zero duty would be written, then all four latch OFF until the board is
reset** — whatever the controller asks for, whatever the arm gate says. A soft
reset does **not** clear the latch; the console says so when you try.

Committing takes **two independent acts, and neither alone does anything**:

1. **Build time** — `RB_CMAKE_ARGS="-DROSE_FLIGHT_COMMIT=1"`. No `--mode` sets
   it, so it has to be typed. Without it `g_flight_commit_key` **is not in the
   image**, `commit.gdb` fails with `No symbol "g_flight_commit_key"`, and **that
   failure is the interlock working**, not a broken script.
2. **Run time** — `hardware/flight/commit.gdb` writes the single 32-bit constant
   `0x464C5921` ("FLY!") from a tethered debugger, with you standing at the
   drone. It is polled, never latched at boot, so a power cycle returns the board
   to uncommitted. The gate **refuses** a commit once the window has already
   latched — reset and commit *before* the motors run.

Committed, the budget becomes **60 s**, not unbounded.

```bash
RB_CMAKE_ARGS="-DROSE_FLIGHT_COMMIT=1" ./rb build flight_controller \
    --target arty200t --config RocketArty200TDroneFullDDRConfig --mode esp-motors
```

Then, with the drone in front of you and ready:

```bash
./rb debug flight_controller --target arty200t \
    --config RocketArty200TDroneFullDDRConfig --mode esp-motors \
    --usb-location <jtag-location> --no-load --no-run \
    --gdb-script hardware/flight/commit.gdb
```

Wait for the console to say

```
MOTOR GATE: *** COMMITTED TO A FLIGHT TEST *** -- motor budget is now 60000 ms (was 5000 ms). Props are live.
```

**Props are live from that moment.** The ESP will show `flags=0x21[ARMED COMMITTED]`.

### 7.4 Reading the guard without touching the board

```bash
./rb debug flight_controller --target arty200t \
    --config RocketArty200TDroneFullDDRConfig --mode esp-motors \
    --usb-location <jtag-location> --no-load --no-run \
    --gdb-script hardware/flight/guard-check.gdb
```

It prints `g_guard_tilt_rad` (accelerometer, no estimator — ~0.785 at 45 deg,
~1.571 at 90 deg, and **3.14159 means `|a|` is outside the trust band**, i.e. the
frame is being shaken), `g_guard_est_rad` (the estimator's, converted out of
Rodrigues) and `g_guard_amag`. This answers the question that could not be
answered when the guard failed: **did it not trip, or is it not looking?**

The same three numbers are appended to the telemetry line as
`tilt=est<N>deg/acc<N>deg |a|=<x.xxx>`. **Watch those, not `roll`/`pitch`** —
`roll`/`pitch`/`yaw` in that line are still the raw Gibbs state, deliberately left
in place so existing parsers keep working.

### 7.5 Console

If a short workload prints nothing, it is usually **not** a UART fault: `rb debug`
runs the program before you attach. Use `--ram-console` (the console goes to
`ram_console_buf` in RAM, readable over JTAG) when you need output from a run that
finishes before you can attach. Note that a RAM-console build is a **different
image** with its own build directory.

---

## 8. What to do next, ranked

**1. Re-run the bench procedure in `docs/flight-safety-bench-procedure.md`, Part A
(battery DISCONNECTED).** It cannot spin a motor and it re-establishes on *your*
machine everything section 4 establishes here. A banner was added to the top of
that document pointing at section 4, because its own section 5 ("Not verified")
predates these runs; the *procedure* is sound and you should walk it anyway.

**2. Build `--mode esp-motors` as it now stands and read the boot banner.** The
PID+EKF+flow+0.060 kg combination has never been built and run (5.1). Confirm the
ENVELOPE GUARD and COMMIT GATE lines appear, confirm `nm` shows no
`g_flight_commit_key`, and confirm the ESP shows `link=UP` with `crcerr=0
noise=0`. Expect `PMW3901 not detected -- flow disabled`; that is 5.1's known
inert knob, not a new fault.

**3. Measure the thrust curve.** This is the single biggest hole (5.2). Use
`workloads/motor_duty` via `./tools/rb-ext.py`, restrained airframe, props on,
following `docs/flight-calibration-procedures.md` if it has landed. Produce: the
duty at which each motor first turns, and the duty at which the airframe's weight
is matched. Then set `MOTOR_MAX_DUTY` from a measurement instead of from the 0.30
calculation.

**4. Calibrate `FLOW_RAD_PER_COUNT`** with `tools/flow-cal.py` (5.3). Run its
`--self-test` first — it needs no hardware. Without this, a hover drifts.

**5. Fix `flow.c`'s hard-coded `CS_GPIO_PIN 19`** so it comes from devicetree.
One line, and it turns `ROSE_FLOW=1` from inert into real on this carrier.

**6. Correct the five stale comments** this handoff identified, in a follow-up
commit: the three "115200 is unverified" notes in
`hardware/zephyr/fpga-esp-uart.overlay`,
`hardware/zephyr/targets/esp32c6/workloads/esp_motors.overlay` and
`integration/zephyr/motor_link/include/riskybird/motor_link.h` (see 4.1), and the
two ESC locked-rotor passages in `main.cpp` and `CMakeLists.txt` (bug 6).

**7. Bring the boot/ready chirps back on the offload path.** They are deliberately
absent — reproducing them means holding the frame stream up during each
`k_msleep`, which is exactly what the receiver's failsafe exists to cut. The right
form is duty frames emitted from a loop, not sleeps. An autoflight offload mode
should not ship without them.

**8. Settle the camera question** (5.4) with the white-card test, or in 30 seconds
by reading the part number off the module. Then port
`docs/hm01b0-colour-investigation.md` and the repaired `frame-to-png.py` onto this
branch, since neither is here.

**9. Consider the ESP hardware watchdog.** `wdt0` is already `okay` on this board
and `CONFIG_WDT_ESP32` is default `y`, so `wdt_install_timeout(WDT_FLAG_RESET_SOC)`
fed from the motor thread would reset the SoC if it were ever starved — and a
reset drops the LEDC outputs, leaving the gates to the carrier's 10k pulldowns.
Deliberately **not** in this first version: it is not needed until WiFi is, and an
unproven self-reset on a motor bench is its own hazard. Note when adding it that
`wdt_esp32` programs both MWDT stages with the same value, so the interrupt fires
at T and the reset at 2T.

**10. Do not merge WiFi into `esp_motors` without reading the priority note** at
the top of `workloads/esp_motors/src/main.c`. The motor thread runs at
`K_PRIO_COOP(0)` = -16 and outranks every WiFi task and the system workqueue —
but **not** the networking traffic-class RX/TX threads, which `net_tc.c` also
creates at `K_PRIO_COOP(0)`. Equal cooperative priority means a long RX burst can
delay the failsafe. Merging needs `CONFIG_NET_TC_THREAD_PRIO_CUSTOM` with
`NET_TC_RX/TX_THREAD_BASE_PRIO` pushed down.

---

## 9. Background reading, in order

| document | why |
|---|---|
| `docs/flight-safety-bench-procedure.md` | the full ordered bench procedure; its section 5 is superseded by section 4 here |
| `docs/motor4-remote-investigation.md` | the original work order; section 2 lists everything already eliminated, so you do not re-run it |
| `integration/zephyr/motor_link/include/riskybird/motor_link.h` | the wire contract, with the reasoning for every field width |
| `workloads/esp_motors/src/main.c` | the receiver's whole safety model, stated case by case |
| `docs/motor-pwm-bringup.md` | the PWM block, the harness inversion, and why `obj/*.v` is the wrong file to read |
| `docs/setup.md` | steps 2d and 3c populate `RB_DTS_DIR` and `RB_BITSTREAM_DIR` |
| `docs/merging-into-main.md` | why `setup` is an orphan branch and what it deliberately omits |
