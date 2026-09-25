# Flight calibration procedures: the optical-flow scale and the thrust curve

Two unmeasured numbers stand between this airframe and a 30-second, 1-metre
hover with a controlled descent:

1. **`FLOW_RAD_PER_COUNT`** converts a PMW3901 motion count into an angle of
   ground-feature motion, and it has never been measured on this drone. The
   current value is the Crazyflie flow-deck constant carried across. Attitude
   survives a wrong value (the gyro compensation cancels the sign either way),
   but horizontal **velocity is then wrong-scaled and position drifts** — over
   30 seconds that is what ends the flight.
2. **Nobody has measured this airframe's thrust against commanded duty.** The
   hover duty is a guess, and the repository contains *three different guesses*
   that disagree by a factor of nearly five (§1.2).

This document is the procedure for measuring both, plus the two tools that
capture the data and the solver that turns it into constants.

> ### Nothing in this document has been run on hardware.
>
> It was written on a Linux host after the drone had already moved to a
> different bench. Every command here **builds**, and the build output quoted is
> real (§3, §7). Every console line quoted is a **format string read out of the
> source, not a capture**. Section 9 lists what that leaves unproven, in detail.
> Read it before you trust a number that comes out of this.

> ### No new bitstream is needed for anything in this document.
>
> Both images run on the existing `RocketArty200TDroneFullDDRConfig` shell.
> `motor_duty` needs only the second UART that shell already elaborates, and it
> deliberately enables no PWM driver at all. A bench that cannot run Vivado can
> do every step here.

---

## 0. The single easiest way to get confusing results

**The ESP32-C6 owns all four motors, and it cuts them if duty frames stop for
120 ms.**

The FPGA drives no motor gate. Ball F13 is dead as an output, so motor 4 could
never be driven from the FPGA, and rather than leave three gates dual-driven
(each gate is a wired-OR of an FPGA 47R and an ESP 47R into a 10k pulldown —
contention parks a contested MOSFET gate near half rail) **all four** moved to
the ESP. The FPGA now computes duty and sends it as 14-byte frames on uart1 at
50 Hz; `workloads/esp_motors` applies them, and cuts all four if no valid frame
arrives for `ESP_MOTORS_TIMEOUT_MS` = 120 ms.

A commander that sends slower than about 8 Hz therefore has its motors cut
**between its own frames**. From the bench that does not look like a failsafe.
It looks like a motor that stutters, or a thrust reading that sags halfway
through a dwell and cannot be reproduced. You will chase the motor, the FET, the
battery and the prop before you look at the frame rate.

`workloads/motor_duty` sends at 50 Hz (2.4× margin) and **asserts that at build
time** so it cannot be misconfigured silently. §3.2 shows the assertion firing.

---

## 1. What is being measured, and why

### 1.1 `FLOW_RAD_PER_COUNT`

The calibration is a **displacement** measurement, and the arithmetic is the
reason it is easy:

```
  flow.c    ang = -dx * K / dt          (per sensor sample)
  main.cpp  v   =  ang * h              (per control tick)
  =>        v   = -dx * K * h / dt
  =>        D   = ∫ v dt = K * h * Σ(dx)        for a constant height h
  =>        K   = D / (h * |Σ dx|)
```

**`dt` cancels.** The constant falls out of a slow hand slide across a bench —
no motion rig, no timing accuracy, no sensitivity to how fast the sensor is
polled — provided every count between the start and the end of the slide is
summed.

Summing is safe across poll rates because the PMW3901's motion burst returns the
deltas accumulated *since the last read* and clears them: a slower poll returns
**bigger deltas**, not fewer counts. The only way to lose counts is to saturate
int16 within one read interval, which cannot happen at hand-slide speeds — and
the capture tool reports the largest per-sample magnitude so that is checked
rather than assumed.

The per-sample small-angle model is exact enough by construction: at ~0.2 m/s,
10 ms and h = 0.15 m each sample subtends ~0.013 rad. The slide's *total* angle
is irrelevant, because the sensor re-acquires features continuously and the tool
integrates increments. That is why a 1 m slide at 0.15 m height is a legitimate
measurement and not a wild extrapolation.

Current value, in
`backends/zephyr-chipyard-sw/samples/rose_flight_controller/src/flow.c`:

```c
#ifndef FLOW_RAD_PER_COUNT
#define FLOW_RAD_PER_COUNT 0.00205f
#endif
```

0.00205 = `FLOW_RESOLUTION * thetapix / Npix` = `0.10 * 0.71674 / 35`, taken
from the Crazyflie flow deck because the sensor and the register init are the
same. It is a **plausible prior**, not a measurement of this lens at this
mounting.

### 1.2 The thrust curve, and three hover duties that cannot all be right

Searching the tree for what hover duty this airframe needs turns up three
numbers:

| number | where | what it actually is |
|---|---|---|
| **0.583** | `controller.hpp`, `controller_pid.cpp:405`, `controller_tinympc.cpp:54-55`, `main.cpp:1099` | The controller's **linearisation point**. `duty = (u + 0.583) * batt_scale`, so `u = 0` *means* 58.3 % duty, and TinyMPC's input box is built as `u ∈ [-0.583, 1-0.583]` around it. |
| **0.71** | `samples/rose_flight_controller/docs/FLIGHT_TUNING_LOG.md` | Observed **in flight** on the ~58 g v3 airframe: "thrust-marginal — ~71 % hover duty, raw command saturating 42–65 % of the flight". Those flights ran `AUTOFLIGHT_MAX_DUTY = 0.95`. |
| **0.15** | a comment on `main.cpp:636`, `AUTOFLIGHT_MAX_DUTY 0.45f /* ... (hover~0.15; margin above) */` | A comment with nothing behind it, numerically **identical to `MOTOR_BREAKAWAY_DUTY 0.15f`** on line 1051. A duty at which a motor only just starts turning cannot also hover a 58 g quadcopter. One of those two was copied from the other. |

This matters beyond bookkeeping:

- If hover is really ~0.65, then **`AUTOFLIGHT_MAX_DUTY = 0.45` is below hover**
  and an autoflight build cannot leave the ground: it saturates and climbs
  nothing, which looks like a controller failure and is not one.
- 0.583 is the point the whole controller is linearised about. If it is 10–20 %
  wrong, the feed-forward is wrong at every instant and the loop spends its
  authority correcting a constant bias.

Measuring the thrust curve settles all three at once.

### 1.3 Every ceiling between you and hover

| ceiling | value | where | effect |
|---|---|---|---|
| `ESP_MOTORS_MAX_DUTY_PCT` | **25 %** | `workloads/esp_motors/src/main.c:171` | Applied by the ESP, on the frames it receives. Independent of anything the FPGA does. |
| `MOTOR_DUTY_MAX` | **2500** (25.00 %) | `workloads/motor_duty/src/main.c` | The highest duty this commander will put on the wire. |
| `MOTOR_MAX_DUTY` | 0.10 | `main.cpp:1032` | Bench ceiling on the non-autoflight flight-controller path. |
| `AUTOFLIGHT_MAX_DUTY` | 0.45 | `main.cpp:636` | In-flight per-motor cap under `--mode autoflight`. |

**At the default ceilings the drone cannot hover, and that is deliberate.** A
25 %-capped sweep will not reach hover duty and you should not try to make it.
It is the safe rehearsal (§5.5); §5.8 raises both ceilings, deliberately and on
both sides, for the pass that does reach hover.

Raising one alone changes nothing at the gate. The ESP applies its own ceiling
to whatever arrives; the commander applies its own before sending. **Both must
move, in the same session, or the result is a sweep that silently flat-tops.**

---

## 2. What you need at the bench

**Common**

- The drone, its 1S LiPo, and the USB-C/JTAG tethers it normally runs on.
- A host with the Zephyr SDK and this repository. Vivado is *not* needed.
- Collected devicetrees and bitstreams (`RB_DTS_DIR`, `RB_BITSTREAM_DIR` — see
  `rb-env.sh`). In the `riskybird-dev-setup` worktree there is no elaborated
  DTS, so **prefix every build with `RB_DTS_DIR=$HOME/riskybird-dts`**.

**Flow calibration (§4)** — no battery, no propellers

- A textured, matte, **non-repeating** surface at least 1.2 m long (§4.2).
- Two hard stops clampable to the bench, 1.000 m apart, and a second pair at
  0.600 m for the validation batch.
- A spacer or clamp that holds the sensor at a **measured, constant** height:
  0.150 m for the fit, 0.250 m for validation. Measure to the flat face of the
  PMW3901 lens barrel, to ±1 mm.
- A steel rule or calipers. The height is the dominant error term (§4.8).

**Thrust curve (§5)** — propellers ON

- A scale: ≥ 500 g range, **0.01–0.1 g resolution**, tare-able.
- A rigid stand-off at least **3 propeller diameters** tall, and a way to bolt
  or zip-tie the airframe to it that will hold **well over 150 g of upward
  force** (thrust-to-weight of ~2.4 on a 60 g airframe).
- A multimeter for pack voltage (the controller scales duty by
  `BATT_NOMINAL_V / g_vbat`, so a sagging pack changes the mapping — §5.6).
- Eye protection, and a bench you are willing to have a propeller strike.

---

## 3. Part A — battery disconnected. Nothing here can spin a motor.

With the pack disconnected the motors **physically cannot turn**: the SI2302s
are low-side FETs switching the motor's ground, and the motor's positive rail
comes from the pack. No pack, no current, whatever the gate does. That is a
hardware guarantee, not a software one, and it is why every step that can be
done without the battery is done first.

### 3.1 Build all three images

```bash
cd /scratch2/loren/riskybird-dev-setup && source ./rb-env.sh
export RB_DTS_DIR=$HOME/riskybird-dts

# Flow capture (FPGA)
./rb build pmw3901_test --target arty200t --config RocketArty200TDroneFullDDRConfig

# Fixed-duty commander (FPGA). `rb fly motors` builds and drives it for you.
./rb build motor_duty --target arty200t \
    --config RocketArty200TDroneFullDDRConfig

# The motor driver (ESP32-C6) — the other end of the link
./rb build esp_motors --target esp32c6
```

Real output, tails only:

```
[7/7] Linking C executable zephyr/zephyr.elf
Memory region         Used Size  Region Size  %age Used
             RAM:       29200 B         1 GB      0.00%
.../build/zephyr/arty200t/pmw3901_test/default/zephyr/zephyr.elf

[132/132] Linking C executable zephyr/zephyr.elf
Memory region         Used Size  Region Size  %age Used
             RAM:       25104 B         1 GB      0.00%
.../build/zephyr/arty200t/motor_duty/default/zephyr/zephyr.elf

     sram0_0_seg:       54832 B     437512 B     12.53%
Successfully created ESP32-C6 image.
.../build/zephyr/esp32c6/esp_motors/default/zephyr/zephyr.elf
```

`motor_duty` is registered in `tools/rb/boards.py` like every other workload (it briefly lived in a
`tools/rb-ext.py` side registry, now removed), so plain `./rb` builds it.

### 3.2 Prove the frame-rate interlock (do this once; it takes 30 seconds)

This is the §0 hazard, checked rather than trusted. Ask for a frame rate that
would fail its own measurement:

```bash
RB_EXTRA_CFLAGS="-DMOTOR_DUTY_HZ=5" ./rb build motor_duty \
    --target arty200t --config RocketArty200TDroneFullDDRConfig
```

Real output:

```
.../zephyr/toolchain/gcc.h:87:36: error: static assertion failed:
  "motor_duty must send at >= 20 Hz: the ESP cuts all four motors after 120 ms
   without a valid frame (ESP_MOTORS_TIMEOUT_MS), so a slower commander fails
   its own measurement mid-dwell."
   87 | #define BUILD_ASSERT(EXPR, MSG...) _Static_assert((EXPR), "" MSG)
.../workloads/motor_duty/src/main.c:172:1: note: in expansion of macro 'BUILD_ASSERT'
FATAL ERROR: command exited with status 1
```

That failure proves two things at once: the frame-rate floor is enforced at
build time, **and** `RB_EXTRA_CFLAGS` definitions genuinely reach this source —
which is the same mechanism §5.8 uses to raise the duty ceiling. Rebuild without
the flag before going further.

### 3.3 Prove `motor_duty` is not the flight controller

`motor_duty` has **no commitment gate and no 5 s window** (§5.2 explains why
that is the right answer and not an omission). Check rather than believe:

```bash
NM=$(find backends/zephyr-chipyard-sw/tools-manual -name 'riscv64-zephyr-elf-nm' | head -1)
$NM build/zephyr/arty200t/motor_duty/default/zephyr/zephyr.elf \
  | grep -E "g_flight_commit_key|g_motor_latched|g_motor_committed"   # expect NOTHING
$NM build/zephyr/arty200t/motor_duty/default/zephyr/zephyr.elf \
  | grep -E " g_latched| g_armed| g_sweeping"                         # expect these
```

Real output — the first grep prints nothing and exits 1; the second prints:

```
0000000080004367 b g_armed
0000000080004318 b g_armed_ms
0000000080004366 b g_latched
0000000080004365 b g_sweeping
```

### 3.4 How a workload actually gets onto these shells

Read this once. Two things about `rb` catch everyone on this board, and both
produce symptoms that look like hardware faults.

**`rb run` does not work on the drone shells.** `RocketArty200TDroneDDRConfig`
and `RocketArty200TDroneFullDDRConfig` are elaborated with `uartTsi = false` and
their devicetree contains no TSI node at all, so `rb run` and `rb load` have
nothing to talk to. **Use `rb debug`**, which loads the ELF over the same JTAG
link the debugger attaches on. `--program` loads the bitstream first, so one
command covers programming and loading.

**The console is a different channel, and you lose the race by default.**
`rb debug` starts the program the moment it runs; the console is a separate
serial port opened by `rb console` in a different terminal. Anything printed
before you attach is gone — which is how a perfectly good UART gets blamed for a
workload that "printed nothing". `rb` warns about this in as many words when you
hit it.

Both of the workloads here are long-running and interactive, and both print
their bounds in a boot banner you need to read, so **always attach first**:

```bash
# terminal A -- attach BEFORE loading anything
./rb console --port <fpga-console-port>       # or set RB_CONSOLE_PORT

# terminal B -- program, load, and stop at the gdb prompt
./rb debug <workload> --target arty200t \
    --config RocketArty200TDroneFullDDRConfig --program \
    --program-usb-location <bitstream-ftdi> --usb-location <debug-ftdi> --no-run
(gdb) continue
```

`rb console` hands the port to `miniterm` and needs a real terminal. Leave the
GDB session attached: it is stop #4 in §5.1. `rb debug --ram-console` buffers
the console into RAM and reads it back over the same JTAG link, for a board
whose UART is unproven — it is not needed here, but it is the fallback if the
console misbehaves.

Find the two FTDI USB locations with `rb probe-ftdi --idcode`. Both are required
when more than one FTDI is attached, which on this bench is always.

### 3.5 Prove the solver, with no hardware at all

```bash
./tools/flow-cal.py --self-test
```

Real output:

```
fit      : 6 synthetic runs at D=1.000 m h=0.150 m
           K_true = 0.001980, K_solved = 0.001960 (-0.99%)
validate : 4 synthetic runs at D=0.600 m h=0.250 m, mean error -0.12%
SELF-TEST: PASS
This exercises the solver only. It says nothing about the sensor, the surface or the airframe.
```

It synthesises counts from a known constant, recovers it, and validates it
against a second synthetic batch at a different height and distance. It
exercises the parser and the arithmetic and **nothing about the sensor**.

---

## 4. Part B — optical-flow calibration. Still no battery, still no props.

### 4.1 Why the battery stays out

Nothing in this part drives a motor, a PWM channel or any output but the console
and the flow sensor's chip select. `workloads/pmw3901_test` includes no PWM
header and enables no PWM driver. Leave the pack disconnected and the whole
section is a measurement that **cannot** spin anything.

### 4.2 The surface

The PMW3901 tracks features. What you slide it over decides whether the counts
mean anything.

**Good** — matte, high contrast, fine grained, **aperiodic**:

- newsprint or a densely printed page;
- 100–150 grit sandpaper sheet;
- a random dot field printed at 600 dpi (dots ~0.5 mm, ~30 % coverage);
- bare concrete, or a close-woven patterned carpet.

**Bad, and each fails differently**:

- plain white paper, a clean desk, anything uniform — SQUAL collapses, the
  flight build discards the samples, and the tool rejects the run;
- anything glossy — specular highlights move with the *sensor*, not with the
  surface, so the counts are a measurement of the lamp;
- **anything periodic** — graph paper, tile grout, a regular weave. A repeating
  pattern aliases: the sensor locks onto the wrong period and returns confident,
  consistent, wrong counts. This is the only failure mode on the list that does
  *not* announce itself in SQUAL, which is why "aperiodic" is stated separately
  from "textured".

**Lighting.** The driver configures the sensor's illumination LED
`GPIO_OUTPUT_INACTIVE` — it is **off** — so the PMW3901 is running on ambient
light alone. Use ordinary, even room lighting; no hard shadow should cross the
field during a slide, and nothing should change the light between the fit batch
and the validation batch. The shutter value is your proxy for this and the tool
rejects a run whose shutter swings more than 4× (§4.9).

### 4.3 The geometry — two methods, and they are not interchangeable

**Method A — clamped drone, moving surface (recommended for the magnitude).**
Clamp the airframe so the flow sensor looks straight down, level, at the
measured height. Tape the textured sheet to a rigid board and slide *the board*
underneath by the measured distance, against a straightedge, between two stops.

- `h` is set once by the clamp and cannot wander.
- No cable is strained and the drone cannot tilt.
- Flow is relative motion, so the magnitude is identical either way.
- **The sign is inverted** relative to flight. Never take the axis sign from a
  Method A run (§4.6).

**Method B — moving drone, fixed surface (required for the sign check).** Lay
two parallel spacer strips of equal, measured thickness on either side of the
textured strip, rest the airframe on them, and slide it along a straightedge
between two stops.

- Sign matches flight, because the drone really moves.
- `h` is only as constant as the strips are equal; the shutter check catches
  gross violations, small ones it does not.

Either way: **one axis at a time**, along a straightedge, between hard stops.
Stops rather than pencil marks — a stop is repeatable, a pencil line is a
judgement call made ten times.

### 4.4 Aim and judge the surface before capturing anything

Program, load and run — and see §3.4 first if you have not loaded a workload on
these shells before, because **`rb run` does not work here**:

```bash
# terminal A, FIRST
./rb console --port <fpga-console-port>

# terminal B
./rb debug pmw3901_test --target arty200t \
    --config RocketArty200TDroneFullDDRConfig --program \
    --program-usb-location <bitstream-ftdi> --usb-location <debug-ftdi> --no-run
(gdb) continue
```

The banner, then one line a second:

```
=====================================================
RiskyBird PMW3901 diagnostic + FLOW_RAD_PER_COUNT capture
  built : <date> <time>
=====================================================

PMW3901 chip ID: 0x49, inverted: 0xB6
PMW3901 initialised.

Keys: r/SPACE=start capture  s/ENTER=stop+report  z=zero  v=per-sample lines  ?=this
Config: read period 10 ms, SQUAL floor 19 (matches flow.c), per-sample lines off

IDLE dx=   +0 dy=   +0 SQUAL= 57 shutter= 1816 | since zero: sumdx=0 sumdy=0 over 100 samples
```

**The chip-ID line is a gate, not decoration.** It must read `0x49` **and**
`0xB6`. The driver used to accept either, which passes a sensor whose reads
cannot be trusted and reports it healthy; for optical flow that is a silently
wrong velocity rather than an absent one. It now requires both, and discards one
read before the pair because on this bench the first register read after the CS
dance returns `0x92` — `0x49` with its bits in the other order — while the very
next read is correct.

**Pass criteria before you slide anything**, with the sensor at the working
height over the working surface:

- `SQUAL` **≥ 40** steady, and never below the flight floor of 19. If the tool
  prints `SQUAL ... is BELOW the flight floor`, fix the surface, not the run.
- `shutter` steady within a few percent as you move the sensor around over the
  surface. A shutter that jumps means the lighting is uneven.
- `dx`/`dy` ≈ 0 when nothing moves. Nonzero counts at rest are vibration or
  aliasing.

### 4.5 The fit batch

Height **0.150 m**, distance **1.000 m**. For each run:

1. Sensor against the start stop, settled, everything still.
2. `r` — the console prints `FLOWCAL START id=N -- move NOW; press s to stop`.
3. Slide smoothly to the far stop. **Aim for ~5 seconds** over 1 m (~0.2 m/s).
   Do not flick; the tool rejects anything under 500 ms as "a flick, not a
   controlled slide".
4. `s` — the summary prints as one greppable `FLOWCAL RUN` line.
5. **Alternate the direction** run to run. The fit uses `|Σ|` so direction does
   not matter to the answer, and alternating cancels any systematic bias from a
   bench that is not perfectly flat.

**Capture at least 6 usable runs.** The solver warns below 5, on the grounds
that the median of four numbers is not a median.

Leave `v` (per-sample lines) OFF during a run. `printk` busy-waits on this
carrier — the FPGA console is polled, `CONFIG_UART_INTERRUPT_DRIVEN` is off — so
a line per sample at 100 Hz steals most of the read loop.

Save the console log to a file.

### 4.6 The sign check — separate, deliberate, Method B only

The fit measures a magnitude. The sign is a different claim and needs its own
test, because Method A inverts it.

With the drone **moving** (Method B), slide it along body **+x** (nose forward)
and read the reported `sumdx`. Record which sign of `sumdx` corresponds to
nose-forward motion, and check it against `flow.c`:

```c
float ax = -(float)m.deltaX * FLOW_RAD_PER_COUNT / dt;   /* +x = -dX */
float ay =  (float)m.deltaY * FLOW_RAD_PER_COUNT / dt;   /* +y = +dY */
```

and against the airframe convention recorded in `FLIGHT_TUNING_LOG.md`:
**`+vy` / `+roll` = LEFT**. If nose-forward produces a *positive* `sumdx`, the
remap in `flow.c` is backwards for this mounting and no amount of getting `K`
right will help.

### 4.7 Solve

```bash
./tools/flow-cal.py --distance 1.000 --height 0.150 --axis x capture.log
```

Worked example. The log below is illustrative — real `FLOWCAL RUN` lines in
exactly this format — and the output is this tool's actual output on it:

```
FLOWCAL RUN id=1 ms=5230 n=523 err=0 sumdx=-3367 sumdy=41 absdx=3367 absdy=119 peakdx=17 peakdy=3 n_ok=523 okdx=-3367 okdy=41 sq_min=51 sq_max=62 sq_mean=57 sh_min=1702 sh_max=1930 sh_mean=1816
FLOWCAL RUN id=3 ms=430 n=43 err=0 sumdx=-3390 sumdy=22 ... n_ok=43 okdx=-3390 okdy=22 ...
FLOWCAL RUN id=5 ms=5120 n=512 err=0 sumdx=-2914 sumdy=61 ... n_ok=447 okdx=-2551 okdy=48 sq_min=11 ...
FLOWCAL RUN id=7 ms=5480 n=548 err=0 sumdx=-3299 sumdy=-704 ... n_ok=548 okdx=-3299 okdy=-704 ...
FLOWCAL RUN id=8 ms=5050 n=498 err=7 sumdx=-3120 sumdy=17 ...
  (plus ids 2, 4, 6, 9, 10 — clean runs)
```

```
10 run(s) parsed, 6 usable, 4 rejected
  REJECT id=3: the run lasted 430 ms (need 500): that was a flick, not a controlled slide
  REJECT id=5: 65 of 512 samples below the SQUAL floor: flight would have discarded them
  REJECT id=7: cross-axis leak 704/3299 = 21%: the slide was not straight
  REJECT id=8: 7 read error(s): counts were lost

  id    counts  SQUAL  shutter   K (rad/count)
   1     -3367     57     1816        0.001980
   2     -3402     56     1822        0.001960
   4     -3331     56     1805        0.002001
   6     -3388     57     1811        0.001968
   9     -3350     57     1809        0.001990
  10     -3372     57     1812        0.001977

median  K = 0.001979 rad/count   <-- use this
mean    K = 0.001979
spread    = 0.000014 (0.7% of the median)
count sign on the x axis: negative

Set it with, e.g.:
  RB_EXTRA_CPPFLAGS="-DFLOW_RAD_PER_COUNT=0.001979f"
or change the #ifndef default in
  backends/zephyr-chipyard-sw/samples/rose_flight_controller/src/flow.c
```

The median, not the mean: a mean and a standard deviation are both dragged by
the very outlier they are meant to find. Above four surviving runs the tool also
flags runs more than 3 robust sigma (MAD-scaled) from the median and drops them
from the fit, naming each one.

If the spread exceeds 10 % of the median the tool says so. **That is the slide
or the surface, not the sensor.** Redo the batch.

### 4.8 The error budget — measure the height carefully

`K = D / (h · |Σdx|)`, so the fractional error in `K` is the sum of the
fractional errors in `D` and `h`:

| term | typical | fractional error |
|---|---|---|
| `h` = 0.150 m, measured to ±1 mm | | **0.67 %** ← dominant |
| `D` = 1.000 m, between hard stops, ±2 mm | | 0.20 % |
| `Σdx` ≈ 3300 counts, ±1 count of quantisation | | 0.03 % |

Everything else is noise you beat down with runs. The height is the one that
does not average away, because it is the same systematic error in every run of
the batch. **Measure to the flat face of the lens barrel, with a rule, twice.**

A second, subtler height error: in flight, `h` comes from the down-ToF, which
measures from *its own* face. Any fixed offset between the ToF reference plane
and the flow lens plane is a constant scale error on every flow-derived
velocity. If the two are more than a few millimetres apart on this airframe,
that offset belongs in the flow path and is not currently anywhere in it.

### 4.9 Every rejection rule, and what each one is really saying

All of these are implemented in `tools/flow-cal.py` and all are overridable.
None of them has been checked against real captures from this airframe.

| rule | default | what it means |
|---|---|---|
| read errors | any | counts were lost; the sum is not the displacement |
| samples below the SQUAL floor | any | flight would have discarded them, so this run is not what flight would have seen |
| run duration | < 500 ms | a flick, not a controlled slide |
| counts on the measured axis | < 200 | one count of quantisation starts to matter |
| cross-axis leak | > 15 % | the slide was not straight, so `D` was not the distance travelled |
| mean SQUAL | < 25 | too close to the flight floor of 19 to be a comfortable margin |
| SQUAL spread | > 60 | the surface changed mid-slide |
| shutter ratio `max/min` | > 4× | the **lighting or the height** changed, so `h` was not the constant the solve assumes |
| peak per-sample count | ≥ 8000 | approaching int16 saturation; slide slower or poll faster |
| K vs median | > 3 robust σ | a stumble (fit only — never applied to a validation) |

### 4.10 Validate against data the fit never saw

This is the step that makes the number believable, and it must use a **separate
batch at a different height and a different distance** — otherwise it only
proves the arithmetic is self-consistent, which §3.5 already proved.

Move the stops to **0.600 m**, raise the sensor to **0.250 m**, capture at least
4 runs, then:

```bash
./tools/flow-cal.py --distance 0.600 --height 0.250 --axis x \
    --validate 0.001979 validation.log
```

Real output on the illustrative validation log:

```
5 run(s) parsed, 5 usable, 0 rejected

Validating K = 0.001979 rad/count against D = 0.6 m at h = 0.25 m
(every usable run counts; nothing is dropped for disagreeing)

  id    counts  SQUAL   predicted D     error
   1     -1213     50      0.6001 m     0.0%
   2     -1198     49      0.5927 m    -1.2%
   3     -1230     51      0.6085 m     1.4%
   4     -1205     50      0.5962 m    -0.6%
   5     -1221     50      0.6041 m     0.7%

mean error +0.1%, spread 0.9%, worst 1.4% over 5 run(s)
tolerance: |mean| and spread both <= 10%
VERDICT: PASS
```

Exit status is 0 on PASS and 1 on FAIL, so it can gate a script.

**Validation deliberately keeps every physically-valid run, including
statistical outliers.** A fit is allowed to choose its data; a validation is not.
Dropping the runs that disagree is exactly how a constant passes a test it
should have failed.

**Why 10 %.** If the flow velocity scale is 10 % low, the position loop nulls
the *measured* velocity and leaves 10 % of the true drift uncorrected. At a
realistic residual drift of 0.2 m/s that is 0.02 m/s of uncommanded translation,
or **0.6 m over a 30 s hover** — about the tether radius. This is an argument
from the control structure, not a measurement, and it is the number to revisit
once a hover has actually been flown.

Changing the height between the fit and the validation is the important part:
`K` and `h` enter the model as a product, so a batch at a single height cannot
separate an error in `K` from an error in the height measurement. Two heights
can.

### 4.11 Put the constant in the build

```bash
RB_EXTRA_CPPFLAGS="-DCS_GPIO_PIN=0 -DFLOW_RAD_PER_COUNT=0.001979f" \
RB_CMAKE_ARGS="-DROSE_FLOW=1" \
./rb build flight_controller --target arty200t \
    --config RocketArty200TDroneFullDDRConfig --mode esp-motors
```

`RB_EXTRA_CPPFLAGS`, not `RB_EXTRA_CFLAGS`: `EXTRA_CFLAGS` reaches C only, and
`main.cpp` is C++. `flow.c` is C and would take either, but using one form for
both avoids a build where half the flags landed.

`CS_GPIO_PIN=0` matters and used to be silently ignored. `flow.c` hardcoded
`#define CS_GPIO_PIN 19` (the ESP32-C6's pin) with no `#ifndef`, so the
command-line definition documented in `flight-on-fpga.md` was *redefined* by the
source — a warning, not an error — and the FPGA flow build drove `gpio0` pin 19
as chip select while the sensor sat on logical index 0. The sensor never
answered and it looked like "no flow". It is `#ifndef`-guarded now, so the
documented override works.

Once you are satisfied, change the `#ifndef` default in `flow.c` rather than
carrying the flag on every command line for the rest of the project's life.

---

## 5. Part C — the thrust curve. **PROPELLERS ON.**

Everything up to here could not spin a motor. This can, and it is the only test
in this document that *wants* propellers fitted, because a thrust curve measured
without them is a measurement of nothing.

### 5.1 Safety — read this whole section before fitting a propeller

**Restraint.** The airframe is bolted or zip-tied to a stand-off, and the
stand-off is clamped to the bench or sitting on the scale as one rigid assembly.
The restraint must hold **well over 150 g of upward force**: a 60 g airframe at
a thrust-to-weight of ~2.4 pulls ~145 g at full duty, and the point of §5.8 is
to go near there. Tape is not a restraint.

**Stand out of the prop plane.** Not beside it, not "mostly" out of it — a
brushed micro prop that lets go leaves in the plane of rotation. Stand above or
below it, or put the assembly behind something. Eye protection regardless.

**Bounded run time, enforced by the image, not by you.** `motor_duty` ends every
step by itself: nothing has to be pressed to stop it, something has to be
pressed to keep it going. Four independent bounds:

| bound | default | what it stops |
|---|---|---|
| image ceiling `MOTOR_DUTY_MAX` | 25.00 % | the highest duty that can reach the wire |
| dwell `MOTOR_DUTY_DWELL_MS` | 6000 ms | an armed step ending by itself |
| sub-break-away cap `MOTOR_DUTY_STALL_MS` | 2000 ms | a stalled motor cooking (§5.5) |
| per-boot budget `MOTOR_DUTY_BUDGET_MS` | 120000 ms | a session drifting into an hour of spinning motors; it **latches**, and only a board reset clears it |

And the ESP applies its **own** 25 % ceiling to whatever arrives, independently.

**Known-good stops, in order. Learn them before you arm anything.**

| # | stop | how it works | how fast |
|---|---|---|---|
| 1 | **Pull the battery** | no pack, no current through the low-side FETs, whatever the gates do | instant — the only real kill |
| 2 | **Press `x`, `q` or SPACE** on the FPGA console | `motor_duty` disarms and sends three explicit zero-duty frames | one frame time (~20 ms) |
| 3 | **Stop sending frames** — unplug the uart1 ribbon, or quit the debugger | the ESP sees no valid frame and cuts all four | 120 ms + one 5 ms tick |
| 4 | **`monitor halt`** at the GDB prompt you already have open | the core stops transmitting, so this is stop #3 by another route | immediate + 125 ms |
| 5 | `hardware/flight/stop.gdb`, if no session is attached | same, but it must attach OpenOCD first | JTAG attach (1–2 s) + 125 ms |
| 6 | Let the dwell expire | the step ends itself | ≤ 6 s |

**Keep the GDB session from §3.4 attached.** Not for debugging — because
`monitor halt` typed at that prompt is stop #4, and it is immediate. A *second*
`rb debug` cannot attach while the first one holds the FTDI, so if you close the
session the fast stop closes with it and you are back to a 1–2 s re-attach.

Stops 3, 4 and 5 are the same mechanism. **On the ESP-offload path, halting the
core IS a stop** — which is *not* true on the old FPGA PWM path, where the
SiFive comparators are separate hardware that keep driving the pins at whatever
duty they hold. `hardware/flight/estop.gdb` carries that warning and it is
marked PWM-path-only for exactly this reason.

**Do not press `?` (reprint the key list) while armed.** It is ~420 characters
on a polled 115200 console, ~36 ms of busy-wait, and it delays the next frame by
that much. 36 ms is comfortably inside the 120 ms failsafe, but there is no
reason to spend a third of the margin on a help screen mid-measurement.

**Watch the `HOLD` line.** While armed, the image prints once a second:

```
MOTORDUTY HOLD duty=15.00% t=1000/6000 ms frames=50 (expect ~50/s)
```

`frames` counts frames sent *since this step armed*. If it is not climbing by
about 50 per second, the link is not being fed and the ESP is about to cut — and
the number on the scale is not the number you think it is.

### 5.2 The 5-second window: what it is, and when it applies to you

The flight controller latches **all four motors off 5 s after they first spin**
(`ROSE_MOTOR_WINDOW_MS`), whatever the controller is asking for, and will not
un-latch until the board is reset. Committing widens that to 60 s
(`ROSE_COMMIT_MAX_MS`) and takes two independent acts, **neither of which alone
does anything**:

1. build time — `RB_CMAKE_ARGS="-DROSE_FLIGHT_COMMIT=1"`, which no `--mode`
   sets. Without it `g_flight_commit_key` is not in the image at all and
   `commit.gdb` fails with "No symbol". That failure *is* the interlock.
2. run time — `hardware/flight/commit.gdb`, which writes one specific 32-bit
   constant from a tethered debugger. It is polled, never latched at boot, so a
   power cycle returns the board to uncommitted.

**A thrust sweep would hit that window immediately.** A single step is 6 s. Even
committed, 60 s of motor-live time does not cover a twelve-step sweep. So:

> **Do not measure the thrust curve through `flight_controller`.** That is why
> `workloads/motor_duty` exists. It is a plain fixed-duty commander with no
> control loop, no arming gate, no commitment gate and no 5 s window — §3.3
> proves the symbols are absent — and its bounds are the dwell, the stall cap
> and the per-boot budget instead. Nothing has to be committed to run a step, so
> nothing forgets to un-commit afterwards.

The image says so in its own banner, because expecting a bound that does not
exist is worse than knowing there is none:

```
NOTE: this is NOT the flight controller. There is no commitment gate and
      no 5 s window in this image -- its bounds are the dwell (6000 ms), the
      sub-break-away cap (2000 ms) and the 120000 ms per-boot budget, all above.
      Nothing has to be committed to run a step, so nothing forgets to
      un-commit afterwards.
```

The window **does** apply to §6, the hover itself: 5 s < 30 s, so the hover
cannot happen in an uncommitted build. That is the one place where committing is
the right answer rather than a workaround.

### 5.3 The scale method

**Mount the drone inverted on a stand-off, the whole assembly on the scale.**

```
        propwash exhausts UP into free air
              ↑    ↑    ↑    ↑
        [ drone, inverted, props pointing up ]
                   |
                   |  stand-off, >= 3 prop diameters
                   |
        =======[ scale platform ]=======
```

Thrust pushes the assembly **down** into the scale, so the reading *increases*
by the thrust. The downwash exhausts upward into free air instead of splashing
off the scale platform, which is what makes an upright "weight-loss" arrangement
unreliable at this scale: the recirculation is a large fraction of the thrust
you are trying to measure.

- Keep the props at least **3 prop diameters** from the platform *and* from any
  wall, bench back or box. Measure your prop diameter; do not assume.
- Cables are part of the rig. Dress the USB, JTAG and battery leads so that **no
  cable crosses between the assembly and the bench under tension**, and anchor
  them so nothing moves when the props start. Re-tare after dressing, not
  before.
- Tare with the battery connected, everything mounted, motors off. **The reading
  during a step is then the thrust in grams, directly.**
- Weigh the flying airframe separately on the same scale — everything that
  leaves the ground, battery included. Call it `m`.

### 5.4 Run the commander

Flash the ESP driver and the FPGA commander:

```bash
./rb flash esp_motors --target esp32c6 --port <esp-port>

# terminal A -- the FPGA console. Attach BEFORE loading (§3.4).
./rb console --port <fpga-console-port>

# terminal B -- program, load, then `continue` at the gdb prompt
./rb debug motor_duty --target arty200t \
    --config RocketArty200TDroneFullDDRConfig --program \
    --program-usb-location <bitstream-ftdi> --usb-location <debug-ftdi> --no-run
(gdb) continue
```

Keep **three terminals open**: the FPGA console (terminal A — this is where you
type the keys and read the `HOLD` lines), the GDB session (terminal B — **leave it
attached**: `monitor halt` there is stop #4 and it is immediate), and the ESP
console. The ESP prints, periodically:

```
MOTOR CMD duty per mille: M1=150 M2=0 M3=0 M4=0
  link=UP seq=203 flags=0x01[ARMED] frames=1015 gap=20ms lost=0 stale=0 crcerr=0 noise=0 cuts=0 pwmerr=0
```

Before fitting a propeller, **prove the whole chain with the battery
disconnected**: arm a step, and check that `link=UP`, `frames` climbs, `cuts=0`,
`crcerr=0`, `noise=0`, and `M1..M4` show the duty you asked for. Then unplug the
uart1 ribbon and confirm you get `cuts=1` and `link=DOWN -- MOTORS OFF
(failsafe)` within ~125 ms. That rehearses stop #3 while nothing can move.

Console keys:

```
  1 2 3 4   toggle that motor in the selection
  a / n     select all four / none
  + -       duty +/- 1.00 %          ] [   duty +/- 5.00 %
  g         GO: arm the selection at the current duty for one dwell
  x q SPC   STOP now (also aborts a sweep)
  t         run the scripted step sweep (g = next step early, x = abort)
  ?         reprint this        [not while armed -- see 5.1]
```

Duties are entered and shown in hundredths of a percent, the wire's own unit, so
nothing is rounded between the key press and the frame.

### 5.5 Pass 1 — break-away, at the default 25 % ceiling

**Break-away is the lowest duty at which a motor reliably starts from rest.**
The recorded 0.15 in `main.cpp` was measured **through the FPGA's own PWM
comparators**, a path that no longer drives the gates at all. It may not carry
over, for three concrete reasons:

1. **Different gate drive.** FPGA bank 16 at its VCCO through a 47R, versus the
   ESP's 3.3 V through its own 47R, gives a different V_GS on the SI2302, a
   different R_DS(on), and therefore a different average motor voltage at the
   same nominal duty.
2. **The FPGA path's outputs are inverted in the generated Verilog**
   (`assign motorN = ~_chiptop0_pwm_*_gpio_*`, 4/4 in every motor-bearing
   config — see `docs/motor-pwm-bringup.md`). Nobody re-derived whether the duty
   the comparator was *asked* for is the duty the gate *saw*. The ESP path has
   no such inversion: `esp_motors` drives LEDC active-high straight at the gate,
   so duty is unambiguously the gate-on fraction.
3. **Different quantisation.** The SiFive comparator resolves 1/2500 = 0.040 %
   at 20 kHz; the ESP's LEDC lands on an 11-bit timer at 20 kHz, 2048 steps =
   0.049 %. Both are far finer than the wire's 0.01 %, so this one is small — but
   it is not zero.

> **A note on the old locked-rotor story.** `main.cpp`'s break-away comment says
> a stalled motor makes "its ESC latch locked-rotor protection", surviving an ELF
> reload and a full FPGA reconfigure. **There are no ESCs on this airframe.** The
> motors are brushed, driven by low-side SI2302 FETs, and there is nothing in
> that path to latch. That comment is stale and should be corrected; it is in a
> file this procedure does not own, so it is flagged here rather than edited.
>
> The *danger* is real and is simply different: a stalled brushed motor on a
> low-side FET is a resistor across the pack. It heats fast, and it is why the
> sub-break-away cap exists.

**Procedure**, one motor at a time, propellers **on** (break-away with props off
is not the break-away that matters — the prop is most of the load):

```bash
RB_EXTRA_CFLAGS="-DMOTOR_DUTY_SWEEP_START=300 -DMOTOR_DUTY_SWEEP_STEP=50 \
                 -DMOTOR_DUTY_SWEEP_STOP=1600" \
  ./rb build motor_duty --target arty200t \
      --config RocketArty200TDroneFullDDRConfig
```

3.00 % to 16.00 % in 0.50 % steps. Every step below the 15.00 % break-away
threshold is capped at 2 s by the image regardless of the dwell, so the whole
sweep is ~68 s of armed time against the 120 s budget.

1. Select one motor (`n`, then `1`).
2. `t` to start the sweep; watch the motor, not the console.
3. Record the lowest duty at which it starts **from rest, 5 times out of 5**.
   Reset the board between attempts if the budget latches.
4. Also record the lowest duty at which an *already spinning* motor keeps
   spinning. It will be lower — brushed motors have real stiction hysteresis —
   and both numbers are useful: the first bounds the controller's minimum
   command, the second bounds how far a running motor may be throttled back
   during a descent.
5. Repeat for motors 2, 3, 4. **The break-away for the airframe is the highest
   of the four, plus margin.**

**Between attempts, touch the motor cans.** If one is too hot to hold
comfortably, stop and let it cool. A sub-break-away step is the most damaging
thing in this document.

### 5.6 Pass 1 — the low end of the thrust curve, still at 25 %

```
t          # the default sweep: 5.00 % -> 25.00 % in 2.50 % steps
```

Read the scale **at the end of each dwell**, when the reading has settled, and
record three things per step: **commanded duty, steady thrust in grams, pack
voltage.**

The pack voltage is not optional. The flight controller multiplies duty by
`BATT_NOMINAL_V / g_vbat` (nominal 3.8 V, clamped to at most 1.30×) precisely
because thrust at a given duty falls as the pack drains. A curve taken across a
0.3 V sag is a curve of two variables. **Keep the whole batch inside ~0.1 V, and
throw the batch away if it is not.** `hardware/flight/battery.gdb` reads the
board's own `g_vbat` on a flight-controller build; for `motor_duty` use a meter
on the pack.

This pass will **not** reach hover. At the default 25 % ceiling four motors
produce a small fraction of the ~60 g needed (§1.3). That is expected. Pass 1 is
where you prove the rig, the scale, the failsafe and the thermal behaviour while
the airframe cannot lift itself.

### 5.7 Thermal drift, and the check that catches it

Brushed motors and their FETs heat, and a hot motor makes less thrust at the
same duty. A sweep that only ever goes up cannot tell a real thrust curve from
that drift — both look like "the top of the curve sags".

Five defences, in the order they matter:

1. **The drift check is built into the sweep.** After the last step, the image
   repeats the **first** step and says so:

   ```
   MOTORDUTY DRIFT CHECK -- repeating the FIRST step last. Its thrust must match
   the first reading, or the curve is thermal.
   ```

   If the repeated reading matches the original within the scale's resolution,
   the batch was taken cold enough to trust. If it does not, the curve is
   thermal, not aerodynamic, and the batch must be redone with longer
   cool-downs. **Record both readings every time.**
2. **Cool-down longer than the dwell.** The default is 9 s of cooling per 6 s
   step (`MOTOR_DUTY_SWEEP_COOL_MS`). Raise it before you shorten the dwell.
3. **One motor at a time** for per-motor curves — the other three are at zero
   and cooling.
4. **Never leave a sub-break-away step running.** That is what the 2 s stall cap
   is for; do not raise it.
5. **A descending sweep on a cold airframe**, as an independent check. Build
   with `SWEEP_START` and `SWEEP_STOP` swapped is not supported (the sweep only
   ascends), so do it by hand: cool fully, then run the steps top-down with `g`
   at each duty. If the ascending and descending curves differ by more than the
   scale's resolution, everything above is thermal.

Split a long batch across two boots with a full cool between. The per-boot
budget latches at 120 s of armed time and tells you so:

```
MOTORDUTY BUDGET EXHAUSTED -- latched off. Reset the board to run more steps.
```

### 5.8 Pass 2 — raise BOTH ceilings and measure to hover

**This is the dangerous pass.** The airframe will now make more thrust than it
weighs and will genuinely try to fly. Re-check the restraint, re-check that you
are out of the prop plane, and have a hand near the battery.

Both ceilings must move, in the same session:

```bash
# FPGA commander: ceiling 80.00 %, sweep 25 % -> 80 % in 5 % steps
RB_EXTRA_CFLAGS="-DMOTOR_DUTY_MAX=8000 -DMOTOR_DUTY_SWEEP_START=2500 \
                 -DMOTOR_DUTY_SWEEP_STEP=500 -DMOTOR_DUTY_SWEEP_STOP=8000" \
  ./rb build motor_duty --target arty200t \
      --config RocketArty200TDroneFullDDRConfig

# ESP driver: its OWN, independent ceiling
RB_EXTRA_CFLAGS="-DESP_MOTORS_MAX_DUTY_PCT=80" \
  ./rb build esp_motors --target esp32c6
```

Twelve steps of 6 s plus the drift check is 78 s of armed time, inside the 120 s
budget — but it is also 78 s of a micro brushed motor at high duty. **Split it:**
25–50 % in one boot, cool the airframe fully, 55–80 % in the next.

**Read the banner, not the path.** Both builds land on the same ELF path
(`build/zephyr/arty200t/motor_duty/default/zephyr/zephyr.elf`) and nothing in
the filename records the ceiling. The image prints its own bounds at boot and
the ESP prints its own:

```
Bounds: image ceiling 80.00%, dwell 6000 ms, sub-break-away cap 2000 ms,
        budget 120000 ms/boot. The ESP applies its OWN ESP_MOTORS_MAX_DUTY_PCT
        ceiling and cuts all four if frames stop for 120 ms.

esp_motors: failsafe 120 ms, tick 5 ms, duty ceiling 80%
```

**Check both lines say what you think before you arm anything.** If the ESP
console still says 25 % you are about to measure a flat top and blame the
motors.

Record the same three columns as pass 1. Stop as soon as total thrust clearly
exceeds the airframe mass — you do not need the top of the curve, you need the
crossing.

### 5.9 Deriving hover duty

Save the pass-1 and pass-2 readings as a whitespace table of
`duty_pct  total_thrust_g  vbat_V` and fit a power law. Thrust goes as roughly
the square of the average motor voltage, and the average voltage goes as duty,
so `T = a·d^b` with `b` near 2 is the expected shape — and a fit that *does not*
come out that way is telling you about the rig, not about aerodynamics.

```bash
python3 - thrust.txt 59.0 <<'PY'
import math, sys
mass_g = float(sys.argv[2])
pts = []
for line in open(sys.argv[1]):
    f = line.split("#")[0].split()
    if len(f) >= 2:
        pts.append((float(f[0]) / 100.0, float(f[1])))
n   = len(pts)
sx  = sum(math.log(d) for d, _ in pts)
sy  = sum(math.log(t) for _, t in pts)
sxx = sum(math.log(d) ** 2 for d, _ in pts)
sxy = sum(math.log(d) * math.log(t) for d, t in pts)
b   = (n * sxy - sx * sy) / (n * sxx - sx * sx)
a   = math.exp((sy - b * sx) / n)
d_h = (mass_g / a) ** (1.0 / b)
res = [t / (a * d ** b) - 1.0 for d, t in pts]
print(f"{n} points, total thrust  T(d) = {a:.1f} * d^{b:.2f} grams, d = duty as a fraction")
print(f"worst residual {max(abs(r) for r in res):.1%}  (a power law that does not fit is a rig fault, not physics)")
print(f"mass {mass_g:.1f} g  ->  HOVER DUTY = {d_h * 100:.1f} %")
print(f"thrust-to-weight at 100 % = {a / mass_g:.2f}   (below ~1.5 the hover is thrust-marginal)")
for name, v in (("controller linearisation point (0.583)", 0.583),
                ("FLIGHT_TUNING_LOG (0.71)", 0.71),
                ("main.cpp comment (0.15)", 0.15)):
    print(f"  vs {name}: {(v - d_h) / d_h:+.0%}")
PY
```

Worked example on an illustrative twelve-point table (25 %…80 %, 59.0 g
airframe) — this is the script's real output:

```
12 points, total thrust  T(d) = 142.8 * d^2.01 grams, d = duty as a fraction
worst residual 1.5%  (a power law that does not fit is a rig fault, not physics)
mass 59.0 g  ->  HOVER DUTY = 64.4 %
thrust-to-weight at 100 % = 2.42   (below ~1.5 the hover is thrust-marginal)
  vs controller linearisation point (0.583): -9%
  vs FLIGHT_TUNING_LOG (0.71): +10%
  vs main.cpp comment (0.15): -77%
```

Read that last block carefully. On these illustrative numbers the in-flight
observation (0.71) and the controller's linearisation point (0.583) bracket the
answer within ±10 %, and the `hover~0.15` comment is refuted by a factor of four.
**Your real numbers may land anywhere**; the point of the block is that the
measurement adjudicates between three inherited claims instead of adding a
fourth.

Two things follow from `d_h`, and neither is done by this document:

1. **The controller's linearisation point should become `d_h`.** The constant
   0.583 appears in four places — `controller.hpp` (the convention),
   `controller_pid.cpp:405`, `controller_tinympc.cpp:54-55` (the input box) and
   `main.cpp:1099` (the duty mapping). They must move together or the box and
   the mapping disagree, which is worse than both being wrong. Those files are
   the controller stack's, not this procedure's.
2. **`AUTOFLIGHT_MAX_DUTY` must be above `d_h` with real margin.** At 0.45 and a
   hover near 0.65 the drone cannot take off at all. The flights in
   `FLIGHT_TUNING_LOG.md` ran 0.95.

---

## 6. Part D — what changes for the 30-second hover

Not a calibration step, but the thing both calibrations are for, and it has one
requirement that catches everyone:

**The hover cannot happen in an uncommitted build.** The window is 5 s and the
hover is 30 s. So:

```bash
# 1. build committed, with the measured flow constant
RB_CMAKE_ARGS="-DROSE_FLIGHT_COMMIT=1 -DROSE_FLOW=1" \
RB_EXTRA_CPPFLAGS="-DCS_GPIO_PIN=0 -DFLOW_RAD_PER_COUNT=<measured>f" \
  ./rb build flight_controller --target arty200t \
      --config RocketArty200TDroneFullDDRConfig --mode esp-motors

# 2. boot it. It is STILL uncommitted; the banner says so.

# 3. standing at the drone, ready:
./rb debug flight_controller --target arty200t \
    --config RocketArty200TDroneFullDDRConfig --mode esp-motors \
    --usb-location <debug> --no-load --no-run \
    --gdb-script hardware/flight/commit.gdb
```

Wait for `MOTOR GATE: *** COMMITTED TO A FLIGHT TEST ***` on the console and
`flags=0x21[ARMED COMMITTED]` on the ESP before treating the props as live. The
budget is then 60 s, which covers a 30 s hover plus climb and descent — a
commitment is permission for *a flight test*, not permission forever.

`--no-load` matters: without it `rb` reloads the ELF and restarts the program,
throwing away the very write the script makes.

The rest of the flight-test procedure — the envelope guard, the tilt/free-fall/
impact cuts, and the ordered bench steps that prove each one — is in
**`docs/flight-safety-bench-procedure.md`**, and it should be run before this.

---

## 7. Command reference

```bash
cd /scratch2/loren/riskybird-dev-setup && source ./rb-env.sh
export RB_DTS_DIR=$HOME/riskybird-dts     # no elaborated DTS in this worktree

# --- builds -------------------------------------------------------------
./rb build pmw3901_test --target arty200t --config RocketArty200TDroneFullDDRConfig
./rb build motor_duty --target arty200t --config RocketArty200TDroneFullDDRConfig
./rb build esp_motors --target esp32c6

# --- run (drone shells load over JTAG: rb debug, NOT rb run) ------------
./rb console --port <fpga-console-port>          # terminal A, attach FIRST
./rb debug pmw3901_test --target arty200t --config RocketArty200TDroneFullDDRConfig \
    --program --program-usb-location <bit> --usb-location <dbg> --no-run
./rb debug motor_duty --target arty200t --config RocketArty200TDroneFullDDRConfig \
    --program --program-usb-location <bit> --usb-location <dbg> --no-run
#   then `continue` at the gdb prompt
./rb flash esp_motors --target esp32c6 --port <esp-port>

# --- flow solver --------------------------------------------------------
./tools/flow-cal.py --self-test
./tools/flow-cal.py --distance 1.000 --height 0.150 --axis x capture.log
./tools/flow-cal.py --distance 0.600 --height 0.250 --axis x --validate <K> validation.log
./tools/flow-cal.py --help        # threshold overrides: --min-counts --min-squal --max-leak --tolerance

# --- stops --------------------------------------------------------------
#   battery out                                     instant
#   x / q / SPACE on the motor_duty console         ~20 ms
#   unplug uart1 / halt the core                    120 ms (ESP failsafe)
./rb debug flight_controller --target arty200t --config RocketArty200TDroneFullDDRConfig \
    --mode esp-motors --usb-location <dbg> --no-load --no-run \
    --gdb-script hardware/flight/stop.gdb
```

Find the two FTDI USB locations with `rb probe-ftdi --idcode`. Both are required
when more than one FTDI is attached, which on this bench is always.

## 8. Files

| file | role |
|---|---|
| `workloads/pmw3901_test/` | flow diagnostic **and** calibration capture; prints `FLOWCAL RUN` lines with SQUAL and shutter |
| `workloads/pmw3901_test/src/pmw3901.c` | motion-burst read timing matched to the flight driver (~285 µs/read instead of ~3 ms) |
| `workloads/motor_duty/` | fixed-duty commander: `motor_link` frames at 50 Hz, four independent bounds, scripted sweep with a thermal drift check |
| `tools/flow-cal.py` | host-side solver, validator and self-test |
| `tools/rb-ext.py` | temporary registry shim for `motor_duty`; delete once `boards.py` carries the row |
| `samples/rose_flight_controller/src/flow.c` | `CS_GPIO_PIN` made overridable; `FLOW_RAD_PER_COUNT` lives here |
| `samples/rose_flight_controller/src/pmw3901.c` | chip-ID check now requires **both** `0x49` and `0xB6`; first read discarded |
| `integration/zephyr/motor_link/include/riskybird/motor_link.h` | the wire format, and the single copy of it |
| `workloads/esp_motors/` | the ESP end: applies duty, enforces its own ceiling, cuts on a 120 ms silence |
| `docs/flight-safety-bench-procedure.md` | the envelope guard, the commitment gate, and every way to stop the motors |

---

## 9. Not verified on hardware

**No step in this document has been executed on the drone.** The drone was on a
different bench, on a different machine, before any of this was written. What
follows is the honest list.

**Proven here, by building or running on the host:**

- all three images compile and link (§3.1, real output);
- the frame-rate `BUILD_ASSERT` fires, and `RB_EXTRA_CFLAGS` definitions reach
  `motor_duty`'s source (§3.2, real compiler error);
- `motor_duty` contains no commit-gate symbols and does contain its own latch
  (§3.3, real `nm` output);
- the flow solver recovers a known constant from synthetic counts and validates
  it against a second synthetic batch (§3.5, real output);
- the solver's rejection rules fire on the cases they claim to (§4.7, real
  output on an illustrative log);
- the thrust fit runs and produces the numbers shown (§5.9, real output on an
  illustrative table).

**Not proven, and the consequence of each being wrong:**

1. **Every console line quoted is a format string, not a capture.** Wording and
   spacing may differ. Nothing downstream parses them except
   `tools/flow-cal.py`, which parses only the `FLOWCAL RUN` line — if that line
   has changed, the solver will say `no usable FLOWCAL RUN lines found` rather
   than misparse.
2. **The `FLOWCAL RUN` line itself has never been printed by real silicon.** The
   capture path, the accumulator and the SQUAL/shutter reporting are all
   untested against a real sensor.
3. **The PMW3901 read-timing change is unmeasured.** The three `k_msleep(1)`
   calls became `busy_wait_us(50/100/50)` on the datasheet's ~35 µs t_SRAD. If
   100 µs is not enough on this board, reads return garbage — which would show
   as wild counts and a failed chip-ID gate, not as a quiet wrong answer.
4. **Every rejection threshold in `flow-cal.py` is a guess** (§4.9). Minimum
   SQUAL 25, leak 15 %, shutter 4×, 500 ms, 200 counts. They are all overridable
   from the command line for exactly this reason. Expect to move at least one
   after the first real batch.
5. **The ±10 % validation tolerance is an argument, not a measurement**
   (§4.10). It comes from a drift estimate that assumes a 0.2 m/s residual.
   Revisit it after a hover has actually been flown.
6. **The flow-lens / ToF reference-plane offset is unmeasured** (§4.8) and is
   currently nowhere in the flow path.
7. **`motor_duty` has never driven a motor.** The frame encoder is the shared
   header both ends compile, so the wire format is structurally right; that the
   ESP accepts these particular frames at this particular cadence is untested.
8. **The 50 Hz frame cadence has never been measured against the 120 ms
   failsafe on real hardware.** The `HOLD` line's `frames` counter exists so
   this is checkable on the first run: it must climb by ~50 per second.
9. **The 115200 baud on the FPGA↔ESP link is itself unverified.** Two overlays
   disagree about whether it was ever measured: `esp_bridge.overlay` says
   `esp_uart` proved it, `fpga-esp-uart.overlay` says it is a starting point.
   Settle that with `workloads/esp_uart` before blaming frames for anything.
10. **The break-away duty through the ESP path is unknown** (§5.5). 0.15 is the
    FPGA-path number and three mechanisms could move it.
11. **The hover duty is unknown**, which is the whole point. The worked example
    in §5.9 uses invented thrust readings chosen to be plausible; do not carry
    64.4 % forward as if it were measured.
12. **The stand-off height, restraint rating and 3-prop-diameter clearance are
    engineering judgement**, not measurements on this rig.
13. **`stop.gdb`'s `shell sleep 0.3` assumes the GDB host has a POSIX `sleep`.**
    On a Mac it does; it is noted because the assumption is silent.
14. **The `main.cpp` locked-rotor/ESC comment is stale and still in the tree**
    (§5.5). There are no ESCs on this airframe. Correcting it is a change to a
    file this procedure does not own.

**If you only check three things before trusting a number:** that the chip-ID
line reads `0x49` and `0xB6`; that the ESP console shows `cuts=0` and a climbing
`frames` for the whole of every thrust step; and that the drift-check step at the
end of a sweep reproduces the first step's thrust.
