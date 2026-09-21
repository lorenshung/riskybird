# Motor 4: remote investigation prompt

**Read this as a work order, not as a report.** It is written for an AI coding
agent running on the Linux host that has Chipyard and Vivado for this project
and can elaborate shells and build bitstreams. It was written on the bench Mac,
which can do neither, and hand-carried across. Nothing from the conversation
that produced it is available to you, so everything needed is restated here.

Evidence levels use the repo vocabulary: **verified** (run here, output quoted),
**recorded** (observed elsewhere, reproduced from someone else's log),
**declared** (asserted by code or docs, never run), **blocked**. Use the same
vocabulary when you report back, and do not promote a declared thing to a
verified one because it looks obvious.

What you have that the bench does not: `backends/chipyard` checked out, Vivado,
and the ability to run `./rb chipyard elaborate` and `./rb chipyard build`. On
the bench machine `backends/chipyard` is an **empty directory** — no Scala, no
`generated-src`, no `*.shell.xdc`, no `obj/report/io.txt`, and no
`arty_files/te0712-rd/board_files/` either. That is the entire reason this file
exists: every pin-level question below is unanswerable there.

---

## 1. The fault

A custom drone runs a RISC-V Rocket soft-core on a Trenz TE0712 (`xc7a200t`) on
a drone carrier. Four brushed motors are switched by low-side SI2302 N-FETs,
gate active-high, 20 kHz PWM. **There are no ESCs** — the FET gate is the motor
control signal, so any diagnosis that involves arming, calibration or
locked-rotor protection describes hardware this drone does not have.

Each gate is driven by two sources onto the same node: the FPGA through a 47R,
and an on-board ESP32-C6 through its own 47R, with a 10k pulldown holding the
FET off when neither side drives. That shared node is what makes the
measurement in section 5 possible.

Channel map, from the harness binder `WithArty200TPWM` in
`backends/chipyard/fpga/src/main/scala/arty200t/HarnessBinders.scala`, which is
the only authority for it:

| Motor | Block | Comparator | Ball | J1 |
| --- | --- | --- | --- | --- |
| `motor1` | `pwm@10050000` | 1 | E22 | J1_31 |
| `motor2` | `pwm@10050000` | 2 | B20 | J1_36 |
| `motor3` | `pwm@10050000` | 3 | R17 | J1_99 |
| `motor4` | `pwm@10051000` | 1 | F13 | J1_100 |

`motor4` is the **only** motor on the second PWM block. Comparator 0 of each
block sets the shared period and Zephyr refuses to drive it
(`"PWM channel 0 cannot be configured"`, returns `-ENOTSUP`), so a
four-comparator block yields three usable outputs and four motors need two
blocks. That asymmetry is the shape of this bug: the one motor that is alone on
the second block is the one that does not work.

Every motor output is inverted in the generated harness:

```verilog
assign motor1 = ~_chiptop0_pwm_0_gpio_1;
assign motor2 = ~_chiptop0_pwm_0_gpio_2;
assign motor3 = ~_chiptop0_pwm_0_gpio_3;
assign motor4 = ~_chiptop0_pwm_1_gpio_1;
```

**The fault: `motor4` never turns when driven from the FPGA.** Motors 1–3 do.

The board owner reports that motor 4 **previously did work** from the FPGA
(**recorded**). Treat this as a fault that developed, not as a design that never
worked. Do not write up a conclusion that requires it to have always been
broken without saying explicitly that it contradicts the owner.

---

## 2. What is already eliminated — do not re-run these

All of the following are closed. Re-deriving any of them wastes a build.

**The register programming is correct — verified.** Direct JTAG readback of
`pwm1` while driving it:

```console
pwm1 cmp1 = 375  (0x177) -> M4 gate 0
pwm1 cmp1 = 2500 (0x9c4) -> M4 gate 0
pwm0 cmp1 = 2500 (control) -> M1 gate 1000
```

Both blocks read `pwmcfg = 0xe0001200` — bit 9 `pwmzerocmp`, bit 12
`pwmenalways`, scale 0, `pwmip1..3` high with `pwmip0` low — and
`pwmcmp0 = 0x9c4` (2500 counts = 20 kHz at 50 MHz), with their counters
advancing. `pwm1`'s configuration is byte-for-byte identical to `pwm0`'s. The
`pwm1` register file is alive and holds what is written to it.

**The gate node receives nothing — verified.** With all four FPGA comparators
driven to 15% simultaneously, read at the gates:

```console
MOTOR GATE duty per mille: M1=145 M2=145 M3=144 M4=0
```

Driving `pwm1` cmp1 to 2500, which should put the pad at ~100% high, still gives
M4 = 0 at the gate. The control on the same run — `pwm0` cmp1 = 2500 — gives
M1 = 1000. This is not a duty error or a polarity error. Motor 4's gate receives
**nothing at all**.

**The probe itself was validated, not assumed — verified.** The obvious
objection to everything above is that the instrument is lying: a stuck-low input
buffer on the one ESP pin that reads motor 4's gate would produce exactly the
observed result while meaning nothing about the FPGA. That objection is now
closed. `workloads/motor_safe` self-tests each pin by driving it high and reading
it back through its own input buffer (2 ms, one pin at a time, then parked):

```console
PROBE SELFTEST M1 (GPIO21): configure=0 readback=1 -- input path OK
PROBE SELFTEST M2 (GPIO20): configure=0 readback=1 -- input path OK
PROBE SELFTEST M3 (GPIO23): configure=0 readback=1 -- input path OK
PROBE SELFTEST M4 (GPIO22): configure=0 readback=1 -- input path OK
```

All four input paths read back their own drive, GPIO22 included. So `M4 = 0` is
a measurement, not a dead sensor.

**One earlier test was confounded and is NOT evidence — say so if you see it
quoted elsewhere.** Driving the gates from the ESP while the FPGA held all four
pads low was meant to discriminate an open F13 (ESP wins at 3.3 V) from a
connected pad (47R/47R divider, ~1.65 V, weaker). Observed: motors 1 and 4 spun
harder than 2 and 3. Motor 1's gate demonstrably *does* receive FPGA drive, so
it should have been divided and weak. It was not. Motor-to-motor stiction
variation swamps the electrical effect at these duties, so that test resolves
nothing in either direction. The gate probe is the evidence; spin strength is
not.

**The motor, the FET, the gate net and the battery rail are good — recorded.**
Motor 4 spins when the ESP32-C6 drives GPIO22 through LEDC
(`samples/riskybird/actuator_bringup`). The ESP reaches the same gate node the
FPGA does, through its own 47R.

**Two different bitstreams, same result — verified.**
`RocketArty200TDroneFullDDRConfig` and `RocketArty200TMotorSpadConfig` both
fail identically. These are different shells with different memory systems, so
this is not one bad build.

**A full power cycle changed nothing — verified.** Both USB cables and the
battery out for 15+ s.

**The Zephyr driver and the devicetree are cleared — verified.** Both PWM
devicetree instances are generated, linked and initialised; there is no
per-port Kconfig in `pwm_sifive.c`; and the elaborated devicetree maps `motor4`
to `pwm@10051000` channel 1. `tests/test_pwm.py` checks the driver `#define`s,
the emitted register map JSON, the harness binder and the devicetree overlay
against each other and passes 22 tests. The generated DTS for
`RocketArty200TDroneFullDDRConfig` has both blocks with four comparators each:

```
L27: pwm@10050000 { interrupts = <2 3 4 5>; sifive,ncomparators = <4>; ... }
L28: pwm@10051000 { interrupts = <6 7 8 9>; sifive,ncomparators = <4>; ... }
```

**The PWM RTL itself is correct — verified in simulation.**
`hardware/pwm/run.sh` Verilates the `PWMTimer` out of a given config's
`gen-collateral` and drives it with exactly the register writes Zephyr's driver
performs. Every duty from 5% to 100% reaches the simulated pin within one count
in 2500, the pin is low at reset and low for `pwm_set_dt(period, 0)`, and three
channels hold three different duties. That covers the block, not the route out
of it.

**Bank membership and connector parity explain nothing — verified.** From the
post-route `report_io` of `RocketArty200TMotorSpadConfig`:

```
| B20 | motor2 | High Range | IO_L16P_T2_16         | OUTPUT | LVCMOS33 | 16 | 12 | SLOW | FIXED | PULLDOWN |
| E22 | motor1 | High Range | IO_L22P_T3_16         | OUTPUT | LVCMOS33 | 16 | 12 | SLOW | FIXED | PULLDOWN |
| F13 | motor4 | High Range | IO_L1P_T0_16          | OUTPUT | LVCMOS33 | 16 | 12 | SLOW | FIXED | PULLDOWN |
| R17 | motor3 | High Range | IO_L24N_T3_A00_D16_14 | OUTPUT | LVCMOS33 | 14 | 12 | SLOW | FIXED | PULLDOWN |
```

F13 is in **bank 16, with `motor1` and `motor2`, both of which work**. So
`VCCO_16` is powered at 3.3 V and its output drivers function; a dead bank is
not the explanation. Motor 4 is also on an **even** J1 pin (J1_100) like
`motor2`'s J1_36, so connector parity is not it either.

**Note the scope of that last table carefully.** It is `report_io` for
`RocketArty200TMotorSpadConfig` and for no other config. Whether
`RocketArty200TDroneFullDDRConfig` was ever checked at pin level — XDC or
`report_io` — is **unknown**. `tests/test_pwm.py` hardcodes
`MOTOR_CONFIG = "RocketArty200TMotorSpadConfig"` and skips entirely when that
config has not been elaborated, so its XDC assertion has never covered FullDDR.
Establishing whether FullDDR's pin constraints and placement match is part of
your job in section 3.

---

## 3. Verification work: do this first, before building anything

These are cheap, need no bitstream, and one of them may settle the whole
question. Do them in order.

Region (A) is *inside the FPGA design*: the comparator output never reaches the
F13 IOB, so the pad is never driven. Region (B) is *physical*: the F13
ball/solder/board-to-board seating, or the carrier's 47R, trace or JB1 pin for
motor 4. The bench cannot separate them. You can narrow (A) a long way from the
build tree alone.

### 3.1 Elaborate FullDDR and trace `motor4` back into `DigitalTop`

```console
$ ./rb chipyard elaborate --board arty200t --config RocketArty200TDroneFullDDRConfig
```

Then, in
`backends/chipyard/fpga/generated-src/chipyard.fpga.arty200t.Arty200THarness.RocketArty200TDroneFullDDRConfig/`:

1. Confirm `gen-collateral/Arty200THarness.sv` contains
   `assign motor4 = ~_chiptop0_pwm_1_gpio_1;` and the three `pwm_0` assignments
   for motors 1–3.
2. **Follow `_chiptop0_pwm_1_gpio_1` upward.** Find where the harness declares
   it, find the `ChipTop` instantiation that produces it, and find the port on
   `DigitalTop` it comes from. The question to answer is not "does the
   assignment exist" — it does — but **is that wire genuinely connected to
   `pwm1`'s comparator 1 inside `DigitalTop`, or is it tied off, driven by a
   constant, or left dangling and defaulted?** Quote the declaration and every
   assignment to it.
3. Do the same for `_chiptop0_pwm_0_gpio_1`, which drives `motor1` and
   **works**. Diff the two paths structurally. A difference between them is the
   most likely single cause of this bug, and it is findable without Vivado.
4. Inside `DigitalTop`, check `TLPWM.sv` and `PWMTimer.sv` for *both* instances.
   A known failure mode on this path: with `PeripheryPWMKey` set but no
   IOBinder consuming the output, `firtool` deletes the whole output path and
   the generated `TLPWM` has **no `gpio` port at all** — while the comparators
   still count, the registers still map, and software still reads back exactly
   what it wrote. That was **verified** on
   `RocketArty200TDroneLogicConfig`:

   ```console
   $ grep -c gpio "$D/gen-collateral/TLPWM.sv"
   0
   ```

   The instructive variant of that failure is a **partial** one: the binder
   consumes block 0's outputs and not block 1's, so block 0 keeps its `gpio`
   port and block 1 loses it. Check the two `PWMTimer` / `TLPWM` instances
   separately; do not check one and generalise.

**Read `gen-collateral/*.sv`, not `obj/*.v`.** Synthesis folds the harness
inverter into a LUT and the `assign motor4 = ~...` form disappears, so a grep
over `obj/` will not find what you are looking for and its absence there means
nothing.

### 3.2 Confirm the IOBinder is applied to both PWM blocks

`chipyard.iobinders.WithPWMPunchthrough` is what brings the comparator outputs
out of `DigitalTop`; `WithArty200TPWM` maps comparator to package pin and adds
the inversion. Both are **declared** present in
`RocketArty200TMotorSpadConfig`'s definition. Whether they are both present in
`RocketArty200TDroneFullDDRConfig`, and whether the punchthrough covers **both**
blocks rather than just the first, is **unknown**.

Determine, and report:

- the exact config lines for `RocketArty200TDroneFullDDRConfig` in
  `backends/chipyard/fpga/src/main/scala/arty200t/Configs.scala`;
- whether `WithPWMPunchthrough` iterates over all `PeripheryPWMKey` instances
  or only one, quoting the Scala;
- **the count of punched-through PWM signals** in the elaborated FullDDR — how
  many `pwm_<n>_gpio_<m>` ports exist on `ChipTop`/`DigitalTop`, and their
  names. Four motors need at least `pwm_0_gpio_{1,2,3}` and `pwm_1_gpio_1`. If
  the count is three, you have found the bug.

### 3.3 The generated XDC

```console
$ grep motor generated-src/chipyard.fpga.arty200t.Arty200THarness.RocketArty200TDroneFullDDRConfig/*.shell.xdc
```

Expect, and confirm the presence of, for `motor4`:

```
set_property PACKAGE_PIN {F13} [get_ports {motor4}]
set_property IOSTANDARD {LVCMOS33} [get_ports {motor4}]
set_property PULLDOWN {TRUE} [get_ports {motor4}]
```

Quote all four motors' entries side by side, including any `DRIVE` and `SLEW`
properties, and say whether `motor4`'s set differs from motors 1–3's in any
respect. Also grep the whole XDC for any **other** constraint mentioning `F13`
or `motor4` — a duplicate `PACKAGE_PIN` on F13 from some other port, or a
`set_property` that makes the port an input, would produce exactly this
symptom. `docs/hm01b0-camera-bringup.md` records checking the camera shell's
XDC for exactly that: "26 `PACKAGE_PIN` constraints, **all unique**". Do the
same check here and report the number and whether they are unique.

Then do the same for `RocketArty200TMotorSpadConfig` and diff the two XDCs over
the motor ports. `docs/te0712-drone-carrier-ddr-bringup.md` used a full
`*.shell.xdc` diff between two configs to settle a comparable question; that is
the pattern to follow.

### 3.4 The implemented design is the authority, not the RTL

**This is the most important instruction in section 3.** Reading
`gen-collateral/*.sv` is **not sufficient on its own** to prove the pad is
driven. The RTL can be perfect and the pad still dead: an output can be
optimised away, tied to a constant, or left without an `OBUF` after
implementation, and nothing in `gen-collateral` records that. The
post-implementation netlist or a pin report is the authority.

For each config you have an implementation run for:

- Search the Vivado synthesis and implementation logs and reports for every
  message naming `motor4` or `F13`. Specifically look for warnings that the
  port is **undriven**, **tied to a constant**, **has no load**, or was
  **optimised away** — and for anything about an unconstrained or
  unplaced port. Quote the messages verbatim with their IDs.
- Check `obj/report/io.txt` for the `motor4`/F13 row and compare it against the
  MotorSpad rows quoted in section 2. Confirm `OUTPUT`, `LVCMOS33`, bank 16,
  drive 12, `PULLDOWN`.
- **Open the placed and routed design and ask what actually drives the F13
  `OBUF`.** Trace from the pad back through the `OBUF` to its driver and say
  whether the driver is a LUT fed by `pwm1`'s comparator, a LUT fed by a
  constant, or a tie cell. Then do the same for E22 (`motor1`) as the control.
  This single check is what distinguishes region (A) from region (B) without
  touching the board. Report the driver cell names for both.

If you cannot open an existing implemented design because no FullDDR run
survives on the machine, say so rather than inferring, and note that the answer
then requires a rebuild.

---

## 4. Diagnostic bitstreams, ranked

Each of these discriminates a specific hypothesis. Judge feasibility yourself
and say so if one is impractical in this shell — that judgement is more useful
than a forced attempt. **Prefer the least invasive one first.**

### (c) Drive F13 from the sifive GPIO block instead of PWM — build this first

**Probably the single most informative build of the three.** Route `motor4`'s
pad at F13 from a plain sifive GPIO output rather than from a PWM comparator,
giving a directly software-togglable static level on F13.

- **Hypothesis it discriminates:** "does ball F13 drive at all". A static high
  written from software is the cheapest possible test, needs no PWM reasoning,
  no phase, no duty measurement, and is readable directly with the existing ESP
  gate probe. A static high should read **1000** per mille at the M4 gate and a
  static low should read **0**.
- **If F13 goes high:** the ball, its solder, the B2B seating, the carrier 47R,
  the trace and the JB1 pin are all good, and the fault is entirely inside the
  FPGA design on the `pwm1`-to-F13 path. Region (A). That closes the board-side
  question that the bench currently cannot close with a meter.
- **If F13 stays at 0 from GPIO too:** the design is exonerated for that pad and
  the fault is physical. Region (B), and the repair is on the board.
- **Known complication, so you are not surprised by it:**
  `RocketArty200TDroneFullDDRConfig` elaborates `gpio@10010000` with three PLIC
  sources (`interrupts = <12 13 14>`), i.e. a **3-bit** GPIO block, and those
  three bits are already bound — the drone overlay uses them with the PMW3901
  optical-flow sensor. So this needs either a widened GPIO block or a rebind of
  one existing bit, and either is a real change to the shell. See the PLIC
  hazard in section 6 before you widen anything.

### (a) Route `motor4` at F13 from a different `pwm0` comparator

- **Hypothesis it discriminates:** whether the fault is specific to the **second
  PWM block's** path out of `DigitalTop`. `pwm0`'s path is proven by three
  working motors.
- **If motor 4 then works:** `pwm1`'s path inside the design is the fault, and
  F13 is fine.
- **If it still fails:** F13 or the board is the fault, and `pwm1` is
  exonerated.
- **Constraint:** `pwm0` cmp0 is **unusable** — it sets the shared period and
  Zephyr refuses it. Cmp1/2/3 are taken by motors 1–3. So this needs either a
  spare comparator on `pwm0` (check whether the block can be instantiated with
  more than four comparators, and what `sifive,ncomparators` and
  `sifive,comparator-widthbits = <16>` permit), or a build in which one of
  motors 1–3 is temporarily given up in exchange for its comparator. The
  give-up variant is a perfectly good diagnostic — say which motor you sacrificed
  so the bench knows to expect it dead.

### (b) Route `pwm1` comparator 1 to a different, known-good J1 pin

- **Hypothesis it discriminates:** whether `pwm1` can drive **any** pad at all.
  This is (a) with the variables swapped, and it is worth having both because
  together they bracket the answer without relying on either being decisive
  alone.
- **How the bench reads it:** a jumper from that J1 pin to motor 4's gate, or a
  probe on it. Choose a J1 pin that is unused in this shell and physically
  reachable, state which one and its ball, and say what it is adjacent to on
  the header so the person placing the jumper does not short it to something
  live.
- Report your pin choice **before** relying on it; the bench has to agree it is
  free on the carrier.

---

## 5. How the bench will verify each build

So you know exactly what evidence comes back and in what form.

**The ESP-as-gate-probe.** `workloads/motor_safe` runs on the ESP32-C6 and
parks GPIO21, GPIO20, GPIO23 and GPIO22 — motors 1, 2, 3, 4 respectively — as
pulled-down **inputs** via a devicetree GPIO hog, with the LEDC output matrix
explicitly disabled. It then samples all four 2,000 times and prints the
fraction high in per mille, every 250 ms:

```console
MOTOR SAFE: 4/4 ESP drivers high-impedance with pull-downs
MOTOR GATE duty per mille: M1=145 M2=145 M3=144 M4=0
```

Because each gate reaches the ESP through one 47R and the FPGA through another,
with a 10k pulldown, a high-impedance ESP input reads the **real SI2302 gate
node** rather than anything upstream of the FPGA pad. That is what makes it
evidence about delivery rather than about intent. No FPGA-side register read can
establish this: the comparator can be perfectly programmed while nothing reaches
the ball, which is precisely the state this board is in.

The 250 ms interval is deliberate — an FPGA sweep drives each motor for as
little as 1 s, and a slower probe samples between pulses and reports a dead gate
for a working one.

**Direct register pokes over JTAG with OpenOCD.** Independent of any workload,
symbol table or control loop:

```gdb
monitor mww 0x10051024 375     # pwm1 cmp1 -> motor4, 15% of 2500
monitor mdw 0x10051024         # read it back; never trust the write
monitor mww 0x10050024 375     # pwm0 cmp1 -> motor1, the control
```

Register layout, both blocks, base `0x10050000` and `0x10051000`:

| | | |
| --- | --- | --- |
| `pwmcfg` | `+0x00` | `0xe0001200` is what Zephyr's init leaves behind |
| `pwmcount` | `+0x08` | resets every period; a difference of two reads proves nothing |
| `pwms` | `+0x10` | the live scaled counter, i.e. the phase |
| `pwmcmp0` | `+0x20` | the shared period, 2500 counts = 20 kHz at 50 MHz |
| `pwmcmp1..3` | `+0x24`, `+0x28`, `+0x2c` | motors |
| `pwmcfg[28+n]` | | `pwmcmpXip`, the live comparator output for channel *n* |

`hardware/flight/stop.gdb` and `hardware/pwm/stop.gdb` already zero exactly
these four comparator addresses, in that order, before halting the core.

**Known-good reference numbers at the gate.** Any new build is read against
these:

| Written to cmp | Expected at gate |
| --- | --- |
| 375 (15%) | **~145** per mille |
| 2500 (100%) | **1000** per mille |
| 0 (stopped) | **0** per mille |

A working channel hits those; motor 4 currently reads 0 for all three.

**One safety note about the bench procedure**, because it constrains what a
useful build looks like: halting the CPU does **not** stop the motors. The
sifive PWM block is separate hardware and keeps driving whatever the comparators
hold. The comparators must be zeroed before the core is halted, never after.

---

## 6. Constraints and hazards you must respect

**The inversion is a safety requirement, not a preference.** Every motor output
is inverted in the harness, and this must be preserved in any new config. The
reason: the sifive comparator drives its output high once the scaled counter has
passed the comparator value, and `pwmcmp = 0` — which is both Zephyr's "stop"
and the **reset state of the register** — makes that condition permanently true.
Uninverted, the pad sits at a constant logic high. On this board that is **four
motors at full throttle from the cycle after configuration releases, before a
single instruction has run.**

| | uninverted | inverted (what ships) |
| --- | --- | --- |
| reset, before software runs | **100%** | 0% |
| `pwm_set_dt(period, 0)` | **100%** | 0% |

`RocketArty200TDroneFullDDRConfig` drives the motor pads from configuration
onward, so there is no window in which this does not apply. The XDC
`PULLDOWN TRUE` covers only the earlier window, before the design is live at all
and the fabric is high-Z. If you build a GPIO-driven variant for diagnostic (c),
the same requirement holds: **its reset state must leave F13 low.** State
explicitly, in what you send back, what level F13 sits at between configuration
and the first software write, and how you established it.

**PLIC source numbering shifts silently.** The drone overlay hardcodes it —
I2C 1, `pwm0` 2–5, `pwm1` 6–9, UART 10, GPIO 11–13, SPI 14, OSPI 15 — and
adding or dropping any interrupt-bearing controller shifts every source above
it. That failure already cost this bench its console UART once. It is also
already live here: `RocketArty200TDroneFullDDRConfig` elaborates a second UART
(`serial@10021000`, the ESP link) at source 11, which pushes GPIO to 12–14 and
SPI to 15, which is why `hardware/zephyr/fpga-esp-uart.overlay` exists beside
`hardware/zephyr/fpga-drone.overlay`. If your diagnostic config changes the
periphery set, say so loudly and say which overlay it needs; `rb` reads the
numbering out of the generated DTS rather than guessing from the config name, so
publishing the DTS (below) is what lets the bench pick correctly.

**Bitstream naming.** Bitstreams reach the bench through
`tools/collect-bitstreams.py`, which copies
`backends/chipyard/fpga/generated-src/chipyard.fpga.arty200t.Arty200THarness.<Config>/obj/Arty200THarness.bit`
to `<board>/<board>.<Config>.bit` and writes `manifest.json` and `README.md`
beside it. The bench consumes that directory through `RB_BITSTREAM_DIR`
(`~/riskybird-bitstreams`), and the elaborated DTS through `RB_DTS_DIR`
(`~/riskybird-dts`) via `tools/collect-dts.py`. So **any new config must be
named so that `arty200t/arty200t.<Config>.bit` is unambiguous**, and the config
name you use in Scala is the name the bench will type. Pick something explicit
about what the build is for.

> **Do not point `--out` at the bench's existing checkouts.** The collectors
> copy files in without deleting, but they **overwrite `manifest.json` and
> `README.md`** with only what *this* machine has elaborated or built. Aim them
> somewhere else and copy across what you want, or the collection arrives with
> one manifest claiming one bitstream beside forty files.

**The bench is a Mac.** It cannot build bitstreams and cannot elaborate —
Vivado is Linux-only and `backends/chipyard` is an empty directory there. It can
program, run Zephyr workloads, drive JTAG and read registers. Anything that
requires elaboration has to happen on your side and arrive as a published
artifact.

**The bitstream lives in SRAM.** `--write-sram` loses it on a power cycle, and
installing or moving motors almost certainly means one. Say in your hand-off
whether a given diagnostic needs a reload after any power event.

**Propellers off.** Every spin test on this bench runs with propellers removed
and +BATT connected. Do not write a procedure that assumes otherwise.

---

## 7. What to send back

Keep it to evidence plus a ranked conclusion. Quote real output; do not
paraphrase it.

**Files:**

- `generated-src/…RocketArty200TDroneFullDDRConfig/*.shell.xdc` — the whole
  file, or at minimum every line mentioning any `motor*` port or `F13`.
- The `assign motor*` lines from
  `gen-collateral/Arty200THarness.sv`, plus the declaration and every
  assignment of `_chiptop0_pwm_1_gpio_1` and `_chiptop0_pwm_0_gpio_1`, and the
  `ChipTop`/`DigitalTop` port lines they connect to.
- `module` headers of both `TLPWM.sv` and `PWMTimer.sv` instances, showing
  whether each has its `io_gpio_*` ports.
- The `RocketArty200TDroneFullDDRConfig` definition from `Configs.scala`, and
  the `WithArty200TPWM` and `WithPWMPunchthrough` bodies.
- `obj/report/io.txt`, the `motor*` rows.

**Report excerpts:**

- Every synthesis/implementation message naming `motor4` or `F13`, verbatim with
  message IDs — especially undriven, constant-tied, no-load and
  optimised-away.
- The placed-design answer to "what drives the F13 `OBUF`", and the same for
  E22 as the control, with cell names.
- The count of punched-through `pwm_<n>_gpio_<m>` signals, and their names.
- The count of `PACKAGE_PIN` constraints in the FullDDR XDC and whether they are
  unique.

**Bitstreams:**

- Published through `tools/collect-bitstreams.py` as
  `arty200t/arty200t.<Config>.bit`, with the accompanying `manifest.json` entry
  (part, LUT, FF, timing, WNS/WHS, sha256, built time).
- The matching elaborated `.dts` through `tools/collect-dts.py`, so the bench
  can resolve the PLIC numbering rather than guess it.
- For each one: the hypothesis it discriminates, which comparator or GPIO bit
  drives F13, whether the harness inversion is preserved, what level F13 idles
  at before software runs, and the exact `monitor mww` address and value the
  bench should poke to exercise it.

**And say which region you think it is.** Region (A) inside the FPGA design or
region (B) physical, with your confidence and the single piece of evidence that
would move you. If section 3 settles it without a build, say that first and
plainly — a diagnostic bitstream nobody needs is the expensive outcome here.
