# Tethered flight attempt — procedure and safety audit

**Airframe:** riskybird v3, ~58–60 g, props ON.
**Mode:** `flight_controller --mode esp-motors` (FPGA computes, ESP32-C6 drives all four gates).
**Written:** 2026-09-21. **Read the whole of Part 0 before touching anything.**

---

## Part 0 — Read this first

### 0.1 Nothing in this document was tested on the drone

Every behavioural claim here was derived by reading the source and confirmed only as far as
"it compiles and the symbol/string is in the ELF". The drone was not attached to the machine
this was written on. Claims are marked:

- **[CODE]** — proven by reading the source, quoted inline. Reliable.
- **[BUILT]** — proven by building and inspecting the artifact on this machine. Reliable.
- **[BENCH]** — measured on the real drone earlier, on a previous session. Reliable, but it was
  a *bench* measurement (props off or low duty), not a flight measurement.
- **[UNVERIFIED]** — expected from the code but never observed. **Treat as a hypothesis.**

The single most important **[UNVERIFIED]** item: *nobody has ever measured this airframe's thrust
curve.* Every duty number below is inferred from one in-flight observation (71 % hover) and a
square law. If the drone behaves differently from what this document predicts, the document is
wrong, not the drone. Stop and re-measure.

### 0.2 The three conditions, and whether they hold

The owner's conditions were: *the watchdog is always on; it never runs too powerfully or for too
long; emergency stops are always available.* The audit is Part 6. The short answers:

| Condition | Holds? | With what caveat |
|---|---|---|
| Watchdog always on | **Yes, now** | It was silently removable via `-DROSE_THREADED=1`. That build is now refused at compile time. One of its seven tests suspends itself under vibration — visibly (`acc180deg`, `accskip=`), and the other six keep running. |
| Never too powerful | **Only if you set BOTH ceilings, with the right knob** | There are two independent caps in two separate images; raising one is silently clamped by the other. And the FPGA-side knob most people reach for (`MOTOR_MAX_DUTY`) is a *scale*, not a cap — using it as a cap detunes the controller instead of limiting it. See 0.3. |
| Never too long | **Yes** | 5 s uncommitted / 60 s committed, latched, clearable only by a board reset. Covers `flight_controller` **only** — not `motor_duty`. |
| Emergency stops available | **Partly** | Two are instant and need no setup. Two need a debugger *already attached* and are not emergency stops if it is not. |

### 0.3 The single most dangerous trap in this system

**THERE ARE TWO INDEPENDENT DUTY CEILINGS, IN TWO SEPARATE IMAGES, ON TWO SEPARATE BOARDS.**

| | Lives in | Default | Behaviour |
|---|---|---|---|
| FPGA side | `MOTOR_DUTY_CEILING` in `main.cpp` (**this is the cap**) | `1.0` | a **collective ceiling**: excess is subtracted from all four, differential preserved |
| FPGA side | `MOTOR_MAX_DUTY` in `main.cpp` | `0.10` | a **scale** on every duty — *not* a cap, see 0.3.1 |
| ESP side | `ESP_MOTORS_MAX_DUTY_PCT` in `workloads/esp_motors` | `25` | a **collective ceiling**, applied last, before the gate |

The ESP deliberately does not trust the FPGA. If you raise `MOTOR_MAX_DUTY` to 0.75 and reflash
only the FPGA, every motor is still clamped at 25 % at the gate. **From the FPGA console this is
indistinguishable from a dead pack, a broken motor or a mis-tuned controller.** The natural
reaction — assume something is broken and start raising other things in a hurry — is exactly how
this ends badly.

Both images must be rebuilt and reflashed **in the same session**, and both boot banners must be
read back. Part 2 does this.

### 0.3.1 `MOTOR_MAX_DUTY` is a SCALE. Do not use it as the flight cap.

This is the trap inside the trap, and it is the single correction this audit made that most
changes what you type.

On the bench path (`--mode esp-motors` is a bench path — it is not autoflight) `MOTOR_MAX_DUTY` is
applied **last, as a multiplier**, to a duty that is already physical [CODE]:

```c
duty[i] = (u[i] + 0.583f) * batt_scale;   /* ... anti-saturation cut ... */
duty[i] *= MOTOR_MAX_DUTY;                /* bench: scale [0,1] -> [0, cap] */
```

The controller's output **is** the physical duty it wants: `u + 0.583` round-trips
`controller_pid.cpp`'s `forceToVoltage()` exactly. So multiplying by 0.75 does not "cap at 75 %",
it makes every number the tuned controller produces 25 % smaller — including the altitude
feedforward, which the PID altitude loop has **no integrator** to recover from, and including the
attitude differential.

Computed from the real `actuator_duty()` arithmetic, at a hover-ish command
`u = [0.137 0.117 0.147 0.127]` (≈ 0.72/0.70/0.73/0.71 duty):

| Form | Applied duty | Differential |
|---|---|---|
| `MOTOR_DUTY_CEILING=0.75` (a ceiling) | `[0.7200 0.7000 0.7300 0.7100]` | `0.0100` — untouched |
| `MOTOR_MAX_DUTY=0.75` (a scale) | `[0.5400 0.5250 0.5475 0.5325]` | `0.0075` — shrunk 25 % |

0.53 duty against a ~0.71 hover duty is about **55 % of hover thrust** (thrust ≈ duty²). The drone
does not fail cleanly: while the altitude error is large the loop saturates and it climbs at the
cap; as the error shrinks the proportional term falls away and, with no integrator to make up the
scaled-down feedforward, it settles well below the setpoint or sinks. With props on, on a tether,
that is a thing you have to watch happen.

**Both forms cap the worst-case duty at the same number.** The difference is that the ceiling
leaves the controller exactly as it was tuned and only acts when it binds, and the scale detunes
it everywhere. For a flight attempt, use the ceiling:

```
-DMOTOR_MAX_DUTY=1.0f        # scale OFF (pass-through)
-DMOTOR_DUTY_CEILING=0.75f   # the actual cap
```

**Both, or neither.** `MOTOR_MAX_DUTY=1.0f` alone leaves this image with no cap of its own — the
one mistake in this chain that fails *towards* more power. The boot banner shouts about it
[BUILT — the string is in an image built that way, and folded out of one built with a cap]:

```
flight_controller: *** DUTY CHAIN -- NO MEANINGFUL POWER CAP ON THIS SIDE *** 100 % is ~1.98x
hover thrust (hover ~0.71 duty, thrust ~ duty^2). If you meant to cap power, set
-DMOTOR_DUTY_CEILING; MOTOR_MAX_DUTY alone is a SCALE and detunes the controller instead of
capping it.
```

### 0.4 Related trap: `AUTOFLIGHT_MAX_DUTY 0.45f` is probably below hover

Not used by `--mode esp-motors`, but read this so you recognise it if you end up in autoflight.
There are three numbers in this tree claiming to be the hover duty and they differ by ~5×:

- **0.583** — the controller's *linearisation point*, not a measurement (`duty = (u + 0.583f) *
  batt_scale` in `actuator_duty()`, and TinyMPC's input box `u_min -0.583 / u_max 1-0.583`).
- **0.71** — **measured in flight.** `FLIGHT_TUNING_LOG.md`: *"~71 % hover duty, raw command
  saturating 42–65 % of the flight"*. Those flights ran `AUTOFLIGHT_MAX_DUTY=0.95`;
  `FLIGHT_BUILD.md`'s recipe passes `0.8f` and notes *"hover needs ~65%"*.
- **0.15** — a bare comment that used to sit beside `AUTOFLIGHT_MAX_DUTY`. It is numerically
  identical to `MOTOR_BREAKAWAY_DUTY` and was evidently copied from it. **It is not a hover duty.**
  That comment has been corrected.

**If hover is really ~0.65–0.71, an autoflight build capped at 0.45 physically cannot lift.** The
drone will sit on the floor with all four motors pinned at the ceiling and a console that looks
perfectly healthy. That is a *safe* failure. **Do not react to it by raising limits in a hurry.**
The build now says so at boot [BUILT]:

```
flight_controller: *** AUTOFLIGHT_MAX_DUTY 0.450 IS BELOW THE MEASURED 0.71 HOVER DUTY *** --
this build may be unable to leave the ground. That is a SAFE failure. Do not react to it by
raising limits in a hurry.
```

### 0.5 Things that will waste your time or hurt you

- **`rb run` does not work on the drone shells.** They are elaborated `uartTsi = false` with no
  TSI node. Use `rb console` first, then `rb debug --program --no-run`, then `continue`.
- **There is a console-attach race.** Attach `rb console` *before* `rb debug`, or the boot banner
  — which is where you read the ceilings back — scrolls past before you are listening.
- **CMake cache variables persist across builds.** [BUILT] A `-DROSE_THREADED=1` passed once
  stayed in the build tree and failed the *next* build that did not pass it. The same is true of
  `MOTOR_MAX_DUTY` and `ESP_MOTORS_MAX_DUTY_PCT`: **a duty cap you set once stays set until the
  build tree is wiped.** Always use `--pristine always` when changing a cap, and always confirm
  the value from the boot banner rather than from the command you think you ran.
- **A stalled brushed motor is a resistor across the pack.** There are no ESCs on this airframe —
  the motors are brushed, driven low-side by SI2302 FETs straight off the LiPo. A motor commanded
  below break-away does not spin, does not cool, and heats in seconds. **Never leave the drone
  sitting at a sub-break-away duty while you read the console.**
- **The 0.15 break-away figure is an FPGA-path number and may not hold now.** Three things changed
  when the ESP took the gates: different V_GS on the SI2302 (ESP 3.3 V through its own 47R vs FPGA
  VCCO through the FPGA's), the FPGA path's motor outputs are *inverted* in the generated Verilog
  and nobody re-derived whether a commanded duty was the duty the gate saw, and the quantisation
  differs (SiFive comparator 1/2500 vs LEDC 11-bit). Treat 0.15 as an order of magnitude.
- **Never run two builds against the same build tree at once.** `--pristine always` deletes the
  build directory, so a concurrent build sees its own generated headers vanish mid-compile and
  fails with a cascade of *"zephyr/syscall_list.h: No such file or directory"*. That is a race,
  not a broken source tree — check for another `west build` before believing a build failure.
  [BUILT: hit twice on this machine and cleared by retrying once the other build finished.]
- **Motors need the battery.** Nothing useful happens on USB-C alone.
- **There is NO remote kill in this build.** Every `rose_cmd_*` hook in `main.cpp` — `estop`,
  `disarm`, `reset` — is inside `#if defined(CONFIG_WIFI)` [CODE, lines 2599–2636], and
  `--mode esp-motors` does not enable `CONFIG_WIFI`, so none of them is compiled in. A
  ground-station ESTOP/DISARM button does nothing on this carrier; a dashboard that appears to ACK
  is the bridge fabricating the ACK itself. **Do not plan around a remote stop you do not have.**
  See 6.3.
- **Console telemetry costs control-loop time, and that time lands on the motor link.** The FPGA
  console is a polled 115200 FTDI link (`CONFIG_UART_INTERRUPT_DRIVEN` is off), so every byte
  `printk` writes busy-waits *inside the control loop*. See 4.3a before turning telemetry up.

---

## Part 1 — What you need at the bench

- The drone, tethered. **Props on.** Tether short enough that the drone cannot reach anything.
- 1S LiPo, freshly charged, **with the connector where your hand can reach it without leaning
  over the props.** This is the primary emergency stop.
- FPGA JTAG/console FTDI, ESP USB-C.
- **Three terminals:** (1) `rb console` on the FPGA, (2) the ESP console, (3) the shell you drive
  `rb build` / `rb debug` from.
- Eye protection.

---

## Part 2 — Build and flash (battery DISCONNECTED)

> **Battery out for the whole of Part 2 and Part 3.** Do not reconnect it until Part 4.

### 2.1 Decide the cap before you build

Recommended for a first tethered attempt: **`MOTOR_DUTY_CEILING=0.75f` with
`MOTOR_MAX_DUTY=1.0f`** on the FPGA, **`ESP_MOTORS_MAX_DUTY_PCT=80`** on the ESP. Read 0.3.1 before
you decide to type `MOTOR_MAX_DUTY=0.75f` instead; it is not the same thing and it is not a cap.

Why 0.75, reasoned out loud so you can disagree with it:

- The only measured hover duty this tree has is **0.71** (`FLIGHT_TUNING_LOG.md`). The cap must be
  *above* it or the drone cannot lift at all (0.4).
- Brushed-motor thrust goes roughly as duty², so a cap `s` gives about `(s/0.71)²` of weight:
  - `s = 0.75` → **1.12 × weight** → net ≈ 0.12 g ≈ **1.2 m/s²**. Enough to lift and be controlled;
    a gentle push against a tether.
  - `s = 1.00` → **1.98 × weight** → net ≈ 0.98 g ≈ **9.6 m/s²**. A hard yank at the end of the
    tether. This is what "full power" means on this airframe and it is why the cap exists.
- `MOTOR_DUTY_CEILING` is the ceiling the **attitude-priority anti-saturation cut** works against,
  so when it binds it lowers the *collective* and leaves the roll/pitch/yaw differential intact —
  a little less altitude, never less attitude. Below it, it does nothing at all and the controller
  behaves exactly as tuned. [CODE] This is the same semantics `AUTOFLIGHT_MAX_DUTY` has on the
  autoflight path, and the reason a plain per-motor clamp is the wrong tool: a clamp flattens four
  different commands into four equal numbers, and four equal duties are zero attitude torque.
- The ESP is set to **80**, slightly *above* the FPGA cap, deliberately. It must not be the binding
  constraint (that is the silent-clamp trap), but it still backstops at 80 % — so even a
  mis-built FPGA image, including one that forgot `MOTOR_DUTY_CEILING` entirely, cannot reach full
  power. Belt and braces, in the right order.
- The ESP's ceiling is now a **collective** ceiling too, not a per-motor clamp (this audit changed
  it). Before, an FPGA image commanding above the ESP's ceiling would have had all four duties
  clamped to the same number at the gate — zero attitude torque at exactly the moment the vehicle
  was asking for the most thrust. Now the two ceilings compose: whichever is lower simply bites
  first, and the vehicle stays controllable either way. [CODE]

**This is arithmetic on one observation, not a thrust measurement.** Climb the ladder in 4.3
rather than jumping to 0.75.

Also recommended: shorten the committed window from 60 s to 20 s
(`-DROSE_COMMIT_MAX_MS=20000`). 20 s is twice the autoflight hard cap (`FLIGHT_MAX_MS` 10 s) and
longer than any recorded hop; 60 s of a 1S LiPo at flight power is most of the pack.

### 2.2 Build the FPGA image

```bash
cd <riskybird checkout>
RB_DTS_DIR=$HOME/riskybird-dts \
RB_CMAKE_ARGS="-DROSE_FLIGHT_COMMIT=1 -DMOTOR_MAX_DUTY=1.0f -DMOTOR_DUTY_CEILING=0.75f -DROSE_COMMIT_MAX_MS=20000" \
  ./rb build flight_controller \
    --target arty200t --config RocketArty200TDroneFullDDRConfig \
    --mode esp-motors --pristine always
```

[BUILT] on this machine, exit 0, `RAM: 418432 B`. Verified in the generated `build.ninja`:
`-DMOTOR_MAX_DUTY=1.0f`, `-DMOTOR_DUTY_CEILING=0.75f`, `-DROSE_FLIGHT_COMMIT=1` all present on the
compile line.

- **Both duty knobs are CMake cache variables**, listed in the sample's `target_compile_definitions`.
  `RB_CMAKE_ARGS` is what reaches them. A bare `-D<name>` for a name that is *not* listed there is
  discarded — the same trap the sample's `CMakeLists.txt` already documents for `PID_MASS_KG` and
  `CS_GPIO_PIN`.

- `-DROSE_FLIGHT_COMMIT=1` is **not** set by any `--mode`, on purpose. Without it the symbol
  `g_flight_commit_key` is not in the image at all, `commit.gdb` fails with *"No symbol"*, and
  motors latch off after 5 s no matter what you do. That failure is the interlock working.
- `--pristine always` is not optional here. See 0.5.

### 2.3 Build and flash the ESP image — **do not skip this**

```bash
RB_CMAKE_ARGS="-DESP_MOTORS_MAX_DUTY_PCT=80" \
  ./rb build esp_motors --target esp32c6 --pristine always
./rb flash esp_motors --target esp32c6
```

[BUILT] exit 0; `CMakeCache.txt` shows `ESP_MOTORS_MAX_DUTY_PCT:STRING=80` and the flag reaches
`main.c`'s compile line.

> **`RB_EXTRA_CFLAGS` DOES NOT WORK ON THIS TARGET.** If another document tells you to use
> `RB_EXTRA_CFLAGS=-DESP_MOTORS_MAX_DUTY_PCT=80`, it is wrong. `tools/rb/cli.py` only assembles
> `EXTRA_CFLAGS` inside `if board is not None`, and the `esp32c6` target has `fpga_board=None`, so
> the flag is silently dropped — the build succeeds, the banner still says 25 %, and nothing
> reports that anything was ignored. [BUILT: with the flag set and `--pristine always`, no
> `-DESP_MOTORS_MAX_DUTY_PCT` appeared anywhere in `compile_commands.json`.] `workloads/esp_motors/CMakeLists.txt`
> now declares both knobs as cache variables so `RB_CMAKE_ARGS` reaches them, which is the form
> shown above.

### 2.4 Read both ceilings back from the boot banners

Attach `rb console` **first** (console-attach race), then load and run:

```bash
# terminal 1
./rb console
# terminal 2 (ESP console, e.g.)
picocom -b 115200 /dev/ttyACM0
# terminal 3
./rb debug flight_controller --target arty200t \
  --config RocketArty200TDroneFullDDRConfig --mode esp-motors \
  --usb-location <debug> --program --no-run
(gdb) continue
```

On the **FPGA** console you must see all of [BUILT: these strings are in the ELF]:

```
flight_controller: ENVELOPE GUARD -- accel tilt >40 deg, free-fall <3.920 m/s2, impact >29.430 m/s2 ...
flight_controller: ENVELOPE GUARD -- est tilt >40 deg, rate >10 rad/s, vel >2.500 m/s, height >2.000 m
flight_controller: ENVELOPE GUARD -- dwell 120 ms (40 ms for free-fall/impact), min 2 samples; ACTIVE ONLY WHILE ARMED
flight_controller: ENVELOPE GUARD -- armed=COMPILE-TIME TRUE in this build, so the latch is live from the first control tick
flight_controller: ENVELOPE GUARD -- accel-tilt test is SUSPENDED while |a| is outside 5.890..14.720 m/s2; telemetry then reads acc180deg and counts it in accskip=. The other six tests keep running.
flight_controller: DUTY CHAIN -- collective ceiling MOTOR_DUTY_CEILING = 0.750 (anti-saturation cut, differential preserved), then x MOTOR_MAX_DUTY 1.000
flight_controller: DUTY CHAIN -- WORST CASE THIS IMAGE CAN COMMAND: 75.00 % per motor
flight_controller: DUTY CHAIN -- the ESP applies its OWN ceiling (ESP_MOTORS_MAX_DUTY_PCT) after this one and does not trust this image; read it off the ESP console's 'duty ceiling NN%' boot line. The LOWER of the two wins.
flight_controller: COMMIT GATE -- build is COMMIT-CAPABLE (ROSE_FLIGHT_COMMIT=1) but NOT COMMITTED: motors latch OFF 5000 ms after they first spin.
flight_controller: MOTOR OFFLOAD -- FPGA drives no gate; duty frames at 50 Hz on uart@10021000 to the ESP32-C6
```

The `WORST CASE` line is the one number that answers *"how powerful can this get"*. The image
computes it (`MOTOR_DUTY_CEILING x MOTOR_MAX_DUTY`) rather than leaving you to multiply two flags
you believe you passed. **It is this image's worst case, not the gate's** — the ESP clamps
afterwards and this image cannot read that ceiling, which is exactly why the third line points at
the ESP console.

On the **ESP** console you must see:

```
esp_motors: failsafe 120 ms, tick 5 ms, duty ceiling 80%
```

**STOP if either number is not what you built.** `75.00 %` and `80%` must both appear, and there
must be **no** `*** DUTY CHAIN -- NO MEANINGFUL POWER CAP ON THIS SIDE ***` line. If the ESP says
`25%`, section 2.3 did not take — go back and wipe the build tree.

---

## Part 3 — Prove the guard is alive, battery still DISCONNECTED

This is the step that makes the rest of it safe. The guard was, until recently, wrong by a factor
of two (`state[3..5]` are Rodrigues parameters, `tan(θ/2)`, and a limit named `SAFE_MAX_TILT_RAD`
was being compared against them as though they were radians — a "57°" guard that really tripped at
90°). **Never assume it is looking. Make it prove it.**

The telemetry line carries the honest numbers, in degrees, at the end:

```
flight_controller: it=1020 dt=28ms roll=-0.949 pitch=-0.029 ... duty=[0.000 0.000 0.000 0.000] tilt=est87deg/acc86deg |a|=9.807 accskip=0
```

- `roll/pitch/yaw` are **Gibbs parameters, not radians and not degrees.** Ignore them.
- `tilt=estNdeg/accNdeg` are the real numbers: `est` from the estimator, `acc` straight out of
  gravity with no estimator in the path.
- `|a|` is the accelerometer magnitude, ~9.81 at rest.

### 3.1 At rest

Expected: `tilt=est0..5deg/acc0..5deg |a|=9.6..10.0`.

**If `acc` reads 180, the accelerometer direction test is suspended** (|a| outside the 0.6–1.5 g
trust band) — at rest that means the IMU is wrong, not that the drone is shaking. **Abort.**
`accskip=` at the end of the line counts how many times this has happened since boot; at rest it
should not be advancing.

### 3.2 Tilt test — the guard must LOOK

Pick the frame up and tilt it slowly to roughly 20–30°, hold, and return it level.

- **`acc` must follow your hand.** At 30° it should read ~30. [CODE: `asinf(sqrtf(horiz2/total2))`]
- `est` should agree within a few degrees at rest.
- Nothing should trip (the limit is 40°).

**If `acc` does not move, the guard is blind and the attempt is over.** Do not continue. There is
no second opinion: `est` alone is what missed the 90° rotation last time.

### 3.3 Trip test — the guard must FIRE

Now tilt deliberately past 40° and **hold it there for about a second.**

Expected [BENCH — this exact line was measured on the drone]:

```
flight_controller: *** EMERGENCY CUTOFF *** tilt limit exceeded: measured 47.457 deg est-tilt, held 141 ms over 5 samples; motors OFF (reset to clear)
```

The reason may print as `est-tilt` or `accel-tilt` — either is a pass. It said `est-tilt` on the
bench, which means the *accelerometer* test was suspended at that moment because moving the frame
by hand pushed |a| out of the gravity band. That is the complementary pair working as designed:
the estimator test covers exactly the case the accelerometer test cannot.

On the ESP console you should simultaneously see `flags=0x02[ESTOP ]` and all four duties at 0.

**This latches `g_estop`.** That is the point. Board-reset before Part 4 — a *soft* reset
(`reset.gdb`) is not enough to start a flight from, and it will tell you so.

### 3.4 Link test

Read the ESP status line and confirm a healthy link before any power goes near a motor:

```
  link=UP seq=113 flags=0x00[off] frames=693 gap=30ms lost=0 stale=0 crcerr=0 noise=0 cuts=0 pwmerr=0
```

[BENCH: 693 frames, zero errors.] `lost`, `stale`, `crcerr`, `noise`, `pwmerr` must all be **0**.
`gap` should be ~20–35 ms (the loop rate, which is slower than the 50 Hz frame rate).

### 3.5 Halt test — prove the offload failsafe

With the core running, halt it from GDB (or just let `stop.gdb` do it). Expected within ~125 ms
[BENCH: `cuts=1`]:

```
  link=DOWN -- MOTORS OFF (failsafe) ... cuts=1
```

This is the property that makes the offload path *safer* than the old PWM path: on the PWM path
the SiFive comparators are free-running hardware and a halted core left them driving the gates
forever. Here, a halted core, a crashed core, a reset and an unplugged ribbon all stop the motors.

---

## Part 4 — Props on, battery connected

> From here on the drone can hurt you. **Hand on the battery connector at all times.**

### 4.1 Before connecting the battery

- [ ] Parts 2 and 3 all passed, including 3.3 firing and 3.5 cutting.
- [ ] Board has been **hard reset** since the 3.3 trip test (`g_estop` latches).
- [ ] Tether attached, short, and clear of the props.
- [ ] Both consoles attached and scrolling.
- [ ] Everyone behind the plane of the props.

### 4.2 First power-on with props

Connect the battery. **Expect nothing to spin.** [CODE: `esp_motors` writes all four to 0 before
the UART is even enabled and starts with the failsafe deadline already expired; the FPGA's
`motors_startup_pulse()` sends one explicit disarmed zero frame. There are no chirps on the
offload path.]

If anything spins on power-up, **pull the battery** and stop.

### 4.3 Climb the duty ladder — do not jump to 0.75

Each step is a **rebuild + reload** of the FPGA image (`MOTOR_DUTY_CEILING` is compile-time). It
is tedious and it is the point: you get to look at the drone at every power level before the next.

`MOTOR_MAX_DUTY=1.0f` stays fixed at every step — only the ceiling moves. (If you change
`MOTOR_MAX_DUTY` instead, re-read 0.3.1; you will be detuning the controller by a different amount
at every step and comparing runs that are not comparable.)

| Step | `MOTOR_DUTY_CEILING` | Worst-case duty | What you are looking for |
|---|---|---|---|
| 1 | `0.25f` | 25 % | All four spin. Directions correct. Nothing rattles loose. **Keep it short** — this is close to break-away and a motor that does *not* spin is heating. |
| 2 | `0.45f` | 45 % | Vibration check. Watch `\|a\|` in telemetry; note whether `acc` ever reads 180 and whether `accskip=` moves. |
| 3 | `0.60f` | 60 % | Drone should get light on its feet, may skitter. Attitude differential visible in `duty=[...]`. |
| 4 | `0.75f` | 75 % | The attempt. |

For each step, rerun 2.2 with the new ceiling (and `--pristine always`), reload, and re-read the
`DUTY CHAIN -- WORST CASE` banner line before connecting the battery. That line should read
`25.00 %`, `45.00 %`, `60.00 %`, `75.00 %` in turn. **If it does not match the step you think you
are on, stop.**

At step 2, note what `\|a\|` does with props spinning, and watch `accskip=`. [BENCH, at low duty:
`\|a\|` ranged 7.85–12.46 m/s², and **0 of 44 samples** fell outside the 0.6–1.5 g band
(5.89–14.72).] If `accskip=` climbs steadily at step 3 or 4, prop vibration is suspending the
accelerometer tilt test — see 6.1 for what that does and does not mean. **It is information, and
it is the measurement this project is missing**, not by itself an abort. Write the number down.

### 4.3a Telemetry rate — check it before you raise it

The console telemetry line is the thing you will be reading for the whole attempt, so it is worth
knowing what it costs and what it does *not* cost.

**What it does not cost: the motor link's bandwidth.** In `--mode esp-motors` the console stays on
**uart0** and the binary duty frames go out on **uart1**. Only `--mode telem` moves the console
onto uart1 [CODE: `flight_controller-telem.overlay` is the only overlay that sets
`zephyr,console = &uart1`]. The two do not share a wire in this mode, and anything that tells you
they do is describing `--mode telem`.

**What it does cost: control-loop time, which is the same thing in the end.** `printk` on this
carrier busy-waits, so a telemetry iteration stalls the loop for as long as the write takes, and
`send_control()` — and therefore the next motor frame — waits behind it. At 115200, the
`it=`/`flow:` pair is roughly **290 bytes ≈ 25 ms of busy-wait** on the iteration that prints it.

**The number that matters is the worst-case gap between motor frames, against the ESP's 120 ms
failsafe.** It is `loop period + polled-write time`, so:

```
worst gap ≈ dt + 25 ms        (on the iteration that prints)
```

Read `dt=` off the telemetry line itself. On this build it has been **28–29 ms**, so the worst gap
is ≈ **55 ms against a 120 ms failsafe** — about 2× margin. That is adequate, and it is the
configuration the rest of this document assumes.

**The rate is `dt × ROSE_TELEM_DIV`, and `--mode esp-motors` sets neither knob**, so it inherits
`ROSE_TELEM=1`, `ROSE_TELEM_DIV=10` from the CMake defaults. At `dt = 29 ms` that is one line every
~290 ms (~3.4 Hz), which is exactly what you want to watch by eye.

**Where this bites:** `DIV` is an *iteration* count, not a time. If the loop ever gets faster —
a build without the down-ToF or without flow, for example — `DIV=10` is 77 Hz and ~22 kB/s of
busy-wait against a link that carries 11.5 kB/s, and the loop simply stops being a control loop.
So:

- **Check `dt=` on the console before trusting any `DIV`.** If `dt` is a few milliseconds rather
  than a few tens, raise `DIV` to keep the line rate near 3–5 Hz (`DIV ≈ 0.25 s / dt`).
- **Never raise the telemetry rate for a flight attempt.** More lines is strictly less margin
  against the failsafe, and the failure it produces — spurious `cuts=` on the ESP — looks like a
  link fault or a flaky receiver rather than like a console you turned up.
- If you want more data, turn telemetry *off* (`-DROSE_TELEM=0`) and use the flight log, which is
  a background thread into RAM and costs the loop nothing. You lose the live view that Part 5 is
  built around, so this is a trade, not an upgrade.
- **Keep `rb console` attached.** An *undrained* polled console fills the TX FIFO and
  `poll_out` spins on it — measured 30–60 ms stalls, ~22 % of a flight, which also corrupts the
  estimator's `dt`. The console being attached is not just for you to read.

### 4.4 The uncommitted run — 5 seconds, free

Load and `continue`. The controller commands thrust immediately (it regulates to `TARGET_Z` and
the drone starts on the floor), so the motors go live at once and you get **5 seconds**:

```
MOTOR GATE: WINDOW OPEN -- motors are live; 5000 ms budget starts now (uncommitted)
MOTOR GATE: MOTORS LIVE 1000/5000 ms (uncommitted)
...
MOTOR GATE: WINDOW EXPIRED after 5009 ms -- ALL FOUR MOTORS COMMANDED OFF AND LATCHED (board reset to clear)
```

[BENCH: latched at 5009 ms.] **Do this at least once at the flight cap before committing.** Five
seconds is enough to hear all four, see the duties in telemetry, and feel whether the drone is
trying to lift. Then it stops itself, latched, and you cannot restart it without a board reset —
which is exactly what you want after a first look.

### 4.5 The committed run

Board-reset. Then, **standing at the drone, ready**:

```bash
./rb debug flight_controller --target arty200t \
  --config RocketArty200TDroneFullDDRConfig --mode esp-motors \
  --usb-location <debug> --no-load --no-run \
  --gdb-script hardware/flight/commit.gdb
```

`--no-load` matters: a reload restarts the program and throws away the write.

**Commit BEFORE the motors run.** The gate refuses a commit once the window has already latched
and tells you so. The commit is not accepted until the console prints:

```
MOTOR GATE: *** COMMITTED TO A FLIGHT TEST *** -- motor budget is now 20000 ms (was 5000 ms). Props are live.
```

**Props are live from that line.** You now have the committed budget (20 s if you set
`ROSE_COMMIT_MAX_MS=20000`, 60 s if you left the default), after which all four latch off and only
a board reset brings them back.

---

## Part 5 — What good looks like, and when to abort

### 5.1 Good

FPGA console, roughly every 300 ms:

```
flight_controller: it=2400 dt=29ms roll=0.031 pitch=-0.012 yaw=0.004 z=0.180 tofv=1 tofh=0.180
  u=[0.402 0.411 0.398 0.407] duty=[0.735 0.742 0.731 0.739] tilt=est3deg/acc4deg |a|=9.930 accskip=0
MOTOR GATE: MOTORS LIVE 4000/20000 ms (COMMITTED)
```

ESP console, every 250 ms:

```
MOTOR CMD duty per mille: M1=735 M2=742 M3=731 M4=739
  ceil=80% link=UP seq=87 flags=0x21[ARMED COMMITTED ] frames=2140 gap=29ms lost=0 stale=0 crcerr=0 noise=0 cuts=0 pwmerr=0
```

`ceil=80%` is the ESP's own ceiling, repeated on every status line so it is visible for the whole
run and not only in a boot banner that has scrolled away. `gap=29ms` is the **measured** interval
between motor frames — this is the number to watch against the 120 ms failsafe (4.3a).
`flags=0x21` is `ARMED | COMMIT`. All four duties within a few per mille of each other while
level; they should *split* fore/aft on a pitch disturbance and left/right on a roll one — that is
the attitude differential and it is the thing you are actually testing.

### 5.2 Abort — pull the battery

- Any of `lost`, `stale`, `crcerr`, `noise`, `pwmerr` leaving 0.
- `gap` above ~60 ms, or `cuts` incrementing while you did not ask for it. (Expected worst case is
  ~54 ms on a telemetry iteration — see 4.3a. Sustained values above that mean the control loop is
  being stalled by something, and the 120 ms failsafe is closer than it looks.)
- `accskip=` climbing fast enough to track the iteration count — the accelerometer tilt test is
  then effectively off and only `est-tilt` is guarding attitude (6.1). Not an instant abort;
  a reason to come down and write the number down.
- `flags` not `0x21` while you expect thrust.
- One motor's duty per mille persistently unlike the other three (a gate not being driven).
- `tilt=` reading something your eyes disagree with.
- `acc180deg` *at rest*.
- Any smell, any change in motor note, any part of the airframe moving that should not.
- **Anything you do not understand.** The battery connector is free and the cost of pulling it is
  one board reset.

---

## Part 6 — The audit

### 6.1 "Watchdog is ALWAYS on"

**Compiled in unconditionally.** The guard block in the control loop is behind no `#if` at all:

```c
{
        int need_ms = SAFE_DEBOUNCE_MS;
        ...
        const char *why = safety_violation(state, f.gyro, f.accel, &need_ms, &meas, &unit);
```

**`g_armed` is a compile-time `true` in `esp-motors`.** Confirmed — `esp-motors` does not set
`ROSE_AUTOFLIGHT`, so:

```c
#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
static volatile bool g_armed;         /* set true by the arming check; false before/after flight */
static int64_t       g_flight_start_ms;
#else
static const bool    g_armed = true;
#endif
```

So the boot banner's "ACTIVE ONLY WHILE ARMED" is true but reads narrower than it is: in this mode
there is **no unarmed window** — the latch is live from the first control tick to the last. The
banner now says so explicitly (2.4).

**No window where motors can be commanded non-zero while the guard is not evaluating.** [CODE]

- *Boot:* `motors_startup_pulse()` transmits one explicit **disarmed, all-zero** frame before the
  loop starts. The ESP starts with all four at 0 and its failsafe deadline already expired.
- *First iteration:* the guard is evaluated after `est.update()` and **before** `send_control(u)`,
  in the same iteration. There is no iteration in which `send_control` runs before the guard has
  looked.
- *IMU failure:* `read_sensor_frame()` returning false `continue`s *without* calling
  `send_control` — the frame stream pauses, and >120 ms of pause is itself a cut at the ESP. Ten
  consecutive misses latch `g_estop`. Fail-safe in both directions.
- *Post-estop:* `g_estop` forces zeros and clears `why` so nothing can un-latch it; only a reset
  clears it.
- *During the commit gate:* `motor_gate_permit()` is called **last**, on the duty that would
  actually have been sent, so no branch reaches the wire without passing through it.

**THE ONE HOLE, NOW CLOSED.** The `-DROSE_THREADED=1` build has a completely different control
loop (`io_block` / `est_block` / `ctrl_block`) that calls `send_control()` and **never calls
`safety_violation()`** — `safety_violation` is called from exactly one place in the file. The boot
banner still printed, because `safety_banner()` runs in `main()` before the blocks start. So that
build commanded thrust with no guard at all and announced a guard it did not have. `ROSE_THREADED`
is not set by any `--mode` and defaults to 0, so this never shipped — but a hand-written
`RB_CMAKE_ARGS` could reach it, and a flight build *already* has to pass `RB_CMAKE_ARGS`.

Now refused at compile time [BUILT — this error was made to fire on purpose]:

```
src/main.cpp:1844:2: error: #error "ROSE_THREADED=1 on a real target: the threaded blocks never
call safety_violation(), so the envelope guard would be absent while motors are commanded. Build
the single loop (-DROSE_THREADED=0, the default), or port the guard into io_block()/ctrl_block()
first."
```

**The accelerometer trust-band hole — what actually happens.** `accel_envelope()` skips the
*direction* test while |a| is outside the trust band:

```c
if (amag >= (SAFE_ACC_TRUST_LO_MPS2) && amag <= (SAFE_ACC_TRUST_HI_MPS2)) {
        const float tilt = asinf(sqrtf(horiz2 / total2));
        g_guard_tilt_rad = tilt;
        if (tilt > (SAFE_ACC_TILT_RAD)) { ... return "accel-tilt"; }
} else {
        g_guard_tilt_rad = 3.14159265f;
        g_guard_acc_skips++;
}
```

**What happens to the other tests when it is skipped: nothing. They all keep running.** Free-fall
and impact are tested *before* the band check and are magnitude tests that do not care about
direction; est-tilt, rate, velocity, height and battery come from the estimator and the gyro and
never touch this path. One of seven tests suspends, not the guard.

**And the suspension is visible, not silent** — `g_guard_tilt_rad` is set to π, which the
telemetry line prints as `acc180deg`, and `g_guard_acc_skips` counts every occurrence and is
printed as `accskip=`. "It did not trip" and "it is not looking" produce different output, which
is the whole point, and there is now a counter so a *transient* blindness that you did not happen
to be watching is still recorded.

**Decision: DO NOT widen the band. It stays at 0.6–1.5 g.**

```c
#define SAFE_ACC_TRUST_LO_MPS2 5.89f   /* 0.6 g */
#define SAFE_ACC_TRUST_HI_MPS2 14.72f  /* 1.5 g */
```

This was the hardest call in the audit, and it was reversed once during it — a wider band
(0.4–2.0 g) was tried and then taken back out. The argument, in full, because the next person to
reach for these numbers should argue with it rather than guess:

**The case for widening** is real: prop vibration at *flight* duty has never been measured, and a
band the vibration pushes |a| out of is a test that quietly stops running exactly when it is most
wanted.

**What killed it** is that widening does not buy a test that trips. Outside the band |a| is not
gravity, so `asin(|horiz|/|a|)` is not attitude — and under thrust- or vibration-dominated |a| the
horizontal part stays small relative to a *larger* total, so the formula **under-reports**. A
widened band gains a test that reads a plausible small number and still does not trip. Against a
limit, *absent* and *silently under-reporting* fail identically. There is no safety gained.

**And widening actively costs something.** `g_guard_tilt_rad` is what the telemetry line prints as
`accNdeg`, and the entire pre-flight proof that the guard is alive (3.2) is *"tilt the frame by
hand and watch `acc` follow"*. Suspended, it publishes π — `acc180deg`, an unmistakable *"I do not
know"*. Widened, it would publish a believable wrong angle instead. Trading an honest refusal for
a plausible fiction is the wrong trade in a device whose last failure was a guard that looked like
it was working.

**What was done instead: make the suspension countable.** `g_guard_acc_skips` counts every sample
the direction test sits out. It is printed as `accskip=` on the telemetry line and by
`hardware/flight/guard-check.gdb`. At rest it stays 0. If it climbs while props spin, the
accelerometer test is effectively off and `est-tilt` is carrying the attitude envelope alone —
which is a thing to know and act on, not a thing to hide behind a wider band.

**Residual risk, unresolved and stated plainly:** nobody has measured |a| at flight-like duty,
only at bench duty (7.85–12.46 m/s², 0 of 44 samples outside the band). If `accskip=` climbs
steadily at ladder steps 3 and 4, then at flight duty this guard is six tests, not seven, and the
band should be revisited **on that evidence**. Record the number (4.3). That measurement is the
thing this decision is currently missing.

### 6.2 "It never runs too powerfully or for too long"

#### Power — the complete chain

1. Controller → `u[i]`, nominally in `[-0.583, 0.417]`, **but not bounded there**: TinyMPC was
   measured returning 1.28 against a bound of 0.417 (which is why `esp-motors` forces PID).
2. `duty[i] = (u[i] + 0.583f) * batt_scale`, then floored at 0.
3. **Battery sag scale** `batt_thrust_scale()`: `clamp(3.8/vbat, [1.0, 1.30])`. **Inactive in
   `esp-motors`** — `ROSE_BATT_SENSE` defaults to 0 and the mode does not set it, so this returns
   exactly 1.0. It also never *reduces* thrust.
4. **Attitude-priority anti-saturation cut**, against `MOTOR_DUTY_CEILING` (this audit replaced a
   hard-coded `1.0f` here with the knob): if the peak exceeds the ceiling, subtract the excess
   from **all four**, preserving the differential. After this, peak ≤ `MOTOR_DUTY_CEILING`.
   ```c
   const float ceiling = (float)(MOTOR_DUTY_CEILING);
   ...
   if (peak > ceiling) {
           const float cut = peak - ceiling;
           for (int i = 0; i < NACTIONS; i++) { duty[i] -= cut; if (duty[i] < 0.0f) duty[i] = 0.0f; }
   }
   ```
5. **`MOTOR_MAX_DUTY`**, applied as a **scale**, then a now-redundant clamp:
   ```c
   duty[i] *= MOTOR_MAX_DUTY;                 /* bench: scale [0,1] -> [0, cap] */
   if (duty[i] > MOTOR_MAX_DUTY) duty[i] = MOTOR_MAX_DUTY;
   ```
   A flight build sets this to `1.0f` so it is a no-op — see 0.3.1 for why using it as the cap
   instead detunes the controller.
6. `motor_link_duty_from_float()` saturates to 10000 (100.00 %) — never wraps, and NaN → 0.
7. **The ESP's independent ceiling**, last thing before the gate. This audit changed it from a
   per-motor clamp to the same collective cut, so that the two ceilings compose without either of
   them flattening the attitude differential:
   ```c
   if (peak > ceiling) {
           const uint32_t cut = peak - ceiling;
           for (unsigned i = 0; i < MOTOR_LINK_NMOTORS; i++) { d[i] = (d[i] > cut) ? (d[i] - cut) : 0U; }
   }
   ...
   if (d[i] > ceiling) { d[i] = ceiling; }   /* unreachable given the cut; kept as the invariant */
   ```

**Worst case that can reach a gate =
`min(MOTOR_DUTY_CEILING x MOTOR_MAX_DUTY, ESP_MOTORS_MAX_DUTY_PCT/100)`.**

- **With stock defaults: 10.00 %** (1.0 ceiling × 0.10 scale = 0.10, vs the ESP's 25 %). Below the
  0.15 break-away — the drone cannot reliably spin all four, let alone fly. The FPGA banner says
  so: `WORST CASE THIS IMAGE CAN COMMAND: 10.00 % per motor  (bench cap -- CANNOT LIFT; hover
  needs ~65-71 %)`.
- **With the recommended flight build: 75.00 %** (0.75 ceiling × 1.0 scale, vs the ESP's 80 %).
- **With `MOTOR_MAX_DUTY=1.0f` and no ceiling: 100 %**, i.e. ~1.98× hover thrust, limited only by
  the ESP's 80 %. This is the one mis-build that fails *towards* power, and it is why the banner
  prints `*** NO MEANINGFUL POWER CAP ON THIS SIDE ***` in that case.
- `AUTOFLIGHT_MAX_DUTY` (0.45) is on the *autoflight* path only and does not apply here.

Computed from the real `actuator_duty()` arithmetic on the host, using the measured saturated
bench command `u = [1.082 0.977 1.429 1.541]`:

| Configuration | Applied duty |
|---|---|
| default bench (ceiling 1.0, scale 0.10) | `[0.0541 0.0436 0.0888 0.1000]` |
| tether flight (ceiling 0.75, scale 1.0) | `[0.2910 0.1860 0.6380 0.7500]` |

Note that in both cases the peak lands exactly on the cap and the differential survives — that is
the anti-saturation cut doing its job.

Recommended cap and its justification are in 2.1. **Nobody has measured the thrust curve**, so the
whole argument rests on one number (0.71) and a square law; be conservative, climb the ladder.

| Cap | Thrust / weight | Net vertical accel |
|---|---|---|
| 0.60 | 0.71 | −2.80 m/s² (cannot lift) |
| 0.70 | 0.97 | −0.27 m/s² (marginal) |
| **0.75** | **1.12** | **+1.14 m/s²** |
| 0.80 | 1.27 | +2.64 m/s² |
| 0.90 | 1.61 | +5.95 m/s² |
| 1.00 | 1.98 | +9.65 m/s² |

(hover duty 0.71, thrust ∝ duty², mass ~60 g. These are the numbers the cap recommendation rests
on and every one of them is arithmetic, not measurement.)

#### Time — the commitment gate

```c
if (g_motor_latched) {
        return false;
}
...
if (g_motor_live_since == 0) {
        if (!live) { return true; }   /* nothing has spun yet: the clock has not started */
        g_motor_live_since = now;
        ...
}
const int64_t elapsed = now - g_motor_live_since;
if (elapsed >= (int64_t)motor_gate_limit_ms()) {
        g_motor_latched = true;
        printk("MOTOR GATE: WINDOW EXPIRED after %d ms -- ALL FOUR MOTORS COMMANDED OFF AND LATCHED ...");
        return false;
}
```

- **The clock starts at first non-zero duty**, not at boot — so a slow sensor bring-up does not eat
  the budget.
- **`CTRL_ITERS=0` does not defeat it.** The gate is timed on `k_uptime_get()` and has no
  relationship to the iteration count. `CTRL_ITERS=0` makes the *loop* unbounded; the *motors* are
  still bounded. [CODE]
- **The latch cannot be cleared except by a reset.** `g_motor_latched` is written `true` in exactly
  one place and never written `false` anywhere. A *soft* reset explicitly does not clear it and
  says so:
  ```c
  if (g_motor_latched) {
          printk("MOTOR GATE: soft reset does NOT clear the window latch -- motors stay OFF "
                 "until the board is reset.\n");
  }
  ```
  Only a board reset / power cycle (BSS re-init) clears it.
- **Commit cannot happen by accident.** Two independent acts, neither of which does anything
  alone: `-DROSE_FLIGHT_COMMIT=1` at build time (without it `g_flight_commit_key` is *not in the
  image*, so `commit.gdb` fails with "No symbol"), and a debugger writing the specific constant
  `0x464C5921` at run time. [BUILT: `nm` on a default `esp-motors` image finds **no**
  `g_flight_commit_key`; on one built with `-DROSE_FLIGHT_COMMIT=1` it is present at
  `0000000080012ac4 B g_flight_commit_key`.] It is polled, never latched at boot, so it cannot survive a power
  cycle. A commit is refused once the window has already latched.

**Is 60 s right for a tethered attempt? No — recommend 20 s** (`-DROSE_COMMIT_MAX_MS=20000`).
Reasoning in 2.1.

#### Which images the gate covers

- **`flight_controller` — covered.** The gate lives in `main.cpp`.
- **`workloads/motor_duty` — NOT covered.** It is a standalone image with no control loop and no
  commit gate; `g_flight_commit_key`, `g_motor_latched` and `g_motor_committed` are all absent
  from it. It has its *own* independent budget/latch (`g_armed`, `g_latched`,
  `MOTOR_DUTY_BUDGET_MS`) and says so at boot. **Do not carry the belief "motors always latch off
  after 5 s" into that workload.**
- `workloads/motor_1..4`, `motor_safe` — different images again, with their own rules.

### 6.3 "Emergency stops are always available"

Ordered **fastest and least setup first**. This is the order to use them in.

| # | Stop | Latency | Setup needed | What the operator does | What they should SEE |
|---|---|---|---|---|---|
| 1 | **Pull the battery** | Immediate | **None** | Pull the LiPo connector | Motors stop instantly. Consoles keep printing if logic is USB-powered. [UNVERIFIED: which rails brown out depends on how the board is powered] |
| 2 | **Guard trips** (automatic) | ~150–170 ms | None | Nothing — it is automatic | `*** EMERGENCY CUTOFF *** <reason> limit exceeded ... motors OFF (reset to clear)`, and `flags=0x02[ESTOP ]` on the ESP. [BENCH: `held 141 ms over 5 samples`] |
| 3 | **Window expires** (automatic) | At the budget | None | Nothing | `MOTOR GATE: WINDOW EXPIRED after 5009 ms -- ALL FOUR MOTORS COMMANDED OFF AND LATCHED` [BENCH] |
| 4 | **Stop the frame stream** — unplug the uart1 ribbon, reset the FPGA, or halt the core | ≤ 125 ms (120 ms failsafe + 5 ms tick) | None for the ribbon/reset | Pull the ribbon or hit reset | ESP: `link=DOWN -- MOTORS OFF (failsafe)` and `cuts=` increments [BENCH: `cuts=1`] |
| 5 | **`estop.gdb`** | One control tick (~30 ms) **once attached** | **Debugger attached** | Run the script | `=== ESTOP LATCHED ===`, then `flags=0x02[ESTOP ]` on the ESP |
| 6 | **`stop.gdb`** | ~30 ms + 300 ms + halt | **Debugger attached** | Run the script | estop frame, then core halted, then `cuts=1` on the ESP |
| — | **Ground-station / dashboard ESTOP** | — | — | — | **DOES NOT EXIST IN THIS BUILD. Do not count on it.** See below. |

**THE REMOTE KILL DOES NOT EXIST, and it is the exact class of thing this audit is for: a control
that appears to work and does nothing.** Every `rose_cmd_*` entry point — `rose_cmd_estop`,
`rose_cmd_disarm`, `rose_cmd_reset` — is compiled only under `CONFIG_WIFI` [CODE]:

```c
#if defined(CONFIG_WIFI)
...
extern "C" void rose_cmd_estop(void) { g_estop = true; }   /* latched remote kill */
...
#endif /* CONFIG_WIFI */
```

`--mode esp-motors` does not enable `CONFIG_WIFI`, and `telem_wifi.c` is not in this checkout at
all, so none of those symbols is in the image. A dashboard that sends `ESTOP`, `DISARM`, `HOVER_Z`
or `PING` and shows an acknowledgement is being acknowledged **by the bridge, which fabricates the
ACK itself** — nothing reaches the flight controller. `hardware/flight/arm-enable.gdb` already
records the same fact from the other direction: with WiFi off, `--gc-sections` drops
`rose_cmd_reset` and `nm` cannot find it, which is why that script writes `g_arm_enabled` directly.

A merged `esp_bridge --mode motors` image implements a real ESP-side kill, but **it has never been
flashed** and is not what you are running. Treat a remote stop as not existing until you have
watched one cut a motor.

**Latency reasoning.** #2: `SAFE_DEBOUNCE_MS` 120 ms (40 ms for free-fall/impact) plus one control
tick to transmit — and the transmit is *not* delayed by the 50 Hz rate limiter, because a flags
change bypasses it: `if (flags == last_flags && now < next_ms) { return; }`. The code also
deliberately commands zero *before* it prints, since `printk` on this carrier busy-waits 30–60 ms.
#4: the ESP's own `ESP_MOTORS_TIMEOUT_MS` 120 plus `ESP_MOTORS_TICK_MS` 5.

**Do these still work on the offload path? Yes — and one of them did not used to.**

- `stop.gdb` zeroes four SiFive PWM comparators. **Under `--mode esp-motors` those writes are a
  no-op that looks like it worked**: `CONFIG_PWM` is off, `pwm_sifive_init()` never runs, and those
  addresses belong to a block connected to nothing. The script now says so in its own header, and
  it no longer relies on them: it latches `g_estop`, **resumes the core for 300 ms so the explicit
  zero-duty ESTOP frame actually goes out**, and only then halts. Halting first would have thrown
  away an explicit "off" in favour of a 120 ms timeout.
- `estop.gdb` resumes rather than parking at a prompt, which is correct on both paths — on the PWM
  path because a halted core cannot stop free-running comparators, and on the offload path because
  a running core is what *transmits* the explicit ESTOP frame.

**⚠ #5 and #6 REQUIRE THE DEBUGGER ALREADY ATTACHED.** Attaching OpenOCD and GDB from cold takes
tens of seconds. **A stop that needs 30 s of setup is not an emergency stop.** If you are not
already attached, your emergency stops are #1 and #4 — the battery and the ribbon. Both are
physical, both are instant, and both are why your hand belongs on the connector.

---

## Part 7 — What could not be verified from here

- **The thrust curve.** Never measured. The entire power argument rests on one in-flight
  observation (71 % hover) and an assumed square law.
- **Break-away on the ESP path.** 0.15 is an FPGA-path number, measured with different gate drive,
  possibly-inverted outputs, and different quantisation. Re-measure with `motor_duty`.
- **|a| at flight-like duty.** Only bench-duty vibration was measured (7.85–12.46 m/s²). Whether
  the accelerometer trust band holds at 75 % duty is unknown, which is why 4.3 asks you to watch
  for `acc180deg`.
- **Everything in Parts 2–5 as a sequence.** The individual behaviours are [CODE]/[BUILT]/[BENCH];
  the procedure as a whole has never been run.
- **Whether pulling the battery leaves the logic alive.** Depends on how the board is powered on
  the day.
- **The new boot banner lines** are in the ELF [BUILT] but have never been seen on a console.
- **The worst-case frame gap under telemetry.** 54 ms is computed (29 ms measured loop + 25.3 ms of
  polled write for a 292-byte telemetry pair at 115200), not observed. The ESP prints the real
  interval as `gap=`; **read it and compare.** If `gap` ever approaches 120 ms, telemetry is
  costing more than this arithmetic says.
- **The ESP's collective ceiling.** The change from a per-motor clamp to a collective cut compiles
  and its arithmetic was checked on the host, but no duty above the ceiling has been commanded on
  real hardware, so the cut has never actually fired on the ESP.

---

## Appendix — changes made by this audit

| File | Change |
|---|---|
| `backends/zephyr-chipyard-sw/samples/rose_flight_controller/src/main.cpp` | `#error` refusing `ROSE_THREADED=1` on a real target (the threaded blocks never call `safety_violation()`); boot banner now states whether `g_armed` is compile-time true, names the accel-tilt suspension band and what it does *not* affect, warns when `AUTOFLIGHT_MAX_DUTY` is below the measured hover duty, and prints this side's duty ceiling plus a pointer to the ESP's; corrected the `MOTOR_MAX_DUTY` comment (it is a scale, not a clamp) and the `MOTOR_BREAKAWAY_DUTY` comment (no ESCs exist — brushed motors on low-side SI2302 FETs, the hazard is thermal; and 0.15 is an FPGA-path number); corrected the `AUTOFLIGHT_MAX_DUTY` "hover~0.15" comment. |
| ...the same `main.cpp`, power | Added `MOTOR_DUTY_CEILING` — the bench path's anti-saturation ceiling, previously a hard-coded `1.0f`, so there is a duty **cap** that is not the `MOTOR_MAX_DUTY` **scale**. Default `1.0f`, so every existing build is bit-identical. Added a `DUTY CHAIN` boot banner printing the ceiling, the scale and the resulting worst-case duty, plus a loud `NO MEANINGFUL POWER CAP` line when the worst case exceeds 0.85. Added `g_guard_acc_skips`, printed as `accskip=`. `static_assert`s on both duty knobs. |
| `...rose_flight_controller/CMakeLists.txt` | Declared `MOTOR_DUTY_CEILING` as a cache variable and wired it into `target_compile_definitions`, so `RB_CMAKE_ARGS` reaches it. Corrected the `MOTOR_BREAKAWAY_DUTY` comment (no ESCs; the hazard is thermal). |
| `workloads/esp_motors/src/main.c` | The independent duty ceiling is now a **collective**, differential-preserving cut instead of a per-motor clamp — a clamp flattened four commands into four equal numbers (zero attitude torque) exactly when the vehicle was asking for the most thrust. Final per-motor clamp kept as a by-inspection invariant. `BUILD_ASSERT` on the percentage. Status line now carries `ceil=NN%`. |
| `workloads/esp_motors/CMakeLists.txt` | Declared `ESP_MOTORS_MAX_DUTY_PCT` and `ESP_MOTORS_TIMEOUT_MS` as cache variables and wired them into `target_compile_definitions`, because `RB_EXTRA_CFLAGS` is silently dropped for `esp32c6` targets and the ceiling could not otherwise be raised. |
| `hardware/flight/guard-check.gdb` | Prints `g_guard_acc_skips` and explains what a rising count means. |
| ...the same `esp_motors/src/main.c`, docs | Corrected the networking thread-priority note in the header. It claimed the net traffic-class threads run at `K_PRIO_COOP(0)` = −16 and recommended `CONFIG_NET_TC_THREAD_PRIO_CUSTOM`. Verified in-tree: with the default cooperative type `BASE_PRIO_RX = NET_TC_NUM_PRIORITIES − 1`, which Kconfig defaults to `NUM_COOP_PRIORITIES`, so the RX thread is **−1** (the lowest coop priority, already outranked), and `NET_TC_TX_COUNT` defaults to 0 so there is no TX thread. Turning `PRIO_CUSTOM` on without setting the bases defaults them to 0 and **creates** the −16 collision it was meant to prevent. Relative coop priority does not govern preemption anyway; the fix that works is `CONFIG_NET_TC_THREAD_PREEMPTIVE=y` + `CONFIG_SYSTEM_WORKQUEUE_PRIORITY=1`. Moot for this image, which has no networking. |
| `docs/tethered-flight-attempt.md` | This document. |

A note on one change that was made and then reversed during this audit, recorded so it is not
re-made by accident: the accelerometer trust band was briefly widened to 0.4–2.0 g and then put
back to 0.6–1.5 g. The reasoning for the reversal is 6.1 in full. The instrumentation that was
added alongside it — `g_guard_acc_skips`, printed as `accskip=` — was **kept**, because it is what
makes the narrow band an informed choice rather than a hopeful one.

Builds run on this machine, both exit 0:

```
# default (bench) FPGA build
./rb build flight_controller --target arty200t --config RocketArty200TDroneFullDDRConfig \
  --mode esp-motors --pristine always                       # RAM: 418176 B

# flight configuration
RB_CMAKE_ARGS="-DROSE_FLIGHT_COMMIT=1 -DMOTOR_MAX_DUTY=1.0f -DMOTOR_DUTY_CEILING=0.75f \
               -DROSE_COMMIT_MAX_MS=20000" \
  ./rb build flight_controller --target arty200t --config RocketArty200TDroneFullDDRConfig \
  --mode esp-motors --pristine always                       # RAM: 418432 B

# ESP, flight ceiling
RB_CMAKE_ARGS="-DESP_MOTORS_MAX_DUTY_PCT=80" \
  ./rb build esp_motors --target esp32c6 --pristine always  # sram0_0_seg: 54832 B (12.53%)
```

Negative tests, run deliberately to prove the interlocks are not decorative:

- `RB_CMAKE_ARGS="-DESP_MOTORS_MAX_DUTY_PCT=200"` → build **fails** with
  `static assertion failed: "ESP_MOTORS_MAX_DUTY_PCT is a percentage of full duty: (0, 100]"`.
  This also proves `RB_CMAKE_ARGS` genuinely reaches that source.
- `nm` on a **default** `esp-motors` image: **no** `g_flight_commit_key` (so `commit.gdb` fails
  with *"No symbol"* — the interlock working). On one built with `-DROSE_FLIGHT_COMMIT=1`:
  `0000000080012bc4 B g_flight_commit_key`.
- The `NO MEANINGFUL POWER CAP` string is present in an image built `-DMOTOR_MAX_DUTY=1.0f` with
  no ceiling, and **absent** (constant-folded out) from one built with `-DMOTOR_DUTY_CEILING=0.75f`.
- Guard symbols present in both: `g_guard_tilt_rad`, `g_guard_est_rad`, `g_guard_amag`,
  `g_guard_acc_skips`, `g_motor_latched`, `g_motor_committed`.

**The build tree was left in the DEFAULT (bench) configuration**, not the flight one. Cache
variables persist; if you build for flight, wipe with `--pristine always` when you go back.
