# Bench procedure: the tilt guard, the commitment gate, and every way to stop the motors

Written after the 2026-09-21 bench run in which the drone was rotated by hand to
roughly 90 degrees under `--mode esp-motors` and **nothing cut the motors**.
That failure and its root cause are in the first section; the procedure follows.

Nothing in this document has been run on hardware. Every claim below is read out
of the source or proven by a build, and the steps are ordered so the ones that
cannot spin a motor come first.

> **Partly superseded, later the same day.** Several of the steps below were
> subsequently run on this hardware. See section 4 of
> `docs/esp-motor-offload-handoff.md` for what was actually measured — the ESP
> failsafe (`cuts=1`), the tilt guard firing at 47.457 deg, the 5009 ms
> commitment-window latch, and the accelerometer trust band surviving props-on
> vibration (0 of 44 samples outside 0.6–1.5 g). **Section 5 of this document
> ("Not verified") is no longer accurate for those items.** One prediction below
> was also wrong: the window-expired flags byte is `0x10`, not `0x11`, because
> the sender clears ARMED as well as setting WINDOW.

---

## 1. What actually went wrong on 2026-09-21

The console showed, with the battery disconnected:

```
it=1020  roll=-0.949  pitch=-0.029  duty=[0.100 0.100 0.100 0.100]
```

and the ESP reported `flags=0x01` (ARMED) the whole way over.

**The estimator was not lagging and was not mis-scaled. It was right, and it was
misread.**

`state[3..5]` are **Rodrigues (Gibbs) parameters**, `r = q_xyz / qw` — see
`estimator.hpp` and both `get_state()` implementations, which divide the
quaternion vector part by `qw`. For a rotation of θ about one axis that is

```
r = tan(θ / 2)
```

so the reported `-0.949` is a true roll of `2·atan(0.949) = 1.518 rad = 87.0°`.
That is the ~90 degrees the operator applied.

What failed is that `safety_violation()` compared that Gibbs number against a
constant named `SAFE_MAX_TILT_RAD` as though it were radians. **1.0 in Gibbs
units is `tan(θ/2) = 1`, i.e. exactly 90.0°** — not the ~57° its own comment
claimed. The guard's real trip point was 90° and the operator reached 87°. It
missed by three degrees.

The codebase already contained the disproof of the "lag" theory. The flow
tilt-compensation in `read_sensor_frame()` derives `cos(tilt)` from *the Gibbs
state*:

```
ct = (1 - ga² - gb² + gc²) / (1 + ga² + gb² + gc²)
```

Put `ga = 0.949` through it and it returns `0.052`, which is `cos(87.0°)`. Two
independent routes, same answer.

The same misreading was in the arm gate: `ARM_MAX_TILT_RAD` `0.10` "(~5.7 deg)"
was really `2·atan(0.10) = 11.4°`.

---

## 2. What changed

### 2.1 A tilt guard that does not go through the estimator

`accel_envelope()` in `main.cpp` reads the accelerometer directly. Gravity is a
true attitude reference with no filter, no convergence, and no parameterisation
to misread. Three tests, thresholds carried over from
`integration/zephyr/safety/motor_guard.h`:

| test | trips at | dwell | why |
|---|---|---|---|
| accel tilt | > 40° from vertical | 120 ms | a person picking the drone up passes 40° almost immediately |
| free fall | \|a\| < 3.92 m/s² (0.4 g) | 40 ms | **tilt structurally cannot catch a drop** — a drone dropped flat stays flat all the way down |
| impact | \|a\| > 29.43 m/s² (3 g) | 40 ms | kills the motors before it bounces into the next thing |

Neither free fall nor impact existed anywhere in the flight controller before.

The direction test is skipped while `|a|` is outside 0.6 g … 1.5 g, because the
direction of a vector that is not gravity says nothing about attitude; the two
magnitude tests own that case. When it is skipped, the published tilt becomes
π rather than a stale value, so the arm gate refuses rather than being permitted
by an old reading.

**Known limitation, stated so nobody over-trusts it.** The accelerometer reads
*specific force*. At rest or hovering that is gravity and the tilt is true. In a
hard translational acceleration the thrust vector dominates and body-frame
horizontal accel stays small even at a real tilt, so this guard *under*-reports
during aggressive flight — which is exactly where the estimator-based check is
strong. They are a complementary pair on purpose.

### 2.2 The estimator checks now compare radians to radians

`tilt_rad_from_gibbs()` converts once, and every attitude limit uses it.
`SAFE_MAX_TILT_RAD` is now **0.70 rad = 40°**, and genuinely 40°.
`ARM_MAX_TILT_RAD` is now **0.20 rad = 11.5°**, which is the angle the arm gate
has actually been enforcing all along — only its name changed, so arming does
not silently get harder on the eve of a flight test.

The arm gate now requires **both** references to agree the frame is level, so an
unconverged or misread estimator cannot permit arming on its own.

### 2.3 Debounce in milliseconds, not iterations

`SAFE_DEBOUNCE_ITERS` was 15 consecutive iterations. That is **20 ms under the
PID cascade (1.3 ms/iter) and 525 ms under TinyMPC (35 ms/iter)** — a factor of
26 between two builds of the same guard, and the offload run that failed was the
slow one. Now `SAFE_DEBOUNCE_MS` 120 ms (40 ms for free fall and impact), plus a
2-sample floor to reject a single garbage reading on a fast loop.

### 2.4 On trip, zero is commanded FIRST and reported afterwards

`send_control()` is now called immediately after `g_estop` latches, before the
`printk` and before `flightlog_flush()`. Previously the motors kept driving
through a busy-waiting 115200 console print (30–60 ms measured), a flash flush,
and a full controller solve (~30 ms) before anything reached the actuator.

### 2.5 The commitment gate

Default, with no special flags: **motors may run for 5 s from the first instant a
non-zero duty would be written, then all four latch OFF until the board is
reset** — whatever the controller asks for, whatever the arm gate says.

This matters because in every non-autoflight mode `g_armed` is a compile-time
`true`, so `--mode esp-motors` is armed from its first control tick and the
controller asks for thrust immediately. The only previous bound was
`ROSE_ACTUATE_TIMEOUT_MS`, and **its default is 0 = disabled** (set in
`CMakeLists.txt`; there is no `#ifndef` fallback in the source, and no `--mode`
overrides it). The default build had no time bound on motor output at all.

Committing takes **two independent acts, and neither alone does anything**:

1. **Build time** — `RB_CMAKE_ARGS="-DROSE_FLIGHT_COMMIT=1"`. No `--mode` sets
   it. Without it `g_flight_commit_key` **is not in the image** — verified with
   `nm`: absent from the uncommitted build, `B g_flight_commit_key` present in
   the committed one. `commit.gdb` then fails with "No symbol", and that failure
   *is* the interlock.
2. **Run time** — `hardware/flight/commit.gdb` writes one specific constant
   (`0x464C5921`, "FLY!") from a tethered debugger. Not 1, not anything
   uninitialised RAM or a stray store can produce. It is polled, never latched at
   boot, so a power cycle returns the board to uncommitted.

Committed, the budget becomes 60 s — a commitment is permission for *a flight
test*, not permission forever.

### 2.6 "Off" is transmitted, never implied by silence

On the offload path every disarmed / estopped / timed-out / inhibited /
window-expired state still sends a frame with duty 0 and `ARMED` clear, and a
flags change bypasses the 50 Hz rate limiter so a stop goes out on the next
control tick. Two new wire flags, `WINDOW` (blocking) and `COMMIT`
(informational), and the receiver's reject set is now
`MOTOR_LINK_FLAGS_BLOCK` in the shared header so the two ends cannot diverge.
`motors_shutdown()` sends three explicit zero frames when the control loop ends,
instead of letting the run trail off into the receive timeout.

---

## 3. Every way to stop the motors, fastest first

| # | mechanism | where | trigger | how fast | works on `esp-motors`? |
|---|---|---|---|---|---|
| 1 | **Pull the battery** | the pack | your hand | instant | **yes — the only real kill** |
| 2 | ESP duty ceiling | `esp_motors/src/main.c` | always | per frame | yes (25 %, independent of the FPGA) |
| 3 | Envelope guard → `g_estop` | `main.cpp` | tilt 40°, free fall, impact, rate, velocity, height, battery | 40–120 ms dwell + one tick | **yes** — latches `g_estop`, which zeroes duty and sets the ESTOP flag |
| 4 | ESP receive failsafe | `esp_motors/src/main.c` | no valid frame for 120 ms | 120 ms + 5 ms tick | **yes — halt the core or unplug the ribbon**; measured `cuts=1` |
| 5 | Commitment window | `main.cpp` | 5 s of motor-live time (uncommitted) | one tick | yes |
| 6 | `estop.gdb` | `hardware/flight/` | operator | 1–2 s (JTAG attach) | yes |
| 7 | `stop.gdb` | `hardware/flight/` | operator | 1–2 s + 0.3 s + 120 ms | yes, **but see below** |
| 8 | Arm gate / `g_armed` | `main.cpp` | autoflight only | one tick | yes (no-op in non-autoflight: `g_armed` is const true) |
| 9 | `ROSE_MOTORS_INHIBIT=1` | build | build time | n/a | yes |
| 10 | `ROSE_ACTUATE_TIMEOUT_MS` | build | N ms after boot | one tick | yes — **but defaults to 0 = DISABLED** |

**`stop.gdb` was misleading and is now fixed.** It zeroes the four SiFive
comparators, which is the *only* thing that helps on the PWM path (halting the
CPU does not stop that block). Under `--mode esp-motors` the FPGA drives no gate
at all, `CONFIG_PWM` is off, and those writes touch a block connected to nothing
— **a no-op that looks like it worked**. It now latches `g_estop`, *resumes for
300 ms so the offload path actually transmits an explicit zero-duty ESTOP
frame*, and only then halts (which is itself the second stop, via the ESP
failsafe). `estop.gdb`'s "halting the core is NOT a stop" warning is now marked
as PWM-path-only.

---

## 4. Ordered bench procedure

### Part A — battery DISCONNECTED. None of these can spin a motor.

1. Build both images.

   ```bash
   cd /scratch2/loren/riskybird-dev-setup && source ./rb-env.sh
   ./rb build flight_controller --target arty200t \
       --config RocketArty200TDroneFullDDRConfig --mode esp-motors
   ./rb build esp_motors --target esp32c6
   ```

2. Prove the build is uncommitted. This must print nothing:

   ```bash
   /home/rose/zephyr-sdk-0.17.0/riscv64-zephyr-elf/bin/riscv64-zephyr-elf-nm \
     build/zephyr/arty200t/flight_controller/esp-motors/zephyr/zephyr.elf \
     | grep g_flight_commit_key
   ```

3. Flash and run. Confirm these lines on the FPGA console **before** anything
   else:

   ```
   flight_controller: ENVELOPE GUARD -- accel tilt >40 deg, free-fall <3.920 m/s2, impact >29.430 m/s2 (no estimator in that path)
   flight_controller: ENVELOPE GUARD -- est tilt >40 deg, rate >10 rad/s, vel >2.500 m/s, height >2.000 m
   flight_controller: ENVELOPE GUARD -- dwell 120 ms (40 ms for free-fall/impact), min 2 samples; ACTIVE ONLY WHILE ARMED
   flight_controller: ENVELOPE GUARD ARMED -- watching accel tilt, free-fall, impact, est tilt, rate, velocity, height
   flight_controller: COMMIT GATE -- UNCOMMITTED BUILD: motors latch OFF 5000 ms after they first spin, and CANNOT be committed at run time.
   ```

4. **Prove the guard can SEE.** This is the step that was missing before. Watch
   the telemetry line's new field while tilting the frame by hand:

   ```
   ... duty=[...] tilt=est0deg/acc0deg |a|=9.810
   ```

   Tilt to ~45°: `acc` must follow to ~45. Tilt past 40° and hold ~0.2 s — the
   console must print

   ```
   flight_controller: *** EMERGENCY CUTOFF *** accel-tilt limit exceeded: measured 52.000 deg accel-tilt, held 131 ms over 5 samples; motors OFF (reset to clear)
   ```

   `hardware/flight/guard-check.gdb` reads the same values over JTAG if the
   console is not convenient.

5. Drop-test the free-fall path with the drone **in your hand**, a 10 cm dip is
   enough: expect `free-fall` in the cutoff line with `measured ... m/s2 |a|`.

6. Prove the ESP failsafe: halt the core (`stop.gdb`). The ESP console must show
   `cuts=1` and `link=DOWN -- MOTORS OFF (failsafe)` within ~125 ms.

7. Prove the commitment window: let it run. Within 5 s of the first non-zero
   duty the console must print

   ```
   MOTOR GATE: WINDOW OPEN -- motors are live; 5000 ms budget starts now (uncommitted)
   MOTOR GATE: MOTORS LIVE 0/5000 ms (uncommitted)
   ...
   MOTOR GATE: WINDOW EXPIRED after 5003 ms -- ALL FOUR MOTORS COMMANDED OFF AND LATCHED (board reset to clear)
   ```

   and the ESP console must show `flags=0x11[WINDOW-EXPIRED]` with duty 0.

### Part B — battery connected, **props off**. These CAN spin motors.

8. Repeat step 4 with the battery in. Confirm the motors stop the instant the
   cutoff line prints, and that the ESP shows `flags=0x03[ESTOP]` / duty 0.

9. Repeat step 6 with the battery in. All four must stop within ~125 ms.

10. Confirm the 5 s window ends the run with motors off and latched.

### Part C — committing to a flight test. Do this last, deliberately.

11. Rebuild, committed:

    ```bash
    RB_CMAKE_ARGS="-DROSE_FLIGHT_COMMIT=1" ./rb build flight_controller \
        --target arty200t --config RocketArty200TDroneFullDDRConfig --mode esp-motors
    ```

12. Boot it. It is still **uncommitted** — the banner says so. Nothing has
    changed about how it behaves yet.

13. Standing at the drone, ready: run
    `hardware/flight/commit.gdb`. Wait for

    ```
    MOTOR GATE: *** COMMITTED TO A FLIGHT TEST *** -- motor budget is now 60000 ms (was 5000 ms). Props are live.
    ```

    and `flags=0x21[ARMED COMMITTED]` on the ESP console.

14. Fly. Stop with the battery, `estop.gdb`, or by halting the core.

---

## 5. Not verified

Nothing here has run on hardware. Specifically unproven:

- every console line quoted above is the format string, not a capture;
- the 40° accel-tilt threshold has not been checked against a protractor on this
  airframe, and the BMI088's mounting bias has not been measured;
- the `0.6 g … 1.5 g` trust band has not been checked against the vibration a
  spinning airframe actually produces — if it is noisier than that, the tilt test
  will silently skip during flight and only free fall and impact remain. **Watch
  `|a|` in the telemetry line with motors spinning before trusting the tilt test
  in flight.**
- `stop.gdb`'s `shell sleep 0.3` assumes the GDB host has a POSIX `sleep`.
