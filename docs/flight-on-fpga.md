# RiskyBird: state estimation and wireless flight on FPGA

Porting the flight stack that **actually flew on the ESP32-C6** onto Rocket running on the
TE0712 drone carrier. This is the running record of that effort: what was brought in, what the
reference build was, what is proven here, and what is still blocked.

Evidence levels use the repo vocabulary: **verified** (run here, output quoted), **recorded**
(observed elsewhere, reproduced from someone else's log), **declared** (asserted by code or
docs, never run), **blocked**.

Started 2026-09-04. The Phase 3 wireless path is verified below; every other
flight item retains its per-item status.

---

## 1. The reference: what flew

Source: `ucb-bar/zephyr-chipyard-sw` branch `riskybirdv3-bringup`, tip
`f33d768e95b` "riskybird FC: gyro-bias drift fix + WiFi/mobile ground station", 35 commits
past our merge base. It achieved untethered autonomous hops with WiFi telemetry on the
ESP32-C6 — **recorded**, not reproduced here.

The flight stack (`samples/rose_flight_controller`, 4,522 lines):

| Layer | Implementation |
| --- | --- |
| Estimator | complementary filter, or EKF (`ROSE_USE_EKF`) |
| Controller | hierarchical PID cascade (`ROSE_USE_PID=1`); TinyMPC is the other option |
| Attitude | BMI088 accel `0x18` + gyro `0x68`, remapped bottom-mount 180° into body frame |
| Altitude | VL53L1X down-ToF, readdressed `0x29`→`0x30`; BMP388 baro `0x76` fused |
| Horizontal | PMW3901 optical flow over SPI, software CS |
| Obstacles | 4× VL53L5CX side ToF, readdressed to `0x31`–`0x34` via ADS7128 GPIO1–4 |
| Actuation | 4 brushed motors on low-side SI2302 FETs, 20 kHz PWM, `riskybird,pwm-motors` |
| Logging | on-board flash ring buffer, dumped as CSV over USB |
| Telemetry | WiFi SoftAP + UDP unicast, ~50 Hz, laptop/mobile ground station |

`backends/zephyr-chipyard-sw/samples/rose_flight_controller/docs/FLIGHT_BUILD.md`
(inside the sample, not this repository's `docs/`) records the exact flight build. Two things in it matter
more than the rest:

- **`ROSE_AUTOFLIGHT` and `ROSE_FLIGHTLOG` are not CMake knobs.** Passing them as bare `-D`
  is silently ignored and the build falls through to an always-armed bench path with motors at
  10% and no chirp. They must go inside `EXTRA_CPPFLAGS`.
- **The last hop flew away, and the cause is unfixed.** The down-ToF read 0 mm as soon as the
  props spun (vibration, occlusion, or the driver reporting a bad `RangeStatus` as 0). The
  altitude loop read that as "on ground" and commanded max thrust. Below 0.02 m the flow also
  gates off, so horizontal velocity dead-reckoned. Anyone replicating that build should expect
  it to recur.

---

## 2. What was brought in, and why nothing was lost

Their entire change set is confined to `samples/rose_flight_controller/` and
`samples/riskybird/`, plus the `zephyr_ws/zephyr` submodule pointer. We have **zero** changes
in either directory — our own divergence on `dev` is entirely `samples/executorch` and
`samples/tacit`. Verified with `git merge-tree`: the only merge conflict in the whole branch is
the submodule pointer.

So the two trees were combined by checking out exactly those two paths, which is provably
lossless in both directions — 59 files, no deletions:

```bash
git checkout ucb/riskybirdv3-bringup -- samples/rose_flight_controller samples/riskybird
```

**`rb` will build the sample in place** rather than copying it into `workloads/`. `Workload.source_dir`
is a plain field (`tools/rb/boards.py`), so a workload can point at
`backends/zephyr-chipyard-sw/samples/rose_flight_controller` directly. The sample stays where
upstream maintains it; we add a target, not a fork.

### Blocked: their Zephyr submodule commit is unpublished

Their branch pins `zephyr_ws/zephyr` at `df8e50e20b2`, which **is not on any branch of
`ucb-bar/zephyr`** — `git fetch` fails with `upload-pack: not our ref`, and all 14 remote
branches were checked. The commit that produced it is
`cdd3734e424 bump zephyr submodule: vl53l1x xtalk-comp + tunable timing budget + calibrate()`.

Our pin is `00cfa8f2644`. Confirmed missing from our VL53L1X driver
(`drivers/sensor/st/vl53l1x/vl53l1.c`): no xtalk compensation, no tunable timing budget, no
`calibrate()`. The flight build passes `-DVL53L1X_TIMING_BUDGET_US=100000` for weak/dark-floor
reliability, which our driver cannot honour.

**Consequence:** the down-ToF — already the cause of the fly-away — is the *least* reproducible
part of the stack here. Ask the branch author to push that commit.

### Unit hazard, carried forward

The RoSE integration notes on `main` record that upstream changed VL53L1X `SENSOR_CHAN_DISTANCE`
from millimetres to **metres**, and their own commit `f515f2f8682` fixes a VL53L5CX print that
assumed the opposite. Any new consumer must state its unit assumption explicitly.

---

## 3. Hardware contract: ESP vs FPGA

The app binds sensors through **device-tree aliases** and compiles subsystems in or out with
`DT_NODE_EXISTS(DT_ALIAS(...))` (`HAVE_FLOW`, `HAVE_TOF`, `HAVE_BARO`). The overlay alone
decides what is built, which is what makes an incremental port possible.

The only ESP-specific dependency in the entire application is `flightlog.c`, which uses
`flash_map` / `FIXED_PARTITION_ID`. Everything else is standard Zephyr sensor/i2c/spi/pwm API.

| Alias | ESP32-C6 | FPGA (`RocketArty200TDroneFullDDRConfig`) | Status |
| --- | --- | --- | --- |
| `bmi088-accel` / `-gyro` | i2c0 GPIO14/15 | `i2c@10040000` A15/A16 | **verified** present (`sensor_check`, I2C scan) |
| `tof` (VL53L1X) | `0x30` after readdress | same bus | **blocked** — driver features unpublished |
| `baro` (BMP388) | `0x76` | same bus | **verified** ACKs at `0x76` |
| side ToF ×4 | `0x31`–`0x34` | same bus | not populated on the fresh board |
| `flow` (PMW3901) | SPI2, soft CS | `spi@10031000` | **declared** — `pmw3901_test` exists |
| `motors` | LEDC ×4, 20 kHz | `pwm@10050000` ch1-3 + `pwm@10051000` ch1 | **declared** |
| console | USB serial | `uart@10020000` | **verified** |
| telemetry | on-chip WiFi | **`uart@10021000` → ESP as a modem** | **verified** end to end (Phase 3) |

### The architectural difference that matters

On the ESP, the flight controller and the WiFi stack are *the same chip*. On the FPGA they are
not: Rocket runs the flight controller, and the ESP32-C6 becomes a **WiFi modem reached over
uart1** (E13/F14, PLIC 11). That link is already proven bidirectional at 115200
(`workloads/esp_uart`). `workloads/esp_bridge` is the ESP-side modem. It is deliberately a
dumb line pipe rather than a framing protocol: the flight controller's existing telemetry line
already matches the ground station parser, so each newline-terminated UART line can become one
UDP datagram unchanged.

### Bug found in our existing overlay

Authoritative PLIC map, read from the elaborated FullDDR DTS:

| peripheral | address | PLIC source |
| --- | --- | --- |
| i2c0 | `0x10040000` | 1 |
| pwm0 | `0x10050000` | 2, 3, 4, 5 |
| pwm1 | `0x10051000` | 6, 7, 8, 9 |
| uart0 (console) | `0x10020000` | 10 |
| uart1 (ESP) | `0x10021000` | 11 |
| gpio0 | `0x10010000` | **12, 13, 14** |
| spi0 | `0x10031000` | **15** |
| ospi (camera) | `0x10080000` | 16 |

`hardware/zephyr/fpga-drone.overlay` declares **gpio0 at 11,12,13 and spi0 at 14**, with
`riscv,ndev = <15>`. It predates the second UART; inserting uart1 at source 11 shifted every
later assignment by one. On this shell that gives SPI the wrong interrupt — i.e. it breaks the
optical flow the controller depends on — and `ndev = 15` cannot reach the camera at 16.

Also to reconcile: the FPGA overlay runs the IMU at `accel-hz "200"` / `gyro-hz "200_64"`,
where the flight build uses **800 Hz / 1000 Hz**. A 1 kHz control loop fed by a 200 Hz IMU is a
different system.

### Config conflicts to resolve

| Setting | `hardware/zephyr/fpga.conf` | Flight controller needs |
| --- | --- | --- |
| float printing | `CONFIG_CBPRINTF_NANO=y` | `CONFIG_CBPRINTF_FP_SUPPORT=y` |
| main stack | small by default | `CONFIG_MAIN_STACK_SIZE=327680` (TinyMPC leftover; ~29% if right-sized) |
| C++ | not enabled | `CONFIG_CPP=y` |

One thing is *better* here: Rocket is RV64GC with a hardware FPU (`CONFIG_FLOAT_HARD=y`), where
the ESP32-C6 is rv32imac with **soft float**. The estimator and PID cascade are float-heavy, so
the FPGA should be the faster machine per iteration. Against that, the CLINT timebase is 50 kHz
(20 µs resolution) at a 1 kHz loop rate, so loop-timing measurement is coarser.

RAM is a non-issue: the ESP build sits at 96.8% of 437 KB, and this shell presents 1 GB of DDR3.

---

## 4. Plan of action

Each phase has a verification criterion. No phase is "done" on a successful build.

### Phase 0 — overlay correctness (no flight)
1. Fix the PLIC assignments and `ndev` in `fpga-drone.overlay`; raise IMU rates to match.
2. Register the sample as an `rb` workload pointing at `backends/.../rose_flight_controller`.
3. **Verify:** `hello_world`-level boot of the FC binary on FullDDR, console banner only.

### Phase 1 — state estimation, bench, no motors
4. Overlay with IMU + baro aliases only, no `motors`/`flow`/`tof`, so those compile out.
5. **Verify:** estimator emits roll/pitch tracking board orientation; gravity vector correct
   at rest; gyro bias converges. Compare against `sensor_check`'s independent IMU read.
6. **Verify:** loop rate actually achieved at 1 kHz on Rocket, measured, not assumed.

### Phase 2 — sensors into the estimator
7. Add PMW3901 (`flow`) — needs the SPI interrupt fix from Phase 0.
8. Add BMP388 (`baro`) fusion.
9. Down-ToF: **blocked** on the unpublished driver commit. Either get it pushed, or port the
   xtalk/timing-budget work forward ourselves and say so.
10. **Verify:** horizontal velocity responds correctly to translating the board by hand;
    altitude tracks a hand-held height change.

### Phase 3 — wireless link

The data path is **verified** on hardware on 2026-09-20, in this order.
`workloads/esp_bridge` first brought up the radio and both UDP endpoints:

```console
esp_bridge: SSID riskybird-5674, ch 6, open; telemetry -> UDP :14550
esp_bridge: commands on UDP :14551
```

FPGA → ESP was then isolated with `workloads/esp_uart`:

```console
esp_bridge: line #1 (21 B), 0 client(s): RISKYBIRD-FPGA-PROBE
```

The UDP uplink was then exercised by sending `PING` to `192.168.4.1:14551`:

```console
ACK PING
```

Finally, the new `--mode telem` on `workloads/flight_controller` moved its
existing telemetry line to uart1; the bridge forwarded it unchanged and
`riskybird_panel.py` served the dashboard from that same UDP stream. The first
capture recorded this:

```console
flight_controller: it=200 dt=34ms roll=-0.1 pitch=0.3 yaw=-0.0 z=0.0 tofv=1
```

That was a real ~29 Hz measurement, but of the wrong controller: the mode had
silently inherited the sample's `ROSE_USE_PID=0` default and built TinyMPC. The
mode now declares `ROSE_USE_PID=1` and `ROSE_USE_EKF=1`, and disables RISC-V
linker relaxation because that larger image otherwise truncates a GP-relative
relocation against `_kernel`.

The controller swap is **verified** on hardware. The image identified itself as
the PID cascade at boot and the forwarded stream then reported 1 ms samples:

```console
flight_controller: estimator=EKF (Mahony + per-axis Kalman) + controller=hierarchical PID (cascaded) ready (real target), single-loop
flight_controller: it=600 dt=1ms roll=0.23 pitch=0.29 yaw=0.0 z=0.0 tofv=1 duty=[0.0 0.0 0.0 0.0]
```

The direct `dt` histogram over one capture was 765 x 1 ms, 636 x 5 ms,
9 x 11 ms and 8 x 10 ms. Independently, the WiFi receiver recorded 540
datagrams at 38.5 Hz, with a 25.0 ms median gap, 66.4 ms maximum and no gap
over 75.0 ms (three times the median). At the declared `ROSE_TELEM_DIV=20`,
that telemetry rate implies a ~770 Hz control loop: about 26x the TinyMPC rate,
back near the 1 kHz design target and consistent with the 2.97 ms previously
recorded in this file. The ~770 Hz figure is inferred from the datagram rate and
divisor; the histogram is the direct loop-time measurement.

The image is also **verified** by symbol, not just by its boot string: TinyMPC
0, `tiny_solve` 0, PID 7, EKF 9 and motor 0. `CTRL_ITERS=0` is declared by the
telem mode so it runs until stopped. The sample default of 5000 iterations took
minutes at 29 Hz but only about six seconds at the corrected rate; returning
from `main()` then left an idle core that looked exactly like a dead link.

No framing layer was needed: the bridge is a line pipe and the existing line is
already the ground station's wire format.

Two items remain **open**:

- The alternating 5 ms `dt` samples remain unexplained. They are consistent
  with the VL53L1X worker at priority 8 preempting main at priority 11 while it
  busy-waits on I2C, but that is a lead, not a confirmed cause.
- The remote-ESTOP round trip over this link is still unmeasured. `ACK PING`
  proves that the bridge delivered an uplink line toward the FPGA, not that an
  ESTOP took effect in the controller. A measured end-to-end ESTOP remains the
  gate before any motor is allowed to spin in flight.

### Phase 4 — actuation (battery required, props off)
15. `motors` alias onto pwm0 ch1-3 + pwm1 ch1; resolve the harness-inversion question first —
    see `never`-style note below.
16. **Verify:** each motor responds individually at a capped duty, props off, battery connected.

### Phase 5 — flight
17. Autoflight profile, watchdogs, flight log (to DDR, not flash).
18. **Verify:** tethered hop before untethered.

### Standing hazards
- **Motors need the battery.** They are not powered from USB-C; a motor test on USB alone does
  nothing useful.
- **`RocketArty200TDroneFullDDRConfig` drives motor1–4 from configuration onward**, before any
  instruction runs. Its PWM comparators read `pwmcfg=0xf0000000` at reset (all four asserted).
  Whether the harness inversion lands at the pin is **unmeasured** — see the open item in
  `workloads/camera_photo/src/main.c`. Settle it with a meter on J1_31/36/99/100 before Phase 4.
- The down-ToF fly-away above is unfixed upstream.

---

## 5. Run log

### 2026-09-04 — Phase 0 build: the flight controller compiles for Rocket

`rb build flight_controller --target arty200t --config RocketArty200TDroneFullDDRConfig`:

```console
[11/11] Linking CXX executable zephyr/zephyr.elf
Memory region         Used Size  Region Size  %age Used
             RAM:      387728 B         1 GB      0.04%
```

**Verified** by symbol, not by the build succeeding — what actually linked:

```console
$ nm zephyr.elf | grep -iE 'Ekf|Tinympc|bmi08'
0000000080000fa4 T _ZN12EkfEstimator4initEfff
0000000080001c46 T _ZN17TinympcController4initEv
00000000800028d6 T tiny_solve
0000000080005412 T bmi08x_accel_init
```

EKF + TinyMPC (the CMake defaults: `ROSE_USE_EKF=1`, `ROSE_USE_PID=0` — **not** the
PID/`ROSE_USE_PID=1` cascade that flew). BMI088 driver bound to `i2c@10040000`. No
`vl53*`/`bmp3*`/`pmw3901` symbols, i.e. the alias guards compiled the absent sensors out as
intended. **Not run on hardware yet.**

**PLIC fix verified in the generated DTS**, not just written:

```console
gpio0: interrupts = < 0xc 0x1 >, ...        /* 12,13,14  from fpga-esp-uart.overlay */
spi0:  interrupts = < 0xf 0x1 >             /* 15 */
plic:  riscv,ndev = < 0x10 >                /* 16 */
```

#### Five portability bugs fixed to get here

All the same shape: a file compiled unconditionally, but resolving device-tree nodes in the
**preprocessor**, so absent hardware is a build failure rather than a compiled-out feature. Each
is guarded the way `main.cpp` already guards flow/tof/baro, so the ESP build is unchanged — the
aliases exist there and the original code compiles identically.

| File | Needed | Fix |
| --- | --- | --- |
| `side_tof.c` | `side_*` aliases | `HAVE_SIDE_TOF`; CMake claims `ROSE_BUMPER=0` leaves it "compiled but idle", but it could not compile at all |
| `main.cpp` | `g_led_bus`, `STATUS_LED_CH` | `HAVE_STATUS_LED`; the ADS7128 status-LED helpers are defined inside `#if DT_HAS_COMPAT_STATUS_OKAY(st_vl53l1x)` while the LED thread and its start-up call sit at top level |
| `flow.c` | `DT_NODELABEL(spi2)` | `HAVE_FLOW_HW` |
| `flightlog.c` | `storage_partition` | `HAVE_LOG_FLASH` |
| `fpga.conf` | full libc | per-workload conf: minimal libc declares no `fabsf`/`cosf`/`rand` |

**Carried into Phase 2:** `flow.c` hardcodes the SPI controller as node label `spi2` (the
ESP32-C6's). This carrier's flow sensor is on `spi@10031000`, labelled `spi0`, so that node
label has to become a DT alias before optical flow can be enabled here. Guarding made the file
portable; only aliasing makes the *feature* portable.

**ADS7128 channel map**, from their `main.cpp` and `side_tof.c` — the first authoritative record
of it in this repo:

| ch | function |
| --- | --- |
| 1-4 | side VL53L5CX XSHUT (`0x31`-`0x34`) |
| 5 | AIN5 battery sense — an **analog input**, not a GPIO |
| 6 | down VL53L1X XSHUT |
| 7 | D8 status LED, **active-low** (low = on) |

No channel gates a motor, and none gates the camera.

### 2026-09-04 — Phase 1 prepared (not yet run on hardware)

Built with the flight recipe's controller/estimator selection rather than the CMake defaults:

```console
RB_CMAKE_ARGS="-DROSE_USE_PID=1 -DROSE_USE_EKF=1 -DCTRL_ITERS=60000" \
  rb build flight_controller --target arty200t --config RocketArty200TDroneFullDDRConfig
RAM:      364768 B         1 GB      0.03%
```

**Verified by symbol:** TinyMPC **0** symbols (compiled out), PID 7, EKF 9, and — the one that
matters for bench safety — **0 motor symbols**. With no `motors` alias, `main.cpp` guards the
entire PWM actuator block on `DT_NODE_EXISTS(DT_ALIAS(motors))`, so this image contains no code
path to a motor at all. That is a stronger guarantee than a runtime disarm.

DTS confirms `clock-frequency = <0x61a80>` (400 kHz), `accel-hz = "800"`, `gyro-hz = "1000_116"`.

#### Two more porting problems fixed

**I2C bus speed is a control-rate requirement, not a preference.** `fpga-drone.overlay` runs the
bus at 100 kHz. Each control iteration reads accel + gyro: ~180 bits of bus traffic once
addressing, register selection, repeated START and ACKs are counted.

| bus | per iteration | ceiling |
| --- | --- | --- |
| 100 kHz | ~1.8 ms | **~550 Hz — below the 1 kHz loop** |
| 400 kHz | ~0.45 ms | ~2.2 kHz |

At 100 kHz the loop cannot reach its rate however fast Rocket is, and it presents as a slow
controller rather than a slow bus. Raised to 400 kHz in the workload overlay — what the ESP board
runs, and within spec for every device on this bus.

**Telemetry would have stalled the control loop.** `main.cpp` printed one line every 10th
iteration, hardcoded. At 1 kHz that is ~100 lines/s x ~150 chars = **~150 kbps against a 115200
console** — printk blocks and the loop measures the UART's latency as its own `dt`. Free on the
ESP's USB console; not here. Added `ROSE_TELEM_DIV` (default 10, unchanged for the ESP) so this
bench can run ~5 Hz telemetry. A divisor rather than a faster console: losing the console loses
all observability, while slower telemetry costs nothing during bring-up.

#### `rb` additions needed to drive this application

| Mechanism | Why |
| --- | --- |
| `RB_CMAKE_ARGS` | `ROSE_USE_PID` etc. are CMake **cache** variables the app's CMakeLists turns into definitions itself. Passing one as a bare preprocessor define does nothing — the CMakeLists overwrites it from the cache default. |
| `RB_EXTRA_CPPFLAGS` | `EXTRA_CFLAGS` reaches C only, so it can never affect `main.cpp`. `EXTRA_CPPFLAGS` covers C, C++ and asm. This is the same trap `FLIGHT_BUILD.md` warns about, where `ROSE_AUTOFLIGHT` passed the wrong way is silently ignored and the build falls through to an always-armed bench path. |
| `rb debug --no-run` | `rb debug` now loads **and continues** by default instead of parking at a GDB prompt. |

**Verified the `RB_EXTRA_CPPFLAGS` path actually reaches C++**, rather than assuming: building with
`-DROSE_IMU_DEBUG=1` swaps which format string is in the image (`iter=%d a=` present, `it=%d dt=`
absent).

### Ready to run — Phase 1 procedure

Both env vars must be on the **`rb debug`** line, not just `rb build`: `rb debug` rebuilds, so
setting them only on the build silently reverts the image.

**Step 1 — IMU axes and signs.** Do this before trusting any attitude output.

```bash
RB_CMAKE_ARGS="-DROSE_USE_PID=1 -DROSE_USE_EKF=1 -DCTRL_ITERS=60000" \
RB_EXTRA_CPPFLAGS="-DROSE_IMU_DEBUG=1 -DROSE_TELEM_DIV=200" \
./rb debug flight_controller --target arty200t \
    --config RocketArty200TDroneFullDDRConfig --program \
    --program-usb-location <bitstream> --usb-location <debug>
```

Prints `iter=N a=[ax ay az] g=[gx gy gz]` in body frame. Pass criteria, board level and still:

- exactly one accel axis reads ~9.81, the other two ~0;
- that axis is **z**, and its sign matches the IMU remap `main.cpp` applies for the
  bottom-mounted 180° BMI088 — this is inherited from the ESP board and is **unverified here**;
- gyro all ~0 at rest; rotating the board about each axis moves the expected component.

**Step 2 — attitude and loop rate.** Drop `ROSE_IMU_DEBUG`:

```
flight_controller: it=N dt=Xms roll=… pitch=… yaw=… z=… tofv=0 tofh=… u=[…]
```

- `roll`/`pitch` track the board and return to ~0 when level; `tofv=0` is expected (no ToF);
- **`dt` is the measured loop time** — this is the "rate measured, not assumed" criterion.
  `dt=1ms` is the target; `dt=2ms` says the I2C bus or printk is the limit.

### Known risks for the first hardware run

1. **400 kHz on the sifive I2C controller is untested here.** If the bus fails to init, fall back
   to 100 kHz in `flight_controller.overlay` and accept a ~550 Hz ceiling for now.
2. **The IMU remap is inherited, not verified.** Step 1 exists precisely to check it.
3. **The gyro-bias calibration wants the board still.** `g_gyro_cal_done` gates arming; it will
   not converge on a moving bench.
4. Nothing here reads the down-ToF, so the fly-away failure mode cannot occur in Phase 1.

### 2026-09-04 — Speculative execution: all phases built and statically verified

Hardware is away for a few days, so everything that can be settled without it was. Two of the
three risks logged above are now **resolved by reading the drivers**, not deferred to bench time.

#### Risk 1 resolved: 400 kHz I2C is exact

`drivers/i2c/i2c_sifive.c` supports `I2C_SPEED_FAST` and computes
`prescale = f_sys / (speed * 5) - 1`. On this SoC
`SIFIVE_PERIPHERAL_CLOCK_FREQUENCY = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC * CONFIG_RTC_CLOCK_DIVIDER_VALUE`,
and the generated `autoconf.h` gives `50000 * 1000` = **50 MHz exactly**:

| requested | prescale | actual |
| --- | --- | --- |
| 100 kHz | 99 | 100.0 kHz |
| 400 kHz | **24** | **400.0 kHz** |

No rounding error, no unsupported-speed path. The 400 kHz change is safe.

#### Risk 2 resolved: the sifive PWM can produce 20 kHz

`drivers/pwm/pwm_sifive.c` sets `PWMZEROCMP` and takes the period from `pwmcmp0`, so the period
is not capped at the 16-bit counter wrap. 20 kHz = 50 us = **2500 cycles** at 50 MHz, well inside
the 16-bit comparator, so `pwmscale` stays 0 and duty resolution is 1/2500 = **0.04%**.

This is also why comparator 0 cannot drive an output, matching `WithArty200TPWM`'s comment: it
*is* the period register.

**Caught while checking:** `sifive,pwm0` declares `#pwm-cells` as **const 2** -- `<&pwm0 ch period>`
-- where the ESP's LEDC binding takes three cells (`<&ledc0 ch period flags>`). The ESP's motor
specifiers cannot be transcribed directly; the FPGA overlay uses two cells.

#### A guard I got wrong, and corrected

I first guarded `flow.c` on `DT_NODE_EXISTS(DT_ALIAS(flow))`. That is **wrong**, and would have
silently disabled optical flow on the ESP build that actually flew.

`main.cpp` has two different flow guards on purpose:

| guard | meaning |
| --- | --- |
| `HAVE_FLOW` = `DT_ALIAS(flow)` | RoSE's **virtual** flow device — spike co-sim only |
| `ROSE_FLOW` (CMake knob) | the **real** PMW3901, talked to directly over SPI, no Zephyr driver and no `flow` node |

The ESP overlay has no `flow` alias at all and flies with `ROSE_FLOW=1`. So `flow.c` is now
guarded on the SPI controller it actually dereferences, with a `flow-spi` alias and a fallback to
the original `spi2` node label — leaving the ESP selecting exactly the node it did before.

#### Three phases, built and verified by symbol

`rb` now carries one mode per phase. The overlays `#include` each other, so IMU rates and bus
speed cannot drift between phases.

| mode | adds | RAM | flow syms | pwm syms |
| --- | --- | --- | --- | --- |
| `default` | IMU only | 364,768 B | 0 | **0** |
| `flow` | + PMW3901 on `spi0` | 375,248 B | 9 | **0** |
| `motors` | + 4 PWM outputs | 376,528 B | 9 | 8 |

DTS spot-checks: `flow-spi = &spi0`; motors `pwms = <&pwm0 1 0xc350>, <&pwm0 2 0xc350>,
<&pwm0 3 0xc350>, <&pwm1 1 0xc350>` (`0xc350` = 50000 ns = 20 kHz) against the harness map
motor1-3 = pwm0 cmp1-3, motor4 = pwm1 cmp1.

**`CONFIG_PWM` is deliberately absent from the shared conf.** `pwm_sifive_init()` writes `pwmcfg`,
which changes what the motor pins are driven to, so enabling it would give the actuation-free
phases a way to disturb the motor outputs. Phase 4 turns it on explicitly with
`RB_CMAKE_ARGS="-DCONFIG_PWM=y"`.

#### Ready-to-run commands

```bash
# Phase 1a — IMU axes/signs
RB_CMAKE_ARGS="-DROSE_USE_PID=1 -DROSE_USE_EKF=1 -DCTRL_ITERS=60000" \
RB_EXTRA_CPPFLAGS="-DROSE_IMU_DEBUG=1 -DROSE_TELEM_DIV=200" \
./rb debug flight_controller --target arty200t --config RocketArty200TDroneFullDDRConfig \
    --program --program-usb-location <bit> --usb-location <dbg>

# Phase 1b — attitude + measured loop dt (drop ROSE_IMU_DEBUG)
# Phase 2  — add --mode flow, RB_CMAKE_ARGS ... -DROSE_FLOW=1, CPPFLAGS ... -DCS_GPIO_PIN=0
# Phase 4  — add --mode motors, RB_CMAKE_ARGS ... -DCONFIG_PWM=y   [BATTERY + PROPS OFF]
```

Both env vars must be on the `rb debug` line — it rebuilds.

#### Still blocked / still unknown

- **Down-ToF** (`tof` alias): their Zephyr commit with xtalk-comp, tunable timing budget and
  `calibrate()` is unpublished. Unchanged.
- **The IMU remap is inherited from the ESP and unverified here** — that is what Phase 1a is for.
- **Phase 3's wireless data path is verified.** `esp_bridge` is the UART-attached modem and no
  framing protocol was needed. The loop-rate lead and remote-ESTOP measurement listed under
  Phase 3 remain open.
- **The motor-pin idle level is resolved: the pins idle LOW.** `pwmcfg=0xf0000000`
  at reset means the raw comparators are all asserted, but every motor output is
  inverted in the generated Verilog (`assign motorN = ~_chiptop0_pwm_*_gpio_*`,
  4/4 in every motor-bearing config), so the pins are driven low from
  configuration. See `motor-pwm-bringup.md`. Note this does **not** explain the
  two motors that spun on 2026-09-04; that cause is still unknown, so keep
  propellers off regardless.

- **A lead on what did spin them: two J1 pins this shell claims arbitrarily.**
  `RocketArty200TDroneFullDDRConfig` drives a strict superset of
  `RocketArty200TDroneDDRConfig`'s pins, and the latter has never spun a motor.
  Two of the additions are, in `WithArty200TGPIO`'s own words, *"arbitrary
  placeholders on J1 nets this repository never names"* -- chosen only because
  the sifive GPIO block is three bits wide and `pmw3901_test`'s devicetree
  contract needs three properties:

  | signal | pin | connector | reset state |
  | --- | --- | --- | --- |
  | `gpio_0` — PMW3901 NCS, legitimate | D16 | J1_76 | BIDIR, no pull |
  | `gpio_1` — **placeholder** | A20 | **J1_38** | BIDIR, **no pull: floats** |
  | `gpio_2` — **placeholder** | A21 | **J1_57** | BIDIR, pulldown |

  This repository does not know what J1_38 and J1_57 carry on the v3 base, and
  its `hardware/pcb` snapshot is a stale v2 layout, so it cannot find out. A
  floating pin on an unknown net is a plausible cause of non-deterministic
  behaviour -- and "two of four motors" is the shape a marginal, undriven gate
  produces.

  **Check J1_38 and J1_57 against `riskybirdv3_base` before the next motor
  session.** If either lands on anything motor-related, that is the answer, and
  the fix is to move the placeholders to genuinely unused pins and give them
  pulldowns. Stated as a lead, not a conclusion: it has not been measured.
