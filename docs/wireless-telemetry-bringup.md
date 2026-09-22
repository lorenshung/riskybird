# Wireless telemetry and the ground-station dashboard

This is the ordered bring-up for getting the drone's telemetry onto a laptop over
WiFi, and a dashboard showing it. It is written to be followed at a **different
bench from the one that wrote it** — everything below was built and, where it
could be, exercised; nothing below was run against the drone, because the drone
was not here. The last section says exactly which claims are unverified.

Every step before step 5 is **structurally incapable of spinning a motor**: the
images used in steps 0–4 contain no code path to a motor gate at all, which is
checked by symbol in step 0, not asserted.

---

## 0. The short version

```bash
cd /scratch2/loren/riskybird-dev-setup           # or wherever the checkout is
export RB_DTS_DIR=$HOME/riskybird-dts            # no elaborated DTS in this worktree

# --- the ESP: WiFi bridge, NO motor path in this image ---------------------
./rb build esp_bridge --target esp32c6
./rb flash esp_bridge --target esp32c6 --port /dev/tty.usbmodemXXXX

# --- the FPGA: telemetry out of uart1, no motor pin driven -----------------
./rb build flight_controller --target arty200t --mode telem \
    --config RocketArty200TDroneFullDDRDmaFlowConfig
./rb program  --board arty200t --config RocketArty200TDroneFullDDRDmaFlowConfig
./rb load flight_controller --board arty200t --mode telem

# --- the Mac: join the drone AP without losing the internet, then look -----
tools/bench/riskybird-wifi-macos.sh status
tools/bench/riskybird-wifi-macos.sh up --ssid riskybird-XXXX
tools/bench/riskybird-wifi-macos.sh verify
tools/bench/riskybird-wifi-macos.sh dash          # -> http://127.0.0.1:8080
```

No drone to hand? The dashboard proves itself with no hardware at all:

```bash
python3 tools/bench/riskybird-dash.py --selftest   # -> http://127.0.0.1:8080
```

---

## 1. The problem this document exists to solve

Three facts collide.

1. **Motor PWM is on the ESP32-C6.** FPGA ball F13 (motor4's gate) is dead as an
   output — proven with a bare-metal bitstream, no Rocket and no Chipyard: four
   pads driven at 4/8/12/16 % read back at the gates as M1=40 M2=80 M3=120 **M4=0**
   per mille, and the same ball as an input reads the carrier's 10k pulldown, so
   the net is intact and the driver is not. `workloads/esp_motors` therefore owns
   all four gates and takes 14-byte duty frames from the FPGA at 50 Hz, with a
   120 ms receive-timeout failsafe.
2. **Telemetry over WiFi is on the ESP32-C6.** `workloads/esp_bridge` brings up a
   SoftAP and forwards whatever arrives on the link to UDP :14550.
3. **There is one ESP, it runs one image, and there is one wire between the FPGA
   and it** — uart1, `serial@10021000`, `uart1_rxd E13/J1_96` and
   `uart1_txd F14/J1_98`, at a measured 115200.

So `esp_bridge` and `esp_motors` cannot both run, and both want the same wire.

### The three options, honestly

| | what it is | what it costs | verdict |
|---|---|---|---|
| **(a) merge the ESP images** | one image that receives duty frames *and* bridges telemetry over WiFi | the WiFi stack now shares a CPU with the motor failsafe; new, unproven image owning four gates | **necessary** — there is no second chip |
| **(b) multiplex uart1** | binary duty frames + ASCII telemetry on one wire, separated by the `0xA5 0x5A` magic and the sliding-window CRC decoder | telemetry bytes steal wire time *and*, because the FPGA console is polled, control-loop time | **necessary** — there is no second wire |
| **(c) telemetry stays on uart0** | keep the FTDI console as the telemetry path; wireless only when motors are not offloaded | **no telemetry at all in flight**: uart0 goes to a cable that is not attached to a flying drone, and motors are permanently offloaded because F13 is dead | rejected as the answer, **kept as the bench fallback** |

(a) and (b) are not alternatives to each other — they are the two halves of the
same thing. (a) is the ESP side, (b) is the wire. You need both, or you have no
in-flight telemetry.

**Decision: implement (a) + (b), and keep (c) available as a zero-radio fallback
for exercising the dashboard.** The merge is built as an **opt-in second mode of
`esp_bridge`, not a change to its default**, so every step of this bring-up that
does not need motors still runs an image with no motor code in it.

### Why (b) is safe on the wire

The duty frame opens with `0xA5 0x5A` — two bytes that cannot occur in ASCII
telemetry — and is validated by **CRC-16/CCITT-FALSE, not by the magic**. The
receiver is a 14-byte sliding window, so it resynchronises within one frame from
any amount of interleaved text, with no timeout and no special state.

What the window does *not* do on its own is keep frame bytes out of the text
stream. A duty field can contain `0x0A`, so naively copying every byte to the line
assembler would inject 14 binary bytes into a telemetry line *and split it*. The
merged image therefore runs a demux that holds the text back by exactly 14 bytes
and retracts them when a frame is accepted (`demux_feed()` in
`workloads/esp_bridge/src/main.c`). Cost: telemetry reaches the radio 1.2 ms
later. Nothing downstream can tell.

---

## 2. LOUD: the scheduling requirement, and a correction to it

> **A note recorded from earlier analysis said:** *"`CONFIG_NET_TC_THREAD_TYPE`
> has no explicit default so `NET_TC_THREAD_COOPERATIVE` wins and the networking
> RX/TX threads land at `K_PRIO_COOP(0)` = -16, the SAME priority as the motor
> thread. Merging REQUIRES `CONFIG_NET_TC_THREAD_PRIO_CUSTOM` with the TC base
> priorities pushed down."*
>
> **The first half is right. The conclusion is wrong in a way that makes things
> worse, and applying it literally would create the exact collision it was
> written to prevent.**

Read out of this checkout's Zephyr 4.2.99 and out of the workload's own generated
`.config` (`build/zephyr/esp32c6/esp_bridge/default/zephyr/.config`):

1. `CONFIG_NET_TC_THREAD_COOPERATIVE=y` — yes, the default does win.
2. But **without** `NET_TC_THREAD_PRIO_CUSTOM`, `subsys/net/ip/net_tc.c:163` sets
   `BASE_PRIO_RX = CONFIG_NET_TC_NUM_PRIORITIES - 1 = 15`, and the RX thread is
   created at `K_PRIO_COOP(15)` = `-(16 - 15)` = **-1**. Not -16. `net_tc.c`'s own
   comment spells the series out: *"-1, -2, -3, …"*.
3. `CONFIG_NET_TC_TX_COUNT=0` in this build — there is **no traffic-class TX
   thread at all**; TX runs in the calling thread, which here is the preemptible
   downlink thread.
4. Turning `CONFIG_NET_TC_THREAD_PRIO_CUSTOM=y` on *without also setting the base
   priorities* defaults `NET_TC_RX/TX_THREAD_BASE_PRIO` to **0**
   (`subsys/net/ip/Kconfig:297-312`), i.e. `K_PRIO_COOP(0)` = **-16** — the tie.
5. **And neither knob fixes the real hazard**, because in Zephyr **a cooperative
   thread is never preempted by another cooperative thread, whatever their
   priorities**: `kernel/include/kthread.h:227`, `should_preempt()`, returns false
   unless the running thread is preemptible or the incoming one is metairq. A
   cooperative net RX thread at -1 still holds the CPU until it blocks, and a -16
   motor thread waits.

There are three cooperative threads in the plain bridge, all at -1, all able to
delay a -16 failsafe:

| thread | priority | where it comes from |
|---|---|---|
| net TC RX | `K_PRIO_COOP(15)` = -1 | `subsys/net/ip/net_tc.c:163,170` |
| `net_mgmt` work queue | `K_PRIO_COOP(15)` = -1 | `subsys/net/ip/net_mgmt.c:419` |
| system work queue | -1 | `CONFIG_SYSTEM_WORKQUEUE_PRIORITY` — and DHCP server work runs here |

### What the merged image actually does

`workloads/esp_bridge/conf/motors.conf` makes them **preemptible**, which is the
property that matters:

```
CONFIG_NET_TC_THREAD_PREEMPTIVE=y     # RX thread becomes K_PRIO_PREEMPT(14);
                                      # also demotes net_mgmt via its own #elif
CONFIG_SYSTEM_WORKQUEUE_PRIORITY=1    # preemptible; DHCP work lives here
```

Confirmed in the generated config of the merged build:

```
CONFIG_NET_TC_THREAD_PREEMPTIVE=y
# CONFIG_NET_TC_THREAD_COOPERATIVE is not set
# CONFIG_NET_TC_THREAD_PRIO_CUSTOM is not set
CONFIG_NET_TC_NUM_PRIORITIES=15
CONFIG_NET_TC_RX_COUNT=1
CONFIG_NET_TC_TX_COUNT=0
CONFIG_SYSTEM_WORKQUEUE_PRIORITY=1
```

`CONFIG_NET_TC_THREAD_PREEMPTIVE` is marked EXPERIMENTAL upstream. That is an
honest label and it is still the right trade: an experimental scheduling class for
the *radio* is a smaller risk than a non-preemptible radio thread sitting in front
of a *motor failsafe*. (Do not write `CONFIG_EXPERIMENTAL=y` yourself — it has no
prompt and Kconfig errors out.)

### And the image checks it at boot

The argument above is unfalsifiable from a bench, so the image prints the answer.
`sched_audit()` walks every thread and emits:

```
esp_bridge: SCHED AUDIT -- N thread(s)
  motorlink        prio -16 COOP   <- failsafe thread
  downlink         prio   5 preempt
  ...
esp_bridge: VERDICT OK -- 'motorlink' is the only cooperative thread; nothing but
an ISR can delay the failsafe
```

**Read that line before the props go on.** If it says `VERDICT WARN`, the named
threads can delay the failsafe and the escape hatch is one Kconfig line:
`CONFIG_NUM_METAIRQ_PRIORITIES=1` turns priority -16 into a *metairq* priority
(`kernel/include/kthread.h:72` — the predicate is just
`prio - K_HIGHEST_THREAD_PRIO < CONFIG_NUM_METAIRQ_PRIORITIES`), which is the only
construct in Zephyr that preempts a cooperative thread, and it needs no code
change. It is off by default because Zephyr's own Kconfig says metairq *"probably
shouldn't be used from application code"*.

**What no Kconfig fixes:** the WiFi MAC does substantial work in interrupt
context, and ISRs preempt everything. That is the realistic source of failsafe
jitter, it is why the 120 ms timeout keeps 3.4 nominal frame intervals of margin,
and it is the first thing to measure on the bench (watch `gap=` and `cuts=` in the
`RBLINK` line under heavy WiFi traffic).

---

## 3. Bandwidth: what fits on 115200, and the divisor it implies

115200 8N1 = **11520 B/s**. Duty frames are 14 B × 50 Hz = **700 B/s (6.1 %)**.

Line lengths counted from the format strings in `main.cpp`, rendered with
realistic values:

| | bytes | wire time |
|---|---|---|
| core telemetry line (`flight_controller: it=… \|a\|=…`) | 190 B | 16.5 ms |
| the `  flow: …` line that `ROSE_FLOW=1` adds | 96 B | 8.3 ms |
| **one telemetry iteration in `--mode esp-motors`** | **286 B** | **24.8 ms** |
| core line *if* it ever gains the v1.1 tail | 262 B | 22.7 ms |

| telemetry rate | B/s | of the link | + duty frames | loop time stalled |
|---|---|---|---|---|
| 25 Hz | 7150 | 62 % | 68 % | 62 % |
| 10 Hz | 2860 | 25 % | 31 % | 25 % |
| 5 Hz | 1430 | 12 % | 19 % | 12 % |

**Telemetry rate is a flight-safety parameter, not a preference.** The FPGA's
serial driver is polled on this carrier (`CONFIG_UART_INTERRUPT_DRIVEN` is off),
so `printk` busy-waits and every byte is charged to the control loop. A prior
measurement recorded in-loop telemetry to a polled 115200 console making `printk`
busy-wait 30–60 ms and freezing the control loop ~22 % of the time; the table's
"loop time stalled" column is the same effect, computed.

The worst-case *contiguous* stall does not shrink with rate — one line is one
line — and it is what the ESP failsafe has to survive:

```
20 ms frame period  +  up to 35 ms control loop  +  24.8 ms telemetry write
= ~80 ms,  against a 120 ms failsafe.
```

That holds with ~33 % margin. What *does* scale with rate is how much control loop
is left.

**Divisors at the ~770 Hz the PID cascade actually achieves:**

| `ROSE_TELEM_DIV` | rate | use |
|---|---|---|
| 10 (the source default) | 77 Hz | **~22 kB/s — nearly 2× what the wire carries. Never.** |
| 30 | 25.7 Hz | what `--mode telem` sets. Fine with **no** motor frames. |
| **80** | **9.6 Hz** | **recommended for an offloaded flight build** |
| 150 | 5.1 Hz | when loop rate matters more than the view |

`--mode esp-motors` does **not** set `ROSE_TELEM_DIV`, so it inherits the source
default of 10. **Pass the divisor** (see step 5).

---

## 4. What was changed, and what was built

Everything here was built in this checkout. Real output:

| what | build | result |
|---|---|---|
| `esp_bridge` default (unchanged behaviour) | `rb build esp_bridge --target esp32c6` | FLASH 583336 B (13.91 %), sram0 188544 B (43.09 %) |
| `esp_bridge` **motors** (the merge) | west, with `esp_bridge-motors.overlay` + `conf/motors.conf` | FLASH 650264 B (15.50 %), sram0 197792 B (45.21 %) |
| `flight_controller --mode telem` | `rb build … --config RocketArty200TDroneFullDDRDmaFlowConfig` | RAM 410512 B |
| `flight_controller --mode esp-motors` + telemetry overlay + `ROSE_TELEM_DIV=80` | as above plus `RB_CMAKE_ARGS` | RAM 416624 B |

Symbol-level proof that the default bridge still cannot touch a motor:

```
motors_apply     default=0 motors=1
g_kill           default=0 motors=1
g_demux          default=0 motors=1
link_thread      default=0 motors=1
status_thread    default=0 motors=1
pwm_led_esp32*   default=0 motors=6      <- the LEDC driver is not even linked in
RBLINK string    default=0 motors=1
```

### Files

| file | what |
|---|---|
| `workloads/esp_bridge/src/main.c` | the merge. Motor half gated on `DT_NODE_EXISTS(DT_ALIAS(pwm_motor0))`, so devicetree alone decides whether this image can drive a gate |
| `workloads/esp_bridge/conf/motors.conf` | the scheduling work of section 2, plus `CONFIG_PWM=y` |
| `hardware/zephyr/targets/esp32c6/workloads/esp_bridge-motors.overlay` | LEDC on GPIO21/20/23/22 + the `pwm-motorN` aliases; otherwise identical to `esp_bridge.overlay` |
| `workloads/esp_bridge/CMakeLists.txt` | includes the shared `integration/zephyr/motor_link` header, with a build-time check that it is there |
| `hardware/zephyr/fpga-esp-telem-on-uart1.overlay` | the FPGA side of (b): puts the console — and therefore telemetry — on uart1 on top of any mode that already owns it |
| `tools/bench/riskybird-dash.py` | the dashboard (stdlib only, self-contained page) |
| `tools/bench/riskybird-wifi-macos.sh` | the macOS join/verify script, now with a `dash` subcommand and telemetry-vs-noise classification in `verify` |

### One thing someone else has to do

`rb` rejects `--mode motors` for `esp_bridge` because the mode list lives in
`tools/rb/boards.py`, which another workstream owns this session. **One line:**

```python
    "esp_bridge": _workload(
        "esp_bridge",
+       modes=("default", "motors"),
        targets=("esp32c6",),
```

Until that lands, build the merged image with the identical west command `rb`
would have run:

```bash
R=$PWD
west build -b esp32c6_devkitc/esp32c6/hpcore \
  -d $R/build/zephyr/esp32c6/esp_bridge/motors -p auto $R/workloads/esp_bridge -- \
  -DRB_ROOT=$R -DRB_TARGET=esp32c6 -DDTC=/usr/bin/dtc \
  -DDTS_ROOT=$R/hardware/zephyr \
  -DKCONFIG_ROOT=$R/hardware/zephyr/Kconfig.workload \
  "-DDTC_OVERLAY_FILE=$R/hardware/zephyr/targets/esp32c6/target.overlay;$R/hardware/zephyr/targets/esp32c6/workloads/esp_bridge-motors.overlay" \
  "-DEXTRA_CONF_FILE=$R/hardware/zephyr/targets/esp32c6/target.conf;$R/workloads/esp_bridge/conf/motors.conf"

# and flash it the way rb flash would:
west flash -d $R/build/zephyr/esp32c6/esp_bridge/motors -- --esp-device /dev/tty.usbmodemXXXX
```

Both need the Zephyr environment `rb` builds for itself — a bare `west` on your
PATH will not find the SDK. Get a shell that has it with:

```bash
source ./rb-env.sh        # puts rb on PATH
rb env shell zephyr       # NOT `rb env enter`; the subcommands are status|setup|shell
```

then run the two commands above inside it. Adding the `boards.py` line makes all
of this unnecessary.

---

## 5. The ordered bring-up

### Step 0 — prove the dashboard with no hardware at all

```bash
python3 tools/bench/riskybird-dash.py --selftest
# [dash] SELFTEST: synthesising telemetry -> 127.0.0.1:14550 at 10.0 Hz
# [dash] dashboard: http://127.0.0.1:8080
```

This synthesises both wire formats and sends them **through the real UDP socket**,
so the socket, the regexes, the SSE stream and the page are all exercised. If this
does not work, nothing later will, and you have learned it without a drone.

Verified here: `/config`, `/stream` and `/` all answer, both line formats parse,
and the selftest deliberately clips the ESP's applied duty to the 25 % ceiling so
the "commanded vs applied" split is visible immediately.

### Step 1 — the ESP, plain bridge (cannot spin a motor)

```bash
./rb build esp_bridge --target esp32c6
./rb flash esp_bridge --target esp32c6 --port /dev/tty.usbmodemXXXX
```

The ESP console is the **USB-C port** (native USB Serial/JTAG, `303a:1001`,
enumerates as `/dev/tty.usbmodem*` on macOS), not the FTDI. Expect:

```
=== RiskyBird ESP link: FPGA uart <-> WiFi UDP ===
esp_bridge: FPGA link up on uart@60000000
esp_bridge: SoftAP up
esp_bridge: SSID riskybird-XXXX, ch 6, open; telemetry -> UDP :14550
esp_bridge: commands on UDP :14551
esp_bridge: SCHED AUDIT -- N thread(s)
  ...
esp_bridge: no motor path in this image -- scheduling is not safety-critical here
```

**Write down the SSID.** It carries the ESP's MAC tail and the bench scripts
default to `riskybird-5674`, which may not be this board.

### Step 2 — join the AP from the Mac, without losing the internet

```bash
tools/bench/riskybird-wifi-macos.sh status
```

It will refuse to go `up` unless a **non-WiFi** internet path is already active —
a Mac has one WiFi radio, so "drone AP + internet" needs an iPhone on USB with
Personal Hotspot, or a USB-C/Thunderbolt Ethernet adapter. That refusal is
deliberate: joining with no second path is how a bench loses its network
mid-session.

```bash
tools/bench/riskybird-wifi-macos.sh up --ssid riskybird-XXXX
```

It cycles the radio (a cached DHCP lease after a drone reboot is worse than no
lease: ping succeeds while the bridge's fresh DHCP server has no lease to unicast
to, so telemetry silently goes to zero clients), waits for a `192.168.4.x` lease,
checks the drone did **not** steal the default route, and proves UDP both ways
with a `PING` → `ACK PING`.

> That `ACK PING` comes from the **bridge**, not the flight controller. It proves
> association + DHCP + UDP in both directions and nothing about the FPGA.

### Step 3 — the FPGA, telemetry mode (cannot spin a motor)

`--mode telem` moves the console to uart1 and declares **no `motors` alias**, so
the PWM actuator is not compiled in at all.

```bash
export RB_DTS_DIR=$HOME/riskybird-dts
./rb build flight_controller --target arty200t --mode telem \
    --config RocketArty200TDroneFullDDRDmaFlowConfig
./rb program --board arty200t --config RocketArty200TDroneFullDDRDmaFlowConfig
./rb load flight_controller --board arty200t --mode telem
```

**The `--config` is not optional.** Only the full drone shells elaborate
`serial@10021000`; on any other config `rb` does not append
`hardware/zephyr/fpga-esp-uart.overlay` and the build fails with
`devicetree error: /chosen: undefined node label 'uart1'`. That is what a wrong
`--config` looks like.

The FTDI console goes quiet in this mode — that is the mode working. Everything
the flight controller prints now leaves over the radio.

### Step 4 — confirm telemetry is flowing

```bash
tools/bench/riskybird-wifi-macos.sh verify
```

```
uplink  PING -> 192.168.4.1:14551
  ACK PING
downlink listening 10s on :14550
  first datagram from 192.168.4.1: flight_controller: it=1240 dt=1ms roll=-0.012 ...
  512 datagrams, 51.2 Hz, median gap 19.5 ms, max gap 41.0 ms, 0 gap(s) > 58.5 ms
  of those: 256 telemetry (25.6 Hz), 0 RBLINK, 256 other console line(s)
```

The **classification line is the one that matters**. In `--mode telem` the whole
console comes over the link, so a datagram count alone says "the radio works"
while saying nothing about whether `ROSE_TELEM` is on. `verify` fails loudly if
zero telemetry lines arrived and prints a sample of what did.

Then leave the dashboard open:

```bash
tools/bench/riskybird-wifi-macos.sh dash        # -> http://127.0.0.1:8080
```

**Stop here and look at the dashboard properly.** Tilt the airframe by hand and
watch roll/pitch/tilt move; cover the down-ToF and watch `z` and `tofh` diverge.
Everything up to this point is reversible and cannot move a motor.

### Step 5 — the merged ESP image (THIS ONE DRIVES THE GATES)

> **Props off. Battery disconnected for the first boot.** Flashing this image
> replaces `motor_safe` — the gate probe every motor measurement in this
> investigation came from — and replaces `esp_motors`. Only one of the four can
> be on the chip.

Build and flash it with the commands in section 4 (or `rb build/flash esp_bridge
--mode motors` once `boards.py` has the mode). Expect:

```
=== RiskyBird ESP link: FPGA uart <-> WiFi UDP + 4x motor gates ===
esp_bridge: MOTORS BUILD -- this image DRIVES the gates. MOTOR1=GPIO21 ... 20 kHz, active-high
esp_bridge: failsafe 120 ms, tick 5 ms, duty ceiling 25%
esp_bridge: SSID riskybird-XXXX, ch 6, open; telemetry -> UDP :14550
esp_bridge: SCHED AUDIT -- N thread(s)
  motorlink        prio -16 COOP   <- failsafe thread
  ...
esp_bridge: VERDICT OK -- 'motorlink' is the only cooperative thread; ...
RBLINK v1 ml=down up=... fl=0x00[off] duty=[0 0 0 0] fr=0 ... cuts=0 kill=0 cl=0
```

`ml=down` with `fr=0` before the FPGA is running is **correct**: the failsafe
deadline starts already expired, so nothing can drive a gate until a valid frame
has arrived. **Meter the four gates now**, with the battery still off, and
confirm they are at 0.

Then the FPGA side, motors offloaded **and** telemetry on the same wire:

```bash
RB_CMAKE_ARGS="-DEXTRA_DTC_OVERLAY_FILE=$PWD/hardware/zephyr/fpga-esp-telem-on-uart1.overlay -DROSE_TELEM_DIV=80" \
./rb build flight_controller --target arty200t --mode esp-motors \
    --config RocketArty200TDroneFullDDRDmaFlowConfig
./rb load flight_controller --board arty200t --mode esp-motors
```

Confirm in `build/zephyr/arty200t/flight_controller/esp-motors/zephyr/zephyr.dts`:

```
zephyr,console = &uart1;    /* in .../fpga-esp-telem-on-uart1.overlay */
```

and in that build directory's `CMakeCache.txt`: `ROSE_TELEM_DIV:STRING=80`.

Now the dashboard should show **both** streams: flight-controller telemetry at
~10 Hz *and* `RBLINK` at 5 Hz. Watch `crc=` and `noise=`:

- `noise=` climbing → something is on the wire at the wrong rate. Different fault.
- `crc=` climbing with `noise=` flat → frames are being damaged. **A small,
  non-zero `crc=` is EXPECTED here** — a printk from a thread other than the
  control loop can land in the middle of a frame. Six *consecutive* damaged
  frames would be needed to reach the failsafe.

### Step 6 — the remote kill, before it is needed

Still props off. Press **ESTOP** on the dashboard. The banner must change to
**KILL LATCHED AT THE ESP** and `RBLINK` must show `kill=1` with `duty=[0 0 0 0]`.
Press **RESET** and watch it clear.

> **The ESTOP button has never done anything on this carrier until now.** The
> flight controller has no uart1 command reader at all: every `rose_cmd_*` hook in
> `main.cpp` is inside `#if defined(CONFIG_WIFI)`, which is only true on the
> ESP-native build, and the `telem_wifi.c` that would call them is not in this
> checkout. The bridge writes the command bytes into the FPGA's receiver, nothing
> reads them, and the bridge generates the ACK itself — so it has always *looked*
> like it worked. In the merged image ESTOP is executed **at the ESP, one
> component before the MOSFET**, with no dependency on the FPGA still running.
> That is the strongest single argument for merging.
>
> **Trust the banner and `kill=1`, not the ACK.** `DISARM` and `HOVER_Z` still do
> nothing on this carrier.

Only after all of the above: props on, battery connected (motor and flight tests
do nothing useful on USB-C alone), and the procedure in
`docs/flight-safety-bench-procedure.md`.

---

## 6. The telemetry line, field by field

Read out of the format string in
`backends/zephyr-chipyard-sw/samples/rose_flight_controller/src/main.cpp` and
confirmed in the built ELF:

```
flight_controller: it=%d dt=%dms roll=%s%d.%03d pitch=%s%d.%03d yaw=%s%d.%03d
  z=%s%d.%03d tofv=%d tofh=%s%d.%03d
  u=[%s%d.%03d ×4] duty=[%s%d.%03d ×4]
  tilt=est%ddeg/acc%ddeg |a|=%s%d.%03d
```

| field | meaning |
|---|---|
| `it` | control-loop iteration counter. Frozen → the loop has stopped. |
| `dt=…ms` | the loop's own measured iteration time. |
| `roll` `pitch` `yaw` | **NOT degrees and NOT radians.** These are the RODRIGUES (Gibbs) vector, `r = tan(θ/2)` per axis — `state[3..5]`. Reading them as radians is what put a tilt guard's real trip point at 90° while its constant said 57°. The dashboard reconstructs the exact quaternion and shows proper ZYX Euler degrees. |
| `z` | estimated altitude, metres. |
| `tofv` | down-ToF validity, 0/1. `tofv=0` means `z` is dead reckoning. |
| `tofh` | raw down-ToF slant range, metres — uncorrected. Shown next to `z` on purpose: they should track, and the gap is the correction. |
| `u=[…]` | the controller's command vector, ~`[-0.583, +0.417]`. **NOT a duty cycle.** (`riskybird_panel.py` draws this as "motor duty"; it is not one.) |
| `duty=[…]` | what the actuator layer was actually asked for, 0…1, motor 1-4. **This is the duty.** |
| `tilt=estNdeg/accNdeg` | tilt in honest DEGREES. `est` is the estimator's, converted; `acc` is straight out of gravity with no estimator involved. **Watch these two, not roll/pitch, when checking the guard.** They diverging means the estimator and gravity disagree. |
| `\|a\|` | accelerometer magnitude, m/s². ~9.81 at rest; near 0 is free fall. |

Extra lines this build also emits, which are **not** telemetry and are ignored by
the parsers: the `  flow: aRaw=… gyro=… v=… sq=… \| est v=…` line from
`ROSE_FLOW=1`, and `  walls[seq=…]` from `ROSE_BUMPER`.

### Fields this build does NOT send

The ground station's optional **v1.1 tail** — `x= y= vx= vy= vz= zsp= vbat= st=` —
is **not emitted**. The format string ends at `|a|=`. Consequences:

- **battery is blank.** `riskybird_panel.py` shows `--`; `riskybird-dash.py` says
  *"not in this build"* rather than drawing a zero. Read the pack on a meter.
- **`st=` is absent, so the flight state is unknown from the FPGA line.**
  `riskybird_panel.py` treats a missing `st` as 0 and will confidently show
  **DISARMED forever, including while the motors are spinning.** Do not use it to
  judge armed state on this carrier.
- Armed/estop state comes from the **`RBLINK` line's `fl=` byte** instead, which
  is the flag byte the FPGA actually put on the wire in each duty frame.

### The ESP's line

```
RBLINK v1 ml=up up=12345 seq=17 fl=0x21[ARMED COMMITTED ] duty=[1000 1100 1050 1020]
  fr=1234 gap=20 lost=0 stale=0 crc=0 noise=0 cuts=0 pwmerr=0 kill=0 txt=98765 cl=1
```

| field | meaning |
|---|---|
| `ml` | `up` / `down`. **`down` = the failsafe has cut.** |
| `up` | ESP uptime, ms. |
| `seq` | last accepted frame's sequence byte. |
| `fl` | the FPGA's flag byte, decoded in the brackets: `ARMED` `COMMITTED` `ESTOP` `TIMEOUT` `INHIBIT` `WINDOW-EXPIRED`. |
| `duty` | per 10000, **as applied at the gates** — after the ESP's own 25 % ceiling and after the kill latch. |
| `fr` / `lost` / `stale` | frames accepted / inferred missing from `seq` gaps / rejected as not newer. |
| `gap` | measured ms between the last two accepted frames. |
| `crc` / `noise` | frames failing CRC / bytes that never belonged to a frame. Different faults — see step 5. |
| `cuts` | how many times the failsafe has fired since boot. |
| `pwmerr` | LEDC write failures. Should be 0. |
| `kill` | 1 = a remote ESTOP is latched here. |
| `txt` | ASCII bytes forwarded to the radio — i.e. proof the demux is working. |
| `cl` | DHCP-leased clients being unicast to. **`cl=0` means telemetry is going nowhere**, which is what a stale association looks like. |

---

## 7. Telling "link dead" from "motors commanded off"

This is the distinction the whole `RBLINK` line exists for, and the FPGA's own
telemetry **structurally cannot** answer it: the FPGA reports what it *sent*.

| what you see | what it means | what to do |
|---|---|---|
| `ml=down`, `cuts` rising | **LINK DEAD.** No valid frame inside 120 ms; all four gates cut. The FPGA is halted (`rb debug` does this routinely), crashed, reset, or the uart1 wire is broken. | look at the FPGA, not the radio |
| `ml=up`, `fl` without `ARMED`, `duty=[0 0 0 0]` | **COMMANDED OFF.** Frames are arriving normally and the FPGA is asking for zero. A healthy link. | nothing is wrong |
| `ml=up`, `fl` has `ESTOP`/`TIMEOUT`/`INHIBIT`/`WINDOW-EXPIRED` | commanded off *for a named reason*, sent by the FPGA | the bracket text is the reason |
| `kill=1` | a remote ESTOP is latched **at the ESP**; frames are being ignored | press RESET, or power-cycle the ESP |
| no `RBLINK` line at all | the ESP is running the **plain bridge** — there is no motor path in that image, and nothing here knows the gate state | expected in steps 1–4 |
| telemetry stops but `RBLINK` continues | the FPGA stopped *printing* while still sending duty frames | check `ROSE_TELEM` / the divisor |
| both stop | the radio or the association dropped | `riskybird-wifi-macos.sh status`, then `up` again |

The dashboard says all of this in words in its second banner, colour-coded, with
the reason spelled out underneath. It never uses colour alone.

---

## 8. Things that look like faults and are not

- **`cl=0` right after a drone reboot while `ping` still works.** The Mac kept its
  old lease; the bridge's fresh DHCP server has none to unicast to. `riskybird-wifi-macos.sh up`
  cycles the radio for exactly this reason.
- **A 169.254.x address on `en0`.** The AP interface came up without an address, so
  no DHCP offer was made and macOS fell back to IPv4LL. 100 % loss to
  192.168.4.1 follows. The radio is fine.
- **100–400 ms gaps in `verify`.** macOS cannot disable STA power-save per
  interface (the Linux profile sets `802-11-wireless.powersave 2`). Not fixable
  from the script.
- **Silent FTDI console in `--mode telem` or with the telemetry overlay.** That is
  the mode working — the console moved to uart1. It also means **a radio that is
  not up leaves you with no console at all**, which is why the radio is brought up
  first.
- **Small non-zero `crc=` when telemetry shares the wire with duty frames.**
  Expected. `noise=` climbing is the wrong-baud signature; `crc=` alone is not.
- **The dashboard's ACK saying `ACK DISARM`.** It means "delivered to the FPGA's
  UART". Nothing reads it. See step 6.

---

## 9. What is NOT verified

Everything behavioural. The drone was not at this bench. Specifically:

**Verified here (real output):**
- both ESP images and both FPGA builds compile and link; sizes above
- the merged image's `.config` carries the preemptive networking threads and the
  preemptible system work queue
- the default bridge image contains **no** LEDC/PWM driver and none of the motor
  symbols — checked with `nm`, not asserted
- the FPGA `esp-motors` + telemetry build really has `zephyr,console = &uart1` and
  `ROSE_TELEM_DIV=80`, read from the generated DTS and CMake cache
- the telemetry format string in the built ELF matches what the parsers expect
- `riskybird-dash.py` parses both line formats, serves the page, streams over SSE,
  and answers `/config` — exercised over a real UDP socket via `--selftest`
- the dashboard's two-series palette passes every colour check in light and dark
- `riskybird-wifi-macos.sh dash` launches the dashboard with the right arguments
- the bandwidth and divisor arithmetic, computed from the actual format strings

**NOT verified — needs the bench:**
- that the merged image's SoftAP still comes up with the motor thread running
- that the scheduler audit prints `VERDICT OK` (it is a prediction until a board
  prints it)
- the actual failsafe latency under WiFi load — the ISR-context jitter from the
  WiFi MAC is the one hazard no Kconfig addresses, and it can only be measured
- that the demux really keeps frame bytes out of telemetry lines on real traffic
  (`txt=` climbing while `crc=` stays near zero is the evidence to look for)
- that ASCII and binary genuinely coexist on uart1 at the chosen divisor
- that the remote ESTOP cuts the gates (**meter them**, do not trust the banner
  the first time)
- every duty number, every gate voltage, every flight behaviour
- the SSID: `riskybird-5674` is this script's default and may not be this board

**Known, not fixed here (owned elsewhere):**
- `boards.py` needs the one-line mode addition (section 4)
- `hardware/zephyr/fpga-esp-uart.overlay` and
  `hardware/zephyr/targets/esp32c6/workloads/esp_motors.overlay` still say 115200
  is unverified. It is now measured — 693 frames, zero CRC and zero noise errors.
  Those comments belong to the motor-offload workstream and were left alone.
- `workloads/esp_motors` and the motor half of `esp_bridge --mode motors` are two
  implementations of the same receiver, from the same shared header. That is one
  too many. Consolidating them (most cleanly: lifting the receiver into
  `integration/zephyr/motor_link/` as a small library both link) is the obvious
  follow-up, and was not done here only because `workloads/esp_motors/` is owned
  by another workstream this session.
