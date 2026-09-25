# Running the drone bench remotely

> Written 2026-09-22, when the ESP ran `esp_motors`. The flying ESP image is now
> `esp_bridge --mode motors` ([`esp-motor-offload.md`](esp-motor-offload.md)): the same
> 120 ms failsafe on the same gates, plus the Wi-Fi bridge and an ESP-side ESTOP. Read
> `esp_motors` below as "a motor-driving ESP image" unless the text is about its console.

For the case where the hardware is on the bench and **you are not in the room**.
Nobody is there to catch the drone, unplug the battery, or notice that a motor
started. Everything below is written for that case and nothing else.

The short version:

1. `tools/bench/rb-watch.sh` — one command, one persistent tmux session, four
   panes, everything logged. Detaching leaves it all running.
2. **Read the log file, never the device.** Only one reader per tty works.
3. **A remote operator has no physical stop.** Every stop available to you
   depends on software that is still working. The only stop that does not is a
   hand on the battery, and you do not have a hand there.
4. So: only run things that **cannot** spin a motor. The boundary is the
   battery, not your intentions — and no telemetry field can prove the pack is
   out, so that boundary is held by a person's word, not a number (§4.1).

---

## 1. Starting the session

```bash
tools/bench/rb-watch.sh          # from the top of the checkout
```

Run it again any time — if the session exists it re-attaches and restarts
nothing. Re-running is not a reset.

```
session 'riskybird' already exists (nothing restarted, nothing re-probed)
```

On a fresh start it prints where the logs are going:

```
creating session 'riskybird'
  logs         : $RB_LOGDIR/sessions/2026-09-22T15-09-35
  FPGA console : /dev/ttyUSB5 -> $RB_LOGDIR/fpga-console.log
  ESP          : /dev/ttyACM0 -> $RB_LOGDIR/esp-live.log
```

### The four panes

```
+---------------------------+------------------------+
|                           |  STATUS  (refreshing)  |
|                           +------------------------+
|   work pane               |  FPGA console          |
|   (shell / tools)         +------------------------+
|                           |  ESP link / gate probe |
+---------------------------+------------------------+
```

| pane | what it is for | what it must never do |
| --- | --- | --- |
| 0, work | builds, greps, git, `rb debug` | open a console device |
| 1, status | `rb-status.sh -w 5`, the one screen you check first | touch the bench |
| 2, FPGA console | **holds** the FPGA console tty and captures it | be opened by anything else |
| 3, ESP | **holds** the ESP tty and captures it | be opened by anything else |

Panes 2 and 3 are not displays. They are the **owners** of the two serial
devices. That is the whole design: two processes hold the two ttys for the life
of the session, and everyone else reads what those two wrote down.

Both retry forever. With nothing plugged in you get

```
=== ESP : /dev/ttyACM_absent not present, waiting ===
```

every five seconds, and the moment a device appears it attaches on its own. **You
do not need to restart anything when hardware is plugged in later**, which
matters when the person doing the plugging is not you.

`ctrl-b d` detaches. Closing the ssh connection does the same thing. Everything
keeps running and keeps logging.

---

## 2. The rule that everything else depends on

> **Read the log file. Never open the device.**

Two readers on one `/dev/ttyUSB*` do not produce an error. The kernel hands each
byte to exactly one of them, so both get roughly half the stream, interleaved at
random. Both consoles look "flaky". Nothing reports a fault. You will chase it
for an hour and blame the UART, the cable, the FPGA, and the baud rate before you
think of it.

The console panes hold the devices. Therefore:

| you want to | do this | never this |
| --- | --- | --- |
| see the FPGA console now | look at pane 2, or `tail -f ~/riskybird-logs/fpga-console.log` | `rb console --port /dev/ttyUSB5` |
| grep for a cutoff | `grep -a CUTOFF ~/riskybird-logs/fpga-console.log` | `cat /dev/ttyUSB5` |
| check the ESP link | `grep -a 'link=' ~/riskybird-logs/esp-live.log \| tail -1` | `miniterm /dev/ttyACM0` |
| script an analysis | read the log file | open the device "just for a second" |

`~/riskybird-logs/fpga-console.log` and `~/riskybird-logs/esp-live.log` are
stable paths that always point at the current capture. Use them everywhere. They
are symlinks into the current session directory (§6), so `tail -f` follows them
fine — but note that `stat` and `du` need `-L` to see through them, which is why
`rb-status.sh` uses `stat -Lc` and `du -hL`. Without that the freshness check
reads the symlink's own mtime and reports "no recent output" forever while data
is pouring in.

### The same rule for JTAG

One OpenOCD per FTDI adapter. `rb debug` holds one; `rb bench`, `rb program` and
`openFPGALoader` each want the same one. A second opener gets

```
cannot open the adapter -- in use by another process, or no device access
```

if you are lucky, and a confused MPSSE if you are not. On this bench a dirty
MPSSE has taken the USB bus down with it.

This is why **`rb-status.sh` no longer probes the adapters on its refresh loop.**
It reads a cached `rb bench` table that `rb-watch.sh` wrote once at session
start, and shows how old it is. `rb-status.sh -p` is the one command that
re-probes, it is yours to run deliberately, and it refuses while OpenOCD is
running.

---

## 3. Check this before you start anything

`rb-status.sh` is the first thing you read after attaching. It opens no serial
device, opens no adapter, programs nothing and writes nothing to the board.

Real output, on this host, with nothing plugged in:

```
 RiskyBird bench   2026-09-22 15:09:57   host garden
 session 2026-09-22T15-09-35
─────────────────────────────────────────────────────────────────

ADAPTERS
  bitstream link   ABSENT -- cannot program the FPGA
  debug link       7-1.3 console=/dev/ttyUSB5
  cached, probed 26s ago -- rb-status.sh -p to re-probe (locations renumber on replug)

FPGA
  configuration    UNCONFIGURED -- debug TAP shifting all ones, nothing driving TDO

ESP32-C6
  usb              ABSENT

CAN A MOTOR TURN RIGHT NOW?
  +BATT rail       ENERGISED  4.19 V -- A MOTOR CAN TURN. Not for unattended use.
                   NOTE: ~4.2 V on USB with no pack fitted looks identical to a full pack.
  commit gate      UNCOMMITTED -- 5 s motor budget, then latched off
  commanded duty   duty=[0.000 0.000 0.000 0.000]
  guard            tilt=est1deg/acc1deg |a|=9.810

RECENT
  flight_controller: ENVELOPE GUARD ARMED -- watching accel tilt, free-fall, impact, est tilt, rate, velocity, height

LOGS
  fpga-console.log           4.0K  last write 15:09:43  fpga-console.001.log
  esp-live.log           not being captured

reads log files only -- never a serial device; programs nothing, writes nothing to the board
```

Top to bottom, what each line is telling you:

**session** — which `rb-watch.sh` run owns the logs right now. `none` means
nothing is capturing, so every "no recent output" below is meaningless: you are
looking at a screen with no inputs.

**ADAPTERS / bitstream link** — `ABSENT` means nothing can program the FPGA. Not
a fault on its own; the drone bench often has only the debug adapter attached.

**ADAPTERS / debug link** — the USB location and the console tty. `ABSENT` here
means you cannot load or halt Zephyr, which also means **you cannot run
`estop.gdb`**. Note the age on the next line: USB locations and ttyUSB numbers
renumber on **every** replug, so a table probed before the last replug is
fiction. Re-derive with `rb-status.sh -p` or `rb bench` rather than copying a
number out of an old command.

**FPGA / configuration** — `CONFIGURED` means the Rocket debug TAP answers, so a
design is loaded and probably running. `UNCONFIGURED` means the part is empty:
nothing is executing, and nothing can drive a motor gate from the FPGA side.

**ESP32-C6 / firmware** — which image owns the four motor gates.
`motor_safe` parks all four ESP pins as high-impedance inputs and only *reads*
the gates, so the ESP cannot drive a motor at all. `esp_motors` means **the ESP
owns all four gates** and will drive them from the frames the FPGA sends.

**ESP32-C6 / link** — the frame link between FPGA and ESP. `link=DOWN -- MOTORS
OFF (failsafe)` is the ESP having cut everything after 120 ms of silence.

**CAN A MOTOR TURN RIGHT NOW? / +BATT rail** — the line that decides everything,
and the one most likely to be misread. It reports whether the rail the motor
FETs switch is **energised**. It is not a pack-presence flag and it cannot be
made into one. See §4.1 before you act on it.

**commit gate** — `UNCOMMITTED` is the safe default: motors latch off 5 s after
they first spin. `WINDOW OPEN` means motors are live *now*. `COMMITTED` means
somebody widened the budget to 60 s for a flight test, and props are live.

**RECENT** — cutoffs, guard trips, window events from the FPGA log.

**LOGS** — size, last write, and which part file is current. "last write" is the
real liveness signal: if it is minutes old while a workload is supposed to be
running, the board is not printing, and no amount of reading the screen will
change that.

---

## 4. The tiering: what you may do from a hotel room

The gate is **physical state, not intent**. The drone's motors are brushed, on
low-side SI2302 FETs — there are no ESCs. With no energy on +BATT there is
nothing for the FET to switch: the gate can be driven perfectly and the rotor
does not move. That is the one guarantee that does not depend on any of our
software being correct, which is why it is the boundary.

But read §4.1 before you assume you can *see* that state. You cannot, and the
reason is in the schematic rather than in the software.

### Tier 1 — cannot spin a motor under any circumstances

Do these freely, remote, unattended, at 3 a.m.

- Anything that never reaches the board: builds, Chipyard elaboration, `nm` and
  symbol checks, doc work, reading and grepping logs, plotting flight logs.
- Workloads with **no motor aliases in their devicetree**, so `CONFIG_PWM` is off
  and `pwm_sifive_init()` never runs. There is no register that can drive a gate
  because the block is not connected to anything:
  `sensor_check`, `bmi088_test`, `ads7128_test`, `i2c_scanner`,
  `pmw3901_test`, `ddr_stress`, `camera_bringup`, `camera_photo`.
- `bootup_check` **in its default mode**. Stage 8 (motors) is compiled out
  entirely unless the `pwm_motor1` alias exists, which only
  `--mode motors` supplies.
- `motor_safe` on the ESP. It configures all four ESP gate pins as inputs and
  uses them as logic probes. It is the *absence* of a driver.

A useful property of tier 1: it is also the tier that answers most questions.
"Is the ToF answering?", "did the camera capture?", "what is the accelerometer
reading?" are all tier 1.

### Tier 2 — safe **only** because the battery is disconnected

These drive gates. Do them remotely **only on a timestamped human statement that
the pack is out of J7** (§4.1 — no telemetry field can give you this), and even
then knowing that ~200 µF of charged +BATT bulk capacitance can still twitch a
rotor. In this tier a mistake is invisible until the pack goes back in.

- Anything that selects a `-motors` overlay, which is what puts the `pwm_motor*`
  aliases in the devicetree and turns `CONFIG_PWM` on: `motor_1` … `motor_4`,
  `bootup_check --mode motors`, `flight_controller --mode motors`. (Only those
  overlays define the aliases — checked across
  `hardware/zephyr/targets/fpga/workloads/*.overlay`.)
- `motor_duty`, which drives no FPGA gate at all but emits the same 14-byte duty
  frames on the same wire at the same 50 Hz, so the ESP cannot tell it from the
  flight controller. "The FPGA has no PWM here" is not a safety property when the
  ESP owns the gates.
- `flight_controller --mode esp-motors` with `esp_motors` on the ESP. In every
  non-autoflight mode `g_armed` is a compile-time `true`, so the controller asks
  for thrust from its first control tick. The image is *commanding* motors from
  the moment it runs.
- Anything that writes the SiFive PWM comparators, including `stop.gdb` on the
  PWM path.

The honest description of tier 2 is: **you are running motor code and trusting a
connector you cannot see, on somebody's word.** That is a real safety argument —
it is the same one the bench procedure uses — but it is the connector doing the
work, not the code, and not the status screen.

### Tier 3 — requires a person physically present

Never remote. Not "remote with care". Not "remote because the guard is good now".

- Anything in tier 2 **with the battery connected**.
- `hardware/flight/commit.gdb`, which widens the motor budget from 5 s to 60 s.
  Its own header says: *run this only when you are standing at the drone and
  ready to fly.*
- `arm-enable.gdb` and the lift-and-place arming gesture.
- Props on. Anything with props on.
- Any first run of a guard change. The envelope guard is measured and it works —
  `47.457 deg est-tilt, held 141 ms over 5 samples; motors OFF`, `cuts=1` on the
  ESP failsafe, `5009 ms` on the commitment window — and every one of those
  numbers was measured by a person standing next to the drone who could pull the
  pack if it had gone the other way.

### 4.1 The +BATT line tells you less than you want, and that is honest

Battery sense is now observable: the flight controller emits `batt=<state>` and
`vbat=<volts>` in its telemetry line, and `rb-status.sh` renders it. Read what it
actually measures before you build a decision on it.

**`vbat=` is the +BATT RAIL VOLTAGE. It is not a pack-presence flag, and on this
board it cannot be made into one.** From `riskybirdv3.kicad_sch`: U4 is an
MCP73831-2 charger with VDD on +5 V (USB VBUS, J8) and its VBAT output wired
*directly* to +BATT. The pack connector J7 lands on that same net. So with USB
plugged in and **no pack fitted**, U4 charges the +BATT bulk capacitance
(C2/C3/C40/C41 4×47 µF plus C48 10 µF, ≈200 µF) to its 4.20 V float and
terminates — and the ADC reads a textbook healthy battery.

> **An empty board on USB and a fully charged pack produce the same number.**

The five states the screen can show, and what each one licenses:

| screen | means | what you may do |
| --- | --- | --- |
| `ENERGISED 4.19 V` | the motor rail is live | nothing in tier 2 or 3. Assume a motor can move. |
| `<low> V` | below the energised threshold | still not proof of anything; see below |
| `STALE <v> V` | polls are failing, the number is frozen | treat as ENERGISED. A power fault can present as a healthy unmoving voltage. |
| `NO DATA` | sense compiled in, ADS7128 never answered | treat as ENERGISED |
| `NOT MEASURED` | `ROSE_BATT_SENSE=0`, not compiled in | treat as ENERGISED |
| `UNKNOWN` | no battery field in the captured telemetry | treat as ENERGISED |

Note how the table collapses: **every state except a genuinely low reading means
"assume a motor can turn"**, and even a low reading is a statement about a rail,
not about a connector. The sentinels are designed for exactly this — `vbat=-1.000`
(not compiled in) and `vbat=-2.000` (never got a sample) contain a minus sign
that the status tool's digit-matching scrape deliberately fails to parse, so
"I have no idea" can never render as a confident green "absent".

So the honest conclusion for a remote operator is stronger than "check the
battery line":

> **"I did not see a battery" is not "there is no battery." And now:
> "I saw a low rail" is not "there is no pack" either.**
>
> No field in telemetry can tell you the pack is disconnected. The one
> discriminator that exists is load response — with no pack the rail collapses
> the instant a motor draws current, because U4 can source only ~500 mA into
> ~200 µF — and deliberately loading the motors to find out is precisely what a
> remote unattended operator does not get to do.

And ~200 µF at 4.2 V is not nothing. It is a twitch, not a flight, but a twitch
from a bare board on USB is still a propeller moving in a room with nobody in it.

What this means in practice:

1. **Stay in tier 1.** It is a large tier. Most work fits. This is not a
   fallback; it is the normal state of remote work.
2. If you need tier 2, the gate is a **human statement with a timestamp** — a
   message from whoever was last at the bench saying the pack is out — not a
   number on a screen. Put it where you will find it again:
   ```bash
   echo "$(date -Is) pack OUT of J7, confirmed by <name> in <where>" \
       >> ~/riskybird-logs/sessions/current/session.txt
   ```
   A note you can re-read is worth more than a memory of a message.
3. Treat a power cycle, a replug, or any gap in the console log as invalidating
   that statement. Somebody was at the bench.
4. Use the rail reading for what it is good at: **catching the case where you
   thought the bench was dead and it is not.** `ENERGISED` when you expected
   nothing is real information and should stop you. `low` when you expected a
   pack is a reason to go and look, not a permission.

The failure all of this prevents is specific and easy: you left the bench on
Friday with the pack out, someone plugged it in to charge on Monday, and on
Tuesday you run "the same harmless motor test as last week" from 500 km away.

### 4.2 The 5-second latch is what protects an unattended bench

With no special flags, the flight controller lets motors run for **5 s from the
first instant a non-zero duty would be written**, then latches all four OFF until
the board is reset — whatever the controller asks for, whatever the arm gate
says. Widening it takes **two independent acts, and neither alone does
anything**: `-DROSE_FLIGHT_COMMIT=1` at build time (without it the symbol
`g_flight_commit_key` is not even in the image, and `commit.gdb` fails with "No
symbol", which *is* the interlock) and `hardware/flight/commit.gdb` at run time.

For a remote operator this 5 s is the difference between a mistake and an
incident. It is the one bound that holds when you are not there.

> **Do not quietly raise `ROSE_MOTOR_WINDOW_MS`, and do not build with
> `-DROSE_FLIGHT_COMMIT=1` "so it is ready for later".** A committed build is one
> `commit.gdb` away from a 60 s budget, and a build that sits on the bench
> committed is a build somebody will boot without reading the banner. If you
> genuinely need a longer run, that is a tier 3 conversation with a person in the
> room, not a flag.
>
> Check before you walk away. This must print **nothing**:
>
> ```bash
> riscv64-zephyr-elf-nm build/zephyr/arty200t/flight_controller/esp-motors/zephyr/zephyr.elf \
>     | grep g_flight_commit_key
> ```

---

## 5. Running work, and leaving it running

### 5.1 The loading recipe, because `rb run` does not work here

`RocketArty200TDroneDDRConfig` and `RocketArty200TDroneFullDDRConfig` are
elaborated `uartTsi = false` and their devicetree has no TSI node, so `rb run`
and `rb load` have nothing to talk to. Use `rb debug`, which loads the ELF over
the same JTAG link the debugger attaches on.

And **attach the console first**. `rb debug` starts the program immediately; the
console is a different channel. Everything printed before you attach is gone,
which is how a perfectly good UART gets blamed for a workload that "printed
nothing".

Remotely, the console is already attached — pane 2 has held it since the session
started. That is most of what `rb-watch.sh` is for. So the remote recipe is just:

```bash
# in the WORK pane. The console pane is already attached and already logging.
./rb debug <workload> --target arty200t \
    --config RocketArty200TDroneFullDDRConfig --program \
    --program-usb-location <bitstream-location> --usb-location <debug-location> \
    --no-run
(gdb) continue
```

Get the two locations from the status screen or `rb bench`; never copy them from
an old command, because they renumber on every replug.

`--program` writes the bitstream to the SPI flash, persistently, before it loads
SRAM, so it changes what the board boots after the next power cycle. Drop it if
the shell is already loaded, or program SRAM alone as `setup.md` describes (the
note under 3a); `rb fly` is SRAM-only.

If the console still misbehaves, `rb debug --ram-console` buffers the console
into RAM and reads it back over the same JTAG link — the fallback for a board
whose UART is unproven, and for short workloads that finish before anything can
attach.

### 5.2 Leaving it

Detach with `ctrl-b d`, or just close the ssh connection. Both consoles keep
capturing, the status pane keeps refreshing, and the work pane keeps whatever was
running.

Before you disconnect, check these four things:

1. **The status screen's `commit gate` line says `UNCOMMITTED`** (or nothing is
   running). If it says `COMMITTED`, reset the board before you leave — a
   committed build with a 60 s budget is not something to leave behind.
2. **`LOGS / last write` is moving.** If it is not, nothing you look at later
   will tell you anything.
3. **Nothing in tier 2 or 3 is mid-run.** A `--mode motors` workload should be
   finished, or the board reset, before you walk away. "It will time out on its
   own" is a claim about software you are about to stop being able to watch.
4. **No GDB session is parked at a prompt.** A halted core does not stop the PWM
   path: the SiFive comparators are separate hardware and keep driving whatever
   they hold. Every gdb script in `hardware/flight/` ends with `monitor resume`
   for this reason, and never Ctrl-C out of one.

Long builds survive detaching by themselves — that is what the persistent session
buys you. Put them in the work pane, detach, come back.

---

## 6. Logs: rotation, session markers, and reading them later

### The layout

```
~/riskybird-logs/
  fpga-console.log     -> sessions/<current>/fpga-console.NNN.log   (always current)
  esp-live.log         -> sessions/<current>/esp-live.NNN.log
  .rb-bench-cache         the adapter table, probed once per session
  sessions/
    current           -> <the newest session directory>
    2026-09-22T15-09-35/
      session.txt        host, repo, git rev, which devices, and rb bench output
      fpga-console.001.log
      fpga-console.002.log      rolled at RB_LOG_MAX_MB (default 32 MB)
      esp-live.001.log
    2026-09-21T09-14-02/
      ...
```

Three properties, each of which exists because of a specific failure:

- **A session per `rb-watch.sh` run.** Yesterday is a directory you can ignore
  rather than the first 400 MB of the file you are trying to tail. The status
  tool reads the tail, so unbounded growth is not only a disk problem — it is a
  grep that takes a minute and a `tail -c` that reads through junk.
- **Parts capped at 32 MB**, rolled automatically, with the stable symlink
  re-pointed at the newest. Override with `RB_LOG_MAX_MB`. The bench host's root
  filesystem is at 90% — a console left capturing over a long weekend is a real
  disk risk, not a theoretical one.
- **Parts are per session, not per attach, and a re-attach appends.** This one is
  subtle and it matters: a console flap (a USB blip, a board reset, a replug)
  must not move the current-log symlink to a fresh empty file, because the status
  screen would instantly forget the battery reading, the commit gate and every
  recent event, none of which had stopped being true. A remote operator must not
  lose the picture because a cable twitched.

Sessions older than `RB_LOG_KEEP_DAYS` (default 14) are deleted whole at the next
`rb-watch.sh` start. Whole, never partly — a half-deleted session reads as a gap
in the evidence, which is worse than a big directory.

### Session markers

Every attach writes a marker line into the log:

```
==== 2026-09-22 15:09:36  FPGA console ATTACH  dev=/dev/ttyUSB5 ====
==== 2026-09-22 15:10:10  FPGA console ATTACH  dev=/dev/ttyUSB5 ====
```

So `grep -a '^====' <log>` is that session's table of contents, and two markers
close together are a flap — a device that went away and came back, which is
itself evidence worth having when you are trying to explain a gap.

### Reading what happened while you were away

```bash
# which sessions exist, and when each one's console was alive
tools/bench/rb-status.sh -s

# what was plugged into a session, and at what git revision
cat ~/riskybird-logs/sessions/2026-09-22T15-09-35/session.txt

# the whole session in order (parts glob-sort correctly)
cat ~/riskybird-logs/sessions/2026-09-22T15-09-35/fpga-console.*.log | less

# the things that would have made you look up, if you had been there
grep -an 'EMERGENCY\|CUTOFF\|WINDOW\|GUARD\|ESTOP\|FAIL' \
    ~/riskybird-logs/sessions/2026-09-22T15-09-35/fpga-console.*.log

# the ESP's side of the same minutes
grep -a 'cuts=\|link=\|MOTOR CMD\|MOTOR GATE' \
    ~/riskybird-logs/sessions/2026-09-22T15-09-35/esp-live.*.log | tail -40

# gaps: the console stopped printing between these markers
grep -a '^====' ~/riskybird-logs/sessions/*/fpga-console.*.log
```

Read the ESP log and the FPGA log together. "Commanded" and "delivered" are
different claims: `MOTOR CMD duty per mille` is what the FPGA asked for,
`MOTOR GATE duty per mille` is what `motor_safe` measured at the gate. A
disagreement between them is the most informative thing in either file.

---

## 7. If you are remote and something is spinning

Read this before you need it.

### What you cannot do

- **You cannot pull the battery.** It is the only stop that needs none of our
  software to work, and it is the only one you do not have.
- **The dashboard ESTOP is a kill only with the right pair of images.** With
  `esp_bridge --mode motors` on the ESP it latches `kill=1` and zeroes the gates
  at the ESP, independent of the FPGA, and a flight controller built with
  `ROSE_UART_CMD` (every flight preset) hears it too. With `esp_motors`, which
  has no uplink, or a flight controller without that knob, the bridge's
  `ACK ESTOP` means only "delivered to the FPGA UART" and nothing stops. Check
  `kill=1` in `RBLINK` before relying on it, and remember it needs the Wi-Fi link
  to still be up.
- **You cannot count on `estop.gdb` or `stop.gdb`.** They are fine scripts, but
  they need a debugger *already attached*. Attaching OpenOCD and halting takes a
  second or two over JTAG, and that is on top of you noticing, deciding, and
  typing. They are backstops, not emergency stops.

### What you can do, fastest first

| # | what | how long | needs |
| --- | --- | --- | --- |
| 1 | **Stop the frame stream.** Kill the process driving it, halt the core, anything that stops frames leaving the FPGA. The ESP cuts all four after 120 ms — measured, `cuts=1`. | ~125 ms once it stops | the ESP firmware working |
| 2 | Wait for the commitment window. Uncommitted, all four latch off 5 s after they first spin — measured, `5009 ms`. | ≤5 s from first spin | the FPGA image working |
| 3 | The envelope guard: tilt >40°, free fall, impact, rate, velocity, height, battery. | 40–120 ms dwell + one tick | the FPGA image working |
| 4 | `hardware/flight/stop.gdb` — zeroes the PWM comparators, latches `g_estop`, resumes 300 ms so an explicit zero-duty ESTOP frame goes out, then halts. | 1–2 s + 0.3 s + 120 ms | a debug adapter, free |
| 5 | `hardware/flight/estop.gdb` — latches `g_estop` and resumes immediately. | 1–2 s | a debug adapter, free |

Every single one depends on software still working. That is the whole reason
tier 3 exists.

### If it is spinning and you are not there

1. **Do #1 immediately.** Stopping the frames is the fastest thing you have and
   it does not need the adapter to be free.
2. **Call whoever is nearest the bench and ask them to pull the battery.** Do
   this in parallel, not after. A person in the room beats everything in the
   table above.
3. Then #4 or #5, if the debug adapter is free. If `rb debug` is already
   attached in the work pane, you are in the good case: `set var g_estop = 1`
   then `monitor resume` at the existing prompt, which skips the 1–2 s of
   attaching.
4. **Do not Ctrl-C a GDB session with motors armed.** On the PWM path a halted
   core leaves the comparators driving the pins with no software able to
   intervene. Measured during motor bring-up: outputs stayed asserted after a
   plain detach.
5. Afterwards, `hardware/flight/reset.gdb` clears the latches without re-enabling
   arming — the deliberate split, so "clear the latches" never means "allow
   takeoff".

And then write down what happened, in the session directory, while it is fresh.

---

## 8. Practical notes for the remote case

**USB numbering.** `/dev/ttyUSB*` numbers and USB locations renumber on **every**
replug. Re-derive with `rb bench` (or `rb-status.sh -p`); never copy a number out
of a doc or an old shell line. The location is stable across a USB reset,
`bus:device` is not.

**USB depth.** Two FT2232Hs three hubs deep drop out in pairs on this bench, and
ESP flashing fails outright when the part is several hubs deep — measured: at
three hubs / 12 Mbps every write died mid-erase and left the part blank. If a
remote flash fails, that is the first thing to suspect, and it needs a person to
reseat into a shallower port. The C6's USB-JTAG is in silicon, so a blank part
still enumerates and is always recoverable — a failed remote flash is not a dead
board.

**`dmesg` tells you about the bus.** A device that vanished from the status
screen and a device that was never there look identical on the screen and
completely different in `dmesg -T | tail -40`. Check it before concluding
anything about hardware.

**Short workloads print nothing** if you attach after they run — but on this
setup the console pane attached before anything started, so you are already past
that. If a workload is short enough to finish inside the JTAG load, use
`rb debug --ram-console`.

**Motor and flight tests do nothing useful on USB-C alone.** That cuts both ways:
it is why tier 2 is safe, and it is why a remote "motor test" that appears to
pass has proved nothing about the motors.

---

## 9. Changes made to the tools for this workflow

All found by testing the scripts with no hardware attached, in a real tmux
session. (The `+BATT rail` rendering comes from the battery-sense work, not from
this change set; the `commit gate` line below sits beside it.)

- **`split-window -p` was removed in tmux 3.4.** On this host (tmux 3.4) it
  failed with `size missing`, every following `send-keys` failed with
  `can't find pane`, and `rb-watch.sh` came up as **one bare shell with no status
  pane and no consoles, capturing nothing** — while printing a cheerful "creating
  session". Now uses `-l <pct>%` with `-p` as the fallback for tmux < 3.1.
- **The status pane froze on its first render.** `timeout` runs its child in a
  new process group; OpenOCD under `rb bench` reads the inherited tty stdin; a
  background process group that reads the terminal gets SIGTTIN and stops.
  `python3 ./rb bench` parked in state `T` and the pane sat on the word
  `ADAPTERS` forever. A frozen status screen looks exactly like a calm bench,
  which makes this the worst possible failure for a remote tool. Fixed twice
  over: the refresh loop no longer probes at all, and every `rb` call redirects
  stdin from `/dev/null`.
- **`debug?` read as ABSENT.** `rb bench` labels the role `debug?` when the
  debug TAP shifts all ones, which is what an *unconfigured* part does — the
  exact state you are in before you program it. Both scripts matched only
  `debug`, so `rb-watch.sh` pointed the FPGA console pane at a device that did
  not exist during the window you most want the boot banner, and `rb-status.sh`
  printed `debug link ABSENT` for an adapter that was plugged in and working.
  Both now match on the prefix, as `rb` itself does.
- **`*COMMITTED*` also matches `UNCOMMITTED`.** The commit-gate line read the
  banner `COMMIT GATE -- UNCOMMITTED BUILD: motors latch OFF 5000 ms...` as
  `COMMITTED -- 60 s motor budget`: a 5 s safety default reported as a flight-test
  commitment, the wrong way round, on the one line that bounds an unattended
  bench. `UNCOMMITTED` is now tested first.
- **`stat` and `du` do not follow symlinks.** With the logs now reached through
  symlinks, `stat -c %Y` returned the link's own mtime, so every freshness check
  reported "no recent output" while data was arriving, and `du -h` reported 0 for
  a 40 MB file. Now `stat -Lc` and `du -hL`.
- **A `commit gate` line on the status screen**, read out of the FPGA log, so a
  remote operator can see whether the running build has the 5 s budget or the
  60 s one without attaching anything.
- **Session directories, markers, size-capped parts and pruning**, as in §6.
- **`-p`** re-probes the adapters once, deliberately. **`-s`** lists past
  sessions. **`RB_FPGA_DEV` / `RB_ESP_DEV`** override device resolution, for a
  bench where `rb bench` cannot resolve a role and for testing with no hardware.
- **`rb-watch.sh` started without a terminal** (`ssh host tools/bench/rb-watch.sh`)
  now creates the session, says how to attach, and exits 0 instead of failing
  after having done the whole job.

---

## Related

- `docs/esp-motor-offload.md` — what was measured on hardware: `cuts=1`, the
  47.457° guard trip, the 5009 ms window; the full record is
  `docs/history/esp-motor-offload-handoff.md` §4.
- `docs/history/flight-safety-bench-procedure.md` — the 2026-09-21 bench procedure
  for the tilt guard, the commitment gate and every way to stop the motors.
- `docs/tutorial-flight.md` — `rb fly stop` and the flight-day procedure.
- `docs/flight-calibration-procedures.md` §3.4 — why `rb run` does not work on
  these shells and what to use instead.
- `hardware/flight/*.gdb` — each script's header explains what it does, what it
  cannot do, and why it resumes the core.
