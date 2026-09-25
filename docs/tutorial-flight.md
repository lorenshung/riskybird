# Flying the RiskyBird FPGA drone from one machine

The whole session -- build, load, fly, publish the logs -- from one laptop (developed on a Mac and
a Linux bench). Chipyard/Vivado are not needed: the FullDDR bitstream comes from
`bitstreams/` (or `$RB_BITSTREAM_DIR`), and everything else builds locally.

What is verified on the drone, 2026-09-25: `rb fly load`, `commit`, `status`, `stop` and `where`,
and `rb flash esp_bridge --target esp32c6 --mode motors`. `rb fly imu` and `rb fly motors` are
marked below: they have **not been run on hardware yet**.

## What you need

- The drone (TE0712 carrier, ESP32-C6), its programmer board (two FT2232H adapters), a USB-C cable,
  and **charged 1S LiPos** (>= 4.1 V at rest; the image refuses to arm below 3.95 V).
- `source ./rb-env.sh`, then `rb doctor` showing `ok` for the Zephyr backend and SDK, west, the
  RISC-V compiler, OpenOCD and GDB. `rb doctor` has no openFPGALoader line; run
  `openFPGALoader --version` for that. On a bench-tier checkout `rb doctor` **exits nonzero**
  because Chipyard, its environment, Espresso, Vivado and (unless you took the models tier)
  ModelBlaster are reported missing. That is expected: none of them is needed here.
- A ground-station machine on the drone's Wi-Fi. It can be the same laptop; see
  [The ground station](#the-ground-station) below.

**The shell goes to SRAM, not flash, for now (2026-09-25).** `rb fly` programs the FullDDR shell
into SRAM on every `load` and `commit`, because the only FullDDR bitstream (built 2026-09-03)
predates the TE0712 SPI-boot properties and never configures from flash. A power blip therefore
leaves the board unconfigured until the next `rb fly commit`. See
[`setup.md`](setup.md) (the note under 3a) for the measurement and what fixing it would take.

## Presets (what flew)

`rb workloads` lists them as `flight_controller` modes. Each is `esp-motors` plus the exact values
in `tools/flight/presets/<mode>.env`:

| mode | flew | notes |
|---|---|---|
| `t21-hover-1.0m` | T20-T22 | **the best flight: 9.2 s, no overshoot, tilt <= 9 deg, 0.24 m drift. Start here.** |
| `t16-hover-0.6m` | T14, T16 | 7.3-7.7 s at 0.6 m |
| `t27-hover-1.6m` | T27 | climb + hold at 1.6 m; needs open floor (floor-step handling on) |
| `t23-hover-1.5m` | T23, T24 | ~12 s at 1.5 m, +0.3 m overshoot (BATT_NOMINAL_V 4.0); prefer t27 |

The presets reproduce the **knobs** those flights were built with. The firmware underneath has
moved since (a merge, and the `--no-relax-gp` link fix in zephyr-chipyard-sw `834b9c6` without
which t21/t23 do not link), so no preset built today is byte-identical to the ELF that flew.
`t21-hover-1.0m` built and loaded through `rb fly` boots and commits with the ESP link at
~247 frames/s; it has not been flown in that form yet.

## Once per drone (and after any ESP firmware change)

```sh
rb build esp_bridge --target esp32c6 --mode motors     # WiFi bridge + the four motor gates, cap 98 %
rb flash esp_bridge --target esp32c6 --mode motors     # banner: "duty ceiling 98%", "SoftAP up"
```

This image owns the four motor gates; the FPGA drives none of them and sends duty frames over
uart1 ([`esp-motor-offload.md`](esp-motor-offload.md)). It **replaces `motor_safe`** on the ESP,
the image the FPGA-PWM bench checks in `setup.md` use; the two drive the same pins in opposite
directions, so only one can be on the ESP at a time.

## Before the first flight of the day (pack OUT, props off or on, nothing spins)

> **Not yet run on hardware.** `rb fly imu` and `rb fly motors` are written and unit-tested but
> have not been run on the drone, and the `motor_duty` image `rb fly motors` loads has never run
> on hardware. Treat the first run of each as a test of the command itself, and be ready to pull
> the pack.

```sh
rb fly imu --pack-out          # guided tilt test: level / nose-up / right-wing-down / nose-left
rb fly motors --duty 20        # PACK IN for this one: each motor alone ~1.5 s; you check the corner,
                               #   spin direction, and that air blows DOWN (1 FR, 2 BR, 3 BL, 4 FL)
```

`rb fly motors` asks you to type `spin`. It is the one command here that turns a motor.

## Each flight

```sh
rb fly commit --mode t21-hover-1.0m --pack-out
```

This programs the FullDDR shell into SRAM, builds and loads the preset, waits for the boot banner and
for the ESP link (~247 motor frames/s, worst gap well under the 120 ms failsafe), writes the commit
key, and prints `READY` with the build record. The session logs are in `results/fly/<time>/`.
The commit budget is 60 s of motor time from the first spin-up, so two or three RESETs fit in one
commit. After that the motors latch off, and you run `rb fly commit` again.

Then:

1. Pack in. Start the ground station and wait for GO (`rb ground gonogo`).
2. Pull the JTAG ribbon, then USB-C. **Watch both streams for 10 s.** If they stop, the board reset
   in the handover: plug USB-C back in and run `rb fly commit` again.
3. Put the drone on a level floor with open space around and under it. Press **RESET** on the panel:
   it arms after ~3 s and flies the preset's profile, then lands and cuts.
4. **ESTOP** (button, or the E key) stops it at any time. It is a real kill with this pairing: the
   ESP latches `kill=1` and zeroes the gates itself, and every preset builds the flight controller
   with `ROSE_UART_CMD` so the FPGA hears it too. Do not use DISARM: it can re-arm.
5. `rb flightlog push --build-file <the rb-build.txt it printed> --note "what you saw"`.

`rb fly stop` halts the core from the bench (the ESP cuts all motors 120 ms later); `rb fly where`
shows where the core is; `rb fly status` summarises the ESP link from its console (opening that
console can reset the ESP, so never while flying).

## The ground station

The drone's ESP runs a SoftAP (`riskybird-XXXX`, the drone at `192.168.4.1`). Telemetry arrives on
UDP :14550 and commands go to :14551. `rb ground` wraps the scripts in `tools/bench/`:

| command | what | platform |
|---|---|---|
| `rb ground up` / `down` / `status` / `verify` | join, leave and check the drone's SoftAP | **macOS only** |
| `rb ground ready [--no-join]` | keep the Mac awake, join once, start the panel on http://127.0.0.1:8080 and the monitor | **macOS only** (it uses `caffeinate` and `ipconfig`) |
| `rb ground monitor` | the monitor alone: owns :14550, logs every datagram to `results/riskybird-logs/raw_*.log`, relays to the panel on :14560, prints GO / NO-GO | macOS and Linux |
| `rb ground gonogo [--watch]` | one line from the running monitor; exits 0 only on a fresh GO | macOS and Linux |

**On macOS**, internet must come from a non-Wi-Fi path (iPhone USB or Ethernet); `rb ground up`
refuses without one. Turn off Wi-Fi auto-join for other networks during tests: every ESP reset
drops the SoftAP for ~8 s and macOS otherwise jumps to another network and stays there. Run
`rb ground up` once after every ESP reset (reload, USB unplug, crash).

```sh
rb ground ready        # panel + monitor; leave it running
rb ground gonogo       # GO needs FC + RBLINK streaming, tilt <= 3, COMMITTED, not WINDOW-EXPIRED
```

**On Linux** there is no up/down to wrap: the drone's SoftAP goes on a **second Wi-Fi adapter**,
set up once with a NetworkManager profile that then joins by itself. The flights so far were run
from macOS; this path is what the scripts do, not yet flown.

```sh
G=backends/zephyr-chipyard-sw/samples/rose_flight_controller/tools/groundstation
$G/riskybird_wifi_setup.sh [ssid] [iface]            # once
python3 -u $G/riskybird_panel.py --drone 192.168.4.1 --telem-port 14560 --cmd-port 14551 \
    --http 8080 --log-dir results/riskybird-logs &   # the panel, fed by the monitor's relay
rb ground monitor
rb ground gonogo
```

Keep the monitor running for the whole session: its raw log is what `rb flightlog push` splits
into trials, and it is the only record of a reset in the handover. The `RBLINK` fields are
explained in [`wireless-telemetry-bringup.md`](wireless-telemetry-bringup.md) section 6.

## Known issues

- **The FR motor (MOTOR1, J3) is dead** after the T26/T27 crashes (T31, T41, T42 and later trials
  tipped at spin-up with it at the 0.95 cap). Replace it before flying, then check all four with
  `rb fly motors`, and repeat that check after any crash.
- **Pulling the cables sometimes resets the board a few seconds later** (the FPGA loses its shell).
  Not explained. The programmer board feeds the drone's +3V3 through J1.2/J1.10 and is the first
  suspect. Keep the ground-station raw log running so the moment is recorded.
- **The SPI flash does not boot the FullDDR shell**; SRAM programming works, which is why `rb fly`
  uses it (above).
- **Height over furniture.** The down-ToF floor-step handling (`TOF_STEP_REJECT_M`, on in t27)
  was added after T26 ran away over a desk; T27 still came down badly after a pass over one. Fly
  over open floor.
- **One side ToF is missing** (the drone is meant to carry three, two answer), so `bootup_check`'s
  ToF WARN is a real fault. The down sensor, the one flight uses, is unaffected.
- On macOS, pass `--console /dev/cu.usbserial-...` to `rb fly` if the FPGA console is not found
  automatically (the drone's adapters have blank EEPROM serials).
