# Flying the RiskyBird FPGA drone from one machine

The whole session -- build, load, fly, publish the logs -- from one laptop (developed on a Mac and
a Linux bench). Chipyard/Vivado are not needed: the FullDDR bitstream comes from
`bitstreams/` (or `$RB_BITSTREAM_DIR`), and everything else builds locally.

## What you need

- The drone (TE0712 carrier, ESP32-C6), its programmer board (two FT2232H adapters), a USB-C cable,
  and **charged 1S LiPos** (>= 4.1 V at rest; the image refuses to arm below 3.95 V).
- `source ./rb-env.sh`, then `rb doctor` clean for the Zephyr SDK, OpenOCD and openFPGALoader.
- A ground-station machine on the drone's Wi-Fi. It can be the same laptop; see `rb ground`
  and docs/HANDOFF-ground-station.md. macOS needs a non-Wi-Fi internet path (`rb ground up` refuses
  without one).

## Presets (what flew)

`rb workloads` lists them as `flight_controller` modes. Each is `esp-motors` plus the exact values
in `tools/flight/presets/<mode>.env`:

| mode | flew | notes |
|---|---|---|
| `t21-hover-1.0m` | T20-T22 | **the best flight: 9.2 s, no overshoot, tilt <= 9 deg, 0.24 m drift. Start here.** |
| `t16-hover-0.6m` | T14, T16 | 7.3-7.7 s at 0.6 m |
| `t27-hover-1.6m` | T27 | climb + hold at 1.6 m; needs open floor (floor-step handling on) |
| `t23-hover-1.5m` | T23, T24 | ~12 s at 1.5 m, +0.3 m overshoot (BATT_NOMINAL_V 4.0); prefer t27 |

## Once per drone (and after any ESP firmware change)

```sh
rb build esp_bridge --target esp32c6 --mode motors     # WiFi bridge + the four motor gates, cap 98 %
rb flash esp_bridge --target esp32c6 --mode motors     # banner: "duty ceiling 98%", "SoftAP up"
```

## Before the first flight of the day (pack OUT, props off or on, nothing spins)

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

1. Pack in. Start the ground station (`rb ground ready`) and wait for GO (`rb ground gonogo`).
2. Pull the JTAG ribbon, then USB-C. **Watch both streams for 10 s.** If they stop, the board reset
   in the handover: plug USB-C back in and run `rb fly commit` again.
3. Put the drone on a level floor with open space around and under it. Press **RESET** on the panel:
   it arms after ~3 s and flies the preset's profile, then lands and cuts.
4. **ESTOP** (button, or the E key) stops it at any time. Do not use DISARM: it can re-arm.
5. `rb flightlog push --build-file <the rb-build.txt it printed> --note "what you saw"`.

`rb fly stop` halts the core from the bench (the ESP cuts all motors 120 ms later); `rb fly where`
shows where the core is; `rb fly status` summarises the ESP link from its console (opening that
console can reset the ESP, so never while flying).

## Known issues

- Pulling the cables sometimes resets the board a few seconds later (the FPGA loses its shell).
  Not root-caused; the programmer board feeds the drone's +3V3 through J1.2/J1.10 and is the first
  suspect. Keep the ground-station raw log running so the moment is recorded.
- The FR motor (MOTOR1, J3) failed after the T26/T27 crashes. Check all four with `rb fly motors`
  before flying after any crash.
- On macOS, pass `--console /dev/cu.usbserial-...` to `rb fly` if the FPGA console is not found
  automatically (the drone's adapters have blank EEPROM serials).
