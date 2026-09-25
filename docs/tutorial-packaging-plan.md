# From remote bench to in-person tutorial: packaging plan

Status: proposal (2026-09-24), written at the end of the T01-T42 flight campaign. Nothing here is committed yet.

## Goal

Fly the RiskyBird FPGA drone **in person at a tutorial, from one local machine**, with a clean,
reproducibly buildable `riskybird-dev` on the `setup` branch. Replace the campaign's split setup
(a remote Linux box built and programmed; a Mac joined the drone's WiFi and recorded) and its
hand-typed environment-variable soup with named `rb` commands and presets.

## What the campaign actually used (inventory)

**Firmware (uncommitted, `backends/zephyr-chipyard-sw`, branch `dev` on lorenshung/zephyr-chipyard-sw):**
`samples/rose_flight_controller` -- 7 files, +632/-37. Every change is a knob whose default keeps
builds that have flown byte-identical:

| change | knob | evidence |
|---|---|---|
| uplink command reader over uart1 (ESTOP/DISARM/RESET/...) + junk resync | `ROSE_UART_CMD` | dashboard control |
| non-blocking ESP telemetry, console telemetry off | `ROSE_TELEM_CONSOLE` | loop 57 -> ~1145 Hz, frames 247/s |
| dt from the cycle counter | (always) | ms-resolution dt was 0 above 500 Hz |
| attitude seeded from gravity at init/RESET | (always) | T01 armed believing a 9 deg tilt was level |
| yaw-authority clamp | `YAW_AUTH_FRAC` | T10 upset |
| left-pair trim as a knob | `MOTOR_TRIM_LEFT` | T10/T11 -6 deg bank drift |
| altitude conditional integration | `ALT_INT_BAND` | T18 +0.39 m overshoot -> T21 none |
| down-ToF floor-step handling, armed only | `TOF_STEP_REJECT_M`, `TOF_STEP_HOLD_S` | T26 desk runaway |
| gyro step guard + `gzpk/apk/grej` telemetry | `GYRO_STEP_MAX` | T10/T27 diagnosis |
| yaw mixing sign | `YAW_MIX_SIGN` | **keep at +1** (the -1 hypothesis was wrong) |
| battery report-only split | `ROSE_BATT_REPORT_ONLY` | unmetered vbat |

**Setup tree (uncommitted):** `workloads/esp_bridge` duty cap as a cache var (`ESP_LINK_MAX_DUTY_PCT`),
`tools/rb/boards.py` esp-motors mode, camera changes, `docs/remote-bench-operation.md`, `tools/bench/rb-*.sh`.
**Also staged by another session: deletion of `integration/zephyr/safety/motor_guard.*` -- confirm before merging.**

**New in this pass:** `tools/flight/presets/*.env` (the four configurations that flew best, verbatim) and
`tools/flight/legacy/` (the campaign scripts, verbatim, as the reference for the new subcommands).

**Ground station (Mac, being packaged separately):** `tools/bench/riskybird-dash.py`,
`tools/bench/riskybird-wifi-macos.sh`, and in `lorenshung/flight-logs`: `record.py`-style raw recording,
`split_trials.py`, `sign_test.py`.

## Proposed `rb` additions (in priority order for the tutorial)

1. **Flight presets as `rb` modes, not environment variables.** Every campaign build was
   `RB_CMAKE_ARGS=... RB_EXTRA_CPPFLAGS=... ./rb build ... --mode esp-motors`, and silently wrong
   flags were the most common failure (`rb debug` rebuilding without them; a `-D` sent down the channel
   a file does not read). Add `cppflags_by_mode` to `boards.Workload` (or promote the `#ifndef` knobs to
   CMake cache vars, the tree's existing convention), then register:
   `hover-0.6m` (T16), `hover-1.0m` (T21), `hover-1.6m` (T27 + fixes), `bench-dry`, `imu-test`.
   Each mode records the trials it flew. Print the effective knobs at build time and in the boot banner.
2. **`rb flight` (replaces flight.sh):** `program` (FullDDR to SRAM; `--flash`), `load`, `commit`
   (always fresh SRAM, waits for the ESP link, verifies the ELF did not change), `status` (RBLINK:
   link rate, gap, kill, committed, window), `stop`, `where`, `dryrun`, `imu` (guided tilt test that
   checks the six signs itself). Adapters resolved like `rb bench` does -- no hardcoded USB locations.
3. **`rb esp flash esp_bridge --mode motors --cap N`:** register esp_bridge modes `default`/`motors`
   (the overlay + motors.conf pair) and expose the duty cap; resolve the port by-id.
4. **`rb motors test`:** register `motor_duty` in boards.py (drop `tools/rb-ext.py`) and drive it:
   `--motor 1..4|all --duty 20 --ms 1500`, with an explicit typed confirmation (props, pack, stop path).
5. **`rb bench log start|stop|tail`:** timestamped FPGA + ESP console capture that survives
   re-enumeration. Note: opening the ESP port resets the ESP (USB-Serial-JTAG); the logger must say so.
6. **`rb release flight`:** bundle bitstream + ESP image + one ELF per flight preset + a manifest (hashes,
   knobs, source revisions), so the tutorial laptop needs no Vivado and no rebuild.

## Tutorial flow (target)

```
rb doctor                                  # tools, adapters, pack voltage via telemetry
rb esp flash esp_bridge --mode motors      # once per ESP
rb flight commit --preset hover-1.0m      # SRAM program + load + link check + commit
<ground station: join riskybird-XXXX, record, dashboard>
unplug ribbon, then USB-C; RESET on the dashboard; ESTOP = E
```

Open questions for the tutorial host: the laptop OS (macOS needs openocd/openFPGALoader with libftdi,
and the Zephyr SDK if it is to build), and whether it builds or only runs `rb release` artifacts.

## Merge plan with the Mac side

1. Firmware: commit the submodule changes on the lorenshung fork (branch to confirm), bump the pointer.
2. Setup: presets + `rb` subcommands + docs, one commit per item; ground-station code under
   `tools/groundstation/` (code) while `flight-logs` keeps only data plus its analysis scripts.
3. Reproducibility check: fresh clone -> `rb doctor` -> build every preset -> compare ELF hashes with
   the manifest.

## Known issues to carry into the tutorial

- **FR motor (MOTOR1, J3) is dead** after the T26/T27 crashes and T31/T41/T42 stalls. Replace, then
  `rb motors test`.
- **Intermittent resets a few seconds after the cables are pulled** (FPGA loses its shell). Not explained;
  capture the ground-station raw log at the moment it happens. The programmer board feeds the drone's
  +3V3 through J1.2/J1.10 -- a suspect worth isolating.
- The SPI flash holds FullDDR but does not boot it to a working SoC; the SRAM program works.
- Height estimate over furniture: handled in flight by the floor-step logic; still fly over open floor.
