# Handoff: ground station → controller session (merge, then one machine)

From the Mac ground-station session (2026-09-23/24, trials T01–T56) to the controller session that
builds and loads the flight firmware. Goal: **build, load, fly and publish logs from one machine.**

## 1. What is on `origin/setup` now

```
73161b0  HANDOFF-PROMPT: correct the telemetry divisor advice          <- where both sessions started
f28d624  Bench: a flight-ready monitor that owns :14550, and one command to start it
ed69839  rb: `ground` and `flightlog`, so a flight session runs from rb like build and debug do
<this>   docs: HANDOFF-ground-station.md
```

| Path | What it is |
|---|---|
| `tools/bench/riskybird-flightready.py` | The monitor. It owns UDP :14550 and writes every datagram (FC lines **and** RBLINK) to `results/riskybird-logs/raw_*.log`, flushing each one. It relays to the panel on 127.0.0.1:14560, prints a live GO / NO-GO, writes `ready-status.json`, and detects stale lease vs off-air. It never sends to :14551. |
| `tools/bench/riskybird-ready.sh` | caffeinate, `riskybird-wifi-macos.sh up` once, the v3 panel on the relay port, then the monitor. `--no-join`, `--auto-rejoin` (off by default). |
| `tools/rb/ground.py` + `rb ground {ready,up,down,status,verify,gonogo,monitor}` | Wrappers for the above. `gonogo` exits 0 only on a fresh GO. |
| `tools/rb/flightlog.py` + `rb flightlog {push,show,compare}` | Publishes to the flight-logs repo (`$RB_FLIGHT_LOGS`, default `~/flight-logs`). A byte-compatible port of that repo's `tools/split_trials.py` and `tools/add_trial.py`, plus: trials already published keep their number, matched by ARMING time. |
| `tests/test_ground.py`, `tests/test_flightlog.py` | unittest, 16 tests, all passing. |
| `.gitignore` | Adds `results/`. Run output stays local, and flight logs go to the flight-logs repo. |

**Not changed:** the `backends/zephyr-chipyard-sw` gitlink is still `4af12b9` (Sept 21). **No firmware
change from this session is in `setup`.** This checkout has `backends/chipyard` and `hardware/pcb`
missing from its working tree. That is local only and was deliberately not committed. It is also why
`test_registry_scratchpad_flags_match_the_chipyard_configs` errors here. Don't mirror it.
`docs/session-2026-09-21-mac-bench.md` stays uncommitted, and its author marked it so.

## 2. Merging into your checkout

```
git fetch origin && git merge origin/setup        # or rebase your local commits onto it
```
- Conflicts can only come from `tools/rb/cli.py` (if you edited it) or `.gitignore` (append-only).
  The cli.py change is three self-contained blocks: `flightlog, ground` in the `from . import (...)`
  list, `cmd_ground` / `_flightlog_build_text` / `cmd_flightlog` just above `cmd_hil`, and the
  `ground` / `flightlog` parsers right after the `hil replay` parser. Keep both sides.
- Then `python3 -m unittest tests.test_ground tests.test_flightlog`.

## 3. What only your session can push (blocks one-machine flying)

1. **The flight firmware.** Every setting flown since T12 (ATT_GAIN 0.85, TILT_SLEW, PID_MASS,
   ALT_INT_BAND, YAW_AUTH_FRAC 0.15, MOTOR_TRIM_LEFT, ROSE_VEL_LOOP/VEL_TILT_MAX, PITCH_TRIM_RAD 0.045,
   BATT_NOMINAL_V 3.8, BATT_ARM_MIN_V 3.95, TOF_STEP_REJECT_M 0.25 / TOF_STEP_HOLD_S 0.15, the
   climb/hold/descend profile, SAFE_MAX_HEIGHT_M, FLIGHT_MAX_MS) exists only in your working copy of
   `zephyr-chipyard-sw`. Commit it there and push it to that repo's remote. Then **bump the gitlink in
   `setup`** and push. Without this, another machine builds the Sept 21 firmware.
2. **`flight.sh`.** Every reload and commit went through `/tmp/claude-2629/.../scratchpad/flight.sh`,
   which is in no repo. Move it to `tools/bench/flight.sh`, with no absolute or `/tmp` paths and USB
   locations derived from `rb bench`, or make it `rb fly commit`. Include the exact
   `rb build flight_controller --mode ... --config ...` line it uses.
3. **Build knobs as data**, if possible: a small file per build (e.g. `configs/flight/T27.conf`, or the
   `RB_EXTRA_CFLAGS` it expands to) so `rb flightlog push --build-file` can record exactly what flew.

## 4. The one-machine loop (once §3 is in)

```
source ./rb-env.sh
rb build flight_controller --mode esp-motors --config <drone config>   # your recipe from flight.sh
tools/bench/flight.sh commit      # -> later: rb fly commit (reload FC + fresh commit window)
rb ground up                      # once after EVERY ESP reset (reload, USB unplug, crash)
rb ground ready --no-join         # panel + monitor; or `rb ground ready` to also join
rb ground gonogo                  # GO needed: FC+RBLINK streaming, tilt<=3, COMMITTED, not WINDOW-EXPIRED
# press RESET in the panel (http://127.0.0.1:8080); ESTOP or the E key stops it
rb flightlog push --build-file build.txt --note "what the operator saw"   # ALWAYS before analysis
rb flightlog show T57 ; rb flightlog compare T21 T57
```
On the Mac: internet must come from iPhone USB or Ethernet (`up` refuses otherwise). Turn off Wi-Fi
Auto-Join for eduroam and the iPhone hotspot during tests: every ESP reset drops the SoftAP for about
8 s, and macOS jumps to eduroam and stays there.

## 5. Where flight testing stands (details in flight-logs README / trial summaries)

- **Best flights:** T14/T16 (0.6 m) and **T21 (1.0 m, 9.2 s, no overshoot, 0.24 m drift)**. T23/T24 flew
  about 12 s at 1.5 m, with a +0.3 m overshoot.
- **Fixed:** the right>left spin-up offset (T02–T11 drift), the T05/T06 thrust shortfall, and the T26
  ToF-step runaway (TOF_STEP_REJECT; worked in T27).
- **Open, and blocking:** since the T26/T27 crashes, **T31, T41, T42 (and T44, T48, T56) tip on the
  floor at spin-up with m0 (FR) at the 0.95 cap while the tilt grows toward that corner.** Suspect a
  damaged FR motor or prop. Inspect before tuning anything else. The earlier "yaw sign" hypothesis
  (T31 summary) is contradicted by T41/T42.
- **Open:** a steady ~+2.5–2.7° mean pitch that PITCH_TRIM +0.045 did not move (T23/T24). Altitude
  overshoot at 1.5–1.8 m. Battery sag, 4.1 → 3.1 V in a 12 s flight (BATT_ARM_MIN 3.95 blocks most
  second flights). The ESP resets on USB unplug and on impact, and takes the FC with it.
- T32–T40, T43, T45–T47 and T49–T55 are RESET presses that never armed. They are kept, but ignore them for tuning.
