# Handoff prompt

Paste the block below into a fresh Claude Code session on the machine that has
the drone attached. It is written to be self-contained: it names the one document
to read, states the constraints that are not negotiable, and says what to do
first and what not to do at all.

Everything in it is either measured on hardware or explicitly marked unverified.
Do not soften it — the value is in the parts that say "this has never been run".

---

```
You are picking up a drone project mid-flight-campaign. The drone is physically
connected to this machine and is TETHERED to cables, so it cannot fly away, but
it HAS PROPELLERS ON and its motors WILL SPIN AT FLIGHT-LIKE POWER.

READ THIS FIRST, before touching anything:
    docs/esp-motor-offload-handoff.md
and then:
    docs/tethered-flight-attempt.md

Both are in this repo, on the `setup` branch. They were written on the Linux
build host by the session that did the work, for you specifically. The handoff
doc marks every claim as measured / verified / declared / unverified. Section 5
is the unverified list; read it properly rather than skimming.

The one-paragraph version: FPGA ball F13, which drove motor 4's gate, has a dead
output driver. The net to the FET is intact -- this was proven with a bare-metal
Verilog bitstream that reads `M1=40 M2=80 M3=120 M4=0` per mille at the gates,
and a second bitstream with F13 as an input reading the carrier's 10k pulldown.
There is no RTL fix. So ALL motor PWM was offloaded to the on-board ESP32-C6: the
FPGA runs the whole flight stack and sends 14-byte duty frames over uart1 at
50 Hz, and the ESP owns all four gates and does nothing but apply them. Motor 4
turns again. That offload is measured working -- 693 frames, zero CRC/noise/loss.

THREE CONSTRAINTS FROM THE OWNER. These are not suggestions and you must not
relax them to make something work:

  1. THE WATCHDOG IS ALWAYS ON. The envelope guard (accel tilt, free-fall,
     impact, estimator tilt, body rate, velocity, height) must be active
     whenever a motor can turn. If you cannot prove it is live, do not spin a
     motor. The telemetry line carries `tilt=estNdeg/accNdeg |a|=X`; tilting the
     airframe by hand with the battery OUT must move `acc`. If it does not move,
     the guard is blind -- stop and fix that before anything else.

  2. NEVER TOO POWERFUL, NEVER TOO LONG. There is a duty cap and a commitment
     gate. Motors latch off 5 s after they first spin unless the build was made
     with `-DROSE_FLIGHT_COMMIT=1` AND `hardware/flight/commit.gdb` was run.
     Do not raise the duty cap beyond what the tethered procedure specifies, and
     do not remove the time bound. Nobody has measured this airframe's thrust
     curve, so every duty number in the tree is calculated, not measured.

     READ THIS BEFORE YOU TOUCH A DUTY NUMBER -- it is the single most likely
     thing to waste your first session:

     (a) THERE ARE TWO INDEPENDENT DUTY CEILINGS AND RAISING ONE DOES NOTHING.
         The FPGA has MOTOR_MAX_DUTY / AUTOFLIGHT_MAX_DUTY. The ESP separately
         applies ESP_MOTORS_MAX_DUTY_PCT, default 25%, as the LAST thing before
         the gate -- deliberately, because it does not trust the FPGA. Raise
         only the FPGA cap and you are silently clamped to 25% at the gate, and
         it looks exactly like a thrust failure or a tuning problem. Both must
         be raised in the same build-and-flash session.

     (b) AUTOFLIGHT_MAX_DUTY = 0.45f MAY BE BELOW HOVER. The tree contains three
         conflicting hover duties: 0.583 (the controller's LINEARISATION point,
         duty = (u + 0.583f) * batt_scale -- not a measurement), 0.71 (observed
         in flight, FLIGHT_TUNING_LOG.md), and 0.15 (a bare comment beside
         AUTOFLIGHT_MAX_DUTY that is numerically identical to
         MOTOR_BREAKAWAY_DUTY and is almost certainly a copy-paste error).
         Recorded flights ran 0.95. If hover is really ~0.65 then an autoflight
         build at 0.45 cannot lift at all -- the drone will simply sit there.
         If that happens it is NOT necessarily broken, and the wrong reaction is
         to start raising limits in a hurry. Measure the thrust curve first:
         docs/flight-calibration-procedures.md has the procedure and the
         motor_duty tool exists for exactly this.

     (c) The 5 s commitment window exists ONLY in flight_controller.
         workloads/motor_duty is a standalone image with no commit gate. Do not
         assume "motors always latch off after 5 s" across every workload.

     (d) IF YOU ENABLE TELEMETRY, PASS ROSE_TELEM_DIV=80 OR HIGHER. The binary
         motor frames and the ASCII telemetry line share uart1 at 115200
         (11520 B/s). With ROSE_FLOW=1 a telemetry iteration is 286 B / 24.8 ms.
         `--mode esp-motors` sets no telemetry knobs, so it inherits DIV=10
         ~= 77 Hz ~= 22 kB/s -- nearly TWICE the wire. The console write is
         POLLED and busy-waits, so a saturated wire directly delays motor
         frames: worst-case gap is about 80 ms against the 120 ms ESP failsafe.
         Push the rate up and you can cause spurious failsafe cuts mid-flight,
         which look like a flaky link rather than what they are.

  3. EMERGENCY STOPS ARE ALWAYS AVAILABLE. Fastest first: pull the battery
     (instant); stop sending frames, which makes the ESP cut all four in 120 ms
     (measured); halt the Rocket core, same effect; then the gdb stop scripts.
     Never start a powered run without knowing which one you will reach for.

OPERATING RULES learned the hard way in the previous session:

  - ASSUME THE BATTERY IS LIVE. A ~40 s powered run happened because the
    previous session believed the motors were inert. They were not.
  - Bound every powered run. Do not start something open-ended and watch it.
  - Flashing `esp_motors` REPLACES `motor_safe`, which is the gate-probe
    instrument every motor measurement came from. They drive the same four pins
    in opposite directions and cannot coexist. Reflash `motor_safe` to measure
    gates again.
  - ESP flashing fails when the part sits several USB hubs deep -- at 3 hubs /
    12 Mbps every write died mid-erase and left it blank. Move it to a shallower
    port. The USB-JTAG is in silicon, so a blank part still enumerates.
  - You do NOT need Vivado and you do NOT need a new bitstream. The FPGA design
    is unchanged; `RocketArty200TDroneFullDDRConfig` sha256 starts `8b21446a`.
    Zephyr builds natively on macOS.

START HERE, in this order:

  1. `source ./rb-env.sh`, then confirm `rb bench` sees the adapters. USB
     locations renumber on every replug; re-derive rather than copying any
     number out of a document.
  2. Build and run with the BATTERY DISCONNECTED first. Confirm the boot banner
     shows the envelope guard and the commit gate, and that the ESP reports
     `link=UP` with `crcerr=0 noise=0`.

     NOTE: `rb run` does NOT work on the drone shells -- they are elaborated
     `uartTsi = false` and have no TSI node, so it cannot load anything. The
     recipe that works is: attach `rb console` FIRST (there is a console-attach
     race -- a workload that prints early will have finished before a console
     started afterwards ever sees it), then `rb debug --program --no-run`, then
     `continue`. Also note `--config` is effectively mandatory: the flow overlay
     references `&spi0`, which only exists on a drone shell, so a bare
     `rb build flight_controller --mode esp-motors` dies with
     `undefined node label 'spi0'`.
  3. Prove the guard can see: battery out, tilt the frame by hand, watch `acc`
     in the telemetry follow.
  4. Only then connect the battery, and follow docs/tethered-flight-attempt.md.

DO NOT, without checking with the owner first:
  - raise any duty cap beyond the tethered procedure's value
  - disable, bypass or loosen the envelope guard or the commitment gate
  - commit or push anything (the owner asks before every commit, and wants no
    AI attribution in commit messages)
  - attempt an untethered flight

THINGS THAT ARE KNOWN-UNVERIFIED -- treat any of these behaving oddly as
expected, not as a new mystery:
  - optical flow has NEVER successfully answered on this carrier through this
    code path. Watch for `flow: PMW3901 up` on boot. If you see
    `flow: PMW3901 not detected`, `CS_GPIO_PIN=0` is wrong and flow is dead
    weight -- say so rather than working around it.
  - PID + EKF + flow together have never been run on hardware in this mode.
  - hover duty and break-away duty through the ESP path are unmeasured.
  - THE GROUND-STATION / DASHBOARD ESTOP BUTTON DOES NOTHING in the current
    build. Every rose_cmd_* hook in main.cpp is inside `#if defined(CONFIG_WIFI)`
    and telem_wifi.c is not in this checkout, so the bridge fabricates the "ACK"
    itself. DISARM / HOVER_Z / PING are inert. You do NOT have a remote kill.
    Do not let anyone believe otherwise. (A merged esp_bridge --mode motors
    image implements a real ESP-side kill, but it has never been flashed, so
    meter the gates the first time it is used.)
  - the camera returns no colour. Three investigations agree no Bayer mosaic
    arrives; where the defect physically originates is CONTESTED and unresolved.
    A software repair ships in `hardware/ospi/frame-to-png.py`. The owner
    believes the fitted sensor is a colour part; the data does not show chroma.

WORKING STYLE THE OWNER EXPECTS:
  - Use subagents freely for parallel work; they cannot touch hardware, you own
    the bench.
  - Report what you measured, quoting real output. Do not report a result you
    inferred as though you observed it.
  - When a previous conclusion turns out to be wrong, say so plainly and move
    on. Several diagnoses in the last session were wrong and the corrections
    were the most useful part of it.
```
