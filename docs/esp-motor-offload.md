# Motor PWM on the ESP32-C6

How the drone's four motors are driven today, and the measurements that put
them there. The full bench record is
[`history/esp-motor-offload-handoff.md`](history/esp-motor-offload-handoff.md)
(2026-09-21); this page keeps what is still true.

## The arrangement

The FPGA runs all of the flight logic and **only commands**. It sends a duty
frame over its second UART to the on-board ESP32-C6, which owns all four motor
gates and does nothing but apply the duties it is sent. The FPGA drives no motor
pin at all.

| side | image | build |
| --- | --- | --- |
| FPGA | `flight_controller --mode esp-motors`, or a preset built on it | `rb fly commit --mode <preset>` ([`tutorial-flight.md`](tutorial-flight.md)) |
| ESP32-C6 | `esp_bridge --mode motors`: Wi-Fi telemetry bridge **plus** the four gates, the 120 ms failsafe and a local ESTOP | `rb build esp_bridge --target esp32c6 --mode motors`, then `rb flash` with the same arguments |

`esp_bridge --mode motors` replaces two earlier ESP images for flying:
`esp_motors` (the first offload receiver, gates only, no Wi-Fi) and
`motor_safe` (which parks the four gate pins as pulled-down inputs so the FPGA
can drive them). `motor_safe` is still the right image for the FPGA-PWM
diagnostics and the gate probe in `hardware/padtest/`; it and any motors image
drive the same four pins in opposite directions and cannot coexist.

- **Link.** `serial@10021000` on the FPGA, `uart1_rxd` E13 / J1_96 and
  `uart1_txd` F14 / J1_98, bank 16. **115200 baud, measured**: 693 frames and,
  in a separate run, 1212 frames, both with `lost=0 stale=0 crcerr=0 noise=0`.
  A flight image today runs ~247 frames/s on it.
- **Wire format.** 14 bytes: magic `0xA5 0x5A`, seq, flags, 4 x uint16 duty in
  units of 1/10000, CRC-16/CCITT-FALSE. One copy, compiled by both ends:
  `integration/zephyr/motor_link/include/riskybird/motor_link.h`.
- **ESP pins.** MOTOR1..4 = GPIO21 / GPIO20 / GPIO23 / GPIO22, LEDC at 20 kHz.
- **Duty ceiling at the gate.** `ESP_LINK_MAX_DUTY_PCT`, 98 in the motors
  mode: just above the flight controller's own 0.95 cap, so it backstops a
  mis-built FPGA image without ever being the limit that binds.

## Why: FPGA ball F13 is dead as an output

Motor 4's gate is driven from FPGA ball F13 (`F13 -> J1_100 -> JB1 99 -> R26
47R -> /MOTOR4 -> Q4 gate`). Measured 2026-09-21 with two bare-metal Verilog
bitstreams in `hardware/padtest/` -- no Rocket, no TileLink, no PWM IP, no
Zephyr -- read back through the ESP's high-impedance gate probe (`motor_safe`):

| test | result |
| --- | --- |
| drive the four pads at 4 / 8 / 12 / 16 % | `M1=40 M2=80 M3=120 M4=0` per mille: three exact, F13 nothing |
| F13 as an **input** with the internal pullup (`hardware/padtest/f13probe/`) | a solid 0: the carrier's 10k gate pulldown is reachable from the ball, so the net is intact |

So the break is the F13 output driver (or its IOB), not the wiring, R26, the
board-to-board contact or Q4. It is not a bank fault -- bank 16 also carries
motor1 and motor2, which work -- and the constraints and post-route structure
match the working motors. Reseating the TE0712 did not help. Motor 4 worked on
2026-09-09, so this is a fault that developed. /MOTOR4 reaches the FPGA only at
F13, so no RTL change or pin remap can fix it; the ESP reaches the same gate
through its own 47R without crossing the board-to-board connector.

Moving all four motors (not just motor 4) to the ESP also removes the
dual-driver contention on the other three gates: each gate is an FPGA 47R and an
ESP 47R into one 10k pulldown, and two drivers disagreeing park it near the
SI2302 threshold.

## What is measured on this path

All on the drone, battery connected, 2026-09-21 unless noted.

- **The ESP boots safe.** Before any frame arrives it reports all four at 0 and
  `link=DOWN -- MOTORS OFF (failsafe)`; the failsafe deadline starts expired.
- **Motor 4 turns again**, from an FPGA command through the ESP.
- **The failsafe cuts.** Halting the Rocket core stopped the frame stream and
  the ESP cut all four within its 120 ms timeout, reporting `cuts=1`. On the old
  FPGA-PWM path a halted core leaves the SiFive comparators driving the gates at
  the last duty until a reconfigure; here a halt, crash, reset or pulled ribbon
  cuts the motors in at most ~120 ms. `rb fly stop` relies on this.
- **The tilt guard fires**: `*** EMERGENCY CUTOFF *** tilt limit exceeded:
  measured 47.457 deg est-tilt, held 141 ms over 5 samples`, and the ESP cut
  all four.
- **The commit window latches** at 5009 ms with `flags=0x10[WINDOW-EXPIRED]`.
- **Dashboard ESTOP is a real kill** with this ESP image and a flight
  controller built with `ROSE_UART_CMD` (every preset sets it): the ESP latches
  `kill=1` and zeroes the gates itself, independent of the FPGA. Confirm
  `kill=1` in `RBLINK`; the green ACK alone means "delivered", not "acted on".
  This pairing flew in trials T01-T42, 2026-09-23/24.

## Mistakes not to repeat

- **`state[3..5]` are Gibbs (Rodrigues) parameters, `tan(theta/2)`, not
  radians.** Telemetry `roll=-0.949` is 87 degrees, not 54. A guard named
  `SAFE_MAX_TILT_RAD = 1.0` and commented "~57 deg" really tripped at 90, and a
  hand tilt to 87 missed it. The estimator was right throughout; the limits now
  convert once through `tilt_rad_from_gibbs()` and compare radians with
  radians.
- **`ROSE_ACTUATE_TIMEOUT_MS` defaulted to 0, which compiled the bench cutoff
  out.** The commit gate (`ROSE_FLIGHT_COMMIT`, `hardware/flight/commit.gdb`)
  is what bounds motor time now.
- **There are no ESCs.** Four brushed motors on low-side SI2302 FETs, gate
  active-high, 20 kHz, 47R series, 10k pulldown. Any diagnosis involving
  arming, calibration or locked-rotor latching describes hardware this drone
  does not have.
- **Motors run from +BATT.** On USB-C alone the least-stiff motors twitch and
  the stiffest does not, which looks exactly like one dead motor. A silent motor
  with no battery is not evidence of anything.
