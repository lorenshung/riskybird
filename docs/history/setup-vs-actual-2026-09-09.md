# RiskyBird bench setup: `docs/setup.md` versus the actual run

> **Historical record (2026-09-09), not instructions.** One cold-clone run of
> `setup.md`; its findings were folded into [`../setup.md`](../setup.md), which supersedes it.
> Known-wrong claims: "the expected two-absent ToF population warning" (the drone is meant to
> carry three ToFs and only two answer, so that WARN is a real fault); motor 4 turning was true
> that day, but FPGA ball F13 has since died ([`../esp-motor-offload.md`](../esp-motor-offload.md)).
> `$WORK` below stands for that run's scratch workspace.


Run date: 2026-09-09  
Cold-clone commit: `8ddc99c` on branch `setup`  
Workspace: `$WORK/riskybird-dev`
Tier: bench only (Zephyr builds, programming, and attached-board checks)

The propellers were confirmed removed before either motor run. The battery was
connected and the drone was secured. No repository was modified outside
`$WORK`, and nothing was committed or pushed.

## Outcome

- Every bench-tier `rb doctor` item is clean. Its exit status is still 1 because
  the intentionally skipped silicon tier leaves the Chipyard backend,
  Chipyard environment, and Espresso missing.
- `hello_world` built at
  `build/zephyr/arty200t/hello_world/default/zephyr/zephyr.elf`: 19,424 B RAM
  out of 1 GiB.
- `bootup_check` built in both modes. Before the safety patch the default image
  used 149,232 B RAM and the motor image used 151,024 B. The patched motor
  image uses 151,280 B.
- `RocketArty200TDroneFullDDRConfig` was written to the S25FL256S, explicitly
  read back to 100% with `openFPGALoader --verify`, and then loaded into FPGA
  SRAM.
- The final whole-board motor pass reported 7 passed, 1 expected ToF warning,
  0 skipped, 0 failed, and `BOARD READY`.
- The operator confirmed the post-fix physical sweep worked. All four logical
  motor lines matched physical motion. No pre-countdown blast was reported on
  the post-fix run.
- The final camera artifact was `bootup-frame.png` (a run output, not committed): 324 ×
  315, 8-bit grayscale, pixel range 0–255. It is a visually coherent,
  non-uniform photograph and is not row-sheared. The raw capture is 102,690
  bytes.

## Ordered deviations and findings

This is the complete ordered list of places where the literal setup page and
the actual run diverged, including environmental failures that were retried.
Expected branches that the page already documents are called out separately so
they are not misrepresented as product defects.

1. **`rb-env.sh` found old side repositories during what was intended to be a
   from-scratch run.** The page says that the first source should leave both
   variables empty:

   ```console
   $ echo "[$RB_DTS_DIR] [$RB_BITSTREAM_DIR]"
   [] []
   ```

   Actual: the host already had `$HOME/riskybird-dts` and
   `$HOME/riskybird-bitstreams`, so `rb-env.sh` exported those paths.
   They were unset for the cold setup. Fresh side repositories were cloned
   inside the requested workspace and every later shell explicitly used:

   ```console
   export RB_DTS_DIR=$WORK/riskybird-dts
   export RB_BITSTREAM_DIR=$WORK/riskybird-bitstreams
   source ./rb-env.sh
   ```

2. **The page's home-directory writes conflicted with the run's workspace-only
   constraint.** Step 2b says it edits `~/.condarc`,
   `~/.conda/environments.txt`, and `~/.wget-hsts`; step 2c registers the SDK
   under `~/.cmake`; steps 2d and 3c clone under `~`. The installers were run
   with their home redirected to `$WORK/cold-home`, and the side
   repositories were instead cloned as:

   ```console
   git clone git@github.com:lorenshung/dts.git $WORK/riskybird-dts
   git clone git@github.com:lorenshung/bitstream.git $WORK/riskybird-bitstreams
   ```

   This was required by the test constraint, not by RiskyBird itself.

3. **The literal first `conda deactivate` failed in the non-interactive
   shell.** Exact output:

   ```text
   CondaError: Run 'conda init' before 'conda deactivate'
   ```

   The page documents this branch. Its prescribed recovery was followed:

   ```console
   source "$(dirname "$(dirname "$(command -v conda)")")/etc/profile.d/conda.sh"
   conda deactivate
   ```

   The Zephyr environment was then installed inside the repository. The
   optional `gym-pybullet-drones` step did not block this Linux run; per the
   operator, a failure there was allowed to be skipped.

4. **Linux group changes could not be made effective in the running agent
   session.** The documented command succeeded:

   ```console
   sudo usermod -aG plugdev,dialout "$USER"
   ```

   `/etc/group` then named the user in both groups, but the page requires a
   logout/login to refresh supplementary groups and that was not possible in
   this session. Hardware commands therefore had to be run with direct device
   access.

5. **The page's adapter-discovery command was unsafe on this shared host.** It
   recommends `rb bench` and later shows unscoped `rb probe-ftdi --idcode`;
   both scan every MPSSE adapter, including another user's Alveo U250. Per the
   host-specific safety requirement, the actual sequence was always:

   ```console
   ./rb probe-ftdi
   ./rb probe-ftdi --idcode \
       --location 7-1.1.4.1 --location 7-1.1.4.3
   ```

   The live table supplied those locations. The scoped result explicitly said
   `5-2 (skipped, not requested)` and identified:

   ```text
   7-1.1.4.1  0x00000001  Rocket debug TAP -- Zephyr/GDB link
   7-1.1.4.3  0x13636093  xc7a200t config TAP -- bitstream link
   ```

6. **A location-scoped IDCODE probe without direct device access failed but
   exited successfully.** Exact result:

   ```text
   7-1.1.4.1  no IDCODE returned
   7-1.1.4.3  no IDCODE returned
   ```

   Exit status was 0, which makes this a false-success interface. The same
   location-scoped command with device access returned the two correct
   IDCODEs. No unscoped scan was used as a workaround.

7. **The documented USB-reset helper raised a raw traceback without device
   access.** For the two live devices it ended in:

   ```text
   FileNotFoundError: [Errno 2] No such file or directory: '/dev/bus/usb/007/010'
   FileNotFoundError: [Errno 2] No such file or directory: '/dev/bus/usb/007/009'
   ```

   The same reset was rerun with direct USB access and succeeded. This should
   be a concise permission error, not a traceback.

8. **The reset renumbered the debug console.** The first table showed debug
   channel B as `/dev/ttyUSB9`; after reset it was `/dev/ttyUSB1`. The table was
   re-read and the live node was used. This behavior is expected for USB, but
   demonstrates why copying the page's example `/dev/ttyUSB5` is not valid.

9. **`rb console` raised a raw traceback when its stdin was not a TTY.** Exact
   terminal error:

   ```text
   termios.error: (25, 'Inappropriate ioctl for device')
   rb: command failed with exit status 1
   ```

   The exact same command was rerun in an allocated pseudo-terminal and then
   worked. `rb console` should reject non-TTY use with a concise actionable
   message.

10. **Clean debug runs printed a contradictory GDB rejection.** Both initial
    board-check runs and the final patched run contained:

    ```text
    Error: attempted 'gdb' connection rejected
    Info : accepting 'gdb' connection on tcp/3333
    ```

    The following connection loaded the ELF successfully. The OpenOCD USB
    interface was not stale, the capture contained 102,690 valid bytes rather
    than zero, and current register values agreed with the console. Therefore
    the troubleshooting section's implication that this message identifies a
    stale OpenOCD is not reliable for this observed sequence.

11. **Programming was initially deferred until propeller state was explicitly
    confirmed.** The setup order would have programmed before the whole-board
    run. Because this shell exposes motor outputs from configuration onward,
    programming was held until the operator confirmed the propellers were off
    and the drone secured. It then completed successfully.

12. **`rb program` writes flash but does not request flash verification.** The
    documented command reached `Erasing: 100.00%`, `Writing: 100.00%`, then
    loaded the image into SRAM, but its openFPGALoader invocation has no
    `--verify`. To substantiate the stronger claim that flash contents were
    verified, this additional command was required:

    ```console
    openFPGALoader --board te0712_8 --busdev-num 7:9 \
        --write-flash --verify \
        --bitstream $WORK/riskybird-bitstreams/arty200t/arty200t.RocketArty200TDroneFullDDRConfig.bit
    ```

    It printed `Verifying write (May take time)`, read flash to 100%, and
    printed `Done`. Because that operation temporarily loads the SPI-over-JTAG
    bridge, the same bitstream then had to be loaded into SRAM again before
    running the workload.

13. **The original motor run exposed a real startup hazard not covered by the
    page's countdown.** After ELF launch but before the formal sensor sequence
    and countdown, two unidentified motors briefly ran at high power and
    stopped. The later announced 15% sweep was clean and all four lines matched
    motion. Because the FPGA and ESP32-C6 both land on the same four MOSFET
    gates with no arbitration, the unannounced motion was not attributed to a
    particular logical output.

14. **The captured pixel range did not match the page's reference.** The page
    gives a reference range of 0–190. Both the initial and final captures were
    324 × 315 with 102,690 raw bytes, but the actual range was 0–255. Visual
    inspection confirmed a real, coherent photograph rather than a uniform or
    sheared buffer.

15. **The ESP target described the wrong flash capacity for the populated
    part.** The first `motor_safe` flash identified an `ESP32-C6FH4` with 4 MiB
    but warned:

    ```text
    WARNING: Set flash_size 8MB is larger than the available flash size of 4MB.
    ```

    `hardware/zephyr/targets/esp32c6/target.overlay` now overrides the devkit's
    8 MiB declaration to the measured 4 MiB. An incremental `--pristine auto`
    rebuild regenerated the DTS but retained the old Kconfig value and warning;
    `--pristine always` was required once to refresh it. The final build reports
    a 4 MiB region and the final flash has no capacity warning.

16. **The final flow-sensor console contained one inconsistent diagnostic
    line.** Verbatim:

    ```text
    PMW3901 chip ID: 0x92, inverted: 0xB6
    [  OK  ] PMW3901 flow         chip id 0x49; dx=+0 dy=+0 SQUAL=41 shutter=7978
    ```

    The stage's validated ID is the expected `0x49` and the stage passed, but
    the preceding `0x92` text does not agree with it. This was recorded rather
    than investigated because the requested work was motor-safety hardening.

## Implemented motor-safety fixes

### Fix 1: passive ESP32-C6 motor owner

A new ESP-only `motor_safe` workload was added. Its devicetree GPIO hog makes
GPIO21, GPIO20, GPIO23, and GPIO22 inputs with pull-downs as soon as the GPIO
driver is ready. LEDC is explicitly disabled. The application reasserts the
same input/pull-down configuration and emits a five-second heartbeat over the
native USB serial/JTAG console.

This is deliberately high-impedance rather than output-low: during the planned
FPGA sweep the FPGA must be able to drive a gate high without fighting an ESP
output driver.

Final flash evidence:

```text
Chip type:          ESP32-C6FH4 (QFN32) (revision v0.2)
Features:           ... Embedded Flash 4MB
Writing ... 100.0%
Verifying written data...
Hash of data verified.
Hard resetting via RTS pin...
```

Live runtime evidence:

```text
MOTOR SAFE heartbeat: 4/4 inputs+pulldowns
MOTOR SAFE heartbeat: 4/4 inputs+pulldowns
MOTOR SAFE heartbeat: 4/4 inputs+pulldowns
```

### Fix 4: park FPGA PWM before `main`

The motor-mode `bootup_check` now registers
`park_motors_before_main` with `SYS_INIT(..., APPLICATION, 0)`. That runs after
the PWM devices initialize but before `main` starts any memory or sensor stage.
It forces zero duty on all four channels and records a bit mask. Stage 8
re-parks the channels and refuses to sweep if the pre-main mask is not `0x0f`.

Hardware evidence from the final run:

```text
         4/4 channels ready, 4/4 parked at 0% duty
         pre-main park mask 0x0f (expect 0x0f)
```

## Final whole-board console output (verbatim)

```text
=== RiskyBird bootup check -- Sep  9 2026 15:33:10 ===
One pass over every peripheral, in boot order.

[  OK  ] main memory (DDR3)   4 MiB at 0x82000000 verified address-in-address
         0x17  ADS7128 expander     ACK
         0x18  BMI088 accel         ACK
         0x68  BMI088 gyro          ACK
         0x24  HM01B0 camera        ACK
         0x76  barometer            ACK
         0x29  VL53Lxx at default   --
[  OK  ] I2C inventory        5 answered; every fixed part present
[  OK  ] ADS7128 expander     SYSTEM_STATUS=0x81 (register framing accepted)
         ch1 side  VL53L5CX   at 0x31 (id 0xf002)
         ch2 side  -- absent (err -116)
         ch3 side  -- absent (err -116)
         ch6 down  VL53L1X    at 0x34 (id 0xeacc)
[ WARN ] ToF array (XSHUT)    2 up, 2 absent -- down sensor present
         |a| = 9748 mm/s^2 (expect 9807 at rest)
[  OK  ] BMI088 IMU           accel |a| = 9748 mm/s^2 within 5% of g; gyro reads
         core holds 104977 px, frame is 105624 -- capturing 315 of 324 rows (2048 px stop margin)
         model 0x01b0, 326x272 measured (324x244 configured), min=0x00 max=0xff
[  OK  ] HM01B0 camera        102690-byte frame captured; reshape rows at 326
PMW3901 chip ID: 0x92, inverted: 0xB6
[  OK  ] PMW3901 flow         chip id 0x49; dx=+0 dy=+0 SQUAL=41 shutter=7978
         4/4 channels ready, 4/4 parked at 0% duty
         pre-main park mask 0x0f (expect 0x0f)

  Motors are parked. Spinning them needs the battery connected and
  the propellers OFF -- USB-C alone will not turn a motor, so a
  silent motor here means the battery, not a fault.
  sweeping in 3 ... (Ctrl-C to stop)
  sweeping in 2 ... (Ctrl-C to stop)
  sweeping in 1 ... (Ctrl-C to stop)

         watch which motor moves as each line prints:
         motor1 -> 15% for 1000 ms ... stopped
         motor2 -> 15% for 1000 ms ... stopped
         motor3 -> 15% for 1000 ms ... stopped
         motor4 -> 15% for 1000 ms ... stopped
[  OK  ] PWM motors           4/4 driven to 15% in turn and returned to 0%

--- summary ---
[  OK  ] main memory (DDR3)   4 MiB at 0x82000000 verified address-in-address
[  OK  ] I2C inventory        5 answered; every fixed part present
[  OK  ] ADS7128 expander     SYSTEM_STATUS=0x81 (register framing accepted)
[ WARN ] ToF array (XSHUT)    2 up, 2 absent -- down sensor present
[  OK  ] BMI088 IMU           accel |a| = 9748 mm/s^2 within 5% of g; gyro reads
[  OK  ] HM01B0 camera        102690-byte frame captured; reshape rows at 326
[  OK  ] PMW3901 flow         chip id 0x49; dx=+0 dy=+0 SQUAL=41 shutter=7978
[  OK  ] PWM motors           4/4 driven to 15% in turn and returned to 0%

0 stage(s) failed, 1 warned, 0 skipped, 7 passed.
BOARD READY

102690 bytes in pixels[] -- see hardware/ospi/bootup-capture.gdb
```

## Physical motor observations

The operator confirmed the fixed sweep worked after being asked to watch for
motion before the countdown and match each announced line:

| Announced line | Physical observation |
| --- | --- |
| `motor1 -> 15% for 1000 ms` | Corresponding motor turned, then stopped |
| `motor2 -> 15% for 1000 ms` | Corresponding motor turned, then stopped |
| `motor3 -> 15% for 1000 ms` | Corresponding motor turned, then stopped |
| `motor4 -> 15% for 1000 ms` | Corresponding motor turned, then stopped |

The motors were not labeled by physical corner in the available instructions,
so the evidence supports logical line-to-motion matching, not a named
front-left/front-right/rear-left/rear-right map.

## Artifact evidence

```text
bootup-frame.png: PNG image data, 324 x 315, 8-bit grayscale, non-interlaced
bootup-frame.raw: 102690 bytes
pixel range: 0..255
PNG SHA-256: c8a8a775fd6b35184e3d3248d7b1d6201a86cecb57e6169dd2cb8950746eaa31
RAW SHA-256: 57ef08e75cb3a33eec0969a37c1349e86dd4551d9c94c2e4571559b3885ef2a9
```

Supporting logs were kept in that run's workspace (`$WORK/run-logs/`), outside the
repository, and are not committed:

- final board console: `run-logs/bootup-console-fixed.log`
- final debug/GDB capture: `run-logs/debug-bootup-motors-fixed.log`
- FullDDR `rb program`: `run-logs/program-full-ddr-fixed-run.log`
- FullDDR explicit flash verification: `run-logs/verify-full-ddr-flash.log`
- final ESP flash: `run-logs/flash-motor-safe-final.log`
- ESP motor-safe console: `run-logs/motor-safe-console.log`
- final doctor: `run-logs/doctor-final.log`

## Verification limits

- The SPI flash was read back and compared, and the image was loaded into SRAM,
  but the board was not physically power-cycled. Autonomous cold boot from
  flash is therefore not claimed.
- The ESP GPIO hog, successful `gpio_pin_configure_dt` calls, repeated runtime
  heartbeats, lack of a reported pre-countdown blast, and a clean FPGA sweep
  are strong software/bench evidence. No oscilloscope or independent GPIO
  register readback was used to prove electrical high impedance.
- GPIO hogging begins after the ESP GPIO driver initializes. It cannot control
  the ESP ROM/reset interval. A production safety design still needs hardware
  arbitration or a gate-enable that defaults off; the software fixes reduce
  the observed startup hazard but do not turn two tied push-pull owners into a
  safety-rated interface.
- The expected two-absent ToF population warning was not treated as a fault.
- The transient `PMW3901 chip ID: 0x92` diagnostic mismatch was not diagnosed.
- Silicon-tier Chipyard/Vivado build commands and the tooling unit-test suite
  gated on Chipyard were intentionally not run.
