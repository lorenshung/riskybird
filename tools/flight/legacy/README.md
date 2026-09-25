# Legacy bench scripts (2026-09-23/24 flight campaign)

These are the scripts that actually ran the T01-T42 campaign from the remote Linux bench, kept verbatim
as the reference for the `rb flight` / `rb esp` / `rb motors` subcommands that replace them. They contain
hardcoded paths (a session scratchpad, USB locations 7-1.1.4.x) and are **not** meant to be run as-is.

| file | what it did | replaced by (planned) |
|---|---|---|
| flight.sh | sram / go / load / dryrun / imu / commit sequencing, banner + link checks | `rb flight ...` |
| fpga-console.py, esp-console.py | background console loggers; the ESP one reconnects across re-enumeration | `rb bench log` |
| keys.py | sends keystrokes to the motor_duty console (motor ID tests M/M2) | `rb motors test` |
| freerun.gdb, where.gdb | resume after load (rb's --gdb-script suppresses continue); halt + backtrace + resume | built into `rb flight` |
| sign_test.py | does the vehicle move the way the controller commands (per-axis) | ground-station analysis tools |

Flight presets that flew are in `../presets/`.
