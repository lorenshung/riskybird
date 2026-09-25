# How `setup` relates to `main`

`setup` is a **separate, curated history**, not a branch of `main`: it was
rebuilt from scratch, one verified piece at a time, starting at `6c6887f`
("setup: rb, the entry point"). The two share **no common ancestor**
(`git merge-base main setup` finds nothing), so `git log main..setup` and a
plain `git merge` do not mean anything here. Compare them path by path:

```console
git diff main setup -- <path>
git log --oneline setup -- <path>        # and the same on main
```

## What is where, by directory (2026-09-25)

| Only on `setup` | |
| --- | --- |
| `rb-env.sh` | the shell setup `rb` scripts use |
| `tools/flight/`, `tools/modelblaster/`, `tools/flow-cal.py` | flight presets and `rb fly`'s predecessors; DroNet prep; flow calibration |
| `workloads/esp_bridge`, `esp_motors`, `motor_safe`, `motor_duty` | the ESP images (Wi-Fi bridge and motor gates) and the motor commander |
| `hardware/padtest/`, `hardware/modelblaster/` | the bare-metal motor-pad test; DroNet GDB scripts |
| `backends/modelblaster`, `artifacts/modelblaster/` | the models tier and its committed kernels |
| most of `docs/` | the flight tutorial, ESP offload, telemetry, remote bench, camera colour, accelerator shells, `docs/history/` |

| Only on `main` | |
| --- | --- |
| `backends/RoSE`, `backends/XPU-RT` | the co-simulation and scheduling backends behind `rb rose`, `hil`, `xpurt`, `schedule` |
| `dse/`, `experiments/`, `benchmarks/`, `demos.md` | design-space exploration, area studies and the research record behind `rb dse` and most of `rb area` |
| `hardware/diag/`, `tools/collect-elfs.py` | older diagnostics and ELF publishing |
| docs on RoSE, XPU-RT, FireSim, the KU040 and FPGA optimisation | |

Both carry `rb`, `tools/rb/`, `tools/bench/`, `workloads/`, `integration/`,
`hardware/{zephyr,openocd,ospi,pwm,flight,bench,spioverjtag,pcb}`, `tests/` and
`backends/{chipyard,zephyr-chipyard-sw}`, but with different contents in about
fifty shared files -- most of `tools/rb/`, several overlays, the shared docs --
and different submodule pins.

## Folding it back

Not a branch swap: replacing `main` with `setup` would drop the research
surface above. The workable route is a port onto `main`:

1. Add the setup-only directories.
2. For each shared path, read `git diff main setup -- <path>` and take
   `setup`'s version where `main` has not moved, or merge by hand where it has.
   `tools/rb/` is the part that needs care: `setup`'s `rb` has flows `main`
   lacks (`fly`, `ground`, `flightlog`, ESP modes, presets) and `main`'s has
   research flows `setup` does not carry.
3. Reconcile the submodule pins. `setup`'s Chipyard pin is on the
   `arty200t-dual-uart` branch of the fork, and its zephyr-chipyard-sw pin
   carries the flight firmware that flew.
4. Run the suite on the result (`python3 -m unittest discover -s tests`); on
   `main` the RoSE, XPU-RT and area tests have their data and should run rather
   than skip.
