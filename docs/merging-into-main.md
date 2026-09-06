# Folding this branch back into `main`

`setup` was built as an orphan branch — a from-scratch rebuild, one piece at a
time, each verified before the next went in. It has no common ancestor with
`main`, so `git merge` will not do anything sensible here. This is what the
decision actually looks like.

## It is not "replace main"

`setup` carries a **subset** of `main`'s paths. Replacing `main` wholesale would
drop the research surface:

| Only on `main` | Files |
| --- | --- |
| `dse/` | 35 |
| `experiments/` | 27 |
| `benchmarks/` | 7 |
| `demos.md` | 1 |

Those were left out deliberately — `setup` covers the supported surface,
Chipyard and Zephyr, and the experimental commands that use those directories
are marked `[experimental]` in `rb --help`.

## It is a 24-file changeset

Only **two files** exist on `setup` and not on `main`:

```
rb-env.sh
tools/bench/acceptance.sh
```

Everything else is a shared path with different content — **22 files**:

| Area | Files |
| --- | --- |
| `tools/rb/` | `cli.py`, `boards.py`, `artifacts.py`, `environments.py` |
| bench tooling | `tools/bench/usbreset.py`, `tools/bench/drone-ddr-check.sh` |
| hardware | `hardware/pwm/run.sh`, `hardware/pwm/tb_motor_pwm.cpp`, `hardware/spioverjtag/setup-openfpgaloader.sh` |
| docs | `setup.md`, `flight-on-fpga.md`, `hm01b0-camera-bringup.md`, `motor-pwm-bringup.md`, `te0712-*-bringup.md`, `verification.md`, `repository-layout.md`, `README.md` |
| tests | `tests/test_rb.py` |
| config | `.gitignore`, `.gitmodules` |

So the sane move is to **port those 24 files onto `main`**, not to swap branches.
`main` keeps its research directories; `rb` gains the fixes.

## What those changes are

Worth knowing before porting, because several are behavioural rather than
cosmetic:

- **`rb` no longer requires Chipyard to build or program.** `Board.generated_dts`
  and `Board.bitstream` fall back to collected directories (`RB_DTS_DIR`,
  `RB_BITSTREAM_DIR`). This is what makes a Mac usable at all.
- **Host portability.** The SDK sysroot and version are discovered rather than
  hardcoded; OpenOCD runs through the ELF loader only when one exists; serial
  discovery knows macOS device names; FTDI discovery falls back to libusb.
- **`rb doctor` and `rb env status` stopped reporting `ok` for empty or
  half-installed directories.** Three separate instances of the same bug.
- **Two errors that misdirected the reader** — the missing-bitstream error told a
  Mac to run Chipyard, and the RAM check suggested a mode belonging to a
  different workload.
- **`rb debug` loads and runs by default** (`--no-run` restores the old
  behaviour), and `RB_CMAKE_ARGS` / `RB_EXTRA_CPPFLAGS` exist because the flight
  controller's knobs are CMake cache variables and C++ defines respectively.
- Two scripts stopped pointing at one particular checkout by absolute path.

`git log --oneline main..setup` will not work (no common ancestor); use
`git diff main setup -- <path>` per file instead.

## The one thing to decide first

Whether `main`'s copies of those 22 files have changed since this branch was
cut. If they have not, the port is a file-by-file checkout. If they have, the
diffs need reading — `setup`'s versions were written against a specific state
and verified there.
