# Integration policy and branch histories

`integration` starts from the curated `setup` commit `8478502`. The earlier
integration branch incorrectly started from `main`; that version is retained
as `archive/integration-main-base` at `ca253e3` for reference. Neither `main`
nor `setup` is changed by this correction.

The parent repository's setup and main histories are unrelated. Do not merge
main wholesale to obtain a feature: it brings research backends, experiments,
workloads and documentation outside setup's supported scope. Compare a proposed
change against setup, retain its provenance and validate the affected workflow.
The original backend merge histories remain intact in their own repositories.

## Admission of changes

- Start from the existing supported workflow and compare its checks before and
  after a change. Add only the pieces needed for that change.
- Record exactly what ran. Unit tests, firmware builds and RTL generation are
  distinct from an end-to-end board test or flight.
- A new backend pin does not inherit an older pin's flight qualification. Record
  the firmware, model, hardware configuration and bitstream used when repeating
  that workflow before describing the new combination as verified.
- Keep research workflows and unverified extensions outside the supported
  surface until their dependencies and end-to-end evidence are available.

The current approved pin changes and remaining validation are listed in
[integration.md](integration.md). Removing the duplicate top-level ModelBlaster
checkout will be a separate caller-path change with its own checks.

## Preserved scope

This branch retains setup's entry point, flight presets, workloads, overlays,
committed model artifacts, bench tools and historical evidence. Experimental
CLI entries already present in setup remain labeled experimental; their omitted
RoSE/XPU-RT backends and research datasets are not added here.

The old directory-by-directory porting proposal in this file is superseded.
Its original contents remain available in the `setup` history.
