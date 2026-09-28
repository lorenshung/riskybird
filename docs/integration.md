# Setup-derived integration

This branch starts at `setup` commit
`847850247e22741a85eddcdb094759a80e12b9d6`. It carries setup's curated tooling,
workloads, flight presets, overlays, model artifacts and documentation. No main
merge or main-only workload, research backend, experiment or dataset is included.

The previous, incorrectly main-based integration tip `ca253e3` is preserved as
`archive/integration-main-base`. The original `setup` and `main` branches and
working checkouts are unchanged.

## Deliberate changes from setup

| Submodule | Setup pin | Integration pin |
| --- | --- | --- |
| backends/chipyard | `bb33c836` | `72ce1b97e16a867da98f79c814d72ec6fb1344cf` |
| backends/zephyr-chipyard-sw | `2cce7ea8` | `35bf7c9becb88171304ced50ee487ac5adedfc36` |
| backends/modelblaster | `12933347` | `80cec139799e6ecf3d8625b14cdaa78ba3bf596a` |
| hardware/pcb | `a8c6393c` | `79564aa2cae9eaea964db871bacde44531f259ba` |

Chipyard uses `lorenshung/chipyard:integration`. The two software backends use
their `integration/riskybird-20260928` branches. PCB uses the lab repository
`ucb-bar/riskybird:riskybirdv3`; its files exactly match the PCB-only `612ee0d`
boundary. The later FPGA/software commits from the layout branch are excluded.

Zephyr's nested ModelBlaster selects the same `80cec139` as the root models
tier. Its RTOS is `ucb-bar/zephyr` commit
`5241922dd100c98dd953f6a6478f1e2d408de8aa`. Setup's flight-firmware commit
`2cce7ea8` is an ancestor of the merged Zephyr backend. The backend histories
retain the original merges and authorship.

The documentation cleanup corrects branch/PCB/ModelBlaster descriptions,
replaces obsolete port-to-main instructions with the integration policy, marks
the old upstream patch proposal as historical and fixes a link to omitted
research documentation. The seven patch files remain intact, and their source
commits are already ancestors of the selected merged backend. Tooling code,
workload source, flight presets, overlays, tests and committed model artifacts
remain identical to setup.

## Validation boundary

The historical bench/flight results in the setup guide describe the original
setup combination. The merged pins above are integration candidates; those
results do not automatically qualify them for flight or hardware deployment.

The same provisioned host-test environment was used before and after changing
the pins: `python3 -m unittest discover -s tests` runs 330 tests, with 267 passes
and 63 skips, in both cases. Skips retain setup's omitted research dependencies
and optional backends. An initial unprovisioned baseline failed two checks
because its Zephyr environment and RTOS driver headers were absent; installing
those existing dependencies resolved both without source changes.

Firmware checks use setup's unchanged `rb` entry point, the selected merged
software, locally installed SDK/west dependencies and fresh DTS files from the
selected Chipyard RTL. They do not program hardware or test flight behavior.

All five firmware builds passed with these pins:

| Workload | Arty200T configuration | Mode/input |
| --- | --- | --- |
| `flight_controller` | `RocketArty200TDroneFullDDRConfig` | `t21-hover-1.0m` |
| `flight_controller` | `RocketArty200TDroneFullDDRConfig` | `t33-hover-1.5m` |
| `flight_controller` | `RocketArty200TDroneFullDDRConfig` | `photo-bench` |
| `bootup_check` | `RocketArty200TDroneFullDDRConfig` | default |
| ModelBlaster DroNet | `RocketArty200TDroneGemminiSaturnFp16At35Config` | committed `gemmini_q31_rvv_softfp-gray-nhwc`, RAM console |

The first four use `./rb build`; the last uses `./rb modelblaster build` with
`--gen-dir artifacts/modelblaster/dronet/int8/gemmini_q31_rvv_softfp-gray-nhwc`.
Local logs are under `/tmp/rb-setup-validation/` on the integration host
(`setup-tests-provisioned.log`, `integration-tests.log`, `build-results.json`
and `build-*.log`). These temporary logs are not clone dependencies.

Earlier backend-specific checks remain scoped to their recorded inputs:

- Chipyard: Scala compilation and ten complete Verilog builds/collateral checks;
  no Vivado synthesis, timing or board validation. See its
  [integration record](https://github.com/lorenshung/chipyard/blob/72ce1b97e16a867da98f79c814d72ec6fb1344cf/riskybird/INTEGRATION.md).
- ModelBlaster: 218 passes, four unchanged pre-existing failures and 11 skips;
  six RISC-V compiler tests passed. The inherited per-config header selection
  issue remains open.
- Merged flight firmware: source/build checks and targeted host protocol tests,
  documented in its
  [merge record](https://github.com/lorenshung/zephyr-chipyard-sw/blob/35bf7c9becb88171304ced50ee487ac5adedfc36/docs/INTEGRATION_20260928.md).

## Next qualified changes

- Repeat setup's relevant end-to-end board/flight checks with exact selected
  firmware and matching bitstream/model pins before promoting their claims to
  verified on this branch.
- Consolidate ModelBlaster caller paths and remove the duplicate root checkout
  as a separate tested change. Current callers retain their existing path.
- Keep new KU040 FP16/FcRoCC configurations outside the supported flight flow
  until their firmware/ISA handling and hardware behavior are validated. The
  inherited KU040 dual FP16 hart 0 is scalar FP16-only; setup's curated FP16
  registry intentionally covers Arty configurations. No main-only KU040
  overlays were added to imply otherwise.
- Review floating west dependencies, the raw-UART clock mismatch and sustained
  trace-DMA limitations independently. Compile success does not resolve them.

Use [the integration policy](merging-into-main.md) for subsequent changes.
