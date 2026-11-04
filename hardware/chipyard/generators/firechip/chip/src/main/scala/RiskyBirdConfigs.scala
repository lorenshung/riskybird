//See LICENSE for license details.

package firechip.chip

import org.chipsalliance.cde.config.Config

/** FireSim targets for the SoCs RiskyBird measures on KU040.
 *
 *  A design point is a `chipyard.<X>Config`. It gets two wrappers that differ
 *  only in what surrounds it:
 *
 *  {{{
 *  // area:        WithKU040Tweaks           ++ WithBroadcastManager   ++ chipyard.<X>Config
 *  // performance: WithDefaultFireSimBridges ++ WithFireSimConfigTweaks ++ chipyard.<X>Config
 *  }}}
 *
 *  Because both wrap the same base, the elaborated SoC is the same machine and
 *  a resource measurement can be joined to a latency measurement exactly. That
 *  is the whole point: without a FireSim wrapper, no configuration measured for
 *  area has ever actually been run, and its benchmark numbers have to be
 *  borrowed from a different machine and labelled extrapolated.
 *
 *  These are keyed on the *base* config, not on the KU040 wrapper, so the
 *  several board variants that differ only in periphery -- plain, `Ospi`, and
 *  `DroneLogic` -- share one FireSim target. Periphery changes area; it does
 *  not change how many cycles a network takes.
 *
 *  Two divergences from the KU040 shells remain and are deliberate, since
 *  FireSim is a performance vehicle rather than a deployment target:
 *  `WithFireSimConfigTweaks` runs every bus at 1000 MHz where the KU040 shells
 *  declare 50, and FireSim models an L2 and DDR where the KU040 shell wires a
 *  32 KiB scratchpad. The first is harmless -- profiles are recorded in cycles
 *  and converted to wall time once, at whatever frequency is being reported --
 *  while the second changes cycle counts and is why the KU040 DDR4 config
 *  still needs writing.
 *
 *  This file is the RiskyBird-owned corner of firechip. Keeping the wrappers
 *  here rather than in `TargetConfigs.scala` means upstream edits to that file
 *  never conflict with ours.
 */

/** Rocket alone. The scalar baseline, and the only KU040 point that leaves
 *  meaningful room on the device (16.2% of LUTs).
 */
class FireSimRiskyBirdRocketConfig extends Config(
  new WithDefaultFireSimBridges ++
  new WithFireSimConfigTweaks ++
  new chipyard.RocketConfig)

/** Rocket plus the V256D128 Saturn vector unit, no matrix engine.
 *  Elaborates with `zvl256b`; the backing profile shape is `V256D128_rvv`.
 */
class FireSimRiskyBirdSaturnV256D128Config extends Config(
  new WithDefaultFireSimBridges ++
  new WithFireSimConfigTweaks ++
  new chipyard.REFV256D128RocketConfig)

/** Rocket plus the default 16x16 int8 Gemmini: 256 mesh cells.
 *  No vector unit -- this hart's ISA carries no `zvl*`.
 */
class FireSimRiskyBirdGemmini16x16Config extends Config(
  new WithDefaultFireSimBridges ++
  new WithFireSimConfigTweaks ++
  new chipyard.GemminiRocketConfig)

/** Rocket plus the Q0.31 weight-stationary 32x32 Gemmini: 1024 mesh cells,
 *  128 KB accumulator. Also has no vector unit, which is why pairing it with
 *  an RVV profile describes hardware it does not have.
 */
class FireSimRiskyBirdGemminiQ31Ws32x32Config extends Config(
  new WithDefaultFireSimBridges ++
  new WithFireSimConfigTweaks ++
  new chipyard.Q31Ws32x32AccGemminiRocketConfig)

/** Rocket plus the V128D128 Saturn vector unit carrying the int8 Outer Product
 *  Unit: 256 OuterProductCell instances, two DSP48E2 each.
 */
class FireSimRiskyBirdSaturnOPUV128D128Config extends Config(
  new WithDefaultFireSimBridges ++
  new WithFireSimConfigTweaks ++
  new chipyard.REFV128D128RocketOPUConfig)

/** V128D128 Saturn vector unit and the Q0.31 32x32 Gemmini on one Rocket.
 *
 *  The same base chain as `Q31Ws32x32AccGemminiSaturnV128D128KU040Config` in
 *  `fpga/src/main/scala/ku040/Configs.scala`, differing only in the two wrapper
 *  tweaks -- which is what makes the area measurement and the latency
 *  measurement describe one machine.
 *
 *  Closest of these to the machine the shipped gemmini_q31 and V256D128_rvv
 *  profiles were captured on: same accelerator pair, one hart instead of two,
 *  and VLEN 128 instead of 256.
 */
class FireSimRiskyBirdGemminiQ31SaturnV128D128Config extends Config(
  new WithDefaultFireSimBridges ++
  new WithFireSimConfigTweaks ++
  new saturn.rocket.WithRocketVectorUnit(128, 128, saturn.common.VectorParams.refParams) ++
  new gemmini.Q31Ws32x32AccGemminiConfig ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/** V128D128 Saturn and the Q0.31 32x32 Gemmini with FP16 vector and scalar.
 *
 *  Base chain copied verbatim from `Q31Ws32x32AccGemminiSaturnV128D128Fp16FullKU040Config`
 *  in `fpga/src/main/scala/ku040/Configs.scala`, with that config's two KU040
 *  wrapper layers -- `WithKU040Tweaks(freqMHz = 100)` and `WithBroadcastManager`
 *  -- replaced by the two FireSim ones. Everything below those layers is
 *  identical, including `robotMpcParams` with `useElementwiseFP64` and
 *  `noPermute`, and `WithRocketFPU16` for the FP16 scalar FPU. That identity is
 *  the whole point: it is what lets the KU040 area record and a FireSim latency
 *  measurement be joined as one machine rather than labelled `extrapolated`.
 *
 *  Note the Saturn params here are `robotMpcParams`, NOT the `refParams` used by
 *  `FireSimRiskyBirdGemminiQ31SaturnV128D128Config` above. Each wrapper tracks
 *  its own KU040 counterpart: that one pairs with
 *  `Q31Ws32x32AccGemminiSaturnV128D128KU040Config` (`refParams`), this one with
 *  the Fp16Full config (`robotMpcParams`). The params are part of the base chain
 *  being matched, so they must not be shared between wrappers.
 *
 *  Two divergences from the KU040 config remain and are inherent to the FireSim
 *  wrapper, not oversights:
 *
 *   - `WithFireSimConfigTweaks` runs the buses at 1000 MHz where the KU040 shell
 *     declares 100. Clock affects wall time only, never cycle counts, and
 *     `tools/rb/fingerprint.py` deliberately excludes it from `identity_id`, so
 *     `rb xpurt` reports the same cycles at whichever frequency is asked for.
 *   - `WithFireSimConfigTweaks` models an L2 and DDR where the KU040 config uses
 *     `WithBroadcastManager` and a 32 KiB scratchpad. This one DOES change cycle
 *     counts and will still show up as a memory divergence until the KU040 DDR4
 *     config is written. It is the last cycle-affecting gap between the two.
 */
class FireSimRiskyBirdGemminiQ31SaturnV128D128Fp16FullConfig extends Config(
  new WithDefaultFireSimBridges ++
  new WithFireSimConfigTweaks ++
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.robotMpcParams.copy(
      useElementwiseFP64 = true,
      noPermute = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new gemmini.Q31Ws32x32AccGemminiConfig ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/** The Fp16 SoC with `mvin_scale_args` and LoopConv both removed -- config A of
 *  the co-design pair in `docs/ku040-codesign-cnn.md`.
 *
 *  Base chain copied from
 *  `Q31Ws32x32AccGemminiSaturnV128D128Fp16NoMvinScaleNoLoopConvOspiSingleDDRKU040Config`
 *  in `fpga/src/main/scala/ku040/Configs.scala`. It differs from the Fp16Full
 *  wrapper above in the Gemmini layer alone: `Q31GemminiConfig` with
 *  `mvin_scale_args = None` and `has_loop_conv = false` instead of the stock
 *  `Q31Ws32x32AccGemminiConfig`. Everything else -- Saturn `robotMpcParams`,
 *  `WithRocketFPU16`, bus width, hart count -- is identical.
 *
 *  Removing LoopConv removes hardware, so the software must take the scalar
 *  fallback path (`im2col_full_C` + scalar maxpool) rather than
 *  `tiled_conv_auto`. That is the point of the pair: this target measures what
 *  the LoopConv-free machine actually costs in cycles, against config B's
 *  `CnnNavKU040Config`, instead of predicting it.
 *
 *  Memory note: its KU040 counterpart is the `OspiSingleDDR` variant, so the
 *  KU040 side is DDR4-backed (`ddr = true, ddrControllers = 1`) rather than the
 *  32 KiB scratchpad the earlier shells wired. That closes most of the memory
 *  divergence called out above -- both sides now have real DRAM behind the
 *  accelerator. What remains is the cache hierarchy: `WithFireSimConfigTweaks`
 *  models an L2 where the KU040 config keeps `WithBroadcastManager`.
 */
class FireSimRiskyBirdGemminiQ31SaturnV128D128Fp16NoMvinScaleNoLoopConvConfig extends Config(
  new WithDefaultFireSimBridges ++
  new WithFireSimConfigTweaks ++
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.robotMpcParams.copy(
      useElementwiseFP64 = true,
      noPermute = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new gemmini.Q31GemminiConfig(
    gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(
      mvin_scale_args = None,
      has_loop_conv = false)) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/** The flight3 SoC as built: LoopConv kept, mvin-scale dropped.
 *
 *  FireSim counterpart of
 *  `Q31Ws32x32AccGemminiSaturnV128D128Fp16NoMvinScaleOspiSingleDDRKU040Config`,
 *  which is the deployable point -- the NoLoopConv wrapper above is its timing
 *  experiment and is kept only so the pair can be compared. Differs from that
 *  wrapper in one field, `has_loop_conv` back at its default, and from
 *  `…Fp16FullConfig` in one other, `mvin_scale_args = None`.
 *
 *  Memory: the KU040 counterpart is the single-controller DDR4 variant
 *  (`ddr = true, ddrControllers = 1`), so both sides of this pair have real DRAM
 *  behind the accelerator. The remaining divergence is the cache hierarchy --
 *  `WithFireSimConfigTweaks` models an L2 where the KU040 config keeps
 *  `WithBroadcastManager` -- plus the usual 1000 MHz bus frequency, which
 *  changes wall time and not cycles.
 */
class FireSimRiskyBirdGemminiQ31SaturnV128D128Fp16NoMvinScaleConfig extends Config(
  new WithDefaultFireSimBridges ++
  new WithFireSimConfigTweaks ++
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.robotMpcParams.copy(
      useElementwiseFP64 = true,
      noPermute = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new gemmini.Q31GemminiConfig(
    gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(
      mvin_scale_args = None)) ++   // has_loop_conv defaults to true
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/* ---------------------------------------------------------------------------
 * Fingerprint probes
 *
 * `tools/rb/fingerprint.py` extracts a machine's identity from an elaborated
 * DTS plus its `gen-collateral` dir. The KU040 flow emits both. FireSim's flow
 * emits NEITHER -- its generated-src holds only FIRRTL, FireSim-generated.sv
 * and XDC -- so a FireSim profile target could previously only be *declared*
 * (see dse/profile-targets/firesim_rocket_saturn.json), which leaves fields
 * null and forces every join to report `extrapolated`.
 *
 * These probes close that gap. Each is its `FireSim*Config` counterpart with
 * `WithDefaultFireSimBridges` removed and nothing else changed. That layer is
 * the only harness-side part of the pair: `WithFireSimConfigTweaks` is entirely
 * target-side (bus frequencies, WithNoClockTap, SerialTL width, UART FIFOs,
 * WithTraceIO, WithExtMemSize(16 GiB), WithBlockDevice), and the bridges
 * contribute nothing to target identity -- no harts, no ISA, no accelerator, no
 * memory region. So elaborating a probe under Chipyard's own TestHarness yields
 * a DTS describing exactly the SoC the FireSim bitstream simulates.
 *
 * Probes are never built to a bitstream. They exist only to be elaborated, so
 * that a descriptor under `dse/profile-targets` can carry
 * `evidence: "extracted"` and the DSE join can compare extracted against
 * extracted.
 *
 * Each probe also adds `chipyard.harness.WithBlockDeviceModel`, which the
 * FireSim wrapper does not need. `WithFireSimConfigTweaks` pulls in
 * `testchipip.iceblk.WithBlockDevice`, and on FireSim the resulting IO is
 * consumed by a bridge; under Chipyard's TestHarness nothing drives it, so
 * firtool aborts with `sink "chiptop0.blockdev_bits_*" not fully initialized`
 * and -- because the firtool rule does `rm -rf` on gen-collateral first --
 * leaves an EMPTY collateral dir. A DTS is still written, so the failure is
 * quiet: the fingerprint extracts with correct harts, ISA and memory but
 * `accelerators: {}`, silently understating any SoC that has one. The binder is
 * a pure model, harness-side only, and cannot affect target identity (which is
 * harts, accelerators and memory alone).
 * ------------------------------------------------------------------------- */

/** Fingerprint probe for `FireSimRiskyBirdRocketConfig` (bitstream
 *  `rb_u250_rocket`, profile target `firesim_rocket_scalar`).
 */
class FireSimRiskyBirdRocketTargetConfig extends Config(
  new chipyard.harness.WithBlockDeviceModel ++
  new WithFireSimConfigTweaks ++
  new chipyard.RocketConfig)

/** Fingerprint probe for `FireSimRiskyBirdGemminiQ31SaturnV128D128Fp16FullConfig`,
 *  whose KU040 counterpart is
 *  `Q31Ws32x32AccGemminiSaturnV128D128Fp16FullKU040Config`.
 */
class FireSimRiskyBirdGemminiQ31SaturnV128D128Fp16FullTargetConfig extends Config(
  new chipyard.harness.WithBlockDeviceModel ++
  new WithFireSimConfigTweaks ++
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.robotMpcParams.copy(
      useElementwiseFP64 = true,
      noPermute = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new gemmini.Q31Ws32x32AccGemminiConfig ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/** Fingerprint probe for
 *  `FireSimRiskyBirdGemminiQ31SaturnV128D128Fp16NoMvinScaleNoLoopConvConfig`,
 *  whose KU040 counterpart is
 *  `Q31Ws32x32AccGemminiSaturnV128D128Fp16NoMvinScaleNoLoopConvOspiSingleDDRKU040Config`.
 *
 *  `has_loop_conv` and `mvin_scale_args` both change the Gemmini module
 *  inventory, so this probe has to carry them: extracting against the Fp16Full
 *  probe instead would report accelerator collateral this machine does not have.
 */
class FireSimRiskyBirdGemminiQ31SaturnV128D128Fp16NoMvinScaleNoLoopConvTargetConfig extends Config(
  new chipyard.harness.WithBlockDeviceModel ++
  new WithFireSimConfigTweaks ++
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.robotMpcParams.copy(
      useElementwiseFP64 = true,
      noPermute = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new gemmini.Q31GemminiConfig(
    gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(
      mvin_scale_args = None,
      has_loop_conv = false)) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/** Fingerprint probe for
 *  `FireSimRiskyBirdGemminiQ31SaturnV128D128Fp16NoMvinScaleConfig`, whose KU040
 *  counterpart is
 *  `Q31Ws32x32AccGemminiSaturnV128D128Fp16NoMvinScaleOspiSingleDDRKU040Config`.
 *
 *  Distinct from the NoLoopConv probe above by `has_loop_conv` alone, which is
 *  exactly the difference `fingerprint.py` learned to see -- the two land on
 *  different `identity_id`s only because the LoopConv modules are now recorded.
 */
class FireSimRiskyBirdGemminiQ31SaturnV128D128Fp16NoMvinScaleTargetConfig extends Config(
  new chipyard.harness.WithBlockDeviceModel ++
  new WithFireSimConfigTweaks ++
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.robotMpcParams.copy(
      useElementwiseFP64 = true,
      noPermute = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new gemmini.Q31GemminiConfig(
    gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(
      mvin_scale_args = None)) ++   // has_loop_conv defaults to true
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)
