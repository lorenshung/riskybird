package chipyard

import org.chipsalliance.cde.config.{Config, Parameters}
import freechips.rocketchip.diplomacy.LazyModule
import freechips.rocketchip.tile.{BuildRoCC, OpcodeSet}

/** Attach the NVFP4 matrix engine as a RoCC accelerator.
  *
  * Mirrors `generators/gemmini/src/main/scala/gemmini/Configs.scala:258`
  * (`DefaultGemminiConfig`): append to `BuildRoCC` rather than replace it, so
  * this mixin composes with Gemmini's in a heterogeneous SoC instead of
  * silently dropping it.
  *
  * custom0, not custom3: Gemmini hard-codes custom3 (`XCUSTOM_ACC 3` in
  * gemmini_params.h) and Rocket's `RoccCommandRouter` requires the opcode sets
  * attached to one tile to be disjoint, so sharing a tile with Gemmini means
  * taking a different opcode. tests/nvfp4.h encodes custom0 to match.
  */
class WithNVFP4Accel(cfg: nvfp4.NVFP4Config = nvfp4.NVFP4Config())
  extends Config((site, here, up) => {
    case BuildRoCC => up(BuildRoCC) ++ Seq(
      (q: Parameters) => {
        implicit val pp: Parameters = q
        LazyModule(new nvfp4.NVFP4Accel(OpcodeSet.custom0, cfg))
      })
  })

/** Rocket + the NVFP4 matrix engine.
  *
  * This is chipyard's stock `RocketConfig` (`WithNHugeCores(1) ++
  * AbstractConfig`) plus the accelerator, and nothing else. The earlier
  * placeholder also pulled in `WithSystemBusWidth(128)`; that is dropped
  * deliberately. Gemmini needs the wider system bus because its DMA is a
  * TileLink master that moves a 16x16 int8 tile per transaction. The NVFP4
  * shell has no TileLink master at all -- all of its DRAM traffic goes through
  * the core's 8-byte HellaCache port -- so a 128-bit system bus would buy it
  * exactly nothing while making this config differ from stock for no reason.
  * Staying close to stock keeps a bring-up failure attributable to the
  * accelerator rather than to the SoC around it.
  */
class NVFP4RocketConfig extends Config(
  new WithNVFP4Accel ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)
