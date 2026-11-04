package chipyard

import org.chipsalliance.cde.config.Config

// -------------------------------------------------------------------------------------
// Throwaway configs for fpga/pynq-z2/rtl_study/pext (PEXT_FEASIBILITY.md).
//
// They exist to answer one question by elaboration rather than by assertion: can a
// functional unit be given to hart 0 and withheld from hart 1 on the shipped
// big.LITTLE pair, and what does the RoCC plumbing cost on top of the datapath?
//
// PynqZ2RocketBigLittleTacitConfig is NOT modified -- these extend it.
// -------------------------------------------------------------------------------------

// Baseline + a RoCC accelerator on hart 0 ONLY.
//
// chipyard.config.WithMultiRoCC rebinds BuildRoCC to a per-tileId lookup:
//   case BuildRoCC => site(MultiRoCCKey).getOrElse(site(TileKey).tileId, Nil)
// and WithMultiRoCCFromBuildRoCC(0) moves whatever the fragments to its right put in
// BuildRoCC into MultiRoCCKey under hart 0. Everything is resolved through the tile's
// own Parameters (BaseTile binds TileKey per tile), so hart 1 resolves BuildRoCC to Nil
// and gets no accelerator, no command router, no response arbiter and no extra
// D-cache port.
class PynqZ2RocketBigLittleRoCCHart0Config extends Config(
  new chipyard.config.WithMultiRoCC ++
  new chipyard.config.WithMultiRoCCFromBuildRoCC(0) ++
  new chipyard.config.WithAccumulatorRoCC ++
  new PynqZ2RocketBigLittleTacitConfig)

// The same accelerator on BOTH harts, to price the per-tile gating itself.
class PynqZ2RocketBigLittleRoCCBothConfig extends Config(
  new chipyard.config.WithAccumulatorRoCC ++
  new PynqZ2RocketBigLittleTacitConfig)
