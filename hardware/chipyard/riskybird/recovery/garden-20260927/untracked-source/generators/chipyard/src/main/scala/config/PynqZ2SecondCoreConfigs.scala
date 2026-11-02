package chipyard

import chisel3._
import org.chipsalliance.cde.config.Config

// Second-core fit study for PYNQ-Z2 (xc7z020clg400-1).
//
// Premise: minimal Saturn (87.6% LUT OOC) and a fully-stripped 4x4 Gemmini (96.7%) both
// blow the device, so a second CPU is the remaining route to heterogeneity. The question
// is whether any non-Rocket core fits in the ~25k LUT of headroom AND is usable by the
// software stack.
//
// The second-core mixin must be OUTER (leftmost) so its `up(TilesLocated)` sees Rocket's
// tile in `prev` and appends to it.

// RV32IMC, no atomics, no VM. NOTE: ibex.WithNIbexCores unconditionally forces
// `SystemBusKey beatBytes = 4`, narrowing the whole system bus to 32-bit.
class PynqZ2RocketTacitIbexConfig extends Config(
  new ibex.WithNIbexCores(1) ++
  new PynqZ2RocketTacitConfig)

// RV64 with atomics and VM -- an SMP candidate on paper.
class PynqZ2RocketTacitVexiiConfig extends Config(
  new vexiiriscv.WithNVexiiRiscvCores(1) ++
  new PynqZ2RocketTacitConfig)

// RV64 application-class. Expected far too big; measured to bound the upper end.
class PynqZ2RocketTacitCVA6Config extends Config(
  new cva6.WithNCVA6Cores(1) ++
  new PynqZ2RocketTacitConfig)

// Standalone references, to price each core without Rocket in the way.
class PynqZ2IbexOnlyConfig extends Config(
  new ibex.WithNIbexCores(1) ++
  new chipyard.config.WithInclusiveCacheWriteBytes(4) ++
  new chipyard.config.WithExtMemIdBits(4) ++
  new freechips.rocketchip.subsystem.WithExtMemSize(BigInt(0x10000000L)) ++
  new chipyard.config.AbstractConfig)

class PynqZ2VexiiOnlyConfig extends Config(
  new vexiiriscv.WithNVexiiRiscvCores(1) ++
  new chipyard.config.WithExtMemIdBits(4) ++
  new freechips.rocketchip.subsystem.WithExtMemSize(BigInt(0x10000000L)) ++
  new chipyard.config.AbstractConfig)

// Ibex forces SystemBusKey beatBytes=4, which trips
// `require(micro.writeBytes <= inner.manager.beatBytes)` in InclusiveCacheParameters.
// WithInclusiveCacheWriteBytes(4) is what the stock IbexConfig pairs it with.
// NOTE: this narrows the ENTIRE system bus to 32-bit, including Rocket's path to memory.
class PynqZ2RocketTacitIbexFixConfig extends Config(
  new ibex.WithNIbexCores(1) ++
  new chipyard.config.WithInclusiveCacheWriteBytes(4) ++
  new PynqZ2RocketTacitConfig)

// Last attempt for Ibex: force the system bus back to 64-bit OUTSIDE the Ibex mixin, so
// Rocket's RV64 D-cache is legal again. Whether the Ibex tile tolerates a 64-bit sbus is
// the open question -- it forces 4 deliberately, which suggests it does not.
class PynqZ2RocketTacitIbex64Config extends Config(
  new Config((site, here, up) => {
    case freechips.rocketchip.subsystem.SystemBusKey =>
      up(freechips.rocketchip.subsystem.SystemBusKey, site).copy(beatBytes = 8)
  }) ++
  new ibex.WithNIbexCores(1) ++
  new PynqZ2RocketTacitConfig)
