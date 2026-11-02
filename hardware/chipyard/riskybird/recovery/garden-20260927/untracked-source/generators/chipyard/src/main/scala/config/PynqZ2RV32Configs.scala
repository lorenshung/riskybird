package chipyard

import org.chipsalliance.cde.config.Config

// RV32 Rocket area study for PYNQ-Z2 (xc7z020clg400-1).
//
// Same shape as PynqZ2RocketTacitConfig in PynqZ2Configs.scala -- WithoutFPU, 64 KB/4-way
// L2, 256 MB ExtMem, 4-bit ExtMem IDs, TACIT encoder + DMA sink -- with WithRV32 added.
//
// WithRV32 is a RocketCoreConfig (xLen=32, sv32 pgLevels=2, fLen=32, mulUnroll=8). It is a
// RocketTileConfig underneath, so it maps over EVERY Rocket tile -- which is what makes the
// mixed big/little configs below same-ISA rather than accidentally heterogeneous.
//
// It touches core params only; it does NOT touch SystemBusKey/MemoryBusKey, so the AXI4
// memory port should stay 64-bit. That is asserted against the generated ChipTop, not
// assumed -- the FPGA top and axi4_to_axi3.v hardcode a 64-bit data path.

// 1. Headline: single RV32 big core + TACIT.
class PynqZ2RV32RocketTacitConfig extends Config(
  new tacit.WithTraceSinkDMA(1) ++
  new freechips.rocketchip.subsystem.WithInclusiveCache(nWays = 4, capacityKB = 64) ++
  new chipyard.WithTacitEncoder ++
  new chipyard.config.WithExtMemIdBits(4) ++
  new freechips.rocketchip.subsystem.WithExtMemSize(BigInt(0x10000000L)) ++
  new freechips.rocketchip.rocket.WithRV32 ++
  new freechips.rocketchip.rocket.WithoutFPU ++
  new freechips.rocketchip.rocket.WithNBigCores(1) ++
  new chipyard.config.AbstractConfig)

// 2. Same without TACIT, to price the encoder on RV32 rather than carrying the RV64 number.
class PynqZ2RV32RocketConfig extends Config(
  new freechips.rocketchip.subsystem.WithInclusiveCache(nWays = 4, capacityKB = 64) ++
  new chipyard.config.WithExtMemIdBits(4) ++
  new freechips.rocketchip.subsystem.WithExtMemSize(BigInt(0x10000000L)) ++
  new freechips.rocketchip.rocket.WithRV32 ++
  new freechips.rocketchip.rocket.WithoutFPU ++
  new freechips.rocketchip.rocket.WithNBigCores(1) ++
  new chipyard.config.AbstractConfig)

// 3a. Two identical RV32 big cores -- symmetric SMP, the simple dual-core case.
class PynqZ2RV32Dual2BigConfig extends Config(
  new tacit.WithTraceSinkDMA(1) ++
  new freechips.rocketchip.subsystem.WithInclusiveCache(nWays = 4, capacityKB = 64) ++
  new chipyard.WithTacitEncoder ++
  new chipyard.config.WithExtMemIdBits(4) ++
  new freechips.rocketchip.subsystem.WithExtMemSize(BigInt(0x10000000L)) ++
  new freechips.rocketchip.rocket.WithRV32 ++
  new freechips.rocketchip.rocket.WithoutFPU ++
  new freechips.rocketchip.rocket.WithNBigCores(2) ++
  new chipyard.config.AbstractConfig)

// 3b. THE INTERESTING ONE: big + small, same ISA, different microarchitecture.
// WithNSmallCores brings useVM=false, no FPU, no BTB, 64-set caches. Because WithRV32 maps
// over all tiles, both are RV32 -- so one Zephyr SMP image covers both and harness_xpurt can
// pin pthreads to either, which Rocket+Ibex (different ISA) cannot do.
class PynqZ2RV32BigLittleConfig extends Config(
  new tacit.WithTraceSinkDMA(1) ++
  new freechips.rocketchip.subsystem.WithInclusiveCache(nWays = 4, capacityKB = 64) ++
  new chipyard.WithTacitEncoder ++
  new chipyard.config.WithExtMemIdBits(4) ++
  new freechips.rocketchip.subsystem.WithExtMemSize(BigInt(0x10000000L)) ++
  new freechips.rocketchip.rocket.WithRV32 ++
  new freechips.rocketchip.rocket.WithoutFPU ++
  new freechips.rocketchip.rocket.WithNBigCores(1) ++
  new freechips.rocketchip.rocket.WithNSmallCores(1) ++
  new chipyard.config.AbstractConfig)
