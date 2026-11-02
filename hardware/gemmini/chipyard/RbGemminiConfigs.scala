package chipyard

// Extracted from gemmini 7d91e9e:
//   chipyard/GemminiConfigs.scala; only the configs RiskyBird appended
// The original still lives in the backend until its pin moves to the plain
// tip; keep the two in step until then.

import org.chipsalliance.cde.config.Config

// Same shape as GemminiRocketConfig but with bit-exact Q0.31 mvout requantize
// (acc_scale_t is SInt(32.W) instead of f32). Used to validate that HW im2col
// + GEMM + Q0.31 requantize matches the agents pipeline's scalar Q0.31 golden.
class Q31GemminiRocketConfig extends Config(
  new gemmini.Q31GemminiConfig ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

// Q0.31 with weight-stationary-only dataflow (drops OS/WS dataflow mux,
// collapses PE control to a single path). Area win at fixed PE count.
class Q31WsGemminiRocketConfig extends Config(
  new gemmini.Q31WsGemminiConfig ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

// Q0.31 WS-only with 32x32 mesh (1024 INT8 PEs == 1024 DSP48E2).  At fixed
// acc_capacity=64 KB the accumulator demotes to LUTRAM at this mesh width;
// see Q31Ws32x32AccGemminiRocketConfig for the BRAM-mapping fix.
class Q31Ws32x32GemminiRocketConfig extends Config(
  new gemmini.Q31Ws32x32GemminiConfig ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

// 32x32 mesh + acc_capacity bumped to 128 KB so the accumulator banks stay
// BRAM-mappable.  Burns ~128 extra RAMB18 tiles to recover ~10K LUTs.
class Q31Ws32x32AccGemminiRocketConfig extends Config(
  new gemmini.Q31Ws32x32AccGemminiConfig ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

// ============================================================================
// IntOnly Rocket variants of the Q31Ws Gemmini optimized configs
// ============================================================================
// Gemmini is integer-only on the int8 path -- the host Rocket FPU is dead
// weight for these builds.  WithoutFPU drops fpu = None on the tile core,
// recovering ~14K LUTs and the FP register file's flops + read/write ports.
// Used in the area-minimized KU040 target sweep alongside the Saturn IntOnly
// configs.
class Q31WsGemminiRocketNoFPUConfig extends Config(
  new gemmini.Q31WsGemminiConfig ++
  new freechips.rocketchip.rocket.WithoutFPU ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class Q31Ws32x32AccGemminiRocketNoFPUConfig extends Config(
  new gemmini.Q31Ws32x32AccGemminiConfig ++
  new freechips.rocketchip.rocket.WithoutFPU ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

// Default Gemmini (float acc_scale, BOTH dataflow) scaled to 32x32 mesh.
// Same mesh+dma_buswidth+acc_capacity overrides as Q31Ws32x32Acc but keeps
// the f32 acc_scale (which goes to LUTs, not DSPs) and BOTH dataflow MUX.
// Baseline against Q31_32x32 to isolate the f32 acc_scale cost at 32x32.
class Default32x32GemminiRocketConfig extends Config(
  new gemmini.DefaultGemminiConfig(gemmini.GemminiConfigs.defaultConfig.copy(
    meshRows     = 32,
    meshColumns  = 32,
    dma_buswidth = 256,
    acc_capacity = gemmini.CapacityInKilobytes(128))) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

// Q31 Gemmini (Q0.31 acc_scale, BOTH dataflow) scaled to 32x32 mesh.
// Pairs with Q31Ws32x32Acc to isolate the BOTH-vs-WS-only delta at 32x32.
class Q3132x32GemminiRocketConfig extends Config(
  new gemmini.Q3132x32GemminiConfig ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

// 8x8 small Gemmini (32KB sp / 16KB acc). Replaces the old
// `gemmini.SmallGemminiConfig` mixin that was dropped in the modular-chipyard
// gemmini refactor — inlined via defaultConfig.copy().
class SmallGemminiRocketConfig extends Config(
  new gemmini.DefaultGemminiConfig(gemmini.GemminiConfigs.defaultConfig.copy(
    sp_capacity  = gemmini.CapacityInKilobytes(32),
    acc_capacity = gemmini.CapacityInKilobytes(16),
    meshRows     = 8,
    meshColumns  = 8)) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class DualSmallGemminiRocketConfig extends Config(
  new gemmini.DefaultGemminiConfig(gemmini.GemminiConfigs.defaultConfig.copy(
    sp_capacity  = gemmini.CapacityInKilobytes(32),
    acc_capacity = gemmini.CapacityInKilobytes(16),
    meshRows     = 8,
    meshColumns  = 8)) ++
  new freechips.rocketchip.rocket.WithNHugeCores(2) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class FP16GemminiRocketConfig extends Config(
  new gemmini.GemminiFP16DefaultConfig ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class FP16ReRoCCGemminiRocketConfig extends Config(
  new rerocc.WithReRoCC ++
  new gemmini.GemminiFP16DefaultConfig ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

