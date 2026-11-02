// RiskyBird area-optimization EXPERIMENTS (not signed-off configs). Safe to delete.
package chipyard.fpga.arty200t

import org.chipsalliance.cde.config._

// Drop Gemmini LoopConv, keep LoopMatmul. Controller.scala instantiates LoopConv
// only if has_loop_conv; LoopMatmul is always present and takes raw_cmd directly
// when LoopConv is gone. DroNet convs then run via software im2col -> tiled_matmul
// (LoopMatmul), i.e. the gemmini_im2col_full_C kernel path (the tiled_conv/LoopConv
// path is no longer available). Same combined SoC (FULL periphery) as
// RocketArty200TDroneGemminiSaturnFp16Config.
//
// MEASURED (Vivado 2023.1 synth, xc7a200t) vs the with-LoopConv baseline:
//   top LUT 130,393 -> 125,552 (-4,841), FF 77,818 -> 74,227 (-3,591),
//   DSP 555 -> 425 (-130); netlist has 0 LoopConv / 7 LoopMatmul modules.
class RocketArty200TDroneGemminiSaturnFp16NoLoopConvConfig extends Config(
  new WithArty200TDroneFullDmaUartBase ++
  new saturn.rocket.WithRocketVectorUnit(128, 64,
    saturn.common.VectorParams.robotMpcParams.copy(useElementwiseFP64 = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new gemmini.Q31GemminiConfig(RbArty200TGemmini.mesh16.copy(has_loop_conv = false)) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)
