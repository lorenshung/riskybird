// See LICENSE for license details.
//
// RiskyBird V64D64 Saturn AREA EXPERIMENT (uncommitted; do not push).
//
// Explores narrowing Saturn's architectural vector length VLEN 128 -> 64 while
// keeping the datapath length DLEN = 64, on the PRIMARY combined SoC
// (16x16 Q0.31 Gemmini + FP16-only Saturn + FP16 scalar FPU + full drone
// periphery).  Everything except the Saturn vLen argument is byte-identical to
// RocketArty200TDroneGemminiSaturnFp16Config, so a synth-only rb-area delta
// isolates exactly the VLEN 128 -> 64 change (VRF + VLEN-chunk sequencing).
//
// egsPerVReg = vLen/dLen goes 2 -> 1, so egsTotal (VRF element groups) 64 -> 32
// (VRF storage halves).  DLEN-wide execution lanes (ExecutionUnitfp/int) are
// unchanged.  Legal per saturn Backend requires: vLen>=64, vLen>=dLen,
// vLen%dLen==0 (64,64,0 all pass); hazardingMultiplier=0 so 1<=egsTotal holds.
//
// Build for area (must match the cached V128D64 baseline's RB_ATTRS=dsp):
//   make -C fpga SUB_PROJECT=arty200t \
//        CONFIG=RocketArty200TDroneGemminiSaturnFp16V64Config RB_ATTRS=dsp rb-area

package chipyard.fpga.arty200t

import org.chipsalliance.cde.config._

// V64D64 counterpart of RocketArty200TDroneGemminiSaturnFp16Config (V128D64).
class RocketArty200TDroneGemminiSaturnFp16V64Config extends Config(
  new WithArty200TDroneFullDmaUartBase ++
  new saturn.rocket.WithRocketVectorUnit(64, 64,
    saturn.common.VectorParams.robotMpcParams.copy(useElementwiseFP64 = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new gemmini.Q31GemminiConfig(RbArty200TGemmini.mesh16) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)
