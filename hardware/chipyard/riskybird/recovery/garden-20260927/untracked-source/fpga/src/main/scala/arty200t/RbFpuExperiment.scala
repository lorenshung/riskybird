// See LICENSE for license details.
//
// RiskyBird MINIMAL SCALAR FP32 FPU area experiment (uncommitted; do not push).
//
// ALTERNATIVE to the FC-RoCC path: instead of offloading the FC estimator+PID
// to a fixed-function integer RoCC (and dropping the scalar FPU entirely), give
// Rocket back a MINIMAL scalar FP32 FPU so the FC runs as plain native fp32 C.
//
// These configs exist ONLY to characterise the scalar FPU area for each knob
// choice.  They reuse RocketArty200TDroneFullDDRConfig (rocket-only drone SoC,
// NO Gemmini / NO Saturn) so elaboration is fast and cannot hit the Saturn
// elab OOM -- the FPU Verilog is a pure function of FPUParams, independent of
// the rest of the SoC, so this is byte-identical FPU RTL to what the combined
// int-only-Saturn config would emit.
//
// FPU knob matrix (see rocket-chip FPUParams: minFLen, fLen, divSqrt):
//   FDH  (baseline)      minFLen=16 fLen=64 divSqrt=on  = full RV64FD + Zfh  (RocketArty200TDroneFullDDRConfig itself)
//   HF   WithRocketFPU32 minFLen=16 fLen=32 divSqrt=on  = fp16+fp32, no double
//   F32  F-only          minFLen=32 fLen=32 divSqrt=on  = fp32 ONLY (minimal, HW div/sqrt)
//   F32nd F-only nodiv   minFLen=32 fLen=32 divSqrt=off = fp32 ONLY, SW Newton div/sqrt (leanest)
package chipyard.fpga.arty200t

import org.chipsalliance.cde.config._

// Pure fp32-only scalar FPU: drop fp16 (minFLen 16->32) AND fp64 (fLen 64->32).
class WithRocketFPUF32Only extends freechips.rocketchip.rocket.RocketCoreConfig(
  c => c.copy(fpu = c.fpu.map(_.copy(minFLen = 32, fLen = 32))))

// HF: fp16+fp32 (the fragment already named in-tree), no double.
class RbFpuHF32DroneConfig extends Config(
  new freechips.rocketchip.rocket.WithRocketFPU32 ++
  new RocketArty200TDroneFullDDRConfig)

// F32: minimal fp32-only, keep HW div/sqrt.
class RbFpuF32OnlyDroneConfig extends Config(
  new WithRocketFPUF32Only ++
  new RocketArty200TDroneFullDDRConfig)

// F32nd: minimal fp32-only, NO HW div/sqrt (fdiv/fsqrt trap-and-emulate or SW Newton).
class RbFpuF32OnlyNoDivSqrtDroneConfig extends Config(
  new freechips.rocketchip.rocket.WithFPUWithoutDivSqrt ++
  new WithRocketFPUF32Only ++
  new RocketArty200TDroneFullDDRConfig)

// ---------------------------------------------------------------------------
// COMBINED TASK-3 TARGET: int-only Saturn (DroNet) + minimal fp32 scalar FPU
// (fast native-fp32 FC).  The FPU-path analogue of RbFcRoCCConfig, byte-identical
// EXCEPT: no WithFcRoCC, and WithRocketFPUF32Only instead of WithoutFPU.  Because
// a scalar FPU is PRESENT, this also SIDESTEPS the WithoutFPU NoFpuConfig FIRRTL
// elab landmine that the RoCC path must patch.
// ---------------------------------------------------------------------------
class RocketArty200TDroneGemminiSaturnIntOnlyFpu32Config extends Config(
  new WithArty200TDroneFullDmaUartBase ++
  new WithRocketFPUF32Only ++                                            // minimal fp32-only scalar FPU (LEFT of Saturn => applied last => minFLen=32 wins over Saturn pin)
  new saturn.rocket.WithRocketVectorUnit(128, 64,
    saturn.common.VectorParams.intOnlyParams.copy(noPermute = false)) ++ // int-only Saturn, KEEP permute
  new gemmini.Q31GemminiConfig(RbArty200TGemmini.mesh16) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

// 35 MHz apples-to-apples with the deployed At35 combined; 44 MHz -2 route target.
class RocketArty200TDroneGemminiSaturnIntOnlyFpu32At35Config extends Config(
  new chipyard.harness.WithHarnessBinderClockFreqMHz(35) ++
  new chipyard.config.WithUniformBusFrequencies(35) ++
  new RocketArty200TDroneGemminiSaturnIntOnlyFpu32Config)
class RocketArty200TDroneGemminiSaturnIntOnlyFpu32At44Config extends Config(
  new chipyard.harness.WithHarnessBinderClockFreqMHz(44) ++
  new chipyard.config.WithUniformBusFrequencies(44) ++
  new RocketArty200TDroneGemminiSaturnIntOnlyFpu32Config)
