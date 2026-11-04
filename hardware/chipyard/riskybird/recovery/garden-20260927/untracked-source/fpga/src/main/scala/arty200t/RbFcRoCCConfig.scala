// See LICENSE for license details.
//
// RiskyBird FC-RoCC target architecture: INT-ONLY Saturn (DroNet) + FC RoCC.
//
// The flight-controller estimator+PID moves off the shared Saturn/scalar core
// onto a fixed-function integer RoCC accelerator (chipyard.fc.FcRoCC, custom0),
// validated to fp32 quality by fc_rocc.c. That lets the Saturn be built
// INTEGER-ONLY (it sheds its fp16 vector-FP units: FMA/DivSqrt/Comp/Conv) and
// the Rocket scalar FPU be dropped, while the integer PermuteUnit is KEPT
// (noPermute=false) because DroNet's int8 maxpool (vslidedown) + gather need it.
//
// Net: -~12-15K LUT (Saturn FP strip + scalar FPU) + ~1.8K LUT (FC RoCC) to
// relieve the 92.8% LUT congestion of RocketArty200TDroneGemminiSaturnFp16At35Config
// and open routing headroom for the -2 clock bump.
//
// Base is a byte-for-byte match of RocketArty200TDroneGemminiSaturnFp16Config
// (same drone periphery/DDR/Gemmini), with three deltas:
//   robotMpcParams(fp16-only)        -> intOnlyParams(noPermute=false)
//   WithRocketFPU16 (scalar fp16)    -> WithoutFPU
//   + chipyard.fc.WithFcRoCC
package chipyard.fpga.arty200t

import org.chipsalliance.cde.config._

class RocketArty200TDroneGemminiSaturnFcRoCCConfig extends Config(
  new chipyard.fc.WithFcRoCC ++                                        // FC estimator+PID RoCC on custom0
  new WithArty200TDroneFullDmaUartBase ++
  new saturn.rocket.WithRocketVectorUnit(128, 64,
    saturn.common.VectorParams.intOnlyParams.copy(noPermute = false)) ++ // int-only Saturn, KEEP permute
  new freechips.rocketchip.rocket.WithoutFPU ++                        // drop scalar FPU (FC is on the RoCC)
  new gemmini.Q31GemminiConfig(RbArty200TGemmini.mesh16) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

// -2 route target: 44 MHz bus/harness (the clock the current 92.8%-LUT combined
// core cannot route). If the FP-strip relieves congestion, this should close.
class RocketArty200TDroneGemminiSaturnFcRoCCAt44Config extends Config(
  new chipyard.harness.WithHarnessBinderClockFreqMHz(44) ++
  new chipyard.config.WithUniformBusFrequencies(44) ++
  new RocketArty200TDroneGemminiSaturnFcRoCCConfig)

// 35 MHz variant (apples-to-apples LUT delta vs the deployed At35 combined).
class RocketArty200TDroneGemminiSaturnFcRoCCAt35Config extends Config(
  new chipyard.harness.WithHarnessBinderClockFreqMHz(35) ++
  new chipyard.config.WithUniformBusFrequencies(35) ++
  new RocketArty200TDroneGemminiSaturnFcRoCCConfig)

// noPermute variant: also drop the Saturn integer PermuteUnit (the
// vrgather/vslide/vcompress shuffle network), on top of the int-only strip
// above. Firmware audit of the DroNet workload (86 vector ops total) found
// ZERO permute-class RVV ops (0 vrgather/vslide/vcompress) actually emitted,
// so -- unlike the fp16 robotMpc-era assumption in the header comment above
// that int8 maxpool needed vslidedown+gather -- dropping the permute lane is
// firmware-safe for this int-only build. Targets the At44 config's core
// register-file WNS miss under -2 route congestion (WNS ~= -1.165ns, 92.8%
// LUT, all failing endpoints in the core RF) by shedding the permute
// crossbar's LUTs to relieve that congestion.
class RocketArty200TDroneGemminiSaturnFcRoCCNoPermuteConfig extends Config(
  new chipyard.fc.WithFcRoCC ++                                        // FC estimator+PID RoCC on custom0
  new WithArty200TDroneFullDmaUartBase ++
  new saturn.rocket.WithRocketVectorUnit(128, 64,
    saturn.common.VectorParams.intOnlyParams.copy(noPermute = true)) ++ // int-only Saturn, DROP permute
  new freechips.rocketchip.rocket.WithoutFPU ++                        // drop scalar FPU (FC is on the RoCC)
  new gemmini.Q31GemminiConfig(RbArty200TGemmini.mesh16) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

// -2 route target for the noPermute variant: 44 MHz bus/harness, the clock
// the permute-carrying FcRoCCAt44Config fails to close (core-RF WNS miss).
class RocketArty200TDroneGemminiSaturnFcRoCCNoPermuteAt44Config extends Config(
  new chipyard.harness.WithHarnessBinderClockFreqMHz(44) ++
  new chipyard.config.WithUniformBusFrequencies(44) ++
  new RocketArty200TDroneGemminiSaturnFcRoCCNoPermuteConfig)

// 40 MHz variant: the At44 noPermute build measured WNS -0.837 (LUT 92.8% ->
// 83.2%, congestion relieved) but didn't close -- residual is a deep-logic
// path (FcRoCC cmd-queue RAM -> core rf, 35 levels) that just needs more
// clock period, not more area relief. 40 MHz gives ~+1.4ns of period back
// over 44 MHz, which should clear that path and land the in-spec,
// timing-closed base for fully-preemptible DroNet + fixed-rate FC.
class RocketArty200TDroneGemminiSaturnFcRoCCNoPermuteAt40Config extends Config(
  new chipyard.harness.WithHarnessBinderClockFreqMHz(40) ++
  new chipyard.config.WithUniformBusFrequencies(40) ++
  new RocketArty200TDroneGemminiSaturnFcRoCCNoPermuteConfig)
