// RiskyBird SoC-clock EXPERIMENTS (not signed-off configs). Safe to delete.
//
// Goal: raise the combined Gemmini16 + Saturn V128D64 + FP16 SoC fabric clock
// above 35 MHz to lift DroNet fps (cycles fixed, so fps scales linearly with the
// fabric clock). Each variant is the SAME combined design as
// RocketArty200TDroneGemminiSaturnFp16Config, only the harness PLL output
// (WithHarnessBinderClockFreqMHz) and the SDC bus period
// (WithUniformBusFrequencies) change -- exactly the pattern of the shipped At35
// variant. Everything else (pin contract, accelerators, DSP-slice injection) is
// identical. The DDR3 MIG UI clock (clk_pll_i, 100 MHz) is in its own domain
// across an async crossing and is unaffected.
//
// FIRMWARE CONTRACT: a different fabric clock changes the UART baud divisor and
// timebase; the generated DTS carries dtsFrequency so the FC image must be
// rebuilt to match before HW use.
package chipyard.fpga.arty200t

import org.chipsalliance.cde.config._

// 38 MHz (26.316 ns) -- just above the ~35.7 MHz as-placed fabric ceiling.
class RocketArty200TDroneGemminiSaturnFp16At38Config extends Config(
  new chipyard.harness.WithHarnessBinderClockFreqMHz(38) ++
  new chipyard.config.WithUniformBusFrequencies(38) ++
  new RocketArty200TDroneGemminiSaturnFp16Config)

// 40 MHz (25.0 ns) -- 9.0 fps at the DroNet cycle count.
class RocketArty200TDroneGemminiSaturnFp16At40Config extends Config(
  new chipyard.harness.WithHarnessBinderClockFreqMHz(40) ++
  new chipyard.config.WithUniformBusFrequencies(40) ++
  new RocketArty200TDroneGemminiSaturnFp16Config)

// 44 MHz (22.727 ns) -- ~9.9 fps (44.4 MHz is the 10 fps point).
class RocketArty200TDroneGemminiSaturnFp16At44Config extends Config(
  new chipyard.harness.WithHarnessBinderClockFreqMHz(44) ++
  new chipyard.config.WithUniformBusFrequencies(44) ++
  new RocketArty200TDroneGemminiSaturnFp16Config)

// 50 MHz (20.0 ns) -- 11.3 fps.
class RocketArty200TDroneGemminiSaturnFp16At50Config extends Config(
  new chipyard.harness.WithHarnessBinderClockFreqMHz(50) ++
  new chipyard.config.WithUniformBusFrequencies(50) ++
  new RocketArty200TDroneGemminiSaturnFp16Config)
