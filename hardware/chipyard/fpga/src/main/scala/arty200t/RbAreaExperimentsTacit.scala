// Combined SoC + TACIT (128-entry BP, 128-bit DMA trace sink) + no-LoopConv on the
// FULL periphery (camera OSPI+DMA, motor PWM x2, flow SPI, telemetry TX-DMA all
// restored). Uses the TACIT infra in TacitDropConfigs.scala (WithTacitDmaTrace128)
// plus the tacit/rocket-chip generator fixes; HW trace capture validated.
//
// FIT RESULT (measured, full synth+P&R on garden): all 4 TACIT-trimmed peripherals
// RESTORED (camera OSPI+DMA, motor PWM x2, flow SPI, telemetry TX-DMA) + TACIT +
// both accelerators FITS at 124,421/134,600 LUT = 92.4% post-route, 425/740 DSP
// = 57.4% -- below the no-TACIT full-periphery baseline (92.8%). Removing LoopConv
// absorbs the whole LUT cost of TACIT, so the TacitTrim peripheral drop is unneeded.
package chipyard.fpga.arty200t

import org.chipsalliance.cde.config._

class RocketArty200TDroneGemminiSaturnFp16At35TacitFullNoLoopConvConfig extends Config(
  new WithTacitDmaTrace128 ++
  new chipyard.harness.WithHarnessBinderClockFreqMHz(35) ++
  new chipyard.config.WithUniformBusFrequencies(35) ++
  new RocketArty200TDroneGemminiSaturnFp16NoLoopConvConfig)
