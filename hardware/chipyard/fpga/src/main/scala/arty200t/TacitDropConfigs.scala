// See LICENSE for license details.
//
// TACIT + DMA-sink variant of the COMBINED accelerator config with a TRIMMED
// peripheral set, for the LUT-critical XC7A200T. The full-peripheral combined
// (Gemmini16 + Saturn V128D64 + FP16 FPU) is already 92.8% LUT post-route; adding
// TACIT overflows/oversubscribes placement (1024-BP over-utilizes LUTs; even
// 256-BP passes the LUT DRC but the placer cannot pack the design at ~99% with
// its control-set count). Per the fit strategy, this drops the peripherals least
// needed for compute benchmarking (the ModelBlaster harness uses a fixed input
// tensor, not the live camera / sensors-in-loop):
//   DROPPED: camera OSPI capture + its DMA master, motor PWM (x2), PMW3901 flow
//            SPI, and the mem->uart1 telemetry TX-DMA.
//   KEPT   : DDR3 (essential -- weights/activations + the TACIT trace-DMA sink),
//            console uart0, ESP uart1, I2C (sensors), control GPIO (CS/reset/LED),
//            both accelerators, and the FP16 scalar FPU.
// The signed-off base classes in AccelConfigs.scala are untouched.
package chipyard.fpga.arty200t

import org.chipsalliance.cde.config._

// Trimmed drone periphery: mirrors WithArty200TDroneFullDmaUartBase but with the
// camera-OSPI+DMA fragment, motor PWM (device + harness + punchthrough), flow SPI
// (device + harness) and the telemetry TX-DMA removed. WithRiskyBirdDronePeriphery
// (which bundles 2x PWM + GPIO + SPI devices) is replaced by the GPIO device alone.
class WithArty200TDroneTraceBase extends Config(
  new WithArty200TUART("E13", "F14", uartNo = 1) ++         // ESP UART = uart1 (harness)
  new chipyard.config.WithUART(address = 0x10021000) ++     // uart1 device (console uart0 from AbstractConfig)
  new WithArty200TGPIO ++                                   // control GPIO harness (CS/reset/LED_N)
  new chipyard.config.WithGPIO(address = 0x10010000, width = 3) ++ // GPIO device (was in WithRiskyBirdDronePeriphery)
  new WithArty200TI2C ++                                    // I2C harness (sensors)
  new chipyard.config.WithI2C ++                            // I2C device
  new WithArty200TTweaks(ddr = true, uartTsi = false) ++
  new chipyard.config.WithBroadcastManager)                // no l2

// Combined accelerators + FP16 scalar FPU on the trimmed periphery base.
class RocketArty200TDroneGemminiSaturnFp16TraceBaseConfig extends Config(
  new WithArty200TDroneTraceBase ++
  new saturn.rocket.WithRocketVectorUnit(128, 64,
    saturn.common.VectorParams.robotMpcParams.copy(useElementwiseFP64 = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new gemmini.Q31GemminiConfig(RbArty200TGemmini.mesh16) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

// TACIT with a 128-entry branch predictor + DMA sink (trace still fully valid;
// smaller BP = slightly less branch-outcome compression, not lost information).
class WithTacitDmaTrace128 extends Config(
  new tacit.WithTraceSinkDMA(1) ++
  new WithTacitEncoderBP(nEntries = 128))

// COMBINED @35 MHz + TACIT (128-BP) on the trimmed base. The deployable combined
// trace bitstream.
class RocketArty200TDroneGemminiSaturnFp16At35TacitTrimConfig extends Config(
  new WithTacitDmaTrace128 ++
  new chipyard.harness.WithHarnessBinderClockFreqMHz(35) ++
  new chipyard.config.WithUniformBusFrequencies(35) ++
  new RocketArty200TDroneGemminiSaturnFp16TraceBaseConfig)

// Same trimmed combined but with the full 1024-entry BP, in case the trim frees
// enough that the larger predictor fits (better trace compression).
class RocketArty200TDroneGemminiSaturnFp16At35TacitTrim1024Config extends Config(
  new WithTacitDmaTrace ++
  new chipyard.harness.WithHarnessBinderClockFreqMHz(35) ++
  new chipyard.config.WithUniformBusFrequencies(35) ++
  new RocketArty200TDroneGemminiSaturnFp16TraceBaseConfig)
