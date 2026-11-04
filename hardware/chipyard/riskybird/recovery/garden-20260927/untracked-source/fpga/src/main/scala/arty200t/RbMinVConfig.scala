// See LICENSE for license details.
//
// Minimal int-only-Saturn V-race repro vehicle: Rocket + INT-ONLY Saturn V128D64,
// NO Gemmini, NO FcRoCC. Same base periphery (UART/JTAG/DDR) as the FcRoCC deploy
// config (board.tcl/MIG are permission-gated, so the base is kept), but the two
// big LUT consumers (Gemmini 16x16 mesh + FcRoCC) are dropped -> fast P&R
// (~20-30min vs ~90min At35) as the fast HW-iteration vehicle for bug B (the
// eager-V-restore load-access V-race in the FP-stripped Saturn).
package chipyard.fpga.arty200t

import org.chipsalliance.cde.config._

class RocketArty200TDroneSaturnIntOnlyMinConfig extends Config(
  new WithArty200TDroneFullDmaUartBase ++
  new saturn.rocket.WithRocketVectorUnit(128, 64,
    saturn.common.VectorParams.intOnlyParams.copy(noPermute = false)) ++ // int-only Saturn, KEEP permute
  new freechips.rocketchip.rocket.WithoutFPU ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

// 35 MHz variant (matches the deployed clock; minimal config has ample headroom so it closes fast).
class RocketArty200TDroneSaturnIntOnlyMinAt35Config extends Config(
  new chipyard.harness.WithHarnessBinderClockFreqMHz(35) ++
  new chipyard.config.WithUniformBusFrequencies(35) ++
  new RocketArty200TDroneSaturnIntOnlyMinConfig)
