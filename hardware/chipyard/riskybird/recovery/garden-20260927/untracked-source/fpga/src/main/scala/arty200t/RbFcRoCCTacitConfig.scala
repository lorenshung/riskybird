// See LICENSE for license details.
//
// DEBUG variant: TACIT instruction-trace on the FC-RoCC arch config.
// On-chip RVV / eager-V instruction trace to capture the preemption ->
// context-switch -> fault sequence (V-race #2 debugging on the int-only Saturn).
// SEPARATE from the deployable RoCC config -- this is a debug build only.
//
// Base = RocketArty200TDroneGemminiSaturnFcRoCCAt35Config (int-only misaVfix
// Saturn + Gemmini + FcRoCC, At35). TACIT = WithTacitDmaTrace (1024-entry BP
// encoder + TraceSinkDMA target 1), the same mixin the signed-off
// RocketArty200TDroneGemminiSaturnFp16At35TacitConfig uses.
//
// Fit (diagram-agent measured): TACIT ~5.0-5.3K LUT + ~0 BRAM; the RoCC config
// is 83.7% LUT with ~21.8K (16.3%) free -> At35 + TACIT lands ~87.7% (~12%
// headroom). At35 is the right target (At44 + TACIT is congestion-risky).
//
// Debug build (one command, when wanted):
//   make -C fpga SUB_PROJECT=arty200t \
//     CONFIG=RocketArty200TDroneGemminiSaturnFcRoCCTacitAt35Config bitstream
// Trace decode: capture the DMA'd trace bytes, decode with software/tacit_decoder
// (see TacitAccelConfigs.scala header). Note: the tacit_decoder has no Gemmini
// disasm; RVV/eager-V ops decode.
package chipyard.fpga.arty200t

import org.chipsalliance.cde.config._

// Full-strength TACIT (1024-entry BP) + DMA sink on the FC-RoCC int-only-Saturn At35 base.
class RocketArty200TDroneGemminiSaturnFcRoCCTacitAt35Config extends Config(
  new WithTacitDmaTrace ++
  new RocketArty200TDroneGemminiSaturnFcRoCCAt35Config)

// Reduced-BP (256-entry) fallback, if the 1024-BP encoder overflows LUTs at At35.
class RocketArty200TDroneGemminiSaturnFcRoCCTacit256At35Config extends Config(
  new WithTacitDmaTrace256 ++
  new RocketArty200TDroneGemminiSaturnFcRoCCAt35Config)
