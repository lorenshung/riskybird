// See LICENSE for license details.
//
// TACIT instruction-trace + DMA-sink variants of the signed-off Arty-200T
// accelerator configs. See the header block below for the DDR trace-buffer /
// readback contract. The signed-off base classes in AccelConfigs.scala are
// untouched; these only ADD the trace unit on top.
//
// Composition (leftmost = outermost in CDE, applied last):
//   WithTraceSinkDMA(1) -- appends a TraceSinkDMA (target id 1) to the encoder's
//                          buildSinks, and installs TraceSinkDMAInjector, which
//                          gives the sink a TL master into the SBUS (-> DDR) plus
//                          a TL-slave regnode for the software DMA descriptor.
//   WithTacitEncoder(BP)-- sets traceParams=Some(TacitEncoder(...)) on the tile
//                          and enableTraceCoreIngress on the Rocket core.
//
// READBACK / trace-buffer DDR region (per tile, tileId 0):
//   Encoder control regnode : 0x3000000  (4 KB page; enable trace, select target 1)
//   DMA sink regnode        : 0x3010000  (4 KB page)
//       +0x00  flush (W1)   +0x04 done (R)   +0x08 dma_start_addr (RW,64b)
//       +0x10  addr_counter (R, bytes written)
//   Program dma_start_addr to a reserved DDR range (DDR base 0x8000_0000), enable
//   the encoder + select target 1, run the workload, flush, then read
//   addr_counter bytes back and decode with software/tacit_decoder:
//     cargo run -- --binary <elf> --encoded-trace <trace.bin> --to_txt
//
// NOTE: both trace regnodes were widened from 0xFF (256 B) to 0xFFF (4 KB) in
// TraceEncoderController.scala (rocket-chip) and TraceSinkDMA.scala (tacit) so
// they satisfy Saturn's PipelinedFaultCheck page-alignment requirement.
package chipyard.fpga.arty200t

import org.chipsalliance.cde.config._
import freechips.rocketchip.diplomacy.LazyModule
import freechips.rocketchip.subsystem.{TilesLocated, InSubsystem}
import freechips.rocketchip.trace.{TraceCoreParams, TraceEncoderParams}
import tacit.{TacitEncoder, TacitBPParams}

// Parameterized Tacit encoder fragment: like chipyard.WithTacitEncoder but with a
// tunable branch-predictor entry count and buffer depth, so the combined
// (LUT-critical) config can shrink the 1024-entry DSC counter table (a big NxM
// read/update mux) when synth pushes past 100%.
class WithTacitEncoderBP(nEntries: Int = 1024, bufferDepth: Int = 16) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: freechips.rocketchip.subsystem.RocketTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      traceParams = Some(TraceEncoderParams(
        encoderBaseAddr = 0x3000000 + tp.tileParams.tileId * 0x1000,
        buildEncoder = (p: Parameters) => LazyModule(new TacitEncoder(new TraceCoreParams(
          nGroups = 1,
          xlen = tp.tileParams.core.xLen,
          iaddrWidth = tp.tileParams.core.xLen
        ),
        bufferDepth = bufferDepth,
        coreStages = 5,
        bpParams = TacitBPParams(xlen = tp.tileParams.core.xLen, n_entries = nEntries))(p)),
        useArbiterMonitor = false
      )),
      core = tp.tileParams.core.copy(enableTraceCoreIngress = true)))
    case other => other
  }
})

// Full-strength TACIT (1024-entry BP): DMA sink (target id 1) + encoder.
class WithTacitDmaTrace extends Config(
  new tacit.WithTraceSinkDMA(1) ++
  new chipyard.WithTacitEncoder)

// Reduced-BP TACIT (256-entry BP): for the LUT-critical combined config.
class WithTacitDmaTrace256 extends Config(
  new tacit.WithTraceSinkDMA(1) ++
  new WithTacitEncoderBP(nEntries = 256))

// ===========================================================================
// 1) COMBINED Gemmini16 + Saturn V128D64 + FP16 FPU @35 MHz + TACIT/DMA.
// ===========================================================================
class RocketArty200TDroneGemminiSaturnFp16At35TacitConfig extends Config(
  new WithTacitDmaTrace ++
  new RocketArty200TDroneGemminiSaturnFp16At35Config)

// Reduced-BP variant of the combined (256-entry BP) if 1024 overflows LUTs.
class RocketArty200TDroneGemminiSaturnFp16At35Tacit256Config extends Config(
  new WithTacitDmaTrace256 ++
  new RocketArty200TDroneGemminiSaturnFp16At35Config)

// ===========================================================================
// 2) Saturn V128D128 @40 MHz + TACIT/DMA.
// ===========================================================================
class RocketArty200TDroneSaturn40TacitConfig extends Config(
  new WithTacitDmaTrace ++
  new RocketArty200TDroneSaturn40Config)

// ===========================================================================
// 3) Gemmini-16 @40 MHz + TACIT/DMA (most headroom).
// ===========================================================================
class RocketArty200TDroneGemmini16At40TacitConfig extends Config(
  new WithTacitDmaTrace ++
  new RocketArty200TDroneGemmini16At40Config)
