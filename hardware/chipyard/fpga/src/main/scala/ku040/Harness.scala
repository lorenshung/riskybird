package chipyard.fpga.ku040

import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import org.chipsalliance.cde.config.{Parameters}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.prci._
import freechips.rocketchip.subsystem.{SystemBusKey}

import sifive.fpgashells.shell.xilinx._
import sifive.fpgashells.shell._
import sifive.fpgashells.clocks._

import chipyard._
import chipyard.harness._

class KU040Harness(override implicit val p: Parameters) extends KU040Shell {
  def dp = designParameters

  val clockOverlay = dp(ClockInputOverlayKey).map(_.place(ClockInputDesignInput())).head
  val harnessSysPLL = dp(PLLFactoryKey)
  val harnessSysPLLNode = harnessSysPLL()
  val dutFreqMHz = (dp(SystemBusKey).dtsFrequency.get / (1000 * 1000)).toInt
  val dutClock = ClockSinkNode(freqMHz = dutFreqMHz)
  println(s"KU040 FPGA Base Clock Freq: ${dutFreqMHz} MHz")
  val dutWrangler = LazyModule(new ResetWrangler())
  val dutGroup = ClockGroup()
  dutClock := dutWrangler.node := dutGroup := harnessSysPLLNode

  harnessSysPLLNode := clockOverlay.overlayOutput.node

  /*** DDR ***/

  // The board carries two independent x16 DDR4 components, one per HP bank, so
  // a controller of KU040DDRSize can be placed against each. They are placed at
  // adjacent base addresses and joined by a crossbar, which presents them to the
  // SoC as one contiguous region -- rather than as two memory channels, whose
  // block-interleaved address sets each controller could not cover.
  //
  // How many are placed follows from ExtMem's size: one MIG per KU040DDRSize, in
  // DDROverlayKey's declaration order, so an ExtMem of one controller's size
  // leaves the bank-46 component (and its ~11,000 LUT) out of the design.
  //
  // Configurations without a TL backing memory (the scratchpad ones) leave
  // ExtTLMem undefined and instantiate no MIG at all.
  val ddrOverlays = dp(ExtTLMem).toSeq.flatMap { extMem =>
    val perController = p(KU040DDRSize)
    val placers = dp(DDROverlayKey)
    require(extMem.master.size % perController == 0 &&
            extMem.master.size / perController >= 1 &&
            extMem.master.size / perController <= placers.size,
      s"KU040 has ${placers.size} x ${perController} B of DDR4; ExtMem size " +
      s"${extMem.master.size} B is not a whole number of controllers within that")
    placers.take((extMem.master.size / perController).toInt).zipWithIndex.map { case (placer, i) =>
      placer.place(DDRDesignInput(extMem.master.base + perController * i, dutWrangler.node, harnessSysPLLNode))
        .asInstanceOf[DDRKU040PlacedOverlay]
    }
  }

  val ddrClient = if (ddrOverlays.isEmpty) None else Some {
    val extMem = dp(ExtTLMem).get
    val client = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLMasterParameters.v1(
      name = "chip_ddr",
      sourceId = IdRange(0, 1 << extMem.master.idBits)
    )))))
    val blockDuringReset = LazyModule(new TLBlockDuringReset(4))
    val xbar = LazyModule(new TLXbar)
    ddrOverlays.foreach { _.overlayOutput.ddr := xbar.node }
    xbar.node := blockDuringReset.node := TLWidthWidget(extMem.master.beatBytes) := client
    (client, blockDuringReset)
  }

  override lazy val module = new HarnessLikeImpl

  class HarnessLikeImpl extends Impl with HasHarnessInstantiators {
    clockOverlay.overlayOutput.node.out(0)._1.reset := ~resetPin

    val clk_200mhz = clockOverlay.overlayOutput.node.out.head._1.clock

    harnessSysPLL.plls.foreach(_._1.getReset.get := pllReset)

    def referenceClockFreqMHz = dutFreqMHz
    def referenceClock = dutClock.in.head._1.clock
    def referenceReset = dutClock.in.head._1.reset
    def success = { require(false, "Unused"); false.B }

    childClock := harnessBinderClock
    childReset := harnessBinderReset

    // Hold the memory port off until every controller has finished calibrating.
    ddrOverlays.foreach { o =>
      o.mig.module.clock := harnessBinderClock
      o.mig.module.reset := harnessBinderReset
    }
    ddrClient.foreach { case (_, blockDuringReset) =>
      val calibrated = ddrOverlays.map(_.mig.module.io.port.c0_init_calib_complete).reduce(_ && _)
      blockDuringReset.module.clock := harnessBinderClock
      blockDuringReset.module.reset := harnessBinderReset.asBool || !calibrated
    }

    instantiateChipTops()
  }
}
