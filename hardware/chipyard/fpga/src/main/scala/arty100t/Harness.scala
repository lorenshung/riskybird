package chipyard.fpga.arty100t

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
import sifive.fpgashells.ip.xilinx.{IBUF, PowerOnResetFPGAOnly}

import sifive.blocks.devices.uart._

import chipyard._
import chipyard.harness._

class Arty100THarness(override implicit val p: Parameters) extends Arty100TShell {
  def dp = designParameters

  val clockOverlay = dp(ClockInputOverlayKey).map(_.place(ClockInputDesignInput())).head
  val harnessSysPLL = dp(PLLFactoryKey)
  val harnessSysPLLNode = harnessSysPLL()
  val dutFreqMHz = (dp(SystemBusKey).dtsFrequency.get / (1000 * 1000)).toInt
  val dutClock = ClockSinkNode(freqMHz = dutFreqMHz)
  println(s"Arty100T FPGA Base Clock Freq: ${dutFreqMHz} MHz")
  val dutWrangler = LazyModule(new ResetWrangler())
  val dutGroup = ClockGroup()
  dutClock := dutWrangler.node := dutGroup := harnessSysPLLNode

  harnessSysPLLNode := clockOverlay.overlayOutput.node

  // Configurations without a TL backing memory -- the 32 KiB mbus scratchpad
  // ones -- leave ExtTLMem undefined and instantiate no MIG at all. This
  // mirrors the KU040 harness, where the same choice is already exercised.
  val ddrOverlay = dp(ExtTLMem).map { extMem =>
    dp(DDROverlayKey).head
      .place(DDRDesignInput(extMem.master.base, dutWrangler.node, harnessSysPLLNode))
      .asInstanceOf[DDRArtyPlacedOverlay]
  }
  val ddrClient = dp(ExtTLMem).map { extMem =>
    val client = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLMasterParameters.v1(
      name = "chip_ddr",
      sourceId = IdRange(0, 1 << extMem.master.idBits)
    )))))
    val blockDuringReset = LazyModule(new TLBlockDuringReset(4))
    ddrOverlay.get.overlayOutput.ddr := blockDuringReset.node := client
    (client, blockDuringReset)
  }

  override lazy val module = new HarnessLikeImpl

  class HarnessLikeImpl extends Impl with HasHarnessInstantiators {
    // all_leds.foreach(_ := DontCare)
    clockOverlay.overlayOutput.node.out(0)._1.reset := ~resetPin

    val clk_100mhz = clockOverlay.overlayOutput.node.out.head._1.clock

    harnessSysPLL.plls.foreach(_._1.getReset.get := pllReset)

    def referenceClockFreqMHz = dutFreqMHz
    def referenceClock = dutClock.in.head._1.clock
    def referenceReset = dutClock.in.head._1.reset
    def success = { require(false, "Unused"); false.B }

    childClock := harnessBinderClock
    childReset := harnessBinderReset

    // Hold the memory port off until the controller has finished calibrating.
    ddrOverlay.foreach { o =>
      o.mig.module.clock := harnessBinderClock
      o.mig.module.reset := harnessBinderReset
    }
    ddrClient.foreach { case (_, blockDuringReset) =>
      blockDuringReset.module.clock := harnessBinderClock
      blockDuringReset.module.reset := harnessBinderReset.asBool ||
        !ddrOverlay.get.mig.module.io.port.init_calib_complete
    }

    instantiateChipTops()
  }
}
