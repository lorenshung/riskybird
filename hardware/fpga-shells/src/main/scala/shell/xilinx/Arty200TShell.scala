package sifive.fpgashells.shell.xilinx

import chisel3._
import chisel3.experimental.dataview._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.prci._
import org.chipsalliance.cde.config._
import sifive.fpgashells.clocks._
import sifive.fpgashells.devices.xilinx.xilinxarty200tmig._
import sifive.fpgashells.ip.xilinx._
import sifive.fpgashells.shell._

// class SysClockArty200TPlacedOverlay(val shell: Arty200TShellBasicOverlays, name: String, val designInput: ClockInputDesignInput, val shellInput: ClockInputShellInput)
//   extends SingleEndedClockInputXilinxPlacedOverlay(name, designInput, shellInput)
// {
//   val node = shell { ClockSourceNode(freqMHz = 100, jitterPS = 50) }

//   shell { InModuleBody {
//     val clk: Clock = io
//     shell.xdc.addPackagePin(clk, "E3")
//     shell.xdc.addIOStandard(clk, "LVCMOS33")
//   } }
// }

class SysClockArty200TPlacedOverlay(val shell: Arty200TShellBasicOverlays, name: String,
      val designInput: ClockInputDesignInput, val shellInput: ClockInputShellInput)
    extends LVDSClockInputXilinxPlacedOverlay(name, designInput, shellInput)   
  {
    val node = shell { ClockSourceNode(freqMHz = 50, jitterPS = 50) }         
    shell { InModuleBody {
      shell.xdc.addPackagePin(io.p, "H4")                                     
      shell.xdc.addPackagePin(io.n, "G4")
      shell.xdc.addIOStandard(io.p, "DIFF_SSTL15")                            
      shell.xdc.addIOStandard(io.n, "DIFF_SSTL15")
    } }
  }

class SysClockArty200TShellPlacer(val shell: Arty200TShellBasicOverlays, val shellInput: ClockInputShellInput)(implicit val valName: ValName)
  extends ClockInputShellPlacer[Arty200TShellBasicOverlays] {
  def place(designInput: ClockInputDesignInput) = new SysClockArty200TPlacedOverlay(shell, valName.name, designInput, shellInput)
}

case object Arty200TDDRSize extends Field[BigInt](0x40000000L) // 1 GB on TE0712
class DDRArty200TPlacedOverlay(val shell: Arty200TShellBasicOverlays, name: String, val designInput: DDRDesignInput, val shellInput: DDRShellInput)
  extends DDRPlacedOverlay[XilinxArty200TMIGPads](name, designInput, shellInput)
{
  val size = p(Arty200TDDRSize)

  val ddrClk1 = shell { ClockSinkNode(freqMHz = 50) } // must equal <InputClkFreq> in the prj
  val ddrGroup = shell { ClockGroup() }
  ddrClk1 := di.wrangler := ddrGroup := di.corePLL

  val migParams = XilinxArty200TMIGParams(address = AddressSet.misaligned(di.baseAddress, size))
  val mig = LazyModule(new XilinxArty200TMIG(migParams))
  val ddrUI     = shell { ClockSourceNode(freqMHz = 100) } // ui_clk = 800 MHz VCO / 8
  val areset    = shell { ClockSinkNode(Seq(ClockSinkParameters())) }
  areset := di.wrangler := ddrUI

  def overlayOutput = DDROverlayOutput(ddr = mig.node)
  def ioFactory = new XilinxArty200TMIGPads(size)

  shell { InModuleBody {
    require (shell.sys_clock.get.isDefined, "Use of DDRArty200TPlacedOverlay depends on SysClockArty200TPlacedOverlay")
    val (sys, _) = shell.sys_clock.get.get.overlayOutput.node.out(0)
    val (ui, _) = ddrUI.out(0)
    val (dclk1, _) = ddrClk1.in(0)
    val (ar, _) = areset.in(0)
    val port = mig.module.io.port

    io <> port.viewAsSupertype(new XilinxArty200TMIGPads(mig.depth))
    ui.clock := port.ui_clk
    ui.reset := !port.mmcm_locked || port.ui_clk_sync_rst
    port.sys_clk_i := dclk1.clock.asUInt
    port.sys_rst := !shell.pllReset // SysResetPolarity = ACTIVE LOW in prj
    port.aresetn := !(ar.reset.asBool)
  } }

  shell.sdc.addGroup(clocks = Seq("clk_pll_i"), pins = Seq(mig.island.module.blackbox.io.ui_clk))
}
class DDRArty200TShellPlacer(val shell: Arty200TShellBasicOverlays, val shellInput: DDRShellInput)(implicit val valName: ValName)
  extends DDRShellPlacer[Arty200TShellBasicOverlays] {
  def place(designInput: DDRDesignInput) = new DDRArty200TPlacedOverlay(shell, valName.name, designInput, shellInput)
}

abstract class Arty200TShellBasicOverlays()(implicit p: Parameters) extends Series7Shell {
  val sys_clock = Overlay(ClockInputOverlayKey, new SysClockArty200TShellPlacer(this, ClockInputShellInput()))
  val ddr       = Overlay(DDROverlayKey, new DDRArty200TShellPlacer(this, DDRShellInput()))
}

class Arty200TShell()(implicit p: Parameters) extends Arty200TShellBasicOverlays
{
  val resetPin = InModuleBody { Wire(Bool()) }
  // PLL reset causes
  val pllReset = InModuleBody { Wire(Bool()) }

  val topDesign = LazyModule(p(DesignKey)(designParameters))

  // Place the sys_clock at the Shell if the user didn't ask for it
  p(ClockInputOverlayKey).foreach(_.place(ClockInputDesignInput()))
  override lazy val module = new Impl
  class Impl extends LazyRawModuleImp(this) {
    override def provideImplicitClockToLazyChildren = true

    val sysclk: Clock = sys_clock.get() match {
      case Some(x: SysClockArty200TPlacedOverlay) => x.clock
    }
    val powerOnReset = PowerOnResetFPGAOnly(sysclk)
    sdc.addAsyncPath(Seq(powerOnReset))

    resetPin := true.B
    pllReset := powerOnReset
  }
}

/*
   Copyright 2016 SiFive, Inc.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
