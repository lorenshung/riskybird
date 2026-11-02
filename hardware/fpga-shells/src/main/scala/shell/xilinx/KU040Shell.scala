package sifive.fpgashells.shell.xilinx

import chisel3._
import chisel3.experimental.dataview._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.prci._
import org.chipsalliance.cde.config._
import sifive.fpgashells.clocks._
import sifive.fpgashells.devices.xilinx.xilinxku040mig.{XilinxKU040MIG, XilinxKU040MIGPads, XilinxKU040MIGParams}
import sifive.fpgashells.ip.xilinx.ku040mig.KU040MIGIODDR
import sifive.fpgashells.ip.xilinx._
import sifive.fpgashells.shell._

// Shell for a custom XCKU040-1SFVA784C board (Trenz TE0841 rev 02), modeled on
// the (Trenz-modified) Arty100TShell for clocking and on the VCU118 shell for
// DDR4. Provides the 200 MHz LVDS system clock input and two DDR4 overlays;
// UART/JTAG pins are placed by chipyard harness binders.
class SysClockKU040PlacedOverlay(val shell: KU040ShellBasicOverlays, name: String,
      val designInput: ClockInputDesignInput, val shellInput: ClockInputShellInput)
    extends LVDSClockInputXilinxPlacedOverlay(name, designInput, shellInput)
{
  val node = shell { ClockSourceNode(freqMHz = 200, jitterPS = 50) }
  shell { InModuleBody {
    // sysclk_200_p/n; board XML specifies diff_term FALSE (IBUFDS default)
    shell.xdc.addPackagePin(io.p, "R25")
    shell.xdc.addPackagePin(io.n, "R26")
    shell.xdc.addIOStandard(io.p, "LVDS")
    shell.xdc.addIOStandard(io.n, "LVDS")
  } }
}

class SysClockKU040ShellPlacer(val shell: KU040ShellBasicOverlays, val shellInput: ClockInputShellInput)(implicit val valName: ValName)
  extends ClockInputShellPlacer[KU040ShellBasicOverlays] {
  def place(designInput: ClockInputDesignInput) = new SysClockKU040PlacedOverlay(shell, valName.name, designInput, shellInput)
}

/** Package pins for the two on-board DDR4 components, taken from the TE0841
  * board file `part0_pins.xml`.
  *
  * The order is the field order of [[XilinxKU040MIGPads]], because that is the
  * order `IOPin.of` walks the bundle in:
  *
  *   adr[0->16], bg, ba[0->1], reset_n, act_n, ck_c, ck_t, cke, cs_n, odt,
  *   dq[0->15], dqs_c[0->1], dqs_t[0->1], dm_dbi_n[0->1]
  */
object KU040DDRPins {
  // HP bank 44, board net prefix A_DDR4 (`c0_ddr4_*` in the board file)
  val bank44 = Seq(
    "AB27", "AF28", "AC27", "AE28", "AC26", "AH26", "AA28", "AG27", "AD26",
    "AF27", "AB25", "AC28", "AG26", "AH27", "AB24", "AG25", "AC23", // adr[0->16]
    "W23",                                                          // bg
    "AA27", "AF25",                                                 // ba[0->1]
    "AB26",                                                         // reset_n
    "AD25",                                                         // act_n
    "AE26", "AE25",                                                 // ck_c, ck_t
    "AC24",                                                         // cke
    "AE27",                                                         // cs_n
    "AD24",                                                         // odt
    "W24", "AA24", "Y27", "Y26", "V26", "AA25", "V24", "W26",
    "Y20", "AA20", "Y21", "AB21", "W21", "AA23", "AA22", "AB22",     // dq[0->15]
    "Y28", "Y23",                                                   // dqs_c[0->1]
    "W28", "Y22",                                                   // dqs_t[0->1]
    "W25", "AC22")                                                  // dm_dbi_n[0->1]

  // HP bank 46, board net prefix B_DDR4 (`c1_ddr4_*` in the board file)
  val bank46 = Seq(
    "B27", "H27", "B28", "J26", "B26", "K28", "A27", "K27", "A28",
    "H28", "C26", "C27", "H26", "J28", "A25", "K26", "K25",          // adr[0->16]
    "D26",                                                           // bg
    "F25", "L24",                                                    // ba[0->1]
    "A24",                                                           // reset_n
    "B25",                                                           // act_n
    "G26", "G25",                                                    // ck_c, ck_t
    "B24",                                                           // cke
    "J25",                                                           // cs_n
    "C24",                                                           // odt
    "D24", "E25", "E26", "F27", "D25", "G27", "D28", "E27",
    "H23", "K21", "J23", "K22", "G24", "K20", "H24", "L22",           // dq[0->15]
    "E28", "J24",                                                    // dqs_c[0->1]
    "F28", "K23",                                                    // dqs_t[0->1]
    "F24", "L23")                                                    // dm_dbi_n[0->1]
}

// 1 GiB per controller: one Samsung K4A8G165WB-BIRC (8 Gb x16) per HP bank.
// The board's 2 GiB total is two of these, joined in the harness.
case object KU040DDRSize extends Field[BigInt](0x40000000L)

class DDRKU040PlacedOverlay(val shell: KU040ShellBasicOverlays, name: String,
      val designInput: DDRDesignInput, val shellInput: DDRShellInput, packagePins: Seq[String], parPin: String)
  extends DDRPlacedOverlay[XilinxKU040MIGPads](name, designInput, shellInput)
{
  val size = p(KU040DDRSize)

  val migParams = XilinxKU040MIGParams(address = AddressSet.misaligned(di.baseAddress, size))
  val mig = LazyModule(new XilinxKU040MIG(migParams))
  // No CLOCK_DEDICATED_ROUTE constraint is needed on the MIG's MMCM input, even
  // though sysclk_200 (R25/R26) is in HP bank 45 while the controllers are in
  // banks 44 and 46. The Trenz reference design does need one, because there
  // System_Clock comes from a board interface and the IP instantiates the IBUFDS
  // itself, driving its MMCM over a direct dedicated route. Here System_Clock is
  // No_Buffer: the IBUFDS sits in the shell and the clock reaches each MMCM
  // through a BUFGCE, which already reaches every clock region -- so there is no
  // dedicated route to constrain. Confirmed by a clean place-and-route with no
  // CLKC-* or Place-30 clocking diagnostics.
  // ui_clk = memory clock / 4 = (1/1111ps) / 4 = 225 MHz (DDR4_CLKOUT0_DIVIDE 4)
  val ddrUI     = shell { ClockSourceNode(freqMHz = 225) }
  val areset    = shell { ClockSinkNode(Seq(ClockSinkParameters())) }
  areset := designInput.wrangler := ddrUI

  def overlayOutput = DDROverlayOutput(ddr = mig.node)
  def ioFactory = new XilinxKU040MIGPads

  shell { InModuleBody {
    require (shell.sys_clock.get().isDefined, "Use of DDRKU040PlacedOverlay depends on SysClockKU040PlacedOverlay")
    val (sys, _) = shell.sys_clock.get().get.overlayOutput.node.out(0)
    val (ui, _) = ddrUI.out(0)
    val (ar, _) = areset.in(0)
    val port = mig.module.io.port

    io <> port.viewAsSupertype(new KU040MIGIODDR)
    ui.clock := port.c0_ddr4_ui_clk
    ui.reset := port.c0_ddr4_ui_clk_sync_rst
    port.c0_sys_clk_i := sys.clock.asUInt
    // sys_rst is active high. The board's SC0841 system controller holds the
    // controller in reset until the DDR4 rail's power-good asserts
    // (MIG_RESET_OUT <= not PG_DDR); mirror that here.
    port.sys_rst := shell.pllReset || !shell.ddrPowerGood
    port.c0_ddr4_aresetn := !(ar.reset.asBool)

    val pins = IOPin.of(io)
    require (pins.size == packagePins.size,
      s"DDRKU040PlacedOverlay ${name}: ${pins.size} pads but ${packagePins.size} package pins")
    (pins zip packagePins) foreach { case (io, pin) => shell.xdc.addPackagePin(io, pin) }

    // Command/address parity. The MIG does not drive this -- CA parity is a
    // registered-DIMM feature -- but PAR is a real input on the component, so
    // hold it deasserted as the board's SC0841 does (C_DDR4_PAR_MODE 0 =>
    // DDR4_PAR_44/46 <= '0'). It lives here rather than in the shell so that it
    // only exists when its controller does, alongside that bank's other DCI
    // users.
    val par = IO(Output(Bool())).suggestName(s"${name}_par")
    par := false.B
    shell.xdc.addPackagePin(IOPin(par), parPin)
    shell.xdc.addIOStandard(IOPin(par), "SSTL12_DCI")
  } }

  shell.sdc.addGroup(pins = Seq(mig.island.module.blackbox.io.c0_ddr4_ui_clk))
}

class DDRKU040ShellPlacer(shell: KU040ShellBasicOverlays, val shellInput: DDRShellInput, packagePins: Seq[String], parPin: String)(implicit val valName: ValName)
  extends DDRShellPlacer[KU040ShellBasicOverlays] {
  def place(designInput: DDRDesignInput) = new DDRKU040PlacedOverlay(shell, valName.name, designInput, shellInput, packagePins, parPin)
}

abstract class KU040ShellBasicOverlays()(implicit p: Parameters) extends UltraScaleShell {
  val sys_clock = Overlay(ClockInputOverlayKey, new SysClockKU040ShellPlacer(this, ClockInputShellInput()))
  // Declaration order is placement order: dp(DDROverlayKey) is [bank44, bank46].
  val ddr_b44 = Overlay(DDROverlayKey, new DDRKU040ShellPlacer(this, DDRShellInput(), KU040DDRPins.bank44, parPin = "AD28")(valName = ValName("ddr_b44")))
  val ddr_b46 = Overlay(DDROverlayKey, new DDRKU040ShellPlacer(this, DDRShellInput(), KU040DDRPins.bank46, parPin = "C28")(valName = ValName("ddr_b46")))

  // PG_DDR, sampled from the board's DDR4 rail power-good.
  def ddrPowerGood: ModuleValue[Bool]
}

class KU040Shell()(implicit p: Parameters) extends KU040ShellBasicOverlays
{
  val resetPin = InModuleBody { Wire(Bool()) }
  // PLL reset causes
  val pllReset = InModuleBody { Wire(Bool()) }
  val ddrPowerGood = InModuleBody { Wire(Bool()) }

  val topDesign = LazyModule(p(DesignKey)(designParameters))

  // Place the sys_clock at the Shell if the user didn't ask for it
  p(ClockInputOverlayKey).foreach(_.place(ClockInputDesignInput()))
  override lazy val module = new Impl
  class Impl extends LazyRawModuleImp(this) {
    override def provideImplicitClockToLazyChildren = true

    val sysclk: Clock = sys_clock.get() match {
      case Some(x: SysClockKU040PlacedOverlay) => x.clock
    }
    val powerOnReset = PowerOnResetFPGAOnly(sysclk)
    sdc.addAsyncPath(Seq(powerOnReset))

    resetPin := true.B
    pllReset := powerOnReset

    // Board housekeeping, all bank 65 (HR, VCCO 3.3V). On the Trenz reference
    // design these are driven by the SC0841 system controller, which ties the
    // three enables high; with nothing driving them Vivado's default
    // BITSTREAM.CONFIG.UNUSEDPIN PULLDOWN leaves the 200 MHz MEMS oscillator
    // and the DDR4 rail switched off.
    val en_osc     = IO(Output(Bool())).suggestName("en_osc")
    val en_ddr4pwr = IO(Output(Bool())).suggestName("en_ddr4pwr")
    val pg_ddr     = IO(Input(Bool())).suggestName("pg_ddr")

    en_osc     := true.B
    en_ddr4pwr := true.B
    ddrPowerGood := pg_ddr

    Seq(("AF24", IOPin(en_osc)), ("AD20", IOPin(en_ddr4pwr)), ("AE20", IOPin(pg_ddr))).foreach {
      case (pin, io) =>
        xdc.addPackagePin(io, pin)
        xdc.addIOStandard(io, "LVCMOS33")
    }
    sdc.addAsyncPath(Seq(IOPin(pg_ddr)))
  }
}
