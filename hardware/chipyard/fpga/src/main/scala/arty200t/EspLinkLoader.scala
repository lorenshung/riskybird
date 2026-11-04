package chipyard.fpga.arty200t

import chisel3._

import org.chipsalliance.cde.config.{Config, Parameters}
import freechips.rocketchip.diplomacy.{LazyRawModuleImp}
import freechips.rocketchip.tilelink.{TLBundle}
import sifive.blocks.devices.uart.{UARTPortIO, UARTParams}
import sifive.fpgashells.shell._
import chipyard.harness._
import chipyard.iobinders._
import chipyard.esplink.{EspLinkMux, EspLinkLoaderParams, EspLinkXilinx}

// The mux RTL and its rationale live in generators/chipyard (esplink/EspLinkMux.scala):
// this sbt project is built without the Chisel compiler plugin, so it cannot define
// Bundles. Only the harness wiring is here.

/** One mux per harness, shared by the three binders below. Chipyard applies a
 *  binder per port, and the mux has three ports' worth of clients (uart1, the
 *  UART-TSI client, and the SoC reset), so whichever binder runs first builds
 *  it -- inside the harness body, under the harness binder clock -- and claims
 *  the two package pins exactly once. */
object EspLinkLoader {
  private val muxes = scala.collection.mutable.Map[HasHarnessInstantiators, EspLinkMux]()

  def apply(th: HasHarnessInstantiators, rxdPin: String, txdPin: String,
            c: EspLinkLoaderParams): EspLinkMux = muxes.getOrElseUpdate(th, {
    val ath = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty200THarness]
    val mux = Module(new EspLinkMux(th.getHarnessBinderClockFreqHz, c)).suggestName("esp_link_mux")
    // UARTPortIO for the pin pair: rxd in, txd out, and a Bundle this
    // plugin-less project can instantiate.
    val pins = IO(new UARTPortIO(UARTParams(0))).suggestName("esp_link")
    mux.io.pinRx := pins.rxd
    pins.txd     := mux.io.pinTx
    mux.io.uartTxd := true.B   // until the uart1 binder connects it
    mux.io.tsiTxd  := true.B   // until the TSI binder connects it
    mux.io.memQuiet := true.B  // until the memory-port binder connects it (no DDR: nothing to fence)
    Seq((rxdPin, IOPin(pins.rxd)), (txdPin, IOPin(pins.txd))).foreach { case (pin, io) =>
      ath.xdc.addPackagePin(io, pin)
      ath.xdc.addIOStandard(io, "LVCMOS33")
      ath.xdc.addIOB(io)
    }
    // A floating line (ESP in reset, unflashed, or unplugged) must read idle,
    // not as a break.
    ath.xdc.addPullup(IOPin(pins.rxd))

    // The configuration flash (S25FL256S) for remote bitstream updates: after
    // configuration its CS# and D0..D3 are user pins in bank 14 (3.3 V), and
    // its clock is CCLK, reachable only through STARTUPE2. D2/D3 are WP#/HOLD#
    // in single-bit SPI and are held high. Pin functions from Vivado's
    // xc7a200tfbg484 package: T19 FCS_B, P22 D00_MOSI, R22 D01_DIN, P21 D02,
    // R21 D03.
    val prims = Module(new EspLinkXilinx).suggestName("esp_link_xilinx")
    prims.io.sck  := mux.io.spi.sck
    prims.io.icap := mux.io.icap
    mux.io.imageId := prims.io.imageId
    val csn  = IO(Output(Bool())).suggestName("qspi_csn")
    val d0   = IO(Output(Bool())).suggestName("qspi_d0")
    val d1   = IO(Input(Bool())).suggestName("qspi_d1")
    val d2   = IO(Output(Bool())).suggestName("qspi_d2")
    val d3   = IO(Output(Bool())).suggestName("qspi_d3")
    csn := mux.io.spi.csn
    d0  := mux.io.spi.mosi
    mux.io.spi.miso := d1
    d2  := true.B
    d3  := true.B
    Seq((csn, "T19"), (d0, "P22"), (d1, "R22"), (d2, "P21"), (d3, "R21")).foreach { case (io, pin) =>
      ath.xdc.addPackagePin(IOPin(io), pin)
      ath.xdc.addIOStandard(IOPin(io), "LVCMOS33")
    }
    ath.xdc.addPullup(IOPin(csn))
    mux
  })
}

class WithArty200TEspLinkUART(rxdPin: String, txdPin: String, uartNo: Int, c: EspLinkLoaderParams)
    extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: UARTPort, chipId: Int) if port.uartNo == uartNo => {
    val mux = EspLinkLoader(th, rxdPin, txdPin, c)
    port.io.rxd := mux.io.uartRxd
    mux.io.uartTxd := port.io.txd
  }
})

class WithArty200TEspLinkTSI(rxdPin: String, txdPin: String, c: EspLinkLoaderParams)
    extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: UARTTSIPort, chipId: Int) => {
    val mux = EspLinkLoader(th, rxdPin, txdPin, c)
    port.io.uart.rxd := mux.io.tsiRxd
    mux.io.tsiTxd := port.io.uart.txd
  }
})

class WithArty200TEspLinkReset(rxdPin: String, txdPin: String, c: EspLinkLoaderParams)
    extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: ResetPort, chipId: Int) => {
    val mux = EspLinkLoader(th, rxdPin, txdPin, c)
    port.io := (th.referenceReset.asBool || mux.io.socReset).asAsyncReset
  }
})

/** WithArty200TDDRTL with a fence the mux can close before it resets the SoC.
 *
 *  The SoC reset does not reach the DDR side: the MIG and its TileLink buffer,
 *  TL-to-AXI4 converter, deinterleaver and clock crossing are reset only with
 *  the harness. A reset that lands inside a multi-beat burst (a cache-line
 *  writeback, say) would leave that state holding half a transaction, and the
 *  next requests on the port -- TSI's -- would be swallowed as its missing beats.
 *
 *  So while memFence is up the A channel is closed, but only at a burst
 *  boundary: a burst already started is let through to its last beat. Requests
 *  are counted against their final response beat, and memQuiet reports the port
 *  stopped with nothing outstanding; the mux resets only then. The fence opens
 *  again after the reset, for TSI. */
class WithArty200TEspLinkDDRTL(rxdPin: String, txdPin: String, c: EspLinkLoaderParams)
    extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: TLMemPort, chipId: Int) => {
    val artyTh = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty200THarness]
    val mux = EspLinkLoader(th, rxdPin, txdPin, c)
    val quiet = artyTh.ddrClient.get._1.out.zip(port.io).map { case ((ddr, edge), soc) =>
      ddr <> soc
      val aOpen = RegInit(false.B)   // a burst has started but not finished
      val close = mux.io.memFence && !aOpen
      ddr.a.valid := soc.a.valid && !close
      soc.a.ready := ddr.a.ready && !close
      val (aFirst, aLast, _) = edge.firstlast(ddr.a)
      val (_, dLast, _) = edge.firstlast(ddr.d)
      when (ddr.a.fire) { aOpen := !aLast }
      val outstanding = RegInit(0.U(16.W))
      outstanding := outstanding + (ddr.a.fire && aFirst).asUInt - (ddr.d.fire && dLast).asUInt
      !aOpen && outstanding === 0.U
    }
    mux.io.memQuiet := quiet.reduce(_ && _)
  }
})

/** WithArty200TPWM, plus a motor inhibit that a loader reset arms.
 *
 *  The sifive PWM's comparator registers have no reset (rocket-chip-blocks
 *  util/Timer.scala: `cmp` is a RegEnable). A SoC reset therefore zeroes the
 *  counter but keeps whatever compare values the previous program left, so for
 *  every nonzero one `(0 >= cmp)` is false, the raw output is low, and the
 *  harness inversion drives the motor pin HIGH -- full on -- from the reset
 *  until the next program reprograms the PWM, which during a load is minutes
 *  away, and never if the new program does not use the FPGA PWM at all. (At
 *  configuration every register starts at zero, so power-on is safe.)
 *
 *  Here each pin is forced low from the loader's SoC reset until its raw
 *  output reads high -- the "off" state, which the PWM reaches as soon as
 *  software rewrites cmp or starts the counter -- and behaves exactly like
 *  WithArty200TPWM after that. It covers the loader's reset only; a JTAG
 *  debug-module reset is inside the SoC and invisible here. */
class WithArty200TEspLinkPWM(
  rxdPin: String,
  txdPin: String,
  c: EspLinkLoaderParams,
  // (harness port name, PWM block, comparator, package pin): as WithArty200TPWM
  motors: Seq[(String, Int, Int, String)] = Seq(
    ("motor1", 0, 1, "E22"),   // J1_31
    ("motor2", 0, 2, "B20"),   // J1_36
    ("motor3", 0, 3, "R17"),   // J1_99
    ("motor4", 1, 1, "F13")))  // J1_100
    extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: PWMPort, chipId: Int) => {
    val ath = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty200THarness]
    val mux = EspLinkLoader(th, rxdPin, txdPin, c)
    motors.filter(_._2 == port.pwmId).foreach { case (name, _, cmp, pin) =>
      require(cmp > 0 && cmp < port.io.gpio.size,
        s"$name: comparator $cmp is not a usable output of PWM block ${port.pwmId}")
      val raw = port.io.gpio(cmp)
      val inhibit = RegInit(false.B).suggestName(s"${name}_inhibit")
      when (mux.io.socReset) { inhibit := true.B } .elsewhen (raw) { inhibit := false.B }
      val harnessIO = IO(Output(Bool())).suggestName(name)
      harnessIO := !inhibit && !raw   // inverted, as WithArty200TPWM; see its POLARITY note
      ath.xdc.addPackagePin(IOPin(harnessIO), pin)
      ath.xdc.addIOStandard(IOPin(harnessIO), "LVCMOS33")
      ath.xdc.addPulldown(IOPin(harnessIO))
    }
  }
})

/** uart1 and a UART-TSI client sharing the ESP pins; see EspLinkMux. Replaces
 *  WithArty200TUART(rxdPin, txdPin, uartNo) -- use one or the other, never
 *  both, or the pins are claimed twice. Needs testchipip.tsi.WithUARTTSIClient
 *  and WithArty200TTweaks(uartTsi = false), so the client exists but the
 *  Arty-pinned P16/U18 binder does not. Also overrides the DDR memory-port and
 *  PWM binders (WithArty200TDDRTL, WithArty200TPWM) with fenced/inhibited
 *  versions, because a loader reset is a SoC-only reset. */
class WithArty200TEspLinkLoader(
  rxdPin: String = "E13",
  txdPin: String = "F14",
  uartNo: Int = 1,
  c: EspLinkLoaderParams = EspLinkLoaderParams()) extends Config(
  new WithArty200TEspLinkUART(rxdPin, txdPin, uartNo, c) ++
  new WithArty200TEspLinkTSI(rxdPin, txdPin, c) ++
  new WithArty200TEspLinkReset(rxdPin, txdPin, c) ++
  new WithArty200TEspLinkDDRTL(rxdPin, txdPin, c) ++   // only fires where there is DDR
  new WithArty200TEspLinkPWM(rxdPin, txdPin, c))       // only fires where there is PWM
