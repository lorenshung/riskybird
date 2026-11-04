package chipyard.fpga.arty100t

import chisel3._

import freechips.rocketchip.jtag.{JTAGIO}
import freechips.rocketchip.subsystem.{PeripheryBusKey}
import freechips.rocketchip.tilelink.{TLBundle}
import freechips.rocketchip.diplomacy.{LazyRawModuleImp}
import org.chipsalliance.diplomacy.nodes.{HeterogeneousBag}
import sifive.blocks.devices.uart.{UARTPortIO, UARTParams}
import sifive.blocks.devices.jtag.{JTAGPins, JTAGPinsFromPort}
import sifive.blocks.devices.pinctrl.{BasePin}
import sifive.fpgashells.shell._
import sifive.fpgashells.ip.xilinx._
import sifive.fpgashells.shell.xilinx._
import sifive.fpgashells.clocks._
import chipyard._
import chipyard.harness._
import chipyard.iobinders._
import testchipip.serdes._

class WithArty100TDDRTL extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: TLMemPort, chipId: Int) => {
    val artyTh = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty100THarness]
    val bundles = artyTh.ddrClient.get._1.out.map(_._1)
    val ddrClientBundle = Wire(new HeterogeneousBag(bundles.map(_.cloneType)))
    bundles.zip(ddrClientBundle).foreach { case (bundle, io) => bundle <> io }
    ddrClientBundle <> port.io
  }
})

class WithArty100TUARTTSI extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: UARTTSIPort, chipId: Int) => {
    val ath = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty100THarness]
    val harnessIO = IO(new UARTPortIO(port.io.uartParams)).suggestName("uart_tsi")
    harnessIO <> port.io.uart
    val packagePinsWithPackageIOs = Seq(
      ("P16" , IOPin(harnessIO.rxd)),
      ("U18", IOPin(harnessIO.txd)))
    packagePinsWithPackageIOs foreach { case (pin, io) => {
      ath.xdc.addPackagePin(io, pin)
      ath.xdc.addIOStandard(io, "LVCMOS33")
      ath.xdc.addIOB(io)
    } }
  }
})

// // Maps the UART device to the on-board USB-UART
class WithArty100TUART(rxdPin: String = "A9", txdPin: String = "D10") extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: UARTPort, chipId: Int) => {
    val ath = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty100THarness]
    val harnessIO = IO(chiselTypeOf(port.io)).suggestName("uart")
    harnessIO <> port.io
    val packagePinsWithPackageIOs = Seq(
      (rxdPin, IOPin(harnessIO.rxd)),
      (txdPin, IOPin(harnessIO.txd)))
    packagePinsWithPackageIOs foreach { case (pin, io) => {
      ath.xdc.addPackagePin(io, pin)
      ath.xdc.addIOStandard(io, "LVCMOS33")
      ath.xdc.addIOB(io)
    } }
  }
})

// The Zephyr console UART on the TE0712 carrier: RV_UART_RX / RV_UART_TX.
// rxd is an FPGA input (carrier drives it), txd an FPGA output.
class WithArty100TPMODUART extends WithArty100TUART("E21", "D21")

// JTAG into the RISC-V debug module. The defaults are the TE0712 drone carrier's
// four header pins (bank 16); a carrier that brings different balls out passes its
// own, the same way WithArty100TUART/WithArty100TI2C/WithArty100TOspi take theirs.
class WithArty100TJTAG(
  tckPin: String = "A19",
  tmsPin: String = "F19",
  tdiPin: String = "A18",
  tdoPin: String = "F20") extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: JTAGPort, chipId: Int) => {
    val ath = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty100THarness]
    val harnessIO = IO(new JTAGChipIO(false)).suggestName("jtag")
    harnessIO.TDO := port.io.TDO
    port.io.TCK := harnessIO.TCK
    port.io.TDI := harnessIO.TDI
    port.io.TMS := harnessIO.TMS
    port.io.reset.foreach(_ := th.referenceReset)

    ath.sdc.addClock("JTCK", IOPin(harnessIO.TCK), 10)
    ath.sdc.addGroup(clocks = Seq("JTCK"))
    ath.xdc.clockDedicatedRouteFalse(IOPin(harnessIO.TCK))
    val packagePinsWithPackageIOs = Seq(
      (tckPin, IOPin(harnessIO.TCK)),
      (tmsPin, IOPin(harnessIO.TMS)),
      (tdiPin, IOPin(harnessIO.TDI)),
      (tdoPin, IOPin(harnessIO.TDO))
    )
    
    packagePinsWithPackageIOs foreach { case (pin, io) => {
      ath.xdc.addPackagePin(io, pin)
      ath.xdc.addIOStandard(io, "LVCMOS33")
      ath.xdc.addPullup(io)
    } }
  }
})

// Brings the SoC's I2C controller out to two FPGA pins as an open-drain 2-wire bus.
// One TLI2C controller == one SCL/SDA bus that addresses many slaves (HM01B0 @ 0x24 + others).
// Placed in WithArty100TOspiPeriphery so it overrides the default chipyard.harness.WithI2CTiedOff.
// PINS: board-specific (xc7a100t-fgg484). I2C needs pull-ups: addPullup enables the weak internal
// pull-up; external ~4.7k is recommended for reliable I2C rise times.
class WithArty100TI2C(sclPin: String = "A15", sdaPin: String = "A16") extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: I2CPort, chipId: Int) => {
    val ath = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty100THarness]
    val harnessIO = IO(new ShellI2CPortIO).suggestName("i2c")
    // Open-drain: drive the line low only when output-enable is asserted; otherwise tri-state.
    UIntToAnalog(port.io.scl.out, harnessIO.scl, port.io.scl.oe)
    UIntToAnalog(port.io.sda.out, harnessIO.sda, port.io.sda.oe)
    port.io.scl.in := AnalogToUInt(harnessIO.scl).asBool
    port.io.sda.in := AnalogToUInt(harnessIO.sda).asBool
    Seq((sclPin, IOPin(harnessIO.scl)), (sdaPin, IOPin(harnessIO.sda))) foreach { case (pin, io) => {
      ath.xdc.addPackagePin(io, pin)
      ath.xdc.addIOStandard(io, "LVCMOS33")
      ath.xdc.addPullup(io)
    } }
  }
})

// Brings the HM01B0 capture peripheral's parallel-video pins out to the xc7a100t-fgg484 shell.
// Pins are the TE0712 drone-carrier mapping. dataPins is ordered D0..D7, i.e. index 0 is the LSB.
//
// VOLTAGE, AND WHY IT IS NOT A SINGLE SWITCH. hardware/ospi/docs/README.md records the HM01B0
// module IOVDD as 1.8 V. `ioStandard` therefore exists so the camera group can be moved to
// LVCMOS18 if its bank VCCO is set to 1.8 V -- but the interface is NOT confined to one bank.
// Measured on xc7a100tfgg484 (Vivado get_package_pins):
//
//   bank 15: D0-D7, PCLK, FVLD, LVLD, MCLK        <- 12 signals, movable to 1.8 V together
//   bank 14: TRIG, INT                            <- shares a bank with UART-TSI (P16/U18)
//
// Setting bank 14 to 1.8 V would take the UART-TSI loader pins with it and break the load path,
// so TRIG and INT cannot follow the rest of the camera group by a bank change alone. For
// bring-up neither is required: capture runs free-running, so TRIG is unused, and INT is only
// the motion interrupt. Leave them disconnected unless a translator is present.
class WithArty100TOspi(
  dataPins: Seq[String] = Seq("H22", "J22", "K18", "K19", "L19", "L20", "J19", "H19"),
  pclkPin: String = "H20",
  fvldPin: String = "K22",
  lvldPin: String = "G20",
  intrPin: String = "V20",
  mclkPin: String = "K21",
  trigPin: String = "U20",
  ioStandard: String = "LVCMOS33") extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: OspiPort, chipId: Int) => {
    val ath = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty100THarness]
    require(dataPins.size == 8, "HM01B0 8-bit mode requires exactly eight data pins")

    val harnessIO = IO(chiselTypeOf(port.io)).suggestName("ospi")
    harnessIO <> port.io

    dataPins.zipWithIndex.foreach { case (pin, i) =>
      val io = IOPin(harnessIO.d, i)
      ath.xdc.addPackagePin(io, pin)
      ath.xdc.addIOStandard(io, ioStandard)
    }

    val pclkIO = IOPin(harnessIO.pclk)
    ath.xdc.addPackagePin(pclkIO, pclkPin)
    ath.xdc.addIOStandard(pclkIO, ioStandard)

    val ctrlPins = Seq(
      (fvldPin, IOPin(harnessIO.fvld)), // FLVD (bundle keeps the historical `fvld` spelling)
      (lvldPin, IOPin(harnessIO.lvld)), // LVLD
      (intrPin, IOPin(harnessIO.intr)), // INT
      (mclkPin, IOPin(harnessIO.mclk)), // MCLK
      (trigPin, IOPin(harnessIO.trig))  // TRIG
    )

    ctrlPins.foreach { case (pin, io) =>
      ath.xdc.addPackagePin(io, pin)
      ath.xdc.addIOStandard(io, ioStandard)
    }

    ath.sdc.addClock("ospi_pclk", pclkIO, 36)
    ath.sdc.addGroup(clocks = Seq("ospi_pclk"))
    // H20 has not been confirmed as a clock-capable (MRCC/SRCC) package pin on this carrier. The
    // override permits routing at the HM01B0's low pixel-clock rate (<= 36 MHz per the datasheet
    // table in hardware/ospi/docs/README.md); move pclkPin to a clock-capable pin if one is wired.
    ath.xdc.clockDedicatedRouteFalse(pclkIO)
  }
})
