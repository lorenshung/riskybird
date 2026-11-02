package chipyard.fpga.arty200t

import chisel3._
import chisel3.experimental.{Analog}

import freechips.rocketchip.jtag.{JTAGIO}
import freechips.rocketchip.subsystem.{PeripheryBusKey}
import freechips.rocketchip.tilelink.{TLBundle}
import freechips.rocketchip.diplomacy.{LazyRawModuleImp}
import org.chipsalliance.diplomacy.nodes.{HeterogeneousBag}
import sifive.blocks.devices.uart.{UARTPortIO, UARTParams}
import sifive.blocks.devices.jtag.{JTAGPins, JTAGPinsFromPort}
import sifive.blocks.devices.pinctrl.{BasePin}
import sifive.blocks.devices.spi.{SPIPortIO}
import sifive.blocks.devices.gpio.{GPIOPortIO}
import sifive.fpgashells.shell._
import sifive.fpgashells.ip.xilinx._
import sifive.fpgashells.shell.xilinx._
import sifive.fpgashells.clocks._
import chipyard._
import chipyard.harness._
import chipyard.iobinders._
import testchipip.serdes._

class WithArty200TDDRTL extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: TLMemPort, chipId: Int) => {
    val artyTh = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty200THarness]
    val bundles = artyTh.ddrClient.get._1.out.map(_._1)
    val ddrClientBundle = Wire(new HeterogeneousBag(bundles.map(_.cloneType)))
    bundles.zip(ddrClientBundle).foreach { case (bundle, io) => bundle <> io }
    ddrClientBundle <> port.io
  }
})

class WithArty200TUARTTSI extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: UARTTSIPort, chipId: Int) => {
    val ath = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty200THarness]
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
class WithArty200TUART(rxdPin: String = "A9", txdPin: String = "D10", uartNo: Int = 0) extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: UARTPort, chipId: Int) if port.uartNo == uartNo => {
    val ath = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty200THarness]
    // One harness IO per UART, and the name has to be unique: a two-UART shell fires this
    // binder once per uartNo, and both instances naming themselves "uart" is a fatal Chisel
    // elaboration error. uart0 keeps the bare "uart" so every single-UART config's generated
    // port names (and the XDC that references them) are unchanged.
    //
    // ku040-codesign-cnn fixed the same duplicate-name bug independently, as
    // suggestName(s"uart${port.uartNo}"). That is equivalent for every config in
    // AccelConfigs.scala -- they all carry two UARTs, and uart1 gets the same name either
    // way -- and differs only in renaming uart0 from "uart" to "uart0", which would churn
    // the generated port names and XDC of every single-UART config. This side is kept for
    // that reason.
    val harnessIO = IO(chiselTypeOf(port.io))
      .suggestName(if (uartNo == 0) "uart" else s"uart$uartNo")
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
class WithArty200TPMODUART extends WithArty200TUART("E21", "D21")

// JTAG into the RISC-V debug module. The defaults are the TE0712 drone carrier's
// four header pins (bank 16); a carrier that brings different balls out passes its
// own, the same way WithArty200TUART/WithArty200TI2C/WithArty200TOspi take theirs.
class WithArty200TJTAG(
  tckPin: String = "A19",
  tmsPin: String = "F19",
  tdiPin: String = "A18",
  tdoPin: String = "F20") extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: JTAGPort, chipId: Int) => {
    val ath = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty200THarness]
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
// Placed in WithArty200TOspiPeriphery so it overrides the default chipyard.harness.WithI2CTiedOff.
// PINS: board-specific (xc7a200t-fbg484). I2C needs pull-ups: addPullup enables the weak internal
// pull-up; external ~4.7k is recommended for reliable I2C rise times.
class WithArty200TI2C(sclPin: String = "A15", sdaPin: String = "A16") extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: I2CPort, chipId: Int) => {
    val ath = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty200THarness]
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

// PMW3901 optical-flow SPI, brought out as three single-mode signals.
//
// The sifive SPI port is a quad interface: dq(0) is MOSI and dq(1) is MISO in single mode, and
// dq(2)/dq(3) are only used for dual/quad transfers. This binder wires the three single-mode
// signals and leaves dq(2)/dq(3) unbound, which is what the PMW3901 needs -- it is a plain
// mode-3 SPI device.
//
// The hard chip-select is deliberately NOT bound. hardware/zephyr/targets/fpga/workloads/
// pmw3901_test.overlay drives CS from gpio0 bit 0 instead, because the driver holds CS low
// across a multi-byte register transaction and the sifive controller's automatic CS does not.
//
// PINS. F16/E17/E16 with NCS on D16, which are J1_82/J1_80/J1_78/J1_76 -- the same J1 connector,
// and the same even row, as the JTAG and I2C signals that work on this bench.
//
// These supersede the SCLK L21 / MISO M21 / MOSI N20 / NCS M20 recorded in the bank survey at
// docs/hm01b0-camera-bringup.md:163. Those are J2_62/64/66/68, on the other connector, and a
// bitstream built with them measured an undriven MISO: with a pulldown the sensor read back a
// clean 0x00, which a connected device could not produce. The sensor itself was never in doubt
// -- it flies under the ESP32 on this same board.
//
// misoPulldown defaults on, and that is a bring-up decision rather than an
// electrical preference. With no pull, an unconnected MISO floats high and a
// missing sensor reads back 0xFF -- which is indistinguishable from a sensor
// that is present but never driving, and is exactly the ambiguity that stalled
// this bring-up. A weak pulldown makes "nothing is driving this pin" read as
// 0x00 instead, so absence and silence stop looking alike. A real device
// asserting the line overpowers it easily.
class WithArty200TSPI(
  sckPin: String = "F16",
  mosiPin: String = "E17",
  misoPin: String = "E16",
  misoPulldown: Boolean = true) extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: SPIPort, chipId: Int) => {
    val ath = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty200THarness]
    // Three separate IOs rather than one anonymous Bundle: a Bundle declared
    // inside a HarnessBinder's partial function is not seen by the Chisel
    // compiler plugin, and elaboration fails on its "plugin is now required"
    // assertion at FIRRTL emission rather than at compile time.
    val sck  = IO(Output(Bool())).suggestName("spi_sck")
    val mosi = IO(Output(Bool())).suggestName("spi_mosi")
    val miso = IO(Input(Bool())).suggestName("spi_miso")

    sck  := port.io.sck
    mosi := port.io.dq(0).o
    port.io.dq(0).i := false.B          // MOSI is output-only here
    port.io.dq(1).i := miso
    port.io.dq(2).i := false.B
    port.io.dq(3).i := false.B

    Seq((sckPin, IOPin(sck)), (mosiPin, IOPin(mosi)), (misoPin, IOPin(miso))) foreach {
      case (pin, io) => {
        ath.xdc.addPackagePin(io, pin)
        ath.xdc.addIOStandard(io, "LVCMOS33")
      }
    }
    if (misoPulldown) {
      ath.xdc.addPulldown(IOPin(miso))
    }
  }
})

// The DroneLogic GPIO block: three bits, ordered to match
// hardware/zephyr/targets/fpga/workloads/pmw3901_test.overlay -- bit 0 CS, bit 1 reset, bit 2 LED.
//
// PINS. Bit 0 is the PMW3901's NCS at D16 (J1_76), alongside the SPI signals above.
//
// Bits 1 and 2 are NRESET and LED_N, and those two nets exist on neither the FPGA nor the ESP32
// -- the sensor is flown without them. So their pins here are arbitrary placeholders on J1 nets
// this repository never names, present only because the GPIO block is three bits wide and
// pmw3901_test's devicetree contract requires all three properties. Nothing depends on them:
// the driver treats reset as optional (`if (config->reset_gpio.port)`) and the LED is cosmetic,
// so a chip-ID read needs only SCK/MOSI/MISO/CS.
class WithArty200TGPIO(
  pins: Seq[String] = Seq("D16", "A20", "A21")) extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: GPIOPort, chipId: Int) => {
    val ath = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty200THarness]
    // Chipyard punches one GPIOPort per pin, not one per controller, so this
    // binder fires once for each bit and port.pinId selects which pin it is.
    // The port is Analog because the sifive GPIO is bidirectional; passing it
    // straight through keeps the direction under the controller's control
    // rather than fixing it here.
    require(port.pinId < pins.size,
      s"WithArty200TGPIO has ${pins.size} pins but the shell elaborated GPIO bit ${port.pinId}")
    val harnessIO = IO(Analog(1.W)).suggestName(s"gpio_${port.pinId}")
    harnessIO <> port.io
    ath.xdc.addPackagePin(IOPin(harnessIO), pins(port.pinId))
    ath.xdc.addIOStandard(IOPin(harnessIO), "LVCMOS33")
  }
})

// The four ESC/motor drive outputs, taken from the two sifive PWM blocks.
//
// CHANNEL MAP. Zephyr's sifive driver uses comparator 0 of each block to set the shared period
// and refuses to drive it ("PWM channel 0 cannot be configured"), so a four-comparator block
// yields three usable outputs. Four motors therefore need two blocks, which is why
// WithRiskyBirdDronePeriphery instantiates pwm@10050000 and pwm@10051000. The (block, comparator)
// pairs below are the ones hardware/zephyr/targets/fpga/workloads/motor_4.overlay names.
//
// PINS. E22/B20/R17/F13 are J1_31, J1_36, J1_99 and J1_100 in the TE0712 board file
// (arty_files/te0712-rd/board_files/TE0712_200_2C/1.0/part0_pins.xml). J1_31 and J1_99 are
// odd-row pins; every signal proven on this bench so far -- JTAG, I2C, the PMW3901 SPI -- is on
// an even-row J1 pin, and the one dead signal, the console UART, is on J1_41/J1_43, both odd.
// That correlation is unexplained, so motors 1 and 3 carry more risk than 2 and 4, and a bench
// test that finds exactly those two dead is evidence about the ribbon rather than about PWM.
// See docs/drone-initial-bringup.md.
//
// POLARITY. The output is inverted here, deliberately, and the SoC is unusable for motors
// without it.
//
// The sifive comparator drives its output high when the scaled counter has passed the
// comparator value: gpio(i) = (pwms >= pwmcmp(i)). Zephyr writes the requested pulse width
// straight into pwmcmp(i), so the pin is high for (period - pulse), not for pulse -- a request
// for 10% duty comes out as 90%. Worse, both the reset state and Zephyr's own "stop" produce
// pwmcmp(i) = 0, and (pwms >= 0) is always true, so an uninverted pin sits at a constant logic
// high. On this board that is four motors at full throttle from the cycle after configuration
// releases, before a single instruction has run.
//
// Inverting at the harness fixes both at once, and fixes the dangerous one in hardware rather
// than in software that has to boot first:
//
//   reset / stop (pwmcmp = 0)   uninverted 100%  ->  inverted 0%     motors off
//   pwm_set_dt(period, pulse)   uninverted 1-D   ->  inverted D      matches Zephyr's API
//
// The RTL's own invert bits (pwmcfg[20+i], the `extra` field, which sifive-blocks names
// pwminvert) would do the same thing, but only once software has written them, which leaves the
// power-on window unprotected. Software must therefore leave those bits alone: setting them
// inverts a second time and puts the motors back at full throttle. Nothing in Zephyr's driver
// touches them -- it only writes bits 8, 9, 10, 12, 16-19 and 24-27.
//
// The pulldown covers the window before the design is live at all, when the FPGA fabric is
// unconfigured and the pin is high-Z.
class WithArty200TPWM(
  // (harness port name, PWM block, comparator, package pin)
  motors: Seq[(String, Int, Int, String)] = Seq(
    ("motor1", 0, 1, "E22"),   // J1_31
    ("motor2", 0, 2, "B20"),   // J1_36
    ("motor3", 0, 3, "R17"),   // J1_99
    ("motor4", 1, 1, "F13")),  // J1_100
  invert: Boolean = true) extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: PWMPort, chipId: Int) => {
    val ath = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty200THarness]
    motors.filter(_._2 == port.pwmId).foreach { case (name, _, cmp, pin) =>
      require(cmp > 0,
        s"$name is mapped to comparator 0 of PWM block ${port.pwmId}, which sets the period " +
        "and cannot drive an output")
      require(cmp < port.io.gpio.size,
        s"$name needs comparator $cmp but PWM block ${port.pwmId} elaborated only " +
        s"${port.io.gpio.size}")
      val harnessIO = IO(Output(Bool())).suggestName(name)
      harnessIO := (if (invert) ~port.io.gpio(cmp) else port.io.gpio(cmp))
      ath.xdc.addPackagePin(IOPin(harnessIO), pin)
      ath.xdc.addIOStandard(IOPin(harnessIO), "LVCMOS33")
      ath.xdc.addPulldown(IOPin(harnessIO))
    }
  }
})

// Brings the HM01B0 capture peripheral's parallel-video pins out to the xc7a200t-fbg484 shell.
// Pins are the TE0712 drone-carrier mapping. dataPins is ordered D0..D7, i.e. index 0 is the LSB.
//
// VOLTAGE, AND WHY IT IS NOT A SINGLE SWITCH. hardware/ospi/docs/README.md records the HM01B0
// module IOVDD as 1.8 V. `ioStandard` therefore exists so the camera group can be moved to
// LVCMOS18 if its bank VCCO is set to 1.8 V -- but the interface is NOT confined to one bank.
// Measured on xc7a100tfgg484 and confirmed identical on xc7a200tfbg484 -- all 484 balls
// match on bank, PIN_FUNC, clock-capability, VREF/VRP and diff-pair type
// (Vivado get_package_pins):
//
//   bank 15: D0-D7, PCLK, FVLD, LVLD, MCLK        <- 12 signals, movable to 1.8 V together
//   bank 14: TRIG, INT                            <- shares a bank with UART-TSI (P16/U18)
//
// Setting bank 14 to 1.8 V would take the UART-TSI loader pins with it and break the load path,
// so TRIG and INT cannot follow the rest of the camera group by a bank change alone. For
// bring-up neither is required: capture runs free-running, so TRIG is unused, and INT is only
// the motion interrupt. Leave them disconnected unless a translator is present.
class WithArty200TOspi(
  dataPins: Seq[String] = Seq("W16", "Y16", "AA16", "AA15", "AB15", "AB17", "AB16", "V13"),
  pclkPin: String = "W11",
  fvldPin: String = "W10",
  lvldPin: String = "T14",
  intrPin: String = "T15",
  mclkPin: String = "W15",
  trigPin: String = "V10",
  ioStandard: String = "LVCMOS33") extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: OspiPort, chipId: Int) => {
    val ath = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[Arty200THarness]
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
