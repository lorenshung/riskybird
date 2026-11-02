package chipyard.fpga.ku040

import chisel3._
import chisel3.experimental.{Analog}

import freechips.rocketchip.jtag.{JTAGIO}
import freechips.rocketchip.subsystem.{PeripheryBusKey}
import freechips.rocketchip.tilelink.{TLBundle}
import freechips.rocketchip.diplomacy.{LazyRawModuleImp}
import org.chipsalliance.diplomacy.nodes.{HeterogeneousBag}
import sifive.blocks.devices.uart.{UART, UARTPortIO, UARTParams}
import sifive.fpgashells.shell._
import sifive.fpgashells.ip.xilinx._
import sifive.fpgashells.shell.xilinx._
import sifive.fpgashells.clocks._
import chipyard._
import chipyard.harness._
import chipyard.iobinders._
import testchipip.serdes._

// Joins the SoC's TL memory port to the pair of DDR4 controllers in the harness.
class WithKU040DDRTL extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: TLMemPort, chipId: Int) => {
    val kth = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[KU040Harness]
    val bundles = kth.ddrClient.get._1.out.map(_._1)
    val ddrClientBundle = Wire(new HeterogeneousBag(bundles.map(_.cloneType)))
    bundles.zip(ddrClientBundle).foreach { case (bundle, io) => bundle <> io }
    ddrClientBundle <> port.io
  }
})

// All pins below sit in bank 68, an HP bank (1.8V max, LVCMOS33 illegal).
// The header must be level-shifted on board if it is 3.3V at the connector,
// as on the Alinx KU040 FMC pins.
//
// UART-TSI is parked on spare bank-68 pins A4/B4 (unconnected for now); the
// PMOD UART pins D3/D4 carry the sifive UART (WithKU040UART below). OSPI configs move RX to C3
// so the P-side clock-capable D3 pin can carry the sensor PCLK.
class WithKU040UARTTSI(rxdPin: String = "A4", txdPin: String = "B4") extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: UARTTSIPort, chipId: Int) => {
    val kth = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[KU040Harness]
    val harnessIO = IO(new UARTPortIO(port.io.uartParams)).suggestName("uart_tsi")
    harnessIO <> port.io.uart
    val packagePinsWithPackageIOs = Seq(
      (rxdPin, IOPin(harnessIO.rxd)),
      (txdPin, IOPin(harnessIO.txd)))
    packagePinsWithPackageIOs foreach { case (pin, io) => {
      kth.xdc.addPackagePin(io, pin)
      kth.xdc.addIOStandard(io, "LVCMOS18")
      kth.xdc.addIOB(io)
    } }
  }
})

// Maps the sifive UART device to the PMOD UART pins (PMOD2):
//   RX (FPGA in)  - D3
//   TX (FPGA out) - D4
//
// `uartNo` selects which UARTPort this binder answers for, so a second UART device (e.g. an
// ESP telemetry link added as uart1) does not collide with this one on the same D3/D4 pins --
// mirrors WithArty200TUART(uartNo) in fpga/src/main/scala/arty200t/HarnessBinders.scala.
class WithKU040UART(rxdPin: String = "D3", txdPin: String = "D4", uartNo: Int = 0,
                    ioStandard: String = "LVCMOS18") extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: UARTPort, chipId: Int) if port.uartNo == uartNo => {
    val kth = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[KU040Harness]
    val harnessIO = IO(chiselTypeOf(port.io)).suggestName(s"uart${port.uartNo}")
    harnessIO <> port.io
    val packagePinsWithPackageIOs = Seq(
      (rxdPin, IOPin(harnessIO.rxd)),
      (txdPin, IOPin(harnessIO.txd)))
    packagePinsWithPackageIOs foreach { case (pin, io) => {
      kth.xdc.addPackagePin(io, pin)
      kth.xdc.addIOStandard(io, ioStandard)
      kth.xdc.addIOB(io)
    } }
  }
})

// Ties off a UART device that has no package-pin mapping yet (e.g. a second UART added for
// ESP telemetry, ahead of a verified carrier pinout). Without this, an unclaimed UARTPort falls
// through to AbstractConfig's default `WithUARTAdapter` HarnessBinder, which instantiates a
// simulation-only DPI UART model -- not synthesizable for a real FPGA build. Hold RX at
// idle-high: DontCare lowers to zero with the supported firtool, asserting a permanent break.
class WithKU040UARTTiedOff(uartNo: Int) extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: UARTPort, chipId: Int) if port.uartNo == uartNo => {
    UART.tieoff(port.io)
  }
})

// Pin assignments (PMOD1):
//   TCK - E5, TMS - C6, TDI - D5, TDO - D6
// `pullup` defaults to false because these four pins are LVCMOS18 in HP bank 68,
// so any real adapter reaches them through a level translator. A TXB0108 holds
// its output with ~4 kOhm one-shot drivers and TI requires external pulls to be
// >50 kOhm; an UltraScale internal PULLUP is ~10-25 kOhm and contends with it.
// Measured on hardware: with PULLUP the JTAG IR capture is non-deterministic
// (0x17, 0x0003, 0x1b across identical runs), without it the same bitstream is
// bit-exact repeatable. Set it true only for a directly-wired 1.8 V probe.
class WithKU040JTAG(tckPin: String = "E5", tmsPin: String = "C6",
                    tdiPin: String = "D5", tdoPin: String = "D6",
                    pullup: Boolean = false, ioStandard: String = "LVCMOS18") extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: JTAGPort, chipId: Int) => {
    val kth = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[KU040Harness]
    val harnessIO = IO(new JTAGChipIO(false)).suggestName("jtag")
    harnessIO.TDO := port.io.TDO
    port.io.TCK := harnessIO.TCK
    port.io.TDI := harnessIO.TDI
    port.io.TMS := harnessIO.TMS
    port.io.reset.foreach(_ := th.referenceReset)

    kth.sdc.addClock("JTCK", IOPin(harnessIO.TCK), 10)
    kth.sdc.addGroup(clocks = Seq("JTCK"))
    // E5 is clock-capable (IO_L13P_GC_QBC_68); kept anyway to match the
    // working Trenz arty100t binder
    kth.xdc.clockDedicatedRouteFalse(IOPin(harnessIO.TCK))
    val packagePinsWithPackageIOs = Seq(
      (tckPin, IOPin(harnessIO.TCK)),
      (tmsPin, IOPin(harnessIO.TMS)),
      (tdiPin, IOPin(harnessIO.TDI)),
      (tdoPin, IOPin(harnessIO.TDO))
    )

    packagePinsWithPackageIOs foreach { case (pin, io) => {
      kth.xdc.addPackagePin(io, pin)
      kth.xdc.addIOStandard(io, ioStandard)
      if (pullup) kth.xdc.addPullup(io)
    } }
  }
})

// Open-drain I2C bus used to configure the HM01B0 (default sensor address 0x24). The weak internal
// pull-ups aid bring-up; use external pull-ups sized for the board capacitance in hardware.
// `ioStandard` defaults to LVCMOS18 (Loren's bank-68 A1/A2 header); the riskybird v3 carrier wires
// this bus (IMU/ToF/baro/ADS7128 + the HM01B0 SCCB via a PCA9306 level-shifter onto the same net)
// to Bank 64 (HR) balls AA12/AB12, so pass "LVCMOS33" there.
class WithKU040I2C(sclPin: String = "A1", sdaPin: String = "A2",
                   ioStandard: String = "LVCMOS18") extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: I2CPort, chipId: Int) => {
    val kth = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[KU040Harness]
    val harnessIO = IO(new ShellI2CPortIO).suggestName("i2c")
    UIntToAnalog(port.io.scl.out, harnessIO.scl, port.io.scl.oe)
    UIntToAnalog(port.io.sda.out, harnessIO.sda, port.io.sda.oe)
    port.io.scl.in := AnalogToUInt(harnessIO.scl).asBool
    port.io.sda.in := AnalogToUInt(harnessIO.sda).asBool
    Seq((sclPin, IOPin(harnessIO.scl)), (sdaPin, IOPin(harnessIO.sda))).foreach { case (pin, io) =>
      kth.xdc.addPackagePin(io, pin)
      kth.xdc.addIOStandard(io, ioStandard)
      kth.xdc.addPullup(io)
    }
  }
})

// PMW3901 optical-flow SPI, mirror of WithArty200TSPI. MOSI-only master (dq0 out, dq1 in = MISO);
// a weak pulldown on MISO makes an absent sensor read 0x00 rather than a floating 0xFF. The
// riskybird v3 carrier wires this to Bank 64 (HR) balls (SCK=AC9, MOSI=AD9, MISO=AH14, LVCMOS33).
// The chip-select is a GPIO (software CS via WithKU040GPIO bit 0), matching flow.c.
class WithKU040SPI(
  sckPin: String = "AC9",
  mosiPin: String = "AD9",
  misoPin: String = "AH14",
  ioStandard: String = "LVCMOS33",
  misoPulldown: Boolean = true) extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: SPIPort, chipId: Int) => {
    val kth = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[KU040Harness]
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
        kth.xdc.addPackagePin(io, pin)
        kth.xdc.addIOStandard(io, ioStandard)
      }
    }
    if (misoPulldown) kth.xdc.addPulldown(IOPin(miso))
  }
})

// The DroneLogic GPIO block, mirror of WithArty200TGPIO: three bits ordered CS/reset/LED to match
// the pmw3901 overlay. Bit 0 is the PMW3901 NCS (riskybird carrier ball AH13, Bank 64). Bits 1/2
// (reset, LED_N) are carrier placeholders on nets this rev leaves unconnected (JB1.37/JB1.58) --
// same as the TE0712 -- but the GPIO block is three bits wide so the pins must be assigned.
class WithKU040GPIO(
  pins: Seq[String] = Seq("AH13", "AB10", "AG10"),
  ioStandard: String = "LVCMOS33") extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: GPIOPort, chipId: Int) => {
    val kth = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[KU040Harness]
    require(port.pinId < pins.size,
      s"WithKU040GPIO has ${pins.size} pins but the shell elaborated GPIO bit ${port.pinId}")
    val harnessIO = IO(Analog(1.W)).suggestName(s"gpio_${port.pinId}")
    harnessIO <> port.io
    kth.xdc.addPackagePin(IOPin(harnessIO), pins(port.pinId))
    kth.xdc.addIOStandard(IOPin(harnessIO), ioStandard)
  }
})

// The four ESC/motor PWM outputs, mirror of WithArty200TPWM. Four motors span the two sifive PWM
// blocks (comparator 0 of each sets the period and can't drive an output), so the (block,comparator)
// map is (0,1)(0,2)(0,3)(1,1) -- identical to the arty200t/Zephyr contract. Riskybird v3 carrier
// balls: motor1=AH8, motor2=AB9 (Bank64), motor3=AF23 (Bank65, HR 3.3V), motor4=AB15 (Bank64).
// The output is INVERTED (and pulled down) so pwmcmp=0 (reset/stop) => 0% duty, i.e. motors OFF at
// power-on -- without the invert an unconfigured pin sits high = full throttle. Software must NOT
// also set the RTL pwminvert bits (that double-inverts back to full throttle).
class WithKU040PWM(
  motors: Seq[(String, Int, Int, String)] = Seq(
    ("motor1", 0, 1, "AH8"),    // JB1.32
    ("motor2", 0, 2, "AB9"),    // JB1.35
    ("motor3", 0, 3, "AF23"),   // JB1.100, Bank 65 (HR, fixed 3.3V)
    ("motor4", 1, 1, "AB15")),  // JB1.99
  ioStandard: String = "LVCMOS33",
  invert: Boolean = true) extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: PWMPort, chipId: Int) => {
    val kth = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[KU040Harness]
    motors.filter(_._2 == port.pwmId).foreach { case (name, _, cmp, pin) =>
      require(cmp > 0,
        s"$name is mapped to comparator 0 of PWM block ${port.pwmId}, which sets the period " +
        "and cannot drive an output")
      require(cmp < port.io.gpio.size,
        s"$name needs comparator $cmp but PWM block ${port.pwmId} elaborated only " +
        s"${port.io.gpio.size}")
      val harnessIO = IO(Output(Bool())).suggestName(name)
      harnessIO := (if (invert) ~port.io.gpio(cmp) else port.io.gpio(cmp))
      kth.xdc.addPackagePin(IOPin(harnessIO), pin)
      kth.xdc.addIOStandard(IOPin(harnessIO), ioStandard)
      kth.xdc.addPulldown(IOPin(harnessIO))
    }
  }
})

// Default allocation uses otherwise-unused bank-68 pins; D3 is a P-side global-clock-capable input.
// All pins are valid for xcku040-sfva784 and use 1.8 V I/O, but the mapping must still be matched to
// the carrier/header schematic before connecting a camera.
class WithKU040Ospi(
  dataPins: Seq[String] = Seq("H3", "H4", "G2", "H2", "E2", "F2", "D1", "E1"),
  pclkPin: String = "D3",
  fvldPin: String = "E3",
  lvldPin: String = "F3",
  intrPin: String = "F4",
  mclkPin: String = "G4",
  trigPin: String = "B1") extends HarnessBinder({
  case (th: HasHarnessInstantiators, port: OspiPort, chipId: Int) => {
    val kth = th.asInstanceOf[LazyRawModuleImp].wrapper.asInstanceOf[KU040Harness]
    require(dataPins.size == 8, "HM01B0 8-bit mode requires exactly eight data pins")

    val harnessIO = IO(chiselTypeOf(port.io)).suggestName("ospi")
    harnessIO <> port.io

    dataPins.zipWithIndex.foreach { case (pin, i) =>
      val io = IOPin(harnessIO.d, i)
      kth.xdc.addPackagePin(io, pin)
      kth.xdc.addIOStandard(io, "LVCMOS18")
    }

    val pclkIO = IOPin(harnessIO.pclk)
    kth.xdc.addPackagePin(pclkIO, pclkPin)
    kth.xdc.addIOStandard(pclkIO, "LVCMOS18")

    Seq(
      (fvldPin, IOPin(harnessIO.fvld)),
      (lvldPin, IOPin(harnessIO.lvld)),
      (intrPin, IOPin(harnessIO.intr)),
      (mclkPin, IOPin(harnessIO.mclk)),
      (trigPin, IOPin(harnessIO.trig))).foreach { case (pin, io) =>
      kth.xdc.addPackagePin(io, pin)
      kth.xdc.addIOStandard(io, "LVCMOS18")
    }

    kth.sdc.addClock("ospi_pclk", pclkIO, 36)
    kth.sdc.addGroup(clocks = Seq("ospi_pclk"))
  }
})
