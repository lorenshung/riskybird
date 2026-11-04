package chipyard

import chisel3._

import org.chipsalliance.cde.config.{Config, Parameters}

// Chipyard's DigitalTop plus the two RiskyBird peripheries that need a
// subsystem trait: the HM01B0 camera capture (ospi) and the memory -> UART TX
// DMA engine (uartdma). Both traits do nothing unless their key is set.
//
// The RiskyBird Chipyard fork mixed these two traits into DigitalTop itself.
// The plain Chipyard pin cannot, so the fragments that enable either periphery
// (ospi.WithOspiCapture*, uartdma.WithUartTxDma) select this system instead
// (WithRbDigitalTop).
//
// The trait list is DigitalTop's, in DigitalTop's order, with the two inserted
// where the fork had them (after CanHavePeripheryGCD). Order is not cosmetic:
// trait bodies run in linearization order, which fixes the order diplomacy
// creates nodes and so the emitted module numbering, port order and devicetree
// labels. Extending DigitalTop instead (`extends DigitalTop with ...`) puts
// both traits last and elaborates equivalent but not identical Verilog.
// tests/test_chipyard_overlay.py keeps this list in step with the pinned
// Chipyard's DigitalTop. desiredName keeps the module named DigitalTop.
class RbDigitalTop(implicit p: Parameters) extends ChipyardSystem
  with testchipip.tsi.CanHavePeripheryUARTTSI
  with testchipip.boot.CanHavePeripheryCustomBootPin
  with testchipip.cosim.CanHaveTraceIO
  with testchipip.soc.CanHaveSubsystemInjectors
  with testchipip.soc.CanHaveSwitchableOffchipBus
  with testchipip.iceblk.CanHavePeripheryBlockDevice
  with testchipip.serdes.CanHavePeripheryTLSerial
  with testchipip.serdes.old.CanHavePeripheryTLSerial
  with testchipip.soc.CanHavePeripheryChipIdPin
  with sifive.blocks.devices.i2c.HasPeripheryI2C
  with sifive.blocks.devices.timer.HasPeripheryTimer
  with sifive.blocks.devices.pwm.HasPeripheryPWM
  with sifive.blocks.devices.uart.HasPeripheryUART
  with sifive.blocks.devices.gpio.HasPeripheryGPIO
  with sifive.blocks.devices.spi.HasPeripherySPIFlash
  with sifive.blocks.devices.spi.HasPeripherySPI
  with icenet.CanHavePeripheryIceNIC
  with chipyard.example.CanHavePeripheryGCD
  with ospi.CanHavePeripheryOspi // RiskyBird: HM01B0 camera capture
  with uartdma.CanHavePeripheryUartTxDma // RiskyBird: memory -> UART TX DMA
  with chipyard.clocking.HasChipyardPRCI
  with chipyard.clocking.CanHaveClockTap
  with constellation.soc.CanHaveGlobalNoC
  with rerocc.CanHaveReRoCCTiles
  with testchipip.ctc.CanHavePeripheryCTC
{
  override lazy val desiredName = "DigitalTop"
  override lazy val module = new RbDigitalTopModule(this)
}

class RbDigitalTopModule(l: RbDigitalTop) extends ChipyardSystemModule(l)
  with freechips.rocketchip.util.DontTouch

class WithRbDigitalTop extends Config((site, here, up) => {
  case BuildSystem => (p: Parameters) => new RbDigitalTop()(p)
})
