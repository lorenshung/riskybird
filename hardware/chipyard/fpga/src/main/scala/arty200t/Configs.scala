// See LICENSE for license details.
package chipyard.fpga.arty200t

import org.chipsalliance.cde.config._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.devices.debug._
import freechips.rocketchip.devices.tilelink._
import org.chipsalliance.diplomacy._
import org.chipsalliance.diplomacy.lazymodule._
import freechips.rocketchip.system._
import freechips.rocketchip.tile._

import sifive.blocks.devices.uart._
import sifive.fpgashells.shell.{DesignKey}

import testchipip.serdes.{SerialTLKey}

import chipyard.{BuildSystem}

// don't use FPGAShell's DesignKey
class WithNoDesignKey extends Config((site, here, up) => {
  case DesignKey => (p: Parameters) => new SimpleLazyRawModule()(p)
})

/** The SOM's 1 GiB DDR3 behind the MIG, as one contiguous TL backing memory.
 *
 *  WithArty200TDDRTL belongs here rather than in the common tweaks: it matches
 *  on TLMemPort and dereferences the harness's DDR client, so it must not be in
 *  the binder list for a configuration whose harness places no MIG.
 */
class WithArty200TDDRMem extends Config(
  new WithArty200TDDRTL ++
  new chipyard.config.WithTLBackingMemory ++
  new freechips.rocketchip.subsystem.WithExtMemSize(BigInt(1) << 30))

/** A 32 KiB mbus scratchpad and no memory port, so the harness places no MIG.
 *
 *  The counterpart of WithKU040ScratchpadMem. Everything the SoC runs has to
 *  fit in those 32 KiB, which is what makes it useful: it is the same Rocket
 *  and the same periphery with the DDR controller and its ~10,500 LUT removed,
 *  so the pair isolates what main memory costs on this part.
 */
class WithArty200TScratchpadMem extends Config(
  new testchipip.soc.WithMbusScratchpad(base = 0x80000000L, size = (BigInt(1) << 15)) ++
  new freechips.rocketchip.subsystem.WithNoMemPort)

// `ddr` chooses whether there is a backing memory port at all; false is the
// 32 KiB mbus scratchpad, which has none.
//
// `uartTsi` chooses whether the SoC gets the serial program loader. It has to be
// a knob rather than an override because the client *creates* the UARTTSIPort:
// a later harness binder can repin that port but cannot remove it, and a carrier
// that wires only one UART has no pins to spare for a second one. false drops
// both the client and its binder, leaving JTAG as the only way in.
class WithArty200TTweaks(freqMHz: Double = 50, ddr: Boolean = true, uartTsi: Boolean = true) extends Config(
  new WithArty200TPMODUART ++
  (if (uartTsi) new WithArty200TUARTTSI else new Config(Parameters.empty)) ++
  new WithArty200TJTAG ++
  new WithNoDesignKey ++
  (if (uartTsi) new testchipip.tsi.WithUARTTSIClient else new Config(Parameters.empty)) ++
  new chipyard.harness.WithSerialTLTiedOff ++
  new chipyard.harness.WithHarnessBinderClockFreqMHz(freqMHz) ++
  new chipyard.config.WithUniformBusFrequencies(freqMHz) ++
  new chipyard.harness.WithAllClocksFromHarnessClockInstantiator ++
  new chipyard.clocking.WithPassthroughClockGenerator ++
  (if (ddr) new WithArty200TDDRMem else new WithArty200TScratchpadMem) ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors)

/** Opt-in HM01B0 capture, including the I2C controller used to configure the sensor. */
class WithArty200TOspiPeriphery extends Config(
  new WithArty200TI2C ++
  new WithArty200TOspi ++
  new chipyard.iobinders.WithOspiPunchthrough ++
  new ospi.WithOspiCapture ++
  new chipyard.config.WithI2C)

class RocketArty200TConfig extends Config(
  new WithArty200TTweaks ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** The FTDI carrier's six signal pins, all in bank 14 of the xc7a200t-fbg484.
 *
 *  Byte-identical to WithArty100TFtdiCarrierPins, and that is a verified claim
 *  rather than an assumption. The TE0712 ships the 100T and the 200T on the same
 *  484-ball module, and Trenz's own two board files --
 *  arty_files/te0712-rd/board_files/TE0712_100_2C/2.0/part0_pins.xml and
 *  .../TE0712_200_1I/2.0/part0_pins.xml -- are identical across all 183 declared
 *  nets: same name, same ball, same iostandard. So J2 lands on the same balls on
 *  both parts.
 *
 *  | Signal | Pin | SOM net | PIN_FUNC | Bank |
 *  | --- | --- | --- | --- | --- |
 *  | `jtag_TDI` | W20 | J2_24 | `IO_L12N_T1_MRCC_14` | 14 |
 *  | `jtag_TMS` | V20 | J2_28 | `IO_L11N_T1_SRCC_14` | 14 |
 *  | `jtag_TCK` | W19 | J2_22 | `IO_L12P_T1_MRCC_14` | 14 |
 *  | `jtag_TDO` | U20 | J2_26 | `IO_L11P_T1_SRCC_14` | 14 |
 *  | `uart_rxd` | V18 | J2_21 | `IO_L14P_T2_SRCC_14` | 14 |
 *  | `uart_txd` | Y18 | J2_25 | `IO_L13P_T2_MRCC_14` | 14 |
 *
 *  Beware when adding a camera config on this carrier: WithArty200TOspi still
 *  defaults INT to V20 and TRIG to U20, which collide with TMS and TDO above.
 */
class WithArty200TFtdiCarrierPins extends Config(
  new WithArty200TUART("V18", "Y18") ++
  new WithArty200TJTAG(tckPin = "W19", tmsPin = "V20", tdiPin = "W20", tdoPin = "U20"))

/** The DDR3 bring-up target for the FTDI carrier on the 200T module.
 *
 *  The 200T counterpart of RocketArty100TFtdiDDRConfig, and the one to build when
 *  the 100T module is unavailable. Everything that matters is shared, verified
 *  rather than assumed:
 *
 *   - **Pinout.** Identical; see WithArty200TFtdiCarrierPins above.
 *   - **Memory.** Also identical. The generated MIG projects for the two parts
 *     (`obj/ip/arty100tmig/.../mig.prj` and its 200T twin) differ in exactly two lines, the
 *     module name and `<TargetFPGA>`; all 77 pin-map lines and every timing
 *     parameter match. Both are one `MT41J256M16XX-125` pair at `<DataWidth>32`
 *     off a `<InputClkFreq>50` MHz input with a 2500 ps `<TimePeriod>`, giving the
 *     same 1 GiB at 0x8000_0000. The DDR3 is on the module, not the carrier, which
 *     is why swapping carriers moved only the six signal pins above.
 *   - **Clock.** Arty200TShell drives the same on-module 50 MHz LVDS pair on
 *     H4/G4 at DIFF_SSTL15.
 *
 *  The only real difference is headroom: the same design is ~74% of the
 *  xc7a100t's 63,400 LUT and ~35% of the xc7a200t's 134,600.
 *
 *  As on the 100T, no UART-TSI: the carrier exposes one RX/TX pair and it is the
 *  FTDI console, so programs go in over JTAG (`hardware/openocd/arty200t.cfg`).
 */
class RocketArty200TFtdiDDRConfig extends Config(
  new WithArty200TFtdiCarrierPins ++
  new WithArty200TTweaks(uartTsi = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** The DDR3 bring-up target for the TE0712 drone carrier.
 *
 *  The J1 counterpart of RocketArty200TFtdiDDRConfig, and the one to build for
 *  the bench in docs/drone-initial-bringup.md. Same design, same 1 GiB of
 *  on-module DDR3 behind the MIG; the only difference is that the six signal
 *  pins stay on WithArty200TTweaks' defaults -- JTAG on A19/F19/A18/F20 in bank
 *  16, the four balls the drone carrier actually lands -- instead of moving to
 *  the FTDI carrier's bank-14 block on J2. Loading the wrong one of the two puts
 *  JTAG on pins nothing is connected to and produces an all-ones scan that reads
 *  exactly like a broken wire.
 *
 *  This is the DDR3 half of a pair with RocketArty200TSpadConfig: identical
 *  Rocket, identical pinout, identical periphery, and main memory the only
 *  variable. That is what makes it a bring-up target rather than a product --
 *  the scratchpad build is known good on this bench, so anything that breaks
 *  here is the MIG.
 *
 *  No UART-TSI, for the same reason as the FTDI carrier: the console pair on
 *  this carrier is not wired (docs/drone-initial-bringup.md records the
 *  measurements), so `rb load` cannot work and programs go in over JTAG. Dropping
 *  the client also drops its second UART, ~843 LUT on the 100T, and removes a
 *  port that no pin on this bench answers on.
 *
 *  Reading output needs `--ram-console`, which retargets the console to a RAM
 *  ring buffer GDB reads over the same JTAG link that loaded the image.
 */
class RocketArty200TDroneDDRConfig extends Config(
  new WithArty200TTweaks(uartTsi = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** RocketArty200TSpadConfig with the console UART's two pins exchanged.
 *
 *  A bring-up experiment, not a target. On the drone bench the console pair is
 *  dead in both directions while JTAG on the same bank works, and a crossed
 *  ribbon explains every measurement at once: the FPGA drives D21 into the
 *  FTDI's own TX driver, so nothing reaches the FTDI's RX on the other wire,
 *  and E21 sits against that high-impedance RX input and -- having no pullup --
 *  floats low, which is the constant 0x00 the sifive UART reports.
 *
 *  So this swaps them: rxd on D21 (J1_43), txd on E21 (J1_41). If the ribbon is
 *  crossed this bitstream makes the console work; if the pair was never landed
 *  at all it changes nothing. Either outcome is decisive, which is why it is
 *  worth a bitstream.
 *
 *  See docs/drone-initial-bringup.md. Delete this once the wiring is settled.
 */
class WithArty200TPMODUARTSwapped extends WithArty200TUART("D21", "E21")

class RocketArty200TSpadSwapUartConfig extends Config(
  new WithArty200TPMODUARTSwapped ++
  new WithArty200TTweaks(ddr = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** RocketArty200TConfig without the DDR3: 32 KiB mbus scratchpad, no MIG.
 *
 *  The scratchpad half of the bring-up pair. No camera, no I2C -- just the
 *  huge Rocket, the two UARTs, and JTAG, which is the smallest design that
 *  still exercises the FTDI path.
 */
class RocketArty200TSpadConfig extends Config(
  new WithArty200TTweaks(ddr = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

class RocketArty200TOspiConfig extends Config(
  new WithArty200TOspiPeriphery ++
  new WithArty200TTweaks ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/**
 * Logical full-drone SoC: camera/I2C plus PMW3901 SPI/control GPIO and two PWM
 * blocks. SPI, GPIO, and motor package pins remain intentionally unbound until
 * an explicit Arty-to-base wiring definition is selected.
 */
/** RocketArty200TDroneLogicConfig with the PMW3901 SPI and GPIO routed to pins.
 *
 *  The DroneLogic config elaborates the SPI, GPIO and PWM controllers into the
 *  SoC but binds none of them to package pins -- the comment above says so, and
 *  the generated shell XDC confirms it: it constrains i2c, ospi, uart, jtag and
 *  the clock, and nothing else. So the optical-flow sensor cannot be reached on
 *  that bitstream no matter how the base board is wired.
 *
 *  This adds the missing routing for the flow sensor only. SPI and CS come from
 *  the measured bank survey in docs/hm01b0-camera-bringup.md:163; the GPIO reset
 *  and LED pins are placeholders, and WithArty200TGPIO explains why that is
 *  still enough to identify the sensor. Motors stay unbound, which is
 *  deliberate: nothing here should be able to spin a propeller.
 *
 *  Scratchpad rather than DDR3, so main memory stays on the path already proven
 *  on this bench and a failure is about SPI and nothing else.
 */
class RocketArty200TFlowSpadConfig extends Config(
  new WithArty200TSPI ++
  new WithArty200TGPIO ++
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithArty200TOspiPeriphery ++
  new WithArty200TTweaks(ddr = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** RocketArty200TFlowSpadConfig with the four motor PWM outputs routed to pins.
 *
 *  The first arty200t config whose PWM controllers can reach the outside world. Every earlier
 *  one -- DroneLogic included -- elaborates the two blocks and leaves them unbound, and the
 *  generated Verilog shows what that costs: with no consumer above the controller, firtool
 *  deletes the output path entirely, so TLPWM in those builds has no gpio port at all. The
 *  registers still read back correctly, which is exactly what makes it worth stating.
 *
 *  Periphery is otherwise identical to RocketArty200TFlowSpadConfig, and deliberately so: the
 *  PLIC source numbering that hardware/zephyr/fpga-drone.overlay hardcodes (i2c 1, pwm0 2-5,
 *  pwm1 6-9, uart 10, gpio 11-13, spi 14, ospi 15) depends on which controllers elaborate, and
 *  adding or dropping one silently shifts every source above it. That is the failure that cost
 *  this bench its console UART. Keeping the set fixed keeps the overlay valid.
 *
 *  Scratchpad rather than DDR3, for the same reason the flow-sensor config uses one: main
 *  memory stays on the path already proven here, so a failure is about PWM and nothing else.
 *
 *  WithArty200TPWM inverts the outputs. Read its comment before changing anything about the
 *  polarity -- an uninverted build drives all four motors at full throttle from power-on.
 */
class RocketArty200TMotorSpadConfig extends Config(
  new WithArty200TPWM ++
  new chipyard.iobinders.WithPWMPunchthrough ++
  new WithArty200TSPI ++
  new WithArty200TGPIO ++
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithArty200TOspiPeriphery ++
  new WithArty200TTweaks(ddr = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

class RocketArty200TDroneLogicConfig extends Config(
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithArty200TOspiPeriphery ++
  new WithArty200TTweaks ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** The baseline OSPI SoC with its 1 GiB of DDR3 replaced by a 32 KiB scratchpad.
 *
 *  Same huge Rocket, same camera/I2C periphery, same 50 MHz. The variant that
 *  shows what is left once the MIG is gone, and the one to run when a workload
 *  fits in 32 KiB and DDR3 timing closure is not wanted in the loop. Neither
 *  variant is close to tight on the xc7a200t's 134,600 LUT; the corresponding
 *  xc7a100t builds measured 38,793 (scratchpad) and 49,228 (DDR).
 */
class RocketArty200TOspiSpadConfig extends Config(
  new WithArty200TOspiPeriphery ++
  new WithArty200TTweaks(ddr = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** Dual full-size Rocket.
 *
 *  This is where the larger part earns its place. The xc7a100t forced a
 *  compromise: its dual-core config had to use WithNSmallCores(1) -- no FPU,
 *  no BTB, no virtual memory, both L1s cut to 64 sets by 1 way -- because the
 *  baseline already placed at 49,228 of 63,400 LUT while a huge tile costs
 *  28,813. Even so it landed at 88%.
 *
 *  On the xc7a200t's 134,600 LUT two huge tiles project to roughly 78,000
 *  (~58%), so the second core can be a real one: FPU, MMU, full caches, and
 *  the same ISA string as tile 0. That projection is scaled from the xc7a100t
 *  measurements and has not been measured on this part yet.
 */
class DualRocketArty200TOspiConfig extends Config(
  new WithArty200TOspiPeriphery ++
  new WithArty200TTweaks ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.DualRocketConfig)

/** Camera periphery sized for bring-up rather than streaming.
 *
 *  Identical to WithArty200TOspiPeriphery except for the capture buffer: the streaming default
 *  holds a whole 324x324 frame (104,977 beats, block RAM), which is far more than a bring-up
 *  capture of one QVGA line needs. `captureBeats` defaults to 1024, comfortably above the 324
 *  pixels plus EOF marker of a single line.
 */
class WithArty200TCameraBringupPeriphery(captureBeats: Int = 1024) extends Config(
  new WithArty200TI2C ++
  new WithArty200TOspi ++
  new chipyard.iobinders.WithOspiPunchthrough ++
  new ospi.WithOspiCapture(frameBufferDepth = captureBeats) ++
  new chipyard.config.WithI2C)

/** Minimal HM01B0 camera bring-up target.
 *
 *  The narrowest SoC that can answer the two bring-up questions: can the FPGA configure the
 *  physical sensor over I2C, and does the Chisel capture path receive real pixel data.
 *
 *  Contents, and nothing else: one Rocket, the 32 KiB mbus scratchpad (no DDR3, no MIG), the
 *  console UART, UART-TSI for loading, JTAG, the I2C master that configures the sensor, and the
 *  HM01B0 capture peripheral with a one-line capture buffer.
 *
 *  Named with the board, unlike the arty100t original, so that passing a config
 *  class name to `rb --config` can never select the wrong board's camera target.
 *
 *  DELIBERATE DEVIATION from "one small Rocket": this keeps chipyard.RocketConfig's single
 *  WithNHugeCores(1) rather than WithNSmallCores(1). A small core drops the FPU and virtual
 *  memory, which changes the DTS ISA string and invalidates the Zephyr board contract that
 *  `rb` validates and that every working image on this board was built against. Deriving from
 *  the known-good target matters more here than shaving a core, and area is not the constraint:
 *  the scratchpad build placed at 59% LUT on the far smaller xc7a100t.
 */
class CameraBringup200TConfig extends Config(
  new WithArty200TCameraBringupPeriphery ++
  new WithArty200TTweaks(ddr = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

class NoCoresArty200TConfig extends Config(
  new WithArty200TTweaks ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.NoCoresConfig)

/** Full drone SoC on the TE0712 drone carrier: DDR3 + I2C + SPI + PWM(x2) + GPIO + camera OSPI,
 *  plus a SECOND peripheral UART for the ESP32 link (FPGA rxd=E13 <- ESP TX ; txd=F14 -> ESP RX).
 *  = RocketArty200TMotorSpadConfig periphery but with DDR3 (ddr=true) and the added ESP UART. */
class RocketArty200TDroneFullDDRConfig extends Config(
  new WithArty200TUART("E13", "F14", uartNo = 1) ++       // ESP UART = uart1 on E13/F14
  new chipyard.config.WithUART(address = 0x10021000) ++   // add uart1 (console uart0 stays 0x10020000)
  new WithArty200TPWM ++
  new chipyard.iobinders.WithPWMPunchthrough ++
  new WithArty200TSPI ++
  new WithArty200TGPIO ++
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithArty200TOspiPeriphery ++
  new WithArty200TTweaks(ddr = true, uartTsi = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** RocketArty200TDroneFullDDRConfig plus remote program loading through the ESP32.
 *
 *  The same SoC, with one change at the ESP pins: instead of uart1 owning E13/F14
 *  outright, WithArty200TEspLinkLoader shares them between uart1 and a UART-TSI
 *  client, switched by hardware on a break + key from the ESP (see
 *  EspLinkLoader.scala). In normal operation uart1 behaves exactly as in the
 *  FullDDR shell; on request the ESP can reset the SoC and write a new ELF into
 *  DDR with `rb load`, so programs can go in over WiFi instead of over JTAG.
 *
 *  uartTsi = false on the tweaks keeps the Arty-pinned P16/U18 TSI binder out;
 *  the client is added here instead and bound to the shared pins. The TSI UART
 *  is not a TLUART, so the memory map, PLIC sources and Zephyr overlays are
 *  those of RocketArty200TDroneFullDDRConfig.
 *
 *  The loader's reset is a SoC-only reset, which the FullDDR periphery was not built
 *  for: the PWM comparators keep their values across it (so the harness inhibits the
 *  motor pins until each PWM output reads "off" again), and the DDR adapters keep
 *  their state (so the harness fences the memory port at a burst boundary first).
 */
class RocketArty200TDroneFullDDRLoaderConfig extends Config(
  // ESP pins: uart1 + UART-TSI. Also binds the motor pins (WithArty200TEspLinkPWM,
  // WithArty200TPWM's mapping plus an inhibit armed by the loader's reset) and the
  // DDR port (fenced), so WithArty200TPWM is deliberately absent here.
  new WithArty200TEspLinkLoader("E13", "F14", uartNo = 1) ++
  new testchipip.tsi.WithUARTTSIClient ++
  new chipyard.config.WithUART(address = 0x10021000) ++   // add uart1 (console uart0 stays 0x10020000)
  new chipyard.iobinders.WithPWMPunchthrough ++
  new WithArty200TSPI ++
  new WithArty200TGPIO ++
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithArty200TOspiPeriphery ++
  new WithArty200TTweaks(ddr = true, uartTsi = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** The ESP loader on the motor-free drone shell: RocketArty200TDroneDDRConfig plus uart1 and
 *  WithArty200TEspLinkLoader on E13/F14.
 *
 *  For proving the loader on the bench without a motor-capable image. Like DroneDDR it has no
 *  PWM blocks and constrains none of the four motor pins (E22/B20/R17/F13), so they are left
 *  to the bitstream's unused-pin pulldown -- whereas loading the FullDDR shell has spun motors
 *  before (2026-09-04, cause unknown). Same Rocket, DDR3 and JTAG as DroneDDR, so programs
 *  built for RocketArty200TDroneDDRConfig run here; uart1 at 0x10021000 is the only addition.
 */
class RocketArty200TDroneDDRLoaderConfig extends Config(
  new WithArty200TEspLinkLoader("E13", "F14", uartNo = 1) ++  // ESP pins: uart1 + UART-TSI
  new testchipip.tsi.WithUARTTSIClient ++
  new chipyard.config.WithUART(address = 0x10021000) ++   // add uart1 (console uart0 stays 0x10020000)
  new WithArty200TTweaks(uartTsi = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** OSPI periphery with the DMA-capable capture peripheral (adds a TileLink master to DDR).
 *  Byte-for-byte mirror of WithArty200TOspiPeriphery except it swaps WithOspiCapture for
 *  WithOspiCaptureDma, which flips OspiParams.enableDma. */
class WithArty200TOspiDmaPeriphery extends Config(
  new WithArty200TI2C ++
  new WithArty200TOspi ++
  new chipyard.iobinders.WithOspiPunchthrough ++
  new ospi.WithOspiCaptureDma ++
  new chipyard.config.WithI2C)

/** RocketArty200TDroneFullDDRConfig, but with the DMA-capable camera OSPI peripheral: captured
 *  frames are DMA'd straight to a software-configured DDR buffer by hardware (TileLink master),
 *  instead of the CPU polling the DATA drain register. Identical DDR3 + I2C + SPI + PWM(x2) + GPIO
 *  + ESP UART (E13/F14) set; only the OSPI periphery differs. */
class RocketArty200TDroneFullDDRDmaConfig extends Config(
  new WithArty200TUART("E13", "F14", uartNo = 1) ++       // ESP UART = uart1 on E13/F14
  new chipyard.config.WithUART(address = 0x10021000) ++   // add uart1 (console uart0 stays 0x10020000)
  new WithArty200TPWM ++
  new chipyard.iobinders.WithPWMPunchthrough ++
  new WithArty200TSPI ++
  new WithArty200TGPIO ++
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithArty200TOspiDmaPeriphery ++
  new WithArty200TTweaks(ddr = true, uartTsi = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** RocketArty200TDroneFullDDRDmaConfig plus a memory->UART TX DMA on the ESP link (uart1).
 *
 *  Byte-for-byte the same SoC as RocketArty200TDroneFullDDRDmaConfig -- same DDR3 + I2C + SPI +
 *  PWM(x2) + GPIO + camera-OSPI-DMA + ESP UART on E13/F14, same pin constraints -- with ONE addition:
 *  the uartdma.WithUartTxDma engine. It lets software hand a DDR buffer to hardware and have it
 *  streamed out uart1 (0x1002_1000) with hardware flow control on the TX-FIFO-full bit, instead of
 *  the CPU spinning per byte. The DMA drives the *existing* uart1 over the bus fabric, so it adds no
 *  chip pins and no interrupt source (the PLIC source numbering the Zephyr overlay hardcodes is
 *  unchanged). Its own control/status registers live in a dedicated MMIO page at 0x1002_2000 (the
 *  page just above uart1's window); DMA_* offsets mirror the OSPI capture DMA (0x44 ADDR_LO,
 *  0x48 ADDR_HI, 0x4c LEN, 0x50 CTRL, 0x54 STATUS, 0x58 BYTES).
 *
 *  RocketArty200TDroneFullDDRDmaConfig stays as the known-good fallback (this fragment is additive).
 */
class RocketArty200TDroneFullDDRDmaUartConfig extends Config(
  new uartdma.WithUartTxDma(address = 0x10022000L, uartTxDataAddr = 0x10021000L) ++  // mem->uart1 TX DMA
  new WithArty200TUART("E13", "F14", uartNo = 1) ++       // ESP UART = uart1 on E13/F14
  new chipyard.config.WithUART(address = 0x10021000) ++   // add uart1 (console uart0 stays 0x10020000)
  new WithArty200TPWM ++
  new chipyard.iobinders.WithPWMPunchthrough ++
  new WithArty200TSPI ++
  new WithArty200TGPIO ++
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithArty200TOspiDmaPeriphery ++
  new WithArty200TTweaks(ddr = true, uartTsi = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** PMW3901 optical-flow bring-up target: RocketArty200TDroneFullDDRDmaConfig, named explicitly.
 *
 *  Functionally identical to RocketArty200TDroneFullDDRDmaConfig. That config ALREADY
 *  instantiates the flow SPI (WithArty200TSPI: sck=F16, mosi=E17, miso=E16 with a weak MISO
 *  pulldown) and the control GPIO (WithArty200TGPIO: bit0=CS=D16, bit1=reset=A20, bit2=LED=A21);
 *  the generated shell XDC confirms all six ports are constrained. This alias exists only so the
 *  optical-flow target is greppable by name and can be built/flashed on its own.
 *
 *  PIN DERIVATION -- riskybirdv3.kicad_sch netlist (PMW3901 = IC1) x the Trenz "Riskybird B2B
 *  Pinout" sheet, cross-checked three ways: the camera DVP derivation (PCLK=W11 etc., 14/14),
 *  the RISC-V console UART (JB1 pads 42/44 -> E21/D21), and the ESP UART1 (JB1 pads 95/97 ->
 *  E13/F14). Each PMW3901 SPI net leaves IC1 through a series resistor to the FPGA side of a
 *  dual-master pad pair on the JB1 (LSHM-150, 2x50) B2B connector:
 *
 *    SCLK  IC1.17 -> R34 -> JB1 pad 81 (module J1_82) -> F16   bank 64   sifive spi@10031000 SCK
 *    MOSI  IC1.16 -> R29 -> JB1 pad 79 (module J1_80) -> E17   bank 64   spi dq(0)
 *    MISO  IC1.18 -> R28 -> JB1 pad 77 (module J1_78) -> E16   bank 64   spi dq(1)
 *    NCS   IC1.19 -> R27 -> JB1 pad 75 (module J1_76) -> D16   bank 64   gpio@10010000 bit 0 (SW CS)
 *
 *  base pad N <-> module pin N+1 is the same hermaphroditic B2B swap the camera derivation used
 *  (base JB3.57 <-> module JM3.58 -> W11). SCLK's copper is on physical pad 81 (the schematic
 *  labels that pin "Pin_61", but pad 61 itself is unconnected), so F16 -- not B15 -- is correct.
 *
 *  The hard SPI chip-select is deliberately left unbound; the driver holds CS low across a
 *  multi-byte register transaction, which the sifive controller's automatic CS does not, so CS
 *  is software-driven from gpio0 bit 0 (D16). flow.c must select gpio0 bit 0 for CS.
 */
class RocketArty200TDroneFullDDRDmaFlowConfig extends Config(
  new WithArty200TUART("E13", "F14", uartNo = 1) ++       // ESP UART = uart1 on E13/F14
  new chipyard.config.WithUART(address = 0x10021000) ++   // add uart1 (console uart0 stays 0x10020000)
  new WithArty200TPWM ++
  new chipyard.iobinders.WithPWMPunchthrough ++
  new WithArty200TSPI ++                                  // PMW3901 flow SPI: sck=F16 mosi=E17 miso=E16
  new WithArty200TGPIO ++                                 // gpio0 bit0=CS=D16, bit1=A20, bit2=A21
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithArty200TOspiDmaPeriphery ++
  new WithArty200TTweaks(ddr = true, uartTsi = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)
