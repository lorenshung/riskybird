// See LICENSE for license details.
package chipyard.fpga.arty100t

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
 *  WithArty100TDDRTL belongs here rather than in the common tweaks: it matches
 *  on TLMemPort and dereferences the harness's DDR client, so it must not be in
 *  the binder list for a configuration whose harness places no MIG.
 */
class WithArty100TDDRMem extends Config(
  new WithArty100TDDRTL ++
  new chipyard.config.WithTLBackingMemory ++
  new freechips.rocketchip.subsystem.WithExtMemSize(BigInt(1) << 30))

/** A 32 KiB mbus scratchpad and no memory port, so the harness places no MIG.
 *
 *  The counterpart of WithKU040ScratchpadMem. Everything the SoC runs has to
 *  fit in those 32 KiB, which is what makes it useful: it is the same Rocket
 *  and the same periphery with the DDR controller and its ~10,500 LUT removed,
 *  so the pair isolates what main memory costs on this part.
 */
class WithArty100TScratchpadMem extends Config(
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
class WithArty100TTweaks(freqMHz: Double = 50, ddr: Boolean = true, uartTsi: Boolean = true) extends Config(
  new WithArty100TPMODUART ++
  (if (uartTsi) new WithArty100TUARTTSI else new Config(Parameters.empty)) ++
  new WithArty100TJTAG ++
  new WithNoDesignKey ++
  (if (uartTsi) new testchipip.tsi.WithUARTTSIClient else new Config(Parameters.empty)) ++
  new chipyard.harness.WithSerialTLTiedOff ++
  new chipyard.harness.WithHarnessBinderClockFreqMHz(freqMHz) ++
  new chipyard.config.WithUniformBusFrequencies(freqMHz) ++
  new chipyard.harness.WithAllClocksFromHarnessClockInstantiator ++
  new chipyard.clocking.WithPassthroughClockGenerator ++
  (if (ddr) new WithArty100TDDRMem else new WithArty100TScratchpadMem) ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors)

/** Opt-in HM01B0 capture, including the I2C controller used to configure the sensor. */
class WithArty100TOspiPeriphery extends Config(
  new WithArty100TI2C ++
  new WithArty100TOspi ++
  new chipyard.iobinders.WithOspiPunchthrough ++
  new ospi.WithOspiCapture ++
  new chipyard.config.WithI2C)

class RocketArty100TConfig extends Config(
  new WithArty100TTweaks ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** The FTDI carrier's six signal pins, all in bank 14 of the xc7a100t-fgg484.
 *
 *  A different carrier from the TE0712 drone carrier the HarnessBinders.scala
 *  defaults describe, so every pin below differs from those defaults. Read out of
 *  Vivado 2023.1 `get_package_pins` for this part:
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
 *  The SOM-net column is why the pinout moved at all: these six are on the
 *  TE0712's J2 connector, and every default in HarnessBinders.scala is on J1
 *  (`A19` = J1_46, `E21` = J1_41, and so on). Same SOM, different carrier, so a
 *  different connector is brought out. Trenz's own board file --
 *  arty_files/te0712-rd/board_files/TE0712_100_2C/2.0/part0_pins.xml -- declares
 *  all six `iostandard="LVCMOS33"`, and the post-route io report of every other
 *  arty100t build already requires VCCO_14 = 3.30 V, so 3.3 V here is the board's
 *  documented level rather than an assumption.
 *
 *  All six are general-purpose and none is shared with the SPI-flash
 *  configuration interface. TCK lands on an MRCC pin, which makes
 *  the `clockDedicatedRouteFalse` the binder applies unnecessary rather than
 *  wrong -- it is left in place because it costs nothing and keeps one JTAG binder
 *  serving both carriers.
 *
 *  The UART is the carrier's FTDI channel, so it is the console UART and not the
 *  UART-TSI loader. Beware when adding a camera config on this carrier:
 *  WithArty100TOspi still defaults INT to V20 and TRIG to U20, which collide with
 *  TMS and TDO above.
 */
class WithArty100TFtdiCarrierPins extends Config(
  new WithArty100TUART("V18", "Y18") ++
  new WithArty100TJTAG(tckPin = "W19", tmsPin = "V20", tdiPin = "W20", tdoPin = "U20"))

/** The DDR3 bring-up target for the FTDI carrier: one Rocket, 1 GiB of DDR3, nothing else.
 *
 *  Same SoC as RocketArty100TConfig -- one huge Rocket, no L2, 50 MHz, the MIG
 *  behind a 1 GiB TL backing memory at 0x8000_0000 -- with two differences, both
 *  forced by the carrier rather than chosen:
 *
 *   - the FTDI carrier pinout above, instead of the TE0712 drone carrier's;
 *   - no UART-TSI. The carrier exposes one RX/TX pair and it is the FTDI console,
 *     so there is no second pair for the loader. Programs go in over JTAG
 *     (`hardware/openocd/arty100t.cfg`, whose work area is already at
 *     0x8000_1000 and therefore in DDR3), which also makes the debug module's
 *     memory access the DDR3 read/write path under test.
 *
 *  Deliberately no scratchpad: `ddr = true` leaves ExtTLMem defined, so the
 *  harness places the MIG and the only memory at 0x8000_0000 is real DDR3. A
 *  failure here is a DDR3, pin, or calibration problem and cannot be masked by a
 *  scratchpad answering in its place. `chipyard.config.AbstractConfig`'s 64 KiB
 *  scratchpad at 0x0800_0000 is still present, as it is in every Chipyard design.
 */
class RocketArty100TFtdiDDRConfig extends Config(
  new WithArty100TFtdiCarrierPins ++
  new WithArty100TTweaks(uartTsi = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** RocketArty100TConfig without the DDR3: 32 KiB mbus scratchpad, no MIG.
 *
 *  The scratchpad half of the bring-up pair. No camera, no I2C -- just the
 *  huge Rocket, the two UARTs, and JTAG, which is the smallest design that
 *  still exercises the FTDI path.
 */
class RocketArty100TSpadConfig extends Config(
  new WithArty100TTweaks(ddr = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

class RocketArty100TOspiConfig extends Config(
  new WithArty100TOspiPeriphery ++
  new WithArty100TTweaks ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/**
 * Logical full-drone SoC: camera/I2C plus PMW3901 SPI/control GPIO and two PWM
 * blocks. SPI, GPIO, and motor package pins remain intentionally unbound until
 * an explicit Arty-to-base wiring definition is selected.
 */
class RocketArty100TDroneLogicConfig extends Config(
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithArty100TOspiPeriphery ++
  new WithArty100TTweaks ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** The baseline OSPI SoC with its 1 GiB of DDR3 replaced by a 32 KiB scratchpad.
 *
 *  Same huge Rocket, same camera/I2C periphery, same 50 MHz. The DDR baseline
 *  fills 49,256 of the xc7a100t's 63,400 LUT; this is the variant that shows
 *  what is left once the MIG is gone, and the one to run when a workload fits
 *  in 32 KiB and the DDR3 timing closure is not wanted in the loop.
 */
class RocketArty100TOspiSpadConfig extends Config(
  new WithArty100TOspiPeriphery ++
  new WithArty100TTweaks(ddr = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** A stretch build: the baseline SoC plus a second, small Rocket tile.
 *
 *  The headroom is the whole question. RocketArty100TOspiConfig places at
 *  49,256 of the xc7a100t's 63,400 LUT, leaving 14,144, and a huge tile is
 *  28,813 of them -- so a second huge core cannot fit and is not attempted.
 *  WithNSmallCores drops the FPU, the BTB, and virtual memory and shrinks both
 *  L1s to 64 sets by 1 way, which is what makes a second tile plausible at all.
 *  Whether it then routes at 50 MHz with the MIG already in place is what the
 *  build answers; treat it as unproven until the report says otherwise.
 */
class DualRocketArty100TOspiConfig extends Config(
  new WithArty100TOspiPeriphery ++
  new WithArty100TTweaks ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new freechips.rocketchip.rocket.WithNSmallCores(1) ++
  new chipyard.RocketConfig)

/** Camera periphery sized for bring-up rather than streaming.
 *
 *  Identical to WithArty100TOspiPeriphery except for the capture buffer: the streaming default
 *  holds a whole 324x324 frame (104,977 beats, block RAM), which is far more than a bring-up
 *  capture of one QVGA line needs. `captureBeats` defaults to 1024, comfortably above the 324
 *  pixels plus EOF marker of a single line.
 */
class WithArty100TCameraBringupPeriphery(captureBeats: Int = 1024) extends Config(
  new WithArty100TI2C ++
  new WithArty100TOspi ++
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
 *  DELIBERATE DEVIATION from "one small Rocket": this keeps chipyard.RocketConfig's single
 *  WithNHugeCores(1) rather than WithNSmallCores(1). A small core drops the FPU and virtual
 *  memory, which changes the DTS ISA string and invalidates the Zephyr board contract that
 *  `rb` validates and that every working image on this board was built against. Deriving from
 *  the known-good target matters more here than shaving a core, and area is not the constraint:
 *  the scratchpad build places at 59% LUT.
 */
class CameraBringupConfig extends Config(
  new WithArty100TCameraBringupPeriphery ++
  new WithArty100TTweaks(ddr = false) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

class NoCoresArty100TConfig extends Config(
  new WithArty100TTweaks ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.NoCoresConfig)
