// See LICENSE for license details.
package chipyard.fpga.ku040

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

/** The board's 1 GiB DDR4 components as one contiguous TL backing memory.
 *
 *  `controllers` selects how many of the two are used. Each placed controller is
 *  KU040DDRSize; the harness derives their number from ExtMem's size and joins
 *  them behind a crossbar, so the SoC sees one contiguous region at ExtMem's
 *  base either way. One controller is 1 GiB and saves the second MIG's ~11,000
 *  LUT and its bank-46 I/O column; the flight mix touched 1.20 MB under FireSim,
 *  so capacity is not what decides this.
 *
 *  The size must be set here rather than overridden from a config layered on top:
 *  WithTLBackingMemory computes ExtTLMem from `up(ExtMem)`, so a higher-priority
 *  WithExtMemSize is never seen by the fragment that consumes it.
 */
class WithKU040DDRMem(controllers: Int = 2) extends Config(
  new WithKU040DDRTL ++
  new chipyard.config.WithTLBackingMemory ++
  new freechips.rocketchip.subsystem.WithExtMemSize(BigInt(0x40000000L) * controllers)) {
  require(controllers == 1 || controllers == 2,
    s"the KU040 carries two DDR4 components; asked for ${controllers} controllers")
}

/** The pre-DDR arrangement: a 32 KiB mbus scratchpad and no memory port.
 *
 *  Retained only so the 32 KiB memory map stays buildable, and because every
 *  KU040 area number recorded before the DDR4 work was measured against it.
 *  Configurations built this way leave ExtTLMem undefined, so the harness
 *  instantiates no MIG at all.
 */
class WithKU040ScratchpadMem(size: BigInt = BigInt(1) << 15) extends Config(
  new testchipip.soc.WithMbusScratchpad(base = 0x80000000L, size = size) ++
  new freechips.rocketchip.subsystem.WithNoMemPort)

// The sifive UART (Zephyr console) gets the wired PMOD pins D3/D4; UART-TSI
// is parked on spare pins A4/B4 until needed.
//
// `ddr` chooses whether there is a backing memory port at all -- false is the
// 32 KiB mbus scratchpad, which has none -- and `ddrControllers` how many of the
// board's two DDR4 components are placed behind it when there is.
class WithKU040Tweaks(freqMHz: Double = 50, uartRxdPin: String = "D3", ddr: Boolean = false,
                      ddrControllers: Int = 2,
                      scratchpadBytes: BigInt = BigInt(1) << 15,
                      uartTxdPin: String = "D4",
                      ioStandard: String = "LVCMOS18",
                      jtagTckPin: String = "E5", jtagTmsPin: String = "C6",
                      jtagTdiPin: String = "D5", jtagTdoPin: String = "D6") extends Config(
  new WithKU040UART(rxdPin = uartRxdPin, txdPin = uartTxdPin, ioStandard = ioStandard) ++
  new WithKU040UARTTSI ++
  new WithKU040JTAG(tckPin = jtagTckPin, tmsPin = jtagTmsPin,
                    tdiPin = jtagTdiPin, tdoPin = jtagTdoPin, ioStandard = ioStandard) ++
  new WithNoDesignKey ++
  new testchipip.tsi.WithUARTTSIClient ++
  new chipyard.harness.WithSerialTLTiedOff ++
  new chipyard.harness.WithHarnessBinderClockFreqMHz(freqMHz) ++
  new chipyard.config.WithUniformBusFrequencies(freqMHz) ++
  new chipyard.harness.WithAllClocksFromHarnessClockInstantiator ++
  new chipyard.clocking.WithPassthroughClockGenerator ++
  (if (ddr) new WithKU040DDRMem(ddrControllers) else new WithKU040ScratchpadMem(scratchpadBytes)) ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors)

/** Opt-in HM01B0 capture, including the I2C controller used to configure the sensor. */
class WithKU040OspiPeriphery extends Config(
  new WithKU040I2C ++
  new WithKU040Ospi ++
  new chipyard.iobinders.WithOspiPunchthrough ++
  new ospi.WithOspiCapture ++
  new chipyard.config.WithI2C)

class RocketKU040Config extends Config(
  new WithKU040Tweaks ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** RocketKU040Config with a 128 KiB scratchpad instead of 32 KiB.
 *
 *  Bring-up convenience only. A default Zephyr `hello_world` for
 *  chipyard_riscv64 is 54,128 B, and even trimmed hard (nano cbprintf, minimal
 *  libc, riscv,ndev cut from 1024 to 32, stacks reduced) it lands at 36,400 B --
 *  still 3,632 B over the 32 KiB budget. 128 KiB leaves room to bring the
 *  console up without also fighting the linker.
 *
 *  Note for `rb`: boards.py treats any config whose name contains "Spad" as a
 *  32 KiB scratchpad config and reports SCRATCHPAD_MEMORY_SIZE for it, so `rb`
 *  will understate this one's memory until that table learns the size.
 */
class RocketKU040Spad128Config extends Config(
  new WithKU040Tweaks(scratchpadBytes = BigInt(128) << 10) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

class RocketKU040OspiConfig extends Config(
  new WithKU040OspiPeriphery ++
  new WithKU040Tweaks(uartRxdPin = "C3") ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** Rocket with the board's full 2 GiB of DDR4 as backing memory.
 *
 *  The DDR counterparts of RocketKU040Config / RocketKU040OspiConfig. They are
 *  separate configurations rather than a change to the existing ones so that
 *  every recorded KU040 area number stays reproducible against the memory map it
 *  was measured on.
 */
class RocketKU040DDRConfig extends Config(
  new WithKU040Tweaks(ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

class RocketKU040OspiDDRConfig extends Config(
  new WithKU040OspiPeriphery ++
  new WithKU040Tweaks(uartRxdPin = "C3", ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** RocketKU040OspiDDRConfig asked for 100 MHz instead of 50. A timing probe.
 *
 *  Nothing else differs, so the pair isolates one question: is the scalar SoC
 *  intrinsically limited to ~68 MHz on this part, or was that an artifact of
 *  never having asked it for more?
 *
 *  The question is live because the accelerator build -- which *is* constrained
 *  at 100 MHz -- reports RocketTile's own logic at -5.0 ns:
 *
 *    core     -5.029 ns (27 logic levels)   frontend  -5.026 ns (28)
 *    fpuOpt   -5.097 ns (28)                dcache    -4.627 ns (19)
 *
 *  i.e. ~66 MHz for Rocket alone, before any accelerator path is considered.
 *  If that number is intrinsic, no accelerator cut reaches 100 MHz and the
 *  target has to move. If instead it is congestion from sharing a 74%-full die
 *  with Gemmini and Saturn, then area reduction and floorplanning are the
 *  levers and 100 MHz stays reachable.
 *
 *  At 50 MHz this same config closes with +5.177 ns, but 86% of its worst path
 *  is route delay -- the signature of a placer that stopped once it met a loose
 *  constraint. That is why the 50 MHz result cannot answer the question and
 *  this config exists.
 */
class RocketKU040OspiDDR100Config extends Config(
  new WithKU040OspiPeriphery ++
  new WithKU040Tweaks(freqMHz = 100, uartRxdPin = "C3", ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** The rest of the frequency sweep: 125, 150, 175 MHz.
 *
 *  100 MHz met with +0.501 ns, which bounds the scalar SoC from below and not
 *  from above: Vivado stops optimizing once a constraint is met, so a positive
 *  WNS says "at least this fast" and nothing more. That is exactly the trap the
 *  50 MHz build set -- it reported +5.177 ns, which read as a 67.5 MHz ceiling
 *  for a design that in fact clears 100.
 *
 *  Ceilings therefore have to be bracketed by asking for more until it fails.
 *  Each point is one synthesis plus one place-and-route; the first constraint
 *  that misses puts the true F_max between it and the last one that met.
 *
 *  The MMCM can supply all three from the 200 MHz board clock: VCO 1000/8,
 *  1200/8, 1050/6. The MIG's user interface is unaffected -- it runs at 225 MHz
 *  in its own domain behind AsynchronousCrossing(8), so the SoC clock is free.
 *
 *  What the answer is for: the accelerator build reports RocketTile's own logic
 *  at -5.0 ns, and whether that is intrinsic or congestion decides whether area
 *  reduction is a fitting exercise or a timing one. The wider the scalar
 *  headroom, the more of that -5.0 ns is attributable to sharing the die.
 */
class RocketKU040OspiDDR125Config extends Config(
  new WithKU040OspiPeriphery ++
  new WithKU040Tweaks(freqMHz = 125, uartRxdPin = "C3", ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

class RocketKU040OspiDDR150Config extends Config(
  new WithKU040OspiPeriphery ++
  new WithKU040Tweaks(freqMHz = 150, uartRxdPin = "C3", ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

class RocketKU040OspiDDR175Config extends Config(
  new WithKU040OspiPeriphery ++
  new WithKU040Tweaks(freqMHz = 175, uartRxdPin = "C3", ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/**
 * Logical full-drone SoC for the custom XCKU040-SFVA784-1-C board.
 *
 * The PMW3901 SPI/control GPIO and motor PWM package pins are deliberately
 * unbound until the custom base-connector mapping and voltage domains are
 * provided and verified.
 */
class RocketKU040DroneLogicConfig extends Config(
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithKU040OspiPeriphery ++
  new WithKU040Tweaks(uartRxdPin = "C3") ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** The full-drone SoC with the board's 2 GiB of DDR4.
 *
 *  This is the deployable drone configuration: the scratchpad variant above is
 *  kept only so the 32 KiB memory map stays buildable for area comparisons.
 *  `rb`'s board registry promises 2 GiB for ku040, so this is the variant whose
 *  generated DTS satisfies the hardware contract.
 */
class RocketKU040DroneLogicDDRConfig extends Config(
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithKU040OspiPeriphery ++
  new WithKU040Tweaks(uartRxdPin = "C3", ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** Clean minimal bring-up: Rocket + the board's 2 GiB of DDR4 + the riskybird sensor-array
 *  periphery (I2C, SPI, GPIO, PWM x2, a second UART), no camera and no accelerator (no Gemmini,
 *  no Saturn, no FcRoCC). Built to give those a real timing headroom on the bigger KU040 that
 *  the congested Artix-200T lacks.
 *
 *  Peripheral set and addresses are ported from the arty200t "full sensor array" reference,
 *  `RocketArty200TDroneFullDDRConfig` (fpga/src/main/scala/arty200t/Configs.scala):
 *
 *    i2c@0x10040000   BMI088 IMU / VL53L1X ToF / BMP388 / ADS7128 expander
 *    spi@0x10031000   PMW3901 optical flow ("spi2" in the arty200t DTS numbering)
 *    gpio@0x10010000  3-bit: PMW3901 CS, reset, LED_N (chipyard.config.WithRiskyBirdDronePeriphery)
 *    pwm@0x10050000   motor block 0 (comparators 1-3)
 *    pwm@0x10051000   motor block 1 (comparator 1) -- 4 motor channels total
 *    uart0@0x10020000 console, on the PMOD D3/D4 pins (WithKU040Tweaks default)
 *    uart1@0x10021000 ESP telemetry
 *
 *  Every one of these device fragments (WithI2C/WithSPI/WithGPIO/WithPWM/WithUART) wires its
 *  interrupt into the PLIC automatically through rocket-chip's normal peripheral-bus interrupt
 *  aggregation -- there is no separate manual PLIC step.
 *
 *  PACKAGE PINS. I2C is physically bound (WithKU040I2C, A1/A2 -- the same bank-68 header pins
 *  used by JTAG/UART0/UART-TSI). SPI, GPIO and PWM follow RocketKU040DroneLogicConfig's own
 *  precedent and are left logical-only: WithRiskyBirdDronePeriphery's doc comment is explicit
 *  that "board harness binders must be added only after the connector-to-package mapping and
 *  I/O voltage compatibility have been verified" for the PMW3901/motor connector, which is a
 *  separate, not-yet-designed connector from the generic bank-68 bring-up header. Left unbound
 *  they are safe to build: SPI/GPIO fall through to AbstractConfig's synthesizable
 *  WithSPITiedOff/WithGPIOTiedOff, and PWM is never punched to ChipTop at all without an
 *  explicit WithPWMPunchthrough (see that class's doc comment in
 *  generators/chipyard/src/main/scala/iobinders/IOBinders.scala). uart1 is given the same
 *  treatment via WithKU040UARTTiedOff, which is needed for a different reason: left unclaimed it
 *  would otherwise fall through to AbstractConfig's WithUARTAdapter, a simulation-only DPI model
 *  that is not synthesizable, rather than a config-safe default.
 *
 *  Zephyr note: no chipyard_riscv64-style KU040 board overlay exists yet for this address map,
 *  and until the SPI/GPIO/PWM/uart1 package pins above are bound, no overlay could reach real
 *  hardware regardless. The bitstream is what this configuration is for.
 */
class RocketKU040DroneSensorsConfig extends Config(
  new WithKU040UARTTiedOff(uartNo = 1) ++
  new chipyard.config.WithUART(address = 0x10021000) ++ // uart1: ESP telemetry (console uart0 stays 0x10020000)
  new chipyard.config.WithRiskyBirdDronePeriphery ++     // spi2 + gpio(3) + pwm x2 -- logical only, see above
  new WithKU040I2C ++
  new chipyard.config.WithI2C ++                         // i2c@0x10040000, physically bound to A1/A2
  // riskybirdv3 carrier: the debug FTDI reaches the RISC-V JTAG + console UART on JB1 =
  // VCCIOA = XCKU040 Bank 64, which is the HR (3.3V-capable) bank per Trenz's 4x5 SoM
  // Integration Guide -- NOT Loren's bank-68 (E5/C6/D5/D6, D3/D4) carrier default. Balls
  // from the riskybird B2B pinout (hermaphroditic swap): TCK=AA10 TMS=Y12 TDI=Y10 TDO=Y11,
  // uart0 rxd=AH12 txd=AG12, all LVCMOS33 (VCCIOA fed +3V3 on JB1.10/12).
  new WithKU040Tweaks(ddr = true,                        // both DDR4 controllers, 2 GiB
    uartRxdPin = "AH12", uartTxdPin = "AG12", ioStandard = "LVCMOS33",
    jtagTckPin = "AA10", jtagTmsPin = "Y12", jtagTdiPin = "Y10", jtagTdoPin = "Y11") ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** Full riskybird v3 drone SoC on the KU040, with EVERY peripheral placed on the riskybird v3
 *  carrier's TE0841 balls (not Loren's KU040 carrier defaults):
 *    - console uart0 + debug jtag on Bank 64 (HR/3.3V) -- bench-verified
 *    - i2c sensor bus on Bank 64 balls AA12/AB12 (IMU/ToF/baro/ADS7128 + the HM01B0 SCCB, which is
 *      level-shifted onto this same bus by an on-carrier PCA9306, so no separate camera-i2c ball)
 *    - PMW3901 optical-flow SPI (AC9/AD9/AH14) + its GPIO CS/reset/LED (AH13/AB10/AG10), Bank 64
 *    - 4 motor PWM outputs (AH8/AB9/AF23/AB15; motor3 is Bank 65, the rest Bank 64 -- both HR/3.3V),
 *      inverted so they are OFF at power-on
 *    - HM01B0 camera OSPI parallel video on Bank 66 (HP, 1.8V, LVCMOS18) -- VCCIOB is wired to the
 *      carrier +1V8 rail, matching the HM01B0's 1.8V IOVDD
 *  All balls cross-validated (KiCAD netlist + Trenz B2B tables + the Trenz 4x5 SoM Integration Guide
 *  bank table) against the working arty200t drone config. Mirrors RocketArty200TDroneFullDDRConfig's
 *  device + punchthrough recipe; 2 GiB DDR4 backing.
 */
class RocketKU040DroneFullConfig extends Config(
  new WithKU040UARTTiedOff(uartNo = 1) ++                 // ESP telem uart1: no carrier balls traced yet
  new chipyard.config.WithUART(address = 0x10021000) ++  // uart1 device (console uart0 stays 0x10020000)
  new WithKU040PWM ++                                     // 4 motors on Bank 64/65 balls (inverted -> off at reset)
  new chipyard.iobinders.WithPWMPunchthrough ++
  new WithKU040SPI ++                                     // PMW3901 flow SPI, Bank 64
  new WithKU040GPIO ++                                    // flow CS/reset/LED, Bank 64
  new chipyard.config.WithRiskyBirdDronePeriphery ++     // SoC devices: spi2 + gpio(3) + pwm x2
  new WithKU040Ospi(                                      // HM01B0 camera parallel video, Bank 66 / LVCMOS18
    dataPins = Seq("N7", "M5", "L5", "J1", "H1", "J4", "J5", "M7"),
    pclkPin = "L8", fvldPin = "L1", lvldPin = "K2", intrPin = "K1", mclkPin = "N8", trigPin = "M1") ++
  new chipyard.iobinders.WithOspiPunchthrough ++
  new ospi.WithOspiCapture ++
  new WithKU040I2C(sclPin = "AA12", sdaPin = "AB12", ioStandard = "LVCMOS33") ++  // Bank 64 sensor + SCCB bus
  new chipyard.config.WithI2C ++                          // i2c@0x10040000
  new WithKU040Tweaks(ddr = true,                        // 2 GiB DDR4; uart0 + jtag on Bank 64 (riskybird carrier)
    uartRxdPin = "AH12", uartTxdPin = "AG12", ioStandard = "LVCMOS33",
    jtagTckPin = "AA10", jtagTmsPin = "Y12", jtagTdiPin = "Y10", jtagTdoPin = "Y11") ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.RocketConfig)

/** Per-tile FP32-only scalar FPU (minFLen=fLen=32: single precision F, no double D).
 *  Scoped by tileId via TileAttachConfig.atTileIds so it can target one core in a
 *  multi-tile design. Must be composed to the LEFT of saturn.WithRocketVectorUnit,
 *  which otherwise force-sets minFLen=16 on its tile and would win. */
class WithFP32OnlyFPUOnTiles(ids: Int*) extends Config(
  new freechips.rocketchip.subsystem.TileAttachConfig[freechips.rocketchip.subsystem.RocketTileAttachParams](
    tp => tp.copy(tileParams = tp.tileParams.copy(core = tp.tileParams.core.copy(
      fpu = tp.tileParams.core.fpu.map(_.copy(minFLen = 32, fLen = 32)))))
  ).atTileIds(ids: _*))

/** "More powerful" heterogeneous DUAL-CORE riskybird v3 drone SoC on the KU040:
 *
 *   - hart 0 ("Saturn core"): Rocket + Saturn V128D128 RVV vector unit (INTEGER-ONLY) +
 *     the FcRoCC flight-controller accelerator (RoCC custom0). Keeps a hardware scalar
 *     FPU restricted to FP32-only (no FP64).
 *   - hart 1 ("Gemmini core"): Rocket + Q0.31 32x32 Gemmini (RoCC custom3). No Saturn/FcRoCC.
 *   - TACIT instruction-trace encoder + DDR trace sink, with TACIT's OWN internal DSC branch
 *     predictor OFF (useBP=false, the default -- saves ~3,045 LUT + 2,048 FF per encoder).
 *     Rocket's own BTB/BHT/RAS are left ENABLED at spec default.
 *   - All riskybird carrier peripherals on the correct TE0841 balls (i2c/spi/gpio/pwm Bank64/65
 *     LVCMOS33, HM01B0 camera Bank66 LVCMOS18) + console uart0/debug jtag on Bank64, same as
 *     RocketKU040DroneFullConfig.
 *   - DDR4: 1 controller (1 GiB) -- fit-conscious for this accel-heavy dual-core design.
 *
 *  FP32-only is applied to BOTH tiles: neither core needs FP64 (the Gemmini core runs int8
 *  dispatch), and dropping FP64 saves LUTs on this tight design. Change to (0) to keep hart 1's
 *  default FP64 FPU.
 *
 *  BUILD (applies the same use_dsp DSP-packing + BRAM opts as the prior accel bitstreams):
 *    make -C fpga SUB_PROJECT=ku040 CONFIG=RocketKU040DroneDualConfig RB_ATTRS=dsp rb-area
 *    make -C fpga SUB_PROJECT=ku040 CONFIG=RocketKU040DroneDualConfig RB_ATTRS=dsp rb-impl
 *
 *  FIT is the #1 risk (first multi-core ku040 config; 32x32 Gemmini alone is ~129k LUT under
 *  use_dsp). Fallbacks if rb-area overruns: 16x16 Gemmini, drop TACIT to hart 0 only, or drop DDR.
 */
class RocketKU040DroneDualConfig extends Config(
  // ---- riskybird carrier peripherals + pins (same as RocketKU040DroneFullConfig) ----
  new WithKU040UARTTiedOff(uartNo = 1) ++
  new chipyard.config.WithUART(address = 0x10021000) ++
  new WithKU040PWM ++
  new chipyard.iobinders.WithPWMPunchthrough ++
  new WithKU040SPI ++
  new WithKU040GPIO ++
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithKU040Ospi(
    dataPins = Seq("N7", "M5", "L5", "J1", "H1", "J4", "J5", "M7"),
    pclkPin = "L8", fvldPin = "L1", lvldPin = "K2", intrPin = "K1", mclkPin = "N8", trigPin = "M1") ++
  new chipyard.iobinders.WithOspiPunchthrough ++
  new ospi.WithOspiCapture ++
  new WithKU040I2C(sclPin = "AA12", sdaPin = "AB12", ioStandard = "LVCMOS33") ++
  new chipyard.config.WithI2C ++
  // ---- DDR + clocking + console uart0/debug jtag on Bank 64 (riskybird carrier) ----
  new WithKU040Tweaks(ddr = true, ddrControllers = 1,
    uartRxdPin = "AH12", uartTxdPin = "AG12", ioStandard = "LVCMOS33",
    jtagTckPin = "AA10", jtagTmsPin = "Y12", jtagTdiPin = "Y10", jtagTdoPin = "Y11") ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  // ---- TACIT trace (both tiles); encoder rightmost of the group; internal DSC BP OFF ----
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder(useBP = false) ++
  // ---- FP32-only scalar FPU on both tiles (LEFT of Saturn so it wins over minFLen=16) ----
  new WithFP32OnlyFPUOnTiles(0, 1) ++
  // ---- per-hart RoCC: FcRoCC (custom0) -> hart 0, Gemmini 32x32 (custom3) -> hart 1 ----
  new chipyard.config.WithMultiRoCC ++
  new chipyard.config.WithMultiRoCCFromBuildRoCC(0) ++
  new chipyard.fc.WithFcRoCC ++
  new chipyard.config.WithMultiRoCCFromBuildRoCC(1) ++
  new gemmini.Q31Ws32x32AccGemminiConfig ++
  // ---- Saturn V128D128 integer-only vector unit -> hart 0 only ----
  new saturn.rocket.WithRocketVectorUnit(128, 128, saturn.common.VectorParams.intOnlyParams,
    cores = Some(Seq(0))) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(2) ++   // tile0=hart0 (Saturn+Fc), tile1=hart1 (Gemmini)
  new chipyard.config.AbstractConfig)

/** 16x16-Gemmini FALLBACK of RocketKU040DroneDualConfig. Identical in every other respect, but
 *  swaps the 32x32 Gemmini (~129k LUT, congestion-5 at synth) for the proven 16x16 DroNet mesh
 *  (RbArty200TGemmini.mesh16 -- the exact Gemmini the flown At35 22fps DroNet used: 16x16, 64KB
 *  acc, has_loop_conv, no mvin-scale). Big LUT + routing-congestion relief; run in parallel with
 *  the 32x32 rb-impl so we have a routable/timing-closing bitstream if the 32x32 doesn't fit.
 *  Build: make -C fpga SUB_PROJECT=ku040 CONFIG=RocketKU040DroneDual16Config RB_ATTRS=dsp rb-area && ... rb-impl
 */
class RocketKU040DroneDual16Config extends Config(
  new WithKU040UARTTiedOff(uartNo = 1) ++
  new chipyard.config.WithUART(address = 0x10021000) ++
  new WithKU040PWM ++
  new chipyard.iobinders.WithPWMPunchthrough ++
  new WithKU040SPI ++
  new WithKU040GPIO ++
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithKU040Ospi(
    dataPins = Seq("N7", "M5", "L5", "J1", "H1", "J4", "J5", "M7"),
    pclkPin = "L8", fvldPin = "L1", lvldPin = "K2", intrPin = "K1", mclkPin = "N8", trigPin = "M1") ++
  new chipyard.iobinders.WithOspiPunchthrough ++
  new ospi.WithOspiCapture ++
  new WithKU040I2C(sclPin = "AA12", sdaPin = "AB12", ioStandard = "LVCMOS33") ++
  new chipyard.config.WithI2C ++
  new WithKU040Tweaks(ddr = true, ddrControllers = 1,
    uartRxdPin = "AH12", uartTxdPin = "AG12", ioStandard = "LVCMOS33",
    jtagTckPin = "AA10", jtagTmsPin = "Y12", jtagTdiPin = "Y10", jtagTdoPin = "Y11") ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder(useBP = false) ++
  new WithFP32OnlyFPUOnTiles(0, 1) ++
  new chipyard.config.WithMultiRoCC ++
  new chipyard.config.WithMultiRoCCFromBuildRoCC(0) ++
  new chipyard.fc.WithFcRoCC ++
  new chipyard.config.WithMultiRoCCFromBuildRoCC(1) ++
  new gemmini.Q31GemminiConfig(chipyard.fpga.arty200t.RbArty200TGemmini.mesh16) ++  // 16x16 fallback
  new saturn.rocket.WithRocketVectorUnit(128, 128, saturn.common.VectorParams.intOnlyParams,
    cores = Some(Seq(0))) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(2) ++
  new chipyard.config.AbstractConfig)

class SaturnKU040Config extends Config(
  new WithKU040Tweaks(ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.REFV256D128RocketConfig)

class SaturnKU040OspiConfig extends Config(
  new WithKU040OspiPeriphery ++
  new WithKU040Tweaks(uartRxdPin = "C3", ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.REFV256D128RocketConfig)

class SaturnKU040DroneLogicConfig extends Config(
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithKU040OspiPeriphery ++
  new WithKU040Tweaks(uartRxdPin = "C3", ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.REFV256D128RocketConfig)

/** Rocket plus the default 16x16 int8 Gemmini, on the board's 2 GiB of DDR4.
 *
 *  The default mesh is 16x16, giving 256 MacUnit instances. That is the
 *  population the `use_dsp` injection in riskybird/tcl/rb_attributes.tcl targets,
 *  so this config is what makes the DSP-inference claim measurable without
 *  moving any submodule pin.
 *
 *  Recorded area for this class predates the DDR4 switch and was measured
 *  against the 32 KiB scratchpad; add the measured 22,027 LUT of the controller
 *  pair, or re-run `rb area`, before comparing against anything current.
 */
class GemminiKU040Config extends Config(
  new WithKU040Tweaks(ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.GemminiRocketConfig)

/** Q0.31 weight-stationary Gemmini, 32x32 mesh, 128 KB accumulator.
 *
 *  1024 MacUnit instances -- the largest use_dsp injection target. The 128 KB
 *  accumulator keeps the banks BRAM-mappable at this mesh width; at 64 KB they
 *  demote to LUTRAM.
 *
 *  Measured synth-only on xcku040-sfva784-1-c, before the DDR4 switch:
 *    pristine  223,841 LUT (92.3%)    326 DSP (17.0%)
 *    use_dsp   129,190 LUT (53.3%)  1,355 DSP (70.6%)
 *  With the controller pair's measured 22,027 LUT the pristine build no longer
 *  fits the device; the use_dsp build lands at ~151,200 LUT (62.4%).
 */
class Q31Ws32x32AccGemminiKU040Config extends Config(
  new WithKU040Tweaks(ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.Q31Ws32x32AccGemminiRocketConfig)

/** V128D128 Saturn vector unit with the int8 Outer Product Unit.
 *
 *  256 OuterProductCell instances -- one OuterProductUnit x 16 clusters x 16
 *  cells. Each cell costs two DSP48E2 under use_dsp, not one: the 8x8 signed
 *  multiply feeds a 32-bit add whose addend arrives through a 4:1 mux over the
 *  cell's register file, so the DSP's internal accumulate feedback cannot be
 *  used. That is why the +517 DSP delta is 256*2+5 and not 512+5.
 *
 *  Measured synth-only on xcku040-sfva784-1-c, before the DDR4 switch:
 *    pristine  191,455 LUT (79.0%)  155 DSP (8.1%)
 *    use_dsp   161,133 LUT (66.5%)  672 DSP (35.0%)
 *  Add the controller pair's measured 22,027 LUT for the current shape.
 */
class SaturnOPUV128D128KU040Config extends Config(
  new WithKU040Tweaks(ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.REFV128D128RocketOPUConfig)

/** V128D128 Saturn vector unit and the Q0.31 32x32 Gemmini on one Rocket.
 *
 *  Both accelerators on a single hart: Saturn as the vector unit in the tile,
 *  Gemmini as a RoCC coprocessor on that tile's port. This is the shape the
 *  FireSim profiling machine has, so it is the design point where a borrowed
 *  profile stands the best chance of transferring.
 *
 *  Feasibility is the open question, and the DDR4 controller pair's measured
 *  22,027 LUT comes out of the same budget. The measured accelerator increments
 *  with use_dsp are ~90,100 LUT for this Gemmini and ~122,100 LUT for the
 *  V128D128 Saturn *including* its Outer Product Unit, which together overrun
 *  the device before DDR is counted at all.
 *  Dropping the OPU is what might make room: the OPU is 256 cells at two
 *  DSP48E2 each, and this workload mix reaches int8 matmul through Gemmini
 *  rather than through Saturn's outer-product path.
 *
 *  The FPU is deliberately kept. `mlp_control` is fp32 and is the only
 *  safety-critical workload in the mix, so the ~14K LUTs a WithoutFPU cut would
 *  recover are not available here.
 */
class Q31Ws32x32AccGemminiSaturnV128D128KU040Config extends Config(
  new WithKU040Tweaks(ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new saturn.rocket.WithRocketVectorUnit(128, 128, saturn.common.VectorParams.refParams) ++
  new gemmini.Q31Ws32x32AccGemminiConfig ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/** As above, with Saturn's floating-point units stripped.
 *
 *  Measured from the hierarchy of the full build: Saturn's `ExecutionUnitfp` is
 *  49,670 LUT, over half the vector unit's 92,447 and the single largest item
 *  after Gemmini's mesh. `intOnlyParams` sets `noFP = true`, removing the
 *  FPFMAPipe, FPConv, FPDiv and FPCmp instantiations.
 *
 *  The Rocket *scalar* FPU is deliberately retained -- `WithRocketNoFPU` is not
 *  applied -- so `mlp_control`, the fp32 and only safety-critical workload in
 *  the mix, still executes. It loses vector acceleration for fp32 and falls
 *  back to the scalar FPU, which its 0.527 ms against a 20 ms deadline can
 *  absorb. The int8 networks are unaffected: they reach the vector unit through
 *  integer paths and Gemmini through RoCC.
 */
class Q31Ws32x32AccGemminiSaturnV128D128IntOnlyKU040Config extends Config(
  new WithKU040Tweaks(ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new saturn.rocket.WithRocketVectorUnit(128, 128, saturn.common.VectorParams.intOnlyParams) ++
  new gemmini.Q31Ws32x32AccGemminiConfig ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/** FP16-only Saturn alongside the Q0.31 32x32 Gemmini, targeting 100 MHz.
 *
 *  Third point in the FP-stripping waterfall: full FP (223,787 LUT, 92.3%) was
 *  infeasible, integer-only (177,064 LUT, 73.0%) fits, and this puts vector FP
 *  back at FP16 only. `robotMpcParams` sets `noFP64` and `noFP32`, keeping the
 *  FP16 FMA and dropping the FP32/FP64 datapaths, and switches to TandemFMAPipe
 *  because FP-stripping is gated there rather than in SegmentedFMAPipe.
 *
 *  `useElementwiseFP64 = true` is inert in this
 *  combination: `SIMDFPFMAFactory` filters SEW=3 and SEW=2 instructions out
 *  before the element-wise mapping runs, so with FP64 and FP32 already stripped
 *  there is nothing left for it to convert. It is set so the intent survives if
 *  the FP64 strip is ever relaxed.
 *
 *  The Rocket scalar FPU is still retained, so `mlp_control` -- fp32, and the
 *  only safety-critical workload in the mix -- keeps a correct execution path.
 *
 *  The 100 MHz target is a *synthesis constraint*, not a closed frequency. The
 *  only KU040 config ever placed and routed is the 16%-LUT Rocket, which
 *  achieved 69.5 MHz. Whether a design at this occupancy closes at 100 MHz is
 *  unknown until it is routed, and synth-only will report area regardless.
 */
class Q31Ws32x32AccGemminiSaturnV128D128Fp16KU040Config extends Config(
  new WithKU040Tweaks(freqMHz = 100, ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.robotMpcParams.copy(useElementwiseFP64 = true)) ++
  new gemmini.Q31Ws32x32AccGemminiConfig ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/** FP16 Saturn paired with an FP16-only Rocket scalar FPU, at 100 MHz.
 *
 *  Completes the pairing upstream's REFV128D128RocketRobotMpcConfig intends:
 *  robotMpcParams strips FP64/FP32 from the vector unit, WithRocketFPU16 does
 *  the same to the scalar FPU. The previous build had only the first half, so
 *  its fpuOpt was still the full 14,060 LUT f16/f32/f64 unit.
 *
 *  Measured composition of that scalar FPU: the FP16 FMA is 489 LUT of 14,032.
 *  The rest is per-precision FMA pipes, three separate div/sqrt units, format
 *  converters, and a 32x65-bit register file sized to hold doubles.
 *
 *  Caveat carried from docs/design-space.md: WithRocketFPU16 is non-spec -- Zfh
 *  without the F base -- and cannot run standard soft-float. It is only viable
 *  once MPC is genuinely fp16 and nothing else in the image needs fp32.
 *
 *  `noPermute` drops the vrgather / vcompress / vslide shuffle network, 2,101
 *  LUT measured. It is the only integer-side cut VectorParams exposes; the
 *  integer divider (1,326 LUT, nothing in the workload divides) and the
 *  bitmanip pipe (1,219 LUT, no vector bitmanip is issued) are unconditional in
 *  `integerALUs` / `integerFUs` and would need knobs adding in Saturn.
 *
 *  The permute cut is safe for an area probe -- synthesis executes nothing --
 *  but is not yet validated functionally. maxpool2d_s8, upsample_nearest_s8 and
 *  the cat ops could plausibly lower to slides or gathers, which would
 *  trap-illegal at runtime. That has to be checked before anything is run.
 */
class Q31Ws32x32AccGemminiSaturnV128D128Fp16FullKU040Config extends Config(
  new WithKU040Tweaks(freqMHz = 100, ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.robotMpcParams.copy(
      useElementwiseFP64 = true,
      noPermute = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new gemmini.Q31Ws32x32AccGemminiConfig ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/** As the FP16 point above, with Gemmini's fp32 mvin scale removed.
 *
 *  Place-and-route of the FP16 build missed 100 MHz by WNS -9.83 ns on a 10 ns
 *  period -- a ~20 ns critical path. Every one of the 100+ paths physical
 *  optimization touched was the same module replicated across scale units:
 *
 *    gemmini/spad/vsm/pipe{,_1,_3}/out_bits_data_muladder/mulAddRecFNToRaw_postMul
 *
 *  That is the `mvin` scale, which the Q0.31 work left as upstream Chipyard
 *  Gemmini: `mvin_scale_args` is inherited from `GemminiConfigs.defaultConfig`
 *  and is fp32 (`Float(8, 24)`), an INToRecFN -> MulAddRecFN -> round chain.
 *  Its `latency = 4` does not help, because `VectorScalarMultiplier` applies
 *  `Pipe(out, latency)` *after* the scale function, so the registers sit outside
 *  the combinational block rather than retiming it. The design was half
 *  converted: fixed-point on the way out, floating-point on the way in.
 *
 *  Removing it is free for this workload. Every call site in the modelblaster
 *  kernels and in the codegen template passes `MVIN_SCALE_IDENTITY` for both A
 *  and B, and requantization is done in scalar software after a `full_C=true`
 *  matmul writes the raw int32 accumulator out. No kernel asks for a
 *  non-identity input scale, so the hardware is dead.
 *
 *  Overridden here rather than in the Gemmini submodule so the existing
 *  measurements of the other Q0.31 configs stay reproducible, and so a
 *  fork-only pin needs no local edit.
 */
/** The LoopConv-keeping Q0.31 point, retargeted to a frequency it can close.
 *
 *  Same SoC as Q31Ws32x32AccGemminiSaturnV128D128Fp16NoMvinScaleKU040Config --
 *  LoopConv retained, because the xpurt flight3 networks need it -- but clocked
 *  at 50 MHz instead of 100.
 *
 *  Why: at 100 MHz that config does not close. Post-route WNS is -8.289 ns, so
 *  the critical path needs 18.289 ns and the achievable frequency is about
 *  54.7 MHz. Vivado still emits a bitstream with only a warning, which makes a
 *  timing-failing build easy to load and benchmark by accident. The NoLoopConv
 *  variants are better but also fail: -4.169 ns plain, -5.014 ns DDR.
 *
 *  One DDR4 controller, not the WithKU040Tweaks default of two: the second MIG
 *  costs roughly 11k LUTs, and every accelerator config that has actually
 *  placed on this part uses ddrControllers = 1. 1 GiB is ample for the flight3
 *  networks.
 *
 *  Build it with use_dsp injection, or it will not fit: pristine synthesis of
 *  this shape is 298,298 slice LUTs against 242,400 available (123%), and
 *  place_design never runs. Injection is a gen-collateral source edit, so the
 *  order is elaborate -> inject -> make bitstream.
 *
 *  50 MHz leaves 1.7 ns of margin against the measured path and matches the
 *  frequency hardware/zephyr/fpga-common.overlay and fpga.conf already assume
 *  (cpu@0 clock-frequency 50 MHz, CLINT timebase 50 kHz), so no Zephyr clock
 *  overrides are needed and MODELBLASTER_WALL_CYCLES stays trustworthy.
 */
class Q31Ws32x32AccGemminiSaturnV128D128Fp16NoMvinScale50KU040Config extends Config(
  new WithKU040Tweaks(freqMHz = 50, ddr = true, ddrControllers = 1) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.robotMpcParams.copy(
      useElementwiseFP64 = true,
      noPermute = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new gemmini.Q31GemminiConfig(
    gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(mvin_scale_args = None)) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

class Q31Ws32x32AccGemminiSaturnV128D128Fp16NoMvinScaleKU040Config extends Config(
  new WithKU040Tweaks(freqMHz = 100, ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.robotMpcParams.copy(
      useElementwiseFP64 = true,
      noPermute = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new gemmini.Q31GemminiConfig(
    gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(mvin_scale_args = None)) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/** The FP16 point again, with Gemmini's convolution loop unroller removed.
 *
 *  `has_loop_conv = false` deletes the LoopConv FSM; Controller.scala:251 then
 *  binds `conv_cmd` straight to `raw_cmd`, so commands pass through to
 *  LoopMatmul untouched. LoopMatmul, and therefore `tiled_matmul_auto`, is
 *  unaffected.
 *
 *  What it is expected to remove, measured from the NoMvinScale hierarchy:
 *    gemmini/mod (LoopConv)   5,435 LUT   132 DSP
 *  and with it the entire worse-than -6 ns population of the design. LoopConv
 *  owns the worst path -- `derived_params()` at LoopConv.scala:1107 elaborates
 *  four dependent DSP48E2 multiplies with no register between the loop-state
 *  registers and the consumer's capture flop, 10.18 ns of the 18.11 ns path --
 *  and it is instantiated five times, once per sub-unit, at lines 1388/1410/
 *  1430/1449/1477.
 *
 *  It is only ~1,300 ns of the 220,860 ns TNS, so this fixes WNS and not
 *  throughput: the predicted next wall is gemmini/ex_controller at -6.727 ns.
 *
 *  DDR is deliberately OFF, matching WithKU040Tweaks' default and therefore the
 *  memory arrangement every recorded accelerator number was measured against
 *  (181,513 LUT synth / 180,448 routed / WNS -8.289 ns). This is a probe whose
 *  only difference from that baseline is has_loop_conv, so the delta is
 *  attributable. It is not a deployable configuration.
 *
 *  Software cost, which is the real open question and is not settled by area:
 *  conv2d_s8 must move from `tiled_conv_auto` to the existing bit-exact
 *  `gemmini_q31_conv2d_s8_gemmini_im2col_full_C.c`, and maxpool2d_s8 from
 *  `tiled_conv_dw_auto` to `rvv/rvv_maxpool2d_s8_direct.c`. Those two are the
 *  only callers of LOOP_CONV_WS in the q31 kernel set, and conv2d is 30.2 ms of
 *  the 61.5 ms of serial op time in the flight3 profile.
 */
class Q31Ws32x32AccGemminiSaturnV128D128Fp16NoMvinScaleNoLoopConvKU040Config extends Config(
  new WithKU040Tweaks(freqMHz = 100) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.robotMpcParams.copy(
      useElementwiseFP64 = true,
      noPermute = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new gemmini.Q31GemminiConfig(
    gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(
      mvin_scale_args = None,
      has_loop_conv = false)) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/** The no-LoopConv point with the board's 2 GiB of DDR4 actually present.
 *
 *  The probe above runs on the 32 KiB scratchpad so its delta against the
 *  recorded baseline was attributable to has_loop_conv alone. That made the
 *  measurement clean and the result unquotable: it reported 174,795 LUT (72.1%)
 *  and WNS -4.169 ns for a configuration with no memory controller.
 *
 *  This adds the controller pair, whose cost is measured rather than estimated:
 *  10,972 + 10,971 LUT for the two MIGs plus 99 for the joining crossbar =
 *  22,042 LUT, 26,474 FF, 50 RAMB36. That projects to 196,837 LUT, 81.2% --
 *  over the 80% routability guideline, under the device. Whether it routes and
 *  what it costs in slack is the question; if it is too tight, dropping to one
 *  controller recovers 11,070 LUT for 1 GiB, which is ample (the FireSim run
 *  used 1.20 MB).
 *
 *  Two things to watch beyond LUTs. The MIG user interfaces run at 225 MHz in
 *  their own domain behind AsynchronousCrossing(8), and in the scalar build
 *  they closed by only +0.014 and +0.015 ns -- noise-level margin that extra
 *  congestion could erase. And DDR adds ~26,000 FF plus 50 RAMB36 of placement
 *  pressure to a design whose worst path is already 75% wire.
 *
 *  OSPI is deliberately still absent, so this is one variable against the probe.
 *  Adding it is a separately measured 1,149 LUT and 33 RAMB36 on a 36 MHz domain
 *  with +24 ns of slack.
 */
class Q31Ws32x32AccGemminiSaturnV128D128Fp16NoMvinScaleNoLoopConvDDRKU040Config extends Config(
  new WithKU040Tweaks(freqMHz = 100, ddr = true) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.robotMpcParams.copy(
      useElementwiseFP64 = true,
      noPermute = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new gemmini.Q31GemminiConfig(
    gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(
      mvin_scale_args = None,
      has_loop_conv = false)) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/** The deployable shape of the no-LoopConv point: camera periphery, one DDR4.
 *
 *  The two configurations above were probes, each isolating one variable. This
 *  is the one meant to fly, so it carries what the platform actually needs and
 *  nothing it does not:
 *
 *    + I2C and OSPI, without which the HM01B0 cannot be configured or read
 *    - the bank-46 DDR4 controller, leaving 1 GiB
 *
 *  Dropping the second controller is a memory-capacity decision that costs
 *  nothing real: the FireSim run of the flight mix touched 1.20 MB, so 1 GiB is
 *  three orders of magnitude of headroom. What it buys is the difference between
 *  a design that routes with margin and one that does not.
 *
 *  Measured on xcku040-sfva784-1-c with use_dsp, against the routed
 *  two-controller build (196,828 LUT, 81.20%, WNS -5.014 ns):
 *
 *    synth   188,842 LUT (77.9%)      routed  186,878 LUT (77.09%)
 *            128,701 FF, 74 RAMB36,           128,548 FF, 74 RAMB36,
 *            417 RAMB18, 1,314 DSP            417 RAMB18, 1,314 DSP
 *
 *  The area half went as expected and decomposes exactly: -12,497 LUT for mig_1,
 *  -99 for the joining crossbar, which degenerates to 1-in/1-out and is optimized
 *  away entirely, +1,121 for I2C and OSPI. It routes clean -- 0 failed nets, 34%
 *  global routing utilization, DRC identical to the two-controller build -- and
 *  it is the first accelerator configuration on this board under the 80%
 *  routability guideline. Power falls 3.351 W to 2.781 W.
 *
 *  The frequency half did not. WNS goes the wrong way, -5.014 to -5.670 ns
 *  (66.6 -> 63.8 MHz), and TNS with it, -140,906 to -175,312 ns. Freeing 10,000
 *  LUT and a whole clock domain bought no slack, which settles what the
 *  two-controller result left open: the accelerator's critical path is not
 *  congestion the DDR4 pair was causing. Every one of the ten worst paths is
 *  inside gemmini/ex_controller/cmd_q, on raddr_reg -- the ReservationStation
 *  read-address decode -- which is the wall the no-LoopConv work predicted, and
 *  it does not care how much of the device is empty around it. Fixing it means
 *  pipelining that decode, not making room.
 *
 *  Two things did behave as argued. The surviving MIG's user interface recovers
 *  from -0.252 to -0.106 ns, still failing but by less, so the pair was indeed
 *  squeezing itself. And OSPI is free: 36 MHz off the sensor's PCLK, +23.784 ns.
 */
class Q31Ws32x32AccGemminiSaturnV128D128Fp16NoMvinScaleNoLoopConvOspiSingleDDRKU040Config extends Config(
  new WithKU040OspiPeriphery ++
  new WithKU040Tweaks(freqMHz = 100, uartRxdPin = "C3", ddr = true, ddrControllers = 1) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.robotMpcParams.copy(
      useElementwiseFP64 = true,
      noPermute = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new gemmini.Q31GemminiConfig(
    gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(
      mvin_scale_args = None,
      has_loop_conv = false)) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/** The flight3 SoC: one DDR4 controller, camera periphery, LoopConv KEPT.
 *
 *  Identical to
 *  `Q31Ws32x32AccGemminiSaturnV128D128Fp16NoMvinScaleNoLoopConvOspiSingleDDRKU040Config`
 *  above except that `has_loop_conv` returns to its default of `true`. This is
 *  the deployable point of that pair, and the one to benchmark; the NoLoopConv
 *  sibling was a timing experiment, not a target.
 *
 *  WHY LOOPCONV COMES BACK. It was removed to buy frequency, and it did not:
 *  post-route WNS went the wrong way, -5.014 -> -5.670 ns (66.6 -> 63.8 MHz),
 *  because every one of the ten worst paths is inside
 *  `gemmini/ex_controller/cmd_q` on `raddr_reg` -- the ReservationStation
 *  read-address decode -- which does not care how much of the device is empty
 *  around it. So the workaround cost real throughput and bought no slack.
 *
 *  What it costs to keep, measured rather than projected: `has_loop_conv` is the
 *  only thing standing between `conv2d`/`maxpool` and the scalar fallback, and
 *  on the FireSim run recorded in docs/firesim-flight3-bringup.md **98.9% of
 *  dronet's runtime is in exactly those two op classes** (`conv2d_s8` alone is
 *  97.7%, 434,011,005 of 444,343,275 cycles). Giving that up to chase 2.8 MHz is
 *  a bad trade by roughly two orders of magnitude.
 *
 *  Area: its NoLoopConv sibling synthesizes at 188,842 LUT (77.9% of the
 *  xcku040's 242,400). The LoopConv delta is unsettled -- the one available A/B
 *  is confounded by an fpga-shells bump, see docs/ku040-codesign-cnn.md -- so the
 *  budget carries the pessimistic +5,435 LUT, putting this at roughly 194,000
 *  (80.1%). `rb area run --config Q31Ws32x32AccGemminiSaturnV128D128Fp16NoMvinScaleOspiSingleDDRKU040Config`
 *  settles it; that number is a projection until it does.
 */
class Q31Ws32x32AccGemminiSaturnV128D128Fp16NoMvinScaleOspiSingleDDRKU040Config extends Config(
  new WithKU040OspiPeriphery ++
  new WithKU040Tweaks(freqMHz = 100, uartRxdPin = "C3", ddr = true, ddrControllers = 1) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.robotMpcParams.copy(
      useElementwiseFP64 = true,
      noPermute = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new gemmini.Q31GemminiConfig(
    gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(
      mvin_scale_args = None)) ++   // has_loop_conv defaults to true
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/* ------------------------------------------------------------------------- *
 *  CNN navigation SoC.
 *
 *  Design points for the deployable navigation model -- a 1.55 M-param /
 *  6.45 M-MAC all-int8 CNN+LSTM -- rather than the DroNet + MPC + YOLOv8n mix
 *  the configs above were shaped for. Rationale, measurements and the area
 *  arithmetic are in docs/ku040-codesign-cnn.md; the short version:
 *
 *   - DroNet sustains 1.44 MACs/cycle on the 1024-PE mesh (0.14% utilization),
 *     and the 1024 `Tile` instances are only 9,641 LUT post-route. Mesh width is
 *     not a performance axis, so it is chosen for area and timing alone.
 *   - `vxufp`, Saturn's FP datapath, is 27,387 LUT -- 14.7% of the routed
 *     baseline -- and an all-int8 CNN uses none of it.
 *   - `has_loop_conv` is worth 3.08x on conv2d and 3.43x on maxpool against the
 *     scalar fallback, so the NoLoopConv timing workaround is reverted and the
 *     critical path attacked directly instead.
 *
 *  None of these has been synthesized. The LUT figures in the doc are projected
 *  from measured per-module numbers, which is exactly what `rb area` is for.
 * ------------------------------------------------------------------------- */

/** Q0.31 32x32 Gemmini + integer-only Saturn, camera periphery, one DDR4.
 *
 *  The reference CNN point, and the control against which the narrower mesh
 *  below is a single-variable probe.
 *
 *  Changes from
 *  Q31Ws32x32AccGemminiSaturnV128D128Fp16NoMvinScaleNoLoopConvOspiSingleDDRKU040Config:
 *
 *    - `intOnlyParams` instead of `robotMpcParams`, dropping `vxufp` entirely
 *      (measured -25,560 LUT, -108 DSP from the 202,835 -> 177,064 pair).
 *    - `WithRocketFPU16` dropped, so the standard f16/f32/f64 scalar FPU returns
 *      (measured +13,543 LUT). This is deliberate and not a regression:
 *      `mlp_control` is fp32 and is the only safety-critical workload in the
 *      mix, it loses vector acceleration when Saturn goes integer-only, and
 *      `WithRocketFPU16` is non-spec (Zfh without the F base) so it cannot carry
 *      it. Measured cost is 107,075 cycles = 1.68 ms at 63.8 MHz against a 20 ms
 *      deadline.
 *    - `has_loop_conv = true`, reverting the timing workaround. Its area delta is
 *      unsettled -- the one available A/B is confounded by an fpga-shells bump,
 *      see the doc -- so the budget assumes the pessimistic +5,435 LUT.
 *    - `has_training_convs = false`. Inference only.
 *    - `reservation_station_entries_ex` 16 -> 8 and `ex_queue_length` 8 -> 4.
 *      This is the cheap attempt at the 100 MHz wall: the routed worst path is
 *      gemmini/ex_controller/cmd_q/raddr_reg -> raddr_reg, 15.473 ns over 26
 *      logic levels, and the reservation station's dependency-check cone is what
 *      feeds that dequeue decision. Config-only, no RTL change. If it does not
 *      move WNS, the fix is to pipeline the decode.
 *
 *  Deliberately kept: `has_first_layer_optimizations`. It gates the
 *  `mvin_scale_pixel_repeater` (3,320 LUT) which sets
 *  `max_pixels_per_row = min(DIM/IC, kcols)`. The nav model's input is 60x90
 *  *greyscale*, so IC = 1 and this packs 3 pixels per mesh row on the first
 *  layer. It is not dead mvin-scale residue.
 *
 *  `noPermute` is carried over from the Fp16Full point and is still only
 *  validated for area, not function: it removes the vrgather / vcompress /
 *  vslide network, and if any kernel lowers to a slide or gather it will
 *  trap-illegal at runtime. The CNN nav model is the best case for it -- maxpool
 *  returns to Gemmini's depthwise path now that `has_loop_conv` is back, and the
 *  model has no upsample and no concat -- but that has to be checked against the
 *  emitted kernels before anything flies. Drop the `.copy(noPermute = true)` to
 *  buy the 2,101 LUT back if it bites.
 */
class CnnNavKU040Config extends Config(
  new WithKU040OspiPeriphery ++
  new WithKU040Tweaks(freqMHz = 100, uartRxdPin = "C3", ddr = true, ddrControllers = 1) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.intOnlyParams.copy(noPermute = true)) ++
  new gemmini.Q31GemminiConfig(
    gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(
      mvin_scale_args                = None,
      has_loop_conv                  = true,
      has_training_convs             = false,
      reservation_station_entries_ex  = 8,
      ex_queue_length                = 4)) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/** As above with a 16x16 mesh: the recommended CNN point.
 *
 *  256 PEs instead of 1024. At 0.14% measured mesh utilization this costs no
 *  throughput, and it takes width out of four blocks that scale with `DIM` and
 *  sit on or near the failing path: `transposer` (5,123 LUT), `tagq` (2,202),
 *  `mesh_cntl_signals_q` (1,909), and the `MeshWithDelays` skew network. It is
 *  the second config-only attempt at 100 MHz.
 *
 *  `acc_capacity` returns to 64 KB and `dma_buswidth` to 128. Both track the
 *  mesh: the 32x32 point needed 128 KB only because at that width 64 KB split
 *  the accumulator into 256x8 tiles that fall under Vivado's BRAM-efficiency
 *  threshold and demote to LUTRAM; at 16x16 the 512x8 split returns. Halving the
 *  DMA bus is what a 16-wide mesh column can drain in one cycle, and it should
 *  take `beatPacker` (10,964 LUT, the largest single item in `spad`) with it.
 *
 *  Projected ~160,500 LUT (66.2%) on the pessimistic `has_loop_conv` assumption,
 *  ~136,200 (56.2%) on the optimistic one, and ~569 DSP either way. That leaves
 *  room for the DroneLogic periphery below and margin over the 80% routability
 *  guideline, which no accelerator config on this board has had before.
 */
class CnnNavMesh16KU040Config extends Config(
  new WithKU040OspiPeriphery ++
  new WithKU040Tweaks(freqMHz = 100, uartRxdPin = "C3", ddr = true, ddrControllers = 1) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.intOnlyParams.copy(noPermute = true)) ++
  new gemmini.Q31GemminiConfig(
    gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(
      meshRows                       = 16,
      meshColumns                    = 16,
      acc_capacity                   = gemmini.CapacityInKilobytes(64),
      dma_buswidth                   = 128,
      mvin_scale_args                = None,
      has_loop_conv                  = true,
      has_training_convs             = false,
      reservation_station_entries_ex  = 8,
      ex_queue_length                = 4)) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/** The 16x16 CNN point plus the full drone periphery: the flight candidate.
 *
 *  Adds `WithRiskyBirdDronePeriphery` -- the PMW3901 SPI, control GPIO and the
 *  two motor PWM blocks -- on top of I2C, OSPI and 1 GiB of DDR4. This is the
 *  first configuration that carries an accelerator *and* everything the airframe
 *  needs, which is the whole point of the area headroom the mesh cut buys.
 *
 *  Two caveats carried from RocketKU040DroneLogicConfig, neither introduced
 *  here. The SPI/GPIO/PWM package pins are still unbound pending the custom
 *  base-connector mapping and voltage domains, so this builds and measures but
 *  does not deploy. And no DroneLogic config has ever been through `rb area`, so
 *  unlike every other number in this family the periphery's cost is unknown
 *  rather than estimated -- measuring it is a reason to build this point early.
 */
class CnnNavDroneLogicKU040Config extends Config(
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithKU040OspiPeriphery ++
  new WithKU040Tweaks(freqMHz = 100, uartRxdPin = "C3", ddr = true, ddrControllers = 1) ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.intOnlyParams.copy(noPermute = true)) ++
  new gemmini.Q31GemminiConfig(
    gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(
      meshRows                       = 16,
      meshColumns                    = 16,
      acc_capacity                   = gemmini.CapacityInKilobytes(64),
      dma_buswidth                   = 128,
      mvin_scale_args                = None,
      has_loop_conv                  = true,
      has_training_convs             = false,
      reservation_station_entries_ex  = 8,
      ex_queue_length                = 4)) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

class NoCoresKU040Config extends Config(
  new WithKU040Tweaks ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new chipyard.NoCoresConfig)

/** FP16-Saturn variant of RocketKU040DroneDualConfig, so the DroNet int8/fp16 HYBRID
 *  can run its final layer (relu/linear/bn/add _f16 tail) on the vector unit.
 *
 *  Deltas vs RocketKU040DroneDualConfig (everything else byte-identical):
 *   - hart0 Saturn: intOnlyParams -> robotMpcParams (noFP64/noFP32, keeps FP16 vector FMA).
 *   - scalar FPU: FP32-only applied to hart1 ONLY; hart0 gets WithRocketFPU16
 *     (minFLen=16, fLen=16 at the selected Rocket pin), so it cannot execute scalar FP32.
 *     FcRoCC is a separate custom accelerator; its presence does not restore scalar FP32.
 *   - hart1 (Gemmini int8 core) unchanged: FP32-only scalar, no Saturn.
 *
 *  AREA RISK is the whole point of building this: intOnly dual already fights FIT (32x32
 *  Gemmini ~129k LUT; single-core FP16 Saturn+Gemmini was 177k LUT / 73%). Adding the FP16
 *  vector datapath to the dual is tighter still -- run rb-area FIRST; fall back to
 *  RocketKU040DroneDual16 (16x16 Gemmini) if it overruns.
 *
 *  BUILD:
 *    make -C fpga SUB_PROJECT=ku040 CONFIG=RocketKU040DroneDualFp16Config RB_ATTRS=dsp rb-area
 *    make -C fpga SUB_PROJECT=ku040 CONFIG=RocketKU040DroneDualFp16Config RB_ATTRS=dsp rb-impl
 */
class RocketKU040DroneDualFp16Config extends Config(
  // ---- riskybird carrier peripherals + pins (same as RocketKU040DroneDualConfig) ----
  new WithKU040UARTTiedOff(uartNo = 1) ++
  new chipyard.config.WithUART(address = 0x10021000) ++
  new WithKU040PWM ++
  new chipyard.iobinders.WithPWMPunchthrough ++
  new WithKU040SPI ++
  new WithKU040GPIO ++
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithKU040Ospi(
    dataPins = Seq("N7", "M5", "L5", "J1", "H1", "J4", "J5", "M7"),
    pclkPin = "L8", fvldPin = "L1", lvldPin = "K2", intrPin = "K1", mclkPin = "N8", trigPin = "M1") ++
  new chipyard.iobinders.WithOspiPunchthrough ++
  new ospi.WithOspiCapture ++
  new WithKU040I2C(sclPin = "AA12", sdaPin = "AB12", ioStandard = "LVCMOS33") ++
  new chipyard.config.WithI2C ++
  // ---- DDR + clocking + console uart0/debug jtag on Bank 64 (riskybird carrier) ----
  new WithKU040Tweaks(ddr = true, ddrControllers = 1,
    uartRxdPin = "AH12", uartTxdPin = "AG12", ioStandard = "LVCMOS33",
    jtagTckPin = "AA10", jtagTmsPin = "Y12", jtagTdiPin = "Y10", jtagTdoPin = "Y11") ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  // ---- TACIT trace (both tiles); encoder rightmost of the group; internal DSC BP OFF ----
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder(useBP = false) ++
  // ---- scalar FPU: hart1 FP32-only (LEFT so it wins); hart0 scalar FP16-only ----
  new WithFP32OnlyFPUOnTiles(1) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  // ---- per-hart RoCC: FcRoCC (custom0) -> hart 0, Gemmini 32x32 (custom3) -> hart 1 ----
  new chipyard.config.WithMultiRoCC ++
  new chipyard.config.WithMultiRoCCFromBuildRoCC(0) ++
  new chipyard.fc.WithFcRoCC ++
  new chipyard.config.WithMultiRoCCFromBuildRoCC(1) ++
  new gemmini.Q31Ws32x32AccGemminiConfig ++
  // ---- Saturn V128D128 FP16 vector unit (robotMpc) -> hart 0 only ----
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.robotMpcParams.copy(useElementwiseFP64 = true),
    cores = Some(Seq(0))) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(2) ++   // tile0=hart0 (Saturn+Fc), tile1=hart1 (Gemmini)
  new chipyard.config.AbstractConfig)

/** 16x16-Gemmini + FP16-Saturn dual config: the DEPLOYABLE-FIT variant of
 *  RocketKU040DroneDualFp16Config. The 32x32-Gemmini FP16 dual synthesized at 99.86%
 *  CLB LUTs (unroutable); swapping to the proven 16x16 DroNet mesh frees ~70k LUT,
 *  leaving room for the FP16 vector datapath. FP16 deltas identical to the 32x32 variant.
 *  NOTE: 16x16 = Gemmini DIM=16 -> the modelblaster model MUST be generated with the
 *  DIM=16 gemmini params (NOT the DIM=32 header), or the DIM-mismatch garbage bug returns.
 *  BUILD: make -C fpga SUB_PROJECT=ku040 CONFIG=RocketKU040DroneDual16Fp16Config RB_ATTRS=dsp rb-area
 */
class RocketKU040DroneDual16Fp16Config extends Config(
  new WithKU040UARTTiedOff(uartNo = 1) ++
  new chipyard.config.WithUART(address = 0x10021000) ++
  new WithKU040PWM ++
  new chipyard.iobinders.WithPWMPunchthrough ++
  new WithKU040SPI ++
  new WithKU040GPIO ++
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithKU040Ospi(
    dataPins = Seq("N7", "M5", "L5", "J1", "H1", "J4", "J5", "M7"),
    pclkPin = "L8", fvldPin = "L1", lvldPin = "K2", intrPin = "K1", mclkPin = "N8", trigPin = "M1") ++
  new chipyard.iobinders.WithOspiPunchthrough ++
  new ospi.WithOspiCapture ++
  new WithKU040I2C(sclPin = "AA12", sdaPin = "AB12", ioStandard = "LVCMOS33") ++
  new chipyard.config.WithI2C ++
  new WithKU040Tweaks(ddr = true, ddrControllers = 1,
    uartRxdPin = "AH12", uartTxdPin = "AG12", ioStandard = "LVCMOS33",
    jtagTckPin = "AA10", jtagTmsPin = "Y12", jtagTdiPin = "Y10", jtagTdoPin = "Y11") ++
  new chipyard.config.WithBroadcastManager ++ // no l2
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder(useBP = false) ++
  new WithFP32OnlyFPUOnTiles(1) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new chipyard.config.WithMultiRoCC ++
  new chipyard.config.WithMultiRoCCFromBuildRoCC(0) ++
  new chipyard.fc.WithFcRoCC ++
  new chipyard.config.WithMultiRoCCFromBuildRoCC(1) ++
  new gemmini.Q31GemminiConfig(chipyard.fpga.arty200t.RbArty200TGemmini.mesh16) ++  // 16x16
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.robotMpcParams.copy(useElementwiseFP64 = true),
    cores = Some(Seq(0))) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(2) ++
  new chipyard.config.AbstractConfig)

/** 32x32 int Gemmini + SMALLER FP16 Saturn (DLEN=64) dual config. The D128 FP16 Saturn
 *  blew area (99.86% LUT, unroutable); halving the physical vector datapath (DLEN 128->64,
 *  VLEN stays 128 so the rvv_f16 kernels are byte-identical) shrinks the FP16 datapath while
 *  KEEPING the 32x32 Gemmini throughput. The int8 DroNet RVV ops (bn/add/pool) run ~2x
 *  narrower but they are a small fraction of runtime (convs dominate on Gemmini); the fp16
 *  tail (relu_f16+linear_f16, 2048 MACs) is trivial at any width. Precedent: MinSaturnRobotMpc
 *  sim configs use WithRocketVectorUnit(128, 64, robotMpcParams).
 *  BUILD: make -C fpga SUB_PROJECT=ku040 CONFIG=RocketKU040DroneDualFp16D64Config RB_ATTRS=dsp rb-area
 */
class RocketKU040DroneDualFp16D64Config extends Config(
  new WithKU040UARTTiedOff(uartNo = 1) ++
  new chipyard.config.WithUART(address = 0x10021000) ++
  new WithKU040PWM ++
  new chipyard.iobinders.WithPWMPunchthrough ++
  new WithKU040SPI ++
  new WithKU040GPIO ++
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithKU040Ospi(
    dataPins = Seq("N7", "M5", "L5", "J1", "H1", "J4", "J5", "M7"),
    pclkPin = "L8", fvldPin = "L1", lvldPin = "K2", intrPin = "K1", mclkPin = "N8", trigPin = "M1") ++
  new chipyard.iobinders.WithOspiPunchthrough ++
  new ospi.WithOspiCapture ++
  new WithKU040I2C(sclPin = "AA12", sdaPin = "AB12", ioStandard = "LVCMOS33") ++
  new chipyard.config.WithI2C ++
  new WithKU040Tweaks(ddr = true, ddrControllers = 1,
    uartRxdPin = "AH12", uartTxdPin = "AG12", ioStandard = "LVCMOS33",
    jtagTckPin = "AA10", jtagTmsPin = "Y12", jtagTdiPin = "Y10", jtagTdoPin = "Y11") ++
  new chipyard.config.WithBroadcastManager ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder(useBP = false) ++
  new WithFP32OnlyFPUOnTiles(1) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new chipyard.config.WithMultiRoCC ++
  new chipyard.config.WithMultiRoCCFromBuildRoCC(0) ++
  new chipyard.fc.WithFcRoCC ++
  new chipyard.config.WithMultiRoCCFromBuildRoCC(1) ++
  new gemmini.Q31Ws32x32AccGemminiConfig ++
  new saturn.rocket.WithRocketVectorUnit(128, 64,
    saturn.common.VectorParams.robotMpcParams.copy(useElementwiseFP64 = true),
    cores = Some(Seq(0))) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(2) ++
  new chipyard.config.AbstractConfig)
