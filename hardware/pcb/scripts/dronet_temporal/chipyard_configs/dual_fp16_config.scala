
/** FP16-Saturn variant of RocketKU040DroneDualConfig, so the DroNet int8/fp16 HYBRID
 *  can run its final layer (relu/linear/bn/add _f16 tail) on the vector unit.
 *
 *  Deltas vs RocketKU040DroneDualConfig (everything else byte-identical):
 *   - hart0 Saturn: intOnlyParams -> robotMpcParams (noFP64/noFP32, keeps FP16 vector FMA).
 *   - scalar FPU: FP32-only applied to hart1 ONLY; hart0 gets WithRocketFPU16 (minFLen=16),
 *     which ADDS the FP16 scalar FMA (~489 LUT) while KEEPING fp32 for the FcRoCC MPC path.
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
  // ---- scalar FPU: hart1 FP32-only (LEFT so it wins); hart0 gets FP16 (minFLen=16) + keeps fp32 ----
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
