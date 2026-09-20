
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
