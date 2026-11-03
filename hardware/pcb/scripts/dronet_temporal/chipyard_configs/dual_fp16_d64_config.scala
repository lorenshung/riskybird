
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
