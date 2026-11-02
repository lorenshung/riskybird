package chipyard

import org.chipsalliance.cde.config.Config

// robotMpc Saturn + Q31 Gemmini(mesh16) Verilator DUT for the switch_handle wild-store
// waveform hunt (H1b: a Gemmini-DMA mvout committing to z_main_thread+0x128 across a
// timer preemption while a DroNet conv DMA is in-flight). Mirrors
// MinSaturnV128D64RobotMpcRocketConfig (full FP Saturn so the decode table doesn't
// need the intOnly strip that OOMs Verilator) + the FcRoCC FPGA build's Gemmini
// (Q31, 16x16 mesh, LoopConv on, mesh16 params byte-for-byte from RbArty200TGemmini).
// espresso on PATH (.conda-env/bin) minimizes the FP decode table at elaboration.
class MinSaturnV128D64RobotMpcGemminiRocketConfig extends Config(
  new gemmini.Q31GemminiConfig(
    gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(
      meshRows                       = 16,
      meshColumns                    = 16,
      acc_capacity                   = gemmini.CapacityInKilobytes(64),
      dma_buswidth                   = 128,
      mvin_scale_args                = None,
      has_loop_conv                  = true,
      has_training_convs             = false,
      reservation_station_entries_ex = 8,
      ex_queue_length                = 4)) ++
  new saturn.rocket.WithRocketVectorUnit(128, 64,
    saturn.common.VectorParams.robotMpcParams.copy(useElementwiseFP64 = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)
