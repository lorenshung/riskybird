package chipyard

import org.chipsalliance.cde.config.Config

// FULL-Saturn (robotMpc: fp16 vector + FP16 scalar FPU) V128D64 verilator DUT for the
// wild-vse8.v-store waveform hunt (V-race #2 / decoupled-VMU RTL bug). Unlike the
// intOnly MinSaturnV128D64RocketConfig (which strips FP to dodge the QMCMinimizer heap
// blowup), the trigger needs the FULL Saturn -- so robotMpcParams (fp16 kept,
// useElementwiseFP64) + WithRocketFPU16. espresso on PATH (.conda-env/bin) does the
// decode-table minimization so the larger FP decode table doesn't OOM at elaboration.
// Same frontend fault-check (PipelinedFaultCheck/IterativeFaultCheck) + store path
// (StoreSegmenter/AddrGen) where the bug lives. chipyard TestHarness, verilator sim only.
class MinSaturnV128D64RobotMpcRocketConfig extends Config(
  new saturn.rocket.WithRocketVectorUnit(128, 64,
    saturn.common.VectorParams.robotMpcParams.copy(useElementwiseFP64 = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)
