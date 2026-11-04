package chipyard

import org.chipsalliance.cde.config.Config

// Minimal INTEGER-ONLY Rocket + Saturn V128D64 verilator config for RTL-sim
// validation of the async-interrupt-during-vmem-op mis-resume fix
// (branch saturn-vmem-irq-defer / commit 6f53ddc6c). intOnlyParams strips the FP
// functional units -> small decode table (avoids the QMCMinimizer heap blowup) while
// keeping the EXACT frontend fault-check (PipelinedFaultCheck/IterativeFaultCheck) +
// store path (StoreSegmenter/AddrGen) where the bug lives. The test uses only integer
// vector (vsetvli/vle8/vsse8), so FP lanes are unnecessary. verilator sim only.
class MinSaturnV128D64RocketConfig extends Config(
  new saturn.rocket.WithRocketVectorUnit(128, 64, saturn.common.VectorParams.intOnlyParams) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)
