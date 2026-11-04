package chipyard

import org.chipsalliance.cde.config.{Config}

/* ------------------------------------------------------------------------- *
 *  Verilator-simulatable SoCs for per-op cycle profiling.
 *
 *  These exist because the per-op profile database has no entry for a 16x16
 *  Gemmini -- every profile in the tree was captured on a 1024-cell mesh -- and
 *  neither of the other two runners can produce one. Spike has no timing model
 *  and cannot execute Gemmini's RoCC instructions without libgemmini.so, which
 *  is not built here; FireSim needs an FPGA host. Verilator is cycle-exact on the
 *  same RTL and needs only a CPU. See docs/verilator-per-op-profiling.md.
 *
 *  WHY NOT THE FireSimRiskyBird*TargetConfig CLASSES: they instantiate
 *  firesim.lib.bridges.RationalClockBridge, whose Verilog exists only inside a
 *  FireSim compile. They elaborate -- which is all the fingerprint extraction
 *  needed -- but Verilator fails to find the module and stops.
 *
 *  WHY NO SATURN: the workload these profile is the gemmini_q31 kernel set,
 *  which contains no RVV intrinsics; the ELF's own DTS reports `rv64gc`. So the
 *  vector unit is never issued to and its absence cannot change a kernel's cycle
 *  count directly. It is dropped because it roughly doubles Verilator build time
 *  and simulation cost for no signal.
 *
 *  That is an assumption with one honest caveat: Saturn attaches to the Rocket
 *  tile, so its presence could perturb core timing even when unused. For the
 *  mesh-width comparison the effect cancels -- both configs below are Saturn-less
 *  -- but when validating against the archived FireSim profile, which *did* carry
 *  a V128D128 Saturn, it is a real divergence. If validation shows a gap, adding
 *  the vector unit back is the first thing to try.
 *
 *  Gemmini parameters are set by `.copy()` at this site rather than in the
 *  gemmini submodule, matching what fpga/src/main/scala/ku040/Configs.scala does
 *  and for the same two reasons: existing measurements of the other Q0.31 configs
 *  stay reproducible, and gemmini is pinned to ucb-bar/gemmini, which RiskyBird
 *  does not own and must not need a local edit to.
 * ------------------------------------------------------------------------- */

/** Simulation twin of CnnNavKU040Config's Gemmini: Q0.31, 32x32, LoopConv on.
 *
 *  Every Gemmini knob matches the KU040 config so a profile captured here
 *  transfers to it. The differences from that config are the ones that cannot
 *  affect these kernels: no Saturn (see above), no KU040 shell, no DDR4 MIG, and
 *  Chipyard's TestHarness in place of KU040Harness.
 */
class CnnNavSimMesh32Config extends Config(
  new gemmini.Q31GemminiConfig(
    gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(
      mvin_scale_args                = None,
      has_loop_conv                  = true,
      has_training_convs             = false,
      reservation_station_entries_ex = 8,
      ex_queue_length                = 4)) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

/** Simulation twin of CnnNavMesh16KU040Config's Gemmini: Q0.31, 16x16.
 *
 *  The point of the whole exercise. `acc_capacity` and `dma_buswidth` track the
 *  mesh for the same reasons given in the KU040 config: 128 bits is what a
 *  16-wide int8 column drains per cycle, and 64 KB restores the BRAM-friendly
 *  512x8 accumulator split at DIM=16.
 *
 *  Paired against CnnNavSimMesh32Config this isolates mesh width alone, which is
 *  the variable no existing profile covers.
 */
class CnnNavSimMesh16Config extends Config(
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
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)
