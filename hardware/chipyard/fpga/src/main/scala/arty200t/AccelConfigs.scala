// See LICENSE for license details.
//
// RiskyBird ML-accelerator SoC configs for the Arty-200T (XC7A200T).
//
// These ADD ML accelerators (Gemmini systolic matmul + Saturn RVV vector unit)
// on top of the existing full-drone SoC. The drone periphery + DDR set is a
// byte-for-byte mirror of RocketArty200TDroneFullDDRDmaUartConfig (DDR3 + I2C +
// camera-OSPI-DMA + PMW3901 SPI + control GPIO + 2x motor PWM + ESP UART on
// E13/F14 + mem->uart1 TX DMA), so the drone firmware (sensors, camera DMA,
// UART DMA telemetry) still works unchanged. Only accelerators are added.
//
// Composition mirrors fpga/src/main/scala/ku040/Configs.scala's accelerator
// design points: chipyard.RocketConfig (= WithNHugeCores(1) ++ AbstractConfig)
// is replaced by
//   <accel fragments> ++ WithSystemBusWidth(128) ++ WithNHugeCores(1) ++ AbstractConfig
// with WithBroadcastManager (no L2) kept from the drone base. 50 MHz is kept
// (the 200T drone runs at 50 MHz).
//
// DSP INFERENCE: the small signed multipliers in Gemmini's MacUnit and Saturn's
// OPU cells map to LUTs by default. Build these with the RiskyBird use_dsp
// attribute injection (riskybird/tcl/rb_attributes.tcl, RB_ATTRS=dsp) so they
// map onto the XC7A200T's 740 DSP48E1 slices instead. Order: elaborate -> inject
// -> synth/bitstream.
//
// SCALAR FPU / FIRMWARE CONTRACT: the FP16 and NoFpu variants prune the Rocket
// scalar FPU (WithRocketFPU16 = non-spec Zfh-without-F, or WithoutFPU). They
// REQUIRE an fp16/soft-float firmware build -- the fast loops (PID, state
// estimation) run in fp16 or soft-float, and TinyMPC is validated in FP16. The
// full-FPU variant keeps hardware fp32 for the existing firmware, for LUT
// comparison.

package chipyard.fpga.arty200t

import org.chipsalliance.cde.config._

// ---------------------------------------------------------------------------
// Shared drone periphery + DDR base.
//
// Identical to the tail of RocketArty200TDroneFullDDRDmaUartConfig above
// chipyard.RocketConfig. Factored out so every accelerator config below shares
// exactly the same SoC contract (same PLIC source numbering, same pins).
// ---------------------------------------------------------------------------
class WithArty200TDroneFullDmaUartBase extends Config(
  new uartdma.WithUartTxDma(address = 0x10022000L, uartTxDataAddr = 0x10021000L) ++  // mem->uart1 TX DMA
  new WithArty200TUART("E13", "F14", uartNo = 1) ++       // ESP UART = uart1 on E13/F14
  new chipyard.config.WithUART(address = 0x10021000) ++   // add uart1 (console uart0 stays 0x10020000)
  new WithArty200TPWM ++
  new chipyard.iobinders.WithPWMPunchthrough ++
  new WithArty200TSPI ++                                  // PMW3901 flow SPI: sck=F16 mosi=E17 miso=E16
  new WithArty200TGPIO ++                                 // gpio0 bit0=CS=D16, bit1=A20, bit2=A21
  new chipyard.config.WithRiskyBirdDronePeriphery ++
  new WithArty200TOspiDmaPeriphery ++
  new WithArty200TTweaks(ddr = true, uartTsi = false) ++
  new chipyard.config.WithBroadcastManager)              // no l2

// ---------------------------------------------------------------------------
// Shared Gemmini design points (Q0.31 weight-stationary, RiskyBird-tuned).
// ---------------------------------------------------------------------------

// 16x16 mesh (256 PEs). acc_capacity 64 KB + dma_buswidth 128 track the mesh
// width, mirroring CnnNavMesh16KU040Config. The combined attempt.
object RbArty200TGemmini {
  val mesh16 = gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(
    meshRows                       = 16,
    meshColumns                    = 16,
    acc_capacity                   = gemmini.CapacityInKilobytes(64),
    dma_buswidth                   = 128,
    mvin_scale_args                = None,   // drop the fp32 mvin scale (~20ns path, dead for this workload)
    has_loop_conv                  = true,
    has_training_convs             = false,
    reservation_station_entries_ex = 8,
    ex_queue_length                = 4)

  // 32x32 mesh (1024 PEs). Default acc_capacity of q31Ws32x32AccConfig (128 KB,
  // BRAM-mappable at this width). NOTE: measured 185% LUT / 100.5% DSP on the
  // XC7A200T -- 1024 PEs exceed the 740 DSP48E1 budget, so ~284 PE multipliers
  // spill to LUTs and the design is infeasible. Kept for the record.
  val mesh32 = gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(
    mvin_scale_args                = None,
    has_loop_conv                  = true,
    has_training_convs             = false,
    reservation_station_entries_ex = 8,
    ex_queue_length                = 4)

  // 24x24 mesh (576 PEs). INVALID: Gemmini requires power-of-2 mesh dims
  // (elaboration throws "I_TILE_BYTE_WIDTH is not power of 2: 24"). Kept only for
  // the record. MUST be `lazy` -- as a plain `val` it is evaluated during this
  // object's initialization, which crashes the elaboration of EVERY config that
  // touches RbArty200TGemmini (including the valid mesh16/mesh32 ones). `lazy`
  // defers the (throwing) evaluation to the point of use, i.e. only the unused
  // RocketArty200TDroneGemmini24Config would ever trip it.
  lazy val mesh24 = gemmini.GemminiQ31WsConfigs.q31Ws32x32AccConfig.copy(
    meshRows                       = 24,
    meshColumns                    = 24,
    mvin_scale_args                = None,
    has_loop_conv                  = true,
    has_training_convs             = false,
    reservation_station_entries_ex = 8,
    ex_queue_length                = 4)
}

// ===========================================================================
// PRIMARY combined target: 16x16 Gemmini + Saturn V128D64 (fp16-only) +
// FP16-only scalar FPU. The tightest fit, and the one meant to carry both
// accelerators on the XC7A200T. Requires fp16/soft-float firmware.
// ===========================================================================
class RocketArty200TDroneGemminiSaturnFp16Config extends Config(
  new WithArty200TDroneFullDmaUartBase ++
  new saturn.rocket.WithRocketVectorUnit(128, 64,
    saturn.common.VectorParams.robotMpcParams.copy(useElementwiseFP64 = true)) ++
  new freechips.rocketchip.rocket.WithRocketFPU16 ++     // scalar FPU: FP16-only (NON-SPEC Zfh-without-F)
  new gemmini.Q31GemminiConfig(RbArty200TGemmini.mesh16) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

// Same accelerators, scalar FPU RETAINED (full f16/f32/f64). For the LUT-delta
// comparison against the FP16 primary; keeps hardware fp32 for existing firmware.
class RocketArty200TDroneGemminiSaturnConfig extends Config(
  new WithArty200TDroneFullDmaUartBase ++
  new saturn.rocket.WithRocketVectorUnit(128, 64,
    saturn.common.VectorParams.robotMpcParams.copy(useElementwiseFP64 = true)) ++
  new gemmini.Q31GemminiConfig(RbArty200TGemmini.mesh16) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

// Maximum LUT savings: NO scalar FPU + integer-only Saturn. NOTE this is a
// different accelerator point, not just an FPU cut: robotMpcParams (fp16 vector)
// sets vfh=true, which rocket-chip requires a scalar fp16 FPU for, so a true
// fpu=None build must also drop vector FP (intOnlyParams -> noFP -> vfLen=0).
// int8 CNN reaches Gemmini via RoCC and Saturn via integer lanes; fp32 control
// runs in soft-float. Area probe for the "how much does the FPU cost" question.
class RocketArty200TDroneGemminiSaturnNoFpuConfig extends Config(
  new WithArty200TDroneFullDmaUartBase ++
  new saturn.rocket.WithRocketVectorUnit(128, 64,
    saturn.common.VectorParams.intOnlyParams.copy(noPermute = true)) ++
  new freechips.rocketchip.rocket.WithoutFPU ++
  new gemmini.Q31GemminiConfig(RbArty200TGemmini.mesh16) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

// ===========================================================================
// Fallback split configs (build these if the combined does not fit). Each has
// the whole fabric to itself, so each carries a LARGER datapath and keeps the
// full scalar FPU (hardware fp32 for the existing drone firmware).
// ===========================================================================

// drone + Gemmini 32x32 (1024 PEs), no Saturn. INFEASIBLE on the 200T
// (185% LUT / 100.5% DSP); kept for the record.
class RocketArty200TDroneGemminiConfig extends Config(
  new WithArty200TDroneFullDmaUartBase ++
  new gemmini.Q31GemminiConfig(RbArty200TGemmini.mesh32) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

// drone + Gemmini 24x24 (576 PEs), no Saturn. INVALID: Gemmini requires
// power-of-2 mesh dims (elaboration: "I_TILE_BYTE_WIDTH is not power of 2: 24").
// Kept for the record; use mesh16 (below) as the deployable Gemmini fallback.
class RocketArty200TDroneGemmini24Config extends Config(
  new WithArty200TDroneFullDmaUartBase ++
  new gemmini.Q31GemminiConfig(RbArty200TGemmini.mesh24) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

// drone + Gemmini 16x16 (256 PEs), no Saturn. THE DEPLOYABLE Gemmini-only
// fallback: 32x32 is infeasible and 24x24 is invalid (non-power-of-2), so 16x16
// is the largest Gemmini that fits the 200T. ~65% LUT (combined minus Saturn),
// routes with wide margin.
class RocketArty200TDroneGemmini16Config extends Config(
  new WithArty200TDroneFullDmaUartBase ++
  new gemmini.Q31GemminiConfig(RbArty200TGemmini.mesh16) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

// drone + Saturn V128D128 (fp16-only vector), no Gemmini. Full scalar FPU kept.
class RocketArty200TDroneSaturnConfig extends Config(
  new WithArty200TDroneFullDmaUartBase ++
  new saturn.rocket.WithRocketVectorUnit(128, 128,
    saturn.common.VectorParams.robotMpcParams.copy(useElementwiseFP64 = true)) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

// ===========================================================================
// LOWER-FABRIC-CLOCK deploy variants. The large accelerator configs miss the
// 50 MHz setup constraint (Saturn V128D128 split: WNS -2.46 ns @ 50 MHz => Fmax
// ~44.5 MHz; combined FP16: a larger miss). These re-target a lower fabric clock
// using the SAME knob WithArty200TTweaks(freqMHz=...) applies --
// WithHarnessBinderClockFreqMHz + WithUniformBusFrequencies -- prepended so they
// override (leftmost wins in CDE) the 50 MHz default baked into the shared base.
// Everything else (SoC pin contract, accelerators, DSP-slice injection) is
// identical to the 50 MHz parent config. The DDR3 MIG runs in its own clock
// domain across an async crossing, so lowering the fabric clock only relaxes the
// fabric-side timing; the MIG UI clock is unchanged.
//
// FIRMWARE CONTRACT: a lower fabric clock changes clock-derived firmware
// constants -- the SiFive UART baud divisor (div = fabric_hz/baud - 1) and the
// timer cycles-per-second -- so the FC image must be rebuilt to match. The
// generated DTS carries the new bus/timebase frequency automatically
// (dtsFrequency), so the divisor/timebase follow from the DTS.
// ===========================================================================

// Saturn V128D128 split at 40 MHz (25.0 ns). 50 MHz missed setup by -2.46 ns.
class RocketArty200TDroneSaturn40Config extends Config(
  new chipyard.harness.WithHarnessBinderClockFreqMHz(40) ++
  new chipyard.config.WithUniformBusFrequencies(40) ++
  new RocketArty200TDroneSaturnConfig)

// Gemmini-16 split at 40 MHz (25.0 ns). 50 MHz missed setup by -4.62 ns (the Q0.31
// Gemmini MAC/DSP path is critical-path-bound, not congestion; only 70.8% LUT).
// Fmax ~40.6 MHz => closes at 40 MHz, matching Saturn-40 for a common-40 MHz split pair.
class RocketArty200TDroneGemmini16At40Config extends Config(
  new chipyard.harness.WithHarnessBinderClockFreqMHz(40) ++
  new chipyard.config.WithUniformBusFrequencies(40) ++
  new RocketArty200TDroneGemmini16Config)

// Combined Gemmini16 + Saturn V128D64 + FP16 scalar FPU at 35 MHz (28.6 ns).
// Only meaningful if the 50 MHz build ROUTED but missed timing -- a routability
// failure (congestion) is not fixed by a lower clock.
class RocketArty200TDroneGemminiSaturnFp16At35Config extends Config(
  new chipyard.harness.WithHarnessBinderClockFreqMHz(35) ++
  new chipyard.config.WithUniformBusFrequencies(35) ++
  new RocketArty200TDroneGemminiSaturnFp16Config)

// ===========================================================================
// INTEGER-ONLY combined target: 16x16 Gemmini + int-only Saturn V128D64 +
// the FULL standard scalar FPU. The arty200t port of CnnNavMesh16KU040Config's
// choice, and the shape an int8 CNN actually wants.
//
// WHY THIS EXISTS. RocketArty200TDroneGemminiSaturnFp16Config pairs an FP16
// vector unit with an FP16-only scalar FPU, and for an int8 DroNet both halves
// are dead weight that costs twice over:
//
//   - vxufp, Saturn's FP datapath, is silicon the workload never issues to. It
//     measured 27,387 LUT at D128 on the KU040; at D64 call it half that, off a
//     design sitting at 93.11%.
//   - WithRocketFPU16 is what forces the whole image to CONFIG_FPU=n, because
//     rocket-chip advertises "f" in riscv,isa for any FPU while that one
//     implements only Zfh. CONFIG_FPU=n forces a soft-float ABI, and the
//     soft-float ABI is what currently blocks the RVV kernels and leaves
//     Saturn idle. An accelerator switched off by its own floating-point unit.
//
// intOnlyParams removes the first. Keeping the standard f16/f32/f64 FPU removes
// the second: an ordinary hard-float image, so gemmini_q31_rvv builds unchanged
// and Saturn runs the integer vector kernels it was put there for.
//
// The FPU is kept for the same reason CnnNavKU040Config keeps it, and that
// reasoning is the airframe's rather than the model's: the flight controller is
// fp32 and is the only safety-critical workload in this mix. WithRocketFPU16
// cannot carry it -- Zfh without the F base is not a thing standard soft-float
// can fall back to -- so a shell that wants to both fly and infer needs the real
// FPU. It costs a measured +13,543 LUT on the KU040 against the FP16 unit.
//
// intOnlyParams also sets noFP, hence vfLen = 0 and vfh = false, so nothing here
// asks for a scalar FP16 unit the way robotMpcParams does. The coupling
// RocketArty200TDroneGemminiSaturnNoFpuConfig documents runs the other way: an
// fp16 VECTOR needs a scalar fp16 FPU, an integer vector needs nothing.
//
// PROJECTION, NOT MEASUREMENT. The FP16 combined shell routes at 93.11% LUT /
// 555 DSP / 244 BRAM. Dropping vxufp should take this well clear of that, and
// the FPU swap adds some of it back. Whether the result closes faster than
// 35 MHz is the open question -- `rb area run --board arty200t --config
// RocketArty200TDroneGemminiSaturnIntConfig --attrs dsp` is what settles it.
//
// noPermute is carried over from the KU040 int-only points and is validated for
// AREA only: it removes the vrgather / vcompress / vslide network, and a kernel
// that lowers to a slide or gather will trap-illegal at run time. Check the
// emitted kernels before flying anything.
// ===========================================================================
class RocketArty200TDroneGemminiSaturnIntConfig extends Config(
  new WithArty200TDroneFullDmaUartBase ++
  new saturn.rocket.WithRocketVectorUnit(128, 64,
    saturn.common.VectorParams.intOnlyParams.copy(noPermute = true)) ++
  new gemmini.Q31GemminiConfig(RbArty200TGemmini.mesh16) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

/** The integer combined shell at 40 MHz, matching the Gemmini-only deploy target.
 *
 *  50 MHz is the default the shared base carries and no combined shell has ever
 *  closed there. 40 is where RocketArty200TDroneGemmini16At40Config closes, so
 *  running both at the same frequency makes the pair a clean A/B: identical
 *  clock, identical periphery, Saturn the only variable.
 */
class RocketArty200TDroneGemminiSaturnIntAt40Config extends Config(
  new chipyard.harness.WithHarnessBinderClockFreqMHz(40) ++
  new chipyard.config.WithUniformBusFrequencies(40) ++
  new RocketArty200TDroneGemminiSaturnIntConfig)

/** The integer combined shell at 35 MHz: the fallback 4aeeae37 named.
 *
 *  GemminiSaturnIntAt40 routes at WNS -0.067 ns even with
 *  RB_IMPL_DIRECTIVE=explore (reproduced 2026-09-23 from a fresh elaboration;
 *  the 2026-09-17 build had already used explore). 35 MHz is the clock the FP16
 *  combined shell closes at, so this pairs with
 *  RocketArty200TDroneGemminiSaturnFp16At35Config at an identical clock.
 */
class RocketArty200TDroneGemminiSaturnIntAt35Config extends Config(
  new chipyard.harness.WithHarnessBinderClockFreqMHz(35) ++
  new chipyard.config.WithUniformBusFrequencies(35) ++
  new RocketArty200TDroneGemminiSaturnIntConfig)
