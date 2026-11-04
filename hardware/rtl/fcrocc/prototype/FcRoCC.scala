// ===========================================================================
// FcRoCC.scala -- Hardened Flight-Controller RoCC accelerator (Chisel prototype)
//
// A single shared fixed-point datapath + microsequencer that runs the whole
// riskybird v3 estimator (Mahony attitude + complementary filter) and the
// hierarchical PID cascade + 4x4 mixer + force->duty as one fixed-function
// pipeline, exposed to the Rocket core as a custom-opcode RoCC accelerator.
//
// Purpose: take the FC's estimator+PID off the shared Saturn vector unit and
// off the scalar core, so (a) the Saturn is freed entirely for DroNet and can
// be built INTEGER-ONLY (smaller -- it sheds its fp16 vector-FP + convert
// units), and (b) the scalar FPU can be dropped. The whole FC loop then costs
// a small, LUT-cheap fixed-function block.
//
// Arithmetic (chosen for minimum LUT -- see design doc / README):
//   * ONE global fixed-point format Q24.24 (48-bit signed): range +/-8.4e6
//     (covers the force-curve discriminant ~65838), resolution 6.0e-8 which is
//     >= the fp32 mantissa near 1.0. That satisfies the precision boundary the
//     fp16 Spike+pybullet harness (branch fp16-estim-impl) established: the
//     drift-sensitive accumulators (quaternion state+integrate+normalise,
//     velocity/position integration, gyro-derivative LP, PID integrators, and
//     the R*a-g gravity cancellation) need ~fp32-equivalent precision. A single
//     wide integer format gives that everywhere with no per-op rescaling.
//   * A single shared, pipelined signed 48x48 multiplier (DSP-mapped), time-
//     multiplexed across every product in the loop (~150) AND across the
//     Newton-Raphson iterations -> no dedicated divider, no FP unit at all.
//   * recip / rsqrt / sqrt via Newton-Raphson microcode on the shared
//     multiplier, seeded by a leading-zero-count initial guess.
//
// This is a FEASIBILITY prototype (not production): the datapath, register
// file, shared multiplier, NR strategy, microword width and op-count match the
// OOC synthesis witness fc_accel.sv one-for-one, so the Vivado OOC area number
// characterises this module. The functional microprogram is validated against
// the C golden (samples/rose_flight_controller/src/fp16_fc_kernels.c) by the
// same off-board harness; here the schedule is sketched with phase comments.
// ===========================================================================
package chipyard.fc

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Parameters, Config, Field}
import freechips.rocketchip.tile._
import freechips.rocketchip.rocket._
import freechips.rocketchip.diplomacy._

// ---- custom-opcode functs -------------------------------------------------
//   PUSH (funct=0): rf[rs1] <- rs2   (stream a Q24.24 sensor input in)
//   RUN  (funct=1): execute one FC step over the resident inputs; resp = done
//   POP  (funct=2): resp.data <- rf[rs1]  (read a Q24.24 motor output back)
//   CFG  (funct=3): rf[rs1] <- rs2   (load a tunable / constant register)
// Inputs/outputs pass as Q24.24 in the low 48 bits of an XLEN GPR. The FC
// firmware converts raw sensor ints <-> Q24.24 with integer shifts (no FP),
// which is exactly what an int-only core wants.
//
// Alternative interface (documented, not the minimal-area default): a single
// RUN with rs1 = DRAM pointer to a sensor struct and rs2 = DRAM pointer to a
// motor-output struct, using the RoCC HellaCache mem port. That suits the
// camera-in-loop path where sensors already land in DDR; it costs the TLB/mem
// handshake logic. The streaming interface below needs no mem port -> smaller.

object FcRoCCISA {
  val FUNCT_PUSH = 0.U
  val FUNCT_RUN  = 1.U
  val FUNCT_POP  = 2.U
  val FUNCT_CFG  = 3.U
}

class FcRoCC(opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  override lazy val module = new FcRoCCModuleImp(this)
}

class FcRoCCModuleImp(outer: FcRoCC)(implicit p: Parameters)
    extends LazyRoCCModuleImp(outer) {
  import FcRoCCISA._

  // ---- fixed-point / sizing params -------------------------------------
  val W    = 48          // Q24.24 datapath width
  val QF   = 24          // fractional bits
  val NREG = 64          // register file depth
  val NUOP = 384         // microprogram length bound
  val RAW  = log2Ceil(NREG)
  val UAW  = log2Ceil(NUOP)

  // ---- register file: 3 read (a,b,c) + 1 write. Chisel Mem -> distributed
  //      RAM, keeping the operand path off the general fabric (min LUT). -----
  val rf = Mem(NREG, SInt(W.W))

  // ---- microword ---------------------------------------------------------
  //  aluop: 0 MUL  1 MAC(a*b+c)  2 ADD  3 SUB  4 MIN  5 MAX  6 MOV  7 NEG
  //         8 ABS  9 RSQRT_SEED 10 LZC 11 ASR(imm=b) 12 SEL(sign c) 13 HALT
  class MicroWord extends Bundle {
    val aluop = UInt(4.W)
    val a     = UInt(RAW.W)
    val b     = UInt(RAW.W)
    val c     = UInt(RAW.W)
    val w     = UInt(RAW.W)
  }
  // The real, C-golden-validated schedule is loaded here. Phases (op-count
  // approximate; total ~300-340 fits in NUOP=384):
  //   E0  lever-arm gyro-deriv LP + accel correction         ~24 ops
  //   E1  Mahony |a|, gate, rsqrt(|a|^2) [NR], normalise      ~26 ops
  //   E2  predicted-up vu, e = a x vu, w += KP*gate*e         ~22 ops
  //   E3  quaternion dq = 0.5*Omega(w)*q, integrate           ~20 ops
  //   E4  |q|^2, rsqrt [NR], q *= inv                          ~14 ops
  //   E5  rotation matrix R (9 elems)                         ~30 ops
  //   E6  world accel R*a, - g, vel += aw*dt                  ~22 ops
  //   E7  flow fuse / ZUPT, vel clamp, pos += vel*dt, tof     ~26 ops
  //   E8  get_state lead (quat lead, Rodrigues r=q_xyz/qw)    ~24 ops
  //   C0  altitude: alt_int, desAcc3, cos poly, desNorm       ~26 ops
  //   C1  horiz vel loop: e, vel_int, desAcc, desRP, slew     ~30 ops
  //   C2  attitude + rate loops (roll/pitch/yaw)              ~20 ops
  //   C3  4x4 mixer M*u                                       ~20 ops
  //   C4  force->duty: t, disc, sqrt [NR], (sqrt-TB)/(2TA)    ~28 ops
  //   NR reciprocal (1/x): x_{n+1}=x_n*(2 - d*x_n), 3 iters   ~6 ops each
  //   NR rsqrt (1/sqrt x):  y*(1.5 - 0.5*d*y^2),   3 iters    ~9 ops each
  val ucode = VecInit(Seq.tabulate(NUOP){ i =>
    // Placeholder deterministic fill for elaboration/area; replace with the
    // assembled schedule (a generated .hex from the fc microassembler).
    val mw = Wire(new MicroWord)
    mw.aluop := (i % 14).U
    mw.a := (i*3      % NREG).U
    mw.b := (i*5 + 1  % NREG).U
    mw.c := (i*11 + 2 % NREG).U
    mw.w := (i*13 + 3 % NREG).U
    mw.asUInt
  })

  val upc = RegInit(0.U(UAW.W))
  val uw  = ucode(upc).asTypeOf(new MicroWord)

  // registered operand selects -> rf async read
  val ra = Reg(UInt(RAW.W)); val rb = Reg(UInt(RAW.W))
  val rc = Reg(UInt(RAW.W)); val rw = Reg(UInt(RAW.W))
  val opa = rf(ra); val opb = rf(rb); val opc = rf(rc)

  // ---- shared pipelined signed 48x48 multiplier (3 stages, DSP) ----------
  val mA1  = Reg(SInt(W.W)); val mB1 = Reg(SInt(W.W))
  val mC1  = Reg(SInt(W.W)); val mC2 = Reg(SInt(W.W))
  val prod = Reg(SInt((2*W).W))
  mA1 := opa; mB1 := opb; mC1 := opc
  prod := mA1 * mB1
  mC2  := mC1
  // renormalise Q48.48 -> Q24.24 (+ optional MAC addend c), saturate to 48b
  val cExt   = (mC2.asSInt << QF).asSInt
  val psum   = prod +& cExt
  val pshift = (psum >> QF).asSInt
  val hi     = pshift(2*W-1, W-1)
  val mulOvf = !(hi.andR) && hi.orR
  val mulMax = Cat(0.U(1.W), ~0.U((W-1).W)).asSInt
  val mulMin = Cat(1.U(1.W),  0.U((W-1).W)).asSInt
  val mulSat = Mux(mulOvf, Mux(pshift(2*W-1), mulMin, mulMax), pshift(W-1,0).asSInt)

  // ---- shared 48-bit add/sub/min/max/neg/abs + LZC seed ------------------
  def sat48(x: SInt): SInt = {
    val mx = Cat(0.U(1.W), ~0.U((W-1).W)).asSInt
    val mn = Cat(1.U(1.W),  0.U((W-1).W)).asSInt
    val ov = x(W) =/= x(W-1)
    Mux(ov, Mux(x(W), mn, mx), x(W-1,0).asSInt)
  }
  val addS = sat48((opa +& opb))
  val subS = sat48((opa -& opb))
  val minS = Mux(opa < opb, opa, opb)
  val maxS = Mux(opa > opb, opa, opb)
  val negS = sat48(-(opa.pad(W+1)))
  val absS = Mux(opa < 0.S, negS, opa)
  val lz   = PriorityEncoder(Reverse(opa.asUInt))          // leading-zero count
  val seedSh = (((W-QF).U - lz) >> 1)
  val rsqrtSeed = ((1.S(W.W) << QF) >> seedSh).asSInt       // NR rsqrt seed
  val asrS   = (opa >> rb).asSInt

  val aluY = MuxLookup(uw.aluop, opa)(Seq(
    0.U -> mulSat, 1.U -> mulSat,
    2.U -> addS,   3.U -> subS,
    4.U -> minS,   5.U -> maxS,
    6.U -> opa,    7.U -> negS,
    8.U -> absS,   9.U -> rsqrtSeed,
    10.U-> lz.asSInt, 11.U -> asrS,
    12.U-> Mux(opc < 0.S, opa, opb)
  ))
  val aluIsMul = (uw.aluop === 0.U) || (uw.aluop === 1.U)

  // ---- write-back --------------------------------------------------------
  val we    = WireDefault(false.B)
  val wdata = WireDefault(0.S(W.W))
  val wsel  = WireDefault(rw)
  when (we) { rf(wsel) := wdata }

  // ---- RoCC handshake + sequencer FSM ------------------------------------
  val cmd = Queue(io.cmd)
  val sIdle :: sRun :: sMulWait :: sDone :: Nil = Enum(4)
  val state  = RegInit(sIdle)
  val mulcnt = Reg(UInt(2.W))
  val respV  = RegInit(false.B)
  val respD  = Reg(UInt(xLen.W))

  cmd.ready       := (state === sIdle) && !respV
  io.resp.valid   := respV
  io.resp.bits.rd := RegEnable(cmd.bits.inst.rd, cmd.fire)
  io.resp.bits.data := respD
  io.busy         := (state =/= sIdle) || cmd.valid
  io.interrupt    := false.B
  when (io.resp.fire) { respV := false.B }

  switch (state) {
    is (sIdle) {
      when (cmd.fire) {
        switch (cmd.bits.inst.funct) {
          is (FUNCT_PUSH) { we := true.B; wsel := cmd.bits.rs1(RAW-1,0); wdata := cmd.bits.rs2(W-1,0).asSInt }
          is (FUNCT_CFG)  { we := true.B; wsel := cmd.bits.rs1(RAW-1,0); wdata := cmd.bits.rs2(W-1,0).asSInt }
          is (FUNCT_POP)  { respD := rf(cmd.bits.rs1(RAW-1,0)).asUInt.pad(xLen); respV := true.B }
          is (FUNCT_RUN)  { upc := 0.U; mulcnt := 0.U; state := sRun }
        }
      }
    }
    is (sRun) {
      ra := uw.a; rb := uw.b; rc := uw.c; rw := uw.w
      when (uw.aluop === 13.U) {                 // HALT
        state := sDone
      } .elsewhen (aluIsMul) {
        mulcnt := 2.U; state := sMulWait          // drain 3-stage multiplier
      } .otherwise {
        we := true.B; wdata := aluY; upc := upc + 1.U
      }
    }
    is (sMulWait) {
      when (mulcnt =/= 0.U) { mulcnt := mulcnt - 1.U }
      .otherwise { we := true.B; wdata := mulSat; upc := upc + 1.U; state := sRun }
    }
    is (sDone) { respD := 1.U; respV := true.B; state := sIdle }
  }
}

// ---- config fragment: add the FC RoCC on custom opcode 0 (custom0) --------
class WithFcRoCC extends Config((site, here, up) => {
  case BuildRoCC => up(BuildRoCC) ++ Seq(
    (p: Parameters) => {
      val fc = LazyModule(new FcRoCC(OpcodeSet.custom0)(p))
      fc
    })
})
