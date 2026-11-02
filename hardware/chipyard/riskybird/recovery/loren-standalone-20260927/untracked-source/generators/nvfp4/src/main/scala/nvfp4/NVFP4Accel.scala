// NVFP4 RoCC shell -- control, memory and Chipyard integration around the
// exact 4-bit datapath in Codec.scala / BlockDot.scala / ScaleAlign.scala.
//
// The datapath is already exact and already tested (src/test/scala/nvfp4).
// Everything here is the machinery that lets a bare-metal binary drive it:
// instruction decode, two tiny register-file scratchpads, a strictly
// serialized HellaCache port for DRAM traffic, and a five-state sequencer.
//
// The normative ISA is notes/nvfp4/05_nvfp4_rocc_design.md sections 4.2-4.4 in
// the modelblaster repo; the authoritative field packing is the C encoder at
// tests/nvfp4.h, which is objdump-verified. Where the spec's prose table and
// its Chisel sketch (section 5.5) disagree -- M_tiles is [15:8] in the table
// and rs1(23,8) in the sketch -- the TABLE is normative and is what this file
// decodes, because that is what the C header emits.
//
// TIMING. The whole datapath is combinational in one cycle: 16 product-table
// lookups, a balanced adder tree, a 4x4 mantissa multiply, a barrel shift of
// up to 28 and a 64-bit add, all between the tile registers and the
// accumulator register, x64 PEs. That is fine for Verilator/VCS, where the
// only cost is simulation time, and it keeps the cycle accounting trivial
// (one K-block per cycle, no pipeline to drain). It is NOT fine for synthesis:
// before any timing closure on the U250 this needs at least a register after
// BlockDot16 and another after ScaleAlign, with the accumulator write pushed
// out by the matching number of stages. Doing that changes nothing about the
// arithmetic -- every stage is exact -- so it is a scheduling change only.
//
// MEMORY. v1 uses the HellaCache port LazyRoCC hands us rather than its own
// TileLink client, and issues exactly one outstanding access at a time. This
// is deliberately the slow choice: the D$ port is 8 bytes wide and every
// access costs a full request/response round trip, so the largest test case
// spends a few hundred cycles in MVIN. It buys correctness -- no DMA engine,
// no reordering, no tag management -- and section 3.3's order-independence
// guarantees it cannot change the numerical answer, only the time to get it.
// Bandwidth is a phase-2 problem and a phase-2 rewrite of exactly this file.
package nvfp4

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tile.{LazyRoCC, LazyRoCCModuleImp, OpcodeSet, HasCoreParameters}
import freechips.rocketchip.rocket.{M_XRD, M_XWR}

/** Shape of the engine.
  *
  * `mPE` x `nPE` is the output-stationary array, one PE per output element.
  * 4x16 is the section 2.3 bandwidth-matched shape. `maxKBlocks` bounds the
  * scratchpad: the A tile is `mPE * maxKBlocks` 64-bit words and the B tile is
  * `nPE * maxKBlocks`, so at the defaults the whole scratchpad is 64+256 words
  * of data plus 320 scale bytes. That is small enough to keep in flip-flops,
  * which is what makes the mPE+nPE simultaneous reads the compute stage needs
  * free -- see the note on Reg(Vec) vs SyncReadMem below.
  */
case class NVFP4Config(mPE: Int = 4, nPE: Int = 16, maxKBlocks: Int = 16)

class NVFP4Accel(opcodes: OpcodeSet, cfg: NVFP4Config = NVFP4Config())
    (implicit p: Parameters) extends LazyRoCC(opcodes, nPTWPorts = 0) {
  override lazy val module = new NVFP4AccelModuleImp(this, cfg)
}

class NVFP4AccelModuleImp(outer: NVFP4Accel, cfg: NVFP4Config)
    (implicit p: Parameters) extends LazyRoCCModuleImp(outer) with HasCoreParameters {

  // --------------------------------------------------------------------- //
  // funct7 encoding (section 4.2 / nvfp4.h)
  // --------------------------------------------------------------------- //
  private val F_CONFIG_EX   =   0.U(7.W)
  private val F_CONFIG_LD_A =   1.U(7.W)
  private val F_CONFIG_LD_B =   2.U(7.W)
  private val F_CONFIG_ST   =   3.U(7.W)
  private val F_SET_GSCALE  =   4.U(7.W)
  private val F_MVIN_A      =   5.U(7.W)
  private val F_MVIN_B      =   6.U(7.W)
  private val F_COMPUTE     =   7.U(7.W)
  private val F_COMPUTE_ACC =   8.U(7.W)
  private val F_MVOUT       =   9.U(7.W)
  private val F_MVOUT_RAW   =  10.U(7.W)
  private val F_FLUSH       =  11.U(7.W)
  private val F_COUNTER     = 126.U(7.W)

  private val aCap = cfg.mPE * cfg.maxKBlocks   // 64 words at the defaults
  private val bCap = cfg.nPE * cfg.maxKBlocks   // 256 words
  // log2Ceil(1) is 0, which would make a bit-slice illegal; clamp so that a
  // degenerate 1x1 configuration still elaborates.
  private def idxW(n: Int): Int = math.max(1, log2Ceil(n))
  private val aIdxW = idxW(aCap)
  private val bIdxW = idxW(bCap)
  private val mIdxW = idxW(cfg.mPE)
  private val nIdxW = idxW(cfg.nPE)

  // --------------------------------------------------------------------- //
  // Command interface
  // --------------------------------------------------------------------- //
  // A 2-deep queue so the core can retire a config instruction while a long
  // MVIN is still running; the sequencer only dequeues in sIdle.
  val cmd   = Queue(io.cmd, 2)
  val funct = cmd.bits.inst.funct
  val rs1   = cmd.bits.rs1
  val rs2   = cmd.bits.rs2

  // --------------------------------------------------------------------- //
  // Configuration state
  // --------------------------------------------------------------------- //
  // CONFIG_EX. mTiles/nTiles/kBlocks describe the *whole* dispatch, not the
  // tile the next COMPUTE works on -- every per-tile shape arrives in the
  // instruction that uses it. v1 therefore latches them for COUNTER/debug and
  // acts on none of them; the same is true of `flags`, because COMPUTE vs
  // COMPUTE_ACC already says whether to accumulate and an instruction-level
  // answer beats a mode bit.
  val cfgAct    = Reg(UInt(4.W))
  val cfgMTiles = Reg(UInt(8.W))    // [15:8], per the section 4.2 table
  val cfgNTiles = Reg(UInt(16.W))   // [31:16]
  val cfgKBlocks= Reg(UInt(16.W))   // rs2[47:32]
  val cfgFlags  = Reg(UInt(32.W))
  // EREF_A/EREF_B are the section 2.4 exponent-window contract. v1 keeps the
  // full 64-bit / 29-position alignment path, in which every term is exactly
  // representable, so there is no window to violate and nothing to act on.
  // They are decoded anyway so that the day the narrow accumulator lands, the
  // decode is already right and only the datapath changes.
  val cfgErefA  = Reg(SInt(16.W))
  val cfgErefB  = Reg(SInt(16.W))

  // CONFIG_LD_A / CONFIG_LD_B. One MVIN moves two planes, so each operand
  // needs two strides and a second (scale-plane) base; the data-plane base is
  // rs1 of the MVIN itself.
  val ldADataStride  = Reg(UInt(32.W))
  val ldAScaleStride = Reg(UInt(32.W))
  val ldAScaleBase   = Reg(UInt(xLen.W))
  val ldBDataStride  = Reg(UInt(32.W))
  val ldBScaleStride = Reg(UInt(32.W))
  val ldBScaleBase   = Reg(UInt(xLen.W))

  // CONFIG_ST.
  val stCStride  = Reg(UInt(32.W))
  val stAct      = Reg(UInt(4.W))
  val stOutDtype = Reg(UInt(4.W))

  // SET_GSCALE. v1 only latches these: the global scale is applied at MVOUT,
  // which v1 does not implement (see the F_MVOUT arm below), and bias is a
  // phase-2 item. Latching them now means the C side can issue the full
  // section 4.4 sequence unchanged.
  val gScaleBits = Reg(UInt(32.W))
  val biasBase   = Reg(UInt(xLen.W))

  // SPEC GAP, resolved here. COMPUTE's rs2 is
  // `B_rows<<48 | B_blocks<<32 | acc_addr`, so B's *scratchpad* address has no
  // field anywhere in the instruction -- section 4.2's table spends the low 32
  // bits on acc_addr and section 4 never says what B_spad then is. nvfp4.h
  // documents the contract as "B lives at whatever base the preceding MVIN_B
  // wrote", and that is exactly what this register implements: MVIN_B latches
  // its spad_addr and COMPUTE uses it as the B base. The alternative fixes
  // would both be ISA changes rather than RTL changes: add a third operand
  // (funct3 0x7 with rd unused is free) or put B_spad in CONFIG_EX. Until one
  // of those happens, a COMPUTE that is not preceded by the MVIN_B it means
  // will silently read the wrong tile, which is the cost of the gap.
  val lastBSpad = RegInit(0.U(32.W))

  // --------------------------------------------------------------------- //
  // Tile storage
  // --------------------------------------------------------------------- //
  // Reg(Vec(...)), not SyncReadMem. The compute stage needs mPE + nPE = 20
  // *simultaneous* reads per cycle at unrelated addresses; an SRAM would need
  // 20 ports or 20 banks plus a crossbar, and would add a read-latency cycle
  // the one-block-per-cycle schedule has no room for. Registers give all 20
  // reads for free. This only works because the tiles are tiny -- 320 words of
  // 64 bits, i.e. 2.5 KiB total. A larger scratchpad (the section 5.4 sketch
  // proposed 128 KiB) must be SRAM and must then reshape the compute schedule,
  // which is the main structural reason this is a v1 and not the final shell.
  val aData  = Reg(Vec(aCap, UInt(64.W)))
  val aScale = Reg(Vec(aCap, UInt(8.W)))
  val bData  = Reg(Vec(bCap, UInt(64.W)))
  val bScale = Reg(Vec(bCap, UInt(8.W)))

  // --------------------------------------------------------------------- //
  // Sticky error bits. Cleared only by reset, readable through COUNTER.
  // --------------------------------------------------------------------- //
  // These exist so that a violated assumption becomes a number the test can
  // fail on, instead of a quietly wrong answer. Nothing here stops the engine:
  // a run that sets a bit still completes, and the C side is expected to read
  // the bits at the end of the dispatch.
  val errOOR    = RegInit(false.B)  // some PE's shift left the representable window
  val errSpad   = RegInit(false.B)  // a scratchpad / accumulator index was out of range
  val errShape  = RegInit(false.B)  // COMPUTE with A_blocks =/= B_blocks
  val errUnimpl = RegInit(false.B)  // unimplemented or unknown funct7
  val errBits   = Cat(0.U(60.W), errUnimpl, errShape, errSpad, errOOR)

  // --------------------------------------------------------------------- //
  // The array
  // --------------------------------------------------------------------- //
  val peEn    = WireDefault(false.B)
  val peClear = WireDefault(false.B)
  val aFeed   = Wire(Vec(cfg.mPE, UInt(64.W)))
  val aSFeed  = Wire(Vec(cfg.mPE, UInt(8.W)))
  val bFeed   = Wire(Vec(cfg.nPE, UInt(64.W)))
  val bSFeed  = Wire(Vec(cfg.nPE, UInt(8.W)))

  // Output-stationary: PE(m)(n) owns C[m][n] for the whole COMPUTE, A
  // broadcasts down the columns and B across the rows. No operand ever moves
  // between PEs, which is why there is no PRELOAD in this ISA and no array
  // fill/drain latency to account for.
  val pes = Seq.tabulate(cfg.mPE, cfg.nPE) { (m, n) =>
    val pe = Module(new PE)
    pe.io.en    := peEn
    pe.io.clear := peClear
    pe.io.a     := aFeed(m)
    pe.io.sA    := aSFeed(m)
    pe.io.b     := bFeed(n)
    pe.io.sB    := bSFeed(n)
    pe
  }
  val accOut = VecInit(pes.map(row => VecInit(row.map(_.io.acc))))
  // PE.oor is cleared by PE.clear, i.e. by every COMPUTE. Mirror it into a
  // reset-only sticky bit so a violation in an early tile survives to the end
  // of the dispatch and can still be read out.
  when (VecInit(pes.flatten.map(_.io.oor)).asUInt.orR) { errOOR := true.B }

  // --------------------------------------------------------------------- //
  // Sequencer state
  // --------------------------------------------------------------------- //
  val sIdle :: sMvin :: sClear :: sCompute :: sMvout :: Nil = Enum(5)
  val state = RegInit(sIdle)

  // MVIN walk state. Addresses and the scratchpad index are kept as running
  // row bases rather than recomputed as base + row*stride, so there is no
  // multiplier in the address path.
  val mvIsB       = Reg(Bool())
  val mvRows      = Reg(UInt(16.W))
  val mvBlocks    = Reg(UInt(16.W))
  val mvRow       = Reg(UInt(16.W))
  val mvK         = Reg(UInt(16.W))
  val mvScalePh   = Reg(Bool())      // false: data plane, true: scale plane
  val mvDataAddr  = Reg(UInt(xLen.W))  // base of the current row, data plane
  val mvScaleAddr = Reg(UInt(xLen.W))  // base of the current row, scale plane
  val mvSpadRow   = Reg(UInt(32.W))    // spad index of the current row's block 0
  val mvDataStr   = Reg(UInt(32.W))
  val mvScaleStr  = Reg(UInt(32.W))

  // COMPUTE state.
  val cpBlocks = Reg(UInt(16.W))
  val cpK      = Reg(UInt(16.W))
  val cpASpad  = Reg(UInt(32.W))
  val cpBSpad  = Reg(UInt(32.W))
  val cpARows  = Reg(UInt(16.W))
  val cpBRows  = Reg(UInt(16.W))

  // MVOUT_RAW state.
  val moRows    = Reg(UInt(16.W))
  val moCols    = Reg(UInt(16.W))
  val moRow     = Reg(UInt(16.W))
  val moCol     = Reg(UInt(16.W))
  val moRowAddr = Reg(UInt(xLen.W))  // base of the current output row

  // Privilege context of the command that started the current transfer. It has
  // to be latched, because by the time the first request goes out the command
  // has already been dequeued and cmd.bits.status is no longer meaningful.
  val reqDprv = Reg(chiselTypeOf(cmd.bits.status.dprv))
  val reqDv   = Reg(chiselTypeOf(cmd.bits.status.dv))

  // Exactly one memory operation in flight. `memBusy` is set when a request is
  // accepted and cleared by its response; nothing else is issued in between.
  // The one-at-a-time rule is also what makes tag 0 sufficient: the replay
  // queue inside SimpleHellaCacheIF matches responses to requests by tag, and
  // with a single inflight request there is nothing to disambiguate.
  val memBusy = RegInit(false.B)

  val memReqValid = WireDefault(false.B)
  val memAddr     = WireDefault(0.U(xLen.W))
  val memCmd      = WireDefault(M_XRD)
  val memSize     = WireDefault(3.U(2.W))   // log2(bytes): 3 = 8B, 0 = 1B
  val memData     = WireDefault(0.U(xLen.W))

  // Stores generate responses just like loads (DCache asserts cpu.resp.valid
  // for any s2 read-or-write hit), and SimpleHellaCacheIFReplayQueue only
  // frees an inflight slot when the response arrives. Waiting for the store
  // response is therefore not optional politeness: dropping it would exhaust
  // the 3-entry replay queue and wedge the engine on the fourth store.
  val memResp = io.mem.resp.valid && memBusy
  when (io.mem.req.fire) { memBusy := true.B }
  when (memResp)         { memBusy := false.B }

  // --------------------------------------------------------------------- //
  // Response path
  // --------------------------------------------------------------------- //
  // Any instruction with xd set MUST get a response or the core stalls forever
  // waiting to write its destination register. Only COUNTER is defined to set
  // xd, but responding to *every* xd command -- with zero if it carries no
  // value -- turns a host-side encoding mistake into a wrong number instead of
  // a hang, which is far easier to debug in a bare-metal run.
  // Free-running while busy and never cleared by a read: the host reads it on
  // either side of a dispatch and subtracts. A read-to-clear counter would
  // make every delta after the first read read back as ~0.
  val cycleCnt = RegInit(0.U(64.W))
  val doResp   = cmd.bits.inst.xd
  val respData = Mux(funct === F_COUNTER,
                     Mux(rs1 === 1.U, errBits, cycleCnt),
                     0.U(xLen.W))

  io.resp.valid     := cmd.valid && (state === sIdle) && doResp
  io.resp.bits.rd   := cmd.bits.inst.rd
  io.resp.bits.data := respData
  // cmd.ready is asserted ONLY in sIdle. That is not just simplicity: it is
  // what makes back-to-back tiles safe without an explicit FLUSH. A COMPUTE
  // streams its operands straight out of the tile registers over `blocks`
  // cycles, so a following MVIN that reuses the same spad base would be a WAR
  // hazard against a compute still in flight. Refusing to dequeue until the
  // sequencer is idle removes the hazard by construction; the cost is that the
  // engine never overlaps a load with a compute, which is a v2 concern.
  cmd.ready := (state === sIdle) && !(doResp && !io.resp.ready)

  io.busy      := (state =/= sIdle) || cmd.valid || memBusy
  io.interrupt := false.B
  when (io.busy) { cycleCnt := cycleCnt + 1.U }

  // --------------------------------------------------------------------- //
  // Decode. Every CONFIG_*, SET_GSCALE, FLUSH and COUNTER completes here in
  // one cycle and never leaves sIdle; only MVIN/COMPUTE/MVOUT_RAW start work.
  // --------------------------------------------------------------------- //
  def startMvin(isB: Bool, dataBase: UInt, desc: UInt,
                dataStr: UInt, scaleStr: UInt, scaleBase: UInt): Unit = {
    val rows   = desc(63, 48)
    val blocks = desc(47, 32)
    val spad   = desc(31, 0)
    mvIsB       := isB
    mvRows      := rows
    mvBlocks    := blocks
    mvRow       := 0.U
    mvK         := 0.U
    mvScalePh   := false.B
    mvDataAddr  := dataBase
    mvScaleAddr := scaleBase
    mvSpadRow   := spad
    mvDataStr   := dataStr
    mvScaleStr  := scaleStr
    // A zero-sized tile is a no-op, not an error: the row/block loops below
    // are `while (row < rows)` in spirit, and entering sMvin with rows == 0
    // would run them once.
    when (rows =/= 0.U && blocks =/= 0.U) { state := sMvin }
  }

  when (cmd.fire) {
    reqDprv := cmd.bits.status.dprv
    reqDv   := cmd.bits.status.dv

    when (funct === F_CONFIG_EX) {
      cfgAct     := rs1(7, 4)
      cfgMTiles  := rs1(15, 8)       // table, not the section 5.5 sketch
      cfgNTiles  := rs1(31, 16)
      cfgErefA   := rs1(47, 32).asSInt
      cfgErefB   := rs1(63, 48).asSInt
      cfgFlags   := rs2(31, 0)
      cfgKBlocks := rs2(47, 32)
    } .elsewhen (funct === F_CONFIG_LD_A) {
      ldADataStride  := rs1(63, 32)
      ldAScaleStride := rs1(31, 0)
      ldAScaleBase   := rs2
    } .elsewhen (funct === F_CONFIG_LD_B) {
      ldBDataStride  := rs1(63, 32)
      ldBScaleStride := rs1(31, 0)
      ldBScaleBase   := rs2
    } .elsewhen (funct === F_CONFIG_ST) {
      stCStride  := rs1(63, 32)
      stAct      := rs1(7, 4)
      stOutDtype := rs1(3, 0)
    } .elsewhen (funct === F_SET_GSCALE) {
      gScaleBits := rs1(31, 0)
      biasBase   := rs2
    } .elsewhen (funct === F_MVIN_A) {
      startMvin(false.B, rs1, rs2, ldADataStride, ldAScaleStride, ldAScaleBase)
    } .elsewhen (funct === F_MVIN_B) {
      startMvin(true.B, rs1, rs2, ldBDataStride, ldBScaleStride, ldBScaleBase)
      lastBSpad := rs2(31, 0)   // the spec gap above: this is B's base for COMPUTE
    } .elsewhen (funct === F_COMPUTE || funct === F_COMPUTE_ACC) {
      val aRows   = rs1(63, 48)
      val aBlocks = rs1(47, 32)
      val aSpad   = rs1(31, 0)
      val bRows   = rs2(63, 48)
      val bBlocks = rs2(47, 32)
      // rs2(31,0) is acc_addr. v1 has exactly one accumulator tile -- the
      // array itself, mPE x nPE registers -- so there is no second tile to
      // select and acc_addr is accepted and ignored. A multi-tile accumulator
      // file is the natural next step (it is what makes COMPUTE_ACC over a
      // split K useful across *different* output tiles), and it is the only
      // place acc_addr would start to mean something.
      when (aBlocks =/= bBlocks) { errShape := true.B }
      // AMBIGUITY, resolved here: the K-block count comes from rs1's A_blocks
      // field, NOT from CONFIG_EX's K_blocks. Section 4.2 never says whether
      // that CONFIG_EX field is per-COMPUTE or the per-dispatch total, and the
      // two readings differ the moment K is split -- one CONFIG_EX followed by
      // two COMPUTEs of kb/2 blocks each is exactly the sequence section 4.4
      // recommends for a large K. Per-instruction wins: it is the only reading
      // under which a K-split works without re-issuing CONFIG_EX between
      // passes, and section 3.3's order-independence makes that split free.
      // cfgKBlocks is therefore latched for readback and never sizes a loop.
      // Clamp the K loop at the scratchpad capacity. Beyond it the indices
      // below would alias, and a 16-bit blocks field could otherwise ask for a
      // 65535-cycle COMPUTE over data that was never loaded. Clamping keeps
      // the run bounded and errSpad says the answer is not to be trusted.
      val blocks = Mux(aBlocks > cfg.maxKBlocks.U, cfg.maxKBlocks.U, aBlocks)
      when (aBlocks > cfg.maxKBlocks.U || aRows > cfg.mPE.U || bRows > cfg.nPE.U) {
        errSpad := true.B
      }
      cpBlocks := blocks
      cpK      := 0.U
      cpASpad  := aSpad
      cpBSpad  := lastBSpad
      cpARows  := aRows
      cpBRows  := bRows
      // COMPUTE clears first, COMPUTE_ACC does not. The clear needs its own
      // cycle because PE gives `clear` priority over `en`, so a cycle that
      // clears cannot also accumulate -- folding them would silently drop
      // block 0 of every COMPUTE.
      when (funct === F_COMPUTE) {
        state := sClear
      } .elsewhen (blocks =/= 0.U) {
        state := sCompute
      }
    } .elsewhen (funct === F_MVOUT_RAW) {
      val rows = rs2(63, 48)
      val cols = rs2(47, 32)
      when (rows > cfg.mPE.U || cols > cfg.nPE.U) { errSpad := true.B }
      moRows    := Mux(rows > cfg.mPE.U, cfg.mPE.U, rows)
      moCols    := Mux(cols > cfg.nPE.U, cfg.nPE.U, cols)
      moRow     := 0.U
      moCol     := 0.U
      moRowAddr := rs1
      when (rows =/= 0.U && cols =/= 0.U) { state := sMvout }
    } .elsewhen (funct === F_FLUSH) {
      // Nothing to flush. The sequencer only accepts a command in sIdle and
      // holds exactly one memory operation in flight, so by the time FLUSH is
      // dequeued every earlier instruction has already retired its last
      // response. rs1's skip flag is therefore a no-op in both settings. The
      // instruction is still accepted, because the section 4.4 sequence emits
      // it and a pipelined v2 will need it to mean something.
    } .elsewhen (funct === F_COUNTER) {
      // Handled entirely by the response path above.
    } .elsewhen (funct === F_MVOUT) {
      // NOT IMPLEMENTED IN v1, deliberately, for two independent reasons.
      //
      // (1) It is the only instruction that needs floating point: normalise a
      //     64-bit signed fixed-point accumulator with LSB 2^-20 into fp32
      //     (leading-zero count, round-to-nearest-even on the discarded bits)
      //     and then multiply by the fp32 global scale, correctly rounded.
      //     That is a real FPU-grade rounding problem and getting it subtly
      //     wrong would be indistinguishable from a datapath bug.
      // (2) More decisively: the frozen test vectors' OUT column currently
      //     uses a different global-scale convention from section 3.5.1's
      //     (decode-scale) one, so the expected values are contested. There is
      //     no point implementing rounding against a target nobody agrees on.
      //
      // MVOUT_RAW exists precisely so bit-exactness can be proved without any
      // of this: it dumps the accumulators before a single float is involved.
      // Until the convention is settled, MVOUT sets the sticky bit and does
      // nothing, so a host that issues it gets a loud, readable failure
      // instead of a buffer of zeros it might mistake for a result.
      errUnimpl := true.B
    } .otherwise {
      errUnimpl := true.B
    }
  }

  // --------------------------------------------------------------------- //
  // sMvin -- serialized two-plane load
  // --------------------------------------------------------------------- //
  // Per row: `blocks` 8-byte data loads, then `blocks` 1-byte scale loads.
  // Splitting by plane rather than interleaving keeps each plane's address
  // walk a simple increment and matches the R1 "two planes, not interleaved"
  // layout contract.
  //
  // Nibble order needs no fixing up. R2 puts element 2i in the low nibble of
  // byte i, and a little-endian 8-byte load puts byte i at bits [8i+7:8i], so
  // element k lands at bits [4k+3:4k] -- exactly where BlockDot16 reads it.
  // The word goes into the tile untouched.
  val mvIdxFull = mvSpadRow +& mvK
  val mvInRange = Mux(mvIsB, mvIdxFull < bCap.U, mvIdxFull < aCap.U)
  val mvLast    = (mvK === mvBlocks - 1.U)

  when (state === sMvin) {
    memReqValid := !memBusy
    memCmd      := M_XRD
    memAddr     := Mux(mvScalePh, mvScaleAddr + mvK, mvDataAddr + (mvK << 3))
    memSize     := Mux(mvScalePh, 0.U, 3.U)

    when (memResp) {
      // Out-of-range writes are dropped rather than wrapped: a wrapped write
      // would corrupt a block that was loaded correctly, turning a capacity
      // mistake into a wrong answer somewhere else entirely.
      when (!mvInRange) {
        errSpad := true.B
      } .elsewhen (mvIsB) {
        when (mvScalePh) { bScale(mvIdxFull(bIdxW - 1, 0)) := io.mem.resp.bits.data(7, 0) }
        .otherwise       { bData(mvIdxFull(bIdxW - 1, 0))  := io.mem.resp.bits.data }
      } .otherwise {
        when (mvScalePh) { aScale(mvIdxFull(aIdxW - 1, 0)) := io.mem.resp.bits.data(7, 0) }
        .otherwise       { aData(mvIdxFull(aIdxW - 1, 0))  := io.mem.resp.bits.data }
      }

      when (!mvLast) {
        mvK := mvK + 1.U
      } .otherwise {
        mvK := 0.U
        when (!mvScalePh) {
          mvScalePh := true.B          // same row, now its scale bytes
        } .otherwise {
          mvScalePh := false.B
          when (mvRow === mvRows - 1.U) {
            state := sIdle
          } .otherwise {
            mvRow       := mvRow + 1.U
            mvDataAddr  := mvDataAddr + mvDataStr
            mvScaleAddr := mvScaleAddr + mvScaleStr
            mvSpadRow   := mvSpadRow + mvBlocks
          }
        }
      }
    }
  }

  // --------------------------------------------------------------------- //
  // sClear / sCompute
  // --------------------------------------------------------------------- //
  // The A and B indices are `spad + row*blocks + k`. `row` is a Scala constant
  // per PE row/column, so this is a constant-by-variable multiply that folds
  // into a few adds, not a real multiplier.
  val aIdxFull = VecInit(Seq.tabulate(cfg.mPE)(m => cpASpad +& (cpBlocks * m.U) +& cpK))
  val bIdxFull = VecInit(Seq.tabulate(cfg.nPE)(n => cpBSpad +& (cpBlocks * n.U) +& cpK))

  // Rows past A_rows (columns past B_rows) are fed a zero nibble word AND a
  // zero scale byte. The zero data alone would already make the term exactly
  // zero -- every E2M1 lane is +0.0, so the block dot is 0 -- and no surplus
  // PE is ever drained, so arithmetically the scale would not matter. It is
  // masked anyway for a simulation reason: the tile registers are
  // uninitialised, so in a 4-state simulator (VCS) an unwritten scale byte is
  // X, `0 * X` is X, and the X reaches ScaleAlign's range check and poisons
  // the sticky OOR bit that the test prints. Feeding 0x00 makes the whole
  // shadow of the array deterministic: E4M3 0x00 decodes to mant 0, exp -9,
  // so the shift is in range and the term is a clean zero.
  for (m <- 0 until cfg.mPE) {
    val live = m.U < cpARows
    aFeed(m)  := Mux(live, aData(aIdxFull(m)(aIdxW - 1, 0)), 0.U)
    aSFeed(m) := Mux(live, aScale(aIdxFull(m)(aIdxW - 1, 0)), 0.U)
  }
  for (n <- 0 until cfg.nPE) {
    val live = n.U < cpBRows
    bFeed(n)  := Mux(live, bData(bIdxFull(n)(bIdxW - 1, 0)), 0.U)
    bSFeed(n) := Mux(live, bScale(bIdxFull(n)(bIdxW - 1, 0)), 0.U)
  }

  when (state === sClear) {
    peClear := true.B
    state   := Mux(cpBlocks === 0.U, sIdle, sCompute)
  }

  when (state === sCompute) {
    peEn := true.B
    val anyOOR = VecInit(
      Seq.tabulate(cfg.mPE)(m => aIdxFull(m) >= aCap.U) ++
      Seq.tabulate(cfg.nPE)(n => bIdxFull(n) >= bCap.U)).asUInt.orR
    when (anyOOR) { errSpad := true.B }
    when (cpK === cpBlocks - 1.U) {
      state := sIdle
    } .otherwise {
      cpK := cpK + 1.U
    }
  }

  // --------------------------------------------------------------------- //
  // sMvout -- serialized MVOUT_RAW
  // --------------------------------------------------------------------- //
  // 8 bytes per element, untruncated, LSB 2^-20, before the global scale.
  //
  // AMBIGUITY, resolved here. Section 4.2 gives MVOUT_RAW the same
  // `rows<<48 | cols<<32 | acc_addr` descriptor as MVOUT but never says which
  // stride the destination uses, and nvfp4.h's comment says "the destination
  // row stride is cols*8" -- which is the same thing only when the caller's C
  // buffer is exactly `rows x cols`. This implementation uses CONFIG_ST's
  // c_row_stride, because that is the field whose whole job is "the row pitch
  // of the destination in DRAM" and because it is strictly more general: a
  // caller that wants the packed layout sets c_row_stride = cols*8 and gets
  // nvfp4.h's reading exactly. A caller dumping into a sub-rectangle of a
  // larger buffer can only express that with a stride.
  val moAddr = moRowAddr + (moCol << 3)

  when (state === sMvout) {
    memReqValid := !memBusy
    memCmd      := M_XWR
    memSize     := 3.U
    memAddr     := moAddr
    memData     := accOut(moRow(mIdxW - 1, 0))(moCol(nIdxW - 1, 0)).asUInt

    when (memResp) {
      when (moCol =/= moCols - 1.U) {
        moCol := moCol + 1.U
      } .otherwise {
        moCol := 0.U
        when (moRow === moRows - 1.U) {
          state := sIdle
        } .otherwise {
          moRow     := moRow + 1.U
          moRowAddr := moRowAddr + stCStride
        }
      }
    }
  }

  // --------------------------------------------------------------------- //
  // HellaCache request wiring
  // --------------------------------------------------------------------- //
  // phys = false means these addresses go through the DTLB. The bare-metal
  // test runs in M-mode with no paging, so VA == PA and the translation is the
  // identity -- but it still enforces alignment, and SimpleHellaCacheIF
  // asserts on any exception ("SimpleHellaCacheIF exception"). If that assert
  // fires, the address arithmetic above is wrong; it is not a cache bug.
  io.mem.req.valid       := memReqValid
  io.mem.req.bits.addr   := memAddr(coreMaxAddrBits - 1, 0)
  io.mem.req.bits.tag    := 0.U
  io.mem.req.bits.cmd    := memCmd
  io.mem.req.bits.size   := memSize
  io.mem.req.bits.signed := false.B
  io.mem.req.bits.data   := memData
  io.mem.req.bits.phys   := false.B
  io.mem.req.bits.dprv   := reqDprv
  io.mem.req.bits.dv     := reqDv
  io.mem.req.bits.no_resp  := false.B
  io.mem.req.bits.no_alloc := false.B
  io.mem.req.bits.no_xcpt  := false.B
  // `mask` only matters for M_PWR (DCache.scala:325 selects it only for that
  // command); M_XRD/M_XWR get a mask derived from size and address. It is
  // driven all-ones anyway rather than left at the DontCare LazyRoCCModuleImp
  // installs, so the generated Verilog has no invalidated bits on a path that
  // feeds a DCache assertion.
  io.mem.req.bits.mask     := ~0.U(coreDataBytes.W)
}
