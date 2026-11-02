package nvfp4

import chisel3._
import chisel3.util._

object NVFP4Params {
  /** Fixed-point accumulator LSB is 2^-ACC_LSB_SHIFT.
    *
    * Why 20 is exactly right, and not a guess: an E4M3 code decodes to
    * `mant * 2^exp` with exp in [-9, 5], so a product of two block scales has
    * exponent `eA + eB` in [-18, 10]. The block dot product is carried in the
    * 4x representation (each element contributes 2|x|), which costs a further
    * 2^-2, giving an overall term exponent `eS = eA + eB - 2` in [-20, +8].
    * An LSB of 2^-20 therefore represents EVERY possible term exactly, with no
    * shift ever going negative and so no rounding anywhere in the datapath.
    */
  val ACC_LSB_SHIFT = 20

  /** Widest left shift: eS_max - eS_min = 8 - (-20) = 28. */
  val MAX_SHIFT = 28

  /** S in [-2304, 2304] (13b signed) times mS in [0, 225] -> 21 bits signed. */
  val PROD_W = 21

  /** Accumulator width. 49 bits covers one aligned term; the rest is headroom
    * for accumulation over K. 64 bits is exact for K <= 65536 -- see the width
    * proof in notes/nvfp4/05_nvfp4_rocc_design.md §3.4.
    */
  val ACC_W = 64
}

/** Align one block's exact contribution onto the shared fixed-point grid.
  *
  * Inputs are the exact 13-bit block dot product and the two E4M3 block scales,
  * already decoded to (mantissa, exponent). Output is that term expressed in
  * units of 2^-20, exactly.
  *
  * Nothing here rounds. `mA*mB` is an exact 4x4->8 bit integer multiply, the
  * exponent is integer addition, and the barrel shift is always LEFT (the shift
  * amount is provably non-negative), so no bits are ever discarded. That is what
  * makes the accumulator associative and the engine order-independent.
  */
class ScaleAlign extends Module {
  import NVFP4Params._
  val io = IO(new Bundle {
    val dot   = Input(SInt(13.W))  // exact block dot, 4x representation
    val mantA = Input(UInt(4.W))
    val expA  = Input(SInt(6.W))
    val mantB = Input(UInt(4.W))
    val expB  = Input(SInt(6.W))
    val out   = Output(SInt(ACC_W.W))
    /** Sticky signal: the shift left the representable window. Should be
      * impossible for well-formed E4M3 input; exposed so a violation becomes a
      * loud failure rather than a silently wrong number. */
    val shiftOOR = Output(Bool())
  })

  private val mS = io.mantA * io.mantB                 // exact, <= 225, 8 bits
  private val prod = (io.dot * mS.zext.asSInt)         // exact, 21 bits signed

  // eS = eA + eB - 2; shift onto the 2^-ACC_LSB_SHIFT grid.
  private val shiftS = io.expA +& io.expB - 2.S + ACC_LSB_SHIFT.S
  io.shiftOOR := (shiftS < 0.S) || (shiftS > MAX_SHIFT.S)

  private val shamt = shiftS.asUInt(log2Ceil(MAX_SHIFT + 1) - 1, 0)

  // Widths: prod is 21 bits signed and shamt is 5 bits, so Chisel infers
  // 21 + (2^5 - 1) = 52 bits for the dynamic shift -- comfortably inside the
  // 64-bit accumulator, so the connect below sign-extends and never truncates.
  // (The real worst case is 21 + MAX_SHIFT = 49 bits.)
  private val shifted = Wire(SInt(ACC_W.W))
  shifted := (prod << shamt)
  io.out := Mux(io.shiftOOR, 0.S, shifted)
}

/** One processing element: owns one output element C[m][n].
  *
  * Output-stationary. Each cycle it consumes one 16-element K-block from each
  * operand and adds the exact aligned term into its accumulator. Because every
  * term is exact and the accumulator is plain integer addition, the result does
  * not depend on the order blocks arrive in -- so the compiler may split K
  * anywhere it likes and the answer cannot change.
  */
class PE extends Module {
  import NVFP4Params._
  val io = IO(new Bundle {
    val en    = Input(Bool())
    val clear = Input(Bool())
    val a     = Input(UInt(64.W))   // 16 packed E2M1 nibbles
    val b     = Input(UInt(64.W))
    val sA    = Input(UInt(8.W))    // E4M3 block scale codes
    val sB    = Input(UInt(8.W))
    val acc   = Output(SInt(ACC_W.W))
    val oor   = Output(Bool())      // sticky shift-out-of-range
  })

  private val dot = Module(new BlockDot16)
  dot.io.a := io.a
  dot.io.b := io.b

  private val decA = Module(new E4M3Decode); decA.io.code := io.sA
  private val decB = Module(new E4M3Decode); decB.io.code := io.sB

  private val align = Module(new ScaleAlign)
  align.io.dot   := dot.io.out
  align.io.mantA := decA.io.mant
  align.io.expA  := decA.io.exp
  align.io.mantB := decB.io.mant
  align.io.expB  := decB.io.exp

  private val acc = RegInit(0.S(ACC_W.W))
  private val oor = RegInit(false.B)
  when(io.clear) {
    acc := 0.S
    oor := false.B
  }.elsewhen(io.en) {
    acc := acc + align.io.out
    oor := oor || align.io.shiftOOR
  }
  io.acc := acc
  io.oor := oor
}
