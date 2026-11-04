// NVFP4 element and scale codecs.
//
// NVFP4 stores a tensor as 4-bit E2M1 elements, one FP8 E4M3 scale per 16
// consecutive elements, and one FP32 per-tensor global scale.
//
// The whole hardware design rests on one observation: every E2M1 magnitude is a
// multiple of 0.5, so 2*|value| is an INTEGER. Products of those integers are
// exact, a 16-element block dot product is an exact 13-bit integer, and an E4M3
// scale decodes exactly to mantissa * 2^exponent. Nothing in the datapath needs
// a floating-point multiplier, and accumulation can be made associative -- which
// is what lets the hardware be bit-exact against the software reference
// (pipeline/nvfp4_numerics.py in the modelblaster repo) rather than merely close.
package nvfp4

import chisel3._
import chisel3.util._

/** E2M1: bit [3] = sign, [2:1] = exponent (bias 1), [0] = mantissa.
  *
  * Magnitude index is bits[2:0]; TIMES2(idx) = 2*|value|, always an integer.
  *
  *   idx      0    1    2    3    4    5    6    7
  *   |value| 0.0  0.5  1.0  1.5  2.0  3.0  4.0  6.0
  *   2*|v|    0    1    2    3    4    6    8   12
  *
  * There are no infinities and no NaN: all 16 codes are finite values.
  */
object E2M1 {
  val TIMES2: Seq[Int] = Seq(0, 1, 2, 3, 4, 6, 8, 12)
  val MAX_TIMES2: Int = 12

  def sign(x: UInt): Bool = x(3)
  def magIdx(x: UInt): UInt = x(2, 0)
}

/** Exact product of two E2M1 magnitudes, in the 2x representation.
  *
  * (2|a|)*(2|b|) for the 64 magnitude-index pairs, range [0, 144] -> 8 bits.
  * This is a 6-bit-address / 8-bit-data ROM: on an UltraScale+ LUT6 it is 8
  * LUTs and NO arithmetic at all. That is the central area argument for NVFP4
  * over int8 -- the multiplier degenerates into a tiny table.
  */
object ProdTable {
  val table: Seq[Int] =
    for (i <- 0 until 8; j <- 0 until 8) yield E2M1.TIMES2(i) * E2M1.TIMES2(j)
  val MAX: Int = 144

  def apply(ai: UInt, bi: UInt): UInt =
    VecInit(table.map(_.U(8.W)))(Cat(ai, bi))
}

/** One lane: signed exact product of two E2M1 nibbles, in the 4x representation.
  *
  * Output is (2|a|)*(2|b|) with the XOR of the signs applied, so the true
  * product is `p * 2^-2`. Range [-144, +144] fits in 9 bits signed.
  */
class E2M1Mul extends Module {
  val io = IO(new Bundle {
    val a = Input(UInt(4.W))
    val b = Input(UInt(4.W))
    val p = Output(SInt(9.W))
  })
  private val mag = ProdTable(E2M1.magIdx(io.a), E2M1.magIdx(io.b))
  private val neg = E2M1.sign(io.a) ^ E2M1.sign(io.b)
  io.p := Mux(neg, -(mag.zext.asSInt), mag.zext.asSInt)
}

/** OCP FP8 E4M3, bias 7, decoded to an EXACT (mantissa, exponent) pair.
  *
  *   value == mant * 2^exp,  mant in [0, 15],  exp in [-9, 5]
  *
  * Normals carry the implicit leading one (mant = 8 + m, exp = e - 10);
  * subnormals are (mant = m, exp = -9). A uniform representation for both means
  * the downstream datapath needs no special case.
  *
  * E4M3 has no infinities; 0x7F/0xFF are NaN and decode to mantissa 0. Block
  * scales are non-negative by construction, so the sign bit is ignored (the
  * memory-layout contract requires it to be 0).
  */
class E4M3Decode extends Module {
  val io = IO(new Bundle {
    val code = Input(UInt(8.W))
    val mant = Output(UInt(4.W))
    val exp  = Output(SInt(6.W))
    val isNaN = Output(Bool())
  })
  private val e = io.code(6, 3)
  private val m = io.code(2, 0)
  private val nan = (e === 0xF.U) && (m === 0x7.U)
  io.isNaN := nan
  io.mant := Mux(nan, 0.U, Mux(e === 0.U, m, Cat(1.U(1.W), m)))
  io.exp  := Mux(e === 0.U, (-9).S(6.W), (e.zext.asSInt - 10.S))
}
