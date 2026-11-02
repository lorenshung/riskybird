package nvfp4

import chisel3._
import chisel3.util._

/** Exact dot product over one NVFP4 block of 16 elements.
  *
  * Each lane contributes (2|a|)*(2|b|) with sign, so the sum is in the 4x
  * representation: the true block dot product is `out * 2^-2`, scaled by the
  * two E4M3 block scales. |out| <= 16 * 144 = 2304, which fits in 13 bits
  * signed -- and it is EXACT, with no rounding anywhere in this module.
  *
  * The reduction is a balanced tree. Its shape does not matter for the result:
  * integer addition is associative, so any tree gives identical bits. That is
  * deliberate -- it is what makes the hardware reproducible against the software
  * reference regardless of how either one tiles the reduction.
  */
class BlockDot16 extends Module {
  val io = IO(new Bundle {
    val a   = Input(UInt(64.W)) // 16 nibbles, element k at nibble k (low first)
    val b   = Input(UInt(64.W))
    val out = Output(SInt(13.W))
  })

  private val prods = Seq.tabulate(16) { k =>
    val m = Module(new E2M1Mul)
    m.io.a := io.a(4 * k + 3, 4 * k)
    m.io.b := io.b(4 * k + 3, 4 * k)
    m.io.p
  }

  // Balanced adder tree, widening at each level so no stage can overflow.
  private def reduce(xs: Seq[SInt]): SInt =
    if (xs.length == 1) xs.head
    else reduce(xs.grouped(2).map {
      case Seq(l, r) => l +& r
      case Seq(l)    => l
      case _         => throw new IllegalStateException("grouped(2) invariant")
    }.toSeq)

  io.out := reduce(prods)
}
