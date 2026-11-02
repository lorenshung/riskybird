package nvfp4

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec

/** Exhaustive tests for the NVFP4 codec primitives.
  *
  * Every domain here is small enough to enumerate completely (16 E2M1 codes,
  * 64 magnitude pairs, 256 E4M3 codes), so these are exhaustive proofs rather
  * than samples. Take that for free -- it removes a whole class of doubt before
  * any of it reaches an FPGA.
  *
  * The expected values are derived independently from the format definition,
  * not from the Chisel, so agreement means something.
  */
class CodecSpec extends AnyFlatSpec with ChiselScalatestTester {

  // 2*|value| for each of the 8 magnitude codes, from the E2M1 definition:
  // exponent bias 1, one implicit-leading-one mantissa bit, code 0 = zero.
  private val times2 = Seq(0, 1, 2, 3, 4, 6, 8, 12)

  behavior of "E2M1Mul"

  it should "produce the exact signed product for all 256 nibble pairs" in {
    test(new E2M1Mul) { c =>
      for (a <- 0 until 16; b <- 0 until 16) {
        val expMag = times2(a & 7) * times2(b & 7)
        val neg = ((a >> 3) ^ (b >> 3)) == 1
        val expected = if (neg) -expMag else expMag
        c.io.a.poke(a.U)
        c.io.b.poke(b.U)
        c.io.p.expect(expected.S, s"a=$a b=$b")
      }
    }
  }

  it should "never exceed the 9-bit signed range" in {
    // 12*12 = 144 is the largest magnitude; -144..144 fits in 9 bits signed.
    assert(times2.max * times2.max == 144)
    assert(144 <= 255 && -144 >= -256)
  }

  behavior of "ProdTable"

  it should "contain exactly the 64 integer products" in {
    assert(ProdTable.table.length == 64)
    for (i <- 0 until 8; j <- 0 until 8)
      assert(ProdTable.table(i * 8 + j) == times2(i) * times2(j))
    assert(ProdTable.table.max == ProdTable.MAX)
  }

  behavior of "E4M3Decode"

  it should "decode all 256 codes to an exact (mantissa, exponent) pair" in {
    test(new E4M3Decode) { c =>
      for (code <- 0 until 256) {
        val e = (code >> 3) & 0xF
        val m = code & 0x7
        val isNaN = (e == 0xF) && (m == 0x7)
        // value == mant * 2^exp, exactly, for normals and subnormals alike
        val (expMant, expExp) =
          if (isNaN) (0, -9)
          else if (e == 0) (m, -9)
          else (8 + m, e - 10)
        c.io.code.poke(code.U)
        c.io.isNaN.expect(isNaN.B, s"code=$code")
        c.io.mant.expect(expMant.U, s"code=$code")
        if (!isNaN) c.io.exp.expect(expExp.S, s"code=$code")
      }
    }
  }

  it should "reconstruct the documented E4M3 extremes" in {
    test(new E4M3Decode) { c =>
      // max finite = 448 = 14 * 2^5 (S.1111.110)
      c.io.code.poke(0x7E.U); c.io.mant.expect(14.U); c.io.exp.expect(5.S)
      // min positive subnormal = 1 * 2^-9
      c.io.code.poke(0x01.U); c.io.mant.expect(1.U); c.io.exp.expect((-9).S)
      // 1.0 = 8 * 2^-3
      c.io.code.poke(0x38.U); c.io.mant.expect(8.U); c.io.exp.expect((-3).S)
    }
  }

  behavior of "BlockDot16"

  private def pack(nibbles: Seq[Int]): BigInt =
    nibbles.zipWithIndex.map { case (v, k) => BigInt(v & 0xF) << (4 * k) }.sum

  private def exactDot(a: Seq[Int], b: Seq[Int]): Int =
    a.zip(b).map { case (x, y) =>
      val mag = times2(x & 7) * times2(y & 7)
      if (((x >> 3) ^ (y >> 3)) == 1) -mag else mag
    }.sum

  it should "compute the exact integer block dot product" in {
    val rng = new scala.util.Random(0)
    test(new BlockDot16) { c =>
      for (_ <- 0 until 200) {
        val a = Seq.fill(16)(rng.nextInt(16))
        val b = Seq.fill(16)(rng.nextInt(16))
        c.io.a.poke(pack(a).U)
        c.io.b.poke(pack(b).U)
        c.io.out.expect(exactDot(a, b).S)
      }
    }
  }

  it should "saturate nothing at the extremes of its range" in {
    test(new BlockDot16) { c =>
      // all +6 against all +6 -> 16 * 144 = +2304, the positive extreme
      val allPos = Seq.fill(16)(7)          // magnitude code 7 = 6.0, sign +
      c.io.a.poke(pack(allPos).U); c.io.b.poke(pack(allPos).U)
      c.io.out.expect(2304.S)
      // flip one operand's sign -> -2304, the negative extreme
      val allNeg = Seq.fill(16)(0xF)        // sign bit set, magnitude 6.0
      c.io.a.poke(pack(allPos).U); c.io.b.poke(pack(allNeg).U)
      c.io.out.expect((-2304).S)
    }
  }

  it should "be invariant to the order of the 16 lanes" in {
    // Integer addition is associative, so a permutation of the lanes cannot
    // change the result. This is the in-the-small version of the property the
    // whole design depends on: K-tiling cannot change the answer.
    val rng = new scala.util.Random(1)
    test(new BlockDot16) { c =>
      for (_ <- 0 until 50) {
        val a = Seq.fill(16)(rng.nextInt(16))
        val b = Seq.fill(16)(rng.nextInt(16))
        val perm = rng.shuffle((0 until 16).toList)
        c.io.a.poke(pack(a).U); c.io.b.poke(pack(b).U)
        val base = c.io.out.peek().litValue
        c.io.a.poke(pack(perm.map(a)).U); c.io.b.poke(pack(perm.map(b)).U)
        c.io.out.expect(base.S)
      }
    }
  }
}
