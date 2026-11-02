package nvfp4

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec

import java.io.File
import scala.io.Source

/** PE / ScaleAlign tests driven by REAL NVFP4 blocks from a shipped NVIDIA
  * checkpoint (nvidia/Nemotron-3-Embed-1B-NVFP4, revision f630128).
  *
  * These are the tier-0 anchor of the verification chain. Synthetic vectors
  * would only prove our encoder agrees with our decoder; these bytes are what a
  * Blackwell GPU would actually consume. The `int2x`, `scale_mant` and
  * `scale_exp` fields of the vector file are convention-free integers involving
  * no float at all, so they are the authoritative fields for hardware and the
  * expected values here are derived from them with exact BigInt arithmetic.
  */
class PESpec extends AnyFlatSpec with ChiselScalatestTester {
  import NVFP4Params._

  private case class Blk(packed: BigInt, scaleCode: Int, mant: Int, exp: Int,
                         int2x: Seq[Int])

  private val vectorPath = sys.env.getOrElse(
    "NVFP4_VECTORS",
    "/scratch2/loren/modelblaster/tests/nvfp4_vectors/nvfp4_blocks.hex")

  private lazy val blocks: Seq[Blk] = {
    val f = new File(vectorPath)
    if (!f.exists) Seq.empty
    else {
      val src = Source.fromFile(f)
      try {
        src.getLines().filter(_.startsWith("B ")).map { line =>
          val t = line.trim.split("\\s+")
          // B <idx> <packed> <scale_code> <ws2_bits> <mant> <exp> <int2x x16> ...
          Blk(BigInt(t(2), 16), Integer.parseInt(t(3), 16),
              t(5).toInt, t(6).toInt,
              (0 until 16).map(k => t(7 + k).toInt))
        }.toList
      } finally src.close()
    }
  }

  /** Exact expected aligned term, in units of 2^-20, by definition. */
  private def expectedTerm(a: Blk, b: Blk): BigInt = {
    val dot = a.int2x.zip(b.int2x).map { case (x, y) => BigInt(x) * BigInt(y) }.sum
    val shift = a.exp + b.exp - 2 + ACC_LSB_SHIFT
    require(shift >= 0, s"shift $shift went negative — ACC_LSB_SHIFT is wrong")
    (dot * BigInt(a.mant) * BigInt(b.mant)) << shift
  }

  behavior of "the frozen vectors"

  it should "be present and parseable" in {
    assume(blocks.nonEmpty, s"vectors not found at $vectorPath — skipping")
    assert(blocks.length >= 64, s"only ${blocks.length} blocks parsed")
    assert(blocks.forall(_.int2x.length == 16))
    // int2x must be drawn from the exact E2M1 2x alphabet, nothing else.
    val allowed = Set(0, 1, 2, 3, 4, 6, 8, 12)
    assert(blocks.forall(_.int2x.forall(v => allowed.contains(math.abs(v)))),
           "a real block contained a value outside the E2M1 alphabet")
  }

  behavior of "E4M3Decode against shipped scale bytes"

  it should "reproduce NVIDIA's exact (mantissa, exponent) for every real scale" in {
    assume(blocks.nonEmpty, "vectors not found — skipping")
    test(new E4M3Decode) { c =>
      for (b <- blocks) {
        c.io.code.poke(b.scaleCode.U)
        c.io.mant.expect(b.mant.U, f"scale byte 0x${b.scaleCode}%02x")
        c.io.exp.expect(b.exp.S, f"scale byte 0x${b.scaleCode}%02x")
      }
    }
  }

  behavior of "BlockDot16 against shipped packed nibbles"

  it should "agree with NVIDIA's int2x decoding on real data" in {
    // This is the load-bearing one: it proves our nibble unpacking and E2M1
    // decode agree with what NVIDIA actually shipped, on production weights.
    assume(blocks.nonEmpty, "vectors not found — skipping")
    test(new BlockDot16) { c =>
      for (Seq(a, b) <- blocks.take(256).grouped(2).filter(_.length == 2)) {
        val expected = a.int2x.zip(b.int2x).map { case (x, y) => x * y }.sum
        c.io.a.poke(a.packed.U)
        c.io.b.poke(b.packed.U)
        c.io.out.expect(expected.S)
      }
    }
  }

  behavior of "PE"

  it should "accumulate one real block pair to the exact expected term" in {
    assume(blocks.nonEmpty, "vectors not found — skipping")
    test(new PE) { c =>
      for (Seq(a, b) <- blocks.take(128).grouped(2).filter(_.length == 2)) {
        c.io.clear.poke(true.B); c.io.en.poke(false.B); c.clock.step()
        c.io.clear.poke(false.B); c.io.en.poke(true.B)
        c.io.a.poke(a.packed.U);  c.io.sA.poke(a.scaleCode.U)
        c.io.b.poke(b.packed.U);  c.io.sB.poke(b.scaleCode.U)
        c.clock.step()
        c.io.en.poke(false.B)
        c.io.acc.expect(expectedTerm(a, b).S, "single-term accumulation")
        c.io.oor.expect(false.B, "shift went out of range on real data")
      }
    }
  }

  it should "accumulate a K-deep run exactly" in {
    assume(blocks.length >= 32, "not enough vectors — skipping")
    val as = blocks.slice(0, 16)
    val bs = blocks.slice(16, 32)
    val expected = as.zip(bs).map { case (a, b) => expectedTerm(a, b) }.sum
    test(new PE) { c =>
      c.io.clear.poke(true.B); c.clock.step(); c.io.clear.poke(false.B)
      c.io.en.poke(true.B)
      for ((a, b) <- as.zip(bs)) {
        c.io.a.poke(a.packed.U); c.io.sA.poke(a.scaleCode.U)
        c.io.b.poke(b.packed.U); c.io.sB.poke(b.scaleCode.U)
        c.clock.step()
      }
      c.io.en.poke(false.B)
      c.io.acc.expect(expected.S)
    }
  }

  it should "be bit-identical under any K-block order" in {
    // The property the whole bit-exactness claim rests on: the compiler may
    // split and reorder K however it likes and the accumulator cannot change.
    assume(blocks.length >= 32, "not enough vectors — skipping")
    val as = blocks.slice(0, 16)
    val bs = blocks.slice(16, 32)
    val perm = new scala.util.Random(7).shuffle((0 until 16).toList)

    def run(order: Seq[Int]): BigInt = {
      var out: BigInt = BigInt(0)
      test(new PE) { c =>
        c.io.clear.poke(true.B); c.clock.step(); c.io.clear.poke(false.B)
        c.io.en.poke(true.B)
        for (i <- order) {
          c.io.a.poke(as(i).packed.U); c.io.sA.poke(as(i).scaleCode.U)
          c.io.b.poke(bs(i).packed.U); c.io.sB.poke(bs(i).scaleCode.U)
          c.clock.step()
        }
        c.io.en.poke(false.B)
        out = c.io.acc.peek().litValue
      }
      out
    }
    assert(run(0 until 16) == run(perm), "accumulator depended on block order")
  }
}
