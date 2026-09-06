package ospi

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import scala.collection.mutable.ArrayBuffer

/** Drives the PCLK-domain capture FSM directly (no FIFO/assembler) and checks the in-band framing
  * markers it produces: SOF on the first pixel, EOL on each line's last pixel, and exactly one EOF
  * marker per frame. */
class CaptureFrontendSpec extends AnyFlatSpec with ChiselScalatestTester {
  private type Beat = (Int, Boolean, Boolean, Boolean) // (data, sof, eol, eof)
  private val p = CaptureParams()

  /** Drive one frame into the frontend, collecting every Valid beat. */
  private def driveFrame(c: CaptureFrontend, w: Int, h: Int, px: (Int, Int) => Int,
                         out: ArrayBuffer[Beat], enable: Boolean = true): Unit = {
    c.io.enable.poke(enable.B)
    def tick(): Unit = {
      if (c.io.out.valid.peek().litToBoolean) {
        val b = c.io.out.bits
        out += ((b.data.peek().litValue.toInt, b.sof.peek().litToBoolean,
                 b.eol.peek().litToBoolean, b.eof.peek().litToBoolean))
      }
      c.clock.step(1)
    }
    c.io.fvld.poke(true.B); c.io.lvld.poke(false.B); c.io.d.poke(0.U)
    tick(); tick() // vertical front porch
    for (r <- 0 until h) {
      c.io.lvld.poke(true.B)
      for (col <- 0 until w) { c.io.d.poke((px(r, col) & 0xff).U); tick() }
      c.io.lvld.poke(false.B); c.io.d.poke(0.U)
      tick(); tick() // horizontal blanking
    }
    c.io.fvld.poke(false.B)
    for (_ <- 0 until 10) tick() // flush + EOF marker
  }

  /** Group pixel beats into rows by EOL; returns (image, #EOF markers). */
  private def parse(beats: scala.collection.Seq[Beat]): (Seq[Seq[Int]], Int) = {
    val eofCount = beats.count(_._4)
    val rows = ArrayBuffer[ArrayBuffer[Int]]()
    var cur = ArrayBuffer[Int]()
    for ((d, _, eol, eof) <- beats if !eof) { cur += d; if (eol) { rows += cur; cur = ArrayBuffer[Int]() } }
    (rows.map(_.toSeq).toSeq, eofCount)
  }

  behavior of "CaptureFrontend"

  it should "produce correct pixel order + SOF/EOL/EOF markers (3x2)" in {
    test(new CaptureFrontend(p)) { c =>
      val (w, h) = (3, 2)
      def px(r: Int, col: Int) = r * w + col
      val beats = ArrayBuffer[Beat](); driveFrame(c, w, h, px, beats)
      val (img, nEof) = parse(beats)
      assert(beats.head._2, "first beat must assert SOF")
      assert(nEof == 1, s"exactly one EOF marker, got $nEof")
      assert(img.length == h && img.forall(_.length == w), s"geometry should be ${w}x${h}, got ${img.map(_.length)}")
      for (r <- 0 until h; col <- 0 until w) assert(img(r)(col) == px(r, col), s"pixel($r,$col)")
      println(s"  3x2 frame -> ${beats.count(!_._4)} pixels, ${beats.count(b => b._3 && !b._4)} EOLs, $nEof EOF")
    }
  }

  it should "handle a 1x1 frame (single pixel is both SOF and EOL)" in {
    test(new CaptureFrontend(p)) { c =>
      val beats = ArrayBuffer[Beat](); driveFrame(c, 1, 1, (_, _) => 0xAB, beats)
      val pix = beats.filterNot(_._4)
      assert(pix.length == 1, s"expected 1 pixel beat, got ${pix.length}")
      assert(pix.head == (0xAB, true, true, false), s"single pixel must be SOF+EOL: ${pix.head}")
      assert(beats.count(_._4) == 1, "exactly one EOF marker")
    }
  }

  it should "emit no beats while disabled" in {
    test(new CaptureFrontend(p)) { c =>
      val beats = ArrayBuffer[Beat](); driveFrame(c, 4, 3, (r, col) => r * 4 + col, beats, enable = false)
      assert(beats.isEmpty, s"disabled frontend must emit nothing, got ${beats.length} beats")
    }
  }

  it should "emit exactly one EOF and re-arm SOF for back-to-back frames" in {
    test(new CaptureFrontend(p)) { c =>
      val (w, h) = (4, 2)
      val b0 = ArrayBuffer[Beat](); driveFrame(c, w, h, (r, col) => r * w + col, b0)
      val b1 = ArrayBuffer[Beat](); driveFrame(c, w, h, (r, col) => 50 + r * w + col, b1)
      for (b <- Seq(b0, b1)) {
        assert(b.count(_._4) == 1, "one EOF per frame")
        assert(b.filterNot(_._4).head._2, "each frame's first pixel asserts SOF")
        assert(b.filterNot(_._4).count(_._2) == 1, "exactly one SOF per frame")
      }
      val (img1, _) = parse(b1)
      for (r <- 0 until h; col <- 0 until w) assert(img1(r)(col) == 50 + r * w + col, s"frame1 pixel($r,$col)")
    }
  }
}
