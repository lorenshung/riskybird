package ospi

import chisel3._
import chisel3.util._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import scala.collection.mutable.ArrayBuffer

/** Single-clock test harness: ties the sensor PCLK to the (only) simulation clock so the whole
  * design runs off one clock in chiseltest. This fully exercises the capture front-end, the CDC
  * FIFO (in same-clock mode), and the system-domain frame assembler — enough to prove
  * "the FPGA receives correct whole images". Genuine async-frequency CDC closure is a separate,
  * later concern (the FIFO is a textbook Gray-pointer design).
  */
class HarnessIO(p: CaptureParams) extends Bundle {
  val ctrl   = new ControlIO(p)
  val status = new StatusIO(p)
  val fvld   = Input(Bool())
  val lvld   = Input(Bool())
  val d      = Input(UInt(p.dataWidth.W))
  val intr   = Input(Bool())
  val pixels = Decoupled(new PixelFlit(p.dataWidth))
}

class CaptureTestHarness(val p: CaptureParams) extends Module {
  val io = IO(new HarnessIO(p))
  val dut = Module(new HM01B0Capture(p))
  dut.io.ctrl        := io.ctrl
  io.status          := dut.io.status
  dut.io.sensor.pclk := clock
  dut.io.sensor.fvld := io.fvld
  dut.io.sensor.lvld := io.lvld
  dut.io.sensor.d    := io.d
  dut.io.sensor.intr := io.intr
  io.pixels <> dut.io.pixels
}

class HM01B0CaptureSpec extends AnyFlatSpec with ChiselScalatestTester {
  private val p = CaptureParams()

  /** A collected beat: (data, sof, eol, eof). */
  private type Beat = (Int, Boolean, Boolean, Boolean)

  private def configure(c: CaptureTestHarness, w: Int, h: Int): Unit = {
    c.io.ctrl.enable.poke(true.B)
    c.io.ctrl.continuous.poke(true.B)
    c.io.ctrl.trigPulse.poke(false.B)
    c.io.ctrl.clear.poke(false.B)
    c.io.ctrl.irqEnable.poke(true.B)
    c.io.ctrl.mclkDiv.poke(0.U)
    c.io.ctrl.expWidth.poke(w.U)
    c.io.ctrl.expHeight.poke(h.U)
    c.io.pixels.ready.poke(true.B)
    c.io.fvld.poke(false.B); c.io.lvld.poke(false.B); c.io.d.poke(0.U); c.io.intr.poke(false.B)
    c.clock.step(5)
  }

  /** Drive one frame of `w`x`h` pixels (value = px(r,col)) with realistic blanking, collecting beats. */
  private def driveFrame(c: CaptureTestHarness, w: Int, h: Int, px: (Int, Int) => Int,
                         out: ArrayBuffer[Beat]): Unit = {
    def tick(): Unit = {
      if (c.io.pixels.valid.peek().litToBoolean && c.io.pixels.ready.peek().litToBoolean) {
        val b = c.io.pixels.bits
        out += ((b.data.peek().litValue.toInt, b.sof.peek().litToBoolean,
                 b.eol.peek().litToBoolean, b.eof.peek().litToBoolean))
      }
      c.clock.step(1)
    }
    c.io.fvld.poke(true.B); c.io.lvld.poke(false.B); c.io.d.poke(0.U)
    tick(); tick() // vertical front porch (FVLD high, no active line yet)
    for (r <- 0 until h) {
      c.io.lvld.poke(true.B)
      for (col <- 0 until w) { c.io.d.poke((px(r, col) & 0xff).U); tick() }
      c.io.lvld.poke(false.B); c.io.d.poke(0.U)
      tick(); tick() // horizontal blanking
    }
    c.io.fvld.poke(false.B)       // end of frame
    for (_ <- 0 until 40) tick()  // flush front-end pipeline + CDC FIFO latency
  }

  /** Parse collected beats into rows of pixels; assert framing invariants. Returns the image. */
  private def reconstruct(beats: scala.collection.Seq[Beat], w: Int, h: Int): Seq[Seq[Int]] = {
    assert(beats.count(_._4) == 1, s"expected exactly 1 EOF marker, got ${beats.count(_._4)}")
    val pix = beats.filterNot(_._4)
    assert(pix.nonEmpty && pix.head._2, "first pixel beat must assert SOF")
    val rows = ArrayBuffer[ArrayBuffer[Int]]()
    var cur  = ArrayBuffer[Int]()
    for ((d, _, eol, _) <- pix) { cur += d; if (eol) { rows += cur; cur = ArrayBuffer[Int]() } }
    assert(cur.isEmpty, s"trailing ${cur.length} pixels with no EOL")
    rows.map(_.toSeq).toSeq
  }

  // ---- pretty-printers so the captured data is visible during the run ----

  /** Render an image as a labeled grid of hex bytes. */
  private def hexGrid(img: Seq[Seq[Int]]): String = {
    if (img.isEmpty) return "        (empty)"
    val w = img.map(_.length).max
    val header = "        " + (0 until w).map(col => f"c$col%-2d").mkString(" ")
    val rows = img.zipWithIndex.map { case (row, r) =>
      f"  r$r%-4d: " + row.map(v => f"$v%02x ").mkString
    }
    (header +: rows).mkString("\n")
  }

  /** Print expected vs captured frame + stream/match summary, then assert exact equality. */
  private def showAndVerifyFrame(label: String, w: Int, h: Int, px: (Int, Int) => Int,
                                 beats: ArrayBuffer[Beat]): Seq[Seq[Int]] = {
    val img      = reconstruct(beats, w, h)
    val expected = Seq.tabulate(h, w)((r, col) => px(r, col) & 0xff)
    val nPix = beats.count(!_._4)
    val nSof = beats.count(b => !b._4 && b._2)
    val nEol = beats.count(b => !b._4 && b._3)
    val nEof = beats.count(_._4)
    val matches = (for {
      r   <- 0 until h
      col <- 0 until w
      if r < img.length && col < img(r).length && img(r)(col) == expected(r)(col)
    } yield 1).sum

    println(s"\n--- $label ---")
    println(s"  beat stream: $nPix pixel beats, $nSof SOF, $nEol EOL, $nEof EOF marker")
    val capW = if (img.nonEmpty) img.head.length else 0
    val capH = img.length
    println(s"  EXPECTED (${w}x${h}, WxH):")
    println(hexGrid(expected))
    println(s"  CAPTURED (${capW}x${capH}, WxH):")
    println(hexGrid(img))
    println(s"  per-pixel match: $matches/${w * h}  " + (if (matches == w * h) "✓ all pixels verified" else "✗ MISMATCH"))

    assert(img.length == h, s"height ${img.length} != $h")
    for (r <- 0 until h) {
      assert(img(r).length == w, s"row $r width ${img(r).length} != $w")
      for (col <- 0 until w)
        assert(img(r)(col) == expected(r)(col), s"pixel($r,$col)=${img(r)(col)} != ${expected(r)(col)}")
    }
    img
  }

  /** Print the status registers as the SoC would read them. */
  private def showStatus(c: CaptureTestHarness): Unit = {
    println(s"  status regs: frameCount=${c.io.status.frameCount.peek().litValue}" +
            s" lastWidth=${c.io.status.lastWidth.peek().litValue}" +
            s" lastHeight=${c.io.status.lastHeight.peek().litValue}" +
            s" overflow=${c.io.status.overflow.peek().litToBoolean}" +
            s" geomErr=${c.io.status.geomErr.peek().litToBoolean}")
  }

  behavior of "HM01B0Capture"

  it should "capture a full frame exactly (8x4)" in {
    test(new CaptureTestHarness(p)) { c =>
      val (w, h) = (8, 4)
      def px(r: Int, col: Int) = r * w + col
      configure(c, w, h)
      val beats = ArrayBuffer[Beat]()
      driveFrame(c, w, h, px, beats)

      showAndVerifyFrame("frame (8x4): pixel value = row*width + col", w, h, px, beats)
      showStatus(c)

      assert(c.io.status.frameCount.peek().litValue == 1, "frameCount should be 1")
      assert(c.io.status.lastWidth.peek().litValue == w, "lastWidth mismatch")
      assert(c.io.status.lastHeight.peek().litValue == h, "lastHeight mismatch")
      assert(!c.io.status.overflow.peek().litToBoolean, "unexpected overflow")
      assert(!c.io.status.geomErr.peek().litToBoolean, "unexpected geomErr")
    }
  }

  it should "capture two back-to-back frames" in {
    test(new CaptureTestHarness(p)) { c =>
      val (w, h) = (6, 3)
      def px0(r: Int, col: Int) = r * w + col
      def px1(r: Int, col: Int) = 100 + r * w + col
      configure(c, w, h)

      val b0 = ArrayBuffer[Beat](); driveFrame(c, w, h, px0, b0)
      showAndVerifyFrame("frame #0 (6x3): value = row*width + col", w, h, px0, b0)
      showStatus(c)

      val b1 = ArrayBuffer[Beat](); driveFrame(c, w, h, px1, b1)
      showAndVerifyFrame("frame #1 (6x3): value = 100 + row*width + col", w, h, px1, b1)
      showStatus(c)

      assert(c.io.status.frameCount.peek().litValue == 2, "frameCount should be 2")
    }
  }

  it should "flag geomErr when the frame size differs from expected" in {
    test(new CaptureTestHarness(p)) { c =>
      val (w, h) = (8, 4)
      def px(r: Int, col: Int) = r * w + col
      configure(c, w, h)
      c.io.ctrl.expHeight.poke((h + 1).U) // expect one more line than will arrive
      val beats = ArrayBuffer[Beat]()
      driveFrame(c, w, h, px, beats)

      // Data still reconstructs correctly...
      showAndVerifyFrame("frame (8x4) with expHeight=5 (one too many)", w, h, px, beats)
      showStatus(c)

      // ...but the geometry-mismatch flag is set.
      assert(c.io.status.geomErr.peek().litToBoolean, "geomErr should be set")
    }
  }
}
