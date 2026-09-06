package ospi

import chisel3._
import chisel3.util._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import scala.collection.mutable.ArrayBuffer

/** Harness exposing the control outputs (trig) in addition to status + the captured stream, so the
  * non-video control path can be observed. (mclk is a generated Clock and not peeked here.) */
class OspiFullHarness(val p: CaptureParams) extends Module {
  val io = IO(new Bundle {
    val ctrl   = new ControlIO(p)
    val status = new StatusIO(p)
    val fvld   = Input(Bool())
    val lvld   = Input(Bool())
    val d      = Input(UInt(p.dataWidth.W))
    val intr   = Input(Bool())
    val pixels = Decoupled(new PixelFlit(p.dataWidth))
    val trig   = Output(Bool())
  })
  val dut = Module(new HM01B0Capture(p))
  dut.io.ctrl        := io.ctrl
  io.status          := dut.io.status
  dut.io.sensor.pclk := clock
  dut.io.sensor.fvld := io.fvld
  dut.io.sensor.lvld := io.lvld
  dut.io.sensor.d    := io.d
  dut.io.sensor.intr := io.intr
  io.pixels <> dut.io.pixels
  io.trig := dut.io.sensor.trig
}

class HM01B0CaptureStressSpec extends AnyFlatSpec with ChiselScalatestTester {
  private type Beat = (Int, Boolean, Boolean, Boolean)

  private def configure(c: CaptureTestHarness, w: Int, h: Int): Unit = {
    c.io.ctrl.enable.poke(true.B); c.io.ctrl.continuous.poke(true.B)
    c.io.ctrl.trigPulse.poke(false.B); c.io.ctrl.clear.poke(false.B); c.io.ctrl.irqEnable.poke(false.B)
    c.io.ctrl.mclkDiv.poke(0.U); c.io.ctrl.expWidth.poke(w.U); c.io.ctrl.expHeight.poke(h.U)
    c.io.pixels.ready.poke(true.B)
    c.io.fvld.poke(false.B); c.io.lvld.poke(false.B); c.io.d.poke(0.U); c.io.intr.poke(false.B)
    c.clock.step(5)
  }

  /** Drive one frame. If `drain` is false, hold pixels.ready low (consumer stalled) and do not collect. */
  private def driveFrame(c: CaptureTestHarness, w: Int, h: Int, px: (Int, Int) => Int,
                         out: ArrayBuffer[Beat], drain: Boolean = true): Unit = {
    c.io.pixels.ready.poke(drain.B)
    def tick(): Unit = {
      if (drain && c.io.pixels.valid.peek().litToBoolean) {
        val b = c.io.pixels.bits
        out += ((b.data.peek().litValue.toInt, b.sof.peek().litToBoolean,
                 b.eol.peek().litToBoolean, b.eof.peek().litToBoolean))
      }
      c.clock.step(1)
    }
    c.io.fvld.poke(true.B); c.io.lvld.poke(false.B); c.io.d.poke(0.U)
    tick(); tick()
    for (r <- 0 until h) {
      c.io.lvld.poke(true.B)
      for (col <- 0 until w) { c.io.d.poke((px(r, col) & 0xff).U); tick() }
      c.io.lvld.poke(false.B); c.io.d.poke(0.U)
      tick(); tick()
    }
    c.io.fvld.poke(false.B)
    for (_ <- 0 until 40) tick()
  }

  private def reconstruct(beats: scala.collection.Seq[Beat]): Seq[Seq[Int]] = {
    val rows = ArrayBuffer[ArrayBuffer[Int]](); var cur = ArrayBuffer[Int]()
    for ((d, _, eol, eof) <- beats if !eof) { cur += d; if (eol) { rows += cur; cur = ArrayBuffer[Int]() } }
    rows.map(_.toSeq).toSeq
  }

  behavior of "HM01B0Capture (stress)"

  it should "capture a non-square frame (10x3) and report its geometry" in {
    test(new CaptureTestHarness(CaptureParams())) { c =>
      val (w, h) = (10, 3)
      def px(r: Int, col: Int) = r * w + col
      configure(c, w, h)
      val beats = ArrayBuffer[Beat](); driveFrame(c, w, h, px, beats)
      val img = reconstruct(beats)
      assert(img.length == h && img.forall(_.length == w), s"expected ${w}x${h}, got ${img.map(_.length)}")
      for (r <- 0 until h; col <- 0 until w) assert(img(r)(col) == px(r, col), s"pixel($r,$col)")
      assert(c.io.status.lastWidth.peek().litValue == w, "lastWidth")
      assert(c.io.status.lastHeight.peek().litValue == h, "lastHeight")
      assert(!c.io.status.geomErr.peek().litToBoolean, "no geomErr for matching geometry")
    }
  }

  it should "set sticky overflow when the consumer stalls, then clear it" in {
    test(new CaptureTestHarness(CaptureParams(fifoDepth = 16))) { c =>
      val (w, h) = (8, 6) // 48 active pixels >> 16-deep FIFO
      configure(c, w, h)
      val beats = ArrayBuffer[Beat]()
      driveFrame(c, w, h, (r, col) => r * w + col, beats, drain = false) // never drain -> FIFO overflows
      assert(c.io.status.overflow.peek().litToBoolean, "overflow must set when the FIFO is starved")

      // Resume draining + clear the sticky flag.
      c.io.pixels.ready.poke(true.B)
      c.io.ctrl.clear.poke(true.B)
      c.clock.step(12)
      c.io.ctrl.clear.poke(false.B)
      c.clock.step(4)
      assert(!c.io.status.overflow.peek().litToBoolean, "overflow must clear after ctrl.clear")
    }
  }

  it should "ignore frames while disabled (frameCount only counts enabled frames)" in {
    test(new CaptureTestHarness(CaptureParams())) { c =>
      val (w, h) = (6, 2)
      def px(r: Int, col: Int) = r * w + col
      configure(c, w, h)

      val b0 = ArrayBuffer[Beat](); driveFrame(c, w, h, px, b0)
      assert(c.io.status.frameCount.peek().litValue == 1, "first enabled frame counted")

      c.io.ctrl.enable.poke(false.B); c.clock.step(5)
      val bIgn = ArrayBuffer[Beat](); driveFrame(c, w, h, px, bIgn)
      assert(c.io.status.frameCount.peek().litValue == 1, "frame while disabled must not be counted")

      c.io.ctrl.enable.poke(true.B); c.clock.step(5)
      val b1 = ArrayBuffer[Beat](); driveFrame(c, w, h, px, b1)
      assert(c.io.status.frameCount.peek().litValue == 2, "frame after re-enable counted")
    }
  }

  it should "pass control/sideband signals through (trig, sensorInt, busy)" in {
    test(new OspiFullHarness(CaptureParams())) { c =>
      // defaults
      c.io.ctrl.enable.poke(true.B); c.io.ctrl.continuous.poke(true.B)
      c.io.ctrl.trigPulse.poke(false.B); c.io.ctrl.clear.poke(false.B); c.io.ctrl.irqEnable.poke(false.B)
      c.io.ctrl.mclkDiv.poke(3.U); c.io.ctrl.expWidth.poke(8.U); c.io.ctrl.expHeight.poke(8.U)
      c.io.pixels.ready.poke(true.B)
      c.io.fvld.poke(false.B); c.io.lvld.poke(false.B); c.io.d.poke(0.U); c.io.intr.poke(false.B)
      c.clock.step(5)

      // TRIG: in continuous mode it stays low even if trigPulse is set...
      c.io.ctrl.trigPulse.poke(true.B); c.clock.step(2)
      assert(!c.io.trig.peek().litToBoolean, "trig stays low in continuous mode")
      // ...in snapshot (continuous=false) trigPulse drives trig.
      c.io.ctrl.continuous.poke(false.B); c.clock.step(2)
      assert(c.io.trig.peek().litToBoolean, "trig follows trigPulse in snapshot mode")

      // sensorInt: synchronized passthrough of the sensor INT pin.
      c.io.intr.poke(true.B); c.clock.step(3)
      assert(c.io.status.sensorInt.peek().litToBoolean, "sensorInt reflects intr")
      c.io.intr.poke(false.B); c.clock.step(3)
      assert(!c.io.status.sensorInt.peek().litToBoolean, "sensorInt clears")

      // busy: synchronized FVLD level.
      c.io.fvld.poke(true.B); c.clock.step(3)
      assert(c.io.status.busy.peek().litToBoolean, "busy reflects FVLD high")
      c.io.fvld.poke(false.B); c.clock.step(3)
      assert(!c.io.status.busy.peek().litToBoolean, "busy clears when FVLD low")
    }
  }
}
