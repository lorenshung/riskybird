package ospi

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import scala.collection.mutable.ArrayBuffer

class FrameBufferSpec extends AnyFlatSpec with ChiselScalatestTester {
  private val p = CaptureParams(maxWidth = 8, maxHeight = 8, defaultWidth = 8, defaultHeight = 8)
  private type Beat = (Int, Boolean, Boolean, Boolean)

  private def init(c: FrameBuffer): Unit = {
    c.io.enq.valid.poke(false.B)
    c.io.enq.bits.data.poke(0.U)
    c.io.enq.bits.sof.poke(false.B)
    c.io.enq.bits.eol.poke(false.B)
    c.io.enq.bits.eof.poke(false.B)
    c.io.deq.ready.poke(false.B)
    c.io.flush.poke(false.B)
    c.clock.step(2)
  }

  private def enqueue(c: FrameBuffer, beat: Beat): Unit = {
    val (data, sof, eol, eof) = beat
    c.io.enq.valid.poke(true.B)
    c.io.enq.bits.data.poke(data.U)
    c.io.enq.bits.sof.poke(sof.B)
    c.io.enq.bits.eol.poke(eol.B)
    c.io.enq.bits.eof.poke(eof.B)
    while (!c.io.enq.ready.peek().litToBoolean) c.clock.step(1)
    assert(c.io.frameReady.peek().litToBoolean == eof,
      "frameReady must pulse exactly when an EOF marker is accepted")
    c.clock.step(1)
    c.io.enq.valid.poke(false.B)
  }

  private def drain(c: FrameBuffer, n: Int): Seq[Beat] = {
    val out = ArrayBuffer[Beat]()
    c.io.deq.ready.poke(true.B)
    while (out.length < n) {
      if (c.io.deq.valid.peek().litToBoolean) {
        out += ((
          c.io.deq.bits.data.peek().litValue.toInt,
          c.io.deq.bits.sof.peek().litToBoolean,
          c.io.deq.bits.eol.peek().litToBoolean,
          c.io.deq.bits.eof.peek().litToBoolean))
      }
      c.clock.step(1)
    }
    c.io.deq.ready.poke(false.B)
    out.toSeq
  }

  behavior of "FrameBuffer"

  it should "accept a whole frame before software drains it" in {
    test(new FrameBuffer(p, depth = 8, useSyncReadMem = true)) { c =>
      init(c)
      val frame = Seq[Beat](
        (0x11, true, false, false),
        (0x22, false, false, false),
        (0x33, false, true, false),
        (0x00, false, false, true))

      frame.foreach(enqueue(c, _))
      assert(c.io.count.peek().litValue == frame.length)
      assert(c.io.deq.valid.peek().litToBoolean)
      assert(drain(c, frame.length) == frame, "buffer must preserve pixels and in-band markers")
      c.clock.step(1)
      assert(c.io.count.peek().litValue == 0)
      assert(!c.io.deq.valid.peek().litToBoolean)
    }
  }

  it should "backpressure at capacity and discard all queued data on flush" in {
    test(new FrameBuffer(p, depth = 4, useSyncReadMem = true)) { c =>
      init(c)
      (0 until 4).foreach(i => enqueue(c, (i, i == 0, i == 3, false)))
      assert(c.io.full.peek().litToBoolean)
      assert(!c.io.enq.ready.peek().litToBoolean)
      assert(c.io.count.peek().litValue == 4)

      c.io.flush.poke(true.B)
      c.clock.step(1)
      c.io.flush.poke(false.B)
      assert(c.io.count.peek().litValue == 0)
      assert(!c.io.deq.valid.peek().litToBoolean)
      assert(!c.io.full.peek().litToBoolean)
    }
  }
}
