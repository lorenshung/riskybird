package ospi

import chisel3._
import chisel3.util._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import scala.collection.mutable.ArrayBuffer

/** Single-clock harness around the dual-clock [[AsyncFifo]] (both clocks tied to the sim clock).
  * This exercises the Gray-pointer / full-empty / overflow logic and pointer wraparound; genuine
  * async-frequency behavior is out of scope for chiseltest. */
class AsyncFifoHarness(val depth: Int = 8, val syncStages: Int = 3) extends Module {
  val io = IO(new Bundle {
    val enq           = Flipped(Decoupled(UInt(8.W)))
    val deq           = Decoupled(UInt(8.W))
    val overflow      = Output(Bool())
    val clearOverflow = Input(Bool())
  })
  val fifo = Module(new AsyncFifo(UInt(8.W), depth, syncStages))
  fifo.enqClock := clock
  fifo.enqReset := reset.asAsyncReset
  fifo.deqClock := clock
  fifo.deqReset := reset.asAsyncReset
  fifo.enq <> io.enq
  io.deq <> fifo.deq
  io.overflow := fifo.overflow
  fifo.clearOverflow := io.clearOverflow
}

class AsyncFifoSpec extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "AsyncFifo"

  it should "be empty at reset" in {
    test(new AsyncFifoHarness()) { c =>
      c.io.enq.valid.poke(false.B)
      c.io.deq.ready.poke(false.B)
      c.io.clearOverflow.poke(false.B)
      c.clock.step(5)
      assert(!c.io.deq.valid.peek().litToBoolean, "FIFO should be empty after reset")
      assert(!c.io.overflow.peek().litToBoolean, "no overflow after reset")
    }
  }

  it should "preserve order and data across many items (pointer wraparound)" in {
    test(new AsyncFifoHarness(depth = 8)) { c =>
      c.io.clearOverflow.poke(false.B)
      var nextVal = 0
      val enqd = ArrayBuffer[Int]()
      val deqd = ArrayBuffer[Int]()
      // Concurrent produce/consume so total throughput >> depth -> pointers wrap repeatedly.
      for (_ <- 0 until 200) {
        c.io.enq.valid.poke(true.B)
        c.io.enq.bits.poke((nextVal & 0xff).U)
        c.io.deq.ready.poke(true.B)
        val enqFire = c.io.enq.valid.peek().litToBoolean && c.io.enq.ready.peek().litToBoolean
        val deqFire = c.io.deq.valid.peek().litToBoolean && c.io.deq.ready.peek().litToBoolean
        if (deqFire) deqd += c.io.deq.bits.peek().litValue.toInt
        c.clock.step(1)
        if (enqFire) { enqd += (nextVal & 0xff); nextVal += 1 }
      }
      // Drain whatever is still buffered.
      c.io.enq.valid.poke(false.B)
      for (_ <- 0 until 40) {
        c.io.deq.ready.poke(true.B)
        if (c.io.deq.valid.peek().litToBoolean) deqd += c.io.deq.bits.peek().litValue.toInt
        c.clock.step(1)
      }
      println(s"  AsyncFifo wraparound: enqueued ${enqd.length}, dequeued ${deqd.length} (depth ${c.depth})")
      assert(enqd.length > c.depth, s"test must push more than depth(${c.depth}) items to exercise wraparound")
      assert(deqd.toSeq == enqd.toSeq, "dequeued sequence must equal enqueued sequence, in order")
    }
  }

  it should "assert sticky overflow when enqueuing into a full FIFO, then clear it" in {
    test(new AsyncFifoHarness(depth = 8)) { c =>
      c.io.deq.ready.poke(false.B)   // never drain -> FIFO fills
      c.io.clearOverflow.poke(false.B)
      var accepted = 0
      for (i <- 0 until 24) {
        c.io.enq.valid.poke(true.B)
        c.io.enq.bits.poke((i & 0xff).U)
        if (c.io.enq.ready.peek().litToBoolean) accepted += 1
        c.clock.step(1)
      }
      println(s"  accepted $accepted items into depth-${c.depth} FIFO before overflow")
      assert(accepted <= c.depth, s"must not accept more than depth(${c.depth}) items")
      assert(c.io.overflow.peek().litToBoolean, "overflow must be set after offering into a full FIFO")

      // Stop offering + drain + clear -> sticky overflow goes away.
      c.io.enq.valid.poke(false.B)
      c.io.deq.ready.poke(true.B)
      c.io.clearOverflow.poke(true.B)
      c.clock.step(c.syncStages + 4)
      c.io.clearOverflow.poke(false.B)
      c.clock.step(2)
      assert(!c.io.overflow.peek().litToBoolean, "overflow must clear after clearOverflow")
    }
  }

  it should "respect backpressure (no data loss when consumer stalls intermittently)" in {
    test(new AsyncFifoHarness(depth = 8)) { c =>
      c.io.clearOverflow.poke(false.B)
      var nextVal = 0
      val enqd = ArrayBuffer[Int]()
      val deqd = ArrayBuffer[Int]()
      for (cyc <- 0 until 300) {
        c.io.enq.valid.poke(true.B)
        c.io.enq.bits.poke((nextVal & 0xff).U)
        // Consumer ready only on ~1/3 of cycles -> producer must back off, FIFO must not drop.
        c.io.deq.ready.poke((cyc % 3 == 0).B)
        val enqFire = c.io.enq.valid.peek().litToBoolean && c.io.enq.ready.peek().litToBoolean
        val deqFire = c.io.deq.valid.peek().litToBoolean && c.io.deq.ready.peek().litToBoolean
        if (deqFire) deqd += c.io.deq.bits.peek().litValue.toInt
        c.clock.step(1)
        if (enqFire) { enqd += (nextVal & 0xff); nextVal += 1 }
      }
      c.io.enq.valid.poke(false.B)
      for (_ <- 0 until 60) {
        c.io.deq.ready.poke(true.B)
        if (c.io.deq.valid.peek().litToBoolean) deqd += c.io.deq.bits.peek().litValue.toInt
        c.clock.step(1)
      }
      // Note: `overflow` may set here because a Decoupled producer legitimately holds `valid` high
      // while the FIFO is full (waiting for `ready`); that flag targets the can't-be-stalled Valid
      // producer (the sensor). The real backpressure guarantee is no data loss / reorder:
      assert(deqd.toSeq == enqd.toSeq, "no data loss/reorder under backpressure")
      assert(enqd.length > c.depth, "test must push more than depth to be meaningful")
    }
  }
}
