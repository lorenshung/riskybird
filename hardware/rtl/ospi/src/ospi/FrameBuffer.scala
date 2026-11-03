package ospi

import chisel3._
import chisel3.util._

/** System-clock-domain storage between the capture CDC FIFO and software.
  *
  * The queue is intentionally large enough, at Chipyard integration, to hold a complete maximum-
  * geometry frame plus its EOF marker. That lets the capture core consume the sensor stream and
  * publish `frameDone` without waiting for software MMIO reads. `useSyncReadMem` infers block RAM
  * on the FPGA instead of a very large register array.
  */
class FrameBufferIO(p: CaptureParams, val depth: Int) extends Bundle {
  val enq        = Flipped(Decoupled(new PixelFlit(p.dataWidth)))
  val deq        = Decoupled(new PixelFlit(p.dataWidth))
  val flush      = Input(Bool())
  val count      = Output(UInt(log2Ceil(depth + 1).W))
  val full       = Output(Bool())
  val frameReady = Output(Bool())
}

class FrameBuffer(p: CaptureParams, val depth: Int, useSyncReadMem: Boolean = true) extends Module {
  require(depth >= 2, "frame-buffer depth must be at least two entries")

  val io = IO(new FrameBufferIO(p, depth))
  private val queue = Module(new Queue(
    new PixelFlit(p.dataWidth),
    entries = depth,
    useSyncReadMem = useSyncReadMem,
    hasFlush = true))

  queue.io.enq <> io.enq
  io.deq <> queue.io.deq
  queue.io.flush.get := io.flush

  io.count := queue.io.count
  io.full := !queue.io.enq.ready
  io.frameReady := queue.io.enq.fire && queue.io.enq.bits.eof
}
