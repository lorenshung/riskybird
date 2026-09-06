package ospi

import chisel3._
import chisel3.util._

/** Dual-clock (asynchronous) FIFO using Gray-coded pointers and multi-flop synchronizers
  * (textbook Cummings style). Crosses captured pixel beats from the sensor PCLK domain into
  * the SoC/system clock domain.
  *
  * At Chipyard integration this is a drop-in for `freechips.rocketchip.util.AsyncQueue`; it is
  * hand-rolled here to keep the standalone project dependency-light (Chisel only).
  *
  * Notes:
  *  - `enq` is in the enqueue clock domain, `deq` in the dequeue clock domain.
  *  - Memory is asynchronous-read so the read port lives in the deq domain while writes are
  *    clocked by the enq domain; data is committed before its pointer crosses, so reads are safe.
  *  - `overflow` is a sticky level latched in the enq domain when a beat is offered while full
  *    (a dropped beat). `clearOverflow` (synchronized into the enq domain) clears it.
  */
class AsyncFifo[T <: Data](gen: T, val depth: Int, val syncStages: Int = 3) extends RawModule {
  require(isPow2(depth) && depth >= 4, "depth must be a power of two >= 4")
  private val addrW = log2Ceil(depth)
  private val ptrW  = addrW + 1 // extra MSB to distinguish full from empty

  val enqClock      = IO(Input(Clock()))
  val enqReset      = IO(Input(AsyncReset()))
  val enq           = IO(Flipped(Decoupled(gen)))
  val deqClock      = IO(Input(Clock()))
  val deqReset      = IO(Input(AsyncReset()))
  val deq           = IO(Decoupled(gen))
  val overflow      = IO(Output(Bool()))      // enq-domain sticky
  val clearOverflow = IO(Input(Bool()))       // enq-domain level (already synchronized by parent)

  private def bin2gray(b: UInt): UInt = b ^ (b >> 1)

  /** Chain `n` flops in (clk,rst) to synchronize a value crossing into that domain. */
  private def syncInto[U <: Data](sig: U, clk: Clock, rst: AsyncReset, n: Int): U =
    withClockAndReset(clk, rst) {
      (0 until n).foldLeft(sig) { (prev, _) => RegNext(prev, 0.U.asTypeOf(chiselTypeOf(sig))) }
    }

  private val mem = Mem(depth, gen)

  // Pointer + flag registers, each created (and thus clocked) in its own domain.
  // `full`/`empty` are REGISTERED (computed from the *next* pointer) to break the
  // ready<->fire<->pointer<->flag combinational loop (textbook Cummings async FIFO).
  private val wbin     = withClockAndReset(enqClock, enqReset) { RegInit(0.U(ptrW.W)) }
  private val wgray    = withClockAndReset(enqClock, enqReset) { RegInit(0.U(ptrW.W)) }
  private val wfullReg = withClockAndReset(enqClock, enqReset) { RegInit(false.B) }
  private val ovfl     = withClockAndReset(enqClock, enqReset) { RegInit(false.B) }
  private val rbin      = withClockAndReset(deqClock, deqReset) { RegInit(0.U(ptrW.W)) }
  private val rgray     = withClockAndReset(deqClock, deqReset) { RegInit(0.U(ptrW.W)) }
  private val remptyReg = withClockAndReset(deqClock, deqReset) { RegInit(true.B) }

  // Cross pointers into the opposite domain (Gray code => safe bit-by-bit synchronization).
  private val rgraySyncW = syncInto(rgray, enqClock, enqReset, syncStages)
  private val wgraySyncR = syncInto(wgray, deqClock, deqReset, syncStages)

  // ---- Enqueue (write) domain ----
  private val winc      = enq.valid && !wfullReg       // accepted write this cycle
  private val wbinNext  = wbin + winc.asUInt
  private val wgrayNext  = bin2gray(wbinNext)
  // Full when the next write Gray pointer would equal the read pointer with the top two bits inverted.
  private val wfullNext = wgrayNext === Cat(~rgraySyncW(ptrW - 1, ptrW - 2), rgraySyncW(ptrW - 3, 0))
  enq.ready := !wfullReg
  wbin     := wbinNext
  wgray    := wgrayNext
  wfullReg := wfullNext
  withClock(enqClock) {
    when(winc) { mem.write(wbin(addrW - 1, 0), enq.bits) }
  }
  // Sticky overflow: a beat offered while full is dropped.
  private val clearW = syncInto(clearOverflow, enqClock, enqReset, syncStages)
  when(enq.valid && wfullReg) { ovfl := true.B }
  when(clearW) { ovfl := false.B }
  overflow := ovfl

  // ---- Dequeue (read) domain ----
  private val rinc       = deq.ready && !remptyReg     // accepted read this cycle
  private val rbinNext   = rbin + rinc.asUInt
  private val rgrayNext   = bin2gray(rbinNext)
  private val remptyNext = rgrayNext === wgraySyncR    // empty when next read Gray catches up to write
  deq.valid := !remptyReg
  withClock(deqClock) {
    deq.bits := mem.read(rbin(addrW - 1, 0))
  }
  rbin      := rbinNext
  rgray     := rgrayNext
  remptyReg := remptyNext
}
