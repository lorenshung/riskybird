package ospi

import chisel3._
import chisel3.util._

/** Parallel-video receiver, entirely in the sensor PCLK domain.
  *
  * FVLD/LVLD/D are driven by the sensor synchronously to PCLK, so within this domain they are
  * already synchronous (no metastability) and only need single registering for edge look-ahead.
  *
  * Timing model: register the inputs once (`*R`). The *raw* inputs (`io.fvld`/`io.lvld`) observed
  * in the same cycle equal what `*R` will hold next cycle, giving a one-cycle look-ahead used to
  * detect end-of-line and the falling edge of FVLD.
  *
  * Output is a `Valid` (not `Decoupled`): the sensor cannot be back-pressured, so beats are pushed
  * unconditionally; if the downstream FIFO is full the beat is dropped there and flagged as overflow.
  */
class CaptureFrontend(p: CaptureParams) extends Module {
  val io = IO(new CaptureFrontendIO(p))

  private val fvR = RegNext(io.fvld, false.B)
  private val lvR = RegNext(io.lvld, false.B)
  private val dR  = RegNext(io.d, 0.U(p.dataWidth.W))
  private val seen = RegInit(false.B) // have we emitted a pixel in the current frame yet?

  private val en         = io.enable
  private val pixelValid = fvR && lvR && en
  private val lastOfLine = pixelValid && !io.lvld    // next cycle LVLD low => last pixel of line
  private val frameEnd   = fvR && !io.fvld && en     // FVLD falling => end of frame

  when(!en || frameEnd) {
    seen := false.B
  }.elsewhen(pixelValid) {
    seen := true.B
  }

  private val sof = pixelValid && !seen

  io.out.valid     := pixelValid || frameEnd
  io.out.bits.data := dR
  io.out.bits.sof  := sof
  io.out.bits.eol  := lastOfLine
  io.out.bits.eof  := frameEnd
}
