package ospi

import chisel3._
import chisel3.util._

/** Top-level HM01B0 parallel-video capture core.
  *
  * Implicit clock/reset = the SoC/system domain. The sensor PCLK domain is driven by
  * `io.sensor.pclk`. Data flow:
  *
  *   D/FVLD/LVLD --(PCLK)--> CaptureFrontend --(PCLK)--> AsyncFifo --(sysclk)--> assembler + io.pixels
  *
  * Sensor I2C configuration is intentionally out of scope here (reuse Chipyard's TLI2C).
  */
class HM01B0Capture(p: CaptureParams = CaptureParams()) extends Module {
  val io = IO(new HM01B0CaptureIO(p))

  // ---- Reset synchronizer into the PCLK domain ----
  // Asserts asynchronously with system reset, deasserts synchronously to PCLK.
  private val pclkRst = withClockAndReset(io.sensor.pclk, reset.asAsyncReset) {
    val r0 = RegInit(true.B); val r1 = RegInit(true.B)
    r0 := false.B; r1 := r0
    r1
  }

  // 2-flop synchronizer into the system (implicit) clock domain.
  private def syncSys(sig: Bool): Bool = RegNext(RegNext(sig, false.B), false.B)
  // N-flop synchronizer of a system-domain level into the PCLK domain.
  private def syncPclk(sig: Bool): Bool =
    withClockAndReset(io.sensor.pclk, pclkRst.asAsyncReset) {
      (0 until p.syncStages).foldLeft(sig) { (prev, _) => RegNext(prev, false.B) }
    }

  // ---- Bring-up diagnostic counters (PCLK domain -> system domain) ----
  // Gray -> binary. Only the destination domain calls this, on an already-synchronized value.
  private def gray2bin(g: UInt, width: Int): UInt = {
    val b = Wire(Vec(width, Bool()))
    b(width - 1) := g(width - 1)
    for (i <- (0 until width - 1).reverse) { b(i) := b(i + 1) ^ g(i) }
    b.asUInt
  }

  /** A counter incremented in the PCLK domain, readable in the system domain.
    *
    * The counter is binary in its own domain and is launched as a *registered* Gray code, so at
    * most one bit changes per increment and the synchronizer chain can only ever resolve to the
    * value before or after an increment -- never a mixture. That is what makes a multibit count
    * safe to cross; independent per-bit synchronizers on a binary counter would not be.
    *
    * If the sensor never drives PCLK the counter never advances and reads back zero, which is
    * exactly the "no camera clock" signature bring-up needs to distinguish.
    */
  private def crossCount(inc: Bool): UInt = {
    val w = p.diagCountWidth
    val grayLaunch = withClockAndReset(io.sensor.pclk, pclkRst.asAsyncReset) {
      val bin = RegInit(0.U(w.W))
      when(inc) { bin := bin + 1.U }
      RegNext(bin ^ (bin >> 1), 0.U(w.W))
    }
    val synced = (0 until p.syncStages).foldLeft(grayLaunch) { (prev, _) =>
      RegNext(prev, 0.U(w.W))
    }
    gray2bin(synced, w)
  }

  // Rising-edge detectors live in the PCLK domain, where FVLD/LVLD are already synchronous.
  private val (fvldRise, lvldRise) =
    withClockAndReset(io.sensor.pclk, pclkRst.asAsyncReset) {
      val fvPrev = RegNext(io.sensor.fvld, false.B)
      val lvPrev = RegNext(io.sensor.lvld, false.B)
      (io.sensor.fvld && !fvPrev, io.sensor.lvld && !lvPrev)
    }

  io.status.pclkCount := crossCount(true.B)
  io.status.fvldRises := crossCount(fvldRise)
  io.status.lvldRises := crossCount(lvldRise)

  // ---- PCLK-domain capture front-end ----
  private val frontend = withClockAndReset(io.sensor.pclk, pclkRst.asAsyncReset) {
    Module(new CaptureFrontend(p))
  }
  frontend.io.fvld   := io.sensor.fvld
  frontend.io.lvld   := io.sensor.lvld
  frontend.io.d      := io.sensor.d
  frontend.io.enable := syncPclk(io.ctrl.enable)

  // ---- Clock-domain-crossing FIFO ----
  private val fifo = Module(new AsyncFifo(new PixelFlit(p.dataWidth), p.fifoDepth, p.syncStages))
  fifo.enqClock      := io.sensor.pclk
  fifo.enqReset      := pclkRst.asAsyncReset
  fifo.deqClock      := clock
  fifo.deqReset      := reset.asAsyncReset
  fifo.clearOverflow := io.ctrl.clear
  // Front-end pushes unconditionally (sensor can't stall); FIFO flags overflow on a dropped beat.
  fifo.enq.valid := frontend.io.out.valid
  fifo.enq.bits  := frontend.io.out.bits

  // ---- Bounded capture gate ----
  // With a non-zero target the core presents beats only while a capture is armed and incomplete.
  // When it is not storing it still asserts ready, so the CDC FIFO keeps draining and a completed
  // capture cannot back-pressure the front-end into a spurious overflow.
  private val bounded   = io.ctrl.pixelTarget =/= 0.U
  private val armedReg  = RegInit(false.B)
  private val doneReg   = RegInit(false.B)
  private val capCount  = RegInit(0.U(32.W))
  private val lastPixR  = RegInit(0.U(p.dataWidth.W))
  private val storing   = !bounded || (armedReg && !doneReg)

  // ---- System-domain consumer: forward stream to SoC and measure geometry ----
  io.pixels.valid  := fifo.deq.valid && storing
  io.pixels.bits   := fifo.deq.bits
  fifo.deq.ready   := Mux(storing, io.pixels.ready, true.B)

  when(io.ctrl.arm) {
    armedReg := true.B
    doneReg  := false.B
    capCount := 0.U
  }
  when(io.pixels.fire && !io.pixels.bits.eof) {
    lastPixR := io.pixels.bits.data
    val next = capCount + 1.U
    capCount := next
    when(bounded && next >= io.ctrl.pixelTarget) {
      doneReg  := true.B
      armedReg := false.B
    }
  }

  io.status.armed          := armedReg
  io.status.captureDone    := doneReg
  io.status.capturedPixels := capCount
  io.status.lastPixel      := lastPixR

  private val pixCol    = RegInit(0.U(p.pixCountWidth.W))
  private val rowCnt    = RegInit(0.U(p.lineCountWidth.W))
  private val widthReg  = RegInit(0.U(p.pixCountWidth.W))
  private val heightReg = RegInit(0.U(p.lineCountWidth.W))
  private val frameCnt  = RegInit(0.U(32.W))
  private val geomErrR  = RegInit(false.B)
  private val frameDone = WireDefault(false.B)

  when(io.pixels.fire) {
    val b = io.pixels.bits
    when(b.eof) {
      // End-of-frame marker: finalize measurement for this frame.
      heightReg := rowCnt
      frameCnt  := frameCnt + 1.U
      frameDone := true.B
      when((widthReg =/= io.ctrl.expWidth) || (rowCnt =/= io.ctrl.expHeight)) {
        geomErrR := true.B
      }
      rowCnt := 0.U
      pixCol := 0.U
    }.otherwise {
      // Pixel beat. `sof` resets the in-frame counters before counting this pixel.
      val colBase = Mux(b.sof, 0.U, pixCol)
      val rowBase = Mux(b.sof, 0.U, rowCnt)
      when(b.eol) {
        widthReg := colBase + 1.U
        rowCnt   := rowBase + 1.U
        pixCol   := 0.U
      }.otherwise {
        pixCol := colBase + 1.U
        rowCnt := rowBase
      }
    }
  }

  when(io.ctrl.clear) { geomErrR := false.B }

  // ---- MCLK generation: divide the system clock. MCLK = sysclk / (2*(mclkDiv+1)). ----
  private val mclkReg = RegInit(false.B)
  private val divCnt  = RegInit(0.U(p.mclkDivWidth.W))
  when(divCnt === io.ctrl.mclkDiv) {
    divCnt  := 0.U
    mclkReg := !mclkReg
  }.otherwise {
    divCnt := divCnt + 1.U
  }
  io.sensor.mclk := mclkReg.asClock

  // ---- TRIG: software-driven pulse (snapshot); held low in free-run continuous mode. ----
  io.sensor.trig := RegNext(io.ctrl.trigPulse && !io.ctrl.continuous, false.B)

  // ---- Status ----
  io.status.frameCount := frameCnt
  io.status.lastWidth  := widthReg
  io.status.lastHeight := heightReg
  io.status.overflow   := syncSys(fifo.overflow)
  io.status.geomErr    := geomErrR
  io.status.frameDone  := frameDone
  io.status.sensorInt  := syncSys(io.sensor.intr)
  io.status.busy       := syncSys(io.sensor.fvld)
  io.status.irq        := frameDone && io.ctrl.irqEnable
}
