package ospi

import chisel3._
import chisel3.util._

/** One captured beat that travels in-order through the CDC FIFO to the SoC.
  *
  * Two kinds of beats are produced:
  *  - a pixel beat:  `eof = false`, `data` valid; `sof` marks the first pixel of a frame,
  *    `eol` marks the last pixel of a line.
  *  - an end-of-frame marker: `eof = true`, `data`/`sof`/`eol` are don't-care. It is pushed
  *    after the last pixel of the frame so the consumer can finalize the image with the same
  *    ordering guarantees as the pixel data (single FIFO, no separate event-CDC race).
  */
class PixelFlit(val dataWidth: Int) extends Bundle {
  val data = UInt(dataWidth.W)
  val sof  = Bool()
  val eol  = Bool()
  val eof  = Bool()
}

/** Sensor-facing pins, from OUR (capture) module's perspective.
  *
  * The original HM01B0 pin directions are the *sensor's*; we see the mirror:
  *   video bus (PCLK/FVLD/LVLD/D/INT) are INPUTS we sample; MCLK/TRIG are OUTPUTS we drive.
  * SCL/SDA (I2C) are intentionally NOT here: sensor configuration is done by Chipyard's
  * existing `sifive.blocks.devices.i2c.TLI2C` at integration, not by this capture core.
  */
class SensorIO(val dataWidth: Int) extends Bundle {
  // Inputs from the sensor
  val pclk = Input(Clock())          // pixel clock (our capture clock, source-synchronous)
  val fvld = Input(Bool())           // FLVD: frame valid
  val lvld = Input(Bool())           // LVLD: line valid
  val d    = Input(UInt(dataWidth.W))// D0-7
  val intr = Input(Bool())           // INT: motion interrupt, active high
  // Outputs to the sensor
  val mclk = Output(Clock())         // master clock to the sensor (divided)
  val trig = Output(Bool())          // TRIG: frame trigger, active high
}

/** SoC-facing control. Plain register fields now; mapped to a TileLink `regmap` at integration. */
class ControlIO(p: CaptureParams) extends Bundle {
  val enable     = Input(Bool())                      // arm the capture datapath
  val continuous = Input(Bool())                      // free-run vs snapshot (drives TRIG policy)
  val trigPulse  = Input(Bool())                      // software-driven TRIG pulse (snapshot)
  val clear      = Input(Bool())                      // clear sticky error flags (overflow/geomErr)
  val irqEnable  = Input(Bool())                      // gate the frame-done interrupt
  val mclkDiv    = Input(UInt(p.mclkDivWidth.W))      // MCLK = sysclk / (2*(mclkDiv+1))
  val expWidth   = Input(UInt(p.pixCountWidth.W))     // expected frame width  (for geomErr)
  val expHeight  = Input(UInt(p.lineCountWidth.W))    // expected frame height (for geomErr)

  // ---- Bounded ("armed") capture, for bring-up ----
  // pixelTarget == 0 keeps the historical free-running behaviour: every beat is presented.
  // A non-zero target makes the core present exactly that many pixel beats after `arm`, then stop
  // cleanly -- while still draining the CDC FIFO, so stopping never manufactures a false overflow.
  val arm         = Input(Bool())                     // pulse: start a bounded capture
  val pixelTarget = Input(UInt(32.W))                 // pixels to capture; 0 = unbounded
}

/** SoC-facing status / events. */
class StatusIO(p: CaptureParams) extends Bundle {
  val frameCount = Output(UInt(32.W))                 // frames completed since reset
  val lastWidth  = Output(UInt(p.pixCountWidth.W))    // measured width of last completed frame
  val lastHeight = Output(UInt(p.lineCountWidth.W))   // measured height of last completed frame
  val overflow   = Output(Bool())                     // sticky: CDC FIFO overflowed (pixels dropped)
  val geomErr    = Output(Bool())                     // sticky: measured geometry != expected
  val frameDone  = Output(Bool())                     // 1-cycle pulse when a frame completes
  val sensorInt  = Output(Bool())                     // synchronized sensor INT level
  val busy       = Output(Bool())                     // synchronized FVLD level (inside a frame)
  val irq        = Output(Bool())                     // frameDone & irqEnable

  // ---- Bring-up diagnostics ----
  // These exist because hardware bring-up has no ILA: they are the only way to tell "the sensor
  // is not clocking" apart from "the sensor clocks but never asserts FVLD" apart from "sync is
  // fine but no pixel data arrives". Each is counted in the PCLK domain and crossed into the
  // system domain as a Gray code, so a read is always a coherent (possibly stale) sample.
  val pclkCount  = Output(UInt(p.diagCountWidth.W))   // PCLK rising edges observed
  val fvldRises  = Output(UInt(p.diagCountWidth.W))   // FVLD rising edges (frame starts) observed
  val lvldRises  = Output(UInt(p.diagCountWidth.W))   // LVLD rising edges (line starts) observed

  val armed          = Output(Bool())                 // a bounded capture is in progress
  val captureDone    = Output(Bool())                 // bounded capture reached its target
  val capturedPixels = Output(UInt(32.W))             // pixel beats presented in this capture
  val lastPixel      = Output(UInt(p.dataWidth.W))    // most recent pixel value presented
}

/** IO for [[CaptureFrontend]]. Named so member access doesn't require structural-type reflection. */
class CaptureFrontendIO(p: CaptureParams) extends Bundle {
  val fvld   = Input(Bool())
  val lvld   = Input(Bool())
  val d      = Input(UInt(p.dataWidth.W))
  val enable = Input(Bool())
  val out    = Valid(new PixelFlit(p.dataWidth))
}

/** Complete top-level IO for [[HM01B0Capture]]. Named (not an anonymous bundle) so member
  * access doesn't require Scala structural-type reflection. */
class HM01B0CaptureIO(p: CaptureParams) extends Bundle {
  val sensor = new SensorIO(p.dataWidth)
  val ctrl   = new ControlIO(p)
  val status = new StatusIO(p)
  val pixels = Decoupled(new PixelFlit(p.dataWidth)) // captured stream to the SoC sink
}
