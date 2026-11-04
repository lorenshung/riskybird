package ospi

import chisel3.util.{isPow2, log2Ceil}

/** Configuration for the HM01B0 capture core.
  *
  *  - The HM01B0 outputs an 8-bit parallel RAW stream (D0-7) gated by FVLD (frame valid) and
  *    LVLD (line valid), clocked by PCLK. v1 supports 8-bit mode only.
  *  - Geometry is not hard-coded into the datapath: the core captures whatever FVLD/LVLD gate
  *    and *measures* the frame. `maxWidth`/`maxHeight` only size the counters; `defaultWidth`/
  *    `defaultHeight` document the expected geometry (datasheet: full 324x324, QVGA window 324x244).
  */
case class CaptureParams(
  dataWidth:    Int = 8,
  defaultWidth: Int = 324,  // QVGA window width  (datasheet 1.4)
  defaultHeight:Int = 244,  // QVGA window height (datasheet 1.4); full frame is 324x324
  maxWidth:     Int = 324,
  maxHeight:    Int = 324,
  fifoDepth:    Int = 1024, // CDC FIFO depth (power of two)
  syncStages:   Int = 3,    // metastability synchronizer flop stages
  mclkDivWidth: Int = 8,    // width of the MCLK divider compare value
  diagCountWidth: Int = 32  // width of the PCLK/FVLD/LVLD bring-up diagnostic counters
) {
  require(dataWidth == 8, "v1 supports 8-bit parallel mode only")
  require(defaultWidth > 0 && defaultWidth <= maxWidth, "defaultWidth must fit maxWidth")
  require(defaultHeight > 0 && defaultHeight <= maxHeight, "defaultHeight must fit maxHeight")
  require(isPow2(fifoDepth) && fifoDepth >= 4, "fifoDepth must be a power of two >= 4")
  require(syncStages >= 2, "need >= 2 synchronizer stages for CDC")
  require(diagCountWidth >= 2 && diagCountWidth <= 32,
    "diagnostic counters are read through 32-bit MMIO registers")

  /** Width to count pixels within a line: 0 .. maxWidth. */
  val pixCountWidth: Int = log2Ceil(maxWidth + 1)
  /** Width to count lines within a frame: 0 .. maxHeight. */
  val lineCountWidth: Int = log2Ceil(maxHeight + 1)
}
