package ospi

import circt.stage.ChiselStage

/** Emit SystemVerilog for the capture core.
  *
  * Requires `firtool` on PATH (available in the sibling Chipyard conda environment).
  *
  * Run: ./mill -i ospi.runMain ospi.Elaborate
  */
object Elaborate extends App {
  val p = CaptureParams()
  ChiselStage.emitSystemVerilogFile(
    new HM01B0Capture(p),
    args        = Array("--target-dir", "generated"),
    firtoolOpts = Array("-disable-all-randomization", "-strip-debug-info")
  )
  println("Wrote generated/HM01B0Capture.sv")
}
