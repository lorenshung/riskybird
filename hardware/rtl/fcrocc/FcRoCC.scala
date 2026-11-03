// ===========================================================================
// FcRoCC.scala -- Hardened Flight-Controller RoCC (validated-core integration).
//
// Wraps the VALIDATED datapath core fc_rocc_core.sv (a BlackBox) in a LazyRoCC,
// translating the RoCC command interface to the core's write/run/read ports.
// The core executes the assembled microcode ROM (fc_rocc_rom.hex) that
// reproduces the fp32 golden (open 6.79um / closed 51um; xsim-confirmed).
//
// custom0 ISA (funct = inst.funct):
//   PUSH(0): rf[rs1]=rs2        RUN(1): run prog@rs1 to HALT, resp=done
//   POP (2): resp.data=rf[rs1]  CFG(3): rf[rs1]=rs2  (boot const/state load)
//
// Build note: fc_rocc_core.sv + fc_rocc_rom.hex + fc_rocc_init.hex are added as
// BlackBox resources (copied into gen-collateral); the core's $readmemh("...hex")
// resolves against the Vivado synth search path. (Alternatively drop the SV
// BlackBox and elaborate the ROM as a Chisel VecInit from the hex.)
// ===========================================================================
package chipyard.fc

import chisel3._
import chisel3.util._
import chisel3.experimental.{IntParam, StringParam}
import org.chipsalliance.cde.config._
import freechips.rocketchip.tile._
import freechips.rocketchip.rocket._
import freechips.rocketchip.diplomacy._

class fc_rocc_core extends BlackBox with HasBlackBoxResource {
  val io = IO(new Bundle {
    val clk    = Input(Clock())
    val rst    = Input(Bool())
    val we     = Input(Bool())
    val waddr  = Input(UInt(8.W))
    val wdata  = Input(SInt(48.W))
    val run    = Input(Bool())
    val rstart = Input(UInt(16.W))
    val raddr  = Input(UInt(8.W))
    val rdata  = Output(SInt(48.W))
    val busy   = Output(Bool())
  })
  addResource("/vsrc/fc_rocc_core.sv")
  addResource("/vsrc/fc_rocc_rom.hex")
  addResource("/vsrc/fc_rocc_init.hex")
}

class FcRoCC(opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  override lazy val module = new FcRoCCModuleImp(this)
}

class FcRoCCModuleImp(outer: FcRoCC)(implicit p: Parameters)
    extends LazyRoCCModuleImp(outer) {
  val FUNCT_PUSH = 0.U; val FUNCT_RUN = 1.U; val FUNCT_POP = 2.U; val FUNCT_CFG = 3.U

  val core = Module(new fc_rocc_core)
  core.io.clk := clock
  core.io.rst := reset.asBool

  val cmd = Queue(io.cmd)
  val funct = cmd.bits.inst.funct
  val rs1   = cmd.bits.rs1
  val rs2   = cmd.bits.rs2

  val sIdle :: sRun :: sResp :: Nil = Enum(3)
  val state = RegInit(sIdle)
  val rdSave = Reg(UInt(64.W))
  val respRd = Reg(UInt(5.W))

  // defaults
  core.io.we := false.B; core.io.waddr := 0.U; core.io.wdata := 0.S
  core.io.run := false.B; core.io.rstart := 0.U; core.io.raddr := 0.U

  cmd.ready := (state === sIdle)
  io.busy   := (state =/= sIdle) || core.io.busy
  io.interrupt := false.B
  io.resp.valid := (state === sResp)
  io.resp.bits.rd := respRd
  io.resp.bits.data := rdSave

  switch (state) {
    is (sIdle) {
      when (cmd.fire) {
        respRd := cmd.bits.inst.rd
        switch (funct) {
          is (FUNCT_PUSH) { core.io.we := true.B; core.io.waddr := rs1(7,0); core.io.wdata := rs2(47,0).asSInt }
          is (FUNCT_CFG)  { core.io.we := true.B; core.io.waddr := rs1(7,0); core.io.wdata := rs2(47,0).asSInt }
          is (FUNCT_POP)  { core.io.raddr := rs1(7,0)
                            rdSave := Cat(Fill(16, core.io.rdata(47)), core.io.rdata.asUInt) // sign-extend 48->64
                            state := sResp }
          is (FUNCT_RUN)  { core.io.run := true.B; core.io.rstart := rs1(15,0); state := sRun }
        }
      }
    }
    is (sRun) { when (!core.io.busy) { rdSave := 1.U; state := sResp } }   // done token
    is (sResp) { when (io.resp.fire) { state := sIdle } }
  }
}

// add the FC RoCC on custom0
class WithFcRoCC extends Config((site, here, up) => {
  case BuildRoCC => up(BuildRoCC) ++ Seq(
    (p: Parameters) => LazyModule(new FcRoCC(OpcodeSet.custom0)(p)))
})
