package chipyard.esplink

import chisel3._
import chisel3.util._

/** Reconfigure the FPGA from another flash address (7-series IPROG, UG470
 *  "Reconfiguration and MultiBoot"). On `start` this writes, one word per
 *  cycle with CSIB low and RDWRB low:
 *
 *    FFFFFFFF dummy        AA995566 sync          20000000 NOOP
 *    30020001 write WBSTAR <addr>                 20000000 NOOP
 *    30008001 write CMD    0000000F IPROG         20000000 NOOP
 *
 *  ICAPE2 takes each byte bit-reversed relative to the bitstream (UG470
 *  "Parallel Bus Bit Order"), hence `swap`. The FPGA starts reconfiguring a
 *  few cycles after IPROG; nothing after that matters.
 */
object EspLinkIprog {
  def words(addr: UInt): Seq[UInt] = Seq(
    "hFFFFFFFF".U(32.W), "hAA995566".U(32.W), "h20000000".U(32.W),
    "h30020001".U(32.W), addr,                "h20000000".U(32.W),
    "h30008001".U(32.W), "h0000000F".U(32.W), "h20000000".U(32.W))
  def swap(w: UInt): UInt = Cat((0 until 4).reverse.map(i => Reverse(w(8 * i + 7, 8 * i))))
}

class EspLinkIcapPort extends Bundle {
  val i     = Output(UInt(32.W))
  val csib  = Output(Bool())
  val rdwrb = Output(Bool())
}

class EspLinkIprog extends Module {
  val io = IO(new Bundle {
    val start = Input(Bool())
    val addr  = Input(UInt(32.W))
    val icap  = new EspLinkIcapPort
  })
  val seq   = EspLinkIprog.words(io.addr)
  val idx   = RegInit(0.U(4.W))
  val going = RegInit(false.B)
  val word  = RegInit("hFFFFFFFF".U(32.W))
  val csib  = RegInit(true.B)
  when (io.start && !going) { going := true.B; idx := 0.U }
  when (going) {
    word := EspLinkIprog.swap(VecInit(seq)(idx))
    csib := false.B
    idx := idx + 1.U
    when (idx === (seq.length - 1).U) { going := false.B }
  } .otherwise {
    csib := true.B
  }
  io.icap.i     := word
  io.icap.csib  := csib
  io.icap.rdwrb := false.B
}

// ---- Xilinx 7-series primitives (instantiated by the harness, never simulated) ----

class STARTUPE2 extends BlackBox {
  val io = IO(new Bundle {
    val CFGCLK    = Output(Clock())
    val CFGMCLK   = Output(Clock())
    val EOS       = Output(Bool())
    val PREQ      = Output(Bool())
    val CLK       = Input(Clock())
    val GSR       = Input(Bool())
    val GTS       = Input(Bool())
    val KEYCLEARB = Input(Bool())
    val PACK      = Input(Bool())
    val USRCCLKO  = Input(Bool())
    val USRCCLKTS = Input(Bool())
    val USRDONEO  = Input(Bool())
    val USRDONETS = Input(Bool())
  })
}

class ICAPE2 extends BlackBox(Map("ICAP_WIDTH" -> chisel3.experimental.StringParam("X32"))) {
  val io = IO(new Bundle {
    val CLK   = Input(Clock())
    val CSIB  = Input(Bool())
    val RDWRB = Input(Bool())
    val I     = Input(UInt(32.W))
    val O     = Output(UInt(32.W))
  })
}

class USR_ACCESSE2 extends BlackBox {
  val io = IO(new Bundle {
    val CFGCLK    = Output(Clock())
    val DATA      = Output(UInt(32.W))
    val DATAVALID = Output(Bool())
  })
}

/** The three primitives wired for the ESP link: CCLK from the flash engine,
 *  IPROG, and the bitstream's USR_ACCESS word (the build TIMESTAMP, set in
 *  fpga-shells arty_a7_200 arty-config.xdc) as the image identity. */
class EspLinkXilinx extends Module {
  val io = IO(new Bundle {
    val sck     = Input(Bool())
    val icap    = Flipped(new EspLinkIcapPort)
    val imageId = Output(UInt(32.W))
  })
  val startup = Module(new STARTUPE2)
  startup.io.CLK       := false.B.asClock
  startup.io.GSR       := false.B
  startup.io.GTS       := false.B
  startup.io.KEYCLEARB := true.B
  startup.io.PACK      := false.B
  startup.io.USRCCLKO  := io.sck
  startup.io.USRCCLKTS := false.B
  startup.io.USRDONEO  := true.B
  startup.io.USRDONETS := true.B

  val icap = Module(new ICAPE2)
  icap.io.CLK   := clock
  icap.io.CSIB  := io.icap.csib
  icap.io.RDWRB := io.icap.rdwrb
  icap.io.I     := io.icap.i

  val ua = Module(new USR_ACCESSE2)
  // DATA is static once configured; register it into this clock domain.
  io.imageId := RegNext(RegNext(ua.io.DATA))
}
