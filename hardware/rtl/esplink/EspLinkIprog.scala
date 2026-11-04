package chipyard.esplink

import chisel3._
import chisel3.util._

/** Reconfigure the FPGA from another flash address (7-series IPROG, UG470
 *  "Reconfiguration and MultiBoot"). On `start` this writes, one word per
 *  cycle with CSIB low and RDWRB low, one of four sequences (`mode`):
 *
 *    0 IPROG   FFFFFFFF dummy, AA995566 sync, 20000000 NOOP,
 *              30020001 write WBSTAR, <addr>, 20000000 NOOP,
 *              30008001 write CMD, 0000000F IPROG, 20000000 NOOP
 *    1 bare    as 0 with the WBSTAR write replaced by NOOPs: IPROG to
 *              whatever WBSTAR already holds (JTAG can set it)
 *    2 WBSTAR  as 0 with IPROG replaced by DESYNC (0000000D): sets WBSTAR and
 *              leaves the FPGA running, so JTAG can read back what landed
 *    3 packed  as 0 without the NOOP between <addr> and the CMD write, the
 *              layout Vivado itself emits for NEXT_CONFIG_REBOOT
 *
 *  `addr` is the raw WBSTAR value. With BITSTREAM.CONFIG.SPI_32BIT_ADDR (set
 *  for every RiskyBird build) WBSTAR[23:0] is flash address bits [31:8], so
 *  the update slot at 0x800000 is WBSTAR 0x8000; the ESP does the shift.
 *  Sending the byte address made every IPROG wrap to 0 (golden) on the bench
 *  until 2026-11-02, which is what modes 1-3 were added to chase.
 *
 *  ICAPE2 takes each byte bit-reversed relative to the bitstream (UG470
 *  "Parallel Bus Bit Order"), hence `swap`. The FPGA starts reconfiguring a
 *  few cycles after IPROG; nothing after that matters.
 */
object EspLinkIprog {
  val NOOP = "h20000000"
  def words(addr: UInt): Seq[Seq[UInt]] = {
    def w(h: String) = h.U(32.W)
    val head = Seq(w("hFFFFFFFF"), w("hAA995566"), w(NOOP))
    Seq(
      head ++ Seq(w("h30020001"), addr, w(NOOP), w("h30008001"), w("h0000000F"), w(NOOP)),
      head ++ Seq(w(NOOP), w(NOOP), w(NOOP), w("h30008001"), w("h0000000F"), w(NOOP)),
      head ++ Seq(w("h30020001"), addr, w(NOOP), w("h30008001"), w("h0000000D"), w(NOOP)),
      head ++ Seq(w("h30020001"), addr, w("h30008001"), w("h0000000F"), w(NOOP), w(NOOP)))
  }
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
    val mode  = Input(UInt(2.W))
    val busy  = Output(Bool())
    val icap  = new EspLinkIcapPort
  })
  val seqs  = EspLinkIprog.words(io.addr)
  val len   = seqs.head.length
  require(seqs.forall(_.length == len))
  val table = VecInit(seqs.map(VecInit(_)))
  val mode  = RegInit(0.U(2.W))
  val idx   = RegInit(0.U(4.W))
  val going = RegInit(false.B)
  val word  = RegInit("hFFFFFFFF".U(32.W))
  val csib  = RegInit(true.B)
  when (io.start && !going) { going := true.B; idx := 0.U; mode := io.mode }
  when (going) {
    word := EspLinkIprog.swap(table(mode)(idx))
    csib := false.B
    idx := idx + 1.U
    when (idx === (len - 1).U) { going := false.B }
  } .otherwise {
    csib := true.B
  }
  io.busy       := going || !csib
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
