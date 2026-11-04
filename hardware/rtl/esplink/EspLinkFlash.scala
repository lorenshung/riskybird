package chipyard.esplink

import chisel3._
import chisel3.util._

/** Configuration-flash bridge behind the ESP link, for remote bitstream updates.
 *
 *  The TE0712's S25FL256S is the FPGA's own configuration flash. After
 *  configuration its CS# (T19) and D0..D3 (P22/R22/P21/R21) are ordinary user
 *  pins, and its clock is CCLK, which a design can only drive through
 *  STARTUPE2.USRCCLKO. This engine speaks single-bit SPI on those pins at
 *  clock/(4*sckHalf) and takes byte-framed commands from the ESP link:
 *
 *    0x01 XFER  wlen:u16le rlen:u16le wdata[wlen]
 *               CS low, shift out wdata, shift in rlen bytes, CS high.
 *               Reply: rdata[rlen] + 0xA5. Only opcodes in `xferOps` (reads,
 *               plus the volatile BRWR and RESET) touch the flash; anything
 *               else is not sent, and the reply is rlen zero bytes + 0xE2. In
 *               particular WREN is refused here, so XFER can never program,
 *               erase, or write a non-volatile register.
 *    0x02 PROG  addr:u32le data[256]
 *               WREN, 4PP (0x12) one 256-byte page, poll SR1 until WIP clears.
 *               Reply: SR1 (bit 6 P_ERR, bit 5 E_ERR), or 0xE1 when addr is not
 *               page-aligned or is below `guardBase` (the data is still
 *               consumed), or 0xE3 if the flash stayed busy past the timeout.
 *    0x03 ERASE addr:u32le
 *               WREN, 4SE (0xDC) one 64 KiB sector, poll. Reply as PROG; 0xE1
 *               unless addr is 64 KiB-aligned and >= guardBase.
 *    0x04 CRC   addr:u32le len:u32le
 *               4READ (0x13) len bytes from addr and reply their CRC-32 (IEEE
 *               802.3, as zlib.crc32) as u32le + 0xA5, so the host can check an
 *               image without reading it back over the 115200 link. len must
 *               be below 64 MiB (0xE1 otherwise). At sckHalf 2 and 50 MHz
 *               that is 12.5 Mbit/s, ~0.7 s per MiB.
 *    other      Reply 0xEE. The host has lost framing.
 *
 *  `guardBase` is the point of doing this in hardware: everything below it is
 *  the golden image, the one the FPGA boots after a power cut and the one that
 *  carries the recovery path, so no software bug anywhere on the ESP or the
 *  host can program or erase it. Only JTAG can.
 *
 *  The host keeps at most `fifoDepth` bytes in flight (it counts reply bytes),
 *  so the input FIFO never overflows while the engine waits on the flash. If
 *  it does anyway, every later reply is 0xEF until the session ends.
 */
object EspLinkFlash {
  val opXfer  = 0x01
  val opProg  = 0x02
  val opErase = 0x03
  val opCrc   = 0x04
  val ok        = 0xA5
  val errGuard  = 0xE1
  val errOp     = 0xE2
  val errBusy   = 0xE3
  val errUnknown = 0xEE
  val errOverflow = 0xEF
  // RDID, RDSR1, RDSR2, RDCR, 4READ, READ, CLSR, WRDI, BRRD, RDID-90, RES/AB,
  // plus two that change no stored data: BRWR (0x17, the VOLATILE bank
  // address register, whose EXTADD bit puts 3-byte commands into 4-byte mode)
  // and software RESET (0xF0, back to power-on state). Before an IPROG the ESP
  // uses them to make sure the FPGA's 3-byte configuration reads will work
  // even after a tool such as openFPGALoader left the part in 4-byte mode.
  val xferOps = Seq(0x9F, 0x05, 0x07, 0x35, 0x13, 0x03, 0x30, 0x04, 0x16, 0x90, 0xAB, 0x17, 0xF0)

  /** One byte of reflected CRC-32 (poly 0xEDB88320), no pre/post inversion. */
  def crcByte(c: UInt, b: UInt): UInt = {
    var x = c ^ b.pad(32)
    for (_ <- 0 until 8) {
      x = Mux(x(0), (x >> 1) ^ "hEDB88320".U(32.W), (x >> 1).pad(32))
    }
    x
  }
}

class EspLinkSpiPins extends Bundle {
  val sck  = Output(Bool())   // to STARTUPE2.USRCCLKO
  val csn  = Output(Bool())
  val mosi = Output(Bool())   // D0
  val miso = Input(Bool())    // D1
}

class EspLinkFlash(clockHz: Double, guardBase: BigInt, sckHalf: Int = 2,
                   busyTimeoutMs: Int = 5000) extends Module {
  import EspLinkFlash._
  require(sckHalf >= 2)
  require(guardBase % 65536 == 0)
  val io = IO(new Bundle {
    val en   = Input(Bool())                    // low: abort, CS high
    val in   = Flipped(Decoupled(UInt(8.W)))
    val out  = Decoupled(UInt(8.W))
    val spi  = new EspLinkSpiPins
    val busy = Output(Bool())                   // mid-command (for the idle timer)
    val overflow = Input(Bool())                // sticky, from the FIFO side
  })

  // ---- SPI byte shifter, mode 0, MSB first --------------------------------
  val shBusy = RegInit(false.B)
  val shTx   = Reg(UInt(8.W))
  val shRx   = Reg(UInt(8.W))
  val shBit  = Reg(UInt(3.W))
  val shCnt  = Reg(UInt(log2Ceil(sckHalf + 1).W))
  val shHigh = RegInit(false.B)
  val shDone = WireDefault(false.B)
  val misoQ  = RegNext(io.spi.miso)
  val sck    = RegInit(false.B)
  val csn    = RegInit(true.B)
  def shStart(b: UInt): Unit = {
    shBusy := true.B; shTx := b; shBit := 7.U; shCnt := (sckHalf - 1).U; shHigh := false.B
  }
  when (shBusy) {
    when (shCnt =/= 0.U) {
      shCnt := shCnt - 1.U
    } .otherwise {
      shCnt := (sckHalf - 1).U
      when (!shHigh) {
        sck := true.B; shHigh := true.B             // rising edge: flash samples MOSI
      } .otherwise {
        // End of the high phase: misoQ is the pin one cycle ago, well after
        // the flash's falling-edge output became valid.
        shRx := Cat(shRx(6, 0), misoQ)
        sck := false.B; shHigh := false.B
        shTx := shTx << 1
        when (shBit === 0.U) { shBusy := false.B; shDone := true.B }
        shBit := shBit - 1.U
      }
    }
  }
  val rxByte = Cat(shRx(6, 0), misoQ)   // valid in the cycle shDone fires

  io.spi.sck  := sck
  io.spi.csn  := csn
  io.spi.mosi := shTx(7)

  // ---- reply queue ---------------------------------------------------------
  val outQ = Module(new Queue(UInt(8.W), 16))
  io.out <> outQ.io.deq
  outQ.io.enq.valid := false.B
  outQ.io.enq.bits  := 0.U
  def emit(b: UInt): Bool = {
    outQ.io.enq.valid := true.B
    outQ.io.enq.bits  := Mux(io.overflow, errOverflow.U, b)
    outQ.io.enq.ready
  }

  // ---- command sequencer ---------------------------------------------------
  // Two patterns: one 23-element :: pattern does not compile (scalac).
  val allStates = Enum(23)
  val (sWarm :: sIdle :: sHdr :: sDispatch :: sXOp :: sXW :: sXR :: sXEnd ::
      sDrain :: sWren :: sWrenEnd :: sCmd :: sAddr :: sData :: sCmdEnd ::
      sPoll :: sPollRd :: sPollEnd :: sReply :: sGap :: Nil) = allStates.take(20)
  val (sCrcCmd :: sCrcRd :: sCrcOut :: Nil) = allStates.drop(20)
  val state   = RegInit(sWarm)
  val op      = Reg(UInt(8.W))
  val hdr     = Reg(Vec(8, UInt(8.W)))
  val hdrIdx  = Reg(UInt(3.W))
  val cnt     = Reg(UInt(17.W))
  val rcnt    = Reg(UInt(17.W))
  val reject  = Reg(Bool())
  val status  = Reg(UInt(8.W))
  val gapCnt  = Reg(UInt(4.W))
  val gapNext = Reg(UInt(state.getWidth.W))
  val started = RegInit(false.B)     // the shifter was kicked in this state
  val timeoutCycles = BigInt(math.round(clockHz * busyTimeoutMs / 1000.0))
  val pollTimer = Reg(UInt(log2Ceil(timeoutCycles + 1).W))
  val addr    = Cat(hdr(3), hdr(2), hdr(1), hdr(0))
  val wlen    = Cat(hdr(1), hdr(0))
  val rlen    = Cat(hdr(3), hdr(2))
  val clen    = Cat(hdr(7), hdr(6), hdr(5), hdr(4))
  val crc     = Reg(UInt(32.W))
  val ccnt    = Reg(UInt(26.W))
  val outIdx  = Reg(UInt(3.W))
  val inTop   = (BigInt(1) << 25).U       // 32 MiB part
  val opOk    = VecInit(xferOps.map(_.U(8.W))).contains(io.in.bits)

  io.in.ready := false.B
  io.busy := state =/= sIdle || shBusy || outQ.io.deq.valid

  def gapThen(next: UInt): Unit = {      // CS high for >= tSHSL (50 ns) before next
    csn := true.B; gapCnt := 7.U; gapNext := next; state := sGap
  }

  when (!io.en) {
    state := Mux(state === sWarm, sWarm, sIdle)
    csn := true.B; sck := false.B; shBusy := false.B; started := false.B
  } .otherwise {
    switch (state) {
      is (sWarm) {
        // STARTUPE2 swallows the first three USRCCLKO edges after
        // configuration: spend eight with CS high so no command loses a bit.
        when (!started) { shStart(0xFF.U); started := true.B }
        .elsewhen (shDone) { started := false.B; state := sIdle }
      }
      is (sIdle) {
        io.in.ready := true.B
        when (io.in.valid) { op := io.in.bits; hdrIdx := 0.U; state := sHdr }
      }
      is (sHdr) {
        when (op =/= opXfer.U && op =/= opProg.U && op =/= opErase.U && op =/= opCrc.U) {
          status := errUnknown.U; state := sReply
        } .otherwise {
          io.in.ready := true.B
          when (io.in.valid) {
            hdr(hdrIdx) := io.in.bits
            hdrIdx := hdrIdx + 1.U
            when (hdrIdx === Mux(op === opCrc.U, 7.U, 3.U)) { state := sDispatch }
          }
        }
      }
      is (sDispatch) {
        reject := false.B
        when (op === opXfer.U) {
          cnt := wlen; rcnt := rlen
          when (wlen === 0.U) { reject := true.B; state := sXR } .otherwise { state := sXOp }
        } .elsewhen (op === opProg.U) {
          val bad = addr < guardBase.U || addr(7, 0) =/= 0.U || addr >= inTop
          cnt := 256.U
          when (bad) { status := errGuard.U; state := sDrain } .otherwise { state := sWren }
        } .elsewhen (op === opCrc.U) {
          crc := "hFFFFFFFF".U; ccnt := clen(25, 0)
          when (clen(31, 26) =/= 0.U) { status := errGuard.U; state := sReply } .otherwise { state := sCrcCmd }
        } .otherwise {
          val bad = addr < guardBase.U || addr(15, 0) =/= 0.U || addr >= inTop
          when (bad) { status := errGuard.U; state := sReply } .otherwise { state := sWren }
        }
      }
      // -- XFER --
      is (sXOp) {
        io.in.ready := !started
        when (!started && io.in.valid) {
          when (opOk) { csn := false.B; shStart(io.in.bits); started := true.B }
          .otherwise  { reject := true.B; cnt := cnt - 1.U; state := sXW }
        } .elsewhen (started && shDone) {
          started := false.B; cnt := cnt - 1.U; state := sXW
        }
      }
      is (sXW) {
        when (cnt === 0.U) {
          state := sXR
        } .elsewhen (reject) {
          io.in.ready := true.B
          when (io.in.valid) { cnt := cnt - 1.U }
        } .otherwise {
          io.in.ready := !started
          when (!started && io.in.valid) { shStart(io.in.bits); started := true.B }
          .elsewhen (started && shDone) { started := false.B; cnt := cnt - 1.U }
        }
      }
      is (sXR) {
        when (rcnt === 0.U) {
          state := sXEnd
        } .elsewhen (reject) {
          when (emit(0.U)) { rcnt := rcnt - 1.U }
        } .otherwise {
          when (!started && outQ.io.enq.ready) { shStart(0.U); started := true.B }
          .elsewhen (started && shDone) {
            // The queue had room when the byte started and only this state
            // enqueues, so it still has room now.
            emit(rxByte); started := false.B; rcnt := rcnt - 1.U
          }
        }
      }
      is (sXEnd) {
        status := Mux(reject, errOp.U, ok.U)
        when (reject) { state := sReply } .otherwise { gapThen(sReply) }
      }
      // -- a refused PROG still consumes its page --
      is (sDrain) {
        when (cnt === 0.U) { state := sReply } .otherwise {
          io.in.ready := true.B
          when (io.in.valid) { cnt := cnt - 1.U }
        }
      }
      // -- PROG / ERASE --
      is (sWren) {
        when (!started) { csn := false.B; shStart(0x06.U); started := true.B }
        .elsewhen (shDone) { started := false.B; state := sWrenEnd }
      }
      is (sWrenEnd) { gapThen(sCmd) }
      is (sCmd) {
        when (!started) {
          csn := false.B; shStart(Mux(op === opProg.U, 0x12.U, 0xDC.U)); started := true.B
          hdrIdx := 3.U
        } .elsewhen (shDone) { started := false.B; state := sAddr }
      }
      is (sAddr) {                       // address MSB first
        when (!started) { shStart(hdr(hdrIdx)); started := true.B }
        .elsewhen (shDone) {
          started := false.B
          hdrIdx := hdrIdx - 1.U
          when (hdrIdx === 0.U) {
            state := Mux(op === opProg.U, sData, Mux(op === opCrc.U, sCrcRd, sCmdEnd))
          }
        }
      }
      is (sData) {
        when (cnt === 0.U) { state := sCmdEnd } .otherwise {
          io.in.ready := !started
          when (!started && io.in.valid) { shStart(io.in.bits); started := true.B }
          .elsewhen (started && shDone) { started := false.B; cnt := cnt - 1.U }
        }
      }
      is (sCmdEnd) { pollTimer := timeoutCycles.U; gapThen(sPoll) }
      is (sPoll) {
        when (!started) { csn := false.B; shStart(0x05.U); started := true.B }
        .elsewhen (shDone) { started := false.B; state := sPollRd }
      }
      is (sPollRd) {                     // SR1 repeats for as long as CS is low
        when (pollTimer =/= 0.U) { pollTimer := pollTimer - 1.U }
        when (!started) { shStart(0.U); started := true.B }
        .elsewhen (shDone) {
          started := false.B
          when (!rxByte(0)) { status := rxByte; state := sPollEnd }
          .elsewhen (pollTimer === 0.U) { status := errBusy.U; state := sPollEnd }
        }
      }
      is (sPollEnd) { gapThen(sReply) }
      is (sReply) {
        when (emit(status)) { state := sIdle }
      }
      // -- CRC: 4READ, address via sAddr, then fold every byte in --
      is (sCrcCmd) {
        when (!started) { csn := false.B; shStart(0x13.U); started := true.B; hdrIdx := 3.U }
        .elsewhen (shDone) { started := false.B; state := sAddr }
      }
      is (sCrcRd) {
        when (ccnt === 0.U && !started) {
          outIdx := 0.U; gapThen(sCrcOut)
        } .elsewhen (!started) {
          shStart(0.U); started := true.B
        } .elsewhen (shDone) {
          started := false.B; crc := crcByte(crc, rxByte); ccnt := ccnt - 1.U
        }
      }
      is (sCrcOut) {
        val fin = ~crc
        when (outIdx === 4.U) {
          status := ok.U; state := sReply
        } .elsewhen (emit(VecInit((0 until 4).map(i => fin(8 * i + 7, 8 * i)))(outIdx(1, 0)))) {
          outIdx := outIdx + 1.U
        }
      }
      is (sGap) {
        when (gapCnt === 0.U) { state := gapNext } .otherwise { gapCnt := gapCnt - 1.U }
      }
    }
  }
}
