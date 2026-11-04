package chipyard.esplink

import chisel3._
import chisel3.util._

/** Remote program loading over the FPGA <-> ESP32 link (E13/F14).
 *
 *  The drone carrier has exactly two wires between the FPGA and the ESP32-C6,
 *  and in a flying image they are uart1: telemetry down, uplink commands and
 *  motor duty frames. This block shares those two pins between uart1 and a
 *  UART-TSI client, which is a hardware TileLink master that can write DDR and
 *  start hart 0 whatever the software is doing -- the same job the JTAG debug
 *  module does over the FT2232, but reachable from the ESP and so over WiFi.
 *
 *  TSI cannot simply listen on the same RX line: every telemetry or command
 *  byte would execute as a bus write. So the pins belong to exactly one client
 *  at a time, and the switch is made by hardware, never by Rocket software,
 *  because a crashed image must not be able to lock the loader out:
 *
 *    UART   default. uart1 owns the pins; TSI's RX is held idle.
 *    BREAK  the ESP held its TX low for >= breakUs. A UART byte at any rate
 *           this link uses is low for at most 9 bit times (78 us at 115200,
 *           0.94 ms at 9600), so this cannot happen in traffic. Neither client
 *           hears any of it: both see the line through EspLinkRxFilter, which
 *           drops a low run too long to be a byte before replaying it. Both RX
 *           inputs are held idle from here on.
 *    KEY    the break ended. The next bytes, decoded here at a fixed 115200,
 *           must be exactly `key`, within keyWindowMs. Any other byte, or the
 *           window expiring, drops back to UART. The key is consumed here and
 *           reaches neither client.
 *    FENCE  key accepted; memFence asks the harness to stop the SoC's memory
 *           port at the next TileLink burst boundary and let outstanding
 *           responses drain (memQuiet). The DDR controller's TileLink/AXI
 *           adapters live in the harness reset domain, so resetting the SoC in
 *           the middle of a burst would leave them holding half of it, and the
 *           next requests -- TSI's -- would be consumed as its missing beats.
 *           memFence stays up through RESET. Gives up waiting after fenceMs
 *           and resets anyway: a port that is still busy then is already wedged.
 *    RESET  memory quiet. Sends `ack` to the ESP so it knows a loader-capable
 *           bitstream answered, and holds the whole SoC in reset for resetMs.
 *           That clears TSI's word-assembly state (a stray byte would otherwise
 *           misalign every following 32-bit TSI word) and parks hart 0 in the
 *           boot ROM, waiting for MSIP. DDR and the MIG are in the harness
 *           reset domain and keep their contents.
 *    TSI    TSI owns the pins. Returns to UART after idleMs with no activity in
 *           EITHER direction -- a long --selfcheck read-back is silent on RX for
 *           seconds while TSI is transmitting, so RX alone is not enough. The
 *           exit is silent: no byte reaches either client.
 *
 *  The last key byte selects the session; the first seven are common:
 *
 *    0x3c LOAD   as above: FENCE, RESET, TSI.                      ack "RBLK"
 *    0x3d POKE   TSI with no fence and no reset: the running program keeps
 *                running and the host reads or writes memory under it (the
 *                remote equivalent of a debugger poke, used to write the
 *                flight commit key). TSI's word assembly is NOT reset, so the
 *                ESP only allows it while it knows TSI is at a command
 *                boundary.                                          ack "RBPK"
 *    0x3e FLASH  configuration-flash bridge (EspLinkFlash), 115200. ack "RBFL"
 *    0x3b FLASH  the same at flashFastBaud: the ack goes at 115200,
 *                every byte after it at the fast rate, both ways.  ack "RBFL"
 *    0x3f BOOT   followed by addr:u32le and chk:u32le inside the key
 *                window; then the ack, then ICAPE2 IPROG with WBSTAR = addr
 *                (flash address >> 8 in SPI 32-bit mode).        ack "RBBT"
 *                chk = ~addr ^ (mode * 0x01010101) selects the ICAP sequence
 *                (EspLinkIprog: 0 IPROG, 1 bare IPROG, 2 WBSTAR only, 3
 *                packed); any other chk is ignored.
 *
 *  Every ack is the four ASCII bytes followed by the 32-bit image id
 *  (USR_ACCESS, the bitstream's build timestamp) MSB first, so the ESP can tell
 *  which bitstream answered. FLASH, POKE and BOOT leave the SoC, its reset and
 *  the memory port alone. FLASH returns to UART after idleMs with no traffic
 *  and nothing in progress, or on a break.
 *
 *  Why a break AND a key: GPIO16 is the ESP32-C6's U0TXD, so the ESP ROM and
 *  boot logs go out on this wire at every ESP reset or reflash, and while EN is
 *  low the pin floats. The RX pin is pulled up so a floating line reads idle,
 *  and a boot log cannot contain both a 2 ms low and the 8-byte binary key.
 */
case class EspLinkLoaderParams(
  baud: Int = 115200,
  breakUs: Int = 2000,
  keyWindowMs: Int = 200,
  resetMs: Int = 10,
  fenceMs: Int = 5,
  idleMs: Int = 2000,
  // "RBLD" + four bytes that ASCII text cannot contain.
  key: Seq[Int] = Seq(0x52, 0x42, 0x4c, 0x44, 0xa5, 0x5a, 0xc3, 0x3c),
  ack: Seq[Int] = Seq(0x52, 0x42, 0x4c, 0x4b),    // "RBLK"
  selPoke: Int = 0x3d, ackPoke: Seq[Int] = Seq(0x52, 0x42, 0x50, 0x4b),   // "RBPK"
  selFlash: Int = 0x3e, selFlashFast: Int = 0x3b,
  ackFlash: Seq[Int] = Seq(0x52, 0x42, 0x46, 0x4c),                      // "RBFL"
  selBoot: Int = 0x3f, ackBoot: Seq[Int] = Seq(0x52, 0x42, 0x42, 0x54),   // "RBBT"
  flashFastBaud: Int = 921600,
  // Bytes below this are the golden bitstream: never programmed or erased.
  flashGuardBase: BigInt = 0x00800000,
  flashFifoDepth: Int = 4096,
  iprogDelayUs: Int = 1000)

object EspLinkLoaderMode {
  val width = 4
  val uart  = 0
  val break = 1
  val key   = 2
  val reset = 3
  val tsi   = 4
  val fence = 5
  val ack   = 6
  val flash = 7
  val bootArg = 8
  val iprog = 9
}

/** 8N1 receiver, sampling each bit at its midpoint. Only used for the key. */
class EspLinkByteRx(div: Int) extends Module {
  require(div >= 4)
  val io = IO(new Bundle {
    val in  = Input(Bool())
    val en  = Input(Bool())
    val out = Valid(UInt(8.W))
  })
  val cnt    = RegInit(0.U(log2Ceil(div + 1).W))
  val bitIdx = RegInit(0.U(4.W))
  val shift  = RegInit(0.U(8.W))
  val busy   = RegInit(false.B)

  io.out.valid := false.B
  io.out.bits  := shift

  when (!io.en) {
    busy := false.B
  } .elsewhen (!busy) {
    when (!io.in) {
      busy   := true.B
      cnt    := (div / 2).U
      bitIdx := 0.U
    }
  } .elsewhen (cnt =/= 0.U) {
    cnt := cnt - 1.U
  } .otherwise {
    cnt := (div - 1).U
    when (bitIdx === 0.U) {
      // Start-bit midpoint: still low or it was a glitch.
      when (io.in) { busy := false.B } .otherwise { bitIdx := 1.U }
    } .elsewhen (bitIdx <= 8.U) {
      shift  := Cat(io.in, shift(7, 1))   // LSB first
      bitIdx := bitIdx + 1.U
    } .otherwise {
      // Stop-bit midpoint. A low stop bit is a framing error: drop the byte.
      busy         := false.B
      io.out.valid := io.in
    }
  }
}

/** 8N1 transmitter, idle high. Only used for the ack. */
class EspLinkByteTx(div: Int) extends Module {
  val io = IO(new Bundle {
    val in   = Flipped(Decoupled(UInt(8.W)))
    val out  = Output(Bool())
    val idle = Output(Bool())
  })
  val shift = RegInit(((1 << 10) - 1).U(10.W))
  val left  = RegInit(0.U(4.W))
  val cnt   = RegInit(0.U(log2Ceil(div + 1).W))

  io.in.ready := left === 0.U
  io.idle     := left === 0.U
  io.out      := Mux(left === 0.U, true.B, shift(0))

  when (io.in.fire) {
    shift := Cat(1.U(1.W), io.in.bits, 0.U(1.W))   // stop, data, start
    left  := 10.U
    cnt   := (div - 1).U
  } .elsewhen (left =/= 0.U) {
    when (cnt === 0.U) {
      cnt   := (div - 1).U
      shift := Cat(1.U(1.W), shift(9, 1))
      left  := left - 1.U
    } .otherwise {
      cnt := cnt - 1.U
    }
  }
}

/** EspLinkByteRx/Tx with the bit period chosen at run time (flash sessions). */
class EspLinkVarRx(maxDiv: Int) extends Module {
  val io = IO(new Bundle {
    val in  = Input(Bool())
    val en  = Input(Bool())
    val div = Input(UInt(log2Ceil(maxDiv + 1).W))
    val out = Valid(UInt(8.W))
  })
  val cnt    = RegInit(0.U(log2Ceil(maxDiv + 1).W))
  val bitIdx = RegInit(0.U(4.W))
  val shift  = RegInit(0.U(8.W))
  val busy   = RegInit(false.B)
  io.out.valid := false.B
  io.out.bits  := shift
  when (!io.en) {
    busy := false.B
  } .elsewhen (!busy) {
    when (!io.in) { busy := true.B; cnt := io.div >> 1; bitIdx := 0.U }
  } .elsewhen (cnt =/= 0.U) {
    cnt := cnt - 1.U
  } .otherwise {
    cnt := io.div - 1.U
    when (bitIdx === 0.U) {
      when (io.in) { busy := false.B } .otherwise { bitIdx := 1.U }
    } .elsewhen (bitIdx <= 8.U) {
      shift  := Cat(io.in, shift(7, 1))
      bitIdx := bitIdx + 1.U
    } .otherwise {
      busy         := false.B
      io.out.valid := io.in
    }
  }
}

class EspLinkVarTx(maxDiv: Int) extends Module {
  val io = IO(new Bundle {
    val in   = Flipped(Decoupled(UInt(8.W)))
    val div  = Input(UInt(log2Ceil(maxDiv + 1).W))
    val out  = Output(Bool())
    val idle = Output(Bool())
  })
  val shift = RegInit(((1 << 10) - 1).U(10.W))
  val left  = RegInit(0.U(4.W))
  val cnt   = RegInit(0.U(log2Ceil(maxDiv + 1).W))
  io.in.ready := left === 0.U
  io.idle     := left === 0.U
  io.out      := Mux(left === 0.U, true.B, shift(0))
  when (io.in.fire) {
    shift := Cat(1.U(1.W), io.in.bits, 0.U(1.W))
    left  := 10.U
    cnt   := io.div - 1.U
  } .elsewhen (left =/= 0.U) {
    when (cnt === 0.U) {
      cnt   := io.div - 1.U
      shift := Cat(1.U(1.W), shift(9, 1))
      left  := left - 1.U
    } .otherwise {
      cnt := cnt - 1.U
    }
  }
}

/** The link's RX line as uart1 and TSI see it: delayed by `delay` cycles and
 *  with any low run longer than `longLow` cycles removed.
 *
 *  A break has to be low for breakUs before the mux can tell it from data, and
 *  until then whichever client has the pins hears it: sifive's UARTRx turns it
 *  into 0x00 bytes. For uart1 that is junk in a command parser; for TSI it is
 *  worse -- the stray bytes shift TSI's 32-bit word assembly, and a POKE
 *  session (no SoC reset) would then execute misaligned words as writes into a
 *  running program. A valid 8N1 frame is low for at most 9 bit times, so with
 *  longLow = 11 and delay = 12 bit times every low that is going to turn out to
 *  be a break is recognised before its falling edge is replayed, and it never
 *  reaches either client. The cost is 12 bit times (104 us at 115200) of
 *  latency. Edges are replayed from a small FIFO of (time, level) pairs; at most
 *  ~12 edges fit in one delay window at this baud. */
class EspLinkRxFilter(delay: Int, longLow: Int) extends Module {
  require(longLow < delay && delay < (1 << 15))
  val io = IO(new Bundle {
    val in  = Input(Bool())
    val out = Output(Bool())
  })
  val now  = RegInit(0.U(16.W))
  now := now + 1.U
  val prev = RegNext(io.in, true.B)
  class Edge extends Bundle { val t = UInt(16.W); val lvl = Bool() }
  val q = Module(new Queue(new Edge, 16))
  q.io.enq.valid    := io.in =/= prev
  q.io.enq.bits.t   := now
  q.io.enq.bits.lvl := io.in
  val outLvl = RegInit(true.B)
  q.io.deq.ready := (now - q.io.deq.bits.t) >= delay.U
  when (q.io.deq.fire) { outLvl := q.io.deq.bits.lvl }
  // kill: from the moment a low outlasts longLow until the line has been high
  // long enough for the replay to have passed its rising edge.
  val lowRun  = RegInit(0.U(log2Ceil(longLow + 1).W))
  val highRun = RegInit(0.U(log2Ceil(delay + 3).W))
  val kill    = RegInit(false.B)
  when (io.in) { lowRun := 0.U } .elsewhen (lowRun =/= longLow.U) { lowRun := lowRun + 1.U }
  when (!io.in) { highRun := 0.U } .elsewhen (highRun =/= (delay + 2).U) { highRun := highRun + 1.U }
  when (!io.in && lowRun === longLow.U) { kill := true.B }
  .elsewhen (highRun === (delay + 2).U) { kill := false.B }
  io.out := outLvl || kill
}

class EspLinkMux(clockHz: Double, c: EspLinkLoaderParams = EspLinkLoaderParams()) extends Module {
  val io = IO(new Bundle {
    val pinRx    = Input(Bool())    // from the ESP's TX
    val pinTx    = Output(Bool())   // to the ESP's RX
    val uartRxd  = Output(Bool())   // into the SoC's uart1
    val uartTxd  = Input(Bool())
    val tsiRxd   = Output(Bool())   // into the UART-TSI client
    val tsiTxd   = Input(Bool())
    val socReset = Output(Bool())   // registered: an async reset must not see decode glitches
    val memFence = Output(Bool())   // hold the SoC memory port at a burst boundary
    val memQuiet = Input(Bool())    // port stopped and no responses outstanding
    val mode     = Output(UInt(EspLinkLoaderMode.width.W))
    val spi      = new EspLinkSpiPins      // configuration flash (CCLK via STARTUPE2)
    val icap     = new EspLinkIcapPort
    val imageId  = Input(UInt(32.W))
  })

  private def cycles(us: Double): Int = math.round(clockHz * us / 1e6).toInt
  val div          = math.round(clockHz / c.baud).toInt
  val breakCycles  = cycles(c.breakUs)
  val keyCycles    = cycles(c.keyWindowMs * 1000.0)
  val resetCycles  = cycles(c.resetMs * 1000.0)
  val idleCycles   = cycles(c.idleMs * 1000.0)
  val fenceCycles  = cycles(c.fenceMs * 1000.0)
  val iprogCycles  = cycles(c.iprogDelayUs)
  val fastDiv      = math.round(clockHz / c.flashFastBaud).toInt
  val timerMax     = Seq(keyCycles, resetCycles, idleCycles, fenceCycles, iprogCycles).max
  require(c.key.length >= 2 && Seq(c.ack, c.ackPoke, c.ackFlash, c.ackBoot).forall(_.length == 4))
  val ackLen = 8   // four ASCII bytes + the image id
  require(cycles(c.resetMs * 1000.0) > div * 10 * ackLen,
    "the SoC reset must outlast the ack so the ESP hears it before TSI is live")
  require(fastDiv >= 8 && math.abs(clockHz / fastDiv / c.flashFastBaud - 1.0) < 0.02)

  // Asynchronous input from another chip: synchronise. Idle (high) at reset.
  val rx = RegNext(RegNext(RegNext(io.pinRx, true.B), true.B), true.B)

  // Consecutive-low counter; breakNow is a one-cycle pulse at the threshold.
  val lowCnt = RegInit(0.U(log2Ceil(breakCycles + 1).W))
  when (rx) {
    lowCnt := 0.U
  } .elsewhen (lowCnt =/= breakCycles.U) {
    lowCnt := lowCnt + 1.U
  }
  val breakNow = !rx && lowCnt === (breakCycles - 1).U

  val sUart  = EspLinkLoaderMode.uart.U(EspLinkLoaderMode.width.W)
  val sBreak = EspLinkLoaderMode.break.U(EspLinkLoaderMode.width.W)
  val sKey   = EspLinkLoaderMode.key.U(EspLinkLoaderMode.width.W)
  val sReset = EspLinkLoaderMode.reset.U(EspLinkLoaderMode.width.W)
  val sTsi   = EspLinkLoaderMode.tsi.U(EspLinkLoaderMode.width.W)
  val sFence = EspLinkLoaderMode.fence.U(EspLinkLoaderMode.width.W)
  val sAck   = EspLinkLoaderMode.ack.U(EspLinkLoaderMode.width.W)
  val sFlash = EspLinkLoaderMode.flash.U(EspLinkLoaderMode.width.W)
  val sBootArg = EspLinkLoaderMode.bootArg.U(EspLinkLoaderMode.width.W)
  val sIprog = EspLinkLoaderMode.iprog.U(EspLinkLoaderMode.width.W)

  val state  = RegInit(sUart)
  val timer  = RegInit(0.U(log2Ceil(timerMax + 1).W))
  val keyIdx = RegInit(0.U(log2Ceil(c.key.length + 1).W))
  val ackIdx = RegInit(0.U(log2Ceil(ackLen + 1).W))
  val keyVec = VecInit(c.key.map(_.U(8.W)))
  // Session kind, chosen by the last key byte.
  val kLoad :: kPoke :: kFlash :: kBoot :: Nil = Enum(4)
  val kind   = RegInit(kLoad)
  val fast   = RegInit(false.B)
  val ackVec = VecInit(Seq(c.ack, c.ackPoke, c.ackFlash, c.ackBoot).map(a => VecInit(a.map(_.U(8.W)))))
  val idBytes = VecInit((0 until 4).map(i => io.imageId(31 - 8 * i, 24 - 8 * i)))
  val ackByte = Mux(ackIdx < 4.U, ackVec(kind)(ackIdx(1, 0)), idBytes(ackIdx(1, 0)))
  val bootArg = Reg(Vec(8, UInt(8.W)))
  val bootIdx = RegInit(0.U(4.W))
  val bootAddr = Cat(bootArg(3), bootArg(2), bootArg(1), bootArg(0))
  val bootChk  = Cat(bootArg(7), bootArg(6), bootArg(5), bootArg(4))
  // The check word also picks the ICAP sequence (EspLinkIprog modes):
  // ~addr ^ (mode * 0x01010101), so plain ~addr is mode 0, the real IPROG.
  val bootMatch = (0 until 4).map(m => bootChk === (~bootAddr ^ (BigInt(m) * 0x01010101L).U(32.W)))
  val bootMode  = RegInit(0.U(2.W))

  val byteRx = Module(new EspLinkByteRx(div))
  byteRx.io.in := rx
  byteRx.io.en := state === sKey || state === sBootArg

  val byteTx = Module(new EspLinkByteTx(div))
  byteTx.io.in.valid := (state === sReset || state === sAck) && ackIdx =/= ackLen.U
  byteTx.io.in.bits  := ackByte

  // ---- flash session datapath ----
  val flashOn = state === sFlash
  val curDiv  = Mux(fast, fastDiv.U, div.U)
  val fRx = Module(new EspLinkVarRx(div))
  fRx.io.in  := rx
  fRx.io.en  := flashOn
  fRx.io.div := curDiv
  // Asynchronous-read memory (LUT RAM), NOT useSyncReadMem: Queue's sync
  // read addresses the slot being written whenever the FIFO is empty, and
  // Vivado's READ_FIRST block RAM then returns that slot's OLD contents -- a
  // byte from an earlier session -- as the first byte after every pause. The
  // Verilator memory model is write-first, so only the hardware showed it
  // (2026-11-01: replies to frames from previous sessions).
  val fifo = Module(new Queue(UInt(8.W), c.flashFifoDepth, useSyncReadMem = false, hasFlush = true))
  fifo.io.flush.get := !flashOn
  fifo.io.enq.valid := fRx.io.out.valid
  fifo.io.enq.bits  := fRx.io.out.bits
  val overflow = RegInit(false.B)
  when (!flashOn) { overflow := false.B } .elsewhen (fRx.io.out.valid && !fifo.io.enq.ready) { overflow := true.B }
  val engine = Module(new EspLinkFlash(clockHz, c.flashGuardBase))
  engine.io.en := flashOn
  engine.io.in <> fifo.io.deq
  engine.io.overflow := overflow
  io.spi.sck  := engine.io.spi.sck
  io.spi.csn  := engine.io.spi.csn
  io.spi.mosi := engine.io.spi.mosi
  engine.io.spi.miso := io.spi.miso
  val fTx = Module(new EspLinkVarTx(div))
  fTx.io.div := curDiv
  fTx.io.in.valid := engine.io.out.valid && flashOn
  fTx.io.in.bits  := engine.io.out.bits
  engine.io.out.ready := fTx.io.in.ready && flashOn

  val iprog = Module(new EspLinkIprog)
  iprog.io.addr  := bootAddr
  iprog.io.mode  := bootMode
  val iprogFired = RegInit(false.B)
  iprog.io.start := state === sIprog && timer === 0.U && !iprogFired
  when (state =/= sIprog) { iprogFired := false.B } .elsewhen (iprog.io.start) { iprogFired := true.B }
  io.icap := iprog.io.icap
  when (byteTx.io.in.fire) { ackIdx := ackIdx + 1.U }

  when (breakNow && state =/= sBreak) {
    // From any state, including TSI: a fresh break always restarts the
    // handshake, and a completed handshake always resets the SoC, so whatever
    // the break did to TSI's word assembly is discarded.
    state := sBreak
  } .otherwise {
    switch (state) {
      is (sBreak) {
        when (rx) {
          state  := sKey
          keyIdx := 0.U
          timer  := keyCycles.U
        }
      }
      is (sKey) {
        when (timer === 0.U) {
          state := sUart
        } .otherwise {
          timer := timer - 1.U
        }
        when (byteRx.io.out.valid) {
          val b = byteRx.io.out.bits
          when (keyIdx =/= (c.key.length - 1).U) {
            when (b =/= keyVec(keyIdx)) { state := sUart } .otherwise { keyIdx := keyIdx + 1.U }
          } .otherwise {
            ackIdx := 0.U
            fast   := b === c.selFlashFast.U
            when (b === c.key.last.U) {
              kind := kLoad;  state := sFence; timer := fenceCycles.U
            } .elsewhen (b === c.selPoke.U) {
              kind := kPoke;  state := sAck
            } .elsewhen (b === c.selFlash.U || b === c.selFlashFast.U) {
              kind := kFlash; state := sAck
            } .elsewhen (b === c.selBoot.U) {
              kind := kBoot;  state := sBootArg; bootIdx := 0.U; timer := keyCycles.U
            } .otherwise {
              state := sUart
            }
          }
        }
      }
      is (sBootArg) {
        when (timer === 0.U) { state := sUart } .otherwise { timer := timer - 1.U }
        when (byteRx.io.out.valid) {
          bootArg(bootIdx) := byteRx.io.out.bits
          bootIdx := bootIdx + 1.U
        }
        when (bootIdx === 8.U) {
          state := Mux(bootMatch.reduce(_ || _), sAck, sUart)
          bootMode := PriorityEncoder(bootMatch)
        }
      }
      is (sAck) {
        when (ackIdx === ackLen.U && byteTx.io.idle) {
          timer := idleCycles.U
          state := MuxLookup(kind, sUart)(Seq(kPoke -> sTsi, kFlash -> sFlash, kBoot -> sIprog))
          when (kind === kBoot) { timer := iprogCycles.U }
        }
      }
      is (sFlash) {
        when (!rx || engine.io.busy || fifo.io.deq.valid || !fTx.io.idle) {
          timer := idleCycles.U
        } .elsewhen (timer === 0.U) {
          state := sUart
        } .otherwise {
          timer := timer - 1.U
        }
      }
      is (sIprog) {
        // Wait iprogDelayUs so the ack has left the wire, start IPROG once,
        // and stay here: the FPGA is about to reconfigure. A break gets out
        // if it somehow does not.
        when (timer =/= 0.U) { timer := timer - 1.U }
        // Mode 2 only sets WBSTAR and desyncs: the FPGA keeps running.
        when (iprogFired && !iprog.io.busy && bootMode === 2.U) { state := sUart }
      }
      is (sFence) {
        when (io.memQuiet || timer === 0.U) {
          state  := sReset
          timer  := resetCycles.U
          ackIdx := 0.U
        } .otherwise {
          timer := timer - 1.U
        }
      }
      is (sReset) {
        when (timer =/= 0.U) {
          timer := timer - 1.U
        } .elsewhen (ackIdx === ackLen.U && byteTx.io.idle) {
          state := sTsi
          timer := idleCycles.U
        }
      }
      is (sTsi) {
        when (!rx || !io.tsiTxd) {
          timer := idleCycles.U
        } .elsewhen (timer === 0.U) {
          state := sUart
        } .otherwise {
          timer := timer - 1.U
        }
      }
    }
  }

  val rxFilter = Module(new EspLinkRxFilter(delay = 12 * div, longLow = 11 * div))
  rxFilter.io.in := rx
  io.uartRxd  := Mux(state === sUart, rxFilter.io.out, true.B)
  io.tsiRxd   := Mux(state === sTsi, rxFilter.io.out, true.B)
  io.pinTx    := RegNext(MuxLookup(state, true.B)(Seq(
    sUart  -> io.uartTxd,
    sTsi   -> io.tsiTxd,
    sReset -> byteTx.io.out,
    sAck   -> byteTx.io.out,
    sFlash -> fTx.io.out)), true.B)
  io.socReset := RegNext(state === sReset, false.B)
  io.memFence := state === sFence || state === sReset || io.socReset
  io.mode     := state
}
