package uartdma

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Config, Field, Parameters}
import freechips.rocketchip.subsystem.{BaseSubsystem, PBUS, FBUS}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.prci._
import freechips.rocketchip.regmapper.{RegField, RegWriteFn}
import freechips.rocketchip.tilelink._

/** SoC-level parameters for the memory -> UART TX DMA engine.
  *
  * `address`        base of this DMA's own control/status MMIO register block (a dedicated 4 KiB
  *                  page, placed adjacent to uart1 -- see RocketArty200TDroneFullDDRDmaUartConfig).
  * `uartTxDataAddr` absolute address of the SiFive UART's TXDATA register that this DMA feeds. For
  *                  the drone SoC that is uart1 (the ESP link) at 0x1002_1000, TXDATA @ offset 0x00,
  *                  read-bit31 = TX-FIFO-full, write-[7:0] = enqueue a byte.
  */
case class UartTxDmaParams(
  address:        BigInt = 0x10022000L,
  uartTxDataAddr: BigInt = 0x10021000L
)

case object UartTxDmaKey extends Field[Option[UartTxDmaParams]](None)

/** A memory -> UART TX DMA with hardware flow control.
  *
  * Software hands the engine a DDR buffer ({DMA_ADDR_HI,DMA_ADDR_LO}, DMA_LEN bytes) and pulses
  * START. Hardware then streams the buffer, in ascending byte-address order (byte N of the buffer,
  * at ADDR+N, is the N-th byte transmitted), into the SiFive UART's TXDATA register one byte at a
  * time, *stalling whenever the TX FIFO is full* so no byte is ever dropped. The CPU no longer spins
  * on the FIFO-full bit; it polls DMA_STATUS.DONE (or just fires and forgets).
  *
  * This peripheral owns NO chip pins -- it drives the *existing* uart1 through the bus fabric, so
  * there is no harness binder / IOBinder to add. It is two diplomatic nodes:
  *   - `node`    : a TileLink *slave* register block (its DMA_* CSRs), on the periphery bus.
  *   - `dmaNode` : a single-outstanding TileLink *master* (sourceId 0..1, all-address visibility,
  *                 exactly like the OSPI capture DMA). It both READS the source bytes from DDR and
  *                 WRITES/POLLS the uart1 TXDATA register, so it must reach the whole map; default
  *                 visibility does that. It is coupled onto the front bus (FBUS), the same bus the
  *                 OSPI DMA uses to reach DDR -- from there the fabric routes 0x8000_0000 to the MIG
  *                 and 0x1002_1000 to uart1.
  *
  * Register map (this block's base = `params.address`). Offsets deliberately mirror the OSPI
  * capture DMA's DMA_* sub-block so firmware register conventions carry over 1:1:
  *   0x44 DMA_ADDR_LO [31:0]  low  word of the 64-bit DDR source base address (RW)
  *   0x48 DMA_ADDR_HI [31:0]  high word of the 64-bit DDR source base address (RW)
  *   0x4c DMA_LEN     number of bytes to stream out the UART (RW). 0 = no-op (completes instantly).
  *   0x50 DMA_CTRL    [0]=enable (arm the engine)
  *                    write-one pulses: [1]=start (kick a transfer now), [3]=clear done/error
  *                    [2] reserved
  *   0x54 DMA_STATUS  [0]=busy [1]=done [2]=error [7:4]=engine state (RO)
  *   0x58 DMA_BYTES   bytes handed to the UART by the current/last transfer (RO)
  *
  * Flow control: before every byte the engine issues a 4-byte Get of TXDATA and inspects bit31; if
  * the FIFO is full it re-reads (stalls) until a slot frees, then issues a 4-byte PutFull whose low
  * byte lane carries the data. Because the engine is single-outstanding and strictly ordered
  * (poll-then-write), the slot it observed free is still free when it writes. CAVEAT: while a
  * transfer is BUSY, software must not also write uart1 TXDATA directly -- the two writers would
  * race on the FIFO. Byte ordering is little-endian in address (ADDR+0 goes out first).
  */
class UartTxDma(params: UartTxDmaParams, beatBytes: Int)(implicit p: Parameters)
    extends ClockSinkDomain(ClockSinkParameters())(p) {
  val device = new SimpleDevice("uart-tx-dma", Seq("ucbbar,uart-tx-dma"))
  val node = TLRegisterNode(
    Seq(AddressSet(params.address, 4096 - 1)),
    device,
    "reg/control",
    beatBytes = beatBytes)

  // Single in-flight transaction (sourceId 0..1), default (all-address) visibility, matching the
  // OSPI capture DMA. Reads the source from DDR *and* writes/polls the uart1 TXDATA register.
  val dmaNode = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLClientParameters(
    name = "uart-tx-dma", sourceId = IdRange(0, 1))))))

  override lazy val module = new UartTxDmaImpl
  class UartTxDmaImpl extends Impl {
    withClockAndReset(clock, reset) {
      // ---- Control/config registers (same layout & offsets as the OSPI DMA sub-block) ----
      val dmaAddrLoReg  = RegInit(0.U(32.W))
      val dmaAddrHiReg  = RegInit(0.U(32.W))
      val dmaLenReg     = RegInit(0.U(32.W))
      val dmaEnableReg  = RegInit(false.B)
      val dmaStartPulse = WireDefault(false.B)
      val dmaClearPulse = WireDefault(false.B)

      val dmaStartWrite = RegWriteFn((valid: Bool, data: UInt) => {
        dmaStartPulse := valid && data(0); true.B
      })
      val dmaClearWrite = RegWriteFn((valid: Bool, data: UInt) => {
        dmaClearPulse := valid && data(0); true.B
      })

      val (mem, edge) = dmaNode.out(0)
      val addrBits = edge.bundle.addressBits
      val busBytes = edge.bundle.dataBits / 8
      require(busBytes >= 1)
      val lgBeat   = log2Ceil(busBytes)

      // ---- Status / engine state ----
      val busyReg      = RegInit(false.B)
      val doneReg      = RegInit(false.B)
      val errReg       = RegInit(false.B)
      val bytesReg     = RegInit(0.U(32.W))   // bytes handed to the UART so far
      val pendingStart = RegInit(false.B)

      // Latched at start so a mid-transfer CSR write cannot corrupt the running transfer.
      val baseAddr = Reg(UInt(64.W))
      val lenBytes = Reg(UInt(32.W))
      val idx      = Reg(UInt(32.W))          // byte offset within the transfer
      val curByte  = Reg(UInt(8.W))           // byte staged for the UART

      when(dmaStartPulse) { pendingStart := true.B }

      val sIdle :: sRead :: sReadResp :: sPoll :: sPollResp :: sWrite :: sWriteResp :: sDone :: Nil =
        Enum(8)
      val state = RegInit(sIdle)

      // Source-read (DDR) address and the fixed UART TXDATA target.
      val srcAddr  = (baseAddr + idx)(addrBits - 1, 0)
      val laneOff  = srcAddr(lgBeat - 1, 0)                    // byte lane within the read beat
      val uartAddr = params.uartTxDataAddr.U(addrBits.W)

      // A: 1-byte Get from DDR; 4-byte Get of TXDATA (poll full); 4-byte PutFull of one byte.
      val getSrc = edge.Get(fromSource = 0.U, toAddress = srcAddr,  lgSize = 0.U)._2
      val getTx  = edge.Get(fromSource = 0.U, toAddress = uartAddr, lgSize = 2.U)._2
      val putTx  = edge.Put(fromSource = 0.U, toAddress = uartAddr, lgSize = 2.U,
                            data = curByte.pad(busBytes * 8))._2

      mem.a.valid := (state === sRead) || (state === sPoll) || (state === sWrite)
      mem.a.bits  := Mux1H(Seq(
        (state === sRead)  -> getSrc,
        (state === sPoll)  -> getTx,
        (state === sWrite) -> putTx))
      mem.d.ready := (state === sReadResp) || (state === sPollResp) || (state === sWriteResp)
      // Master issues only Get/Put on A; tie off the coherence channels we never use.
      mem.b.ready := false.B
      mem.c.valid := false.B
      mem.e.valid := false.B

      // Extract the addressed byte lane from the DDR read response (TileLink aligns sub-beat data).
      val respBytes = VecInit((0 until busBytes).map(i => mem.d.bits.data(8 * i + 7, 8 * i)))
      val txFull    = mem.d.bits.data(31)     // TXDATA bit31; TXDATA sits at lane 0 of an aligned beat

      switch(state) {
        is(sIdle) {
          busyReg := false.B
          when(dmaEnableReg && pendingStart) {
            pendingStart := false.B
            busyReg  := true.B
            doneReg  := false.B
            errReg   := false.B
            baseAddr := Cat(dmaAddrHiReg, dmaAddrLoReg)
            lenBytes := dmaLenReg
            idx      := 0.U
            bytesReg := 0.U
            state    := Mux(dmaLenReg === 0.U, sDone, sRead)
          }
        }
        is(sRead) {
          when(mem.a.fire) { state := sReadResp }
        }
        is(sReadResp) {
          when(mem.d.fire) {
            curByte := respBytes(laneOff)
            errReg  := errReg || mem.d.bits.denied || mem.d.bits.corrupt
            state   := sPoll
          }
        }
        is(sPoll) {
          when(mem.a.fire) { state := sPollResp }
        }
        is(sPollResp) {
          when(mem.d.fire) {
            // Hardware flow control: TX FIFO full -> re-poll (stall); else push the byte.
            state := Mux(txFull, sPoll, sWrite)
          }
        }
        is(sWrite) {
          when(mem.a.fire) { state := sWriteResp }
        }
        is(sWriteResp) {
          when(mem.d.fire) {
            errReg := errReg || mem.d.bits.denied || mem.d.bits.corrupt
            val nextIdx = idx + 1.U
            idx      := nextIdx
            bytesReg := nextIdx
            state    := Mux(nextIdx === lenBytes, sDone, sRead)
          }
        }
        is(sDone) {
          busyReg := false.B
          doneReg := true.B
          state   := sIdle
        }
      }

      when(dmaClearPulse) {
        doneReg := false.B
        errReg  := false.B
      }

      node.regmap(
        0x44 -> Seq(RegField(32, dmaAddrLoReg)),
        0x48 -> Seq(RegField(32, dmaAddrHiReg)),
        0x4c -> Seq(RegField(32, dmaLenReg)),
        0x50 -> Seq(
          RegField(1, dmaEnableReg),      // [0] enable / arm
          RegField.w(1, dmaStartWrite),   // [1] start (W1P)
          RegField(1),                    // [2] reserved
          RegField.w(1, dmaClearWrite),   // [3] clear done/error (W1P)
          RegField(28)),
        0x54 -> Seq(RegField.r(8, Cat(state.pad(4), 0.U(1.W), errReg, doneReg, busyReg))),
        0x58 -> Seq(RegField.r(32, bytesReg))
      )
    }
  }
}

/** Mix into the SoC to optionally instantiate the UART TX DMA engine. Reads `UartTxDmaKey`; when the
  * key is None (every existing config) this adds no node and no hardware, exactly like the OSPI
  * trait already on DigitalTop. */
trait CanHavePeripheryUartTxDma { this: BaseSubsystem =>
  private val portName = "uart-tx-dma"
  private val pbus = locateTLBusWrapper(PBUS)

  val uart_tx_dma = p(UartTxDmaKey).map { params =>
    val dmaLM = LazyModule(new UartTxDma(params, pbus.beatBytes)(p))
    dmaLM.clockNode := pbus.fixedClockNode

    // Slave register block on the periphery bus (mirrors the OSPI register-node crossing).
    pbus.coupleTo(portName) {
      TLInwardClockCrossingHelper(s"${portName}_crossing", dmaLM, dmaLM.node)(SynchronousCrossing()) :=
      TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _
    }

    // Master onto the front bus: reaches DDR (the source buffer) and uart1 TXDATA (the sink).
    val fbus = locateTLBusWrapper(FBUS)
    fbus.coupleFrom(s"${portName}_dma") {
      _ := TLOutwardClockCrossingHelper(s"${portName}_dma_crossing", dmaLM, dmaLM.dmaNode)(SynchronousCrossing())
    }
  }
}

/** Config fragment: enable the memory -> UART TX DMA engine.
  *
  * @param address        base of the DMA's control/status MMIO page (default 0x1002_2000, the page
  *                        just above uart1's 0x1002_1000 window)
  * @param uartTxDataAddr absolute TXDATA address the DMA feeds (default uart1 @ 0x1002_1000)
  */
class WithUartTxDma(
  address:        BigInt = 0x10022000L,
  uartTxDataAddr: BigInt = 0x10021000L
) extends Config((site, here, up) => {
  case UartTxDmaKey => Some(UartTxDmaParams(address = address, uartTxDataAddr = uartTxDataAddr))
})
