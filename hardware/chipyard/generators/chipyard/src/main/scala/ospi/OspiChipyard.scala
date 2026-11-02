package ospi

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Config, Field, Parameters}
import freechips.rocketchip.subsystem.{BaseSubsystem, PBUS, FBUS}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.interrupts.{IntSourceNode, IntSourcePortSimple}
import freechips.rocketchip.prci._
import freechips.rocketchip.regmapper.{RegField, RegReadFn, RegWriteFn}
import freechips.rocketchip.tilelink._

/** SoC-level parameters for the HM01B0 capture peripheral. */
case class OspiParams(
  address:          BigInt = 0x10080000L,
  dataWidth:        Int    = 8,
  defaultWidth:     Int    = 324,
  defaultHeight:    Int    = 244,
  maxWidth:         Int    = 324,
  maxHeight:        Int    = 324,
  captureFifoDepth: Int    = 1024,
  frameBufferDepth: Int    = 324 * 324 + 1,
  // When true the peripheral gains a TileLink *master* (DMA) port that writes captured pixel
  // bytes straight into DDR, in addition to the existing MMIO drain register. Defaults to false
  // so every existing config elaborates byte-for-byte identically (no extra diplomatic node).
  enableDma:        Boolean = false
) {
  // A full-frame buffer is what a streaming configuration needs. A bring-up configuration
  // deliberately sizes this for a single line instead, so the requirement is that the buffer can
  // hold whatever bounded capture software asks for -- checked in software via CAPACITY -- with a
  // floor that keeps the EOF marker from displacing pixel data.
  require(frameBufferDepth >= 2, "frameBufferDepth must hold at least a pixel and its EOF marker")
  def holdsFullFrame: Boolean = frameBufferDepth >= maxWidth * maxHeight + 1
}

case object OspiKey extends Field[Option[OspiParams]](None)

/** TileLink MMIO wrapper around [[HM01B0Capture]].
  *
  * The capture core drains continuously into a block-RAM frame buffer. This is essential: the
  * core only commits `frameCount` when its in-band EOF marker is accepted, so exposing the CDC
  * FIFO directly to software would make "wait for frame, then drain" deadlock. The default frame
  * buffer holds a complete 324x324 frame plus its EOF marker.
  *
  * Register map (word offsets):
  *   0x00 CTRL      [0]=enable [1]=continuous [2]=irqEnable
  *                  write-one pulses: [3]=trigger [4]=clear status/IRQ [5]=flush frame buffer
  *   0x04 GEOM      [15:0]=expWidth [31:16]=expHeight
  *   0x08 MCLKDIV   MCLK = sysclk / (2*(mclkDiv+1))
  *   0x0c FIFOCOUNT number of buffered pixel/marker beats (RO)
  *   0x10 FRAMECNT  completed frames committed to the frame buffer (RO)
  *   0x14 LASTWIDTH measured width of the last completed frame (RO)
  *   0x18 LASTHEIGHT measured height of the last completed frame (RO)
  *   0x1c FLAGS     [0]=dataValid [1]=overflow [2]=geomErr [3]=sensorInt [4]=busy
  *                  [5]=irqPending [6]=frameBufferFull (RO)
  *   0x20 DATA      nonblocking read/pop: [31]=valid [10]=eof [9]=eol [8]=sof [7:0]=data
  *   0x24 CAPACITY  frame-buffer capacity in beats (RO)
  *
  * Bring-up diagnostics. There is no ILA on this board, so these counters and the sticky flags
  * above are the only observability into the sensor interface:
  *   0x28 PIXTARGET pixels to capture after ARM; 0 = free-running (RW)
  *   0x2c CAPCOUNT  pixel beats captured in the current/last bounded capture (RO)
  *   0x30 PCLKCNT   PCLK rising edges observed (RO) -- zero means no camera clock at all
  *   0x34 FVLDCNT   FVLD rising edges observed (RO) -- zero means no frame sync
  *   0x38 LVLDCNT   LVLD rising edges observed (RO) -- zero means no line sync
  *   0x3c LASTPIX   [7:0] most recent pixel value presented (RO)
  *   0x40 CAPSTAT   [0]=armed [1]=captureDone (RO)
  *
  * DMA path (only present when `enableDma`). When enabled, hardware drains the frame buffer into a
  * software-configured DDR buffer over a TileLink master port instead of the CPU polling DATA. The
  * MMIO DATA drain (0x20) still exists but is *inhibited* while DMA_CTRL.enable is set so the two
  * consumers never race on the frame buffer's read port. One byte is packed per pixel, little-endian
  * within a beat (pixel N lands at DMA_ADDR + N):
  *   0x44 DMA_ADDR_LO [31:0]  low  word of the 64-bit DDR target base address (RW)
  *   0x48 DMA_ADDR_HI [31:0]  high word of the 64-bit DDR target base address (RW)
  *   0x4c DMA_LEN     byte cap for one transfer; 0 = drain until the in-band EOF marker (RW)
  *   0x50 DMA_CTRL    [0]=enable (route drain to DMA, inhibit MMIO DATA)
  *                    [2]=auto  (start a transfer automatically when a frame completes)
  *                    write-one pulses: [1]=start (kick a transfer now), [3]=clear done/error
  *   0x54 DMA_STATUS  [0]=busy [1]=done [2]=error [3]=sawEof [7:4]=engine state (RO)
  *   0x58 DMA_BYTES   bytes written to DDR by the current/last transfer (RO)
  */
class OspiCapture(params: OspiParams, beatBytes: Int)(implicit p: Parameters)
    extends ClockSinkDomain(ClockSinkParameters())(p) {
  val device = new SimpleDevice("ospi", Seq("ucbbar,ospi-hm01b0"))
  val node = TLRegisterNode(
    Seq(AddressSet(params.address, 4096 - 1)),
    device,
    "reg/control",
    beatBytes = beatBytes)
  val intnode = IntSourceNode(IntSourcePortSimple(num = 1, resources = device.int))

  // Optional TileLink master (DMA) node. Created only when DMA is enabled so that non-DMA configs
  // introduce no dangling diplomatic node. A single in-flight transaction (sourceId 0..1) with
  // default (all-address) visibility, matching chipyard's InitZero / testchipip BlockDevice masters.
  val dmaNode: Option[TLClientNode] =
    if (params.enableDma)
      Some(TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLClientParameters(
        name = "ospi-dma", sourceId = IdRange(0, 1)))))))
    else None

  override lazy val module = new OspiImpl
  class OspiImpl extends Impl {
    private val cp = CaptureParams(
      dataWidth = params.dataWidth,
      defaultWidth = params.defaultWidth,
      defaultHeight = params.defaultHeight,
      maxWidth = params.maxWidth,
      maxHeight = params.maxHeight,
      fifoDepth = params.captureFifoDepth)

    val io = IO(new SensorIO(params.dataWidth))

    withClockAndReset(clock, reset) {
      val capture = Module(new HM01B0Capture(cp))
      // Isolate the async-reset CDC dequeue flag from the block-RAM write controls. Besides adding
      // two beats of elasticity, this synchronously reset register queue lets Vivado analyze the
      // frame-buffer enables without REQP-1839 async-control warnings.
      val capturePipe = Module(new Queue(new PixelFlit(cp.dataWidth), entries = 2))
      val frameBuffer = Module(new FrameBuffer(cp, params.frameBufferDepth))

      // Sensor pins: chip IO <-> capture core.
      capture.io.sensor.pclk := io.pclk
      capture.io.sensor.fvld := io.fvld
      capture.io.sensor.lvld := io.lvld
      capture.io.sensor.d    := io.d
      capture.io.sensor.intr := io.intr
      io.mclk := capture.io.sensor.mclk
      io.trig := capture.io.sensor.trig

      // Persistent controls and write-one command pulses.
      val enableReg     = RegInit(false.B)
      val continuousReg = RegInit(false.B)
      val irqEnableReg  = RegInit(false.B)
      val geomInit      = (BigInt(cp.defaultHeight) << 16) | BigInt(cp.defaultWidth)
      val geomReg       = RegInit(geomInit.U(32.W))
      val mclkDivReg    = RegInit(0.U(cp.mclkDivWidth.W))
      val pixTargetReg  = RegInit(0.U(32.W))
      val triggerPulse  = WireDefault(false.B)
      val clearPulse    = WireDefault(false.B)
      val flushPulse    = WireDefault(false.B)
      val armPulse      = WireDefault(false.B)

      val triggerWrite = RegWriteFn((valid: Bool, data: UInt) => {
        triggerPulse := valid && data(0)
        true.B
      })
      val clearWrite = RegWriteFn((valid: Bool, data: UInt) => {
        clearPulse := valid && data(0)
        true.B
      })
      val flushWrite = RegWriteFn((valid: Bool, data: UInt) => {
        flushPulse := valid && data(0)
        true.B
      })
      val armWrite = RegWriteFn((valid: Bool, data: UInt) => {
        armPulse := valid && data(0)
        true.B
      })

      capture.io.ctrl.enable     := enableReg
      capture.io.ctrl.continuous := continuousReg
      capture.io.ctrl.irqEnable  := irqEnableReg
      capture.io.ctrl.trigPulse  := triggerPulse
      capture.io.ctrl.clear      := clearPulse
      capture.io.ctrl.mclkDiv    := mclkDivReg
      capture.io.ctrl.expWidth   := geomReg(cp.pixCountWidth - 1, 0)
      capture.io.ctrl.expHeight  := geomReg(16 + cp.lineCountWidth - 1, 16)
      capture.io.ctrl.arm        := armPulse
      capture.io.ctrl.pixelTarget := pixTargetReg

      // Drain the CDC FIFO autonomously. Pause both handshakes for the flush cycle; buffered CDC
      // or elastic-stage beats remain in order and resume on the next cycle.
      capturePipe.io.enq.valid := capture.io.pixels.valid && !flushPulse
      capturePipe.io.enq.bits  := capture.io.pixels.bits
      capture.io.pixels.ready  := capturePipe.io.enq.ready && !flushPulse
      frameBuffer.io.enq.valid := capturePipe.io.deq.valid && !flushPulse
      frameBuffer.io.enq.bits  := capturePipe.io.deq.bits
      capturePipe.io.deq.ready := frameBuffer.io.enq.ready && !flushPulse
      frameBuffer.io.flush     := flushPulse

      // ---- Optional DMA engine (TileLink master). ----
      // Shared handles so the frame-buffer read-port mux and the interrupt/regmap below can be
      // written uniformly whether or not DMA is compiled in.
      val dmaEnableReg = RegInit(false.B)            // route drain to DMA + inhibit MMIO DATA
      val dmaPopReady  = WireDefault(false.B)        // DMA-side frame-buffer pop request
      val dmaDonePulse = WireDefault(false.B)        // 1-cycle: a transfer just finished
      // Status wires surfaced to the regmap (constant when DMA is not compiled in).
      val dmaBusy      = WireDefault(false.B)
      val dmaDone      = WireDefault(false.B)
      val dmaError     = WireDefault(false.B)
      val dmaSawEof    = WireDefault(false.B)
      val dmaState     = WireDefault(0.U(4.W))
      val dmaBytesOut  = WireDefault(0.U(32.W))
      // DMA control/config registers exist unconditionally (harmless RW scratch when DMA is off),
      // but only take effect when the master node is present.
      val dmaAddrLoReg = RegInit(0.U(32.W))
      val dmaAddrHiReg = RegInit(0.U(32.W))
      val dmaLenReg    = RegInit(0.U(32.W))
      val dmaAutoReg   = RegInit(false.B)
      val dmaStartPulse = WireDefault(false.B)
      val dmaClearPulse = WireDefault(false.B)

      val dmaStartWrite = RegWriteFn((valid: Bool, data: UInt) => {
        dmaStartPulse := valid && data(0)
        true.B
      })
      val dmaClearWrite = RegWriteFn((valid: Bool, data: UInt) => {
        dmaClearPulse := valid && data(0)
        true.B
      })

      dmaNode.foreach { dn =>
        val (mem, edge) = dn.out(0)
        val addrBits = edge.bundle.addressBits
        val busBytes = edge.bundle.dataBits / 8
        require(busBytes >= 1)
        val lgBeat   = log2Ceil(busBytes)
        val cntW     = log2Ceil(busBytes + 1)

        // Byte-packing buffer: one captured pixel byte per lane, little-endian within a beat.
        val msgBuffer    = Reg(Vec(busBytes, UInt(8.W)))
        val collectCount = RegInit(0.U(cntW.W))
        val bytesWritten = RegInit(0.U(32.W))
        val busyReg      = RegInit(false.B)
        val doneReg      = RegInit(false.B)
        val errReg       = RegInit(false.B)
        val sawEofReg    = RegInit(false.B)
        val finalBeat    = RegInit(false.B)
        val pendingStart = RegInit(false.B)

        val dIdle :: dCollect :: dWrite :: dResp :: dFinish :: Nil = Enum(5)
        val dstate = RegInit(dIdle)

        // A transfer is kicked either by an explicit start pulse or, in auto mode, when a whole
        // frame (its EOF marker) has just been committed to the frame buffer.
        when(dmaStartPulse || (dmaAutoReg && frameBuffer.io.frameReady)) {
          pendingStart := true.B
        }

        // Byte cap: bytesWritten already committed + bytes staged this beat.
        val lenReached = (dmaLenReg =/= 0.U) &&
          ((bytesWritten +& collectCount) >= dmaLenReg)

        // Single-beat PutFullData for a full beat; PutPartialData (masked) for a short tail beat.
        val writeAddr   = (Cat(dmaAddrHiReg, dmaAddrLoReg) +& bytesWritten)(addrBits - 1, 0)
        val beatData    = Cat(msgBuffer.reverse) // msgBuffer(0) -> least-significant byte lane
        val fullBeat    = collectCount === busBytes.U
        val partialMask = ((1.U << collectCount) - 1.U)(busBytes - 1, 0)
        val putFull = edge.Put(
          fromSource = 0.U, toAddress = writeAddr, lgSize = lgBeat.U, data = beatData)._2
        val putPart = edge.Put(
          fromSource = 0.U, toAddress = writeAddr, lgSize = lgBeat.U,
          data = beatData, mask = partialMask)._2

        mem.a.valid := dstate === dWrite
        mem.a.bits  := Mux(fullBeat, putFull, putPart)
        mem.d.ready := dstate === dResp
        // Master issues only Get/Put on A; tie off the coherence channels we never use.
        mem.b.ready := false.B
        mem.c.valid := false.B
        mem.e.valid := false.B

        switch(dstate) {
          is(dIdle) {
            busyReg := false.B
            when(dmaEnableReg && pendingStart) {
              pendingStart := false.B
              busyReg      := true.B
              doneReg      := false.B
              errReg       := false.B
              sawEofReg    := false.B
              finalBeat    := false.B
              collectCount := 0.U
              bytesWritten := 0.U
              dstate       := dCollect
            }
          }
          is(dCollect) {
            when(lenReached) {
              finalBeat := true.B
              dstate    := Mux(collectCount === 0.U, dFinish, dWrite)
            }.elsewhen(frameBuffer.io.deq.valid) {
              dmaPopReady := true.B // pop this flit (deq.ready is muxed to this below)
              when(frameBuffer.io.deq.bits.eof) {
                sawEofReg := true.B
                finalBeat := true.B
                dstate    := Mux(collectCount === 0.U, dFinish, dWrite)
              }.otherwise {
                msgBuffer(collectCount) := frameBuffer.io.deq.bits.data
                val nextCount = collectCount + 1.U
                collectCount := nextCount
                when(nextCount === busBytes.U) {
                  finalBeat := false.B
                  dstate    := dWrite
                }
              }
            }
            // else: frame buffer momentarily empty -> hold in dCollect until more data / EOF.
          }
          is(dWrite) {
            when(mem.a.fire) { dstate := dResp }
          }
          is(dResp) {
            when(mem.d.fire) {
              bytesWritten := bytesWritten + collectCount
              errReg       := errReg || mem.d.bits.denied || mem.d.bits.corrupt
              collectCount := 0.U
              dstate       := Mux(finalBeat, dFinish, dCollect)
            }
          }
          is(dFinish) {
            busyReg      := false.B
            doneReg      := true.B
            dmaDonePulse := true.B
            dstate       := dIdle
          }
        }

        when(dmaClearPulse) {
          doneReg := false.B
          errReg  := false.B
        }

        dmaBusy     := busyReg
        dmaDone     := doneReg
        dmaError    := errReg
        dmaSawEof   := sawEofReg
        dmaState    := dstate.pad(4)
        dmaBytesOut := bytesWritten
      }

      // ---- Frame-buffer read-port arbitration ----
      // Exactly one consumer pops at a time: the DMA engine when DMA is enabled, otherwise the
      // nonblocking MMIO DATA register. This keeps the legacy drain path bit-identical when DMA is
      // off or not compiled in.
      val mmioPopReady = WireDefault(false.B)
      val dmaActive    = if (params.enableDma) dmaEnableReg else false.B
      frameBuffer.io.deq.ready := Mux(dmaActive, dmaPopReady, mmioPopReady)

      // Latch completion so the PLIC sees a level until software acknowledges it with CTRL.clear.
      // In DMA mode the completion event is "frame landed in DDR" (dmaDonePulse); otherwise it is
      // "frame committed to the frame buffer" (frameReady), preserving legacy behaviour.
      val irqPending = RegInit(false.B)
      when(clearPulse) { irqPending := false.B }
      if (params.enableDma) {
        when(frameBuffer.io.frameReady && !dmaEnableReg) { irqPending := true.B }
        when(dmaDonePulse) { irqPending := true.B }
      } else {
        when(frameBuffer.io.frameReady) { irqPending := true.B }
      }
      val (interrupts, _) = intnode.out(0)
      interrupts(0) := irqPending && irqEnableReg

      val flags = Cat(
        frameBuffer.io.full,           // [6]
        irqPending,                    // [5]
        capture.io.status.busy,        // [4]
        capture.io.status.sensorInt,   // [3]
        capture.io.status.geomErr,     // [2]
        capture.io.status.overflow,    // [1]
        frameBuffer.io.deq.valid       // [0]
      )

      // DATA never stalls TileLink. An empty read returns zero; bit 31 distinguishes that from a
      // valid all-zero pixel. A valid read pops exactly one buffered flit -- but only when DMA is
      // not draining the buffer (dmaActive gates the pop in the mux above).
      val dataWord = Mux(frameBuffer.io.deq.valid,
        Cat(
          1.B,
          0.U(20.W),
          frameBuffer.io.deq.bits.eof,
          frameBuffer.io.deq.bits.eol,
          frameBuffer.io.deq.bits.sof,
          frameBuffer.io.deq.bits.data),
        0.U(32.W))
      val dataRead = RegReadFn((ready: Bool) => {
        mmioPopReady := ready
        (true.B, dataWord)
      })

      val baseMap: Seq[(Int, Seq[RegField])] = Seq(
        0x00 -> Seq(
          RegField(1, enableReg),
          RegField(1, continuousReg),
          RegField(1, irqEnableReg),
          RegField.w(1, triggerWrite),
          RegField.w(1, clearWrite),
          RegField.w(1, flushWrite),
          RegField.w(1, armWrite),
          RegField(25)),
        0x04 -> Seq(RegField(32, geomReg)),
        0x08 -> Seq(RegField(cp.mclkDivWidth, mclkDivReg)),
        0x0c -> Seq(RegField.r(32, frameBuffer.io.count.pad(32))),
        0x10 -> Seq(RegField.r(32, capture.io.status.frameCount)),
        0x14 -> Seq(RegField.r(16, capture.io.status.lastWidth.pad(16))),
        0x18 -> Seq(RegField.r(16, capture.io.status.lastHeight.pad(16))),
        0x1c -> Seq(RegField.r(7, flags)),
        0x20 -> Seq(RegField.r(32, dataRead)),
        0x24 -> Seq(RegField.r(32, params.frameBufferDepth.U(32.W))),
        0x28 -> Seq(RegField(32, pixTargetReg)),
        0x2c -> Seq(RegField.r(32, capture.io.status.capturedPixels)),
        0x30 -> Seq(RegField.r(32, capture.io.status.pclkCount.pad(32))),
        0x34 -> Seq(RegField.r(32, capture.io.status.fvldRises.pad(32))),
        0x38 -> Seq(RegField.r(32, capture.io.status.lvldRises.pad(32))),
        0x3c -> Seq(RegField.r(8, capture.io.status.lastPixel.pad(8))),
        0x40 -> Seq(RegField.r(2, Cat(capture.io.status.captureDone, capture.io.status.armed)))
      )

      val dmaMap: Seq[(Int, Seq[RegField])] =
        if (params.enableDma) Seq(
          0x44 -> Seq(RegField(32, dmaAddrLoReg)),
          0x48 -> Seq(RegField(32, dmaAddrHiReg)),
          0x4c -> Seq(RegField(32, dmaLenReg)),
          0x50 -> Seq(
            RegField(1, dmaEnableReg),      // [0] enable / route to DMA
            RegField.w(1, dmaStartWrite),   // [1] start (W1P)
            RegField(1, dmaAutoReg),        // [2] auto-start on frame complete
            RegField.w(1, dmaClearWrite),   // [3] clear done/error (W1P)
            RegField(28)),
          0x54 -> Seq(RegField.r(8, Cat(dmaState, dmaSawEof, dmaError, dmaDone, dmaBusy))),
          0x58 -> Seq(RegField.r(32, dmaBytesOut))
        ) else Seq()

      node.regmap((baseMap ++ dmaMap): _*)
    }
  }
}

/** Mix into the SoC to optionally instantiate the OSPI capture peripheral. */
trait CanHavePeripheryOspi { this: BaseSubsystem =>
  private val portName = "ospi"
  private val pbus = locateTLBusWrapper(PBUS)

  val ospi_sensor = p(OspiKey).map { params =>
    val ospiLM = LazyModule(new OspiCapture(params, pbus.beatBytes)(p))
    ospiLM.clockNode := pbus.fixedClockNode
    pbus.coupleTo(portName) {
      TLInwardClockCrossingHelper(s"${portName}_crossing", ospiLM, ospiLM.node)(SynchronousCrossing()) :=
      TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _
    }
    ibus.fromSync := ospiLM.intnode

    // DMA master: couple the client node onto the front bus so it reaches main memory (DDR).
    // The peripheral runs in the pbus fixed-clock domain; with uniform bus frequencies a
    // synchronous crossing (a TLBuffer) is the correct boundary to the fbus clock domain, mirroring
    // the inward crossing used for the register node above.
    ospiLM.dmaNode.foreach { dn =>
      val fbus = locateTLBusWrapper(FBUS)
      fbus.coupleFrom(s"${portName}_dma") {
        _ := TLOutwardClockCrossingHelper(s"${portName}_dma_crossing", ospiLM, dn)(SynchronousCrossing())
      }
    }

    InModuleBody {
      val sensor = IO(chiselTypeOf(ospiLM.module.io)).suggestName("ospi_sensor")
      sensor <> ospiLM.module.io
      sensor
    }
  }
}

/** Config fragment: enable the OSPI capture peripheral. */
class WithOspiCapture(
  address: BigInt = 0x10080000L,
  frameBufferDepth: Int = 324 * 324 + 1
) extends Config((site, here, up) => {
  case OspiKey => Some(OspiParams(address = address, frameBufferDepth = frameBufferDepth))
})

/** Config fragment: enable the OSPI capture peripheral *with* the TileLink DMA master path.
  * Identical to [[WithOspiCapture]] but flips `enableDma`, adding the master port and the
  * DMA_* register block. The frame buffer must hold a full frame, since the DMA drains it after
  * the EOF marker is committed. */
class WithOspiCaptureDma(
  address: BigInt = 0x10080000L,
  frameBufferDepth: Int = 324 * 324 + 1
) extends Config((site, here, up) => {
  case OspiKey => Some(OspiParams(address = address, frameBufferDepth = frameBufferDepth, enableDma = true))
})
