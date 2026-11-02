package sifive.fpgashells.ip.xilinx.ku040mig

import chisel3._
import chisel3.experimental.Analog
import freechips.rocketchip.util.ElaborationArtefacts
import org.chipsalliance.cde.config._

// IP VLNV: xilinx.com:ip:ddr4:2.2
// Black Box
//
// One DDR4 controller for the custom XCKU040-SFVA784-1-C board (Trenz TE0841
// rev 02). The board carries two independent x16 DDR4 components, one in HP
// bank 44 and one in HP bank 46, each a Samsung K4A8G165WB-BIRC (8 Gb x16 =
// 1 GiB). They are separate memory interfaces, not one wide one, so this single
// IP is instantiated twice -- once per bank -- and the two 1 GiB regions are
// joined into a contiguous 2 GiB by a TLXbar in the harness. Package pins are
// assigned per instance by DDRKU040PlacedOverlay; everything else (IOSTANDARD,
// OUTPUT_IMPEDANCE, DRIVE, SLEW, the DQ/DQS byte-lane timing) comes from this
// IP's own scoped XDC, which applies to both instances.
//
// Memory timings come from the board vendor's custom-parts CSV rather than a
// Vivado catalog part, because the Samsung device is not in the catalog. The
// CSV, the 1111 ps tCK (DDR4-1866) and the 4999 ps input clock period are all
// taken from the TE0841 board file's ddr4_sdram_preset.

class KU040MIGIODDR extends Bundle {
  val c0_ddr4_adr           = Output(Bits(17.W))
  val c0_ddr4_bg            = Output(Bits(1.W))
  val c0_ddr4_ba            = Output(Bits(2.W))
  val c0_ddr4_reset_n       = Output(Bool())
  val c0_ddr4_act_n         = Output(Bool())
  val c0_ddr4_ck_c          = Output(Bits(1.W))
  val c0_ddr4_ck_t          = Output(Bits(1.W))
  val c0_ddr4_cke           = Output(Bits(1.W))
  val c0_ddr4_cs_n          = Output(Bits(1.W))
  val c0_ddr4_odt           = Output(Bits(1.W))

  val c0_ddr4_dq            = Analog(16.W)
  val c0_ddr4_dqs_c         = Analog(2.W)
  val c0_ddr4_dqs_t         = Analog(2.W)
  val c0_ddr4_dm_dbi_n      = Analog(2.W)
}

//reused directly in io bundle for sifive.fpgashells.devices.xilinx.xilinxku040mig
trait KU040MIGIOClocksReset extends Bundle {
  //inputs
  //"NO_BUFFER" clock source (must be connected to IBUF outside of IP)
  val c0_sys_clk_i              = Input(Bool())
  //user interface signals
  val c0_ddr4_ui_clk            = Output(Clock())
  val c0_ddr4_ui_clk_sync_rst   = Output(Bool())
  val c0_ddr4_aresetn           = Input(Bool())
  //misc
  val c0_init_calib_complete    = Output(Bool())
  val sys_rst                   = Input(Bool())
}

//scalastyle:off
//turn off linter: blackbox name must match verilog module
class ku040mig(implicit val p: Parameters) extends BlackBox
{
  val io = IO(new KU040MIGIODDR with KU040MIGIOClocksReset {
    //slave interface write address ports
    val c0_ddr4_s_axi_awid            = Input(Bits(4.W))
    val c0_ddr4_s_axi_awaddr          = Input(Bits(30.W))
    val c0_ddr4_s_axi_awlen           = Input(Bits(8.W))
    val c0_ddr4_s_axi_awsize          = Input(Bits(3.W))
    val c0_ddr4_s_axi_awburst         = Input(Bits(2.W))
    val c0_ddr4_s_axi_awlock          = Input(Bits(1.W))
    val c0_ddr4_s_axi_awcache         = Input(Bits(4.W))
    val c0_ddr4_s_axi_awprot          = Input(Bits(3.W))
    val c0_ddr4_s_axi_awqos           = Input(Bits(4.W))
    val c0_ddr4_s_axi_awvalid         = Input(Bool())
    val c0_ddr4_s_axi_awready         = Output(Bool())
    //slave interface write data ports
    val c0_ddr4_s_axi_wdata           = Input(Bits(64.W))
    val c0_ddr4_s_axi_wstrb           = Input(Bits(8.W))
    val c0_ddr4_s_axi_wlast           = Input(Bool())
    val c0_ddr4_s_axi_wvalid          = Input(Bool())
    val c0_ddr4_s_axi_wready          = Output(Bool())
    //slave interface write response ports
    val c0_ddr4_s_axi_bready          = Input(Bool())
    val c0_ddr4_s_axi_bid             = Output(Bits(4.W))
    val c0_ddr4_s_axi_bresp           = Output(Bits(2.W))
    val c0_ddr4_s_axi_bvalid          = Output(Bool())
    //slave interface read address ports
    val c0_ddr4_s_axi_arid            = Input(Bits(4.W))
    val c0_ddr4_s_axi_araddr          = Input(Bits(30.W))
    val c0_ddr4_s_axi_arlen           = Input(Bits(8.W))
    val c0_ddr4_s_axi_arsize          = Input(Bits(3.W))
    val c0_ddr4_s_axi_arburst         = Input(Bits(2.W))
    val c0_ddr4_s_axi_arlock          = Input(Bits(1.W))
    val c0_ddr4_s_axi_arcache         = Input(Bits(4.W))
    val c0_ddr4_s_axi_arprot          = Input(Bits(3.W))
    val c0_ddr4_s_axi_arqos           = Input(Bits(4.W))
    val c0_ddr4_s_axi_arvalid         = Input(Bool())
    val c0_ddr4_s_axi_arready         = Output(Bool())
    //slave interface read data ports
    val c0_ddr4_s_axi_rready          = Input(Bool())
    val c0_ddr4_s_axi_rid             = Output(Bits(4.W))
    val c0_ddr4_s_axi_rdata           = Output(Bits(64.W))
    val c0_ddr4_s_axi_rresp           = Output(Bits(2.W))
    val c0_ddr4_s_axi_rlast           = Output(Bool())
    val c0_ddr4_s_axi_rvalid          = Output(Bool())
  })

  // The custom-parts CSV is copied into $ipdir first: Vivado rewrites an
  // absolute DDR4_CustomParts path into one relative to the .xci, and a path
  // that reaches back out of the build tree into fpga-shells would break as
  // soon as the build directory moves.
  ElaborationArtefacts.add(
    "ku040mig.vivado.tcl",
    """
      file copy -force [file join $boarddir mig K4A8G165WB-BIRC.csv] $ipdir
      create_ip -vendor xilinx.com -library ip -version 2.2 -name ddr4 -module_name ku040mig -dir $ipdir -force
      set_property -dict [list \
      CONFIG.C0.DDR4_isCustom                     {true} \
      CONFIG.C0.DDR4_CustomParts                  [file join $ipdir K4A8G165WB-BIRC.csv] \
      CONFIG.C0.DDR4_MemoryPart                   {K4A8G165WB-BIRC_DDR4-1866} \
      CONFIG.C0.DDR4_MemoryType                   {Components} \
      CONFIG.C0.DDR4_MemoryVoltage                {1.2V} \
      CONFIG.C0.DDR4_DataWidth                    {16} \
      CONFIG.C0.DDR4_TimePeriod                   {1111} \
      CONFIG.C0.DDR4_InputClockPeriod             {4999} \
      CONFIG.C0.DDR4_AxiSelection                 {true} \
      CONFIG.C0.DDR4_AxiDataWidth                 {64} \
      CONFIG.C0.DDR4_AxiAddressWidth              {30} \
      CONFIG.C0.DDR4_AxiIDWidth                   {4} \
      CONFIG.C0.DDR4_AxiNarrowBurst               {false} \
      CONFIG.C0.DDR4_DataMask                     {DM_NO_DBI} \
      CONFIG.C0.DDR4_Ecc                          {false} \
      CONFIG.C0.DDR4_Mem_Add_Map                  {ROW_COLUMN_BANK} \
      CONFIG.C0.DDR4_Ordering                     {Normal} \
      CONFIG.C0.DDR4_PhyClockRatio                {4:1} \
      CONFIG.C0.BANK_GROUP_WIDTH                  {1} \
      CONFIG.C0.ControllerType                    {DDR4_SDRAM} \
      CONFIG.C0_CLOCK_BOARD_INTERFACE             {Custom} \
      CONFIG.C0_DDR4_BOARD_INTERFACE              {Custom} \
      CONFIG.RESET_BOARD_INTERFACE                {Custom} \
      CONFIG.Debug_Signal                         {Disable} \
      CONFIG.DIFF_TERM_SYSCLK                     {false} \
      CONFIG.Enable_SysPorts                      {true} \
      CONFIG.No_Controller                        {1} \
      CONFIG.Phy_Only                             {Complete_Memory_Controller} \
      CONFIG.System_Clock                         {No_Buffer} \
      ] [get_ips ku040mig]"""
  )
}
//scalastyle:on
