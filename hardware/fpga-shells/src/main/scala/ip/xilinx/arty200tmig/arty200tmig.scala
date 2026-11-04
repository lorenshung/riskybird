package sifive.fpgashells.ip.xilinx.arty200tmig

import chisel3._
import chisel3.experimental.Analog
import freechips.rocketchip.util.ElaborationArtefacts
import org.chipsalliance.cde.config._

// Black Box

class Arty200TMIGIODDR(depth : BigInt) extends Bundle {
  require((depth<=0x40000000L),"Arty200TMIGIODDR supports upto 1GB depth configuraton")
  val ddr3_addr             = Output(Bits(15.W)) // RowAddress = 15
  val ddr3_ba               = Output(Bits(3.W))  // BankAddress = 3
  val ddr3_ras_n            = Output(Bool())
  val ddr3_cas_n            = Output(Bool())
  val ddr3_we_n             = Output(Bool())
  val ddr3_reset_n          = Output(Bool())
  val ddr3_ck_p             = Output(Bits(1.W))
  val ddr3_ck_n             = Output(Bits(1.W))
  val ddr3_cke              = Output(Bits(1.W))
  val ddr3_cs_n             = Output(Bits(1.W))
  val ddr3_dm               = Output(Bits(4.W))  // 32-bit bus = 4 byte lanes
  val ddr3_odt              = Output(Bits(1.W))

  val ddr3_dq               = Analog(32.W)
  val ddr3_dqs_n            = Analog(4.W)
  val ddr3_dqs_p            = Analog(4.W)
}

trait Arty200TMIGIOClocksReset extends Bundle {
  //inputs
  //"NO_BUFFER" clock source (must be driven from fabric clock outside of IP)
  val sys_clk_i             = Input(Bool())
  // NOTE: no clk_ref_i here — the 200 MHz reference is generated inside the
  // MIG (UIExtraClocks=1) and looped back blackbox-internally in the island.
  //user interface signals
  val ui_clk                = Output(Clock())
  val ui_clk_sync_rst       = Output(Bool())
  val mmcm_locked           = Output(Bool())
  val aresetn               = Input(Bool())
  //misc
  val init_calib_complete   = Output(Bool())
  val sys_rst               = Input(Bool())
}

//scalastyle:off
//turn off linter: blackbox name must match verilog module
class arty200tmig(depth : BigInt)(implicit val p:Parameters) extends BlackBox
{
  require((depth<=0x40000000L),"trenzmig supports upto 1 GB depth configuraton")

  val io = IO(new Arty200TMIGIODDR(depth) with Arty200TMIGIOClocksReset {
    // Reference clock loopback (vendor scheme): ui_addn_clk_0 is the 200 MHz
    // MMCM output (UIExtraClocks=1); it drives clk_ref_i in the island.
    val clk_ref_i             = Input(Bool())
    val ui_addn_clk_0         = Output(Clock())
    // User interface signals
    val app_sr_req            = Input(Bool())
    val app_ref_req           = Input(Bool())
    val app_zq_req            = Input(Bool())
    val app_sr_active         = Output(Bool())
    val app_ref_ack           = Output(Bool())
    val app_zq_ack            = Output(Bool())
    //axi_s
    //slave interface write address ports
    val s_axi_awid            = Input(Bits(4.W))
    val s_axi_awaddr          = Input(Bits(if(depth<=0x40000000) 30.W else 32.W))
    val s_axi_awlen           = Input(Bits(8.W))
    val s_axi_awsize          = Input(Bits(3.W))
    val s_axi_awburst         = Input(Bits(2.W))
    val s_axi_awlock          = Input(Bits(1.W))
    val s_axi_awcache         = Input(Bits(4.W))
    val s_axi_awprot          = Input(Bits(3.W))
    val s_axi_awqos           = Input(Bits(4.W))
    val s_axi_awvalid         = Input(Bool())
    val s_axi_awready         = Output(Bool())
    //slave interface write data ports
    val s_axi_wdata           = Input(Bits(64.W))
    val s_axi_wstrb           = Input(Bits(8.W))
    val s_axi_wlast           = Input(Bool())
    val s_axi_wvalid          = Input(Bool())
    val s_axi_wready          = Output(Bool())
    //slave interface write response ports
    val s_axi_bready          = Input(Bool())
    val s_axi_bid             = Output(Bits(4.W))
    val s_axi_bresp           = Output(Bits(2.W))
    val s_axi_bvalid          = Output(Bool())
    //slave interface read address ports
    val s_axi_arid            = Input(Bits(4.W))
    val s_axi_araddr          = Input(Bits(if(depth<=0x40000000) 30.W else 32.W))
    val s_axi_arlen           = Input(Bits(8.W))
    val s_axi_arsize          = Input(Bits(3.W))
    val s_axi_arburst         = Input(Bits(2.W))
    val s_axi_arlock          = Input(Bits(1.W))
    val s_axi_arcache         = Input(Bits(4.W))
    val s_axi_arprot          = Input(Bits(3.W))
    val s_axi_arqos           = Input(Bits(4.W))
    val s_axi_arvalid         = Input(Bool())
    val s_axi_arready         = Output(Bool())
    //slave interface read data ports
    val s_axi_rready          = Input(Bool())
    val s_axi_rid             = Output(Bits(4.W))
    val s_axi_rdata           = Output(Bits(64.W))
    val s_axi_rresp           = Output(Bits(2.W))
    val s_axi_rlast           = Output(Bool())
    val s_axi_rvalid          = Output(Bool())
    //misc
    val device_temp           = Output(Bits(12.W))
  })


  // Byte-for-byte Trenz's own MIG project for this module --
  // arty_files/te0712-rd/board_files/TE0712_200_1I/2.0/mig.prj -- except for
  // <ModuleName>, <SystemClock> (No Buffer, because the shell feeds sys_clk_i
  // from the harness PLL rather than the pins) and <C0_S_AXI_DATA_WIDTH> (64,
  // to match the SoC's memory port). All 77 pin-map lines and every timing
  // parameter are Trenz's.
  //
  // <TargetFPGA> MUST track xilinx/arty_a7_200/tcl/board.tcl. MIG bakes the
  // speed grade into the generated PHY as the FPGA_SPEED_GRADE parameter, so a
  // mismatch is a functional error in the controller and not only a
  // timing-signoff one.
  val migprj = """{<?xml version="1.0" encoding="UTF-8" standalone="no" ?>
<Project NoOfControllers="1">

<!-- IMPORTANT: This is an internal file that has been generated by the MIG software. Any direct editing or changes made to this file may result in unpredictable behavior or data corruption. It is strongly advised that users do not edit the contents of this file. Re-run the MIG GUI with the required settings if any of the options provided below need to be altered. -->

  <ModuleName>arty200tmig</ModuleName>

  <dci_inouts_inputs>1</dci_inouts_inputs>

  <dci_inputs>1</dci_inputs>

  <Debug_En>OFF</Debug_En>

  <DataDepth_En>1024</DataDepth_En>

  <LowPower_En>ON</LowPower_En>

  <XADC_En>Enabled</XADC_En>

  <TargetFPGA>xc7a200t-fbg484/-1</TargetFPGA>

  <Version>4.2</Version>

  <SystemClock>No Buffer</SystemClock>

  <ReferenceClock>No Buffer</ReferenceClock>

  <SysResetPolarity>ACTIVE LOW</SysResetPolarity>

  <BankSelectionFlag>FALSE</BankSelectionFlag>

  <InternalVref>0</InternalVref>

  <dci_hr_inouts_inputs>50 Ohms</dci_hr_inouts_inputs>

  <dci_cascade>0</dci_cascade>

  <Controller number="0">
    <MemoryDevice>DDR3_SDRAM/Components/MT41J256m16XX-125</MemoryDevice>
    <TimePeriod>2500</TimePeriod>
    <VccAuxIO>1.8V</VccAuxIO>
    <PHYRatio>4:1</PHYRatio>
    <InputClkFreq>50</InputClkFreq>
    <UIExtraClocks>1</UIExtraClocks>
    <MMCM_VCO>800</MMCM_VCO>
    <MMCMClkOut0> 4.000</MMCMClkOut0>
    <MMCMClkOut1>8</MMCMClkOut1>
    <MMCMClkOut2>16</MMCMClkOut2>
    <MMCMClkOut3>1</MMCMClkOut3>
    <MMCMClkOut4>1</MMCMClkOut4>
    <DataWidth>32</DataWidth>
    <DeepMemory>1</DeepMemory>
    <DataMask>1</DataMask>
    <ECC>Disabled</ECC>
    <Ordering>Normal</Ordering>
    <BankMachineCnt>4</BankMachineCnt>
    <CustomPart>FALSE</CustomPart>
    <NewPartName/>
    <RowAddress>15</RowAddress>
    <ColAddress>10</ColAddress>
    <BankAddress>3</BankAddress>
    <MemoryVoltage>1.5V</MemoryVoltage>
    <C0_MEM_SIZE>1073741824</C0_MEM_SIZE>
    <UserMemoryAddressMap>BANK_ROW_COLUMN</UserMemoryAddressMap>
    <PinSelection>
      <Pin IN_TERM="" IOSTANDARD="" PADName="J1" SLEW="" VCCAUX_IO="" name="ddr3_addr[0]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="L4" SLEW="" VCCAUX_IO="" name="ddr3_addr[10]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="P5" SLEW="" VCCAUX_IO="" name="ddr3_addr[11]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="K2" SLEW="" VCCAUX_IO="" name="ddr3_addr[12]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="M1" SLEW="" VCCAUX_IO="" name="ddr3_addr[13]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="M5" SLEW="" VCCAUX_IO="" name="ddr3_addr[14]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="P6" SLEW="" VCCAUX_IO="" name="ddr3_addr[1]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="N5" SLEW="" VCCAUX_IO="" name="ddr3_addr[2]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="N3" SLEW="" VCCAUX_IO="" name="ddr3_addr[3]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="G1" SLEW="" VCCAUX_IO="" name="ddr3_addr[4]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="M3" SLEW="" VCCAUX_IO="" name="ddr3_addr[5]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="N2" SLEW="" VCCAUX_IO="" name="ddr3_addr[6]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="J5" SLEW="" VCCAUX_IO="" name="ddr3_addr[7]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="L1" SLEW="" VCCAUX_IO="" name="ddr3_addr[8]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="P2" SLEW="" VCCAUX_IO="" name="ddr3_addr[9]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="P4" SLEW="" VCCAUX_IO="" name="ddr3_ba[0]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="H5" SLEW="" VCCAUX_IO="" name="ddr3_ba[1]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="H2" SLEW="" VCCAUX_IO="" name="ddr3_ba[2]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="M2" SLEW="" VCCAUX_IO="" name="ddr3_cas_n"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="P1" SLEW="" VCCAUX_IO="" name="ddr3_ck_n[0]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="R1" SLEW="" VCCAUX_IO="" name="ddr3_ck_p[0]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="L3" SLEW="" VCCAUX_IO="" name="ddr3_cke[0]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="K1" SLEW="" VCCAUX_IO="" name="ddr3_cs_n[0]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="W2" SLEW="" VCCAUX_IO="" name="ddr3_dm[0]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="Y7" SLEW="" VCCAUX_IO="" name="ddr3_dm[1]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="V4" SLEW="" VCCAUX_IO="" name="ddr3_dm[2]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="V5" SLEW="" VCCAUX_IO="" name="ddr3_dm[3]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="T1" SLEW="" VCCAUX_IO="" name="ddr3_dq[0]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="AB7" SLEW="" VCCAUX_IO="" name="ddr3_dq[10]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="AA8" SLEW="" VCCAUX_IO="" name="ddr3_dq[11]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="AB8" SLEW="" VCCAUX_IO="" name="ddr3_dq[12]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="AB6" SLEW="" VCCAUX_IO="" name="ddr3_dq[13]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="Y8" SLEW="" VCCAUX_IO="" name="ddr3_dq[14]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="Y9" SLEW="" VCCAUX_IO="" name="ddr3_dq[15]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="AB1" SLEW="" VCCAUX_IO="" name="ddr3_dq[16]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="AB5" SLEW="" VCCAUX_IO="" name="ddr3_dq[17]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="AB3" SLEW="" VCCAUX_IO="" name="ddr3_dq[18]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="AA1" SLEW="" VCCAUX_IO="" name="ddr3_dq[19]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="U3" SLEW="" VCCAUX_IO="" name="ddr3_dq[1]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="Y4" SLEW="" VCCAUX_IO="" name="ddr3_dq[20]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="AA5" SLEW="" VCCAUX_IO="" name="ddr3_dq[21]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="AB2" SLEW="" VCCAUX_IO="" name="ddr3_dq[22]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="W4" SLEW="" VCCAUX_IO="" name="ddr3_dq[23]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="T4" SLEW="" VCCAUX_IO="" name="ddr3_dq[24]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="U6" SLEW="" VCCAUX_IO="" name="ddr3_dq[25]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="T6" SLEW="" VCCAUX_IO="" name="ddr3_dq[26]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="AA6" SLEW="" VCCAUX_IO="" name="ddr3_dq[27]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="Y6" SLEW="" VCCAUX_IO="" name="ddr3_dq[28]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="T5" SLEW="" VCCAUX_IO="" name="ddr3_dq[29]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="U2" SLEW="" VCCAUX_IO="" name="ddr3_dq[2]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="U5" SLEW="" VCCAUX_IO="" name="ddr3_dq[30]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="R6" SLEW="" VCCAUX_IO="" name="ddr3_dq[31]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="U1" SLEW="" VCCAUX_IO="" name="ddr3_dq[3]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="Y2" SLEW="" VCCAUX_IO="" name="ddr3_dq[4]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="W1" SLEW="" VCCAUX_IO="" name="ddr3_dq[5]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="Y1" SLEW="" VCCAUX_IO="" name="ddr3_dq[6]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="V2" SLEW="" VCCAUX_IO="" name="ddr3_dq[7]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="V7" SLEW="" VCCAUX_IO="" name="ddr3_dq[8]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="W9" SLEW="" VCCAUX_IO="" name="ddr3_dq[9]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="R2" SLEW="" VCCAUX_IO="" name="ddr3_dqs_n[0]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="V8" SLEW="" VCCAUX_IO="" name="ddr3_dqs_n[1]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="AA3" SLEW="" VCCAUX_IO="" name="ddr3_dqs_n[2]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="W5" SLEW="" VCCAUX_IO="" name="ddr3_dqs_n[3]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="R3" SLEW="" VCCAUX_IO="" name="ddr3_dqs_p[0]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="V9" SLEW="" VCCAUX_IO="" name="ddr3_dqs_p[1]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="Y3" SLEW="" VCCAUX_IO="" name="ddr3_dqs_p[2]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="W6" SLEW="" VCCAUX_IO="" name="ddr3_dqs_p[3]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="K3" SLEW="" VCCAUX_IO="" name="ddr3_odt[0]"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="M6" SLEW="" VCCAUX_IO="" name="ddr3_ras_n"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="H3" SLEW="" VCCAUX_IO="" name="ddr3_reset_n"/>
      <Pin IN_TERM="" IOSTANDARD="" PADName="J2" SLEW="" VCCAUX_IO="" name="ddr3_we_n"/>
    </PinSelection>
    <System_Clock>
      <Pin Bank="35" PADName="H4/G4(CC_P/N)" name="sys_clk_p/n"/>
    </System_Clock>
    <System_Control>
      <Pin Bank="Select Bank" PADName="No connect" name="sys_rst"/>
      <Pin Bank="Select Bank" PADName="No connect" name="init_calib_complete"/>
      <Pin Bank="Select Bank" PADName="No connect" name="tg_compare_error"/>
    </System_Control>
    <TimingParameters>
      <Parameters tcke="5" tfaw="40" tras="35" trcd="13.75" trefi="7.8" trfc="260" trp="13.75" trrd="7.5" trtp="7.5" twtr="7.5"/>
    </TimingParameters>
    <mrBurstLength name="Burst Length">8 - Fixed</mrBurstLength>
    <mrBurstType name="Read Burst Type and Length">Sequential</mrBurstType>
    <mrCasLatency name="CAS Latency">6</mrCasLatency>
    <mrMode name="Mode">Normal</mrMode>
    <mrDllReset name="DLL Reset">No</mrDllReset>
    <mrPdMode name="DLL control for precharge PD">Slow Exit</mrPdMode>
    <emrDllEnable name="DLL Enable">Enable</emrDllEnable>
    <emrOutputDriveStrength name="Output Driver Impedance Control">RZQ/7</emrOutputDriveStrength>
    <emrMirrorSelection name="Address Mirroring">Disable</emrMirrorSelection>
    <emrCSSelection name="Controller Chip Select Pin">Enable</emrCSSelection>
    <emrRTT name="RTT (nominal) - On Die Termination (ODT)">RZQ/4</emrRTT>
    <emrPosted name="Additive Latency (AL)">0</emrPosted>
    <emrOCD name="Write Leveling Enable">Disabled</emrOCD>
    <emrDQS name="TDQS enable">Enabled</emrDQS>
    <emrRDQS name="Qoff">Output Buffer Enabled</emrRDQS>
    <mr2PartialArraySelfRefresh name="Partial-Array Self Refresh">Full Array</mr2PartialArraySelfRefresh>
    <mr2CasWriteLatency name="CAS write latency">5</mr2CasWriteLatency>
    <mr2AutoSelfRefresh name="Auto Self Refresh">Enabled</mr2AutoSelfRefresh>
    <mr2SelfRefreshTempRange name="High Temparature Self Refresh Rate">Normal</mr2SelfRefreshTempRange>
    <mr2RTTWR name="RTT_WR - Dynamic On Die Termination (ODT)">Dynamic ODT off</mr2RTTWR>
    <PortInterface>AXI</PortInterface>
    <AXIParameters>
      <C0_C_RD_WR_ARB_ALGORITHM>RD_PRI_REG</C0_C_RD_WR_ARB_ALGORITHM>
      <C0_S_AXI_ADDR_WIDTH>30</C0_S_AXI_ADDR_WIDTH>
      <C0_S_AXI_DATA_WIDTH>64</C0_S_AXI_DATA_WIDTH>
      <C0_S_AXI_ID_WIDTH>4</C0_S_AXI_ID_WIDTH>
      <C0_S_AXI_SUPPORTS_NARROW_BURST>0</C0_S_AXI_SUPPORTS_NARROW_BURST>
    </AXIParameters>
  </Controller>
</Project> } """




  val migprjname = """{/arty200tmig.prj}"""
  val modulename = """arty200tmig"""

  ElaborationArtefacts.add(
    modulename++".vivado.tcl",
    """set migprj """++migprj++"""
   set migprjfile """++migprjname++"""
   set migprjfilepath $ipdir$migprjfile
   set fp [open $migprjfilepath w+]
   puts $fp $migprj
   close $fp
   create_ip -vendor xilinx.com -library ip -name mig_7series -module_name """ ++ modulename ++ """ -dir $ipdir -force
   set_property CONFIG.XML_INPUT_FILE $migprjfilepath [get_ips """ ++ modulename ++ """] """
  )


}

/*
   Copyright 2016 SiFive, Inc.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
