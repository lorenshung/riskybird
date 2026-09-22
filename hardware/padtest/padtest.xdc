# ---------------------------------------------------------------------------
# padtest.xdc - RiskyBird TE0712 (xc7a200tfbg484-2) minimal motor pad test
#
# Pin assignments are copied verbatim from the working full design's generated
# shell constraints:
#   backends/chipyard/fpga/generated-src/
#     chipyard.fpga.arty200t.Arty200THarness.RocketArty200TDroneFullDDRConfig/
#     ...RocketArty200TDroneFullDDRConfig.shell.xdc
# ---------------------------------------------------------------------------

# ---- 50 MHz differential system clock (H4/G4, DIFF_SSTL15) ----------------
set_property PACKAGE_PIN {H4} [get_ports {sys_clock_p}]
set_property PACKAGE_PIN {G4} [get_ports {sys_clock_n}]
set_property IOSTANDARD {DIFF_SSTL15} [get_ports {sys_clock_p}]
set_property IOSTANDARD {DIFF_SSTL15} [get_ports {sys_clock_n}]

# 20.0 ns period = 50 MHz.  Matches the full design's generated .shell.sdc
# ("create_clock -name sys_clock -period 20.0 [get_ports {sys_clock_p}]") and
# Arty200TShell.scala's ClockSourceNode(freqMHz = 50).
create_clock -name sys_clock -period 20.0 [get_ports {sys_clock_p}]
set_input_jitter sys_clock 0.5

# ---- four brushed-motor low-side FET gates --------------------------------
# Active high into a SI2302 N-FET gate through 47R, 10k gate pulldown.
set_property PACKAGE_PIN {E22} [get_ports {motor1}]
set_property IOSTANDARD {LVCMOS33} [get_ports {motor1}]
set_property PULLDOWN {TRUE} [get_ports {motor1}]

set_property PACKAGE_PIN {B20} [get_ports {motor2}]
set_property IOSTANDARD {LVCMOS33} [get_ports {motor2}]
set_property PULLDOWN {TRUE} [get_ports {motor2}]

set_property PACKAGE_PIN {R17} [get_ports {motor3}]
set_property IOSTANDARD {LVCMOS33} [get_ports {motor3}]
set_property PULLDOWN {TRUE} [get_ports {motor3}]

set_property PACKAGE_PIN {F13} [get_ports {motor4}]
set_property IOSTANDARD {LVCMOS33} [get_ports {motor4}]
set_property PULLDOWN {TRUE} [get_ports {motor4}]

# ---- configuration / safety ------------------------------------------------
# Every pin not named above is pulled down, so nothing else on the carrier can
# float high out of configuration either.
set_property BITSTREAM.CONFIG.UNUSEDPIN PULLDOWN [current_design]
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
