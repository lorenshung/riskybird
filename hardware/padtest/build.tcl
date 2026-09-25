# ---------------------------------------------------------------------------
# build.tcl - self-contained non-project Vivado batch flow for padtest_top
#   /ecad/tools/xilinx/Vivado/2023.1/bin/vivado -mode batch -nojournal -nolog \
#       -source hardware/padtest/build.tcl
# ---------------------------------------------------------------------------
# This script's own directory, so the flow runs from any checkout.
set d [file dirname [file normalize [info script]]]
set part xc7a200tfbg484-2

file mkdir $d/out
set_part $part

read_verilog $d/padtest_top.v
read_xdc     $d/padtest.xdc

synth_design -top padtest_top -part $part
write_checkpoint -force $d/out/post_synth.dcp
report_utilization -file $d/out/post_synth_util.rpt

opt_design
place_design
phys_opt_design
route_design

write_checkpoint -force $d/out/post_route.dcp
report_utilization       -file $d/out/post_route_util.rpt
report_timing_summary    -file $d/out/post_route_timing.rpt
report_drc               -file $d/out/post_route_drc.rpt
report_io                -file $d/out/post_route_io.rpt

write_bitstream -force $d/out/padtest.bit

puts "PADTEST_BUILD_OK bitstream=$d/out/padtest.bit"
