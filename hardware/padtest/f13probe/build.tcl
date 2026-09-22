# ---------------------------------------------------------------------------
# build.tcl - self-contained non-project Vivado batch flow for f13probe_top
#   /ecad/tools/xilinx/Vivado/2023.1/bin/vivado -mode batch -nojournal -nolog \
#       -source /scratch2/loren/riskybird-dev/hardware/padtest/f13probe/build.tcl
# Outputs are named f13probe.* in this subdirectory; the earlier padtest.bit is
# in the parent directory and is never touched.
# ---------------------------------------------------------------------------
set d /scratch2/loren/riskybird-dev/hardware/padtest/f13probe
set part xc7a200tfbg484-2

file mkdir $d/out
set_part $part

read_verilog $d/f13probe_top.v
read_xdc     $d/f13probe.xdc

synth_design -top f13probe_top -part $part
write_checkpoint -force $d/out/f13probe_post_synth.dcp
report_utilization -file $d/out/f13probe_post_synth_util.rpt

opt_design
place_design
phys_opt_design
route_design

write_checkpoint -force $d/out/f13probe_post_route.dcp
report_utilization    -file $d/out/f13probe_post_route_util.rpt
report_timing_summary -file $d/out/f13probe_post_route_timing.rpt
report_drc            -file $d/out/f13probe_post_route_drc.rpt
report_io             -file $d/out/f13probe_post_route_io.rpt

write_bitstream -force $d/out/f13probe.bit

puts "F13PROBE_BUILD_OK bitstream=$d/out/f13probe.bit"
