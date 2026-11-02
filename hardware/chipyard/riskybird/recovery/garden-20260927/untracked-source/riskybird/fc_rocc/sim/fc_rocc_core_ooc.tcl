set part xc7a200tfbg484-2
file mkdir ooc_out
read_verilog -sv fc_rocc_core.sv
synth_design -mode out_of_context -top fc_rocc_core -part $part -flatten_hierarchy rebuilt
opt_design -quiet
report_utilization -file ooc_out/fc_rocc_core_util.txt
create_clock -name clk -period 22.7 [get_ports clk]
report_timing_summary -max_paths 1 -file ooc_out/fc_rocc_core_timing.txt
puts "==== FC_ROCC_CORE OOC ===="
report_utilization
