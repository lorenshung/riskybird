# OOC (out-of-context) synthesis of the FC RoCC accelerator ALONE.
# Same part / speed grade as the deployed combined SoC (xc7a200tfbg484-2, -2).
# Light: synth-only, no P&R, no bitstream -- polite to the deployable queue.
#
#   vivado -nojournal -mode batch -source fc_accel_ooc.tcl
#
set part   xc7a200tfbg484-2
set top    fc_accel
set outdir [pwd]/ooc_out
file mkdir $outdir

read_verilog -sv fc_accel.sv

# OOC: no top-level I/O buffers, characterise the module's own logic only.
synth_design -mode out_of_context -top $top -part $part -flatten_hierarchy rebuilt

# a light opt so the numbers reflect what the SoC integrator would see
opt_design -quiet

report_utilization           -file $outdir/fc_accel_util.txt
report_utilization -hierarchical -file $outdir/fc_accel_util_hier.txt

# rough timing feel at the SoC clocks we care about (35 / 40 / 44 MHz targets)
create_clock -name clk -period 22.7 [get_ports clk]   ;# ~44 MHz
report_timing_summary -max_paths 3 -file $outdir/fc_accel_timing_44mhz.txt

puts "==== FC_ACCEL OOC UTILISATION ===="
report_utilization
puts "==== FC_ACCEL WNS @44MHz ===="
report_timing_summary -max_paths 1 -no_detailed_paths
