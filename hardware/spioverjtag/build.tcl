# Build an openFPGALoader spiOverJtag bridge bitstream for xcku040-sfva784.
#
# Non-project batch flow, standing in for upstream's spiOverJtag/build.py --
# that script needs the `edalize` Python package, which this flow avoids.
#
#   vivado -nojournal -nolog -mode batch -source build.tcl \
#          -tclargs <openFPGALoader>/spiOverJtag <output-dir>
#
# The two Verilog sources stay in the upstream checkout rather than being
# vendored here; only the constraints and this script are RiskyBird's.

set src [lindex $argv 0]
set out [lindex $argv 1]
set part xcku040-sfva784-1-c
set here [file dirname [file normalize [info script]]]

file mkdir $out

read_verilog [list $src/xilinx_spiOverJtag.v $src/spiOverJtag_core.v \
                  $here/te0841_spiOverJtag.v]

# `xilinxultrascale` selects the STARTUPE3 path and removes every module port.
# `secondaryflash` is intentionally NOT defined: the TE0841 has one flash, and
# defining it would add bank-65 user I/O that this package must not drive.
synth_design -top te0841_spiOverJtag -part $part -verilog_define xilinxultrascale=1

read_xdc $here/constr_xcku040_sfva784.xdc

opt_design
place_design
route_design

report_utilization -file $out/utilization.rpt
write_bitstream -force $out/spiOverJtag.bit
puts "SPIOVERJTAG_BUILD_OK"
