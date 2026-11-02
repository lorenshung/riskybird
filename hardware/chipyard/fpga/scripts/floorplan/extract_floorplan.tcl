# extract_floorplan.tcl -- Vivado batch extractor for the FPGA PAR floorplan tool.
#
# Opens a POST-ROUTE checkpoint and emits, into an output directory, everything
# the plotter needs -- with the block partition and per-block utilization
# re-derived from the checkpoint (nothing hardcoded):
#   meta.txt                 PART / CONFIG / DCP / timestamp
#   blocks_util.csv          block,luts,ff,bram_tiles,dsp,cells  (+ DEVICE / DEVICE_AVAIL rows)
#   cells_<block>.txt        one placed-primitive site (LOC) per line, per block
#   crit_src.txt/crit_dst.txt  optional -1/-2 critical-path endpoints (Gemmini spad), if present
#
# The five blocks form a MUTUALLY-EXCLUSIVE, EXHAUSTIVE partition of every placed
# primitive. NOTE the MIG filter uses the clean "mig/*" prefix -- do NOT use
# "*ddr*", which false-matches every "addr" register across the whole design.
#
# Usage:
#   vivado -mode batch -source extract_floorplan.tcl -tclargs <post_route.dcp> <outdir>

if {[llength $argv] < 2} {
    puts "ERROR: usage: vivado -mode batch -source extract_floorplan.tcl -tclargs <post_route.dcp> <outdir>"
    exit 1
}
set dcp    [lindex $argv 0]
set outdir [lindex $argv 1]
if {![file exists $dcp]} { puts "ERROR: DCP not found: $dcp"; exit 1 }
file mkdir $outdir

# ---- block partition: id -> get_cells filter (over IS_PRIMITIVE && LOC != "") ----
# Anchored on the Rocket tile hierarchy; acc_adders (Gemmini accumulator adder
# tree) and fpuOpt (core FPU) are hoisted to the top level by flatten, so they
# are matched explicitly. Keep these in sync with plot_floorplan.py presentation.
set B(gemmini) {(NAME =~ "*rockettile/gemmini/*" || NAME =~ "acc_adders/*")}
set B(saturn)  {NAME =~ "*rockettile/vector_unit/*"}
set B(rocket)  {((NAME =~ "*rockettile/*" && NAME !~ "*rockettile/gemmini/*" && NAME !~ "*rockettile/vector_unit/*") || NAME =~ "fpuOpt/*")}
set B(mig)     {NAME =~ "mig/*"}
set B(uncore)  {(NAME !~ "*rockettile/*" && NAME !~ "fpuOpt/*" && NAME !~ "acc_adders/*" && NAME !~ "mig/*")}
set order {gemmini saturn rocket mig uncore}

puts "==== opening checkpoint: $dcp ===="
open_checkpoint $dcp
set part [get_property PART [current_design]]

# derive config name from the DCP path (.../<Harness>.<Config>/obj/post_route.dcp)
set cfgdir [file tail [file dirname [file dirname $dcp]]]
set cfg $cfgdir
regsub {^.*Harness\.} $cfg "" cfg

set fh [open "$outdir/meta.txt" w]
puts $fh "PART $part"
puts $fh "CONFIG $cfg"
puts $fh "DCP $dcp"
puts $fh "DATE [clock format [clock seconds]]"
close $fh
puts "part=$part config=$cfg"

# ---- helper: first regexp capture from a file (or default) ----
proc grep1 {file re {dflt 0}} {
    if {![file exists $file]} { return $dflt }
    set f [open $file r]; set data [read $f]; close $f
    foreach line [split $data "\n"] {
        if {[regexp $re $line -> v]} { return $v }
    }
    return $dflt
}

set csv [open "$outdir/blocks_util.csv" w]
puts $csv "block,luts,ff,bram_tiles,dsp,cells"

foreach b $order {
    set cells [get_cells -hierarchical -filter "IS_PRIMITIVE && LOC != \"\" && $B($b)"]
    set n [llength $cells]
    puts "---- block $b : $n placed primitives ----"

    # placement dump (LOC only; site type + X/Y are parsed from it by the plotter)
    set ph [open "$outdir/cells_$b.txt" w]
    if {$n > 0} { puts $ph [join [get_property LOC $cells] "\n"] }
    close $ph

    # per-block utilization -> parse LUT/FF/BRAM/DSP
    set luts 0; set ff 0; set bram 0; set dsp 0
    if {$n > 0} {
        set rpt "$outdir/util_$b.rpt"
        report_utilization -cells $cells -file $rpt
        set luts [grep1 $rpt {\|\s*Slice LUTs\s*\|\s*(\d+)\s*\|}]
        set ff   [grep1 $rpt {\|\s*Register as Flip Flop\s*\|\s*(\d+)\s*\|}]
        set bram [grep1 $rpt {\|\s*Block RAM Tile\s*\|\s*(\d+)\s*\|}]
        set dsp  [grep1 $rpt {\|\s*DSPs\s*\|\s*(\d+)\s*\|}]
    }
    puts $csv "$b,$luts,$ff,$bram,$dsp,$n"
}

# ---- whole-device totals + capacities (for occupancy %) ----
set full "$outdir/util_full.rpt"
report_utilization -file $full
set dl [grep1 $full {\|\s*Slice LUTs\s*\|\s*(\d+)\s*\|}]
set df [grep1 $full {\|\s*Register as Flip Flop\s*\|\s*(\d+)\s*\|}]
set db [grep1 $full {\|\s*Block RAM Tile\s*\|\s*(\d+)\s*\|}]
set dd [grep1 $full {\|\s*DSPs\s*\|\s*(\d+)\s*\|}]
# available (device capacity) = 4th integer column on the same rows
set al [grep1 $full {\|\s*Slice LUTs\s*\|\s*\d+\s*\|\s*\d+\s*\|\s*\d+\s*\|\s*(\d+)\s*\|}]
set af [grep1 $full {\|\s*Register as Flip Flop\s*\|\s*\d+\s*\|\s*\d+\s*\|\s*\d+\s*\|\s*(\d+)\s*\|}]
set ab [grep1 $full {\|\s*Block RAM Tile\s*\|\s*\d+\s*\|\s*\d+\s*\|\s*\d+\s*\|\s*(\d+)\s*\|}]
set ad [grep1 $full {\|\s*DSPs\s*\|\s*\d+\s*\|\s*\d+\s*\|\s*\d+\s*\|\s*(\d+)\s*\|}]
puts $csv "DEVICE,$dl,$df,$db,$dd,0"
puts $csv "DEVICE_AVAIL,$al,$af,$ab,$ad,0"
close $csv

# ---- optional critical-path endpoints (Gemmini scratchpad); graceful if absent ----
proc dump_locs {file cells} {
    set fh [open $file w]
    if {[llength $cells] > 0} { puts $fh [join [get_property LOC $cells] "\n"] }
    close $fh
}
dump_locs "$outdir/crit_src.txt" \
    [get_cells -hierarchical -filter {IS_PRIMITIVE && LOC != "" && NAME =~ "*rockettile/gemmini/spad/reader/beatPacker/bytesSent*"}]
dump_locs "$outdir/crit_dst.txt" \
    [get_cells -hierarchical -filter {IS_PRIMITIVE && LOC != "" && NAME =~ "*rockettile/gemmini/spad/acc_mems*" && REF_NAME =~ "RAMB*"}]

puts "==== EXTRACT DONE -> $outdir ===="
