# RiskyBird synthesis-only area characterization.
#
# Runs Chipyard/fpga-shells synthesis and stops. No place, route, timing
# closure, or bitstream. Emits hierarchical utilization reports plus a
# provenance manifest describing exactly how the numbers were produced.
#
# Invoked by ../area.mk, which is included by fpga/Makefile. Standard
# fpga-shells arguments arrive through tclargs (-top-module / -F / -board /
# -ip-vivado-tcls); RiskyBird options arrive through the environment, because
# fpga-shells' prologue.tcl rejects unknown flags.
#
# Environment:
#   RB_ATTRS        none | hierarchy | dsp | all   (default none)
#   RB_CONFIG       Chipyard config class name
#   RB_SUB_PROJECT  Chipyard FPGA subproject
#   RB_BOARD        fpga-shells board name
#   RB_MODEL        harness top module
#
# Outputs, under $build_dir/obj/report/:
#   rb_utilization_hier.txt     report_utilization -hierarchical (stamp file)
#   rb_utilization_hier_d6.txt  hierarchical, depth 6
#   rb_utilization_flat.txt     device totals
#   rb_ram_utilization.txt      per-macro RAM mapping detail
#   rb_manifest.json            provenance for this run
# and $build_dir/obj/post_synth.dcp from fpga-shells' synth.tcl.
#
# WHY THIS EXISTS AS A SEPARATE FLOW: a full place-and-route plus bitgen costs
# roughly an order of magnitude more wall time than synthesis. Area comparison
# across a config matrix does not need routing. Post-synthesis utilization is a
# usable proxy for relative area, but it is NOT the same number a routed design
# reports, and it says nothing about timing closure. Records from this flow are
# tagged flow="synth-only" and must never share an axis with post-route results.

set rb_scriptdir [file normalize [file dirname [info script]]]
set rb_dir       [file dirname $rb_scriptdir]
set base_dir     [file dirname $rb_dir]

# fpga-shells' prologue/init/synth read their own location out of $scriptdir.
set scriptdir [file join $base_dir fpga fpga-shells xilinx common tcl]
if {![file isdirectory $scriptdir]} {
    return -code error [list "fpga-shells common tcl not found at" $scriptdir]
}

proc rb_env {name default} {
    if {[info exists ::env($name)] && $::env($name) ne ""} { return $::env($name) }
    return $default
}

set rb_attrs       [rb_env RB_ATTRS none]
set rb_config      [rb_env RB_CONFIG unknown]
set rb_sub_project [rb_env RB_SUB_PROJECT unknown]

if {[lsearch -exact {none hierarchy dsp all} $rb_attrs] < 0} {
    return -code error [list "RB_ATTRS must be none|hierarchy|dsp|all, got" $rb_attrs]
}

# ---------------------------------------------------------------------------
# Project setup and IP generation, straight from fpga-shells. Unmodified.
# ---------------------------------------------------------------------------
source [file join $scriptdir "prologue.tcl"]
source [file join $scriptdir "init.tcl"]

# ---------------------------------------------------------------------------
# Optional RTL attribute injection, between elaboration output and synthesis.
#
# prologue.tcl calls `add_files`, which registers file references; the content
# is not parsed until synth_design. Editing gen-collateral here therefore does
# take effect.
# ---------------------------------------------------------------------------
set rb_gen_collateral [file join [file dirname $wrkdir] gen-collateral]
source [file join $rb_scriptdir "rb_attributes.tcl"]

set rb_injections {}
if {$rb_attrs ne "none"} {
    if {![file isdirectory $rb_gen_collateral]} {
        return -code error [list "RB_ATTRS=$rb_attrs but gen-collateral is missing at" $rb_gen_collateral]
    }
    puts "rb-area: attribute injection mode '$rb_attrs' into $rb_gen_collateral"
    set rb_injections [rb_inject_attributes $rb_gen_collateral $rb_attrs]
} else {
    puts "rb-area: attribute injection disabled (RB_ATTRS=none); measuring pristine generated RTL"
}

# ---------------------------------------------------------------------------
# Synthesis. fpga-shells' synth.tcl runs
#   synth_design -top $top -flatten_hierarchy rebuilt
# and writes $wrkdir/post_synth.dcp.
# ---------------------------------------------------------------------------
source [file join $scriptdir "synth.tcl"]

# ---------------------------------------------------------------------------
# Reports. Only what is meaningful post-synthesis: no timing summary, no DRC,
# no IO report -- those need a placed and routed design. fpga-shells' report.tcl
# is deliberately NOT sourced, because it runs those and exits 1 on negative
# slack, which is not a failure condition for an area probe.
# ---------------------------------------------------------------------------
set rptdir [file join $wrkdir report]
file mkdir $rptdir

report_utilization -hierarchical                       -file [file join $rptdir rb_utilization_hier.txt]
report_utilization -hierarchical -hierarchical_depth 6 -file [file join $rptdir rb_utilization_hier_d6.txt]
report_utilization                                     -file [file join $rptdir rb_utilization_flat.txt]
report_ram_utilization -detail                         -file [file join $rptdir rb_ram_utilization.txt]

# ---------------------------------------------------------------------------
# Provenance manifest.
# ---------------------------------------------------------------------------
proc rb_json_escape {s} {
    return [string map {\\ \\\\ \" \\\" \n \\n \r \\r \t \\t} $s]
}
proc rb_json_str {s} { return "\"[rb_json_escape $s]\"" }

set rb_part       [get_property PART [current_project]]
set rb_board_part [get_property BOARD_PART [current_project]]
set rb_version    [version -short]

set fh [open [file join $rptdir rb_manifest.json] w]
puts $fh "\{"
puts $fh "  \"schema\": \"riskybird.area-run.v1\","
puts $fh "  \"generated_by\": \"backends/chipyard/riskybird/tcl/rb_synth_area.tcl\","
puts $fh "  \"flow\": \"synth-only\","
puts $fh "  \"synthesized_at\": [rb_json_str [clock format [clock seconds] -format {%Y-%m-%dT%H:%M:%S%z}]],"
puts $fh "  \"tool\": \{"
puts $fh "    \"name\": \"vivado\","
puts $fh "    \"version\": [rb_json_str $rb_version]"
puts $fh "  \},"
puts $fh "  \"device\": \{"
puts $fh "    \"part\": [rb_json_str $rb_part],"
puts $fh "    \"board_part\": [rb_json_str $rb_board_part],"
puts $fh "    \"fpga_shells_board\": [rb_json_str $board]"
puts $fh "  \},"
puts $fh "  \"design\": \{"
puts $fh "    \"sub_project\": [rb_json_str $rb_sub_project],"
puts $fh "    \"config\": [rb_json_str $rb_config],"
puts $fh "    \"top_module\": [rb_json_str $top]"
puts $fh "  \},"
puts $fh "  \"synth_options\": [rb_json_str {-flatten_hierarchy rebuilt}],"
puts $fh "  \"rtl_mutation\": \{"
puts $fh "    \"mode\": [rb_json_str $rb_attrs],"
puts $fh "    \"pristine\": [expr {$rb_attrs eq "none" ? "true" : "false"}],"
if {$rb_attrs eq "hierarchy" || $rb_attrs eq "all"} {
    puts $fh "    \"totals_are_upper_bound\": true,"
    puts $fh "    \"totals_caveat\": \"keep_hierarchy inhibits cross-boundary optimization; device totals from this run are an upper bound and are not comparable with mode=none totals\","
} else {
    puts $fh "    \"totals_are_upper_bound\": false,"
}
puts $fh "    \"injections\": \["
set n [llength $rb_injections]
for {set i 0} {$i < $n} {incr i} {
    lassign [lindex $rb_injections $i] f m a ms ws w
    set sep [expr {$i == $n - 1 ? "" : ","}]
    puts $fh "      \{\"file\": [rb_json_str $f], \"module\": [rb_json_str $m], \"attribute\": [rb_json_str $a], \"module_status\": [rb_json_str $ms], \"wire_status\": [rb_json_str $ws], \"why\": [rb_json_str $w]\}$sep"
}
puts $fh "    \]"
puts $fh "  \},"
puts $fh "  \"reports\": \["
puts $fh "    \"rb_utilization_hier.txt\","
puts $fh "    \"rb_utilization_hier_d6.txt\","
puts $fh "    \"rb_utilization_flat.txt\","
puts $fh "    \"rb_ram_utilization.txt\""
puts $fh "  \]"
puts $fh "\}"
close $fh

puts "rb-area: DONE. part=$rb_part config=$rb_config attrs=$rb_attrs"
puts "rb-area: reports in $rptdir"
