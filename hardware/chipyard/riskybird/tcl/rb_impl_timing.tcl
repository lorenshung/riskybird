# Place and route an already-synthesized checkpoint, then report timing and power.
#
# WHY THIS EXISTS: `rb area` is synthesis-only, so it answers "does it fit" and
# says nothing about whether the design closes at its target frequency. The
# missing evidence is post-route: achieved F_max, worst negative slack, and
# power.
#
# WHY IT STARTS FROM A CHECKPOINT rather than re-running the whole flow. The
# area flow measures RTL that has had `use_dsp` attributes injected into
# gen-collateral, and that injection is a working-tree edit undone by the next
# elaboration. Re-running `make bitstream` re-elaborates, discards the
# injection, and would route a different design from the one whose utilization
# was recorded. Opening `obj/post_synth.dcp` instead routes exactly the netlist
# that was measured -- the attributes are already baked into it -- and skips
# synthesis, which is most of the runtime.
#
# Report filenames match what `rb area timing` reads: timing.txt and clocks.txt.
#
# Usage, from the fpga directory:
#   vivado -nojournal -mode batch -source riskybird/tcl/rb_impl_timing.tcl \
#          -tclargs -checkpoint <path/to/post_synth.dcp>

set rb_scriptdir [file normalize [file dirname [info script]]]

proc rb_env {name default} {
    if {[info exists ::env($name)] && $::env($name) ne ""} { return $::env($name) }
    return $default
}

# The checkpoint locates everything else: reports go beside it.
set rb_checkpoint ""
for {set i 0} {$i < [llength $argv]} {incr i} {
    if {[lindex $argv $i] eq "-checkpoint"} {
        set rb_checkpoint [lindex $argv [expr {$i + 1}]]
    }
}
if {$rb_checkpoint eq ""} {
    set rb_checkpoint [rb_env RB_CHECKPOINT ""]
}
if {$rb_checkpoint eq "" || ![file exists $rb_checkpoint]} {
    return -code error [list "no post-synthesis checkpoint at" $rb_checkpoint \
        "-- run `rb area run` for this config first"]
}

set rb_objdir [file dirname [file normalize $rb_checkpoint]]
set rb_rptdir [file join $rb_objdir report]
file mkdir $rb_rptdir

puts "rb-impl: opening $rb_checkpoint"
open_checkpoint $rb_checkpoint

# What the netlist actually is, before touching it. If this disagrees with the
# recorded area measurement then the checkpoint is not the one that was
# measured, and the timing result would describe a different design.
report_utilization -file [file join $rb_rptdir rb_impl_pre_utilization.txt]
puts "rb-impl: constrained clocks in the checkpoint:"
report_clocks

# ---------------------------------------------------------------------------
# Implementation. phys_opt_design after place and after route because at high
# occupancy the placer and router both leave slack on the table that physical
# optimization recovers; skipping them understates achievable F_max.
# ---------------------------------------------------------------------------
# RB_IMPL_DIRECTIVE selects Vivado's implementation effort. Empty (the default)
# runs every step on its default directive, which is what every recorded result
# here was produced with -- so leaving it unset reproduces them.
#
# WHY IT EXISTS. A design can miss its constraint by a margin smaller than the
# spread between directives: the Gemmini-16 shell routed at WNS -0.091 ns on the
# defaults, 91 ps short of 40 MHz on a 25 ns period. That is not a design that
# needs re-architecting, it is a design the placer gave up on early, and the
# remedy is more effort rather than a slower clock or a smaller accelerator.
#
# Set RB_IMPL_DIRECTIVE=explore for the usual escalation. The directives differ
# per command, so they are mapped rather than passed through verbatim.
set rb_directive [rb_env RB_IMPL_DIRECTIVE ""]
switch -- [string tolower $rb_directive] {
    "" {
        set rb_opt_dir    ""
        set rb_place_dir  ""
        set rb_phys_dir   ""
        set rb_route_dir  ""
    }
    "explore" {
        set rb_opt_dir    "-directive Explore"
        set rb_place_dir  "-directive Explore"
        set rb_phys_dir   "-directive Explore"
        set rb_route_dir  "-directive Explore"
    }
    "aggressive" {
        set rb_opt_dir    "-directive Explore"
        set rb_place_dir  "-directive ExtraTimingOpt"
        set rb_phys_dir   "-directive AggressiveExplore"
        set rb_route_dir  "-directive AggressiveExplore"
    }
    default {
        return -code error [list "unknown RB_IMPL_DIRECTIVE" $rb_directive \
            "-- expected one of: (empty), explore, aggressive"]
    }
}
if {$rb_directive ne ""} {
    puts "rb-impl: directive '$rb_directive' -- NOT the default-effort flow the\
          recorded numbers came from; record this alongside any result"
}

puts "rb-impl: opt_design $rb_opt_dir"
eval opt_design $rb_opt_dir
puts "rb-impl: place_design $rb_place_dir"
eval place_design $rb_place_dir
puts "rb-impl: phys_opt_design (post-place) $rb_phys_dir"
eval phys_opt_design $rb_phys_dir
puts "rb-impl: route_design $rb_route_dir"
eval route_design $rb_route_dir
puts "rb-impl: phys_opt_design (post-route) $rb_phys_dir"
eval phys_opt_design $rb_phys_dir

write_checkpoint -force [file join $rb_objdir rb_post_route.dcp]

# ---------------------------------------------------------------------------
# Reports. timing.txt and clocks.txt are the names `rb area timing` parses; the
# rest are for inspection.
# ---------------------------------------------------------------------------
report_timing_summary -max_paths 10 -file [file join $rb_rptdir timing.txt]
report_clocks                        -file [file join $rb_rptdir clocks.txt]
report_utilization -hierarchical     -file [file join $rb_rptdir rb_impl_utilization_hier.txt]
report_utilization                   -file [file join $rb_rptdir rb_impl_utilization_flat.txt]
report_power                         -file [file join $rb_rptdir power.txt]
report_drc                           -file [file join $rb_rptdir drc.txt]

# Worst slack on the console too, so a batch log is enough to read the outcome.
set rb_wns [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]
puts "rb-impl: worst negative slack (setup): $rb_wns ns"

# Bitstream last, and non-fatal. A design that misses timing still produces a
# valid bitstream, and a DRC failure here must not discard the reports above --
# those are the reason the run happened.
if {[catch {write_bitstream -force [file join $rb_objdir rb_impl.bit]} rb_bit_err]} {
    puts "rb-impl: bitstream NOT written: $rb_bit_err"
} else {
    puts "rb-impl: bitstream [file join $rb_objdir rb_impl.bit]"
}

puts "rb-impl: reports in $rb_rptdir"
