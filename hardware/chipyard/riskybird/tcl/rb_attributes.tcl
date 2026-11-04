# RiskyBird RTL attribute injection.
#
# Vivado's synthesis heuristics leave two kinds of area on the table that no
# Chipyard-level parameter can reach:
#
#   1. Small signed multipliers (Gemmini's MacUnit, Saturn's OuterProductCell)
#      are emitted as combinational `a * b + c` and mapped to LUTs instead of a
#      DSP48E2, because the sub-9-bit operands and the surrounding muxes defeat
#      the inference heuristic.
#   2. `synth_design -flatten_hierarchy rebuilt` optimizes across module
#      boundaries before rebuilding hierarchy for reporting, so a hierarchical
#      utilization report attributes shared logic arbitrarily.
#
# Both are addressed with SystemVerilog attributes, which Chisel 6 + CIRCT
# firtool cannot emit on a module declaration: the legacy
# firrtl.AttributeAnnotation is only consumed by the SFC AddDescriptionNodes
# pass, which firtool does not run. So the attributes are injected into the
# generated Verilog after elaboration and before synthesis.
#
# This is a real caveat, not a free win:
#
#   * Injection MUTATES generated RTL in gen-collateral. Every applied edit is
#     recorded in the run manifest, and the original is preserved as a
#     <file>.rb-orig sibling so `make rb-area-restore` is exact.
#   * keep_hierarchy INHIBITS cross-boundary optimization. A `hierarchy` or
#     `all` run gives faithful per-module attribution but its totals are an
#     UPPER BOUND. Never plot `none` totals and `hierarchy` totals on one axis.
#   * use_dsp is a hint, not a mandate. Vivado may ignore it. The manifest
#     records what was requested; the utilization report records what happened.
#
# Injection is opt-in and defaults to off (RB_ATTRS=none).

# Injection table. Each entry:
#   mode      which RB_ATTRS mode enables it (hierarchy | dsp)
#   glob      filename pattern under gen-collateral
#   attr      attribute text to inject
#   wire_re   optional -line regex; the matched declaration gets the attribute
#             prefixed in place. {} means module declaration only.
#   why       recorded in the manifest
set rb_injection_table {
    {hierarchy ChipTop.sv            {keep_hierarchy = "yes"} {} {top-level SoC boundary for attribution}}
    {hierarchy RocketTile*.sv        {keep_hierarchy = "yes"} {} {per-tile attribution}}
    {hierarchy ShuttleTile*.sv       {keep_hierarchy = "yes"} {} {per-tile attribution}}
    {hierarchy SaturnRocketUnit*.sv  {keep_hierarchy = "yes"} {} {separate vector unit from host core}}
    {hierarchy SaturnShuttleUnit*.sv {keep_hierarchy = "yes"} {} {separate vector unit from host core}}
    {hierarchy Gemmini.sv            {keep_hierarchy = "yes"} {} {separate RoCC accelerator from tile}}
    {hierarchy MeshWithDelays.sv     {keep_hierarchy = "yes"} {} {isolate the systolic array from Gemmini control}}
    {hierarchy OuterProductUnit.sv   {keep_hierarchy = "yes"} {} {isolate the OPU mesh from the Saturn backend}}

    {dsp MacUnit.sv           {use_dsp = "yes"} {wire \[15:0\] _io_out_d_T} {Gemmini PE 8x8 signed mul + 20b add -> one DSP48E2}}
    {dsp OuterProductCell.sv  {use_dsp = "yes"} {wire \[15:0\]\s+prod =}    {Saturn OPU cell 8x8 signed mul + 32b acc add -> one DSP48E2}}
    {dsp MulAddRecFNPipe_*.sv {use_dsp = "yes"} {} {hardfloat FMA mantissa mul; defensive, historically already inferred}}
}

# Return 1 if $mode is enabled by the RB_ATTRS setting $setting.
proc rb_mode_enabled {setting mode} {
    if {$setting eq "all"} { return 1 }
    return [expr {$setting eq $mode}]
}

# Prefix "(* $attr *)" onto the declaration of module $mod in $txt.
# Returns {status text}, where status is applied | already-present | not-found.
proc rb_inject_module {txt mod attr} {
    if {[regexp "\\(\\*\[^\n\]*\\*\\)\\s+module ${mod}\\M" $txt]} {
        return [list already-present $txt]
    }
    if {[regsub -line "^module ${mod}\\M" $txt "(* $attr *)\nmodule $mod" out] == 0} {
        return [list not-found $txt]
    }
    return [list applied $out]
}

# Insert "(* $attr *) " immediately before the first match of $wire_re, which
# preserves the declaration text and its indentation without reconstructing
# either. $wire_re must NOT be anchored to start-of-line: it has to keep
# matching after injection so that an already-annotated declaration is reported
# as already-present rather than as a missing pattern.
#
# Returns {status text}, where status is applied | already-present | not-found.
proc rb_inject_wire {txt wire_re attr} {
    if {![regexp -indices $wire_re $txt idx]} { return [list not-found $txt] }
    lassign $idx s e
    # Everything between the start of the matched line and the match itself.
    set nl [string last "\n" [string range $txt 0 $s]]
    set prefix [string range $txt [expr {$nl + 1}] [expr {$s - 1}]]
    if {[string first $attr $prefix] >= 0} { return [list already-present $txt] }
    # Insert, do not replace: `string replace` with last < first is a no-op in
    # Tcl and would silently drop the attribute.
    set out [string range $txt 0 [expr {$s - 1}]]
    append out "(* $attr *) "
    append out [string range $txt $s end]
    return [list applied $out]
}

# Apply every injection enabled by $setting to the generated RTL in $gen_dir.
# Returns a list of records: {file module attr module_status wire_status why}
#   module_status  applied | already-present | not-found | file-absent
#   wire_status    applied | already-present | not-found | n/a | file-absent
# Module and wire outcomes are reported separately: a wire pattern that stops
# matching means the generated RTL changed shape and the DSP hint is incomplete,
# which must stay visible even when the module attribute applied cleanly.
proc rb_inject_attributes {gen_dir setting} {
    global rb_injection_table
    set records {}
    if {$setting eq "none" || $setting eq ""} { return $records }

    foreach entry $rb_injection_table {
        lassign $entry mode glob attr wire_re why
        if {![rb_mode_enabled $setting $mode]} { continue }

        set matches [glob -nocomplain -directory $gen_dir $glob]
        if {[llength $matches] == 0} {
            lappend records [list $glob {} $attr file-absent file-absent $why]
            continue
        }

        foreach path [lsort $matches] {
            set mod [file rootname [file tail $path]]
            set fh [open $path r]
            set txt [read $fh]
            close $fh
            set orig $txt

            lassign [rb_inject_module $txt $mod $attr] mstatus txt

            set wstatus n/a
            if {$wire_re ne {}} {
                lassign [rb_inject_wire $txt $wire_re $attr] wstatus txt
            }

            if {$txt ne $orig} {
                # Preserve the pristine file once, so restore is exact.
                if {![file exists ${path}.rb-orig]} {
                    set bh [open ${path}.rb-orig w]
                    puts -nonewline $bh $orig
                    close $bh
                }
                set fh [open $path w]
                puts -nonewline $fh $txt
                close $fh
                puts "rb-area: injected ($attr) into [file tail $path]"
            }

            lappend records [list [file tail $path] $mod $attr $mstatus $wstatus $why]
        }
    }
    return $records
}
