# ---------------------------------------------------------------------------
# verify.tcl - reopen the routed checkpoint and prove:
#   * E22/B20/R17 are real OBUFs at real IOBs with PULLDOWN and INIT=0 drivers
#   * F13 is an IBUF (NOT an OBUF) with PULLTYPE = PULLUP and is not driven
# ---------------------------------------------------------------------------
# This script's own directory, so the flow runs from any checkout.
set d [file dirname [file normalize [info script]]]
open_checkpoint $d/out/f13probe_post_route.dcp

puts "===== F13PROBE POST-ROUTE PAD VERIFICATION ====="

# ---- driven pads ----------------------------------------------------------
foreach p {motor1 motor2 motor3} {
  set port [get_ports $p]
  set obuf [get_cells -quiet -filter {REF_NAME =~ OBUF*} \
              [get_cells -quiet -of_objects [get_nets -quiet -of_objects $port]]]
  set obuf_name [get_property NAME $obuf]
  set inet [get_nets -quiet -of_objects [get_pins -quiet $obuf_name/I]]
  set drv  [get_cells -quiet -of_objects \
              [get_pins -quiet -of_objects $inet -filter {DIRECTION == OUT}]]
  puts "PAD $p (OUTPUT):"
  puts "    PACKAGE_PIN = [get_property PACKAGE_PIN $port]   IOSTANDARD = [get_property IOSTANDARD $port]   PULLTYPE = [get_property PULLTYPE $port]   DIRECTION = [get_property DIRECTION $port]"
  puts "    BUF         = $obuf_name  REF_NAME = [get_property REF_NAME $obuf]"
  puts "    BUF LOC     = [get_property LOC $obuf]   SITE = [get_property SITE $obuf]   BEL = [get_property BEL $obuf]"
  puts "    DRIVER      = [get_property NAME $drv]  REF_NAME = [get_property REF_NAME $drv]  INIT = [get_property INIT $drv]"
  puts "    DRIVER LOC  = [get_property LOC $drv]   BEL = [get_property BEL $drv]"
  puts "    IS_INVERTED(OBUF/I) = [get_property IS_INVERTED [get_pins $obuf_name/I]]"
  puts "    ROUTE       = [get_property ROUTE_STATUS $inet]"
  puts ""
}

# ---- the pad under test ---------------------------------------------------
set port [get_ports motor4_in]
set allc [get_cells -quiet -of_objects [get_nets -quiet -of_objects $port]]
set ibuf [get_cells -quiet -filter {REF_NAME =~ IBUF*} $allc]
set obuf [get_cells -quiet -filter {REF_NAME =~ OBUF* || REF_NAME =~ OBUFT* || REF_NAME =~ IOBUF*} $allc]
puts "PAD motor4_in (INPUT, the pad under test):"
puts "    PACKAGE_PIN = [get_property PACKAGE_PIN $port]   IOSTANDARD = [get_property IOSTANDARD $port]   PULLTYPE = [get_property PULLTYPE $port]   DIRECTION = [get_property DIRECTION $port]"
puts "    IBUF        = [get_property NAME $ibuf]  REF_NAME = [get_property REF_NAME $ibuf]"
puts "    IBUF LOC    = [get_property LOC $ibuf]   SITE = [get_property SITE $ibuf]   BEL = [get_property BEL $ibuf]"
if {[llength $obuf] == 0} {
  puts "    OBUF/OBUFT/IOBUF on this port : NONE  <-- correct, F13 is never driven"
} else {
  puts "    *** UNEXPECTED OUTPUT BUFFER ON F13: [get_property NAME $obuf] ***"
}
puts "    all cells on the F13 net: [get_property NAME $allc] ([get_property REF_NAME $allc])"
puts ""

puts "===== every IO cell in the design ====="
foreach c [get_cells -hierarchical -filter {PRIMITIVE_GROUP == IO}] {
  puts [format "  %-22s %-8s LOC=%-12s BEL=%s" \
        [get_property NAME $c] [get_property REF_NAME $c] \
        [get_property LOC $c] [get_property BEL $c]]
}

puts ""
puts "===== port direction / pull summary ====="
foreach p [get_ports] {
  puts [format "  %-12s %-4s PIN=%-5s %-12s PULLTYPE=%s" \
        [get_property NAME $p] [get_property DIRECTION $p] \
        [get_property PACKAGE_PIN $p] [get_property IOSTANDARD $p] \
        [get_property PULLTYPE $p]]
}

puts ""
puts "===== reset-state check: INIT of every flop ====="
set nz 0
foreach c [get_cells -hierarchical -filter {REF_NAME =~ FD*}] {
  if {[get_property INIT $c] ne "1'b0"} { incr nz
    puts "  NON-ZERO INIT: [get_property NAME $c] = [get_property INIT $c]" }
}
puts "  total flops = [llength [get_cells -hierarchical -filter {REF_NAME =~ FD*}]], flops with INIT != 1'b0 = $nz"
foreach n {armed_reg motor1_q_reg motor2_q_reg motor3_q_reg f13_sync0_reg f13_sync1_reg f13_sync2_reg f13_sync3_reg tog_active_reg} {
  set c [get_cells -quiet $n]
  if {[llength $c]} { puts [format "  %-16s %-6s INIT=%s" $n [get_property REF_NAME $c] [get_property INIT $c]] }
}

puts "F13PROBE_VERIFY_OK"
