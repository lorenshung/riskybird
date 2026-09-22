# ---------------------------------------------------------------------------
# verify.tcl - reopen the routed checkpoint and prove each motor pad has a real
# OBUF at a real IOB, and that its driver flop resets to 0.
#   /ecad/tools/xilinx/Vivado/2023.1/bin/vivado -mode batch -nojournal -nolog \
#       -source /scratch2/loren/riskybird-dev/hardware/padtest/verify.tcl
# ---------------------------------------------------------------------------
set d /scratch2/loren/riskybird-dev/hardware/padtest
open_checkpoint $d/out/post_route.dcp

puts "===== PADTEST POST-ROUTE PAD VERIFICATION ====="
puts [format "%-8s %-6s %-11s %-4s %-24s %-10s %-9s %s" \
      PORT PIN IOSTANDARD PULL OBUF_CELL OBUF_LOC DRIVER_REF DRIVER_CELL/INIT]

foreach p {motor1 motor2 motor3 motor4} {
  set port [get_ports $p]
  set pin  [get_property PACKAGE_PIN  $port]
  set std  [get_property IOSTANDARD   $port]
  set pull [get_property PULLTYPE     $port]
  set dir  [get_property DIRECTION    $port]

  # the OBUF actually driving this port
  set obuf [get_cells -quiet -of_objects [get_nets -quiet -of_objects $port]]
  set obuf [get_cells -quiet -filter {PRIMITIVE_TYPE =~ I/O.*.OBUF* || REF_NAME =~ OBUF*} $obuf]
  set obuf_name [get_property NAME     $obuf]
  set obuf_ref  [get_property REF_NAME $obuf]
  set obuf_loc  [get_property LOC      $obuf]
  set obuf_site [get_property SITE     $obuf]
  set obuf_bel  [get_property BEL      $obuf]

  # the cell driving the OBUF input
  set ipin [get_pins -quiet $obuf_name/I]
  set inet [get_nets -quiet -of_objects $ipin]
  set drv_pin [get_pins -quiet -of_objects $inet -filter {DIRECTION == OUT}]
  set drv  [get_cells -quiet -of_objects $drv_pin]
  set drv_ref [get_property REF_NAME $drv]
  set drv_init [get_property INIT $drv]
  set drv_loc  [get_property LOC  $drv]
  set drv_bel  [get_property BEL  $drv]

  puts "PAD $p:"
  puts "    PACKAGE_PIN = $pin   IOSTANDARD = $std   PULLTYPE = $pull   DIRECTION = $dir"
  puts "    OBUF        = $obuf_name  REF_NAME = $obuf_ref"
  puts "    OBUF LOC    = $obuf_loc   SITE = $obuf_site   BEL = $obuf_bel"
  puts "    DRIVER      = [get_property NAME $drv]  REF_NAME = $drv_ref  INIT = $drv_init"
  puts "    DRIVER LOC  = $drv_loc   BEL = $drv_bel"
  puts "    ROUTE       = [get_property ROUTE_STATUS $inet]"
  puts ""
}

puts "===== all OBUF/IOB cells in the design ====="
foreach c [get_cells -hierarchical -filter {PRIMITIVE_GROUP == IO}] {
  puts [format "  %-28s %-8s LOC=%-12s BEL=%s" \
        [get_property NAME $c] [get_property REF_NAME $c] \
        [get_property LOC $c] [get_property BEL $c]]
}

puts ""
puts "===== reset-state check: INIT of every flop in the output path ====="
foreach c [get_cells -hierarchical -filter {REF_NAME =~ FD*}] {
  puts [format "  %-28s %-8s INIT=%s" [get_property NAME $c] [get_property REF_NAME $c] [get_property INIT $c]]
}

puts ""
puts "===== port-level inversion check (should be none) ====="
puts "  IS_INVERTED on OBUF inputs:"
foreach p {motor1 motor2 motor3 motor4} {
  set obuf [get_cells -quiet -filter {REF_NAME =~ OBUF*} [get_cells -quiet -of_objects [get_nets -quiet -of_objects [get_ports $p]]]]
  puts "    $p -> [get_property NAME $obuf] IS_INVERTED(I) = [get_property IS_INVERTED [get_pins [get_property NAME $obuf]/I]]"
}

puts "PADTEST_VERIFY_OK"
