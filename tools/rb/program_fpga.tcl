# Program the attached Xilinx FPGA whose part name matches argv[1].
#
# Searches every JTAG target, not just the first. `open_hw_target` with no
# argument opens whichever target the server happens to list first, and on a
# bench with more than one cable attached that is a coin flip: with a U250 card
# and the TE0712 both present it opened the U250 and reported "no attached FPGA
# matches xc7a200t; discovered: xcu250" -- which reads like the board is
# missing when it is simply on the next cable along.

if {$argc != 2} {
    error "usage: program_fpga.tcl BITSTREAM DEVICE_PATTERN"
}

set bitstream [file normalize [lindex $argv 0]]
set device_pattern [string tolower [lindex $argv 1]]

open_hw_manager
connect_hw_server -allow_non_jtag

set selected ""
set discovered {}
foreach target [get_hw_targets] {
    current_hw_target $target
    open_hw_target
    foreach device [get_hw_devices] {
        set part_name [string tolower [get_property PART $device]]
        lappend discovered "[get_property PART $device] on $target"
        if {[string first $device_pattern $part_name] >= 0} {
            set selected $device
            break
        }
    }
    if {$selected ne ""} {
        break
    }
    close_hw_target
}

if {$selected eq ""} {
    error "no attached FPGA matches '$device_pattern'; discovered: $discovered"
}

puts "Programming $selected with $bitstream"
current_hw_device $selected
refresh_hw_device $selected
set_property PROGRAM.FILE $bitstream $selected
program_hw_devices $selected
refresh_hw_device $selected
close_hw_target
disconnect_hw_server
close_hw_manager
