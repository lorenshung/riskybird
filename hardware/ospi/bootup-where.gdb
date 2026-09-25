# Where is a bootup_check image, without loading or resuming it?
#
#   ./rb debug bootup_check --target arty200t --config <Config> \
#       --usb-location <debug-usb> --ram-console --no-load \
#       --gdb-script hardware/ospi/bootup-where.gdb
#
# Reports pc, a short backtrace and how many frame bytes the camera stage
# stored, then leaves the hart halted.

set pagination off
set confirm off
printf "\n=== where ===\n"
info registers pc
bt 6
printf "camera_frame_pixels = %u\n", camera_frame_pixels
monitor halt
quit
