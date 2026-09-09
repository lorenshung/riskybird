# Run the whole-board bootup check and read its photograph out over JTAG.
#
#   ./rb debug bootup_check --target arty200t \
#       --config RocketArty200TDroneFullDDRConfig \
#       --usb-location 7-1.1.4.1 \
#       --gdb-script hardware/ospi/bootup-capture.gdb
#
# Add --mode motors to include the motor sweep (BATTERY REQUIRED, PROPELLERS OFF).
#
# The console report comes out on the serial node (rb console --port ...); this
# script only handles the frame, which is far too large to print. bootup_check
# leaves it in pixels[] and stops at camera_frame_ready() after the summary has
# already been printed, so reading the picture out never costs you the report.
#
# Only works on a DDR-backed shell: 326x242 is 78892 bytes, a scratchpad is 32768.
#
# Unlike capture-frame.gdb there is no second breakpoint on fail(): bootup_check
# has no such function -- a failed camera stage is one FAIL line in its table and
# the run continues to the other peripherals -- so the single anchor is reached
# whether or not the capture worked, and a partial frame is dumped either way.

set pagination off
set confirm off

# No `load` here: `rb debug` already passes `-ex load` unless --no-load, so a
# load in this script is a second, redundant transfer of the whole image.

break camera_frame_ready
continue

printf "\n=== capture core ===\n"
printf "  measured   %u x %u\n", *(unsigned int *)0x10080014, *(unsigned int *)0x10080018
printf "  configured %u x %u\n", \
    *(unsigned int *)0x10080004 & 0xffff, (*(unsigned int *)0x10080004 >> 16) & 0xffff
printf "  pclk=%u fvld=%u lvld=%u frames=%u\n", \
    *(unsigned int *)0x10080030, *(unsigned int *)0x10080034, \
    *(unsigned int *)0x10080038, *(unsigned int *)0x10080010
printf "  flags=0x%08x capstat=0x%08x\n", \
    *(unsigned int *)0x1008001c, *(unsigned int *)0x10080040
# CAPACITY is the hard ceiling on a single capture, and the one number that
# decides whether a given WIDTHxHEIGHT can be asked for at all. Printed because
# a request larger than this fails in the workload before the core is even
# armed, and the console line saying so is easy to miss when the frame is the
# thing being looked for.
printf "  capacity=%u pixels; pixtarget=%u; capcount=%u\n", \
    *(unsigned int *)0x10080024, *(unsigned int *)0x10080028, \
    *(unsigned int *)0x1008002c

printf "\n=== frame ===\n"
printf "  valid bytes in pixels[]: %u\n", camera_frame_pixels

# Guarded: dumping a zero-length range is a GDB error, and a run whose camera
# stage failed is a normal outcome to report rather than a reason to abort.
if camera_frame_pixels > 0
    dump binary memory bootup-frame.raw &pixels[0] &pixels[0] + camera_frame_pixels
    printf "  wrote bootup-frame.raw\n"
    # The reshape width is a runtime value -- the width the core MEASURED, not
    # the width GEOM is configured for -- so eval substitutes it in rather than
    # leaving the reader a command to run and a raw file they cannot view.
    set $measured_width = *(unsigned int *)0x10080014
    eval "shell python3 hardware/ospi/frame-to-png.py bootup-frame.raw %u", $measured_width
else
    printf "  camera stage captured nothing -- no files written\n"
end

detach
quit
