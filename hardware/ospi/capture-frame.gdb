# Capture one HM01B0 frame and read it out over JTAG.
#
#   ./rb debug camera_photo --target arty200t \
#       --config RocketArty200TDroneFullDDRConfig --program \
#       --program-usb-location 7-1.1.4.3 --usb-location 7-1.1.4.1 \
#       --gdb-script hardware/ospi/capture-frame.gdb
#
# The frame is far too large to print, so the workload leaves it in pixels[] and
# this script copies it straight out of RAM. That only works on a DDR-backed
# shell: 326x242 is 78892 bytes and a scratchpad holds 32768.
#
# camera_photo captures a whole frame by default and needs no extra flags. The
# older camera_bringup target defaults to a single line, and widening it needs
# RB_EXTRA_CFLAGS=-DCAM_CAPTURE_PIXELS=78892 on `rb debug` and not just on
# `rb build` -- `rb debug` rebuilds, so setting it only on the build silently
# reverts the capture size.
#
# Stops at camera_frame_ready() on success and inside fail() on any failed
# stage, so a partial capture is still dumped and still inspectable rather than
# leaving GDB running with nothing written.

set pagination off
set confirm off

# No `load` here: `rb debug` already passes `-ex load` unless --no-load, so a
# load in this script is a second, redundant transfer of the whole image over
# JTAG. Kept out rather than duplicated.

# The motor PWM state as the bitstream leaves it, read while the core is still
# halted at reset with no instruction executed. This is the only direct evidence
# about what RocketArty200TDroneFullDDRConfig drives motor1-4 to between
# configuration and the first store, which is the window nothing in software can
# cover. All-zero is the documented reset state, which WithArty200TPWM's harness
# inversion is supposed to render as 0% duty.
printf "\n=== motor PWM at reset (pre-execution) ===\n"
printf "  pwm0 cfg=0x%08x cmp=%08x %08x %08x %08x\n", \
    *(unsigned int *)0x10050000, *(unsigned int *)0x10050020, \
    *(unsigned int *)0x10050024, *(unsigned int *)0x10050028, *(unsigned int *)0x1005002c
printf "  pwm1 cfg=0x%08x cmp=%08x %08x %08x %08x\n", \
    *(unsigned int *)0x10051000, *(unsigned int *)0x10051020, \
    *(unsigned int *)0x10051024, *(unsigned int *)0x10051028, *(unsigned int *)0x1005102c

break camera_frame_ready
break fail
continue

# Which breakpoint stopped us decides what is worth printing. fail() carries the
# reason in its own arguments, so a failed run explains itself over JTAG without
# needing the console -- which matters because the console is not always the link
# being used to run this.
if $_hit_bpnum == 2
    printf "\n=== FAILED: stage=%s -- %s ===\n", stage, detail
end

printf "\n=== capture core ===\n"
printf "  measured   %u x %u\n", *(unsigned int *)0x10080014, *(unsigned int *)0x10080018
printf "  configured %u x %u\n", \
    *(unsigned int *)0x10080004 & 0xffff, (*(unsigned int *)0x10080004 >> 16) & 0xffff
printf "  pclk=%u fvld=%u lvld=%u frames=%u\n", \
    *(unsigned int *)0x10080030, *(unsigned int *)0x10080034, \
    *(unsigned int *)0x10080038, *(unsigned int *)0x10080010
printf "  flags=0x%08x capstat=0x%08x\n", \
    *(unsigned int *)0x1008001c, *(unsigned int *)0x10080040

printf "\n=== frame ===\n"
printf "  valid bytes in pixels[]: %u\n", camera_frame_pixels

# Guarded: dumping a zero-length range is a GDB error, and a run that captured
# nothing is a normal outcome to report rather than a reason to abort the script
# before the diagnostics above have been read.
if camera_frame_pixels > 0
    dump binary memory camera-frame.raw &pixels[0] &pixels[0] + camera_frame_pixels
    printf "  wrote camera-frame.raw\n"
    # Convert here rather than printing the command to run. The reshape width is
    # a runtime value -- the width the core MEASURED, which is not the width GEOM
    # is configured for -- so `eval` substitutes it into the shell command.
    # Leaving this as an instruction meant a successful capture looked like a
    # failed one: the raw dump is not viewable, and "where is my photo" is the
    # obvious response to a directory with no image in it.
    set $measured_width = *(unsigned int *)0x10080014
    eval "shell python3 hardware/ospi/frame-to-png.py camera-frame.raw %u", $measured_width
else
    printf "  nothing captured -- no files written\n"
end

detach
quit
