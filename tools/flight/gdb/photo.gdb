# Read the camera photo out of a RUNNING flight image, over JTAG, without a reset or reload.
#
#   rb fly photo                                    (the normal way: session dir, PNG paths)
#
#   ./rb debug flight_controller --target arty200t \
#       --config RocketArty200TDroneFullDDRConfig --mode t21-hover-1.0m-photo \
#       --usb-location <debug> --no-load --gdb-script tools/flight/gdb/photo.gdb
#
# The image is a photo build (--mode t21-hover-1.0m-photo, or photo-bench on the bench). It
# captures one HM01B0 frame mid-hover, drains it into rose_photo_pixels[] once the motors are off, and
# records how that went in rose_photo_state. This script only reads: it halts, prints every
# rose_photo_* field, and -- only when the state says the pixels are valid -- dumps them and
# converts them to PNG at the MEASURED width (rose_photo_width, the core's LASTWIDTH; the
# configured width is 2 px narrower and shears the image one pixel per row).
#
# Output directory: $photo_outdir if the caller set it (rb fly photo sets it to the session
# directory, results/fly/<ts>/), else the current directory -- which under `rb debug` is the
# repository root. Files: photo.raw, photo.png (frame-to-png.py), photo-colour.png
# (frame-to-colour.py), photo-gdb.log (this script's output).
#
# THE CORE IS LEFT HALTED. `rb debug` halts it on attach (`monitor halt`), and a plain detach
# does not resume it (measured on the board, docs/motor-pwm-bringup.md). That is deliberate:
# a halted core stops the ESP frame stream, and the ESP cuts all four motors on its 120 ms
# receive failsafe, so whatever state the drone was in, it is now safe to handle. To run the
# image again, reload it (rb fly load / commit). Run this only after landing.

set pagination off
set confirm off

if $_isvoid($photo_outdir)
    set $photo_outdir = "."
end

eval "set logging file %s/photo-gdb.log", $photo_outdir
set logging overwrite on
set logging enabled on

# Read every field once, with the core halted: nothing changes under us after this, and the
# decisions below are all made on the same snapshot that is printed.
set $photo_state    = rose_photo_state
set $photo_count    = rose_photo_count
set $photo_width    = rose_photo_width
set $photo_height   = rose_photo_height
set $photo_captured = rose_photo_captured
set $photo_flags    = rose_photo_flags
set $photo_arm_t_ms = rose_photo_arm_t_ms
set $photo_arm_z_mm = rose_photo_arm_z_mm
set $photo_capacity = sizeof(rose_photo_pixels)

printf "\n=== camera photo (core halted) ===\n"
printf "  rose_photo_state    = %d\n", $photo_state
printf "  rose_photo_count    = %u   (capacity %u)\n", $photo_count, $photo_capacity
printf "  rose_photo_width    = %u   (measured LASTWIDTH -- the reshape width)\n", $photo_width
printf "  rose_photo_height   = %u\n", $photo_height
printf "  rose_photo_captured = %u   (CAPCOUNT)\n", $photo_captured
printf "  rose_photo_flags    = 0x%08x\n", $photo_flags
printf "  rose_photo_arm_t_ms = %d   (ms since takeoff at arming; bench: uptime)\n", $photo_arm_t_ms
printf "  rose_photo_arm_z_mm = %d   (estimated altitude at arming)\n", $photo_arm_z_mm
# Both strings are NULL unless something failed, and printf %s on NULL is a memory error
# that would abort the batch before the diagnosis below.
if rose_photo_stage != 0
    printf "  rose_photo_stage    = %s\n", rose_photo_stage
else
    printf "  rose_photo_stage    = (none)\n"
end
if rose_photo_detail != 0
    printf "  rose_photo_detail   = %s\n", rose_photo_detail
else
    printf "  rose_photo_detail   = (none)\n"
end
# One line for rb fly photo to parse.
printf "PHOTO-GDB state=%d count=%u width=%u height=%u\n", $photo_state, $photo_count, $photo_width, $photo_height

printf "\n=== diagnosis ===\n"
set $photo_dump = 0
if $photo_state == 5
    if $photo_count == 0
        printf "  DRAINED with zero pixels -- nothing to dump (firmware says OK but drained nothing)\n"
    else
        if $photo_count > $photo_capacity
            printf "  DRAINED count %u exceeds rose_photo_pixels[%u] -- not dumping past the array\n", $photo_count, $photo_capacity
        else
            printf "  OK: frame drained, %u pixels valid\n", $photo_count
            set $photo_dump = 1
        end
    end
end
if $photo_state == 0
    printf "  OFF: the photo path never started. Is this a photo build (rb-build.txt: -DROSE_CAMERA=1)?\n"
    printf "  Or the image running is not the one these symbols came from (rb fly load again).\n"
end
if $photo_state == 1
    printf "  PREPARING: the sensor was still being brought up (boot + ~6.3 s AE settle) when the\n"
    printf "  core was halted. Halted too early, or sensor bring-up hung. Reload to try again.\n"
end
if $photo_state == 2
    printf "  READY but never ARMED: the sensor streamed, the trigger never fired.\n"
    printf "  Flight: it never reached mid-hover, climb + hover/2 (not committed, or aborted before).\n"
    printf "  Bench: halted before ROSE_PHOTO_BENCH_AFTER_MS (15 s after boot) -- wait ~25 s.\n"
end
if $photo_state == 3
    printf "  ARMED but no frame captured yet (CAPCOUNT %u of %u x %u).\n", $photo_captured, $photo_width, $photo_height
    printf "  Halted mid-capture, or the capture core never saw a whole frame (it should time out to -2).\n"
end
if $photo_state == 4
    printf "  CAPTURED in the core but NOT DRAINED into RAM -- rose_photo_pixels is not valid.\n"
    printf "  The drain runs only with the motors off: halted before disarm, or between capture and\n"
    printf "  drain. The frame sits in the capture core, which this script does not read.\n"
end
if $photo_state == -1
    printf "  FAILED (-1): prepare failed -- sensor bring-up (I2C id / config). See stage/detail above.\n"
end
if $photo_state == -2
    printf "  FAILED (-2): capture incomplete or timed out -- CAPCOUNT %u, flags 0x%08x.\n", $photo_captured, $photo_flags
    printf "  Intermittent on this sensor: an armed capture does not always complete. Retry.\n"
end
if $photo_state == -3
    printf "  FAILED (-3): drain from the capture core into RAM failed. See stage/detail above.\n"
end
if $photo_state > 5 || $photo_state < -3
    printf "  UNKNOWN state %d -- the firmware contract has moved; update photo.gdb\n", $photo_state
end

if $photo_dump
    eval "dump binary memory %s/photo.raw &rose_photo_pixels[0] &rose_photo_pixels[%u]", $photo_outdir, $photo_count
    eval "printf \"  wrote %s/photo.raw (%u bytes)\\n\"", $photo_outdir, $photo_count
    if $photo_width * $photo_height != $photo_count
        printf "  note: count %u != width %u x height %u -- a partial frame; the PNG is what drained\n", $photo_count, $photo_width, $photo_height
    end
    if $photo_width > 2
        # eval, as capture-frame.gdb does: the reshape width is a runtime value.
        eval "shell python3 hardware/ospi/frame-to-png.py '%s/photo.raw' %u", $photo_outdir, $photo_width
        eval "shell python3 hardware/ospi/frame-to-colour.py '%s/photo.raw' %u", $photo_outdir, $photo_width
    else
        printf "  measured width %u is unusable -- raw written, no PNG. Try a width by hand:\n", $photo_width
        printf "    python3 hardware/ospi/frame-to-png.py photo.raw 326\n"
    end
else
    printf "  no pixels dumped\n"
end

# Deliberately NOT resumed (see the header): halted core -> no ESP frames -> motors cut.
monitor halt
printf "\n=== core left HALTED -- the ESP cuts the motors by failsafe; reload to run again ===\n"
set logging enabled off

detach
quit
