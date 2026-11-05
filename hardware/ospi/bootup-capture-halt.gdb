# Run bootup_check to its end anchor, read out BOTH the RAM console report and
# the camera frame over JTAG, and leave the core HALTED.
#
#   timeout 300 ./rb debug bootup_check --target arty200t \
#       --config <Config> --usb-location <debug-usb> \
#       --ram-console --gdb-script hardware/ospi/bootup-capture-halt.gdb
#
# Same flow as bootup-capture.gdb, except:
#   - ram_console_buf is printed and dumped, so the report does not depend on
#     attaching the serial console in time (build with --ram-console);
#   - the outputs are capture-ramconsole.bin and capture-frame.raw, not
#     bootup-frame.*, so a run with this script never overwrites the other's;
#   - no `detach`: the core stays halted at camera_frame_ready, which lets
#     tools/bench/gdb/accel-regs.gdb read the capture core afterwards.
#
# Output files land in the directory GDB runs in, which for `rb debug` is the
# repository root. Move or rename them between runs.
#
# `continue` below is what starts the core (rb debug adds no `continue` when a
# --gdb-script is given). It returns when the breakpoint is hit.

set pagination off
set confirm off

break camera_frame_ready
continue

printf "\n=== halted at ===\n"
info registers pc

printf "\n=== ram console ===\n"
dump binary value capture-ramconsole.bin ram_console_buf
printf "%s", ram_console_buf
printf "\n=== end ram console ===\n"

printf "\n=== capture core ===\n"
printf "  measured   %u x %u\n", *(unsigned int *)0x10080014, *(unsigned int *)0x10080018
printf "  configured %u x %u\n", \
    *(unsigned int *)0x10080004 & 0xffff, (*(unsigned int *)0x10080004 >> 16) & 0xffff
printf "  pclk=%u fvld=%u lvld=%u frames=%u\n", \
    *(unsigned int *)0x10080030, *(unsigned int *)0x10080034, \
    *(unsigned int *)0x10080038, *(unsigned int *)0x10080010
printf "  flags=0x%08x capstat=0x%08x\n", \
    *(unsigned int *)0x1008001c, *(unsigned int *)0x10080040
printf "  capacity=%u pixels; pixtarget=%u; capcount=%u\n", \
    *(unsigned int *)0x10080024, *(unsigned int *)0x10080028, \
    *(unsigned int *)0x1008002c

printf "\n=== frame ===\n"
printf "  valid bytes in pixels[]: %u\n", camera_frame_pixels
if camera_frame_pixels > 0
    dump binary memory capture-frame.raw &pixels[0] &pixels[0] + camera_frame_pixels
    printf "  wrote capture-frame.raw\n"
else
    printf "  camera stage captured nothing -- no files written\n"
end

# Leave the hart halted. OpenOCD is terminated by rb after GDB exits; the
# target is not resumed.
monitor halt
quit
