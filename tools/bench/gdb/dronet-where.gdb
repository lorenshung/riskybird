# Where is a DroNet image that never reached sys_reboot or the fatal handler?
#
#   ./rb debug dronet --target arty200t --config <Config> --ram-console \
#       --usb-location <debug-usb> --no-load --no-run \
#       --gdb-script tools/bench/gdb/dronet-where.gdb
#
# The companion of hardware/modelblaster/dronet-capture.gdb for a run that hit
# neither of its breakpoints. Attaches without loading, halts, and reports pc,
# a backtrace, the trap CSRs and the RAM console. Leaves the hart halted.

set pagination off
set confirm off
set print elements 0
monitor halt
printf "\n=== where ===\n"
info registers pc
bt 8
printf "\n=== trap CSRs ===\n"
info registers mcause mepc mtval mstatus mie mip satp pmpcfg0 pmpaddr0 pmpaddr1 pmpaddr2 pmpaddr3
printf "\n=== ram_console_buf BEGIN ===\n"
printf "%s", ram_console_buf
printf "\n=== ram_console_buf END ===\n"
monitor halt
quit
