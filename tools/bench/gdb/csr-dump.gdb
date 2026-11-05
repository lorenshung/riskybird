# Read-only machine-mode CSR snapshot, hart halted (no load, no resume).
#
#   ./rb debug <workload> --target arty200t --config <Config> \
#       --usb-location <debug-usb> --no-load --no-run \
#       --gdb-script tools/bench/gdb/csr-dump.gdb
#
# Used to compare what one image leaves behind against another: trap state,
# translation (satp), PMP, delegation. Leaves the hart halted.

set pagination off
set confirm off
monitor halt
printf "\n=== CSR snapshot ===\n"
info registers pc
info registers mstatus mcause mepc mtval mie mip mtvec satp misa
info registers pmpcfg0 pmpcfg2 pmpaddr0 pmpaddr1 pmpaddr2 pmpaddr3 pmpaddr4 pmpaddr5 pmpaddr6 pmpaddr7
info registers mcounteren mscratch medeleg mideleg
monitor halt
quit
