# Read-only peripheral register dump over JTAG, hart halted (no load, no resume).
#
#   ./rb debug <workload> --target arty200t --config <Config> \
#       --usb-location <debug-usb> --no-load --no-run \
#       --gdb-script hardware/bench/accel-regs.gdb
#
# Covers the OSPI capture core and its DMA block, uart0 (console), uart1 (ESP)
# and the uart1 TX-DMA block. Use it after hardware/ospi/bootup-capture-halt.gdb
# has stopped at camera_frame_ready to see whether the frame came through the
# DMA engine or through the CPU-popped DATA register, and to read the UART
# divisors a derived clock actually produced.
#
# Two registers are deliberately NOT read, because reading them has a side
# effect: OSPI DATA (0x10080020) pops a pixel, and UART rxdata (+0x04) pops a
# received byte.

set pagination off
set confirm off
monitor halt
printf "\n=== where ===\n"
info registers pc
printf "\n=== OSPI capture DMA (0x10080000) ===\n"
printf "  CTRL      0x00 = 0x%08x\n", *(unsigned int *)0x10080000
printf "  MCLKDIV   0x08 = 0x%08x\n", *(unsigned int *)0x10080008
printf "  FLAGS     0x1c = 0x%08x\n", *(unsigned int *)0x1008001c
printf "  DMA_ADDR_LO 0x44 = 0x%08x\n", *(unsigned int *)0x10080044
printf "  DMA_ADDR_HI 0x48 = 0x%08x\n", *(unsigned int *)0x10080048
printf "  DMA_LEN     0x4c = 0x%08x\n", *(unsigned int *)0x1008004c
printf "  DMA_CTRL    0x50 = 0x%08x   ([0]=enable)\n", *(unsigned int *)0x10080050
printf "  DMA_STATUS  0x54 = 0x%08x   ([0]busy [1]done [2]err [3]sawEof [7:4]state)\n", *(unsigned int *)0x10080054
printf "  DMA_BYTES   0x58 = 0x%08x\n", *(unsigned int *)0x10080058
printf "\n=== uart0 console (0x10020000) ===\n"
printf "  txdata 0x00 = 0x%08x  txctrl 0x08 = 0x%08x  div 0x18 = %u\n", *(unsigned int *)0x10020000, *(unsigned int *)0x10020008, *(unsigned int *)0x10020018
printf "\n=== uart1 ESP (0x10021000) ===\n"
printf "  txdata 0x00 = 0x%08x   ([31]=full)\n", *(unsigned int *)0x10021000
printf "  txctrl 0x08 = 0x%08x   rxctrl 0x0c = 0x%08x\n", *(unsigned int *)0x10021008, *(unsigned int *)0x1002100c
printf "  ie     0x10 = 0x%08x   ip     0x14 = 0x%08x\n", *(unsigned int *)0x10021010, *(unsigned int *)0x10021014
printf "  div    0x18 = %u\n", *(unsigned int *)0x10021018
printf "\n=== uart1 TX DMA (0x10022000) ===\n"
printf "  DMA_ADDR_LO 0x44 = 0x%08x\n", *(unsigned int *)0x10022044
printf "  DMA_ADDR_HI 0x48 = 0x%08x\n", *(unsigned int *)0x10022048
printf "  DMA_LEN     0x4c = 0x%08x\n", *(unsigned int *)0x1002204c
printf "  DMA_CTRL    0x50 = 0x%08x   ([0]=enable)\n", *(unsigned int *)0x10022050
printf "  DMA_STATUS  0x54 = 0x%08x   ([0]busy [1]done [2]err [7:4]state)\n", *(unsigned int *)0x10022054
printf "  DMA_BYTES   0x58 = 0x%08x\n", *(unsigned int *)0x10022058
monitor halt
quit
