# Run one ModelBlaster inference from a --ram-console image and print its console.
#
#   ./rb debug --target arty200t --workload dronet --config <Config> \
#       --ram-console --gdb-script hardware/modelblaster/dronet-capture.gdb \
#       2>&1 | tee <capture>.log
#
# rb debug has already halted the core and loaded the ELF when this runs. It
# adds no `continue` of its own when a script is given, so this script starts
# the program itself.
#
# The harness prints every marker (VERIFY, OUTPUT, PROFILE, WALL_CYCLES) and
# then calls sys_reboot(), which these shells do not implement. So a breakpoint
# on sys_reboot is "the run finished and the markers are in the buffer". A
# fault goes through k_sys_fatal_error_handler instead, after Zephyr has
# printed mcause/mepc into the same buffer. Whichever stops first, the buffer is
# printed, and `rb modelblaster report` can parse the tee'd log directly.
#
# The core is left HALTED at the breakpoint. The image contains no PWM or motor
# code, so there is nothing to stop first.

set pagination off
set confirm off
set print elements 0

break sys_reboot
break k_sys_fatal_error_handler
continue

printf "\n=== ram_console_buf BEGIN ===\n"
printf "%s", ram_console_buf
printf "\n=== ram_console_buf END ===\n"
printf "stopped in: "
frame 0
info registers pc
quit
