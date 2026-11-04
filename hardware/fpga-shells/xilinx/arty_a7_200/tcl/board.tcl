# See LICENSE for license details.
#
# Trenz TE0712 carrying the XC7A200T. The 200T and 100T TE0712 variants are the
# same 484-ball footprint and, verified against Vivado's device database, all
# 484 package balls match on name, bank, PIN_FUNC, clock-capability, VREF/VRP
# and diff-pair type. The constraint set is therefore identical to arty_a7_100
# and only the part string differs.
#
# SPEED GRADE -1, and this is the module on the bench rather than a guess.
# Trenz's own board files name the part for every TE0712 ordering code:
# arty_files/te0712-rd/board_files/TE0712_board_files.csv maps TE0712-02-200-1I
# to `xc7a200tfbg484-1`, and TE0712_200_1I/2.0/board.xml declares the same
# part_name and describes the module as "speed grade -1 and industrial
# temperature grade". Vivado 2023.1 offers no separate industrial part at this
# grade -- `get_parts xc7a200t*fbg484*` returns -1/-2/-2L/-3 plus
# xc7a200tifbg484-1L -- so -1 is what both Trenz and the tool call this die.
#
# This was `xc7a200tfbg484-2` until 2026-08-31, copied from the 100T board's
# commercial -2. That is not a harmless placeholder: it signs timing off
# against a faster speed file than the silicon, and it propagates into the MIG,
# whose generated PHY takes FPGA_SPEED_GRADE as a parameter. The DDR3
# configuration did not calibrate on hardware while it was wrong. Building for
# -1 is also the safe direction if a faster module ever appears on the bench.
set name {arty-a7-200}
set part_fpga {xc7a200tfbg484-1}
set part_board {}
set bootrom_inst {rom}
