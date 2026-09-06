# spiOverJtag constraints for the Trenz TE0841 module: xcku040-sfva784-1-c.
#
# The primary QSPI flash needs no pin constraints. On UltraScale it is reached
# through the STARTUPE3 primitive, which drives the dedicated bank-0
# configuration pins (D00_MOSI/D01_DIN/D02/D03, RDWR_FCS_B, CCLK). Those are
# not user I/O and take no placement constraints, which is why this bridge is
# buildable for a package upstream has never targeted.
#
# Do not add the secondary-flash block from upstream's
# constr_xcku040_ffva1156.xdc: it places user I/O in bank 65, which on the
# TE0841 carries board housekeeping. That is also why the ffva1156 prebuilt
# bitstream must not be loaded on this package.

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]

# UG917 Table 1-5. The TE0841's configuration flash is a 1.8 V part
# (n25q256-1.8v per the Trenz board files), so bank 0 is 1.8 V and CFGBVS ties
# to GND. These are the values upstream's KU040 constraint file asserts.
set_property CONFIG_VOLTAGE 1.8 [current_design]
set_property CFGBVS GND [current_design]

# Leave every unconstrained user pin floating rather than pulled down, so the
# module's system controller keeps its own ties.
set_property BITSTREAM.CONFIG.UNUSEDPIN PULLNONE [current_design]

# Board enables, held high by te0841_spiOverJtag.v. Pins and I/O standard match
# the KU040 harness shell XDC.
set_property PACKAGE_PIN AF24 [get_ports {en_osc}]
set_property IOSTANDARD LVCMOS33 [get_ports {en_osc}]
set_property PACKAGE_PIN AD20 [get_ports {en_ddr4pwr}]
set_property IOSTANDARD LVCMOS33 [get_ports {en_ddr4pwr}]
