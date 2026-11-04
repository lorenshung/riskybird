# See LICENSE for license details.
#
# Configuration and SPI-flash boot properties for the Trenz TE0712.
#
# Every line of this file used to be commented out, which is not the same as
# the file being absent: prologue.tcl globs `constraints/*.xdc` into the
# constraint set, so the file was read on every build and contributed nothing.
# The visible symptom was `WARNING: [DRC CFGBVS-1] Missing CFGBVS and
# CONFIG_VOLTAGE Design Properties` in every arty200t Vivado log.
#
# The invisible symptom was worse. The part programs perfectly over JTAG but
# never boots from its own flash: the image is read back byte-identical, yet
# after a power cycle the configuration status register reads
#
#     STAT = 0x5000190c  -> DONE=0 EOS=0 STARTUP=0 CRC_ERR=0 MODE=001
#
# Master SPI is selected and initialisation completed, no corruption is
# reported, and the startup sequencer simply never runs -- the FPGA never
# finds the 0xAA995566 sync word. Because the fabric never comes up, the
# Rocket debug TAP on the J1 user-I/O pins does not exist, and every OpenOCD
# run against it reports `all ones` / `IR capture error` / `DTM version 15`,
# which reads like a broken cable and is not one. The bitstream JTAG link
# keeps working throughout, because it talks to the FPGA's dedicated config
# TAP, which is alive whenever the part has power.
#
# The values below are Trenz's own, copied from the TE0712 reference design at
# arty_files/te0712-rd/constraints/_i_bitgen_common.xdc rather than inferred.
# They are what the module is validated with.
#
# SPI_32BIT_ADDR is the one that is not cosmetic. The TE0712 carries a 256 Mbit
# (32 MB) Spansion S25FL256S -- confirmed on the bench, openFPGALoader reports
# `spansion S25FL256S 512 sectors size: 256Mb`. That is past the 16 MB a 3-byte
# address can reach, and the 4-byte instruction set is what Trenz configures
# the part for. A master issuing 3-byte reads lands at the wrong offset, never
# sees the sync word, and waits forever, which is precisely the signature
# above.
#
# CONFIGRATE 66 on a x4 bus also matters in practice: it takes a 9.7 MB image
# from roughly 26 s at the 3 MHz default down to well under a second. A board
# that takes half a minute to come up after a power cycle is indistinguishable
# from a board that never comes up, and gets debugged as the latter.
#
# If flash boot is ever marginal, back CONFIGRATE down the ladder 66 -> 33 ->
# 22 -> 16 before changing anything else, and try
# BITSTREAM.CONFIG.SPI_FALL_EDGE {YES}, which buys half a clock of margin.
# SPI x1, not x4: the module's S25FL256S ships with CR1.QUAD = 0, and in x4
# the FPGA switches to quad reads after the header and gets garbage on IO2/IO3.
# Measured 2026-11-01: x4 images never configured (STAT 0x5000190c, DONE=0; an
# IPROG into one fell back to golden), the same image regenerated for x1 did.
# Vivado's flash programmer would set QUAD; openFPGALoader 0.x here cannot, and
# a compressed ~5 MB image loads in well under a second at x1 anyway.
#
# Remote updates (RiskyBird ESP link) keep two images in this flash: the golden
# one at 0 and an update slot at 8 MiB, entered by ICAPE2 IPROG. COMPRESS keeps
# each image well under 8 MiB (and shortens both the remote write and the boot),
# and CONFIGFALLBACK makes a failed IPROG load (CRC or ID error) fall back to
# the golden image instead of leaving the FPGA unconfigured. USR_ACCESS
# TIMESTAMP is what the ESP link reports as the image identity.
set_property -dict [list \
	CFGBVS {VCCO} \
	CONFIG_VOLTAGE {3.3} \
	CONFIG_MODE {SPIx1} \
	BITSTREAM.CONFIG.SPI_BUSWIDTH {1} \
	BITSTREAM.CONFIG.SPI_32BIT_ADDR {YES} \
	BITSTREAM.CONFIG.CONFIGRATE {66} \
	BITSTREAM.CONFIG.M0PIN {PULLNONE} \
	BITSTREAM.CONFIG.M1PIN {PULLNONE} \
	BITSTREAM.CONFIG.M2PIN {PULLNONE} \
	BITSTREAM.CONFIG.USR_ACCESS {TIMESTAMP} \
	BITSTREAM.GENERAL.COMPRESS {TRUE} \
	BITSTREAM.CONFIG.CONFIGFALLBACK {ENABLE} \
	] [current_design]
