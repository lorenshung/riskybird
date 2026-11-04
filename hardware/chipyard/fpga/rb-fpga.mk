#########################################################################################
# RiskyBird wrapper around a plain Chipyard's fpga/Makefile.
#
#   make -C backends/chipyard/fpga -f $(RB_ROOT)/hardware/chipyard/fpga/rb-fpga.mk \
#        SUB_PROJECT=arty200t CONFIG=RocketArty200TDroneFullDDRConfig verilog
#
# (`rb chipyard verilog|bitstream` and `rb area` run exactly that.) It adds the
# RiskyBird sub-projects, then includes the stock Makefile, then RiskyBird's
# area/impl flow. The Scala for these sub-projects reaches Chipyard through the
# sbt plugin hardware/chipyard/project/RiskyBird.scala; the Vivado board directories
# through the links in .backend-overlays. -C fpga is required: the stock
# Makefile takes base_dir from the working directory.
#
# Every assignment is ?=, so a value given on the command line wins, and the
# stock Makefile's own blocks (also ?=) cannot override these.
#########################################################################################

rb_chipyard_dir := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))..)

ifeq ($(SUB_PROJECT),arty200t)
	# Trenz TE0712 carrying the XC7A200T. Same carrier and same 484-ball
	# footprint as arty100t; the part string in
	# hardware/fpga-shells/xilinx/arty_a7_200/tcl/board.tcl is the only
	# physical difference.
	SBT_PROJECT       ?= chipyard_fpga
	MODEL             ?= Arty200THarness
	VLOG_MODEL        ?= Arty200THarness
	MODEL_PACKAGE     ?= chipyard.fpga.arty200t
	CONFIG            ?= RocketArty200TConfig
	CONFIG_PACKAGE    ?= chipyard.fpga.arty200t
	GENERATOR_PACKAGE ?= chipyard
	TB                ?= none # unused
	TOP               ?= ChipTop
	BOARD             ?= arty_a7_200
	FPGA_BRAND        ?= xilinx
endif
ifeq ($(SUB_PROJECT),ku040)
	SBT_PROJECT       ?= chipyard_fpga
	MODEL             ?= KU040Harness
	VLOG_MODEL        ?= KU040Harness
	MODEL_PACKAGE     ?= chipyard.fpga.ku040
	# Default to the board's real 2 GiB of DDR4. The scratchpad-only
	# configurations (RocketKU040Config and the accelerator area probes) are
	# still selectable with an explicit CONFIG=; they have to be, because two
	# DDR4 controllers cost ~22k LUT and the probes already sit above 90%.
	CONFIG            ?= RocketKU040DDRConfig
	CONFIG_PACKAGE    ?= chipyard.fpga.ku040
	GENERATOR_PACKAGE ?= chipyard
	TB                ?= none # unused
	TOP               ?= ChipTop
	BOARD             ?= ku040
	FPGA_BRAND        ?= xilinx
endif
ifeq ($(SUB_PROJECT),arty100t)
	# The stock arty100t sub-project, on the TE0712-100T: its Scala (Arty100T
	# harness, binders, configs and fpga-shells' Arty100TShell/MIG) is
	# shadowed by hardware/, and its Vivado board files are the reworked
	# arty_a7_100 directory, linked in as te0712_100t.
	BOARD             ?= te0712_100t
endif

include Makefile

include $(rb_chipyard_dir)/riskybird/area.mk

# Chipyard rebuilds its jar when one of its own sources changes; make it see
# the overlay's too (the AutoPlugin in hardware/chipyard/project compiles them).
rb_root := $(abspath $(rb_chipyard_dir)/../..)
rb_overlay_sources := $(shell find $(rb_root)/hardware/chipyard $(rb_root)/hardware/rtl \
	$(rb_root)/hardware/fpga-shells/src $(rb_root)/hardware/gemmini \
	-path '*/recovery' -prune -o \( -name '*.scala' -o -name '*.sv' -o -name '*.hex' \) -print)
$(GENERATOR_CLASSPATH): $(rb_overlay_sources)
