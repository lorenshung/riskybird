# RiskyBird additions to chipyard/fpga/Makefile, extracted from chipyard 7346121.
# The originals still live in the backend's fpga/Makefile until its pin moves to
# the plain tip; not wired up yet (make -C fpga -f rb-fpga.mk is the plan).

ifeq ($(SUB_PROJECT),arty200t)
	# Trenz TE0712 carrying the XC7A200T. Same carrier and same 484-ball
	# footprint as arty100t; the part string in
	# fpga-shells/xilinx/arty_a7_200/tcl/board.tcl is the only physical
	# difference.
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
#########################################################################################
# RiskyBird additions
#
# All RiskyBird-owned collateral in this Chipyard checkout lives in
# $(base_dir)/riskybird/. This is the only hook. See that directory's README.md.
# Included last so every variable above (build_dir, synth_list_f, MODEL, BOARD,
# CONFIG, SUB_PROJECT, FPGA_BRAND) is already defined.
#########################################################################################
-include $(base_dir)/riskybird/area.mk

