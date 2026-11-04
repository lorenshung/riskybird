# RiskyBird FPGA area-characterization flow.
#
# This file is RiskyBird-owned collateral living inside the Chipyard backend.
# It is hooked into the backend by exactly one line at the end of fpga/Makefile:
#
#     -include $(base_dir)/riskybird/area.mk
#
# Everything else RiskyBird adds to Chipyard's FPGA flow lives in this
# directory. See ./README.md.
#
# Targets:
#   rb-area            synthesize only, emit a hierarchical utilization report
#   rb-area-force      re-run rb-area even if the report is up to date
#   rb-area-restore    undo any RTL attribute injection in gen-collateral
#
# Usage (from the repository root, through the RiskyBird CLI):
#   rb area --board ku040
#   rb area --board ku040 --config SaturnKU040OspiConfig --attrs hierarchy
#
# Usage (directly):
#   make -C backends/chipyard/fpga SUB_PROJECT=ku040 \
#        CONFIG=RocketKU040OspiConfig RB_ATTRS=none rb-area

rb_dir := $(base_dir)/riskybird

# Which RTL attribute injections to apply to gen-collateral before synthesis.
#   none       pristine generated RTL (default)
#   hierarchy  keep_hierarchy for faithful per-module attribution
#   dsp        use_dsp on MAC/OPU/hardfloat multipliers
#   all        hierarchy + dsp
#
# Injection edits the generated Verilog in place. It is recorded in the run
# manifest so a measurement is never silently attributed to pristine RTL.
# keep_hierarchy inhibits cross-boundary optimization, so totals from a
# `hierarchy` or `all` run are an upper bound and must not share an axis with
# `none` totals.
RB_ATTRS ?= none

# The report that marks a completed area run. rb_manifest.json is written
# alongside it and carries the provenance.
RB_AREA_DIR    := $(build_dir)/obj/report
RB_AREA_REPORT := $(RB_AREA_DIR)/rb_utilization_hier.txt

$(RB_AREA_REPORT): $(synth_list_f)
ifeq ($(FPGA_BRAND),xilinx)
	cd $(build_dir); \
	RB_ATTRS="$(RB_ATTRS)" \
	RB_CONFIG="$(CONFIG)" \
	RB_SUB_PROJECT="$(SUB_PROJECT)" \
	RB_BOARD="$(BOARD)" \
	RB_MODEL="$(MODEL)" \
	vivado \
		-nojournal -mode batch \
		-source $(rb_dir)/tcl/rb_synth_area.tcl \
		-tclargs \
			-top-module "$(MODEL)" \
			-F "$(synth_list_f)" \
			-board "$(BOARD)" \
			-ip-vivado-tcls "$(shell find '$(build_dir)' -name '*.vivado.tcl')"
else
	$(error rb-area requires FPGA_BRAND=xilinx, got '$(FPGA_BRAND)' for SUB_PROJECT=$(SUB_PROJECT))
endif

# Place and route the checkpoint rb-area already produced, then report timing
# and power. Deliberately starts from obj/post_synth.dcp rather than re-running
# the flow: re-elaboration would discard the gen-collateral attribute injection
# and route a different design from the one whose area was recorded.
RB_IMPL_CHECKPOINT := $(build_dir)/obj/post_synth.dcp

.PHONY: rb-impl
rb-impl:
	@test -f $(RB_IMPL_CHECKPOINT) || { \
	  echo "rb-impl: no checkpoint at $(RB_IMPL_CHECKPOINT)"; \
	  echo "rb-impl: run 'rb area run --board $(BOARD) --config $(CONFIG)' first"; \
	  exit 1; }
	cd $(build_dir); \
	RB_CONFIG="$(CONFIG)" \
	RB_SUB_PROJECT="$(SUB_PROJECT)" \
	RB_BOARD="$(BOARD)" \
	vivado \
		-nojournal -mode batch \
		-source $(rb_dir)/tcl/rb_impl_timing.tcl \
		-tclargs -checkpoint $(RB_IMPL_CHECKPOINT)

.PHONY: rb-area
rb-area: $(RB_AREA_REPORT)

.PHONY: rb-area-force
rb-area-force:
	rm -f $(RB_AREA_REPORT)
	+$(MAKE) rb-area

# Restore every file that attribute injection modified. Injection writes a
# <file>.rb-orig sibling before its first edit, so this is exact rather than a
# best-effort regex reversal.
.PHONY: rb-area-restore
rb-area-restore:
	@found=0; \
	for f in $$(find '$(build_dir)/gen-collateral' -name '*.rb-orig' 2>/dev/null); do \
		mv -f "$$f" "$${f%.rb-orig}"; \
		echo "restored $${f%.rb-orig}"; \
		found=1; \
	done; \
	if [ $$found -eq 0 ]; then echo "no injected files to restore under $(build_dir)/gen-collateral"; fi
