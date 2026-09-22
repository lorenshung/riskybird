# ---------------------------------------------------------------------------
# f13probe.xdc - RiskyBird TE0712 (xc7a200tfbg484-2) F13 fault localiser
#
# Pin assignments copied verbatim from the working full design's generated
# shell constraints, same as padtest.xdc, EXCEPT that F13 is an INPUT with an
# internal PULLUP here instead of a PULLDOWN output.
# ---------------------------------------------------------------------------

# ---- 50 MHz differential system clock (H4/G4, DIFF_SSTL15) ----------------
set_property PACKAGE_PIN {H4} [get_ports {sys_clock_p}]
set_property PACKAGE_PIN {G4} [get_ports {sys_clock_n}]
set_property IOSTANDARD {DIFF_SSTL15} [get_ports {sys_clock_p}]
set_property IOSTANDARD {DIFF_SSTL15} [get_ports {sys_clock_n}]

create_clock -name sys_clock -period 20.0 [get_ports {sys_clock_p}]
set_input_jitter sys_clock 0.5

# ---- driven pads: active high into a SI2302 gate, idle low ----------------
# motor1 / E22 : liveness marker, 4 % -> M1 = 40
set_property PACKAGE_PIN {E22} [get_ports {motor1}]
set_property IOSTANDARD {LVCMOS33} [get_ports {motor1}]
set_property PULLDOWN {TRUE} [get_ports {motor1}]

# motor2 / B20 : instability flag, 50 % -> M2 = 500, else 0
set_property PACKAGE_PIN {B20} [get_ports {motor2}]
set_property IOSTANDARD {LVCMOS33} [get_ports {motor2}]
set_property PULLDOWN {TRUE} [get_ports {motor2}]

# motor3 / R17 : static echo of F13 -> M3 = 1000 (read 1) or 0 (read 0)
set_property PACKAGE_PIN {R17} [get_ports {motor3}]
set_property IOSTANDARD {LVCMOS33} [get_ports {motor3}]
set_property PULLDOWN {TRUE} [get_ports {motor3}]

# ---- the pad under test: INPUT with internal PULLUP -----------------------
# F13 is NOT driven by this design.  The on-die pullup fights the carrier's
# 10k gate pulldown; a clean 1 therefore means the net is OPEN.
set_property PACKAGE_PIN {F13} [get_ports {motor4_in}]
set_property IOSTANDARD {LVCMOS33} [get_ports {motor4_in}]
set_property PULLUP {TRUE} [get_ports {motor4_in}]

# The input is asynchronous and goes straight into a 3-stage synchroniser.
set_false_path -from [get_ports {motor4_in}]

# ---- configuration / safety ------------------------------------------------
set_property BITSTREAM.CONFIG.UNUSEDPIN PULLDOWN [current_design]
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
