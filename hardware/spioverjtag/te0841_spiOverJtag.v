`default_nettype none
// TE0841-specific wrapper around openFPGALoader's spiOverJtag bridge.
//
// The upstream UltraScale bridge has no ports at all: the QSPI signals reach
// the flash through STARTUPE3's dedicated bank-0 pins. That leaves every user
// pin "unused", and Vivado's default BITSTREAM.CONFIG.UNUSEDPIN PULLDOWN then
// pulls down the TE0841's board enables -- including en_osc, which gates the
// module's 200 MHz oscillator. This wrapper drives the two enables high so the
// module stays powered and clocked while the bridge is resident.
//
// pg_ddr (AE20) is deliberately absent: it is a power-good input driven by the
// module's system controller, and must not be driven by the FPGA.
module te0841_spiOverJtag (
	output wire en_osc,
	output wire en_ddr4pwr
);
	assign en_osc     = 1'b1;
	assign en_ddr4pwr = 1'b1;

	spiOverJtag u_bridge ();
endmodule
`default_nettype wire
