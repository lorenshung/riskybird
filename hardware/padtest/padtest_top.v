// ---------------------------------------------------------------------------
// padtest_top.v
//
// Minimal bare-Verilog pad connectivity test for the RiskyBird TE0712 carrier.
// No Rocket, no TileLink, no Chipyard, no Zephyr - just IBUFDS + counters.
//
// Purpose: prove (or disprove) that ball F13 (motor4) can drive at all on this
// board, independent of the full design.  All four motor pads are driven
// continuously at four DISTINCT low duty cycles so the ESP32-C6 high-impedance
// gate probe can identify each pad unambiguously in a single reading.
//
//   motor1 -> E22 ->  4%  -> expect ~40  per mille
//   motor2 -> B20 ->  8%  -> expect ~80  per mille
//   motor3 -> R17 -> 12%  -> expect ~120 per mille
//   motor4 -> F13 -> 16%  -> expect ~160 per mille
//
// SAFETY: outputs are ACTIVE HIGH and reset to 0.  There is no inversion
// anywhere in this file.  Every output flop declares an initial value of 0, so
// its INIT is 0 in the bitstream and GSR releases it to 0.  In addition an
// `armed` flop (also INIT 0) gates all four outputs and is only set after a
// ~1 second startup counter expires, so the pads are held at a hard 0 for one
// full second after configuration before any PWM edge appears.  Combined with
// the board's 10k gate pulldown and PULLDOWN TRUE in the XDC, the pads are LOW
// from power-on, through configuration, and for ~1 s afterwards.
//
// Input clock: 50 MHz differential on H4/G4 (DIFF_SSTL15).
// ---------------------------------------------------------------------------

`timescale 1ns / 1ps

module padtest_top (
    input  wire sys_clock_p,
    input  wire sys_clock_n,
    output wire motor1,
    output wire motor2,
    output wire motor3,
    output wire motor4
);

    // ---- 50 MHz differential system clock -------------------------------
    wire clk;

    IBUFDS #(
        .DIFF_TERM    ("FALSE"),
        .IBUF_LOW_PWR ("FALSE"),
        .IOSTANDARD   ("DIFF_SSTL15")
    ) ibufds_sys_clock (
        .O  (clk),
        .I  (sys_clock_p),
        .IB (sys_clock_n)
    );

    // ---- timing constants -----------------------------------------------
    // 50 MHz / 2500 = 20.000 kHz PWM, matching the real motor drive frequency.
    localparam integer PWM_PERIOD     = 2500;
    localparam integer DUTY_MOTOR1    =  100;   //  4.0 %
    localparam integer DUTY_MOTOR2    =  200;   //  8.0 %
    localparam integer DUTY_MOTOR3    =  300;   // 12.0 %
    localparam integer DUTY_MOTOR4    =  400;   // 16.0 %
    // 50_000_000 cycles @ 50 MHz = 1.000 s of guaranteed-low output.
    localparam integer STARTUP_CYCLES = 50000000;

    // ---- startup hold-off: outputs forced low for ~1 s after config ------
    reg [25:0] startup_cnt = 26'd0;
    reg        armed       = 1'b0;

    always @(posedge clk) begin
        if (startup_cnt == (STARTUP_CYCLES - 1)) begin
            armed <= 1'b1;
        end else begin
            startup_cnt <= startup_cnt + 1'b1;
        end
    end

    // ---- free running PWM counter ---------------------------------------
    reg [11:0] pwm_cnt = 12'd0;

    always @(posedge clk) begin
        if (pwm_cnt == (PWM_PERIOD - 1))
            pwm_cnt <= 12'd0;
        else
            pwm_cnt <= pwm_cnt + 1'b1;
    end

    // ---- output flops: active high, INIT 0, gated by `armed` ------------
    (* IOB = "TRUE" *) reg motor1_q = 1'b0;
    (* IOB = "TRUE" *) reg motor2_q = 1'b0;
    (* IOB = "TRUE" *) reg motor3_q = 1'b0;
    (* IOB = "TRUE" *) reg motor4_q = 1'b0;

    always @(posedge clk) begin
        motor1_q <= armed & (pwm_cnt < DUTY_MOTOR1);
        motor2_q <= armed & (pwm_cnt < DUTY_MOTOR2);
        motor3_q <= armed & (pwm_cnt < DUTY_MOTOR3);
        motor4_q <= armed & (pwm_cnt < DUTY_MOTOR4);
    end

    assign motor1 = motor1_q;
    assign motor2 = motor2_q;
    assign motor3 = motor3_q;
    assign motor4 = motor4_q;

endmodule
