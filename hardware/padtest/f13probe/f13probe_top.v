// ---------------------------------------------------------------------------
// f13probe_top.v
//
// Minimal bare-Verilog F13 fault localiser for the RiskyBird TE0712 carrier.
// No Rocket, no TileLink, no Chipyard, no Zephyr.
//
// Follow-up to padtest_top.v, which proved F13 (motor4) drives NOTHING while
// E22/B20/R17 hit 40/80/120 per mille exactly.  That test could not tell a dead
// F13 OUTPUT DRIVER from an OPEN NET between the F13 ball and the SI2302 gate.
// This design discriminates the two.
//
// Method: F13 becomes an INPUT with the FPGA's INTERNAL PULLUP enabled, and we
// echo what it reads onto the known-good pads so the existing ESP gate probe
// reads the answer with no ESP firmware change.
//
//   Every motor gate node on the carrier has a 10k pulldown to ground.
//     - F13 OPEN  -> internal pullup wins outright   -> F13 reads a solid 1
//     - F13 WIRED -> internal pullup (~7k..45k) fights the 10k board pulldown,
//                    so the node sits at/below mid-rail, well under LVCMOS33
//                    VIH(min)=2.0 V  -> F13 reads 0, or chatters
//
//   motor1 (E22) = 4 % PWM              -> M1 ~ 40    liveness marker
//   motor2 (B20) = 50 % PWM if unstable -> M2 ~ 500   F13 toggled in last 0.5 s
//                  else static 0        -> M2 = 0     F13 is a stable level
//   motor3 (R17) = static sync'd F13    -> M3 = 1000  F13 read 1  (OPEN NET)
//                                       -> M3 = 0     F13 read 0  (NET INTACT)
//   motor4 (F13) = INPUT, not driven at all.
//
// SAFETY: identical property to padtest_top.  The three DRIVEN pads are active
// high with no inversion anywhere, every flop declares an initial value of 0 so
// its INIT is 0 in the bitstream, and an `armed` flop (also INIT 0) gates all
// three outputs until a ~1 second startup counter expires.  Pads are a hard 0
// from configuration through the first second.  F13 is never driven by this
// design at all, so motor 4 cannot turn on under any circumstance here.
//
// Input clock: 50 MHz differential on H4/G4 (DIFF_SSTL15).
// ---------------------------------------------------------------------------

`timescale 1ns / 1ps

module f13probe_top (
    input  wire sys_clock_p,
    input  wire sys_clock_n,
    output wire motor1,      // E22 - liveness, 4 %
    output wire motor2,      // B20 - instability flag, 50 % or 0
    output wire motor3,      // R17 - echo of F13, static 1 or 0
    input  wire motor4_in    // F13 - the pad under test, INPUT + PULLUP
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

    // ---- F13 as an input -------------------------------------------------
    // Plain IBUF.  The pullup is applied by PULLTYPE/PULLUP in the XDC, which
    // is what actually arms the on-die resistor in the IOB.
    wire f13_raw;

    IBUF #(
        .IBUF_LOW_PWR ("FALSE"),
        .IOSTANDARD   ("LVCMOS33")
    ) ibuf_f13 (
        .O (f13_raw),
        .I (motor4_in)
    );

    // ---- timing constants -----------------------------------------------
    localparam integer PWM_PERIOD     = 2500;       // 50 MHz / 2500 = 20.000 kHz
    localparam integer DUTY_LIVENESS  =  100;       //  4.0 % -> M1 =  40
    localparam integer DUTY_UNSTABLE  = 1250;       // 50.0 % -> M2 = 500
    localparam integer STARTUP_CYCLES = 50000000;   // 1.000 s @ 50 MHz
    localparam integer TOGGLE_HOLD    = 25000000;   // 0.500 s @ 50 MHz

    // ---- startup hold-off: outputs forced low for ~1 s after config ------
    reg [25:0] startup_cnt = 26'd0;
    reg        armed       = 1'b0;

    always @(posedge clk) begin
        if (startup_cnt == (STARTUP_CYCLES - 1))
            armed <= 1'b1;
        else
            startup_cnt <= startup_cnt + 1'b1;
    end

    // ---- free running PWM counter ---------------------------------------
    reg [11:0] pwm_cnt = 12'd0;

    always @(posedge clk) begin
        if (pwm_cnt == (PWM_PERIOD - 1))
            pwm_cnt <= 12'd0;
        else
            pwm_cnt <= pwm_cnt + 1'b1;
    end

    // ---- 3-stage synchroniser on the asynchronous F13 input ---------------
    // Requirement is "at least 2 flops"; three are used so the value feeding
    // the echo output is one stage past the metastability-resolving pair.
    (* ASYNC_REG = "TRUE" *) reg f13_sync0 = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg f13_sync1 = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg f13_sync2 = 1'b0;
                             reg f13_sync3 = 1'b0;   // 1-cycle delay for edge detect

    always @(posedge clk) begin
        f13_sync0 <= f13_raw;
        f13_sync1 <= f13_sync0;
        f13_sync2 <= f13_sync1;
        f13_sync3 <= f13_sync2;
    end

    wire f13_changed = (f13_sync2 != f13_sync3);

    // ---- "disagreed with itself in the last ~0.5 s" -----------------------
    // Held in reset until `armed`, so power-on settling of the pullup cannot
    // be mistaken for a floating input.
    reg [24:0] tog_cnt    = 25'd0;
    reg        tog_active = 1'b0;

    always @(posedge clk) begin
        if (!armed) begin
            tog_cnt    <= 25'd0;
            tog_active <= 1'b0;
        end else if (f13_changed) begin
            tog_cnt    <= TOGGLE_HOLD - 1;
            tog_active <= 1'b1;
        end else if (tog_cnt != 25'd0) begin
            tog_cnt    <= tog_cnt - 1'b1;
        end else begin
            tog_active <= 1'b0;
        end
    end

    // ---- output flops: active high, INIT 0, gated by `armed` ------------
    (* IOB = "TRUE" *) reg motor1_q = 1'b0;
    (* IOB = "TRUE" *) reg motor2_q = 1'b0;
    (* IOB = "TRUE" *) reg motor3_q = 1'b0;

    always @(posedge clk) begin
        motor1_q <= armed & (pwm_cnt < DUTY_LIVENESS);
        motor2_q <= armed & tog_active & (pwm_cnt < DUTY_UNSTABLE);
        motor3_q <= armed & f13_sync2;
    end

    assign motor1 = motor1_q;
    assign motor2 = motor2_q;
    assign motor3 = motor3_q;

endmodule
