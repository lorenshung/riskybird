// ===========================================================================
// fc_accel.sv  --  Hardened Flight-Controller RoCC accelerator (OOC synth witness)
//
// A shared fixed-point datapath + microsequencer that runs the riskybird v3
// estimator (Mahony attitude + complementary filter) and the hierarchical PID
// cascade + 4x4 mixer + force->duty as ONE fixed-function pipeline, so the FC
// loop no longer runs on the scalar core or the Saturn vector unit.
//
// This is the *synthesis witness*: a self-contained, chipyard-independent RTL
// that is microarchitecturally identical to FcRoCC.scala (same register file,
// same single shared multiplier, same Newton-Raphson recip/rsqrt/sqrt, same
// microsequencer width and op-count). It exists so `synth_design -mode
// out_of_context` produces a truthful LUT/FF/DSP estimate for the accelerator
// ALONE without elaborating the whole SoC. See README.md for the equivalence
// argument.
//
// Arithmetic (chosen for minimum LUT, justified in README/design doc):
//   * ONE global fixed-point format Q24.24 (48-bit signed).
//       - range +/-8.4e6  (covers the force-curve discriminant ~65838)
//       - resolution 6.0e-8  (>= fp32 mantissa near 1.0, so the drift-sensitive
//         accumulators -- quaternion, velocity, position, PID integrators --
//         meet the fp32-equivalent precision boundary the fp16 Spike/pybullet
//         harness identified).
//   * A single shared, pipelined signed 48x48 multiplier (DSP-mapped) time-
//       multiplexed across every product in the loop (~150 mults).
//   * No hardware divider / no big FP unit: reciprocal, rsqrt and sqrt are
//       Newton-Raphson iterations expressed as microcode on the shared
//       multiplier (seeded by a leading-zero-count guess).
// ===========================================================================
`default_nettype none

module fc_accel #(
    parameter int W      = 48,     // Q24.24 datapath width
    parameter int QF     = 24,     // fractional bits
    parameter int NREG   = 64,     // register file depth (state+temp+in+out+const)
    parameter int NUOP   = 384,    // microprogram length (>= measured op count)
    parameter int UAW    = 9,      // ceil(log2(NUOP))
    parameter int RAW    = 6       // ceil(log2(NREG))
) (
    input  wire              clk,
    input  wire              rst,          // synchronous, active high

    // ---- RoCC-style command / response (mirrors RoCCCoreIO cmd/resp/busy) ----
    input  wire              cmd_valid,
    output wire              cmd_ready,
    input  wire [6:0]        cmd_funct,    // 0=PUSH input, 1=RUN, 2=POP output, 3=CFG const
    input  wire [63:0]       cmd_rs1,      // PUSH/POP/CFG: index ; value in rs2
    input  wire [63:0]       cmd_rs2,      // value (sign-extended Q24.24 in low 48b)
    output reg               resp_valid,
    input  wire              resp_ready,
    output reg  [63:0]       resp_data,
    output wire              busy
);

    // -----------------------------------------------------------------------
    // Register file: 3 read ports (a,b,c for MAC) + 1 write.  Written
    // synchronously, read asynchronously -> Vivado infers distributed RAM
    // (LUTRAM), the low-LUT-*logic* choice (keeps the operand path off the
    // general fabric).  On a chipyard build this is the Chisel `Mem`.
    // -----------------------------------------------------------------------
    (* ram_style = "distributed" *) reg signed [W-1:0] rf [0:NREG-1];

    reg  [RAW-1:0] ra, rb, rc, rw;
    reg  [W-1:0]   wdata;
    reg            we;
    always @(posedge clk) if (we) rf[rw] <= wdata;
    wire signed [W-1:0] opa = rf[ra];
    wire signed [W-1:0] opb = rf[rb];
    wire signed [W-1:0] opc = rf[rc];

    // -----------------------------------------------------------------------
    // Microcode control store.  Fields:
    //   [3:0]  aluop   0 MUL  1 MAC(a*b+c)  2 ADD  3 SUB  4 MIN  5 MAX
    //                  6 MOV  7 NEG  8 ABS  9 RSQRT_SEED 10 LZC 11 ASR(imm)
    //                  12 SEL(c?a:b sign) 13 HALT
    //   [3:0]  next control (loop-back / branch class for NR iterations)
    //   [RAW*4] a,b,c,w register selects
    // The exact program is not needed for AREA; it is initialised to a value-
    // dependent pattern (so no field folds to a constant) and every field
    // drives the datapath, giving a representative control-logic estimate.
    // On a chipyard build this ROM holds the real, verified op schedule.
    // -----------------------------------------------------------------------
    localparam int UW = 4 + 4 + 4*RAW;         // microword width
    (* rom_style = "block" *) reg [UW-1:0] ucode [0:NUOP-1];
    integer i;
    initial begin
        for (i = 0; i < NUOP; i = i + 1) begin
            // deterministic non-constant fill: exercises full decode + all
            // read/write selects across the whole regfile.
            ucode[i] = { 4'(i),                        // aluop
                         4'(i*7),                      // nextctl
                         RAW'(i*3),                    // a
                         RAW'(i*5 + 1),                // b
                         RAW'(i*11 + 2),               // c
                         RAW'(i*13 + 3) };             // w
        end
    end

    reg  [UAW-1:0] upc;
    wire [UW-1:0]  uw   = ucode[upc];
    wire [3:0]     aluop   = uw[UW-1 -: 4];
    wire [3:0]     nextctl = uw[UW-5 -: 4];
    wire [RAW-1:0] f_a = uw[4*RAW-1 -: RAW];
    wire [RAW-1:0] f_b = uw[3*RAW-1 -: RAW];
    wire [RAW-1:0] f_c = uw[2*RAW-1 -: RAW];
    wire [RAW-1:0] f_w = uw[1*RAW-1 -: RAW];

    // -----------------------------------------------------------------------
    // Shared pipelined signed 48x48 multiplier (DSP-mapped, 3 stages).
    // Time-multiplexed across every product AND every NR iteration.
    // -----------------------------------------------------------------------
    (* use_dsp = "yes" *) reg signed [2*W-1:0] p_s2;
    reg signed [W-1:0] m_a1, m_b1;
    reg signed [W-1:0] macc1;                       // MAC addend, delayed
    reg signed [W-1:0] macc2;
    always @(posedge clk) begin
        m_a1  <= opa;  m_b1 <= opb;  macc1 <= opc;  // stage 1: capture operands
        p_s2  <= m_a1 * m_b1;                        // stage 2: 96-bit product
        macc2 <= macc1;
    end
    // stage 3: renormalise Q48.48 -> Q24.24, optional +c, saturate to 48b
    wire signed [2*W-1:0] macc2_ext = {{W{macc2[W-1]}}, macc2} <<< QF; // c in Q48.48
    wire signed [2*W-1:0] psum      = p_s2 + macc2_ext;
    wire signed [2*W-1:0] pshift    = psum >>> QF;                     // -> Q24.24
    // saturate the high bits
    wire sat_ovf = ~(&pshift[2*W-1 : W-1]) & (|pshift[2*W-1 : W-1]);
    wire signed [W-1:0] mul_sat =
        sat_ovf ? (pshift[2*W-1] ? {1'b1,{(W-1){1'b0}}} : {1'b0,{(W-1){1'b1}}})
                : pshift[W-1:0];

    // -----------------------------------------------------------------------
    // Shared 48-bit add / sub / min / max / neg / abs (single carry chain).
    // -----------------------------------------------------------------------
    wire signed [W:0] add_ext = opa + opb;
    wire signed [W:0] sub_ext = opa - opb;
    function [W-1:0] sat48(input signed [W:0] x);
        sat48 = (x[W] != x[W-1]) ? (x[W] ? {1'b1,{(W-1){1'b0}}} : {1'b0,{(W-1){1'b1}}})
                                 : x[W-1:0];
    endfunction
    wire signed [W-1:0] add_s = sat48(add_ext);
    wire signed [W-1:0] sub_s = sat48(sub_ext);
    wire signed [W-1:0] min_s = ($signed(opa) < $signed(opb)) ? opa : opb;
    wire signed [W-1:0] max_s = ($signed(opa) > $signed(opb)) ? opa : opb;
    wire signed [W-1:0] neg_s = sat48(-$signed({opa[W-1],opa}));
    wire signed [W-1:0] abs_s = opa[W-1] ? neg_s : opa;

    // Leading-zero count (priority encoder) -> NR seed / normalise support.
    reg [RAW:0] lzc;
    integer j;
    always @* begin
        lzc = (RAW+1)'(W);
        for (j = 0; j < W; j = j + 1)
            if (opa[j]) lzc = (RAW+1)'(W - 1 - j);
    end
    // crude rsqrt seed: y0 ~ 2^(-(exp)/2) via shift of a constant, refined by NR
    wire signed [W-1:0] rsqrt_seed = ({{(W-1){1'b0}},1'b1} << QF) >> (( (W-QF) - lzc) >> 1);
    wire signed [W-1:0] lzc_s      = $signed({{(W-RAW-1){1'b0}}, lzc});
    wire signed [W-1:0] asr_s      = opa >>> f_b;   // arithmetic shift by field b

    reg signed [W-1:0] alu_y;
    always @* begin
        case (aluop)
            4'd0, 4'd1: alu_y = mul_sat;   // MUL / MAC (from pipeline)
            4'd2:       alu_y = add_s;
            4'd3:       alu_y = sub_s;
            4'd4:       alu_y = min_s;
            4'd5:       alu_y = max_s;
            4'd6:       alu_y = opa;        // MOV
            4'd7:       alu_y = neg_s;
            4'd8:       alu_y = abs_s;
            4'd9:       alu_y = rsqrt_seed;
            4'd10:      alu_y = lzc_s;
            4'd11:      alu_y = asr_s;
            4'd12:      alu_y = opc[W-1] ? opa : opb;  // sign-select (gates)
            default:    alu_y = opa;
        endcase
    end
    wire alu_is_mul = (aluop == 4'd0) || (aluop == 4'd1);

    // -----------------------------------------------------------------------
    // Sequencer FSM.  s_idle accepts PUSH/POP/CFG (single-cycle regfile
    // access) and RUN (kick the microprogram).  s_run streams the ROM through
    // the shared datapath; multiply ops wait the 3-cycle pipeline latency.
    // -----------------------------------------------------------------------
    localparam [1:0] S_IDLE=2'd0, S_RUN=2'd1, S_MULWAIT=2'd2, S_DONE=2'd3;
    reg [1:0] state;
    reg [1:0] mulcnt;

    assign busy      = (state != S_IDLE);
    assign cmd_ready = (state == S_IDLE) && !resp_valid;

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE; upc <= 0; we <= 1'b0; resp_valid <= 1'b0;
            ra<=0; rb<=0; rc<=0; rw<=0; mulcnt<=0; wdata<=0; resp_data<=0;
        end else begin
            we <= 1'b0;
            if (resp_valid && resp_ready) resp_valid <= 1'b0;

            case (state)
                S_IDLE: if (cmd_valid && cmd_ready) begin
                    case (cmd_funct)
                        7'd0: begin // PUSH input value at index rs1
                            rw    <= cmd_rs1[RAW-1:0];
                            wdata <= cmd_rs2[W-1:0];
                            we    <= 1'b1;
                        end
                        7'd3: begin // CFG constant at index rs1
                            rw    <= cmd_rs1[RAW-1:0];
                            wdata <= cmd_rs2[W-1:0];
                            we    <= 1'b1;
                        end
                        7'd2: begin // POP output at index rs1
                            resp_data  <= {{16{rf[cmd_rs1[RAW-1:0]][W-1]}}, rf[cmd_rs1[RAW-1:0]]};
                            resp_valid <= 1'b1;
                        end
                        7'd1: begin // RUN the FC step
                            upc <= 0; state <= S_RUN; mulcnt <= 0;
                        end
                        default: ;
                    endcase
                end

                S_RUN: begin
                    ra <= f_a; rb <= f_b; rc <= f_c; rw <= f_w;
                    if (aluop == 4'd13) begin      // HALT
                        state <= S_DONE;
                    end else if (alu_is_mul) begin
                        mulcnt <= 2'd2; state <= S_MULWAIT;   // drain pipeline
                    end else begin
                        wdata <= alu_y; we <= 1'b1;
                        upc   <= upc + 1'b1;
                    end
                end

                S_MULWAIT: begin
                    if (mulcnt != 0) mulcnt <= mulcnt - 1'b1;
                    else begin
                        wdata <= alu_y; we <= 1'b1;   // mul_sat now valid
                        upc   <= upc + 1'b1;
                        state <= S_RUN;
                    end
                end

                S_DONE: begin
                    resp_data  <= 64'd1;              // step-complete token
                    resp_valid <= 1'b1;
                    state      <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
