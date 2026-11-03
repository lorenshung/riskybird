// ===========================================================================
// fc_rocc_core.sv -- FcRoCC datapath + microcode sequencer (validated RTL).
//
// Executes the assembled microcode ROM (fc_rocc_asm.c emits fc_rocc_rom.hex +
// fc_rocc_init.hex). This is the synthesizable + simulatable core: one shared
// signed 48x48 multiply with a per-op renorm shift + saturation, a 48-bit
// ADD/SUB/MIN/MAX/MOV/NEG, plus MSB (leading set bit), VSH (signed
// register-sourced shift), CLT (a<b?1:0) and SEL (c!=0?a:b) -- the exact ISA
// the C interpreter (exec()) was validated against (open 6.78um / closed 51um).
//
// Sequencer: RUN latches pc=rstart and streams microwords one/cycle until HALT.
// A simple write/read port loads consts+state (init) and streams sensor inputs
// in / motor outputs out (the RoCC PUSH/RUN/POP maps onto these).
// ===========================================================================
`default_nettype none
module fc_rocc_core #(
    parameter int W    = 48,
    parameter int RAW  = 8,
    parameter int NREG = 256,
    parameter int ROMD = 1024,
    parameter     ROM_FILE  = "fc_rocc_rom.hex",
    parameter     INIT_FILE = "fc_rocc_init.hex"
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 we,       // write reg (init / input push)
    input  wire [RAW-1:0]       waddr,
    input  wire signed [W-1:0]  wdata,
    input  wire                 run,      // pulse: start a program
    input  wire [15:0]          rstart,   // its ROM entry address
    input  wire [RAW-1:0]       raddr,    // async read (output pop)
    output wire signed [W-1:0]  rdata,
    output reg                  busy
);
    localparam int OP_MUL=0, OP_CLT=1, OP_ADD=2, OP_SUB=3, OP_MIN=4, OP_MAX=5,
                   OP_MOV=6, OP_NEG=7, OP_MSB=10, OP_VSH=11, OP_SEL=12, OP_HALT=13;

    (* rom_style="block" *) reg [63:0] rom [0:ROMD-1];
    // regfile: sync write + async reads -> distributed RAM (LUTRAM). NOT $readmemh-
    // initialised (that forces FF+mux mapping); consts/state are loaded at boot via
    // the `we` port (the RoCC CFG path). INIT_FILE param kept for reference only.
    (* ram_style="distributed" *) reg signed [W-1:0] rf [0:NREG-1];
    initial $readmemh(ROM_FILE, rom);

    reg [15:0] pc;
    wire [63:0] uw   = rom[pc];
    wire [3:0]  op   = uw[3:0];
    wire signed [6:0] sh = uw[10:4];
    wire [RAW-1:0] fd = uw[11 +: RAW];
    wire [RAW-1:0] fa = uw[19 +: RAW];
    wire [RAW-1:0] fb = uw[27 +: RAW];
    wire [RAW-1:0] fc = uw[35 +: RAW];

    // 3 read ports (LUTRAM). POP (raddr) shares the fa port when idle.
    wire [RAW-1:0]      fa_eff = busy ? fa : raddr;
    wire signed [W-1:0] opa = rf[fa_eff];
    wire signed [W-1:0] opb = rf[fb];
    wire signed [W-1:0] opc = rf[fc];

    // ---- saturating helpers ----
    localparam signed [W-1:0] SMAX = {1'b0, {(W-1){1'b1}}};
    localparam signed [W-1:0] SMIN = {1'b1, {(W-1){1'b0}}};
    function automatic signed [W-1:0] sat96(input signed [2*W-1:0] x);
        if (x > SMAX)      sat96 = SMAX;
        else if (x < SMIN) sat96 = SMIN;
        else               sat96 = x[W-1:0];
    endfunction
    function automatic signed [W-1:0] sat49(input signed [W:0] x);
        if (x > SMAX)      sat49 = SMAX;
        else if (x < SMIN) sat49 = SMIN;
        else               sat49 = x[W-1:0];
    endfunction

    // ---- shared multiply + per-op renorm shift ----
    (* use_dsp = "yes" *) wire signed [2*W-1:0] prod = opa * opb;
    wire signed [2*W-1:0] mul_sh = (sh >= 0) ? (prod >>> sh) : (prod <<< (-sh));
    wire signed [W-1:0]   mul_r  = sat96(mul_sh);

    // ---- VSH: signed register-sourced shift (NR normalize/denormalize) ----
    wire signed [31:0]    vs    = opb[31:0];
    wire signed [2*W-1:0] opa_x = {{W{opa[W-1]}}, opa};
    wire signed [2*W-1:0] vsh_l = opa_x <<< vs;
    wire signed [W-1:0]   vsh_r = opa >>> (-vs);
    wire signed [W-1:0]   vsh_out = (vs >= 0) ? sat96(vsh_l) : vsh_r;

    // ---- MSB: index of leading set bit of |opa| ----
    wire signed [W-1:0] absa = opa[W-1] ? (~opa + 1'b1) : opa;
    integer j; reg [7:0] msb_i;
    always @* begin msb_i = 8'd0; for (j=0;j<W;j=j+1) if (absa[j]) msb_i = j[7:0]; end

    reg signed [W-1:0] alu;
    always @* begin
        case (op)
            OP_MUL: alu = mul_r;
            OP_ADD: alu = sat49($signed({opa[W-1],opa}) + $signed({opb[W-1],opb}));
            OP_SUB: alu = sat49($signed({opa[W-1],opa}) - $signed({opb[W-1],opb}));
            OP_MIN: alu = (opa < opb) ? opa : opb;
            OP_MAX: alu = (opa > opb) ? opa : opb;
            OP_MOV: alu = opa;
            OP_NEG: alu = sat49(-$signed({opa[W-1],opa}));
            OP_MSB: alu = $signed({{(W-8){1'b0}}, msb_i});
            OP_VSH: alu = vsh_out;
            OP_CLT: alu = (opa < opb) ? {{(W-1){1'b0}},1'b1} : {W{1'b0}};
            OP_SEL: alu = (opc != 0) ? opa : opb;
            default: alu = opa;
        endcase
    end

    assign rdata = opa;   // POP reads via fa port (fa_eff=raddr when idle)

    // single write port (mutually exclusive: sequencer while busy, we while idle)
    wire            seq_wr = busy && (op != OP_HALT);
    wire            rf_we  = seq_wr || (we && !busy);
    wire [RAW-1:0]  rf_wa  = busy ? fd  : waddr;
    wire signed [W-1:0] rf_wd = busy ? alu : wdata;
    always @(posedge clk) if (rf_we) rf[rf_wa] <= rf_wd;

    // sequencer
    always @(posedge clk) begin
        if (rst) begin busy <= 1'b0; pc <= 16'd0; end
        else if (!busy) begin if (run) begin pc <= rstart; busy <= 1'b1; end end
        else if (op == OP_HALT) busy <= 1'b0;
        else pc <= pc + 16'd1;
    end
endmodule
`default_nettype wire
