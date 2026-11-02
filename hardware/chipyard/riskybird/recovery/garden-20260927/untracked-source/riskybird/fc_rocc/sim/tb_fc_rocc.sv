// xsim testbench: run the recorded sensor trace through fc_rocc_core (the RTL
// executing the assembled ROM) and write a golden-format replay CSV. Compared
// against replay_ref.csv (fp32 golden) by tools/compare.py.
`default_nettype none
module tb;
  localparam int W=48, RAW=8, Q_POS=27, Q_ATT=24, Q_VEL=28, Q_U=24;
`include "fc_rocc_map.svh"

  reg clk=0, rst=1, we=0, run=0;
  reg [RAW-1:0] waddr=0, raddr=0;
  reg signed [W-1:0] wdata=0;
  reg [15:0] rstart=0;
  wire signed [W-1:0] rdata;
  wire busy;

  fc_rocc_core #(.W(W),.RAW(RAW),.NREG(256),.ROMD(1024)) dut
    (.clk(clk),.rst(rst),.we(we),.waddr(waddr),.wdata(wdata),
     .run(run),.rstart(rstart),.raddr(raddr),.rdata(rdata),.busy(busy));

  always #5 clk = ~clk;

  integer fin, fout, r, step;
  reg [1023:0] waddr_line;  // header scratch
  real dt,ax,ay,az,gx,gy,gz,fx,fy,fv,hh,tv,zsp;

  function [W-1:0] toq(input real v, input int q);
    real s; begin s = v * (2.0**q); toq = $rtoi(s + (v>=0.0?0.5:-0.5)); end
  endfunction
  function real toreal(input signed [W-1:0] x, input int q);
    longint signed li; begin li = x; toreal = $itor(li) / (2.0**q); end
  endfunction

  task wr(input [RAW-1:0] a, input signed [W-1:0] d);
    begin @(negedge clk); we=1; waddr=a; wdata=d; @(negedge clk); we=0; end
  endtask
  task runprog(input [15:0] start);
    begin @(negedge clk); run=1; rstart=start; @(negedge clk); run=0;
          while (busy) @(negedge clk); end
  endtask
  real oo[0:12];
  task rd1(input int idx, input int a, input int q);
    begin raddr=a[RAW-1:0]; #1; oo[idx]=toreal(rdata,q); end
  endtask
  task snapshot;
    begin
      rd1(0,SOUT[0],Q_POS); rd1(1,SOUT[1],Q_POS); rd1(2,SOUT[2],Q_POS);
      rd1(3,SOUT[3],Q_ATT); rd1(4,SOUT[4],Q_ATT); rd1(5,SOUT[5],Q_ATT);
      rd1(6,SOUT[6],Q_VEL); rd1(7,SOUT[7],Q_VEL); rd1(8,SOUT[8],Q_VEL);
      rd1(9,U0,Q_U); rd1(10,U1,Q_U); rd1(11,U2,Q_U); rd1(12,U3,Q_U);
    end
  endtask

  integer ri;
  reg signed [W-1:0] initmem [0:255];
  initial begin
    repeat(4) @(negedge clk); rst=0; @(negedge clk);
    // boot: load consts+state into the regfile via the write port (RoCC CFG path)
    $readmemh("fc_rocc_init.hex", initmem);
    for (ri=0; ri<256; ri=ri+1) wr(ri[RAW-1:0], initmem[ri]);
    fin = $fopen("out/sensors.csv","r");
    fout= $fopen("out/replay_rtl.csv","w");
    $fdisplay(fout,"# impl=fc_rocc_core RTL (xsim, assembled ROM)");
    $fdisplay(fout,"step,ex,ey,ez,er1,er2,er3,evx,evy,evz,u0,u1,u2,u3");
    r = $fgets(waddr_line, fin);  // skip header (dummy read)
    step = 0;
    while (!$feof(fin)) begin
      r = $fscanf(fin,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                  dt,ax,ay,az,gx,gy,gz,fx,fy,fv,hh,tv,zsp);
      if (r == 13) begin
        // push estimator inputs (Q24 algebra; flags Q0)
        wr(IN_AX,toq(ax,Q_ATT)); wr(IN_AY,toq(ay,Q_ATT)); wr(IN_AZ,toq(az,Q_ATT));
        wr(IN_GX,toq(gx,Q_ATT)); wr(IN_GY,toq(gy,Q_ATT)); wr(IN_GZ,toq(gz,Q_ATT));
        wr(IN_F0,toq(fx,Q_ATT)); wr(IN_F1,toq(fy,Q_ATT));
        wr(IN_H, toq(hh,Q_ATT)); wr(IN_DT,toq(dt,Q_ATT));
        wr(IN_FV, fv>0.5?48'd1:48'd0); wr(IN_TV, tv>0.5?48'd1:48'd0);
        runprog(EST_START[15:0]);
        // control setpoint: desH=zsp, others 0
        wr(IN_DESH,toq(zsp,Q_ATT)); wr(IN_DESV1,48'd0); wr(IN_DESV2,48'd0); wr(IN_YAWT,48'd0);
        wr(IN_DT, toq(dt,Q_ATT));
        runprog(CTRL_START[15:0]);
        snapshot();
        $fdisplay(fout,"%0d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f",
          step, oo[0],oo[1],oo[2],oo[3],oo[4],oo[5],oo[6],oo[7],oo[8],oo[9],oo[10],oo[11],oo[12]);
        step = step + 1;
      end
    end
    $fclose(fin); $fclose(fout);
    $display("TB done: %0d steps -> out/replay_rtl.csv", step);
    $finish;
  end
endmodule
`default_nettype wire
