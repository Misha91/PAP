
module HU(input clk, BranchD, MemToRegE, RegWriteE, MemToRegM, RegWriteM, RegWriteW,
          input [4:0] RsD, RtD, RsE, RtE, WriteRegE, WriteRegM, WriteRegW,
	  output StallF, StallD, ForwardAD, ForwardBD, FlushE,
          output [1:0] ForwardAE, ForwardBE);

  reg StallF, StallD, ForwardAD, ForwardBD, FlushE, lwstall;
  reg [1:0] ForwardAE, ForwardBE;

  initial begin
    StallF = 0;
    StallD = 0;
    FlushE = 0;
    ForwardAD = 0;
    ForwardBD = 0;
    ForwardAE = 0;
    ForwardBE = 0;
  end

  always @(*)
  begin
    if ((RsE != 0) && (RsE == WriteRegM) && (RegWriteM))
      ForwardAE = 2'b10;
    else if ((RsE != 0) && (RsE == WriteRegW) && (RegWriteW))
      ForwardAE = 2'b01;
    else ForwardAE = 2'b00;

    if ((RtE != 0) && (RtE == WriteRegM) && (RegWriteM))
      ForwardBE = 2'b10;
    else if ((RtE != 0) && (RtE == WriteRegW) && (RegWriteW))
      ForwardBE = 2'b01;
    else ForwardBE = 2'b00;

    if (!((RsD === 5'bX) || (RtE=== 5'bX) || (RtD=== 5'bX) || (MemToRegE=== 1'bX) || (MemToRegE=== 1'bZ)))
    begin
      lwstall = ((RsD == RtE) || (RtD == RtE)) && MemToRegE ? 1 : 0;
      StallF = lwstall;
      StallD = lwstall;
      FlushE = lwstall;
      //$display("X check: %b %b %b %b", RsD, RtE, RtD, MemToRegE);
    end
    

  end

endmodule
