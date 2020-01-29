
module HU(input BranchD, MemToRegE, RegWriteE, MemToRegM, RegWriteM, RegWriteW,
          input [4:0] RsD, RtD, RsE, RtE, WriteRegE, WriteRegM, WriteRegW,
	  output StallF, StallD, ForwardAD, ForwardBD, FlushE,
          output [1:0] ForwardAE, ForwardBE);

  reg StallF, StallD, ForwardAD, ForwardBD, FlushE;
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

endmodule
