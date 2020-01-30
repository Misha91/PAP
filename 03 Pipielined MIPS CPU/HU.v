
module HU(input clk, BranchD, MemToRegE, RegWriteE, MemToRegM, RegWriteM, RegWriteW,
          input [4:0] RsD, RtD, RsE, RtE, WriteRegE, WriteRegM, WriteRegW,
	  output StallF, StallD, ForwardAD, ForwardBD, FlushE,
          output [1:0] ForwardAE, ForwardBE);

  reg StallF, StallD, ForwardAD, ForwardBD, FlushE, lwstall, branchstall;
  reg [1:0] ForwardAE, ForwardBE;

  initial begin
    lwstall <= 0;
    branchstall <= 0;
    StallF <= 0;
    StallD <= 0;
    FlushE <= 0;
    ForwardAD <= 0;
    ForwardBD <= 0;
    ForwardAE <= 0;
    ForwardBE <= 0;
  end

  always @(*)
  begin
    if ((RsE != 0) && (RsE == WriteRegM) && (RegWriteM))
      ForwardAE <= 2'b10;
    else if ((RsE != 0) && (RsE == WriteRegW) && (RegWriteW))
      ForwardAE <= 2'b01;
    else ForwardAE = 2'b00;

    if ((RtE != 0) && (RtE == WriteRegM) && (RegWriteM))
      ForwardBE <= 2'b10;
    else if ((RtE != 0) && (RtE == WriteRegW) && (RegWriteW))
      ForwardBE <= 2'b01;
    else ForwardBE <= 2'b00;

    if (!((RsD === 5'bX) || (RtE=== 5'bX) || (RtD=== 5'bX) || (MemToRegE=== 1'bX) || (MemToRegE=== 1'bZ)))
      lwstall <= ((RsD == RtE) || (RtD == RtE)) && MemToRegE ? 1 : 0;
    
    if (!((WriteRegM === 5'bX)  || (RegWriteM=== 1'bX) ))
    begin
      if (!((RsD === 5'bX) ))
        ForwardAD <= ((RsD != 0) && (RsD == WriteRegM) && RegWriteM) ? 1 : 0;
      if (!((RtD === 5'bX) ))
        ForwardBD <= ((RtD != 0) && (RtD == WriteRegM) && RegWriteM )? 1 : 0;
    end
    
    if (!((BranchD === 1'bX) || (RegWriteE=== 1'bX) || (WriteRegE=== 5'bX) || (RtD=== 5'bX) || (RsD=== 5'bX) ))
      begin
      branchstall <= (BranchD && RegWriteE && (WriteRegE == RsD || WriteRegE == RtD)) ;
      $display("This");
      end
    if (!((BranchD === 1'bX) || (MemToRegM=== 1'bZ) || (WriteRegM=== 5'bX) || (RtD=== 5'bX) || (RsD=== 5'bX) ))
      begin
      branchstall <= (BranchD && MemToRegM && (WriteRegM == RsD || WriteRegM == RtD));
      $display("That");
      end
      
    StallF <= lwstall | branchstall;
    StallD <= lwstall | branchstall;
    FlushE <= lwstall | branchstall;
    

  end

  always @(negedge clk)
    $display("HU: %d %d %d %d %d %d %h %h", branchstall, BranchD, RegWriteE, MemToRegM, WriteRegE, WriteRegM, RsD, RtD );

endmodule
