//Test bench module
module mips_tb;
  reg [31:0] PC;
  reg clk;

  mips_pipelined my_mips(clk, PC);

  initial begin 
    $dumpfile("test.vcd");
    $dumpvars;
    clk = 0; 
    PC = 0; 
    //#900 $finish;
  end  

  always #10 clk = ~clk;
endmodule



module mips_pipelined(input clk,
	    input [31:0] PC_init);

  wire StallF, StallD, ForwardAD, ForwardBD, FlushE;
  wire [1:0] ForwardAE, ForwardBE;
  wire RegWriteE, MemtoRegE, MemWriteE, ALUSrcD, RegDstD, BranchD, PCSrcD, RegWriteM, MemtoRegM, MemWriteM, RegWriteW, MemtoRegW;
  wire [2:0] ALUControlD;
  wire [4:0] RsD, RtD, RsE, RtE, RdE, shamtE, WriteRegM, WriteRegW, WriteRegE;
  wire[31:0] cmdD, PCPlusFourD, RD1D, RD2D, SignImmD, PCBranchD, ALUOutM, WriteDataM, ALUOutW, ReadDataW, ResultW;

  HU my_HU(clk, BranchD, MemtoRegE, RegWriteE, MemToRegM, RegWriteM, RegWriteW, RsD, RtD, RsE, RtE, WriteRegE, WriteRegM, WriteRegW, StallF, StallD, ForwardAD, ForwardBD, FlushE, ForwardAE, ForwardBE);
  IF my_IF(clk, StallF, StallD, PCSrcD, PC_init, PCBranchD, cmdD, PCPlusFourD);
  ID my_ID(clk, ForwardAD, ForwardBD, FlushE, RegWriteW, WriteRegW, cmdD, PCPlusFourD, ResultW, ALUOutM, RegWriteE, MemtoRegE, MemWriteE, ALUSrcD, RegDstD, BranchD, PCSrcD, ALUControlD, RsD, RtD, RsE, RtE, RdE, shamtE, RD1D, RD2D, SignImmD, PCBranchD);
  EX my_EX(clk, RegWriteE, MemtoRegE, MemWriteE, ALUSrcD, RegDstD, ForwardAE, ForwardBE, ALUControlD, RsE, RtE, RdE, shamtE, RD1D, RD2D, SignImmD, ResultW, RegWriteM, MemtoRegM, MemWriteM, WriteRegE, WriteRegM, ALUOutM, WriteDataM);
  MEM my_MEM(clk, RegWriteM, MemtoRegM, MemWriteM, WriteRegM, ALUOutM, WriteDataM, RegWriteW, MemtoRegW, WriteRegW, ALUOutW, ReadDataW);

  mux2 write(ALUOutW, ReadDataW, MemtoRegW, ResultW);

endmodule



