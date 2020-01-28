

module MEM(input clk, RegWriteM, MemtoRegM, MemWriteM,
	   input [4:0] WriteRegM,
	   input [31:0] ALUOutM, WriteDataM,
	   output RegWriteW, MemtoRegW,
	   output [4:0] WriteRegW,
	   output [31:0] ALUOutW, ReadDataW);

  wire [31:0] RD;
  reg RegWriteW, MemtoRegW;
  reg [4:0] WriteRegW;
  reg [31:0] ALUOutW, ReadDataW;

  dmem my_dmem(clk, MemWriteM, ALUOutM, WriteDataM, RD);

  always @(posedge clk)
    begin
    RegWriteW = RegWriteM;
    MemtoRegW = MemtoRegM;
    ReadDataW = RD;
    ALUOutW = ALUOutM;
    WriteRegW = WriteRegM;
    end

endmodule
    
