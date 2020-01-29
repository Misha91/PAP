

module EX(input clk, RegWriteE, MemtoRegE, MemWriteE, ALUSrcE, RegDstE, 
          input [1:0] ForwardAE, ForwardBE,
          input [2:0] ALUControlE,
	  input [4:0] RsE, RtE, RdE, shamtE,
	  input [31:0] RD1E, RD2E, SignImmE, ResultW,
	  output RegWriteM, MemtoRegM, MemWriteM,
	  output [4:0] WriteRegE, WriteRegM,
	  output [31:0] ALUOutM, WriteDataM);


  wire Zero;
  wire [4:0] WriteRegE;
  wire [31:0] srcAE, srcBE, WriteDataE, ALUOutE;

  reg RegWriteM, MemtoRegM, MemWriteM;
  reg [4:0] WriteRegM;
  reg [31:0] ALUOutM, WriteDataM;

  mux2_5 wr_reg(RtE, RdE, RegDstE, WriteRegE);

  mux3 srcA_mux3(RD1E, ResultW, ALUOutM, ForwardAE, srcAE);
  mux3 srcB_mux3(RD2E, ResultW, ALUOutM, ForwardBE, WriteDataE);

  mux2 srcB_mux3_fin(WriteDataE, SignImmE, ALUSrcE, srcBE);

  ALU my_alu(srcAE, srcBE, ALUControlE, shamtE, ALUOutE, Zero);

  always @(posedge clk)
  begin
    RegWriteM <= RegWriteE;
    MemtoRegM <= MemtoRegE;
    MemWriteM <= MemWriteE;
    ALUOutM <= ALUOutE;
    WriteRegM <= WriteRegE;
    WriteDataM <= WriteDataE;
    //$display("EX: %b %h %h %h %h %h %b %h %b %h %h %h", ForwardBE, RD1E, RD2E, WriteDataE, RtE, RdE, RegDstE, WriteRegE, ALUControlE, srcAE, srcBE, ALUOutE);
  end


endmodule


