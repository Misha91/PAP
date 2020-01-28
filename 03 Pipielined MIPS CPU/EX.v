

module EX(input clk, RegWriteE, MemtoRegE, MemWriteE, ALUSrcE, RegDstE, 
          input [1:0] ForwardAE, ForwardBE,
          input [2:0] ALUControlE,
	  input [4:0] RsE, RtE, RdE, shamtE,
	  input [31:0] RD1E, RD2E, SignImmE,
	  output RegWriteM, MemtoRegM, MemWriteM,
	  output [4:0] WriteRegM,
	  output [31:0] ALUOutM, WriteDataM);


  wire Zero;
  wire [4:0] WriteRegE;
  wire [31:0] srcAE, srcBE, WriteDataE, ALUOutE;

  reg RegWriteM, MemtoRegM, MemWriteM;
  reg [4:0] WriteRegM;
  reg [31:0] ALUOutM, WriteDataM;

  mux2_5 wr_reg(RtE, RdE, RegDstE, WriteRegE);

  mux3 srcA_mux3(RD1E, 0, 0, ForwardAE, srcAE);
  mux3 srcB_mux3(RD2E, 0, 0, ForwardAE, WriteDataE);

  mux2 srcB_mux3_fin(WriteDataE, SignImmE, ALUSrcE, srcBE);

  ALU my_alu(srcAE, srcBE, ALUControlE, shamtE, ALUOutE, Zero);

  always @(posedge clk)
  begin
    RegWriteM <= RegWriteE;
    MemtoRegM <= MemtoRegE;
    MemWriteM <= MemWriteE;
    ALUOutM <= ALUOutE;
    WriteRegM <= WriteRegE;
    $display("EX: %b %h %h %h", ALUControlE, srcAE, srcBE, ALUOutE);
  end


endmodule


