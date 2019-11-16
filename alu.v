// ALU module. Codes of 100 and 101 
// were used for logical shift left
// and right respectivly.

module ALU(input [31:0] SrcA, SrcB, 
	   input [2:0] ALUControl,
	   input [4:0] shamt, 
	   output reg [31:0] ALUResult,
           output reg Zero);

always @(*)
  begin
  case (ALUControl)
  3'b000 : ALUResult <= SrcA & SrcB;
  3'b001 : ALUResult <= SrcA | SrcB;
  3'b010 : ALUResult <= SrcA + SrcB;  
  3'b011 : ALUResult <= SrcA ^ SrcB;
  3'b100 : ALUResult <= SrcB << shamt;
  3'b101 : ALUResult <= SrcB >> shamt;
  3'b110 : ALUResult <= SrcA - SrcB;
  3'b111 : ALUResult <= SrcA < SrcB;
  3'bzzz : ALUResult <= 32'bx;
  default : $display("Error in ALU CASE");   
endcase
  //$display("Y=%d, Z=%h, Ctrl=%b", ALUResult, Zero, ALUControl);
  
end

always @(ALUResult) begin
  if (ALUResult == 0) Zero = 1;
  else Zero = 0;

  //$display("Y=%d, Z=%h, Ctrl=%h, A=%d, B=%d, shamt=%d", ALUResult, Zero, ALUControl, SrcA, SrcB, shamt);
end

endmodule 



