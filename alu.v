module ALU(input [31:0] SrcA, SrcB, 
	   input [2:0] ALUControl, 
	   output reg [31:0] ALUResult,
           output reg Zero);
always @(*)
  begin
  case (ALUControl)
  3'b000 : ALUResult <= SrcA & SrcB;
  3'b001 : ALUResult <= SrcA | SrcB;
  3'b010 : ALUResult <= SrcA + SrcB;
  3'b110 : ALUResult <= SrcA - SrcB;
  3'b011 : ALUResult <= SrcA ^ SrcB;
  3'b111 : ALUResult <= SrcA < SrcB;
  
  default : $display("Error in ALU CASE");   
endcase

  //Zero <= (&ALUResult);
end

always @(ALUResult) begin
if (ALUResult == 0) Zero = 1;
  else Zero = 0;
end
  //assign Zero = ALUResult == 0? 1 : 0;
endmodule 



