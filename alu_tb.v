module tb_c;
  reg [2:0] ctrl;
  reg [31:0] inpA;
  reg [31:0] inpB;
  wire [31:0] Y;
  wire Zero;
  ALU ALU_test (inpA, inpB, ctrl, Y, Zero);

  initial begin
  inpA = 4'b0101;
  inpB = 4'b1100;
  ctrl = 3'b011;
  end

  always @(Y) $display( "Time: %d, ctrl=%b, A=%b, B=%b,Y=%b, Z=%b.",$time, ctrl, inpA, inpB, Y, Zero);
endmodule
  


