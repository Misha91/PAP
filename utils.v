module reg_file(input [4:0] A1, A2, A3,
		input [31:0] WD3,
		input clk, WE3,
		output reg [31:0] RD1, RD2);

    reg [31:0] registers [31:0];

    initial begin
    registers[0] = 0;
    end

    always @(posedge clk)
    begin
        RD1 <= WE3? 32'bz : registers[A1];
	RD2 <= WE3? 32'bz : registers[A2];
        registers[A3] <= WE3 ? WD3 : registers[A3];
    end     
  
endmodule 

module sign_ext(input [15:0] A,
		output [31:0] Y);
	assign Y = { {16{A[15]}}, A[15:0]};
        //assign Y = { A[15:0], A[15:0]};
endmodule 

