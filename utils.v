module reg_file(input [4:0] A1, A2, A3,
		input [31:0] WD3,
		input clk, WE3,
		output reg [31:0] RD1, RD2);

    reg [31:0] registers [31:0];

    always @(*)
    begin
        RD1 <= A1 ? registers[A1] : 0;
	RD2 <= A2 ? registers[A2] : 0;
    end     

    always @(posedge clk)
    begin
        registers[A3] <= A3 ? (WE3 ? WD3 : registers[A3]) : 0;
    end     

  always #1000 $display( "%d,%d,%d,%d,%d,%d", registers[0],registers[1],registers[2],registers[3],registers[4],registers[5]);
//always @(registers[31]) $display( "%d",registers[31] );

endmodule 

module sign_ext(input [15:0] A,
		output [31:0] Y);
	assign Y = { {16{A[15]}}, A[15:0]};
        //assign Y = { A[15:0], A[15:0]};
endmodule 


module dmem (input clk, we,
             input [31:0] a, wd,
             output [31:0] rd);

  reg [31:0] RAM[127:0];

  assign rd = RAM[a[7:2]]; // word aligned

  always@(posedge clk)
    if(we) RAM[a[31:2]] <= wd;

endmodule

module imem (input [5:0] a,
             output [31:0] rd);
 
  // The "a" is the address of instruction to fetch, what
  // for our purpose can be taken from ProgramCounter[7:2]
 
  reg [31:0] RAM[127:0];
 
  initial  $readmemh ("memfile.dat",RAM);
  
  assign rd = RAM[a]; // word aligned
 
endmodule

module mux2 (input [31:0] d0, d1,
 input s,
 output [31:0] y);

 assign y = ~s? d0 : 32'bz;
 assign y = s? d1 : 32'bz;
endmodule

module mux2_5 (input [4:0] d0, d1,
 input s,
 output [4:0] y);

 assign y = ~s? d0 : 5'bz;
 assign y = s? d1 : 5'bz;
endmodule

module mux2_5_A3 (input [4:0] d0,
 input s,
 output [4:0] y);

 assign y = ~s? d0 : 5'bz;
 assign y = s? 31 : 5'bz;
endmodule


module m_and (input x, y, bne,
 output z);

 assign z = x & (bne ? ~y : y);

endmodule
