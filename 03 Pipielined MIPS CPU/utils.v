// Register file to store data in registers
// Keep register 0 always equal to 0
// Register 31 to be used as return adress

module reg_file(input printWire,
                input [4:0] A1, A2, A3,
		input [31:0] WD3,
		input clk, WE3,
		output reg [31:0] RD1, RD2);

    reg [31:0] registers [31:0];

    initial registers[0] = 0;

    always @(*)
    begin
        RD1 <= A1 ? registers[A1] : 0;
	RD2 <= A2 ? registers[A2] : 0;
    end     

    always @(negedge clk)
    begin
        registers[A3] <= A3 ? (WE3 ? WD3 : registers[A3]) : 0;
    end     

  always @(posedge clk) $display( "%d,%d,%d,%d,%d,%d", registers[0],registers[1],registers[2],registers[3],registers[4],registers[5]);
  //always @(posedge printWire) $display( "%d,%d,%d,%d,%d,%d", registers[0],registers[1],registers[2],registers[3],registers[4],registers[5]);
  
endmodule 


//Helper module for sign extension
module sign_ext(input [15:0] A,
		output [31:0] Y);
	assign Y = { {16{A[15]}}, A[15:0]};
endmodule 



//Helper module for data memory
module dmem (input clk, we,
             input [31:0] a, wd,
             output [31:0] rd);

  reg [31:0] RAM[127:0];

  assign rd = RAM[a[7:2]]; // word aligned

  always@(posedge clk)
    if(we) RAM[a[31:2]] <= wd;

endmodule


//Helper module for instruction memory
module imem (input [5:0] a,
             output [31:0] rd);
 
  // The "a" is the address of instruction to fetch, what
  // for our purpose can be taken from ProgramCounter[7:2]
 
  reg [31:0] RAM[127:0];
 
  initial  $readmemh ("memfile.dat",RAM);
  
  assign rd = RAM[a]; // word aligned
 
endmodule

//3:1 Mulitplexer with data length of 32 bits
module mux3 (input [31:0] d0, d1, d2,
 input[1:0] s,
 output [31:0] y);
 
 wire [31:0] d_tmp;
 mux2 mux_tmp(d0,d1,s[0], d_tmp);
 mux2 mux_fin(d_tmp, d2, s[1], y);

endmodule

//2:1 Mulitplexer with data length of 32 bits
module mux2 (input [31:0] d0, d1,
 input s,
 output [31:0] y);

 assign y = ~s? d0 : 32'bz;
 assign y = s? d1 : 32'bz;
endmodule

//2:1 Mulitplexer with data length of 5 bits
module mux2_5 (input [4:0] d0, d1,
 input s,
 output [4:0] y);

 assign y = ~s? d0 : 5'bz;
 assign y = s? d1 : 5'bz;
endmodule




// Module for PC update. If command type is jr,
// it jumps on adress present in srcA. If command
// is jal it jumps on label. Increments PC by 4
// otherwise.

module pc_update (input [31:0] PC, addr_w_offset, srcA,
 input PCSrc, jal, jr,
 output [31:0] newPC);

 assign newPC = jr ? srcA : (jal ? addr_w_offset*4 : (PCSrc ? PC + 4 + (addr_w_offset*4): PC + 4));
endmodule




//Helper module for PC source select

module m_and (input x, y, bne,
 output z);

 assign z = x & (bne ? ~y : y);

endmodule
