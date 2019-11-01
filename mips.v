module mips;
  reg [31:0] PC;
  wire [31:0] cmd, result;
  reg clk;
    

  imem imem_mod(PC[7:2], cmd);
  ctrl my_ctrl(clk, cmd, result);
  

  initial begin 
    //cmd = 32'bz;
    clk = 0; 
    PC = 0; 
    #200 $finish;
  end  

  always #10 clk = ~clk;

  always @(posedge clk)
  begin
      PC = PC + 4;
  end     

  //always @(cmd) $display( "time = %d, pc = %h, cmd = %h.", $time, PC[7:2], cmd);

endmodule
    
module ctrl(input clk,
	    input [31:0] cmd,
            output [31:0] result);

  reg WE3;
  reg [4:0] A1, A2, A3;
  reg [5:0] Op, funct;
  reg [31:0] WD3;
  wire [31:0] addr_w_offset;

  sign_ext my_sign(cmd[15:0], addr_w_offset);

  
  initial begin
  funct = 6'bx;
  A1 = 5'bz;
  A2 = 5'bz;
  Op = 6'bz;
  end
	
  always @(cmd)
  begin
    funct = cmd[5:0];
    A1 = cmd[25:21];
    A2 = cmd[20:16];
    Op = cmd[31:26];
    

  case (Op)
    6'b000000 : begin
      case (funct)
        6'b000000: begin
	  $display("cmd = %h, Op=%b, f=%b - SLL", cmd, Op, funct);
	  end 
 
        6'b000010: begin
	  $display("cmd = %h, Op=%b, f=%b - SRL", cmd, Op, funct);
	  end 

        6'b001000: begin
	  $display("cmd = %h, Op=%b, f=%b - JR", cmd, Op, funct);
	  end 

        6'b100000: begin
	  $display("cmd = %h, Op=%b, f=%b - ADD", cmd, Op, funct);
	  end 

        6'b100010: begin
	  $display("cmd = %h, Op=%b, f=%b - SUB", cmd, Op, funct);
	  end 

        6'b100100: begin
	  $display("cmd = %h, Op=%b, f=%b - AND", cmd, Op, funct);
	  end 

        6'b100101: begin
	  $display("cmd = %h, Op=%b, f=%b - OR", cmd, Op, funct);
	  end 

        6'b101010: begin
	  $display("cmd = %h, Op=%b, f=%b - SLT", cmd, Op, funct);
	  end 

 
	default : $display("cmd = %b, UNSUPPORTED OPCODE", cmd);   
      endcase
    end

    6'b000100 : begin
      $display("cmd = %h, Op=%b, f=%b - BEQ", cmd, Op, funct);
    end

    6'b000101 : begin
      $display("cmd = %h, Op=%b, f=%b - BNE", cmd, Op, funct);
    end

    6'b001000 : begin
      $display("cmd = %h, Op=%b, f=%b - ADDI", cmd, Op, funct);
    end

    6'b100011 : begin
      $display("cmd = %h, Op=%b, f=%b - LW", cmd, Op, funct);
    end

    6'b101011 : begin
      $display("cmd = %h, Op=%b, f=%b - SW", cmd, Op, funct);
    end
  
    default : $display("cmd = %h, UNSUPPORTED OPCODE", cmd);   
  endcase
  end


    /*always @(clk) $display("clk = %b, code = %h, code_b = %b, offset = %b, funct = %b, Op = %b, A1 = %b, A2 = %b, add_w_o = %b", clk, cmd, cmd, cmd[15:0], funct, Op, A1, A2, addr_w_offset);*/
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
 
  reg [31:0] RAM[11:0];
 
  initial  $readmemh ("memfile.dat",RAM);
  
  assign rd = RAM[a]; // word aligned
 
endmodule
