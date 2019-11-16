//Test bench module
module mips_tb;
  reg [31:0] PC;
  reg clk;

  mips my_mips(clk, PC);

  initial begin 
    clk = 0; 
    PC = 0; 
    #1000 $finish;
  end  

  always #10 clk = ~clk;
endmodule


/*

Main MIPS module - has processor submodule, 
requires clk and initial PC to start. Updates
PC on every clk.

*/
module mips(input clk,
	    input [31:0] PC_init);

  wire [31:0] cmd, result, newPC;
  reg [31:0] PC;

  always @(PC_init) PC = PC_init;

  imem imem_mod(PC[7:2], cmd);
  processor my_ctrl(clk, cmd, PC, newPC, result);
  
  always @(posedge clk)
  begin
      PC <= newPC;
  end      
  //always @(PC) $display("PC=%d", PC);

endmodule

/*

Main module. Performs all required computation, 
updates register file and data memory. It has 
$display statement for debug easiness.
 
*/
    
module processor(input clk,
	    	input [31:0] cmd, PC, 
            	output [31:0] newPC, result);

  reg WE3, regDst, aluSrc, memToReg, memWrite, branch, bne, jal, jr;
  reg [4:0] A1, A2, shamt;
  reg [5:0] Op, funct;
  reg [31:0] WD3;
  wire Zero, PCSrc;
  wire [4:0] A3, A3_pre;
  wire [31:0] srcA, RD, RD2, ALUResult, srcB, resultPre, result;
  wire [31:0] addr_w_offset;
  reg [2:0] ALUControl;

  sign_ext my_sign(cmd[15:0], addr_w_offset);
  dmem my_dmem(clk, memWrite, ALUResult, RD2, RD);
  reg_file my_reg(A1, A2, A3, result, clk, WE3, srcA, RD2);
  ALU my_alu(srcA, srcB, ALUControl, shamt, ALUResult, Zero);
  pc_update my_pc_update(PC, addr_w_offset, srcA, PCSrc, jal, jr, newPC);

  mux2 mux_srcB(RD2, addr_w_offset, aluSrc, srcB);
  mux2 mux_res(ALUResult, RD, memToReg, resultPre);
  mux2_5 mux_A3(cmd[20:16], cmd[15:11], regDst, A3_pre);
  mux2_5 mux_A3_final(A3_pre, 31, jal, A3);
  m_and m_and_PCSrc(branch, Zero, bne, PCSrc);
  mux2 mux_res_final(resultPre, PC+4, jal, result);
 
	
  always @(cmd)
  begin
    funct = cmd[5:0];
    A1 = cmd[25:21];
    A2 = cmd[20:16];
    shamt = cmd[10:6];
    Op = cmd[31:26];
    ALUControl <= 3'bzzz;
    aluSrc <= 1'bz;
    regDst <= 1'bz;
    memToReg <= 1'bz;
    WE3 <= 1'bz;
    branch <= 0;
    bne <= 0;
    jal <= 0;
    jr <= 0;
    memWrite <= 0;


  case (Op)
    
    6'b000000 : begin
      case (funct)
        6'b000000: begin
	  $display("cmd = %h, Op=%b, f=%b - SLL", cmd, Op, funct);
	  ALUControl <= 3'b100;
	  aluSrc <= 0;
	  regDst <= 1;
	  memToReg <= 0;
	  WE3 <= 1;
	  end 
 
        6'b000010: begin
	  $display("cmd = %h, Op=%b, f=%b - SRL", cmd, Op, funct);
	  ALUControl <= 3'b101;
	  aluSrc <= 0;
	  regDst <= 1;
	  memToReg <= 0;
	  WE3 <= 1;
	  end 

        6'b001000: begin
	  $display("cmd = %h, Op=%b, f=%b - JR", cmd, Op, funct);
	  jr <= 1;
	  end 

        6'b100000: begin
	  $display("cmd = %h, Op=%b, f=%b A1=%h, A2=%h - ADD", cmd, Op, funct, A1, A2);
	  ALUControl <= 3'b010;
   	  aluSrc <= 0;
    	  regDst <= 1;
    	  memToReg <= 0;
    	  WE3 <= 1; 
	  end 

        6'b100010: begin
	  $display("cmd = %h, Op=%b, f=%b A1=%h, A2=%h - SUB", cmd, Op, funct, A1, A2);
	  ALUControl <= 3'b110;
   	  aluSrc <= 0;
    	  regDst <= 1;
    	  memToReg <= 0;
    	  WE3 <= 1;    
	  end 

        6'b100100: begin
	  $display("cmd = %h, Op=%b, f=%b - AND", cmd, Op, funct);
	  ALUControl <= 3'b000;
   	  aluSrc <= 0;
    	  regDst <= 1;
    	  memToReg <= 0;
    	  WE3 <= 1;    
	  end 

        6'b100101: begin
	  $display("cmd = %h, Op=%b, f=%b - OR", cmd, Op, funct);
	  ALUControl <= 3'b001;
   	  aluSrc <= 0;
    	  regDst <= 1;
    	  memToReg <= 0;
    	  WE3 <= 1;    
	  end 

        6'b101010: begin
	  $display("cmd = %h, Op=%b, f=%b A1=%h, A2=%h - SLT", cmd, Op, funct, A1, A2);
	  ALUControl <= 3'b111;
   	  aluSrc <= 0;
    	  regDst <= 1;
    	  memToReg <= 0;
    	  WE3 <= 1;
	  end 

 
	default : $display("cmd = %b, UNSUPPORTED OPCODE", cmd);   
      endcase
    end

    6'b000011 : begin
      $display("cmd = %h, Op=%b, f=%b - JAL", cmd, Op, funct);
      jal <= 1;
      WE3 <= 1;
    end
    
    6'b000100 : begin
      $display("cmd = %h, Op=%b, f=%b A1=%h, A2=%h - BEQ", cmd, Op, funct, A1, A2);
      ALUControl <= 3'b110;
      aluSrc <= 0;
      branch <= 1;
    end

    6'b000101 : begin
      $display("cmd = %h, Op=%b, f=%b A1=%h, A2=%h - BNE", cmd, Op, funct, A1, A2);
      ALUControl <= 3'b110;
      aluSrc <= 0;
      branch <= 1;
      bne <= 1;
    end
    
    6'b001000 : begin
      $display("cmd = %h, Op=%b, f=%d, A1=%h, A2=%h - ADDI", cmd, Op, funct, A1, A2);
      ALUControl <= 3'b010;
      aluSrc <= 1;
      regDst <= 0;
      memToReg <= 0;
      WE3 <= 1;
    end

    6'b100011 : begin
      $display("cmd = %h, Op=%b, f=%b - LW", cmd, Op, funct);
      aluSrc <= 1;
      ALUControl <= 3'b010;
      memToReg <= 1;
      regDst <= 0;
      WE3 <= 1;
    end

    6'b101011 : begin
      $display("cmd = %h, Op=%b, f=%b - SW", cmd, Op, funct);
      aluSrc <= 1;
      ALUControl <= 3'b010;
      memWrite <= 1;
    end

    default : begin
      $display("cmd = %h, DONE! CHECK RESULT IN R3!", cmd);  
    end 
  endcase
  end
  //always @(ALUResult) $display("srcA=%d, srcB=%d, ac=%b, r=%d",srcA, srcB, ALUControl, ALUResult);
  //always @(newPC) $display("r=%d Zero=%h, PCSrc=%d, branch=%d, PC=%h, newPC=%h", ALUControl, Zero, PCSrc, branch, PC, newPC);

endmodule



