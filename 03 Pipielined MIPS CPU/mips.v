//Test bench module
module mips_tb;
  reg [31:0] PC;
  reg clk;

  mips_pipelined my_mips(clk, PC);

  initial begin 
    $dumpfile("test.vcd");
    $dumpvars;
    clk = 0; 
    PC = 0; 
    #250 $finish;
  end  

  always #10 clk = ~clk;
endmodule



module mips_pipelined(input clk,
	    input [31:0] PC_init);

  reg stallF, stallD, ForwardAD, ForwardBD, FlushE;
  reg [1:0] ForwardAE, ForwardBE;
  //reg [4:0] ;
  //reg [31:0] ;
  wire RegWriteD, MemtoRegD, MemWriteE, ALUSrcD, RegDstD, BranchD, PCSrcD, RegWriteM, MemtoRegM, MemWriteM, RegWriteW, MemtoRegW;
  wire [2:0] ALUControlD;
  wire [4:0] RsD, RtD, RdE, shamtE, WriteRegM, WriteRegW;
  wire[31:0] cmdD, PCPlusFourD, RD1D, RD2D, SignImmD, PCBranchD, ALUOutM, WriteDataM, ALUOutW, ReadDataW, ResultW;

  initial begin
    stallF = 0;
    stallD = 0; 
    //ResultW = 0;
    //WriteRegW = 0;
    //PCSrcD = 0;
    //PCBranchD = 0;
    //RegWriteW = 0;
    ForwardAD = 0;
    ForwardBD = 0;
    ForwardAE = 0;
    ForwardBE = 0;
    FlushE = 0;
  end

  IF my_IF(clk, stallF, stallD, PCSrcD, PC_init, PCBranchD, cmdD, PCPlusFourD);
  ID my_ID(clk, ForwardAD, ForwardBD, FlushE, RegWriteW, WriteRegW, cmdD, PCPlusFourD, ResultW, RegWriteD, MemtoRegD, MemWriteE, ALUSrcD, RegDstD, PCSrcD, ALUControlD, RsD, RtD, RdE, shamtE, RD1D, RD2D, SignImmD, PCBranchD);
  EX my_EX(clk, RegWriteD, MemtoRegD, MemWriteE, ALUSrcD, RegDstD, ForwardAE, ForwardBE, ALUControlD, RsD, RtD, RdE, shamtE, RD1D, RD2D, SignImmD, RegWriteM, MemtoRegM, MemWriteM, WriteRegM, ALUOutM, WriteDataM);
  MEM my_MEM(clk, RegWriteM, MemtoRegM, MemWriteM, WriteRegM, ALUOutM, WriteDataM, RegWriteW, MemtoRegW, WriteRegW, ALUOutW, ReadDataW);

  mux2 write(ALUOutW, ReadDataW, MemtoRegW, ResultW);
	

endmodule




/*

Main MIPS module - has processor submodule, 
requires clk and initial PC to start. Updates
PC on every clk.

*/

/*

module mips(input clk,
	    input [31:0] PC_init);

  wire [31:0] cmd, result, newPC;
  reg [31:0] PC;

  always @(PC_init) PC = PC_init;


 
  always @(posedge clk)
  begin
      PC <= newPC;
  end      
  processor my_ctrl(clk, cmd, PC, newPC, result);
endmodule
*/


/*

Main module. Performs all required computation, 
updates register file and data memory. It has 
$display statement for debug easiness.
 
*/

/*
module processor(input clk,
	    	input [31:0] cmd, PC, 
            	output [31:0] newPC, 
                output [31:0] result);

  reg WE3, regDst, aluSrc, memToReg, memWrite, branch, bne, jal, jr, printWire;
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
  reg_file my_reg(printWire, A1, A2, A3, result, clk, WE3, srcA, RD2);
  ALU my_alu(srcA, srcB, ALUControl, shamt, ALUResult, Zero);
  pc_update my_pc_update(PC, addr_w_offset, srcA, PCSrc, jal, jr, newPC);

  mux2 mux_srcB(RD2, addr_w_offset, aluSrc, srcB);
  mux2 mux_res(ALUResult, RD, memToReg, resultPre);
  mux2_5 mux_A3(cmd[20:16], cmd[15:11], regDst, A3_pre);
  mux2_5 mux_A3_final(A3_pre, 31, jal, A3);
  m_and m_and_PCSrc(branch, Zero, bne, PCSrc);
  mux2 mux_res_final(resultPre, PC+4, jal, result);
  
    imem imem_mod(PC[7:2], cmd);
	
  always @(*)
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
    printWire <= 0;


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
      if ((PC != 0) ) 
        begin
          printWire = 1;
          $finish(2); 
        end
    end 
  endcase
  end

endmodule
*/


