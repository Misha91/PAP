

module ID(input clk, ForwardAD, ForwardBD, FlushE, RegWriteW,
	  input [4:0] WriteRegW,
	  input [31:0] cmd, PCPlusFourD, ResultW, ALUOutM,
	  output RegWriteE, MemtoRegE, MemWriteE, ALUSrcE, RegDstE, branch, PCSrcD,
	  output [2:0] ALUControlE,
	  output [4:0] A1, A2, RsE, RtE, RdE, shamtE,
	  output [31:0] RD1E, RD2E, SignImmD, PCBranchD);

  reg RegWriteE, MemtoRegE, MemWriteE, ALUSrcE, RegDstE, PCSrcD, EqualID;
  reg [31:0] PCBranchD, RD1E, RD2E, SignImmD;

  reg WE3, regDst, aluSrc, memToReg, memWrite, branch, bne, jal, jr, printWire;
  reg [4:0] A1, A2, RdD, shamt, shamtE, RsE, RtE, RdE, weAreFinished;
  reg [5:0] Op, funct;
  //reg [31:0] WD3;
  wire Zero, PCSrc;
  wire [4:0] A3, A3_pre;
  wire [31:0] RD1, RD2, RD1D, RD2D, ALUResult, srcB, resultPre, WD3;
  wire [31:0] SignImm;
  reg [2:0] ALUControl, ALUControlE;
  
  reg_file my_reg(printWire, A1, A2, WriteRegW, ResultW, clk, RegWriteW, RD1, RD2);
  sign_ext my_sign(cmd[15:0], SignImm);
  mux2 rd1_mux(RD1, ALUOutM, ForwardAD, RD1D);
  mux2 rd2_mux(RD2, ALUOutM, ForwardBD, RD2D);
  //always @(cmd) $display("ID %h %h", PCPlusFourD, cmd);

  initial begin
    PCSrcD = 0;
    weAreFinished = 0;
  end

  always @(posedge clk)
  begin
    RegWriteE <= FlushE ? 0 : WE3;
    MemtoRegE <= FlushE ? 0 : memToReg;
    MemWriteE <= FlushE ? 0 : memWrite;
    ALUSrcE <= FlushE ? 0 : aluSrc;
    RegDstE <= FlushE ? 0 : regDst;
    ALUControlE <= FlushE ? 0 : ALUControl;
    RsE <= FlushE ? 0 : A1;
    RtE <= FlushE ? 0 : A2;
    RdE <= FlushE ? 0 : RdD;
    shamtE <= FlushE ? 0 : shamt; 
    RD1E <= FlushE ? 0 : RD1D;
    RD2E <= FlushE ? 0 : RD2D;
    SignImmD = FlushE ? 0 : SignImm;
  
    if (cmd === 32'bX) weAreFinished <= weAreFinished + 1;
    else weAreFinished = 0;
    $display("UNKNOWN COMMAND %h", weAreFinished);

    if (weAreFinished == 5) begin
       printWire = 1;
       $finish;
    end
    //$display("ID 
  end

  always @(*)
  begin
    funct = cmd[5:0];
    A1 = cmd[25:21];
    A2 = cmd[20:16];
    shamt = cmd[10:6];
    Op = cmd[31:26];
    RdD = cmd[15:11];
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
    PCBranchD <= jal ? SignImm * 4 : PCPlusFourD + SignImm * 4;
    PCSrcD <= branch & (RD1D == RD2D ? 1 : 0);
	
    if (cmd == 0)
      printWire = 1;

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
      //$display("JAL OPERAND: %h %h %h", SignImm, PCPlusFourD, PCBranchD);
      //branch <= 1;
      //PCSrcD <= 1;
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

    default :
      begin

      //$finish(2); 
      end
  endcase
  end
endmodule
