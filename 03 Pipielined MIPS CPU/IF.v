module IF(input clk, stallF, stallD, PCSrcD, 
	  input [31:0] PC_init, PCBranchD,
	  output [31:0] cmdD, PCPlusFourD);

  wire [31:0] cmd, newPC;
  reg [31:0] PC, PCplusFour, cmdD, PCPlusFourD;


  always @(PC_init) begin
	PC = PC_init;
        
  end

	
  always @(posedge clk)
  begin
    PC = stallF ? PC : newPC;
    
    cmdD = PCSrcD ? 0 : (stallD ? cmdD : cmd); 
    PCPlusFourD = PCSrcD ? 0 : (stallD ? PCPlusFourD : PCplusFour); 

    $display("IM pos clk: %h %h %h %h %h %h %h", PC, stallF, stallD, PCSrcD, PCBranchD, cmdD, PCPlusFourD);


  end    

  always @(PC) PCplusFour <= PC + 4;

  imem imem_mod(PC[7:2], cmd);
  mux2 pc_upd(PCplusFour, PCBranchD, PCSrcD, newPC);

endmodule

