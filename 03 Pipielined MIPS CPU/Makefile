
default : all

all :
	iverilog -t vvp -o test.vvp alu.v ID.v IF.v EX.v MEM.v HU.v utils.v mips.v
	vvp test.vvp
	

clean :
	rm -f *.vcd *.vvp
