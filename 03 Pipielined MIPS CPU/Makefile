
default : all

all :
	iverilog -t vvp -o test.vvp alu.v utils.v mips.v
	vvp test.vvp
	

clean :
	rm -f *.vcd *.vvp
