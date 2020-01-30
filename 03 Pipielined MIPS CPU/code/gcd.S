	#include "inputvars.def"
	addi     $2, $0, srcB
	addi     $1, $0, srcA

loop:
	beq    $1, $0, result_b
	beq     $2, $0, result_a
	slt $4, $1, $2 //  a < b ? 1 : 0
	bne $4, $0, divB

divA:
	sub $1, $1, $2
	slt $5, $1, $2
	beq $5, $0, divA
	jal loop

divB: 
	sub $2, $2, $1
	slt $5, $2, $1
	beq $5, $0, divB
	jal loop
                  

result_a:
	add $3, $1, $0
	jal print_res

result_b:
	add $3, $2, $0
	jal print_res

print_res:
	nop
	//jal print_res
