
#define zero	$0
#define AT	$1
#define v0 	$2
#define v1	$3
#define a0	$4
#define a1	$5
#define a2	$6
#define	a3	$7

#define t0	$8
#define t1	$9
#define t2	$10
#define t3	$11
#define t4	$12
#define t5	$13
#define t6	$14
#define t7	$15
#define t8	$24
#define t9	$25
#define k0	$26
#define k1	$27

#define s0	$16
#define s1	$17
#define s2	$18
#define s3	$19
#define s4	$20
#define s5	$21
#define s6	$22
#define s7	$23

#define gp	$28
#define sp	$29
#define fp	$30
#define ra	$31

.globl start
.globl _start
.set noat
.set noreorder
.ent start

start:
_start:
	addi     $1, $0, 36
	addi     $2, $0, 60

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

nop
.end start
