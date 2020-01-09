// Directives to make interresting windows visible
#pragma qtmips show registers
#pragma qtmips show memory

.globl _start
.set noat
.set noreorder

.text

_start:
 // load the word from absolute address
 addi     $1, $0, 34
 // store the word to absolute address
 addi     $2, $0, 51
loop:

 // stop execution wait for debugger/user
 //break
 // ensure that continuation does not
 // interpret random data
 beq    $1, $0, result_b
                   beq     $2, $0, result_a
                   slt $4, $1, $2 //  a < b ? 1 : 0
                   bne $4, $0, label1
                   div $1, $2
                   mfhi $1
                   j loop
label1: 
                   div $2, $1
                   mfhi $2
                   j loop
                   



              

result_a:
                   add $3, $1, $0
                   j print_res

result_b:
                   add $3, $2, $0
                   j print_res

print_res:
                   nop
     j print_res

.data
.org 0x2000

src_val:
 .word  88
                   .word  92


dst_val:
 .word  0

// Specify location to show in memory window
#pragma qtmips focus memory src_val
