	addi     $1, $0, 1
	jal sectionA
	addi	 $5, $4, 1
	jal sectionB

sectionA:
	addi     $4, $0, 5
	jr       ra

sectionB:
	addi	 $2, $0, 10
	addi     $3, $0, 0
