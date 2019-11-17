	addi     $1, $0, 1
	slt      $2, $0, $1
	bne	 $2, $0, labelB
	addi	 $3, $0, 10

labelA:
	addi	 $5, $0, 15
	jal 	 end

labelB:
	addi     $3, $0, 5
	slt	 $4, $0, $3
	beq	 $2, $4, labelA
	addi	 $5, $0, 25

end:
	nop
