main:	addi $sp, $sp, -48
	addi $t0, $zero, 31
	sw $t0, 44($sp)
	addi $t0, $zero, 20
	sw $t0, 40($sp)
	addi $t0, $zero, 19
	sw $t0, 36($sp)
	addi $t0, $zero, 14
	sw $t0, 32($sp)
	addi $t0, $zero, 12
	sw $t0, 28($sp)
	addi $t0, $zero, 9
	sw $t0, 24($sp)
	addi $t0, $zero, 7
	sw $t0, 20($sp)
	addi $t0, $zero, 6
	sw $t0, 16($sp)
	addi $t0, $zero, 4
	sw $t0, 12($sp)
	addi $t0, $zero, 2
	sw $t0, 8($sp)
	addi $t0, $zero, 1
	sw $t0, 4($sp)
	addiu $a0, $sp, 4	# Parameter0 *array
	addu $a1, $zero, $zero	# Parameter1 0	//first index
	addiu $a2, $zero, 10	# Parameter2 10	//last index
	addiu $a3, $zero, 31	# Parameter3 Target Number
	jal binary
	addu $t0, $v0, $zero	# Save return value into $t0 for easy checking
	addiu $sp, $sp, 48
	shutdown
binary:	addi $sp, $sp, -8
	sw $ra, 4($sp)
	add $v0, $a1, $a2	# mid = first + last;
	srl $v1, $v0, 1		# mid /= 2;
	sw $v1, 0($sp)		# v1 = mid;
	slt $a1, $a2, $a1
	addiu $v0, $zero, 1
	beq $a1, $v0, err	# if (a1 > a2) err
	sll $v1, $v1, 2		# mid *= 4;
	addu $v1, $a0, $v1	# v1 = a + mid;
	lw $v0, 0($v1)		# v0 = *v1; // a[mid]
	beq $a3, $v0, bingo	# if (v0 == number) bingo
	slt $v0, $v0, $a3	# if (v0 < num) upper
	addiu $v1, $zero, 1
	beq $v0, $v1, upper
lower:	lw $a2, 0($sp)		# else lower
	addi $a2, $a2, -1
	jal binary
	j out
upper:	lw $a1, 0($sp)
	addi $a1, $a1, 1
	jal binary
	j out
bingo:	lw $v0, 0($sp)
	j out
err:	li $v0, -1
	j out
out:	lw $ra, 4($sp)
	addi $sp, $sp, 8
	jr $ra
