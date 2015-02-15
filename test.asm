addi $t1, $zero, 0x3f
addu $t2, $t1, $zero
sw $t1, 0x0($t2)
lw $t3, 0x0($t2)
addi $t0, $zero, 0x3e
addi $t1, $zero, 0x3
sw $t1, 0x0($t0)
lw $s0, 0x0($t0)
lw $s2, 0x0($t2)
sw $s2, 0x0($t0)
sw $s0, 0x0($t2)
