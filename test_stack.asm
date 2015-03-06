push:
addi $t1, $zero, 4
addi $sp, $sp, -4
sw $t1, 0($sp)
addi $t1, $zero, 0
pop:
lw $t1, 0($sp)
addi $sp, $sp, 4
