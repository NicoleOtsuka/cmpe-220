addi $t1, $zero, 0x7fff
addi $t2, $zero, 1024
addi $t3, $zero, 64
mult $t1, $t2
mflo $t4
mult $t4, $t3
mflo $t4
addiu $t1, $zero, 0xffff
addu $t1, $t1, $t4
addi $t2, $zero, 0x4
add $t3, $t1, $t2
mfc0 $t0 $14
