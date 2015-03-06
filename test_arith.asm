addi $t0, $zero, 0x2
addi $t1, $zero, 0x4
mult $t1, $t0
mflo $t2
subi $t2, $t2, 1
div $t2, $t0
mflo $t3
mfhi $t4
sub $t2, $zero, $t2
div $t2, $t0
mflo $t3
mfhi $t4
