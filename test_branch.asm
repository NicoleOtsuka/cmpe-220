addi $s0, $zero, 96
addi $s1, $zero, 128
addu $t0, $zero, $s0
loop:
sub $t1, $s1, $t0
bltz $t1, out
sw $t0, 0($t0)
addi $t0, $t0, 4
j loop
out:
sw $zero, 0($t0)
subi $t0, $t0, 4
sub $t1, $t0, $s0
bgez $t1, out
addu $s0, $zero, $zero
addu $s1, $zero, $zero
addu $t0, $zero, $zero
addu $t1, $zero, $zero
