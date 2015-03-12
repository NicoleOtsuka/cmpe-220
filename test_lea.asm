addi $s0, $zero, 4
addi $s1, $zero, 5
lea $t0, 2($s0, $s1, 2)	# t0 = 4 + 5 * 2 + 2 (16 - 0x10)
lead $t1, ($s0, $s1, 2)	# t1 = 4 + 5 * 2 (14 - 0xe)
lea $t2, 2($s0, $s1)	# t2 = 4 + 5 + 2 (11 - 0xb)
lead $t3, ($s0, $s1)	# t3 = 4 + 5 (9 - 0x9)
