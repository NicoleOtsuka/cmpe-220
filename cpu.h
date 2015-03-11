/*
 * CMPE-220 Projects: BEST CPU -- A software simlutor for MIPS ISA.
 *
 * Copyright (C) 2015 Nicolin Chen <nicoleotsuka@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Header file for BEST CPU architecture internal definition.
 */

#ifndef _BEST_CPU_H_
#define _BEST_CPU_H_

/* 32-bit or 64-bit Insturction Set Architecture */
#define ISA_LEN		32

/* Total Register Number */
#define REG_MAX		32

#define REG_ZERO	0
#define REG_AT		1
#define REG_V0		2
#define REG_V1		3
#define REG_A0		4
#define REG_A1		5
#define REG_A2		6
#define REG_A3		7
#define REG_T0		8
#define REG_T1		9
#define REG_T2		10
#define REG_T3		11
#define REG_T4		12
#define REG_T5		13
#define REG_T6		14
#define REG_T7		15
#define REG_S0		16
#define REG_S1		17
#define REG_S2		18
#define REG_S3		19
#define REG_S4		20
#define REG_S5		21
#define REG_S6		22
#define REG_S7		23
#define REG_T8		24
#define REG_T9		25
#define REG_K0		26
#define REG_K1		27
#define REG_GP		28
#define REG_SP		29
#define REG_S8		30
#define REG_RA		31
/* TODO float Point support needs Float/Double type registers */

/* Total Memory Size */
#define MEM_MAX		256
#define MEM_DATA_END	(MEM_MAX - 4)
#define MEM_DATA_START	96
#define MEM_INSTR_END	(MEM_DATA_START - 4)
#define MEM_INSTR_START	0

/* The start address of the stack is the largest valid address */
#define MEM_STACK_START	MEM_DATA_END
#define MEM_STACK_LIMIT	188

/*
 * Machine Code Description (Starting from MSB)
 *
 * Pre-check 0x00000000 -> NOP
 *	     0xFFFFFFFF -> SHUTDOWN -- Shut down the CPU and exit
 *
 * Offset 26: 6 bit opcode:
 *	000000 -> R-type: (Check the function code field)
 *	000001 -> I-type: BGEZ -- Branch on greater than or equal to zero
 *			  TODO there are some other Branch instructions.
 *	000010 -> J-type: J -- Jump
 *	000011 -> J-type: JAL -- Jump and link
 *	000100 -> I-type: BEQ -- Branch on equal
 *	000101 -> I-type: BNE -- Branch on not equal
 *	000110 -> I-type: BLEZ -- Branch on less than or equal to zero
 *	000111 -> I-type: BGTZ -- Branch on greater than zero
 *	001000 -> I-type: ADDI -- Add immediate (with overflow)
 *	001001 -> I-type: ADDIU -- Add immediate unsigned (no overflow)
 *	001010 -> I-type: SLTI -- Set on less than immediate (signed)
 *	001011 -> I-type: SLTIU -- Set on less than immediate unsigned
 *	001100 -> I-type: ANDI -- Bitwise and immediate
 *	001101 -> I-type: ORI -- Bitwise or immediate
 *	001110 -> I-type: XORI -- Bitwise exclusive or immediate
 *	001111 -> I-type: LUI -- Load upper immediate
 *	100000 -> I-type: LB -- Load byte
 *	100011 -> I-type: LW -- Load word
 *	101000 -> I-type: SB -- Store byte
 *	101011 -> I-type: SW -- Store word
 *	110000 -> LEA: Load effective address
 *		       Structure:
 *			6-bit opcode
 *			5-bit rs
 *			5-bit rt
 *			5-bit rd
 *			5-bit shift amount
 *			6-bit displacement (function code)
 *
 * The lower 26 bits for three types:
 *	1) R-type:
 *		Offset 21: 5 bit source reg;
 *		Offset 16: 5 bit target reg;
 *		Offset 11: 5 bit destination reg;
 *		Offset 6: 5 bit shift amount;
 *		Offset 0: 6 bit function code;
 *			  000000 -> SLL -- Shift left logical
 *			  000010 -> SRL -- Shift right logical
 *			  000011 -> SRA -- Shift right arithmetic
 *			  000100 -> SLLV -- Shift left logical variable
 *			  000110 -> SRLV -- Shift right logical variable
 *			  001000 -> JR -- Jump Register
 *			  001100 -> SYSCALL -- System call
 *			  010000 -> MFHI -- Move from HI
 *			  010010 -> MFLO -- Move from Load
 *			  010011 -> MFC -- Move from EPC
 *			  011000 -> MULT -- Multiply
 *			  011001 -> MULTU -- Multiply unsigned
 *			  011010 -> DIV -- Divide
 *			  011011 -> DIVU -- Divide unsigned
 *			  100000 -> ADD – Add (with overflow)
 *			  100001 -> ADDU -- Add unsigned (no overflow)
 *			  100010 -> SUB -- Subtract
 *			  100011 -> SUBU -- Subtract unsigned
 *			  100100 -> AND -- Bitwise and
 *			  100101 -> OR -- Bitwise or
 *			  100110 -> XOR -- Bitwise exclusive OR
 *			  101010 -> SLT -- Set on less than (signed)
 *			  101011 -> SLTU -- Set on less than unsigned
 *	2) I-type:
 *		Offset 21: 5 bit source reg;
 *		Offset 16: 5 bit target reg;
 *		Offset 0: 16 bit immediate number;
 *	3) J-type:
 *		Offset 0: 26 bit target address;
 */
#define NOP		0x0
#define SHUTDOWN	0xFFFFFFFF

#define OP_WIDTH	6
#define OP_OFFSET	26
#define OP_MASK		(((1 << OP_WIDTH) - 1) << OP_OFFSET)
#define OP_RTYPE	(0x0 << OP_OFFSET)
#define OP_BLTZ		(0x1 << OP_OFFSET)
#define OP_BLTZAL	(0x1 << OP_OFFSET)
#define OP_BGEZ		(0x1 << OP_OFFSET)
#define OP_BGEZAL	(0x1 << OP_OFFSET)
#define OP_J		(0x2 << OP_OFFSET)
#define OP_JAL		(0x3 << OP_OFFSET)
#define OP_BEQ		(0x4 << OP_OFFSET)
#define OP_BNE		(0x5 << OP_OFFSET)
#define OP_BLEZ		(0x6 << OP_OFFSET)
#define OP_BGTZ		(0x7 << OP_OFFSET)
#define OP_ADDI		(0x8 << OP_OFFSET)
#define OP_ADDIU	(0x9 << OP_OFFSET)
#define OP_SLTI		(0xA << OP_OFFSET)
#define OP_SLTIU	(0xB << OP_OFFSET)
#define OP_ANDI		(0xC << OP_OFFSET)
#define OP_ORI		(0xD << OP_OFFSET)
#define OP_XORI		(0xE << OP_OFFSET)
#define OP_LUI		(0xF << OP_OFFSET)
#define OP_LB		(0x20 << OP_OFFSET)
#define OP_LW		(0x23 << OP_OFFSET)
#define OP_SB		(0x28 << OP_OFFSET)
#define OP_SW		(0x2B << OP_OFFSET)
/* LEA instruction: $rd = mem[$rs + $rt * shamt + d]; */
#define OP_LEA		(0x30 << OP_OFFSET)

#define RS_WIDTH	5
#define RS_OFFSET	21
#define RS_MASK		(((1 << RS_WIDTH) - 1) << RS_OFFSET)
#define RS_SET(x)	(((x) << RS_OFFSET) & RS_MASK)
#define RS_GET(x)	(((x) & RS_MASK) >> RS_OFFSET)

#define RT_WIDTH	5
#define RT_OFFSET	16
#define RT_MASK		(((1 << RT_WIDTH) - 1) << RT_OFFSET)
#define RT_SET(x)	(((x) << RT_OFFSET) & RT_MASK)
#define RT_GET(x)	(((x) & RT_MASK) >> RT_OFFSET)

/* Some special configurations using RT for branch insturctions */
#define RT_BGEZ		0x1
#define RT_BGEZAL	0x11
#define RT_BLTZ		0x0
#define RT_BLTZAL	0x10

#define RD_WIDTH	5
#define RD_OFFSET	11
#define RD_MASK		(((1 << RD_WIDTH) - 1) << RD_OFFSET)
#define RD_SET(x)	(((x) << RD_OFFSET) & RD_MASK)
#define RD_GET(x)	(((x) & RD_MASK) >> RD_OFFSET)

#define SHAMT_WIDTH	5
#define SHAMT_OFFSET	6
#define SHAMT_MASK	(((1 << SHAMT_WIDTH) - 1) << SHAMT_OFFSET)
#define SHAMT_SET(x)	(((x) << SHAMT_OFFSET) & SHAMT_MASK)
#define SHAMT_GET(x)	(((x) & SHAMT_MASK) >> SHAMT_OFFSET)

#define FUNCT_WIDTH	6
#define FUNCT_OFFSET	0
#define FUNCT_MASK	(((1 << FUNCT_WIDTH) - 1) << FUNCT_OFFSET)
#define FUNCT_SET(x)	(((x) << FUNCT_OFFSET) & FUNCT_MASK)
#define FUNCT_GET(x)	(((x) & FUNCT_MASK) >> FUNCT_OFFSET)
#define FUNCT_SLL	(0x0 << FUNCT_OFFSET)
#define FUNCT_SRL	(0x2 << FUNCT_OFFSET)
#define FUNCT_SRA	(0x3 << FUNCT_OFFSET)
#define FUNCT_SLLV	(0x4 << FUNCT_OFFSET)
#define FUNCT_SRLV	(0x6 << FUNCT_OFFSET)
#define FUNCT_JR	(0x8 << FUNCT_OFFSET)
#define FUNCT_SYSCALL	(0xC << FUNCT_OFFSET)
#define FUNCT_MFHI	(0x10 << FUNCT_OFFSET)
#define FUNCT_MFLO	(0x12 << FUNCT_OFFSET)
#define FUNCT_MFC	(0x13 << FUNCT_OFFSET)
#define FUNCT_MULT	(0x18 << FUNCT_OFFSET)
#define FUNCT_MULTU	(0x19 << FUNCT_OFFSET)
#define FUNCT_DIV	(0x1A << FUNCT_OFFSET)
#define FUNCT_DIVU	(0x1B << FUNCT_OFFSET)
#define FUNCT_ADD	(0x20 << FUNCT_OFFSET)
#define FUNCT_ADDU	(0x21 << FUNCT_OFFSET)
#define FUNCT_SUB	(0x22 << FUNCT_OFFSET)
#define FUNCT_SUBU	(0x23 << FUNCT_OFFSET)
#define FUNCT_AND	(0x24 << FUNCT_OFFSET)
#define FUNCT_OR	(0x25 << FUNCT_OFFSET)
#define FUNCT_XOR	(0x26 << FUNCT_OFFSET)
#define FUNCT_SLT	(0x2A << FUNCT_OFFSET)
#define FUNCT_SLTU	(0x2B << FUNCT_OFFSET)

#define IMM_WIDTH	16
#define IMM_OFFSET	0
#define IMM_MASK	(((1 << IMM_WIDTH) - 1) << IMM_OFFSET)
#define IMM_GET(x)	(((x) & IMM_MASK) >> IMM_OFFSET)
#define IMM_SET(x)	(((x) << IMM_OFFSET) & IMM_MASK)

#define TARGET_WIDTH	26
#define TARGET_OFFSET	0
#define TARGET_MASK	(((1 << TARGET_WIDTH) - 1) << TARGET_OFFSET)
#define TARGET_GET(x)	(((x) & TARGET_MASK) >> TARGET_OFFSET)
#define TARGET_SET(x)	(((x) << TARGET_OFFSET) & TARGET_MASK)

/* Control Unit Signals */
#define ALU_OP_ADD	0x0
#define ALU_OP_SUB	0x1
#define ALU_OP_MUL	0x2
#define ALU_OP_DIV	0x3
#define ALU_OP_AND	0x4
#define ALU_OP_OR	0x5
#define ALU_OP_XOR	0x6
#define ALU_OP_SLT	0x7
#define ALU_OP_RTYPE	0x10

#define ALU_CTL_AND	0x0
#define ALU_CTL_OR	0x1
#define ALU_CTL_ADD	0x2
#define ALU_CTL_MUL	0x3
#define ALU_CTL_DIV	0x4
#define ALU_CTL_XOR	0x5
#define ALU_CTL_SUB	0x6
#define ALU_CTL_SLT	0x7

#define BRANCH_BEQ	0x1
#define BRANCH_BGEZ	0x2
#define BRANCH_BGEZAL	0x3
#define BRANCH_BNE	0x4
#define BRANCH_BLEZ	0x5
#define BRANCH_BGTZ	0x6
#define BRANCH_BLTZ	0x7
#define BRANCH_BLTZAL	0x8

#endif /* _BEST_CPU_H_ */
