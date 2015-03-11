/*
 * CMPE-220 Projects: BEST CPU -- A software simlutor for MIPS ISA.
 *
 * Copyright (C) 2015 Nicolin Chen <nicoleotsuka@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

/* Activate Debug Information: '#define DEBUG 1'; Deactivate: '0' */
#define DEBUG 1
/* DEBUG_LEVEL from 0 (highest) to 5 (lowest)*/
#define DEBUG_LEVEL 0

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cpu.h"
#include "utils.h"

#define DUMP_CHANGED		0
#define DUMP_ALL		1

#define DUMP_POLICY		(DEBUG_LEVEL == 0 ? DUMP_ALL : DUMP_CHANGED)

static u32 test[24] = {
	/* Test immediate number: addi $t1, $zero, 0x7c */
	(OP_ADDI | RS_SET(REG_ZERO) | RT_SET(REG_T1) | IMM_SET(0x7c)),
	/* Test Register to Register: addu $t2, $t1, $zero */
	(RS_SET(REG_T1) | RT_SET(REG_ZERO) | RD_SET(REG_T2) | FUNCT_ADDU),
	/* Test Register to Memory: sw $t1, 0x0($t2) */
	(OP_SW | RS_SET(REG_T2) | RT_SET(REG_T1) | IMM_SET(0x0)),
	/* Test Memory to Register: lw $t3, 0x0($t2) */
	(OP_LW | RS_SET(REG_T2) | RT_SET(REG_T3) | IMM_SET(0x0)),
	/*
	 * Now we have value 0x7c in Mem Addr 0x7c, set another one:
	 * addi $t0, $zero, 0x78 #addr; addi $t1, $zero, 0x3 #data
	 */
	(OP_ADDI | RT_SET(REG_T0) | RS_SET(REG_ZERO) | IMM_SET(0x78)),
	(OP_ADDI | RT_SET(REG_T1) | RS_SET(REG_ZERO) | IMM_SET(0x3)),
	/* Copy 0x3 to Mem Addr 0x78: sw $t1, 0x0($t0) */
	(OP_SW | RS_SET(REG_T0) | RT_SET(REG_T1) | IMM_SET(0x0)),
	/*
	 * Swap them: (Note: $t2, $t0 are addresses)
	 * lw $s0, 0x0($t0); lw $s2, 0x0($t2)
	 * sw $s2, 0x0($t0); sw $s0, 0x0($t2)
	 */
	(OP_LW | RS_SET(REG_T0) | RT_SET(REG_S0) | IMM_SET(0x0)),
	(OP_LW | RS_SET(REG_T2) | RT_SET(REG_S2) | IMM_SET(0x0)),
	(OP_SW | RS_SET(REG_T0) | RT_SET(REG_S2) | IMM_SET(0x0)),
	(OP_SW | RS_SET(REG_T2) | RT_SET(REG_S0) | IMM_SET(0x0)),
	/* Pause one cycle */
	(NOP),
	/* Test logic operations */
	(FUNCT_OR | RS_SET(REG_T0) | RT_SET(REG_T1) | RD_SET(REG_T4)),
	(FUNCT_AND | RS_SET(REG_T0) | RT_SET(REG_T1) | RD_SET(REG_T5)),
	(FUNCT_XOR | RS_SET(REG_T0) | RT_SET(REG_T1) | RD_SET(REG_T6)),
	(FUNCT_SLT | RS_SET(REG_T0) | RT_SET(REG_T1) | RD_SET(REG_T7)),
	(FUNCT_SLT | RS_SET(REG_T1) | RT_SET(REG_T0) | RD_SET(REG_T7)),
	(OP_ORI | RS_SET(REG_T0) | RT_SET(REG_T4) | IMM_SET(0x3)),
	(OP_ANDI | RS_SET(REG_T0) | RT_SET(REG_T5) | IMM_SET(0x3)),
	(OP_XORI | RS_SET(REG_T0) | RT_SET(REG_T6) | IMM_SET(0x3)),
	(OP_SLTI | RS_SET(REG_T0) | RT_SET(REG_T7) | IMM_SET(0x3)),
	(OP_SLTI | RS_SET(REG_T0) | RT_SET(REG_T7) | IMM_SET(0x4f)),
	/* SHUTDOWN CPU */
	0xffffffff,
};

struct best_cpu_flags {
	bool zero;
	bool sign;
	bool carry;
	bool segment;
	bool overflow;
	bool divbyzero;
	bool unknowinstr;
};

struct best_cpu {
	struct best_cpu_flags flags;
	s32 mem[MEM_MAX / 4];
	s32 mem_data;
	u32 mem_addr;
	s32 reg_lo, reg_hi;
	s32 reg[REG_MAX];
	u32 reg_epc;
	s32 reg_data;
	u32 reg_addr;
	u32 pc;
};

/**
 * Data structure for control signals
 *
 * @mem_to_reg: Selects the data path from data memory instead of from ALU
 * @mem_write: Enables memory write operation for SW like instructions
 * @alu_src: Selects immediate number to take part in ALU operations
 * @reg_dst: Selects the 3rd register in the instructions to update (R-type)
 * @reg_write: Enables register write operation
 * @jump: Generates a separate signal to enable jump instruction
 * @alu_unsigned: Lets alu do unsigned operations
 * @alu_control: Selects a specific function of ALU (ADD, SUB, MULT and etc.)
 * @branch: Enable branch operations
 */
struct control_signals {
	bool mem_to_reg;
	bool mem_write;
	bool alu_src;
	bool reg_dst;
	bool reg_write;
	bool jump;
	bool alu_unsigned;
	u8 alu_control;
	u8 branch;
};

static void dump_cpu_flags(struct best_cpu_flags *flags)
{
	if (flags->zero)
		pr_debug("\tZero flag raised\n");
	if (flags->sign)
		pr_debug("\tSign flag raised\n");
	if (flags->carry)
		pr_debug("\tCarry flag raised\n");
	if (flags->segment)
		pr_debug("\tSegment flag raised\n");
	if (flags->overflow)
		pr_debug("\tOverflow flag raised\n");
	if (flags->divbyzero)
		pr_debug("\tDivided by zero flag raised\n");
	if (flags->unknowinstr)
		pr_debug("\tUnknown instruction flag raised\n");
}

static void dump_reg_file(struct best_cpu *cpu, int mode)
{
	s32 *reg = &cpu->reg[cpu->reg_addr];
	u32 i;

	switch (mode) {
	case DUMP_ALL:
		for (i = 0; i < REG_MAX; i++)
			pr_debug("\tRegister 0x%x, value 0x%x\n",
				 i, cpu->reg[i]);
		break;
	case DUMP_CHANGED:
		pr_debug("\tRegister 0x%x value changed from 0x%x to 0x%x\n",
			 cpu->reg_addr, cpu->reg_data, *reg);
	default:
		break;
	}
}

static void dump_data_mem(struct best_cpu *cpu, int mode)
{
	s32 *mem = &cpu->mem[cpu->mem_addr / 4];
	int i;

	switch (mode) {
	case DUMP_ALL:
		for (i = MEM_DATA_END; i >= 0; i -= 4) {
			if (i == MEM_STACK_START)
				pr_debug("------Stack Memory Starts\n");

			pr_debug("\tMemory addr 0x%x, value 0x%x\n",
				 i, cpu->mem[i / 4]);

			if (i == MEM_INSTR_START)
				pr_debug("------Instruction Memory Starts\n");
			else if (i == MEM_DATA_START)
				pr_debug("------Data Memory Starts (Heap)\n");
			else if (i == MEM_STACK_LIMIT)
				pr_debug("------Stack Memory Limits\n");
		}
		break;
	case DUMP_CHANGED:
		pr_debug("\tMemory addr 0x%x value changed from 0x%x to 0x%x\n",
			 cpu->mem_addr, cpu->mem_data, *mem);
	default:
		break;
	}
}

int control_unit_decoder(struct best_cpu_flags *flags,
			 struct control_signals *signals,
			 u32 opcode, u32 funct, u32 rt)
{
	u8 alu_op = ALU_OP_ADD;

	/* Reset all control signals */
	memset(signals, 0, sizeof(*signals));

	/* Main Decoder */
	switch (opcode) {
	case OP_RTYPE:
		signals->reg_write = true;
		signals->reg_dst = true;
		alu_op = ALU_OP_RTYPE;
		break;
	case OP_BGEZ:
		switch (rt) {
		case RT_BGEZ:
			signals->branch = BRANCH_BGEZ;
			break;
		case RT_BGEZAL:
			signals->branch = BRANCH_BGEZAL;
			break;
		case RT_BLTZ:
			signals->branch = BRANCH_BLTZ;
			break;
		case RT_BLTZAL:
			signals->branch = BRANCH_BLTZAL;
			break;
		default:
			flags->unknowinstr = true;
			printf("\tUndefined insuction! branch RT: %x\n", rt);
			return -EINVAL;
		}
		alu_op = ALU_OP_SUB;
		break;
	case OP_J:
		signals->jump = true;
		break;
	case OP_JAL:
		signals->jump = true;
		break;
	case OP_BEQ:
		signals->branch = BRANCH_BEQ;
		alu_op = ALU_OP_SUB;
		break;
	case OP_BNE:
		signals->branch = BRANCH_BNE;
		alu_op = ALU_OP_SUB;
		break;
	case OP_BLEZ:
		signals->branch = BRANCH_BLEZ;
		alu_op = ALU_OP_SUB;
		break;
	case OP_BGTZ:
		signals->branch = BRANCH_BGTZ;
		alu_op = ALU_OP_SUB;
		break;
	case OP_ADDI:
		signals->reg_write = true;
		signals->alu_src = true;
		alu_op = ALU_OP_ADD;
		break;
	case OP_ADDIU:
		signals->reg_write = true;
		signals->alu_src = true;
		signals->alu_unsigned = true;
		alu_op = ALU_OP_ADD;
		break;
	case OP_SLTI:
		signals->reg_write = true;
		signals->alu_src = true;
		alu_op = ALU_OP_SLT;
		break;
	case OP_SLTIU:
		signals->reg_write = true;
		signals->alu_src = true;
		signals->alu_unsigned = true;
		alu_op = ALU_OP_SLT;
		break;
	case OP_ANDI:
		signals->reg_write = true;
		signals->alu_src = true;
		alu_op = ALU_OP_AND;
		break;
	case OP_ORI:
		signals->reg_write = true;
		signals->alu_src = true;
		alu_op = ALU_OP_OR;
		break;
	case OP_XORI:
		signals->reg_write = true;
		signals->alu_src = true;
		alu_op = ALU_OP_XOR;
		break;
	case OP_LUI:
		/* TODO activate corresponding signals */
		break;
	case OP_LB:
		/* TODO activate corresponding signals */
		break;
	case OP_LW:
		signals->reg_write = true;
		signals->alu_src = true;
		signals->mem_to_reg = true;
		break;
	case OP_SB:
		/* TODO activate corresponding signals */
		break;
	case OP_SW:
		signals->alu_src = true;
		signals->mem_write = true;
		break;
	default:
		flags->unknowinstr = true;
		printf("\tUndefined insuction! opcode: %x\n", opcode);
		return -EINVAL;
	}

	/* ALU Decoder */
	switch (alu_op) {
	case ALU_OP_ADD:
		signals->alu_control = ALU_CTL_ADD;
		break;
	case ALU_OP_SUB:
		signals->alu_control = ALU_CTL_SUB;
		break;
	case ALU_OP_MUL:
		signals->alu_control = ALU_CTL_MUL;
		break;
	case ALU_OP_DIV:
		signals->alu_control = ALU_CTL_DIV;
		break;
	case ALU_OP_AND:
		signals->alu_control = ALU_CTL_AND;
		break;
	case ALU_OP_OR:
		signals->alu_control = ALU_CTL_OR;
		break;
	case ALU_OP_XOR:
		signals->alu_control = ALU_CTL_XOR;
		break;
	case ALU_OP_SLT:
		signals->alu_control = ALU_CTL_SLT;
		break;
	case ALU_OP_RTYPE:
		switch (funct) {
		case FUNCT_SLL:
			/* TODO activate the alu_control signals */
			break;
		case FUNCT_SRL:
			/* TODO activate the alu_control signals */
			break;
		case FUNCT_SRA:
			/* TODO activate the alu_control signals */
			break;
		case FUNCT_SLLV:
			/* TODO activate the alu_control signals */
			break;
		case FUNCT_SRLV:
			/* TODO activate the alu_control signals */
			break;
		case FUNCT_JR:
			/* TODO activate the alu_control signals */
			break;
		case FUNCT_SYSCALL:
			/* TODO activate the alu_control signals */
			break;
		case FUNCT_MULT:
			signals->reg_write = false;
			signals->reg_dst = false;
			signals->alu_control = ALU_CTL_MUL;
			break;
		case FUNCT_MULTU:
			signals->reg_write = false;
			signals->reg_dst = false;
			signals->alu_control = ALU_CTL_MUL;
			signals->alu_unsigned = true;
			break;
		case FUNCT_DIV:
			signals->reg_write = false;
			signals->reg_dst = false;
			signals->alu_control = ALU_CTL_DIV;
			break;
		case FUNCT_DIVU:
			signals->reg_write = false;
			signals->reg_dst = false;
			signals->alu_control = ALU_CTL_DIV;
			signals->alu_unsigned = true;
			break;
		case FUNCT_ADD:
			signals->alu_control = ALU_CTL_ADD;
			break;
		case FUNCT_ADDU:
			signals->alu_control = ALU_CTL_ADD;
			signals->alu_unsigned = true;
			break;
		case FUNCT_SUB:
			signals->alu_control = ALU_CTL_SUB;
			break;
		case FUNCT_SUBU:
			signals->alu_control = ALU_CTL_SUB;
			signals->alu_unsigned = true;
			break;
		case FUNCT_AND:
			signals->alu_control = ALU_CTL_AND;
			break;
		case FUNCT_OR:
			signals->alu_control = ALU_CTL_OR;
			break;
		case FUNCT_XOR:
			signals->alu_control = ALU_CTL_XOR;
			break;
		case FUNCT_SLT:
			signals->alu_control = ALU_CTL_SLT;
			break;
		case FUNCT_SLTU:
			signals->alu_control = ALU_CTL_SLT;
			signals->alu_unsigned = true;
			break;
		default:
			flags->unknowinstr = true;
			printf("\tUndefined insuction! funct: %x\n", funct);
			return -EINVAL;
		}
		break;
	default:
		flags->unknowinstr = true;
		printf("\tUndefined insuction! alu_op:%x\n", alu_op);
		return -EINVAL;
	}

	return 0;
}

void control_unit_branch_pre(struct best_cpu *cpu, u8 branch, s32 *srcB)
{
	switch (branch) {
	case BRANCH_BGEZAL:
	case BRANCH_BLTZAL:
		cpu->reg[REG_RA] = cpu->pc + 8;
	case BRANCH_BGEZ:
	case BRANCH_BLEZ:
	case BRANCH_BGTZ:
	case BRANCH_BLTZ:
		*srcB = 0;
		break;
	default:
		break;
	}
}

bool contorl_unit_branch_post(u8 branch, bool zero, bool sign)
{
	bool ret = false;

	switch (branch) {
	case BRANCH_BEQ:
		if (zero)
			ret = true;
		break;
	case BRANCH_BGEZ:
	case BRANCH_BGEZAL:
		if (!sign)
			ret = true;
		break;
	case BRANCH_BNE:
		if (!zero)
			ret = true;
		break;
	case BRANCH_BLEZ:
		if (sign || zero)
			ret = true;
		break;
	case BRANCH_BGTZ:
		if (!sign && !zero)
			ret = true;
		break;
	case BRANCH_BLTZ:
	case BRANCH_BLTZAL:
		if (sign)
			ret = true;
		break;
	default:
		break;
	}

	return ret;
}

bool alu_exec_add(s32 srcA, s32 srcB, s32 *result)
{
	int c = 0;

	while (srcB) {
		c = srcA & srcB;
		srcA ^= srcB;
		srcB = c << 1;
	}

	*result = srcA;

	return (c != 0);
}

bool alu_exec_sub(s32 srcA, s32 srcB, s32 *result)
{
	int c = 0;

	*result = ~srcB;
	c = alu_exec_add(1, *result, result);
	c = alu_exec_add(srcA, *result, result);

	return (c != 0);
}

bool alu_exec_mul(s32 srcA, s32 srcB, s32 *reg_lo)
{
	*reg_lo = 0;

	while (srcB) {
		if (srcB & 0x1)
			alu_exec_add(*reg_lo, srcA, reg_lo);
		srcA <<= 1;
		srcB >>= 1;
	}

	return (*reg_lo == 0);
}

bool alu_exec_div(bool alu_unsigned, s32 srcA, s32 srcB,
		  s32 *reg_lo, s32 *reg_hi)
{
	s32 temp1, temp2, temp3;
	bool signedA = false;
	bool signedB = false;
	s32 remainder = 0;
	s32 quotient = 0;
	s32 buffer;
	s32 i;

	/* Record the sign bits and make operands positive */
	if (!alu_unsigned) {
		if (srcA < 0) {
			alu_exec_sub(0, srcA, &srcA);
			signedA = true;
		}
		if (srcB < 0) {
			alu_exec_sub(0, srcB, &srcB);
			signedB = true;
		}
	}

	for (i = 0; i < ISA_LEN; i++) {
		remainder = (remainder << 1) | ((srcA >> (ISA_LEN - 1)) & 0x1);
		if (remainder < srcB) {
			quotient = (quotient << 1) | 0x0;
		} else {
			quotient = (quotient << 1) | 0x1;

			buffer = ~srcB;
			temp3 = 1;
			do {
				temp1 = buffer & temp3;
				temp2 = buffer ^ temp3;
				buffer = temp1 << 1;
				temp3 = temp2;
			} while (temp1);

			buffer = temp2;

			do {
				temp1 = remainder & buffer;
				temp2 = remainder ^ buffer;
				remainder = temp1 << 1;
				buffer = temp2;
			} while (temp1);
			remainder = temp2;
		}
		srcA = srcA << 1;
	}

	/* Restore the sign bit for remainder and result */
	if (signedA)
		alu_exec_sub(0, remainder, reg_hi);
	else
		*reg_hi = remainder;

	if ((signedA && signedB) || (!signedA && !signedB))
		*reg_lo = quotient;
	else
		alu_exec_sub(0, quotient, reg_lo);

	return (*reg_lo == 0);
}

void alu_exec(struct best_cpu_flags *flags, bool alu_unsigned, u32 control,
	      s32 srcA, s32 srcB, s32 *result, s32 *reg_lo, s32 *reg_hi)
{
	bool carry = false;
	bool zero = false;

	switch (control) {
	case ALU_CTL_ADD:
		/* Bitwise addition does not care about sign */
		carry = alu_exec_add(srcA,  srcB, result);
		break;
	case ALU_CTL_SUB:
		/* Bitwise subtraction does not care about sign */
		carry = alu_exec_sub(srcA,  srcB, result);
		break;
	case ALU_CTL_MUL:
		/* Bitwise multiplication does not care about sign */
		zero = alu_exec_mul(srcA,  srcB, reg_lo);
		break;
	case ALU_CTL_DIV:
		/* Bypass divided by zero */
		if (srcB == 0) {
			flags->divbyzero = true;
			return;
		}
		zero = alu_exec_div(alu_unsigned, srcA,  srcB, reg_lo, reg_hi);
		break;
	case ALU_CTL_AND:
		*result = srcA & srcB;
		break;
	case ALU_CTL_OR:
		*result = srcA | srcB;
		break;
	case ALU_CTL_XOR:
		*result = srcA ^ srcB;
		break;
	case ALU_CTL_SLT:
		if (alu_unsigned)
			*result = (u32)srcA < (u32)srcB;
		else
			*result = srcA < srcB;
		break;
	default:
		break;
	}

	flags->zero = (*result == 0) | zero;
	flags->sign = *result < 0;
	if (alu_unsigned)
		flags->carry = carry;
	flags->overflow = (srcA > 0 && srcB > 0 && *result < 0) ||
			  (srcA < 0 && srcB < 0 && *result >= 0);

	return;
}

int running(struct best_cpu *cpu)
{
	struct best_cpu_flags *flags = &cpu->flags;
	struct control_signals signals;
	bool pc_source = false;
	s32 mem_read_data = 0;
	s32 *reg = cpu->reg;
	s32 *mem = cpu->mem;
	u32 opcode, funct;
	s32 shamt, target;
	u32 rs, rt, rd;
	s32 alu_result;
	s32 srcA, srcB;
	s32 result;
	s32 imm;
	u32 ins;
	int ret;

	/* Initialize stack pointer and program counter */
	cpu->reg[REG_SP] = MEM_STACK_START;
	cpu->pc = MEM_INSTR_START;

	/* Start to run from boot-up address */
	while (cpu->pc <= MEM_INSTR_END) {
		/* Fetch the instruction being pointed by PC */
		ins = cpu->mem[cpu->pc / 4];
		/* Reset all flags and signals */
		memset(flags, 0, sizeof(*flags));
		memset(&signals, 0, sizeof(signals));

		pr_debug("Executing Machine Code: 0x%x@I-MEM Addr:0x%x\n",
			 ins, cpu->pc);

		/* Pre-check: bypass for NOP; exit for SHUTDOWN */
		if (ins == NOP) {
			cpu->pc += 4;
			continue;
		}

		else if (ins == SHUTDOWN)
			return 0;

		/* Extract all fields from the instruction */
		opcode = ins & OP_MASK;
		rs = RS_GET(ins);
		rt = RT_GET(ins);
		rd = RD_GET(ins);
		shamt = SHAMT_GET(ins);
		funct = FUNCT_GET(ins);
		imm = IMM_GET(ins);
		target = TARGET_GET(ins);

		if (opcode == OP_J || opcode == OP_JAL) {
			if (opcode == OP_JAL)
				cpu->reg[REG_RA] = cpu->pc + 8;
			cpu->pc = ((cpu->pc + 4) & 0xf0000000) | (target << 2);
			pr_debug("\tJump to new pc: 0x%x\n", cpu->pc);
			continue;
		}

		/* Direct copy for EPC, LO and HI registers */
		switch (opcode | funct) {
		case FUNCT_MFC:
			cpu->reg_addr = rd;
			cpu->reg_data = cpu->reg[rd];
			signals.reg_write = true;
			reg[rd] = cpu->reg_epc;
			goto out_of_stage;
		case FUNCT_MFLO:
			cpu->reg_addr = rd;
			cpu->reg_data = cpu->reg[rd];
			signals.reg_write = true;
			reg[rd] = cpu->reg_lo;
			goto out_of_stage;
		case FUNCT_MFHI:
			cpu->reg_addr = rd;
			cpu->reg_data = cpu->reg[rd];
			signals.reg_write = true;
			reg[rd] = cpu->reg_hi;
			goto out_of_stage;
		default:
			break;
		}

		/* Parse the instruction and activate corresponding signals */
		ret = control_unit_decoder(flags, &signals, opcode, funct, rt);
		if (ret)
			return ret;

		/* Sign extension for immediate number */
		if (!signals.alu_unsigned && (imm & 0x8000))
			imm |= 0xffff0000;

		srcA = reg[rs];
		srcB = signals.alu_src ? imm : reg[rt];

		/* Override srcB for some branch instructions */
		control_unit_branch_pre(cpu, signals.branch, &srcB);

		alu_exec(flags, signals.alu_unsigned, signals.alu_control,
			 srcA, srcB, &alu_result, &cpu->reg_lo, &cpu->reg_hi);

		pc_source = contorl_unit_branch_post(signals.branch,
						     flags->zero, flags->sign);

		/* Branch taken */
		if (pc_source) {
			cpu->pc += 4 + (imm << 2);
			pr_debug("\tBranch taken to new pc: 0x%x\n", cpu->pc);
			continue;
		}

		/* Store the instruction causing overflow in register epc */
		if (flags->overflow) {
			cpu->reg_epc = cpu->pc;
			pr_debug("Insuction (%x) @ MemAddr %x casued overflow!\n",
				 ins, cpu->pc);
		}

		/* Back up for back trace */
		if (signals.mem_write || signals.mem_to_reg) {
			cpu->mem_addr = alu_result;
			cpu->mem_data = mem[alu_result / 4];
		}

		if ((signals.mem_write || signals.mem_to_reg) &&
		    alu_result < MEM_DATA_START)
			flags->segment = true;

		if (flags->segment)
			return -EINVAL;

		/* Memory operations */
		if (signals.mem_write)
			mem[alu_result / 4] = reg[rt];
		else if (signals.mem_to_reg)
			mem_read_data = mem[alu_result / 4];

		result = signals.mem_to_reg ? mem_read_data : alu_result;

		/* Write back to Register */
		if (signals.reg_write) {
			/* Back up for back trace */
			cpu->reg_addr = signals.reg_dst ? rd : rt;
			cpu->reg_data = reg[cpu->reg_addr];

			/* Bypass writing $zero -- READ ONLY */
			if (cpu->reg_addr == REG_ZERO)
				break;

			/* Overwrite the register */
			reg[cpu->reg_addr] = result;
		}

		if (reg[REG_SP] < MEM_STACK_LIMIT ||
		    reg[REG_SP] > MEM_STACK_START) {
			printf("\tSegmentation fault!\n");
			flags->segment = true;
		}

out_of_stage:
		if (signals.mem_write)
			dump_data_mem(cpu, DUMP_POLICY);

		if (signals.reg_write)
			dump_reg_file(cpu, DUMP_POLICY);

		dump_cpu_flags(flags);

		/* Normally increase program counter */
		cpu->pc += 4;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	struct best_cpu *cpu;
	FILE *codefile;
	char code[64];
	int i = 0, ret;

	cpu = malloc(sizeof(*cpu));
	if (!cpu)
		return -ENOMEM;

	memset(cpu, 0, sizeof(*cpu));

	if (argc > 1) {
		codefile = fopen(argv[1], "r");
		if (!codefile)
			return -ENOENT;

		/* TODO add some code to detect if's valid machine code */

		/* Read machine code from input file */
		while (fgets(code, sizeof(code), codefile))
			cpu->mem[i++] = strtol(code, NULL, 16);
	} else {
		i = MEM_INSTR_START;
		/* Initialize insuctions into boot-up Memory location */
		for (; i < ARRAY_SIZE(test) && i <= MEM_INSTR_END; i++)
			cpu->mem[i] = test[i];
	}

	ret = running(cpu);
	if (ret)
		goto err;

	printf("======Shutting down CPU\n");

	free(cpu);

	return 0;

err:
	printf("======CPU trapped!\n");
	if (cpu->flags.segment)
		printf("\tCPU: Segmentation Fault!\n");
	free(cpu);

	return -EINVAL;
}
