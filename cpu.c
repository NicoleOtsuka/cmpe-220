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
	/* Test immediate number: addi $t1, $zero, 0x3f */
	(OP_ADDI | RS_SET(REG_ZERO) | RT_SET(REG_T1) | IMM_SET(0x3f)),
	/* Test Register to Register: addu $t2, $t1, $zero */
	(RS_SET(REG_T1) | RT_SET(REG_ZERO) | RD_SET(REG_T2) | FUNCT_ADDU),
	/* Test Register to Memory: sw $t1, 0x0($t2) */
	(OP_SW | RS_SET(REG_T2) | RT_SET(REG_T1) | IMM_SET(0x0)),
	/* Test Memory to Register: lw $t3, 0x0($t2) */
	(OP_LW | RS_SET(REG_T2) | RT_SET(REG_T3) | IMM_SET(0x0)),
	/*
	 * Now we have value 0x3f in Mem Addr 0x3f, set another one:
	 * addi $t0, $zero, 0x3e #addr; addi $t1, $zero, 0x3 #data
	 */
	(OP_ADDI | RT_SET(REG_T0) | RS_SET(REG_ZERO) | IMM_SET(0x3e)),
	(OP_ADDI | RT_SET(REG_T1) | RS_SET(REG_ZERO) | IMM_SET(0x3)),
	/* Copy 0x3 to Mem Addr 0x3e: sw $t1, 0x0($t0) */
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

struct best_cpu {
	s32 mem[MEM_MAX];
	s32 mem_data;
	u32 mem_addr;
	s32 reg[REG_MAX];
	s32 reg_data;
	u32 reg_addr;
	u32 pc;
};

struct control_signals {
	bool mem_to_reg;
	bool mem_write;
	bool branch;
	bool alu_src;
	bool reg_dst;
	bool reg_write;
	bool jump;
	u8 alu_control;
};

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
	s32 *mem = &cpu->mem[cpu->mem_addr];
	u32 i;

	switch (mode) {
	case DUMP_ALL:
		for (i = MEM_DATA_START; i <= MEM_DATA_END; i++)
			pr_debug("\tMemory addr 0x%x, value 0x%x\n",
				 i, cpu->mem[i]);
		break;
	case DUMP_CHANGED:
		pr_debug("\tMemory addr 0x%x value changed from 0x%x to 0x%x\n",
			 cpu->mem_addr, cpu->mem_data, *mem);
	default:
		break;
	}
}

int control_unit_decoder(struct control_signals *signals, u32 opcode, u32 funct)
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
		/* TODO activate corresponding signals */
		break;
	case OP_J:
		signals->jump = true;
		break;
	case OP_JAL:
		/* TODO activate corresponding signals */
		break;
	case OP_BEQ:
		signals->branch = true;
		alu_op = ALU_OP_SUB;
		break;
	case OP_BNE:
		/* TODO activate corresponding signals */
		break;
	case OP_BLEZ:
		/* TODO activate corresponding signals */
		break;
	case OP_BGTZ:
		/* TODO activate corresponding signals */
		break;
	case OP_ADDI:
		signals->reg_write = true;
		signals->alu_src = true;
		break;
	case OP_ADDIU:
		/* TODO activate corresponding signals */
		break;
	case OP_SLTI:
		signals->reg_write = true;
		signals->alu_src = true;
		alu_op = ALU_OP_SLT;
		break;
	case OP_SLTIU:
		signals->reg_write = true;
		signals->alu_src = true;
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
		/* TODO may need to mark some flags */
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
		case FUNCT_MFHI:
			/* TODO activate the alu_control signals */
			break;
		case FUNCT_MFLO:
			/* TODO activate the alu_control signals */
			break;
		case FUNCT_MULT:
			/* TODO activate the alu_control signals */
			break;
		case FUNCT_MULTU:
			/* TODO activate the alu_control signals */
			break;
		case FUNCT_DIV:
			/* TODO activate the alu_control signals */
			break;
		case FUNCT_DIVU:
			/* TODO activate the alu_control signals */
			break;
		case FUNCT_ADD:
			signals->alu_control = ALU_CTL_ADD;
			break;
		case FUNCT_ADDU:
			signals->alu_control = ALU_CTL_ADD;
			break;
		case FUNCT_SUB:
			signals->alu_control = ALU_CTL_SUB;
			break;
		case FUNCT_SUBU:
			signals->alu_control = ALU_CTL_SUB;
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
			break;
		default:
			/* TODO may need to mark some flags */
			printf("\tUndefined insuction! funct: %x\n", funct);
			return -EINVAL;
		}
		break;
	default:
		/* TODO may need to mark some flags */
		printf("\tUndefined insuction! alu_op:%x\n", alu_op);
		return -EINVAL;
	}

	return 0;
}

/* Return signal zero as a flag for branch */
bool alu_exec(u32 control, s32 srcA, s32 srcB, s32 *result)
{
	switch (control) {
	case ALU_CTL_ADD:
		/* TODO Replace it with arithmatic operation */
		*result = srcA + srcB;
		break;
	case ALU_CTL_SUB:
		/* TODO Replace it with arithmatic operation */
		*result = srcA - srcB;
		break;
	case ALU_CTL_MUL:
		/* TODO Replace it with arithmatic operation */
		*result = srcA * srcB;
		break;
	case ALU_CTL_DIV:
		/* TODO Replace it with arithmatic operation */
		*result = srcA / srcB;
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
		*result = srcA < srcB;
		break;
	default:
		break;
	}

	return (*result == 0);
}

int running(struct best_cpu *cpu)
{
	struct control_signals signals;
	s32 mem_read_data = 0;
	s32 *reg = cpu->reg;
	s32 *mem = cpu->mem;
	u32 opcode, funct;
	s32 shamt, target;
	u32 rs, rt, rd;
	s32 alu_result;
	s32 srcA, srcB;
	s32 result;
	u32 zero;
	s32 imm;
	u32 ins;
	int ret;

	/* Start to run from boot-up address */
	for (cpu->pc = MEM_INSTR_START; cpu->pc <= MEM_INSTR_END; cpu->pc++) {
		/* Fetch the instruction being pointed by PC */
		ins = cpu->mem[cpu->pc];

		pr_debug("Executing Machine Code: 0x%x@I-MEM Addr:0x%x\n",
			 ins, cpu->pc);

		/* Pre-check: bypass for NOP; exit for SHUTDOWN */
		if (ins == NOP)
			continue;
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

		/* Parse the instruction and activate corresponding signals */
		ret = control_unit_decoder(&signals, opcode, funct);
		if (ret)
			return ret;

		srcA = reg[rs];
		srcB = signals.alu_src ? imm : reg[rt];

		zero = alu_exec(signals.alu_control, srcA, srcB, &alu_result);

		/* Back up for back trace */
		cpu->mem_addr = alu_result;
		cpu->mem_data = mem[alu_result];

		/* Memory operations */
		if (signals.mem_write)
			mem[alu_result] = reg[rt];
		else
			mem_read_data = mem[alu_result];

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

		if (signals.mem_write)
			dump_data_mem(cpu, DUMP_POLICY);

		if (signals.reg_write)
			dump_reg_file(cpu, DUMP_POLICY);
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
	free(cpu);

	return -EINVAL;
}
