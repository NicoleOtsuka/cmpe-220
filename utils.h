/*
 * CMPE-220 Projects: BEST CPU -- A software simlutor for MIPS ISA.
 *
 * Copyright (C) 2015 Nicolin Chen <nicoleotsuka@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Useful utilities header file for BEST CPU architecture.
 */

#ifndef _BEST_CPU_UTLIS_H_
#define _BEST_CPU_UTLIS_H_

typedef signed char s8;
typedef unsigned char u8;
typedef signed short s16;
typedef unsigned short u16;
typedef signed int s32;
typedef unsigned int u32;
typedef signed long long s64;
typedef unsigned long long u64;

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define pr_debug(fmt, ...) \
	do { \
		if (DEBUG) \
			printf(fmt, ##__VA_ARGS__); \
	} while (0)

#endif /* _BEST_CPU_UTLIS_H_ */
