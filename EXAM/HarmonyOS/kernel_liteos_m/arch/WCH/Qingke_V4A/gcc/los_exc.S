/*
 * Copyright (c) 2023, smartmx <smartmx@qq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-08-28     smartmx      the first version
 *
 */

#ifndef _LOS_EXC_S
#define _LOS_EXC_S

#include "riscv_encoding.h"

.extern HalTaskSwitch

.section    .highcode.SW_Handler, "ax", @progbits
.global SW_Handler
.align 2
.func
SW_Handler:

    addi sp, sp, -( 32 * 4 )
    sw x1,  1  * 4(sp)    /* RA */
    sw x5,  2  * 4(sp)
    sw x6,  3  * 4(sp)
    sw x7,  4  * 4(sp)
    sw x8,  5  * 4(sp)
    sw x9,  6  * 4(sp)
    sw x10, 7  * 4(sp)
    sw x11, 8  * 4(sp)
    sw x12, 9  * 4(sp)
    sw x13, 10 * 4(sp)
    sw x14, 11 * 4(sp)
    sw x15, 12 * 4(sp)
    sw x16, 13 * 4(sp)
    sw x17, 14 * 4(sp)
    sw x18, 15 * 4(sp)
    sw x19, 16 * 4(sp)
    sw x20, 17 * 4(sp)
    sw x21, 18 * 4(sp)
    sw x22, 19 * 4(sp)
    sw x23, 20 * 4(sp)
    sw x24, 21 * 4(sp)
    sw x25, 22 * 4(sp)
    sw x26, 23 * 4(sp)
    sw x27, 24 * 4(sp)
    sw x28, 25 * 4(sp)
    sw x29, 26 * 4(sp)
    sw x30, 27 * 4(sp)
    sw x31, 28 * 4(sp)

    sw x4, 29 * 4( sp )				/* tp - not sure it is useful */

    /* Store sp to task stack */
    la t0, g_losTask
    lw t0, 0(t0)
    sw sp, 0(t0)

    csrr t0, mepc
    sw t0, 0(sp)

    /* Switch task context */
    jal HalTaskSwitch

   	li t0, 0x1800					/* set MPIE to 0, make it safe when switch tasks. */
	csrw mstatus, t0

	/* Load mret with the address of the next instruction in the task to run next. */
	la t0, SW_Handler_Switch
	csrw mepc, t0

	mret
	.endfunc

.section	.highcode.SW_Handler_Switch,"ax",@progbits
.global SW_Handler_Switch
.align 2
.func
SW_Handler_Switch:
	li t0, 0x1880					/* set MPIE to 1, make interrupt enabled after this function. */
	csrw mstatus, t0

    /* Load new task */
    la t0, g_losTask
    lw t0, 0(t0)
    lw sp, 0x0(t0)                /* Read sp from first TCB member */

    /* Pop PC from stack and set MEPC */
	lw t0, 0(sp)					/* Load mret with the address of the next instruction in the task to run next. */
	csrw mepc, t0

    /* Restore Registers from Stack */
    lw x1,  1  * 4(sp)    /* RA */
    lw x5,  2  * 4(sp)
    lw x6,  3  * 4(sp)
    lw x7,  4  * 4(sp)
    lw x8,  5  * 4(sp)
    lw x9,  6  * 4(sp)
    lw x10, 7  * 4(sp)
    lw x11, 8  * 4(sp)
    lw x12, 9  * 4(sp)
    lw x13, 10 * 4(sp)
    lw x14, 11 * 4(sp)
    lw x15, 12 * 4(sp)
    lw x16, 13 * 4(sp)
    lw x17, 14 * 4(sp)
    lw x18, 15 * 4(sp)
    lw x19, 16 * 4(sp)
    lw x20, 17 * 4(sp)
    lw x21, 18 * 4(sp)
    lw x22, 19 * 4(sp)
    lw x23, 20 * 4(sp)
    lw x24, 21 * 4(sp)
    lw x25, 22 * 4(sp)
    lw x26, 23 * 4(sp)
    lw x27, 24 * 4(sp)
    lw x28, 25 * 4(sp)
    lw x29, 26 * 4(sp)
    lw x30, 27 * 4(sp)
    lw x31, 28 * 4(sp)

	lw  x4, 29 * 4( sp )			/* tp - not sure it is useful */

    addi sp, sp, ( 32 * 4 )

    mret


#endif /* _LOS_EXC_S */

