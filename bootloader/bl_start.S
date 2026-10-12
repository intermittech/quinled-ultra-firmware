/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Bootloader startup: no vector table, no interrupts (mstatus.MIE stays 0 from reset). Large buffers are in
 * .noinit (not cleared), so the fast boot path only clears a few bytes at 8 MHz. */
	.section	.init,"ax",@progbits
	.global	_start
	.option	push
	.option	norvc
_start:
	j	bl_reset                     /* 4 bytes; upd_bl_info_t follows at 0x10 (.blinfo) */
	.option	pop

	.section	.text.bl_reset,"ax",@progbits
	.align	1
bl_reset:
	.option	push
	.option	norelax
	la	gp, __global_pointer$
	.option	pop
	la	sp, _eusrstack
	la	a0, _data_lma
	la	a1, _data_vma
	la	a2, _edata
	bgeu	a1, a2, 2f
1:
	lw	t0, (a0)
	sw	t0, (a1)
	addi	a0, a0, 4
	addi	a1, a1, 4
	bltu	a1, a2, 1b
2:
	la	a0, _sbss
	la	a1, _ebss
	bgeu	a0, a1, 2f
1:
	sw	zero, (a0)
	addi	a0, a0, 4
	bltu	a0, a1, 1b
2:
	li	t0, 0x1f                       /* pipelining / branch prediction, as the WCH startup (the app sets it too) */
	csrw	0xbc0, t0
	jal	main
3:
	j	3b
