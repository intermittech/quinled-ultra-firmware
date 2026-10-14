/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* 0.6.4's layout (UPPER_RECORDS=0, tools/build_target.sh). The app behind the bootloader (firmware update,
 * src/core/ultra_update.h, wiki: Updating over the data line):
 *   0x00000000-0x00001FFF  bootloader (8 KB, its own link: bootloader/bl.ld)
 *   0x00002000-0x000020FF  app descriptor page (written by the update / production, not part of this link)
 *   0x00002100-0x0000F5FF  this app (53.25 KB); its startup sets mtvec to its own vector table
 *   0x0000F600-0x0000F6FF  current calibration (src/hal/board.h CAL_PAGE_ADDR): inside the bootloader's app slot,
 *                          but no app reaches it, and an update erases only the new app's length
 *   0x0000F700-0x0000FFFF  scene log + settings page (src/hal/board.h), never touched by updates
 * RAM starts after the 32-byte hand-over block at 0x20000000 (upd_handover_t). Same sections as Link.ld. */
ENTRY( _start )

/* the stack: 2 KB. tools/build_target.sh STACK_SIZE=<bytes> (QL_STACK_SIZE, 0.6.6): 1536 is ready (the static worst
 * case 880 B with the ISRs nested, tools/stack.py) and waits for the painted `stack=` of the bench's 30-day soak */
__stack_size = DEFINED(QL_STACK_SIZE) ? QL_STACK_SIZE : 2048;

PROVIDE( _stack_size = __stack_size );

MEMORY
{
	FLASH (rx) : ORIGIN = 0x00002100, LENGTH = 0xF600 - 0x2100
	RAM (xrw) : ORIGIN = 0x20000020, LENGTH = 20K - 32
}

INCLUDE Link_sections.ld
