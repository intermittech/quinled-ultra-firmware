/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The app behind the bootloader (firmware update, src/core/ultra_update.h, wiki: Updating over the data line), 0.6.5:
 *   0x00000000-0x00001FFF  bootloader (8 KB, its own link: bootloader/bl.ld)
 *   0x00002000-0x000020FF  app descriptor page (written by the update / production, not part of this link)
 *   0x00002100-0x0000FFFF  this app (55.75 KB, to the zero-wait flash's end); its startup sets mtvec to its own vector
 *                          table
 *   0x08010000-0x08011FFF  the record pages (src/hal/board.h), upper CodeFlash: never part of an image
 * 0.6.4's layout (records at 0xF600-0xFFFF): Link_app_lower.ld. RAM starts after the 32-byte hand-over block at
 * 0x20000000 (upd_handover_t). Same sections as Link.ld. */
ENTRY( _start )

__stack_size = 2048;

PROVIDE( _stack_size = __stack_size );

MEMORY
{
	FLASH (rx) : ORIGIN = 0x00002100, LENGTH = 0x10000 - 0x2100
	RAM (xrw) : ORIGIN = 0x20000020, LENGTH = 20K - 32
}

INCLUDE Link_sections.ld
