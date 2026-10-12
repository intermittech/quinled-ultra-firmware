/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* QuinLED dig2analog-ultra-5ch bootloader: 8 KB at the start of flash (sectors 0-1, write-protected by production).
 * RAM starts after the 32-byte hand-over block (src/core/ultra_update.h, UPD_HANDOVER_ADDR). */
ENTRY( _start )

__stack_size = 1536;

MEMORY
{
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x2000
	RAM (xrw) : ORIGIN = 0x20000020, LENGTH = 20K - 32
}

SECTIONS
{
	.init :
	{
		KEEP(*(SORT_NONE(.init)))
		. = 0x10;                       /* upd_bl_info_t at UPD_BL_INFO (0x08000010), read by the app and mkfactory.py */
		KEEP(*(.blinfo))
	} >FLASH AT>FLASH

	.text :
	{
		. = ALIGN(4);
		*(.text .text.*)
		*(.rodata .rodata*)
		*(.srodata .srodata.*)
		. = ALIGN(4);
	} >FLASH AT>FLASH

	.dlalign :
	{
		. = ALIGN(4);
		PROVIDE(_data_lma = .);
	} >FLASH AT>FLASH

	.data :
	{
		. = ALIGN(4);
		PROVIDE(_data_vma = .);
		*(.data .data.*)
		. = ALIGN(8);
		PROVIDE( __global_pointer$ = . + 0x800 );
		*(.sdata .sdata.*)
		. = ALIGN(4);
		PROVIDE(_edata = .);
	} >RAM AT>FLASH

	.bss (NOLOAD) :
	{
		. = ALIGN(4);
		PROVIDE(_sbss = .);
		*(.sbss*)
		*(.bss*)
		*(COMMON*)
		. = ALIGN(4);
		PROVIDE(_ebss = .);
	} >RAM

	.noinit (NOLOAD) :
	{
		. = ALIGN(4);
		*(.noinit*)
		. = ALIGN(4);
		PROVIDE(_end_noinit = .);
	} >RAM

	.stack ORIGIN(RAM) + LENGTH(RAM) - __stack_size (NOLOAD) :
	{
		PROVIDE(_susrstack = .);
		. = . + __stack_size;
		PROVIDE(_eusrstack = .);
	} >RAM

	ASSERT(_end_noinit <= _susrstack, "bootloader RAM: buffers overlap the stack")
	ASSERT(bl_info == 0x10, "the bootloader info must be at 0x10")
}
