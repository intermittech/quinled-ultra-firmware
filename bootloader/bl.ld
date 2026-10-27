/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* QuinLED Ultra bootloader: 8 KiB at the start of flash, write-protected by production.
 * RAM starts after the 32-byte handover block (src/core/upd_platform.h, UPD_HANDOVER_ADDR). */
ENTRY( _start )
INCLUDE boot_api_layout.ld

__stack_size = 1536;

MEMORY
{
	FLASH (rx) : ORIGIN = MCU_BOOT_LINK_BASE, LENGTH = MCU_BOOT_BYTES
	RAM (xrw) : ORIGIN = 0x20000020, LENGTH = (DEFINED(MCU_BOOT_RAM_BYTES) ? MCU_BOOT_RAM_BYTES : 20K) - 32
}

SECTIONS
{
	.init :
	{
		KEEP(*(SORT_NONE(.init)))
		. = 0x10;                       /* upd_bl_info_t at UPD_BL_INFO (0x08000010), read by the app and mkfactory.py */
		KEEP(*(.blinfo))
		. = MCU_BOOT_API_OFFSET;
		KEEP(*(.boot_api))
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
		*(.ramtext .ramtext.*)
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
	ASSERT(bl_api == MCU_BOOT_LINK_BASE + MCU_BOOT_API_OFFSET, "boot API fixed address")
	ASSERT(bl_info == 0x10, "the bootloader info must be at 0x10")
}
