/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED
 * Written from the CH32V407 reference manual V1.2 (mtvec p.69, flash p.579) and the project-owned startup, update,
 * record and interrupt contracts. FLASH/RAM and __stack_size are supplied by the selected layout.
 */

SECTIONS {
    /* Reset is a four-byte jump; mtvec uses the unified assembly dispatcher (1 KB aligned, isr.S). */
    .init : {
        _sinit = .; KEEP( *(SORT_NONE(.init)) )
        . = ALIGN(4); _einit = .;
    } > FLASH

    .vector : { *(.vector) . = ALIGN(4); } > FLASH

    .text : {
        . = ALIGN(4);
        INCLUDE text_order.ld
        *(.text .text.*)
        *(.rodata .rodata*)
        *(.srodata .srodata.* .gnu.linkonce.t.*)
        . = ALIGN(4);
    } > FLASH

    .fini : {
        KEEP( *(SORT_NONE(.fini)) ) . = ALIGN(4);
        _etext = .; _eitcm = .;
    } > FLASH

    .preinit_array : {
        __preinit_array_start = .; KEEP( *(.preinit_array) )
        __preinit_array_end = .;
    } > FLASH
    .init_array : {
        __init_array_start = .;
        KEEP( *(SORT_BY_INIT_PRIORITY(.init_array.*)) )
        KEEP( *(.init_array) )
        __init_array_end = .;
    } > FLASH
    .fini_array : {
        __fini_array_start = .;
        KEEP( *(SORT_BY_INIT_PRIORITY(.fini_array.*)) )
        KEEP( *(.fini_array) )
        __fini_array_end = .;
    } > FLASH

    /* One flash load range, copied in words by startup.S. The SRAM fault handler (isr.S .ramtext.0fault) is an
     * mtvec target during flash operations: it opens .ramtext on a 1 KB boundary. */
    .ramtext : ALIGN(1024) {
        _data_vma = .;
        KEEP(*(.ramtext.0fault))
        *(SORT_NONE(.ramtext) SORT_NONE(.ramtext.*))
        . = ALIGN(4); __ramtext_end = .;
    } > RAM AT > FLASH
    _data_lma = LOADADDR(.ramtext);
    .data : {
        *(.gnu.linkonce.r.* .data .data.* .gnu.linkonce.d.*)
        . = ALIGN(4);
        __global_pointer$ = DEFINED(QL_GP_BSS) ? __ql_bssgp : . + 1536;
        *(.sdata .sdata.* .sdata2.* .gnu.linkonce.s.*)
        . = ALIGN(4); _edata = .;
    } > RAM AT > FLASH

    .bss : {
        . = ALIGN(4); _sbss = .;
        *(.sbss*)
        __ql_bssgp = .; *(SORT_BY_NAME(.bss.gp.*))
        *(.gnu.linkonce.sb.* .bss* .gnu.linkonce.b.* COMMON*)
        . = ALIGN(4); _ebss = .; _end = .; end = .;
    } > RAM

    /* Retained state survives application and bootloader startup: it lies above the bootloader's 32 KB RAM. */
    .noinit ORIGIN(RAM) + LENGTH(RAM) - __stack_size - 96 (NOLOAD) : {
        *(.noinit .noinit.*)
    } > RAM
    .stack ORIGIN(RAM) + LENGTH(RAM) - __stack_size (NOLOAD) : {
        _heap_end = .; . = ALIGN(4); _susrstack = .;
        . += __stack_size; _eusrstack = .;
    } > RAM

    ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) <= 0x80000, "Code exceeds the zero-wait flash")
    ASSERT((trap_entry & 1023) == 0, "mtvec base trap_entry is not 1 KB aligned")
    ASSERT((flash_fault & 1023) == 0, "mtvec base flash_fault is not 1 KB aligned")
    ASSERT(LOADADDR(.data) - LOADADDR(.ramtext) == ADDR(.data) - ADDR(.ramtext), "SRAM load range is not contiguous")
    ASSERT(_ebss <= ADDR(.noinit), "BSS reaches retained reset state")
    ASSERT(SIZEOF(.noinit) <= 96, "Retained reset state exceeds its reservation")
    ASSERT(((ADDR(.noinit) + 2048) >> 12) == ((ADDR(.noinit) + 95 + 2048) >> 12),
           "Retained state crosses a rounded address page")
    ASSERT(ADDR(.noinit) >= 0x20008000, "Retained state inside the bootloader's RAM (32 KB, target.json)")
    ASSERT(_ebss <= _susrstack, "BSS reaches application stack")
}
