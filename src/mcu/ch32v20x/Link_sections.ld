/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED
 * Written from the CH32FV2x/V3x reference manual V2.5, CH32V203 datasheet
 * V3.0 and the project-owned startup, update, record and interrupt contracts.
 * FLASH/RAM and __stack_size are supplied by the selected layout.
 */

SECTIONS {
    /* Reset is a four-byte jump; mtvec uses the unified assembly dispatcher. */
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

    /* The project is C-only. Retain conventional array sections if supplied. */
    .preinit_array : {
        __preinit_array_start = .; KEEP( *(.preinit_array) )
        __preinit_array_end = .;
    } > FLASH
    .init_array : {
        __init_array_start = .;
        KEEP( *(SORT_BY_INIT_PRIORITY(.init_array.*)) )
        KEEP( *(SORT_BY_INIT_PRIORITY(.ctors.*)) )
        KEEP( *(.init_array) )
        KEEP( *(EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors) )
        __init_array_end = .;
    } > FLASH
    .fini_array : {
        __fini_array_start = .;
        KEEP( *(SORT_BY_INIT_PRIORITY(.fini_array.*)) )
        KEEP( *(SORT_BY_INIT_PRIORITY(.dtors.*)) )
        KEEP( *(.fini_array) )
        KEEP( *(EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .dtors) )
        __fini_array_end = .;
    } > FLASH

    /* One flash load range, copied in words by startup.S. */
    .dalign : { . = ALIGN(4); _data_vma = .; } > RAM
    .dlalign : { . = ALIGN(4); _data_lma = .; } > FLASH
    .ramtext : {
        __ramtext_start = .;
        *(SORT_NONE(.ramtext) SORT_NONE(.ramtext.*))
        . = ALIGN(4); __ramtext_end = .;
    } > RAM AT > FLASH
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

    /* Retained state must survive both application and bootloader startup.
     * The 96-byte reservation shares one rounded RISC-V %hi page, as isr.S
     * requires, and remains below the bootloader's stack at 0x20004a00.
     */
    .noinit ORIGIN(RAM) + LENGTH(RAM) - __stack_size - 96 (NOLOAD) : {
        *(.noinit .noinit.*)
    } > RAM
    .stack ORIGIN(RAM) + LENGTH(RAM) - __stack_size (NOLOAD) : {
        _heap_end = .; . = ALIGN(4); _susrstack = .;
        . += __stack_size; _eusrstack = .;
    } > RAM

    ASSERT(DEFINED(QL_DIAG_BIG_SLOT) || ORIGIN(FLASH) + LENGTH(FLASH) <= 65536,
           "Code exceeds zero-wait flash")
    ASSERT(_ebss <= ADDR(.noinit), "BSS reaches retained reset state")
    ASSERT(SIZEOF(.noinit) <= 96, "Retained reset state exceeds its reservation")
    ASSERT(((ADDR(.noinit) + 2048) >> 12) == ((ADDR(.noinit) + 95 + 2048) >> 12),
           "Retained state crosses a rounded address page")
    ASSERT(ADDR(.noinit) + 96 <= 0x20004a00, "Retained state reaches bootloader stack")
    ASSERT(_ebss <= _susrstack, "BSS reaches application stack")
}
