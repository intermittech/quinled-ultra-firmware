/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* measure_proto.py's link: code + constants in the zero-wait flash at 0, data in SRAM (rv32emu copies .data) */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 64K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}
SECTIONS
{
    .text : { KEEP(*(.text.b_*)) *(.text .text.*) *(.rodata .rodata.* .srodata .srodata.*) } > FLASH
    .data : { *(.data .data.* .sdata .sdata.*) } > RAM AT > FLASH
    .bss (NOLOAD) : { *(.bss .bss.* .sbss .sbss.* COMMON) } > RAM
    __global_pointer$ = ORIGIN(RAM) + 0x800;
    /DISCARD/ : { *(.comment) *(.riscv.attributes) }
}
