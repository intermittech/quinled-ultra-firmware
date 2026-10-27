/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED
 * Written from the CH32FV2x/V3x reference manual V2.5, CH32V203 datasheet
 * V3.0, the referenced QingKe V4 processor manual V1.5 and the project's
 * linker/interrupt ABI. Interrupt dispatch is unified at trap_entry (isr.S).
 * No address vector table is used in this mode (mtvec.MODE = 0).
 */

    .section .init,"ax",@progbits
    .globl _start
    .option push
    .option norvc
_start:
    jal zero, handle_reset
    .option pop

    .section .text.handle_reset,"ax",@progbits
    .balign 2
    .globl handle_reset
handle_reset:
    /* gp must be computed without relaxation through its own old value. */
    .option push
    .option norelax
    lla gp, __global_pointer$
    .option pop
    lla sp, _eusrstack

    .macro advance_word cursor
        addi \cursor, \cursor, 4
    .endm

    /* .ramtext and .data form one contiguous flash-to-RAM load range. */
    lla a0, _data_lma
    lla a1, _data_vma
    lla a2, _edata
    bgeu a1, a2, .Lzero_range
.Lcopy_word:
    lw t0, 0(a0)
    sw t0, 0(a1)
    advance_word a0
    advance_word a1
    bltu a1, a2, .Lcopy_word

.Lzero_range:
    /* The handover, breadcrumbs and stack are outside the zeroed range. */
    lla a0, _sbss
    lla a1, _ebss
    bgeu a0, a1, .Lcore_setup
.Lclear_word:
    sw zero, 0(a0)
    advance_word a0
    bltu a0, a1, .Lclear_word

.Lcore_setup:
    /* Project boot contract: CORECFGR=31, HPE with two interrupt levels,
     * MIE/MPIE set and MPP=0. The app uses GINTENR for critical sections.
     */
    .macro configure_csr number, value
        li t0, \value
        csrw \number, t0
    .endm
    configure_csr 0xbc0, 31
    configure_csr 0x804, 3
    configure_csr 0x300, 0x88
    lla t0, trap_entry
    csrw 0x305, t0

    jal ra, SystemInit
    lla t0, main
    csrw 0x341, t0
    mret
