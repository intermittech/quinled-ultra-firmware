/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED
 * Written from the CH32V407 reference manual V1.2 (CSRs p.66-78, mtvec p.69) and the QingKe V3 processor manual
 * V1.5 (HPE 3.4, nesting 3.2). Interrupt dispatch is unified at trap_entry (isr.S, 1 KB aligned).
 *
 * The bootloader may enter at the physical entry (0x0800xxxx) or the link alias (0x0000xxxx): every RAM and
 * vector address here is absolute (lui/addi), so the reset path is correct at either alias; the mret to main
 * continues at the link alias.
 *
 * The memory split first (decided 2026-10-22: RAM_CODE_MOD = 0, 512 KB zero-wait code + 200 KB SRAM; FLASH_OBR bit 8,
 * RM p.584-585): with RAM_CODE_MOD = 1 the SRAM is 136 KB and this image's stack and data (up to 200 KB) do not exist,
 * so it touches no RAM and waits for the watchdog (the bootloader's IWDG): the bootloader writes mode 0 (bl_core.c
 * protect) before an app runs. The strip stays as the bootloader left it (held LOW).
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
    lui t0, %hi(0x4002201C)                 /* FLASH_OBR */
    lw t1, %lo(0x4002201C)(t0)
    andi t1, t1, 0x100                      /* RAM_CODE_MOD */
    bnez t1, .Lwrong_split
    .option push
    .option norelax
    lui gp, %hi(__global_pointer$)
    addi gp, gp, %lo(__global_pointer$)
    .option pop
    lui sp, %hi(_eusrstack)
    addi sp, sp, %lo(_eusrstack)

    .macro absolute reg, sym
        lui \reg, %hi(\sym)
        addi \reg, \reg, %lo(\sym)
    .endm
    .macro advance_word cursor
        addi \cursor, \cursor, 4
    .endm

    /* .ramtext and .data form one contiguous flash-to-RAM load range (Link_sections.ld asserts it). */
    absolute a0, _data_lma
    absolute a1, _data_vma
    absolute a2, _edata
    bgeu a1, a2, .Lzero_range
.Lcopy_word:
    lw t0, 0(a0)
    sw t0, 0(a1)
    advance_word a0
    advance_word a1
    bltu a1, a2, .Lcopy_word

.Lzero_range:
    /* The handover, breadcrumbs and stack are outside the zeroed range. */
    absolute a0, _sbss
    absolute a1, _ebss
    bgeu a0, a1, .Lcore_setup
.Lclear_word:
    sw zero, 0(a0)
    advance_word a0
    bltu a0, a1, .Lclear_word

.Lcore_setup:
    /* CPU_RUN_CTLR (0xbc0) = 1: prefetch on, everything else reset (RM p.76-77; 31 sets reserved bits here).
     * INEST_CTLR (0xbc1) = 1: two nesting levels (reset: none, RM p.77-78). HW_POPDM_CTLR (0x804) = 7: HPE,
     * nesting, preemption on priority bit 7 (RM p.73; reset 4). mstatus = 0x1888: MPP = machine (main runs in
     * machine mode: the flash operation rewrites mtvec), MPIE and MIE (RM p.67-68). */
    .macro configure_csr number, value
        li t0, \value
        csrw \number, t0
    .endm
    configure_csr 0xbc0, 0x01
    configure_csr 0xbc1, 0x01
    configure_csr 0x804, 0x07
    configure_csr 0x300, 0x1888
    absolute t0, trap_entry
    csrw 0x305, t0

    jal ra, SystemInit
    absolute t0, main
    csrw 0x341, t0
    mret

.Lwrong_split:
    j .Lwrong_split
