/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Interrupt entry. The WCH startup enables HPE (CSR 0x804 = 3): the QingKe V4 core saves and restores the
 * caller-saved registers (ra, t0-t6, a0-a7) in hardware, so a plain C function plus mret is a complete handler.
 * This avoids the WCH-only "WCH-Interrupt-fast" attribute and works with any RISC-V GCC. */
#include "board.h"

    /* 0.6.5 (Quindor's choice: -188 B for the vector table, a few instructions per interrupt): one entry for every
     * interrupt and exception (mtvec in unified mode, vendor/wch/startup_ch32v20x_D6.S). mcause: bit 31 + the
     * interrupt number; its low 6 bits identify the four sources (no exception code of this core is 12 or 44-46, and
     * a masked NMI, interrupt 2, goes to the fault handler with the exceptions). Checked by frequency: the poll tick first (4
     * instructions before its call), then the gate (6), the slews (8 / 10). HPE has saved t0 and every other
     * caller-saved register; a nested interrupt (the gate's priority) enters here again. */
    .section .text.isr_stubs, "ax", @progbits
    .align 2                                /* (mtvec's low 2 bits are its mode) */
    .global trap_entry
trap_entry:
    csrr    t0, mcause
    andi    t0, t0, 63
    addi    t0, t0, -12                     /* SysTick: the 20 kHz poll tick */
    bnez    t0, 1f
    call    poll_isr
    mret
1:  addi    t0, t0, -34                     /* TIM4 (46) CC2: the gate, next toggle */
    bnez    t0, 2f
    call    gate_isr
    mret
2:  addi    t0, t0, 2                       /* TIM2 (44) update: phase slew (adaptive stagger) */
    bnez    t0, 3f
    call    slew2_isr
    mret
3:  addi    t0, t0, -1                      /* TIM3 (45) update: phase slew */
    bnez    t0, HardFault_Handler
    call    slew3_isr
    mret

    /* An exception, an NMI or anything unexpected (0.6.5, D1): its cause and PC into the reset breadcrumbs (main.c
     * bcrumb: pc at 8, cause at 12; Link_sections.ld keeps the block in one %hi page), then wait for the watchdog as
     * before (interrupts are off in a trap: the poll tick no longer overwrites the PC) */
    .global HardFault_Handler
HardFault_Handler:
    csrr    t0, mcause
    csrr    t1, mepc
    lui     t2, %hi(bcrumb)
    sw      t0, %lo(bcrumb + 12)(t2)
    sw      t1, %lo(bcrumb + 8)(t2)
1:  j       1b
#if BOARD_RETURN
    .align 2
    .global rc_uart_entry                   /* USART3 (VTF slot 1, hw.c): the return channel's bytes */
rc_uart_entry:
    call    rc_uart_isr
    mret
#endif

    /* 0.6.4: during a flash operation (hw.c flash_op) the tick comes through VTF slot 0, straight to this stub in
     * SRAM (nothing fetched from the flash); mtvec points to flash_fault meanwhile: an exception or NMI forces the
     * outputs off and waits for the watchdog. Same convention as above (HPE + mret). */
    .section .ramtext, "ax", @progbits
    .align 2
    .global flash_tick
flash_tick:
    call    flash_poll_isr
    mret

    .align 2
    .global flash_fault
flash_fault:
    call    hw_flash_trip
1:  j       1b
