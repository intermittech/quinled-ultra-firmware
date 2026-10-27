/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED
 * Interrupt entry of the QingKe V3V (CH32V407 RM V1.2 p.69, QingKe V3 manual V1.5 3.4): HPE (CSR 0x804, startup.S)
 * saves and restores the caller-saved registers in hardware, so a plain C function plus mret is a complete handler.
 * mtvec is used in unified mode; its base must be 1 KB aligned (RM p.69: mtvec[9:2] read as 0), so the entry and the
 * flash operation's SRAM fault handler each start a 1 KB aligned section. mcause[7:0] is the vector number
 * (vectors reach 103 here).
 */
#include "../../target.h"

    .section .text.isr_stubs, "ax", @progbits
    .balign 1024
    .global trap_entry
trap_entry:
    csrr    t0, mcause
    andi    t0, t0, 255
    addi    t0, t0, -12                     /* SysTick (12): the 20 kHz poll tick */
    bnez    t0, 1f
    call    poll_isr
    mret
1:  addi    t0, t0, -(51 - 12)              /* SPI1 (51): the delay line's error interrupt (OVR) */
    bnez    t0, 2f
    call    dl_spi1_isr
    mret
2:  addi    t0, t0, -(CONSOLE_IRQN - 51)    /* USART1 (53): the console's bytes */
    bnez    t0, 3f
    call    con_uart_isr
    mret
3:  addi    t0, t0, -(RC_IRQN - CONSOLE_IRQN)   /* USART7 (88): the answer's bytes, should the core take it here
                                               rather than through its VTF slot 1 (hw_cold.c) */
    bnez    t0, HardFault_Handler
    call    rc_uart_isr
    mret

    /* An exception, an NMI or anything unexpected: cause and PC into the reset breadcrumbs (main_ur_digital_m.c
     * bcrumb: pc at 8, cause at 12; Link_sections.ld keeps the block in one %hi page), then the watchdog. */
    .global HardFault_Handler
HardFault_Handler:
    csrr    t0, mcause
    csrr    t1, mepc
    lui     t2, %hi(bcrumb)
    sw      t0, %lo(bcrumb + 12)(t2)
    sw      t1, %lo(bcrumb + 8)(t2)
1:  j       1b

    .align 2
    .global rc_uart_entry                   /* USART7 (VTF slot 1, hw_cold.c): the answer's bytes */
rc_uart_entry:
    call    rc_uart_isr
    mret

    /* During a flash operation (hw_cold.c flash_op) the tick comes through VTF slot 0 to this SRAM stub, and mtvec
     * points to flash_fault (SRAM): an exception or NMI releases the strip and the hub's line, then the watchdog. */
    .section .ramtext.0fault, "ax", @progbits
    .balign 1024
    .global flash_fault
flash_fault:
    call    hw_flash_trip
1:  j       1b

    .align 2
    .global flash_tick
flash_tick:
    call    flash_poll_isr
    mret
