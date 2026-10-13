/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Interrupt entry stubs. The WCH startup enables HPE (CSR 0x804 = 3): the QingKe V4 core saves and restores
 * the caller-saved registers (ra, t0-t6, a0-a7) in hardware, so a plain C function plus mret is a complete
 * handler. This avoids the WCH-only "WCH-Interrupt-fast" attribute and works with any RISC-V GCC. */
#include "board.h"

    .section .text.isr_stubs, "ax", @progbits
    .align 2

    .global TIM4_IRQHandler                 /* TIM4 CC2: gate, next toggle */
TIM4_IRQHandler:
    call    gate_isr
    mret

    .global SysTick_Handler                 /* 20 kHz poll tick */
SysTick_Handler:
    call    poll_isr
    mret

    .global TIM2_IRQHandler                 /* TIM2 update: phase slew (adaptive stagger) */
TIM2_IRQHandler:
    call    slew2_isr
    mret

    .global TIM3_IRQHandler                 /* TIM3 update: phase slew (adaptive stagger) */
TIM3_IRQHandler:
    call    slew3_isr
    mret
