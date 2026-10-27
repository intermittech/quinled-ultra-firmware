/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#ifndef MCU_IRQ_H
#define MCU_IRQ_H
#if defined(__GNUC__) && defined(__riscv)
#define MCU_RAMCODE __attribute__((section(".ramtext"), noinline))
#else
#define MCU_RAMCODE
#endif
/* isr.S: unified mtvec entry (1 KB aligned, RM p.69), HPE + mret (QK3 3.4); Link_sections.ld keeps .ramtext in SRAM. */
#if defined(BOARD) && BOARD != 3
#error "The CH32V40x family binds only the Diff-Power-UR-Digital-M (BOARD 3)"
#endif
#ifndef ANSWER_UART
#define ANSWER_UART 7
#endif
#if ANSWER_UART != 7
#error "The V407 Digital-M answers on USART7 (PA9)"
#endif
/* vector numbers (RM p.82-85, number column) */
#define RC_IRQN 88
#define RC_BRR_BASE (MCU_PCLK1_HZ / 1000000u)   /* USART7 on PCLK1: 100 = 1 Mbaud */
#define CONSOLE_IRQN 53
#define CONSOLE_TX 1
#endif
