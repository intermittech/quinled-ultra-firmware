/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#ifndef MCU_IRQ_H
#define MCU_IRQ_H
#if defined(__GNUC__) && defined(__riscv)
#define MCU_RAMCODE __attribute__((section(".ramtext"), noinline))
#else
#define MCU_RAMCODE
#endif
/* isr.S is the HPE/mret ABI; Link_sections.ld keeps .ramtext with .data in SRAM. */
/* Board 1 has no return UART; its frozen binding keeps the legacy selector. */
#ifndef ANSWER_UART
#if !defined(BOARD) || BOARD == 1
#define ANSWER_UART 3
#else
#define ANSWER_UART 1
#endif
#endif
#if defined(BOARD) && (BOARD == 2 || BOARD == 3) && ANSWER_UART != 1
#error "Receiver boards require ANSWER_UART=1"
#endif
/* WCH PFIC/vector numbering includes the sixteen core exception entries. */
#if !defined(BOARD) || BOARD == 1
#define RC_IRQN 55
#define RC_BRR_BASE (MCU_PCLK1_HZ / 1000000u)
#else
#define RC_IRQN 53
#define RC_BRR_BASE (MCU_PCLK2_HZ / 1000000u)
#endif
#define CONSOLE_TX 1
#endif
