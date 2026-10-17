/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Bootloader hardware layer (CH32V203F8U6, register level, no interrupts). BENCH items are listed in README.md. */
#ifndef BL_HW_H
#define BL_HW_H

#include <stdint.h>

#define HW_RING_WORDS   2048u          /* SPI samples at 36 MHz: 0.91 ms */
#define HW_TICKS_US     144u           /* SysTick at HCLK = 144 MHz */

extern uint16_t hw_ring[HW_RING_WORDS];

#if BL_HW_ID == 3
void hw_blank_early(void);              /* the Ultra-Digital receiver: BLANK_N LOW first (bl_hw.c) */
#endif
uint32_t hw_reset_flags_take(void);    /* RCC->RSTSCKR, then cleared (works at any clock) */
void hw_crc_on(void);                  /* CRC unit clock (also used by the fast boot path at 8 MHz) */
void hw_crc_off(void);                 /* back to the reset state before a jump */
void hw_full_init(void);               /* 144 MHz, GPIO, SysTick, IWDG kick, PVD, SPI, TIM2 gate, feedback PWM, UART */
/* set by every flash operation (HCLK / 2 meanwhile: the samples of that time are useless); the caller clears it */
extern volatile uint8_t hw_flash_stall;
uint32_t hw_ticks(void);               /* SysTick low word (144 per us) */
uint32_t hw_ms(void);                  /* ms since hw_full_init (call at least every 44 s) */
uint32_t hw_ring_pos(void);            /* DMA write position in hw_ring */
uint16_t hw_edges(void);               /* DIN falling edges since the last gate re-arm */
void hw_gate_open(void);               /* FWD_EN high: forward everything; edge counter restarts */
void hw_gate_ultra(void);              /* FWD_EN low for 256 bits, then high (an Ultra board's own slot, wire VERSION 2) */
void hw_feedback(int on);              /* red channel 10 % + D9 */
void hw_kick(void);
void hw_puts(const char *s);           /* debug UART (TP5, 115200 8N1), blocking */
void hw_puthex(uint32_t v);
__attribute__((noreturn)) void hw_soft_reset(void);
__attribute__((noreturn)) void hw_jump_app(void);

#endif
