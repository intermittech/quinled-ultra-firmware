/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Bootloader hardware layer (CH32V203F8U6, register level, no interrupts). BENCH items are listed in README.md. */
#ifndef BL_HW_H
#define BL_HW_H

#include <stdint.h>

#include "../src/mcu/profile.h"
#define HW_RING_WORDS   MCU_BL_RING_WORDS   /* SPI samples: 2048 at 36 MHz (0.91 ms), 4096 at 50 MHz (1.31 ms) */
#define HW_TICKS_US     (MCU_HCLK_HZ / 1000000u)           /* SysTick at HCLK = 144 MHz */

extern uint16_t hw_ring[HW_RING_WORDS];

#if BL_HW_ID == 3
void hw_blank_early(void);              /* the Diff-Power-UR-Digital-M: BLANK_N LOW first (bl_hw.c) */
#endif
uint32_t hw_reset_flags_take(void);    /* RCC->RSTSCKR, then cleared (works at any clock) */
void hw_crc_on(void);                  /* CRC unit clock (also used by the fast boot path at 8 MHz) */
void hw_crc_off(void);                 /* back to the reset state before a jump */
void hw_full_init(void);               /* 144 MHz, safe GPIO, SysTick, watchdog, PVD, DIN sampling and gate */
/* set by every flash operation (HCLK / 2 meanwhile: the samples of that time are useless); the caller clears it */
extern volatile uint8_t hw_flash_stall;
uint32_t hw_ticks(void);               /* SysTick low word (144 per us) */
uint32_t hw_ms(void);                  /* ms since hw_full_init (call at least every 44 s) */
uint32_t hw_ring_pos(void);            /* DMA write position in hw_ring */
uint16_t hw_edges(void);               /* DIN falling edges since the last gate re-arm */
void hw_gate_open(void);               /* FWD_EN high: forward everything; edge counter restarts */
void hw_gate_ultra(void);              /* FWD_EN low for 256 bits, then high: consume the board's normal command slot */
void hw_kick(void);
void hw_led(int on);                   /* the status LED (only the recovery blink drives it) */
__attribute__((noreturn)) void hw_soft_reset(void);
__attribute__((noreturn)) void hw_jump_app(void);
/* (status profile) the answer: lead byte + 32 bytes on the answer UART at `brr`, enabled at local tick `at`; 0 = sent,
 * -1 = suppressed (late, or the line not released) */
int hw_reply(const uint8_t *a, uint32_t at, uint16_t brr, uint32_t t0);

#endif
