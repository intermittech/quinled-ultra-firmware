/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* hw.c's internals shared with hw_cold.c (review round 9: the boot and the flash operation built with -msave-restore):
 * the register nibbles and offsets, pin_cfg, and the state flash_op pauses and restores. Not part of the HAL's
 * interface (hw.h). */
#ifndef HW_PRIV_H
#define HW_PRIV_H

#include <stdint.h>
#include "ch32v20x.h"
#include "hw.h"

/* GPIO CFGxR nibbles (STM32F1 style) */
#define CFG_IN_FLOAT 0x4u
#define CFG_OUT_PP   0x3u        /* 50 MHz push-pull */
#define CFG_AF_PP    0xBu        /* 50 MHz alternate-function push-pull */
#define CFG_ANALOG   0x0u
#define CFG_IN_PULL  0x8u        /* input with pull-up / pull-down (ODR) */

/* TIMx_CHCTLRx output compare modes */
#define OCM_FROZEN   0u
#define OCM_TOGGLE   3u
#define OCM_FORCE_LO 4u
#define OCM_FORCE_HI 5u
#define OCM_PWM1     6u          /* active while CNT < CCR */
#define OCM_PWM2     7u          /* active while CNT >= CCR */

/* TIM register offsets for the DMA burst base address (DBA = offset / 4) */
#define TIM_OFS_ATRLR  0x2Cu
#define TIM_OFS_CH1CVR 0x34u
#define TIM_OFS_CH3CVR 0x3Cu

#define DMA_MEM2PER_16_CIRC (DMA_CFGR1_DIR | DMA_CFGR1_CIRC | DMA_CFGR1_MINC | DMA_CFGR1_PSIZE_0 | DMA_CFGR1_MSIZE_0)

/* TIMx DMAINTENR bits not named in the WCH header */
#define TIM_CC2IE_ 0x0004u
#define TIM_CC2DE_ 0x0400u
#define TIM_CC3DE_ 0x0800u

#define FWD_PIN    1u                                    /* PA1 = FWD_EN */

static inline void pin_cfg(GPIO_TypeDef *g, uint32_t pin, uint32_t cfg)
{
    volatile uint32_t *r = pin < 8u ? &g->CFGLR : &g->CFGHR;
    uint32_t sh = (pin & 7u) * 4u;
    *r = (*r & ~(0xFu << sh)) | (cfg << sh);
}

/* the PWM state (hw.c pwm_start): flash_op holds TIM2 / TIM3 at the plain period while the clock is halved, and clears
 * the phase slew's pending steps */
extern uint32_t cur_steps;
extern uint8_t cur_spread;
extern volatile int32_t slew_pend2, slew_pend3;
extern volatile uint8_t adc_ready;                   /* hw_adc_init done: the poll tick may start sequences (0 during
                                                        flash_op too: the SRAM tick does not run them) */
#if CONSOLE_FULL
#define UART_RX_SIZE 64u                                /* the console's RX ring (hw.c "UART"; hw_init starts its DMA) */
extern uint8_t uart_rx[UART_RX_SIZE];
extern uint32_t uart_rx_r;                           /* the ring's read index, ... */
extern uint8_t uart_rx_drop;                         /* ... flash_op dropped bytes: 1 = tell main.c, 2 = to CR / LF */
uint32_t uart_rx_w(void);                            /* where the RX DMA writes next */
#endif

#endif
