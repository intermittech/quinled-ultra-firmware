/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* hw.c's internals shared with hw_cold.c and hw_dline.c: the GPIO nibbles, pin_cfg, the answer UART's pins and the
 * state a flash operation pauses. Not part of the HAL's interface (hw.h). */
#ifndef HW_PRIV_H
#define HW_PRIV_H

#include <stdint.h>
#include "ch32v40x.h"
#include "hw.h"

/* GPIO CFGxR nibbles (RM p.128-129): CNF[3:2] MODE[1:0]; MODE 11 = 50 MHz, 10 = 2 MHz (the USB-type pads PA11/PA12/PB6/
 * PB7 meet only the 2 / 10 MHz figures, DS p.69 note 2) */
#define CFG_IN_FLOAT  0x4u
#define CFG_OUT_PP    0x3u
#define CFG_OUT_PP_2M 0x2u
#define CFG_AF_PP     0xBu
#define CFG_ANALOG    0x0u
#define CFG_IN_PULL   0x8u

/* the answer UART (board.h ANSWER_UART 7): USART7, remap 11 puts TX on PA9 (RM p.121, 138) */
#define RC_USART   USART7
#define RC_IRQ     USART7_IRQn
#define RC_TX_GPIO GPIOA
#define RC_TX_PIN  9u
#define RC_DE_PIN  11u                                   /* input transceiver DE + /RE on PA11 */
#define RC_BLANK_PIN 8u                                  /* BLANK_N on PA8 */
typedef char rc_irqn_is_board_h[RC_IRQ == RC_IRQN ? 1 : -1];

/* (atomic: the answer UART's handler, the tick's ARGB output and the main loop share GPIOA's CFGHR) */
static inline void pin_cfg(GPIO_TypeDef *g, uint32_t pin, uint32_t cfg)
{
    gpio_cfg_set(pin < 8u ? &g->CFGLR : &g->CFGHR, (pin & 7u) * 4u, cfg);
}

extern volatile uint8_t adc_ready;                   /* hw_adc_init done (0 during flash_op: the SRAM tick runs no ADC) */
extern volatile uint8_t con_hold;                    /* flash_op: the console's interrupt holds its next byte */

#endif
