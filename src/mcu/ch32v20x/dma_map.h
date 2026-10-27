/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* DMA/peripheral ownership; channel numbers are family facts, never board facts. */
#ifndef MCU_DMA_MAP_H
#define MCU_DMA_MAP_H
#define MCU_DMA_ADC DMA1_Channel1
#define MCU_DMA_SPI_RX DMA1_Channel2
#define MCU_DMA_SPI_TX DMA1_Channel3
#define MCU_DMA_CONSOLE_RX DMA1_Channel6
#define MCU_DMA_CONSOLE_TX DMA1_Channel7
/* PWM reuses DMA channels 2..6 when BOARD_PWM; hardware sequences are in hw.c.
 * TIM1: G/R, TIM2: B + phase slew, TIM3: W2/W1 + DIN capture, TIM4: edge/gate.
 * Digital: TIM3 capture -> TIM4 edge count; TIM1/2 stay unclocked. */
#endif
