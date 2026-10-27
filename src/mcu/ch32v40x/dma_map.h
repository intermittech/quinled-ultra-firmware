/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* DMA ownership (RM 11.2 tables p.147-148); the request mapping is fixed per channel. */
#ifndef MCU_DMA_MAP_H
#define MCU_DMA_MAP_H
#define MCU_DMA_SPI_RX DMA1_Channel2     /* shared with TIM2_UP / TIM3_CH3: their DMA enables stay 0 */
#define MCU_DMA_SPI_TX DMA1_Channel3     /* shared with TIM3_UP / TIM3_CH4 */
#define MCU_DMA_ARGB DMA1_Channel10      /* shared with USART10_TX (unused) */
/* Not used: USART1_TX (DMA1 ch4, shared with the TIM4_CH2 forwarding gate), ADC1 (ch12), USART7_TX (DMA2 ch8). */
#endif
