/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The host soak test's stand-in for WCH's ch32v20x.h (test/test_soak.c): src/app/main.c is built for the host, and
 * the few registers it reads through hal/hw.h's macros (DIN_EDGES, POLL_ACK, ISENSE_RAW, TESTJIG_PAD_LOW) are plain
 * variables the board model sets. Interrupts never preempt the main loop in the model (the poll ISR runs between
 * main loop passes), so the interrupt switches do nothing. */
#ifndef SOAK_CH32V20X_H
#define SOAK_CH32V20X_H

#include <stdint.h>

typedef struct { volatile uint32_t CNT; } soak_tim_t;
typedef struct { volatile uint32_t SR; } soak_systick_t;
typedef struct { volatile uint32_t RDATAR; } soak_adc_t;
typedef struct { volatile uint32_t INDR; } soak_gpio_t;
extern soak_tim_t soak_tim4;
extern soak_systick_t soak_systick;
extern soak_adc_t soak_adc1;
extern soak_gpio_t soak_gpiob;
#define TIM4    (&soak_tim4)
#define SysTick (&soak_systick)
#define ADC1    (&soak_adc1)
#define RAMCODE                                  /* (hal/hw.h: SRAM code on the chip) */
#define GPIOB   (&soak_gpiob)

#define __disable_irq() ((void)0)
#define __enable_irq()  ((void)0)
#define BARRIER()       ((void)0)
#define CPU_SLEEP()     ((void)0)
#define RCC_IWDGRSTF    0x20000000u
#if !defined(__GNUC__)
#define __attribute__(x)
#endif

#endif
