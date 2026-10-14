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
typedef struct { volatile uint32_t SR, CNT; } soak_systick_t;  /* CNT: the low word (hal/hw.h POLL_CNT) */
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
#define HW_UID  soak_uid                         /* (hal/hw.h: the chip's unique id) */
extern const uint32_t soak_uid[3];

#define __disable_irq() ((void)0)
#define __enable_irq()  ((void)0)
#define BARRIER()       ((void)0)
#define CPU_SLEEP()     soak_sleep()        /* (test_soak.c: nothing, or r5cpu's model of the WFI) */
void soak_sleep(void);
#define TRAP_PC()       0x00003A1Cu              /* (0.6.5, D1) the main loop's PC the tick sees */
#define RCC_IWDGRSTF    0x20000000u
#if !defined(__GNUC__)
#define __attribute__(x)
#endif

#endif
