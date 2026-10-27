/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#ifndef MCU_SYSTICK_H
#define MCU_SYSTICK_H
#include "clock.h"
#define MCU_SYSTICK_BITS 64u
#define MCU_SYSTICK_LOW_BITS 32u
/* Runtime elapsed values use low-word unsigned subtraction, never absolute ordering. */
#define MCU_TICK_DELTA(now, then) ((uint32_t)((now) - (then)))
#define MCU_POLL_COUNT() (*(volatile uint32_t *)&SysTick->CNT)
#define MCU_POLL_PENDING() (SysTick->SR & 1u)
#endif
