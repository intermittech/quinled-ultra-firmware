/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* SysTick of the QingKe V3V: a 32-bit counter only (RM p.80-82): CNT at 0x08, CMP at 0x10. */
#ifndef MCU_SYSTICK_H
#define MCU_SYSTICK_H
#include "clock.h"
#define MCU_SYSTICK_BITS 32u
#define MCU_SYSTICK_LOW_BITS 32u
#define MCU_TICK_DELTA(now, then) ((uint32_t)((now) - (then)))
#define MCU_POLL_COUNT() (SysTick->CNT)
#define MCU_POLL_PENDING() (SysTick->SR & 1u)
#endif
