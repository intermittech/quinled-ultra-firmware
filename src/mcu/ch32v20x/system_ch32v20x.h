/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED
 * Written from the CH32FV2x/V3x reference manual V2.5, chapter 3.
 * SystemInit is the project's clock setup in hw_cold.c.
 */
#ifndef SYSTEM_CH32V20X_H
#define SYSTEM_CH32V20X_H

#include <stdint.h>

extern uint32_t SystemCoreClock;
void SystemInit(void);

#endif
