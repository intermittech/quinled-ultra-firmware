/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* minimal replacement for WCH's system_ch32v20x.h (ch32v20x.h includes it); the clock setup lives in hw.c */
#ifndef SYSTEM_CH32V20X_H
#define SYSTEM_CH32V20X_H

#include <stdint.h>

extern uint32_t SystemCoreClock;
void SystemInit(void);

#endif
