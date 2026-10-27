/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* WS2805 8-bit input -> 16-bit level. Linear (x 257) by default: controllers such as WLED already send
 * gamma-corrected values. Gamma 2.2 / 2.8 for controllers that send linear values. */
#ifndef GAMMA_H
#define GAMMA_H

#include <stdint.h>

enum { GAMMA_LINEAR = 0, GAMMA_22 = 1, GAMMA_28 = 2 };

uint16_t gamma_apply(uint8_t mode, uint8_t v);

#endif
