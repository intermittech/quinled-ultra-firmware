/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* WS2805 8-bit input -> 16-bit level: linear (x 257), or gamma 2.2 / 2.8 through the board's power curve
 * (core/ultra.c ultra_gamma) instead of two 512-byte tables (to save flash). The result is within 1 count of
 * round(65535 x (in / 255) ^ gamma) for every input, monotonic, 0 -> 0, 255 -> 65535, an input above 0 never 0
 * (test_cal.c, test_gamma_tables.c check all 512). */
#include "gamma.h"
#include "ultra.h"

uint16_t gamma_apply(uint8_t mode, uint8_t v)
{
    uint32_t x, y;
    if (mode != GAMMA_22 && mode != GAMMA_28) {
        return (uint16_t)(v * 257u);
    }
    x = (uint32_t)v * 257u * 65535u;
    x += x >> 15;                                      /* in / 255 as a Q32 fraction (ultra.c q32_of) */
    y = ultra_gamma(x, mode == GAMMA_22 ? 144179u : 183501u);   /* gamma x 65536 */
    if (v != 0u && y == 0u) {
        y = 1u;
    }
    return (uint16_t)y;
}
