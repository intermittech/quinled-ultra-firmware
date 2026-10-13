/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* WS2805 8-bit input -> 16-bit level: linear (x 257), or gamma 2.2 / 2.8 through the board's exact power curve
 * (core/ultra.c ultra_gamma) instead of two 512-byte tables (to save flash). The result is
 * round(65535 x (in / 255) ^ gamma), an input above 0 never 0, for every input: the curve's interpolation lands one
 * count off for the inputs in fix[], which are corrected (test_cal.c checks all 512 against the formula). */
#include "gamma.h"
#include "ultra.h"

static const struct {
    uint8_t mode, in;                                  /* one count up */
} fix[] = {
    { GAMMA_22, 187 }, { GAMMA_28, 114 }, { GAMMA_28, 183 }, { GAMMA_28, 184 }, { GAMMA_28, 197 }, { GAMMA_28, 220 },
};

uint16_t gamma_apply(uint8_t mode, uint8_t v)
{
    uint32_t x, y, i;
    if (mode != GAMMA_22 && mode != GAMMA_28) {
        return (uint16_t)(v * 257u);
    }
    x = (uint32_t)v * 257u * 65535u;
    x += x >> 15;                                      /* in / 255 as a Q32 fraction (ultra.c q32_of) */
    y = ultra_gamma(x, mode == GAMMA_22 ? 144179u : 183501u);   /* gamma x 65536 */
    if (v != 0u && y == 0u) {
        y = 1u;
    }
    for (i = 0; i < sizeof fix / sizeof fix[0]; i++) {
        if (fix[i].mode == mode && fix[i].in == v) {
            y++;
        }
    }
    return (uint16_t)y;
}
