/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Voltage channels: see vsense.h. 32-bit arithmetic only (no 64-bit division on the target). */
#include "vsense.h"

void vsense_init(vsense_t *v, uint32_t ratio_q8, uint32_t vref_mv)
{
    v->ratio_q8 = ratio_q8;
    v->vref_mv = vref_mv;
    v->acc = v->n = v->ms = 0;
    v->mv = 0;
    v->min_mv = 0xFFFFFFFFu;
    v->max_mv = 0;
    v->valid = 0;
}

uint32_t vsense_mv(const vsense_t *v, uint32_t sum2, uint32_t n)
{
    uint32_t x, pin10;
    if (n == 0u) {
        return 0;
    }
    x = sum2 / n * v->vref_mv + (sum2 % n) * v->vref_mv / n;      /* pair average x vref: <= 8190 x 3300 */
    pin10 = x / 819u;                                              /* the pin in 0.1 mV: x x 10 / (2 x 4095) */
    return (pin10 * v->ratio_q8 / 10u + 128u) >> 8;
}

void vsense_add(vsense_t *v, uint32_t sum2, uint32_t n)
{
    v->acc += sum2;
    v->n += n;
    if (++v->ms < VSENSE_WIN_MS) {
        return;
    }
    if (v->n != 0u) {
        v->mv = vsense_mv(v, v->acc, v->n);
        v->valid = 1;
        if (v->mv < v->min_mv) {
            v->min_mv = v->mv;
        }
        if (v->mv > v->max_mv) {
            v->max_mv = v->mv;
        }
    }
    v->acc = v->n = v->ms = 0;
}

void vsense_report(vsense_t *v, uint32_t *min_mv, uint32_t *max_mv)
{
    *min_mv = v->min_mv == 0xFFFFFFFFu ? v->mv : v->min_mv;
    *max_mv = v->max_mv;
    if (v->valid) {
        v->min_mv = v->max_mv = v->mv;
    } else {                                                       /* nothing measured yet: no restart value */
        v->min_mv = 0xFFFFFFFFu;
        v->max_mv = 0;
    }
}
