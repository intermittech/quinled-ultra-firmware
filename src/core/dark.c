/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "dark.h"

void dark_init(dark_t *d)
{
    d->active = 0;
    d->level = 65535u;
    d->target = 65535u;
    d->t0 = 0;
    d->until = 0;
}

uint32_t dark_factor(const dark_t *d, uint32_t now)
{
    uint32_t t = now - d->t0;
    int32_t a = d->level, b = d->target;
    if (t >= DARK_FADE_MS) {
        return (uint32_t)b;
    }
    return (uint32_t)(a + (b - a) * (int32_t)t / (int32_t)DARK_FADE_MS);   /* linear over the fade */
}

void dark_hold(dark_t *d, uint32_t now, uint32_t hold_ms)
{
    d->level = (uint16_t)dark_factor(d, now);          /* from wherever it is: no jump */
    d->target = 0;
    d->t0 = now;
    d->until = now + hold_ms;
    d->active = 1;
}

void dark_release(dark_t *d, uint32_t now)
{
    d->level = (uint16_t)dark_factor(d, now);
    d->target = 65535u;
    d->t0 = now;
    d->active = 0;
}

uint32_t dark_tick(dark_t *d, uint32_t now)
{
    if (d->active && (int32_t)(now - d->until) >= 0) {
        dark_release(d, now);                          /* the hub never released it: back on its own */
    }
    if (now - d->t0 > DARK_FADE_MS) {
        d->level = d->target;                          /* faded: settled, so the fade can't replay when ms wraps */
    }
    return (uint32_t)(d->level != d->target);
}

void dark_apply(const dark_t *d, uint32_t now, uint16_t v[5])
{
    uint32_t f = dark_factor(d, now), i;
    if (f >= 65535u) {
        return;
    }
    for (i = 0; i < 5u; i++) {
        v[i] = (uint16_t)(((uint32_t)v[i] * f + 32767u) / 65535u);
    }
}
