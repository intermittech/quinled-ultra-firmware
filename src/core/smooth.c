/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "smooth.h"
#include "lto.h"

void smooth_init(smooth_t *s)
{
    uint8_t i;
    for (i = 0; i < 5u; i++) {
        s->cur[i] = s->from[i] = s->to[i] = 0;
    }
    s->t0 = s->dur = s->last_frame = s->interval = 0;
    s->have_frame = 0;
}

static void start(smooth_t *s, const uint16_t tgt[5], uint32_t now, uint32_t dur)
{
    uint8_t i;
    for (i = 0; i < 5u; i++) {
        s->from[i] = s->cur[i];               /* from wherever the output is now, also mid-ramp */
        s->to[i] = tgt[i];
    }
    s->t0 = now;
    s->dur = dur;
    if (dur == 0u) {
        for (i = 0; i < 5u; i++) {
            s->cur[i] = tgt[i];
        }
    }
}

void smooth_frame(smooth_t *s, const uint16_t tgt[5], uint32_t now, uint8_t ramp)
{
    uint32_t dur = 0;
    if (s->have_frame) {
        uint32_t dt = now - s->last_frame;
        if (dt > SMOOTH_MAX_MS * 4u * SMOOTH_TICKS_PER_MS) {
            dt = SMOOTH_MAX_MS * 4u * SMOOTH_TICKS_PER_MS;     /* a pause in the data must not stall the EMA */
        }
        s->interval = s->interval ? s->interval + ((int32_t)(dt - s->interval) >> 2) : dt;
    }
    s->last_frame = now;
    s->have_frame = 1;
    if (ramp && s->interval >= SMOOTH_MIN_MS * SMOOTH_TICKS_PER_MS && s->interval <= SMOOTH_MAX_MS * SMOOTH_TICKS_PER_MS) {
        dur = s->interval;
    }
    start(s, tgt, now, dur);
}

void smooth_fade(smooth_t *s, const uint16_t tgt[5], uint32_t now, uint32_t dur)
{
    start(s, tgt, now, dur);
}

MEASURED uint8_t smooth_update(smooth_t *s, uint32_t now)
{
    uint32_t el;
    uint8_t i, changed = 0;
    if (s->dur == 0u) {
        return 0;
    }
    el = now - s->t0;
    if (el >= s->dur) {
        el = s->dur;
    }
    {
        uint32_t e = el, du = s->dur;
        while (du > 16384u) {                 /* keep d * e within int32 (|d| <= 65535) */
            e >>= 1;
            du >>= 1;
        }
        for (i = 0; i < 5u; i++) {
            int32_t d = (int32_t)s->to[i] - (int32_t)s->from[i];
            uint16_t v = (uint16_t)((int32_t)s->from[i] + d * (int32_t)e / (int32_t)du);
            if (v != s->cur[i]) {
                s->cur[i] = v;
                changed = 1;
            }
        }
    }
    if (el == s->dur) {
        for (i = 0; i < 5u; i++) {
            if (s->cur[i] != s->to[i]) {
                s->cur[i] = s->to[i];         /* exact end value despite the scaling */
                changed = 1;
            }
        }
        s->dur = 0;                           /* ramp done */
    }
    return changed;
}
