/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Test jig mode: see testjig.h. */
#include "testjig.h"

void tj_init(testjig_t *t, uint32_t cycle_ms)
{
    uint8_t i;
    if (cycle_ms < 1000u) {
        cycle_ms = 1000u;
    }
    if (cycle_ms > 2000u) {
        cycle_ms = 2000u;
    }
    t->active = 0;
    t->pad_low = 0;
    t->run_ms = 0;
    t->t0 = 0;
    t->cycle_ms = cycle_ms;
    t->slot_ms = cycle_ms / 5u;                          /* 200..400 ms per channel */
    t->ramp_ms = (t->slot_ms - TJ_HOLD_MS) / 2u;         /* up, hold, down */
    t->acc = 0;
    t->n = 0;
    t->meas_ch = 0xFFu;
    for (i = 0; i < 5u; i++) {
        t->ma[i] = 0;
    }
    t->cycles = 0;
}

uint8_t tj_pad(testjig_t *t, uint32_t now, uint8_t pad_low)
{
    uint8_t i;
    pad_low = (uint8_t)(pad_low != 0u);
    if (pad_low != t->pad_low) {
        t->pad_low = pad_low;
        t->run_ms = 0;
    }
    if (t->run_ms < 0xFFFFu) {
        t->run_ms++;
    }
    if (!t->active && pad_low && t->run_ms >= TJ_ENTER_MS) {
        t->active = 1;
        t->t0 = now;
        t->acc = 0;
        t->n = 0;
        t->meas_ch = 0xFFu;
        for (i = 0; i < 5u; i++) {
            t->ma[i] = 0;
        }
        t->cycles = 0;
        return 1;
    }
    if (t->active && !pad_low && t->run_ms >= TJ_LEAVE_MS) {
        t->active = 0;
        return 1;
    }
    return 0;
}

/* where now falls: the channel (5 = the cycle's remainder, dark) and the ms into its slot */
static uint32_t tj_where(const testjig_t *t, uint32_t now, uint32_t *x)
{
    uint32_t ph = (now - t->t0) % t->cycle_ms;
    uint32_t ch = ph / t->slot_ms;
    *x = ph - ch * t->slot_ms;
    return ch;
}

void tj_outputs(const testjig_t *t, uint32_t now, uint16_t v[5])
{
    uint32_t x, p = 0;
    uint32_t ch = tj_where(t, now, &x);
    uint8_t i;
    for (i = 0; i < 5u; i++) {
        v[i] = 0;
    }
    if (ch >= 5u) {
        return;
    }
    if (x < t->ramp_ms) {
        p = x * 65535u / t->ramp_ms;                                      /* up */
    } else if (x < t->ramp_ms + TJ_HOLD_MS) {
        p = 65535u;                                                       /* hold at the peak */
    } else if (x < 2u * t->ramp_ms + TJ_HOLD_MS) {
        p = (2u * t->ramp_ms + TJ_HOLD_MS - 1u - x) * 65535u / t->ramp_ms;   /* down: the up ramp mirrored */
    }
    v[ch] = (uint16_t)(p * p / 65535u * TJ_PEAK / 65535u);               /* gamma 2, scaled to the peak */
}

uint8_t tj_measure(testjig_t *t, uint32_t now, int32_t i_ma, int32_t zero_ma)
{
    uint32_t x;
    uint32_t ch = tj_where(t, now, &x);
    uint32_t w1 = t->ramp_ms + TJ_HOLD_MS, w0 = w1 - TJ_MEAS_MS;
    uint8_t done = 0;
    if (ch < 5u && x >= w0 && x < w1 && (t->meas_ch == ch || t->n == 0u)) {
        t->meas_ch = (uint8_t)ch;
        t->acc += i_ma - zero_ma;
        t->n++;
    } else if (t->n != 0u) {                             /* the window ended (or a late main loop skipped past it) */
        t->ma[t->meas_ch] = t->acc / (int32_t)t->n;
        if (t->meas_ch == 4u) {
            t->cycles++;
            done = 1;
        }
        t->acc = 0;
        t->n = 0;
        t->meas_ch = 0xFFu;
    }
    return done;
}

uint8_t tj_led(const testjig_t *t, uint32_t now)
{
    return (uint8_t)((now - t->t0) % t->cycle_ms >= TJ_LED_GAP_MS);
}
