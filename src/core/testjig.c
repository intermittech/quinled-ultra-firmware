/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Test jig mode: see testjig.h. */
#include "testjig.h"
#include "ultra.h"                   /* ultra_breathe: the breathe effect's curve */

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
    t->rst = TJ_RST_IDLE;
    t->rst_ok = 0;
    t->rst_ms = 0;
}

uint8_t tj_pad(testjig_t *t, uint32_t now, uint8_t pad_low)
{
    uint8_t i, ev = 0;
    uint32_t el = now - t->rst_ms;
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
        /* the reset gesture: its second hold, or a first one on a running board */
        t->rst = t->rst == TJ_RST_WAIT ? TJ_RST_CONFIRM : t->rst_ok ? TJ_RST_HOLD : TJ_RST_IDLE;
        t->rst_ms = now;
        return TJ_EV_MODE;
    }
    if (!pad_low && t->run_ms >= TJ_LEAVE_MS) {
        t->rst_ok = 1;                                   /* high once since power-up: gestures count from here */
        if (t->active) {
            t->active = 0;
            t->rst = t->rst == TJ_RST_ARMED ? TJ_RST_WAIT : TJ_RST_IDLE;
            t->rst_ms = now;
            return TJ_EV_MODE;
        }
    }
    if (t->rst == TJ_RST_HOLD && el >= TJ_RST_HOLD_MS) {
        t->rst = TJ_RST_ARMED;
    } else if (t->rst == TJ_RST_WAIT && el >= TJ_RST_WAIT_MS) {
        t->rst = TJ_RST_IDLE;
    } else if (t->rst == TJ_RST_CONFIRM && el >= TJ_RST_CONFIRM_MS) {
        t->rst = TJ_RST_IDLE;
        ev = TJ_EV_RESET;
    }
    return ev;
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
    uint32_t x, ph = 0;
    uint32_t ch = tj_where(t, now, &x);
    uint8_t i;
    for (i = 0; i < 5u; i++) {
        v[i] = 0;
    }
    if (ch >= 5u) {
        return;
    }
    /* one breathe (the Ultra effect's curve): up its first half, TJ_HOLD_MS at the top, down its second half */
    if (x < t->ramp_ms) {
        ph = x * 32768u / t->ramp_ms;
    } else if (x < t->ramp_ms + TJ_HOLD_MS) {
        v[ch] = (uint16_t)TJ_PEAK;                       /* the hold: exactly the peak (the current is measured here) */
        return;
    } else if (x < 2u * t->ramp_ms + TJ_HOLD_MS) {
        ph = 32768u + (x + 1u - t->ramp_ms - TJ_HOLD_MS) * 32768u / t->ramp_ms;
    }
    v[ch] = (uint16_t)(ultra_breathe(ph) * TJ_PEAK / 65536u);
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
    if (t->rst >= TJ_RST_ARMED) {
        return (uint8_t)((now / 50u) & 1u);              /* armed: 10 Hz */
    }
    return (uint8_t)((now - t->t0) % t->cycle_ms >= TJ_LED_GAP_MS);
}
