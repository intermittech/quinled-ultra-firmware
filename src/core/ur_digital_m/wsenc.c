/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The WS281x symbol encoder of the Diff-Power-UR-Digital-M's local pixel effects: see wsenc.h. */
#include <stddef.h>
#include "wsenc.h"

#ifndef WS_RAMCODE
#if defined(__GNUC__) && defined(__riscv)
#define WS_RAMCODE MCU_RAMCODE   /* the flash operation's SRAM tick encodes too */
#else
#define WS_RAMCODE
#endif
#endif
#if defined(__GNUC__)
#define WS_INLINE static inline __attribute__((always_inline))
#else
#define WS_INLINE static __inline
#endif

void ws_lut_build(ws_lut_t *L, const ws_chip_t *c)
{
    uint32_t b, q, v, k;
    uint16_t symbol[2][3] = {{0}};
    for (b = 0; b < 2u; b++) {
        uint32_t hi = b ? c->t1h : c->t0h;
        for (q = 0; q < hi; q++) {
            symbol[b][q >> 4] |= (uint16_t)(0x8000u >> (q & 15u));
        }
    }
    for (v = 0; v < 16u; v++) {
        uint32_t wd[12];                        /* (words, not halfwords: no merged halfword store the alignment scan would flag) */
        for (b = 0; b < 4u; b++) {
            const uint16_t *s = symbol[(v >> (3u - b)) & 1u];
            for (q = 0; q < 3u; q++) wd[3u * b + q] = s[q];
        }
        for (k = 0; k < 6u; k++) {                   /* (little-endian: the earlier word in the low half) */
            L->nib[v][k] = (uint32_t)wd[2u * k] | (uint32_t)wd[2u * k + 1u] << 16;
        }
    }
}

#if defined(__GNUC__)
__attribute__((noinline))                        /* (the tick has two callers: one copy) */
#endif
void ws_enc_init(ws_enc_t *e, const ws_chip_t *c, const uint8_t *fb, uint32_t npx, uint32_t w0, uint32_t period_w)
{
    e->fb = fb;
    e->nbytes = npx * c->bpp;
    e->i = 0;
    e->w = w0;
    e->low_w = 0;                                        /* (the first frame waits a whole reset: a clean latch) */
    e->next_w = w0 + c->reset_w;                         /* (the grid starts at the first frame) */
    e->period_w = period_w;
    e->frames = 0;
    e->start_w = w0;
    e->reset_w = c->reset_w;
    e->state = WS_LOW;
    e->ready = 0;
    e->late = 0;
    e->lost = 0;
}

/* zero words from ring index idx (wrapping at its end) */
WS_INLINE void ws_zero(uint16_t *buf, uint32_t idx, uint32_t n)
{
    while (n != 0u) {
        uint32_t run = idx + n > WS_RING_WORDS ? WS_RING_WORDS - idx : n;
        uint16_t *q = buf + idx;
        n -= run;
        idx += run;
        if (run != 0u && ((uintptr_t)q & 2u)) {          /* to a 32-bit boundary */
            *q++ = 0;
            run--;
        }
        for (; run >= 2u; run -= 2u, q += 2) {
            *(uint32_t *)(void *)q = 0;
        }
        if (run != 0u) {
            *q = 0;
        }
        if (idx == WS_RING_WORDS) {
            idx = 0;
        }
    }
}

WS_INLINE void ws_sat8(uint8_t *c)
{
    if (*c != 0xFFu) {
        (*c)++;
    }
}

/* The core. The ring has WS_GUARD words past its 512: a byte's stores that pass the end land there and are moved to the
 * start. Whole bytes only: a byte advances 24 words and starts only with that much room; the leftover waits for the next call (the next tick, <= ~60 us: the deadline is
 * ~190 us: WS_LATE_W). */
WS_RAMCODE uint32_t ws_ring_fill(ws_enc_t *e, const ws_lut_t *L, uint16_t *buf, uint32_t limit)
{
    uint32_t w = e->w, w0 = w;
    uint16_t *const end = buf + WS_RING_WORDS;

    if ((int32_t)(limit - w) > (int32_t)WS_LATE_W) {     /* behind RX: those words are out (or going out) already.
                                                            The frame in progress is lost; LOW from here, a full reset */
        ws_sat8(&e->lost);
        e->state = WS_LOW;
        e->low_w = 0;
        e->i = 0;
        e->ready = 0;
        w = w0 = e->w = limit;
        if ((int32_t)(e->next_w - (limit + e->reset_w)) < 0) {
            e->next_w = limit + e->reset_w;
        }
    }
    for (;;) {
        if (e->state == WS_LOW) {
            /* LOW up to the frame's start (a whole reset written, the grid's word reached, the frame rendered) */
            uint32_t stop = limit;
            if (e->ready) {
                uint32_t s = w + (e->low_w < e->reset_w ? e->reset_w - e->low_w : 0u);
                if ((int32_t)(e->next_w - s) > 0) {
                    s = e->next_w;
                }
                s += s & 1u;                             /* even word: aligned 32-bit stores */
                if ((int32_t)(limit - s) > 0) {
                    stop = s;
                }
            }
            ws_zero(buf, w & WS_RING_MASK, stop - w);
            e->low_w = e->low_w + (stop - w) < e->low_w ? 0xFFFFFFFFu : e->low_w + (stop - w);
            w = stop;
            if (w == limit) {
                break;
            }
            if (e->period_w != 0u) {                     /* the grid: the next frame's word (late: the next one) */
                if ((int32_t)(w - e->next_w) > (int32_t)(WS_W_PER_MS / 2u)) {
                    ws_sat8(&e->late);
                }
                /* Skip whole slots in constant time, even after a long render stall.
                 * LOW scheduling above guarantees w >= next_w (signed, wrap-safe). */
                e->next_w += ((w - e->next_w) / e->period_w + 1u) * e->period_w;
            } else {
                e->next_w = w;
            }
            e->ready = 0;
            e->state = WS_FRAME;
            e->i = 0;
            e->start_w = w;
            e->frames++;
        }
        if (limit - w < 24u) {
            break;
        }
        {                                               /* a byte is 24 words, 12 32-bit stores */
            uint32_t i = e->i;
            const uint32_t n = e->nbytes;
            const uint8_t *fb = e->fb;
            uint32_t *q = (uint32_t *)(void *)(buf + (w & WS_RING_MASK));
            uint32_t *const qend = (uint32_t *)(void *)end;
            while (i < n && limit - w >= 24u) {
                const uint32_t v = fb[i++];
                const uint32_t *a = L->nib[v >> 4], *b = L->nib[v & 15u];
                q[0] = a[0]; q[1] = a[1]; q[2] = a[2]; q[3] = a[3]; q[4] = a[4]; q[5] = a[5];
                q[6] = b[0]; q[7] = b[1]; q[8] = b[2]; q[9] = b[3]; q[10] = b[4]; q[11] = b[5];
                q += 12;
                w += 24u;
                if (q >= qend) {                         /* past the ring's end: those words to its start */
                    uint32_t j, over = (uint32_t)(q - qend);
                    for (j = 0; j < over; j++) {
                        ((uint32_t *)(void *)buf)[j] = qend[j];
                    }
                    q -= WS_RING_WORDS / 2u;
                }
            }
            e->i = i;
            if (i == n) {
                e->state = WS_LOW;
                e->low_w = 0;
            }
        }
        if (e->state == WS_FRAME) {
            break;
        }
    }
    e->w = w;
    return w - w0;
}
