/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The WS281x symbol encoder of the Ultra-Digital receiver's local pixel effects: see wsenc.h. */
#include <stddef.h>
#include "wsenc.h"

#ifndef WS_RAMCODE
#if defined(__GNUC__) && defined(__riscv)
#define WS_RAMCODE __attribute__((section(".ramtext"), noinline))   /* the flash operation's SRAM tick encodes too */
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
    uint32_t b, p, q, s1 = c->s > WS_LUT_S_MAX ? WS_LUT_S_MAX : c->s;
    L->fast = (uint8_t)(c->s == 48u);
    if (L->fast) {                                       /* nibble v: 4 bits of 3 words, word k of the nibble */
        uint32_t v, k;
        for (v = 0; v < 16u; v++) {
            uint16_t wd[12];
            for (k = 0; k < 12u; k++) {
                wd[k] = 0;
            }
            for (b = 0; b < 4u; b++) {
                uint32_t hi = (v >> (3u - b)) & 1u ? c->t1h : c->t0h;
                for (q = 0; q < hi; q++) {
                    wd[3u * b + (q >> 4)] |= (uint16_t)(0x8000u >> (q & 15u));
                }
            }
            for (k = 0; k < 6u; k++) {                   /* (little-endian: the earlier word in the low half) */
                L->t.nib[v][k] = (uint32_t)wd[2u * k] | (uint32_t)wd[2u * k + 1u] << 16;
            }
        }
        L->s1 = 48;
        L->tail = 0;
        return;
    }
    for (b = 0; b < 2u; b++) {
        uint32_t hi = b ? c->t1h : c->t0h;
        for (p = 0; p < 16u; p++) {
            uint16_t *h = L->t.exact.h[b][p];
            h[0] = h[1] = h[2] = h[3] = 0;
            for (q = 0; q < hi; q++) {                   /* sample p + q: word (p + q) >> 4, MSB = the earliest */
                h[(p + q) >> 4] |= (uint16_t)(0x8000u >> ((p + q) & 15u));
            }
        }
    }
    for (p = 0; p < 16u; p++) {
        L->t.exact.adv[p] = (uint8_t)((p + s1) >> 4);
    }
    L->s1 = (uint8_t)s1;
    L->tail = (uint8_t)(c->s - s1);
}

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
    e->cur = 0;
    e->p = 0;
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
 * start. Whole bytes only: a byte advances at most 8 x ((15 + s) >> 4) words and its last bit stores 3 words, so it
 * starts only with that much room; the leftover waits for the next call (the next tick, <= ~60 us: the deadline is
 * ~190 us: WS_LATE_W). */
WS_RAMCODE uint32_t ws_ring_fill(ws_enc_t *e, const ws_lut_t *L, uint16_t *buf, uint32_t limit)
{
    uint32_t w = e->w, w0 = w;
    const uint32_t need = L->fast ? 24u : 8u * ((15u + L->s1 + L->tail) >> 4) + 3u;   /* a byte's words + stores */
    uint16_t *const end = buf + WS_RING_WORDS;

    if ((int32_t)(limit - w) > (int32_t)WS_LATE_W) {     /* behind RX: those words are out (or going out) already.
                                                            The frame in progress is lost; LOW from here, a full reset */
        ws_sat8(&e->lost);
        e->state = WS_LOW;
        e->low_w = 0;
        e->i = 0;
        e->p = 0;
        e->cur = 0;
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
                s += s & L->fast;                        /* (fast: an even word, the 32-bit stores aligned) */
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
        if (limit - w < need) {
            break;
        }
        if (L->fast) {                                   /* 48-sample bits: a byte is 24 words, 12 32-bit stores */
            uint32_t i = e->i;
            const uint32_t n = e->nbytes;
            const uint8_t *fb = e->fb;
            uint32_t *q = (uint32_t *)(void *)(buf + (w & WS_RING_MASK));
            uint32_t *const qend = (uint32_t *)(void *)end;
            while (i < n && limit - w >= 24u) {
                const uint32_t v = fb[i++];
                const uint32_t *a = L->t.nib[v >> 4], *b = L->t.nib[v & 15u];
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
        } else {
            uint32_t p = e->p, cur = e->cur, i = e->i;
            const uint32_t n = e->nbytes, s1 = L->s1, tail = L->tail;
            const uint8_t *fb = e->fb;
            uint16_t *q = buf + (w & WS_RING_MASK);
            while (i < n && limit - w >= need) {
                uint32_t byte = fb[i++], k;
                for (k = 0; k < 8u; k++) {
                    const uint16_t *h = L->t.exact.h[(byte >> 7) & 1u][p];
                    const uint32_t a = L->t.exact.adv[p], h0 = h[0], h1 = h[1], h2 = h[2], hn = h[a];
                    q[0] = (uint16_t)(cur | h0);
                    q[1] = (uint16_t)h1;
                    q[2] = (uint16_t)h2;
                    cur = hn;
                    q += a;
                    w += a;
                    p = (p + s1) & 15u;
                    if (tail != 0u) {                    /* (400 kHz: the bit's LOW tail) */
                        uint32_t t = p + tail;
                        if (t >= 16u) {
                            *q++ = (uint16_t)cur;
                            w++;
                            t -= 16u;
                            cur = 0;
                            for (; t >= 16u; t -= 16u) {
                                *q++ = 0;
                                w++;
                            }
                        }
                        p = t;
                    }
                    byte <<= 1;
                }
                if (q >= end) {                          /* past the ring's end: those words to its start */
                    uint32_t j, over = (uint32_t)(q - end) + 3u;
                    for (j = 0; j < over; j++) {
                        buf[j] = end[j];
                    }
                    q -= WS_RING_WORDS;
                }
            }
            if (i == n) {                                /* the frame's end: the partial word out, LOW from here */
                if (p != 0u) {
                    *q = (uint16_t)cur;
                    w++;
                }
                p = 0;
                cur = 0;
                e->state = WS_LOW;
                e->low_w = 0;
            }
            e->p = (uint8_t)p;
            e->cur = (uint16_t)cur;
            e->i = i;
        }
        if (e->state == WS_FRAME) {
            break;
        }
    }
    e->w = w;
    return w - w0;
}
