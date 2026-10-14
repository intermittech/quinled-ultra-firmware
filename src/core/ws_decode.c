/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "ws_decode.h"
#include "lto.h"

/* leading zeros of a byte (8 for 0) */
static const uint8_t clz8[256] = {
    8, 7, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

/* Walks the level transitions word by word: t has a bit set wherever a sample differs from the previous
 * one, so each word costs a few instructions plus ~10 per edge (two edges per WS bit). */
MEASURED int ws_decode_frame(const uint16_t *buf, uint32_t nsamples, uint16_t nbits, const ws_timing_t *t, uint8_t *out)
{
    uint32_t nwords = nsamples >> 4, k, b = 0, prev = 0, rise = 0, fall = 0, seen_rise = 0;
    const uint32_t thr = t->thr_one, hmax = t->high_max, gmin = t->gap_min;

    if (nbits == 0u) {
        return 0;
    }
    for (k = 0; k < (uint32_t)(nbits + 7u) / 8u; k++) {
        out[k] = 0;
    }
    if (nwords == 0u || (buf[0] & 0x8000u) != 0u) {
        return WS_ERR_NO_GAP;                         /* must start inside the reset gap */
    }
    for (k = 0; k < nwords; k++) {
        uint32_t w = buf[k];
        uint32_t tr = (w ^ ((w >> 1) | (prev << 15))) & 0xFFFFu;
        prev = w & 1u;
        while (tr != 0u) {
            uint32_t j = (tr & 0xFF00u) ? clz8[tr >> 8] : 8u + clz8[tr & 0xFFu];
            uint32_t pos = (k << 4) + j;
            tr &= ~(0x8000u >> j);
            if ((w << j) & 0x8000u) {                 /* rising edge */
                if (seen_rise && pos - fall >= gmin) {
                    return WS_ERR_SHORT;
                }
                rise = pos;
                seen_rise = 1;
            } else if (seen_rise) {                   /* falling edge: one bit done */
                uint32_t high = pos - rise;
                if (high > hmax) {
                    return WS_ERR_PULSE;
                }
                if (high >= thr) {
                    out[b >> 3] |= (uint8_t)(0x80u >> (b & 7u));
                }
                if (++b == nbits) {
                    return (int)nbits;
                }
                fall = pos;
            }
        }
    }
    return seen_rise ? WS_ERR_TRUNCATED : WS_ERR_NO_EDGE;
}

MEASURED int ws_decode_last(const uint16_t *buf, uint32_t nsamples, const ws_timing_t *t, uint8_t *out)
{
    uint32_t nwords = nsamples >> 4, k, n = 0, prev, rise = 0, fall = 0, seen_rise = 0, hi = 0, lo = 0;
    const uint32_t thr = t->thr_one, hmax = t->high_max, gmin = t->gap_min;

    if (nwords == 0u) {
        return WS_ERR_NO_EDGE;
    }
    prev = (buf[0] & 0x8000u) ? 1u : 0u;              /* started inside a high pulse: its fall is not a bit */
    for (k = 0; k < nwords; k++) {
        uint32_t w = buf[k];
        uint32_t tr = (w ^ ((w >> 1) | (prev << 15))) & 0xFFFFu;
        if (k == 0u) {
            tr &= 0x7FFFu;                               /* no transition before the first sample */
        }
        prev = w & 1u;
        while (tr != 0u) {
            uint32_t j = (tr & 0xFF00u) ? clz8[tr >> 8] : 8u + clz8[tr & 0xFFu];
            uint32_t pos = (k << 4) + j;
            tr &= ~(0x8000u >> j);
            if ((w << j) & 0x8000u) {                 /* rising edge */
                if (seen_rise && pos - fall >= gmin) {
                    return WS_ERR_SHORT;               /* a reset gap inside the window */
                }
                rise = pos;
                seen_rise = 1;
            } else if (seen_rise) {                   /* falling edge: one bit done */
                uint32_t high = pos - rise;
                if (high > hmax) {
                    return WS_ERR_PULSE;
                }
                hi = (hi << 1) | (lo >> 31);
                lo = (lo << 1) | (high >= thr ? 1u : 0u);
                n++;
                fall = pos;
            }
        }
    }
    if (n == 0u) {
        return WS_ERR_NO_EDGE;
    }
    for (k = 0; k < 4u; k++) {
        out[k] = (uint8_t)(hi >> (24u - 8u * k));
        out[4u + k] = (uint8_t)(lo >> (24u - 8u * k));
    }
    return (int)n;
}
