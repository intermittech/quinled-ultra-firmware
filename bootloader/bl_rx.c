/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Streaming WS28xx decoder (see bl_rx.h). The hot loop keeps its state in locals; bits go into a 32-bit
 * accumulator, flushed into a circular buffer of 12 words, so a bit costs a shift and an add. */
#include "bl_rx.h"

#define RUN_SAT 0x40000000u
/* (the Diff-Power-UR-Digital-M, hardware id 3) on a pixel port the hub sends every Ultra frame behind a marker, a HIGH of
 * 15-40 us (docs/ur_digital_m-firmware.md section 4): a high run over high_max before a frame's first bit is that
 * marker, not a bit and not a bad frame */
#if defined(BL_HW_ID) && BL_HW_ID == 3
#define BL_RX_MARKER 1u
#else
#define BL_RX_MARKER 0u
#endif

/* leading zeros of a byte (8 for 0), as in core/ws_decode.c */
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

void bl_rx_init(bl_rx_t *r, const bl_rx_timing_t *t)
{
    r->t = *t;
    r->locked_period = 0;
    bl_rx_resync(r);
}

void bl_rx_resync(bl_rx_t *r)
{
    r->level = 0;
    r->run = 0;                                   /* the next gap counts from here */
    r->nbits = 0;
    r->acc = 0;
    r->wi = 0;
    r->synced = 0;
    r->ended = 1;
    r->bad = 0;
    r->high = r->period = 0;
#if MCU_BL_MARKER_TRACK
    r->marker = 0;
#endif
}

static void bit(bl_rx_t *r, uint32_t low)
{
    uint32_t period = r->high + low, p = r->period;
    if (!r->high) return;
    if (low < r->t.gap_min) {
        if (!p) {
            if (period >= MCU_BL_PER0_LO && period <= MCU_BL_PER0_HI) p = MCU_BL_PER0;
            else if (period >= MCU_BL_PER1_LO && period <= MCU_BL_PER1_HI) p = MCU_BL_PER1;
            else if (period >= MCU_BL_PER2_LO && period <= MCU_BL_PER2_HI) p = MCU_BL_PER2;
            else if (period >= MCU_BL_PER3_LO && period <= MCU_BL_PER3_HI) p = MCU_BL_PER3;
            else r->bad = 1;
            r->period = (uint16_t)p;
        }
        if (period < p * 7u / 8u || period > p * 9u / 8u) r->bad = 1;
    }
    if (!p || r->high < 2u || r->high > p - 2u ||
        (r->locked_period && r->locked_period != p)) r->bad = 1;
    r->acc = (r->acc << 1) | (r->high >= p / 2u);
    if ((++r->nbits & 31u) == 0u) {
        r->words[r->wi] = r->acc;
        r->wi = r->wi == BL_RX_TAIL_WORDS - 1u ? 0u : r->wi + 1u;
    }
    r->high = 0;
}

/* the frame's last min(nbits, 384) bits, right-aligned */
static void put_bit(uint8_t *t, uint32_t pos, uint32_t bit)
{
    if (bit) {
        t[pos >> 3] = (uint8_t)(t[pos >> 3] | (0x80u >> (pos & 7u)));
    }
}

static void emit(const bl_rx_t *r, bl_frame_t *out)
{
    /* the bits kept, oldest first: nw complete words before wi, then the `part` newest bits in acc */
    uint32_t part = r->nbits & 31u, nw = r->nbits >> 5, j, b;
    uint32_t k = r->nbits < BL_RX_TAIL_BITS ? r->nbits : BL_RX_TAIL_BITS, skip, pos = BL_RX_TAIL_BITS - k;
    if (nw > BL_RX_TAIL_WORDS) {
        nw = BL_RX_TAIL_WORDS;
    }
    skip = nw * 32u + part - k;                   /* older bits than the last 384 */
    for (j = 0; j < BL_RX_TAIL_BITS / 8u; j++) {
        out->tail[j] = 0;
    }
    for (j = 0; j < nw; j++) {
        uint32_t w = r->words[(r->wi + BL_RX_TAIL_WORDS - nw + j) % BL_RX_TAIL_WORDS];
        for (b = 0; b < 32u; b++) {
            if (skip) {
                skip--;
            } else {
                put_bit(out->tail, pos++, (w >> (31u - b)) & 1u);
            }
        }
    }
    for (j = 0; j < part; j++) {
        if (skip) {
            skip--;
        } else {
            put_bit(out->tail, pos++, (r->acc >> (part - 1u - j)) & 1u);
        }
    }
    out->nbits = r->nbits;
    out->bad = r->bad;
#if MCU_BL_MARKER_TRACK
    out->marker = r->marker;
#endif
    out->period = r->period;
}

/* the low run reached the gap (rare path, once per frame) */
static int gap(bl_rx_t *r, bl_frame_t *out)
{
    int got = 0;
    if (!r->ended) {
        r->ended = 1;
        bit(r, r->t.gap_min);
        if (r->synced && r->nbits != 0u) {
            emit(r, out);
            got = 1;
        }
    }
    r->synced = 1;
    r->nbits = 0;
    r->acc = 0;
    r->wi = 0;
    r->bad = 0;
    r->high = r->period = 0;
#if MCU_BL_MARKER_TRACK
    r->marker = 0;
#endif
    return got;
}

int bl_rx_feed(bl_rx_t *r, const uint16_t *w, uint32_t n, bl_frame_t *out)
{
    int got = 0;
    uint32_t level = r->level, run = r->run;
    const uint32_t hmax = r->t.high_max, gmin = r->t.gap_min;
    const uint16_t *end = w + n;

    while (w < end) {
        uint32_t v = *w++, tr, last = 0;
        if (v == (level ? 0xFFFFu : 0u)) {        /* no edge in this word (most words between bits / in gaps) */
            run += 16u;
            if (!level && run >= gmin) {
                if (!r->ended) {
                    got |= gap(r, out);
#if MCU_BL_RX_EDGE_AGE
                    out->end_ago = run + (uint32_t)(end - w) * 16u;
#endif
                } else {
                    r->synced = 1;
                }
                if (run > RUN_SAT) {
                    run = RUN_SAT;
                }
            }
            continue;
        }
        tr = (v ^ ((v >> 1) | (level << 15))) & 0xFFFFu;   /* bit set where a sample differs from the one before */
        do {
            uint32_t j = (tr & 0xFF00u) ? clz8[tr >> 8] : 8u + clz8[tr & 0xFFu];   /* sample of the edge */
            tr &= ~(0x8000u >> j);
            run += j - last;                      /* the old level lasted up to sample j */
            last = j;
            if (level) {                          /* falling edge: one bit */
                if (r->synced && !(BL_RX_MARKER && r->nbits == 0u && run >= MCU_BL_MARK_MIN && run <= MCU_BL_MARK_MAX)) {
                    if (run > hmax) {
                        r->bad = 1;
                    }
                    r->high = (uint16_t)(run > 65535u ? 65535u : run);
                }
#if MCU_BL_MARKER_TRACK
                else if (r->synced) r->marker = 1;
#endif
                level = 0;
            } else {                              /* rising edge */
                if (run >= gmin && !r->ended) {   /* a gap that ended inside this word */
                    got |= gap(r, out);
#if MCU_BL_RX_EDGE_AGE
                    out->end_ago = run + (uint32_t)(end - w) * 16u + 16u - last;
#endif
                } else if (run >= gmin) {
                    r->synced = 1;
                } else bit(r, run);
                level = 1;
                r->ended = 0;
            }
            run = 0;
        } while (tr != 0u);
        run += 16u - last;
        if (!level && run >= gmin && !r->ended) {
            got |= gap(r, out);
#if MCU_BL_RX_EDGE_AGE
            out->end_ago = run + (uint32_t)(end - w) * 16u;
#endif
        }
    }
    r->level = level;
    /* A compliant release can rise later in the same feed that ended the
     * frame. Its HIGH run must not replace the frame's saved edge age. */
    r->run = run;
    return got;
}
