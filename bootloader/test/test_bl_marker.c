/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* (round 11) the hardware-id-3 bootloader's marker skip (bl_rx.c BL_RX_MARKER: the QuinLED Diff-Power Ultra-Digital
 * receiver). On a pixel port the hub sends every Ultra frame behind a marker (docs/ultradigital-firmware.md section
 * 4): >= 350 us LOW, a HIGH of 15-40 us, a 5-20 us gap, then the frame. Built twice by build_bl_test.cmd:
 *   /DBL_HW_ID=3  every UPD frame behind a marker decodes: 384 bits, not bad, its bytes;
 *   (hardware 1)  the negative control: the same waveform's marker is a bad first bit (385 bits, bad).
 * Both: a frame without a marker decodes as before, and a long HIGH inside a frame (not before its first bit) stays a
 * bad frame. Fed in random chunks (1..BL_RX_MAX_FEED words), as the main loop does. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bl_rx.h"

#if defined(BL_HW_ID) && BL_HW_ID == 3
#define MARKER_SKIPPED 1
#else
#define MARKER_SKIPPED 0
#endif

static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
                           printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint32_t rs = 0x2545F491u;
static uint32_t rnd(void)
{
    rs ^= rs << 13;
    rs ^= rs >> 17;
    rs ^= rs << 5;
    return rs;
}

typedef struct {
    uint16_t *w;
    uint32_t nw, cap, bit;
    double frac;
} wave_t;

static void put(wave_t *wv, uint32_t level, double ns)          /* (test_bl.c wave_put: 36 MHz, MSB first) */
{
    double s = ns * 1e-9 * 36e6 + wv->frac;
    uint32_t n = (uint32_t)s, i;
    wv->frac = s - n;
    for (i = 0; i < n; i++) {
        if (wv->bit == 0u) {
            if (wv->nw == wv->cap) {
                wv->cap = wv->cap ? wv->cap * 2u : 8192u;
                wv->w = (uint16_t *)realloc(wv->w, wv->cap * 2u);
            }
            wv->w[wv->nw++] = 0;
        }
        if (level) {
            wv->w[wv->nw - 1u] |= (uint16_t)(0x8000u >> wv->bit);
        }
        wv->bit = (wv->bit + 1u) & 15u;
    }
}

/* the hub's bits: '1' 700 / 650 ns, '0' 250 / 1050 ns, +-40 ns of edge jitter; long_at: a 10 us HIGH at that bit */
static void frame(wave_t *wv, const uint8_t *b, uint32_t nbits, int long_at)
{
    uint32_t k;
    for (k = 0; k < nbits; k++) {
        int one = (b[k / 8u] >> (7u - k % 8u)) & 1;
        double e = ((int)(rnd() % 81u) - 40);
        put(wv, 1, (int)k == long_at ? 10000.0 : (one ? 700.0 : 250.0) + e);
        put(wv, 0, (one ? 650.0 : 1050.0) - e);
    }
}

typedef struct { uint32_t n, nbits[16]; uint8_t bad[16], tail[16][48]; } got_t;

static void decode(const wave_t *wv, got_t *g)
{
    static const bl_rx_timing_t t = BL_RX_TIMING_36MHZ;
    bl_rx_t r;
    uint32_t pos = 0;
    bl_rx_init(&r, &t);
    g->n = 0;
    while (pos < wv->nw) {
        bl_frame_t f;
        uint32_t n = 1u + rnd() % BL_RX_MAX_FEED;
        if (n > wv->nw - pos) {
            n = wv->nw - pos;
        }
        if (bl_rx_feed(&r, wv->w + pos, n, &f) && g->n < 16u) {
            g->nbits[g->n] = f.nbits;
            g->bad[g->n] = f.bad;
            memcpy(g->tail[g->n], f.tail, 48);
            g->n++;
        }
        pos += n;
    }
}

int main(void)
{
    static const double mk_us[] = { 15.0, 20.0, 40.0, 27.5 }, gap_us[] = { 5.0, 10.0, 20.0, 12.5 };
    uint8_t b[4][48];
    uint32_t k, i, run;
    printf("bootloader marker skip (hardware id %s): UPD frames behind 15-40 us markers\n",
           MARKER_SKIPPED ? "3, the Ultra-Digital receiver: skipped" : "1: the negative control, a bad first bit");
    for (run = 0; run < 50u; run++) {
        wave_t wv = { 0 };
        got_t g;
        for (k = 0; k < 4u; k++) {
            for (i = 0; i < 48u; i++) {
                b[k][i] = (uint8_t)rnd();
            }
        }
        put(&wv, 0, 400e3);                              /* the hub's >= 350 us LOW */
        for (k = 0; k < 4u; k++) {
            put(&wv, 1, mk_us[k] * 1e3);                 /* the marker */
            put(&wv, 0, gap_us[k] * 1e3);
            frame(&wv, b[k], 384u, -1);
            put(&wv, 0, 400e3);
        }
        frame(&wv, b[0], 384u, -1);                      /* a frame without a marker */
        put(&wv, 0, 400e3);
        put(&wv, 1, 20e3);                               /* a marker, then a frame with a 10 us HIGH at bit 100 */
        put(&wv, 0, 10e3);
        frame(&wv, b[1], 384u, 100);
        put(&wv, 0, 400e3);
        decode(&wv, &g);
        CHECK(g.n == 6u, "run %u: 6 frames decoded (%u)", run, g.n);
        for (k = 0; k < 4u && k < g.n; k++) {
            if (MARKER_SKIPPED) {
                CHECK(g.nbits[k] == 384u && !g.bad[k] && memcmp(g.tail[k], b[k], 48) == 0, "run %u, marker %.1f us, "
                      "gap %.1f us: 384 bits, not bad, its bytes (%u bits, bad %u)", run, mk_us[k], gap_us[k],
                      g.nbits[k], g.bad[k]);
            } else {
                CHECK(g.nbits[k] == 385u && g.bad[k], "run %u, marker %.1f us: the marker a bad first bit (%u bits, "
                      "bad %u)", run, mk_us[k], g.nbits[k], g.bad[k]);
            }
        }
        if (g.n == 6u) {
            CHECK(g.nbits[4] == 384u && !g.bad[4] && memcmp(g.tail[4], b[0], 48) == 0, "run %u: a frame without a "
                  "marker as before (%u bits, bad %u)", run, g.nbits[4], g.bad[4]);
            CHECK(g.bad[5] && g.nbits[5] == (MARKER_SKIPPED ? 384u : 385u), "run %u: a long HIGH inside a frame stays "
                  "bad (%u bits, bad %u)", run, g.nbits[5], g.bad[5]);
        }
        free(wv.w);
    }
    printf("%u checks, %u failed\n", checks, fails);
    return fails != 0;
}
