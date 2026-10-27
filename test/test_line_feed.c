/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The acquisition feed (core/ws_decode.c ws_line_feed) against its word-by-word reference: the Digital-M's feed skips
 * the words without an edge; the state and the runs must be byte for byte the reference's. Random line waveforms
 * (800 / 600 / 400 / 200 kHz at 36 MHz sampling, glitches, long runs, a feed starting after a HIGH) and noise words,
 * fed in random chunks. Build: test\build_host.cmd (with /DBOARD=3, and without: the plain feed against the same
 * reference). Usage: line_feed_tests.exe [cases] [seed] */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../src/core/ws_decode.h"

#define LINE_MAXW 6000u

/* the reference: every word walked edge by edge (the feed before the Digital-M's skip) */
static uint8_t ref_clz8(uint32_t x) { uint8_t n = 0; while (!(x & 0x80u)) { x <<= 1; n++; } return n; }
static void ref_feed(ws_line_t *s, uint16_t *runs, const uint16_t *samples, uint32_t words)
{
    static const uint16_t line_period[4] = MCU_LINE_PERIODS;
    uint32_t k, prev = s->prev, base = s->pos, rise = s->rise, fall = s->fall, bits = s->bits;
    if (s->ended) return;
    for (k = 0; k < words; k++) {
        uint32_t w = samples[k], tr = (w ^ ((w >> 1) | (prev << 15))) & 0xFFFFu;
        prev = w & 1u;
        while (tr) {
            uint32_t j = (tr & 0xFF00u) ? ref_clz8(tr >> 8) : 8u + ref_clz8(tr & 255u);
            uint32_t p = base + (k << 4) + j;
            tr ^= 0x8000u >> j;
            if ((w << j) & 0x8000u) {
                if (s->have) {
                    uint32_t low = p - fall, period = p - rise;
                    if (low >= MCU_US2S(10u)) { s->ended = 1; goto done; }
                    if (period < s->period_min) s->period_min = (uint16_t)period;
                    if (period > s->period_max) s->period_max = (uint16_t)period;
                    if (low < s->low_min) s->low_min = (uint16_t)low;
                    if (low > s->low_max) s->low_max = (uint16_t)low;
                    if (s->intervals < 256u && ++s->intervals == 256u) {
                        uint32_t span = p - s->first, q;
                        for (q = 0; q < 4u; q++) {
                            uint32_t nominal = (uint32_t)line_period[q] * 256u;
                            if (span * 100u >= nominal * 92u && span * 100u <= nominal * 108u) s->profile = (uint8_t)q;
                        }
                        if (s->profile == 255) s->bad = 1;
                    }
                } else { s->first = p; s->have = 1; }
                rise = p;
            } else if (s->have) {
                uint32_t high = p - rise;
                if (high < s->high_min) s->high_min = (uint16_t)high;
                if (high > s->high_max) s->high_max = (uint16_t)high;
                if (!high || high >= MCU_US2S(10u)) s->bad = 1;
                if (bits < WS_LINE_HEAD) runs[bits] = (uint16_t)high;
                runs[WS_LINE_HEAD + (bits & 63u)] = (uint16_t)high;
                bits++;
                fall = p;
            }
        }
    }
done:
    s->rise = rise; s->fall = fall; s->bits = bits;
    s->prev = (uint8_t)prev;
    s->pos = base + (words << 4);
}

static uint32_t rng = 12345u;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
static uint16_t buf[LINE_MAXW];

int main(int argc, char **argv)
{
    static const uint32_t per[4] = { 45, 60, 90, 180 };    /* samples per bit at 36 MHz: 800 / 600 / 400 / 200 kHz */
    unsigned long cases = argc > 1 ? strtoul(argv[1], 0, 10) : 40000ul, c, fails = 0, bits = 0;
    if (argc > 2) rng = (uint32_t)strtoul(argv[2], 0, 10);
    for (c = 0; c < cases; c++) {
        uint32_t mode = rnd() % 8u, prof = rnd() % 4u, nw = 1u + rnd() % LINE_MAXW, i, pos = 0, level = rnd() & 1u;
        ws_line_t a, b;
        uint16_t ra[WS_LINE_WORDS], rb[WS_LINE_WORDS];
        memset(buf, 0, sizeof buf);
        if (mode == 0) {
            for (i = 0; i < nw; i++) buf[i] = (uint16_t)rnd();
        } else {
            while (pos < nw * 16u) {
                uint32_t len = level ? ((rnd() & 1u) ? per[prof] / 4u : per[prof] * 3u / 4u) + rnd() % 5u
                                     : per[prof] - per[prof] / 2u + rnd() % 9u;
                if (mode == 1 && rnd() % 64u == 0u) len = 1u + rnd() % 3u;            /* a glitch */
                if (mode == 2 && rnd() % 200u == 0u) len = 400u + rnd() % 2000u;      /* a long run: the end, a HIGH */
                for (i = 0; i < len && pos < nw * 16u; i++, pos++)
                    if (level) buf[pos >> 4] |= (uint16_t)(0x8000u >> (pos & 15u));
                level ^= 1u;
            }
        }
        memset(&a, 0, sizeof a); memset(&b, 0, sizeof b);
        ws_line_init(&a); ws_line_init(&b);
        memset(ra, 0, sizeof ra); memset(rb, 0, sizeof rb);
        if (rnd() & 1u) a.prev = b.prev = 1;                                          /* the feed starts after a HIGH */
        for (i = 0; i < nw;) {
            uint32_t n = 1u + rnd() % (mode == 3 ? 4u : 700u);
            if (n > nw - i) n = nw - i;
            ws_line_feed(&a, ra, buf + i, n);
            ref_feed(&b, rb, buf + i, n);
            i += n;
        }
        bits += a.bits;
        if (memcmp(&a, &b, sizeof a) != 0 || memcmp(ra, rb, sizeof ra) != 0) {
            if (fails++ < 5) printf("FAIL case %lu: mode %u, profile %u, %u words\n", c, mode, prof, nw);
        }
    }
    printf("line feed: %lu cases, %lu bits, %lu differ from the reference\n", cases, bits, fails);
    return fails != 0;
}
