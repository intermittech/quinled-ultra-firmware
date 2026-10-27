/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Soft start (core/sstart.c) and the outputs' factor in isense_apply, included by test_main.c: nothing before the
 * first light; then dark for the chip-id offset (0..100 ms), 20 % -> 100 % over 300 ms, monotonic, then done; a
 * trip's retry runs it again; off = never; the offsets of many chip ids spread over 0..100 ms; isense_apply scales the
 * outputs by it without touching the limiter (k, limiting) and keeps an output that should be on at least 1. */
#include "../src/core/sstart.h"

static void test_sstart(void)
{
    sstart_t s;
    uint32_t t, f, prev, bad = 0, seen[101];
    isense_t is;
    uint16_t req[5] = { 40000, 0, 65535, 1, 0 }, out[5];

    printf("soft start: first light, offset, ramp, retry, off, chip-id spread, isense_apply\n");
    ss_init(&s, 1234567u);
    CHECK(s.offset_ms == 1234567u % 101u, "offset from the id: %u", s.offset_ms);
    for (t = 0; t < 5000u; t++) {
        bad += ss_factor(&s, t, 0, 1) != 65536u;          /* dark: nothing to ramp */
    }
    CHECK(bad == 0 && s.state == SS_ARMED, "before the first light: factor 1, still armed");
    /* the first light at 5000 ms */
    prev = 0;
    for (t = 5000u; t < 5000u + s.offset_ms; t++) {
        bad += ss_factor(&s, t, 1, 1) != 0u;
    }
    CHECK(bad == 0, "dark for the offset (%u ms)", s.offset_ms);
    f = ss_factor(&s, 5000u + s.offset_ms, 1, 1);
    CHECK(f == SS_START_Q16, "the ramp starts at 20 %% (%u)", f);
    for (t = 5000u + s.offset_ms; t < 5000u + s.offset_ms + SS_RAMP_MS; t++) {
        f = ss_factor(&s, t, 1, 1);
        bad += f < prev || f > 65536u;
        prev = f;
    }
    CHECK(bad == 0 && f > 65000u, "ramp monotonic to 100 %% (%u at its end)", f);
    CHECK(ss_factor(&s, t, 1, 1) == 65536u && s.state == SS_DONE, "done after %u ms", SS_RAMP_MS);
    CHECK(ss_factor(&s, t + 100000u, 0, 1) == 65536u && ss_factor(&s, t + 100001u, 1, 1) == 65536u,
          "later lights: no ramp");
    /* half way */
    ss_retry(&s, 100000u);
    f = ss_factor(&s, 100000u + s.offset_ms + SS_RAMP_MS / 2u, 1, 1);
    CHECK(f >= 39000u && f <= 40000u, "the retry runs it again: 60 %% half way (%u)", f);
    ss_retry(&s, 200000u);
    CHECK(ss_factor(&s, 200000u, 1, 0) == 65536u && s.state == SS_DONE, "off: no ramp, also after a retry");
    ss_init(&s, 99u);
    CHECK(ss_factor(&s, 10, 1, 0) == 65536u && ss_factor(&s, 11, 1, 1) == 65536u, "off at the first light: done");

    /* 1000 chip ids (as hw_uid_hash spreads them): every offset in 0..100 is used, none piles up */
    memset(seen, 0, sizeof seen);
    for (t = 0; t < 1000u; t++) {
        uint32_t h = ((uint32_t)rnd() ^ (uint32_t)(rnd() >> 32)) * 0x9E3779B9u;
        ss_init(&s, h ^ (h >> 16));
        seen[s.offset_ms]++;
    }
    for (t = 0, f = 0, prev = 0; t <= 100u; t++) {
        f += seen[t] == 0u;
        prev = seen[t] > prev ? seen[t] : prev;
    }
    CHECK(f <= 3u && prev <= 25u, "offsets: %u of 101 unused, at most %u on one", f, prev);

    /* isense_apply: the outputs scaled, the limiter untouched */
    isense_init(&is, 1, 10000, 8000);
    is.k_ss = 32768u;
    isense_apply(&is, req, out);
    CHECK(out[0] == 20000u && out[1] == 0u && out[2] == 32768u && out[3] == 1u && is.k == 65536u && !is.limiting &&
          is.applied[0] == 20000u, "k_ss 0.5: %u %u %u %u, k %u, limiting %u", out[0], out[1], out[2], out[3], is.k,
          is.limiting);
    is.k_ss = 0;
    isense_apply(&is, req, out);
    CHECK(out[0] == 1u && out[2] == 1u && out[1] == 0u && !is.limiting, "k_ss 0: on stays at 1");
    is.k_ss = 65536u;
    isense_apply(&is, req, out);
    CHECK(out[0] == 40000u && out[2] == 65535u && out[3] == 1u, "k_ss 1: unchanged");
}
