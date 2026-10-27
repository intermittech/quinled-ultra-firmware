/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Current calibration and the board limit (included by test_main.c): the calibration page (core/calstore.c: records,
 * newest wins, wear, clearing), the limit in effect (core/climit.c: sources, "no limit needed"), the model with curves
 * (core/isense.c without a plant) and the chain-wide test: one Ultra frame calibrates every board (or a selection) at
 * once, repeats don't restart it. */
#include "../src/core/calstore.h"
#include "../src/core/climit.h"
#include "../src/core/dark.h"
#include "../src/core/gamma.h"
#include "../src/core/isense.h"

static void cal_example(isense_cal_t *c, uint16_t scale)
{
    static const uint16_t ma[5][4] = { { 495, 981, 1461, 1932 }, { 372, 739, 1102, 1462 }, { 0, 0, 0, 0 },
                                       { 1099, 2158, 3183, 4172 }, { 739, 1459, 2163, 2850 } };
    uint32_t i, l;
    memset(c, 0, sizeof *c);
    c->levels = 4;
    c->pct[0] = 25; c->pct[1] = 50; c->pct[2] = 75; c->pct[3] = 100;
    c->zero_ma = 23;
    for (i = 0; i < 5u; i++) {
        for (l = 0; l < 4u; l++) {
            c->ma[i][l] = (uint16_t)(ma[i][l] * scale / 100u);
        }
    }
}

/* the page as hw.c writes it: next free slot, erase when full */
static uint8_t cal_page[CAL_PAGE_BYTES];
static uint32_t cal_pg_seq, cal_pg_erases;
static uint16_t cal_pg_count;
static void cal_page_write(const isense_cal_t *c)
{
    cal_rec_t rec;
    int slot = cal_next_free(cal_page);
    if (c->levels != 0u) {
        cal_pg_count++;
    }
    cal_pack(c, ++cal_pg_seq, cal_pg_count, &rec);
    if (slot >= (int)CAL_RECS) {
        memset(cal_page, 0xFF, sizeof cal_page);
        cal_pg_erases++;
        slot = 0;
    }
    memcpy(cal_page + (uint32_t)slot * CAL_REC_BYTES, &rec, sizeof rec);
}

static void test_calstore(void)
{
    isense_cal_t a, b, got;
    uint32_t seq, n;
    uint16_t count;
    int idx;
    printf("calibration page: records, newest wins, wear, clearing\n");
    memset(cal_page, 0xFF, sizeof cal_page);
    CHECK(cal_find(cal_page, &got, &seq, &count) < 0 && cal_next_free(cal_page) == 0, "empty page");
    cal_example(&a, 100);
    cal_example(&b, 90);
    cal_page_write(&a);
    idx = cal_find(cal_page, &got, &seq, &count);
    CHECK(idx == 0 && memcmp(&got, &a, sizeof a) == 0 && count == 1u, "one record round trip");
    for (n = 0; n < 10u; n++) {
        cal_page_write(n & 1u ? &a : &b);
    }
    idx = cal_find(cal_page, &got, &seq, &count);
    CHECK(memcmp(&got, &a, sizeof a) == 0 && count == 11u && seq == 11u, "newest of 11 (count %u seq %u)",
          (unsigned)count, (unsigned)seq);
    CHECK(cal_pg_erases == 2u, "4 records per erase: %u erases for 11", (unsigned)cal_pg_erases);
    /* a torn write (bad CRC) in the newest slot: the one before it wins */
    cal_page[(uint32_t)idx * CAL_REC_BYTES + 20u] ^= 0x40u;
    (void)cal_find(cal_page, &got, &seq, &count);
    CHECK(seq == 10u && memcmp(&got, &b, sizeof b) == 0, "corrupt newest record skipped");
    cal_page[(uint32_t)idx * CAL_REC_BYTES + 20u] ^= 0x40u;
    /* factory reset: a cleared record */
    memset(&got, 0, sizeof got);
    cal_page_write(&got);
    (void)cal_find(cal_page, &got, &seq, &count);
    CHECK(got.levels == 0u && count == 11u, "cleared record wins (levels 0, count kept)");
    /* the CH32V203's erased value 0xE339 counts as erased too */
    {
        uint8_t pg[CAL_PAGE_BYTES];
        uint32_t i;
        for (i = 0; i < CAL_PAGE_BYTES; i += 2) { pg[i] = 0x39; pg[i + 1] = 0xE3; }
        CHECK(cal_next_free(pg) == 0 && cal_find(pg, &got, &seq, &count) < 0, "0xE339 erased page");
    }
}

static void test_isense_curves(void)
{
    isense_t is;
    isense_cal_t c, back;
    uint16_t v[5];
    uint32_t m;
    printf("isense model with curves (no plant)\n");
    isense_init(&is, 1, 10000, 8000);
    CHECK(isense_connected_ma(&is) == 0u, "no model: connected load not known");
    cal_example(&c, 100);
    isense_cal_set(&is, &c);
    CHECK(isense_model_ok(&is) && is.calibrated, "restored test makes the model calibrated");
    CHECK(isense_cal_get(&is, &back) && memcmp(&back, &c, sizeof c) == 0, "test round trip");
    CHECK(isense_connected_ma(&is) == 1932u + 1462u + 4172u + 2850u, "connected = sum of the 100 %% points");
    CHECK(is.ch_ma[2] == 0, "nothing connected on B");
    /* the curve passes through the measured points */
    CHECK(abs((int)isense_ch_ma(&is, 3, 16384) - 1099) <= 2 && abs((int)isense_ch_ma(&is, 3, 32768) - 2158) <= 2 &&
          abs((int)isense_ch_ma(&is, 3, 49152) - 3183) <= 2 && abs((int)isense_ch_ma(&is, 3, 65535) - 4172) <= 2,
          "curve through the points");
    CHECK(abs((int)isense_ch_ma(&is, 3, 8192) - 550) <= 2, "below 25 %%: linear to 0 (%u)",
          (unsigned)isense_ch_ma(&is, 3, 8192));
    m = isense_ch_ma(&is, 3, 40960);                        /* 62.5 %: halfway between 2158 and 3183 */
    CHECK(abs((int)m - 2670) <= 3, "between points: linear (%u)", (unsigned)m);
    v[0] = 65535; v[1] = 0; v[2] = 65535; v[3] = 32768; v[4] = 16384;
    CHECK(abs((int)isense_model_ma(&is, v) - (1932 + 0 + 0 + 2158 + 739)) <= 4, "board model");
    /* the current while on: higher at low duty with sag (the stagger weights) */
    CHECK(isense_on_ma(&is, 3, 16384) > isense_on_ma(&is, 3, 65535) && abs((int)isense_on_ma(&is, 3, 65535) - 4172) <= 3,
          "current while on from the curve (%u at 25 %%, %u at 100 %%)", (unsigned)isense_on_ma(&is, 3, 16384),
          (unsigned)isense_on_ma(&is, 3, 65535));
    /* a single-level test: linear curve, I_k scaled from its duty */
    memset(&c, 0, sizeof c);
    c.levels = 1; c.pct[0] = 25; c.zero_ma = 20;
    c.ma[0][0] = 500; c.ma[3][0] = 1250;
    isense_cal_set(&is, &c);
    CHECK(abs(is.ch_ma[0] - 2000) <= 2 && abs(is.ch_ma[3] - 5000) <= 3 && abs((int)isense_ch_ma(&is, 0, 32768) - 1000) <= 2,
          "single level: linear (%d %d)", (int)is.ch_ma[0], (int)is.ch_ma[3]);
    /* invalid tests are ignored */
    c.levels = 3;
    isense_cal_set(&is, &c);
    CHECK(abs(is.ch_ma[0] - 2000) <= 2, "3 levels ignored");
    /* restoring the 100 % values from the settings extension resets the curves to linear */
    {
        int32_t ma[5] = { 1000, 1000, 1000, 1000, 1000 };
        cal_example(&c, 100);
        isense_cal_set(&is, &c);
        isense_set_model(&is, ma, 20);
        CHECK(isense_ch_ma(&is, 3, 16384) == 250u, "extension restore: linear");
    }
}

/* the calibration's own sequence without a plant: each board measures its synthetic load */
static void run_board_cal(isense_t *is, const double load[5], double zero_a, isense_cal_t *out, uint32_t *ms)
{
    uint16_t req[5] = { 1000, 2000, 3000, 4000, 5000 }, o[5];
    uint32_t ev = 0, t;
    isense_cal_start(is, 0);
    isense_apply(is, req, o);
    for (t = 0; t < 2000u && !(ev & ISENSE_EV_CAL_DONE); t++) {
        double a = zero_a;
        uint32_t k, raw;
        for (k = 0; k < 5u; k++) {
            double d = o[k] / 65535.0;
            a += load[k] * d * (1.0 - 0.06 * d);           /* a little sag at high duty */
        }
        raw = (uint32_t)(a * 1000.0 * 65536.0 / is->ma_per_lsb_q16 + 0.5);
        ev = isense_ms(is, raw * 20u, 20u, raw);
        isense_apply(is, req, o);
    }
    *ms = t;
    (void)isense_cal_get(is, out);
}

static void test_chain_calibrate(void)
{
    enum { NB = 6 };
    static ultra_t ub[NB];
    static isense_t ib[NB];
    uint8_t frame[NB][ULTRA_SLOT_BYTES];
    uint8_t seq[NB];
    uint32_t b, started = 0;
    printf("chain-wide calibration: one Ultra frame starts every board (or a selection)\n");
    for (b = 0; b < NB; b++) {
        ultra_init(&ub[b], 100u + b);
        isense_init(&ib[b], 1, 10000, 8000);
        seq[b] = (uint8_t)(7u + b);
    }
    /* the hub's frame: slot k for board k, all of them IDENTIFY 2 with duty 0 (the standard test) */
    for (b = 0; b < NB; b++) {
        ultra_slot_calibrate(frame[b], seq[b], 0);
    }
    for (b = 0; b < NB; b++) {
        ultra_on_header(&ub[b], tk(100), HOST0 + 100000u);
        if (ultra_on_slot(&ub[b], frame[b], tk(100)) == ULTRA_EV_IDENTIFY && ub[b].identify == ULTRA_IDENT_CAL &&
            ub[b].identify_arg == 0u) {
            started |= 1u << b;
        }
    }
    CHECK(started == (1u << NB) - 1u, "every board starts from the same frame (0x%x)", (unsigned)started);
    /* the hub repeats the command (repeats = 3): the same SEQ is not applied again */
    for (b = 0; b < NB; b++) {
        CHECK(ultra_on_slot(&ub[b], frame[b], tk(102)) == 0u, "repeat %u restarted the test", (unsigned)b);
    }
    /* a selection: boards 1 and 4 get the command, the others a refresh of something else */
    started = 0;
    for (b = 0; b < NB; b++) {
        if (b == 1u || b == 4u) {
            ultra_slot_calibrate(frame[b], (uint8_t)(seq[b] + 1u), 0);
        } else {
            ultra_slot_identify(frame[b], (uint8_t)(seq[b] + 50u), ULTRA_IDENT_STOP);
        }
        if (ultra_on_slot(&ub[b], frame[b], tk(200)) == ULTRA_EV_IDENTIFY && ub[b].identify == ULTRA_IDENT_CAL) {
            started |= 1u << b;
        }
    }
    CHECK(started == 0x12u, "selection: boards 1 and 4 only (0x%x)", (unsigned)started);
    /* every board's test runs on its own: same length, its own load */
    {
        uint32_t ms[NB], ok = 1;
        for (b = 0; b < NB; b++) {
            double load[5] = { 0.5 + 0.3 * b, 1.0, 0.0, 2.0 + 0.4 * b, 1.5 };
            isense_cal_t c;
            run_board_cal(&ib[b], load, 0.025, &c, &ms[b]);
            if (ms[b] != isense_cal_ms(0) || c.levels != 4u || abs((int)c.ma[0][3] - (int)(load[0] * 940.0)) > 30 ||
                c.ma[2][3] != 0u || abs((int)c.ma[3][1] - (int)(load[3] * 485.0)) > 30) {
                ok = 0;
                printf("  board %u: %u ms, %u / %u / %u mA\n", (unsigned)b, (unsigned)ms[b], c.ma[0][3], c.ma[2][3],
                       c.ma[3][1]);
            }
        }
        CHECK(ok, "parallel tests: length %u ms and values", (unsigned)isense_cal_ms(0));
        printf("  6 boards in parallel: %u ms each from the frame end, visible: each channel 25 -> 100 %% for 180 ms\n",
               (unsigned)isense_cal_ms(0));
    }
}

static void test_climit(void)
{
    climit_t c;
    printf("board limit in effect: sources, no limit needed\n");
    climit_eval(&c, 0, 8000, 10000, 0);
    CHECK(c.limit_ma == 8000u && c.src == CLIMIT_SRC_BOARD && c.need == CLIMIT_NEED_UNKNOWN, "board rating, load unknown");
    climit_eval(&c, 5000, 8000, 10000, 4200);
    CHECK(c.limit_ma == 5000u && c.src == CLIMIT_SRC_USER && c.need == CLIMIT_NEED_NO, "user limit, no limit needed");
    climit_eval(&c, 5000, 8000, 10000, 9000);
    CHECK(c.need == CLIMIT_NEED_YES, "connected above the limit");
    climit_eval(&c, 0, 8000, 10000, 8000);
    CHECK(c.need == CLIMIT_NEED_NO, "connected exactly at the limit: never above it");
    climit_eval(&c, 12000, 8000, 10000, 9000);
    CHECK(c.limit_ma == 8000u && c.src == CLIMIT_SRC_BOARD, "a user limit above the rating is ignored");
    climit_eval(&c, 0, 12000, 10000, 0);
    CHECK(c.limit_ma == 10000u && c.src == CLIMIT_SRC_FUSE, "the fuse");
    climit_eval(&c, 0, 0, 0, 7000);
    CHECK(c.limit_ma == 0xFFFFu && c.src == CLIMIT_SRC_NONE && c.need == CLIMIT_NEED_UNKNOWN,
          "nothing given: none (boards without current sense, no CONFIG 21)");
    climit_eval(&c, 3000, 0, 0, 7000);
    CHECK(c.limit_ma == 3000u && c.src == CLIMIT_SRC_USER && c.need == CLIMIT_NEED_YES,
          "CONFIG 21 on a board without current sense");
}

/* the dark hold of a pin's calibration (core/dark.c): fade to dark, hold, release / time out, no jumps */
static void test_dark(void)
{
    dark_t d;
    uint16_t v[5];
    uint32_t t, prev = 65535u, jumps = 0, mono = 1;
    uint8_t s[ULTRA_SLOT_BYTES];
    printf("dark hold (IDENTIFY 3 / 4): fade, hold, release, time-out\n");
    dark_init(&d);
    CHECK(dark_factor(&d, 1000) == 65535u && dark_tick(&d, 1000) == 0u, "idle: full, nothing to refresh");
    dark_hold(&d, 1000, 5000);
    for (t = 1000; t <= 1000 + DARK_FADE_MS; t++) {
        uint32_t f = dark_factor(&d, t);
        if (f > prev) mono = 0;
        if (prev - f > 65535u / DARK_FADE_MS + 2u) jumps++;
        prev = f;
    }
    CHECK(mono && jumps == 0u && dark_factor(&d, 1000 + DARK_FADE_MS) == 0u, "fades to dark in %u ms, no jump",
          (unsigned)DARK_FADE_MS);
    v[0] = 65535; v[1] = 40000; v[2] = 1; v[3] = 0; v[4] = 30000;
    dark_apply(&d, 2000, v);
    CHECK(v[0] == 0u && v[1] == 0u && v[2] == 0u && v[4] == 0u, "dark while held");
    CHECK(dark_tick(&d, 3000) == 0u && d.active, "held: nothing to refresh");
    /* the hub's release, half-way into a new hold's fade: continues from where it is */
    dark_init(&d);
    dark_hold(&d, 10000, 60000);
    dark_release(&d, 10150);
    CHECK(abs((int)dark_factor(&d, 10150) - 32767) < 400 && dark_factor(&d, 10150 + DARK_FADE_MS) == 65535u,
          "release mid-fade: from the current level back to full");
    /* never released: back on its own when the hold time is up */
    dark_hold(&d, 20000, 3000);
    CHECK(dark_tick(&d, 22999) == 0u && d.active, "still held before the time");
    (void)dark_tick(&d, 23000);
    CHECK(!d.active && dark_factor(&d, 23000 + DARK_FADE_MS) == 65535u, "time-out releases");
    v[0] = 50000;
    dark_apply(&d, 23000 + DARK_FADE_MS, v);
    CHECK(v[0] == 50000u, "after the release: outputs untouched");
    /* the finished fade settles (dark_tick runs every ms). With level != target kept, the 300 ms fade up from dark
     * would replay on the outputs when the ms counter comes round again (49.7 days later). */
    (void)dark_tick(&d, 23000u + DARK_FADE_MS + 1u);
    for (t = 0, jumps = 0; t < 20000u; t++) {
        uint32_t now = 23000u + 0xFFFFFFFFu - 9999u + t;    /* 10 s either side of 2^32 ms after the release */
        jumps += dark_tick(&d, now) != 0u;
        jumps += dark_factor(&d, now) != 65535u;
    }
    CHECK(jumps == 0u, "no fade replay when the ms counter comes round again (%u ms off)", jumps);
    /* the commands on the wire (the board engine takes them as IDENTIFY events) */
    ultra_init(&U, 21);
    ultra_slot_hold_dark(s, 30, 45);
    CHECK(send(&U, 100, s) == ULTRA_EV_IDENTIFY && U.identify == ULTRA_IDENT_DARK && U.identify_arg == 45, "DARK slot");
    ultra_slot_identify(s, 31, ULTRA_IDENT_RELEASE);
    CHECK(send(&U, 101, s) == ULTRA_EV_IDENTIFY && U.identify == ULTRA_IDENT_RELEASE, "RELEASE slot");
}

/* the ~100 ms average of the board current (`current=` in status, `watch`): on a steady current it must settle on the
 * measurement itself. An integer step (i_ms - i_avg) / 64 would stop up to 63 mA short (2.00 A read 1.94 A) and
 * stay up to 63 mA above zero after the light goes out. */
static void test_isense_average(void)
{
    isense_t is;
    uint32_t t;
    printf("current sense: the ~100 ms average settles on a steady current\n");
    isense_init(&is, 1, 10000, 8000);                   /* ISENSE_GAIN_DEFAULT_Q16: 500 counts = 2002 mA */
    for (t = 0; t < 2000u; t++) {
        (void)isense_ms(&is, 500u * 20u, 20u, 500u);
    }
    CHECK(is.i_avg == is.i_ms, "steady 2 A: the average is the measurement (%d vs %d mA)", (int)is.i_avg, (int)is.i_ms);
    for (t = 0; t < 2000u; t++) {
        (void)isense_ms(&is, 0u, 20u, 0u);
    }
    CHECK(is.i_avg == 0 && is.i_ms == 0, "back to 0 A: the average follows to 0 (%d mA)", (int)is.i_avg);
    for (t = 0; t < 2000u; t++) {
        (void)isense_ms(&is, 1u * 20u, 20u, 1u);        /* 1 count: 4 mA, the smallest step */
    }
    CHECK(is.i_avg == is.i_ms && is.i_ms == 4, "4 mA: followed exactly (%d mA)", (int)is.i_avg);
}

/* WS2805 gamma through ultra_gamma (core/gamma.c) against round(65535 x (in / 255) ^ g): within 1 count (no
 * correction list; test_gamma_tables.c compares with the former output too) */
static void test_gamma_exact(void)
{
    uint32_t m, i, off = 0, bad = 0;
    printf("WS2805 gamma 2.2 / 2.8 against round(65535 x (in / 255) ^ g)\n");
    for (m = GAMMA_22; m <= GAMMA_28; m++) {
        double g = m == GAMMA_22 ? 2.2 : 2.8;
        for (i = 0; i < 256u; i++) {
            long want = lround(65535.0 * pow(i / 255.0, g));
            long got = gamma_apply((uint8_t)m, (uint8_t)i);
            if (i > 0u && want == 0) want = 1;
            off += got != want;
            bad += labs(got - want) > 1;
        }
    }
    printf("  %u of 512 one count off, %u more\n", (unsigned)off, (unsigned)bad);
    CHECK(bad == 0u, "gamma: %u of 512 more than 1 count off", (unsigned)bad);
    CHECK(gamma_apply(GAMMA_LINEAR, 200) == 200u * 257u, "linear");
}
