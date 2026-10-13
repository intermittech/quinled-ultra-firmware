/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Test jig mode (core/testjig.c, included by test_main.c after test_features.c): the pad's debounce (boot, noise, a
 * floating pad, release), the pattern's timing and shape at 1 / 1.5 / 2 s, the current report and its console line,
 * D9's pattern, and the return to normal operation. */
#include "../src/core/testjig.h"

/* runs the pad for ms [t0, t1) with pad(t); returns the first ms test mode was on (or ~0u) */
static uint32_t tj_run(testjig_t *t, uint32_t t0, uint32_t t1, int (*pad)(uint32_t), uint32_t *changes)
{
    uint32_t ms, first = ~0u;
    for (ms = t0; ms != t1; ms++) {
        *changes += tj_pad(t, ms, (uint8_t)pad(ms));
        if (t->active && first == ~0u) {
            first = ms;
        }
    }
    return first;
}

static int pad_high(uint32_t ms) { (void)ms; return 0; }
static int pad_low(uint32_t ms) { (void)ms; return 1; }
static int pad_low49(uint32_t ms) { return ms < 49u; }
static int pad_bouncy(uint32_t ms) { return (ms % 40u) != 39u; }                /* low, a 1 ms high every 40 ms */
static int pad_noise(uint32_t ms) { (void)ms; return (int)(rnd() & 1u); }       /* a floating pad picking up noise */

static void test_testjig(void)
{
    testjig_t t;
    uint32_t ch, first, ms, cyc, bad;
    uint16_t v[5];
    int c;

    printf("test jig mode: debounce, pattern timing, current report, D9, release\n");

    /* debounce */
    tj_init(&t, TJ_CYCLE_MS);
    ch = 0;
    first = tj_run(&t, 0, 600000u, pad_high, &ch);
    CHECK(first == ~0u && ch == 0, "a floating pad (pull-up, high): never");
    tj_init(&t, TJ_CYCLE_MS);
    ch = 0;
    first = tj_run(&t, 0, 60u, pad_low49, &ch);
    CHECK(first == ~0u, "low for 49 ms: not yet");
    tj_init(&t, TJ_CYCLE_MS);
    ch = 0;
    first = tj_run(&t, 0, 100u, pad_low, &ch);
    CHECK(first == TJ_ENTER_MS - 1u && ch == 1 && t.t0 == first, "grounded at boot: on after 50 ms (ms %u)", first);
    tj_init(&t, TJ_CYCLE_MS);
    ch = 0;
    first = tj_run(&t, 0, 60000u, pad_bouncy, &ch);
    CHECK(first == ~0u, "a bouncing contact (high 1 ms in 40): never");
    for (c = 0; c < 20; c++) {
        tj_init(&t, TJ_CYCLE_MS);
        ch = 0;
        first = tj_run(&t, 0, 600000u, pad_noise, &ch);
        if (first != ~0u) {
            break;
        }
    }
    CHECK(c == 20, "random noise on the pad, 20 x 10 min: never");
    /* grounded later (a board powered first, then put in the jig), then released */
    tj_init(&t, TJ_CYCLE_MS);
    ch = 0;
    tj_run(&t, 0, 5000u, pad_high, &ch);
    first = tj_run(&t, 5000u, 10000u, pad_low, &ch);
    CHECK(first == 5000u + TJ_ENTER_MS - 1u && t.active, "grounded at 5 s: on 50 ms later");
    for (ms = 10000u; ms < 10000u + TJ_LEAVE_MS - 1u; ms++) {
        ch += tj_pad(&t, ms, 0);
    }
    CHECK(t.active, "released for 49 ms: still on");
    ch += tj_pad(&t, ms, 0);
    CHECK(!t.active && ch == 2, "released for 50 ms: back to normal operation (%u changes)", ch);
    for (ms = 10050u; ms < 10080u; ms++) {
        ch += tj_pad(&t, ms, (uint8_t)(ms % 3u == 0u));
    }
    CHECK(!t.active && ch == 2, "a short bounce on release: stays off");

    /* the pattern: one channel at a time, R -> G -> B -> W1 -> W2, up / hold / down, at 1, 1.5 and 2 s */
    {
        static const uint32_t cyc_ms[3] = { 1000u, 1500u, 2000u };
        for (cyc = 0; cyc < 3u; cyc++) {
            uint32_t slot = cyc_ms[cyc] / 5u, ramp = (slot - TJ_HOLD_MS) / 2u;
            uint32_t full_ms[5] = { 0 }, lit_first[5], lit_last[5], peak_at[5];
            uint16_t prev[5] = { 0 };
            uint8_t up_bad = 0, two = 0;
            for (ch = 0; ch < 5u; ch++) {
                lit_first[ch] = ~0u;
                lit_last[ch] = peak_at[ch] = 0;
            }
            tj_init(&t, cyc_ms[cyc]);
            for (ms = 0; ms < 100u; ms++) {
                tj_pad(&t, ms, 1);
            }
            for (ms = t.t0; ms < t.t0 + 3u * cyc_ms[cyc]; ms++) {
                uint32_t ph = (ms - t.t0) % cyc_ms[cyc], n = 0;
                tj_outputs(&t, ms, v);
                for (ch = 0; ch < 5u; ch++) {
                    if (v[ch] != 0u) {
                        n++;
                        if (ms < t.t0 + cyc_ms[cyc]) {
                            if (lit_first[ch] == ~0u) lit_first[ch] = ph;
                            lit_last[ch] = ph;
                        }
                    }
                    if (v[ch] == TJ_PEAK && ms < t.t0 + cyc_ms[cyc]) {
                        if (full_ms[ch] == 0u) peak_at[ch] = ph;
                        full_ms[ch]++;
                    }
                    /* rising before the hold, falling after it */
                    if (ph >= ch * slot && ph < ch * slot + ramp && ms > t.t0 && v[ch] < prev[ch]) up_bad = 1;
                    if (ph > ch * slot + ramp + TJ_HOLD_MS && ph < (ch + 1u) * slot && v[ch] > prev[ch]) up_bad = 1;
                    prev[ch] = v[ch];
                }
                two |= (uint8_t)(n > 1u);
            }
            bad = 0;
            for (ch = 0; ch < 5u; ch++) {
                bad += full_ms[ch] != TJ_HOLD_MS || peak_at[ch] != ch * slot + ramp ||
                       lit_first[ch] < ch * slot + 1u || lit_first[ch] > ch * slot + 2u || lit_last[ch] + 1u < (ch + 1u) * slot - 2u;
            }
            CHECK(!bad && !two && !up_bad, "%u ms cycle: each channel alone, in order, %u ms up, %u ms at %u %%, "
                  "down (bad %u, overlap %u, not monotonic %u)", cyc_ms[cyc], ramp, TJ_HOLD_MS, TJ_PEAK_PCT, bad, two,
                  up_bad);
        }
        /* the ramp's shape: square law, half way in time = a quarter */
        tj_init(&t, 1500u);
        for (ms = 0; ms < 50u; ms++) {
            tj_pad(&t, ms, 1);
        }
        tj_outputs(&t, t.t0 + t.ramp_ms / 2u, v);
        CHECK(v[0] > TJ_PEAK * 23u / 100u && v[0] < TJ_PEAK * 27u / 100u, "half way up: %u (a quarter of the peak, "
              "gamma 2)", v[0]);
        bad = 0;
        for (ms = t.t0; ms < t.t0 + 1500u; ms++) {
            tj_outputs(&t, ms, v);
            for (ch = 0; ch < 5u; ch++) {
                bad += v[ch] > TJ_PEAK;
            }
        }
        CHECK(!bad && TJ_PEAK == 21627u, "never above the peak: %u %% = %u", TJ_PEAK_PCT, (unsigned)TJ_PEAK);
        tj_init(&t, 500u);
        CHECK(t.cycle_ms == 1000u, "cycle clamped to 1..2 s");
    }

    /* the current report: the board current over the end of each hold, minus the zero */
    {
        static const int32_t want[5] = { 412, 398, 1205, 2410, 4990 };
        uint32_t reports = 0;
        tj_init(&t, 1500u);
        for (ms = 0; ms < 50u; ms++) {
            tj_pad(&t, ms, 1);
        }
        for (ms = t.t0 + 1u; ms < t.t0 + 2u * 1500u + 1u; ms++) {
            int32_t i_ma = 38;                         /* the zero: logic + offset */
            tj_outputs(&t, ms - 1u, v);                /* the last ms's outputs, as main.c measures them */
            for (ch = 0; ch < 5u; ch++) {
                i_ma += (int32_t)((int64_t)want[ch] * v[ch] / 65535);
            }
            i_ma += (int32_t)(rnd() % 21u) - 10;       /* +-10 mA of noise */
            if (tj_measure(&t, ms, i_ma, 38)) {
                reports++;
                bad = 0;
                for (ch = 0; ch < 5u; ch++) {
                    int32_t at = (int32_t)((int64_t)want[ch] * TJ_PEAK / 65535);   /* the current at the peak */
                    bad += t.ma[ch] < at - 6 || t.ma[ch] > at + 6;
                }
                CHECK(!bad, "cycle %u: %d / %d / %d / %d / %d mA", t.cycles, t.ma[0], t.ma[1], t.ma[2], t.ma[3],
                      t.ma[4]);
            }
        }
        CHECK(reports == 2 && t.cycles == 2, "one report per cycle (%u)", reports);
        /* a late main loop skipping ms: the windows still close */
        tj_init(&t, 1500u);
        for (ms = 0; ms < 50u; ms++) {
            tj_pad(&t, ms, 1);
        }
        reports = 0;
        for (ms = t.t0 + 1u; ms < t.t0 + 3u * 1500u; ms += 7u) {
            reports += tj_measure(&t, ms, 1000, 0);
        }
        CHECK(reports >= 2 && t.ma[4] == 1000, "every 7th ms only: reports %u", reports);
    }

    /* D9: on, off for the first 100 ms of each cycle */
    tj_init(&t, 1500u);
    for (ms = 0; ms < 50u; ms++) {
        tj_pad(&t, ms, 1);
    }
    bad = 0;
    for (ms = t.t0; ms < t.t0 + 3000u; ms++) {
        bad += tj_led(&t, ms) != ((ms - t.t0) % 1500u >= TJ_LED_GAP_MS);
    }
    CHECK(!bad && !tj_led(&t, t.t0) && tj_led(&t, t.t0 + 100u), "D9 in test mode");

    /* the console line */
    {
        cli_t cc;
        static const int32_t ma[5] = { 136, 131, 398, 795, -3 };
        memset(&cc, 0, sizeof cc);
        cc.write = cap_write;
        cli_out[0] = 0;
        cli_test_report(&cc, ma, 33, 7, 24012);
        CHECK(strcmp(cli_out, "test @33%: R 136 mA, G 131 mA, B 398 mA, W1 795 mA, W2 0 mA; "
                              "est. @100%: R 412, G 397, B 1206, W1 2409, W2 0 mA; supply 24012 mV (cycle 7)\r\n") == 0,
              "report line: %s", cli_out);
        cli_out[0] = 0;
        cli_test_report(&cc, ma, 33, 1, 0);
        CHECK(strstr(cli_out, "supply n/a (cycle 1)") != NULL, "report line without the supply");
    }
}
