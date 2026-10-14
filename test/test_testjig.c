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

    /* (0.6.7) the pads: TP6 low counts only while no byte arrives; TP9 always */
    {
        uint32_t rx = 1234, n6 = 0, n9 = 0, ntr = 0, nbr = 0, nun = 0;
        tj_init(&t, TJ_CYCLE_MS);
        for (ms = 0; ms < 100u; ms++) {
            nun += tj_pads(&t, 0, rx, 0);                /* unconnected: the pull-up */
        }
        for (ms = 0; ms < 1000u; ms++) {                 /* console traffic: a 0x00 stream reads low, bytes arriving */
            rx += 11u;
            ntr += tj_pads(&t, 1, rx, 0);
        }
        rx++;                                            /* a break: its one 0x00, then the line held low */
        nbr += tj_pads(&t, 1, rx, 0);
        for (ms = 0; ms < 100u; ms++) {
            n6 += tj_pads(&t, 1, rx, 0);
        }
        for (ms = 0; ms < 100u; ms++) {                  /* TP9 grounded, whatever the console does */
            rx += (ms & 1u);
            n9 += tj_pads(&t, 0, rx, 1);
        }
        CHECK(!nun && !ntr && !nbr && n6 == 100u && n9 == 100u, "pads: unconnected %u, traffic %u, the break's byte "
              "%u, a held break / grounded TP6 %u of 100, TP9 %u of 100", nun, ntr, nbr, n6, n9);
    }

    /* the pattern (0.6.7): all off for TJ_GAP_MS, then one channel at a time at the peak, R -> G -> B -> W1 -> W2, each
     * for its slot, at 1, 1.5 and 2 s */
    {
        static const uint32_t cyc_ms[3] = { 1000u, 1500u, 2000u };
        for (cyc = 0; cyc < 3u; cyc++) {
            uint32_t slot = (cyc_ms[cyc] - TJ_GAP_MS) / 5u;
            tj_init(&t, cyc_ms[cyc]);
            for (ms = 0; ms < 100u; ms++) {
                tj_pad(&t, ms, 1);
            }
            bad = 0;
            for (ms = t.t0; ms < t.t0 + 3u * cyc_ms[cyc]; ms++) {
                uint32_t ph = (ms - t.t0) % cyc_ms[cyc], want = ph < TJ_GAP_MS ? 5u : (ph - TJ_GAP_MS) / slot;
                tj_outputs(&t, ms, v);
                for (ch = 0; ch < 5u; ch++) {
                    bad += v[ch] != (ch == want ? TJ_PEAK : 0u);
                }
            }
            CHECK(!bad && slot == (cyc_ms[cyc] - 100u) / 5u, "%u ms cycle: %u ms dark, then each channel alone, in "
                  "order, %u ms at %u %% (bad %u)", cyc_ms[cyc], TJ_GAP_MS, slot, TJ_PEAK_PCT, bad);
        }
        tj_init(&t, 500u);
        CHECK(TJ_PEAK == 21627u + (t.cycle_ms - 1000u), "the level: %u %% = %u", TJ_PEAK_PCT, (unsigned)TJ_PEAK);
        CHECK(t.cycle_ms == 1000u, "cycle clamped to 1..2 s");
    }

    /* the current report: the board current over each slot's last TJ_MEAS_MS, minus the zero */
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

    /* D9 (0.6.7): on, off in each cycle's gap until the first verdict; then PASS steady, FAIL 5 Hz; the verdict:
     * each channel within TJ_PASS_MIN_MA .. TJ_PASS_MAX_MA */
    {
        static const int32_t ok[5] = { 136, 131, 398, 795, 50 }, bad1[5] = { 136, 131, 398, 795, 49 };
        static const int32_t hi[5] = { 3000, 3001, 1000, 1000, 1000 };
        const int32_t *cur = ok;
        uint32_t k;
        tj_init(&t, 1500u);
        for (ms = 0; ms < 50u; ms++) {
            tj_pad(&t, ms, 1);
        }
        bad = 0;
        for (ms = t.t0; ms < t.t0 + 1000u; ms++) {
            bad += tj_led(&t, ms) != ((ms - t.t0) % 1500u >= TJ_GAP_MS);
        }
        CHECK(!bad && !tj_led(&t, t.t0) && tj_led(&t, t.t0 + 100u), "D9 before the first verdict");
        for (k = 0; k < 3u; k++) {                       /* a cycle of each: PASS, FAIL (W2 49 mA), FAIL (G 3,001 mA) */
            cur = k == 0u ? ok : k == 1u ? bad1 : hi;
            for (ms = t.t0 + 1u + k * 1500u; ms <= t.t0 + (k + 1u) * 1500u; ms++) {
                int32_t i_ma = 0;
                tj_outputs(&t, ms - 1u, v);
                for (ch = 0; ch < 5u; ch++) {
                    i_ma += v[ch] ? cur[ch] : 0;
                }
                (void)tj_measure(&t, ms, i_ma, 0);
            }
            bad = 0;
            for (ms = t.t0 + (k + 1u) * 1500u + 1u; ms < t.t0 + (k + 1u) * 1500u + 400u; ms++) {
                bad += tj_led(&t, ms) != (k == 0u ? 1u : (ms / 100u) & 1u);
            }
            CHECK(t.cycles == k + 1u && t.fail == (k == 0u ? 0u : k == 1u ? 16u : 2u) && !bad,
                  "cycle %u: fail=%u, D9 %s", k + 1u, t.fail, k == 0u ? "steady" : "5 Hz");
        }
    }

    /* the console (0.6.7): the jig PC's line only, its format unchanged (0.6.5 D10): pass / fail, the currents, the
     * supply, the chip's id and the version */
    {
        cli_t cc;
        cli_status_t cst;
        static const uint32_t uid[3] = { 0x01234567u, 0x89ABCDEFu, 0xFEDCBA98u };
        static const int32_t ma[5] = { 136, 131, 398, 795, -3 };
        memset(&cc, 0, sizeof cc);
        memset(&cst, 0, sizeof cst);
        cst.uid = uid;
        cst.version = "0.6.5-beta.1";
        cc.st = &cst;
        cc.write = cap_write;
        cli_out[0] = 0;
        cli_test_report(&cc, ma, 16, 7, 24012);
        CHECK(strcmp(cli_out, "jig cycle=7 FAIL fail=16 a=0.14/0.13/0.40/0.80/0.00 supply=24.01 "
                              "uid=19088743,2309737967,4275878552 fw=0.6.5-beta.1\r\n") == 0,
              "the jig line: %s", cli_out);
        cli_out[0] = 0;
        cli_test_report(&cc, ma, 0, 1, 0);
        CHECK(strcmp(cli_out, "jig cycle=1 PASS fail=0 a=0.14/0.13/0.40/0.80/0.00 supply=0.00 "
                              "uid=19088743,2309737967,4275878552 fw=0.6.5-beta.1\r\n") == 0,
              "the jig line without the supply: %s", cli_out);
    }
}

/* the TP9 factory-reset gesture (0.6.0): runs ms [0, end) with the pad low in the given intervals; returns the resets
 * and the ms the first came (or ~0u) */
static uint32_t tjr_low[4][2];
static uint32_t tjr_n;
static int pad_iv(uint32_t ms)
{
    uint32_t i;
    for (i = 0; i < tjr_n; i++) {
        if (ms >= tjr_low[i][0] && ms < tjr_low[i][1]) {
            return 1;
        }
    }
    return 0;
}

static uint32_t tj_gesture(uint32_t end, uint32_t *at, uint32_t *armed_ms)
{
    testjig_t t;
    uint32_t ms, resets = 0;
    tj_init(&t, TJ_CYCLE_MS);
    *at = ~0u;
    *armed_ms = 0;
    for (ms = 0; ms < end; ms++) {
        uint8_t ev = tj_pad(&t, ms, (uint8_t)pad_iv(ms));
        if (ev & TJ_EV_RESET) {
            resets++;
            if (*at == ~0u) {
                *at = ms;
            }
        }
        if (t.rst >= TJ_RST_ARMED) {
            (*armed_ms)++;
            CHECK(tj_led(&t, ms) == ((ms / 50u) & 1u), "armed: D9 blinks at 10 Hz (ms %u)", ms);
        }
    }
    return resets;
}

static void test_tj_reset(void)
{
    static const struct { uint32_t n, iv[4][2], end, resets; const char *what; } G[] = {
        { 2, { { 10000, 16000 }, { 18000, 19200 } }, 30000, 1, "hold 6 s, release, 1.2 s again within 5 s: reset" },
        { 2, { { 10000, 14900 }, { 16000, 17200 } }, 30000, 0, "first hold 4.9 s: nothing" },
        { 2, { { 10000, 16000 }, { 21200, 22400 } }, 30000, 0, "the second hold after 5.2 s: nothing" },
        { 2, { { 10000, 16000 }, { 18000, 18900 } }, 30000, 0, "the second hold 0.9 s: nothing" },
        { 1, { { 0, 40000 } }, 60000, 0, "a jig grounding it from power-up for 40 s: never" },
        { 2, { { 0, 20000 }, { 21000, 23000 } }, 40000, 0, "a jig from power-up, released, grounded again: never" },
        { 1, { { 10000, 300000 } }, 310000, 0, "grounded on a running board and left there: never" },
        { 4, { { 10000, 16000 }, { 18000, 19200 }, { 30000, 36000 }, { 38000, 39500 } }, 50000, 2, "twice: two" },
    };
    uint32_t g, r, at, armed;

    printf("TP9 factory reset: the gesture, and what never triggers it\n");
    for (g = 0; g < sizeof G / sizeof G[0]; g++) {
        memcpy(tjr_low, G[g].iv, sizeof tjr_low);
        tjr_n = G[g].n;
        r = tj_gesture(G[g].end, &at, &armed);
        CHECK(r == G[g].resets, "%s (%u resets)", G[g].what, r);
        if (g == 0u) {
            /* armed from 5 s into the first hold (10049 + 5000) until the second hold's 1 s ends (18049 + 1000) */
            CHECK(at == 18049u + TJ_RST_CONFIRM_MS && armed == at - (10049u + TJ_RST_HOLD_MS),
                  "reset at %u ms, armed for %u ms", at, armed);
        }
    }
    /* 2 h of random noise on the pad after boot: never */
    {
        testjig_t t;
        uint32_t ms, resets = 0;
        tj_init(&t, TJ_CYCLE_MS);
        for (ms = 0; ms < 7200000u; ms++) {
            resets += (tj_pad(&t, ms, (uint8_t)(rnd() & 1u)) & TJ_EV_RESET) != 0u;
        }
        CHECK(resets == 0, "2 h of noise: %u resets", resets);
    }
}
