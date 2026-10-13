/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Dim-to-warm tests (included by test_main.c): the board's CW / WW mix follows M (LIGHT_OPT), red assist below the
 * WW LED, ESPHome's normalisations kept, smooth during board-run transitions. */

#define DTW_CW_K   6500.0
#define DTW_WW_K   2700.0
#define MIRED10(k) ((uint16_t)(1e7 / (k) + 0.5))

typedef struct {
    ultra_t u;
    uint32_t tick;
    uint8_t seq;
} dtw_board_t;

static const uint8_t DTW_RGBWW[5] = { ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE, ULTRA_ROLE_WARM_WHITE, ULTRA_ROLE_COLD_WHITE };

static void dtw_setup(dtw_board_t *b, uint8_t def_flags, int dtw, double warm_k, double assist)
{
    static const uint16_t mn[5] = { 0, 0, 0, 0, 0 }, mx[5] = { 65535, 65535, 65535, 65535, 65535 };
    uint8_t s[ULTRA_SLOT_BYTES];
    ultra_init(&b->u, 11);
    b->tick = 100000;
    b->seq = 1;
    ultra_on_header(&b->u, b->tick, 50000000u);
    ultra_slot_light_def(s, 0, b->seq, DTW_RGBWW, 2800, def_flags, mn, mx);
    ultra_on_slot(&b->u, s, b->tick);
    ultra_slot_light_opt(s, 0, b->seq, (uint8_t)(dtw ? ULTRA_OPT_DIM_TO_WARM : 0), MIRED10(DTW_CW_K), MIRED10(DTW_WW_K),
                         MIRED10(warm_k), (uint16_t)(assist * 65535.0 + 0.5));
    CHECK((ultra_on_slot(&b->u, s, b->tick) & (ULTRA_EV_LIGHT | ULTRA_EV_DEF)) == (ULTRA_EV_LIGHT | ULTRA_EV_DEF), "LIGHT_OPT event");
}

/* ESPHome's CT -> cold / warm (LightCall: fractions normalised to max 1, gamma-uncorrected), as A values */
static void dtw_ct(double kelvin, uint16_t *a)
{
    double mired = 1e6 / kelvin, cwm = 1e6 / DTW_CW_K, wwm = 1e6 / DTW_WW_K;
    double ww = (mired - cwm) / (wwm - cwm), cw = 1.0 - ww, m = ww > cw ? ww : cw;
    a[0] = a[1] = a[2] = 0;
    a[3] = (uint16_t)(pow(ww / m, 1.0 / 2.8) * 65535.0 + 0.5);   /* CH4 warm */
    a[4] = (uint16_t)(pow(cw / m, 1.0 / 2.8) * 65535.0 + 0.5);   /* CH5 cold */
}

static void dtw_set(dtw_board_t *b, uint16_t m, const uint16_t *a, uint32_t dur_ms, uint16_t *out)
{
    static const uint16_t one[5] = { 65535, 65535, 65535, 65535, 65535 };
    uint8_t s[ULTRA_SLOT_BYTES];
    ultra_slot_set(s, 0, ++b->seq, dur_ms ? ULTRA_SET_SMOOTH : 0, m, a, one, dur_ms, 0);
    b->u.frame_ms += 0;                                    /* SET starts at the header's time */
    ultra_on_slot(&b->u, s, b->tick);
    if (out) {
        b->tick += 20;
        ultra_render(&b->u, b->tick, out);
    }
}

/* effective colour temperature of the outputs (linear CW / WW shares), Kelvin */
static double dtw_kelvin(const uint16_t *o)
{
    double ww = o[3], cw = o[4], cwm = 1e6 / DTW_CW_K, wwm = 1e6 / DTW_WW_K;
    if (ww + cw <= 0) return 0;
    return 1e6 / (cwm + (wwm - cwm) * ww / (ww + cw));
}

static void test_dtw(void)
{
    dtw_board_t on, off;
    uint16_t a[5], o1[5], o2[5];
    double ks[] = { 6500, 5000, 4000, 3000, 2700 };
    int i, k, worst = 0, cb, mono = 1, total_bad = 0, assist_bad = 0, steps_max = 0;
    double k_lo = 0;

    printf("dim to warm: continuity at full brightness, mix vs brightness, red assist, smooth transitions\n");
    /* 1. at full brightness dim-to-warm changes nothing (both normalisations, 5 colour temperatures) */
    for (cb = 0; cb < 2; cb++) {
        uint8_t fl = cb ? ULTRA_DEF_CONSTANT_BRIGHTNESS : 0;
        for (k = 0; k < 5; k++) {
            dtw_setup(&on, fl, 1, 2000, 0.3);
            dtw_setup(&off, fl, 0, 2000, 0.3);
            dtw_ct(ks[k], a);
            dtw_set(&on, 65535, a, 0, o1);
            dtw_set(&off, 65535, a, 0, o2);
            for (i = 0; i < 5; i++) worst = abs((int)o1[i] - (int)o2[i]) > worst ? abs((int)o1[i] - (int)o2[i]) : worst;
        }
    }
    CHECK(worst <= 2, "full brightness: same outputs as without dim to warm (max %d counts)", worst);

    /* 2. brightness sweep at 4000 K, warm end 2200 K, no red assist: warmer as it dims, CW + WW kept (cb) */
    dtw_setup(&on, ULTRA_DEF_CONSTANT_BRIGHTNESS, 1, 2200, 0.0);
    dtw_setup(&off, ULTRA_DEF_CONSTANT_BRIGHTNESS, 0, 2200, 0.0);
    dtw_ct(4000, a);
    {
        double prev = 1e9;
        uint32_t m;
        for (m = 65535; m >= 2000; m -= 1000) {
            double kk;
            dtw_set(&on, (uint16_t)m, a, 0, o1);
            dtw_set(&off, (uint16_t)m, a, 0, o2);
            kk = dtw_kelvin(o1);
            if (kk > prev + 0.5) mono = 0;
            prev = kk;
            /* constant brightness: CW + WW equals the light's total without dim to warm */
            if (abs((int)(o1[3] + o1[4]) - (int)(o2[3] + o2[4])) > 3) total_bad++;
            if (o1[0] || o1[1] || o1[2]) assist_bad++;                         /* no assist configured */
        }
        k_lo = prev;
    }
    CHECK(mono, "colour temperature falls monotonically as the light dims");
    CHECK(total_bad == 0, "constant brightness: CW + WW unchanged by dim to warm (%d off)", total_bad);
    CHECK(assist_bad == 0, "no red without red_assist");
    CHECK(k_lo < 2800 && k_lo >= 2690, "at 3 %% brightness: at the WW LED (%.0f K, the warm end 2200 K needs red)", k_lo);
    printf("  4000 K at full brightness -> %.0f K at 3 %% (WW LED 2700 K), CW + WW constant\n", k_lo);

    /* 3. warm end within the LED range: 4000 K -> 3000 K, half brightness = half way in mireds */
    dtw_setup(&on, 0, 1, 3000, 0.0);
    dtw_ct(4000, a);
    dtw_set(&on, 32768, a, 0, o1);
    {
        double kk = dtw_kelvin(o1), want = 1e6 / ((1e6 / 4000 + 1e6 / 3000) / 2);
        CHECK(fabs(kk - want) < 15, "M = 50 %%: half way in mireds (%.0f K, expected %.0f K)", kk, want);
        CHECK(o1[3] > o1[4] ? o1[3] : o1[4], "brighter channel set");
    }

    /* 4. red assist below the WW LED: R grows as it dims, G = R / 4, nothing above 2700 K; off with RGB on */
    dtw_setup(&on, ULTRA_DEF_CONSTANT_BRIGHTNESS, 1, 1800, 0.3);
    dtw_ct(2700, a);
    {
        uint32_t m, g_ok = 1, red_at_top;
        double worst_r = 0, at18 = 0;
        dtw_set(&on, 65535, a, 0, o1);
        red_at_top = o1[0];
        for (m = 60000; m >= 1000; m -= 1000) {
            double t = 1.0 - m / 65535.0, want = 0.3 * t, got;   /* 2700 K: past the WW LED as soon as it dims */
            dtw_set(&on, (uint16_t)m, a, 0, o1);
            got = o1[3] ? (double)o1[0] / o1[3] : 0;
            if (o1[3] >= 300 && fabs(got - want) > worst_r) worst_r = fabs(got - want);   /* above quantisation */
            if (m == 12000) at18 = got;
            if (abs((int)o1[1] - (int)(o1[0] / 4)) > 1 || o1[2] != 0) g_ok = 0;
        }
        CHECK(red_at_top == 0, "no red at full brightness");
        CHECK(worst_r < 0.005, "red / WW = assist x share of the way past the WW LED (max error %.4f)", worst_r);
        CHECK(g_ok, "green = red / 4, blue 0");
        printf("  red assist 30 %%, warm end 1800 K: red / WW at 18 %% brightness = %.1f %% (expected %.1f %%), "
               "green = red / 4\n", at18 * 100, 30.0 * (1 - 12000 / 65535.0));
        /* RGB part on (rgbww without interlock): no assist */
        a[0] = 65535; a[1] = 20000;
        dtw_set(&on, 5000, a, 0, o1);
        dtw_setup(&off, ULTRA_DEF_CONSTANT_BRIGHTNESS, 0, 1800, 0.3);
        dtw_set(&off, 5000, a, 0, o2);
        CHECK(o1[0] == o2[0] && o1[1] == o2[1], "RGB part on: its channels untouched (%u vs %u)", o1[0], o2[0]);
    }

    /* 5. a board-run transition (10 s, 100 % -> 3 %) stays smooth with dim to warm: largest step per 1 ms update */
    dtw_setup(&on, ULTRA_DEF_CONSTANT_BRIGHTNESS, 1, 1800, 0.3);
    dtw_ct(4000, a);
    dtw_set(&on, 65535, a, 0, o1);
    dtw_set(&on, 2000, a, 10000, NULL);
    {
        uint32_t t;
        uint16_t prev[5];
        ultra_render(&on.u, on.tick, prev);
        for (t = 1; t <= 10200; t++) {
            ultra_render(&on.u, on.tick + t * 20u, o1);
            for (i = 0; i < 5; i++) {
                int d = abs((int)o1[i] - (int)prev[i]);
                steps_max = d > steps_max ? d : steps_max;
            }
            memcpy(prev, o1, sizeof prev);
        }
        printf("  10 s transition 100 %% -> 3 %% at 4000 K, warm end 1800 K, red assist: largest step %d counts "
               "(%.3f %%) per 1 ms update\n", steps_max, steps_max * 100.0 / 65535);
        CHECK(steps_max < 80, "smooth: largest step per 1 ms update %d counts", steps_max);
    }
    /* 6. off is off; the channel limit holds on the red assist */
    dtw_set(&on, 0, a, 0, o1);
    CHECK(!o1[0] && !o1[3] && !o1[4], "off");
    {
        uint8_t s[ULTRA_SLOT_BYTES], ids[4] = { ULTRA_CFG_CH_CURRENT, ULTRA_CFG_CH_CURRENT + 3, ULTRA_CFG_CH_CURRENT + 4, ULTRA_CFG_CH_MAX };
        uint16_t vals[4] = { 3000, 5000, 5000, 100 };
        uint32_t ma;
        dtw_ct(2700, a);
        ultra_slot_config(s, 9, ids, vals, 4);
        ultra_on_slot(&on.u, s, on.tick);
        dtw_set(&on, 20000, a, 0, o1);
        ma = o1[0] * 3000u / 65535u;
        CHECK(o1[0] != 0 && ma <= 100, "channel limit 100 mA holds on the red assist (%u mA)", ma);
    }

    /* 7. set points (ULTRA_OPT_POINTS), sent out of order: 100 % 4000 K, 30 % 3000 K, 5 % 2700 K */
    {
        static const uint16_t mn[5] = { 0, 0, 0, 0, 0 }, mx[5] = { 65535, 65535, 65535, 65535, 65535 };
        static const struct { double m, ha_k, want_k; const char *what; } pc[] = {
            { 1.00, 5000, 4000, "100 % point" }, { 0.65, 5000, 1e6 / (250 + (1e6 / 3000 - 250) * 0.5), "between 100 % and 30 %" },
            { 0.30, 5000, 3000, "30 % point" }, { 0.10, 5000, 1e6 / (1e6 / 3000 + (370.37 - 1e6 / 3000) * 0.8), "between 30 % and 5 %" },
            { 0.02, 5000, 2700, "below the lowest point: held" }, { 1.00, 2800, 2800, "never colder than the light's own (2800 K)" },
        };
        uint16_t pm[3], pt[3];
        uint8_t s[ULTRA_SLOT_BYTES];
        int pi, bad = 0;
        pm[0] = 19661; pt[0] = MIRED10(3000);             /* 30 % */
        pm[1] = 65535; pt[1] = MIRED10(4000);             /* 100 % */
        pm[2] = 3277;  pt[2] = MIRED10(2700);             /* 5 % */
        for (pi = 0; pi < 6; pi++) {
            dtw_board_t pb;
            double kk;
            ultra_init(&pb.u, 11);
            pb.tick = 100000;
            pb.seq = 1;
            ultra_on_header(&pb.u, pb.tick, 50000000u);
            ultra_slot_light_def(s, 0, pb.seq, DTW_RGBWW, 2800, ULTRA_DEF_CONSTANT_BRIGHTNESS, mn, mx);
            ultra_on_slot(&pb.u, s, pb.tick);
            ultra_slot_light_opt_points(s, 0, pb.seq, ULTRA_OPT_DIM_TO_WARM | ULTRA_OPT_POINTS, MIRED10(DTW_CW_K),
                                        MIRED10(DTW_WW_K), MIRED10(2700), 0, 3, pm, pt);
            ultra_on_slot(&pb.u, s, pb.tick);
            dtw_ct(pc[pi].ha_k, a);
            dtw_set(&pb, (uint16_t)(pc[pi].m * 65535 + 0.5), a, 0, o1);
            kk = dtw_kelvin(o1);
            if (fabs(kk - pc[pi].want_k) > 20) {
                bad++;
                printf("  set points: %s: %.0f K, expected %.0f K\n", pc[pi].what, kk, pc[pi].want_k);
            }
        }
        CHECK(bad == 0, "set points: the curve at, between and below the points, never colder than the light's own (%d)", bad);
        /* without a 100 % point the light's own temperature (5000 K) is the top: at 65 %, half way to the 30 % point */
        {
            dtw_board_t pb;
            double kk, want = 1e6 / (200 + (1e6 / 3000 - 200) * 0.5);
            ultra_init(&pb.u, 11);
            pb.tick = 100000;
            pb.seq = 1;
            ultra_on_header(&pb.u, pb.tick, 50000000u);
            ultra_slot_light_def(s, 0, pb.seq, DTW_RGBWW, 2800, ULTRA_DEF_CONSTANT_BRIGHTNESS, mn, mx);
            ultra_on_slot(&pb.u, s, pb.tick);
            ultra_slot_light_opt_points(s, 0, pb.seq, ULTRA_OPT_DIM_TO_WARM | ULTRA_OPT_POINTS, MIRED10(DTW_CW_K),
                                        MIRED10(DTW_WW_K), MIRED10(2700), 0, 2, pm, pt);   /* 30 %, 100 % ... */
            pm[1] = 3277; pt[1] = MIRED10(2700);          /* ... make it 30 % and 5 % */
            ultra_slot_light_opt_points(s, 0, ++pb.seq, ULTRA_OPT_DIM_TO_WARM | ULTRA_OPT_POINTS, MIRED10(DTW_CW_K),
                                        MIRED10(DTW_WW_K), MIRED10(2700), 0, 2, pm, pt);
            ultra_on_slot(&pb.u, s, pb.tick);
            dtw_ct(5000, a);
            dtw_set(&pb, (uint16_t)(0.65 * 65535 + 0.5), a, 0, o1);
            kk = dtw_kelvin(o1);
            CHECK(fabs(kk - want) < 20, "no 100 %% point: the light's own temperature on top (%.0f K at 65 %%, expected %.0f K)", kk, want);
            dtw_set(&pb, 65535, a, 0, o1);
            CHECK(fabs(dtw_kelvin(o1) - 5000) < 20, "no 100 %% point: full brightness = the light's own (%.0f K)", dtw_kelvin(o1));
        }
        /* red assist below the WW LED: points 100 % 2700 K, 50 % 2000 K, assist 30 %: red = 30 % of WW at 50 % and below
         * (checked at 40 %: enough counts for the ratio) */
        {
            dtw_board_t pb;
            uint16_t qm[2] = { 65535, 32768 }, qt[2];
            qt[0] = MIRED10(2700);
            qt[1] = MIRED10(2000);
            ultra_init(&pb.u, 11);
            pb.tick = 100000;
            pb.seq = 1;
            ultra_on_header(&pb.u, pb.tick, 50000000u);
            ultra_slot_light_def(s, 0, pb.seq, DTW_RGBWW, 2800, ULTRA_DEF_CONSTANT_BRIGHTNESS, mn, mx);
            ultra_on_slot(&pb.u, s, pb.tick);
            ultra_slot_light_opt_points(s, 0, pb.seq, ULTRA_OPT_DIM_TO_WARM | ULTRA_OPT_POINTS, MIRED10(DTW_CW_K),
                                        MIRED10(DTW_WW_K), MIRED10(2000), 19661, 2, qm, qt);
            ultra_on_slot(&pb.u, s, pb.tick);
            dtw_ct(2700, a);
            dtw_set(&pb, 26214, a, 0, o1);
            CHECK(o1[3] > 0 && fabs((double)o1[0] / o1[3] - 0.3) < 0.01 && o1[4] == 0,
                  "set points warmer than WW: red = 30 %% of WW (%.3f), CW off", o1[3] ? (double)o1[0] / o1[3] : 0.0);
            dtw_set(&pb, 65535, a, 0, o1);
            CHECK(o1[0] == 0, "no red at the 100 %% point (2700 K = the WW LED)");
        }
        printf("  set points: 100 %% 4000 K / 30 %% 3000 K / 5 %% 2700 K, interpolated in mireds, held below, never colder "
               "than the light's own; without a 100 %% point the light's own temperature on top; red assist past WW\n");
    }
}
