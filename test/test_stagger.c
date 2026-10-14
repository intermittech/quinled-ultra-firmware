/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Adaptive stagger tests (included by test_main.c): ripple formula vs a sampled waveform, the search vs the
 * fixed stagger (example mix and 1500 random mixes, the numbers of tools/review/stagger_ripple.py), the edge
 * rule, hysteresis, and the phase slew at every PWM preset. */
#include "../src/core/stagger.h"

static const uint16_t PEAK_MA[5] = { 3000, 3000, 3000, 5000, 5000 };   /* R, G, B, W1, W2 */

/* sampled AC rms of the summed current, same layout as stagger.h */
static double sampled_ripple(const uint16_t *d, uint16_t o2, uint16_t o3)
{
    static double i_t[4096];
    double sum = 0, sq = 0, start[5], w[5];
    int n, k;
    for (k = 0; k < 5; k++) w[k] = d[k] / 65535.0;
    start[0] = 0; start[1] = 1 - w[1]; start[2] = o2 / 65536.0; start[3] = o3 / 65536.0; start[4] = o3 / 65536.0 - w[4];
    for (n = 0; n < 4096; n++) {
        double t = (n + 0.5) / 4096, v = 0;
        for (k = 0; k < 5; k++) {
            double ph = fmod(t - start[k] + 2.0, 1.0);
            if (ph < w[k]) v += PEAK_MA[k] / 1000.0;
        }
        i_t[n] = v;
        sum += v;
        sq += v * v;
    }
    return sqrt(sq / 4096 - (sum / 4096) * (sum / 4096));
}

/* own generator, seeded per run: the mixes don't depend on the tests before this one */
static unsigned long long st_rs = 1;
static double st_urand(double a, double b)
{
    st_rs ^= st_rs << 13; st_rs ^= st_rs >> 7; st_rs ^= st_rs << 17;
    return a + (b - a) * (double)(st_rs >> 11) / 9007199254740992.0;
}
static void st_seed(unsigned long long seed) { st_rs = 0x9E3779B97F4A7C15ULL ^ (seed * 0xBF58476D1CE4E5B9ULL); if (!st_rs) st_rs = 1; }

static void random_mix(uint16_t *d)
{
    double v[5], avg = 0;
    int k;
    for (k = 0; k < 5; k++) {
        v[k] = st_urand(0, 1) * (st_urand(0, 1) < 0.75 ? 1 : 0);
        avg += v[k] * PEAK_MA[k] / 1000.0;
    }
    for (k = 0; k < 5; k++) {
        if (avg > 8) v[k] *= 8 / avg;
        d[k] = (uint16_t)(v[k] * 65535 + 0.5);
    }
}

static int cmp_u32(const void *a, const void *b)
{
    uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
    return x < y ? -1 : x > y;
}

typedef struct {
    int worse_valid, worse_edge, broken, moved;
    uint32_t worst_ma;
    double worst_ratio;
    uint16_t worst_d[5], broken_d[5], broken_o2, broken_o3;
} st_mix_t;

/* n random mixes from the fixed offsets: adaptive worse than fixed while fixed keeps the edge rule (a search miss)
 * or because fixed breaks it (the documented move to a valid position <= 10 % worse); edge rule broken after a move */
static void st_mixes(int n, st_mix_t *m, uint32_t *fx, uint32_t *ad, uint32_t *ab)
{
    uint16_t d[5], gap = stagger_gap_q16(5760);
    stagger_result_t r;
    int k, i;
    memset(m, 0, sizeof *m);
    for (k = 0; k < n; k++) {
        uint32_t a;
        random_mix(d);
        stagger_choose(d, PEAK_MA, gap, STAGGER_FIXED_O2, STAGGER_FIXED_O3, &r);
        a = r.move ? r.ripple_best : r.ripple_now;
        if (fx) { fx[k] = r.ripple_now; ad[k] = a; ab[k] = r.ripple_best < r.ripple_now ? r.ripple_best : r.ripple_now; }
        if (a > r.ripple_now) {
            if (r.valid_now) m->worse_valid++; else m->worse_edge++;
            if (a - r.ripple_now > m->worst_ma) {
                m->worst_ma = a - r.ripple_now;
                m->worst_ratio = (double)a / r.ripple_now;
                for (i = 0; i < 5; i++) m->worst_d[i] = d[i];
            }
        }
        if (r.move) {
            m->moved++;
            if (!stagger_valid(d, r.o2, r.o3, gap)) {
                m->broken++;
                for (i = 0; i < 5; i++) m->broken_d[i] = d[i];
                m->broken_o2 = r.o2;
                m->broken_o3 = r.o3;
            }
        }
    }
}

static void test_stagger_sweep(int seeds);
static void test_stagger_cap(void);

static void test_stagger(void)
{
    static uint32_t fx[1500], ad[1500], ab[1500];
    uint16_t d[5];
    stagger_result_t r;
    double maxe = 0;
    int k, worse = 0, broken = 0, moved = 0;
    uint16_t gap = stagger_gap_q16(5760);   /* 25 kHz */

    printf("stagger: adaptive TIM2 / TIM3 offsets (LED_+ ripple)\n");
    if (getenv("TEST_SWEEP")) test_stagger_sweep(atoi(getenv("TEST_SWEEP")) > 0 ? atoi(getenv("TEST_SWEEP")) : 200);
    /* analytic ripple vs sampled */
    for (k = 0; k < 300; k++) {
        uint16_t o2 = (uint16_t)rnd(), o3 = (uint16_t)rnd();
        double e;
        random_mix(d);
        e = fabs(stagger_ripple_ma(d, PEAK_MA, o2, o3) / 1000.0 - sampled_ripple(d, o2, o3));
        if (e > maxe) maxe = e;
    }
    printf("  ripple formula vs 4096-point sampled waveform: max difference %.3f A\n", maxe);
    CHECK(maxe < 0.01, "ripple formula %.3f A off", maxe);

    /* the example from the request: R 83 %, G 70 %, B 42 %, W1 9 %, W2 34 % */
    {
        static const double ex[5] = { 0.83, 0.70, 0.42, 0.09, 0.34 };
        for (k = 0; k < 5; k++) d[k] = (uint16_t)(ex[k] * 65535 + 0.5);
        stagger_choose(d, PEAK_MA, gap, STAGGER_FIXED_O2, STAGGER_FIXED_O3, &r);
        printf("  example mix: fixed %.2f A rms -> %.2f A rms at TIM2 %.3f / TIM3 %.3f T, move %d\n",
               r.ripple_now / 1000.0, r.ripple_best / 1000.0, r.o2 / 65536.0, r.o3 / 65536.0, r.move);
        CHECK(r.move && r.ripple_best < 1200 && r.ripple_now > 5100, "example: %u -> %u mA", r.ripple_now, r.ripple_best);
        CHECK(stagger_valid(d, r.o2, r.o3, gap), "example result keeps the edge rule");
        stagger_choose(d, PEAK_MA, gap, r.o2, r.o3, &r);
        CHECK(!r.move, "no further move from the optimum");
    }

    /* random mixes, average <= 8 A, four fixed seeds. Adaptive is never worse than fixed while the fixed offsets keep
     * the edge rule; where they break it, the board moves to a valid position up to 10 % worse (by design, stagger.h).
     * Seeds 17, 91 and 870 hit the grid off-by-one (fixed 2026-10-12): a move to offsets that broke the edge rule. */
    {
        static const unsigned long long seeds[] = { 1, 17, 91, 870 };
        st_mix_t m;
        int sk, we = 0;
        double ratio = 0;
        for (sk = 0; sk < 4; sk++) {
            st_seed(seeds[sk]);
            if (sk == 0) st_mixes(1500, &m, fx, ad, ab); else st_mixes(1500, &m, NULL, NULL, NULL);
            worse += m.worse_valid;
            we += m.worse_edge;
            broken += m.broken;
            moved += m.moved;
            if (m.worse_edge && m.worst_ratio > ratio) ratio = m.worst_ratio;
        }
        printf("  4 seeds x 1500 mixes: worse than fixed %d with fixed valid, %d where fixed breaks the edge rule (at most "
               "x%.3f, allowed x1.10)\n", worse, we, ratio);
        CHECK(ratio <= 1.10, "a move off invalid fixed offsets is at most 10 %% worse (x%.3f)", ratio);
    }
    /* the three mixes of the off-by-one: the chosen offsets keep the edge rule */
    {
        static const uint16_t reg[3][5] = { { 38288, 0, 14472, 4054, 62841 }, { 0, 38942, 55431, 18565, 1470 },
                                            { 12112, 14361, 38060, 43473, 22664 } };
        for (k = 0; k < 3; k++) {
            stagger_choose(reg[k], PEAK_MA, gap, STAGGER_FIXED_O2, STAGGER_FIXED_O3, &r);
            CHECK(!r.move || stagger_valid(reg[k], r.o2, r.o3, gap), "grid off-by-one mix %d: offsets keep the edge rule", k);
        }
    }
    qsort(fx, 1500, sizeof fx[0], cmp_u32);
    qsort(ad, 1500, sizeof ad[0], cmp_u32);
    qsort(ab, 1500, sizeof ab[0], cmp_u32);
    printf("  1500 random mixes (seed 1, avg <= 8 A, 25 kHz): worst %.2f -> %.2f A rms (best offsets %.2f), median %.2f -> "
           "%.2f (%.2f); moved %d (4 seeds), worse %d, edge rule broken %d\n", fx[1499] / 1000.0, ad[1499] / 1000.0,
           ab[1499] / 1000.0, fx[750] / 1000.0, ad[750] / 1000.0, ab[750] / 1000.0, moved, worse, broken);
    CHECK(worse == 0 && broken == 0, "adaptive never worse while fixed is valid (%d), edge rule kept (%d)", worse, broken);
    CHECK(ab[1499] < 2600 && ad[1499] < 2900 && ad[750] < 1600, "adaptive statistics");

    /* edge rule: two turn-ons closer than the gap are rejected, a turn-on next to a turn-off is fine */
    {
        uint16_t dd[5] = { 20000, 0, 20000, 0, 0 };      /* R [0, 0.3), B [o2, o2 + 0.3) */
        CHECK(!stagger_valid(dd, 100, 0, gap), "B on 100/65536 after R on: rejected");
        CHECK(stagger_valid(dd, 20000, 0, gap), "B on where R turns off: allowed");
        CHECK(stagger_valid(dd, 30000, 0, gap), "apart: allowed");
    }
    /* hysteresis: a little off the optimum (T / 100) is not worth moving */
    {
        static const double ex[5] = { 0.83, 0.70, 0.42, 0.09, 0.34 };
        uint16_t o2, o3;
        for (k = 0; k < 5; k++) d[k] = (uint16_t)(ex[k] * 65535 + 0.5);
        stagger_choose(d, PEAK_MA, gap, STAGGER_FIXED_O2, STAGGER_FIXED_O3, &r);
        o2 = (uint16_t)(r.o2 + 655u);
        o3 = (uint16_t)(r.o3 - 655u);
        stagger_choose(d, PEAK_MA, gap, o2, o3, &r);
        printf("  T/100 off the optimum: %.2f A rms vs best %.2f, move %d\n", r.ripple_now / 1000.0,
               r.ripple_best / 1000.0, r.move);
        CHECK(!r.move, "small improvement: no move");
    }

    /* slew: period-by-period model of a timer moved towards a new offset (its period length changes one period
       after it is written), at every preset; also the only-shorter case at 2.197 kHz */
    {
        uint32_t p, worst_n = 0;
        int ok = 1;
        for (p = 0; p < PWM_PRESET_COUNT; p++) {
            uint32_t steps = pwm_preset_steps[p], maxs = steps / 512u < 2u ? 2u : steps / 512u, t;
            int longer = steps + maxs <= 65536u;
            for (t = 0; t < 20; t++) {
                uint32_t cur = (uint32_t)rnd() % steps, tgt = (uint32_t)rnd() % steps, n = 0;
                int32_t pend = 0, step;
                for (;;) {
                    step = stagger_slew(cur, tgt, pend, steps, maxs, longer);
                    if (step > (int32_t)maxs || step < -(int32_t)maxs || (!longer && step > 0)) ok = 0;
                    cur = (uint32_t)((int32_t)cur + pend + (int32_t)steps) % steps;   /* last period's change lands */
                    pend = step;
                    if (step == 0 && pend == 0) {
                        uint32_t e = (tgt + steps - cur) % steps;
                        if (e > 4u && e < steps - 4u) {
                            if (++n > 3u) { ok = 0; break; }
                            continue;
                        }
                        break;
                    }
                    if (++n > 3u * steps / maxs + 10u) { ok = 0; break; }
                }
                if (n > worst_n) worst_n = n;
            }
        }
        printf("  slew: every preset, random moves arrive within +-4 counts, <= %u periods, |step| <= period / 512%s\n",
               worst_n, ok ? "" : " FAILED");
        CHECK(ok, "slew");
    }
    test_stagger_cap();
}

/* 0.6.5: C10's ripple load: % of the rating per preset (a raw period: the preset at or above its frequency), the
 * ~1 min rms (a filter of the square), the peak; the research tool's one 5 A channel at 50 %; the console */
static void test_stagger_cap(void)
{
    static const uint32_t lim[PWM_PRESET_COUNT] = { 2940, 2900, 2710, 2690, 2980, 2620, 2920, 3110, 3460, 3730, 3950,
                                                     4160, 4510, 5150, 6140 };   /* k-interp.txt, mA rms */
    stagger_cap_t c;
    uint32_t p, s;
    uint16_t duty[5] = { 0, 0, 0, 32768, 0 }, peak[5] = { 0, 0, 0, 5000, 0 };
    cli_t cl;
    cli_status_t st;

    printf("C10 ripple load: per preset, the ~1 min rms, the peak, the console\n");
    for (p = 0; p < PWM_PRESET_COUNT; p++) {          /* half the limit is 50 % at every preset (+-1 for the 25 mA
                                                        table) */
        memset(&c, 0, sizeof c);
        stagger_cap_second(&c, lim[p] / 2u, pwm_preset_steps[p]);
        CHECK(c.now >= 49u && c.now <= 51u, "preset %u: half the limit is %u %%", p, c.now);
    }
    memset(&c, 0, sizeof c);
    stagger_cap_second(&c, 1730, 6000);               /* 24 kHz: the 25 kHz preset's limit, 3.46 A */
    CHECK(c.now == 50u, "a raw period between presets: %u %%", c.now);
    /* one 5 A channel at 50 %: 2.5 A rms of ripple, 72 % at 25 kHz (the research tool: 70 %, steps 82 %) */
    memset(&c, 0, sizeof c);
    stagger_cap_second(&c, stagger_ripple_ma(duty, peak, STAGGER_FIXED_O2, STAGGER_FIXED_O3), 5760);
    CHECK(c.now == 72u, "one 5 A channel at 50 %%, 25 kHz: %u %%", c.now);
    /* the ~1 min rms: 10 min at 50 %, then off (C10 cools with ~64 s), the peak stays; 0 / 100 % each second: the
     * rms, 71 % (heating goes with the square), not the mean */
    memset(&c, 0, sizeof c);
    for (s = 0; s < 600u; s++) {
        stagger_cap_second(&c, 1730, 5760);
    }
    CHECK(c.avg >= 49u && c.avg <= 50u && c.peak == c.avg, "10 min at 50 %%: ~1 min %u %%", c.avg);
    for (s = 0; s < 64u; s++) {
        stagger_cap_second(&c, 0, 5760);
    }
    CHECK(c.now == 0u && c.avg >= 28u && c.avg <= 32u && c.peak >= 49u, "64 s off: %u %% (e^-1/2 of 50), peak %u %%",
          c.avg, c.peak);
    memset(&c, 0, sizeof c);
    for (s = 0; s < 1200u; s++) {
        stagger_cap_second(&c, (s & 1u) ? 3460u : 0u, 5760);
    }
    CHECK(c.avg >= 69u && c.avg <= 72u, "0 / 100 %% every other second: %u %% (rms)", c.avg);
    {                                                 /* the HEALTH page's byte 23 (health_page first) */
        uint8_t pg[24];
        health_t h;
        health_init(&h);
        health_page(&h, 412, 75, pg);
        stagger_cap_page(&c, pg);
        CHECK(pg[STAGGER_CAP_PG] == c.avg && ultra_get24(pg + ULTRA_HE_TEMP_AGE) == 75u, "HEALTH page: byte 23 = %u %%",
              pg[STAGGER_CAP_PG]);
        c.avg = 300;
        stagger_cap_page(&c, pg);
        CHECK(pg[STAGGER_CAP_PG] == 255u, "HEALTH page: 300 %% as 255");
        c.avg = 0;
    }

    /* the console: with the stagger's lines */
    memset(&cl, 0, sizeof cl);
    memset(&st, 0, sizeof st);
    settings_default(&cli_set);
    st.version = "test";
    st.steps = 5760;
    st.en = &test_en;
    st.ripple_ma = 1730;
    st.cap = c;
    cl.write = cap_write;
    cl.apply = cap_apply;
    cl.save_now = cap_save;
    cl.set = &cli_set;
    cl.st = &st;
    run_cli(&cl, "status\r");
    {
        char want[100];
        sprintf(want, "ripple_rms=1.73 A\r\nc10_load=%u %%, 1 min %u %%, peak %u %%\r\n",
                (uint32_t)c.now, (uint32_t)c.avg, (uint32_t)c.peak);
        CHECK(strstr(cli_out, want) != NULL, "status: %s", want);
    }
}

/* ------------------------------------------------------------------------------------------------ spread */
static void test_spread(void)
{
    static uint16_t arr[PATTERN_MAX_LEN], pat[PATTERN_MAX_LEN];
    uint16_t F = dither_floor_counts(20);
    uint32_t p;
    int ok_tab = 1;

    printf("spread spectrum: period tables, dither with varying periods (every 16-bit level, every preset)\n");
    for (p = 0; p < PWM_PRESET_COUNT; p++) {
        uint32_t steps = pwm_preset_steps[p], i, len = pattern_len_for(steps);
        int32_t sum = 0, mn = 0x7FFFFFFF, mx = -0x7FFFFFFF, span = (int32_t)((steps * STAGGER_SPREAD_Q16 + 32768u) >> 16);
        if (len < STAGGER_SPREAD_LEN) len = STAGGER_SPREAD_LEN;
        stagger_spread_arr(steps, len, arr);
        for (i = 0; i < len; i++) {
            int32_t d = (int32_t)arr[i] + 1 - (int32_t)steps;
            sum += d;
            if (d < mn) mn = d;
            if (d > mx) mx = d;
            if (arr[i] > 65535u) ok_tab = 0;
        }
        if (steps + (uint32_t)span <= 65536u) {
            if (sum != 0 || mn < -span - 1 || mx > span + 1 || mx - mn < span) ok_tab = 0;     /* +-6 %, mean 0 */
        } else if (mx > 0 || mn < -2 * span - 1) {
            ok_tab = 0;                                                                      /* 2.197 kHz */
        }
    }
    CHECK(ok_tab, "spread tables: +-6 %% (2.197 kHz: -12..0 %%), zero mean, ARR <= 65535");

    for (p = 0; p < PWM_PRESET_COUNT; p++) {
        uint32_t steps = pwm_preset_steps[p], v, len = pattern_len_for(steps), ml = len, i, tot_p = 0;
        dither_opts_t o = { DITHER_ON, 0, FLOOR_SKIP, 0 };
        double maxerr = 0, maxerr_low = 0, prev = -1, f = (double)F_TIM_HZ / steps;
        int mono = 1, ok_pulse = 1, worst_gap = 0;
        o.floor = F;
        if (len < STAGGER_SPREAD_LEN) len = STAGGER_SPREAD_LEN;
        stagger_spread_arr(steps, len, arr);
        for (i = 0; i < len; i++) tot_p += arr[i] + 1u;
        for (v = 0; v <= 65535u; v += (v < 2000u ? 1u : 7u)) {
            double sum = 0, avg, exact = (double)v / 65535.0;
            int gap = 0, maxgap = 0, any = 0;
            dither_build_var((uint16_t)v, steps, arr, (uint8_t)len, (uint8_t)ml, &o, (uint8_t)(v % len), pat, 1, 0);
            for (i = 0; i < 2u * len; i++) {
                uint32_t x = pat[i % len], pp = arr[i % len] + 1u;
                if (i < len) sum += x;
                if (x > pp) ok_pulse = 0;
                if (x != 0 && x < F) ok_pulse = 0;
                if (x != pp && x + F > pp && x != 0) ok_pulse = 0;
                if (x) { any = 1; if (gap > maxgap) maxgap = gap; gap = 0; } else gap++;
            }
            avg = sum / tot_p;                                           /* time-weighted duty */
            if (avg < prev - 0.5 / 65535.0) {                            /* monotonic within 0.5 LSB16 */
                if (mono) printf("    dip at v %u: %.3f LSB16 (exact %.1f counts, floor %u)\n", v, (prev - avg) * 65535.0, exact * steps, F);
                mono = 0;
            }
            prev = avg;
            if (any && maxgap + 1 > worst_gap) worst_gap = maxgap + 1;
            if (exact * tot_p / len < F) {                               /* average period of the table */
                double e = fabs(avg - exact) * tot_p / len;              /* counts per period */
                if (e > maxerr_low) maxerr_low = e;
            } else if (exact * tot_p / len <= tot_p / len - F) {
                double e = fabs(avg - exact) * 65535.0;                  /* 16-bit LSB */
                if (e > maxerr) maxerr = e;
            }
        }
        printf("  preset %2u %8.1f Hz table %2u: error %.3f LSB16 above the floor, %.2f counts below, pulses repeat >= %5.0f Hz%s%s\n",
               p, f, len, maxerr, maxerr_low, f / worst_gap, mono ? "" : "  NOT MONOTONIC", ok_pulse ? "" : "  SHORT PULSE");
        CHECK(maxerr <= 0.501 && mono && ok_pulse, "spread preset %u: error %.3f", p, maxerr);
        CHECK(maxerr_low <= ((len + ml - 1) / ml) * F / (2.0 * len) + 0.05, "spread preset %u: error below floor %.2f", p, maxerr_low);
        CHECK(f / worst_gap >= 1500.0 || ml == 1, "spread preset %u: pulse repetition %.0f Hz", p, f / worst_gap);
    }
    /* right-aligned and modes */
    {
        uint32_t steps = 5760, len = STAGGER_SPREAD_LEN, i, s1 = 0, s2 = 0, tot = 0;
        static uint16_t pr[PATTERN_MAX_LEN];
        dither_opts_t o = { DITHER_ON, 0, FLOOR_SKIP, 29 };
        stagger_spread_arr(steps, len, arr);
        dither_build_var(20000, steps, arr, (uint8_t)len, 12, &o, 3, pat, 1, 0);
        dither_build_var(20000, steps, arr, (uint8_t)len, 12, &o, 3, pr, 1, 1);
        for (i = 0; i < len; i++) {
            s1 += pat[i];
            s2 += arr[i] + 1u - pr[i];
            tot += arr[i] + 1u;
        }
        CHECK(s1 == s2, "right-aligned table carries the same pulses");
        o.mode = DITHER_OFF;
        dither_build_var(20000, steps, arr, (uint8_t)len, 12, &o, 0, pat, 1, 0);
        for (i = 0; i < len; i++) {
            double t = 20000.0 * (arr[i] + 1u) / 65535.0;
            CHECK(fabs(pat[i] - t) <= 0.5001, "dither off: nearest count of each period");
        }
        printf("  right-aligned tables, dither off (nearest count per period): ok; 25 kHz table spans %u..%u counts\n",
               5760u - 346u, 5760u + 346u);
        (void)tot;
    }
    /* adaptive with spread: offsets limited to 0.85 of the period */
    {
        static const double ex[5] = { 0.83, 0.70, 0.42, 0.09, 0.34 };
        uint16_t d[5];
        stagger_result_t r;
        int k, over = 0;
        for (k = 0; k < 5; k++) d[k] = (uint16_t)(ex[k] * 65535 + 0.5);
        stagger_choose_max(d, PEAK_MA, stagger_gap_q16(5760), STAGGER_FIXED_O2, STAGGER_FIXED_O3, STAGGER_SPREAD_OMAX, &r);
        printf("  adaptive + spread, example mix: %.2f -> %.2f A rms at %.3f / %.3f T\n", r.ripple_now / 1000.0,
               r.ripple_best / 1000.0, r.o2 / 65536.0, r.o3 / 65536.0);
        CHECK(r.move && r.o2 <= STAGGER_SPREAD_OMAX && r.o3 <= STAGGER_SPREAD_OMAX && r.ripple_best < 1500, "adaptive + spread");
        for (k = 0; k < 500; k++) {
            random_mix(d);
            stagger_choose_max(d, PEAK_MA, stagger_gap_q16(5760), STAGGER_FIXED_O2, STAGGER_FIXED_O3, STAGGER_SPREAD_OMAX, &r);
            if (r.move && (r.o2 > STAGGER_SPREAD_OMAX || r.o3 > STAGGER_SPREAD_OMAX)) over++;
        }
        CHECK(over == 0, "offsets above the spread limit");
        CHECK(stagger_slew_straight(100, 50000, 0, 11) == 11 && stagger_slew_straight(50000, 100, 0, 11) == -11 &&
              stagger_slew_straight(100, 103, 0, 11) == 0, "straight slew");
    }
}

/* opt-in sweep: TEST_SWEEP=<seeds> (no number: 200) runs the random-mix check over seeds 1..n (1500 mixes each) and prints every
 * failing seed: adaptive worse while fixed keeps the edge rule, worse by more than 10 % where fixed breaks it, or a
 * move to offsets that break the edge rule */
static void test_stagger_sweep(int seeds)
{
    int sd, fails = 0, wv = 0, we = 0, br = 0;
    uint32_t worst = 0;
    double ratio = 0;
    for (sd = 1; sd <= seeds; sd++) {
        st_mix_t m;
        st_seed((unsigned long long)sd);
        st_mixes(1500, &m, NULL, NULL, NULL);
        wv += m.worse_valid; we += m.worse_edge; br += m.broken;
        if (m.worst_ma > worst) { worst = m.worst_ma; ratio = m.worst_ratio; }
        if (m.worse_valid || m.broken || m.worst_ratio > 1.10) {
            fails++;
            printf("  stagger sweep FAIL seed %d: worse with fixed valid %d, edge rule broken %d, worst x%.3f\n", sd,
                   m.worse_valid, m.broken, m.worst_ratio);
            if (m.broken)
                printf("    edge rule broken at %u/%u/%u/%u/%u, offsets %u / %u\n", m.broken_d[0], m.broken_d[1],
                       m.broken_d[2], m.broken_d[3], m.broken_d[4], m.broken_o2, m.broken_o3);
        }
    }
    printf("stagger sweep: %d seeds x 1500 mixes: %d failing seeds; worse with fixed valid %d, edge rule broken %d; by "
           "design (fixed breaks the edge rule, a valid position <= 10 %% worse) %d, worst +%u mA (x%.3f)\n", seeds,
           fails, wv, br, we, worst, ratio);
    CHECK(fails == 0, "stagger sweep: %d failing seeds", fails);
}
