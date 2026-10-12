/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* MCU temperature (core/mcutemp.c, included by test_main.c): conversion against the datasheet's sensor model over
 * -40..+125 C and VDDA 2.4..3.6 V, plausibility limits, the filter (convergence, steps, spikes, missing conversions),
 * the offset and the STATUS "n/a" value. */
#include "../src/core/mcutemp.h"

/* the filter's "not valid" is the STATUS page's n/a */
typedef char mcutemp_na_check[(uint16_t)MCUTEMP_NA == ULTRA_ST_NA ? 1 : -1];

/* the ADC's two conversions of the sensor and of VREFINT at die temperature `c` and VDDA `vdda`, with `noise` LSB of
 * random noise per conversion (the typical sensor: V25 1.40 V, 4.3 mV/C; VREFINT 1.20 V) */
static void mt_adc(double c, double vdda, double noise, uint32_t *ts2, uint32_t *vref2)
{
    double vs = 1.40 - 0.0043 * (c - 25.0), vr = 1.20;
    int i;
    *ts2 = *vref2 = 0;
    for (i = 0; i < 2; i++) {
        double a = floor(vs / vdda * 4095.0 + 0.5 + urand(-noise, noise));
        double b = floor(vr / vdda * 4095.0 + 0.5 + urand(-noise, noise));
        *ts2 += (uint32_t)(a < 0 ? 0 : a > 4095 ? 4095 : a);
        *vref2 += (uint32_t)(b < 0 ? 0 : b > 4095 ? 4095 : b);
    }
}

static void test_mcutemp(void)
{
    mcutemp_t t;
    uint32_t ts2, vref2, i;
    int16_t c;
    uint16_t v;
    double worst = 0, worst_v = 0, x;
    int bad = 0;

    printf("MCU temperature: conversion, plausibility, filter\n");
    /* the conversion: quantisation only (two 12-bit conversions each, no noise) */
    for (x = -40.0; x <= 125.0; x += 0.25) {
        double vd;
        for (vd = 2.4; vd <= 3.6001; vd += 0.05) {
            mt_adc(x, vd, 0, &ts2, &vref2);
            if (!mcutemp_convert(ts2, vref2, &c, &v)) {
                bad++;
                continue;
            }
            if (fabs(c / 10.0 - x) > worst) worst = fabs(c / 10.0 - x);
            if (fabs(v - vd * 1000.0) > worst_v) worst_v = fabs(v - vd * 1000.0);
        }
    }
    printf("  -40..+125 C, VDDA 2.4..3.6 V: worst %.2f C, VDDA within %.1f mV (quantisation of 2 x 12 bits)\n", worst,
           worst_v);
    CHECK(bad == 0 && worst <= 0.35 && worst_v <= 3.0, "conversion: %d rejected, %.2f C, %.1f mV", bad, worst, worst_v);
    mt_adc(25.0, 3.3, 0, &ts2, &vref2);
    CHECK(mcutemp_convert(ts2, vref2, &c, &v) && c == 250 && v >= 3298 && v <= 3302, "25.0 C at 3.3 V (%d, %u)", c, v);
    mt_adc(-12.34, 3.3, 0, &ts2, &vref2);
    CHECK(mcutemp_convert(ts2, vref2, &c, &v) && c <= -121 && c >= -126, "below zero rounds to the nearest (%d)", c);
    /* not plausible: no VREFINT, VDDA out of 2.3-3.7 V, temperature out of -50..+150 C, out of range */
    CHECK(!mcutemp_convert(0, 0, &c, &v) && !mcutemp_convert(3000, 0, &c, &v), "no conversion");
    mt_adc(25.0, 2.0, 0, &ts2, &vref2);
    CHECK(!mcutemp_convert(ts2, vref2, &c, &v), "VDDA 2.0 V rejected");
    mt_adc(25.0, 4.0, 0, &ts2, &vref2);
    CHECK(!mcutemp_convert(ts2, vref2, &c, &v), "VDDA 4.0 V rejected");
    mt_adc(160.0, 3.3, 0, &ts2, &vref2);
    CHECK(!mcutemp_convert(ts2, vref2, &c, &v), "160 C rejected");
    mt_adc(-60.0, 3.3, 0, &ts2, &vref2);
    CHECK(!mcutemp_convert(ts2, vref2, &c, &v), "-60 C rejected");
    CHECK(!mcutemp_convert(8191, 3000, &c, &v) && !mcutemp_convert(3000, 8191, &c, &v), "beyond 2 x 4095 rejected");

    /* the filter: 2 LSB of noise per conversion, one sample a second */
    mcutemp_init(&t);
    CHECK(!t.valid && t.c10 == MCUTEMP_NA, "not valid before the first sample (STATUS: 0x8000)");
    for (i = 0; i < 60; i++) {
        mt_adc(41.0, 3.31, 2.0, &ts2, &vref2);
        mcutemp_sample(&t, ts2, vref2);
        if (i == 0) CHECK(t.valid && abs(t.c10 - 410) <= 8, "the first sample is taken as is (%d)", t.c10);
    }
    CHECK(t.valid && abs(t.c10 - 410) <= 3 && abs((int)t.vdda_mv - 3310) <= 10, "steady 41.0 C: %d, VDDA %u mV",
          t.c10, t.vdda_mv);
    {
        int k63 = -1;
        for (i = 0; i < 40; i++) {                        /* the die warms by 6 C at once (the board's load) */
            mt_adc(47.0, 3.31, 0.0, &ts2, &vref2);
            mcutemp_sample(&t, ts2, vref2);
            if (k63 < 0 && t.c10 >= 410 + 38) k63 = (int)i + 1;
        }
        printf("  filter: a 6 C step reaches 63 %% after %d samples (s), noise 2 LSB -> steady within 0.3 C\n", k63);
        CHECK(k63 >= 6 && k63 <= 10 && abs(t.c10 - 470) <= 2, "6 C step: 63 %% after %d s, ends at %d", k63, t.c10);
    }
    i = t.rejected;
    mt_adc(95.0, 3.31, 0, &ts2, &vref2);                  /* one spike: rejected, the value stays */
    mcutemp_sample(&t, ts2, vref2);
    CHECK(t.valid && abs(t.c10 - 470) <= 2 && t.rejected == i + 1, "a single 48 C spike is rejected (%d)", t.c10);
    mt_adc(47.0, 3.31, 0, &ts2, &vref2);
    mcutemp_sample(&t, ts2, vref2);
    for (i = 0; i < 3; i++) {                             /* a real step of > 10 C: taken after 3 samples */
        mt_adc(70.0, 3.31, 0, &ts2, &vref2);
        mcutemp_sample(&t, ts2, vref2);
        if (i < 2) CHECK(abs(t.c10 - 470) <= 2, "step: sample %u held back (%d)", i, t.c10);
    }
    CHECK(abs(t.c10 - 700) <= 3, "a step of 23 C taken after 3 samples (%d)", t.c10);
    for (i = 0; i < MCUTEMP_BAD_MAX - 1u; i++) {
        mcutemp_sample(&t, 0, 0);                         /* conversions that didn't complete */
    }
    CHECK(t.valid && abs(t.c10 - 700) <= 3, "4 missing samples: still valid");
    mcutemp_sample(&t, 0, 0);
    CHECK(!t.valid && t.c10 == MCUTEMP_NA && t.vdda_mv == 0, "5 missing samples in a row: n/a");
    mt_adc(33.0, 3.29, 0, &ts2, &vref2);
    mcutemp_sample(&t, ts2, vref2);
    CHECK(t.valid && abs(t.c10 - 330) <= 2, "valid again with the next good sample, no stale filter (%d)", t.c10);
    t.offset_c10 = -45;                                   /* a one-point calibration: this chip reads 4.5 C high */
    mcutemp_sample(&t, ts2, vref2);
    CHECK(abs(t.c10 - 285) <= 2, "offset applied (%d)", t.c10);
}
