/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "mcutemp.h"
#include "lto.h"

MEASURED void mcutemp_init(mcutemp_t *t)
{
    t->acc = 0;
    t->offset_c10 = 0;
    t->c10 = MCUTEMP_NA;
    t->vdda_mv = 0;
    t->valid = t->have = t->bad = t->jump = 0;
    t->samples = t->rejected = 0;
}

/* n / d rounded to the nearest, d > 0 */
static int32_t div_round(int32_t n, int32_t d)
{
    return n >= 0 ? (n + d / 2) / d : -((-n + d / 2) / d);
}

int mcutemp_convert(uint32_t ts2, uint32_t vref2, int16_t *c10, uint16_t *vdda_mv)
{
    int32_t t, v;
    if (vref2 == 0u || ts2 > 8190u || vref2 > 8190u) {
        return 0;
    }
    v = div_round(MCUTEMP_VREF_MV * 8190, (int32_t)vref2);                  /* VDDA = 1.2 V x full scale / VREFINT */
    /* T(0.1 C) = 250 + (V25 - Vsense) / slope x 10, Vsense = 1.2 V x ts / vref: in 0.1 mV over vref (one division) */
    t = 250 + div_round((MCUTEMP_V25_MV10 * (int32_t)vref2 - MCUTEMP_VREF_MV * 10 * (int32_t)ts2) * 10,
                        (MCUTEMP_SLOPE_UV / 100) * (int32_t)vref2);
    if (v < 2300 || v > 3700 || t < -500 || t > 1500) {
        return 0;
    }
    *c10 = (int16_t)t;
    *vdda_mv = (uint16_t)v;
    return 1;
}

MEASURED void mcutemp_sample(mcutemp_t *t, uint32_t ts2, uint32_t vref2)
{
    int16_t c;
    uint16_t v;
    int32_t d;
    t->samples++;
    if (!mcutemp_convert(ts2, vref2, &c, &v)) {
        t->rejected++;
        if (t->bad < 255u) {
            t->bad++;
        }
    } else {
        d = (int32_t)c * 16 - t->acc;
        if (t->have && (d > 100 * 16 || d < -100 * 16) && ++t->jump < 3u) {
            t->rejected++;                               /* a spike: wait for a second one */
            if (t->bad < 255u) {
                t->bad++;
            }
        } else {
            if (!t->have || t->jump >= 3u) {
                t->acc = (int32_t)c * 16;                /* the first sample, or a real step */
                t->have = 1;
            } else {
                t->acc += d / 8;
            }
            t->jump = 0;
            t->bad = 0;
            t->vdda_mv = v;
        }
    }
    if (t->bad >= MCUTEMP_BAD_MAX) {
        t->valid = 0;
        t->have = 0;
        t->vdda_mv = 0;
    } else if (t->have) {
        t->valid = 1;
    }
    t->c10 = t->valid ? (int16_t)(div_round(t->acc, 16) + t->offset_c10) : MCUTEMP_NA;
}
