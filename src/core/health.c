/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Channel health: see health.h. */
#include "health.h"
#include "lto.h"

MEASURED void health_init(health_t *h)
{
    uint8_t *b = (uint8_t *)h;
    uint32_t i;
    for (i = 0; i < sizeof(*h); i++) {
        b[i] = 0;
    }
    for (i = 0; i < 5u; i++) {
        h->test_dev[i] = h->model_dev[i] = ULTRA_HE_NA;
    }
}

uint32_t health_i100(const isense_cal_t *c, uint32_t ch)
{
    if (c->levels == ISENSE_LEVELS) {
        return c->ma[ch][ISENSE_LEVELS - 1u];
    }
    return c->levels == 1u && c->pct[0] ? (uint32_t)c->ma[ch][0] * 100u / c->pct[0] : 0u;
}

/* (a - b) in % of b, clamped to int8 (-127..127) */
static int8_t dev_pct(int32_t a, int32_t b)
{
    int32_t d = (a - b) * 100 / b;
    return (int8_t)(d > 127 ? 127 : d < -127 ? -127 : d);
}

/* the zero (logic + offset) against a reference zero: the dark flag */
static uint8_t dark(health_t *h, int32_t zero, int32_t ref, uint8_t have_ref)
{
    int32_t d = have_ref ? zero - ref : zero;
    h->dark_ma = (int16_t)(d > 32767 ? 32767 : d < -32768 ? -32768 : d);
    return (uint8_t)(d >= (have_ref ? HEALTH_DARK_MA : HEALTH_DARK_ABS_MA) ? ULTRA_HE_F_DARK : 0u);
}

MEASURED void health_test_start(health_t *h, const isense_cal_t *stored)
{
    uint32_t i;
    h->prev_have = (uint8_t)(stored->levels != 0u);
    h->prev_zero = stored->zero_ma;
    for (i = 0; i < 5u; i++) {
        h->prev_ma[i] = (uint16_t)health_i100(stored, i);
    }
}

MEASURED uint8_t health_test_done(health_t *h, const isense_cal_t *now)
{
    uint32_t ch, l;
    uint8_t code = 0;
    h->flags = ULTRA_HE_F_TESTED;
    for (ch = 0; ch < 5u; ch++) {
        int32_t n = (int32_t)health_i100(now, ch), p = h->prev_have ? h->prev_ma[ch] : 0;
        uint8_t r = 0;
        h->test_dev[ch] = ULTRA_HE_NA;
        if (n < HEALTH_NOLOAD_MA) {
            r = p >= HEALTH_NOLOAD_MA ? ULTRA_HE_R_BELOW : 0u;     /* lost its load */
        } else {
            if (now->levels == ISENSE_LEVELS) {
                const uint16_t *m = now->ma[ch];
                for (l = 1; l < ISENSE_LEVELS; l++) {
                    if (m[l] <= m[l - 1u]) {
                        r = ULTRA_HE_R_NOT_RISING;
                    }
                }
                if (m[ISENSE_LEVELS - 1u] < 2u * m[0]) {
                    r = ULTRA_HE_R_NOT_RISING;
                }
            }
            if (p >= HEALTH_NOLOAD_MA) {
                h->test_dev[ch] = dev_pct(n, p);
                if (n - p >= HEALTH_TEST_MA && h->test_dev[ch] >= HEALTH_TEST_PCT) {
                    r |= ULTRA_HE_R_ABOVE;
                }
                if (p - n >= HEALTH_TEST_MA && h->test_dev[ch] <= -HEALTH_TEST_PCT) {
                    r |= ULTRA_HE_R_BELOW;
                }
            }
        }
        h->cls[ch] = (uint8_t)(r ? ULTRA_HE_ABNORMAL | r : n < HEALTH_NOLOAD_MA ? ULTRA_HE_NO_LOAD : ULTRA_HE_NORMAL);
        if (r && !code) {
            code = (uint8_t)(ch + 1u);
            h->flags |= ULTRA_HE_F_ABNORMAL;
        }
    }
    h->flags |= dark(h, now->zero_ma, h->prev_zero, h->prev_have);
    return code ? code : (h->flags & ULTRA_HE_F_DARK) ? 6u : 0u;
}

MEASURED void health_run(health_t *h, const isense_t *is, const isense_cal_t *stored)
{
    uint32_t ch, drift = 0;                /* 1: a channel drifts, 2: a drifting channel is on */
    uint8_t f = (uint8_t)(h->flags & (ULTRA_HE_F_TESTED | ULTRA_HE_F_ABNORMAL | HEALTH_F_CAL));
    for (ch = 0; ch < 5u; ch++) {
        int32_t t = (int32_t)health_i100(stored, ch), d = is->ch_ma[ch] - t, thr = HEALTH_NEWLOAD_MA;
        int8_t m = ULTRA_HE_NA;
        if (isense_model_ok(is)) {         /* thr: the change that counts (a load where the test found none: ...) */
            if (t >= HEALTH_NOLOAD_MA) {
                m = dev_pct(is->ch_ma[ch], t);
                thr = 0x7FFFFFFF;
                if (m >= HEALTH_DRIFT_PCT || m <= -HEALTH_DRIFT_PCT) {
                    f |= ULTRA_HE_F_DRIFT;
                    thr = HEALTH_TEST_MA;
                }
            } else if (stored->levels == 0u) {
                thr = 0x7FFFFFFF;          /* (no test: nothing to compare) */
            }
            if ((d < 0 ? -d : d) >= thr) {
                drift |= 1u + (is->applied[ch] != 0u);
            }
        }
        h->model_dev[ch] = m;
    }
    /* the seconds a drifting channel was on (dark ones hold the count); its 16-bit wrap after 18 h of drift does no
     * harm: the flag is latched by then */
    h->drift_s = drift != 0u ? (uint16_t)(h->drift_s + (drift >> 1)) : 0u;
    if (h->drift_s >= HEALTH_SUGGEST_S) {
        f |= HEALTH_F_CAL;                 /* until the next test (health_test_done) */
    }
    if (is->zero_valid) {
        f |= dark(h, is->zero_ma, stored->zero_ma, (uint8_t)(stored->levels != 0u));
    }
    h->flags = f;
}

void health_page(const health_t *h, int16_t temp_peak, uint32_t peak_age_s, uint8_t out[24])
{
    uint32_t i;
    for (i = 0; i < 24u; i++) {
        out[i] = 0;
    }
    out[ULTRA_HE_FLAGS] = h->flags;
    for (i = 0; i < 5u; i++) {
        out[ULTRA_HE_CLASS + i] = h->cls[i];
        out[ULTRA_HE_TEST_DEV + i] = (uint8_t)h->test_dev[i];
        out[ULTRA_HE_MODEL_DEV + i] = (uint8_t)h->model_dev[i];
    }
    ultra_put16(out + ULTRA_HE_DARK_MA, (uint16_t)h->dark_ma);
    ultra_put16(out + ULTRA_HE_TEMP_PEAK, (uint16_t)temp_peak);
    ultra_put24(out + ULTRA_HE_TEMP_AGE, peak_age_s > 0xFFFFFEu ? 0xFFFFFEu : peak_age_s);
}
