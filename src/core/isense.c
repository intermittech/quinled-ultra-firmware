/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Board current sense: limiter, trips, calibration, regression (see isense.h). Integer only (RV32IMC, no FPU). */
#include "isense.h"

#define Q16              65536u
#define K_MIN            1024u        /* the limiter never scales below 1/64 (a fault trips instead) */
#define MARGIN_FB_Q16    66192u       /* feedback limiter engages above limit x 1.010 */
#define MARGIN_MODEL_Q16 66847u       /* with a model: above limit x 1.020 (model errors only) */
#define RELEASE_BELOW_Q16 64881u      /* release while below limit x 0.990 */
#define CONF_MIN         64u          /* regression windows with a small error before the model is used */
#define CONF_MAX         1000u
#define G_MIN            16384u       /* model gain correction 0.25 .. 4 */
#define G_MAX            262144u

static uint32_t umin(uint32_t a, uint32_t b) { return a < b ? a : b; }

static const uint16_t LVL_DUTY[ISENSE_LEVELS] = { 16384u, 32768u, 49152u, 65535u };
static const uint8_t LVL_PCT[ISENSE_LEVELS] = { 25u, 50u, 75u, 100u };

static void shape_linear(isense_t *s, uint32_t ch)
{
    s->shape[ch][0] = 8192u;
    s->shape[ch][1] = 16384u;
    s->shape[ch][2] = 24576u;
}

/* a channel's current at output v (Q16) over that at 100 %, Q15: linear between 0, the 25 / 50 / 75 % points, 1 */
static uint32_t shape_at(const isense_t *s, uint32_t ch, uint32_t v)
{
    uint32_t j = v >> 14, f = v & 0x3FFFu;
    int32_t a, b;
    if (j > 3u) {
        return 32768u;
    }
    a = j == 0u ? 0 : (int32_t)s->shape[ch][j - 1u];
    b = j == 3u ? 32768 : (int32_t)s->shape[ch][j];
    return (uint32_t)(a + (((b - a) * (int32_t)f) >> 14));
}

static int32_t raw_to_ma(const isense_t *s, uint64_t sum, uint32_t n)
{
    return (int32_t)((sum * s->ma_per_lsb_q16 / n) >> 16);
}

static void set_fast_level(isense_t *s)
{
    uint32_t ma = (uint32_t)s->fuse_ma * 3u / 2u, raw;
    uint32_t fs_ma = (uint32_t)(((uint64_t)ISENSE_ADC_MAX * s->ma_per_lsb_q16) >> 16);
    if (ma > fs_ma * 97u / 100u) {
        ma = fs_ma * 97u / 100u;                           /* below the ADC's end of range */
    }
    raw = (uint32_t)(((uint64_t)ma << 16) / s->ma_per_lsb_q16);
    s->fast_raw = (uint16_t)(s->enabled ? raw : 0xFFFFu);
}

void isense_init(isense_t *s, uint8_t enabled, uint16_t fuse_ma, uint16_t board_ma)
{
    uint8_t *p = (uint8_t *)s;
    uint32_t i;
    for (i = 0; i < sizeof(*s); i++) {
        p[i] = 0;
    }
    s->enabled = enabled;
    s->ma_per_lsb_q16 = ISENSE_GAIN_DEFAULT_Q16;
    s->fuse_ma = fuse_ma;
    s->board_ma = board_ma;
    s->limit_ma = board_ma;
    s->k = s->k_meas = s->k_fb = Q16;
    s->g = Q16;
    for (i = 0; i < ISENSE_CH; i++) {
        shape_linear(s, i);
    }
    set_fast_level(s);
}

void isense_set_limit(isense_t *s, uint16_t limit_ma)
{
    s->limit_ma = (limit_ma == 0u || limit_ma > s->board_ma) ? s->board_ma : limit_ma;
}

void isense_set_gain(isense_t *s, uint32_t ma_per_lsb_q16)
{
    s->ma_per_lsb_q16 = ma_per_lsb_q16;
    set_fast_level(s);
}

void isense_set_model(isense_t *s, const int32_t ch_ma[ISENSE_CH], int32_t zero_ma)
{
    uint32_t i;
    for (i = 0; i < ISENSE_CH; i++) {
        s->ch_ma[i] = ch_ma[i];
        shape_linear(s, i);
    }
    s->zero_ma = zero_ma;
    s->zero_valid = 1;
    s->calibrated = 1;
    s->conf = CONF_MAX;
    s->g = Q16;
}

void isense_cal_start(isense_t *s, uint16_t duty)
{
    if (!s->enabled || s->tripped != ISENSE_TRIP_NONE) {
        return;
    }
    s->cal_step = 1;
    s->cal_t = 0;
    s->cal_levels = duty == 0u ? (uint8_t)ISENSE_LEVELS : 1u;
    s->cal_duty = duty == 0u ? 0u : (duty < 1024u ? 1024u : duty);
    s->cal_sum = 0;
    s->cal_n = 0;
}

uint32_t isense_cal_ms(uint16_t duty)
{
    if (duty != 0u) {
        return (1u + ISENSE_CH) * (ISENSE_CAL_SETTLE_MS + ISENSE_CAL_MEAS_MS);
    }
    return ISENSE_CAL_ZERO_SETTLE_MS + ISENSE_CAL_ZERO_MEAS_MS +
           ISENSE_CH * ISENSE_LEVELS * (ISENSE_CAL_LVL_SETTLE_MS + ISENSE_CAL_LVL_MEAS_MS);
}

/* the model from a test: I_k = the channel's current at 100 % (single level: scaled from its duty), the curve
 * through the 25 / 50 / 75 % points; a channel under 20 mA at 100 % has nothing connected */
static void model_from_cal(isense_t *s)
{
    uint32_t i, l;
    for (i = 0; i < ISENSE_CH; i++) {
        shape_linear(s, i);
        if (s->cal_have == 1u) {
            uint32_t di = s->cal_ma[i][0], d = (uint32_t)s->cal_pct1 * 65535u / 100u;
            int32_t ma = d != 0u ? (int32_t)((di << 16) / d) : 0;
            s->ch_ma[i] = (di < 10u || ma < 20) ? 0 : ma;
        } else {
            uint32_t full = s->cal_ma[i][ISENSE_LEVELS - 1u];
            s->ch_ma[i] = full < 20u ? 0 : (int32_t)full;
            if (full >= 20u) {
                for (l = 0; l + 1u < ISENSE_LEVELS; l++) {
                    uint32_t q = ((uint32_t)s->cal_ma[i][l] << 15) / full;
                    s->shape[i][l] = (uint16_t)(q > 65535u ? 65535u : q);
                }
            }
        }
    }
    s->zero_ma = s->cal_zero;
    s->zero_valid = 1;
    s->calibrated = 1;
    s->conf = CONF_MAX;
    s->g = Q16;
}

int isense_cal_get(const isense_t *s, isense_cal_t *c)
{
    uint32_t i, l;
    uint8_t *p = (uint8_t *)c;
    for (i = 0; i < sizeof(*c); i++) {
        p[i] = 0;
    }
    if (s->cal_have == 0u) {
        return 0;
    }
    c->levels = s->cal_have;
    for (l = 0; l < ISENSE_LEVELS; l++) {
        c->pct[l] = s->cal_have == 1u ? (l == 0u ? s->cal_pct1 : 0u) : LVL_PCT[l];
    }
    c->zero_ma = s->cal_zero;
    for (i = 0; i < ISENSE_CH; i++) {
        for (l = 0; l < ISENSE_LEVELS; l++) {
            c->ma[i][l] = s->cal_ma[i][l];
        }
    }
    return 1;
}

void isense_cal_set(isense_t *s, const isense_cal_t *c)
{
    uint32_t i, l;
    if (c->levels != 1u && c->levels != ISENSE_LEVELS) {
        return;
    }
    if (c->levels == 1u && (c->pct[0] < 5u || c->pct[0] > 100u)) {
        return;
    }
    s->cal_have = c->levels;
    s->cal_pct1 = c->levels == 1u ? c->pct[0] : 0u;
    s->cal_zero = c->zero_ma;
    for (i = 0; i < ISENSE_CH; i++) {
        for (l = 0; l < ISENSE_LEVELS; l++) {
            s->cal_ma[i][l] = c->ma[i][l];
        }
    }
    model_from_cal(s);
}

uint32_t isense_ch_ma(const isense_t *s, uint32_t ch, uint32_t v)
{
    if (ch >= ISENSE_CH || s->ch_ma[ch] <= 0) {
        return 0;
    }
    return (uint32_t)(((uint64_t)(uint32_t)s->ch_ma[ch] * shape_at(s, ch, v)) >> 15);
}

uint32_t isense_on_ma(const isense_t *s, uint32_t ch, uint32_t v)
{
    if (ch >= ISENSE_CH || s->ch_ma[ch] <= 0) {
        return 0;
    }
    if (v < 16384u) {
        v = 16384u;                                    /* below 25 %: the 25 % point's current while on */
    }
    return (uint32_t)(((uint64_t)isense_ch_ma(s, ch, v) << 16) / v);
}

uint32_t isense_connected_ma(const isense_t *s)
{
    uint32_t i, sum = 0;
    if (!isense_model_ok(s)) {
        return 0;
    }
    for (i = 0; i < ISENSE_CH; i++) {
        if (s->ch_ma[i] > 0) {
            sum += (uint32_t)s->ch_ma[i];
        }
    }
    return sum;
}

static int model_usable(const isense_t *s)
{
    return s->calibrated || s->conf >= CONF_MIN;
}

int isense_model_ok(const isense_t *s)
{
    return s->enabled && model_usable(s);
}

/* model: board LED current (mA, zero excluded) for outputs v at k = 1 */
uint32_t isense_model_ma(const isense_t *s, const uint16_t v[ISENSE_CH])
{
    uint32_t i, p = 0;
    for (i = 0; i < ISENSE_CH; i++) {
        p += isense_ch_ma(s, i, v[i]);
    }
    return p;
}

/* LED share of the limit (the zero, i.e. the logic current, counts against it) */
static uint32_t limit_led(const isense_t *s)
{
    int32_t z = s->zero_valid ? s->zero_ma : 0;
    int32_t l = (int32_t)s->limit_ma - z;
    return l > 50 ? (uint32_t)l : 50u;
}

static uint32_t overload_level(const isense_t *s)
{
    uint32_t a = (uint32_t)s->limit_ma * 5u / 4u, b = (uint32_t)s->limit_ma + 500u;
    return umin(a > b ? a : b, s->fuse_ma);
}

static uint32_t retry_ms(uint8_t n)
{
    uint32_t t = ISENSE_RETRY_MS << n;
    return t > ISENSE_RETRY_MAX_MS ? ISENSE_RETRY_MAX_MS : t;
}

uint32_t isense_retry_left_ms(const isense_t *s)
{
    uint32_t t;
    if (s->tripped == ISENSE_TRIP_NONE) {
        return 0;
    }
    t = retry_ms(s->retry_n);
    return s->trip_cause == ISENSE_TRIP_NONE ? t : (s->off_ms >= t ? 0u : t - s->off_ms);   /* not registered yet: all */
}

/* the limiter state after a trip: the retry starts scaled so that the current seen at the trip lands at the
 * limit (a legitimate overload comes back regulated; a short trips again and backs off) */
static void trip_scale(isense_t *s, uint32_t i_trip_ma)
{
    uint32_t l = limit_led(s), kk;
    if (i_trip_ma <= l) {
        return;
    }
    kk = (uint32_t)(((uint64_t)s->k_meas * l) / i_trip_ma);
    s->k_fb = kk < K_MIN ? K_MIN : kk;
}

/* one 16 ms window done: zero tracking or an NLMS step on the channel currents */
static void window_done(isense_t *s)
{
    int32_t mean_i = (int32_t)(s->win_i / s->win_n);
    uint32_t d[ISENSE_CH], i;
    int64_t norm = 0, pred = 0, e;
    if (s->win_dark) {
        if (s->dark_ms >= ISENSE_WIN_MS) {             /* settled: the input current is the logic + offset */
            if (!s->zero_valid) {
                s->zero_ma = mean_i;
                s->zero_valid = 1;
            } else {
                s->zero_ma += (mean_i - s->zero_ma) / 4;
            }
        }
        return;
    }
    for (i = 0; i < ISENSE_CH; i++) {
        d[i] = s->win_d[i] / s->win_n;                 /* mean of the applied duty through the curve, Q16 */
        norm += (int64_t)d[i] * d[i];
        pred += (int64_t)d[i] * s->ch_ma[i];
    }
    if (norm < (int64_t)Q16 * Q16 / 400) {             /* < 5 % total drive: too little signal */
        return;
    }
    pred >>= 16;
    e = (int64_t)(mean_i - (s->zero_valid ? s->zero_ma : 0)) - pred;
    {
        int64_t lim = (int64_t)(mean_i / 25), ae = e < 0 ? -e : e;   /* 4 % */
        if (lim < 30) {
            lim = 30;
        }
        if (ae <= lim) {
            if (s->conf < CONF_MAX) s->conf++;
        } else {
            s->conf /= 2;
        }
    }
    for (i = 0; i < ISENSE_CH; i++) {
        /* dI_k = mu e d_k / sum d^2, mu = 1/4 (1/16 once calibrated) */
        int64_t num = e * (int64_t)d[i] * (int64_t)Q16;
        int64_t dk = num / norm / (s->calibrated ? 16 : 4);
        int32_t v = s->ch_ma[i] + (int32_t)dk;
        s->ch_ma[i] = v < 0 ? 0 : (v > 20000 ? 20000 : v);
    }
}

static uint32_t cal_ms(isense_t *s)
{
    uint32_t i, l, n = 1u + ISENSE_CH * (uint32_t)s->cal_levels;
    uint32_t settle = s->cal_levels == 1u ? ISENSE_CAL_SETTLE_MS
                      : s->cal_step == 1u ? ISENSE_CAL_ZERO_SETTLE_MS : ISENSE_CAL_LVL_SETTLE_MS;
    uint32_t meas = s->cal_levels == 1u ? ISENSE_CAL_MEAS_MS
                    : s->cal_step == 1u ? ISENSE_CAL_ZERO_MEAS_MS : ISENSE_CAL_LVL_MEAS_MS;
    s->cal_t++;
    if (s->cal_t > settle) {
        s->cal_sum += s->i_ms;
        s->cal_n++;
    }
    if (s->cal_t < settle + meas) {
        return 0;
    }
    s->cal_res[s->cal_step - 1u] = (int32_t)(s->cal_sum / (int64_t)s->cal_n);
    s->cal_sum = 0;
    s->cal_n = 0;
    s->cal_t = 0;
    if (s->cal_step < n) {
        s->cal_step++;                                 /* next level / channel */
        return ISENSE_EV_REAPPLY;
    }
    s->cal_step = 0;                                   /* done: the test, then the model from it */
    s->cal_have = s->cal_levels;
    s->cal_pct1 = s->cal_levels == 1u ? (uint8_t)((s->cal_duty * 100u + 32767u) / 65535u) : 0u;
    s->cal_zero = (int16_t)(s->cal_res[0] < -32768 ? -32768 : (s->cal_res[0] > 32767 ? 32767 : s->cal_res[0]));
    for (i = 0; i < ISENSE_CH; i++) {
        for (l = 0; l < ISENSE_LEVELS; l++) {
            int32_t di = l < s->cal_levels ? s->cal_res[1u + i * s->cal_levels + l] - s->cal_res[0] : 0;
            s->cal_ma[i][l] = (uint16_t)(di < 10 ? 0 : (di > 65535 ? 65535 : di));   /* < 10 mA: nothing */
        }
    }
    model_from_cal(s);
    s->k_fb = Q16;
    return ISENSE_EV_CAL_DONE | ISENSE_EV_REAPPLY;
}

uint32_t isense_ms(isense_t *s, uint32_t sum, uint32_t n, uint32_t peak_raw)
{
    uint32_t ev = 0, i, dark = 1;
    int32_t i_led;
    if (!s->enabled) {
        return 0;
    }
    if (n != 0u) {
        s->i_ms = raw_to_ma(s, sum, n);
    }
    s->peak_ma = (uint16_t)umin((uint32_t)((peak_raw * (uint64_t)s->ma_per_lsb_q16) >> 16), 65535u);
    s->i_avg += (s->i_ms - s->i_avg) / 64;
    s->k_meas = s->k;                                  /* the k of the ms just measured */

    /* ---- tripped: wait, then retry ---- */
    if (s->tripped != ISENSE_TRIP_NONE) {
        if (s->trip_cause == ISENSE_TRIP_NONE) {       /* new trip (ISR or below) */
            s->trip_cause = s->tripped;
            s->trips++;
            s->off_ms = 0;
            trip_scale(s, s->trip_cause == ISENSE_TRIP_FAST
                              ? (uint32_t)(((uint64_t)s->fast_raw * s->ma_per_lsb_q16) >> 16)
                              : (uint32_t)(s->i_ms > 0 ? s->i_ms : 0));
            if (s->cal_step != 0u) {
                s->cal_step = 0;
                ev |= ISENSE_EV_CAL_FAIL;
            }
        }
        s->ovl_ms = 0;
        s->win_n = 0;
        s->dark_ms = 0;
        if (++s->off_ms >= retry_ms(s->retry_n)) {
            if (s->retry_n < 5u) {
                s->retry_n++;
            }
            s->trip_cause = ISENSE_TRIP_NONE;
            s->calm_ms = 0;
            s->fast_cnt = 0;
            s->tripped = ISENSE_TRIP_NONE;             /* the ISR may trip again from here */
            ev |= ISENSE_EV_RELEASE | ISENSE_EV_REAPPLY;
        }
        return ev;
    }
    if (++s->calm_ms >= ISENSE_RETRY_RESET_MS) {
        s->calm_ms = ISENSE_RETRY_RESET_MS;
        s->retry_n = 0;
    }

    /* ---- overload trip: the current stays above the trip level ---- */
    if (s->i_ms > (int32_t)overload_level(s)) {
        if (++s->ovl_ms >= ISENSE_OVL_MS) {
            s->tripped = ISENSE_TRIP_OVL;              /* registered (and scaled) in the next call */
            return ev | ISENSE_EV_TRIP;
        }
    } else {
        s->ovl_ms = 0;
    }

    /* ---- calibration ---- */
    if (s->cal_step != 0u) {
        return ev | cal_ms(s);
    }

    /* ---- feedback limiter (always on; with a model it only catches model errors) ---- */
    i_led = s->i_ms - (s->zero_valid ? s->zero_ma : 0);
    {
        uint32_t l = limit_led(s);
        uint32_t hi = (uint32_t)(((uint64_t)l * (model_usable(s) ? MARGIN_MODEL_Q16 : MARGIN_FB_Q16)) >> 16);
        uint32_t lo = (uint32_t)(((uint64_t)l * RELEASE_BELOW_Q16) >> 16);
        uint32_t kfb = s->k_fb, tgt;
        if (i_led > (int32_t)hi) {
            tgt = (uint32_t)(((uint64_t)s->k_meas * l) / (uint32_t)i_led);   /* lands at the limit */
            if (tgt < kfb) {
                kfb = tgt < K_MIN ? K_MIN : tgt;
            }
        } else if (i_led < (int32_t)lo && kfb < Q16) {
            uint32_t up = kfb + (uint32_t)(((uint64_t)kfb * ISENSE_RELEASE_Q16) >> 16) + 1u;
            if (i_led > 0) {
                uint32_t cap = (uint32_t)(((uint64_t)s->k_meas * l) / (uint32_t)i_led);
                if (cap < up && cap > kfb) {
                    up = cap;
                }
            }
            kfb = umin(up, Q16);
        }
        if (kfb != s->k_fb) {
            s->k_fb = kfb;
            ev |= ISENSE_EV_REAPPLY;
        }
    }

    /* ---- model gain: measured vs predicted for what was applied ---- */
    if (model_usable(s)) {
        uint32_t pred = isense_model_ma(s, s->applied);
        if (pred >= 100u && i_led > 0) {
            uint32_t gobs = (uint32_t)(((uint64_t)(uint32_t)i_led << 16) / pred);   /* measured / model */
            if (gobs > s->g + s->g / 20u) {
                s->g = gobs;                           /* fast: the model under-predicts */
            } else {
                s->g = (uint32_t)((int32_t)s->g + ((int32_t)gobs - (int32_t)s->g) / 32);
            }
            if (s->g < G_MIN) s->g = G_MIN;
            if (s->g > G_MAX) s->g = G_MAX;
            if (s->limiting) {
                ev |= ISENSE_EV_REAPPLY;               /* k_ff follows g */
            }
        }
    }

    /* ---- regression / zero windows (duties applied during the measured ms) ---- */
    for (i = 0; i < ISENSE_CH; i++) {
        if (s->applied[i] != 0u) {
            dark = 0;
        }
    }
    if (s->win_n == 0u) {
        s->win_i = 0;
        for (i = 0; i < ISENSE_CH; i++) {
            s->win_d[i] = 0;
        }
        s->win_dark = 1;
    }
    s->win_i += s->i_ms;
    for (i = 0; i < ISENSE_CH; i++) {
        s->win_d[i] += shape_at(s, i, s->applied[i]) << 1;   /* the curve: I_k(d) = ch_ma x shape(d) */
    }
    if (!dark) {
        s->win_dark = 0;
    }
    if (++s->win_n >= ISENSE_WIN_MS) {
        window_done(s);
        s->win_n = 0;
    }
    if (dark) {
        if (s->dark_ms < 255u) s->dark_ms++;
    } else {
        s->dark_ms = 0;
    }
    return ev;
}

void isense_apply(isense_t *s, const uint16_t req[ISENSE_CH], uint16_t out[ISENSE_CH])
{
    uint32_t i, k = Q16;
    for (i = 0; i < ISENSE_CH; i++) {
        s->req[i] = req[i];
    }
    if (!s->enabled) {
        for (i = 0; i < ISENSE_CH; i++) {
            out[i] = req[i];
        }
        return;
    }
    if (s->cal_step != 0u) {                           /* calibration pattern: zero, then each channel alone */
        uint32_t st = s->cal_step >= 2u ? s->cal_step - 2u : 0u, ch = st / s->cal_levels;
        uint16_t d = s->cal_levels == 1u ? s->cal_duty : LVL_DUTY[st % ISENSE_LEVELS];
        for (i = 0; i < ISENSE_CH; i++) {
            out[i] = (s->cal_step >= 2u && i == ch) ? d : 0u;
            s->applied[i] = out[i];
        }
        s->k = Q16;
        s->limiting = 0;
        return;
    }
    if (model_usable(s)) {                             /* feed-forward: the content's predicted current */
        uint32_t a = (uint32_t)(((uint64_t)isense_model_ma(s, req) * s->g) >> 16), l = limit_led(s);
        if (a > l) {
            k = (uint32_t)(((uint64_t)l << 16) / a);
        }
    }
    if (s->k_fb < k) {
        k = s->k_fb;
    }
    if (k < K_MIN) {
        k = K_MIN;
    }
    for (i = 0; i < ISENSE_CH; i++) {
        uint32_t v = k >= Q16 ? req[i] : (uint32_t)(((uint64_t)req[i] * k + 32768u) >> 16);
        if (v == 0u && req[i] != 0u) {
            v = 1u;                                    /* an output that should be on stays on */
        }
        out[i] = (uint16_t)v;
        s->applied[i] = s->tripped != ISENSE_TRIP_NONE ? 0u : out[i];
    }
    s->k = k;
    s->limiting = (uint8_t)(k < Q16);
}
