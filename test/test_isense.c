/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Firmware model and pin-role contract. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/core/isense.h"

/* sense path under test (mV per A, bandwidth, highest output at VS 3.3 V):
 *   default: the rev 0.4 sense path (200 mV / A, 1 MHz, VS - 0.15 V);
 *   /DISENSE_TEST_MV_PER_A=100u /DISENSE_TEST_BW=50e3 /DISENSE_TEST_VOMAX=3.28: a slower, lower-gain sense path;
 *   /DISENSE_TEST_BW=90e3 /DISENSE_TEST_VOMAX=3.28: a slower sense path.
 * Firmware: ISENSE_MV_PER_A (board.json current.sense_mv_per_a, default 200). */
#ifndef ISENSE_TEST_MV_PER_A
#define ISENSE_TEST_MV_PER_A 200u
#endif
#ifndef ISENSE_TEST_BW
#define ISENSE_TEST_BW 1e6
#endif
#ifndef ISENSE_TEST_VOMAX
#define ISENSE_TEST_VOMAX 3.15
#endif

static int g_fail, g_checks;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static unsigned long long rs = 0x9E3779B97F4A7C15ull;
static unsigned long long rnd(void) { rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17; return rs; }
static double urand(double a, double b) { return a + (b - a) * (double)(rnd() >> 11) / 9007199254740992.0; }
static double gauss(void) { double u = urand(1e-12, 1.0), v = urand(0.0, 1.0); return sqrt(-2.0 * log(u)) * cos(6.283185307 * v); }

#define DT 0.5e-6
#define NCH 5
enum { R = 0, G = 1, B = 2, W1 = 3, W2 = 4 };

typedef struct {
    /* firmware model */
    double vset, ilim, cpsu, vp;
    double psu_tau, isrc;                /* supply regulation: 0 = instant, else its output current follows the
                                            demand with this time constant (a slow loop: sags / overshoots) */
    double rc, lc, ic;
    double rpath;                       /* the board's series resistance in the current path */
    double cb, esr, vcap, vb;
    double ilogic;
    double inom[NCH], vnom;              /* strip current at 100 % (while on) at vnom */
    double alpha[NCH];                   /* strip current ~ (V / vnom)^alpha: 1 resistive (default), 0 constant
                                            current, -1 constant power (strips with their own regulators) */
    int short_ch; double t_short, rshort, lshort, ishort;
    double fuse_in, fuse_tau, fuse_t, fuse_tmax; int fuse_open;
    double ifet_peak, iin_peak;
    /* PWM */
    double period_nom, period, t_per;    /* current period, its start */
    int spread;
    double duty[NCH], duty_next[NCH];
    int forced_off;
    /* measurement */
    double gain_vpa, tau_csa, k_csa, tau_rc, vcsa, vrc, vref, adc_noise, csa_off;
    double conv_period, t_conv; unsigned adc;
    /* firmware */
    isense_t is;
    double t;
    long tick_n, ms_n;
    double mcu_start;                    /* the MCU runs from here (reset ~28 ms after power) */
    /* stats */
    double light_sum, ideal_sum; long ms_cnt;
    double i_true_ms_sum; long i_true_ms_n;
    double t_first_trip; int trips_seen;
    double t_isr_trip;                   /* first fast trip (tick) */
    double ms_true_acc; long ms_true_n; double ms_true_max;   /* 1 ms averages of the true input current */
    double leak_r; double t_leak;        /* firmware model */
    uint16_t last_req[NCH];              /* content of the last ms */
    double spike_v; int prev_on[NCH]; double vspike;   /* PWM edge spikes coupled onto the CSA output */
} sim_t;

typedef void (*content_fn)(sim_t *s, double t, uint16_t req[NCH]);

static void sim_init(sim_t *s, double v, double ilim, double f_pwm)
{
    int k;
    memset(s, 0, sizeof(*s));
    s->vset = v; s->ilim = ilim; s->cpsu = 2000e-6; s->vp = v;
    s->rc = 0.03; s->lc = 1e-6;
    s->rpath = 0.008;
    s->cb = 165e-6; s->esr = 0.03; s->vcap = v; s->vb = v;
    s->ilogic = 0.020;
    s->vnom = v;
    for (k = 0; k < NCH; k++) { s->inom[k] = 1.0; s->alpha[k] = 1.0; }
    s->short_ch = -1;
    s->rshort = 0.05; s->lshort = 2e-6;
    s->fuse_in = 10.0; s->fuse_tau = 26.5 / (2.25 * 100.0);
    s->period_nom = s->period = 1.0 / f_pwm;
    s->gain_vpa = ISENSE_TEST_MV_PER_A / 1000.0; s->tau_csa = 1.0 / (6.2832 * ISENSE_TEST_BW); s->tau_rc = 47e-6;
    s->k_csa = 1.0 - exp(-DT / s->tau_csa);
    s->vref = 3.28; s->adc_noise = 0.8; s->csa_off = 0.0008;
    s->conv_period = 84.0 / 12e6;        /* 71.5-cycle sampling at 12 MHz */
    s->t_conv = s->conv_period;
    s->mcu_start = 0.0;
    s->t_first_trip = -1;
    s->t_isr_trip = -1;
    isense_init(&s->is, 1, 10000, 8000);
    isense_set_gain(&s->is, ISENSE_GAIN_DEFAULT_Q16 * 200u / ISENSE_TEST_MV_PER_A);
}

static int pwm_on(const sim_t *s, int k, double tr)
{
    double T = s->period, d = s->duty[k], x;
    if (d <= 0.0) return 0;
    if (d >= 1.0) return 1;
    switch (k) {
    case R:  return tr < d * T;
    case G:  return tr >= T - d * T;
    case B:  x = fmod(tr - T / 3.0 + T, T); return x < d * T;
    case W1: x = fmod(tr - 2.0 * T / 3.0 + T, T); return x < d * T;
    default: x = fmod(tr - 2.0 * T / 3.0 + T, T); return x >= T - d * T;
    }
}

/* one main-loop ms: the firmware's view (mirrors the planned main.c integration) */
static void sim_ms(sim_t *s, content_fn content)
{
    uint32_t sum, n, peak, ev;
    uint16_t req[NCH], out[NCH];
    int k;
    isense_take(&s->is, &sum, &n, &peak);
    ev = isense_ms(&s->is, sum, n, peak);
    if (ev & ISENSE_EV_TRIP) s->forced_off = 1;
    if (ev & ISENSE_EV_RELEASE) s->forced_off = 0;
    content(s, s->t, req);
    memcpy(s->last_req, req, sizeof(req));
    isense_apply(&s->is, req, out);
    for (k = 0; k < NCH; k++) s->duty_next[k] = out[k] / 65535.0;
    /* delivered light (proportional to LED current) vs the ideal: content scaled exactly to the limit */
    {
        double a = 0, l = 0, lim = s->is.limit_ma / 1000.0 - s->ilogic;
        for (k = 0; k < NCH; k++) {
            a += req[k] / 65535.0 * s->inom[k];
            l += (s->forced_off ? 0.0 : out[k] / 65535.0) * s->inom[k];
        }
        s->light_sum += l;
        s->ideal_sum += a > lim ? lim : a;
        s->ms_cnt++;
    }
}

static void sim_step(sim_t *s, content_fn content)
{
    double tr, iload = 0.0, icap, v_led;
    int k;
    /* PWM period boundary: new duties (table latency), spread spectrum */
    if (s->t - s->t_per >= s->period) {
        s->t_per += s->period;
        s->period = s->spread ? s->period_nom * urand(0.94, 1.06) : s->period_nom;
        for (k = 0; k < NCH; k++) s->duty[k] = s->duty_next[k];
    }
    tr = s->t - s->t_per;
    for (k = 0; k < NCH; k++) {
        int on = !s->forced_off && s->t >= s->mcu_start && pwm_on(s, k, tr);
        if (on != s->prev_on[k]) {
            s->vspike += s->spike_v;                 /* one step (0.5 us) of spike_v: 100 mV x 200 ns -> 40 mV */
            s->prev_on[k] = on;
        }
        if (k == s->short_ch && s->t >= s->t_short) {
            if (on) {
                s->ishort += (s->vb - s->rshort * s->ishort) / s->lshort * DT;
                if (s->ishort < 0) s->ishort = 0;
            } else {
                s->ishort = 0;                       /* freewheels through the flyback diode, not via the caps */
            }
            iload += s->ishort;
            if (s->ishort > s->ifet_peak) s->ifet_peak = s->ishort;
        } else if (on) {
            double r;
            v_led = s->vb > 0 ? s->vb : 0;
            r = v_led / s->vnom;
            if (s->alpha[k] != 1.0 && r > 0.5) {
                r = pow(r, s->alpha[k]);             /* a regulator: constant current / power above half voltage */
            } else if (s->alpha[k] != 1.0) {
                r = 2.0 * r * pow(0.5, s->alpha[k]); /* below it the regulator drops out (resistive to 0) */
            }
            iload += s->inom[k] * r;
        }
    }
    if (s->leak_r > 0 && s->t >= s->t_leak) {
        iload += s->vb / s->leak_r;              /* firmware model */
    }
    /* board node */
    icap = s->ic - iload - (s->vb > 3.0 ? s->ilogic : 0.0);
    s->vcap += icap / s->cb * DT;
    if (s->vcap < 0) s->vcap = 0;
    s->vb = s->vcap + s->esr * icap;
    if (s->vb < 0) s->vb = 0;
    /* firmware model */
    if (!s->fuse_open) {
        s->ic += (s->vp - s->vb - (s->rc + s->rpath) * s->ic) / s->lc * DT;
        if (s->ic < -50) s->ic = -50;
    } else {
        s->ic = 0;
    }
    if (s->ic > s->iin_peak) s->iin_peak = s->ic;
    /* PSU: output cap behind a fast regulator with a current limit */
    {
        double isrc = (s->vset - s->vp) / 0.01;
        if (isrc < 0) isrc = 0;
        if (isrc > s->ilim) isrc = s->ilim;
        if (s->psu_tau > 0) {
            s->isrc += (isrc - s->isrc) * DT / s->psu_tau;
            isrc = s->isrc;
        }
        s->vp += (isrc - s->ic) / s->cpsu * DT;
        if (s->vp < 0) s->vp = 0;
    }
    /* fuse heat */
    s->fuse_t += (s->ic * s->ic / (s->fuse_in * s->fuse_in) - s->fuse_t) / s->fuse_tau * DT;
    if (s->fuse_t > s->fuse_tmax) s->fuse_tmax = s->fuse_t;
    if (s->fuse_t >= 2.25) s->fuse_open = 1;
    /* CSA -> RC -> ADC */
    {
        double vin = s->gain_vpa * s->ic + s->csa_off + s->vspike;
        s->vspike = 0;
        if (vin < 0) vin = 0;
        if (vin > ISENSE_TEST_VOMAX) vin = ISENSE_TEST_VOMAX;
        s->vcsa += (vin - s->vcsa) * s->k_csa;      /* exact first-order step (a 1 MHz amplifier is faster than DT) */
        s->vrc += (s->vcsa - s->vrc) * DT / s->tau_rc;
    }
    if (s->t >= s->t_conv) {
        double code = s->vrc / s->vref * 4096.0 + s->adc_noise * gauss();
        s->adc = code < 0 ? 0u : (code > 4095.0 ? 4095u : (unsigned)(code + 0.5));
        s->t_conv += s->conv_period;
    }
    s->i_true_ms_sum += s->ic;
    s->i_true_ms_n++;
    s->ms_true_acc += s->ic;
    if (++s->ms_true_n >= 2000) {
        double a = s->ms_true_acc / s->ms_true_n;
        if (a > s->ms_true_max) s->ms_true_max = a;
        s->ms_true_acc = 0; s->ms_true_n = 0;
    }
    /* 20 kHz poll tick, 1 ms main loop */
    s->t += DT;
    if (s->t >= (s->tick_n + 1) * 50e-6) {
        s->tick_n++;
        if (s->t >= s->mcu_start) {
            if (isense_tick(&s->is, s->adc)) {
                s->forced_off = 1;               /* pwm_force_off(1) */
                if (s->t_isr_trip < 0) s->t_isr_trip = s->t;
            }
            if (s->tick_n % 20 == 0) {
                sim_ms(s, content);
                if (s->is.trips > (uint32_t)s->trips_seen) {
                    if (s->t_first_trip < 0) s->t_first_trip = s->t;
                    s->trips_seen = (int)s->is.trips;
                }
            }
        }
    }
}

static void sim_run(sim_t *s, double t_end, content_fn c)
{
    while (s->t < t_end && !s->fuse_open) {
        sim_step(s, c);
    }
}

/* ------------------------------------------------------------------------------------------------ contents */
static double c_level, c_t0, c_t1;            /* scenario parameters */

static void cont_all(sim_t *s, double t, uint16_t req[NCH])
{
    int k;
    (void)s;
    for (k = 0; k < NCH; k++) req[k] = (uint16_t)(t >= c_t0 ? c_level * 65535.0 : 0.0);
}

static void cont_off(sim_t *s, double t, uint16_t req[NCH])
{
    int k;
    (void)s; (void)t;
    for (k = 0; k < NCH; k++) req[k] = 0;
}

/* fade 0 -> 100 % over c_t1 s from c_t0, hold, fade back */
static void cont_fade(sim_t *s, double t, uint16_t req[NCH])
{
    double x;
    int k;
    (void)s;
    if (t < c_t0) x = 0;
    else if (t < c_t0 + c_t1) x = (t - c_t0) / c_t1;
    else if (t < c_t0 + 2 * c_t1) x = 1.0;
    else if (t < c_t0 + 3 * c_t1) x = 1.0 - (t - c_t0 - 2 * c_t1) / c_t1;
    else x = 0;
    for (k = 0; k < NCH; k++) req[k] = (uint16_t)(x * 65535.0);
}

/* effect: every ms a random walk per channel (flicker / candle-like), 40-100 % */
static double fx_v[NCH];
static void cont_fx(sim_t *s, double t, uint16_t req[NCH])
{
    int k;
    (void)s;
    for (k = 0; k < NCH; k++) {
        if (t < c_t0) { req[k] = 0; continue; }
        fx_v[k] += urand(-0.03, 0.03);
        if (fx_v[k] < 0.4) fx_v[k] = 0.4;
        if (fx_v[k] > 1.0) fx_v[k] = 1.0;
        req[k] = (uint16_t)(fx_v[k] * 65535.0);
    }
}

/* colour changes every 200 ms (random mixes, some channels off) */
static uint16_t mix_v[NCH]; static double mix_next;
static void cont_mix(sim_t *s, double t, uint16_t req[NCH])
{
    int k;
    (void)s;
    if (t >= mix_next) {
        mix_next = t + 0.2;
        for (k = 0; k < NCH; k++) mix_v[k] = (uint16_t)(urand(0, 1) < 0.25 ? 0 : urand(0.0, 1.0) * 65535.0);
    }
    for (k = 0; k < NCH; k++) req[k] = mix_v[k];
}

/* the calibration pattern is played by isense_apply; the content underneath is a steady mix */
static void cont_steady(sim_t *s, double t, uint16_t req[NCH])
{
    static const uint16_t v[NCH] = { 40000, 20000, 60000, 10000, 30000 };
    int k;
    (void)s; (void)t;
    for (k = 0; k < NCH; k++) req[k] = v[k];
}

/* one channel alone at c_level from c_t0 (the true reference of a calibration level) */
static int c_ch;
static void cont_one(sim_t *s, double t, uint16_t req[NCH])
{
    int k;
    (void)s;
    for (k = 0; k < NCH; k++) req[k] = (uint16_t)(k == c_ch && t >= c_t0 ? c_level * 65535.0 : 0.0);
}

/* a fixed mix */
static double c_mix[NCH];
static void cont_fixed(sim_t *s, double t, uint16_t req[NCH])
{
    int k;
    (void)s;
    for (k = 0; k < NCH; k++) req[k] = (uint16_t)(t >= c_t0 ? c_mix[k] * 65535.0 : 0.0);
}

static void set_strips(sim_t *s, double r, double g, double b, double w1, double w2)
{
    s->inom[R] = r; s->inom[G] = g; s->inom[B] = b; s->inom[W1] = w1; s->inom[W2] = w2;
}

static double true_avg_since(sim_t *s)
{
    double a = s->i_true_ms_n ? s->i_true_ms_sum / s->i_true_ms_n : 0;
    s->i_true_ms_sum = 0; s->i_true_ms_n = 0;
    return a;
}

/* ------------------------------------------------------------------------------------------------ tests */
static void test_measurement_accuracy(void)
{
    static const double fpwm[] = { 2197.0, 20000.0, 25000.0, 50000.0, 100000.0 };
    int i, sp;
    printf("measurement: mean of the tick samples vs the true input current (0.4 s, 4.6 A)\n");
    for (sp = 0; sp < 2; sp++) {
        for (i = 0; i < 5; i++) {
            sim_t s;
            double t_avg;
            int32_t m;
            sim_init(&s, 24.0, 50.0, fpwm[i]);
            s.spread = sp;
            set_strips(&s, 2.0, 1.5, 1.0, 2.5, 1.5);
            c_level = 0.5; c_t0 = 0.0;
            sim_run(&s, 0.1, cont_all);
            true_avg_since(&s);
            s.is.i_avg = s.is.i_ms;
            {
                long n0 = s.ms_cnt; double acc = 0;
                while (s.t < 0.5) {
                    long before = s.ms_cnt;
                    sim_step(&s, cont_all);
                    if (s.ms_cnt != before) acc += s.is.i_ms;
                }
                m = (int32_t)(acc / (s.ms_cnt - n0));
            }
            t_avg = true_avg_since(&s) * 1000.0;
            printf("  %6.0f Hz spread %d: true %7.1f mA, measured %7d mA (%+.2f %%)\n", fpwm[i], sp, t_avg, (int)m,
                   100.0 * (m - t_avg) / t_avg);
            CHECK(fabs(m - t_avg) < 0.015 * t_avg + 15.0, "measurement bias at %.0f Hz spread %d", fpwm[i], sp);
        }
    }
    {
        /* worst-case coupling: every PWM edge puts a +100 mV, 200 ns spike on the CSA output (same polarity) */
        sim_t s;
        double t_avg, acc = 0;
        long n0;
        sim_init(&s, 48.0, 50.0, 100000.0);
        s.spread = 1;
        s.spike_v = 0.04;
        set_strips(&s, 2.0, 1.5, 1.0, 2.5, 1.5);
        c_level = 0.5; c_t0 = 0.0;
        sim_run(&s, 0.1, cont_all);
        true_avg_since(&s);
        n0 = s.ms_cnt;
        while (s.t < 0.5) {
            long before = s.ms_cnt;
            sim_step(&s, cont_all);
            if (s.ms_cnt != before) acc += s.is.i_ms;
        }
        t_avg = true_avg_since(&s) * 1000.0;
        printf("  100 kHz, edge spikes 100 mV x 200 ns at all 10 edges / period: true %.1f mA, measured %.0f mA (%+.2f %%), trips %u\n",
               t_avg, acc / (s.ms_cnt - n0), 100.0 * (acc / (s.ms_cnt - n0) - t_avg) / t_avg, s.is.trips);
        CHECK(s.is.trips == 0u, "edge spikes trip");
    }
}

static void test_powerup(void)
{
    sim_t s;
    printf("power-up: 48 V hot-plug into the empty bulk caps, MCU from 28 ms, outputs from 100 ms\n");
    sim_init(&s, 48.0, 50.0, 25000.0);
    s.vcap = s.vb = 0.0; s.vp = 48.0; s.ic = 0.0;
    s.mcu_start = 0.028;
    set_strips(&s, 2.0, 2.0, 2.0, 1.0, 1.0);
    c_level = 1.0; c_t0 = 0.1;
    sim_run(&s, 0.6, cont_all);
    printf("  inrush peak %.1f A, fuse heat max %.3f (opens at 2.25), trips %u\n", s.iin_peak, s.fuse_tmax, s.is.trips);
    CHECK(!s.fuse_open && s.is.trips == 0u, "power-up trip or fuse");
    CHECK(s.fuse_tmax < 1.0, "inrush heats the fuse too much");
}

/* step 0 -> 100 % on all channels */
static void test_step(double total_a, int calibrated, int expect_trip)
{
    sim_t s;
    double per = total_a / 5.0;
    int32_t ch[NCH];
    int k;
    sim_init(&s, 24.0, 50.0, 25000.0);
    s.spread = 1;
    set_strips(&s, per, per, per, per, per);
    if (calibrated) {
        for (k = 0; k < NCH; k++) ch[k] = (int32_t)(per * 1000.0);
        isense_set_model(&s.is, ch, 20);
    }
    c_level = 1.0; c_t0 = 0.05;
    sim_run(&s, 0.04, cont_all);
    s.iin_peak = 0;
    s.ms_true_max = 0;
    sim_run(&s, 1.5, cont_all);
    printf("  step to %4.1f A (limit 8 A), %s: input peak %5.2f A (LC ringing), max 1 ms avg %5.2f A, settled %5.0f mA, k %.3f, trips %u, fuse heat %.3f\n",
           total_a, calibrated ? "model    " : "no model ", s.iin_peak, s.ms_true_max, (double)s.is.i_avg,
           s.is.k / 65536.0, s.is.trips, s.fuse_tmax);
    CHECK(!s.fuse_open, "fuse opened");
    if (expect_trip) {
        CHECK(s.is.trips >= 1u, "expected a trip");
    } else {
        CHECK(s.is.trips == 0u, "unexpected trip");
    }
    if (total_a > 8.0) {
        CHECK(fabs(s.is.i_avg - 8000.0) < 250.0, "not regulated to the limit (%d mA)", (int)s.is.i_avg);
    }
    if (calibrated && total_a > 8.0) {
        CHECK(s.ms_true_max < 8.0 * 1.06, "feed-forward overshoot: 1 ms average %.2f A", s.ms_true_max);
    }
}

static void test_limit_quality(content_fn c, const char *name, int calibrated, double total_a)
{
    sim_t s;
    double per = total_a / 5.0, kmin = 2.0, kmax = 0.0, worst_step = 0.0, prev_light = -1.0;
    double dev2 = 0, dev_max = 0;
    int32_t ch[NCH];
    int k;
    long over = 0, n = 0;
    sim_init(&s, 24.0, 50.0, 25000.0);
    s.spread = 1;
    set_strips(&s, per * 1.2, per * 0.8, per, per * 1.1, per * 0.9);
    if (calibrated) {
        for (k = 0; k < NCH; k++) ch[k] = (int32_t)(s.inom[k] * 1000.0 * urand(0.95, 1.05));   /* +-5 % model */
        isense_set_model(&s.is, ch, 20);
    }
    for (k = 0; k < NCH; k++) fx_v[k] = 0.7;
    c_t0 = 0.05; c_t1 = 1.0;
    while (s.t < 4.2 && !s.fuse_open) {
        long before = s.ms_cnt;
        sim_step(&s, c);
        if (s.ms_cnt != before && s.t > 0.3) {
            double light = 0, want = 0, lim = 8.0 - s.ilogic;
            for (k = 0; k < NCH; k++) {
                light += s.duty_next[k] * s.inom[k];
                want += s.last_req[k] / 65535.0 * s.inom[k];
            }
            if (want > lim) want = lim;
            if (want > 0.5) {
                double d = light / want - 1.0;
                dev2 += d * d;
                if (fabs(d) > dev_max) dev_max = fabs(d);
            }
            if (prev_light >= 0 && fabs(light - prev_light) > worst_step) worst_step = fabs(light - prev_light);
            prev_light = light;
            if (s.is.k / 65536.0 < kmin) kmin = s.is.k / 65536.0;
            if (s.is.k / 65536.0 > kmax) kmax = s.is.k / 65536.0;
            if (s.is.i_ms > 8300) over++;
            n++;
        }
    }
    printf("  %-24s %s: k %.3f..%.3f, ms over 8.3 A %ld/%ld, light vs ideal: rms %.2f %%, max %.1f %%, worst step %.3f A/ms\n",
           name, calibrated ? "model   " : "no model", kmin, kmax, over, n, 100.0 * sqrt(dev2 / (n ? n : 1)),
           100.0 * dev_max, worst_step);
    CHECK(s.is.trips == 0u && !s.fuse_open, "%s: trip / fuse", name);
    CHECK(over < n / 50 + 5, "%s: too long over the limit", name);
    if (calibrated) {
        CHECK(sqrt(dev2 / (n ? n : 1)) < 0.04, "%s: limited light deviates from the ideal (model)", name);
    }
}

/* expect: 0 = trip, fuse intact; 1 = informative (report only); 2 = LED_+ -> GND, the fuse must open;
 * 3 = LED_+ -> GND, informative */
static void test_short(double v, double ilim, double rshort, double lshort, const char *name, int expect)
{
    sim_t s;
    sim_init(&s, v, ilim, 25000.0);
    s.spread = 1;
    set_strips(&s, 1.0, 1.0, 1.0, 1.0, 1.0);
    if (expect >= 2) {
        s.leak_r = rshort; s.t_leak = 0.5;
    } else {
        s.short_ch = R; s.t_short = 0.5; s.rshort = rshort; s.lshort = lshort;
    }
    c_level = 0.6; c_t0 = 0.05;
    sim_run(&s, 0.5, cont_all);
    s.iin_peak = 0; s.ifet_peak = 0; s.fuse_tmax = 0;
    sim_run(&s, 6.0, cont_all);
    printf("  %-38s first trip %6.0f us after the fault (%s), input peak %6.1f A, FET peak %6.1f A, fuse heat %.2f%s, trips %u\n",
           name, s.t_isr_trip > 0 ? (s.t_isr_trip - 0.5) * 1e6 : (s.t_first_trip > 0 ? (s.t_first_trip - 0.5) * 1e6 : -1.0),
           s.t_isr_trip > 0 ? "fast" : (s.t_first_trip > 0 ? "overload" : "none"),
           s.iin_peak, s.ifet_peak, s.fuse_tmax, s.fuse_open ? " -> OPEN" : "", s.is.trips);
    if (expect == 0) {
        CHECK(!s.fuse_open, "%s: fuse opened", name);
        CHECK(s.is.trips >= 1u, "%s: no trip", name);
        CHECK(s.t_isr_trip > 0 && s.t_isr_trip - 0.5 < 400e-6, "%s: fast trip too slow", name);
    } else if (expect == 2) {
        CHECK(s.fuse_open, "%s: LED_+ to GND: the fuse must open", name);
    }
}

static void test_calibration(void)
{
    sim_t s;
    int k, ok = 1;
    printf("calibration: zero, then each channel alone at 25 %%\n");
    for (k = 0; k < 4; k++) {
        int j;
        double err_max = 0;
        sim_init(&s, k & 1 ? 48.0 : 24.0, 50.0, k < 2 ? 25000.0 : 100000.0);
        s.spread = 1;
        set_strips(&s, urand(0.1, 3), urand(0.1, 3), urand(0.0, 3), urand(0.5, 5), urand(0.5, 5));
        if (k == 3) s.inom[B] = 0.0;               /* nothing connected */
        sim_run(&s, 0.1, cont_steady);
        isense_cal_start(&s.is, 16384);
        while (s.is.cal_step != 0 && s.t < 2.0) sim_step(&s, cont_steady);
        for (j = 0; j < NCH; j++) {
            double e = s.is.ch_ma[j] - s.inom[j] * 1000.0;
            if (fabs(e) > err_max) err_max = fabs(e);
            if (fabs(e) > 0.03 * s.inom[j] * 1000.0 + 40.0) ok = 0;
        }
        printf("  run %d (%2.0f V, %3.0f kHz): true %4.0f %4.0f %4.0f %4.0f %4.0f  measured %4d %4d %4d %4d %4d mA, zero %d mA, worst %.0f mA, %.0f ms\n",
               k, s.vset, 1e-3 / s.period_nom, s.inom[0] * 1e3, s.inom[1] * 1e3, s.inom[2] * 1e3, s.inom[3] * 1e3, s.inom[4] * 1e3,
               (int)s.is.ch_ma[0], (int)s.is.ch_ma[1], (int)s.is.ch_ma[2], (int)s.is.ch_ma[3], (int)s.is.ch_ma[4],
               (int)s.is.zero_ma, err_max, (s.t - 0.1) * 1e3);
    }
    CHECK(ok, "calibration error above 3 %% + 40 mA");
}

/* The standard test: zero, then each channel at 25 / 50 / 75 / 100 %. Plants with sag (a long thin supply cable)
 * and strips with their own regulators; the reference is the true LED current of each channel alone at each level
 * (its own simulation, settled). Then the model with its curves against the true current of mixes. */
typedef struct {
    const char *name;
    double v, rc, fpwm;
    double inom[NCH], alpha[NCH];
    double psu_tau, cpsu;                /* a slow supply: loop time constant, output cap (0 = the default) */
} cal_case_t;

static void sim_case(sim_t *s, const cal_case_t *c)
{
    int k;
    sim_init(s, c->v, 50.0, c->fpwm);
    s->spread = 1;
    s->rc = c->rc;
    for (k = 0; k < NCH; k++) { s->inom[k] = c->inom[k]; s->alpha[k] = c->alpha[k]; }
    if (c->psu_tau > 0) { s->psu_tau = c->psu_tau; s->cpsu = c->cpsu; s->isrc = 0; }
}

/* the true average LED current (A) of a content, settled (logic current excluded) */
static double true_led(const cal_case_t *c, content_fn f)
{
    sim_t s;
    sim_case(&s, c);
    c_t0 = 0.0;
    sim_run(&s, 0.06, f);
    (void)true_avg_since(&s);
    sim_run(&s, 0.14, f);
    return true_avg_since(&s) - s.ilogic;
}

static void test_calibration_levels(void)
{
    static const cal_case_t cases[] = {
        { "24 V, model parameter cable (sag), resistive strips, 25 kHz", 24.0, 0.4, 25000.0,
          { 2.0, 1.5, 1.0, 4.5, 3.0 }, { 1, 1, 1, 1, 1 } },
        { "24 V, model parameter cable, regulated strips (CC / CC / CP / CP / resistive), 8.8 kHz", 24.0, 0.3, 8789.0,
          { 1.5, 1.5, 1.2, 3.5, 2.5 }, { 0, 0, -1, -1, 1 } },
        { "48 V, model parameter, resistive, 100 kHz (nearly linear)", 48.0, 0.03, 100000.0,
          { 1.0, 2.5, 0.0, 4.0, 2.0 }, { 1, 1, 1, 1, 1 } },
        { "24 V, slow supply (loop 3 ms, model parameter), model parameter cable, resistive, 2.197 kHz", 24.0, 0.2, 2197.0,
          { 2.5, 2.0, 1.5, 5.0, 4.0 }, { 1, 1, 1, 1, 1 }, 3e-3, 1000e-6 },
    };
    static const double lvl[4] = { 0.25, 0.5, 0.75, 1.0 };
    int ci;
    const double t_std = isense_cal_ms(0) / 1e3;
    printf("calibration, standard test: zero, then each channel at 25 / 50 / 75 / 100 %% (%u ms, level %u + %u ms)\n",
           (unsigned)isense_cal_ms(0), (unsigned)ISENSE_CAL_LVL_SETTLE_MS, (unsigned)ISENSE_CAL_LVL_MEAS_MS);
    CHECK(isense_cal_ms(0) == ISENSE_CAL_ZERO_SETTLE_MS + ISENSE_CAL_ZERO_MEAS_MS +
                              20u * (ISENSE_CAL_LVL_SETTLE_MS + ISENSE_CAL_LVL_MEAS_MS) && isense_cal_ms(16384) == 180u,
          "calibration length");
    for (ci = 0; ci < (int)(sizeof cases / sizeof cases[0]); ci++) {
        const cal_case_t *c = &cases[ci];
        sim_t s;
        int k, l, ok = 1;
        double err_max = 0, t0, lin_err = 0, crv_err = 0;
        isense_cal_t cal;
        printf("  %s\n", c->name);
        sim_case(&s, c);
        sim_run(&s, 0.1, cont_steady);
        t0 = s.t;
        isense_cal_start(&s.is, 0);
        while (s.is.cal_step != 0 && s.t < 2.0) sim_step(&s, cont_steady);
        CHECK(fabs(s.t - t0 - t_std) < 3e-3, "%s: the test took %.1f ms", c->name, (s.t - t0) * 1e3);
        CHECK(isense_cal_get(&s.is, &cal) && cal.levels == 4u, "%s: no 4-level result", c->name);
        for (k = 0; k < NCH; k++) {
            printf("    ch %d true / measured mA:", k);
            for (l = 0; l < 4; l++) {
                double tr;
                c_ch = k; c_level = lvl[l];
                tr = true_led(c, cont_one) * 1e3;
                printf(" %5.0f/%5u", tr, (unsigned)cal.ma[k][l]);
                if (fabs(cal.ma[k][l] - tr) > fabs(err_max)) err_max = cal.ma[k][l] - tr;
                if (fabs(cal.ma[k][l] - tr) > 0.01 * tr + 8.0) ok = 0;   /* 6 + 16 ms per level fails it (slow supply) */
            }
            printf("  (100 %% / 4 x 25 %% = %.3f)\n", cal.ma[k][0] ? cal.ma[k][3] / (4.0 * cal.ma[k][0]) : 0.0);
        }
        CHECK(ok, "%s: a level off by more than 1 %% + 8 mA (worst %.0f mA)", c->name, err_max);
        /* the model with curves against a linear one from the 25 % point (the old single-level test), for mixes */
        {
            static const double mixes[4][NCH] = { { 0, 0, 0, 1.0, 0 }, { 0.6, 0.2, 0, 0.8, 0.3 },
                                                  { 1, 1, 1, 0, 0 }, { 0.4, 0.4, 0.4, 0.4, 0.4 } };
            int m;
            for (m = 0; m < 4; m++) {
                uint16_t v[NCH];
                double tr, lin = 0;
                uint32_t crv;
                for (k = 0; k < NCH; k++) {
                    v[k] = (uint16_t)(mixes[m][k] * 65535.0);
                    c_mix[k] = mixes[m][k];
                    lin += mixes[m][k] * 4.0 * cal.ma[k][0];
                }
                tr = true_led(c, cont_fixed) * 1e3;
                crv = isense_model_ma(&s.is, v);
                if (tr > 200) {
                    if (fabs(crv - tr) / tr > crv_err) crv_err = fabs(crv - tr) / tr;
                    if (fabs(lin - tr) / tr > lin_err) lin_err = fabs(lin - tr) / tr;
                }
            }
            printf("    mixes: model with curves worst %.1f %%, linear from 25 %% worst %.1f %%\n", crv_err * 100,
                   lin_err * 100);
            CHECK(crv_err < 0.06, "%s: the model with curves is off by %.1f %% for a mix", c->name, crv_err * 100);
            CHECK(crv_err <= lin_err + 0.005, "%s: the curves are no better than linear", c->name);
        }
        /* the regression keeps refining I_k through the curves: 10 s of random mixes, no drift */
        {
            int32_t before[NCH];
            int drift = 0;
            for (k = 0; k < NCH; k++) before[k] = s.is.ch_ma[k];
            mix_next = 0;
            sim_run(&s, s.t + 10.0, cont_mix);
            for (k = 0; k < NCH; k++) {
                if (abs(s.is.ch_ma[k] - before[k]) > 0.08 * before[k] + 40) drift = 1;
            }
            printf("    after 10 s of mixes: I_k %d %d %d %d %d mA (calibrated %d %d %d %d %d)\n", (int)s.is.ch_ma[0],
                   (int)s.is.ch_ma[1], (int)s.is.ch_ma[2], (int)s.is.ch_ma[3], (int)s.is.ch_ma[4], (int)before[0],
                   (int)before[1], (int)before[2], (int)before[3], (int)before[4]);
            CHECK(!drift, "%s: the regression drifted from the calibration", c->name);
        }
    }
}

/* the limiter never engages when the connected load can't reach the limit ("no limit needed") */
static void test_no_limit_needed(void)
{
    sim_t s;
    double kmin = 1.0;
    printf("no limit needed: calibrated 7.2 A connected, limit 8 A, everything at 100 %% for 1 s\n");
    sim_init(&s, 24.0, 50.0, 25000.0);
    s.spread = 1;
    set_strips(&s, 1.2, 1.2, 1.2, 2.0, 1.6);
    sim_run(&s, 0.05, cont_off);
    isense_cal_start(&s.is, 0);
    while (s.is.cal_step != 0 && s.t < 2.0) sim_step(&s, cont_off);
    printf("  connected %u mA (model)\n", (unsigned)isense_connected_ma(&s.is));
    CHECK(isense_connected_ma(&s.is) > 6800u && isense_connected_ma(&s.is) < 7600u, "connected load");
    c_level = 1.0; c_t0 = s.t;
    while (s.t < c_t0 + 1.0) {
        sim_step(&s, cont_all);
        if (s.is.k / 65536.0 < kmin) kmin = s.is.k / 65536.0;
    }
    printf("  %d mA, k min %.3f, trips %u\n", (int)s.is.i_avg, kmin, s.is.trips);
    CHECK(kmin == 1.0 && s.is.trips == 0u, "the limiter engaged below the limit");
}

static void test_regression(void)
{
    sim_t s;
    int j, ok = 1;
    printf("regression (no calibration): random colour mixes every 200 ms for 20 s, 24 V 25 kHz\n");
    sim_init(&s, 24.0, 50.0, 25000.0);
    s.spread = 1;
    set_strips(&s, 1.2, 0.7, 2.1, 3.3, 0.4);
    mix_next = 0;
    sim_run(&s, 20.0, cont_mix);
    for (j = 0; j < NCH; j++) {
        double e = s.is.ch_ma[j] - s.inom[j] * 1000.0;
        if (fabs(e) > 0.06 * s.inom[j] * 1000.0 + 40.0) ok = 0;
    }
    printf("  true %4.0f %4.0f %4.0f %4.0f %4.0f  learned %4d %4d %4d %4d %4d mA, zero %d mA, confidence %u\n",
           s.inom[0] * 1e3, s.inom[1] * 1e3, s.inom[2] * 1e3, s.inom[3] * 1e3, s.inom[4] * 1e3,
           (int)s.is.ch_ma[0], (int)s.is.ch_ma[1], (int)s.is.ch_ma[2], (int)s.is.ch_ma[3], (int)s.is.ch_ma[4],
           (int)s.is.zero_ma, s.is.conf);
    CHECK(ok, "regression error above 6 %% + 40 mA");
    CHECK(s.is.conf >= 64u, "model not trusted after 20 s");
}

static void test_preset_change(void)
{
    sim_t s;
    printf("PWM preset change 25 -> 100 -> 2.197 kHz under 7 A: no trip, no limiting\n");
    sim_init(&s, 48.0, 50.0, 25000.0);
    set_strips(&s, 1.5, 1.5, 1.5, 1.5, 1.0);
    c_level = 1.0; c_t0 = 0.02;
    sim_run(&s, 0.3, cont_all);
    s.period_nom = 1.0 / 100000.0;
    sim_run(&s, 0.6, cont_all);
    s.period_nom = 1.0 / 2197.0;
    sim_run(&s, 1.0, cont_all);
    printf("  %d mA, trips %u, k %.3f\n", (int)s.is.i_avg, s.is.trips, s.is.k / 65536.0);
    CHECK(s.is.trips == 0u && s.is.k == 65536u, "preset change disturbed the limiter");
}

static void test_limit_change(void)
{
    sim_t s;
    printf("remote limit change 8 -> 3 -> 8 A under 6 A of content\n");
    sim_init(&s, 24.0, 50.0, 25000.0);
    set_strips(&s, 1.2, 1.2, 1.2, 1.2, 1.2);
    c_level = 1.0; c_t0 = 0.02;
    sim_run(&s, 0.5, cont_all);
    isense_set_limit(&s.is, 3000);
    sim_run(&s, 1.0, cont_all);
    printf("  limit 3 A: %d mA, k %.3f\n", (int)s.is.i_avg, s.is.k / 65536.0);
    CHECK(fabs(s.is.i_avg - 3000.0) < 150.0, "3 A limit not held");
    isense_set_limit(&s.is, 8000);
    sim_run(&s, 2.0, cont_all);
    printf("  limit 8 A: %d mA, k %.3f\n", (int)s.is.i_avg, s.is.k / 65536.0);
    CHECK(s.is.k == 65536u && fabs(s.is.i_avg - 6000.0) < 200.0, "release after the limit was raised");
    CHECK(s.is.trips == 0u, "trip");
}

static void test_disabled(void)
{
    isense_t is;
    uint16_t req[NCH] = { 100, 200, 65535, 0, 7 }, out[NCH];
    int k;
    isense_init(&is, 0, 10000, 8000);
    CHECK(isense_tick(&is, 4095) == 0u, "disabled: tick trips");
    CHECK(isense_ms(&is, 4095u * 20u, 20, 4095) == 0u, "disabled: events");
    isense_apply(&is, req, out);
    for (k = 0; k < NCH; k++) CHECK(out[k] == req[k], "disabled: outputs changed");
}

/* the time to the retry after a trip (power events: POWER page retry seconds); the retry policy */
#define RMS (ISENSE_RETRY_S * 1000u)
static uint32_t trip_once(isense_t *is)                /* a short, registered: its events */
{
    isense_tick(is, 4095);
    isense_tick(is, 4095);
    return isense_ms(is, 4095u * 20u, 20, 4095);
}
static uint32_t wait_release(isense_t *is, uint32_t max_ms)   /* ms until RELEASE (max_ms + 1: none) */
{
    uint32_t ms;
    for (ms = 1; ms <= max_ms; ms++) {
        if (isense_ms(is, 0, 20, 0) & ISENSE_EV_RELEASE) {
            return ms;
        }
    }
    return max_ms + 1u;
}
static void test_retry_policy(void)
{
    isense_t is;
    uint32_t k, ev, latched = 0, ms;
    printf("retry policy: 10 retries 1 s apart, then off until a new command; 0 never, 255 forever\n");
    isense_init(&is, 1, 10000, 8000);
    isense_set_gain(&is, ISENSE_GAIN_DEFAULT_Q16);
    isense_set_limit(&is, 0);
    CHECK(is.retry_max == 10u && is.retry_s == 1u, "the default: 10 retries, 1 s apart");
    for (k = 0; k < 10u; k++) {
        ev = trip_once(&is);
        ms = wait_release(&is, 5000u);
        CHECK(!(ev & ISENSE_EV_LATCH) && ms == RMS - 1u, "retry %u 1 s after its trip (%u ms after the trip's ms)",
              k + 1u, ms);
    }
    ev = trip_once(&is);
    CHECK((ev & ISENSE_EV_LATCH) && is.latched && isense_retry_left_ms(&is) == 0xFFFFFFFFu, "the 11th trip: the "
          "retries used up, latched");
    CHECK(wait_release(&is, 120000u) == 120001u && is.tripped != ISENSE_TRIP_NONE, "latched: no retry in 2 min");
    isense_rearm(&is);
    CHECK(wait_release(&is, 5u) == 1u && is.retry_n == 1u, "a new command: the retry at the next ms, the count anew");
    for (k = 0; k < 60000u + 10u; k++) {
        (void)isense_ms(&is, 0, 20, 0);
    }
    CHECK(is.retry_n == 0u, "60 s without a trip: the count starts again (%u)", is.retry_n);
    is.retry_max = 0;                                  /* never retry */
    ev = trip_once(&is);
    CHECK((ev & ISENSE_EV_LATCH) && wait_release(&is, 5000u) == 5001u, "count 0: off at the first trip");
    isense_rearm(&is);
    (void)wait_release(&is, 5u);
    is.retry_max = 255;                                /* forever */
    is.retry_s = 3;
    for (k = 0; k < 300u; k++) {
        latched |= trip_once(&is) & ISENSE_EV_LATCH;
        if (wait_release(&is, 4000u) != 2999u) {
            break;
        }
    }
    CHECK(k == 300u && !latched, "count 255: 300 retries, each 3 s after its trip (%u)", k);
}

static void test_retry_left(void)
{
    isense_t is;
    uint32_t ms, ev = 0;
    isense_init(&is, 1, 10000, 8000);
    isense_set_gain(&is, ISENSE_GAIN_DEFAULT_Q16);
    isense_set_limit(&is, 0);
    CHECK(isense_retry_left_ms(&is) == 0u, "not tripped: no retry");
    isense_tick(&is, 4095);
    CHECK(isense_tick(&is, 4095) == 1u && isense_retry_left_ms(&is) == RMS, "short: the whole first wait");
    isense_ms(&is, 4095u * 20u, 20, 4095);         /* registered: 1 ms of the wait gone */
    CHECK(isense_retry_left_ms(&is) == RMS - 1u, "1 ms later (%u)", isense_retry_left_ms(&is));
    for (ms = 1; ms < RMS && !(ev & ISENSE_EV_RELEASE); ms++) {
        ev = isense_ms(&is, 0, 20, 0);
    }
    CHECK((ev & ISENSE_EV_RELEASE) && ms == RMS && isense_retry_left_ms(&is) == 0u, "released after 1 s");
    isense_tick(&is, 4095);
    isense_tick(&is, 4095);
    isense_ms(&is, 4095u * 20u, 20, 4095);
    CHECK(isense_retry_left_ms(&is) == RMS - 1u, "second trip: the same 1 s (no doubling) (%u)",
          isense_retry_left_ms(&is));
}

int main(void)
{
    printf("isense host test (sense %u mV / A, %.0f kHz, output <= %.2f V)\n", (unsigned)ISENSE_TEST_MV_PER_A,
           ISENSE_TEST_BW / 1e3, ISENSE_TEST_VOMAX);
    test_disabled();
    test_retry_left();
    test_retry_policy();
    test_measurement_accuracy();
    test_powerup();
    printf("legitimate steps 0 -> 100 %% (24 V, 25 kHz, spread)\n");
    test_step(6.0, 0, 0);
    test_step(12.0, 0, 0);
    test_step(12.0, 1, 0);
    test_step(19.0, 0, 1);
    test_step(19.0, 1, 0);
    printf("limiting quality (content above the 8 A limit)\n");
    test_limit_quality(cont_fade, "fade 0-100-0 % (1 s)", 0, 12.0);
    test_limit_quality(cont_fade, "fade 0-100-0 % (1 s)", 1, 12.0);
    test_limit_quality(cont_fx, "flicker effect 40-100 %", 0, 12.0);
    test_limit_quality(cont_fx, "flicker effect 40-100 %", 1, 12.0);
    test_preset_change();
    test_limit_change();
    printf("shorts on the R channel at t = 0.5 s (60 %% content, 5 A load)\n");
    test_short(24.0, 12.0, 0.25, 4e-6, "24 V, PSU 12 A, strip-end model parameter", 0);
    test_short(48.0, 15.0, 0.30, 4e-6, "48 V, PSU 15 A, strip-end model parameter", 0);
    test_short(24.0, 50.0, 0.15, 3e-6, "24 V, PSU 50 A, model parameter", 0);
    test_short(48.0, 50.0, 0.05, 1e-6, "48 V, PSU 50 A, model parameter + model parameter", 0);
    test_short(48.0, 50.0, 0.01, 0.3e-6, "48 V, PSU 50 A, bolted model parameter at output connector", 1);
    test_short(24.0, 6.0, 0.25, 4e-6, "24 V, PSU 6 A (below the limit)", 1);
    test_short(24.0, 50.0, 0.3, 0, "24 V, PSU 50 A, LED_+ -> GND model parameter", 2);
    test_short(24.0, 50.0, 2.0, 0, "24 V, PSU 50 A, LED_+ -> GND model parameter", 3);
    test_calibration();
    test_calibration_levels();
    test_no_limit_needed();
    test_regression();
    printf("%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
