/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Host tests for the portable firmware core (build: test/build_host.cmd, MSVC).
 * The decoder test models the real capture path: WS2915 / WS2805 frames with spec-wide timing, HSI error,
 * edge distortion, random sample phase, poll latency and the DMA ring copy. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/core/ack.h"
#include "../src/core/config.h"
#include "../src/core/dither.h"
#include "../src/core/presets.h"
#include "../src/core/proto.h"
#include "../src/core/ws_decode.h"
#include "../src/core/smooth.h"
#include "../src/core/gamma.h"
#include "../src/core/store.h"
#include "../src/core/cli.h"

static int g_fail;
#define CHECK(c, ...) do { if (!(c)) { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ------------------------------------------------------------------------------------------------ rng */
static unsigned long long rs = 0x9E3779B97F4A7C15ull;
static unsigned long long rnd(void) { rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17; return rs; }
static double urand(double a, double b) { return a + (b - a) * (double)(rnd() >> 11) / 9007199254740992.0; }
static unsigned rint_(unsigned n) { return (unsigned)(rnd() % n); }

/* ------------------------------------------------------------------------------------------------ waveform */
#define MAXP 4096
typedef struct { double rise[MAXP], fall[MAXP]; int n; } wave_t;   /* pulses in ns, increasing */

typedef struct {
    double t0h_min, t0h_max, t1h_min, t1h_max;   /* ns */
    double period_min, period_max;               /* ns, bit period */
    double distort;                              /* systematic high-time error per frame, +- ns */
    double hiccup_prob, hiccup_max;              /* intra-frame gaps (controller jitter), ns */
} timing_spec_t;

/* appends one frame of `nb` bits starting at time t (ns); returns time after the last bit's low part */
static double add_frame(wave_t *w, double t, const uint8_t *bits, int nb, const timing_spec_t *s, double *fall_of)
{
    int i;
    double d = urand(-s->distort, s->distort);
    for (i = 0; i < nb; i++) {
        int b = (bits[i >> 3] >> (7 - (i & 7))) & 1;
        double h = b ? urand(s->t1h_min, s->t1h_max) : urand(s->t0h_min, s->t0h_max);
        double per = urand(s->period_min, s->period_max);
        double hd = h + d + urand(-3.0, 3.0);
        if (per < h + 250.0) per = h + 250.0;
        w->rise[w->n] = t;
        w->fall[w->n] = t + hd;
        if (fall_of) fall_of[i] = t + hd;
        w->n++;
        t += per;
        if (s->hiccup_prob > 0 && urand(0, 1) < s->hiccup_prob) t += urand(0, s->hiccup_max);
    }
    return t;
}

/* sample with period ts into 16-bit words, MSB = earliest (SPI 16-bit frames); returns sample count */
static uint32_t sample_wave(const wave_t *w, double t_start, double ts, uint32_t nsamp, uint16_t *out)
{
    uint32_t k;
    int p = 0;
    memset(out, 0, ((nsamp + 15) / 16) * 2);
    for (k = 0; k < nsamp; k++) {
        double t = t_start + k * ts;
        while (p < w->n && w->fall[p] <= t) p++;
        if (p < w->n && w->rise[p] <= t) out[k >> 4] |= (uint16_t)(0x8000u >> (k & 15));
    }
    return nsamp;
}

/* ------------------------------------------------------------------------------------------------ decoder */
typedef struct { long trials, ok, err[8], wrong; } dec_stats_t;

static wave_t W;
static uint16_t SAMP[1 << 15];

/* one capture: random previous frame, gap, our frame (own bits + downstream bits), poll-latency copy */
static void decode_trial(double fs, ws_timing_t tm, const timing_spec_t *spec, uint32_t copy_samples, dec_stats_t *st)
{
    static uint8_t bits[(PROTO_MAX_BITS + 80 * 40) / 8 + 16], dec[16];
    static double fall[PROTO_MAX_BITS + 80 * 40 + 8];
    int proto = rint_(3);                 /* 0 WS2915-32, 1 WS2915-26, 2 WS2805 */
    int own = proto == 0 ? 112 : proto == 1 ? 106 : 40;
    int down = (int)rint_(20) * (proto == 2 ? 40 : 80);   /* downstream boards' bits */
    int total = own + down, i, r;
    double ts = 1e9 / (fs * (1.0 + urand(-0.022, 0.022)));   /* HSI error */
    double t, t_tick1, t_copy, phase = urand(0, ts);
    uint32_t n_end, n_start;
    long w_tick1, w_copy;
    timing_spec_t sp = *spec;
    if (proto == 2 && sp.t1h_min < 580) sp.t1h_min = 580;    /* WS2805 T1H min */

    for (i = 0; i < (total + 7) / 8 + 1; i++) bits[i] = (uint8_t)rnd();
    W.n = 0;
    t = 0;
    t = add_frame(&W, t, bits, 60, &sp, NULL);                /* tail of the previous frame */
    t += urand(285e3, 600e3);                                 /* reset gap */
    t = add_frame(&W, t, bits, total, &sp, fall);
    /* firmware model (20 kHz poll ISR, DMA position = words fully written, one word behind the shifter):
       - the tick that first sees TIM2 count > 0 (0..50 us after our first edge) notes pos - 135 words (60 us)
       - the tick after the one that sees count >= own copies from that word up to the current position */
    t_tick1 = W.rise[60] + urand(0.0, 50e3) + 0.2e3;
    t_copy = fall[own - 1] + urand(0.0, 50e3) + 50e3 + 0.2e3;
    w_tick1 = (long)floor((t_tick1 - phase) / ts / 16.0) - 1;
    w_copy = (long)floor((t_copy - phase) / ts / 16.0) - 1;
    n_start = (uint32_t)(w_tick1 - 135) * 16u;
    n_end = (uint32_t)w_copy * 16u;
    if (n_end - n_start > copy_samples) n_end = n_start + copy_samples;
    sample_wave(&W, phase + n_start * ts, ts, n_end - n_start, SAMP);

    st->trials++;
    r = ws_decode_frame(SAMP, n_end - n_start, (uint16_t)own, &tm, dec);
    if (r < 0) { st->err[-r]++; return; }
    if (memcmp(dec, bits, own / 8) != 0 || (own % 8 && ((dec[own / 8] ^ bits[own / 8]) & (0xFF00 >> (own % 8)) & 0xFF))) {
        st->wrong++;
        return;
    }
    st->ok++;
}

static void test_decoder(void)
{
    timing_spec_t spec = { 220, 380, 520, 1000, 1200, 1500, 0, 0.02, 20e3 };
    ws_timing_t t36 = WS_TIMING_DEFAULT;
    ws_timing_t t18 = { 8u, 40u, 900u };
    double dist[] = { 0, 20, 30, 40, 60 };
    int i, j;

    printf("decoder: random frames, full spec timing (T0H 220-380, T1H 520/580-1000 ns), HSI +-2.2 %%,\n"
           "         2 %% of bits followed by a controller hiccup up to 20 us, ring copied 50-100 us after our last bit\n");
    for (j = 0; j < 2; j++) {
        double fs = j ? 18e6 : 36e6;
        ws_timing_t tm = j ? t18 : t36;
        uint32_t copy = j ? 8192u : 16384u;          /* 455 us of samples */
        for (i = 0; i < 5; i++) {
            dec_stats_t st;
            long n = 20000, k;
            memset(&st, 0, sizeof st);
            spec.distort = dist[i];
            for (k = 0; k < n; k++) decode_trial(fs, tm, &spec, copy, &st);
            printf("  %2.0f MHz, edge distortion +-%2.0f ns: %ld/%ld frames ok, wrong bits %ld, errors %ld/%ld/%ld/%ld/%ld\n",
                   fs / 1e6, dist[i], st.ok, st.trials, st.wrong, st.err[1], st.err[2], st.err[3], st.err[4], st.err[5]);
            if (j == 0 && dist[i] <= 30) CHECK(st.ok == st.trials, "36 MHz decode must be error-free up to +-30 ns");
        }
    }
    /* typical controller timing (ESP32 RMT / WLED-like): T0H 300, T1H 700-800, 1.25 us */
    {
        timing_spec_t typ = { 280, 320, 680, 820, 1240, 1260, 30, 0.02, 20e3 };
        dec_stats_t st;
        long k;
        memset(&st, 0, sizeof st);
        for (k = 0; k < 50000; k++) decode_trial(36e6, t36, &typ, 16384u, &st);
        printf("  36 MHz, typical controller timing, +-30 ns distortion: %ld/%ld ok\n", st.ok, st.trials);
        CHECK(st.ok == st.trials, "typical timing must decode error-free");
    }
}

#include "test_features.c"
#include "test_ultra.c"
#include "test_stagger.c"
#include "test_boot.c"
#include "test_wire.c"
#include "test_scene.c"
#include "test_dtw.c"
#include "test_update.c"
#include "test_cal.c"
#include "test_div64.c"
#include "test_mcutemp.c"
#include "test_version.c"
#include "test_ocled.c"
#include "test_pwrev.c"
#include "test_vsense.c"

/* ------------------------------------------------------------------------------------------------ proto */
static void test_proto(void)
{
    proto_state_t s;
    uint32_t n, a, b;
    int i, both = 0;
    gate_plan_t g;

    printf("proto: classification, lock, gate plans\n");
    for (n = 0; n < 70000u; n++) {
        a = (proto_classify(n, 0) == PROTO_WS2915_32) + (proto_classify(n, 0) == PROTO_WS2915_26) + (proto_classify(n, 0) == PROTO_WS2805);
        if (a > 1) both++;
    }
    CHECK(both == 0, "frame lengths must classify uniquely");
    CHECK(proto_classify(112, 0) == PROTO_WS2915_32 && proto_classify(32 + 80 * 50, 0) == PROTO_WS2915_32, "WS2915-32");
    CHECK(proto_classify(106, 0) == PROTO_WS2915_26 && proto_classify(40, 0) == PROTO_WS2805 && proto_classify(40 * 77, 0) == PROTO_WS2805, "26/2805");
    CHECK(proto_classify(80, 0) == PROTO_WS2805 && proto_classify(111, 0) == PROTO_NONE && proto_classify(0, 0) == PROTO_NONE, "edge");

    proto_init(&s, PMODE_AUTO, PROTO_NONE);
    CHECK(proto_active(&s) == PROTO_NONE, "starts unlocked");
    proto_frame_end(&s, 192, 0); proto_frame_end(&s, 192, 0);
    CHECK(proto_active(&s) == PROTO_NONE, "needs 3 frames");
    proto_frame_end(&s, 192, 0);
    CHECK(proto_active(&s) == PROTO_WS2915_32, "locks WS2915");
    proto_frame_end(&s, 77, 0); proto_frame_end(&s, 400, 0); proto_frame_end(&s, 400, 0);
    CHECK(proto_active(&s) == PROTO_WS2915_32, "odd frame / 2 x WS2805 keep the lock");
    proto_frame_end(&s, 400, 0);
    CHECK(proto_active(&s) == PROTO_WS2805, "3 x WS2805 re-locks");
    for (i = 0; i < 2; i++) { b = 186; proto_frame_end(&s, b, 0); }
    proto_frame_end(&s, 40, 0);   /* interruption resets the candidate */
    proto_frame_end(&s, 186, 0);
    CHECK(proto_active(&s) == PROTO_WS2805, "interrupted candidate must not lock");
    proto_init(&s, PMODE_WS2915, PROTO_NONE);
    proto_frame_end(&s, 186, 0);
    CHECK(proto_active(&s) == PROTO_WS2915_26, "forced WS2915 learns the 26-bit header");

    g = proto_gate_plan(PROTO_WS2915_32);
    CHECK(g.start_open == 1 && g.t1 == 32 && g.t2 == 112 && g.own_bits == 112, "gate WS2915");
    g = proto_gate_plan(PROTO_WS2805);
    CHECK(g.start_open == 0 && g.t1 == 40 && g.t2 == 0 && g.own_bits == 40, "gate WS2805");
    g = proto_gate_plan(PROTO_NONE);
    CHECK(g.start_open == 0 && g.t1 == 0 && g.own_bits == PROTO_DETECT_BITS, "gate unlocked forwards nothing");

    /* Ultra: 64 + 256 n with a valid header; the same lengths without a header stay WS */
    CHECK(proto_classify(64 + 256, 1) == PROTO_ULTRA && proto_classify(64 + 256 * 9, 1) == PROTO_ULTRA &&
          proto_classify(64, 1) == PROTO_ULTRA, "Ultra lengths");
    CHECK(proto_classify(64 + 255, 1) == PROTO_NONE && proto_classify(112, 1) == PROTO_NONE, "Ultra header, odd length");
    CHECK(proto_classify(320, 0) == PROTO_WS2805 && proto_classify(64 + 256 * 3, 0) == PROTO_WS2915_32,
          "Ultra lengths without header are WS (why the header decides)");
    proto_init(&s, PMODE_AUTO, PROTO_NONE);
    for (i = 0; i < 3; i++) proto_frame_end(&s, 320, 0);
    CHECK(proto_active(&s) == PROTO_WS2805, "WS2805 at 320 bits");
    for (i = 0; i < 3; i++) proto_frame_end(&s, 320 + 256 * (uint32_t)i, 1);
    CHECK(proto_active(&s) == PROTO_ULTRA, "auto locks Ultra");
    proto_init(&s, PMODE_ULTRA, PROTO_NONE);
    CHECK(proto_active(&s) == PROTO_ULTRA, "forced Ultra");
    g = proto_gate_plan(PROTO_ULTRA);
    CHECK(g.start_open == 0 && g.t1 == 256 && g.t2 == 0 && g.own_bits == 256, "gate Ultra: own slot blocked, then open");
    CHECK(proto_tail_wanted(320) && proto_tail_wanted(64 + 256 * 9) && proto_tail_wanted(64) && !proto_tail_wanted(321) &&
          !proto_tail_wanted(112) && proto_tail_wanted(PROTO_UPD_BITS), "tail read for Ultra lengths only");
}

static void putbits(uint8_t *b, uint32_t pos, uint32_t v, uint32_t n)
{
    while (n--) {
        if ((v >> n) & 1u) b[pos >> 3] |= (uint8_t)(0x80u >> (pos & 7u));
        pos++;
    }
}

static void test_parse(void)
{
    uint8_t bits[PROTO_BITS_BUF];
    slot_t s;
    uint8_t gain[5] = { 31, 0, 17, 5, 22 };
    uint16_t ch[5] = { 0xFFFF, 0x0000, 0x1234, 0x8001, 0x7FFE };
    int hdr, i, ok;

    printf("parse: WS2915 32/26-bit header, WS2805\n");
    for (hdr = 0; hdr < 2; hdr++) {
        uint32_t hb = hdr ? 26u : 32u, g = 0, pos;
        memset(bits, 0, sizeof bits);
        for (i = 0; i < 5; i++) g = (g << 5) | gain[i];
        putbits(bits, 0, hdr ? (g << 1) | 1u : g << 7, hb);   /* as the ESPHome component builds it */
        pos = hb;
        for (i = 0; i < 5; i++, pos += 16) putbits(bits, pos, ch[i], 16);
        CHECK(proto_parse(hdr ? PROTO_WS2915_26 : PROTO_WS2915_32, bits, &s) == 0, "parse");
        for (ok = 1, i = 0; i < 5; i++) ok &= s.ch[i] == ch[i] && s.gain[i] == gain[i];
        CHECK(ok && s.hdr_ok && s.raw_len == 10, "WS2915 %u-bit header fields", hb);
    }
    memset(bits, 0, sizeof bits);
    for (i = 0; i < 5; i++) putbits(bits, (uint32_t)i * 8u, (uint32_t)(ch[i] >> 8), 8);
    CHECK(proto_parse(PROTO_WS2805, bits, &s) == 0, "parse 2805");
    for (ok = 1, i = 0; i < 5; i++) ok &= s.ch[i] == (uint16_t)((ch[i] >> 8) * 257u) && s.raw[i] == (uint8_t)(ch[i] >> 8);
    CHECK(ok && s.raw_len == 5, "WS2805 8-bit * 257");
}

/* ------------------------------------------------------------------------------------------------ config */
static void slot_from_raw(slot_t *s, const uint8_t *raw, int ws2915)
{
    int i;
    s->raw_len = (uint8_t)(ws2915 ? 10 : 5);
    for (i = 0; i < s->raw_len; i++) s->raw[i] = raw[i];
}

static void test_config(void)
{
    settings_t st;
    config_rx_t rx;
    slot_t sl, normal;
    uint8_t raw[10];
    cfg_result_t r;
    int i, mode;
    long fp = 0, held = 0, n;

    printf("config: 40-bit message, 3 in a row, same / NACK, re-trigger rule, false positives\n");
    memset(&normal, 0, sizeof normal);
    normal.raw_len = 5;
    for (mode = 0; mode < 2; mode++) {
        settings_default(&st);
        config_rx_init(&rx);
        normal.raw_len = (uint8_t)(mode ? 10 : 5);
        config_build(CMD_PWM_PRESET, 2, (uint8_t)mode, raw);
        slot_from_raw(&sl, raw, mode);
        r = config_process(&rx, &st, &sl); CHECK(r.event == CFG_PENDING, "1st frame pending");
        r = config_process(&rx, &st, &sl); CHECK(r.event == CFG_PENDING, "2nd frame pending");
        r = config_process(&rx, &st, &sl); CHECK(r.event == CFG_ACK && r.value_code == 3 && st.pwm_preset == 2, "3rd frame ACK");
        for (i = 0; i < 10; i++) { r = config_process(&rx, &st, &sl); CHECK(r.event == CFG_PENDING, "repeats do not re-trigger"); }
        r = config_process(&rx, &st, &normal); CHECK(r.event == CFG_NONE, "normal frame");
        for (i = 0; i < 3; i++) r = config_process(&rx, &st, &sl);
        CHECK(r.event == CFG_SAME, "same value after a normal frame -> SAME");
        config_build(CMD_PWM_PRESET, 15, (uint8_t)mode, raw);
        slot_from_raw(&sl, raw, mode);
        for (i = 0; i < 3; i++) r = config_process(&rx, &st, &sl);
        CHECK(r.event == CFG_NACK && st.pwm_preset == 2, "out of range -> NACK, unchanged");
        config_build(CMD_PERIOD_HI, 0x30, (uint8_t)mode, raw); slot_from_raw(&sl, raw, mode);
        for (i = 0; i < 3; i++) r = config_process(&rx, &st, &sl);
        config_build(CMD_PERIOD_LO, 0x00, (uint8_t)mode, raw); slot_from_raw(&sl, raw, mode);
        for (i = 0; i < 3; i++) r = config_process(&rx, &st, &sl);
        CHECK(r.event == CFG_ACK && st.pwm_preset == 0xFF && settings_steps(&st) == 0x3000, "raw period");
        if (mode) {
            config_build(CMD_DITHER_MODE, 1, 1, raw);
            raw[9] ^= 1;                                   /* complement check */
            slot_from_raw(&sl, raw, 1);
            for (i = 0; i < 3; i++) r = config_process(&rx, &st, &sl);
            CHECK(r.event == CFG_NONE, "WS2915: broken complement is a normal frame");
        }
    }
    /* false positives: random slots (fades) and random static colours */
    settings_default(&st);
    config_rx_init(&rx);
    for (n = 0; n < 20000000L; n++) {
        for (i = 0; i < 5; i++) normal.raw[i] = (uint8_t)rnd();
        normal.raw_len = 5;
        r = config_process(&rx, &st, &normal);
        if (r.event == CFG_PENDING) held++;
        else if (r.event != CFG_NONE) fp++;
    }
    printf("  random WS2805 slots, 20M frames: %ld applied a setting, %ld single frames held the outputs\n"
           "  (MAGIC + CRC match ~1 in 16.7M frames; applying needs 3 identical frames in a row)\n", fp, held);
    CHECK(fp == 0, "false positives");
}

/* ------------------------------------------------------------------------------------------------ ack */
static uint32_t run_ack(ack_kind_t k, uint8_t code, uint32_t *on_ms, uint32_t *flashes)
{
    ack_t a;
    uint8_t on, prev = 0;
    uint32_t ms = 0;
    *on_ms = 0;
    *flashes = 0;
    ack_start(&a, k, code);
    while (ack_tick(&a, &on)) {
        ms++;
        if (on) (*on_ms)++;
        if (on && !prev) (*flashes)++;
        prev = on;
        if (ms > 100000) break;
    }
    return ms;
}

static void test_ack(void)
{
    uint32_t on, fl, ms;
    printf("ack: pattern lengths\n");
    ms = run_ack(ACK_P_ACK, 9, &on, &fl);    /* preset 8 (25 kHz) = 1 long + 4 short */
    printf("  ACK + code 9 (1 long + 4 short): %u ms, %u flashes, %u ms on\n", ms, fl, on);
    CHECK(fl == 3 + 1 + 4 && on == 300 + 700 + 4 * 200, "ACK code 9");
    ms = run_ack(ACK_P_ACK, 15, &on, &fl);
    CHECK(fl == 3 + 3 && on == 300 + 3 * 700, "ACK code 15 = 3 long");
    printf("  ACK + code 15 (3 long): %u ms\n", ms);
    ms = run_ack(ACK_P_NACK, 0, &on, &fl);
    CHECK(fl == 1 && on == 1500, "NACK");
    ms = run_ack(ACK_P_SAME, 0, &on, &fl);
    CHECK(fl == 1 && on == 200, "SAME");
    ms = run_ack(ACK_P_FACTORY, 0, &on, &fl);
    CHECK(fl == 6, "factory = ACK twice");
    ms = run_ack(ACK_P_IDENTIFY, 0, &on, &fl);
    printf("  identify stops after %u ms\n", ms);
    CHECK(ms >= 30000 && ms <= 32000, "identify 30 s");
}

int main(void)
{
    test_proto();
    test_ftrack();
    test_sticky_lock();
    test_wire();
    test_scene();
    test_dtw();
    test_parse();
    test_config();
    test_ack();
    test_dither();
    test_floor();
    test_smooth();
    test_gamma();
    test_store();
    test_store_ext();
    test_isense_config();
    test_cli();
    test_ultra();
    test_stagger();
    test_spread();
    test_decoder();
    test_update();
    test_calstore();
    test_isense_curves();
    test_chain_calibrate();
    test_climit();
    test_div64();
    test_dark();
    test_gamma_exact();
    test_mcutemp();
    test_version();
    test_ocled();
    test_pwrev();
    test_vsense();
    test_rxpower();
    printf("\n%s (%d failures)\n", g_fail ? "FAILED" : "ALL TESTS PASSED", g_fail);
    return g_fail ? 1 : 0;
}
