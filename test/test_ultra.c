/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "removed_analog.h"
/* Ultra mode tests (included by test_main.c): protocol, exact gamma, engine vs ESPHome's formulas, transitions,
 * effects, power limit, clock sync of two boards with HSI error, long uptime, end-to-end waveform decode. */
#include "../src/core/ultra.h"
#include "../src/target.h"
#include "../src/core/frame_rx.h"

static ultra_t U;
static uint8_t useq = 1;
static const uint32_t HOST0 = ULTRA_TIME_WRAP - 0x100000u;   /* host TIME at board ms 0: wraps after ~1 s (unwrap test) */

/* ideal board: 20 ticks per ms, host clock = board clock */
static uint32_t tk(uint32_t ms) { return ms * 20u + 7u; }
static void hdr(ultra_t *u, uint32_t ms) { ultra_on_header(u, tk(ms), ultra_time_wire((uint64_t)HOST0 + ms * 1000ull)); }
static uint32_t send(ultra_t *u, uint32_t ms, const uint8_t *slot)
{
    hdr(u, ms);
    return ultra_on_slot(u, slot, tk(ms));
}
static void rend(ultra_t *u, uint32_t ms, uint16_t *o) { ultra_render(u, tk(ms), o); }

static uint16_t f16(double v) { return (uint16_t)(v <= 0 ? 0 : v >= 1 ? 65535 : floor(v * 65535 + 0.5)); }

static void def_light(ultra_t *u, uint32_t ms, uint8_t light, const uint8_t *roles, double gamma, uint8_t flags,
                      double mn, double mx)
{
    uint8_t s[32];
    uint16_t a[5], b[5];
    int i;
    for (i = 0; i < 5; i++) { a[i] = f16(mn); b[i] = f16(mx); }
    ultra_slot_light_def(s, light, useq++, roles, (uint16_t)(gamma * 1000 + 0.5), flags, a, b, 0);
    send(u, ms, s);
}

static void set_light(ultra_t *u, uint32_t ms, uint8_t light, uint8_t flags, double m, const double *a,
                      uint32_t dur, int16_t start)
{
    uint8_t s[32];
    uint16_t av[5], bv[5];
    int i;
    for (i = 0; i < 5; i++) { av[i] = f16(a[i]); bv[i] = 65535; }
    ultra_slot_set(s, light, useq++, flags, f16(m), av, bv, dur, start);
    send(u, ms, s);
}

static double gref(double x, double g) { return x <= 0 ? 0 : pow(x, g) * 65535.0; }

/* ------------------------------------------------------------------------------------------------ protocol */
static void test_ultra_protocol(void)
{
    uint8_t h[8], s[32];
    uint32_t t = 0, i, bad = 0;
    printf("ultra: protocol builders / checks\n");
    ultra_build_header(h, 0x12345678u);
    CHECK(ultra_parse_header(h, &t) == 0 && t == 0x12345678u, "header roundtrip");
    for (i = 0; i < 64; i++) {
        h[i >> 3] ^= (uint8_t)(0x80u >> (i & 7));
        if (ultra_parse_header(h, &t) == 0) bad++;
        h[i >> 3] ^= (uint8_t)(0x80u >> (i & 7));
    }
    CHECK(bad == 0, "any single bit error in the header is rejected");
    {   /* TIME counts modulo ULTRA_TIME_WRAP (2^22 x 1000 us): a header at or over it is invalid, with a right CRC */
        static const uint32_t ok_t[] = { 0u, 999u, ULTRA_TIME_WRAP - 1u }, bad_t[] = { ULTRA_TIME_WRAP, ULTRA_TIME_WRAP + 1u, 0xFFFFFFFFu };
        uint64_t x = 0x9E3779B97F4A7C15ull;
        for (i = 0, bad = 0; i < 3u; i++) {
            ultra_build_header(h, ok_t[i]);
            bad += !(ultra_parse_header(h, &t) == 0 && t == ok_t[i]);
            ultra_build_header(h, bad_t[i]);
            t = 12345u;
            bad += !(ultra_parse_header(h, &t) != 0 && t == 12345u);
        }
        CHECK(bad == 0 && ULTRA_TIME_WRAP == 4194304000u && ULTRA_TIME_WRAP % 1000u == 0u,
              "TIME >= ULTRA_TIME_WRAP: an invalid header (%u wrong)", bad);
        for (i = 0, bad = 0; i < 200000u; i++) {           /* the 32-bit-division helpers against 64-bit arithmetic */
            uint64_t us = i < 64u ? (i < 32u ? (uint64_t)ULTRA_TIME_WRAP * i + (i & 1u ? 0u : ULTRA_TIME_WRAP - 1u)
                                             : ~0ull - (i - 32u) - ULTRA_TIME_WRAP) : (x = x * 6364136223846793005ull + 1442695040888963407ull) >> (i % 40u);
            uint32_t a = ultra_time_wire(us), k = (uint32_t)(x >> 33) % (ULTRA_TIME_WRAP / 2u);
            uint32_t b = ultra_time_wire(us + k);
            bad += a != (uint32_t)(us % ULTRA_TIME_WRAP);
            bad += ultra_time_add(a, k) != b;
            bad += ultra_time_sub(b, a) != (int32_t)k || ultra_time_sub(a, b) != -(int32_t)k;
        }
        CHECK(bad == 0, "ultra_time_wire / _add / _sub exact (%u wrong)", bad);
    }
    ultra_slot_nop(s);
    CHECK(ultra_slot_check(s) == 0, "slot CRC");
    for (i = 0, bad = 0; i < 256; i++) {
        s[i >> 3] ^= (uint8_t)(0x80u >> (i & 7));
        if (ultra_slot_check(s) == 0) bad++;
        s[i >> 3] ^= (uint8_t)(0x80u >> (i & 7));
    }
    CHECK(bad == 0, "any single bit error in a slot is rejected");
    CHECK(ultra_crc16((const uint8_t *)"123456789", 9) == 0x29B1u, "CRC-16/CCITT-FALSE check value");
    CHECK(ultra_crc8((const uint8_t *)"123456789", 9) == 0xF4u, "CRC-8/SMBus check value");
}

/* ------------------------------------------------------------------------------------------------ shared starts */
/* One shared start (light 0 on, light 1 off), its two items in two frames, on two boards: one synchronized at a host epoch
 * hours (or wraps) before, one fresh. For every phase of the frames, gap and start: both items land on the same host ms on both
 * boards (t0, and the first rendered tick), the ms the host rounded the start to; a board's ms count is the host's minus whole
 * TIME wraps (2^22 ms each) */
static uint32_t ep_tick(uint64_t boot, uint64_t t) { return (uint32_t)((t - boot) / 50u) + 7u; }
static void test_ultra_epochs(void)
{
    static const uint8_t r0[5] = { ULTRA_ROLE_BRIGHTNESS, 0, 0, 0, 0 }, r1[5] = { 0, ULTRA_ROLE_BRIGHTNESS, 0, 0, 0 };
    static const uint64_t boots[] = { 0ull, 5000000000ull, 10000000000ull, 2ull * ULTRA_TIME_WRAP - 2000000ull,
                                      7ull * 4294967296ull + 123457ull, 1000000000000ull };
    static ultra_t base[2], u[2];
    const uint16_t lo[5] = { 0, 0, 0, 0, 0 }, hi[5] = { 65535, 65535, 65535, 65535, 65535 };
    uint8_t s[32];
    uint32_t e, ph, i, bad_t0 = 0, bad_tick = 0, bad_bound = 0, bad_cross = 0, runs = 0;
    int64_t worst = 0;
    printf("ultra: shared start across frames and boot epochs: two boards (one booted at the epoch, one fresh), 1000 phases\n");
    for (e = 0; e < sizeof boots / sizeof boots[0]; e++) {
        const uint64_t b0 = boots[e], T = b0 + 3600000000ull + 777u;   /* the test frames an hour after board 0's boot */
        const uint64_t b1 = T - 1500000u;                              /* board 1 boots 1.5 s before them */
        uint64_t t, boot[2];
        boot[0] = b0; boot[1] = b1;
        for (i = 0; i < 2u; i++) {
            ultra_init(&base[i], 3u + i);
            for (t = boot[i]; t < T - 200000u; t += (t < boot[i] + 2000000u ? 100000u : 600000000u)) {
                ultra_on_header(&base[i], ep_tick(boot[i], t), ultra_time_wire(t));   /* headers keep it synced */
            }
            ultra_slot_light_def(s, 0, 1, r0, 1000, 0, lo, hi, 0);
            ultra_on_slot(&base[i], s, ep_tick(boot[i], t));
            ultra_slot_light_def(s, 1, 1, r1, 1000, 0, lo, hi, 0);
            ultra_on_slot(&base[i], s, ep_tick(boot[i], t));
            ultra_slot_set(s, 1, 2, 0, 65535, hi, hi, 0, 0);               /* light 1 on, light 0 dark */
            ultra_on_slot(&base[i], s, ep_tick(boot[i], t));
        }
        for (ph = 0; ph < 1000u; ph++) {
            const uint64_t ta = T + ph, tb = ta + 500u + (ph * 7919u) % 30000u, start = ta + 200000u + (ph * 104729u) % 1000u;
            const int64_t sm = (int64_t)((start + 500u) / 1000u);           /* the host's ms of the start, once */
            uint32_t want[2], first[2][2];
            int64_t dt[2];
            for (i = 0; i < 2u; i++) {
                const uint32_t tka = ep_tick(boot[i], ta) + ph % 3u, tkb = ep_tick(boot[i], tb) + (ph >> 2) % 3u;
                uint32_t k, tick;
                uint16_t o[5];
                u[i] = base[i];
                ultra_on_header(&u[i], tka, ultra_time_wire(ta));
                ultra_slot_set(s, 0, 3, 0, 65535, hi, hi, 0, (int16_t)(sm - (int64_t)(ta / 1000u)));
                ultra_on_slot(&u[i], s, tka);
                ultra_on_header(&u[i], tkb, ultra_time_wire(tb));
                ultra_slot_set(s, 1, 3, 0, 0, hi, hi, 0, (int16_t)(sm - (int64_t)(tb / 1000u)));
                ultra_on_slot(&u[i], s, tkb);
                want[i] = (uint32_t)((uint64_t)sm - (boot[i] / ULTRA_TIME_WRAP) * 4194304ull);
                bad_t0 += u[i].light[0].cur.t0 != want[i] || u[i].light[1].cur.t0 != want[i];
                first[i][0] = first[i][1] = 0;
                for (k = 0; k < 2000u; k++) {                               /* the renderer, tick by tick */
                    tick = ep_tick(boot[i], start) - 400u + k;
                    ultra_render(&u[i], tick, o);
                    if (!first[i][0] && o[0] != 0u) first[i][0] = tick;
                    if (!first[i][1] && o[1] == 0u) first[i][1] = tick;
                }
                bad_tick += first[i][0] == 0u || first[i][0] != first[i][1];
                dt[i] = (int64_t)((uint64_t)(first[i][0] - 7u) * 50u + boot[i]) - (int64_t)start;   /* its true host time - start */
                worst = dt[i] > worst ? dt[i] : -dt[i] > worst ? -dt[i] : worst;
            }
            bad_bound += sm * 1000 - (int64_t)start < -500 || sm * 1000 - (int64_t)start > 499;
            bad_cross += dt[0] - dt[1] > 150 || dt[1] - dt[0] > 150;          /* a tick (50) + the header dating (0..100) */
            runs++;
        }
    }
    printf("  %u runs x 2 boards: t0 wrong %u, items on different ticks %u, start ms outside -500 .. +499 us %u, boards more than "
           "150 us apart %u; worst first tick vs the start %lld us\n", runs, bad_t0, bad_tick, bad_bound, bad_cross, (long long)worst);
    CHECK(bad_t0 == 0u && bad_tick == 0u && bad_bound == 0u && bad_cross == 0u && worst <= 650,
          "one shared start, one host ms on every board whatever its epoch");
}

/* ------------------------------------------------------------------------------------------------ gamma */
static void test_ultra_gamma(void)
{
    static const double gs[] = { 1.0, 0.45, 1.8, 2.2, 2.8, 3.0 };
    uint32_t gi, m, prev, mono = 1;
    printf("ultra: exact gamma (log2 / exp2 polynomials) vs pow()\n");
    for (gi = 0; gi < sizeof gs / sizeof gs[0]; gi++) {
        uint32_t g = (uint32_t)(gs[gi] * 65536 + 0.5);
        double maxe = 0;
        long k;
        for (k = 0; k < 400000; k++) {
            uint32_t a = (uint32_t)(rnd() & 0xFFFF), b = (uint32_t)(rnd() & 0xFFFF), x = a * b;
            double ref, e;
            if (k < 65536) { a = (uint32_t)k; b = 65535; x = a * b; }
            x += x >> 15;
            ref = gref((double)a * b / (65535.0 * 65535.0), gs[gi]);
            e = fabs((double)ultra_gamma(x, g) - ref);
            if (e > maxe) maxe = e;
        }
        prev = 0;
        for (m = 0; m < 65536; m++) {                  /* a brightness sweep must never step back */
            uint32_t x = m * 65535u, y;
            x += x >> 15;
            y = ultra_gamma(x, g);
            if (y < prev) mono = 0;
            prev = y;
        }
        printf("  gamma %.2f: max error %.2f counts of 65535, full scale %u\n", gs[gi], maxe, prev);
        CHECK(maxe < 1.0, "gamma %.2f error %.2f", gs[gi], maxe);
        CHECK(prev == 65535u, "full scale");
    }
    CHECK(mono, "monotonic");
    {
        uint32_t x = 1u * 65535u, y;   /* M = 1/65535 at A = 1: ~1.6e-14 of full scale at 2.8 */
        x += x >> 15;
        y = ultra_gamma(x, 183501u);
        printf("  lowest step (M = 1/65535, gamma 2.8) -> %u (clamped to 1 by the engine: on stays on)\n", y);
    }
}

/* ------------------------------------------------------------------------------------------------ engine */
static void test_ultra_engine(void)
{
    static const uint8_t rgbw[5] = { ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE, ULTRA_ROLE_WHITE, 0 };
    static const uint8_t mono[5] = { 0, 0, 0, 0, ULTRA_ROLE_BRIGHTNESS };
    static const uint8_t cwww[5] = { ULTRA_ROLE_COLD_WHITE, ULTRA_ROLE_WARM_WHITE, 0, 0, 0 };
    uint16_t o[5];
    double maxe = 0;
    int k, i;
    uint32_t ms = 10;

    printf("ultra: engine outputs vs ESPHome's formulas (exact pow), min / max power\n");
    ultra_init(&U, 1);
    def_light(&U, ms, 0, rgbw, 2.8, 0, 0, 1);
    def_light(&U, ms, 1, mono, 2.8, 0, 0, 1);
    for (k = 0; k < 2000; k++) {
        double m = urand(0, 1), a[5], mm = urand(0, 1), one[5] = { 1, 1, 1, 1, 1 };
        for (i = 0; i < 5; i++) a[i] = urand(0, 1);
        set_light(&U, ms, 0, 0, m, a, 0, 0);
        set_light(&U, ms, 1, 0, mm, one, 0, 0);
        rend(&U, ms, o);
        for (i = 0; i < 4; i++) {
            double ref = gref((double)f16(m) * f16(a[i]) / (65535.0 * 65535.0), 2.8);
            if (ref > 0 && ref < 1) ref = 1;
            if (fabs(o[i] - ref) > maxe) maxe = fabs(o[i] - ref);
        }
        {
            double ref = gref(f16(mm) / 65535.0, 2.8);
            if (fabs(o[4] - ref) > maxe) maxe = fabs(o[4] - ref);
        }
        ms++;
    }
    printf("  RGBW + monochromatic, gamma 2.8: max error %.2f counts\n", maxe);
    CHECK(maxe <= 1.0, "engine RGBW error %.2f", maxe);

    /* RGB with color_brightness in B (inside gamma, interpolated separately like ESPHome) */
    maxe = 0;
    for (k = 0; k < 2000; k++) {
        uint8_t s[32];
        uint16_t av[5], bv[5];
        double m = urand(0, 1), cb = urand(0, 1), a[5];
        for (i = 0; i < 5; i++) { a[i] = urand(0, 1); av[i] = f16(a[i]); bv[i] = i < 3 ? f16(cb) : 65535; }
        ultra_slot_set(s, 0, useq++, 0, f16(m), av, bv, 0, 0);
        send(&U, ms, s);
        rend(&U, ms, o);
        for (i = 0; i < 3; i++) {
            double ref = gref((double)f16(m) * f16(a[i]) * f16(cb) / (65535.0 * 65535.0 * 65535.0), 2.8);
            if (ref > 0 && ref < 1) ref = 1;
            if (fabs(o[i] - ref) > maxe) maxe = fabs(o[i] - ref);
        }
        ms++;
    }
    printf("  RGB x color brightness: max error %.2f counts\n", maxe);
    CHECK(maxe <= 1.5, "color brightness error %.2f", maxe);

    /* constant brightness CW / WW: g(M) x max(gcw, gww) x g(this) / (gcw + gww) */
    ultra_init(&U, 2);
    def_light(&U, ms, 0, cwww, 2.8, ULTRA_DEF_CONSTANT_BRIGHTNESS, 0, 1);
    maxe = 0;
    for (k = 0; k < 2000; k++) {
        double m = urand(0, 1), a[5] = { urand(0, 1), urand(0, 1), 0, 0, 0 }, gm, gc, gw, sum;
        set_light(&U, ms, 0, 0, m, a, 0, 0);
        rend(&U, ms, o);
        gm = pow(f16(m) / 65535.0, 2.8); gc = pow(f16(a[0]) / 65535.0, 2.8); gw = pow(f16(a[1]) / 65535.0, 2.8);
        sum = gc + gw;
        if (sum > 0) {
            double rc = gm * (gc > gw ? gc : gw) * gc / sum * 65535, rw = gm * (gc > gw ? gc : gw) * gw / sum * 65535;
            if (fabs(o[0] - rc) > maxe && rc >= 1) maxe = fabs(o[0] - rc);
            if (fabs(o[1] - rw) > maxe && rw >= 1) maxe = fabs(o[1] - rw);
        }
        ms++;
    }
    printf("  CW / WW constant_brightness: max error %.2f counts\n", maxe);
    CHECK(maxe <= 3.0, "constant brightness error %.2f", maxe);

    /* min / max power, zero_means_zero */
    {
        double z[5] = { 0, 0, 0, 0, 0 }, full[5] = { 1, 1, 1, 1, 1 };
        ultra_init(&U, 3);
        def_light(&U, ms, 1, mono, 1.0, 0, 0.1, 0.9);
        set_light(&U, ms, 1, 0, 0, z, 0, 0);
        rend(&U, ms, o);
        CHECK(o[4] == f16(0.1), "off = min_power without zero_means_zero (%u)", o[4]);
        set_light(&U, ms, 1, 0, 1, full, 0, 0);
        rend(&U, ms, o);
        CHECK(abs((int)o[4] - (int)f16(0.9)) <= 1, "full = max_power (%u)", o[4]);
        set_light(&U, ms, 1, 0, 0.5, full, 0, 0);
        rend(&U, ms, o);
        CHECK(abs((int)o[4] - (int)f16(0.5)) <= 1, "half = min + 0.5 (max - min) (%u)", o[4]);
        def_light(&U, ms, 1, mono, 1.0, ULTRA_DEF_ZERO_MEANS_ZERO, 0.1, 0.9);
        set_light(&U, ms, 1, 0, 0, z, 0, 0);
        rend(&U, ms, o);
        CHECK(o[4] == 0, "zero_means_zero");
        printf("  min / max power and zero_means_zero ok\n");
    }

    /* fade resolution: 10 s 0 -> 100 % at gamma 2.8, one output per ms */
    {
        double z[5] = { 0, 0, 0, 0, 0 }, full[5] = { 1, 1, 1, 1, 1 };
        uint32_t t, distinct = 0, maxstep = 0, first = 0;
        uint16_t prev = 0;
        ultra_init(&U, 4);
        def_light(&U, ms, 1, mono, 2.8, 0, 0, 1);
        set_light(&U, ms, 1, 0, 0, z, 0, 0);
        set_light(&U, ms + 1, 1, 0, 1, full, 10000, 0);
        for (t = 0; t <= 10000; t++) {
            rend(&U, ms + 1 + t, o);
            if (o[4] != prev) {
                distinct++;
                if (!first) first = o[4];
                if ((uint32_t)(o[4] - prev) > maxstep) maxstep = o[4] - prev;
            }
            prev = o[4];
        }
        printf("  10 s linear fade 0 -> 100 %%, gamma 2.8: %u output changes in 10000 ms, first level %u, "
               "largest step %u (%.3f %%), ends at %u\n", distinct, first, maxstep, maxstep * 100.0 / 65535, prev);
        CHECK(prev == 65535 && maxstep < 40, "fade resolution");
    }
}

/* ------------------------------------------------------------------------------------------------ transitions */
static void test_ultra_transitions(void)
{
    static const uint8_t rgb[5] = { ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE, 0, 0 };
    double red[5] = { 1, 0, 0, 0, 0 }, blue[5] = { 0, 0, 1, 0, 0 };
    uint16_t o[5];
    uint32_t ms = 100, t;
    double maxe = 0;

    printf("ultra: transitions (linear / smoothstep, off -> on, via off, scheduled, late, seq)\n");
    ultra_init(&U, 5);
    def_light(&U, ms, 0, rgb, 1.0, 0, 0, 1);         /* gamma 1: outputs = M x A */
    set_light(&U, ms, 0, 0, 0, red, 0, 0);
    ms += 10;
    set_light(&U, ms, 0, ULTRA_SET_SMOOTH, 1, red, 1000, 0);
    for (t = 0; t <= 1000; t += 10) {
        double p = t / 1000.0, s = p * p * p * (p * (p * 6 - 15) + 10);
        rend(&U, ms + t, o);
        if (fabs(o[0] - s * 65535) > maxe) maxe = fabs(o[0] - s * 65535);
    }
    printf("  smoothstep 1 s: max error vs 6p^5 - 15p^4 + 10p^3: %.1f counts\n", maxe);
    CHECK(maxe < 4, "smoothstep");
    ms += 1100;

    /* color change on: linear red -> blue */
    set_light(&U, ms, 0, 0, 1, blue, 1000, 0);
    rend(&U, ms + 500, o);
    CHECK(abs((int)o[0] - 32768) < 40 && abs((int)o[2] - 32768) < 40, "linear crossfade midpoint %u %u", o[0], o[2]);
    ms += 1100;

    /* off -> on: colors jump, only M fades */
    set_light(&U, ms, 0, 0, 0, blue, 0, 0);
    ms += 10;
    set_light(&U, ms, 0, 0, 1, red, 1000, 0);
    rend(&U, ms + 500, o);
    CHECK(abs((int)o[0] - 32768) < 40 && o[2] == 0, "off -> on: new color from the start (%u %u)", o[0], o[2]);
    ms += 1100;

    /* via off (color mode change): red down to 0 in the first half, blue up in the second */
    set_light(&U, ms, 0, ULTRA_SET_VIA_OFF, 1, blue, 1000, 0);
    rend(&U, ms + 250, o);
    CHECK(abs((int)o[0] - 32768) < 40 && o[2] == 0, "via off, 1/4: old color half (%u %u)", o[0], o[2]);
    rend(&U, ms + 500, o);
    CHECK(o[0] == 0 && o[2] == 0, "via off, 1/2: dark");
    rend(&U, ms + 750, o);
    CHECK(o[0] == 0 && abs((int)o[2] - 32768) < 40, "via off, 3/4: new color half (%u %u)", o[0], o[2]);
    ms += 1100;

    /* scheduled: start 100 ms after the header; late copy: started 300 ms ago */
    set_light(&U, ms, 0, 0, 1, red, 1000, 100);
    rend(&U, ms + 50, o);
    CHECK(o[2] == 65535 && o[0] == 0, "before the start time the old state holds");
    rend(&U, ms + 600, o);
    CHECK(abs((int)o[0] - 32768) < 40, "half way 500 ms after the start (%u)", o[0]);
    ms += 1500;
    set_light(&U, ms, 0, 0, 1, blue, 1000, -300);
    rend(&U, ms, o);
    CHECK(abs((int)o[2] - f16(0.3)) < 40, "late copy catches up: 30 %% at receipt (%u)", o[2]);
    ms += 1100;

    /* repeats of a command (same seq) are not applied again, even with other values */
    {
        uint8_t s[32];
        uint16_t av[5] = { 65535, 0, 0, 0, 0 }, bv[5] = { 65535, 65535, 65535, 65535, 65535 };
        ultra_slot_set(s, 0, 200, 0, 65535, av, bv, 0, 0);
        CHECK((send(&U, ms, s) & ULTRA_EV_LIGHT), "first copy applied");
        av[0] = 0; av[1] = 65535;
        ultra_slot_set(s, 0, 200, 0, 65535, av, bv, 0, 0);
        CHECK(send(&U, ms + 1, s) == 0, "same seq ignored");
        rend(&U, ms + 2, o);
        CHECK(o[0] == 65535 && o[1] == 0, "still the first command");
    }
    printf("  linear, off -> on, via off, scheduled start, late catch-up, seq dedupe ok\n");
}

/* ------------------------------------------------------------------------------------------------ effects */
static void fx_send(ultra_t *u, uint32_t ms, uint8_t light, uint8_t fx, const uint8_t *p, uint32_t n)
{
    uint8_t s[32];
    ultra_slot_effect(s, light, useq++, fx, 0, p, n);
    send(u, ms, s);
}

static void test_ultra_effects(void)
{
    ultra_t before; uint8_t slot[32]; uint32_t t, l, seq;
    static const uint8_t removed[] = {3,4,10};
    ultra_init(&U, 123); before=U;
    for (t=0;t<sizeof(removed);++t) for (l=0;l<256;++l) for (seq=0;seq<4;++seq) {
        memset(slot,0xA5,sizeof(slot)); slot[0]=(uint8_t)(removed[t] | (seq << 5)); slot[1]=(uint8_t)l; slot[2]=(uint8_t)seq;
        ultra_slot_seal(slot);
        CHECK(ultra_on_slot(&U,slot,0)==0 && !memcmp(&U,&before,sizeof(U)), "removed opcode %u mutates state or SEQ",removed[t]);
    }
}


/* ------------------------------------------------------------------------------------------------ power / config */
static void test_ultra_power(void)
{
    static const uint8_t five[5] = { ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE, ULTRA_ROLE_COLD_WHITE,
                                     ULTRA_ROLE_WARM_WHITE };
    double full[5] = { 1, 1, 1, 1, 1 };
    uint8_t s[32], ids[9];
    uint16_t vals[9], o[5];
    int i;
    uint32_t ms = 50;

    printf("ultra: CONFIG, power limit per channel / board\n");
    ultra_init(&U, 8);
    def_light(&U, ms, 0, five, 1.0, 0, 0, 1);
    set_light(&U, ms, 0, 0, 1, full, 0, 0);
    for (i = 0; i < 5; i++) {
        ids[0]=ULTRA_CFG_CH_CURRENT; vals[0]=1000;
        ids[1]=ULTRA_CFG_CH_MAX; vals[1]=i ? 3000 : 500;
        ultra_slot_config(s,(uint8_t)i,useq++,ids,vals,2); send(&U,ms,s);
    }
    ids[0] = ULTRA_CFG_PWM_PRESET; vals[0] = 3;
    ultra_slot_config(s,ULTRA_TARGET_BOARD, useq, ids, vals, 1);
    CHECK(send(&U, ms, s) == ULTRA_EV_CONFIG && (U.cfg_changed & (1u << ULTRA_CFG_PWM_PRESET)) && U.cfg[ULTRA_CFG_PWM_PRESET] == 3,
          "CONFIG applies");
    CHECK(send(&U, ms, s) == 0, "CONFIG repeat ignored");
    U.cfg_changed = 0;
    ultra_slot_config(s,ULTRA_TARGET_BOARD, ++useq, ids, vals, 1);
    CHECK(send(&U, ms, s) == 0 && U.cfg_changed == 0, "same values, new seq: no change event");
    useq++;
    rend(&U, ms, o);
    CHECK(abs((int)o[0] - 32767) <= 1 && o[1] == 65535, "channel limit 500 of 1000 mA (%u)", o[0]);
    CHECK(U.ch_limited, "a channel cap: CH_LIMITING");
    ids[0] = ULTRA_CFG_BOARD_MAX; vals[0] = 2250;              /* 500 + 4 x 1000 = 4500 mA configured */
    ultra_slot_config(s,ULTRA_TARGET_BOARD, useq++, ids, vals, 1);
    send(&U, ms, s);
    printf("  limits ok (%u %u %u %u %u)\n", o[0], o[1], o[2], o[3], o[4]);
    /* CONFIG 21 is the measured limit (main.c: the current sense); the engine leaves the outputs alone */
    rend(&U, ms + 2, o);
    CHECK(abs((int)o[1] - 65535) <= 2 && abs((int)o[0] - 32767) <= 2, "board limit: the current sense's (%u %u)",
          o[0], o[1]);
    ultra_slot_calibrate(s, useq++, 40);
    CHECK(send(&U, ms, s) == ULTRA_EV_IDENTIFY && U.identify == ULTRA_IDENT_CAL && U.identify_arg == 40,
          "IDENTIFY calibrate + duty");
    ultra_slot_identify(s, useq++, ULTRA_IDENT_ON);
    CHECK(send(&U, ms, s) == ULTRA_EV_IDENTIFY && U.identify == ULTRA_IDENT_ON, "IDENTIFY on");
    { /* Unknown current is not a settable value; neither state nor CRC changes. */
        static const uint8_t targets[4]={0,0,255,255};
        static const uint8_t current_ids[4]={11,12,21,33};
        uint16_t before_cfg[ULTRA_CFG_COUNT];
        uint32_t valid=U.cfg_valid;
        memcpy(before_cfg,U.cfg,sizeof before_cfg);
        U.cfg_changed=0;
        for(i=0;i<4;++i) {
            ids[0]=current_ids[i];vals[0]=ULTRA_LIMIT_UNKNOWN;
            ultra_slot_config(s,targets[i],useq++,ids,vals,1);
            CHECK(send(&U,ms,s)==0 && U.cfg_valid==valid && !U.cfg_changed &&
                  !memcmp(before_cfg,U.cfg,sizeof before_cfg),"unknown current rejected for CONFIG %u",ids[0]);
        }
    }
}

/* ------------------------------------------------------------------------------------------------ clock sync */
typedef struct { ultra_t u; double err, phase; } board_t;
static board_t BA, BB;
static uint32_t btick(const board_t *b, double t_us) { return (uint32_t)floor((t_us * (1.0 + b->err) + b->phase) / 50.0); }

static void test_ultra_clock(void)
{
    static const uint8_t mono[5] = { 0, 0, 0, 0, ULTRA_ROLE_BRIGHTNESS };
    double t = 0, next = 0, maxd = 0, maxo = 0, off_sum = 0;
    board_t *bs[2] = { &BA, &BB };
    uint8_t slot[32], p[24];
    int i, n = 0;
    uint32_t hb = ULTRA_TIME_WRAP - 1u - 5000000u;     /* host TIME wraps 5 s in */
    uint16_t oa[5], ob[5];
    int maxod = 0, fx_sent = 0;

    printf("ultra: shared clock, two boards with HSI +2.0 %% / -1.7 %%, headers every 20-300 ms\n");
    BA.err = 0.020; BA.phase = 12345; BB.err = -0.017; BB.phase = 777;
    for (i = 0; i < 2; i++) ultra_init(&bs[i]->u, (uint32_t)(i + 10));
    /* light 0 on both: brightness channel, breathe effect */
    memset(p, 0, sizeof p);
    while (t < 60e6) {
        if (t >= next) {
            uint32_t hus = ultra_time_wire((uint64_t)hb + (uint64_t)t);
            int fx_now = !fx_sent && t >= 10e6;
            fx_sent |= fx_now;
            for (i = 0; i < 2; i++) {
                double rx = t + 80 + urand(0, 50);       /* header end + poll latency */
                ultra_on_header(&bs[i]->u, btick(bs[i], rx), hus);
                if (t < 1) {
                    uint16_t a[5] = { 0, 0, 0, 0, 0 }, b[5] = { 65535, 65535, 65535, 65535, 65535 };
                    uint16_t one[5] = { 65535, 65535, 65535, 65535, 65535 };
                    ultra_slot_light_def(slot, 0, 1, mono, 1000, 0, a, b, 0);
                    ultra_on_slot(&bs[i]->u, slot, 0);
                    ultra_slot_set(slot, 0, 2, 0, 65535, one, b, 0, 0);
                    ultra_on_slot(&bs[i]->u, slot, 0);
                }
                if (fx_now) {
                    ultra_put24(p, 3000); ultra_put16(p + 3, 0); ultra_put16(p + 5, 65535);
                    ultra_slot_effect(slot, 0, 3, ULTRA_FX_BREATHE, 30, p, 7);
                    ultra_on_slot(&bs[i]->u, slot, 0);
                }
            }
            next = t + urand(20e3, 300e3);
        }
        t += urand(200, 3000);
        if (t > 20e6) {                                  /* compare the boards' host time estimates */
            double ea = (double)ultra_host_us(&BA.u, btick(&BA, t)), eb = (double)ultra_host_us(&BB.u, btick(&BB, t));
            double truth = (double)hb + t - 105;        /* boards date the header ~105 us late (end + poll) */
            if (fabs(ea - eb) > maxd) maxd = fabs(ea - eb);
            if (fabs(ea - truth) > maxo) maxo = fabs(ea - truth);
            off_sum += ea - truth; n++;
            ultra_render(&BA.u, btick(&BA, t), oa);
            ultra_render(&BB.u, btick(&BB, t), ob);
            if (abs((int)oa[4] - (int)ob[4]) > maxod) maxod = abs((int)oa[4] - (int)ob[4]);
        }
    }
    printf("  after sync: boards differ by max %.0f us, vs host max %.0f us (mean %.1f), breathe outputs max %d counts apart\n",
           maxd, maxo, off_sum / n, maxod);
    printf("  rate estimate A %.4f %% B %.4f %% (true %.2f / %.2f)\n",
           (BA.u.clk.rate_q16 / 65536.0 / 50.0 - 1) * 100, (BB.u.clk.rate_q16 / 65536.0 / 50.0 - 1) * 100,
           (1 / 1.02 - 1) * 100, (1 / 0.983 - 1) * 100);
    CHECK(maxd < 250 && maxo < 150, "clock sync");
    CHECK(maxod < 350, "effects in phase across boards");

    /* host reboot: time jumps back; ramps keep going, sequence numbers start over */
    {
        uint16_t a[5] = { 0, 0, 0, 0, 0 }, b[5] = { 65535, 65535, 65535, 65535, 65535 }, o1[5], o2[5];
        uint32_t tb = btick(&BA, t);
        ultra_slot_effect(slot, 0, 4, ULTRA_FX_NONE, 0, p, 0);
        ultra_on_header(&BA.u, tb, ultra_time_wire((uint64_t)hb + (uint64_t)t));
        ultra_on_slot(&BA.u, slot, tb);
        ultra_slot_set(slot, 0, 5, 0, 0, a, b, 4000, 0);   /* 4 s fade to 0 */
        ultra_on_slot(&BA.u, slot, tb);
        ultra_render(&BA.u, tb + 20000, o1);               /* 1 s in */
        ultra_on_header(&BA.u, tb + 20001, 1234567u);      /* rebooted host */
        ultra_render(&BA.u, tb + 20040, o2);
        CHECK(abs((int)o1[4] - (int)o2[4]) < 100 && o1[4] > 40000, "fade continues over a host reboot (%u -> %u)", o1[4], o2[4]);
        ultra_slot_set(slot, 0, 5, 0, 65535, a, b, 0, 0);  /* same seq as before the reboot */
        CHECK((ultra_on_slot(&BA.u, slot, tb) & ULTRA_EV_LIGHT), "seq restarts after a host reboot");
    }
}

/* ------------------------------------------------------------------------------------------------ long uptime */
static void test_ultra_uptime(void)
{
    uint64_t ms; uint16_t o[5];
    static const uint8_t mono[5]={0,0,0,0,ULTRA_ROLE_BRIGHTNESS};
    static const double a[5]={1,1,1,1,1};
    ultra_init(&U,9); def_light(&U,1,1,mono,1,0,0,1); set_light(&U,1,1,0,0.5,a,0,0);
    for(ms=2;ms<60ull*86400000ull;ms+=9973) {
        uint32_t tick=(uint32_t)(ms*20+7);
        ultra_on_header(&U,tick,ultra_time_wire((uint64_t)HOST0+(uint64_t)(ms*1000))); ultra_render(&U,tick,o);
        CHECK(o[4]==32768,"static level changes at clock wrap");
    }
}


/* ------------------------------------------------------------------------------------------------ clock tend */
/* the engine's time without headers. Four one-channel lights: candle (CH1), flicker (CH2), random (CH3),
 * breathe (CH4), at M 0.8.
 *  (a) the power-on effect on the board's own clock from boot (tick 0) for 3 days: past the tick's wrap (2.49 days);
 *  (b) a host for a minute, then gone (signal loss "hold": the effects keep running) for 3 days: past 1.24 days (the
 *      int32 of the ticks since the last header) and past the wrap.
 * Rendered every second, and every ms for 65 s windows from 5 s before those points; ultra_clock_tend as main.c calls
 * it. The negative control runs without it: the engine's time jumps there, and on the board's own
 * clock (a) it jumps back 2.5 days: candle, flicker and random freeze. */
#define E2E_TAIL_WORDS 576u
static void test_ultra_e2e(void)
{
    timing_spec_t spec = { 220, 380, 520, 1000, 1200, 1500, 30, 0.02, 20e3 };
    ws_timing_t tm = WS_TIMING_DEFAULT;
    static uint8_t frame[32 * 4 + 8], dec[PROTO_BITS_BUF], tail[8];
    static double fall[8 * (32 * 4 + 8)];
    uint32_t copy = DIN_COPY_WORDS * 16u, maxw = 0, min_tail_bits = 100000;
    long k, ok = 0, trials = 4000, applied = 0, err[9] = { 0 };
    static const uint8_t mono[5] = { 0, 0, 0, 0, ULTRA_ROLE_BRIGHTNESS };

    printf("ultra: end to end (builder -> WS waveform, full spec timing, +-30 ns, hiccups -> 36 MHz capture ->\n"
           "       slot from the frame start, header from the frame end -> engine), %u-word copy, %u-word tail\n",
           DIN_COPY_WORDS, E2E_TAIL_WORDS);
    for (k = 0; k < trials; k++) {
        int boards = 1 + (int)rint_(4), i, r, nb = 256 * boards + 64;
        double ts = 1e9 / (36e6 * (1.0 + urand(-0.022, 0.022))), t, phase = urand(0, ts), t_tick1, t_copy, t_edge;
        long w_tick1, w_copy, w_edge;
        uint32_t n_start, n_end, tu, tu_sent = (uint32_t)rnd() % ULTRA_TIME_WRAP;
        ultra_t *u = &BA.u;
        for (i = 0; i < boards; i++) {
            uint16_t a[5] = { 0, 0, 0, 0, 0 }, b[5] = { 65535, 65535, 65535, 65535, 65535 };
            if (i == 0) ultra_slot_light_def(frame, 0, (uint8_t)k, mono, 2800, 0, a, b, 0);
            else ultra_slot_set(frame + 32 * i, 0, (uint8_t)rnd(), 0, (uint16_t)rnd(), a, b, 1000, 0);
        }
        ultra_build_header(frame + 32 * boards, tu_sent);
        W.n = 0;
        t = add_frame(&W, 0, frame, 60, &spec, NULL);
        t += urand(285e3, 600e3);
        add_frame(&W, t, frame, nb, &spec, fall);
        /* the slot: from 60 us before the first edge to one tick after bit 256 */
        t_tick1 = W.rise[60] + urand(0.0, 50e3) + 0.2e3;
        t_copy = fall[255] + urand(0.0, 50e3) + 50e3 + 0.2e3;
        w_tick1 = (long)floor((t_tick1 - phase) / ts / 16.0) - 1;
        w_copy = (long)floor((t_copy - phase) / ts / 16.0) - 1;
        n_start = (uint32_t)(w_tick1 - 135) * 16u;
        n_end = (uint32_t)w_copy * 16u;
        if ((n_end - n_start) / 16u > maxw) maxw = (n_end - n_start) / 16u;
        if (n_end - n_start > copy) n_end = n_start + copy;
        sample_wave(&W, phase + n_start * ts, ts, n_end - n_start, SAMP);
        r = ws_decode_frame(SAMP, n_end - n_start, 256, &tm, dec);
        if (r != 256) { err[-r]++; continue; }
        if (memcmp(dec, frame, 32) != 0) { err[6]++; continue; }
        /* the header: the poll tick that saw the last edges (0-50 us after it) notes the ring position; the
         * window ends 4 words after it, 448 words long */
        t_edge = fall[nb - 1] + urand(0.0, 50e3) + 0.2e3;
        w_edge = (long)floor((t_edge - phase) / ts / 16.0) - 1;
        n_end = (uint32_t)(w_edge + 4) * 16u;
        n_start = n_end - E2E_TAIL_WORDS * 16u;
        sample_wave(&W, phase + n_start * ts, ts, n_end - n_start, SAMP);
        r = ws_decode_last(SAMP, n_end - n_start, &tm, tail);
        if (r < 64) { err[r < 0 ? 7 : 8]++; continue; }
        if ((uint32_t)r < min_tail_bits) min_tail_bits = (uint32_t)r;
        if (ultra_parse_header(tail, &tu) != 0 || tu != tu_sent) { err[6]++; continue; }
        ok++;
        ultra_init(u, 1);
        ultra_on_header(u, 100, tu);
        if ((ultra_on_slot(u, dec, 100) & ULTRA_EV_LIGHT) && u->light[0].defined) applied++;
    }
    printf("  %ld/%ld frames decoded, %ld slots applied; errors %ld/%ld/%ld/%ld/%ld, wrong %ld, tail %ld/%ld; "
           "longest slot capture %u words, fewest bits in a tail window %u\n",
           ok, trials, applied, err[1], err[2], err[3], err[4], err[5], err[6], err[7], err[8], maxw, min_tail_bits);
    CHECK(ok == trials && applied == trials, "end to end");
    CHECK(maxw <= DIN_COPY_WORDS, "capture fits the copy buffer");
    CHECK(min_tail_bits >= 80, "the tail window holds the header with margin (%u bits)", min_tail_bits);
}

/* ------------------------------------------------------------------------------------------------ compatibility */
/* ultra_protocol.h "Compatibility rules": what this firmware doesn't know is ignored (the engine's state stays
 * byte-identical) or falls back (unknown effect: the plain state; unknown role: the channel unused). Pairs of engines
 * get the same slots, one with the unknown values and one without: their states must stay identical. */
static ultra_t UR_ANALOG_M, UB;
static uint8_t cseq = 1;

static int u_same(void) { return memcmp(&UR_ANALOG_M, &UB, sizeof UR_ANALOG_M) == 0; }

/* the slot to both engines (b: a variant, NULL = the same); 1 if both return the same events */
static int c_pair(const uint8_t *a, const uint8_t *b, uint32_t ms)
{
    uint32_t ea = ultra_on_slot(&UR_ANALOG_M, a, tk(ms)), eb = ultra_on_slot(&UB, b ? b : a, tk(ms));
    return ea == eb;
}

/* a slot only to UB (UR_ANALOG_M unchanged): 1 if it was ignored (no event, state identical) */
static int c_ignored(const uint8_t *s, uint32_t ms) { return ultra_on_slot(&UB, s, tk(ms)) == 0 && u_same(); }

static void c_reseal(uint8_t *s) { ultra_slot_seal(s); }

static void c_setup(void)
{
    static const uint8_t rgbww[5] = { ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE, ULTRA_ROLE_COLD_WHITE,
                                      ULTRA_ROLE_WARM_WHITE };
    static const double a[5] = { 1.0, 0.5, 0.25, 0.75, 0.4 };
    ultra_init(&UR_ANALOG_M, 7);
    ultra_init(&UB, 7);
    hdr(&UR_ANALOG_M, 0);
    hdr(&UB, 0);
    useq = 50;
    def_light(&UR_ANALOG_M, 0, 0, rgbww, 2.8, 0, 0, 1);
    useq = 50;
    def_light(&UB, 0, 0, rgbww, 2.8, 0, 0, 1);
    useq = 60;
    set_light(&UR_ANALOG_M, 0, 0, 0, 0.8, a, 0, 0);
    useq = 60;
    set_light(&UB, 0, 0, 0, 0.8, a, 0, 0);
}

static void test_ultra_compat(void)
{
    static const uint8_t rgbww[5] = { ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE, ULTRA_ROLE_COLD_WHITE,
                                      ULTRA_ROLE_WARM_WHITE };
    static const uint8_t brp[7] = { 0x00, 0x0B, 0xB8, 0x00, 0x00, 0xFF, 0xFF };   /* breathe 3 s, 0..1 */
    uint16_t a[5] = { 65535, 30000, 20000, 40000, 10000 }, b[5] = { 65535, 65535, 65535, 65535, 65535 };
    uint16_t z[5] = { 0, 0, 0, 0, 0 }, o[5] = { 65535, 65535, 65535, 65535, 65535 }, oa[5], ob[5];
    uint8_t s[32], v[32];
    uint32_t i, k, bad;
    int ok;

    printf("ultra: compatibility rules (unknown types, LIGHT 5-14, CONFIG ids, effects, roles, flags, spare bytes, "
           "IDENTIFY / SCENE values, header EXT / HINT, broadcast lengths)\n");

    /* unknown slot TYPE 0x0A-0xFF (incl. broadcast 0x80-0xFF, e.g. a wrong board count): ignored, also with a
     * payload that would be a valid command of a known type */
    c_setup();
    for (k = ULTRA_TYPES, bad = 0; k <= 0xFFu; k++) {
        ultra_slot_set(s, 0, cseq++, ULTRA_SET_SMOOTH, 40000, a, b, 500, 0);
        if (!(k & ULTRA_TYPE_BCAST) && (k & ULTRA_TYPE_MASK)<ULTRA_LIGHT_TYPES) continue;
        s[0] = (uint8_t)k;
        c_reseal(s);
        if (!c_ignored(s, 10)) bad++;
        s[1] = (uint8_t)(ULTRA_TARGET_BOARD);
        c_reseal(s);
        if (!c_ignored(s, 10)) bad++;
    }
    CHECK(bad == 0, "unknown slot types 0x0A-0xFF ignored, state unchanged (%u)", bad);

    /* LIGHT 5-14: ignored for every type */
    for (k = ULTRA_LIGHTS, bad = 0; k < ULTRA_TARGET_BOARD; k++) {
        for (i = 1; i < ULTRA_TYPES; i++) {
            ultra_slot_set(s, 0, cseq++, 0, 40000, a, b, 500, 0);
            s[0] = (uint8_t)i;
            s[1] = (uint8_t)k;
            c_reseal(s);
            if (!c_ignored(s, 10)) bad++;
        }
    }
    CHECK(bad == 0, "LIGHT 5-14 ignored for every slot type (%u)", bad);

    /* unknown CONFIG ids (23-127 official, 128-255 hardware-specific): that pair skipped, the others applied */
    c_setup();
    ultra_slot_begin(s, ULTRA_CONFIG, ULTRA_TARGET_BOARD, 0, cseq++);
    {
        static const uint8_t ids[9] = { 200, ULTRA_CFG_DIM_FLOOR, ULTRA_CFG_COUNT, 127, ULTRA_CFG_STAGGER, 128, 255, 0,
                                        ULTRA_CFG_LOSS_TIMEOUT };
        static const uint16_t vals[9] = { 0xBEEF, 1, 0x1111, 0x2222, 3, 0x3333, 0x4444, 0x5555, 12 };
        for (i = 0; i < 9; i++) {
            s[ULTRA_PAYLOAD + 3 * i] = ids[i];
            ultra_put16(s + ULTRA_PAYLOAD + 3 * i + 1, vals[i]);
        }
    }
    c_reseal(s);
    k = UB.cfg_valid;
    ok = ultra_on_slot(&UB, s, tk(10)) == ULTRA_EV_CONFIG;
    CHECK(ok && UB.cfg[ULTRA_CFG_DIM_FLOOR] == 1 && UB.cfg[ULTRA_CFG_STAGGER] == 3 && UB.cfg[ULTRA_CFG_LOSS_TIMEOUT] == 12 &&
          (UB.cfg_valid & ~k) == ((1u << ULTRA_CFG_DIM_FLOOR) | (1u << ULTRA_CFG_STAGGER) | (1u << ULTRA_CFG_LOSS_TIMEOUT)),
          "unknown CONFIG ids skipped, the known pairs of the same slot applied");

    /* unknown roles 8-15: the channel isn't part of the light (same state as ULTRA_ROLE_NONE) */
    c_setup();
    {
        static const uint8_t known[5] = { ULTRA_ROLE_RED, ULTRA_ROLE_NONE, ULTRA_ROLE_BLUE, ULTRA_ROLE_NONE,
                                          ULTRA_ROLE_WARM_WHITE };
        ultra_slot_light_def(s, 0, cseq, known, 2800, 0, z, o, 0);
        memcpy(v, s, 32);
        v[ULTRA_PAYLOAD] = (uint8_t)((ULTRA_ROLE_RED << 4) | 8u);           /* CH2 role 8 (amber?) */
        v[ULTRA_PAYLOAD + 1] = (uint8_t)((ULTRA_ROLE_BLUE << 4) | 15u);     /* CH4 role 15 */
        c_reseal(v);
        cseq++;
        ok = c_pair(s, v, 10) && u_same() && UB.light[0].mask == 0x15u;
        static const double full[5] = { 1, 1, 1, 1, 1 };
        set_light(&UR_ANALOG_M, 20, 0, 0, 1.0, full, 0, 0);
        rend(&UR_ANALOG_M, 30, oa);
        CHECK(ok && oa[1] == 0 && oa[3] == 0 && oa[0] != 0, "unknown roles 8-15: the channel is unused (mask 0x%02X)",
              UB.light[0].mask);
    }

    /* unknown flag bits (SET / LIGHT_DEF / LIGHT_OPT flags, slot FLAGS of every other type) and spare payload bytes:
     * the same state as without them */
    c_setup();
    bad = 0;
    ultra_slot_set(s, 0, cseq++, ULTRA_SET_SMOOTH, 30000, a, b, 800, 0);
    memcpy(v, s, 32);
    v[0] |= 0u; /* SET uses both TYPE flags; no unknown flag remains. */                                       /* SET flags 0x04, 0x08 */
    c_reseal(v);
    if (!c_pair(s, v, 10) || !u_same()) bad |= 1;
    ultra_slot_light_def(s, 0, cseq++, rgbww, 2200, ULTRA_DEF_CONSTANT_BRIGHTNESS, z, o, 0);
    memcpy(v, s, 32);
    v[0] |= ULTRA_TYPE_FLAGS;                                       /* slot FLAGS */
    v[ULTRA_PAYLOAD + 5] |= 0xFCu;                       /* LIGHT_DEF flags 0x04-0x80 */
    v[ULTRA_PAYLOAD + 2] |= 0x0Fu;                       /* the low nibble after CH5's role */
    c_reseal(v);
    if (!c_pair(s, v, 20) || !u_same()) bad |= 2;
    ultra_slot_light_opt(s, 0, cseq++, ULTRA_OPT_DIM_TO_WARM, 1538, 3704, 4500, 20000);
    memcpy(v, s, 32);
    v[0] |= ULTRA_TYPE_FLAGS;
    v[ULTRA_PAYLOAD] |= 0xFCu;                           /* LIGHT_OPT flags 0x04-0x80 */
    v[ULTRA_PAYLOAD + 25] = v[ULTRA_PAYLOAD + 26] = 0xFFu;
    c_reseal(v);
    if (!c_pair(s, v, 30) || !u_same()) bad |= 4;
    ultra_slot_scene(s, 0, cseq++, 1, 5, 40000, a, b);
    memcpy(v, s, 32);
    v[0] |= ULTRA_TYPE_FLAGS;
    v[ULTRA_PAYLOAD + 24] = v[ULTRA_PAYLOAD + 25] = v[ULTRA_PAYLOAD + 26] = 0xFFu;
    c_reseal(v);
    if (!c_pair(s, v, 40) || !u_same()) bad |= 8;
    {
        static const uint8_t ids[2] = { ULTRA_CFG_DITHER, ULTRA_CFG_FEEDBACK };
        static const uint16_t vals[2] = { 2, 1 };
        ultra_slot_config(s,ULTRA_TARGET_BOARD, cseq++, ids, vals, 2);
        memcpy(v, s, 32);
        v[0] |= ULTRA_TYPE_FLAGS;
        c_reseal(v);
        if (!c_pair(s, v, 50) || !u_same()) bad |= 16;
    }
    ultra_slot_identify(s, cseq++, ULTRA_IDENT_ON);
    memcpy(v, s, 32);
    v[0] |= ULTRA_TYPE_FLAGS;
    for (i = 1; i < ULTRA_PAYLOAD_LEN; i++) v[ULTRA_PAYLOAD + i] = 0xFFu;   /* ON: the argument and the rest spare */
    c_reseal(v);
    if (!c_pair(s, v, 60) || UR_ANALOG_M.identify != ULTRA_IDENT_ON || UB.identify != ULTRA_IDENT_ON) bad |= 32;
    UB.identify_arg = UR_ANALOG_M.identify_arg;                   /* (stored as is; main.c reads it only for CAL / DARK) */
    if (!u_same()) bad |= 32;
    {
        uint16_t sa[5] = { 1, 2, 3, 4, 5 }, sb[5] = { 6, 7, 8, 9, 10 };
        static const uint8_t two[1] = { 2 };
        ultra_slot_effect(s, 0, cseq, ULTRA_FX_STROBE, 0, two, 1);
        memcpy(v, s, 32);
        v[0] |= ULTRA_TYPE_FLAGS;
        c_reseal(v);
        hdr(&UR_ANALOG_M, 70);
        hdr(&UB, 70);
        if (!c_pair(s, v, 70) || !u_same()) bad |= 64;
        for (i = 0; i < 2; i++) {
            ultra_slot_step(s, 0, cseq, (uint8_t)i, 2, 200, 0, 50000, i ? sa : a, i ? sb : b);
            memcpy(v, s, 32);
            v[0] |= ULTRA_TYPE_FLAGS;
            c_reseal(v);
            if (!c_pair(s, v, 71) || !u_same()) bad |= 64;
        }
        cseq++;
    }
    CHECK(bad == 0, "unknown flag bits and spare bytes ignored: the same state as without (0x%X)", bad);

    /* EFFECT spare parameter bytes (after the effect's own): the same output */
    c_setup();
    ultra_slot_effect(s, 0, cseq++, ULTRA_FX_BREATHE, 0, brp, 7);
    memcpy(v, s, 32);
    for (i = 5 + 7; i < ULTRA_PAYLOAD_LEN; i++) v[ULTRA_PAYLOAD + i] = 0xA5u;
    c_reseal(v);
    hdr(&UR_ANALOG_M, 100);
    hdr(&UB, 100);
    ok = c_pair(s, v, 100);
    for (i = 120, bad = 0; i < 9000; i += 11) {
        rend(&UR_ANALOG_M, i, oa);
        rend(&UB, i, ob);
        if (memcmp(oa, ob, sizeof oa) != 0) bad++;
    }
    CHECK(ok && bad == 0, "EFFECT spare parameter bytes ignored (%u differing renders)", bad);

    /* IDENTIFY values 6-255 and CAL arguments 1-4 / 101-255: ignored; the defined ones (0-5) are taken */
    c_setup();
    for (k = ULTRA_IDENT_VERSION + 1u, bad = 0; k <= 0xFFu; k++) {
        ultra_slot_identify(s, cseq++, ULTRA_IDENT_ON);
        s[ULTRA_PAYLOAD] = (uint8_t)k;
        c_reseal(s);
        if (!c_ignored(s, 10)) bad++;
    }
    ultra_slot_identify(s, cseq++, ULTRA_IDENT_VERSION);   /* 5: the version blink */
    if (s[ULTRA_PAYLOAD] != ULTRA_IDENT_VERSION) bad++;
    if (ultra_on_slot(&UB, s, tk(10)) != ULTRA_EV_IDENTIFY || UB.identify != ULTRA_IDENT_VERSION) bad++;
    memcpy(&UB, &UR_ANALOG_M, sizeof UB);
    ultra_slot_identify(s, cseq++, ULTRA_IDENT_VERSION + 1u);   /* the builder still maps reserved values to STOP */
    if (s[ULTRA_PAYLOAD] != ULTRA_IDENT_STOP) bad++;
    for (k = 1; k <= 0xFFu; k++) {
        uint32_t ev;
        ultra_slot_calibrate(s, cseq++, 0);
        s[ULTRA_PAYLOAD + 1] = (uint8_t)k;
        c_reseal(s);
        if (k < ULTRA_IDENT_CAL_MIN || k > ULTRA_IDENT_CAL_MAX) {
            if (!c_ignored(s, 10)) bad++;
        } else {
            ev = ultra_on_slot(&UB, s, tk(10));
            if (ev != ULTRA_EV_IDENTIFY || UB.identify != ULTRA_IDENT_CAL || UB.identify_arg != k) bad++;
            memcpy(&UB, &UR_ANALOG_M, sizeof UB);
        }
    }
    for (k = 0; k <= ULTRA_IDENT_RELEASE; k++) {
        ultra_slot_identify(s, cseq++, (uint8_t)k);
        if (ultra_on_slot(&UB, s, tk(10)) != ULTRA_EV_IDENTIFY || UB.identify != k) bad++;
    }
    CHECK(bad == 0, "IDENTIFY: values 6-255 and CAL arguments 1-4 / 101-255 ignored, 0-5 taken (%u)", bad);

    /* SCENE modes 2-255: ignored (the stored scene stays) */
    c_setup();
    for (k = 2, bad = 0; k <= 0xFFu; k++) {
        ultra_slot_scene(s, 0, cseq++, 1, 5, 40000, a, b);
        s[ULTRA_PAYLOAD] = (uint8_t)k;
        c_reseal(s);
        if (!c_ignored(s, 10)) bad++;
    }
    CHECK(bad == 0, "SCENE modes 2-255 ignored (%u)", bad);

    /* the hub's builders never emit an undefined value, whatever they are given */
    {
        uint8_t rr[5] = { 9, 15, 3, 8, 7 }, ids[9] = { 0, 1, 200, ULTRA_CFG_COUNT, 22, 128, 255, 5, 127 };
        uint16_t vals[9] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
        uint8_t par[22];
        memset(par, 0xEE, sizeof par);
        bad = 0;
        ultra_slot_set(s, 0, 1, 0xFF, 1, a, b, 1, 0);
        if ((s[0] & ULTRA_TYPE_FLAGS) != ULTRA_SET_FLAGS) bad |= 1;
        ultra_slot_light_def(s, 0, 1, rr, 2800, 0xFF, z, o, 0);
        if (s[ULTRA_PAYLOAD] != 0x00u || s[ULTRA_PAYLOAD + 1] != 0x30u || s[ULTRA_PAYLOAD + 2] != 0x70u ||
            s[ULTRA_PAYLOAD + 5] != ULTRA_DEF_FLAGS) bad |= 2;
        ultra_slot_light_opt(s, 0, 1, 0xFF, 1, 2, 3, 4);
        if (s[ULTRA_PAYLOAD] != ULTRA_OPT_FLAGS) bad |= 4;
        ultra_slot_effect(s, 0, 1, 200, 0, par, 22);
        for (i = 0; i < ULTRA_PAYLOAD_LEN; i++) if (s[ULTRA_PAYLOAD + i] != 0) bad |= 8;   /* FX_NONE, no params */
        ultra_slot_effect(s, 0, 1, ULTRA_FX_CANDLE, 0, par, 22);
        for (i = 5 + 4; i < ULTRA_PAYLOAD_LEN; i++) if (s[ULTRA_PAYLOAD + i] != 0) bad |= 8;
        ultra_slot_config(s,ULTRA_TARGET_BOARD, 1, ids, vals, 9);
        for(i=0;i<9u;++i) if(s[3u+3u*i]!=ids[i] || ultra_get16(s+4u+3u*i)!=vals[i]) bad |= 16;

        ultra_slot_identify(s, 1, 9);
        if (s[ULTRA_PAYLOAD] != ULTRA_IDENT_STOP) bad |= 32;
        ultra_slot_calibrate(s, 1, 3);
        if (s[ULTRA_PAYLOAD + 1] != ULTRA_IDENT_CAL_MIN) bad |= 32;
        ultra_slot_calibrate(s, 1, 250);
        if (s[ULTRA_PAYLOAD + 1] != ULTRA_IDENT_CAL_MAX) bad |= 32;
        ultra_slot_scene(s, 0, 1, 7, 1, 1, a, b);
        if (s[ULTRA_PAYLOAD] != 7) bad |= 64; /* reserved selector stays reserved */
        CHECK(bad == 0, "builders: only defined values (0x%X)", bad);
    }

    /* header flags at the frame end (frame_rx): EXT and RW keep the lock and the clock sync but apply no slot, HINT is
     * processed as usual, UPD goes to the update side (no sync); also an EXT frame locks a fresh auto board */
    {
        frx_t r;
        frx_end_t e;
        slot_t ws;
        uint8_t h[8];
        uint32_t f, fl;
        static const uint16_t on[5] = { 65535, 65535, 65535, 65535, 65535 };
        bad = 0;
        frx_init(&r, PMODE_ULTRA, PROTO_NONE);
        ultra_slot_set(s, 0, 9, 0, 40000, on, on, 0, 0);
        for (fl = 0, f = 1; fl < 16u; fl++, f++) {
            int want_sync = !(fl & ULTRA_HDR_UPD), want_slot = want_sync && !(fl & (ULTRA_HDR_RW | ULTRA_HDR_EXT));
            frx_capture(&r, f, PROTO_ULTRA, s, 256, 256, 0, &ws);
            ultra_build_header_flags(h, 1000u * f, (uint8_t)fl);
            frx_frame_end(&r, f, 256 * 3 + 64, h, &e);
            if (e.ultra_header != 1 || e.flags != fl || e.ultra_sync != want_sync || e.ultra_slot != want_slot ||
                e.after != PROTO_ULTRA)
                bad++;
        }
        CHECK(bad == 0, "header flags: EXT / RW sync without slot, HINT normal, UPD no sync (%u)", bad);
        frx_init(&r, PMODE_AUTO, PROTO_NONE);
        frx_capture(&r, 1, PROTO_NONE, s, 256, 256, 0, &ws);
        ultra_build_header_flags(h, 5000u, ULTRA_HDR_EXT | ULTRA_HDR_HINT);
        frx_frame_end(&r, 1, 256 * 2 + 64, h, &e);
        CHECK(e.after == PROTO_ULTRA && e.ultra_sync && !e.ultra_slot && r.ps.locked == PROTO_ULTRA,
              "an EXT frame locks a fresh auto board (no slot applied)");
        /* another VERSION (with a valid CRC): no header at all - no lock, no clock, no slot */
        for (fl = 0, bad = 0; fl < 16u; fl++) {
            uint32_t t;
            if (fl == ULTRA_VERSION) continue;
            ultra_build_header(h, 7000u);
            h[2] = (uint8_t)((fl << 4) | (h[2] & 0x0Fu));
            h[7] = ultra_crc8(h, 7);
            if (ultra_parse_header(h, &t) == 0) bad++;
            frx_init(&r, PMODE_AUTO, PROTO_NONE);
            frx_capture(&r, 1, PROTO_NONE, s, 256, 256, 0, &ws);
            frx_frame_end(&r, 1, 256 * 2 + 64, h, &e);
            if (e.ultra_header || e.ultra_sync || e.ultra_slot || r.ps.locked == PROTO_ULTRA) bad++;
        }
        CHECK(bad == 0, "another header VERSION is rejected entirely (%u)", bad);
    }

    /* broadcast slots keep the frame an Ultra length: 256 x (N + k) + 64 for any N + k */
    for (k = 1, bad = 0; k <= 255u; k++) {
        if (proto_classify(256u * k + 64u, PROTO_UH_HEADER) != PROTO_ULTRA ||
            proto_classify(256u * k + 64u, PROTO_UH_HEADER | PROTO_UH_SLOT) != PROTO_ULTRA) bad++;
    }
    CHECK(bad == 0, "classifier: 256 x (N + k) + 64 is an Ultra length for N + k = 1..255 (%u)", bad);
}

#include "test_m1_light_base.c"
static void test_ultra(void)
{
    test_m1_light_base();
    test_ultra_protocol();
    test_ultra_compat();
    test_ultra_gamma();
    test_ultra_engine();
    test_ultra_transitions();
    test_ultra_effects();
    test_ultra_power();
    test_ultra_clock();
    test_ultra_epochs();
    test_ultra_uptime();

    test_ultra_e2e();
}
