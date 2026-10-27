/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Host tests of the local-effects engine (src/core/fx*.c, docs/ur_digital_m-effects.md): the math, every effect (chunk and
 * order invariance, determinism, speed, robustness against odd parameters), the golden frames, the segments, the
 * transitions and the packer. Real code, no mocks. Build + run: test\build_fx.cmd (MSVC); test_fx.exe --golden prints a new
 * golden table (test/fx_golden.inc) after a deliberate change of an effect's look.
 *
 * The golden CRCs pin what the effects look like today (a regression net for refactors and ports); the other tests check
 * what they must do. fx_effects.c is included so the tests can reach its internals (pac_sum, the effects). */
#include <math.h>
#if defined(_MSC_VER)
#pragma warning(disable : 4127)                 /* (while (0) in the CHECK macro) */
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define FX_SAT_CHECK 1
#include <stdint.h>
uint32_t fx_sat_over;
#include "../src/core/fx_effects.c"

static int g_fail, g_checks;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---------------------------------------------------------------------------------------------------- helpers */
typedef struct { uint8_t id, speed, intensity, palette; uint8_t c1[3], c2[3], c3[3]; uint8_t custom1, custom2; } defaults_t;
static const defaults_t DEF[] = {
#include "fx_defaults.inc"
};
#define NDEF ((int)(sizeof(DEF) / sizeof(DEF[0])))

static uint32_t rng_s = 0x12345678u;
static uint32_t rnd(void) { rng_s ^= rng_s << 13; rng_s ^= rng_s >> 17; rng_s ^= rng_s << 5; return rng_s; }

static uint32_t crc32_u(uint32_t crc, const void *d, size_t n)
{
    const uint8_t *p = (const uint8_t *)d;
    while (n--) {
        uint32_t k;
        crc ^= *p++;
        for (k = 0; k < 8; k++) {
            crc = crc & 1u ? 0xEDB88320u ^ (crc >> 1) : crc >> 1;
        }
    }
    return crc;
}

static void params_from(const defaults_t *d, fx_params_t *p)
{
    memset(p, 0, sizeof(*p));
    p->id = d->id; p->speed = d->speed; p->intensity = d->intensity; p->palette = d->palette;
    p->c[0] = fx_rgb(d->c1[0], d->c1[1], d->c1[2]);
    p->c[1] = fx_rgb(d->c2[0], d->c2[1], d->c2[2]);
    p->c[2] = fx_rgb(d->c3[0], d->c3[1], d->c3[2]);
    p->bri = 255; p->custom1 = d->custom1; p->custom2 = d->custom2; p->seed = 0x1234u;
}
static const defaults_t *def_of(uint32_t id)
{
    int k;
    for (k = 0; k < NDEF; k++) {
        if (DEF[k].id == id) {
            return &DEF[k];
        }
    }
    return 0;
}
static void params_of(uint32_t id, fx_params_t *p) { params_from(def_of(id), p); }

static int rgb_eq(rgb_t a, rgb_t b) { return a.r == b.r && a.g == b.g && a.b == b.b; }
static int near_rgb(rgb_t a, rgb_t b, int tol)
{
    return abs((int)a.r - b.r) <= tol && abs((int)a.g - b.g) <= tol && abs((int)a.b - b.b) <= tol;
}
static uint32_t crc_rgb(const rgb_t *p, uint32_t n)
{
    uint32_t c = 0xFFFFFFFFu, i;
    for (i = 0; i < n; i++) {
        c = crc32_u(c, &p[i].r, 1);
        c = crc32_u(c, &p[i].g, 1);
        c = crc32_u(c, &p[i].b, 1);
    }
    return ~c;
}
static uint32_t sumrgb(const rgb_t *p, uint32_t n)
{
    uint32_t s = 0, i;
    for (i = 0; i < n; i++) {
        s += (uint32_t)p[i].r + p[i].g + p[i].b;
    }
    return s;
}

static uint8_t g_state[8192];

/* one ctx rendered whole at time t (after frames at t0..t every `step` ms) */
static void render_whole(const fx_params_t *p, uint32_t n, uint32_t t, rgb_t *out)
{
    fx_ctx_t x;
    fx_begin(&x, p, (uint16_t)n, g_state);
    fx_frame(&x, t);
    fx_render(&x, out, 0, n);
}

/* ---------------------------------------------------------------------------------------------------- math */
static void test_math(void)
{
    uint32_t th, maxe = 0, k;
    int32_t prev;
    for (th = 0; th < 65536u; th++) {
        const double ref = 32767.0 * sin(2.0 * 3.14159265358979323846 * (double)th / 65536.0);
        const int32_t v = fx_sin16(th);
        const uint32_t e = (uint32_t)fabs((double)v - ref);
        maxe = e > maxe ? e : maxe;
        if (v != -fx_sin16((65536u - th) & 0xFFFFu)) {
            CHECK(0, "sin16 not odd at %u", th);
            break;
        }
    }
    CHECK(maxe <= 4, "sin16 max error %u", maxe);
    CHECK(fx_sin16(0) == 0 && fx_sin16(16384) == 32767 && fx_sin16(49152) == -32767 && fx_sin16(32768) == 0, "sin16 landmarks");
    CHECK(fx_sin8(0) == 128 && fx_sin8(64) == 255 && fx_sin8(192) == 0, "sin8 landmarks %u %u %u", fx_sin8(0), fx_sin8(64), fx_sin8(192));
    for (th = 0, prev = fx_sin16(0); th < 65536u; th += 64) {              /* monotonic over the quarter */
        if (th <= 16384u) {
            CHECK(fx_sin16(th) >= prev, "sin16 not monotonic in the first quarter");
            prev = fx_sin16(th);
        }
    }
    for (k = 0; k < 20000u; k++) {                                          /* beats: the 32-bit product wraps cleanly */
        const uint32_t t = rnd(), bpm = rnd() & 0xFFFFu;
        const uint64_t ref = ((uint64_t)t * bpm * 280u >> 16) & 0xFFFFu;
        CHECK(fx_beat88(t, bpm) == ref, "beat88(%u, %u)", t, bpm);
        if (g_fail > 5) {
            break;
        }
    }
    for (k = 0; k < 5000u; k++) {
        const uint32_t v = fx_beatsin88(rnd(), rnd() & 0xFFFFu, 10, 200, 0);
        CHECK(v >= 10 && v <= 200, "beatsin range %u", v);
    }
    {                                                                       /* value noise: range, mean, smoothness */
        fx_nz1_t z;
        uint32_t x, lo = 255, hi = 0, mx = 0;
        uint64_t sum = 0;
        uint8_t last = 0;
        fx_nz1_begin(&z, 0x1234u);
        for (x = 0; x < 256u * 400u; x++) {
            const uint8_t v = fx_nz1(&z, x);
            sum += v;
            lo = v < lo ? v : lo;
            hi = v > hi ? v : hi;
            if (x > 0) {
                const uint32_t d = abs((int)v - last);
                mx = d > mx ? d : mx;
            }
            last = v;
        }
        CHECK(mx <= 3, "noise jumps by %u per 1/256 lattice step", mx);
        CHECK(lo < 25 && hi > 230, "noise range %u..%u", lo, hi);
        CHECK(sum / (256u * 400u) > 110 && sum / (256u * 400u) < 146, "noise mean %u", (unsigned)(sum / (256u * 400u)));
        fx_nz1_begin(&z, 0x1234u);                                          /* the cache gives what a fresh one gives */
        for (x = 0; x < 3000u; x += 7) {
            fx_nz1_t w;
            fx_nz1_begin(&w, 0x1234u);
            CHECK(fx_nz1(&z, x) == fx_nz1(&w, x), "noise cache at %u", x);
        }
    }
    {
        uint32_t i;
        rgb_t pal[16];
        for (i = 0; i < 16u; i++) {
            pal[i] = fx_rgb(i * 16u, 255u - i * 16u, i);
        }
        CHECK(rgb_eq(fx_pal_lin(pal, 0), pal[0]) && rgb_eq(fx_pal_lin(pal, 255), pal[15]), "pal_lin ends");
        for (i = 0; i < 16u; i++) {
            CHECK(rgb_eq(fx_pal_at(pal, i << 12), pal[i]), "pal_at entry %u", i);
        }
        CHECK(near_rgb(fx_pal_at(pal, 0xFFFFu), pal[0], 17), "pal_at wraps to entry 0");
        CHECK(rgb_eq(fx_blend(fx_rgb(10, 20, 30), fx_rgb(200, 100, 0), 0), fx_rgb(10, 20, 30)), "blend 0");
        CHECK(rgb_eq(fx_blend(fx_rgb(10, 20, 30), fx_rgb(200, 100, 0), 255), fx_rgb(200, 100, 0)), "blend 255");
        CHECK(rgb_eq(fx_hue(0), fx_rgb(255, 0, 0)) && rgb_eq(fx_hue(85), fx_hue(85)), "hue 0");
        CHECK(fx_hue(43).r >= 250 && fx_hue(43).g == 255 && fx_hue(43).b == 0, "hue yellow");
        CHECK(rgb_eq(fx_sat(fx_rgb(255, 0, 0), 255), fx_rgb(255, 0, 0)) && rgb_eq(fx_sat(fx_rgb(255, 0, 0), 0), fx_rgb(255, 255, 255)),
              "sat ends");
        CHECK(fx_scale8(255, 255) == 255 && fx_scale8(255, 0) == 0 && fx_scale8(128, 255) == 128, "scale8");
    }
    {                                                                       /* tables */
        uint32_t i;
        uint32_t worst = 0;
        for (i = 1; i < 256u; i++) {
            const uint32_t d = (uint32_t)abs((int)fx_gamma(i) - (int)floor(255.0 * pow(i / 255.0, 2.8) + 0.5));
            CHECK(fx_gamma(i) >= fx_gamma(i - 1u), "gamma monotonic at %u", i);
            CHECK(d == 0u, "gamma 2.8 at %u: %u, the rounded curve %.2f", i, fx_gamma(i), 255.0 * pow(i / 255.0, 2.8));
            worst = d > worst ? d : worst;
        }
        CHECK(fx_gamma(0) == 0 && fx_gamma(255) == 255, "gamma ends");
        printf("  gamma 8-bit (the 16-bit table rounded): at most %u off the rounded exact curve, i.e. the 8-bit table\n", worst);
    }
}

/* ---------------------------------------------------------------------------------------------------- effects */
#define NMAX 1300
static rgb_t bufA[NMAX], bufB[NMAX];

/* the chunked and ordered render must equal the whole render, and nothing outside [from, to) may change */
static void test_chunks(void)
{
    static const uint32_t sizes[] = { 1, 2, 3, 7, 8, 9, 31, 32, 33, 64, 150, 301, 1000 };
    static const uint32_t times[] = { 0, 1, 700, 12345, 654321, 3600000u + 17u };
    uint32_t id, si, ti, k;
    for (id = 1; id <= FX_LAST; id++) {
        for (si = 0; si < sizeof(sizes) / sizeof(sizes[0]); si++) {
            const uint32_t n = sizes[si];
            for (ti = 0; ti < sizeof(times) / sizeof(times[0]); ti += 2 + (si & 1u)) {
                fx_params_t p;
                fx_ctx_t x, y;
                uint32_t pos, left;
                params_of(id, &p);
                p.vlen = (si % 3u == 2u) ? (uint16_t)(n + 10u) : 0u;           /* some on a longer virtual strip */
                p.vstart = (si % 4u == 3u) ? 7u : 0u;
                p.flags = 0;
                fx_begin(&x, &p, (uint16_t)n, g_state);
                fx_frame(&x, times[ti]);
                fx_render(&x, bufA, 0, n);
                memset(bufB, 0xA5, sizeof(bufB));
                fx_begin(&y, &p, (uint16_t)n, g_state + 4096);
                fx_frame(&y, times[ti]);
                /* pieces of random size, in a shuffled order */
                {
                    uint32_t starts[NMAX], cnt = 0, i;
                    for (pos = 0; pos < n; pos += left) {
                        left = 1u + rnd() % 40u;
                        left = pos + left > n ? n - pos : left;
                        starts[cnt++] = pos | (left << 16);
                    }
                    for (i = cnt; i > 1; i--) {
                        const uint32_t j = rnd() % i, tmp = starts[i - 1u];
                        starts[i - 1u] = starts[j];
                        starts[j] = tmp;
                    }
                    for (i = 0; i < cnt; i++) {
                        const uint32_t from = starts[i] & 0xFFFFu, len = starts[i] >> 16;
                        rgb_t piece[64];
                        memset(piece, 0x5A, sizeof(piece));
                        fx_render(&y, piece, from, from + len);
                        memcpy(bufB + from, piece, len * sizeof(rgb_t));
                        for (k = len; k < 64u; k++) {
                            if (piece[k].r != 0x5A || piece[k].g != 0x5A || piece[k].b != 0x5A) {
                                CHECK(0, "effect %u wrote past its range (n %u, from %u, len %u)", id, n, from, len);
                                break;
                            }
                        }
                    }
                }
                CHECK(memcmp(bufA, bufB, n * sizeof(rgb_t)) == 0, "effect %u: chunked / shuffled render differs (n %u, t %u)", id, n, times[ti]);
            }
        }
    }
}

/* two contexts, same params, same frame times: identical; frame spacing does not matter for the effects without
 * per-frame accumulators; speed 0 stands still */
static void test_time(void)
{
    uint32_t id, k;
    for (id = 1; id <= FX_LAST; id++) {
        fx_params_t p;
        fx_ctx_t a, b;
        const int accum = id == FX_COLORWAVES || id == FX_PRIDE || id == FX_PACIFICA || id == FX_FIRE;
        params_of(id, &p);
        fx_begin(&a, &p, 100, g_state);
        fx_begin(&b, &p, 100, g_state + 4096);
        for (k = 0; k <= 5000u; k += 33) {
            fx_frame(&a, k);
            fx_frame(&b, k);
        }
        fx_render(&a, bufA, 0, 100);
        fx_render(&b, bufB, 0, 100);
        CHECK(memcmp(bufA, bufB, 100 * sizeof(rgb_t)) == 0, "effect %u not deterministic", id);
        if (!accum) {                                                  /* one jump to t equals many steps to t */
            fx_params_t q = p;
            fx_ctx_t c;
            fx_begin(&c, &q, 100, g_state + 2048);
            fx_frame(&c, 4950);
            fx_frame(&c, 4950 + 33 - 33);                              /* (the same time twice: no change) */
            fx_ctx_t d = a;
            (void)d;
            fx_begin(&b, &p, 100, g_state + 4096);
            for (k = 0; k <= 4950u; k += 150) {
                fx_frame(&b, k);
            }
            fx_render(&b, bufB, 0, 100);
            fx_render(&c, bufA, 0, 100);
            CHECK(memcmp(bufA, bufB, 100 * sizeof(rgb_t)) == 0, "effect %u: the frame spacing changes the picture", id);
        }
        if (id != FX_SUNRISE && id != FX_SUNSET) {                     /* speed 0: standing still */
            fx_params_t z = p;
            z.speed = 0;
            render_whole(&z, 100, 1000, bufA);
            render_whole(&z, 100, 900000, bufB);
            CHECK(memcmp(bufA, bufB, 100 * sizeof(rgb_t)) == 0, "effect %u moves at speed 0", id);
        }
        if (id != FX_SUNRISE && id != FX_SUNSET && id != FX_FIRE && id != FX_SOLID && id != FX_GRADIENT) {
            fx_ctx_t x;                                                /* a speed change keeps the picture (et is continuous) */
            fx_begin(&x, &p, 100, g_state);
            fx_frame(&x, 3000);
            fx_render(&x, bufA, 0, 100);
            fx_set_speed(&x, (uint8_t)(p.speed / 2u + 3u));
            fx_frame(&x, 3000);
            fx_render(&x, bufB, 0, 100);
            CHECK(memcmp(bufA, bufB, 100 * sizeof(rgb_t)) == 0, "effect %u jumps when the speed changes", id);
        }
    }
    {                                                                   /* the rate: faster means further */
        uint32_t s;
        for (s = 1; s < 255u; s++) {
            CHECK(fx_rate256(s + 1u) >= fx_rate256(s), "rate not monotonic at %u", s);
        }
        CHECK(fx_rate256(0) == 0 && fx_rate256(128) > 230 && fx_rate256(128) < 270, "rate 128 is ~1x: %u", fx_rate256(128));
    }
}

/* odd parameters, ranges and times must never crash or write outside */
static void test_fuzz(void)
{
    static const uint16_t ns[] = { 0, 1, 2, 3, 5, 8, 9, 31, 33, 100, 255, 700, 1250 };
    uint32_t it;
    for (it = 0; it < 4000u; it++) {
        fx_params_t p;
        fx_ctx_t x;
        const uint32_t n = ns[rnd() % (sizeof(ns) / sizeof(ns[0]))];
        uint32_t f, from, to;
        memset(&p, 0, sizeof(p));
        p.id = (uint8_t)(it % 5u == 0u ? rnd() : 1u + rnd() % FX_LAST);
        p.speed = (uint8_t)(rnd() % 3u == 0u ? (rnd() & 1u ? 0u : 255u) : rnd());
        p.intensity = (uint8_t)(rnd() % 3u == 0u ? (rnd() & 1u ? 0u : 255u) : rnd());
        p.palette = (uint8_t)(rnd() % 4u == 0u ? 0xFEu + (rnd() & 1u) : rnd() % 30u);
        p.c[0] = fx_rgb(rnd(), rnd(), rnd());
        p.c[1] = fx_rgb(rnd(), rnd(), rnd());
        p.c[2] = fx_rgb(rnd(), rnd(), rnd());
        p.custom1 = (uint8_t)rnd();
        p.custom2 = (uint8_t)rnd();
        p.seed = rnd();
        p.vstart = (uint16_t)(rnd() % 3u == 0u ? rnd() : 0u);
        p.vlen = (uint16_t)(rnd() % 3u == 0u ? 1u + rnd() % 2000u : 0u);
        memset(g_state, 0xEE, 2048);
        fx_begin(&x, &p, (uint16_t)n, rnd() % 7u == 0u ? 0 : g_state);
        for (f = 0; f < 6u; f++) {
            const uint32_t t = rnd() % 4u == 0u ? rnd() : f * 40u + (rnd() % 5u == 0u ? 0xFFFFFF00u : 0u);
            rgb_t guard[NMAX + 64];
            fx_frame(&x, t);
            from = rnd() % (n + 3u);
            to = from + rnd() % 70u;
            memset(guard, 0x77, sizeof(guard));
            fx_render(&x, guard + 16, from, to);
            {
                uint32_t i, bad = 0;
                const uint32_t lim = to > n ? n : to, cnt = from < lim ? lim - from : 0u;
                for (i = 0; i < 16u; i++) {
                    bad |= guard[i].r != 0x77;
                }
                for (i = 16u + cnt; i < NMAX + 64u; i++) {
                    bad |= guard[i].r != 0x77 || guard[i].g != 0x77 || guard[i].b != 0x77;
                }
                CHECK(!bad, "fuzz: write outside the range (id %u n %u [%u, %u))", p.id, n, from, to);
            }
        }
    }
}

/* golden frames: each effect at its catalogue defaults, 64 pixels, rendered at 4 times */
typedef struct { uint8_t id; uint32_t crc[4]; } golden_t;
static const golden_t GOLDEN[] = {
#include "fx_golden.inc"
};
static void golden_frame(uint32_t id, uint32_t *crc)
{
    static const uint32_t times[4] = { 0, 700, 12345, 123456 };
    fx_params_t p;
    fx_ctx_t x;
    uint32_t k;
    params_of(id, &p);
    fx_begin(&x, &p, 64, g_state);
    for (k = 0; k < 4u; k++) {
        fx_frame(&x, times[k]);
        fx_render(&x, bufA, 0, 64);
        crc[k] = crc_rgb(bufA, 64);
    }
}
static void print_golden(void)
{
    int k;
    for (k = 0; k < NDEF; k++) {
        uint32_t crc[4];
        golden_frame(DEF[k].id, crc);
        printf("{ %u, { 0x%08Xu, 0x%08Xu, 0x%08Xu, 0x%08Xu } },\n", DEF[k].id, crc[0], crc[1], crc[2], crc[3]);
    }
}
static void test_golden(void)
{
    int k;
    CHECK(sizeof(GOLDEN) / sizeof(GOLDEN[0]) == (size_t)NDEF, "golden table has %u effects, the catalogue %d", (unsigned)(sizeof(GOLDEN) / sizeof(GOLDEN[0])), NDEF);
    for (k = 0; k < NDEF && k < (int)(sizeof(GOLDEN) / sizeof(GOLDEN[0])); k++) {
        uint32_t crc[4], i;
        CHECK(GOLDEN[k].id == DEF[k].id, "golden order");
        golden_frame(DEF[k].id, crc);
        for (i = 0; i < 4u; i++) {
            CHECK(crc[i] == GOLDEN[k].crc[i], "golden: effect %u frame %u CRC %08X, expected %08X", DEF[k].id, i, crc[i], GOLDEN[k].crc[i]);
        }
    }
}

/* what each effect must do (not just stay the same) */
static void test_behaviour(void)
{
    fx_params_t p;
    uint32_t i, t, id;
    /* Solid */
    params_of(FX_SOLID, &p);
    p.c[0] = fx_rgb(10, 20, 30);
    render_whole(&p, 50, 777, bufA);
    for (i = 0; i < 50u; i++) {
        CHECK(rgb_eq(bufA[i], p.c[0]), "solid pixel %u", i);
    }
    /* every effect: not all black at its defaults (except the sunrise that starts dark) and colours stay in range */
    for (id = 1; id <= FX_LAST; id++) {
        uint32_t lit = 0;
        params_of(id, &p);
        for (t = 0; t < 20000u; t += 1000u) {
            render_whole(&p, 100, t, bufA);
            lit += sumrgb(bufA, 100) > 0u;
        }
        CHECK(id == FX_SUNRISE ? lit >= 1 : lit >= 15, "effect %u is dark (%u of 20 frames lit)", id, lit);
    }
    /* Gradient: ends in c1 .. c3, the middle c2 */
    params_of(FX_GRADIENT, &p);
    render_whole(&p, 101, 0, bufA);
    CHECK(rgb_eq(bufA[0], p.c[0]) && rgb_eq(bufA[100], p.c[2]), "gradient ends");
    CHECK(near_rgb(bufA[50], p.c[1], 2), "gradient middle %u,%u,%u", bufA[50].r, bufA[50].g, bufA[50].b);
    p.intensity = 16;                                                  /* looping once: thirds c1, c2, c3 */
    p.speed = 0;
    render_whole(&p, 90, 0, bufA);
    CHECK(rgb_eq(bufA[0], p.c[0]) && near_rgb(bufA[30], p.c[1], 3) && near_rgb(bufA[60], p.c[2], 3), "looping gradient thirds");
    /* Breathe: depth */
    params_of(FX_BREATHE, &p);
    p.intensity = 0;
    for (t = 0; t < 8000u; t += 500u) {
        render_whole(&p, 4, t, bufA);
        CHECK(rgb_eq(bufA[0], p.c[0]), "breathe at depth 0 must hold the colour (t %u)", t);
    }
    p.intensity = 255;
    {
        uint32_t lo = 999, hi = 0;
        for (t = 0; t < 12000u; t += 50u) {
            uint32_t v;
            render_whole(&p, 1, t, bufA);
            v = bufA[0].r;
            lo = v < lo ? v : lo;
            hi = v > hi ? v : hi;
        }
        CHECK(lo < 12 && hi > 245, "breathe swing %u..%u", lo, hi);
    }
    /* Candle: one flame colour for the whole strip, a flicker; Candle Multi: every pixel its own */
    params_of(FX_CANDLE, &p);
    {
        uint32_t lo = 999, hi = 0;
        for (t = 0; t < 10000u; t += 20u) {
            render_whole(&p, 10, t, bufA);
            CHECK(rgb_eq(bufA[0], bufA[9]), "candle: pixels differ");
            lo = bufA[0].r < lo ? bufA[0].r : lo;
            hi = bufA[0].r > hi ? bufA[0].r : hi;
            CHECK(bufA[0].r >= bufA[0].g && bufA[0].g >= bufA[0].b, "candle colour order");
        }
        CHECK(hi - lo > 40, "candle flicker %u..%u", lo, hi);
    }
    params_of(FX_CANDLE_MULTI, &p);
    render_whole(&p, 40, 5000, bufA);
    {
        uint32_t same = 0;
        for (i = 1; i < 40u; i++) {
            same += rgb_eq(bufA[i], bufA[0]);
        }
        CHECK(same < 5, "candle multi: pixels flicker together (%u equal)", same);
    }
    /* twinkles: density 0 is the background; more density lights more */
    {
        static const uint8_t ids[] = { FX_TWINKLEFOX, FX_COLORTWINKLES, FX_FAIRY };
        uint32_t k;
        for (k = 0; k < 3u; k++) {
            uint32_t lit0 = 0, lit1 = 0, lit2 = 0;
            params_of(ids[k], &p);
            p.c[1] = fx_rgb(0, 0, 0);
            p.intensity = 0;
            for (t = 0; t < 6000u; t += 300u) {
                render_whole(&p, 200, t, bufA);
                lit0 += sumrgb(bufA, 200) > 0u;
            }
            CHECK(lit0 == 0, "twinkle %u: lit pixels at density 0", ids[k]);
            p.intensity = 90;
            for (t = 0; t < 6000u; t += 300u) {
                render_whole(&p, 200, t, bufA);
                for (i = 0; i < 200u; i++) lit1 += bufA[i].r + bufA[i].g + bufA[i].b > 30u;
            }
            p.intensity = 255;
            for (t = 0; t < 6000u; t += 300u) {
                render_whole(&p, 200, t, bufA);
                for (i = 0; i < 200u; i++) lit2 += bufA[i].r + bufA[i].g + bufA[i].b > 30u;
            }
            CHECK(lit1 > 0 && lit2 > lit1, "twinkle %u: density %u / %u lit", ids[k], lit1, lit2);
        }
    }
    /* Sunrise / Sunset */
    params_of(FX_SUNRISE, &p);
    p.speed = 20;                                                      /* 10 s + 14 s * 20 = 290 s */
    p.intensity = 0;
    {
        uint32_t prev = 0, dur = 290000u;
        render_whole(&p, 30, 0, bufA);
        CHECK(sumrgb(bufA, 30) == 0, "sunrise starts dark");
        for (t = 0; t <= dur; t += 5000u) {
            uint32_t s;
            render_whole(&p, 30, t, bufA);
            s = sumrgb(bufA, 30);
            CHECK(s >= prev, "sunrise gets dimmer at %u", t);
            prev = s;
        }
        render_whole(&p, 30, dur + 1000u, bufA);
        CHECK(near_rgb(bufA[0], fx_pal_entry(PAL_SUNRISE, 15), 2), "sunrise ends in the last palette entry");
        p.id = FX_SUNSET;
        render_whole(&p, 30, 0, bufA);
        CHECK(near_rgb(bufA[0], fx_pal_entry(PAL_SUNRISE, 15), 2), "sunset starts bright");
        render_whole(&p, 30, dur + 50000u, bufA);
        CHECK(sumrgb(bufA, 30) == 0, "sunset ends dark");
        render_whole(&p, 30, 3600000u * 5u, bufA);
        CHECK(sumrgb(bufA, 30) == 0, "sunset stays dark");
        p.id = FX_SUNRISE;
        p.intensity = 255;                                             /* spread: the near end first */
        render_whole(&p, 30, 100000u, bufA);
        CHECK(sumrgb(bufA, 1) > sumrgb(bufA + 29, 1), "sunrise spread: pixel 0 should lead");
    }
    /* Fire: heat builds at the start, none at the far end; no state: solid c1 */
    params_of(FX_FIRE, &p);
    {
        fx_ctx_t x;
        fx_begin(&x, &p, 100, g_state);
        for (t = 0; t < 4000u; t += 20u) {
            fx_frame(&x, t);
        }
        fx_render(&x, bufA, 0, 100);
        CHECK(sumrgb(bufA, 20) > 600, "fire: no flames at the start");
        CHECK(sumrgb(bufA + 90, 10) < sumrgb(bufA, 10), "fire: the far end is hotter than the start");
        fx_begin(&x, &p, 100, 0);
        fx_frame(&x, 100);
        fx_render(&x, bufA, 0, 100);
        CHECK(rgb_eq(bufA[50], p.c[0]), "fire without state falls back to colour 1");
        CHECK(fx_state_bytes(FX_FIRE, 100) == 100 && fx_state_bytes(FX_SOLID, 100) == 0, "state bytes");
        CHECK(fx_state_bytes(FX_FIRE, 1000) == FX_FIRE_CELLS && fx_state_bytes(FX_FIRE, 1) == 1, "state bytes are capped at the cells");
        fx_begin(&x, &p, 1000, g_state);                                   /* more pixels than cells: interpolated */
        for (t = 0; t < 4000u; t += 20u) {
            fx_frame(&x, t);
        }
        fx_render(&x, bufA, 0, 1000);
        {
            uint32_t jump = 0, k;
            for (k = 1; k < 1000u; k++) {
                const uint32_t d = (uint32_t)abs((int)bufA[k].r - bufA[k - 1u].r) + abs((int)bufA[k].g - bufA[k - 1u].g);
                jump = d > jump ? d : jump;
            }
            CHECK(sumrgb(bufA, 100) > 1500, "fire on 1000 px: no flames at the start");
            CHECK(jump < 160, "fire on 1000 px: cells are interpolated, largest step %u", jump);
        }
    }
    /* Meteor: head colour at the head, the background ahead */
    params_of(FX_METEOR, &p);
    p.c[1] = fx_rgb(0, 0, 40);
    render_whole(&p, 100, 2000, bufA);
    {
        uint32_t head = 0, k;
        for (k = 0; k < 100u; k++) {
            head = bufA[k].r > bufA[head].r ? k : head;
        }
        CHECK(bufA[head].r >= 220, "meteor head not bright (%u)", bufA[head].r);
        CHECK(head + 1u >= 100u || rgb_eq(bufA[head + 1u], p.c[1]), "meteor: the pixel ahead is not the background");
    }
    /* Sinelon: one bright dot; two dots */
    params_of(FX_SINELON, &p);
    p.palette = PAL_RAINBOW;
    {
        uint32_t bright = 0;
        render_whole(&p, 100, 1500, bufA);
        for (i = 0; i < 100u; i++) bright += (bufA[i].r > 120u) | (bufA[i].g > 120u) | (bufA[i].b > 120u);
        CHECK(bright >= 1 && bright <= 6, "sinelon dot size %u", bright);
        p.custom1 = 200;
        bright = 0;
        render_whole(&p, 100, 1500, bufA);
        for (i = 0; i < 100u; i++) bright += (bufA[i].r > 120u) | (bufA[i].g > 120u) | (bufA[i].b > 120u);
        CHECK(bright >= 2, "sinelon with two dots: %u bright", bright);
    }
    /* Flow: an even number of zones spread over the whole strip; the colour at a zone border is the same on both sides
     * (neighbouring zones run in opposite directions) */
    params_of(FX_FLOW, &p);
    p.intensity = 200;
    {
        fx_ctx_t x;
        uint32_t k, n, zones;
        for (n = 20; n <= 200; n += 37) {
            fx_begin(&x, &p, (uint16_t)n, g_state);
            fx_frame(&x, 5000);
            fx_render(&x, bufA, 0, n);
            zones = x.u[0];
            CHECK(zones >= 2 && (zones & 1u) == 0u && zones * 6u <= n + 6u * 2u, "flow zones %u on %u px", zones, n);
            for (k = 1; k < zones; k++) {
                const uint32_t bd = (k * n) / zones;
                CHECK(near_rgb(bufA[bd - 1u], bufA[bd], 30), "flow border %u of %u (%u px): %u,%u,%u | %u,%u,%u", k, zones, n, bufA[bd - 1u].r, bufA[bd - 1u].g, bufA[bd - 1u].b, bufA[bd].r, bufA[bd].g, bufA[bd].b);
            }
        }
    }
    /* Rainbow: hue steps are small between neighbours; Palette follows its palette */
    params_of(FX_RAINBOW, &p);
    render_whole(&p, 200, 3000, bufA);
    for (i = 1; i < 200u; i++) {
        CHECK(near_rgb(bufA[i], bufA[i - 1u], 30), "rainbow step at %u", i);
    }
    params_of(FX_PALETTE, &p);
    p.palette = PAL_COLORS;
    p.c[0] = fx_rgb(255, 0, 0);
    p.c[1] = fx_rgb(0, 255, 0);
    p.c[2] = fx_rgb(0, 0, 255);
    p.speed = 0;
    p.intensity = 128;                                                 /* one palette over the strip */
    render_whole(&p, 96, 0, bufA);
    CHECK(rgb_eq(bufA[0], p.c[0]) && near_rgb(bufA[32], p.c[1], 20) && near_rgb(bufA[64], p.c[2], 20), "palette c1-c2-c3 thirds: %u,%u,%u", bufA[32].r, bufA[32].g, bufA[32].b);
    /* the palette ids: 0xFF = the effect's own */
    {
        fx_ctx_t x;
        params_of(FX_FIRE, &p);
        fx_begin(&x, &p, 20, g_state);
        CHECK(x.pal_id == PAL_HEAT && rgb_eq(x.pal[15], fx_pal_entry(PAL_HEAT, 15)), "fire's default palette");
        p.palette = PAL_OCEAN;
        fx_begin(&x, &p, 20, g_state);
        CHECK(rgb_eq(x.pal[3], fx_pal_entry(PAL_OCEAN, 3)), "explicit palette");
    }
}

/* the closed forms against the originals' sequential sums */
static void test_closed_forms(void)
{
    uint32_t th0, m, maxd = 0;
    fx_params_t p;
    uint32_t i, t;
    /* Pacifica: sum of the colour-index increments for pixels 1..m */
    for (th0 = 0; th0 < 65536u; th0 += 997u) {
        uint32_t wh;
        for (wh = 650u; wh < 1900u; wh += 311u) {
            uint32_t seq = 0, th = th0;
            for (m = 1; m <= 1300u; m++) {
                const uint32_t s16 = (uint32_t)(fx_sin16(th + 250u * m) + 32768);
                seq += ((s16 * wh) >> 16) + wh;
                if (m % 97u == 0u || m == 1300u) {
                    const uint32_t cf = pac_sum(th0, m, wh), d = (cf - seq) & 0xFFFFu, ad = d > 32768u ? 65536u - d : d;
                    maxd = ad > maxd ? ad : maxd;
                }
            }
            (void)th;
        }
    }
    CHECK(maxd <= 48, "pacifica closed form differs from the sequential sum by up to %u (of 65536)", maxd);
    printf("  pacifica closed form: largest difference %u of 65536 over 1300 pixels\n", maxd);
    /* Pacifica rendered whole vs from a chunk start: no seam of more than a few levels */
    params_of(FX_PACIFICA, &p);
    {
        fx_ctx_t x;
        uint32_t maxc = 0;
        fx_begin(&x, &p, 1000, g_state);
        for (t = 0; t < 40000u; t += 977u) {
            fx_frame(&x, t);
        }
        fx_render(&x, bufA, 0, 1000);
        for (i = 32; i < 1000u; i += 32) {
            fx_render(&x, bufB, i, i + 32u);
            {
                const rgb_t a = bufA[i], b = bufB[0];
                const uint32_t d = abs((int)a.r - b.r) + abs((int)a.g - b.g) + abs((int)a.b - b.b);
                maxc = d > maxc ? d : maxc;
            }
        }
        CHECK(maxc <= 12, "pacifica: the chunk start differs from a whole render by %u levels", maxc);
        printf("  pacifica chunk start vs whole render: largest difference %u levels\n", maxc);
    }
    /* Pacifica against a straight sequential implementation of the original's loops (a sine per step, no phasor, no blocks) */
    {
        fx_ctx_t x;
        const uint32_t n = 400;
        uint32_t maxc = 0, L, th[4], ci[4], wh[4], br[4], ang[4];
        fx_begin(&x, (params_of(FX_PACIFICA, &p), &p), (uint16_t)n, g_state);
        for (t = 0; t < 61000u; t += 41u) {
            fx_frame(&x, t);
        }
        fx_render(&x, bufA, 0, n);
        wh[0] = fx_beatsin88(x.et, 3u * 256u, 11u * 256u, 14u * 256u, 0u);
        wh[1] = fx_beatsin88(x.et, 4u * 256u, 6u * 256u, 9u * 256u, 0u);
        wh[2] = 6u * 256u;
        wh[3] = 5u * 256u;
        br[0] = fx_beatsin88(x.et, 10u * 256u, 70u, 130u, 0u);
        br[1] = fx_beatsin88(x.et, 17u * 256u, 40u, 80u, 0u);
        br[2] = fx_beatsin88(x.et, 9u * 256u, 10u, 38u, 0u);
        br[3] = fx_beatsin88(x.et, 8u * 256u, 10u, 28u, 0u);
        ang[0] = 0u - fx_beat88(x.et, 301u);
        ang[1] = fx_beat88(x.et, 401u);
        ang[2] = 0u - fx_beat88(x.et, 503u);
        ang[3] = fx_beat88(x.et, 601u);
        for (L = 0; L < 4u; L++) {
            wh[L] = wh[L] / 2u + 20u;
            ci[L] = x.u[L];
            th[L] = ang[L];
        }
        for (i = 0; i < n; i++) {
            rgb_t c = fx_rgb(2, 6, 10);
            const uint32_t base = fx_beatsin88(x.et, 9u * 256u, 55u, 65u, 0u);
            uint32_t avg, thr;
            for (L = 0; L < 4u; L++) {
                uint32_t s16, cs, sidx;
                th[L] += 250u;
                s16 = (uint32_t)(fx_sin16(th[L]) + 32768);
                cs = ((s16 * wh[L]) >> 16) + wh[L];
                ci[L] += cs;
                sidx = ((uint32_t)(fx_sin16(ci[L]) + 32768) * 240u) >> 16;
                c = fx_add_sat(c, fx_scale_rgb(fx_pal_at(fx_pal_pacifica[L < 2u ? L : 2u], sidx << 8), br[L]));
            }
            thr = fx_scale8(fx_sin8(((fx_beat88(x.et, 7u * 256u) >> 8) + 7u * i) & 255u), 20u) + base;
            avg = avg3(c);
            if (avg > thr) {
                const uint32_t over = fx_scale8(avg - thr, 255u), over2 = fx_qadd8(over, over);
                c = fx_rgb(fx_qadd8(c.r, over), fx_qadd8(c.g, over2), fx_qadd8(c.b, fx_qadd8(over2, over2)));
            }
            c.b = fx_scale8(c.b, 145u);
            c.g = fx_scale8(c.g, 200u);
            c.r = c.r > 2u ? c.r : 2u;
            c.g = c.g > 5u ? c.g : 5u;
            c.b = c.b > 7u ? c.b : 7u;
            {
                const uint32_t d = (uint32_t)(abs((int)c.r - bufA[i].r) + abs((int)c.g - bufA[i].g) + abs((int)c.b - bufA[i].b));
                maxc = d > maxc ? d : maxc;
            }
        }
        CHECK(maxc <= 9, "pacifica differs from the original's sequential loops by %u levels (sum of channels)", maxc);
        printf("  pacifica vs the sequential original (400 px): largest difference %u levels (sum of channels)\n", maxc);
    }
    /* Pride / ColorWaves: the original per-pixel accumulation against the closed form */
    for (th0 = FX_COLORWAVES; th0 <= FX_PRIDE; th0++) {
        fx_ctx_t x;
        uint32_t n = 150, ref_h, ref_th;
        params_of(th0, &p);
        p.intensity = 128;
        fx_begin(&x, &p, (uint16_t)n, g_state);
        for (t = 0; t < 30000u; t += 33u) {
            fx_frame(&x, t);
        }
        fx_render(&x, bufA, 0, n);
        ref_h = x.u[1];
        ref_th = x.u[0];
        for (i = 0; i < n; i++) {                                      /* the original loop, i = 0 .. n-1 from the far end */
            const uint32_t sat = x.u[2] >> 8, depth = x.u[2] & 255u;
            const uint32_t pix = n - 1u - i;
            rgb_t c;
            uint32_t hue16, b16, bri16, bri;
            ref_h = (ref_h + x.u[4]) & 0xFFFFu;
            ref_th = (ref_th + x.u[3]) & 0xFFFFu;
            hue16 = ref_h;
            b16 = (uint32_t)(fx_sin16(ref_th) + 32768);
            bri16 = (b16 * b16) >> 16;
            bri = ((bri16 * depth) >> 16) + (255u - depth);
            c = th0 == FX_PRIDE ? fx_scale_rgb(fx_sat(fx_hue(hue16 >> 8), sat), bri) : fx_scale_rgb(fx_pal_at(x.pal, (((hue16 & 0x8000u) ? 0xFFFFu - hue16 : hue16) * 2u * 240u) >> 8), bri);
            CHECK(rgb_eq(bufA[pix], c), "effect %u pixel %u differs from the original's accumulation", th0, pix);
            if (g_fail > 3) {
                return;
            }
        }
    }
}

/* the closed-form integrals of the beats against a plain numerical integral (doubles, 1 ms steps) */
static double beat_ref(double t, uint32_t bpm88, double lo, double hi)
{
    const uint32_t ph = fx_beat88((uint32_t)t, bpm88);
    return lo + (hi - lo) * (fx_sin16(ph) + 32768) / 65536.0;
}

static void test_integrals(void)
{
    static const struct { uint32_t bpm, lo, hi; } B[] = { { 147, 23, 60 }, { 400, 5, 9 }, { 1011, 10, 13 }, { 257, 4, 6 } };
    static const struct { uint32_t sb, bpm, lo, hi; } P[] = { { 768, 1011, 10, 13 }, { 768, 777, 8, 11 }, { 1024, 777, 8, 11 },
                                                               { 768, 501, 5, 7 }, { 1024, 257, 4, 6 } };
    uint32_t k, t, maxd = 0, maxp = 0;
    for (k = 0; k < 4u; k++) {
        double acc = 0.0;
        for (t = 0; t < 200000u; t++) {
            acc += beat_ref((double)t + 0.5, B[k].bpm, B[k].lo, B[k].hi);
            if (t % 9973u == 0u) {
                const uint32_t cf = fx_beat_cum(t + 1u, B[k].bpm, B[k].lo, B[k].hi);
                const double ref = acc;
                const double d = fabs(fmod((double)cf - ref + 32768.0, 65536.0) - 32768.0);       /* modulo the 16 bits used */
                maxd = d > maxd ? (uint32_t)d : maxd;
            }
        }
    }
    CHECK(maxd <= 64, "beat integrals: closed form differs from the numerical one by up to %u (of 65536)", maxd);
    for (k = 0; k < 5u; k++) {
        double acc = 0.0;
        for (t = 0; t < 200000u; t++) {
            acc += beat_ref((double)t + 0.5, P[k].sb, 179, 269) * beat_ref((double)t + 0.5, P[k].bpm, P[k].lo, P[k].hi) / 256.0;
            if (t % 9973u == 0u) {
                const uint32_t cf = fx_beat_cum2(t + 1u, P[k].sb, 179, 269, P[k].bpm, P[k].lo, P[k].hi) >> 8;
                const double d = fabs(fmod((double)cf - acc + 32768.0, 65536.0) - 32768.0);
                maxp = d > maxp ? (uint32_t)d : maxp;
            }
        }
    }
    CHECK(maxp <= 64, "Pacifica integrals: closed form differs from the numerical one by up to %u (of 65536)", maxp);
    printf("  beat integrals vs numerical (200 s): largest difference %u / product %u of 65536\n", maxd, maxp);
    {                                                                        /* the inverse sine */
        int32_t sv, worst = 0;
        for (sv = -32767; sv <= 32767; sv += 7) {
            const int32_t back = fx_sin16((uint32_t)fx_asin16(sv) & 0xFFFFu), e = back > sv ? back - sv : sv - back;
            worst = e > worst ? e : worst;
        }
        CHECK(worst <= 96, "asin16: sin16(asin16(s)) is off by up to %d", worst);
        CHECK(fx_asin16(0) == 0 && fx_asin16(32767) >= 16380 && fx_asin16(-32767) <= -16380, "asin16 ends");
    }
}

/* two receivers that start the same slot at different moments (same anchor) must draw the same picture, for every
 * effect that is not stateful by design (Fire) */
static void test_join_time(void)
{
    uint32_t id, t;
    for (id = 1; id <= FX_LAST; id++) {
        fx_params_t p;
        fx_ctx_t early, late;
        if (id == FX_FIRE) {
            continue;
        }
        params_of(id, &p);
        fx_begin(&early, &p, 150, g_state);
        for (t = 0; t <= 20000u; t += 33u) {
            fx_frame(&early, t);
        }
        fx_render(&early, bufA, 0, 150);
        fx_begin(&late, &p, 150, g_state + 4096);
        fx_frame(&late, 19998u);                                             /* joins 20 s in */
        fx_frame(&late, 20000u - 20000u % 33u);                              /* (the same instant as early's last frame) */
        fx_frame(&early, 20000u - 20000u % 33u);
        fx_render(&late, bufB, 0, 150);
        fx_render(&early, bufA, 0, 150);
        CHECK(memcmp(bufA, bufB, 150 * sizeof(rgb_t)) == 0, "effect %u: a receiver that joins later draws another picture", id);
    }
}

/* the paces: the same slider position, about the same speed as WLED's namesakes (at 1x) */
static void test_pace(void)
{
    fx_params_t p;
    rgb_t a0, a1;
    params_of(FX_RAINBOW, &p);
    p.intensity = 128;
    render_whole(&p, 100, 1000, bufA);
    render_whole(&p, 100, 1000 + 1930u, bufB);                               /* ~1.9 s a round (et runs at 0.97x at speed 128) */
    CHECK(near_rgb(bufA[10], bufB[10], 12) && near_rgb(bufA[60], bufB[60], 12), "rainbow round is not ~1.9 s");
    render_whole(&p, 100, 1000 + 965u, bufB);
    CHECK(!near_rgb(bufA[10], bufB[10], 60), "rainbow stands still");
    CHECK(fx_rate256(128) > 240 && fx_rate256(128) < 256 && fx_rate256(255) > 440 && fx_rate256(255) < 520, "rate: 128 -> %u, 255 -> %u", fx_rate256(128), fx_rate256(255));
    {                                                                        /* a meteor's lap takes the same time on any length */
        uint32_t k, n[2] = { 40, 400 }, head[2][2];
        params_of(FX_METEOR, &p);
        for (k = 0; k < 2u; k++) {
            uint32_t i, best;
            render_whole(&p, n[k], 500, bufA);
            for (best = 0, i = 0; i < n[k]; i++) best = bufA[i].r > bufA[best].r ? i : best;
            head[k][0] = best * 1000u / n[k];
            render_whole(&p, n[k], 500 + 1690u, bufA);                       /* a lap later (1.64 s / 0.97) */
            for (best = 0, i = 0; i < n[k]; i++) best = bufA[i].r > bufA[best].r ? i : best;
            head[k][1] = best * 1000u / n[k];
            CHECK((uint32_t)abs((int)head[k][1] - (int)head[k][0]) < 120, "meteor lap on %u px: head %u -> %u (per mille)", n[k], head[k][0], head[k][1]);
        }
    }
    params_of(FX_PALETTE, &p);                                               /* Palette scrolls the other way from Rainbow */
    p.palette = PAL_COLORS;
    p.c[0] = fx_rgb(255, 0, 0); p.c[1] = fx_rgb(0, 255, 0); p.c[2] = fx_rgb(0, 0, 255);
    p.intensity = 128;
    render_whole(&p, 90, 0, bufA);
    render_whole(&p, 90, 150, bufB);
    {                                                                        /* red moves towards higher pixels: look for the pixel nearest to c1 */
        uint32_t i, r0 = 0, r1 = 0;
        for (i = 0; i < 90u; i++) {
            if (bufA[i].r > bufA[r0].r || (bufA[i].r == bufA[r0].r && i < 5u)) r0 = i;
            if (bufB[i].r > bufB[r1].r) r1 = i;
        }
        a0 = bufA[r0]; a1 = bufB[r1];
        CHECK(a0.r > 200 && a1.r > 200, "palette peaks");
    }
    {                                                                        /* Sinelon: the tail keeps its side after the turn */
        uint32_t t, prev = 0, falling = 0, rising = 0;
        params_of(FX_SINELON, &p);
        p.palette = PAL_RAINBOW;
        for (t = 0; t < 5000u; t += 25u) {                                   /* pixel 20 of 120 over a few passes */
            uint32_t v;
            render_whole(&p, 120, t, bufA);
            v = bufA[20].r + bufA[20].g + bufA[20].b;
            if (t > 0 && v + 8u < prev) falling++;
            if (t > 0 && v > prev + 40u) rising++;
            prev = v;
        }
        CHECK(rising >= 1 && falling >= 20, "sinelon: a pixel lights up when the dot passes and fades slowly after (%u rises, %u falls)", rising, falling);
        p.intensity = 255;                                                   /* Trail: higher = shorter */
        {
            uint32_t lit_short = 0, lit_long = 0, i;
            render_whole(&p, 120, 2300, bufA);
            for (i = 0; i < 120u; i++) lit_short += bufA[i].r + bufA[i].g + bufA[i].b > 30u;
            p.intensity = 10;
            render_whole(&p, 120, 2300, bufA);
            for (i = 0; i < 120u; i++) lit_long += bufA[i].r + bufA[i].g + bufA[i].b > 30u;
            CHECK(lit_long > lit_short, "sinelon trail slider: %u lit at 255, %u at 10", lit_short, lit_long);
        }
    }
}

/* ---------------------------------------------------------------------------------------------------- more effects */
static uint32_t lit_count(const rgb_t *p, uint32_t n)           /* pixels that are not black */
{
    uint32_t i, k = 0;
    for (i = 0; i < n; i++) {
        k += (p[i].r | p[i].g | p[i].b) != 0u;
    }
    return k;
}

static void test_more(void)
{
    fx_params_t p;
    uint32_t i, t, k;
    /* Stripes: Share 0 / 255 are all colour 2 / all colour 1; 128 is about half; the stripes move towards the higher pixels, one period
     * (8 px at Size 190) in 8 / 20 s; the edges are soft (a blend now and then) but never leave the two colours' range */
    params_of(FX_STRIPES, &p);
    p.custom1 = 0;
    render_whole(&p, 120, 1234, bufA);
    for (i = 0; i < 120u; i++) CHECK(rgb_eq(bufA[i], p.c[1]), "stripes share 0: pixel %u", i);
    p.custom1 = 255;
    render_whole(&p, 120, 1234, bufA);
    for (i = 0; i < 120u; i++) CHECK(rgb_eq(bufA[i], p.c[0]), "stripes share 255: pixel %u", i);
    params_of(FX_STRIPES, &p);
    for (t = 0; t < 4000u; t += 500u) {
        uint32_t red = 0;
        render_whole(&p, 160, t, bufA);
        for (i = 0; i < 160u; i++) {                                  /* c1 red (255,0,0), c2 white: the green channel is the mix */
            red += bufA[i].g < 128u;
            CHECK(bufA[i].r == 255u && bufA[i].g == bufA[i].b, "stripes: a pixel outside the two colours' blend (%u,%u,%u)", bufA[i].r, bufA[i].g, bufA[i].b);
        }
        CHECK(red > 60u && red < 100u, "stripes share 128 at t %u: %u of 160 pixels are colour 1", t, red);
    }
    {                                                                 /* the pattern moves towards the higher pixels (20 px/s: ~1.3 px in 70 ms) */
        uint32_t best = 0, bestd = 0;
        render_whole(&p, 160, 1000, bufA);
        for (k = 0; k < 8u; k++) {                                    /* the shift s (0..7 px) that maps frame 1 onto frame 2 best */
            uint32_t d = 0;
            render_whole(&p, 160, 1000u + 120u, bufB);
            for (i = 8; i < 152u; i++) d += (uint32_t)abs((int)bufB[i].g - (int)bufA[i - k].g);
            if (k == 0u || d < bestd) { bestd = d; best = k; }
        }
        CHECK(best >= 1u && best <= 4u, "stripes move towards the higher pixels (best shift %u px in 120 ms, 20 px/s -> ~2)", best);
    }
    /* Bulbs: ~Size of every cell lit; the lit pixels are palette colours; the colours hop one bulb every ~0.68 s (and not at speed 0) */
    params_of(FX_BULBS, &p);
    p.speed = 128;
    p.palette = PAL_COLORS;
    p.c[0] = fx_rgb(255, 0, 0); p.c[1] = fx_rgb(0, 0, 0); p.c[2] = fx_rgb(0, 0, 255);
    render_whole(&p, 160, 100, bufA);
    k = lit_count(bufA, 160);
    CHECK(k > 60u && k < 110u, "bulbs size 128: %u of 160 pixels lit", k);
    render_whole(&p, 160, 100 + 1400u, bufB);                         /* two hops later: the colour of the bulb has changed */
    CHECK(memcmp(bufA, bufB, 160 * sizeof(rgb_t)) != 0, "bulbs: the colours do not hop");
    p.speed = 0;
    render_whole(&p, 160, 100, bufA);
    render_whole(&p, 160, 100 + 9000u, bufB);
    CHECK(memcmp(bufA, bufB, 160 * sizeof(rgb_t)) == 0, "bulbs move at speed 0");
    p.speed = 128; p.custom2 = 0;                                     /* Colour step 0: one colour for every bulb */
    render_whole(&p, 160, 5000, bufA);
    for (i = 0; i < 160u; i++) {
        CHECK(bufA[i].g == 0u && bufA[i].b == 0u, "bulbs colour step 0: pixel %u is not colour 1 (%u,%u,%u)", i, bufA[i].r, bufA[i].g, bufA[i].b);
    }
    p.custom1 = 0;
    render_whole(&p, 160, 5000, bufA);
    CHECK(lit_count(bufA, 160) == 0u, "bulbs size 0: some are lit");
    /* Wipe: a sweep fills from pixel 0, front colour then the next; at the end of a sweep the strip is one colour; the cycle boundary has no jump */
    params_of(FX_WIPE, &p);
    p.speed = 128;
    {
        rgb_t f0 = p.c[0], f1 = p.c[1];
        uint32_t cyc = 0;
        render_whole(&p, 100, 100, bufA);                             /* early in the sweep: the front is c1 at the start, c2 behind */
        CHECK(near_rgb(bufA[0], f0, 2) && near_rgb(bufA[99], f1, 2), "wipe start: front %u,%u,%u back %u,%u,%u", bufA[0].r, bufA[0].g, bufA[0].b, bufA[99].r, bufA[99].g, bufA[99].b);
        for (i = 1; i < 100u; i++) {                                  /* the front is a prefix: once it is the back colour it stays so */
            if (near_rgb(bufA[i], f1, 2) && !near_rgb(bufA[i - 1], f1, 2)) cyc++;
        }
        CHECK(cyc <= 1u, "wipe: front is not one run");
        render_whole(&p, 100, 3700, bufA);                            /* sweep done (4096 - 64 * 12 = 3328 et ms = 3.43 s): all c1 */
        for (i = 0; i < 100u; i++) CHECK(near_rgb(bufA[i], f0, 2), "wipe hold: pixel %u not the front colour", i);
        {                                                             /* the next cycle starts from that colour and wipes c2 over it */
            const uint32_t cyc_ms = 4096u * 256u / 248u;                  /* (a cycle, 4096 et ms, in host ms at speed 128) */
            render_whole(&p, 100, cyc_ms - 40u, bufA);
            render_whole(&p, 100, cyc_ms + 40u, bufB);
            CHECK(near_rgb(bufA[50], f0, 2) && near_rgb(bufB[99], f0, 4), "wipe: a jump at the cycle boundary");
            render_whole(&p, 100, cyc_ms + 1500u, bufB);
            CHECK(near_rgb(bufB[0], f1, 2) && near_rgb(bufB[99], f0, 2), "wipe: the second wipe goes c2 over c1");
        }
        p.custom1 = 200;                                              /* palette colours: successive wipes differ */
        {
            rgb_t c[4];
            for (k = 0; k < 4u; k++) {
                render_whole(&p, 40, 4228u * k + 3900u, bufA);       /* (the hold phase of each of 4 cycles: one colour over the strip) */
                c[k] = bufA[20];
            }
            CHECK(!rgb_eq(c[0], c[1]) && !rgb_eq(c[1], c[2]) && !rgb_eq(c[2], c[3]), "wipe palette mode: the colours repeat");
        }
    }
    /* Ripples: Rain 0 is calm water (colour 2), Rain 255 has rings, they add up to bounded colours, a ring is symmetric about its centre */
    params_of(FX_RIPPLES, &p);
    p.custom1 = 0;
    p.c[1] = fx_rgb(1, 2, 3);
    for (t = 0; t < 20000u; t += 777u) {
        render_whole(&p, 120, t, bufA);
        for (i = 0; i < 120u; i++) CHECK(rgb_eq(bufA[i], p.c[1]), "ripples rain 0 at t %u pixel %u", t, i);
    }
    p.custom1 = 255;
    {
        uint32_t frames_lit = 0;
        for (t = 0; t < 20000u; t += 250u) {
            render_whole(&p, 160, t, bufA);
            k = 0;
            for (i = 0; i < 160u; i++) k += !rgb_eq(bufA[i], p.c[1]);
            frames_lit += k > 0u;
        }
        CHECK(frames_lit >= 70u, "ripples rain 255: %u of 80 frames show rings", frames_lit);
    }
    /* Lava Lamp: blobs stay in the strip's range, move, and the background shows between them (Blob size small) */
    params_of(FX_LAVA_LAMP, &p);
    p.intensity = 0;
    p.c[1] = fx_rgb(0, 0, 0);
    for (t = 0; t < 40000u; t += 4000u) {
        render_whole(&p, 160, t, bufA);
        k = lit_count(bufA, 160);
        CHECK(k > 10u && k < 150u, "lava lamp small blobs at t %u: %u of 160 lit", t, k);
    }
    render_whole(&p, 160, 1000, bufA);
    render_whole(&p, 160, 6000, bufB);
    CHECK(memcmp(bufA, bufB, 160 * sizeof(rgb_t)) != 0, "lava lamp stands still");
}


/* ---------------------------------------------------------------------------------------------------- more effects, part 2 */
static uint32_t runs_of(const rgb_t *p, uint32_t n)       /* the number of places where a pixel differs from its left neighbour */
{
    uint32_t i, k = 0;
    for (i = 1; i < n; i++) {
        k += !rgb_eq(p[i], p[i - 1u]);
    }
    return k;
}

static void test_more2(void)
{
    fx_params_t p;
    uint32_t i, t, k;
    /* Mosaic Drift: Whiten 255 is neutral white everywhere; Whiten 0 keeps tiles of one colour (a few pixels of seam only), 100 px at Size 100 hold ~4 tiles */
    params_of(FX_MOSAIC_DRIFT, &p);
    p.custom1 = 255;
    render_whole(&p, 100, 3000, bufA);
    for (i = 0; i < 100u; i++) CHECK(near_rgb(bufA[i], fx_rgb(255, 250, 235), 1), "mosaic whiten 255: pixel %u", i);
    p.custom1 = 0; p.intensity = 100;
    for (t = 500; t < 20000u; t += 3700u) {
        render_whole(&p, 100, t, bufA);
        k = runs_of(bufA, 100);
        CHECK(k >= 1u && k <= 24u, "mosaic at t %u: %u colour changes along 100 px (tiles of ~25 px, 1 px seams)", t, k);
    }
    render_whole(&p, 100, 1000, bufA);                          /* the tiles melt: a pixel's colour changes slowly, with no jump between frames */
    {
        uint32_t worst = 0;
        for (t = 1000; t < 12000u; t += 33u) {
            render_whole(&p, 100, t + 33u, bufB);
            for (i = 0; i < 100u; i++) {
                const uint32_t d = (uint32_t)(abs((int)bufB[i].r - bufA[i].r) + abs((int)bufB[i].g - bufA[i].g) + abs((int)bufB[i].b - bufA[i].b));
                worst = d > worst ? d : worst;
            }
            memcpy(bufA, bufB, 100 * sizeof(rgb_t));
        }
        CHECK(worst < 40u, "mosaic: a pixel changes by %u in 33 ms (it should melt)", worst);
    }
    /* Silk Weave: with all three colours equal and no white, the strip is that colour; the crossings are whiter than the ribbons */
    params_of(FX_SILK_WEAVE, &p);
    p.c[0] = p.c[1] = p.c[2] = fx_rgb(40, 60, 90);
    p.custom1 = 0;
    render_whole(&p, 120, 4321, bufA);
    for (i = 0; i < 120u; i++) CHECK(rgb_eq(bufA[i], fx_rgb(40, 60, 90)), "silk, one colour: pixel %u", i);
    params_of(FX_SILK_WEAVE, &p);
    p.custom1 = 255;
    {
        uint32_t whitest = 0;
        for (t = 0; t < 8000u; t += 500u) {
            render_whole(&p, 160, t, bufA);
            for (i = 0; i < 160u; i++) { const uint32_t m = bufA[i].r < bufA[i].g ? (bufA[i].r < bufA[i].b ? bufA[i].r : bufA[i].b) : (bufA[i].g < bufA[i].b ? bufA[i].g : bufA[i].b); whitest = m > whitest ? m : whitest; }
        }
        CHECK(whitest >= 215u, "silk: the crossings never turn pearly (the whitest pixel's weakest channel is %u)", whitest);
    }
    /* Canopy Shade: Depth 0 is the pure wash; at the default most of the strip is lit (the wash) and some of it is shadow */
    params_of(FX_CANOPY_SHADE, &p);
    p.intensity = 0;
    render_whole(&p, 120, 2000, bufA);
    for (i = 0; i < 120u; i++) CHECK(rgb_eq(bufA[i], p.c[0]), "canopy depth 0: pixel %u", i);
    params_of(FX_CANOPY_SHADE, &p);
    p.intensity = 255;
    for (t = 0; t < 20000u; t += 4000u) {
        uint32_t wash = 0, dark = 0;
        render_whole(&p, 200, t, bufA);
        for (i = 0; i < 200u; i++) {
            wash += rgb_eq(bufA[i], p.c[0]);
            dark += bufA[i].r < 200u;
        }
        CHECK(wash > 100u && dark > 10u, "canopy at t %u: %u pixels pure wash, %u in shadow of 200", t, wash, dark);
    }
    /* Opal Facets: without reflection every facet is one colour for good; with it some pixels turn whiter than their facet */
    params_of(FX_OPAL_FACETS, &p);
    p.custom1 = 0;
    render_whole(&p, 100, 1000, bufA);
    render_whole(&p, 100, 7000, bufB);
    CHECK(memcmp(bufA, bufB, 100 * sizeof(rgb_t)) == 0, "opal strength 0: the facets change");
    k = runs_of(bufA, 100);
    CHECK(k >= 4u && k <= 40u, "opal: %u facet edges along 100 px", k);
    p.custom1 = 255;
    {
        uint32_t brighter = 0;
        render_whole(&p, 100, 4000, bufB);
        for (i = 0; i < 100u; i++) {
            brighter += (uint32_t)bufB[i].r + bufB[i].g + bufB[i].b > (uint32_t)bufA[i].r + bufA[i].g + bufA[i].b + 30u;
        }
        CHECK(brighter >= 3u, "opal: %u pixels carry a reflection", brighter);
    }
    /* Quiet Landings: Pearls 0 is one pearl (a few pixels lit), 255 three (more); a pearl stands on the ground at the start of a cycle */
    params_of(FX_QUIET_LANDINGS, &p);
    p.custom1 = 0;
    {
        uint32_t one = 0, three = 0;
        for (t = 0; t < 12000u; t += 400u) {
            render_whole(&p, 120, t, bufA);
            k = lit_count(bufA, 120);
            one = k > one ? k : one;
        }
        p.custom1 = 255;
        for (t = 0; t < 12000u; t += 400u) {
            render_whole(&p, 120, t, bufA);
            k = lit_count(bufA, 120);
            three = k > three ? k : three;
        }
        CHECK(one >= 3u && one <= 7u, "landings, one pearl: up to %u lit pixels", one);
        CHECK(three > one && three <= 21u, "landings, three pearls: up to %u lit pixels (one: %u)", three, one);
    }
    p.custom1 = 0;
    render_whole(&p, 120, 0, bufA);
    CHECK(bufA[0].r + bufA[0].g + bufA[0].b > 150u, "landings: the pearl does not start on the ground");
    {                                                         /* the pearl stays under Height of the strip */
        uint32_t top = 0;
        p.intensity = 128;
        for (t = 0; t < 12000u; t += 50u) {
            render_whole(&p, 120, t, bufA);
            for (i = 0; i < 120u; i++) if (bufA[i].r + bufA[i].g + bufA[i].b > 30u) top = i > top ? i : top;
        }
        CHECK(top <= 120u / 2u + 4u, "landings: a pearl at Height 128 reached pixel %u of 120", top);
    }
}

/* ---------------------------------------------------------------------------------------------------- smooth dimming (CONFIG 76) */
extern uint32_t fx_gamma16_of(uint32_t v);

/* the 8-bit gamma table's values: round(255 * (v / 255) ^ 2.8), computed here, not read from the firmware's table */
static uint32_t ref_g8(uint32_t v) { return (uint32_t)floor(255.0 * pow(v / 255.0, 2.8) + 0.5); }

/* the 8-bit packer (smoothing off) as a reference (a copy of its algorithm: brightness, gamma cubic, auto-white, order) */
static uint32_t ref_pack8(const rgb_t *in, uint8_t *wire, uint32_t n, uint8_t bri, const fx_pack_cfg_t *c)
{
    uint32_t i, sum = 0, k;
    for (i = 0; i < n; i++, wire += c->bpp) {
        uint8_t src[5];
        uint32_t r = in[i].r, g = in[i].g, b = in[i].b;
        if (bri != 255u) {
            r = fx_scale8(r, bri); g = fx_scale8(g, bri); b = fx_scale8(b, bri);
        }
        if (c->gamma) {
            r = ref_g8(r); g = ref_g8(g); b = ref_g8(b);
        }
        src[3] = src[4] = 0;
        if (c->bpp >= 4u && (c->white_mode == FX_W_ACCURATE || c->white_mode == FX_W_BRIGHT)) {
            uint32_t w = r < g ? r : g;
            w = w < b ? w : b;
            if (c->white_mode == FX_W_ACCURATE) {
                r -= w; g -= w; b -= w;
            }
            if (c->bpp == 5u) {
                src[3] = fx_scale8(w, 255u - c->white_split);
                src[4] = fx_scale8(w, c->white_split);
            } else {
                src[3] = (uint8_t)w;
            }
        }
        src[0] = (uint8_t)r; src[1] = (uint8_t)g; src[2] = (uint8_t)b;
        for (k = 0; k < c->bpp; k++) {
            wire[k] = src[c->ord[k]];
            sum += wire[k];
        }
    }
    return sum;
}

static void test_smooth(void)
{
    static const uint8_t ords[3][5] = { { 1, 0, 2, 0, 0 }, { 0, 1, 2, 3, 0 }, { 0, 1, 2, 3, 4 } };
    uint32_t v, i, k, round_, bad;
    int worst = 0;
    /* the 16-bit gamma against the exact curve: monotonic, ends exact, within ~10 of 65280 */
    for (v = 0; v < 256u; v++) {
        const int d = abs((int)fx_gamma16_of(v) - (int)floor(65280.0 * pow(v / 255.0, 2.8) + 0.5));
        worst = d > worst ? d : worst;
        if (v > 0u) {
            CHECK(fx_gamma16_of(v) >= fx_gamma16_of(v - 1u), "gamma16 not monotonic at %u", v);
        }
    }
    CHECK(fx_gamma16_of(0) == 0 && fx_gamma16_of(255) == 65280, "gamma16 ends: %u %u", fx_gamma16_of(0), fx_gamma16_of(255));
    CHECK(worst <= 1, "gamma16 is %d of 65280 off the exact curve", worst);
    printf("  gamma 16-bit: at most %d of 65280 off the exact curve\n", worst);
    /* option 3 (FX_SM_8BIT) is the old packer, byte for byte, on random frames: RGB, RGBW (accurate / bright / off), WS2805, brightness, gamma on / off */
    for (round_ = 0; round_ < 3000u; round_++) {
        fx_pack_cfg_t c;
        rgb_t px[40];
        uint8_t w1[200], w2[200];
        uint32_t s1, s2;
        const uint32_t bi = round_ % 3u;
        memset(&c, 0, sizeof(c));
        c.bpp = (uint8_t)(3u + bi);
        for (k = 0; k < 5u; k++) c.ord[k] = ords[bi][k];
        c.gamma = (uint8_t)(rnd() & 1u);
        c.white_mode = (uint8_t)(rnd() % 4u);
        c.white_split = (uint8_t)rnd();
        c.smooth = FX_SM_8BIT;
        c.phase = (uint8_t)rnd();
        c.px0 = (uint16_t)rnd();
        for (i = 0; i < 40u; i++) {
            px[i].r = (uint8_t)rnd(); px[i].g = (uint8_t)rnd(); px[i].b = (uint8_t)rnd();
        }
        k = rnd() & 255u;
        s1 = fx_pack(px, w1, 40, (uint8_t)k, &c);
        s2 = ref_pack8(px, w2, 40, (uint8_t)k, &c);
        CHECK(s1 == s2 && memcmp(w1, w2, 40u * c.bpp) == 0, "smoothing off differs from the 8-bit packer (round %u)", round_);
    }
    /* dim fades: one channel at every brightness 255 .. 0, in each mode: never a rise as the light dims, and the 16-bit paths use far more levels than 8 bits */
    {
        uint32_t levels[3];
        uint32_t mode;
        for (mode = 0; mode < 3u; mode++) {
            fx_pack_cfg_t c;
            rgb_t px = { 200, 100, 50 };
            uint8_t w[3], prev = 255;
            uint32_t seen = 0, bri;
            uint8_t used[256];
            memset(&c, 0, sizeof(c));
            memset(used, 0, sizeof(used));
            c.bpp = 3; c.ord[1] = 1; c.ord[2] = 2; c.gamma = 1; c.smooth = (uint8_t)mode;
            for (bri = 255; bri > 0u; bri--) {
                fx_pack(&px, w, 1, (uint8_t)bri, &c);
                CHECK(w[0] <= prev, "mode %u: the red channel rose from %u to %u as the light dimmed (bri %u)", mode, prev, w[0], bri);
                prev = w[0];
                if (!used[w[0]]) { used[w[0]] = 1; seen++; }
            }
            levels[mode] = seen;
        }
        CHECK(levels[1] >= levels[0] && levels[2] >= levels[0], "levels: %u (8 bit), %u (round), %u (dither)", levels[0], levels[1], levels[2]);
        printf("  dim fade of red 200: %u wire levels in the 8-bit path, %u rounded, %u dithered\n", levels[0], levels[1], levels[2]);
    }
    /* the dither: over the 256 frames of its pattern every channel's average is exactly the 16-bit value / 256 (checked on RGB, RGBW and five channels, with brightness) */
    bad = 0;
    for (round_ = 0; round_ < 400u; round_++) {
        fx_pack_cfg_t c;
        rgb_t px;
        uint8_t w[5];
        uint32_t total[5] = { 0, 0, 0, 0, 0 }, ph, g16[3], bri = 1u + rnd() % 255u, gb = fx_gamma16_of(bri), mb = gb + (gb >> 8);
        memset(&c, 0, sizeof(c));
        c.bpp = 3; c.ord[1] = 1; c.ord[2] = 2; c.gamma = 1; c.smooth = FX_SM_DITHER;
        c.px0 = (uint16_t)(rnd() % 600u);
        px.r = (uint8_t)(rnd() % 200u); px.g = (uint8_t)(rnd() % 200u); px.b = (uint8_t)(rnd() % 200u);
        for (ph = 0; ph < 256u; ph++) {
            c.phase = fx_dither_phase(ph);
            fx_pack(&px, w, 1, (uint8_t)bri, &c);
            for (k = 0; k < 3u; k++) total[k] += w[k];
        }
        g16[0] = fx_gamma16_of(px.r); g16[1] = fx_gamma16_of(px.g); g16[2] = fx_gamma16_of(px.b);
        for (k = 0; k < 3u; k++) {
            const uint32_t e = bri == 255u ? g16[k] : (g16[k] * mb + 32768u) >> 16;
            bad += total[k] != e;                                          /* sum over 256 frames of floor((e + t) / 256) = e exactly */
        }
    }
    CHECK(bad == 0u, "%u dither averages are not the 16-bit value", bad);
    /* two receivers on the same frame make the same bytes, and a frame cut into chunks equals the whole one (the pixel index decides, not the call) */
    {
        fx_pack_cfg_t c;
        rgb_t px[100];
        uint8_t wa[300], wb[300], wc[300];
        memset(&c, 0, sizeof(c));
        c.bpp = 3; c.ord[1] = 1; c.ord[2] = 2; c.gamma = 1; c.smooth = FX_SM_DITHER; c.phase = fx_dither_phase(12345u);
        for (i = 0; i < 100u; i++) { px[i].r = (uint8_t)(i * 3u); px[i].g = (uint8_t)(i * 2u + 7u); px[i].b = (uint8_t)(255u - i); }
        fx_pack(px, wa, 100, 90, &c);
        fx_pack(px, wb, 100, 90, &c);
        CHECK(memcmp(wa, wb, 300) == 0, "two receivers differ");
        c.px0 = 0; fx_pack(px, wc, 32, 90, &c);
        c.px0 = 32; fx_pack(px + 32, wc + 96, 68, 90, &c);
        CHECK(memcmp(wa, wc, 300) == 0, "a chunked frame differs from the whole frame");
    }
}

/* no scaling may be asked for more than 255 (fx_scale8 does not saturate: a factor of 260 wraps a bright channel to dark): every effect
 * x parameters x palettes x times, counted by the host build's fx_scale8 (FX_SAT_CHECK) */
static void test_saturation(void)
{
    static const uint8_t lv[3] = { 0, 100, 255 }, cu[3] = { 0, 128, 255 };
    uint32_t id, a, b, c, pal, t;
    fx_sat_over = 0;
    for (id = 1; id <= FX_LAST; id++) {
        for (a = 0; a < 3u; a++) {
            for (b = 0; b < 3u; b++) {
                for (c = 0; c < 3u; c++) {
                    for (pal = 0; pal <= PAL_COUNT; pal++) {
                        fx_params_t p;
                        params_of(id, &p);
                        p.intensity = lv[a]; p.custom1 = cu[b]; p.custom2 = cu[c];
                        p.palette = (uint8_t)(pal == PAL_COUNT ? PAL_COLORS : pal);
                        p.c[0] = fx_rgb(255, 255, 255); p.c[1] = fx_rgb(255, 255, 255); p.c[2] = fx_rgb(255, 255, 255);
                        for (t = 500; t < 9000u; t += 2300u) {
                            render_whole(&p, 60, t, bufA);
                        }
                    }
                }
            }
        }
        CHECK(fx_sat_over == 0u, "effect %u: %u scalings above 255", id, fx_sat_over);
        fx_sat_over = 0;
    }
}

/* the speed-integrated time wraps at 2^32 ms (26 days at speed 255, 51 at 128): no effect may jump there. Frames 33 ms apart across the wrap;
 * the number of pixels that change by more than 64 between two frames must not be larger at the wrap than at the other frames
 * (Fire keeps state and Sunrise / Sunset run on the raw time, which saturates: both are left out) */
/* (Ripples: a drop is born at a cycle boundary, and every power of two from 2^12 up is one: a ring of a few pixels appearing is the effect, not a jump) */
static void test_etwrap(void)
{
    static const uint32_t bits[] = { 24, 26, 28, 29, 30, 31, 32 };      /* the effect time's powers of two: an effect's own arithmetic may wrap at any of them (Gradient's did at 2^28) */
    uint32_t id, sp, v, k, n = 200, bi;
    for (id = 1; id <= FX_LAST; id++) {
        if (id == FX_FIRE || id == FX_SUNRISE || id == FX_SUNSET) {
            continue;
        }
        for (bi = 0; bi < sizeof bits / sizeof bits[0]; bi++) {
          for (sp = 0; sp < 2u; sp++) {
            for (v = 0; v < 3u; v++) {
                fx_params_t p;
                fx_ctx_t x;
                uint32_t cnt, other = 0, wrap = 0, prev_et;
                params_of(id, &p);
                p.speed = sp ? 255 : 128;
                if (v == 1u) {
                    p.custom1 = 200;                                   /* (Wipe: palette colours) */
                } else if (v == 2u) {
                    p.intensity = 200;                                 /* (Gradient: only from 16 up does it loop and scroll) */
                }
                fx_begin(&x, &p, (uint16_t)n, g_state);
                x.et = (bits[bi] == 32u ? 0u : 1u << bits[bi]) - 700u;
                x.et_frac = 0;
                fx_frame(&x, 0);
                fx_render(&x, bufA, 0, n);
                for (k = 1; k <= 60u; k++) {
                    uint32_t i;
                    prev_et = x.et;
                    fx_frame(&x, k * 33u);
                    fx_render(&x, bufB, 0, n);
                    for (cnt = 0, i = 0; i < n; i++) {
                        const int d = abs((int)bufB[i].r - bufA[i].r) + abs((int)bufB[i].g - bufA[i].g) + abs((int)bufB[i].b - bufA[i].b);
                        cnt += d > 64;
                    }
                    if (bits[bi] == 32u ? x.et < prev_et : (x.et >> bits[bi]) != (prev_et >> bits[bi])) {
                        wrap = cnt;
                    } else if (cnt > other) {
                        other = cnt;
                    }
                    memcpy(bufA, bufB, n * sizeof(rgb_t));
                }
                CHECK(wrap <= other + (id == FX_RIPPLES ? 6u : 2u), "effect %u (speed %u, custom1 %u): %u pixels jump at the effect time's 2^%u (%u at other frames)", id, p.speed, p.custom1, wrap, bits[bi], other);
            }
          }
        }
    }
}

/* ---------------------------------------------------------------------------------------------------- engine */
#define NPX 200
static fx_engine_t E;
static uint8_t pool[NPX * 2];
static rgb_t out1[NPX], out2[NPX];

static void render_strip(fx_engine_t *e, rgb_t *out, uint32_t n)
{
    uint32_t a;
    for (a = 0; a < n; a += FX_CHUNK) {
        const uint32_t b = a + FX_CHUNK < n ? a + FX_CHUNK : n;
        fx_engine_render(e, out + a, a, b);
    }
}
static void solid_params(fx_params_t *p, rgb_t c, uint8_t bri, uint16_t trans)
{
    params_of(FX_SOLID, p);
    p->c[0] = c;
    p->bri = bri;
    p->trans_ms = trans;
}

static void test_engine(void)
{
    fx_params_t p, q;
    uint32_t i, t;
    const rgb_t RED = { 255, 0, 0 }, BLUE = { 0, 0, 255 }, GREEN = { 0, 255, 0 };
    /* a plain full-strip effect through the engine = the effect alone */
    fx_engine_init(&E, NPX, pool, sizeof(pool));
    params_of(FX_RAINBOW, &p);
    fx_engine_set(&E, 0, &p, 1000, 1000);
    fx_engine_frame(&E, 3500);
    render_strip(&E, out1, NPX);
    {
        fx_params_t r = p;
        r.seed = p.seed;
        render_whole(&r, NPX, 2500, out2);
        CHECK(memcmp(out1, out2, sizeof(out1)) == 0, "engine, one segment: differs from the effect alone");
    }
    CHECK(!fx_engine_dark(&E), "engine reports dark with a lit effect");
    /* two segments, a gap, len 0 = to the end, a later segment over an earlier one */
    fx_engine_init(&E, NPX, pool, sizeof(pool));
    solid_params(&p, RED, 255, 0);
    p.seg_start = 10;
    p.seg_len = 40;
    fx_engine_set(&E, 0, &p, 0, 0);
    solid_params(&q, BLUE, 255, 0);
    q.seg_start = 150;
    q.seg_len = 0;
    fx_engine_set(&E, 1, &q, 0, 0);
    fx_engine_frame(&E, 10);
    render_strip(&E, out1, NPX);
    for (i = 0; i < NPX; i++) {
        const rgb_t want = (i >= 10 && i < 50) ? RED : (i >= 150 ? BLUE : fx_rgb(0, 0, 0));
        CHECK(rgb_eq(out1[i], want), "segments: pixel %u is %u,%u,%u", i, out1[i].r, out1[i].g, out1[i].b);
    }
    solid_params(&q, GREEN, 255, 0);
    q.seg_start = 30;
    q.seg_len = 40;
    fx_engine_set(&E, 2, &q, 10, 0);
    fx_engine_frame(&E, 20);
    render_strip(&E, out1, NPX);
    CHECK(rgb_eq(out1[29], RED) && rgb_eq(out1[30], GREEN) && rgb_eq(out1[69], GREEN) && rgb_eq(out1[70], fx_rgb(0, 0, 0)), "overlap: the later segment is on top");
    solid_params(&q, GREEN, 255, 0);
    q.seg_start = 190;
    q.seg_len = 100;                                                   /* runs past the strip: clipped */
    fx_engine_set(&E, 3, &q, 20, 0);
    fx_engine_frame(&E, 30);
    render_strip(&E, out1, NPX);
    CHECK(rgb_eq(out1[199], GREEN) && rgb_eq(out1[190], GREEN), "a segment past the end is clipped");
    fx_engine_set(&E, 3, &q, 30, 0);                                   /* stream: inactive */
    q.id = FX_STREAM;
    fx_engine_set(&E, 3, &q, 30, 0);
    render_strip(&E, out1, NPX);
    CHECK(rgb_eq(out1[195], BLUE), "a segment set to stream shows what is below it");
    /* brightness */
    fx_engine_init(&E, NPX, pool, sizeof(pool));
    solid_params(&p, fx_rgb(200, 100, 50), 128, 0);
    fx_engine_set(&E, 0, &p, 0, 0);
    fx_engine_frame(&E, 5);
    render_strip(&E, out1, 10);
    CHECK(near_rgb(out1[3], fx_rgb(100, 50, 25), 1), "segment brightness: %u,%u,%u", out1[3].r, out1[3].g, out1[3].b);
    /* reverse and mirror */
    {
        static rgb_t plain[NPX], rev[NPX], mir[NPX];
        fx_params_t a;
        uint32_t len;
        for (len = 2; len <= NPX; len += (len < 20u ? 1u : 37u)) {
            fx_engine_init(&E, NPX, pool, sizeof(pool));
            params_of(FX_RAINBOW, &a);
            a.seg_len = (uint16_t)len;
            fx_engine_set(&E, 0, &a, 0, 0);
            fx_engine_frame(&E, 2000);
            render_strip(&E, plain, len);
            a.flags = FX_F_REVERSE;
            fx_engine_init(&E, NPX, pool, sizeof(pool));
            fx_engine_set(&E, 0, &a, 0, 0);
            fx_engine_frame(&E, 2000);
            render_strip(&E, rev, len);
            for (i = 0; i < len; i++) {
                if (!rgb_eq(rev[i], plain[len - 1u - i])) {
                    CHECK(0, "reverse: pixel %u of %u", i, len);
                    break;
                }
            }
            a.flags = FX_F_MIRROR;
            fx_engine_init(&E, NPX, pool, sizeof(pool));
            fx_engine_set(&E, 0, &a, 0, 0);
            fx_engine_frame(&E, 2000);
            render_strip(&E, mir, len);
            for (i = 0; i < len; i++) {
                if (!rgb_eq(mir[i], mir[len - 1u - i])) {
                    CHECK(0, "mirror: pixel %u of %u is not symmetric", i, len);
                    break;
                }
            }
            {                                                          /* the left half is the effect on half the length */
                fx_params_t h = a;
                h.flags = 0;
                h.seg_len = (uint16_t)((len + 1u) >> 1);
                fx_engine_init(&E, NPX, pool, sizeof(pool));
                fx_engine_set(&E, 0, &h, 0, 0);
                fx_engine_frame(&E, 2000);
                render_strip(&E, plain, h.seg_len);
                for (i = 0; i < h.seg_len; i++) {
                    if (!rgb_eq(mir[i], plain[i])) {
                        CHECK(0, "mirror: pixel %u of %u differs from the half-length effect", i, len);
                        break;
                    }
                }
            }
        }
    }
    /* brightness ramp, colour ramp */
    fx_engine_init(&E, NPX, pool, sizeof(pool));
    solid_params(&p, RED, 255, 0);
    fx_engine_set(&E, 0, &p, 0, 0);
    fx_engine_frame(&E, 0);
    solid_params(&q, BLUE, 255, 1000);
    q.bri = 51;
    fx_engine_set(&E, 0, &q, 100, 0);
    {
        uint8_t last_r = 255;
        for (t = 100; t <= 1100u; t += 50u) {
            fx_engine_frame(&E, t);
            render_strip(&E, out1, 4);
            CHECK(out1[0].r <= last_r, "colour ramp: red goes up at %u", t);
            last_r = out1[0].r;
            if (t == 600u) {
                const rgb_t want = fx_rgb(127 * 153 / 255, 0, 127 * 153 / 255);
                CHECK(near_rgb(out1[0], want, 5), "ramp at the middle: %u,%u,%u wanted ~%u,0,%u", out1[0].r, out1[0].g, out1[0].b, want.r, want.b);
            }
        }
        CHECK(near_rgb(out1[0], fx_rgb(0, 0, 51), 1), "ramp end: %u,%u,%u", out1[0].r, out1[0].g, out1[0].b);
        CHECK(!fx_engine_dark(&E), "not dark at brightness 51");
    }
    /* off: brightness 0 with a transition: dark only once it is done */
    solid_params(&q, BLUE, 0, 400);
    fx_engine_set(&E, 0, &q, 2000, 0);
    fx_engine_frame(&E, 2100);
    CHECK(!fx_engine_dark(&E), "dark too early");
    fx_engine_frame(&E, 2500);
    render_strip(&E, out1, 4);
    CHECK(fx_engine_dark(&E) && sumrgb(out1, 4) == 0, "dark after the fade out");
    /* from dark, fading in */
    solid_params(&q, GREEN, 255, 500);
    fx_engine_set(&E, 0, &q, 3000, 0);
    fx_engine_frame(&E, 3250);
    render_strip(&E, out1, 4);
    CHECK(near_rgb(out1[0], fx_rgb(0, 127, 0), 3), "fade in at the middle: %u", out1[0].g);
    fx_engine_frame(&E, 3600);
    render_strip(&E, out1, 4);
    CHECK(rgb_eq(out1[0], GREEN), "fade in done");
    /* crossfade between two effects: red -> blue; the old one is released */
    fx_engine_init(&E, NPX, pool, sizeof(pool));
    solid_params(&p, RED, 255, 0);
    fx_engine_set(&E, 0, &p, 0, 0);
    fx_engine_frame(&E, 0);
    params_of(FX_COLORLOOP, &q);
    q.intensity = 255;
    q.speed = 0;                                                       /* hue 0 stays red: use the rainbow instead */
    solid_params(&q, BLUE, 255, 1000);
    q.id = FX_BREATHE;                                                 /* another effect id: breathe depth 0 = a solid colour */
    q.intensity = 0;
    q.c[0] = BLUE;
    fx_engine_set(&E, 0, &q, 100, 0);
    fx_engine_frame(&E, 100);
    render_strip(&E, out1, 4);
    CHECK(near_rgb(out1[0], RED, 2), "crossfade start shows the old effect");
    CHECK(E.seg[0].xf >= 0, "crossfade slot taken");
    fx_engine_frame(&E, 600);
    render_strip(&E, out1, 4);
    CHECK(near_rgb(out1[0], fx_rgb(127, 0, 127), 4), "crossfade middle: %u,%u,%u", out1[0].r, out1[0].g, out1[0].b);
    fx_engine_frame(&E, 1200);
    render_strip(&E, out1, 4);
    CHECK(near_rgb(out1[0], BLUE, 1) && E.seg[0].xf < 0 && !E.xf[0].used, "crossfade done and released: %u,%u,%u", out1[0].r, out1[0].g, out1[0].b);
    /* only FX_MAX_XFADE crossfades at once; the next switches hard */
    fx_engine_init(&E, NPX, pool, sizeof(pool));
    for (i = 0; i < 3u; i++) {
        solid_params(&p, RED, 255, 0);
        p.seg_start = (uint16_t)(i * 20u);
        p.seg_len = 20;
        fx_engine_set(&E, i, &p, 0, 0);
    }
    fx_engine_frame(&E, 0);
    for (i = 0; i < 3u; i++) {
        solid_params(&q, BLUE, 255, 1000);
        q.id = FX_BREATHE;
        q.intensity = 0;
        q.seg_start = (uint16_t)(i * 20u);
        q.seg_len = 20;
        fx_engine_set(&E, i, &q, 10, 0);
    }
    fx_engine_frame(&E, 10);
    render_strip(&E, out1, 60);
    CHECK(near_rgb(out1[0], RED, 2) && near_rgb(out1[20], RED, 2) && near_rgb(out1[40], BLUE, 2), "the third crossfade switches hard (%u,%u,%u)", out1[40].r, out1[40].g, out1[40].b);
    /* Fire keeps its heat while fading, including a queued palette change. */
    fx_engine_init(&E, NPX, pool, sizeof(pool));
    params_of(FX_FIRE, &p);
    fx_engine_set(&E, 0, &p, 0, 0);
    for (t = 0; t < 3000u; t += 20u) {
        fx_engine_frame(&E, t);
    }
    render_strip(&E, out1, NPX);
    CHECK(sumrgb(out1, 40) > 300, "engine fire burns");
    params_of(FX_RAINBOW, &q);
    q.trans_ms = 500;
    fx_engine_set(&E, 0, &q, 3000, 0);
    CHECK(E.seg[0].xf >= 0, "fire -> rainbow crossfades");
    for (t = 3000; t < 3600u; t += 20u) {
        fx_engine_frame(&E, t);
        render_strip(&E, out1, NPX);
    }
    CHECK(E.seg[0].xf < 0, "fire's crossfade ended");
    params_of(FX_FIRE, &p);
    p.trans_ms = 500;
    fx_engine_set(&E, 0, &p, 3600, 0);
    p.palette = PAL_LAVA;
    fx_engine_set(&E, 0, &p, 3700, 0);
    CHECK(E.seg[0].xf >= 0 && E.xf[E.seg[0].xf].used == 3, "Fire palette change waits for the running fade");
    fx_engine_frame(&E, 3950);
    CHECK(E.seg[0].xf >= 0 && E.xf[E.seg[0].xf].prev.state == E.seg[0].cur.state,
          "Fire palette crossfade must share heat without clearing or double-stepping it");
    {
        uint8_t tiny[10];
        fx_engine_init(&E, NPX, tiny, sizeof(tiny));
        params_of(FX_FIRE, &p);
        fx_engine_set(&E, 0, &p, 0, 0);
        fx_engine_frame(&E, 100);
        render_strip(&E, out1, NPX);
        CHECK(rgb_eq(out1[10], p.c[0]), "fire without pool room shows colour 1");
    }
    /* the whole strip rendered in shuffled chunks equals the in-order render, with crossfade, reverse and mirror live */
    fx_engine_init(&E, NPX, pool, sizeof(pool));
    params_of(FX_PRIDE, &p);
    p.seg_len = 120;
    p.flags = FX_F_REVERSE;
    fx_engine_set(&E, 0, &p, 0, 0);
    params_of(FX_AURORA, &q);
    q.seg_start = 120;
    q.flags = FX_F_MIRROR;
    fx_engine_set(&E, 1, &q, 0, 0);
    fx_engine_frame(&E, 4000);
    params_of(FX_COLORWAVES, &p);
    p.seg_len = 120;
    p.flags = FX_F_REVERSE;
    p.trans_ms = 2000;
    fx_engine_set(&E, 0, &p, 4000, 0);
    fx_engine_frame(&E, 5000);
    CHECK(E.seg[0].xf >= 0, "crossfade live for the shuffle test");
    render_strip(&E, out1, NPX);
    {
        uint32_t order[8] = { 5, 2, 7, 0, 3, 6, 1, 4 }, k;
        for (k = 0; k < 8u; k++) {
            const uint32_t a = order[k] * 25u, b = a + 25u;
            fx_engine_render(&E, out2 + a, a, b);
        }
        CHECK(memcmp(out1, out2, sizeof(out1)) == 0, "engine: shuffled chunks differ from the in-order render");
    }
    /* a time-driven effect with its anchor in the past (the receiver joins a running sunrise) */
    fx_engine_init(&E, NPX, pool, sizeof(pool));
    params_of(FX_SUNRISE, &p);
    p.speed = 20;
    p.intensity = 0;
    fx_engine_set(&E, 0, &p, 100000, 100000u - 290000u / 2u);
    fx_engine_frame(&E, 100000);
    render_strip(&E, out1, 4);
    {
        fx_params_t r = p;
        render_whole(&r, 4, 290000u / 2u, out2);
        CHECK(rgb_eq(out1[0], out2[0]), "sunrise joined half way shows the half-way colour");
    }
    /* a new anchor with another change (here the seed) restarts the sun */
    p.seed ^= 1u;
    fx_engine_set(&E, 0, &p, 100000, 100000u);
    fx_engine_frame(&E, 100000);
    render_strip(&E, out1, 4);
    CHECK(sumrgb(out1, 4) == 0, "a new anchor with another change restarts the sunrise");
    /* segment seeds differ: two random segments are not copies */
    fx_engine_init(&E, NPX, pool, sizeof(pool));
    params_of(FX_COLORTWINKLES, &p);
    p.seg_len = 100;
    p.intensity = 200;
    fx_engine_set(&E, 0, &p, 0, 0);
    p.seg_start = 100;
    p.seg_len = 100;
    fx_engine_set(&E, 1, &p, 0, 0);
    fx_engine_frame(&E, 5000);
    render_strip(&E, out1, NPX);
    CHECK(memcmp(out1, out1 + 100, 100 * sizeof(rgb_t)) != 0, "two random segments with one seed are identical");
}

/* the hub renews PIXEL_FX's signed 32-bit START about every 20 days: the same slot with only a new anchor must be invisible */
static void test_renewal(void)
{
    uint32_t id, t, k;
    rng_s = 0x2468ACE1u;
    for (id = 1; id <= FX_LAST; id++) {
        static const uint32_t ages[3] = { 1700000000u, 100000u, 7u };         /* the anchor is 19.7 days / 100 s / 7 ms old */
        uint32_t ai;
        for (ai = 0; ai < 3u; ai++) {
            static fx_engine_t A, B;
            static uint8_t poolA[1024], poolB[1024];
            static rgb_t ra[NPX], rb[NPX];
            fx_params_t p;
            const uint32_t now0 = 3000000000u, anchor = now0 - ages[ai];
            params_of(id, &p);
            p.seg_len = 150;
            p.flags = (id & 1u) ? FX_F_REVERSE : 0u;
            if (id == FX_SUNRISE || id == FX_SUNSET) {
                p.speed = 40;                                                  /* 570 s */
            }
            fx_engine_init(&A, NPX, poolA, sizeof(poolA));
            fx_engine_init(&B, NPX, poolB, sizeof(poolB));
            fx_engine_set(&A, 0, &p, now0, anchor);
            fx_engine_set(&B, 0, &p, now0, anchor);
            for (t = now0; t <= now0 + 5000u; t += 33u) {
                fx_engine_frame(&A, t);
                fx_engine_frame(&B, t);
            }
            fx_engine_set(&B, 0, &p, t, t - 100u);                             /* the renewal: START re-sent, new anchor */
            fx_engine_frame(&B, t);
            fx_engine_frame(&A, t);
            render_strip(&A, ra, 150);
            render_strip(&B, rb, 150);
            CHECK(memcmp(ra, rb, 150 * sizeof(rgb_t)) == 0, "effect %u (anchor age %u): the picture jumps at a renewal", id, ages[ai]);
            for (k = 0; k < 400u; k++) {                                       /* and goes on the same */
                t += 33u;
                fx_engine_frame(&A, t);
                fx_engine_frame(&B, t);
            }
            render_strip(&A, ra, 150);
            render_strip(&B, rb, 150);
            CHECK(memcmp(ra, rb, 150 * sizeof(rgb_t)) == 0, "effect %u (anchor age %u): the picture differs after a renewal", id, ages[ai]);
        }
    }
    {                                                                          /* an effect runs on past 2^31 and 2^32 ms of host time */
        static fx_engine_t C;
        static uint8_t poolC[8];
        static rgb_t r0[NPX], r1[NPX];
        uint32_t steps, moved = 0, anchor = 5u, now = 5u + 100u;
        fx_params_t p;
        params_of(FX_RAINBOW, &p);
        p.seg_len = 100;
        fx_engine_init(&C, NPX, poolC, sizeof(poolC));
        fx_engine_set(&C, 0, &p, now, anchor);
        fx_engine_frame(&C, now);
        render_strip(&C, r0, 100);
        for (steps = 0; steps < 2u * 86400u * 60u; steps++) {                  /* 60 days in 30 s steps */
            now += 30000u;
            fx_engine_frame(&C, now);
            if (steps % 1000u == 0u || now - anchor < 2200000000u + 60000u) {
                render_strip(&C, r1, 100);
                moved += memcmp(r0, r1, 100 * sizeof(rgb_t)) != 0;
                memcpy(r0, r1, sizeof(r0));
            }
            if (steps % 57600u == 56000u) {                                    /* a renewal every 20 days */
                fx_engine_set(&C, 0, &p, now, now);
            }
        }
        CHECK(moved > 1500, "60 days: the picture stopped moving (%u of the checked frames changed)", moved);
        CHECK(C.seg[0].cur.t > 3000000000u, "60 days: the raw effect time did not keep counting (%u)", C.seg[0].cur.t);
    }
}

/* ---------------------------------------------------------------------------------------------------- packer */
static void test_pack(void)
{
    static const rgb_t in[3] = { { 100, 150, 200 }, { 10, 20, 30 }, { 255, 255, 255 } };
    uint8_t w[20];
    fx_pack_cfg_t c;
    uint32_t sum, k;
    memset(&c, 0, sizeof(c));
    c.bpp = 3; c.ord[0] = 1; c.ord[1] = 0; c.ord[2] = 2;
    sum = fx_pack(in + 1, w, 1, 255, &c);
    CHECK(w[0] == 20 && w[1] == 10 && w[2] == 30 && sum == 60, "GRB: %u %u %u sum %u", w[0], w[1], w[2], sum);
    c.gamma = 1;
    sum = fx_pack(in, w, 1, 255, &c);
    CHECK(w[0] == fx_gamma(150) && w[1] == fx_gamma(100) && w[2] == fx_gamma(200), "gamma applied");
    CHECK(sum == (uint32_t)w[0] + w[1] + w[2], "sum of the wire bytes");
    sum = fx_pack(in, w, 1, 128, &c);
    CHECK(w[1] == fx_gamma(fx_scale8(100, 128)) && w[0] == fx_gamma(fx_scale8(150, 128)), "brightness before gamma");
    c.gamma = 0;
    /* RGBW accurate: W = min taken out */
    c.bpp = 4; c.ord[0] = 0; c.ord[1] = 1; c.ord[2] = 2; c.ord[3] = 3; c.white_mode = FX_W_ACCURATE;
    sum = fx_pack(in, w, 1, 255, &c);
    CHECK(w[0] == 0 && w[1] == 50 && w[2] == 100 && w[3] == 100 && sum == 250, "RGBW accurate: %u %u %u %u", w[0], w[1], w[2], w[3]);
    c.ord[0] = 3; c.ord[1] = 0; c.ord[2] = 1; c.ord[3] = 2;               /* WRGB */
    fx_pack(in, w, 1, 255, &c);
    CHECK(w[0] == 100 && w[1] == 0 && w[2] == 50 && w[3] == 100, "WRGB: %u %u %u %u", w[0], w[1], w[2], w[3]);
    c.ord[0] = 1; c.ord[1] = 0; c.ord[2] = 2; c.ord[3] = 3;               /* SK6812 GRBW */
    c.white_mode = FX_W_BRIGHT;
    sum = fx_pack(in, w, 1, 255, &c);
    CHECK(w[0] == 150 && w[1] == 100 && w[2] == 200 && w[3] == 100 && sum == 550, "GRBW brighter keeps RGB");
    c.white_mode = FX_W_OFF;
    fx_pack(in, w, 1, 255, &c);
    CHECK(w[3] == 0 && w[1] == 100, "no auto-white");
    /* white of a pure colour is 0 */
    c.white_mode = FX_W_ACCURATE;
    {
        const rgb_t red = { 255, 0, 0 };
        fx_pack(&red, w, 1, 255, &c);
        CHECK(w[3] == 0 && w[1] == 255, "pure red has no white");
    }
    /* WS2805: the white split between warm and cool */
    c.bpp = 5; c.ord[0] = 0; c.ord[1] = 1; c.ord[2] = 2; c.ord[3] = 3; c.ord[4] = 4;
    c.white_split = 0;
    fx_pack(in + 2, w, 1, 255, &c);
    CHECK(w[0] == 0 && w[3] == 255 && w[4] == 0, "WS2805 all warm: %u %u %u %u %u", w[0], w[1], w[2], w[3], w[4]);
    c.white_split = 255;
    fx_pack(in + 2, w, 1, 255, &c);
    CHECK(w[3] == 0 && w[4] == 255, "WS2805 all cool");
    c.white_split = 128;
    sum = fx_pack(in + 2, w, 1, 255, &c);
    CHECK(abs((int)w[3] + w[4] - 255) <= 1 && sum == (uint32_t)w[3] + w[4], "WS2805 split sums to the white (%u + %u)", w[3], w[4]);
    /* every order: the multiset of wire bytes is the same, many pixels, wire positions per pixel */
    {
        uint8_t o[5] = { 4, 2, 0, 3, 1 }, big[40];
        rgb_t px[8];
        for (k = 0; k < 8u; k++) {
            px[k] = fx_rgb(k * 30u, 255u - k * 20u, k * 7u + 5u);
        }
        memcpy(c.ord, o, 5);
        c.white_split = 90;
        sum = fx_pack(px, big, 8, 200, &c);
        {
            uint32_t s2 = 0;
            for (k = 0; k < 40u; k++) {
                s2 += big[k];
            }
            CHECK(s2 == sum, "sum over 8 pixels of 5 channels");
        }
        c.ord[0] = 0; c.ord[1] = 1; c.ord[2] = 2; c.ord[3] = 3; c.ord[4] = 4;
        {
            uint8_t ref[40];
            fx_pack(px, ref, 8, 200, &c);
            for (k = 0; k < 8u; k++) {
                CHECK(big[k * 5u + 0u] == ref[k * 5u + 4u] && big[k * 5u + 1u] == ref[k * 5u + 2u] && big[k * 5u + 2u] == ref[k * 5u + 0u] &&
                      big[k * 5u + 3u] == ref[k * 5u + 3u] && big[k * 5u + 4u] == ref[k * 5u + 1u], "order permutation, pixel %u", k);
            }
        }
    }
    /* the limiter */
    {
        static rgb_t frame[300];
        static uint8_t wire[900];
        uint32_t s0, i, max_sum;
        uint8_t b;
        for (i = 0; i < 300u; i++) {
            frame[i] = fx_rgb(255, 200, (i * 3u) & 255u);
        }
        c.bpp = 3; c.ord[0] = 1; c.ord[1] = 0; c.ord[2] = 2; c.gamma = 1; c.white_mode = 0;
        s0 = fx_pack(frame, wire, 300, 255, &c);
        CHECK(fx_limit_bri(255, s0, s0) == 255 && fx_limit_bri(200, s0, s0 + 1000u) == 200, "limiter leaves a frame under the limit");
        for (max_sum = s0 / 8u; max_sum < s0; max_sum += s0 / 8u) {
            uint32_t s1;
            b = fx_limit_bri(255, s0, max_sum);
            s1 = fx_pack(frame, wire, 300, b, &c);
            CHECK(s1 <= max_sum * 5u / 4u && s1 >= max_sum / 2u, "limiter: wanted %u, got %u at brightness %u", max_sum, s1, b);
            CHECK(b < 255, "limiter did not dim");
        }
        CHECK(fx_limit_bri(255, s0, s0 / 4u) <= fx_limit_bri(255, s0, s0 / 2u), "limiter monotonic");
    }
}

/* ---------------------------------------------------------------------------------------------------- sizes */
static void report_sizes(void)
{
    printf("  sizeof: rgb_t %u, fx_ctx_t %u, fx_params_t %u, fx_seg_t %u, fx_xfade_t %u, fx_engine_t %u (FX_MAX_SEG %u, FX_MAX_XFADE %u)\n",
           (unsigned)sizeof(rgb_t), (unsigned)sizeof(fx_ctx_t), (unsigned)sizeof(fx_params_t), (unsigned)sizeof(fx_seg_t),
           (unsigned)sizeof(fx_xfade_t), (unsigned)sizeof(fx_engine_t), FX_MAX_SEG, FX_MAX_XFADE);
}

/* Long legal transitions must approach the target before their final frame, also across host-ms wrap. */
static void test_long_transitions(void)
{
    static const uint16_t durations[] = { 1000, 10000, 32769, 40000, 60000, 65535 };
    fx_engine_t e;
    fx_params_t p;
    uint32_t j, t, now = 0xFFFFF000u;
    for (j = 0; j < sizeof durations / sizeof durations[0]; j++) {
        const uint32_t dur = durations[j];
        params_of(FX_SOLID, &p);
        p.c[0] = fx_rgb(255, 0, 0);
        p.trans_ms = (uint16_t)dur;
        fx_engine_init(&e, 8, 0, 0);
        fx_engine_set(&e, 0, &p, now, now);
        for (t = 0; t <= dur; t += dur / 7u) {
            int expected = (int)(255u * t / dur);
            fx_engine_frame(&e, now + t);
            CHECK(abs((int)e.seg[0].bri - expected) <= 1, "long ramp dur=%u t=%u bri=%u expected=%d", dur, t, e.seg[0].bri, expected);
        }
        fx_engine_frame(&e, now + dur - 1u);
        CHECK(e.seg[0].bri >= 253u, "long ramp final step dur=%u bri=%u", dur, e.seg[0].bri);
        fx_engine_frame(&e, now + dur);
        p.id = FX_BREATHE;
        fx_engine_set(&e, 0, &p, now + dur, now);
        fx_engine_frame(&e, now + 2u * dur - 1u);
        CHECK(e.seg[0].xf >= 0 && e.xf[e.seg[0].xf].w >= 254u, "long crossfade final step dur=%u", dur);
    }
}

/* Independent scalar oracle: the original Fire 2012 update and four-step cap. */
static void fire_reference(fx_ctx_t *x, uint32_t de)
{
    uint32_t acc = x->u[0] + de, steps = 0, n = fire_cells(x), k;
    if (!x->state || n < 8u) return;
    while (acc >= 30u && steps < 4u) {
        const uint32_t cool = (uint32_t)x->custom1 * 10u / n + 2u;
        acc -= 30u; steps++;
        if (x->fire_prev) memcpy(x->fire_prev, x->state, n);
        for (k = 0; k < n; k++) {
            x->u[1] = xorshift(x->u[1]);
            x->state[k] = fx_qsub8(x->state[k], ((x->u[1] & 255u) * cool) >> 8);
        }
        for (k = n - 1u; k >= 2u; k--)
            x->state[k] = (uint8_t)(((uint32_t)x->state[k - 1u] + 2u * x->state[k - 2u]) * 85u >> 8);
        x->u[1] = xorshift(x->u[1]);
        if ((x->u[1] & 255u) < x->intensity) {
            k = ((x->u[1] >> 8) & 255u) * 7u >> 8;
            x->state[k] = fx_qadd8(x->state[k], 160u + ((((x->u[1] >> 16) & 255u) * 95u) >> 8));
        }
    }
    x->u[0] = acc >= 30u ? 0u : acc;
}

static void test_fire_slices(void)
{
    const uint16_t lengths[] = {0, 1, 7, 8, 9, 63, 64, 65, 255, 256, 600};
    const uint8_t speeds[] = {0, 1, 16, 64, 128, 255};
    uint32_t li, si, f, k;
    for (li = 0; li < sizeof lengths / sizeof lengths[0]; li++) {
        for (si = 0; si < 2u * sizeof speeds; si++) {
            fx_ctx_t x, ref;
            fx_params_t p;
            uint8_t heat[256], expected[256], before[256];
            uint8_t prev[256] = {0}, expected_prev[256] = {0};
            rgb_t whole[600], shuffled[600], want[600];
            const uint32_t n = lengths[li], cells = n < 256u ? n : 256u;
            uint32_t t = 0xFFFFF000u;
            params_of(FX_FIRE, &p); p.speed = speeds[si % sizeof speeds];
            fx_begin(&x, &p, (uint16_t)n, heat);
            fx_begin(&ref, &p, (uint16_t)n, expected);
            if (si >= sizeof speeds) { x.fire_prev = prev; ref.fire_prev = expected_prev; }
            x.t_last = t;                         /* exercise host-clock rollover */
            for (f = 0; f < 120; f++) {
                uint32_t slices = 0, et = x.et;
                t += f % 11u == 0u ? 500u : 17u;   /* four-step catch-up and drop */
                memcpy(before, heat, cells);
                fx_frame_begin(&x, t);
                CHECK(memcmp(before, heat, cells) == 0, "Fire begin ran heat work");
                fire_reference(&ref, x.et - et);
                ref.et_frac = x.et_frac;
                while (fx_frame_step(&x)) {
                    uint32_t changed = 0;
                    for (k = 0; k < cells; k++) changed += before[k] != heat[k];
                    CHECK(changed <= 65u, "Fire slice changed %u cells", changed);
                    memcpy(before, heat, cells);
                    CHECK(++slices <= 32u, "Fire catch-up is unbounded");
                    if (slices > 32u) break;
                }
                CHECK(memcmp(heat, expected, cells) == 0 && x.u[0] == ref.u[0] && x.u[1] == ref.u[1],
                      "Fire heat/RNG changed n=%u speed=%u frame=%u", n, x.speed, f);
                CHECK(memcmp(prev, expected_prev, cells) == 0, "Fire previous heat differs after catch-up");
                fx_render(&x, whole, 0, n);
                fx_render(&ref, want, 0, n);
                /* Reverse chunk order, with different boundaries every frame. */
                for (k = n; k; ) {
                    const uint32_t lo = k > 1u + f % 32u ? k - (1u + f % 32u) : 0;
                    fx_render(&x, shuffled + lo, lo, k); k = lo;
                }
                CHECK(memcmp(whole, want, n * sizeof(rgb_t)) == 0 && memcmp(whole, shuffled, n * sizeof(rgb_t)) == 0,
                      "Fire slice/chunk/order changed the picture");
                /* No jump on a speed change; subsequent steps use the new rate. */
                fx_set_speed(&x, speeds[(si + f / 20u) % sizeof speeds]);
                fx_render(&x, shuffled, 0, n);
                CHECK(memcmp(whole, shuffled, n * sizeof(rgb_t)) == 0, "Fire speed change jumps");
                x.custom1 = ref.custom1 = (uint8_t)(f * 37u);
                x.intensity = ref.intensity = (uint8_t)(f * 19u);
            }
        }
    }
    /* Commands (including changes of cooling/sparking/speed), fades in/out and
     * palette sharing must match the synchronous API, including queued fades. */
    {
        fx_engine_t a, b;
        fx_params_t p;
        uint8_t ha[1024] = {0}, hb[1024] = {0};
        rgb_t wa[32], wb[32];
        fx_engine_init(&a, 600, ha, sizeof ha);
        fx_engine_init(&b, 600, hb, sizeof hb); b.deferred = 1;
        params_of(FX_FIRE, &p);
        for (f = 0; f < 160; f++) {
            const uint32_t t = f * 47u;
            if (f % 13u == 0u) {
                p.id = f % 3u ? FX_FIRE : FX_RAINBOW;
                p.palette = f % 2u ? PAL_LAVA : PAL_HEAT;
                p.speed = (uint8_t)(f * 17u); p.intensity = (uint8_t)(f * 11u); p.custom1 = (uint8_t)(f * 29u);
                p.trans_ms = 2000;
                fx_engine_set(&a, 0, &p, t, 0);
                fx_engine_set(&b, 0, &p, t, 0);
                while (fx_engine_step(&b)) { }
            }
            fx_engine_frame(&a, t); fx_engine_frame(&b, t);
            CHECK(memcmp(ha, hb, 256) == 0, "deferred Fire heat differs");
            for (k = 0; k < 600; k += 32) {
                const uint32_t end = k + 32 < 600 ? k + 32 : 600;
                fx_engine_render(&a, wa, k, end); fx_engine_render(&b, wb, k, end);
                CHECK(memcmp(wa, wb, (end - k) * sizeof(rgb_t)) == 0, "deferred Fire command/fade differs");
            }
        }
    }
}

static void test_fire_smoothing(void)
{
    fx_engine_t e;
    fx_params_t p;
    uint8_t pool[(FX_MAX_SEG + FX_FIRE_SMOOTH) * FX_FIRE_CELLS + 16];
    uint8_t saved[512];
    rgb_t before[4][600], after[600];
    uint32_t i, t, changed[4] = {0}, unchanged_heat = 0;
    memset(pool, 0xA5, sizeof pool);
    fx_engine_init(&e, 600, pool, sizeof pool - 16);
    params_of(FX_FIRE, &p); p.speed = 16;
    /* Reverse command order must not affect slot assignment. */
    for (i = 4; i--;) fx_engine_set(&e, i, &p, 0, 0);
    CHECK(e.seg[0].cur.fire_prev == pool + 1024 && e.seg[1].cur.fire_prev == pool + 1280 &&
          !e.seg[2].cur.fire_prev && !e.seg[3].cur.fire_prev, "Fire smoothing slots are not fixed to indices 0/1");
    for (t = 0; t <= 4000; t += 10) {
        const uint32_t rng = e.seg[0].cur.u[1];
        fx_engine_frame(&e, t);
        for (i = 0; i < 4; i++) {
            fx_render(&e.seg[i].cur, after, 0, 600);
            if (t > 2000 && rng == e.seg[0].cur.u[1])
                changed[i] += memcmp(before[i], after, sizeof after) != 0;
            memcpy(before[i], after, sizeof after);
        }
        if (t > 2000 && rng == e.seg[0].cur.u[1]) unchanged_heat++;
    }
    CHECK(changed[0] > 100 && changed[1] > 100 && changed[2] == 0 && changed[3] == 0,
          "Fire between-step motion: %u/%u/%u/%u of %u intervals", changed[0], changed[1], changed[2], changed[3], unchanged_heat);
    printf("  Fire between-step picture changes: %u/%u/%u/%u of %u (slots 0/1 smooth)\n",
           changed[0], changed[1], changed[2], changed[3], unchanged_heat);
    /* Freeze a warmed, interpolated picture, then resume without a phase jump. */
    p.speed = 0;
    for (i = 0; i < 4; i++) fx_engine_set(&e, i, &p, 4000, 0);
    fx_engine_frame(&e, 9000);
    for (i = 0; i < 4; i++) {
        fx_render(&e.seg[i].cur, after, 0, 600);
        CHECK(memcmp(before[i], after, sizeof after) == 0, "smoothed Fire moves at speed zero");
    }
    p.speed = 255;
    fx_engine_set(&e, 0, &p, 9000, 0); fx_engine_frame(&e, 9000);
    fx_render(&e.seg[0].cur, after, 0, 600);
    CHECK(memcmp(before[0], after, sizeof after) == 0, "smoothed Fire jumps on speed change");
    /* A palette fade owns one heat pair and one fraction; changing earlier
     * geometry must not move/clear the other smoothing slot. */
    memcpy(saved, e.seg[1].cur.state, 256);
    memcpy(saved + 256, e.seg[1].cur.fire_prev, 256);
    p.palette = PAL_LAVA; p.trans_ms = 2000;
    fx_engine_set(&e, 0, &p, 9000, 0); fx_engine_frame(&e, 9017);
    {
        const fx_ctx_t *old = &e.xf[e.seg[0].xf].prev, *cur = &e.seg[0].cur;
        CHECK(old->state == cur->state && old->fire_prev == cur->fire_prev &&
              old->u[0] == cur->u[0] && old->et_frac == cur->et_frac, "Fire palette fade phase/heat differs");
    }
    p.id = FX_STREAM; fx_engine_set(&e, 0, &p, 9017, 0);
    p.id = FX_FIRE; p.seg_len = 8; fx_engine_set(&e, 0, &p, 9017, 9017);
    CHECK(e.seg[0].cur.fire_prev == pool + 1024 && e.seg[1].cur.fire_prev == pool + 1280 &&
          memcmp(saved, e.seg[1].cur.state, 256) == 0 && memcmp(saved + 256, e.seg[1].cur.fire_prev, 256) == 0,
          "Fire reconfiguration moved or corrupted smoothing slots");
    for (i = sizeof pool - 16; i < sizeof pool; i++) CHECK(pool[i] == 0xA5, "Fire pool overrun");
    /* The original four-slot pool still supports four unsmoothed Fires. */
    fx_engine_init(&e, 600, pool, 1024);
    p.seg_len = 0;
    for (i = 0; i < 4; i++) {
        fx_engine_set(&e, i, &p, 0, 0);
        CHECK(e.seg[i].cur.state != 0 && !e.seg[i].cur.fire_prev, "small pool lost unsmoothed Fire");
    }
}

static void test_fire_segment_isolation(void)
{
    fx_engine_t e;
    fx_params_t p;
    uint8_t heat[FX_MAX_SEG * FX_FIRE_CELLS], saved[64];
    uint32_t t, k;
    fx_engine_init(&e, 512, heat, sizeof heat);
    params_of(FX_FIRE, &p);
    p.seg_start = 300;
    p.seg_len = 64;
    fx_engine_set(&e, 1, &p, 0, 0);       /* the hub may configure segments in any order */
    for (t = 0; t <= 2000; t += 30) {
        fx_engine_frame(&e, t);
    }
    memcpy(saved, e.seg[1].cur.state, sizeof saved);
    p.seg_start = 0;
    p.seg_len = 256;
    fx_engine_set(&e, 0, &p, 2010, 2010);
    CHECK(memcmp(saved, e.seg[1].cur.state, sizeof saved) == 0, "starting Fire segment 0 changed running segment 1 heat");
    CHECK(e.seg[0].cur.state + 256 <= e.seg[1].cur.state || e.seg[1].cur.state + 64 <= e.seg[0].cur.state,
          "Fire segments share heat cells after reverse setup order");
    /* Resizing an earlier segment must not move its allocation into a later one. */
    fx_engine_init(&e, 512, heat, sizeof heat);
    p.seg_len = 8;
    fx_engine_set(&e, 0, &p, 0, 0);
    p.seg_start = 300;
    p.seg_len = 64;
    fx_engine_set(&e, 1, &p, 0, 0);
    for (k = 0; k < sizeof saved; k++) {
        e.seg[1].cur.state[k] = saved[k] = (uint8_t)(k + 1u);
    }
    p.seg_start = 0;
    p.seg_len = 256;
    fx_engine_set(&e, 0, &p, 2010, 2010);
    CHECK(memcmp(saved, e.seg[1].cur.state, sizeof saved) == 0, "resizing Fire segment 0 changed segment 1 heat");
    /* The live RNG must be isolated too: extra Fire work and a fading-out Fire
     * on segment 0 must not change segment 1's subsequent heat or pixels. */
    {
        fx_engine_t control;
        uint8_t control_heat[FX_MAX_SEG * FX_FIRE_CELLS];
        rgb_t got[32], want[32];
        fx_engine_init(&e, 512, heat, sizeof heat);
        fx_engine_init(&control, 512, control_heat, sizeof control_heat);
        params_of(FX_FIRE, &p);
        p.seg_start = 300;
        p.seg_len = 64;
        fx_engine_set(&e, 1, &p, 0, 0);
        fx_engine_set(&control, 1, &p, 0, 0);
        for (t = 0; t <= 3000; t += 30) {
            if (t == 900 || t == 1500 || t == 2100) {
                p.seg_start = 0;
                p.seg_len = t == 900 ? 8 : 256;
                p.id = t == 2100 ? FX_SOLID : FX_FIRE;
                p.trans_ms = t == 2100 ? 600 : 0;
                fx_engine_set(&e, 0, &p, t, t);
            }
            fx_engine_frame(&e, t);
            fx_engine_frame(&control, t);
            fx_engine_render(&e, got, 300, 332);
            fx_engine_render(&control, want, 300, 332);
            CHECK(memcmp(got, want, sizeof got) == 0 &&
                  memcmp(e.seg[1].cur.state, control.seg[1].cur.state, 64) == 0,
                  "Fire segment 1 RNG/heat coupled to segment 0 at %u ms", t);
        }
    }
}

static void test_gradient_short_virtual(void)
{
    fx_ctx_t x;
    fx_params_t p;
    rgb_t whole[32], one;
    uint32_t n, vlen, offset, k;
    for (n = 2; n <= 32; n *= 2) {
        for (vlen = 1; vlen <= 7; vlen++) {
            for (offset = 0; offset <= 65535; offset += 65535) {
                params_of(FX_GRADIENT, &p);
                p.intensity = 255;
                p.vlen = (uint16_t)vlen;
                p.vstart = (uint16_t)offset;
                fx_begin(&x, &p, (uint16_t)n, 0);
                fx_frame(&x, 12345);
                fx_render(&x, whole, 0, n);
                for (k = 0; k < n; k++) {
                    const uint32_t step = (8u * (3u << 16)) / vlen;
                    const uint32_t g = (uint32_t)(((uint64_t)(offset + k) * step + ((x.et * 4u) & 65535u) * 3u) % (3u << 16));
                    const uint32_t s = g >> 16;
                    const rgb_t expected = fx_blend(p.c[s], p.c[s == 2u ? 0u : s + 1u], (g >> 8) & 255u);
                    fx_render(&x, &one, k, k + 1);
                    CHECK(rgb_eq(whole[k], expected) && rgb_eq(one, expected), "gradient n=%u vlen=%u offset=%u px=%u escapes colour cycle/chunk", n, vlen, offset, k);
                }
            }
        }
    }
}

static void test_wipe_virtual_extent(void)
{
    const uint32_t lengths[] = { 32767, 32768, 40000, 65535 };
    fx_ctx_t x;
    fx_params_t p;
    rgb_t out[32];
    uint32_t j, k, offset;
    for (j = 0; j < sizeof lengths / sizeof lengths[0]; j++) {
        for (offset = 0; offset < lengths[j]; offset += lengths[j] - 16u) {
            params_of(FX_WIPE, &p);
            p.vlen = (uint16_t)lengths[j];
            p.vstart = (uint16_t)offset;
            p.intensity = 255;
            p.custom1 = 0;
            p.c[0] = fx_rgb(255, 0, 0);
            p.c[1] = fx_rgb(0, 0, 255);
            fx_begin(&x, &p, 32, 0);
            x.et = 3000;                         /* cycle 0's fully completed wipe/hold */
            fx_frame(&x, 0);
            fx_render(&x, out, 0, 32);
            for (k = 0; k < 32; k++) {
                CHECK(rgb_eq(out[k], p.c[offset + k < lengths[j] ? 0 : 1]),
                      "Wipe held colour vlen=%u offset=%u px=%u", lengths[j], offset, k);
            }
        }
    }
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "--golden") == 0) {
        print_golden();
        return 0;
    }
    test_math();
    test_chunks();
    test_time();
    test_fuzz();
    test_golden();
    test_behaviour();
    test_closed_forms();
    test_integrals();
    test_join_time();
    test_pace();
    test_more();
    test_more2();
    test_smooth();
    test_saturation();
    test_etwrap();
    test_engine();
    test_renewal();
    test_pack();
    test_long_transitions();
    test_fire_slices();
    test_fire_smoothing();
    test_fire_segment_isolation();
    test_gradient_short_virtual();
    test_wipe_virtual_extent();
    report_sizes();
    printf("test_fx: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail != 0;
}
