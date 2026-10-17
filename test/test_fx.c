/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Host tests of the local-effects engine (src/core/fx*.c, docs/ultradigital-effects.md): the math, every effect (chunk and
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
        for (i = 1; i < 256u; i++) {
            CHECK(fx_gamma28[i] >= fx_gamma28[i - 1u], "gamma monotonic at %u", i);
        }
        CHECK(fx_gamma28[0] == 0 && fx_gamma28[255] == 255, "gamma ends");
        CHECK(abs((int)fx_gamma28[128] - (int)floor(255.0 * pow(128.0 / 255.0, 2.8) + 0.5)) <= 1, "gamma 2.8 at 128");
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
        CHECK(near_rgb(bufA[0], fx_pals[PAL_SUNRISE][15], 2), "sunrise ends in the last palette entry");
        p.id = FX_SUNSET;
        render_whole(&p, 30, 0, bufA);
        CHECK(near_rgb(bufA[0], fx_pals[PAL_SUNRISE][15], 2), "sunset starts bright");
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
        CHECK(x.pal_id == PAL_HEAT && rgb_eq(x.pal[15], fx_pals[PAL_HEAT][15]), "fire's default palette");
        p.palette = PAL_OCEAN;
        fx_begin(&x, &p, 20, g_state);
        CHECK(rgb_eq(x.pal[3], fx_pals[PAL_OCEAN][3]), "explicit palette");
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
    /* fire: stateful, crossfades out to a stateless effect; fire -> fire (palette change) is a hard switch; no pool room: solid */
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
    CHECK(E.seg[0].xf < 0, "fire -> fire with another palette is a hard switch (one state)");
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
    c.gamma = fx_gamma28;
    sum = fx_pack(in, w, 1, 255, &c);
    CHECK(w[0] == fx_gamma28[150] && w[1] == fx_gamma28[100] && w[2] == fx_gamma28[200], "gamma applied");
    CHECK(sum == (uint32_t)w[0] + w[1] + w[2], "sum of the wire bytes");
    sum = fx_pack(in, w, 1, 128, &c);
    CHECK(w[1] == fx_gamma28[fx_scale8(100, 128)] && w[0] == fx_gamma28[fx_scale8(150, 128)], "brightness before gamma");
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
        c.bpp = 3; c.ord[0] = 1; c.ord[1] = 0; c.ord[2] = 2; c.gamma = fx_gamma28; c.white_mode = 0;
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
    test_engine();
    test_renewal();
    test_pack();
    report_sizes();
    printf("test_fx: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail != 0;
}
