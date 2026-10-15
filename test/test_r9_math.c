/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Review round 9, code size: core/ultra.c's maths in 32 bits against the 64-bit code it replaced (firmware
 * 0.6.10 before round 9), old and new side by side over the whole input domain or a covering set of it: the default run (~15 s)
 * sweeps every 16-bit input exhaustively against random / edge-heavy choices of the others, `r9_math.exe long` adds
 * the exhaustive 2^32 sweeps (minutes). test\build_host.cmd builds and runs it (r9_math.exe).
 *
 * The former code is copied here verbatim (old_*); this file includes core/ultra.c itself to reach the static
 * functions it tests (built with ultra_tables.c alone). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../src/core/ultra.c"

static int g_fail, g_long;
#define CHECK(c, ...) do { if (!(c)) { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint64_t rs = 0x2545F4914F6CDD1Dull;
static uint64_t rnd64(void)
{
    rs ^= rs << 13;
    rs ^= rs >> 7;
    rs ^= rs << 17;
    return rs;
}
static uint32_t rnd32(void) { return (uint32_t)(rnd64() >> 32); }

/* a 16-bit value, the edges every other draw */
static uint32_t edge16(void)
{
    static const uint16_t e[] = { 0, 1, 2, 3, 255, 256, 257, 32767, 32768, 32769, 65533, 65534, 65535 };
    uint32_t k = rnd32() % 26u;
    return k < 13u ? e[k] : (rnd32() & 0xFFFFu);
}

/* a 32-bit value, the edges every other draw */
static uint32_t edge32(void)
{
    static const uint32_t e[] = { 0, 1, 2, 0x7FFFFFFFu, 0x80000000u, 0xFFFFFFFEu, 0xFFFFFFFFu, 65535u, 65536u,
                                  65535u << 16, 65534u << 16, 1u << 16 };
    uint32_t k = rnd32() % 24u;
    return k < 12u ? e[k] : rnd32();
}

/* ================================================================================ 1. dim to warm (3732a57) */
static uint64_t old_dtw_points(const ultra_light_t *L, uint32_t m, uint64_t own_q)
{
    uint32_t i, hm = 65535u;
    int64_t hq = (int64_t)own_q;
    for (i = 0; i < L->opt_n; i++) {
        uint32_t pm = L->opt_pm[i];
        int64_t pq = (int64_t)((uint64_t)L->opt_pt[i] << 16);
        if (m >= pm) {
            return hm == pm ? (uint64_t)pq : (uint64_t)(pq + (hq - pq) * (int64_t)(m - pm) / (int64_t)(hm - pm));
        }
        hm = pm;
        hq = pq;
    }
    return (uint64_t)hq;
}

static void old_dim_to_warm(const ultra_light_t *L, uint32_t m, uint32_t cw_ch, uint32_t ww_ch, uint32_t gcw, uint32_t gww,
                            uint32_t *y, uint8_t *on)
{
    uint32_t sum = gcw + gww, mx = gcw > gww ? gcw : gww, cwm = L->opt_cw, wwm = L->opt_ww, warm = L->opt_warm;
    uint32_t t = 0, scale, fw, fc, ch, r_ch = 5, g_ch = 5, rgb_on = 0;
    uint64_t cw_q, ww_q, warm_q, top_q, tgt_q;            /* mireds x 10, Q16 */
    if (sum == 0u || wwm <= cwm) {
        return;                                          /* white off, or no colour temperatures */
    }
    cw_q = (uint64_t)cwm << 16;
    ww_q = (uint64_t)wwm << 16;
    warm_q = (uint64_t)warm << 16;
    top_q = cw_q + (uint64_t)(wwm - cwm) * (((uint32_t)gww << 16) / sum);   /* the light's own temperature */
    tgt_q = top_q;
    if (L->opt_flags & ULTRA_OPT_POINTS) {
        uint64_t pq = old_dtw_points(L, m, top_q);
        if (pq > top_q) {
            tgt_q = pq;                                  /* never colder than the light's own temperature */
        }
    } else if (warm_q > top_q) {
        uint32_t d = (uint32_t)(warm_q - top_q), w = 65535u - m, a = d / 65535u, b = d - a * 65535u;
        tgt_q += a * w + (b * w) / 65535u;
    }
    if (tgt_q <= ww_q) {
        fw = (uint32_t)(tgt_q - cw_q) / (wwm - cwm);    /* WW share, Q16 */
    } else {
        fw = 65536u;                                     /* warmer than the WW LED: all WW, red assist */
        t = (uint32_t)(tgt_q - ww_q) / (warm - wwm);
    }
    fc = 65536u - fw;
    scale = (g16(L, m) * mx + 32767u) / 65535u;          /* g(M) x max(gcw, gww) */
    if (L->flags & ULTRA_DEF_CONSTANT_BRIGHTNESS) {
        y[cw_ch] = (scale * fc + 32768u) >> 16;          /* CW + WW = g(M) x max */
        y[ww_ch] = (scale * fw + 32768u) >> 16;
    } else {
        uint32_t fmax = fw > fc ? fw : fc;               /* the brighter channel stays at g(M) x max */
        y[cw_ch] = (scale * fc + fmax / 2u) / fmax;
        y[ww_ch] = (scale * fw + fmax / 2u) / fmax;
    }
    on[cw_ch] = (uint8_t)(scale != 0u && fc != 0u);
    on[ww_ch] = (uint8_t)(scale != 0u && fw != 0u);
    if (t == 0u || L->opt_assist == 0u) {
        return;
    }
    for (ch = 0; ch < 5u; ch++) {
        if (L->role[ch] == ULTRA_ROLE_RED) r_ch = ch;
        if (L->role[ch] == ULTRA_ROLE_GREEN) g_ch = ch;
        if (is_rgb(L->role[ch]) && on[ch]) rgb_on = 1;
    }
    if (r_ch < 5u && !rgb_on) {                          /* red assist: only while the light's RGB part is off */
        uint32_t r = (((y[ww_ch] * t) >> 16) * L->opt_assist + 32767u) / 65535u;   /* (y, t <= 65536: fits) */
        y[r_ch] = r;
        on[r_ch] = (uint8_t)(r != 0u);
        if (g_ch < 5u) {
            y[g_ch] = r / 4u;                            /* a touch of green keeps it on the warm-white line */
            on[g_ch] = (uint8_t)(r / 4u != 0u);
        }
    }
}

/* a LIGHT_OPT as the slot parser stores it (ultra_apply_slot: the fields, opt_points: the valid points sorted by M,
 * descending, the warm end at least the warmest point); 16-bit values with the edges often */
static void opt_random(ultra_light_t *L, int points)
{
    uint8_t p[32];
    uint32_t i;
    memset(p, 0, sizeof p);
    p[0] = (uint8_t)(ULTRA_OPT_DIM_TO_WARM | (points ? ULTRA_OPT_POINTS : 0u));
    for (i = 0; i < 4u; i++) {
        ultra_put16(p + 1 + 2 * i, edge16());            /* cw, ww, warm, assist */
    }
    for (i = 0; i < ULTRA_OPT_POINTS_MAX; i++) {
        ultra_put16(p + 9 + 4 * i, (rnd32() & 1u) ? edge16() : 0u);   /* a point or an unused slot */
        ultra_put16(p + 11 + 4 * i, edge16());
    }
    L->opt_flags = (uint8_t)(p[0] & ULTRA_OPT_FLAGS);
    L->opt_cw = (uint16_t)ultra_get16(p + 1);
    L->opt_ww = (uint16_t)ultra_get16(p + 3);
    L->opt_warm = (uint16_t)ultra_get16(p + 5);
    L->opt_assist = (uint16_t)ultra_get16(p + 7);
    opt_points(L, p);
}

static void test_dtw(void)
{
    static ultra_light_t L;
    uint32_t cfg, m, bad = 0, over = 0, ncfg = g_long ? 100000u : 10000u, used = 0;
    printf("1. dim to warm in 32 bits (dtw_points / dim_to_warm against 3732a57's 64-bit code)\n");
    /* 1a. dtw_points: every M for random point sets (0..4 points, duplicates, M 65535) and any 32-bit own_q */
    for (cfg = 0; cfg < ncfg; cfg++) {
        uint32_t own_q = edge32();
        memset(&L, 0, sizeof L);
        opt_random(&L, 1);
        used += L.opt_n != 0u;
        for (m = 0; m < 65536u; m++) {
            uint64_t o = old_dtw_points(&L, m, own_q);
            uint32_t n = dtw_points(&L, m, own_q);
            over += o > 0xFFFFFFFFull;
            bad += o != (uint64_t)n;
        }
    }
    CHECK(bad == 0u && over == 0u, "dtw_points: %u of %u configs x 65536 M differ, %u old results above 32 bits",
          (unsigned)bad, (unsigned)ncfg, (unsigned)over);
    printf("   dtw_points: %u point sets (%u with points) x every M: identical\n", (unsigned)ncfg, (unsigned)used);
    /* 1b. dim_to_warm: every M for random lights (roles, CW / WW mireds, the warm end, assist, points or the line,
     * constant brightness or not, gamma, the gamma'd CW / WW levels) */
    ncfg = g_long ? 5000u : 400u;
    bad = 0;
    for (cfg = 0; cfg < ncfg; cfg++) {
        uint32_t cw_ch = rnd32() % 5u, ww_ch = (cw_ch + 1u + rnd32() % 4u) % 5u, gcw = edge16(), gww = edge16(), i;
        static const uint32_t g[] = { 65536u, 183500u, 32768u, 262144u, 6554u, 327680u };
        memset(&L, 0, sizeof L);
        opt_random(&L, (int)(rnd32() & 1u));
        for (i = 0; i < 5u; i++) {
            L.role[i] = (uint8_t)(rnd32() % 8u);         /* NONE .. WARM_WHITE */
        }
        L.role[cw_ch] = ULTRA_ROLE_COLD_WHITE;
        L.role[ww_ch] = ULTRA_ROLE_WARM_WHITE;
        L.flags = (uint8_t)((rnd32() & 1u) ? ULTRA_DEF_CONSTANT_BRIGHTNESS : 0u);
        L.g_q16 = (rnd32() & 1u) ? g[rnd32() % 6u] : 6554u + rnd32() % 321127u;
        for (m = 0; m < 65536u; m++) {
            uint32_t y1[5] = { 0, 0, 0, 0, 0 }, y2[5] = { 0, 0, 0, 0, 0 };
            uint8_t on1[5] = { 0, 0, 0, 0, 0 }, on2[5] = { 0, 0, 0, 0, 0 };
            old_dim_to_warm(&L, m, cw_ch, ww_ch, gcw, gww, y1, on1);
            dim_to_warm(&L, m, cw_ch, ww_ch, gcw, gww, y2, on2);
            bad += memcmp(y1, y2, sizeof y1) != 0 || memcmp(on1, on2, sizeof on1) != 0;
        }
    }
    CHECK(bad == 0u, "dim_to_warm: %u of %u lights x 65536 M differ", (unsigned)bad, (unsigned)ncfg);
    printf("   dim_to_warm: %u lights x every M: identical outputs and on[] flags\n", (unsigned)ncfg);
}

/* ================================================================================ 2. the effects (3732a57) */
static int32_t old_cubic(uint32_t r16, uint32_t intensity)   /* rnd_cubic with its rnd16() draw as the argument */
{
    int64_t r = (int64_t)r16 * 2 - 65535;
    int64_t c = (((r * r) >> 16) * r) >> 16;
    return (int32_t)((c * (int64_t)intensity) >> 16);
}

static void old_fx_color_loop(const ultra_light_t *L, uint32_t now, const ultra_vals_t *b, ultra_vals_t *o)
{
    uint32_t per = fx_period(L), sat = ultra_get16(L->fxp + 3), h6, f, pv, q, t, rgb[3], i, n = 0;
    h6 = frac16((now - L->fx_t0) % per, per) * 6u;
    f = h6 & 0xFFFFu;
    pv = 65535u - sat;
    q = 65535u - (uint32_t)(((uint64_t)sat * f) >> 16);
    t = 65535u - (uint32_t)(((uint64_t)sat * (65536u - f)) >> 16);
    switch ((h6 >> 16) % 6u) {
    case 0: rgb[0] = 65535u; rgb[1] = t; rgb[2] = pv; break;
    case 1: rgb[0] = q; rgb[1] = 65535u; rgb[2] = pv; break;
    case 2: rgb[0] = pv; rgb[1] = 65535u; rgb[2] = t; break;
    case 3: rgb[0] = pv; rgb[1] = q; rgb[2] = 65535u; break;
    case 4: rgb[0] = t; rgb[1] = pv; rgb[2] = 65535u; break;
    default: rgb[0] = 65535u; rgb[1] = pv; rgb[2] = q; break;
    }
    vals_cpy(o, b);
    for (i = 0; i < 5u; i++) {
        if (is_rgb(L->role[i])) {
            o->a[i] = (uint16_t)rgb[L->role[i] - ULTRA_ROLE_RED];
            n++;
        }
    }
    for (i = 0; n && i < 5u; i++) {
        if (L->role[i] == ULTRA_ROLE_WHITE || is_cww(L->role[i])) {
            o->a[i] = 0;
        }
    }
}

static int old_fx_sun(const ultra_light_t *L, uint32_t now, const ultra_vals_t *b, ultra_vals_t *o, int rise)
{
    static const uint16_t dawn[3] = { 65535u, 11796u, 0u };   /* (1, 0.18, 0) */
    uint32_t d = ultra_get24(L->fxp), el = now - L->fx_t0, p, w, i;
    if (d == 0u) {
        d = 1u;
    }
    p = frac16(el, d);
    w = rise ? p : 65536u - p;                           /* weight of the light's own state */
    vals_cpy(o, b);
    o->m = (uint16_t)((b->m * (uint32_t)(((uint64_t)w * w) >> 16)) >> 16);   /* eases in / out (quadratic) */
    for (i = 0; i < 5u; i++) {                           /* (lerp16: ultra_lerp16 since review round 9's item 4) */
        uint8_t r = L->role[i];
        if (is_rgb(r)) {
            o->a[i] = ultra_lerp16(dawn[r - ULTRA_ROLE_RED], b->a[i], w);
        } else if (r == ULTRA_ROLE_WARM_WHITE) {
            o->a[i] = ultra_lerp16(65535u, b->a[i], w);
        } else if (r == ULTRA_ROLE_COLD_WHITE || r == ULTRA_ROLE_WHITE) {
            o->a[i] = ultra_lerp16(0, b->a[i], w);
        }
    }
    return el >= d;
}

static void vals_random(ultra_vals_t *v)
{
    uint32_t i;
    v->m = (uint16_t)edge16();
    for (i = 0; i < 5u; i++) {
        v->a[i] = (uint16_t)edge16();
        v->b[i] = (uint16_t)edge16();
    }
}

static void roles_random(ultra_light_t *L)
{
    uint32_t i;
    for (i = 0; i < 5u; i++) {
        L->role[i] = (uint8_t)(rnd32() % 8u);            /* NONE .. WARM_WHITE, any mix */
    }
}

static void test_cubic(void)
{
    static ultra_light_t L, L2;
    uint32_t r16, i, k, bad = 0, nint = g_long ? 65536u : 256u;
    printf("2a. flicker's random cubic in 32 bits (cubic_q16 / rnd_cubic against rnd_cubic)\n");
    for (k = 0; k < nint; k++) {                         /* every draw x the edges + random intensities (long: all) */
        static const uint16_t e[] = { 0, 1, 2, 3, 255, 256, 32767, 32768, 65533, 65534, 65535 };
        uint32_t inten = g_long ? k : (k < 11u ? e[k] : (rnd32() & 0xFFFFu));
        for (r16 = 0; r16 < 65536u; r16++) {
            bad += cubic_q16(r16, inten) != old_cubic(r16, inten);
        }
    }
    CHECK(bad == 0u, "cubic_q16: %u of 65536 x %u differ", (unsigned)bad, (unsigned)nint);
    printf("   cubic_q16: every draw x %u intensities: identical\n", (unsigned)nint);
    memset(&L, 0, sizeof L);
    for (i = 0; i < 2000000u; i++) {                     /* the draw itself: the same xorshift stream */
        uint32_t inten = edge16();
        L2 = L;
        bad += rnd_cubic(&L, inten) != old_cubic(rnd16(&L2), inten);
        bad += L.rng != L2.rng;
    }
    CHECK(bad == 0u, "rnd_cubic: %u of 2 M draws differ", (unsigned)bad);
}

static void test_color_loop(void)
{
    static ultra_light_t L;
    static const uint32_t pers[] = { 1, 2, 3, 6, 7, 6000, 30000, 65535, 65536, 65537, 393216, 0xFFFFFF, 0 };
    ultra_vals_t b, o1, o2;
    uint32_t sat, pi, k, bad = 0, nel = g_long ? 1024u : 16u, calls = 0;
    printf("2b. color loop in 32 bits (fx_color_loop against 3732a57)\n");
    for (pi = 0; pi < sizeof pers / sizeof pers[0]; pi++) {
        uint32_t per = pers[pi], per_eff = per ? per : 30000u;
        for (k = 0; k < nel; k++) {
            uint32_t el = k == 0u ? 0u : (k == 1u ? per_eff - 1u : (k == 2u ? per_eff / 2u : (k == 3u ? per_eff / 3u :
                          (k == 4u ? per_eff / 6u : (k == 5u ? 2u * per_eff + 1u : rnd32())))));
            memset(&L, 0, sizeof L);
            L.fx = ULTRA_FX_COLOR_LOOP;
            L.fxp[0] = (uint8_t)(per >> 16);
            L.fxp[1] = (uint8_t)(per >> 8);
            L.fxp[2] = (uint8_t)per;
            L.fx_t0 = rnd32();
            roles_random(&L);
            vals_random(&b);
            for (sat = 0; sat < 65536u; sat++) {         /* every saturation */
                ultra_put16(L.fxp + 3, sat);
                old_fx_color_loop(&L, L.fx_t0 + el, &b, &o1);
                fx_color_loop(&L, L.fx_t0 + el, &b, &o2);
                bad += memcmp(&o1, &o2, sizeof o1) != 0;
                calls++;
            }
        }
    }
    CHECK(bad == 0u, "fx_color_loop: %u of %u calls differ", (unsigned)bad, (unsigned)calls);
    printf("   fx_color_loop: every saturation x %u (period, elapsed) pairs: identical\n", (unsigned)(calls / 65536u));
    if (g_long) {                                        /* the two products over their whole domain */
        uint32_t f;
        for (sat = 0; sat < 65536u; sat++) {
            for (f = 0; f < 65536u; f++) {
                bad += (65535u - (uint32_t)(((uint64_t)sat * f) >> 16)) != (65535u - ((sat * f) >> 16));
                bad += (65535u - (uint32_t)(((uint64_t)sat * (65536u - f)) >> 16)) != (65535u - ((sat * (65536u - f)) >> 16));
            }
        }
        CHECK(bad == 0u, "color loop's q / t: %u of 2^32 (sat, f) differ", (unsigned)bad);
        printf("   q / t: every (saturation, hue fraction): identical\n");
    }
}

static void test_sun(void)
{
    static ultra_light_t L;
    static const uint32_t ds[] = { 0, 1, 2, 3, 1000, 65535, 65536, 65537, 131072, 0xFFFFFF };
    ultra_vals_t b, o1, o2;
    uint32_t di, el, k, w, m, bad = 0, calls = 0, nm = g_long ? 65536u : 256u;
    printf("2c. sunrise / sunset in 32 bits (fx_sun against 3732a57)\n");
    for (di = 0; di < sizeof ds / sizeof ds[0]; di++) {
        uint32_t d = ds[di], d_eff = d ? d : 1u, nel = d_eff <= 65537u ? d_eff + 2u : (g_long ? 65536u : 4096u);
        for (el = 0; el < nel; el++) {                   /* every elapsed time of a short effect: every weight */
            uint32_t e = d_eff <= 65537u ? el : rnd32() % (2u * d_eff);
            for (k = 0; k < 4u; k++) {
                memset(&L, 0, sizeof L);
                L.fxp[0] = (uint8_t)(d >> 16);
                L.fxp[1] = (uint8_t)(d >> 8);
                L.fxp[2] = (uint8_t)d;
                L.fx_t0 = rnd32();
                roles_random(&L);
                vals_random(&b);
                bad += old_fx_sun(&L, L.fx_t0 + e, &b, &o1, (int)(k & 1u)) != fx_sun(&L, L.fx_t0 + e, &b, &o2, (int)(k & 1u));
                bad += memcmp(&o1, &o2, sizeof o1) != 0;
                calls++;
            }
        }
    }
    CHECK(bad == 0u, "fx_sun: %u of %u calls differ", (unsigned)bad, (unsigned)calls);
    printf("   fx_sun: %u (duration, elapsed, rise, light) calls: identical\n", (unsigned)calls);
    for (w = 0; w <= 65536u; w++) {                      /* the eased brightness: every weight x M (long: all M) */
        uint32_t w2 = w < 65536u ? (w * w) >> 16 : 65536u;
        for (k = 0; k < nm; k++) {
            m = g_long ? k : (k < 2u ? 65535u - k : (rnd32() & 0xFFFFu));
            bad += ((m * (uint32_t)(((uint64_t)w * w) >> 16)) >> 16) != ((m * w2) >> 16);
        }
    }
    CHECK(bad == 0u, "fx_sun's eased M: %u of 65537 x %u differ", (unsigned)bad, (unsigned)nm);
    printf("   eased M: every weight x %u M: identical\n", (unsigned)nm);
}

/* ================================================================================ 3. scene.c's fades on ultra_lerp16 */
static uint32_t old_scene_in_mix(uint32_t from, uint32_t sv, uint32_t s)   /* scene_in's expression (3732a57) */
{
    return (uint32_t)((int32_t)from + (int32_t)((((int64_t)sv - (int64_t)from) * (int64_t)s) >> 16));
}

static uint16_t old_scene_mix_fade(uint32_t sv, uint32_t v, uint32_t s)   /* scene_mix's expression (3732a57) */
{
    return (uint16_t)((int32_t)sv + (int32_t)((((int64_t)v - (int64_t)sv) * (int64_t)s) >> 16));
}

static void test_lerp(void)
{
    uint32_t s, k, bad = 0, npair = g_long ? 0u : 64u;
    printf("3. scene.c's fades on ultra_lerp16 (against scene_in / scene_mix's own expressions)\n");
    if (g_long) {                                        /* every s x every difference b - a (the sum carries a) */
        int32_t d;
        for (d = -65535; d <= 65535; d++) {
            uint32_t a = d < 0 ? (uint32_t)-d : 0u, b = d < 0 ? 0u : (uint32_t)d;
            for (s = 0; s <= 65536u; s++) {
                bad += old_scene_in_mix(a, b, s) != ultra_lerp16(a, b, s);
                bad += old_scene_mix_fade(a, b, s) != ultra_lerp16(a, b, s);
            }
        }
    }
    for (k = 0; k < npair; k++) {                        /* every s x 64 (a, b) pairs, the edges often */
        uint32_t a = edge16(), b = edge16();
        for (s = 0; s <= 65536u; s++) {
            bad += old_scene_in_mix(a, b, s) != ultra_lerp16(a, b, s);
            bad += old_scene_mix_fade(a, b, s) != ultra_lerp16(a, b, s);
        }
    }
    for (k = 0; k < (1u << 24); k++) {                   /* 16 M random (a, b, s) */
        uint32_t a = edge16(), b = edge16();
        s = (rnd32() & 1u) ? 65536u : edge16();
        bad += old_scene_in_mix(a, b, s) != ultra_lerp16(a, b, s);
        bad += old_scene_mix_fade(a, b, s) != ultra_lerp16(a, b, s);
    }
    CHECK(bad == 0u, "ultra_lerp16 against scene.c's expressions: %u differ", (unsigned)bad);
    printf("   ultra_lerp16: %s16 M random (a, b, s): identical\n", g_long ? "every s x every b - a, " : "every s x 64 (a, b), ");
}

int main(int argc, char **argv)
{
    g_long = argc > 1 && strcmp(argv[1], "long") == 0;
    printf("review round 9: ultra.c's 32-bit maths against the former 64-bit code%s\n", g_long ? " (long)" : "");
    test_dtw();
    test_cubic();
    test_color_loop();
    test_sun();
    test_lerp();
    printf(g_fail ? "FAILED: %d\n" : "all passed (%d failures)\n", g_fail);
    return g_fail ? 1 : 0;
}
