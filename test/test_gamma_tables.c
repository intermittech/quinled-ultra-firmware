/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The gamma polynomials (0.5.5) against the former 257-point tables (included by test_main.c), every input:
 *  - ultra_gamma at every 16-bit level and 256 gammas 0.45-3.0, plus 2 M random two-level products at 26 gammas:
 *    the largest difference in 16-bit steps, monotonic in the level, 0 and full scale exact;
 *  - backward steps over the 32-bit input (ultra_gamma(x) < ultra_gamma(x - 1)) at 27 gammas: around every output
 *    transition, every power of two and in random runs, with the 33-point tables tried first in 0.5.5 as the
 *    negative control (the sweep must find their steps); TEST_GAMMA_FULL=1 adds all 2^32 inputs at 2.2 and 2.8;
 *  - the WS2805 gamma 2.2 / 2.8 (gamma_apply), all 512 inputs: against the former output (with its correction list)
 *    and against round(65535 x (in / 255) ^ gamma), monotonic, 0 -> 0, 255 -> 65535, above 0 never 0. */
#include "old_gamma_tables.inc"

/* the former ultra_gamma (257 points, linear interpolation), verbatim but for the table names */
static uint32_t old_clz32(uint32_t x)
{
    uint32_t n = 0;
    if ((x & 0xFFFF0000u) == 0u) { n += 16u; x <<= 16; }
    if ((x & 0xFF000000u) == 0u) { n += 8u; x <<= 8; }
    if ((x & 0xF0000000u) == 0u) { n += 4u; x <<= 4; }
    if ((x & 0xC0000000u) == 0u) { n += 2u; x <<= 2; }
    if ((x & 0x80000000u) == 0u) { n += 1u; }
    return n;
}

static uint32_t old_gamma(uint32_t x, uint32_t g_q16)
{
    uint32_t n, f, idx, r, lm, fp, e, sh, y;
    int64_t l2, L, k;
    if (x == 0u) {
        return 0;
    }
    if (g_q16 == 65536u) {
        return (uint32_t)(((uint64_t)x * 65535u + 0x80000000u) >> 32);
    }
    n = old_clz32(x);
    f = (x << n) << 1;
    idx = f >> 24;
    r = f & 0xFFFFFFu;
    lm = old_log2_tab[idx] + (uint32_t)(((uint64_t)(old_log2_tab[idx + 1] - old_log2_tab[idx]) * r) >> 24);
    l2 = (int64_t)lm - ((int64_t)(n + 1u) << 31);
    L = (l2 * (int64_t)g_q16) / 65536;
    k = -((-L + 0x7FFFFFFF) >> 31);
    fp = (uint32_t)(L - k * 0x80000000LL);
    idx = fp >> 23;
    r = fp & 0x7FFFFFu;
    e = old_exp2_tab[idx] + (uint32_t)(((uint64_t)(old_exp2_tab[idx + 1] - old_exp2_tab[idx]) * r) >> 23);
    sh = (uint32_t)(6 - k);
    if (sh >= 32u) {
        return 0;
    }
    y = (uint32_t)(((uint64_t)(e >> sh) * 65535u + 0x800000u) >> 24);
    return y > 65535u ? 65535u : y;
}

/* the former gamma_apply: the same curve, plus its six one-count corrections */
static uint16_t old_gamma_apply(uint8_t mode, uint8_t v)
{
    static const struct { uint8_t mode, in; } fix[] = {
        { GAMMA_22, 187 }, { GAMMA_28, 114 }, { GAMMA_28, 183 }, { GAMMA_28, 184 }, { GAMMA_28, 197 }, { GAMMA_28, 220 },
    };
    uint32_t x, y, i;
    x = (uint32_t)v * 257u * 65535u;
    x += x >> 15;
    y = old_gamma(x, mode == GAMMA_22 ? 144179u : 183501u);
    if (v != 0u && y == 0u) {
        y = 1u;
    }
    for (i = 0; i < sizeof fix / sizeof fix[0]; i++) {
        if (fix[i].mode == mode && fix[i].in == v) {
            y++;
        }
    }
    return (uint16_t)y;
}

static uint32_t q32_of_t(uint32_t a, uint32_t b)
{
    uint32_t x = a * b;
    return x + (x >> 15);
}

/* the 33-point version (quadratic interpolation) tried first in 0.5.5, verbatim but for the table names: it steps
 * back by 1 count at 29-71 of the 2^32 inputs per gamma (all of them found by gamma_backward) */
static uint32_t q33_interp(const uint32_t *tab, uint32_t pos, uint32_t s)
{
    uint32_t i = pos >> s, t, d0, m, u, c;
    int32_t d1;
    if (i > 30u) {
        i = 30u;
    }
    t = pos - (i << s);
    d0 = tab[i + 1u] - tab[i];
    d1 = (int32_t)(tab[i + 2u] - tab[i + 1u]) - (int32_t)d0;
    m = (uint32_t)(((uint64_t)(uint32_t)(d1 < 0 ? -d1 : d1) * t) >> s);
    u = t >= (1u << s) ? t - (1u << s) : (1u << s) - t;
    c = (uint32_t)(((uint64_t)m * u) >> (s + 1u));
    return tab[i] + (uint32_t)(((uint64_t)d0 * t) >> s) + (((d1 < 0) != (t < (1u << s))) ? (uint32_t)0 - c : c);
}

static uint32_t q33_gamma(uint32_t x, uint32_t g_q16)
{
    uint32_t n, f, lm, fp, e, sh, y;
    int64_t l2, L, k;
    if (x == 0u) {
        return 0;
    }
    if (g_q16 == 65536u) {
        return (uint32_t)(((uint64_t)x * 65535u + 0x80000000u) >> 32);
    }
    n = old_clz32(x);
    f = (x << n) << 1;
    lm = q33_interp(q33_log2_tab, f, 27u);
    l2 = (int64_t)lm - ((int64_t)(n + 1u) << 31);
    L = (l2 * (int64_t)g_q16) / 65536;
    k = -((-L + 0x7FFFFFFF) >> 31);
    fp = (uint32_t)(L - k * 0x80000000LL);
    e = q33_interp(q33_exp2_tab, fp, 26u);
    sh = (uint32_t)(6 - k);
    if (sh >= 32u) {
        return 0;
    }
    y = (uint32_t)(((uint64_t)(e >> sh) * 65535u + 0x800000u) >> 24);
    return y > 65535u ? 65535u : y;
}

typedef uint32_t (*gamma_fn_t)(uint32_t x, uint32_t g_q16);

/* backward steps fn(x) < fn(x - 1) for x = a .. b (a >= 1); the first one into *first (if still 0) */
static uint32_t gamma_back_range(gamma_fn_t fn, uint32_t g, uint64_t a, uint64_t b, uint32_t *first)
{
    uint32_t n = 0, prev = fn((uint32_t)(a - 1u), g);
    for (; a <= b; a++) {
        uint32_t y = fn((uint32_t)a, g);
        if (y < prev) {
            n++;
            if (*first == 0u) {
                *first = (uint32_t)a;
            }
        }
        prev = y;
    }
    return n;
}

/* Backward steps over the 32-bit input. A curve that rises everywhere but for its rounding can only step back where
 * its output changes: each of the 65,535 transitions y -> y + 1 is found by bisection and the inputs within +-16 of
 * it are checked one by one (the 33-point version's steps lie within +-4 of one). Plus +-4096 around every power of
 * two (the mantissa wraps there) and 64 random runs of 4096 inputs. ~4.8 M calls per gamma. */
#define GAMMA_SWEEP_W 16u
static uint32_t gamma_backward(gamma_fn_t fn, uint32_t g, uint32_t *first)
{
    uint64_t done = 0, a, b;
    uint32_t y, m, n = 0;
    *first = 0;
    for (y = 0; y < 65535u; y++) {
        uint32_t lo = 0, hi = 0xFFFFFFFFu;               /* fn(lo) <= y < fn(hi) */
        while (hi - lo > 1u) {
            uint32_t mid = lo + (hi - lo) / 2u;
            if (fn(mid, g) > y) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        a = hi > GAMMA_SWEEP_W ? (uint64_t)hi - GAMMA_SWEEP_W : 1u;
        a = a > done ? a : done + 1u;                    /* windows of neighbouring transitions overlap at the ends */
        b = (uint64_t)hi + GAMMA_SWEEP_W < 0xFFFFFFFFu ? (uint64_t)hi + GAMMA_SWEEP_W : 0xFFFFFFFFu;
        if (a <= b) {
            n += gamma_back_range(fn, g, a, b, first);
            done = b;
        }
    }
    for (m = 1; m < 32u; m++) {
        uint64_t c = (uint64_t)1 << m;
        n += gamma_back_range(fn, g, c > 4096u ? c - 4096u : 1u, c + 4096u < 0xFFFFFFFFu ? c + 4096u : 0xFFFFFFFFu,
                              first);
    }
    for (m = 0; m < 64u; m++) {
        a = 1u + rnd() % (0xFFFFFFFFu - 4096u);
        n += gamma_back_range(fn, g, a, a + 4095u, first);
    }
    return n;
}

static void test_gamma_tables(void)
{
    uint32_t gi, v, worst = 0, worst_g = 0, worst_v = 0, mono = 0, ends = 0, i, m;
    printf("gamma polynomials against the 257-point tables (linear), every 16-bit level\n");
    for (gi = 0; gi < 256u; gi++) {
        uint32_t g = (uint32_t)((0.45 + (3.0 - 0.45) * gi / 255.0) * 65536.0 + 0.5), prev = 0;
        for (v = 0; v <= 65535u; v++) {
            uint32_t x = q32_of_t(v, 65535u), a = old_gamma(x, g), b = ultra_gamma(x, g);
            uint32_t d = a > b ? a - b : b - a;
            if (d > worst) {
                worst = d;
                worst_g = g;
                worst_v = v;
            }
            mono += b < prev;
            prev = b;
            if ((v == 0u && b != 0u) || (v == 65535u && b != 65535u)) {
                ends++;
            }
        }
    }
    printf("  every level x 256 gammas: worst %u step(s) (level %u, gamma %.3f); monotonic breaks %u; ends %u\n",
           worst, worst_v, worst_g / 65536.0, mono, ends);
    CHECK(worst <= 1u && mono == 0u && ends == 0u, "gamma tables: %u steps, %u breaks, %u ends", worst, mono, ends);

    worst = 0;
    for (i = 0; i < 2000000u; i++) {
        uint32_t a = 1u + (uint32_t)(rnd() % 65535u), b = 1u + (uint32_t)(rnd() % 65535u);
        uint32_t g = (uint32_t)((0.45 + 0.1 * (i % 26u)) * 65536.0 + 0.5), x = q32_of_t(a, b);
        uint32_t ra = old_gamma(x, g), rb = ultra_gamma(x, g), d = ra > rb ? ra - rb : rb - ra;
        if (d > worst) {
            worst = d;
        }
    }
    printf("  2 M random products (color x brightness), 26 gammas: worst %u step(s)\n", worst);
    CHECK(worst <= 1u, "gamma tables, products: %u steps", worst);

    {
        static const double gq[3] = { 0.45, 2.2, 2.8 };
        uint32_t back = 0, first = 0, bad_g = 0, f;
        for (gi = 0; gi < 27u; gi++) {                   /* 0.45, 0.5 .. 3.0 */
            double gd = gi == 0u ? 0.45 : 0.4 + 0.1 * gi;
            uint32_t n = gamma_backward(ultra_gamma, (uint32_t)(gd * 65536.0 + 0.5), &f);
            if (n != 0u && back == 0u) {
                first = f;
                bad_g = (uint32_t)(gd * 65536.0 + 0.5);
            }
            back += n;
        }
        printf("  backward steps over the 32-bit input, 27 gammas 0.45-3.0 (every transition +-%u, powers of two, "
               "random runs): %u", GAMMA_SWEEP_W, back);
        if (back != 0u) {
            printf(" (first at 0x%08X, gamma %.3f)", first, bad_g / 65536.0);
        }
        printf("\n");
        CHECK(back == 0u, "gamma: %u backward steps", back);
        printf("  the same sweep on the 33-point tables (negative control):");
        for (gi = 0; gi < 3u; gi++) {
            uint32_t n = gamma_backward(q33_gamma, (uint32_t)(gq[gi] * 65536.0 + 0.5), &f);
            printf(" gamma %.2f %u step(s)%s", gq[gi], n, gi < 2u ? "," : "\n");
            CHECK(n > 0u, "gamma sweep: no backward step found in the 33-point tables at %.2f", gq[gi]);
        }
    }
    if (getenv("TEST_GAMMA_FULL") != NULL) {             /* all 2^32 inputs: ~70 s per gamma */
        static const uint32_t gf[2] = { 144179u, 183501u };   /* 2.2, 2.8 */
        for (gi = 0; gi < 2u; gi++) {
            uint32_t x = 0, prev = 0, back = 0, w_old = 0;
            do {
                uint32_t a, b, d;
                x++;
                a = ultra_gamma(x, gf[gi]);
                b = old_gamma(x, gf[gi]);
                d = a > b ? a - b : b - a;
                w_old = d > w_old ? d : w_old;
                back += a < prev;
                prev = a;
            } while (x != 0xFFFFFFFFu);
            printf("  all 2^32 inputs, gamma %.1f: %u backward steps, worst %u step(s) from the 257-point tables, "
                   "full scale %u\n", gf[gi] / 65536.0, back, w_old, prev);
            CHECK(back == 0u && w_old <= 1u && prev == 65535u, "gamma %.1f, all inputs: %u back, %u steps",
                  gf[gi] / 65536.0, back, w_old);
        }
    }

    for (m = GAMMA_22; m <= GAMMA_28; m++) {
        uint32_t w_old = 0, w_ex = 0, bad = 0;
        uint16_t prev = 0;
        double g = m == GAMMA_22 ? 144179.0 / 65536.0 : 183501.0 / 65536.0;
        for (v = 0; v < 256u; v++) {
            uint16_t y = gamma_apply((uint8_t)m, (uint8_t)v), o = old_gamma_apply((uint8_t)m, (uint8_t)v);
            uint32_t ex = (uint32_t)(65535.0 * pow(v / 255.0, g) + 0.5), d1 = y > o ? y - o : o - y;
            uint32_t d2 = y > ex ? y - ex : ex - y;
            w_old = d1 > w_old ? d1 : w_old;
            w_ex = d2 > w_ex ? d2 : w_ex;
            bad += y < prev || (v != 0u && y == 0u);
            prev = y;
        }
        bad += gamma_apply((uint8_t)m, 0) != 0u || gamma_apply((uint8_t)m, 255) != 65535u;
        printf("  WS2805 gamma %.1f: against the former output %u step(s), against the formula %u, faults %u\n",
               g, w_old, w_ex, bad);
        CHECK(w_old <= 1u && w_ex <= 1u && bad == 0u, "WS2805 gamma %.1f: %u / %u steps, %u faults", g, w_old, w_ex,
              bad);
    }
}
