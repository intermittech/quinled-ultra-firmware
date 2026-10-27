/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Frozen b7a03fc arithmetic, host oracle only. */
static uint32_t ref_clz32(uint32_t x)
{
    uint32_t n = 0;
    if ((x & 0xFFFF0000u) == 0u) { n += 16u; x <<= 16; }
    if ((x & 0xFF000000u) == 0u) { n += 8u; x <<= 8; }
    if ((x & 0xF0000000u) == 0u) { n += 4u; x <<= 4; }
    if ((x & 0xC0000000u) == 0u) { n += 2u; x <<= 2; }
    if ((x & 0x80000000u) == 0u) { n += 1u; }
    return n;
}

uint16_t ref_old_lerp16(uint32_t a, uint32_t b, uint32_t s)   /* (ultra.h) */
{
    int32_t d = (int32_t)b - (int32_t)a;
    return (uint16_t)((int32_t)a + (int32_t)(((int64_t)d * s) >> 16));
}

/* ramp_eval's own inline copy: its 11 lerps per render stay inline (through ref_lerp16 the "5 lights fading" render
 * row was +400 cycles, +9 %); the effects and scene.c call ref_lerp16 */
static uint16_t old_lerp16(uint32_t a, uint32_t b, uint32_t s)
{
    int32_t d = (int32_t)b - (int32_t)a;
    return (uint16_t)((int32_t)a + (int32_t)(((int64_t)d * s) >> 16));
}

/* one Horner step of the polynomials in ultra_tables.c: acc = c + acc x t (t = [0, 1) as Q32, the product floored).
 * With coefficients >= 0 every step is monotonic in t. */
#define REF_HORNER(acc, c, t) ((acc) = (c) + (uint32_t)(((uint64_t)(acc) * (t)) >> 32))

/* x^g for x = x_q32 / 2^32 = 2^(g log2 x): log2 of the mantissa and 2^ of the fraction as fixed-point polynomials
 * with non-negative coefficients (tools/gen_ultra_tables.py), so the curve is monotonic at every input and continuous
 * across powers of two. Within 1 count of the former 257-point tables at every 32-bit input, 0 and full scale
 * exact (test/test_gamma_tables.c). Result x^g x 65535 rounded (0..65535). */
uint32_t ref_gamma(uint32_t x, uint32_t g_q16)
{
    const uint32_t *cl = ultra_log2_poly, *ce = ultra_exp2_poly;
    uint32_t n, f, q, fp, e, sh, y;
    int64_t l2, L, k;
    if (x == 0u) {
        return 0;
    }
    if (g_q16 == 65536u) {
        return (uint32_t)(((uint64_t)x * 65535u + 0x80000000u) >> 32);
    }
    n = ref_clz32(x);
    f = (x << n) << 1;                                   /* mantissa 1.f, f as Q32 */
    f = ~f;                                              /* u = 1 - .f: log2(1.f) = 1 - q(u) */
    q = cl[6];
    REF_HORNER(q, cl[5], f);
    REF_HORNER(q, cl[4], f);
    REF_HORNER(q, cl[3], f);
    REF_HORNER(q, cl[2], f);
    REF_HORNER(q, cl[1], f);
    REF_HORNER(q, cl[0], f);
    REF_HORNER(q, 0u, f);                                    /* q(u), Q29, <= 1 (the coefficients sum to 1) */
    l2 = ((int64_t)((1u << 29) - q) << 2) - ((int64_t)(n + 1u) << 31);   /* log2(x), Q31, < 0 */
    L = (l2 * (int64_t)g_q16) / 65536;                   /* g * log2(x), Q31, <= 0 */
    k = -((-L + 0x7FFFFFFF) >> 31);                      /* floor(L / 2^31) */
    fp = (uint32_t)(L - k * 0x80000000LL) << 1;          /* [0, 1) as Q32 */
    e = ce[5];
    REF_HORNER(e, ce[4], fp);
    REF_HORNER(e, ce[3], fp);
    REF_HORNER(e, ce[2], fp);
    REF_HORNER(e, ce[1], fp);
    REF_HORNER(e, 1u << 30, fp);                           /* 2^fp, Q30 (ce[0] = 1.0) */
    if (e > 0x80000000u) {
        e = 0x80000000u;
    }
    /* x^g = 2^fp * 2^k = e * 2^(k - 30), k <= 0; as Q24: e >> (6 - k), then x 65535 */
    sh = (uint32_t)(6 - k);
    if (sh >= 32u) {
        return 0;
    }
    y = (uint32_t)(((uint64_t)(e >> sh) * 65535u + 0x800000u) >> 24);
    return y > 65535u ? 65535u : y;
}

/* product of two 16-bit fractions (0..65535 = 0..1) as a Q32 fraction, 65535 x 65535 -> ~1.0 */
static uint32_t ref_q32_of(uint32_t a, uint32_t b)
{
    uint32_t x = a * b;
    return x + (x >> 15);
}

uint32_t ref_smoothstep(uint32_t p)
{
    int64_t x = (int64_t)p;
    int64_t x3 = (x * x * x) >> 16;                                   /* Q32 */
    int64_t in = (x * (6 * x - 15 * 65536) + 10 * 4294967296LL) >> 8; /* Q24, 6x^2 - 15x + 10 */
    int64_t s = (x3 * in + (1LL << 39)) >> 40;                         /* Q16 */
    return s < 0 ? 0u : (s > 65536 ? 65536u : (uint32_t)s);
}


#undef REF_HORNER
