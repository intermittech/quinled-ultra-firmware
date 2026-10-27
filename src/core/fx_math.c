/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The effect engine's math: sine, noise, palette lookup, hue. Integer only (fx_int.h). */
#include "fx_int.h"

rgb_t fx_blend(rgb_t a, rgb_t b, uint32_t f) { return fx_blend_ai(a, b, f); }
#include "math32.h"

int32_t fx_sin16(uint32_t th)
{
    uint32_t r = th & 0x3FFFu, idx, frac;
    int32_t v;
    if (th & 0x4000u) {
        r = 0x4000u - r;                                 /* the second and fourth quarter run backwards */
    }
    idx = r >> 8;
    frac = r & 255u;
    v = (int32_t)fx_qsin16[idx] + (((int32_t)fx_qsin16[idx + 1u] - (int32_t)fx_qsin16[idx]) * (int32_t)frac >> 8);
    return (th & 0x8000u) ? -v : v;
}

uint8_t fx_sin8(uint32_t th)
{
    return (uint8_t)((fx_sin16(th << 8) >> 8) + 128);
}

/* ------------------------------------------------------------------------------- value noise */
static uint32_t lat(uint32_t xi, uint32_t yi)
{
    uint32_t h = xi * 0x9E3779B1u ^ yi * 0x85EBCA6Bu;
    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    return (h >> 8) & 255u;
}

void fx_nz1_begin(fx_nz1_t *z, uint32_t y8)
{
    z->y = (y8 >> 8) & 0x3FFFFu;                          /* (the lattice row, mod 2^18: the callers' times wrap at 2^32 ms, a multiple, so no row jumps) */
    z->v = (uint8_t)fx_smooth8(y8 & 255u);
    z->ok = 0;
}

static uint32_t nz_col(const fx_nz1_t *z, uint32_t xi)   /* the lattice column at xi, blended in y */
{
    const int32_t p = (int32_t)lat(xi, z->y), q = (int32_t)lat(xi, (z->y + 1u) & 0x3FFFFu);
    return (uint32_t)(p + (((q - p) * (int32_t)z->v) >> 8));
}

uint8_t fx_nz1(fx_nz1_t *z, uint32_t x8)
{
    const uint32_t xi = (x8 >> 8) & 0x3FFFFu, f = fx_smooth8(x8 & 255u);      /* (columns mod 2^18, like the rows) */
    int32_t a, b;
    if (!z->ok || z->xi != xi) {
        if (z->ok && ((z->xi + 1u) & 0x3FFFFu) == xi) {
            z->a = z->b;                                 /* (walking up: the right column becomes the left one) */
        } else {
            z->a = nz_col(z, xi);
        }
        z->b = nz_col(z, (xi + 1u) & 0x3FFFFu);
        z->xi = xi;
        z->ok = 1;
    }
    a = (int32_t)(z->a & 255u);
    b = (int32_t)(z->b & 255u);
    return (uint8_t)(a + (((b - a) * (int32_t)f) >> 8));
}

/* ------------------------------------------------------------------------------- palettes, colour */

rgb_t fx_pal_at(const rgb_t *pal, uint32_t idx16)
{
    const uint32_t e = (idx16 >> 12) & 15u, f = (idx16 >> 4) & 255u;
    const rgb_t a = pal[e], b = pal[(e + 1u) & 15u];
    return fx_blend(a, b, f);
}

rgb_t fx_pal_lin(const rgb_t *pal, uint32_t idx8)
{
    const uint32_t pos = ((idx8 & 255u) * 3855u) >> 8, e = pos >> 8, f = pos & 255u;   /* 3855 = 15 * 257: 255 -> 15.0 */
    return fx_blend(pal[e], pal[e + 1u > 15u ? 15u : e + 1u], f);                     /* (one call: fx_pal_at's hop cost 90 cycles a pixel) */
}

rgb_t fx_hue(uint32_t h)
{
    const uint32_t s = (h & 255u) * 6u, f = s & 255u;
    rgb_t o;
    switch (s >> 8) {
    case 0:  o.r = 255; o.g = (uint8_t)f; o.b = 0; break;
    case 1:  o.r = (uint8_t)(255u - f); o.g = 255; o.b = 0; break;
    case 2:  o.r = 0; o.g = 255; o.b = (uint8_t)f; break;
    case 3:  o.r = 0; o.g = (uint8_t)(255u - f); o.b = 255; break;
    case 4:  o.r = (uint8_t)f; o.g = 0; o.b = 255; break;
    default: o.r = 255; o.g = 0; o.b = (uint8_t)(255u - f); break;
    }
    return o;
}

rgb_t fx_sat(rgb_t c, uint32_t s)
{
    c.r = (uint8_t)(255u - fx_scale8(255u - c.r, s));
    c.g = (uint8_t)(255u - fx_scale8(255u - c.g, s));
    c.b = (uint8_t)(255u - fx_scale8(255u - c.b, s));
    return c;
}

/* ------------------------------------------------------------------------------- inverse sine, integrals of beats */
int32_t fx_asin16(int32_t s)
{
    const uint32_t m = (uint32_t)(s < 0 ? -s : s);
    uint32_t lo = 0, hi = 64, a;
    while (hi - lo > 1u) {                               /* the quarter table is increasing: find its segment */
        const uint32_t mid = (lo + hi) >> 1;
        if (fx_qsin16[mid] <= m) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    a = lo * 256u + ((m - fx_qsin16[lo]) * 256u) / (fx_qsin16[lo + 1u] - fx_qsin16[lo]);
    return s < 0 ? -(int32_t)a : (int32_t)a;
}

/* integral of sin16(phase(t)) over 0..t for a beat of bpm88: (65536^2 / (2 pi 280 bpm88)) * (32767 - cos16(phase)) */
#define SIN_INT_K 2441217u
static int64_t sin_int(uint32_t t, uint32_t bpm88)
{
    return (int64_t)(SIN_INT_K / bpm88) * (32767 - fx_cos16(fx_beat88(t, bpm88)));
}

uint32_t fx_beat_cum(uint32_t t, uint32_t bpm88, uint32_t lo, uint32_t hi)
{
    const uint32_t d = hi - lo;                           /* beatsin88 = (lo + hi) / 2 + d * sin16 / 65536 */
    const uint32_t k=SIN_INT_K/bpm88,s=(uint32_t)(32767-fx_cos16(fx_beat88(t,bpm88)));
    const uint32_t l=k*s,h=m32_hi(k,s);
    return ((lo+hi)*t>>1)+((l*d>>16)|((m32_hi(l,d)+h*d)<<16));
}

uint32_t fx_beat_cum2(uint32_t t, uint32_t bpm_a, uint32_t lo_a, uint32_t hi_a, uint32_t bpm_b, uint32_t lo_b, uint32_t hi_b)
{
    const int64_t da = (int64_t)(hi_a - lo_a), db = (int64_t)(hi_b - lo_b);
    const int64_t ma = (int64_t)(lo_a + hi_a), mb = (int64_t)(lo_b + hi_b);       /* twice the means */
    const uint32_t bd0 = bpm_a > bpm_b ? bpm_a - bpm_b : bpm_b - bpm_a, bd = bd0 != 0u ? bd0 : 1u, bs = bpm_a + bpm_b;
    const int64_t i_a = sin_int(t, bpm_a), i_b = sin_int(t, bpm_b);
    /* the integral of sin16(pa) * sin16(pb) = 32767 / 2 * (the integral of cos16(pa - pb) - the integral of cos16(pa + pb)) */
    const int64_t c_d = (int64_t)(SIN_INT_K / bd) * fx_sin16(fx_beat88(t, bd));
    const int64_t c_s = (int64_t)(SIN_INT_K / bs) * fx_sin16(fx_beat88(t, bs));
    const int64_t i_ab = (32767 * (c_d - c_s)) >> 1;
    /* (ma / 2 + da s_a)(mb / 2 + db s_b) with s = sin16 / 65536:
     *   ma mb / 4 * t  +  ma db / 2 * I_b / 65536  +  mb da / 2 * I_a / 65536  +  da db * I_ab / 2^32 */
    const uint32_t r0 = (uint32_t)(((uint64_t)(ma * mb) * t) >> 2);
    return r0 + (uint32_t)((ma * db * i_b + mb * da * i_a) >> 17) + (uint32_t)((da * db * i_ab) >> 32);
}
