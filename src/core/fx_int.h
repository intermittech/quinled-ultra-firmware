/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The effect engine's internal helpers (fx.c, fx_effects.c, fx_engine.c, fx_pack.c): 8-bit scaling, blends, the sine and
 * beat functions, the hash and the value noise. Integer only. Not part of the engine's API. */
#ifndef FX_INT_H
#define FX_INT_H

#include "fx.h"

#if defined(__GNUC__)
#define FX_NOINL __attribute__((noinline))
#define FX_INL   static inline __attribute__((always_inline))   /* (the hot per-pixel helpers: -Os would call them) */
#else
#define FX_NOINL
#define FX_INL   static inline
#endif

extern const uint16_t fx_qsin16[66];
extern const uint16_t fx_asin_recip[64];

static inline uint8_t fx_scale8(uint32_t i, uint32_t s) { return (uint8_t)((i * (s + 1u)) >> 8); }   /* s 255: i */
static inline uint8_t fx_qadd8(uint32_t a, uint32_t b) { a += b; return (uint8_t)(a > 255u ? 255u : a); }
static inline uint8_t fx_qsub8(uint32_t a, uint32_t b) { return (uint8_t)(a > b ? a - b : 0u); }
static inline uint8_t fx_u8(uint32_t v) { return (uint8_t)(v > 255u ? 255u : v); }

static inline rgb_t fx_rgb(uint32_t r, uint32_t g, uint32_t b)
{
    rgb_t o;
    o.r = (uint8_t)r;
    o.g = (uint8_t)g;
    o.b = (uint8_t)b;
    return o;
}

static inline void fx_put(rgb_t *o, rgb_t c)         /* (field stores: -Os turns a struct copy into a memcpy call) */
{
    o->r = c.r;
    o->g = c.g;
    o->b = c.b;
}

FX_INL rgb_t fx_scale_rgb(rgb_t c, uint32_t s)
{
    c.r = fx_scale8(c.r, s);
    c.g = fx_scale8(c.g, s);
    c.b = fx_scale8(c.b, s);
    return c;
}

/* a -> b by f (0..255): f 0 is a, f 255 is b exactly */
FX_INL rgb_t fx_blend(rgb_t a, rgb_t b, uint32_t f)
{
    const int32_t g = (int32_t)(f + (f >> 7));       /* 0..256 */
    rgb_t o;
    o.r = (uint8_t)(a.r + (((int32_t)b.r - a.r) * g >> 8));
    o.g = (uint8_t)(a.g + (((int32_t)b.g - a.g) * g >> 8));
    o.b = (uint8_t)(a.b + (((int32_t)b.b - a.b) * g >> 8));
    return o;
}

FX_INL uint8_t fx_lerp8(uint32_t a, uint32_t b, uint32_t f)   /* a -> b by f 0..255 */
{
    return (uint8_t)((int32_t)a + (((int32_t)b - (int32_t)a) * (int32_t)(f + (f >> 7)) >> 8));
}

FX_INL rgb_t fx_add_sat(rgb_t a, rgb_t b)
{
    a.r = fx_qadd8(a.r, b.r);
    a.g = fx_qadd8(a.g, b.g);
    a.b = fx_qadd8(a.b, b.b);
    return a;
}

/* ------------------------------------------------------------------------------- sine and beats (fx_math.c) */
int32_t  fx_sin16(uint32_t th);                      /* th: 65536 = a turn; -32767 .. 32767 */
FX_INL int32_t fx_sin16_i(uint32_t th)               /* the same, inline (a hot per-pixel loop) */
{
    uint32_t r = th & 0x3FFFu;
    int32_t v;
    if (th & 0x4000u) {
        r = 0x4000u - r;
    }
    v = (int32_t)fx_qsin16[r >> 8] + (((int32_t)fx_qsin16[(r >> 8) + 1u] - (int32_t)fx_qsin16[r >> 8]) * (int32_t)(r & 255u) >> 8);
    return (th & 0x8000u) ? -v : v;
}
#define  fx_cos16(th)  fx_sin16((th) + 16384u)
uint8_t  fx_sin8(uint32_t th);                       /* th: 256 = a turn; 0 .. 255 around 128 */
/* a beat: the phase (0..65535) of a clock of bpm88 (beats per minute x 256) at t ms; FastLED's constants */
static inline uint32_t fx_beat88(uint32_t t, uint32_t bpm88) { return ((t * bpm88 * 280u) >> 16) & 0xFFFFu; }
/* lo .. hi following sin of the beat (+ phase), 16 bit */
static inline uint32_t fx_beatsin88(uint32_t t, uint32_t bpm88, uint32_t lo, uint32_t hi, uint32_t phase)
{
    const uint32_t s = (uint32_t)(fx_sin16(fx_beat88(t, bpm88) + phase) + 32768);
    return lo + (((hi - lo) * s) >> 16);
}
#define fx_beatsin(t, bpm, lo, hi) fx_beatsin88((t), (uint32_t)(bpm) << 8, (lo), (hi), 0u)
/* the angle (65536 = a turn, -16384 .. 16384) whose fx_sin16 is s (|s| <= 32767) */
int32_t  fx_asin16(int32_t s);
/* Closed-form integrals of beats from t = 0 to t ms (modulo 2^32, in value x ms), so an effect's state can be a pure function of
 * its time instead of a sum over frames: fx_beat_cum = the integral of fx_beatsin88(t, bpm88, lo, hi); fx_beat_cum2 = of the
 * product of two beats (the first one's value / 256 is the second's rate multiplier in Pacifica) */
uint32_t fx_beat_cum(uint32_t t, uint32_t bpm88, uint32_t lo, uint32_t hi);
uint32_t fx_beat_cum2(uint32_t t, uint32_t bpm_a, uint32_t lo_a, uint32_t hi_a, uint32_t bpm_b, uint32_t lo_b, uint32_t hi_b);

/* ------------------------------------------------------------------------------- hash and noise */
static inline uint32_t fx_hash(uint32_t x)           /* (lowbias32, public domain) */
{
    x ^= x >> 16;
    x *= 0x7FEB352Du;
    x ^= x >> 15;
    x *= 0x846CA68Bu;
    x ^= x >> 16;
    return x;
}
static inline uint32_t fx_hash2(uint32_t a, uint32_t b) { return fx_hash(a * 0x9E3779B1u ^ fx_hash(b + 0x85EBCA6Bu)); }
/* a cheap two-input hash (one multiply to mix, one to finish) for the per-pixel random effects: the top bits are good,
 * the low ones after the final xor-shift too */
static inline uint32_t fx_hashf(uint32_t a, uint32_t b)
{
    uint32_t x = a * 0x9E3779B1u + b * 0x85EBCA6Bu;
    x ^= x >> 15;
    x *= 0x2C1B3C6Du;
    x ^= x >> 16;
    return x;
}

static inline uint32_t fx_smooth8(uint32_t f)        /* 3f^2 - 2f^3 on 0..255 */
{
    return (f * f * (768u - 2u * f)) >> 16;
}

/* 1-D value noise along x at a fixed y, the lattice values cached (a frame's pixels walk x upward):
 * x, y are 8.8 (the lattice every 256). value 0..255 */
typedef struct { uint32_t y, xi; uint32_t a, b; uint8_t v, ok; } fx_nz1_t;
void     fx_nz1_begin(fx_nz1_t *z, uint32_t y8);
uint8_t  fx_nz1(fx_nz1_t *z, uint32_t x8);

/* ------------------------------------------------------------------------------- palettes and colour */
rgb_t    fx_pal_at(const rgb_t *pal, uint32_t idx16);   /* circular: 0..65535 round the 16 entries */
FX_INL rgb_t fx_pal_at_i(const rgb_t *pal, uint32_t idx16)   /* the same, inline */
{
    const uint32_t e = (idx16 >> 12) & 15u;
    return fx_blend(pal[e], pal[(e + 1u) & 15u], (idx16 >> 4) & 255u);
}
rgb_t    fx_pal_lin(const rgb_t *pal, uint32_t idx8);   /* non-wrapping: 0..255 = entry 0 .. entry 15 */
rgb_t    fx_hue(uint32_t h);                           /* full saturation and value, h 0..255 round the wheel */
rgb_t    fx_sat(rgb_t c, uint32_t s);                  /* toward white: s 255 keeps the colour */

extern const rgb_t fx_pals[PAL_COUNT][16];
extern const rgb_t fx_pal_pacifica[3][16];

#endif
