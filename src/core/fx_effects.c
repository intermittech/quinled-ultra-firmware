/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The effects (fx.h): fx_begin / fx_frame / fx_render and the 26 effects of the catalogue. Integer only.
 *
 * Every effect is a closed-form function of (parameters, effect time, pixel index): fx_render(from, to) in any order
 * and chunking gives the same pixels. What changes once a frame is worked out in fx_frame's per-effect prep (the
 * x->u[] numbers); the per-pixel loops only walk. At most a few divisions per frame, none per pixel.
 *
 * Time: x->t is the effect's milliseconds since its anchor; x->et is the speed-integrated time the animations use
 * (linear in the slider: 128 is about 1.0x, 255 about 1.9x, 1 about 0.04x, 0 stands still). A speed change keeps the picture
 * continuous because et is an integral.
 *
 * Origins (NOTICE.md): the effects with FastLED names are re-implementations of Mark Kriegsman's FastLED examples
 * (MIT): Pride2015, ColorWavesWithPalettes, TwinkleFOX, Pacifica (with Mary Corey March), Fire2012, and DemoReel100's
 * sinelon. Nothing here is WLED code; the other effects are written from their described behaviour. */
#include "fx_int.h"

extern const uint8_t fx_pal_default_tab[FX_COUNT];

#define VP(x, i)   ((x)->vbase + (i))                    /* the pixel's place in the virtual strip */
#define U_FILL(x, a, b, o, c) do { const rgb_t c_ = (c); for (; (a) < (b); (a)++) { fx_put((o) + (a), c_); } } while (0)

typedef void (*render_fn)(fx_ctx_t *, rgb_t *, uint32_t, uint32_t);
typedef void (*prep_fn)(fx_ctx_t *, uint32_t);           /* (x, the effect ms since the last frame) */

static inline uint32_t pack_rgb(rgb_t c) { return (uint32_t)c.r << 16 | (uint32_t)c.g << 8 | c.b; }
static inline rgb_t unpack_rgb(uint32_t v) { return fx_rgb(v >> 16, v >> 8, v); }
static inline uint32_t avg3(rgb_t c) { return ((uint32_t)c.r + c.g + c.b) * 85u >> 8; }

/* ------------------------------------------------------------------------------- fills */
static FX_NOINL void fx_solid(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    U_FILL(x, a, b, o, x->c[0]);
}

static FX_NOINL void prep_breathe(fx_ctx_t *x, uint32_t de)
{
    const uint32_t s = fx_sin8((x->et * 105u) >> 10);      /* a breath every ~2.5 s at 1x (WLED's pace) */
    const uint32_t lv = s * s >> 8;                         /* (squared: a longer rest at the bottom) */
    (void)de;
    x->u[0] = 255u - fx_scale8(255u - lv, x->intensity);   /* intensity = the depth */
}

static FX_NOINL void fx_breathe(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    U_FILL(x, a, b, o, fx_blend(x->c[1], x->c[0], x->u[0]));
}

static FX_NOINL void prep_colorloop(fx_ctx_t *x, uint32_t de)
{
    (void)de;
    x->u[0] = pack_rgb(fx_sat(fx_hue((x->et * 138u) >> 10), x->intensity));  /* ~1.9 s a round at 1x (WLED's pace) */
}

static FX_NOINL void fx_colorloop(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    U_FILL(x, a, b, o, unpack_rgb(x->u[0]));
}

/* ------------------------------------------------------------------------------- rainbow, palette
 * One rainbow / palette run along the strip. intensity = how many over the virtual length: 2^((I - 128) / 32), so 128 is one,
 * 0 is a sixteenth, 255 is sixteen (the law of WLED's Size slider, smoothed). Speed: a round in ~1.9 s (Rainbow, towards pixel 0)
 * and ~3.9 s (Palette, the other way) at 1x, the paces of WLED's namesakes. */
static FX_NOINL void prep_scroll(fx_ctx_t *x, uint32_t de)
{
    const uint32_t i = x->intensity, mant = 32u + (i & 31u);       /* (mant << (I >> 5)) / 512 = the count */
    (void)de;
    x->u[0] = ((mant << (i >> 5)) * 128u) / x->vlen;                /* the 16 bit step per pixel */
    x->u[1] = x->id == FX_PALETTE ? 0u - x->et * 17u : x->et * 35u;
}

static FX_NOINL void fx_rainbow(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t step = x->u[0];
    uint32_t h = x->u[1] + VP(x, a) * step;
    for (; a < b; a++, h += step) {
        fx_put(o + a, fx_hue(h >> 8));
    }
}

static FX_NOINL void fx_palette(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t step = x->u[0];
    uint32_t h = x->u[1] + VP(x, a) * step;
    for (; a < b; a++, h += step) {
        fx_put(o + a, fx_pal_at(x->pal, h));
    }
}

/* ------------------------------------------------------------------------------- Pride 2015 and ColorWaves
 * (Mark Kriegsman, FastLED, MIT). Slow beats modulate the hue spread, the brightness waves and their speed; the pixel's
 * hue / brightness phases are sums along the strip, which here are closed form (start + (i + 1) * increment), and the two
 * running sums over time (sPseudotime, sHue16) are the closed-form integrals of their rate beats: the picture is a pure function of
 * the effect time, so receivers that start at different moments draw the same one. The original's nblend with the previous
 * frame is left out (no per-pixel history). */
static FX_NOINL void prep_pride(fx_ctx_t *x, uint32_t de)
{
    const uint32_t e = x->et * 6u, wave = x->id == FX_PRIDE;        /* (6x: the pace of WLED's version, ~5.8x the original's) */
    const uint32_t hueinc = wave ? fx_beatsin88(e, 113u, 1u, 3000u, 0u) : fx_beatsin88(e, 113u, 300u, 1500u, 0u);
    (void)de;
    x->u[0] = fx_beat_cum(e, 147u, 23u, 60u) & 0xFFFFu;             /* sPseudotime: the integral of its rate beat, closed form */
    x->u[1] = fx_beat_cum(e, 400u, 5u, 9u) & 0xFFFFu;               /* sHue16 */
    x->u[2] = fx_beatsin88(e, 87u, 220u, 250u, 0u) << 8 | fx_beatsin88(e, 341u, 96u, 224u, 0u);   /* sat8, brightdepth */
    x->u[3] = fx_beatsin88(e, 203u, 25u * 256u, 40u * 256u, 0u);                       /* brightnessthetainc16 */
    x->u[4] = (hueinc * (x->intensity + 64u) * 341u) >> 16;                            /* (intensity 128: as the original) */
}

static FX_NOINL void fx_pride(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t sat = x->u[2] >> 8, depth = x->u[2] & 255u, tinc = x->u[3], hinc = x->u[4];
    const int pal = x->id == FX_COLORWAVES;
    for (; a < b; a++) {
        const uint32_t i = x->vlen - VP(x, a);                  /* (the original runs from the far end: pixel N-1-j, + 1) */
        const uint32_t hue16 = (x->u[1] + i * hinc) & 0xFFFFu;
        const uint32_t b16 = (uint32_t)(fx_sin16(x->u[0] + i * tinc) + 32768);
        const uint32_t bri16 = (b16 * b16) >> 16;
        const uint32_t bri = ((bri16 * depth) >> 16) + (255u - depth);
        if (pal) {                                                /* the hue folded into a triangle (as the original), over the whole palette */
            const uint32_t hf = (hue16 & 0x8000u) ? 0xFFFFu - hue16 : hue16;
            fx_put(o + a, fx_scale_rgb(fx_pal_at(x->pal, (hf * 2u * 240u) >> 8), bri));
        } else {
            fx_put(o + a, fx_scale_rgb(fx_sat(fx_hue(hue16 >> 8), sat), bri));
        }
    }
}

/* ------------------------------------------------------------------------------- Sinelon (FastLED DemoReel100's idea, WLED's behaviour)
 * A dot swings back and forth (12 beats a minute at 1x) and leaves a fading tail. The original fades the previous frame; here
 * the pixel's brightness is computed from the time since the dot last passed it (the angle the dot's phase has turned since:
 * an inverse sine finds the two phases at which it is at that pixel), so the tail stays where the dot has been, also after it
 * turned round, and nothing is kept between frames. intensity = Trail: higher is a SHORTER tail (as WLED). Dots: custom1 >= 128:
 * two, mirrored. The colour wanders round the palette. */
static FX_NOINL void prep_sinelon(fx_ctx_t *x, uint32_t de)
{
    const uint32_t tail = 800u + (255u - x->intensity) * 40u;      /* the tail in 1/65536 turns of the swing */
    (void)de;
    x->u[0] = fx_beat88(x->et, 12u * 256u);                         /* the swing's phase */
    x->u[1] = tail;
    x->u[2] = (x->et * 6u) & 0xFFFFu;                               /* the colour: a round in ~11 s */
    x->u[3] = (255u << 16) / tail;
    x->u[4] = (x->vlen > 1u ? x->vlen - 1u : 1u);                   /* span in pixels */
}

static uint32_t swing_bri(uint32_t th, uint32_t a1, uint32_t a2, uint32_t tail, uint32_t inv)
{
    const uint32_t d1 = (th - a1) & 0xFFFFu, d2 = (th - a2) & 0xFFFFu, d = d1 < d2 ? d1 : d2;   /* since the last pass */
    return d >= tail ? 0u : 255u - ((d * inv) >> 16);
}

static FX_NOINL void fx_sinelon(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const rgb_t col = fx_pal_at(x->pal, x->u[2]), bg = x->c[1];
    const uint32_t two = x->custom1 >= 128u, th = x->u[0], th2 = (x->u[0] + 32768u) & 0xFFFFu;
    const uint32_t tail = x->u[1], inv = x->u[3], inv_span = 0xFFFFFFFFu / x->u[4];
    for (; a < b; a++) {
        const uint32_t v = VP(x, a) < x->u[4] ? VP(x, a) : x->u[4];
        int32_t s = (int32_t)((v * inv_span) >> 16) - 32768;          /* the pixel's place on the swing: -32768 .. 32767 */
        uint32_t bri, a1, a2;
        s = s > 32767 ? 32767 : s;
        a1 = (uint32_t)fx_asin16(s) & 0xFFFFu;                        /* the two phases at which the dot is at this pixel */
        a2 = (32768u - a1) & 0xFFFFu;
        bri = swing_bri(th, a1, a2, tail, inv);
        if (two) {
            bri = fx_qadd8(bri, swing_bri(th2, a1, a2, tail, inv));
        }
        fx_put(o + a, fx_blend(bg, col, bri));
    }
}

/* ------------------------------------------------------------------------------- TwinkleFOX (Mark Kriegsman, FastLED, MIT)
 * Every pixel runs its own clock (speed 8/8 .. 23/8, random offset); a slow cycle derived from it decides whether the
 * pixel twinkles this round and in which palette colour, a fast cycle is its attack / decay. The original's pseudo
 * random sequence (a chain along the strip) is replaced by a hash of the pixel index, so any pixel can be rendered
 * alone. Background = colour 2. custom1 >= 128 switches the incandescent cooling off. */
static FX_NOINL void fx_twinklefox(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t dens = (x->intensity * 9u) >> 8, bgl = avg3(x->c[1]), et = x->et;
    const rgb_t bg = x->c[1];
    for (; a < b; a++) {
        const uint32_t h = fx_hashf(VP(x, a), x->seed), off = h & 0xFFFFu, r2 = h >> 16;
        const uint32_t mult = ((((r2 & 0xFFu) >> 4) + (r2 & 0x0Fu)) & 0x0Fu) + 8u, salt = r2 >> 8;
        const uint32_t clock = ((et * mult) >> 3) + off, ticks = (clock >> 4) & 0xFFFFu, fast = ticks & 255u;
        uint32_t slow = ((ticks >> 8) + salt) & 0xFFFFu;
        uint32_t slow8, bri = 0;
        rgb_t c = bg;
        slow = (slow + fx_sin8(slow & 255u)) & 0xFFFFu;
        slow = (slow * 2053u + 1384u) & 0xFFFFu;
        slow8 = ((slow & 255u) + (slow >> 8)) & 255u;
        if (((slow8 & 0x0Eu) >> 1) < dens) {
            bri = fast < 86u ? fast * 3u : 255u - ((fast - 86u) + ((fast - 86u) >> 1));
        }
        if (bri > 0u) {
            const uint32_t hue = (slow8 - salt) & 255u;
            rgb_t t = fx_scale_rgb(x->pal[hue >> 4], bri);                   /* (NOBLEND, as the original) */
            int32_t delta;
            if (x->custom1 < 128u && fast >= 128u) {                        /* cool like an incandescent bulb */
                const uint32_t cool = (fast - 128u) >> 4;
                t.g = fx_qsub8(t.g, cool);
                t.b = fx_qsub8(t.b, cool * 2u);
            }
            delta = (int32_t)avg3(t) - (int32_t)bgl;
            c = (delta >= 32 || (bg.r | bg.g | bg.b) == 0u) ? t : (delta > 0 ? fx_blend(bg, t, (uint32_t)delta * 8u) : bg);
        }
        fx_put(o + a, c);
    }
}

/* ------------------------------------------------------------------------------- Colortwinkles, Fairy
 * Stateless random twinkles: a pixel's clock gives it a cycle number and a phase; the hash of (pixel, cycle) decides
 * whether this cycle lights it (density = intensity) and in which palette colour. Colortwinkles: a quick rise and a
 * long fade; Fairy: sparse (a quarter of the density), a slow smooth swell. Background = colour 2. */
static void twinkle(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b, uint32_t soft)
{
    const uint32_t thr = soft ? (uint32_t)x->intensity >> 2 : x->intensity, et = x->et, sh = soft ? 13u : 12u;
    const rgb_t bg = x->c[1];
    for (; a < b; a++) {
        const uint32_t h = fx_hashf(VP(x, a), x->seed), mult = 8u + ((h >> 16) & 15u);
        const uint32_t clock = ((et * mult) >> 3) + (h & 0xFFFFu), cy = clock >> sh, ph = (clock & ((1u << sh) - 1u)) >> (sh - 8u);
        const uint32_t g = fx_hashf(VP(x, a) ^ x->seed, cy);
        uint32_t bri = 0;
        if ((g & 255u) < thr) {
            if (soft) {
                const uint32_t s = (uint32_t)fx_sin16(ph << 7) >> 7;         /* a half sine, squared */
                bri = (s * s) >> 8;
            } else if (ph < 24u) {
                bri = (ph * 171u) >> 4;
            } else {
                const uint32_t e = 255u - (((ph - 24u) * 282u) >> 8);
                bri = (e * e) >> 8;
            }
            fx_put(o + a, fx_blend(bg, fx_pal_at(x->pal, (g >> 8) & 0xFFFFu), bri > 255u ? 255u : bri));
        } else {
            fx_put(o + a, bg);
        }
    }
}

static FX_NOINL void fx_colortwinkles(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b) { twinkle(x, o, a, b, 0u); }
static FX_NOINL void fx_fairy(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b) { twinkle(x, o, a, b, 1u); }

/* ------------------------------------------------------------------------------- Candle, Candle Multi
 * A flame's brightness is smooth random: two octaves of hashed values (64 ms and 256 ms steps) interpolated in time.
 * The flame blends from colour 2 (the dim colour, black by default) to colour 1. intensity = how deep the flicker goes. */
static uint32_t flick(uint32_t e, uint32_t salt)
{
    const uint32_t s1 = e >> 6, f1 = fx_smooth8((e & 63u) << 2), s2 = e >> 8, f2 = fx_smooth8(e & 255u);
    const uint32_t a = fx_hashf(s1, salt) & 255u, b = fx_hashf(s1 + 1u, salt) & 255u;
    const uint32_t c = fx_hashf(s2, salt + 77u) & 255u, d = fx_hashf(s2 + 1u, salt + 77u) & 255u;
    const uint32_t fast = fx_lerp8(a, b, f1), slow = fx_lerp8(c, d, f2);
    return (fast * 3u + slow) >> 2;
}

static rgb_t flame(const fx_ctx_t *x, uint32_t f)
{
    return fx_blend(x->c[1], x->c[0], 255u - fx_scale8(f, x->intensity));
}

static FX_NOINL void prep_candle(fx_ctx_t *x, uint32_t de)
{
    (void)de;
    x->u[0] = pack_rgb(flame(x, flick(x->et, x->seed)));
}

static FX_NOINL void fx_candle(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    U_FILL(x, a, b, o, unpack_rgb(x->u[0]));
}

static FX_NOINL void fx_candle_multi(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    for (; a < b; a++) {
        fx_put(o + a, flame(x, flick(x->et, VP(x, a) * 0x9E3779B1u ^ x->seed)));
    }
}

/* ------------------------------------------------------------------------------- Aurora
 * Curtains of light: two value-noise fields drifting in opposite directions give the brightness (bright patches with dark
 * gaps between); which one leads at a pixel picks the palette position. intensity = the curtains' size. */
static FX_NOINL void prep_aurora(fx_ctx_t *x, uint32_t de)
{
    (void)de;
    x->u[0] = 65536u / (6u + ((255u - x->intensity) >> 3));  /* lattice units per pixel, 16 bit */
}

static FX_NOINL void fx_aurora(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    fx_nz1_t n1, n2;
    const uint32_t inc = x->u[0], drift = x->et >> 4;
    fx_nz1_begin(&n1, (x->et >> 5) + x->seed);
    fx_nz1_begin(&n2, (x->et >> 6) + 0x3100u + x->seed);
    for (; a < b; a++) {
        const uint32_t p = (VP(x, a) * inc) >> 8;
        const uint32_t v1 = fx_nz1(&n1, p + drift), v2 = fx_nz1(&n2, (p * 5u >> 2) + 0x40000u - (drift * 3u >> 1));
        uint32_t br = v1 > v2 ? v1 : v2, idx;
        br = fx_qsub8(br, 70u);
        br = fx_u8((br * 3u) >> 1);
        br = (br * br) >> 8;
        {
            const int32_t k = 128 + (((int32_t)v1 - (int32_t)v2) * 3 >> 1);     /* which curtain leads picks the colour */
            idx = k < 0 ? 0u : (k > 255 ? 255u : (uint32_t)k);
        }
        fx_put(o + a, fx_scale_rgb(fx_pal_lin(x->pal, idx), br));
    }
}

/* ------------------------------------------------------------------------------- Blends
 * A smooth rippling wash of the palette's colours in the palette's order: along the strip the palette index swings through the whole
 * palette and back (a sine ripple; intensity = how tight, 8 .. 40 pixels per ripple), and the whole pattern drifts through the
 * palette (a round in ~4 s at 1x), so the colours blend into each other as they move. */
static FX_NOINL void prep_blends(fx_ctx_t *x, uint32_t de)
{
    (void)de;
    x->u[0] = 65536u / (8u + ((255u - x->intensity) >> 3));    /* the ripple: turns per pixel, 16 bit */
    x->u[1] = (x->et * 16u) & 0xFFFFu;                          /* the drift */
}

static FX_NOINL void fx_blends(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    for (; a < b; a++) {
        const uint32_t w = (uint32_t)(fx_sin16(VP(x, a) * x->u[0]) + 32768) >> 8;     /* 0 .. 255 */
        fx_put(o + a, fx_pal_at(x->pal, x->u[1] + (w << 8)));
    }
}

/* ------------------------------------------------------------------------------- Flow
 * The strip is cut into an even number of zones that each show the whole palette once, neighbouring zones running in opposite
 * directions, the palette flowing through them (a round in ~2 s at 1x). intensity sets the zone LENGTH (6 pixels at 255 up to half
 * the strip at 0); the zone count is the strip over that, rounded down to even (at least 2), and the zones are spread evenly over
 * the strip (borders at z * length / zones), so no pixels are left over. */
static FX_NOINL void prep_flow(fx_ctx_t *x, uint32_t de)
{
    const uint32_t half = x->vlen >> 1, span = half > 6u ? half - 6u : 0u;
    const uint32_t zl = 6u + (((255u - x->intensity) * span) >> 8);
    uint32_t zones = x->vlen / zl;
    (void)de;
    zones &= ~1u;
    x->u[0] = zones < 2u ? 2u : zones;
    x->u[1] = (x->et * 33u) & 0xFFFFu;                          /* the flow */
}

static FX_NOINL void fx_flow(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t zones = x->u[0], cnt = x->u[1], vlen = x->vlen;
    uint32_t v = VP(x, a), z = 0, zs = 0, ze = 0, inv = 0, zlen = 1;
    for (; a < b; a++, v++) {
        uint32_t i;
        if (v >= vlen) {                                        /* (outside the virtual strip) */
            fx_put(o + a, x->c[0]);
            continue;
        }
        if (v >= ze || v < zs) {                                /* into another zone: its borders and step */
            z = ((v + 1u) * zones - 1u) / vlen;                 /* the zone whose first pixel (z * length / zones) is the last one at or before v */
            zs = (z * vlen) / zones;
            ze = ((z + 1u) * vlen) / zones;
            zlen = ze > zs ? ze - zs : 1u;
            inv = 65536u / zlen;
        }
        i = v - zs;
        fx_put(o + a, fx_pal_at(x->pal, ((z & 1u) ? i : zlen - 1u - i) * inv - cnt));
    }
}

/* ------------------------------------------------------------------------------- Fire 2012 (Mark Kriegsman, FastLED, MIT)
 * Heat per pixel (the state, one byte each): cool down, drift up, spark at the start. Steps run at ~33 per second of
 * effect time. speed = the pace, intensity = sparking, custom1 = cooling. The heat goes through the palette (Heat). */
static uint32_t fire_cells(const fx_ctx_t *x) { return x->n < FX_FIRE_CELLS ? x->n : FX_FIRE_CELLS; }

static FX_NOINL void fire_step(fx_ctx_t *x)
{
    uint8_t *h = x->state;
    const uint32_t n = fire_cells(x), cool = ((uint32_t)x->custom1 * 10u) / n + 2u;
    uint32_t k;
    for (k = 0; k < n; k++) {
        x->rnd ^= x->rnd << 13;
        x->rnd ^= x->rnd >> 17;
        x->rnd ^= x->rnd << 5;
        h[k] = fx_qsub8(h[k], ((x->rnd & 255u) * cool) >> 8);
    }
    for (k = n - 1u; k >= 2u; k--) {
        h[k] = (uint8_t)(((uint32_t)h[k - 1u] + h[k - 2u] + h[k - 2u]) * 85u >> 8);
    }
    x->rnd ^= x->rnd << 13;
    x->rnd ^= x->rnd >> 17;
    x->rnd ^= x->rnd << 5;
    if ((x->rnd & 255u) < x->intensity) {
        k = ((x->rnd >> 8) & 255u) * 7u >> 8;
        h[k] = fx_qadd8(h[k], 160u + ((((x->rnd >> 16) & 255u) * 95u) >> 8));
    }
}

static FX_NOINL void prep_fire(fx_ctx_t *x, uint32_t de)
{
    uint32_t acc = x->u[0] + de, steps = 0;
    if (x->state == 0 || fire_cells(x) < 8u) {
        return;
    }
    while (acc >= 30u && steps < 4u) {
        acc -= 30u;
        steps++;
        fire_step(x);
    }
    x->u[0] = acc >= 30u ? 0u : acc;                          /* (a long gap: drop the rest) */
}

static FX_NOINL void fx_fire(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint8_t *h = x->state;
    const uint32_t m = fire_cells(x);
    if (h == 0 || m < 8u) {
        U_FILL(x, a, b, o, x->c[0]);
        return;
    }
    if (m == x->n) {
        for (; a < b; a++) {
            fx_put(o + a, fx_pal_lin(x->pal, h[a]));
        }
    } else {                                                 /* more pixels than cells: interpolate between the cells */
        const uint32_t cpp = (m << 16) / x->n;               /* cells per pixel, 16.16 */
        for (; a < b; a++) {
            const uint32_t pos = a * cpp, c = pos >> 16, nx = c + 1u < m ? c + 1u : c;
            fx_put(o + a, fx_pal_lin(x->pal, fx_lerp8(h[c], h[nx], (pos >> 8) & 255u)));
        }
    }
}

/* ------------------------------------------------------------------------------- Gradient
 * c1 -> c2 -> c3 across the strip. Intensity < 16: once, standing still, ending in c3. Else it loops back to c1 and
 * repeats ((I - 16) / 32 + 1 times) and scrolls with the speed. */
#define GRAD_PERIOD (3u << 16)
static FX_NOINL void prep_gradient(fx_ctx_t *x, uint32_t de)
{
    const uint32_t cyc = x->intensity >= 16u;
    (void)de;
    if (cyc) {
        const uint32_t rep = ((x->intensity - 16u) >> 5) + 1u;
        x->u[0] = (rep * GRAD_PERIOD) / x->vlen;
        x->u[1] = (x->et * 16u) % GRAD_PERIOD;
    } else {
        x->u[0] = (2u << 16) / (x->vlen > 1u ? x->vlen - 1u : 1u);
        x->u[1] = 0;
    }
}

static FX_NOINL void fx_gradient(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t cyc = x->intensity >= 16u, step = x->u[0];
    uint32_t g = VP(x, a) * step + x->u[1];
    if (cyc) {
        g %= GRAD_PERIOD;
    }
    for (; a < b; a++) {
        const uint32_t s = g >> 16, f = (g >> 8) & 255u;
        if (cyc) {
            fx_put(o + a, fx_blend(x->c[s], x->c[s == 2u ? 0u : s + 1u], f));
            g += step;
            if (g >= GRAD_PERIOD) {
                g -= GRAD_PERIOD;
            }
        } else {
            fx_put(o + a, s >= 2u ? x->c[2] : fx_blend(x->c[s], x->c[s + 1u], f));
            g += step;
        }
    }
}

/* ------------------------------------------------------------------------------- Pacifica (Mark Kriegsman and Mary
 * Corey March, FastLED, MIT). Four wave layers of the sea palettes, each a slowly drifting colour index that is itself a
 * sum along the strip; here the sums are closed form at the start of each call (the sum of a sine is a cosine) and
 * stepped from there. Whitecaps where the layers add up bright; the blues and greens deepened. Intensity = whitecaps. */
static FX_NOINL void prep_pacifica(fx_ctx_t *x, uint32_t de)
{
    const uint32_t e = x->et;
    (void)de;
    /* sCIStart1..4: the integrals of (speed beat / 256) * (a second beat): closed form, a pure function of the time */
    x->u[0] = (fx_beat_cum2(e, 3u * 256u, 179u, 269u, 1011u, 10u, 13u) >> 8) & 0xFFFFu;
    x->u[1] = (0u - (((fx_beat_cum2(e, 3u * 256u, 179u, 269u, 777u, 8u, 11u) >> 1) +
                      (fx_beat_cum2(e, 4u * 256u, 179u, 269u, 777u, 8u, 11u) >> 1)) >> 8)) & 0xFFFFu;
    x->u[2] = (0u - (fx_beat_cum2(e, 3u * 256u, 179u, 269u, 501u, 5u, 7u) >> 8)) & 0xFFFFu;
    x->u[3] = (0u - (fx_beat_cum2(e, 4u * 256u, 179u, 269u, 257u, 4u, 6u) >> 8)) & 0xFFFFu;
}

/* the sum of the layer's colour-index increments for pixels 1..m, closed form (the increments are
 * wh * (1.5 + sin / 2) - 0.5; 250 turn-units apart; 1 / (2 sin(125 units)) = 41.72) */
static uint32_t pac_sum(uint32_t th0, uint32_t m, uint32_t wh)
{
    const int32_t dcos = fx_cos16(th0 + 125u) - fx_cos16(th0 + 250u * m + 125u);
    const int32_t s16 = (dcos * 10680) >> 8;
    return (wh * 3u * m >> 1) - (m >> 1) + (uint32_t)(((s16 >> 4) * (int32_t)wh) >> 12);
}

#define PAC_BLOCK 32u                                      /* the closed form is evaluated at multiples of this */

typedef struct { uint32_t ci[4], wh[4], br[4], ang0[4]; int32_t sn[4], cn[4]; } pac_layers_t;   /* (sn, cn: sin, cos of the wave angle, Q15) */

static void pac_start(const fx_ctx_t *x, pac_layers_t *P, uint32_t base)   /* the layers' state before pixel `base` */
{
    uint32_t L;
    for (L = 0; L < 4u; L++) {
        const uint32_t ang = P->ang0[L] + 250u * base;
        P->ci[L] = x->u[L] + pac_sum(P->ang0[L], base, P->wh[L]);
        P->sn[L] = fx_sin16(ang);
        P->cn[L] = fx_cos16(ang);
    }
}

/* one pixel on: the wave angle turns by 250 units (a rotation in Q15: cos 250 = 32758 / 32768, sin 250 = 785 / 32768; it is
 * restarted exactly every PAC_BLOCK pixels) and the colour index grows by the wave's increment */
FX_INL void pac_step(pac_layers_t *P, uint32_t L)
{
    const int32_t s = P->sn[L], c = P->cn[L];
    P->sn[L] = (s * 32758 + c * 785) >> 15;
    P->cn[L] = (c * 32758 - s * 785) >> 15;
    P->ci[L] += (((uint32_t)(P->sn[L] + 32768)) * P->wh[L] >> 16) + P->wh[L];
}

static FX_NOINL void fx_pacifica(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t e = x->et;
    pac_layers_t P;
    uint32_t L, wave, wthr, v = VP(x, a), cnt = b - a;
    P.wh[0] = fx_beatsin88(e, 3u * 256u, 11u * 256u, 14u * 256u, 0u);
    P.wh[1] = fx_beatsin88(e, 4u * 256u, 6u * 256u, 9u * 256u, 0u);
    P.wh[2] = 6u * 256u;
    P.wh[3] = 5u * 256u;
    P.br[0] = fx_beatsin88(e, 10u * 256u, 70u, 130u, 0u);
    P.br[1] = fx_beatsin88(e, 17u * 256u, 40u, 80u, 0u);
    P.br[2] = fx_beatsin88(e, 9u * 256u, 10u, 38u, 0u);
    P.br[3] = fx_beatsin88(e, 8u * 256u, 10u, 28u, 0u);
    P.ang0[0] = 0u - fx_beat88(e, 301u);
    P.ang0[1] = fx_beat88(e, 401u);
    P.ang0[2] = 0u - fx_beat88(e, 503u);
    P.ang0[3] = fx_beat88(e, 601u);
    for (L = 0; L < 4u; L++) {
        P.wh[L] = P.wh[L] / 2u + 20u;
    }
    wthr = fx_beatsin88(e, 9u * 256u, 55u, 65u, 0u);
    wave = fx_beat88(e, 7u * 256u) >> 8;
    while (cnt != 0u) {
        /* the closed form is re-anchored at every PAC_BLOCK-th pixel, so the picture does not depend on how the strip
         * is cut into calls; a call starting inside a block steps through the pixels before its first */
        uint32_t skip = v % PAC_BLOCK, run = PAC_BLOCK - skip;
        pac_start(x, &P, v - skip);
        run = run > cnt ? cnt : run;
        for (; skip != 0u; skip--) {
            for (L = 0; L < 4u; L++) {
                pac_step(&P, L);
            }
        }
        cnt -= run;
        for (; run != 0u; run--, a++, v++) {
            rgb_t c = fx_rgb(2, 6, 10);
            uint32_t avg, thr;
            for (L = 0; L < 4u; L++) {
                const rgb_t *p = fx_pal_pacifica[L == 0u ? 0 : (L == 1u ? 1 : 2)];
                uint32_t sidx;
                pac_step(&P, L);
                sidx = ((uint32_t)(fx_sin16_i(P.ci[L]) + 32768) * 240u) >> 16;         /* 0..239 */
                c = fx_add_sat(c, fx_scale_rgb(fx_pal_at_i(p, sidx << 8), P.br[L]));
            }
            thr = fx_scale8(fx_sin8((wave + 7u * v) & 255u), 20u) + wthr;
            avg = avg3(c);
            if (avg > thr) {
                const uint32_t over = fx_scale8(avg - thr, x->intensity), over2 = fx_qadd8(over, over);
                c.r = fx_qadd8(c.r, over);
                c.g = fx_qadd8(c.g, over2);
                c.b = fx_qadd8(c.b, fx_qadd8(over2, over2));
            }
            c.b = fx_scale8(c.b, 145u);
            c.g = fx_scale8(c.g, 200u);
            c.r = c.r > 2u ? c.r : 2u;
            c.g = c.g > 5u ? c.g : 5u;
            c.b = c.b > 7u ? c.b : 7u;
            fx_put(o + a, c);
        }
    }
}

/* ------------------------------------------------------------------------------- Sunrise, Sunset
 * Raw effect time (not speed-scaled): the progress t / duration, duration = 10 s + 14 s * speed (speed 128: 30 min).
 * The palette runs from dark to day; intensity spreads the progress along the strip (the sun rises at the far end
 * first). Sunset runs it backwards and stays dark. */
static FX_NOINL void prep_sun(fx_ctx_t *x, uint32_t de)
{
    const uint32_t dur = 10000u + 14000u * x->speed, k = 0xFFFFFFFFu / dur;      /* k = 2^32 / duration */
    uint32_t p = x->t >= dur ? 65535u : (uint32_t)(((uint64_t)x->t * k) >> 16);
    (void)de;
    if (p > 65535u) {
        p = 65535u;
    }
    x->u[0] = x->id == FX_SUNSET ? 65535u - p : p;
    x->u[1] = 65536u / x->vlen;
}

static FX_NOINL void fx_sun(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t p = x->u[0], sp = x->intensity, base = (p * (256u + sp)) >> 8;
    for (; a < b; a++) {
        const uint32_t q = (VP(x, a) * x->u[1]) >> 8;                  /* 0..255 along the strip */
        const uint32_t sub = q * sp;                                    /* 16 bit */
        uint32_t pl = base > sub ? base - sub : 0u;
        if (pl > 65535u) {
            pl = 65535u;
        }
        fx_put(o + a, fx_pal_lin(x->pal, pl >> 8));
    }
}

/* ------------------------------------------------------------------------------- Glitter
 * The palette moves slowly along the strip (two rounds per strip); every ~30 ms a few random pixels flash colour 1.
 * Intensity = how many. */
static FX_NOINL void prep_glitter(fx_ctx_t *x, uint32_t de)
{
    (void)de;
    x->u[0] = (131072u) / x->vlen;
    x->u[1] = (x->et * 17u) & 0xFFFFu;                      /* a round in ~3.9 s at 1x (WLED's pace) */
    x->u[2] = x->et >> 5;                                   /* the sparkle slot */
}

static FX_NOINL void fx_glitter(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t step = x->u[0], thr = (uint32_t)x->intensity >> 2, key = fx_hash(x->u[2] ^ x->seed);
    for (; a < b; a++) {
        const uint32_t v = VP(x, a);
        fx_put(o + a, (fx_hashf(v, key) & 255u) < thr ? x->c[0] : fx_pal_at(x->pal, x->u[1] + v * step));
    }
}

/* ------------------------------------------------------------------------------- Meteor
 * A head in colour 1 runs along the strip (a lap takes the same time on any length, as WLED's); its tail (intensity = length, a share of the strip) fades with random
 * shimmer; colour 2 is the background. It runs out of the strip's end before it starts again. */
static FX_NOINL void prep_meteor(fx_ctx_t *x, uint32_t de)
{
    const uint32_t tail = 3u + ((x->vlen * x->intensity) >> 10);
    const uint64_t loop = ((uint64_t)x->vlen + tail) << 8;
    (void)de;
    x->u[0] = (uint32_t)(((uint64_t)((x->et * 40u) & 0xFFFFu) * loop) >> 16);   /* the head, 24.8 px: a lap in ~1.6 s at 1x, whatever the length */
    x->u[1] = (255u << 8) / tail;
    x->u[2] = tail << 8;
    x->u[3] = x->et >> 5;
}

static FX_NOINL void fx_meteor(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    for (; a < b; a++) {
        const uint32_t v = VP(x, a);
        const int32_t d = (int32_t)x->u[0] - (int32_t)(v << 8);
        uint32_t bri = 0;
        if (d >= 0 && (uint32_t)d <= x->u[2]) {
            const uint32_t k = ((uint32_t)d * x->u[1]) >> 16;
            bri = k >= 255u ? 0u : 255u - k;
            if (d > 256) {
                bri = fx_scale8(bri, 112u + (fx_hashf(v, x->u[3] ^ x->seed) & 127u));
            }
        }
        fx_put(o + a, fx_blend(x->c[1], x->c[0], bri));
    }
}

/* ------------------------------------------------------------------------------- Plasma, Noise, Running */
static FX_NOINL void fx_plasma(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t k = 1u + (x->intensity >> 5), w = 400u * k, t1 = x->et * 3u, t2 = x->et * 5u, t3 = x->et * 2u;
    for (; a < b; a++) {
        const uint32_t v = VP(x, a);
        const int32_t s = fx_sin16(v * w + t1) + fx_sin16(v * (w * 7u >> 2) - t2) + fx_sin16((v * (w * 5u >> 3)) + t3 + 9000u);
        fx_put(o + a, fx_pal_lin(x->pal, ((uint32_t)(s + 98304) * 1365u) >> 20));
    }
}

static FX_NOINL void prep_noise(fx_ctx_t *x, uint32_t de)
{
    (void)de;
    x->u[0] = 65536u / (4u + ((255u - x->intensity) >> 3));
}

static FX_NOINL void fx_noise(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    fx_nz1_t n1, n2;
    const uint32_t inc = x->u[0];
    fx_nz1_begin(&n1, (x->et >> 5) + x->seed);
    fx_nz1_begin(&n2, (x->et >> 4) + 0x5100u + x->seed);
    for (; a < b; a++) {
        const uint32_t p = (VP(x, a) * inc) >> 8;
        const uint32_t v = (fx_nz1(&n1, p) * 3u + fx_nz1(&n2, p * 2u)) >> 2;
        fx_put(o + a, fx_pal_lin(x->pal, fx_u8((fx_qsub8(v, 50u) * 5u) >> 2)));
    }
}

static FX_NOINL void prep_running(fx_ctx_t *x, uint32_t de)
{
    (void)de;
    x->u[0] = 65536u / (3u + ((255u - x->intensity) >> 3));  /* the wavelength 3 .. 34 px, as a step in 16-bit turns */
    x->u[1] = x->et * 65u;                                   /* a round a second at 1x */
}

static FX_NOINL void fx_running(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    for (; a < b; a++) {
        fx_put(o + a, fx_blend(x->c[1], x->c[0], fx_sin8((VP(x, a) * x->u[0] - x->u[1]) >> 8)));
    }
}

/* ------------------------------------------------------------------------------- the table and the engine */
typedef struct { render_fn render; prep_fn prep; } fx_def_t;
static const fx_def_t fx_defs[FX_COUNT] = {
    { fx_solid, 0 },                      /* 0: no effect (never rendered) */
    { fx_solid, 0 },
    { fx_breathe, prep_breathe },
    { fx_colorloop, prep_colorloop },
    { fx_rainbow, prep_scroll },
    { fx_palette, prep_scroll },
    { fx_pride, prep_pride },             /* colorwaves */
    { fx_pride, prep_pride },
    { fx_sinelon, prep_sinelon },
    { fx_twinklefox, 0 },
    { fx_colortwinkles, 0 },
    { fx_candle, prep_candle },
    { fx_candle_multi, 0 },
    { fx_aurora, prep_aurora },
    { fx_blends, prep_blends },
    { fx_flow, prep_flow },
    { fx_fire, prep_fire },
    { fx_gradient, prep_gradient },
    { fx_pacifica, prep_pacifica },
    { fx_sun, prep_sun },
    { fx_sun, prep_sun },
    { fx_glitter, prep_glitter },
    { fx_meteor, prep_meteor },
    { fx_fairy, 0 },
    { fx_plasma, 0 },
    { fx_noise, prep_noise },
    { fx_running, prep_running },
};

uint32_t fx_state_bytes(uint8_t id, uint32_t n)
{
    return id == FX_FIRE ? (n < FX_FIRE_CELLS ? n : FX_FIRE_CELLS) : 0u;
}

uint8_t fx_default_palette(uint8_t id)
{
    return id < FX_COUNT ? fx_pal_default_tab[id] : 0u;
}

static void build_palette(fx_ctx_t *x)
{
    uint32_t k;
    if (x->pal_id == PAL_COLORS) {                            /* c1 -> c2 -> c3 -> c1 over the 16 entries */
        for (k = 0; k < 16u; k++) {
            const uint32_t s = (k * 3u) >> 4, f = ((k * 3u) & 15u) << 4;
            x->pal[k] = fx_blend(x->c[s], x->c[s == 2u ? 0u : s + 1u], f + (f >> 4));
        }
    } else {
        for (k = 0; k < 16u; k++) {
            x->pal[k] = fx_pals[x->pal_id < PAL_COUNT ? x->pal_id : 0u][k];
        }
    }
}

void fx_begin(fx_ctx_t *x, const fx_params_t *p, uint16_t n, uint8_t *state)
{
    uint32_t k;
    x->id = p->id < FX_COUNT ? p->id : (uint8_t)FX_SOLID;
    x->speed = p->speed;
    x->intensity = p->intensity;
    x->palette = p->palette;
    x->custom1 = p->custom1;
    x->custom2 = p->custom2;
    x->flags = p->flags;
    x->reserved = 0;
    for (k = 0; k < 3u; k++) {
        x->c[k] = p->c[k];
    }
    x->n = n;
    x->vlen = p->vlen != 0u ? p->vlen : (n != 0u ? n : 1u);
    x->vbase = p->vstart;
    x->seed = p->seed;
    x->t = x->t_last = x->et = 0;
    x->et_frac = 0;
    x->rnd = p->seed | 1u;
    for (k = 0; k < 5u; k++) {
        x->u[k] = 0;
    }
    x->state = state;
    if (state != 0) {
        for (k = 0; k < fx_state_bytes(x->id, n); k++) {
            state[k] = 0;
        }
    }
    x->pal_id = p->palette == PAL_DEFAULT ? fx_default_palette(x->id) : p->palette;
    build_palette(x);
}

void fx_set_colors(fx_ctx_t *x, const rgb_t c[3])
{
    x->c[0] = c[0];
    x->c[1] = c[1];
    x->c[2] = c[2];
    if (x->pal_id == PAL_COLORS) {
        build_palette(x);
    }
}

static uint32_t fx_rate256(uint32_t s)                    /* effect ms per host ms, x 256 */
{
    return s == 0u ? 0u : 8u + ((s * 15u) >> 3);
}

void fx_frame(fx_ctx_t *x, uint32_t t_ms)
{
    int32_t dt = (int32_t)(t_ms - x->t_last);
    uint64_t acc;
    uint32_t de;
    if (dt < 0) {                                         /* (the host clock stepped back: stand still) */
        dt = 0;
    }
    acc = (uint64_t)(uint32_t)dt * fx_rate256(x->speed) + x->et_frac;
    de = (uint32_t)(acc >> 8);
    x->et += de;
    x->et_frac = (uint8_t)acc;
    x->t = x->t + dt < x->t ? 0xFFFFFFFFu : x->t + (uint32_t)dt;      /* (raw effect time: counts up, saturates; no wrap at 49 days) */
    x->t_last = t_ms;
    if (x->pal_id == PAL_COLORS) {
        build_palette(x);
    }
    if (fx_defs[x->id].prep != 0) {
        fx_defs[x->id].prep(x, de);
    }
}

void fx_set_speed(fx_ctx_t *x, uint8_t speed)
{
    x->speed = speed;
}

void fx_render(fx_ctx_t *x, rgb_t *out, uint32_t from, uint32_t to)       /* out[0 .. to - from) */
{
    if (to > x->n) {
        to = x->n;
    }
    if (from < to) {                                      /* (the effects index out[] by pixel: bias the base) */
        fx_defs[x->id].render(x, (rgb_t *)((uintptr_t)out - from * sizeof(rgb_t)), from, to);
    }
}
