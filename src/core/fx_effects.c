/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "math32.h"
/* The effects (fx.h): fx_begin / fx_frame / fx_render and the 36 effects of the catalogue. Integer only.
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
#include <string.h>
#ifndef FX_V407
#define FX_V407 0
#endif
#if FX_V407
#include "fx_v407/fxv_gen.h"                               /* the V407 Digital-M's exact palettes 31.. */
#endif

extern const uint8_t fx_pal_default_tab[FX_COUNT];

#define VP(x, i)   ((x)->vbase + (i))                    /* the pixel's place in the virtual strip */
#define U_FILL(x, a, b, o, c) do { const rgb_t c_ = (c); for (; (a) < (b); (a)++) { fx_put((o) + (a), c_); } } while (0)

typedef void (*render_fn)(fx_ctx_t *, rgb_t *, uint32_t, uint32_t);

static inline uint32_t pack_rgb(rgb_t c) { return (uint32_t)c.r << 16 | (uint32_t)c.g << 8 | c.b; }
static inline rgb_t unpack_rgb(uint32_t v) { return fx_rgb(v >> 16, v >> 8, v); }
static inline uint32_t avg3(rgb_t c) { return ((uint32_t)c.r + c.g + c.b) * 85u >> 8; }

/* ------------------------------------------------------------------------------- with a palette
 * An effect whose Effect default is its own colours runs on a chosen palette with pal_own 0 (fx_begin): its motion and
 * brightness, the palette's colour. Solid, Breathe, Candle, Candle Multi and Canopy Shade's wash lay the palette end to end
 * along the strip: fx_begin sets u[4] = (2 << 16) / (vlen - 1) and u[3] = 255 for them (Breathe and Candle set u[3], the
 * spread's brightness, each frame), and fx_candle_multi renders the spread. */
static FX_NOINL rgb_t spread_px(const fx_ctx_t *x, uint32_t v, uint32_t level)   /* virtual pixel v of the spread at level */
{
    return fx_scale_rgb(fx_pal_at(x->pal, v + 1u >= x->vlen ? 61440u : (v * x->u[4] * 15u) >> 5), level);   /* (61440: entry 15) */
}

static FX_NOINL void fx_candle_multi(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b);

/* ------------------------------------------------------------------------------- fills */
static FX_NOINL void fx_fill(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)   /* colour 1 (the stream id 0, Fire without heat) */
{
    U_FILL(x, a, b, o, x->c[0]);
}

static FX_NOINL void fx_solid(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    if (!x->pal_own) {                                   /* a palette: end to end along the strip, still */
        fx_candle_multi(x, o, a, b);
        return;
    }
    fx_fill(x, o, a, b);
}

static void prep_breathe(fx_ctx_t *x, uint32_t de)
{
    const uint32_t s = fx_sin8((x->et * 105u) >> 10);      /* a breath every ~2.5 s at 1x (WLED's pace) */
    const uint32_t lv = s * s >> 8;                         /* (squared: a longer rest at the bottom) */
    const uint32_t f = 255u - fx_scale8(255u - lv, x->intensity);
    (void)de;
    /* intensity = the depth; the frame's colour worked out here (the engine ramps the colours before each frame's prep):
     * the render is fx_colorloop's fill of u[0]; with a palette the palette along the strip breathes */
    x->u[3] = f;
    x->u[0] = pack_rgb(fx_blend(x->c[1], x->c[0], f));
}

static void prep_colorloop(fx_ctx_t *x, uint32_t de)
{
    const uint32_t h = x->et * 138u;                        /* ~1.9 s a round at 1x (WLED's pace); a palette: round it */
    (void)de;
    x->u[0] = pack_rgb(fx_sat(x->pal_own ? fx_hue(h >> 10) : fx_pal_at(x->pal, h >> 2), x->intensity));
}

static FX_NOINL void fx_colorloop(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)   /* (Breathe's and Candle's render too) */
{
    if (!x->pal_own && x->id != FX_COLORLOOP) {          /* Breathe, Candle with a palette: the palette along the strip */
        fx_candle_multi(x, o, a, b);
        return;
    }
    U_FILL(x, a, b, o, unpack_rgb(x->u[0]));
}

/* ------------------------------------------------------------------------------- rainbow, palette
 * One rainbow / palette run along the strip. intensity = how many over the virtual length: 2^((I - 128) / 32), so 128 is one,
 * 0 is a sixteenth, 255 is sixteen (the law of WLED's Size slider, smoothed). Speed: a round in ~1.9 s (Rainbow, towards pixel 0)
 * and ~3.9 s (Palette, the other way) at 1x, the paces of WLED's namesakes. */
static void prep_scroll(fx_ctx_t *x, uint32_t de)   /* (Glitter too: two rounds over the strip, a round in ~3.9 s at 1x, WLED's pace) */
{
    const uint32_t i = x->intensity, mant = 32u + (i & 31u);       /* (mant << (I >> 5)) / 512 = the count */
    const uint32_t id = x->id;
    (void)de;
    x->u[0] = (id == FX_GLITTER ? 131072u : (mant << (i >> 5)) * 128u) / x->vlen;   /* the 16 bit step per pixel */
    x->u[1] = id == FX_PALETTE ? 0u - x->et * 17u : x->et * (id == FX_GLITTER ? 17u : 35u);
    x->u[2] = x->et >> 5;                                           /* (Glitter: the sparkle slot) */
}

static FX_NOINL void fx_palette(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t step = x->u[0];
    uint32_t h = x->u[1] + VP(x, a) * step;
    for (; a < b; a++, h += step) {
        fx_put(o + a, fx_pal_at(x->pal, h));
    }
}

static FX_NOINL void fx_rainbow(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t step = x->u[0];
    uint32_t h = x->u[1] + VP(x, a) * step;
    if (!x->pal_own) {                                   /* a palette: it moves along the strip at the rainbow's pace */
        fx_palette(x, o, a, b);
        return;
    }
    for (; a < b; a++, h += step) {
        fx_put(o + a, fx_hue(h >> 8));
    }
}

/* ------------------------------------------------------------------------------- Pride 2015 and ColorWaves
 * (Mark Kriegsman, FastLED, MIT). Slow beats modulate the hue spread, the brightness waves and their speed; the pixel's
 * hue / brightness phases are sums along the strip, which here are closed form (start + (i + 1) * increment), and the two
 * running sums over time (sPseudotime, sHue16) are the closed-form integrals of their rate beats: the picture is a pure function of
 * the effect time, so receivers that start at different moments draw the same one. The original's nblend with the previous
 * frame is left out (no per-pixel history). A virtual strip under 8 px is shorter than one brightness wave (6-10 px) and
 * would pulse as one at 2-5 Hz: it runs at the original's pace. */
static FX_NOINL void prep_pride(fx_ctx_t *x, uint32_t de)
{
    const uint32_t e = x->vlen < 8u ? x->et : x->et * 6u, wave = x->id == FX_PRIDE;   /* (6x: the pace of WLED's version, ~5.8x the original's) */
    const uint32_t hueinc = wave ? fx_beatsin88(e, 113u, 1u, 3000u, 0u) : fx_beatsin88(e, 113u, 300u, 1500u, 0u);
    (void)de;
    x->u[0] = fx_beat_cum(e, 147u, 23u, 60u);                       /* sPseudotime: the integral of its rate beat, closed form (an angle: its low 16 bits) */
    x->u[1] = fx_beat_cum(e, 400u, 5u, 9u);                         /* sHue16 (the same) */
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
        rgb_t c;
        if (pal) {                                                /* the hue folded into a triangle (as the original), over the whole palette */
            const uint32_t hf = (hue16 & 0x8000u) ? 0xFFFFu - hue16 : hue16;
            c = fx_pal_at(x->pal, (hf * 2u * 240u) >> 8);
        } else if (!x->pal_own) {                                 /* Pride on a palette: the hue walks the palette */
            c = fx_pal_at(x->pal, hue16);
        } else {
            c = fx_sat(fx_hue(hue16 >> 8), sat);
        }
        fx_put(o + a, fx_scale_rgb(c, bri));
    }
}

/* ------------------------------------------------------------------------------- Sinelon (FastLED DemoReel100's idea, WLED's behaviour)
 * A dot swings back and forth (12 beats a minute at 1x) and leaves a fading tail. The original fades the previous frame; here
 * the pixel's brightness is computed from the time since the dot last passed it (the angle the dot's phase has turned since:
 * an inverse sine finds the two phases at which it is at that pixel), so the tail stays where the dot has been, also after it
 * turned round, and nothing is kept between frames. intensity = Trail: higher is a SHORTER tail (as WLED). Dots: custom1 >= 128:
 * two, mirrored. The colour wanders round the palette. */
static void prep_sinelon(fx_ctx_t *x, uint32_t de)
{
    const uint32_t tail = 800u + (255u - x->intensity) * 40u;      /* the tail in 1/65536 turns of the swing */
    (void)de;
    x->u[0] = fx_beat88(x->et, 12u * 256u);                         /* the swing's phase */
    x->u[1] = tail;
    x->u[2] = x->et * 6u;                                           /* the colour: a round in ~11 s (a palette index: its low 16 bits) */
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
    const uint32_t two = x->custom1 >= 128u, th = x->u[0], th2 = x->u[0] + 32768u;   /* (the phases: low 16 bits, swing_bri) */
    const uint32_t tail = x->u[1], inv = x->u[3], inv_span = 0xFFFFFFFFu / x->u[4];
    for (; a < b; a++) {
        const uint32_t v = VP(x, a) < x->u[4] ? VP(x, a) : x->u[4];
        int32_t s = (int32_t)((v * inv_span) >> 16) - 32768;          /* the pixel's place on the swing: -32768 .. 32767 */
        uint32_t bri, a1, a2;
        s = s > 32767 ? 32767 : s;
        a1 = (uint32_t)fx_asin16(s);                                  /* the two phases at which the dot is at this pixel */
        a2 = 32768u - a1;
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
        const uint32_t clock = ((et * mult) >> 3) + (h & 0xFFFFu), cy = (clock >> sh) & (0x1FFFFFFFu >> sh), ph = (clock & ((1u << sh) - 1u)) >> (sh - 8u);
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
            fx_put(o + a, fx_blend(bg, fx_pal_at(x->pal, g >> 8), bri > 255u ? 255u : bri));
        } else {
            fx_put(o + a, bg);
        }
    }
}

static FX_NOINL void fx_colortwinkles(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b) { twinkle(x, o, a, b, 0u); }
static FX_NOINL void fx_fairy(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b) { twinkle(x, o, a, b, 1u); }

/* ------------------------------------------------------------------------------- Candle, Candle Multi
 * Candle Multi's flames: smooth random brightness, two octaves of hashed values (64 ms and 256 ms steps) interpolated in time;
 * on a 1-px strip it is one flame: Candle's (prep_candle runs for it there).
 * Candle: a calm flame. Every 128 ms a draught dips it at a random moment by a random depth q^2 (q 0..255): a 64-ms fall,
 * then q drops by 1 every 3 ms, so the flame comes back fast from deep and settles slowly near the top. The deepest dip of
 * the last 9 slots (1,152 ms, longer than any dip) wins, plus a slow wander (flick at 1/8 the pace: 512-ms and 2-s steps;
 * it steps once when et wraps at 2^32). Only the falls are quick: no rise is fast enough to count as a flash (the
 * 3-per-second flash-comfort gate holds for any colour at speed and intensity 128).
 * The flame blends from colour 2 (the dim colour, black by default) to colour 1. intensity = how deep the flicker goes. */
static uint32_t flick(uint32_t e, uint32_t salt)
{
    const uint32_t s1 = e >> 6, f1 = fx_smooth8((e & 63u) << 2), s2 = e >> 8, f2 = fx_smooth8(e & 255u);
    const uint32_t a = fx_hashf(s1, salt) & 255u, b = fx_hashf((s1 + 1u) & 0x3FFFFFFu, salt) & 255u;     /* (the cell index wraps with e at 2^32) */
    const uint32_t c = fx_hashf(s2, salt + 77u) & 255u, d = fx_hashf((s2 + 1u) & 0xFFFFFFu, salt + 77u) & 255u;
    const uint32_t fast = fx_lerp8(a, b, f1), slow = fx_lerp8(c, d, f2);
    return (fast * 3u + slow) >> 2;
}

static void prep_candle(fx_ctx_t *x, uint32_t de)
{
    const uint32_t e = x->et, gd = fx_scale8(100u, x->intensity) + 1u;
    uint32_t k, g = 0, f;
    (void)de;
    for (k = 0; k < 9u; k++) {
        const uint32_t s = ((e >> 7) - k) & 0x1FFFFFFu, h = fx_hashf(s, x->seed);
        const uint32_t t = e - (s << 7) - ((h >> 16) & 127u);      /* (before the dip: huge, so q runs out) */
        const uint32_t r = t < 64u ? 0u : (t - 64u) / 3u;
        uint32_t q = (h >> 8) & 255u, v;
        q = r < q ? q - r : 0u;
        v = q * q * gd >> 16;
        if (t < 64u) v = v * t >> 6;                              /* (the fall) */
        if (v > g) g = v;
    }
    f = 255u - g - fx_scale8(flick(e >> 3, x->seed + 99u), fx_scale8(16u, x->intensity));
    x->u[3] = f;                                         /* (a palette: the spread flickers as one) */
    x->u[0] = pack_rgb(fx_blend(x->c[1], x->c[0], f));   /* (rendered by fx_colorloop's fill) */
}

static FX_NOINL void fx_candle_multi(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)   /* (and the spread at u[3]) */
{
    const uint32_t two = x->vlen == 2u;                  /* two flames would often flicker a 2-px strip as one: the second mirrors the first */
    const uint32_t own = x->id == FX_CANDLE_MULTI && x->vlen != 1u;   /* (a 1-px strip: one flame, Candle's calm one from prep_candle) */
    for (; a < b; a++) {
        const uint32_t v = VP(x, a);
        uint32_t f = x->u[3];
        if (own) {
            const uint32_t m = v & two;                    /* (1: the 2-px strip's second flame) */
            f = 255u - fx_scale8(flick(x->et, (v - m) * 0x9E3779B1u ^ x->seed) ^ ((0u - m) & 255u), x->intensity);
        }
        fx_put(o + a, x->pal_own ? fx_blend(x->c[1], x->c[0], f) : spread_px(x, v, f));
    }
}

/* ------------------------------------------------------------------------------- Aurora
 * Curtains of light: two value-noise fields drifting in opposite directions give the brightness (bright patches with dark
 * gaps between); which one leads at a pixel picks the palette position. intensity = the curtains' size. */
/* Aurora, Blends, Noise, Running: u[0] = 65536 / (K + (255 - intensity) / 8), the size slider's step (Aurora: lattice units per
 * pixel; Blends: the ripple, turns per pixel; Noise: lattice units; Running: the wavelength 3 .. 34 px as a step in turns), and
 * u[1] the drift (Blends: a round in ~4 s at 1x; Running: a round a second; the other two don't read it) */
static void prep_slider(fx_ctx_t *x, uint32_t de)
{
    const uint32_t id = x->id;
    const uint32_t k = id == FX_AURORA ? 6u : id == FX_BLENDS ? 8u : id == FX_NOISE ? 4u : 3u;
    (void)de;
    x->u[0] = 65536u / (k + ((255u - x->intensity) >> 3));
    x->u[1] = x->et * (id == FX_BLENDS ? 16u : 65u);
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
static void prep_flow(fx_ctx_t *x, uint32_t de)
{
    const uint32_t half = x->vlen >> 1, span = half > 6u ? half - 6u : 0u;
    const uint32_t zl = 6u + (((255u - x->intensity) * span) >> 8);
    uint32_t zones = x->vlen / zl;
    (void)de;
    zones &= ~1u;
    x->u[0] = zones < 2u ? 2u : zones;
    x->u[1] = x->et * 33u;                                      /* the flow (a palette index: its low 16 bits) */
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

static uint32_t xorshift(uint32_t r)
{
    r ^= r << 13;
    r ^= r >> 17;
    return r ^ (r << 5);
}

/* One bounded slice. u[2] = steps left (+ FX_FIRE_INIT for previous-heat clearing), u[3] = cooling/diffusion cursor,
 * u[4] = frozen cooling and sparking parameters (a command can change them). */
uint32_t fx_frame_step(fx_ctx_t *x)
{
    uint8_t *h = x->state;
    const uint32_t n = fire_cells(x);
    uint32_t k, end, pos = x->u[3];
    if (x->id != FX_FIRE || !x->u[2]) return 0;
    if (x->u[2] & FX_FIRE_INIT) {
        end = pos + 64u < n ? pos + 64u : n;
        for (; pos < end; pos++) x->fire_prev[pos] = 0;
        x->u[3] = pos == n ? 0u : pos;
        if (pos == n) x->u[2] &= ~FX_FIRE_INIT;
        return 1;
    }
    if (pos < n) {
        end = pos + 64u < n ? pos + 64u : n;
        for (; pos < end; pos++) {
            if (x->fire_prev) x->fire_prev[pos] = h[pos];
            x->u[1] = xorshift(x->u[1]);
            h[pos] = fx_qsub8(h[pos], ((x->u[1] & 255u) * (x->u[4] & 65535u)) >> 8);
        }
    } else {
        end = pos + 64u < 2u * n - 2u ? pos + 64u : 2u * n - 2u;
        for (; pos < end; pos++) {
            k = 2u * n - 1u - pos;
            h[k] = (uint8_t)(((uint32_t)h[k - 1u] + h[k - 2u] + h[k - 2u]) * 85u >> 8);
        }
        if (pos == 2u * n - 2u) {
            x->u[1] = xorshift(x->u[1]);
            if ((x->u[1] & 255u) < (x->u[4] >> 16)) {
                k = ((x->u[1] >> 8) & 255u) * 7u >> 8;
                h[k] = fx_qadd8(h[k], 160u + ((((x->u[1] >> 16) & 255u) * 95u) >> 8));
            }
            x->u[2]--;
            pos = 0;
        }
    }
    x->u[3] = pos;
    return 1;
}

static FX_NOINL void prep_fire(fx_ctx_t *x, uint32_t de)
{
    uint32_t acc = x->u[0] + de;
    if (x->state == 0 || fire_cells(x) < 8u) {
        return;
    }
    x->u[2] = (x->u[2] & FX_FIRE_INIT) | (acc / 30u < 4u ? acc / 30u : 4u);
    x->u[0] = acc >= 150u ? 0u : acc % 30u;                /* same four-step/drop policy */
    x->u[3] = 0;
    x->u[4] = ((uint32_t)x->custom1 * 10u) / fire_cells(x) + 2u + ((uint32_t)x->intensity << 16);
}

static FX_NOINL void fx_fire(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint8_t *h = x->state;
    const uint32_t m = fire_cells(x);
    if (h == 0 || m < 8u) {                                  /* (no heat: colour 1, with a palette too) */
        fx_fill(x, o, a, b);
        return;
    }
    {                                                        /* more pixels than cells: interpolate between the cells (as many as pixels: a zero fraction) */
        const uint32_t cpp = (m << 16) / x->n;               /* cells per pixel, 16.16 */
        const uint32_t mix = ((x->u[0] << 8) + x->et_frac) / 30u;
        for (; a < b; a++) {
            const uint32_t pos = a * cpp, c = pos >> 16, nx = c + 1u < m ? c + 1u : c;
            uint32_t heat = fx_lerp8(h[c], h[nx], (pos >> 8) & 255u);
            if (x->fire_prev) {
                const uint8_t *old = x->fire_prev;
                heat = fx_lerp8(fx_lerp8(old[c], old[nx], (pos >> 8) & 255u), heat, mix);
            }
            fx_put(o + a, fx_pal_lin(x->pal, heat));
        }
    }
}

/* ------------------------------------------------------------------------------- Gradient
 * c1 -> c2 -> c3 across the strip. Intensity < 16: once, standing still, ending in c3. Else it loops back to c1 and
 * repeats ((I - 16) / 32 + 1 times) and scrolls with the speed. */
#define GRAD_PERIOD (3u << 16)
static void prep_gradient(fx_ctx_t *x, uint32_t de)
{
    const uint32_t cyc = x->intensity >= 16u;
    (void)de;
    if (cyc) {
        const uint32_t rep = ((x->intensity - 16u) >> 5) + 1u;
        x->u[0] = (rep * GRAD_PERIOD) / x->vlen;
        x->u[1] = ((x->et * 4u) & 0xFFFFu) * 3u;                 /* a turn (c1 -> c2 -> c3 -> c1) in 2^14 ms = 16.4 s at 1x: a power of two, so nothing jumps when et wraps */
    } else {
        x->u[0] = (2u << 16) / (x->vlen > 1u ? x->vlen - 1u : 1u);
        x->u[1] = 0;
    }
}

static FX_NOINL void fx_gradient(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t cyc = x->intensity >= 16u;
    const uint32_t step = cyc ? x->u[0] % GRAD_PERIOD : x->u[0];
    const uint32_t vp=VP(x,a),pl=vp*step+x->u[1],ph=m32_hi(vp,step)+(pl<vp*step);
    uint32_t g;
    if (cyc) {
        /* Short/virtual strips can cross several colour cycles per pixel. Reduce
         * the step too; the initial product must also survive a large offset. */
        /* GRAD_PERIOD = 3 * 65536: only the whole-colour index needs modulo 3.
         * Its upper part fits 32 bits; no 64-bit divide is needed. */
        g = ((((ph<<16)|(pl>>16))%3u)<<16)|(pl&65535u);
    } else {
        g = (ph||pl>(2u<<16))?(2u<<16):pl;
    }
    for (; a < b; a++) {
        uint32_t s = g >> 16, f = (g >> 8) & 255u, nx = s + 1u;
        if (cyc) {
            nx = s == 2u ? 0u : nx;
        } else if (s >= 2u) {                                /* past the end: colour 3 */
            s = nx = 2u;
        }
        /* a palette: round it once a repeat (g / 3), or once end to end (61440: entry 15) */
        fx_put(o + a, x->pal_own ? fx_blend(x->c[s], x->c[nx], f) :
                      fx_pal_at(x->pal, cyc ? (g * 21845u) >> 16 : (g >= (2u << 16) ? 61440u : (g * 15u) >> 5)));
        g += step;
        if (cyc && g >= GRAD_PERIOD) {
            g -= GRAD_PERIOD;
        }
    }
}

/* ------------------------------------------------------------------------------- Pacifica (Mark Kriegsman and Mary
 * Corey March, FastLED, MIT). Four wave layers of the sea palettes, each a slowly drifting colour index that is itself a
 * sum along the strip; here the sums are closed form at the start of each call (the sum of a sine is a cosine) and
 * stepped from there. Whitecaps where the layers add up bright; the blues and greens deepened. Intensity = whitecaps. */
#if !FX_NO_PACIFICA
static FX_NOINL void prep_pacifica(fx_ctx_t *x, uint32_t de)
{
    const uint32_t e = x->et;
    (void)de;
    /* sCIStart1..4: the integrals of (speed beat / 256) * (a second beat): closed form, a pure function of the time */
    x->u[0] = fx_beat_cum2(e, 3u * 256u, 179u, 269u, 1011u, 10u, 13u) >> 8;   /* (unmasked: the sines use 16 bits, the palette more) */
    x->u[1] = (0u - (((fx_beat_cum2(e, 3u * 256u, 179u, 269u, 777u, 8u, 11u) >> 1) +
                      (fx_beat_cum2(e, 4u * 256u, 179u, 269u, 777u, 8u, 11u) >> 1)) >> 8));
    x->u[2] = 0u - (fx_beat_cum2(e, 3u * 256u, 179u, 269u, 501u, 5u, 7u) >> 8);
    x->u[3] = 0u - (fx_beat_cum2(e, 4u * 256u, 179u, 269u, 257u, 4u, 6u) >> 8);
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
FX_AI void pac_step(pac_layers_t *P, uint32_t L)
{
    const int32_t s = P->sn[L], c = P->cn[L];
    P->sn[L] = (s * 32758 + c * 785) >> 15;
    P->cn[L] = (c * 32758 - s * 785) >> 15;
    P->ci[L] += (((uint32_t)(P->sn[L] + 32768)) * P->wh[L] >> 16) + P->wh[L];
}

/* Pacifica on a chosen palette: each layer keeps its sea palette's brightness at its wave (the largest channel there, so the
 * troughs stay dark) and takes its colour from the palette at its own running colour index (an eighth of the palette a
 * wave, drifting with the layer), so the layers cross in different palette colours and the whole palette comes round;
 * a palette colour lights one or two channels where the sea lit two, so the level is doubled; the whitecaps are white */
FX_AI rgb_t pac_pal_px(const fx_ctx_t *x, pac_layers_t *P, uint32_t v, uint32_t wave, uint32_t wthr)
{
    rgb_t c = fx_rgb(0, 0, 0);
    uint32_t L, avg, thr;
    for (L = 0; L < 4u; L++) {
        const rgb_t *p = fx_pal_pacifica[L == 0u ? 0 : (L == 1u ? 1 : 2)];
        uint32_t sidx, lum;
        rgb_t sea;
        pac_step(P, L);
        sidx = ((uint32_t)(fx_sin16_i(P->ci[L]) + 32768) * 240u) >> 16;
        sea = fx_pal_at_i(p, sidx << 8);
        lum = sea.r > sea.g ? sea.r : sea.g;
        lum = lum > sea.b ? lum : sea.b;
        c = fx_add_sat_ai(c, fx_scale_rgb_ai(fx_pal_at_i(x->pal, P->ci[L] >> 3), fx_u8((lum * P->br[L]) >> 7)));
    }
    thr = fx_scale8(fx_sin8((wave + 7u * v) & 255u), 20u) + wthr;
    avg = avg3(c);
    if (avg > thr) {
        const uint32_t over = fx_scale8(avg - thr, x->intensity), w = fx_qadd8(over, over);
        c = fx_add_sat_ai(c, fx_rgb(w, w, w));
    }
    return c;
}

/* Preserve the hot loop's propagation when the other effects favour compact code. */
#if defined(__GNUC__)
__attribute__((optimize("tree-forwprop")))
#endif
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
    const uint32_t beat9 = (uint32_t)(fx_sin16(fx_beat88(e, 9u * 256u)) + 32768);
    P.br[2] = 10u + ((28u * beat9) >> 16);
    P.br[3] = fx_beatsin88(e, 8u * 256u, 10u, 28u, 0u);
    P.ang0[0] = 0u - fx_beat88(e, 301u);
    P.ang0[1] = fx_beat88(e, 401u);
    P.ang0[2] = 0u - fx_beat88(e, 503u);
    P.ang0[3] = fx_beat88(e, 601u);
    for (L = 0; L < 4u; L++) {
        P.wh[L] = P.wh[L] / 2u + 20u;
    }
    wthr = 55u + ((10u * beat9) >> 16);             /* the same beat, with its original separate rounding */
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
            if (!x->pal_own) {
                fx_put(o + a, pac_pal_px(x, &P, v, wave, wthr));
                continue;
            }
            for (L = 0; L < 4u; L++) {
                const rgb_t *p = fx_pal_pacifica[L == 0u ? 0 : (L == 1u ? 1 : 2)];
                uint32_t sidx;
                pac_step(&P, L);
                sidx = ((uint32_t)(fx_sin16_i(P.ci[L]) + 32768) * 240u) >> 16;         /* 0..239 */
                c = fx_add_sat_ai(c, fx_scale_rgb_ai(fx_pal_at_i(p, sidx << 8), P.br[L]));
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

#if FX_V407
/* fx_pacifica in resumable pieces for the V407 layer (fx.h): its frame's layers once, then the pixels in order with its
 * anchors at every PAC_BLOCK-th pixel and its steps, so the pixels are its own; a piece is at most 16 anchor steps or
 * `max` pixels. (The pixel loop is fx_pacifica's, kept apart: the V203 images stay byte-identical.) */
typedef struct { pac_layers_t P; uint32_t wave, wthr, v, a, skip, n; } pac_run_t;
typedef char pac_run_fits[sizeof(pac_run_t) <= sizeof(fx_pac_run_t) ? 1 : -1];

void fx_pacifica_begin(const fx_ctx_t *x, fx_pac_run_t *run)
{
    pac_run_t *r = (pac_run_t *)(void *)run;
    const uint32_t e = x->et;
    uint32_t L, beat9;
    r->P.wh[0] = fx_beatsin88(e, 3u * 256u, 11u * 256u, 14u * 256u, 0u);
    r->P.wh[1] = fx_beatsin88(e, 4u * 256u, 6u * 256u, 9u * 256u, 0u);
    r->P.wh[2] = 6u * 256u;
    r->P.wh[3] = 5u * 256u;
    r->P.br[0] = fx_beatsin88(e, 10u * 256u, 70u, 130u, 0u);
    r->P.br[1] = fx_beatsin88(e, 17u * 256u, 40u, 80u, 0u);
    beat9 = (uint32_t)(fx_sin16(fx_beat88(e, 9u * 256u)) + 32768);
    r->P.br[2] = 10u + ((28u * beat9) >> 16);
    r->P.br[3] = fx_beatsin88(e, 8u * 256u, 10u, 28u, 0u);
    r->P.ang0[0] = 0u - fx_beat88(e, 301u);
    r->P.ang0[1] = fx_beat88(e, 401u);
    r->P.ang0[2] = 0u - fx_beat88(e, 503u);
    r->P.ang0[3] = fx_beat88(e, 601u);
    for (L = 0; L < 4u; L++) {
        r->P.wh[L] = r->P.wh[L] / 2u + 20u;
    }
    r->wthr = 55u + ((10u * beat9) >> 16);
    r->wave = fx_beat88(e, 7u * 256u) >> 8;
    r->v = VP(x, 0u);
    r->a = 0;
    r->n = x->n;
    r->skip = r->v % PAC_BLOCK;
    pac_start(x, &r->P, r->v - r->skip);
}

uint32_t fx_pacifica_run(const fx_ctx_t *x, fx_pac_run_t *run, rgb_t *o, uint32_t max)
{
    pac_run_t *r = (pac_run_t *)(void *)run;
    uint32_t L, end;
    if (r->skip != 0u) {                                   /* the first block's pixels before the strip (as a call's skip) */
        end = r->skip > 16u ? 16u : r->skip;
        r->skip -= end;
        for (; end != 0u; end--) {
            for (L = 0; L < 4u; L++) {
                pac_step(&r->P, L);
            }
        }
        return 1;
    }
    end = r->a + max < r->n ? r->a + max : r->n;
    for (; r->a < end; r->a++, r->v++) {
        rgb_t c = fx_rgb(2, 6, 10);
        uint32_t avg, thr;
        if (r->v % PAC_BLOCK == 0u && r->a != 0u) pac_start(x, &r->P, r->v);
        if (!x->pal_own) {
            fx_put(o + r->a, pac_pal_px(x, &r->P, r->v, r->wave, r->wthr));
            continue;
        }
        for (L = 0; L < 4u; L++) {
            const rgb_t *p = fx_pal_pacifica[L == 0u ? 0 : (L == 1u ? 1 : 2)];
            uint32_t sidx;
            pac_step(&r->P, L);
            sidx = ((uint32_t)(fx_sin16_i(r->P.ci[L]) + 32768) * 240u) >> 16;
            c = fx_add_sat_ai(c, fx_scale_rgb_ai(fx_pal_at_i(p, sidx << 8), r->P.br[L]));
        }
        thr = fx_scale8(fx_sin8((r->wave + 7u * r->v) & 255u), 20u) + r->wthr;
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
        fx_put(o + r->a, c);
    }
    return r->a < r->n;
}
#endif
#endif
/* ------------------------------------------------------------------------------- Sunrise, Sunset
 * Raw effect time (not speed-scaled): the progress t / duration, duration = 10 s + 14 s * speed (speed 128: 30 min).
 * The palette runs from dark to day; intensity spreads the progress along the strip (the sun rises at the far end
 * first). Sunset runs it backwards and stays dark. */
static void prep_sun(fx_ctx_t *x, uint32_t de)
{
    const uint32_t dur = 10000u + 14000u * x->speed, k = 0xFFFFFFFFu / dur;      /* k = 2^32 / duration */
    uint32_t p = x->t >= dur ? 65535u : m32_shr(x->t,k,16);
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
 * Intensity = how many. A virtual strip under 3 px would flash whole with every sparkle: there a pixel sparkles at most once
 * in 64 slots (~2 s at 1x, with 16 times a slot's chance, at one of the first 16), lighting at once and fading out over 48
 * (the V407 short-strip hold's look). */
static FX_NOINL void fx_glitter(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t step = x->u[0], s = x->u[2], sh = x->vlen < 3u ? 4u : 0u;            /* (short: a key per 64 slots) */
    const uint32_t thr = ((uint32_t)x->intensity >> 2) << sh, key = fx_hash(s >> (sh + (sh >> 1)) ^ x->seed);
    for (; a < b; a++) {
        const uint32_t v = VP(x, a), h = fx_hashf(v, key), lit = (h & 255u) < thr;
        rgb_t c = x->c[0];
        if (sh) {
            const uint32_t age = (s & 63u) - ((h >> 8) & 15u);
            c = fx_blend(fx_pal_at(x->pal, x->u[1] + v * step), c, lit && age < 48u ? 255u - (age * age * 7u >> 6) : 0u);
        } else if (!lit) {
            c = fx_pal_at(x->pal, x->u[1] + v * step);
        }
        fx_put(o + a, c);
    }
}

/* ------------------------------------------------------------------------------- Meteor
 * A head in colour 1 runs along the strip (a lap takes the same time on any length, as WLED's); its tail (intensity = length, a share of the strip) fades with random
 * shimmer; colour 2 is the background. It runs out of the strip's end before it starts again. */
static void prep_meteor(fx_ctx_t *x, uint32_t de)
{
    const uint32_t tail = 3u + ((x->vlen * x->intensity) >> 10);
    const uint32_t loop=(x->vlen+tail)<<8;
    (void)de;
    x->u[0] = m32_shr((x->et*40u)&65535u,loop,16);   /* the head, 24.8 px: a lap in ~1.6 s at 1x, whatever the length */
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
        rgb_t col = x->c[0];
        if (d >= 0 && (uint32_t)d <= x->u[2]) {
            const uint32_t k = ((uint32_t)d * x->u[1]) >> 16;
            bri = k >= 255u ? 0u : 255u - k;
            if (d > 256 && x->vlen > 1u) {               /* (on 1 px the shimmer would flicker the whole strip) */
                bri = fx_scale8(bri, 112u + (fx_hashf(v, x->u[3] ^ x->seed) & 127u));
            }
            if (!x->pal_own && k < 255u) {               /* a palette: the head its last colour, the tail back through it */
                col = fx_pal_lin(x->pal, 255u - k);
            }
        }
        fx_put(o + a, fx_blend(x->c[1], col, bri));
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

static FX_NOINL void fx_running(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    for (; a < b; a++) {
        const uint32_t ph = VP(x, a) * x->u[0] - x->u[1];
        rgb_t lo = x->c[1], hi = x->c[0];
        if (!x->pal_own) {                               /* a palette: the waves carry it (an eighth a wave), the troughs dark */
            lo = fx_rgb(0, 0, 0);
            hi = fx_pal_at(x->pal, ph >> 3);
        }
        fx_put(o + a, fx_blend(lo, hi, fx_sin8(ph >> 8)));
    }
}

/* ------------------------------------------------------------------------------- Ripples, Wipe, Stripes, Bulbs, Lava Lamp
 * All five are our own designs (nothing from WLED or FastLED), pure functions of the effect time like the rest. */

/* The coverage 0..255 of the window [0, duty) on a 16-bit phase circle, seen by pixels `step` turns apart: the window's edges are ramps one
 * pixel wide centred on the edge, so stripes and bulbs glide over the pixels instead of stepping (mk = (255 << 12) / step). */
static uint32_t window_cov(uint32_t ph, uint32_t duty, uint32_t mk)
{
    int32_t s;
    ph &= 0xFFFFu;
    if (duty == 0u) {
        return 0u;
    }
    if (duty >= 65535u) {
        return 255u;
    }
    if (ph < duty) {
        s = (int32_t)(ph < duty - ph ? ph : duty - ph);
    } else {
        const uint32_t d = ph - duty, e = 65536u - ph;
        s = -(int32_t)(d < e ? d : e);
    }
    s = 128 + (((s >> 4) * (int32_t)mk) >> 8);
    return s < 0 ? 0u : (s > 255 ? 255u : (uint32_t)s);
}

/* The period 2 px .. 504 px in 1/16 px for a slider position (doubling every 32 steps; 255 is the finest) */
static uint32_t period16(uint32_t slider)
{
    const uint32_t j = 255u - slider;
    return (32u + (j & 31u)) << (j >> 5);
}

/* Ripples: raindrops on a pond. Three drop slots, a third of a cycle (1.4 s) apart, each cycle drops a ring (when the slot's roll is under
 * Rain) at a random place; the ring spreads (Size = how far, 8 .. 72 px), 2.5 px each side wide, and fades with its age. The ring takes a
 * random palette colour; rings add up where they cross; Colour 2 is the water. */
#define RIP_T  4096u                                       /* a slot's cycle, ms at 1x (2^12: 2^32 is a multiple, so the cycle index is continuous across the effect time's wrap) */
#define RIP_W  40u                                         /* the ring's / pearl's half width, 1/16 px */
typedef struct { uint32_t c, r; rgb_t col; } fx_pearl_t;   /* a ring of radius r (0: a dot) at c (1/16 px) in col */

/* Ripples' and Quiet Landings' pixels: colour 2 plus the rings / pearls that reach the pixel (the same soft edge) */
static void pearls(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b, const fx_pearl_t *dr, uint32_t nd)
{
    uint32_t j;
    for (; a < b; a++) {
        const uint32_t v = VP(x, a) << 4;
        rgb_t c = x->c[1];
        for (j = 0; j < nd; j++) {
            const uint32_t d = v > dr[j].c ? v - dr[j].c : dr[j].c - v, e = d > dr[j].r ? d - dr[j].r : dr[j].r - d;
            if (e < RIP_W) {
                c = fx_add_sat(c, fx_scale_rgb(dr[j].col, ((RIP_W - e) * 51u) >> 3));          /* (255 at the centre) */
            }
        }
        fx_put(o + a, c);
    }
}

static FX_NOINL void fx_ripples(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    fx_pearl_t dr[3];
    const uint32_t rmax = 128u + 4u * x->intensity;
    uint32_t j, nd = 0;
    for (j = 0; j < 3u; j++) {
        const uint32_t tt = x->et + j * (RIP_T / 3u), n = tt >> 12, age = tt & (RIP_T - 1u), h = fx_hashf(n * 3u + j, x->seed);
        if ((h & 255u) < x->custom1) {
            const uint32_t left = 255u - ((age * 255u) >> 12);      /* 255 .. 1 */
            dr[nd].c = (((h >> 8) & 0xFFFFu) * x->vlen) >> 12;
            dr[nd].r = (age * rmax) >> 12;
            dr[nd].col = fx_scale_rgb(fx_pal_at(x->pal, fx_hash(h) >> 16), (left * left) >> 8);
            nd++;
        }
    }
    pearls(x, o, a, b, dr, nd);
}

/* Wipe: a colour wipes over the strip from pixel 0, then the strip holds, then the next colour does the same. A cycle is 4.096 s at 1x
 * (a power of two, so the cycle count is continuous when the effect time wraps at 2^32); Hold takes up to three quarters of it
 * (I * 12 ms), the sweep the rest. Colours: colour 1 and 2 take turns, or (Colours 128 and up) every wipe is a new palette colour (a golden-ratio step round the
 * palette). The edge is one pixel soft. */
static void prep_wipe(fx_ctx_t *x, uint32_t de)
{
    const uint32_t sweep = 4096u - x->intensity * 12u, n = x->et >> 12, r = x->et & 4095u;
    uint32_t k;
    (void)de;
    x->u[0] = (r < sweep ? (r << 16) / sweep : 65536u) * x->vlen;
    for (k = 0; k < 2u; k++) {
        const uint32_t turn = n - k;
        const rgb_t c = x->custom1 < 128u ? x->c[turn & 1u] : fx_pal_at(x->pal, turn * 0x9E37u);
        x->u[k + 1u] = pack_rgb(c);
    }
}

static FX_NOINL void fx_wipe(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const rgb_t f = unpack_rgb(x->u[1]), k = unpack_rgb(x->u[2]);
    for (; a < b; a++) {
        const uint32_t v = VP(x, a), edge = x->u[0] >> 16;
        /* Compare whole pixels before the fraction: the virtual strip may span
         * all 65535 pixels, beyond the signed 16.16 coordinate range. */
        const uint32_t mix = v < edge ? 255u : v > edge ? 0u : (x->u[0] >> 8) & 255u;
        fx_put(o + a, fx_blend(k, f, mix));
    }
}

/* Stripes: colour 1 and colour 2 in hard-edged stripes (period from Size, colour 1 takes Share of it) moving towards the higher pixels at
 * ~20 px/s at 1x; the edges are one pixel soft. */
static FX_NOINL void fx_stripes(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    for (; a < b; a++) {
        const uint32_t ph = VP(x, a) * x->u[0] - x->u[3], h = ph - (x->u[2] >> 1);
        rgb_t c1 = x->c[0], c2 = x->c[1];
        if (!x->pal_own) {                               /* a palette, hard-edged: period n's stripes are entries n and n + 8 */
            c1 = x->pal[((h + 32768u) >> 16) & 15u];     /* (each edge pixel blends the two stripes it touches) */
            c2 = x->pal[((h >> 16) + 8u) & 15u];
        }
        fx_put(o + a, fx_blend(c2, c1, window_cov(ph, x->u[2], x->u[1])));
    }
}

/* Bulbs: a string of lights. One bulb per cell (Spacing), lit for Size of the cell with soft edges, the rest dark (colour 2); bulb n takes
 * the palette colour (n - shift) * Colour step, and the shift grows by one every 0.68 s at 1x, so the colours hop along the string. */
static FX_NOINL void fx_bulbs(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t step = x->u[0], cs = (uint32_t)x->custom2 << 8;
    for (; a < b; a++) {
        const uint32_t ph = VP(x, a) * step, cov = window_cov(ph - x->u[4], x->u[3], x->u[1]);
        rgb_t c = x->c[1];
        if (cov != 0u) {
            c = fx_blend(c, fx_pal_at(x->pal, ((ph >> 16) - x->u[2]) * cs), cov);
        }
        fx_put(o + a, c);
    }
}

/* Lava Lamp: three big soft blobs (radius from Blob size, 9 % .. 40 % of the strip) wander to and fro with their own periods (13, 16 and
 * 22 s at 1x); each has a palette colour (the palette turns once a minute); blobs add up where they meet, colour 2 is the background. */
static FX_NOINL void prep_lava(fx_ctx_t *x, uint32_t de)
{
    const uint32_t rs = (6000u + x->intensity * 80u) >> 4, sd = x->seed * 0x9E37u;
    uint32_t c[3], k;
    (void)de;
    for (k = 0; k < 3u; k++) {
        c[k] = (uint32_t)(32768 + ((fx_sin16(x->et * (5u - k) + k * 21845u + sd) * 24000) >> 15));
    }
    x->u[0] = c[0] | c[1] << 16;
    x->u[1] = c[2] | (x->et & 0xFFFFu) << 16;
    x->u[2] = 0xFFFFFFFFu / x->vlen;
    x->u[4] = rs | ((255u << 8) / rs) << 16;
}

static FX_NOINL void fx_lava(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t inv = x->u[2], rs = x->u[4] & 0xFFFFu, kk = x->u[4] >> 16;
    const uint32_t cc[3] = { x->u[0] & 0xFFFFu, x->u[0] >> 16, x->u[1] & 0xFFFFu };
    rgb_t col[3];
    uint32_t k;
    for (k = 0; k < 3u; k++) {
        col[k] = fx_pal_at(x->pal, (x->u[1] >> 16) + k * 0x5555u);
    }
    for (; a < b; a++) {
        const uint32_t f = (VP(x, a) * inv) >> 16;
        rgb_t c = x->c[1];
        for (k = 0; k < 3u; k++) {
            const uint32_t d = (f > cc[k] ? f - cc[k] : cc[k] - f) >> 4;
            if (d < rs) {
                const uint32_t t = ((rs - d) * kk) >> 8;
                c = fx_add_sat(c, fx_scale_rgb(col[k], fx_smooth8(t)));
            }
        }
        fx_put(o + a, c);
    }
}

/* ------------------------------------------------------------------------------- Mosaic Drift, Silk Weave, Canopy Shade, Opal Facets, Quiet Landings
 * Our own designs again (nothing from WLED or FastLED). Every time period
 * is a power of two in ms, so 2^32 is a multiple of it and nothing jumps when the effect time wraps. */
#define NEUTRAL_WHITE fx_rgb(255, 250, 235)

static uint32_t cell_step(uint32_t slider)                /* turns per pixel (16 bit) of cells 2 .. 504 px wide, doubling every 32 steps */
{
    return (1u << 20) / period16(slider);
}

/* Mosaic Drift: fixed adjoining tiles (Tile size), each melting from one palette colour to the next over 8.2 s at 1x (every tile on its own clock),
 * eased; Whiten mixes neutral white in. A tile edge is anti-aliased over one pixel. */
static rgb_t mosaic_tile(const fx_ctx_t *x, uint32_t n)
{
    const uint32_t tt = x->et + (fx_hashf(n, x->seed) & 8191u), c = tt >> 13, key = n + x->seed * 0x9E37u, m = 0x7FFFFu;
    const rgb_t a = fx_pal_at(x->pal, fx_hashf(key, c & m) >> 16), b = fx_pal_at(x->pal, fx_hashf(key, (c + 1u) & m) >> 16);
    return fx_blend(fx_blend(a, b, fx_smooth8((tt & 8191u) >> 5)), NEUTRAL_WHITE, x->custom1);
}

/* Stripes, Bulbs, Silk Weave, Mosaic Drift, Canopy Shade, Opal Facets: u[0] = the cell step (turns per pixel), u[1] = the soft
 * edge's slope for that step; Stripes adds its scroll (u[3]) and share (u[2]), Bulbs its colour hop (u[2]), duty (u[3]) and
 * window centre (u[4]), Silk Weave its second ribbon's step (u[1], over the slope: it has no soft edge) and the two scrolls.
 * A virtual strip under 3 px sees every stripe / ribbon pass as a whole-strip change: they scroll at most as many a second
 * as 32 px cells (sc, 0.6 a second at 1x) */
static void prep_cells(fx_ctx_t *x, uint32_t de)
{
    const uint32_t step = cell_step(x->id == FX_CANOPY_SHADE ? x->custom1 : x->intensity);
    const uint32_t cap = x->vlen < 3u && step > 2048u, sc = cap ? 2048u : step;
    (void)de;
    x->u[0] = step;
    x->u[1] = (255u << 12) / step;
    if (x->id == FX_STRIPES) {
        x->u[3] = m32_shr(x->et, sc * 21u, 10);
        x->u[2] = x->custom1 * 257u;
    } else if (x->id == FX_BULBS) {
        const uint32_t duty = x->custom1 * 257u;
        x->u[2] = (x->et * 3u) >> 11;
        x->u[3] = duty;
        x->u[4] = 32768u - (duty >> 1);
    } else if (x->id == FX_SILK_WEAVE) {
        const uint32_t sb = (step * 16u + 6u) / 13u;
        x->u[1] = sb;
        x->u[2] = m32_shr(x->et, sc * 17u, 10);            /* (the sines use the low 16 bits; the palette walks on) */
        x->u[3] = m32_shr(x->et, (cap ? sc + (sc >> 2) : sb) * 17u, 10);
    }
}

static FX_NOINL void fx_mosaic(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t step = x->u[0];
    uint32_t ph = VP(x, a) * step, cur = 0u;
    rgb_t c0 = fx_rgb(0, 0, 0), c1 = c0;
    for (; a < b; a++, ph += step) {
        const uint32_t n = ph >> 16, over = (ph & 0xFFFFu) + step;
        rgb_t c;
        if (cur == 0u || n != cur - 1u) {                  /* (cur: the tile held in c0, plus 1) */
            c0 = (cur != 0u && n == cur) ? c1 : mosaic_tile(x, n);
            c1 = mosaic_tile(x, n + 1u);
            cur = n + 1u;
        }
        c = c0;
        if (over > 65536u) {
            c = fx_blend(c0, c1, fx_u8(((over - 65536u) >> 4) * x->u[1] >> 8));
        }
        fx_put(o + a, c);
    }
}

/* Opal Facets: fixed facets (Facet width) in palette colours; on each a broad pearly reflection rolls across the face once every 8.2 s (every facet on
 * its own clock), swelling and receding (a half sine) so it is gone at the facet's edges. Reflection = how white it gets. */
static FX_NOINL void fx_opal(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t step = x->u[0];
    uint32_t ph = VP(x, a) * step, cur = 0xFFFFFFFFu, cen = 0, amp = 0;
    rgb_t col = fx_rgb(0, 0, 0);
    for (; a < b; a++, ph += step) {
        const uint32_t n = ph >> 16, f = ph & 0xFFFFu;
        uint32_t d, r = 0;
        if (n != cur) {
            const uint32_t h = fx_hashf(n, x->seed), p = (x->et + (h & 8191u)) & 8191u;
            col = fx_pal_at(x->pal, h >> 16);
            cen = p << 3;                                  /* the reflection's centre travels the face */
            amp = (uint32_t)fx_sin16(p << 2) >> 7;         /* its strength: a half sine over the cycle */
            amp = (amp * x->custom1) >> 8;
            cur = n;
        }
        d = f > cen ? f - cen : cen - f;
        if (d < 16384u) {
            const uint32_t t = fx_smooth8((16384u - d) >> 6);
            r = (t * amp) >> 8;
        }
        fx_put(o + a, fx_blend(col, NEUTRAL_WHITE, r));
    }
}

/* Silk Weave: two broad ribbons of soft light slide through each other in opposite directions (Ribbon spacing; the second has 13/16 of the first's
 * spacing, so the pattern never repeats exactly); colour 1 and 2 are their tints over the dim ground (colour 3); where they cross they turn pearly
 * (White at crossings). ~16 px/s at 1x. */
static FX_NOINL void fx_silk(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    for (; a < b; a++) {
        const uint32_t v = VP(x, a), pa = v * x->u[0] - x->u[2], pb = v * x->u[1] + x->u[3];
        uint32_t ma = ((uint32_t)(fx_sin16(pa) + 32768)) >> 8, mb = ((uint32_t)(fx_sin16(pb) + 32768)) >> 8;
        const uint32_t cross = (ma * mb >> 8) * x->custom1 >> 8;
        rgb_t c;
        ma = ma * ma >> 8;
        mb = mb * mb >> 8;
        rgb_t ca = x->c[0], cb = x->c[1];
        if (!x->pal_own) {                               /* a palette: each ribbon carries it along, the two half a round apart */
            ca = fx_pal_at(x->pal, pa >> 3);
            cb = fx_pal_at(x->pal, (pb >> 3) + 32768u);
        }
        c = fx_blend(fx_blend(x->c[2], ca, ma), cb, mb);                    /* ribbon 2 over ribbon 1 over the ground */
        fx_put(o + a, fx_blend(c, NEUTRAL_WHITE, cross));
    }
}

/* Canopy Shade: a steady pale wash (colour 1) with leaf-like shadows (colour 2) anchored one to a cell (Leaf spacing): each has its own width and
 * place in the cell, its width breathes (13 s at 1x) and it leans a little (22 s); Depth = how dark. A leaf stays inside its cell. */
static FX_NOINL void fx_canopy(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    const uint32_t step = x->u[0];
    uint32_t ph = VP(x, a) * step, cur = 0xFFFFFFFFu, cen = 0, hw = 1, k = 0, dep = 0;
    for (; a < b; a++, ph += step) {
        const uint32_t n = ph >> 16, f = ph & 0xFFFFu;
        uint32_t d, s = 0;
        if (n != cur) {
            const uint32_t h = fx_hashf(n, x->seed), w0 = 9000u + (h & 8191u);
            hw = w0 + (uint32_t)((int32_t)w0 * fx_sin16(x->et * 5u + (h >> 16)) >> 17);                 /* +- a quarter */
            cen = (uint32_t)(32768 + (int32_t)((h >> 13) & 16383u) - 8192 + (fx_sin16(x->et * 3u + (h >> 4)) * 2000 >> 15));
            k = 0xFFFFFFFFu / hw;
            dep = fx_scale8(x->intensity, 150u + ((h >> 24) & 105u));      /* each leaf has its own depth */
            cur = n;
        }
        d = f > cen ? f - cen : cen - f;
        if (d < hw) {
            const uint32_t t = ((hw - d) * (k >> 8)) >> 16;
            s = fx_scale8(fx_smooth8(t > 255u ? 255u : t), dep);
        }
        /* a palette: the wash along the strip, a shadow down to a quarter */
        fx_put(o + a, x->pal_own ? fx_blend(x->c[0], x->c[1], s) : spread_px(x, VP(x, a), 255u - ((s * 3u) >> 2)));
    }
}

/* Quiet Landings: one to three soft pearls (Pearls) rise and fall in parabolic arcs and touch down without a trail; arcs last 2.0, 4.1 and 2.0 s at 1x,
 * each with its own height (up to Height of the strip, a little different every time), in palette colours; colour 2 is the ground. */
static FX_NOINL void fx_landings(fx_ctx_t *x, rgb_t *o, uint32_t a, uint32_t b)
{
    fx_pearl_t pr[3];
    const uint32_t cnt = 1u + ((x->custom1 * 3u) >> 8), top = (((x->vlen > 1u ? x->vlen - 1u : 1u) << 4) * (x->intensity + 1u)) >> 8;
    uint32_t j;
    for (j = 0; j < cnt; j++) {
        const uint32_t k = j == 1u ? 12u : 11u, tt = x->et + j * 1553u, u = (tt & ((1u << k) - 1u)) << (16u - k);
        const uint32_t h = fx_hashf(tt >> k, x->seed + j), hgt = (top * (170u + (h & 85u))) >> 8;
        pr[j].c = m32_shr((u*(65536u-u))>>14,hgt,16);   /* the arc, in 1/16 px above the ground */
        pr[j].r = 0;
        pr[j].col = fx_pal_at(x->pal, j * 0x5555u);
    }
    pearls(x, o, a, b, pr, cnt);
}

/* ------------------------------------------------------------------------------- the table and the engine */
/* Render is the hot indirect dispatch. Preparation uses direct calls below so
 * small preparations can merge with the frame setup without a second table. */
static const render_fn fx_renderers[FX_COUNT] = {
    fx_fill,                          /* (the stream: fx_begin's id 0, no palette) */
    fx_solid,
    fx_colorloop,                     /* Breathe: its colour from prep_breathe */
    fx_colorloop,
    fx_rainbow,
    fx_palette,
    fx_pride,
    fx_pride,
    fx_sinelon,
    fx_twinklefox,
    fx_colortwinkles,
    fx_colorloop,                     /* Candle: its colour from prep_candle */
    fx_candle_multi,
    fx_aurora,
    fx_blends,
    fx_flow,
    fx_fire,
    fx_gradient,
#if FX_NO_PACIFICA
    fx_solid,                         /* unreachable from admitted UR_DIGITAL_M commands; preserve numeric IDs */
#else
    fx_pacifica,
#endif
    fx_sun,
    fx_sun,
    fx_glitter,
    fx_meteor,
    fx_fairy,
    fx_plasma,
    fx_noise,
    fx_running,
    fx_ripples,
    fx_wipe,
    fx_stripes,
    fx_bulbs,
    fx_lava,
    fx_mosaic,
    fx_silk,
    fx_canopy,
    fx_opal,
    fx_landings,
};

uint32_t fx_state_bytes(uint8_t id, uint32_t n)
{
    return id == FX_FIRE ? (n < FX_FIRE_CELLS ? n : FX_FIRE_CELLS) : 0u;
}

uint8_t fx_default_palette(uint8_t id)
{
    return id < FX_COUNT ? fx_pal_default_tab[id] : 0u;
}

/* Expand once per effect start: raw RGB565 or indexed gradient stops. */
static void pal_expand(rgb_t *pal, uint32_t id)
{
    const uint8_t *p = fx_pal_data;
    uint32_t n, k, a = 0;
    rgb_t ca;
#if FX_V407
    if (id >= PAL_COUNT && id < FXV_PAL_COUNT) {           /* the V407's exact 16-stop palettes (fx_v407/fxv_gen.c) */
        for (k = 0; k < 16u; k++) {
            pal[k] = fx_rgb(fxv_pal_exact[id - PAL_COUNT][k][0], fxv_pal_exact[id - PAL_COUNT][k][1], fxv_pal_exact[id - PAL_COUNT][k][2]);
        }
        return;
    }
#endif
    if (id >= PAL_COUNT) id = 0;
    while (id-- != 0u) {
        n = *p;
        p += n ? 3u * n - 1u : 33u;
    }
    n = *p++;
    if (!n) {
        for (k = 0; k < 16u; k++, p += 2) {
            pal[k] = fx_unpack565((uint32_t)p[0] | (uint32_t)p[1] << 8);
        }
        return;
    }
    ca = fx_unpack565((uint32_t)p[0] | (uint32_t)p[1] << 8);
    p += 2;
    for (k = 1; k < n; k++) {
        const uint32_t b = k == n - 1u ? 15u : *p++;
        const rgb_t cb = fx_unpack565((uint32_t)p[0] | (uint32_t)p[1] << 8);
        uint32_t e;
        p += 2;
        for (e = a; e <= b; e++) {
            pal[e] = fx_blend(ca, cb, ((e - a) * 255u) / (b - a));
        }
        a = b;
        ca = cb;
    }
}

#if FX_V407
void fx_pal_expand(rgb_t *pal, uint32_t id)                /* the V407 layer's palettes (fx_v407/fxv_core.c) */
{
    pal_expand(pal, id);
}
#endif

rgb_t fx_pal_entry(uint32_t id, uint32_t k)
{
    rgb_t pal[16];
    pal_expand(pal, id);
    return pal[k];
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
        pal_expand(x->pal, x->pal_id);
    }
}

void fx_begin(fx_ctx_t *x, const fx_params_t *p, uint16_t n, uint8_t *state)
{
    uint32_t k;
    memcpy(&x->id, &p->id, 4u + sizeof x->c);            /* id, speed, intensity, palette, the colours (fx.h: the same order) */
    memcpy(&x->flags, &p->flags, 3u);                     /* flags, custom1, custom2 */
    if (x->id >= FX_COUNT) {
        x->id = FX_SOLID;
    }
    x->n = n;
    x->vlen = p->vlen != 0u ? p->vlen : (n != 0u ? n : 1u);
    x->vbase = p->vstart;
    x->seed = p->seed;
    x->et_frac = 0;
    memset(&x->t, 0, (size_t)((uint8_t *)&x->pal - (uint8_t *)&x->t));   /* t, t_last, et, u[], state, fire_prev */
    if (x->id == FX_FIRE) {                                /* context-owned, independent of every other segment and crossfade */
        x->u[1] = p->seed | 1u;
    }
    x->state = state;
    if (state != 0) {
        for (k = 0; k < fx_state_bytes(x->id, n); k++) {
            state[k] = 0;
        }
    }
    x->pal_id = fx_pal_resolve(x->id, p->palette);
    x->pal_own = x->pal_id == PAL_DEFAULT;                /* (pal[] then holds palette 0, unused) */
    build_palette(x);
    if (!x->pal_own && fx_default_palette(x->id) == PAL_DEFAULT) {   /* an own-colour effect on a palette: the spread */
        x->u[3] = 255u;                                   /* (only those: Fire keeps u[3] as a cursor from 0) */
        x->u[4] = (2u << 16) / (x->vlen > 1u ? x->vlen - 1u : 1u);
    }
}

void fx_set_colors(fx_ctx_t *x, const rgb_t c[3])
{
    if (x->c != c) memcpy(x->c, c, sizeof x->c);
    if (x->pal_id == PAL_COLORS) {
        build_palette(x);
    }
}

static uint32_t fx_rate256(uint32_t s)                    /* effect ms per host ms, x 256 */
{
    return s == 0u ? 0u : 8u + ((s * 15u) >> 3);
}

void fx_frame_begin(fx_ctx_t *x, uint32_t t_ms)
{
    int32_t dt = (int32_t)(t_ms - x->t_last);
    uint32_t acc,rate;
    uint32_t de;
    if (dt < 0) {                                         /* (the host clock stepped back: stand still) */
        dt = 0;
    }
    rate=fx_rate256(x->speed);
    acc=(uint32_t)dt*rate+x->et_frac;
    de=m32_addshr((uint32_t)dt,rate,x->et_frac,8);
    x->et += de;
    x->et_frac = (uint8_t)acc;
    x->t = x->t + dt < x->t ? 0xFFFFFFFFu : x->t + (uint32_t)dt;      /* (raw effect time: counts up, saturates; no wrap at 49 days) */
    x->t_last = t_ms;
    /* fx_begin / fx_set_colors keep the palette current, including during colour ramps. */
    switch (x->id) {
case FX_BREATHE: prep_breathe(x, de); break;
    case FX_COLORLOOP: prep_colorloop(x, de); break;
    case FX_RAINBOW: case FX_PALETTE: case FX_GLITTER: prep_scroll(x, de); break;
    case FX_COLORWAVES: case FX_PRIDE: prep_pride(x, de); break;
    case FX_SINELON: prep_sinelon(x, de); break;
    case FX_CANDLE_MULTI: if (x->vlen != 1u) break;     /* (a 1-px strip: Candle's flame) */
        /* fall through */
    case FX_CANDLE: prep_candle(x, de); break;
    case FX_AURORA: case FX_BLENDS: case FX_NOISE: case FX_RUNNING: prep_slider(x, de); break;
    case FX_FLOW: prep_flow(x, de); break;
    case FX_FIRE: prep_fire(x, de); break;
    case FX_GRADIENT: prep_gradient(x, de); break;
#if !FX_NO_PACIFICA
    case FX_PACIFICA: prep_pacifica(x, de); break;
#endif
    case FX_SUNRISE: case FX_SUNSET: prep_sun(x, de); break;
    case FX_METEOR: prep_meteor(x, de); break;
    case FX_WIPE: prep_wipe(x, de); break;
    case FX_LAVA_LAMP: prep_lava(x, de); break;
    case FX_STRIPES: case FX_BULBS: case FX_SILK_WEAVE: case FX_MOSAIC_DRIFT: case FX_CANOPY_SHADE: case FX_OPAL_FACETS:
        prep_cells(x, de); break;
    default: break;
    }
}

void fx_frame(fx_ctx_t *x, uint32_t t_ms)
{
    fx_frame_begin(x, t_ms);
    while (fx_frame_step(x)) { }
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
        fx_renderers[x->id](x, (rgb_t *)((uintptr_t)out - from * sizeof(rgb_t)), from, to);
    }
}
