/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Batch B1 "analytic": closed-form effects of (parameters, effect time, pixel). Ids 37, 38, 39, 43, 44,
 * 47, 52, 54, 63, 76, 77, 79, 80, 97, 98, 99, 101, 109, 114; manifest/b1_analytic.json, JS fx/v407/b1_analytic.js.
 * B0 put Kelvin Drift (38) here as the worked example of the effect contract (README.md). */
#include "fxv_gen.h"

/* ------------------------------------------------------------------------------- 38 Kelvin Drift
 * Two slow sines along the strip (1.5 and 2.5 waves over the virtual length, opposite directions) set each pixel's
 * colour temperature around the centre (custom1: 0 = 2000 K .. 255 = 6500 K), intensity = the warm / cool range (255:
 * about +-3400 K), a third sine is the brightness ripple (custom2: 255 = 50 % dips). Natural white: q 255, so RGBCCT
 * strips drive WW / CW, RGBW strips W plus a tint, RGB strips mix it. At 1x a wave takes ~20 s and ~33 s to pass. */
static uint32_t kd_bytes(uint32_t n, uint32_t tier)
{
    (void)n;
    (void)tier;
    return 0;
}

static void kd_init(fxv_ctx_t *x)
{
    (void)x;
}

static void kd_advance(fxv_ctx_t *x)
{
    const uint32_t et = x->et, vl = x->vlen;
    x->u[0] = (et * 13u) >> 2;                           /* phase A, 65536 a turn */
    x->u[1] = 0u - et * 2u;                              /* phase B, the other way */
    x->u[2] = 98304u / vl;                               /* 1.5 turns over the virtual length */
    x->u[3] = 163840u / vl;                              /* 2.5 */
    x->u[4] = et * 5u;                                   /* the ripple */
    x->u[5] = 262144u / vl;                              /* 4 */
}

/* a palette coordinate folded back and forth (period 131072): continuous on cyclic and on clamped palettes */
static uint32_t b1_fold(uint32_t v)
{
    v &= 131071u;
    return v > 65535u ? 131071u - v : v;
}

/* With a palette: the colour temperature becomes a palette position (centre custom1, the waves' swing x2 so the default
 * range covers the whole palette, folded at its ends); the waves, their speeds and the brightness ripple stay. */
static void kd_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const int32_t centre = (int32_t)x->custom1 * 256, range = x->intensity;
    const uint32_t rip = x->custom2;
    uint32_t i, vp = x->vbase + from;
    for (i = 0; i < count; i++, vp++) {
        const int32_t w = fxv_sin16(x->u[0] + vp * x->u[2]) + (fxv_sin16(x->u[1] + vp * x->u[3]) >> 1);
        const uint32_t pos = b1_fold((uint32_t)(centre + ((w * range) >> 7)));
        const uint32_t s = (uint32_t)(fxv_sin16(x->u[4] + vp * x->u[5]) + 32767);
        o[i] = fxv_scale_px(fxv_pal(x, pos), 65535u - ((rip * s) >> 9));
    }
}

static void kd_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    if (!x->pal_own) { kd_render_pal(x, from, count, o); return; }
    const uint32_t centre = x->custom1, range = x->intensity, rip = x->custom2;
    uint32_t i, vp = x->vbase + from;
    for (i = 0; i < count; i++, vp++) {
        const int32_t w = fxv_sin16(x->u[0] + vp * x->u[2]) + (fxv_sin16(x->u[1] + vp * x->u[3]) >> 1);
        const int32_t kk = (int32_t)centre + ((w * (int32_t)range) >> 16);
        const uint32_t k = kk < 0 ? 0u : kk > 255 ? 255u : (uint32_t)kk;
        const uint32_t s = (uint32_t)(fxv_sin16(x->u[4] + vp * x->u[5]) + 32767);
        o[i] = fxv_scale_px(fxv_kelvin16(k), 65535u - ((rip * s) >> 9));
    }
}

const fxv_desc_t fxv_d_kelvin_drift = { kd_bytes, kd_init, kd_advance, 0, kd_render, 0, 0, 0 };

/* B1-local candidates for B0: advected 1-D lattice noise (no second time plane), a rounded compact light kernel,
 * and a fractional long-period clock. No render-order state, no per-pixel divide, no heap. All spatial coordinates
 * below use 65536 per virtual strip; products which intentionally wrap are explicit b1_mul32 calls. */
static uint32_t b1_mul32(uint32_t a, uint32_t b) { return a * b; }
static uint32_t b1_min(uint32_t a, uint32_t b) { return a < b ? a : b; }
static uint32_t b1_dist(uint32_t a, uint32_t b) { return a > b ? a - b : b - a; }
static uint32_t b1_wave(uint32_t p) { return (uint32_t)(fxv_sin16(p) + 32767); }
static uint32_t b1_soft(uint32_t d, uint32_t sharp)
{
    const uint32_t v = 65535u - b1_min(65535u, d * sharp);
    return fxv_smooth16(v);
}
static uint32_t b1_softq8(uint32_t d, uint32_t slope)
{
    return fxv_smooth16(65535u - b1_min(65535u, (d * slope) >> 8));
}
static uint32_t b1_noise(uint32_t seed, uint32_t pos, uint32_t time)
{
    const uint32_t p = pos + time, cell = p >> 16;
    const uint32_t a = fxv_hash32(seed ^ cell) >> 16, b = fxv_hash32(seed ^ (cell + 1u)) >> 16;
    return fxv_lerp16(a, b, fxv_smooth16(p & 65535u));
}
static fxv_px_t b1_rgb(uint32_t r, uint32_t g, uint32_t b)
{
    fxv_px_t c;
    c.r = (uint16_t)(r * 256u); c.g = (uint16_t)(g * 256u); c.b = (uint16_t)(b * 256u);
    c.q = 0; c.k = 0;
    return c;
}
static fxv_px_t b1_white(fxv_px_t c, uint32_t k, uint32_t q)
{
    c.q = (uint8_t)q; c.k = (uint8_t)k;
    return c;
}
static uint32_t b1_pos(const fxv_ctx_t *x, uint32_t i)
{
    return b1_mul32(x->vbase + i, x->u[0]) & 65535u;
}
static void b1_advance(fxv_ctx_t *x)
{
    x->u[0] = 65536u / x->vlen;
    x->u[1] = x->et;
}
/* Q16 phase plus an exact millisecond remainder: a 24-hour period does not lose fractional phase each frame.
 * job[0] is the previous wrapping host stamp; job[1] the phase; job[2] the division remainder; job[3] its period.
 * u[7] counts complete cycles independently, so accents do not reset when a Q16 phase counter rolls over.
 * A tempo/duration change keeps phase, speed zero freezes, and a backwards host stamp contributes no time. */
static void b1_clock(fxv_ctx_t *x, uint32_t period)
{
    uint32_t dt = x->t_last - x->job[0];
    uint64_t a;
    if (dt > 0x7FFFFFFFu || x->speed == 0u) dt = 0;
    if (x->job[3] && x->job[3] != period) x->job[2] = (uint32_t)((uint64_t)x->job[2] * period / x->job[3]);
    a = (uint64_t)dt * 65536u + x->job[2];
    x->job[0] = x->t_last;
    x->job[2] = (uint32_t)(a % period);
    a = a / period + x->job[1];
    x->u[7] += (uint32_t)(a >> 16);
    x->job[1] = (uint32_t)a & 65535u;
    x->job[3] = period;
    x->u[6] = x->job[1];
}
/* the palette coordinate of the last entry: 65535 on a clamped palette, 61440 (entry 15 itself) on a cyclic one, whose
 * top blends back into entry 0 */
static uint32_t b1_pal_top(const fxv_ctx_t *x) { return x->pal_wrap ? 61440u : 65535u; }

/* 37 Daylight Arc. Log-like 1 minute..24 hour duration; daylight follows the visible half of the solar arc.
 * The low broad sky is separate from the soft moving sun; night remains a dim RGB blue. */
static void da_advance(fxv_ctx_t *x)
{
    const uint32_t logp = ((255u - x->speed) * 2687u) / 255u;
    const uint32_t base = 60000u << (logp >> 8);
    b1_advance(x);
    b1_clock(x, b1_min(86400000u, base + (base >> 8) * (logp & 255u)));
    x->u[2] = (x->u[6] + x->custom1 * 256u) & 65535u;
    x->u[3] = x->u[2] >= 16384u && x->u[2] <= 49152u ? b1_wave(x->u[2] - 16384u) : 0u;
    /* sin(phase - quarter) is positive only between dawn and dusk. */
    x->u[3] = x->u[3] > 32767u ? (x->u[3] - 32767u) * 2u : 0u;
    x->u[4] = x->u[2] < 16384u ? 0u : x->u[2] > 49152u ? 65535u : (x->u[2] - 16384u) * 2u;
}
/* With a palette: the dawn-to-noon ramp walks the palette (night and dawn / dusk = its start, noon = its end); the solar
 * arc, the sky / sun brightness and the night level stay, the night takes the palette's start colour at that level. */
static void da_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t day = x->u[3];                        /* 0 .. 65534 (the sine's peak at noon) */
    const fxv_px_t col = fxv_pal(x, (day * b1_pal_top(x)) / 65534u);
    const fxv_px_t night = fxv_scale_px(col, x->custom2 * 25u);
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t d = b1_dist(b1_pos(x, from + i), x->u[4]);
        const uint32_t disk = b1_soft(d, 3u + ((255u - x->intensity) >> 3));
        const uint32_t glow = fxv_scale16(day, 18000u + fxv_scale16(disk, 47535u));
        o[i] = fxv_blend16(night, fxv_scale_px(col, glow), day);
    }
}
static void da_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    if (!x->pal_own) { da_render_pal(x, from, count, o); return; }
    const uint32_t day = x->u[3];
    const fxv_px_t sun = fxv_kelvin16(12u + ((day * 158u) >> 16));
    const fxv_px_t night = fxv_scale_px(b1_rgb(8u, 12u, 35u), x->custom2 * 180u);
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t d = b1_dist(b1_pos(x, from + i), x->u[4]);
        const uint32_t disk = b1_soft(d, 3u + ((255u - x->intensity) >> 3));
        const uint32_t glow = fxv_scale16(day, 18000u + fxv_scale16(disk, 47535u));
        o[i] = fxv_blend16(night, fxv_scale_px(sun, glow), day);
    }
}
const fxv_desc_t fxv_d_daylight_arc = { kd_bytes, kd_init, da_advance, 0, da_render, 0, 0, 0 };

/* 39 Cloud Shadows: independently advected broad and fine cloud layers; the narrow threshold rim gives silver edges. */
/* With a palette: the cloud field picks the colour (clear sky = the palette's start, dense cloud = its end); the two
 * drifting layers, the shadow depth (now a darkening) and the silver edge glow stay. The field's real spread, 9216 ..
 * 56320 (its 2.4 % / 97.6 % points over 2 h x 32 seeds; about what a 120 s window reaches), is stretched over the whole
 * palette: k = ceil(top x 65536 / 47104), so 47104 x k still fits 32 bits and lands on top exactly. */
static void cs_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t k = x->pal_wrap ? 85483u : 91180u;
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i);
        const uint32_t a = b1_noise(x->seed, p * 4u, b1_mul32(x->u[1], 3u));
        const uint32_t b = b1_noise(x->seed ^ 0xAB17u, p * 7u, b1_mul32(x->u[1], 5u));
        const uint32_t field = (a * 3u + b) >> 2;
        const uint32_t cover = b1_min(65535u, field + x->intensity * 160u);
        const uint32_t cloud = fxv_scale16(fxv_smooth16(b1_min(65535u, cover > 30000u ? (cover - 30000u) * 2u : 0u)), x->intensity * 257u);
        const uint32_t rim = fxv_scale16(b1_soft(b1_dist(cover, 38000u), 9u), x->intensity * 257u);
        const uint32_t f = field > 9216u ? b1_min(field - 9216u, 47104u) : 0u;
        const fxv_px_t lit = fxv_pal(x, (f * k) >> 16);
        const fxv_px_t body = fxv_scale_px(lit, 65535u - fxv_scale16(fxv_scale16(cloud, x->custom1 * 257u), x->custom1 * 160u));
        o[i] = fxv_add_px(body, fxv_scale_px(lit, (rim * x->custom2) >> 10));
    }
}
static void cs_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    if (!x->pal_own) { cs_render_pal(x, from, count, o); return; }
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i);
        const uint32_t a = b1_noise(x->seed, p * 4u, b1_mul32(x->u[1], 3u));
        const uint32_t b = b1_noise(x->seed ^ 0xAB17u, p * 7u, b1_mul32(x->u[1], 5u));
        const uint32_t field = (a * 3u + b) >> 2;
        const uint32_t cover = b1_min(65535u, field + x->intensity * 160u);
        const uint32_t cloud = fxv_scale16(fxv_smooth16(b1_min(65535u, cover > 30000u ? (cover - 30000u) * 2u : 0u)), x->intensity * 257u);
        const uint32_t rim = fxv_scale16(b1_soft(b1_dist(cover, 38000u), 9u), x->intensity * 257u);
        const fxv_px_t lit = b1_white(fxv_rgb16(x->c[0]), 105u, 230u);
        const fxv_px_t shade = fxv_scale_px(b1_white(fxv_rgb16(x->c[1]), 220u, 170u), 65535u - x->custom1 * 160u);
        const fxv_px_t body = fxv_blend16(lit, shade, fxv_scale16(cloud, x->custom1 * 257u));
        o[i] = fxv_add_px(body, fxv_scale_px(lit, (rim * x->custom2) >> 10));
    }
}
const fxv_desc_t fxv_d_cloud_shadows = { kd_bytes, kd_init, b1_advance, 0, cs_render, 0, 0, 0 };

/* 43 Night Sky: fixed seeded star positions/magnitudes/temperatures, a still diffuse galactic band, rare slow meteors. */
static void ns_advance(fxv_ctx_t *x)
{
    b1_advance(x);
    x->u[2] = x->et / 24000u;
    x->u[3] = x->et % 24000u;
    x->u[4] = (fxv_rand(x->seed, x->u[2], 7u) & 255u) < x->custom1 ? 1u : 0u;
}
/* With a palette: each star's own colour index picks its palette colour, the sky is the palette along the strip at night
 * level (brighter in the Milky Way band), a meteor gets one palette colour; positions, twinkle and meteors stay. */
static void ns_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    uint32_t i;
    const fxv_px_t meteor = fxv_pal(x, fxv_rand(x->seed, x->u[2], 9u) >> 16);
    for (i = 0; i < count; i++) {
        const uint32_t vp = x->vbase + from + i, p = b1_pos(x, from + i);
        const uint32_t h = fxv_hash32(x->seed ^ vp), tw = 47000u + (b1_wave((x->u[1] >> 2) + (h >> 8)) >> 2);
        const uint32_t milky = fxv_scale16(b1_soft(b1_dist(p, 40000u), 3u), x->custom2 * 20u);
        fxv_px_t c = fxv_scale_px(fxv_pal(x, p), 3000u + milky);
        if ((h & 255u) < ((uint32_t)x->intensity >> 2) || (x->intensity && x->vlen == 1u)) {
            const uint32_t mag = 20000u + ((h >> 16) & 32767u);
            c = fxv_add_px(c, fxv_scale_px(fxv_pal(x, (h >> 24) * 257u), fxv_scale16(mag, tw)));
        }
        if (x->u[4] && x->u[3] < 4000u) {
            const uint32_t head = x->u[3] * 16u;
            const uint32_t tail = p <= head ? b1_soft(head - p, 10u) : b1_soft(p - head, 80u);
            const uint32_t env = b1_min(65535u, b1_min(x->u[3], 4000u - x->u[3]) * 80u);
            c = fxv_add_px(c, fxv_scale_px(meteor, fxv_scale16(tail, env)));
        }
        o[i] = c;
    }
}
static void ns_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    if (!x->pal_own) { ns_render_pal(x, from, count, o); return; }
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t vp = x->vbase + from + i, p = b1_pos(x, from + i);
        const uint32_t h = fxv_hash32(x->seed ^ vp), tw = 47000u + (b1_wave((x->u[1] >> 2) + (h >> 8)) >> 2);
        const uint32_t milky = fxv_scale16(b1_soft(b1_dist(p, 40000u), 3u), x->custom2 * 20u);
        fxv_px_t c = fxv_add_px(fxv_rgb16(x->c[0]), fxv_scale_px(b1_white(b1_rgb(150u, 170u, 220u), 210u, 100u), milky));
        if ((h & 255u) < ((uint32_t)x->intensity >> 2) || (x->intensity && x->vlen == 1u)) {
            const uint32_t mag = 20000u + ((h >> 16) & 32767u);
            c = fxv_add_px(c, fxv_scale_px(fxv_kelvin16(40u + ((h >> 24) * 215u >> 8)), fxv_scale16(mag, tw)));
        }
        if (x->u[4] && x->u[3] < 4000u) {
            const uint32_t head = x->u[3] * 16u;
            const uint32_t tail = p <= head ? b1_soft(head - p, 10u) : b1_soft(p - head, 80u);
            const uint32_t env = b1_min(65535u, b1_min(x->u[3], 4000u - x->u[3]) * 80u);
            c = fxv_add_px(c, fxv_scale_px(fxv_kelvin16(240u), fxv_scale16(tail, env)));
        }
        o[i] = c;
    }
}
const fxv_desc_t fxv_d_night_sky = { kd_bytes, kd_init, ns_advance, 0, ns_render, 0, 0, 0 };

/* 44 Moon Path: the moon never moves; a directional, broken reflection gets shorter/fainter away from it. */
static void mp_advance(fxv_ctx_t *x)
{
    b1_advance(x);
    x->u[2] = 16777216u / (2000u + x->intensity * 230u);
    if (!x->pal_own) {        /* the palette scale u[3] = ceil(top x 65536 / far), far = the farthest strip end's distance */
        const uint32_t moon = x->custom1 * 257u, end = (x->vlen - 1u) * x->u[0];
        uint32_t far = end > moon ? end - moon : 0u;
        if (far < moon) far = moon;
        if (!far) far = 1u;
        x->u[3] = (uint32_t)((((uint64_t)b1_pal_top(x) << 16) + far - 1u) / far);
    }
}
/* With a palette: the distance from the moon walks the palette (moon = its start, the farthest strip end = its last
 * entry, so a central moon spans it too; the shimmer nudges it outward, capped at the last entry); the still moon, the
 * broken reflection and its length stay, the water is that colour at a low level. d <= far, so d x u[3] fits 32 bits
 * and (d x u[3]) >> 16 <= top. */
static void mp_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    uint32_t i;
    const uint32_t moon = x->custom1 * 257u, top = b1_pal_top(x), k = x->u[3];
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i), d = b1_dist(p, moon);
        const uint32_t n = b1_noise(x->seed, p * (8u + (x->custom2 >> 3)), b1_mul32(x->u[1], 12u));
        const uint32_t n2 = b1_noise(x->seed ^ 81u, p * 29u, 0u - b1_mul32(x->u[1], 7u));
        const uint32_t broken = fxv_scale16(n, n2);
        const uint32_t along = (moon < 32768u ? p >= moon : p <= moon) ? 1u : 0u;
        const uint32_t reflected = along ? fxv_scale16(b1_softq8(d, x->u[2]), 10000u + fxv_scale16(broken, 55535u)) : 0u;
        const uint32_t amount = b1_min(65535u, b1_soft(d, 70u) + fxv_scale16(reflected, 55000u));
        o[i] = fxv_scale_px(fxv_pal(x, b1_min(top, ((d * k) >> 16) + (broken >> 3))), 4500u + fxv_scale16(amount, 61035u));
    }
}
static void mp_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    if (!x->pal_own) { mp_render_pal(x, from, count, o); return; }
    uint32_t i;
    const uint32_t moon = x->custom1 * 257u;
    const fxv_px_t light = b1_white(fxv_rgb16(x->c[0]), 225u, 255u);
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i), d = b1_dist(p, moon);
        const uint32_t n = b1_noise(x->seed, p * (8u + (x->custom2 >> 3)), b1_mul32(x->u[1], 12u));
        const uint32_t n2 = b1_noise(x->seed ^ 81u, p * 29u, 0u - b1_mul32(x->u[1], 7u));
        const uint32_t broken = fxv_scale16(n, n2);
        const uint32_t along = (moon < 32768u ? p >= moon : p <= moon) ? 1u : 0u;
        const uint32_t reflected = along ? fxv_scale16(b1_softq8(d, x->u[2]), 10000u + fxv_scale16(broken, 55535u)) : 0u;
        const uint32_t amount = b1_min(65535u, b1_soft(d, 70u) + fxv_scale16(reflected, 55000u));
        o[i] = fxv_blend16(fxv_rgb16(x->c[1]), light, amount);
    }
}
const fxv_desc_t fxv_d_moon_path = { kd_bytes, kd_init, mp_advance, 0, mp_render, 0, 0, 0 };

/* 47 Pool Caustics: light focuses where three evolving surface slopes cancel. Broad depth softens the fine ridges. */
/* With a palette: the broad surface wave picks the colour, the water from the palette's first half (dimmed), the light
 * lines from the matching place in its second half; the moving web, its sharpness and depth stay. */
static void pc_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i) * (2u + (x->custom1 >> 5)), t = x->u[1];
        const uint32_t w0 = b1_wave(p + b1_mul32(t, 5u));
        const uint32_t slopes = w0 + b1_wave(p * 2u - b1_mul32(t, 3u)) + b1_wave(p * 3u + t * 2u);
        const uint32_t d = b1_dist(slopes, 98301u);
        const uint32_t fine = b1_soft(d, 3u + (x->intensity >> 4));
        const uint32_t broad = b1_soft(d, 2u + (x->intensity >> 6));
        const uint32_t line = fxv_lerp16(fine, broad, x->custom2 * 257u);
        o[i] = fxv_blend16(fxv_scale_px(fxv_pal(x, w0 >> 1), 30000u), fxv_pal(x, 32768u + (w0 >> 1)), line);
    }
}
static void pc_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    if (!x->pal_own) { pc_render_pal(x, from, count, o); return; }
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i) * (2u + (x->custom1 >> 5)), t = x->u[1];
        const uint32_t slopes = b1_wave(p + b1_mul32(t, 5u)) + b1_wave(p * 2u - b1_mul32(t, 3u)) + b1_wave(p * 3u + t * 2u);
        const uint32_t d = b1_dist(slopes, 98301u);
        const uint32_t fine = b1_soft(d, 3u + (x->intensity >> 4));
        const uint32_t broad = b1_soft(d, 2u + (x->intensity >> 6));
        const uint32_t line = fxv_lerp16(fine, broad, x->custom2 * 257u);
        o[i] = fxv_blend16(fxv_rgb16(x->c[0]), b1_white(fxv_rgb16(x->c[1]), 240u, 255u), line);
    }
}
const fxv_desc_t fxv_d_pool_caustics = { kd_bytes, kd_init, b1_advance, 0, pc_render, 0, 0, 0 };

/* 52 Incense Smoke: a slow noise field warps a faster, finer ascending field; density thresholds the wisps,
 * and fade height independently limits how far smoke reaches above the source (strip start = bottom). */
static void is_advance(fxv_ctx_t *x)
{
    b1_advance(x);
    x->u[2] = 16777216u / (20000u + x->custom2 * 170u);
}
static void is_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i), t = x->u[1];
        const uint32_t warp = b1_noise(x->seed, p * 3u, 0u - t * 2u);
        const uint32_t field = b1_noise(x->seed ^ 0x51u, p * 12u + ((warp * x->custom1) >> 5), 0u - t * 7u);
        const uint32_t wisp = b1_soft(b1_dist(field, 36000u), 2u + ((255u - x->intensity) >> 5));
        const uint32_t fade = b1_softq8(p, x->u[2]);
        o[i] = fxv_scale_px(fxv_pal(x, (warp + (field >> 1)) & 65535u), fxv_scale16(fxv_scale16(wisp, fade), x->intensity * 257u));
    }
}
const fxv_desc_t fxv_d_incense_smoke = { kd_bytes, kd_init, is_advance, 0, is_render, 0, 0, 0 };

/* 54 Sand Ripples: a long sun face and a short lee face, slowly advected and bent by irregularity. */
/* With a palette: the colour runs along the strip (bent by the slow irregularity), the ripple relief is its brightness
 * (sand 78 %, sun face up to full, lee face down to 35 %, as the default colours); motion and sun height stay. */
static void sr_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i);
        const uint32_t warp = b1_noise(x->seed, p * 3u, x->u[1] >> 3);
        const uint32_t ph = (p * (2u + (x->intensity >> 4)) - (x->u[1] >> 2) + ((warp * x->custom2) >> 8)) & 65535u;
        const uint32_t face = ph < 49152u ? (ph * 21845u) >> 14 : (65535u - ph) * 4u;
        const uint32_t relief = ph < 49152u ? fxv_lerp16(51000u, 65535u, fxv_smooth16(face)) : fxv_lerp16(23000u, 51000u, fxv_smooth16(face));
        o[i] = fxv_scale_px(fxv_pal(x, ((p * 7u) >> 3) + (warp >> 3)), fxv_lerp16(51000u, relief, 65535u - x->custom1 * 170u));
    }
}
static void sr_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    if (!x->pal_own) { sr_render_pal(x, from, count, o); return; }
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i);
        const uint32_t warp = b1_noise(x->seed, p * 3u, x->u[1] >> 3);
        const uint32_t ph = (p * (2u + (x->intensity >> 4)) - (x->u[1] >> 2) + ((warp * x->custom2) >> 8)) & 65535u;
        const uint32_t face = ph < 49152u ? (ph * 21845u) >> 14 : (65535u - ph) * 4u;
        const fxv_px_t sand = fxv_rgb16(x->c[0]);
        const fxv_px_t lit = b1_white(fxv_rgb16(x->c[1]), 40u, 180u);
        const fxv_px_t shade = fxv_rgb16(x->c[2]);
        const fxv_px_t relief = ph < 49152u ? fxv_blend16(sand, lit, fxv_smooth16(face)) : fxv_blend16(shade, sand, fxv_smooth16(face));
        o[i] = fxv_blend16(sand, relief, 65535u - x->custom1 * 170u);
    }
}
const fxv_desc_t fxv_d_sand_ripples = { kd_bytes, kd_init, b1_advance, 0, sr_render, 0, 0, 0 };

/* 63 Marble Veins: three scales of stone distortion; zero crossings of the warped bands give fine branching veins. */
static void mv_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i), t = x->u[1] >> 1;
        const uint32_t a = b1_noise(x->seed, p * 3u, t), b = b1_noise(x->seed ^ 71u, p * 7u, t * 2u);
        const uint32_t c = b1_noise(x->seed ^ 137u, p * 17u, t * 3u);
        const uint32_t stone = (a * 4u + b * 2u + c) * 9362u >> 16;
        const uint32_t ph = p * (2u + (x->intensity >> 5)) + stone * 2u + t;
        const uint32_t vein = b1_soft(b1_dist(b1_wave(ph), 32767u), 3u + (x->custom1 >> 3));
        const uint32_t fork = b1_soft(b1_dist(b1_wave(ph + (b >> 1)), 32767u), 8u + (x->custom1 >> 2));
        const uint32_t glow = fxv_scale16(b1_min(65535u, vein + (fork >> 1)), 16000u + x->custom2 * 194u);
        o[i] = fxv_blend16(fxv_pal(x, stone), b1_white(fxv_rgb16(x->c[0]), 220u, 255u), glow);
    }
}
const fxv_desc_t fxv_d_marble_veins = { kd_bytes, kd_init, b1_advance, 0, mv_render, 0, 0, 0 };

/* 76 Soap Film. A compact 256 x RGB thin-film table (three wavelength responses, no per-pixel interference).
 * Generated from the three wavelength responses in flash: zero arena/initialisation work, even at n=0.
 * Table byte (i,c) = (fxv_sin16(49152 + i * frequency[c]) + 32767) >> 8, frequencies = {430,545,690}.
 * Start of strip is the thicker lower edge.
 * Drainage thins the top toward gold/black. A slow fade at the end of a pop cycle prevents a bright reset flash. */
static const uint8_t sf_table[768] = {
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 2, 1, 2, 4, 2, 4, 6, 3, 6, 9, 5, 8, 13,
    6, 11, 17, 8, 13, 22, 10, 17, 27, 12, 20, 32, 15, 24, 38, 17, 28, 44, 20, 32, 51, 23, 37, 58,
    26, 42, 65, 30, 47, 72, 33, 52, 80, 37, 58, 88, 41, 63, 96, 45, 69, 104, 49, 75, 113, 53, 81, 121,
    57, 88, 130, 62, 94, 138, 66, 101, 147, 71, 107, 155, 76, 114, 163, 81, 120, 171, 86, 127, 179, 91, 134, 187,
    96, 140, 194, 101, 147, 201, 106, 154, 208, 111, 160, 214, 116, 167, 220, 122, 173, 226, 127, 179, 231, 132, 185, 236,
    137, 191, 240, 143, 197, 244, 148, 202, 247, 153, 208, 250, 158, 213, 252, 163, 218, 254, 168, 222, 255, 173, 227, 255,
    178, 231, 255, 183, 234, 255, 188, 238, 254, 192, 241, 252, 197, 244, 250, 201, 247, 247, 206, 249, 244, 210, 251, 240,
    214, 253, 236, 217, 254, 231, 221, 255, 226, 225, 255, 220, 228, 255, 214, 231, 255, 208, 234, 255, 201, 237, 254, 194,
    240, 253, 186, 242, 251, 179, 244, 249, 171, 246, 247, 163, 248, 245, 155, 250, 242, 146, 251, 239, 138, 253, 235, 129,
    254, 232, 121, 254, 228, 113, 255, 223, 104, 255, 219, 96, 255, 214, 88, 255, 209, 80, 255, 204, 72, 255, 198, 65,
    254, 192, 57, 253, 187, 50, 252, 181, 44, 250, 174, 38, 249, 168, 32, 247, 162, 26, 245, 155, 21, 243, 149, 17,
    241, 142, 13, 238, 135, 9, 235, 129, 6, 232, 122, 4, 229, 115, 2, 226, 109, 1, 223, 102, 0, 219, 96, 0,
    215, 89, 0, 211, 83, 1, 207, 77, 2, 203, 71, 4, 199, 65, 7, 194, 59, 10, 190, 53, 13, 185, 48, 17,
    180, 43, 22, 175, 38, 27, 170, 33, 32, 165, 29, 38, 160, 25, 44, 155, 21, 51, 150, 17, 58, 145, 14, 65,
    140, 11, 72, 134, 9, 80, 129, 6, 88, 124, 4, 96, 119, 3, 105, 113, 1, 113, 108, 0, 121, 103, 0, 130,
    98, 0, 138, 93, 0, 147, 88, 0, 155, 83, 1, 163, 78, 2, 171, 73, 3, 179, 68, 5, 187, 64, 7, 194,
    59, 10, 201, 55, 13, 208, 50, 16, 214, 46, 19, 220, 42, 23, 226, 38, 27, 231, 35, 31, 236, 31, 36, 240,
    28, 40, 244, 24, 45, 247, 21, 51, 250, 19, 56, 252, 16, 62, 254, 13, 68, 255, 11, 74, 255, 9, 80, 255,
    7, 86, 255, 5, 92, 254, 4, 99, 252, 3, 105, 250, 2, 112, 247, 1, 119, 244, 0, 125, 240, 0, 132, 236,
    0, 139, 231, 0, 145, 226, 0, 152, 220, 0, 159, 214, 1, 165, 208, 2, 171, 201, 3, 178, 194, 4, 184, 186,
    6, 190, 179, 7, 195, 171, 9, 201, 163, 12, 206, 155, 14, 211, 146, 16, 216, 138, 19, 221, 129, 22, 225, 121,
    25, 230, 112, 28, 234, 104, 32, 237, 96, 35, 240, 88, 39, 243, 80, 43, 246, 72, 47, 248, 64, 51, 250, 57,
    55, 252, 50, 60, 254, 44, 64, 255, 38, 69, 255, 32, 74, 255, 26, 79, 255, 21, 84, 255, 17, 88, 254, 13,
    94, 253, 9, 99, 252, 6, 104, 250, 4, 109, 248, 2, 114, 245, 1, 120, 243, 0, 125, 240, 0, 130, 236, 0,
    135, 233, 1, 141, 229, 2, 146, 224, 4, 151, 220, 7, 156, 215, 10, 161, 210, 13, 166, 205, 17, 171, 200, 22,
    176, 194, 27, 181, 188, 32, 186, 182, 38, 191, 176, 44, 195, 170, 51, 199, 163, 58, 204, 157, 65, 208, 150, 73,
    212, 144, 80, 216, 137, 88, 220, 130, 96, 223, 124, 105, 227, 117, 113, 230, 110, 122, 233, 104, 130, 236, 97, 138,
    239, 91, 147, 241, 84, 155, 243, 78, 163, 246, 72, 171, 247, 66, 179, 249, 60, 187, 251, 55, 194, 252, 49, 201,
    253, 44, 208, 254, 39, 214, 255, 35, 220, 255, 30, 226, 255, 26, 231, 255, 22, 236, 255, 18, 240, 255, 15, 244,
    254, 12, 247, 253, 9, 250, 252, 7, 252, 251, 5, 254, 250, 3, 255, 248, 2, 255, 246, 1, 255, 244, 0, 255,
    242, 0, 254, 239, 0, 252, 236, 0, 250, 234, 1, 247, 231, 2, 244, 227, 3, 240, 224, 5, 236, 220, 7, 231,
    217, 9, 226, 213, 12, 220, 209, 15, 214, 205, 18, 207, 200, 22, 201, 196, 26, 194, 192, 30, 186, 187, 35, 178,
};
static void sf_advance(fxv_ctx_t *x)
{
    b1_advance(x);
    x->u[2] = x->custom2 ? ((x->et % (90000u - x->custom2 * 250u)) * 32768u / (90000u - x->custom2 * 250u)) * 2u : 0u;
    x->u[3] = x->custom2 ? b1_min(65535u, b1_min(x->u[2], 65535u - x->u[2]) * 12u) : 65535u;
}
/* With a palette: the film thickness walks the palette instead of the interference table (folded, 1-2 passes over the
 * film; the drained top = its start, dark where the film is thinnest); swirl, drainage, sheen and the pop stay. */
static void sf_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i);
        const uint32_t curl = b1_noise(x->seed, p * 4u, x->u[1] * 3u);
        const uint32_t drain = fxv_scale16(p, b1_min(65535u, x->custom1 * 220u + (x->u[2] >> 2)));
        const uint32_t thick = 9000u + x->intensity * 170u + (curl >> 2);
        const uint32_t field = thick > drain ? thick - drain : 0u;
        const fxv_px_t col = fxv_pal(x, b1_fold(b1_min(65535u, field) * 3u));
        const fxv_px_t film = fxv_scale_px(col, b1_min(65535u, field * 8u));
        const uint32_t sheen = fxv_scale16(b1_soft(b1_dist(curl, 32767u), 12u), b1_min(65535u, field * 8u));
        o[i] = fxv_scale_px(fxv_add_px(film, fxv_scale_px(col, sheen >> 4)), x->u[3]);
    }
}
static void sf_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    if (!x->pal_own) { sf_render_pal(x, from, count, o); return; }
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i);
        const uint32_t curl = b1_noise(x->seed, p * 4u, x->u[1] * 3u);
        const uint32_t drain = fxv_scale16(p, b1_min(65535u, x->custom1 * 220u + (x->u[2] >> 2)));
        const uint32_t thick = 9000u + x->intensity * 170u + (curl >> 2);
        const uint32_t field = thick > drain ? thick - drain : 0u;
        const uint32_t at = b1_min(65535u, field), j = at >> 8, j2 = j < 255u ? j + 1u : 255u;
        const fxv_px_t a = b1_rgb(sf_table[j * 3u], sf_table[j * 3u + 1u], sf_table[j * 3u + 2u]);
        const fxv_px_t b = b1_rgb(sf_table[j2 * 3u], sf_table[j2 * 3u + 1u], sf_table[j2 * 3u + 2u]);
        const fxv_px_t iridescent = fxv_blend16(a, b, (at & 255u) * 257u);
        const fxv_px_t gold = fxv_scale_px(b1_rgb(255u, 170u, 30u), b1_min(65535u, field * 8u));
        const fxv_px_t film = fxv_blend16(gold, iridescent, fxv_smooth16(b1_min(65535u, field > 6000u ? (field - 6000u) * 6u : 0u)));
        const uint32_t sheen = fxv_scale16(b1_soft(b1_dist(curl, 32767u), 12u), b1_min(65535u, field * 8u));
        o[i] = fxv_scale_px(fxv_add_px(film, fxv_scale_px(fxv_kelvin16(210u), sheen >> 4)), x->u[3]);
    }
}
const fxv_desc_t fxv_d_soap_film = { kd_bytes, kd_init, sf_advance, 0, sf_render, 0, 0, 0 };

/* 77 Pendulum Wave: integer-related frequencies guarantee realignment. Colour mode paints a row of phases;
 * dot mode keeps one bob inside each pendulum's cell, with swing width independent of pendulum count. */
static void pw_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    uint32_t i;
    const uint32_t pend = 2u + (x->intensity >> 4), phase = x->u[1] >> 3;
    for (i = 0; i < count; i++) {
        const uint32_t cell = b1_pos(x, from + i) * pend, index = cell >> 16, local = cell & 65535u;
        const uint32_t swing = b1_wave(b1_mul32(phase, 24u + index));
        const uint32_t bob = 32768u - x->custom2 * 100u + ((swing * x->custom2 * 200u) >> 16);
        const uint32_t amount = x->custom1 < 128u ? 12000u + fxv_scale16(swing, 53535u) : b1_soft(b1_dist(local, bob), 12u);
        const fxv_px_t colour = fxv_pal(x, (index * 9000u + (swing >> 2)) & 65535u);
        const fxv_px_t bright = fxv_blend16(colour, fxv_kelvin16(140u), fxv_scale16(swing, 4500u));
        o[i] = fxv_blend16(fxv_rgb16(x->c[1]), bright, amount);
    }
}
const fxv_desc_t fxv_d_pendulum_wave = { kd_bytes, kd_init, b1_advance, 0, pw_render, 0, 0, 0 };

/* 79 Interference: equal-frequency counterpropagating sources, a separately controllable relative phase drift. */
/* With a palette: each source's ripples carry the palette outward (a crest's travelling phase, 4 crests from start to
 * end, folded), the two add where they meet; wavelength, spacing, drift and the standing bands stay. */
static void in_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    uint32_t i;
    const uint32_t left = 32768u - x->custom1 * 120u, right = 32768u + x->custom1 * 120u;
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i), wavelength = 2u + (x->intensity >> 4);
        const uint32_t pa = b1_dist(p, left) * wavelength - x->u[1] * 9u;
        const uint32_t pb = b1_dist(p, right) * wavelength - x->u[1] * 9u + (b1_mul32(x->u[1], x->custom2) >> 6);
        const fxv_px_t a = fxv_scale_px(fxv_pal(x, b1_fold(pa >> 2)), b1_wave(pa) >> 1);
        const fxv_px_t b = fxv_scale_px(fxv_pal(x, b1_fold(pb >> 2)), b1_wave(pb) >> 1);
        o[i] = fxv_add_px(a, b);
    }
}
static void in_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    if (!x->pal_own) { in_render_pal(x, from, count, o); return; }
    uint32_t i;
    const uint32_t left = 32768u - x->custom1 * 120u, right = 32768u + x->custom1 * 120u;
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i), wavelength = 2u + (x->intensity >> 4);
        const uint32_t wa = b1_wave(b1_dist(p, left) * wavelength - x->u[1] * 9u);
        const uint32_t wb = b1_wave(b1_dist(p, right) * wavelength - x->u[1] * 9u + (b1_mul32(x->u[1], x->custom2) >> 6));
        const uint32_t crest = fxv_scale16(wa, wb);
        const fxv_px_t a = fxv_scale_px(fxv_rgb16(x->c[0]), wa >> 1), b = fxv_scale_px(fxv_rgb16(x->c[1]), wb >> 1);
        o[i] = fxv_blend16(fxv_add_px(a, b), fxv_kelvin16(140u), crest >> 3);
    }
}
const fxv_desc_t fxv_d_interference = { kd_bytes, kd_init, b1_advance, 0, in_render, 0, 0, 0 };

/* Seven spectral anchors for 80 Prism, interpolated once across the fan rather than seven splats per pixel. */
static fxv_px_t b1_spectrum(uint32_t p)
{
    const uint32_t u = p * 6u, j = u >> 16, f = u & 65535u;
    fxv_px_t c = b1_rgb(0u, 0u, 0u);
    c.q = 0; c.k = 0;
    switch (j) {
    case 0: c.r = 65280u; c.g = (uint16_t)((30720u * f) >> 16); c.b = 0; break;
    case 1: c.r = 65280u; c.g = (uint16_t)(30720u + ((30720u * f) >> 16)); c.b = 0; break;
    case 2: c.r = (uint16_t)(65280u - ((65280u * f) >> 16)); c.g = (uint16_t)(61440u + ((3840u * f) >> 16)); c.b = (uint16_t)((7680u * f) >> 16); break;
    case 3: c.r = 0; c.g = (uint16_t)(65280u - ((21760u * f) >> 16)); c.b = (uint16_t)(7680u + ((57600u * f) >> 16)); break;
    case 4: c.r = (uint16_t)((7680u * f) >> 16); c.g = (uint16_t)(43520u - ((43520u * f) >> 16)); c.b = 65280u; break;
    default: c.r = (uint16_t)(7680u + ((38400u * f) >> 16)); c.g = 0; c.b = 65280u; break;
    }
    return c;
}
static void pr_advance(fxv_ctx_t *x)
{
    uint32_t beam;
    b1_advance(x);
    for (beam = 0; beam < 3u; beam++) {
        const uint32_t phase = x->et * 4u + beam * 21845u;
        x->u[2u + beam] = b1_wave(phase);
        x->u[5u + beam] = fxv_scale16(b1_wave(phase * 2u + 16384u), x->intensity * 257u);
        x->job[beam] = 16777216u / (1000u + fxv_scale16(x->u[5u + beam], 22000u));
    }
}
/* With a palette: the palette replaces the spectrum: the background glow is the palette along the strip, a beam's core
 * is the palette's middle colour fanning out over the whole palette (its ends at half width) with the dispersion. */
static void pr_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    uint32_t i, beam;
    const uint32_t beams = 1u + (x->custom1 * 3u >> 8);
    const fxv_px_t core = fxv_pal(x, 32768u);
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i);
        fxv_px_t c = fxv_scale_px(fxv_pal(x, p), x->custom2 * 60u);
        for (beam = 0; beam < beams; beam++) {
            const uint32_t centre = x->u[2u + beam], spread = x->u[5u + beam];
            const uint32_t d = b1_dist(p, centre), sharp = x->job[beam];
            const uint32_t amount = b1_softq8(d, sharp);
            const uint32_t offset = b1_min(32767u, (d * sharp) >> 8);
            const uint32_t spectral = p < centre ? 32768u - offset : 32768u + offset;
            if (!amount) continue;
            c = fxv_add_px(c, fxv_scale_px(fxv_blend16(core, fxv_pal(x, spectral), spread), amount));
        }
        o[i] = c;
    }
}
static void pr_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    if (!x->pal_own) { pr_render_pal(x, from, count, o); return; }
    uint32_t i, beam;
    const uint32_t beams = 1u + (x->custom1 * 3u >> 8);
    const fxv_px_t white = b1_white(fxv_rgb16(x->c[0]), 210u, 255u);
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i);
        fxv_px_t c = fxv_scale_px(b1_spectrum(p), x->custom2 * 60u);
        for (beam = 0; beam < beams; beam++) {
            const uint32_t centre = x->u[2u + beam], spread = x->u[5u + beam];
            const uint32_t d = b1_dist(p, centre), sharp = x->job[beam];
            const uint32_t amount = b1_softq8(d, sharp);
            const uint32_t offset = b1_min(32767u, (d * sharp) >> 9);
            const uint32_t spectral = p < centre ? 32768u - offset : 32768u + offset;
            if (!amount) continue;
            const fxv_px_t fan = fxv_blend16(white, b1_spectrum(spectral), spread);
            c = fxv_add_px(c, fxv_scale_px(fan, amount));
        }
        o[i] = c;
    }
}
const fxv_desc_t fxv_d_prism = { kd_bytes, kd_init, pr_advance, 0, pr_render, 0, 0, 0 };

/* 97 Beat Pump: actual 60..180 BPM in host time, a dip-and-swell rather than a black/white strobe.
 * Accent every custom1 beats (zero disables), a small neutral lift only, no discontinuity when tempo changes. */
static void bp_advance(fxv_ctx_t *x)
{
    const uint32_t bpm = 60u + (x->speed * 120u) / 255u;
    uint32_t pump;
    b1_advance(x);
    b1_clock(x, (60000u + bpm - 1u) / bpm);
    pump = x->u[6] < 6000u ? fxv_smooth16(b1_min(65535u, x->u[6] * 11u)) : 65535u - fxv_smooth16(((x->u[6] - 6000u) * 65535u) / 59535u);
    x->u[2] = 65535u - fxv_scale16(pump, x->intensity * 257u);
    x->u[3] = x->custom1 && x->u[7] % x->custom1 == 0u ? b1_soft(x->u[6], 4u) >> 4 : 0u;
}
static void bp_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i) * (1u + ((255u - x->custom2) >> 4)) + x->u[1] * 3u;
        o[i] = fxv_add_px(fxv_scale_px(fxv_pal(x, p & 65535u), x->u[2]), fxv_scale_px(fxv_kelvin16(180u), x->u[3]));
    }
}
const fxv_desc_t fxv_d_beat_pump = { kd_bytes, kd_init, bp_advance, 0, bp_render, 0, 0, 0 };

/* 98 Beat Bars: bar targets vary on stable beat/object hashes; the analytic falling parabola is gravity,
 * with a short attack and an independently slower held peak. Each peak is the maximum of a bounded nine-beat
 * history, including its hold and fall: independent of display cadence, pause/resume and long jumps.
 * Cache 16 little-endian (level, peak) pairs, one bar per advance slice. */
static uint32_t bb_bytes(uint32_t n, uint32_t tier) { (void)n; (void)tier; return 64u; }
static void b1_put16(fxv_ctx_t *x, uint32_t at, uint32_t value)
{
    x->arena[at] = (uint8_t)value; x->arena[at + 1u] = (uint8_t)(value >> 8);
}
static uint32_t b1_get16(const fxv_ctx_t *x, uint32_t at) { return x->arena[at] + x->arena[at + 1u] * 256u; }
static uint32_t bb_target(const fxv_ctx_t *x, uint32_t bar, uint32_t beat)
{
    const uint32_t h = fxv_rand(x->seed, beat, bar);
    const uint32_t groove = fxv_lerp16(56000u - bar * 1800u, 16000u + (h >> 17), x->custom1 * 257u);
    return fxv_scale16(18000u + (h >> 17), groove);
}
static void bb_advance(fxv_ctx_t *x)
{
    const uint32_t bpm = 60u + (x->speed * 120u) / 255u;
    b1_advance(x);
    b1_clock(x, (60000u + bpm - 1u) / bpm);
    x->u[2] = 2u + (x->intensity * 14u) / 255u;
    x->u[3] = 0;
    x->u[4] = (60000u + bpm - 1u) / bpm;
}
static uint32_t bb_slice(fxv_ctx_t *x, uint32_t quota)
{
    const uint32_t j = x->u[3], period = x->u[4];
    const uint32_t attack = b1_min(65535u, x->u[6] * 8u);
    const uint32_t fall = 65535u - fxv_scale16(x->u[6], x->u[6]);
    const uint32_t level = fxv_scale16(bb_target(x, j, x->u[7]), fxv_scale16(attack, fall));
    uint32_t back, peak = level;
    (void)quota;
    for (back = 0; back < 9u; back++) {
        const uint32_t since = back * period + ((x->u[6] * period) >> 16), peak_time = period >> 3;
        uint32_t age, drop, candidate;
        if (back > x->u[7]) break;
        if (since < peak_time) continue;
        age = since - peak_time;
        drop = age > x->custom2 * 4u ? (age - x->custom2 * 4u) * 30u : 0u;
        candidate = fxv_scale16(bb_target(x, j, x->u[7] - back), 64512u);
        candidate = candidate > drop ? candidate - drop : 0u;
        if (candidate > peak) peak = candidate;
    }
    b1_put16(x, j * 4u, level);
    b1_put16(x, j * 4u + 2u, peak);
    x->u[3] = j + 1u;
    return x->u[3] < 16u;
}
static void bb_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t cell = b1_pos(x, from + i) * x->u[2], j = cell >> 16, local = cell & 65535u;
        const uint32_t level = b1_get16(x, j * 4u), peak = b1_get16(x, j * 4u + 2u);
        const uint32_t body = local < level ? b1_min(65535u, (level - local) * 20u) : 0u;
        const uint32_t dot = peak > 1000u ? b1_soft(b1_dist(local, peak), 55u) : 0u;
        o[i] = fxv_add_px(fxv_scale_px(fxv_pal(x, local), body), fxv_scale_px(fxv_kelvin16(230u), dot));
    }
}
const fxv_desc_t fxv_d_beat_bars = { bb_bytes, kd_init, bb_advance, bb_slice, bb_render, 0, 0, 0 };

/* 99 Disco Ball: one moving hashed grid, every reflection has the same rotation speed and a compact soft footprint. */
static void db_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t grid = b1_pos(x, from + i) * (4u + (x->intensity >> 2)) + x->u[1] * 3u;
        const uint32_t h = fxv_hash32(x->seed ^ (grid >> 16));
        const uint32_t spot = b1_soft(b1_dist(grid & 65535u, 32768u), 2u + ((255u - x->custom1) >> 4));
        const fxv_px_t white = b1_white(fxv_rgb16(x->c[0]), 230u, 255u);
        const fxv_px_t colour = fxv_blend16(white, fxv_pal(x, h >> 16), x->custom2 * 257u);
        o[i] = fxv_blend16(fxv_rgb16(x->c[1]), colour, spot);
    }
}
const fxv_desc_t fxv_d_disco_ball = { kd_bytes, kd_init, b1_advance, 0, db_render, 0, 0, 0 };

/* 101 Energy Blade: ignition/hold/retraction, a restrained continuous hum, and an isolated slow clash envelope. */
static void eb_advance(fxv_ctx_t *x)
{
    const uint32_t period = 12000u + x->custom2 * 100u, age = x->et % period;
    b1_advance(x);
    x->u[2] = x->custom2 == 255u ? 65535u : age < 3000u ? age * 21u : age > period - 3000u ? (period - age) * 21u : 65535u;
    x->u[3] = x->et / 8000u;
    x->u[4] = x->et % 8000u;
    x->u[5] = b1_noise(x->seed, 0u, x->et * 50u);
}
/* With a palette: the blade is the palette from hilt (its start) to tip (its end), a clash flares in one palette colour
 * of its own; ignition / retraction, the hum and the clash timing stay. */
static void eb_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    uint32_t i;
    const uint32_t h = fxv_rand(x->seed, x->u[3], 0u);
    const uint32_t event = x->u[4] < 1200u && (h & 255u) < x->custom1 ? b1_soft(b1_dist(x->u[4], 600u), 100u) : 0u;
    const uint32_t hum = 65535u - fxv_scale16(x->u[5], x->intensity * 70u);
    const fxv_px_t flare = fxv_pal(x, fxv_hash32(h) >> 16);
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i), len = x->u[2];
        const uint32_t lit = p < len ? b1_min(65535u, (len - p) * 50u) : 0u;
        const uint32_t clash = fxv_scale16(event, b1_soft(b1_dist(p, (h >> 16)), 15u));
        const fxv_px_t body = fxv_scale_px(fxv_pal(x, p), fxv_scale16(lit, hum));
        o[i] = fxv_blend16(body, flare, fxv_scale16(clash, lit));
    }
}
static void eb_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    if (!x->pal_own) { eb_render_pal(x, from, count, o); return; }
    uint32_t i;
    const uint32_t h = fxv_rand(x->seed, x->u[3], 0u);
    const uint32_t event = x->u[4] < 1200u && (h & 255u) < x->custom1 ? b1_soft(b1_dist(x->u[4], 600u), 100u) : 0u;
    const uint32_t hum = 65535u - fxv_scale16(x->u[5], x->intensity * 70u);
    const fxv_px_t blade = fxv_blend16(fxv_rgb16(x->c[0]), fxv_kelvin16(220u), 11000u);
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i), len = x->u[2];
        const uint32_t lit = p < len ? b1_min(65535u, (len - p) * 50u) : 0u;
        const uint32_t clash = fxv_scale16(event, b1_soft(b1_dist(p, (h >> 16)), 15u));
        const fxv_px_t body = fxv_scale_px(blade, fxv_scale16(lit, hum));
        o[i] = fxv_blend16(body, b1_white(fxv_rgb16(x->c[1]), 200u, 255u), fxv_scale16(clash, lit));
    }
}
const fxv_desc_t fxv_d_energy_blade = { kd_bytes, kd_init, eb_advance, 0, eb_render, 0, 0, 0 };

/* 109 Tinsel Garland: stationary baubles, a moving curved specular highlight and seeded foil under a travelling lamp. */
static void tg_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    uint32_t i;
    const uint32_t lamp = b1_wave(x->u[1]), shift = b1_wave(x->u[1] * 2u);
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i), cell = p * (2u + (x->intensity >> 4)), index = cell >> 16;
        const uint32_t local = cell & 65535u, h = fxv_hash32(x->seed ^ (x->vbase + from + i));
        const uint32_t bauble = b1_soft(b1_dist(local, 32768u), 2u + ((255u - x->custom2) >> 4));
        const uint32_t spec = fxv_scale16(bauble, b1_soft(b1_dist(local, 20000u + (shift >> 2)), 16u));
        const uint32_t glint = (h & 255u) < x->custom1 ? fxv_scale16(b1_soft(b1_dist(p, lamp), 7u), b1_soft(b1_dist(b1_wave(x->u[1] * 3u + (h >> 16)), 32767u), 12u)) : 0u;
        const fxv_px_t tinsel = fxv_scale_px(b1_white(fxv_rgb16(x->c[0]), 65u, 50u), 22000u);
        const fxv_px_t ball = fxv_blend16(fxv_rgb16(x->c[1]), fxv_pal(x, index * 11000u), 14000u);
        const fxv_px_t body = fxv_blend16(tinsel, ball, bauble);
        o[i] = fxv_add_px(body, fxv_scale_px(fxv_kelvin16(60u), b1_min(65535u, glint + spec)));
    }
}
const fxv_desc_t fxv_d_tinsel_garland = { kd_bytes, kd_init, b1_advance, 0, tg_render, 0, 0, 0 };

/* 114 Heartbeat: a rounded lub/dub pair with propagation from the chosen origin and an independently fading spread. */
static void hb_advance(fxv_ctx_t *x)
{
    const uint32_t bpm = 40u + (x->speed * 80u) / 255u;
    b1_advance(x);
    b1_clock(x, (60000u + bpm - 1u) / bpm);
}
/* With a palette: the distance from the origin walks the palette (origin = its start, the pulse's reach = its end); the
 * lub-dub timing, propagation and spread stay, rest is that colour at the default rest level (~12 %). */
static void hb_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i), centre = b1_min(65535u, b1_dist(p, 32768u) * 2u);
        const uint32_t ends = b1_min(p, 65535u - p) * 2u;
        const uint32_t d = x->custom2 <= 128u ? fxv_lerp16(centre, p, x->custom2 * 511u) : fxv_lerp16(p, ends, (x->custom2 - 128u) * 516u);
        const uint32_t phase = (x->u[6] - (d >> 3)) & 65535u;
        const uint32_t lub = b1_soft(b1_dist(phase, 9000u), 8u);
        const uint32_t dub = fxv_scale16(b1_soft(b1_dist(phase, 22500u), 10u), x->custom1 * 257u);
        const uint32_t reach = b1_soft(d, 1u + ((255u - x->intensity) >> 6));
        const uint32_t beat = fxv_scale16(b1_min(65535u, lub + dub), reach);
        o[i] = fxv_scale_px(fxv_pal(x, d), 7700u + fxv_scale16(beat, 57835u));
    }
}
static void hb_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    if (!x->pal_own) { hb_render_pal(x, from, count, o); return; }
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t p = b1_pos(x, from + i), centre = b1_min(65535u, b1_dist(p, 32768u) * 2u);
        const uint32_t ends = b1_min(p, 65535u - p) * 2u;
        const uint32_t d = x->custom2 <= 128u ? fxv_lerp16(centre, p, x->custom2 * 511u) : fxv_lerp16(p, ends, (x->custom2 - 128u) * 516u);
        const uint32_t phase = (x->u[6] - (d >> 3)) & 65535u;
        const uint32_t lub = b1_soft(b1_dist(phase, 9000u), 8u);
        const uint32_t dub = fxv_scale16(b1_soft(b1_dist(phase, 22500u), 10u), x->custom1 * 257u);
        const uint32_t reach = b1_soft(d, 1u + ((255u - x->intensity) >> 6));
        const uint32_t beat = fxv_scale16(b1_min(65535u, lub + dub), reach);
        o[i] = fxv_blend16(fxv_rgb16(x->c[1]), fxv_rgb16(x->c[0]), beat);
    }
}
const fxv_desc_t fxv_d_heartbeat = { kd_bytes, kd_init, hb_advance, 0, hb_render, 0, 0, 0 };
