/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Batch B4 "mechanics": springs, strings, waves and swinging things. Ids 55 Wheat Field, 61 Wind
 * Chimes, 62 Plucked String, 68 Linen Pleats, 78 Newton Cradle, 81 Sonar Ping, 86 Rope Pulse, 87 Soliton Courier,
 * 92 Echo Vault; manifest/b4_mechanics.json, JS fx/v407/b4_mechanics.js (same names, same bytes).
 *
 * State lives in the arena: a 1 KiB header of uint32 words (b4_h) and, for the wave effects, three int16 planes of n
 * cells (prev / cur / next, rotated). Nothing uses the particle slab. Analytic effects (55, 61, 78, 81, 87) write a
 * per-frame object table into the header in advance_begin and read it in render16; the simulated ones (62, 86, 92 wave
 * fields, 68 a spring chain) run fixed simulation ticks in bounded slices (fxv_tick_next) and interpolate at render.
 * B4-local helpers that are candidates for B0: b4_bump (a smooth bump of a Q16 offset with a precomputed inverse
 * width), b4_period (the labelled log duration of a speed slider, as B1 / B2 use), the wave job (inject + step +
 * rotate in slices, mixed end conditions, partial internal walls), the per-chunk blip prefilter (b4_blips), the
 * real-time lane clock (b4_clock: frozen at speed 0, a labelled period). */
#include "fxv_gen.h"
#include "../math32.h"

#define B4_HDR        1024u              /* header words: 256 */
#define B4_WAVE_SLICE 96u                /* wave cells a slice steps (~4,000 cycles) */
#define B4_INJ_SLICE  40u                /* cells a slice injects (two bumps a cell: ~4,200 cycles) */
#define B4_C2         3600u              /* wave coupling Q12: Courant ~0.94 */
#define B4_WHITE      255u               /* cool white (6500 K) for highlights / chrome */
#define B4_WARM       40u                /* warm white (2700 K) for sheen / fold highlights */

static uint32_t *b4_h(const fxv_ctx_t *x) { return (uint32_t *)x->arena; }
static int16_t *b4_plane(const fxv_ctx_t *x, uint32_t k) { return (int16_t *)(x->arena + B4_HDR) + k * x->n; }
static uint32_t b4_min(uint32_t a, uint32_t b) { return a < b ? a : b; }
static uint32_t b4_max(uint32_t a, uint32_t b) { return a > b ? a : b; }
static uint32_t b4_abs(int32_t v) { return (uint32_t)(v < 0 ? -v : v); }
static uint32_t b4_sat(uint32_t v) { return v > 65535u ? 65535u : v; }
static uint32_t b4_winv(uint32_t w) { return (1u << 24) / b4_max(w, 256u); }     /* 2^24 / a Q16 width */
/* the Q of a virtual position for 55 / 61 / 68 / 81: Q16 under 8,192 px, a bit less a doubling (Q13 at 65,535 px), so
 * positions, fronts and their differences stay well inside int32 on any virtual geometry (no wrap-around) */
static uint32_t b4_vq(uint32_t vlen) { uint32_t q = 16u; while (q > 13u && vlen >= (1u << (29u - q))) q--; return q; }
/* a x b >> s, exact (the 64-bit product): a Q16 product in Q(16 - s) */
static uint32_t b4_qmul(uint32_t a, uint32_t b, uint32_t s) { return s ? m32_shr(a, b, s) : a * b; }
/* a smooth bump (1 at d = 0, 0 at |d| >= w, zero slope at both) of a Q16 offset; winv = b4_winv(w) */
static uint32_t b4_bump(int32_t d, uint32_t w, uint32_t winv)
{
    const uint32_t ad = b4_abs(d);
    if (ad >= w) return 0;
    return fxv_smooth16(65535u - b4_min(65535u, (ad >> 8) * winv));
}
/* the labelled duration of a speed slider: hi ms at speed 0, lo at 255, log interpolation (B1 / B2 convention) */
static uint32_t b4_period(uint32_t lo, uint32_t hi, uint32_t speed)
{
    uint32_t oct = 0, base = lo, q;
    while (base * 2u <= hi) { base *= 2u; oct += 256u; }
    oct += ((hi - base) * 256u) / base;
    q = ((255u - speed) * oct) / 255u;
    base = lo << (q >> 8);
    return b4_min(hi, base + ((base * (q & 255u)) >> 8));
}
/* a real-time lane clock in header words h[at] (last t) and h[at + 1] (phase, 65536 a period): advances by the frame's
 * real ms / period, stands still at speed 0 (B1's convention: the label is a real duration), a period change keeps
 * the phase; a jump counts at most 60 s */
static void b4_clock(fxv_ctx_t *x, uint32_t at, uint32_t period)
{
    uint32_t *h = b4_h(x);
    uint32_t dt = x->t_last - h[at];
    if (dt > 60000u) dt = 60000u;
    h[at] = x->t_last;
    if (x->speed) h[at + 1u] += (dt * 65536u) / period;
}
static fxv_px_t b4_white(fxv_px_t c, uint32_t q, uint32_t k) { c.q = (uint8_t)q; c.k = (uint8_t)k; return c; }
/* the displacement between ticks: cur + (cur - prev) x frac (0..65535) */
static int32_t b4_extrap(int32_t cur, int32_t prev, uint32_t frac)
{
    return cur + (((cur - prev) * (int32_t)(frac >> 2)) >> 14);
}
/* a blip table in the header at word t: 4 words a blip: [0] position Q16 (int32), [1] half-width Q16, [2] winv,
 * [3] amp16 | kind << 16. The per-chunk prefilter lists the blips reaching pixels [from, from + count) of the run
 * (positions are virtual: the run starts at vbase; q: the positions' Q) */
static uint32_t b4_blips(const fxv_ctx_t *x, const uint32_t *t, uint32_t nb, uint32_t from, uint32_t count, uint8_t *list,
                         uint32_t q)
{
    const int32_t lo = (int32_t)((x->vbase + from) << q), hi = (int32_t)((x->vbase + from + count) << q);
    uint32_t k, m = 0;
    for (k = 0; k < nb; k++) {
        const int32_t p = (int32_t)t[k * 4u], w = (int32_t)t[k * 4u + 1u];
        if (p + w <= lo || p - w >= hi) continue;
        list[m++] = (uint8_t)k;
    }
    return m;
}
static void b4_blip_set(uint32_t *t, uint32_t k, int32_t pos, uint32_t w, uint32_t amp, uint32_t kind)
{
    t[k * 4u] = (uint32_t)pos;
    t[k * 4u + 1u] = w;
    t[k * 4u + 2u] = b4_winv(w);
    t[k * 4u + 3u] = amp | (kind << 16);
}
/* the frame's tick fraction for interpolation */
static uint32_t b4_frac(const fxv_ctx_t *x)
{
    return x->tick_ms ? b4_sat((x->tick_acc << 16) / x->tick_ms) : 0u;
}
static uint32_t b4_hdr_bytes(uint32_t n, uint32_t tier) { (void)n; (void)tier; return B4_HDR; }
static void b4_init_jobs(fxv_ctx_t *x) { x->job[0] = x->job[1] = x->job[2] = x->job[3] = 0; }
static uint32_t b4_digest_words(const fxv_ctx_t *x, uint32_t words)
{
    const uint32_t *h = b4_h(x);
    uint32_t k, d = 0x4B4u;
    for (k = 0; k < words; k++) d = fxv_hash2(d, h[k]);
    return d;
}

/* ------------------------------------------------------------------------------- 55 Wheat Field
 * Gusts start at the virtual strip's beginning every P ms (gustiness: 8 s .. 1.9 s, with jitter) and roll along it in
 * 3.5 s at 1x. A gust is a smooth bump of half-width W (gust width: 4 .. 30 % of the strip) that bends the stalks into
 * the light (sheen); behind it the stalks sway back and forth (sway: amplitude and 0.4 .. 1.2 Hz) and settle in 2.5 s.
 * Header: 4 gusts x 8 words: [0] front Q16, [1] amp16, [4] active. The sheen is warm white (q by the bend, 2700 K). */
#define WF_GUSTS 4u
static void wf_init(fxv_ctx_t *x) { b4_init_jobs(x); }
static void wf_advance(fxv_ctx_t *x)
{
    uint32_t *h = b4_h(x);
    const uint32_t L = x->vlen, et = x->et, q = b4_vq(L);
    const uint32_t P = 8000u - x->intensity * 24u;
    const uint32_t A = 28000u + x->intensity * 145u;
    const uint32_t Vq = (L << q) / 3500u;
    const uint32_t W = b4_max(1u << q, (L * (2560u + x->custom2 * 66u)) >> (16u - q));
    const uint32_t g0 = et / P;
    uint32_t j;
    x->u[0] = W;
    x->u[1] = b4_winv(W);
    x->u[2] = Vq ? (0xFFFFFFFFu / Vq) >> 8 : 0u;        /* 2^24 / V: (d >> 8) x this >> 16 = ms behind the front */
    x->u[3] = 400u + x->custom1 * 3u;                     /* sway mHz */
    x->u[4] = b4_min(65535u, x->custom1 * 385u);          /* sway amplitude */
    x->u[5] = 65536u / L;                                 /* the ripple's spatial step */
    x->u[6] = et * 2u;                                    /* the ripple's phase */
    for (j = 0; j < WF_GUSTS; j++) {
        const uint32_t g = g0 - j, r = fxv_rand(x->seed, g, 55u);
        const uint32_t tg = g * P + (((r >> 16) * (P >> 2)) >> 16);
        const uint32_t age = et - tg;
        uint32_t *e = h + j * 8u;
        if (age > 6000u) { e[4] = 0; continue; }
        e[0] = age * Vq;
        e[1] = (A * (39322u + (((r & 0xFFFFu) * 26214u) >> 16))) >> 16;
        e[4] = 1;
    }
}

/* the bend of the stalks at virtual pixel vp: > 0 into the light (sheen), < 0 swaying back (shade) */
static int32_t wf_bend(const fxv_ctx_t *x, uint32_t vp)
{
    const uint32_t *h = b4_h(x);
    const uint32_t W = x->u[0], winv = x->u[1], vinv = x->u[2], f = x->u[3], S = x->u[4];
    const uint32_t q = b4_vq(x->vlen);
    const int32_t xq = (int32_t)((vp << q) + (1u << (q - 1u)));
    int32_t b = (fxv_sin16(vp * x->u[5] * 3u - x->u[6]) * 3) >> 5;
    uint32_t j;
    for (j = 0; j < WF_GUSTS; j++) {
        const uint32_t *e = h + j * 8u;
        int32_t d;
        if (!e[4]) continue;
        d = (int32_t)e[0] - xq;
        if (d < -(int32_t)W) continue;
        if (d < (int32_t)W) b += (int32_t)(((b4_bump(d, W, winv) >> 1) * (e[1] >> 1)) >> 14);
        if (d > 0) {
            const uint32_t tau = (((uint32_t)d >> 8) * vinv) >> 16;
            if (tau < 2500u) {
                const uint32_t dec = fxv_smooth16(65535u - tau * 26u);
                const int32_t s = fxv_sin16((tau * f * 67u) >> 10);
                const uint32_t a = b4_min(65535u, fxv_scale16(fxv_scale16(e[1], S), dec) * 3u / 2u);
                b -= (s * (int32_t)(a >> 1)) >> 15;
            }
        }
    }
    return b;
}

/* Palette: position along the field; the field takes the palette's first half (at the field's ~67 %), the sheen the
 * second half at full; gusts, sway, texture and the shade behind a gust stay. */
static void wf_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t ps = 65536u / x->vlen;
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t vp = x->vbase + from + i, u = (vp * ps + (ps >> 1)) >> 1;
        const int32_t b = wf_bend(x, vp);
        const uint32_t tex = 56000u + (fxv_hash32(vp * 0x9E3779B1u + x->seed) >> 20);
        const fxv_px_t field = fxv_scale_px(fxv_pal(x, u), 43700u);
        if (b > 0) {
            o[i] = fxv_blend16(fxv_scale_px(field, tex), fxv_pal(x, u + 32768u), b4_sat((uint32_t)b));
        } else {
            const uint32_t w = b4_sat((uint32_t)(-b));
            o[i] = fxv_scale_px(field, tex - fxv_scale16(tex, w));
        }
    }
}

static void wf_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b4_h(x);
    const uint32_t W = x->u[0], winv = x->u[1], vinv = x->u[2], f = x->u[3], S = x->u[4];
    const fxv_px_t field = b4_white(fxv_rgb16(x->c[0]), 0u, B4_WARM), sheen = b4_white(fxv_rgb16(x->c[1]), 255u, B4_WARM);
    const uint32_t q = b4_vq(x->vlen);
    uint32_t i;
    if (!x->pal_own) { wf_render_pal(x, from, count, o); return; }
    for (i = 0; i < count; i++) {
        const uint32_t vp = x->vbase + from + i;
        const int32_t xq = (int32_t)((vp << q) + (1u << (q - 1u)));
        int32_t b = (fxv_sin16(vp * x->u[5] * 3u - x->u[6]) * 3) >> 5;
        uint32_t j, tex;
        for (j = 0; j < WF_GUSTS; j++) {
            const uint32_t *e = h + j * 8u;
            int32_t d;
            if (!e[4]) continue;
            d = (int32_t)e[0] - xq;
            if (d < -(int32_t)W) continue;
            if (d < (int32_t)W) b += (int32_t)(((b4_bump(d, W, winv) >> 1) * (e[1] >> 1)) >> 14);
            if (d > 0) {
                const uint32_t tau = (((uint32_t)d >> 8) * vinv) >> 16;
                if (tau < 2500u) {
                    const uint32_t dec = fxv_smooth16(65535u - tau * 26u);
                    const int32_t s = fxv_sin16((tau * f * 67u) >> 10);
                    const uint32_t a = b4_min(65535u, fxv_scale16(fxv_scale16(e[1], S), dec) * 3u / 2u);
                    b -= (s * (int32_t)(a >> 1)) >> 15;
                }
            }
        }
        tex = 56000u + (fxv_hash32(vp * 0x9E3779B1u + x->seed) >> 20);
        if (b > 0) {
            o[i] = fxv_blend16(fxv_scale_px(field, tex), sheen, b4_sat((uint32_t)b));
        } else {
            const uint32_t w = b4_sat((uint32_t)(-b));
            o[i] = fxv_scale_px(field, tex - fxv_scale16(tex, w));
        }
    }
}

/* ------------------------------------------------------------------------------- 61 Wind Chimes
 * M chimes (3 .. 12) hang evenly along the virtual strip, each a short tube coloured from the palette (position k / M).
 * A breeze starts every 7 s at 1x (jitter up to half of that) at a random spot, spreading both ways over 2.5 s per
 * strip length; the chimes within its width (15 .. 100 % of the strip) are struck as it reaches them, the far ones
 * softer. A strike: a cool-white flare (250 ms) over a ring envelope (900 ms .. 6 s, ring decay) with a slow hum.
 * Header: chime k at 8 k: [0] pos Q, [1] env16, [2] flare16, [3..5] r g b; breezes at 128 + 4 j: [0] centre Q,
 * [1] start ms, [2] half-width Q, [3] strength16 (Q = b4_vq: Q16 under 8,192 px). */
#define WC_MAX 12u
static void wc_init(fxv_ctx_t *x) { b4_init_jobs(x); }
static void wc_advance(fxv_ctx_t *x)
{
    uint32_t *h = b4_h(x);
    const uint32_t L = x->vlen, et = x->et, M = 3u + (x->custom1 * 9u) / 255u, q = b4_vq(L), s = 16u - q;
    const uint32_t P = 7000u, b0 = et / P;
    const uint32_t Vb = (L << q) / 2500u;
    const uint32_t wb = (L * (9830u + x->custom2 * 218u)) >> s;
    const uint32_t tw = b4_max(39322u >> s, (L << q) / (6u * M));
    uint32_t j;
    x->u[0] = M;
    x->u[1] = tw;
    x->u[2] = b4_winv(tw);
    x->u[3] = tw * 3u;
    x->u[4] = b4_winv(tw * 3u);
    x->u[5] = (M * 65536u) / L;
    x->u[6] = Vb;
    x->u[7] = et;
    for (j = 0; j < 3u; j++) {
        const uint32_t b = b0 - j, r = fxv_rand(x->seed, b, 61u);
        uint32_t *e = h + 128u + j * 4u;
        e[0] = ((r >> 16) * L) >> s;
        e[1] = b * P + (((r & 0xFFFFu) * (P >> 1)) >> 16);
        e[2] = wb;
        e[3] = 32768u + (fxv_hash32(r) >> 17);
    }
    x->job[0] = 0;                                        /* the chimes follow in the slices */
}

#define WC_STEP 3u                   /* chimes a slice (~1,300 cycles each at all-max: all 12 in advance_begin cost ~15,400) */
static uint32_t wc_slice(fxv_ctx_t *x, uint32_t quota)
{
    uint32_t *h = b4_h(x);
    const uint32_t L = x->vlen, et = x->et, M = x->u[0], Vb = x->u[6], q = b4_vq(L);
    const uint32_t B = L << (q - 1u), Bq = B / M, Br = B - Bq * M;     /* chime k at (2 k + 1) B / M, exact */
    const uint32_t P = 7000u, b0 = et / P;
    const uint32_t ring = 6000u - x->intensity * 20u;
    const fxv_px_t white = fxv_kelvin16(B4_WHITE);
    const uint32_t k1 = b4_min(M, x->job[0] + WC_STEP);
    uint32_t j, k;
    (void)quota;
    for (k = x->job[0]; k < k1; k++) {
        uint32_t *c = h + k * 8u;
        const uint32_t pos = (2u * k + 1u) * Bq + ((2u * k + 1u) * Br) / M;
        const fxv_px_t col = fxv_pal(x, (k * 65536u + 32768u) / M);
        uint32_t env = 0, flare = 0;
        for (j = 0; j < 3u; j++) {
            const uint32_t *e = h + 128u + j * 4u;
            const uint32_t dist = b4_abs((int32_t)pos - (int32_t)e[0]);
            uint32_t tau, str, a, hum;
            if (dist > e[2] || Vb == 0u) continue;
            tau = et - (e[1] + dist / Vb);
            if (tau > 0x7FFFFFFFu) continue;
            str = fxv_scale16(e[3], 65535u - m32_muldiv(dist, 32768u, e[2]));
            str = fxv_scale16(str, 45875u + (fxv_rand(x->seed, b0 - j, 61u * 256u + k) >> 18));
            hum = 65536u + ((fxv_sin16(tau * (197u + k * 46u)) * 9830) >> 15);
            a = fxv_scale16(fxv_scale16(fxv_env(tau, 25u, 0u, ring), str), b4_sat(hum));
            if (a > env) env = a;
            a = fxv_scale16(fxv_env(tau, 20u, 30u, 250u), str);
            if (a > flare) flare = a;
        }
        {
            const uint32_t lev = 8000u + fxv_scale16(env, 57535u);
            const fxv_px_t tube = fxv_blend16(fxv_scale_px(b4_white(col, 0u, B4_WHITE), lev), fxv_scale_px(white, lev),
                                              fxv_scale16(flare, 45875u));
            c[0] = pos;
            c[1] = env;
            c[2] = flare;
            c[3] = tube.r;
            c[4] = tube.g;
            c[5] = tube.b;
            c[6] = tube.q;
        }
    }
    x->job[0] = k1;
    return k1 < M;
}

static void wc_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b4_h(x);
    const uint32_t M = x->u[0], tw = x->u[1], twinv = x->u[2], hw = x->u[3], hwinv = x->u[4], kq = x->u[5], Vb = x->u[6], et = x->u[7];
    const fxv_px_t night = fxv_rgb16(x->c[1]), haze = fxv_scale_px(fxv_rgb16(x->c[0]), 3000u);
    const uint32_t q = b4_vq(x->vlen), s = 16u - q, bw = 98304u >> s, bwinv = b4_winv(bw), rmax = 131072u >> s;
    uint32_t bc[3], br[3], nb = 0, i, j;          /* the breezes still spreading: centre, ring radius (once a call) */
    for (j = 0; j < 3u; j++) {
        const uint32_t *e = h + 128u + j * 4u, tau = et - e[1];
        if (tau > 0x7FFFFFFFu || tau * Vb > e[2] + rmax) continue;
        bc[nb] = e[0];
        br[nb++] = tau * Vb;
    }
    for (i = 0; i < count; i++) {
        const uint32_t vp = x->vbase + from + i, k0 = (vp * kq) >> 16;
        const int32_t xq = (int32_t)((vp << q) + (1u << (q - 1u)));
        fxv_px_t out = night;
        uint32_t k;
        for (k = k0 > 0u ? k0 - 1u : 0u; k <= k0 + 1u && k < M; k++) {
            const uint32_t *c = h + k * 8u;
            const int32_t d = (int32_t)c[0] - xq;
            fxv_px_t tube;
            uint32_t w;
            if (b4_abs(d) >= hw) continue;
            tube.r = (uint16_t)c[3]; tube.g = (uint16_t)c[4]; tube.b = (uint16_t)c[5]; tube.q = (uint8_t)c[6]; tube.k = B4_WHITE;
            w = b4_bump(d, tw, twinv);
            if (w) out = fxv_add_px(out, fxv_scale_px(tube, w));
            w = fxv_scale16(fxv_scale16(b4_bump(d, hw, hwinv), c[1]), 16384u);
            if (w) out = fxv_add_px(out, fxv_scale_px(tube, w));
        }
        for (j = 0; j < nb; j++) {
            const uint32_t w = b4_bump((int32_t)(b4_abs((int32_t)bc[j] - xq) - br[j]), bw, bwinv);
            if (w) out = fxv_add_px(out, fxv_scale_px(haze, w));
        }
        o[i] = out;
    }
}

/* ------------------------------------------------------------------------------- the wave job (62, 86, 92)
 * Three int16 planes (prev, cur, next) of n cells, 10 ms ticks. Header: [0] plane rotation, [1] the next event's tick,
 * [2] inject pending, [3] centre Q16, [4] half-width Q16, [5] winv, [6] amp (int32), [7] the bump's shift in prev
 * (Q16: a travelling pulse), [8] events so far, [10..12] wall cells (92). Frame numbers: u[0] tick fraction,
 * u[1] damping Q12, u[2] coupling Q12, u[3] the far end's edge, u[4] wall reflection Q12. The job: for every tick of
 * the frame, inject the pending bump (cells near it, in slices), then step the field in slices, then rotate. */
static uint32_t wv_bytes(uint32_t n, uint32_t tier) { (void)tier; return B4_HDR + 6u * n; }
static int16_t *wv_p(const fxv_ctx_t *x, uint32_t which)
{
    const uint32_t r = b4_h(x)[0] + which;
    return b4_plane(x, r >= 3u ? r - 3u : r);
}

static void wv_walls(fxv_ctx_t *x)   /* 92: three internal walls, symmetric (custom1 0) to irregular (up to n / 8 off) */
{
    uint32_t *h = b4_h(x);
    const uint32_t n = x->n;
    uint32_t k;
    for (k = 0; k < 3u; k++) {
        const uint32_t r = fxv_rand(x->seed, k, 920u);
        const int32_t off = (((int32_t)(r >> 20) - 2048) * (int32_t)(n * x->custom1)) >> 22;
        int32_t w = (int32_t)((n * (k + 1u)) / 4u) + off;
        if (w < 1) w = 1;
        if (w > (int32_t)n - 2) w = (int32_t)n - 2;
        h[10u + k] = (uint32_t)w;
    }
}

static void wv_init(fxv_ctx_t *x)
{
    uint32_t *h = b4_h(x);
    b4_init_jobs(x);
    h[1] = x->tick + (x->id == 86u ? 40u : 30u);
    if (x->id == 92u) wv_walls(x);
}

static void wv_changed(fxv_ctx_t *x)
{
    if (x->id == 92u) wv_walls(x);
}

static void wv_event(fxv_ctx_t *x)   /* the start of a tick: a pluck / flick / strike due? */
{
    uint32_t *h = b4_h(x);
    const uint32_t n = x->n, r = fxv_rand(x->seed, x->tick, x->id);
    uint32_t hw, gap, interval;
    if ((int32_t)(x->tick - h[1]) < 0) return;
    h[8]++;
    if (x->id == 62u) {
        hw = b4_max(131072u, (n * 65536u) / 5u);
        gap = n > (hw >> 15) ? n - (hw >> 15) : 0u;
        h[3] = hw + (r >> 16) * gap;
        h[6] = 4000u + x->intensity * 70u;
        h[7] = 0;
        interval = 60000u / (2u + (x->custom2 * 58u) / 255u);          /* plucks per minute -> ms */
        interval = (interval * (49152u + ((r & 0xFFFFu) >> 1))) >> 16;  /* 0.75 .. 1.25 */
    } else if (x->id == 86u) {
        hw = b4_max(98304u, (n * 65536u) / 14u);
        h[3] = hw + (hw >> 2);
        h[6] = 20000u;
        h[7] = 61400u;                                                  /* the pulse travels: prev holds it 0.94 px back */
        interval = 12000u - x->custom1 * 41u;
        interval = (interval * (52429u + ((r & 0xFFFFu) * 26214u >> 16))) >> 16;   /* 0.8 .. 1.2 */
    } else {
        const uint32_t rate = 8u + ((x->speed * 15u) >> 3);
        hw = b4_max(131072u, (n * 65536u) / 6u);
        gap = n > (hw >> 15) ? n - (hw >> 15) : 0u;
        h[3] = hw + (r >> 16) * gap;
        h[6] = 16000u;
        h[7] = 0;
        interval = (b4_period(3000u, 20000u, x->speed) * rate) >> 8;    /* a real interval in effect ms */
    }
    h[4] = hw;
    h[5] = b4_winv(hw);
    h[2] = 1;
    h[1] = x->tick + b4_max(1u, interval / 10u);
}

static void wv_inject(fxv_ctx_t *x, uint32_t from, uint32_t to)
{
    const uint32_t *h = b4_h(x);
    int16_t *prev = wv_p(x, 0), *cur = wv_p(x, 1);
    const int32_t centre = (int32_t)h[3], shift = (int32_t)h[7], amp = (int32_t)h[6];
    const uint32_t w = h[4], winv = h[5];
    uint32_t i;
    for (i = from; i < to; i++) {
        const int32_t ci = (int32_t)(i * 65536u + 32768u);
        const int32_t bc = (int32_t)(b4_bump(ci - centre, w, winv) >> 1), bp = (int32_t)(b4_bump(ci + shift - centre, w, winv) >> 1);
        cur[i] = (int16_t)fxv_sat16(cur[i] + ((bc * amp) >> 15));
        prev[i] = (int16_t)fxv_sat16(prev[i] + ((bp * amp) >> 15));
    }
}

static void wv_step(fxv_ctx_t *x, uint32_t from, uint32_t to)
{
    const uint32_t *h = b4_h(x);
    const int16_t *prev = wv_p(x, 0), *cur = wv_p(x, 1);
    int16_t *next = wv_p(x, 2);
    const uint32_t n = x->n, damp = x->u[1], c2 = x->u[2], far = x->u[3], R = x->u[4];
    uint32_t k, i;
    if (n == 0u) return;
    /* fxv_wave_step's arithmetic with the neighbours read inline (B4-local, a B0 candidate: 139 -> ~35 cycles a cell):
     * the near end pinned (0 outside), the far end pinned or free (the end cell repeats) */
    for (i = from; i < to; i++) {
        const int32_t c = cur[i], vel = c - prev[i];
        const int32_t l = i == 0u ? 0 : cur[i - 1u];
        const int32_t r = i + 1u < n ? cur[i + 1u] : far == FXV_EDGE_CLAMP ? c : 0;
        next[i] = (int16_t)fxv_sat16(c + vel - ((vel * (int32_t)damp) >> 12) + (((int32_t)c2 * (l - 2 * c + r)) >> 12));
    }
    if (x->id == 92u) {
        for (k = 0; k < 3u; k++) {
            const uint32_t w = h[10u + k];
            if (w >= from && w < to) next[w] = (int16_t)fxv_sat16(next[w] - (((int32_t)R * cur[w]) >> 12));
        }
    }
}

static void wv_advance(fxv_ctx_t *x)
{
    x->u[0] = b4_frac(x);
    x->u[2] = x->id == 62u ? 2400u : B4_C2;             /* the string: Courant 0.77, 77 px / s at 1x */
    x->u[3] = FXV_EDGE_ZERO;
    x->u[4] = 0;
    if (x->id == 62u) {
        x->u[1] = 40960u / (500u + x->custom1 * 37u);                  /* ring time 0.5 .. 10 s */
    } else if (x->id == 86u) {
        x->u[1] = 1u + (x->intensity * 40u) / 255u;
        x->u[3] = x->custom2 >= 128u ? FXV_EDGE_CLAMP : FXV_EDGE_ZERO;
    } else {
        x->u[1] = 1u + (x->custom2 * 40u) / 255u;
        x->u[4] = (x->intensity * 3000u) / 255u;
    }
}

static uint32_t wv_slice(fxv_ctx_t *x, uint32_t quota)
{
    uint32_t *h = b4_h(x);
    const uint32_t n = x->n;
    (void)quota;
    for (;;) {
        if (x->job[0] == 0u) {
            if (!fxv_tick_next(x)) return 0;
            wv_event(x);
            if (h[2]) {
                const uint32_t c = h[3], w = h[4];
                x->job[0] = 1;
                x->job[1] = c > w ? (c - w) >> 16 : 0u;
                x->job[2] = b4_min(n, ((c + w) >> 16) + 2u);
            } else {
                x->job[0] = 2;
                x->job[1] = 0;
            }
        }
        if (x->job[0] == 1u) {
            const uint32_t a = x->job[1], b = b4_min(x->job[2], a + B4_INJ_SLICE);
            if (b > a) wv_inject(x, a, b);
            x->job[1] = b;
            if (b >= x->job[2]) { h[2] = 0; x->job[0] = 2; x->job[1] = 0; }
            return 1;
        }
        {
            const uint32_t a = x->job[1], b = b4_min(n, a + B4_WAVE_SLICE);
            wv_step(x, a, b);
            x->job[1] = b;
            if (b >= n) { h[0] = h[0] + 1u >= 3u ? 0u : h[0] + 1u; x->job[0] = 0; }
            return 1;
        }
    }
}

static uint32_t wv_digest(const fxv_ctx_t *x)
{
    const int16_t *cur = wv_p(x, 1);
    uint32_t k, d = b4_digest_words(x, 16u);
    for (k = 0; k < x->n; k++) d = fxv_hash2(d, (uint32_t)(uint16_t)cur[k]);
    return d;
}

/* 62 Plucked String: plucked at random spots (a bump a tenth of the string wide, pluck strength), the pinned string
 * rings (pitch = the tick rate, ring time) and dies down; colour from the palette by how far a point swings, the dim
 * palette line at rest, a cool-white highlight where it swings fastest. */
static void ps_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const int16_t *prev = wv_p(x, 0), *cur = wv_p(x, 1);
    const uint32_t frac = x->u[0];
    const fxv_px_t white = fxv_kelvin16(B4_WHITE);
    uint32_t i;
    for (i = 0; i < count; i++) {
        const int32_t c = cur[from + i], p = prev[from + i], d = b4_extrap(c, p, frac);
        const uint32_t s = b4_abs(d), v = b4_abs(c - p);
        const uint32_t lev = 5000u + fxv_scale16(b4_sat(s * 5u), 60535u);
        const fxv_px_t col = fxv_scale_px(fxv_pal(x, b4_sat(s * 6u)), lev);
        o[i] = fxv_blend16(col, fxv_scale_px(white, lev), fxv_scale16(b4_sat(v * 12u), 52428u));
    }
}

/* 86 Rope Pulse: a flick at the near end (every 12 .. 1.5 s) sends a bump down the rope; the far end fixed (it comes
 * back flipped: a darker tint of the pulse colour) or free (the same way up). */
/* Palette: position along the rope; the rope is that colour at the rope's ~24 %, an upright bump lights it to full,
 * a flipped bump shows the palette backwards (a dark tint); the wave, its speed and damping stay. */
static void rp_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const int16_t *prev = wv_p(x, 0), *cur = wv_p(x, 1);
    const uint32_t frac = x->u[0], ps = 65536u / x->n;
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t u = (from + i) * ps + (ps >> 1);
        const int32_t d = b4_extrap(cur[from + i], prev[from + i], frac);
        const uint32_t w = b4_sat(b4_abs(d) * 3u);
        const fxv_px_t c = fxv_pal(x, u);
        o[i] = fxv_blend16(fxv_scale_px(c, 15420u), d >= 0 ? c : fxv_scale_px(fxv_pal(x, 65535u - u), 22000u), w);
    }
}

static void rp_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const int16_t *prev = wv_p(x, 0), *cur = wv_p(x, 1);
    const uint32_t frac = x->u[0];
    const fxv_px_t rope = fxv_rgb16(x->c[0]), pulse = fxv_rgb16(x->c[1]), dark = fxv_scale_px(pulse, 22000u);
    uint32_t i;
    if (!x->pal_own) { rp_render_pal(x, from, count, o); return; }
    for (i = 0; i < count; i++) {
        const int32_t d = b4_extrap(cur[from + i], prev[from + i], frac);
        const uint32_t w = b4_sat(b4_abs(d) * 3u);
        o[i] = fxv_blend16(rope, d >= 0 ? pulse : dark, w);
    }
}

/* 92 Echo Vault: a broad soft strike (every 20 .. 3 s, real time) between partially reflecting invisible walls;
 * positive displacement in the strike colour, negative in the echo colour, dark where they cancel. */
/* Palette: position along the vault; positive displacement takes the palette's first half, negative its second half,
 * over a dark space (that place's colour at ~5 %); strikes, walls and echoes stay. */
static void ev_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const int16_t *prev = wv_p(x, 0), *cur = wv_p(x, 1);
    const uint32_t frac = x->u[0], ps = 65536u / x->n;
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t u = (from + i) * ps + (ps >> 1);
        const int32_t d = b4_extrap(cur[from + i], prev[from + i], frac);
        const uint32_t w = b4_sat(b4_abs(d) * 8u);
        o[i] = fxv_add_px(fxv_scale_px(fxv_pal(x, u), 3341u), fxv_scale_px(fxv_pal(x, d >= 0 ? u >> 1 : (u >> 1) + 32768u), w));
    }
}

static void ev_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const int16_t *prev = wv_p(x, 0), *cur = wv_p(x, 1);
    const uint32_t frac = x->u[0];
    const fxv_px_t space = fxv_rgb16(x->c[1]), strike = fxv_rgb16(x->c[0]), echo = fxv_rgb16(x->c[2]);
    uint32_t i;
    if (!x->pal_own) { ev_render_pal(x, from, count, o); return; }
    for (i = 0; i < count; i++) {
        const int32_t d = b4_extrap(cur[from + i], prev[from + i], frac);
        const uint32_t w = b4_sat(b4_abs(d) * 8u);
        o[i] = fxv_add_px(space, fxv_scale_px(d >= 0 ? strike : echo, w));
    }
}

/* ------------------------------------------------------------------------------- 68 Linen Pleats
 * M + 1 fold edges (pleats 4 .. 24) joined by springs over the virtual strip; the first edge is pushed in slowly
 * (compression pace: a 3 min .. 12 s cycle, real time; speed 0 stands still) by up to 30 % of the strip (fold depth),
 * the last one is pinned; the chain follows with lag and never crosses (ordering clamp). 20 ms ticks. Each pleat is
 * shaded by its own compression: fabric, a dark valley in its middle, a warm-white lit fold edge at its ends.
 * Header: [0] M, [1] drive phase, [2] rest length Q16, [3] min gap, [4] drive amplitude Q16, [5] damping Q12,
 * [6] phase step per tick; nodes: pos at 16 + i, vel at 48 + i (int32, i <= 24); the frame's pleat table at
 * 80 + 4 i: [0] start Q16, [1] inverse length, [2] valley16, [3] edge16. */
#define LP_MAX 24u
static void lp_setup(fxv_ctx_t *x)
{
    uint32_t *h = b4_h(x);
    const uint32_t L = x->vlen, M = 4u + (x->custom1 * 20u) / 255u, Lq = L << b4_vq(L);
    uint32_t i;
    h[0] = M;
    h[2] = Lq / M;
    h[3] = h[2] >> 2;
    for (i = 0; i <= M; i++) {
        h[16u + i] = (Lq / M) * i;
        h[48u + i] = 0;
    }
}

static void lp_init(fxv_ctx_t *x)
{
    b4_init_jobs(x);
    lp_setup(x);
}

static void lp_changed(fxv_ctx_t *x)
{
    if (b4_h(x)[0] != 4u + (x->custom1 * 20u) / 255u) lp_setup(x);
}

static void lp_advance(fxv_ctx_t *x)
{
    uint32_t *h = b4_h(x);
    const uint32_t rate = 8u + ((x->speed * 15u) >> 3);
    const uint32_t period = b4_period(12000u, 180000u, x->speed);     /* real ms a cycle */
    h[4] = (x->vlen * (uint32_t)x->intensity * 77u) >> (16u - b4_vq(x->vlen));   /* 30 % of the strip at 255 */
    h[5] = 20u + (x->custom2 * 300u) / 255u;
    h[6] = rate ? 335544320u / (period * rate) : 0u;                   /* 65536 x 20 ms x 256 / (period x rate) */
}

static void lp_tick(fxv_ctx_t *x)
{
    uint32_t *h = b4_h(x);
    int32_t *pos = (int32_t *)(h + 16u), *vel = (int32_t *)(h + 48u);
    const uint32_t M = h[0], damp = h[5], L16 = (uint32_t)x->vlen << b4_vq(x->vlen);
    const int32_t gap = (int32_t)h[3];
    uint32_t i;
    h[1] += h[6];
    pos[0] = 0;
    pos[1] = (int32_t)h[2] + (int32_t)((((uint32_t)(32767 - fxv_sin16(h[1] + 16384u)) >> 4) * (h[4] >> 8)) >> 4);
    pos[M] = (int32_t)L16;
    for (i = 2; i < M; i++) {
        const int32_t lap = pos[i + 1u] - 2 * pos[i] + pos[i - 1u];
        vel[i] += ((lap >> 4) * 180) >> 8;
        vel[i] -= (vel[i] * (int32_t)damp) >> 12;
    }
    for (i = 2; i < M; i++) pos[i] += vel[i];
    for (i = 2; i < M; i++) {
        if (pos[i] < pos[i - 1u] + gap) { pos[i] = pos[i - 1u] + gap; vel[i] = vel[i - 1u]; }
    }
    for (i = M - 1u; i >= 2u; i--) {
        if (pos[i] > pos[i + 1u] - gap) { pos[i] = pos[i + 1u] - gap; vel[i] = vel[i + 1u]; }
    }
}

#define LP_TABLE_STEP 12u            /* pleats of the table a slice (all 24 in one cost ~5,060 cycles) */
static void lp_table(fxv_ctx_t *x, uint32_t a, uint32_t b)   /* the frame's pleat table [a, b), after the ticks */
{
    uint32_t *h = b4_h(x);
    const int32_t *pos = (const int32_t *)(h + 16u);
    const uint32_t r = h[2], depth = x->intensity * 257u;
    uint32_t i;
    for (i = a; i < b; i++) {
        uint32_t *t = h + 80u + i * 4u;
        const uint32_t len = (uint32_t)(pos[i + 1u] - pos[i]), len8 = b4_max(1u, len >> 8);
        const uint32_t comp = b4_min(4u * 65536u, m32_muldiv(r, 65536u, b4_max(len, 1u)));
        const uint32_t e = b4_min(65535u, comp > 65536u ? ((comp - 65536u) * 5u) >> 1 : 0u);
        t[0] = (uint32_t)pos[i];
        t[1] = 0x80000000u / len8;
        t[2] = 22937u + fxv_scale16(depth, 26000u) + fxv_scale16(e, 16598u);
        t[3] = 24000u + fxv_scale16(depth, 24000u) + fxv_scale16(e, 17535u);
    }
}

static uint32_t lp_slice(fxv_ctx_t *x, uint32_t quota)
{
    const uint32_t M = b4_h(x)[0], a = x->job[0];
    (void)quota;
    if (a == 0u && fxv_tick_next(x)) { lp_tick(x); return 1; }
    lp_table(x, a, b4_min(M, a + LP_TABLE_STEP));          /* job[0]: the table's next pleat (0: the ticks first) */
    if (a + LP_TABLE_STEP < M) { x->job[0] = a + LP_TABLE_STEP; return 1; }
    x->job[0] = 0u;
    return 0;
}

/* Palette: the cloth's own coordinate (pleat j and the place inside it), so the palette folds with the fabric; the
 * levels of fabric (~53 %), valley (~14 %) and lit fold edge (~75 %) stay, as do the folds and their motion. */
static void lp_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b4_h(x);
    const uint32_t M = h[0], kM = 65536u / M;
    const uint32_t q = b4_vq(x->vlen);
    uint32_t i, j = 0;
    const int32_t x0 = (int32_t)(((x->vbase + from) << q) + (1u << (q - 1u)));
    while (j + 1u < M && x0 >= (int32_t)h[80u + (j + 1u) * 4u]) j++;
    for (i = 0; i < count; i++) {
        const int32_t xq = x0 + (int32_t)(i << q);
        const uint32_t *t;
        int32_t d;
        uint32_t f, ed, e, v;
        while (j + 1u < M && xq >= (int32_t)h[80u + (j + 1u) * 4u]) j++;
        t = h + 80u + j * 4u;
        d = xq - (int32_t)t[0];
        if (d < 0) { o[i] = fxv_scale_px(fxv_pal(x, 0u), 4234u); continue; }
        f = b4_min(65535u, (((uint32_t)d >> 8) * t[1]) >> 15);
        ed = f < 32768u ? f : 65535u - f;
        e = fxv_scale16(fxv_smooth16(65535u - b4_min(65535u, ed * 8u)), t[3]);
        v = fxv_scale16(fxv_smooth16(65535u - b4_min(65535u, b4_abs((int32_t)f - 32768) * 3u)), t[2]);
        o[i] = fxv_scale_px(fxv_pal(x, j * kM + ((f * kM) >> 16)), fxv_lerp16(fxv_lerp16(34950u, 9250u, v), 48800u, e));
    }
}

static void lp_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b4_h(x);
    const uint32_t M = h[0];
    const fxv_px_t fabric = b4_white(fxv_rgb16(x->c[0]), 0u, B4_WARM), valley = b4_white(fxv_rgb16(x->c[1]), 0u, B4_WARM);
    const fxv_px_t edge = b4_white(fxv_rgb16(x->c[2]), 255u, B4_WARM), shade = fxv_scale_px(valley, 30000u);
    const uint32_t q = b4_vq(x->vlen);
    uint32_t i, j = 0;
    const int32_t x0 = (int32_t)(((x->vbase + from) << q) + (1u << (q - 1u)));
    if (!x->pal_own) { lp_render_pal(x, from, count, o); return; }
    while (j + 1u < M && x0 >= (int32_t)h[80u + (j + 1u) * 4u]) j++;
    for (i = 0; i < count; i++) {
        const int32_t xq = x0 + (int32_t)(i << q);
        const uint32_t *t;
        int32_t d;
        uint32_t f, ed, e, v;
        while (j + 1u < M && xq >= (int32_t)h[80u + (j + 1u) * 4u]) j++;
        t = h + 80u + j * 4u;
        d = xq - (int32_t)t[0];
        if (d < 0) { o[i] = shade; continue; }
        f = b4_min(65535u, (((uint32_t)d >> 8) * t[1]) >> 15);
        ed = f < 32768u ? f : 65535u - f;
        e = fxv_scale16(fxv_smooth16(65535u - b4_min(65535u, ed * 8u)), t[3]);
        v = fxv_scale16(fxv_smooth16(65535u - b4_min(65535u, b4_abs((int32_t)f - 32768) * 3u)), t[2]);
        o[i] = fxv_blend16(fxv_blend16(fabric, valley, v), edge, e);
    }
}

static uint32_t lp_digest(const fxv_ctx_t *x) { return b4_digest_words(x, 80u); }

/* ------------------------------------------------------------------------------- 78 Newton Cradle
 * B balls (3 .. 7) of diameter D (a 40th of the strip, 1.5 .. 8 px) rest in a row at the middle; m of them (balls
 * lifted) swing out to the right (2.5 D), clack back, the left m fly out, and so on: a 1.8 s cycle at 1x. Chrome: the
 * ball colour half cool white. The clack: a one-pixel white blip (clack flash) at the contact point for 80 ms.
 * Header: ball k position Q16 at k; frame numbers: u[0] B, u[1] D / 2 Q16, u[2] its winv, u[3] spec winv,
 * u[4] flash position, u[5] flash amp16, u[6] / u[7] the row's reach. */
static void nc_init(fxv_ctx_t *x) { b4_init_jobs(x); }
static void nc_advance(fxv_ctx_t *x)
{
    uint32_t *h = b4_h(x);
    const uint32_t L = x->vlen, et = x->et, B = 3u + (x->custom1 * 4u) / 255u, m = 1u + (x->intensity * (B - 2u)) / 255u;
    const uint32_t D = b4_min(8u * 65536u, b4_max(98304u, (L * 65536u) / 40u)), T0 = 1800u;
    const uint32_t ph = ((et % T0) * 65536u) / T0, half = ph & 32767u, side = ph >> 15;
    const int32_t A = (int32_t)((D * 5u) >> 1), sw = (fxv_sin16(half) * (A >> 8)) >> 7;
    const int32_t centre = (int32_t)(L * 32768u);
    const uint32_t tau = et % (T0 / 2u);
    uint32_t k;
    for (k = 0; k < B; k++) {
        int32_t p = centre + (int32_t)((2 * (int32_t)k - (int32_t)(B - 1u)) * (int32_t)(D >> 1));
        if (side == 0u && k >= B - m) p += sw;
        if (side == 1u && k < m) p -= sw;
        h[k] = (uint32_t)p;
    }
    x->u[0] = B;
    x->u[1] = D >> 1;
    x->u[2] = b4_winv(D >> 1);
    x->u[3] = b4_winv(D / 5u);
    x->u[4] = side == 0u ? (h[B - m - 1u] + h[B - m]) >> 1 : (h[m - 1u] + h[m]) >> 1;
    x->u[5] = fxv_scale16(fxv_env(tau, 0u, 20u, 60u), x->custom2 * 257u);
    x->u[6] = h[0] - D;
    x->u[7] = h[B - 1u] + D;
}

/* Palette: the row of balls is the palette laid out, ball k its k-th slice, left to right across the ball, and the
 * palette moves on along the row by one ball a swing (1.8 s at 1x; on an open palette it runs there and back), so
 * every ball, the still ones too, shows every colour; the background colour, the white specular highlight and the
 * clack flash stay, as do the swing and the clack. */
static void nc_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b4_h(x);
    const uint32_t B = x->u[0], hw = x->u[1], hwinv = x->u[2], spinv = x->u[3], fl = x->u[5], seg = 65536u / B;
    const uint32_t off = ((x->et % (3600u * B)) * 65536u) / (1800u * B);     /* 0 .. 131071: twice round, or there and back */
    const fxv_px_t bg = fxv_rgb16(x->c[1]), white = fxv_kelvin16(B4_WHITE);
    const int32_t lo = (int32_t)((x->vbase + from) * 65536u), hi = (int32_t)((x->vbase + from + count) * 65536u);
    const int skip = (int32_t)x->u[7] < lo || (int32_t)x->u[6] > hi;
    uint32_t i;
    for (i = 0; i < count; i++) {
        const int32_t xq = (int32_t)((x->vbase + from + i) * 65536u + 32768u);
        fxv_px_t out = bg;
        uint32_t k, w;
        if (!skip) {
            for (k = 0; k < B; k++) {
                const int32_t d = (int32_t)h[k] - xq;
                uint32_t f, u;
                w = b4_bump(d, hw, hwinv);
                if (!w) continue;
                f = b4_min(65535u, ((((uint32_t)((int32_t)hw - d)) >> 8) * hwinv) >> 1);
                u = (k * seg + ((f * seg) >> 16) + off) & 131071u;
                if (x->pal_wrap) u &= 65535u; else if (u > 65535u) u = 131071u - u;
                out = fxv_add_px(out, fxv_scale_px(fxv_pal(x, u), fxv_scale16(w, 52428u)));
                w = b4_bump(d, hw / 5u * 2u, spinv);
                if (w) out = fxv_add_px(out, fxv_scale_px(white, fxv_scale16(w, 39321u)));
            }
        }
        if (fl) {
            w = b4_bump((int32_t)x->u[4] - xq, 65536u, 256u);
            if (w) out = fxv_add_px(out, fxv_scale_px(white, fxv_scale16(w, fl)));
        }
        o[i] = out;
    }
}

static void nc_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b4_h(x);
    const uint32_t B = x->u[0], hw = x->u[1], hwinv = x->u[2], spinv = x->u[3], fl = x->u[5];
    const fxv_px_t bg = fxv_rgb16(x->c[1]), white = fxv_kelvin16(B4_WHITE), chrome = fxv_blend16(fxv_rgb16(x->c[0]), white, 32768u);
    const int32_t lo = (int32_t)((x->vbase + from) * 65536u), hi = (int32_t)((x->vbase + from + count) * 65536u);
    const int skip = (int32_t)x->u[7] < lo || (int32_t)x->u[6] > hi;
    uint32_t i;
    if (!x->pal_own) { nc_render_pal(x, from, count, o); return; }
    for (i = 0; i < count; i++) {
        const int32_t xq = (int32_t)((x->vbase + from + i) * 65536u + 32768u);
        fxv_px_t out = bg;
        uint32_t k, w;
        if (!skip) {
            for (k = 0; k < B; k++) {
                const int32_t d = (int32_t)h[k] - xq;
                w = b4_bump(d, hw, hwinv);
                if (!w) continue;
                out = fxv_add_px(out, fxv_scale_px(chrome, fxv_scale16(w, 52428u)));
                w = b4_bump(d, hw / 5u * 2u, spinv);
                if (w) out = fxv_add_px(out, fxv_scale_px(white, fxv_scale16(w, 39321u)));
            }
        }
        if (fl) {
            w = b4_bump((int32_t)x->u[4] - xq, 65536u, 256u);
            if (w) out = fxv_add_px(out, fxv_scale_px(white, fxv_scale16(w, fl)));
        }
        o[i] = out;
    }
}

/* ------------------------------------------------------------------------------- 81 Sonar Ping
 * Every 5 s at 1x a ping leaves the virtual strip's start and spreads to its range (30 .. 100 % of the strip) in
 * 2.25 s, a soft ring with a fading trail (afterglow 0.15 .. 3.2 s); K hidden targets (1 .. 16, re-placed every fourth
 * ping) flare in the ping colour as it passes and send an echo back to the source in the echo colour, the far ones
 * fainter. Header: the blip table at 0 (targets then echoes, 4 words each); u[0] blips, u[1] front Q, u[2] ring
 * amp16, u[3] trail decay per Q px, u[4] range Q, u[5] the trail's length Q (Q = b4_vq: Q16 under 8,192 px). */
static void sp_init(fxv_ctx_t *x) { b4_init_jobs(x); }
static void sp_advance(fxv_ctx_t *x)
{
    const uint32_t L = x->vlen, et = x->et, P = 5000u, g = et / P, tau = et - g * P;
    const uint32_t Rg = b4_qmul(L, 19660u + x->custom1 * 180u, 16u - b4_vq(L)), V = b4_max(1u, Rg / 2250u);
    const uint32_t Ta = 100u + x->custom2 * 6u, reach = V * Ta;
    x->u[0] = 0;
    x->u[1] = tau <= 2250u ? tau * V : Rg;
    x->u[2] = tau <= 2250u ? 65535u : fxv_env(tau - 2250u, 0u, 0u, 400u);
    x->u[3] = (65535u * 256u) / b4_max(1u, reach >> 8);         /* the trail's age16 per Q px behind the front */
    x->u[4] = Rg;
    x->u[5] = reach;                                             /* Q px behind the front where the trail is out */
    x->job[0] = 0;                                               /* the targets follow in the slices */
}

#define SP_STEP 4u                   /* targets a slice (~300 cycles each with its echo: all 16 in advance_begin reached ~5,100) */
static uint32_t sp_slice(fxv_ctx_t *x, uint32_t quota)
{
    uint32_t *h = b4_h(x);
    const uint32_t L = x->vlen, et = x->et, P = 5000u, g = et / P, tau = et - g * P, s = 16u - b4_vq(L);
    const uint32_t Rg = b4_qmul(L, 19660u + x->custom1 * 180u, s), V = b4_max(1u, Rg / 2250u), K = 1u + (x->intensity * 15u) / 255u;
    const uint32_t k1 = b4_min(K, x->job[0] + SP_STEP);
    uint32_t k, nb = x->u[0];
    (void)quota;
    for (k = x->job[0]; k < k1; k++) {
        const uint32_t r = fxv_rand(x->seed, g >> 2, 81u * 256u + k);
        const uint32_t pos = m32_shr(Rg, 5000u + (((r >> 16) * 60535u) >> 16), 16u);
        const uint32_t tk = pos / V, wk = (65536u + (r & 3u) * 32768u) >> s;
        uint32_t a, e;
        if (tau < tk) continue;
        a = fxv_env(tau - tk, 40u, 60u, 700u);
        if (a) b4_blip_set(h, nb++, (int32_t)pos, wk, a, 0u);
        e = (tau - tk) * V;                                     /* the echo's way back */
        if (e < pos) {
            const uint32_t fade = 65535u - m32_muldiv(pos, 39321u, Rg);
            b4_blip_set(h, nb++, (int32_t)(pos - e), 163840u >> s, fade, 1u);
        }
    }
    x->u[0] = nb;
    x->job[0] = k1;
    return k1 < K;
}

/* Palette: the range from the source (0 at the start, the full range at the ping's reach). The ping, its trail and
 * the targets take the palette's first half, the echoes and the dark sea (~4 %) its second half; pings, targets,
 * echoes and their timing stay. */
static void sp_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b4_h(x);
    const uint32_t nb = x->u[0], front = x->u[1], ramp = x->u[2], decay = x->u[3], Rg = x->u[4], reach = x->u[5];
    const uint32_t q = b4_vq(x->vlen), s = 16u - q, pw = 163840u >> s, pwinv = b4_winv(pw);
    const uint32_t rs = (0xFFFFFFFFu >> s) / Rg;          /* range16 a pixel */
    uint8_t list[32];
    const uint32_t m = b4_blips(x, h, nb, from, count, list, q);
    uint32_t i, j;
    for (i = 0; i < count; i++) {
        const uint32_t vp = x->vbase + from + i, r = b4_min(65535u, vp * rs + (rs >> 1)) >> 1;
        const int32_t xq = (int32_t)((vp << q) + (1u << (q - 1u)));
        const int32_t d = (int32_t)front - xq;
        const fxv_px_t ping = fxv_pal(x, r), echo = fxv_pal(x, r + 32768u);
        fxv_px_t out = fxv_scale_px(echo, 2600u);
        uint32_t w;
        if ((uint32_t)xq < Rg + (196608u >> s)) {
            w = b4_bump(d, pw, pwinv);
            if (w) out = fxv_add_px(out, fxv_scale_px(ping, fxv_scale16(w, ramp)));
            if (d > 0 && (uint32_t)d < reach) {
                const uint32_t age = b4_sat((((uint32_t)d >> 8) * decay) >> 8);
                out = fxv_add_px(out, fxv_scale_px(ping, fxv_scale16(fxv_scale16(65535u - age, 65535u - age), 9000u)));
            }
        }
        for (j = 0; j < m; j++) {
            const uint32_t *t = h + list[j] * 4u;
            w = b4_bump((int32_t)t[0] - xq, t[1], t[2]);
            if (!w) continue;
            w = fxv_scale16(w, t[3] & 0xFFFFu);
            out = fxv_add_px(out, fxv_scale_px((t[3] >> 16) ? echo : ping, w));
        }
        o[i] = out;
    }
}

static void sp_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b4_h(x);
    const uint32_t nb = x->u[0], front = x->u[1], ramp = x->u[2], decay = x->u[3], Rg = x->u[4], reach = x->u[5];
    const fxv_px_t sea = fxv_scale_px(fxv_rgb16(x->c[1]), 2600u), ping = fxv_rgb16(x->c[0]), echo = fxv_rgb16(x->c[1]);
    const uint32_t q = b4_vq(x->vlen), s = 16u - q, pw = 163840u >> s, pwinv = b4_winv(pw);
    uint8_t list[32];
    uint32_t m, i, j;
    if (!x->pal_own) { sp_render_pal(x, from, count, o); return; }
    m = b4_blips(x, h, nb, from, count, list, q);
    for (i = 0; i < count; i++) {
        const int32_t xq = (int32_t)(((x->vbase + from + i) << q) + (1u << (q - 1u)));
        const int32_t d = (int32_t)front - xq;
        fxv_px_t out = sea;
        uint32_t w;
        if ((uint32_t)xq < Rg + (196608u >> s)) {
            w = b4_bump(d, pw, pwinv);
            if (w) out = fxv_add_px(out, fxv_scale_px(ping, fxv_scale16(w, ramp)));
            if (d > 0 && (uint32_t)d < reach) {
                const uint32_t age = b4_sat((((uint32_t)d >> 8) * decay) >> 8);
                out = fxv_add_px(out, fxv_scale_px(ping, fxv_scale16(fxv_scale16(65535u - age, 65535u - age), 9000u)));
            }
        }
        for (j = 0; j < m; j++) {
            const uint32_t *t = h + list[j] * 4u;
            w = b4_bump((int32_t)t[0] - xq, t[1], t[2]);
            if (!w) continue;
            w = fxv_scale16(w, t[3] & 0xFFFFu);
            out = fxv_add_px(out, fxv_scale_px((t[3] >> 16) ? echo : ping, w));
        }
        o[i] = out;
    }
}

/* ------------------------------------------------------------------------------- 87 Soliton Courier
 * Packets A leave the virtual strip's start, packets B its end, one traverse in 20 .. 3 s (real time, frozen at speed
 * 0), spaced 2.2 .. 0.35 traverses apart (collision frequency, with jitter); width 1.5 .. 11.5 px. When an A and a B
 * cross, both jump ahead by up to 0.9 widths (phase shift), eased over the overlap; in the overlap each dims by the
 * other (luminance compression, no white flash). Header: [0] clock t, [1] lane phase; the blip table at 8 (A then B). */
static void sc_init(fxv_ctx_t *x) { b4_init_jobs(x); b4_h(x)[0] = x->t_last; }
static void sc_advance(fxv_ctx_t *x)
{
    uint32_t *h = b4_h(x), *t = h + 8u;
    const uint32_t L = x->vlen, S = 144179u - x->custom1 * 475u, W = 98304u + x->intensity * 2570u;
    const uint32_t Wph = b4_max(1u, (W * 2u) / L), delta = (W * x->custom2 * 9u) / 2550u;
    uint32_t ph, na = 0, nb = 0, ra[6], rb[6], j, m, span;
    b4_clock(x, 0u, b4_period(3000u, 20000u, x->speed));
    ph = h[1];
    span = 65536u + S / 3u + Wph;
    for (j = ph / S; na < 6u; j--) {                          /* lane A: launches j S + jitter, the latest first */
        const uint32_t a = j * S + (((fxv_rand(x->seed, j, 870u) >> 16) * (S / 3u)) >> 16), r = ph - a;
        if (r > 0x7FFFFFFFu) continue;                        /* not launched yet */
        if (r > span) break;
        ra[na++] = r;
    }
    for (j = (ph - S / 2u) / S; nb < 6u; j--) {
        const uint32_t b = j * S + S / 2u + (((fxv_rand(x->seed, j, 871u) >> 16) * (S / 3u)) >> 16), r = ph - b;
        if (r > 0x7FFFFFFFu) continue;
        if (r > span) break;
        rb[nb++] = r;
    }
    for (j = 0; j < na; j++) {                                /* positions with the phase shifts of the crossings */
        uint32_t sh = 0;
        for (m = 0; m < nb; m++) {
            const uint32_t prog = ra[j] + rb[m] + Wph;        /* crossing at ra + rb = 65536 */
            if (prog >= 65536u + 2u * Wph) sh += delta;
            else if (prog > 65536u) sh += fxv_scale16(delta, fxv_smooth16(m32_muldiv(prog - 65536u, 65535u, 2u * Wph)));
        }
        b4_blip_set(t, j, (int32_t)(ra[j] * L + sh), W, 65535u, 0u);
    }
    for (m = 0; m < nb; m++) {
        uint32_t sh = 0;
        for (j = 0; j < na; j++) {
            const uint32_t prog = ra[j] + rb[m] + Wph;
            if (prog >= 65536u + 2u * Wph) sh += delta;
            else if (prog > 65536u) sh += fxv_scale16(delta, fxv_smooth16(m32_muldiv(prog - 65536u, 65535u, 2u * Wph)));
        }
        b4_blip_set(t, na + m, (int32_t)(L * 65536u - rb[m] * L - sh), W, 65535u, 1u);
    }
    x->u[0] = na + nb;
}

/* Palette: position along the strip; packets A take the palette's first half, packets B its second half, so each
 * changes colour as it travels; the space is that place's colour at ~4 %. Travel, crossings and dimming stay. */
static void sc_render_pal(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *t = b4_h(x) + 8u, ps = 65536u / x->vlen;
    uint8_t list[12];
    const uint32_t m = b4_blips(x, t, x->u[0], from, count, list, 16u);
    uint32_t i, j;
    for (i = 0; i < count; i++) {
        const uint32_t vp = x->vbase + from + i, u = vp * ps + (ps >> 1);
        const int32_t xq = (int32_t)(vp * 65536u + 32768u);
        uint32_t wa = 0, wb = 0, w;
        fxv_px_t out = fxv_scale_px(fxv_pal(x, u), 2570u);
        for (j = 0; j < m; j++) {
            const uint32_t *b = t + list[j] * 4u;
            w = b4_bump((int32_t)b[0] - xq, b[1], b[2]);
            w = fxv_scale16(w, w);
            if (b[3] >> 16) wb += w; else wa += w;
        }
        wa = b4_sat(wa);
        wb = b4_sat(wb);
        w = fxv_scale16(wb, 29491u);
        if (wa) out = fxv_add_px(out, fxv_scale_px(fxv_pal(x, u >> 1), wa - fxv_scale16(wa, w)));
        if (wb) out = fxv_add_px(out, fxv_scale_px(fxv_pal(x, (u >> 1) + 32768u), wb - fxv_scale16(wb, fxv_scale16(wa, 29491u))));
        o[i] = out;
    }
}

static void sc_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *t = b4_h(x) + 8u;
    const fxv_px_t space = fxv_rgb16(x->c[1]), ca = fxv_rgb16(x->c[0]), cb = fxv_rgb16(x->c[2]);
    uint8_t list[12];
    uint32_t m, i, j;
    if (!x->pal_own) { sc_render_pal(x, from, count, o); return; }
    m = b4_blips(x, t, x->u[0], from, count, list, 16u);
    for (i = 0; i < count; i++) {
        const int32_t xq = (int32_t)((x->vbase + from + i) * 65536u + 32768u);
        uint32_t wa = 0, wb = 0, w;
        for (j = 0; j < m; j++) {
            const uint32_t *b = t + list[j] * 4u;
            w = b4_bump((int32_t)b[0] - xq, b[1], b[2]);
            w = fxv_scale16(w, w);
            if (b[3] >> 16) wb += w; else wa += w;
        }
        wa = b4_sat(wa);
        wb = b4_sat(wb);
        w = fxv_scale16(wb, 29491u);
        o[i] = fxv_add_px(fxv_add_px(space, fxv_scale_px(ca, wa - fxv_scale16(wa, w))),
                          fxv_scale_px(cb, wb - fxv_scale16(wb, fxv_scale16(wa, 29491u))));
    }
}

/* ------------------------------------------------------------------------------- descriptors */
const fxv_desc_t fxv_d_wheat_field     = { b4_hdr_bytes, wf_init, wf_advance, 0, wf_render, 0, 0, 0 };
const fxv_desc_t fxv_d_wind_chimes     = { b4_hdr_bytes, wc_init, wc_advance, wc_slice, wc_render, 0, 0, 0 };
const fxv_desc_t fxv_d_plucked_string  = { wv_bytes, wv_init, wv_advance, wv_slice, ps_render, 0, wv_changed, wv_digest };
const fxv_desc_t fxv_d_linen_pleats    = { b4_hdr_bytes, lp_init, lp_advance, lp_slice, lp_render, 0, lp_changed, lp_digest };
const fxv_desc_t fxv_d_newton_cradle   = { b4_hdr_bytes, nc_init, nc_advance, 0, nc_render, 0, 0, 0 };
const fxv_desc_t fxv_d_sonar_ping      = { b4_hdr_bytes, sp_init, sp_advance, sp_slice, sp_render, 0, 0, 0 };
const fxv_desc_t fxv_d_rope_pulse      = { wv_bytes, wv_init, wv_advance, wv_slice, rp_render, 0, wv_changed, wv_digest };
const fxv_desc_t fxv_d_soliton_courier = { b4_hdr_bytes, sc_init, sc_advance, 0, sc_render, 0, 0, 0 };
const fxv_desc_t fxv_d_echo_vault      = { wv_bytes, wv_init, wv_advance, wv_slice, ev_render, 0, wv_changed, wv_digest };
