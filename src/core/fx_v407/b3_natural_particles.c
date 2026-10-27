/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Batch B3 "natural particles": 45 Firefly Sync, 46 Rain on Glass, 50 Koi Pond, 53 Dust Motes,
 * 56 Petal Fall, 57 Starlings, 67 Dew Necklace, 75 Marble Run, 90 Orbit Slingshot, 107 Snowfall, 112 Champagne;
 * manifest/b3_natural_particles.json, JS fx/v407/b3_natural_particles.js (the same names, the same bytes).
 *
 * Everything is one-dimensional: the scene lives on the virtual strip (vbase / vlen, so receivers sharing a virtual
 * strip simulate the same scene) and moves along it; "top" is the strip's start, "bottom" its end (Champagne rises from
 * the start). Ten effects run on the frame driver below: their particles are the layer's slab range (x->p, x->pn; the
 * count may change between frames, records fade in after a spawn and out when no longer wanted), stepped on fixed
 * simulation ticks (fxv_tick_next); after the ticks the records are drawn into raster planes in the arena (uint16 a
 * pixel) and render16 only reads those: rendering costs the same at any particle count and any chunk order gives the
 * same pixels. Positions are Q16.16 virtual pixels; drawing adds the tick fraction (smooth motion at any frame rate).
 * Dew Necklace is closed-form on its own real-time clock (no particles).
 * B3-local helpers B0 may want: b3_splat / b3_splat4 (fxv_splat into planes), b3_k_of (a colour's temperature),
 * b3_isqrt, b3_sort_step (an insertion sort sliced by element over an index order), the frame driver itself. */
#include "fxv_gen.h"

#define B3_LMAX    32000u              /* the longest simulated strip (Q16.16 positions in an int32) */
#define B3_IDLE    0xFFu               /* job[2]: between two ticks */

/* the driver's words at the start of the arena (uint32); an effect's own words from H_E */
#define H_PRIMED   0u                  /* 1: the wanted records were spread over the strip once */
#define H_N        1u                  /* records wanted this frame */
#define H_CNT      2u                  /* records in use: min(pn, cap) */
#define H_FRAC     3u                  /* the tick fraction for drawing, Q16 */
#define H_SORTN    4u                  /* the order table covers this many records (0: rebuild it) */
#define H_L        5u                  /* the simulated length, px */
#define H_PLANES   6u                  /* byte offset of the first plane */
#define H_STRIDE   7u                  /* uint16 a plane */
#define H_E        8u

typedef struct {
    uint16_t hdr, extra;               /* state bytes, then the effect's tables (multiples of 4) */
    uint8_t  cap, npass;               /* records (the manifest's particles), passes a tick */
    uint8_t  batch, sbatch;            /* work units a pass slice (a record 1, a sort step 1 + shifts / 8); records a
                                          prime / draw slice */
    uint8_t  nkeep, nplane;            /* planes kept from frame to frame, then planes cleared and drawn each frame */
    uint8_t  cstep, maxt;              /* pixels a clear slice; ticks a frame may run (0: the layer's 16, the rest debt) */
    uint8_t  tailq, spare;             /* after the planes: tailq bytes a sample, a sample every 4 px (+ 2) */
    uint32_t (*want)(fxv_ctx_t *x, uint32_t *h);                  /* per frame: x->u[]; returns the records wanted */
    void (*prime)(fxv_ctx_t *x, uint32_t *h, uint32_t k);        /* once: record k anywhere on the strip */
    void (*tick)(fxv_ctx_t *x, uint32_t *h);                     /* a tick starts */
    uint32_t (*pass)(fxv_ctx_t *x, uint32_t *h, uint32_t pass, uint32_t k);   /* returns its work units */
    void (*tick_end)(fxv_ctx_t *x, uint32_t *h);
    void (*post)(fxv_ctx_t *x, uint32_t *h);                     /* after the frame's ticks (small tables) */
    void (*clear)(fxv_ctx_t *x, uint32_t *h, uint32_t i);        /* pixel i at the clear (a kept plane ages) */
    void (*splat)(fxv_ctx_t *x, uint32_t *h, uint32_t k);        /* draw record k */
} b3_spec_t;

/* ------------------------------------------------------------------------------- small helpers */
static uint32_t *b3_h(const fxv_ctx_t *x)
{
    return (uint32_t *)x->arena;
}

static uint16_t *b3_pl(const fxv_ctx_t *x, uint32_t j)     /* plane j (the kept ones first) */
{
    const uint32_t *h = b3_h(x);
    return (uint16_t *)(x->arena + h[H_PLANES]) + j * h[H_STRIDE];
}

static uint32_t b3_rnd(const fxv_ctx_t *x, uint32_t k, uint32_t salt)
{
    return fxv_rand(x->seed, x->tick, k * 16u + salt);
}

static int32_t b3_ix(const uint32_t *h, const fxv_particle_t *p)   /* the drawing position: x + v x the tick fraction */
{
    return p->x + (((p->v >> 6) * (int32_t)(h[H_FRAC] >> 6)) >> 4);
}

static int32_t b3_clampi(int32_t v, int32_t lim)
{
    return v > lim ? lim : v < -lim ? -lim : v;
}

static uint32_t b3_want(uint32_t L, uint32_t intensity, uint32_t per)   /* records for a strip: (I + 8) x L / per */
{
    const uint32_t w = L * (intensity + 8u) / per;
    return intensity == 0u ? 0u : w < 1u ? 1u : w;
}

/* fade in after a spawn, out once the record is no longer wanted; 0: it has faded out (the record is free again) */
static uint32_t b3_fade(fxv_particle_t *p, uint32_t keep, uint32_t step)
{
    if (keep) {
        const uint32_t f = (uint32_t)p->flags + step;
        p->flags = (uint8_t)(f > 255u ? 255u : f);
        return 1;
    }
    if (p->flags > step) {
        p->flags = (uint8_t)(p->flags - step);
        return 1;
    }
    p->flags = 0;
    p->life = 0;
    return 0;
}

/* B3-local (a B0 candidate): the colour temperature byte (0 = 2000 K .. 255 = 6500 K) whose kelvin white has this
 * colour's blue / red; bluer than 6500 K (or no red) gives 255 */
static uint32_t b3_k_of(rgb_t c)
{
    uint32_t t, j, k;
    if (c.r == 0u || c.b >= c.r) return 255u;
    t = (uint32_t)c.b * 255u / c.r;
    for (j = 0; j < 16u && fxv_kelvin_tab[j + 1u][2] < t; j++) { }
    if (j >= 16u) return 255u;
    if (t <= fxv_kelvin_tab[j][2]) return j * 16u;
    k = j * 16u + (t - fxv_kelvin_tab[j][2]) * 16u / (uint32_t)(fxv_kelvin_tab[j + 1u][2] - fxv_kelvin_tab[j][2]);
    return k > 255u ? 255u : k;
}

/* B3-local (a B0 candidate): floor(sqrt(v)) */
static uint32_t b3_isqrt(uint32_t v)
{
    uint32_t r = 0, b = 1u << 30;
    while (b > v) b >>= 2;
    while (b != 0u) {
        if (v >= r + b) {
            v -= r + b;
            r = (r >> 1) + b;
        } else {
            r >>= 1;
        }
        b >>= 2;
    }
    return r;
}

/* palette mode (pal_own 0): the palette's step along the virtual strip, Q8 of a palette position (u16) a pixel; pixel vp
 * sits at (vp x step + step / 2) >> 8, so the strip spans the whole palette (no division per pixel) */
static uint32_t b3_pstep(uint32_t L)
{
    return (65536u << 8) / (L ? L : 1u);
}

static fxv_px_t b3_white(fxv_px_t c, uint32_t k)            /* a colour as natural white of temperature k */
{
    c.q = 255;
    c.k = (uint8_t)k;
    return c;
}

/* B3-local (a B0 candidate: fxv_splat into a plane): add amp (0..65535) x a triangular kernel of radius r (Q16, at
 * least 0.5 px) centred at the virtual position xv (Q16) into the run's plane pl (run pixel = virtual - vbase),
 * saturating at 65535 */
static void b3_splat(uint16_t *pl, const fxv_ctx_t *x, int32_t xv, uint32_t r, uint32_t amp)
{
    int32_t c, i, lo, hi;
    uint32_t kq;
    if (x->vbase >= 32768u || amp == 0u) return;
    if (r < 32768u) r = 32768u;
    c = xv - (int32_t)(x->vbase << 16);
    lo = (c - (int32_t)r) >> 16;
    hi = (c + (int32_t)r) >> 16;
    if (lo < 0) lo = 0;
    if (hi >= (int32_t)x->n) hi = (int32_t)x->n - 1;
    if (lo > hi) return;
    kq = (65535u << 8) / (r >> 8);
    for (i = lo; i <= hi; i++) {
        const int32_t dd = i * 65536 + 32768 - c;
        const uint32_t d = (uint32_t)(dd < 0 ? -dd : dd);
        uint32_t w, v;
        if (d >= r) continue;
        w = (((r - d) >> 8) * kq) >> 8;
        if (w > 65535u) w = 65535u;
        v = pl[i] + ((amp * w) >> 16);
        pl[i] = (uint16_t)(v > 65535u ? 65535u : v);
    }
}

/* the same kernel, a working colour into planes j0, j0 + 1, j0 + 2 and a white amount into plane j0 + 3 */
static void b3_splat4(const fxv_ctx_t *x, uint32_t j0, int32_t xv, uint32_t r, fxv_px_t col, uint32_t wa)
{
    uint16_t *pr = b3_pl(x, j0), *pg = b3_pl(x, j0 + 1u), *pb = b3_pl(x, j0 + 2u), *pw = b3_pl(x, j0 + 3u);
    int32_t c, i, lo, hi;
    uint32_t kq;
    if (x->vbase >= 32768u) return;
    if (r < 32768u) r = 32768u;
    c = xv - (int32_t)(x->vbase << 16);
    lo = (c - (int32_t)r) >> 16;
    hi = (c + (int32_t)r) >> 16;
    if (lo < 0) lo = 0;
    if (hi >= (int32_t)x->n) hi = (int32_t)x->n - 1;
    if (lo > hi) return;
    kq = (65535u << 8) / (r >> 8);
    for (i = lo; i <= hi; i++) {
        const int32_t dd = i * 65536 + 32768 - c;
        const uint32_t d = (uint32_t)(dd < 0 ? -dd : dd);
        uint32_t w, v;
        if (d >= r) continue;
        w = (((r - d) >> 8) * kq) >> 8;
        if (w > 65535u) w = 65535u;
        v = pr[i] + ((col.r * w) >> 16); pr[i] = (uint16_t)(v > 65535u ? 65535u : v);
        v = pg[i] + ((col.g * w) >> 16); pg[i] = (uint16_t)(v > 65535u ? 65535u : v);
        v = pb[i] + ((col.b * w) >> 16); pb[i] = (uint16_t)(v > 65535u ? 65535u : v);
        v = pw[i] + ((wa * w) >> 16); pw[i] = (uint16_t)(v > 65535u ? 65535u : v);
    }
}

/* planes j0 .. j0 + 3 at run pixel i -> a working colour, the white plane as natural white of temperature k */
static fxv_px_t b3_px4(const fxv_ctx_t *x, uint32_t j0, uint32_t i, uint32_t k)
{
    fxv_px_t c;
    const uint32_t r = b3_pl(x, j0)[i], g = b3_pl(x, j0 + 1u)[i], b = b3_pl(x, j0 + 2u)[i], w = b3_pl(x, j0 + 3u)[i];
    c.r = (uint16_t)(r > FXV_FULL ? FXV_FULL : r);
    c.g = (uint16_t)(g > FXV_FULL ? FXV_FULL : g);
    c.b = (uint16_t)(b > FXV_FULL ? FXV_FULL : b);
    c.q = 0;
    c.k = 0;
    return w ? fxv_add_px(c, fxv_scale_px(fxv_kelvin16(k), w)) : c;
}

/* B3-local (a B0 candidate): insert ord[j] into the sorted ord[0 .. j) by position (free records last); called for
 * j = 0, 1, .. in slices it is an insertion sort: cheap when the order barely changed since the last tick */
static uint32_t b3_sort_step(const fxv_particle_t *p, uint8_t *ord, uint32_t j)   /* returns its work units */
{
    const uint32_t k = ord[j];
    const int32_t key = p[k].life ? p[k].x : 0x7FFFFFFF;
    uint32_t i = j;
    while (i > 0u) {
        const uint32_t q = ord[i - 1u];
        if ((p[q].life ? p[q].x : 0x7FFFFFFF) <= key) break;
        ord[i] = (uint8_t)q;
        i--;
    }
    ord[i] = (uint8_t)k;
    return 1u + ((j - i) >> 3);
}

static void b3_order_init(uint32_t *h, uint8_t *ord)       /* the order table after the record count changed */
{
    uint32_t i;
    if (h[H_SORTN] == h[H_CNT]) return;
    for (i = 0; i < h[H_CNT]; i++) ord[i] = (uint8_t)i;
    h[H_SORTN] = h[H_CNT];
}

/* ------------------------------------------------------------------------------- the frame driver */
static uint32_t b3_bytes(const b3_spec_t *s, uint32_t n)
{
    return (uint32_t)s->hdr + s->extra + (uint32_t)(s->nkeep + s->nplane) * 2u * ((n + 1u) & ~1u) + s->tailq * ((n >> 2) + 2u);
}

static void b3_begin(fxv_ctx_t *x, const b3_spec_t *s)
{
    uint32_t *h = b3_h(x), cnt = x->p ? x->pn : 0u, w;
    if (cnt > s->cap) cnt = s->cap;
    if (cnt != h[H_CNT]) {
        h[H_CNT] = cnt;
        h[H_SORTN] = 0;
    }
    h[H_L] = x->vlen < B3_LMAX ? x->vlen : B3_LMAX;
    h[H_PLANES] = (uint32_t)s->hdr + s->extra;
    h[H_STRIDE] = ((uint32_t)x->n + 1u) & ~1u;
    h[H_FRAC] = x->tick_ms ? (x->tick_acc << 16) / x->tick_ms : 0u;
    w = s->want(x, h);
    h[H_N] = w < cnt ? w : cnt;
    x->job[0] = h[H_PRIMED] ? 1u : 0u;
    x->job[1] = 0;
    x->job[2] = B3_IDLE;
    x->job[3] = 0;                                       /* ticks run this frame */
}

static uint32_t b3_slice(fxv_ctx_t *x, const b3_spec_t *s)
{
    uint32_t *h = b3_h(x), *j = x->job, k, end, lim, w;
    switch (j[0]) {
    case 0:                                              /* once: spread the wanted records over the strip */
        lim = h[H_N];
        end = j[1] + s->sbatch < lim ? j[1] + s->sbatch : lim;
        for (k = j[1]; k < end; k++) s->prime(x, h, k);
        j[1] = end;
        if (end >= lim) {
            h[H_PRIMED] = 1;
            j[0] = 1;
            j[1] = 0;
        }
        return 1;
    case 1:                                              /* the frame's ticks: tick(), the passes, tick_end() */
        if (j[2] == B3_IDLE) {
            if ((s->maxt && j[3] >= s->maxt) || !fxv_tick_next(x)) {
                if (s->post) s->post(x, h);
                if (!s->nplane && !s->clear) {
                    j[0] = 4;
                    return 0;
                }
                j[0] = 2;
                j[1] = 0;
                return 1;
            }
            j[3]++;
            if (s->tick) s->tick(x, h);
            j[1] = 0;
            j[2] = 0;
            return 1;
        }
        lim = h[H_CNT];
        for (k = j[1], w = 0; k < lim && w < s->batch; k++) w += s->pass(x, h, j[2], k);
        j[1] = k;
        if (k >= lim) {
            j[1] = 0;
            if (++j[2] >= s->npass) {
                if (s->tick_end) s->tick_end(x, h);
                j[2] = B3_IDLE;
            }
        }
        return 1;
    case 2:                                              /* clear the drawn planes (and age a kept one) */
        lim = x->n;
        end = j[1] + s->cstep;
        if (end > lim) end = lim;
        if (s->nplane) {
            uint16_t *pl = b3_pl(x, s->nkeep);
            uint32_t q;
            for (q = 0; q < s->nplane; q++, pl += h[H_STRIDE]) for (k = j[1]; k < end; k++) pl[k] = 0;
        }
        if (s->clear) for (k = j[1]; k < end; k++) s->clear(x, h, k);
        j[1] = end;
        if (end >= lim) {
            j[0] = 3;
            j[1] = 0;
        }
        return 1;
    case 3:                                              /* draw the records */
        lim = h[H_CNT];
        end = j[1] + s->sbatch < lim ? j[1] + s->sbatch : lim;
        if (s->splat) for (k = j[1]; k < end; k++) s->splat(x, h, k);
        j[1] = end;
        if (end < lim) return 1;
        j[0] = 4;
        return 0;
    default:
        return 0;
    }
}

static void b3_init(fxv_ctx_t *x, const b3_spec_t *s)
{
    uint32_t *h = b3_h(x);
    h[H_PLANES] = (uint32_t)s->hdr + s->extra;
    h[H_STRIDE] = ((uint32_t)x->n + 1u) & ~1u;
    x->job[0] = 4;
}

/* the state for the C <-> JS goldens: the arena's words and the records */
static uint32_t b3_digest(const fxv_ctx_t *x)
{
    const uint32_t *h = b3_h(x);
    uint32_t k, d = 0x0B3u;
    for (k = 0; k < 64u; k++) d = fxv_hash2(d, h[k]);
    for (k = 0; k < h[H_CNT]; k++) {
        const fxv_particle_t *p = &x->p[k];
        d = fxv_hash2(d, (uint32_t)p->x);
        d = fxv_hash2(d, (uint32_t)p->v);
        d = fxv_hash2(d, (uint32_t)p->age | (uint32_t)p->life << 16);
        d = fxv_hash2(d, (uint32_t)p->pal | (uint32_t)p->phase << 8 | (uint32_t)p->flags << 16 | (uint32_t)p->kind << 24);
    }
    return d;
}

#define B3_DESC(sym, spec, render)                                                                                    \
    static uint32_t sym##_bytes(uint32_t n, uint32_t tier) { (void)tier; return b3_bytes(&spec, n); }                \
    static void sym##_init(fxv_ctx_t *x) { b3_init(x, &spec); }                                                      \
    static void sym##_adv(fxv_ctx_t *x) { b3_begin(x, &spec); }                                                      \
    static uint32_t sym##_slice(fxv_ctx_t *x, uint32_t quota) { (void)quota; return b3_slice(x, &spec); }            \
    const fxv_desc_t fxv_d_##sym = { sym##_bytes, sym##_init, sym##_adv, sym##_slice, render, 0, 0, b3_digest };

/* ------------------------------------------------------------------------------- 45 Firefly Sync
 * Fireflies drift over the dark meadow (c2), each blinking c1 on its own rhythm (a phase oscillator, ~1.6 s a blink at
 * 1x, +-6 %). Kuramoto coupling to the flock's mean phase (custom1) runs in a slow cycle (4,096 ticks): a quarter apart,
 * falling into step, the whole field flashing together, then drifting apart. custom2: the glow's length. Saturated
 * RGB (q 0): the blink stays yellow-green on white strips. */
#define FF_SX   (H_E + 0u)             /* last tick's sums of (cos + 32768), (sin + 32768) and their count */
#define FF_SY   (H_E + 1u)
#define FF_NA   (H_E + 2u)
#define FF_AX   (H_E + 3u)             /* this tick's sums */
#define FF_AY   (H_E + 4u)
#define FF_AN   (H_E + 5u)
#define FF_MC   (H_E + 6u)             /* this tick: mean cos and sin (+ 32768), the coupling */
#define FF_MS   (H_E + 7u)
#define FF_K    (H_E + 8u)

static uint32_t ff_want(fxv_ctx_t *x, uint32_t *h)
{
    x->u[0] = 7864u + x->custom2 * 110u;                 /* the glow's decay: 0.12 .. 0.55 of a blink cycle */
    x->u[1] = 0xFFFFFFFFu / x->u[0];
    if (!x->pal_own) x->u[7] = b3_pstep(h[H_L]);
    return b3_want(h[H_L], x->intensity, 2400u);
}

static void ff_spawn(fxv_ctx_t *x, const uint32_t *h, fxv_particle_t *p, uint32_t k, uint32_t salt)
{
    const uint32_t r = b3_rnd(x, k, salt), r2 = b3_rnd(x, k, salt + 1u);
    p->x = (int32_t)((r >> 16) * h[H_L]);
    p->v = 0;
    p->age = (uint16_t)r;                                /* the blink phase */
    p->life = (uint16_t)(770u + (((r2 >> 16) * 100u) >> 16));    /* its rate a tick: ~80 ticks a blink */
    p->flags = 0;
}

static void ff_prime(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    ff_spawn(x, h, &x->p[k], k, 8u);
}

static void ff_tick(fxv_ctx_t *x, uint32_t *h)
{
    const uint32_t na = h[FF_NA], cyc = (x->tick + (x->seed >> 20)) & 4095u;
    uint32_t env;
    h[FF_MC] = na ? h[FF_SX] / na : 32768u;
    h[FF_MS] = na ? h[FF_SY] / na : 32768u;
    if (cyc < 1024u) env = 0;                            /* apart: each on its own rhythm */
    else if (cyc < 1638u) env = fxv_smooth16((cyc - 1024u) * 106u);
    else if (cyc < 3482u) env = 65535u;                  /* in step */
    else env = fxv_smooth16(65535u - (cyc - 3482u) * 106u);
    h[FF_K] = (x->custom1 * 2u * env) >> 16;
    h[FF_AX] = 0;
    h[FF_AY] = 0;
    h[FF_AN] = 0;
}

static uint32_t ff_pass(fxv_ctx_t *x, uint32_t *h, uint32_t pass, uint32_t k)
{
    fxv_particle_t *p = &x->p[k];
    const uint32_t keep = k < h[H_N];
    const int32_t L16 = (int32_t)(h[H_L] << 16);
    uint32_t th, r;
    int32_t nudge;
    (void)pass;
    if (p->life == 0u) {
        if (!keep) return 1u;
        ff_spawn(x, h, p, k, 0);
    }
    if (!b3_fade(p, keep, 3u)) return 1u;
    th = p->age;                                         /* phase: own rate + coupling x R sin(mean - phase) */
    nudge = ((int32_t)h[FF_K] * (((((int32_t)h[FF_MS] - 32768) * fxv_cos16(th)) >> 15)
                                 - ((((int32_t)h[FF_MC] - 32768) * fxv_sin16(th)) >> 15))) >> 15;
    th = (th + p->life + (uint32_t)nudge) & 0xFFFFu;
    p->age = (uint16_t)th;
    h[FF_AX] += (uint32_t)(fxv_cos16(th) + 32768);
    h[FF_AY] += (uint32_t)(fxv_sin16(th) + 32768);
    h[FF_AN]++;
    r = b3_rnd(x, k, 2u);                                /* drift: a damped random walk, turned back at the ends */
    p->v += ((int32_t)(r >> 16) - 32768) >> 6;
    p->v -= p->v >> 5;
    p->v = b3_clampi(p->v, 4000);
    p->x += p->v;
    if (p->x < 0) {
        p->x = -p->x;
        p->v = -p->v;
    }
    if (p->x >= L16) {
        p->x = L16 - 1 - (p->x - L16);
        p->v = -p->v;
    }
    return 1u;
}

static void ff_tick_end(fxv_ctx_t *x, uint32_t *h)
{
    (void)x;
    h[FF_SX] = h[FF_AX];
    h[FF_SY] = h[FF_AY];
    h[FF_NA] = h[FF_AN];
}

static void ff_splat(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    const fxv_particle_t *p = &x->p[k];
    const uint32_t ph = p->age;
    uint32_t e, a;
    int32_t c;
    if (p->life == 0u) return;
    if (ph < 4096u) {
        e = fxv_smooth16(ph * 16u);                      /* a quick rise */
    } else if (ph - 4096u < x->u[0]) {
        e = fxv_smooth16(65535u - (((ph - 4096u) * x->u[1]) >> 16));
        e = (e * e) >> 16;                               /* a soft glowing tail */
    } else {
        return;
    }
    a = (e * ((uint32_t)p->flags + 1u)) >> 8;
    c = b3_ix(h, p);
    b3_splat(b3_pl(x, 0), x, c, 104858u, a);             /* the light, 1.6 px */
    b3_splat(b3_pl(x, 0), x, c, 229376u, a >> 2);        /* its halo, 3.5 px */
}

static void ff_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint16_t *pl = b3_pl(x, 0) + from;
    if (x->pal_own) {
        const fxv_px_t bg = fxv_rgb16(x->c[1]), fly = fxv_rgb16(x->c[0]);
        uint32_t i;
        for (i = 0; i < count; i++) o[i] = pl[i] ? fxv_add_px(bg, fxv_scale_px(fly, pl[i])) : bg;
    } else {
        /* palette: the place on the meadow (the strip) picks the colour, so each firefly glows the palette where it
         * floats and a synced flash shows the whole palette; the meadow is that colour at the meadow's darkness (the
         * default meadow's 6 / 255), the glow adds to that level (one scale a pixel) */
        const uint32_t st = x->u[7];
        uint32_t i, a = (x->vbase + from) * st + (st >> 1);
        for (i = 0; i < count; i++, a += st) {
            const uint32_t lv = pl[i] + 1542u;
            o[i] = fxv_scale_px(fxv_pal(x, a >> 8), lv > 65535u ? 65535u : lv);
        }
    }
}

static const b3_spec_t FF = { 256, 0, 64, 1, 9, 8, 0, 1, 128, 0, 0, 0, ff_want, ff_prime, ff_tick, ff_pass, ff_tick_end, 0, 0, ff_splat };
B3_DESC(firefly_sync, FF, ff_render)

/* ------------------------------------------------------------------------------- 46 Rain on Glass
 * Raindrops land on the pane (speed: the rain), cling, merge with neighbours they touch (an ordered sweep, no
 * all-pairs), and run down toward the strip's end once heavy, sweeping up the beads in their way, slowing as they
 * shed water and leaving a wet streak that dries (custom2: how long). Behind the glass: soft blurred city lights,
 * palette colours on slow noise (custom1: their brightness). A drop is a little lens: it shows the lights behind it
 * flipped and brighter, with a cool white highlight on its upper side (q 255). Intensity: drop size. */
#define RG_ACC  (H_E + 0u)             /* landings due (Q16) */
#define RG_FR   (H_E + 1u)             /* frames run: the light samples refresh a quarter a frame */

static uint8_t *rg_ord(const fxv_ctx_t *x)
{
    return x->arena + 256u;
}

static uint32_t rg_r(uint32_t m)                         /* a drop's radius (Q16) from its mass */
{
    const uint32_t r = 30000u + m * 12u;
    return r > 196608u ? 196608u : r;
}

static uint32_t rg_want(fxv_ctx_t *x, uint32_t *h)
{
    const uint32_t de = x->de < 4000u ? x->de : 4000u, w = h[H_L] / 10u;
    x->u[0] = de * 65536u / (600u + x->custom2 * 45u);   /* the streaks dry over 0.6 .. 12 s */
    if (x->u[0] > 65535u) x->u[0] = 65535u;
    x->u[1] = x->et * 2u;                                /* the lights' slow drift */
    x->u[2] = x->custom1 * 200u;                         /* their brightness (255: 0.78) */
    h[RG_FR]++;
    return w < 1u ? 1u : w;
}

static void rg_prime(fxv_ctx_t *x, uint32_t *h, uint32_t k)   /* it has been raining: half the drops already there */
{
    fxv_particle_t *p = &x->p[k];
    const uint32_t r = b3_rnd(x, k, 8u), m0 = 400u + x->intensity * 14u;
    if (k & 1u) return;
    p->x = (int32_t)((r >> 16) * h[H_L]);
    p->v = 0;
    p->age = 0;
    p->life = (uint16_t)((m0 * (154u + (((r >> 8) & 255u) * 205u >> 8))) >> 8);
    p->pal = (uint8_t)r;
    p->phase = (uint8_t)(r >> 24);
    p->flags = 0;
    p->kind = 0;
}

/* a drop lands: on a free record (k < N), else it joins a drop already there */
static void rg_land(fxv_ctx_t *x, uint32_t *h, uint32_t n)
{
    const uint32_t r = b3_rnd(x, 200u + n, 0), r2 = b3_rnd(x, 200u + n, 1u), N = h[H_N];
    const uint32_t m0 = 400u + x->intensity * 14u, m = (m0 * (154u + (((r2 >> 24) * 205u) >> 8))) >> 8;
    uint32_t k, s, i;
    if (N == 0u) return;
    s = ((r >> 16) * N) >> 16;
    for (i = 0; i < N; i++) {
        k = s + i < N ? s + i : s + i - N;
        if (x->p[k].life == 0u) {
            fxv_particle_t *p = &x->p[k];
            p->x = (int32_t)((r2 & 0xFFFFu) * h[H_L]);
            p->v = 0;
            p->age = 0;
            p->life = (uint16_t)m;
            p->pal = (uint8_t)r;
            p->phase = (uint8_t)(r2 >> 16);              /* stickiness */
            p->flags = 0;
            p->kind = 0;
            return;
        }
    }
    {
        fxv_particle_t *p = &x->p[s];
        const uint32_t mm = p->life + m;
        p->life = (uint16_t)(mm > 65535u ? 65535u : mm);
    }
}

static void rg_tick(fxv_ctx_t *x, uint32_t *h)
{
    uint32_t n = 0;
    b3_order_init(h, rg_ord(x));
    h[RG_ACC] += h[H_L] * 33u;                           /* ~2.5 drops a second per 100 px at 1x */
    while (h[RG_ACC] >= 65536u && n < 2u) {
        h[RG_ACC] -= 65536u;
        rg_land(x, h, n++);
    }
    if (h[RG_ACC] >= 65536u) h[RG_ACC] = 65535u;
}

static uint32_t rg_pass(fxv_ctx_t *x, uint32_t *h, uint32_t pass, uint32_t k)
{
    fxv_particle_t *p;
    uint32_t m;
    if (pass == 1u) {
        return k < h[H_SORTN] ? b3_sort_step(x->p, rg_ord(x), k) : 1u;
    }
    p = &x->p[k];
    if (p->life == 0u || !b3_fade(p, k < h[H_N], 40u)) return 1u;
    m = p->life;
    if (p->kind == 0u) {                                 /* clinging: until heavy enough to slide */
        if (p->age < 65535u) p->age++;
        if (m > 6000u + p->phase * 24u) {
            p->kind = 1;
            p->v = 1300;
        }
        return 1u;
    } else {                                             /* running: gravity, drag, it sheds water, wets the glass */
        const int32_t old = p->x;
        int32_t i, i1;
        p->v += 300 + (int32_t)(m >> 5);
        p->v -= p->v >> 8;
        if (p->v > 98304) p->v = 98304;
        p->x += p->v;
        if (x->vbase < 32768u) {
            uint16_t *wet = b3_pl(x, 0);
            i = (old >> 16) - (int32_t)x->vbase;
            i1 = (p->x >> 16) - (int32_t)x->vbase;
            if (i < 0) i = 0;
            if (i1 >= (int32_t)x->n) i1 = (int32_t)x->n - 1;
            for (; i <= i1; i++) if (wet[i] < 52000u) wet[i] = 52000u;
        }
        m -= (m >> 9) + 2u;
        if (m < 3000u) {                                 /* too light: it stops and clings again */
            p->kind = 0;
            p->v = 0;
            p->age = 0;
        }
        p->life = (uint16_t)m;
        if (p->x >= (int32_t)((h[H_L] + 4u) << 16)) {
            p->life = 0;
            p->flags = 0;
        }
    }
    return 1u;
}

/* touching neighbours merge: the lighter joins the heavier at their mass centre (at most 4 merges a tick) */
static void rg_tick_end(fxv_ctx_t *x, uint32_t *h)
{
    const uint8_t *ord = rg_ord(x);
    uint32_t j, a = 0xFFFFu, merges = 0;
    for (j = 0; j < h[H_SORTN] && merges < 4u; j++) {
        const uint32_t k = ord[j];
        fxv_particle_t *q = &x->p[k];
        if (q->life == 0u) break;                        /* free records are sorted last */
        if (a != 0xFFFFu) {
            fxv_particle_t *pa = &x->p[a];
            const uint32_t gap = (uint32_t)(q->x - pa->x);
            if (gap < rg_r(pa->life) + rg_r(q->life)) {
                const uint32_t ma = pa->life, mb = q->life, w = (mb << 8) / (ma + mb);
                const int32_t nx = pa->x + (int32_t)((gap * w) >> 8);
                fxv_particle_t *kp = ma >= mb ? pa : q, *gone = ma >= mb ? q : pa;
                kp->x = nx;
                kp->life = (uint16_t)(ma + mb > 65535u ? 65535u : ma + mb);
                if (gone->kind) {
                    kp->kind = 1;
                    if (gone->v > kp->v) kp->v = gone->v;
                }
                if (gone->flags > kp->flags) kp->flags = gone->flags;
                gone->life = 0;
                gone->flags = 0;
                merges++;
                a = ma >= mb ? a : k;
                continue;
            }
        }
        a = k;
    }
}

/* a drop: its body (coverage), where its lens looks (Q8 px offset: back through the centre, half scale), and its
 * highlight (stronger on the upper side) */
static void rg_splat(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    const fxv_particle_t *p = &x->p[k];
    uint16_t *cov = b3_pl(x, 1), *off = b3_pl(x, 2), *hl = b3_pl(x, 3);
    uint32_t r, kr;
    int32_t c, i, lo, hi;
    if (p->life == 0u || x->vbase >= 32768u) return;
    r = rg_r(p->life);
    kr = (65535u << 8) / (r >> 8);
    c = b3_ix(h, p) - (int32_t)(x->vbase << 16);
    lo = (c - (int32_t)r) >> 16;
    hi = (c + (int32_t)r) >> 16;
    if (lo < 0) lo = 0;
    if (hi >= (int32_t)x->n) hi = (int32_t)x->n - 1;
    for (i = lo; i <= hi; i++) {
        const int32_t d = i * 65536 + 32768 - c;
        const uint32_t ad = (uint32_t)(d < 0 ? -d : d);
        uint32_t lin, cv, hv;
        if (ad >= r) continue;
        lin = ((ad >> 8) * kr) >> 8;
        cv = (fxv_smooth16(65535u - lin) * ((uint32_t)p->flags + 1u)) >> 8;
        if (cv > cov[i]) {
            cov[i] = (uint16_t)cv;
            off[i] = (uint16_t)(-(d * 3) >> 9);
        }
        hv = (cv * (d < 0 ? 40000u : 14000u)) >> 16;
        if (p->kind) hv += hv >> 2;
        if (hv > hl[i]) hl[i] = (uint16_t)hv;
    }
}

/* the city lights behind the glass from two noises: hue on 64 px, blobs on 16 px */
static fxv_px_t rg_light(const fxv_ctx_t *x, int32_t n1, int32_t n2)
{
    const int32_t l = n2 - 4000;
    uint32_t lv = 0;
    if (l > 0) lv = fxv_smooth16((uint32_t)(l * 3 > 65535 ? 65535 : l * 3));
    lv = 5000u + ((lv * 60535u) >> 16);
    return fxv_scale_px(fxv_pal(x, ((uint32_t)(n1 + 32768) * 2u) & 0xFFFFu), (lv * x->u[2]) >> 16);
}

/* the lights at run pixel 4 s (a sample every 4 px, after the planes: r, g, b) */
static void rg_sample(fxv_ctx_t *x, uint32_t s)
{
    uint16_t *t = b3_pl(x, 4) + s * 3u;
    const uint32_t vp = x->vbase + s * 4u;
    const fxv_px_t c = rg_light(x, fxv_noise1_q15(x->seed ^ 0x46Au, vp * 1024u + 512u, x->u[1], 1),
                                fxv_noise1_q15(x->seed ^ 0x46Bu, vp * 4096u + 2048u, x->u[1] * 3u, 2));
    t[0] = c.r;
    t[1] = c.g;
    t[2] = c.b;
}

static void rg_clear(fxv_ctx_t *x, uint32_t *h, uint32_t i)   /* the streaks dry; the lights are sampled */
{
    uint16_t *wet = b3_pl(x, 0);
    const uint32_t w = wet[i];
    if (w) wet[i] = (uint16_t)(w - ((w * x->u[0]) >> 16) - 1u);
    if ((i & 3u) == 0u && (h[RG_FR] == 1u || (((i >> 2) + h[RG_FR]) & 3u) == 0u)) rg_sample(x, i >> 2);
    if (i + 1u == x->n && (h[RG_FR] == 1u || (((i >> 2) + 1u + h[RG_FR]) & 3u) == 0u)) rg_sample(x, (i >> 2) + 1u);
}

static fxv_px_t rg_bg(const fxv_ctx_t *x, uint32_t s, uint32_t f)   /* between samples s and s + 1 (f 0..65535) */
{
    const uint16_t *t = b3_pl(x, 4) + s * 3u;
    fxv_px_t c;
    c.r = (uint16_t)fxv_lerp16(t[0], t[3], f);
    c.g = (uint16_t)fxv_lerp16(t[1], t[4], f);
    c.b = (uint16_t)fxv_lerp16(t[2], t[5], f);
    c.q = 0;
    c.k = 0;
    return c;
}

static void rg_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint16_t *wet = b3_pl(x, 0) + from, *cov = b3_pl(x, 1) + from, *off = b3_pl(x, 2) + from, *hl = b3_pl(x, 3) + from;
    const fxv_px_t sheen = fxv_kelvin16(230);
    const uint32_t smax = ((x->n - 1u) >> 2) + 1u;
    uint32_t i;
    for (i = 0; i < count; i++) {
        const uint32_t px = from + i;
        fxv_px_t c = rg_bg(x, px >> 2, (px & 3u) << 14);
        if (wet[i]) {                                    /* wet glass: clearer, with a faint sheen */
            c = fxv_add_px(c, fxv_scale_px(c, (wet[i] * 39321u) >> 16));
            c = fxv_add_px(c, fxv_scale_px(sheen, wet[i] >> 5));
        }
        if (cov[i]) {                                    /* a drop: the lights behind it, flipped and focused */
            int32_t u8 = ((int32_t)(px << 8) + (int16_t)off[i]) >> 2;
            uint32_t sn, f;
            fxv_px_t lens;
            if (u8 < 0) u8 = 0;
            sn = (uint32_t)u8 >> 8;
            f = ((uint32_t)u8 & 255u) << 8;
            if (sn >= smax) {
                sn = smax - 1u;
                f = 65535u;
            }
            lens = rg_bg(x, sn, f);
            lens = fxv_add_px(lens, fxv_scale_px(lens, 32768u));
            c = fxv_blend16(c, lens, cov[i]);
            if (hl[i]) c = fxv_add_px(c, fxv_scale_px(sheen, hl[i]));
        }
        o[i] = c;
    }
}

/* (8 pixels a clear slice: the first frame samples the lights every 4th pixel, plus the strip's end; 10 put three
 * samples and the end in one slice, 5,357 cycles at 250 RGBCCT) */
/* (sbatch 8: 12 drops drawn a slice reached 4,886 cycles at 800 px, all-max) */
static const b3_spec_t RG = { 256, 64, 64, 2, 12, 8, 1, 3, 8, 8, 6, 0, rg_want, rg_prime, rg_tick, rg_pass, rg_tick_end, 0, rg_clear, rg_splat };
B3_DESC(rain_on_glass, RG, rg_render)

/* ------------------------------------------------------------------------------- 50 Koi Pond
 * A dim pond (c2, a slow shimmer): up to six koi (intensity 1 .. 6), each a short soft body (custom1: length) in two
 * palette colours with patches (Koi palette: orange, white, gold, black). They glide, pause, flick, and turn (the body
 * folds through itself: shorter, then out the other way); now and then one rises (brighter) and sends a ripple ring
 * both ways along the strip (custom2: ripples). Fish colours keep their neutral share (white koi use W / WW + CW). */
#define KP_RIPN (H_E + 0u)             /* ripples sent (the next slot) */
#define KP_RIP  (H_E + 1u)             /* six ripples: origin (Q16), the tick it started */
#define KP_NF   (H_E + 13u)            /* fish in the frame table */
#define KP_FT   256u                   /* the frame table: 6 fish x 8 words, then 6 ripples x 2 words */

static uint32_t kp_want(fxv_ctx_t *x, uint32_t *h)
{
    const uint32_t f = 1u + (x->intensity * 5u + 127u) / 255u, lim = h[H_L] / 20u;
    x->u[0] = (4u << 16) + ((x->custom1 * 14u) << 16) / 255u;   /* body length, Q16 px */
    return f < lim ? f : lim < 1u ? 1u : lim;
}

static void kp_spawn(fxv_ctx_t *x, const uint32_t *h, fxv_particle_t *p, uint32_t k, uint32_t salt)
{
    const uint32_t r = b3_rnd(x, k, salt), r2 = b3_rnd(x, k, salt + 1u);
    p->x = (int32_t)((r >> 16) * h[H_L]);
    p->v = 0;
    p->age = (uint16_t)(60u + (r & 255u));               /* the state's ticks left */
    p->life = 0x8000u;                                   /* (bits 0-14: rising ticks) */
    p->pal = (uint8_t)r2;                                /* palette position */
    p->phase = (uint8_t)((r2 >> 8) & 1u ? 127 : -127);   /* facing */
    p->flags = 0;
    p->kind = (uint8_t)((r2 >> 16) & 0xF8u);             /* bits 0-1 state, 2 turning toward +, 3-4 kind, 5-7 size */
}

static void kp_prime(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    kp_spawn(x, h, &x->p[k], k, 8u);
}

static uint32_t kp_pass(fxv_ctx_t *x, uint32_t *h, uint32_t pass, uint32_t k)
{
    fxv_particle_t *p = &x->p[k];
    const uint32_t keep = k < h[H_N], r = b3_rnd(x, k, 2u);
    const int32_t L16 = (int32_t)(h[H_L] << 16), spd = 9000 + (int32_t)(p->kind >> 5) * 900;
    const int32_t mg = (int32_t)((x->u[0] >> 8) * (200u + (p->kind >> 5) * 16u)) + 4 * 65536;   /* body + 4 px */
    int32_t f = (int8_t)p->phase, dir = f >= 0 ? 1 : -1, tgt = 0;
    uint32_t st = p->kind & 3u, rise = p->life & 0x7FFFu, gain = 4u;
    (void)pass;
    if (p->life == 0u) {
        if (!keep) return 1u;
        kp_spawn(x, h, p, k, 0);
        f = (int8_t)p->phase;
        dir = f >= 0 ? 1 : -1;
        st = 0;
    }
    if (!b3_fade(p, keep, 3u)) return 1u;
    if (st != 3u && ((dir > 0 && p->x > L16 - mg) || (dir < 0 && p->x < mg))) {
        st = 3;                                          /* the pond's edge: turn */
        p->kind = (uint8_t)((p->kind & ~4u) | (dir < 0 ? 4u : 0u));
    }
    switch (st) {
    case 0:                                              /* glide */
        tgt = dir * spd;
        if (--p->age == 0u) {
            const uint32_t c = r & 255u;
            if (c < 140u) { st = 0; p->age = (uint16_t)(100u + ((r >> 8) & 255u)); }
            else if (c < 205u) { st = 1; p->age = (uint16_t)(50u + ((r >> 8) & 127u)); }
            else if (c < 235u) { st = 2; p->age = 15; }
            else { st = 3; p->kind = (uint8_t)((p->kind & ~4u) | (dir < 0 ? 4u : 0u)); }
        }
        break;
    case 1:                                              /* pause */
        if (--p->age == 0u) {
            if ((r & 255u) < 100u) { st = 3; p->kind = (uint8_t)((p->kind & ~4u) | (dir < 0 ? 4u : 0u)); }
            else { st = 0; p->age = (uint16_t)(100u + ((r >> 8) & 255u)); }
        }
        break;
    case 2:                                              /* a flick of the tail */
        tgt = dir * spd * 3;
        gain = 2;
        if (--p->age == 0u) { st = 0; p->age = (uint16_t)(80u + ((r >> 8) & 127u)); }
        break;
    default:                                             /* turn: the facing swings through 0 */
        f += (p->kind & 4u) ? 6 : -6;
        if (f >= 127 || f <= -127) {
            f = f > 0 ? 127 : -127;
            st = 0;
            p->age = (uint16_t)(100u + ((r >> 8) & 255u));
        }
        break;
    }
    p->v += (tgt - p->v) >> gain;
    p->x += p->v;
    if (p->x < 0) p->x = 0;
    if (p->x >= L16) p->x = L16 - 1;
    p->phase = (uint8_t)f;
    p->kind = (uint8_t)((p->kind & ~3u) | st);
    if (rise == 0u) {                                    /* now and then it rises */
        if (((r >> 16) & 1023u) == 0u) rise = 1;
    } else {
        rise++;
        if (rise == 50u && x->custom2) {
            const uint32_t s = h[KP_RIPN] % 6u;
            h[KP_RIP + s * 2u] = (uint32_t)p->x;
            h[KP_RIP + s * 2u + 1u] = x->tick;
            h[KP_RIPN] = s + 1u;
        }
        if (rise >= 100u) rise = 0;
    }
    p->life = (uint16_t)(0x8000u | rise);
    return 1u;
}

/* the frame table: each fish's head, body (signed: it trails behind the facing), 1 / length, two colours with their
 * neutral share, and brightness; each ripple's origin, radius and strength */
static void kp_post(fxv_ctx_t *x, uint32_t *h)
{
    int32_t *t = (int32_t *)(x->arena + KP_FT);
    uint32_t k, nf = 0, rr;
    for (k = 0; k < h[H_CNT]; k++) {
        const fxv_particle_t *p = &x->p[k];
        const int32_t f = (int8_t)p->phase, af = f < 0 ? -f : f;
        uint32_t len, q, mx, mn, j, rise, br;
        fxv_px_t c[2];
        if (p->life == 0u) continue;
        len = (x->u[0] >> 8) * (200u + (p->kind >> 5) * 16u);            /* Q16 px (x 0.78 .. 1.22) */
        len = (len * (40u + ((uint32_t)af * 60u >> 7))) / 100u;          /* folded while turning */
        if (len < 65536u) len = 65536u;
        c[0] = fxv_pal(x, (uint32_t)p->pal * 257u);
        c[1] = fxv_pal(x, ((uint32_t)p->pal * 257u + 0x5555u) & 0xFFFFu);
        if (c[0].r < 16384u && c[0].g < 16384u && c[0].b < 16384u) {   /* a dark sample: the bright one is the body */
            const fxv_px_t tmp = c[0];
            c[0] = c[1];
            c[1] = tmp;
        }
        if (((p->kind >> 3) & 3u) == 2u) c[1] = fxv_scale_px(c[0], 6000u);  /* black patches on gold */
        for (j = 0; j < 2u; j++) {                       /* the neutral share: min / max of the channels */
            mx = c[j].r > c[j].g ? c[j].r : c[j].g;
            if (c[j].b > mx) mx = c[j].b;
            mn = c[j].r < c[j].g ? c[j].r : c[j].g;
            if (c[j].b < mn) mn = c[j].b;
            q = mx ? mn * 255u / mx : 0u;
            c[j].q = (uint8_t)(q > 64u ? q : 0u);
            c[j].k = 140;
        }
        rise = p->life & 0x7FFFu;
        br = 40000u + (rise ? ((uint32_t)fxv_sin16(rise * 327u) * 25535u) >> 15 : 0u);
        br = (br * ((uint32_t)p->flags + 1u)) >> 8;
        t[nf * 8u + 0u] = b3_ix(h, p);
        t[nf * 8u + 1u] = f >= 0 ? -(int32_t)len : (int32_t)len;
        t[nf * 8u + 2u] = (int32_t)((65535u << 8) / (len >> 8));
        t[nf * 8u + 3u] = (int32_t)((uint32_t)c[0].r | (uint32_t)c[0].g << 16);
        t[nf * 8u + 4u] = (int32_t)((uint32_t)c[0].b | (uint32_t)c[0].q << 16);
        t[nf * 8u + 5u] = (int32_t)((uint32_t)c[1].r | (uint32_t)c[1].g << 16);
        t[nf * 8u + 6u] = (int32_t)((uint32_t)c[1].b | (uint32_t)c[1].q << 16);
        t[nf * 8u + 7u] = (int32_t)(br | ((uint32_t)p->kind >> 3 & 3u) << 16 | (uint32_t)p->pal << 24);
        nf++;
    }
    h[KP_NF] = nf;
    for (rr = 0; rr < 6u; rr++) {                         /* ripples: 2.5 s at 1x, 6 px a second */
        const uint32_t age = x->tick - h[KP_RIP + rr * 2u + 1u];
        int32_t *e = t + 48 + rr * 2u;
        if (h[KP_RIP + rr * 2u + 1u] == 0u || age >= 125u || x->custom2 == 0u) {
            e[0] = 0;
            e[1] = 0;
            continue;
        }
        {
            const uint32_t a16 = (age << 16) + h[H_FRAC], left = 65535u - a16 / 125u;
            const uint32_t amp = (((left * left) >> 16) * x->custom2) / 255u;
            e[0] = (int32_t)h[KP_RIP + rr * 2u];
            e[1] = (int32_t)((((a16 >> 8) * 31u) & 0xFFFFFu) | (amp >> 4) << 20);   /* radius Q16 (0.12 px a tick) */
        }
    }
}

static fxv_px_t kp_col(int32_t a, int32_t b)
{
    fxv_px_t c;
    c.r = (uint16_t)a;
    c.g = (uint16_t)((uint32_t)a >> 16);
    c.b = (uint16_t)b;
    c.q = (uint8_t)((uint32_t)b >> 16);
    c.k = 140;
    return c;
}

static void kp_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b3_h(x), nf = h[KP_NF];
    const int32_t *t = (const int32_t *)(x->arena + KP_FT);
    const fxv_px_t water = fxv_rgb16(x->c[1]), glint = fxv_kelvin16(150);
    uint32_t i, vp = x->vbase + from, e;
    for (i = 0; i < count; i++, vp++) {
        const int32_t pc = (int32_t)(vp << 16) + 32768;
        const int32_t sh = (fxv_sin16(vp * 1100u + x->et * 2u) * fxv_sin16(vp * 470u - x->et * 3u)) >> 15;
        fxv_px_t c = fxv_scale_px(water, (uint32_t)(57672 + ((sh * 7863) >> 15)));
        if (vp >= 32768u) { o[i] = c; continue; }
        for (e = 0; e < nf; e++) {                       /* the fish (later ones over earlier ones) */
            const int32_t *f = t + e * 8u, len = f[1];
            const int32_t d = len < 0 ? f[0] - pc : pc - f[0], al = len < 0 ? -len : len;
            uint32_t s, a;
            if (d < -65536 || d >= al) continue;
            if (d < 0) {
                a = (fxv_smooth16((uint32_t)(65536 + d)) * 52428u) >> 16;     /* the rounded nose */
                s = 0;
            } else {                                     /* head (0) .. tail (65535): full, then tapering */
                s = (((uint32_t)d >> 8) * (uint32_t)f[2]) >> 8;
                if (s > 65535u) s = 65535u;
                if (s < 7864u) a = 52428u + ((fxv_smooth16(s * 8u) * 13107u) >> 16);
                else if (s < 36044u) a = 65535u;
                else {
                    const uint32_t u = ((s - 36044u) * 569u) >> 8;
                    a = 65535u - ((fxv_smooth16(u > 65535u ? 65535u : u) * 45875u) >> 16);
                }
                if (s > 55705u) {
                    const uint32_t u = ((65535u - s) * 1707u) >> 8;
                    a = (a * fxv_smooth16(u > 65535u ? 65535u : u)) >> 16;
                }
            }
            {
                const uint32_t meta = (uint32_t)f[7], kind = (meta >> 16) & 3u;
                const int32_t ps = fxv_sin16(s + (s >> 1) + (meta >> 24) * 257u);
                const int32_t th = kind == 1u ? -9000 : 9000;
                uint32_t m = ps > th ? (uint32_t)(ps - th) * 6u : 0u;
                fxv_px_t fc;
                if (m > 65535u) m = 65535u;
                fc = fxv_blend16(kp_col(f[3], f[4]), kp_col(f[5], f[6]), m);
                fc = fxv_scale_px(fc, meta & 0xFFFFu);
                c = fxv_blend16(c, fc, a);
            }
        }
        for (e = 0; e < 6u; e++) {                       /* ripple rings over everything */
            const int32_t *rp = t + 48 + e * 2u;
            const uint32_t amp = ((uint32_t)rp[1] >> 20) << 4, rad = (uint32_t)rp[1] & 0xFFFFFu;
            int32_t dd;
            uint32_t ad;
            if (amp == 0u) continue;
            dd = pc - rp[0];
            ad = (uint32_t)(dd < 0 ? -dd : dd);
            ad = ad > rad ? ad - rad : rad - ad;
            if (ad < 91750u) c = fxv_add_px(c, fxv_scale_px(glint, ((((91750u - ad) * 182u >> 8) * amp >> 16) * 22000u) >> 16));
        }
        o[i] = c;
    }
}

static const b3_spec_t KP = { 256, 240, 6, 1, 6, 6, 0, 0, 128, 0, 0, 0, kp_want, kp_prime, 0, kp_pass, 0, kp_post, 0, 0 };
B3_DESC(koi_pond, KP, kp_render)

/* ------------------------------------------------------------------------------- 53 Dust Motes
 * A warm beam of light (c1, natural white at c1's temperature: WW / W on white strips) with soft edges (custom1: its
 * width) sways slowly along the strip (custom2: how far). Dust motes (intensity) float on slow air currents (noise)
 * with a little Brownian jitter; each turns as it floats and glints (c2, natural white) when it faces the light, so
 * they show only in the beam and sparkle most in its brightest part. */
static uint32_t dm_want(fxv_ctx_t *x, uint32_t *h)
{
    const uint32_t L = h[H_L], et = x->et, amp = x->custom2 * L / 850u;
    const int32_t s = fxv_sin16((et * 71u) >> 6) + (fxv_sin16((et * 28u) >> 6) >> 2);   /* ~59 s and ~150 s at 1x */
    uint32_t hw = L * (13u + ((x->custom1 * 102u) >> 8));                               /* half width, Q8 px */
    if (hw < 512u) hw = 512u;
    x->u[0] = (L << 7) + (uint32_t)((int32_t)amp * (s >> 7));                           /* the centre, Q8 px */
    x->u[1] = (hw * 9u) >> 4;                                                            /* a flat middle */
    x->u[2] = hw - x->u[1];                                                              /* the soft edge */
    x->u[3] = (65535u << 8) / x->u[2];
    x->u[4] = 40000u + ((uint32_t)(fxv_sin16(et * 3u) + 32767) * 3000u >> 16);          /* it breathes a little */
    x->u[5] = b3_k_of(x->c[0]) | b3_k_of(x->c[1]) << 8;
    if (!x->pal_own) {                                   /* palette: across the beam, sliding 3/4 px either way (~40 s) */
        x->u[6] = (32767u << 12) / hw;
        x->u[7] = (uint32_t)((fxv_sin16((et * 13u) >> 3) * (int32_t)((192u * x->u[6]) >> 12)) >> 15);
    }
    return b3_want(L, x->intensity, 900u);
}

static uint32_t dm_beam(const fxv_ctx_t *x, uint32_t p8)   /* the beam's profile at a virtual position (Q8): 0..65535 */
{
    const uint32_t d = p8 > x->u[0] ? p8 - x->u[0] : x->u[0] - p8;
    if (d <= x->u[1]) return 65535u;
    if (d >= x->u[1] + x->u[2]) return 0u;
    return fxv_smooth16(65535u - (((d - x->u[1]) * x->u[3]) >> 8));
}

static void dm_spawn(fxv_ctx_t *x, const uint32_t *h, fxv_particle_t *p, uint32_t k, uint32_t salt)
{
    const uint32_t r = b3_rnd(x, k, salt), r2 = b3_rnd(x, k, salt + 1u);
    p->x = (int32_t)((r >> 16) * h[H_L]);
    p->v = 0;
    p->age = (uint16_t)r;                                /* its turn (the glint at 0) */
    p->life = (uint16_t)(200u + (r2 & 511u));            /* a turn in ~1.8 .. 6.5 s at 1x */
    p->pal = (uint8_t)(64u + ((r2 >> 9) & 191u));        /* size */
    p->phase = 0;                                        /* the air's pull (int8 x 32) */
    p->flags = 0;
}

static void dm_prime(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    dm_spawn(x, h, &x->p[k], k, 8u);
}

static uint32_t dm_pass(fxv_ctx_t *x, uint32_t *h, uint32_t pass, uint32_t k)
{
    fxv_particle_t *p = &x->p[k];
    const uint32_t keep = k < h[H_N];
    const int32_t L16 = (int32_t)(h[H_L] << 16);
    uint32_t r;
    (void)pass;
    if (p->life == 0u) {
        if (!keep) return 1u;
        dm_spawn(x, h, p, k, 0);
    }
    if (!b3_fade(p, keep, 4u)) return 1u;
    if (((x->tick + k) & 7u) == 0u) {                    /* the air current where it floats, every 8 ticks */
        const int32_t a = fxv_noise1_q15(x->seed ^ 0x53u, (uint32_t)p->x / 60u, x->tick * 40u, 1);
        p->phase = (uint8_t)((a * 123) >> 15);
    }
    r = b3_rnd(x, k, 2u);
    p->v += ((((int32_t)(int8_t)p->phase * 32) - p->v) >> 5) + (((int32_t)(r >> 16) - 32768) >> 7);
    p->v = b3_clampi(p->v, 8000);
    p->x += p->v;
    if (p->x < 0) p->x += L16;
    else if (p->x >= L16) p->x -= L16;
    p->age = (uint16_t)(p->age + p->life);
    return 1u;
}

static void dm_splat(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    const fxv_particle_t *p = &x->p[k];
    const uint32_t g = p->age, dg = g < 32768u ? g : 65536u - g;
    uint32_t pr, e, a;
    int32_t c;
    if (p->life == 0u) return;
    c = b3_ix(h, p);
    pr = dm_beam(x, c < 0 ? 0u : (uint32_t)c >> 8) + 2000u;
    if (pr > 65535u) pr = 65535u;
    e = dg < 3120u ? fxv_smooth16(65535u - dg * 21u) : 0u;              /* the glint: facing the light */
    a = (pr * (24000u + ((e * 41535u) >> 16))) >> 16;
    a = (a * p->pal) >> 8;
    a = (a * ((uint32_t)p->flags + 1u)) >> 8;
    b3_splat(b3_pl(x, 0), x, c, 45875u, a);
}

static void dm_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint16_t *pl = b3_pl(x, 0) + from;
    uint32_t i, p8 = (x->vbase + from) * 256u + 128u;
    if (x->pal_own) {
        const fxv_px_t beam = b3_white(fxv_rgb16(x->c[0]), x->u[5] & 255u), dust = b3_white(fxv_rgb16(x->c[1]), x->u[5] >> 8);
        for (i = 0; i < count; i++, p8 += 256u) {
            fxv_px_t c = fxv_scale_px(beam, 2600u + ((dm_beam(x, p8) * x->u[4]) >> 16));
            if (pl[i]) c = fxv_add_px(c, fxv_scale_px(dust, pl[i]));
            o[i] = c;
        }
    } else {
        /* palette: the place across the beam (one edge to the other) picks the colour, a stained-glass beam that sways
         * with its colours; the colours slide slowly by up to 3/4 px either way, so a narrow, still beam shows every
         * entry too; the motes glint in the beam's colour where they float; the beam's light profile stays */
        const uint32_t hw = x->u[1] + x->u[2];
        for (i = 0; i < count; i++, p8 += 256u) {
            const uint32_t ad = p8 > x->u[0] ? p8 - x->u[0] : x->u[0] - p8, dq = ((ad < hw ? ad : hw) * x->u[6]) >> 12;
            int32_t v = (int32_t)(p8 > x->u[0] ? 32768u + dq : 32768u - dq) + (int32_t)x->u[7];   /* dq <= 32767 */
            fxv_px_t pc, c;
            if (x->pal_wrap) v &= 0xFFFF;
            else v = v < 0 ? 0 : v > 65535 ? 65535 : v;
            pc = fxv_pal(x, (uint32_t)v);
            c = fxv_scale_px(pc, 2600u + ((dm_beam(x, p8) * x->u[4]) >> 16));
            if (pl[i]) c = fxv_add_px(c, fxv_scale_px(pc, pl[i]));
            o[i] = c;
        }
    }
}

static const b3_spec_t DM = { 256, 0, 96, 1, 16, 16, 0, 1, 128, 0, 0, 0, dm_want, dm_prime, 0, dm_pass, 0, 0, 0, dm_splat };
B3_DESC(dust_motes, DM, dm_render)

/* ------------------------------------------------------------------------------- 56 Petal Fall
 * Petals (palette colours; Rose by default, Autumn makes it leaves) drift from the strip's start (the top) to its end,
 * fluttering (custom1: they surge and hover) and turning: dimmer edge-on, brighter flat, a short natural-white glint
 * when one faces the light. custom2: they settle into a pile at the end (0: they fall off); a full pile is blown
 * away softly and starts again. */
#define PF_LVL   (H_E + 0u)            /* the pile's height, Q8 px */
#define PF_SLOTS (H_E + 1u)            /* petals in it (2 px each) */
#define PF_ALPHA (H_E + 2u)            /* its fade (being blown away) */
#define PF_BLOW  (H_E + 3u)
#define PF_RING  (H_E + 4u)            /* 64 petal colours, bytes (16 words) */

static uint32_t pf_want(fxv_ctx_t *x, uint32_t *h)
{
    const uint32_t mx = (x->custom2 * h[H_L] * 64u) >> 8;
    x->u[0] = mx < 30720u ? mx : 30720u;                 /* the full pile: a quarter of the strip, at most 120 px */
    x->u[1] = (x->custom1 * 243u) >> 8;                  /* flutter, Q8 (up to 0.95) */
    return b3_want(h[H_L], x->intensity, 2400u);
}

static void pf_spawn(fxv_ctx_t *x, fxv_particle_t *p, uint32_t k, int32_t x0, uint32_t salt)
{
    const uint32_t r = b3_rnd(x, k, salt);
    p->x = x0;
    p->v = 0;
    p->age = (uint16_t)r;                                /* the turn */
    p->life = (uint16_t)((r >> 16) | 1u);                /* the flutter's phase (odd: never 0) */
    p->pal = (uint8_t)(r >> 8);                          /* colour */
    p->phase = (uint8_t)(r >> 24);                       /* how fast it turns */
    p->flags = 0;
    p->kind = (uint8_t)(r >> 16);                        /* size */
}

static void pf_prime(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    pf_spawn(x, &x->p[k], k, (int32_t)((b3_rnd(x, k, 9u) >> 16) * h[H_L]), 8u);
}

static void pf_tick(fxv_ctx_t *x, uint32_t *h)
{
    if (x->custom2 == 0u) {
        h[PF_LVL] = 0;
        h[PF_SLOTS] = 0;
        h[PF_BLOW] = 0;
        h[PF_ALPHA] = 65535u;
        return;
    }
    if (h[PF_ALPHA] == 0u && !h[PF_BLOW]) h[PF_ALPHA] = 65535u;
    if (h[PF_BLOW]) {                                    /* blown away: it fades, then a new pile starts */
        if (h[PF_ALPHA] > 328u) {
            h[PF_ALPHA] -= 328u;
        } else {
            h[PF_LVL] = 0;
            h[PF_SLOTS] = 0;
            h[PF_BLOW] = 0;
            h[PF_ALPHA] = 65535u;
        }
    } else if (h[PF_LVL] >= x->u[0]) {
        h[PF_BLOW] = 1;
    }
}

static uint32_t pf_pass(fxv_ctx_t *x, uint32_t *h, uint32_t pass, uint32_t k)
{
    fxv_particle_t *p = &x->p[k];
    const uint32_t keep = k < h[H_N];
    const int32_t L16 = (int32_t)(h[H_L] << 16);
    uint32_t fl;
    int32_t vt, v;
    (void)pass;
    if (p->life == 0u) {
        uint32_t r;
        if (!keep) return 1u;
        r = b3_rnd(x, k, 0);
        if ((r & 15u) != 0u) return 1u;                     /* a new petal on 1 tick in 16: a steady drift */
        pf_spawn(x, p, k, -(int32_t)((r >> 16) & 0x1FFFFu), 1u);
    }
    if (!b3_fade(p, keep, 6u)) return 1u;
    fl = ((uint32_t)p->life + 600u + ((uint32_t)p->kind << 1)) | 1u;
    p->life = (uint16_t)fl;
    p->age = (uint16_t)(p->age + 350u + p->phase * 2u + (uint32_t)((fxv_sin16(fl) + 32768) >> 8));
    vt = 10486 + (int32_t)p->kind * 40;                  /* 0.16 .. 0.32 px a tick */
    v = vt + ((((vt * (int32_t)x->u[1]) >> 8) * fxv_sin16(fl)) >> 15);
    p->v = v;
    p->x += v;
    if (x->custom2 && !h[PF_BLOW] && p->x >= L16 - (int32_t)(h[PF_LVL] << 8)) {   /* it settles on the pile */
        const uint32_t s = h[PF_SLOTS] & 63u, w = PF_RING + (s >> 2), sh = (s & 3u) * 8u;
        h[w] = (h[w] & ~(255u << sh)) | (uint32_t)p->pal << sh;
        h[PF_SLOTS]++;
        h[PF_LVL] += 512u;
        p->life = 0;
        p->flags = 0;
        return 1u;
    }
    if (p->x >= L16 + 131072) {
        p->life = 0;
        p->flags = 0;
    }
    return 1u;
}

static void pf_splat(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    const fxv_particle_t *p = &x->p[k];
    int32_t cs;
    uint32_t ac, b, g, pres;
    fxv_px_t col;
    if (p->life == 0u) return;
    cs = fxv_cos16(p->age);
    ac = (uint32_t)(cs < 0 ? -cs : cs);
    b = 22938u + ((ac * 42597u) >> 15);                  /* edge-on 0.35 .. flat 1.0 */
    g = cs > 30474 ? (uint32_t)(cs - 30474) * 28u : 0u;  /* facing the light */
    g = (((g * g) >> 16) * 36000u) >> 16;
    pres = (uint32_t)p->flags + 1u;
    col = fxv_scale_px(fxv_pal(x, (uint32_t)p->pal * 257u), (b * pres) >> 8);
    b3_splat4(x, 0, b3_ix(h, p), 65536u + (uint32_t)p->kind * 154u, col, (g * pres) >> 8);
}

static void pf_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b3_h(x), L = h[H_L], lvl = h[PF_LVL], al = h[PF_ALPHA];
    uint32_t i, vp = x->vbase + from;
    for (i = 0; i < count; i++, vp++) {
        fxv_px_t c = b3_px4(x, 0, from + i, 150);
        if (lvl && vp < L) {                             /* the pile at the end, the newest petals on top */
            const uint32_t e = L - 1u - vp, e8 = e << 8;
            if (e8 < lvl) {
                const uint32_t s = e >> 1, hue = (h[PF_RING + ((s & 63u) >> 2)] >> ((s & 3u) * 8u)) & 255u;
                const uint32_t part = lvl - e8 >= 256u ? 65535u : (lvl - e8) * 256u;
                const uint32_t tex = 22000u + ((fxv_hash2(x->seed, vp) & 255u) << 5);
                fxv_px_t pc = fxv_scale_px(fxv_pal(x, hue * 257u), (((tex * al) >> 16) * part) >> 16);
                c = fxv_add_px(c, pc);
            }
        }
        o[i] = c;
    }
}

static const b3_spec_t PF = { 256, 0, 40, 1, 12, 8, 0, 4, 96, 0, 0, 0, pf_want, pf_prime, pf_tick, pf_pass, 0, 0, 0, pf_splat };
B3_DESC(petal_fall, PF, pf_render)

/* ------------------------------------------------------------------------------- 57 Starlings
 * A murmuration against a dusk sky (c2 with a horizon glow, natural white at c2's temperature: WW / W): the birds
 * (c1, dark silhouettes that take the light away) follow a leader that wanders the strip; each steers by its ordered
 * neighbours only (cohesion: custom1, alignment, separation; an insertion sort keeps the order, no all-pairs). The
 * cohesion breathes (the flock stretches thin, bunches up) and now and then half the flock follows a second leader
 * (it splits, then joins again). custom2: the sky's glow. */
#define SL_T    (H_E + 0u)             /* the leaders (Q16), the cohesion of this tick */
#define SL_T2   (H_E + 1u)
#define SL_COH  (H_E + 2u)

static uint8_t *sl_ord(const fxv_ctx_t *x)
{
    return x->arena + 256u;
}

static int16_t *sl_dv(const fxv_ctx_t *x)
{
    return (int16_t *)(x->arena + 256u + 160u);
}

static uint32_t sl_want(fxv_ctx_t *x, uint32_t *h)
{
    x->u[0] = x->custom2 * 210u + 12000u > 65535u ? 65535u : x->custom2 * 210u + 12000u;
    x->u[1] = b3_k_of(x->c[1]);
    x->u[2] = h[H_L] * 159u;                             /* the horizon glow's centre, Q8 (0.62) */
    x->u[3] = (65535u << 8) / ((h[H_L] * 192u) | 1u);    /* 1 / its reach (0.75 of the strip, Q8) */
    if (!x->pal_own) x->u[7] = b3_pstep(h[H_L]);
    return b3_want(h[H_L], x->intensity, 740u);
}

static void sl_spawn(fxv_ctx_t *x, const uint32_t *h, fxv_particle_t *p, uint32_t k, int32_t x0, uint32_t salt)
{
    const uint32_t r = b3_rnd(x, k, salt);
    const int32_t L16 = (int32_t)(h[H_L] << 16);
    int32_t px = x0 + (((int32_t)(r >> 16) - 32768) << 4);
    if (px < 0) px = 0;
    if (px >= L16) px = L16 - 1;
    p->x = px;
    p->v = 0;
    p->age = 0;
    p->life = 1;
    p->pal = (uint8_t)r;
    p->phase = (uint8_t)((r >> 8) & 1u);                 /* which leader it follows when the flock splits */
    p->flags = 0;
    p->kind = 0;
}

static void sl_prime(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    sl_spawn(x, h, &x->p[k], k, (int32_t)(h[H_L] << 15), 8u);
}

static void sl_tick(fxv_ctx_t *x, uint32_t *h)
{
    const uint32_t L = h[H_L], ph = (x->tick + (x->seed >> 21)) & 2047u, cyc = (x->tick + (x->seed >> 21)) >> 11;
    const int32_t n = fxv_noise1_q15(x->seed ^ 0x57u, x->tick * 50u, 0x5157u, 2);
    const int32_t nc = fxv_noise1_q15(x->seed ^ 0x75u, x->tick * 30u, 0x7557u, 1);
    int32_t t = (int32_t)(L << 15) + (int32_t)L * ((n * 28835) >> 15), t2 = t;
    uint32_t env = 0;
    b3_order_init(h, sl_ord(x));
    if ((fxv_hash2(x->seed, cyc) & 3u) != 0u) {          /* most cycles: a split, half the flock away and back */
        if (ph >= 700u && ph < 900u) env = fxv_smooth16((ph - 700u) * 327u);
        else if (ph >= 900u && ph < 1300u) env = 65535u;
        else if (ph >= 1300u && ph < 1500u) env = fxv_smooth16(65535u - (ph - 1300u) * 327u);
    }
    if (env) {
        const int32_t dd = (int32_t)(L * 56u * (env >> 8));         /* up to 0.22 of the strip, Q16 */
        t2 = t > (int32_t)(L << 15) ? t - dd : t + dd;
    }
    h[SL_T] = (uint32_t)t;
    h[SL_T2] = (uint32_t)t2;
    h[SL_COH] = ((200u + x->custom1 * 4u) * (29491u + ((uint32_t)((nc + 32768) * 9) >> 4))) >> 16;
}

static uint32_t sl_pass(fxv_ctx_t *x, uint32_t *h, uint32_t pass, uint32_t k)
{
    uint8_t *ord = sl_ord(x);
    int16_t *dv = sl_dv(x);
    if (pass != 1u && (x->tick & 1u)) return 1u;        /* steering (and its sort) every other tick */
    if (pass == 0u) {                                    /* steering, from the ordered neighbours (old positions) */
        const uint32_t n = h[H_SORTN];
        uint32_t kk, cnt = 0, d;
        int32_t sx = 0, sv = 0, acc = 0, dl;
        const fxv_particle_t *p;
        static const uint16_t rcp[5] = { 0, 256, 128, 85, 64 };
        if (k >= n) return 1u;
        kk = ord[k];
        p = &x->p[kk];
        if (p->life == 0u) {
            dv[kk] = 0;
            return 1u;
        }
        for (d = 0; d < 4u; d++) {
            const int32_t j = (int32_t)k + (d < 2u ? (int32_t)d - 2 : (int32_t)d - 1);
            const fxv_particle_t *q;
            int32_t dx;
            if (j < 0 || j >= (int32_t)n) continue;
            q = &x->p[ord[j]];
            if (q->life == 0u) continue;
            dx = b3_clampi(q->x - p->x, 40 << 16);
            sx += dx >> 4;
            sv += q->v >> 4;
            cnt++;
            if ((d == 1u || d == 2u) && (dx < 0 ? -dx : dx) < 22938) {   /* too close: apart */
                acc += dx < 0 ? (22938 + dx) >> 3 : -((22938 - dx) >> 3);
            }
        }
        if (cnt) {
            const int32_t mx = (sx * rcp[cnt]) >> 8, mv = (sv * rcp[cnt]) >> 8;
            acc += (mx * (int32_t)h[SL_COH]) >> 12;
            acc += ((mv - (p->v >> 4)) * 3932) >> 12;
        }
        dl = b3_clampi((int32_t)((p->phase & 1u) ? h[SL_T2] : h[SL_T]) - p->x, 60 << 16);
        acc += ((dl >> 4) * 20) >> 12;
        acc += ((int32_t)(b3_rnd(x, kk, 3u) >> 16) - 32768) >> 6;
        dv[kk] = (int16_t)b3_clampi(acc, 3000);
        return 1u;
    }
    if (pass == 1u) {                                    /* fly */
        fxv_particle_t *p = &x->p[k];
        const uint32_t keep = k < h[H_N];
        const int32_t L16 = (int32_t)(h[H_L] << 16);
        if (p->life == 0u) {
            if (!keep) return 1u;
            sl_spawn(x, h, p, k, (int32_t)h[SL_T], 0);
            dv[k] = 0;
        }
        if (!b3_fade(p, keep, 5u)) return 1u;
        p->v += dv[k];
        p->v -= p->v >> 6;
        p->v = b3_clampi(p->v, 131072);
        p->x += p->v;
        if (p->x < 0) {
            p->x = -p->x;
            p->v = -p->v;
        }
        if (p->x >= L16) {
            p->x = L16 - 1 - (p->x - L16);
            p->v = -p->v;
        }
        return 1u;
    }
    return k < h[H_SORTN] ? b3_sort_step(x->p, ord, k) : 1u;
}

static void sl_splat(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    const fxv_particle_t *p = &x->p[k];
    if (p->life == 0u) return;
    b3_splat(b3_pl(x, 0), x, b3_ix(h, p), 52429u, (19661u * ((uint32_t)p->flags + 1u)) >> 8);
}

static void sl_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint16_t *pl = b3_pl(x, 0) + from;
    uint32_t i, p8 = (x->vbase + from) * 256u + 128u;
    if (x->pal_own) {
        const fxv_px_t sky = b3_white(fxv_rgb16(x->c[1]), x->u[1]), bird = fxv_rgb16(x->c[0]);
        for (i = 0; i < count; i++, p8 += 256u) {
            const uint32_t d = p8 > x->u[2] ? p8 - x->u[2] : x->u[2] - p8;
            uint32_t lin = (d * x->u[3]) >> 8, g, a;
            fxv_px_t c;
            if (lin > 65535u) lin = 65535u;
            g = 26214u + ((fxv_smooth16(65535u - lin) * 39321u) >> 16);
            c = fxv_scale_px(sky, (g * x->u[0]) >> 16);
            a = (uint32_t)pl[i] * 3u;
            if (a) c = fxv_blend16(c, bird, a > 65535u ? 65535u : a);
            o[i] = c;
        }
    } else {
        /* palette: the place in the sky (the strip) picks the colour, the horizon glow keeps its light; the birds stay
         * dark silhouettes: the sky's colour there at the default birds' darkness */
        const uint32_t st = x->u[7];
        uint32_t a8 = (x->vbase + from) * st + (st >> 1);
        for (i = 0; i < count; i++, p8 += 256u, a8 += st) {
            const uint32_t d = p8 > x->u[2] ? p8 - x->u[2] : x->u[2] - p8;
            const fxv_px_t sky = fxv_pal(x, a8 >> 8);
            uint32_t lin = (d * x->u[3]) >> 8, g, a;
            fxv_px_t c;
            if (lin > 65535u) lin = 65535u;
            g = 26214u + ((fxv_smooth16(65535u - lin) * 39321u) >> 16);
            c = fxv_scale_px(sky, (g * x->u[0]) >> 16);
            a = (uint32_t)pl[i] * 3u;
            if (a) c = fxv_blend16(c, fxv_scale_px(sky, 3084u), a > 65535u ? 65535u : a);   /* 12 / 255 */
            o[i] = c;
        }
    }
}

static const b3_spec_t SL = { 256, 480, 160, 3, 12, 16, 0, 1, 128, 8, 0, 0, sl_want, sl_prime, sl_tick, sl_pass, 0, 0, 0, sl_splat };
B3_DESC(starlings, SL, sl_render)

/* ------------------------------------------------------------------------------- 107 Snowfall
 * Snowflakes (c1, natural white at c1's temperature: W / CW) drift from the strip's start down to its end over the
 * night (c2): each sways a little, gusts (custom1: wind, noise in time) push them along, small flakes more. They pile
 * into a bank at the end (custom2: how high) that glitters; a full bank melts away and the snow starts a new one. */
#define SF_LVL  (H_E + 0u)             /* the bank's height, Q8 px */
#define SF_MELT (H_E + 1u)
#define SF_GUST (H_E + 2u)             /* this tick's gust, Q16 px a tick (signed) */

static uint32_t sf_want(fxv_ctx_t *x, uint32_t *h)
{
    x->u[0] = b3_k_of(x->c[0]);
    x->u[1] = (x->custom2 * h[H_L] * 64u) >> 8;          /* a full bank: up to a quarter of the strip, Q8 */
    if (!x->pal_own) x->u[7] = b3_pstep(h[H_L]);
    return b3_want(h[H_L], x->intensity, 680u);
}

static void sf_spawn(fxv_ctx_t *x, fxv_particle_t *p, uint32_t k, int32_t x0, uint32_t salt)
{
    const uint32_t r = b3_rnd(x, k, salt);
    p->x = x0;
    p->v = 0;
    p->age = (uint16_t)r;                                /* sway */
    p->life = 1;
    p->pal = (uint8_t)(r >> 16);                         /* size */
    p->phase = (uint8_t)(r >> 24);                       /* sway rate */
    p->flags = 0;
    p->kind = 0;
}

static void sf_prime(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    sf_spawn(x, &x->p[k], k, (int32_t)((b3_rnd(x, k, 9u) >> 16) * h[H_L]), 8u);
}

static void sf_tick(fxv_ctx_t *x, uint32_t *h)
{
    const int32_t n = fxv_noise1_q15(x->seed ^ 0x107u, x->tick * 220u, 0x1234u, 2);
    h[SF_GUST] = (uint32_t)(((n + 13000) * (int32_t)x->custom1) >> 8);   /* mostly along the fall */
    if (x->custom2 == 0u) {
        h[SF_LVL] = 0;
        h[SF_MELT] = 0;
    } else if (h[SF_MELT]) {                             /* melting: ~25 s at 1x */
        const uint32_t dec = x->u[1] / 1250u + 1u;
        h[SF_LVL] = h[SF_LVL] > dec ? h[SF_LVL] - dec : 0u;
        if (h[SF_LVL] == 0u) h[SF_MELT] = 0;
    } else if (h[SF_LVL] >= x->u[1]) {
        h[SF_MELT] = 1;
    }
}

static uint32_t sf_pass(fxv_ctx_t *x, uint32_t *h, uint32_t pass, uint32_t k)
{
    fxv_particle_t *p = &x->p[k];
    const uint32_t keep = k < h[H_N];
    const int32_t L16 = (int32_t)(h[H_L] << 16);
    int32_t v;
    (void)pass;
    if (p->life == 0u) {
        uint32_t r;
        if (!keep) return 1u;
        r = b3_rnd(x, k, 0);
        if ((r & 7u) != 0u) return 1u;                      /* a new flake on 1 tick in 8 */
        sf_spawn(x, p, k, -(int32_t)((r >> 16) & 0x1FFFFu), 1u);
    }
    if (!b3_fade(p, keep, 8u)) return 1u;
    p->age = (uint16_t)(p->age + 300u + p->phase * 4u);
    v = 13107 + (int32_t)p->pal * 77 + (((int32_t)h[SF_GUST] * (384 - (int32_t)p->pal)) >> 8) + (fxv_sin16(p->age) >> 3);
    p->v = v;
    p->x += v;
    if (x->custom2 && p->x >= L16 - (int32_t)(h[SF_LVL] << 8)) {   /* it lands on the bank (a melting one takes none) */
        if (!h[SF_MELT]) {
            const uint32_t lv = h[SF_LVL] + 20u + ((p->pal * 60u) >> 8);
            h[SF_LVL] = lv < x->u[1] + 256u ? lv : x->u[1] + 256u;
        }
        p->life = 0;
        p->flags = 0;
        return 1u;
    }
    if (p->x >= L16 + 131072 || p->x < -(4 << 16)) {
        p->life = 0;
        p->flags = 0;
    }
    return 1u;
}

static void sf_splat(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    const fxv_particle_t *p = &x->p[k];
    uint32_t a;
    if (p->life == 0u) return;
    a = (90u + ((p->pal * 165u) >> 8)) * 220u;
    a = (a * ((uint32_t)p->flags + 1u)) >> 8;
    b3_splat(b3_pl(x, 0), x, b3_ix(h, p), 49152u, a);
}

static void sf_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b3_h(x), L = h[H_L], lvl = h[SF_LVL], melt = h[SF_MELT], st = x->u[7];
    const uint16_t *pl = b3_pl(x, 0) + from;
    fxv_px_t night = fxv_rgb16(x->c[1]), snow = b3_white(fxv_rgb16(x->c[0]), x->u[0]);
    uint32_t i, vp = x->vbase + from, a8 = vp * st + (st >> 1);
    for (i = 0; i < count; i++, vp++, a8 += st) {
        fxv_px_t c;
        if (!x->pal_own) {
            /* palette: the depth of the fall (the place on the strip) picks the colour, so a flake runs through the
             * whole palette on its way down and the bank takes the end's colours; the night is that colour at the
             * default night's darkness */
            snow = fxv_pal(x, a8 >> 8);
            night = fxv_scale_px(snow, 5140u);              /* 20 / 255 */
        }
        c = pl[i] ? fxv_add_px(night, fxv_scale_px(snow, pl[i])) : night;
        if (lvl && vp < L) {
            const uint32_t e = L - 1u - vp, e8 = e << 8;
            if (e8 < lvl) {                              /* the bank: packed snow, glittering unless it melts */
                const uint32_t hs = fxv_hash2(x->seed ^ 0x6117u, vp);
                const uint32_t part = lvl - e8 >= 256u ? 65535u : (lvl - e8) * 256u;
                uint32_t b = 40600u + ((hs & 255u) * 36u);
                if (melt) {
                    b = (b * 53740u) >> 16;
                } else {
                    const uint32_t g = (hs >> 16) + x->et * (3u + ((hs >> 8) & 7u)), gg = g & 0xFFFFu;
                    if (gg < 2048u) {
                        const uint32_t dd = gg > 1024u ? gg - 1024u : 1024u - gg, s = fxv_smooth16(65535u - (dd << 6 > 65535u ? 65535u : dd << 6));
                        b += (s * 22000u) >> 16;
                    }
                }
                c = fxv_blend16(c, fxv_scale_px(snow, b > 65535u ? 65535u : b), part);
            }
        }
        o[i] = c;
    }
}

static const b3_spec_t SF = { 256, 0, 150, 1, 24, 20, 0, 1, 128, 0, 0, 0, sf_want, sf_prime, sf_tick, sf_pass, 0, 0, 0, sf_splat };
B3_DESC(snowfall, SF, sf_render)

/* ------------------------------------------------------------------------------- 112 Champagne
 * Golden wine (c1, darker deeper down, a slow shimmer) fills the strip from its start (the bottom) to the surface
 * (custom1); above it, dim glass. Bubbles (c2, natural white at c2's temperature: W / WW; intensity: how many) rise
 * from near the bottom, speeding up and growing brighter as they rise, with a little wobble, and pop at the surface
 * in a short white sparkle (custom2) on a faint meniscus line. */
#define CH_S    (H_E + 0u)             /* the surface, Q16 px from the start */

static uint32_t ch_want(fxv_ctx_t *x, uint32_t *h)
{
    uint32_t s = (h[H_L] * (x->custom1 + 1u)) >> 8;
    if (s < 1u) s = 1u;
    h[CH_S] = s << 16;
    x->u[0] = b3_k_of(x->c[1]);
    x->u[1] = s << 8;
    x->u[2] = (65535u << 8) / (s << 8);
    return b3_want(s, x->intensity, 590u);
}

static void ch_spawn(fxv_ctx_t *x, fxv_particle_t *p, uint32_t k, int32_t x0, uint32_t salt)
{
    const uint32_t r = b3_rnd(x, k, salt);
    int32_t v = 7864 + (x0 >> 16) * 100;                 /* 6 px a second at 1x, faster higher up */
    p->x = x0;
    p->v = v > 78643 ? 78643 : v;
    p->age = (uint16_t)r;                                /* wobble */
    p->life = 1;
    p->pal = (uint8_t)(r >> 16);                         /* size */
    p->phase = (uint8_t)(r >> 24);                       /* wobble rate */
    p->flags = 0;
    p->kind = 0;                                         /* 1: a pop */
}

static void ch_prime(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    ch_spawn(x, &x->p[k], k, (int32_t)((b3_rnd(x, k, 9u) >> 16) * (h[CH_S] >> 16)), 8u);
}

static uint32_t ch_pass(fxv_ctx_t *x, uint32_t *h, uint32_t pass, uint32_t k)
{
    fxv_particle_t *p = &x->p[k];
    const uint32_t keep = k < h[H_N];
    const int32_t s16 = (int32_t)h[CH_S];
    int32_t vv;
    (void)pass;
    if (p->life == 0u) {
        uint32_t r, u2;
        if (!keep) return 1u;
        r = b3_rnd(x, k, 0);
        if ((r & 15u) != 0u) return 1u;                     /* a new bubble on 1 tick in 16 */
        u2 = ((r >> 16) * (r >> 16)) >> 16;              /* mostly near the bottom */
        ch_spawn(x, p, k, (int32_t)(u2 * (((uint32_t)s16 >> 16) * 77u >> 8)), 1u);
    }
    if (!b3_fade(p, keep, 32u)) return 1u;
    if (p->kind) {                                       /* a pop: 9 ticks of sparkle, then the record is free */
        if (++p->age >= 9u) {
            p->life = 0;
            p->flags = 0;
        }
        return 1u;
    }
    p->age = (uint16_t)(p->age + 1500u + p->phase * 8u);
    p->v += 50 + (((int32_t)p->pal * 120) >> 8);
    if (p->v > 78643) p->v = 78643;
    vv = p->v + (((p->v >> 3) * (fxv_sin16(p->age) >> 2)) >> 13);
    p->x += vv;
    if (p->x >= s16) {
        p->kind = 1;
        p->age = 0;
        p->x = s16 - (int32_t)(b3_rnd(x, k, 2u) & 0x17FFFu);
        p->v = 0;
    }
    return 1u;
}

static void ch_splat(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    const fxv_particle_t *p = &x->p[k];
    const uint32_t pres = (uint32_t)p->flags + 1u;
    uint32_t a;
    if (p->life == 0u) return;
    if (p->kind) {
        const uint32_t e = p->age < 2u ? (p->age + 1u) * 21845u : (9u - p->age) * 9362u;
        a = (((e * x->custom2) / 255u) * pres) >> 8;
        b3_splat(b3_pl(x, 1), x, p->x, 78643u + p->age * 16384u, a);
    } else {
        uint32_t rf = ((uint32_t)(p->x < 0 ? 0 : p->x) >> 8) * x->u[2] >> 8, lvl;
        if (rf > 65535u) rf = 65535u;
        lvl = 22938u + ((rf * 42597u) >> 16);
        a = (((((90u + ((p->pal * 165u) >> 8)) * 257u) * lvl) >> 16) * pres) >> 8;
        b3_splat(b3_pl(x, 0), x, b3_ix(h, p), 39322u + p->pal * 128u, a);
    }
}

static void ch_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint16_t *pb = b3_pl(x, 0) + from, *pp = b3_pl(x, 1) + from;
    fxv_px_t wine = fxv_rgb16(x->c[0]), bub = b3_white(fxv_rgb16(x->c[1]), x->u[0]);
    fxv_px_t pop = fxv_kelvin16(x->u[0] + 40u > 255u ? 255u : x->u[0] + 40u);
    const uint32_t s8 = x->u[1];
    uint32_t i, vp = x->vbase + from, p8 = vp * 256u + 128u;
    for (i = 0; i < count; i++, vp++, p8 += 256u) {
        fxv_px_t c;
        uint32_t d;
        if (!x->pal_own) {
            /* palette: the height in the glass (bottom to surface) picks the colour, so a bubble runs through the
             * palette as it rises and pops in the surface's colour; the colours rise slowly through the glass (a turn
             * in ~65 s at 1x, there and back on an open palette), so a shallow glass shows every entry too; the wine
             * is that colour at the default wine's level (darker deeper down as before), the bubbles and pops its
             * full light */
            uint32_t v = (p8 < s8 ? (p8 * x->u[2]) >> 8 : 65535u) - x->et;
            if (x->pal_wrap) v &= 0xFFFFu;
            else v = (v & 0x1FFFFu) < 65536u ? v & 0xFFFFu : 131071u - (v & 0x1FFFFu);
            bub = fxv_pal(x, v);
            pop = bub;
            wine = fxv_scale_px(bub, 30840u);               /* 120 / 255 */
        }
        if (p8 < s8) {                                   /* in the glass: deeper is darker, a slow shimmer */
            const uint32_t f = (p8 * x->u[2]) >> 8;
            c = fxv_scale_px(wine, (uint32_t)(34000 + (int32_t)((f * 24000u) >> 16) + ((fxv_sin16(vp * 1400u + x->et * 5u) * 2500) >> 15)));
        } else {
            c = fxv_scale_px(wine, 2600u);
        }
        d = p8 > s8 ? p8 - s8 : s8 - p8;
        if (d < 384u) c = fxv_add_px(c, fxv_scale_px(bub, ((384u - d) * 170u * 26214u) >> 16));   /* the surface */
        if (pb[i]) c = fxv_add_px(c, fxv_scale_px(bub, pb[i]));
        if (pp[i]) c = fxv_add_px(c, fxv_scale_px(pop, pp[i]));
        o[i] = c;
    }
}

/* (sbatch 12: 20 bubbles drawn a slice reached 4,951 cycles at 800 px, defaults, late) */
static const b3_spec_t CH = { 256, 0, 200, 1, 20, 12, 0, 2, 128, 8, 0, 0, ch_want, ch_prime, 0, ch_pass, 0, 0, 0, ch_splat };
B3_DESC(champagne, CH, ch_render)

/* ------------------------------------------------------------------------------- 75 Marble Run
 * Glass marbles (palette colours, a swirl inside that turns as they roll, a small W / CW highlight on the side of the
 * light) roll on an invisible hilly track: the slope of slow 2-octave noise (custom1: hilliness) pulls them (speed:
 * gravity), friction (custom2) slows them, they bounce off each other (equal masses, ordered neighbours) and off the
 * ends, and gather in the valleys until the whole track tilts the other way (every ~30 s at 1x). */
#define MR_TILT (H_E + 0u)             /* this tick's tilt, Q16 px a tick^2 (signed) */

static uint8_t *mr_ord(const fxv_ctx_t *x)
{
    return x->arena + 256u;
}

static uint32_t mr_r8(const fxv_particle_t *p)           /* a marble's radius, Q8 px: 1.2 .. 2.5 */
{
    return 307u + ((p->phase * 333u) >> 8);
}

static uint32_t mr_want(fxv_ctx_t *x, uint32_t *h)
{
    const uint32_t f = 2u + x->intensity * 22u / 255u, lim = h[H_L] / 10u;
    x->u[0] = 30u + x->custom2 * 3u;                     /* friction, Q16 a tick */
    if (x->intensity == 0u) return 0;
    return f < lim ? f : lim < 1u ? 1u : lim;
}

static void mr_spawn(fxv_ctx_t *x, const uint32_t *h, fxv_particle_t *p, uint32_t k, uint32_t salt)
{
    const uint32_t r = b3_rnd(x, k, salt);
    p->x = (int32_t)((r >> 16) * h[H_L]);
    p->v = 0;
    p->age = (uint16_t)r;                                /* how far it has rolled (the swirl's angle) */
    p->life = 1;
    p->pal = (uint8_t)(r >> 8);
    p->phase = (uint8_t)(r >> 24);                       /* size */
    p->flags = 0;
    p->kind = 0;                                         /* a knock's glint, ticks left */
}

static void mr_prime(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    mr_spawn(x, h, &x->p[k], k, 8u);
}

static void mr_tick(fxv_ctx_t *x, uint32_t *h)
{
    int32_t s = (fxv_sin16((x->tick + (x->seed >> 16)) * 22u) * 9) >> 2;   /* a plateaued tilt, ~30 s a cycle */
    b3_order_init(h, mr_ord(x));
    s = b3_clampi(s, 32767);
    h[MR_TILT] = (uint32_t)((s * 520) >> 15);
}

static uint32_t mr_pass(fxv_ctx_t *x, uint32_t *h, uint32_t pass, uint32_t k)
{
    fxv_particle_t *p;
    const int32_t L16 = (int32_t)(h[H_L] << 16);
    int32_t r16, np, nm, xm;
    uint32_t q;
    if (pass == 1u) {
        return k < h[H_SORTN] ? b3_sort_step(x->p, mr_ord(x), k) : 1u;
    }
    p = &x->p[k];
    if (p->life == 0u) {
        if (k >= h[H_N]) return 1u;
        mr_spawn(x, h, p, k, 0);
    }
    if (!b3_fade(p, k < h[H_N], 8u)) return 1u;
    r16 = (int32_t)(mr_r8(p) << 8);
    xm = p->x - 131072 < 0 ? 0 : p->x - 131072;
    np = fxv_noise1_q15(x->seed ^ 0x75u, (uint32_t)(p->x + 131072) / 48u, x->tick * 6u, 2);
    nm = fxv_noise1_q15(x->seed ^ 0x75u, (uint32_t)xm / 48u, x->tick * 6u, 2);
    p->v += -(((np - nm) * (int32_t)x->custom1) >> 9) + (int32_t)h[MR_TILT];
    p->v -= (p->v * (int32_t)x->u[0]) >> 16;
    p->v = b3_clampi(p->v, 131072);
    p->x += p->v;
    q = (uint32_t)(p->v < 0 ? -p->v : p->v) * 41u / mr_r8(p);
    p->age = (uint16_t)(p->v < 0 ? p->age - q : p->age + q);
    if (p->x < r16) {
        p->x = r16;
        if (p->v < 0) p->v = -(p->v >> 1);
    }
    if (p->x > L16 - r16) {
        p->x = L16 - r16;
        if (p->v > 0) p->v = -(p->v >> 1);
    }
    if (p->kind) p->kind--;
    return 1u;
}

static void mr_tick_end(fxv_ctx_t *x, uint32_t *h)       /* knocks: neighbours that overlap swap their speeds */
{
    const uint8_t *ord = mr_ord(x);
    uint32_t j;
    for (j = 0; j + 1u < h[H_SORTN]; j++) {
        fxv_particle_t *pa = &x->p[ord[j]], *pb = &x->p[ord[j + 1u]];
        int32_t gap, rs;
        if (pa->life == 0u || pb->life == 0u) break;
        gap = pb->x - pa->x;
        rs = (int32_t)((mr_r8(pa) + mr_r8(pb)) << 8);
        if (gap < rs) {
            if (pa->v > pb->v) {
                const int32_t va = pa->v, vb = pb->v;
                pa->v = (vb * 225) >> 8;
                pb->v = (va * 225) >> 8;
                if (va - vb > 20000) pa->kind = pb->kind = 8;
            }
            pa->x -= (rs - gap) >> 1;
            pb->x += (rs - gap) >> 1;
        }
    }
}

static void b3_add4(const fxv_ctx_t *x, uint32_t i, fxv_px_t c, uint32_t w)   /* add to planes 0..3 at run pixel i */
{
    uint16_t *pl = b3_pl(x, 0) + i;
    const uint32_t st = b3_h(x)[H_STRIDE];
    uint32_t v;
    v = pl[0] + (uint32_t)c.r; pl[0] = (uint16_t)(v > 65535u ? 65535u : v);
    v = pl[st] + (uint32_t)c.g; pl[st] = (uint16_t)(v > 65535u ? 65535u : v);
    v = pl[2u * st] + (uint32_t)c.b; pl[2u * st] = (uint16_t)(v > 65535u ? 65535u : v);
    v = pl[3u * st] + w; pl[3u * st] = (uint16_t)(v > 65535u ? 65535u : v);
}

static void mr_splat(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    const fxv_particle_t *p = &x->p[k];
    uint32_t r8, r, kr, sr, ks;
    int32_t c, i, lo, hi;
    fxv_px_t ca, cb;
    if (p->life == 0u || x->vbase >= 32768u) return;
    r8 = mr_r8(p);
    r = r8 << 8;
    kr = (65535u << 8) / r8;
    sr = (r * 77u) >> 8;                                 /* the highlight: 0.3 r wide, 0.35 r toward the start */
    ks = (65535u << 8) / (sr >> 8);
    ca = fxv_pal(x, (uint32_t)p->pal * 257u);
    cb = fxv_pal(x, ((uint32_t)p->pal * 257u + 0x1800u) & 0xFFFFu);
    c = b3_ix(h, p) - (int32_t)(x->vbase << 16);
    lo = (c - (int32_t)r) >> 16;
    hi = (c + (int32_t)r) >> 16;
    if (lo < 0) lo = 0;
    if (hi >= (int32_t)x->n) hi = (int32_t)x->n - 1;
    for (i = lo; i <= hi; i++) {
        const int32_t d = i * 65536 + 32768 - c, sd = d + (int32_t)((r * 90u) >> 8);
        const uint32_t ad = (uint32_t)(d < 0 ? -d : d), asd = (uint32_t)(sd < 0 ? -sd : sd);
        uint32_t body, w, spec = 0;
        fxv_px_t col;
        if (ad >= r) continue;
        body = fxv_smooth16(65535u - (((ad >> 8) * kr) >> 8));
        w = (uint32_t)(fxv_sin16(p->age + (uint32_t)((d >> 6) * 40)) + 32768);
        col = fxv_blend16(ca, cb, w > 65535u ? 65535u : w);
        col = fxv_scale_px(col, ((16384u + ((body * 49151u) >> 16)) * ((uint32_t)p->flags + 1u)) >> 8);
        if (asd < sr) spec = (fxv_smooth16(65535u - (((asd >> 8) * ks) >> 8)) * 39321u) >> 16;
        spec += (((uint32_t)p->kind * 8192u > 65535u ? 65535u : (uint32_t)p->kind * 8192u) * body) >> 17;
        b3_add4(x, (uint32_t)i, col, (spec * ((uint32_t)p->flags + 1u)) >> 8);
    }
}

static void mr_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    uint32_t i;
    for (i = 0; i < count; i++) o[i] = b3_px4(x, 0, from + i, 200);
}

static const b3_spec_t MR = { 256, 24, 24, 2, 3, 3, 0, 4, 96, 0, 0, 0, mr_want, mr_prime, mr_tick, mr_pass, mr_tick_end, 0, 0, mr_splat };
B3_DESC(marble_run, MR, mr_render)

/* ------------------------------------------------------------------------------- 90 Orbit Slingshot
 * Dim wells of light (c3) drift slowly along the strip (one per ~180 px, up to four) over space (c2). Satellites
 * (palette colours; Mulberry Milk by default; intensity: how many) circle them on eccentric orbits seen edge-on
 * (projected 1-D: the eccentric anomaly advances as 1 / (1 - e cos E), so they race through the close passes; custom1:
 * tighter orbits), and at a close pass one may transfer (custom2: how often) to a neighbouring well or a new orbit:
 * an explicit blend between the two orbits, never a singular force. Fast satellites widen (no aliasing) and brighten;
 * a little natural white in their cores. */
#define OS_W    (H_E + 0u)             /* wells, their centres (Q16) */
#define OS_C    (H_E + 1u)

static uint32_t os_want(fxv_ctx_t *x, uint32_t *h)
{
    const uint32_t L = h[H_L];
    uint32_t w = L / 180u, amp4, j;
    if (w < 1u) w = 1u;
    if (w > 4u) w = 4u;
    h[OS_W] = w;
    amp4 = (L * 225u) / (64u * w);
    for (j = 0; j < w; j++) {                            /* the wells drift slowly about their places */
        const int32_t n = fxv_noise1_q15(x->seed + j * 0x9E37u, x->tick * 25u, 0x90u, 1);
        const int32_t base4 = (int32_t)(((2u * j + 1u) * L * 16u) / (2u * w));
        h[OS_C + j] = (uint32_t)((base4 + (((n >> 3) * (int32_t)amp4) >> 12)) * 4096);
    }
    x->u[0] = b3_k_of(x->c[2]);
    for (j = 0; j < 4u; j++) x->u[1u + j] = 20000u + ((uint32_t)(fxv_sin16(x->et * 2u + j * 16384u) + 32768) * 5000u >> 16);
    return b3_want(L, x->intensity, 1700u);
}

static int32_t os_orbit(const fxv_ctx_t *x, const uint32_t *h, uint32_t k, uint32_t gen, uint32_t well, uint32_t e)
{
    const uint32_t hq = fxv_hash2(x->seed ^ 0x90u, k * 256u + gen), w = h[OS_W];
    const uint32_t span4 = (h[H_L] * 16u) / w, amax4 = (span4 * 82u) >> 8;
    const uint32_t e16 = 13107u + ((hq >> 8) & 255u) * 140u, e2 = (e16 * e16) >> 16, sq = b3_isqrt((65536u - e2) << 16);
    const uint32_t ps = (hq >> 16) & 0xFFFFu;
    uint32_t a4 = 96u + (((hq & 255u) * (amax4 > 96u ? amax4 - 96u : 0u)) >> 8), b4;
    int32_t xx, yy, pr;
    a4 = (a4 * (320u - x->custom1)) >> 8;
    if (a4 < 64u) a4 = 64u;
    if (a4 > 8000u) a4 = 8000u;
    b4 = (a4 * sq) >> 16;
    xx = ((int32_t)a4 * (fxv_cos16(e) - (int32_t)(e16 >> 1))) >> 15;
    yy = ((int32_t)b4 * fxv_sin16(e)) >> 15;
    pr = (xx * fxv_cos16(ps) + yy * fxv_sin16(ps)) >> 15;
    return (int32_t)h[OS_C + well] + pr * 4096;
}

static void os_spawn(fxv_ctx_t *x, const uint32_t *h, fxv_particle_t *p, uint32_t k, uint32_t salt)
{
    const uint32_t r = b3_rnd(x, k, salt), well = ((r >> 16) * h[OS_W]) >> 16;
    p->age = (uint16_t)(r >> 8);                         /* the eccentric anomaly */
    p->life = (uint16_t)(0x8000u | (r & 255u));          /* bits 0-7: the orbit's generation, 8-14: a transfer */
    p->kind = (uint8_t)(well | well << 2);               /* the well, the well it came from */
    p->pal = (uint8_t)(r >> 24);
    p->flags = 0;
    p->x = os_orbit(x, h, k, r & 255u, well, p->age);
    p->v = 0;
}

static void os_prime(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    os_spawn(x, h, &x->p[k], k, 8u);
}

static uint32_t os_pass(fxv_ctx_t *x, uint32_t *h, uint32_t pass, uint32_t k)
{
    fxv_particle_t *p = &x->p[k];
    const uint32_t keep = k < h[H_N], w = h[OS_W];
    uint32_t gen, xf, well, from, hq, e16, om, e, de;
    int32_t pos;
    (void)pass;
    if (p->life == 0u) {
        if (!keep) return 1u;
        os_spawn(x, h, p, k, 0);
    }
    if (!b3_fade(p, keep, 4u)) return 1u;
    gen = p->life & 255u;
    xf = (p->life >> 8) & 127u;
    well = p->kind & 3u;
    from = (p->kind >> 2) & 3u;
    if (well >= w) well = w - 1u;
    if (from >= w) from = w - 1u;
    hq = fxv_hash2(x->seed ^ 0x90u, k * 256u + gen);
    e16 = 13107u + ((hq >> 8) & 255u) * 140u;
    om = 220u + (((hq >> 24) * 436u) >> 8);
    e = p->age;
    de = (om << 16) / (uint32_t)(65536 - (((int32_t)e16 * fxv_cos16(e)) >> 15));   /* Kepler: fast at the close pass */
    e += de;
    if (e > 65535u && xf == 0u) {                        /* a close pass: maybe a transfer */
        const uint32_t r = b3_rnd(x, k, 4u);
        if (((r >> 16) & 255u) < 5u + x->custom2 * 60u / 255u) {
            from = well;
            if (w > 1u) well = (well == 0u || (well + 1u < w && (r & 1u))) ? well + 1u : well - 1u;
            gen = (gen + 1u) & 255u;
            xf = 1;
        }
    } else if (xf) {
        if (++xf > 80u) xf = 0;
    }
    e &= 0xFFFFu;
    pos = os_orbit(x, h, k, gen, well, e);
    if (xf) {                                            /* the blend from the old orbit to the new one (1.6 s at 1x) */
        const int32_t old = os_orbit(x, h, k, (gen - 1u) & 255u, from, e), diff = pos - old;
        const int32_t s = (int32_t)((fxv_smooth16(xf * 819u) + 128u) >> 8);
        pos = old + (diff >> 8) * s + (((diff & 255) * s) >> 8);
    }
    p->v = b3_clampi(pos - p->x, 262144);
    p->x = pos;
    p->age = (uint16_t)e;
    p->life = (uint16_t)(0x8000u | gen | xf << 8);
    p->kind = (uint8_t)(well | from << 2);
    return 1u;
}

static void os_splat(fxv_ctx_t *x, uint32_t *h, uint32_t k)
{
    const fxv_particle_t *p = &x->p[k];
    uint32_t sp, r, b, a;
    if (p->life == 0u) return;
    sp = (uint32_t)(p->v < 0 ? -p->v : p->v);
    r = 45875u + ((sp * 154u) >> 8);                     /* 0.7 px, wider when fast */
    if (r > 262144u) r = 262144u;
    b = (sp >> 4) * 26214u / 4915u;
    b = 39321u + (b > 26214u ? 26214u : b);              /* brighter when fast */
    a = (((b * 45875u) / r) * ((uint32_t)p->flags + 1u)) >> 8;
    b3_splat4(x, 0, b3_ix(h, p), r, fxv_scale_px(fxv_pal(x, (uint32_t)p->pal * 257u), a), a >> 2);
}

static void os_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b3_h(x), w = h[OS_W];
    const fxv_px_t space = fxv_rgb16(x->c[1]), well = fxv_rgb16(x->c[2]);
    uint32_t i, j, vp = x->vbase + from;
    for (i = 0; i < count; i++, vp++) {
        fxv_px_t c = space;
        if (vp < 32768u) {
            const int32_t pc = (int32_t)(vp << 16) + 32768;
            for (j = 0; j < w; j++) {                    /* the wells: dim, softly pulsing */
                const int32_t d = pc - (int32_t)h[OS_C + j];
                const uint32_t ad = (uint32_t)(d < 0 ? -d : d);
                if (ad < 393216u) {
                    const uint32_t g = (393216u - ad) >> 3;
                    c = fxv_add_px(c, fxv_scale_px(well, ((((g * g) >> 16) * x->u[1u + j]) >> 16)));
                }
            }
        }
        if (b3_pl(x, 0)[from + i] | b3_pl(x, 1)[from + i] | b3_pl(x, 2)[from + i] | b3_pl(x, 3)[from + i]) {
            c = fxv_add_px(c, b3_px4(x, 0, from + i, x->u[0]));
        }
        o[i] = c;
    }
}

static const b3_spec_t OS = { 256, 0, 128, 1, 3, 6, 0, 4, 96, 8, 0, 0, os_want, os_prime, 0, os_pass, 0, 0, 0, os_splat };
B3_DESC(orbit_slingshot, OS, os_render)

/* ------------------------------------------------------------------------------- 67 Dew Necklace
 * A thread (c2) of fixed dew anchors (custom1: spacing, intensity: the water). Over the coarsening period (speed:
 * 15 min at 0 .. 45 s at 255, its own real-time clock) the fine beads (c1) feed the pearl of their group of seven,
 * nearest first, the water conserved; the pearls hold, then redistribute it back into fine beads (custom2: 5 .. 45 s),
 * and a new cycle picks other pearls. Volume sets each bead's width (a sqrt) and its pearl heart (c3, natural white at
 * c3's temperature). Closed-form per anchor: the frame's slices fill a table of the anchors near the run. */
#define DN_T    (H_E + 0u)             /* the clock: last t, stage (0 fine, 1 feeding, 2 pearls, 3 back), its phase */
#define DN_ST   (H_E + 1u)
#define DN_PH   (H_E + 2u)
#define DN_CYC  (H_E + 3u)
#define DN_A0   (H_E + 4u)             /* the table: first anchor, count; the strip's anchors; spacing and fine radius */
#define DN_NA   (H_E + 5u)             /* (Q8); the largest radius; this cycle's group offset */
#define DN_AN   (H_E + 6u)
#define DN_S8   (H_E + 7u)
#define DN_RF   (H_E + 8u)
#define DN_RMAX (H_E + 9u)
#define DN_OFF  (H_E + 10u)
#define DN_TAB  256u

static uint32_t dn_cap(uint32_t n)
{
    return (n + 40u) / 3u + 4u;
}

static uint32_t dn_bytes(uint32_t n, uint32_t tier)
{
    (void)tier;
    return 256u + 12u * dn_cap(n);
}

static void dn_init(fxv_ctx_t *x)
{
    b3_h(x)[DN_T] = x->t_last;
    x->job[0] = 0xFFFFu;
}

static uint32_t dn_period(uint32_t speed)               /* the coarsening period, ms: 15 min .. 45 s, even in log time */
{
    const uint32_t q = (speed * 1111u) >> 8, base = 900000u >> (q >> 8);
    return base - ((base * (q & 255u)) >> 9);
}

static void dn_adv(fxv_ctx_t *x)
{
    uint32_t *h = b3_h(x);
    const uint32_t P = dn_period(x->speed), L = x->vlen < B3_LMAX ? x->vlen : B3_LMAX, st = h[DN_ST];
    const uint32_t D = st == 0u ? (P * 31u) >> 8 : st == 1u ? (P * 154u) >> 8 : st == 2u ? (P * 72u) >> 8 : 5000u + x->custom2 * 157u;
    uint32_t dt = x->t_last - h[DN_T], ph, s8, rf, rm, an, lo, a0, a1, cnt;
    h[DN_T] = x->t_last;
    if (dt > 0x7FFFFFFFu) dt = 0;
    if (dt > 4000u) dt = 4000u;
    ph = h[DN_PH] + dt * (0xFFFFFFFFu / D);
    if (ph < h[DN_PH]) {                                 /* the stage is over */
        h[DN_ST] = (st + 1u) & 3u;
        if (h[DN_ST] == 0u) h[DN_CYC]++;
    }
    h[DN_PH] = ph;
    s8 = 768u + x->custom1 * 9u;
    rf = (s8 * (15u + ((x->intensity * 87u) >> 8))) >> 8;
    rm = rf * 3u + 256u;
    an = (L << 8) / s8;
    lo = x->vbase * 256u;
    a0 = lo > rm ? (lo - rm) / s8 : 0u;
    if (a0) a0--;
    a1 = ((x->vbase + x->n) * 256u + rm) / s8 + 1u;
    if (a1 >= an) a1 = an ? an - 1u : 0u;
    cnt = an && a1 >= a0 ? a1 - a0 + 1u : 0u;
    if (cnt > dn_cap(x->n)) cnt = dn_cap(x->n);
    h[DN_A0] = a0;
    h[DN_NA] = cnt;
    h[DN_AN] = an;
    h[DN_S8] = s8;
    h[DN_RF] = rf;
    h[DN_RMAX] = 0;
    h[DN_OFF] = ((fxv_hash2(x->seed, h[DN_CYC]) >> 16) * 7u) >> 16;
    x->u[0] = b3_k_of(x->c[2]);
    if (!x->pal_own) x->u[7] = b3_pstep(L);
    x->job[0] = 0;
}

static uint32_t dn_v0(const fxv_ctx_t *x, uint32_t i)    /* an anchor's own water, Q16 of a fine bead: 0.85 .. 1.15 */
{
    return 55705u + (((fxv_hash2(x->seed, i) >> 16) * 19661u) >> 16);
}

/* how much of member m's water has gone to its pearl pm (0..65535), at this stage and phase */
static uint32_t dn_gone(const fxv_ctx_t *x, const uint32_t *h, uint32_t i, uint32_t m, uint32_t pm)
{
    const uint32_t st = h[DN_ST], c = h[DN_PH] >> 16, dl = m > pm ? m - pm : pm - m, dmax = pm > 6u - pm ? pm : 6u - pm;
    uint32_t t, lin;
    if (st == 0u) return 0u;
    if (st == 2u) return 65535u;
    if (st == 1u) {                                      /* feeding: the nearest first, each over 0.3 of the stage */
        t = (dl - 1u) * 40632u / dmax + (fxv_hash2(x->seed ^ 0x67u, i) & 4095u);
        lin = c > t ? ((c - t) * 853u) >> 8 : 0u;
        return fxv_smooth16(lin > 65535u ? 65535u : lin);
    }
    t = (dl - 1u) * 32768u / dmax;                       /* back: the nearest refill first */
    lin = c > t ? ((c - t) * 569u) >> 8 : 0u;
    return 65535u - fxv_smooth16(lin > 65535u ? 65535u : lin);
}

static uint32_t dn_vol(const fxv_ctx_t *x, const uint32_t *h, uint32_t i)   /* anchor i's water now */
{
    const uint32_t g = (i + h[DN_OFF]) / 7u, m = i + h[DN_OFF] - g * 7u;
    const uint32_t pm = 1u + (((fxv_hash2(x->seed ^ h[DN_CYC], g) >> 16) * 5u) >> 16);
    uint32_t v = dn_v0(x, i), mm;
    if (m != pm) return ((v >> 1) * (65535u - ((dn_gone(x, h, i, m, pm) * 60293u) >> 16))) >> 15;
    for (mm = 0; mm < 7u; mm++) {                        /* the pearl: its own water and what its group gave */
        const uint32_t j = g * 7u + mm;
        if (mm == pm || j < h[DN_OFF] || j - h[DN_OFF] >= h[DN_AN]) continue;
        v += ((dn_v0(x, j - h[DN_OFF]) >> 1) * ((dn_gone(x, h, j - h[DN_OFF], mm, pm) * 60293u) >> 16)) >> 15;
    }
    return v;
}

/* anchors a slice: a pearl sums its group of seven (~1,800 cycles feeding or refilling, a fine bead ~600); two pearls
 * can be 3 anchors apart, so 7 a slice reached 6,462 cycles (800 px, all-max, late); 3 hold at most one pearl */
#define DN_STEP 3u
static uint32_t dn_slice(fxv_ctx_t *x, uint32_t quota)
{
    uint32_t *h = b3_h(x), t, end;
    int32_t *tab = (int32_t *)(x->arena + DN_TAB);
    (void)quota;
    if (x->job[0] >= h[DN_NA]) return 0;
    end = x->job[0] + DN_STEP < h[DN_NA] ? x->job[0] + DN_STEP : h[DN_NA];
    for (t = x->job[0]; t < end; t++) {
        const uint32_t i = h[DN_A0] + t, s8 = h[DN_S8], sv = b3_isqrt(dn_vol(x, h, i));
        const uint32_t r8 = (h[DN_RF] * sv) >> 8, hw = sv > 200u ? ((sv - 200u) * 120u > 65535u ? 65535u : (sv - 200u) * 120u) : 0u;
        tab[t * 3u] = (int32_t)(i * s8 + (s8 >> 1)) + ((((int32_t)(fxv_hash2(x->seed ^ 0xDEu, i) >> 24) - 128) * (int32_t)s8) >> 10);
        tab[t * 3u + 1u] = (int32_t)(r8 | hw << 16);
        tab[t * 3u + 2u] = (int32_t)(r8 >= 16u ? (65535u << 8) / r8 : 0u);
        if (r8 > h[DN_RMAX]) h[DN_RMAX] = r8;
    }
    x->job[0] = end;
    return end < h[DN_NA];
}

static void dn_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b3_h(x), na = h[DN_NA], rm = h[DN_RMAX] + 256u;
    const int32_t *tab = (const int32_t *)(x->arena + DN_TAB);
    fxv_px_t stem = fxv_rgb16(x->c[1]), dew = fxv_rgb16(x->c[0]), heart = b3_white(fxv_rgb16(x->c[2]), x->u[0]);
    int32_t c8 = (int32_t)((x->vbase + from) * 256u + 128u);
    uint32_t i, t = 0, a8 = (x->vbase + from) * x->u[7] + (x->u[7] >> 1);
    if (na && c8 - (int32_t)rm > tab[0]) {
        t = (uint32_t)(c8 - (int32_t)rm - tab[0]) / h[DN_S8];
        t = t > 1u ? t - 1u : 0u;
        if (t >= na) t = na - 1u;
    }
    for (i = 0; i < count; i++, c8 += 256, a8 += x->u[7]) {
        fxv_px_t c;
        uint32_t j, best = 0;
        if (!x->pal_own) {
            /* palette: the place on the thread picks the colour, the beads strung along the whole palette; the stem,
             * the dew and the pearl hearts are that colour at the default slots' levels (the hearts the brightest) */
            const fxv_px_t pc = fxv_pal(x, a8 >> 8);
            stem = fxv_scale_px(pc, 4626u);                 /* 18 / 255 */
            dew = fxv_scale_px(pc, 37779u);                 /* 147 / 255 */
            heart = fxv_scale_px(pc, 51143u);               /* 199 / 255 */
        }
        c = stem;
        while (t + 1u < na && tab[t * 3u] + (int32_t)rm < c8) t++;
        for (j = t; j < na && tab[j * 3u] - (int32_t)rm <= c8; j++) {
            const int32_t pp = tab[j * 3u], d = c8 - pp;
            const uint32_t r8 = (uint32_t)tab[j * 3u + 1u] & 0xFFFFu, ad = (uint32_t)(d < 0 ? -d : d);
            int32_t lo, hi;
            uint32_t cov, b, lin, hh;
            fxv_px_t bc;
            if (r8 < 16u || ad >= r8 + 128u) continue;
            lo = c8 - 128 > pp - (int32_t)r8 ? c8 - 128 : pp - (int32_t)r8;    /* how much of the pixel it covers */
            hi = c8 + 128 < pp + (int32_t)r8 ? c8 + 128 : pp + (int32_t)r8;
            cov = (uint32_t)(hi - lo) * 256u;
            if (cov > 65535u) cov = 65535u;
            if (cov <= best) continue;
            best = cov;
            lin = ad < r8 ? (ad * (uint32_t)tab[j * 3u + 2u]) >> 8 : 65535u;
            if (lin > 65535u) lin = 65535u;
            b = fxv_smooth16(65535u - lin);
            hh = lin < 32768u ? fxv_smooth16(65535u - lin * 2u) : 0u;
            hh = (hh * ((uint32_t)tab[j * 3u + 1u] >> 16)) >> 16;
            bc = fxv_add_px(fxv_scale_px(dew, 29491u + ((b * 36044u) >> 16)), fxv_scale_px(heart, hh));
            c = fxv_blend16(stem, bc, cov);
        }
        o[i] = c;
    }
}

const fxv_desc_t fxv_d_dew_necklace = { dn_bytes, dn_init, dn_adv, dn_slice, dn_render, 0, 0, b3_digest };
