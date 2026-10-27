/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The V407 effects layer's context lifecycle (fxv_api.h): the registry, palettes, the effect clock and the simulation
 * ticks, the staged start (the arena zeroed in slices, then init), the frame's advance in bounded slices, rendering.
 * The JS port: fx/v407/common.js (same names, same bytes). */
#include "fxv_gen.h"
#include "../math32.h"

#define ST_ZERO  0u                      /* zeroing the arena (FXV_ZERO_STEP bytes a slice), then the particle range */
#define ST_INIT  1u
#define ST_IDLE  FXV_ST_IDLE             /* the frame's advance is complete: render may run (fxv_ready) */
#define ST_ADV   3u                      /* advance slices pending */
#define ST_HOLD  4u                      /* the short-strip hold's pass (FXV_D_HOLD, under FXV_HOLD_N pixels) */
#define FXV_ZERO_STEP 256u
/* The short-strip hold: on a strip under FXV_HOLD_N pixels anything local covers the whole strip, so an FXV_D_HOLD
 * effect's pixels may brighten at once but dim at most at a ~2 s release (FXV_HOLD_DECAY / 65536 a ms on the working
 * colour: luminance falls by under 10 % of full scale in 71 ms, the flash gate's transition). After the advance the
 * layer renders the frame into a held image (the arena's last 8 x n bytes) FXV_HOLD_STEP pixels a slice, keeping each
 * pixel's brighter one (the gate's luminance) of the new and the held-and-released; render16 is then a copy. */
#define FXV_HOLD_STEP  2u
#define FXV_HOLD_DECAY 33u

extern const fxv_desc_t fxv_d_legacy, fxv_d_pacifica;
void fx_pal_expand(rgb_t *pal, uint32_t id);   /* fx_effects.c (FX_V407): 16 entries of built-in palette id */

const fxv_desc_t *fxv_desc(uint32_t id)
{
    if (id == 0u || id >= FXV_FX_COUNT) return 0;
#if !FX_NO_PACIFICA
    if (id == FX_PACIFICA) return &fxv_d_pacifica;  /* (rendered ahead: fxv_legacy.c) */
#endif
    if (id < FX_COUNT) return &fxv_d_legacy;
    return fxv_registry[id - FX_COUNT];
}

uint32_t fxv_built(uint32_t id)
{
    return fxv_desc(id) != 0;
}

uint8_t fxv_default_palette(uint32_t id)
{
    return id < FXV_FX_COUNT ? fxv_pal_default_tab[id] : 0u;
}

/* The one resolution of a slot's palette byte (fxv_begin and the engine's same-effect check): PAL_DEFAULT gives the
 * effect's default, PAL_DEFAULT again for an effect whose default is its own colours (pal_own); unknown ids: 0 */
uint8_t fxv_pal_resolve(uint32_t id, uint32_t palette)
{
    const uint32_t pal = palette == PAL_DEFAULT ? fxv_default_palette(id) : palette;
    return (uint8_t)(pal == PAL_COLORS || pal == PAL_DEFAULT || pal < FXV_PAL_COUNT ? pal : 0u);
}

static void build_palette(fxv_ctx_t *x)
{
    uint32_t k;
    if (x->palette == PAL_COLORS) {                      /* c1 -> c2 -> c3 -> c1 over the 16 entries (as fx_effects.c) */
        for (k = 0; k < 16u; k++) {
            const uint32_t s = (k * 3u) >> 4, f = ((k * 3u) & 15u) << 4;
            x->pal[k] = fx_blend(x->c[s], x->c[s == 2u ? 0u : s + 1u], f + (f >> 4));
        }
        x->pal_wrap = 1;
    } else {
        const uint32_t id = x->pal_own ? 0u : x->palette;  /* (own colours: palette 0, unused) */
        fx_pal_expand(x->pal, id);
        x->pal_wrap = (uint8_t)((fxv_pal_cyclic[id >> 3] >> (id & 7u)) & 1u);
    }
}

/* (a segment set is one receiver step: the context is zeroed a word at a time, its palette, up to ~2,400 cycles for a
 * legacy gradient, waits for the first start slice) */
void fxv_begin(fxv_ctx_t *x, const fx_params_t *p, uint32_t n, uint32_t fmt, uint8_t *arena, uint32_t slot)
{
    uint32_t k;
    fxv_word_t *w = (fxv_word_t *)(void *)x;
    for (k = 0; k < (uint32_t)(sizeof(*x) / 4u); k++) w[k] = 0;
    x->d = fxv_desc(p->id);
    x->id = p->id;
    x->speed = p->speed;
    x->intensity = p->intensity;
    x->custom1 = p->custom1;
    x->custom2 = p->custom2;
    x->flags = p->flags;
    x->fmt = (uint8_t)fmt;
    for (k = 0; k < 3u; k++) x->c[k] = p->c[k];
    x->n = (uint16_t)n;
    x->vlen = p->vlen != 0u ? p->vlen : (uint16_t)(n != 0u ? n : 1u);
    x->vbase = p->vstart;
    x->seed = p->seed;
    x->tick_ms = x->d ? fxv_meta[x->id].tick_ms : 0u;
    x->palette = fxv_pal_resolve(x->id, p->palette);
    x->pal_own = x->palette == PAL_DEFAULT;
    x->pal_due = 1;
    x->arena = arena;
    x->slot = (uint8_t)slot;
    x->need = x->d ? x->d->bytes(n, 0) : 0u;
    x->hold = (uint8_t)(x->d && (fxv_meta[x->id].flags & FXV_D_HOLD) && n < FXV_HOLD_N);
    if (x->hold) x->need = ((x->need + 7u) & ~7u) + 8u * n;
    if (x->need > FXV_SLOT_MAX) x->need = FXV_SLOT_MAX;    /* (an effect over its contract: the host tests fail it) */
    x->stage = ST_ZERO;
}

void fxv_clock(fxv_ctx_t *x, uint32_t t_ms)
{
    int32_t dt = (int32_t)(t_ms - x->t_last);
    const uint32_t rate = x->speed == 0u ? 0u : 8u + ((x->speed * 15u) >> 3);   /* = fx_effects.c's fx_rate256 */
    uint32_t acc, de;
    if (dt < 0) dt = 0;                                  /* (the host clock stepped back: stand still) */
    acc = (uint32_t)dt * rate + x->et_frac;
    de = m32_addshr((uint32_t)dt, rate, x->et_frac, 8);
    x->et += de;
    x->de = de;
    x->et_frac = (uint8_t)acc;
    x->t = x->t + (uint32_t)dt < x->t ? 0xFFFFFFFFu : x->t + (uint32_t)dt;
    x->t_last = t_ms;
    if (x->hold) x->hold_dt = (uint8_t)(x->hold_dt + (uint32_t)dt > 255u ? 255u : x->hold_dt + (uint32_t)dt);
    if (x->tick_ms != 0u) {
        const uint32_t a = x->tick_acc + de < de ? 0xFFFFFFFFu : x->tick_acc + de, tk = a / x->tick_ms;
        const uint32_t debt = x->tick_debt + tk < tk ? 0xFFFFFFFFu : x->tick_debt + tk;
        x->tick_acc = a - tk * x->tick_ms;
        if (debt > FXV_RESYNC_MS / x->tick_ms) {
            /* a discontinuity (a late start, a long stall, a big speed jump): restart with seed + epoch, no catch-up */
            x->epoch++;
            x->seed = fxv_hash2(x->seed, x->epoch);
            x->tick_debt = 0;
            x->tick_frame = 0;
            x->tick_acc = 0;
            x->stage = ST_ZERO;
            x->zpos = 0;
        } else {
            x->tick_debt = debt;
            x->tick_frame = debt < FXV_TICKS_MAX ? debt : FXV_TICKS_MAX;
        }
    }
}

void fxv_frame_begin(fxv_ctx_t *x, uint32_t t_ms)
{
    fxv_clock(x, t_ms);
    if (x->stage < ST_IDLE) {
        x->pending_adv = 1;                              /* advance_begin after init */
    } else if (x->d) {
        x->d->advance_begin(x);
        x->stage = ST_ADV;
    }
}

static fxv_px_t *hold_img(const fxv_ctx_t *x)
{
    return (fxv_px_t *)(void *)(x->arena + x->need - 8u * x->n);
}

static uint32_t hold_lum(fxv_px_t c)                     /* the flash gate's luminance, x 10,000 */
{
    return 2126u * fxv_gamma16f(c.r) + 7152u * fxv_gamma16f(c.g) + 722u * fxv_gamma16f(c.b);
}

static FX_NOINL uint32_t hold_slice(fxv_ctx_t *x)      /* (out of line: fxv_frame_step's advance path stays as lean) */
{
    fxv_px_t raw[FXV_HOLD_STEP], *img = hold_img(x);
    const uint32_t a = x->zpos, b = a + FXV_HOLD_STEP < x->n ? a + FXV_HOLD_STEP : x->n;
    const uint32_t f = 65535u - x->hold_dt * FXV_HOLD_DECAY;
    uint32_t k;
    x->d->render16(x, a, b - a, raw);
    for (k = a; k < b; k++) {
        const fxv_px_t h = fxv_scale_px(img[k], f);
        img[k] = hold_lum(raw[k - a]) >= hold_lum(h) ? raw[k - a] : h;
    }
    x->zpos = b;
    return b < x->n;
}

uint32_t fxv_frame_step(fxv_ctx_t *x)
{
    if (!x->d) return 0;
    if (x->stage == ST_ZERO) {
        const uint32_t pb = (uint32_t)x->pn * (uint32_t)sizeof(fxv_particle_t), total = x->need + pb;
        uint32_t end = x->zpos + FXV_ZERO_STEP, k;
        if (x->pal_due) {                                /* (fxv_begin's palette: the first start slice) */
            build_palette(x);
            x->pal_due = 0;
            return 1;
        }
        if (end > total) end = total;
        for (k = x->zpos; k < end; k++) {
            if (k < x->need) x->arena[k] = 0;
            else ((uint8_t *)x->p)[k - x->need] = 0;
        }
        x->zpos = end;
        if (end >= total) x->stage = ST_INIT;
        return 1;
    }
    if (x->stage == ST_INIT) {
        x->d->init(x);
        x->stage = ST_IDLE;
        if (x->pending_adv) {
            x->pending_adv = 0;
            x->d->advance_begin(x);
            x->stage = ST_ADV;
        }
        return 1;
    }
    if (x->stage == ST_ADV) {
        if (x->d->advance_slice && x->d->advance_slice(x, FXV_QUOTA)) return 1;
        if (x->hold) {
            x->stage = ST_HOLD;
            x->zpos = 0;
            return 1;
        }
        x->stage = ST_IDLE;
    }
    if (x->stage == ST_HOLD) {
        if (hold_slice(x)) return 1;
        x->hold_dt = 0;
        x->stage = ST_IDLE;
    }
    return 0;
}

void fxv_frame(fxv_ctx_t *x, uint32_t t_ms)
{
    fxv_frame_begin(x, t_ms);
    while (fxv_frame_step(x)) { }
}

void fxv_render(const fxv_ctx_t *x, fxv_px_t *out, uint32_t from, uint32_t to)
{
    uint32_t k;
    if (to > x->n) to = x->n;
    if (!x->d || from >= to) return;
    if (x->hold) {
        const fxv_px_t *img = hold_img(x);
        for (k = from; k < to; k++) out[k - from] = img[k];
    } else {
        x->d->render16(x, from, to - from, out);
    }
}

void fxv_set_colors(fxv_ctx_t *x, const rgb_t c[3])
{
    uint32_t k;
    for (k = 0; k < 3u; k++) x->c[k] = c[k];
    if (x->palette == PAL_COLORS && !x->pal_due) build_palette(x);
    if (x->d && x->d->changed && x->stage >= ST_IDLE) x->d->changed(x);
}

void fxv_set_params(fxv_ctx_t *x, uint8_t speed, uint8_t intensity, uint8_t custom1, uint8_t custom2, uint32_t seed)
{
    x->speed = speed;
    x->intensity = intensity;
    x->custom1 = custom1;
    x->custom2 = custom2;
    x->seed = seed;
    if (x->d && x->d->changed && x->stage >= ST_IDLE) x->d->changed(x);
}

void fxv_end(fxv_ctx_t *x)
{
    if (x->d && x->d->destroy && x->stage >= ST_IDLE) x->d->destroy(x);
    x->d = 0;
    x->p = 0;
    x->pn = 0;
}

uint32_t fxv_digest(const fxv_ctx_t *x)
{
    return x->d && x->d->digest && x->stage >= ST_IDLE ? x->d->digest(x) : 0u;
}
