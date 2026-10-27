/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Effects 1-36 on the V407 layer: the legacy engine's context (fx.h layer 1) in the arena, its prep / slices as the
 * advance, its 8-bit pixels widened to working colour (x 256, q 0). Every effect keeps its own heat (Fire) in its own
 * slot: no state is shared across a crossfade (PLAN section 1), and every Fire gets the smoothing heat array. */
#include "fxv_gen.h"

#define LX_CTX   256u                    /* the fx_ctx_t, padded (host pointers are 8 bytes) */
#define LX_HEAT  FX_FIRE_CELLS           /* Fire: heat, then the previous heat */
typedef char lx_ctx_fits[sizeof(fx_ctx_t) <= LX_CTX ? 1 : -1];

static fx_ctx_t *lx(const fxv_ctx_t *x)
{
    return (fx_ctx_t *)(void *)x->arena;
}

static uint32_t lx_bytes(uint32_t n, uint32_t tier)
{
    (void)n;
    (void)tier;
    return LX_CTX + 2u * LX_HEAT;
}

static void lx_init(fxv_ctx_t *x)
{
    fx_params_t p;
    uint8_t *b = (uint8_t *)&p;
    uint32_t k;
    for (k = 0; k < (uint32_t)sizeof p; k++) b[k] = 0;
    p.id = x->id;
    p.speed = x->speed;
    p.intensity = x->intensity;
    p.palette = x->palette;                              /* (resolved: fx_begin builds 31.. from the exact table; PAL_DEFAULT
                                                            = own resolves to own again: the same pal_own) */
    for (k = 0; k < 3u; k++) p.c[k] = x->c[k];
    p.flags = x->flags;
    p.custom1 = x->custom1;
    p.custom2 = x->custom2;
    p.vstart = (uint16_t)x->vbase;
    p.vlen = x->vlen;
    p.seed = x->seed;
    fx_begin(lx(x), &p, x->n, 0);                      /* (the layer zeroed the arena: fx_begin's own zeroing of the
                                                            heat, a byte a call, cost Fire's start 5,345 cycles) */
    lx(x)->state = x->arena + LX_CTX;
    if (x->id == FX_FIRE) lx(x)->fire_prev = x->arena + LX_CTX + LX_HEAT;   /* (zeroed with the arena) */
}

static void lx_advance_begin(fxv_ctx_t *x)
{
    fx_frame_begin(lx(x), x->t_last);
}

static uint32_t lx_advance_slice(fxv_ctx_t *x, uint32_t quota)
{
    (void)quota;
    return fx_frame_step(lx(x));
}

static void lx_render16(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *out)
{
    rgb_t t[FXV_CHUNK];
    uint32_t k;
    fx_render(lx(x), t, from, from + count);
    for (k = 0; k < count; k++) out[k] = fxv_rgb16(t[k]);
}

static void lx_changed(fxv_ctx_t *x)
{
    fx_ctx_t *l = lx(x);
    fx_set_speed(l, x->speed);
    l->intensity = x->intensity;
    l->custom1 = x->custom1;
    l->custom2 = x->custom2;
    l->seed = x->seed;
    fx_set_colors(l, x->c);
}

const fxv_desc_t fxv_d_legacy = { lx_bytes, lx_init, lx_advance_begin, lx_advance_slice, lx_render16, 0, lx_changed, 0 };

#if !FX_NO_PACIFICA
/* Pacifica (18): a render call re-anchors its waves at the call's 32-pixel block and steps up to 31 pixels to its first
 * one (6,186 cycles for a 1-pixel step), so the advance renders the strip ahead into the arena in bounded pieces
 * (fx_pacifica_run: 16 anchor steps or PA_STEP pixels) and render16 only copies. The same pixels as the legacy engine. */
#define PA_STEP 4u
static fx_pac_run_t *pa_run(const fxv_ctx_t *x) { return (fx_pac_run_t *)(void *)(x->arena + LX_CTX); }
static rgb_t *pa_image(const fxv_ctx_t *x) { return (rgb_t *)(void *)(x->arena + LX_CTX + sizeof(fx_pac_run_t)); }

static uint32_t pa_bytes(uint32_t n, uint32_t tier)
{
    (void)tier;
    return LX_CTX + (uint32_t)sizeof(fx_pac_run_t) + 3u * n;
}

static void pa_advance_begin(fxv_ctx_t *x)
{
    fx_frame_begin(lx(x), x->t_last);
    x->job[0] = 0;
}

static uint32_t pa_advance_slice(fxv_ctx_t *x, uint32_t quota)
{
    (void)quota;
    if (!x->job[0]) {
        fx_pacifica_begin(lx(x), pa_run(x));
        x->job[0] = 1;
        return 1;
    }
    return fx_pacifica_run(lx(x), pa_run(x), pa_image(x), PA_STEP);
}

static void pa_render16(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *out)
{
    const rgb_t *img = pa_image(x) + from;
    uint32_t k;
    for (k = 0; k < count; k++) out[k] = fxv_rgb16(img[k]);
}

const fxv_desc_t fxv_d_pacifica = { pa_bytes, lx_init, pa_advance_begin, pa_advance_slice, pa_render16, 0, lx_changed, 0 };
#endif
