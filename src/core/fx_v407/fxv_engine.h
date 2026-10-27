/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The V407 layer's segment engine (fxv_engine.c): up to four segments and two crossfades (fx.h layer 2's behaviour) on
 * fxv contexts in six fixed arena slots, the global particle slab with owner quotas, the particle step-down and the
 * 16-bit compositor. The receiver's glue (core/ur_digital_m/udfx.c, FX_V407) drives it like fx_engine_t. */
#ifndef FXV_ENGINE_H
#define FXV_ENGINE_H

#include "fxv_gen.h"

#ifndef FXV_MAX_SEG
#define FXV_MAX_SEG    4u
#endif
#define FXV_MAX_XFADE  2u

typedef struct {
    fxv_ctx_t cur;
    rgb_t    tc[3];                      /* the last slot's colours, brightness, virtual strip: the ramps' targets */
    uint8_t  tbri;
    uint8_t  flags, active;
    uint8_t  started;                    /* the anchor has been reached */
    uint8_t  due;                        /* the frame start is the next engine step's (fxv_engine_frame_seg) */
    uint16_t tvstart, tvlen;
    uint16_t start, len;                 /* the segment's pixels on the strip */
    uint16_t n_eff;                      /* pixels the effect renders (mirror: half) */
    uint16_t tr_dur;
    uint32_t tseed;                      /* the slot's seed + the segment (the renewal test; the context's may carry an epoch) */
    uint32_t anchor;                     /* host ms of the effect's t = 0 */
    uint32_t tr_t0, tr_inv;              /* the ramps: start, 2^24 / trans_ms (0: none) */
    rgb_t    c0[3];
    uint8_t  bri0, bri;
    int8_t   xf;                         /* the crossfade slot, or -1 */
} fxv_seg_t;

typedef struct {
    fxv_ctx_t prev;                      /* the effect fading out (it keeps its arena slot) */
    uint32_t t0, inv, dur;
    uint32_t anchor;
    uint8_t  started;
    uint8_t  w;                          /* the new effect's weight 0..255 */
    uint8_t  used;                       /* bit 0 live, bit 1 a queued replacement, bit 2 prev's frame start due (a step) */
    uint8_t  w0;
    fx_params_t next;
    uint32_t next_anchor;
} fxv_xfade_t;

typedef struct {
    fxv_seg_t   seg[FXV_MAX_SEG];
    fxv_xfade_t xf[FXV_MAX_XFADE];
    uint8_t    *arena;                   /* FXV_ARENA_BYTES: slot k at k x FXV_SLOT_MAX */
    fxv_particle_t *slab;                /* FXV_PARTICLES records */
    uint32_t    now;                     /* host ms of the last frame preparation */
    uint32_t    cost;                    /* the modeled cycles of a 42 fps frame (the quota's model) */
    uint16_t    npx;
    uint8_t     fmt;                     /* wire bytes per pixel */
    uint8_t     slot_used;               /* bit k: slot k owned by a live context */
    uint8_t     rebal, moving, moved, tier;
    uint8_t     quoting, qtier;          /* the new quotas: tier qtier next (one tier a step) */
    uint16_t    mpos;                    /* records of the moving owner done (copied, then zeroed): FXV_MOVE_STEP a step */
    uint16_t    pfirst[FXV_SLOTS], pcount[FXV_SLOTS];   /* each slot's particle range now */
    uint16_t    nfirst[FXV_SLOTS], ncount[FXV_SLOTS];   /* ... and after the pending moves */
} fxv_engine_t;

void     fxv_engine_init(fxv_engine_t *e, uint32_t npx, uint32_t fmt, uint8_t *arena, fxv_particle_t *slab);
/* the PIXEL_FX of segment s at host ms now, its t = 0 at anchor: fx_engine_set's rules (renewals, ramps, crossfades, a
 * queued replacement finishing a live fade within 250 ms); an id this build does not run turns the segment off */
void     fxv_engine_set(fxv_engine_t *e, uint32_t s, const fx_params_t *p, uint32_t now, uint32_t anchor);
/* a segment's frame start: its colour ramp; the effect's frame start and the fading one's are the next engine steps */
void     fxv_engine_frame_seg(fxv_engine_t *e, uint32_t s, uint32_t now);
uint32_t fxv_engine_dequeue(fxv_engine_t *e, uint32_t s, uint32_t now);
uint32_t fxv_engine_step(fxv_engine_t *e);       /* one bounded slice: a frame start, quotas, a particle move, zeroing,
                                                    init, advance */
void     fxv_engine_frame(fxv_engine_t *e, uint32_t now);                    /* all of it (tests, JS) */
/* strip pixels [from, to) (to - from <= FXV_CHUNK), black where no segment, brightness applied */
void     fxv_engine_render(const fxv_engine_t *e, fxv_px_t *out, uint32_t from, uint32_t to);
uint32_t fxv_engine_dark(const fxv_engine_t *e);
/* the next render + pack step [from, return) of [from, to): one run (one top segment, one side of a mirror), 5,000
 * modeled cycles (a crossfade: less its fixed part) at that run's own cost a pixel, at most FXV_CHUNK pixels, at least
 * one (from < to) */
uint32_t fxv_engine_span(const fxv_engine_t *e, uint32_t from, uint32_t to);
uint32_t fxv_engine_cost(const fxv_engine_t *e);

#endif
