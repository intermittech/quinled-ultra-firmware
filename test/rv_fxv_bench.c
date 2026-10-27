/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The V407 effects layer's entry points for tools/measure_fxv.py (rv32emu): an effect at its catalogue defaults on its
 * own (begin / frame / one slice / render in chunks), the 16-bit packer, and the segment engine. Freestanding RV32. */
#include <stdint.h>
#include "fx_v407/fxv_engine.h"

typedef struct { uint8_t id, speed, intensity, palette; uint8_t c1[3], c2[3], c3[3]; uint8_t custom1, custom2; } def_t;
static const def_t defs[] = {
#include "fx_v407/fxv_defaults.inc"
};

static uint8_t arena[FXV_ARENA_BYTES] __attribute__((aligned(16)));
static fxv_particle_t slab[FXV_PARTICLES];
static fxv_ctx_t X;
static fxv_engine_t E;
static fxv_px_t px[FXV_N_MAX];
static uint8_t wire[FXV_N_MAX * 5];
static fxv_white_t wc;
static uint32_t seeded;
static uint32_t pal_force = 256u;                       /* b_palette: every effect on this palette (256: its default) */

void *memset(void *d, int c, uint32_t n) { uint8_t *p = d; while (n--) *p++ = (uint8_t)c; return d; }
void *memcpy(void *d, const void *s, uint32_t n) { uint8_t *p = d; const uint8_t *q = s; while (n--) *p++ = *q++; return d; }

static void params(fx_params_t *p, uint32_t id)
{
    uint32_t k;
    memset(p, 0, sizeof *p);
    for (k = 0; k < sizeof defs / sizeof defs[0]; k++) {
        const def_t *d = &defs[k];
        if (d->id != id) continue;
        p->id = d->id; p->speed = d->speed; p->intensity = d->intensity; p->palette = d->palette;
        p->c[0].r = d->c1[0]; p->c[0].g = d->c1[1]; p->c[0].b = d->c1[2];
        p->c[1].r = d->c2[0]; p->c[1].g = d->c2[1]; p->c[1].b = d->c2[2];
        p->c[2].r = d->c3[0]; p->c[2].g = d->c3[1]; p->c[2].b = d->c3[2];
        p->custom1 = d->custom1; p->custom2 = d->custom2;
    }
    if (pal_force < 256u) p->palette = (uint8_t)pal_force;
    p->bri = 255;
    p->seed = 0x5EED0000u + id;
}
void b_palette(uint32_t pal) { pal_force = pal; }       /* (tools/measure_fxv*.py --palette: G3 in palette mode) */

/* a control corner (the slice gate's: 0 defaults, 1 every control 255, 2 speed 255 + the rest 0, 3 speed 1 + the rest 255) */
static void corner(fx_params_t *p, uint32_t m)
{
    if (m == 1u) { p->speed = 255; p->intensity = 255; p->custom1 = 255; p->custom2 = 255; }
    else if (m == 2u) { p->speed = 255; p->intensity = 0; p->custom1 = 0; p->custom2 = 0; }
    else if (m == 3u) { p->speed = 1; p->intensity = 255; p->custom1 = 255; p->custom2 = 255; }
}
static uint32_t begin_corner;
void b_corner(uint32_t m) { begin_corner = m; }         /* b_begin's corner (tools/measure_fxv_models.py) */

uint32_t b_begin(uint32_t id, uint32_t n, uint32_t fmt)
{
    fx_params_t p;
    params(&p, id);
    corner(&p, begin_corner);
    fxv_begin(&X, &p, n, fmt, arena, 0);
    if (fxv_meta[id].particles) {                        /* a lone owner: its design count */
        X.p = slab;
        X.pn = fxv_meta[id].particles;
    }
    while (fxv_frame_step(&X)) { }
    return X.need;
}
void b_frame_begin(uint32_t t) { fxv_frame_begin(&X, t); }
/* a late state from the JS mirror (tools/measure_fxv_late.py): after b_begin, the arena, the particles, u / job and the
 * clock are poked in (b_snap_ref: their addresses), then b_snap_restore; the next frame continues from that state */
static uint32_t snap_in[13];
uint32_t b_snap_ref(uint32_t k)
{
    return k == 0u ? (uint32_t)X.arena : k == 1u ? (uint32_t)X.p : k == 2u ? (uint32_t)X.u : k == 3u ? (uint32_t)X.job :
           k == 4u ? (uint32_t)snap_in : (uint32_t)px;
}
void b_snap_restore(void)
{
    X.et = snap_in[0]; X.seed = snap_in[1]; X.t_last = snap_in[2]; X.t = snap_in[3]; X.tick = snap_in[4];
    X.tick_acc = snap_in[5]; X.tick_debt = snap_in[6]; X.tick_frame = snap_in[7]; X.et_frac = (uint8_t)snap_in[8];
    X.de = snap_in[9]; X.epoch = (uint16_t)snap_in[10]; X.hold_dt = (uint8_t)snap_in[11]; X.pending_adv = (uint8_t)snap_in[12];
    X.stage = FXV_ST_IDLE;
}
uint32_t b_step(void) { return fxv_frame_step(&X); }
void b_render(uint32_t n, uint32_t chunk)
{
    uint32_t i;
    for (i = 0; i < n; i += chunk) fxv_render(&X, px + i, i, i + chunk < n ? i + chunk : n);
}
uint32_t b_pack(uint32_t n, uint32_t bpp, uint32_t smooth, uint32_t bri, uint32_t q)
{
    static const uint8_t ords[3][5] = { { 1, 0, 2, 0, 0 }, { 1, 0, 2, 3, 0 }, { 0, 1, 2, 3, 4 } };
    fx_pack_cfg_t c;
    uint32_t i;
    if (!seeded) { fxv_white_init(&wc, FXV_K_W_DEFAULT, FXV_K_WW_DEFAULT, FXV_K_CW_DEFAULT); seeded = 1; }
    memset(&c, 0, sizeof c);
    c.gamma = 1; c.bpp = (uint8_t)bpp; memcpy(c.ord, ords[bpp - 3u], 5); c.white_mode = FX_W_ACCURATE; c.white_split = 128;
    c.smooth = (uint8_t)smooth; c.phase = 77; c.px0 = 0; c.dmin = 4;
    for (i = 0; i < n; i++) {
        px[i] = q ? fxv_kelvin16((i * 7u) & 255u) : fxv_rgb16(fx_rgb(i * 37u, i * 91u, i * 13u));
    }
    return fxv_pack16(px, wire, n, (uint8_t)bri, &c, &wc);
}
/* the engine: kind 0 one segment, 1 reverse, 2 mirror, 3 four overlapping segments, 4 two crossfades running */
void b_eng_setup(uint32_t id, uint32_t n, uint32_t kind, uint32_t id2)
{
    fx_params_t p;
    uint32_t s;
    fxv_engine_init(&E, n, 3, arena, slab);
    params(&p, id);
    if (kind == 1u) p.flags = FX_F_REVERSE;
    if (kind == 2u) p.flags = FX_F_MIRROR;
    fxv_engine_set(&E, 0, &p, 1000, 1000);
    if (kind >= 3u) {
        for (s = 1; s < 4u; s++) fxv_engine_set(&E, s, &p, 1000, 1000);
    }
    fxv_engine_frame(&E, 1500);
    if (kind == 4u) {                                    /* segments 0 and 1 fade to id2 */
        params(&p, id2);
        p.trans_ms = 5000;
        fxv_engine_set(&E, 0, &p, 1600, 1600);
        fxv_engine_set(&E, 1, &p, 1600, 1600);
        fxv_engine_frame(&E, 1700);
    }
}
void b_eng_frame_begin(uint32_t now)
{
    uint32_t s;
    for (s = 0; s < FXV_MAX_SEG; s++) {
        (void)fxv_engine_dequeue(&E, s, now);
        fxv_engine_frame_seg(&E, s, now);
    }
}
uint32_t b_eng_step(void) { return fxv_engine_step(&E); }
void b_eng_render(uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i += FXV_CHUNK) fxv_engine_render(&E, px + i, i, i + FXV_CHUNK < n ? i + FXV_CHUNK : n);
}
/* the integration gate (tools/measure_fxv_slices.py): effects on the engine at a control corner (0 defaults, 1 all
 * controls 255, 2 speed 255 + the rest 0, 3 speed 1 + the rest 255) and the receiver's render + pack step as udfx.c runs
 * it: fxv_engine_render then fxv_pack16 of the same pixels (dithered, brightness 200, accurate white) */
static fx_pack_cfg_t SC;
void b_eng_init(uint32_t n, uint32_t bpp)
{
    static const uint8_t ords[3][5] = { { 1, 0, 2, 0, 0 }, { 1, 0, 2, 3, 0 }, { 0, 1, 2, 3, 4 } };
    if (!seeded) { fxv_white_init(&wc, FXV_K_W_DEFAULT, FXV_K_WW_DEFAULT, FXV_K_CW_DEFAULT); seeded = 1; }
    fxv_engine_init(&E, n, bpp, arena, slab);
    memset(&SC, 0, sizeof SC);
    SC.gamma = 1; SC.bpp = (uint8_t)bpp; memcpy(SC.ord, ords[bpp - 3u], 5); SC.white_mode = FX_W_ACCURATE; SC.white_split = 128;
    SC.smooth = 2; SC.phase = 77; SC.dmin = 4;
}
/* a segment's PIXEL_FX in two calls, so the engine's own set is timed alone (b_eng_set: one receiver step, ux_apply):
 * b_eng_params builds it (look: bits 0-1 the segment's FX_F_REVERSE / FX_F_MIRROR, bits 8-15 its brightness, 0: 255) */
static fx_params_t P;
void b_eng_params(uint32_t id, uint32_t mode, uint32_t start_len, uint32_t look)
{
    const uint32_t m = mode & 255u;          /* (bits 8+: the crossfade's trans_ms) */
    params(&P, id);
    corner(&P, m);
    P.seg_start = (uint16_t)(start_len >> 16); P.seg_len = (uint16_t)start_len;
    P.trans_ms = (uint16_t)(mode >> 8);
    P.flags = (uint8_t)(look & (FX_F_REVERSE | FX_F_MIRROR));
    if ((look >> 8) & 255u) P.bri = (uint8_t)(look >> 8);
}
void b_eng_set(uint32_t s, uint32_t now) { fxv_engine_set(&E, s, &P, now, now); }
/* the receiver's frame start in its own steps (udfx.c ux_prepare): each segment's dequeue, then each one's frame start */
uint32_t b_eng_dequeue(uint32_t s, uint32_t now) { return fxv_engine_dequeue(&E, s, now); }
void b_eng_seg(uint32_t s, uint32_t now) { fxv_engine_frame_seg(&E, s, now); }
uint32_t b_eng_active(uint32_t s) { return E.seg[s].active; }
void b_eng_whole(uint32_t now) { fxv_engine_frame(&E, now); }
uint32_t b_eng_span(uint32_t from, uint32_t to) { return fxv_engine_span(&E, from, to); }
/* segment s's look in place (the render only; the effect keeps its state): b_eng_params' look bits */
void b_eng_look(uint32_t s, uint32_t look)
{
    E.seg[s].flags = (uint8_t)((E.seg[s].flags & ~(FX_F_REVERSE | FX_F_MIRROR)) | (look & FX_F_REVERSE));
    E.seg[s].bri = (uint8_t)((look >> 8) & 255u ? (look >> 8) & 255u : 255u);
}
uint32_t b_eng_cost(void) { return fxv_engine_cost(&E); }
uint32_t b_eng_tier(void) { return E.tier; }
/* context k (segment k / 2, its fading-out one when k is odd): ready | id << 8 | stage << 16 | tier << 24; ~0: none */
uint32_t b_eng_ctx(uint32_t k)
{
    const fxv_seg_t *sg = &E.seg[k >> 1];
    const fxv_ctx_t *x = (k & 1u) ? (sg->xf >= 0 ? &E.xf[sg->xf].prev : 0) : (sg->active ? &sg->cur : 0);
    if (!x || !x->d) return 0xFFFFFFFFu;
    return fxv_ready(x) | (uint32_t)x->id << 8 | (uint32_t)x->stage << 16 | (uint32_t)x->tier << 24;
}
/* rebal | moving << 8 | quoting << 9 | segment s's frame start due << (16 + s) | its fading one's << (20 + s) */
uint32_t b_eng_flags(void)
{
    uint32_t f = E.rebal | (uint32_t)E.moving << 8 | (uint32_t)E.quoting << 9, s;
    for (s = 0; s < FXV_MAX_SEG; s++) {
        if (E.seg[s].due) f |= 1u << (16u + s);
        if (E.seg[s].xf >= 0 && (E.xf[E.seg[s].xf].used & 4u)) f |= 1u << (20u + s);
    }
    return f;
}
void b_slice(uint32_t from, uint32_t cnt)
{
    fxv_engine_render(&E, px, from, from + cnt);
    SC.px0 = (uint16_t)from;
    (void)fxv_pack16(px, wire + from * SC.bpp, cnt, 200, &SC, &wc);
}
/* the legacy packer on the same pixels (8-bit), for comparison */
uint32_t b_pack_legacy(uint32_t n, uint32_t bpp, uint32_t smooth, uint32_t bri)
{
    static const uint8_t ords[3][5] = { { 1, 0, 2, 0, 0 }, { 1, 0, 2, 3, 0 }, { 0, 1, 2, 3, 4 } };
    static rgb_t in8[FXV_N_MAX];
    fx_pack_cfg_t c;
    uint32_t i;
    memset(&c, 0, sizeof c);
    c.gamma = 1; c.bpp = (uint8_t)bpp; memcpy(c.ord, ords[bpp - 3u], 5); c.white_mode = FX_W_ACCURATE; c.white_split = 128;
    c.smooth = (uint8_t)smooth; c.phase = 77; c.px0 = 0; c.dmin = 4;
    for (i = 0; i < n; i++) in8[i] = fx_rgb(i * 37u, i * 91u, i * 13u);
    return fx_pack(in8, wire, n, (uint8_t)bri, &c);
}
