/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Entry points for tools/measure_fx.py: the effect engine built for the CH32V203 (RV32IMC) and run in tools/rv32emu.py
 * (an instruction-class cycle model of the QingKe V4). Static buffers, no start-up code: the emulator calls each function. */
#include "../src/core/fx.h"

#define NPX 300u

typedef struct { uint8_t id, speed, intensity, palette; uint8_t c1[3], c2[3], c3[3]; uint8_t custom1, custom2; } defaults_t;
static const defaults_t DEF[] = {
#include "fx_defaults.inc"
};

static rgb_t px[NPX];
static uint8_t fb[NPX * 5u], st[NPX * 2u];
static fx_ctx_t X;
static fx_engine_t E;
static fx_params_t P, Q;
static fx_pack_cfg_t C;

void *memset(void *d, int c, unsigned n);
void *memcpy(void *d, const void *s, unsigned n);
void *memset(void *d, int c, unsigned n) { uint8_t *p = d; while (n--) *p++ = (uint8_t)c; return d; }
void *memcpy(void *d, const void *s, unsigned n) { uint8_t *p = d; const uint8_t *q = s; while (n--) *p++ = *q++; return d; }

static void params(uint32_t id, fx_params_t *p)
{
    uint32_t k;
    const defaults_t *d = DEF;
    for (k = 0; k < sizeof(DEF) / sizeof(DEF[0]); k++) {
        if (DEF[k].id == id) {
            d = &DEF[k];
        }
    }
    p->id = d->id; p->speed = d->speed; p->intensity = d->intensity; p->palette = d->palette;
    p->c[0].r = d->c1[0]; p->c[0].g = d->c1[1]; p->c[0].b = d->c1[2];
    p->c[1].r = d->c2[0]; p->c[1].g = d->c2[1]; p->c[1].b = d->c2[2];
    p->c[2].r = d->c3[0]; p->c[2].g = d->c3[1]; p->c[2].b = d->c3[2];
    p->bri = 255; p->flags = 0; p->custom1 = d->custom1; p->custom2 = d->custom2;
    p->seg_start = 0; p->seg_len = 0; p->trans_ms = 0; p->vstart = 0; p->vlen = 0; p->seed = 0xC0FFEEu;
}

/* ------------------------------------------------------------------ one effect */
uint32_t b_begin(uint32_t id, uint32_t n)
{
    params(id, &P);
    fx_begin(&X, &P, (uint16_t)n, st);
    return 0;
}
uint32_t b_frame(uint32_t t)                       /* fx_frame only (the per-frame prep) */
{
    fx_frame(&X, t);
    return X.u[0];
}
uint32_t b_render(uint32_t n)                      /* the pixels, in chunks of 32 as the receiver renders them */
{
    uint32_t a;
    for (a = 0; a < n; a += FX_CHUNK) {
        fx_render(&X, px, a, a + FX_CHUNK < n ? a + FX_CHUNK : n);
    }
    return px[0].r;
}

/* ------------------------------------------------------------------ the packer */
uint32_t b_pack(uint32_t n, uint32_t bpp, uint32_t smooth, uint32_t bri)
{
    static const uint8_t ord[5] = { 1, 0, 2, 3, 4 };
    uint32_t k;
    for (k = 0; k < n; k++) {
        px[k].r = (uint8_t)(k * 7u); px[k].g = (uint8_t)(k * 13u + 40u); px[k].b = (uint8_t)(k * 29u + 3u);
    }
    C.gamma = 1;
    C.smooth = (uint8_t)smooth;
    C.phase = 77;
    C.bpp = (uint8_t)bpp;
    for (k = 0; k < 5u; k++) {
        C.ord[k] = ord[k];
    }
    C.white_mode = FX_W_ACCURATE;
    C.white_split = 100;
    return fx_pack(px, fb, n, (uint8_t)bri, &C);
}
uint32_t b_limit(uint32_t sum)
{
    return fx_limit_bri(255, sum, 1000000u);
}

/* ------------------------------------------------------------------ the engine: kind 0 plain, 1 reverse, 2 mirror,
 * 3 two segments, 4 plain with a crossfade running, 5 plain with a brightness ramp running */
uint32_t b_eng_setup(uint32_t id, uint32_t n, uint32_t kind)
{
    fx_engine_init(&E, (uint16_t)n, st, sizeof(st));
    params(id, &P);
    if (kind == 1) { P.flags = FX_F_REVERSE; }
    if (kind == 2) { P.flags = FX_F_MIRROR; }
    if (kind == 3) { P.seg_len = (uint16_t)(n / 2u); }
    fx_engine_set(&E, 0, &P, 0, 0);
    if (kind == 3) {
        Q = P;
        Q.seg_start = (uint16_t)(n / 2u);
        Q.seg_len = 0;
        Q.id = FX_RAINBOW;
        fx_engine_set(&E, 1, &Q, 0, 0);
    }
    if (kind == 4) {
        Q = P;
        Q.id = FX_RAINBOW;
        Q.trans_ms = 60000;
        fx_engine_set(&E, 0, &Q, 10, 0);
    }
    if (kind == 5) {
        Q = P;
        Q.bri = 100;
        Q.trans_ms = 60000;
        fx_engine_set(&E, 0, &Q, 10, 0);
    }
    return 0;
}
uint32_t b_eng_frame(uint32_t t)
{
    fx_engine_frame(&E, t);
    return E.seg[0].bri;
}
uint32_t b_eng_render(uint32_t n)
{
    uint32_t a;
    for (a = 0; a < n; a += FX_CHUNK) {
        fx_engine_render(&E, px, a, a + FX_CHUNK < n ? a + FX_CHUNK : n);
    }
    return px[0].r;
}

/* the RAM the engine's types take (printed by tools/measure_fx.py) */
uint32_t b_sizeof(uint32_t which)
{
    switch (which) {
    case 0:  return sizeof(fx_ctx_t);
    case 1:  return sizeof(fx_seg_t);
    case 2:  return sizeof(fx_xfade_t);
    case 3:  return sizeof(fx_engine_t);
    default: return sizeof(fx_params_t);
    }
}

/* the engine rendered in calls of `chunk` pixels (the receiver's race-the-beam renderer calls it with the few pixels a tick frees) */
uint32_t b_eng_render_chunks(uint32_t n, uint32_t chunk)
{
    uint32_t a;
    for (a = 0; a < n; a += chunk) {
        fx_engine_render(&E, px, a, a + chunk < n ? a + chunk : n);
    }
    return px[0].r;
}
