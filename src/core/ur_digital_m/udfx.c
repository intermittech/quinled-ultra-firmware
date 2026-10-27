/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "../math32.h"
/* The Diff-Power-UR-Digital-M's local pixel effects: see udfx.h. */
#include <stddef.h>
#include <string.h>
#include "udfx.h"
#include "../../board/boards.h"
#include "udpack.h"

#if defined(__GNUC__)
#define UX_BARRIER() __asm volatile("" ::: "memory")
#else
#define UX_BARRIER() ((void)0)
#endif

#if FX_V407
/* two wire frames (the encoder sends one, the renderer builds the other), the six context slots' arena, the particle slab */
#if defined(__GNUC__)
uint8_t ux_frames[2][UX_POOL_BYTES] __attribute__((aligned(16)));
static uint8_t ux_arena[FXV_ARENA_BYTES] __attribute__((aligned(16)));
#elif defined(_MSC_VER)
__declspec(align(16)) uint8_t ux_frames[2][UX_POOL_BYTES];
__declspec(align(16)) static uint8_t ux_arena[FXV_ARENA_BYTES];
#else
uint8_t ux_frames[2][UX_POOL_BYTES];
static uint8_t ux_arena[FXV_ARENA_BYTES];
#endif
uint8_t ux_wr;
static fxv_particle_t ux_slab[FXV_PARTICLES];
fxv_white_t ux_white;
#if defined(__GNUC__) && defined(__riscv)
fxv_engine_t ux_eng __attribute__((section(".bss.gp.2eng")));   /* (gp's window: main_ur_digital_m.c, after dl and ux) */
#else
fxv_engine_t ux_eng;
#endif
#define UX_SEG_T                 fxv_seg_t
#define ux_eng_set(s, q, n, a)   fxv_engine_set(&ux_eng, (s), (q), (n), (a))
#define ux_eng_step()            fxv_engine_step(&ux_eng)
#define ux_eng_dequeue(s, n)     fxv_engine_dequeue(&ux_eng, (s), (n))
#define ux_eng_frame_seg(s, n)   fxv_engine_frame_seg(&ux_eng, (s), (n))
#define ux_eng_dark()            fxv_engine_dark(&ux_eng)
#define UX_PAL_COUNT             FXV_PAL_COUNT
#else
#if defined(__GNUC__)
uint8_t ux_pool[UX_POOL_BYTES] __attribute__((aligned(4)));
#elif defined(_MSC_VER)
__declspec(align(4)) uint8_t ux_pool[UX_POOL_BYTES];
#else
uint8_t ux_pool[UX_POOL_BYTES];
#endif
static uint8_t ux_state[UX_STATE_BYTES];                 /* the engine's state pool: Fire's heat */
#if defined(__GNUC__) && defined(__riscv)
fx_engine_t ux_eng __attribute__((section(".bss.gp.2eng")));   /* (gp's window: main_ur_digital_m.c, after dl and ux) */
#else
fx_engine_t ux_eng;
#endif
#define UX_SEG_T                 fx_seg_t
#define ux_eng_set(s, q, n, a)   fx_engine_set(&ux_eng, (s), (q), (n), (a))
#define ux_eng_step()            fx_engine_step(&ux_eng)
#define ux_eng_dequeue(s, n)     fx_engine_dequeue(&ux_eng, (s), (n))
#define ux_eng_frame_seg(s, n)   fx_engine_frame_seg(&ux_eng, (s), (n))
#define ux_eng_dark()            fx_engine_dark(&ux_eng)
#define UX_PAL_COUNT             PAL_COUNT
#endif
#if defined(__GNUC__) && defined(__riscv)
/* out of the small data: the encoder takes its address once per tick, while the 384 bytes pushed the small data past gp's
 * window and every initialised variable out of it (one more instruction per access) */
ws_lut_t ux_lut __attribute__((section(".bss.ux_lut")));
#else
ws_lut_t ux_lut;
#endif

/* ------------------------------------------------------------------------------------------------ the chips */
#ifndef MCU_SYM_T1_V5
#define MCU_SYM_T1_V5 MCU_SYM_NS(694u)
#endif
typedef struct {
    uint8_t  s, t0h, t1h, bpp;            /* the 36 MHz sample grid (docs: tools/ur_digital_m_timing_check.py), bytes per pixel */
    uint16_t reset_w;                     /* LOW words between frames */
    uint16_t latch_w;                     /* LOW words after which the strip has surely latched, the HSI 2.2 % fast */
    uint8_t  ord[5];                      /* the default wire order (canonical channels R 0, G 1, B 2, W 3, CW 4) */
} ux_chip_t;
typedef char ux_chip_prefix[(sizeof(ws_chip_t) == 6u && offsetof(ws_chip_t, bpp) == 3u && offsetof(ws_chip_t, reset_w) == 4u &&
                             offsetof(ux_chip_t, bpp) == 3u && offsetof(ux_chip_t, reset_w) == 4u) ? 1 : -1];

/* resets (words of 0.444 us), each above the chip's minimum with the HSI 2.2 % fast (tools/ur_digital_m_timing_check.py): the V5 class
 * > 280 us: 680 = 302 us; the SK6812 Rev B/1 > 200 us: 480 = 213 us; the older chips 200 us. The latch: the least reset the
 * chip family needs (the SK6812 Rev 02's 80 us: a hub at the Rev B/1's 200 us is no less welcome), in words with the HSI 2.2 %
 * fast and a margin (checked by the same tool): a LOW run that long latches the strip, so a mode switch inside it never cuts a
 * frame (a shorter LOW can be inside a controller's frame: an RMT refill) */
static const ux_chip_t ux_chips[ULTRA_PXC_COUNT] = {
    { MCU_SYM_BIT, MCU_SYM_NS(361u), MCU_SYM_T1_V5, 3, MCU_NS2W_NEAREST(302222u), MCU_NS2W_NEAREST(288889u), { 1, 0, 2, 0, 0 } },   /* WS2812B V5 / 2020: 361 / 694 ns */
    { MCU_SYM_BIT, MCU_SYM_NS(361u), MCU_SYM_NS(778u), 3, MCU_NS2W_NEAREST(200000u), MCU_NS2W_NEAREST(53333u), { 1, 0, 2, 0, 0 } },   /* WS2812B classic: 361 / 778 ns */
    { MCU_SYM_BIT, MCU_SYM_NS(333u), MCU_SYM_NS(722u), 3, MCU_NS2W_NEAREST(213333u), MCU_NS2W_NEAREST(84444u), { 1, 0, 2, 0, 0 } },   /* SK6812 RGB: 333 / 722 ns */
    { MCU_SYM_BIT, MCU_SYM_NS(333u), MCU_SYM_NS(722u), 4, MCU_NS2W_NEAREST(213333u), MCU_NS2W_NEAREST(84444u), { 1, 0, 2, 3, 0 } },   /* SK6812 RGBW */
    { MCU_SYM_BIT, MCU_SYM_NS(361u), MCU_SYM_T1_V5, 4, MCU_NS2W_NEAREST(302222u), MCU_NS2W_NEAREST(288889u), { 0, 1, 2, 3, 0 } },   /* WS2814 RGBW (V5 timing) */
    { MCU_SYM_BIT, MCU_SYM_NS(361u), MCU_SYM_T1_V5, 5, MCU_NS2W_NEAREST(302222u), MCU_NS2W_NEAREST(288889u), { 0, 1, 2, 3, 4 } },   /* WS2805 RGBCCT */
    { MCU_SYM_BIT, MCU_SYM_NS(361u), MCU_SYM_T1_V5, 3, MCU_NS2W_NEAREST(302222u), MCU_NS2W_NEAREST(288889u), { 1, 0, 2, 0, 0 } },   /* WS2815 (V5 timing assumed) */
};

static uint32_t ux_cfg_get(const udfx_t *u, uint32_t id)
{
    return u->cfg[id - ULTRA_CFG_PX_FIRST];
}

/* AUTO's detected stream owns the strip, independent of saved local settings. */
UX_AINLINE uint32_t ux_auto_stream(const udfx_t *u)
{
    return u->stream && ux_cfg_get(u, ULTRA_CFG_PX_LINE_MODE) == ULTRA_PXLM_AUTO;
}

/* the stream comes back on the strip: a configured SOURCE, or AUTO's detected stream (three cold sites; udfx_service
 * keeps its own inline test) */
static NOINLINE uint32_t ux_replay(const udfx_t *u)
{
    return ux_cfg_get(u, ULTRA_CFG_PX_SOURCE) || ux_auto_stream(u);
}

/* the configured chip's bytes per pixel (CONFIG 64; an id out of range reads as chip 0, as ux_derive) */
static NOINLINE uint32_t ux_cfg_bpp(const udfx_t *u)
{
    const uint32_t id = ux_cfg_get(u, ULTRA_CFG_PX_CHIP);
    return ux_chips[id < ULTRA_PXC_COUNT ? id : 0u].bpp;
}

/* a wire order over bpp channels: each channel < bpp (5: the five), no repeat. 0 (the default) is not one */
static uint32_t ux_order_ok(uint32_t v, uint32_t bpp)
{
    uint32_t k, seen = 0;
    for (k = 0; k < bpp; k++) {
        uint32_t c = (v >> (3u * k)) & 7u;
        if (c >= bpp || (seen & (1u << c))) {
            return 0;
        }
        seen |= 1u << c;
    }
    return 1;
}

static NOINLINE uint32_t ux_max_px(uint32_t bpp)   /* (two sites) */
{
    return bpp == 3u ? ULTRA_PX_MAX_RGB : bpp == 4u ? ULTRA_PX_MAX_RGBW : ULTRA_PX_MAX_RGBCCT;
}

#if FX_V407
/* the frame grid: the strip's wire time (bits x the chip's bit period) + its reset + UX_GUARD_US, but at least
 * 1 / FXV_FPS_MAX s and 1 / the CONFIG 67 ceiling (0: none): short strips run up to 100 fps, the longest ~39-41 */
static uint32_t ux_grid_us(const udfx_t *u, uint32_t fps)
{
    const uint32_t wire = ((uint32_t)u->npx * u->chip.bpp * 8u * u->chip.s + 99u) / 100u;   /* (s: 10 ns units) */
    const uint32_t need = wire + u->chip.reset_w * MCU_US_PER_WORD_NUM / MCU_US_PER_WORD_DEN + UX_GUARD_US;
    const uint32_t cap = 1000000u / (fps != 0u && fps < FXV_FPS_MAX ? fps : FXV_FPS_MAX);
    return need > cap ? need : cap;
}
#endif

/* the configuration into the derived fields and the engine (not while LOCAL: the tick uses the tables and the encoder) */
static void ux_derive(udfx_t *u)
{
    const uint32_t id = ux_cfg_get(u, ULTRA_CFG_PX_CHIP);
    const ux_chip_t *c = &ux_chips[id < ULTRA_PXC_COUNT ? id : 0u];
    const uint32_t ord = ux_cfg_get(u, ULTRA_CFG_PX_ORDER), fps = ux_cfg_get(u, ULTRA_CFG_PX_FPS), w = ux_cfg_get(u, ULTRA_CFG_PX_WHITE);
    uint32_t k, px = ux_cfg_get(u, ULTRA_CFG_PX_PIXELS);
#if FX_V407
    /* (field by field, never merged: the compiler would join the timing bytes into halfword / word accesses at odd or
       2 mod 4 offsets here and in ws_lut_build: the misaligned-site scan) */
    u->chip.s = c->s;
    u->chip.t0h = c->t0h;
    UX_BARRIER();
    u->chip.t1h = c->t1h;
    UX_BARRIER();
    u->chip.bpp = c->bpp;
    u->chip.reset_w = c->reset_w;
#else
    memcpy(&u->chip, c, sizeof u->chip);                 /* s, t0h, t1h, bpp, reset_w: ux_chip_t's first fields */
#endif
    u->latch_w = c->latch_w;
    u->pack.gamma = 1;
    u->pack.bpp = c->bpp;
    for (k = 0; k < 5u; k++) {
        u->pack.ord[k] = (ord != 0u && ux_order_ok(ord, c->bpp)) ? (uint8_t)((ord >> (3u * k)) & 7u) : c->ord[k];
    }
    u->white_def = (uint8_t)((w & 3u) != 0u ? (w & 3u) : FX_W_ACCURATE);
    u->pack.white_mode = u->white_def;
    u->pack.white_split = (uint8_t)(((w >> 8) & 0xFFu) != 0u ? (w >> 8) & 0xFFu : 128u);
    u->max_px = (uint16_t)ux_max_px(c->bpp);
    u->npx = (uint16_t)(px > u->max_px ? u->max_px : px);
#if FX_V407
    u->period_us = ux_grid_us(u, fps);
    u->period_w = m32_shr(u->period_us,u->wpu24,24);
    ws_lut_build(&ux_lut, &u->chip);
    fxv_engine_init(&ux_eng, u->npx, c->bpp, ux_arena, ux_slab);   /* frame setup restores running segments */
#else
    u->period_us = fps != 0u ? 1000000u / fps : 0u;
    u->period_w = fps != 0u ? m32_shr(u->period_us,u->wpu24,24) : 0u;
    ws_lut_build(&ux_lut, &u->chip);
    fx_engine_init(&ux_eng, u->npx, ux_state, sizeof ux_state);   /* frame setup restores running segments */
    ux_eng.deferred = 1;
#endif
}

void udfx_init(udfx_t *u)
{
    uint8_t *p = (uint8_t *)u;
    uint32_t i;
    for (i = 0; i < (uint32_t)sizeof *u; i++) {
        p[i] = 0;
    }
    u->wq24 = UX_RATE_NOM;
    u->wpu24 = m32_div(65536u,0,UX_RATE_NOM);
    u->lim_bri = 255;
    u->cl_q8 = 256;
    u->input_limit_ma = ISENSE_BOARD_MA;
    u->cfg[ULTRA_CFG_PX_MAX-ULTRA_CFG_PX_FIRST] = BOARD_OUTPUT_RATING_MA;
    u->cfg[ULTRA_CFG_PX_MA_CH-ULTRA_CFG_PX_FIRST] = 20;
#if FX_V407
    u->cfg[ULTRA_CFG_PX_FPS - ULTRA_CFG_PX_FIRST] = 0;   /* as fast as the strip allows, up to FXV_FPS_MAX */
    fxv_white_init(&ux_white, FXV_K_W_DEFAULT, FXV_K_WW_DEFAULT, FXV_K_CW_DEFAULT);
#else
    u->cfg[ULTRA_CFG_PX_FPS - ULTRA_CFG_PX_FIRST] = ULTRA_PX_DEFAULT_FPS;
#endif
    ux_derive(u);
}

/* ------------------------------------------------------------------------------------------------ the clock */
uint32_t udfx_host_us(const udfx_t *u, uint32_t w)
{
    return u->a_host + m32_sar((int32_t)(w-u->a_w),u->wq24,24);
}

uint32_t udfx_word_at(const udfx_t *u, uint32_t host_us)
{
    return u->a_w + m32_sar((int32_t)(host_us-u->a_host),u->wpu24,24);
}

/* the engine's host ms: a count that goes on across the 32-bit us wrap, for a host time near the reference. The reference
 * (host us, a multiple of 1000, and its ms) moves with the time (ux_ms_advance), so differences of ms are floor(us / 1000)
 * differences: the same on every receiver that gets the same host times */
static int32_t ux_ms_delta(int32_t d)
{
    /* Floor signed us to ms without negating INT32_MIN or adding 999 to it. */
    return d / 1000 - (d % 1000 < 0);
}

static NOINLINE uint32_t ux_ms(const udfx_t *u, uint32_t host_us)   /* (three sites: one copy) */
{
    const int32_t d = (int32_t)(host_us - u->hus_ref);
    return u->hms_ref + (uint32_t)ux_ms_delta(d);
}

/* the reference moved towards host_us when it is half a second or more away, and the host ms of host_us (= ux_ms after the
 * move, with the one floor division: the moved reference leaves no whole ms, d - 1000 q is 0 .. 999) */
static uint32_t ux_ms_advance(udfx_t *u, uint32_t host_us)
{
    const int32_t d = (int32_t)(host_us - u->hus_ref);
    const int32_t q = ux_ms_delta(d);
    if (d >= 500000 || d <= -500000) {
        u->hms_ref += (uint32_t)q;
        u->hus_ref += (uint32_t)q * 1000u;
        return u->hms_ref;
    }
    return u->hms_ref + (uint32_t)q;
}

uint32_t udfx_host_of(udfx_t *u, uint32_t time)
{
    u->t_host = u->t_have ? u->t_host + (uint32_t)ultra_time_sub(time, u->t_time) : time;
    u->t_have = 1;
    u->t_time = time;
    return u->t_host;
}

void udfx_anchor(udfx_t *u, uint32_t host_us, uint32_t w)
{
    uint32_t dh, since, pred;
    int32_t err;
    uint32_t stepped = 0;
    if (!u->a_have) {                                    /* the first: the clock is the host's from here */
        if (u->standalone) {                             /* a stored effect has run on the free-running clock: its ms count
                                                            moves with the clock's step, the effect goes on without a jump */
            u->hus_ref += host_us - udfx_host_us(u, w);
        }
        if (u->t_have) {                                 /* the ms count onto the host's ms grid (the TIME wrap is whole ms:
                                                            TIME's ms are the host's), at a whole host ms near the header: every
                                                            board then adds a START to the same host ms, whatever its boot */
            const uint32_t g = u->t_host - u->t_time % 1000u;
            u->hms_ref = ux_ms(u, g);
            u->hus_ref = g;
        }
        u->a_have = 1;
        u->a_host = host_us;
        u->a_w = w;
        u->r_host = host_us;
        u->r_w = w;
        return;
    }
    pred = udfx_host_us(u, w);                           /* (the mapping as it was, before the rate changes) */
    err = (int32_t)(host_us - pred);
    dh = host_us - u->r_host;
    if (dh >= 1000000u && dh <= 30000000u) {             /* the rate from two anchors >= 1 s apart */
        uint32_t dw = w - u->r_w;
        uint32_t q=dw?m32_divsat(dh>>8,dh<<24,dw):0u;
        /* within 1/16 of nominal; once known, within 0.3 % of the rate it was (the HSI drifts slowly: a pair that spans a
           jump of the hub's clock is not a rate) */
        if (q - (UX_RATE_NOM - UX_RATE_NOM / 16u) < UX_RATE_NOM / 8u &&
            (!u->r_have || q - (u->wq24 - u->wq24 / 330u) < u->wq24 / 165u)) {
            stepped = !u->r_have;                        /* (the first measurement: the clock steps, no history) */
            u->wq24 = (uint32_t)q;
            u->wpu24 = m32_div(65536u,0,q);
            u->r_have = 1;
            if (FX_V407 || ux_cfg_get(u, ULTRA_CFG_PX_FPS) != 0u) {   /* (V407: always on a grid) */
                u->e.period_w = u->period_w = m32_shr(u->period_us,u->wpu24,24);
            }
        }
        u->r_host = host_us;
        u->r_w = w;
    } else if (dh > 30000000u) {
        u->r_host = host_us;
        u->r_w = w;
    }
    since = w - u->a_w;                                  /* words since the last anchor: slew by 0.5 % of its time */
    if (stepped || (err > (int32_t)UX_STEP_US) || (err < -(int32_t)UX_STEP_US)) {
        u->a_host = host_us;
    } else {
        int32_t lim = (int32_t)(since / (200u * MCU_US_PER_WORD_DEN / MCU_US_PER_WORD_NUM));   /* 0.5 %: since x us per word / 200 (450 at 36 MHz) */
        int32_t adj = err > lim ? lim : err < -lim ? -lim : err;
        u->a_host = pred + (uint32_t)adj;
    }
    u->a_w = w;
}

/* ------------------------------------------------------------------------------------------------ the commands */
/* the PIXEL_FX slot's fields as the engine's parameters (CONFIG 71 / 72: the virtual strip). The first 17 bytes (effect .. custom2:
 * ids, the three colours, brightness, flags, custom1, custom2) have the same layout in both, and the two flag bits the engine takes are
 * the slot's own: one copy, then the palette's and the flags' mapping */
typedef char ux_params_prefix[(offsetof(fx_params_t, c) == 4u && offsetof(fx_params_t, bri) == 13u && offsetof(fx_params_t, custom2) == 16u &&
    offsetof(ultra_pixel_fx_t, c) == 4u && offsetof(ultra_pixel_fx_t, brightness) == 13u && offsetof(ultra_pixel_fx_t, custom2) == 16u &&
    FX_F_REVERSE == ULTRA_PXF_REVERSE && FX_F_MIRROR == ULTRA_PXF_MIRROR) ? 1 : -1];
#if defined(__GNUC__)
typedef uint32_t __attribute__((may_alias)) ux_word_t;
#else
typedef uint32_t ux_word_t;
#endif
static void ux_params(const udfx_t *u, uint32_t si, const ultra_pixel_fx_t *f, fx_params_t *q)
{
    const ux_word_t *src = (const ux_word_t *)(const void *)f;   /* four words and a byte (both structs are 4-byte aligned; a
                                                                     memcpy of 17, or a loop of four, is a library call) */
    ux_word_t *dst = (ux_word_t *)(void *)q;
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
    q->custom2 = f->custom2;
    q->palette = f->palette == ULTRA_PX_PAL_COLORS ? (uint8_t)PAL_COLORS
               : (f->palette == ULTRA_PX_PAL_DEFAULT || f->palette >= UX_PAL_COUNT) ? (uint8_t)PAL_DEFAULT : f->palette;
    q->flags = (uint8_t)(f->flags & (FX_F_REVERSE | FX_F_MIRROR));
    q->seg_start = f->seg_start;
    q->seg_len = f->seg_len;
    q->trans_ms = f->transition_ms;
    q->vstart = (uint16_t)ux_cfg_get(u, ULTRA_CFG_PX_VOFFSET);
    q->vlen = (uint16_t)ux_cfg_get(u, ULTRA_CFG_PX_VLENGTH);
    q->seed = (uint32_t)f->custom2 * 2654435761u + 1u + (uint32_t)u->seg[si].restarts * 0x9E3779B1u;   /* (the engine adds the segment) */
}

static void ux_stream(udfx_t *u)
{
    fx_params_t q;
    uint32_t i;
    for (i = 0; i < sizeof q; i++) {
        ((uint8_t *)&q)[i] = 0;
    }
    q.id = FX_STREAM;
    u->stream_pending = 0;
    for (i = 0; i < UX_SEGS; i++) {
        u->seg[i].run = 0;
        u->seg[i].pend_have = 0;
        ux_eng_set(i, &q, 0, 0);                         /* (inactive) */
    }
    u->want_local = 0;
    UX_BARRIER();
    u->want_stream = (uint8_t)(!u->chain && ux_replay(u));   /* CHAIN has no replay */
    u->standalone = 0;
    u->dark = u->black_sent = 0;
}

/* f applied to segment si now: the engine ramps / crossfades from what it shows */
static void ux_apply(udfx_t *u, uint32_t si, const ultra_pixel_fx_t *f, uint32_t now_ms, uint32_t anchor_ms)
{
    fx_params_t q;
    ux_seg_t *s = &u->seg[si];
    if (f->flags & ULTRA_PXF_RESTART) {                  /* not a renewal: the engine sees another seed and takes the new anchor */
        s->restarts++;
    }
    ux_params(u, si, f, &q);
    ux_eng_set(si, &q, now_ms, anchor_ms);
    s->p = *f;
    s->p.flags &= (uint8_t)(0xFFu ^ ULTRA_PXF_RESTART);         /* (an action, not a setting) */
    {
        const UX_SEG_T *sg = &ux_eng.seg[si];
        /* Metadata describes s->p, including an A09 queued target. A CONFIG
         * rebuild must restore that target at its own anchor. */
        s->anchor = sg->xf >= 0 && (ux_eng.xf[sg->xf].used & 2u) ? anchor_ms : sg->anchor;
    }
    s->run = (uint8_t)(f->effect != FX_STREAM);
    u->dark = u->black_sent = 0;
}

static void ux_seg_cmd(udfx_t *u, uint32_t si, const ultra_pixel_fx_t *f, uint32_t hdr_us)
{
    ux_seg_t *s = &u->seg[si];
    s->pend = *f;
    s->pend.start_ms = (int32_t)(ux_ms(u, hdr_us) + (uint32_t)f->start_ms);
    s->pend_have = (uint8_t)(f->start_ms > 0 ? 2 : 1);
    u->dark = 0;                                        /* wake a dark LOCAL renderer to examine the queue */
}

static void ux_queue_stream(udfx_t *u)
{
    ultra_pixel_fx_t f = {0};
    uint32_t i;
    for (i = 0; i < UX_SEGS; i++) {
        ux_seg_cmd(u, i, &f, 0);
    }
    u->stream_pending = 1;
}

/* START is a host ms (signed milliseconds from the header's host ms, so a positive delay can exceed half a host-us wrap):
 * due from the first frame in that ms, the same frame on every board */
static uint32_t ux_due(const ux_seg_t *s, uint32_t ms)
{
    return s->pend_have && (s->pend_have == 1u || (int32_t)(ms - (uint32_t)s->pend.start_ms) >= 0);
}

/* a CRC-good PIXEL_FX slot into f (= ultra_pixel_fx_parse: its TARGET, or -1): the thirteen bytes speed .. brightness lie
 * the same way in the slot and in the structure (one copy), the rest field by field */
typedef char ux_parse_layout[(offsetof(ultra_pixel_fx_t, speed) == 1u && offsetof(ultra_pixel_fx_t, brightness) == 13u &&
    ULTRA_PX_INTENSITY == ULTRA_PX_SPEED + 1u && ULTRA_PX_PALETTE == ULTRA_PX_SPEED + 2u && ULTRA_PX_C1 == ULTRA_PX_SPEED + 3u &&
    ULTRA_PX_BRIGHTNESS == ULTRA_PX_SPEED + 12u) ? 1 : -1];
static int ux_parse(const uint8_t *s, ultra_pixel_fx_t *f)
{
    const uint32_t target = s[1];
    if ((s[0] & (ULTRA_TYPE_MASK | ULTRA_TYPE_BCAST)) != ULTRA_PIXEL_FX || (target >> 4) > ULTRA_PIXEL_OUTPUT_MAX) {
        return -1;
    }
    f->effect = s[ULTRA_PX_EFFECT];
    f->start_ms = (int32_t)ultra_get32(s + ULTRA_PX_START);
    memcpy(&f->speed, s + ULTRA_PX_SPEED, 13u);
    f->transition_ms = (uint16_t)ultra_get16(s + ULTRA_PX_TRANSITION);
    f->seg_start = (uint16_t)ultra_get16(s + ULTRA_PX_SEG_START);
    f->seg_len = (uint16_t)ultra_get16(s + ULTRA_PX_SEG_LEN);
    f->flags = (uint8_t)(s[ULTRA_PX_FLAGS] & 0x7Fu);
    f->custom1 = s[ULTRA_PX_CUSTOM1];
    f->custom2 = s[ULTRA_PX_CUSTOM2];
    return (int)target;
}

void udfx_pixel_fx(udfx_t *u, const uint8_t *slot, uint32_t hdr_us)
{
    ultra_pixel_fx_t f;
    int seg = ux_parse(slot, &f);
    uint32_t idx, i;
    if (seg < 0 || (seg >> 4) != 0 || ((seg & 15) >= UX_SEGS && (seg & 15) != 15)) {
        return;
    }
#if FX_V407
    if (f.effect != FX_STREAM && !fxv_built(f.effect)) return;   /* an effect this build does not run: never Solid */
#elif FX_NO_PACIFICA
    if(f.effect==FX_PACIFICA) return;
#endif
    idx = seg == 15 ? UX_SEGS : (uint32_t)seg;
    if (u->seq_have & (1u << idx) && u->seq5[idx] == slot[2]) {
        return;                                          /* applied once per SEQ */
    }
    u->seq_have = (uint8_t)(u->seq_have | (1u << idx));
    u->seq5[idx] = slot[2];
    if (seg == 15) {
        for (i=0;i<UX_SEGS;++i) {u->seq5[i]=slot[2];u->seq_have|=(uint8_t)(1u<<i);}
    }
    if (f.effect == ULTRA_PX_FX_STOP && ux_cfg_get(u,ULTRA_CFG_PX_SOURCE)) {
        if (f.flags & ULTRA_PXF_STORE) {
            u->stored_have = 0;
            u->save_due = 1;
            u->save_ms = u->ms;
        }
        ux_queue_stream(u);                            /* STREAM ignores START; STORE above is immediate */
        return;
    }
    u->lock = (uint8_t)((f.flags & ULTRA_PXF_LOCK) != 0u);
    u->standalone = 0;
    u->want_stream = 0;
    u->stream_pending = 0;
    if (f.flags & ULTRA_PXF_STORE) {
        const uint32_t k = seg == 15 ? 0u : (uint32_t)seg;
        u->stored[k] = f;
        u->stored[k].start_ms = 0;
        u->stored[k].flags &= (uint8_t)(0xFFu ^ ULTRA_PXF_RESTART);
        u->stored_have = (uint8_t)(seg == 15 ? 1u : (u->stored_have | (1u << k)));
        u->save_due = 1;
        u->save_ms = u->ms;
    }
    if (seg == 15) {                 /* the whole strip: segment 0, the others stopped */
        ultra_pixel_fx_t cancel = {0};
        cancel.start_ms = f.start_ms;
        for (i = 1; i < UX_SEGS; i++) {
            ux_seg_cmd(u, i, &cancel, hdr_us);
        }
        ux_seg_cmd(u, 0, &f, hdr_us);
    } else {
        ux_seg_cmd(u, (uint32_t)seg, &f, hdr_us);
    }
    /* (the switch into LOCAL is asked for by udfx_service, in the call that also computes the grid's first word: the tick
     * must not switch before it has one) */
}

uint32_t udfx_config(udfx_t *u, uint32_t id, uint32_t v)
{
    uint32_t k, was_auto_stream = ux_auto_stream(u);
    if (id < ULTRA_CFG_PX_FIRST || id >= ULTRA_CFG_PX_FIRST + ULTRA_CFG_PX_COUNT || v > 0xFFFFu) {
        return 0;
    }
    k = id - ULTRA_CFG_PX_FIRST;
    if (v == ULTRA_LIMIT_UNKNOWN && (id == ULTRA_CFG_PX_MA_CH || id == ULTRA_CFG_PX_MAX)) return 0;
    switch (id) {
    case ULTRA_CFG_PX_CHIP:
        if (v >= ULTRA_PXC_COUNT) {
            return 0;
        }
        break;
    case ULTRA_CFG_PX_ORDER:
        if (v != 0u && !ux_order_ok(v, ux_cfg_bpp(u))) {
            return 0;
        }
        break;
    case ULTRA_CFG_PX_FPS:
        if (v > 1000u) {
            return 0;
        }
        break;
    case ULTRA_CFG_PX_POLICY:
        v &= 7u;
        break;
    case ULTRA_CFG_PX_LOSS_TIMEOUT:
        if (v != 0u && (v < 5u || v > 600u)) {
            return 0;
        }
        break;
    case ULTRA_CFG_PX_LINE_MODE:
        if (v > ULTRA_PXLM_CHAIN) {
            return 0;
        }
        break;
    case ULTRA_CFG_PX_SMOOTH:
        if (v > ULTRA_PXS_OFF) {
            return 0;
        }
        break;
    default:
        break;
    }
    if (id == ULTRA_CFG_PX_PIXELS) {
        uint32_t max = ux_max_px(ux_cfg_bpp(u));
        if (v > max) v = max;
    }
    if (id == ULTRA_CFG_PX_MAX && (!v || v > BOARD_OUTPUT_RATING_MA)) v = BOARD_OUTPUT_RATING_MA;
    if (id == ULTRA_CFG_PX_MA_CH && !v) v = 20;
    if (id == ULTRA_CFG_PX_SOURCE && (v > 1u || (v && ux_cfg_get(u,ULTRA_CFG_PX_LINE_MODE) != ULTRA_PXLM_STREAM))) return 0;
    if (id == ULTRA_CFG_PX_LINE_MODE && v != ULTRA_PXLM_STREAM && ux_cfg_get(u,ULTRA_CFG_PX_SOURCE)) return 0;
    u->cfg_valid |= (uint16_t)(1u << k);
    if (u->cfg[k] == v) {
        return 0;
    }
    u->cfg[k] = (uint16_t)v;
    if (id == ULTRA_CFG_PX_SOURCE || (id == ULTRA_CFG_PX_LINE_MODE && was_auto_stream != ux_auto_stream(u))) {
        if(ux_replay(u)) ux_queue_stream(u);
        else {u->want_stream=0; u->rebegin=1;}
    }
    u->save_due = 1;
    u->save_ms = u->ms;
    u->dark = u->black_sent = 0;
    if (id == ULTRA_CFG_PX_LINE_MODE || id == ULTRA_CFG_PX_SMOOTH) {
        return 1;                                        /* (the glue, udchain, reads the line mode each ms; the smoothing is read at each frame) */
    }
    if (id <= ULTRA_CFG_PX_FPS || id == ULTRA_CFG_PX_WHITE) {
        u->reconf = 1;                                   /* the tables / encoder / pool: leave and re-enter local */
    } else if (id == ULTRA_CFG_PX_VOFFSET || id == ULTRA_CFG_PX_VLENGTH) {
        u->rebegin = 1;
    }
    return 1;
}

/* ------------------------------------------------------------------------------------------------ the renderer */
/* the frame about to start: its host time, the engine's frame, the white mode; 0: dark (nothing to render) */
static uint32_t ux_frame_begin(udfx_t *u)
{
    ws_enc_t *e = &u->e;
    uint32_t i, host, nw, now_ms;
    if (u->period_us != 0u) {                            /* on the host grid: frame k at host time k x P */
        nw = e->next_w;
        host = udfx_host_us(u, nw);
        {                                               /* round without overflowing before the division at host-us wrap */
            const uint32_t rem = host % u->period_us;
            host = host - rem + (rem >= (u->period_us + 1u) / 2u ? u->period_us : 0u);
        }
    } else {
        nw = e->w + (e->low_w < e->reset_w ? e->reset_w - e->low_w : 0u);
        host = udfx_host_us(u, nw);
    }
    {                                                    /* the output step (CONFIG 76): AUTO dithers from 50 frames a second; the phase is the host frame index */
        const uint32_t s = ux_cfg_get(u, ULTRA_CFG_PX_SMOOTH);
        const uint32_t fast = u->period_us != 0u ? u->period_us <= 1000000u / ULTRA_PXS_AUTO_FPS : u->fps10 >= 10u * ULTRA_PXS_AUTO_FPS;
        u->pack.smooth = (uint8_t)(s == ULTRA_PXS_OFF ? FX_SM_8BIT : s == ULTRA_PXS_ROUND ? FX_SM_ROUND : (s == ULTRA_PXS_DITHER || fast) ? FX_SM_DITHER : FX_SM_ROUND);
        u->pack.phase = fx_dither_phase(u->period_us != 0u ? host / u->period_us : u->frames_done);
        u->pack.dmin = (uint8_t)(s == ULTRA_PXS_AUTO ? ULTRA_PXS_AUTO_MIN_LEVEL : 0u);
    }
    now_ms = ux_ms(u, host);
    if (u->stream_pending) {
        u->stream_pending = 0;
        ux_stream(u);
        if (!u->chain) {
            return 0;
        }
    }
    for (i = 0; i < UX_SEGS; i++) {
        ux_seg_t *s = &u->seg[i];
        s->apply = (uint8_t)(u->rebegin && s->run ? 2 : 0);
        if (ux_due(s, now_ms)) {
            s->p = s->pend;
            s->anchor = (uint32_t)s->pend.start_ms;
            s->pend_have = 0;
            s->apply = 1;
        }
    }
    u->rebegin = 0;
    ux_eng.now = now_ms;
    u->rendered = 0;
#if FX_V407
    if (e->in_flight != ux_frames[ux_wr ^ 1u]) {         /* the other frame is not on the wire (ready is 0 here): build into it */
        ux_wr ^= 1u;
        e->fb_next = ux_frames[ux_wr];
    }
#endif
    u->begun = 2;                                       /* freeze commands together, then prepare them in bounded steps */
    return 1;
}

#if !FX_V407                                             /* (V407: every render step's own span, udfx_render) */
static void ux_chunk(udfx_t *u)
{
    u->chunk = (uint8_t)fx_engine_chunk(&ux_eng, u->chain ? 12u : FX_CHUNK);
}
#endif

static uint32_t ux_frame_ready(udfx_t *u)
{
    uint32_t i, m;
#if !FX_V407
    ux_chunk(u);
#endif
    u->frame_dark = (uint8_t)ux_eng_dark();
    if (u->frame_dark) {
        if (u->black_sent) {
            u->dark = 1;
            u->begun = 0;
            u->dark_at = u->ms;
            return 0;
        }
    } else {
        u->black_sent = 0;
    }
    m = u->white_def;                                    /* the first segment that names a mode wins (the engine packs one) */
    for (i = 0; i < UX_SEGS; i++) {
        const uint32_t f = (u->seg[i].p.flags & ULTRA_PXF_WHITE_MASK) >> ULTRA_PXF_WHITE_SHIFT;
        if (u->seg[i].run && f != 0u) {
            m = f;
            break;
        }
    }
    u->pack.white_mode = (uint8_t)m;
    u->fsum = 0;
    u->rendered = 0;
    return 1;
}

/* the limiter, once a frame: the brightness for the next one from this frame's channel sum (fx_limit_bri accounts for the
 * gamma) against the limit in wire-byte-sum units; the measured bus current over the limit trims the limit as well */
static void ux_limit(udfx_t *u)
{
    const uint32_t ma = ux_cfg_get(u,ULTRA_CFG_PX_MA_CH) ? ux_cfg_get(u,ULTRA_CFG_PX_MA_CH) : 20u;
    const uint32_t lim = u->input_limit_ma ? u->input_limit_ma : ISENSE_BOARD_MA;
    const uint32_t out = ux_cfg_get(u,ULTRA_CFG_PX_MAX) ? ux_cfg_get(u,ULTRA_CFG_PX_MAX) : BOARD_OUTPUT_RATING_MA;
    const uint32_t vk = u->vk_q8 ? u->vk_q8 : 256u;
    uint32_t maxs=m32_muldiv(lim,65280u,ma*vk);
    uint32_t out_maxs=out*255u/ma;
    if(maxs>out_maxs) maxs=out_maxs;
    uint8_t nb;
    maxs=m32_shr(maxs,u->cl_q8,8);
    if (maxs > 0x7FFFFFu) {                              /* (fx_limit_bri shifts it by 8; a strip's whole sum is under 2 M) */
        maxs = 0x7FFFFFu;
    }
    nb = fx_limit_bri(u->lim_bri, u->fsum, (uint32_t)maxs);
    if (nb < u->lim_bri) {
        u->lim_bri = nb;
    } else if (u->lim_bri < 255u && u->fsum * 10u < maxs * 9u) {   /* headroom: back up, a step of an eighth */
        u->lim_bri = (uint8_t)(u->lim_bri + 1u + ((255u - u->lim_bri) >> 3));
        if (u->lim_bri < nb) {
            u->lim_bri = nb;
        }
    }
    if (u->meas_ma != 0u && u->meas_ma > lim) {         /* measured over the limit: trim */
        u->cl_q8 = (uint16_t)(u->cl_q8 - (u->cl_q8 + 15u) / 16u);
        if (u->cl_q8 < 8u) {
            u->cl_q8 = 8;
        }
    } else if (u->cl_q8 < 256u) {
        u->cl_q8 = (uint16_t)(u->cl_q8 + 2u);
        if (u->cl_q8 > 256u) {
            u->cl_q8 = 256;
        }
    }
}

#if defined(__GNUC__)
__attribute__((noinline))
#endif
static uint32_t ux_prepare(udfx_t *u)
{
    if (!u->begun) {
        return ux_frame_begin(u);                    /* snapshot, commands and prep have separate steps */
    }
    /* No scene changes or new frame times while a Fire job is in flight. The
     * previous wire frame is untouched; render starts only after all slices. */
    if (ux_eng_step()) return 1;
    while (u->begun <= UX_SEGS + 1u) {
        const uint32_t si = u->begun - 2u;
        ux_seg_t *s = &u->seg[si];
        if (s->apply) {
            ultra_pixel_fx_t f = s->p;
            if (s->apply == 2u) f.transition_ms = 0;
            ux_apply(u, si, &f, ux_eng.now, s->anchor);
            s->apply = 0;
            return 1;
        }
        u->begun++;
        if (ux_eng_dequeue(si, ux_eng.now)) {
            s->anchor = ux_eng.seg[si].anchor;
            return 1;
        }
    }
    while (u->begun <= 2u * UX_SEGS + 1u) {
        const uint32_t si = u->begun++ - UX_SEGS - 2u;
        if (ux_eng.seg[si].active) {
            ux_eng_frame_seg(si, ux_eng.now);
            return 1;
        }
    }
    u->begun = 1;
    return ux_frame_ready(u);
}

uint32_t udfx_render(udfx_t *u)
{
    ws_enc_t *e = &u->e;
    const uint32_t bpp = u->chip.bpp, npx = u->npx;
    uint32_t upto, n;
#if !FX_V407
    const uint32_t ch = u->chunk;
#endif
    if (u->calibration || !u->local || u->want_stream || u->pause_req || e->ready || u->dark || npx == 0u) {
        return 0;
    }
    if (u->begun != 1u) {
        return ux_prepare(u);
    }
#if FX_V407
    upto = npx;                                          /* (the frame being built is never the one on the wire) */
#else
    upto = e->state == WS_FRAME ? e->i / bpp : npx;      /* (bytes before e->i are in the ring: their pixels are free) */
#endif
    if (upto > npx) {
        upto = npx;
    }
    n = upto - u->rendered;
#if FX_V407
    if (upto > u->rendered) {                            /* one run's span (CHAIN: at most 12 pixels a step) */
        fxv_px_t px[FXV_CHUNK];
        n = fxv_engine_span(&ux_eng, u->rendered, u->chain && n > 12u ? u->rendered + 12u : upto) - u->rendered;
#else
    if (upto > u->rendered && (n >= ch || upto == npx)) {   /* frame-frozen quota, or the frame's tail */
        rgb_t px[UX_CHUNK];
        if (n > ch) {
            n = ch;
        }
#endif
#ifdef UDFX_RENDER_PAD                                   /* a measurement build only (tools / the budget sweep): this many extra loop turns (3 cycles each) per render step */
        {
            uint32_t pad = UDFX_RENDER_PAD;
            __asm__ volatile("1: addi %0, %0, -1\n bnez %0, 1b" : "+r"(pad));
        }
#endif
#if FX_V407
        fxv_engine_render(&ux_eng, px, u->rendered, u->rendered + n);
        u->pack.px0 = (uint16_t)u->rendered;
        u->fsum += fxv_pack16(px, ux_pool + u->rendered * bpp, n, u->lim_bri, &u->pack, &ux_white);
#else
        fx_engine_render(&ux_eng, px, u->rendered, u->rendered + n);
        u->pack.px0 = (uint16_t)u->rendered;
        u->fsum += fx_pack(px, ux_pool + u->rendered * bpp, n, u->lim_bri, &u->pack);
#endif
        u->rendered += n;
        return 1;
    }
    if (u->rendered == npx && (FX_V407 || e->state == WS_LOW)) {   /* the frame is whole (V203: and the last one out): hand it over */
        u->black_sent = u->frame_dark;
        ux_limit(u);
        u->begun = 0;
        u->frames_done++;
        UX_BARRIER();                                  /* all frame bytes precede the tick's ready flag */
        e->ready = 1;
    }
    return 0;
}

uint32_t udfx_cal_prepare(udfx_t *u)
{
    if(u->begun==1u) u->begun=0;
    /* No encoder publication: preserve Fire, transitions, seeds and host anchors.
     * One call per main pass; never after that pass's raw render/reduction. */
    return ux_prepare(u);
}

uint32_t udfx_flash_ok(const udfx_t *u)
{
    UX_BARRIER(); /* acquire the tick-owned encoder state before flash admission */
    return u->local && u->dark && !u->want_stream && !u->stream_pending && !u->e.ready &&
           u->e.state == WS_LOW && u->e.low_w >= u->e.reset_w + WS_RING_WORDS;
}

/* ------------------------------------------------------------------------------------------------ the service */
void udfx_service(udfx_t *u, const dline_t *d, uint32_t ms, uint32_t meas_ma, uint32_t vk_q8)
{
    const uint32_t now_w = d->wpos;
    uint32_t i, host, now_ms, any_run = 0, quiet_ms, request_local = u->want_local;
    u->ms = ms;
    u->meas_ma = (uint16_t)(meas_ma > 0xFFFFu ? 0xFFFFu : meas_ma);
    u->vk_q8 = (uint16_t)(vk_q8 > 0xFFFFu ? 0xFFFFu : vk_q8);
    if (u->now_w == 0u && u->a_w == 0u && !u->a_have) {
        u->a_w = now_w;                                  /* the free-running clock starts now */
    }
    u->now_w = now_w;
    if ((int32_t)(now_w - u->a_w) > 0x10000000) {        /* ~2 min without an anchor: re-base (no information lost) */
        u->a_host = udfx_host_us(u, now_w);
        u->a_w = now_w;
    }
    host = udfx_host_us(u, now_w);
    now_ms = ux_ms_advance(u, host);
    if(u->calibration) return;                           /* raw owner suppresses policy, reconf and stream takeover */
    if (ux_auto_stream(u)) {
        /* Finish any local frame through the existing latch handover. Stored
         * effects, loss policy and configured geometry cannot reclaim replay. */
        u->want_local = 0;
        if (u->local && !u->want_stream && !u->stream_pending) ux_queue_stream(u);
        if (!u->local && u->stream_pending) ux_stream(u);
        return;
    }
    if (!u->local && u->stream_pending) {
        ux_stream(u);                                  /* already STREAM: no render can be in progress */
        request_local = 0;
    }

    for (i = 0; i < UX_SEGS; i++) {
        ux_seg_t *s = &u->seg[i];
        any_run |= s->run;
        if (!u->stream_pending && ux_due(s, now_ms)) {
            any_run = 1;                                /* enter LOCAL; only the renderer applies the scene */
            u->dark = 0;
        }
    }

    /* Finish frozen scene preparation before resetting the engine. */
    if (u->reconf && u->begun <= 1u) {                    /* the chip / pixels / order / fps changed */
        if (u->local && (u->chain || !ux_cfg_get(u,ULTRA_CFG_PX_SOURCE))) {                      /* CHAIN: the encoder pauses in LOW (the tick), the tables change,
                                                            it goes on at once: the ring is never the line's */
            UX_BARRIER(); /* publish pause request before waiting for encoder ownership */
            u->pause_req = 1;
            if (u->paused) {
                UX_BARRIER();                           /* the tick has finished using the old tables */
                ux_derive(u);
                u->reconf = 0;
                u->begun = 0;
                u->rebegin = 1;                             /* restore segments through the bounded frame-setup steps */
                UX_BARRIER();                           /* new tables complete before the tick resumes */
                u->pause_req = 0;                        /* (the tick starts the encoder again from where it is) */
            }
        } else if (u->local) {
            u->want_stream = 1;
        } else {
            ux_derive(u);
            u->reconf = 0;
            u->begun = 0;
            u->rebegin = 1;                             /* restore segments through the bounded frame-setup steps */
            if (any_run && u->npx != 0u) {
                u->want_stream = 0;
                request_local = 1;
            }
        }
    }

    /* the standalone policy: the stored effect when the line stays quiet from power-up / goes quiet */
    quiet_ms = (d->wpos - d->edge_w) / DLINE_W_PER_MS;
    if ((!u->local || u->chain || !ux_cfg_get(u,ULTRA_CFG_PX_SOURCE)) && !request_local && u->stored_have && u->npx != 0u && !any_run) {
        const uint32_t pol = ux_cfg_get(u, ULTRA_CFG_PX_POLICY), lt = ux_cfg_get(u, ULTRA_CFG_PX_LOSS_TIMEOUT);
        const uint32_t loss_ms = (lt != 0u ? lt : 30u) * 100u;
        const uint32_t never_seen = d->st.frames == 0u && d->st.marks == 0u;
        if (((pol & ULTRA_PXP_POWER_ON) && never_seen && quiet_ms >= UX_POWER_ON_MS && ms >= UX_POWER_ON_MS) ||
            ((pol & ULTRA_PXP_ON_LOSS) && !never_seen && quiet_ms >= loss_ms)) {
            uint32_t k;
            for (k = 0; k < UX_SEGS; k++) {
                if (u->stored_have & (1u << k)) {
                    ultra_pixel_fx_t f = u->stored[k];
                    f.start_ms = 0;
                    if (f.transition_ms < UX_FADE_IN_MS) {
                        f.transition_ms = UX_FADE_IN_MS;
                    }
                    ux_seg_cmd(u, k, &f, host);
                }
            }
            u->standalone = 1;
            u->lock = 0;
            request_local = !u->local;
            u->frames_seen = d->st.frames;
        }
    }
    if (ux_cfg_get(u,ULTRA_CFG_PX_SOURCE) && u->local && !u->want_stream && !u->chain && !u->undec) {   /* a plain stream takes the strip back (not locked; CHAIN: every
                                                            Ultra frame would look like one, undecided: it may be) */
        if ((ux_cfg_get(u, ULTRA_CFG_PX_POLICY) & ULTRA_PXP_STREAM_WINS) && !u->lock &&
            d->st.frames - u->frames_seen >= 2u) {
            ux_queue_stream(u);
        }
    }
    if (!u->local && !request_local) {
        u->frames_seen = d->st.frames;
    }
    if ((!ux_cfg_get(u,ULTRA_CFG_PX_SOURCE) || any_run) && !u->local && !u->want_stream && !request_local && u->npx != 0u && !u->reconf) {
        request_local = 1;
    }

    /* the first frame's start word (the grid), refreshed while a switch waits; the grid's phase while LOCAL */
    if (request_local && !u->local) {
        if (u->period_us != 0u) {
            const uint32_t start = host + u->chip.reset_w * MCU_US_PER_WORD_NUM / MCU_US_PER_WORD_DEN + 1000u;   /* a reset + 1 ms of margin */
            const uint32_t td = (start + u->period_us - 1u) / u->period_us * u->period_us;
            u->first_w = udfx_word_at(u, td);
        } else {
            u->first_w = 0;
        }
        UX_BARRIER();                                  /* publish the grid word before the tick can take LOCAL */
        u->want_local = 1;
    }
    if (u->local && u->period_us != 0u && u->nudge == 0 && (ms & 15u) == 0u) {
        const uint32_t nw = u->e.next_w;
        const uint32_t th = udfx_host_us(u, nw), k = (th + u->period_us / 2u) / u->period_us;
        const int32_t dw = (int32_t)(udfx_word_at(u, k * u->period_us) - nw);
        const int32_t lim = (int32_t)(u->period_w / 4u);
        if ((dw >= 2 || dw <= -2) && dw < (int32_t)(u->period_w / 2u) && dw > -(int32_t)(u->period_w / 2u)) {
            u->nudge = dw > lim ? lim : dw < -lim ? -lim : dw;
        }
    }
    if (u->dark && ms - u->dark_at >= 1000u) {           /* a black frame once a second keeps a glitched strip honest */
        u->dark = 0;
        u->black_sent = 0;
    }
    if (ms - u->fps_ms >= 1000u) {
        u->fps10 = (uint16_t)((u->frames_done - u->fps_frames) * 10u);
        u->fps_frames = u->frames_done;
        u->fps_ms = ms;
    }
}

/* ------------------------------------------------------------------------------------------------ the tick */
void udfx_restarted(udfx_t *u, dline_t *d)
{
    if (u->local) {
        ws_enc_init(&u->e, &u->chip, ux_pool, u->npx, d->wpos, u->period_w);
        u->begun = 0;
        u->rendered = 0;
    }
}

/* the LOW run a pending switch waits for once its other conditions hold: the chip's latch, and after UX_LATCH_WAIT_W of waiting
 * dline's (a stream that never has a gap of the chip's latch must not keep the strip from its switch for ever) */
static uint32_t ux_need(udfx_t *u, uint32_t w, uint32_t ok)
{
    if (!ok) {
        u->wait_on = 0;
        return u->latch_w;
    }
    if (!u->wait_on) {
        u->wait_on = 1;
        u->wait_w = w;
    }
#if defined(MCU_SOURCE_SWITCH_LOW_W)
    {
        /* a source selector (SOURCE_SEL: the replay or the serialiser): the replay shows the word D words behind RX, so
         * the incoming LOW run must also cover the whole ring for the replay to be inside it at the switch */
        const uint32_t n = (int32_t)(w - u->wait_w) >= (int32_t)UX_LATCH_WAIT_W ? DLINE_W_LATCH : u->latch_w;
        return n < MCU_SOURCE_SWITCH_LOW_W ? MCU_SOURCE_SWITCH_LOW_W : n;
    }
#else
    return (int32_t)(w - u->wait_w) >= (int32_t)UX_LATCH_WAIT_W ? DLINE_W_LATCH : u->latch_w;
#endif
}

/* the encoder takes the ring at word w: LOW from here (the first frame waits a whole reset and the grid) */
static void ux_enter_local(udfx_t *u, dline_t *d, uint16_t *ring, uint32_t w)
{
    UX_BARRIER(); /* acquire tables and first_w from the LOCAL request */
    ws_enc_init(&u->e, &u->chip, ux_pool, u->npx, w, u->period_w);
    if (u->period_w != 0u && (int32_t)(u->first_w - u->e.next_w) > 0) {
        u->e.next_w = u->first_w;
    }
    u->begun = 0;
    u->rendered = 0;
    u->want_local = 0;
    u->local = 1;
    d->local = 1;
    u->frames_seen = d->st.frames;                       /* (the hub's frames up to here are not a controller taking over) */
    u->switches++;
    (void)ws_ring_fill(&u->e, &ux_lut, ring, w);
}

void udfx_tick(udfx_t *u, dline_t *d, uint16_t *ring)
{
    const uint32_t w = d->wpos;
    if (d->restart != 0u) {
        return;
    }
    if (!(u->local ? u->want_stream : (u->want_local && !u->want_stream))) {
        u->wait_on = 0;                                  /* (no switch is pending: its waiting starts anew) */
    }
    if (u->local) {
        if (u->nudge != 0) {
            u->e.next_w += (uint32_t)u->nudge;
            u->nudge = 0;
        }
        if (u->paused && !u->pause_req) {                /* CHAIN: the main loop is done with the tables: the encoder goes on from where it is
                                                            (before any fill: a frame it hands over now starts only after this) */
            UX_BARRIER(); /* acquire rebuilt tables after pause_req clears */
            u->paused = 0;
            ws_enc_init(&u->e, &u->chip, ux_pool, u->npx, u->e.w, u->period_w);
        }
        if (u->want_stream || u->pause_req) {
            u->e.ready = 0;                              /* no new frame: the one in progress ends */
        }
        (void)ws_ring_fill(&u->e, &ux_lut, ring, w);
        if (u->pause_req) {                              /* CHAIN, tables about to change: idle in LOW, a whole reset out */
            UX_BARRIER(); /* encoder state complete before the pause acknowledgement */
            u->paused = (uint8_t)(u->e.state == WS_LOW && u->e.low_w >= u->e.reset_w && u->e.w == w);
        }
        if (u->want_stream && u->e.state == WS_LOW && u->e.low_w >= u->e.reset_w && u->e.w == w &&
            dline_low_run(d) >= ux_need(u, w, 1u)) {
            u->wait_on = 0;
            u->local = 0;                                /* the hub's stream passes again, inside its latch */
            d->local = 0;
            u->want_stream = 0;
            u->begun = 0;
            u->switches++;
        }
    } else if (u->chain && d->ch_enter == 2u) {         /* CHAIN at boot: dline zeroed the ring and switched the mux, BLANK_N is
                                                            still held: the encoder takes the ring now (the strip line stays LOW) */
        d->ch_enter = 0;
        ux_enter_local(u, d, ring, w);
    } else if (u->want_local && !u->want_stream && (u->npx != 0u || u->chain || u->calibration) && !d->direct && !d->rec &&
               dline_low_run(d) >= ux_need(u, w, 1u)) {
        u->wait_on = 0;
        ux_enter_local(u, d, ring, w);
    }
}

/* CHAIN on / off from the main loop. Off: the usual way back to the stream (at a latch); on: dline has taken the ring
 * (dline_ch_start) or the encoder follows at a latch of the incoming line */
NOINLINE void udfx_chain(udfx_t *u, uint32_t mode)   /* (five sites: one copy) */
{
    const uint32_t on = mode == 1u;
    u->chain = (uint8_t)on;
    *(volatile uint8_t *)&u->undec = (uint8_t)(mode == 2u);   /* (a byte store: chain + undec are no halfword at chain's odd offset) */
    u->stream = (uint8_t)(mode == 0u);
    if (mode == 2u) {
        return;
    }
    if (on) {
        /* AUTO may decide CHAIN while a standalone frame is already rendering. */
#if !FX_V407
        if (u->begun == 1u) ux_chunk(u);
#endif
        u->want_stream = 0;
        UX_BARRIER();
        u->want_local = (uint8_t)!u->local;
    } else {
        u->pause_req = 0;
        if(ux_replay(u)) ux_queue_stream(u); /* STREAM again: no effect runs (the hub starts one with PIXEL_FX), the encoder hands
                                                            the strip back at a latch */
    }
}

/* ------------------------------------------------------------------------------------------------ the record */
/* 128 bytes: magic (2), version (1), segments (1), seq (4), the twelve values (24), four segments of 23 bytes (92: the
 * PIXEL_FX fields but its START; effect 0 = none), the stored mask (1), 1 spare, the CRC-16 (never an erased value) */
/* CH32 and the Windows host are little endian, as for udcal's stored model.
 * Copy only asserted contiguous fields; never include structure padding. */
typedef char ux_byte_fields_contiguous[(offsetof(ultra_pixel_fx_t, brightness) == 13u &&
    offsetof(ultra_pixel_fx_t, flags) == 14u && offsetof(ultra_pixel_fx_t, custom2) == 16u) ? 1 : -1];
typedef char ux_record_halfwords_contiguous[(offsetof(ultra_pixel_fx_t, seg_start) == offsetof(ultra_pixel_fx_t, transition_ms) + 2u &&
    offsetof(ultra_pixel_fx_t, seg_len) == offsetof(ultra_pixel_fx_t, transition_ms) + 4u) ? 1 : -1];

/* Byte by byte, NOIPA (no constant size reaches it, so GCC can neither expand nor widen the copy): a segment is 23
 * bytes, its fields sit at every address mod 4, and misaligned loads / stores are not qualified on the core
 * (a bench check). */
static NOIPA void ux_copy(void *d, const void *s, uint32_t n)
{
    uint8_t *p = d;
    const uint8_t *q = s;
    while (n--) {
        *p++ = *q++;
    }
}

/* The four segments, record <-> stored (to_rec: into the record), byte by byte and NOIPA as ux_copy: a segment's record
 * bytes 0..13 are the structure's effect .. brightness, 14..19 transition_ms .. seg_len, 20..22 flags .. custom2; a load
 * clears START. */
static NOIPA void ux_segs(uint8_t *q, ultra_pixel_fx_t *f, uint32_t to_rec)
{
    uint32_t i, j;
    for (i = 0; i < UX_SEGS; i++, q += 23u, f++) {
        for (j = 0; j < 23u; j++) {
            uint8_t *b = (uint8_t *)f + (j < 14u ? j : j < 20u ? j - 14u + offsetof(ultra_pixel_fx_t, transition_ms) :
                                         j - 20u + offsetof(ultra_pixel_fx_t, flags));
            if (to_rec) {
                q[j] = *b;
            } else {
                *b = q[j];
            }
        }
        if (!to_rec) {
            f->start_ms = 0;
        }
    }
}

static void ux_put16(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void ux_put32(uint8_t *p, uint32_t v)
{
    ux_put16(p, v);
    ux_put16(p + 2, v >> 16);
}

static uint32_t ux_get16(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8;
}

static uint32_t ux_get32(const uint8_t *p)
{
    return ux_get16(p) | ux_get16(p + 2) << 16;
}

void udfx_rec_pack(const udfx_t *u, uint32_t seq, uint8_t *r)
{
    uint32_t i, c;
    for (i = 0; i < UX_REC_BYTES; i++) {
        r[i] = 0;
    }
    ux_put16(r, UX_REC_MAGIC);
    r[2] = UX_REC_FORMAT;
    r[3] = BOARD_OUTPUTS;
    ux_put32(r + 4, seq);
    ux_copy(r + 8, u->cfg, 2u * UX_REC_SLOTS);
    ux_put16(r + 8u + 2u * (UX_REC_SLOTS - 1u), (uint32_t)(u->cfg[ULTRA_CFG_PX_LINE_MODE - ULTRA_CFG_PX_FIRST] & 3u) |
           (uint32_t)(u->cfg[ULTRA_CFG_PX_SMOOTH - ULTRA_CFG_PX_FIRST] & 3u) << 2 |
           (uint32_t)(u->cfg[ULTRA_CFG_PX_SOURCE - ULTRA_CFG_PX_FIRST] & 1u) << 4);
    ux_segs(r + UX_REC_SEG0, (ultra_pixel_fx_t *)u->stored, 1);
    r[UX_REC_MASK] = u->stored_have;
    do {
        r[UX_REC_MASK + 1u]++;                                        /* (a CRC that would read erased: one more spare count) */
        c = ud_crc16(r, UX_REC_BYTES - 2u);
    } while (ud_crc_erased(c));
    ux_put16(r + UX_REC_BYTES - 2u, c);
}

int udfx_rec_valid(const uint8_t *r)
{
    const uint32_t c = ux_get16(r + UX_REC_BYTES - 2u);
    return ux_get16(r) == UX_REC_MAGIC && r[2] == UX_REC_FORMAT && r[3] == BOARD_OUTPUTS && ux_get16(r + 8) < ULTRA_PXC_COUNT && !ud_crc_erased(c) &&
           c == ud_crc16(r, UX_REC_BYTES - 2u);
}

void udfx_load(udfx_t *u, const uint8_t *r)
{
#if FX_V407 || FX_NO_PACIFICA
    uint32_t i;
#endif
    u->rec_seq = ux_get32(r + 4);
    ux_copy(u->cfg, r + 8, 2u * UX_REC_SLOTS);
    u->cfg[ULTRA_CFG_PX_SMOOTH - ULTRA_CFG_PX_FIRST] = (uint16_t)((u->cfg[ULTRA_CFG_PX_LINE_MODE - ULTRA_CFG_PX_FIRST] >> 2) & 3u);
    u->cfg[ULTRA_CFG_PX_SOURCE-ULTRA_CFG_PX_FIRST] = (uint16_t)((u->cfg[ULTRA_CFG_PX_LINE_MODE-ULTRA_CFG_PX_FIRST] >> 4) & 1u);
    u->cfg[ULTRA_CFG_PX_LINE_MODE - ULTRA_CFG_PX_FIRST] &= 3u;
    ux_segs((uint8_t *)r + UX_REC_SEG0, u->stored, 0);
    u->stored_have = r[UX_REC_MASK];
#if FX_V407
    for (i = 0; i < UX_SEGS; i++)
        if (u->stored[i].effect != FX_STREAM && !fxv_built(u->stored[i].effect)) u->stored_have &= (uint8_t)~(1u << i);
#elif FX_NO_PACIFICA
    for(i=0;i<UX_SEGS;i++)
        if(u->stored[i].effect==FX_PACIFICA) u->stored_have&=(uint8_t)~(1u<<i);
#endif
    ux_derive(u);
    u->save_due = 0;
}

void udfx_rec_saved(udfx_t *u)
{
    u->save_due = 0;
}

/* ------------------------------------------------------------------------------------------------ page 0x81 */
int udfx_page(const udfx_t *u, uint32_t part, uint8_t *d)
{
    uint32_t i, run = 0;
    if (part == ULTRA_PXG_PART_STATE) {
        ud_put16(d + ULTRA_PXG_CATALOGUE, UX_CATALOGUE_VERSION);
#if FX_V407
        d[ULTRA_PXG_EFFECTS] = (uint8_t)(FX_LAST + FXV_BUILT_NEW);
        d[ULTRA_PXG_PALETTES] = (uint8_t)FXV_PAL_COUNT;
#else
        d[ULTRA_PXG_EFFECTS] = (uint8_t)(FX_LAST-FX_NO_PACIFICA);
        d[ULTRA_PXG_PALETTES] = (uint8_t)PAL_COUNT;
#endif
        d[ULTRA_PXG_CHIPS] = (uint8_t)ULTRA_PXC_COUNT;
        d[ULTRA_PXG_SEGMENTS] = (uint8_t)UX_SEGS;
        ud_put16(d + ULTRA_PXG_MAX_PIXELS, u->max_px);
        for (i = 0; i < UX_SEGS; i++) {
            d[ULTRA_PXG_RUNNING + i] = u->seg[i].run ? u->seg[i].p.effect : 0u;
            d[ULTRA_PXG_SEQ + i] = (u->seq_have & (1u << i)) ? u->seq5[i] : 0u;
            run |= u->seg[i].run;
        }
        /* (CHAIN with no effect running is the strip dark: STREAM, nothing runs) */
        d[ULTRA_PXG_MODE] = (uint8_t)(u->local && (run || !u->chain) ? (u->standalone ? ULTRA_PXM_STANDALONE : ULTRA_PXM_LOCAL)
                                                                      : ULTRA_PXM_STREAM);
        ud_put16(d + ULTRA_PXG_FPS10, u->fps10);
        d[ULTRA_PXG_LOAD] = u->load;
        d[ULTRA_PXG_LIMIT] = u->lim_bri;
        d[ULTRA_PXG_LATE] = u->e.late;
        ud_put16(d + ULTRA_PXG_CONFIG_CRC, ud_px_config_crc(u->cfg,u->cfg_valid,0,0));
        return 0;
    }
    if (part == ULTRA_PXG_PART_EFFECTS || part == ULTRA_PXG_PART_PALETTES) {
#if FX_V407
        const uint8_t *m = part == ULTRA_PXG_PART_EFFECTS ? fxv_map_effects : fxv_map_palettes;   /* generated: built ids only */
        for (i = 0; i < 24u; i++) d[i] = m[i];
#else
        for (i = 0; i < 192u; i++) {
            if (part == ULTRA_PXG_PART_EFFECTS ? (i != 0u && i < FX_COUNT && (!FX_NO_PACIFICA || i!=FX_PACIFICA)) : i < PAL_COUNT) {   /* (effect 0 is the stream) */
                d[i >> 3] = (uint8_t)(d[i >> 3] | (1u << (i & 7u)));
            }
        }
#endif
        return 0;
    }
    return -1;
}

NOINLINE uint16_t ud_px_config_crc(const uint16_t *values, uint16_t valid, uint16_t input_ma, uint8_t input_valid)
{
    uint8_t b[(ULTRA_CFG_PX_COUNT + 1u) * 4u];
    uint32_t k, n = 0;
    for (k = 0; k < ULTRA_CFG_PX_COUNT; ++k) {
        if (!(valid & (1u << k))) continue;
        b[n++] = 0; b[n++] = (uint8_t)(ULTRA_CFG_PX_FIRST + k);
        ultra_put16(b + n, values[k]); n += 2u;
    }
    if(input_valid) {
        b[n++]=ULTRA_TARGET_BOARD; b[n++]=ULTRA_CFG_BOARD_MAX;
        ultra_put16(b+n,input_ma); n+=2;
    }
    return ud_crc16(b, n);
}
