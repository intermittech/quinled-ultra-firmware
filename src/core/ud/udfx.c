/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The Ultra-Digital receiver's local pixel effects: see udfx.h. */
#include <stddef.h>
#include "udfx.h"

#if defined(__GNUC__)
uint8_t ux_pool[UX_POOL_BYTES] __attribute__((aligned(4)));
#elif defined(_MSC_VER)
__declspec(align(4)) uint8_t ux_pool[UX_POOL_BYTES];
#else
uint8_t ux_pool[UX_POOL_BYTES];
#endif
static uint8_t ux_state[UX_STATE_BYTES];                 /* the engine's state pool: Fire's heat */

/* ------------------------------------------------------------------------------------------------ the chips */
typedef struct {
    uint8_t  s, t0h, t1h, bpp;            /* the 36 MHz sample grid (docs: tools/ud_timing_check.py), bytes per pixel */
    uint16_t reset_w;                     /* LOW words between frames */
    uint16_t latch_w;                     /* LOW words after which the strip has surely latched, the HSI 2.2 % fast */
    uint8_t  ord[5];                      /* the default wire order (canonical channels R 0, G 1, B 2, W 3, CW 4) */
} ux_chip_t;

/* resets (words of 0.444 us), each above the chip's minimum with the HSI 2.2 % fast (tools/ud_timing_check.py): the V5 class
 * > 280 us: 680 = 302 us; the SK6812 Rev B/1 > 200 us: 480 = 213 us; the older chips 200 us. The latch: the least reset the
 * chip family needs (the SK6812 Rev 02's 80 us: a hub at the Rev B/1's 200 us is no less welcome), in words with the HSI 2.2 %
 * fast and a margin (checked by the same tool): a LOW run that long latches the strip, so a mode switch inside it never cuts a
 * frame (a shorter LOW can be inside a controller's frame: an RMT refill) */
static const ux_chip_t ux_chips[ULTRA_PXC_COUNT] = {
    { 48, 13, 25, 3, 680, 650, { 1, 0, 2, 0, 0 } },   /* WS2812B V5 / 2020: 361 / 694 ns */
    { 48, 13, 28, 3, 450, 120, { 1, 0, 2, 0, 0 } },   /* WS2812B classic: 361 / 778 ns */
    { 48, 12, 26, 3, 480, 190, { 1, 0, 2, 0, 0 } },   /* SK6812 RGB: 333 / 722 ns */
    { 48, 12, 26, 4, 480, 190, { 1, 0, 2, 3, 0 } },   /* SK6812 RGBW */
    { 48, 13, 25, 4, 680, 650, { 0, 1, 2, 3, 0 } },   /* WS2814 RGBW (V5 timing) */
    { 48, 13, 25, 5, 680, 650, { 0, 1, 2, 3, 4 } },   /* WS2805 RGBCCT */
    { 48, 13, 25, 3, 680, 650, { 1, 0, 2, 0, 0 } },   /* WS2815 (V5 timing assumed) */
    { 90, 18, 43, 3, 450, 120, { 0, 1, 2, 0, 0 } },   /* WS2811 at 400 kHz: 500 / 1194 ns */
};

static uint32_t ux_cfg_get(const udfx_t *u, uint32_t id)
{
    return u->cfg[id - ULTRA_CFG_PX_FIRST];
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

static uint32_t ux_max_px(uint32_t bpp)
{
    return UX_POOL_BYTES / bpp;
}

/* the configuration into the derived fields and the engine (not while LOCAL: the tick uses the tables and the encoder) */
static void ux_derive(udfx_t *u)
{
    const uint32_t id = ux_cfg_get(u, ULTRA_CFG_PX_CHIP);
    const ux_chip_t *c = &ux_chips[id < ULTRA_PXC_COUNT ? id : 0u];
    const uint32_t ord = ux_cfg_get(u, ULTRA_CFG_PX_ORDER), fps = ux_cfg_get(u, ULTRA_CFG_PX_FPS), w = ux_cfg_get(u, ULTRA_CFG_PX_WHITE);
    uint32_t k, px = ux_cfg_get(u, ULTRA_CFG_PX_PIXELS);
    u->chip.s = c->s;
    u->chip.t0h = c->t0h;
    u->chip.t1h = c->t1h;
    u->chip.bpp = c->bpp;
    u->chip.reset_w = c->reset_w;
    u->latch_w = c->latch_w;
    u->pack.gamma = fx_gamma28;
    u->pack.bpp = c->bpp;
    for (k = 0; k < 5u; k++) {
        u->pack.ord[k] = (ord != 0u && ux_order_ok(ord, c->bpp)) ? (uint8_t)((ord >> (3u * k)) & 7u) : c->ord[k];
    }
    u->white_def = (uint8_t)((w & 3u) != 0u ? (w & 3u) : FX_W_ACCURATE);
    u->pack.white_mode = u->white_def;
    u->pack.white_split = (uint8_t)(((w >> 8) & 0xFFu) != 0u ? (w >> 8) & 0xFFu : 128u);
    u->max_px = (uint16_t)ux_max_px(c->bpp);
    u->npx = (uint16_t)(px > u->max_px ? u->max_px : px);
    u->period_us = fps != 0u ? 1000000u / fps : 0u;
    u->period_w = fps != 0u ? (uint32_t)(((uint64_t)u->period_us * u->wpu24) >> 24) : 0u;
    ws_lut_build(&u->lut, &u->chip);
    fx_engine_init(&u->eng, u->npx, ux_state, sizeof ux_state);   /* (every segment inactive: ux_reapply puts the running ones back) */
}

void udfx_init(udfx_t *u)
{
    uint8_t *p = (uint8_t *)u;
    uint32_t i;
    for (i = 0; i < (uint32_t)sizeof *u; i++) {
        p[i] = 0;
    }
    u->wq24 = UX_RATE_NOM;
    u->wpu24 = (uint32_t)(((uint64_t)1 << 48) / UX_RATE_NOM);
    u->lim_bri = 255;
    u->cl_q8 = 256;
    ux_derive(u);
}

/* ------------------------------------------------------------------------------------------------ the clock */
uint32_t udfx_host_us(const udfx_t *u, uint32_t w)
{
    return u->a_host + (uint32_t)((int64_t)(int32_t)(w - u->a_w) * (int64_t)u->wq24 >> 24);
}

uint32_t udfx_word_at(const udfx_t *u, uint32_t host_us)
{
    return u->a_w + (uint32_t)((int64_t)(int32_t)(host_us - u->a_host) * (int64_t)u->wpu24 >> 24);
}

/* the engine's host ms: a count that goes on across the 32-bit us wrap, for a host time near the reference. The reference
 * (host us, a multiple of 1000, and its ms) moves with the time (ux_ms_advance), so differences of ms are floor(us / 1000)
 * differences: the same on every receiver that gets the same host times */
static uint32_t ux_ms(const udfx_t *u, uint32_t host_us)
{
    const int32_t d = (int32_t)(host_us - u->hus_ref);
    return u->hms_ref + (uint32_t)(d >= 0 ? d / 1000 : -((-d + 999) / 1000));
}

static void ux_ms_advance(udfx_t *u, uint32_t host_us)
{
    const int32_t d = (int32_t)(host_us - u->hus_ref);
    if (d >= 500000 || d <= -500000) {
        const int32_t q = d >= 0 ? d / 1000 : -((-d + 999) / 1000);
        u->hms_ref += (uint32_t)q;
        u->hus_ref += (uint32_t)q * 1000u;
    }
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
        uint64_t q = dw != 0u ? ((uint64_t)dh << 24) / dw : 0u;
        /* within 1/16 of nominal; once known, within 0.3 % of the rate it was (the HSI drifts slowly: a pair that spans a
           jump of the hub's clock is not a rate) */
        if (q - (UX_RATE_NOM - UX_RATE_NOM / 16u) < UX_RATE_NOM / 8u &&
            (!u->r_have || q - (u->wq24 - u->wq24 / 330u) < u->wq24 / 165u)) {
            stepped = !u->r_have;                        /* (the first measurement: the clock steps, no history) */
            u->wq24 = (uint32_t)q;
            u->wpu24 = (uint32_t)(((uint64_t)1 << 48) / q);
            u->r_have = 1;
            if (ux_cfg_get(u, ULTRA_CFG_PX_FPS) != 0u) {
                u->e.period_w = u->period_w = (uint32_t)(((uint64_t)u->period_us * u->wpu24) >> 24);
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
        int32_t lim = (int32_t)(since / 450u);           /* 0.5 %: since x 0.444 us / 200 */
        int32_t adj = err > lim ? lim : err < -lim ? -lim : err;
        u->a_host = pred + (uint32_t)adj;
    }
    u->a_w = w;
}

/* ------------------------------------------------------------------------------------------------ the commands */
/* the PIXEL_FX slot's fields as the engine's parameters (CONFIG 136 / 137: the virtual strip) */
static void ux_params(const udfx_t *u, uint32_t si, const ultra_pixel_fx_t *f, fx_params_t *q)
{
    uint32_t i;
    q->id = f->effect;
    q->speed = f->speed;
    q->intensity = f->intensity;
    q->palette = f->palette == ULTRA_PX_PAL_COLORS ? (uint8_t)PAL_COLORS
               : (f->palette == ULTRA_PX_PAL_DEFAULT || f->palette >= PAL_COUNT) ? (uint8_t)PAL_DEFAULT : f->palette;
    for (i = 0; i < 3u; i++) {
        q->c[i].r = f->c[i][0];
        q->c[i].g = f->c[i][1];
        q->c[i].b = f->c[i][2];
    }
    q->bri = f->brightness;
    q->flags = (uint8_t)((f->flags & ULTRA_PXF_REVERSE ? FX_F_REVERSE : 0u) | (f->flags & ULTRA_PXF_MIRROR ? FX_F_MIRROR : 0u));
    q->custom1 = f->custom1;
    q->custom2 = f->custom2;
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
    for (i = 0; i < UX_SEGS; i++) {
        u->seg[i].run = 0;
        u->seg[i].pend_have = 0;
        fx_engine_set(&u->eng, i, &q, 0, 0);             /* (inactive) */
    }
    u->want_local = 0;
    u->want_stream = 1;
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
    fx_engine_set(&u->eng, si, &q, now_ms, anchor_ms);
    s->p = *f;
    s->p.flags &= (uint8_t)(0xFFu ^ ULTRA_PXF_RESTART);         /* (an action, not a setting) */
    s->anchor = u->eng.seg[si].anchor;                   /* (a renewal, the same slot with a new START, keeps the running anchor) */
    s->run = 1;
    u->dark = u->black_sent = 0;
}

/* every running segment into the engine again (the engine was re-initialised, or the virtual strip changed): at once, the
 * effect's own time goes on from its anchor */
static void ux_reapply(udfx_t *u, uint32_t now_ms)
{
    uint32_t i;
    for (i = 0; i < UX_SEGS; i++) {
        if (u->seg[i].run) {
            ultra_pixel_fx_t f = u->seg[i].p;
            fx_params_t q;
            ux_params(u, i, &f, &q);
            q.trans_ms = 0;
            fx_engine_set(&u->eng, i, &q, now_ms, u->seg[i].anchor);
        }
    }
}

static void ux_seg_cmd(udfx_t *u, uint32_t si, const ultra_pixel_fx_t *f, uint32_t hdr_us)
{
    ux_seg_t *s = &u->seg[si];
    if (f->start_ms > 20) {                              /* a future START: wait for it */
        s->pend = *f;
        s->pend_at = hdr_us + (uint32_t)f->start_ms * 1000u;
        s->pend_have = 1;
        return;
    }
    s->pend_have = 0;
    ux_apply(u, si, f, ux_ms(u, hdr_us), ux_ms(u, hdr_us) + (uint32_t)f->start_ms);
}

void udfx_pixel_fx(udfx_t *u, const uint8_t *slot, uint32_t hdr_us)
{
    ultra_pixel_fx_t f;
    int seg = ultra_pixel_fx_parse(slot, &f);
    uint32_t idx, i;
    if (seg < 0) {
        return;
    }
    idx = seg == (int)ULTRA_LIGHT_BOARD ? UX_SEGS : (uint32_t)seg;
    if (u->seq_have & (1u << idx) && u->seq5[idx] == slot[2]) {
        return;                                          /* applied once per SEQ */
    }
    u->seq_have = (uint8_t)(u->seq_have | (1u << idx));
    u->seq5[idx] = slot[2];
    if (f.effect == ULTRA_PX_FX_STREAM) {
        if (f.flags & ULTRA_PXF_STORE) {
            u->stored_have = 0;
            u->save_due = 1;
            u->save_ms = u->ms;
        }
        ux_stream(u);
        return;
    }
    u->lock = (uint8_t)((f.flags & ULTRA_PXF_LOCK) != 0u);
    u->standalone = 0;
    u->want_stream = 0;
    if (f.flags & ULTRA_PXF_STORE) {
        const uint32_t k = seg == (int)ULTRA_LIGHT_BOARD ? 0u : (uint32_t)seg;
        u->stored[k] = f;
        u->stored[k].start_ms = 0;
        u->stored[k].flags &= (uint8_t)(0xFFu ^ ULTRA_PXF_RESTART);
        u->stored_have = (uint8_t)(seg == (int)ULTRA_LIGHT_BOARD ? 1u : (u->stored_have | (1u << k)));
        u->save_due = 1;
        u->save_ms = u->ms;
    }
    if (seg == (int)ULTRA_LIGHT_BOARD) {                 /* the whole strip: segment 0, the others stopped */
        fx_params_t q;
        for (i = 0; i < sizeof q; i++) {
            ((uint8_t *)&q)[i] = 0;
        }
        q.id = FX_STREAM;
        for (i = 1; i < UX_SEGS; i++) {
            u->seg[i].run = 0;
            u->seg[i].pend_have = 0;
            fx_engine_set(&u->eng, i, &q, 0, 0);
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
    uint32_t k;
    if (id < ULTRA_CFG_PX_FIRST || id >= ULTRA_CFG_PX_FIRST + ULTRA_CFG_PX_COUNT || v > 0xFFFFu) {
        return 0;
    }
    k = id - ULTRA_CFG_PX_FIRST;
    switch (id) {
    case ULTRA_CFG_PX_CHIP:
        if (v >= ULTRA_PXC_COUNT) {
            return 0;
        }
        break;
    case ULTRA_CFG_PX_ORDER:
        if (v != 0u && !ux_order_ok(v, ux_chips[ux_cfg_get(u, ULTRA_CFG_PX_CHIP) < ULTRA_PXC_COUNT ? ux_cfg_get(u, ULTRA_CFG_PX_CHIP) : 0u].bpp)) {
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
    default:
        break;
    }
    if (u->cfg[k] == v) {
        return 0;
    }
    u->cfg[k] = (uint16_t)v;
    u->save_due = 1;
    u->save_ms = u->ms;
    u->dark = u->black_sent = 0;
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
    uint32_t i, host, nw, now_ms, m;
    if (u->period_us != 0u) {                            /* on the host grid: frame k at host time k x P */
        nw = e->next_w;
        host = udfx_host_us(u, nw);
        host = (host + u->period_us / 2u) / u->period_us * u->period_us;
    } else {
        nw = e->w + (e->low_w < e->reset_w ? e->reset_w - e->low_w : 0u);
        host = udfx_host_us(u, nw);
    }
    now_ms = ux_ms(u, host);
    if (u->rebegin) {
        u->rebegin = 0;
        ux_reapply(u, now_ms);
    }
    fx_engine_frame(&u->eng, now_ms);
    u->frame_dark = (uint8_t)fx_engine_dark(&u->eng);
    if (u->frame_dark) {
        if (u->black_sent) {
            u->dark = 1;
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
    u->begun = 1;
    return 1;
}

/* the limiter, once a frame: the brightness for the next one from this frame's channel sum (fx_limit_bri accounts for the
 * gamma) against the limit in wire-byte-sum units; the measured bus current over the limit trims the limit as well */
static void ux_limit(udfx_t *u)
{
    const uint32_t ma10 = ux_cfg_get(u, ULTRA_CFG_PX_MA_CH) != 0u ? ux_cfg_get(u, ULTRA_CFG_PX_MA_CH) : 200u;
    const uint32_t lim = ux_cfg_get(u, ULTRA_CFG_PX_LIMIT_MA) != 0u ? ux_cfg_get(u, ULTRA_CFG_PX_LIMIT_MA) : 2250u;
    const uint32_t vk = u->vk_q8 != 0u ? u->vk_q8 : 256u;
    uint64_t maxs = (uint64_t)lim * 256u * 2550u / ((uint64_t)ma10 * vk);   /* mA = sum x ma10 / 2550 x vk / 256 */
    uint8_t nb;
    maxs = maxs * u->cl_q8 >> 8;
    if (maxs > 0x7FFFFFu) {                              /* (fx_limit_bri shifts it by 8; a strip's whole sum is under 2 M) */
        maxs = 0x7FFFFFu;
    }
    nb = fx_limit_bri(u->lim_bri, u->fsum, (uint32_t)maxs);
    if (nb < u->lim_bri) {
        u->lim_bri = nb;
    } else if (u->lim_bri < 255u && (uint64_t)u->fsum * 10u < maxs * 9u) {   /* headroom: back up, a step of an eighth */
        u->lim_bri = (uint8_t)(u->lim_bri + 1u + ((255u - u->lim_bri) >> 3));
        if (u->lim_bri < nb) {
            u->lim_bri = nb;
        }
    }
    if (u->meas_ma != 0u && u->meas_ma > lim) {         /* measured over the limit: trim */
        u->cl_q8 = (uint16_t)(u->cl_q8 - u->cl_q8 / 16u);
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

uint32_t udfx_render(udfx_t *u)
{
    ws_enc_t *e = &u->e;
    const uint32_t bpp = u->chip.bpp, npx = u->npx;
    uint32_t upto, n;
    if (!u->local || u->want_stream || e->ready || u->dark || npx == 0u) {
        return 0;
    }
    if (!u->begun && !ux_frame_begin(u)) {
        return 0;
    }
    upto = e->state == WS_FRAME ? e->i / bpp : npx;      /* (bytes before e->i are in the ring: their pixels are free) */
    if (upto > npx) {
        upto = npx;
    }
    n = upto - u->rendered;
    if (upto > u->rendered && (n >= UX_CHUNK || upto == npx)) {   /* (a chunk of 32 pixels, or what is left of the frame) */
        rgb_t px[UX_CHUNK];
        if (n > UX_CHUNK) {
            n = UX_CHUNK;
        }
        fx_engine_render(&u->eng, px, u->rendered, u->rendered + n);
        u->fsum += fx_pack(px, ux_pool + u->rendered * bpp, n, u->lim_bri, &u->pack);
        u->rendered += n;
        return 1;
    }
    if (u->rendered == npx && e->state == WS_LOW) {      /* the frame is whole and the last one is out: hand it over */
        u->black_sent = u->frame_dark;
        ux_limit(u);
        u->begun = 0;
        u->frames_done++;
        e->ready = 1;
    }
    return 0;
}

uint32_t udfx_flash_ok(const udfx_t *u)
{
    return !u->local || u->dark;
}

/* ------------------------------------------------------------------------------------------------ the service */
void udfx_service(udfx_t *u, const dline_t *d, uint32_t ms, uint32_t meas_ma, uint32_t vk_q8)
{
    const uint32_t now_w = d->wpos;
    uint32_t i, host, any_run = 0, quiet_ms;
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
    ux_ms_advance(u, host);

    for (i = 0; i < UX_SEGS; i++) {
        ux_seg_t *s = &u->seg[i];
        if (s->pend_have && (int32_t)(host - s->pend_at) >= 0) {
            const ultra_pixel_fx_t f = s->pend;
            s->pend_have = 0;
            ux_apply(u, i, &f, ux_ms(u, host), ux_ms(u, s->pend_at));
            if (!u->local && u->npx != 0u) {
                u->want_local = 1;
            }
        }
        any_run |= s->run;
    }

    if (u->reconf) {                                     /* the chip / pixels / order / fps changed */
        if (u->local) {
            u->want_stream = 1;
        } else {
            ux_derive(u);
            u->reconf = 0;
            u->begun = 0;
            ux_reapply(u, ux_ms(u, host));
            if (any_run && u->npx != 0u) {
                u->want_stream = 0;
                u->want_local = 1;
            }
        }
    }

    /* the standalone policy: the stored effect when the line stays quiet from power-up / goes quiet */
    quiet_ms = (d->wpos - d->edge_w) / DLINE_W_PER_MS;
    if (!u->local && !u->want_local && u->stored_have && u->npx != 0u && !any_run) {
        const uint32_t pol = ux_cfg_get(u, ULTRA_CFG_PX_POLICY), lt = ux_cfg_get(u, ULTRA_CFG_PX_LOSS_TIMEOUT);
        const uint32_t loss_ms = (lt != 0u ? lt : 30u) * 100u;
        const uint32_t never_seen = d->st.frames == 0u && d->st.marks == 0u;
        if (((pol & ULTRA_PXP_POWER_ON) && never_seen && quiet_ms >= UX_POWER_ON_MS && ms >= UX_POWER_ON_MS) ||
            ((pol & ULTRA_PXP_ON_LOSS) && !never_seen && quiet_ms >= loss_ms)) {
            uint32_t k;
            const uint32_t now_ms = ux_ms(u, host);
            for (k = 0; k < UX_SEGS; k++) {
                if (u->stored_have & (1u << k)) {
                    ultra_pixel_fx_t f = u->stored[k];
                    f.start_ms = 0;
                    if (f.transition_ms < UX_FADE_IN_MS) {
                        f.transition_ms = UX_FADE_IN_MS;
                    }
                    ux_apply(u, k, &f, now_ms, now_ms);
                }
            }
            u->standalone = 1;
            u->lock = 0;
            u->want_local = 1;
            u->frames_seen = d->st.frames;
        }
    }
    if (u->local && !u->want_stream) {                   /* a plain stream takes the strip back (not locked) */
        if ((ux_cfg_get(u, ULTRA_CFG_PX_POLICY) & ULTRA_PXP_STREAM_WINS) && !u->lock &&
            d->st.frames - u->frames_seen >= 2u) {
            ux_stream(u);
        }
    }
    if (!u->local && !u->want_local) {
        u->frames_seen = d->st.frames;
    }
    if (any_run && !u->local && !u->want_stream && !u->want_local && u->npx != 0u && !u->reconf) {
        u->want_local = 1;
    }

    /* the first frame's start word (the grid), refreshed while a switch waits; the grid's phase while LOCAL */
    if (u->want_local && !u->local) {
        if (u->period_us != 0u) {
            const uint32_t start = host + u->chip.reset_w * 4u / 9u + 1000u;   /* a reset + 1 ms of margin */
            const uint32_t td = (start + u->period_us - 1u) / u->period_us * u->period_us;
            u->first_w = udfx_word_at(u, td);
        } else {
            u->first_w = 0;
        }
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
    return (int32_t)(w - u->wait_w) >= (int32_t)UX_LATCH_WAIT_W ? DLINE_W_LATCH : u->latch_w;
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
        if (u->want_stream) {
            u->e.ready = 0;                              /* no new frame: the one in progress ends */
        }
        (void)ws_ring_fill(&u->e, &u->lut, ring, w);
        if (u->want_stream && u->e.state == WS_LOW && u->e.low_w >= u->e.reset_w && u->e.w == w &&
            dline_low_run(d) >= ux_need(u, w, 1u)) {
            u->wait_on = 0;
            u->local = 0;                                /* the hub's stream passes again, inside its latch */
            d->local = 0;
            u->want_stream = 0;
            u->begun = 0;
            u->switches++;
        }
    } else if (u->want_local && !u->want_stream && u->npx != 0u && !d->direct && !d->rec &&
               dline_low_run(d) >= ux_need(u, w, 1u)) {
        u->wait_on = 0;
        ws_enc_init(&u->e, &u->chip, ux_pool, u->npx, w, u->period_w);
        if (u->period_w != 0u && (int32_t)(u->first_w - u->e.next_w) > 0) {
            u->e.next_w = u->first_w;
        }
        u->begun = 0;
        u->rendered = 0;
        u->want_local = 0;
        u->local = 1;
        d->local = 1;
        u->frames_seen = d->st.frames;                   /* (the hub's frames up to here are not a controller taking over) */
        u->switches++;
        (void)ws_ring_fill(&u->e, &u->lut, ring, w);
    }
}

/* ------------------------------------------------------------------------------------------------ the record */
/* 128 bytes: magic (2), version (1), segments (1), seq (4), the eleven values (22), four segments of 23 bytes (92: the
 * PIXEL_FX fields but its START; effect 0 = none), the stored mask (1), 3 spare, the CRC-16 (never an erased value) */
static uint32_t ux_erased(uint32_t c)
{
    return c == 0xFFFFu || c == 0xE339u;
}

static void ux_put(uint8_t *p, uint32_t v, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        p[i] = (uint8_t)(v >> (8u * i));
    }
}

static uint32_t ux_get(const uint8_t *p, uint32_t n)
{
    uint32_t i, v = 0;
    for (i = 0; i < n; i++) {
        v |= (uint32_t)p[i] << (8u * i);
    }
    return v;
}

void udfx_rec_pack(const udfx_t *u, uint32_t seq, uint8_t *r)
{
    uint32_t i, k, c;
    for (i = 0; i < UX_REC_BYTES; i++) {
        r[i] = 0;
    }
    ux_put(r, UX_REC_MAGIC, 2);
    r[2] = 1;
    r[3] = UX_SEGS;
    ux_put(r + 4, seq, 4);
    for (i = 0; i < ULTRA_CFG_PX_COUNT; i++) {
        ux_put(r + 8u + 2u * i, u->cfg[i], 2);
    }
    for (i = 0; i < UX_SEGS; i++) {
        uint8_t *q = r + 30u + 23u * i;
        const ultra_pixel_fx_t *f = &u->stored[i];
        q[0] = f->effect;
        q[1] = f->speed;
        q[2] = f->intensity;
        q[3] = f->palette;
        for (k = 0; k < 9u; k++) {
            q[4u + k] = f->c[k / 3u][k % 3u];
        }
        q[13] = f->brightness;
        ux_put(q + 14, f->transition_ms, 2);
        ux_put(q + 16, f->seg_start, 2);
        ux_put(q + 18, f->seg_len, 2);
        q[20] = f->flags;
        q[21] = f->custom1;
        q[22] = f->custom2;
    }
    r[122] = u->stored_have;
    do {
        r[123]++;                                        /* (a CRC that would read erased: one more spare count) */
        c = ultra_crc16(r, UX_REC_BYTES - 2u);
    } while (ux_erased(c));
    ux_put(r + UX_REC_BYTES - 2u, c, 2);
}

int udfx_rec_valid(const uint8_t *r)
{
    const uint32_t c = ux_get(r + UX_REC_BYTES - 2u, 2);
    return ux_get(r, 2) == UX_REC_MAGIC && r[2] == 1u && r[3] == UX_SEGS && !ux_erased(c) &&
           c == ultra_crc16(r, UX_REC_BYTES - 2u);
}

void udfx_load(udfx_t *u, const uint8_t *r)
{
    uint32_t i, k;
    u->rec_seq = ux_get(r + 4, 4);
    for (i = 0; i < ULTRA_CFG_PX_COUNT; i++) {
        u->cfg[i] = (uint16_t)ux_get(r + 8u + 2u * i, 2);
    }
    for (i = 0; i < UX_SEGS; i++) {
        const uint8_t *q = r + 30u + 23u * i;
        ultra_pixel_fx_t *f = &u->stored[i];
        f->effect = q[0];
        f->speed = q[1];
        f->intensity = q[2];
        f->palette = q[3];
        for (k = 0; k < 9u; k++) {
            f->c[k / 3u][k % 3u] = q[4u + k];
        }
        f->brightness = q[13];
        f->transition_ms = (uint16_t)ux_get(q + 14, 2);
        f->seg_start = (uint16_t)ux_get(q + 16, 2);
        f->seg_len = (uint16_t)ux_get(q + 18, 2);
        f->flags = q[20];
        f->custom1 = q[21];
        f->custom2 = q[22];
        f->start_ms = 0;
    }
    u->stored_have = r[122];
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
    uint32_t i;
    if (part == ULTRA_PXG_PART_STATE) {
        ultra_put16(d + ULTRA_PXG_CATALOGUE, UX_CATALOGUE_VERSION);
        d[ULTRA_PXG_EFFECTS] = (uint8_t)FX_LAST;
        d[ULTRA_PXG_PALETTES] = (uint8_t)PAL_COUNT;
        d[ULTRA_PXG_CHIPS] = (uint8_t)ULTRA_PXC_COUNT;
        d[ULTRA_PXG_SEGMENTS] = (uint8_t)UX_SEGS;
        ultra_put16(d + ULTRA_PXG_MAX_PIXELS, u->max_px);
        d[ULTRA_PXG_MODE] = (uint8_t)(u->local ? (u->standalone ? ULTRA_PXM_STANDALONE : ULTRA_PXM_LOCAL) : ULTRA_PXM_STREAM);
        for (i = 0; i < UX_SEGS; i++) {
            d[ULTRA_PXG_RUNNING + i] = u->seg[i].run ? u->seg[i].p.effect : 0u;
            d[ULTRA_PXG_SEQ + i] = (u->seq_have & (1u << i)) ? u->seq5[i] : 0u;
        }
        ultra_put16(d + ULTRA_PXG_FPS10, u->fps10);
        d[ULTRA_PXG_LOAD] = u->load;
        d[ULTRA_PXG_LIMIT] = u->lim_bri;
        d[ULTRA_PXG_LATE] = u->e.late;
        ultra_put16(d + ULTRA_PXG_CONFIG_CRC, ultra_px_config_crc(u->cfg));
        return 0;
    }
    if (part == ULTRA_PXG_PART_EFFECTS || part == ULTRA_PXG_PART_PALETTES) {
        for (i = 0; i < 192u; i++) {
            if (part == ULTRA_PXG_PART_EFFECTS ? (i != 0u && i < FX_COUNT) : i < PAL_COUNT) {   /* (effect 0 is the stream) */
                d[i >> 3] = (uint8_t)(d[i >> 3] | (1u << (i & 7u)));
            }
        }
        return 0;
    }
    return -1;
}
