/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The effect engine's segments and transitions (fx.h layer 2): up to FX_MAX_SEG segments on the strip (start / length,
 * reverse, mirror), brightness and colour ramps and the effect crossfade done here, and the compositor that renders a
 * chunk of the strip. Time is host ms (wrapping uint32): an effect's own time is host time minus its anchor. */
#include "fx_int.h"

/* the effect's time (host ms since its anchor): a future anchor holds it at 0 until it is reached; after that it is plain modular
 * arithmetic, so an effect that has run past 2^31 ms (24.8 days) keeps going (fx_frame turns it into a saturating raw time) */
static uint32_t eff_time(uint32_t now, uint32_t anchor, uint8_t *started)
{
    if (!*started) {
        if ((int32_t)(now - anchor) < 0) {
            return 0;
        }
        *started = 1;
    }
    return now - anchor;
}

static uint32_t state_off(const fx_engine_t *e, uint32_t s)   /* each segment reserves the state of its length, in index order */
{
    uint32_t k, off = 0;
    for (k = 0; k < s; k++) {
        off += fx_state_bytes(FX_FIRE, e->seg[k].len);        /* (the most any effect needs) */
    }
    return off;
}

void fx_engine_init(fx_engine_t *e, uint16_t npx, uint8_t *pool, uint32_t pool_bytes)
{
    uint8_t *p = (uint8_t *)e;
    uint32_t k;
    for (k = 0; k < sizeof(*e); k++) {
        p[k] = 0;
    }
    for (k = 0; k < FX_MAX_SEG; k++) {
        e->seg[k].xf = -1;
    }
    e->pool = pool;
    e->pool_bytes = pool_bytes;
    e->npx = npx;
}

static void seg_params(const fx_seg_t *sg, const fx_params_t *p, uint32_t s, fx_params_t *q)
{
    *q = *p;
    q->vstart = (uint16_t)(p->vstart + sg->start);
    if (p->vlen != 0u && (sg->flags & FX_F_MIRROR)) {
        q->vlen = (uint16_t)((p->vlen + 1u) >> 1);
    }
    q->seed = p->seed + s * 0x01000193u;                     /* (the segments differ) */
}

static void drop_xfade(fx_engine_t *e, fx_seg_t *sg)
{
    if (sg->xf >= 0) {
        e->xf[sg->xf].used = 0;
        sg->xf = -1;
    }
}

static void begin_effect(fx_engine_t *e, uint32_t s, const fx_params_t *p, uint32_t now)
{
    fx_seg_t *sg = &e->seg[s];
    fx_params_t q;
    const uint32_t need = fx_state_bytes(p->id, sg->n_eff);
    uint8_t *st = 0;
    seg_params(sg, p, s, &q);
    if (need != 0u && sg->st_off + need <= e->pool_bytes) {
        st = e->pool + sg->st_off;
    }
    fx_begin(&sg->cur, &q, sg->n_eff, st);
    sg->started = 0;
    fx_frame(&sg->cur, eff_time(now, sg->anchor, &sg->started));
}

void fx_engine_set(fx_engine_t *e, uint32_t s, const fx_params_t *p, uint32_t now, uint32_t anchor)
{
    fx_seg_t *sg;
    uint32_t start, len, k, flags, n_eff, id, geom_same, same;
    if (s >= FX_MAX_SEG) {
        return;
    }
    sg = &e->seg[s];
    start = p->seg_start < e->npx ? p->seg_start : e->npx;
    len = p->seg_len != 0u ? p->seg_len : e->npx - start;
    if (len > e->npx - start) {
        len = e->npx - start;
    }
    if (p->id == FX_STREAM || len == 0u) {                   /* no local effect on this segment */
        drop_xfade(e, sg);
        sg->active = 0;
        sg->start = (uint16_t)start;
        sg->len = (uint16_t)len;
        return;
    }
    id = p->id < FX_COUNT ? p->id : (uint32_t)FX_SOLID;
    flags = p->flags & (FX_F_REVERSE | FX_F_MIRROR);
    n_eff = (flags & FX_F_MIRROR) ? (len + 1u) >> 1 : len;
    geom_same = sg->active && sg->start == start && sg->len == len && sg->flags == flags &&
                sg->tvstart == p->vstart && sg->tvlen == p->vlen;
    same = geom_same && sg->cur.id == id &&
           (p->palette == PAL_DEFAULT ? fx_default_palette((uint8_t)id) : p->palette) == sg->cur.pal_id;
    if (!sg->active) {
        sg->bri = 0;                                         /* switched on: from dark (ramping up if asked) */
    }
    if (same) {
        /* the same effect: bring it to now at the old speed, then change the settings; the picture stays */
        const uint32_t seed = p->seed + s * 0x01000193u;
        uint32_t k2, renewal;
        fx_frame(&sg->cur, eff_time(now, sg->anchor, &sg->started));
        renewal = sg->cur.speed == p->speed && sg->cur.intensity == p->intensity && sg->cur.custom1 == p->custom1 &&
                  sg->cur.custom2 == p->custom2 && sg->cur.seed == seed && sg->tbri == p->bri;
        for (k2 = 0; k2 < 3u; k2++) {
            renewal = renewal && sg->tc[k2].r == p->c[k2].r && sg->tc[k2].g == p->c[k2].g && sg->tc[k2].b == p->c[k2].b;
        }
        if (renewal) {                                       /* only the anchor may be new: keep the running time base */
            return;
        }
        fx_set_speed(&sg->cur, p->speed);
        sg->cur.intensity = p->intensity;
        sg->cur.custom1 = p->custom1;
        sg->cur.custom2 = p->custom2;
        sg->cur.seed = seed;
        if (sg->anchor != anchor) {                          /* a new anchor with other changes restarts the time-driven effects (the sun) */
            sg->anchor = anchor;
            sg->started = 0;
            sg->cur.t = sg->cur.t_last = eff_time(now, anchor, &sg->started);
        }
    } else {
        int xi = -1;
        if (geom_same && p->trans_ms != 0u && sg->bri != 0u &&
            !(fx_state_bytes((uint8_t)id, n_eff) != 0u && fx_state_bytes(sg->cur.id, sg->n_eff) != 0u)) {
            drop_xfade(e, sg);
            for (k = 0; k < FX_MAX_XFADE; k++) {
                if (!e->xf[k].used) {
                    xi = (int)k;
                    break;
                }
            }
        }
        if (xi >= 0) {                                       /* the old effect keeps running and fades out */
            fx_xfade_t *xf = &e->xf[xi];
            xf->prev = sg->cur;
            xf->anchor = sg->anchor;
            xf->started = sg->started;
            xf->t0 = now;
            xf->dur = p->trans_ms;
            xf->inv = 65536u / p->trans_ms;
            xf->w = 0;
            xf->used = 1;
            sg->xf = (int8_t)xi;
        } else {
            drop_xfade(e, sg);
        }
        sg->start = (uint16_t)start;
        sg->len = (uint16_t)len;
        sg->flags = (uint8_t)flags;
        sg->n_eff = (uint16_t)n_eff;
        sg->st_off = (uint16_t)state_off(e, s);
        sg->anchor = anchor;
        begin_effect(e, s, p, now);
    }
    sg->active = 1;
    for (k = 0; k < 3u; k++) {
        sg->tc[k] = p->c[k];
    }
    sg->tbri = p->bri;
    sg->tvstart = p->vstart;
    sg->tvlen = p->vlen;
    if (sg->bri == 0u) {                                     /* dark: nothing shows, so the new colours need no ramp */
        fx_set_colors(&sg->cur, p->c);
    }
    if (p->trans_ms != 0u) {                                 /* ramp the colours and the brightness from where they are */
        for (k = 0; k < 3u; k++) {
            sg->c0[k] = sg->cur.c[k];
        }
        sg->bri0 = sg->bri;
        sg->tr_t0 = now;
        sg->tr_dur = p->trans_ms;
        sg->tr_inv = 65536u / p->trans_ms;
    } else {
        sg->tr_inv = 0;
        sg->bri = p->bri;
        fx_set_colors(&sg->cur, p->c);
    }
}

static uint32_t ramp256(uint32_t elapsed, uint32_t dur, uint32_t inv)
{
    uint32_t w;
    if (elapsed >= dur) {
        return 256u;
    }
    w = (elapsed * inv) >> 8;
    return w > 256u ? 256u : w;
}

static uint8_t lerp256(uint32_t a, uint32_t b, uint32_t w)   /* w 0..256 */
{
    return (uint8_t)((int32_t)a + (((int32_t)b - (int32_t)a) * (int32_t)w >> 8));
}

void fx_engine_frame(fx_engine_t *e, uint32_t now)
{
    uint32_t s, k;
    e->now = now;
    for (s = 0; s < FX_MAX_SEG; s++) {
        fx_seg_t *sg = &e->seg[s];
        if (!sg->active) {
            continue;
        }
        if (sg->tr_inv != 0u) {
            const uint32_t w = ramp256(now - sg->tr_t0, sg->tr_dur, sg->tr_inv);
            rgb_t c[3];
            for (k = 0; k < 3u; k++) {
                c[k].r = lerp256(sg->c0[k].r, sg->tc[k].r, w);
                c[k].g = lerp256(sg->c0[k].g, sg->tc[k].g, w);
                c[k].b = lerp256(sg->c0[k].b, sg->tc[k].b, w);
            }
            sg->bri = lerp256(sg->bri0, sg->tbri, w);
            fx_set_colors(&sg->cur, c);
            if (w >= 256u) {
                sg->tr_inv = 0;
                sg->bri = sg->tbri;
            }
        }
        fx_frame(&sg->cur, eff_time(now, sg->anchor, &sg->started));
        if (sg->xf >= 0) {
            fx_xfade_t *xf = &e->xf[sg->xf];
            const uint32_t w = ramp256(now - xf->t0, xf->dur, xf->inv);
            if (w >= 256u) {
                drop_xfade(e, sg);
            } else {
                xf->w = (uint8_t)(w > 255u ? 255u : w);
                fx_frame(&xf->prev, eff_time(now, xf->anchor, &xf->started));
            }
        }
    }
}

uint32_t fx_engine_dark(const fx_engine_t *e)
{
    uint32_t s;
    for (s = 0; s < FX_MAX_SEG; s++) {
        const fx_seg_t *sg = &e->seg[s];
        if (sg->active && (sg->bri != 0u || sg->tr_inv != 0u || sg->tbri != 0u)) {
            return 0;
        }
    }
    return 1;
}

/* effect pixels [e0, e1) of the segment into dst[0 .. e1 - e0) (the crossfade: the old effect blended under), reversed
 * when asked: the strip pixel i of the piece is effect pixel e1 - 1 - i */
static void render_piece(const fx_engine_t *e, fx_seg_t *sg, rgb_t *dst, uint32_t e0, uint32_t e1, int rev)
{
    rgb_t t1[FX_CHUNK], t2[FX_CHUNK];
    const fx_xfade_t *xf = sg->xf >= 0 ? &e->xf[sg->xf] : 0;
    const uint32_t cnt = e1 - e0;
    uint32_t k;
    if (!rev && xf == 0) {
        fx_render(&sg->cur, dst, e0, e1);
        return;
    }
    fx_render(&sg->cur, t1, e0, e1);
    if (xf != 0) {
        fx_render((fx_ctx_t *)&xf->prev, t2, e0, e1);
        for (k = 0; k < cnt; k++) {
            t1[k] = fx_blend(t2[k], t1[k], xf->w);
        }
    }
    if (rev) {
        for (k = 0; k < cnt; k++) {
            fx_put(dst + k, t1[cnt - 1u - k]);
        }
    } else {
        for (k = 0; k < cnt; k++) {
            fx_put(dst + k, t1[k]);
        }
    }
}

static void render_segment(fx_engine_t *e, fx_seg_t *sg, rgb_t *out, uint32_t lo, uint32_t hi, uint32_t from)
{
    const uint32_t l0 = lo - sg->start, l1 = hi - sg->start, len = sg->len;    /* segment-local [l0, l1) */
    rgb_t *dst = out + (lo - from);
    uint32_t k;
    if (sg->flags & FX_F_MIRROR) {                       /* effect pixel l for l < n_eff, len - 1 - l after (reverse is moot) */
        const uint32_t half = sg->n_eff;
        const uint32_t a1 = l1 < half ? l1 : half, b0 = l0 > half ? l0 : half;
        if (l0 < a1) {
            render_piece(e, sg, dst, l0, a1, 0);
        }
        if (b0 < l1) {
            render_piece(e, sg, dst + (b0 - l0), len - l1, len - b0, 1);
        }
    } else if (sg->flags & FX_F_REVERSE) {
        render_piece(e, sg, dst, len - l1, len - l0, 1);
    } else {
        render_piece(e, sg, dst, l0, l1, 0);
    }
    if (sg->bri != 255u) {
        for (k = 0; k < hi - lo; k++) {
            fx_put(dst + k, fx_scale_rgb(dst[k], sg->bri));
        }
    }
}

void fx_engine_render(fx_engine_t *e, rgb_t *out, uint32_t from, uint32_t to)
{
    uint32_t s, k;
    for (k = 0; k < to - from; k++) {
        out[k].r = out[k].g = out[k].b = 0;
    }
    for (s = 0; s < FX_MAX_SEG; s++) {
        fx_seg_t *sg = &e->seg[s];
        const uint32_t lo = from > sg->start ? from : sg->start, end = (uint32_t)sg->start + sg->len;
        const uint32_t hi = to < end ? to : end;
        if (sg->active && sg->bri != 0u && lo < hi) {
            render_segment(e, sg, out, lo, hi, from);
        }
    }
}
