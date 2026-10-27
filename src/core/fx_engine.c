/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The effect engine's segments and transitions (fx.h layer 2): up to FX_MAX_SEG segments on the strip (start / length,
 * reverse, mirror), brightness and colour ramps and the effect crossfade done here, and the compositor that renders a
 * chunk of the strip. Time is host ms (wrapping uint32): an effect's own time is host time minus its anchor. */
#include "fx_int.h"
#include <string.h>

#define FX_FINISH_MS 250u
static uint32_t ramp256(uint32_t elapsed, uint32_t dur, uint32_t inv);
static uint8_t lerp256(uint32_t a, uint32_t b, uint32_t w);
static void ramp_frame(fx_seg_t *sg, uint32_t now);

static void frame(const fx_engine_t *e, fx_ctx_t *x, uint32_t t)
{
    if (e->deferred) fx_frame_begin(x, t);
    else fx_frame(x, t);
}

static uint32_t xfade_weight(const fx_xfade_t *xf, uint32_t now)
{
    return now - xf->t0 >= xf->dur ? 256u : xf->w0 + (((now - xf->t0) * xf->inv) >> 16);
}

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

static uint32_t state_off(const fx_engine_t *e, uint32_t s)
{
    /* Fixed slots: another segment may be configured later or resized while this
     * Fire (including a fading-out Fire) still owns its heat cells. */
    return s * fx_state_bytes(FX_FIRE, e->npx);
}

void fx_engine_init(fx_engine_t *e, uint16_t npx, uint8_t *pool, uint32_t pool_bytes)
{
    uint32_t k;
    memset(e, 0, sizeof(*e));
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
    const fx_ctx_t *prev = sg->xf >= 0 ? &e->xf[sg->xf].prev : 0;
    const uint32_t share = prev && prev->id == FX_FIRE && p->id == FX_FIRE;
    seg_params(sg, p, s, &q);
    if (need != 0u && sg->st_off + need <= e->pool_bytes) {
        st = e->pool + sg->st_off;
    }
    fx_begin(&sg->cur, &q, sg->n_eff, share ? 0 : st);
    if (!share && st && s < FX_FIRE_SMOOTH) {
        const uint32_t cells = fx_state_bytes(FX_FIRE, e->npx);
        const uint32_t off = (FX_MAX_SEG + s) * cells;
        if (off + cells <= e->pool_bytes) {
            sg->cur.fire_prev = e->pool + off;
            sg->cur.u[2] = FX_FIRE_INIT;             /* clear the extra heat in bounded slices, too */
        }
    }
    sg->started = 0;
    frame(e, &sg->cur, eff_time(now, sg->anchor, &sg->started));
    if (share) {
        sg->cur.state = prev->state;                     /* Fire palette changes blend over one live heat field */
        sg->cur.fire_prev = prev->fire_prev;
        sg->cur.u[0] = prev->u[0];
        sg->cur.et_frac = prev->et_frac;
        if (sg->cur.seed == prev->seed) sg->cur.u[1] = prev->u[1];
    }
}

#if defined(__GNUC__)
__attribute__((optimize("no-tree-sra")))
#endif
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
           fx_pal_resolve((uint8_t)id, p->palette) == sg->cur.pal_id;
    if (sg->xf >= 0 && (!same || (e->xf[sg->xf].used & 2u))) {
        fx_xfade_t *xf = &e->xf[sg->xf];
        /* Keep both live contexts. Later arrivals replace only the target, never
         * extend the first replacement's finish deadline. */
        xf->next = *p;
        xf->next_anchor = anchor;
        /* A newly received target cannot start before this call. An expired
         * fade may still be attached until the next frame preparation. */
        if (now - xf->t0 >= xf->dur) xf->t0 = now - xf->dur;
        else if (!(xf->used & 2u) && xf->dur - (now - xf->t0) > FX_FINISH_MS) {
            const uint32_t w = xfade_weight(xf, now);
            xf->w0 = (uint8_t)w;
            xf->t0 = now;
            xf->dur = FX_FINISH_MS;
            xf->inv = ((256u - w) << 16) / FX_FINISH_MS;
            if (sg->tr_inv) {
                ramp_frame(sg, now);
                memcpy(sg->c0, sg->cur.c, sizeof sg->c0);
                sg->bri0 = sg->bri;
                sg->tr_t0 = now;
                sg->tr_dur = FX_FINISH_MS;
                sg->tr_inv = 16777216u / FX_FINISH_MS;
            }
        }
        xf->used |= 2u;
        return;
    }
    if (!sg->active) {
        sg->bri = 0;                                         /* switched on: from dark (ramping up if asked) */
    }
    if (same) {
        /* the same effect: bring it to now at the old speed, then change the settings; the picture stays */
        const uint32_t seed = p->seed + s * 0x01000193u;
        const uint8_t *tc = (const uint8_t *)sg->tc, *pc = (const uint8_t *)p->c;
        uint32_t k2, renewal;
        frame(e, &sg->cur, eff_time(now, sg->anchor, &sg->started));
        renewal = sg->cur.speed == p->speed && sg->cur.intensity == p->intensity && sg->cur.custom1 == p->custom1 &&
                  sg->cur.custom2 == p->custom2 && sg->cur.seed == seed && sg->tbri == p->bri;
        for (k2 = 0; k2 < sizeof sg->tc; k2++) {          /* (the three colours, byte by byte) */
            renewal = renewal && tc[k2] == pc[k2];
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
        if (geom_same && p->trans_ms != 0u && sg->bri != 0u) {
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
            xf->inv = 16777216u / p->trans_ms;
            xf->w = 0;
            xf->w0 = 0;
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
    memcpy(sg->tc, p->c, sizeof sg->tc);
    sg->tbri = p->bri;
    sg->tvstart = p->vstart;
    sg->tvlen = p->vlen;
    if (same && sg->bri == 0u) {                             /* a new effect already has these colours from fx_begin */
        fx_set_colors(&sg->cur, p->c);
    }
    if (p->trans_ms != 0u) {                                 /* ramp the colours and the brightness from where they are */
        memcpy(sg->c0, sg->cur.c, sizeof sg->c0);
        sg->bri0 = sg->bri;
        sg->tr_t0 = now;
        sg->tr_dur = p->trans_ms;
        sg->tr_inv = 16777216u / p->trans_ms;
    } else {
        sg->tr_inv = 0;
        sg->bri = p->bri;
        if (same) fx_set_colors(&sg->cur, p->c);
    }
}

static uint32_t ramp256(uint32_t elapsed, uint32_t dur, uint32_t inv)
{
    uint32_t w;
    if (elapsed >= dur) {
        return 256u;
    }
    /* elapsed < dur <= 65535: the product stays below 2^24. Q24 keeps even
     * a 65-second transition within one weight step of its linear ramp. */
    w = (elapsed * inv) >> 16;
    return w > 256u ? 256u : w;
}

static uint8_t lerp256(uint32_t a, uint32_t b, uint32_t w)   /* w 0..256 */
{
    return (uint8_t)((int32_t)a + (((int32_t)b - (int32_t)a) * (int32_t)w >> 8));
}

static void ramp_frame(fx_seg_t *sg, uint32_t now)
{
    uint32_t k;
    if (sg->tr_inv != 0u) {
        const uint32_t w = ramp256(now - sg->tr_t0, sg->tr_dur, sg->tr_inv);
        const uint8_t *c0 = (const uint8_t *)sg->c0, *tc = (const uint8_t *)sg->tc;   /* (c0 / tc: the three colours, then bri0 / tbri) */
        uint8_t *c = (uint8_t *)sg->cur.c;
        for (k = 0; k < sizeof sg->c0; k++) {
            c[k] = lerp256(c0[k], tc[k], w);
        }
        sg->bri = lerp256(sg->bri0, sg->tbri, w);
        fx_set_colors(&sg->cur, sg->cur.c);              /* (in place: rebuilds a c1-c2-c3 palette) */
        if (w >= 256u) {
            sg->tr_inv = 0;
            sg->bri = sg->tbri;
        }
    }
}

uint32_t fx_engine_dequeue(fx_engine_t *e, uint32_t s, uint32_t now)
{
    fx_seg_t *sg = &e->seg[s];
    if (sg->xf >= 0) {
        fx_xfade_t *xf = &e->xf[sg->xf];
        if ((xf->used & 2u) && now - xf->t0 >= xf->dur) {
            const uint32_t at = xf->t0 + xf->dur, anchor = xf->next_anchor;
            ramp_frame(sg, at);
            drop_xfade(e, sg);                          /* releases any fading-out Fire before fx_begin clears heat */
            fx_engine_set(e, s, &xf->next, at, anchor);
            return 1;
        }
    }
    return 0;
}

void fx_engine_frame_seg(fx_engine_t *e, uint32_t s, uint32_t now)
{
    if (!e->deferred) (void)fx_engine_dequeue(e, s, now);
    e->now = now;
    fx_seg_t *sg = &e->seg[s];
    if (!sg->active) {
        return;
    }
    ramp_frame(sg, now);
    frame(e, &sg->cur, eff_time(now, sg->anchor, &sg->started));
    if (sg->xf >= 0) {
        fx_xfade_t *xf = &e->xf[sg->xf];
        const uint32_t w = xfade_weight(xf, now);
        if (w >= 256u) {
            drop_xfade(e, sg);
        } else {
            xf->w = (uint8_t)(w > 255u ? 255u : w);
            if (!xf->prev.state || xf->prev.state != sg->cur.state) {
                frame(e, &xf->prev, eff_time(now, xf->anchor, &xf->started));
            } else {                                    /* shared heat has one interpolation phase, too */
                xf->prev.u[0] = sg->cur.u[0];
                xf->prev.et_frac = sg->cur.et_frac;
            }
        }
    }
}

uint32_t fx_engine_step(fx_engine_t *e)
{
    uint32_t s;
    for (s = 0; s < FX_MAX_SEG; s++) {
        fx_seg_t *sg = &e->seg[s];
        if (!sg->active) continue;
        if (fx_frame_step(&sg->cur)) return 1;
        if (sg->xf >= 0) {
            fx_ctx_t *prev = &e->xf[sg->xf].prev;
            if (prev->state != sg->cur.state && fx_frame_step(prev)) return 1;
        }
    }
    return 0;
}

void fx_engine_frame(fx_engine_t *e, uint32_t now)
{
    uint32_t s;
    for (s = 0; s < FX_MAX_SEG; s++) {
        while (fx_engine_step(e)) { }
        (void)fx_engine_dequeue(e, s, now);
        while (fx_engine_step(e)) { }
        fx_engine_frame_seg(e, s, now);
    }
    while (fx_engine_step(e)) { }
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

uint32_t fx_engine_chunk(const fx_engine_t *e, uint32_t quota)
{
    uint32_t s, fire = 0, layers = 0, chunk = FX_CHUNK;
    for (s = 0; s < FX_MAX_SEG; s++) {
        const fx_seg_t *sg = &e->seg[s];
        if (!sg->active) continue;
        layers++;
        fire |= sg->cur.id == FX_FIRE;
#if !FX_NO_PACIFICA                                      /* (a build without Pacifica refuses the id: udfx.c) */
        if (sg->cur.id == FX_PACIFICA) chunk = 4;
#endif
        if (sg->xf >= 0) {
            const uint32_t id = e->xf[sg->xf].prev.id;
            layers++;
            fire |= id == FX_FIRE;
#if !FX_NO_PACIFICA
            if (id == FX_PACIFICA) chunk = 4;
#endif
        }
    }
    /* Fire's setup is sliced too; bound its compositor/packer calls, including
     * overlapping segments and a fade to/from Pacifica. */
    if (fire) chunk = layers > 1u ? 2u : 16u;
    /* The controller supplies a smaller total pixel budget on chain lines.
     * Count fading-out contexts as work too, even when segments overlap. */
#if !FX_NO_PACIFICA
    if (!fire && chunk == 4u) return chunk; /* Pacifica needs a setup-aware continuation, not a smaller quota. */
#endif
    if (quota < FX_CHUNK && layers) {
        quota = layers > FX_MAX_SEG ? 1u : quota / layers;
        if (!quota) quota = 1;
        if (chunk > quota) chunk = quota;
    }
    return chunk;
}

/* effect pixels [e0, e1) of the segment into dst[0 .. e1 - e0) (the crossfade: the old effect blended under), reversed
 * when asked: the strip pixel i of the piece is effect pixel e1 - 1 - i */
static void render_piece(const fx_engine_t *e, fx_seg_t *sg, rgb_t *dst, uint32_t e0, uint32_t e1, int rev)
{
    rgb_t t[FX_CHUNK];
    const fx_xfade_t *xf = sg->xf >= 0 ? &e->xf[sg->xf] : 0;
    const uint32_t cnt = e1 - e0;
    uint32_t k;
    if (!rev && xf == 0) {                               /* the plain segment: straight into the chunk (the hot path, as it was) */
        fx_render(&sg->cur, dst, e0, e1);
        return;
    }
    fx_render(&sg->cur, rev ? t : dst, e0, e1);
    if (rev) {
        for (k = 0; k < cnt; k++) {
            fx_put(dst + k, t[cnt - 1u - k]);
        }
    }
    if (xf != 0) {                                       /* the old effect blended under (reversed the same way) */
        fx_render((fx_ctx_t *)&xf->prev, t, e0, e1);
        for (k = 0; k < cnt; k++) {
            fx_put(dst + k, fx_blend(t[rev ? cnt - 1u - k : k], dst[k], xf->w));
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
    for (k = 0; k < to - from; k++) {                    /* (a loop, not memset: the bench's libc memset costs 10 cycles a pixel) */
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
