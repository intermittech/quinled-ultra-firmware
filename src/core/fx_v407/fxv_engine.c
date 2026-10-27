/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The V407 layer's segments and transitions (fxv_engine.h): fx_engine.c's behaviour (segments with start / length,
 * reverse, mirror; colour and brightness ramps; the effect crossfade with a queued replacement; renewals) on fxv
 * contexts, each owning one of six fixed arena slots (four segments + two fading out; a fade moves the old context and
 * its slot, the new effect takes a free slot: no state is shared, nothing is compacted), the global particle slab with
 * owner quotas and the particle step-down, and a 16-bit compositor. Time is host ms (wrapping); an effect's time is
 * host time minus its anchor. Everything bounded runs in fxv_engine_step slices; JS: fx/v407/engine.js. */
#include "fxv_engine.h"
#include "../math32.h"

#define FXV_FINISH_MS 250u

static uint32_t ramp256(uint32_t elapsed, uint32_t dur, uint32_t inv)
{
    uint32_t w;
    if (elapsed >= dur) return 256u;
    w = (elapsed * inv) >> 16;
    return w > 256u ? 256u : w;
}

static uint8_t lerp256(uint32_t a, uint32_t b, uint32_t w)
{
    return (uint8_t)((int32_t)a + (((int32_t)b - (int32_t)a) * (int32_t)w >> 8));
}

static uint32_t xfade_weight(const fxv_xfade_t *xf, uint32_t now)
{
    return now - xf->t0 >= xf->dur ? 256u : xf->w0 + (((now - xf->t0) * xf->inv) >> 16);
}

static uint32_t eff_time(uint32_t now, uint32_t anchor, uint8_t *started)
{
    if (!*started) {
        if ((int32_t)(now - anchor) < 0) return 0;
        *started = 1;
    }
    return now - anchor;
}

/* ------------------------------------------------------------------------------- slots */
static uint32_t slot_alloc(fxv_engine_t *e)
{
    uint32_t k;
    for (k = 0; k < FXV_SLOTS; k++) {
        if (!(e->slot_used & (1u << k))) {
            e->slot_used = (uint8_t)(e->slot_used | (1u << k));
            return k;
        }
    }
    return FXV_SLOTS;                                    /* (cannot happen: six contexts, six slots) */
}

static void ctx_release(fxv_engine_t *e, fxv_ctx_t *x)
{
    if (x->d) {
        fxv_end(x);
        e->slot_used = (uint8_t)(e->slot_used & ~(1u << x->slot));
        e->rebal = 1;
    }
}

static void ctx_move(fxv_ctx_t *d, fxv_ctx_t *s)        /* d = s, s off (a word at a time: a set is one step) */
{
    fxv_word_t *a = (fxv_word_t *)(void *)d;
    const fxv_word_t *b = (const fxv_word_t *)(const void *)s;
    uint32_t k;
    for (k = 0; k < (uint32_t)(sizeof(*d) / 4u); k++) a[k] = b[k];
    s->d = 0;
}

static void drop_xfade(fxv_engine_t *e, fxv_seg_t *sg)
{
    if (sg->xf >= 0) {
        fxv_xfade_t *xf = &e->xf[sg->xf];
        ctx_release(e, &xf->prev);
        xf->used = 0;
        sg->xf = -1;
    }
}

void fxv_engine_init(fxv_engine_t *e, uint32_t npx, uint32_t fmt, uint8_t *arena, fxv_particle_t *slab)
{
    uint8_t *p = (uint8_t *)e;
    uint32_t k;
    for (k = 0; k < (uint32_t)sizeof(*e); k++) p[k] = 0;
    for (k = 0; k < FXV_MAX_SEG; k++) e->seg[k].xf = -1;
    e->npx = (uint16_t)npx;
    e->fmt = (uint8_t)fmt;
    e->arena = arena;
    e->slab = slab;
}

static void seg_params(const fxv_seg_t *sg, const fx_params_t *p, uint32_t s, fx_params_t *q)
{
    *q = *p;
    q->vstart = (uint16_t)(p->vstart + sg->start);
    if (p->vlen != 0u && (sg->flags & FX_F_MIRROR)) q->vlen = (uint16_t)((p->vlen + 1u) >> 1);
    q->seed = p->seed + s * 0x01000193u;
}

static void begin_effect(fxv_engine_t *e, uint32_t s, const fx_params_t *p, uint32_t now)
{
    fxv_seg_t *sg = &e->seg[s];
    fx_params_t q;
    const uint32_t slot = slot_alloc(e);
    seg_params(sg, p, s, &q);
    fxv_begin(&sg->cur, &q, sg->n_eff, e->fmt, e->arena + slot * FXV_SLOT_MAX, slot);
    sg->tseed = q.seed;
    sg->started = 0;
    fxv_frame_begin(&sg->cur, eff_time(now, sg->anchor, &sg->started));
    e->rebal = 1;
}

static void ramp_frame(fxv_seg_t *sg, uint32_t now)
{
    uint32_t k;
    if (sg->tr_inv != 0u) {
        const uint32_t w = ramp256(now - sg->tr_t0, sg->tr_dur, sg->tr_inv);
        rgb_t c[3];
        for (k = 0; k < 3u; k++) {
            c[k].r = lerp256(sg->c0[k].r, sg->tc[k].r, w);
            c[k].g = lerp256(sg->c0[k].g, sg->tc[k].g, w);
            c[k].b = lerp256(sg->c0[k].b, sg->tc[k].b, w);
        }
        sg->bri = lerp256(sg->bri0, sg->tbri, w);
        fxv_set_colors(&sg->cur, c);
        if (w >= 256u) {
            sg->tr_inv = 0;
            sg->bri = sg->tbri;
        }
    }
}

static uint32_t same_colors(const rgb_t *a, const rgb_t *b)
{
    uint32_t k;
    for (k = 0; k < 3u; k++) {
        if (a[k].r != b[k].r || a[k].g != b[k].g || a[k].b != b[k].b) return 0;
    }
    return 1;
}

void fxv_engine_set(fxv_engine_t *e, uint32_t s, const fx_params_t *p, uint32_t now, uint32_t anchor)
{
    fxv_seg_t *sg;
    uint32_t start, len, k, flags, n_eff, id, geom_same, same, pal;
    if (s >= FXV_MAX_SEG) return;
    sg = &e->seg[s];
    start = p->seg_start < e->npx ? p->seg_start : e->npx;
    len = p->seg_len != 0u ? p->seg_len : e->npx - start;
    if (len > e->npx - start) len = e->npx - start;
    if (p->id == FX_STREAM || len == 0u || !fxv_built(p->id)) {   /* no local effect (an unbuilt id never renders Solid) */
        drop_xfade(e, sg);
        ctx_release(e, &sg->cur);
        sg->active = 0;
        sg->start = (uint16_t)start;
        sg->len = (uint16_t)len;
        return;
    }
    id = p->id;
    flags = p->flags & (FX_F_REVERSE | FX_F_MIRROR);
    n_eff = (flags & FX_F_MIRROR) ? (len + 1u) >> 1 : len;
    pal = fxv_pal_resolve(id, p->palette);
    geom_same = sg->active && sg->start == start && sg->len == len && sg->flags == flags &&
                sg->tvstart == p->vstart && sg->tvlen == p->vlen;
    same = geom_same && sg->cur.id == id && pal == sg->cur.palette;
    if (sg->xf >= 0 && (!same || (e->xf[sg->xf].used & 2u))) {
        fxv_xfade_t *xf = &e->xf[sg->xf];
        xf->next = *p;                                   /* keep both live contexts; the latest arrival is the target */
        xf->next_anchor = anchor;
        if (now - xf->t0 >= xf->dur) xf->t0 = now - xf->dur;
        else if (!(xf->used & 2u) && xf->dur - (now - xf->t0) > FXV_FINISH_MS) {
            const uint32_t w = xfade_weight(xf, now);
            xf->w0 = (uint8_t)w;
            xf->t0 = now;
            xf->dur = FXV_FINISH_MS;
            xf->inv = ((256u - w) << 16) / FXV_FINISH_MS;
            if (sg->tr_inv) {
                ramp_frame(sg, now);
                for (k = 0; k < 3u; k++) sg->c0[k] = sg->cur.c[k];
                sg->bri0 = sg->bri;
                sg->tr_t0 = now;
                sg->tr_dur = FXV_FINISH_MS;
                sg->tr_inv = 16777216u / FXV_FINISH_MS;
            }
        }
        xf->used |= 2u;
        return;
    }
    if (!sg->active) sg->bri = 0;                        /* switched on: from dark */
    if (same) {
        const uint32_t seed = p->seed + s * 0x01000193u;
        uint32_t renewal;
        fxv_clock(&sg->cur, eff_time(now, sg->anchor, &sg->started));   /* to now at the old speed */
        renewal = sg->cur.speed == p->speed && sg->cur.intensity == p->intensity && sg->cur.custom1 == p->custom1 &&
                  sg->cur.custom2 == p->custom2 && sg->tseed == seed && sg->tbri == p->bri && same_colors(sg->tc, p->c);
        if (renewal) return;                             /* only the anchor may be new: the time base runs on */
        if (sg->tseed != seed) sg->cur.seed = seed;
        sg->tseed = seed;
        fxv_set_params(&sg->cur, p->speed, p->intensity, p->custom1, p->custom2, sg->cur.seed);
        if (sg->anchor != anchor) {                      /* a new anchor with other changes restarts time-driven effects */
            sg->anchor = anchor;
            sg->started = 0;
            sg->cur.t = sg->cur.t_last = eff_time(now, anchor, &sg->started);
        }
        e->rebal = 1;                                    /* (speed: the particle model) */
    } else {
        int xi = -1;
        if (geom_same && p->trans_ms != 0u && sg->bri != 0u) {
            drop_xfade(e, sg);
            for (k = 0; k < FXV_MAX_XFADE; k++) {
                if (!e->xf[k].used) { xi = (int)k; break; }
            }
        }
        if (xi >= 0) {                                   /* the old effect (and its slot) keeps running and fades out */
            fxv_xfade_t *xf = &e->xf[xi];
            ctx_move(&xf->prev, &sg->cur);
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
            ctx_release(e, &sg->cur);
        }
        sg->start = (uint16_t)start;
        sg->len = (uint16_t)len;
        sg->flags = (uint8_t)flags;
        sg->n_eff = (uint16_t)n_eff;
        sg->anchor = anchor;
        begin_effect(e, s, p, now);
    }
    sg->active = 1;
    for (k = 0; k < 3u; k++) sg->tc[k] = p->c[k];
    sg->tbri = p->bri;
    sg->tvstart = p->vstart;
    sg->tvlen = p->vlen;
    if (same && sg->bri == 0u) fxv_set_colors(&sg->cur, p->c);
    if (p->trans_ms != 0u) {                             /* ramp the colours and the brightness from where they are */
        for (k = 0; k < 3u; k++) sg->c0[k] = sg->cur.c[k];
        sg->bri0 = sg->bri;
        sg->tr_t0 = now;
        sg->tr_dur = p->trans_ms;
        sg->tr_inv = 16777216u / p->trans_ms;
    } else {
        sg->tr_inv = 0;
        sg->bri = p->bri;
        if (same) fxv_set_colors(&sg->cur, p->c);
    }
}

uint32_t fxv_engine_dequeue(fxv_engine_t *e, uint32_t s, uint32_t now)
{
    fxv_seg_t *sg = &e->seg[s];
    if (sg->xf >= 0) {
        fxv_xfade_t *xf = &e->xf[sg->xf];
        if ((xf->used & 2u) && now - xf->t0 >= xf->dur) {
            const uint32_t at = xf->t0 + xf->dur, anchor = xf->next_anchor;
            const fx_params_t next = xf->next;
            ramp_frame(sg, at);
            drop_xfade(e, sg);
            fxv_engine_set(e, s, &next, at, anchor);
            return 1;
        }
    }
    return 0;
}

/* the colour ramp's step; the effect's frame start and the fading one's are the next engine steps', before anything
 * else (one effect's start a step: Colors 1-2-3's ramp + Pacifica's start + Koi Pond's fading start were 5,283 cycles) */
void fxv_engine_frame_seg(fxv_engine_t *e, uint32_t s, uint32_t now)
{
    fxv_seg_t *sg = &e->seg[s];
    e->now = now;
    if (!sg->active) return;
    ramp_frame(sg, now);
    sg->due = 1;
    if (sg->xf >= 0) {
        fxv_xfade_t *xf = &e->xf[sg->xf];
        const uint32_t w = xfade_weight(xf, now);
        if (w >= 256u) {
            drop_xfade(e, sg);
        } else {
            xf->w = (uint8_t)(w > 255u ? 255u : w);
            xf->used = (uint8_t)(xf->used | 4u);
        }
    }
}

/* ------------------------------------------------------------------------------- particles: quotas, step-down, moves */
static fxv_ctx_t *slot_owner(fxv_engine_t *e, uint32_t o, uint32_t *fading)
{
    uint32_t s;
    for (s = 0; s < FXV_MAX_SEG; s++) {
        fxv_seg_t *sg = &e->seg[s];
        if (sg->active && sg->cur.d && sg->cur.slot == o) { *fading = 0; return &sg->cur; }
        if (sg->xf >= 0 && e->xf[sg->xf].prev.d && e->xf[sg->xf].prev.slot == o) { *fading = 1; return &e->xf[sg->xf].prev; }
    }
    return 0;
}

/* ticks one 42 fps frame (23,810 us) runs at the context's speed: the model's frame */
static uint32_t ticks_42(const fxv_ctx_t *x)
{
    const uint32_t rate = x->speed == 0u ? 0u : 8u + ((x->speed * 15u) >> 3);
    return x->tick_ms ? (23810u * rate / 256u + 999u) / 1000u / x->tick_ms + 1u : 0u;
}

uint32_t fxv_engine_cost(const fxv_engine_t *e)
{
    return e->cost;
}

/* the new layout: current owners share FXV_PARTICLES - FXV_PART_RESERVE by length (capped by their design count >>
 * tier), fading-out owners keep what they have of the rest; the tier rises (all current owners) while the modeled frame
 * is over FXV_BUDGET_CYCLES. Ranges are contiguous in slot order. One tier a step (six owners at four tiers in one step
 * cost ~5,600 cycles): quota_tier leaves the current owners' shares in e->ncount and the model in e->cost. */
static uint32_t quota_tier(fxv_engine_t *e, uint32_t tier)
{
    uint32_t want[FXV_SLOTS], n[FXV_SLOTS], fad[FXV_SLOTS], q[FXV_SLOTS], o, sumw = 0, sumn = 0, left, cost = 0;
    fxv_ctx_t *x[FXV_SLOTS];
    for (o = 0; o < FXV_SLOTS; o++) {
        x[o] = slot_owner(e, o, &fad[o]);
        want[o] = n[o] = q[o] = 0;
        if (!x[o]) continue;
        {
            const fxv_meta_t *m = &fxv_meta[x[o]->id];
            if (m->particles != 0u && !fad[o]) {
                want[o] = m->particles >> tier;
                n[o] = x[o]->n + 1u;
                sumw += want[o];
                sumn += n[o];
            }
        }
    }
    left = FXV_PARTICLES - FXV_PART_RESERVE;
    for (o = 0; o < FXV_SLOTS; o++) {                    /* current owners: all they want, or a share by length */
        if (!x[o] || fad[o] || !want[o]) continue;
        q[o] = sumw <= left ? want[o] : m32_muldiv(FXV_PARTICLES - FXV_PART_RESERVE, n[o], sumn);
        if (q[o] > want[o]) q[o] = want[o];
    }
    for (o = 0; o < FXV_SLOTS; o++) left -= q[o];
    for (o = 0; o < FXV_SLOTS; o++) {                    /* what is left, in slot order */
        if (x[o] && !fad[o] && want[o] > q[o]) {
            const uint32_t add = want[o] - q[o] < left ? want[o] - q[o] : left;
            q[o] += add;
            left -= add;
        }
    }
    for (o = 0; o < FXV_SLOTS; o++) {                    /* the frame model at this tier */
        e->ncount[o] = (uint16_t)q[o];
        if (!x[o]) continue;
        {
            const fxv_meta_t *m = &fxv_meta[x[o]->id];
            const uint32_t t42 = ticks_42(x[o]);           /* (each term < 2^31; the sum saturates; a palette: its own costs) */
            const uint32_t fr = x[o]->pal_own ? m->c_frame : m->c_frame_pal, px = x[o]->pal_own ? m->c_px : m->c_px_pal;
            const uint32_t term = fr + px * x[o]->n + t42 * (m->c_tick + m->c_part * (fad[o] ? e->pcount[o] : q[o]));
            cost = cost + term < cost ? 0xFFFFFFFFu : cost + term;
        }
    }
    e->cost = cost;
    return cost <= FXV_BUDGET_CYCLES || tier >= 3u;
}

/* the layout of the tier found: the fading owners' shares of what is left, the ranges in slot order */
static void quota_end(fxv_engine_t *e, uint32_t tier)
{
    uint32_t fad[FXV_SLOTS], q[FXV_SLOTS], o, left = FXV_PARTICLES, base = 0;
    fxv_ctx_t *x[FXV_SLOTS];
    for (o = 0; o < FXV_SLOTS; o++) {
        x[o] = slot_owner(e, o, &fad[o]);
        q[o] = e->ncount[o];
        left -= q[o];
    }
    for (o = 0; o < FXV_SLOTS; o++) {                    /* fading owners: no growth, what is left */
        if (x[o] && fad[o] && fxv_meta[x[o]->id].particles != 0u) {
            q[o] = e->pcount[o] < left ? e->pcount[o] : left;
            left -= q[o];
        }
        if (x[o] && !fad[o]) x[o]->tier = (uint8_t)tier;
    }
    for (o = 0; o < FXV_SLOTS; o++) {
        e->nfirst[o] = (uint16_t)base;
        e->ncount[o] = (uint16_t)q[o];
        base += q[o];
    }
    e->tier = (uint8_t)tier;
}

/* one owner's move to its new range (left movers ascending, then right movers descending: never over a range that is
 * still to move), its new records zeroed, its context pointed at it; FXV_MOVE_STEP records a step (e->mpos: done so
 * far), the same owner again until it is complete (a lone owner's 448 new records in one step cost ~7,300 cycles) */
#define FXV_MOVE_STEP 128u
static uint32_t move_one(fxv_engine_t *e)
{
    uint32_t pass, k;
    for (pass = 0; pass < 2u; pass++) {
        for (k = 0; k < FXV_SLOTS; k++) {
            const uint32_t o = pass == 0u ? k : FXV_SLOTS - 1u - k;
            const uint32_t of = e->pfirst[o], nf = e->nfirst[o];
            uint32_t keep, ncopy, total, m, end, fading;
            fxv_ctx_t *x;
            if (e->moved & (1u << o)) continue;
            if (pass == 0u ? nf > of : nf < of) continue;
            keep = e->pcount[o] < e->ncount[o] ? e->pcount[o] : e->ncount[o];
            ncopy = nf != of ? keep : 0u;
            total = ncopy + e->ncount[o] - keep;
            end = e->mpos + FXV_MOVE_STEP < total ? e->mpos + FXV_MOVE_STEP : total;
            for (m = e->mpos; m < end; m++) {
                if (m < ncopy) {
                    const uint32_t j = nf < of ? m : ncopy - 1u - m;
                    e->slab[nf + j] = e->slab[of + j];
                } else {
                    uint8_t *b = (uint8_t *)&e->slab[nf + keep + (m - ncopy)];
                    uint32_t i;
                    for (i = 0; i < (uint32_t)sizeof(fxv_particle_t); i++) b[i] = 0;
                }
            }
            if (end < total) {
                e->mpos = (uint16_t)end;
                return 1;
            }
            e->mpos = 0;
            e->moved = (uint8_t)(e->moved | (1u << o));
            x = slot_owner(e, o, &fading);
            if (x) {
                x->p = e->ncount[o] ? e->slab + nf : 0;
                x->pn = e->ncount[o];
            }
            return 1;
        }
    }
    for (k = 0; k < FXV_SLOTS; k++) {
        e->pfirst[k] = e->nfirst[k];
        e->pcount[k] = e->ncount[k];
    }
    return 0;
}

/* one slice of a context still preparing its frame: 1 after it, even when that was its last (the next context's first
 * slice waits for the next step: two slices in one step reached ~6,400 cycles, e.g. Ink Bloom's last + Plucked String's) */
static uint32_t ctx_slice(fxv_ctx_t *x)
{
    if (!x->d || fxv_ready(x)) return 0;
    (void)fxv_frame_step(x);
    return 1;
}

uint32_t fxv_engine_step(fxv_engine_t *e)
{
    uint32_t s;
    for (s = 0; s < FXV_MAX_SEG; s++) {                  /* the frame starts fxv_engine_frame_seg left: one a step */
        fxv_seg_t *sg = &e->seg[s];
        if (sg->due) {
            sg->due = 0;
            if (sg->active) fxv_frame_begin(&sg->cur, eff_time(e->now, sg->anchor, &sg->started));
            return 1;
        }
        if (sg->xf >= 0 && (e->xf[sg->xf].used & 4u)) {
            fxv_xfade_t *xf = &e->xf[sg->xf];
            xf->used = (uint8_t)(xf->used & ~4u);
            fxv_frame_begin(&xf->prev, eff_time(e->now, xf->anchor, &xf->started));
            return 1;
        }
    }
    if (e->rebal) {                                      /* the owners changed: new quotas (a tier a step), then the moves */
        e->rebal = 0;
        e->quoting = 1;
        e->qtier = 0;
    }
    if (e->quoting) {
        if (!quota_tier(e, e->qtier)) {
            e->qtier++;
            return 1;
        }
        quota_end(e, e->qtier);
        e->quoting = 0;
        e->moved = 0;
        e->mpos = 0;
        e->moving = 1;
        return 1;
    }
    if (e->moving) {
        if (move_one(e)) return 1;
        e->moving = 0;
    }
    for (s = 0; s < FXV_MAX_SEG; s++) {
        fxv_seg_t *sg = &e->seg[s];
        if (!sg->active) continue;
        if (ctx_slice(&sg->cur)) return 1;
        if (sg->xf >= 0 && ctx_slice(&e->xf[sg->xf].prev)) return 1;
    }
    return 0;
}

void fxv_engine_frame(fxv_engine_t *e, uint32_t now)
{
    uint32_t s;
    for (s = 0; s < FXV_MAX_SEG; s++) {
        while (fxv_engine_step(e)) { }
        (void)fxv_engine_dequeue(e, s, now);
        while (fxv_engine_step(e)) { }
        fxv_engine_frame_seg(e, s, now);
    }
    while (fxv_engine_step(e)) { }
}

uint32_t fxv_engine_dark(const fxv_engine_t *e)
{
    uint32_t s;
    for (s = 0; s < FXV_MAX_SEG; s++) {
        const fxv_seg_t *sg = &e->seg[s];
        if (sg->active && (sg->bri != 0u || sg->tr_inv != 0u || sg->tbri != 0u)) return 0;
    }
    return 1;
}

/* ------------------------------------------------------------------------------- the compositor */
/* the run at strip pixel p (< to): its top segment (0: black) and its end. A later segment covers an earlier one
 * (brightness 0: none), so a run renders one segment, at most two layers a pixel however many segments overlap */
static const fxv_seg_t *run_at(const fxv_engine_t *e, uint32_t p, uint32_t to, uint32_t *end)
{
    const fxv_seg_t *top = 0;
    uint32_t q = to, s;
    for (s = 0; s < FXV_MAX_SEG; s++) {
        const fxv_seg_t *sg = &e->seg[s];
        const uint32_t a = sg->start, b = a + sg->len;
        if (!sg->active || sg->bri == 0u) continue;
        if (a > p) {
            if (a < q) q = a;
        } else if (b > p) {
            if (b < q) q = b;
            top = sg;
        }
    }
    *end = q;
    return top;
}

/* the render + pack step's modeled cycles a pixel besides the layers' c_step (rv32emu, tools/measure_fxv_slices.py):
 * a black pixel (measured 193 / 219 / 235 at 3 / 4 / 5 bytes), the crossfade's blend, the reversed copy (18-22), the
 * segment brightness (35-37) */
#define FXV_STEP_BLACK 280u
#define FXV_STEP_BLEND 120u
#define FXV_STEP_COPY  30u
#define FXV_STEP_SCALE 45u
#define FXV_STEP_MAX   5000u
/* a crossfade step's fixed part: a layer's c_step is a pixel's share of its step at its own step size, fixed part
 * included; a crossfade's steps are 1-3 pixels, where both layers' render calls' fixed parts show (pairs of the effects
 * with the largest fixed parts: up to 1,157 cycles over the pixels' model, 65 -> 27 at RGBCCT 500; + 30 %) */
#define FXV_STEP_XFIX  1500u

/* the next render + pack step from strip pixel `from`: [from, return), within [from, to) and FXV_CHUNK pixels. It
 * stays in one run, on one side of a mirror's middle (one render piece), and takes as many of that run's pixels as fit
 * FXV_STEP_MAX (a crossfade: less its fixed part) at its own cycles a pixel (its layers' c_step plus the extras above);
 * at least one */
static uint32_t step_cost(const fxv_ctx_t *x)           /* a layer's c_step (on a chosen palette: c_step_pal) */
{
    return x->pal_own ? fxv_meta[x->id].c_step : fxv_meta[x->id].c_step_pal;
}

uint32_t fxv_engine_span(const fxv_engine_t *e, uint32_t from, uint32_t to)
{
    uint32_t q, c = FXV_STEP_BLACK, max = FXV_STEP_MAX, k;
    const fxv_seg_t *sg;
    if (to > from + FXV_CHUNK) to = from + FXV_CHUNK;
    sg = run_at(e, from, to, &q);
    if (sg) {
        c = step_cost(&sg->cur);
        if (sg->xf >= 0) {
            c += step_cost(&e->xf[sg->xf].prev) + FXV_STEP_BLEND;
            max -= FXV_STEP_XFIX;
        }
        if (sg->xf >= 0 || (sg->flags & (FX_F_REVERSE | FX_F_MIRROR))) c += FXV_STEP_COPY;
        if (sg->bri != 255u) c += FXV_STEP_SCALE;
        if (sg->flags & FX_F_MIRROR) {
            const uint32_t half = (uint32_t)sg->start + sg->n_eff;
            if (from < half && half < q) q = half;
        }
    }
    k = max / c;
    if (k < 1u) k = 1u;
    return q - from < k ? q : from + k;
}

static void render_piece(const fxv_engine_t *e, const fxv_seg_t *sg, fxv_px_t *dst, uint32_t e0, uint32_t e1, int rev)
{
    fxv_px_t t1[FXV_CHUNK], t2[FXV_CHUNK];
    const fxv_xfade_t *xf = sg->xf >= 0 ? &e->xf[sg->xf] : 0;
    const uint32_t cnt = e1 - e0;
    uint32_t k;
    if (!rev && xf == 0) {
        fxv_render(&sg->cur, dst, e0, e1);
        return;
    }
    fxv_render(&sg->cur, t1, e0, e1);
    if (xf != 0) {
        const uint32_t w = (uint32_t)xf->w * 257u;
        fxv_render(&xf->prev, t2, e0, e1);
        for (k = 0; k < cnt; k++) t1[k] = fxv_blend16(t2[k], t1[k], w);
    }
    for (k = 0; k < cnt; k++) dst[k] = t1[rev ? cnt - 1u - k : k];
}

static void render_segment(const fxv_engine_t *e, const fxv_seg_t *sg, fxv_px_t *out, uint32_t lo, uint32_t hi, uint32_t from)
{
    const uint32_t l0 = lo - sg->start, l1 = hi - sg->start, len = sg->len;
    fxv_px_t *dst = out + (lo - from);
    uint32_t k;
    if (sg->flags & FX_F_MIRROR) {
        const uint32_t half = sg->n_eff, a1 = l1 < half ? l1 : half, b0 = l0 > half ? l0 : half;
        if (l0 < a1) render_piece(e, sg, dst, l0, a1, 0);
        if (b0 < l1) render_piece(e, sg, dst + (b0 - l0), len - l1, len - b0, 1);
    } else if (sg->flags & FX_F_REVERSE) {
        render_piece(e, sg, dst, len - l1, len - l0, 1);
    } else {
        render_piece(e, sg, dst, l0, l1, 0);
    }
    if (sg->bri != 255u) {
        for (k = 0; k < hi - lo; k++) dst[k] = fxv_scale_px(dst[k], (uint32_t)sg->bri * 257u);
    }
}

/* run by run: each renders only its top segment (a receiver step is one run: fxv_engine_span) */
void fxv_engine_render(const fxv_engine_t *e, fxv_px_t *out, uint32_t from, uint32_t to)
{
    uint32_t p = from, q, k;
    while (p < to) {
        const fxv_seg_t *top = run_at(e, p, to, &q);
        if (top) {
            render_segment(e, top, out, p, q, from);
        } else {
            for (k = p - from; k < q - from; k++) {
                out[k].r = out[k].g = out[k].b = 0;
                out[k].q = out[k].k = 0;
            }
        }
        p = q;
    }
}
