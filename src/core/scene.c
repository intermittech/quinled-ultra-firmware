/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include <string.h>
#include "scene.h"
#include "store.h"                   /* (config.h: crc8_smbus) */
#include "../board/boards.h"
#include "ultra.h"                   /* ultra_smoothstep */

void scene_clear(scene_t *s)
{
    memset(s, 0, sizeof(*s));
    s->mask = 0;
    s->fade = s->loss_fade = SCENE_FADE_DEFAULT;
}

int scene_equal(const scene_t *a, const scene_t *b)
{
    uint32_t i;
    if (a->mask != b->mask || a->fade != b->fade || a->loss_mask != b->loss_mask || a->loss_fade != b->loss_fade) {
        return 0;
    }
    for (i = 0; i < 5u; i++) {
        if (((a->mask & (1u << i)) && a->v[i] != b->v[i]) || ((a->loss_mask & (1u << i)) && a->loss_v[i] != b->loss_v[i])) {
            return 0;
        }
    }
    return 1;
}

int scene_major(const scene_t *a, const scene_t *b)
{
    uint32_t i;
    if (a->mask != b->mask || a->loss_mask != b->loss_mask) {
        return 1;
    }
    for (i = 0; i < 5u; i++) {
        int32_t d = (int32_t)a->v[i] - (int32_t)b->v[i];
        int32_t loss = (int32_t)a->loss_v[i] - (int32_t)b->loss_v[i];
        if ((a->loss_mask & (1u << i)) && ((a->loss_v[i] == 0u) != (b->loss_v[i] == 0u) || loss > (int32_t)SCENE_MAJOR_STEP || loss < -(int32_t)SCENE_MAJOR_STEP)) return 1;
        if (!(a->mask & (1u << i))) {
            continue;
        }
        if ((a->v[i] == 0u) != (b->v[i] == 0u) || d > (int32_t)SCENE_MAJOR_STEP || d < -(int32_t)SCENE_MAJOR_STEP) {
            return 1;
        }
    }
    return 0;
}

void budget_init(scene_budget_t *b, uint32_t now, uint8_t cap, uint32_t period_ms)
{
    b->tokens = b->cap = cap;
    b->refill_ms = now;
    b->period_ms = period_ms;
}

void scene_budget_init(scene_budget_t *b, uint32_t now)
{
    budget_init(b, now, SCENE_BUDGET_CAP, SCENE_REFILL_MS);
}

uint32_t budget_avail(scene_budget_t *b, uint32_t now)
{
    while (now - b->refill_ms >= b->period_ms) {
        b->refill_ms += b->period_ms;
        if (b->tokens < b->cap) {
            b->tokens++;
        }
    }
    if (b->tokens >= b->cap) {
        b->refill_ms = now;                              /* full: the next token comes a whole period after a write */
    }
    return b->tokens;
}

int budget_take(scene_budget_t *b, uint32_t now, uint32_t need)
{
    if (budget_avail(b, now) < need) {
        return 0;
    }
    b->tokens--;
    return 1;
}

int scene_save_due(scene_budget_t *b, uint32_t now, uint32_t changed_ms, int major)
{
    return now - changed_ms >= SCENE_SAVE_MS && budget_take(b, now, major ? 1u : SCENE_BUDGET_CAP / 2u);
}

/* ----------------------------------------------------------------------------------------------- log */
static uint8_t rec_crc(const scene_rec_t *r)
{
    return crc8_smbus((const uint8_t *)r, SCENE_REC_BYTES - 2u);       /* (config.h: the same CRC-8, one copy in the image) */
}

int scene_rec_valid(const uint8_t *b)
{
    const scene_rec_t *r = (const scene_rec_t *)b;
    return r->mark == SCENE_MARK && r->format == 4u && r->channels == BOARD_CHANNELS && r->crc == rec_crc(r) && ((r->s.mask | r->s.loss_mask) & ~0x1Fu) == 0u;
}

/* 16 records a page, the u16 seq at offset 0 (before: a ring of its own, the same records) */
typedef char scene_pair[(SCENE_LOG_PAGES == 2u && SCENE_RECS_PAGE <= 16u && sizeof(scene_rec_t) == SCENE_REC_BYTES) ? 1 : -1];
const pp_desc_t scene_pp = { 256u, SCENE_REC_BYTES, SCENE_RECS_PAGE, 0u, 16u, scene_rec_valid };

void scene_log_pack(const scene_t *s, uint16_t seq, scene_rec_t *rec)
{
    rec->seq = seq;
    rec->format = 4u; rec->channels = BOARD_CHANNELS; rec->salt = 0;
    rec->s = *s;
    rec->mark = SCENE_MARK;
    rec->crc = rec_crc(rec);
}

/* Runtime levels are physical PWM outputs. SET supplies the takeover duration. */
static void begin(scene_run_t *r, const scene_t *s, const uint16_t *from, uint32_t now)
{
    uint32_t i;
    memset(r, 0, sizeof(*r));
    r->sc = *s; r->hold = s->mask & 0x1Fu; r->in = r->hold;
    r->t_in = now;
    for (i=0; i<5u; ++i) r->from[i] = from ? from[i] : 0u;
}
void scene_boot(scene_run_t *r, const scene_t *s) { begin(r, s, 0, 0); }
void scene_return(scene_run_t *r, const scene_t *s, const uint16_t *from, uint32_t now)
{
    scene_t loss = *s;
    {
        uint32_t i;
        for (i = 0; i < 5u; ++i)
            loss.v[i] = (s->loss_mask & (1u << i)) ? s->loss_v[i] : 0u;
    }
    /* An unset loss scene is dark, including channels outside any light. */
    loss.mask = 0x1Fu; loss.fade = s->loss_fade;
    begin(r, &loss, from, now);
}
static uint16_t scene_in(scene_run_t *r, uint32_t i, uint32_t now)
{
    uint32_t el = now-r->t_in, dur = (uint32_t)r->sc.fade*100u;
    if (!dur || el >= dur) { r->in &= (uint8_t)~(1u<<i); return r->sc.v[i]; }
    return ultra_lerp16(r->from[i], r->sc.v[i], ultra_smoothstep((el<<16)/dur));
}
void scene_takeover(scene_run_t *r, uint8_t mask, uint32_t now, uint32_t dur, uint8_t curve_flags)
{
    uint32_t i; uint8_t m = r->hold & mask;
    for (i=0; i<5u; ++i) if (m & (1u<<i)) {
        r->t0[i]=now; r->dur[i]=dur;
    }
    r->smooth = (uint8_t)((r->smooth & ~m) | ((curve_flags & 1u) ? m : 0u));
    r->via_off = (uint8_t)((r->via_off & ~m) | ((curve_flags & 2u) ? m : 0u));
    r->hold &= (uint8_t)~m; r->fading |= m;
}
uint8_t scene_mix(scene_run_t *r, uint32_t now, uint16_t *v)
{
    uint32_t i;
    for (i=0; i<5u; ++i) {
        uint32_t bit=1u<<i;
        if (r->hold & bit) v[i]=(r->in & bit) ? scene_in(r,i,now) : r->sc.v[i];
        else if (r->fading & bit) {
            int32_t el=(int32_t)(now-r->t0[i]);
            if (el<0) v[i]=(r->in & bit) ? scene_in(r,i,now) : r->sc.v[i];
            else {
                if (r->in & bit) {
                    r->sc.v[i]=scene_in(r,i,r->t0[i]);
                    r->in &= (uint8_t)~bit;
                }
                if (!r->dur[i] || (uint32_t)el>=r->dur[i]) r->fading &= (uint8_t)~bit;
                else {
                    uint32_t e=(uint32_t)el,d=r->dur[i],p,from=r->sc.v[i],to=v[i];
                    while (d>=65536u) {e>>=1;d>>=1;}
                    p=(e<<16)/d;
                    if (r->via_off & bit) {
                        if (p<32768u) {p*=2u;to=0;}
                        else {p=p*2u-65536u;from=0;}
                    }
                    v[i]=ultra_lerp16(from,to,(r->smooth&bit)?ultra_smoothstep(p):p);
                }
            }
        }
    }
    return (r->fading | r->in)!=0u;
}
