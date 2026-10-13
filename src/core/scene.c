/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include <string.h>
#include "scene.h"
#include "store.h"                   /* store_erased */
#include "ultra.h"                   /* ultra_smoothstep */

void scene_clear(scene_t *s)
{
    uint32_t i;
    for (i = 0; i < 5u; i++) {
        s->v[i] = 0;
    }
    s->mask = 0;
    s->fade = SCENE_FADE_DEFAULT;
}

int scene_equal(const scene_t *a, const scene_t *b)
{
    uint32_t i;
    if (a->mask != b->mask || a->fade != b->fade) {
        return 0;
    }
    for (i = 0; i < 5u; i++) {
        if ((a->mask & (1u << i)) && a->v[i] != b->v[i]) {
            return 0;
        }
    }
    return 1;
}

int scene_major(const scene_t *a, const scene_t *b)
{
    uint32_t i;
    if (a->mask != b->mask) {
        return 1;
    }
    for (i = 0; i < 5u; i++) {
        int32_t d = (int32_t)a->v[i] - (int32_t)b->v[i];
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

int budget_take(scene_budget_t *b, uint32_t now, uint32_t need)
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
    if (b->tokens < need) {
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
    return crc8_smbus((const uint8_t *)r, 14u);       /* (config.h: the same CRC-8, one copy in the image) */
}

static int rec_valid(const scene_rec_t *r)
{
    return r->mark == SCENE_MARK && r->crc == rec_crc(r) && (r->mask & ~0x1Fu) == 0u;
}

static int rec_blank(const uint8_t *b)
{
    return store_erased(b, SCENE_REC_BYTES);
}

void scene_log_pack(const scene_t *s, uint16_t seq, scene_rec_t *rec)
{
    uint32_t i;
    rec->seq = seq;
    for (i = 0; i < 5u; i++) {
        rec->v[i] = s->v[i];
    }
    rec->mask = s->mask;
    rec->fade = s->fade;
    rec->mark = SCENE_MARK;
    rec->crc = rec_crc(rec);
}

/* newest = the valid record whose seq is ahead of every other (16-bit serial numbers: at most SCENE_RECS live) */
int scene_log_find(const uint8_t *base, scene_t *out, uint16_t *seq)
{
    int best = -1;
    uint16_t best_seq = 0;
    uint32_t i, k;
    for (i = 0; i < SCENE_RECS; i++) {
        const scene_rec_t *r = (const scene_rec_t *)(base + i * SCENE_REC_BYTES);
        if (rec_valid(r) && (best < 0 || (int16_t)(uint16_t)(r->seq - best_seq) > 0)) {
            best = (int)i;
            best_seq = r->seq;
        }
    }
    if (best >= 0) {
        const scene_rec_t *r = (const scene_rec_t *)(base + (uint32_t)best * SCENE_REC_BYTES);
        for (k = 0; k < 5u; k++) {
            out->v[k] = r->v[k];
        }
        out->mask = r->mask;
        out->fade = r->fade;
        *seq = best_seq;
    }
    return best;
}

int scene_log_next(const uint8_t *base, int newest, uint8_t *erase)
{
    uint32_t i = newest < 0 ? 0u : ((uint32_t)newest + 1u) % SCENE_RECS;
    *erase = 0;
    if (newest < 0) {                                    /* nothing stored: start at page 0 */
        *erase = (uint8_t)!rec_blank(base);
        return 0;
    }
    /* the rest of the current page, if blank (a record lost mid-write leaves a used slot: skip it) */
    while (i % SCENE_RECS_PAGE != 0u) {
        if (rec_blank(base + i * SCENE_REC_BYTES)) {
            return (int)i;
        }
        i = (i + 1u) % SCENE_RECS;
    }
    *erase = 1;                                          /* a page boundary: the next page holds the oldest records */
    return (int)i;
}

/* the power-on effect store's image (core/ultra.h ULTRA_FX_*, written by hal/hw.c): the valid lights' heads */
uint8_t ultra_fx_load(ultra_t *u, const uint8_t *img, uint8_t crc[ULTRA_LIGHTS])
{
    uint8_t mask = 0;
    uint32_t i;
    if ((img[0] | ((uint32_t)img[1] << 8) | ((uint32_t)img[2] << 16) | ((uint32_t)img[3] << 24)) != ULTRA_FX_MAGIC) {
        return 0;                                        /* (little endian, as hal/hw.c writes the word) */
    }
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        const uint8_t *h = img + ULTRA_FX_HDR + i * ULTRA_HEAD_BYTES;
        if ((img[4] & (1u << i)) && crc8_smbus(h, ULTRA_HEAD_BYTES) == img[5u + i]) {
            memcpy(&u->light[i], h, ULTRA_HEAD_BYTES);
            crc[i] = img[5u + i];
            mask |= (uint8_t)(1u << i);
        }
    }
    return mask;
}

/* ----------------------------------------------------------------------------------------------- runtime */
void scene_boot(scene_run_t *r, const scene_t *s)
{
    r->sc = *s;
    r->hold = (uint8_t)(s->mask & 0x1Fu);
    r->fading = 0;
    r->t0 = 0;
    r->dur = 0;
}

void scene_takeover(scene_run_t *r, uint8_t mask, uint32_t now_ms)
{
    uint8_t m = (uint8_t)(r->hold & mask);
    if (m == 0u) {
        return;
    }
    if (r->fading == 0u) {
        r->t0 = now_ms;
        r->dur = (uint32_t)r->sc.fade * 100u;
    }
    r->hold = (uint8_t)(r->hold & ~m);
    r->fading = (uint8_t)(r->fading | m);
}

uint8_t scene_mix(scene_run_t *r, uint32_t now_ms, uint16_t *v)
{
    uint32_t i, s = 65536u, el;
    if ((r->hold | r->fading) == 0u) {
        return 0;
    }
    if (r->fading) {
        el = now_ms - r->t0;
        if (r->dur == 0u || el >= r->dur) {
            r->fading = 0;
        } else {
            s = ultra_smoothstep((el << 16) / r->dur);  /* dur <= 25.5 s: el << 16 fits */
        }
    }
    for (i = 0; i < 5u; i++) {
        uint32_t bit = 1u << i, sv = (r->sc.mask & bit) ? r->sc.v[i] : 0u;
        if (r->hold & bit) {
            v[i] = (uint16_t)sv;
        } else if (r->fading & bit) {
            v[i] = (uint16_t)((int32_t)sv + (int32_t)((((int64_t)v[i] - (int64_t)sv) * (int64_t)s) >> 16));
        }
    }
    return r->fading != 0u;
}
