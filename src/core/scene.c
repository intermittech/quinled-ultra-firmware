/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include <string.h>
#include "scene.h"
#include "store.h"                   /* (config.h: crc8_smbus) */
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
    return crc8_smbus((const uint8_t *)r, 14u);       /* (config.h: the same CRC-8, one copy in the image) */
}

int scene_rec_valid(const uint8_t *b)
{
    const scene_rec_t *r = (const scene_rec_t *)b;
    return r->mark == SCENE_MARK && r->crc == rec_crc(r) && (r->s.mask & ~0x1Fu) == 0u;
}

/* 16 records a page, the u16 seq at offset 0 (0.6.4; before: a ring of its own, the same records) */
typedef char scene_pair[(SCENE_LOG_PAGES == 2u && SCENE_RECS_PAGE <= 16u && SCENE_REC_BYTES == 16u) ? 1 : -1];
const pp_desc_t scene_pp = { SCENE_REC_BYTES, SCENE_RECS_PAGE, 0u, 16u, scene_rec_valid };

void scene_log_pack(const scene_t *s, uint16_t seq, scene_rec_t *rec)
{
    rec->seq = seq;
    rec->s = *s;
    rec->mark = SCENE_MARK;
    rec->crc = rec_crc(rec);
}

/* a head as this firmware writes it (ultra_apply_slot validated every field): what the engine indexes with is in
 * range. The CRC-8 alone lets 1 in 256 of any other content through (another layout under the same magic, a damaged
 * page): roles 8-255 would be output, a mask with bits 5-7 would keep the power-on effect running after the host's
 * take-over, set points past the 4 kept would be read from the fields after them. */
static int head_ok(const ultra_light_t *L)
{
    uint32_t i, mask = 0;
    for (i = 0; i < 5u; i++) {
        if (L->role[i] > ULTRA_ROLE_WARM_WHITE) {
            return 0;
        }
        if (L->role[i] != ULTRA_ROLE_NONE) {
            mask |= 1u << i;
        }
    }
    if (L->opt_n > ULTRA_OPT_POINTS_MAX) {
        return 0;
    }
    for (i = 0; i < L->opt_n; i++) {                     /* as opt_points leaves them: valid, sorted by M, descending,
                                                            none warmer than the warm end (dim_to_warm divides by
                                                            warm end - WW LED when a point lies past the WW LED) */
        if (L->opt_pm[i] == 0u || L->opt_pt[i] == 0u || L->opt_pt[i] > L->opt_warm ||
            (i > 0u && L->opt_pm[i - 1u] < L->opt_pm[i])) {
            return 0;
        }
    }
    return L->mask == mask && (L->flags & ~ULTRA_DEF_FLAGS) == 0u && (L->opt_flags & ~ULTRA_OPT_FLAGS) == 0u &&
           L->scene_mode <= 1u && L->scene_fx < ULTRA_FX_COUNT && L->g_q16 <= (65535u * 65536u + 500u) / 1000u;
}

/* the power-on effect store's image (core/ultra.h ULTRA_FX_*, written by hal/hw.c): the valid lights' heads */
uint8_t ultra_fx_rejected;

uint8_t ultra_fx_load(ultra_t *u, const uint8_t *img, uint8_t crc[ULTRA_LIGHTS])
{
    uint8_t mask = 0;
    uint32_t i;
    ultra_fx_rejected = 0;
    if ((img[0] | ((uint32_t)img[1] << 8) | ((uint32_t)img[2] << 16) | ((uint32_t)img[3] << 24)) != ULTRA_FX_MAGIC) {
        return 0;                                        /* (little endian, as hal/hw.c writes the word) */
    }
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        const uint8_t *h = img + ULTRA_FX_HDR + i * ULTRA_HEAD_BYTES;
        if ((img[4] & (1u << i)) && crc8_smbus(h, ULTRA_HEAD_BYTES) == img[5u + i]) {
            uint32_t g0 = u->light[i].g_q16;             /* (the init's value: the head holds g_q16) */
            memcpy(&u->light[i], h, ULTRA_HEAD_BYTES);
            u->light[i].out_ok = 0;                      /* (a new head: the render cache) */
            if (!head_ok(&u->light[i])) {
                memset(&u->light[i], 0, ULTRA_HEAD_BYTES);   /* not this firmware's: as if erased */
                u->light[i].g_q16 = g0;
                ultra_fx_rejected += MON_EXTRA;          /* (0.6.5, D5) */
                continue;
            }
            crc[i] = img[5u + i];
            mask |= (uint8_t)(1u << i);
        } else if (MON_EXTRA && (img[4] & (1u << i))) {
            ultra_fx_rejected++;                         /* (0.6.5, D5) listed, its CRC wrong */
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
