/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Stored analog power-on and signal-loss levels, with fades into each. */
#ifndef SCENE_H
#define SCENE_H

#include <stdint.h>
#include "pagepair.h"

#define SCENE_REC_BYTES   32u
#define SCENE_PAGE_BYTES  256u
#define SCENE_LOG_PAGES   2u          /* A/B pair; never erase the newest valid record */
#define SCENE_RECS_PAGE   (SCENE_PAGE_BYTES / SCENE_REC_BYTES)
#define SCENE_RECS        (SCENE_LOG_PAGES * SCENE_RECS_PAGE)
#define SCENE_MARK        0xA5u
#define SCENE_FADE_DEFAULT 10u        /* 1 s */
#define SCENE_SAVE_MS     5000u       /* quiet time before a write */
#define SCENE_BUDGET_CAP  24u         /* write budget: tokens */
#define SCENE_REFILL_MS   2400000u    /* one per 40 min: 36 writes/day, preserving the erase wear budget */
#define SCENE_MAJOR_STEP  8192u       /* 12.5 % of full scale */

typedef struct {
    uint16_t v[5];                    /* outputs CH1..CH5 */
    uint8_t  mask;                    /* channels shown at power-up (bit 0 = CH1); 0 = no scene */
    uint8_t fade;                     /* fade into power-on levels, x 100 ms */
    uint16_t loss_v[5];
    uint8_t loss_mask, loss_fade;
} scene_t;

typedef struct {
    uint16_t seq;
    uint8_t format, channels;
    scene_t  s;                       /* both level sets and their fades */
    uint16_t salt;
    uint8_t  crc;                     /* CRC-8/SMBus over the preceding 30 bytes */
    uint8_t  mark;                    /* SCENE_MARK; erased flash reads 0xFF or 0x39 / 0xE3 (store_erased) */
} scene_rec_t;

typedef struct {                      /* a flash write budget: cap tokens, one more every period_ms */
    uint8_t  tokens, cap;
    uint32_t refill_ms;               /* time of the last refill (moves along while the budget is full) */
    uint32_t period_ms;
} scene_budget_t;

/* running scene: which channels still show it */
typedef struct {
    scene_t  sc;
    uint8_t  hold;                    /* channels showing the scene (no data yet) */
    uint8_t  fading;                  /* channels cross-fading to the controller's values */
    uint32_t dur[5];                  /* SET transition length per channel, ms */
    uint32_t t0[5];                   /* each channel's transition start, ms */
    uint8_t smooth;
    uint8_t via_off;
    uint8_t  in;                      /* channels still fading into stored levels */
    uint16_t from[5];
    uint32_t t_in;
} scene_run_t;
#define SCENE_IN_MS 1000u             /* WS takeover, where no SET transition is available */

void    scene_clear(scene_t *s);
int     scene_equal(const scene_t *a, const scene_t *b);
/* a change worth writing soon: a channel joins / leaves, turns on / off, or moves by more than SCENE_MAJOR_STEP */
int     scene_major(const scene_t *a, const scene_t *b);
void    scene_budget_init(scene_budget_t *b, uint32_t now);  /* the scene's: SCENE_BUDGET_CAP, SCENE_REFILL_MS */
void    budget_init(scene_budget_t *b, uint32_t now, uint8_t cap, uint32_t period_ms);
/* refill, then take a token if at least `need` are there; 1 = taken */
int     budget_take(scene_budget_t *b, uint32_t now, uint32_t need);
/* the tokens there now (refilled; none taken) */
uint32_t budget_avail(scene_budget_t *b, uint32_t now);
/* write now? changed_ms: last change of the pending scene. Takes the token when it returns 1. */
int     scene_save_due(scene_budget_t *b, uint32_t now, uint32_t changed_ms, int major);

/* the log's page pair (core/pagepair.h): a record valid (mark, CRC-8, mask), the pair's description */
int     scene_rec_valid(const uint8_t *r);
extern const pp_desc_t scene_pp;
void    scene_log_pack(const scene_t *s, uint16_t seq, scene_rec_t *rec);

void    scene_boot(scene_run_t *r, const scene_t *s);
/* Signal loss fades from live outputs into loss_v over loss_fade x 100 ms.
 * Unset channels are dark. Valid data takes over using the SET transition. */
void    scene_return(scene_run_t *r, const scene_t *s, const uint16_t *from, uint32_t now_ms);
/* valid data: the channels in `mask` that still hold the scene start their cross-fade at now_ms (each its own; a
 * time still to come: the scene shows until then) */
void    scene_takeover(scene_run_t *r, uint8_t mask, uint32_t now_ms, uint32_t dur_ms, uint8_t curve_flags);
/* v: the controller's outputs, replaced by the scene / mixed during the cross-fade. Returns 1 while the scene
 * still shows (hold or fade) and the outputs must be refreshed every ms during a fade. */
uint8_t scene_mix(scene_run_t *r, uint32_t now_ms, uint16_t *v);

#endif
