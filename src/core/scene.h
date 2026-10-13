/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Power-on scene (README: power-on scene).
 *
 * What the board shows from power-up until valid data arrives: 16-bit outputs for CH1..CH5 and the channels that
 * take part. The default is no scene (dark, as before). Then the controller takes over: its outputs cross-fade from
 * the scene (smoothstep over `fade` x 100 ms), so a controller that comes back in the same state shows no change and
 * a different state fades in instead of jumping.
 *  - Ultra: per light, from the host (SCENE slot, following ESPHome's restore_mode / initial_state); the board
 *    computes the outputs with the light's gamma, min / max power and the power limit.
 *  - WS modes: config frame / console `scene 1` stores the current outputs, `scene 0` clears it.
 *
 * Storage: a log of 16-byte records in SCENE_LOG_PAGES flash pages below the settings page (wear levelling:
 * SCENE_RECS = 128 records per erase cycle of a page, so 10k cycles (datasheet minimum) = 1.28 M writes).
 * Writes: SCENE_SAVE_MS after the last change, and only with a token from a write budget (SCENE_BUDGET_CAP
 * tokens, one more every SCENE_REFILL_MS): bursts of changes are written at once, the long-term average is capped
 * at 96 writes / day (>= 36 years at 10k cycles). A small change (no channel joins / leaves or turns on / off,
 * none moves more than SCENE_MAJOR_STEP) needs a half-full budget, so it never uses up the reserve of the big ones. */
#ifndef SCENE_H
#define SCENE_H

#include <stdint.h>

#define SCENE_REC_BYTES   16u
#define SCENE_PAGE_BYTES  256u
#define SCENE_LOG_PAGES   8u
#define SCENE_RECS_PAGE   (SCENE_PAGE_BYTES / SCENE_REC_BYTES)
#define SCENE_RECS        (SCENE_LOG_PAGES * SCENE_RECS_PAGE)
#define SCENE_MARK        0xA5u
#define SCENE_FADE_DEFAULT 10u        /* 1 s */
#define SCENE_SAVE_MS     5000u       /* quiet time before a write */
#define SCENE_BUDGET_CAP  24u         /* write budget: tokens */
#define SCENE_REFILL_MS   900000u     /* one token per 15 min = 96 writes / day sustained */
#define SCENE_MAJOR_STEP  8192u       /* 12.5 % of full scale */

typedef struct {
    uint16_t v[5];                    /* outputs CH1..CH5 */
    uint8_t  mask;                    /* channels shown at power-up (bit 0 = CH1); 0 = no scene */
    uint8_t  fade;                    /* take-over cross-fade, x 100 ms */
} scene_t;

typedef struct {
    uint16_t seq;
    uint16_t v[5];
    uint8_t  mask, fade;
    uint8_t  crc;                     /* CRC-8/SMBus over the 14 bytes before it */
    uint8_t  mark;                    /* SCENE_MARK; erased flash reads 0xFF or 0x39 / 0xE3 (store_erased) */
} scene_rec_t;

typedef struct {
    uint8_t  tokens;
    uint32_t refill_ms;               /* time of the last refill (moves along while the budget is full) */
} scene_budget_t;

/* running scene: which channels still show it */
typedef struct {
    scene_t  sc;
    uint8_t  hold;                    /* channels showing the scene (no data yet) */
    uint8_t  fading;                  /* channels cross-fading to the controller's values */
    uint32_t t0, dur;                 /* cross-fade start / length, ms */
} scene_run_t;

void    scene_clear(scene_t *s);
int     scene_equal(const scene_t *a, const scene_t *b);
/* a change worth writing soon: a channel joins / leaves, turns on / off, or moves by more than SCENE_MAJOR_STEP */
int     scene_major(const scene_t *a, const scene_t *b);
void    scene_budget_init(scene_budget_t *b, uint32_t now);
/* write now? changed_ms: last change of the pending scene. Takes the token when it returns 1. */
int     scene_save_due(scene_budget_t *b, uint32_t now, uint32_t changed_ms, int major);

/* log in a flash image of SCENE_LOG_PAGES pages: newest valid record (index, or -1) */
int     scene_log_find(const uint8_t *base, scene_t *out, uint16_t *seq);
/* where the record after `newest` goes; *erase = 1 when its page must be erased first */
int     scene_log_next(const uint8_t *base, int newest, uint8_t *erase);
void    scene_log_pack(const scene_t *s, uint16_t seq, scene_rec_t *rec);

void    scene_boot(scene_run_t *r, const scene_t *s);
/* valid data: the channels in `mask` that still hold the scene start their cross-fade at now_ms */
void    scene_takeover(scene_run_t *r, uint8_t mask, uint32_t now_ms);
/* v: the controller's outputs, replaced by the scene / mixed during the cross-fade. Returns 1 while the scene
 * still shows (hold or fade) and the outputs must be refreshed every ms during a fade. */
uint8_t scene_mix(scene_run_t *r, uint32_t now_ms, uint16_t *v);

#endif
