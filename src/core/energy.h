/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Input energy (0.6.0): the supply reading (core/vsense.h) x the measured board current (core/isense.h), every ms.
 * Monitoring only: nothing here changes the outputs.
 *  - since boot, in mWh (RAM);
 *  - the board's lifetime, in Wh: the value stored in flash at boot plus what came since. Stored in its own page
 *    (ENERGY_ADDR, hal/board.h): a log of 8-byte entries (the value and its complement, so an entry cut short by a
 *    power loss is never read as valid; the highest valid value is the newest), ENERGY_RECS per erase.
 * When it is stored (only with >= 1 Wh not stored yet, and only with a token from a write budget: ENERGY_BUDGET_CAP
 * tokens, one more every ENERGY_REFILL_MS):
 *  - once an hour of light (ENERGY_SAVE_MS since the last write);
 *  - when the outputs have been dark for ENERGY_DARK_MS (a controller switched the light off);
 *  - at the start of a supply dip (core/vdip.h): the supply may be going away (a wall switch). Only into an erased
 *    entry (no page erase then): if the board loses power during the write, the entry is invalid and the one before
 *    stays.
 * Wear: at most 24 writes a day sustained (the budget), 32 per erase: >= 36 years at 10k cycles (datasheet minimum).
 * What isn't stored when the power goes is lost (below 1 Wh, or a light that never went dark / dipped in time).
 * A factory reset erases the page. Pure logic, host-tested (test/test_energy.c). */
#ifndef ENERGY_H
#define ENERGY_H

#include <stdint.h>
#include "scene.h"                  /* the write budget (scene_budget_t) */
#include "store.h"

#define ENERGY_REC_BYTES   8u
#define ENERGY_RECS        (256u / ENERGY_REC_BYTES)
#define ENERGY_SAVE_MS     3600000u
#define ENERGY_DARK_MS     30000u
#define ENERGY_BUDGET_CAP  4u
#define ENERGY_REFILL_MS   3600000u
#define ENERGY_UJ_PER_MWH  3600000u

/* why energy_due() wants a write */
#define ENERGY_DUE_NONE    0u
#define ENERGY_DUE_ERASE   1u       /* a regular write (the page may be erased first) */
#define ENERGY_DUE_DIP     2u       /* a supply dip: only into an erased entry */

typedef struct {
    uint32_t uj;                    /* below one mWh, uJ */
    uint32_t boot_mwh;              /* since boot */
    uint32_t base_wh;               /* the lifetime value stored at boot */
    uint32_t saved_wh;              /* the last value stored */
    uint32_t saved_ms;              /* when (saved = 1) */
    uint32_t saved_s;               /* ... as uptime in s (the page's age) */
    uint32_t lit_ms;                /* the last ms with an output on */
    uint8_t  saved;                 /* stored since boot */
    uint8_t  lit_new;               /* an output was on since the last write (a flag: no time compare that can age) */
    uint16_t dips;                  /* the supply dips seen */
    scene_budget_t budget;
} energy_t;

void     energy_init(energy_t *e, uint32_t stored_wh, uint32_t now);
/* every ms: the supply (mV), the board current (mA) and whether an output is on */
void     energy_ms(energy_t *e, uint32_t now, uint32_t mv, int32_t ma, uint8_t lit);
/* the lifetime value now, Wh */
uint32_t energy_total_wh(const energy_t *e);
/* every ms: a write due now (ENERGY_DUE_*)? dips = the supply dips since boot (core/vdip.h). Takes a budget token
 * when it says yes; call energy_saved() after the write. */
uint32_t energy_due(energy_t *e, uint32_t now, uint16_t dips);
void     energy_saved(energy_t *e, uint32_t now, uint32_t now_s);
/* the log in a 256-byte page: the newest valid value (0 when none); the next erased entry (ENERGY_RECS = full) */
uint32_t energy_log_find(const uint8_t *page);
#define energy_log_next(page) store_first_erased((page), ENERGY_REC_BYTES, ENERGY_RECS)
/* ENERGY page DATA (protocol 0.2.3, ULTRA_EN_*); now_s = the uptime in s */
void     energy_page(const energy_t *e, uint32_t now_s, uint32_t power_mw, uint8_t out[24]);

#endif
