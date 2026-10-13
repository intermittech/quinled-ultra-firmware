/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Input energy: see energy.h. */
#include "energy.h"
#include "lto.h"
#include "ultra_protocol.h"

MEASURED void energy_init(energy_t *e, uint32_t stored_wh, uint32_t now)
{
    uint8_t *b = (uint8_t *)e;
    uint32_t i;
    for (i = 0; i < sizeof(*e); i++) {
        b[i] = 0;
    }
    e->base_wh = e->saved_wh = stored_wh;
    e->saved_ms = e->lit_ms = now;
    budget_init(&e->budget, now, ENERGY_BUDGET_CAP, ENERGY_REFILL_MS);
}

MEASURED void energy_ms(energy_t *e, uint32_t now, uint32_t mv, int32_t ma, uint8_t lit)
{
    if (ma > 0) {
        e->uj += mv * (uint32_t)ma / 1000u;              /* mW x 1 ms = uJ (75 V x 20 A: 1.5e9, fits) */
        if (e->uj >= ENERGY_UJ_PER_MWH) {
            e->uj -= ENERGY_UJ_PER_MWH;
            e->boot_mwh++;
        }
    }
    if (lit) {
        e->lit_ms = now;
        e->lit_new = lit;
    }
}

uint32_t energy_total_wh(const energy_t *e)
{
    return e->base_wh + e->boot_mwh / 1000u;
}

MEASURED uint32_t energy_due(energy_t *e, uint32_t now, uint16_t dips)
{
    uint32_t why = ENERGY_DUE_NONE;
    if (dips != e->dips) {
        e->dips = dips;                                  /* a new supply dip: the power may be going */
        why = ENERGY_DUE_DIP;
    } else if ((now - e->saved_ms >= ENERGY_SAVE_MS || now - e->lit_ms >= ENERGY_DARK_MS) && e->lit_new) {
        why = ENERGY_DUE_ERASE;                          /* lit since the last write: hourly, or dark again */
    }
    if (why == ENERGY_DUE_NONE || energy_total_wh(e) == e->saved_wh || !budget_take(&e->budget, now, 1u)) {
        return ENERGY_DUE_NONE;
    }
    return why;
}

void energy_saved(energy_t *e, uint32_t now, uint32_t now_s)
{
    e->saved_wh = energy_total_wh(e);
    e->saved_ms = now;
    e->saved_s = now_s;
    e->saved = 1;
    e->lit_new = 0;
}

uint32_t energy_log_find(const uint8_t *page)
{
    uint32_t i, best = 0;
    for (i = 0; i < ENERGY_RECS; i++) {
        const uint32_t *r = (const uint32_t *)(page + i * ENERGY_REC_BYTES);
        if (r[1] == ~r[0] && r[0] > best) {
            best = r[0];                                 /* (erased or cut short: never value + complement) */
        }
    }
    return best;
}

void energy_page(const energy_t *e, uint32_t now_s, uint32_t power_mw, uint8_t out[24])
{
    uint32_t i, age = now_s - e->saved_s;
    for (i = 0; i < 24u; i++) {
        out[i] = 0;
    }
    ultra_put32(out + ULTRA_EN_BOOT_MWH, e->boot_mwh);
    ultra_put32(out + ULTRA_EN_TOTAL_WH, energy_total_wh(e));
    ultra_put32(out + ULTRA_EN_POWER_MW, power_mw);
    ultra_put24(out + ULTRA_EN_SAVED_AGE, !e->saved ? 0xFFFFFFu : age > 0xFFFFFEu ? 0xFFFFFEu : age);
}
