/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Input energy and the lifetime record: see energy.h. */
#include <stddef.h>
#include "energy.h"
#include "config.h"                 /* crc16_ccitt */
#include "lto.h"
#include "ultra_protocol.h"
#include "rc.h"                         /* rc_pack */

#define LIT_FAR 0x40000000u         /* lit_ms is kept at most this far back (2^30 ms) */
typedef char life_rec_size[(sizeof(life_rec_t) == LIFE_REC_BYTES && LIFE_RECS * LIFE_REC_BYTES <= 256u) ? 1 : -1];

MEASURED void energy_init(energy_t *e, uint32_t found, uint32_t now, uint32_t ver, uint32_t wd)
{
    uint8_t *b = (uint8_t *)e;
    life_rec_t *r = &e->lt;
    uint32_t i;
    for (i = found ? sizeof(*r) : 0u; i < sizeof(*e); i++) {
        b[i] = 0;
    }
    r->ups++;
    if (wd && r->wd != 0xFFu) {
        r->wd++;
    }
    if (ver != 0u && r->ver != ver) {
        r->updates += (uint16_t)found;                   /* a new board: its first version is no update. Stored with
                                                            the next write; until then each boot of it counts it again,
                                                            in RAM: once */
        r->ver = ver;
    }
    r->magic = LIFE_MAGIC;
    e->ev = 1;
    e->base_wh = e->saved_wh = r->wh;
    e->base_on_s = e->saved_on_s = r->on_s;
    e->lit_ms = now - LIT_FAR;
    budget_init(&e->budget, now, LIFE_BUDGET_CAP, LIFE_REFILL_MS);
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
    return e->base_wh + (e->boot_mwh - e->mwh_seen) / 1000u;   /* (a difference: across boot_mwh's wrap) */
}

/* a lifetime peak in its unit (rounded): v when higher */
static NOINLINE void peak(uint8_t *p, int32_t v, int32_t unit)
{
    if (v > *p * unit + unit / 2) {
        v = (v + unit / 2) / unit;
        *p = (uint8_t)(v > 255 ? 255 : v);
    }
}

MEASURED void energy_second(energy_t *e, uint32_t now, uint32_t now_s, uint32_t trips, int32_t i_ma,
                            int32_t temp_c10)
{
    life_rec_t *r = &e->lt;
    uint32_t d = now - e->lit_ms, k, n = (e->boot_mwh - e->mwh_seen) / 1000u, allow, wh;
    e->base_wh += n;                                     /* whole Wh carried, the remainder kept */
    e->mwh_seen += n * 1000u;
    r->on_s = e->base_on_s + now_s;
    if (d < 1000u) {
        r->lit_s++;                                      /* an output was on in the last second */
    } else if (d > LIT_FAR) {
        e->lit_ms = now - LIT_FAR;
    }
    for (k = 0; k < 2u; k++, trips >>= 16) {             /* overload, short: the new ones since the last second */
        n = (uint16_t)(trips - e->trips_seen[k]);
        e->trips_seen[k] = (uint16_t)trips;
        e->ev |= (uint8_t)(n != 0u);
        n += r->trips[k];
        r->trips[k] = (uint16_t)(n > 0xFFFFu ? 0xFFFFu : n);
    }
    peak(&r->peak_a, i_ma, 100);                         /* mA -> 0.1 A */
    peak(&r->peak_t, temp_c10, 5);                       /* 0.1 C -> 0.5 C (MCUTEMP_NA is never higher) */
    n = r->on_s - e->saved_on_s;                         /* powered since the last write, s */
    wh = energy_total_wh(e) != e->saved_wh;              /* >= 1 Wh new */
    allow = r->writes < LIFE_RESERVE + r->on_s / LIFE_EARN_S;
    e->dip_ok = (uint8_t)(allow && (e->ev || wh || n >= LIFE_DIP_S));
    e->due = (uint8_t)(allow && (e->ev || (wh && e->lit_new) || n >= LIFE_IDLE_S) &&   /* (dark: the logic's own
                                                                                           Wh wait for a day) */
                       ((e->lit_new && d >= ENERGY_DARK_MS) || now_s - e->saved_s >= LIFE_SAVE_S));
}

MEASURED uint32_t energy_due(energy_t *e, uint32_t now, uint16_t dips, uint32_t admit)
{
    uint32_t why = e->due;                               /* (ENERGY_DUE_SAVE or none) */
    if (dips != e->dips) {
        e->lt.dips += (uint16_t)(dips - e->dips);        /* a new supply dip: the power may be going */
        e->dips = dips;
        why = e->dip_ok ? ENERGY_DUE_DIP : ENERGY_DUE_NONE;
    }
    if (!(why & admit)) {
        return ENERGY_DUE_NONE;                          /* (none, or not admitted now: it waits, no token used) */
    }
    if (!budget_take(&e->budget, now, 1u)) {
        e->due = 0;                                      /* (energy_second asks again in a second) */
        return ENERGY_DUE_NONE;
    }
    return why;
}

void energy_pack(energy_t *e)
{
    life_rec_t *r = &e->lt;
    r->wh = energy_total_wh(e);
    do {
        r->writes++;                                     /* (0.6.5: a count skipped when the CRC would read erased) */
    } while (crc16_erased(r->crc = crc16_ccitt((const uint8_t *)r, LIFE_REC_BYTES - 2u)));
}

void energy_saved(energy_t *e, uint32_t now_s)
{
    e->saved_wh = e->lt.wh;
    e->saved_on_s = e->lt.on_s;
    e->saved_s = now_s;
    e->saved = 1;
    e->lit_new = e->ev = e->due = e->dip_ok = 0;
}

/* the record's page pair (core/pagepair.h, 0.6.4: the 0.6.3 code made generic; the same records, positions and
 * decisions) */
static int life_valid(const uint8_t *s)
{
    uint32_t c = ((const life_rec_t *)s)->crc;
    return s[offsetof(life_rec_t, magic)] == LIFE_MAGIC && !crc16_erased(c) && c == crc16_ccitt(s, LIFE_REC_BYTES - 2u);
}

const pp_desc_t life_pp = { LIFE_REC_BYTES, LIFE_RECS, (uint8_t)offsetof(life_rec_t, writes), 0u, life_valid };

uint32_t life_find(const uint8_t *page0, const uint8_t *page1, life_rec_t *r, life_pos_t *pos)
{
    const uint8_t *best = pp_find(&life_pp, page0, page1, pos);
    if (best) {
        *r = *(const life_rec_t *)best;
    }
    return best != 0;
}

uint32_t life_plan(life_pos_t *pos)
{
    return pp_plan(&life_pp, pos);
}

uint32_t life_free(const life_pos_t *pos)
{
    return pp_free(&life_pp, pos);
}

MEASURED void energy_ch_ms(energy_t *e, const uint16_t out[ISENSE_CH], int32_t ma)
{
    uint32_t k;
    for (k = 0; k < ISENSE_CH; k++) {
        e->ch_d[k] += out[k];
    }
    e->ch_i += (uint32_t)ma;                             /* (>= 0) */
    e->ch_n++;
}

MEASURED void energy_ch_second(energy_t *e, const isense_t *is, uint32_t mv)
{
    uint32_t k, n = e->ch_n, f[ISENSE_CH], sum = 0, mj;
    int32_t i;
    if (n == 0u) {
        return;
    }
    i = (int32_t)(e->ch_i / n) - (is->zero_valid ? is->zero_ma : 0);
    mj = i > 0 ? mv * (uint32_t)i / 1000u * n / 1000u : 0u;     /* mV x mA = uW (75 V x 20 A fits); x n ms: mJ */
    for (k = 0; k < ISENSE_CH; k++) {
        /* the modelled current at the mean output, mA (< 2^15); the mean of 16-bit levels as 16 bits: GCC then still
         * knows every isense_ch_ma caller's level fits (shape_at keeps no range check: isense_ms 20 cycles) */
        f[k] = isense_ch_ma(is, k, (uint16_t)(e->ch_d[k] / n));
        sum += f[k];
        e->ch_d[k] = 0;
    }
    for (k = 0; sum != 0u && k < ISENSE_CH; k++) {
        /* its share, Q16 (<= 65536), of the second's mJ (< 2^21): a 64-bit product, no division by sum per mJ */
        uint32_t m = e->ch_mj[k] + (uint32_t)(((uint64_t)mj * ((f[k] << 16) / sum)) >> 16);
        e->ch_mwh[k] += m / 3600u;
        e->ch_mj[k] = (uint16_t)(m % 3600u);
    }
    e->ch_i = e->ch_n = 0;
}

void energy_page(const energy_t *e, uint32_t now_s, uint32_t power_mw, uint8_t out[24])
{
    uint32_t i, age = now_s - e->saved_s;
    for (i = 0; i < 24u; i++) {
        out[i] = 0;
    }
    {                                    /* ULTRA_EN_BOOT_MWH, _TOTAL_WH, _POWER_MW, _SAVED_AGE */
        static const uint8_t len[4] = { 4, 4, 4, 3 };
        uint32_t v[4];
        v[0] = e->boot_mwh;
        v[1] = energy_total_wh(e);
        v[2] = power_mw;
        v[3] = !e->saved ? 0xFFFFFFu : age > 0xFFFFFEu ? 0xFFFFFEu : age;
        rc_pack(out, v, len, 4);
    }
}
