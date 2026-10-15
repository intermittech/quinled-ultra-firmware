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

/* the lifetime allowance: a write left (the board's own powered time only: 0.6.10) */
static NOINLINE uint32_t life_allow(const life_rec_t *r)
{
    return r->writes < LIFE_RESERVE + (r->on_s - LIFE_MERGED(r) * 360u) / LIFE_EARN_S;
}

MEASURED void energy_init(energy_t *e, uint32_t found, uint32_t now, uint32_t ver, uint32_t wd)
{
    uint8_t *b = (uint8_t *)e;
    life_rec_t *r = &e->lt;
    uint32_t i;
    for (i = found ? sizeof(*r) : 0u; i < sizeof(*e); i++) {
        b[i] = 0;
    }
    r->ups++;
    e->ev = (uint8_t)(2u | (wd != 0u));                  /* the power-up (bit 1), a watchdog reset (bit 0) */
    e->ev_save = 3u;                                     /* (main.c: the receiver 1) */
    if (wd && r->wd != 0xFFu) {
        r->wd++;
    }
    if (r->magic == LIFE_MAGIC_F2) {                     /* (0.6.10) format 2: bytes 20..23 the version word */
        r->ver = crc8_smbus(&r->ver, 4u);                /* (energy_ver of the word in place) */
        r->merged_hi = 0;
        r->merged = 0;                                   /* (all its powered time its own) */
    }
    i = ver != 0u ? energy_ver(ver) : r->ver;
    if (r->ver != i) {
        r->updates += (uint16_t)found;                   /* a new board: its first version is no update. Stored with
                                                            the next write; until then each boot of it counts it again,
                                                            in RAM: once */
        r->ver = (uint8_t)i;
        e->ev |= (uint8_t)found;                         /* (an update: worth a planned power-off's write) */
    }
    r->magic = LIFE_MAGIC;
    e->base_wh = e->saved_wh = r->wh;
    e->base_on_s = e->saved_on_s = r->on_s;
    e->dip_ok = (uint8_t)life_allow(r);
                                                         /* (round 9 phase B) energy_second's first: a dip in the
                                                            first second stores the power-up too (the allowance as
                                                            ever: a flicker of short cycles writes no more than cycles
                                                            of a few seconds already could) */
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
    uint32_t w = e->base_wh + (e->boot_mwh - e->mwh_seen) / 1000u;   /* (a difference: across boot_mwh's wrap) */
    return w < e->base_wh ? 0xFFFFFFFFu : w;             /* (round 9 phase B: at the ceiling it stays, never wraps) */
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
    wh = e->base_wh + n;
    e->base_wh = wh < n ? 0xFFFFFFFFu : wh;              /* whole Wh carried, the remainder kept */
    e->mwh_seen += n * 1000u;
    k = e->base_on_s + now_s;
    r->on_s = k < now_s ? 0xFFFFFFFFu : k;               /* (round 9 phase B) the counters' ceiling, 2^32 - 1 (s: ~136
                                                            years; a hub's merges can bring it near): they stay there,
                                                            never wrap. The own time then stops counting (the allowance
                                                            with it); `merged` never grows past it (energy_sync) */
    if (d < 1000u) {
        r->lit_s += r->lit_s != 0xFFFFFFFFu;             /* an output was on in the last second */
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
    allow = life_allow(r);
    e->held = (uint8_t)!allow;
    k = wh || n >= LIFE_DIP_S;                           /* (a dip's own: something to store) */
    e->dip_ok = (uint8_t)(allow && (e->ev || k)) | (uint8_t)(allow && ((e->ev & 1u) || k)) << 1;
    e->due = (uint8_t)(allow && ((e->ev & 4u) ||      /* (round 8) a dip's write still pending: no dark wait */
                                 (((e->ev & e->ev_save) || (wh && e->lit_new) || n >= LIFE_IDLE_S) &&   /* (dark: the
                                                                         logic's own Wh wait for a day) */
                                  ((e->lit_new && d >= ENERGY_DARK_MS) || now_s - e->saved_s >= LIFE_SAVE_S))));
}

MEASURED uint32_t energy_due(energy_t *e, uint32_t now, uint16_t dips, uint32_t admit)
{
    uint32_t why = e->due;                               /* (ENERGY_DUE_SAVE or none) */
    if (dips != e->dips || (admit & ENERGY_PLANNED)) {
        e->lt.dips += (uint16_t)(dips - e->dips);        /* a new supply dip: the power may be going (the planned
                                                            cut: none counted) */
        e->dips = dips;
        why = (e->dip_ok & (admit & ENERGY_PLANNED ? 2u : 1u)) ? ENERGY_DUE_DIP : ENERGY_DUE_NONE;
        e->ev |= (uint8_t)(why << 1);                    /* (round 8) bit 2: pending until stored (energy_saved) */
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
    r->writes++;
    while (crc16_erased(r->crc = crc16_ccitt((const uint8_t *)r, LIFE_REC_BYTES - 2u))) {
        r->lit_s += r->lit_s != 0xFFFFFFFFu ? 1u : 0xFFFFFFFFu;   /* (a CRC that would read erased: one more lit
                                                            second (at the ceiling one less), not a second count as
                                                            0.6.5-0.6.9: that took an allowance credit the board's own
                                                            time could never give back) */
    }
}

/* the hub's value's step ahead of own (masked: the wire's width), 0 = none or more than cap */
uint32_t energy_ver(uint32_t ver)
{
    return crc8_smbus((const uint8_t *)&ver, 4u);
}

/* the hub's value (n bytes at p, big-endian) ahead of own: the step if 0 < d <= cap, else 0; ls_capped set when it is
 * ahead by more (3 bytes: the wire's 24 bits, modular, ahead below 2^23; 2 / 4: plain, a value below own never (round
 * 9 phase B: 4 bytes were modular too, so a hub's 5 Wh came out "ahead" of 0xFFFFFFF0), ahead below 2^31) */
static uint8_t ls_capped;

static NOINLINE uint32_t ls_step(uint32_t own, const uint8_t *p, uint32_t n, uint32_t cap)
{
    uint32_t v = 0, k = n, half = n == 3u ? 0x7FFFFFu : 0x7FFFFFFFu;
    while (k-- != 0u) {
        v = v << 8 | *p++;
    }
    if (n == 3u) {
        v = (v - own) & 0xFFFFFFu;
    } else if (v < own) {
        return 0;
    } else {
        v -= own;
    }
    if (v > cap) {
        ls_capped |= v <= half;
        v = 0;
    }
    return v;
}

/* the room left below the counters' ceiling (2^32 - 1 s), 0.1 h */
static uint32_t ls_room(uint32_t s, uint32_t cap)
{
    s = (0xFFFFFFFFu - s) / 360u;
    return s < cap ? s : cap;
}

NOINLINE uint32_t energy_sync(energy_t *e, const uint8_t *a, uint32_t uid16)
{
    /* the counts: their offsets in the record and on the wire (LIFETIME's), wire bytes, applied bit */
    static const uint8_t cnt[4][4] = {
        { offsetof(life_rec_t, ups), ULTRA_LT_POWER_UPS, 3u, ULTRA_LS_F_UPS },
        { offsetof(life_rec_t, dips), ULTRA_LT_DIPS, 3u, ULTRA_LS_F_DIPS },
        { offsetof(life_rec_t, trips), ULTRA_LT_OVERLOAD_TRIPS, 2u, ULTRA_LS_F_TRIPS },
        { offsetof(life_rec_t, trips) + 2u, ULTRA_LT_SHORT_TRIPS, 2u, ULTRA_LS_F_TRIPS } };
    life_rec_t *r = &e->lt;
    const uint8_t *l = a + ULTRA_LS_LIFETIME;
    uint32_t m, d, k, ds, cap;
    if (ultra_get16(a + ULTRA_LS_UID16) != uid16) {
        return ULTRA_LS_R_UID << 8;
    }
    if (a[ULTRA_LS_RECORDS] != (uint8_t)r->writes) {
        return ULTRA_LS_R_RECORDS << 8;                  /* (the record the hub's counters aren't reconciled with) */
    }
    if (e->synced) {
        return ULTRA_LS_R_ONCE << 8;
    }
    ls_capped = 0;
    k = LIFE_MERGED(r);
    cap = 0xFFFFFFu - k;
    d = ls_step(r->on_s / 360u, l + ULTRA_LT_POWERED, 3u, ls_room(r->on_s, cap < LIFE_SYNC_MAX_H10 ? cap :
                                                                           LIFE_SYNC_MAX_H10));
                                                         /* (round 9 phase B: and the room below on_s's ceiling, so a
                                                            merge never wraps it: a hub further ahead, CAPPED) */
    ds = d * 360u;                                       /* the powered time raised, s */
    e->base_on_s += ds;
    e->saved_on_s += ds;                                 /* (merged = stored: no write due for it) */
    r->on_s += ds;
    k += d;                                              /* (the allowance: own time only) */
    r->merged = (uint16_t)k;
    r->merged_hi = (uint8_t)(k >> 16);
    m = d != 0u;                                         /* (ULTRA_LS_F_POWERED) */
    d = ls_step(r->lit_s / 360u, l + ULTRA_LT_LIT, 3u, ls_room(r->lit_s, d + 1u));
    r->lit_s += d * 360u;
    m |= (uint32_t)(d != 0u) << 1;                       /* (ULTRA_LS_F_LIT) */
    cap = ds / LIFE_SYNC_EV_S + LIFE_SYNC_EV_MIN;
    for (k = 0; k < 4u; k++) {
        uint8_t *f = (uint8_t *)r + cnt[k][0];
        if (cnt[k][2] == 3u) {
            d = ls_step(*(uint32_t *)f, l + cnt[k][1], 3u, cap);
            *(uint32_t *)f += d;
        } else {
            d = ls_step(*(uint16_t *)f, l + cnt[k][1], 2u, cap);
            *(uint16_t *)f = (uint16_t)(*(uint16_t *)f + d);
        }
        m |= d != 0u ? cnt[k][3] : 0u;
    }
    d = ls_step(energy_total_wh(e), l + 16u, 4u, (ds + 3600u) / 36u * (LIFE_SYNC_MAX_W / 100u));
    e->base_wh += d;
    e->saved_wh += d;
    m |= (uint32_t)(d != 0u) << 5;                       /* (ULTRA_LS_F_WH) */
    e->synced = (uint8_t)(m != 0u);
    return m | (ls_capped ? ULTRA_LS_R_CAPPED : m ? ULTRA_LS_R_APPLIED : ULTRA_LS_R_NOOP) << 8;
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
    uint32_t m = s[offsetof(life_rec_t, magic)];
    return (m == LIFE_MAGIC || m == LIFE_MAGIC_F2) && !crc16_erased(c) && c == crc16_ccitt(s, LIFE_REC_BYTES - 2u);
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
