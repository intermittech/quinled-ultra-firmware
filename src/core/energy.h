/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Input energy and persistent lifetime counters. The application admits writes
 * only while dark. The 52-byte native record has magic, explicit format 3 and
 * channel/output count, a CRC salt and CRC-16 in its final halfword. Incompatible
 * record lengths/formats are rejected; there is no legacy migration.
 *
 * Four records fit each 256-byte A/B page. The newest valid record survives a
 * torn write; the page containing it is never erased before its replacement.
 * Four-hour refill/earned allowance retains the wear budget at this density.
 * A bright power cut can lose uncommitted RAM counters. LIFE_SYNC reconciles
 * bounded counters once per boot without granting extra flash-write allowance.
 * Its positive energy bound comes from the descriptor input power rating;
 * unknown maximum voltage means no positive energy merge.
 *
 * Measurements integrate V x I in mWh, with wide accumulators only where needed.
 * Per-channel energy is a RAM-only split using the current model. */
#ifndef ENERGY_H
#define ENERGY_H

#include <stdint.h>
#include "isense.h"                 /* the channel model (energy per channel) */
#include "scene.h"                  /* the write budget (scene_budget_t) */
#include "store.h"
#include "pagepair.h"

#define LIFE_MAGIC         0x4Eu    /* format 3 (not 0xFF / 0xE3: erased flash; 0x4C: format 1) */
#define LIFE_MAGIC_F2      0x4Du    /* incompatible magic, retained for rejection tests */
#define LIFE_REC_BYTES     52u
#define LIFE_RECS          4u       /* per 256-byte page (the last 48 bytes stay unused) */
#define ENERGY_DARK_MS     30000u   /* dark this long after being lit: a write */
#define LIFE_SAVE_S        43200u   /* a write at most this long after the last one (12 h) */
#define LIFE_IDLE_S        86400u   /* powered time alone is worth a regular write after a day */
#define LIFE_DIP_S         600u     /* ... and a dip write after 10 min */
#define LIFE_PREP_S        10u      /* an erase while dark: not before this long after power-up (steady supply) */
#define LIFE_BUDGET_CAP    4u       /* write tokens at boot */
#define LIFE_REFILL_MS     14400000u   /* one more per 4 h: <= 6 a day sustained */
#define LIFE_RESERVE       32768u   /* lifetime allowance: these writes ... */
#define LIFE_EARN_S        14400u   /* ... plus one per 4 h of powered time */
#define ENERGY_UJ_PER_MWH  3600000u
/* the hub's counters (LIFE_SYNC page, energy_sync): the steps a merge takes at most */
#define LIFE_SYNC_MAX_H10  7200u    /* powered time: 30 days (0.1 h) */
#define LIFE_SYNC_EV_S     10u      /* power-ups: one per this much of the powered time raised ... */
#define LIFE_SYNC_EV_MIN   64u      /* ... + this; the trips and dips: this */
#define ENERGY_SYNC_ONCE   0x01u    /* energy_t.synced: a merge applied this boot (the next one: ULTRA_LS_R_ONCE) */
#define ENERGY_SYNC_HUB    0x02u    /* the hub synced this boot, applied or nothing ahead (not CAPPED):
                                       writes only for an event, a dip or a week (energy_second; the receivers) */
#define LIFE_SYNC_SAVE_S   604800u  /* a synced boot's time write: a week powered since the last write
                                       (merges included), well under the 30-day step (LIFE_SYNC_MAX_H10) */

/* why energy_due() wants a write (either goes into an erased entry); also the bits of its `admit` argument */
#define ENERGY_DUE_NONE    0u
#define ENERGY_DUE_SAVE    1u       /* a regular write */
#define ENERGY_DUE_DIP     2u       /* a supply dip */
#define ENERGY_PLANNED     4u       /* admit: the planned cut starts now (the receiver: a dip during its
                                       IDENTIFY 7 notice, core/vdip.h in_dip 2; `dips` unchanged): not counted, and
                                       written only for something worth storing (not the power-up alone) */

typedef struct {                    /* the lifetime record, format 3, 52 bytes */
    uint32_t wh;                    /*  0: input energy, Wh */
    uint32_t on_s;                  /*  4: powered time, s */
    uint32_t lit_s;                 /*  8: time with an output on, s */
    uint32_t ups;                   /* 12: power-ups */
    uint32_t dips;                  /* 16: supply dips */
    uint8_t  ver;                   /* 20: the app version last seen: the CRC-8 of its core/fwver.h word (energy_ver;
                                       development builds aren't compared); format 2: bytes 20..23 the word itself */
    uint8_t  merged_hi;             /* 21: (format 3) the powered time a hub merged in (LIFE_SYNC), 0.1 h, 24 bits: */
    uint16_t merged;                /* 22: ... its low 16 (LIFE_MERGED). The board's own powered time is on_s - merged x
                                       360 s, its write allowance's */
    uint32_t writes;                /* 24: records written, this one included (the newest has the most; one a write) */
    uint16_t trips[2];              /* 28: overload trips, 30: short-circuit trips (saturating) */
    uint16_t updates;               /* 32: firmware updates: boots with a new app version */
    uint8_t  wd;                    /* 34: watchdog resets (saturating) */
    uint8_t  peak_t;                /* 35: the highest MCU temperature, 0.5 C (0..127.5 C) */
    uint8_t  peak_a;                /* 36: the highest board current (~100 ms average), 0.5 A */
    /* (format 2, B2) the fault memory: the last of each, 0 = none */
    uint8_t  trip;                  /* 37: the last trip: 1 short, 2 overload (bits 0-1) | its current, 2 A (bits 2-7,
                                       saturating at 126 A; a short: its peak) */
    uint16_t trip_h;                /* 38: ... the lit hours then (saturating) */
    uint8_t  dip_pct;               /* 40: the last supply dip: its lowest reading, % of its reference */
    uint8_t  dip_cs;                /* 41: ... how long, x 10 ms (255: longer) */
    uint8_t  dip_a;                 /* 42: ... the board current at its lowest, 0.5 A (saturating) */
    uint8_t  rst;                   /* 43: the last abnormal reset: 1 watchdog, 2 an exception (then the watchdog)
                                       (bits 0-1) | the exception's mcause (bits 2-7) */
    uint8_t  rst_h;                 /* 44: ... the uptime of that run, h (saturating) */
    uint8_t  magic;                 /* 45: LIFE_MAGIC */
    uint8_t format, count;
    uint16_t salt;
    uint16_t crc;                   /* 50: CRC-16/CCITT-FALSE (ultra_crc16) over bytes 0..49 */
} life_rec_t;

typedef pp_pos_t life_pos_t;        /* where the next record goes: page << 4 | entry, erase pending (pagepair.h) */
#define LIFE_PLAN_NONE     PP_NONE

typedef struct {
    life_rec_t lt;                  /* the lifetime values now: as stored + the counters since; wh, writes, crc: as
                                       last stored (energy_total_wh() is the energy now). Written from here. */
    uint32_t uj;                    /* below one mWh, uJ */
    uint32_t boot_mwh;              /* since boot (wraps: the ENERGY page's counter) */
    uint32_t mwh_seen;              /* boot_mwh already carried into base_wh (whole Wh) */
    uint32_t base_wh;               /* the lifetime energy, Wh: as stored at boot + the Wh carried since */
    uint32_t base_on_s;             /* as stored at boot */
    uint32_t saved_wh, saved_on_s;  /* as stored by the last write (at boot: as loaded) */
    uint32_t saved_s;               /* uptime of the last write, s (the page's age, LIFE_SAVE_S) */
    uint32_t lit_ms;                /* the last ms with an output on (settled: never more than 2^30 back) */
    uint8_t  lit_new;               /* an output was on since the last write (a flag: no time compare that can age) */
    uint8_t  ev;                    /* a counter event since the last write: bit 0 a trip, watchdog reset or update,
                                       bit 1 the power-up (alone it writes nothing at a planned power-off);
                                       bit 2 a dip's write not made yet (not admitted, no token,
                                       failed): energy_second makes it a regular one, without the dark wait */
    uint8_t  due;                   /* energy_second: a regular write is due (and allowed); also a dip's pending
                                       one (ev bit 2), without the dark wait (DIAG FP_LIFETIME) */
    uint8_t  dip_ok;                /* energy_second: a dip would have something to store (and is allowed); bit 1:
                                       a planned power-off's too; these four in one word: energy_saved clears
                                       them at once */
    uint8_t  ev_save;               /* the ev bits a regular write takes: 3 (energy_init); the receiver 1:
                                       its power-up alone is stored by an unplanned cut's dip write only, so a standby
                                       cycle (power-up, a light, dark, the notice) writes nothing for itself */
    uint8_t  held;                  /* energy_second: the lifetime allowance is used
                                       up (the receiver's DIAG EXHAUSTED; see the limit above) */
    uint16_t dips;                  /* the supply dips since boot, as last seen (vdip's count, u16: differences) */
    uint16_t trips_seen[2];         /* the trips since boot (core/pwrev.h's u16 counts), as last seen */
    uint8_t  saved;                 /* stored since boot */
    uint8_t  synced;                /* energy_sync this boot: ENERGY_SYNC_ONCE a merge applied (one a boot);
                                       ENERGY_SYNC_HUB the hub's ledger agrees (applied or nothing ahead) */
    scene_budget_t budget;
    /* energy per channel (RAM only) */
    uint32_t ch_d[ISENSE_CH];       /* this second: each output's sum (16-bit levels x ms) */
    uint32_t ch_i;                  /* ... the board current's sum, mA */
    uint32_t ch_n;                  /* ... ms */
    uint32_t ch_mwh[ISENSE_CH];     /* each channel since boot, mWh */
    uint16_t ch_mj[ISENSE_CH];      /* ... below one mWh, mJ */
} energy_t;

/* at boot: found = e->lt holds the newest stored record (life_find; else it is cleared); ver = this app's version
 * (fwver word, 0 = a development build: not compared); wd = the reset was the watchdog's. Counts the power-up. */
void     energy_init(energy_t *e, uint32_t found, uint32_t now, uint32_t ver, uint32_t wd);
/* every ms: the supply (mV), the board current (mA) and whether an output is on */
void     energy_ms(energy_t *e, uint32_t now, uint32_t mv, int32_t ma, uint8_t lit);
/* once a second (now_s = uptime, s): the energy's carry, powered / lit time, the trips since boot (pwrev's u16
 * counts: short << 16 | overload), the board current (mA, ~100 ms average) and the MCU temperature peak since boot
 * (0.1 C, MCUTEMP_NA = none); decides what is due */
void     energy_second(energy_t *e, uint32_t now, uint32_t now_s, uint32_t trips, int32_t i_ma, int32_t temp_c10);
/* the energy now, Wh */
uint32_t energy_total_wh(const energy_t *e);
/* every ms: a write due now (ENERGY_DUE_*)? dips = the supply dips since boot (core/vdip.h); admit = the writes the
 * flash may take now (ENERGY_DUE_SAVE / ENERGY_DUE_DIP bits; 0 while no erased entry is there). A write that isn't
 * admitted waits, without a token. Takes a budget token when it says yes: then the store (life_plan, energy_pack,
 * program e->lt), then energy_saved(). */
uint32_t energy_due(energy_t *e, uint32_t now, uint16_t dips, uint32_t admit);
/* e->lt as the record to write now: the energy, the next write count, the check */
void     energy_pack(energy_t *e);
void     energy_saved(energy_t *e, uint32_t now_s);
/* (the receiver's LIFE_SYNC page) a = the POLL arguments (ULTRA_LS_*), uid16 = the board's:
 * the hub's counters merged, once a boot, when its UID16 and RECORDS low 8 bits match. Each counter is raised by the
 * hub's step d ahead (wire units: 0.1 h, Wh, counts; a 24-bit field ahead below 2^23, modular; the others plain: a
 * lower value never) when 0 < d <= its step: powered LIFE_SYNC_MAX_H10 (and `merged` below 2^24); lit the powered
 * step + 1; both also no further than the room below their seconds' ceiling (2^32 - 1 s, ~136 years;
 * a merge never wraps them: further ahead, CAPPED); power-ups one per LIFE_SYNC_EV_S of the powered time raised +
 * LIFE_SYNC_EV_MIN, trips and dips LIFE_SYNC_EV_MIN; the energy num / den Wh per 0.1 h over it + 1 h, rounded up
 * (the board's input rating x maximum input voltage: BOARD_IN_WH_NUM / BOARD_IN_WH_DEN, exact; num 0: either unknown,
 * no energy merge; num x (LIFE_SYNC_MAX_H10 + 10) + den - 1 below 2^32, tools/board_descriptor.py checks it). At the
 * ceiling the board's own counting stops too (on_s, lit_s, the energy: they stay, never lowered; its own time, and so
 * the allowance, stop growing).
 * A merged step counts as stored (saved_on_s / saved_wh too): it never makes a write due; it rides along with the
 * board's own next one. Except powered time of LIFE_IDLE_S (a day) or more: that makes a write due, as a
 * day powered does, so a receiver whose standby wakes never write keeps it and the hub's lead never grows past its
 * step. Applied or nothing ahead (not CAPPED) marks the boot ENERGY_SYNC_HUB: the hub's ledger carries
 * the energy and the time, so this boot writes the record only for an event (a trip, a watchdog reset, an update), an
 * unplanned dip, or a week powered since the last write (LIFE_SYNC_SAVE_S, merges included); never for the energy, a
 * day or the 10 minutes alone (a planned cut, a dark moment). Returns the applied fields (ULTRA_LS_F_*) |
 * ULTRA_LS_R_* << 8 */
uint32_t energy_sync(energy_t *e, const uint8_t *a, uint32_t uid16, uint32_t num, uint32_t den);
/* the record's version field for a core/fwver.h word: its CRC-8 (crc8_smbus over its 4 bytes) */
uint32_t energy_ver(uint32_t ver);
/* the powered time a hub merged in, 0.1 h (format 3's 24 bits) */
#define LIFE_MERGED(r)     ((uint32_t)(r)->merged_hi << 16 | (r)->merged)
/* the pair's description (core/pagepair.h; flashrec.c's descriptor): a record valid (magic, CRC-16) */
extern const pp_desc_t life_pp;
/* the two pages: the newest valid record into *r (returns 1; 0 = none) and where the next one goes (the entry after
 * it when that reads erased, else the other page's first: erased first unless it is) */
uint32_t life_find(const uint8_t *page0, const uint8_t *page1, life_rec_t *r, life_pos_t *pos);
/* a write at pos: returns where (page << 4 | entry: PP_PAGE / PP_ENTRY), or LIFE_PLAN_NONE while the next page must
 * be erased first (pos->erase: page PP_PAGE(pos->next); set when a write fills its page). Moves pos on. */
uint32_t life_plan(life_pos_t *pos);
/* erased entries left for writes without an erase (pagepair.h pp_free) */
uint32_t life_free(const life_pos_t *pos);
/* energy per channel: every ms the outputs in effect (isense's applied) and the board current (mA); once a second the
 * split, with the channel model and the supply (mV) */
void     energy_ch_ms(energy_t *e, const uint16_t out[ISENSE_CH], int32_t ma);
void     energy_ch_second(energy_t *e, const isense_t *is, uint32_t mv);
/* ENERGY page DATA (ULTRA_EN_*); now_s = the uptime in s */
void     energy_page(const energy_t *e, uint32_t now_s, uint32_t power_mw, uint8_t out[24]);

#endif
