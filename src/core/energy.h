/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Input energy and the board's lifetime record (0.6.3; the energy alone since 0.6.0). Monitoring only: nothing here
 * changes the outputs.
 *  - Energy: the supply reading (core/vsense.h) x the measured board current (core/isense.h), every ms; since boot in
 *    mWh (RAM).
 *  - The lifetime record ("odometer"): energy (Wh), powered time, lit time (any output on), power-ups, short / overload
 *    trips, supply dips, the MCU temperature and board current peaks, watchdog resets and firmware updates (a new app
 *    version at boot). A factory reset never clears it, a firmware update never touches it: only a full chip erase.
 *
 * STABLE FORMAT (LIFE_MAGIC 0x4D = format 2, from 0.6.9 on; format 1, 0x4C, 0.6.3-0.6.8, 40 bytes, isn't read: no
 * board had shipped, Quindor): a firmware that changes the record must still read this one (by its magic) and carry
 * its values over. life_rec_t is little-endian, 48 bytes, programmed in address order: the magic (byte 45) in the
 * second-last half-word, the CRC-16 (bytes 46-47, ultra_crc16 over bytes 0..45) in the last. Format 2 added the fault
 * memory (0.6.9, B2): the last trip, the last supply dip and the last abnormal reset, set in RAM when they happen and
 * stored with the record's next write (never a write of their own). An entry cut short by a power loss either reads its magic as erased flash (0xFF / 0xE3: never taken) or
 * lacks only its CRC (its data complete). A page whose erase was cut short passes as a record 1 in 65,536.
 *
 * Storage: an A/B page pair (hal/board.h LIFE_PAGE0_ADDR / LIFE_PAGE1_ADDR), LIFE_RECS records each, appended in
 * turn. The valid record with the most writes is the newest. A write only ever goes into an erased entry, never
 * erases: when it fills a page, the other page (older records only: the page holding the newest record is never
 * erased) is erased on its own. An interrupted program or erase leaves the old or the new complete record, never
 * none.
 *
 * When the flash may work (main.c flash_may / life_admit, 0.6.4: Quindor's policy for every store; 0.6.3: dark only):
 * a flash operation halves HCLK (the PWM carrier with it, the duty unchanged) and the main loop waits; the erase only
 * in a low moment (every output below 15 % for 500 ms, nothing rising) or at power-up before the outputs start; a
 * write into an erased entry also while brighter (one write an hour for all stores, keeping the last erased entry for
 * a dip) and at a supply dip (the power going away). A write that isn't admitted waits, without a token or an entry;
 * everything builds up in RAM meanwhile, for days if need be.
 *
 * When it is written (only with a write token: LIFE_BUDGET_CAP at boot, one more per LIFE_REFILL_MS, and only within
 * the lifetime allowance: at most LIFE_RESERVE + one per LIFE_EARN_S of powered time over the board's life, which
 * counts across power cycles, so a board powered on and off all day can't wear the flash either):
 *  - at the start of a supply dip (core/vdip.h): the supply may be going away (a wall switch). Only with >= 1 Wh new,
 *    a counter event, or LIFE_DIP_S powered since the last write;
 *  - when the outputs have been dark for ENERGY_DARK_MS after being lit (a controller switched off), or LIFE_SAVE_S
 *    since the last write (then at the first dark moment): with >= 1 Wh new while lit, a counter event, or LIFE_IDLE_S
 *    powered since the last write (a dark board's own logic current never asks for more than a write a day).
 * Counter events (power-up, trip, watchdog reset, update) only ride along with these writes; the dips and the peaks
 * are stored with them but never ask for a write. What isn't stored when the power goes is lost: a board lit without
 * a low moment keeps its session through the hour's writes and the dip write.
 * Energy: the since-boot mWh (u32, the ENERGY page's wrapping counter) is carried into the lifetime Wh once a second,
 * whole Wh, the remainder kept: no step at its 2^32 mWh wrap (466 days at 384 W).
 * Wear (10k erase cycles, the datasheet minimum; LIFE_RECS writes per erase of a page): a board powered all the
 * time, any pattern: <= 8 writes a day, 0.8 erases a day per page: 34 years (format 1, 6 a page: 41). Any pattern at
 * all, with power cycles: <= LIFE_RESERVE + 8 a powered day (test/test_energy.c simulates both, and prints the years).
 *
 * Energy per channel (0.6.5): the measured input energy split over the five channels by the channel model. Every ms
 * the outputs in effect (core/isense.h `applied`, after the limiter and the soft start) and the board current are
 * summed; once a second the second's LED energy (the supply x (the mean current - the zero, the board's own current),
 * whole mJ) is shared out by each channel's modelled current at its mean output (isense_ch_ma: the calibrated /
 * learned curve). RAM only, since boot (the lifetime record's stable format has no room for it): `status`
 * `energy R/G/B/W1/W2=`. Without a usable model (all channels 0) the second goes to no channel; the zero's share
 * (logic, standby) is in `energy=` only. mWh counters wrap like boot_mwh (the console shows up to 2,147 kWh each).
 * Pure logic, host-tested (test/test_energy.c). */
#ifndef ENERGY_H
#define ENERGY_H

#include <stdint.h>
#include "isense.h"                 /* the channel model (energy per channel) */
#include "scene.h"                  /* the write budget (scene_budget_t) */
#include "store.h"
#include "pagepair.h"

#define LIFE_MAGIC         0x4Du    /* format 2 (not 0xFF / 0xE3: erased flash; 0x4C: format 1) */
#define LIFE_REC_BYTES     48u
#define LIFE_RECS          5u       /* per 256-byte page (the last 16 bytes stay unused) */
#define ENERGY_DARK_MS     30000u   /* dark this long after being lit: a write */
#define LIFE_SAVE_S        43200u   /* a write at most this long after the last one (12 h) */
#define LIFE_IDLE_S        86400u   /* powered time alone is worth a regular write after a day */
#define LIFE_DIP_S         600u     /* ... and a dip write after 10 min */
#define LIFE_PREP_S        10u      /* an erase while dark: not before this long after power-up (steady supply) */
#define LIFE_BUDGET_CAP    4u       /* write tokens at boot */
#define LIFE_REFILL_MS     10800000u   /* one more per 3 h: <= 8 a day sustained */
#define LIFE_RESERVE       32768u   /* lifetime allowance: these writes ... */
#define LIFE_EARN_S        10800u   /* ... plus one per 3 h of powered time */
#define ENERGY_UJ_PER_MWH  3600000u

/* why energy_due() wants a write (either goes into an erased entry); also the bits of its `admit` argument */
#define ENERGY_DUE_NONE    0u
#define ENERGY_DUE_SAVE    1u       /* a regular write */
#define ENERGY_DUE_DIP     2u       /* a supply dip */

typedef struct {                    /* the lifetime record, format 2 (STABLE: see above) */
    uint32_t wh;                    /*  0: input energy, Wh */
    uint32_t on_s;                  /*  4: powered time, s */
    uint32_t lit_s;                 /*  8: time with an output on, s */
    uint32_t ups;                   /* 12: power-ups */
    uint32_t dips;                  /* 16: supply dips */
    uint32_t ver;                   /* 20: the app version last seen (core/fwver.h word; 0 = development builds only) */
    uint32_t writes;                /* 24: records written, this one included (the newest has the most) */
    uint16_t trips[2];              /* 28: overload trips, 30: short-circuit trips (saturating) */
    uint16_t updates;               /* 32: firmware updates: boots with a new app version */
    uint8_t  wd;                    /* 34: watchdog resets (saturating) */
    uint8_t  peak_t;                /* 35: the highest MCU temperature, 0.5 C (0..127.5 C) */
    uint8_t  peak_a;                /* 36: the highest board current (~100 ms average), 0.1 A */
    /* (format 2, 0.6.9, B2) the fault memory: the last of each, 0 = none */
    uint8_t  trip;                  /* 37: the last trip: 1 short, 2 overload (bits 0-1) | its current, 0.5 A (bits 2-7,
                                       saturating at 31.5 A; a short: its peak) */
    uint16_t trip_h;                /* 38: ... the lit hours then (saturating) */
    uint8_t  dip_pct;               /* 40: the last supply dip: its lowest reading, % of its reference */
    uint8_t  dip_cs;                /* 41: ... how long, x 10 ms (255: longer) */
    uint8_t  dip_a;                 /* 42: ... the board current at its lowest, 0.1 A (saturating) */
    uint8_t  rst;                   /* 43: the last abnormal reset: 1 watchdog, 2 an exception (then the watchdog)
                                       (bits 0-1) | the exception's mcause (bits 2-7) */
    uint8_t  rst_h;                 /* 44: ... the uptime of that run, h (saturating) */
    uint8_t  magic;                 /* 45: LIFE_MAGIC */
    uint16_t crc;                   /* 46: CRC-16/CCITT-FALSE (ultra_crc16) over bytes 0..45 */
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
    uint8_t  ev;                    /* a counter event since the last write */
    uint8_t  due;                   /* energy_second: a regular write is due (and allowed) */
    uint8_t  dip_ok;                /* energy_second: a dip would have something to store (and is allowed); these
                                       four in one word: energy_saved clears them at once */
    uint16_t dips;                  /* the supply dips since boot, as last seen (vdip's count, u16: differences) */
    uint16_t trips_seen[2];         /* the trips since boot (core/pwrev.h's u16 counts), as last seen */
    uint8_t  saved;                 /* stored since boot */
    scene_budget_t budget;
    /* energy per channel (0.6.5, RAM only) */
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
/* the pair's description (core/pagepair.h; flashrec.c's descriptor, 0.6.6): a record valid (magic, CRC-16) */
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
/* ENERGY page DATA (protocol 0.2.3, ULTRA_EN_*); now_s = the uptime in s */
void     energy_page(const energy_t *e, uint32_t now_s, uint32_t power_mw, uint8_t out[24]);

#endif
