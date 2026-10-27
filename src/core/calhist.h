/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Calibration history: each channel's current over the board's lit life, from the current calibration tests
 * (core/isense.h, core/calstore.h). Display only (minute records h and H; bench cal_history / cal_trend).
 * CAL2 deliberately starts a new history on upgrade from CALH; calibration and lifetime records are unchanged.
 *
 * Each stored test adds an entry: the lifetime record's lit hours (core/energy.h) and each channel's current at
 * 100 % (core/health.h health_i100; a single-level test scaled). Kept: the first test since the history began (a
 * factory reset begins it anew) and the newest CALHIST_N - 1; cal_trend compares the newest with the first.
 *
 * Storage: the whole history is one 40-byte record ("CAL2", the sequence and a CRC-16 like the calibration's), written
 * after each stored test into its own A/B page pair (core/pagepair.h; 6 records a page) through main.c's flash policy
 * (flash_may: a program of 20 half-words, as the calibration's; erases while low). The newest valid record is the
 * history. The store: core/flashrec.c calhist_load / calhist_store / calhist_pending / calhist_prepare, the record
 * pairs' one path, in its own two sectors of the upper CodeFlash (mcu/ch32v20x/layout.h CALHIST_ADDR / CALHIST_B_ADDR), loaded
 * in the boot read window. Built with extra monitoring (core/config.h MON_EXTRA): not for the
 * Diff-Power-UR-Analog-M. Wear: a write per stored test (CAL_BUDGET_CAP at once, then one per 6 h: <= 4 a
 * day), each page erased once per 12 writes: >= 82 years at 10,000 cycles even with the test running all the time.
 *
 * What it shows is the electrical load, not the light: an LED's current at a fixed duty hardly changes as its light
 * output fades (lumen depreciation can't be seen here). It changes when segments fail (open: less current), LEDs
 * short (more), a strip is changed, or the supply differs (a constant-voltage strip's current follows the supply
 * minus its LEDs' forward voltage: compare tests at the same supply), and with the strip's temperature (warm LEDs
 * have a lower forward voltage: a few % more current).
 * Pure logic, host-tested (test/test_calhist.c). */
#ifndef CALHIST_H
#define CALHIST_H

#include <stdint.h>
#include "../board/boards.h"
#include "isense.h"

#define CALHIST_N         2u
#define CALHIST_MAGIC     0x4143u   /* "CAL2": first and latest only */
#define CALHIST_REC_BYTES 40u
#define CALHIST_RECS      6u            /* per 256-byte page */

typedef struct {
    uint16_t lit_h;                     /* the lifetime record's lit hours at the test (saturating) */
    uint16_t ma[ISENSE_CH];             /* each channel at 100 %, mA (0: no load) */
} calhist_ent_t;

typedef struct {                        /* the record as stored, and the history in RAM */
    uint16_t magic;
    uint8_t format, channels;                     /*  0: CALHIST_MAGIC */
    uint32_t seq;                       /*  4: records written (the page pair's sequence number) */
    uint16_t tests;                     /*  8: tests since the history began */
    uint8_t  n;                         /* 10: entries */
    uint8_t  rsv;                       /* 11 */
    calhist_ent_t e[CALHIST_N];         /* 12: the first test, then the newest, oldest first */
    uint16_t rsv2;                      /* 36 */
    uint16_t crc;                       /* 38: CRC-16/CCITT-FALSE over bytes 0..37 */
} calhist_t;

/* a record with the magic and its CRC (core/pagepair.h's check) */
int      calhist_valid(const uint8_t *r);
/* a stored test (lit_s: the lifetime record's lit time, s) */
void     calhist_add(calhist_t *h, uint32_t lit_s, const isense_cal_t *c);
/* the newest test against the first, % per channel (ULTRA_HE_NA: fewer than two, or no load in either); returns the
 * lit hours between them */
uint32_t calhist_trend(const calhist_t *h, int8_t pct[ISENSE_CH]);
/* the record to write: the magic, the next sequence number, the CRC */
void     calhist_pack(calhist_t *h);

#endif
