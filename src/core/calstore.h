/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Current calibration in flash (boards with the input current sense, core/isense.h).
 *
 * 64-byte records (cal_rec_t), CAL_RECS per 256-byte page, in an A/B page pair (0.6.4, core/pagepair.h; read and
 * written by core/flashrec.c cal_load / cal_store). Where the two pages are: hal/board.h's record map (CAL_PAGE_ADDR /
 * CAL_B_ADDR); a firmware update never erases them.
 *  - Commit: a stored test is appended with the next sequence number into an erased entry; a write never erases. The
 *    CRC-16 is the last half-word, programmed last, and (0.6.5, the upper layout) never an erased value (cal_pack), so
 *    a record cut short by a power loss never counts. When a page is full the other one is next: it is erased first,
 *    in a low moment (ahead, or by the write when its erase is due; main.c flash_may). The page holding the newest
 *    record is never erased.
 *  - Recovery at boot (pp_find): of both pages, the valid record (magic and CRC) with the newest sequence number wins,
 *    so a power cut at any moment leaves the last complete record (before 0.6.4 a cut after the erase of the one full
 *    page lost the calibration). A failed write uses up its entry but never counts.
 *  - A record with levels = 0 clears the calibration (factory reset). Until it is written, the settings record's
 *    STORE_VOID_CAL (core/store.h) keeps the old one from loading.
 *  - Write budget (0.6.2, main.c): CAL_BUDGET_CAP tests stored at once, then one per CAL_REFILL_MS; a test waiting for
 *    its token is in use already, and the newest one is stored when the token comes.
 *  - Wear: each page is erased once per 2 x CAL_RECS records, so an automation running the test all the time (4 a
 *    day) erases each page every 2 days: >= 54 years at the datasheet's 10k cycles. */
#ifndef CALSTORE_H
#define CALSTORE_H

#include <stdint.h>
#include "isense.h"

#define CAL_PAGE_BYTES    256u
#define CAL_REC_BYTES     64u
#define CAL_RECS          (CAL_PAGE_BYTES / CAL_REC_BYTES)
#define CAL_MAGIC         0x324C4143u   /* "CAL2": the test as isense_cal_t (0.6.1) */
#define CAL_BUDGET_CAP    4u
#define CAL_REFILL_MS     21600000u     /* one test stored per 6 h sustained */

typedef struct {
    uint32_t magic;
    uint32_t seq;
    uint16_t count;                   /* calibrations stored on this board (wraps at 65535) */
    uint16_t rsv;                     /* 0xFFFF */
    isense_cal_t c;                   /* the test (48 bytes; levels 0 = cleared) */
    uint8_t  rsv2[2];                 /* 0xFF */
    uint16_t crc;                     /* CRC-16/CCITT-FALSE over the 62 bytes before it */
} cal_rec_t;

/* a record with the magic and its CRC (core/pagepair.h's check) */
int  cal_valid(const uint8_t *b);
/* newest valid record: its index (and *out, *seq, *count), or -1 */
int  cal_find(const uint8_t *page, isense_cal_t *out, uint32_t *seq, uint16_t *count);
/* where the next record goes: the first erased slot, CAL_RECS when the page is full (erase it, then slot 0) */
#include "store.h"
static inline int cal_next_free(const uint8_t *page)
{
    return (int)store_first_erased(page, CAL_REC_BYTES, CAL_RECS);
}
void cal_pack(const isense_cal_t *c, uint32_t seq, uint16_t count, cal_rec_t *rec);

#endif
