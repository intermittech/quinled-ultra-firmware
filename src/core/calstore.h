/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Current calibration in flash (boards with the input current sense, core/isense.h).
 *
 * One 256-byte page at CAL_PAGE_ADDR (src/hal/board.h: the top page of the app slot, which the app's link leaves
 * free; a firmware update only erases the new app's own length, so the page survives updates). Four 64-byte
 * records, appended one per calibration; only a full page is erased. The newest valid record (highest sequence
 * number, CRC ok) wins at boot. A record with levels = 0 clears the calibration (factory reset).
 * Wear: 4 calibrations per erase, 10k erase cycles (datasheet minimum) = 40k calibrations (a daily schedule:
 * 100 years). A power loss during the erase of a full page loses the calibration (the regression relearns; test
 * again). */
#ifndef CALSTORE_H
#define CALSTORE_H

#include <stdint.h>
#include "isense.h"

#define CAL_PAGE_BYTES    256u
#define CAL_REC_BYTES     64u
#define CAL_RECS          (CAL_PAGE_BYTES / CAL_REC_BYTES)
#define CAL_MAGIC         0x314C4143u   /* "CAL1" */

typedef struct {
    uint32_t magic;
    uint32_t seq;
    uint16_t count;                   /* calibrations stored on this board (wraps at 65535) */
    uint8_t  levels;                  /* isense_cal_t.levels; 0 = cleared */
    uint8_t  flags;                   /* reserved, 0xFF */
    uint8_t  pct[ISENSE_LEVELS];
    int16_t  zero_ma;
    uint16_t ma[ISENSE_CH][ISENSE_LEVELS];
    uint8_t  rsv[4];                  /* 0xFF */
    uint16_t crc;                     /* CRC-16/CCITT-FALSE over the 62 bytes before it */
} cal_rec_t;

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
