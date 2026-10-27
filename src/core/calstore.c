/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include <stddef.h>
#include "calstore.h"
#include "store.h"
#include "ultra_protocol.h"

typedef char cal_rec_size_check[(sizeof(cal_rec_t) == CAL_REC_BYTES &&
    offsetof(cal_rec_t, c) == 12u && sizeof(isense_cal_t) == 48u &&
    offsetof(cal_rec_t, rsv2) == 60u && offsetof(cal_rec_t, crc) == 62u) ? 1 : -1];

void cal_pack(const isense_cal_t *c, uint32_t seq, uint16_t count, cal_rec_t *rec)
{
    uint8_t *b = (uint8_t *)rec;
    /* The payload is copied below; only reserved bytes need the erased value. */
    rec->rsv = 0xFFFFu;
    rec->rsv2[0] = rec->rsv2[1] = 0xFFu;
    rec->magic = CAL_MAGIC;
    rec->format = 3u; rec->channels = BOARD_CHANNELS;
    rec->seq = seq;
    rec->count = count;
    rec->c = *c;
    while (crc16_erased(rec->crc = crc16_ccitt(b, CAL_REC_BYTES - 2u))) {
        rec->rsv2[0]--;                                  /* (see crc16_erased; rsv2 is read by nothing) */
    }
}

int cal_valid(const uint8_t *b)
{
    const cal_rec_t *r = (const cal_rec_t *)b;
    return r->magic == CAL_MAGIC && r->format == 3u && r->channels == BOARD_CHANNELS && !crc16_erased(r->crc) && r->crc == crc16_ccitt(b, CAL_REC_BYTES - 2u);
}

int cal_find(const uint8_t *page, isense_cal_t *out, uint32_t *seq, uint16_t *count)
{
    int best = -1;
    uint32_t i, best_seq = 0;
    for (i = 0; i < CAL_RECS; i++) {
        const cal_rec_t *r = (const cal_rec_t *)(page + i * CAL_REC_BYTES);
        if (cal_valid((const uint8_t *)r) && (best < 0 || r->seq > best_seq)) {
            best = (int)i;
            best_seq = r->seq;
        }
    }
    if (best >= 0) {
        const cal_rec_t *r = (const cal_rec_t *)(page + (uint32_t)best * CAL_REC_BYTES);
        *out = r->c;
        *seq = best_seq;
        *count = r->count;
    }
    return best;
}

