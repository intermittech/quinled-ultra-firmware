/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "calstore.h"
#include "store.h"
#include "ultra_protocol.h"

typedef char cal_rec_size_check[(sizeof(cal_rec_t) == CAL_REC_BYTES) ? 1 : -1];

void cal_pack(const isense_cal_t *c, uint32_t seq, uint16_t count, cal_rec_t *rec)
{
    uint32_t i;
    uint8_t *b = (uint8_t *)rec;
    for (i = 0; i < CAL_REC_BYTES; i++) {
        b[i] = 0xFF;
    }
    rec->magic = CAL_MAGIC;
    rec->seq = seq;
    rec->count = count;
    rec->c = *c;
    rec->crc = ultra_crc16(b, CAL_REC_BYTES - 2u);
}

int cal_find(const uint8_t *page, isense_cal_t *out, uint32_t *seq, uint16_t *count)
{
    int best = -1;
    uint32_t i, best_seq = 0;
    for (i = 0; i < CAL_RECS; i++) {
        const cal_rec_t *r = (const cal_rec_t *)(page + i * CAL_REC_BYTES);
        if (r->magic == CAL_MAGIC && r->crc == ultra_crc16((const uint8_t *)r, CAL_REC_BYTES - 2u) &&
            (best < 0 || r->seq > best_seq)) {
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

