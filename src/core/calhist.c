/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Calibration history: see calhist.h. */
#include "calhist.h"
#include "config.h"                         /* crc16_ccitt */
#include "health.h"
#include "lto.h"

typedef char calhist_size[(sizeof(calhist_t) == CALHIST_REC_BYTES && CALHIST_N == 2u) ? 1 : -1];

int calhist_valid(const uint8_t *r)
{
    const calhist_t *h = (const calhist_t *)r;
    /* a CRC that reads like erased flash: a record cut before its last half-word (config.h
     * crc16_erased, as the calibration's and the lifetime record's) */
    return h->magic == CALHIST_MAGIC && h->format == 3u && h->channels == BOARD_CHANNELS && h->n <= CALHIST_N && !crc16_erased(h->crc) &&
           h->crc == crc16_ccitt(r, CALHIST_REC_BYTES - 2u);
}

MEASURED void calhist_add(calhist_t *h, uint32_t lit_s, const isense_cal_t *c)
{
    uint16_t *e;
    uint32_t k, v;
    if (h->n >= CALHIST_N) {                /* full: the first stays, the oldest of the others goes */
        h->n = CALHIST_N - 1u;
    }
    e = &h->e[h->n++].lit_h;
    v = lit_s / 3600u;
    e[0] = (uint16_t)(v > 65535u ? 65535u : v);
    for (k = 0; k < ISENSE_CH; k++) {
        e[1u + k] = (uint16_t)health_i100(c, k);   /* (< 2^16: the ADC's range) */
    }
    h->tests++;
}

uint32_t calhist_trend(const calhist_t *h, int8_t pct[ISENSE_CH])
{
    const calhist_ent_t *a = &h->e[0], *b = &h->e[h->n > 1u ? h->n - 1u : 0u];
    uint32_t k;
    for (k = 0; k < ISENSE_CH; k++) {
        pct[k] = h->n < 2u || a->ma[k] < HEALTH_NOLOAD_MA || b->ma[k] < HEALTH_NOLOAD_MA ? (int8_t)ULTRA_HE_NA
                 : dev_pct(b->ma[k], a->ma[k]);
    }
    return (uint32_t)(b->lit_h - a->lit_h);
}

void calhist_pack(calhist_t *h)
{
    h->magic = CALHIST_MAGIC;
    h->format = 3u; h->channels = BOARD_CHANNELS;
    h->seq++;
    while (crc16_erased(h->crc = crc16_ccitt((const uint8_t *)h, CALHIST_REC_BYTES - 2u))) {
        h->rsv2--;                                       /* (never a CRC that reads erased; rsv2 is read
                                                            by nothing) */
    }
}
