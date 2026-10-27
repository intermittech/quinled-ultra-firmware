/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include <stddef.h>
#include "store.h"

typedef char store_slot_size_check[(sizeof(store_slot_t) == STORE_SLOT_BYTES && sizeof(settings_t) == 16u &&
                                    sizeof(store_ext_t) == 7u &&
                                    offsetof(store_slot_t, crc) == 8u + 16u + 7u) ? 1 : -1];   /* no padding */

/* seq, s and ext: the bytes from seq up to the crc */
static uint8_t slot_crc(const store_slot_t *sl)
{
    return crc8_smbus((const uint8_t *)&sl->seq, 4u + (uint32_t)sizeof(settings_t) + (uint32_t)sizeof(store_ext_t));
}

/* the CRC byte is programmed last: a record cut short leaves it erased (0xFF, or 0xE3 with 0xE339 erased
 * flash), which a CRC-8 would match 1 time in 256 (found by test r5dip). store_pack_ext never writes those values (a
 * reserved byte takes another value instead), so a record whose CRC byte reads erased was never completed. */
static int crc_erased(uint32_t c)
{
    return c == 0xFFu || c == 0xE3u;
}

void store_ext_clear(store_ext_t *e)
{
    uint32_t i;
    for (i = 0; i < sizeof(*e); i++) {
        ((uint8_t *)e)[i] = 0xFF;
    }
}

void store_pack_ext(const settings_t *s, const store_ext_t *e, uint32_t seq, store_slot_t *slot)
{
    /* Every byte is assigned below; store_slot_size_check proves there is no padding. */
    slot->magic = STORE_MAGIC;
    slot->format = 3u; slot->channels = BOARD_CHANNELS;
    slot->seq = seq;
    slot->s = *s;
    slot->ext = *e;
    while (crc_erased(slot->crc = slot_crc(slot))) {
        slot->ext.reserved[0]--;                         /* (reserved: nothing reads it) */
    }
}

void store_pack(const settings_t *s, uint32_t seq, store_slot_t *slot)
{
    store_ext_t e;
    store_ext_clear(&e);
    store_pack_ext(s, &e, seq, slot);
}

int store_valid(const uint8_t *r)
{
    const store_slot_t *sl = (const store_slot_t *)r;
    return sl->magic == STORE_MAGIC && sl->format == 3u && sl->channels == BOARD_CHANNELS && !crc_erased(sl->crc) && sl->crc == slot_crc(sl);
}

int store_find_ext(const uint8_t *page, settings_t *out, store_ext_t *ext, uint32_t *seq)
{
    int best = -1;
    uint32_t i, best_seq = 0;
    for (i = 0; i < STORE_SLOTS; i++) {
        const store_slot_t *sl = (const store_slot_t *)(page + i * STORE_SLOT_BYTES);
        if (store_valid((const uint8_t *)sl) && (best < 0 || sl->seq > best_seq)) {
            best = (int)i;
            best_seq = sl->seq;
        }
    }
    if (best >= 0) {
        const store_slot_t *sl = (const store_slot_t *)(page + (uint32_t)best * STORE_SLOT_BYTES);
        *out = sl->s;
        *ext = sl->ext;
        *seq = best_seq;
    }
    return best;
}

int store_find(const uint8_t *page, settings_t *out, uint32_t *seq)
{
    store_ext_t e;
    return store_find_ext(page, out, &e, seq);
}

int store_erased(const uint8_t *b, uint32_t n)
{
    uint32_t i;
    for (i = 0; i + 1u < n; i += 2u) {
        uint16_t h = (uint16_t)(b[i] | (b[i + 1u] << 8));    /* little endian, as the CPU reads it */
        if (h != 0xFFFFu && h != 0xE339u) {
            return 0;
        }
    }
    return 1;
}

uint32_t store_first_erased(const uint8_t *p, uint32_t rec, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n && !store_erased(p + i * rec, rec); i++) {
    }
    return i;
}
