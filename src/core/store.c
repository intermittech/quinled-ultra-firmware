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

void store_ext_clear(store_ext_t *e)
{
    uint32_t i;
    for (i = 0; i < sizeof(*e); i++) {
        ((uint8_t *)e)[i] = 0xFF;
    }
}

void store_pack_ext(const settings_t *s, const store_ext_t *e, uint32_t seq, store_slot_t *slot)
{
    uint32_t i;
    uint8_t *b = (uint8_t *)slot;
    for (i = 0; i < STORE_SLOT_BYTES; i++) {
        b[i] = 0xFF;
    }
    slot->magic = STORE_MAGIC;
    slot->seq = seq;
    slot->s = *s;
    slot->ext = *e;
    slot->crc = slot_crc(slot);
}

void store_pack(const settings_t *s, uint32_t seq, store_slot_t *slot)
{
    store_ext_t e;
    store_ext_clear(&e);
    store_pack_ext(s, &e, seq, slot);
}

int store_find_ext(const uint8_t *page, settings_t *out, store_ext_t *ext, uint32_t *seq)
{
    int best = -1;
    uint32_t i, best_seq = 0;
    for (i = 0; i < STORE_SLOTS; i++) {
        const store_slot_t *sl = (const store_slot_t *)(page + i * STORE_SLOT_BYTES);
        if (sl->magic == STORE_MAGIC && sl->crc == slot_crc(sl) && (best < 0 || sl->seq > best_seq)) {
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

int store_next_free(const uint8_t *page)
{
    uint32_t i;
    for (i = 0; i < STORE_SLOTS; i++) {
        if (store_erased(page + i * STORE_SLOT_BYTES, STORE_SLOT_BYTES)) {
            return (int)i;
        }
    }
    return (int)STORE_SLOTS;
}
