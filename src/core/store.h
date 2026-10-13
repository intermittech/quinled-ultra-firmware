/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Settings records in one 256-byte flash page (wear levelling).
 * 8 slots of 32 bytes are appended one per save; only a full page is erased (fast page erase, 16 ms).
 * Flash endurance is 10k erase cycles minimum (datasheet), so >= 80k saves. Saves are also debounced
 * (2 s after the last change), so a burst of config commands costs one record.
 * The newest valid record (highest sequence number, CRC ok) wins at boot. */
#ifndef STORE_H
#define STORE_H

#include <stdint.h>
#include "config.h"

#define STORE_SLOTS       8u
#define STORE_SLOT_BYTES  32u
#define STORE_MAGIC       0x55AA2917u   /* a record of this layout (the CRC last) */

/* the record's extension: the board current limit (core/isense.h); 0xFF = not set. The current calibration has its
 * own page (core/calstore.h). */
typedef struct {
    uint8_t ilimit50;          /* board current limit, x 50 mA (0 / 0xFF = the board rating) */
    uint8_t reserved[6];       /* 0xFF */
} store_ext_t;

typedef struct {
    uint32_t    magic;
    uint32_t    seq;
    settings_t  s;             /* 16 bytes */
    store_ext_t ext;           /* 7 bytes */
    uint8_t     crc;           /* CRC-8 over seq + s + ext */
} store_slot_t;

/* newest valid record: returns its slot index and fills *out / *seq, or -1 */
int  store_find(const uint8_t *page, settings_t *out, uint32_t *seq);
/* same, with the extension */
int  store_find_ext(const uint8_t *page, settings_t *out, store_ext_t *ext, uint32_t *seq);
void store_ext_clear(store_ext_t *e);
/* first erased slot, STORE_SLOTS when the page is full (erase needed) */
int  store_next_free(const uint8_t *page);
/* n bytes (even, half-word aligned) of erased flash: every half-word reads 0xFFFF, or 0xE339 - the CH32V203 RM
 * (32.5.4) gives 0xE339E339 as the erased value; both are accepted until the bench settles it */
int  store_erased(const uint8_t *b, uint32_t n);
/* a record (store_pack: with a cleared extension) */
void store_pack(const settings_t *s, uint32_t seq, store_slot_t *slot);
void store_pack_ext(const settings_t *s, const store_ext_t *e, uint32_t seq, store_slot_t *slot);

#endif
