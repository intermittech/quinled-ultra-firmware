/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Settings records in one 256-byte flash page (wear levelling).
 * 8 slots of 32 bytes are appended one per save; only a full page is erased (fast page erase, 16 ms).
 * Flash endurance is 10k erase cycles minimum (datasheet), so >= 80k saves. Saves are also debounced
 * (2 s after the last change), so a burst of config commands costs one record.
 * Write budget (0.6.2, main.c): STORE_BUDGET_CAP saves at once, then one per STORE_REFILL_MS (scene.h's budget). An
 * automation that changes a stored value all the time (Ultra CONFIG 22 / 23, config frames) costs at most 8 records
 * a day, one erase: >= 27 years at 10k cycles. A change waiting for its token is stored with the newest values when
 * the token comes (none lost, only stored later). The console's `save` writes at once.
 * The newest valid record (highest sequence number, CRC ok) wins at boot. */
#ifndef STORE_H
#define STORE_H

#include <stdint.h>
#include "config.h"

#define STORE_SLOTS       8u
#define STORE_SLOT_BYTES  32u
#define STORE_MAGIC       0x55AA2917u   /* a record of this layout (the CRC last) */
#define STORE_BUDGET_CAP  16u           /* write budget: a console session's worth of changes at once ... */
#define STORE_REFILL_MS   10800000u     /* ... then one token per 3 h (8 a day sustained) */

/* the record's extension: the board current limit (core/isense.h) and options; 0xFF = not set. The current
 * calibration has its own page (core/calstore.h). */
#define STORE_OPT_SOFT_START 0x01u  /* opts: soft start on (core/sstart.h; 0.6.0) */
typedef struct {
    uint8_t ilimit50;          /* board current limit, x 50 mA (0 / 0xFF = the board rating) */
    uint8_t opts;              /* STORE_OPT_*, 0xFF = the defaults (on) */
    uint8_t reserved[5];       /* 0xFF */
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
/* the first of n records of rec bytes that reads erased (n: none) */
uint32_t store_first_erased(const uint8_t *p, uint32_t rec, uint32_t n);
static inline int store_next_free(const uint8_t *page)
{
    return (int)store_first_erased(page, STORE_SLOT_BYTES, STORE_SLOTS);
}
/* n bytes (even, half-word aligned) of erased flash: every half-word reads 0xFFFF, or 0xE339 - the CH32V203 RM
 * (32.5.4) gives 0xE339E339 as the erased value; both are accepted until the bench settles it */
int  store_erased(const uint8_t *b, uint32_t n);
/* a record (store_pack: with a cleared extension) */
void store_pack(const settings_t *s, uint32_t seq, store_slot_t *slot);
void store_pack_ext(const settings_t *s, const store_ext_t *e, uint32_t seq, store_slot_t *slot);

#endif
