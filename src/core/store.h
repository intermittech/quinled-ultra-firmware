/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Settings records: 32-byte records (store_slot_t), STORE_SLOTS per 256-byte page, in an A/B page pair
 * (core/pagepair.h; read and written by core/flashrec.c settings_load / settings_store). Where the two pages are:
 * mcu/ch32v20x/layout.h's record map (SETTINGS_ADDR / SETTINGS_B_ADDR; the upper layout: each page in its own 4 KB sector).
 *  - Commit: a save appends a record with the next sequence number into an erased entry; a write never erases. The
 *    CRC byte is in the last half-word, programmed last, and (the upper layout) never takes an erased value
 *    (store_pack_ext), so a record cut short by a power loss never counts. When a page is full the other one is next:
 *    it is erased first, in a low moment (ahead, or by the write when its erase is due; main.c flash_may). The page
 *    holding the newest record is never erased.
 *  - Recovery at boot (pp_find): of both pages, the valid record (magic and CRC) with the newest sequence number wins,
 *    so a power cut at any moment leaves the last complete record. An entry after it that doesn't read erased (a cut
 *    or failed write) makes its page count as full; a failed write uses up its entry but never counts.
 *  - Saves are debounced (2 s after the last change: a burst of config commands costs one record) and budgeted
 *    (main.c): STORE_BUDGET_CAP at once, then one per STORE_REFILL_MS (scene.h's budget). A change waiting for its
 *    token is stored with the newest values when the token comes (none lost, only stored later). The full console's
 *    `save` (CONSOLE_FULL=1) writes at once.
 *  - Wear: each page is erased once per 2 x STORE_SLOTS records, so an automation that changes a stored value all
 *    the time (Ultra CONFIG 22 / 23, config frames: 8 records a day) erases each page every 2 days: >= 54 years at
 *    the datasheet's 10,000 cycles.
 *  - The record also carries a factory reset's pending clears (STORE_VOIDS below). */
#ifndef STORE_H
#define STORE_H

#include <stdint.h>
#include "../board/boards.h"
#include "config.h"

#define STORE_SLOTS       8u
#define STORE_SLOT_BYTES  32u
#define STORE_MAGIC       0x2917u   /* a record of this layout (the CRC last) */
#define STORE_BUDGET_CAP  16u           /* write budget: a console session's worth of changes at once ... */
#define STORE_REFILL_MS   10800000u     /* ... then one token per 3 h (8 a day sustained) */

/* the record's extension: the board current limit (core/isense.h) and options; 0xFF = not set. The current
 * calibration has its own page (core/calstore.h). */
#define STORE_OPT_SOFT_START 0x01u  /* opts: soft start on (core/sstart.h) */
typedef struct {
    uint8_t ilimit50;          /* board current limit, x 50 mA (0 / 0xFF = the board rating) */
    uint8_t opts;              /* STORE_OPT_*, 0xFF = the defaults (on) */
    uint8_t reserved[5];       /* 0xFF ([0]: store_pack_ext's, [1]: STORE_VOIDS) */
} store_ext_t;
/* a factory reset's clears not stored yet: a bit 0 = that store is void, its records from
 * before the reset don't load at power-up (main.c records_load) and its clear is due again; the bit goes back to 1
 * with the store's next write. 0xFF: none. So a power cut at any moment after a reset
 * never brings back what it cleared, though the clears wait for a low moment (an erase) */
#define STORE_VOIDS(e)    ((e)->reserved[1])
#define STORE_VOID_SCENE  0x01u
#define STORE_VOID_CAL    0x02u
#define STORE_VOID_FX     0x04u
#define STORE_VOID_HIST   0x08u

typedef struct {
    uint16_t magic;
    uint8_t format, channels;
    uint32_t    seq;
    settings_t  s;             /* 16 bytes */
    store_ext_t ext;           /* 7 bytes */
    uint8_t     crc;           /* CRC-8 over seq + s + ext */
} store_slot_t;

/* a record with the magic and its CRC (core/pagepair.h's check) */
int  store_valid(const uint8_t *r);
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
