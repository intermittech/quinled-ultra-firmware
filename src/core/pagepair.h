/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* An A/B pair of flash pages holding fixed-size records (0.6.4: the 0.6.3 lifetime record's scheme, made generic for
 * the settings and the current calibration too).
 *
 * Records are appended; a write never erases. When a write fills its page, the other page is next: it must be erased
 * first (pending), unless it was erased ahead. The page holding the newest record is never erased, so a power cut at
 * any moment leaves the last complete record or the one being written (its check last: a cut record never counts).
 * At boot, pp_find takes the valid record with the newest sequence number (a signed difference: valid across the
 * wrap; 32-bit numbers, or 16-bit ones with seq_shl 16) and the position after it; a record cut short there makes its
 * page count as full.
 *
 * Erases (main.c, at power-up or in a dark / dim moment, never inside a write): the one that is due (pos->erase), or
 * one ahead: the other page, once the newest record is on this one (it holds only older records), so writes while the
 * outputs are bright (programs only) find erased entries on both pages. Each page is still erased once per round of
 * 2 x recs writes: no extra wear.
 *
 * Positions: page << 4 | entry (entry 0..recs - 1, recs <= 16). The power-on scene log uses it too (0.6.4: 16 records
 * of 16 bytes a page, 16-bit sequence numbers). */
#ifndef PAGEPAIR_H
#define PAGEPAIR_H

#include <stdint.h>

typedef struct {
    uint8_t  bytes;                     /* record size (even, <= 255) */
    uint8_t  recs;                      /* records per page (1..16) */
    uint8_t  seq_ofs;                   /* offset of the sequence number (read as 4 bytes) */
    uint8_t  seq_shl;                   /* 0: a u32 sequence number; 16: a u16 (the 2 bytes after it shift out) */
    int    (*valid)(const uint8_t *r);  /* magic and check ok */
} pp_desc_t;

typedef struct {                        /* where the next record goes */
    uint8_t next;                       /* page << 4 | entry */
    uint8_t erase;                      /* its page must be erased first */
    uint8_t ahead;                      /* the other page reads erased (erased ahead) */
} pp_pos_t;
#define PP_NONE 0xFFu
#define PP_PAGE(at)  ((uint32_t)(at) >> 4)   /* a position's page (0 / 1) */
#define PP_ENTRY(at) ((uint32_t)(at) & 15u)  /* ... and entry */

/* the two pages: returns the newest valid record (NULL: none) and where the next one goes (the entry after it when
 * that reads erased, else the other page's first: erased first unless it reads erased) */
const uint8_t *pp_find(const pp_desc_t *d, const uint8_t *page0, const uint8_t *page1, pp_pos_t *pos);
/* a write at pos: returns where (page << 4 | entry), or PP_NONE while the next page must be erased first. Moves pos
 * on (a write that fills its page: the other page next, erased first unless it was erased ahead). */
uint32_t pp_plan(const pp_desc_t *d, pp_pos_t *pos);
/* the erase to do: 2 = due (no write before it), 1 = possible ahead, 0 = none; *page: which page (0 / 1) */
uint32_t pp_erase_due(const pp_pos_t *pos, uint32_t *page);
/* ... it was done */
void     pp_erased(pp_pos_t *pos);
/* erased entries left for writes without an erase */
uint32_t pp_free(const pp_desc_t *d, const pp_pos_t *pos);

#endif
