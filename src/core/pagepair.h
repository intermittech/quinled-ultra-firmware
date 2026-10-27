/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* A/B page pairs with explicit page_bytes and fixed-size native records.
 * Append only to erased entries. Never erase the page holding the newest valid
 * record before another complete record exists. A torn entry is consumed;
 * sequence comparison is wrap-safe. Full-page erased checks include unused tail
 * bytes. Positions encode page << 4 | entry, with 1..16 entries per page.
 * The application independently enforces dark-only admission for all writes. */
#ifndef PAGEPAIR_H
#define PAGEPAIR_H

#include <stdint.h>

typedef struct {
    uint16_t page_bytes;
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
    uint8_t bad;                        /* pp_find: entries neither erased nor valid (a record cut by a
                                           power loss, a corrupted one, one of another layout) */
    uint8_t held;                       /* next's page holds a committed record (the newest): erase ahead */
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
/* the write at `at` (pp_plan's) is done: ok = programmed (FLASH_OK). A failed one never holds the newest record:
 * if it filled a page that holds none, that page is erased and used again (the other page keeps the newest) */
void     pp_done(pp_pos_t *pos, uint32_t at, uint32_t ok);
/* the erase to do: 2 = due (no write before it), 1 = possible ahead, 0 = none; *page: which page (0 / 1) */
uint32_t pp_erase_due(const pp_pos_t *pos, uint32_t *page);
/* ... it was done */
void     pp_erased(pp_pos_t *pos);
/* erased entries left for writes without an erase */
uint32_t pp_free(const pp_desc_t *d, const pp_pos_t *pos);

#endif
