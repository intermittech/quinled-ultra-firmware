/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* A page-transaction store: a host prototype, not wired into the firmware.
 *
 * The fallback if the bench finds that the CH32V203's half-word programming rewrites the whole 256-byte page (README,
 * a bench check: "Half-word programming semantics"): then every record appended to a page (core/pagepair.h) would
 * wear the page, and a power cut during an append could damage the page's other records. Here a write never programs
 * into a page that holds data:
 *  - each write is one transaction: a whole page (the fast 256-byte page program) into a page erased ahead, holding
 *    the store's newest record, its sequence number and a CRC over the page;
 *  - the pages rotate (PT_PAGES_MIN or more), so the newest is never erased: the page erased ahead is always an older
 *    one, and a cut while one is written leaves that page invalid (its CRC) and every older one intact;
 *  - the load takes the valid page with the highest sequence number (modulo 2^32).
 * The erases stay where the firmware has them today (core/flashrec.h: only in a low moment, ahead of the write): a
 * write while bright is a page program into erased flash only. The cost: one erase per write (an append store erases
 * once per page of records), spread over the pages in rotation (test_pagetx.c prints the wear per store).
 *
 * The flash: two operations the caller provides (mcu/ch32v20x/hw.c flash_op's, once wired in), and the pages read as memory. */
#ifndef PAGETX_H
#define PAGETX_H

#include <stdint.h>

#define PT_PAGE        256u         /* a fast-programming page */
#define PT_PAGES_MIN   2u           /* the newest + one erased ahead */
#define PT_MAGIC       0x31585450u  /* "PTX1" */
#define PT_HDR         12u          /* magic (4) | seq (4) | the record's length (2) | 0xFFFF (2) */
#define PT_PAYLOAD     (PT_PAGE - PT_HDR - 4u)   /* the record's bytes at most: 240 (the CRC-32 last) */
/* the CRC: ultra_update.h upd_crc32_words over the page's first 63 words (the CH32's CRC unit computes it), its
 * last word. A CRC-32, not the records' CRC-16: a write cut short leaves the CRC word erased, so the chance that a
 * torn page reads as valid is 2^-32 (a CRC-16's 2^-16 is hit in a sweep of a few hundred thousand cuts) */

#define PT_OK          1u           /* (core/flashrec.h FLASH_OK) */
#define PT_NONE        0u           /* nothing done: the next page isn't erased (pt_prepare first) */

typedef struct {
    uint32_t (*erase)(uint32_t addr);                         /* erase the page at addr: PT_OK or an error */
    uint32_t (*program)(uint32_t addr, const uint8_t *page);  /* program the whole erased page: PT_OK or an error */
    const uint8_t *(*page)(uint32_t addr);                    /* the page as memory (the target: the address) */
} pt_flash_t;

typedef struct {
    uint32_t base;                  /* the first page's address (the pages follow at PT_PAGE) */
    uint8_t  pages;                 /* in rotation, >= PT_PAGES_MIN */
    uint16_t bytes;                 /* the record's bytes, <= PT_PAYLOAD */
} pt_desc_t;

typedef struct {
    uint32_t seq;                   /* the newest's sequence number (valid when cur >= 0) */
    int8_t   cur;                   /* the newest's page, -1: none */
    uint8_t  next;                  /* the page the next write goes into */
    uint8_t  erased;                /* that page is erased (at boot: found so; after pt_prepare) */
} pt_pos_t;

/* at boot: the newest valid record (NULL: none) and the position */
const uint8_t *pt_find(const pt_desc_t *d, const pt_flash_t *f, pt_pos_t *p);
/* the next page's erase: 2 = due before the next write (it holds an older version or a torn write), 0 = none */
uint32_t pt_pending(const pt_pos_t *p);
/* ... that erase (PT_OK, an error, or PT_NONE when none is due) */
uint32_t pt_prepare(const pt_desc_t *d, const pt_flash_t *f, pt_pos_t *p);
/* a write: the record into the erased next page (PT_OK; PT_NONE: not erased yet; else the program's error, the page
 * then torn: due for an erase again, the newest unchanged) */
uint32_t pt_write(const pt_desc_t *d, const pt_flash_t *f, pt_pos_t *p, const void *rec);

#endif
