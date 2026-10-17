/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The page-transaction store: see pagetx.h (a prototype, not wired in). */
#include <string.h>
#include "pagetx.h"
#include "../../src/core/ultra_update.h"     /* upd_crc32_words (CRC-32/MPEG-2, as the CH32's CRC unit) */

static uint32_t pt_addr(const pt_desc_t *d, uint32_t i)
{
    return d->base + i * PT_PAGE;
}

static uint32_t get32(const uint8_t *b)
{
    return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24;
}

static void put32(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
    b[2] = (uint8_t)(v >> 16);
    b[3] = (uint8_t)(v >> 24);
}

/* a page is a valid transaction of this store: the magic, the record's length, the CRC over everything before it */
static int pt_valid(const pt_desc_t *d, const uint8_t *g)
{
    return get32(g) == PT_MAGIC && (g[8] | g[9] << 8) == d->bytes &&
           upd_crc32_words(0xFFFFFFFFu, g, PT_PAGE / 4u - 1u) == get32(g + PT_PAGE - 4u);
}

static int pt_blank(const uint8_t *g)
{
    uint32_t i;
    for (i = 0; i < PT_PAGE; i++) {
        if (g[i] != 0xFFu) {
            return 0;
        }
    }
    return 1;
}

const uint8_t *pt_find(const pt_desc_t *d, const pt_flash_t *f, pt_pos_t *p)
{
    uint32_t i;
    const uint8_t *best = 0;
    p->cur = -1;
    p->seq = 0;
    for (i = 0; i < d->pages; i++) {
        const uint8_t *g = f->page(pt_addr(d, i));
        uint32_t s = get32(g + 4);
        if (pt_valid(d, g) && (best == 0 || (int32_t)(s - p->seq) > 0)) {
            best = g;
            p->seq = s;
            p->cur = (int8_t)i;
        }
    }
    p->next = (uint8_t)(p->cur < 0 ? 0u : ((uint32_t)p->cur + 1u) % d->pages);
    p->erased = (uint8_t)pt_blank(f->page(pt_addr(d, p->next)));
    return best ? best + PT_HDR : 0;
}

uint32_t pt_pending(const pt_pos_t *p)
{
    return p->erased ? 0u : 2u;
}

uint32_t pt_prepare(const pt_desc_t *d, const pt_flash_t *f, pt_pos_t *p)
{
    uint32_t r;
    if (p->erased) {
        return PT_NONE;
    }
    r = f->erase(pt_addr(d, p->next));      /* (never the newest: next != cur with >= 2 pages) */
    p->erased = (uint8_t)(r == PT_OK);
    return r;
}

uint32_t pt_write(const pt_desc_t *d, const pt_flash_t *f, pt_pos_t *p, const void *rec)
{
    uint8_t g[PT_PAGE];
    uint32_t s = p->seq + 1u, r, c;
    if (!p->erased) {
        return PT_NONE;                     /* (the erase ahead first: only while low, as the stores' today) */
    }
    memset(g, 0xFF, sizeof g);
    put32(g, PT_MAGIC);
    put32(g + 4, s);
    g[8] = (uint8_t)d->bytes;
    g[9] = (uint8_t)(d->bytes >> 8);
    memcpy(g + PT_HDR, rec, d->bytes);
    c = upd_crc32_words(0xFFFFFFFFu, g, PT_PAGE / 4u - 1u);
    put32(g + PT_PAGE - 4u, c);
    r = f->program(pt_addr(d, p->next), g);
    if (r != PT_OK) {
        p->erased = 0;                      /* (torn: erased again before its next use; the newest unchanged) */
        return r;
    }
    p->cur = (int8_t)p->next;
    p->seq = s;
    p->next = (uint8_t)((p->next + 1u) % d->pages);
    p->erased = (uint8_t)pt_blank(f->page(pt_addr(d, p->next)));   /* (an older version, or never used yet) */
    return r;
}
