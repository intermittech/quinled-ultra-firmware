/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "pagepair.h"
#include "store.h"

static uint32_t seq_of(const pp_desc_t *d, const uint8_t *r)
{
    const uint8_t *s = r + d->seq_ofs;
    return ((uint32_t)s[0] | (uint32_t)s[1] << 8 | (uint32_t)s[2] << 16 | (uint32_t)s[3] << 24) << d->seq_shl;
}

const uint8_t *pp_find(const pp_desc_t *d, const uint8_t *page0, const uint8_t *page1, pp_pos_t *pos)
{
    const uint8_t *pg[2], *s, *best = 0;
    uint32_t p, k, at = 0, after = 0, bytes = d->page_bytes;   /* the page and entry after the newest */
    if (!d->bytes || !d->recs || d->recs>16u || (uint32_t)d->bytes*d->recs>d->page_bytes) {
        pos->next=0; pos->erase=1; pos->ahead=pos->held=pos->bad=0;
        return 0;
    }
    pg[0] = page0;
    pg[1] = page1;
    pos->bad = 0;
    for (p = 0; p < 2u; p++) {
        for (k = 0, s = pg[p]; k < d->recs; k++, s += d->bytes) {
            if (!d->valid(s)) {
#if MON_EXTRA
                pos->bad = (uint8_t)(pos->bad + !store_erased(s, d->bytes));   /* rejected */
#endif
            } else if (!best || (int32_t)(seq_of(d, s) - seq_of(d, best)) > 0) {
                best = s;
                at = p;
                after = k + 1u;
            }
        }
    }
    if (after != 0u && after < d->recs) {
        /* every entry after it reads erased, not only the next: a write refused for VDD
         * leaves its entry erased and used up, and a later one on the page may be cut short; programmed over, that
         * one corrupts the next record although the store answers FLASH_OK */
        if (store_erased(best + d->bytes, (d->recs - after) * d->bytes)) {
            pos->next = (uint8_t)(at << 4 | after);
            pos->erase = 0;
            pos->ahead = (uint8_t)store_erased(pg[at ^ 1u], bytes);   /* (only older records there) */
            pos->held = 1;
            return best;
        }
        after = d->recs;                                 /* cut short by a power loss: as if full */
    }
    if (after == d->recs) {
        at ^= 1u;                                        /* its page is full: the other one */
    }
    /* a page's first entry (none stored: page 0's): erase the page first unless it reads erased */
    pos->next = (uint8_t)(at << 4);
    pos->erase = (uint8_t)!store_erased(pg[at], bytes);
    pos->ahead = 0;                                      /* (the other page holds the newest, or nothing is stored) */
    pos->held = 0;
    return best;
}

uint32_t pp_plan(const pp_desc_t *d, pp_pos_t *pos)
{
    uint32_t at = pos->next;
    if (pos->erase) {
        return PP_NONE;
    }
    if (PP_ENTRY(at) + 1u == d->recs) {                     /* this fills its page: the other one is next, erased first
                                                            unless it was erased ahead */
        pos->next = (uint8_t)((at ^ 16u) & 16u);
        pos->erase = (uint8_t)!pos->ahead;
        pos->ahead = 0;
    } else {
        pos->next = (uint8_t)(at + 1u);
    }
    return at;
}

void pp_done(pp_pos_t *pos, uint32_t at, uint32_t ok)
{
    if (PP_PAGE(at) == PP_PAGE(pos->next)) {
        pos->held |= (uint8_t)ok;                        /* (a failed entry is used up, never a record) */
    } else if (ok || pos->held) {
        pos->held = 0;                                   /* it filled its page, which holds the newest: the other page
                                                            next (pp_plan), nothing committed there yet */
    } else {
        pos->next = (uint8_t)(at & 16u);                 /* it filled a page that holds no record: the newest is on
                                                            the other one, which must not be erased; this one again */
        pos->erase = 1;
        pos->ahead = 0;
    }
}

uint32_t pp_erase_due(const pp_pos_t *pos, uint32_t *page)
{
    *page = PP_PAGE(pos->next);
    if (pos->erase) {
        return 2;
    }
    *page ^= 1u;                                         /* ahead: the other page, once this one holds the newest */
    return !pos->ahead && pos->held ? 1u : 0u;
}

void pp_erased(pp_pos_t *pos)
{
    if (pos->erase) {
        pos->erase = 0;
    } else {
        pos->ahead = 1;
    }
}

uint32_t pp_free(const pp_desc_t *d, const pp_pos_t *pos)
{
    return (pos->erase ? 0u : d->recs - PP_ENTRY(pos->next)) + (pos->ahead ? d->recs : 0u);
}
