/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* (round 12, S10) the page-transaction prototype (test/pagetx/pagetx.h) on a flash model with the faults that matter.
 * test\build_host.cmd builds and runs it (pagetx_tests.exe).
 *  1. writes and boots: 3,000 writes on 2, 3 and 4 pages in rotation, a boot (pt_find) after each: the newest always
 *     comes back; nothing is ever programmed into a page that isn't erased
 *  2. a power cut at every point of a write: the page program cut after each of its 256 bytes, cut with random bits
 *     programmed, the erase ahead cut (the page left as anything): the next boot finds the newest committed record or
 *     the one being written (whole), never older, never garbage; then the store writes on
 *  3. the sequence number across 2^32
 *  4. the negative control: an append store (records added to a page that holds older ones, as core/pagepair.h does) on
 *     a flash whose half-word program rewrites its whole page (the bench's open question), cut during that rewrite:
 *     committed records lost; the page-transaction store on the same flash: none
 *  5. the wear: each store's sustained write budget against 10,000 erase cycles a page, appended as today and as page
 *     transactions in rotation */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pagetx.h"
#include "../../src/core/ultra_protocol.h"

static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
                           printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint32_t rs = 0x9E3779B9u;
static uint32_t rnd(void)
{
    rs ^= rs << 13;
    rs ^= rs >> 17;
    rs ^= rs << 5;
    return rs;
}

/* ---- the flash model: 16 pages at a made-up address; an erase sets 0xFF, a program only clears bits */
#define FBASE 0x10000u
#define FPAGES 16u
static uint8_t fl[FPAGES * PT_PAGE];
enum { CUT_NONE = 0, CUT_AT, CUT_BITS, CUT_ERASE };
static int cut_mode, cut_at;
static uint32_t bad_prog, erases[FPAGES];

static uint8_t *at(uint32_t addr)
{
    return fl + (addr - FBASE);
}

static const uint8_t *f_page(uint32_t addr)
{
    return at(addr);
}

static uint32_t f_erase(uint32_t addr)
{
    uint32_t i;
    erases[(addr - FBASE) / PT_PAGE]++;
    if (cut_mode == CUT_ERASE) {
        for (i = 0; i < PT_PAGE; i++) {
            at(addr)[i] &= (uint8_t)rnd();               /* (cut: any content) */
        }
        cut_mode = CUT_NONE;
        return 3u;                                       /* (FLASH_ERR_EOP) */
    }
    memset(at(addr), 0xFF, PT_PAGE);
    return PT_OK;
}

static uint32_t f_program(uint32_t addr, const uint8_t *g)
{
    uint8_t *p = at(addr);
    uint32_t i;
    for (i = 0; i < PT_PAGE; i++) {
        bad_prog += p[i] != 0xFFu;
    }
    for (i = 0; i < PT_PAGE; i++) {
        if (cut_mode == CUT_AT && (int)i == cut_at) {
            cut_mode = CUT_NONE;
            return 3u;
        }
        p[i] &= cut_mode == CUT_BITS ? (uint8_t)(g[i] | rnd() | rnd()) : g[i];   /* (bits: some of its 0s land) */
    }
    if (cut_mode == CUT_BITS) {
        cut_mode = CUT_NONE;
        return 3u;
    }
    return PT_OK;
}

static const pt_flash_t FL = { f_erase, f_program, f_page };

static void rec_make(uint8_t *r, uint32_t n, uint32_t v)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        r[i] = (uint8_t)(v * 31u + i * 7u + (v >> 8));
    }
    r[0] = (uint8_t)v; r[1] = (uint8_t)(v >> 8); r[2] = (uint8_t)(v >> 16); r[3] = (uint8_t)(v >> 24);
}

static uint32_t rec_val(const uint8_t *r)
{
    return (uint32_t)r[0] | (uint32_t)r[1] << 8 | (uint32_t)r[2] << 16 | (uint32_t)r[3] << 24;
}

/* a boot: the newest's value (0xFFFFFFFF: none); also checks that the record is whole */
static uint32_t boot(const pt_desc_t *d, pt_pos_t *p)
{
    uint8_t want[PT_PAYLOAD];
    const uint8_t *r = pt_find(d, &FL, p);
    if (!r) {
        return 0xFFFFFFFFu;
    }
    rec_make(want, d->bytes, rec_val(r));
    CHECK(memcmp(r, want, d->bytes) == 0, "a boot: record %u whole", rec_val(r));
    return rec_val(r);
}

/* a write as main.c would: the erase ahead (in a low moment) when due, then the write */
static uint32_t put(const pt_desc_t *d, pt_pos_t *p, uint32_t v)
{
    uint8_t r[PT_PAYLOAD];
    if (pt_pending(p) == 2u && pt_prepare(d, &FL, p) != PT_OK) {
        return 0;
    }
    rec_make(r, d->bytes, v);
    return pt_write(d, &FL, p, r);
}

static void test_rotation(void)
{
    uint32_t k, n, v, lost = 0, e, emax, emin;
    printf("pagetx 1: 3,000 writes and a boot after each, 2-4 pages in rotation\n");
    for (n = 2; n <= 4u; n++) {
        pt_desc_t d = { FBASE, (uint8_t)n, 48u };
        pt_pos_t p;
        memset(fl, 0xFF, sizeof fl);
        memset(erases, 0, sizeof erases);
        bad_prog = 0;
        CHECK(boot(&d, &p) == 0xFFFFFFFFu && p.erased, "%u pages: an erased store: none, the first page erased", n);
        for (v = 1; v <= 3000u; v++) {
            CHECK(put(&d, &p, v) == PT_OK, "%u pages: write %u", n, v);
            lost += boot(&d, &p) != v;
        }
        emax = 0;
        emin = 0xFFFFFFFFu;
        for (k = 0; k < n; k++) {
            e = erases[k];
            emax = e > emax ? e : emax;
            emin = e < emin ? e : emin;
        }
        printf("  %u pages: %u writes, %u lost, %u programs onto data; erases a page %u..%u\n", n, v - 1u, lost,
               bad_prog, emin, emax);
        CHECK(lost == 0u && bad_prog == 0u && emax - emin <= 1u, "%u pages: the newest after every write, every "
              "program into erased flash, the wear even (%u lost, %u bad, erases %u..%u)", n, lost, bad_prog, emin,
              emax);
    }
}

static void test_cuts(void)
{
    pt_desc_t d = { FBASE, 3u, 200u };
    pt_pos_t p;
    uint32_t k, t, base, got, older = 0, newer = 0, kept = 0, garb = 0;
    printf("pagetx 2: a power cut at every byte of a page program, with random bits, and in the erase ahead\n");
    for (k = 0; k <= PT_PAGE; k++) {                     /* (k = PT_PAGE: not cut) */
        memset(fl, 0xFF, sizeof fl);
        boot(&d, &p);
        for (base = 1; base <= 7u; base++) {
            put(&d, &p, base);
        }
        cut_mode = k < PT_PAGE ? CUT_AT : CUT_NONE;
        cut_at = (int)k;
        put(&d, &p, 8u);
        got = boot(&d, &p);
        CHECK(got == (k < PT_PAGE ? 7u : 8u), "cut after byte %u: the newest committed (%u)", k, got);
        CHECK(put(&d, &p, 9u) == PT_OK && boot(&d, &p) == 9u, "cut after byte %u: writes on", k);
    }
    for (t = 0; t < 20000u; t++) {
        memset(fl, 0xFF, sizeof fl);
        boot(&d, &p);
        base = 1u + rnd() % 9u;
        for (k = 1; k <= base; k++) {
            put(&d, &p, k);
        }
        cut_mode = (t & 3u) == 3u ? CUT_ERASE : CUT_BITS;
        if (cut_mode == CUT_ERASE && pt_pending(&p) != 2u) {
            cut_mode = CUT_BITS;                         /* (no erase due: cut the program instead) */
        }
        put(&d, &p, base + 1u);
        cut_mode = CUT_NONE;
        got = boot(&d, &p);
        older += got != 0xFFFFFFFFu && got < base;
        garb += got == 0xFFFFFFFFu || got > base + 1u;
        kept += got == base;
        newer += got == base + 1u;
        CHECK(put(&d, &p, base + 2u) == PT_OK && boot(&d, &p) == base + 2u, "trial %u: writes on", t);
    }
    printf("  every byte: the newest committed, then writes on; 20,000 random cuts: %u kept the newest, %u the "
           "record being written, %u older, %u garbage or none\n", kept, newer, older, garb);
    CHECK(older == 0u && garb == 0u, "random cuts: never older, never garbage (%u, %u)", older, garb);
}

static void test_wrap(void)
{
    pt_desc_t d = { FBASE, 4u, 16u };
    pt_pos_t p;
    uint32_t v, lost = 0;
    printf("pagetx 3: the sequence number across 2^32\n");
    memset(fl, 0xFF, sizeof fl);
    boot(&d, &p);
    p.seq = 0xFFFFFFF0u;                                 /* (a long life) */
    for (v = 1; v <= 40u; v++) {
        put(&d, &p, v);
        lost += boot(&d, &p) != v;
    }
    printf("  40 writes from seq 0xFFFFFFF0: %u lost, seq now %u\n", lost, p.seq);
    CHECK(lost == 0u && p.seq == 0x18u, "across 2^32: the newest (%u lost, seq %u)", lost, p.seq);
}

/* ---- test 4's append store: 2 pages of 8 records of 32 bytes (value 4 | filler | CRC-16), appended half-word by
 * half-word as core/pagepair.h does; on a "rewrite" flash a half-word program reads the page, erases it and programs
 * it back with the half-word changed: a cut then leaves the page as anything */
#define AP_REC 32u
static uint32_t ap_newest(void)
{
    uint32_t pg, i, best = 0xFFFFFFFFu;
    for (pg = 0; pg < 2u; pg++) {
        for (i = 0; i < PT_PAGE / AP_REC; i++) {
            const uint8_t *r = fl + pg * PT_PAGE + i * AP_REC;
            if (ultra_crc16(r, AP_REC - 2u) == (uint16_t)(r[AP_REC - 2u] << 8 | r[AP_REC - 1u]) &&
                (best == 0xFFFFFFFFu || rec_val(r) > best)) {
                best = rec_val(r);
            }
        }
    }
    return best;
}

static void ap_append(uint32_t v, int cut_at_half)
{
    uint8_t r[AP_REC];
    uint32_t n = v - 1u, pg = n / 8u % 2u, slot = n % 8u, i, c;
    uint8_t *page = fl + pg * PT_PAGE;
    if (slot == 0u) {
        memset(page, 0xFF, PT_PAGE);                     /* (the next page: erased when the store enters it) */
    }
    rec_make(r, AP_REC - 2u, v);
    c = ultra_crc16(r, AP_REC - 2u);
    r[AP_REC - 2u] = (uint8_t)(c >> 8);
    r[AP_REC - 1u] = (uint8_t)c;
    for (i = 0; i < AP_REC / 2u; i++) {
        if ((int)i == cut_at_half) {
            uint32_t j;
            for (j = 0; j < PT_PAGE; j++) {
                page[j] &= (uint8_t)rnd();               /* (the cut during the page's rewrite: anything) */
            }
            return;
        }
        page[slot * AP_REC + 2u * i] &= r[2u * i];
        page[slot * AP_REC + 2u * i + 1u] &= r[2u * i + 1u];
    }
}

static void test_rewrite(void)
{
    uint32_t t, v, lost_ap = 0, lost_pt = 0, got;
    pt_desc_t d = { FBASE, 2u, 28u };
    pt_pos_t p;
    printf("pagetx 4: a half-word program that rewrites its page (the bench's question), cut mid-rewrite\n");
    for (t = 0; t < 2000u; t++) {
        uint32_t n = 3u + rnd() % 12u;                   /* records before the cut: page 0 full or not, page 1 begun */
        memset(fl, 0xFF, sizeof fl);
        for (v = 1; v <= n; v++) {
            ap_append(v, -1);
        }
        ap_append(n + 1u, (int)(rnd() % (AP_REC / 2u)));
        got = ap_newest();
        lost_ap += got == 0xFFFFFFFFu || got < n;        /* (n: the newest committed) */
        memset(fl, 0xFF, sizeof fl);
        boot(&d, &p);
        for (v = 1; v <= n; v++) {
            put(&d, &p, v);
        }
        if (pt_pending(&p) == 2u) {
            pt_prepare(&d, &FL, &p);                     /* (the erase ahead: a page of its own, never the newest's) */
        }
        cut_mode = CUT_BITS;                             /* (the one whole-page program, cut mid-rewrite: that
                                                            page only, erased ahead) */
        put(&d, &p, n + 1u);
        got = boot(&d, &p);
        lost_pt += got == 0xFFFFFFFFu || got < n;
    }
    printf("  2,000 cuts: the append store lost the newest committed record %u times, the page-transaction store "
           "%u\n", lost_ap, lost_pt);
    CHECK(lost_ap > 0u && lost_pt == 0u, "the control loses records (%u), the transactions never (%u)", lost_ap,
          lost_pt);
}

static void test_wear(void)
{
    static const struct { const char *name; double per_day; uint32_t recs_page; } st[] = {
        { "settings (8 a day: STORE_REFILL_MS)", 8.0, 8u },
        { "calibration (4 a day: CAL_REFILL_MS)", 4.0, 4u },
        { "power-on scene log (72 a day: SCENE_REFILL_MS)", 72.0, 16u },
        { "lifetime record (8 a day: LIFE_REFILL_MS)", 8.0, 5u },
        { "calibration history (4 a day)", 4.0, 4u },
    };
    static const uint32_t rot[] = { 2u, 4u, 8u, 16u, 32u };
    uint32_t i, k;
    printf("pagetx 5: years to 10,000 erases a page at each store's sustained budget: appended today (2 pages), and "
           "as page transactions over n pages (one erase a write)\n");
    printf("  %-48s %8s", "store", "today");
    for (k = 0; k < 5u; k++) {
        printf("   n=%-3u", rot[k]);
    }
    printf("\n");
    for (i = 0; i < 5u; i++) {
        printf("  %-48s %8.1f", st[i].name, 10000.0 / (st[i].per_day / (st[i].recs_page * 2.0)) / 365.25);
        for (k = 0; k < 5u; k++) {
            printf(" %7.1f", 10000.0 * rot[k] / st[i].per_day / 365.25);
        }
        printf("\n");
    }
}

int main(void)
{
    test_rotation();
    test_cuts();
    test_wrap();
    test_rewrite();
    test_wear();
    printf("pagetx: %u checks, %u failed\n", checks, fails);
    return fails != 0;
}
