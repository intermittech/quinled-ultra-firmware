/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The A/B page pairs' position after a skipped entry (core/pagepair.c pp_find).
 * test\build_host.cmd builds and runs it (r6_pagepair.exe)
 *
 * A write refused with FLASH_ERR_VDD (VDD below the PVD) programs nothing but uses up its entry (pp_done);
 * a later write on the same page may then be cut short by a power loss. A pp_find that took the erased entry right
 * after the newest record as the next position at the next boot would program the write after it over the cut record:
 * on this flash model (a program only clears bits) the record is corrupt although the store answered FLASH_OK, so the
 * committed record would be lost at the following boot (the chip itself refuses or corrupts such a program). Every
 * store on the shared path (settings, calibration, scene log, lifetime record, calibration history) depends on it.
 * Part 1 replays that sequence for each store; part 2 is a seeded random run of writes, erases, failures (VDD, an
 * operation ending without EOP after k units, a power cut after k units) and reboots, with three invariants: a
 * committed record is never lost, no half-word is programmed that isn't erased, no erase hits the page holding the
 * committed record. With such a pp_find: part 1 fails for every store, part 2 thousands of times. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../src/core/flashrec.h"
#include "../src/core/pagepair.h"
#include "../src/target.h"

__declspec(dllimport) void *__stdcall VirtualAlloc(void *addr, size_t size, unsigned long type, unsigned long prot);

#define FL_BASE 0x08000000u
#define FL_SIZE (RECORDS_END - FL_BASE)
static uint8_t *fl;
static int g_fail;
#define CHECK(c, ...) do { if (!(c)) { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); \
                                       printf("\n"); } } while (0)

/* ---- the flash model: an erase sets the page to 0xFF, a program only clears bits (as NOR flash), and the faults */
enum { F_NONE = 0, F_VDD, F_EOP, F_CUT };
static int f_mode, f_k, cut;
static uint32_t bad_prog, bad_erase, guard_lo, guard_hi;

uint32_t flash_op(uint32_t addr, const uint16_t *src, uint32_t halfwords, uint32_t erase_addr)
{
    uint32_t i;
    if (cut) {
        return FLASH_ERR_EOP;                            /* (the power is gone) */
    }
    if (f_mode == F_VDD) {
        return FLASH_ERR_VDD;                            /* (hw.c: nothing started) */
    }
    if (erase_addr) {
        if (f_mode >= F_EOP && f_k-- <= 0) {
            cut = f_mode == F_CUT;
            return FLASH_ERR_EOP;
        }
        if (guard_hi && erase_addr < guard_hi && erase_addr + 256u > guard_lo) {
            bad_erase++;
        }
        memset(fl + (erase_addr - FL_BASE), 0xFF, 256u);
    }
    for (i = 0; i < halfwords; i++) {
        uint16_t *p = (uint16_t *)(uintptr_t)(addr + 2u * i);
        if (f_mode >= F_EOP && f_k-- <= 0) {
            cut = f_mode == F_CUT;
            return FLASH_ERR_EOP;
        }
        bad_prog += *p != 0xFFFFu;
        *p &= src[i];
    }
    return FLASH_OK;
}

/* ---- the stores, one interface: write value v (16 bits), load (1: found, *v), the erase due / ahead, the pages */
typedef struct {
    const char *name;
    uint32_t a, b, bytes, recs;
    uint32_t (*write)(uint32_t v);
    int (*load)(uint32_t *v);
    uint32_t (*prepare)(void);
} store_if_t;

static uint32_t set_write(uint32_t v)
{
    settings_t s;
    store_ext_t e;
    memset(&s, 0, sizeof s);
    memset(&e, 0, sizeof e);
    settings_default(&s);
    store_ext_clear(&e);
    s.raw_steps = 0x10000u + v;
    return settings_store(&s, &e);
}
static int set_load(uint32_t *v)
{
    settings_t s;
    store_ext_t e;
    if (settings_load(&s, &e) != 0) {
        return 0;
    }
    *v = s.raw_steps - 0x10000u;
    return 1;
}

static uint32_t cal_write(uint32_t v)
{
    isense_cal_t c;
    uint16_t n;
    memset(&c, 0, sizeof c);
    c.levels = 1;
    c.pct[0] = 50;
    c.ma[0][0] = (uint16_t)v;
    c.ma[4][0] = (uint16_t)~v;
    return cal_store(&c, &n);
}
static int cal_ld(uint32_t *v)
{
    isense_cal_t c;
    uint16_t n;
    if (cal_load(&c, &n) != 0) {
        return 0;
    }
    *v = c.ma[0][0];
    return 1;
}

static uint32_t scene_write(uint32_t v)
{
    scene_t sc;
    memset(&sc, 0, sizeof sc);
    sc.mask = 0x11u;
    sc.v[0] = (uint16_t)v;
    sc.v[4] = (uint16_t)~v;
    return scene_store(&sc);
}
static int scene_ld(uint32_t *v)
{
    scene_t sc;
    if (scene_load(&sc) != 0) {
        return 0;
    }
    *v = sc.v[0];
    return 1;
}

static energy_t en;
static uint32_t life_write(uint32_t v)
{
    en.lt.magic = LIFE_MAGIC;
    en.lt.on_s = v;
    en.lt.lit_s = ~v;
    return life_store(&en);
}
static int life_ld(uint32_t *v)
{
    life_rec_t r;
    if (!life_load(&r)) {
        return 0;
    }
    en.lt = r;                                           /* (energy_init: the stored values go on) */
    *v = r.on_s;
    return 1;
}

#if MON_EXTRA
static calhist_t ch;
static uint32_t hist_write(uint32_t v)
{
    ch.tests = (uint16_t)v;
    ch.e[0].lit_h = (uint16_t)~v;
    return calhist_store(&ch);
}
static int hist_ld(uint32_t *v)
{
    calhist_t h;
    if (!calhist_load(&h)) {
        return 0;
    }
    ch = h;
    *v = h.tests;
    return 1;
}
#endif

static const store_if_t stores[] = {
    { "settings", SETTINGS_ADDR, SETTINGS_B_ADDR, STORE_SLOT_BYTES, STORE_SLOTS, set_write, set_load, 0 },
    { "calibration", CAL_PAGE_ADDR, CAL_B_ADDR, CAL_REC_BYTES, CAL_RECS, cal_write, cal_ld, 0 },
    { "scene log", SCENE_LOG_ADDR, SCENE_B_ADDR, SCENE_REC_BYTES, SCENE_RECS_PAGE, scene_write, scene_ld, 0 },
    { "lifetime record", LIFE_PAGE0_ADDR, LIFE_PAGE1_ADDR, LIFE_REC_BYTES, LIFE_RECS, life_write, life_ld, 0 },
#if MON_EXTRA
    { "calibration history", CALHIST_ADDR, CALHIST_B_ADDR, CALHIST_REC_BYTES, CALHIST_RECS, hist_write, hist_ld, 0 },
#endif
};
#define NSTORES (sizeof stores / sizeof stores[0])

static uint32_t prep(uint32_t i)
{
    return i == 0u ? settings_prepare() : i == 1u ? cal_prepare() : i == 2u ? scene_prepare() : i == 3u ? life_prepare()
#if MON_EXTRA
         : calhist_prepare();
#else
         : FLASH_NONE;
#endif
}

static void erase_pair(const store_if_t *s)
{
    memset(fl + (s->a - FL_BASE), 0xFF, 256u);
    memset(fl + (s->b - FL_BASE), 0xFF, 256u);
}

/* ---- part 1: a write, a skipped one (VDD), one cut before its last half-word; reboot; two writes; reboot */
static void part1(void)
{
    uint32_t i, v, r, r4, r5, b0;
    printf("part 1: a skipped entry, a cut write, a reboot, two writes, a reboot: the newest record\n");
    for (i = 0; i < NSTORES; i++) {
        const store_if_t *s = &stores[i];
        erase_pair(s);
        (void)s->load(&v);                               /* (the boot: the positions) */
        r = s->write(101u);
        f_mode = F_VDD;
        (void)s->write(102u);                            /* VDD below the PVD: nothing programmed, the entry used up */
        f_mode = F_CUT;
        f_k = (int)(s->bytes / 2u) - 1;                  /* the power goes before the last half-word (its check) */
        (void)s->write(103u);
        f_mode = F_NONE;
        cut = 0;
        CHECK(s->load(&v) && v == 101u, "%s: the record before the cut (%u)", s->name, v);
        b0 = bad_prog;
        r4 = s->write(104u);
        r5 = s->write(105u);
        CHECK(r == FLASH_OK && r4 == FLASH_OK && r5 == FLASH_OK, "%s: the writes", s->name);
        CHECK(bad_prog == b0, "%s: %u half-words programmed that were not erased", s->name, bad_prog - b0);
        v = 0;
        CHECK(s->load(&v) && v == 105u, "%s: the committed record 105 after the reboot (loaded %u)", s->name, v);
        printf("  %s: loaded %u after the reboot (committed 105)\n", s->name, v);
    }
}

/* ---- part 2: random writes, erases, failures and reboots */
static unsigned long long rs = 0x5DEECE66Dull;
static uint32_t rnd(uint32_t n)
{
    rs ^= rs << 13;
    rs ^= rs >> 7;
    rs ^= rs << 17;
    return (uint32_t)(rs % n);
}

/* where a valid record of store i holding v is (guard_lo; 0: none) */
static uint32_t place_of(uint32_t i, uint32_t v)
{
    const store_if_t *s = &stores[i];
    uint32_t p, k;
    for (p = 0; p < 2u; p++) {
        for (k = 0; k < s->recs; k++) {
            uint32_t a = (p ? s->b : s->a) + k * s->bytes, x = 0;
            const uint8_t *r = (const uint8_t *)(uintptr_t)a;
            int ok;
            switch (i) {
            case 0: ok = store_valid(r); x = ((const store_slot_t *)r)->s.raw_steps - 0x10000u; break;
            case 1: ok = cal_valid(r); x = ((const cal_rec_t *)r)->c.ma[0][0]; break;
            case 2: ok = scene_pp.valid(r); x = ((const scene_rec_t *)r)->s.v[0]; break;
            case 3: ok = life_pp.valid(r); x = ((const life_rec_t *)r)->on_s; break;
#if MON_EXTRA
            default: ok = calhist_valid(r); x = ((const calhist_t *)r)->tests; break;
#else
            default: ok = 0; break;
#endif
            }
            if (ok && x == v) {
                return a;
            }
        }
    }
    return 0;
}

static void part2(uint32_t steps)
{
    uint32_t i, n;
    printf("part 2: %u random steps per store (writes, erases, VDD / EOP / cut failures, reboots)\n", steps);
    for (i = 0; i < NSTORES; i++) {
        const store_if_t *s = &stores[i];
        uint32_t v = 0, val = 0, committed = 0, tried = 0, have = 0, lost = 0, fails0 = (uint32_t)g_fail;
        uint32_t bp0 = bad_prog, be0 = bad_erase;
        erase_pair(s);
        guard_lo = guard_hi = 0;
        (void)s->load(&v);
        for (n = 0; n < steps; n++) {
            uint32_t op = rnd(10);
            f_mode = rnd(4) == 0 ? 1 + (int)rnd(3) : F_NONE;
            f_k = (int)rnd(s->bytes / 2u + 2u);
            if (op < 6) {
                uint32_t r = s->write(++val);
                if (r == FLASH_OK) {
                    committed = val;
                    have = 1;
                }
                if (r != FLASH_NONE) {
                    tried = val;
                }
            } else if (op < 9) {
                (void)prep(i);
            }
            f_mode = F_NONE;
            if (op == 9 || cut) {
                cut = 0;
                if (s->load(&v)) {
                    if (have && (v < committed || v > tried)) {
                        lost++;
                    }
                    if (have) {
                        committed = v;                   /* (a later write that completed is the newest now) */
                    }
                } else {
                    lost += have;
                }
            }
            guard_lo = have ? place_of(i, committed) : 0u;
            guard_hi = guard_lo ? guard_lo + s->bytes : 0u;
            lost += have && !guard_lo;
        }
        guard_lo = guard_hi = 0;
        CHECK(lost == 0u && bad_prog == bp0 && bad_erase == be0, "%s: %u steps with the committed record lost, %u "
              "half-words programmed that were not erased, %u erases of the committed record's page", s->name, lost,
              bad_prog - bp0, bad_erase - be0);
        printf("  %s: %u writes, lost %u, bad programs %u, bad erases %u%s\n", s->name, val, lost, bad_prog - bp0,
               bad_erase - be0, (uint32_t)g_fail != fails0 ? " FAILED" : "");
    }
}

static void page_geometry(void)
{
    uint8_t p0[512],p1[512];
    pp_pos_t pos;
    pp_desc_t d=scene_pp;
    d.page_bytes=512;
    memset(p0,255,sizeof p0); memset(p1,255,sizeof p1);
    p0[511]=0; /* dirty tail beyond every record and beyond a 256-byte page */
    CHECK(pp_find(&d,p0,p1,&pos)==0 && pos.erase,"512-byte page tail must be erased");
    d.page_bytes=16;
    CHECK(pp_find(&d,p0,p1,&pos)==0 && pos.erase,"records cannot exceed page_bytes");
}

int main(void)
{
    page_geometry();
    fl = (uint8_t *)VirtualAlloc((void *)(uintptr_t)FL_BASE, FL_SIZE, 0x3000u, 0x04u);   /* RESERVE|COMMIT, RW */
    if (fl != (uint8_t *)(uintptr_t)FL_BASE) {
        printf("cannot map the flash model at 0x%08X\n", FL_BASE);
        return 2;
    }
    memset(fl, 0xFF, FL_SIZE);
    part1();
    part2(20000u);
    printf("r6 pagepair: %s (%d failures)\n", g_fail ? "FAILED" : "passed", g_fail);
    return g_fail ? 1 : 0;
}
