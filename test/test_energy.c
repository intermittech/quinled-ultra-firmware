/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Input energy and the lifetime record (core/energy.c), included by test_main.c.
 *  - the integration (24 V x 2 A for an hour = 48 Wh, no drift, no overflow at 75 V x 20 A);
 *  - when the record is written: a dip (>= 1 Wh new, a counter event, or 10 min powered), 30 s after the light goes
 *    dark, every 12 h with something new, powered time alone after a day; never while it only idles; a dip storm;
 *  - the record: every field, the check, both erased patterns (0xFFFF / 0xE339), the next entry, the page pair: a
 *    write never erases, the next page is erased while dark (at power-up, or a second after the light went off);
 *  - flash only while dark: a write due while lit waits (no token, no entry) for the first dark second; a dip while
 *    lit writes only if dips are admitted;
 *  - power cuts: a write cut after each of its half-words (24: format 2), at every place of the page pair; an erase cut short (the
 *    old page half erased, or garbage); every step of a 30-record run cut in turn (both erased patterns, the writes
 *    counter across its 32-bit wrap), each image booted, the boot's own erase cut too: always the last complete
 *    record or the one being written, never an older one or none; nothing programmed that isn't erased;
 *  - the energy across the since-boot mWh counter's 2^32 wrap (466 days at 384 W): no step, the remainder exact;
 *  - an update boot counts once (also when the power goes before it is stored), a new board's first version is no
 *    update, development builds are never compared; watchdog resets; power-ups;
 *  - the write budget and the lifetime allowance; the wraps (the ms counter, lit_ms dark for 60 days);
 *  - the wear over simulated years: a flash model of the two pages counting every erase (and those while lit), the
 *    real code booting and writing as on the board, for the worst patterns (always powered, power cycled) and
 *    everyday ones;
 *  - the ENERGY page. */
#include "../src/core/energy.h"

#define EN_TEMP_NA (-32768)

/* ---- a model of the two flash pages: hw.c's life_load / life_store / life_prepare around life_find / life_plan ---- */
typedef struct {
    union { uint8_t b[256]; uint32_t w[64]; } pg[2];
    uint32_t erases[2];
    uint32_t lit_erases;                                 /* erases while an output was on */
    uint32_t writes;                                     /* records programmed (complete or not) */
    uint32_t bad;                                        /* half-words programmed that weren't erased */
    uint16_t erased;                                     /* the erased half-word: 0xFFFF or 0xE339 */
    life_pos_t pos;
} lflash_t;

static void lf_erase(lflash_t *f, uint32_t p)
{
    uint32_t i;
    for (i = 0; i < 256u; i += 2u) {
        f->pg[p].b[i] = (uint8_t)f->erased;
        f->pg[p].b[i + 1u] = (uint8_t)(f->erased >> 8);
    }
    f->erases[p]++;
}

static void lf_init(lflash_t *f, uint16_t erased)
{
    memset(f, 0, sizeof *f);
    f->erased = erased;
    lf_erase(f, 0);
    lf_erase(f, 1);
    f->erases[0] = f->erases[1] = 0;
}

static uint32_t lf_load(lflash_t *f, life_rec_t *r)
{
    return life_find(f->pg[0].b, f->pg[1].b, r, &f->pos);
}

#define LH (LIFE_REC_BYTES / 2u)   /* the record's half-words (0.6.9, format 2: 24) */

/* life_store: `cut` = the half-words programmed before the power goes (LH = all) */
static uint32_t lf_store(lflash_t *f, energy_t *e, uint32_t cut)
{
    uint32_t at = life_plan(&f->pos), i;
    uint8_t *d;
    const uint8_t *s = (const uint8_t *)&e->lt;
    if (at == LIFE_PLAN_NONE) {
        return 0;
    }
    energy_pack(e);
    d = f->pg[(at >> 4) & 1u].b + (at & 15u) * LIFE_REC_BYTES;
    for (i = 0; i < LIFE_REC_BYTES / 2u && i < cut; i++) {
        uint16_t h = (uint16_t)(d[2u * i] | d[2u * i + 1u] << 8);
        f->bad += h != f->erased;                        /* (the chip would refuse it) */
        d[2u * i] = s[2u * i];
        d[2u * i + 1u] = s[2u * i + 1u];
    }
    f->writes++;
    pp_done(&f->pos, at, cut >= LIFE_REC_BYTES / 2u);   /* (0.6.6: as flashrec.c life_store) */
    return cut >= LIFE_REC_BYTES / 2u;
}

/* life_prepare: the pending erase of the next page (lit: an output on meanwhile) */
static void lf_prepare(lflash_t *f, uint32_t lit)
{
    if (f->pos.erase) {
        lf_erase(f, (f->pos.next >> 4) & 1u);
        f->pos.erase = 0;
        f->lit_erases += lit != 0u;
    }
}

/* ---- a board: its time bases, inputs, and the flash ---- */
typedef struct {
    energy_t e;
    lflash_t f;
    uint32_t ms, s, sub;                                 /* ms_now, up_s, ms into the second */
    uint32_t mv;
    int32_t  ma;
    uint8_t  lit;
    uint16_t dips;                                       /* vdip.count (since boot) */
    uint32_t trips;                                      /* pwrev: short << 16 | overload (since boot) */
    int32_t  i_avg, temp;
    uint32_t nw, ndip;                                   /* writes / dip writes since boot */
    uint32_t boots;
    uint32_t mj;                                         /* bd_fast: energy below one mWh, mJ */
    uint8_t  no_dip;                                     /* (0.6.3's FLASH_ADMIT_DIP 0) no dip write while lit */
    uint32_t lit_saves, lit_dips;                        /* regular writes / dip writes while lit */
} ebd_t;

/* 0.6.3's admission (main.c flash_admit then): dark = no output on for a second. 0.6.4's flash_may admits more (a low
 * moment, one write an hour while brighter, a dip): the same energy_due decisions and budgets, so these wear runs
 * bound 0.6.4's writes too (its extra writes while lit come out of the same tokens) */
static uint32_t bd_dark(const ebd_t *b)
{
    return b->ms - b->e.lit_ms >= 1000u;
}

static uint32_t bd_admit(const ebd_t *b)
{
    return b->f.pos.erase ? 0u : bd_dark(b) ? ENERGY_DUE_SAVE | ENERGY_DUE_DIP : b->no_dip ? 0u : ENERGY_DUE_DIP;
}

/* main.c's erase, once a second: while dark */
static void bd_erase_check(ebd_t *b)
{
    if (b->f.pos.erase && b->s >= LIFE_PREP_S && bd_dark(b)) {
        lf_prepare(&b->f, 0);
    }
}

static void bd_boot(ebd_t *b, uint32_t ms0, uint32_t ver, uint32_t wd)
{
    life_rec_t r;
    uint32_t found = lf_load(&b->f, &r);
    if (found) {
        b->e.lt = r;
    }
    b->ms = ms0;
    b->s = b->sub = 0;
    b->dips = 0;
    b->trips = 0;
    b->i_avg = 0;
    b->temp = EN_TEMP_NA;
    b->nw = b->ndip = 0;
    b->boots++;
    energy_init(&b->e, found, ms0, ver, wd);
    lf_prepare(&b->f, 0);                                /* main.c: before the outputs start */
}

static void bd_write(ebd_t *b, uint32_t w)
{
    uint32_t lit = !bd_dark(b);
    if (w != ENERGY_DUE_NONE && lf_store(&b->f, &b->e, LH)) {
        energy_saved(&b->e, b->s);
        b->nw++;
        b->ndip += w == ENERGY_DUE_DIP;
        b->lit_saves += lit && w == ENERGY_DUE_SAVE;
        b->lit_dips += lit && w == ENERGY_DUE_DIP;
    }
}

/* n ms, as main.c: every ms the energy and energy_due, every 1000 ms the uptime, energy_second and the erase check */
static void bd_ms(ebd_t *b, uint32_t n)
{
    while (n--) {
        b->ms++;
        energy_ms(&b->e, b->ms, b->mv, b->ma, b->lit);
        if (++b->sub == 1000u) {
            b->sub = 0;
            b->s++;
            energy_second(&b->e, b->ms, b->s, b->trips, b->i_avg, b->temp);
            bd_erase_check(b);
        }
        bd_write(b, energy_due(&b->e, b->ms, b->dips, bd_admit(b)));
    }
}

/* fast: n steps of dt seconds at mw (the energy added directly; lit_s counts steps, not seconds) */
static void bd_fast(ebd_t *b, uint32_t n, uint32_t dt, uint32_t mw)
{
    while (n--) {
        b->ms += 1000u * dt;
        b->s += dt;
        b->mj += mw * dt;                                /* mW x s = mJ */
        while (b->mj >= 3600u) {
            b->mj -= 3600u;
            b->e.boot_mwh++;
        }
        if (b->lit) {
            b->e.lit_ms = b->ms;
            b->e.lit_new = 1;
        }
        energy_second(&b->e, b->ms, b->s, b->trips, b->i_avg, b->temp);
        bd_erase_check(b);
        bd_write(b, energy_due(&b->e, b->ms, b->dips, bd_admit(b)));
    }
}

static void bd_dip(ebd_t *b)
{
    b->dips++;
    bd_write(b, energy_due(&b->e, b->ms, b->dips, bd_admit(b)));
}

/* the record as the next boot reads it */
static life_rec_t bd_stored(ebd_t *b, uint32_t *found)
{
    life_rec_t r;
    life_pos_t pos = b->f.pos;
    memset(&r, 0, sizeof r);
    *found = lf_load(&b->f, &r);
    b->f.pos = pos;
    return r;
}

/* ---- the integration and the write rules ---- */
static void test_energy_rules(void)
{
    static ebd_t b;
    life_rec_t r;
    uint32_t found, n, k;

    printf("energy: integration, the write rules\n");
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    bd_boot(&b, 0, 0x00060300u, 0);
    CHECK(b.e.lt.ups == 1u && b.e.lt.updates == 0u && b.e.lt.ver == energy_ver(0x00060300u) && b.e.ev,
          "a new board: 1 power-up, no update, its version");
    b.mv = 24000;
    b.ma = 2000;
    b.lit = 1;
    bd_ms(&b, 3600000u);
    CHECK(b.e.boot_mwh == 48000u && energy_total_wh(&b.e) == 48u, "24 V x 2 A for 1 h: %u mWh", b.e.boot_mwh);
    CHECK(b.nw == 0, "lit: not written within the first hour (%u)", b.nw);
    bd_ms(&b, 11u * 3600000u);
    CHECK(b.nw == 0 && b.e.due && b.e.budget.tokens == LIFE_BUDGET_CAP, "lit for 12 h: due, but not while lit (no "
          "token used: %u)", b.e.budget.tokens);
    bd_ms(&b, 2u * 86400000u);                           /* 2 more days lit: it waits, everything in RAM */
    CHECK(b.nw == 0 && b.e.budget.tokens == LIFE_BUDGET_CAP && b.f.pos.next == 0u && energy_total_wh(&b.e) == 2880u,
          "lit 2 days more: still waiting, no token, no entry used (%u Wh in RAM)", energy_total_wh(&b.e));
    b.lit = 0;
    b.ma = 0;
    bd_ms(&b, 999u);
    CHECK(b.nw == 0, "dark for 0.999 s: not yet");
    bd_ms(&b, 2u);
    r = bd_stored(&b, &found);
    CHECK(b.nw == 1 && found && r.wh == 2880u && r.on_s == 2u * 86400u + 43200u + 1u && r.lit_s >= 2u * 86400u + 43200u &&
          b.lit_saves == 0u, "written at the first dark second: %u Wh, %u s powered, %u s lit", r.wh, r.on_s, r.lit_s);
    /* dark: 30 s after the light goes off */
    b.lit = 1;
    b.ma = 2000;
    bd_ms(&b, 600000u);                                  /* 10 min lit: 8 Wh */
    b.lit = 0;
    b.ma = 0;
    bd_ms(&b, 29000u);
    CHECK(b.nw == 1, "dark for 29 s: not yet");
    bd_ms(&b, 2000u);
    r = bd_stored(&b, &found);
    CHECK(b.nw == 2 && r.wh == 2888u, "dark for 30 s: written (%u, %u Wh)", b.nw, r.wh);
    k = r.on_s;
    /* idling: powered time alone, after a day */
    bd_ms(&b, 23u * 3600000u);
    CHECK(b.nw == 2, "idle for 23 h: nothing written (%u)", b.nw);
    bd_ms(&b, 3600000u);
    r = bd_stored(&b, &found);
    CHECK(b.nw == 3 && r.on_s == k + 86400u, "idle for 24 h: the powered time "
          "written (%u, %u s)", b.nw, r.on_s);
    /* lit below 1 Wh, then dark: nothing new but powered time: not written (until a day) */
    b.lit = 1;
    b.ma = 20;                                           /* 0.48 W for 10 min: 0.08 Wh */
    bd_ms(&b, 600000u);
    b.lit = 0;
    b.ma = 0;
    bd_ms(&b, 60000u);
    CHECK(b.nw == 3, "0.08 Wh lit, then dark: nothing written (%u)", b.nw);
    /* a dip: written at once (10 min powered since), into an erased entry */
    bd_dip(&b);
    CHECK(b.nw == 4 && b.ndip == 1, "a dip with 11 min powered since: written (%u, %u)", b.nw, b.ndip);
    bd_ms(&b, 1000u);
    bd_dip(&b);
    CHECK(b.nw == 4, "a second dip a second later: nothing to store (%u)", b.nw);
    r = bd_stored(&b, &found);
    CHECK(r.dips == 1u && b.e.lt.dips == 2u, "the dips: 1 stored, 2 counted (%u, %u)", r.dips, b.e.lt.dips);
    /* a trip is an event: the next dip stores it, also while lit (dips admitted) */
    b.trips = 1u << 16;                                  /* a short */
    b.lit = 1;
    bd_ms(&b, 1000u);
    bd_dip(&b);
    r = bd_stored(&b, &found);
    CHECK(b.nw == 5 && r.trips[1] == 1u && r.trips[0] == 0u && r.dips == 3u && b.lit_dips == 1u, "a short, then a dip "
          "while lit: stored (%u)", b.nw);
    /* dips not admitted while lit (0.6.3's FLASH_ADMIT_DIP 0): a lit dip writes nothing, uses no token */
    b.no_dip = 1;
    b.trips = 2u << 16;
    bd_ms(&b, 1000u);
    k = b.e.budget.tokens;
    bd_dip(&b);
    CHECK(b.nw == 5 && b.e.budget.tokens == k, "dips not admitted: a dip while lit writes nothing, no token (%u)", b.nw);
    b.lit = 0;
    bd_ms(&b, 31000u);
    CHECK(b.nw == 6 && b.lit_saves == 0u, "... stored 30 s after the light went off (%u)", b.nw);
    b.no_dip = 0;
    /* a dark board's own current (0.72 W): its Wh never ask for more than the daily write */
    {
        uint32_t w0 = b.nw;
        b.ma = 30;
        bd_ms(&b, 12u * 3600000u);
        CHECK(b.nw - w0 <= 1u, "dark, 0.72 W of logic for 12 h (8.6 Wh): %u writes (the daily one at most)",
              b.nw - w0);
        b.ma = 0;
    }
    /* a dip storm with nothing new: one write per 10 min while tokens last, then one per 3 h */
    {
        uint32_t w0 = b.nw;
        for (k = 0; k < 6u * 3600u; k++) {               /* a dip every second for 6 h */
            bd_ms(&b, 1000u);
            bd_dip(&b);
        }
        n = b.nw - w0;
        CHECK(n >= 3u && n <= LIFE_BUDGET_CAP + 2u, "a dip every second for 6 h: %u writes", n);
    }
    /* the energy across the since-boot mWh counter's wrap (2^32 mWh = 4,294,967 Wh + 296 mWh) */
    {
        uint32_t t0 = energy_total_wh(&b.e), s0 = b.s;
        b.e.boot_mwh = b.e.mwh_seen = 0xFFFFFFFFu - 2500u;    /* (as if 466 days at 384 W, all carried) */
        b.lit = 1;
        b.mv = 48000;
        b.ma = 8000;                                     /* 384 W: 384 mWh in 3.6 s */
        bd_ms(&b, 30000u);                               /* 3.2 Wh, across the wrap */
        CHECK(energy_total_wh(&b.e) == t0 + 3u && b.e.boot_mwh < 1000u && b.e.base_wh == t0 + 3u,
              "across the mWh wrap: %u -> %u Wh (+3, the 0.2 Wh remainder kept), since boot %u mWh", t0,
              energy_total_wh(&b.e), b.e.boot_mwh);
        bd_ms(&b, 8000u);                                /* +0.85 Wh: the remainder completes the 4th */
        CHECK(energy_total_wh(&b.e) == t0 + 3u + 1u, "... the remainder exact: +4 Wh after 4.05 Wh (%u)",
              energy_total_wh(&b.e) - t0);
        (void)s0;
        b.lit = 0;
        b.ma = 0;
    }
    /* the peaks */
    b.i_avg = 7949;
    b.temp = 684;
    bd_ms(&b, 1000u);
    CHECK(b.e.lt.peak_a == 79u && b.e.lt.peak_t == 137u, "peaks: 7.9 A, 68.5 C (%u, %u)", b.e.lt.peak_a,
          b.e.lt.peak_t);
    b.i_avg = 2000;
    b.temp = EN_TEMP_NA;
    bd_ms(&b, 1000u);
    CHECK(b.e.lt.peak_a == 79u && b.e.lt.peak_t == 137u, "peaks: kept (lower values, no temperature)");
    b.i_avg = 30000;
    b.temp = 2000;
    bd_ms(&b, 1000u);
    CHECK(b.e.lt.peak_a == 255u && b.e.lt.peak_t == 255u, "peaks: saturate at 25.5 A / 127.5 C");
    /* the largest input; a negative current */
    {
        energy_t e;
        energy_init(&e, 0, 0, 0, 0);
        for (k = 1; k <= 60000u; k++) {
            energy_ms(&e, k, 75000u, 20000, 1);
        }
        CHECK(e.boot_mwh == 25000u, "75 V x 20 A for 1 min: %u mWh (25 Wh)", e.boot_mwh);
        energy_init(&e, 0, 0, 0, 0);
        for (k = 1; k <= 1000u; k++) {
            energy_ms(&e, k, 24000u, -50, 1);
        }
        CHECK(e.boot_mwh == 0u && e.uj == 0u, "a negative current (offset) adds nothing");
    }
}

/* ---- the record, the pages, power cuts ---- */
static void test_energy_record(void)
{
    static ebd_t b, c;
    life_rec_t r;
    uint32_t found, k, cut, place, er;
    unsigned long long seed = 0x1234567ull;

    printf("energy: the lifetime record, the page pair, power cuts\n");
    for (er = 0; er < 2u; er++) {
        uint16_t erased = er ? 0xE339u : 0xFFFFu;
        memset(&b, 0, sizeof b);
        lf_init(&b.f, erased);
        found = lf_load(&b.f, &r);
        CHECK(!found && b.f.pos.next == 0u && !b.f.pos.erase, "erased (%04X): none, next 0", erased);
        bd_boot(&b, 0, 0x00060300u, 1);
        b.e.lt.wh = 0;
        b.e.boot_mwh = 1284321u;
        b.e.base_wh = 1000u;
        b.e.lt.on_s = 77u;
        b.e.lt.lit_s = 66u;
        b.e.lt.dips = 14u;
        b.e.lt.trips[0] = 3u;
        b.e.lt.trips[1] = 2u;
        b.e.lt.peak_t = 137u;
        b.e.lt.peak_a = 79u;
        CHECK(lf_store(&b.f, &b.e, LH) == 1u, "stored");
        r = bd_stored(&b, &found);
        CHECK(found && r.wh == 1000u + 1284u && r.on_s == 77u && r.lit_s == 66u && r.ups == 1u && r.dips == 14u &&
              r.ver == energy_ver(0x00060300u) && r.writes == 1u && r.trips[0] == 3u && r.trips[1] == 2u &&
              r.updates == 0u && r.wd == 1u && r.peak_t == 137u && r.peak_a == 79u && r.magic == LIFE_MAGIC,
              "every field back (%04X)", erased);
        CHECK(b.f.pos.next == 1u && !b.f.pos.erase, "next: entry 1");
        {                                                /* the writes counter across its 32-bit wrap */
            static ebd_t w;
            uint32_t i;
            w = b;
            lf_init(&w.f, erased);
            w.e.lt.writes = 0xFFFFFFFDu;
            for (i = 0; i < 4u; i++) {
                (void)lf_store(&w.f, &w.e, LH);         /* 0xFFFFFFFE, 0xFFFFFFFF, 0, 1 */
            }
            found = lf_load(&w.f, &r);
            CHECK(found && r.writes == 1u && w.f.pos.next == 4u, "the writes counter across its wrap: the newest is "
                  "1 (%u)", r.writes);
        }
        /* page 0's last entry: page 1 is next, its erase pending (a write never erases); no write until the erase */
        for (k = 0; k < LIFE_RECS - 1u; k++) {
            (void)lf_store(&b.f, &b.e, LH);
        }
        CHECK(b.f.pos.next == 16u && b.f.pos.erase && b.f.erases[0] + b.f.erases[1] == 0u, "a page of records: page 0 full, "
              "page 1 next, its erase pending, nothing erased yet");
        CHECK(lf_store(&b.f, &b.e, LH) == 0u, "a write now: none (no erased entry)");
        lf_prepare(&b.f, 0);
        CHECK(!b.f.pos.erase && b.f.erases[1] == 1u && lf_store(&b.f, &b.e, LH) == 1u, "erased (dark): page 1 written");
        /* page 1 full: page 0 (its older records) pending; a boot sees the same and erases it before the outputs start */
        for (k = 0; k < LIFE_RECS - 1u; k++) {
            (void)lf_store(&b.f, &b.e, LH);
        }
        found = lf_load(&b.f, &r);                       /* the next boot */
        CHECK(found && r.writes == 2u * LIFE_RECS && b.f.pos.next == 0u && b.f.pos.erase && b.f.erases[0] == 0u,
              "two pages of records, a boot: page 0 next, to be erased (%u)", r.writes);
        lf_prepare(&b.f, 0);
        CHECK(b.f.erases[0] == 1u && lf_store(&b.f, &b.e, LH) == 1u && b.f.bad == 0u, "erased at power-up, written");
        r = bd_stored(&b, &found);
        CHECK(found && r.writes == 2u * LIFE_RECS + 1u, "two pages and one records (%u)", r.writes);

        /* power cuts: a write cut after each half-word, at each place: entry 1 (mid-page), the last entry (it
         * fills the page), the first entry of a page just erased */
        for (place = 0; place < 3u; place++) {
            for (cut = 0; cut <= LH; cut++) {
                life_rec_t before, after;
                uint32_t f2, prev;
                c = b;
                if (place >= 1u) {
                    while ((c.f.pos.next & 15u) != LIFE_RECS - 1u) {
                        (void)lf_store(&c.f, &c.e, LH);
                    }
                    if (place == 2u) {
                        (void)lf_store(&c.f, &c.e, LH); /* fills it; the next page erased */
                        lf_prepare(&c.f, 0);
                    }
                }
                before = bd_stored(&c, &f2);
                prev = before.writes;
                c.e.lt.on_s += 1000u;
                (void)lf_store(&c.f, &c.e, cut);
                /* power lost; the next boot */
                f2 = lf_load(&c.f, &after);
                c.e.lt.writes = after.writes;            /* (the boot's RAM: the stored record) */
                CHECK(f2 && (after.writes == (cut == LH ? prev + 1u : prev) || (cut == LH - 1u && after.writes == prev + 1u)),
                      "place %u, cut after %u half-words: the newest is write %u (%u before)", place, cut,
                      after.writes, prev);
                /* the boot's erase (if one is pending); the next write goes into an erased entry, the newest */
                lf_prepare(&c.f, 0);
                (void)lf_store(&c.f, &c.e, LH);
                after = bd_stored(&c, &f2);
                CHECK(f2 && after.writes == c.e.lt.writes && c.f.bad == 0u,
                      "place %u, cut %u: the next write is the newest (%u), no half-word programmed twice (%u)",
                      place, cut, after.writes, c.f.bad);
            }
        }

        /* an erase cut short: the page being erased (older records only) half erased or garbage */
        for (k = 0; k < 2000u; k++) {
            uint32_t i, p, f2, w, e0;
            life_rec_t got;
            c = b;
            while ((c.f.pos.next & 15u) != LIFE_RECS - 1u) {
                (void)lf_store(&c.f, &c.e, LH);
            }
            (void)lf_store(&c.f, &c.e, LH);             /* fills the page: the other one is erased ... */
            w = c.e.lt.writes;
            p = (c.f.pos.next >> 4) & 1u;
            c.f.pg[p] = c.f.pg[p ^ 1u];                  /* ... cut short: it held older records (here: these) */
            for (i = 0; i < LIFE_RECS; i++) {            /* as older records */
                life_rec_t *o = (life_rec_t *)(c.f.pg[p].b + i * LIFE_REC_BYTES);
                o->writes -= LIFE_RECS;
                o->crc = crc16_ccitt((const uint8_t *)o, LIFE_REC_BYTES - 2u);
            }
            for (i = 0; i < 64u; i++) {                  /* half erased: bits go to 1 (or garbage, every 4th run) */
                seed ^= seed << 13; seed ^= seed >> 7; seed ^= seed << 17;
                c.f.pg[p].w[i] = (k & 3u) == 3u ? (uint32_t)seed : c.f.pg[p].w[i] | ((uint32_t)seed & (uint32_t)(seed >> 32));
            }
            f2 = lf_load(&c.f, &got);                    /* the next boot */
            CHECK(f2 && got.writes == w, "erase cut short (%u): the newest is still write %u (%u)", k, w, got.writes);
            e0 = c.f.erases[p];
            CHECK(c.f.pos.next == (p << 4) && c.f.pos.erase && lf_store(&c.f, &c.e, LH) == 0u, "... the next write "
                  "waits for its erase (%u)", k);
            lf_prepare(&c.f, 0);
            CHECK(c.f.erases[p] == e0 + 1u && lf_store(&c.f, &c.e, LH) == 1u && c.f.bad == 0u, "... erased at power-up "
                  "or while dark, then written (%u)", k);
            if (g_fail > 20) {
                break;
            }
        }
    }

    /* the ENERGY page */
    {
        energy_t e;
        uint8_t pg[24];
        energy_init(&e, 0, 0, 0, 0);
        e.base_wh = 1000u;
        e.boot_mwh = 48000u;
        energy_saved(&e, 3600u);
        energy_page(&e, 3700u, 48000u, pg);
        CHECK(ultra_get32(pg + ULTRA_EN_BOOT_MWH) == 48000u && ultra_get32(pg + ULTRA_EN_TOTAL_WH) == 1048u &&
              ultra_get32(pg + ULTRA_EN_POWER_MW) == 48000u && ultra_get24(pg + ULTRA_EN_SAVED_AGE) == 100u,
              "ENERGY page fields");
        energy_init(&e, 0, 0, 0, 0);
        energy_page(&e, 5u, 0, pg);
        CHECK(ultra_get24(pg + ULTRA_EN_SAVED_AGE) == 0xFFFFFFu, "ENERGY page: not stored since boot");
    }
}

/* ---- boots: power-ups, updates, watchdog resets, the allowance, the wraps ---- */
static void test_energy_boots(void)
{
    static ebd_t b;
    life_rec_t r;
    uint32_t found, k;

    printf("energy: power-ups, updates (counted once), watchdog resets, the allowance, the wraps\n");
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    bd_boot(&b, 0, 0x00060300u, 0);                      /* v0.6.3: a new board */
    bd_ms(&b, 700000u);
    bd_dip(&b);
    r = bd_stored(&b, &found);
    CHECK(found && r.ups == 1u && r.updates == 0u && r.ver == energy_ver(0x00060300u), "boot 1: stored, no update");
    bd_boot(&b, 0, 0x00060400u, 0);                      /* v0.6.4, the power goes before anything is stored */
    CHECK(b.e.lt.updates == 1u && b.e.lt.ups == 2u, "boot 2 (new version): the update counted in RAM");
    bd_ms(&b, 5000u);
    bd_boot(&b, 0, 0x00060400u, 0);                      /* v0.6.4 again: still differs from the stored version */
    CHECK(b.e.lt.updates == 1u && b.e.lt.ups == 2u, "boot 3: counted once (%u), the lost boot too (%u ups: boot 2 "
          "was never stored)", b.e.lt.updates, b.e.lt.ups);
    bd_ms(&b, 700000u);
    bd_dip(&b);
    bd_boot(&b, 0, 0x00060400u, 0);
    CHECK(b.e.lt.updates == 1u && b.e.lt.ups == 3u, "boot 4 (same version): no update (%u)", b.e.lt.updates);
    bd_ms(&b, 700000u);
    bd_dip(&b);
    bd_boot(&b, 0, 0, 0);                                /* a development build: not compared, the version kept */
    CHECK(b.e.lt.updates == 1u && b.e.lt.ver == energy_ver(0x00060400u), "a development build: no update, version "
          "kept");
    bd_ms(&b, 700000u);
    bd_dip(&b);
    bd_boot(&b, 0, 0x00070000u, 1);                      /* v0.7.0 after a watchdog reset */
    CHECK(b.e.lt.updates == 2u && b.e.lt.wd == 1u, "v0.7.0 after a watchdog reset: update 2, wd 1");
    bd_ms(&b, 700000u);
    bd_dip(&b);
    r = bd_stored(&b, &found);
    CHECK(found && r.ups == 5u && r.updates == 2u && r.wd == 1u && r.ver == energy_ver(0x00070000u),
          "stored: 5 power-ups (%u; "
          "the 6th boot was never stored), 2 updates, 1 watchdog reset", r.ups);
    b.e.lt.wd = 254u;
    for (k = 0; k < 3u; k++) {
        bd_ms(&b, 700000u);
        bd_dip(&b);
        bd_boot(&b, 0, 0x00070000u, 1);
    }
    CHECK(b.e.lt.wd == 255u, "watchdog resets saturate at 255 (%u)", b.e.lt.wd);

    /* the lifetime allowance: LIFE_RESERVE + one per 3 h powered, across boots */
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    bd_boot(&b, 0, 1u, 0);
    b.e.lt.writes = LIFE_RESERVE - 1u;                   /* (as if the reserve were nearly used) */
    bd_ms(&b, 700000u);
    bd_dip(&b);
    CHECK(b.nw == 1u, "the last write of the reserve (%u)", b.nw);
    for (k = 0; k < 20u; k++) {                          /* 20 boots of 4 h, each ending in a dip: each earns it */
        bd_boot(&b, 0, 1u, 0);
        bd_fast(&b, 1440u, 10u, 0);
        bd_dip(&b);
    }
    r = bd_stored(&b, &found);
    CHECK(r.writes == LIFE_RESERVE + 20u && r.writes <= LIFE_RESERVE + r.on_s / LIFE_EARN_S,
          "then within one per 3 h powered: 20 boots of 4 h stored (%u writes, %u s)", r.writes, r.on_s);
    for (k = 0; k < 30u; k++) {                          /* 30 boots of 15 min: the 6 writes left, then none (the */
        bd_boot(&b, 0, 1u, 0);                           /* powered time of a boot never stored isn't earned) */
        bd_fast(&b, 90u, 10u, 0);
        bd_dip(&b);
    }
    r = bd_stored(&b, &found);
    CHECK(r.writes == LIFE_RESERVE + r.on_s / LIFE_EARN_S && r.writes > LIFE_RESERVE + 20u,
          "15 min boots: the allowance used up, never exceeded (%u writes, %u s)", r.writes, r.on_s);

    /* the wraps: booted 65 s before the ms counter's, lit, then dark for 60 days (lit_ms 2^32 ms back) */
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    bd_boot(&b, 0xFFFF0000u, 1u, 0);
    b.mv = 24000;
    b.ma = 2000;
    b.lit = 1;
    bd_ms(&b, 600000u);
    b.lit = 0;
    b.ma = 30;
    bd_ms(&b, 40000u);
    r = bd_stored(&b, &found);
    CHECK(b.nw == 1u && r.lit_s >= 599u && r.lit_s <= 601u, "across the ms wrap: lit 10 min (%u s), written 30 s dark "
          "(%u)", r.lit_s, b.nw);
    b.ma = 0;
    bd_fast(&b, 60u * 86400u, 1u, 0);                    /* 60 days dark, second by second */
    CHECK(b.e.lt.lit_s == r.lit_s && b.nw >= 60u && b.nw <= 61u, "60 days dark: no lit second added (%u), one write a "
          "day (%u)", b.e.lt.lit_s, b.nw);
    b.lit = 1;
    b.ma = 2000;
    bd_ms(&b, 120000u);
    b.lit = 0;
    bd_ms(&b, 40000u);
    CHECK(b.e.lt.lit_s >= r.lit_s + 119u && b.e.lt.lit_s <= r.lit_s + 121u && b.e.saved && b.nw >= 61u &&
          b.e.saved_s == b.s - 10u, "lit again: 2 min counted, written 30 s dark (%u)", b.nw);
}

/* ---- wear over simulated years ---- */
typedef struct {
    const char *name;
    uint32_t years;
} wear_t;

/* years = simulated; the erase rate over them is extrapolated to 10k cycles (check: >= 25 years unless short, a
 * run shorter than that: then only the allowance is checked) */
static void wear_report(ebd_t *b, const char *name, double years, double powered_h, uint32_t sessions, uint32_t lit_ok)
{
    life_rec_t r;
    uint32_t found, emax = b->f.erases[0] > b->f.erases[1] ? b->f.erases[0] : b->f.erases[1];
    double per_year = emax / years, life = per_year > 0 ? 10000.0 / per_year : 1e9;
    r = bd_stored(b, &found);
    printf("  %-44s %5.1f y: %7u writes (%5.1f/day; %u lit dips), erases %5u/%5u (lit %u), %6.1f/page/year -> ", name,
           years, b->f.writes, b->f.writes / (years * 365.25), b->lit_dips, b->f.erases[0], b->f.erases[1],
           b->f.lit_erases, per_year);
    if (years >= 20.0) {
        printf("10k cycles in %5.1f y", life);
    } else {
        printf("(the lifetime reserve first: the 30-year runs give the life)");
    }
    printf("; stored: %u power-ups of %u, %.0f of %.0f h\n", r.ups, sessions, r.on_s / 3600.0, powered_h);
    CHECK(life >= 25.0 || years < 20.0, "%s: >= 25 years at 10k cycles (%.1f)", name, life);
    CHECK(lit_ok || b->f.lit_erases == 0u, "%s: every erase while dark (%u while lit)", name, b->f.lit_erases);
    CHECK(b->lit_saves == 0u, "%s: no regular write while lit (%u)", name, b->lit_saves);
    CHECK(b->f.bad == 0u, "%s: no half-word programmed twice", name);
    CHECK(b->f.writes <= LIFE_RESERVE + (uint32_t)(powered_h * 3600.0 / LIFE_EARN_S) + 1u,
          "%s: within the lifetime allowance", name);
}

static void test_energy_wear(void)
{
    static ebd_t b;
    uint32_t d, k, days;

    printf("energy: wear over simulated years (two pages, %u records each, 10k erase cycles)\n", LIFE_RECS);
    /* 1. powered 24/7 for 30 years, the worst controller: lit 2 min at 384 W, dark 30 s, again; a supply dip and a
     *    trip every cycle (each one an event) */
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    bd_boot(&b, 0, 1u, 0);
    days = 30u * 365u + 7u;
    for (k = 0; k < days * 86400u / 150u; k++) {
        b.lit = 1;
        bd_fast(&b, 12u, 10u, 384000u);
        b.trips += 1u;
        bd_dip(&b);
        b.lit = 0;
        bd_fast(&b, 3u, 10u, 1000u);
    }
    wear_report(&b, "powered 24/7, on 2 min / off 30 s, dip + trip", days / 365.25, days * 24.0, 1u, 0);
    /* 2. lit 24/7 for 30 years, never dark, a power cut once a month: stored only by the dip write */
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    for (k = 0; k < days / 30u; k++) {
        bd_boot(&b, 0, 1u, 0);
        b.lit = 1;
        bd_fast(&b, 30u * 8640u, 10u, 100000u);
        bd_dip(&b);
    }
    wear_report(&b, "lit 24/7, never dark, a power cut a month", days / 365.25, k * 720.0, k, 0);
    CHECK(b.lit_saves == 0u && b.lit_dips == b.f.writes, "never dark: only the dip writes (%u of %u)", b.lit_dips,
          b.f.writes);
    {
        life_rec_t r;
        uint32_t found;
        r = bd_stored(&b, &found);
        CHECK(found && r.ups == k && r.on_s == k * 30u * 86400u, "never dark: every month stored by its dip");
    }
    /* the same without dip writes while lit (0.6.3's FLASH_ADMIT_DIP 0): nothing is ever stored but the first
     * power-up */
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    b.no_dip = 1;
    for (k = 0; k < 12u; k++) {
        bd_boot(&b, 0, 1u, 0);
        b.lit = 1;
        bd_fast(&b, 30u * 8640u, 10u, 100000u);
        bd_dip(&b);
    }
    {
        life_rec_t r;
        uint32_t found;
        r = bd_stored(&b, &found);
        printf("  lit 24/7, never dark, dips not admitted, a year: %u writes, stored %u power-ups of 12, %u h of 8640 "
               "(each month lost at its power cut)\n", b.f.writes, found ? r.ups : 0u, found ? r.on_s / 3600u : 0u);
        CHECK(b.f.writes == 0u, "dips not admitted, never dark: nothing written while lit");
    }
    /* 3. the worst at all: powered 24/7 but cut every 3 h (4 fresh tokens each time), the light toggled as in 1. (a
     *    write 30 s after every off, a dip + trip every cycle): the lifetime reserve goes first, then the allowance
     *    holds it at one write per 3 h powered */
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    for (d = 0; d < days * 8u; d++) {
        bd_boot(&b, 0, 1u, 0);
        for (k = 0; k < 72u; k++) {                      /* 72 x 150 s = 3 h */
            b.lit = 1;
            bd_fast(&b, 12u, 10u, 384000u);
            b.trips += 1u;
            bd_dip(&b);
            b.lit = 0;
            bd_fast(&b, 3u, 10u, 1000u);
        }
        bd_dip(&b);
    }
    wear_report(&b, "cut every 3 h, on 2 min / off 30 s, dip + trip", days / 365.25, days * 24.0, days * 8u, 0);
    /* 3. power cycled 24/7: on 10 min, a dip, off (no time): 30 years */
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    for (k = 0; k < days * 144u; k++) {
        bd_boot(&b, 0, 1u, 0);
        b.lit = 1;
        bd_fast(&b, 60u, 10u, 50000u);
        bd_dip(&b);
    }
    wear_report(&b, "power cycled 24/7: on 10 min, off, again", days / 365.25, days * 24.0, k, 0);
    /* 4. the same, on 30 s each time: one year */
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    for (k = 0; k < 366u * 2880u; k++) {
        bd_boot(&b, 0, 1u, 0);
        b.lit = 1;
        bd_fast(&b, 3u, 10u, 50000u);
        bd_dip(&b);
    }
    wear_report(&b, "power cycled 24/7: on 30 s, off, again", 366.0 / 365.25, 366.0 * 24.0, k, 0);
    CHECK(b.f.writes >= LIFE_RESERVE, "on 30 s: the lifetime reserve stored every boot first (%u)", b.f.writes);
    /* 4b. (round 9 phase B) a flicker: on 0.5 s with a dip, off, 60,000 times: the dip in the first second writes
     * (energy_init's dip_ok; before, nothing), the reserve first; no second ever passes, no own time earns more */
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    for (k = 0; k < 60000u; k++) {
        bd_boot(&b, 0, 1u, 0);
        b.ms += 500u;
        bd_dip(&b);
    }
    wear_report(&b, "a flicker: on 0.5 s with a dip, off, 60,000 x", 60000.0 / 86400.0 / 365.25, 60000.0 / 7200.0, k,
                0);
    CHECK(b.f.writes == LIFE_RESERVE, "flicker: the reserve, then nothing (%u writes)", b.f.writes);
    /* 5. a hallway on a wall switch: 30 times a day for 3 min, 25 years */
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    days = 25u * 365u + 6u;
    for (d = 0; d < days; d++) {
        for (k = 0; k < 30u; k++) {
            bd_boot(&b, 0, 1u, 0);
            b.lit = 1;
            bd_fast(&b, 18u, 10u, 40000u);
            bd_dip(&b);
        }
    }
    wear_report(&b, "hallway on a wall switch: 30 x 3 min a day", days / 365.25, days * 1.5, days * 30u, 0);
    /* 6. a wall switch: 3 times a day for 2 h, 25 years: everything stored */
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    for (d = 0; d < days; d++) {
        for (k = 0; k < 3u; k++) {
            bd_boot(&b, 0, 1u, 0);
            b.lit = 1;
            bd_fast(&b, 720u, 10u, 60000u);
            bd_dip(&b);
        }
    }
    wear_report(&b, "wall switch: 3 x 2 h a day", days / 365.25, days * 6.0, days * 3u, 0);
    {
        life_rec_t r;
        uint32_t found;
        r = bd_stored(&b, &found);
        CHECK(r.ups == days * 3u && r.on_s == days * 3u * 7200u, "wall switch: every power-up and hour stored");
    }
    /* 7. always powered, a controller switching the light 4 times a day (6 h lit in all), 25 years */
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    bd_boot(&b, 0, 1u, 0);
    for (d = 0; d < days; d++) {
        for (k = 0; k < 4u; k++) {                       /* 90 min lit, 4.5 h dark */
            b.lit = 1;
            bd_fast(&b, 540u, 10u, 80000u);
            b.lit = 0;
            bd_fast(&b, 1620u, 10u, 1000u);
        }
    }
    wear_report(&b, "always powered, light on / off 4 x a day", days / 365.25, days * 24.0, 1u, 0);
}

/* ---- (0.6.10) the receiver's standby cycles over 10 years ---- */
static void bd_planned(ebd_t *b)                         /* the cut after a planned power-off notice (main.c: vdip
                                                            doesn't count it, ENERGY_PLANNED at its start) */
{
    bd_write(b, energy_due(&b->e, b->ms, b->dips, bd_admit(b) | ENERGY_PLANNED));
}

static uint32_t bd_exhausted(const ebd_t *b)             /* the allowance used up (main.c: DIAG's EXHAUSTED) */
{
    return b->e.lt.writes >= LIFE_RESERVE + b->e.lt.on_s / LIFE_EARN_S;
}

/* a day: an evening lit 3 h at 30 W (dark 10 min, then the port off), and n standby cycles: a wake, lit 1 min at
 * 30 W (0.5 Wh) and dark 3 min (pre: dark 2 min, a pre-wake; pre 2: lit 2 min, 1 Wh: worth a write), then the port
 * off. rx: the receiver's rules (ev_save 1, the notice); else 0.6.9's (a cut is a dip, the power-up an event).
 * Returns the day the lifetime reserve ran out (an allowance-limited write: 0 = never); sb_exh: the cycles that
 * ended with the allowance used up, sb_last: the writes in the last year */
static uint32_t sb_exh, sb_last;

static uint32_t standby_run(ebd_t *b, uint32_t days, uint32_t n, uint32_t pre, uint32_t rx)
{
    uint32_t d, k, out = 0, w = 0;
    sb_exh = 0;
    memset(b, 0, sizeof *b);
    lf_init(&b->f, 0xFFFFu);
    for (d = 0; d < days; d++) {
        for (k = 0; k <= n; k++) {
            bd_boot(b, 0, 1u, 0);
            if (rx) {
                b->e.ev_save = 1u;
            }
            b->lit = 1;
            if (k == n) {
                bd_fast(b, 1080u, 10u, 30000u);          /* the evening */
                b->lit = 0;
                bd_fast(b, 60u, 10u, 1000u);
            } else if (pre != 1u) {
                bd_fast(b, pre ? 12u : 6u, 10u, 30000u);
                b->lit = 0;
                bd_fast(b, 18u, 10u, 1000u);
            } else {
                b->lit = 0;
                bd_fast(b, 12u, 10u, 1000u);
            }
            if (rx) {
                bd_planned(b);
            } else {
                bd_dip(b);
            }
            if (!out && b->e.lt.writes >= LIFE_RESERVE) {
                out = d + 1u;
            }
            sb_exh += bd_exhausted(b);
        }
        if (d + 365u == days) {
            w = b->f.writes;
        }
    }
    sb_last = b->f.writes - w;
    return out;
}

static void test_energy_standby(void)
{
    static ebd_t b;
    static const uint32_t cyc[2] = { 50u, 300u };
    uint32_t i, k, rx, out, days = 10u * 365u + 2u;
    printf("energy: the receiver's standby cycles, 10 years (0.6.10: the planned power-off's write gate)\n");
    for (i = 0; i < 2u; i++) {
        for (rx = 1u; rx <= 1u; rx--) {
            uint32_t emax;
            out = standby_run(&b, days, cyc[i], i, rx);
            emax = b.f.erases[0] > b.f.erases[1] ? b.f.erases[0] : b.f.erases[1];
            printf("  %3u %s a day + an evening, %-28s %7u writes (%5.2f/day), erases %5u/%5u, the reserve %s",
                   cyc[i], i ? "pre-wakes" : "lit wakes", rx ? "with the gate (0.6.10):" : "each cut a dip (0.6.9):",
                   b.f.writes, b.f.writes / (double)days, b.f.erases[0], b.f.erases[1],
                   out ? "gone on day " : "kept");
            if (out) {
                printf("%u\n", out);
            } else {
                printf(" (%u of %u)\n", b.f.writes, LIFE_RESERVE);
            }
            if (rx) {
                CHECK(!out && !sb_exh && b.f.writes <= 2u * days && emax < 10000u / 4u,
                      "standby %u a day: the gate keeps the reserve, the allowance and the pages (%u writes, %u "
                      "erases, %u cycles exhausted)", cyc[i], b.f.writes, emax, sb_exh);
            } else {
                CHECK(out != 0u && out < days, "standby %u a day as 0.6.9: a write per cycle, the reserve gone "
                      "(day %u)", cyc[i], out);
            }
        }
    }
    /* (review round 8, R5) wakes worth a write each (1 Wh: stored 30 s into their dark, notice or not): the reserve
     * goes, then the evenings' 3 h keep earning (about one write a day: the DIAG flag between) */
    out = standby_run(&b, days, 50u, 2u, 1u);
    printf("  %3u wakes of 1 Wh a day + an evening, with the gate: %7u writes, the reserve gone on day %u, then %u "
           "writes in the last year (the evenings), %u of %u cycles ended exhausted\n", 50u, b.f.writes, out, sb_last,
           sb_exh, days * 51u);
    CHECK(out != 0u && sb_last >= 300u && sb_exh != 0u, "wakes of 1 Wh: the reserve used up, the evenings recharge "
          "(%u writes in the last year)", sb_last);
    /* (R5) a forced exhaustion: short runs (10 min, a dip each) write nothing and their time is lost (the limit,
     * documented); one run of 3 h earns a write again */
    {
        life_rec_t r;
        uint32_t found, w0;
        memset(&b, 0, sizeof b);
        lf_init(&b.f, 0xFFFFu);
        bd_boot(&b, 0, 1u, 0);
        b.e.lt.writes = LIFE_RESERVE - 1u;
        b.e.lt.on_s = 0;
        bd_ms(&b, 700000u);
        bd_dip(&b);                                      /* the reserve's last write */
        r = bd_stored(&b, &found);
        w0 = b.f.writes;
        for (k = 0; k < 1000u; k++) {
            bd_boot(&b, 0, 1u, 0);
            b.e.ev_save = 1u;
            b.lit = 1;
            bd_fast(&b, 60u, 10u, 30000u);               /* 10 min at 30 W: worth a dip write if allowed */
            CHECK(bd_exhausted(&b), "exhausted: run %u flagged", k);
            bd_dip(&b);
        }
        printf("  forced exhaustion: 1000 runs of 10 min: %u writes, stored %u h of %u h powered; ", b.f.writes - w0,
               bd_stored(&b, &found).on_s / 3600u, r.on_s / 3600u + 1000u / 6u);
        CHECK(b.f.writes == w0, "exhausted: short runs write nothing (%u)", b.f.writes - w0);
        bd_boot(&b, 0, 1u, 0);
        b.e.ev_save = 1u;
        b.lit = 1;
        bd_fast(&b, 1080u, 10u, 30000u);                 /* one evening: 3 h */
        b.lit = 0;
        bd_fast(&b, 60u, 10u, 1000u);
        bd_dip(&b);
        printf("then one run of 3 h: %u write(s), the flag %s\n", b.f.writes - w0, bd_exhausted(&b) ? "set" : "clear");
        CHECK(b.f.writes > w0, "exhausted: one 3 h run writes again (%u)", b.f.writes - w0);
    }
}

/* ---- every flash step cut in turn ---- */
typedef struct {
    lflash_t f;
    energy_t e;
    uint32_t step, stop;                                 /* steps done; the step the power goes at */
    uint32_t committed;                                  /* the newest complete record's writes */
    uint8_t  have, cut, mode;                            /* a record complete; the power gone; how an erase is cut */
    unsigned long long seed;
} pcut_t;

static uint32_t pc_rand(pcut_t *c)
{
    c->seed ^= c->seed << 13;
    c->seed ^= c->seed >> 7;
    c->seed ^= c->seed << 17;
    return (uint32_t)c->seed;
}

/* an erase cut short: the page untouched, half erased (bits gone to 1) or garbage */
static void pc_cut_erase(pcut_t *c, uint32_t p)
{
    uint32_t i;
    for (i = 0; i < 64u && c->mode; i++) {
        c->f.pg[p].w[i] = c->mode == 1u ? c->f.pg[p].w[i] | (pc_rand(c) & pc_rand(c)) : pc_rand(c);
    }
}

static uint32_t pc_step(pcut_t *c)
{
    if (c->step++ == c->stop) {
        c->cut = 1;
        return 0;
    }
    return 1;
}

/* the board's next flash work: the pending erase (it runs while dark), then a record, half-word by half-word */
static void pc_write(pcut_t *c)
{
    uint32_t at, i;
    uint8_t *d;
    const uint8_t *src = (const uint8_t *)&c->e.lt;
    if (c->f.pos.erase) {
        if (!pc_step(c)) {
            pc_cut_erase(c, (c->f.pos.next >> 4) & 1u);
            return;
        }
        lf_prepare(&c->f, 0);
    }
    at = life_plan(&c->f.pos);
    c->e.lt.on_s = (c->e.lt.writes + 1u) * 7u;           /* the content of record number writes + 1 */
    energy_pack(&c->e);
    d = c->f.pg[(at >> 4) & 1u].b + (at & 15u) * LIFE_REC_BYTES;
    for (i = 0; i < LIFE_REC_BYTES / 2u; i++) {
        uint16_t h = (uint16_t)(d[2u * i] | d[2u * i + 1u] << 8);
        if (!pc_step(c)) {
            return;
        }
        c->f.bad += h != c->f.erased;
        d[2u * i] = src[2u * i];
        d[2u * i + 1u] = src[2u * i + 1u];
    }
    pp_done(&c->f.pos, at, 1u);                          /* (0.6.6: as flashrec.c life_store) */
    c->committed = c->e.lt.writes;
    c->have = 1;
}

/* the next boot: the newest valid record must be the last complete one (or the one cut after its data: complete);
 * its content must be that record's */
static uint32_t pc_boot(pcut_t *c, const char *when, uint32_t stop)
{
    life_rec_t r;
    uint32_t found = lf_load(&c->f, &r), ok;
    ok = (found == c->have || (found && r.writes == c->committed + 1u)) &&
         (!found || ((r.writes == c->committed || r.writes == c->committed + 1u) && r.on_s == r.writes * 7u));
    CHECK(ok, "cut at step %u (%s, erased %04X, mode %u): found %u, writes %u, committed %u", stop, when, c->f.erased,
          c->mode, found, found ? r.writes : 0u, c->committed);
    if (found) {
        c->e.lt = r;                                     /* (the boot's RAM) */
        c->committed = r.writes;
    }
    return ok;
}

static void test_energy_powercut_all(void)
{
    static pcut_t c;
    uint32_t er, mode, stop, total = 0, k, runs = 0;
    int fails = g_fail;
    printf("energy: every flash step of a 30-record run cut in turn, each image booted, the boot's erase cut too\n");
    for (er = 0; er < 2u; er++) {
        for (mode = 0; mode < 3u; mode++) {
            for (stop = 0, total = 0;; stop++) {
                memset(&c, 0, sizeof c);
                lf_init(&c.f, er ? 0xE339u : 0xFFFFu);
                c.e.lt.writes = c.committed = 0xFFFFFFF0u;   /* (the counter crosses its wrap in the run) */
                c.e.lt.magic = LIFE_MAGIC;               /* (as energy_init) */
                c.stop = stop;
                c.mode = (uint8_t)mode;
                c.seed = 0x9E3779B97F4A7C15ull + stop;
                for (k = 0; k < 30u && !c.cut; k++) {
                    pc_write(&c);
                }
                if (!c.cut) {
                    total = c.step;                      /* (every step done: the last run) */
                }
                if (!pc_boot(&c, "power cut", stop)) {
                    break;
                }
                if (c.f.pos.erase && (stop & 1u)) {      /* the boot erases the full page's partner: cut that too */
                    pc_cut_erase(&c, (c.f.pos.next >> 4) & 1u);
                    if (!pc_boot(&c, "boot erase cut", stop)) {
                        break;
                    }
                }
                c.stop = 0xFFFFFFFFu;
                c.cut = 0;
                for (k = 0; k < 8u; k++) {
                    pc_write(&c);
                }
                CHECK(pc_boot(&c, "8 records later", stop) && c.f.bad == 0u, "cut at step %u: 8 records later the "
                      "newest, nothing programmed twice (%u)", stop, c.f.bad);
                runs++;
                if (total || g_fail > fails + 10) {
                    break;
                }
            }
        }
    }
    printf("  %u runs (every one of %u steps, 2 erased patterns, 3 kinds of cut erase): %s\n", runs, total,
           g_fail == fails ? "the newest complete record every time" : "FAILED");
}

/* 0.6.5: energy per channel. ms ms of the outputs `o` (or, with flip, R alternating 0 / o[0] every ms), the board
 * current = the zero + the model's current x (1 + extra) (the model's gain off by `extra`), at mv; a second each */
static void ech_run(energy_t *e, const isense_t *is, uint32_t secs, const uint16_t o[5], double extra, int flip,
                    uint32_t mv)
{
    uint32_t s, t, k;
    for (s = 0; s < secs; s++) {
        for (t = 0; t < 1000u; t++) {
            uint16_t v[5];
            double ma = is->zero_ma;
            for (k = 0; k < 5u; k++) {
                v[k] = (uint16_t)(k == 0u && flip && (t & 1u) ? 0u : o[k]);
                ma += isense_ch_ma(is, k, v[k]) * (1.0 + extra);
            }
            energy_ms(e, t, mv, (int32_t)(ma + 0.5), 1);
            energy_ch_ms(e, v, (int32_t)(ma + 0.5));
        }
        energy_ch_second(e, is, mv);
    }
}

static void test_energy_ch(void)
{
    static const uint16_t i100[5] = { 3000, 3000, 2500, 0, 5000 };
    static const uint16_t r_full[5] = { 65535, 0, 0, 0, 0 };
    static const uint16_t mix[5] = { 32768, 0, 0, 0, 16384 };
    isense_cal_t c;
    isense_t is;
    energy_t e;
    uint32_t k, sum;
    cli_t cl;
    cli_status_t st;

    printf("energy per channel: the split by the channel model, effects, no model, the console\n");
    memset(&c, 0, sizeof c);
    c.levels = ISENSE_LEVELS;
    c.pct[0] = 25; c.pct[1] = 50; c.pct[2] = 75; c.pct[3] = 100;
    c.zero_ma = 30;
    for (k = 0; k < 5u; k++) {
        uint32_t l;
        for (l = 0; l < ISENSE_LEVELS; l++) {
            c.ma[k][l] = (uint16_t)(i100[k] * (l + 1u) / 4u);
        }
    }
    isense_init(&is, 1, 10000, 8000);
    isense_cal_set(&is, &c);
    /* an hour of R at 100 %, 24 V: R 72.00 Wh (24 V x 3 A), the others 0; the logic's 0.72 Wh in energy= only */
    energy_init(&e, 0, 0, 0, 0);
    ech_run(&e, &is, 3600, r_full, 0, 0, 24000);
    CHECK(e.ch_mwh[0] >= 71950u && e.ch_mwh[0] <= 72050u && e.ch_mwh[1] + e.ch_mwh[2] + e.ch_mwh[3] + e.ch_mwh[4] == 0u,
          "R 100 %% for 1 h: %u mWh (72000)", e.ch_mwh[0]);
    CHECK(e.boot_mwh >= e.ch_mwh[0] + 700u && e.boot_mwh <= e.ch_mwh[0] + 740u, "the zero's 0.72 Wh: in the total only "
          "(%u - %u mWh)", e.boot_mwh, e.ch_mwh[0]);
    /* R 50 % + W2 25 %, the measured current 10 % above the model: shared 1500 : 1250, the sum is what was measured */
    energy_init(&e, 0, 0, 0, 0);
    ech_run(&e, &is, 3600, mix, 0.10, 0, 24000);
    sum = e.ch_mwh[0] + e.ch_mwh[4];
    CHECK(sum + 800u >= e.boot_mwh && sum + 640u <= e.boot_mwh, "mix: the channels %u + the zero = the total %u mWh",
          sum, e.boot_mwh);
    CHECK(e.ch_mwh[0] * 1250u >= e.ch_mwh[4] * 1500u - 3000u * 1250u && e.ch_mwh[0] * 1250u <= e.ch_mwh[4] * 1500u +
          3000u * 1250u && e.ch_mwh[0] > 39000u, "mix: R %u : W2 %u mWh (1500 : 1250)", e.ch_mwh[0], e.ch_mwh[4]);
    /* an effect (R 0 / 100 % every other ms): R gets the LED energy, at the mean: 36 Wh */
    energy_init(&e, 0, 0, 0, 0);
    ech_run(&e, &is, 3600, r_full, 0, 1, 24000);
    CHECK(e.ch_mwh[0] >= 35950u && e.ch_mwh[0] <= 36050u && e.ch_mwh[4] == 0u, "effect: R %u mWh (36000)",
          e.ch_mwh[0]);
    /* a small load for a long time: no loss below a mWh a second (0.2 % of B: the model's 4 mA, 0.1 W) */
    {
        static const uint16_t dim[5] = { 0, 0, 131, 0, 0 };      /* 0.2 % of 2.5 A = 5 mA */
        energy_init(&e, 0, 0, 0, 0);
        ech_run(&e, &is, 12u * 3600u, dim, 0, 0, 24000);
        k = isense_ch_ma(&is, 2, 131) * 24u * 12u;       /* (the model's 4 mA: 1152 mWh) */
        CHECK(e.ch_mwh[2] >= k - 2u && e.ch_mwh[2] <= k + 2u, "%u mA on B for 12 h: %u mWh (%u)",
              isense_ch_ma(&is, 2, 131), e.ch_mwh[2], k);
    }
    /* no usable model (nothing learned yet): nothing shared out */
    isense_init(&is, 1, 10000, 8000);
    energy_init(&e, 0, 0, 0, 0);
    for (k = 0; k < 1000u; k++) {
        energy_ch_ms(&e, r_full, 3030);
    }
    energy_ch_second(&e, &is, 24000);
    CHECK(e.ch_mwh[0] == 0u && e.ch_mj[0] == 0u && e.ch_n == 0u, "no model: nothing (%u)", e.ch_mwh[0]);
    energy_ch_second(&e, &is, 24000);                     /* (a second without any ms: nothing, no division) */

    /* the console */
    memset(&cl, 0, sizeof cl);
    memset(&st, 0, sizeof st);
    settings_default(&cli_set);
    st.version = "test";
    st.steps = 5760;
    st.en = &e;
    cl.write = cap_write;
    cl.apply = cap_apply;
    cl.save_now = cap_save;
    cl.set = &cli_set;
    cl.eff = &cli_set;
    cl.st = &st;
    e.boot_mwh = 76540;
    e.ch_mwh[0] = 72000; e.ch_mwh[1] = 5; e.ch_mwh[2] = 1440; e.ch_mwh[3] = 0; e.ch_mwh[4] = 123456;
    run_cli(&cl, "status\r");
    CHECK(strstr(cli_out, "energy=76.54 Wh since boot\r\nenergy R/G/B/W1/W2=72.00/0.01/1.44/0.00/123.46 Wh\r\n") != NULL,
          "status: energy per channel");
}

/* ---- (0.6.10, protocol 0.2.6) the hub's counters: energy_sync (the review's tests 1-6 and 8-11), format 3 ---- */
#define LS_UID 0x1234u
#define LS_ALL (ULTRA_LS_F_POWERED | ULTRA_LS_F_LIT | ULTRA_LS_F_UPS | ULTRA_LS_F_TRIPS | ULTRA_LS_F_DIPS | \
                ULTRA_LS_F_WH)

typedef struct { uint32_t on, lit, ups, ovl, sht, dips, wh; } ls_led_t;   /* a hub's ledger, wire units */

/* the LIFE_SYNC POLL arguments for board e: UID16, LIFETIME bytes 0..15, the Wh, RECORDS low 8 bits */
static void ls_args(uint8_t a[23], const energy_t *e, const ls_led_t *l)
{
    memset(a, 0, 23);
    ultra_put16(a + ULTRA_LS_UID16, LS_UID);
    ultra_put24(a + ULTRA_LS_LIFETIME + ULTRA_LT_POWERED, l->on);
    ultra_put24(a + ULTRA_LS_LIFETIME + ULTRA_LT_LIT, l->lit);
    ultra_put24(a + ULTRA_LS_LIFETIME + ULTRA_LT_POWER_UPS, l->ups);
    ultra_put16(a + ULTRA_LS_LIFETIME + ULTRA_LT_OVERLOAD_TRIPS, l->ovl);
    ultra_put16(a + ULTRA_LS_LIFETIME + ULTRA_LT_SHORT_TRIPS, l->sht);
    ultra_put24(a + ULTRA_LS_LIFETIME + ULTRA_LT_DIPS, l->dips);
    ultra_put32(a + ULTRA_LS_TOTAL_WH, l->wh);
    a[ULTRA_LS_RECORDS] = (uint8_t)e->lt.writes;
}

/* the board's counters now, as a ledger */
static ls_led_t ls_live(const energy_t *e)
{
    ls_led_t l;
    l.on = (e->lt.on_s / 360u) & 0xFFFFFFu;
    l.lit = (e->lt.lit_s / 360u) & 0xFFFFFFu;
    l.ups = e->lt.ups & 0xFFFFFFu;
    l.ovl = e->lt.trips[0];
    l.sht = e->lt.trips[1];
    l.dips = e->lt.dips & 0xFFFFFFu;
    l.wh = energy_total_wh(e);
    return l;
}

static uint32_t ls_sync(energy_t *e, const ls_led_t *l)
{
    uint8_t a[23];
    ls_args(a, e, l);
    return energy_sync(e, a, LS_UID);
}

/* a board as booted from a record: 1000 h powered, 400 h lit, 500 power-ups, trips 3 / 1, 20 dips, 9000 Wh */
static void ls_board(energy_t *e)
{
    memset(e, 0, sizeof *e);
    e->lt.on_s = 3600000u;
    e->lt.lit_s = 1440000u;
    e->lt.ups = 500u;
    e->lt.trips[0] = 3u;
    e->lt.trips[1] = 1u;
    e->lt.dips = 20u;
    e->lt.wh = 9000u;
    e->lt.writes = 7u;
    e->lt.magic = LIFE_MAGIC;
    energy_init(e, 1u, 0, 0, 0);                         /* (the power-up: 501) */
}

/* one counter's step at its limit and one past it, on a fresh board each (dp: the powered step with it) */
static void ls_cap(const char *name, uint32_t dp, uint32_t f, uint32_t step)
{
    static energy_t e;
    ls_led_t l;
    uint32_t k, r[2];
    for (k = 0; k < 2u; k++) {
        ls_board(&e);
        l = ls_live(&e);
        l.on += dp;
        if (f == ULTRA_LS_F_LIT) l.lit += step + k;
        if (f == ULTRA_LS_F_UPS) l.ups += step + k;
        if (f == ULTRA_LS_F_TRIPS) l.ovl += step + k;
        if (f == ULTRA_LS_F_DIPS) l.dips += step + k;
        if (f == ULTRA_LS_F_WH) l.wh += step + k;
        if (f == ULTRA_LS_F_POWERED) l.on += step + k;
        r[k] = ls_sync(&e, &l);
    }
    CHECK((r[0] >> 8) == ULTRA_LS_R_APPLIED && (r[0] & f) && (r[1] >> 8) == ULTRA_LS_R_CAPPED &&
          !(r[1] & f), "cap %s: %u applied (%04X), %u + 1 ignored (%04X)", name, step, r[0], step, r[1]);
}

static uint32_t ls_diff(uint32_t a, uint32_t b)
{
    return a > b ? a - b : b - a;
}

/* (round 9 phase B, Fable's items 1-2) a hub's ledger as the spec has it (the protocol wiki, Return channel, "0x0E
 * LIFE_SYNC"): its own accounting in finer units than the wire's: each session's seconds (STATUS uptime / the port's
 * on time), its lit seconds (its own count of the outputs on: here the board's own, the simulation's lit count being
 * per step), ENERGY 0's mWh; POWER_UPS = the ledger + the power-on just performed; trips and dips from the reads (max).
 * The board's reads raise the time / energy totals only by a lead beyond one wire unit (ls_raise, R9-R1). Sent: the
 * whole wire units. (Before: an echo of the reads, max per field: it restores nothing of a session the board didn't
 * store.) */
typedef struct { uint64_t s, lit, mwh; uint32_t ups, ovl, sht, dips, lit0, naive, seen; } ls_fine_t;


static uint64_t ls_max64(uint64_t a, uint64_t b)
{
    return a > b ? a : b;
}

/* (round 9 phase B re-check, R9-R1) a board read raises a fine total only by its lead beyond one wire unit: within
 * one, it is the board's own rounding (a merge keeps the board's remainder below a unit, so after an honest hub's merge
 * the board may lead by up to one); further, time the hub didn't see (first contact, a missed session, another hub).
 * naive: the rules before, a plain max (the board's rounding taken back as elapsed time at every write: it drifts),
 * and for the power-ups the max before this power-on's + 1 (the board's read already holds it: counted twice) */
static uint64_t ls_raise(const ls_fine_t *h, uint64_t f, uint32_t w, uint32_t unit)
{
    uint64_t fl = f / unit;
    if (h->naive) {
        return ls_max64(f, (uint64_t)w * unit);
    }
    return w > fl + 1u ? f + (w - fl - 1u) * (uint64_t)unit : f;
}

/* after the board's power-up: the ledger + this power-on, raised to the board's reads; the LIFE_SYNC; returns its
 * result */
static uint32_t ls_fine_sync(ls_fine_t *h, energy_t *e)
{
    ls_led_t r = ls_live(e), l;
    uint32_t m;
    if (!h->seen) {                                      /* first contact: the board's values (its power-ups: this */
        h->seen = 1u;                                    /* power-on in them already) */
        h->s = (uint64_t)r.on * 360u;
        h->lit = (uint64_t)r.lit * 360u;
        h->mwh = (uint64_t)r.wh * 1000u;
        h->ups = r.ups;
    } else {
        if (h->naive) {
            h->ups = (h->ups > r.ups ? h->ups : r.ups) + 1u;   /* (the old order: the max, then + 1) */
        } else {
            h->ups++;                                    /* this power-on first, then the max */
            h->ups = h->ups > r.ups ? h->ups : r.ups;
        }
        h->s = ls_raise(h, h->s, r.on, 360u);
        h->lit = ls_raise(h, h->lit, r.lit, 360u);
        h->mwh = ls_raise(h, h->mwh, r.wh, 1000u);
    }
    h->ovl = h->ovl > r.ovl ? h->ovl : r.ovl;
    h->sht = h->sht > r.sht ? h->sht : r.sht;
    h->dips = h->dips > r.dips ? h->dips : r.dips;
    l.on = (uint32_t)(h->s / 360u);
    l.lit = (uint32_t)(h->lit / 360u);
    l.ups = h->ups;
    l.ovl = h->ovl;
    l.sht = h->sht;
    l.dips = h->dips;
    l.wh = (uint32_t)(h->mwh / 1000u);
    m = ls_sync(e, &l);
    h->lit0 = e->lt.lit_s;                               /* (the session's lit seconds from here) */
    return m;
}

/* before the notice: the session added */
static void ls_fine_end(ls_fine_t *h, const ebd_t *b)
{
    ls_led_t r = ls_live(&b->e);
    h->s += b->s;
    h->lit += b->e.lt.lit_s - h->lit0;
    h->mwh += b->e.boot_mwh;
    h->ovl = h->ovl > r.ovl ? h->ovl : r.ovl;
    h->sht = h->sht > r.sht ? h->sht : r.sht;
    h->dips = h->dips > r.dips ? h->dips : r.dips;
}

/* the board against the ledger's truth: the largest error, wire units */
static uint32_t ls_fine_err(const ls_fine_t *h, const energy_t *e)
{
    ls_led_t r = ls_live(e);
    uint32_t k = ls_diff((uint32_t)(h->s / 360u), r.on);
    k = ls_diff((uint32_t)(h->lit / 360u), r.lit) > k ? ls_diff((uint32_t)(h->lit / 360u), r.lit) : k;
    k = ls_diff(h->ups, r.ups) > k ? ls_diff(h->ups, r.ups) : k;
    k = ls_diff((uint32_t)(h->mwh / 1000u), r.wh) > k ? ls_diff((uint32_t)(h->mwh / 1000u), r.wh) : k;
    k = ls_diff(h->dips, r.dips) > k ? ls_diff(h->dips, r.dips) : k;
    return k;
}

/* a day: n standby cycles (a wake, lit 1 min at 30 W, dark 3 min, the planned cut) and an evening (lit 3 h, dark 10
 * min, the cut); hub 0 none, 1 a hub with its own ledger (the spec's), synced at every boot, 2 a hostile hub (every
 * counter at its full step ahead). Returns the writes; *err: the honest hub's board against the truth (every
 * session's own, summed apart from the ledger) at the end (wire units, the largest; + 1000 when the ledger itself
 * isn't the truth) */
static uint32_t sync_run(ebd_t *b, uint32_t days, uint32_t n, uint32_t hub, uint32_t *err, uint32_t *own_ok)
{
    static ls_fine_t h, t;                               /* the hub; the truth (an oracle of its own) */
    ls_led_t l;
    uint32_t d, k, ok = 1;
    uint64_t own = 0;
    memset(b, 0, sizeof *b);
    memset(&h, 0, sizeof h);
    memset(&t, 0, sizeof t);
    lf_init(&b->f, 0xFFFFu);
    for (d = 0; d <= days; d++) {
        for (k = 0; k <= (d < days ? n : 20u); k++) {     /* (the last day: 20 cycles, no evening, nothing stored) */
            bd_boot(b, 0, 1u, 0);
            b->e.ev_save = 1u;
            t.ups++;
            if (hub == 1u) {
                (void)ls_fine_sync(&h, &b->e);
            } else if (hub == 2u) {
                l = ls_live(&b->e);
                l.on += LIFE_SYNC_MAX_H10;
                l.lit += LIFE_SYNC_MAX_H10 + 1u;
                l.ups += 259264u;
                l.ovl = 0xFFFFu;
                l.dips += 259264u;
                l.wh += 721000u;
                (void)ls_sync(&b->e, &l);
            }
            t.lit0 = b->e.lt.lit_s;
            b->lit = 1;
            if (k == n && d < days) {
                bd_fast(b, 1080u, 10u, 30000u);          /* the evening */
                b->lit = 0;
                bd_fast(b, 60u, 10u, 1000u);
                own += 11400u;
            } else {
                bd_fast(b, 6u, 10u, 30000u);
                b->lit = 0;
                bd_fast(b, 18u, 10u, 1000u);
                own += 240u;
            }
            ls_fine_end(&h, b);                          /* the hub before the notice */
            t.s += b->s;                                 /* (the truth: every session's own) */
            t.lit += b->e.lt.lit_s - t.lit0;
            t.mwh += b->e.boot_mwh;
            bd_planned(b);
        }
    }
    bd_boot(b, 0, 1u, 0);                                /* a last power-up: synced, then compared */
    t.ups++;
    if (hub == 1u) {
        (void)ls_fine_sync(&h, &b->e);
    }
    *err = hub == 1u ? ls_fine_err(&t, &b->e) + (h.s != t.s || h.lit != t.lit || h.mwh != t.mwh) * 1000u : 0u;
                                                         /* (the board against the truth; the ledger exactly it) */
    *own_ok = ok && b->e.lt.on_s - LIFE_MERGED(&b->e.lt) * 360u <= own + 360u;   /* (the allowance's: the board's) */
    return b->f.writes;
}

/* 10 years: 100 wakes a day worth a write each (lit 2 min at 30 W: 1 Wh) and an evening, the reserve used up from the
 * start (the allowance decides). hub 0 none, 1 honest (its own ledger, the spec's), 2 hostile (every step at its
 * most), 3 hostile: each merge lands just below a 3 h boundary (what an earn-credit count would have taken as the
 * board's own). Returns the writes; *own: the powered time, s; *late: merges that raised the powered time in the
 * last year */
static uint32_t sync_run10(ebd_t *b, uint32_t hub, uint32_t *own, uint32_t *late)
{
    static ls_fine_t h;
    ls_led_t l;
    uint32_t d, k, m, on, bnd, w0;
    memset(b, 0, sizeof *b);
    memset(&h, 0, sizeof h);
    lf_init(&b->f, 0xFFFFu);
    *own = *late = 0;
    bd_boot(b, 0, 1u, 0);
    b->e.lt.writes = LIFE_RESERVE - 1u;
    (void)lf_store(&b->f, &b->e, LH);                    /* (the reserve used up: stored) */
    w0 = b->f.writes;
    for (d = 0; d < 3652u; d++) {
        for (k = 0; k <= 100u; k++) {
            bd_boot(b, 0, 1u, 0);
            b->e.ev_save = 1u;
            m = 0;
            l = ls_live(&b->e);
            if (hub == 1u) {
                m = ls_fine_sync(&h, &b->e);
            } else if (hub == 2u) {
                l.on += LIFE_SYNC_MAX_H10;
                l.ups += 259264u;
                l.wh += 721000u;
                m = ls_sync(&b->e, &l);
            } else if (hub == 3u) {
                on = b->e.lt.on_s;
                bnd = (on + 721u + LIFE_EARN_S - 1u) / LIFE_EARN_S * LIFE_EARN_S;
                l.on += (bnd - 1u - on) / 360u;          /* (on_s then 1-360 s below the boundary) */
                m = ls_sync(&b->e, &l);
            }
            *late += d >= 3652u - 365u && (m & ULTRA_LS_F_POWERED);
            b->lit = 1;
            if (k == 100u) {
                bd_fast(b, 1080u, 10u, 30000u);          /* the evening */
                b->lit = 0;
                bd_fast(b, 60u, 10u, 1000u);
                *own += 11400u;
            } else {
                bd_fast(b, 12u, 10u, 30000u);
                b->lit = 0;
                bd_fast(b, 18u, 10u, 1000u);
                *own += 300u;
            }
            ls_fine_end(&h, b);
            bd_planned(b);
        }
    }
    return b->f.writes - w0;
}

/* (round 9 phase B, Fable's probe kept as a test) a standby-only receiver: 30 days of 100 wakes (4 min, lit 1 min at
 * 30 W), none worth a write of its own, so it never stores anything. hub 0: an echo of the reads (the max per field,
 * the counts as read: the negative control); 1: the spec's own ledger. Returns the board's error against the truth
 * (wire units, the largest of powered, lit, power-ups, Wh); *got: the powered time the board shows, s */
static uint32_t sync_standby(ebd_t *b, uint32_t hub, uint32_t *got_on, uint32_t *got_wh, uint32_t *got_ups,
                             ls_fine_t *truth)
{
    static ls_fine_t h;
    ls_led_t led, l;
    uint32_t d, k;
    memset(b, 0, sizeof *b);
    memset(&h, 0, sizeof h);
    memset(&led, 0, sizeof led);
    memset(truth, 0, sizeof *truth);
    lf_init(&b->f, 0xFFFFu);
    for (d = 0; d <= 30u; d++) {
        for (k = 0; k < (d < 30u ? 100u : 1u); k++) {    /* (the last: a power-up, synced, compared) */
            bd_boot(b, 0, 1u, 0);
            b->e.ev_save = 1u;
            truth->ups++;
            if (hub == 1u) {
                (void)ls_fine_sync(&h, &b->e);
            } else {
                (void)ls_sync(&b->e, &led);
            }
            if (d == 30u) {
                break;
            }
            b->lit = 1;
            bd_fast(b, 6u, 10u, 30000u);                 /* 1 min at 30 W */
            b->lit = 0;
            bd_fast(b, 18u, 10u, 1000u);                 /* 3 min dark at 1 W */
            truth->s += b->s;
            truth->mwh += b->e.boot_mwh;
            truth->lit += 6u;                            /* (the board's lit count: one a 10 s step) */
            ls_fine_end(&h, b);
            l = ls_live(&b->e);                          /* the echo: the reads before the notice, max per field */
            led.on = l.on > led.on ? l.on : led.on;
            led.lit = l.lit > led.lit ? l.lit : led.lit;
            led.ups = l.ups > led.ups ? l.ups : led.ups;
            led.dips = l.dips > led.dips ? l.dips : led.dips;
            led.wh = l.wh > led.wh ? l.wh : led.wh;
            bd_planned(b);
        }
    }
    *got_on = b->e.lt.on_s;
    *got_wh = energy_total_wh(&b->e);
    *got_ups = b->e.lt.ups;
    return ls_fine_err(truth, &b->e);
}

/* (round 9 phase B, Fable's power-up probe kept as a test) 30 days of 50 standby cycles and an evening a day: the
 * spec's ledger (POWER_UPS = the ledger + the power-on just performed) restores every power-up the board didn't store;
 * an echo of the reads restores none (the negative control). Returns the board's power-ups; *n: the true count */
static uint32_t sync_ups(ebd_t *b, uint32_t hub, uint32_t *n)
{
    static ls_fine_t h;
    ls_led_t led, l;
    uint32_t d, k;
    memset(b, 0, sizeof *b);
    memset(&h, 0, sizeof h);
    memset(&led, 0, sizeof led);
    lf_init(&b->f, 0xFFFFu);
    *n = 0;
    for (d = 0; d <= 30u; d++) {
        for (k = 0; k <= 50u; k++) {
            bd_boot(b, 0, 1u, 0);
            b->e.ev_save = 1u;
            ++*n;
            if (hub == 1u) {
                (void)ls_fine_sync(&h, &b->e);
            } else {
                (void)ls_sync(&b->e, &led);
            }
            if (d == 30u) {
                return b->e.lt.ups;                      /* (the last power-up: synced, compared) */
            }
            b->lit = 1;
            if (k == 50u) {
                bd_fast(b, 1080u, 10u, 30000u);          /* the evening: 3 h at 30 W, then dark: its write */
                b->lit = 0;
                bd_fast(b, 60u, 10u, 1000u);
            } else {
                bd_fast(b, 6u, 10u, 30000u);             /* a wake: 1 min lit, 3 min dark */
                b->lit = 0;
                bd_fast(b, 18u, 10u, 1000u);
            }
            ls_fine_end(&h, b);
            l = ls_live(&b->e);
            led.on = l.on > led.on ? l.on : led.on;
            led.lit = l.lit > led.lit ? l.lit : led.lit;
            led.ups = l.ups > led.ups ? l.ups : led.ups;
            led.dips = l.dips > led.dips ? l.dips : led.dips;
            led.wh = l.wh > led.wh ? l.wh : led.wh;
            bd_planned(b);
        }
    }
    return b->e.lt.ups;
}

static void test_energy_ledger(void)
{
    static ebd_t b;
    ls_fine_t t;
    uint32_t on[2], wh[2], ups[2], err[2], hub, n[2];
    printf("energy: LIFE_SYNC's ledger (the spec: the hub's own accounting, POWER_UPS + this power-on)\n");
    for (hub = 0; hub < 2u; hub++) {
        err[hub] = sync_standby(&b, hub, &on[hub], &wh[hub], &ups[hub], &t);
        printf("  a standby-only receiver, 30 days x 100 wakes, hub %s: powered %u h of %u, energy %u Wh of %u, %u "
               "power-ups of %u, lit %u x 0.1 h of %u (the largest error %u units)\n", hub ? "with its own ledger" :
               "echoing its reads ", on[hub] / 3600u, (uint32_t)(t.s / 3600u), wh[hub], (uint32_t)(t.mwh / 1000u),
               ups[hub], t.ups, ls_live(&b.e).lit, (uint32_t)(t.lit / 360u), err[hub]);
    }
    CHECK(err[1] <= 1u, "LIFE_SYNC, the spec's ledger: a standby-only receiver within one unit of the truth (%u)",
          err[1]);
    CHECK(on[0] < (uint32_t)(t.s / 2u) && wh[0] < (uint32_t)(t.mwh / 2000u) && ups[0] <= 2u,
          "LIFE_SYNC, an echo of the reads (the negative control): restores nothing (powered %u s, %u Wh, %u power-ups)",
          on[0], wh[0], ups[0]);
    for (hub = 0; hub < 2u; hub++) {
        ups[hub] = sync_ups(&b, hub, &n[hub]);
    }
    printf("  30 days of 50 standby cycles and an evening: %u of %u power-ups with an echo hub, %u of %u with the "
           "spec's ledger\n", ups[0], n[0], ups[1], n[1]);
    CHECK(ups[1] + 1u >= n[1] && ups[1] <= n[1] && ups[0] <= 31u + 2u, "LIFE_SYNC power-ups: the spec's ledger "
          "restores them (%u of %u), an echo doesn't (%u)", ups[1], n[1], ups[0]);
}

/* (round 9 phase B re-check, R9-R1: GPT's scenario) a year of 50 standby wakes and an evening a day (3 h lit at
 * 30 W, then dark for 600-959 s: the cut's residue varies), an honest hub syncing at every power-up. The truth: every
 * session's own seconds, lit seconds, mWh and power-ups, summed apart from the ledger. naive: the hub's reconcile a
 * plain max (the rule before R9-R1); miss: a day whose evening the hub doesn't see (it was off: no sync, no
 * accounting; the board stores that session itself): the truth counts it, the hub takes in the board's lead beyond
 * one unit the next morning, and loses what cancels against its own merged wakes the board never stored; *u: the
 * session it didn't see. */
static void sync_round(ebd_t *b, uint32_t naive, uint32_t miss, ls_fine_t *h, ls_fine_t *t, ls_fine_t *u)
{
    uint32_t d, k, seen;
    memset(b, 0, sizeof *b);
    memset(h, 0, sizeof *h);
    memset(t, 0, sizeof *t);
    memset(u, 0, sizeof *u);
    h->naive = naive;
    lf_init(&b->f, 0xFFFFu);
    for (d = 0; d <= 365u; d++) {
        for (k = 0; k <= 50u; k++) {
            bd_boot(b, 0, 1u, 0);
            b->e.ev_save = 1u;
            t->ups++;
            seen = !(miss && d == miss && k == 50u);
            if (seen) {
                (void)ls_fine_sync(h, &b->e);
            }
            if (d == 365u) {
                return;                                  /* (the last power-up: synced, compared) */
            }
            t->lit0 = b->e.lt.lit_s;
            b->lit = 1;
            if (k == 50u) {
                bd_fast(b, 1080u, 10u, 30000u);          /* 3 h at 30 W */
                b->lit = 0;
                bd_fast(b, 600u + d * 37u % 360u, 1u, 1000u);   /* dark 600-959 s, then the cut */
            } else {
                bd_fast(b, 6u, 10u, 30000u);             /* a wake: 1 min lit, 3 min dark */
                b->lit = 0;
                bd_fast(b, 18u, 10u, 1000u);
            }
            t->s += b->s;
            t->lit += b->e.lt.lit_s - t->lit0;
            t->mwh += b->e.boot_mwh;
            if (seen) {
                ls_fine_end(h, b);
            } else {
                u->s += b->s;
                u->lit += b->e.lt.lit_s - t->lit0;
                u->mwh += b->e.boot_mwh;
            }
            bd_planned(b);
        }
    }
}

static void test_energy_ledger_round(void)
{
    static ebd_t b;
    ls_fine_t h, t, u;
    uint32_t k;
    int64_t ds[3], dl[3], dm[3];
    uint32_t err[3];
    printf("energy: LIFE_SYNC's ledger against the truth (R9-R1): a year of 50 wakes and an evening a day\n");
    for (k = 0; k < 3u; k++) {                           /* 0 the rule; 1 the plain max (before); 2 the rule, a day missed */
        sync_round(&b, k == 1u, k == 2u ? 100u : 0u, &h, &t, &u);
        ds[k] = (int64_t)h.s - (int64_t)t.s;
        dl[k] = (int64_t)h.lit - (int64_t)t.lit;
        dm[k] = (int64_t)h.mwh - (int64_t)t.mwh;
        err[k] = ls_fine_err(&t, &b.e);
        printf("  %s: the ledger off by %lld s, lit %lld s, %lld mWh, power-ups %d; the receiver within %u units of the "
               "truth (%u writes)\n", k == 0u ? "the rule (a lead beyond one unit)   " : k == 1u ?
               "the plain max (before: the control)" : "the rule, an evening not seen      ", (long long)ds[k],
               (long long)dl[k], (long long)dm[k], (int)(h.ups - t.ups), err[k], b.f.writes);
    }
    CHECK(ds[0] == 0 && dl[0] == 0 && dm[0] == 0 && err[0] <= 1u, "R9-R1: the rule keeps the ledger at the truth (%lld s, "
          "%lld lit s, %lld mWh) and the receiver within one unit (%u)", (long long)ds[0], (long long)dl[0],
          (long long)dm[0], err[0]);
    CHECK(ds[1] >= 360, "R9-R1 control: the plain max takes the receiver's rounding back as time (%lld s)",
          (long long)ds[1]);
    CHECK(ds[2] <= 0 && ds[2] >= -(int64_t)(u.s + 360u) && dl[2] <= 0 && dl[2] >= -(int64_t)(u.lit + 360u) &&
          dm[2] <= 0 && dm[2] >= -(int64_t)(u.mwh + 1000u), "R9-R1: an evening not seen (%llu s, %llu lit s, %llu mWh): "
          "never more than the truth, at most that session lost, no drift after (%lld s, %lld lit s, %lld mWh)",
          (unsigned long long)u.s, (unsigned long long)u.lit, (unsigned long long)u.mwh, (long long)ds[2],
          (long long)dl[2], (long long)dm[2]);
}

/* (round 9 phase B, the ESPHome session's measurement) a used board: its stored record has remainders below a unit
 * (powered 1000 x 0.1 h + 300 s, lit 400 x 0.1 h + 350 s; 9000 Wh, 500 power-ups), then a month of 100 standby wakes a
 * day, every 10th worth a write of its own (lit 2 min at 30 W: 1 Wh; the others 1 min), an honest hub syncing at
 * every power-up from first contact on. The truth: the record's values and every session, apart from the ledger.
 * naive: the rules before (a plain max at every power-up, the power-ups' max before the + 1). */
static void sync_used(ebd_t *b, uint32_t naive, ls_fine_t *h, ls_fine_t *t)
{
    uint32_t d, k;
    memset(b, 0, sizeof *b);
    memset(h, 0, sizeof *h);
    memset(t, 0, sizeof *t);
    h->naive = naive;
    lf_init(&b->f, 0xFFFFu);
    bd_boot(b, 0, 1u, 0);
    b->e.lt.on_s = 1000u * 360u + 300u;
    b->e.lt.lit_s = 400u * 360u + 350u;
    b->e.base_wh = b->e.saved_wh = 9000u;               /* (energy_pack stores the total: the record's 9000 Wh) */
    b->e.lt.ups = 500u;
    b->e.lt.writes = 7u;
    (void)lf_store(&b->f, &b->e, LH);                    /* (the used record, stored) */
    t->s = 1000u * 360u + 300u;
    t->lit = 400u * 360u + 350u;
    t->mwh = 9000u * 1000u;
    t->ups = 500u;
    for (d = 0; d <= 30u; d++) {
        for (k = 0; k < 100u; k++) {
            bd_boot(b, 0, 1u, 0);
            b->e.ev_save = 1u;
            t->ups++;
            (void)ls_fine_sync(h, &b->e);
            if (d == 30u) {
                return;                                  /* (the last power-up: synced, compared) */
            }
            t->lit0 = b->e.lt.lit_s;
            b->lit = 1;
            bd_fast(b, k % 10u == 9u ? 12u : 6u, 10u, 30000u);   /* lit 2 min (1 Wh: a write) or 1 min at 30 W */
            b->lit = 0;
            bd_fast(b, 18u, 10u, 1000u);                 /* dark 3 min */
            t->s += b->s;
            t->lit += b->e.lt.lit_s - t->lit0;
            t->mwh += b->e.boot_mwh;
            ls_fine_end(h, b);
            bd_planned(b);
        }
    }
}

static void test_energy_ledger_used(void)
{
    static ebd_t b;
    ls_fine_t h, t;
    uint32_t k, w[2], err[2];
    int64_t ds[2], dl[2], dm[2], du[2];
    printf("energy: LIFE_SYNC's ledger on a used board (stored remainders 300 s / 350 s, every 10th wake stored)\n");
    for (k = 0; k < 2u; k++) {
        sync_used(&b, k, &h, &t);
        ds[k] = (int64_t)h.s - (int64_t)t.s;
        dl[k] = (int64_t)h.lit - (int64_t)t.lit;
        dm[k] = (int64_t)h.mwh - (int64_t)t.mwh;
        du[k] = (int64_t)h.ups - (int64_t)t.ups;
        err[k] = ls_fine_err(&t, &b.e);
        w[k] = b.f.writes;
        printf("  %s: the ledger off by %lld s, lit %lld s, %lld mWh, power-ups %lld; the receiver within %u units of "
               "the truth (%u writes)\n", k ? "the old rules (plain max, max before + 1)" :
               "the rules (lead beyond one unit, + 1 first)", (long long)ds[k], (long long)dl[k], (long long)dm[k],
               (long long)du[k], err[k], w[k]);
    }
    CHECK(ds[0] <= 0 && ds[0] > -360 && dl[0] <= 0 && dl[0] > -360 && dm[0] <= 0 && dm[0] > -1000 && du[0] == 0 &&
          err[0] <= 1u, "the ledger on a used board: never above the truth, below it by the stored remainders only "
          "(%lld s, %lld lit s, %lld mWh, power-ups %lld; the receiver %u units)", (long long)ds[0], (long long)dl[0],
          (long long)dm[0], (long long)du[0], err[0]);
    CHECK(dl[1] >= 360 && du[1] > 0, "the old rules on a used board (the control): over the truth (lit %lld s, "
          "power-ups %lld; powered %lld s)", (long long)dl[1], (long long)du[1], (long long)ds[1]);
}

/* (round 9 phase B) a hub's first-contact energy, TOTAL_WH x 1000 + BOOT_MWH mod 1000 (the ESPHome ledger's): the
 * board's own count, never above the truth. TOTAL_WH = the record's whole Wh + merged Wh + floor(BOOT_MWH / 1000)
 * (energy_total_wh), and the record keeps whole Wh only (a write drops the session's part below a Wh), so the sum is
 * (stored + merged) x 1000 + BOOT_MWH. */
static void test_energy_contact_wh(void)
{
    static ebd_t b;
    uint8_t pg[24];
    uint64_t truth, f;
    ls_led_t l;
    printf("energy: a hub's first-contact energy (TOTAL_WH x 1000 + BOOT_MWH mod 1000): the board's count, bounded\n");
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    bd_boot(&b, 0, 1u, 0);
    b.e.base_wh = b.e.saved_wh = 9000u;                  /* the record: 9000 Wh (the truth: 9000.123 Wh, the record */
    (void)lf_store(&b.f, &b.e, LH);                      /* dropped the 123 mWh) */
    truth = 9000123u;
    bd_boot(&b, 0, 1u, 0);
    l = ls_live(&b.e);
    l.wh += 50u;                                         /* a hub's merge: +50 Wh of its own (true) count */
    (void)ls_sync(&b.e, &l);
    truth += 50000u;
    b.e.boot_mwh += 12345u;                              /* this session: 12.345 Wh */
    truth += 12345u;
    energy_second(&b.e, 1000u, 1u, 0, 0, EN_TEMP_NA);
    energy_page(&b.e, 1u, 0, pg);
    f = (uint64_t)ultra_get32(pg + ULTRA_EN_TOTAL_WH) * 1000u + ultra_get32(pg + ULTRA_EN_BOOT_MWH) % 1000u;
    CHECK(f == 9062345u && f <= truth, "first-contact energy: %llu mWh = (stored + merged) x 1000 + BOOT_MWH, <= the "
          "truth %llu", (unsigned long long)f, (unsigned long long)truth);
    (void)lf_store(&b.f, &b.e, LH);                      /* a write (energy_pack): whole Wh only */
    CHECK(b.e.lt.wh == 9062u, "the record keeps whole Wh only (%u)", b.e.lt.wh);
    bd_boot(&b, 0, 1u, 0);
    energy_page(&b.e, 0, 0, pg);
    f = (uint64_t)ultra_get32(pg + ULTRA_EN_TOTAL_WH) * 1000u + ultra_get32(pg + ULTRA_EN_BOOT_MWH) % 1000u;
    CHECK(f == 9062000u && f <= truth, "first contact after the write: %llu mWh (the record's whole Wh), <= the truth "
          "%llu", (unsigned long long)f, (unsigned long long)truth);
    printf("  %llu mWh at first contact during the session, %llu after a write and a power-up; the truth %llu\n",
           9062345ull, (unsigned long long)f, (unsigned long long)truth);
}

static void test_energy_sync(void)
{
    static energy_t e;
    static ebd_t b;
    ls_led_t l;
    life_rec_t r;
    uint8_t a[23];
    uint32_t m, k, found, w[3], err, ok;
    printf("energy: the hub's counters (LIFE_SYNC): the merge, its steps, the guards, once a boot, the allowance\n");
    /* 1. merged; holds through energy_second; the energy total jumps; counts as stored (no write due) */
    ls_board(&e);
    l = ls_live(&e);
    l.on += 20u; l.lit += 10u; l.ups += 30u; l.ovl += 2u; l.dips += 2u; l.wh += 50u;
    m = ls_sync(&e, &l);
    CHECK(m == (LS_ALL | ULTRA_LS_R_APPLIED << 8) && e.lt.on_s == 3607200u && e.lt.lit_s == 1443600u &&
          e.lt.ups == 531u && e.lt.trips[0] == 5u && e.lt.trips[1] == 1u && e.lt.dips == 22u &&
          energy_total_wh(&e) == 9050u && e.lt.merged == 20u, "the hub ahead: every counter raised (%04X)", m);
    for (k = 1; k <= 3u; k++) {
        energy_second(&e, k * 1000u, k, 0, 0, 0);
    }
    CHECK(e.lt.on_s == 3607203u && energy_total_wh(&e) == 9050u && e.lt.on_s - e.saved_on_s == 3u &&
          energy_total_wh(&e) == e.saved_wh && !e.due && !(e.dip_ok & 2u), "held through energy_second (%u s, %u Wh), "
          "counted as stored: nothing due", e.lt.on_s, energy_total_wh(&e));
    /* 2. nothing lowered; a no-op doesn't use up the boot's merge */
    for (k = 0; k < 7u; k++) {
        ls_board(&e);
        l = ls_live(&e);
        if (k == 0u) l.on--;
        if (k == 1u) l.lit--;
        if (k == 2u) l.ups--;
        if (k == 3u) l.ovl--;
        if (k == 4u) l.sht--;
        if (k == 5u) l.dips--;
        if (k == 6u) l.wh--;
        m = ls_sync(&e, &l);
        CHECK(m == ULTRA_LS_R_NOOP << 8 && e.lt.on_s == 3600000u && e.lt.ups == 501u && e.lt.trips[1] == 1u &&
              energy_total_wh(&e) == 9000u && !e.synced, "field %u behind: nothing lowered, a no-op (%04X)", k, m);
    }
    /* 3. every step at its limit and one past it */
    ls_cap("powered", 0, ULTRA_LS_F_POWERED, LIFE_SYNC_MAX_H10);
    ls_cap("lit (powered +10)", 10u, ULTRA_LS_F_LIT, 11u);
    ls_cap("power-ups", 0, ULTRA_LS_F_UPS, LIFE_SYNC_EV_MIN);
    ls_cap("power-ups (powered +30 d)", LIFE_SYNC_MAX_H10, ULTRA_LS_F_UPS, 259264u);
    ls_cap("trips", 0, ULTRA_LS_F_TRIPS, LIFE_SYNC_EV_MIN);
    ls_cap("dips (powered +1 h)", 10u, ULTRA_LS_F_DIPS, 424u);
    ls_cap("energy", 0, ULTRA_LS_F_WH, 1000u);
    ls_cap("energy (powered +30 d)", LIFE_SYNC_MAX_H10, ULTRA_LS_F_WH, 721000u);
    ls_board(&e);                                        /* `merged` near its end */
    e.lt.merged_hi = 0xFFu;
    e.lt.merged = 0xFFFFu - 5u;
    l = ls_live(&e);
    l.on += 6u;
    m = ls_sync(&e, &l);
    CHECK(m == ULTRA_LS_R_CAPPED << 8 && LIFE_MERGED(&e.lt) == 0xFFFFFAu, "merged powered time saturating (24 bits): "
          "ignored (%04X)", m);
    /* 4. another board's UID16, another RECORDS: nothing */
    ls_board(&e);
    l = ls_live(&e);
    l.ups += 5u;
    ls_args(a, &e, &l);
    m = energy_sync(&e, a, LS_UID ^ 1u);
    k = energy_sync(&e, (a[ULTRA_LS_RECORDS]++, a), LS_UID);
    CHECK(m == ULTRA_LS_R_UID << 8 && k == ULTRA_LS_R_RECORDS << 8 && e.lt.ups == 501u && !e.synced,
          "UID16 / RECORDS not the board's: nothing merged (%u, %u)", m, k);
    /* 5. once a boot */
    m = ls_sync(&e, &l);
    l.ups += 5u;
    k = ls_sync(&e, &l);
    CHECK(m == (ULTRA_LS_F_UPS | ULTRA_LS_R_APPLIED << 8) && k == ULTRA_LS_R_ONCE << 8 && e.lt.ups == 506u,
          "one merge a boot (%04X, then %u)", m, k);
    /* 6. the allowance counts the board's own time only, across a reboot too */
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    bd_boot(&b, 0, 1u, 0);
    b.e.lt.writes = LIFE_RESERVE - 1u;
    bd_ms(&b, 700000u);
    bd_dip(&b);                                          /* the reserve's last write: exhausted */
    bd_boot(&b, 0, 1u, 0);
    l = ls_live(&b.e);
    l.on += LIFE_SYNC_MAX_H10;
    m = ls_sync(&b.e, &l);
    bd_fast(&b, 2u, 1u, 0);
    w[0] = b.e.held;
    (void)lf_store(&b.f, &b.e, LH);                      /* (stored all the same: as by a write) */
    bd_boot(&b, 0, 1u, 0);
    bd_fast(&b, 2u, 1u, 0);
    CHECK((m >> 8) == ULTRA_LS_R_APPLIED && w[0] && b.e.held && b.e.lt.merged == LIFE_SYNC_MAX_H10 &&
          b.e.lt.on_s - b.e.lt.merged * 360u < 710u, "30 days merged: still exhausted, after a reboot too (merged %u, "
          "own %u s)", b.e.lt.merged, b.e.lt.on_s - b.e.lt.merged * 360u);
    /* 8. merged, then the board's own write: the record holds the merged values */
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    bd_boot(&b, 0, 1u, 0);
    l = ls_live(&b.e);
    l.ups += 40u; l.wh += 300u;
    (void)ls_sync(&b.e, &l);
    bd_ms(&b, 700000u);
    bd_dip(&b);
    r = bd_stored(&b, &found);
    CHECK(found && r.ups == 41u && r.wh == 300u && r.magic == LIFE_MAGIC, "merged, then an own write: stored with "
          "it (%u ups, %u Wh)", r.ups, r.wh);
    /* 9. a dip's write, then a merge, then the power goes: the record from before the merge */
    l = ls_live(&b.e);
    bd_boot(&b, 0, 1u, 0);
    bd_ms(&b, 700000u);
    bd_dip(&b);
    l = ls_live(&b.e);
    l.ups += 9u;
    (void)ls_sync(&b.e, &l);
    bd_boot(&b, 0, 1u, 0);
    CHECK(b.e.lt.ups == 43u, "merged after the last write, then cut: the record before it (%u ups)", b.e.lt.ups);
    /* 10. the limits: saturated trips, the 24-bit counters' wrap, the board ahead */
    ls_board(&e);
    e.lt.trips[0] = 0xFFFFu;
    e.lt.trips[1] = 0xFFF0u;
    e.lt.ups = 0x01FFFFF0u;
    e.lt.dips = 0x00FFFFFFu;
    l = ls_live(&e);
    l.ovl = 0xFFFFu; l.sht = 0xFFFFu; l.ups = 0x000010u; l.dips = 0x000004u;
    m = ls_sync(&e, &l);
    CHECK(m == ((ULTRA_LS_F_UPS | ULTRA_LS_F_TRIPS | ULTRA_LS_F_DIPS) | ULTRA_LS_R_APPLIED << 8) &&
          e.lt.trips[0] == 0xFFFFu && e.lt.trips[1] == 0xFFFFu && e.lt.ups == 0x02000010u && e.lt.dips == 0x01000004u,
          "saturated trips stay, a counter across its 24-bit wrap (%04X)", m);
    ls_board(&e);
    e.lt.ups = 0x00900001u;                              /* the board ahead by 2^23 + 1, and by 1 */
    l = ls_live(&e);
    l.ups = 0x00100000u;
    m = ls_sync(&e, &l);
    l.ups = 0x00900000u;
    k = ls_sync(&e, &l);
    CHECK(!(m & 0xFFu) && k == ULTRA_LS_R_NOOP << 8 && e.lt.ups == 0x00900001u,
          "the board ahead (by 2^23 + 1, by 1): not merged (%04X, %04X)", m, k);
    /* 11. 2 years of 100 standby cycles a day: the same writes with or without a hub, honest (the spec's own ledger:
     * the board within one unit of the truth) or hostile */
    w[0] = sync_run(&b, 730u, 100u, 0, &err, &ok);
    w[1] = sync_run(&b, 730u, 100u, 1u, &err, &ok);
    k = err;
    m = ok;
    w[2] = sync_run(&b, 730u, 100u, 2u, &err, &ok);
    printf("  2 years, 100 cycles a day: %u writes without a hub, %u with one (odometer off by %u at most), %u with a "
           "hostile one (its time not the allowance's: %u)\n", w[0], w[1], k, w[2], ok);
    CHECK(w[1] <= w[0] + 1u && w[0] <= w[1] + 1u && k <= 1u && m, "an honest hub: the same writes (%u / %u), the "
          "odometer within 1 unit (%u)", w[0], w[1], k);
    CHECK(w[2] <= w[0] + 1u && ok, "a hostile hub: no more writes (%u / %u), the allowance on the board's own time",
          w[2], w[0]);
    /* 10 years, the allowance deciding: the writes follow the board's own powered time, whatever the hub */
    for (k = 0; k < 4u; k++) {
        uint32_t own, late, n = sync_run10(&b, k, &own, &late);
        printf("  10 years, 100 wakes of 1 Wh a day, the reserve used up, %s: %u writes (own time earns %u), %u "
               "powered merges in year 10, merged %u x 0.1 h\n", k == 0u ? "no hub" : k == 1u ? "an honest hub" :
               k == 2u ? "a hostile hub" : "a hub landing at the boundaries", n, own / LIFE_EARN_S, late,
               LIFE_MERGED(&b.e.lt));
        CHECK(n <= own / LIFE_EARN_S + 1u && (k != 1u || late >= 300u), "10 years, hub %u: the writes within the own "
              "time's earn (%u / %u)%s", k, n, own / LIFE_EARN_S, k == 1u ? ", powered merges still applied" : "");
    }
}

/* (0.6.10) a record whose CRC would read erased: one more lit second, still one count (the allowance's) */
static void test_energy_erased_crc(void)
{
    static energy_t e;
    uint32_t k, w, c;
    printf("energy: a record whose CRC would read erased: one count a write\n");
    memset(&e, 0, sizeof e);
    e.lt.magic = LIFE_MAGIC;
    e.lt.writes = 41u;
    for (k = 0; k < 2000000u; k++) {                     /* (find a lit time that gives an erased-looking CRC) */
        e.lt.lit_s = k;
        e.lt.writes = 42u;
        if (crc16_erased(crc16_ccitt((const uint8_t *)&e.lt, LIFE_REC_BYTES - 2u))) {
            break;
        }
    }
    CHECK(k < 2000000u, "an erased-looking CRC found (lit %u)", k);
    e.lt.writes = 41u;
    w = e.lt.writes;
    energy_pack(&e);
    c = crc16_ccitt((const uint8_t *)&e.lt, LIFE_REC_BYTES - 2u);
    CHECK(e.lt.writes == w + 1u && e.lt.lit_s == k + 1u && c == e.lt.crc && !crc16_erased(c), "packed: writes %u -> %u "
          "(one), lit %u -> %u, CRC %04X", w, e.lt.writes, k, e.lt.lit_s, c);
}

/* format 3 (0.6.10): a format 2 record read and carried over; a 0.6.9 reader never takes a format 3 record */
static int ls_f2_valid(const uint8_t *s)                 /* (0.6.9's life_valid) */
{
    uint32_t c = (uint32_t)s[46] | (uint32_t)s[47] << 8;
    return s[45] == 0x4Du && !crc16_erased(c) && c == crc16_ccitt(s, 46u);
}

static void test_energy_format3(void)
{
    static ebd_t b;
    life_rec_t r;
    uint8_t *p;
    uint32_t found, k, n = 0;
    printf("energy: the record's format 3: format 2 carried over, never taken by a 0.6.9 reader\n");
    memset(&b, 0, sizeof b);
    lf_init(&b.f, 0xFFFFu);
    p = b.f.pg[0].b;                                     /* a 0.6.9 record: the version word at 20..23 */
    memset(p, 0, LIFE_REC_BYTES);
    p[4] = 0x10; p[5] = 0x0E;                            /* on_s 3600 */
    p[12] = 7;                                           /* 7 power-ups */
    p[20] = 0xFF; p[21] = 0x09; p[22] = 0x06; p[23] = 0x00;   /* 0x000609FF */
    p[24] = 12;                                          /* writes */
    p[45] = 0x4Du;
    k = crc16_ccitt(p, 46u);
    p[46] = (uint8_t)k;
    p[47] = (uint8_t)(k >> 8);
    bd_boot(&b, 0, 0x000609FFu, 0);
    CHECK(b.e.lt.ver == energy_ver(0x000609FFu) && LIFE_MERGED(&b.e.lt) == 0u && b.e.lt.ups == 8u && !b.e.lt.updates &&
          b.e.lt.on_s == 3600u, "a format 2 record carried over, the same version: no update (%04X)", b.e.lt.ver);
    bd_ms(&b, 700000u);
    bd_dip(&b);
    r = bd_stored(&b, &found);
    CHECK(found && r.magic == LIFE_MAGIC && r.writes == 13u && b.f.pg[0].b[LIFE_REC_BYTES + 45u] == LIFE_MAGIC &&
          !b.f.bad, "the next write: format 3, after it on the page (%u)", r.writes);
    bd_boot(&b, 0, 0x00060AFFu, 0);
    CHECK(b.e.lt.updates == 1u && b.e.lt.ver == energy_ver(0x00060AFFu), "a new version: an update");
    for (k = 0; k < 12u; k++) {                          /* both pages full of format 3 records */
        bd_ms(&b, 700000u);
        bd_dip(&b);
        bd_boot(&b, 0, 0x00060AFFu, 0);
    }
    for (k = 0; k < 2u * LIFE_RECS; k++) {
        n += ls_f2_valid(b.f.pg[k / LIFE_RECS].b + (k % LIFE_RECS) * LIFE_REC_BYTES);
    }
    CHECK(n == 0u && b.f.pg[0].b[45] == LIFE_MAGIC && b.f.pg[1].b[45] == LIFE_MAGIC, "a 0.6.9 reader takes none of "
          "them (%u)", n);
}

/* ---- (round 9 phase B, R9-B1) the counters' ceiling: a merge never wraps them; the own counting stops there ---- */
#define LS_MAX 0xFFFFFFFFu

/* a board as booted from a record with these seconds / Wh and merged (0.1 h) */
static void ls_board_at(energy_t *e, uint32_t on_s, uint32_t lit_s, uint32_t wh, uint32_t merged)
{
    memset(e, 0, sizeof *e);
    e->lt.on_s = on_s;
    e->lt.lit_s = lit_s;
    e->lt.ups = 500u;
    e->lt.wh = wh;
    e->lt.writes = 7u;
    e->lt.merged_hi = (uint8_t)(merged >> 16);
    e->lt.merged = (uint16_t)merged;
    e->lt.magic = LIFE_MAGIC;
    energy_init(e, 1u, 0, 0, 0);
}

/* the review's reachable sequence: boots of 600 s own time, each with a hostile +30 d powered merge and a planned
 * write; the wire's powered time never lower, the merges CAPPED at the ceiling, the own time exact until on_s stops
 * there, never more than the board's own after it. Returns the boot of the first CAPPED merge (0: none) */
static uint32_t ls_reach(uint32_t boots, uint32_t *bad, uint32_t *sat)
{
    static energy_t e;
    ls_led_t l;
    uint32_t b, m, before, own, first = 0;
    *bad = *sat = 0;
    memset(&e, 0, sizeof e);
    energy_init(&e, 0u, 0, 0, 0);
    for (b = 1; b <= boots; b++) {
        if (b > 1u) {
            energy_init(&e, 1u, 0, 0, 0);
        }
        e.ev_save = 1u;
        l = ls_live(&e);
        l.on += LIFE_SYNC_MAX_H10;
        before = e.lt.on_s;
        m = ls_sync(&e, &l);
        if ((m >> 8) == ULTRA_LS_R_CAPPED && !first) {
            first = b;
        }
        *bad += e.lt.on_s < before || ((m >> 8) == ULTRA_LS_R_CAPPED && (m & ULTRA_LS_F_POWERED)) ||
                (first && (m >> 8) != ULTRA_LS_R_CAPPED);
        before = e.lt.on_s;
        energy_second(&e, 600000u, 600u, 0, 0, EN_TEMP_NA);
        *bad += e.lt.on_s < before;
        if (energy_due(&e, 600000u, 0, ENERGY_PLANNED | ENERGY_DUE_DIP) == ENERGY_DUE_DIP) {
            energy_pack(&e);                             /* (the planned cut's write: the 600 s to store) */
            energy_saved(&e, 600u);
        } else {
            *bad += e.lt.on_s != LS_MAX;                 /* (only at the ceiling: less than a write's worth new) */
        }
        own = e.lt.on_s - LIFE_MERGED(&e.lt) * 360u;
        if (e.lt.on_s == LS_MAX) {
            *sat += 1u;
            *bad += own > b * 600u;                      /* (stopped: never more than its own) */
        } else {
            *bad += own != b * 600u;                     /* (exact) */
        }
    }
    return first;
}

static void test_energy_ceiling(void)
{
    static energy_t e;
    ls_led_t l;
    uint32_t m, k, s, bad, sat, on, lit, wh;
    printf("energy: the counters' ceiling (2^32 - 1): a merge never wraps them, the own counting stops there\n");
    /* 1. powered: a merge past on_s's room is CAPPED, the field unapplied (0.6.10 c4719e5: APPLIED, on_s 104) */
    ls_board_at(&e, 0xFFFFFF00u, 1000u, 9000u, 11930464u);
    l = ls_live(&e);
    l.on += 1u;
    m = ls_sync(&e, &l);
    CHECK((m >> 8) == ULTRA_LS_R_CAPPED && !(m & ULTRA_LS_F_POWERED) && e.lt.on_s == 0xFFFFFF00u &&
          LIFE_MERGED(&e.lt) == 11930464u, "ceiling: powered +1 at 0xFFFFFF00 s: %04X, on_s %u, merged %u", m,
          e.lt.on_s, LIFE_MERGED(&e.lt));
    /* 2. exactly the room: applied; one more: CAPPED */
    for (k = 0; k < 2u; k++) {
        ls_board_at(&e, LS_MAX - 5u * 360u - 7u, 1000u, 9000u, 0);
        l = ls_live(&e);
        l.on += 5u + k;
        m = ls_sync(&e, &l);
        CHECK(k ? (m >> 8) == ULTRA_LS_R_CAPPED && e.lt.on_s == LS_MAX - 5u * 360u - 7u :
                  (m >> 8) == ULTRA_LS_R_APPLIED && (m & ULTRA_LS_F_POWERED) && e.lt.on_s == LS_MAX - 7u,
              "ceiling: powered +%u with room for 5: %04X, on_s %u", 5u + k, m, e.lt.on_s);
    }
    /* 3. lit: the same at lit_s's ceiling */
    ls_board_at(&e, 3600000u, 0xFFFFFF00u, 9000u, 0);
    l = ls_live(&e);
    l.lit += 1u;
    m = ls_sync(&e, &l);
    CHECK((m >> 8) == ULTRA_LS_R_CAPPED && !(m & ULTRA_LS_F_LIT) && e.lt.lit_s == 0xFFFFFF00u,
          "ceiling: lit +1 at 0xFFFFFF00 s: %04X, lit_s %u", m, e.lt.lit_s);
    /* 4. the energy: plain, a lower hub value never (c4719e5: 5 came out 21 ahead of 0xFFFFFFF0: APPLIED, 5 Wh) */
    ls_board_at(&e, 3600000u, 1000u, 0xFFFFFFF0u, 0);
    l = ls_live(&e);
    l.wh = 5u;
    m = ls_sync(&e, &l);
    CHECK(m == (uint32_t)ULTRA_LS_R_NOOP << 8 && energy_total_wh(&e) == 0xFFFFFFF0u,
          "ceiling: Wh 5 against 0xFFFFFFF0: %04X, total %u", m, energy_total_wh(&e));
    l.on += 20u;
    l.wh = LS_MAX;
    m = ls_sync(&e, &l);
    CHECK((m >> 8) == ULTRA_LS_R_APPLIED && (m & ULTRA_LS_F_WH) && energy_total_wh(&e) == LS_MAX,
          "ceiling: Wh up to 2^32 - 1: %04X, total %u", m, energy_total_wh(&e));
    /* 5. the board's own counting at the ceiling: on_s, lit_s and the energy stay there, never wrap */
    ls_board_at(&e, LS_MAX - 3u, LS_MAX - 3u, LS_MAX - 1u, 0);
    on = e.lt.on_s, lit = e.lt.lit_s, wh = energy_total_wh(&e), bad = 0;
    for (s = 1; s <= 10u; s++) {
        e.lit_ms = s * 1000u;
        e.boot_mwh += 1500u;                             /* (1.5 Wh a second) */
        energy_second(&e, s * 1000u, s, 0, 0, EN_TEMP_NA);
        bad += e.lt.on_s < on || e.lt.lit_s < lit || energy_total_wh(&e) < wh;
        on = e.lt.on_s, lit = e.lt.lit_s, wh = energy_total_wh(&e);
    }
    CHECK(!bad && on == LS_MAX && lit == LS_MAX && wh == LS_MAX && e.lt.on_s - LIFE_MERGED(&e.lt) * 360u == LS_MAX,
          "ceiling: own counting 10 s from 3 s below: %u lower, on_s %u, lit_s %u, Wh %u", bad, on, lit, wh);
    energy_pack(&e);
    CHECK(e.lt.on_s == LS_MAX && e.lt.lit_s >= LS_MAX - 1u && e.lt.wh == LS_MAX,
          "ceiling: the record packed at it: on_s %u, lit_s %u, Wh %u", e.lt.on_s, e.lt.lit_s, e.lt.wh);
    /* 6. a merge to 100 s below the ceiling, then the next ticks */
    ls_board_at(&e, LS_MAX - LIFE_SYNC_MAX_H10 * 360u - 100u, 1000u, 9000u, 0);
    l = ls_live(&e);
    l.on += LIFE_SYNC_MAX_H10;
    m = ls_sync(&e, &l);
    on = e.lt.on_s, bad = 0;
    for (s = 1; s <= 200u; s++) {
        energy_second(&e, s * 1000u, s, 0, 0, EN_TEMP_NA);
        bad += e.lt.on_s < on;
        on = e.lt.on_s;
    }
    CHECK((m & ULTRA_LS_F_POWERED) && !bad && on == LS_MAX && LIFE_MERGED(&e.lt) == LIFE_SYNC_MAX_H10,
          "ceiling: merged to 100 s below, 200 ticks: %04X, %u lower, on_s %u", m, bad, on);
    /* 7. the review's reachable sequence: 6000 boots (the merges stop at ~1657, on_s at the ceiling ~4200 later) */
    k = ls_reach(6000u, &bad, &sat);
    CHECK(!bad && k > 1600u && k < 1700u && sat > 0u, "ceiling: 6000 boots of +30 d merges: %u wrong, first CAPPED at "
          "boot %u, %u boots at the ceiling", bad, k, sat);
    printf("  reachable sequence: merges CAPPED from boot %u, on_s at its ceiling for the last %u boots, never lower;"
           " the own time exact before\n", k, sat);
}

static void test_energy(void)
{
    test_energy_ch();
    test_energy_rules();
    test_energy_record();
    test_energy_powercut_all();
    test_energy_boots();
    test_energy_wear();
    test_energy_standby();
    test_energy_sync();
    test_energy_ledger();
    test_energy_ledger_round();
    test_energy_ledger_used();
    test_energy_contact_wh();
    test_energy_format3();
    test_energy_erased_crc();
    test_energy_ceiling();
}
