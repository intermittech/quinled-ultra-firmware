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
 *  - power cuts: a write cut after each of its 20 half-words, at every place of the page pair; an erase cut short (the
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

/* life_store: `cut` = the half-words programmed before the power goes (20 = all) */
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
    if (w != ENERGY_DUE_NONE && lf_store(&b->f, &b->e, 20u)) {
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
    CHECK(b.e.lt.ups == 1u && b.e.lt.updates == 0u && b.e.lt.ver == 0x00060300u && b.e.ev,
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
        CHECK(lf_store(&b.f, &b.e, 20u) == 1u, "stored");
        r = bd_stored(&b, &found);
        CHECK(found && r.wh == 1000u + 1284u && r.on_s == 77u && r.lit_s == 66u && r.ups == 1u && r.dips == 14u &&
              r.ver == 0x00060300u && r.writes == 1u && r.trips[0] == 3u && r.trips[1] == 2u && r.updates == 0u &&
              r.wd == 1u && r.peak_t == 137u && r.peak_a == 79u && r.magic == LIFE_MAGIC,
              "every field back (%04X)", erased);
        CHECK(b.f.pos.next == 1u && !b.f.pos.erase, "next: entry 1");
        {                                                /* the writes counter across its 32-bit wrap */
            static ebd_t w;
            uint32_t i;
            w = b;
            lf_init(&w.f, erased);
            w.e.lt.writes = 0xFFFFFFFDu;
            for (i = 0; i < 4u; i++) {
                (void)lf_store(&w.f, &w.e, 20u);         /* 0xFFFFFFFE, 0xFFFFFFFF, 0, 1 */
            }
            found = lf_load(&w.f, &r);
            CHECK(found && r.writes == 1u && w.f.pos.next == 4u, "the writes counter across its wrap: the newest is "
                  "1 (%u)", r.writes);
        }
        /* page 0's last entry: page 1 is next, its erase pending (a write never erases); no write until the erase */
        for (k = 0; k < 5u; k++) {
            (void)lf_store(&b.f, &b.e, 20u);
        }
        CHECK(b.f.pos.next == 16u && b.f.pos.erase && b.f.erases[0] + b.f.erases[1] == 0u, "6 records: page 0 full, "
              "page 1 next, its erase pending, nothing erased yet");
        CHECK(lf_store(&b.f, &b.e, 20u) == 0u, "a write now: none (no erased entry)");
        lf_prepare(&b.f, 0);
        CHECK(!b.f.pos.erase && b.f.erases[1] == 1u && lf_store(&b.f, &b.e, 20u) == 1u, "erased (dark): page 1 written");
        /* page 1 full: page 0 (records 1-6) pending; a boot sees the same and erases it before the outputs start */
        for (k = 0; k < 5u; k++) {
            (void)lf_store(&b.f, &b.e, 20u);
        }
        found = lf_load(&b.f, &r);                       /* the next boot */
        CHECK(found && r.writes == 12u && b.f.pos.next == 0u && b.f.pos.erase && b.f.erases[0] == 0u,
              "12 records, a boot: page 0 next, to be erased (%u)", r.writes);
        lf_prepare(&b.f, 0);
        CHECK(b.f.erases[0] == 1u && lf_store(&b.f, &b.e, 20u) == 1u && b.f.bad == 0u, "erased at power-up, written");
        r = bd_stored(&b, &found);
        CHECK(found && r.writes == 13u, "13 records (%u)", r.writes);

        /* power cuts: a write cut after each half-word, at each place: entry 1 (mid-page), entry 5 (it fills the
         * page), the first entry of a page just erased */
        for (place = 0; place < 3u; place++) {
            for (cut = 0; cut <= 20u; cut++) {
                life_rec_t before, after;
                uint32_t f2, prev;
                c = b;
                if (place >= 1u) {
                    while ((c.f.pos.next & 15u) != 5u) {
                        (void)lf_store(&c.f, &c.e, 20u);
                    }
                    if (place == 2u) {
                        (void)lf_store(&c.f, &c.e, 20u); /* fills it; the next page erased */
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
                CHECK(f2 && (after.writes == (cut == 20u ? prev + 1u : prev) || (cut == 19u && after.writes == prev + 1u)),
                      "place %u, cut after %u half-words: the newest is write %u (%u before)", place, cut,
                      after.writes, prev);
                /* the boot's erase (if one is pending); the next write goes into an erased entry, the newest */
                lf_prepare(&c.f, 0);
                (void)lf_store(&c.f, &c.e, 20u);
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
            while ((c.f.pos.next & 15u) != 5u) {
                (void)lf_store(&c.f, &c.e, 20u);
            }
            (void)lf_store(&c.f, &c.e, 20u);             /* fills the page: the other one is erased ... */
            w = c.e.lt.writes;
            p = (c.f.pos.next >> 4) & 1u;
            c.f.pg[p] = c.f.pg[p ^ 1u];                  /* ... cut short: it held older records (here: these) */
            for (i = 0; i < 6u; i++) {                   /* as older records */
                life_rec_t *o = (life_rec_t *)(c.f.pg[p].b + i * LIFE_REC_BYTES);
                o->writes -= 6u;
                o->crc = crc16_ccitt((const uint8_t *)o, 38u);
            }
            for (i = 0; i < 64u; i++) {                  /* half erased: bits go to 1 (or garbage, every 4th run) */
                seed ^= seed << 13; seed ^= seed >> 7; seed ^= seed << 17;
                c.f.pg[p].w[i] = (k & 3u) == 3u ? (uint32_t)seed : c.f.pg[p].w[i] | ((uint32_t)seed & (uint32_t)(seed >> 32));
            }
            f2 = lf_load(&c.f, &got);                    /* the next boot */
            CHECK(f2 && got.writes == w, "erase cut short (%u): the newest is still write %u (%u)", k, w, got.writes);
            e0 = c.f.erases[p];
            CHECK(c.f.pos.next == (p << 4) && c.f.pos.erase && lf_store(&c.f, &c.e, 20u) == 0u, "... the next write "
                  "waits for its erase (%u)", k);
            lf_prepare(&c.f, 0);
            CHECK(c.f.erases[p] == e0 + 1u && lf_store(&c.f, &c.e, 20u) == 1u && c.f.bad == 0u, "... erased at power-up "
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
    CHECK(found && r.ups == 1u && r.updates == 0u && r.ver == 0x00060300u, "boot 1: stored, no update");
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
    CHECK(b.e.lt.updates == 1u && b.e.lt.ver == 0x00060400u, "a development build: no update, version kept");
    bd_ms(&b, 700000u);
    bd_dip(&b);
    bd_boot(&b, 0, 0x00070000u, 1);                      /* v0.7.0 after a watchdog reset */
    CHECK(b.e.lt.updates == 2u && b.e.lt.wd == 1u, "v0.7.0 after a watchdog reset: update 2, wd 1");
    bd_ms(&b, 700000u);
    bd_dip(&b);
    r = bd_stored(&b, &found);
    CHECK(found && r.ups == 5u && r.updates == 2u && r.wd == 1u && r.ver == 0x00070000u, "stored: 5 power-ups (%u; "
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
    cl.st = &st;
    e.boot_mwh = 76540;
    e.ch_mwh[0] = 72000; e.ch_mwh[1] = 5; e.ch_mwh[2] = 1440; e.ch_mwh[3] = 0; e.ch_mwh[4] = 123456;
    run_cli(&cl, "status\r");
    CHECK(strstr(cli_out, "energy=76.54 Wh since boot\r\nenergy R/G/B/W1/W2=72.00/0.01/1.44/0.00/123.46 Wh\r\n") != NULL,
          "status: energy per channel");
}

static void test_energy(void)
{
    test_energy_ch();
    test_energy_rules();
    test_energy_record();
    test_energy_powercut_all();
    test_energy_boots();
    test_energy_wear();
}
