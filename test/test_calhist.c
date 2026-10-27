/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The calibration history (core/calhist.c), included by test_main.c (built with CALHIST_ADDR bound, as the
 * upper record layout would): the first test kept and the newest after it, the lit hours, a single-level test scaled
 * to 100 %, the trend, the record on an A/B page pair (pagepair.c: the newest wins, a torn record or a cut erase
 * leaves the one before), the console. The store itself on the flash model: the soak's cfg scenario (main.c). */
#include "../src/core/calhist.h"

/* a standard test: each channel's current at 25 / 50 / 75 / 100 % (linear from i100) */
static void ch_test(isense_cal_t *c, const uint16_t i100[5])
{
    uint32_t k, l;
    memset(c, 0, sizeof *c);
    c->levels = ISENSE_LEVELS;
    c->pct[0] = 25; c->pct[1] = 50; c->pct[2] = 75; c->pct[3] = 100;
    c->zero_ma = 30;
    for (k = 0; k < 5u; k++) {
        for (l = 0; l < ISENSE_LEVELS; l++) {
            c->ma[k][l] = (uint16_t)(i100[k] * (l + 1u) / 4u);
        }
    }
}

static void test_calhist(void)
{
    static const uint16_t base[5] = { 3000, 3000, 2500, 0, 5000 };
    static const pp_desc_t d = { 256u, CALHIST_REC_BYTES, CALHIST_RECS, 4u, 0u, calhist_valid };
    uint8_t pg[512];
    pp_pos_t pos;
    calhist_t h, r;
    isense_cal_t c;
    int8_t t[5];
    uint16_t v[5];
    uint32_t k, w;
    cli_t cl;
    cli_status_t st;
    const calhist_t *found;

    printf("calibration history: the first and the newest tests, the trend, the page pair, the console\n");
    memset(&h, 0, sizeof h);
    CHECK(calhist_trend(&h, t) == 0u && t[0] == ULTRA_HE_NA, "empty: no trend");
    ch_test(&c, base);
    calhist_add(&h, 3600u * 12u + 59u, &c);             /* the first: 12 h lit */
    CHECK(h.n == 1u && h.tests == 1u && h.e[0].lit_h == 12u && h.e[0].ma[0] == 3000u && h.e[0].ma[3] == 0u &&
          h.e[0].ma[4] == 5000u, "the first test (%u h)", h.e[0].lit_h);
    CHECK(calhist_trend(&h, t) == 0u && t[0] == ULTRA_HE_NA, "one test: no trend");
    /* W2 loses 1 % per test, R a segment (-10 %) at the 4th; 7 tests: the first and the latest kept */
    memcpy(v, base, sizeof v);
    for (k = 2; k <= 7u; k++) {
        v[4] = (uint16_t)(5000u - 50u * (k - 1u));
        if (k == 4u) {
            v[0] = 2700;
        }
        ch_test(&c, v);
        calhist_add(&h, 3600u * 1000u * k, &c);
    }
    CHECK(h.n == CALHIST_N && h.tests == 7u && h.e[0].lit_h == 12u &&
          h.e[1].lit_h == 7000u && h.e[1].ma[4] == 4700u, "7 tests: first and latest only");
    w = calhist_trend(&h, t);
    CHECK(w == 6988u && t[0] == -10 && t[1] == 0 && t[2] == 0 && t[3] == ULTRA_HE_NA && t[4] == -6,
          "trend over %u h: R %d, W1 %d, W2 %d %%", w, t[0], t[3], t[4]);
    /* a single-level test at 40 % is scaled to 100 %; the lit hours saturate */
    memset(&c, 0, sizeof c);
    c.levels = 1;
    c.pct[0] = 40;
    for (k = 0; k < 5u; k++) {
        c.ma[k][0] = (uint16_t)(base[k] * 40u / 100u);
    }
    calhist_add(&h, 0xFFFFFFFFu, &c);
    CHECK(h.n == CALHIST_N && h.e[CALHIST_N-1].ma[0] == 3000u && h.e[CALHIST_N-1].ma[4] == 5000u && h.e[CALHIST_N-1].lit_h == 65535u &&
          h.e[0].lit_h == 12u, "single level scaled, lit hours saturate");

    /* the record on a page pair: 6 a page; the newest wins; a torn record (no CRC yet) leaves the one before; an
     * erased-ahead page; a garbage (cut) erase */
    memset(pg, 0xFF, sizeof pg);
    memset(&h, 0, sizeof h);
    found = (const calhist_t *)pp_find(&d, pg, pg + 256, &pos);
    CHECK(found == 0 && pos.next == 0u && !pos.erase, "nothing stored: page 0, entry 0");
    for (k = 0; k < 11u; k++) {
        uint32_t at;
        ch_test(&c, base);
        calhist_add(&h, 3600u * k, &c);
        if (pp_erase_due(&pos, &w) == 2u) {
            memset(pg + 256u * w, 0xFF, 256);
            pp_erased(&pos);
        }
        at = pp_plan(&d, &pos);
        calhist_pack(&h);
        memcpy(pg + PP_PAGE(at) * 256u + PP_ENTRY(at) * CALHIST_REC_BYTES, &h, sizeof h);
    }
    found = (const calhist_t *)pp_find(&d, pg, pg + 256, &pos);
    CHECK(found && found->seq == 11u && found->tests == 11u && found->n == CALHIST_N && found->e[CALHIST_N-1].lit_h == 10u,
          "11 records: the newest (seq %u)", found ? found->seq : 0u);
    r = h;                                                /* a 12th torn before its CRC: the 11th */
    calhist_add(&r, 3600u * 11u, &c);
    calhist_pack(&r);
    memcpy(pg + PP_PAGE(pos.next) * 256u + PP_ENTRY(pos.next) * CALHIST_REC_BYTES, &r, sizeof r - 2u);
    found = (const calhist_t *)pp_find(&d, pg, pg + 256, &pos);
    CHECK(found && found->seq == 11u, "a torn record: the one before (seq %u)", found ? found->seq : 0u);
    r.crc ^= 1u;
    CHECK(!calhist_valid((const uint8_t *)&r), "a bad CRC: invalid");
    r.crc ^= 1u;
    r.n = CALHIST_N + 1u;
    calhist_pack(&r);
    CHECK(!calhist_valid((const uint8_t *)&r), "more entries than it holds: invalid");

    /* the console (status, after the calibration) */
    memset(&cl, 0, sizeof cl);
    memset(&st, 0, sizeof st);
    settings_default(&cli_set);
    st.version = "test";
    st.steps = 5760;
    st.en = &test_en;
    cl.write = cap_write;
    cl.apply = cap_apply;
    cl.save_now = cap_save;
    cl.set = &cli_set;
    cl.eff = &cli_set;
    cl.st = &st;
    run_cli(&cl, "status\r");
    CHECK(strstr(cli_out, "cal_history") == NULL && strstr(cli_out, "cal_trend") == NULL, "status: no history");
    memset(&h, 0, sizeof h);
    ch_test(&c, base);
    calhist_add(&h, 3600u * 12u, &c);
    v[0] = 2700; v[1] = 3000; v[2] = 2500; v[3] = 0; v[4] = 4800;
    ch_test(&c, v);
    calhist_add(&h, 3600u * 1012u, &c);
    st.ch = &h;
    run_cli(&cl, "status\r");
    CHECK(strstr(cli_out, "cal_history 12 h lit=3.00/3.00/2.50/0.00/5.00 A\r\n"
                          "cal_history 1012 h lit=2.70/3.00/2.50/0.00/4.80 A\r\n"
                          "cal_trend R/G/B/W1/W2=-10/0/0/-/-4 % over 1000 h lit, 2 tests\r\n") != NULL,
          "status: the history and its trend");
}
