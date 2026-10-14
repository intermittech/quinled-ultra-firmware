/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Channel health (core/health.c), included by test_main.c: the test's classes (normal, no load, not rising, far
 * above / below the test before, a lost load), the dark current at the test and while running, the running model's
 * drift against the stored test, single-level tests, the D9 code, and the HEALTH page; "calibration suggested"
 * (0.6.5, test_health_cal). */
#include "../src/core/health.h"

/* a standard test: each channel's current at 25 / 50 / 75 / 100 % (linear from i100 unless set after), the zero */
static void he_test(isense_cal_t *c, const uint16_t i100[5], int16_t zero)
{
    uint32_t ch, l;
    memset(c, 0, sizeof *c);
    c->levels = ISENSE_LEVELS;
    c->pct[0] = 25; c->pct[1] = 50; c->pct[2] = 75; c->pct[3] = 100;
    c->zero_ma = zero;
    for (ch = 0; ch < 5u; ch++) {
        for (l = 0; l < ISENSE_LEVELS; l++) {
            c->ma[ch][l] = (uint16_t)(i100[ch] * (l + 1u) / 4u);
        }
    }
}

/* 0.6.5: "calibration suggested" (HEALTH_F_CAL): the running model off the stored test for HEALTH_SUGGEST_S s while
 * the channel is on, latched until the next test; the HEALTH page bit, the console */
/* health_run, and whether this call raised HEALTH_F_CAL */
static uint32_t hc_run(health_t *h, const isense_t *is, const isense_cal_t *st)
{
    uint32_t was = h->flags & HEALTH_F_CAL;
    health_run(h, is, st);
    return (h->flags & HEALTH_F_CAL) && !was;
}

static void test_health_cal(void)
{
    static const uint16_t base[5] = { 3000, 3000, 2500, 0, 5000 };
    static const uint16_t small[5] = { 100, 0, 0, 0, 0 };
    isense_cal_t st, none;
    health_t h;
    isense_t is;
    cli_t c;
    cli_status_t cs;
    uint32_t s, raised = 0;
    uint8_t pg[24];

    printf("channel health: calibration suggested\n");
    health_init(&h);
    he_test(&st, base, 30);
    memset(&none, 0, sizeof none);
    isense_init(&is, 1, 10000, 8000);
    isense_cal_set(&is, &st);
    is.ch_ma[4] = 6750;                                   /* W2 +35 %, +1750 mA, on */
    is.applied[4] = 30000;
    for (s = 1; s < HEALTH_SUGGEST_S; s++) {
        raised |= hc_run(&h, &is, &st);
    }
    CHECK(!raised && !(h.flags & HEALTH_F_CAL) && (h.flags & ULTRA_HE_F_DRIFT) && h.drift_s == HEALTH_SUGGEST_S - 1u,
          "drifting %u s: not yet (%u)", s - 1u, h.drift_s);
    is.applied[4] = 0;                                    /* dark: neither counts nor resets */
    for (s = 0; s < 1000u; s++) {
        raised |= hc_run(&h, &is, &st);
    }
    CHECK(!raised && h.drift_s == HEALTH_SUGGEST_S - 1u, "dark: held (%u)", h.drift_s);
    is.applied[4] = 1;
    CHECK(hc_run(&h, &is, &st) != 0u && (h.flags & HEALTH_F_CAL), "raised at %u s on", HEALTH_SUGGEST_S);
    for (s = 0; s < 100u; s++) {
        raised |= hc_run(&h, &is, &st);
    }
    CHECK(!raised && (h.flags & HEALTH_F_CAL), "raised once, then latched");
    is.ch_ma[4] = 5000;                                   /* the model back at the test: still latched */
    (void)health_run(&h, &is, &st);
    CHECK((h.flags & HEALTH_F_CAL) && !(h.flags & ULTRA_HE_F_DRIFT) && h.drift_s == 0u, "latched without drift");
    health_page(&h, 400, 1, pg);
    CHECK((pg[ULTRA_HE_FLAGS] & HEALTH_F_CAL) != 0u, "HEALTH page: flags bit 4");

    /* the console: on the health_model line */
    memset(&c, 0, sizeof c);
    memset(&cs, 0, sizeof cs);
    settings_default(&cli_set);
    cs.version = "test";
    cs.steps = 5760;
    cs.en = &test_en;
    cs.hl = &h;
    c.write = cap_write;
    c.apply = cap_apply;
    c.save_now = cap_save;
    c.set = &cli_set;
    c.eff = &cli_set;
    c.st = &cs;
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "health_model R/G/B/W1/W2=0/0/0/-/0 %, dark 0.00 A, calibration suggested\r\n") != NULL,
          "status: calibration suggested");

    /* a new test clears it */
    health_test_start(&h, &st);
    (void)health_test_done(&h, &st);
    (void)health_run(&h, &is, &st);
    CHECK(!(h.flags & HEALTH_F_CAL), "a new test clears it");
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "calibration suggested") == NULL, "status: not after the test");

    /* a drift that goes away before the time starts counting again */
    is.ch_ma[4] = 6750;
    for (s = 0; s < HEALTH_SUGGEST_S - 10u; s++) {
        raised |= hc_run(&h, &is, &st);
    }
    is.ch_ma[4] = 5000;
    (void)health_run(&h, &is, &st);
    is.ch_ma[4] = 3250;                                   /* -35 % */
    for (s = 0; s < HEALTH_SUGGEST_S - 1u; s++) {
        raised |= hc_run(&h, &is, &st);
    }
    CHECK(!raised && !(h.flags & HEALTH_F_CAL), "an interrupted drift starts again");
    CHECK(hc_run(&h, &is, &st) != 0u, "-35 %% raises it too");

    /* not: +20 %; +40 % but only 40 mA (a 100 mA channel); no stored test; no usable model */
    health_test_start(&h, &st);
    (void)health_test_done(&h, &st);
    is.ch_ma[4] = 6000;
    for (s = 0; s < 2u * HEALTH_SUGGEST_S; s++) {
        raised |= hc_run(&h, &is, &st);
    }
    CHECK(!raised && !(h.flags & HEALTH_F_CAL), "+20 %%: nothing");
    he_test(&st, small, 30);
    isense_cal_set(&is, &st);
    is.ch_ma[0] = 140;
    is.applied[0] = 30000;
    for (s = 0; s < 2u * HEALTH_SUGGEST_S; s++) {
        raised |= hc_run(&h, &is, &st);
    }
    CHECK(!raised && (h.flags & ULTRA_HE_F_DRIFT) && !(h.flags & HEALTH_F_CAL), "100 -> 140 mA: drift, no suggestion");
    is.ch_ma[0] = 6750;
    for (s = 0; s < 2u * HEALTH_SUGGEST_S; s++) {
        raised |= hc_run(&h, &is, &none);
    }
    CHECK(!raised && !(h.flags & HEALTH_F_CAL), "no stored test: nothing");
    he_test(&st, base, 30);
    isense_init(&is, 1, 10000, 8000);                     /* learned only, not trusted yet: nothing */
    is.ch_ma[4] = 6750;
    is.applied[4] = 30000;
    for (s = 0; s < 2u * HEALTH_SUGGEST_S; s++) {
        raised |= hc_run(&h, &is, &st);
    }
    CHECK(!raised && !(h.flags & HEALTH_F_CAL), "model not usable: nothing");

    /* a strip added where the test found nothing (W1): 250 mA learned raises it, 150 mA doesn't */
    isense_cal_set(&is, &st);
    is.ch_ma[3] = 150;
    is.applied[3] = 20000;
    for (s = 0; s < 2u * HEALTH_SUGGEST_S; s++) {
        raised |= hc_run(&h, &is, &st);
    }
    CHECK(!raised && !(h.flags & HEALTH_F_CAL) && h.model_dev[3] == ULTRA_HE_NA, "W1 empty at the test, 150 mA: nothing");
    is.ch_ma[3] = 250;
    for (s = 0; s < HEALTH_SUGGEST_S; s++) {
        raised |= hc_run(&h, &is, &st);
    }
    CHECK(raised && (h.flags & HEALTH_F_CAL), "W1 empty at the test, 250 mA: a new load");
}

static void test_health(void)
{
    static const uint16_t base[5] = { 3000, 3000, 2500, 0, 5000 };
    isense_cal_t prev, now;
    health_t h;
    isense_t is;
    uint8_t code, pg[24];
    uint16_t v[5];
    uint32_t ch;

    printf("channel health: classes, dark current, model drift, D9 code, page\n");
    /* a first test (none before): R, G, B, W2 normal, W1 nothing connected */
    health_init(&h);
    memset(&prev, 0, sizeof prev);
    health_test_start(&h, &prev);
    he_test(&now, base, 35);
    code = health_test_done(&h, &now);
    CHECK(code == 0 && h.flags == ULTRA_HE_F_TESTED, "first test: code %u, flags %02X", code, h.flags);
    CHECK(h.cls[0] == ULTRA_HE_NORMAL && h.cls[3] == ULTRA_HE_NO_LOAD && h.cls[4] == ULTRA_HE_NORMAL &&
          h.test_dev[0] == ULTRA_HE_NA, "first test: normal / no load, no comparison");

    /* the same strips again, 5 % more on R: normal, +5 % */
    prev = now;
    health_test_start(&h, &prev);
    memcpy(v, base, sizeof v);
    v[0] = 3150;
    he_test(&now, v, 36);
    code = health_test_done(&h, &now);
    CHECK(code == 0 && h.cls[0] == ULTRA_HE_NORMAL && h.test_dev[0] == 5 && h.test_dev[1] == 0 &&
          h.test_dev[3] == ULTRA_HE_NA, "again: R +%d %%, normal", h.test_dev[0]);

    /* B 45 % above, W2 lost its load, G not rising (flat: a FET stuck partly on), W1 newly connected */
    prev = now;
    health_test_start(&h, &prev);
    memcpy(v, base, sizeof v);
    v[2] = 3625;
    v[4] = 0;
    v[3] = 1200;
    he_test(&now, v, 36);
    now.ma[1][0] = 2900; now.ma[1][1] = 2950; now.ma[1][2] = 2980; now.ma[1][3] = 3000;
    code = health_test_done(&h, &now);
    CHECK(code == 2 && (h.flags & ULTRA_HE_F_ABNORMAL), "findings: code %u (G first)", code);
    CHECK(h.cls[1] == (ULTRA_HE_ABNORMAL | ULTRA_HE_R_NOT_RISING), "G: not rising (%02X)", h.cls[1]);
    CHECK(h.cls[2] == (ULTRA_HE_ABNORMAL | ULTRA_HE_R_ABOVE) && h.test_dev[2] == 45, "B: above, +%d %%",
          h.test_dev[2]);
    CHECK(h.cls[3] == ULTRA_HE_NORMAL, "W1: newly connected is normal (%02X)", h.cls[3]);
    CHECK(h.cls[4] == (ULTRA_HE_ABNORMAL | ULTRA_HE_R_BELOW), "W2: lost its load (%02X)", h.cls[4]);
    /* small changes are no findings: 40 mA more on a 100 mA channel (+40 %, but under 50 mA) */
    {
        static const uint16_t small[5] = { 100, 0, 0, 0, 0 };
        static const uint16_t small2[5] = { 140, 0, 0, 0, 0 };
        he_test(&prev, small, 30);
        health_test_start(&h, &prev);
        he_test(&now, small2, 30);
        CHECK(health_test_done(&h, &now) == 0 && h.cls[0] == ULTRA_HE_NORMAL && h.test_dev[0] == 40,
              "100 -> 140 mA: normal (+%d %%)", h.test_dev[0]);
    }
    /* B 35 % below */
    he_test(&prev, base, 30);
    health_test_start(&h, &prev);
    memcpy(v, base, sizeof v);
    v[2] = 1625;
    he_test(&now, v, 30);
    CHECK(health_test_done(&h, &now) == 3 && h.cls[2] == (ULTRA_HE_ABNORMAL | ULTRA_HE_R_BELOW) &&
          h.test_dev[2] == -35, "B: below, %d %%", h.test_dev[2]);

    /* the dark current: 60 mA over the zero of the test before (a stuck FET): code 6 when nothing else */
    he_test(&prev, base, 30);
    health_test_start(&h, &prev);
    he_test(&now, base, 90);
    code = health_test_done(&h, &now);
    CHECK(code == 6 && (h.flags & ULTRA_HE_F_DARK) && h.dark_ma == 60, "dark 60 mA over: code %u, %d mA", code,
          h.dark_ma);
    he_test(&now, base, 70);
    health_test_start(&h, &prev);
    CHECK(health_test_done(&h, &now) == 0 && !(h.flags & ULTRA_HE_F_DARK), "dark 40 mA over: fine");
    /* without a test before: absolute, 100 mA */
    memset(&prev, 0, sizeof prev);
    health_test_start(&h, &prev);
    he_test(&now, base, 95);
    CHECK(health_test_done(&h, &now) == 0, "first test, zero 95 mA: fine");
    health_test_start(&h, &prev);
    he_test(&now, base, 130);
    CHECK(health_test_done(&h, &now) == 6, "first test, zero 130 mA: dark");

    /* a single-level test at 40 %: scaled to 100 %, compared, no rising check */
    he_test(&prev, base, 30);
    health_test_start(&h, &prev);
    memset(&now, 0, sizeof now);
    now.levels = 1;
    now.pct[0] = 40;
    now.zero_ma = 30;
    for (ch = 0; ch < 5u; ch++) {
        now.ma[ch][0] = (uint16_t)(base[ch] * 40u / 100u);
    }
    now.ma[0][0] = 1800;                                  /* R: 4500 mA at 100 %, +50 % */
    code = health_test_done(&h, &now);
    CHECK(code == 1 && h.cls[0] == (ULTRA_HE_ABNORMAL | ULTRA_HE_R_ABOVE) && h.test_dev[0] == 50 &&
          h.cls[1] == ULTRA_HE_NORMAL && h.cls[3] == ULTRA_HE_NO_LOAD, "single level: R +%d %%", h.test_dev[0]);

    /* running: the model against the stored test (W2 +35 %), the dark current from the zero tracking */
    health_init(&h);
    he_test(&now, base, 30);
    isense_init(&is, 1, 10000, 8000);
    isense_cal_set(&is, &now);
    health_run(&h, &is, &now);
    CHECK(h.flags == 0 && h.model_dev[0] == 0 && h.model_dev[3] == ULTRA_HE_NA && h.dark_ma == 0,
          "running, as tested: nothing");
    is.ch_ma[4] = 6750;
    is.zero_ma = 110;
    health_run(&h, &is, &now);
    CHECK((h.flags & ULTRA_HE_F_DRIFT) && h.model_dev[4] == 35 && (h.flags & ULTRA_HE_F_DARK) && h.dark_ma == 80,
          "running: W2 +%d %%, dark %d mA", h.model_dev[4], h.dark_ma);
    is.ch_ma[4] = 5500;
    is.zero_ma = 40;
    health_run(&h, &is, &now);
    CHECK(h.flags == 0 && h.model_dev[4] == 10, "running: +10 %% is no drift");
    memset(&prev, 0, sizeof prev);                        /* no stored test: nothing to compare */
    health_run(&h, &is, &prev);
    CHECK(h.model_dev[0] == ULTRA_HE_NA && !(h.flags & ULTRA_HE_F_DRIFT), "running without a test: n/a");

    /* the HEALTH page (protocol 0.2.3) */
    he_test(&prev, base, 30);
    health_test_start(&h, &prev);
    memcpy(v, base, sizeof v);
    v[2] = 3625;
    he_test(&now, v, 30);
    (void)health_test_done(&h, &now);
    health_run(&h, &is, &now);
    health_page(&h, 412, 75, pg);
    CHECK(pg[ULTRA_HE_FLAGS] == h.flags && pg[ULTRA_HE_CLASS + 2] == (ULTRA_HE_ABNORMAL | ULTRA_HE_R_ABOVE) &&
          (int8_t)pg[ULTRA_HE_TEST_DEV + 2] == 45 && (int8_t)pg[ULTRA_HE_TEST_DEV + 3] == ULTRA_HE_NA &&
          (int8_t)pg[ULTRA_HE_MODEL_DEV + 4] == h.model_dev[4] && (int16_t)ultra_get16(pg + ULTRA_HE_TEMP_PEAK) == 412 &&
          ultra_get24(pg + ULTRA_HE_TEMP_AGE) == 75, "HEALTH page fields");
    health_page(&h, (int16_t)ULTRA_ST_NA, 1234u, pg);  /* (0.2.4) no temperature peak yet: none, not the uptime */
    CHECK(ultra_get16(pg + ULTRA_HE_TEMP_PEAK) == ULTRA_ST_NA && ultra_get24(pg + ULTRA_HE_TEMP_AGE) == 0xFFFFFFu,
          "HEALTH page: no peak yet = 0x8000 / 0xFFFFFF (age %06X)", ultra_get24(pg + ULTRA_HE_TEMP_AGE));

    /* the D9 code: one long flash, then the code as a value code, D9 only */
    {
        ack_t a;
        uint8_t on;
        uint32_t t, flashes = 0, prev_on = 0;
        ack_start(&a, ACK_P_FAULT, 3);
        CHECK(a.led_only, "the fault code is D9 only");
        for (t = 0; t < 10000u && ack_tick(&a, &on); t++) {
            flashes += on && !prev_on;
            prev_on = on;
        }
        CHECK(flashes == 4 && t > 3000u && t < 6000u, "code 3: 1 long + 3 short (%u flashes, %u ms)", flashes, t);
        ack_start(&a, ACK_P_ACK, 0);
        CHECK(!a.led_only, "the acknowledge stays on the feedback channel too");
    }
    test_health_cal();
}
