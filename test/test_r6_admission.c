/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Included at the end of test_soak.c: the flash admission, the console and the boot on
 * main.c, each in its own process (soak_tests.exe <name>; test_soak.c's model: the records at their real addresses,
 * sim_vdd_low = VDD below the PVD, fl_fail = operations on the settings pages fail).
 *
 *   r6vdd     a settings change pending while VDD is below the PVD: nothing is attempted (flash_op refuses at once:
 *             FLASH_ERR_VDD; a write tried every ms would take a token of the settings' write budget (16, then one per
 *             3 h) each and move the A/B pair's position, so when VDD came back the change would wait up to 3 h)
 *   r6erasef  an erase ahead that keeps failing backs off (a retry every ms would be a flash operation with its clock
 *             switch and a console message each; on the board throttled only by the message's UART time)
 *   r6boot    the boot's erases ahead: one call per store, all five stores of the upper layout (M2: the calibration
 *             history's), so none waits for the first low moment after LIFE_PREP_S
 *   r6fxreset (evidence, no fix: a policy item) reset input's factory reset while an Ultra host keeps the light
 *             bright: the power-on effects' clear (two erases) waits for a low moment the jig's pattern and the lit
 *             light never give; a power cycle before it brings the effects back */

/* ------------------------------------------------------------------------------------------------ r6vdd */
static void sc_r6vdd(void)
{
    uint32_t w0, tok0, ea, eb;
    printf("soak r6vdd: a settings change pending while VDD is below the PVD\n");
    boot(0, 0, 0);
    run_ms(3000);
    w0 = W_SETTINGS();
    tok0 = set_budget.tokens;
    ea = fl_erases[page_of(SETTINGS_ADDR)];
    eb = fl_erases[page_of(SETTINGS_B_ADDR)];
    console("set dither 1");                             /* pending: the 2 s debounce */
    sim_vdd_low = 1;
    run_ms(5000);                                        /* VDD low through the debounce and 3 s after it */
    printf("  while VDD was low: %u settings tokens left of %u, %u writes, erases %u / %u\n", (uint32_t)set_budget.tokens,
           tok0, W_SETTINGS() - w0, fl_erases[page_of(SETTINGS_ADDR)] - ea, fl_erases[page_of(SETTINGS_B_ADDR)] - eb);
    CHECK(set_budget.tokens == tok0 && W_SETTINGS() == w0, "r6vdd: nothing attempted while VDD is below the PVD (%u of "
          "%u tokens left)", (uint32_t)set_budget.tokens, tok0);
#if MON_EXTRA
    CHECK(stat.fl_errs == 0u, "r6vdd: no failed operation counted (%u)", (uint32_t)stat.fl_errs);
#endif
    sim_vdd_low = 0;
    run_ms(300);
    printf("  300 ms after VDD came back: %u writes, dirty %u\n", W_SETTINGS() - w0, settings_dirty);
    CHECK(W_SETTINGS() == w0 + 1u && !settings_dirty && set.dither_mode == 1u, "r6vdd: written right after VDD came "
          "back (%u)", W_SETTINGS() - w0);
}

/* ------------------------------------------------------------------------------------------------ r6erasef */
static void sc_r6erasef(void)
{
    uint32_t n0, n;
    printf("soak r6erasef: an erase ahead that keeps failing (write protection)\n");
    boot(0, 0, 0);
    host_on = 1;                                         /* an Ultra hub: the lock stores the sticky protocol (dark) */
    cmd_def(0, 0, 3, 2800);
    run_ms(4000);
    CHECK(W_SETTINGS() == 1u && settings_pending() == 1u, "r6erasef: the settings on page A, page B's erase ahead due "
          "(%u writes, pending %u)", W_SETTINGS(), settings_pending());
    fl_fail = 1000000u;                                  /* every operation on the settings pages fails from now on */
    run_ms(7500);                                        /* past LIFE_PREP_S: the erases ahead begin */
    n0 = sim_stalls;
    run_ms(30000);                                       /* (the hold-off, 10 s doubling) */
    n = sim_stalls - n0;
    printf("  failed erase attempts in 30 s: %u\n", n);
    CHECK(n >= 1u && n <= 3u, "r6erasef: a failing erase backs off (%u attempts in 30 s)", n);
    fl_fail = 0;
    run_ms(45000);                                       /* (the next try: at most 40 s after the last) */
    CHECK(settings_pending() == 0u, "r6erasef: ... and is done once the flash works again (pending %u)",
          settings_pending());
}

/* ------------------------------------------------------------------------------------------------ r6boot */
/* every store's newest record on its page B, page A full of older ones: at boot each pair's erase ahead (of A) is due */
static void r6_fill_stores(void)
{
    settings_t s, sg;
    store_ext_t ex, eg;
    isense_cal_t c, cg;
    uint16_t cnt;
    scene_t sc, scg;
    energy_t e;
    life_rec_t lg;
    uint32_t k;
    (void)settings_load(&sg, &eg);
    for (k = 0; k <= STORE_SLOTS; k++) {
        r5_settings_val(k, &s, &ex);
        if (settings_pending() == 2u) (void)settings_prepare();
        (void)settings_store(&s, &ex);
    }
    (void)cal_load(&cg, &cnt);
    for (k = 0; k <= CAL_RECS; k++) {
        r5_cal_val(k, &c);
        if (cal_pending() == 2u) (void)cal_prepare();
        (void)cal_store(&c, &cnt);
    }
    (void)scene_load(&scg);
    for (k = 0; k <= SCENE_RECS_PAGE; k++) {
        scene_clear(&sc);
        sc.mask = 0x1Fu;
        sc.v[0] = (uint16_t)(1000u + k);
        if (scene_pending() == 2u) (void)scene_prepare();
        (void)scene_store(&sc);
    }
    memset(&e, 0, sizeof e);
    e.lt.magic = LIFE_MAGIC;
    (void)life_load(&lg);
    for (k = 0; k <= LIFE_RECS; k++) {
        e.lt.on_s = 7000u + k;
        if (life_pending() == 2u) (void)life_prepare();
        (void)life_store(&e);
    }
#if MON_EXTRA
    {
        calhist_t h;
        memset(&h, 0, sizeof h);
        (void)calhist_load(&h);
        for (k = 0; k <= CALHIST_RECS; k++) {
            h.tests = (uint16_t)k;
            if (calhist_pending() == 2u) (void)calhist_prepare();
            (void)calhist_store(&h);
        }
    }
#endif
}

static void sc_r6boot(void)
{
    uint32_t due;
    printf("soak r6boot: every store with an erase ahead due at power-up\n");
    r6_fill_stores();
    due = (settings_pending() == 1u) + (cal_pending() == 1u) + (scene_pending() == 1u) + (life_pending() == 1u);
#if MON_EXTRA
    due += calhist_pending() == 1u;
#endif
    CHECK(due == (uint32_t)REC_COUNT, "r6boot: the set-up: %u of %u stores with an erase ahead due", due,
          (uint32_t)REC_COUNT);
    boot(0, 0, 0);                                       /* the boot's erases ahead, before the outputs start */
    due = (settings_pending() != 0u) + (cal_pending() != 0u) + (scene_pending() != 0u) + (life_pending() != 0u);
#if MON_EXTRA
    due += calhist_pending() != 0u;
#endif
    printf("  after the boot: %u erases ahead still due\n", due);
    CHECK(due == 0u, "r6boot: every store's erase ahead done at power-up (%u left)", due);
}

static int r6f_scenario(const char *s)
{
    if (!strcmp(s, "r6vdd")) {
        sc_r6vdd();
    } else if (!strcmp(s, "r6erasef")) {
        sc_r6erasef();
    } else if (!strcmp(s, "r6boot")) {
        sc_r6boot();
    } else {
        return 0;
    }
    return 1;
}
