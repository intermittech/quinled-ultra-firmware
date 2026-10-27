/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Included by test_soak.c. A rejected/torn program returns to the main loop
 * without a reset; the supply then recovers and erase-ahead is admitted. */
static void sc_r5retry(void)
{
    uint32_t which, mode, round, k, n, before = 0, after, failed, prepared;
    settings_t s, sg;
    store_ext_t ex, eg;
    isense_cal_t c, cg;
    uint16_t cnt;
    scene_t sc, scg;
    energy_t e;
    life_rec_t lg;
    printf("r5retry: a failed first program on the partner page, erase-ahead, then reboot\n");
    for (which = 0; which < 4u; which++) {
        for (mode = 0; mode < 3u; mode++) {
            for (round = 1; round <= 2u; round++) {
                r5_page_erase(SETTINGS_ADDR); r5_page_erase(SETTINGS_B_ADDR);
                r5_page_erase(CAL_PAGE_ADDR); r5_page_erase(CAL_B_ADDR);
                r5_page_erase(SCENE_LOG_ADDR); r5_page_erase(SCENE_B_ADDR);
                r5_page_erase(LIFE_PAGE0_ADDR); r5_page_erase(LIFE_PAGE1_ADDR);
                (void)settings_load(&sg, &eg); (void)cal_load(&cg, &cnt);
                (void)scene_load(&scg); (void)life_load(&lg);
                r5_settings_val(77, &s, &ex); r5_cal_val(77, &c);
                scene_clear(&sc); sc.mask = 1; sc.v[0] = 12345;
                memset(&e, 0, sizeof e); e.lt.magic = LIFE_MAGIC; e.lt.on_s = 12345;
                n = which == 0u ? STORE_SLOTS : which == 1u ? CAL_RECS :
                    which == 2u ? SCENE_RECS_PAGE : LIFE_RECS;
                for (k = 0; k < round * n; k++) {
                    if (which == 0u) { (void)settings_prepare(); (void)settings_store(&s, &ex); }
                    if (which == 1u) { (void)cal_prepare(); (void)cal_store(&c, &cnt); }
                    if (which == 2u) { (void)scene_prepare(); (void)scene_store(&sc); }
                    if (which == 3u) { (void)life_prepare(); (void)life_store(&e); }
                }
                if (which == 0u) { (void)settings_prepare(); before = settings_load(&sg, &eg) == 0; }
                if (which == 1u) { (void)cal_prepare(); before = cal_load(&cg, &cnt) == 0; }
                if (which == 2u) { (void)scene_prepare(); before = scene_load(&scg) == 0; }
                if (which == 3u) { (void)life_prepare(); before = life_load(&lg) != 0u; }
                /* The boot-like load above only establishes the starting position.
                 * No loader is called between the error and erase-ahead. */
                sim_vdd_low = mode == 0u;
                fl_cut = mode == 0u ? -1 : mode == 1u ? 0 : 3;
                failed = which == 0u ? settings_store(&s, &ex) : which == 1u ? cal_store(&c, &cnt) :
                         which == 2u ? scene_store(&sc) : life_store(&e);
                sim_vdd_low = 0; fl_cut = -1;             /* recovery without reset */
                prepared = which == 0u ? settings_prepare() : which == 1u ? cal_prepare() :
                           which == 2u ? scene_prepare() : life_prepare();
                after = which == 0u ? settings_load(&sg, &eg) == 0 : which == 1u ? cal_load(&cg, &cnt) == 0 :
                        which == 2u ? scene_load(&scg) == 0 : life_load(&lg) != 0u;
                printf("  store %u failure %u direction %u: before %u, error %u, prepare %u, after %u\n",
                       which, mode, round, before, failed, prepared, after);
                CHECK(before && failed != FLASH_OK && after,
                      "r5retry: store %u failure %u direction %u lost the last good record", which, mode, round);
            }
        }
    }
    /* (the fix: pagepair.c pp_done) every write on the partner page fails, each followed by the erase that is
     * possible then. Without a reboot, the next good write goes to the partner page again (erased first) and is the
     * newest; after a reboot instead, the page with the old record was never erased */
    for (mode = 1; mode < 3u; mode++) {
        for (round = 0; round < 2u; round++) {
            r5_page_erase(SETTINGS_ADDR);
            r5_page_erase(SETTINGS_B_ADDR);
            (void)settings_load(&sg, &eg);
            for (k = 0; k < STORE_SLOTS; k++) {
                r5_settings_val(k, &s, &ex);
                (void)settings_prepare();
                (void)settings_store(&s, &ex);
            }
            (void)settings_prepare();                    /* (the partner page erased ahead, before the failures) */
            for (k = 0, n = 0; k < STORE_SLOTS; k++) {
                r5_settings_val(100u + k, &s, &ex);
                fl_cut = mode == 1u ? 0 : 3;
                n += settings_store(&s, &ex) != FLASH_OK;
                fl_cut = -1;
                (void)settings_prepare();
            }
            failed = FLASH_OK;
            if (round == 1u) {
                r5_settings_val(200u, &s, &ex);
                (void)settings_prepare();
                failed = settings_store(&s, &ex);
            } else {
                r5_settings_val(STORE_SLOTS - 1u, &s, &ex);
            }
            after = settings_load(&sg, &eg) == 0 && memcmp(&sg, &s, sizeof s) == 0;
            printf("  a page of failed writes (failure %u), %s: %u failed, then %u, the record %s %u\n", mode,
                   round ? "a good write" : "a reboot", n, failed, round ? "written" : "kept", after);
            CHECK(n == STORE_SLOTS && failed == FLASH_OK && after, "r5retry: a page of failed writes (failure %u, "
                  "round %u): %u failed, then %u, the record %u", mode, round, n, failed, after);
        }
    }
}
