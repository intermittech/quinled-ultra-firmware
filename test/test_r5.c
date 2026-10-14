/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* 0.6.5 tests (the upper flash, dip writes): included at the end of test_soak.c, on its hardware model and its flash
 * (the records at their real addresses, the soak's flash_op with a power cut after any erase or half-word: fl_cut).
 * Each scenario runs in a fresh process: soak_tests.exe <name>. */

/* ------------------------------------------------------------------------------------------- the dip writes */
/* Quindor 2026-10-14: a dip write is a program into an already erased entry, never an erase, the check last, so a
 * write cut short is not read and the previous record stays. Proven here for every store that writes at a dip: the
 * cut after every half-word of the write (k = 0 .. all), at every position of the A/B pair (pre = the records before
 * it, so the write lands in either page, on a page's last entry and on the other page's first, erased ahead or not),
 * then a reboot (the load finds the newest record and the position again): always the old or the new record. */
static void r5_page_erase(uint32_t a)
{
    memset(fl + (a - FL_BASE), 0xFF, 256u);
}

static void r5_snap(uint8_t *buf, uint32_t a, uint32_t b, int save)
{
    if (save) {
        memcpy(buf, fl + (a - FL_BASE), 256u);
        memcpy(buf + 256, fl + (b - FL_BASE), 256u);
    } else {
        memcpy(fl + (a - FL_BASE), buf, 256u);
        memcpy(fl + (b - FL_BASE), buf + 256, 256u);
    }
}

static void r5_settings_val(uint32_t k, settings_t *s, store_ext_t *e)
{
    memset(s, 0, sizeof *s);                             /* (the padding too: the records are compared whole) */
    memset(e, 0, sizeof *e);
    settings_default(s);
    store_ext_clear(e);
    s->raw_steps = 1440u + k;
    s->min_pulse = (uint8_t)k;
    e->ilimit50 = (uint8_t)(k * 7u);
}

static void r5_cut_settings(uint32_t *runs, uint32_t *bad)
{
    static uint8_t img[512];
    settings_t a, b, got;
    store_ext_t ea, eb, eg;
    uint32_t pre, k, n = STORE_SLOT_BYTES / 2u, ahead;
    for (pre = 0; pre < 2u * STORE_SLOTS + 2u; pre++) {
        for (ahead = 0; ahead < 2u; ahead++) {
            r5_page_erase(SETTINGS_ADDR);
            r5_page_erase(SETTINGS_B_ADDR);
            (void)settings_load(&got, &eg);
            for (k = 0; k <= pre; k++) {
                r5_settings_val(k, &a, &ea);
                (void)settings_store(&a, &ea);           /* (the due erase in it, as in a low moment) */
            }
            if (settings_pending() == 2u || (ahead && settings_pending() == 1u)) {
                (void)settings_prepare();                /* the erase a low moment did earlier: an erased entry */
            }
            r5_snap(img, SETTINGS_ADDR, SETTINGS_B_ADDR, 1);
            r5_settings_val(pre + 1u, &b, &eb);
            for (k = 0; k <= n; k++) {
                r5_snap(img, SETTINGS_ADDR, SETTINGS_B_ADDR, 0);
                (void)settings_load(&got, &eg);          /* (the boot before the dip) */
                fl_cut = (int32_t)k;
                (void)settings_store(&b, &eb);           /* the dip write: a program only */
                fl_cut = -1;
                memset(&got, 0x5A, sizeof got);
                (*runs)++;
                if (settings_load(&got, &eg) != 0 ||
                    (k < n ? memcmp(&got, &a, sizeof got) || memcmp(&eg, &ea, sizeof eg)
                           : memcmp(&got, &b, sizeof got) || memcmp(&eg, &eb, sizeof eg))) {
                    (*bad)++;
                }
            }
        }
    }
}

static void r5_cal_val(uint32_t k, isense_cal_t *c)
{
    uint32_t i, j;
    memset(c, 0, sizeof *c);
    c->levels = ISENSE_LEVELS;
    for (j = 0; j < ISENSE_LEVELS; j++) {
        c->pct[j] = (uint8_t)(25u * (j + 1u));
        for (i = 0; i < ISENSE_CH; i++) {
            c->ma[i][j] = (uint16_t)(100u * (j + 1u) + k * 3u + i);
        }
    }
    c->zero_ma = (int16_t)(40 + (int32_t)k);
}

static void r5_cut_cal(uint32_t *runs, uint32_t *bad)
{
    static uint8_t img[512];
    isense_cal_t a, b, got;
    uint16_t cnt;
    uint32_t pre, k, n = CAL_REC_BYTES / 2u, ahead;
    for (pre = 0; pre < 2u * CAL_RECS + 2u; pre++) {
        for (ahead = 0; ahead < 2u; ahead++) {
            r5_page_erase(CAL_PAGE_ADDR);
            r5_page_erase(CAL_B_ADDR);
            (void)cal_load(&got, &cnt);
            for (k = 0; k <= pre; k++) {
                r5_cal_val(k, &a);
                (void)cal_store(&a, &cnt);
            }
            if (cal_pending() == 2u || (ahead && cal_pending() == 1u)) {
                (void)cal_prepare();
            }
            r5_snap(img, CAL_PAGE_ADDR, CAL_B_ADDR, 1);
            r5_cal_val(pre + 1u, &b);
            for (k = 0; k <= n; k++) {
                r5_snap(img, CAL_PAGE_ADDR, CAL_B_ADDR, 0);
                (void)cal_load(&got, &cnt);
                fl_cut = (int32_t)k;
                (void)cal_store(&b, &cnt);
                fl_cut = -1;
                memset(&got, 0x5A, sizeof got);
                (*runs)++;
                if (cal_load(&got, &cnt) != 0 || memcmp(&got, k < n ? &a : &b, sizeof got) != 0) {
                    (*bad)++;
                }
            }
        }
    }
}

static void r5_cut_scene(uint32_t *runs, uint32_t *bad)
{
    static uint8_t img[512];
    scene_t a, b, got;
    uint32_t pre, k, n = SCENE_REC_BYTES / 2u, ahead;
    scene_clear(&a);
    for (pre = 0; pre < 2u * SCENE_RECS_PAGE + 2u; pre++) {
        for (ahead = 0; ahead < 2u; ahead++) {
            r5_page_erase(SCENE_LOG_ADDR);
            r5_page_erase(SCENE_B_ADDR);
            (void)scene_load(&got);
            for (k = 0; k <= pre; k++) {
                scene_clear(&a);
                a.mask = 0x1Fu;
                a.v[0] = (uint16_t)(1000u + k);
                a.fade = (uint8_t)k;
                (void)scene_store(&a);
            }
            if (scene_pending() == 2u || (ahead && scene_pending() == 1u)) {
                (void)scene_prepare();
            }
            r5_snap(img, SCENE_LOG_ADDR, SCENE_B_ADDR, 1);
            b = a;
            b.v[0] = (uint16_t)(1000u + pre + 1u);
            b.fade = (uint8_t)(pre + 1u);
            for (k = 0; k <= n; k++) {
                r5_snap(img, SCENE_LOG_ADDR, SCENE_B_ADDR, 0);
                (void)scene_load(&got);
                fl_cut = (int32_t)k;
                (void)scene_store(&b);
                fl_cut = -1;
                memset(&got, 0x5A, sizeof got);
                (*runs)++;
                if (scene_load(&got) != 0 || memcmp(&got, k < n ? &a : &b, sizeof got) != 0) {
                    (*bad)++;
                }
            }
        }
    }
}

static void r5_cut_life(uint32_t *runs, uint32_t *bad)
{
    static uint8_t img[512];
    static energy_t e;
    life_rec_t got;
    uint32_t pre, k, n = LIFE_REC_BYTES / 2u, ahead, on_a = 0, on_b;
    for (pre = 0; pre < 2u * LIFE_RECS + 2u; pre++) {
        for (ahead = 0; ahead < 2u; ahead++) {
            r5_page_erase(LIFE_PAGE0_ADDR);
            r5_page_erase(LIFE_PAGE1_ADDR);
            memset(&e, 0, sizeof e);
            e.lt.magic = LIFE_MAGIC;                     /* (energy_init sets it on the board) */
            (void)life_load(&got);
            for (k = 0; k <= pre; k++) {
                if (life_pending() == 2u) {
                    (void)life_prepare();                /* (the lifetime record's erases: only ever separate) */
                }
                e.lt.on_s = on_a = 7000u + k;
                (void)life_store(&e);
            }
            if (life_pending() == 2u || (ahead && life_pending() == 1u)) {
                (void)life_prepare();
            }
            r5_snap(img, LIFE_PAGE0_ADDR, LIFE_PAGE1_ADDR, 1);
            on_b = 7000u + pre + 1u;
            for (k = 0; k <= n; k++) {
                life_rec_t keep = e.lt;
                r5_snap(img, LIFE_PAGE0_ADDR, LIFE_PAGE1_ADDR, 0);
                (void)life_load(&got);
                e.lt.on_s = on_b;
                fl_cut = (int32_t)k;
                (void)life_store(&e);
                fl_cut = -1;
                e.lt = keep;
                memset(&got, 0x5A, sizeof got);
                (*runs)++;
                if (!life_load(&got) || got.on_s != (k < n ? on_a : on_b)) {
                    (*bad)++;
                }
            }
        }
    }
}

static void r5_dip(void)
{
    uint32_t runs[4] = { 0, 0, 0, 0 }, bad[4] = { 0, 0, 0, 0 }, t;
    printf("soak r5dip: a power cut at every half-word of a dip write, every position of each A/B store, then a "
           "reboot\n");
    r5_cut_settings(&runs[0], &bad[0]);
    r5_cut_cal(&runs[1], &bad[1]);
    r5_cut_scene(&runs[2], &bad[2]);
    r5_cut_life(&runs[3], &bad[3]);
    printf("  cuts: settings %u, calibration %u, scene %u, lifetime %u: the old or the new record in %u / %u / %u / "
           "%u\n", runs[0], runs[1], runs[2], runs[3], runs[0] - bad[0], runs[1] - bad[1], runs[2] - bad[2],
           runs[3] - bad[3]);
    CHECK(bad[0] + bad[1] + bad[2] + bad[3] == 0u && runs[0] && runs[1] && runs[2] && runs[3],
          "dip: every cut leaves the old or the new record (%u %u %u %u bad)", bad[0], bad[1], bad[2], bad[3]);
    /* the rules on the running firmware: a dip admits a program, never an erase; below the PVD nothing starts */
    boot(0, 0, 0);
    run_ms(15000);                                       /* (past LIFE_PREP_S: the erases ahead are done) */
    CHECK(sim_window == 1u, "dip: the boot read the records in its one window (%u)", sim_window);
    vdip.in_dip = 1;
    CHECK(flash_may(1u, 0u) == 0u && flash_may(0u, 0u) == (FLASH_DIP_WRITES ? FM_PROG : 0u),
          "dip: no erase, a program (FLASH_DIP_WRITES %u): %u / %u", (unsigned)FLASH_DIP_WRITES, flash_may(1u, 0u),
          flash_may(0u, 0u));
    vdip.in_dip = 0;
    t = W_SETTINGS();
    sim_vdd_low = 1;
    con_out_n = 0;
    console("save");
    CHECK(W_SETTINGS() == t && settings_dirty && strstr(con_out, "pending") != 0,   /* (r6: flash_may says so) */
          "dip: VDD below the PVD: the console's save starts nothing, stays pending (%u)", W_SETTINGS() - t);
    sim_vdd_low = 0;
    console("save");
    CHECK(W_SETTINGS() >= t + 1u && !settings_dirty, "dip: ... written once VDD is back (%u)", W_SETTINGS() - t);
}

/* ------------------------------------------------------------------------------- the power-on effect banks */
/* 0.6.5 (upper layout): two banks; a store rewrites the other one, its header last with the next sequence number. A
 * power cut at every operation and half-word of a store, then a reboot: the effects stored before, or the new ones;
 * never none (0.6.4: the one bank's header erased first). A factory reset's clear (mask 0) likewise. */
static ultra_t r5_u, r5_v, r5_w;                        /* the first set, the second, the boots' loads */

static uint8_t r5_fx_setup(ultra_t *u, uint8_t which, uint8_t crc[ULTRA_LIGHTS])
{
    static const uint8_t roles0[5] = { ULTRA_ROLE_BRIGHTNESS, 0, 0, 0, 0 }, roles1[5] = { 0, ULTRA_ROLE_BRIGHTNESS, 0, 0, 0 };
    static const uint16_t a5[5] = { 65535, 65535, 65535, 65535, 65535 }, mn[5] = { 0, 0, 0, 0, 0 };
    uint8_t s[32], p[4] = { 0x9C, 0x40, 0, 100 }, seq = (uint8_t)(1u + which * 16u), mask = 0;
    uint32_t i;
    ultra_init(u, 1);
    p[3] = (uint8_t)(60u + which * 30u);
    ultra_slot_light_def(s, 0, seq++, roles0, 1000, 0, mn, a5); (void)ultra_on_slot(u, s, 0);
    ultra_slot_light_def(s, 1, seq++, roles1, 1000, 0, mn, a5); (void)ultra_on_slot(u, s, 0);
    ultra_slot_scene(s, 0, seq++, 1, 10, 65535, a5, a5); (void)ultra_on_slot(u, s, 0);
    ultra_slot_scene(s, 1, seq++, 1, 10, 65535, a5, a5); (void)ultra_on_slot(u, s, 0);
    ultra_slot_scene_effect(s, 0, seq++, ULTRA_FX_CANDLE, p, 4); (void)ultra_on_slot(u, s, 0);
    ultra_slot_scene_effect(s, 1, seq++, ULTRA_FX_BREATHE, p, 4); (void)ultra_on_slot(u, s, 0);
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        crc[i] = 0;
        if (u->light[i].scene_fx_have && ultra_scene_fx_ok(&u->light[i])) {
            crc[i] = crc8_smbus((const uint8_t *)&u->light[i], ULTRA_HEAD_BYTES);
            mask |= (uint8_t)(1u << i);
        }
    }
    return mask;
}

static void r5_fx(void)
{
    static uint8_t img_a[512], img_b[512];
    uint8_t c1[ULTRA_LIGHTS], c2[ULTRA_LIGHTS], got[ULTRA_LIGHTS], m1, m2, mg;
    uint32_t k, total, bad = 0, olds = 0, news = 0, runs = 0, pass, none = 0;
    printf("soak r5fx: the power-on effect store's two banks: a power cut at every step of a store, then a reboot\n");
    m1 = r5_fx_setup(&r5_u, 0, c1);
    m2 = r5_fx_setup(&r5_v, 1, c2);
    CHECK(m1 == 3u && m2 == 3u && memcmp(c1, c2, 2) != 0, "fx: two different sets of effects (%X %X)", m1, m2);
    for (pass = 0; pass < 4u; pass++) {                  /* passes 1 / 3: the clear (a factory reset), mask 0;
                                                            2 / 3: the first bank's sequence number at 0x8001 */
        r5_page_erase(SCENE_FX_ADDR);
        r5_page_erase(SCENE_FX_ADDR + 256u);
        r5_page_erase(SCENE_FX_B_ADDR);
        r5_page_erase(SCENE_FX_B_ADDR + 256u);
        (void)scene_fx_load(&r5_w, got);
        CHECK(scene_fx_store(&r5_u, m1, c1) == FLASH_OK, "fx: the first store");
        if (pass & 1u) {                                 /* the second store first: the banks swap once more */
            CHECK(scene_fx_store(&r5_u, m1, c1) == FLASH_OK, "fx: a second store of the same");
        }
        if (pass >= 2u) {                                /* the newest bank's sequence near the middle of its range */
            uint32_t at = (pass & 1u) ? SCENE_FX_ADDR : SCENE_FX_B_ADDR;
            fl[at - FL_BASE + 10u] = 0x01u;
            fl[at - FL_BASE + 11u] = 0x80u;
        }
        memcpy(img_a, fl + (SCENE_FX_ADDR - FL_BASE), 512u);
        memcpy(img_b, fl + (SCENE_FX_B_ADDR - FL_BASE), 512u);
        /* how many units the store takes: 2 erases + the heads + the header (pass 1: 1 erase + the header) */
        total = (pass & 1u) == 0u ? 2u + 2u * (ULTRA_HEAD_BYTES / 2u) + ULTRA_FX_HDR / 2u : 1u + ULTRA_FX_HDR / 2u;
        for (k = 0; k <= total; k++) {
            memcpy(fl + (SCENE_FX_ADDR - FL_BASE), img_a, 512u);
            memcpy(fl + (SCENE_FX_B_ADDR - FL_BASE), img_b, 512u);
            (void)scene_fx_load(&r5_w, got);             /* (the boot before) */
            fl_cut = (int32_t)k;
            (void)scene_fx_store(&r5_v, (pass & 1u) == 0u ? m2 : 0u, c2);   /* the second set's heads */
            fl_cut = -1;
            mg = scene_fx_load(&r5_w, got);
            runs++;
            if (mg == m1 && memcmp(got, c1, 2) == 0) {
                olds++;
            } else if ((pass & 1u) == 0u && mg == m2 && memcmp(got, c2, 2) == 0 && k == total) {
                news++;
            } else if ((pass & 1u) && mg == 0u && k == total) {
                none++;
            } else {
                bad++;
            }
        }
    }
    printf("  %u cuts: the effects before in %u, the new ones (complete store) in %u, cleared (complete clear) in %u, "
           "anything else %u\n", runs, olds, news, none, bad);
    CHECK(bad == 0u && news == 2u && none == 2u, "fx: every cut keeps the effects before, only a complete store / "
          "clear changes them (%u bad)", bad);
}

/* ------------------------------------------------------------------------- the console's status in parts */
/* review round 4 #39 (gpt-opt2): a `status` (~1,400 bytes) written at once filled the UART's two 512-byte buffers and
 * held the main loop ~44 ms (effects, limiter, frames paused); seven queued ones ~0.8 s, past the watchdog. 0.6.5:
 * one part per main-loop pass, only when the UART takes it without a wait (the soak's UART drains at 115200 baud and
 * counts every byte written past its buffers: tx_blocks); the next line waits in the RX ring. Also: backspace. */
static void r5_console(void)
{
    const char *p;
    uint32_t n = 0, k, b0;
    printf("soak r5console: `status` in parts, seven queued, the UART never full; backspace\n");
    boot(0, 0, 0);
    run_ms(3000);
    b0 = tx_blocks;
    con_out_n = 0;
    con_out[0] = 0;
    con_in_r = con_in_w = 0;
    for (k = 0; k < 7u; k++) {                           /* seven at once in the RX ring (64 bytes) */
        memcpy(con_in + con_in_w, "status\r", 7u);
        con_in_w += 7u;
    }
    run_ms(3000);
    for (p = con_out; (p = strstr(p, "lifetime=")) != 0; p++) {
        n++;
    }
    printf("  7 x status: %u complete (%u bytes), writes past the UART's buffers: %u\n", n, con_out_n, tx_blocks - b0);
    CHECK(n == 7u && cli.st_part == 0u && tx_blocks == b0, "console: seven statuses, complete, the UART never full "
          "(%u, %u blocked bytes)", n, tx_blocks - b0);
    /* (0.6.6, review round 5: the regression) seven queued `get`s: each reply waits for the UART's room */
    b0 = tx_blocks;
    con_out_n = 0;
    con_out[0] = 0;
    con_in_r = con_in_w = 0;
    for (k = 0; k < 7u; k++) {
        memcpy(con_in + con_in_w, "get\r", 4u);
        con_in_w += 4u;
    }
    run_ms(1000);
    for (p = con_out, n = 0; (p = strstr(p, "period=")) != 0; p++) {
        n++;
    }
    printf("  7 x get: %u complete (%u bytes), writes past the UART's buffers: %u\n", n, con_out_n, tx_blocks - b0);
    CHECK(n == 7u && tx_blocks == b0, "console: seven gets, complete, the UART never full (%u, %u blocked bytes)", n,
          tx_blocks - b0);
    /* (0.6.6) the input at 115200 baud into the board's 64-byte RX ring while the replies wait for the UART's room: a
     * pasted 64 bytes of `get`s (2.9 KB of replies), then 400 bytes of lines with short replies; the ring holds what
     * waits (63 bytes), so a host sends at most that much past a long reply (a positive control of the model) */
    b0 = tx_blocks;
    con_out_n = 0;
    con_out[0] = 0;
    con_in_r = con_in_w = con_in_due = rx_overruns = 0;
    con_rx_paced = 1;
    for (k = 0; k < 16u; k++) {
        memcpy(con_in + con_in_w, "get\r", 4u);
        con_in_w += 4u;
    }
    run_ms(1500);
    for (p = con_out, n = 0; (p = strstr(p, "period=")) != 0; p++) {
        n++;
    }
    printf("  16 x get at the line rate: %u complete (%u bytes), RX overruns %u, writes past the buffers: %u\n", n,
           con_out_n, rx_overruns, tx_blocks - b0);
    CHECK(n == 16u && rx_overruns == 0u && tx_blocks == b0, "console: a 64-byte paste of gets (%u complete, %u RX "
          "overruns, %u blocked bytes)", n, rx_overruns, tx_blocks - b0);
    con_out_n = 0;
    con_out[0] = 0;
    con_in_r = con_in_w = con_in_due = 0;
    for (k = 0; k < 25u; k++) {
        memcpy(con_in + con_in_w, "set softstart 0\r", 16u);
        con_in_w += 16u;
    }
    run_ms(500);
    for (p = con_out, n = 0; (p = strstr(p, "ok")) != 0; p++) {
        n++;
    }
    printf("  25 x set at the line rate (400 bytes): %u ok, RX overruns %u, writes past the buffers: %u\n", n,
           rx_overruns, tx_blocks - b0);
    CHECK(n == 25u && rx_overruns == 0u && tx_blocks == b0, "console: 400 bytes of sets (%u ok, %u RX overruns, %u "
          "blocked bytes)", n, rx_overruns, tx_blocks - b0);
    con_out_n = 0;
    con_out[0] = 0;
    con_in_r = con_in_w = con_in_due = 0;
    memcpy(con_in, "status\r", 7u);
    con_in_w = 7u;
    for (k = 0; k < 6u; k++) {
        memcpy(con_in + con_in_w, "set softstart 0\r", 16u);
        con_in_w += 16u;
    }
    run_ms(1000);
    printf("  status and 96 bytes behind it (the model's positive control): RX overruns %u\n", rx_overruns);
    CHECK(rx_overruns != 0u && tx_blocks == b0, "console: the RX model counts an overrun (%u)", rx_overruns);
    con_rx_paced = 0;
    rx_overruns = 0;
    /* (0.6.6) the firmware's own messages while a status fills the UART: queued, in order, none lost, none blocking;
     * then `watch 100` with messages: its lines wait for the room too, a second still has ~10 */
    b0 = tx_blocks;
    con_out_n = 0;
    con_out[0] = 0;
    con_in_r = con_in_w = 0;
    memcpy(con_in, "status\r", 7u);
    con_in_w = 7u;
    run_ms(30);                                          /* (its first parts are out) */
    say(CZ_m_flash_err);
    say(CZ_m_tj_off);
    say(CZ_m_flash_err);
    run_ms(1000);
    p = strstr(con_out, "flash: error\r\n");
    k = p != 0 && strstr(p, "test mode: off\r\n") != 0 && strstr(strstr(p, "test mode: off\r\n"), "flash: error\r\n");
    printf("  messages during a status: in order %u, the status complete %u, writes past the buffers: %u\n", k,
           strstr(con_out, "lifetime=") != 0, tx_blocks - b0);
    CHECK(k && strstr(con_out, "lifetime=") != 0 && say_n == 0u && tx_blocks == b0, "console: messages queued in "
          "order during a status (%u, %u blocked bytes)", k, tx_blocks - b0);
    b0 = tx_blocks;
    con_out_n = 0;
    con_out[0] = 0;
    con_in_r = con_in_w = 0;
    memcpy(con_in, "watch 100\r", 10u);
    con_in_w = 10u;
    run_ms(500);
    for (k = 0; k < 6u; k++) {                           /* a burst: four queued, two written at once (never seen) */
        say(CZ_m_flash_err);
    }
    run_ms(600);
    for (p = con_out, n = 0; (p = strstr(p, "  k ")) != 0; p++) {
        n++;
    }
    for (p = con_out, k = 0; (p = strstr(p, "flash: error")) != 0; p++) {
        k++;
    }
    printf("  watch 100 for 1.1 s with six messages: %u lines, %u messages, writes past the buffers: %u\n", n, k,
           tx_blocks - b0);
    CHECK(n >= 10u && k == 6u && tx_blocks == b0, "console: watch lines and messages (%u, %u, %u blocked bytes)", n, k,
          tx_blocks - b0);
    con_out_n = 0;
    con_out[0] = 0;
    con_in_r = con_in_w = 0;
    con_in_w = (uint32_t)strlen(strcpy(con_in, "\r"));   /* (any key stops watch) */
    run_ms(50);
    con_out_n = 0;
    con_out[0] = 0;
    con_in_r = con_in_w = 0;
    con_in_w = (uint32_t)strlen(strcpy(con_in, "sett\x08 softstart 0\r"));   /* a typo, backspace, the rest */
    run_ms(100);
    CHECK(strstr(con_out, "ok") != 0 && !(sext.opts & STORE_OPT_SOFT_START), "console: backspace (%s)", con_out);
}

static int r5_scenario(const char *s)
{
    if (!strcmp(s, "r5dip")) {
        r5_dip();
    } else if (!strcmp(s, "r5fx")) {
        r5_fx();
    } else if (!strcmp(s, "r5console")) {
        r5_console();
    } else {
        return 0;
    }
    return 1;
}
