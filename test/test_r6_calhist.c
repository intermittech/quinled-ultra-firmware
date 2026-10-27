/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Included at the end of test_soak.c: the calibration-history reproducers, each a scenario here
 * (soak_tests.exe <name>).
 *
 *   r6gcut    (R6-02) a calibration history record cut so that its CRC reads like erased flash (0xFFFF / 0xE339)
 *             never replaces the committed one, nor does the erase ahead destroy it; every half-word cut, both A/B
 *             directions, a sequence wrap
 *   r6gadmit  (R6-03) the calibration history's write due (an erase first) in the pass whose outputs_update makes an
 *             output bright: no erase while bright
 *   r6gretry  (R6-06) the history's write pending while VDD is below the PVD for a second, a status queued: the tries
 *             bounded, the UART never overfilled, the write done once VDD is back and a token refilled
 */

#if MON_EXTRA
static void sc_r6gcut(void)
{
    calhist_t old, next, got, torn;
    uint32_t k, pattern, found, cut, dir, sweep = 0;
    static const uint16_t erased[2] = {0xffff, 0xe339};
    printf("soak r6gcut: a torn calibration history record never replaces the committed one (R6-02)\n");
    for (pattern = 0; pattern < 2u; pattern++) {
        memset(&old, 0, sizeof old);
        old.n = 1; old.tests = 1; old.e[0].ma[0] = 1000;
        calhist_pack(&old);
        next = old;
        next.n = 2; next.tests = 2; next.e[1].ma[0] = 2000;
        /* Find a plausible hour count whose prefix CRC collides with erased flash.
         * Cut before entry 1's currents: its first current is still erased. */
        cut = 26;
        found = 0;
        for (k = 0; k < 65536u; k++) {
            uint32_t j;
            next.seq = old.seq;
            next.e[1].lit_h = (uint16_t)k;
            calhist_pack(&next);
            for (j = 0; j < sizeof torn / 2u; j++) ((uint16_t *)&torn)[j] = erased[pattern];
            memcpy(&torn, &next, cut);
            if (crc16_ccitt((const uint8_t *)&torn, 62) == erased[pattern]) { found = 1; break; }
        }
        CHECK(found, "collision search found a prefix");
        memset((void *)(uintptr_t)CALHIST_ADDR, 0xff, 256);
        memset((void *)(uintptr_t)CALHIST_B_ADDR, 0xff, 256);
        memcpy((void *)(uintptr_t)CALHIST_ADDR, &old, sizeof old);
        memcpy((void *)(uintptr_t)CALHIST_B_ADDR, &torn, sizeof torn);
        memset(&got, 0, sizeof got);
        (void)calhist_load(&got);
        printf("erased=%04x, cut=%u, hours=%u, loaded seq=%u, current=%u\n",
               erased[pattern], cut, k, got.seq, got.e[1].ma[0]);
        CHECK(got.seq == old.seq, "torn history replaces the committed history");
        if (got.seq != old.seq) {
            (void)calhist_prepare();
            printf("old history after erase-ahead: valid=%d\n", calhist_valid((const uint8_t *)(uintptr_t)CALHIST_ADDR));
        }
        CHECK(calhist_valid((const uint8_t *)(uintptr_t)CALHIST_ADDR), "erase-ahead destroyed the committed history");
        /* Every half-word boundary in both A/B directions, including complete
         * writes and a sequence wrap. The returned record must be old or complete-new. */
        for (dir = 0; dir < 2u; dir++) {
            for (cut = 0; cut <= CALHIST_REC_BYTES; cut += 2u) {
                uint32_t a = dir ? CALHIST_B_ADDR : CALHIST_ADDR;
                uint32_t b = dir ? CALHIST_ADDR : CALHIST_B_ADDR;
                uint32_t j;
                old.seq = 0xfffffffeu; calhist_pack(&old);
                next = old; next.n = 2; next.tests = 2; next.e[1].ma[0] = 2000;
                calhist_pack(&next);
                for (j = 0; j < 128; j++) {
                    ((uint16_t *)(uintptr_t)a)[j] = erased[pattern];
                    ((uint16_t *)(uintptr_t)b)[j] = erased[pattern];
                }
                memcpy((void *)(uintptr_t)(a + 3u * CALHIST_REC_BYTES), &old, sizeof old);
                memcpy((void *)(uintptr_t)b, &next, cut);
                (void)calhist_load(&got);
                CHECK(memcmp(&got, cut == CALHIST_REC_BYTES ? &next : &old, sizeof got) == 0,
                      "cut %u direction %u pattern %u: wrong record", cut, dir, pattern);
                sweep++;
            }
        }
    }
    printf("half-word boundary checks: %u\n", sweep);
}

static void sc_r6gadmit(void)
{
    uint32_t before, may;
    uint16_t bright[5] = {40000, 0, 0, 0, 0};
    printf("soak r6gadmit: an erase decided in the pass that makes the outputs bright (R6-03)\n");
    boot(0, 0, 0);
    run_ms(2000);
    sext.opts = (uint8_t)(sext.opts & ~STORE_OPT_SOFT_START);
    /* Match a pending clear after defaults, while the history pair needs an erase. */
    memset((void *)(uintptr_t)CALHIST_ADDR, 0, 256);
    (void)calhist_load(&chist);
    chist_pend = 1;
    /* A frame or effect sets a bright value after vsense_service sampled the old table.
     * outputs_update runs before the calibration/history stores in this same pass. */
    flash_hi_ms = ms_now - FLASH_LOW_MS - 1u;
    smooth_fade(&sm, bright, tick_now, 0);
    outputs_dirty = 1;
    before = fl_bright_erases;
    pass();
    may = flash_may(1u, 0u);
    printf("output=%u, bright erases=%u, admission=%u, history pending=%u\n",
           out_now[0], fl_bright_erases - before, may, chist_pend);
    CHECK(out_now[0] >= FLASH_LOW_LEVEL, "setup: output became bright");
    CHECK(fl_bright_erases == before, "an erase ran after the outputs became bright in the same pass");
}

static void sc_r6gretry(void)
{
    uint32_t before, after, i;
    printf("soak r6gretry: the calibration history's retries under a PVD refusal and a queued status (R6-06)\n");
    soak_busy_model = 1;                                 /* (its runner: uart_busy() from the queued TX) */
    boot(0, 0, 0);
    run_ms(35000);                     /* include erase-ahead, enabled after LIFE_PREP_S */
    chist.n = 1; chist.tests = 1; chist.e[0].ma[0] = 1000;
    chist_pend = 1;
    sim_vdd_low = 1;
    /* Queue a status so there is simultaneous back-pressure. PVD refuses immediately. */
    con_in_r = con_in_w = 0;
    memcpy(con_in, "status\r", 7); con_in_w = 7;
    before = stat.fl_errs;
    for (i = 0; i < 20000; i++) { sim_tick(); pass(); }
    after = stat.fl_errs;
    printf("1 s PVD refusal: attempts=%u, blocked bytes=%u, pending=%u\n", after-before, tx_blocks, chist_pend);
    CHECK(after-before <= 2u * CAL_BUDGET_CAP, "history / erase-ahead retries have no bound");
    CHECK(tx_blocks == 0, "error reports overfilled the UART buffers");
    CHECK(chist_pend, "history remains pending");
    sim_vdd_low = 0;
    jump_ms(CAL_REFILL_MS);
    run_ms(100);
    CHECK(!chist_pend && W_HIST() == 1u, "pending history commits after supply recovery and refill");
}
#endif

static int r6g_scenario(const char *s)
{
    (void)s;                                             /* (the receiver: none of these) */
    if (0) {
#if MON_EXTRA
    } else if (!strcmp(s, "r6gcut")) {
        sc_r6gcut();
    } else if (!strcmp(s, "r6gadmit")) {
        sc_r6gadmit();
    } else if (!strcmp(s, "r6gretry")) {
        sc_r6gretry();
#endif
    } else {
        return 0;
    }
    return 1;
}
