/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Review round 6 (claude-fw), included at the end of test_soak.c: claude-fw's soak scenarios (each was a
 * test_r6_soak_<x>.c of its own), soak_tests.exe <name>.
 *
 *   r6race    (B4, patch 06) the calibration history's write in the pass the outputs rise
 *   r6hist    (B2, patch 02) the calibration history's write failing on a dark board: its retries bounded
 *   r6erase   (B3, patch 04) the settings' erase ahead failing on a dark board: its retries bounded
 *   r6fxclear (B3, patch 04) the power-on effects' clear (a factory reset) failing: its retries bounded
 * (0.6.7: one design for every failing operation, main.c flash_may's hold-off after a failure, doubling, and the
 * calibration history's budget; the retries' timing as the tests' windows allow)
 */

#if MON_EXTRA
/* r6race (patch 06): the calibration's and the calibration history's writes decide on the outputs
 * the PWM runs.
 *
 * flash_may's "low" is flash_hi_ms, which vsense_service updates in the ms block from out_now (the tables the PWM runs).
 * Every store decides inside that ms block, before outputs_update rebuilds the tables, so the outputs can't rise
 * during the operation (main.c: "nothing can cross the level during an operation"). The calibration write (0.6.4) and
 * the calibration history's (0.6.6) sit after outputs_update and run every pass: in the pass where the outputs have
 * just been rebuilt brighter, flash_hi_ms is still the old one, so they take FM_LOW: an erase while an output is at or
 * above 15 % (16 ms at half the PWM frequency) and a program without the hour's token. Here: the history's next
 * record needs an erase (its page A full, page B not erased), the hour's token is used up, the board dark; `identify`
 * starts the acknowledge pattern (R at 25 %) and the history's write becomes due in the pass whose ms block turns the
 * pattern on. Before the fix: an erase and a program while R is at 25 %. After: both just before the tables change. */
static uint8_t r6r_block;

static uint32_t r6r_fault(uint32_t addr, uint32_t halfwords, uint32_t erase_addr)
{
    (void)addr;
    (void)halfwords;
    return r6r_block && erase_addr == CALHIST_B_ADDR ? FLASH_ERR_EOP : 0u;   /* (the boot's erase of page B) */
}

static void sc_r6race(void)
{
    calhist_t h;
    uint32_t k, e0, p0;
    printf("soak r6race: the calibration history's write in the pass the outputs rise\n");
    memset(&h, 0, sizeof h);
    for (k = 0; k < CALHIST_RECS; k++) {                 /* page A full: the next record goes to page B, erased first */
        h.tests = (uint16_t)(k + 1u);
        calhist_pack(&h);
        memcpy(fl + (CALHIST_ADDR - FL_BASE) + k * CALHIST_REC_BYTES, &h, sizeof h);
    }
    memset(fl + (CALHIST_B_ADDR - FL_BASE), 0x00, 256u);  /* page B: not erased */
    fl_fault = r6r_fault;
    r6r_block = 1;                                       /* (the boot's erase ahead of it fails: still due after) */
    boot(0, 0, 0);
    r6r_block = 0;
    flash_err = 0;                                       /* (0.6.7: the staged failure's hold-off; this test is
                                                            about the order, not the retries) */
    console("set softstart 0");                          /* (on, it ramps an output up from dark: the rise is slower) */
    run_ms(3000);                                        /* dark: low (no erase ahead before LIFE_PREP_S) */
    CHECK(calhist_pending() == 2u, "r6race: the history's erase is due (%u)", calhist_pending());
    (void)budget_take(&flash_bright, ms_now, 1u);        /* the hour's token used up */
    con_out_n = 0;
    con_in_r = con_in_w = 0;
    strcpy(con_in, "identify\r");
    con_in_w = 9;
    while (!ack.active) {
        run_ticks(1);                                    /* the line: the pattern starts (on at its first ms) */
    }
    while (poll_sub != POLL_HZ / 1000u - 1u) {
        run_ticks(1);                                    /* up to the pass whose ms block turns it on */
    }
    e0 = fl_bright_erases;
    p0 = fl_bright_progs;
    chist_pend = 1;                                      /* the history's write due from this pass on */
    run_ticks(1);
    run_ms(100);
    printf("  R=%u: %u erases and %u programs while an output was at or above 15 %%, the write %s\n", out_now[CH_R],
           fl_bright_erases - e0, fl_bright_progs - p0, chist_pend ? "pending" : "done");
    CHECK(fl_bright_erases == e0 && fl_bright_progs == p0, "r6race: %u erases and %u programs while bright",
          fl_bright_erases - e0, fl_bright_progs - p0);
    CHECK(!chist_pend, "r6race: the history's record written (before the outputs rose)");
    fl_fault = 0;
}


/* r6hist (patch 02): the calibration history's write has a budget (main.c chist_budget).
 *
 * 0.6.6's M2 write (`chist_pend && flash_may(...)`) was the only store write without a token: a write that fails
 * (a program without EOP or WRP on the history's pages, FLASH_VERIFY's read-back: bench items of the upper CodeFlash)
 * was tried again on every main-loop pass while the outputs were low. Each try used up an entry, every fourth an
 * erase of the same page (pagepair.c: a page of failed writes is erased and used again), so the page saw hundreds of
 * erases a minute (10k cycles: minutes), each try stalled the main loop (the frame tracking disarmed: frames lost) and
 * said "flash: error" (the queue full, the UART then waits). Here every program into the history's two pages fails
 * (the erases work), after a factory reset (it begins the history anew: a record pending) on a dark board.
 * Before the fix: thousands of programs and hundreds of erases in 20 s. After: the first try and CAL_BUDGET_CAP
 * more, at most two erases, then one try per CAL_REFILL_MS (the first write after a stored test or a factory reset
 * takes no token: soak cfg's reset clears the history in flash at once). */
static uint32_t r6h_progs, r6h_erases;

static uint32_t r6h_fault(uint32_t addr, uint32_t halfwords, uint32_t erase_addr)
{
    uint32_t a = (erase_addr ? erase_addr : addr) & ~0xFFu;
    (void)halfwords;
    if (a != CALHIST_ADDR && a != CALHIST_B_ADDR) {
        return 0;                                        /* (every other store: as before) */
    }
    if (erase_addr) {
        r6h_erases++;
        return 0;                                        /* the erases work */
    }
    r6h_progs++;
    return FLASH_ERR_EOP;                                /* every program into the history's pages fails */
}

static void sc_r6hist(void)
{
    uint32_t errs0;
    printf("soak r6hist: the calibration history's write failing on a dark board: its retries bounded\n");
    boot(0, 0, 0);
    run_ms(12000);                                       /* dark: a low moment, past LIFE_PREP_S */
    fl_fault = r6h_fault;
    errs0 = stat.fl_errs;
    console("defaults");                                 /* a factory reset: the history begins anew, written when low */
    run_ms(20000);
    printf("  20 s after the reset: %u programs and %u erases on the history's pages, %u failed operations, %u "
           "bytes the UART would have waited for\n", r6h_progs, r6h_erases, (uint32_t)(uint16_t)(stat.fl_errs - errs0),
           tx_blocks);
    CHECK(r6h_progs >= 1u, "r6hist: the history's write was tried");
    CHECK(r6h_progs <= CAL_BUDGET_CAP + 1u, "r6hist: %u tries of the history's write in 20 s (budget %u + 1)",
          r6h_progs, CAL_BUDGET_CAP);
    CHECK(r6h_erases <= 2u, "r6hist: %u erases of the history's pages in 20 s", r6h_erases);
    CHECK(tx_blocks == 0u, "r6hist: the console's messages made the UART wait (%u bytes)", tx_blocks);
    run_ms(5000);
    CHECK(r6h_progs <= CAL_BUDGET_CAP + 1u, "r6hist: no further try before a token (%u)", r6h_progs);
    fl_fault = 0;
}

#endif

/* r6erase / r6fxclear (patch 04): a failing erase ahead (or the factory reset's clear of the power-on effects)
 * is tried again after FLASH_RETRY_MS, not every ms.
 *
 * flash_erase_ahead (main.c) runs every ms while the outputs are low; after a failed erase it ran again the next ms
 * (0.6.4 to 0.6.6: flash_stall_end sets flash_recheck whatever the result, and fx_clear_pend bypasses it). A failure
 * that stays (an erase that ends without EOP or with WRP on the upper CodeFlash: bench items; FLASH_ERR_VDD while VDD
 * sits below the PVD) then means an erase of the same page every ~17 ms: 10k cycles within minutes when the erase
 * itself works and only its status is wrong, the main loop stalled 16 ms of every 17 (frames dropped, forwarding gaps),
 * and a "flash: error" each time (the say queue full, then the UART waits). Here the erase of the settings' page B
 * (due ahead: page A holds the newest record) fails, then the power-on effects' clear after a factory reset fails.
 * Before the fix: hundreds of erase attempts in 20 s. After: one, then one per FLASH_RETRY_MS; once the fault is gone
 * the erase is done. */
static uint32_t r6e_page, r6e_tries, r6e_fail = 1;

static uint32_t r6e_fault(uint32_t addr, uint32_t halfwords, uint32_t erase_addr)
{
    (void)addr;
    (void)halfwords;
    if (erase_addr == r6e_page || (r6e_page == SCENE_FX_ADDR && erase_addr == SCENE_FX_B_ADDR)) {
        r6e_tries++;
        return r6e_fail ? FLASH_ERR_EOP : 0u;            /* (the erase ends without EOP: nothing erased) */
    }
    return 0;
}

static void sc_r6erase(void)
{
    settings_t s;
    store_ext_t e;
    uint32_t t20;
    printf("soak r6erase: the settings' erase ahead failing on a dark board: its retries bounded\n");
    settings_default(&s);
    store_ext_clear(&e);
    store_pack_ext(&s, &e, 1u, (store_slot_t *)(fl + (SETTINGS_B_ADDR - FL_BASE)));   /* page B: an older record */
    store_pack_ext(&s, &e, 2u, (store_slot_t *)(fl + (SETTINGS_ADDR - FL_BASE)));     /* page A: the newest */
    r6e_page = SETTINGS_B_ADDR;
    fl_fault = r6e_fault;
    boot(0, 0, 0);
    run_ms(20000);
    t20 = r6e_tries;
    printf("  20 s: %u erase attempts of page B, %u bytes the UART would have waited for\n", t20, tx_blocks);
    CHECK(t20 >= 1u && t20 <= 2u, "r6erase: %u erase attempts in 20 s", t20);
    CHECK(tx_blocks == 0u, "r6erase: the console's messages made the UART wait (%u bytes)", tx_blocks);
    r6e_fail = 0;                                        /* the fault is gone: the erase is done at the next retry */
    run_sparse(11u * 60u, 0);
    printf("  11 min later: %u attempts, the erase %s\n", r6e_tries, settings_pending() ? "still due" : "done");
    CHECK(settings_pending() == 0u && r6e_tries <= t20 + 2u, "r6erase: the retry after the fault (%u attempts, "
          "pending %u)", r6e_tries, settings_pending());
    fl_fault = 0;
}

static void sc_r6fxclear(void)
{
    uint32_t t20;
    printf("soak r6fxclear: the power-on effects' clear (a factory reset) failing on a dark board: retries bounded\n");
    boot(0, 0, 0);
    run_ms(12000);                                       /* dark, past LIFE_PREP_S */
    r6e_page = SCENE_FX_ADDR;
    fl_fault = r6e_fault;
    console("defaults");
    run_ms(20000);
    t20 = r6e_tries;
    printf("  20 s after the reset: %u erase attempts of the effects' pages, %u bytes the UART would have waited for\n",
           t20, tx_blocks);
    CHECK(t20 >= 1u && t20 <= 2u, "r6fxclear: %u erase attempts in 20 s", t20);
    CHECK(tx_blocks == 0u, "r6fxclear: the console's messages made the UART wait (%u bytes)", tx_blocks);
    r6e_fail = 0;
    run_sparse(11u * 60u, 0);
    printf("  11 min later: %u attempts, the clear %s\n", r6e_tries, fx_clear_pend ? "still pending" : "done");
    CHECK(!fx_clear_pend && r6e_tries <= t20 + 2u, "r6fxclear: the retry after the fault (%u attempts)", r6e_tries);
    fl_fault = 0;
}


static int r6c_scenario(const char *s)
{
    (void)s;                                             /* (the receiver: none of these) */
    if (0) {
#if MON_EXTRA
    } else if (strcmp(s, "r6race") == 0) {
        sc_r6race();
    } else if (strcmp(s, "r6hist") == 0) {
        sc_r6hist();
#endif
    } else if (strcmp(s, "r6erase") == 0) {
        sc_r6erase();
    } else if (strcmp(s, "r6fxclear") == 0) {
        sc_r6fxclear();
    } else {
        return 0;
    }
    return 1;
}
