/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Integration tests: included at the end of test_soak.c, on its hardware model. Each scenario
 * runs in a fresh process: soak_tests.exe <name>. */

/* run until the next tick is the poll ISR's 1 ms tick (the one that calls hw_adc_ms) */
static void r4_to_ms_tick(void)
{
    uint32_t m = ms_now;
    while (ms_now == m) {
        run_ticks(1);
    }
    run_ticks(19);
}

/* H7: the temperature's injected sequence holds PA7's conversion for 84 us, so the tick after the one that
 * started it reads the same sample again: one high conversion must not count as the two consecutive ticks of a fast
 * trip. A real short (fresh samples before and after the sequence) still trips. */
static void r4_h7(void)
{
    boot(0, 0, 0);
    run_ms(1500);
    CHECK(!sim_forced && is.tripped == ISENSE_TRIP_NONE, "h7: no trip at the start");
    r4_to_ms_tick();
    soak_temp_req = 1;                                   /* the next 1 ms tick starts the temperature's sequence */
    short_ticks = 1;                                     /* one high sample: the conversion just before it */
    run_ticks(1);
    CHECK(soak_adc_held && !sim_forced, "h7: the start tick: the sequence started, one high sample counted");
    run_ticks(1);
    CHECK(!sim_forced && is.tripped == ISENSE_TRIP_NONE, "h7: the held sample is not counted again (no trip)");
    run_ticks(20);
    CHECK(!sim_forced && is.tripped == ISENSE_TRIP_NONE, "h7: still no trip a ms later");
    /* a real short across the sequence: high before it and high after it (two fresh samples) */
    run_ms(100);
    r4_to_ms_tick();
    soak_temp_req = 1;
    short_ticks = 3;
    run_ticks(2);
    CHECK(!sim_forced, "h7: a short: not yet (one fresh sample, one held)");
    run_ticks(1);
    CHECK(sim_forced && is.tripped == ISENSE_TRIP_FAST, "h7: a short: tripped on the next fresh sample (%u)",
          (unsigned)is.tripped);
    /* without the sequence: two consecutive high samples trip */
    run_ms(3000);
    CHECK(!sim_forced, "h7: retried");
    r4_to_ms_tick();
    run_ticks(5);
    short_ticks = 2;
    run_ticks(2);
    CHECK(sim_forced && is.tripped == ISENSE_TRIP_FAST, "h7: two high samples in a row trip");
}

/* The frame tracker's 8-bit wrap count wrapped itself after 2^24 edges; a frame ending there (count 0,
 * wraps 0) never ended (no re-arm, no classification). Saturated at 255: it ends, with a length no protocol has. */
static void r4_wraps(void)
{
    uint32_t k;
    v_rx_setup(PMODE_AUTO, PROTO_NONE);
    for (k = 40; k < 16777216u; k += 40) {
        soak_tim4.CNT = (uint16_t)k;
        poll_isr();
    }
    soak_tim4.CNT = 0;                                   /* the last edges: 2^24 in all, the count back at 0 */
    poll_isr();
    for (k = 0; k < 4u; k++) {
        poll_isr();
    }
    CHECK(fe_ready && fe_bits == 255u * 65536u, "wraps: a frame of 2^24 edges ends (ready %u, %u bits)",
          (unsigned)fe_ready, (unsigned)fe_bits);
}

/* the flash policy (main.c flash_may, decided 2026-10-14): bright = programs into erased flash only, at
 * most one an hour (coalesced), never an erase; dim (< 15 %) or dark = anything, and the stores' next pages are erased
 * ahead; nothing fading up counts as dim; the console's `save` writes at once; a failed write stays pending */
static void r4_flash(void)
{
    uint32_t w0, p0;
    boot(0, 0, 0);
    host_on = 1;
    cmd_def(0, 0, 3, 2800);
    cmd_set(0, 65535, 0, 0);                             /* R, G, B full: bright */
    host_next = sim_ticks;
    run_ms(5000);
    CHECK(out_now[0] == 65535u && W_SETTINGS() == 0u, "flash: bright, the sticky protocol deferred (%u)", W_SETTINGS());
    /* bright: host-managed changes (CONFIG 23, stored, no blink): one write an hour (the first may take the token the
     * boot left) */
    cmd_config(ULTRA_CFG_SOFT_START, 0);
    host_next = sim_ticks;
    run_ms(10u * 60000u);
    w0 = W_SETTINGS();
    cmd_config(ULTRA_CFG_SOFT_START, 1);
    host_next = sim_ticks;
    run_ms(10u * 60000u);
    CHECK(W_SETTINGS() == w0 && settings_dirty && (sext.opts & STORE_OPT_SOFT_START),
          "flash: bright: a change waits for the hour's write (%u)", W_SETTINGS() - w0);
    run_sparse(3600u, 0);
    CHECK(W_SETTINGS() == w0 && settings_dirty, "flash: bright remains deferred past an hour (%u)", W_SETTINGS() - w0);
    CHECK(fl_bright_erases == 0u, "flash: no erase while bright (%u)", fl_bright_erases);
    /* the console's save: at once, also while bright */
    cmd_config(ULTRA_CFG_SOFT_START, 0);
    host_next = sim_ticks;
    run_ms(100);
    w0 = W_SETTINGS();
    con_out_n = 0;
    console("save");
    CHECK(W_SETTINGS() == w0 && strstr(con_out, "pending") != 0, "flash: `save` while bright stays pending (%u)",
          W_SETTINGS() - w0);
    /* dim (3.6 %): every store's next page is erased ahead (nothing left to erase) */
    cmd_set(0, 20000, 0, 0);
    host_next = sim_ticks;
    run_ms(20u * 1000u);
    CHECK(out_now[0] > 0u && flash_may(1u, 0u) == 0u, "flash: dim (%u) still forbids flash",
          out_now[0]);
    CHECK(flash_may(0u, FM_PROG) == 0u,
          "flash: dim: every next page erased ahead (%u %u %u %u)", settings_pending(), cal_pending(), life_pending(),
          scene_pending());
    CHECK(fl_bright_erases == 0u, "flash: still no erase while bright");
    /* fading up from dim: not low, even while the outputs are still below 15 % */
    cmd_set(0, 65535, 3000, ULTRA_SET_SMOOTH);
    host_next = sim_ticks;
    run_ms(150);
    CHECK(out_now[0] > 0u && flash_may(1u, 0u) != FM_LOW, "flash: fading up (%u): not low", out_now[0]);
    run_ms(5000);
    CHECK(out_now[0] == 65535u && flash_may(1u, 0u) != FM_LOW, "flash: bright again");
    /* failing writes (dark): the change stays pending and the console says so; written once the flash takes it */
    cmd_set(0, 0, 0, 0);
    host_next = sim_ticks;
    run_ms(2000);
    w0 = W_SETTINGS();
    p0 = fl_bright_progs;
    fl_fail = 1000u;
    con_out_n = 0;
    cmd_config(ULTRA_CFG_SOFT_START, 1);
    host_next = sim_ticks;
    run_ms(3000);
    CHECK(settings_dirty && W_SETTINGS() == w0 && strstr(con_out, "flash: error") != 0,
          "flash: failing writes: the change stays pending (%u written)", W_SETTINGS() - w0);
    fl_fail = 0;
    run_sparse(4u * 3600u, 0);
    CHECK(!settings_dirty && W_SETTINGS() == w0 + 1u, "flash: ... written later (%u)", W_SETTINGS() - w0);
    CHECK(fl_bright_progs == p0, "flash: dark: no bright programs (%u)", fl_bright_progs - p0);
}

static int r4_scenario(const char *s)
{
    if (!strcmp(s, "r4flash")) {
        r4_flash();
    } else if (!strcmp(s, "r4wraps")) {
        r4_wraps();
    } else if (!strcmp(s, "r4h7")) {
        r4_h7();
    } else {
        return 0;
    }
    return 1;
}
