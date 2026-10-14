/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* r5ovr (0.6.5 D4), included by test_soak.c: the overrun counters. An Ultra hub at 50 fps with the main loop on time:
 * none (no `overruns=` line, nothing in watch). Then the main loop stops for 12 ms while the hub sends a frame every
 * ~1.5 ms: the first capture and frame end wait in their mailboxes, the later ones are dropped / not handled
 * (counted), and the render ms comes ~11 ms late; then a settings write (a flash stall of 16 ms or more): not
 * counted. The board shows the hub's frames again right after. */
static void sc_r5ovr(void)
{
    uint32_t i, f0, f1, ms0, ms1, w0, cap, fe, late;
    unsigned a, b, c;
    const char *p;
    printf("soak r5ovr: overrun counters (captures dropped, frame ends not handled, render ms late)\n");
    boot(0, 0, 0);
    host_on = 1;
    cmd_def(0, 0, 3, 2800);                              /* RGB on CH1-3 */
    cmd_set(0, 32768, 0, 0);
    run_ms(3000);
    console("status");
    CHECK(ultra_on && ovr[0] == 0u && ovr[1] == 0u && ovr[2] == 0u && !strstr(con_out, "overruns="),
          "r5ovr: on time, nothing counted, no status line (%u %u %u)", ovr[0], ovr[1], ovr[2]);
    console("watch 500");
    run_ms(1200);
    CHECK(strstr(con_out, "dips 0\r\n") && !strstr(con_out, " ovr "), "r5ovr: watch without the counters");
    console("");

    /* the main loop stops for 12 ms (the ISR alone), the hub sends a frame every 30 ticks or as soon as it can */
    host_period = 30u;
    host_next = sim_ticks;
    f0 = frames;
    ms0 = ms_now;
    for (i = 0; i < 240u; i++) {
        sim_tick();
    }
    f1 = frames;
    ms1 = ms_now;
    pass();
    cap = ovr[0];
    fe = ovr[1];
    late = ovr[2];
    printf("  12 ms late: %u frames ended meanwhile; dropped captures %u, frame ends %u, ms late %u (ms %u -> %u)\n",
           f1 - f0, cap, fe, late, ms0, ms1);
    CHECK(f1 - f0 >= 5u && fe == f1 - f0 - 1u && (cap == fe || cap == fe + 1u),
          "r5ovr: each frame after the first: its capture dropped, its end not handled (%u frames: %u, %u)", f1 - f0,
          cap, fe);
    CHECK(late >= ms1 - ms0 - 2u && late <= ms1 - ms0 - 1u, "r5ovr: the render ms %u late (%u ms stopped)", late,
          ms1 - ms0);
    host_period = 400u;
    run_ms(500);
    CHECK(ovr[0] == cap && ovr[1] == fe && ovr[2] == late && out_now[0] != 0u, "r5ovr: on time again, the light shown");

    /* a settings write: its stall (no frames, the gate closed) is housekeeping, not an overrun */
    w0 = W_SETTINGS();
    console("set loss 2");
    run_ms(3000);
    CHECK(W_SETTINGS() > w0 && ovr[2] == late && ovr[0] == cap && ovr[1] == fe, "r5ovr: a flash write isn't counted "
          "(writes %u -> %u, ms late %u)", w0, W_SETTINGS(), ovr[2]);

    console("status");
    p = strstr(con_out, "overruns=");
    CHECK(p && sscanf(p, "overruns=%u captures, %u frame ends, %u ms late", &a, &b, &c) == 3 && a == cap &&
          b == fe && c == late, "r5ovr: status line: %.60s", p ? p : "(none)");
    console("watch 500");
    run_ms(600);
    p = strstr(con_out, " ovr ");
    CHECK(p && sscanf(p, " ovr %u/%u/%u", &a, &b, &c) == 3 && a == cap && b == fe && c == late,
          "r5ovr: watch: %.40s", p ? p : "(none)");
    console("");
}
