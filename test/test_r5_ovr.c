/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Overrun counters (0.6.5 D4) in the console, included by test_main.c after test_features.c (its console helpers):
 * the status line and the watch part only when a counter is not 0 (or no counters: NULL). The counting itself runs on
 * main.c in the soak (scenario r5ovr). */
static void test_r5_ovr(void)
{
    cli_t c;
    cli_status_t st;
    uint32_t ovr[3] = { 0, 0, 0 };
    printf("overrun counters (0.6.5 D4): status / watch lines\n");
    memset(&c, 0, sizeof c);
    memset(&st, 0, sizeof st);
    settings_default(&cli_set);
    config_rx_init(&cli_rx);
    st.version = "test";
    st.steps = 5760;
    st.en = &test_en;
    c.write = cap_write;
    c.apply = cap_apply;
    c.save_now = cap_save;
    c.set = &cli_set;
    c.st = &st;
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "config_accepted=") && !strstr(cli_out, "overruns"), "no counters (NULL): no line");
    st.ovr = ovr;
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "config_accepted=") && !strstr(cli_out, "overruns"), "all 0: no line");
    cli_out[0] = 0;
    cli_watch(&c);
    CHECK(strstr(cli_out, "  k 0.0 %\r\n") && !strstr(cli_out, "ovr"), "watch without them: %s", cli_out);
    ovr[2] = 12u;
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "\r\noverruns=0 captures, 0 frame ends, 12 ms late\r\nconfig_accepted=") != NULL,
          "one counter: the line, before config_accepted");
    ovr[0] = 4000000000u;
    ovr[1] = 7u;
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "overruns=4000000000 captures, 7 frame ends, 12 ms late\r\n") != NULL, "all three");
    cli_out[0] = 0;
    cli_watch(&c);
    CHECK(strstr(cli_out, "  ovr 4000000000/7/12  k 0.0 %\r\n") != NULL, "watch: %s", cli_out);
}
