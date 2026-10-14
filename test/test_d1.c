/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* (0.6.5, D1) the reset breadcrumbs' console line (core/cli.c `before_reset=`), included by test_main.c after
 * test_console060.c (its helpers): shown after a watchdog / software / pin reset when the block holds this firmware's
 * magic (the PC in decimal, fault 0 = none); never after a power-on (random RAM) or without the magic. main.c's side
 * (the copy at boot, the tick's PC, the uptime): test_soak.c `d1`. */

static void test_d1(void)
{
    cli_t c;
    cli_status_t st;
    const char *p;

    printf("D1 reset breadcrumbs: the before_reset line\n");
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
    c.eff = &cli_set;
    c.st = &st;

    st.bc.magic = BC_MAGIC;
    st.bc.up_s = 90123u;                                 /* 1d 1:02:03 */
    st.bc.pc = 0x00004A2Eu;
    st.bc.cause = 5u;                                    /* a load access fault */
    st.reset_flags = 0x20000000u;                        /* watchdog */
    run_cli(&c, "status\r");
    p = strstr(cli_out, "before_reset=");
    CHECK(p && strncmp(p, "before_reset=1d 1:02:03 up, pc 18990, fault 5\r\n", 47) == 0,
          "watchdog after a fault: %.60s", p ? p : "(none)");
    CHECK(p && strstr(cli_out, "reset=watchdog") && strstr(cli_out, "reset=watchdog") < p, "after the reset= line");

    st.bc.cause = 0;
    st.bc.up_s = 59u;
    st.bc.pc = 0xDEADBEEFu;
    st.reset_flags = 0x10000000u | 0x04000000u;          /* software (+ pin) */
    run_cli(&c, "status\r");
    p = strstr(cli_out, "before_reset=");
    CHECK(p && strncmp(p, "before_reset=0:00:59 up, pc 3735928559, fault 0\r\n", 49) == 0, "software reset: %.50s",
          p ? p : "(none)");

    st.bc.pc = 0;
    st.reset_flags = 0x04000000u;                        /* pin alone: RAM kept */
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "before_reset=0:00:59 up, pc 0, fault 0\r\n") != NULL, "pin reset: shown");

    st.reset_flags = 0x08000000u | 0x04000000u;          /* power-on + pin: random RAM */
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "before_reset") == NULL && strstr(cli_out, "reset=power-on"), "not after a power-on");

    st.reset_flags = 0x20000000u;
    st.bc.magic = BC_MAGIC ^ 1u;                         /* not this firmware's block */
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "before_reset") == NULL, "not without the magic");
    st.bc.magic = 0;
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "before_reset") == NULL, "not on a cleared block");
}
