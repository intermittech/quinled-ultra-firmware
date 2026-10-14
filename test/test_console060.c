/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The 0.6.0 console (core/cli.c), included by test_main.c after test_features.c (its capture helpers): `watch`
 * (the line, its period, any key stops it), the status lines of the supply dips, the energy, the channel health, the
 * MCU temperature peak and the power events' dip latch, `softstart` in help / get / set. */

static void test_cli060(void)
{
    cli_t c;
    cli_status_t st;
    vsense_t vs;
    vdip_t d;
    health_t h;
    pwrev_t pw;
    pwrev_in_t in;
    char want[200];
    uint32_t x20;

    printf("console 0.6.0: watch, supply dips, energy, health, temperature peak, softstart\n");
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

    /* watch: the period (default 1 s, 100 ms .. 60 s), the line, any key stops it (and is used up) */
    run_cli(&c, "watch 500\r");
    CHECK(c.watch_ms == 500 && cli_out[0] == 0, "watch 500");
    st.uptime_s = 3723u;                                 /* 1:02:03 */
    st.vs_supply_mv = 24012;
    st.i_ma = 3450;
    st.i_now_ch_ma[0] = st.i_now_ch_ma[1] = st.i_now_ch_ma[2] = 500;
    st.i_now_ch_ma[3] = st.i_now_ch_ma[4] = 950;
    st.out[0] = st.out[1] = st.out[2] = 65535;
    st.out[3] = st.out[4] = 32768;
    st.mcu_temp_c10 = 452;
    st.mcu_peak_c10 = 517;
    st.mcu_peak_s = 3u;
    st.i_k_permille = 1000;
    vdip_init(&d);
    d.count = 2;
    st.dip = &d;
    vsense_init(&vs, 220000u, 10000u, 3280u);
    st.dip_vs = &vs;
    cli_out[0] = 0;
    cli_watch(&c);
    CHECK(strcmp(cli_out, "1:02:03 24.01 V 3.45 A 82.8 W  ch 0.50/0.50/0.50/0.95/0.95 A  out 100.0/100.0/100.0/50.0/50.0 %"
                          "  cpu 0.0 %  45.2 C  k 100.0 %  dips 2\r\n") == 0, "watch line: %s", cli_out);
    run_cli(&c, "s");
    CHECK(c.watch_ms == 0 && cli_out[0] == 0 && c.len == 0, "any key stops watch (and is used up)");
    run_cli(&c, "watch\r");
    CHECK(c.watch_ms == 1000, "watch: 1 s by default");
    run_cli(&c, "\n");
    run_cli(&c, "watch 5\r");
    CHECK(c.watch_ms == 100, "watch 5: at least 100 ms");
    run_cli(&c, "\n");
    run_cli(&c, "watch 99999\r");
    CHECK(c.watch_ms == 60000, "watch 99999: at most 60 s");
    run_cli(&c, "\n");

    /* status: the temperature peak, the supply dips, the energy, the health, the dip latch */
    x20 = 2u * 1086u;                                     /* ~20 V on the supply divider */
    d.ref_q12 = (2u * 1302u) << 12;                       /* ~24 V */
    d.count = 3;
    d.deep.t_s = 1u;
    d.deep.min_x = (uint16_t)(2u * 900u);
    d.deep.ref_x = (uint16_t)(2u * 1302u);
    d.deep.dur_ms = 12;
    d.deep.i_ma = 6200;
    d.n_log = 1;
    d.head = 0;
    d.log[0] = d.deep;
    d.log[0].t_s = 3600u;
    d.log[0].min_x = (uint16_t)x20;
    d.log[0].dur_ms = 40;
    d.log[0].i_ma = 1500;
    st.dip_ref_mv = 24020;
    memset(&test_en, 0, sizeof test_en);
    test_en.boot_mwh = 12340;                             /* the lifetime record: 1,284.3 kWh with this boot's 12 Wh */
    test_en.base_wh = 1284288u;
    test_en.lt.on_s = 77040000u;                          /* 21,400 h */
    test_en.lt.lit_s = 35460000u;                         /* 9,850 h */
    test_en.lt.ups = 1312u;
    test_en.lt.trips[1] = 2u;
    test_en.lt.dips = 14u;
    test_en.lt.peak_t = 137u;                             /* 68.5 C */
    test_en.lt.peak_a = 79u;                              /* 7.9 A */
    test_en.lt.updates = 6u;
    test_en.lt.writes = 9876u;
    health_init(&h);
    h.flags = ULTRA_HE_F_TESTED | ULTRA_HE_F_ABNORMAL;
    h.cls[0] = ULTRA_HE_NORMAL;
    h.cls[1] = ULTRA_HE_ABNORMAL | ULTRA_HE_R_NOT_RISING;
    h.cls[2] = ULTRA_HE_NORMAL;
    h.cls[3] = ULTRA_HE_NO_LOAD;
    h.cls[4] = ULTRA_HE_ABNORMAL | ULTRA_HE_R_ABOVE | ULTRA_HE_R_BELOW;
    h.test_dev[0] = 5; h.test_dev[1] = 0; h.test_dev[2] = -2; h.test_dev[4] = 45;
    h.model_dev[0] = 2; h.model_dev[1] = -1; h.model_dev[4] = 35;
    h.dark_ma = 10;
    st.hl = &h;
    pwrev_init(&pw, 0);
    memset(&in, 0, sizeof in);
    in.limit_ma = 8000;
    in.factor = 65535;
    in.dips = 3;
    pwrev_update(&pw, 1, &in);
    st.pw = &pw;
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "mcu_temp=45.2 C (die), vdda=0.00 V, peak 51.7 C 1:02:00 ago\r\n") != NULL,
          "status: the temperature peak");
    CHECK(strstr(cli_out, "supply_dips=3, reference 24.02 V\r\n") != NULL, "status: supply_dips");
    sprintf(want, "supply_dip_deepest=1:02:02 ago, %u.%02u V (69 %%), 12 ms, 6.20 A\r\n",
            (vsense_mv(&vs, 1800u, 1u) + 5u) / 1000u, ((vsense_mv(&vs, 1800u, 1u) + 5u) / 10u) % 100u);
    CHECK(strstr(cli_out, want) != NULL, "status: %s", want);
    sprintf(want, "supply_dip 1=0:02:03 ago, %u.%02u V (83 %%), 40 ms, 1.50 A\r\n",
            (vsense_mv(&vs, x20, 1u) + 5u) / 1000u, ((vsense_mv(&vs, x20, 1u) + 5u) / 10u) % 100u);
    CHECK(strstr(cli_out, want) != NULL, "status: %s", want);
    CHECK(strstr(cli_out, "energy=12.34 Wh since boot\r\n") != NULL, "status: energy");
    CHECK(strstr(cli_out, "lifetime=1284.30 kWh, powered 21400.0 h, lit 9850.0 h, 1312 power-ups, trips 2 / 0, dips 14, "
                          "max 68.5 C / 7.9 A, wd 0, updates 6, 9876 records\r\n") != NULL, "status: lifetime");
    CHECK(strstr(cli_out, "health_test R/G/B/W1/W2=ok/FAULT:rise/ok/none/FAULT:high:low, change 5/0/-2/-/45 %\r\n")
          != NULL, "status: health_test");
    CHECK(strstr(cli_out, "health_model R/G/B/W1/W2=2/-1/-/-/35 %, dark 0.01 A\r\n") != NULL, "status: health_model");
    CHECK(strstr(cli_out, ", latched: dip\r\n") != NULL, "status: the dip latch");
    h.flags |= ULTRA_HE_F_DARK;
    h.dark_ma = 120;
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, ", dark 0.12 A (!)\r\n") != NULL, "status: dark current flagged");
    h.flags = 0;
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "health_test") == NULL && strstr(cli_out, "health_model") != NULL, "status: no test since boot");
    d.in_dip = 1;
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "supply_dips=3 (one now), reference") != NULL, "status: a dip now");
    CHECK(strlen(cli_out) + 1u < sizeof cli_out, "status fits the capture (%u bytes)", (unsigned)strlen(cli_out));

    /* softstart */
    run_cli(&c, "help\r");
    CHECK(strstr(cli_out, "watch [ms]") && strstr(cli_out, "softstart: 0 off 1 on"), "help: watch, softstart");
    st.softstart = 1;
    run_cli(&c, "get\r");
    CHECK(strstr(cli_out, "softstart=1\r\n") != NULL, "get softstart");
    run_cli(&c, "set softstart 0\r");
    CHECK(strstr(cli_out, "ok\r\n") != NULL, "set softstart 0");
    run_cli(&c, "set softstart 2\r");
    CHECK(strstr(cli_out, "error") != NULL, "set softstart 2: error");
}
