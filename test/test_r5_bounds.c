/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The console's outputs bounded (the parted status, #39), included by test_main.c after
 * test_console060.c (its helpers): with every optional line and the longest values, each part of `status` and of
 * `help` (cli_poll), and every one-part output (get, version), is shorter than CLI_PART_ROOM, so main.c's UART takes it
 * without a wait (the decode errors and the Ultra lines are in part 1 of `status`; `help` goes out in parts too).
 * Every line once, in order. */
static uint32_t r5b_parts(cli_t *c, const char *cmd, uint32_t *longest, char *all, size_t all_size)
{
    uint32_t n = 0;
    *longest = 0;
    all[0] = 0;
    cli_out[0] = 0;
    cli_exec(c, cmd);
    for (;;) {
        size_t len = strlen(cli_out);
        if (len > *longest) {
            *longest = (uint32_t)len;
        }
        printf("    part %u: %u bytes\n", n, (uint32_t)len);
        strncat(all, cli_out, all_size - strlen(all) - 1);
        cli_out[0] = 0;
        if (!c->st_part) {
            break;
        }
        (void)cli_poll(c);
        n++;
    }
    return n;
}

#define STATUS_PARTS_TEST (11u + MON_EXTRA)   /* (cli.c STATUS_PARTS: with the event log's and
                                                the calibration history's, M2) */

static void test_r5_bounds(void)
{
    static char all[16384];
    static const char *const keyname[] = { "protocol", "preset", "dither", "thresh", "feedback", "stagger", "floor",
        "minpulse", "loss", "smooth", "gamma", "scene", "scenefade", "ilimit", "ical", "softstart" };   /* (cli.c keys) */
    cli_t c;
    cli_status_t st;
    vsense_t vs;
    vdip_t d;
    health_t h;
    pwrev_t pw;
    scene_t sc;
    uint32_t i, parts, longest;
    const char *p;

    printf("console: every part of status / help and every other output under CLI_PART_ROOM\n");
    memset(&c, 0, sizeof c);
    memset(&st, 0, sizeof st);
    settings_default(&cli_set);
    config_rx_init(&cli_rx);
    memset(&test_en, 0, sizeof test_en);
    c.write = cap_write;
    c.apply = cap_apply;
    c.save_now = cap_save;
    c.set = &cli_set;
    {                                                    /* the settings in effect: 3 digits and '*' each */
        static settings_t eff;
        memset(&eff, 0xFF, sizeof eff);
        c.eff = &eff;
    }
    st.retry_n = st.retry_s = 255;
    st.save_why = 2;                                     /* (the longest reason: "waiting for dark") */
    st.cal_fail = ISENSE_CAL_F_SUPPLY;
    c.st = &st;
    st.version = "0.6.123-beta.45 (development build)";
    st.bl_version = 255;
    st.hw_id = 255;
    st.hw_rev = 255;
    st.app_state = 1;
    strcpy(st.bl_build, "20261231");
    st.uptime_s = 0xFFFFFFFFu;
    st.stack_max = 2048;
    st.reset_flags = 0x34000000u;         /* (every flag but power-on: the breadcrumbs' line too, longer) */
    st.bc.magic = BC_MAGIC;                /* (every helper's line at its longest) */
    st.bc.up_s = st.bc.pc = st.bc.cause = 0xFFFFFFFFu;
    st.cpu_pm = st.cpu_peak = 65535;
    st.fl_pend = 31;                       /* (D5) every store pending and rejected, the longest code */
    st.fl_rej = 0x3FFFFFFFu;
    st.fl_errs = 65535;
    st.fl_err = FLASH_ERR_VERIFY;
    st.cap.now = st.cap.avg = st.cap.peak = 65535;   /* (M5) */
    {
        static volatile uint16_t hwd[HW_DIAG_N];
        for (i = 0; i < HW_DIAG_N; i++) {
            hwd[i] = 65535;                /* (D6) */
        }
        st.hwd = hwd;
    }
    {
        static volatile uint32_t ovr[3] = { 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu };
        static evlog_t ev;
        for (i = 0; i < EVLOG_N; i++) {
            ev.e[i].t_s = 0;
            ev.e[i].ma = 65535;
            ev.e[i].type = EVL_CAL;
        }
        st.ovr = ovr;
        st.ev = &ev;
    }
#if MON_EXTRA
    {
        static calhist_t ch;               /* four tests, the widest numbers, the trend at its widest */
        uint32_t k;
        for (i = 0; i < CALHIST_N; i++) {
            ch.e[i].lit_h = (uint16_t)(i ? 65535u : 0u);
            for (k = 0; k < 5u; k++) {
                ch.e[i].ma[k] = (uint16_t)(i ? 65535u : 20000u);
            }
        }
        ch.n = CALHIST_N;
        ch.tests = 65535;
        st.ch = &ch;
    }
#endif
    st.proto_locked = 1;
    st.proto_provisional = 1;
    st.frames = st.decoded = st.config_frames = st.saves = st.stagger_moves = 0xFFFFFFFFu;
    st.fps = 2400;
    for (i = 0; i < 6u; i++) {
        st.dec_err[i] = 0xFFFFFFFFu;
    }
    st.ultra_headers = st.ultra_tail_err = 0xFFFFFFFFu;
    st.steps = 1440;
    st.stagger_o2 = st.stagger_o3 = 65535;
    st.ripple_ma = 65535;
    for (i = 0; i < 5u; i++) {
        st.out[i] = 65535;
        st.i_ch_ma[i] = 65535;
        st.cal.ma[i][0] = st.cal.ma[i][1] = st.cal.ma[i][2] = st.cal.ma[i][3] = 65535;
    }
    st.i_ma = st.i_peak_ma = st.i_zero_ma = 65535;
    st.i_limit_ma = 65535;
    st.i_k_permille = 1000;
    st.i_trips = 0xFFFFFFFFu;
    st.i_tripped = 1;
    st.i_trip_cause = 2;
    st.i_model = 2;
    st.cal.levels = 4;
    st.cal.pct[0] = 25;
    st.cal.pct[1] = 50;
    st.cal.pct[2] = 75;
    st.cal.pct[3] = 100;
    st.cal_count = 65535;
    st.lim_ma = 65534;
    st.lim_src = 3;
    st.lim_need = 2;
    st.lim_conn_ma = 0xFFFFFFFFu;
    st.mcu_temp_c10 = -1234;
    st.vdda_mv = 65535;
    st.mcu_peak_c10 = -1234;
    st.vs_supply_mv = st.vs_supply_min_mv = st.vs_supply_max_mv = 99999;
    st.vs_aux_mv = 99999;
    vsense_init(&vs, 23u * 256u, 3280u);
    memset(&d, 0, sizeof d);
    d.count = 65535;
    d.in_dip = 1;
    d.n_log = VDIP_LOG;
    for (i = 0; i < VDIP_LOG; i++) {
        d.log[i].min_x = 1;
        d.log[i].ref_x = 65535;
        d.log[i].dur_ms = 65535;
        d.log[i].i_ma = 65535;
    }
    d.deep = d.log[0];
    d.zsteps = 0xFFFF;
    d.znum = 0x1FFFFF;
    d.zden = 8;
    st.dip = &d;
    st.dip_vs = &vs;
    st.dip_ref_mv = 99999;
    st.test_mode = 1;
    st.oc_show = 2;
    st.oc_cause = 2;
    st.oc_hold_ms = 0xFFFFFFFFu;
    memset(&pw, 0, sizeof pw);
    pw.in.limiting = 1;
    pw.in.trip_cause = 2;
    pw.in.ch_limiting = 1;
    pw.limit_on = pw.ch_on = 1;
    pw.in.retry_ms = 0xFFFFFFF0u;
    pw.limit_events = pw.ovl_trips = pw.short_trips = 65535;
    pw.limit_age_s = pw.trip_age_s = 0xFFFFFFFEu;
    pw.event_ma = pw.trip_ma = 65535;
    pw.last_cause = 2;
    st.pw = &pw;
    memset(&h, 0, sizeof h);
    h.flags = 0xFF;
    for (i = 0; i < 5u; i++) {
        h.cls[i] = 0x73;
        h.test_dev[i] = h.model_dev[i] = -127;
    }
    h.dark_ma = -32768;
    st.hl = &h;
    memset(&sc, 0, sizeof sc);
    sc.mask = 0x1F;
    sc.fade = 255;
    for (i = 0; i < 5u; i++) {
        sc.v[i] = 65535;
    }
    c.scene = &sc;
    test_en.boot_mwh = 0xFFFFFFFFu;
    for (i = 0; i < 5u; i++) {
        test_en.ch_mwh[i] = 0x7FFFFFFFu;   /* (M3: %A shows int32) */
    }
    test_en.base_wh = 0xFFFF0000u;
    test_en.lt.on_s = test_en.lt.lit_s = test_en.lt.ups = test_en.lt.dips = test_en.lt.writes = 0xFFFFFFFFu;
    test_en.lt.trips[0] = test_en.lt.trips[1] = test_en.lt.updates = 65535;
    test_en.lt.wd = test_en.lt.peak_t = test_en.lt.peak_a = 255;
    test_en.lt.trip = test_en.lt.dip_pct = test_en.lt.dip_cs = test_en.lt.dip_a = test_en.lt.rst = test_en.lt.rst_h = 255;
    test_en.lt.trip_h = 65535;                           /* (the fault memory) */
    st.en = &test_en;

    parts = r5b_parts(&c, "status", &longest, all, sizeof all);
    printf("  status, the longest values: %u bytes in %u parts, the longest %u\n", (uint32_t)strlen(all), parts,
           longest);
    CHECK(parts == STATUS_PARTS_TEST && longest < CLI_PART_ROOM, "status: %u parts, each under %u bytes (longest "
          "%u)", STATUS_PARTS_TEST, CLI_PART_ROOM, longest);
    for (i = 0, p = all; i < 5u; i++) {
        static const char *const order[5] = { "decoded=", "decode_error pulse=", "ultra_tail_errors=",
                                              "config_accepted=", "lifetime=" };
        p = strstr(p, order[i]);
        CHECK(p != NULL && strstr(p + 1, order[i]) == NULL, "status: %s once, in order", order[i]);
        if (!p) {
            break;
        }
    }
    parts = r5b_parts(&c, "help", &longest, all, sizeof all);
    printf("  help: %u bytes in %u parts, the longest %u\n", (uint32_t)strlen(all), parts, longest);
    CHECK(parts == 1u && longest < CLI_PART_ROOM && c.st_part == 0u, "help: 1 part, under %u bytes (longest %u)",
          CLI_PART_ROOM, longest);
    CHECK(strncmp(all, "status | get | ", 15) == 0 && strstr(all + 1, "status | get | ") == NULL, "help: the commands "
          "once, first");
    {                                                    /* the keys, in order, on one line */
        char want[256] = "keys:";
        for (i = 0; i < sizeof keyname / sizeof keyname[0]; i++) {
            strcat(want, " ");
            strcat(want, keyname[i]);
        }
        strcat(want, "\r\n");
        p = strstr(all, want);
        CHECK(p != NULL && strlen(p) == strlen(want), "help: the keys' line at the end (%s)", all);
    }
    run_cli(&c, "?\r");
    CHECK(strstr(cli_out, " softstart\r\n") != NULL, "? = help, through cli_poll");
    for (i = 0; i < 2u; i++) {
        static const char *const one[2] = { "get", "version" };
        parts = r5b_parts(&c, one[i], &longest, all, sizeof all);
        CHECK(parts == 0u && longest < CLI_PART_ROOM, "%s: one output under %u bytes (%u)", one[i], CLI_PART_ROOM,
              longest);
    }
}
