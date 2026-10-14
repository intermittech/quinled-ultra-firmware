/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* (0.6.5, D2) the event log, included by test_main.c after test_testjig.c (test_pwrev.c's pw_in, test_features.c's
 * console helpers): pwrev_t.ev (the events the update counted just now: a limit event once per event, a trip once,
 * a new supply dip from in->dips), and the console (`status` ends with the ring, newest first: its ages and
 * currents; nothing for an empty log). main.c's side (what feeds it): test_soak.c `d2`. */

static uint8_t d2_upd(pwrev_t *p, uint32_t t, const pwrev_in_t *in)   /* an update, its events */
{
    pwrev_update(p, t, in);
    return p->ev;
}

static void test_d2(void)
{
    pwrev_t p;
    pwrev_in_t in;
    evlog_t l;
    cli_t c;
    cli_status_t st;
    uint32_t t, k, sum;
    uint8_t ev;
    const char *q;

    printf("D2 event log: pwrev's events, the log at the end of status\n");
    pwrev_init(&p, 0);
    in = pw_in(0, 0, 0, 120, 0);
    for (t = 0, sum = 0; t < 1000u; t++) {
        sum |= d2_upd(&p, t, &in);
    }
    CHECK(sum == 0u, "quiet: no events (%02X)", sum);
    in = pw_in(1, 0, 0, 8100, 0);
    ev = d2_upd(&p, 1000, &in);
    CHECK(ev == ULTRA_PW_EV_LIMITED && p.limit_events == 1u, "a limit event starts: reported once (%02X)", ev);
    for (t = 1001, sum = 0; t < 1500u; t++) {
        sum |= d2_upd(&p, t, &in);
    }
    in = pw_in(0, 0, 0, 5900, 0);
    for (; t < 3000u; t++) {
        sum |= d2_upd(&p, t, &in);
    }
    in = pw_in(1, 0, 0, 8300, 0);
    sum |= d2_upd(&p, 3000, &in);
    CHECK(sum == 0u && p.limit_events == 1u, "the same event within the 2 s hold: not again (%02X)", sum);
    in = pw_in(0, 0, 0, 5900, 0);
    for (t = 3001; t < 5100u; t++) {
        (void)d2_upd(&p, t, &in);
    }
    in = pw_in(1, 0, 0, 8200, 0);
    ev = d2_upd(&p, 5100, &in);
    CHECK(ev == ULTRA_PW_EV_LIMITED && p.limit_events == 2u, "after the hold: a new one (%02X)", ev);
    in = pw_in(0, 0, ULTRA_PW_CAUSE_SHORT, 0, 15000);
    ev = d2_upd(&p, 6000, &in);
    CHECK(ev == ULTRA_PW_EV_SHORT, "a short trip (%02X)", ev);
    ev = d2_upd(&p, 6001, &in);
    CHECK(ev == 0u, "the trip holds: not again (%02X)", ev);
    in = pw_in(0, 0, 0, 0, 0);
    (void)d2_upd(&p, 7500, &in);
    in = pw_in(0, 0, ULTRA_PW_CAUSE_OVERLOAD, 9000, 9000);
    ev = d2_upd(&p, 7600, &in);
    CHECK(ev == ULTRA_PW_EV_OVERLOAD, "an overload trip (%02X)", ev);
    in = pw_in(0, 0, 0, 100, 0);
    in.dips = 1;
    ev = d2_upd(&p, 8000, &in);
    CHECK(ev == ULTRA_PW_EV_SUPPLY_DIP, "a new supply dip (%02X)", ev);
    CHECK(d2_upd(&p, 8001, &in) == 0u, "the same dip count: nothing");
    in.dips = 0;                                         /* (the u16 count wrapped) */
    CHECK(d2_upd(&p, 8002, &in) == ULTRA_PW_EV_SUPPLY_DIP, "a count that changed by wrapping: a dip");
    in = pw_in(1, 0, ULTRA_PW_CAUSE_SHORT, 9500, 16000);
    in.dips = 1;
    ev = d2_upd(&p, 20000, &in);
    CHECK(ev == (ULTRA_PW_EV_LIMITED | ULTRA_PW_EV_SHORT | ULTRA_PW_EV_SUPPLY_DIP), "three in one ms (%02X)", ev);

    /* the console: a ring of 10 adds (as main.c ev_add does) */
    memset(&c, 0, sizeof c);
    memset(&st, 0, sizeof st);
    memset(&l, 0, sizeof l);
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
    st.ev = &l;
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "event ") == NULL, "an empty log: no event lines");
    {
        static const uint8_t ty[10] = { EVL_CAL, EVL_LIMIT, EVL_DIP, EVL_SHORT, EVL_OVERLOAD, EVL_CAL, EVL_DIP,
                                        EVL_LIMIT, EVL_SHORT, EVL_OVERLOAD };
        for (k = 0; k < 10u; k++) {
            evl_ev_t *e = &l.e[l.head = (uint8_t)((l.head + 1u) & (EVLOG_N - 1u))];
            e->t_s = 100u * k;
            e->type = ty[k];
            e->ma = (uint16_t)(ty[k] == EVL_SHORT || ty[k] == EVL_OVERLOAD || ty[k] == EVL_LIMIT ? 1000u * k + 230u
                                                                                                : 0u);
        }
    }
    st.uptime_s = 3723u;
    run_cli(&c, "status\r");
    q = strstr(cli_out, "event 1=");
    CHECK(q && strcmp(q, "event 1=0:47:03 ago, overload, 9.23 A\r\n"     /* t 900: 3723 - 900 s */
                         "event 2=0:48:43 ago, short, 8.23 A\r\n"
                         "event 3=0:50:23 ago, limit, 7.23 A\r\n"
                         "event 4=0:52:03 ago, supply dip, 0.00 A\r\n"
                         "event 5=0:53:43 ago, calibration, 0.00 A\r\n"
                         "event 6=0:55:23 ago, overload, 4.23 A\r\n"
                         "event 7=0:57:03 ago, short, 3.23 A\r\n"
                         "event 8=0:58:43 ago, supply dip, 0.00 A\r\n") == 0,
          "status ends with the 8 newest, newest first:\n%s", q ? q : "(none)");
    l.e[l.head].type = EVL_NONE;                         /* (a ring that never filled: stops at an empty entry) */
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "event ") == NULL, "stops at the first empty entry");
}
