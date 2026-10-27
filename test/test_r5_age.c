/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* r5age, included by test_soak.c: a `status` in parts while the uptime moves on and an event
 * and a supply dip come between two parts. The status takes its snapshot at the command, but its later parts read the
 * event and dip logs live: their ages must count from the time the part is written (main.c refreshes stat.uptime_s),
 * never from the command's (else an event a second newer than the snapshot shows "49710d 6:28:15 ago"). */
#if !CONSOLE_FULL
static void sc_r5age(void)
{
    uint32_t t;
    boot(0,0,0);run_ms(2995);
    while(!cli.st_part)run_ms(1);
    tx_q=SOAK_TX_CAP;t=up_s;
    while(up_s==t) { tx_q=SOAK_TX_CAP; run_ms(1); }
    ev_add(EVL_LIMIT,1234);vdip.deep.t_s=up_s;vdip.deep.ref_x=2604;
    t=up_s;
    while(cli.st_part)run_ms(5);
    CHECK(con_field('N',evl.head,0)==t,"numeric event timestamp across dump parts");
    CHECK(con_field('v',4,0)==t,"numeric dip timestamp across dump parts");
}
#else
static void sc_r5age(void)
{
    const char *p;
    uint32_t t0;
    printf("soak r5age: an event and a dip between the parts of a status, across a second boundary\n");
    boot(0, 0, 0);
    run_ms(2995);                                        /* 5 ms before the uptime turns 3 s */
    con_out_n = 0;
    con_out[0] = 0;
    con_in_r = con_in_w = 0;
    memcpy(con_in, "status\r", 7);
    con_in_w = 7;
    while (!cli.st_part) {
        run_ms(1);
    }
    t0 = stat.uptime_s;                                  /* the command's snapshot */
    tx_q = 1024u;                                        /* the UART full (other output): the next parts wait ~44 ms,
                                                            past the second, whatever the parts' lengths (a shorter
                                                            banner could let part 6 out before it) */
    while (up_s == t0) {
        run_ms(1);
    }
    CHECK(cli.st_part != 0u && cli.st_part <= 7u + MON_EXTRA, "r5age: the next second before the dips' part (part %u; "
          "the dips' part is 6, after the settings' own, 7 with MON_EXTRA's calibration history)", cli.st_part);
    ev_add(EVL_LIMIT, 1234u);                            /* an event and a dip, newer than the snapshot */
    vdip.deep.t_s = up_s;
    vdip.deep.min_x = 1800;
    vdip.deep.ref_x = 2604;
    vdip.deep.dur_ms = 12;
    vdip.count = 1;
    while (cli.st_part) {
        run_ms(5);
    }
    printf("  snapshot %u s, the event and the dip at %u s\n", t0, up_s);
    p = strstr(con_out, "event 1=");                     /* (the dip: pwrev's event, newest) */
    CHECK(p != NULL && strncmp(p, "event 1=0:00:00 ago, supply dip", 31) == 0, "r5age: the dip event's age (%.40s)",
          p ? p : "(none)");
    p = strstr(con_out, "event 2=");
    CHECK(p != NULL && strncmp(p, "event 2=0:00:00 ago, limit", 26) == 0, "r5age: the limit event's age (%.40s)",
          p ? p : "(none)");
    p = strstr(con_out, "supply_dip_deepest=");
    CHECK(p != NULL && strncmp(p, "supply_dip_deepest=0:00:00 ago", 30) == 0, "r5age: the dip's age (%.40s)",
          p ? p : "(none)");
    CHECK(strstr(con_out, "49710d") == NULL, "r5age: no age from an underflow");
}
#endif
