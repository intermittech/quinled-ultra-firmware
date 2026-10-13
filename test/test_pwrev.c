/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Power events (core/pwrev.c, included by test_main.c): the 2 s holds, limit events (one long overload = one event),
 * trips by cause with the current (a short's peak one ms later), ages in seconds, the acknowledge rule (a latch clears
 * only when its count still matches), the POWER page bytes (big-endian, offsets of ultra_protocol.h 0.2.2), counter
 * and ms wrap, and the console lines. */
#include "../src/core/pwrev.h"

static pwrev_in_t pw_in(uint8_t lim, uint8_t ch, uint8_t trip, uint16_t i_ma, uint16_t trip_ma)
{
    pwrev_in_t in;
    memset(&in, 0, sizeof in);
    in.limiting = lim;
    in.ch_limiting = ch;
    in.trip_cause = trip;
    in.i_ma = i_ma;
    in.trip_ma = trip_ma;
    in.limit_ma = 6000;
    in.limit_src = ULTRA_PW_SRC_USER;
    in.factor = lim ? 53215u : 65535u;              /* 0.812 */
    in.retry_ms = trip ? 1500u : 0u;
    return in;
}

/* ms [t0, t1) with the same inputs */
static void pw_run(pwrev_t *p, uint32_t t0, uint32_t t1, pwrev_in_t in)
{
    uint32_t t;
    for (t = t0; t != t1; t++) {
        pwrev_update(p, t, &in);
    }
}

static void test_pwrev(void)
{
    pwrev_t p;
    uint8_t g[PWREV_PAGE_BYTES], a[7];
    pwrev_in_t in;
    uint32_t i;

    printf("power events: holds, limit events, trips, ages, acknowledge, POWER page\n");
    pwrev_init(&p, 0);
    pw_run(&p, 0, 1000, pw_in(0, 0, 0, 120, 0));
    pwrev_page(&p, g);
    CHECK(pwrev_state(&p) == 0 && g[ULTRA_PW_NOW] == 0 && g[ULTRA_PW_LATCHED] == 0 &&
          ultra_get16(g + ULTRA_PW_LIMIT_EVENTS) == 0 && ultra_get24(g + ULTRA_PW_LIMIT_AGE) == 0xFFFFFFu &&
          ultra_get24(g + ULTRA_PW_TRIP_AGE) == 0xFFFFFFu && ultra_get16(g + ULTRA_PW_FACTOR) == 65535u &&
          g[ULTRA_PW_RETRY_S] == 0, "quiet: nothing set, ages none");
    CHECK(ultra_get16(g + ULTRA_PW_LIMIT_MA) == 6000u && (g[ULTRA_PW_SRC_CAUSE] >> 4) == ULTRA_PW_SRC_USER,
          "the limit in effect and its source");

    /* limiting 500 ms, quiet 1.5 s (inside the 2 s hold), limiting again: one event */
    pw_run(&p, 1000, 1500, pw_in(1, 0, 0, 8100, 0));
    CHECK(p.limit_events == 1 && p.event_ma == 8100 && (pwrev_state(&p) & ULTRA_ST_LIMITING) &&
          (pwrev_state(&p) & ULTRA_ST_POWER_EVENT) && pwrev_latched(&p) == ULTRA_PW_EV_LIMITED,
          "a limit event: counted, its current, LIMITING, latched");
    pw_run(&p, 1500, 3000, pw_in(0, 0, 0, 5900, 0));
    CHECK(pwrev_state(&p) & ULTRA_ST_LIMITING, "LIMITING held 2 s after the last reduction");
    pw_run(&p, 3000, 3200, pw_in(1, 0, 0, 8300, 0));
    CHECK(p.limit_events == 1 && p.event_ma == 8100, "a reduction within 2 s: the same event");
    pw_run(&p, 3200, 5199, pw_in(0, 0, 0, 5900, 0));
    CHECK(pwrev_state(&p) & ULTRA_ST_LIMITING, "held until 2 s after 3199");
    in = pw_in(0, 0, 0, 5900, 0);
    pwrev_update(&p, 5199, &in);
    CHECK(!(pwrev_state(&p) & ULTRA_ST_LIMITING), "LIMITING clears 2 s after the last reduction (3199 + 2000)");
    pw_run(&p, 6000, 6100, pw_in(1, 0, 0, 9000, 0));
    CHECK(p.limit_events == 2 && p.event_ma == 9000 && p.limit_age_s == 0, "after 2 s quiet: a new event");
    pw_run(&p, 6100, 11600, pw_in(0, 0, 0, 5000, 0));
    CHECK(p.limit_age_s == 5 || p.limit_age_s == 6, "age in whole seconds (%u)", p.limit_age_s);

    /* acknowledge: a stale count keeps the latch, the current one clears it, repeats and no-ACK do nothing */
    a[0] = ULTRA_PW_ARG_ACK; ultra_put16(a + 1, 1); ultra_put16(a + 3, 0); ultra_put16(a + 5, 0);
    pwrev_ack(&p, a);
    CHECK(pwrev_latched(&p) == ULTRA_PW_EV_LIMITED, "acknowledged 1 of 2 events: still latched");
    ultra_put16(a + 1, 2);
    pwrev_ack(&p, a);
    CHECK(pwrev_latched(&p) == 0 && !(pwrev_state(&p) & ULTRA_ST_POWER_EVENT), "acknowledged 2 of 2: cleared");
    pwrev_ack(&p, a);
    CHECK(pwrev_latched(&p) == 0, "a repeated acknowledge: no change");
    a[0] = 0;
    ultra_put16(a + 1, 0);
    pwrev_ack(&p, a);
    CHECK(pwrev_latched(&p) == 0 && p.ack_limit == 2, "without the ACK flag: ignored");

    /* a short: counted, TRIPPED, factor 0, retry, the peak one ms later; the overload separately */
    pw_run(&p, 20000, 20001, pw_in(0, 0, ULTRA_PW_CAUSE_SHORT, 0, 9000));
    pw_run(&p, 20001, 20002, pw_in(0, 0, ULTRA_PW_CAUSE_SHORT, 0, 15200));
    pw_run(&p, 20002, 21000, pw_in(0, 0, ULTRA_PW_CAUSE_SHORT, 0, 200));
    pwrev_page(&p, g);
    CHECK(p.short_trips == 1 && p.ovl_trips == 0 && ultra_get16(g + ULTRA_PW_TRIP_MA) == 15200u,
          "short: counted, the peak sample (%u mA)", (unsigned)ultra_get16(g + ULTRA_PW_TRIP_MA));
    CHECK((g[ULTRA_PW_NOW] & ULTRA_ST_TRIPPED) && ultra_get16(g + ULTRA_PW_FACTOR) == 0 && g[ULTRA_PW_RETRY_S] == 2 &&
          (g[ULTRA_PW_SRC_CAUSE] & 0x0Fu) == ULTRA_PW_CAUSE_SHORT && g[ULTRA_PW_LATCHED] == ULTRA_PW_EV_SHORT,
          "tripped: factor 0, retry 2 s (1.5 s rounded up), cause short, latched");
    in = pw_in(0, 0, ULTRA_PW_CAUSE_SHORT, 0, 0);
    in.retry_ms = 0;                                /* the ms before the trip is registered */
    pwrev_update(&p, 21000, &in);
    pwrev_page(&p, g);
    CHECK(g[ULTRA_PW_RETRY_S] == 1, "tripped: the retry shows at least 1 s");
    pw_run(&p, 21001, 24000, pw_in(0, 0, 0, 3000, 0));
    pwrev_page(&p, g);
    CHECK(!(g[ULTRA_PW_NOW] & ULTRA_ST_TRIPPED) && g[ULTRA_PW_RETRY_S] == 0 && ultra_get24(g + ULTRA_PW_TRIP_AGE) >= 2u &&
          ultra_get24(g + ULTRA_PW_TRIP_AGE) <= 4u, "released: TRIPPED off, trip age %u s",
          (unsigned)ultra_get24(g + ULTRA_PW_TRIP_AGE));
    pw_run(&p, 30000, 30300, pw_in(0, 0, ULTRA_PW_CAUSE_OVERLOAD, 0, 10600));
    pw_run(&p, 30300, 30400, pw_in(0, 0, 0, 5000, 0));
    pwrev_page(&p, g);
    CHECK(p.ovl_trips == 1 && ultra_get16(g + ULTRA_PW_OVL_TRIPS) == 1u && ultra_get16(g + ULTRA_PW_TRIP_MA) == 10600u &&
          (g[ULTRA_PW_SRC_CAUSE] & 0x0Fu) == ULTRA_PW_CAUSE_OVERLOAD &&
          g[ULTRA_PW_LATCHED] == (ULTRA_PW_EV_SHORT | ULTRA_PW_EV_OVERLOAD), "overload trip: own counter and latch");

    /* channel limits: their own bit, held 2 s, no event */
    pw_run(&p, 40000, 40100, pw_in(0, 1, 0, 3000, 0));
    pw_run(&p, 40100, 42099, pw_in(0, 0, 0, 3000, 0));
    CHECK((pwrev_state(&p) & ULTRA_ST_CH_LIMITING) && !(pwrev_state(&p) & ULTRA_ST_LIMITING) && p.limit_events == 2,
          "CH_LIMITING held, no limit event");
    pw_run(&p, 42099, 42101, pw_in(0, 0, 0, 3000, 0));
    CHECK(!(pwrev_state(&p) & ULTRA_ST_CH_LIMITING), "CH_LIMITING clears after 2 s");

    /* the page is big-endian at the header's offsets; counters wrap and stay latched */
    p.limit_events = 0x0102u;
    pwrev_page(&p, g);
    CHECK(g[ULTRA_PW_LIMIT_EVENTS] == 0x01 && g[ULTRA_PW_LIMIT_EVENTS + 1] == 0x02, "u16 big-endian");
    p.limit_events = 0xFFFFu;
    p.ack_limit = 0xFFFFu;
    pw_run(&p, 50000, 50001, pw_in(1, 0, 0, 7000, 0));
    CHECK(p.limit_events == 0 && (pwrev_latched(&p) & ULTRA_PW_EV_LIMITED), "the counter wraps, the latch holds");

    /* the ms counter wraps: the hold across it, and none 49 days later */
    pwrev_init(&p, 0xFFFFF000u);
    pw_run(&p, 0xFFFFF000u, 0xFFFFFF00u, pw_in(0, 0, 0, 100, 0));
    pw_run(&p, 0xFFFFFF00u, 0xFFFFFF01u, pw_in(1, 0, 0, 7000, 0));
    pw_run(&p, 0xFFFFFF01u, 0xFFFFFF00u + 1999u, pw_in(0, 0, 0, 100, 0));
    CHECK(pwrev_state(&p) & ULTRA_ST_LIMITING, "hold across the wrap");
    pw_run(&p, 0xFFFFFF00u + 1999u, 0xFFFFFF00u + 2001u, pw_in(0, 0, 0, 100, 0));
    CHECK(!(pwrev_state(&p) & ULTRA_ST_LIMITING), "clears after the wrap");
    in = pw_in(0, 0, 0, 100, 0);
    pwrev_update(&p, 0x7FFFFF00u, &in);
    CHECK(!(pwrev_state(&p) & ULTRA_ST_LIMITING) && p.limit_age_s > 2000000u, "no false hold much later (age %u s)",
          p.limit_age_s);
    for (i = 0; i < 4u; i++) {
        uint8_t m[4] = { CLIMIT_SRC_BOARD, CLIMIT_SRC_USER, CLIMIT_SRC_FUSE, CLIMIT_SRC_NONE };
        uint8_t e[4] = { ULTRA_PW_SRC_BOARD, ULTRA_PW_SRC_USER, ULTRA_PW_SRC_FUSE, ULTRA_PW_SRC_BOARD };
        CHECK(pwrev_src(m[i]) == e[i], "limit source %u -> %u", m[i], e[i]);
    }
    printf("  limit events after 2 s quiet, trips by cause + peak, latches cleared only by the matching count\n");

    /* the console lines (from pwrev_t, as the page has them) */
    {
        cli_t c;
        cli_status_t st;
        memset(&c, 0, sizeof c);
        memset(&st, 0, sizeof st);
        settings_default(&cli_set);
        st.version = "test";
        st.steps = 5760;
        c.write = cap_write;
        c.apply = cap_apply;
        c.save_now = cap_save;
        c.set = &cli_set;
        c.st = &st;
        pwrev_init(&p, 0);
        st.pw = &p;
        run_cli(&c, "status\r");
        CHECK(strstr(cli_out, "power_state=ok\r\npower_events=limit 0, overload trips 0, short trips 0, nothing latched\r\n"
                              "power_last_limit=none\r\npower_last_trip=none\r\n") != NULL, "status: quiet");
        pw_run(&p, 0, 1000, pw_in(0, 0, ULTRA_PW_CAUSE_SHORT, 0, 15200));
        pw_run(&p, 1000, 4000, pw_in(0, 0, 0, 3000, 0));
        pw_run(&p, 4000, 9500, pw_in(1, 0, 0, 8100, 0));
        run_cli(&c, "status\r");
        CHECK(strstr(cli_out, "power_state=limiting (factor 81.2 %)\r\n"
                              "power_events=limit 1, overload trips 0, short trips 1, latched: limit short\r\n"
                              "power_last_limit=5 s ago, 8.10 A\r\npower_last_trip=9 s ago, short, 15.20 A\r\n") != NULL,
              "status: limiting, a short before");
        pw_run(&p, 9500, 9600, pw_in(0, 0, ULTRA_PW_CAUSE_OVERLOAD, 0, 10600));
        run_cli(&c, "status\r");
        CHECK(strstr(cli_out, "power_state=limiting, tripped (retry in 2 s)\r\n") != NULL, "status: tripped");
    }
}
