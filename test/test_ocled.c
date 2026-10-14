/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The overcurrent signal on D9 (core/ocled.c, included by test_main.c): the limit blink, the trip double flash, the
 * holds after the condition ends, a brief event, the priorities against the acknowledge patterns, the episode counts,
 * the ms counter wrapping, and the console line. */
#include "../src/core/ocled.h"

typedef struct { uint32_t on, rises; uint8_t prev; } oc_count_t;

/* ms [t0, t1) with the conditions given (no acknowledge pattern): D9's on-ms and flashes */
static void oc_run(ocled_t *o, uint32_t t0, uint32_t t1, uint8_t lim, uint8_t trip, oc_count_t *k)
{
    uint32_t t;
    k->on = k->rises = 0;
    for (t = t0; t != t1; t++) {
        uint8_t l;
        ocled_update(o, t, t / 1000u, lim, trip);
        l = ocled_led(o, t, 0, 0);
        k->on += l;
        k->rises += (uint32_t)(l && !k->prev);
        k->prev = l;
    }
}

static void test_ocled(void)
{
    ocled_t o;
    oc_count_t k;
    uint32_t t;
    int bad;

    printf("D9 overcurrent signal: limit blink, trip double flash, holds, priorities\n");
    ocled_init(&o);
    memset(&k, 0, sizeof k);
    oc_run(&o, 0, 5000, 0, 0, &k);
    CHECK(k.on == 0 && o.show == OCLED_OFF, "nothing: D9 stays off (%u ms on)", k.on);

    /* limiting for 2 s: 5 Hz from the first ms, then the 3 s hold (the last on-ms is 6999) */
    ocled_update(&o, 5000, 5u, 1, 0);
    CHECK(o.show == OCLED_LIMIT && ocled_led(&o, 5000, 0, 0) == 1, "the limit blink starts with a flash");
    k.prev = 1;
    oc_run(&o, 5001, 7000, 1, 0, &k);
    CHECK(k.rises == 9 && k.on == 999, "limit: 100 / 100 ms, 5 Hz (%u flashes, %u ms on in 2 s)", k.rises + 1, k.on + 1);
    CHECK(ocled_hold_left(&o, 6999) == 0, "no hold while the limit is on");
    oc_run(&o, 7000, 9999, 0, 0, &k);
    CHECK(o.show == OCLED_LIMIT && k.rises == 15 && k.on == 1500, "limit hold: 3 s more (%u flashes)", k.rises);
    CHECK(ocled_hold_left(&o, 8000) == 2000 - 1, "hold left (%u)", ocled_hold_left(&o, 8000));
    ocled_update(&o, 9999, 9u, 0, 0);
    CHECK(o.show == OCLED_OFF && ocled_led(&o, 9999, 0, 0) == 0 && ocled_hold_left(&o, 9999) == 0,
          "off once the hold has run out (3 s after the last limiting ms)");
    printf("  limit: 100/100 ms, 2 s limiting + 3 s hold = %u flashes\n", 10u + 15u);

    /* a brief limit (1 ms) is still seen: the whole hold */
    oc_run(&o, 20000, 20001, 1, 0, &k);
    oc_run(&o, 20001, 25000, 0, 0, &k);
    CHECK(k.rises == 14 && o.show == OCLED_OFF, "1 ms of limiting: 3 s of blinking (%u flashes)",
          k.rises + 1);

    /* a short trip, 2 s with the outputs off: the double flash, 2 x 100 ms per 800 ms */
    oc_run(&o, 30000, 32000, 0, 1, &k);
    CHECK(o.show == OCLED_TRIP && k.rises == 6 && k.on == 600, "trip: double flash (%u flashes, %u ms on in 2 s)",
          k.rises, k.on);
    CHECK(o.last_cause == 1, "trip, cause short");
    /* the retry comes back scaled (the limiter holds it): the trip pattern wins through its 10 s hold */
    oc_run(&o, 32000, 35000, 1, 0, &k);
    CHECK(o.show == OCLED_TRIP && k.rises == 7, "after the retry: still the trip pattern while limiting (%u)", k.rises);
    CHECK(ocled_hold_left(&o, 35000) == 10000u - (35000u - 31999u), "trip hold left (%u)", ocled_hold_left(&o, 35000));
    oc_run(&o, 35000, 41999, 0, 0, &k);
    CHECK(o.show == OCLED_TRIP, "trip hold: 10 s after the outputs came back");
    ocled_update(&o, 41999, 41u, 0, 0);
    CHECK(o.show == OCLED_OFF, "off after the trip hold (the limit's hold ran out under it)");

    /* limiting outlasts the trip hold: the limit blink takes over, starting with a flash */
    oc_run(&o, 50000, 50500, 0, 2, &k);
    CHECK(o.last_cause == 2, "second trip, overload");
    oc_run(&o, 50500, 60499, 1, 0, &k);
    CHECK(o.show == OCLED_TRIP, "trip hold still on at 60498");
    ocled_update(&o, 60499, 60u, 1, 0);
    CHECK(o.show == OCLED_LIMIT && ocled_led(&o, 60499, 0, 0) == 1 && o.since_s == 60u,
          "the limit blink takes over from the trip pattern with a flash");
    ocled_update(&o, 60599, 60u, 1, 0);
    CHECK(ocled_led(&o, 60599, 0, 0) == 0 && ocled_led(&o, 60699, 0, 0) == 1, "and keeps its 5 Hz phase");

    /* priorities: trip > acknowledge / identify / version > limit > off */
    CHECK(ocled_led(&o, 60499, 1, 0) == 0 && ocled_led(&o, 60599, 1, 1) == 1,
          "an acknowledge pattern drives D9 over the limit blink");
    oc_run(&o, 70000, 70001, 0, 1, &k);
    CHECK(o.show == OCLED_TRIP, "tripped");
    bad = 0;
    for (t = 70001; t < 72000; t++) {
        ocled_update(&o, t, t / 1000u, 0, 1);
        bad += ocled_led(&o, t, 1, 1) != ocled_level(&o, t);
        bad += ocled_led(&o, t, 1, 0) != ocled_level(&o, t);
    }
    CHECK(bad == 0, "the trip pattern drives D9 over an acknowledge pattern (%d ms differ)", bad);
    ocled_init(&o);
    CHECK(ocled_led(&o, 100, 1, 1) == 1 && ocled_led(&o, 100, 1, 0) == 0 && ocled_led(&o, 100, 0, 1) == 0,
          "no overcurrent: the acknowledge pattern as is, else off");

    /* the 1 ms counter wraps (49.7 days): holds across the wrap, and no false hold much later */
    ocled_init(&o);
    oc_run(&o, 0xFFFFFF00u, 0xFFFFFF01u, 1, 0, &k);
    oc_run(&o, 0xFFFFFF01u, 0xFFFFFF00u + 2999u, 0, 0, &k);
    CHECK(o.show == OCLED_LIMIT && k.rises == 14, "hold across the wrap (%u flashes)", k.rises + 1);
    ocled_update(&o, 0xFFFFFF00u + 3000u, (0xFFFFFF00u + 3000u) / 1000u, 0, 0);
    CHECK(o.show == OCLED_OFF, "off after the wrap");
    ocled_update(&o, 0xFFFFFF00u + 0x80000000u, (0xFFFFFF00u + 0x80000000u) / 1000u, 0, 0);
    ocled_update(&o, 0xFFFFFF00u + 0xFFFFFFF0u, (0xFFFFFF00u + 0xFFFFFFF0u) / 1000u, 0, 0);
    CHECK(o.show == OCLED_OFF && ocled_hold_left(&o, 0xFFFFFF00u + 0xFFFFFFF0u) == 0, "no false hold 49 days later");
    printf("  trip: 2 x 100 ms per 800 ms + 10 s hold; trip > acknowledge > limit > off; ms wrap ok\n");

    /* the console line */
    {
        cli_t c;
        cli_status_t st;
        pwrev_t pq;
        memset(&c, 0, sizeof c);
        memset(&st, 0, sizeof st);
        pwrev_init(&pq, 0);
        st.pw = &pq;                                     /* (the power events' lines follow) */
        st.en = &test_en;
        settings_default(&cli_set);
        st.version = "test";
        st.steps = 5760;
        c.write = cap_write;
        c.apply = cap_apply;
        c.save_now = cap_save;
        c.set = &cli_set;
        c.st = &st;
        run_cli(&c, "status\r");
        CHECK(strstr(cli_out, "overcurrent_led=off\r\npower_state=ok\r\n") != NULL, "status: off");
        st.uptime_s = 3;
        st.oc_show = OCLED_TRIP;
        st.oc_cause = 1;
        st.oc_tripped = 1;
        st.oc_since_s = 1;
        run_cli(&c, "status\r");
        CHECK(strstr(cli_out, "overcurrent_led=trip (short), since 0:00:01 (2 s ago), outputs off\r\n"
                              "power_state=") != NULL, "status: tripped");
        st.oc_tripped = 0;
        st.oc_cause = 2;
        st.oc_hold_ms = 7500;
        run_cli(&c, "status\r");
        CHECK(strstr(cli_out, "overcurrent_led=trip (overload), since 0:00:01 (2 s ago), hold 7.5 s\r\n") != NULL,
              "status: trip hold");
        st.oc_show = OCLED_LIMIT;
        st.oc_limiting = 1;
        st.oc_hold_ms = 0;
        run_cli(&c, "status\r");
        CHECK(strstr(cli_out, "overcurrent_led=limit, since 0:00:01 (2 s ago), board limit active\r\n") != NULL,
              "status: limiting");
    }
}
