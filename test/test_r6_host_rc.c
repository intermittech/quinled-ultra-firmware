/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Review round 6 (gpt-protocol): focused regressions of the return channel and the power events, reusing the
 * original unit fixtures (test_main.c). test\build_host.cmd runs it plain; test\build_robust.cmd runs
 * `ack-buffer` (the POWER unit test's ACK buffer) with AddressSanitizer. */
#define main original_host_main
#include "test_main.c"
#undef main

static void r6_latches(void)
{
    pwrev_t p;
    pwrev_in_t in = pw_in(0, 0, 0, 100, 0);
    uint8_t a[9] = { ULTRA_PW_ARG_ACK };
    uint32_t n, t = 0;
    pwrev_init(&p, 0);
    in.dips = 1;
    pwrev_update(&p, ++t, &in);
    ultra_put16(a + 7, 1);
    pwrev_ack(&p, a);
    in.dips = 2;
    pwrev_update(&p, ++t, &in);
    ultra_put16(a + 7, 2);
    pwrev_ack(&p, a);
    ultra_put16(a + 7, 1);  /* delayed retry of an older ACK */
    pwrev_ack(&p, a);
    CHECK(pwrev_latched(&p) == 0, "R6: stale ACK must not resurrect an acknowledged event");

    pwrev_init(&p, 0);
    pwrev_ack(&p, a);       /* a saved ACK arriving after the board restarted */
    CHECK(pwrev_latched(&p) == 0, "R6: unmatched ACK must not invent an event after restart");
    in.dips = 1;
    pwrev_update(&p, 1, &in);
    CHECK(pwrev_latched(&p) == ULTRA_PW_EV_SUPPLY_DIP,
          "R6: unmatched ACK must not pre-acknowledge a future event");

    pwrev_init(&p, 0);
    in = pw_in(0, 0, 0, 100, 0);
    /* All three event counts can return to their initial ACK without any ACK being sent.
     * Exercise real updates, not direct writes to the implementation's counters. */
    for (n = 1, t = 0; n <= 65536u; n++) {   /* (0.6.10: limit events and trips in turns of their own: a reduction
                                                within 2 s of updates after a trip is the trip's, no limit event) */
        in.dips = (uint16_t)n;
        in.limiting = 1;
        pwrev_update(&p, ++t, &in);
        in.limiting = 0;
        t += 2001u;
        pwrev_update(&p, t, &in);
    }
    for (n = 1; n <= 65536u; n++) {
        in.trip_cause = ULTRA_PW_CAUSE_SHORT;
        pwrev_update(&p, ++t, &in);
        in.trip_cause = 0;
        t += 2001u;
        pwrev_update(&p, t, &in);
    }
    CHECK(p.limit_events == 0 && p.short_trips == 0 && p.in.dips == 0,
          "R6: all tested counts wrapped");
    CHECK(pwrev_latched(&p) == (ULTRA_PW_EV_LIMITED | ULTRA_PW_EV_SHORT | ULTRA_PW_EV_SUPPLY_DIP),
          "R6: unacknowledged latches survive a full counter cycle (got %u)", pwrev_latched(&p));
    memset(a + 1, 0, 8);
    pwrev_ack(&p, a);
    CHECK(pwrev_latched(&p) == 0, "R6: matching wrapped counts clear the latches");
}

/* (0.6.8, protocol 0.2.4) the poll ISR's latency from its tick to the line (tools/measure_rc.py: the window's end
 * ~1.1 us, the burst's HIGH ~0.6 us, behind the current sense's and the 1 ms tick's work and a slew ISR): a bound */
#define R6_ISR_US 5.0

static void r6_schedule(void)
{
    uint8_t p[32];
    rc_sched_t s;
    static const double hsi_values[] = { 0.978, 1.0, 1.022 };
    unsigned slots, rate, back, sign, h, idx, bad = 0, bad_burst = 0;
    double lo = 1e9, hi = -1e9, b_lo = 1e9, b_hi = -1e9, g_lo = 1e9;
    /* End phase exhaustive at DIN sample resolution; +/-200 ppm is the existing
     * test_rc.c model's rate-estimation bound. No hardware timings are asserted. */
    for (slots = 1; slots <= 16; slots++) for (rate = 0; rate < 3; rate++) {
        rc_poll_slot(p, 1, 0xFF, (uint8_t)slots, 0, (uint8_t)rate, 0, 0);
        for (h = 0; h < 3; h++) for (sign = 0; sign < 2; sign++) for (back = 0; back < 1800; back++) {
            const double error = sign ? 1.0002 : 0.9998;
            const double hsi = hsi_values[h];
            const uint32_t tick = 0xFFFFFFF0u, rq = (uint32_t)(50.0 / hsi * error * 65536.0);
            const double pitch = rc_slot_us((uint8_t)rate), bit = rc_brr(rq, (uint8_t)rate) / (144.0 * hsi);
            double end, w = 1000.0 + pitch * slots;
            rc_schedule(&s, tick, (int32_t)back, rq, p);
            end = ((double)(uint32_t)(s.end_tick - tick) * 50.0 + back / 36.0) / hsi;
            if (end - w < lo) lo = end - w;
            if (end - w + R6_ISR_US > hi) hi = end - w + R6_ISR_US;
            if (end < w || end + R6_ISR_US > w + ULTRA_RW_END_US) bad++;
            /* (0.2.4) each index's burst: HIGH within s_i - EARLY (slot 0: EARLY0) .. s_i + LATE, then the preamble
             * (RC_PREAMBLE_CYC), the lead byte and the answer, LOW again by s_i + pitch - GUARD */
            for (idx = 0; idx < slots; idx++) {
                const double si = 1000.0 + pitch * idx;
                double x, e;
                p[RC_P_INDEX] = (uint8_t)idx;
                rc_schedule(&s, tick, (int32_t)back, rq, p);
                x = ((double)(uint32_t)(s.tx_tick - tick) * 50.0 + back / 36.0) / hsi - si;
                e = x + R6_ISR_US + RC_PREAMBLE_CYC / (144.0 * hsi) + (RC_LEAD_BYTES + RC_ANSWER_BYTES) * 10.0 * bit;
                b_lo = x < b_lo ? x : b_lo;
                b_hi = x + R6_ISR_US > b_hi ? x + R6_ISR_US : b_hi;
                g_lo = pitch - ULTRA_RW_GUARD_US - e < g_lo ? pitch - ULTRA_RW_GUARD_US - e : g_lo;
                if (x < -(double)(idx ? ULTRA_RW_EARLY_US : ULTRA_RW_EARLY0_US) || x + R6_ISR_US > ULTRA_RW_LATE_US ||
                    e > pitch - ULTRA_RW_GUARD_US) {
                    bad_burst++;
                }
            }
            p[RC_P_INDEX] = 0xFF;
        }
    }
    printf("R6 schedule model: end minus W %.6f .. %.6f us (+%.0f us ISR), %u violations\n", lo, hi, R6_ISR_US, bad);
    printf("R6 schedule model: burst HIGH minus s_i %.3f .. %.3f us, guard margin %.3f us (lead byte included), %u "
           "violations\n", b_lo, b_hi, g_lo, bad_burst);
    CHECK(bad == 0, "R6: forward must stay inside W..W+ULTRA_RW_END_US for every legal R/rate/phase");
    CHECK(bad_burst == 0, "0.2.4: every burst inside its slot's EARLY / LATE / GUARD (lead byte included)");
    /* Correct low-rate baud rounding must use the full divider, not round at 1 Mbaud. */
    CHECK(rc_brr((uint32_t)(50.0 / 1.002 * 65536.0), 2) == 577u,
          "R6: 250 kbaud divider at +0.2%% HSI must round to 577 (got %u)",
          rc_brr((uint32_t)(50.0 / 1.002 * 65536.0), 2));
}

static void r6_probe_model(void)
{
    static rcs_t c;
    const uint8_t idx[2] = { 0, 1 };
    double bit, t;
    rcs_random_chain(&c, 2, 2, 0);
    c.b[0].fail = RCS_LOST;
    (void)rcs_window(&c, idx, ULTRA_PAGE_STATUS, 1);
    bit = (double)rc_brr(c.b[1].rate_q16, 0) / (144.0 * c.b[1].hsi);
    t = c.b[1].check_at + 1.0;
    CHECK(rcs_tx_level(&c, &c.b[1], t, bit) == 1, "R6: model must include the HIGH turnaround probe");
    CHECK(rcs_lower_drives(&c, 1, t), "R6: quiet decision cannot disable the driver before it samples the probe");
    CHECK(c.fights[1] != 0, "R6: probe against a forward LOW driver is a real modeled fight");
    printf("R6 probe model: %u conflicting 50-ns steps on failed segment\n", c.fights[1]);
    rcs_free(&c);
}

int main(int argc, char **argv)
{
    if (argc > 1 && !strcmp(argv[1], "ack-buffer")) {
        test_pwrev(); /* run this with MSVC AddressSanitizer */
    } else if (argc > 1 && !strcmp(argv[1], "chain")) {
        test_rc_chain();
    } else if (argc > 1 && !strcmp(argv[1], "console")) {
        test_console_cz();
    } else {
        test_rc_units();
        r6_latches();
        r6_schedule();
        r6_probe_model();
    }
    printf("R6 units: %s (%d failures)\n", g_fail ? "FAILED" : "passed", g_fail);
    return g_fail ? 1 : 0;
}
