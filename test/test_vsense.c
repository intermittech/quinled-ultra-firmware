/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Voltage channels (core/vsense.c) and the Diff-Power ultra receiver's bus-side power (core/rxpower.c), included by
 * test_main.c: divider scaling against the exact value over every code, full scale without overflow, the 16 ms
 * window, min / max since the last report; the efficiency table (ends, interpolation), the output power estimate,
 * the link limit and the largest inputs. */
#include "../src/core/vsense.h"
#include "../src/core/rxpower.h"

/* mV at the input for n pairs summing to sum2: the exact value */
static double vs_exact(uint32_t rt, uint32_t rb, uint32_t vref, uint32_t sum2, uint32_t n)
{
    return (double)sum2 / n / 2.0 * vref / 4095.0 * (rt + rb) / (rb ? rb : 1u);
}

static void test_vsense(void)
{
    static const struct { uint32_t rt, rb; double tol; const char *name; } DIV[] = {
        { 220000u, 10000u, 3.0, "supply 220k / 10k" },          /* board.h VSENSE_SUPPLY_RT / RB */
        { 120000u, 10000u, 2.0, "receiver V_B 120k / 10k" },
        { 0u, 1u, 0.7, "the pin itself (TP8)" },
    };
    vsense_t v;
    uint32_t d, c, t, mn, mx;
    int bad;

    printf("voltage channels: scaling, window, min / max\n");
    for (d = 0; d < sizeof DIV / sizeof DIV[0]; d++) {
        double worst = 0;
        vsense_init(&v, DIV[d].rt, DIV[d].rb, 3280u);
        CHECK(v.ratio_q8 == ((DIV[d].rt + DIV[d].rb) << 8) / DIV[d].rb, "%s: ratio %u", DIV[d].name, v.ratio_q8);
        /* every pair sum (one pair), then 32 pairs of a code each (a 16 ms window) */
        for (c = 0; c <= 8190u; c++) {
            double e = fabs(vsense_mv(&v, c, 1) - vs_exact(DIV[d].rt, DIV[d].rb, 3280u, c, 1));
            if (e > worst) worst = e;
        }
        for (c = 0; c <= 4095u; c += 7u) {
            double e = fabs(vsense_mv(&v, c * 64u, 32) - vs_exact(DIV[d].rt, DIV[d].rb, 3280u, c * 64u, 32));
            if (e > worst) worst = e;
        }
        CHECK(worst <= DIV[d].tol, "%s: worst error %.2f mV", DIV[d].name, worst);
        /* full scale over a window: 32 pairs of 4095 + 4095 */
        c = vsense_mv(&v, 32u * 8190u, 32);
        CHECK(fabs(c - 3280.0 * (DIV[d].rt + DIV[d].rb) / DIV[d].rb) <= DIV[d].tol, "%s: full scale %u", DIV[d].name, c);
        CHECK(vsense_mv(&v, 1234u, 0) == 0, "%s: no pairs", DIV[d].name);
    }

    /* 48 V on the supply divider: 2087 mV at the pin, code 2605.5 */
    vsense_init(&v, 220000u, 10000u, 3280u);
    c = vsense_mv(&v, 2605u + 2606u, 1);
    CHECK(c >= 47980u && c <= 48020u, "48 V: %u mV", c);

    /* the window: 15 ms don't publish, the 16th does; ms without conversions count as time only */
    vsense_init(&v, 220000u, 10000u, 3280u);
    for (t = 0; t < VSENSE_WIN_MS - 1u; t++) {
        vsense_add(&v, 2u * 5211u, 2);
    }
    CHECK(!v.valid && v.mv == 0, "window: published early");
    vsense_add(&v, 0, 0);
    CHECK(v.valid && v.mv >= 47980u && v.mv <= 48020u, "window: %u mV (valid %u)", v.mv, v.valid);
    for (t = 0; t < VSENSE_WIN_MS; t++) {
        vsense_add(&v, 0, 0);                                   /* a window without any conversion: keeps the last */
    }
    CHECK(v.valid && v.mv >= 47980u && v.mv <= 48020u, "empty window: %u mV", v.mv);

    /* min / max: 48 V, a dip to 40 V, 52 V; the report restarts both at the newest average */
    vsense_init(&v, 220000u, 10000u, 3280u);
    vsense_report(&v, &mn, &mx);
    CHECK(mn == 0 && mx == 0, "report before any measurement: %u / %u", mn, mx);
    bad = 0;
    for (t = 0; t < 3u * VSENSE_WIN_MS; t++) {
        uint32_t code = t < VSENSE_WIN_MS ? 2606u : t < 2u * VSENSE_WIN_MS ? 2171u : 2822u;
        vsense_add(&v, 4u * code, 2);
    }
    vsense_report(&v, &mn, &mx);
    bad += !(mn >= 39980u && mn <= 40020u) || !(mx >= 51980u && mx <= 52040u);
    CHECK(!bad, "min / max: %u / %u", mn, mx);
    vsense_report(&v, &mn, &mx);
    CHECK(mn == v.mv && mx == v.mv, "after the report: %u / %u (mv %u)", mn, mx, v.mv);
}

static void test_rxpower(void)
{
    const rxp_eff_t *e = &RXP_EFF_DEFAULT;
    rxp_t r;
    uint32_t i, p;
    int bad = 0;

    printf("Diff-Power ultra receiver: bus-side power, efficiency table, link limit\n");
    CHECK(rxp_eff(e, 0) == e->eff_permille[0], "below the table: %u", rxp_eff(e, 0));
    CHECK(rxp_eff(e, 500000u) == e->eff_permille[RXP_EFF_POINTS - 1u], "above the table: %u", rxp_eff(e, 500000u));
    for (i = 0; i < RXP_EFF_POINTS; i++) {
        bad += rxp_eff(e, e->pin_mw[i]) != e->eff_permille[i];
    }
    CHECK(!bad, "table points");
    CHECK(rxp_eff(e, 10000u) == 840u, "between 5 and 15 W: %u", rxp_eff(e, 10000u));
    /* between the points: within the two neighbours */
    bad = 0;
    for (p = 0; p <= 120000u; p += 37u) {
        uint16_t f = rxp_eff(e, p);
        bad += f < 800u || f > 930u;
    }
    CHECK(!bad, "interpolation out of range");

    rxp_update(&r, e, 55000u, 2000u, 24000u);
    CHECK(r.pin_mw == 110000u && r.eff_permille == 920u && r.pout_mw == 101200u, "55 V 2 A: %u mW in, %u permille, %u mW out",
          r.pin_mw, r.eff_permille, r.pout_mw);
    CHECK(r.vb_mv == 24000u && !r.over_link && r.link_permille == 888u, "55 V 2 A: link %u", r.link_permille);
    rxp_update(&r, e, 55000u, RXP_LINK_LIMIT_MA, 24000u);
    CHECK(r.link_permille == 1000u && !r.over_link, "at the link limit: %u %u", r.link_permille, r.over_link);
    rxp_update(&r, e, 55000u, RXP_LINK_LIMIT_MA + 1u, 24000u);
    CHECK(r.over_link, "over the link limit");
    rxp_update(&r, e, 54000u, 185u, 12000u);                    /* ~10 W */
    CHECK(r.pin_mw == 9990u && r.eff_permille == 839u, "10 W: %u mW, %u permille", r.pin_mw, r.eff_permille);
    rxp_update(&r, e, 65535u, 65535u, 65535u);                  /* the largest inputs: no overflow */
    CHECK(r.pin_mw == 6553u * 65535u / 100u && r.link_permille == 65535u * 1000u / RXP_LINK_LIMIT_MA && r.over_link,
          "largest inputs: %u mW, link %u", r.pin_mw, r.link_permille);
    CHECK(r.pout_mw == r.pin_mw / 10u * r.eff_permille / 100u, "largest inputs: out %u", r.pout_mw);
}
