/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Supply dips (core/vdip.c), included by test_main.c: 1 ms readings of the supply divider with ADC noise at 12, 24
 * and 48 V; a steady supply and a slow sag never dip; a 10 ms dip to 80 % is one dip with its length, lowest reading
 * and current; a 2 ms one isn't; the hysteresis; a supply that settles lower ends its dip; the log (newest first) and
 * the deepest; the boot hold-off and the ms without a reading; the SUPPLY page and the POWER page latch; the ms
 * counter's wrap (no blind 250 ms after it, a dip across it) and ages in seconds; the supply / cable impedance
 * (test_vdip_z). */
#include "../src/core/vdip.h"

/* the 1 ms pair sum of PA4 for `mv` at the supply (ratio 23, 3.28 V), with +-noise LSB per conversion */
static uint32_t vd_x(uint32_t mv, uint32_t noise)
{
    double code = mv / 23.0 / 3280.0 * 4095.0;
    int32_t a = (int32_t)(code + 0.5) + (noise ? (int32_t)rint_(2u * noise + 1u) - (int32_t)noise : 0);
    int32_t b = (int32_t)(code + 0.5) + (noise ? (int32_t)rint_(2u * noise + 1u) - (int32_t)noise : 0);
    return (uint32_t)(a + b);
}

/* the uptime in s at ms vd_t0 (the ms counter and the uptime as main.c keeps them) */
static uint32_t vd_t0, vd_s0;

/* ms t0..t1 at mv (1 pair a ms, none at every 1000th ms like the temperature's) */
static void vd_run(vdip_t *d, uint32_t t0, uint32_t t1, uint32_t mv, int32_t ma)
{
    uint32_t t;
    for (t = t0; t != t1; t++) {                        /* (across the wrap too) */
        vdip_ms(d, t, vd_s0 + (t - vd_t0) / 1000u, t % 1000u == 999u ? 0u : vd_x(mv, 2), t % 1000u == 999u ? 0u : 1u,
                ma);
    }
}

/* Firmware model and pin-role contract. */
static double vz_pin, vz_v0, vz_r, vz_other;
static uint32_t vz_t, vz_last_steps;
static vsense_t vz_vs;

static double vz_noise(uint32_t noise)                   /* +-noise / 2 LSB, uniform: it dithers the 18.4 mV steps */
{
    return noise ? (rint_(1000u) / 999.0 - 0.5) * noise : 0.0;
}

static void vz_run(vdip_t *d, uint32_t ms, double ma, uint32_t noise)
{
    uint32_t k, x;
    vz_last_steps = d->zsteps;
    for (k = 0; k < ms; k++, vz_t++) {
        double v = vz_v0 - vz_r * (ma + vz_other) / 1000.0, code = vz_pin / 23.0 / 3280.0 * 4095.0;
        vz_pin += (v - vz_pin) * 0.0989;                  /* 1 - e^(-1 / 9.6) */
        x = (uint32_t)(code + vz_noise(noise) + 0.5) + (uint32_t)(code + vz_noise(noise) + 0.5);
        vdip_ms(d, vz_t, vz_t / 1000u, x, 1u, (int32_t)ma);
        vsense_add(&vz_vs, x, 1u);
        vdip_z(d, &vz_vs, (uint32_t)ma);
    }
}

/* boot at v0 behind r (the hold-off, the reference) */
static void vz_boot(vdip_t *d, double v0, double r)
{
    vdip_init(d);
    vsense_init(&vz_vs, 23u * 256u, 3280u);
    vz_v0 = v0;
    vz_r = r;
    vz_other = 0;
    vz_pin = 0;                                           /* (PA4's RC charges at power-up) */
    vz_t = 0;
    vz_run(d, 1000, 300, 0);
}

/* n plateaus alternating lo / hi mA, each `ms` long */
static void vz_steps(vdip_t *d, uint32_t n, uint32_t ms, double lo, double hi, uint32_t noise)
{
    uint32_t k;
    for (k = 0; k < n; k++) {
        vz_run(d, ms, (k & 1u) ? hi : lo, noise);
    }
}

static int vz_near(uint32_t got, double want, double tol)
{
    printf("    impedance: %u mOhm for %.0f (%u steps)\n", got, want, (unsigned)vz_last_steps);
    return got >= want * (1.0 - tol) - 1.0 && got <= want * (1.0 + tol) + 1.0;
}

static void test_vdip_z(void)
{
    vdip_t d;
    cli_t c;
    cli_status_t st;
    uint32_t z, k, s0;
    char want[80];

    printf("supply impedance: load steps, RC, noise, fades, effects, dips, a changed cable, the console\n");
    /* firmware model */
    vz_boot(&d, 24000, 150);
    CHECK(vdip_z_mohm(&d) == 0u && d.zsteps == 0u, "no step yet: 0");
    vz_steps(&d, 4, 300, 500, 4500, 2);
    CHECK(d.zsteps == 3u && vdip_z_mohm(&d) == 0u, "3 steps: 0 (%u)", d.zsteps);
    vz_steps(&d, 60, 300, 500, 4500, 2);
    z = vdip_z_mohm(&d);
    CHECK(d.zsteps == 63u && vz_near(z, 150, 0.03), "24 V, 150 mOhm, 4 A steps: %u mOhm (%u steps)", z,
          d.zsteps);
    /* short plateaus (64 ms, just settled): the RC's residue stays under 3 % */
    vz_steps(&d, 200, 64, 500, 4500, 2);
    z = vdip_z_mohm(&d);
    CHECK(vz_near(z, 150, 0.03), "64 ms plateaus: %u mOhm", z);
    /* firmware model */
    vz_boot(&d, 12000, 20);
    vz_steps(&d, 400, 200, 1000, 2000, 2);
    z = vdip_z_mohm(&d);
    CHECK(vz_near(z, 20, 0.10), "12 V, 20 mOhm, 1 A steps: %u mOhm (%u steps)", z, d.zsteps);
    /* 48 V behind 2 Ohm (a long bus), 0.3 A steps */
    vz_boot(&d, 48000, 2000);
    vz_steps(&d, 100, 200, 300, 600, 2);
    z = vdip_z_mohm(&d);
    CHECK(vz_near(z, 2000, 0.05), "48 V, 2 Ohm, 0.3 A steps: %u mOhm", z);
    /* steps under 250 mA count nothing */
    vz_boot(&d, 24000, 150);
    vz_steps(&d, 100, 200, 300, 540, 2);
    CHECK(d.zsteps == 0u, "240 mA steps: none (%u)", d.zsteps);

    /* an effect, a new level every 10-34 ms: steps only where two levels in a row happen to be within the tolerance
     * (a real plateau of >= 48 ms), so what it shows is still right */
    vz_boot(&d, 24000, 150);
    for (k = 0; k < 4000u; k++) {
        vz_run(&d, 10u + rint_(25), 300.0 + rint_(4000), 2);
    }
    z = vdip_z_mohm(&d);
    CHECK(d.zsteps < 40u && vz_near(z, 150, 0.10), "an effect: %u mOhm from %u steps in 88 s", z, d.zsteps);
    /* a slow fade (0.3 -> 5 A over 10 s): its windows are settled one by one, no step */
    vz_boot(&d, 24000, 150);
    for (k = 0; k < 10000u; k++) {
        vz_run(&d, 1, 300.0 + 0.47 * k, 2);
    }
    CHECK(d.zsteps == 0u, "a 10 s fade: no step (%u)", d.zsteps);
    /* 1 s fades between 0.5 and 4.5 A plateaus: steps, as with the plateaus alone */
    vz_boot(&d, 24000, 150);
    for (k = 0; k < 20u; k++) {
        uint32_t f;
        vz_run(&d, 500, 500, 2);
        for (f = 0; f < 1000u; f++) {
            vz_run(&d, 1, 500.0 + 4.0 * f, 2);
        }
        vz_run(&d, 500, 4500, 2);
        for (f = 0; f < 1000u; f++) {
            vz_run(&d, 1, 4500.0 - 4.0 * f, 2);
        }
    }
    z = vdip_z_mohm(&d);
    CHECK(d.zsteps >= 20u && vz_near(z, 150, 0.03), "1 s fades: %u mOhm (%u steps)", z, d.zsteps);
    /* a fade of 5 s: not a step */
    vz_run(&d, 500, 500, 2);
    s0 = d.zsteps;
    for (k = 0; k < 5000u; k++) {
        vz_run(&d, 1, 500.0 + 0.8 * k, 2);
    }
    vz_run(&d, 500, 4500, 2);
    CHECK(d.zsteps == s0, "a 5 s fade: not a step (%u -> %u)", s0, d.zsteps);

    /* a dip (the supply to 80 % for 30 ms, the current steady) between two plateaus: no step across it */
    vz_boot(&d, 24000, 150);
    vz_steps(&d, 22, 300, 500, 4500, 2);
    z = vdip_z_mohm(&d);
    vz_run(&d, 200, 500, 2);
    s0 = d.zsteps;
    vz_v0 = 19200;
    vz_run(&d, 30, 500, 2);
    vz_v0 = 24000;
    vz_run(&d, 100, 4500, 2);                             /* (the step right after the dip) */
    CHECK(d.count == 1u && d.zsteps == s0, "a dip: counted, no step across it (%u -> %u)", s0, d.zsteps);
    vz_steps(&d, 20, 300, 500, 4500, 2);
    CHECK(vz_near(vdip_z_mohm(&d), (double)z, 0.02), "after the dip: as before the dip");

    /* firmware model */
    vz_r = 300;
    vz_steps(&d, 80, 300, 500, 4500, 2);
    z = vdip_z_mohm(&d);
    CHECK(vz_near(z, 300, 0.05), "150 -> 300 mOhm: %u mOhm after 80 steps", z);

    /* another load on the same cable: unrelated (1 A switching every 700 ms): noise, little bias;
     * switching with ours (+2 A at our steps): biased by the ratio of the currents, (4 + 2) / 4 (the documented limit) */
    vz_boot(&d, 24000, 150);
    for (k = 0; k < 300u; k++) {
        vz_other = ((k * 3u / 7u) & 1u) ? 1000 : 0;
        vz_run(&d, 300, (k & 1u) ? 4500 : 500, 2);
    }
    z = vdip_z_mohm(&d);
    CHECK(vz_near(z, 150, 0.10), "an unrelated load: %u mOhm", z);
    vz_boot(&d, 24000, 150);
    for (k = 0; k < 100u; k++) {
        vz_other = (k & 1u) ? 2000 : 0;
        vz_run(&d, 300, (k & 1u) ? 4500 : 500, 2);
    }
    z = vdip_z_mohm(&d);
    CHECK(vz_near(z, 150.0 * 6.0 / 4.0, 0.05), "a load switching with ours: %u mOhm (biased x 1.5)", z);

    /* the console */
    memset(&c, 0, sizeof c);
    memset(&st, 0, sizeof st);
    settings_default(&cli_set);
    st.version = "test";
    st.steps = 5760;
    st.en = &test_en;
    st.dip = &d;
    st.dip_vs = &vz_vs;
    c.write = cap_write;
    c.apply = cap_apply;
    c.save_now = cap_save;
    c.set = &cli_set;
    c.eff = &cli_set;
    c.st = &st;
    z = vdip_z_mohm(&d);
    sprintf(want, "V)\r\nsupply_impedance=%u mohm (%u steps)\r\nsupply_dips=", z, (uint32_t)d.zsteps);
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, want) != NULL, "status: %s", want);
    vdip_init(&d);
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "supply_impedance=0 mohm (0 steps)\r\nsupply_dips=0, reference 0.00 V\r\n") != NULL,
          "status: supply_impedance before any step");
}

/* (N1) the class of a dip is the hint at its first low reading, not at its
 * confirmation (VDIP_MIN_MS readings later): the notice between the two leaves a real dip real, the hint ending between
 * them leaves a planned one planned; a candidate that comes back up is dropped with its class */
static void vdip_low(vdip_t *d, uint32_t t, uint32_t x)
{
    vdip_ms(d, t, 0, x, 1, 100);
}

static void test_vdip_planned(void)
{
    vdip_t d;
    uint32_t k, t;
    printf("supply dips: the planned cut's class from the first low reading\n");
    for (k = 0; k < 2u; k++) {                           /* 0: the notice after the first low; 1: the hint ends */
        vdip_init(&d);
        d.ref_q12 = 1000u << 12;
        d.planned = (uint8_t)k;
        vdip_low(&d, 100, 750);                          /* the first low reading (75 %) */
        CHECK(!d.in_dip && d.run == 1u, "class %u: the first low reading: a candidate, no dip yet", k);
        d.planned = (uint8_t)!k;
        vdip_low(&d, 101, 750);
        vdip_low(&d, 102, 750);                          /* confirmed (VDIP_MIN_MS) */
        CHECK(d.in_dip == 1u + k && d.count == (k ? 0u : 1u), "class %u: confirmed as at its first low reading "
              "(in_dip %u, count %u)", k, d.in_dip, d.count);
        for (t = 103; t < 106u; t++) {
            vdip_low(&d, t, 1000);
        }
        CHECK(!d.in_dip && d.count == (k ? 0u : 1u) && d.n_log == (k ? 0u : 1u) && (d.deep.ref_x != 0u) == !k,
              "class %u: %s (count %u, log %u)", k, k ? "planned: no trace" : "real: counted and logged", d.count,
              d.n_log);
    }
    vdip_init(&d);                                       /* a planned candidate back up: dropped with its class */
    d.ref_q12 = 1000u << 12;
    d.planned = 1;
    vdip_low(&d, 200, 750);
    vdip_low(&d, 201, 1000);
    d.planned = 0;
    for (t = 202; t < 205u; t++) {
        vdip_low(&d, t, 750);
    }
    CHECK(d.in_dip == 1u && d.count == 1u, "a new candidate after the hint: real (in_dip %u, count %u)", d.in_dip,
          d.count);
    /* a slow decay after the notice (big port caps: 0.08 % of the level a ms, 10 % in 1.25 s): the
     * reference holds from the notice, so the decay is `falling` from 3 % down, a dip from 10 % and the planned cut's;
     * it stays one through the decay after the hint ends (the reference held through it too). Without the hint the
     * reference follows the decay: the same decay is a dip later (the negative control) */
    for (k = 0; k < 2u; k++) {
        uint32_t fall = 0, dip = 0, x;
        vdip_init(&d);
        d.ref_q12 = 1000u << 12;
        d.planned = (uint8_t)!k;
        for (t = 0; t < 8000u; t++) {
            x = 1000u - (t * 8u) / 100u;                 /* 1000 -> 360 over 8 s */
            if (t == 5000u) {
                d.planned = 0;                           /* (main.c: PLAN_CUT_MS) */
            }
            vdip_low(&d, 1000u + t, x);
            fall = fall ? fall : d.falling ? t : 0u;
            dip = dip ? dip : d.in_dip ? t : 0u;
        }
        printf("  slow decay, %s: falling from %u ms, a dip from %u ms (class %u), the reference %u\n",
               k ? "no notice" : "the notice", fall, dip, d.in_dip, d.ref_q12 >> 12);
        if (!k) {
            CHECK(fall >= 385u && fall <= 395u && dip >= 1260u && dip <= 1270u && d.in_dip == 2u && d.count == 0u &&
                  (d.ref_q12 >> 12) == 1000u, "a slow decay after the notice: falling at 3 %%, the planned cut's dip at "
                  "10 %% and to the end, the reference held (fall %u, dip %u, in_dip %u, ref %u)", fall, dip, d.in_dip,
                  d.ref_q12 >> 12);
        } else {
            CHECK(dip > 1300u && d.in_dip == 1u && d.count == 1u && (d.ref_q12 >> 12) < 1000u,
                  "the same decay without the notice: a real dip, later (the reference follows: %u ms)", dip);
        }
    }
}

static void test_vdip(void)
{
    static const uint32_t VS[3] = { 12000u, 24000u, 48000u };
    vsense_t vs;
    vdip_t d;
    const vdip_ev_t *e;
    uint32_t k, t, mv;
    uint8_t pg[24], arg[9];
    pwrev_t pw;
    pwrev_in_t in;

    printf("supply dips: detection, length, hysteresis, log, page, latch\n");
    vsense_init(&vs, 23u * 256u, 3280u);
    for (k = 0; k < 3u; k++) {
        vdip_init(&d);
        vd_run(&d, 0, 250, VS[k] / 3u, 0);                /* charging: before the hold-off, ignored */
        vd_run(&d, 250, 60000, VS[k], 1500);
        CHECK(d.count == 0 && !d.in_dip, "%u mV steady 60 s with noise: %u dips", VS[k], d.count);
        mv = vsense_mv(&vs, d.ref_q12 >> 8, 16u);
        CHECK(mv + 60u >= VS[k] && mv <= VS[k] + 60u, "%u mV: the reference %u mV", VS[k], mv);
        /* 10 ms at 80 %, the current at its lowest 6.2 A */
        vd_run(&d, 60000, 60004, VS[k] * 85u / 100u, 6000);
        vd_run(&d, 60004, 60006, VS[k] * 80u / 100u, 6200);
        vd_run(&d, 60006, 60010, VS[k] * 84u / 100u, 6100);
        vd_run(&d, 60010, 61000, VS[k], 1500);
        e = vdip_event(&d, 0);
        CHECK(d.count == 1 && d.n_log == 1 && e != 0 && e->t_s == 60u && e->dur_ms == 10u && e->i_ma == 6200u &&
              vdip_pct(e) == 80u, "%u mV, 10 ms at 80 %%: %u dips, start %u s, %u ms, %u mA, %u %%", VS[k], d.count,
              e ? e->t_s : 0u, e ? e->dur_ms : 0u, e ? e->i_ma : 0u, e ? vdip_pct(e) : 0u);
        mv = e ? vsense_mv(&vs, e->min_x, 1u) : 0u;
        CHECK(mv + VS[k] / 100u >= VS[k] * 80u / 100u && mv <= VS[k] * 81u / 100u, "%u mV: lowest %u mV", VS[k], mv);
        /* 2 ms: too short */
        vd_run(&d, 61000, 61002, VS[k] * 70u / 100u, 1500);
        vd_run(&d, 61002, 62000, VS[k], 1500);
        CHECK(d.count == 1, "%u mV: a 2 ms dip is none (%u)", VS[k], d.count);
    }

    /* hysteresis: 85 % for 20 ms, then 91 % (between 90 and 93) for 50 ms: one dip of 70 ms */
    vdip_init(&d);
    vd_run(&d, 0, 10000, 24000, 1000);
    vd_run(&d, 10000, 10020, 24000u * 85u / 100u, 1000);
    vd_run(&d, 10020, 10070, 24000u * 91u / 100u, 1000);
    CHECK(d.in_dip && d.count == 1, "hysteresis: still in the dip at 91 %%");
    vd_run(&d, 10070, 11000, 24000u * 95u / 100u, 1000);
    e = vdip_event(&d, 0);
    CHECK(!d.in_dip && d.count == 1 && e && e->dur_ms == 70u, "hysteresis: one dip of %u ms", e ? e->dur_ms : 0u);

    /* a slow sag (-15 % over 10 s): the reference follows, no dip; a step to -15 % that stays: one dip, ending */
    vdip_init(&d);
    vd_run(&d, 0, 10000, 24000, 1000);
    for (t = 0; t < 10000u; t++) {
        vd_run(&d, 10000u + t, 10001u + t, 24000u - 3600u * t / 10000u, 1000);
    }
    vd_run(&d, 20000, 40000, 20400, 1000);
    CHECK(d.count == 0, "slow sag: %u dips", d.count);
    vd_run(&d, 40000, 41000, 20400u * 85u / 100u, 1000);
    CHECK(d.count == 1 && d.in_dip, "a step down: a dip");
    vd_run(&d, 41000, 60000, 20400u * 85u / 100u, 1000);
    e = vdip_event(&d, 0);
    CHECK(!d.in_dip && e && e->dur_ms > 1000u && e->dur_ms < 15000u, "a lower supply ends its dip: %u ms",
          e ? e->dur_ms : 0u);

    /* the log: 6 dips, deeper and deeper but the third the deepest; newest first, 4 kept */
    vdip_init(&d);
    vd_run(&d, 0, 10000, 24000, 1000);
    for (k = 0; k < 6u; k++) {
        uint32_t depth = k == 2u ? 60u : 86u - k;
        vd_run(&d, 10000u + 1000u * k, 10010u + 1000u * k, 24000u * depth / 100u, (int32_t)(1000u + k));
        vd_run(&d, 10010u + 1000u * k, 11000u + 1000u * k, 24000, 1000);
    }
    CHECK(d.count == 6 && d.n_log == VDIP_LOG, "log: %u dips, %u kept", d.count, d.n_log);
    for (k = 0; k < VDIP_LOG; k++) {
        e = vdip_event(&d, k);
        CHECK(e && e->t_s == 10u + (5u - k) && e->i_ma == 1000u + 5u - k, "log %u: start %u s", k, e ? e->t_s : 0u);
    }
    CHECK(vdip_event(&d, VDIP_LOG) == 0, "log: nothing past it");
    CHECK(d.deep.t_s == 12u && vdip_pct(&d.deep) == 60u, "the deepest: %u s, %u %%", d.deep.t_s, vdip_pct(&d.deep));

    /* the SUPPLY page */
    vdip_page(&d, &vs, 30, 24012, 1, pg);
    e = vdip_event(&d, 1);
    CHECK(ultra_get16(pg + ULTRA_SU_DIPS) == 6 && pg[ULTRA_SU_LOGGED] == VDIP_LOG &&
          ultra_get16(pg + ULTRA_SU_NOW_10MV) == 2401 && pg[ULTRA_SU_DEEP_PCT] == 60 &&
          ultra_get24(pg + ULTRA_SU_DEEP_AGE) == 18 && ultra_get24(pg + ULTRA_SU_EV_AGE) == 16 &&
          ultra_get16(pg + ULTRA_SU_EV_MS) == 10 && ultra_get16(pg + ULTRA_SU_EV_MA) == 1004 &&
          pg[ULTRA_SU_EV_PCT] == vdip_pct(e) && ultra_get16(pg + ULTRA_SU_EV_10MV) == vsense_mv(&vs, e->min_x, 1u)/10u,
          "SUPPLY page fields");
    vdip_page(&d, &vs, 30, 24012, 7, pg);
    CHECK(ultra_get24(pg + ULTRA_SU_EV_AGE) == 0xFFFFFFu && ultra_get16(pg + ULTRA_SU_EV_MS) == 0, "SUPPLY page: "
          "an index past the log");

    /* the ms counter wraps (49.7 days): boot 30 s before it (the hold-off counts readings, not ms), a dip 100 ms after
     * it (a compare of now < 250 would be blind there), one across it (its length), ages in s over 100 days */
    vdip_init(&d);
    vd_t0 = 0xFFFF8AD0u;                                  /* 30 s before the wrap, uptime 0 */
    vd_s0 = 0;
    vd_run(&d, vd_t0, vd_t0 + 250u, 8000, 0);
    CHECK(d.ref_q12 == 0u, "wrap: the hold-off holds 250 readings at any ms value");
    vd_run(&d, vd_t0 + 250u, 0xFFFFFFF0u, 24000, 1000);
    CHECK(d.ref_q12 != 0u && d.count == 0, "wrap: the reference started, no dip");
    vd_run(&d, 0xFFFFFFF0u, 8u, 24000u * 80u / 100u, 3000);   /* 24 ms at 80 %, ending 8 ms after the wrap */
    vd_run(&d, 8u, 100u, 24000, 1000);
    e = vdip_event(&d, 0);
    CHECK(d.count == 1 && e && e->dur_ms == 24u && e->t_s == 29u, "wrap: a dip across it, %u ms (24), at %u s (29)",
          e ? e->dur_ms : 0u, e ? e->t_s : 0u);
    vd_run(&d, 100u, 110u, 24000u * 80u / 100u, 3000);    /* 10 ms at 80 %, 100 ms after the wrap */
    vd_run(&d, 110u, 2000u, 24000, 1000);
    e = vdip_event(&d, 0);
    CHECK(d.count == 2 && e && e->dur_ms == 10u, "wrap: a dip 100 ms after it is seen (%u dips)", d.count);
    vdip_page(&d, &vs, 100u * 86400u, 24000, 0, pg);      /* 100 days later: the age in s, not the ms counter's */
    CHECK(ultra_get24(pg + ULTRA_SU_EV_AGE) == 100u * 86400u - e->t_s, "wrap: the age after 100 days (%u s)",
          ultra_get24(pg + ULTRA_SU_EV_AGE));
    vd_t0 = vd_s0 = 0;

    /* the POWER page latch: a new dip latches until the controller acknowledges its count */
    pwrev_init(&pw, 0);
    memset(&in, 0, sizeof in);
    in.limit_ma = 8000;
    in.factor = 65535;
    in.dips = 2;
    pwrev_update(&pw, 1, &in);
    CHECK((pwrev_latched(&pw) & ULTRA_PW_EV_SUPPLY_DIP) && (pwrev_state(&pw) & ULTRA_ST_POWER_EVENT), "dip latched");
    memset(arg, 0, sizeof arg);
    arg[0] = ULTRA_PW_ARG_ACK;
    arg[8] = 1;                                           /* the host read 1 */
    pwrev_ack(&pw, arg);
    CHECK(pwrev_latched(&pw) & ULTRA_PW_EV_SUPPLY_DIP, "dip: acknowledged 1 of 2 stays latched");
    arg[8] = 2;
    pwrev_ack(&pw, arg);
    CHECK(pwrev_latched(&pw) == 0, "dip: acknowledged 2 of 2 clears");
    test_vdip_z();
    test_vdip_planned();
}
