/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Supply dips (core/vdip.c), included by test_main.c: 1 ms readings of the supply divider with ADC noise at 12, 24
 * and 48 V; a steady supply and a slow sag never dip; a 10 ms dip to 80 % is one dip with its length, lowest reading
 * and current; a 2 ms one isn't; the hysteresis; a supply that settles lower ends its dip; the log (newest first) and
 * the deepest; the boot hold-off and the ms without a reading; the SUPPLY page and the POWER page latch. */
#include "../src/core/vdip.h"

/* the 1 ms pair sum of PA4 for `mv` at the supply (220 k / 10 k, 3.28 V), with +-noise LSB per conversion */
static uint32_t vd_x(uint32_t mv, uint32_t noise)
{
    double code = mv * 10.0 / 230.0 / 3280.0 * 4095.0;
    int32_t a = (int32_t)(code + 0.5) + (noise ? (int32_t)rint_(2u * noise + 1u) - (int32_t)noise : 0);
    int32_t b = (int32_t)(code + 0.5) + (noise ? (int32_t)rint_(2u * noise + 1u) - (int32_t)noise : 0);
    return (uint32_t)(a + b);
}

/* ms t0..t1 at mv (1 pair a ms, none at every 1000th ms like the temperature's) */
static void vd_run(vdip_t *d, uint32_t t0, uint32_t t1, uint32_t mv, int32_t ma)
{
    uint32_t t;
    for (t = t0; t < t1; t++) {
        vdip_ms(d, t, t % 1000u == 999u ? 0u : vd_x(mv, 2), t % 1000u == 999u ? 0u : 1u, ma);
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
    vsense_init(&vs, 220000u, 10000u, 3280u);
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
        CHECK(d.count == 1 && d.n_log == 1 && e != 0 && e->t_ms == 60000u && e->dur_ms == 10u && e->i_ma == 6200u &&
              vdip_pct(e) == 80u, "%u mV, 10 ms at 80 %%: %u dips, start %u, %u ms, %u mA, %u %%", VS[k], d.count,
              e ? e->t_ms : 0u, e ? e->dur_ms : 0u, e ? e->i_ma : 0u, e ? vdip_pct(e) : 0u);
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
        CHECK(e && e->t_ms == 10000u + 1000u * (5u - k) && e->i_ma == 1000u + 5u - k, "log %u: start %u", k,
              e ? e->t_ms : 0u);
    }
    CHECK(vdip_event(&d, VDIP_LOG) == 0, "log: nothing past it");
    CHECK(d.deep.t_ms == 12000u && vdip_pct(&d.deep) == 60u, "the deepest: %u ms, %u %%", d.deep.t_ms,
          vdip_pct(&d.deep));

    /* the SUPPLY page (protocol 0.2.3) */
    vdip_page(&d, &vs, 30000, 24012, 1, pg);
    e = vdip_event(&d, 1);
    CHECK(ultra_get16(pg + ULTRA_SU_DIPS) == 6 && pg[ULTRA_SU_LOGGED] == VDIP_LOG &&
          ultra_get16(pg + ULTRA_SU_NOW_MV) == 24012 && pg[ULTRA_SU_DEEP_PCT] == 60 &&
          ultra_get24(pg + ULTRA_SU_DEEP_AGE) == 18 && ultra_get24(pg + ULTRA_SU_EV_AGE) == 16 &&
          ultra_get16(pg + ULTRA_SU_EV_MS) == 10 && ultra_get16(pg + ULTRA_SU_EV_MA) == 1004 &&
          pg[ULTRA_SU_EV_PCT] == vdip_pct(e) && ultra_get16(pg + ULTRA_SU_EV_MV) == vsense_mv(&vs, e->min_x, 1u),
          "SUPPLY page fields");
    vdip_page(&d, &vs, 30000, 24012, 7, pg);
    CHECK(ultra_get24(pg + ULTRA_SU_EV_AGE) == 0xFFFFFFu && ultra_get16(pg + ULTRA_SU_EV_MS) == 0, "SUPPLY page: "
          "an index past the log");

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
}
