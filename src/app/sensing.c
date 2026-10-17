/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* sensing.c: every ms / second: the supply voltages, their dips and the energy (vsense_service), the MCU temperature
 * and the health checks (temp_service).
 * A unit of src/app/main.c (round 12, S10): main.c includes the units in order as one translation
 * unit, so their statics stay static and the image is the single file's; compiled only through main.c. */

/* every ms: the injected group's voltage pairs of that ms; the supply dips; the energy and the lifetime record */
static void vsense_service(void)
{
    uint32_t sum2[2], n, i, cut, lit = 0, hi = TJ_ACTIVE() || is.cal_step != 0u || ss.state == SS_RUN;
    hw_vsense_take(sum2, &n);
    vsense_add(&vs_sup, sum2[0], n);
    vsense_add(&vs_aux, sum2[1], n);
    i = vdip.in_dip;
    vdip_ms(&vdip, ms_now, up_s, sum2[0], n, is.i_ms);
    if (i == 1u && !vdip.in_dip) {         /* (0.6.9, B2) the fault memory: a dip ended, the last dip (0.6.10: not
                                              the planned cut's, in_dip 2) */
        const vdip_ev_t *d = vdip_event(&vdip, 0);
        if (d != 0 && d->ref_x != 0u) {
            en.lt.dip_pct = (uint8_t)((uint32_t)d->min_x * 100u / d->ref_x);
            en.lt.dip_cs = (uint8_t)(d->dur_ms / 10u > 255u ? 255u : d->dur_ms / 10u);
            en.lt.dip_a = (uint8_t)(d->i_ma / 100u > 255u ? 255u : d->i_ma / 100u);
        }
    }
    cut = PLAN_CUT(i);
    vdip_z(&vdip, &vs_sup, (uint32_t)is.i_ms);            /* the supply / cable impedance (0.6.5; i_ms >= 0) */
    for (i = 0; i < 5u; i++) {
        uint32_t o = out_now[i];
        lit |= o;
        hi |= o >= FLASH_LOW_LEVEL;
    }
    if (!hi && !ultra_on && sm.dur != 0u) {              /* a WS ramp to a brighter target: the same OR, its test hoisted
                                                            out of the loop (-29 cycles a ms) */
        for (i = 0; i < 5u; i++) {
            hi |= sm.to[i] >= FLASH_LOW_LEVEL;
        }
    }
    if (hi || ((ultra_on || fx_ch) && ultra_rising(&ul, tick_now))) {
        flash_hi_ms = ms_now;                            /* the flash: the outputs are not low (see above) */
    }
    energy_ms(&en, ms_now, vs_sup.mv, is.i_ms, (uint8_t)(lit != 0u));
#if MON_EXTRA
    energy_ch_ms(&en, is.applied, is.i_ms);              /* (0.6.5) energy per channel */
#endif
    n = ENERGY_DUE_NONE;
    if (en.due || vdip.count != en.dips || cut) {        /* (else energy_due answers NONE whatever it may admit, and
                                                            life_admit's flash_may / life_pending reads are saved) */
        n = energy_due(&en, ms_now, vdip.count, life_admit() | (cut ? ENERGY_PLANNED : 0u));
    }
    if (n != ENERGY_DUE_NONE) {
        flash_stall_begin(n == ENERGY_DUE_DIP ? FM_PROG : flash_may(0u, 0u));
        if (flash_stall_end(life_store(&en))) {
            energy_saved(&en, up_s);
        }
    }
}

/* once a second, in the main loop: read the conversion started a second ago, start the next one. Not while a current
 * calibration runs (its 30 ms windows read PA7 alone; the injected group would hold the PA7 results for 84 us). */
#define TEMP_HOT_C10      950               /* (0.6.9, B6) the warning: the die at 95.0 C */
#define TEMP_HOT_HYST_C10 100               /* ... until it is back under 85.0 C */
static uint8_t too_hot;
static uint8_t hl_seen;                    /* (0.6.9, B3) the HEALTH flags a second ago */

static void temp_service(void)
{
    uint32_t ts2 = 0, vref2 = 0;
    if (ms_now - temp_ms < 1000u) {
        return;
    }
    temp_ms = ms_now;
#if MON_EXTRA
    hw_dma_check();                        /* (0.6.5, D6) the DMA's transfer errors, once a second */
#endif
    if (temp_started && !hw_temp_read(&ts2, &vref2)) {
        ts2 = vref2 = 0;                   /* not done: counts as a bad sample */
    }
    {
        isense_cal_t c;
        uint8_t up;
        (void)isense_cal_get(&is, &c);
        health_run(&hl, &is, &c);          /* the running model and the dark current against the stored test */
        up = (uint8_t)(hl.flags & ~hl_seen & (ULTRA_HE_F_DARK | HEALTH_F_CAL));
        hl_seen = hl.flags;
        if (up != 0u) {                    /* (0.6.9, B3) D9's code when either rises: a FET stuck on, a test due */
            ack_start(&ack, ACK_P_FAULT, (up & ULTRA_HE_F_DARK) ? 6u : HEALTH_CODE_CAL_SUGGEST);
        }
    }
    if (temp_started) {
        uint8_t hot;
        mcutemp_sample(&mt, ts2, vref2);
        /* (0.6.9, B6) a warning only (nothing dims): the die over TEMP_HOT_C10 (its absolute offset varies per chip,
         * core/mcutemp.h), back under it by TEMP_HOT_HYST_C10: the event and D9's code once, the HEALTH flag and
         * STATUS's reason meanwhile */
        hot = (uint8_t)(mt.valid && mt.c10 >= TEMP_HOT_C10 - (too_hot ? TEMP_HOT_HYST_C10 : 0));
        if (hot && !too_hot) {
            ev_add(EVL_HOT, 0);
            ack_start(&ack, ACK_P_FAULT, HEALTH_CODE_HOT);
        }
        too_hot = hot;
        hl.flags = (uint8_t)((hl.flags & ~ULTRA_HE_F_HOT) | (hot ? ULTRA_HE_F_HOT : 0u));
        if (mt.valid && mt.c10 > temp_peak) {
            temp_peak = mt.c10;
            temp_peak_s = up_s;
        }
    }
    temp_started = (uint8_t)(is.cal_step == 0u);
    if (temp_started) {
        hw_temp_start();
    }
#if MON_EXTRA
    if (stagger_is_adaptive(eff.stagger) && isense_model_ok(&is)) {
        stagger_cap_second(&stat.cap, stat.ripple_ma, steps);   /* (0.6.5) C10's ripple load: the weights measured */
    }
#endif
}
