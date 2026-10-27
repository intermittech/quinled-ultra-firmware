/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* outputs.c: the settings in effect and the outputs: Ultra's CONFIG overlay, the board current sense (the limit, its
 * setup, the calibration, the service), apply_settings, build / build_all, outputs_update.
 * A unit of src/app/main.c: main.c includes the units in order as one translation
 * unit, so their statics stay static and the image is the single file's; compiled only through main.c. */

/* Ultra CONFIG values (host-managed, RAM only) over the stored settings: the byte fields by the settings table (the
 * CONFIG id, the field, its range: config.h settings_desc; one if per field would be ~130 B more), the
 * period / preset pair as code */
static void ultra_overlay(settings_t *e)
{
    const setdesc_t *t = settings_desc + SETD_OVL;
    const uint16_t *c = ul.cfg;
    uint32_t v = ul.cfg_valid, i;
#define HAS(id) ((v & (1u << (id))) != 0u)
    uint32_t period = mcu_pwm_period_10ns(c[ULTRA_CFG_PWM_PERIOD]);
    if (HAS(ULTRA_CFG_PWM_PERIOD) && period) {
        e->pwm_preset = 0xFF;
        e->raw_steps = (uint16_t)period;
    } else if (HAS(ULTRA_CFG_PWM_PRESET)) {
        e->pwm_preset = (uint8_t)c[ULTRA_CFG_PWM_PRESET];
    }
    for (i = 0; i < SETD_OVL_N; i++, t++) {
        uint32_t id = t->cfg, x = c[id];
        if (HAS(id)) {
            ((uint8_t *)e)[t->ofs] = (uint8_t)x;
        }
    }
#undef HAS
}

/* ------------------------------------------------------------------------------- board current sense */
/* the limit in effect: the stored one (config frame / console) and, in Ultra mode, the host's CONFIG 21; the
 * lower of the two that are set (0 = the board rating, isense clamps to it) */
static uint16_t isense_limit_ma(void)
{
    uint32_t l = (sext.ilimit50 == 0xFFu) ? 0u : (uint32_t)sext.ilimit50 * 50u;
    if (ultra_on && (ul.cfg_valid & (1u << ULTRA_CFG_BOARD_MAX)) && ul.cfg[ULTRA_CFG_BOARD_MAX] != 0u) {
        uint32_t u = ul.cfg[ULTRA_CFG_BOARD_MAX];
        l = (l == 0u || u < l) ? u : l;
    }
    return (uint16_t)(l > 65535u ? 65535u : l);
}

/* a channel's LED current at 100 % (stagger weights without the current sense): Ultra CONFIG 11-15 when set, else
 * the default */
static uint16_t cfg_load_ma(uint8_t ch)
{
    static const uint16_t def_ma[5] = { 3000, 3000, 3000, 5000, 5000 };   /* CH_R, CH_G, CH_B, CH_W1, CH_W2 */
    uint8_t id = (uint8_t)(ULTRA_CFG_SLOT_CURRENT + ch);
    return (ultra_on && (ul.cfg_valid & (1u << id)) && ul.cfg[id] != 0u) ? ul.cfg[id] : def_ma[ch];
}

/* The board limit in effect (core/climit.h): the lowest of the board rating, the user's limit (stored ilimit / Ultra
 * CONFIG 21) and the fuse. The current sense's soft limiter holds it (measured); the connected load is the calibrated /
 * learned one. Every apply_settings, limit command, finished calibration and once a second (the learned model
 * drifts). */
static void limit_update(void)
{
    lim_ms = ms_now;
    stat.lim_user_ma = isense_limit_ma();  /* (the console's Y: its one caller keeps it inline) */
    climit_eval(&clim, stat.lim_user_ma, ISENSE_BOARD_MA, ISENSE_FUSE_MA, isense_connected_ma(&is));
    isense_set_limit(&is, clim.limit_ma);
}

/* calibration (config frame, console, Ultra IDENTIFY 2): duty_pct 0 = the standard test (each channel alone at
 * 25 / 50 / 75 / 100 %), 5..100 = each channel alone at that duty only */
/* the current sense from scratch (boot, factory reset): the tick is running, so no half-initialised trip level */
static void isense_setup(void)
{
    uint8_t trip;
    __disable_irq();
    trip = is.tripped;                    /* factory reset: keep a hardware force-off paired with a retry */
    isense_init(&is, 1, ISENSE_FUSE_MA, ISENSE_BOARD_MA);
    is.tripped = trip;
    isense_set_gain(&is, ISENSE_GAIN_DEFAULT_Q16 * 200u / ISENSE_MV_PER_A);   /* the board's sense mV per A */
    __enable_irq();
}

static uint8_t isense_calibrate(uint8_t duty_pct)
{
    isense_cal_t c;
    if (is.tripped != ISENSE_TRIP_NONE || TJ_ACTIVE()) {
        return 0;                          /* (the test jig's pattern keeps the outputs) */
    }
    (void)isense_cal_get(&is, &c);
    health_test_start(&hl, &c);            /* the test before this one (levels 0: none) */
    isense_cal_start(&is, duty_pct == 0u ? 0u : (uint16_t)((uint32_t)duty_pct * 65535u / 100u));
    ack_stop(&ack);                        /* the calibration pattern is the feedback */
    outputs_dirty = 1;
    return 1;
}

/* every ms, before the outputs are rendered: the tick's samples of that ms -> limit, trips, calibration */
static void isense_service(void)
{
    uint32_t sum, n, peak, ev = 0;
    if (vdip.in_dip || PLANNED) {
        ev = isense_cal_abort(&is, ISENSE_CAL_F_SUPPLY);    /* a supply dip: a test's currents aren't the
                                                               load's (none running: nothing); the power-off notice
                                                               too, and a test started meanwhile. Before isense_ms
: its last ms would commit the model */
    }
    __disable_irq();
    isense_take(&is, &sum, &n, &peak);
    __enable_irq();
    ev |= PERF_CALL(PERF_ISENSE, isense_ms(&is, sum, n, peak));
#if BOARD_RETURN
    if (is.i_avg > (int32_t)rc_peak_ma) {
        rc_peak_ma = (uint16_t)(is.i_avg > 65535 ? 65535 : is.i_avg);   /* (ENERGY 17) for the hub: the
                                                                            port's PTCs between its polls */
    }
#endif
    if (ev & ISENSE_EV_TRIP) {
        pwm_force_off(1);                  /* overload the limit could not hold */
    }
    if (ev & ISENSE_EV_RELEASE) {
        __disable_irq();
        if (is.tripped == ISENSE_TRIP_NONE) {
            pwm_force_off(0);              /* retry (not if the tick tripped again meanwhile) */
        }
        __enable_irq();
        ss_retry(&ss, ms_now);             /* with the soft start (offset, ramp) */
    }
    if (ev & ISENSE_EV_REAPPLY) {
        outputs_dirty = 1;
    }
    if (ev & ISENSE_EV_CAL_DONE) {
        isense_cal_t c;
        uint8_t code;
        cal_save_pend = 1;                 /* the test with its curves: its own page, once the content is back */
#if BOARD_RETURN
        cal_done_s = up_s + 1u;
#endif
        limit_update();                    /* the connected load is known now */
        (void)isense_cal_get(&is, &c);
        code = health_test_done(&hl, &c);
        ev_add(EVL_CAL, clim.connected_ma);   /* with the connected load it found */
        if (code != 0u) {
            ack_start(&ack, ACK_P_FAULT, code);   /* status LED: the first abnormal channel, or 6 = current while dark */
        }
    }
    if (ev & ISENSE_EV_LATCH) {
        ev_add(EVL_LATCH, is.trip_cause == ISENSE_TRIP_FAST ? is.peak_ma : (uint32_t)(is.i_ms > 0 ? is.i_ms : 0));
    }                                      /* the retries used up: off until a new command or frame value */
    if (ev & ISENSE_EV_CAL_FAIL) {         /* the test stopped (is.cal_fail: why): the stored test and the
                                              model unchanged; logged with the current then, status LED code 8 */
        ev_add(EVL_CAL_FAIL, (uint32_t)(is.i_ms > 0 ? is.i_ms : 0));
        ack_start(&ack, ACK_P_FAULT, HEALTH_CODE_CAL_FAIL);
    }
}

static void apply_settings(void)
{
    uint32_t s;
    eff = set;
    if (ultra_on) {
        ultra_overlay(&eff);
    }
    s = settings_steps(&eff);
    dopt.mode = (dither_mode_t)eff.dither_mode;
    dopt.threshold16 = dither_threshold16(eff.dither_thresh);
    dopt.floor_policy = (floor_policy_t)eff.dim_floor;
    dopt.floor = dither_floor_counts(eff.min_pulse);
    if (s != steps || eff.stagger != run_stagger) {
        steps = s;
        run_stagger = eff.stagger;
        len = pattern_len_for(steps);
        if (stagger_is_spread(eff.stagger) && len < STAGGER_SPREAD_LEN) {
            len = STAGGER_SPREAD_LEN;            /* spread spectrum: a longer period sequence */
        }
        pwm_start(steps, len, eff.stagger);
        st_o2 = STAGGER_FIXED_O2;            /* pwm_start puts TIM2 / TIM3 at the fixed offsets */
        st_o3 = STAGGER_FIXED_O3;
        st_force = 1;
    }
    limit_update();                          /* CONFIG 21 / 11-15 */
    is.retry_max = (uint8_t)(sext.reserved[2] ^ RETRY_N_X);   /* the stored retry policy */
    is.retry_s = (uint8_t)(sext.reserved[3] ^ RETRY_S_X);
    built_ok = 0;                            /* new period / options: every table again */
    outputs_dirty = 1;
}

static void build(uint8_t ch, uint16_t v, uint8_t phase)
{
    uint8_t i, stride;
    uint16_t *dst = pwm_table(ch, &stride);
    const uint16_t *arr = pwm_spread_arr();
    if (arr != 0) {                        /* spread spectrum: every period its own length */
        dither_build_var(v, steps, arr, len, pattern_len_for(steps), &dopt, phase, dst, stride, pwm_right_aligned(ch));
        return;
    }
    dither_build(v, steps, len, &dopt, phase, dst, stride);
    if (pwm_right_aligned(ch)) {           /* PWM mode 2: active while CNT >= CCR */
        for (i = 0; i < len; i++) {
            dst[(uint32_t)i * stride] = (uint16_t)(steps - dst[(uint32_t)i * stride]);
        }
    }
}

/* the channels' tables for the values v (all, or only the ones whose value changed): phases spread the extra dither
 * steps / skipped pulses of the five channels over the pattern */
static void build_all(const uint16_t *v, uint8_t changed_only)
{
    static const uint8_t order[5] = { CH_G, CH_R, CH_B, CH_W2, CH_W1 };
    uint8_t i;
    for (i = 0; i < 5u; i++) {
        uint8_t ch = order[i];
        if (!changed_only || out_now[ch] != v[ch]) {
            build(ch, v[ch], (uint8_t)(i * len / 5u));
        }
        out_now[ch] = v[ch];
        if (v[ch] >= FLASH_LOW_LEVEL) {
            flash_hi_ms = ms_now;           /* bright from now: a store deciding after this (every
                                               one decides before outputs_update) never sees a stale "low" */
        }
    }
    st_moved = 1;
}

static void outputs_update(uint8_t ack_on)
{
    uint16_t v[5];
    uint8_t i;
#if BOARD_TESTJIG
    if (tj.active) {
        tj_outputs(&tj, ms_now, v);        /* test jig: the fade over the five channels; data doesn't change it */
    } else
#endif
    if (ack.active && !ack.led_only && eff.feedback_ch != 2u) {   /* feedback off: status LED only, the outputs
                                                                    keep rendering */
        uint16_t lvl = ack_on ? 16384u : 0u;          /* 25 % */
        for (i = 0; i < 5u; i++) {
            v[i] = (eff.feedback_ch == 1u || (eff.feedback_ch == 0u && i == CH_R)) ? lvl : 0u;
        }
        if (PLANNED) {
            dark_apply(&dk, ms_now, v);    /* the power-off notice's fade: over a pattern too (status LED goes on) */
        }
    } else {
        const uint16_t *o = ultra_on ? uout : sm.cur;   /* (the test out of the loop) */
        for (i = 0; i < 5u; i++) {
            v[i] = o[i];
        }
        scene_mix(&srun, ms_now, v);       /* the power-on scene until data comes, then the cross-fade */
        dark_apply(&dk, ms_now, v);        /* the pin's calibration: dark (a test still overrides it below) */
    }
    is.k_ss = TJ_ACTIVE() ? 65536u                                    /* soft start (not the test jig's pattern) */
                        : ss_factor(&ss, ms_now, (uint8_t)((v[0] | v[1] | v[2] | v[3] | v[4]) != 0u),
                                    (uint8_t)(sext.opts & STORE_OPT_SOFT_START));
    isense_apply(&is, v, v);               /* board limit (all channels scaled together), or the calibration pattern */
    if (upd.info.bl_version != 0u && upd.info.hw_id != BOARD_HW_ID) {
        v[0] = v[1] = v[2] = v[3] = v[4] = 0;   /* behind another hardware's bootloader (a wrong
                                                   image, SWD): dark, whatever comes */
    }
    PERF_DO(PERF_TABLES, build_all(v, built_ok));
    built_ok = 1;
}

/* pwm_start (hw.h): the tables for the new period / mode, from the values they held, before the timers start */
void pwm_fill(void)
{
    build_all(out_now, 0);
}

/* what the outputs show now, without an acknowledge pattern */
static void shown_outputs(uint16_t *v)
{
    scene_run_t r = srun;
    const uint16_t *o = ultra_on ? uout : sm.cur;
    uint8_t i;
    for (i = 0; i < 5u; i++) {
        v[i] = o[i];
    }
    scene_mix(&r, ms_now, v);
}
