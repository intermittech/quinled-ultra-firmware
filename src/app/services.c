/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* services.c: the periodic services: the adaptive stagger, the power-on effect's start, signal loss, power events, the
 * test jig, the CPU load, the status snapshot.
 * A unit of src/app/main.c (round 12, S10): main.c includes the units in order as one translation
 * unit, so their statics stay static and the image is the single file's; compiled only through main.c. */

/* Adaptive stagger (STAGGER_ADAPTIVE, _ADAPTIVE_SPREAD): at most every 50 ms, when an output moved by more than ~0.4 %, pick the TIM2 /
 * TIM3 offsets with the least LED_+ ripple current for the current outputs (stagger.c, ~0.3 ms) and slew there.
 * Weights: the LED current per channel at 100 % measured by the current sense (once calibrated / learned), else
 * from Ultra's CONFIG, else 3 A (R/G/B) and 5 A (W1/W2). */
static NOINLINE void stagger_adapt_run(void)
{
    uint16_t peak[5];
    stagger_result_t r;
    uint8_t i, changed = st_force;
    st_moved = 0;
    for (i = 0; i < 5u; i++) {
        int32_t dv = (int32_t)out_now[i] - (int32_t)st_seen[i];
        if (dv > 256 || dv < -256) {
            changed = 1;
        }
    }
    if (!changed) {
        return;
    }
    st_ms = ms_now;
    st_force = 0;
    for (i = 0; i < 5u; i++) {
        st_seen[i] = out_now[i];
        peak[i] = cfg_load_ma(i);
        if (isense_model_ok(&is)) {        /* measured (calibrated / learned): the real strips, in every mode; */
            uint32_t on = isense_on_ma(&is, i, out_now[i]);   /* the current while on, from the channel's curve */
            peak[i] = (uint16_t)(on > 65535u ? 65535u : on);
        }
    }
    PERF_DO(PERF_STAGGER, stagger_choose_max(out_now, peak, stagger_gap_q16(steps), st_o2, st_o3,
                       (uint16_t)(stagger_is_spread(eff.stagger) ? STAGGER_SPREAD_OMAX : 0u), &r));
    stat.ripple_ma = r.move ? r.ripple_best : r.ripple_now;
    if (r.move) {
        st_o2 = r.o2;
        st_o3 = r.o3;
        pwm_set_phase(st_o2, st_o3);
        stat.stagger_moves++;
    }
    stat.stagger_o2 = st_o2;
    stat.stagger_o3 = st_o3;
}

/* every ms (the test here, out of the search's big frame; the compare of the outputs only after they changed) */
static void stagger_adapt(void)
{
#if BOARD_RETURN
    if (RC_BUSY()) {
        return;                            /* (~0.5 ms: the window's deadline) */
    }
#endif
    if (stagger_is_adaptive(eff.stagger) && ms_now - st_ms >= 50u && (st_moved || st_force)) {
        stagger_adapt_run();
    }
}

/* the stored power-on effects (fx_mask) start in the engine (power-up, signal loss); returns their channels */
static uint8_t fx_start(void)
{
    uint8_t ch, fx = 0;
    ultra_scene_fx_start(&ul, fx_mask, tick_now);
    for (ch = 0; ch < ULTRA_LIGHTS; ch++) {
        if (fx_mask & (1u << ch)) {
            fx |= ul.light[ch].mask;
        }
    }
    return fx;
}

/* (0.6.9, B10) signal loss "scene": the power-on scene and effects as at power-up, faded in from the outputs now; the
 * host's next slot starts the take-over (per light at its SET, or SCENE_TAKEOVER_MS after it). 0: nothing stored */
static uint8_t loss_scene(void)
{
    if ((scene_want.mask & 0x1Fu) == 0u && fx_mask == 0u) {
        return 0;                          /* (the scene as set: the stored one, or one to be stored at a low moment) */
    }
    fx_ch = fx_start();
    scene_return(&srun, &scene_want, fx_ch, out_now, ms_now);
    ultra_seen = 0;
    return 1;
}

static void signal_loss_check(void)
{
    static const uint16_t off[5] = { 0, 0, 0, 0, 0 };
    uint32_t timeout = LOSS_TIMEOUT_MS;
#if BOARD == BOARD_DP_RX
    if (vdip.planned && ms_now - plan_ms >= PLAN_CUT_MS) {
        vdip.planned = 0;                  /* (0.6.10) still powered: the notice forgotten (a dip it began stays the
                                              planned cut's); round 11: after PLAN_CUT_MS (the outputs back after
                                              PLAN_HOLD_MS) */
    }
    if ((vdip.in_dip == 2u || (vdip.planned && vdip.falling)) &&
        (!dk.active || (int32_t)(dk.until - ms_now) < (int32_t)(PLAN_HOLD_MS / 2u))) {
        dark_hold(&dk, ms_now, PLAN_HOLD_MS);   /* (round 11, A4) the cut under way (the supply 3 % down after the
                                                   notice) or its dip: dark while it lasts (a slow decay outlasts the
                                                   notice's hold); a longer hold stays */
    }
    if (grace && (last_data_ms != grace_ms || ms_now - grace_ms > GRACE_MS)) {
        grace = 0;                         /* (0.6.10) a valid frame first (Ultra or WS): its state, no scene this
                                              power-up; none: the power-on scene and effects, faded in (B10's) */
        if (last_data_ms == grace_ms) {
            outputs_dirty |= loss_scene();
            signal_lost |= eff.signal_loss == 3u;   /* (round 9 phase B) mode 3's loss scene is this one: the 2 s
                                                        loss path doesn't start it (and its effects) again */
        }
    }
#endif
    if (ultra_on) {
        timeout = (uint32_t)ul.cfg[ULTRA_CFG_LOSS_TIMEOUT] * 100u;
        if (timeout < 500u) {
            timeout = 500u;
        }
    }
    if (ms_now - last_data_ms < timeout) {
        return;
    }
    if (eff.signal_loss == 3u && !signal_lost) {
        signal_lost = 1;
        outputs_dirty = loss_scene();      /* (0.6.9, B10) the power-on scene / effect; nothing stored: hold */
    }
    if (eff.signal_loss == 0u || eff.signal_loss == 3u) {
        /* mode 0 = hold (Ultra: effects keep running), like a WS2805. The host's SET applies again when it is back
         * (every ms of the gap: no slot comes meanwhile): its SEQ may have gone round by whole 256s (a streamed
         * effect, 4.4 s at 58 frames a second), and an 8-bit repeat would leave the light in its old state until the
         * next change. An unchanged SET's re-apply shows nothing. */
        uint32_t i;
        for (i = 0; ultra_on && i < ULTRA_LIGHTS; i++) {
            ul.last_seq[i][ULTRA_SET] = 0x100u;
        }
        return;
    }
    if (signal_lost) {
        return;
    }
    signal_lost = 1;
    if (ultra_on) {
        ultra_all_off(&ul, tick_now, eff.signal_loss == 2u ? LOSS_FADE_MS : 0u);
    } else {
        smooth_fade(&sm, off, tick_now, eff.signal_loss == 2u ? LOSS_FADE_MS * SMOOTH_TICKS_PER_MS : 0u);
    }
    outputs_dirty = 1;
}

static uint16_t sat16(int32_t v)
{
    return (uint16_t)(v < 0 ? 0 : v > 65535 ? 65535 : v);
}

/* every ms: the power events (core/pwrev.h) and D9. The state comes from the current sense's limiter and trips, and
 * the Ultra channel limits (CONFIG 16-20). D9 by priority: trip > acknowledge / identify / version / the health code >
 * the TP9 reset gesture / test mode > limit > off (core/ocled.h; the feedback channel shows the acknowledge patterns,
 * outputs_update) */
static void power_service(uint8_t ack_on)
{
    pwrev_in_t in;
    uint8_t lv;
    uint32_t k;
    in.limit_ma = clim.limit_ma;
    in.limit_src = pwrev_src(clim.src);
    in.ch_limiting = (uint8_t)(ultra_on && ul.ch_limited);
    in.trip_cause = is.tripped;            /* ISENSE_TRIP_FAST / _OVL = ULTRA_PW_CAUSE_SHORT / _OVERLOAD */
    in.limiting = (uint8_t)(is.limiting && in.trip_cause == ISENSE_TRIP_NONE);
    in.i_ma = sat16(is.i_ms);
    in.trip_ma = in.trip_cause == ISENSE_TRIP_FAST ? is.peak_ma : in.i_ma;
    in.factor = (uint16_t)(is.k >= 65535u ? 65535u : is.k);
    in.retry_ms = isense_retry_left_ms(&is);
    in.dips = vdip.count;                  /* (0.6.10: the planned cut's isn't counted, core/vdip.h) */
    pwrev_update(&pw, ms_now, &in);
    for (k = 0, lv = pw.ev; lv != 0u; k++, lv >>= 1) {   /* (0.6.5, D2) the events it counted: limit, overload,
                                                             short, dip */
        if (lv & 1u) {
            ev_add(EVL_LIMIT + k, in.trip_ma);   /* (the board current; a short's: its peak = pwrev's) */
        }
    }
    if (pw.ev & (ULTRA_PW_EV_OVERLOAD | ULTRA_PW_EV_SHORT)) {   /* (0.6.9, B2) the fault memory: the last trip */
        uint32_t a = in.trip_ma / 500u;
        en.lt.trip = (uint8_t)((pw.ev & ULTRA_PW_EV_SHORT ? 1u : 2u) | (a > 63u ? 63u : a) << 2);
        en.lt.trip_h = (uint16_t)(en.lt.lit_s / 3600u > 65535u ? 65535u : en.lt.lit_s / 3600u);
    }
    ocled_update(&ocl, ms_now, up_s, in.limiting, in.trip_cause);
    lv = ocled_led(&ocl, ms_now, ack.active, ack_on);
#if BOARD_TESTJIG
    if ((tj.active || tj.rst >= TJ_RST_ARMED) && ocl.show == OCLED_OFF && !ack.active) {
        lv = tj_led(&tj, ms_now);          /* test jig: on, off at each cycle start; the reset gesture armed: fast
                                              (overcurrent / acknowledge win) */
    }
#endif
    if (lv != led_now) {
        led_now = lv;
        led_set(lv);
    }
}

#if BOARD_TESTJIG
/* every ms: the test jig's pad (debounced), the TP9 reset gesture, and in test mode the pattern (every ms) and its
 * current report */
static void testjig_service(void)
{
#if CONSOLE_FULL
    uint8_t ev = tj_pad(&tj, ms_now, tj_pads(&tj, HW_TP6_LOW(), uart_rx_pos(), HW_TP9_LOW()));
#else
    uint8_t ev = tj_pad(&tj, ms_now, tj_pads(&tj, HW_TP6_LOW(), 0u, HW_TP9_LOW()));   /* (no RX: TP6 a GPIO alone) */
#endif
    if (ev & TJ_EV_MODE) {
        say(tj.active ? ZJ(m_tj_on, "test mode: on\r\n") : ZJ(m_tj_off, "test mode: off\r\n"));
        outputs_dirty = 1;
#if !CONSOLE_FULL
        cli.st_part = 0;                   /* (the release console: the stream pauses, a status in progress ends) */
#endif
    }
    if (ev & TJ_EV_RESET) {
        say(ZJ(m_tj_reset, "factory reset (jig pad)\r\n"));
        (void)cli_apply(CMD_FACTORY_RESET, 0xA5u);     /* as the console's `defaults` */
    }
    if (!tj.active) {
        return;
    }
    outputs_dirty = 1;
    tj_rep |= (uint8_t)tj_measure(&tj, ms_now, is.i_ms, is.zero_ma);
    if (tj_rep && uart_room() >= CLI_PART_ROOM) {   /* (0.6.6: when the UART takes it whole; the cycle's values stay
                                                       until the next one, 1-2 s) */
        tj_rep = 0;
        cli_test_report(&cli, tj.ma, tj.fail, tj.cycles, vs_sup.valid ? vs_sup.mv : 0u);
    }
}
#endif

/* CPU load (core/cpuload.h, 0.6.5 D3): the cycles asleep in the idle WFI since cpu_t0 (tick_now then), the busiest
 * second since the last status (permille). Once a second: the second that ended into stat.cpu_pm. */
#define POLL_CYC (144000000u / POLL_HZ)   /* HCLK cycles per poll tick: SysTick's period (hw.c) */
static uint32_t cpu_idle, cpu_t0;

static void cpu_second(void)
{
    uint32_t t = tick_now, pm = cpu_permille(cpu_idle, t - cpu_t0, POLL_CYC);
    cpu_idle = 0;
    cpu_t0 = t;
    stat.cpu_pm = (uint16_t)pm;
    if (pm > cpu_peak) {
        cpu_peak = (uint16_t)pm;
    }
}

static inline __attribute__((always_inline)) void status_snapshot(void)
{
    uint8_t i;
    /* the live line's values (cli_watch) */
    stat.uptime_s = up_s;
    stat.proto_locked = (uint8_t)proto_active(&frx.ps);
    for (i = 0; i < 5u; i++) {
        stat.out[i] = (ultra_on ? uout : sm.cur)[i];
        stat.i_now_ch_ma[i] = (int32_t)isense_ch_ma(&is, i, is.applied[i]);
    }
    stat.i_ma = is.i_avg;
    stat.i_k_permille = (is.k * 1000u) >> 16;
    stat.mcu_temp_c10 = mt.c10;
    stat.vs_supply_mv = vs_sup.valid ? vs_sup.mv : 0u;
#if CONSOLE_FULL || STREAM_STATUS                /* ... and the full status's (the receiver's release console has none) */
    stat.stack_max = stack_used();
    stat.proto_mode = (uint8_t)frx.ps.mode;
    stat.proto_provisional = frx.ps.provisional;
    stat.frames = frames;
    stat.steps = steps;
    stat.ilimit50 = sext.ilimit50 == 0xFFu ? 0u : sext.ilimit50;
    stat.softstart = (uint8_t)(sext.opts & STORE_OPT_SOFT_START);
    {
        stat.i_peak_ma = is.peak_ma;
        stat.i_zero_ma = is.zero_valid ? is.zero_ma : 0;
        stat.i_limit_ma = is.limit_ma;
        stat.i_trips = is.trips;
        stat.i_tripped = (uint8_t)(is.tripped != ISENSE_TRIP_NONE);
        stat.i_trip_cause = is.trip_cause != ISENSE_TRIP_NONE ? is.trip_cause : stat.i_trip_cause;
        stat.i_cal = (uint8_t)(is.cal_step != 0u);
        stat.cal_fail = is.cal_fail;
        stat.retry_n = is.retry_max;
        stat.retry_s = is.retry_s;
        stat.save_why = save_why();
        stat.i_model = (uint8_t)(is.calibrated ? 2u : (isense_model_ok(&is) ? 1u : 0u));
        for (i = 0; i < 5u; i++) {
            stat.i_ch_ma[i] = is.ch_ma[i];
        }
        (void)isense_cal_get(&is, &stat.cal);   /* (levels 0: none) */
        stat.cal_count = cal_cnt;
    }
    stat.lim_ma = clim.limit_ma;
    stat.lim_src = clim.src;
    stat.lim_need = clim.need;
    stat.lim_conn_ma = clim.connected_ma;
    stat.vdda_mv = mt.vdda_mv;
    stat.mcu_peak_c10 = temp_peak;
    stat.mcu_peak_s = temp_peak_s;
    stat.test_mode = (uint8_t)TJ_ACTIVE();
    /* Extrema are consumed only by status_begin, never by watch or keystrokes. */
    stat.vs_aux_mv = vs_aux.valid ? vs_aux.mv : 0u;
    stat.dip_ref_mv = vsense_mv(&vs_sup, vdip.ref_q12 >> 8, 16u);   /* Q12 as 16 pairs */
    stat.oc_show = ocl.show;
    stat.oc_cause = ocl.last_cause;
    stat.oc_limiting = ocl.limiting;
    stat.oc_tripped = (uint8_t)(ocl.trip_cause != 0u);
    stat.oc_since_s = ocl.since_s;
    stat.oc_hold_ms = ocled_hold_left(&ocl, ms_now);
#if MON_EXTRA
    stat.fl_pend = (uint8_t)(settings_dirty | (cal_save_pend != 0u) << 1 |   /* (0.6.5, D5) */
                             (uint32_t)scene_pend << 2 | (fx_pend == 2u || fx_clear_pend) << 4);
#endif
#endif
}

#if CONSOLE_FULL || STREAM_STATUS                /* (0.6.9: the receiver's release console has no status) */
static void status_begin(void)
{
    stat.cpu_peak = cpu_peak;              /* the busiest second since the last status; again from the last one */
    cpu_peak = stat.cpu_pm;
    vsense_report(&vs_sup, &stat.vs_supply_min_mv, &stat.vs_supply_max_mv);
}
#endif
