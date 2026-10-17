/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* rc_answer.c: the return channel's answers (BOARD_RETURN): every page, and arming the window.
 * A unit of src/app/main.c (round 12, S10): main.c includes the units in order as one translation
 * unit, so their statics stay static and the image is the single file's; compiled only through main.c. */

#if BOARD_RETURN
/* ------------------------------------------------------------------------------- the return channel's answers
 * Pages: core/ultra_protocol.h and the protocol wiki, Return channel (every page of 0.2.7). Built from live values
 * when the POLL's frame ends. */
#define RC_FEATURES (ULTRA_FEAT_SCENE | ULTRA_FEAT_DIM_TO_WARM | ULTRA_FEAT_DTW_POINTS | ULTRA_FEAT_CURRENT_SENSE | \
                     ULTRA_FEAT_CALIBRATION | ULTRA_FEAT_DARK_HOLD | (UPD_APP ? ULTRA_FEAT_FW_UPDATE : 0u) | \
                     ULTRA_FEAT_BOOT_PROTOCOL | ULTRA_FEAT_STAGGER_ADAPTIVE | ULTRA_FEAT_MCU_TEMP | \
                     ULTRA_FEAT_VERSION_BLINK | ULTRA_FEAT_POWER_EVENTS | ULTRA_FEAT_SUPPLY_DIPS | ULTRA_FEAT_HEALTH | \
                     ULTRA_FEAT_ENERGY | ULTRA_FEAT_SOFT_START | ULTRA_FEAT_SCENE_EFFECT | ULTRA_FEAT_LIFETIME | \
                     ULTRA_FEAT_DIAG | ULTRA_FEAT_RETRY | ULTRA_FEAT_LOSS_SCENE | ULTRA_FEAT_STANDBY | \
                     ULTRA_FEAT_LIFE_SYNC)
/* 0.2.4 hardware capabilities: answers (0x01), relays the boards behind it (0x02), the bus voltage (0x04), the input
 * current (0x08); no board NTC (0x10). (0.6.8: the proposal's 0x20, a second voltage input, is gone from 0.2.4: no
 * page carries its value) */
#define RC_HW_CAPS  (ULTRA_HWCAP_RETURN | ULTRA_HWCAP_RELAY | ULTRA_HWCAP_VBUS | ULTRA_HWCAP_CURRENT)
#define RC_ST_TEMP_BOARD 11u               /* 0.2.4 STATUS offsets (ULTRA_ST_* has 8, 9, 15) */
#define RC_ST_VBUS       13u
#define RC_ST_CURRENT    17u
#define RC_ST_MEASURED   19u
static uint16_t sat16(int32_t v);

/* CRC-16 over (id, value) of every CONFIG id the host set, in id order (STATUS 21, ACKS 21) */
static uint16_t rc_cfg_crc(void)
{
    uint8_t b[ULTRA_CFG_COUNT * 3u];
    uint32_t id, n = 0;
    for (id = 0; id < ULTRA_CFG_COUNT; id++) {
        if (ul.cfg_valid & (1u << id)) {
            b[n] = (uint8_t)id;
            ultra_put16(b + n + 1u, ul.cfg[id]);
            n += 3u;
        }
    }
    return crc16_ccitt(b, n);
}

static const uint8_t rc_u16[11] = { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 };   /* (rc_pack: u16 fields) */

/* (0.6.9, protocol 0.2.5) STATUS 8 bits 4-7: why the light isn't as the host set it, the first that applies */
static uint32_t light_why(void)
{
    const vdip_ev_t *d = vdip_event(&vdip, 0);
    return is.tripped != ISENSE_TRIP_NONE ? (is.latched ? ULTRA_ST_WHY_RETRIES_OUT : is.tripped == ISENSE_TRIP_FAST ?
                                             ULTRA_ST_WHY_SHORT : ULTRA_ST_WHY_OVERLOAD)
         : TJ_ACTIVE() ? ULTRA_ST_WHY_TEST
         : is.cal_step != 0u ? ULTRA_ST_WHY_CALIBRATING
         : too_hot ? ULTRA_ST_WHY_HOT
         : (vdip.in_dip == 1u || (d != 0 && up_s - d->t_s < ULTRA_ST_DIP_RECENT_S)) ? ULTRA_ST_WHY_SUPPLY
         : signal_lost ? ULTRA_ST_WHY_SIGNAL_LOSS
         : ss.state == SS_RUN ? ULTRA_ST_WHY_SOFT_START
         : dk.active ? ULTRA_ST_WHY_DARK_HOLD
         : ack.active ? ULTRA_ST_WHY_PATTERN : ULTRA_ST_WHY_NONE;
}

/* the page's DATA into d (24 bytes, zero before); returns the PAGE to answer (ULTRA_PAGE_UNSUPPORTED: not here) */
static MEASURED uint8_t rc_page(uint8_t page, const uint8_t *arg, uint8_t *d)
{
    isense_cal_t c;
    uint32_t i, j, v[14];
    switch (page) {
    case ULTRA_PAGE_STATUS: {                            /* (0.2.4: 0-7, 11-14, 17-23; 8, 9, 15 before) */
        static const uint8_t fl[14] = { 3, 1, 3, 1, 1, 2, 2, 2, 2, 2, 1, 1, 2, 1 };
        v[0] = fw_version >> 8;                          /* major, minor, patch (0: a development build) */
        v[1] = stat.hw_id != 0u ? stat.hw_id : BOARD_HW_ID;   /* ULTRA_ST_HW_ID (0.2.7: the whole byte) */
        v[2] = up_s;
        v[3] = stat.reset_flags >> 26;                   /* RSTSCKR 26-31: pin, power, software, IWDG, WWDG, low power */
        v[4] = pwrev_state(&pw) | light_why() << ULTRA_ST_WHY_SHIFT;   /* ULTRA_ST_STATE (0.2.5: the reason) */
        v[5] = (uint16_t)mt.c10;                         /* ULTRA_ST_MCU_TEMP */
        v[6] = ULTRA_ST_NA;                              /* RC_ST_TEMP_BOARD: no sensor */
        v[7] = vs_sup.valid ? vs_sup.mv / 10u : 0xFFFFu; /* RC_ST_VBUS */
        v[8] = mt.vdda_mv;                               /* ULTRA_ST_VDDA */
        v[9] = sat16(is.i_avg);                          /* RC_ST_CURRENT */
        v[10] = (vs_sup.valid ? 0x01u : 0u) | (is.enabled ? 0x02u : 0u) | (is.calibrated ? 0x08u : 0u);
        v[11] = eff.pwm_preset;
        v[12] = rc_cfg_crc();
        v[13] = (frx.ps.mode << 4) | (eff.stagger & 0x0Fu);
        rc_pack(d, v, fl, 14);
        break;
    }
    case ULTRA_PAGE_OUTPUTS:
        for (i = 0; i < 5u; i++) {
            v[i] = out_now[i];
            v[5u + i] = sat16((int32_t)isense_ch_ma(&is, i, is.applied[i]));
        }
        rc_pack(d, v, rc_u16, 10);
        for (i = 0; i < ULTRA_LIGHTS; i++) {
            d[20] |= (uint8_t)((ul.light[i].fx != ULTRA_FX_NONE) << i);
            d[21] |= (uint8_t)((ul.light[i].defined != 0u) << i);
        }
        d[22] = srun.hold;
        d[23] = (uint8_t)((is.limiting ? 0x01u : 0u) | (is.tripped != ISENSE_TRIP_NONE ? 0x02u : 0u) |
                          (ul.ch_limited ? 0x04u : 0u) | (dk.active ? 0x08u : 0u) | (ack.active ? 0x10u : 0u) |
                          (ss.state == SS_RUN ? 0x20u : 0u) | (signal_lost ? 0x40u : 0u) |
                          ((TJ_ACTIVE() || is.cal_step != 0u) ? 0x80u : 0u));
        break;
    case ULTRA_PAGE_ACKS: {
        static const uint8_t def[3] = { ULTRA_LIGHT_DEF, ULTRA_SET, ULTRA_EFFECT };
        uint32_t never = 0, q;
        for (j = 0; j < 3u; j++) {
            uint32_t t = arg[j] != 0u ? arg[j] : def[j];
            for (i = 0; i <= ULTRA_LIGHTS; i++) {      /* lights 0-4, then the board (LIGHT 0xF) */
                q = t < ULTRA_TYPES ? ul.last_seq[i][t] : 0x100u;
                if (q > 0xFFu) {
                    never |= 1u << (3u * i + j);
                    q = 0;
                }
                d[3u * i + j] = (uint8_t)q;
            }
        }
        ultra_put24(d + 18, never);
        ultra_put16(d + 21, rc_cfg_crc());
        break;
    }
    case ULTRA_PAGE_ERRORS:
        for (j = 0, i = 1; i < 6u; i++) {
            j += stat.dec_err[i];
        }
        v[0] = frames;
        v[1] = stat.ultra_headers;
        v[2] = stat.ultra_tail_err;
        v[3] = stat.decoded;
        v[4] = j;
        v[5] = rc_windows;
        v[6] = rc_answers;
        v[7] = rc_missed;
        v[8] = rc_aborts;
        v[9] = stat.saves;
        v[10] = stack_used();
        rc_pack(d, v, rc_u16, 11);
        break;
    case ULTRA_PAGE_IDENTITY:                            /* (0.2.7) two parts: 0 the board, 1 the features */
        if (arg[0] > ULTRA_ID_PART_FEATURES) {
            return ULTRA_PAGE_UNSUPPORTED;
        }
        if (arg[0] == ULTRA_ID_PART_BOARD) {
            static const uint8_t fl[6] = { 4, 1, 1, 1, 1, 4 };
            for (i = 0; i < 12u; i++) {
                d[i] = ((const uint8_t *)HW_UID)[i];   /* the unique id, in address order */
            }
            v[0] = fw_version;                           /* ULTRA_ID_FW_VERSION */
            v[1] = stat.bl_version;
            v[2] = stat.hw_id != 0u ? stat.hw_id : BOARD_HW_ID;
            v[3] = stat.hw_rev;                          /* ULTRA_ID_HW_REV */
            v[4] = 0;                                    /* (reserved) */
            v[5] = RC_HW_CAPS;                           /* ULTRA_ID_CAPS */
            rc_pack(d + 12, v, fl, 6);
        } else {
            ultra_put32(d + ULTRA_IDF_FEATURES, RC_FEATURES);   /* word 0; words 1-5: none yet */
        }
        break;
    case ULTRA_PAGE_CALIBRATION: {
        static const uint8_t fl[13] = { 1, 1, 2, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1 };
        v[0] = isense_cal_get(&is, &c) ? c.levels : 0u;
        v[1] = (is.calibrated ? 2u : isense_model_ok(&is) ? 1u : 0u) | (uint32_t)is.cal_fail << 4;   /* (0.2.5: the
                                                           last test's end) */
        v[2] = cal_cnt;
        v[3] = cal_done_s != 0u ? up_s + 1u - cal_done_s : 0xFFFFFFu;
        if (cal_done_s != 0u && v[3] > 0xFFFFFEu) {
            v[3] = 0xFFFFFEu;                            /* known age: saturate below the NONE sentinel */
        }
        v[4] = (uint16_t)(is.zero_valid ? is.zero_ma : 0);
        for (i = 0; i < 5u; i++) {
            v[5u + i] = sat16(is.ch_ma[i]);              /* CH1-CH5 at 100 % */
        }
        v[10] = clim.connected_ma > 65535u ? 65535u : clim.connected_ma;
        v[11] = clim.limit_ma;
        v[12] = (pwrev_src(clim.src) << 4) | clim.need;
        rc_pack(d, v, fl, 13);
        break;
    }
    case ULTRA_PAGE_CAL_TEST:
        if (arg[0] > 1u) {
            return ULTRA_PAGE_UNSUPPORTED;               /* two parts */
        }
        (void)isense_cal_get(&is, &c);                   /* (all 0 without a test) */
        for (i = 0; i < 12u; i++) {                      /* part 0: CH1-CH3, part 1: CH4-CH5 (8 values) */
            j = arg[0] * 12u + i;
            if (j < ISENSE_CH * ISENSE_LEVELS) {
                ultra_put16(d + 2u * i, c.ma[j / ISENSE_LEVELS][j % ISENSE_LEVELS]);
            }
        }
        if (arg[0] != 0u) {
            ultra_put16(d + 16, (uint16_t)c.zero_ma);
            for (i = 0; i < ISENSE_LEVELS; i++) {
                d[18u + i] = c.pct[i];
            }
            d[22] = c.levels;
        }
        break;
    case ULTRA_PAGE_POWER:
        pwrev_page(&pw, d);
        break;
    case ULTRA_PAGE_SUPPLY:
        vdip_page(&vdip, &vs_sup, up_s, vs_sup.valid ? vs_sup.mv : 0u, arg[0], d);
        break;
    case ULTRA_PAGE_HEALTH:
        health_page(&hl, temp_peak, up_s - temp_peak_s, d);
        break;
    case ULTRA_PAGE_ENERGY:
        energy_page(&en, up_s, vs_sup.mv / 10u * (uint32_t)(is.i_avg > 0 ? is.i_avg : 0) / 100u, d);
        ultra_put16(d + ULTRA_EN_VB, vs_aux.valid ? vs_aux.mv / 10u : 0xFFFFu);   /* (0.2.5) after the buck */
        ultra_put16(d + ULTRA_EN_PEAK_MA, rc_peak_ma);
        rc_peak_ma = 0;
        break;
    case ULTRA_PAGE_LIFE_SYNC:                           /* (0.6.10, protocol 0.2.6, ULTRA_LS_*: merged in rc_arm) */
    case ULTRA_PAGE_LIFETIME: {                          /* (0.2.5, ULTRA_LT_*) the record as it is now */
        static const uint8_t fl[11] = { 3, 3, 3, 2, 2, 3, 1, 1, 1, 2, 3 };
        static const uint8_t fs[9] = { 3, 3, 3, 2, 2, 3, 4, 2, 2 };   /* (LIFE_SYNC: Wh, result + applied, records) */
        const life_rec_t *r = &en.lt;
        v[0] = r->on_s / 360u;                           /* 0.1 h */
        v[1] = r->lit_s / 360u;
        v[2] = r->ups;
        v[3] = r->trips[0];                              /* overload, short */
        v[4] = r->trips[1];
        v[5] = r->dips;
        v[6] = r->peak_t;
        v[7] = r->peak_a;
        v[8] = r->wd;
        v[9] = r->updates;
        v[10] = r->writes;
        if (page == ULTRA_PAGE_LIFE_SYNC) {              /* (bytes 16.. its own: ULTRA_LS_A_*) */
            v[6] = energy_total_wh(&en);
            v[7] = ls_res;                               /* (the result, then the applied fields) */
            v[8] = r->writes;
        }
        rc_pack(d, v, page == ULTRA_PAGE_LIFE_SYNC ? fs : fl, page == ULTRA_PAGE_LIFE_SYNC ? 9u : 11u);
        break;
    }
    case ULTRA_PAGE_DIAG:                                /* (0.2.5, ULTRA_DG_*) two parts */
        if (arg[0] > 1u) {
            return ULTRA_PAGE_UNSUPPORTED;
        }
        if (arg[0] == 0u) {                              /* the running board */
            static const uint8_t fl[13] = { 1, 1, 2, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1 };
            v[0] = stat.reset_flags >> 26;
            v[1] = stat.bc.magic == BC_MAGIC ? stat.bc.cause : 0xFFu;
            v[2] = stat.bc.pc;
            v[3] = stat.bc.up_s > 0xFFFFFFu ? 0xFFFFFFu : stat.bc.up_s;
            v[4] = stat.cpu_pm;
            v[5] = cpu_peak;
            for (i = 0; i < 3u; i++) {
                j = ovr[i];
                v[6u + i] = j > 0xFFFFu ? 0xFFFFu : j;
            }
            j = vdip_z_mohm(&vdip);
            v[9] = j > 0xFFFFu ? 0xFFFFu : j;
            v[10] = flash_err;
            v[11] = settings_dirty | (cal_save_pend != 0u) << 1 | (uint32_t)scene_pend << 2 | (uint32_t)en.due << 3 |
                    (fx_pend == 2u || fx_clear_pend) << 4 | (uint32_t)en.held << 7;   /* (0.6.10, R5: EXHAUSTED) */
            v[12] = save_why();
            rc_pack(d, v, fl, 13);
            d[22] = d[23] = 0xFFu;                       /* the C10 load: not measured (MON_EXTRA 0) */
        } else {                                         /* the fault memory (core/energy.h), the calibration trend */
            static const uint8_t fl[7] = { 1, 2, 1, 1, 1, 1, 1 };
            const life_rec_t *r = &en.lt;
            v[0] = r->trip;
            v[1] = r->trip_h;
            v[2] = r->dip_pct;
            v[3] = r->dip_cs;
            v[4] = r->dip_a;
            v[5] = r->rst;
            v[6] = r->rst_h;
            rc_pack(d, v, fl, 7);
            for (i = 8; i < 13u; i++) {
                d[i] = 0x80u;                            /* the trend: not known (MON_EXTRA 0: no history) */
            }
            d[13] = d[14] = 0xFFu;
        }
        break;
    default:
        return ULTRA_PAGE_UNSUPPORTED;                   /* FW_UPDATE (reserved), pages this firmware doesn't know */
    }
    return page;
}

/* the frame of a POLL ended (main loop, at once: the window's deadline is ULTRA_RW_DIR_MAX_US after t0). d: samples
 * from t0 (the frame's last falling edge) to the end of the tail copy, -1 = none; odd: the copy's end is
 * TAIL_PAST - odd words past the ring position read at end_tick. Takes part only in an RW frame, with the clock's
 * rate measured (unrated, a 2 % HSI error would move slot 15 by 160 us), and while it can still turn around in time. */
static MEASURED void rc_arm(const frx_end_t *e, uint32_t end_tick, int32_t d, uint32_t odd)
{
    const uint8_t *poll = frx.slot;        /* (rc_poll_parse at the capture; again here: a later capture may have
                                              replaced it when this frame end comes that late) */
    uint32_t i;
    uint8_t page, rf;
    rc_pend = 0;
    if (d < 0 || !e->ultra_sync || (e->flags & (ULTRA_HDR_RW | ULTRA_HDR_EXT)) != ULTRA_HDR_RW ||
        rc_poll_parse(poll) != 0) {
        return;                            /* no window (a POLL is valid only in an RW frame; review r6: and not with
                                              EXT, "must understand": a board ignores every slot of such a frame) */
    }
    if (!ul.clk.rated) {
        rc_missed++;
        return;
    }
    rc_schedule(&rc_s, end_tick, d - (int32_t)(TAIL_PAST - odd) * 16, ul.clk.rate_q16, poll);
    rc_send = 0;
    if (poll[RC_P_INDEX] != ULTRA_POLL_RELAY_ONLY) {     /* (relay only: the page and its arguments are ignored) */
        for (i = 0; i < RC_ANSWER_BYTES; i++) {
            rc_ans[i] = 0;
        }
        if (poll[RC_P_PAGE] == ULTRA_PAGE_POWER) {
            pwrev_ack(&pw, poll + RC_P_ARG);             /* the answer shows the state after it */
        }
        if (poll[RC_P_PAGE] == ULTRA_PAGE_LIFE_SYNC) {
            ls_res = (uint16_t)energy_sync(&en, poll + RC_P_ARG, rc_uid16);   /* (0.6.10; the same) */
        }
        page = rc_page(poll[RC_P_PAGE], poll + RC_P_ARG, rc_ans + ULTRA_RSP_DATA);
        rf = (uint8_t)((pwrev_latched(&pw) ? ULTRA_RSP_EVENT : 0u) | (rc_contention ? ULTRA_RSP_CONTENTION : 0u));
        rc_answer(rc_ans, poll, page, rf, rc_uid16);
        rc_send = 1;
    }
    rc_brr_v = rc_brr(ul.clk.rate_q16, poll[RC_P_RATE]);
    __disable_irq();                       /* (review r6, T14) no tick between the late check and ARMED */
    if ((int32_t)(tick_now + 2u - rc_s.late_tick) > 0) {
        __enable_irq();
        rc_missed++;                       /* a long main-loop pass: it would turn around after ULTRA_RW_DIR_MAX_US */
        return;
    }
    BARRIER();                             /* the schedule and the answer before the ISR sees ARMED */
    rc_state = RC_ARMED;
    __enable_irq();
    rc_windows++;
}
#endif
