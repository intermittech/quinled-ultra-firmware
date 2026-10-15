/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The return channel on the whole firmware (BOARD=2, the Diff-Power ultra receiver; included by test_soak.c, built as
 * soak_rc.exe): src/app/main.c answers RW frames on the soak's hardware model.
 *
 * The model: hal/hw.c's return-channel calls are recorded with their time (rc_hw_dir: turn around / quiet / forward;
 * rc_hw_send: the turnaround check (n 0) and the burst, with the bytes and the BRR); a test can make the line read
 * LOW (contention) at the next check or burst. The hub sends an RW frame with this receiver's POLL as its first slot
 * (it sits at chain position k: N - k slots) at a random tick phase; its clock runs 0.6 % fast (test_soak.c), so the
 * receiver's rate scaling is exercised. The window's times are checked in host us from the frame's last falling edge.
 *
 * Scenarios (soak_rc.exe <name>):
 *   rcpages  every page from this receiver, the answer framing (SYNC, PAGE, INDEX, SEQ, UID16, CRC) and each page's
 *            fields against the firmware's own state; unknown / reserved pages answer UNSUPPORTED; ACKS against the
 *            SEQs the hub sent; IDENTITY's features and capabilities; the timing of every window (turn around
 *            250-900 us, the check 900-977 us, line HIGH at s_i -20 / +80 us, forward again W..W+64 us); traffic on
 *            DIN during the window is not a frame (no capture, no frame end, the gate held closed and re-armed once at
 *            the end), and the next frame is decoded as usual
 *   rcpower  power events: a short (trip), a limit event (CONFIG 21 low), a supply dip; POWER's latches, RFLAGS EVENT
 *            on every page and STATUS 8 POWER_EVENT while one is latched; the acknowledge with the counts read; an
 *            event after the read stays latched; a repeated or 0.2.2 acknowledge (dips 0) does no harm
 *   rcfail   no window: a POLL in a frame without RW, the clock not rated yet (just locked), a late main loop
 *            (the frame end handled after ULTRA_RW_DIR_MAX_US), a POLL with a CRC error; relay only (turns around,
 *            sends nothing); contention at the turnaround check (quiet, no answer) and at the burst, CONTENTION in the
 *            next answer; 500 / 250 kbaud (pitch, BRR); the chain positions 0-3 (1-4 slots); a window lost to a flash
 *            write right before its frame, retried; no flash operation while a window runs (a settings write and a
 *            supply dip's lifetime write wait for its end) */
#if BOARD_RETURN

/* ---- hal/hw.h: the return channel, recorded */
typedef struct {
    uint64_t turn_ns, check_ns, quiet_ns, tx_ns, fwd_ns;
    uint32_t turns, checks, quiets, sends, fwds;
    uint8_t gate_at_turn;                                /* the gate was forced closed before the turn */
    uint16_t brr;
    uint8_t buf[32];
} rc_rec_t;
static rc_rec_t rcr;
static int rc_contend_check, rc_contend_tx;              /* a test: the line reads LOW at the next check / burst */
static uint32_t rcx = 0x2545F491u;
static uint32_t rnd32(void)
{
    rcx ^= rcx << 13;
    rcx ^= rcx >> 17;
    rcx ^= rcx << 5;
    return rcx;
}

void rc_hw_dir(uint32_t mode)
{
    if (mode == RC_DIR_ANSWER) {
        rcr.turn_ns = sim_ns;
        rcr.turns++;
        rcr.gate_at_turn = sim_gate_forced;
    } else if (mode == RC_DIR_QUIET) {
        rcr.quiet_ns = sim_ns;
        rcr.quiets++;
    } else {
        rcr.fwd_ns = sim_ns;
        rcr.fwds++;
    }
}

uint32_t rc_hw_send(const uint8_t *b, uint32_t n, uint16_t brr)
{
    if (n == 0u) {
        rcr.check_ns = sim_ns;
        rcr.checks++;
        if (rc_contend_check) {
            rc_contend_check--;
            return 1;
        }
        return 0;
    }
    if (rc_contend_tx) {
        rc_contend_tx--;
        return 1;
    }
    rcr.tx_ns = sim_ns;
    rcr.sends++;
    rcr.brr = brr;
    memcpy(rcr.buf, b, 32);
    return 0;
}

/* the unique id in address order (test_soak.c's soak_uid, hal/hw.h HW_UID) */
#define rc_uid ((const uint8_t *)soak_uid)

/* ---- the hub */
static uint8_t rc_seq = 0x40;
static uint32_t rc_phase_ns = 7000u;                     /* the frame's start after a tick */
static int rc_trace;                                     /* (debug: every tick around the frame end) */

static void rc_trace_line(uint64_t t_last)
{
    printf("    t%+7.1f c %5u last %5u idle %u armed %u cap %u fe %u capr %u frames %u rc %u pend %u hdr %u dec %u\n",
           ((double)sim_ns - (double)t_last) / 1000.0, (unsigned)soak_tim4.CNT, ft.last_cnt, ft.idle, ft.armed,
           ft.cap_state, fe_ready, cap_ready, frames, rc_state, rc_pend, stat.ultra_headers, stat.decoded);
}
typedef struct {
    int sent, turned, checked, quiet, gate_closed;
    int stalled;                                         /* a flash write right before the frame: it is lost */
    double turn_us, check_us, tx_us, fwd_us, w_us;      /* host us after the frame's last falling edge */
    uint8_t a[32];
} rc_obs_t;

static double rc_host_us_since(uint64_t ns, uint64_t t0_ns)
{
    return ((double)ns - (double)t0_ns) / 1000.0 * (1.0 + host_ppm / 1e6);
}

/* a POLL slot */
static void rc_poll_build(uint8_t *s, uint8_t seq, uint8_t index, uint8_t slots, uint8_t page, uint8_t rate,
                          const uint8_t *arg, uint32_t nargs)
{
    uint32_t i;
    memset(s, 0, 32);
    s[0] = ULTRA_POLL;
    s[1] = (uint8_t)(ULTRA_LIGHT_BOARD << 4);
    s[2] = seq;
    s[RC_P_INDEX] = index;
    s[RC_P_SLOTS] = slots;
    s[RC_P_PAGE] = page;
    s[RC_P_RATE] = rate;
    for (i = 0; i < nargs && i < 23u; i++) {
        s[RC_P_ARG + i] = arg[i];
    }
    ultra_put16(s + 30, ultra_crc16(s, 30));
}

/* run ticks without main loop passes (a long pass: the stagger search, a console line) */
static void rc_ticks_no_pass(uint32_t n)
{
    while (n--) {
        sim_tick();
    }
}

/* one RW frame (n_slots slots, this receiver's POLL first) and its window; noise: a burst on DIN during the window
 * (as relayed answers); late_us: the main loop stalls this long from the frame end. Returns the observations. */
static void rc_window(const uint8_t *poll, uint32_t n_slots, uint8_t flags, int noise, uint32_t late_us, rc_obs_t *o)
{
    uint8_t f[32 * 8 + 8], nz[64];
    uint32_t i, nbits = 256u * n_slots + 64u, rate = poll[RC_P_RATE], slots = poll[RC_P_SLOTS];
    uint64_t t_start, t_last, t_end;
    int was = host_on;
    uint32_t stalls0 = sim_stalls;
    double w = ULTRA_RW_FIRST_US + (double)rc_slot_us((uint8_t)rate) * slots;
    host_on = 0;
    while (wf_on) {
        run_ticks(1);
    }
    run_ms(1);                                           /* >= 280 us of reset gap */
    memcpy(f, poll, 32);
    for (i = 1; i < n_slots; i++) {
        ultra_slot_nop(f + 32u * i);
    }
    t_start = sim_ns + rc_phase_ns;
    ultra_build_header_flags(f + 32u * n_slots, host_us(t_start + (uint64_t)nbits * 1250u), flags);
    memset(&rcr, 0, sizeof rcr);
    stalls0 = sim_stalls - stalls0;                      /* (flash writes in the gap before the frame) */
    wire_frame(f, nbits, t_start);
    t_last = wf_fall(nbits - 1u);
    while (sim_ns < t_last) {
        run_ticks(1);
        if (rc_trace && sim_ns + 400000u >= t_last) {
            rc_trace_line(t_last);
        }
    }
    if (late_us) {
        rc_ticks_no_pass(late_us / 50u + 4u);            /* (the frame end comes 100-150 us after the edge) */
    }
    t_end = t_last + (uint64_t)((w + 300.0) * 1000.0);
    while (sim_ns < t_end) {
        run_ticks(1);
        if (rc_trace && sim_ns < t_last + 700000u) {
            rc_trace_line(t_last);
        }
        if (noise && !wf_on && sim_ns >= t_last + 1150000u && sim_ns < t_last + 1200000u) {
            for (i = 0; i < sizeof nz; i++) {
                nz[i] = (uint8_t)(rnd32() >> 7);   /* (as relayed answers: any edges) */
            }
            wire_frame(nz, 256u, sim_ns + 1000u);        /* 256 edges' worth: a plan would open the gate */
            noise = 0;
        }
    }
    host_on = was;
    memset(o, 0, sizeof *o);
    o->w_us = w;
    o->turned = rcr.turns != 0u;
    o->checked = rcr.checks != 0u;
    o->quiet = rcr.quiets != 0u;
    o->sent = rcr.sends != 0u;
    o->gate_closed = rcr.gate_at_turn;
    o->stalled = stalls0 != 0u;
    o->turn_us = rc_host_us_since(rcr.turn_ns, t_last);
    o->check_us = rc_host_us_since(rcr.check_ns, t_last);
    o->tx_us = rc_host_us_since(rcr.tx_ns, t_last);
    o->fwd_us = rc_host_us_since(rcr.fwd_ns, t_last);
    memcpy(o->a, rcr.buf, 32);
}

/* the spec's windows (host us from the frame end) */
static int rc_timing_ok(const rc_obs_t *o, const uint8_t *poll)
{
    double s_i = ULTRA_RW_FIRST_US + (double)rc_slot_us(poll[RC_P_RATE]) * poll[RC_P_INDEX];
    int ok = o->turned && o->gate_closed && o->turn_us >= ULTRA_RW_DIR_MIN_US && o->turn_us <= ULTRA_RW_DIR_MAX_US &&
             o->checked && o->check_us >= ULTRA_RW_DIR_MAX_US && o->check_us <= ULTRA_RW_FIRST_US - 23.0 &&
             o->fwd_us >= o->w_us && o->fwd_us <= o->w_us + ULTRA_RW_END_US;
    if (o->sent) {
        ok &= o->tx_us >= s_i - 20.0 && o->tx_us <= s_i + 80.0;
    }
    if (!ok) {
        printf("  window: turn %.1f, check %.1f, tx %.1f (s_i %.0f), forward %.1f (W %.0f) us\n", o->turn_us,
               o->check_us, o->sent ? o->tx_us : -1.0, s_i, o->fwd_us, o->w_us);
    }
    return ok;
}

/* the answer's framing; returns its PAGE byte, -1 = bad */
static int rc_answer_ok(const rc_obs_t *o, const uint8_t *poll)
{
    const uint8_t *a = o->a;
    const uint8_t *u = rc_uid;
    if (!o->sent || a[0] != ULTRA_RSP_SYNC || ultra_get16(a + 30) != ultra_crc16(a, 30) ||
        (a[2] >> 4) != poll[RC_P_INDEX] || a[3] != poll[2] || ultra_get16(a + 4) != ultra_crc16(u, 12)) {
        printf("  answer: sent %d sync %02X crc %d index %u seq %u/%u uid %04X\n", o->sent, a[0],
               ultra_get16(a + 30) == ultra_crc16(a, 30), a[2] >> 4, a[3], poll[2], (unsigned)ultra_get16(a + 4));
        return -1;
    }
    return a[1];
}

static int rc_ask(uint8_t page, const uint8_t *arg, uint32_t nargs, rc_obs_t *o)
{
    uint8_t p[32];
    rc_poll_build(p, rc_seq++, 0, 1, page, 0, arg, nargs);
    rc_window(p, 1, ULTRA_HDR_RW, 0, 0, o);
    CHECK(rc_timing_ok(o, p), "rc: page %u: the window's timing", page);
    return rc_answer_ok(o, p);
}

#define D(o) ((o)->a + ULTRA_RSP_DATA)

static void rc_boot_locked(void)
{
    boot(5000u, 100000u, 7000000ull);
    host_on = 1;
    run_ms(1500);                                        /* locked, the clock's rate measured */
    CHECK(ultra_on && ul.clk.rated, "rc: locked and rated (%u %u)", ultra_on, ul.clk.rated);
}

/* ---------------------------------------------------------------------------------------------- rcpages */
static void sc_rcpages(void)
{
    rc_obs_t o;
    int pg, i, j;
    uint8_t arg[23], seq_def, seq_set, seq_fx, seq_cfg;
    uint32_t frames0, dec0, rearms0;
    printf("soak rcpages: every page, the framing and the fields, the window's timing, DIN traffic in the window\n");
    rc_boot_locked();
    seq_def = useq;
    cmd_def(0, 0, 3, 2800);
    cmd_def(1, 3, 2, 1000);
    seq_set = useq;
    cmd_set(0, 40000, 0, 0);
    cmd_set(1, 20000, 0, 0);
    seq_fx = useq;
    cmd_effect(1, ULTRA_FX_BREATHE, P_BREATHE, 7);
    seq_cfg = useq;
    cmd_config(ULTRA_CFG_SOFT_START, 1);
    run_ms(3000);

    /* STATUS */
    pg = rc_ask(ULTRA_PAGE_STATUS, 0, 0, &o);
    CHECK(pg == ULTRA_PAGE_STATUS, "rcpages: STATUS answered (%d)", pg);
    CHECK(ultra_get24(D(&o)) == fw_version >> 8 && D(&o)[3] == (BOARD << 4) && labs((long)ultra_get24(D(&o) + 4) -
          (long)up_s) <= 1 && D(&o)[7] == 0x03u, "rcpages: STATUS version %06X, hw %02X, uptime %u (%u), reset %02X",
          ultra_get24(D(&o)), D(&o)[3], ultra_get24(D(&o) + 4), up_s, D(&o)[7]);
    CHECK(D(&o)[ULTRA_ST_STATE] == pwrev_state(&pw) && (int16_t)ultra_get16(D(&o) + ULTRA_ST_MCU_TEMP) == mt.c10 &&
          ultra_get16(D(&o) + 11) == ULTRA_ST_NA && ultra_get16(D(&o) + ULTRA_ST_VDDA) == mt.vdda_mv,
          "rcpages: STATUS state %02X, MCU %d (%d), NTC %04X, VDDA %u", D(&o)[ULTRA_ST_STATE],
          (int16_t)ultra_get16(D(&o) + ULTRA_ST_MCU_TEMP), mt.c10, ultra_get16(D(&o) + 11),
          ultra_get16(D(&o) + ULTRA_ST_VDDA));
    CHECK(abs((int)ultra_get16(D(&o) + 13) - 2400) <= 5 && abs((int)ultra_get16(D(&o) + 17) - (int)is.i_avg) <= 20 &&
          D(&o)[19] == 0x03u && D(&o)[20] == eff.pwm_preset && ultra_get16(D(&o) + 21) == rc_cfg_crc() &&
          D(&o)[23] == (uint8_t)((frx.ps.mode << 4) | (eff.stagger & 0x0Fu)),
          "rcpages: STATUS V+ %u x10 mV, I %u mA (%d), "
          "flags %02X, preset %u, CONFIG crc %04X, mode %02X", ultra_get16(D(&o) + 13), ultra_get16(D(&o) + 17),
          (int)is.i_avg, D(&o)[19], D(&o)[20], ultra_get16(D(&o) + 21), D(&o)[23]);
    CHECK(o.a[2] == 0x00u, "rcpages: no RFLAGS (%02X; latched %02X, limit events %u, i %d mA)", o.a[2],
          pwrev_latched(&pw), pw.limit_events, is.i_avg);

    /* OUTPUTS */
    pg = rc_ask(ULTRA_PAGE_OUTPUTS, 0, 0, &o);
    CHECK(pg == ULTRA_PAGE_OUTPUTS, "rcpages: OUTPUTS answered (%d)", pg);
    for (i = 0, j = 0; i < 5; i++) {
        j += abs((int)ultra_get16(D(&o) + 2 * i) - (int)out_now[i]) > 2000;   /* (breathe moves CH4-5) */
    }
    CHECK(j == 0 && D(&o)[20] == 0x02u && D(&o)[21] == 0x03u && D(&o)[22] == 0u && (D(&o)[23] & ~0x20u) == 0u,
          "rcpages: OUTPUTS %u %u %u, effects %02X, defined %02X, scene %02X, why %02X", ultra_get16(D(&o)),
          ultra_get16(D(&o) + 2), ultra_get16(D(&o) + 4), D(&o)[20], D(&o)[21], D(&o)[22], D(&o)[23]);
    CHECK(ultra_get16(D(&o) + 10) == sat16((int32_t)isense_ch_ma(&is, 0, is.applied[0])) &&
          ultra_get16(D(&o) + 18) < 2000u, "rcpages: OUTPUTS currents CH1 %u mA (the model's %u), CH5 %u mA",
          ultra_get16(D(&o) + 10), sat16((int32_t)isense_ch_ma(&is, 0, is.applied[0])), ultra_get16(D(&o) + 18));

    /* ACKS: the default types, then CONFIG / IDENTIFY / SCENE */
    pg = rc_ask(ULTRA_PAGE_ACKS, 0, 0, &o);
    CHECK(pg == ULTRA_PAGE_ACKS && D(&o)[0] == seq_def && D(&o)[1] == seq_set && D(&o)[3] == (uint8_t)(seq_def + 1u) &&
          D(&o)[4] == (uint8_t)(seq_set + 1u) && D(&o)[5] == seq_fx && D(&o)[2] == 0u,
          "rcpages: ACKS light 0 %u %u %u, light 1 %u %u %u (sent %u %u / %u %u %u)", D(&o)[0], D(&o)[1], D(&o)[2],
          D(&o)[3], D(&o)[4], D(&o)[5], seq_def, seq_set, seq_def + 1, seq_set + 1, seq_fx);
    CHECK(ultra_get24(D(&o) + 18) == 0x3FFC4u && ultra_get16(D(&o) + 21) == rc_cfg_crc(),
          "rcpages: ACKS never received %06X (light 0 EFFECT, lights 2-4, the board)", ultra_get24(D(&o) + 18));
    arg[0] = ULTRA_CONFIG;
    arg[1] = ULTRA_IDENTIFY;
    arg[2] = ULTRA_SCENE;
    pg = rc_ask(ULTRA_PAGE_ACKS, arg, 3, &o);
    CHECK(pg == ULTRA_PAGE_ACKS && D(&o)[15] == seq_cfg && (ultra_get24(D(&o) + 18) & (7u << 15)) == (6u << 15),
          "rcpages: ACKS the board's CONFIG %u (sent %u), never %06X", D(&o)[15], seq_cfg, ultra_get24(D(&o) + 18));

    /* ERRORS (built before this window counts, and before its answer is sent) */
    pg = rc_ask(ULTRA_PAGE_ERRORS, 0, 0, &o);
    for (i = 1, j = 0; i < 6; i++) {
        j += (int)stat.dec_err[i];
    }
    CHECK(pg == ULTRA_PAGE_ERRORS && ultra_get16(D(&o)) == (uint16_t)frames &&
          ultra_get16(D(&o) + 10) == (uint16_t)(rc_windows - 1u) && ultra_get16(D(&o) + 12) ==
          (uint16_t)(rc_answers - 1u) && ultra_get16(D(&o) + 14) == 0u && ultra_get16(D(&o) + 16) == 0u &&
          ultra_get16(D(&o) + 8) == (uint16_t)j && ultra_get16(D(&o) + 20) == 0x2A0u,
          "rcpages: ERRORS frames %u (%u), windows %u, answers %u, missed %u, aborted %u, decode errors %u",
          ultra_get16(D(&o)), frames, ultra_get16(D(&o) + 10), ultra_get16(D(&o) + 12), ultra_get16(D(&o) + 14),
          ultra_get16(D(&o) + 16), ultra_get16(D(&o) + 8));

    /* IDENTITY */
    pg = rc_ask(ULTRA_PAGE_IDENTITY, 0, 0, &o);
    CHECK(pg == ULTRA_PAGE_IDENTITY && memcmp(D(&o), rc_uid, 12) == 0 &&
          ultra_get32(D(&o) + ULTRA_ID_FW_VERSION) == fw_version && D(&o)[17] == BOARD &&
          ultra_get16(D(&o) + 18) == 0x000Fu && ultra_get32(D(&o) + ULTRA_ID_FEATURES) == 0x7FFFBFu,   /* (0.6.9:
                                                    LIFETIME, DIAG, RETRY, LOSS_SCENE; 0.6.10 STANDBY, LIFE_SYNC) */
          "rcpages: IDENTITY version %08X, hw %u, caps %04X, features %08X", ultra_get32(D(&o) + 12), D(&o)[17],
          ultra_get16(D(&o) + 18), ultra_get32(D(&o) + 20));

    /* CALIBRATION / CAL_TEST before and after a test (IDENTIFY 2) */
    pg = rc_ask(ULTRA_PAGE_CALIBRATION, 0, 0, &o);
    CHECK(pg == ULTRA_PAGE_CALIBRATION && D(&o)[0] == 0u && ultra_get24(D(&o) + 4) == 0xFFFFFFu &&
          ultra_get16(D(&o) + 21) == clim.limit_ma, "rcpages: CALIBRATION before a test: levels %u, age %06X, "
          "limit %u", D(&o)[0], ultra_get24(D(&o) + 4), ultra_get16(D(&o) + 21));
    cmd_identify(ULTRA_IDENT_CAL, 0);
    run_ms(4000);
    pg = rc_ask(ULTRA_PAGE_CALIBRATION, 0, 0, &o);
    CHECK(pg == ULTRA_PAGE_CALIBRATION && D(&o)[0] == 4u && D(&o)[1] == 2u && ultra_get24(D(&o) + 4) <= 4u &&
          abs((int)ultra_get16(D(&o) + 9) - (int)ld_ma[0]) < (int)ld_ma[0] / 10 &&   /* (0.6.9: the bus-side loads) */
          abs((int)ultra_get16(D(&o) + 15) - (int)ld_ma[3]) < (int)ld_ma[3] / 10 &&
          ultra_get16(D(&o) + 19) == clim.connected_ma, "rcpages: CALIBRATION after a test: levels %u, model %u, "
          "age %u, CH1 %u, CH4 %u, connected %u", D(&o)[0], D(&o)[1], ultra_get24(D(&o) + 4), ultra_get16(D(&o) + 9),
          ultra_get16(D(&o) + 15), ultra_get16(D(&o) + 19));
    arg[0] = 0;
    pg = rc_ask(ULTRA_PAGE_CAL_TEST, arg, 1, &o);
    CHECK(pg == ULTRA_PAGE_CAL_TEST && abs((int)ultra_get16(D(&o) + 6) - (int)ld_ma[0]) < (int)ld_ma[0] / 10 &&
          abs((int)ultra_get16(D(&o) + 0) - (int)ld_ma[0] / 4) < 60, "rcpages: CAL_TEST part 0: CH1 25 %% %u, 100 %% %u",
          ultra_get16(D(&o)), ultra_get16(D(&o) + 6));
    arg[0] = 1;
    pg = rc_ask(ULTRA_PAGE_CAL_TEST, arg, 1, &o);
    CHECK(pg == ULTRA_PAGE_CAL_TEST && abs((int)ultra_get16(D(&o) + 14) - (int)ld_ma[4]) < (int)ld_ma[4] / 10 &&
          D(&o)[18] == 25u &&
          D(&o)[21] == 100u && D(&o)[22] == 4u, "rcpages: CAL_TEST part 1: CH5 100 %% %u, duties %u..%u, levels %u",
          ultra_get16(D(&o) + 14), D(&o)[18], D(&o)[21], D(&o)[22]);
    arg[0] = 2;
    pg = rc_ask(ULTRA_PAGE_CAL_TEST, arg, 1, &o);
    CHECK(pg == ULTRA_PAGE_UNSUPPORTED, "rcpages: CAL_TEST part 2: UNSUPPORTED (%d)", pg);

    /* POWER, SUPPLY, HEALTH, ENERGY: the core builders, from main.c's state */
    {
        uint8_t ref[24];
        pg = rc_ask(ULTRA_PAGE_POWER, 0, 0, &o);
        pwrev_page(&pw, ref);
        CHECK(pg == ULTRA_PAGE_POWER && memcmp(D(&o), ref, 21) == 0, "rcpages: POWER = pwrev_page");
        pg = rc_ask(ULTRA_PAGE_SUPPLY, 0, 0, &o);
        CHECK(pg == ULTRA_PAGE_SUPPLY && ultra_get16(D(&o)) == 0u && abs((int)ultra_get16(D(&o) + 4) - 24000) < 60,
              "rcpages: SUPPLY dips %u, now %u mV", ultra_get16(D(&o)), ultra_get16(D(&o) + 4));
        pg = rc_ask(ULTRA_PAGE_HEALTH, 0, 0, &o);
        CHECK(pg == ULTRA_PAGE_HEALTH && (D(&o)[0] & ULTRA_HE_F_TESTED) && D(&o)[1] == ULTRA_HE_NORMAL &&
              (int16_t)ultra_get16(D(&o) + 18) == temp_peak, "rcpages: HEALTH flags %02X, CH1 class %02X, peak %d",
              D(&o)[0], D(&o)[1], (int16_t)ultra_get16(D(&o) + 18));
        pg = rc_ask(ULTRA_PAGE_ENERGY, 0, 0, &o);
        CHECK(pg == ULTRA_PAGE_ENERGY && ultra_get32(D(&o)) > 0u && ultra_get32(D(&o) + 8) > 1000u,
              "rcpages: ENERGY %u mWh, %u mW", ultra_get32(D(&o)), ultra_get32(D(&o) + 8));
    }

    /* pages this firmware doesn't have */
    CHECK(rc_ask(ULTRA_PAGE_FW_UPDATE, 0, 0, &o) == ULTRA_PAGE_UNSUPPORTED && rc_ask(0x0F, 0, 0, &o) ==
          ULTRA_PAGE_UNSUPPORTED && rc_ask(0x80, 0, 0, &o) == ULTRA_PAGE_UNSUPPORTED && rc_ask(0xFE, 0, 0, &o) ==
          ULTRA_PAGE_UNSUPPORTED, "rcpages: FW_UPDATE, 0x0F (0.6.9: 0x0C / 0x0D defined; 0.6.10: 0x0E), 0x80, 0xFE "
          "answer UNSUPPORTED");
    for (i = 0, j = 0; i < 24; i++) {
        j |= D(&o)[i];
    }
    CHECK(j == 0, "rcpages: UNSUPPORTED's DATA is 0");

    /* DIN traffic in the window (relayed answers): no capture, no frame, the gate closed, re-armed once */
    {
        uint8_t p[32];
        uint32_t tail0 = stat.ultra_tail_err;
        frames0 = frames;
        dec0 = stat.decoded;
        rearms0 = sim_rearms;
        rc_poll_build(p, rc_seq++, 0, 4, ULTRA_PAGE_STATUS, 0, 0, 0);
        rc_window(p, 1, ULTRA_HDR_RW, 1, 0, &o);
        CHECK(rc_timing_ok(&o, p) && rc_answer_ok(&o, p) == ULTRA_PAGE_STATUS, "rcpages: a window with traffic");
        CHECK(frames == frames0 + 1u && stat.decoded == dec0 + 1u && sim_rearms == rearms0 + 2u && !sim_gate_forced,
              "rcpages: the window's traffic is no frame (frames +%u, decoded +%u, re-arms +%u, gate forced %u)",
              frames - frames0, stat.decoded - dec0, sim_rearms - rearms0, sim_gate_forced);
        dec0 = stat.decoded;
        run_ms(100);
        CHECK(stat.decoded >= dec0 + 4u && ultra_on && stat.ultra_tail_err == tail0, "rcpages: the frames after it "
              "decode (%u), tail errors +%u", stat.decoded - dec0, stat.ultra_tail_err - tail0);
    }
    CHECK(rc_flash_in_window == 0u, "rcpages: no flash operation in a window (%u)", rc_flash_in_window);
}

/* ---------------------------------------------------------------------------------------------- rcpower */
static void rc_ack(const rc_obs_t *read, int dips_too, rc_obs_t *o)
{
    uint8_t arg[9];
    memset(arg, 0, sizeof arg);
    arg[0] = ULTRA_PW_ARG_ACK;
    memcpy(arg + 1, D(read) + ULTRA_PW_LIMIT_EVENTS, 6);
    if (dips_too) {
        ultra_put16(arg + 7, vdip.count);                /* (the SUPPLY page's count; read with it) */
    }
    CHECK(rc_ask(ULTRA_PAGE_POWER, arg, 9, o) == ULTRA_PAGE_POWER, "rcpower: POWER with the acknowledge");
}

static void sc_rcpower(void)
{
    rc_obs_t o, r;
    int pg;
    printf("soak rcpower: a short, a limit event, a supply dip: latches, EVENT, the acknowledge\n");
    rc_boot_locked();
    cmd_def(0, 0, 3, 1000);
    cmd_set(0, 65535, 0, 0);                             /* ~1.4 A (0.6.9: bus side, test_soak.c ld_ma) */
    run_ms(2000);
    pg = rc_ask(ULTRA_PAGE_POWER, 0, 0, &r);
    CHECK(pg == ULTRA_PAGE_POWER && D(&r)[ULTRA_PW_LATCHED] == 0u && !(r.a[2] & ULTRA_RSP_EVENT),
          "rcpower: nothing latched at first (%02X)", D(&r)[ULTRA_PW_LATCHED]);

    /* a short: the fast trip, latched until acknowledged */
    expect_trips = 1;
    short_ticks = 4;
    run_ms(50);
    pg = rc_ask(ULTRA_PAGE_STATUS, 0, 0, &o);
    CHECK((o.a[2] & ULTRA_RSP_EVENT) && (D(&o)[ULTRA_ST_STATE] & (ULTRA_ST_POWER_EVENT | ULTRA_ST_TRIPPED)) ==
          (ULTRA_ST_POWER_EVENT | ULTRA_ST_TRIPPED), "rcpower: after a short: EVENT on STATUS, state %02X",
          D(&o)[ULTRA_ST_STATE]);
    pg = rc_ask(ULTRA_PAGE_POWER, 0, 0, &r);
    CHECK(D(&r)[ULTRA_PW_LATCHED] == ULTRA_PW_EV_SHORT && ultra_get16(D(&r) + ULTRA_PW_SHORT_TRIPS) == 1u &&
          (D(&r)[ULTRA_PW_NOW] & ULTRA_ST_TRIPPED), "rcpower: POWER latched %02X, short trips %u, now %02X",
          D(&r)[ULTRA_PW_LATCHED], ultra_get16(D(&r) + ULTRA_PW_SHORT_TRIPS), D(&r)[ULTRA_PW_NOW]);
    run_ms(3000);                                        /* the retry (its recovery counts a limit event too: 0.6.4) */
    rc_ack(&r, 1, &o);                                   /* the read from before it */
    CHECK(D(&o)[ULTRA_PW_LATCHED] == (ultra_get16(D(&o) + ULTRA_PW_LIMIT_EVENTS) ? ULTRA_PW_EV_LIMITED : 0u),
          "rcpower: the short acknowledged, an event after the read stays latched (%02X, limit events %u)",
          D(&o)[ULTRA_PW_LATCHED], ultra_get16(D(&o) + ULTRA_PW_LIMIT_EVENTS));
    rc_ack(&o, 1, &r);
    CHECK(D(&r)[ULTRA_PW_LATCHED] == 0u && !(r.a[2] & ULTRA_RSP_EVENT), "rcpower: all acknowledged: latched %02X, "
          "RFLAGS %02X", D(&r)[ULTRA_PW_LATCHED], r.a[2]);
    run_ms(2500);                                        /* 2 s without a reduction: the next one is a new event */

    /* a supply dip, then a limit event after the hub's read: the acknowledge leaves the newer one latched */
    supply_mv = 18000u;
    run_ms(20);
    supply_mv = 24000u;
    run_ms(200);
    pg = rc_ask(ULTRA_PAGE_POWER, 0, 0, &r);
    CHECK(D(&r)[ULTRA_PW_LATCHED] == ULTRA_PW_EV_SUPPLY_DIP && vdip.count == 1u, "rcpower: a dip latched (%02X, %u)",
          D(&r)[ULTRA_PW_LATCHED], vdip.count);
    {
        uint8_t a[1] = { 0 };
        pg = rc_ask(ULTRA_PAGE_SUPPLY, a, 1, &o);
        CHECK(pg == ULTRA_PAGE_SUPPLY && ultra_get16(D(&o)) == 1u && D(&o)[12] == 1u && ultra_get16(D(&o) + 16) <
              19000u, "rcpower: SUPPLY dips %u, logged %u, lowest %u mV", ultra_get16(D(&o)), D(&o)[12],
              ultra_get16(D(&o) + 16));
    }
    cmd_config(ULTRA_CFG_BOARD_MAX, 1000);               /* 1 A: the limiter reduces (a limit event; 0.6.9: 2 A, the
                                                            bus-side load is 1.4 A) */
    run_ms(500);
    rc_ack(&r, 0, &o);                                   /* a 0.2.2 hub: dips 0, read before the limit event */
    CHECK(D(&o)[ULTRA_PW_LATCHED] == (ULTRA_PW_EV_LIMITED | ULTRA_PW_EV_SUPPLY_DIP) && (o.a[2] & ULTRA_RSP_EVENT) &&
          ultra_get16(D(&o) + ULTRA_PW_LIMIT_EVENTS) == ultra_get16(D(&r) + ULTRA_PW_LIMIT_EVENTS) + 1u,
          "rcpower: after a 0.2.2 acknowledge of the earlier read: limit + dip latched (%02X), limit events %u (%u "
          "read)", D(&o)[ULTRA_PW_LATCHED], ultra_get16(D(&o) + ULTRA_PW_LIMIT_EVENTS),
          ultra_get16(D(&r) + ULTRA_PW_LIMIT_EVENTS));
    rc_ack(&o, 1, &r);
    CHECK(D(&r)[ULTRA_PW_LATCHED] == 0u && !(r.a[2] & ULTRA_RSP_EVENT), "rcpower: both acknowledged (%02X)",
          D(&r)[ULTRA_PW_LATCHED]);
    rc_ack(&o, 1, &r);
    CHECK(D(&r)[ULTRA_PW_LATCHED] == 0u, "rcpower: a repeated acknowledge does no harm (%02X)",
          D(&r)[ULTRA_PW_LATCHED]);
    pg = rc_ask(ULTRA_PAGE_STATUS, 0, 0, &o);
    CHECK(!(D(&o)[ULTRA_ST_STATE] & ULTRA_ST_POWER_EVENT) && (D(&o)[ULTRA_ST_STATE] & ULTRA_ST_LIMITING),
          "rcpower: STATUS limiting, nothing latched (%02X)", D(&o)[ULTRA_ST_STATE]);
    CHECK(rc_flash_in_window == 0u, "rcpower: no flash operation in a window (%u)", rc_flash_in_window);
}

/* ---------------------------------------------------------------------------------------------- rcfail */
static void sc_rcfail(void)
{
    rc_obs_t o;
    uint8_t p[32];
    uint32_t n, miss0, ok = 1, w0, life0, retries = 0;
    printf("soak rcfail: no RW, unrated, late, CRC, relay only, contention, rates, chain positions, flash\n");
    boot(5000u, 100000u, 7000000ull);
    host_on = 1;
    run_ms(200);                                         /* locked, not rated yet (0.5 s of headers) */
    rc_poll_build(p, rc_seq++, 0, 1, ULTRA_PAGE_STATUS, 0, 0, 0);
    miss0 = rc_missed;
    rc_window(p, 1, ULTRA_HDR_RW, 0, 0, &o);
    CHECK(ultra_on && !ul.clk.rated && !o.turned && rc_missed == miss0 + 1u, "rcfail: not rated yet: no part "
          "(turned %d, missed +%u)", o.turned, rc_missed - miss0);
    run_ms(1500);
    rc_window(p, 1, 0, 0, 0, &o);                        /* a POLL without RW: dropped, not applied */
    CHECK(!o.turned && rc_missed == miss0 + 1u, "rcfail: a POLL in a frame without RW: no window");
    rc_window(p, 1, ULTRA_HDR_RW | ULTRA_HDR_EXT, 0, 0, &o);   /* (review r6) RW with an extension it doesn't know */
    CHECK(!o.turned && rc_missed == miss0 + 1u, "rcfail: RW with EXT (must understand: every slot ignored): no window "
          "(turned %d)", o.turned);
    rc_window(p, 1, ULTRA_HDR_RW, 0, 900, &o);           /* the main loop 900 us late */
    CHECK(!o.turned && rc_missed == miss0 + 2u, "rcfail: a late main loop: no part (missed +%u)", rc_missed - miss0);
    rc_window(p, 1, ULTRA_HDR_RW, 0, 250, &o);           /* 250 us late: still in time */
    CHECK(o.turned && o.sent && rc_timing_ok(&o, p), "rcfail: 250 us late: in time");
    p[20] ^= 0x01u;                                      /* a POLL with a CRC error */
    rc_window(p, 1, ULTRA_HDR_RW, 0, 0, &o);
    CHECK(!o.turned, "rcfail: a POLL with a CRC error: no window");
    rc_poll_build(p, rc_seq++, ULTRA_POLL_RELAY_ONLY, 3, ULTRA_PAGE_POWER, 0, (const uint8_t *)"\x01\x00\x09", 3);
    rc_window(p, 1, ULTRA_HDR_RW, 1, 0, &o);
    CHECK(o.turned && o.checked && !o.sent && rc_timing_ok(&o, p) && pw.ack_limit == 0u, "rcfail: relay only: "
          "turns around, sends nothing, ignores the page's acknowledge");

    /* contention: at the check (quiet for the window), then at the burst; CONTENTION in the next answer */
    rc_contend_check = 1;
    rc_poll_build(p, rc_seq++, 0, 1, ULTRA_PAGE_STATUS, 0, 0, 0);
    rc_window(p, 1, ULTRA_HDR_RW, 0, 0, &o);
    CHECK(o.turned && o.quiet && !o.sent && o.fwd_us >= o.w_us, "rcfail: the check finds the line taken: quiet, no "
          "answer, forward at W");
    rc_contend_tx = 1;
    rc_window(p, 1, ULTRA_HDR_RW, 0, 0, &o);
    CHECK(o.turned && o.quiet && !o.sent && o.fwd_us >= o.w_us, "rcfail: the burst finds the line taken: no answer, "
          "quiet for the rest of the window (as at the check), forward at W (quiet %d)", o.quiet);
    rc_window(p, 1, ULTRA_HDR_RW, 0, 0, &o);
    CHECK(rc_answer_ok(&o, p) == ULTRA_PAGE_STATUS && (o.a[2] & 0x0Fu) == ULTRA_RSP_CONTENTION, "rcfail: the next "
          "answer has CONTENTION (%02X)", o.a[2]);
    rc_window(p, 1, ULTRA_HDR_RW, 0, 0, &o);
    CHECK((o.a[2] & 0x0Fu) == 0u && rc_aborts == 2u, "rcfail: and the one after it not (%02X), aborts %u", o.a[2],
          rc_aborts);

    /* rates, indices, chain positions, tick phases. A flash write in the gap right before a frame loses that frame
     * (0.6.4: the tracker re-arms after 10 ms idle or a frame end): no window, as for a lost POLL; the hub retries. */
    for (n = 0; n < 48u; n++) {
        uint8_t rate = (uint8_t)(n % 3u), slots = (uint8_t)(1u + n % 7u), idx = (uint8_t)(n % slots);
        rc_phase_ns = 1000u + (n * 7919u) % 49000u;
        rc_poll_build(p, rc_seq++, idx, slots, ULTRA_PAGE_ENERGY, rate, 0, 0);
        {
            uint32_t m0 = rc_missed, w0n = rc_windows, f0 = frames, d0 = stat.decoded, h0 = stat.ultra_headers;
            uint32_t t0e = stat.ultra_tail_err, e0 = stat.dec_err[1] + stat.dec_err[2] + stat.dec_err[3] +
                           stat.dec_err[4] + stat.dec_err[5];
            int good;
            rc_trace = getenv("RC_TRACE") && (uint32_t)atoi(getenv("RC_TRACE")) == n;
            rc_window(p, 1u + n % 4u, ULTRA_HDR_RW, n % 2u, 0, &o);
            rc_trace = 0;
            if (!o.turned && o.stalled) {
                retries++;
                p[2] = rc_seq++;                         /* the hub's retry: a new SEQ */
                ultra_put16(p + 30, ultra_crc16(p, 30));
                rc_window(p, 1u + n % 4u, ULTRA_HDR_RW, n % 2u, 0, &o);
            }
            good = rc_timing_ok(&o, p) && rc_answer_ok(&o, p) == ULTRA_PAGE_ENERGY &&
                   abs((int)rcr.brr - (int)(rc_brr(ul.clk.rate_q16, rate))) == 0 && (rcr.brr >> rate) >= 143u &&
                   (rcr.brr >> rate) <= 145u;
            if (!good) {
                printf("  window %u (rate %u, R %u, index %u, %u slots, phase %u ns): missed +%u, windows +%u, frames "
                       "+%u, decoded +%u, headers +%u, tail errors +%u, decode errors +%u, locked %u, rated %u\n",
                       n, rate, slots, idx, 1u + n % 4u, rc_phase_ns, rc_missed - m0, rc_windows - w0n, frames - f0,
                       stat.decoded - d0, stat.ultra_headers - h0, stat.ultra_tail_err - t0e,
                       stat.dec_err[1] + stat.dec_err[2] + stat.dec_err[3] + stat.dec_err[4] + stat.dec_err[5] - e0,
                       ultra_on, ul.clk.rated);
            }
            ok &= good;
        }
    }
    CHECK(ok && retries <= 3u, "rcfail: 48 windows over the rates, indices, 1-4 slots and tick phases (BRR %u at the "
          "0.6 %% clock; %u retried after a flash write)", rcr.brr, retries);
    printf("  48 windows: %u lost to a flash write right before the frame, retried\n", retries);

    /* flash: a settings write and a dip's lifetime write while RW frames come every 10 ms: never inside a window */
    w0 = W_SETTINGS();
    life0 = W_LIFE();
    cmd_config(ULTRA_CFG_BOOT_PROTOCOL, PMODE_ULTRA);    /* persisted: written 2 s later, while the flash admits it */
    run_ms(100);
    supply_mv = 18000u;
    for (n = 0; n < 1000u; n++) {
        rc_poll_build(p, rc_seq++, 0, 3, ULTRA_PAGE_STATUS, 0, 0, 0);
        rc_window(p, 1, ULTRA_HDR_RW, 0, 0, &o);
        if (n == 1u) {
            supply_mv = 24000u;
        }
    }
    CHECK(W_SETTINGS() == w0 + 1u && W_LIFE() > life0 && rc_flash_in_window == 0u, "rcfail: the settings write (%u) "
          "and the dip's lifetime write (%u) between windows, none inside (%u)", W_SETTINGS() - w0, W_LIFE() - life0,
          rc_flash_in_window);
}

#include "test_r6_rc_soak.c"

static int rc_scenario(const char *s)
{
    if (!strcmp(s, "r6rc")) {
        sc_r6rc();
    } else if (!strcmp(s, "rcpages")) {
        sc_rcpages();
    } else if (!strcmp(s, "rcpower")) {
        sc_rcpower();
    } else if (!strcmp(s, "rcfail")) {
        sc_rcfail();
    } else {
        return 0;
    }
    return 1;
}
#else
static int rc_scenario(const char *s)
{
    (void)s;
    return 0;
}
#endif
