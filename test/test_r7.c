/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* (0.6.9, review round 7) the approved items' scenarios. Included by test_soak.c; soak_tests.exe <name>.
 *
 *   r7ack <0 | 2>  (A1, gpt #1) a WS config frame's acknowledgement: with feedback 2 ("off") D9 only, the outputs keep
 *                  showing the frames' values every ms of it; with feedback 0 the strip acknowledgement as before
 *                  (0.6.8): the red channel shows the pattern, the others dark for it
 *   r7cal <ok | limit | supply | trip>  (A2, gpt #2) a host's calibration test (IDENTIFY 2) after a complete one, the
 *                  light at 50 %: ok completes (no outcome); limit: CONFIG 21 holds the board at 0.8 x R's load
 *                  (1.2 A, R 1.5 A at 100 %; the receiver 360 / 450 mA): the test stops at the limit, never above it; supply: a 40 ms dip during it; trip: a
 *                  short during it. A stop: the stored test and the model as before (nothing stored), the event
 *                  "calibration incomplete", D9 code 8, `calibration_last` (full console), the receiver's CALIBRATION
 *                  byte 1 high nibble; the light back after it
 *   r7retry        (B7, Quindor: "10 hiccups with 1 s in between") a short that stays: 10 retries 1 s apart, then the
 *                  outputs stay off (the event "off: retries used up", the POWER page's retry 255 s), no retry for
 *                  70 s; a new command (Ultra SET): a retry, the count anew; the short gone and a new command: the
 *                  light back; CONFIG 24 = 2 retries 3 s apart: stored (the settings extension), then that policy
 *   r7retryws      (B7) the policy by WS config frames 0x15 / 0x16
 *   r7loss <ultra | ws> <scene | none>  (B10) signal loss "scene" (3): the light at 50 %, a power-on scene of 20 %
 *                  stored (none: nothing stored); the controller stops: after the loss timeout the outputs fade to the
 *                  scene (none: they hold); the controller back: they cross-fade back to its value. No output step
 *                  over 200 counts in any ms (no blink)
 *   r7fault        (B2) the fault memory in the lifetime record (format 2): a boot after a watchdog reset that
 *                  followed an exception (the breadcrumbs: cause 5, 2 h 2 min up), a short (its peak), a 40 ms supply
 *                  dip: the record's trip / dip / reset fields; stored with the record's dark write, no write of their
 *                  own; read back from the flash; `faults=` in the status (full console)
 *   r7con <ultra | ws>  (0.6.9, the console) the live line's link: "no signal", "Ultra 50fps", "WS2805 50fps"
 *                  (both consoles; the release console's stream, the full console's `watch`); ultra, the status (full
 *                  console):
 *                  `settings=` by the console keys, '*' on a value an Ultra CONFIG overrides (nothing to save), `save=`
 *                  pending: debounce after a `set`, stored after it (the hour's write while bright); then CONFIG 24
 *                  while lit at 50 %: "waiting for dark"; ok once stored (dark)
 *   r7pages        (0.6.9, protocol 0.2.5; the receiver's builds, else nothing to check) the LIFETIME and DIAG pages: a
 *                  boot after a watchdog reset that followed an exception (breadcrumbs: cause 5, pc 0x4A2E, 7320 s
 *                  up), the lifetime record set to known values: LIFETIME's fields; DIAG 0: the reset, the breadcrumbs,
 *                  the CPU load, the overruns, the impedance, the flash state and the save state (CONFIG 24 while lit:
 *                  pending, debounce; lit: waiting for dark (no hour's write while bright yet); dark: stored), C10
 *                  not measured; a short and a supply dip: DIAG 1's fault memory, the trend not known; DIAG 2:
 *                  UNSUPPORTED; IDENTITY's features
 *   r7hot          (B6) the die over 95 C (the sensor's count 1340: ~99 C): the event "too hot" once, D9's fault code,
 *                  the HEALTH flag (and STATUS's reason on a receiver) while hot; under 85 C again: the flag off, no
 *                  second event; nothing dims (the outputs as set throughout)
 *   r7b3           (B3, Quindor's option b: the heartbeat and the request are parked) D9's code once when the running
 *                  model's flags rise: current while dark (a FET stuck on): 6; a calibration suggested: 7; D9 only
 *   r7why          (0.6.9, protocol 0.2.5; the receiver's builds) STATUS 8's reason (bits 4-7) through a run: none, a
 *                  short, a supply dip (30 s), the soft start, signal loss; the receiver's ratings bus side (POWER's and
 *                  CALIBRATION's limit 2250 mA); ENERGY's V_B (the aux input) and the input current's peak since the
 *                  last ENERGY answer
 *   r7upd <0 | 1>  (0.6.9, Quindor) an update session's SAME / NACK on D9 only, with feedback 0 (red) and 1 (all): the
 *                  board as if behind a bootloader (upd.info: bootloader 2, the app's descriptor valid), lit at 50 %;
 *                  a session (PINGs, ANNOUNCE 0 / 1, END) of the version it has: SAME; the same session's END missed
 *                  (shown 2 s after the last UPD frame); a session for another hardware id: NACK. Each time D9 shows
 *                  the pattern (its length: SAME 1.2 s, NACK 2.5 s) and no output leaves its value in any ms */

/* n WS2805 frames of one board (5 bytes), 50 per second, the board's 40 bits first */
static void r7_ws(const uint8_t *raw5, uint32_t n)
{
    static uint8_t b[16];
    uint32_t k;
    memcpy(b, raw5, 5);
    for (k = 0; k < n; k++) {
        wire_frame(b, 40u, sim_ns + 7000u);
        run_ms(20);
    }
}

static void sc_r7ack(uint8_t fb)
{
    static const uint8_t col[5] = { 0x40, 0x80, 0xC0, 0x20, 0x10 };
    uint8_t raw[10];
    uint16_t before[5];
    uint32_t t, i, act = 0, moved[5] = { 0, 0, 0, 0, 0 }, led0;
    printf("soak r7ack %u: a WS config acknowledgement with feedback %u (%s)\n", fb, fb, fb == 2u ? "off: D9 only" :
           "the red channel");
    boot(0, 0, 0);
    r7_ws(col, 100);                                     /* WS2805 locked and shown */
    config_build(CMD_FEEDBACK_CH, fb, 0, raw);
    r7_ws(raw, 3);                                       /* the feedback setting (its own acknowledgement) */
    r7_ws(col, 300);                                     /* 6 s: that one over */
    CHECK(proto_active(&frx.ps) == PROTO_WS2805 && set.feedback_ch == fb && !ack.active, "r7ack: WS2805, feedback %u "
          "(set %u, ack %u)", fb, set.feedback_ch, ack.active);
    memcpy(before, out_now, sizeof before);
    led0 = sim_led_on;
    config_build(CMD_DITHER_THRESH, set.dither_thresh, 0, raw);   /* the same value: SAME */
    r7_ws(raw, 3);
    for (t = 0; t < 5000u; t++) {
        if (t % 20u == 0u) {
            wire_frame(col, 40u, sim_ns + 7000u);
        }
        run_ms(1);
        if (ack.active) {
            act++;
            for (i = 0; i < 5u; i++) {
                moved[i] += out_now[i] != before[i];
            }
        }
    }
    printf("  acknowledgement %u ms, D9 on %u times; ms with an output off its value: %u %u %u %u %u\n", act,
           sim_led_on - led0, moved[0], moved[1], moved[2], moved[3], moved[4]);
    CHECK(act > 0u && sim_led_on > led0, "r7ack: the acknowledgement ran (%u ms) and D9 blinked", act);
    if (fb == 2u) {
        CHECK(moved[0] + moved[1] + moved[2] + moved[3] + moved[4] == 0u, "r7ack: feedback off: no output left its "
              "value during the acknowledgement");
    } else {
        CHECK(moved[CH_R] > 0u && moved[1] == act && moved[2] == act && moved[3] == act && moved[4] == act,
              "r7ack: feedback red: the strip acknowledgement as before (the red channel's pattern, the others dark)");
    }
}

static void sc_r7cal(const char *how)
{
    static const char *const names[4] = { "ok", "trip", "limit", "supply" };
    isense_cal_t c0, c1;
    uint32_t t, want, over = 0, led0, k, logged = 0;
    int32_t lim_led;
    for (want = 0; want < 4u && strcmp(how, names[want]) != 0; want++) {
    }
    if (want == 4u) {
        want = 0;
    }
    printf("soak r7cal %s: a calibration test %s\n", names[want], want == 0u ? "that completes" : "that stops");
    boot(0, 0, 0);
    host_on = 1;
    cmd_def(0, 0, 5, 1000);
    cmd_set(0, 32768, 0, 0);
    run_ms(3000);
    cmd_identify(ULTRA_IDENT_CAL, 0);                    /* a complete test first: the one that must survive */
    run_ms(3000);
    CHECK(is.calibrated && is.cal_fail == 0u && isense_cal_get(&is, &c0), "r7cal: the first test complete");
    run_ms(2000);
    if (want == ISENSE_CAL_F_LIMIT) {
        cmd_config(ULTRA_CFG_BOARD_MAX, (uint16_t)(ld_ma[0] * 4u / 5u));   /* the installation's limit: R draws 1.25 x
                                                            it at 100 % (1.2 / 1.5 A; the receiver's bus side: 360 /
                                                            450 mA) */
        run_ms(500);
    }
    lim_led = (int32_t)(ld_ma[0] * 4u / 5u) - (is.zero_valid ? is.zero_ma : 0);
    led0 = sim_led_on;
    cmd_identify(ULTRA_IDENT_CAL, 0);
    for (t = 0; t < 3000u; t++) {
        run_ms(1);
        if (want == ISENSE_CAL_F_SUPPLY && t == 600u) {
            supply_mv = 19000u;
        }
        if (want == ISENSE_CAL_F_SUPPLY && t == 640u) {
            supply_mv = 24000u;
        }
        if (want == ISENSE_CAL_F_TRIP && t == 600u) {
            expect_trips = 1;
            short_ticks = 3;
        }
        if (want == ISENSE_CAL_F_LIMIT && is.cal_step != 0u &&
            is.i_ms - (is.zero_valid ? is.zero_ma : 0) > lim_led * 103 / 100) {
            over++;                                      /* (the limiter's threshold 1.010, the ms to react) */
        }
    }
    run_ms(3000);
    (void)isense_cal_get(&is, &c1);
    printf("  outcome %u, cal_step %u, last event %u, ms over the limit %u, D9 on %u times, out %u\n", is.cal_fail,
           is.cal_step, evl.e[evl.head].type, over, sim_led_on - led0, out_now[0]);
    CHECK(is.cal_fail == want && is.cal_step == 0u, "r7cal: the test's end: %u (want %u)", is.cal_fail, want);
    if (want != 0u) {
        CHECK(memcmp(&c0, &c1, sizeof c0) == 0 && is.calibrated, "r7cal: the first test and the model kept (the "
              "stopped one never replaces them)");
        for (k = 0; k < EVLOG_N; k++) {
            logged += evl.e[k].type == EVL_CAL_FAIL;
        }
        CHECK(logged == 1u, "r7cal: the event \"calibration incomplete\" (%u)", logged);
        CHECK(sim_led_on > led0, "r7cal: D9 blinked (code 8)");
        CHECK(over <= 2u, "r7cal: never above the limit beyond the limiter's reaction (%u ms)", over);
#if CONSOLE_FULL
        console("status");
        CHECK(strstr(con_out, want == ISENSE_CAL_F_LIMIT ? "calibration_last=incomplete: limit" :
                     want == ISENSE_CAL_F_SUPPLY ? "calibration_last=incomplete: supply" :
                     "calibration_last=incomplete: trip") != NULL, "r7cal: the console's calibration_last");
#endif
#if BOARD_RETURN
        {
            rc_obs_t o;
            CHECK(rc_ask(ULTRA_PAGE_CALIBRATION, 0, 0, &o) == ULTRA_PAGE_CALIBRATION && (uint32_t)(D(&o)[1] >> 4) == want,
                  "r7cal: the CALIBRATION page's byte 1: the test's end (%02X)", D(&o)[1]);
        }
#endif
    } else {
        CHECK(memcmp(&c0, &c1, sizeof c0) != 0 || is.calibrated, "r7cal: the second test complete");
    }
    CHECK(out_now[0] != 0u, "r7cal: the light back after the test (%u)", out_now[0]);
}

/* a short that stays: every trip (is.trips) for `ms`, or until latched; the gaps between trips (ms) into gap[] */
static uint32_t r7_trips(uint32_t ms, uint32_t *gap, uint32_t ngap)
{
    uint32_t t, last = 0, n = 0, t0 = is.trips;
    for (t = 1; t <= ms && !is.latched; t++) {
        run_ms(1);
        if (is.trips != t0 + n) {
            if (n != 0u && n - 1u < ngap) {
                gap[n - 1u] = t - last;
            }
            last = t;
            n = is.trips - t0;
        }
    }
    return n;
}

static void sc_r7retry(void)
{
    uint32_t k, n, gap[32], ok, latch_ev = 0;
    printf("soak r7retry: the trips' retry policy (default 10 x 1 s, then off until a new command; CONFIG 24)\n");
    boot(0, 0, 0);
    host_on = 1;
    cmd_def(0, 0, 5, 1000);
    cmd_set(0, 32768, 0, 0);
    run_ms(3000);
    CHECK(is.retry_max == 10u && is.retry_s == 1u, "r7retry: the default policy (%u x %u s)", is.retry_max, is.retry_s);
    expect_trips = 1000u;
    short_ticks = 0xFFFFFFFFu;                           /* a short that stays */
    n = r7_trips(30000u, gap, 32u);
    for (k = 0, ok = 1; k + 1u < n && k < 32u; k++) {
        ok &= gap[k] >= 995u && gap[k] <= 1010u;         /* each retry 1 s after its trip */
    }
    for (k = 0; k < EVLOG_N; k++) {
        latch_ev += evl.e[k].type == EVL_LATCH;
    }
    printf("  %u trips (the first + %u retries), 1 s apart: %u (gap 1: %u ms); latched %u, the event %u, retry left "
           "%08X\n", n, n - 1u, ok, gap[0], is.latched, latch_ev, isense_retry_left_ms(&is));
    CHECK(n == 11u && ok && is.latched && latch_ev == 1u && sim_forced && isense_retry_left_ms(&is) == 0xFFFFFFFFu,
          "r7retry: 10 retries 1 s apart, then off and the event (%u trips)", n);
    n = r7_trips(70000u, gap, 0u);
    CHECK(n == 0u && sim_forced && is.latched, "r7retry: off: no retry in 70 s (%u)", n);
    cmd_set(0, 30000, 0, 0);                             /* a new command */
    run_ms(200);
    CHECK(is.trips > 0u && !is.latched && is.retry_n == 1u, "r7retry: a new command: a retry, the count anew (%u)",
          is.retry_n);
    short_ticks = 0;                                     /* the short repaired */
    run_ms(3000);
    CHECK(!sim_forced && is.tripped == ISENSE_TRIP_NONE && out_now[0] != 0u, "r7retry: the short gone: the light back "
          "(%u)", out_now[0]);
    /* CONFIG 24: 2 retries 3 s apart, persisted (the light dark meanwhile: the settings write waits for it) */
    cmd_set(0, 0, 0, 0);
    cmd_config(ULTRA_CFG_RETRY, 0x0302u);
    run_ms(500);
    CHECK(is.retry_max == 2u && is.retry_s == 3u && sext.reserved[2] == (2u ^ 0xF5u) && sext.reserved[3] == (3u ^ 0xFEu),
          "r7retry: CONFIG 24 = 2 x 3 s, in the settings extension (%u x %u)", is.retry_max, is.retry_s);
    run_ms(70000);                                       /* 60 s calm: the count starts again; the settings stored */
    {
        settings_t s0;
        store_ext_t e0;
        uint32_t seq;
        memset(&e0, 0, sizeof e0);
        (void)store_find_ext((const uint8_t *)SETTINGS_ADDR, &s0, &e0, &seq);
        printf("  stored: dirty %u, extension bytes %02X %02X\n", settings_dirty, e0.reserved[2], e0.reserved[3]);
    }
    CHECK(!settings_dirty, "r7retry: the policy stored (settings written)");
    cmd_set(0, 32768, 0, 0);
    run_ms(1000);
    short_ticks = 0xFFFFFFFFu;
    n = r7_trips(30000u, gap, 32u);
    for (k = 0, ok = 1; k + 1u < n && k < 32u; k++) {
        ok &= gap[k] >= 2995u && gap[k] <= 3010u;
    }
    CHECK(n == 3u && ok && is.latched, "r7retry: CONFIG 24's policy: %u retries 3 s apart, then off", n - 1u);
    short_ticks = 0;
}

/* r7retryws (B7): the policy by WS config frames (0x15 count, 0x16 seconds): each acknowledged, stored, in effect */
static void sc_r7retryws(void)
{
    static const uint8_t col[5] = { 0x40, 0x80, 0xC0, 0x20, 0x10 };
    uint8_t raw[10];
    printf("soak r7retryws: the retry policy by WS config frames\n");
    boot(0, 0, 0);
    r7_ws(col, 100);
    config_build(CMD_RETRY_COUNT, 3, 0, raw);
    r7_ws(raw, 3);
    r7_ws(col, 200);
    config_build(CMD_RETRY_S, 2, 0, raw);
    r7_ws(raw, 3);
    r7_ws(col, 200);
    CHECK(is.retry_max == 3u && is.retry_s == 2u && sext.reserved[2] == (3u ^ 0xF5u) && sext.reserved[3] == (2u ^ 0xFEu)
          && stat.config_frames >= 2u, "r7retryws: 3 retries 2 s apart, in the settings extension (%u x %u)",
          is.retry_max, is.retry_s);
}

/* run `ms`, the largest output step between two ms (max over the five channels) */
static uint32_t r7_steps(uint32_t ms)
{
    uint16_t prev[5];
    uint32_t t, i, mx = 0;
    memcpy(prev, out_now, sizeof prev);
    for (t = 0; t < ms; t++) {
        run_ms(1);
        for (i = 0; i < 5u; i++) {
            uint32_t d = out_now[i] > prev[i] ? out_now[i] - prev[i] : prev[i] - out_now[i];
            mx = d > mx ? d : mx;
            prev[i] = out_now[i];
        }
    }
    return mx;
}

static void sc_r7loss(const char *how, const char *what)
{
    static uint8_t col[5] = { 0x80, 0x80, 0x80, 0x80, 0x80 }, dim[5] = { 0x33, 0x33, 0x33, 0x33, 0x33 };
    const int ws = !strcmp(how, "ws"), scene = strcmp(what, "none") != 0;
    uint8_t raw[10];
    uint32_t st1, st2, st3, t;
    uint16_t at_loss;
    printf("soak r7loss %s %s: signal loss \"scene\" (3)\n", ws ? "ws" : "ultra", scene ? "scene" : "none");
    boot(0, 0, 0);
    if (ws) {
        r7_ws(dim, 100);                                 /* 20 %: shown, stored as the scene (CMD_SCENE 1) */
        if (scene) {
            config_build(CMD_SCENE, 1, 0, raw);
            r7_ws(raw, 3);
            r7_ws(dim, 400);
        }
        config_build(CMD_SIGNAL_LOSS, 3, 0, raw);
        r7_ws(raw, 3);
        r7_ws(col, 600);                                 /* 50 %, settings stored meanwhile */
    } else {
        host_on = 1;
        cmd_def(0, 0, 5, 1000);
        cmd_set(0, 13107, 0, 0);                         /* 20 % */
        run_ms(2000);
        if (scene) {
            cmd_scene(0, 13107);
            run_ms(70000);                               /* the scene stored (SCENE_SAVE_MS) */
        }
        cmd_config(ULTRA_CFG_LOSS, 3);
        cmd_set(0, 32768, 0, 0);                         /* 50 % */
        run_ms(3000);
    }
    printf("  scene set: mask %02X, %u; settings loss %u (eff %u); out %u\n", scene_want.mask, scene_want.v[0],
           set.signal_loss, eff.signal_loss, out_now[0]);
    CHECK(eff.signal_loss == 3u && (scene ? scene_want.mask != 0u : scene_want.mask == 0u), "r7loss: mode 3, the "
          "scene %s", scene ? "set" : "not set");
    at_loss = out_now[0];
    host_on = 0;                                         /* the controller stops (WS: no more frames) */
    st1 = r7_steps(ws ? 1900u : 2900u);                  /* before the timeout: the light as it was */
    CHECK(out_now[0] + 2u >= at_loss && out_now[0] <= at_loss + 2u && !signal_lost, "r7loss: before the timeout: as it "
          "was (%u)", out_now[0]);
    st2 = r7_steps(3000u);                               /* the loss and its 1 s fade */
    printf("  after the loss: out %u (was %u), lost %u, scene in %02X hold %02X; largest step %u\n", out_now[0],
           at_loss, signal_lost, srun.in, srun.hold, st2);
    if (scene) {
        t = scene_want.v[0];
        CHECK(out_now[0] + 300u >= t && out_now[0] <= t + 300u, "r7loss: faded to the scene (%u, scene %u)",
              out_now[0], t);
    } else {
        CHECK(out_now[0] + 2u >= at_loss && out_now[0] <= at_loss + 2u, "r7loss: nothing set: holds (%u)", out_now[0]);
    }
    if (ws) {                                            /* the controller back */
        for (t = 0; t < 150u; t++) {
            wire_frame(col, 40u, sim_ns + 7000u);
            st3 = r7_steps(20u);
            st2 = st3 > st2 ? st3 : st2;
        }
    } else {
        host_on = 1;
        host_next = sim_ticks;
        cmd_set(0, 32768, 0, 0);                         /* (the hub's refresh: its light's state again) */
        st3 = r7_steps(5000u);
        st2 = st3 > st2 ? st3 : st2;
    }
    printf("  back: out %u, largest step %u (before %u)\n", out_now[0], st2, st1);
    CHECK(out_now[0] + 300u >= at_loss && out_now[0] <= at_loss + 300u && !signal_lost, "r7loss: the controller back: "
          "its value again (%u)", out_now[0]);
    CHECK(st1 < 200u && st2 < 200u, "r7loss: no output step over 200 counts in a ms (%u, %u)", st1, st2);
}

static void sc_r7fault(void)
{
    life_rec_t r;
    life_pos_t pos;
    uint32_t w0, w1, w2, found;
    printf("soak r7fault: the fault memory (the last trip, dip, abnormal reset) in the lifetime record\n");
    bcrumb.magic = BC_MAGIC;                             /* the last run: an exception, then the watchdog's reset */
    bcrumb.up_s = 7320u;
    bcrumb.pc = 0x00004A2Eu;
    bcrumb.cause = 5u;
    soak_reset_flags = 0x20000000u;                      /* IWDGRSTF */
    boot(0, 0, 0);
    CHECK(en.lt.rst == (2u | 5u << 2) && en.lt.rst_h == 2u, "r7fault: the abnormal reset: an exception (cause 5) after "
          "2 h (%02X, %u h)", en.lt.rst, en.lt.rst_h);
    host_on = 1;
    cmd_def(0, 0, 5, 1000);
    cmd_set(0, 65535, 0, 0);
    run_ms(4000);
    expect_trips = 1;
    short_ticks = 3;                                     /* a short: its peak, the retry 1 s later */
    run_ms(3000);
    CHECK((en.lt.trip & 3u) == 1u && (en.lt.trip >> 2) != 0u && en.lt.trip_h == en.lt.lit_s / 3600u, "r7fault: the "
          "trip: a short, %u x 0.5 A, at %u h lit", en.lt.trip >> 2, en.lt.trip_h);
    w0 = W_LIFE();
    supply_mv = 19000u;                                  /* a 40 ms dip */
    run_ms(40);
    supply_mv = 24000u;
    w1 = W_LIFE();
    run_ms(2000);
    CHECK(en.lt.dip_pct >= 75u && en.lt.dip_pct <= 82u && en.lt.dip_cs >= 3u && en.lt.dip_cs <= 5u && en.lt.dip_a != 0u,
          "r7fault: the dip: %u %%, %u0 ms, %u x 0.1 A", en.lt.dip_pct, en.lt.dip_cs, en.lt.dip_a);
    CHECK(W_LIFE() == w1, "r7fault: the dip's end writes nothing (%u writes at the dip, its start's)", w1 - w0);
#if CONSOLE_FULL
    console("status");
    CHECK(strstr(con_out, "faults=trip short ") != NULL && strstr(con_out, ", reset fault 5 after 2 h") != NULL,
          "r7fault: `faults=` in the status");
#endif
    run_ms(BOARD == BOARD_DP_RX ? 90000u : 30000u);      /* lit: > 1 Wh more (166 W; the receiver's 51 W: 90 s) */
    w1 = W_LIFE();
    cmd_set(0, 0, 0, 0);                                 /* dark: the record's regular write carries the dip */
    run_ms(60000);
    w2 = W_LIFE();
    found = life_find((const uint8_t *)LIFE_PAGE0_ADDR, (const uint8_t *)LIFE_PAGE1_ADDR, &r, &pos);
    printf("  writes: %u at the dip, %u after dark; stored trip %02X / %u h, dip %u %% %u cs %u, reset %02X / %u h\n",
           w1 - w0, w2 - w1, r.trip, r.trip_h, r.dip_pct, r.dip_cs, r.dip_a, r.rst, r.rst_h);
    CHECK(found && r.magic == LIFE_MAGIC && r.trip == en.lt.trip && r.trip_h == en.lt.trip_h &&
          r.dip_pct == en.lt.dip_pct && r.dip_cs == en.lt.dip_cs && r.dip_a == en.lt.dip_a && r.rst == en.lt.rst &&
          r.rst_h == en.lt.rst_h, "r7fault: the fault memory in the flash (format 2)");
    CHECK(w2 - w1 == 1u, "r7fault: one regular write after dark (> 1 Wh lit), as before (%u)", w2 - w1);
}

#if BOARD_RETURN
static void r7_diag0(rc_obs_t *o, const char *when)
{
    static const uint8_t a0[1] = { 0 };
    int pg = rc_ask(ULTRA_PAGE_DIAG, a0, 1, o);
    CHECK(pg == ULTRA_PAGE_DIAG, "r7pages: DIAG 0 answered %s (%d)", when, pg);
}
#endif

static void sc_r7pages(void)
{
#if BOARD_RETURN
    static const uint8_t a1[1] = { 1 }, a2[1] = { 2 };
    rc_obs_t o;
    const uint8_t *q = D(&o);
    uint32_t k, ok;
    int pg;
    printf("soak r7pages: the receiver's LIFETIME and DIAG pages (protocol 0.2.5)\n");
    bcrumb.magic = BC_MAGIC;                             /* the last run: an exception, then the watchdog's reset */
    bcrumb.up_s = 7320u;
    bcrumb.pc = 0x00004A2Eu;
    bcrumb.cause = 5u;
    soak_reset_flags = 0x20000000u;                      /* IWDGRSTF */
    soak_wfi = 1;                                        /* the WFI's model (as conout): a board's CPU load, not the */
    soak_busy = 288u;                                    /* soak's 100 % */
    rc_boot_locked();
    cmd_def(0, 0, 5, 1000);
    cmd_set(0, 32768, 0, 0);
    run_ms(3000);

    /* IDENTITY: the new features */
    pg = rc_ask(ULTRA_PAGE_IDENTITY, 0, 0, &o);
    k = ULTRA_FEAT_LIFETIME | ULTRA_FEAT_DIAG | ULTRA_FEAT_RETRY | ULTRA_FEAT_LOSS_SCENE;
    CHECK(pg == ULTRA_PAGE_IDENTITY && (ultra_get32(q + ULTRA_ID_FEATURES) & k) == k, "r7pages: IDENTITY's features "
          "(%08X)", ultra_get32(q + ULTRA_ID_FEATURES));

    /* LIFETIME: the record as it is now (set to known values) */
    en.lt.on_s = 36000000u + 100u;                       /* 10,000 h */
    en.lt.lit_s = 18000000u + 200u;                      /* 5,000 h */
    en.lt.ups = 123456u;
    en.lt.trips[0] = 7u;
    en.lt.trips[1] = 9u;
    en.lt.dips = 0x123456u;
    en.lt.peak_t = 150u;
    en.lt.peak_a = 88u;
    en.lt.updates = 12u;
    en.lt.writes = 0x01ABCDEFu;
    pg = rc_ask(ULTRA_PAGE_LIFETIME, 0, 0, &o);
    printf("  LIFETIME %u / %u x 0.1 h, ups %u, trips %u / %u, dips %06X, peaks %u / %u, wd %u, updates %u, records "
           "%06X\n", ultra_get24(q + ULTRA_LT_POWERED), ultra_get24(q + ULTRA_LT_LIT), ultra_get24(q + ULTRA_LT_POWER_UPS),
           ultra_get16(q + ULTRA_LT_OVERLOAD_TRIPS), ultra_get16(q + ULTRA_LT_SHORT_TRIPS), ultra_get24(q + ULTRA_LT_DIPS),
           q[ULTRA_LT_TEMP_PEAK], q[ULTRA_LT_CURRENT_PEAK], q[ULTRA_LT_WATCHDOG], ultra_get16(q + ULTRA_LT_UPDATES),
           ultra_get24(q + ULTRA_LT_RECORDS));
    CHECK(pg == ULTRA_PAGE_LIFETIME && ultra_get24(q + ULTRA_LT_POWERED) == 100000u &&
          ultra_get24(q + ULTRA_LT_LIT) == 50000u && ultra_get24(q + ULTRA_LT_POWER_UPS) == 123456u &&
          ultra_get16(q + ULTRA_LT_OVERLOAD_TRIPS) == 7u && ultra_get16(q + ULTRA_LT_SHORT_TRIPS) == 9u &&
          ultra_get24(q + ULTRA_LT_DIPS) == 0x123456u && q[ULTRA_LT_TEMP_PEAK] == 150u && q[ULTRA_LT_CURRENT_PEAK] == 88u &&
          q[ULTRA_LT_WATCHDOG] == en.lt.wd && en.lt.wd >= 1u && ultra_get16(q + ULTRA_LT_UPDATES) == 12u &&
          ultra_get24(q + ULTRA_LT_RECORDS) == 0xABCDEFu, "r7pages: LIFETIME's fields");

    /* DIAG 0: the running board */
    r7_diag0(&o, "after boot");
    printf("  DIAG 0: reset %02X, breadcrumbs %u / %04X / %u s, cpu %u / %u, overruns %u %u %u, z %u, flash %u / %02X / "
           "%u, C10 %02X %02X\n", q[ULTRA_DG_RESET], q[ULTRA_DG_BC_CAUSE], ultra_get16(q + ULTRA_DG_BC_PC),
           ultra_get24(q + ULTRA_DG_BC_UPTIME), ultra_get16(q + ULTRA_DG_CPU), ultra_get16(q + ULTRA_DG_CPU_PEAK),
           ultra_get16(q + ULTRA_DG_OVR_CAPTURES), ultra_get16(q + ULTRA_DG_OVR_FRAME_ENDS),
           ultra_get16(q + ULTRA_DG_OVR_LATE_MS), ultra_get16(q + ULTRA_DG_SUPPLY_Z), q[ULTRA_DG_FLASH_ERRS],
           q[ULTRA_DG_FLASH_PENDING], q[ULTRA_DG_SAVE_WAIT], q[ULTRA_DG_C10_LOAD], q[ULTRA_DG_C10_PEAK]);
    CHECK(q[ULTRA_DG_RESET] == ULTRA_ST_RST_WATCHDOG && q[ULTRA_DG_BC_CAUSE] == 5u &&
          ultra_get16(q + ULTRA_DG_BC_PC) == 0x4A2Eu && ultra_get24(q + ULTRA_DG_BC_UPTIME) == 7320u,
          "r7pages: DIAG 0: the reset, the breadcrumbs");
    CHECK(ultra_get16(q + ULTRA_DG_CPU) > 0u && ultra_get16(q + ULTRA_DG_CPU) < 1000u &&
          ultra_get16(q + ULTRA_DG_CPU_PEAK) >= ultra_get16(q + ULTRA_DG_CPU), "r7pages: DIAG 0: the CPU load");
    for (k = 0, ok = 1; k < 3u; k++) {
        ok &= ultra_get16(q + ULTRA_DG_OVR_CAPTURES + 2u * k) == (ovr[k] > 0xFFFFu ? 0xFFFFu : ovr[k]);
    }
    CHECK(ok && ultra_get16(q + ULTRA_DG_SUPPLY_Z) == (vdip_z_mohm(&vdip) > 0xFFFFu ? 0xFFFFu : vdip_z_mohm(&vdip)) &&
          q[ULTRA_DG_FLASH_ERRS] == 0u && q[ULTRA_DG_FLASH_PENDING] == 0u && q[ULTRA_DG_SAVE_WAIT] == ULTRA_DG_SW_NONE &&
          q[ULTRA_DG_C10_LOAD] == 0xFFu && q[ULTRA_DG_C10_PEAK] == 0xFFu, "r7pages: DIAG 0: the overruns, the "
          "impedance, the flash, nothing pending, C10 not measured");
    cmd_config(ULTRA_CFG_RETRY, 0x0302u);                /* a persisted setting while lit at 50 % */
    run_ms(300);
    r7_diag0(&o, "after CONFIG 24");
    CHECK(q[ULTRA_DG_FLASH_PENDING] == ULTRA_DG_FP_SETTINGS && q[ULTRA_DG_SAVE_WAIT] == ULTRA_DG_SW_DEBOUNCE,
          "r7pages: DIAG 0: the settings pending, the debounce (%02X %u)", q[ULTRA_DG_FLASH_PENDING],
          q[ULTRA_DG_SAVE_WAIT]);
    run_ms(3000);
    r7_diag0(&o, "3 s later");
    CHECK(q[ULTRA_DG_FLASH_PENDING] == ULTRA_DG_FP_SETTINGS && q[ULTRA_DG_SAVE_WAIT] == ULTRA_DG_SW_DARK &&
          out_now[0] == 32768u, "r7pages: DIAG 0: lit, the hour's write while bright not earned yet: waiting for dark "
          "(%02X %u)", q[ULTRA_DG_FLASH_PENDING], q[ULTRA_DG_SAVE_WAIT]);
    cmd_set(0, 0, 0, 0);                                 /* dark: stored */
    run_ms(2000);
    r7_diag0(&o, "dark");
    CHECK(q[ULTRA_DG_FLASH_PENDING] == 0u && q[ULTRA_DG_SAVE_WAIT] == ULTRA_DG_SW_NONE && !settings_dirty,
          "r7pages: DIAG 0: dark: stored, nothing pending (%02X %u)", q[ULTRA_DG_FLASH_PENDING], q[ULTRA_DG_SAVE_WAIT]);
    cmd_set(0, 32768, 0, 0);
    run_ms(1000);

    /* DIAG 1: the fault memory after a short and a dip; the trend not known */
    expect_trips = 1;
    short_ticks = 3;
    run_ms(3000);
    supply_mv = 19000u;
    run_ms(40);
    supply_mv = 24000u;
    run_ms(2000);
    pg = rc_ask(ULTRA_PAGE_DIAG, a1, 1, &o);
    printf("  DIAG 1: trip %02X at %u h, dip %u %% %u x 10 ms %u x 0.1 A, reset %02X after %u h; trend %02X.., %04X, "
           "%u\n", q[ULTRA_DG_TRIP], ultra_get16(q + ULTRA_DG_TRIP_LIT_H), q[ULTRA_DG_DIP_PCT], q[ULTRA_DG_DIP_10MS],
           q[ULTRA_DG_DIP_CURRENT], q[ULTRA_DG_RST], q[ULTRA_DG_RST_UP_H], q[ULTRA_DG_TREND],
           ultra_get16(q + ULTRA_DG_TREND_LIT_H), q[ULTRA_DG_TREND_TESTS]);
    CHECK(pg == ULTRA_PAGE_DIAG && q[ULTRA_DG_TRIP] == en.lt.trip && (en.lt.trip & 3u) == 1u &&
          ultra_get16(q + ULTRA_DG_TRIP_LIT_H) == en.lt.trip_h && en.lt.trip_h == 5000u &&
          q[ULTRA_DG_DIP_PCT] == en.lt.dip_pct && en.lt.dip_pct != 0u && q[ULTRA_DG_DIP_10MS] == en.lt.dip_cs &&
          q[ULTRA_DG_DIP_CURRENT] == en.lt.dip_a && q[ULTRA_DG_RST] == (2u | 5u << 2) && q[ULTRA_DG_RST_UP_H] == 2u,
          "r7pages: DIAG 1: the fault memory");
    for (k = 0, ok = 1; k < 5u; k++) {
        ok &= q[ULTRA_DG_TREND + k] == 0x80u;
    }
    CHECK(ok && ultra_get16(q + ULTRA_DG_TREND_LIT_H) == 0xFFFFu && q[ULTRA_DG_TREND_TESTS] == 0u,
          "r7pages: DIAG 1: the trend not known (no history on the receiver)");
    pg = rc_ask(ULTRA_PAGE_DIAG, a2, 1, &o);
    CHECK(pg == ULTRA_PAGE_UNSUPPORTED, "r7pages: DIAG 2: UNSUPPORTED (%d)", pg);
    short_ticks = 0;
#else
    printf("soak r7pages: not a receiver (BOARD_RETURN 0): no pages\n");
#endif
}

static void sc_r7hot(void)
{
    uint32_t k, ev = 0, t, dimmed = 0, d9 = 0;
    uint16_t out0;
    printf("soak r7hot: the die temperature warning (B6)\n");
    boot(0, 0, 0);
    host_on = 1;
    cmd_def(0, 0, 5, 1000);
    cmd_set(0, 32768, 0, 0);
    run_ms(5000);
    out0 = out_now[0];                                   /* (as set: the receiver's 2.25 A limit may hold it lower) */
    CHECK(!too_hot && !(hl.flags & ULTRA_HE_F_HOT) && mt.c10 < 400, "r7hot: ~30 C: not hot (%d)", mt.c10);
    soak_ts = 1340u;                                     /* ~99 C */
    for (t = 0; t < 30000u; t++) {
        uint8_t was = too_hot;
        run_ms(1);
        dimmed += out_now[0] != out0 && !(ack.active && !ack.led_only);
        d9 += !was && too_hot && ack.active && ack.led_only;   /* D9's code from the event on */
    }
    for (k = 0; k < EVLOG_N; k++) {
        ev += evl.e[k].type == EVL_HOT;
    }
    printf("  hot: %d (0.1 C), too_hot %u, HEALTH flags %02X, events %u, D9 at the event %u, outputs dimmed %u ms\n",
           mt.c10, too_hot, hl.flags, ev, d9, dimmed);
    CHECK(too_hot && (hl.flags & ULTRA_HE_F_HOT) && ev == 1u && d9 == 1u && dimmed == 0u, "r7hot: over 95 C: the event "
          "once, D9's code, the flag, nothing dimmed");
    soak_ts = 1395u;                                     /* ~88 C: still over 85 (the hysteresis) */
    run_ms(30000);
    CHECK(too_hot, "r7hot: 88 C: still hot (the hysteresis) (%d)", mt.c10);
    soak_ts = 1710u;
    run_ms(30000);
    for (k = 0, ev = 0; k < EVLOG_N; k++) {
        ev += evl.e[k].type == EVL_HOT;
    }
    CHECK(!too_hot && !(hl.flags & ULTRA_HE_F_HOT) && ev == 1u && out_now[0] == out0, "r7hot: cool again: the flag "
          "off, no second event (%u)", ev);
}

static void sc_r7b3(void)
{
    uint32_t t, d9 = 0, led0, n6 = 0;
    printf("soak r7b3: D9's codes 6 / 7 when the running model finds them (B3, Quindor's option b)\n");
    boot(0, 0, 0);
    host_on = 1;
    cmd_def(0, 0, 5, 1000);
    cmd_set(0, 0, 0, 0);                                 /* dark */
    run_ms(5000);
    led0 = sim_led_on;
    soak_leak_ma = 600u;                                 /* current with every output off: a FET stuck on */
    for (t = 0; t < 15000u; t++) {
        uint8_t was = (uint8_t)(hl.flags & ULTRA_HE_F_DARK);
        run_ms(1);
        n6 += !was && (hl.flags & ULTRA_HE_F_DARK) && ack.active && ack.led_only;
    }
    printf("  dark current: HEALTH flags %02X, D9's code at its rise %u, D9 on %u times\n", hl.flags, n6,
           sim_led_on - led0);
    CHECK((hl.flags & ULTRA_HE_F_DARK) && n6 == 1u && sim_led_on > led0, "r7b3: current while dark: D9's code 6 once");
    soak_leak_ma = 0;
    run_ms(15000);
    cmd_set(0, 20000, 0, 0);
    run_ms(5000);
    hl.flags |= HEALTH_F_CAL;                            /* a calibration suggested (the model drifted) */
    for (t = 0; t < 1100u; t++) {
        run_ms(1);
        d9 += ack.active && ack.led_only;
    }
    CHECK(d9 > 0u && out_now[0] != 0u, "r7b3: a calibration suggested: D9's code 7 (D9 only, the outputs as set)");
    run_ms(20000);
    led0 = sim_led_on;
    run_ms(9000);
    CHECK(sim_led_on == led0 && !ack.active, "r7b3: once, then D9 dark again (no heartbeat: parked) (%u)",
          sim_led_on - led0);
}

static void sc_r7why(void)
{
#if BOARD_RETURN
    static const uint8_t a0[1] = { 0 };
    rc_obs_t o;
    const uint8_t *q = D(&o);
    uint32_t why, p1, p2;
    int pg;
    printf("soak r7why: STATUS's reason, the receiver's ratings, ENERGY's V_B and peak (protocol 0.2.5)\n");
#define WHY() (rc_ask(ULTRA_PAGE_STATUS, 0, 0, &o) == ULTRA_PAGE_STATUS ? (uint32_t)(q[ULTRA_ST_STATE] >> 4) : 99u)
    rc_boot_locked();
    cmd_def(0, 0, 5, 1000);
    cmd_set(0, 32768, 0, 0);
    run_ms(60);
    why = WHY();
    printf("  60 ms after the first light: why %u (soft start %u)\n", why, ss.state == SS_RUN);
    CHECK(why == (ss.state == SS_RUN ? ULTRA_ST_WHY_SOFT_START : ULTRA_ST_WHY_NONE), "r7why: the soft start (%u)", why);
    run_ms(3000);
    why = WHY();
    printf("  3 s: why %u, soft start %u since %u ms, trips %u, out %u, i %d mA, limiting %u\n", why, ss.state,
           ms_now - ss.t0, is.trips, out_now[0], is.i_avg, is.limiting);
    CHECK(why == ULTRA_ST_WHY_NONE, "r7why: lit as set: none (%u)", why);
    pg = rc_ask(ULTRA_PAGE_POWER, 0, 0, &o);
    CHECK(pg == ULTRA_PAGE_POWER && ultra_get16(q + ULTRA_PW_LIMIT_MA) == 2250u, "r7why: POWER's limit: the link's "
          "2250 mA, bus side (%u)", ultra_get16(q + ULTRA_PW_LIMIT_MA));
    pg = rc_ask(ULTRA_PAGE_CALIBRATION, 0, 0, &o);
    CHECK(pg == ULTRA_PAGE_CALIBRATION && ultra_get16(q + 21) == 2250u, "r7why: CALIBRATION's limit 2250 mA (%u)",
          ultra_get16(q + 21));
    pg = rc_ask(ULTRA_PAGE_ENERGY, 0, 0, &o);
    p1 = ultra_get16(q + ULTRA_EN_PEAK_MA);
    printf("  ENERGY: V_B %u x 10 mV (aux %u mV), peak %u mA (now %d)\n", ultra_get16(q + ULTRA_EN_VB), vs_aux.mv, p1,
           is.i_avg);
    CHECK(pg == ULTRA_PAGE_ENERGY && vs_aux.valid && ultra_get16(q + ULTRA_EN_VB) == vs_aux.mv / 10u && p1 > 0u &&
          p1 >= (uint32_t)is.i_avg, "r7why: ENERGY: V_B, the peak");
    cmd_set(0, 6554, 0, 0);                              /* 10 %: less current */
    run_ms(2000);
    pg = rc_ask(ULTRA_PAGE_ENERGY, 0, 0, &o);
    p2 = ultra_get16(q + ULTRA_EN_PEAK_MA);
    run_ms(2000);
    pg = rc_ask(ULTRA_PAGE_ENERGY, 0, 0, &o);
    printf("  peaks: %u, then %u (the dimming in its window), then %u\n", p1, p2, ultra_get16(q + ULTRA_EN_PEAK_MA));
    CHECK(ultra_get16(q + ULTRA_EN_PEAK_MA) < p1 && ultra_get16(q + ULTRA_EN_PEAK_MA) > 0u, "r7why: the peak starts "
          "again after each answer");
    cmd_set(0, 32768, 0, 0);
    run_ms(2000);
    expect_trips = 1;
    short_ticks = 3;
    run_ms(100);
    why = WHY();
    CHECK(why == ULTRA_ST_WHY_SHORT, "r7why: off after a short (%u)", why);
    short_ticks = 0;
    run_ms(3000);
    why = WHY();
    CHECK(why == ULTRA_ST_WHY_NONE, "r7why: back (%u)", why);
    supply_mv = 19000u;
    run_ms(40);
    supply_mv = 24000u;
    run_ms(2000);
    why = WHY();
    CHECK(why == ULTRA_ST_WHY_SUPPLY, "r7why: a supply dip 2 s ago (%u)", why);
    run_ms(31000);
    why = WHY();
    CHECK(why == ULTRA_ST_WHY_NONE, "r7why: 31 s later: none (%u)", why);
    cmd_identify(ULTRA_IDENT_CAL, 0);
    run_ms(200);
    why = WHY();
    CHECK(why == ULTRA_ST_WHY_CALIBRATING, "r7why: a calibration test (%u)", why);
    run_ms(5000);
    (void)a0;
#undef WHY
#else
    printf("soak r7why: not a receiver (BOARD_RETURN 0): no pages\n");
#endif
}

/* the console's output from here on */
static void r7_con_clear(void)
{
    con_out_n = 0;
    con_out[0] = 0;
}

static void sc_r7con(const char *how)
{
    static const uint8_t col[5] = { 0x40, 0x80, 0xC0, 0x20, 0x10 };
    const char *v;
    printf("soak r7con %s: the live line's link%s\n", how, strcmp(how, "ws") ? ", the status's settings and save "
           "state" : "");
    boot(0, 0, 0);
#if CONSOLE_FULL
    console("watch");
#endif
    if (!strcmp(how, "ws")) {                            /* a WS2805 controller */
        r7_ws(col, 150);
        r7_con_clear();
        r7_ws(col, 110);
        CHECK(strstr(con_out, " WS2805 50fps  ") != NULL, "r7con: WS2805 (%.40s)", con_out);
        return;
    }
    r7_con_clear();
    run_ms(2500);
    CHECK(strstr(con_out, " no signal  ") != NULL && strstr(con_out, "fps") == NULL, "r7con: no signal (%.40s)",
          con_out);
    host_on = 1;
    cmd_def(0, 0, 5, 1000);
    cmd_set(0, 32768, 0, 0);
    run_ms(3000);
    r7_con_clear();
    run_ms(2100);
    CHECK(strstr(con_out, " Ultra 50fps  ") != NULL && strstr(con_out, "no signal") == NULL, "r7con: Ultra (%.40s)",
          con_out);
#if CONSOLE_FULL
    console("x");                                        /* (any key stops `watch`) */
    console("status");
    v = strstr(con_out, "settings=");
    printf("  %.*s\n", v ? (int)strcspn(v, "\r") : 0, v ? v : "");
    CHECK(v && strncmp(v, "settings=protocol 0, preset ", 28) == 0 && strstr(v, ", dither 0, ") != NULL &&
          strstr(v, "*") == NULL && strstr(v, "softstart 1, retry 10 x 1 s\r\nsave=ok\r\n") != NULL,
          "r7con: the stored settings, nothing overridden, nothing to save");
    cmd_config(ULTRA_CFG_DITHER, 2);
    run_ms(200);
    console("status");
    v = strstr(con_out, "settings=");
    CHECK(v && strstr(v, ", dither 2*, ") != NULL && strstr(v, "save=ok\r\n") != NULL && set.dither_mode == 0u,
          "r7con: an Ultra CONFIG over the stored setting: '*', nothing to save (%.120s)", v ? v : "");
    console("set smooth 0");
    console("status");
    v = strstr(con_out, "save=");
    CHECK(v && strncmp(v, "save=pending: debounce\r\n", 24) == 0 && strstr(con_out, ", smooth 0, ") != NULL,
          "r7con: a `set`: pending, the debounce (%.40s)", v ? v : "");
    run_ms(3000);
    console("status");
    v = strstr(con_out, "save=");
    CHECK(v && strncmp(v, "save=ok\r\n", 9) == 0 && !settings_dirty && flash_bright.tokens == 0u, "r7con: lit at 50 %%: "
          "stored after the debounce, the hour's write while bright used (%.40s)", v ? v : "");
    cmd_config(ULTRA_CFG_RETRY, 0x0302u);                /* a persisted CONFIG (no acknowledgement on the outputs) */
    run_ms(3000);
    console("status");
    v = strstr(con_out, "save=");
    CHECK(v && strncmp(v, "save=pending: waiting for dark\r\n", 32) == 0 && settings_dirty && out_now[0] == 32768u &&
          strstr(con_out, "retry 2 x 3 s\r\n") != NULL, "r7con: CONFIG 24 while lit: pending, waiting for dark (%.40s)",
          v ? v : "");
    cmd_set(0, 0, 0, 0);                                 /* dark: stored */
    run_ms(5000);
    console("status");
    v = strstr(con_out, "save=");
    CHECK(v && strncmp(v, "save=ok\r\n", 9) == 0 && !settings_dirty, "r7con: dark: stored, ok (%.40s)", v ? v : "");
#else
    (void)v;
#endif
}

/* ===================================================================================================== r7upd */
typedef struct {
    uint16_t before[5];
    uint32_t act, strip, moved;                          /* ms: an acknowledgement on, on the strip, an output moved */
} r7u_t;

static void r7u_ms(r7u_t *m, uint32_t n)
{
    uint32_t i;
    while (n--) {
        run_ms(1);
        m->act += ack.active;
        m->strip += ack.active && !ack.led_only;
        for (i = 0; i < 5u; i++) {
            m->moved += out_now[i] != m->before[i];
        }
    }
}

static void r7u_frame(r7u_t *m, uint8_t type, uint8_t session, uint32_t idx, const uint8_t *data32)
{
    uint8_t f[UPD_FRAME_BYTES];
    uint64_t t0;
    while (wf_on) {
        run_ticks(1);
    }
    t0 = sim_ns + 7000u;
    upd_build_frame(f, type, session, idx, 0x11, 0, data32, host_us(t0 + 384u * 1250u));
    wire_frame(f, UPD_FRAME_BITS, t0);
    r7u_ms(m, 20);
}

/* one session; end: its END sent (else missed: the result 2 s after the last UPD frame) */
static void r7u_session(uint8_t fb, const char *what, uint8_t hw_id, uint8_t session, int end, uint32_t want,
                        uint32_t len_ms)
{
    uint8_t d[64];
    r7u_t m;
    uint32_t led0, k, seen0 = upd.sessions_seen;
    host_on = 1;
    host_next = sim_ticks;
    run_ms(3000);
    host_on = 0;
    while (wf_on) {
        run_ticks(1);
    }
    run_ms(10);
    CHECK(!ack.active && out_now[0] != 0u && !upd_app_transparent(&upd), "r7upd %s: lit, nothing showing (ack %u)",
          what, ack.active);
    memset(&m, 0, sizeof m);
    memcpy(m.before, out_now, sizeof m.before);
    led0 = sim_led_on;
    memset(d, 0, sizeof d);                              /* the descriptor (test_robust_fw.c's mk_desc) */
    memcpy(d, "QDU5", 4);
    d[UPD_D_DESC_VERSION] = UPD_DESC_VERSION;
    d[UPD_D_HW_ID] = hw_id;
    upd_put16(d + UPD_D_HW_REV_MASK, (uint16_t)(1u << upd.info.hw_rev));
    upd_put32(d + UPD_D_FW_VERSION, upd.info.version);
    upd_put32(d + UPD_D_BODY_LEN, 30548u);
    upd_put32(d + UPD_D_BODY_CRC, 0x12345678u);
    upd_put32(d + UPD_D_LOAD_ADDR, UPD_APP_BODY);
    upd_put32(d + UPD_D_ENTRY, UPD_APP_ENTRY);
    d[UPD_D_MIN_BL] = 1;
    upd_put32(d + UPD_D_CRC, upd_crc32_words(0xFFFFFFFFu, d, 15u));
    for (k = 0; k < 3u; k++) {
        r7u_frame(&m, UPD_PING, session, 0, NULL);
    }
    r7u_frame(&m, UPD_ANNOUNCE, session, 0, d);
    r7u_frame(&m, UPD_ANNOUNCE, session, 1, d + 32);
    CHECK(upd_app_transparent(&upd) && upd.sessions_seen == seen0 + 1u && upd.decided && upd.result == want,
          "r7upd %s: the session decided (transparent %u, seen %u, result %u, want %u)", what,
          upd_app_transparent(&upd), upd.sessions_seen - seen0, upd.result, want);
    if (end) {
        r7u_frame(&m, UPD_END, session, 0, NULL);
        CHECK(ack.active && upd.result == 0u, "r7upd %s: shown at the END (ack %u)", what, ack.active);
        host_on = 1;                                     /* the host's frames again at once */
        host_next = sim_ticks;
    } else {
        r7u_ms(&m, 1970);                                /* 1990 ms after the last UPD frame */
        CHECK(!ack.active && upd.result == want, "r7upd %s: nothing shown before the idle time (ack %u)", what,
              ack.active);
        r7u_ms(&m, 40);
        CHECK(ack.active && upd.result == 0u && !upd_app_transparent(&upd), "r7upd %s: shown 2 s after the last UPD "
              "frame (ack %u)", what, ack.active);
        host_on = 1;
        host_next = sim_ticks;
    }
    r7u_ms(&m, 3000);
    printf("  %s, feedback %u: the acknowledgement %u ms (on the strip %u), D9 on %u times, output-ms off their value "
           "%u\n", what, fb, m.act, m.strip, sim_led_on - led0, m.moved);
    CHECK(m.act + 20u >= len_ms && m.act <= len_ms + 20u && sim_led_on == led0 + 1u, "r7upd %s: D9's pattern (%u ms, "
          "want %u; D9 on %u)", what, m.act, len_ms, sim_led_on - led0);
    CHECK(m.strip == 0u && m.moved == 0u && !ack.active, "r7upd %s, feedback %u: D9 only, the light kept (strip %u ms, "
          "moved %u)", what, fb, m.strip, m.moved);
}

static void sc_r7upd(uint8_t fb)
{
    printf("soak r7upd %u: an update session's SAME / NACK on D9 only (feedback %u)\n", fb, fb);
    boot(0, 0, 0);
    upd.info.bl_version = 2;                             /* as if behind a bootloader, this version's descriptor valid */
    upd.info.hw_id = 0x21;
    upd.info.hw_rev = 3;
    upd.info.desc_ok = 1;
    upd.info.version = 0x00000609u;
    host_on = 1;
    cmd_def(0, 0, 5, 1000);
    cmd_set(0, 32768, 0, 0);
    if (fb != 0u) {
        cmd_config(ULTRA_CFG_FEEDBACK, fb);
    }
    run_ms(5000);
    CHECK(ultra_on && eff.feedback_ch == fb && out_now[0] != 0u, "r7upd: lit, feedback %u (%u)", fb, eff.feedback_ch);
    r7u_session(fb, "same version", 0x21, 0x51, 1, UPD_APP_SHOW_SAME, 1200u);
    r7u_session(fb, "same version, END missed", 0x21, 0x52, 0, UPD_APP_SHOW_SAME, 1200u);
    r7u_session(fb, "another hardware", 0x22, 0x53, 1, UPD_APP_SHOW_REJECTED, 2500u);
}

static int r7_scenario(const char *s, int argc, char **argv)
{
    if (!strcmp(s, "r7ack")) {
        sc_r7ack((uint8_t)(argc > 2 ? atoi(argv[2]) : 2));
    } else if (!strcmp(s, "r7upd")) {
        sc_r7upd((uint8_t)(argc > 2 ? atoi(argv[2]) : 0));
    } else if (!strcmp(s, "r7b3")) {
        sc_r7b3();
    } else if (!strcmp(s, "r7hot")) {
        sc_r7hot();
    } else if (!strcmp(s, "r7why")) {
        sc_r7why();
    } else if (!strcmp(s, "r7pages")) {
        sc_r7pages();
    } else if (!strcmp(s, "r7con")) {
        sc_r7con(argc > 2 ? argv[2] : "ultra");
    } else if (!strcmp(s, "r7fault")) {
        sc_r7fault();
    } else if (!strcmp(s, "r7loss")) {
        sc_r7loss(argc > 2 ? argv[2] : "ultra", argc > 3 ? argv[3] : "scene");
    } else if (!strcmp(s, "r7retryws")) {
        sc_r7retryws();
    } else if (!strcmp(s, "r7retry")) {
        sc_r7retry();
    } else if (!strcmp(s, "r7cal")) {
        sc_r7cal(argc > 2 ? argv[2] : "limit");
    } else {
        return 0;
    }
    return 1;
}
