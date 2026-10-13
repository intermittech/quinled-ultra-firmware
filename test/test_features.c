/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Host tests for the v0.2 features: dither + dim floor, smoothing / fades, gamma, settings records, console.
 * Included by test_main.c (shares CHECK / g_fail). */

/* ------------------------------------------------------------------------------------------------ dither */
static double avg_of(const uint16_t *p, uint32_t n)
{
    double a = 0;
    uint32_t i;
    for (i = 0; i < n; i++) a += p[i];
    return a / n;
}

static void test_dither(void)
{
    static uint16_t pat[PATTERN_MAX_LEN];
    dither_opts_t o = { DITHER_ON, 0, FLOOR_OFF, 0 };
    uint32_t p;

    printf("dither: every 16-bit level at every preset, table length as used (no floor)\n");
    for (p = 0; p < PWM_PRESET_COUNT; p++) {
        uint32_t steps = pwm_preset_steps[p], v;
        uint8_t len = pattern_len_for(steps), i;
        double maxerr = 0, prev = -1, f = (double)F_TIM_HZ / steps;
        int mono = 1, range = 1;
        for (v = 0; v <= 65535u; v++) {
            double avg, exact = (double)v * steps / 65535.0, err;
            dither_build((uint16_t)v, steps, len, &o, (uint8_t)(v % len), pat, 1);
            for (i = 0; i < len; i++) if (pat[i] > steps) range = 0;
            avg = avg_of(pat, len);
            err = fabs(avg - exact) * 65535.0 / steps;                 /* in 16-bit LSB */
            if (err > maxerr) maxerr = err;
            if (avg < prev) mono = 0;
            prev = avg;
        }
        printf("  preset %2u %8.1f Hz steps %5u table %2u: max error %.3f LSB16, slowest ripple %5.0f Hz%s%s\n",
               p, f, steps, len, maxerr, f / len, mono ? "" : "  NOT MONOTONIC", range ? "" : "  OUT OF RANGE");
        CHECK(maxerr <= 0.501, "preset %u error %.3f", p, maxerr);
        CHECK(mono && range, "preset %u monotonic/range", p);
    }
    {   /* modes: off = nearest native step, threshold switch has no jump */
        uint32_t steps = 5760;
        uint8_t len = pattern_len_for(steps), i;
        uint16_t thr = dither_threshold16(10), v;
        double jump = 0;
        for (v = (uint16_t)(thr - 200); v < thr + 200; v++) {
            double ex = (double)v * steps / 65535.0;
            o.mode = DITHER_OFF_LOW;
            o.threshold16 = thr;
            dither_build(v, steps, len, &o, 0, pat, 1);
            for (i = 0; i < len; i++) CHECK(v >= thr || pat[i] == pat[0], "below threshold must be constant");
            if (fabs(avg_of(pat, len) - ex) > jump) jump = fabs(avg_of(pat, len) - ex);
            o.mode = DITHER_OFF;
            dither_build(v, steps, len, &o, 0, pat, 1);
            CHECK(fabs(avg_of(pat, len) - ex) <= 0.5001, "off mode must be nearest step");
        }
        printf("  modes: threshold 1.0 %% = %u/65535, max deviation around it %.3f native steps (<= 0.5 = no jump)\n", thr, jump);
        CHECK(jump <= 0.5001, "threshold switch jump %.3f", jump);
    }
}

static void test_floor(void)
{
    static uint16_t pat[PATTERN_MAX_LEN];
    uint16_t F = dither_floor_counts(20);                 /* default 200 ns -> 29 counts */
    dither_opts_t o = { DITHER_ON, 0, FLOOR_SKIP, 0 };
    uint32_t p;

    o.floor = F;
    printf("dim floor: min pulse %u counts (%.0f ns), policy skip, every level at every preset\n", F, F / 0.144);
    CHECK(F == 29, "floor counts");
    for (p = 0; p < PWM_PRESET_COUNT; p++) {
        uint32_t steps = pwm_preset_steps[p], v, lowest = 0, worst_gap = 0;
        uint8_t len = pattern_len_for(steps), i;
        double f = (double)F_TIM_HZ / steps, maxerr_low = 0, maxerr_mid = 0, prev = -1;
        int ok_pulse = 1, mono = 1;
        for (v = 0; v <= 65535u; v++) {
            double exact = (double)v * steps / 65535.0, avg;
            uint32_t gap = 0, maxgap = 0, any = 0;
            dither_build((uint16_t)v, steps, len, &o, (uint8_t)(v % len), pat, 1);
            for (i = 0; i < 2u * len; i++) {               /* twice round: gaps across the wrap */
                uint16_t x = pat[i % len];
                if (x != 0 && x < F) ok_pulse = 0;          /* every pulse at least the floor */
                if (x != steps && x > steps - F) ok_pulse = 0;   /* every off-time at least the floor */
                if (x) { any = 1; if (gap > maxgap) maxgap = gap; gap = 0; } else gap++;
            }
            avg = avg_of(pat, len);
            if (avg < prev - 1e-9) mono = 0;
            prev = avg;
            if (any && !lowest) lowest = v;
            if (any && maxgap + 1 > worst_gap) worst_gap = maxgap + 1;
            if (exact < F) {
                if (fabs(avg - exact) > maxerr_low) maxerr_low = fabs(avg - exact);
            } else if (exact <= steps - F) {
                double e = fabs(avg - exact) * 65535.0 / steps;
                if (e > maxerr_mid) maxerr_mid = e;
            }
        }
        printf("  preset %2u %8.1f Hz: lowest on %5.3f %% (= %u/65535), pulse repeat >= %5.0f Hz, err below floor %.2f counts, above %.3f LSB16%s%s\n",
               p, f, lowest * 100.0 / 65535.0, lowest, f / worst_gap, maxerr_low, maxerr_mid,
               ok_pulse ? "" : "  SHORT PULSE", mono ? "" : "  NOT MONOTONIC");
        CHECK(ok_pulse && mono, "preset %u pulses / monotonic", p);
        CHECK(maxerr_mid <= 0.501, "preset %u error above floor %.3f", p, maxerr_mid);
        CHECK(maxerr_low <= F / (2.0 * len) + 0.01, "preset %u error below floor %.2f", p, maxerr_low);
        CHECK(f / worst_gap >= 1500.0 || len == 1, "preset %u pulse repetition %.0f Hz", p, f / worst_gap);
    }
    {   /* clamp: 0 or one floor pulse every period below the floor */
        uint32_t steps = 5760, v;
        uint8_t len = pattern_len_for(steps), i;
        o.floor_policy = FLOOR_CLAMP;
        for (v = 1; v < 400; v++) {
            dither_build((uint16_t)v, steps, len, &o, 0, pat, 1);
            for (i = 0; i < len; i++) CHECK(pat[i] == 0 || pat[i] >= F, "clamp pulse");
        }
        o.floor_policy = FLOOR_OFF;
        dither_build(50, steps, len, &o, 0, pat, 1);
        CHECK(avg_of(pat, len) > 0 && avg_of(pat, len) < F, "floor off keeps short pulses");
    }
}

/* ------------------------------------------------------------------------------------------------ smooth */
static void test_smooth(void)
{
    smooth_t s;
    uint16_t a[5] = { 0, 0, 0, 0, 0 }, b[5] = { 65535, 1000, 30000, 257, 0 };
    uint32_t t = 1000, k;
    int lin = 1;

    printf("smoothing: frame-interval ramps, fast / sparse frames jump, fades\n");
    smooth_init(&s);
    for (k = 0; k < 10; k++, t += 400) smooth_frame(&s, (k & 1) ? b : a, t, 1);   /* 50 fps: 20 ms = 400 ticks */
    CHECK(s.interval >= 395 && s.interval <= 405, "interval estimate %u", s.interval);
    smooth_frame(&s, b, t, 1);                         /* a -> b over ~400 ticks */
    smooth_update(&s, t + 200);
    CHECK(abs((int)s.cur[0] - 32767) < 400 && abs((int)s.cur[2] - 15000) < 200, "midpoint %u %u", s.cur[0], s.cur[2]);
    {
        uint32_t prev = s.cur[0];
        for (k = 201; k <= 400; k++) {
            smooth_update(&s, t + k);
            if (s.cur[0] < prev) lin = 0;
            prev = s.cur[0];
        }
    }
    CHECK(lin && s.cur[0] == 65535 && s.cur[1] == 1000 && s.cur[4] == 0 && s.dur == 0, "ramp ends exactly on target");
    /* new frame mid-ramp starts from the current value: no jump */
    t += 1000;
    smooth_frame(&s, a, t, 1);
    smooth_update(&s, t + 100);
    k = s.cur[0];
    smooth_frame(&s, b, t + 100, 1);
    CHECK(s.cur[0] == k, "mid-ramp retarget keeps the output");
    /* fast frames (1 ms) and sparse frames (500 ms): no ramp */
    smooth_init(&s);
    for (k = 0, t = 0; k < 20; k++, t += 20) smooth_frame(&s, (k & 1) ? b : a, t, 1);
    CHECK(s.dur == 0 && s.cur[0] == 65535, "1 ms frames jump straight to the last frame");
    smooth_init(&s);
    for (k = 0, t = 0; k < 6; k++, t += 10000) smooth_frame(&s, (k & 1) ? b : a, t, 1);
    CHECK(s.dur == 0 && s.cur[0] == 65535, "500 ms frames jump");
    /* fade to off over 1 s (20000 ticks) */
    smooth_fade(&s, a, t, 20000);
    smooth_update(&s, t + 10000);
    CHECK(abs((int)s.cur[0] - 32767) < 40, "fade midpoint %u", s.cur[0]);
    smooth_update(&s, t + 20000);
    CHECK(s.cur[0] == 0 && s.cur[2] == 0, "fade ends at 0");
    printf("  ok: 50 fps -> 20 ms linear ramps, retarget without jump, 1 ms / 500 ms frames unsmoothed, 1 s fade\n");
}

/* ------------------------------------------------------------------------------------------------ gamma */
static void test_gamma(void)
{
    int m, v, ok = 1;
    printf("gamma: WS2805 8-bit -> 16-bit tables\n");
    for (m = 0; m < 3; m++) {
        CHECK(gamma_apply((uint8_t)m, 0) == 0 && gamma_apply((uint8_t)m, 255) == 65535, "endpoints %d", m);
        for (v = 1; v < 256; v++) {
            if (gamma_apply((uint8_t)m, (uint8_t)v) < gamma_apply((uint8_t)m, (uint8_t)(v - 1)) || gamma_apply((uint8_t)m, (uint8_t)v) == 0) ok = 0;
        }
    }
    CHECK(ok, "monotonic, on stays on");
    printf("  level 1/255: linear %u, gamma 2.2 %u, gamma 2.8 %u (of 65535); 128/255: %u / %u / %u\n",
           gamma_apply(0, 1), gamma_apply(1, 1), gamma_apply(2, 1), gamma_apply(0, 128), gamma_apply(1, 128), gamma_apply(2, 128));
}

/* ------------------------------------------------------------------------------------------------ store */
static void test_store(void)
{
    uint8_t page[256];
    settings_t s, out;
    store_slot_t slot;
    uint32_t seq, k;

    printf("settings records: append, newest wins, corruption ignored, full page\n");
    memset(page, 0xFF, sizeof page);
    CHECK(store_find(page, &out, &seq) == -1 && store_next_free(page) == 0, "empty page");
    settings_default(&s);
    for (k = 1; k <= 3; k++) {
        s.pwm_preset = (uint8_t)k;
        store_pack(&s, k, &slot);
        memcpy(page + store_next_free(page) * STORE_SLOT_BYTES, &slot, sizeof slot);
    }
    CHECK(store_find(page, &out, &seq) == 2 && seq == 3 && out.pwm_preset == 3, "newest record");
    page[2 * STORE_SLOT_BYTES + 10] ^= 0x40;            /* corrupt the newest (e.g. power lost mid-write) */
    CHECK(store_find(page, &out, &seq) == 1 && out.pwm_preset == 2, "corrupt record ignored");
    CHECK(store_next_free(page) == 3, "next free after a corrupt slot");
    for (k = 4; k <= 8; k++) {
        store_pack(&s, k, &slot);
        memcpy(page + store_next_free(page) * STORE_SLOT_BYTES, &slot, sizeof slot);
    }
    CHECK(store_next_free(page) == (int)STORE_SLOTS, "full page needs an erase");
    /* the CH32V203 RM gives 0xE339 as the erased half-word: an erased page must still read as empty */
    for (k = 0; k < sizeof page; k += 2) { page[k] = 0x39; page[k + 1] = 0xE3; }
    CHECK(store_find(page, &out, &seq) == -1 && store_next_free(page) == 0, "empty page, erased = 0xE339");
    for (k = 1; k <= 3; k++) {
        store_pack(&s, k, &slot);
        memcpy(page + store_next_free(page) * STORE_SLOT_BYTES, &slot, sizeof slot);
    }
    CHECK(store_find(page, &out, &seq) == 2 && seq == 3 && store_next_free(page) == 3, "append, erased = 0xE339");
    page[3 * STORE_SLOT_BYTES + 6] = 0x00;              /* a half-word that is neither erased value: used */
    CHECK(store_next_free(page) == 4, "a half-written slot is not free");
}

/* records with the extension (the current limit): round trip, covered by the CRC; another magic is ignored */
static void test_store_ext(void)
{
    uint8_t page[256];
    settings_t s, out;
    store_ext_t e, eo;
    store_slot_t slot;
    uint32_t seq;
    uint8_t i;

    printf("settings records: extension round trip, extension covered by the CRC, foreign records ignored\n");
    memset(page, 0xFF, sizeof page);
    settings_default(&s);
    s.pwm_preset = 5;
    /* records with an earlier layout's magic (0x55AA2915, 0x55AA2916: the CRC before the extension), even with a CRC
     * that is valid in this layout: ignored */
    for (i = 0; i < 2u; i++) {
        memset(&slot, 0xFF, sizeof slot);
        slot.magic = 0x55AA2915u + i;
        slot.seq = 9;
        slot.s = s;
        slot.crc = crc8_smbus((const uint8_t *)&slot.seq,
                              4u + (uint32_t)sizeof(settings_t) + (uint32_t)sizeof(store_ext_t));
        memcpy(page, &slot, sizeof slot);
        memset(&eo, 0, sizeof eo);
        CHECK(store_find_ext(page, &out, &eo, &seq) == -1, "a record with magic 0x%08X is ignored", slot.magic);
    }
    store_pack(&s, 1, &slot);
    memcpy(page, &slot, sizeof slot);
    CHECK(store_find_ext(page, &out, &eo, &seq) == 0 && seq == 1 && out.pwm_preset == 5 && eo.ilimit50 == 0xFFu,
          "a record without a limit");
    /* with the limit, appended */
    store_ext_clear(&e);
    e.ilimit50 = 120;                                    /* 6 A */
    for (i = 0; i < 6u; i++) e.reserved[i] = 0xFFu;
    s.pwm_preset = 6;
    store_pack_ext(&s, &e, 2, &slot);
    memcpy(page + store_next_free(page) * STORE_SLOT_BYTES, &slot, sizeof slot);
    CHECK(store_find_ext(page, &out, &eo, &seq) == 1 && seq == 2 && out.pwm_preset == 6 && memcmp(&e, &eo, sizeof e) == 0,
          "extension round trip");
    page[1 * STORE_SLOT_BYTES + 25] ^= 0x01;             /* flip a bit of the limit: the CRC covers the extension */
    CHECK(store_find_ext(page, &out, &eo, &seq) == 0 && out.pwm_preset == 5, "corrupt extension -> older record");
    store_pack(&s, 3, &slot);                            /* store_pack: v2 with a cleared extension */
    memcpy(page + store_next_free(page) * STORE_SLOT_BYTES, &slot, sizeof slot);
    CHECK(store_find_ext(page, &out, &eo, &seq) == 2 && seq == 3 && eo.ilimit50 == 0xFFu && eo.reserved[0] == 0xFFu,
          "store_pack: extension not set");
    CHECK(store_find(page, &out, &seq) == 2 && seq == 3, "store_find reads it");
}

/* ------------------------------------------------------------------------------------------------ current sense */
/* config commands 0x12 / 0x13: config.c only validates them (main.c stores the limit / runs the calibration) */
static void test_isense_config(void)
{
    settings_t s, ref;
    config_rx_t rx;
    cfg_result_t r;
    printf("config: current limit / calibration commands\n");
    settings_default(&s);
    ref = s;
    config_rx_init(&rx);
    r = config_apply(&rx, &s, CMD_ISENSE_LIMIT, 120);
    CHECK(r.event == CFG_ACK && memcmp(&s, &ref, sizeof s) == 0, "limit: ACK, settings untouched");
    r = config_apply(&rx, &s, CMD_ISENSE_LIMIT, 0);
    CHECK(r.event == CFG_ACK, "limit 0 = board rating");
    r = config_apply(&rx, &s, CMD_ISENSE_CAL, 0);
    CHECK(r.event == CFG_ACK, "calibrate at the default duty");
    r = config_apply(&rx, &s, CMD_ISENSE_CAL, 50);
    CHECK(r.event == CFG_ACK, "calibrate at 50 pct");
    r = config_apply(&rx, &s, CMD_ISENSE_CAL, 4);
    CHECK(r.event == CFG_NACK, "calibrate below 5 pct -> NACK");
    r = config_apply(&rx, &s, CMD_ISENSE_CAL, 101);
    CHECK(r.event == CFG_NACK, "calibrate above 100 pct -> NACK");
}

/* ------------------------------------------------------------------------------------------------ console */
static char cli_out[4096];
static settings_t cli_set;
static config_rx_t cli_rx;
static void cap_write(const char *s) { strncat(cli_out, s, sizeof cli_out - strlen(cli_out) - 1); }
static cfg_result_t cap_apply(uint8_t cmd, uint8_t arg) { return config_apply(&cli_rx, &cli_set, cmd, arg); }
static void cap_save(void) { cap_write("<saved>"); }

static void run_cli(cli_t *c, const char *in)
{
    cli_out[0] = 0;
    while (*in) cli_feed(c, *in++);
}

static void test_cli(void)
{
    cli_t c;
    cli_status_t st;
    printf("console: help / get / set / period / status / errors\n");
    memset(&c, 0, sizeof c);
    memset(&st, 0, sizeof st);
    settings_default(&cli_set);
    config_rx_init(&cli_rx);
    st.version = "test";
    st.steps = 5760;
    st.fps = 42;
    c.write = cap_write;
    c.apply = cap_apply;
    c.save_now = cap_save;
    c.set = &cli_set;
    c.st = &st;
    run_cli(&c, "help\r\n");
    CHECK(strstr(cli_out, "minpulse") && strstr(cli_out, "loss"), "help lists keys");
    run_cli(&c, "get\n");
    CHECK(strstr(cli_out, "preset=8\r\n") && strstr(cli_out, "smooth=1") && strstr(cli_out, "minpulse=20"), "get defaults");
    run_cli(&c, "set preset 2\r");
    CHECK(strstr(cli_out, "ok") && cli_set.pwm_preset == 2, "set preset");
    run_cli(&c, "set preset 2\r");
    CHECK(strstr(cli_out, "unchanged") != NULL, "same value");
    run_cli(&c, "set preset 99\r");
    CHECK(strstr(cli_out, "error") && cli_set.pwm_preset == 2, "range check");
    run_cli(&c, "set loss 2\r");
    CHECK(cli_set.signal_loss == 2, "loss fade");
    run_cli(&c, "period 5000\r");
    CHECK(strstr(cli_out, "ok") && cli_set.pwm_preset == 0xFF && settings_steps(&cli_set) == 5000, "raw period");
    run_cli(&c, "period 100\r");
    CHECK(strstr(cli_out, "error") != NULL, "period range");
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "fps=42") && strstr(cli_out, "pwm_hz=25000"), "status");
    run_cli(&c, "frobnicate\r");
    CHECK(strstr(cli_out, "unknown") != NULL, "unknown command");
    run_cli(&c, "save\r");
    CHECK(strstr(cli_out, "<saved>") != NULL, "save");
    run_cli(&c, "defaults\r");
    CHECK(cli_set.pwm_preset == PWM_PRESET_DEFAULT && cli_set.signal_loss == 0, "factory reset");
    /* current sense: keys and the status block */
    run_cli(&c, "set ilimit 120\r");
    CHECK(strstr(cli_out, "ok") != NULL, "set ilimit");
    run_cli(&c, "set ical 4\r");
    CHECK(strstr(cli_out, "error") != NULL, "ical range");
    st.ilimit50 = 120;
    st.i_model = 2;
    st.i_ma = 5432;
    st.i_peak_ma = 6100;
    st.i_zero_ma = 38;
    st.i_limit_ma = 6000;
    st.i_k_permille = 1000;
    st.i_trips = 1;
    st.i_trip_cause = 2;
    st.i_ch_ma[0] = 1000; st.i_ch_ma[1] = 1100; st.i_ch_ma[2] = 1200; st.i_ch_ma[3] = 1300; st.i_ch_ma[4] = 1400;
    run_cli(&c, "get\r");
    CHECK(strstr(cli_out, "ilimit=120\r\n") && strstr(cli_out, "ical=0\r\n"), "get ilimit / ical");
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "current=5.43 A\r\ncurrent_peak=6.10 A\r\ncurrent_zero=0.04 A\r\ncurrent_limit=6.00 A\r\n"
                          "current_scale=100.0 %\r\ncurrent_trips=1\r\n") &&
          strstr(cli_out, "current_state=ok, last trip overload") &&
          strstr(cli_out, "channel R/G/B/W1/W2=1.00/1.10/1.20/1.30/1.40 A (calibrated)"), "status current sense");
    st.i_tripped = 1;
    st.i_trip_cause = 1;
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "current_state=tripped, outputs off, last trip short") != NULL, "status tripped");
    /* voltage channels: the supply on PA4 with min / max and the input power, the spare input on TP8 */
    st.vs_aux_mv = 1650;
    st.vs_supply_mv = 24012;
    st.vs_supply_min_mv = 23980;
    st.vs_supply_max_mv = 24050;
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "supply=24.01 V (min 23.98 V, max 24.05 V)\r\ninput=130.4 W\r\ntp8=1.65 V\r\n"
                          "test_mode=off\r\n") != NULL, "status voltages, input power (24.012 V x 5432 mA)");
    /* uptime h:mm:ss (days beyond 24 h), outputs in %, the stagger offsets in % of the period, ripple in A */
    st.uptime_ms = 3723000u;
    st.out[0] = 34078; st.out[1] = 0; st.out[2] = 65535; st.out[3] = 655; st.out[4] = 21627;
    st.stagger_moves = 3;
    st.stagger_o2 = 22938;
    st.stagger_o3 = 43691;
    st.ripple_ma = 1234;
    st.i_k_permille = 812;
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "uptime=1:02:03\r\n") && strstr(cli_out, "out R/G/B/W1/W2=52.0/0.0/100.0/1.0/33.0 %\r\n") &&
          strstr(cli_out, "stagger_tim2=35.0 %\r\nstagger_tim3=66.7 %\r\nstagger_moves=3\r\nripple_rms=1.23 A\r\n") &&
          strstr(cli_out, "current_scale=81.2 %\r\n"), "status: uptime, outputs, stagger, ripple, scale");
    st.uptime_ms = 90061000u;                  /* 1 d 1:01:01 */
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "uptime=1d 1:01:01\r\n") != NULL, "status: uptime with days");
    st.uptime_ms = 0;
    /* volts with two decimals, rounded to 10 mV, no float */
    {
        static const struct { uint32_t mv; const char *txt; } VR[] = {
            { 12234u, "supply=12.23 V (" }, { 12235u, "supply=12.24 V (" }, { 0u, "supply=0.00 V (" },
            { 4u, "supply=0.00 V (" }, { 5u, "supply=0.01 V (" }, { 1095u, "supply=1.10 V (" },
            { 9995u, "supply=10.00 V (" }, { 75440u, "supply=75.44 V (" },
        };
        uint32_t k;
        for (k = 0; k < sizeof VR / sizeof VR[0]; k++) {
            st.vs_supply_mv = VR[k].mv;
            run_cli(&c, "status\r");
            CHECK(strstr(cli_out, VR[k].txt) != NULL, "%u mV shows as %s", VR[k].mv, VR[k].txt);
        }
        st.mcu_temp_c10 = 350;
        st.vdda_mv = 3281;
        run_cli(&c, "status\r");
        CHECK(strstr(cli_out, "mcu_temp=35.0 C (die), vdda=3.28 V\r\n") != NULL, "status: the 3.3 V rail in volts");
        st.vs_supply_mv = 24012;
    }
}
