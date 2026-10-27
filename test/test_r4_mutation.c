/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Round-4 regression tests derived from surviving single-change mutants.
 * Included by test_main.c: these verify safety/API contracts, not mutated text. */
static void test_r4_current_edges(void)
{
    isense_t s;
    uint32_t i, ev;
    isense_init(&s, 1, 2633, 8000);
    isense_set_gain(&s, 65536u); /* 1 mA/count: nominal 1.5 x fuse = 3949 mA, below the 97% cap (3972). */
    CHECK(s.fast_raw == 3949u, "R4: fast threshold below ADC cap must stay at 1.5 x fuse (%u)", s.fast_raw);
    CHECK(isense_tick(&s, 3948) == 0u && isense_tick(&s, 3948) == 0u,
          "R4: one ADC count below the fast threshold must not trip");
    CHECK(isense_tick(&s, 3949) == 0u && isense_tick(&s, 3949) == 1u,
          "R4: exact fast threshold trips on the second consecutive sample");
    isense_init(&s, 1, 10000, 8000);
    isense_set_gain(&s, 65536u);
    CHECK(s.fast_raw == 3972u, "R4: out-of-range fast threshold clamps to 97 percent of ADC range");

    /* Calibration must not start with sensing disabled, or while already tripped. */
    isense_init(&s, 0, 10000, 8000);
    isense_cal_start(&s, 0);
    CHECK(s.cal_step == 0u, "R4: disabled calibration must not start");
    isense_init(&s, 1, 10000, 8000);
    s.tripped = ISENSE_TRIP_FAST;
    isense_cal_start(&s, 0);
    CHECK(s.cal_step == 0u, "R4: tripped calibration must not start");

    /* A short during calibration aborts the test, and must not publish a completed calibration. */
    isense_init(&s, 1, 10000, 8000);
    isense_cal_start(&s, 0);
    (void)isense_tick(&s, 4095);
    CHECK(isense_tick(&s, 4095) == 1u, "R4: injected short trips during calibration");
    ev = isense_ms(&s, 81900u, 20, 4095);
    CHECK((ev & ISENSE_EV_CAL_FAIL) && !(ev & ISENSE_EV_CAL_DONE) && s.cal_step == 0u,
          "R4: trip aborts calibration and reports failure");

    /* Invalid restored single-level duty is ignored, including the exact ends. */
    for (i = 0; i < 2u; i++) {
        isense_cal_t c;
        memset(&c, 0, sizeof c);
        c.levels = 1; c.pct[0] = (uint8_t)(i ? 101 : 4);
        isense_init(&s, 1, 10000, 8000);
        isense_cal_set(&s, &c);
        CHECK(s.cal_have == 0u && !s.calibrated, "R4: invalid restored duty %u ignored", c.pct[0]);
    }
}

static void test_r4_limit_ties(void)
{
    climit_t c;
    climit_eval(&c, 8000, 8000, 8000, 8001);
    CHECK(c.limit_ma == 8000u && c.src == CLIMIT_SRC_BOARD && c.need == CLIMIT_NEED_YES,
          "R4: equal user/fuse limits keep the board source");
    climit_eval(&c, 6000, 8000, 6000, 6000);
    CHECK(c.limit_ma == 6000u && c.src == CLIMIT_SRC_USER && c.need == CLIMIT_NEED_NO,
          "R4: equal fuse limit keeps user source and equality is not overload");
}

static void test_r4_decoder_errors(void)
{
    static const ws_timing_t t = WS_TIMING_DEFAULT;
    uint16_t words[512];
    uint8_t out[8];
    uint32_t i, high;
    slot_t slot;
    memset(words, 0, sizeof words);
    CHECK(ws_decode_frame(words, 0, 8, &t, out) == WS_ERR_NO_GAP, "R4: empty capture has no reset gap");
    CHECK(ws_decode_last(words, 0, &t, out) == WS_ERR_NO_EDGE, "R4: empty tail has no edges");
    CHECK(ws_decode_frame(words, 8192, 8, &t, out) == WS_ERR_NO_EDGE, "R4: dark capture has no edges");
    CHECK(ws_decode_last(words, 8192, &t, out) == WS_ERR_NO_EDGE, "R4: dark tail has no edges");
    words[0] = 0x8000u;
    CHECK(ws_decode_frame(words, 8192, 8, &t, out) == WS_ERR_NO_GAP, "R4: capture starts high, no reset gap");
    for (high = t.high_max; high <= (uint32_t)t.high_max + 1u; high++) {
        memset(words, 0, sizeof words);
        for (i = 16; i < 16u + high; i++) words[i >> 4] |= (uint16_t)(0x8000u >> (i & 15u));
        CHECK(ws_decode_frame(words, 8192, 1, &t, out) ==
              (high == t.high_max ? 1 : WS_ERR_PULSE), "R4: maximum high pulse boundary (%u)", high);
        CHECK(ws_decode_last(words, 8192, &t, out) ==
              (high == t.high_max ? 1 : WS_ERR_PULSE), "R4: tail maximum high pulse boundary (%u)", high);
    }
    memset(words, 0, sizeof words);
    words[1] = 0xF000u;
    i = 20u + t.gap_min;
    words[i >> 4] |= (uint16_t)(0x8000u >> (i & 15u));
    CHECK(ws_decode_frame(words, 8192, 2, &t, out) == WS_ERR_SHORT, "R4: reset gap inside capture rejects frame");
    CHECK(ws_decode_last(words, 8192, &t, out) == WS_ERR_SHORT, "R4: reset gap inside tail rejects frame");
    CHECK(proto_parse(PROTO_NONE, out, &slot) == -1, "R4: unknown protocol cannot parse a slot");
}



static void test_r4_record_edges(void)
{
    uint8_t image[512], crc[ULTRA_LIGHTS], page[CAL_PAGE_BYTES];
    isense_cal_t c;
    uint32_t seq = 0x12345678u;
    uint16_t count = 1234;
    static const uint8_t roles[5] = { ULTRA_ROLE_BRIGHTNESS, 0, 0, 0, 0 };
    memset(page, 0xFF, sizeof page);
    memset(&c, 0x5A, sizeof c);
    CHECK(cal_find(page, &c, &seq, &count) == -1 && seq == 0x12345678u && count == 1234 && c.levels == 0x5A,
          "R4: missing calibration returns documented -1 and leaves outputs untouched");
    {
        cal_rec_t rec;
        memset(&c, 0, sizeof c); c.levels = 4; c.ma[0][3] = 1000;
        cal_pack(&c, 100, 7, &rec); memcpy(page, &rec, sizeof rec);
        c.ma[0][3] = 2000;
        cal_pack(&c, 100, 8, &rec); memcpy(page + CAL_REC_BYTES, &rec, sizeof rec);
        CHECK(cal_find(page, &c, &seq, &count) == 0 && c.ma[0][3] == 1000 && count == 7,
              "R4: duplicate-sequence flash image keeps the established first-record tie policy");
    }
    {
        uint8_t settings_page[STORE_SLOTS * STORE_SLOT_BYTES];
        settings_t settings, got;
        store_slot_t rec;
        memset(settings_page, 0xFF, sizeof settings_page);
        settings_default(&settings);
        store_pack(&settings, 100, &rec); memcpy(settings_page, &rec, sizeof rec);
        settings.dither_mode = DITHER_OFF;
        store_pack(&settings, 1, &rec); memcpy(settings_page + STORE_SLOT_BYTES, &rec, sizeof rec);
        CHECK(store_find(settings_page, &got, &seq) == 0 && seq == 100u && got.dither_mode == DITHER_ON,
              "R4: highest sequence wins even when it is physically the first record");
    }
}

static void test_r4_variable_floor(void)
{
    static const uint16_t arr[4] = { 99, 99, 99, 99 };
    dither_opts_t o = { DITHER_ON, 655, FLOOR_CLAMP, 20 };
    uint16_t dst[8];
    uint32_t i, aligned;
    for (aligned = 0; aligned < 2u; aligned++) {
        for (i = 0; i < 8u; i++) dst[i] = 0xBEEFu;
        dither_build_var(62258, 100, arr, 4, 1, &o, 0, dst, 2, (uint8_t)aligned); /* 95%, off-time rounds to 0 */
        for (i = 0; i < 4u; i++) {
            CHECK(dst[2u*i] == (aligned ? 0u : 100u) && dst[2u*i+1u] == 0xBEEFu,
                  "R4: CLAMP near full on writes every active entry and preserves other DMA channels");
        }
        dither_build_var(55705, 100, arr, 4, 1, &o, 0, dst, 2, (uint8_t)aligned); /* 85%, off-time rounds to floor */
        for (i = 0; i < 4u; i++) CHECK(dst[2u*i] == (aligned ? 20u : 80u), "R4: CLAMP ceiling keeps clean off-time");
    }
    for (i = 0; i < 8u; i++) dst[i] = 0xBEEFu;
    dither_build(12345, 5760, 0, &o, 0, dst, 2);
    dither_build_var(12345, 100, arr, 0, 1, &o, 0, dst, 2, 0);
    dither_build_var(12345, 100, arr, 65, 1, &o, 0, dst, 2, 0);
    for (i = 0; i < 8u; i++) CHECK(dst[i] == 0xBEEFu, "R4: invalid pattern length must not write destinations");
    o.floor_policy = FLOOR_SKIP;
    dither_build_var(6000, 100, arr, 4, 0, &o, 0, dst, 2, 0);
    for (i = 0; i < 4u; i++) CHECK(dst[2u*i] == 0u, "R4: omitted minimum length defaults to one period");
}

static void test_r4_stagger_precision(void)
{
    static const uint16_t duty[5] = { 21808, 18309, 283, 42081, 59179 };
    static const uint16_t peak[5] = { 4935, 4359, 570, 3636, 2280 };
    stagger_result_t r;
    uint32_t witness = stagger_ripple_ma(duty, peak, 22530, 15017);
    CHECK(stagger_valid(duty, 22530, 15017, 819), "R4: ripple witness respects edge gap");
    stagger_choose_max(duty, peak, 819, 21845, 43691, 55705, &r);
    CHECK(r.move && stagger_valid(duty, r.o2, r.o3, 819) && r.ripple_best <= witness,
          "R4: search must preserve enough cost precision for this 7.25 A mix (best %u, witness %u)",
          r.ripple_best, witness);
}

static void test_r4_update_session_change(void)
{
    tu_board_t b;
    uint8_t desc[UPD_DESC_BYTES];
    tu_desc(desc, TU_V(0, 4, 0, 0xFF), UPD_HW_ID_D2A_PLUS_5CH, TU_REVS, 1);
    tu_init(&b, TU_V(0, 5, 0, 0xFF), 1, 1);
    tu_upd(&b, UPD_PING, 0, 0, 0, 0); tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    tu_upd(&b, UPD_ANNOUNCE, 10, 0, UPD_F_FORCE, desc);
    CHECK(b.u.have_half == 1u && b.u.pending_session == 10u && b.u.force,
          "R4: initial forced half received");
    tu_upd(&b, UPD_ANNOUNCE, 11, 1, 0, desc + UPD_CHUNK_BYTES);
    CHECK(b.u.have_half == 2u && b.u.pending_session == 11u && !b.u.force && !b.u.decided && b.nact == 0u,
          "R4: new session discards partial descriptor and FORCE from the previous session");
    tu_upd(&b, UPD_ANNOUNCE, 11, 0, 0, desc);
    CHECK(b.u.decided && b.u.session == 11u && !tu_has(&b, UPD_APP_TAKE),
          "R4: unforced older image is never taken after session change");
    tu_end(&b, 11);
    CHECK(tu_has(&b, UPD_APP_SHOW_REJECTED), "R4: new session reports the older image rejected");
}

static void test_r4_mutation(void)
{
    printf("R4 mutation regressions: current thresholds, calibration safety, limit ties, malformed input\n");
    test_r4_current_edges();
    test_r4_limit_ties();
    test_r4_decoder_errors();

    test_r4_record_edges();
    test_r4_update_session_change();
    test_r4_variable_floor();
    test_r4_stagger_precision();
}
