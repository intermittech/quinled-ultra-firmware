/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* scenarios. Included by test_soak.c; soak_tests.exe <name>.
 *
 *   r8evtype       the event log's names: a log kept across a reset (.noinit, evlog_boot) may hold a type this build
 *                  doesn't name (a newer image's, or bytes a fault left): `status` shows "?" for it (the name
 *                  list's walk stops at its end: never into the other texts, for 255 never past the compressed text's
 *                  end) and names the known ones. The release console: its next full status (the receiver's sends none) */

#if CONSOLE_FULL
/* the event line at e: "... ago" then exactly tail (" ago, <name>, <current> A\r\n") */
static int r8_ev_is(const char *e, const char *tail)
{
    const char *p = e ? strstr(e, " ago, ") : NULL, *nl = e ? strstr(e, "\r\n") : NULL;
    return p && nl && p < nl && strncmp(p, tail, strlen(tail)) == 0;
}
#endif

static void sc_r8evtype(void)
{
    printf("soak r8evtype: event types this build doesn't name show as \"?\"\n");
    boot(0, 0, 0);
    run_ms(2000);
    ev_add(EVL_DIP, 1000u);
    ev_add(EVL_HOT + 1u, 2000u);                         /* the next type a newer image may log */
    ev_add(255u, 3000u);
#if !CONSOLE_FULL
    while(cli.st_part)run_ms(10);
    con_out_n=0;con_out[0]=0;
    while(!cli.st_part)run_ms(10);
    while(cli.st_part)run_ms(10);
    CHECK(con_field('N',evl.head,2)==255u,"unknown event ID preserved numerically");
    CHECK(con_field('N',(evl.head-1u)&(EVLOG_N-1u),2)==EVL_HOT+1u,"future event ID preserved");
    CHECK(con_field('N',(evl.head-2u)&(EVLOG_N-1u),2)==EVL_DIP,"known event ID preserved");
#else
    {
        const char *e1, *e2, *e3;
#if CONSOLE_FULL
        console("status");
#else
        while (cli.st_part) {                            /* the release console's next full status, whole */
            run_ms(10);
        }
        con_out_n = 0;
        con_out[0] = 0;
        while (!cli.st_part) {
            run_ms(10);
        }
        while (cli.st_part) {
            run_ms(10);
        }
#endif
        e1 = strstr(con_out, "event 1=");
        e2 = strstr(con_out, "event 2=");
        e3 = strstr(con_out, "event 3=");
        printf("  %.150s", e1 ? e1 : "(no event lines)\n");
        CHECK(r8_ev_is(e1, " ago, ?, 3.00 A\r\n"), "r8evtype: type 255: \"?\" (%.50s)", e1 ? e1 : "");
        CHECK(r8_ev_is(e2, " ago, ?, 2.00 A\r\n"), "r8evtype: the next type: \"?\" (%.50s)", e2 ? e2 : "");
        CHECK(r8_ev_is(e3, " ago, supply dip, 1.00 A\r\n"), "r8evtype: a known type named (%.50s)",
              e3 ? e3 : "");
    }
#endif
}

/* ======================================================================================================= r8sb */
/* the receiver's standby: `soak_tests.exe r8sb <case>`. The image starts with a stored scene
 * (every channel at 50 %). The notice = IDENTIFY 7 (ultra_slot_power_off), its copies spread over 100 ms with one SEQ;
 * the cut = the supply falls (a dip) 200 ms after it. Cases (the receiver's builds; app / g_app the Dig2Analog-Plus-5ch's,
 * the others print "receiver only" there):
 *   worth     lit at 100 % for 90 s (>= 1 Wh), off 3 s, the notice, the cut: one lifetime write at the cut, no dip
 *             (the record's count, the power events, the fault memory, the event log), no second write
 *   nothing   lit at 20 % for 10 s (nothing worth storing), the notice, the cut: no write; the dip not counted
 *   darkwait  as nothing, dark 40 s before the notice: the power-up alone makes no regular write either (the
 *             receiver's ev_save), so the standby cycle writes nothing
 *   missed    as nothing without the notice: the cut is a dip, counted and written
 *   brief     the notice, a 40 ms dip 100 ms later that recovers (no cut): nothing counted, nothing written; 2 s on the
 *             notice is forgotten: a later dip counts and writes as after a power-up
 *   copies    the copies at 0, 50 and 100 ms (one SEQ): applied once (the hold and its time from the first)
 *   bright    the notice while lit at 50 %: dark in 0.3 s without a step up, back after 2 s without the cut
 *   slow      (O1's R5) lit at 50 %, the notice, the cut's bus falling 0.8 V a second:
 *             the dip is the planned cut's (the reference held from the notice), nothing counted, dark throughout
 *             (never a real dip after the notice's 2 s with the lights back meanwhile)
 *   cal       the notice during a calibration test: the test stops at once (supply), nothing stored
 *   pending   the record's next page waits for an erase (bright since power-up, its erase refused at boot): the cut
 *             writes nothing (the notice can't make room)
 *   race      the cut's dip first (counted, its write), the notice 5 ms later: no second write
 *   (each fails on the standby branch before its fix)
 *   expire    R1: the notice, a 40 ms dip 50 ms later: SUPPLY (count, log, deepest) and STATUS show none, during and
 *             after it; past the notice's 2 s still none (the held count never replays as a power event)
 *   cross     R1: the notice, a dip from 50 ms that ends after the 2 s: the planned cut's all its length (no fault
 *             memory, no event, no count)
 *   pre       R1: a dip already running when the notice comes, ending in its 2 s: a dip, its fault memory too
 *   ack       R2: an identify pattern running at the notice fades dark with it (status LED goes on); a version pattern
 *             started meanwhile stays dark too
 *   cal_edge  R3: the notice in a calibration test's last ms: aborted, not committed, nothing stored; a test started
 *             meanwhile stops at once
 *   pvd       R4: an unplanned dip below the PVD: counted, its write refused and kept pending; VDD back: written
 *             within 1 s (the regular admission)
 *   pgfail    R4: the dip's write starts, the PVD falls before the program (FLASH_ERR_VDD): kept pending, written
 *             after the hold-off (10 s)
 *   exhaust   R5: the stored record's allowance used up (the next write earned 60 s into the run): DIAG shows
 *             EXHAUSTED and a dip writes nothing; past 60 s the flag clears and a dip writes
 *   sync      the hub's lifetime counters, LIFE_SYNC over the return channel: another UID16 or
 *             RECORDS merges nothing (results 2 / 3); the right ones raise the counters behind, the answer shows them
 *             (result 0, the applied mask, the Wh, RECORDS); again this power-up: result 4; no write; the feature bit
 *   syncgate  a merge, then the notice and the cut: no write (the merged values count as stored)
 *   syncneg   all-0 / all-0xFF arguments: refused; a relay-only POLL with them: nothing merged
 *   g_frame   grace: the host's frames from 0.4 s: dark until then, its state after, the scene never
 *   g_none    grace: no frame: dark for 1.5 s, then the scene fades in
 *   g_noslot  grace: Ultra headers from 0.4 s that carry no slot for it (a receiver beyond the hub's
 *             board count): they end the grace as signal, the scene never, dark for good (R7, accepted)
 *   g_ws      grace: WS2805 frames from 0.4 s: dark until then, the WS values, the scene never
 *   g_app     (the Dig2Analog-Plus-5ch) the scene at once
 *   app       (the Dig2Analog-Plus-5ch) the notice is ignored: no hold, the identify pattern runs on, nothing written */
static void r8_scene_image(void)
{
    scene_t sc;
    scene_rec_t rec;
    uint32_t i;
    scene_clear(&sc);
    sc.mask = 0x1Fu;
    for (i = 0; i < 5u; i++) {
        sc.v[i] = 32768u;
    }
    scene_log_pack(&sc, 1, &rec);
    memcpy(fl + (SCENE_LOG_ADDR - FL_BASE), &rec, sizeof rec);
}

static void r8_light(uint16_t m, uint32_t ms)            /* an Ultra host: light 0 (every channel) at m */
{
    host_on = 1;
    cmd_def(0, 0, 5, 1000);
    cmd_set(0, m, 0, 0);
    run_ms(ms);
}

static void r8_notice(uint8_t seq, uint32_t spread)      /* the copies: 0 / spread / 2 x spread ms */
{
    uint8_t s[32];
    uint32_t k;
    ultra_slot_power_off(s, seq);
    for (k = 0; k < 3u; k++) {
        host_send(s);                                    /* (the soak's host repeats each slot 3 times) */
        run_ms(k < 2u ? spread : 1u);
    }
}

static void r8_dip(uint32_t ms)                          /* a supply dip (24 V -> 18 V), then back */
{
    supply_mv = 18000u;
    run_ms(ms);
    supply_mv = 24000u;
}

static uint32_t r8_evl_dips(void)                        /* "supply dip" entries in the event log */
{
    uint32_t k, n = 0;
    for (k = 0; k < EVLOG_N; k++) {
        n += evl.e[k].type == EVL_DIP;
    }
    return n;
}

static uint32_t r8_vdd_falls(uint32_t addr, uint32_t halfwords, uint32_t erase_addr)   /* fl_fault: once, a
                                                                                         lifetime page's program */
{
    (void)erase_addr;
    if (halfwords != 0u && ((addr & ~0xFFu) == LIFE_PAGE0_ADDR || (addr & ~0xFFu) == LIFE_PAGE1_ADDR)) {
        fl_fault = 0;
        return FLASH_ERR_VDD;
    }
    return 0;
}

static void sc_r8sb(const char *c)
{
    uint32_t w0, d0, t, i, mx = 0;
    printf("soak r8sb %s: the receiver's standby (the notice, the grace window)\n", c);
    r8_scene_image();
#if BOARD == BOARD_UR_ANALOG_M
    if (!strcmp(c, "g_app") || !strcmp(c, "app")) {
        printf("  (the Dig2Analog-Plus-5ch's)\n");
        return;
    }
    if (!strncmp(c, "g_", 2)) {
        boot(0, 0, 0);
        if (!strcmp(c, "g_none")) {
            run_ms(1450);
            CHECK((out_now[0] | out_now[4]) == 0u && grace, "r8sb g_none: dark at 1.45 s (%u)", out_now[0]);
            run_ms(2600);
            CHECK(!grace && out_now[0] == 32768u && out_now[4] == 32768u, "r8sb g_none: the scene after 1.5 s, faded "
                  "in (%u)", out_now[0]);
            return;
        }
        for (t = 0; t < 400u; t++) {
            run_ms(1);
            mx |= out_now[0] | out_now[4];
        }
        CHECK(mx == 0u, "r8sb %s: dark until the first frame (%u)", c, mx);
        if (!strcmp(c, "g_noslot")) {                    /* frames whose slots end before this board: */
            uint8_t h[8];                                /* header-only frames (a receiver beyond the hub's count) */
            uint32_t frames0 = host_frames;
            for (t = 0, mx = 0; t < 5000u; t++) {
                if (t % 20u == 0u) {
                    ultra_build_header(h, host_us(sim_ns + 7000u + 64u * 1250u));
                    wire_frame(h, 64u, sim_ns + 7000u);
                }
                run_ms(1);
                mx |= out_now[0] | out_now[4];
            }
            printf("  5 s of headers without a slot for it: ultra %u, grace %u, signal lost %u, the most out %u, scene "
                   "shown %u\n", ultra_on, grace, signal_lost, mx, srun.hold);
            CHECK(ultra_on && !grace && !signal_lost && mx == 0u && host_frames == frames0,
                  "r8sb g_noslot: the headers end the grace (signal, no scene) and drive nothing: dark for good (R7, "
                  "accepted: a receiver beyond the hub's count; most %u)", mx);
            CHECK(line_lock.good==3u && line_lock.profile==4u &&
                  !line_stats.trained && !ul.clk.synced && !ul.clk.rated &&
                  !ws_line_ready(&line_lock,ms_now) && !ultra_seen,
                  "header-only keepalive stays tentative without timing or slot qualification");
            eff.signal_loss=3u;
            scene_want.loss_mask=31u;
            for(i=0;i<5u;i++) scene_want.loss_v[i]=16384u;
            run_ms(5000);
            CHECK(signal_lost && out_now[0]==16384u && out_now[4]==16384u,
                  "real loss after header-only traffic reaches the loss scene (%u/%u mask %u hold %u value %u)",
                  signal_lost,out_now[0],scene_want.mask,srun.hold,scene_want.v[0]);
            return;
        }
        if (!strcmp(c, "g_frame")) {
            r8_light(19661u, 0);
        } else {
            static const uint8_t col[5] = { 0x40, 0x80, 0xC0, 0x20, 0x10 };
            r7_ws(col, 1);
        }
        for (t = 0, mx = 0; t < 3000u; t++) {            /* (past G, and past a scene's 1 s fade) */
            if (strcmp(c, "g_frame") != 0 && t % 20u == 0u) {
                static const uint8_t col[5] = { 0x40, 0x80, 0xC0, 0x20, 0x10 };
                wire_frame(col, 40u, sim_ns + 7000u);
            }
            run_ms(1);
            mx = out_now[4] > mx ? out_now[4] : mx;
        }
        printf("  after the frames: out %u %u, the highest CH5 %u, grace %u\n", out_now[0], out_now[4], mx, grace);
        CHECK(!grace && mx < 20000u && out_now[0] != 0u && (strcmp(c, "g_frame") != 0 || out_now[0] == 19661u),
              "r8sb %s: the controller's state, the scene never (CH5 at most %u)", c, mx);
        return;
    }
    if (!strncmp(c, "sync", 4)) {
#if BOARD_RETURN
        rc_obs_t o;
        uint8_t a[23], p[32];
        uint32_t k, ups, wh, res;
        boot(0, 0, 0);
        r8_light(13107u, 10000u);
        r8_light(0u, 3000u);
        w0 = W_LIFE();
        CHECK(rc_ask(ULTRA_PAGE_LIFETIME, 0, 0, &o) == ULTRA_PAGE_LIFETIME, "r8sb sync: LIFETIME answered");
        memset(a, 0, sizeof a);
        memcpy(a + ULTRA_LS_LIFETIME, D(&o), 16);        /* the board's own, then the hub's steps ahead */
        ups = ultra_get24(D(&o) + ULTRA_LT_POWER_UPS);
        wh = energy_total_wh(&en);
        ultra_put24(a + ULTRA_LS_LIFETIME + ULTRA_LT_POWERED, ultra_get24(D(&o) + ULTRA_LT_POWERED) + 50u);   /* 5 h */
        ultra_put24(a + ULTRA_LS_LIFETIME + ULTRA_LT_POWER_UPS, ups + 40u);
        ultra_put24(a + ULTRA_LS_LIFETIME + ULTRA_LT_DIPS, ultra_get24(D(&o) + ULTRA_LT_DIPS) + 3u);
        ultra_put32(a + ULTRA_LS_TOTAL_WH, wh + 120u);
        a[ULTRA_LS_RECORDS] = (uint8_t)ultra_get24(D(&o) + ULTRA_LT_RECORDS);
        if (!strcmp(c, "syncgate")) {                    /* the review's 7: a sync, the notice, the cut: no write */
            ultra_put16(a + ULTRA_LS_UID16, rc_uid16);
            CHECK(rc_ask(ULTRA_PAGE_LIFE_SYNC, a, sizeof a, &o) == ULTRA_PAGE_LIFE_SYNC &&
                  D(&o)[ULTRA_LS_A_RESULT] == ULTRA_LS_R_APPLIED, "r8sb syncgate: merged");
            run_ms(1500);                                /* (the write rules see the merge: energy_second) */
            r8_notice(0x59, 50u);
            run_ms(100);
            supply_mv = 18000u;
            host_on = 0;
            run_ms(60);
            CHECK(W_LIFE() == w0 && en.lt.ups == ups + 40u, "r8sb syncgate: a merge, the notice, the cut: no "
                  "write (%u)", W_LIFE() - w0);
            return;
        }
        if (!strcmp(c, "syncneg")) {                     /* the review's 12: zero / 0xFF arguments, a relay-only POLL */
            static const uint8_t z[23] = { 0 };
            uint8_t f[23];
            memset(f, 0xFF, sizeof f);
            CHECK(rc_ask(ULTRA_PAGE_LIFE_SYNC, z, sizeof z, &o) == ULTRA_PAGE_LIFE_SYNC &&
                  D(&o)[ULTRA_LS_A_RESULT] == (rc_uid16 ? ULTRA_LS_R_UID : ULTRA_LS_R_RECORDS) &&
                  rc_ask(ULTRA_PAGE_LIFE_SYNC, f, sizeof f, &o) == ULTRA_PAGE_LIFE_SYNC &&
                  D(&o)[ULTRA_LS_A_RESULT] == (rc_uid16 != 0xFFFFu ? ULTRA_LS_R_UID : ULTRA_LS_R_RECORDS),
                  "r8sb syncneg: all-0 / all-0xFF arguments: refused (%u)", D(&o)[ULTRA_LS_A_RESULT]);
            ultra_put16(a + ULTRA_LS_UID16, rc_uid16);
            rc_poll_build(p, rc_seq++, ULTRA_POLL_RELAY_ONLY, 1, ULTRA_PAGE_LIFE_SYNC, 0, a, sizeof a);
            rc_window(p, 1, ULTRA_HDR_RW, 0, 0, &o);
            run_ms(10);
            CHECK(en.lt.ups == ups && energy_total_wh(&en) == wh && !en.synced, "r8sb syncneg: relay only: nothing "
                  "merged");
            return;
        }
        for (k = 0; k < 4u; k++) {                       /* another UID16, another RECORDS, this board's, again */
            ultra_put16(a + ULTRA_LS_UID16, k == 0u ? rc_uid16 ^ 0x5A5Au : rc_uid16);
            a[ULTRA_LS_RECORDS] = (uint8_t)(a[ULTRA_LS_RECORDS] + (k == 1u) - (k == 2u));
            CHECK(rc_ask(ULTRA_PAGE_LIFE_SYNC, a, sizeof a, &o) == ULTRA_PAGE_LIFE_SYNC, "r8sb sync: answered (%u)", k);
            res = D(&o)[ULTRA_LS_A_RESULT];
            printf("  sync %u: result %u, applied %02X, power-ups %u (own %u), dips %u, powered %u x 0.1 h, %u Wh "
                   "(own %u), records %u\n", k, res, D(&o)[ULTRA_LS_A_APPLIED], ultra_get24(D(&o) + ULTRA_LT_POWER_UPS),
                   ups, ultra_get24(D(&o) + ULTRA_LT_DIPS), ultra_get24(D(&o) + ULTRA_LT_POWERED),
                   ultra_get32(D(&o) + ULTRA_LS_A_TOTAL_WH), wh, ultra_get16(D(&o) + ULTRA_LS_A_RECORDS));
            CHECK(res == (k == 0u ? ULTRA_LS_R_UID : k == 1u ? ULTRA_LS_R_RECORDS : k == 2u ? ULTRA_LS_R_APPLIED :
                  ULTRA_LS_R_ONCE) && ultra_get24(D(&o) + ULTRA_LT_POWER_UPS) == (k < 2u ? ups : ups + 40u) &&
                  ultra_get32(D(&o) + ULTRA_LS_A_TOTAL_WH) == (k < 2u ? wh : wh + 120u) &&
                  D(&o)[ULTRA_LS_A_APPLIED] == (k == 2u ? ULTRA_LS_F_POWERED | ULTRA_LS_F_UPS | ULTRA_LS_F_DIPS |
                  ULTRA_LS_F_WH : 0u) && ultra_get16(D(&o) + ULTRA_LS_A_RECORDS) == (uint16_t)en.lt.writes,
                  "r8sb sync %u: %s", k, k == 0u ? "another UID16: nothing merged" : k == 1u ?
                  "another RECORDS: nothing merged" : k == 2u ? "merged, the answer shows it" :
                  "again this power-up: nothing more");
        }
        CHECK(W_LIFE() == w0, "r8sb sync: no write of its own (%u)", W_LIFE() - w0);
        {
            static const uint8_t part1[1] = { ULTRA_ID_PART_FEATURES };   /* (IDENTITY part 1) */
            CHECK(rc_ask(ULTRA_PAGE_IDENTITY, part1, 1, &o) == ULTRA_PAGE_IDENTITY &&
                  (ultra_get32(D(&o) + ULTRA_IDF_FEATURES) & ULTRA_FEAT_LIFE_SYNC), "r8sb sync: the feature bit");
        }
#endif
        return;
    }
    if (!strcmp(c, "exhaust")) {
#if BOARD_RETURN
        static const uint8_t a0[1] = { 0 };
        rc_obs_t o;
        life_rec_t r;
        uint32_t f[3];
        memset(&r, 0, sizeof r);
        r.on_s = 4u * LIFE_EARN_S - 60u;                 /* earned: LIFE_RESERVE + 3, the 4th 60 s into the run */
        r.writes = LIFE_RESERVE + 3u;
        r.ups = 1000u;
        r.ver = (uint8_t)energy_ver(fwver_parse(FW_VERSION));   /* (no update: else stored once earned, at 60 s) */
        r.magic = LIFE_MAGIC; r.format=3; r.count=BOARD_CHANNELS;
        r.crc = crc16_ccitt((const uint8_t *)&r, LIFE_REC_BYTES - 2u);
        memcpy(fl + (LIFE_PAGE0_ADDR - FL_BASE), &r, sizeof r);
        boot(0, 0, 0);
        r8_light(13107u, 10000u);
        r8_light(0u, 3000u);
        w0 = W_LIFE();
        for (i = 0; i < 3u; i++) {                       /* 13 s: used up; 65 s: earned; after its write: used up */
            if (i == 1u) {
                run_ms(50000);
            }
            CHECK(rc_ask(ULTRA_PAGE_DIAG, a0, 1, &o) == ULTRA_PAGE_DIAG, "r8sb exhaust: DIAG 0 answered");
            f[i] = D(&o)[ULTRA_DG_FLASH_PENDING];
            if (i < 2u) {
                r8_dip(40u);
                run_ms(1500);
            }
            printf("  at %u s: DIAG flash pending %02X, written %u (record writes %u over the reserve, %u h)\n", up_s,
                   f[i], W_LIFE() - w0, en.lt.writes - LIFE_RESERVE, en.lt.on_s / 3600u);
        }
        CHECK((f[0] & ULTRA_DG_FP_EXHAUSTED) && W_LIFE() == w0 + 1u && en.lt.writes == LIFE_RESERVE + 4u &&
              !(f[1] & ULTRA_DG_FP_EXHAUSTED) && (f[2] & ULTRA_DG_FP_EXHAUSTED), "r8sb exhaust: used up: flagged, "
              "the dip writes nothing; earned: the flag clear, the dip writes; then flagged again");
#endif
        return;
    }
    if (!strcmp(c, "pending")) {                         /* page A full (writes 6-10), page B older (1-5) */
        life_rec_t r;
        uint32_t p, k;
        for (p = 0; p < 2u; p++) {
            for (k = 0; k < LIFE_RECS; k++) {
                memset(&r, 0, sizeof r);
                r.writes = (p ? 1u : 6u) + k;
                r.ups = r.writes;
                r.magic = LIFE_MAGIC; r.format=3; r.count=BOARD_CHANNELS;
                r.crc = crc16_ccitt((const uint8_t *)&r, LIFE_REC_BYTES - 2u);
                memcpy(fl + ((p ? LIFE_PAGE1_ADDR : LIFE_PAGE0_ADDR) - FL_BASE) + k * LIFE_REC_BYTES, &r, sizeof r);
            }
        }
        sim_vdd_low = 1;                                 /* the erase at power-up refused */
        boot(0, 0, 0);
        run_ms(1);
        sim_vdd_low = 0;
        r8_light(32768u, 20000u);                        /* bright: no erase ahead either */
        CHECK(life_pending() == 2u, "r8sb pending: the record's next page waits for its erase (%u)", life_pending());
        w0 = W_LIFE();
        d0 = en.lt.dips;
        r8_notice(0x59, 50u);
        run_ms(100u);
        r8_dip(40u);
        CHECK(W_LIFE() == w0 && en.lt.dips == d0, "r8sb pending: nothing written at the cut (%u)", W_LIFE() - w0);
        return;
    }
    boot(0, 0, 0);
    if (!strcmp(c, "worth")) {
        r8_light(65535u, 90000u);                        /* ~50 W x 90 s: > 1 Wh */
    } else {
        r8_light(13107u, 10000u);
    }
    if (!strcmp(c, "bright")) {
        uint8_t s[32];
        r8_light(32768u, 3000u);
        ultra_slot_power_off(s, 0x59);
        for (t = 0, mx = out_now[0], i = 0; t < 400u; t++) {
            uint16_t o = out_now[0];
            if (t == 0u || t == 50u || t == 100u) {
                host_send(s);                            /* the copies */
            }
            run_ms(1);
            i += out_now[0] > o;                         /* a step up */
        }
        CHECK(out_now[0] == 0u && i == 0u && vdip.planned && mx >= 32000u, "r8sb bright: dark within 0.4 s, never up "
              "(%u, %u ups)", out_now[0], i);
        run_ms(2500);
        CHECK(vdip.planned && out_now[0] == 32768u, "r8sb bright: no cut: back after 2 s (%u; a dip still "
              "the cut's for 10 s)", out_now[0]);
        run_ms(7500);
        CHECK(!vdip.planned && out_now[0] == 32768u, "r8sb bright: 10 s on, the notice forgotten (%u)", out_now[0]);
        return;
    }
    if (!strcmp(c, "slow")) {                            /* (O1's R5) big port caps: the cut's bus */
        uint8_t s[32];                                   /* falls 0.8 V a second, 24 V to 0 in 30 s */
        uint32_t w1, n0, top = 0, on = 0, cls = 0;
        r8_light(32768u, 3000u);
        w1 = W_LIFE();
        n0 = vdip.count;
        d0 = en.lt.dips;
        i = r8_evl_dips();
        ultra_slot_power_off(s, 0x59);
        for (t = 0; t < 12000u; t++) {
            if (t == 0u || t == 50u || t == 100u) {
                host_send(s);                            /* the copies */
            }
            if (t >= 200u) {
                supply_mv = 24000u - (t - 200u) * 4u / 5u;   /* the cut at 200 ms, then the slow decay */
                host_on = 0;
            }
            run_ms(1);
            if (t >= 400u) {
                top = out_now[0] > top ? out_now[0] : top;  /* (after the notice's 0.3 s fade) */
            }
            if (!on && vdip.in_dip) {
                on = t;
                cls = vdip.in_dip;
            }
        }
        printf("  the slow cut: a dip from %u ms (class %u), now %u; the most out after the fade %u; counted %u, "
               "lifetime dips %u, events %u, written %u\n", on, cls, vdip.in_dip, top, vdip.count - n0, en.lt.dips - d0,
               r8_evl_dips() - i, W_LIFE() - w1);
        CHECK(cls == 2u && vdip.in_dip == 2u && top == 0u && vdip.count == n0 && en.lt.dips == d0 &&
              r8_evl_dips() == i && !(pwrev_latched(&pw) & ULTRA_PW_EV_SUPPLY_DIP),
              "r8sb slow: the planned cut's dip all along (class %u), dark throughout (the most %u), nothing counted",
              cls, top);
        return;
    }
    if (!strcmp(c, "cal")) {
        uint32_t wc = W_CAL();
        r8_light(32768u, 1000u);
        cmd_identify(ULTRA_IDENT_CAL, 0);
        run_ms(300);
        CHECK(is.cal_step != 0u, "r8sb cal: the test runs (%u)", is.cal_step);
        r8_notice(0x59, 1u);
        run_ms(5);
        CHECK(is.cal_step == 0u && is.cal_fail == ISENSE_CAL_F_SUPPLY, "r8sb cal: the test stopped at the notice (%u, "
              "%u)", is.cal_step, is.cal_fail);
        run_ms(2000);
        CHECK(W_CAL() == wc, "r8sb cal: nothing stored (%u)", W_CAL() - wc);
        return;
    }
    w0 = W_LIFE();                                       /* (before the dark: a regular write would come 30 s in) */
    r8_light(0u, !strcmp(c, "darkwait") ? 40000u : 3000u);
    d0 = en.lt.dips;
    i = r8_evl_dips();
    if (!strcmp(c, "race")) {
        r8_dip(1u);                                      /* the cut first (its dip: counted, written) */
        supply_mv = 18000u;
        run_ms(5);
        r8_notice(0x59, 50u);
        run_ms(100);
        CHECK(W_LIFE() == w0 + 1u && en.lt.dips == d0 + 1u, "r8sb race: the dip counted before the notice, one write "
              "(%u)", W_LIFE() - w0);
        return;
    }
    if (!strcmp(c, "ack")) {
        cmd_identify(ULTRA_IDENT_ON, 0);
        run_ms(500);
        r8_notice(0x59, 50u);
        run_ms(300);                                     /* past the 0.3 s fade */
        for (t = mx = 0; t < 800u; t++) {
            run_ms(1);
            mx |= out_now[0];
        }
        CHECK(mx == 0u && ack.active && vdip.planned, "r8sb ack: the identify pattern dark with the notice, still "
              "running for the status LED (out %u, ack %u)", mx, ack.active);
        cmd_identify(ULTRA_IDENT_VERSION, 0);            /* a pattern started in the notice's 2 s */
        for (t = mx = 0; t < 500u; t++) {
            run_ms(1);
            mx |= out_now[0];
        }
        CHECK(mx == 0u && vdip.planned, "r8sb ack: a pattern started meanwhile stays dark (%u)", mx);
        return;
    }
    if (!strcmp(c, "pvd") || !strcmp(c, "pgfail")) {
        uint32_t f0 = stat.fl_errs, pv = !strcmp(c, "pvd");
        if (pv) {
            sim_vdd_low = 1;                             /* below the PVD all through the dip */
        } else {
            fl_fault = r8_vdd_falls;                     /* admitted, then the PVD at flash_op */
        }
        r8_dip(40u);
        sim_vdd_low = 0;
        run_ms(100);
        CHECK(W_LIFE() == w0 && en.lt.dips == d0 + 1u, "r8sb %s: the dip counted, not written yet (%u)", c,
              W_LIFE() - w0);
        run_ms(pv ? 1100u : 11000u);
        printf("  written %u, pending %u, flash errors %u\n", W_LIFE() - w0, en.ev & 4u, stat.fl_errs - f0);
        CHECK(W_LIFE() == w0 + 1u && !(en.ev & 4u) && en.lt.dips == d0 + 1u, "r8sb %s: written when it may (%u)", c,
              W_LIFE() - w0);
        fl_fault = 0;
        return;
    }
    if (!strcmp(c, "cal_edge")) {
        uint8_t s[32];
        uint32_t wc = W_CAL();
        cmd_identify(ULTRA_IDENT_CAL, 0);
        for (t = 0; t < 3000u; t++) {
            run_ms(1);
            if (is.cal_step == ISENSE_CAL_STEPS_MAX && is.cal_t == ISENSE_CAL_LVL_SETTLE_MS + ISENSE_CAL_LVL_MEAS_MS - 1u) {
                break;
            }
        }
        CHECK(t < 3000u, "r8sb cal_edge: the test's last ms reached");
        ultra_slot_power_off(s, 0x59);
        handle_ultra((uint32_t)(sim_ns / 1000u), tick_now, s);   /* the notice exactly then */
        run_ms(5);
        CHECK(vdip.planned && is.cal_step == 0u && is.cal_fail == ISENSE_CAL_F_SUPPLY && is.cal_have == 0u &&
              !cal_save_pend && W_CAL() == wc, "r8sb cal_edge: aborted, not committed (fail %u, have %u, pending %u)",
              is.cal_fail, is.cal_have, cal_save_pend);
        cmd_identify(ULTRA_IDENT_CAL, 0);                /* a test started in the notice's 2 s */
        for (t = mx = i = 0; t < 600u; t++) {
            run_ms(1);
            i = is.cal_step != 0u ? i + 1u : 0u;
            mx = i > mx ? i : mx;
        }
        run_ms(2000);
        CHECK(mx <= 1u && is.cal_have == 0u && W_CAL() == wc, "r8sb cal_edge: a test started meanwhile stops at once "
              "(ran %u ms, stored %u)", mx, W_CAL() - wc);
        return;
    }
    if (!strcmp(c, "copies")) {
        r8_notice(0x59, 50u);
        t = plan_ms;
        run_ms(100);
        CHECK(vdip.planned && t == plan_ms && dk.until == t + PLAN_HOLD_MS, "r8sb copies: applied once, at the first "
              "copy (%u, %u)", t, plan_ms);
        return;
    }
    if (strcmp(c, "missed") != 0 && strcmp(c, "pre") != 0) {
        r8_notice(0x59, 50u);
        CHECK(vdip.planned, "r8sb %s: the notice taken", c);
    }
    if (!strcmp(c, "expire") || !strcmp(c, "cross")) {
        uint8_t pg[24];
        uint32_t n0 = vdip.count, why;
        run_ms(50);
        supply_mv = 18000u;
        run_ms(20);
        why = light_why();
        run_ms(!strcmp(c, "cross") ? 2080u : 20u);       /* cross: back up after the notice's 2 s */
        supply_mv = 24000u;
        run_ms(100);
        vdip_page(&vdip, &vs_sup, up_s, vs_sup.mv, 0, pg);
        printf("  after the dip: planned %u, SUPPLY count %u, logged %u, STATUS why %u (in the dip %u)\n", vdip.planned,
               ultra_get16(pg), vdip.n_log, light_why(), why);
        CHECK(ultra_get16(pg) == n0 && vdip.count == n0 && vdip.n_log == 0u && vdip.deep.ref_x == 0u &&
              why != ULTRA_ST_WHY_SUPPLY && light_why() != ULTRA_ST_WHY_SUPPLY, "r8sb %s: SUPPLY / STATUS: no dip", c);
        run_ms(PLAN_CUT_MS);
        printf("  past the notice: planned %u, lifetime dips %u, fault memory %u %%, events %u, latch %02X\n",
               vdip.planned, en.lt.dips - d0, en.lt.dip_pct, r8_evl_dips() - i, pwrev_latched(&pw));
        CHECK(!vdip.planned && en.lt.dips == d0 && en.lt.dip_pct == 0u && r8_evl_dips() == i &&
              !(pwrev_latched(&pw) & ULTRA_PW_EV_SUPPLY_DIP) && vdip.count == n0 && light_why() != ULTRA_ST_WHY_SUPPLY &&
              W_LIFE() == w0, "r8sb %s: past the notice: still no dip anywhere, nothing written (%u)", c, W_LIFE() - w0);
        return;
    }
    if (!strcmp(c, "pre")) {
        supply_mv = 18000u;                              /* a dip first */
        run_ms(20);
        r8_notice(0x59, 1u);
        run_ms(20);
        supply_mv = 24000u;                              /* back up in the notice's 2 s */
        run_ms(100);
        printf("  planned %u, lifetime dips %u, logged %u, fault memory %u %%, events %u, latch %02X\n", vdip.planned,
               en.lt.dips - d0, vdip.n_log, en.lt.dip_pct, r8_evl_dips() - i, pwrev_latched(&pw));
        CHECK(vdip.planned && en.lt.dips == d0 + 1u && vdip.n_log == 1u && en.lt.dip_pct != 0u && r8_evl_dips() == i + 1u
              && (pwrev_latched(&pw) & ULTRA_PW_EV_SUPPLY_DIP), "r8sb pre: a dip before the notice stays one, its fault "
              "memory too");
        return;
    }
    if (!strcmp(c, "brief")) {
        run_ms(100);
        r8_dip(40u);
        run_ms(100);
        CHECK(W_LIFE() == w0 && en.lt.dips == d0 && r8_evl_dips() == i && !(pwrev_latched(&pw) & ULTRA_PW_EV_SUPPLY_DIP)
              && en.lt.dip_pct == 0u, "r8sb brief: the dip in the notice's 2 s: nothing counted or written (%u, %u)",
              W_LIFE() - w0, en.lt.dips - d0);
        run_ms(PLAN_CUT_MS);
        CHECK(!vdip.planned, "r8sb brief: 10 s on (2 s before): the notice forgotten");
        r8_dip(40u);
        run_ms(100);
        CHECK(W_LIFE() == w0 + 1u && en.lt.dips == d0 + 1u, "r8sb brief: then a dip counts and writes as after a "
              "power-up (%u, %u)", W_LIFE() - w0, en.lt.dips - d0);
        return;
    }
    run_ms(100);                                         /* the cut 200 ms after the first copy */
    supply_mv = 18000u;
    host_on = 0;
    run_ms(60);
    printf("  the cut: writes %u, dips %u (count %u), events %u, power-event latch %02X\n", W_LIFE() - w0,
           en.lt.dips - d0, vdip.count, r8_evl_dips() - i, pwrev_latched(&pw));
    if (!strcmp(c, "missed")) {
        CHECK(W_LIFE() == w0 + 1u && en.lt.dips == d0 + 1u, "r8sb missed: a dip, counted and written (%u, %u)",
              W_LIFE() - w0, en.lt.dips - d0);
        return;
    }
    CHECK(W_LIFE() == w0 + (uint32_t)!strcmp(c, "worth") && en.lt.dips == d0 && r8_evl_dips() == i &&
          !(pwrev_latched(&pw) & ULTRA_PW_EV_SUPPLY_DIP), "r8sb %s: the planned cut: no dip anywhere, written %s (%u)",
          c, !strcmp(c, "worth") ? "once (>= 1 Wh)" : "never (nothing worth storing)", W_LIFE() - w0);
    run_ms(300);
    CHECK(W_LIFE() == w0 + (uint32_t)!strcmp(c, "worth"), "r8sb %s: no second write (%u)", c, W_LIFE() - w0);
#else
    (void)w0, (void)d0, (void)t, (void)i, (void)mx;
    boot(0, 0, 0);
    if (!strcmp(c, "g_app")) {
        run_ms(150);
        t = out_now[0];
        run_ms(300);
        CHECK(t != 0u && out_now[0] > t && out_now[0] < 32768u, "r8sb g_app: the Dig2Analog-Plus-5ch's scene fades up "
              "once (lit at 0.15 s: %u; the soft start's ramp done by 0.45 s: %u)", t, out_now[0]);
    } else if (!strcmp(c, "app")) {
        r8_light(19661u, 20000u);
        r8_light(0u, 3000u);
        cmd_identify(ULTRA_IDENT_ON, 0);                 /* the identify pattern (30 s): an applied IDENTIFY 7 would */
        run_ms(500);                                     /* end it (main.c: an unknown value stops the pattern) */
        w0 = W_LIFE();
        r8_notice(0x59, 50u);
        run_ms(3000);
        CHECK(W_LIFE() == w0 && ack.active && !dk.active, "r8sb app: the Dig2Analog-Plus-5ch ignores IDENTIFY 7 "
              "(%u, ack %u)", W_LIFE() - w0, ack.active);
    } else {
        printf("  (the receiver's)\n");
    }
#endif
}

static int r8_scenario(const char *s, int argc, char **argv)
{
    if (!strcmp(s, "r8evtype")) {
        sc_r8evtype();
    } else if (!strcmp(s, "r8sb")) {
        sc_r8sb(argc > 2 ? argv[2] : "worth");
    } else {
        return 0;
    }
    return 1;
}
