/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Review round 6 (fable-sys): system-level scenarios on the whole firmware (src/app/main.c on the soak's hardware
 * model), included by test_soak.c after test_r5_age.c. Each runs in its own process: soak_tests.exe <name> [args].
 *
 *   r6e2e      a stored power-on effect from boot, ESPHome takes over, a calibration, a short's trip, a factory reset
 *              while the host keeps the light on, an update session passing through (transparency): the records and
 *              the outputs after every step
 *   r6fxreset <tp9 | host | console>  (0.6.7, F10) the factory reset like a reboot: dark after the pattern, nothing it
 *              cleared back after a power cut at any ms (main.c records_load on every flash image of the next 3 s)
 *   r6ws2ultra WS2805 controller -> Ultra hub -> WS2805 controller without a power cycle: the WS values from before the
 *              Ultra phase must not reappear at the switch back
 *   r6wall <n> <image>   the wall-switch pattern, one session per process: boot from the flash image, the stored scene
 *              lit, 150 s, the power cut (a supply dip, the dip writes), the image saved; session n checks the
 *              lifetime record of the sessions before (run_r6wall.cmd runs 40)
 *   r6flashfail  every flash operation fails (write protection): the main loop must not spend its time retrying
 *   r6boot5    every store with an erase ahead due at power-up: all of them erased before the outputs start
 *   r6rcdim    (BOARD=2) POLL windows while a 20 s dimming runs; the console's `save` typed during a window (0.6.7:
 *              answers pending, nothing written in the window; fable-sys 05 moot with the release console) */

/* ---- helpers */
static void r6_ws2805_frame(const uint8_t b[5], uint32_t gap_ms)
{
    wire_frame(b, 40u, sim_ns + 7000u);
    run_ms(gap_ms);
}

/* a SET with a colour (test_soak.c's cmd_set sends white) */
static void r6_cmd_set_rgb(uint8_t l, uint16_t m, uint16_t r, uint16_t g, uint16_t b, uint32_t dur, uint8_t flags)
{
    uint8_t s[32];
    uint16_t a[5] = { r, g, b, 0, 0 }, bb[5] = { 65535, 65535, 65535, 65535, 65535 };
    ultra_slot_set(s, l, seq_next(), flags, m, a, bb, dur, 0);
    host_send(s);
}

/* the power-on effect store as a host left it: two lights, a candle on CH1 and a 4 s breathe on CH2, SCENE 100 % */
static uint8_t r6_fx_image(void)
{
    static const uint8_t roles0[5] = { ULTRA_ROLE_BRIGHTNESS, 0, 0, 0, 0 }, roles1[5] = { 0, ULTRA_ROLE_BRIGHTNESS, 0, 0, 0 };
    static const uint16_t a5[5] = { 65535, 65535, 65535, 65535, 65535 }, mn[5] = { 0, 0, 0, 0, 0 };
    uint8_t s[32], crc[ULTRA_LIGHTS], m = 0, seq = 1;
    uint32_t i;
    r5_page_erase(SCENE_FX_ADDR);
    r5_page_erase(SCENE_FX_ADDR + 256u);
    r5_page_erase(SCENE_FX_B_ADDR);
    r5_page_erase(SCENE_FX_B_ADDR + 256u);
    ultra_init(&r5_u, 1);
    ultra_slot_light_def(s, 0, seq++, roles0, 1000, 0, mn, a5); (void)ultra_on_slot(&r5_u, s, 0);
    ultra_slot_light_def(s, 1, seq++, roles1, 1000, 0, mn, a5); (void)ultra_on_slot(&r5_u, s, 0);
    ultra_slot_scene(s, 0, seq++, 1, 10, 65535, a5, a5); (void)ultra_on_slot(&r5_u, s, 0);
    ultra_slot_scene(s, 1, seq++, 1, 10, 65535, a5, a5); (void)ultra_on_slot(&r5_u, s, 0);
    ultra_slot_scene_effect(s, 0, seq++, ULTRA_FX_CANDLE, P_CANDLE, 4); (void)ultra_on_slot(&r5_u, s, 0);
    ultra_slot_scene_effect(s, 1, seq++, ULTRA_FX_BREATHE, P_BREATHE, 7); (void)ultra_on_slot(&r5_u, s, 0);
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        crc[i] = 0;
        if (r5_u.light[i].scene_fx_have && ultra_scene_fx_ok(&r5_u.light[i])) {
            crc[i] = crc8_smbus((const uint8_t *)&r5_u.light[i], ULTRA_HEAD_BYTES);
            m |= (uint8_t)(1u << i);
        }
    }
    (void)scene_fx_store(&r5_u, m, crc);
    return m;
}

/* what the flash holds for the power-on effects (a reboot's load) */
static uint8_t r6_fx_stored(void)
{
    uint8_t crc[ULTRA_LIGHTS];
    return scene_fx_load(&r5_w, crc);
}

static uint32_t r6_count(const char *hay, const char *needle)
{
    uint32_t n = 0;
    for (; (hay = strstr(hay, needle)) != 0; hay++) {
        n++;
    }
    return n;
}

/* the main loop's statistics of a stretch: ms late and flash stall time */
typedef struct { uint64_t ns0, stall0; uint32_t late0; } r6_stat_t;
static void r6_stat_begin(r6_stat_t *s) { s->ns0 = sim_ns; s->stall0 = sim_stall_us; s->late0 = ovr[2]; }
static double r6_stall_share(const r6_stat_t *s) { return (double)(sim_stall_us - s->stall0) * 1000.0 / (double)(sim_ns - s->ns0); }

/* an UPD frame (a PING: every board forwards it, none takes it) on the wire */
static void r6_upd_ping(uint8_t session)
{
    uint8_t f[UPD_FRAME_BYTES];
    uint64_t t0 = sim_ns + 7000u;
    while (wf_on) {
        run_ticks(1);
    }
    upd_build_frame(f, UPD_PING, session, 0, 0x11, 0, 0, host_us(t0 + 384u * 1250u));
    wire_frame(f, UPD_FRAME_BITS, t0);
    run_ms(20);
}

/* ====================================================================================================== r6e2e */
static void sc_r6e2e(void)
{
    win_t w;
    uint32_t m, k, w0, c0, h0 = 0, fx0, t_trip, fe0;
    (void)h0;
    printf("soak r6e2e: power-on effect -> host -> calibration -> trip -> factory reset (host lit) -> update session\n");
    m = r6_fx_image();
    fx0 = W_FX();
    CHECK(m == 3u && r6_fx_stored() == 3u, "e2e: the effects stored before the boot (%X)", m);
    boot(0, 0, 0);
    CHECK(fx_mask == 3u && fx_ch == 3u, "e2e: the power-on effects run from boot (mask %X, channels %X)", fx_mask, fx_ch);
    run_ms(1500);
    win_begin(&w);
    run_ms(2000);
    win_end();
    CHECK(w.chg[0] > 500u && w.chg[1] > 500u && out_now[2] == 0u, "e2e: candle (CH1) and breathe (CH2) move, CH3 dark "
          "(%u, %u, %u)", w.chg[0], w.chg[1], out_now[2]);
    /* 1. ESPHome arrives (NOP frames): the lock, the power-on effect's take-over SCENE_TAKEOVER_MS after the first
     *    slot (0.6.7, F8: 15 s, each light at its own SET before that; 0.6.6: 3 s after the first header), then its
     *    lights */
    host_on = 1;
    run_ms(1000);
    CHECK(ultra_on && fx_ch == 3u && ul.light[0].scene_run, "e2e: locked, the effect still runs 1 s in (%u %X)", ultra_on,
          fx_ch);
    run_ms(SCENE_TAKEOVER_MS);
    CHECK(fx_ch == 0u && !ul.light[0].scene_run && !ul.light[1].scene_run, "e2e: the take-over SCENE_TAKEOVER_MS after "
          "the first slot ends the power-on effect (%X)", fx_ch);
    run_ms(1500);
    CHECK(out_now[0] == 0u && out_now[1] == 0u, "e2e: no light defined by the host: dark after the scene fade (%u %u)",
          out_now[0], out_now[1]);
    cmd_def(0, 0, 3, 1000);
    r6_cmd_set_rgb(0, 39321, 65535, 65535, 65535, 0, 0);          /* white 60 % */
    run_ms(2000);
    CHECK(out_now[0] == 39321u && out_now[1] == 39321u && out_now[2] == 39321u, "e2e: the host's light (%u %u %u)",
          out_now[0], out_now[1], out_now[2]);
    run_ms(3000);
    CHECK(W_SETTINGS() == 1u && set.last_proto == PROTO_ULTRA, "e2e: the sticky protocol stored (%u)", W_SETTINGS());
    /* 2. a calibration test from the host (IDENTIFY 2) while lit: stored with the hour's bright token */
    c0 = W_CAL();
#if MON_EXTRA
    h0 = W_HIST();
#endif
    cmd_identify(ULTRA_IDENT_CAL, 0);
    run_ms(3000);
    CHECK(is.calibrated && cal_save_pend == 1u && W_CAL() == c0, "e2e: the test done, its record waits (lit, the "
          "token taken by the sticky write) (%u, pend %u)", is.calibrated, cal_save_pend);
    CHECK(out_now[0] == 39321u, "e2e: the light is back after the test (%u)", out_now[0]);
    burst_next_ns = sim_ns;
    run_sparse(3700u, 20u);
    CHECK(W_CAL() == c0 + 1u && cal_present && !cal_save_pend, "e2e: the calibration stored within the hour (%u)",
          W_CAL() - c0);
#if MON_EXTRA
    run_sparse(3700u, 20u);
    CHECK(W_HIST() == h0 + 1u && !chist_pend && chist.n == 1u, "e2e: the calibration history stored the hour after (%u, "
          "pend %u)", W_HIST() - h0, chist_pend);
#endif
    /* 3. a short: the fast trip, the retry a second later, the soft start */
    host_next = sim_ticks;
    run_ms(200);
    expect_trips = 1;
    short_ticks = 3;
    t_trip = up_s;
    run_ms(50);
    CHECK(sim_forced && is.tripped == ISENSE_TRIP_FAST, "e2e: tripped (%u %u)", sim_forced, is.tripped);
    run_ms(2500);
    CHECK(!sim_forced && is.tripped == ISENSE_TRIP_NONE && out_now[0] == 39321u && pw.short_trips == 1u,
          "e2e: the retry brought the light back (%u %u %u, trips %u)", sim_forced, is.tripped, out_now[0],
          (unsigned)pw.short_trips);
    (void)t_trip;
    /* 4. a factory reset from the console while the host keeps the light on */
    w0 = W_SETTINGS();
    k = W_SCENE();
    con_out_n = 0;
    console("defaults");
    CHECK(strstr(con_out, "ok") != 0 && fx_mask == 0u && fx_clear_pend && !cal_present && cal_save_pend == 2u &&
          sext.ilimit50 == 0xFFu && scene_want.mask == 0u && set.protocol == PMODE_AUTO,
          "e2e: the reset's RAM state (fx %X, clear %u, cal %u / %u, limit %02X, scene %u, protocol %u)", fx_mask,
          fx_clear_pend, cal_present, cal_save_pend, sext.ilimit50, scene_want.mask, set.protocol);
    run_ms(6000);                                        /* the factory pattern (2.7 s), then the host's light again */
    CHECK(ultra_on && out_now[0] == 39321u && !signal_lost, "e2e: Ultra stays locked, the host's light is back after "
          "the pattern (%u, %u)", out_now[0], ultra_on);
    /* the pattern's final dark second is the low moment the erase needs (main.c): the effect store is cleared and the
     * default settings written there, although the host keeps the light on before and after */
    CHECK(r6_fx_stored() == 0u && !fx_clear_pend, "e2e: the effect store erased in the pattern's dark second (%u lights "
          "stored, pending %u)", r6_fx_stored(), fx_clear_pend);
    CHECK(W_SETTINGS() >= w0 + 1u, "e2e: the default settings stored at once (0.6.7, F10: with the void bits) (%u)",
          W_SETTINGS() - w0);
    CHECK(scene_want.mask == 0u && !scene_pend && W_SCENE() == k, "e2e: the scene: none stored before, none now, no "
          "record needed (%u written, pending %u)", W_SCENE() - k, scene_pend);
    burst_next_ns = sim_ns;
    run_sparse(3700u, 20u);                              /* an hour: the next bright token */
    printf("  1 h after the reset, the host lit: settings +%u, scene +%u, cal record cleared %u, effect store erased "
           "%u\n", W_SETTINGS() - w0, W_SCENE() - k, (unsigned)!cal_present && !cal_save_pend,
           (unsigned)(r6_fx_stored() == 0u));
    CHECK(!settings_dirty && STORE_VOIDS(&sext) == 0xFFu, "e2e: an hour on, the clears stored and the settings with them "
          "(dirty %u, voids %02X)", settings_dirty, STORE_VOIDS(&sext));
    CHECK(!cal_save_pend && !cal_present, "e2e: the calibration's cleared record stored within the hour (pending %u)",
          cal_save_pend);
    /* 5. an update session passes through (a PING burst: no bootloader here, so the board only forwards): the light
     * holds, transparency ends 2 s after the last UPD frame, the gate plan comes back */
    host_on = 0;
    while (wf_on) {
        run_ticks(1);
    }
    run_ms(10);
    fe0 = frames;
    win_begin(&w);
    for (k = 0; k < 6u; k++) {
        r6_upd_ping(0x42);
    }
    CHECK(upd_app_transparent(&upd) && plan_buf[plan_idx].own_bits == PROTO_UPD_BITS && upd.frames >= 3u,
          "e2e: transparent after the UPD frames (%u, own %u, frames %u)", upd_app_transparent(&upd),
          plan_buf[plan_idx].own_bits, upd.frames);
    run_ms(2500);
    win_end();
    CHECK(!upd_app_transparent(&upd) && plan_buf[plan_idx].own_bits == ULTRA_SLOT_BITS, "e2e: back to the Ultra plan "
          "2 s after the session (%u, own %u)", upd_app_transparent(&upd), plan_buf[plan_idx].own_bits);
    CHECK(w.chg[0] == 0u && w.chg[1] == 0u && w.chg[2] == 0u && out_now[0] == 39321u && !signal_lost,
          "e2e: the light held through the session (%u %u %u changes, lost %u)", w.chg[0], w.chg[1], w.chg[2],
          signal_lost);
    CHECK(frames >= fe0 + 6u, "e2e: the UPD frames were tracked as frames (%u)", frames - fe0);
    host_on = 1;
    host_next = sim_ticks;
    run_ms(2000);
    CHECK(ultra_on && out_now[0] == 39321u && !signal_lost, "e2e: the host's frames again, the light unchanged");
    CHECK(W_FX() == fx0 || r6_fx_stored() == 0u, "e2e: the effect store was never rewritten by the board (%u -> %u)",
          fx0, W_FX());
    CHECK(sim_led_on == 0u || 1, "(D9 blinks for the patterns: not checked)");
}

/* =================================================================================================== r6fxreset */
/* No host at all: the stored power-on effects and scene show from power-up (bright). A factory reset by TP9's gesture
 * (the console's `defaults` takes the same path) must leave a board whose flash holds no effect and no scene, so that a
 * power cut afterwards doesn't bring them back (the factory pattern's final dark second is the flash's low moment),
 * and a board that shows what a reset board shows: nothing until data (0.6.6 keeps the effect and the scene running
 * from RAM until a power cycle, while its flash says "none"). */
/* r6fxreset <tp9 | host | console> (0.6.7, F10, Quindor: "a factory reset needs to look like a reboot"): stored
 * settings, a power-on scene, power-on effects, a calibration (and its history) from boot; then the factory reset: tp9 =
 * the jig pad's gesture with no host (the power-on effect bright; the receiver: TP6), host = the gesture while an Ultra
 * host keeps its light at 100 %, console = `defaults` while the host keeps it at 100 %. For 3 s after it, the flash as a
 * power cut at any ms would leave it (each time it changed): main.c records_load on it loads nothing the reset cleared
 * (the settings at their defaults, no scene, no effect, no calibration, an empty history; the image before the reset:
 * all of them, the check's control). Without a host the board is dark after the pattern; within a minute the clears
 * are stored. (fable-sys B2 / fable-fw B5: 0.6.6 showed the effect and the scene from RAM until a power cycle, and its
 * clears waited for a low moment a lit host never gave: a power cut brought them back.) */
#define R6_IMG_MAX 48u
static uint8_t *r6_img[R6_IMG_MAX];
static uint32_t r6_img_ms[R6_IMG_MAX], r6_imgs;

static void r6_img_take(uint32_t ms)
{
    if (r6_imgs < R6_IMG_MAX) {
        r6_img[r6_imgs] = (uint8_t *)malloc(FL_SIZE);
        memcpy(r6_img[r6_imgs], fl, FL_SIZE);
        r6_img_ms[r6_imgs++] = ms;
    }
}

/* records_load on the flash as it is: how many of the five cleared things it brings back */
static uint32_t r6_old_loads(void)
{
    settings_t d;
    uint32_t n = 0;
    int cal;
    settings_default(&d);
    cal = records_load();
    n += set.raw_steps != d.raw_steps || set.min_pulse != d.min_pulse;
    n += scene_stored.mask != 0u;
    n += fx_mask != 0u;
    n += cal == 0 && ((const isense_cal_t *)core_scratch)->levels != 0u;
#if MON_EXTRA
    n += chist.n != 0u;
#endif
    return n;
}

static void r6_pad_low(uint32_t low)                     /* the jig pad: TP9 (PB10), the receiver's TP6 (PA3) */
{
#if BOARD == BOARD_D2A_ULTRA
    soak_gpiob.INDR = low ? 0xFFFFFBFFu : 0xFFFFFFFFu;
#else
    soak_tp6_low = (uint8_t)low;
#endif
}

static void sc_r6fxreset(const char *how)
{
    settings_t st, sg;
    store_ext_t ex, eg;
    isense_cal_t c, cg;
    uint16_t cnt;
    scene_t sc, scg;
    uint32_t k, ops, t, lit, n_old, n_pre, host = strcmp(how, "tp9") != 0, all;
    uint8_t stored;
#if MON_EXTRA
    calhist_t h;
#endif
    printf("soak r6fxreset %s: settings, a scene, effects, a calibration stored; the factory reset (%s); a power cut "
           "at every ms of the next 3 s\n", how, !host ? "the pad's gesture, no host" : !strcmp(how, "host") ?
           "the pad's gesture, the host's light at 100 %" : "`defaults`, the host's light at 100 %");
    (void)settings_load(&sg, &eg);                       /* (the stores' positions) */
    r5_settings_val(5u, &st, &ex);
    ex.ilimit50 = 0xFFu;                                 /* (the board's rating: r5_settings_val's limit would dim it) */
    (void)settings_store(&st, &ex);
    (void)cal_load(&cg, &cnt);
    r5_cal_val(5u, &c);
    (void)cal_store(&c, &cnt);
    (void)scene_load(&scg);
    scene_clear(&sc);                                    /* CH3..5 at 60 % */
    sc.mask = 0x1Cu;
    sc.v[2] = sc.v[3] = sc.v[4] = 39321u;
    (void)scene_store(&sc);
#if MON_EXTRA
    memset(&h, 0, sizeof h);
    (void)calhist_load(&h);
    h.n = 1;
    h.tests = 1;
    h.e[0].ma[0] = 1000;
    (void)calhist_store(&h);
#endif
    (void)r6_fx_image();
    boot(0, 0, 0);
    CHECK(fx_mask == 3u && fx_ch == 3u && scene_stored.mask == 0x1Cu && cal_present && set.raw_steps == st.raw_steps,
          "fxreset: everything stored loads at boot (fx %X, scene %X, cal %u, steps %u)", fx_mask, scene_stored.mask,
          cal_present, set.raw_steps);
    if (host) {
        host_on = 1;                                     /* an Ultra host: its light at 100 % */
        cmd_def(0, 0, 3, 1000);
        cmd_set(0, 65535, 0, 0);
    }
    run_ms(15000);                                       /* (past LIFE_PREP_S; the outputs bright all along) */
    CHECK(host ? out_now[0] == 65535u : out_now[2] == 39321u, "fxreset: lit before the reset (%u %u)", out_now[0],
          out_now[2]);
    r6_img_take(0);                                      /* (the image before the reset: the check's control) */
    if (!strcmp(how, "console")) {
        con_out_n = 0;
        con_in_r = con_in_w = 0;
        strcpy(con_in, "defaults\r");
        con_in_w = 9u;
    } else {
        r6_pad_low(1);                                   /* the gesture: 6 s low, release, 1.2 s low within 5 s */
        run_ms(6000);
        r6_pad_low(0);
        run_ms(500);
        r6_pad_low(1);
    }
    for (k = 0; k < 3000u && !fx_clear_pend; k++) {
        run_ms(1);                                       /* to the reset's ms */
    }
    CHECK(fx_clear_pend && fx_mask == 0u && scene_want.mask == 0u, "fxreset: the reset ran (fx %X, scene %u)", fx_mask,
          scene_want.mask);
    ops = fl_ops;
    r6_img_take(1);                                      /* (the reset's ms) */
    for (t = 1; t <= 3000u; t++) {
        if (t == 1200u && strcmp(how, "console") != 0) {
            r6_pad_low(0);                               /* (the second hold ends) */
        }
        run_ms(1);
        if (fl_ops != ops) {
            ops = fl_ops;
            r6_img_take(t + 1u);                         /* the flash changed in this ms */
        }
    }
    r6_pad_low(0);
    run_ms(60000);                                       /* a minute: the pattern and its dark second, low moments */
    stored = r6_fx_stored();
    lit = (uint32_t)out_now[0] | out_now[1] | out_now[2] | out_now[3] | out_now[4];
    printf("  60 s after: effect store %u lights (clear pending %u), scene pending %u, cal pending %u, voids %02X, "
           "outputs %u/%u/%u/%u/%u, effect channels %X, hold %X; %u flash images in the 3 s\n", stored, fx_clear_pend,
           scene_pend, cal_save_pend, STORE_VOIDS(&sext), out_now[0], out_now[1], out_now[2], out_now[3], out_now[4],
           fx_ch, srun.hold, r6_imgs);
    CHECK(fx_ch == 0u && srun.hold == 0u && (host || lit == 0u), "fxreset: like a reboot: no effect, no scene, %s "
          "(outputs %u/%u/%u/%u/%u, effect channels %X, hold %X)", host ? "the host's light" : "dark until data",
          out_now[0], out_now[1], out_now[2], out_now[3], out_now[4], fx_ch, srun.hold);
    if (!host) {
        CHECK(stored == 0u && !fx_clear_pend && !scene_pend && !cal_save_pend && STORE_VOIDS(&sext) == 0xFFu,
              "fxreset: the clears stored within a minute (effects %u, pending %u %u %u, voids %02X)", stored,
              fx_clear_pend, scene_pend, cal_save_pend, STORE_VOIDS(&sext));
    }
    /* every image: the boot on it */
    all = 0;
    for (k = 0; k < r6_imgs; k++) {
        memcpy(fl, r6_img[k], FL_SIZE);
        n_old = r6_old_loads();
        if (k == 0u) {
            n_pre = n_old;
            CHECK(n_pre == 4u + MON_EXTRA, "fxreset: the image before the reset loads all of them (%u)", n_pre);
        } else {
            all += n_old != 0u;
            CHECK(n_old == 0u, "fxreset: a power cut %u ms after the reset brings back %u of the cleared things",
                  r6_img_ms[k], n_old);
        }
        free(r6_img[k]);
    }
    printf("  a power cut at any ms after the reset: %u of %u images bring anything back\n", all, r6_imgs - 1u);
    (void)n_pre;
}

/* ================================================================================================== r6ws2ultra */
static void sc_r6ws2ultra(void)
{
    static const uint8_t green[5] = { 0x00, 0xFF, 0x00, 0x00, 0x00 };
    static const uint8_t white[5] = { 0x00, 0x00, 0x00, 0x33, 0x33 };
    uint32_t k, j, max_g = 0, max_b = 0, first_white = 0;
    printf("soak r6ws2ultra: a WS2805 controller, then the Ultra hub, then the WS2805 controller again\n");
    boot(0, 0, 0);
    run_ms(500);
    for (k = 0; k < 25u; k++) {                          /* A: WS2805 at 50 fps, green */
        r6_ws2805_frame(green, 20);
    }
    CHECK(proto_active(&frx.ps) == PROTO_WS2805 && out_now[1] == 65535u && out_now[0] == 0u, "ws2ultra: WS2805 locked, "
          "green (%u, %u)", proto_active(&frx.ps), out_now[1]);
    /* B: the controller is replaced by the Ultra hub: its preamble (NOP frames here: the WS-locked board shows their
     * first bytes as a colour for 3 frames, as it would the hub's DARK slots), then its commands */
    host_on = 1;
    host_next = sim_ticks;
    run_ms(500);
    CHECK(ultra_on && proto_active(&frx.ps) == PROTO_ULTRA, "ws2ultra: Ultra locked after the preamble (%u)", ultra_on);
    cmd_def(0, 0, 3, 1000);
    r6_cmd_set_rgb(0, 32768, 0, 0, 65535, 0, 0);          /* blue 50 % */
    run_ms(3000);
    CHECK(ultra_on && out_now[2] == 32768u && out_now[1] == 0u, "ws2ultra: Ultra: blue, no green (%u, %u)", out_now[2],
          out_now[1]);
    /* C: the hub is replaced by the WS2805 controller again (white 20 % on W1 / W2): between the lock and the first
     * WS frame shown, the outputs must not show the green of phase A, nor the blue of phase B jump up */
    host_on = 0;
    while (wf_on) {
        run_ticks(1);
    }
    run_ms(5);
    for (k = 0; k < 25u; k++) {
        wire_frame(white, 40u, sim_ns + 7000u);
        for (j = 0; j < 20u; j++) {
            run_ticks(20);
            if (out_now[1] > max_g) {
                max_g = out_now[1];
            }
            if (out_now[2] > max_b) {
                max_b = out_now[2];
            }
            if (!first_white && out_now[3] != 0u) {
                first_white = k + 1u;
            }
        }
    }
    printf("  phase C: locked %u, out %u/%u/%u/%u/%u; the highest green seen %u, blue %u; W1 from frame %u\n",
           proto_active(&frx.ps), out_now[0], out_now[1], out_now[2], out_now[3], out_now[4], max_g, max_b, first_white);
    CHECK(proto_active(&frx.ps) == PROTO_WS2805 && out_now[3] == 0x3333u && out_now[4] == 0x3333u && !ultra_on,
          "ws2ultra: WS2805 again, white 20 %% (%u, %u)", proto_active(&frx.ps), out_now[3]);
    CHECK(max_g == 0u, "ws2ultra: the green from before the Ultra phase never reappears at the switch back (max %u; "
          "0.6.6 shows the stale WS values for a frame, then ramps from them)", max_g);
    CHECK(max_b <= 32768u, "ws2ultra: the blue never rises at the switch (%u)", max_b);
    CHECK(events_seen == 0u && is.trips == 0u, "ws2ultra: no false events (%X)", events_seen);
}

/* ====================================================================================================== r6wall */
/* One session of the wall-switch pattern: `soak_tests.exe r6wall <n> <image>`; session 1 starts the image (a stored
 * scene: every channel at 50 %), the others load it: the flash as the sessions before left it, the RAM fresh (a real
 * power cut). 150 s lit (no controller), then the supply goes (a dip): the lifetime record's dip write, and in session
 * 5 every store with a write pending (how many operations one dip has to carry). Checks at boot: the record the
 * sessions before stored (one write per session: power-ups, dips, lit time), its pages erased only at power-up. */
static int r6_image_load(const char *path)
{
    FILE *f = fopen(path, "rb");
    size_t n;
    if (!f) {
        return 0;
    }
    n = fread(fl, 1, FL_SIZE, f);
    fclose(f);
    return n == FL_SIZE;
}

static void r6_image_save(const char *path)
{
    FILE *f = fopen(path, "wb");
    if (f) {
        fwrite(fl, 1, FL_SIZE, f);
        fclose(f);
    }
}

static void sc_r6wall(int n, const char *image)
{
    scene_t sc;
    scene_rec_t rec;
    life_rec_t r;
    life_pos_t p;
    uint32_t found, e0, e1, ops0, wl0, ws0, wsc0, wc0, lit0, i;
    printf("soak r6wall: session %u (%s)\n", (unsigned)n, image);
    if (n <= 1 || !r6_image_load(image)) {
        if (n > 1) {
            printf("  (no image: starting afresh)\n");
        }
        memset(fl, 0xFF, FL_SIZE);
        scene_clear(&sc);
        sc.mask = 0x1Fu;
        for (i = 0; i < 5u; i++) {
            sc.v[i] = 32768u;
        }
        scene_log_pack(&sc, 1, &rec);
        memcpy(fl + (SCENE_LOG_ADDR - FL_BASE), &rec, sizeof rec);
    }
    found = life_find((const uint8_t *)LIFE_PAGE0_ADDR, (const uint8_t *)LIFE_PAGE1_ADDR, &r, &p);
    e0 = fl_erases[page_of(LIFE_PAGE0_ADDR)];
    e1 = fl_erases[page_of(LIFE_PAGE1_ADDR)];
    boot(0, 0, 0);
    printf("  boot: record found %u (writes %u, ups %u, dips %u, lit %u s, on %u s), erases at power-up: A %u, B %u, "
           "pending now %u\n", found, found ? r.writes : 0u, found ? r.ups : 0u, found ? r.dips : 0u,
           found ? r.lit_s : 0u, found ? r.on_s : 0u, fl_erases[page_of(LIFE_PAGE0_ADDR)] - e0,
           fl_erases[page_of(LIFE_PAGE1_ADDR)] - e1, life_pending());
    if (n > 1) {
        CHECK(found && r.writes == (uint32_t)(n - 1) && r.ups == (uint32_t)(n - 1) && r.dips == (uint32_t)(n - 1),
              "wall %d: every session before stored one record at its power cut (writes %u, ups %u, dips %u)", n,
              r.writes, r.ups, r.dips);
        CHECK(r.lit_s >= (uint32_t)(n - 1) * 148u && r.lit_s <= (uint32_t)(n - 1) * 152u + 20u, "wall %d: the lit "
              "time of the sessions before (%u s, %u x ~150; session 5 runs ~12 s longer)", n, r.lit_s, n - 1);
    } else {
        CHECK(!found, "wall 1: no record yet");
    }
    CHECK(en.lt.ups == (uint32_t)n, "wall %d: this power-up counted (%u)", n, en.lt.ups);
    CHECK(life_pending() != 2u, "wall %d: the record's next entry is erased (an erase due was done at power-up: %u)", n,
          life_pending());
    run_ms(2000);
    CHECK(out_now[0] == 32768u && out_now[4] == 32768u && srun.hold == 0x1Fu, "wall %d: the stored scene lit (%u)", n,
          out_now[0]);
    lit0 = en.lt.lit_s;
    wl0 = W_LIFE();
    ws0 = W_SETTINGS();
    wsc0 = W_SCENE();
    wc0 = W_CAL();
    run_sparse(60u, 0);
    if (n == 5) {
        /* every store with a write pending when the power goes: the settings (the hour's bright token spent by the
         * first change, the second waits), the scene (`set scene 1`), a calibration test and (M2) its history */
        console("set dither 1");
        run_ms(3000);
        CHECK(W_SETTINGS() == ws0 + 1u, "wall 5: the first change written with the boot's bright token (%u)",
              W_SETTINGS() - ws0);
        console("set dither 2");
        run_ms(3000);
        CHECK(W_SETTINGS() == ws0 + 1u && settings_dirty, "wall 5: the second waits (%u)", W_SETTINGS() - ws0);
        console("set scenefade 20");                     /* (`scene 1` would equal the stored one) */
        run_ms(6000);
        CHECK(scene_pend && W_SCENE() == wsc0, "wall 5: the scene waits (%u)", W_SCENE() - wsc0);
        console("set ical 50");
        run_ms(isense_cal_ms((uint16_t)(50u * 65535u / 100u)) + 500u);
        CHECK(cal_save_pend == 1u && W_CAL() == wc0, "wall 5: the calibration record waits (%u)", W_CAL() - wc0);
    }
    run_sparse(88u, 0);
    CHECK(W_LIFE() == wl0, "wall %d: no lifetime write while lit without a dip (%u)", n, W_LIFE() - wl0);
    /* the wall switch: the supply falls; the hold-up time carries the dip writes */
    ops0 = sim_stalls;
    {
        uint32_t ws1 = W_SETTINGS(), hw0 = 0;
#if MON_EXTRA
        hw0 = W_HIST();
#endif
        (void)hw0;
        supply_mv = 18000u;
        run_ms(40);
        printf("  the power cut: %u flash operations in the dip (lifetime +%u, settings +%u, scene +%u, cal +%u"
#if MON_EXTRA
               ", history +%u"
#endif
               "), lit this session %u s\n", sim_stalls - ops0, W_LIFE() - wl0, W_SETTINGS() - ws1, W_SCENE() - wsc0,
               W_CAL() - wc0,
#if MON_EXTRA
               W_HIST() - hw0,
#endif
               en.lt.lit_s - lit0);
    }
    CHECK(W_LIFE() == wl0 + 1u && vdip.in_dip, "wall %d: the lifetime record written at the dip (%u)", n, W_LIFE() - wl0);
    if (n == 5) {
        CHECK(W_SETTINGS() == ws0 + 2u && W_SCENE() == wsc0 + 1u && W_CAL() == wc0 + 1u && !settings_dirty && !scene_pend
              && !cal_save_pend, "wall 5: every pending write went out at the dip (%u %u %u)", W_SETTINGS() - ws0,
              W_SCENE() - wsc0, W_CAL() - wc0);
        printf("  (one dip carried %u operations: the hold-up must cover their sum, not one record)\n", sim_stalls - ops0);
    }
    CHECK(fl_lit_erases == 0u, "wall %d: no erase while lit (%u)", n, fl_lit_erases);
    r6_image_save(image);
}

/* ================================================================================================= r6flashfail */
/* Every operation fails (write protection on the record sectors, a worn page, a flash that never says EOP): the
 * records stay pending, as documented. 0.6.6 retries an erase ahead every ms while low (16 ms of stall each) and the
 * calibration history's write every pass: the main loop spends >= 90 % of its time in failing operations, the console
 * says `flash: error` every ms. */
static void sc_r6flashfail(void)
{
    settings_t s;
    store_ext_t e;
    r6_stat_t st;
    uint32_t k, errs0, b0;
    double share;
    printf("soak r6flashfail: every flash operation fails; an erase ahead is due\n");
    settings_default(&s);
    store_ext_clear(&e);
    r5_page_erase(SETTINGS_ADDR);
    for (k = 0; k < 3u; k++) {                           /* page A: 3 records, the newest last */
        store_pack_ext(&s, &e, 1u + k, (store_slot_t *)(fl + (SETTINGS_ADDR - FL_BASE) + k * STORE_SLOT_BYTES));
    }
    memset(fl + (SETTINGS_B_ADDR - FL_BASE), 0x00, 256u);   /* page B: older records (not erased): an erase ahead is due */
    fl_fail_all = 1;
    boot(0, 0, 0);
    CHECK(settings_pending() == 1u, "flashfail: the settings' other page is due for its erase ahead (%u)", settings_pending());
    /* the host: a dim light (5 %: the flash counts the outputs as low), frames at 50 fps */
    host_on = 1;
    cmd_def(0, 0, 3, 1000);
    cmd_set(0, 3277, 0, 0);
    run_ms(10500);                                       /* LIFE_PREP_S: the erases ahead start */
    CHECK(ultra_on && out_now[0] == 3277u, "flashfail: the dim light (%u)", out_now[0]);
    errs0 = sim_stalls;                                  /* (every attempt is one flash_op: the model counts them; the
                                                            receiver's build has no D5 error counter) */
    b0 = tx_blocks;
    k = stat.decoded;
    con_out_n = 0;
    con_out[0] = 0;
    r6_stat_begin(&st);
    run_ms(5000);
    share = r6_stall_share(&st);
    cmd_set(0, 6554, 0, 0);                              /* a host command: 10 % */
    run_ms(500);
    printf("  5 s dim with a failing flash: %u frames decoded of ~250 sent, %.1f %% of the time in flash operations, "
           "%u failed operations, %u `flash: error` lines, %u bytes written past the UART's buffers; the host's next "
           "command applied: %u\n", stat.decoded - k, share * 100.0, sim_stalls - errs0,
           r6_count(con_out, "flash: error"), tx_blocks - b0, out_now[0] == 6554u);
    CHECK(stat.decoded - k >= 200u && out_now[0] == 6554u, "flashfail: the data path keeps working while the flash "
          "fails (%u frames decoded, the command applied %u; 0.6.6 retries the erase every ms: every frame lost)",
          stat.decoded - k, out_now[0] == 6554u);
    CHECK(tx_blocks == b0, "flashfail: the console never stalls the main loop (%u blocked bytes)", tx_blocks - b0);
    CHECK(sim_stalls - errs0 <= 2u, "flashfail: a failed operation is retried with a hold-off (%u retries in 5 s)",
          sim_stalls - errs0);
    (void)share;
    /* the console's save still tries at once (a user's command; the dip write likewise) */
    k = sim_stalls;
    con_out_n = 0;
    console("save");
    CHECK(sim_stalls > k && strstr(con_out, "failed") != 0, "flashfail: `save` tries at once and reports failed "
          "(%u, %.20s)", sim_stalls - k, con_out);
    fl_fail_all = 0;
    run_sparse(3u * 3600u + 30u, 0);                     /* the flash works again: the erase ahead, then the settings
                                                            (their budget's tokens went into the failed retries) */
    CHECK(settings_pending() == 0u && !settings_dirty, "flashfail: the flash back: the erase ahead and the settings "
          "done (%u, %u)", settings_pending(), settings_dirty);
}

/* ====================================================================================================== r6boot5 */
/* Every store has an erase ahead possible at power-up (its page holds the newest record, the other page old ones):
 * main.c runs flash_erase_ahead a fixed number of times at boot. 0.6.6: four, one fewer than the stores (M2 added the
 * fifth): the calibration history's waits for a low moment, which a bright power-on scene never gives. */
static void sc_r6boot5(void)
{
    settings_t s;
    store_ext_t e;
    isense_cal_t c;
    scene_t sc;
    scene_rec_t rec;
    energy_t en2;
#if MON_EXTRA
    calhist_t h;
#endif
    uint32_t k, pend = 0, i;
    static const uint32_t a[] = { SETTINGS_ADDR, CAL_PAGE_ADDR, SCENE_LOG_ADDR, LIFE_PAGE0_ADDR
#if MON_EXTRA
                                  , CALHIST_ADDR                 /* (0.6.4's layout has no history pair) */
#endif
                                };
    static const uint32_t b[] = { SETTINGS_B_ADDR, CAL_B_ADDR, SCENE_B_ADDR, LIFE_PAGE1_ADDR
#if MON_EXTRA
                                  , CALHIST_B_ADDR
#endif
                                };
    printf("soak r6boot5: every store with an erase ahead due at power-up, a bright power-on scene\n");
    for (k = 0; k < sizeof a / sizeof a[0]; k++) {
        r5_page_erase(a[k]);
        memset(fl + (b[k] - FL_BASE), 0x00, 256u);       /* the other page: old records */
    }
    settings_default(&s);
    store_ext_clear(&e);
    store_pack_ext(&s, &e, 1u, (store_slot_t *)(fl + (SETTINGS_ADDR - FL_BASE)));
    r5_cal_val(1, &c);
    cal_pack(&c, 1u, 1u, (cal_rec_t *)(fl + (CAL_PAGE_ADDR - FL_BASE)));
    scene_clear(&sc);
    sc.mask = 0x1Fu;
    for (i = 0; i < 5u; i++) {
        sc.v[i] = 32768u;
    }
    scene_log_pack(&sc, 1, &rec);
    memcpy(fl + (SCENE_LOG_ADDR - FL_BASE), &rec, sizeof rec);
    memset(&en2, 0, sizeof en2);
    en2.lt.magic = LIFE_MAGIC;
    en2.lt.on_s = 1000;
    energy_pack(&en2);
    memcpy(fl + (LIFE_PAGE0_ADDR - FL_BASE), &en2.lt, sizeof en2.lt);
#if MON_EXTRA
    memset(&h, 0, sizeof h);
    calhist_add(&h, 3600u, &c);
    calhist_pack(&h);
    memcpy(fl + (CALHIST_ADDR - FL_BASE), &h, sizeof h);
#endif
    boot(0, 0, 0);
    for (k = 0; k < REC_COUNT; k++) {
        pend += flash_rec_pending(k) != 0u;
    }
    printf("  after the boot's erases: %u of %u stores still have an erase due; B pages erased: settings %u, cal %u, "
           "scene %u, lifetime %u"
#if MON_EXTRA
           ", history %u"
#endif
           "\n", pend, (unsigned)REC_COUNT, fl_erases[page_of(SETTINGS_B_ADDR)], fl_erases[page_of(CAL_B_ADDR)],
           fl_erases[page_of(SCENE_B_ADDR)], fl_erases[page_of(LIFE_PAGE1_ADDR)]
#if MON_EXTRA
           , fl_erases[page_of(CALHIST_B_ADDR)]
#endif
           );
    CHECK(pend == 0u, "boot5: every store's erase ahead done at power-up, before the outputs start (%u left)", pend);
    run_ms(1000);
    CHECK(out_now[0] == 32768u, "boot5: the scene lit (%u)", out_now[0]);
    run_sparse(120u, 0);
    CHECK(fl_bright_erases == 0u, "boot5: no erase while the scene is lit (%u)", fl_bright_erases);
}

/* ====================================================================================================== r6rcdim */
#if BOARD_RETURN
static void sc_r6rcdim(void)
{
    rc_obs_t o;
    uint8_t p[32];
    uint32_t k, down = 0, up = 0, bad = 0, w0;
    uint16_t prev;
    printf("soak r6rcdim: the receiver answers POLLs while a 20 s dimming runs; `save` typed during a window\n");
    rc_boot_locked();
    cmd_def(0, 0, 3, 1000);
    cmd_set(0, 65535, 0, 0);
    run_ms(3000);
    cmd_set(0, 3277, 20000, ULTRA_SET_SMOOTH);           /* 100 -> 5 % over 20 s */
    run_ms(500);
    prev = out_now[0];
    for (k = 0; k < 30u; k++) {
        rc_poll_build(p, rc_seq++, 0, 2, ULTRA_PAGE_STATUS, 0, 0, 0);
        rc_window(p, 1, ULTRA_HDR_RW, 0, 0, &o);
        bad += !(rc_timing_ok(&o, p) && rc_answer_ok(&o, p) == ULTRA_PAGE_STATUS);
        host_on = 1;
        host_next = sim_ticks;
        run_ms(400);
        if (out_now[0] < prev) {
            down++;
        } else if (out_now[0] > prev) {
            up++;
        }
        prev = out_now[0];
    }
    printf("  30 windows during the dimming: %u bad, the light went down %u times, up %u, now %u\n", bad, down, up,
           out_now[0]);
    CHECK(bad == 0u && up == 0u && down >= 25u, "rcdim: every window answered, the dimming monotonic (%u bad, %u up)",
          bad, up);
    CHECK(rc_flash_in_window == 0u, "rcdim: no flash operation in a window (%u)", rc_flash_in_window);
    /* the console's `save` while a window is running: README 0.6.6 says it waits out the window */
    w0 = W_SETTINGS();
    cmd_config(ULTRA_CFG_SOFT_START, 0);                 /* something to store (persisted, like 22) */
    run_ms(500);
    CHECK(settings_dirty, "rcdim: a change pending (lit: the hour's token was the sticky write's)");
    {                                                    /* the RW frame (as rc_window), the line typed in its window */
        uint8_t f[32 * 2 + 8];
        uint64_t t_start;
        uint32_t guard = 0;
        rc_poll_build(p, rc_seq++, 0, 4, ULTRA_PAGE_STATUS, 2, 0, 0);   /* 4 slots at 250 kbaud: a 6.6 ms window */
        host_on = 0;
        while (wf_on) {
            run_ticks(1);
        }
        run_ms(1);
        memcpy(f, p, 32);
        t_start = sim_ns + rc_phase_ns;
        ultra_build_header_flags(f + 32, host_us(t_start + 320u * 1250u), ULTRA_HDR_RW);
        wire_frame(f, 320u, t_start);
        while (rc_state == RC_IDLE && ++guard < 40000u) {
            run_ticks(1);                                /* the frame, its end, the window armed */
        }
        while (rc_state == RC_ARMED && ++guard < 40000u) {
            run_ticks(1);                                /* turned around: inside the window */
        }
        CHECK(rc_state == RC_TURNED || rc_state == RC_CHECKED, "rcdim: inside the window (%u)", rc_state);
        con_out_n = 0;
        con_out[0] = 0;
        con_in_r = con_in_w = 0;
        memcpy(con_in, "save\r", 5u);
        con_in_w = 5u;                                   /* read by the main loop's next pass, in the window */
        while (rc_state != RC_IDLE && ++guard < 40000u) {
            run_ticks(1);
        }
        run_ms(100);
    }
    printf("  `save` typed inside a window: console says %s; settings written +%u\n",
           strstr(con_out, "pending") ? "pending" : strstr(con_out, "ok") ? "ok" : "?", W_SETTINGS() - w0);
    /* (0.6.7) fable-sys 05 (`save` waits out the window) not taken: the release console has no input (F12 moot); the
     * console answers pending and nothing is written inside the window */
    CHECK(W_SETTINGS() == w0 && strstr(con_out, "pending") != 0, "rcdim: `save` inside a window: pending, nothing "
          "written in it; %u written, console: %.24s", W_SETTINGS() - w0, con_out);
    host_on = 1;
    host_next = sim_ticks;
    run_ms(500);
    CHECK(ultra_on && !signal_lost, "rcdim: the hub's frames again");
}
#endif

static int r6s_scenario(const char *s, int argc, char **argv)
{
    if (!strcmp(s, "r6e2e")) {
        sc_r6e2e();
    } else if (!strcmp(s, "r6fxreset")) {
        sc_r6fxreset(argc > 2 ? argv[2] : "tp9");
    } else if (!strcmp(s, "r6ws2ultra")) {
        sc_r6ws2ultra();
    } else if (!strcmp(s, "r6wall")) {
        sc_r6wall(argc > 2 ? atoi(argv[2]) : 1, argc > 3 ? argv[3] : "r6wall.bin");
    } else if (!strcmp(s, "r6flashfail")) {
        sc_r6flashfail();
    } else if (!strcmp(s, "r6boot5")) {
        sc_r6boot5();
#if BOARD_RETURN
    } else if (!strcmp(s, "r6rcdim")) {
        sc_r6rcdim();
#endif
    } else {
        return 0;
    }
    return 1;
}
