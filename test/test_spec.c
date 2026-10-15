/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Spec-conformance tests (review round 4, "fable-spec"), included by test_main.c after test_ultra.c (it uses its
 * helpers: U, tk, hdr, send, rend, f16, def_light, set_light, fx_send, useq; UA / UB, c_setup, c_pair, u_same, cseq).
 *  1. signal loss "off" / "fade off" (CONFIG 8 = 1 / 2, main.c: ultra_all_off): the host back without a reboot repeats
 *     its state with the SEQ numbers already applied; the light must come back (the protocol wiki, Slot types, "SEQ": the refresh
 *     lets a board that missed something catch up). 0.6.2 ignores the refresh: the lights stay off.
 *  2. "on -> off: only M falls" (ULTRA_PROTOCOL.md "How it renders a SET", as ESPHome's transition): a SET to
 *     M = 0 whose A / B differ from the light's must not fade the colours. 0.6.2 lerps A and B as well.
 *  3. EFFECT / SCENE_EFFECT spare parameter bytes (the compatibility table: "ignored", the engine's state
 *     byte-identical): 0.6.2 keeps them in pend_p / scene_fxp, so a SCENE_EFFECT with spare bytes changes the stored
 *     head (its CRC) and costs a flash write.
 *  4. Documented behaviours that had no test: an effect on a light that is off stays off (breathe and strobe steps
 *     with their own M); STEP bounds (index >= count, count > 8: ignored); CONFIG 9's default. */
static void test_spec_loss_refresh(void)
{
    static const uint8_t mono[5] = { 0, 0, 0, 0, ULTRA_ROLE_BRIGHTNESS };
    uint16_t av[5] = { 65535, 65535, 65535, 65535, 65535 }, bv[5] = { 65535, 65535, 65535, 65535, 65535 }, o[5];
    uint8_t s[32];
    uint32_t ms, k;

    printf("spec: signal loss off / fade off, then the host's refresh (the same SEQ) restores the light\n");
    for (k = 0; k < 2; k++) {
        const char *what = k ? "fade off" : "off";
        uint32_t fade = k ? 1000u : 0u;
        ms = 1000;
        ultra_init(&U, 21);
        def_light(&U, ms, 1, mono, 1.0, 0, 0, 1);
        ultra_slot_set(s, 1, 77, 0, f16(0.8), av, bv, 0, 0);        /* the host's state, SEQ 77 */
        CHECK((send(&U, ms, s) & ULTRA_EV_STATE) != 0u, "%s: the state applied", what);
        rend(&U, ms + 10, o);
        CHECK(abs((int)o[4] - (int)f16(0.8)) <= 1, "%s: on at 80 %% (%u)", what, o[4]);
        CHECK(send(&U, ms + 1000, s) == 0u, "%s: the refresh while nothing happened: a repeat, ignored", what);
        /* 3 s without a frame: main.c's signal_loss_check calls ultra_all_off (CONFIG 8 = 1: at once, 2: a 1 s fade) */
        ms += 4000;
        ultra_all_off(&U, tk(ms), fade);
        if (fade) {
            rend(&U, ms + 500, o);
            CHECK(o[4] > 5000u && o[4] < 50000u, "%s: half way (%u)", what, o[4]);
        }
        rend(&U, ms + 1100, o);
        CHECK(o[4] == 0u, "%s: dark after the signal loss (%u)", what, o[4]);
        /* the host is back; its clock ran on (no reboot, no rebase): it repeats the current state with SEQ 77 */
        ms += 2000;
        CHECK((send(&U, ms, s) & ULTRA_EV_STATE) != 0u, "%s: the host's refresh applies after a signal-loss off", what);
        rend(&U, ms + 10, o);
        CHECK(abs((int)o[4] - (int)f16(0.8)) <= 1, "%s: the light is back at 80 %% (%u)", what, o[4]);
        /* the refresh of the other command types applies again too, with no effect on the state */
        def_light(&U, ms + 20, 1, mono, 1.0, 0, 0, 1);
        rend(&U, ms + 30, o);
        CHECK(abs((int)o[4] - (int)f16(0.8)) <= 1, "%s: a repeated LIGHT_DEF leaves the light on (%u)", what, o[4]);
    }
}

static void test_spec_off_colours(void)
{
    static const uint8_t rgb[5] = { ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE, 0, 0 };
    double red[5] = { 1, 0, 0, 0, 0 }, blue[5] = { 0, 0, 1, 0, 0 };
    uint16_t o[5];
    uint32_t ms = 100;

    printf("spec: on -> off: only M falls (the colours stay); off -> on: the new colour from the start\n");
    ultra_init(&U, 22);
    def_light(&U, ms, 0, rgb, 1.0, 0, 0, 1);              /* gamma 1: out = M x A */
    set_light(&U, ms, 0, 0, 1, red, 0, 0);
    ms += 10;
    set_light(&U, ms, 0, 0, 0, blue, 1000, 0);            /* off; the host's colour value happens to be blue */
    rend(&U, ms + 500, o);
    CHECK(abs((int)o[0] - 32768) < 40 && o[2] == 0u, "on -> off: red fades, no blue appears (%u %u)", o[0], o[2]);
    rend(&U, ms + 1000, o);
    CHECK(o[0] == 0u && o[1] == 0u && o[2] == 0u, "on -> off: dark at the end (%u %u %u)", o[0], o[1], o[2]);
    ms += 1100;
    set_light(&U, ms, 0, 0, 1, blue, 1000, 0);            /* off -> on: the host's colour from the start */
    rend(&U, ms + 500, o);
    CHECK(abs((int)o[2] - 32768) < 40 && o[0] == 0u, "off -> on: blue from the start, no red (%u %u)", o[0], o[2]);
    ms += 1100;
    set_light(&U, ms, 0, ULTRA_SET_VIA_OFF | ULTRA_SET_SMOOTH, 0, red, 1000, 0);   /* via off, to off */
    rend(&U, ms + 250, o);
    CHECK(o[0] == 0u && o[2] > 10000u, "via off to off: the old colour fades (%u %u)", o[0], o[2]);
    rend(&U, ms + 750, o);
    CHECK(o[0] == 0u && o[2] == 0u, "via off to off: dark in the second half (%u %u)", o[0], o[2]);
}

static void test_spec_fx_spare(void)
{
    static const uint8_t brp[7] = { 0x00, 0x0B, 0xB8, 0x00, 0x00, 0xFF, 0xFF };   /* breathe 3 s, 0..1 */
    uint8_t s[32], v[32];
    uint32_t i;

    printf("spec: EFFECT / SCENE_EFFECT spare parameter bytes: the engine's state (the stored head) unchanged\n");
    c_setup();
    ultra_slot_scene_effect(s, 0, cseq++, ULTRA_FX_BREATHE, brp, 7);
    memcpy(v, s, 32);
    for (i = 1 + 7; i < ULTRA_PAYLOAD_LEN; i++) v[ULTRA_PAYLOAD + i] = 0xA5u;
    c_reseal(v);
    CHECK(c_pair(s, v, 10) && u_same(), "SCENE_EFFECT: spare bytes ignored, the head byte-identical");
    CHECK(UB.light[0].scene_fx == ULTRA_FX_BREATHE && memcmp(UB.light[0].scene_fxp, brp, 7) == 0,
          "SCENE_EFFECT: its own parameters kept");
    ultra_slot_effect(s, 0, cseq++, ULTRA_FX_BREATHE, 0, brp, 7);
    memcpy(v, s, 32);
    for (i = 5 + 7; i < ULTRA_PAYLOAD_LEN; i++) v[ULTRA_PAYLOAD + i] = 0xA5u;
    c_reseal(v);
    hdr(&UA, 20);
    hdr(&UB, 20);
    CHECK(c_pair(s, v, 20) && u_same(), "EFFECT: spare bytes ignored, the state byte-identical");
}

static void test_spec_misc(void)
{
    static const uint8_t mono[5] = { 0, 0, 0, 0, ULTRA_ROLE_BRIGHTNESS };
    double z[5] = { 0, 0, 0, 0, 0 };
    uint16_t av[5] = { 65535, 65535, 65535, 65535, 65535 }, bv[5] = { 65535, 65535, 65535, 65535, 65535 }, o[5];
    uint8_t p[24], s[32], fseq;
    uint32_t ms = 100, t;
    int dark = 1;

    printf("spec: an effect on a light that is off stays off; STEP bounds; the signal loss timeout's default\n");
    ultra_init(&U, 23);
    CHECK(U.cfg[ULTRA_CFG_LOSS_TIMEOUT] == 30u, "CONFIG 9 default 30 = 3 s (%u)", U.cfg[ULTRA_CFG_LOSS_TIMEOUT]);
    def_light(&U, ms, 1, mono, 1.0, 0, 0, 1);
    set_light(&U, ms, 1, 0, 0, z, 0, 0);                  /* off */
    memset(p, 0, sizeof p);
    ultra_put24(p, 2000); ultra_put16(p + 3, f16(0.2)); ultra_put16(p + 5, f16(1.0));
    fx_send(&U, ms, 1, ULTRA_FX_BREATHE, p, 7);
    for (t = 1; t < 4000; t += 7) {
        rend(&U, ms + t, o);
        dark &= o[4] == 0u;
    }
    CHECK(dark, "breathe 20..100 %% on a light that is off: stays off");
    fseq = useq++;                                        /* strobe steps carry their own M: still off */
    memset(p, 0, sizeof p);
    p[0] = 2;
    ultra_slot_effect(s, 1, fseq, ULTRA_FX_STROBE, 0, p, 1);
    send(&U, ms + 4000, s);
    ultra_slot_step(s, 1, fseq, 0, 2, 100, 0, 65535, av, bv);
    send(&U, ms + 4001, s);
    ultra_slot_step(s, 1, fseq, 1, 2, 100, 0, 32768, av, bv);
    send(&U, ms + 4002, s);
    for (t = 4010; t < 5000; t += 7) {
        rend(&U, ms + t, o);
        dark &= o[4] == 0u;
    }
    CHECK(dark, "strobe steps on a light that is off: stays off");
    {
        uint8_t have = U.light[1].step_have;
        ultra_slot_step(s, 1, fseq, 2, 2, 100, 0, 65535, av, bv);        /* index >= count */
        CHECK(send(&U, ms + 5000, s) == 0u && U.light[1].step_have == have, "STEP index >= count: ignored");
        ultra_slot_step(s, 1, fseq, 0, 9, 100, 0, 65535, av, bv);        /* count > ULTRA_MAX_STEPS */
        CHECK(send(&U, ms + 5001, s) == 0u && U.light[1].step_have == have, "STEP count > 8: ignored");
    }
}

static void test_spec(void)
{
    test_spec_loss_refresh();
    test_spec_off_colours();
    test_spec_fx_spare();
    test_spec_misc();
}
