/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Review round 6 (r6-claude-esphome), included by test_soak.c.
 * r6candle  A power-on candle with no host for 26 days (a niche without ESPHome, examples/
 *           quinled-dig2analog-ultra-candle-niche.yaml): 0.6.6 freezes the flame once the effect's ms clock is 2^31 ms
 *           (24.86 days) old: light_values takes the effect time as negative and holds it at the effect's start, and
 *           the rebase that keeps pulse / strobe / breathe / colour loop running skips the effects without a period
 *           (candle, flicker, random). Frozen until day 49.7. The same for a candle the host started and left on.
 *           Expected: the flame flickers in a window 24.9 days after power-up as at boot. */
static void sc_r6candle(void)
{
    ultra_t u;
    uint8_t s[32], crc, *img;
    uint16_t z[5] = { 0, 0, 0, 0, 0 }, one[5] = { 65535, 65535, 65535, 65535, 65535 };
    const uint8_t mono[5] = { ULTRA_ROLE_BRIGHTNESS, 0, 0, 0, 0 };
    win_t ref, w;
    printf("soak r6candle: a power-on candle without a host for 26 days (the effect's ms clock past 2^31)\n");
    ultra_init(&u, 1);                                   /* the store: light 0 on CH1, scene 80 %, candle */
    ultra_slot_light_def(s, 0, 1, mono, 1000, 0, z, one);
    (void)ultra_on_slot(&u, s, 0);
    ultra_slot_scene(s, 0, 2, 1, 10, 52428, one, one);
    (void)ultra_on_slot(&u, s, 0);
    ultra_slot_scene_effect(s, 0, 3, ULTRA_FX_CANDLE, P_CANDLE, 4);
    (void)ultra_on_slot(&u, s, 0);
    img = fl + (SCENE_FX_ADDR - FL_BASE);
    crc = crc8_smbus((const uint8_t *)&u.light[0], ULTRA_HEAD_BYTES);
    memcpy(img + ULTRA_FX_HDR, &u.light[0], ULTRA_HEAD_BYTES);
    img[5] = crc;
    img[0] = 0x53; img[1] = 0x46; img[2] = 0x58; img[3] = 0x31;   /* ULTRA_FX_MAGIC, little endian */
    img[4] = 0x01u;
    boot(0, 0, 0);
    GRACE_WAIT();                                        /* (0.6.10: the receiver's grace window) */
    CHECK(fx_mask == 0x01u && fx_ch == 0x01u, "r6candle: the candle loaded (mask %X, channels %X)", fx_mask, fx_ch);
    run_ms(3000);
    win_begin(&ref);
    run_ms(10000);
    win_end();
    run_sparse(2147484u + 3600u - 13u, 0);               /* to 1 h past 2^31 ms of the effect's time */
    win_begin(&w);
    run_ms(10000);
    win_end();
    printf("  10 s windows: from boot %u changes, at 24.9 days %u changes (output %u)\n", ref.chg[0], w.chg[0],
           out_now[0]);
    CHECK(ref.chg[0] > 1000u && w.chg[0] * 2u >= ref.chg[0], "r6candle: the candle still flickers 24.9 days after "
          "power-up (%u changes in 10 s, %u at boot; 0.6.6: frozen until day 49.7)", w.chg[0], ref.chg[0]);
}
