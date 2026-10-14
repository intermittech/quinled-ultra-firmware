/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Included after test_ultra.c: signal-loss fade uses the last emitted effect value. */
static void test_ultra_all_off(void)
{
    static const uint8_t mono[5] = { ULTRA_ROLE_BRIGHTNESS, 0, 0, 0, 0 };
    static const uint8_t rgb[5] = { ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE, 0, 0 };
    static const double a[5] = { 1, 1, 1, 1, 1 };
    uint16_t before[5], after[5], prev[5];
    uint8_t p[7], prior_seq = useq;
    uint32_t i, ms;
    printf("Ultra signal-loss fade: continuity from effects, colour, immediate off, undefined lights\n");
    useq = 1; /* Keep this fixture independent of other tests' sequence counter. */
    ultra_init(&U, 1);
    def_light(&U, 100, 0, mono, 1.0, 0, 0, 1);
    set_light(&U, 100, 0, 0, 1, a, 0, 0);
    ultra_put24(p, 4000); ultra_put16(p + 3, 13107); ultra_put16(p + 5, 65535);
    fx_send(&U, 100, 0, ULTRA_FX_BREATHE, p, 7);
    rend(&U, 100, before);
    ultra_all_off(&U, tk(100), 1000);
    rend(&U, 100, after);
    CHECK(before[0] == 13107 && memcmp(before, after, sizeof before) == 0,
          "signal-loss fade must not jump at start (%u -> %u)", before[0], after[0]);
    memcpy(prev, after, sizeof prev);
    for (ms = 101; ms <= 1100; ms++) {
        rend(&U, ms, after);
        CHECK(after[0] <= prev[0], "signal-loss fade must only decrease, ms=%u", ms);
        memcpy(prev, after, sizeof prev);
    }
    CHECK(after[0] == 0 && U.light[0].fx == ULTRA_FX_NONE && !U.light[0].pend,
          "signal-loss fade completes at exactly 1000 ms and cancels the effect");
    for (i = 1; i < ULTRA_LIGHTS; i++) CHECK(!U.light[i].defined, "off must not define light %u", i);

    /* A colour-changing effect must keep its displayed colour during the fade. */
    ultra_init(&U, 2);
    def_light(&U, 2000, 0, rgb, 1.0, 0, 0, 1);
    set_light(&U, 2000, 0, 0, 1, a, 0, 0);
    ultra_put24(p, 4000); ultra_put16(p + 3, 65535);
    fx_send(&U, 2000, 0, ULTRA_FX_COLOR_LOOP, p, 5);
    rend(&U, 2000, before);
    CHECK(before[0] == 65535 && before[1] == 0 && before[2] == 0, "colour loop starts red");
    ultra_all_off(&U, tk(2000), 1000);
    rend(&U, 2000, after);
    CHECK(memcmp(before, after, sizeof before) == 0, "signal-loss fade preserves effect colour at start");
    rend(&U, 2500, after);
    CHECK(abs((int)after[0] - 32768) <= 2 && after[1] == 0 && after[2] == 0,
          "signal-loss fade stays red halfway (%u %u %u)", after[0], after[1], after[2]);
    rend(&U, 3000, after);
    CHECK(after[0] == 0 && after[1] == 0 && after[2] == 0, "colour-effect fade completes dark");

    /* Ordinary state uses the same fade, and zero duration is still immediate off. */
    set_light(&U, 4000, 0, 0, 0.8, a, 0, 0);
    rend(&U, 4000, before);
    ultra_all_off(&U, tk(4000), 1000);
    rend(&U, 4000, after);
    CHECK(memcmp(before, after, sizeof before) == 0, "ordinary signal-loss fade preserves start");
    rend(&U, 4500, after);
    CHECK(abs((int)after[0] - 26214) <= 2, "ordinary fade remains half brightness at midpoint");
    set_light(&U, 6000, 0, 0, 1, a, 0, 0);
    fx_send(&U, 6000, 0, ULTRA_FX_BREATHE, p, 7);
    rend(&U, 6000, before);
    ultra_all_off(&U, tk(6000), 0);
    rend(&U, 6000, after);
    for (i = 0; i < 5; i++) CHECK(after[i] == 0, "zero-duration signal loss is immediately off");
    useq = prior_seq;
}
