/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Power-on effects (SCENE_EFFECT, 0.6.0; core/ultra.c), included by test_main.c after test_ultra.c: the slot, the
 * persisted head in the store image (as hal/hw.c writes it) and its loader, and a power-up without a host: a board that
 * loads the image runs exactly the effect a host would have started (breathe, candle with its randomness, colour loop,
 * sunrise), output for output over seconds. Then the take-over (the effect stops; a light the host didn't define goes
 * dark, one it defines shows its state), the host's own effect replacing it, which effects count, and a damaged image. */

static ultra_t FA, FB, FC;

/* the store image of engine u's lights in mask, as hal/hw.c scene_fx_store writes it */
static void fx_image(const ultra_t *u, uint8_t mask, uint8_t *img)
{
    uint32_t i, m = ULTRA_FX_MAGIC;
    memset(img, 0xFF, 512);
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        if (mask & (1u << i)) {
            memcpy(img + ULTRA_FX_HDR + i * ULTRA_HEAD_BYTES, &u->light[i], ULTRA_HEAD_BYTES);
            img[5u + i] = crc8_smbus(img + ULTRA_FX_HDR + i * ULTRA_HEAD_BYTES, ULTRA_HEAD_BYTES);
        }
    }
    for (i = 0; i < 4u; i++) {
        img[i] = (uint8_t)(m >> (8u * i));
    }
    img[4] = mask;
}

/* host engine u: light 0 RGB at gamma 2.8, SCENE 60 % warm orange (fade 1 s) and SCENE_EFFECT fx */
static void fx_host(ultra_t *u, uint8_t fx, const uint8_t *p, uint32_t n)
{
    static const uint8_t roles[5] = { ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE, 0, 0 };
    static const uint16_t a[5] = { 65535, 26214, 6554, 0, 0 }, b[5] = { 65535, 65535, 65535, 65535, 65535 };
    uint8_t s[32];
    ultra_init(u, 5);
    def_light(u, 10, 0, roles, 2.8, 0, 0, 1);
    ultra_slot_scene(s, 0, useq++, 1, 10, 39321, a, b);
    send(u, 10, s);
    ultra_slot_scene_effect(s, 0, useq++, fx, p, n);
    CHECK(send(u, 10, s) == ULTRA_EV_SCENE, "SCENE_EFFECT %u: applied", fx);
}

/* the reference: a host that sent the same light with SET (the SCENE values) and EFFECT fx at local time 0 */
static void fx_reference(ultra_t *u, uint8_t fx, const uint8_t *p, uint32_t n)
{
    static const uint16_t a[5] = { 65535, 26214, 6554, 0, 0 }, b[5] = { 65535, 65535, 65535, 65535, 65535 };
    uint8_t s[32];
    ultra_init(u, 77);
    memcpy(&u->light[0], &FA.light[0], ULTRA_HEAD_BYTES);
    u->light[0].defined = 1;
    ultra_slot_set(s, 0, 9, 0, 39321, a, b, 0, 0);
    (void)ultra_on_slot(u, s, 0);
    ultra_slot_effect(s, 0, 9, fx, 0, p, n);
    (void)ultra_on_slot(u, s, 0);
}

static void test_scenefx(void)
{
    static const struct { uint8_t fx, n, p[8]; const char *name; } FX[] = {
        { ULTRA_FX_BREATHE, 7, { 0x00, 0x07, 0xD0, 0x33, 0x33, 0xFF, 0xFF }, "breathe 2 s 20..100 %" },
        { ULTRA_FX_CANDLE, 4, { 0x80, 0x00, 0x00, 0x64 }, "candle" },
        { ULTRA_FX_COLOR_LOOP, 5, { 0x00, 0x27, 0x10, 0xFF, 0xFF }, "colour loop 10 s" },
        { ULTRA_FX_SUNRISE, 3, { 0x00, 0x0B, 0xB8 }, "sunrise 3 s" },
    };
    uint8_t img[512], crc[ULTRA_LIGHTS], s[32];
    uint16_t ob[5], oc[5];
    uint32_t k, ms, diff, hb = ULTRA_HEAD_BYTES;

    printf("power-on effects: SCENE_EFFECT, the stored head, power-up without a host, take-over\n");
    CHECK(hb % 2u == 0u && ULTRA_FX_HDR + ULTRA_LIGHTS * hb <= 512u, "the store fits two pages (%u-byte heads)", hb);
    for (k = 0; k < sizeof FX / sizeof FX[0]; k++) {
        fx_host(&FA, FX[k].fx, FX[k].p, FX[k].n);
        CHECK(FA.light[0].scene_fx_have && FA.light[0].scene_fx == FX[k].fx && ultra_scene_fx_ok(&FA.light[0]) &&
              memcmp(FA.light[0].scene_fxp, FX[k].p, FX[k].n) == 0, "%s: stored in the head", FX[k].name);
        fx_image(&FA, 1, img);
        ultra_init(&FB, 77);                              /* power-up: another engine, nothing from a host */
        CHECK(ultra_fx_load(&FB, img, crc) == 1u && crc[0] == img[5] && !FB.light[0].defined, "%s: loaded",
              FX[k].name);
        ultra_scene_fx_start(&FB, 1, 0);
        fx_reference(&FC, FX[k].fx, FX[k].p, FX[k].n);
        diff = 0;
        for (ms = 0; ms < 6000u; ms++) {
            ultra_render(&FB, ms * 20u, ob);
            ultra_render(&FC, ms * 20u, oc);
            diff += memcmp(ob, oc, sizeof ob) != 0;
        }
        CHECK(diff == 0 && FB.light[0].fx == (FX[k].fx == ULTRA_FX_SUNRISE ? ULTRA_FX_NONE : FX[k].fx),
              "%s: the board runs it as a host would, %u of 6000 ms differ", FX[k].name, diff);
    }
    /* the effect moves the outputs (breathe: down to 20 % and back) */
    fx_host(&FA, FX[0].fx, FX[0].p, FX[0].n);
    fx_image(&FA, 1, img);
    ultra_init(&FB, 77);
    ultra_scene_fx_start(&FB, ultra_fx_load(&FB, img, crc), 0);
    {
        uint16_t lo = 65535, hi = 0;
        for (ms = 0; ms < 4000u; ms++) {
            ultra_render(&FB, ms * 20u, ob);
            lo = ob[0] < lo ? ob[0] : lo;
            hi = ob[0] > hi ? ob[0] : hi;
        }
        CHECK(lo < 2000u && hi > 15000u, "breathe at power-up: R between %u and %u", lo, hi);
    }

    /* the take-over: stop; a light the host didn't define goes dark, one it defines shows its SET */
    ultra_scene_fx_stop(&FB);
    ultra_render(&FB, 90000u, ob);
    CHECK(FB.light[0].fx == ULTRA_FX_NONE && !FB.light[0].scene_run && ob[0] == 0 && ob[1] == 0,
          "stopped, not defined by the host: dark");
    ultra_init(&FB, 77);
    ultra_scene_fx_start(&FB, ultra_fx_load(&FB, img, crc), 0);
    {
        static const uint8_t roles[5] = { ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE, 0, 0 };
        static const double a[5] = { 0, 1, 0, 0, 0 };
        def_light(&FB, 500, 0, roles, 2.8, 0, 0, 1);
        set_light(&FB, 500, 0, 0, 1.0, a, 0, 0);
        ultra_scene_fx_stop(&FB);
        rend(&FB, 600, ob);
        CHECK(ob[0] == 0 && ob[1] == 65535 && FB.light[0].fx == ULTRA_FX_NONE, "the host's SET: green (%u %u)", ob[0],
              ob[1]);
    }
    /* the host's own effect replaces the power-on one, also through the stop */
    ultra_init(&FB, 77);
    ultra_scene_fx_start(&FB, ultra_fx_load(&FB, img, crc), 0);
    {
        static const uint8_t roles[5] = { ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE, 0, 0 };
        static const uint8_t p[5] = { 0x00, 0x27, 0x10, 0xFF, 0xFF };
        def_light(&FB, 500, 0, roles, 2.8, 0, 0, 1);
        fx_send(&FB, 500, 0, ULTRA_FX_COLOR_LOOP, p, 5);
        CHECK(!FB.light[0].scene_run, "the host's EFFECT ends the power-on one");
        rend(&FB, 501, ob);
        ultra_scene_fx_stop(&FB);
        rend(&FB, 502, ob);
        CHECK(FB.light[0].fx == ULTRA_FX_COLOR_LOOP, "the host's effect runs on (%u)", FB.light[0].fx);
    }

    /* which effects count; the slot's own rules */
    fx_host(&FA, ULTRA_FX_STROBE, (const uint8_t *)"\x02", 1);
    CHECK(!ultra_scene_fx_ok(&FA.light[0]), "strobe: not a power-on effect (its steps aren't stored)");
    fx_host(&FA, ULTRA_FX_NONE, 0, 0);
    CHECK(!ultra_scene_fx_ok(&FA.light[0]) && FA.light[0].scene_fx_have, "none: no power-on effect");
    fx_host(&FA, ULTRA_FX_CANDLE, FX[1].p, 4);
    FA.light[0].scene_mode = 0;
    CHECK(!ultra_scene_fx_ok(&FA.light[0]), "SCENE mode 0: no power-on effect");
    FA.light[0].scene_mode = 1;
    FA.light[0].defined = 0;
    CHECK(!ultra_scene_fx_ok(&FA.light[0]), "an undefined light: none");
    FA.light[0].defined = 1;
    ultra_slot_scene_effect(s, 0, 200, ULTRA_FX_CANDLE, FX[1].p, 4);
    s[3] = 0xC0;                                          /* a custom effect id (the builder never sends one) */
    ultra_slot_seal(s);
    CHECK(send(&FA, 20, s) == ULTRA_EV_SCENE && FA.light[0].scene_fx == ULTRA_FX_NONE, "an unknown effect id: none");
    CHECK(send(&FA, 21, s) == 0, "the same seq again: ignored");

    /* a damaged image */
    fx_host(&FA, FX[0].fx, FX[0].p, FX[0].n);
    def_light(&FA, 30, 2, (const uint8_t *)"\x05\x05\x05\x05\x05", 2.2, 0, 0, 1);
    fx_image(&FA, 5, img);
    ultra_init(&FB, 1);
    CHECK(ultra_fx_load(&FB, img, crc) == 5u, "two lights loaded");
    img[ULTRA_FX_HDR + 2u * ULTRA_HEAD_BYTES + 7u] ^= 0x10u;
    CHECK(ultra_fx_load(&FB, img, crc) == 1u, "a damaged head: that light dropped");
    img[1] ^= 1u;
    CHECK(ultra_fx_load(&FB, img, crc) == 0u, "a damaged header: nothing");
    memset(img, 0xFF, sizeof img);
    CHECK(ultra_fx_load(&FB, img, crc) == 0u, "an erased store: nothing");
}
