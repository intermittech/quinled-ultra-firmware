/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Included after test_ultra.c helpers; exercise mapping, cache and persisted head. */
static void test_m1_light_base(void)
{
    uint8_t s[32], roles[5] = { ULTRA_ROLE_BRIGHTNESS, 0, 0, 0, 0 };
    uint16_t lo[5] = {0}, hi[5] = {65535,65535,65535,65535,65535}, out[5];
    uint8_t img[512], crcs[ULTRA_LIGHTS] = {0};
    scene_t saved, scene;
    uint32_t i, n;
    ultra_init(&U, 1);
    ultra_slot_light_def(s, 0, 1, roles, 1000, 0, lo, hi, 4);
    CHECK(send(&U, 1, s) & ULTRA_EV_DEF, "base channel 4 accepted");
    ultra_slot_set(s, 0, 2, 0, 32768, hi, hi, 0, 0);
    send(&U, 2, s);
    for (n=0; n<2; ++n) {
        rend(&U, 3+n, out);
        for (i=0; i<5; ++i) CHECK(out[i] == (i==4 ? 32768 : 0), "mapped/cached output %u", i);
    }
    roles[1] = ULTRA_ROLE_RED;
    ultra_slot_light_def(s, 0, 3, roles, 1000, 0, lo, hi, 4);
    CHECK(send(&U, 5, s) == 0 && U.light[0].base_channel == 4, "overflow rejects entire definition");
    roles[1] = 0;
    ultra_slot_light_def(s, 1, 3, roles, 1000, 0, lo, hi, 4);
    send(&U, 6, s);
    ultra_slot_set(s, 1, 4, 0, 12345, hi, hi, 0, 0);
    send(&U, 7, s); rend(&U, 8, out);
    CHECK(out[4] == 12345, "highest defined light owns overlap");
    ultra_slot_scene(s, 0, 5, 0, 0, 20000, hi, hi); send(&U, 9, s);
    scene_clear(&saved); ultra_scene(&U, &saved, &scene);
    CHECK(scene.mask == 16 && scene.v[4] == 20000, "scene uses physical channel mask");
    memset(roles,0,sizeof roles);
    ultra_slot_light_def(s, 0, 6, roles, 1000, 0, lo, hi, 239);
    CHECK(send(&U, 10, s) & ULTRA_EV_DEF, "NONE roles need not fit channel count");
    CHECK(ultra_channel_mask(&U.light[0]) == 0, "empty mapping has no physical ownership");
    CHECK(mcu_pwm_period_10ns(0)==0, "zero selects preset");
    CHECK(mcu_pwm_period_10ns(1000)==1440 && mcu_pwm_period_10ns(4000)==5760, "physical PWM units");
    CHECK(mcu_pwm_period_10ns(1001)==1442 && mcu_pwm_period_10ns(65535)==65535, "PWM rounds up and clamps");

}
