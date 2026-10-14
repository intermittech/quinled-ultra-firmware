/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Review round 6 (r6-claude-esphome), included by test_soak.c.
 * r6gamma0  LIGHT_DEF with gamma 0 (gamma x 1000 = 0). ESPHome's gamma_correct: 0 is linear (its gamma table is
 *           i / 255; light/__init__.py generate_gamma_table), and the component sent it as 0: 0.6.6 computes x^0 = 1,
 *           every level that is on shows at 100 %. Expected: linear, as gamma 1. */
static void sc_r6gamma0(void)
{
    uint8_t s[32];
    uint16_t a[5] = { 65535, 65535, 65535, 65535, 65535 }, b[5] = { 65535, 65535, 65535, 65535, 65535 };
    const uint8_t roles[5] = { ULTRA_ROLE_BRIGHTNESS, 0, 0, 0, 0 };
    const uint16_t z5[5] = { 0, 0, 0, 0, 0 };
    printf("soak r6gamma0: LIGHT_DEF gamma 0 (ESPHome gamma_correct: 0, linear)\n");
    boot(0, 0, 0);
    host_on = 1;
    ultra_slot_light_def(s, 0, 1, roles, 0, 0, z5, a);   /* CH1, gamma 0 */
    host_send(s);
    ultra_slot_set(s, 0, 2, 0, 16384, a, b, 0, 0);       /* 25 % */
    host_send(s);
    run_ms(2000);
    CHECK(out_now[0] >= 16383u && out_now[0] <= 16385u, "r6gamma0: 25 %% shows as 25 %% (%u; 0.6.6: 65535, x^0)",
          out_now[0]);
    CHECK(events_seen == 0u, "r6gamma0: no false events (%X)", events_seen);
}
