/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Current wire-3 LIGHT_DEF: gamma zero selects linear output.
 * Exercise the real acquisition, command dispatch and PWM path. */
static void sc_r6gamma0(void)
{
    uint8_t s[32];
    uint16_t a[5] = { 65535, 65535, 65535, 65535, 65535 }, b[5] = { 65535, 65535, 65535, 65535, 65535 };
    const uint8_t roles[5] = { ULTRA_ROLE_BRIGHTNESS, 0, 0, 0, 0 };
    const uint16_t z5[5] = { 0, 0, 0, 0, 0 };
    printf("soak r6gamma0: LIGHT_DEF gamma 0 (ESPHome gamma_correct: 0, linear)\n");
    boot(0, 0, 0);
    host_on = 1;
    ultra_slot_light_def(s, 0, 1, roles, 0, 0, z5, a, 0);   /* CH1, gamma 0 */
    host_send(s);
    ultra_slot_set(s, 0, 2, 0, 16384, a, b, 0, 0);       /* 25 % */
    host_send(s);
    run_ms(2000);
    CHECK(out_now[0] >= 16383u && out_now[0] <= 16385u, "r6gamma0: 25 %% shows as 25 %% (%u)",
          out_now[0]);
    ultra_slot_light_def(s, 0, 3, roles, 1000, 0, z5, a, 0);
    host_send(s);
    run_ms(200);
    CHECK(out_now[0] >= 16383u && out_now[0] <= 16385u,
          "r6gamma0: gamma 1000 has the same linear output (%u)", out_now[0]);
    CHECK(events_seen == 0u, "r6gamma0: no false events (%X)", events_seen);
}
