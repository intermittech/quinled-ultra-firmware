/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Defaults while tripped must retain protection and eventually retry. */
static void sc_r4reset(void)
{
    uint32_t t;
    settings_default(&set); store_ext_clear(&sext);
    scene_clear(&scene_want); scene_clear(&scene_stored);
    isense_setup();
    is.tripped = ISENSE_TRIP_FAST; pwm_force_off(1);
    (void)cli_apply(CMD_FACTORY_RESET, 0xA5);
    CHECK(sim_forced && is.tripped != ISENSE_TRIP_NONE, "factory reset orphaned the PWM force-off latch");
    for (t = 0; t < ISENSE_RETRY_S * 1000u + 2u; t++) {
        ms_now++;
        isense_service();
    }
    CHECK(!sim_forced && is.tripped == ISENSE_TRIP_NONE, "outputs never retried after defaults");
}
