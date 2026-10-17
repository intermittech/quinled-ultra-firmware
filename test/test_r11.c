/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* (round 11) scenarios. Included by test_soak.c; soak_tests.exe <name>.
 *
 *   r11confirm <ws | ultra | wd | none>  (B4) a trial image (behind a bootloader, its descriptor valid, not CONFIRMED)
 *                  on a line that never sends a PING: ws, a WS2805 controller (WLED); ultra, an Ultra host without the
 *                  update feature (the sender library). Not confirmed before a minute up; confirmed once after it, as
 *                  at a PING. wd: the same frames after a watchdog reset: never (the image may be what hangs); none:
 *                  no frames at all: never. (The soak builds are standalone: the CONFIRM action is the decision,
 *                  test_update.c the record's write and its retry.)
 *   r11hwid        (B2) the outputs dark behind another hardware's bootloader (below) */

static void sc_r11confirm(const char *how)
{
    static const uint8_t col[5] = { 0x40, 0x80, 0xC0, 0x20, 0x10 };
    uint32_t s;
    int ws = !strcmp(how, "ws") || !strcmp(how, "wd"), ultra = !strcmp(how, "ultra");
    int want = (ws || ultra) && strcmp(how, "wd") != 0;
    printf("soak r11confirm %s: a trial image without PINGs (%s)\n", how,
           !strcmp(how, "wd") ? "after a watchdog reset" : ws ? "WS2805" : ultra ? "Ultra, no update feature" :
           "no frames");
    if (!strcmp(how, "wd")) {
        soak_reset_flags = 0x20000000u;                  /* IWDGRSTF */
    }
    boot(0, 0, 0);
    upd.info.bl_version = 2;                             /* as if behind a bootloader: this image's descriptor valid, */
    upd.info.hw_id = BOARD_HW_ID;                        /* the status without CONFIRMED (a trial boot) */
    upd.info.hw_rev = 3;
    upd.info.desc_ok = 1;
    upd.info.confirmed = 0;
    upd.info.st_next = 1;
    upd.info.version = 0x00000700u;
    upd.confirm_done = 0;
    if (ultra) {
        host_on = 1;
        cmd_def(0, 0, 5, 1000);
        cmd_set(0, 32768, 0, 0);
    }
    for (s = 0; s < 120u; s++) {                         /* two minutes, a second at a time */
        if (ws) {
            r7_ws(col, 50);
        } else {
            run_ms(1000);
        }
        if (up_s < UPD_APP_AUTO_S - 1u) {
            CHECK(!upd.confirm_done, "r11confirm %s: not confirmed at %u s", how, up_s);
        }
    }
    printf("  after %u s: frames decoded %u, auto %u, confirmed %u\n", up_s, stat.decoded, upd.auto_ok,
           upd.confirm_done);
    CHECK(upd.confirm_done == (uint8_t)want && upd.auto_ok == (uint8_t)want && upd.ping_seen == 0u,
          "r11confirm %s: %s (auto %u, confirm %u, frames %u)", how, want ? "confirmed without a PING" : "never confirmed",
          upd.auto_ok, upd.confirm_done, stat.decoded);
    if (want) {
        CHECK(stat.decoded >= UPD_APP_AUTO_FRAMES && (ultra ? ultra_on : proto_active(&frx.ps) == PROTO_WS2805),
              "r11confirm %s: the frames were taken (%u, ultra %u)", how, stat.decoded, ultra_on);
    }
}

/* r11hwid       (B2) behind a bootloader of another hardware id (an image for the other board, flashed over SWD):
 *                  the outputs stay dark whatever the host sends (a light at 50 %, an identify pattern); behind its own
 *                  bootloader, and without one (standalone), the light as sent */
static void sc_r11hwid(void)
{
    uint32_t t, top = 0;
    printf("soak r11hwid: the bootloader's hardware id against this build's (%u)\n", (uint32_t)BOARD_HW_ID);
    boot(0, 0, 0);
    upd.info.bl_version = 2;
    upd.info.hw_id = (uint8_t)(BOARD_HW_ID ^ 3u);        /* the other board's (1 <-> 2) */
    upd.info.hw_rev = 3;
    host_on = 1;
    cmd_def(0, 0, 5, 1000);
    cmd_set(0, 32768, 0, 0);
    cmd_identify(ULTRA_IDENT_ON, 0);
    for (t = 0; t < 4000u; t++) {
        run_ms(1);
        top |= out_now[0] | out_now[1] | out_now[2] | out_now[3] | out_now[4];
    }
    CHECK(ultra_on && top == 0u, "r11hwid: behind hardware %u's bootloader: dark (the most %u)", upd.info.hw_id, top);
    upd.info.hw_id = BOARD_HW_ID;
    cmd_identify(ULTRA_IDENT_STOP, 0);
    cmd_set(0, 32768, 0, 0);
    run_ms(2000);
    outputs_dirty = 1;
    run_ms(10);
    CHECK(out_now[0] == 32768u, "r11hwid: behind its own: the light (%u)", out_now[0]);
    upd.info.bl_version = 0;                             /* standalone: no bootloader to compare */
    upd.info.hw_id = 0;
    outputs_dirty = 1;
    run_ms(10);
    CHECK(out_now[0] == 32768u, "r11hwid: standalone: the light (%u)", out_now[0]);
}

static int r11_scenario(const char *s, int argc, char **argv)
{
    if (!strcmp(s, "r11confirm")) {
        sc_r11confirm(argc > 2 ? argv[2] : "ws");
        return 1;
    }
    if (!strcmp(s, "r11hwid")) {
        sc_r11hwid();
        return 1;
    }
    return 0;
}
