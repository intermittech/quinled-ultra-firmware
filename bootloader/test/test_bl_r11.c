/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Round 11 regressions. Uses bl_boot itself (also called by bl_main), not a second implementation of boot policy. */
static void r11_factory(void)
{
    board_reset_model();
    make_image(&im_old, 6000, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 61);
    make_image(&im_new, 6000, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 62);
    factory(&im_old);
}

static void test_r11_watchdog(void)
{
    uint32_t i;
    printf("r11 watchdog: early app hang, three resets\n");
    r11_factory();
    board_power_on();
    CHECK(B.in_app && watchdog_starts == 1u, "confirmed fast boot starts IWDG before app code");
    B.app_hung = 1;
    for (i = 1; i <= 3u; i++) {
        host_idle(4000000u);
        CHECK(HO.wd_resets == i, "hung startup reset %u detected without app watchdog setup", (unsigned)i);
        CHECK(B.in_app == (i < 3u), "only third watchdog reset enters recovery");
    }
}

static void test_r11_recovery(void)
{
    uint32_t i;
    printf("r11 recovery: three resets, timed retry and SAME retry\n");
    r11_factory();
    board_power_on();
    for (i = 0; i < 3u; i++) { board_boot(BL_RST_WATCHDOG); }
    CHECK(!B.in_app && HO.wd_resets == 3u, "three resets enter recovery independently of early watchdog test");
    B.app_hung = 1;
    host_idle(59999000u);
    CHECK(!B.in_app, "59.999 s is still recovery");
    host_idle(1000u);
    CHECK(B.in_app && HO.wd_resets == 0u, "60 s retries confirmed app with a fresh watchdog budget");
    host_idle(12000000u);
    CHECK(!B.in_app && HO.wd_resets == 3u, "a persistent hang returns to recovery after another three resets");
    B.app_hung = 0;
    host_session(&im_old, 91, &SESS_DEFAULT);
    CHECK(B.in_app && HO.wd_resets == 0u && HO.result == UPD_RES_SAME, "SAME skip escapes watchdog recovery");

    /* Modular time arithmetic, no timeout for an invalid/unconfirmed image; PING is not a session. */
    bl_app_read(&BA);
    bl_sess_init(&BS, BL_ACT_RECOVERY, &BA, 0xFFFF8000u);
    CHECK(!bl_sess_poll(&BS, 0xFFFF8000u + 59999u), "recovery before wrapped deadline");
    CHECK(bl_sess_poll(&BS, 0xFFFF8000u + 60000u), "recovery deadline across ms wrap");
    BA.confirmed = 0;
    bl_sess_init(&BS, BL_ACT_RECOVERY, &BA, 0);
    CHECK(!bl_sess_poll(&BS, 120000u), "unconfirmed stays in recovery");
    BA.confirmed = 1; BA.desc_ok = 0;
    bl_sess_init(&BS, BL_ACT_RECOVERY, &BA, 0);
    CHECK(!bl_sess_poll(&BS, 120000u), "invalid stays in recovery");
    BA.desc_ok = 1;
    bl_sess_init(&BS, BL_ACT_RECOVERY, &BA, 0);
    {
        uint8_t f[UPD_FRAME_BYTES], d[32] = {0};
        upd_build_frame(f, UPD_PING, 1, 0, 0, 0, d, 0);
        bl_sess_frame(&BS, UPD_FRAME_BITS, f, 59000);
    }
    CHECK(bl_sess_poll(&BS, 60000u), "PING traffic cannot make recovery sticky");
    bl_sess_init(&BS, BL_ACT_RECOVERY, &BA, 0);
    BS.state = BL_S_SKIP;
    BS.t_last = 59999u;
    CHECK(!bl_sess_poll(&BS, 60000u), "active session keeps its own timeout at the recovery deadline");
}

static void test_r11_erase(void)
{
    uint8_t desc_page[256];
    printf("r11 revoke: first-sector partial erase changes body bits but preserves descriptor/status\n");
    r11_factory();
    board_power_on();
    memcpy(desc_page, fl + UPD_APP_DESC - FL_BASE, sizeof desc_page);
    fl_partial_erase = 1;
    if (setjmp(cut_jmp) == 0) { host_session(&im_new, 92, &SESS_DEFAULT); }
    fl_partial_erase = 0;
    CHECK(fl_revokes == 1u && fl_erases == 1u, "one revoke before the first erase");
    CHECK(memcmp(desc_page + 2, fl + UPD_APP_DESC - FL_BASE + 2, 254) == 0,
          "torn erase left descriptor CRC and CONFIRMED status unchanged");
    CHECK(memcmp(im_old.body, fl + UPD_APP_BODY - FL_BASE, im_old.len) != 0, "partial erase really damaged body");
    board_power_on();
    CHECK(!B.in_app && !BA.desc_ok, "damaged confirmed body cannot boot after partial erase");
    host_session(&im_new, 93, &SESS_DEFAULT);
    host_after();
    CHECK(running(&im_new) && !fl_violations, "retry installs after partial erase");
    r11_factory();
    board_power_on();
    fl_revoke_fail = 1;
    host_session(&im_new, 94, &SESS_DEFAULT);
    CHECK(fl_erases == 0u && flash_has(&im_old), "failed revoke stops before any erase");
    board_power_on();
    CHECK(running(&im_old), "old intact body still boots after a refused revoke");
    fl_revoke_fail = 0;
}

static void test_r11_escape(void)
{
    uint32_t i;
    static const uint32_t duration[] = {0, 1, 19999, 20000, 25000};
    printf("r11 DIN escape: threshold, reset kind, timeout, session and fresh image verification\n");
    for (i = 0; i < sizeof duration / sizeof duration[0]; i++) {
        r11_factory();
        din_high_for = duration[i] * 8u;
        board_power_on();
        CHECK(B.in_app == (duration[i] < 20000u), "DIN high %u us: qualification", (unsigned)duration[i]);
        if (!B.in_app) {
            host_idle(2999000);
            CHECK(!B.in_app, "escape listens for full 3 s even with confirmed app");
            host_idle(1000);
            CHECK(B.in_app, "escape timeout boots confirmed app");
        }
    }
    r11_factory();
    board_power_on();
    din_high_for = 200000;
    board_boot(BL_RST_SOFT);
    CHECK(B.in_app && boot_tick == 0u, "no escape sampling on software reset");
    board_boot(BL_RST_WATCHDOG);
    CHECK(B.in_app && boot_tick == 0u, "no escape sampling on watchdog reset");
    r11_factory();
    boot_tick = din_start = 0xFFFF0000u;
    din_high_for = 160000;
    board_power_on();
    CHECK(!B.in_app && BS.mode == BL_ACT_SESSION, "20 ms DIN qualification across timer wrap");
    host_session(&im_new, 95, &SESS_DEFAULT);
    host_after();
    CHECK(running(&im_new), "power-on escape can install update");
    r11_factory();
    fl_fill_erased(UPD_APP_DESC - FL_BASE + 64u, 192u);  /* committed, not yet VERIFIED */
    fl[UPD_APP_BODY - FL_BASE] ^= 1;
    din_high_for = 160000;
    board_power_on();
    host_idle(3000000);
    CHECK(!B.in_app && !BA.desc_ok, "escape timeout does not bypass body VERIFY");
}

static void test_r11_protection(void)
{
    uint32_t i;
    printf("r11 WRPR: before app, once, preserve options, reset/readback, unsafe inputs and cuts\n");
    r11_factory();
    options[4] = 0xAF; options[5] = 0x5A; options[6] = 0xC3; options[7] = 0x69;
    boot_open = 1;
    din_high_for = 160000;                             /* escape survives the options' soft reset */
    board_power_on();
    CHECK(!B.in_app && BS.mode == BL_ACT_SESSION && B.boots == 2u, "provision first, retain DIN escape");
    CHECK(options_writes == 1u && options_reads == 2u && !boot_open && HO.reserved == BL_PROT_OK,
          "one option write then reset and independent read-back");
    CHECK(options[0] == 0xA5 && options[1] == 0x3F && options[2] == 0x12 && options[3] == 0x34 &&
          options[4] == 0xAC && options[5] == 0x5A && options[6] == 0xC3 && options[7] == 0x69,
          "only WRPR0 bits 0-1 change; RDPR USER DATA WRPR1-3 preserved");
    host_idle(3000000);
    CHECK(B.in_app && options_writes == 1u, "protected app finally runs, no further option writes");
    board_power_on();
    CHECK(B.in_app && options_writes == 1u, "subsequent power-on does not rewrite options");
    for (i = 0; i < 6u; i++) {
        r11_factory();
        boot_open = 1; options[4] = 0xFF;
        if (i == 0u) { fl_power = 0; }
        if (i == 1u) { options_ok = 0; }
        if (i == 2u) { options[0] = 0xFF; }
        if (i == 3u) { options_fail = 1; }
        if (i == 4u) { options_corrupt = 1; }
        if (i == 5u) { fl_fill_erased(UPD_APP_DESC - FL_BASE, UPD_PAGE_BYTES); }
        board_power_on();
        CHECK(!B.in_app, "unsafe protection case %u cannot run app", (unsigned)i);
        CHECK(options_writes == (uint32_t)(i >= 3u), "case %u: correct option write admission", (unsigned)i);
        if (i == 4u) { CHECK(HO.reserved == BL_PROT_FAIL, "read-back checks ALL option bytes, not just WPR"); }
        if (i == 5u) { CHECK(!boot_open, "protect even without an installed descriptor"); }
        host_idle(65000000);
        CHECK(!B.in_app, "protection failure cannot time out into an unprotected app");
    }
    r11_factory();
    boot_open = 1; options[4] = 0xFF; options_cut = 1;
    if (setjmp(cut_jmp) == 0) { board_power_on(); }
    options_cut = 0;
    board_power_on();
    CHECK(!B.in_app && options_writes == 1u, "option-erase cut: no unsafe automatic rewrite; jig reprovisions");
}

static void test_r11_all(void)
{
    test_r11_watchdog();
    test_r11_recovery();
    test_r11_erase();
    test_r11_escape();
    test_r11_protection();
}
