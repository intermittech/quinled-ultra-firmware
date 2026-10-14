/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Review round 4 (bootloader / update path): cases the earlier tests did not cover. Included by test_bl.c before
 * its main(); test_sender.cpp includes test_bl.c too, so this file is C and C++.
 *
 * - reset classification (bl_reset_kind: an internal reset may raise PINRSTF as well) and what the model does when
 *   every software reset carries that flag;
 * - a foreign ANNOUNCE (other hardware, another session id) during an install must not abandon it;
 * - hostile / odd frames: DATA beyond the image, ANNOUNCE half 2, END with another image's CRC or length, ABORT of
 *   another session, mixed halves of two images, DATA before any session;
 * - the host restarts the same image under a new session id mid-install: pages kept, one erase cycle;
 * - a session id reused for another image: the mixed body never runs (VERIFY), the retry installs;
 * - VDD low during VERIFY: nothing written, decided again at the next boot;
 * - a torn status record is skipped by both sides;
 * - the trial window: WS / bad / PING frames do not end it, a normal frame does, and the records;
 * - the calibration page (0x0800F600): intact after an image of the linker's full size, and the erase plan's edge;
 * - a reset (RAM kept, as the watchdog's) instead of a power cut at every flash operation. */

#define R4_IMG_LINK_MAX   (0x0800F600u - UPD_APP_BODY)        /* 54,528: Link_app.ld's slot (qdu.APP_BODY_MAX) */
#define R4_FL_CAL         (0x0800F600u - FL_BASE)

static image_t r4_a, r4_b, r4_hw;

/* ------------------------------------------------------------------------------------------- reset classification */
#ifdef BL_RESET_KIND_API
static void test_r4_reset_kind(void)
{
    uint32_t i;
    printf("reset classification (RCC_RSTSCKR; an internal reset may raise PINRSTF too, the QFN20 has no NRST pin)\n");
    CHECK(bl_reset_kind(BL_RCC_SFTRSTF | BL_RCC_PINRSTF) == BL_RST_SOFT, "software reset + PINRSTF: soft");
    CHECK(bl_reset_kind(BL_RCC_SFTRSTF) == BL_RST_SOFT, "software reset alone: soft");
    CHECK(bl_reset_kind(BL_RCC_IWDGRSTF | BL_RCC_PINRSTF) == BL_RST_WATCHDOG, "IWDG + PINRSTF: watchdog");
    CHECK(bl_reset_kind(BL_RCC_IWDGRSTF | BL_RCC_SFTRSTF | BL_RCC_PINRSTF) == BL_RST_WATCHDOG, "IWDG + SFT + PIN: watchdog");
    CHECK(bl_reset_kind(BL_RCC_WWDGRSTF) == BL_RST_WATCHDOG, "WWDG: watchdog");
    CHECK(bl_reset_kind(BL_RCC_PORRSTF | BL_RCC_PINRSTF) == BL_RST_POWER, "POR + PINRSTF: power");
    CHECK(bl_reset_kind(BL_RCC_PORRSTF) == BL_RST_POWER, "POR alone: power");
    CHECK(bl_reset_kind(BL_RCC_LPWRRSTF) == BL_RST_POWER, "low-power reset: power (RAM not trusted)");
    CHECK(bl_reset_kind(BL_RCC_PINRSTF) == BL_RST_POWER, "PINRSTF alone: power (nothing carries over)");
    CHECK(bl_reset_kind(0) == BL_RST_SOFT, "no flag at all: soft (the flags were already taken)");

    /* the whole update flow on a chip whose software resets also raise PINRSTF */
    board_reset_model();
    soft_kind = bl_reset_kind(BL_RCC_SFTRSTF | BL_RCC_PINRSTF);
    make_image(&r4_a, 8000, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 401);
    make_image(&r4_b, 8000, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 403);
    factory(&r4_a);
    for (i = 0; i < sizeof(HO) / 4u; i++) {
        ((uint32_t *)&HO)[i] = rnd();
    }
    B.busy_until_us = 0;
    board_boot(bl_reset_kind(BL_RCC_PORRSTF | BL_RCC_PINRSTF));
    CHECK(B.in_app, "power-on (POR + PIN): the confirmed app");
    HO.request = UPD_REQ_ENTER;
    bl_ho_seal(&HO);
    board_boot(soft_kind);
    CHECK(!B.in_app && B.last_act == BL_ACT_SESSION, "the app's request survives its software reset (act %d)",
          (int)B.last_act);
    host_session(&r4_b, 0x40, &SESS_DEFAULT);
    host_after();
    CHECK(running(&r4_b), "installed, verified, trial window, then the app runs (no trial loop)");
    {
        bl_app_t a;
        bl_app_read(&a);
        CHECK(a.confirmed && a.trials == 1, "confirmed after the PING; one trial record (%u)", (unsigned)a.trials);
    }
    CHECK(fl_violations == 0 && keep_areas_intact(), "flash rules");
    soft_kind = BL_RST_SOFT;
}
#endif

/* -------------------------------------------------------------------------------- foreign ANNOUNCE mid-install */
static void test_r4_foreign_announce(void)
{
    sess_opts_t o = SESS_DEFAULT;
    printf("an ANNOUNCE for other hardware (another session id) during an install\n");
    board_reset_model();
    make_image(&r4_a, 8000, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 411);
    make_image(&r4_b, 8000, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 413);
    make_image(&r4_hw, 8000, V(0, 6, 0, 0xFF), 2, REV_OK, 1, 415);
    factory(&r4_a);
    board_power_on();
    o.passes = 0;
    o.end_ms = 0;
    host_session(&r4_b, 0x61, &o);                        /* lead-in only: accepted, erased, receiving */
    CHECK(!B.in_app && BS.state == BL_S_RECEIVE && BS.session == 0x61, "receiving session 0x61");
    host_upd(UPD_ANNOUNCE, 0x62, 0, 0, 0, r4_hw.desc, 0);
    host_upd(UPD_ANNOUNCE, 0x62, 1, 0, 0, r4_hw.desc + 32, 0);
    CHECK(BS.state == BL_S_RECEIVE && BS.session == 0x61, "still receiving 0x61 (state %d, session 0x%02X)",
          (int)BS.state, BS.session);
    host_upd(UPD_END, 0x62, 0, 0, 0, 0, 0);
    CHECK(BS.state == BL_S_RECEIVE && BS.mode != BL_ACT_RECOVERY, "the foreign END changes nothing");
    host_session(&r4_b, 0x61, &SESS_DEFAULT);             /* our host goes on (its lead-in repeats are ignored) */
    host_after();
    CHECK(running(&r4_b), "the install finished");
    CHECK(fl_violations == 0 && keep_areas_intact(), "flash rules");
}

/* ------------------------------------------------------------------------------------------- hostile / odd frames */
static void test_r4_hostile_frames(void)
{
    static uint8_t snap[FL_SIZE];
    sess_opts_t o = SESS_DEFAULT;
    uint8_t d[32], endd[32];
    uint32_t npages, written_before, half_before;
    printf("hostile / odd frames: DATA beyond the image, ANNOUNCE half 2, wrong END, foreign ABORT, mixed halves\n");
    board_reset_model();
    make_image(&r4_a, 8000, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 421);
    make_image(&r4_b, 8000, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 423);
    make_image(&r4_hw, 8000, V(0, 6, 0, 0xFF), 1, REV_OK, 1, 425);   /* another image for this board */
    factory(&r4_a);
    board_power_on();
    /* DATA and END without a session: the app forwards (the model's app ignores them); a board in its trial window
     * must not write anything either */
    fl_fill_erased((UPD_APP_DESC - FL_BASE) + 66u, 2u);   /* un-confirm the factory app: the next boot is a trial */
    board_power_on();
    CHECK(!B.in_app && BS.mode == BL_ACT_TRIAL, "trial window");
    memcpy(snap, fl, FL_SIZE);
    memset(d, 0xAB, sizeof(d));
    host_upd(UPD_DATA, 0x11, 0, 0, 0, d, 0);
    host_upd(UPD_DATA, 0x11, 0xFFFF, 0, 0, d, 0);
    memset(endd, 0, sizeof(endd));
    host_upd(UPD_END, 0x11, 0, 0, 0, endd, 0);
    host_upd(UPD_ABORT, 0x11, 0, 0, 0, 0, 0);
    CHECK(memcmp(snap, fl, FL_SIZE) == 0 && BS.state == BL_S_LISTEN, "DATA / END / ABORT without a session: nothing");
    host_idle(200000u);
    CHECK(B.in_app, "the window ends, the app runs");

    /* in a session */
    o.passes = 0;
    o.end_ms = 0;
    host_session(&r4_b, 0x63, &o);
    CHECK(!B.in_app && BS.state == BL_S_RECEIVE, "receiving 0x63");
    npages = (r4_b.len + 255u) / 256u;
    written_before = BS.nwritten;
    memcpy(snap, fl, FL_SIZE);
    host_upd(UPD_DATA, 0x63, 0xFFFF, 0, 0, d, 0);         /* chunk 65535: page 8191 */
    host_upd(UPD_DATA, 0x63, npages * 8u, 0, 0, d, 2700u);   /* the first chunk past the image */
    host_upd(UPD_DATA, 0x63, (npages - 1u) * 8u + 7u, 0, 0, d, 2700u);   /* chunk 7 of a short last page? */
    CHECK(BS.nwritten == written_before && memcmp(snap, fl, FL_SIZE) == 0, "DATA beyond the image: nothing written");
    host_upd(UPD_DATA, 0x64, 0, 0, 0, r4_b.body, 0);      /* another session's chunk */
    CHECK(BS.nwritten == written_before && memcmp(snap, fl, FL_SIZE) == 0, "DATA of another session: ignored");
    half_before = BS.have_half;
    host_upd(UPD_ANNOUNCE, 0x65, 2, 0, 0, r4_b.desc, 0);  /* half 2 does not exist */
    CHECK(BS.have_half == half_before && BS.state == BL_S_RECEIVE && BS.session == 0x63, "ANNOUNCE half 2: ignored");
    host_upd(UPD_ANNOUNCE, 0x66, 0, 0, 0, r4_b.desc, 0);  /* mixed halves of two images under one id */
    host_upd(UPD_ANNOUNCE, 0x66, 1, 0, 0, r4_hw.desc + 32, 0);
    CHECK(BS.state == BL_S_RECEIVE && BS.session == 0x63 && BS.have_half == 0,
          "halves of two images: not a descriptor, wait for the next pair (state %d, session 0x%02X)", (int)BS.state,
          BS.session);
    host_upd(UPD_ABORT, 0x67, 0, 0, 0, 0, 0);
    CHECK(BS.state == BL_S_RECEIVE && BS.mode != BL_ACT_RECOVERY, "ABORT of another session: ignored");
    /* the whole body, then END frames of another image (CRC / length), then the right one */
    {
        uint32_t k, c;
        for (k = 0; k < npages; k++) {
            for (c = 0; c < 8u; c++) {
                uint32_t idx = k * 8u + c, off = idx * 32u;
                if (off >= r4_b.len) {
                    break;
                }
                memset(d, 0xFF, sizeof(d));
                memcpy(d, r4_b.body + off, r4_b.len - off < 32u ? r4_b.len - off : 32u);
                host_upd(UPD_DATA, 0x63, idx, 0, 0, d, (c == 7u || off + 32u >= r4_b.len) ? 2700u : 0u);
            }
        }
    }
    memset(endd, 0, sizeof(endd));
    memcpy(endd, r4_hw.desc + UPD_D_BODY_CRC, 4);         /* another image's CRC, our length */
    memcpy(endd + 4, r4_b.desc + UPD_D_BODY_LEN, 4);
    host_upd(UPD_END, 0x63, 0, 0, 0, endd, 0);
    CHECK(BS.state == BL_S_RECEIVE, "END with another body CRC: ignored");
    memcpy(endd, r4_b.desc + UPD_D_BODY_CRC, 4);
    upd_put32(endd + 4, r4_b.len + 256u);
    host_upd(UPD_END, 0x63, 0, 0, 0, endd, 0);
    CHECK(BS.state == BL_S_RECEIVE, "END with another length: ignored");
    memcpy(endd + 4, r4_b.desc + UPD_D_BODY_LEN, 4);
    host_upd(UPD_END, 0x63, 0, 0, 0, endd, 0);
    host_after();
    CHECK(running(&r4_b), "the right END commits; installed");
    CHECK(fl_violations == 0 && keep_areas_intact(), "flash rules");
}

/* ---------------------------------------------------------------------- the host restarts the same image mid-way */
static void test_r4_restart_same_image(void)
{
    sess_opts_t o = SESS_DEFAULT;
    uint32_t written_before, ops_before;
    printf("the host restarts the same image under a new session id before END: pages kept, one erase\n");
    board_reset_model();
    make_image(&r4_a, 16000, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 431);
    make_image(&r4_b, 16000, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 433);
    factory(&r4_a);
    board_power_on();
    o.passes = 1;
    o.end_ms = 0;
    loss_ppm = 50000;                                     /* 5 %: some pages stay incomplete */
    host_session(&r4_b, 0x71, &o);
    loss_ppm = 0;
    CHECK(!B.in_app && BS.state == BL_S_RECEIVE && BS.nwritten > 10u && BS.nwritten < BS.npages,
          "receiving 0x71, %u / %u pages so far", (unsigned)BS.nwritten, (unsigned)BS.npages);
    written_before = BS.nwritten;
    ops_before = fl_ops;
    host_upd(UPD_ANNOUNCE, 0x72, 0, 0, 0, r4_b.desc, 0);
    host_upd(UPD_ANNOUNCE, 0x72, 1, 0, 0, r4_b.desc + 32, 0);
    CHECK(BS.state == BL_S_RECEIVE && BS.session == 0x72 && BS.nwritten == written_before && fl_ops == ops_before,
          "same image, new id 0x72: the session moves over, nothing erased, pages kept");
    host_session(&r4_b, 0x72, &SESS_DEFAULT);
    host_after();
    CHECK(running(&r4_b), "installed by the second session");
    CHECK(BS.pages_programmed == BS.npages, "every page programmed exactly once (%u of %u)",
          (unsigned)BS.pages_programmed, (unsigned)BS.npages);
    CHECK(fl_violations == 0 && keep_areas_intact(), "flash rules");
}

/* ------------------------------------------------------------------------------ one session id, two images */
static void test_r4_session_id_reuse(void)
{
    sess_opts_t o = SESS_DEFAULT;
    bl_app_t a;
    printf("the host reuses a session id for another image (hub: random id, 1 in 128): the mixed body never runs\n");
    board_reset_model();
    make_image(&r4_a, 16000, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 441);
    make_image(&r4_b, 16000, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 443);
    make_image(&r4_hw, 16000, V(0, 6, 0, 0xFF), 1, REV_OK, 1, 445);   /* same length, other content */
    factory(&r4_a);
    board_power_on();
    o.passes = 1;
    o.end_ms = 0;
    loss_ppm = 300000;                                    /* 30 %: many pages missing */
    host_session(&r4_b, 0x55, &o);
    loss_ppm = 0;
    CHECK(!B.in_app && BS.state == BL_S_RECEIVE && BS.nwritten < BS.npages, "receiving 0x55, incomplete");
    host_session(&r4_hw, 0x55, &SESS_DEFAULT);            /* the other image under the same id fills the holes */
    host_idle(4000000u);                                  /* its END is of another body: the idle timeout commits */
    bl_app_read(&a);
    CHECK(!B.in_app || flash_has(&r4_b) || flash_has(&r4_hw), "never runs a mixed body (in_app %d)", B.in_app);
    CHECK(!B.in_app && BS.mode == BL_ACT_RECOVERY && !a.desc_ok, "VERIFY caught the mix: descriptor erased, recovery");
    host_session(&r4_hw, 0x56, &SESS_DEFAULT);
    host_after();
    CHECK(running(&r4_hw), "the retry installs");
    CHECK(fl_violations == 0 && keep_areas_intact(), "flash rules");
}

/* ----------------------------------------------------------------------------------------- VERIFY with VDD low */
static void test_r4_verify_low_power(void)
{
    static uint8_t snap[FL_SIZE];
    bl_app_t a;
    uint32_t ops;
    printf("VDD low during VERIFY: nothing written, decided again at the next boot\n");
    board_reset_model();
    make_image(&r4_a, 8000, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 451);
    make_image(&r4_b, 8000, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 453);
    factory(&r4_a);
    board_power_on();
    host_session(&r4_b, 0x81, &SESS_DEFAULT);             /* commits, verifies, trial window */
    host_idle(1000000u);
    /* a good body, committed but not yet verified, VDD low at the boot */
    fl_fill_erased((UPD_APP_DESC - FL_BASE) + 64u, 192u);
    fl_power = 0;
    memcpy(snap, fl, FL_SIZE);
    ops = fl_ops;
    board_power_on();
    CHECK(!B.in_app && BS.mode == BL_ACT_TRIAL, "good body, VDD low: verified in RAM, trial window as usual");
    host_idle(200000u);
    bl_app_read(&a);
    CHECK(B.in_app && fl_ops == ops && memcmp(snap, fl, FL_SIZE) == 0 && !a.verified,
          "the app runs; no VERIFIED and no trial record written (VDD low)");
    fl_power = 1;
    board_power_on();
    host_idle(200000u);
    bl_app_read(&a);
    CHECK(B.in_app && a.verified && a.trials == 1, "VDD fine: VERIFIED recorded at the next boot, then a trial record");
    /* a bad body under a valid, unverified descriptor, VDD low at the boot */
    fl_fill_erased((UPD_APP_DESC - FL_BASE) + 64u, 192u);
    fl[UPD_APP_BODY - FL_BASE + 300] ^= 0xFF;
    fl_power = 0;
    memcpy(snap, fl, FL_SIZE);
    ops = fl_ops;
    board_power_on();
    bl_app_read(&a);
    CHECK(!B.in_app && BS.mode == BL_ACT_RECOVERY && fl_ops == ops && memcmp(snap, fl, FL_SIZE) == 0,
          "bad body, VDD low: recovery, the descriptor not erased yet");
    CHECK(a.desc_ok, "(the descriptor is still in flash)");
    fl_power = 1;
    board_power_on();
    bl_app_read(&a);
    CHECK(!B.in_app && BS.mode == BL_ACT_RECOVERY && !a.desc_ok, "VDD fine: VERIFY again, descriptor erased");
    host_session(&r4_b, 0x82, &SESS_DEFAULT);
    host_after();
    CHECK(running(&r4_b), "a session installs");
    CHECK(fl_violations == 0 && keep_areas_intact(), "flash rules");
}

/* -------------------------------------------------------------------------------------- a torn status record */
static void test_r4_torn_record(void)
{
    bl_app_t a;
    uint32_t off = (UPD_APP_DESC - FL_BASE) + 2u * (UPD_ST_FIRST + 1u);
    printf("a torn status record (power cut while it was written) is skipped by the bootloader and the app\n");
    board_reset_model();
    make_image(&r4_a, 8000, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 461);
    make_image(&r4_b, 8000, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 463);
    factory(&r4_a);
    board_power_on();
    host_session(&r4_b, 0x91, &SESS_DEFAULT);
    host_idle(1000000u);                                  /* VERIFIED at 0, a trial record at 1 */
    bl_app_read(&a);
    CHECK(a.verified && a.st_next == 2, "VERIFIED + one trial record (%u)", (unsigned)a.st_next);
    fl[off] = 0x5A;                                       /* record 1 torn: neither erased nor a known value */
    fl[off + 1u] = 0x12;
    bl_app_read(&a);
    CHECK(a.verified && !a.confirmed && a.trials == 0 && a.st_next == 2, "torn record skipped, next slot after it");
    board_power_on();
    CHECK(!B.in_app && BS.mode == BL_ACT_TRIAL, "trial window");
    host_after();                                         /* normal frames end it; PING: the app confirms */
    bl_app_read(&a);
    CHECK(a.confirmed && a.st_next == 4, "trial record at 2, CONFIRMED at 3 (%u)", (unsigned)a.st_next);
    board_power_on();
    CHECK(B.in_app && B.last_act == BL_ACT_JUMP, "confirmed: fast path");
    /* a torn record that happens to read as erased: the next record is written over it (model: the same cell) */
    fl_fill_erased((UPD_APP_DESC - FL_BASE) + 66u, 190u);   /* records 1.. erased again: only VERIFIED stays */
    fl[(UPD_APP_DESC - FL_BASE) + 66u] = 0xFF;            /* reads 0xFFFF */
    fl[(UPD_APP_DESC - FL_BASE) + 67u] = 0xFF;
    bl_app_read(&a);
    CHECK(a.st_next == 1 && !a.confirmed, "0xFFFF counts as erased: slot 1 is the next");
    CHECK(fl_violations == 0 && keep_areas_intact(), "flash rules");
}

/* ------------------------------------------------------------------------------------------------ trial window */
static void test_r4_trial_window(void)
{
    bl_app_t a;
    uint64_t t0;
    uint8_t ws[48];
    printf("trial window: WS / bad / PING frames do not end it, the timeout and a normal frame do\n");
    board_reset_model();
    make_image(&r4_a, 8000, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 471);
    make_image(&r4_b, 8000, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 473);
    factory(&r4_a);
    board_power_on();
    host_session(&r4_b, 0xA1, &SESS_DEFAULT);
    host_idle(1000000u);
    board_power_on();
    CHECK(!B.in_app && BS.mode == BL_ACT_TRIAL, "trial window");
    t0 = B.now_us;
    memset(ws, 0x55, sizeof(ws));
    bl_sess_frame(&BS, 40, ws, ms());                     /* a WS frame */
    bl_sess_frame(&BS, 0, ws, ms());                      /* a bad frame (bl_main passes nbits 0) */
    host_upd(UPD_PING, 0, 0, 0, 0, 0, 0);
    B.now_us += 100000u;
    board_poll();
    CHECK(!B.in_app && BS.mode == BL_ACT_TRIAL && BS.state == BL_S_LISTEN, "100 ms of WS, bad and PING frames: still open");
    host_idle(60000u);
    CHECK(B.in_app && B.now_us - t0 >= BL_TRIAL_MS * 1000u, "ends at %u ms", (unsigned)BL_TRIAL_MS);
    bl_app_read(&a);
    CHECK(a.trials == 2 && a.strikes == 0, "a quiet trial record (trials %u, strikes %u)", (unsigned)a.trials,
          (unsigned)a.strikes);
    board_power_on();
    host_normal(2);
    bl_app_read(&a);
    CHECK(B.in_app && a.trials == 3 && a.strikes == 1, "a normal frame ends it at once: a strike");
    CHECK(fl_violations == 0 && keep_areas_intact(), "flash rules");
}

#if UPPER_RECORDS
/* ---------------------------------------------------------------- (0.6.5) the record pages above the slot */
static void test_r4_cal_page(void)
{
    uint32_t ops[32], n, k, past = 0;
    static image_t *big;
    printf("the largest image (%u B, to the zero-wait flash's end) and the record pages above it (0.6.5)\n",
           (unsigned)UPD_APP_BODY_MAX);
    n = bl_erase_plan(UPD_APP_BODY_MAX, ops, 32u);
    for (k = 0; k < n; k++) {
        uint32_t a = ops[k] & ~1u, sz = (ops[k] & 1u) ? 256u : 4096u;
        past |= a + sz > UPD_APP_END;
    }
    CHECK(!past, "erase plan for the largest image ends at 0x08010000");
    big = (image_t *)malloc(sizeof(image_t));
    board_reset_model();
    make_image(&r4_a, 8000, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 481);
    make_image(big, UPD_APP_BODY_MAX, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 483);
    factory(&r4_a);
    board_power_on();
    host_session(big, 0xB1, &SESS_DEFAULT);
    host_after();
    CHECK(running(big), "the %u-byte image installed", (unsigned)big->len);
    CHECK(fl_violations == 0 && keep_areas_intact(), "the record pages survived");
    free(big);
}
#else
/* ---------------------------------------------------------------------------- the calibration page at 0xF600 */
static void test_r4_cal_page(void)
{
    uint32_t ops[32], n, k, touches_cal;
    static image_t *big;
    printf("the calibration page (0x0800F600) under the linker's largest image, and the erase plan's edge\n");
    n = bl_erase_plan(R4_IMG_LINK_MAX, ops, 32u);
    for (k = 0, touches_cal = 0; k < n; k++) {
        uint32_t a = ops[k] & ~1u, sz = (ops[k] & 1u) ? 256u : 4096u;
        touches_cal |= (a <= 0x0800F600u && a + sz > 0x0800F600u);
    }
    CHECK(!touches_cal, "erase plan for %u bytes (Link_app.ld's slot) leaves 0xF600 alone", (unsigned)R4_IMG_LINK_MAX);
    n = bl_erase_plan(R4_IMG_LINK_MAX + 4u, ops, 32u);
    for (k = 0, touches_cal = 0; k < n; k++) {
        uint32_t a = ops[k] & ~1u, sz = (ops[k] & 1u) ? 256u : 4096u;
        touches_cal |= (a <= 0x0800F600u && a + sz > 0x0800F600u);
    }
    CHECK(touches_cal, "4 bytes more would erase it (the bootloader's slot ends at 0xF700: mkimage.py refuses this)");

    big = (image_t *)malloc(sizeof(image_t));
    board_reset_model();
    make_image(&r4_a, 8000, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 481);
    make_image(big, R4_IMG_LINK_MAX, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 483);
    factory(&r4_a);
    memset(&fl[R4_FL_CAL], 0x66, 256u);                   /* a calibration page with records */
    memset(&fl_prog[R4_FL_CAL / 2u], 1, 128u);            /* (programmed: any write there is a violation) */
    board_power_on();
    host_session(big, 0xB1, &SESS_DEFAULT);
    host_after();
    CHECK(running(big), "the %u-byte image installed", (unsigned)big->len);
    for (k = 0, touches_cal = 0; k < 256u; k++) {
        touches_cal |= (fl[R4_FL_CAL + k] != 0x66u);
    }
    CHECK(!touches_cal && fl_violations == 0 && keep_areas_intact(), "the calibration page and the record pages survived");
    free(big);
}

#endif

/* ------------------------------------------------------------ a reset with RAM kept at every flash operation */
static void test_r4_reset_cuts(uint32_t len, uint32_t stride)
{
    uint32_t total, k, cases = 0, after_ok = 0, ran_bad = 0, recov = 0, old_kept = 0, newer = 0;
    printf("watchdog reset (RAM and hand-over kept) at flash operation k (image %u B, every %u. op)\n", (unsigned)len,
           (unsigned)stride);
    board_reset_model();
    make_image(&r4_a, len, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 491);
    make_image(&r4_b, len, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 493);
    factory(&r4_a);
    board_power_on();
    fl_ops = 0;
    host_session(&r4_b, 0xC0, &SESS_DEFAULT);
    host_after();
    total = fl_ops;
    CHECK(running(&r4_b), "clean reference run");
    for (k = 1; k <= total; k += stride) {
        board_reset_model();
        factory(&r4_a);
        board_power_on();
        fl_ops = 0;
        fl_cut_at = k;
        if (setjmp(cut_jmp) == 0) {
            host_session(&r4_b, 0xC1, &SESS_DEFAULT);
            host_after();
        }
        fl_cut_at = 0;
        cases++;
        B.busy_until_us = 0;
        board_boot(BL_RST_WATCHDOG);                      /* the IWDG fired mid-operation: RAM survives */
        if (B.in_app) {
            if (flash_has(&r4_a)) {
                old_kept++;
            } else if (flash_has(&r4_b)) {
                newer++;
            } else {
                ran_bad++;
            }
        } else {
            recov++;
        }
        host_session(&r4_b, 0xC2, &SESS_DEFAULT);
        host_after();
        board_power_on();
        if (running(&r4_b)) {
            after_ok++;
        }
        CHECK(fl_violations == 0 && keep_areas_intact(), "reset at op %u: flash rules", (unsigned)k);
    }
    printf("  %u ops, %u cut points: after the reset %u ran the old app, %u the new one, %u in recovery, %u ran a broken"
           " image; %u / %u updated by the retry\n", (unsigned)total, (unsigned)cases, (unsigned)old_kept,
           (unsigned)newer, (unsigned)recov, (unsigned)ran_bad, (unsigned)after_ok, (unsigned)cases);
    CHECK(ran_bad == 0, "never runs a broken image");
    CHECK(after_ok == cases, "a retry always succeeds");
}

static void test_r4_all(void)
{
#ifdef BL_RESET_KIND_API
    test_r4_reset_kind();
#else
    printf("reset classification: bl_reset_kind not in this bootloader (01-reset-kind.patch): skipped\n");
#endif
    test_r4_foreign_announce();
    test_r4_hostile_frames();
    test_r4_restart_same_image();
    test_r4_session_id_reuse();
    test_r4_verify_low_power();
    test_r4_torn_record();
    test_r4_trial_window();
    test_r4_cal_page();
    test_r4_reset_cuts(6000, 1);
    test_r4_reset_cuts(30548, 5);
}
