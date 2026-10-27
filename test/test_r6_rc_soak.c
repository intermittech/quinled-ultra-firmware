/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Included by test_soak_rc.c: exercise the actual main.c return-channel path. */
static void sc_r6rc(void)
{
    rc_obs_t o;
    uint8_t p[32], a[9] = { ULTRA_PW_ARG_ACK }, d[24] = {0};
    uint16_t before;
    uint32_t saved_up, saved_cal;
    rc_boot_locked();
    rc_poll_build(p, rc_seq++, 0, 1, ULTRA_PAGE_STATUS, 0, 0, 0);
    rc_window(p, 1, ULTRA_HDR_RW | ULTRA_HDR_EXT, 0, 0, &o);
    CHECK(!o.turned && !o.sent, "R6: unknown must-understand EXT must suppress RW");
    rc_window(p, 1, ULTRA_HDR_RW | ULTRA_HDR_UPD, 0, 0, &o);
    CHECK(!o.turned && !o.sent, "R6: RW+UPD must remain silent");
    rc_window(p, 1, ULTRA_HDR_RW | ULTRA_HDR_HINT, 0, 0, &o);
    CHECK(o.turned && o.sent, "R6: optional HINT still allows RW");

    before = pw.ack_limit;
    pw.limit_events = (uint16_t)(before + 1u);
    ultra_put16(a + 1, pw.limit_events);
    rc_poll_build(p, rc_seq++, 0, 1, ULTRA_PAGE_POWER, 0, a, sizeof a);
    rc_window(p, 1, ULTRA_HDR_RW | ULTRA_HDR_EXT, 0, 0, &o);
    CHECK(pw.ack_limit == before, "R6: unknown EXT must not execute a POWER ACK");

    rc_contend_tx = 1;
    rc_poll_build(p, rc_seq++, 0, 2, ULTRA_PAGE_STATUS, 0, 0, 0);
    rc_window(p, 1, ULTRA_HDR_RW, 0, 0, &o);
    CHECK(o.turned && !o.sent && o.quiet, "R6: burst contention must stop upstream relay until W");

    memset(a, 0, sizeof a); /* calibration kind 0, part 0 */
    saved_up = up_s;
    saved_cal = cal_done_s;
    cal_done_s = 1; /* a calibration at uptime 0 */
    up_s = 0xFFFFFFu;
    (void)rc_page(ULTRA_PAGE_CALIBRATION, a, d);
    CHECK(ultra_get24(d + 4) == 0xFFFFFEu, "R6: known calibration age must not become NONE at 194 days");
    up_s = 0x1000000u;
    (void)rc_page(ULTRA_PAGE_CALIBRATION, a, d);
    CHECK(ultra_get24(d + 4) == 0xFFFFFEu, "R6: calibration age must saturate, not become fresh again");
    cal_done_s = saved_cal;
    up_s = saved_up;
}
