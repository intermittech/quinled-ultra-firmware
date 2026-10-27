/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* (decided 2026-10-23) BOARD 1 / 2: the update-transparent plan's frame end is idle-only (frame_track:
 * 200-250 us of idle DIN, ULTRA_RW_REARM_US, no length or CRC proof), so the plan re-arms the gate closed and the tick opens it only after
 * >= 350 us of LOW without an edge (capture_isr.c gate_quiet): never inside a frame. Only valid UPD headers of whole
 * frames make the app transparent: a CRC-bad or a partial frame never counts. Every gate re-arm and every open
 * (gate_rearm / gate_open, logged by the hardware model) is checked against the wire. Included by test_soak.c. */
static void fo_send(const uint8_t *f, uint32_t nbits, uint32_t gap_us)
{
    while (wf_on) {
        run_ticks(1);
    }
    wire_frame(f, nbits, sim_ns + 7000u);
    while (wf_on) {
        run_ticks(1);
    }
    run_ticks(gap_us / 50u);
}

static void fo_ordinary(void)                  /* the host's normal frames: an ordinary header ends any UPD run */
{
    host_on = 1;
    host_next = sim_ticks;
    run_ms(60);
    host_on = 0;
    while (wf_on) {
        run_ticks(1);
    }
    run_ms(10);
}

static void sc_fwdopen(void)
{
    uint8_t f[UPD_FRAME_BYTES], bad[UPD_FRAME_BYTES];
    uint32_t i, k, o0, bits0;
    printf("soak fwdopen: the transparent plan opens only after >= 350 us of quiet LOW (BOARD %d)\n", BOARD);
    boot(0, 0, 0);
    fo_ordinary();
    fo_logging = 1;
    upd_build_frame(f, UPD_PING, 7, 0, 0, 0, 0, 0);
    memcpy(bad, f, sizeof bad);
    bad[UPD_FRAME_BYTES - 1] ^= 0x5Au;                 /* the header's CRC-8 */
    /* one valid UPD header, then CRC-bad ones */
    fo_send(f, UPD_FRAME_BITS, 20000u);
    for (k = 0; k < 3u; k++) fo_send(bad, UPD_FRAME_BITS, 20000u);
    CHECK(!upd_app_transparent(&upd) && fo_nopen == 0u, "fwdopen: one valid UPD header + CRC-bad ones: closed "
          "(transparent %u, opens %u)", upd_app_transparent(&upd), fo_nopen);
    /* one valid, then partial frames (cut after 200 bits) */
    fo_ordinary();
    fo_send(f, UPD_FRAME_BITS, 20000u);
    for (k = 0; k < 3u; k++) fo_send(f, 200u, 20000u);
    CHECK(!upd_app_transparent(&upd) && fo_nopen == 0u, "fwdopen: one valid UPD header + partial frames: closed "
          "(transparent %u, opens %u)", upd_app_transparent(&upd), fo_nopen);
    /* two valid UPD headers at 500 us gaps: transparent; the gate opens in a later gap and every later frame passes */
    fo_ordinary();
    o0 = fo_nopen;
    fo_send(f, UPD_FRAME_BITS, 500u);
    fo_send(f, UPD_FRAME_BITS, 500u);
    CHECK(upd_app_transparent(&upd), "fwdopen: two valid UPD headers: transparent");
    fo_send(f, UPD_FRAME_BITS, 500u);
    fo_send(f, UPD_FRAME_BITS, 500u);
    memset(forward_bytes, 0, sizeof forward_bytes);
    forward_bits = 0;
    forward_watch = 1;
    bits0 = forward_bits;
    fo_send(f, UPD_FRAME_BITS, 500u);
    forward_watch = 0;
    CHECK(fo_nopen > o0, "fwdopen: the gate opened (%u opens)", fo_nopen - o0);
    CHECK(forward_bits - bits0 == UPD_FRAME_BITS && memcmp(forward_bytes, f, UPD_FRAME_BYTES) == 0,
          "fwdopen: a frame after the open passes whole (%u bits)", forward_bits - bits0);
    /* a frame 300 us after the last one (below the 500 us minimum): it is blocked whole, never cut */
    memset(forward_bytes, 0, sizeof forward_bytes);
    forward_bits = 0;
    forward_watch = 1;
    bits0 = forward_bits;
    fo_send(f, UPD_FRAME_BITS, 300u);
    fo_send(f, UPD_FRAME_BITS, 500u);
    forward_watch = 0;
    CHECK(forward_bits - bits0 == 0u || forward_bits - bits0 == UPD_FRAME_BITS || forward_bits - bits0 == 2u * UPD_FRAME_BITS,
          "fwdopen: frames 300 us apart pass whole or not at all (%u bits)", forward_bits - bits0);
    /* every open: on an idle line >= 350 us after a complete frame's last edge; every re-arm: >= ULTRA_RW_REARM_US idle after
     * an Ultra plan's frame, >= 150 us after a WS2805 / WS2915 plan's (its reset gap is >= 280 us) */
    for (i = 0; i < fo_nopen; i++) {
        CHECK(!fo_open[i].midframe && fo_open[i].idle_ns >= 350000u, "fwdopen: open %u at %.3f ms %s, %.1f us after "
              "the last edge", i, fo_open[i].t_ns / 1e6, fo_open[i].midframe ? "INSIDE a frame" : "idle",
              fo_open[i].idle_ns / 1000.0);
    }
    for (i = 0; i < fo_n; i++) {
        CHECK(!fo_log[i].midframe && fo_log[i].idle_ns >= (fo_log[i].ws_end ? 150u : ULTRA_RW_REARM_US) * 1000u, "fwdopen: re-arm %u at "
              "%.3f ms %s, %.1f us after the last edge (plan open %u, ws_end %u)", i, fo_log[i].t_ns / 1e6,
              fo_log[i].midframe ? "INSIDE a frame" : "idle", fo_log[i].idle_ns / 1000.0, fo_log[i].start_open, fo_log[i].ws_end);
        CHECK(!(fo_log[i].start_open && fo_log[i].own_bits == UPD_FRAME_BITS), "fwdopen: re-arm %u opened the "
              "transparent plan at once", i);
    }
    printf("  %u re-arms, %u opens: every open >= 350 us after a complete frame's last edge, never inside a frame\n",
           fo_n, fo_nopen);
    fo_logging = 0;
}
