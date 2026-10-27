/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Boot and lock tests (included by test_main.c): frame tracker (core/frame_track.h), sticky auto, the one-frame
 * Ultra lock. The chain behaviour (what downstream boards really see on the wire) is in test_wire.c. */
#include "../src/core/frame_track.h"

/* ------------------------------------------------------------------------------------------------ tracker */
static uint32_t ft_run(ftrack_t *t, const uint16_t *c, int n, uint16_t own, uint32_t *all)
{
    uint32_t ev, any = 0;
    int i;
    for (i = 0; i < n; i++) {
        ev = ftrack_tick(t, c[i], own, 0, 1);            /* (the WS end: FT_END_TICKS; the Ultra end below) */
        if (all) all[i] = ev;
        any |= ev;
    }
    return any;
}

static void test_ftrack(void)
{
    ftrack_t t;
    uint32_t ev[16];
    printf("frame tracker: boot arm on an idle line, partial frames, short frames, disarm\n");

    /* idle line after boot: arms after FT_BOOT_TICKS ticks (10 ms), the next frame is captured from its first edge */
    {
        static const uint16_t c[] = { 0, 0, 0, 0, 5, 45, 90, 112, 112, 112, 112, 112 };
        uint32_t i, early = 0;
        memset(&t, 0, sizeof t);
        for (i = 0; i < FT_BOOT_TICKS - 3u; i++) early |= ftrack_tick(&t, 0, 112, 0, 1);
        CHECK(!(early & FT_BOOT_ARM) && !t.armed, "no boot arm before 10 ms of idle line");
        ft_run(&t, c, 12, 112, ev);
        CHECK(ev[2] == FT_BOOT_ARM && ev[3] == 0, "boot arm after %u idle ticks (%x)", FT_BOOT_TICKS, ev[2]);
        CHECK(ev[4] == (FT_CAP_START | FT_EDGE) && t.cap_bits == 112, "first frame after boot captured");
        CHECK(ev[6] == FT_EDGE && ev[7] == FT_EDGE && ev[8] == FT_CAP_COPY, "edge ticks, copy one tick after own bits");
        CHECK(ev[10] == FT_FRAME_END, "frame end, classified (%x)", ev[10]);
    }
    /* booted mid-frame: no capture, frame end flagged partial (not classified), the next frame is captured */
    {
        static const uint16_t c[] = { 30, 70, 100, 100, 100, 100, 0, 0, 4, 40, 40, 40, 40 };
        memset(&t, 0, sizeof t);
        ft_run(&t, c, 13, 64, ev);
        CHECK(!(ev[0] & FT_CAP_START) && ev[5] == (FT_FRAME_END | FT_PARTIAL), "running frame is partial (%x)", ev[5]);
        CHECK(ev[8] == (FT_CAP_START | FT_EDGE), "next frame captured (%x)", ev[8]);
        CHECK(ev[12] == (FT_FRAME_END | FT_CAP_SHORT) && t.cap_bits == 40, "short frame: copy what came (%x)", ev[12]);
    }
    /* idle for only 2 ticks before the first edges: not sure it was a gap, so that frame is partial */
    {
        static const uint16_t c[] = { 0, 0, 8, 50, 50, 50, 50 };
        memset(&t, 0, sizeof t);
        ft_run(&t, c, 7, 40, ev);
        CHECK(!(ev[2] & FT_CAP_START) && ev[6] == (FT_FRAME_END | FT_PARTIAL), "too short an idle gap: partial");
    }
    /* WS2915 at board k: header (32), a gap of k x 100 us (the boards before block their bits), then the data.
     * The gap does not end the frame, and the capture restarts at the data (80 bits) */
    {
        static const uint16_t c[] = { 0, 0, 0, 0, 20, 32, 32, 32, 32, 32, 32, 32, 32, 50, 100, 112, 140, 140, 140, 140 };
        uint32_t all = 0;
        int i;
        memset(&t, 0, sizeof t);
        t.armed = 1;                                     /* a frame end came before (not the boot rule) */
        for (i = 0; i < 20; i++) {
            ev[i < 16 ? i : 15] = ftrack_tick(&t, c[i], 112, 32, 1);
            if (i >= 5 && i <= 12) all |= ev[i];
        }
        CHECK(!(all & FT_FRAME_END), "a 350 us gap after the WS2915 header does not end the frame");
        CHECK(ev[13] == (FT_CAP_START | FT_CAP_DATA | FT_EDGE), "capture restarts at the data (%x)", ev[13]);
        CHECK(ev[15] == FT_FRAME_END, "frame end after the data (%x)", ev[15]);
        memset(&t, 0, sizeof t);                         /* no gap (board 0): the normal capture from the frame start */
        t.armed = 1;
        {
            static const uint16_t c2[] = { 0, 0, 0, 0, 20, 60, 100, 112, 112, 112, 112 };
            uint32_t a2 = 0;
            for (i = 0; i < 11; i++) a2 |= ftrack_tick(&t, c2[i], 112, 32, 1);
            CHECK(!(a2 & FT_CAP_DATA) && (a2 & FT_CAP_COPY), "board 0: one capture from the frame start");
        }
        memset(&t, 0, sizeof t);                         /* 32 edges then silence: ends after FT_HOLD_TICKS */
        t.armed = 1;
        for (i = 0; i < 4; i++) ftrack_tick(&t, 0, 112, 32, 1);
        ftrack_tick(&t, 32, 112, 32, 1);
        for (i = 0, all = 0; i < (int)FT_HOLD_TICKS + 2; i++) all |= ftrack_tick(&t, 32, 112, 32, 1) & FT_FRAME_END ? (uint32_t)i : 0u;
        CHECK(all >= FT_HOLD_TICKS - 1u, "a lone 32-bit burst ends after the hold time (%u)", all);
    }
    /* flash write mid-frame: disarm drops the capture, the frame end is partial, the next idle gap re-arms */
    {
        static const uint16_t c1[] = { 0, 0, 0, 6, 60 };
        static const uint16_t c2[] = { 300, 300, 300, 300, 0, 0, 0, 0, 3 };
        memset(&t, 0, sizeof t);
        t.armed = 1;
        ft_run(&t, c1, 5, 320, NULL);
        CHECK(t.cap_state == 1, "capturing");
        ftrack_disarm(&t);
        ft_run(&t, c2, 9, 320, ev);
        CHECK(ev[3] == (FT_FRAME_END | FT_PARTIAL) && ev[0] == FT_EDGE, "disarmed frame: partial, no copy");
        CHECK(ev[8] == (FT_CAP_START | FT_EDGE), "armed again at the next frame end");
        memset(&t, 0, sizeof t);
        t.armed = 1;
        ftrack_disarm(&t);                               /* idle line: re-armed by the boot rule */
        {
            uint32_t i;
            for (i = 0; i < FT_BOOT_TICKS - 3u; i++) ftrack_tick(&t, 0, 320, 0, 1);
        }
        ft_run(&t, c2 + 4, 5, 320, ev);
        CHECK(ev[2] == FT_BOOT_ARM && ev[4] == (FT_CAP_START | FT_EDGE), "disarmed on an idle line: arms after 10 ms");
        /* power-up inside a LOW gap of a frame (the gates before us): 600 us quiet, then the rest of the frame. It
         * must not arm there: the rest is partial (not captured, not classified), the next frame is captured */
        {
            uint32_t i, all = 0;
            memset(&t, 0, sizeof t);
            for (i = 0; i < 12u; i++) all |= ftrack_tick(&t, 0, 256, 0, 0);   /* the gap */
            for (i = 1; i <= 20u; i++) all |= ftrack_tick(&t, (uint16_t)(i * 50u), 256, 0, 0);   /* the rest */
            for (i = 0; i < FT_REARM_TICKS; i++) all |= ftrack_tick(&t, 1000, 256, 0, 0);   /* frame end: counter back to 0 */
            CHECK(!(all & (FT_BOOT_ARM | FT_CAP_START)) && (all & FT_PARTIAL), "power-up inside a gap: rest of the frame partial");
            all = 0;
            for (i = 1; i <= 8u; i++) all |= ftrack_tick(&t, (uint16_t)(i * 50u), 256, 0, 0);
            CHECK(all & FT_CAP_START, "the next frame is captured");
        }
    }
    /* the frame end per plan: an Ultra plan after FT_REARM_TICKS idle ticks (200-250 us of LOW: past the hub's release), a WS plan
     * after FT_END_TICKS (150-200 us: its reset gap is >= 280 us) */
    {
        uint32_t ws, i, at;
        for (ws = 0; ws < 2u; ws++) {
            memset(&t, 0, sizeof t);
            t.armed = 1;
            (void)ftrack_tick(&t, 300, 256, 0, ws);
            for (i = 1, at = 0; i <= 8u && !at; i++) at = ftrack_tick(&t, 300, 256, 0, ws) & FT_FRAME_END ? i : 0u;
            CHECK(at == (ws ? FT_END_TICKS : FT_REARM_TICKS), "%s plan: the frame ends after %u idle ticks", ws ? "WS" : "Ultra", at);
        }
        CHECK(FT_REARM_TICKS * MCU_POLL_US >= ULTRA_RW_REARM_US && (FT_REARM_TICKS + 1u) * MCU_POLL_US <= ULTRA_DP_RELEASE_MAX_US &&
              (FT_END_TICKS + 1u) * MCU_POLL_US < 280u, "end ticks: Ultra %u (>= 200 us, < 250 us), WS %u (< 280 us)",
              FT_REARM_TICKS, FT_END_TICKS);
    }
}

/* ------------------------------------------------------------------------------------------------ lock rules */
static void test_sticky_lock(void)
{
    proto_state_t s;
    settings_t st, back;
    store_slot_t slot;
    uint8_t page[256];
    uint32_t seq;
    int i;
    printf("lock: sticky auto (provisional), one-frame Ultra lock, WS still 3 frames\n");

    proto_init(&s, PMODE_AUTO, PROTO_ULTRA);
    CHECK(proto_active(&s) == PROTO_ULTRA && s.provisional, "sticky Ultra active from power-up");
    proto_frame_end(&s, 77, 0);
    CHECK(proto_active(&s) == PROTO_ULTRA && s.provisional, "odd frame keeps the stored protocol");
    proto_frame_end(&s, 64 + 256 * 2, PROTO_UH_HEADER);
    CHECK(proto_active(&s) == PROTO_ULTRA && !s.provisional, "first Ultra frame confirms it");

    proto_init(&s, PMODE_AUTO, PROTO_WS2805);
    proto_frame_end(&s, 192, 0);                         /* a WS2915 frame (32 + 2 x 80) */
    CHECK(proto_active(&s) == PROTO_NONE && !s.provisional, "other protocol: forward nothing until it locks");
    proto_frame_end(&s, 192, 0);
    CHECK(proto_active(&s) == PROTO_NONE, "not yet");
    proto_frame_end(&s, 192, 0);
    CHECK(proto_active(&s) == PROTO_WS2915_32, "re-locked after 3 frames, the first one counting");

    proto_init(&s, PMODE_AUTO, PROTO_WS2915_26);
    CHECK(proto_active(&s) == PROTO_WS2915_26, "sticky 26-bit header");
    proto_init(&s, PMODE_WS2805, PROTO_ULTRA);
    CHECK(proto_active(&s) == PROTO_WS2805 && !s.provisional, "forced mode ignores the stored protocol");
    proto_init(&s, PMODE_AUTO, (proto_t)9);
    CHECK(proto_active(&s) == PROTO_NONE, "invalid stored value ignored");
    proto_init(&s, PMODE_ULTRA, PROTO_NONE);
    proto_set_mode(&s, PMODE_AUTO);
    CHECK(proto_active(&s) == PROTO_ULTRA && !s.provisional, "forced Ultra -> auto keeps Ultra (no gap)");
    proto_set_mode(&s, PMODE_WS2805);
    CHECK(proto_active(&s) == PROTO_WS2805, "auto -> forced WS2805");

    proto_init(&s, PMODE_AUTO, PROTO_NONE);
    proto_frame_end(&s, 320, PROTO_UH_HEADER | PROTO_UH_SLOT);
    CHECK(proto_active(&s) == PROTO_ULTRA, "Ultra: header + slot CRCs lock on the first frame");
    proto_init(&s, PMODE_AUTO, PROTO_NONE);
    proto_frame_end(&s, 64, PROTO_UH_HEADER);
    proto_frame_end(&s, 64, PROTO_UH_HEADER);
    CHECK(proto_active(&s) == PROTO_NONE, "header without a slot: like WS");
    proto_frame_end(&s, 64, PROTO_UH_HEADER);
    CHECK(proto_active(&s) == PROTO_ULTRA, "... 3 frames");
    proto_init(&s, PMODE_AUTO, PROTO_NONE);
    proto_frame_end(&s, 321, PROTO_UH_HEADER | PROTO_UH_SLOT);
    CHECK(proto_active(&s) == PROTO_NONE, "valid header and slot but an odd length: no lock");
    for (i = 0; i < 2; i++) proto_frame_end(&s, 400, 0);
    CHECK(proto_active(&s) == PROTO_NONE, "WS2805 needs 3 frames");
    proto_frame_end(&s, 400, 0);
    CHECK(proto_active(&s) == PROTO_WS2805, "WS2805 locked");
    proto_frame_end(&s, 320, PROTO_UH_HEADER | PROTO_UH_SLOT);
    CHECK(proto_active(&s) == PROTO_ULTRA, "a locked board switches to Ultra on one proven frame");

    /* the stored protocol survives a (simulated) power cycle through the flash record */
    settings_default(&st);
    CHECK(st.last_proto == PROTO_NONE, "default: nothing stored");
    st.last_proto = PROTO_ULTRA;
    memset(page, 0xFF, sizeof page);
    store_pack(&st, 7, &slot);
    memcpy(page, &slot, sizeof slot);
    CHECK(store_find(page, &back, &seq) == 0 && back.last_proto == PROTO_ULTRA, "stored");
    proto_init(&s, (proto_mode_t)back.protocol, (proto_t)back.last_proto);
    CHECK(proto_active(&s) == PROTO_ULTRA && s.provisional, "after the reboot: Ultra from the first frame");
}
