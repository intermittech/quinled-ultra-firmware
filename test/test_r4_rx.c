/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Included by test_soak.c: inject the ISR while a mailbox is owned by main. */
static void sc_r4rx(void)
{
    gate_plan_t g = proto_gate_plan(PROTO_WS2805);
    frx_init(&frx, PMODE_WS2805, PROTO_NONE);
    plan = plan_buf[0] = plan_buf[1] = g;
    plan_idx = 0; rx_ready = 1; frames = 8;
    memset(&ft, 0, sizeof ft); ft.armed = 1;
    cap_ready = 1; cap_frame = 7; cap_words = 16; cap_bits = cap_own = 40;
    cap_skip = 0; cap_proto = PROTO_WS2805; cap_start = 0;
    din_copy[0] = 0x1234;
    soak_tim4.CNT = 40;
    poll_isr(); /* next frame start: must not replace the pending frame's metadata */
    CHECK(cap_frame == 7 && cap_words == 16, "owned capture metadata was overwritten");
    poll_isr(); /* next frame copy: must not overwrite the pending samples either */
    CHECK(din_copy[0] == 0x1234 && cap_words == 16, "owned capture samples were overwritten");
    fe_ready = 1; fe_frame = 6; fe_bits = 40; fe_tick = 123; tail_words = 16;
    tail_copy[0] = 0x5678;
    ft.idle = FT_END_TICKS - 1u;
    poll_isr();
    CHECK(fe_frame == 6 && fe_bits == 40 && fe_tick == 123 && tail_words == 16 && tail_copy[0] == 0x5678,
          "owned frame-end mailbox was overwritten");
    /* Drop the unread mailboxes, then a later frame must be admitted normally. */
    cap_ready = fe_ready = 0;
    soak_tim4.CNT = 40;
    poll_isr(); poll_isr();
    CHECK(cap_ready && cap_frame == frames && cap_bits == 40, "capture did not recover after release");
    process_capture();
    CHECK(!cap_ready, "decoder error path did not release the mailbox");
    /* A rejected WS2915 frame stays rejected when its data resumes after the header pause. */
    g = proto_gate_plan(PROTO_WS2915_32);
    frx_init(&frx, PMODE_WS2915, PROTO_NONE);
    plan = plan_buf[0] = plan_buf[1] = g;
    memset(&ft, 0, sizeof ft); ft.armed = 1;
    cap_ready = 1;
    soak_tim4.CNT = 32; poll_isr(); poll_isr();
    cap_ready = 0; /* main finished the earlier frame during this one's header pause */
    soak_tim4.CNT = 112; poll_isr(); poll_isr();
    CHECK(!cap_ready, "rejected WS2915 frame acquired stale metadata after its header pause");
}
