/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* A silent end to an update must restore gating before the next frame. */
static void sc_r4gate(void)
{
    upd_app_info_t info = { 0 };
    uint32_t rearms;
    frx_init(&frx, PMODE_ULTRA, PROTO_NONE);
    upd_app_init(&upd, &info);
    upd.transparent = 1;
    upd.last_upd_ms = 0;
    ms_now = UPD_APP_IDLE_MS;
    plan = plan_buf[0] = plan_buf[1] = proto_gate_plan_transparent();
    plan_idx = 0;
    memset(&ft, 0, sizeof ft); ft.armed = 1;
    soak_tim4.CNT = 0;
    upd_service();
    CHECK(!upd_app_transparent(&upd) && plan.own_bits == ULTRA_SLOT_BITS &&
          plan_buf[plan_idx].own_bits == ULTRA_SLOT_BITS, "timeout left the forwarding gate transparent");
    /* If data started concurrently, publish the next plan without cutting this frame. */
    upd.transparent = 1;
    plan = proto_gate_plan_transparent();
    ft.cap_state = 1; soak_tim4.CNT = 40;
    rearms = sim_rearms;
    upd_service();
    CHECK(sim_rearms == rearms && plan.own_bits == UPD_FRAME_BITS &&
          plan_buf[plan_idx].own_bits == ULTRA_SLOT_BITS, "timeout reset an active frame");
}
