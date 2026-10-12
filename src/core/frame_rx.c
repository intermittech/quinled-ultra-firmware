/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "frame_rx.h"

void frx_init(frx_t *r, proto_mode_t mode, proto_t sticky)
{
    proto_init(&r->ps, mode, sticky);
    r->cap_frame = 0xFFFFFFFFu;
    r->have_ultra = 0;
    r->have_ws = 0;
    r->ws_proto = PROTO_NONE;
}

int frx_tail_wanted(const frx_t *r, uint32_t total)
{
    return proto_tail_wanted(total) ||
           (r->ps.mode == PMODE_AUTO && (r->ps.locked == PROTO_NONE || r->ps.provisional) && total >= ULTRA_HDR_BITS);
}

uint32_t frx_capture(frx_t *r, uint32_t frame, proto_t p, const uint8_t *bits, uint32_t nbits, uint32_t own,
                     uint32_t skip, slot_t *ws)
{
    uint32_t i;
    r->cap_frame = frame;
    r->have_ultra = 0;
    r->have_ws = 0;
    if (skip != 0u) {                                    /* WS2915 data after the header pause */
        if (nbits < 80u || proto_parse_data(p, bits, ws) != 0) {
            return 0;
        }
    } else if (nbits >= ULTRA_SLOT_BITS && (p == PROTO_ULTRA || p == PROTO_NONE || r->ps.provisional) &&
        ultra_slot_check(bits) == 0) {
        for (i = 0; i < ULTRA_SLOT_BYTES; i++) {
            r->slot[i] = bits[i];                        /* applied once the header at the frame end is in */
        }
        r->have_ultra = 1;
    }
    if (skip == 0u && (p == PROTO_ULTRA || p == PROTO_NONE || nbits < own || proto_parse(p, bits, ws) != 0)) {
        return 0;
    }
    if (p != proto_active(&r->ps)) {
        return 0;                                        /* the frame before changed the protocol meanwhile */
    }
    if (r->ps.provisional) {                             /* stored protocol: the frame length must confirm it */
        r->ws = *ws;
        r->ws_proto = (uint8_t)p;
        r->have_ws = 1;
        return 0;
    }
    return FRX_SHOW_WS;
}

void frx_frame_end(frx_t *r, uint32_t frame, uint32_t total, const uint8_t *tail8, frx_end_t *e)
{
    uint8_t ultra = 0, was_prov = r->ps.provisional, mine = (uint8_t)(r->cap_frame == frame);
    uint32_t t_us;

    e->close_gate = e->show_ws = e->ultra_header = e->ultra_sync = e->ultra_slot = e->flags = 0;
    e->host_us = 0;
    e->before = proto_active(&r->ps);
    if (tail8 != 0 && ultra_parse_header(tail8, &t_us) == 0) {
        ultra = PROTO_UH_HEADER;
        e->ultra_header = 1;
        e->host_us = t_us;
        e->flags = ultra_header_flags(tail8);
        if (mine && r->have_ultra) {
            ultra |= PROTO_UH_SLOT;                      /* header CRC-8 + slot CRC-16: one frame locks */
        }
    }
    e->cls = proto_frame_end(&r->ps, total, ultra);
    e->after = proto_active(&r->ps);
    if (was_prov && e->after != e->before) {
        e->close_gate = 1;                               /* its plan may cut the next frame at the wrong bits */
    }
    if (mine && r->have_ws && e->cls == (proto_t)r->ws_proto && e->after == e->cls && !e->ultra_header) {
        e->show_ws = 1;                                  /* the stored protocol is confirmed by this frame */
    }
    if (e->ultra_header && e->after == PROTO_ULTRA && !(e->flags & ULTRA_HDR_UPD)) {
        e->ultra_sync = 1;                               /* RW / EXT (not supported): clock and lock, but no slot */
        e->ultra_slot = (uint8_t)(mine && r->have_ultra && !(e->flags & (ULTRA_HDR_RW | ULTRA_HDR_EXT)));
    }
    r->have_ultra = 0;
    r->have_ws = 0;
    e->plan = proto_gate_plan(e->after);
}
