/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "proto.h"
#include "lto.h"

void proto_init(proto_state_t *s, proto_mode_t mode, proto_t sticky)
{
    s->mode = mode;
    s->locked = PROTO_NONE;
    s->cand = PROTO_NONE;
    s->cand_count = 0;
    s->forced_hdr = 32;
    s->provisional = 0;
    if (mode == PMODE_AUTO && sticky > PROTO_NONE && sticky <= PROTO_ULTRA) {
        s->locked = sticky;                              /* gate and decoder ready for the first frame */
        s->provisional = 1;
        s->forced_hdr = (sticky == PROTO_WS2915_26) ? 26u : 32u;
    }
}

proto_t proto_classify(uint32_t total_bits, uint8_t ultra)
{
    if (ultra & PROTO_UH_HEADER) {
        return (total_bits >= 64u && (total_bits - 64u) % 256u == 0u) ? PROTO_ULTRA : PROTO_NONE;
    }
    if (total_bits >= 112u && (total_bits - 32u) % 80u == 0u) {
        return PROTO_WS2915_32;
    }
    if (total_bits >= 106u && (total_bits - 26u) % 80u == 0u) {
        return PROTO_WS2915_26;
    }
    if (total_bits >= 40u && total_bits % 40u == 0u) {
        return PROTO_WS2805;
    }
    return PROTO_NONE;
}

proto_t proto_frame_end(proto_state_t *s, uint32_t total_bits, uint8_t ultra)
{
    proto_t c = proto_classify(total_bits, ultra);

    if (c == PROTO_WS2915_32 || c == PROTO_WS2915_26) {
        s->forced_hdr = (c == PROTO_WS2915_26) ? 26u : 32u;   /* also used by forced WS2915 mode */
    }
    if (c == PROTO_NONE) {
        s->cand_count = 0;                                   /* odd frame: keep the current lock */
        return c;
    }
    if (c == s->locked) {
        s->cand_count = 0;
        s->provisional = 0;                                  /* the stored protocol is confirmed */
        return c;
    }
    if (s->provisional) {                                    /* the stored protocol was wrong: forward nothing */
        s->locked = PROTO_NONE;                              /* until the new one has locked */
        s->provisional = 0;
    }
    if (c == PROTO_ULTRA && (ultra & PROTO_UH_SLOT)) {
        s->locked = c;                                       /* header CRC-8 + slot CRC-16: one frame is enough */
        s->cand_count = 0;
        return c;
    }
    if (c == s->cand) {
        if (++s->cand_count >= PROTO_LOCK_FRAMES) {
            s->locked = c;
            s->cand_count = 0;
        }
    } else {
        s->cand = c;
        s->cand_count = 1;
    }
    return c;
}

proto_t proto_active(const proto_state_t *s)
{
    switch (s->mode) {
    case PMODE_WS2915:
        return s->forced_hdr == 26u ? PROTO_WS2915_26 : PROTO_WS2915_32;
    case PMODE_WS2805:
        return PROTO_WS2805;
    case PMODE_ULTRA:
        return PROTO_ULTRA;
    default:
        return s->locked;
    }
}

void proto_set_mode(proto_state_t *s, proto_mode_t mode)
{
    proto_t cur = proto_active(s);
    s->mode = mode;
    if (mode == PMODE_AUTO && cur != PROTO_NONE) {
        s->locked = cur;
        s->cand_count = 0;
    }
    s->provisional = 0;                                  /* a forced mode has no stored protocol to confirm either:
                                                            frx_capture would hold its WS slots to the frame end */
}

gate_plan_t proto_gate_plan(proto_t p)
{
    gate_plan_t g = { 0, 0, 0, 0, 0 };
    switch (p) {
    case PROTO_WS2915_32:
    case PROTO_WS2915_26:
        g.hdr_bits = (p == PROTO_WS2915_26) ? 26u : 32u;
        g.start_open = 1;                    /* header passes to every board */
        g.t1 = g.hdr_bits;                   /* close: our 80 bits */
        g.t2 = (uint16_t)(g.hdr_bits + 80u); /* open: the rest */
        g.own_bits = g.t2;
        break;
    case PROTO_WS2805:
        g.start_open = 0;                    /* our 40 bits blocked */
        g.t1 = 40;
        g.own_bits = 40;
        break;
    case PROTO_ULTRA:
        g.start_open = 0;                    /* our 256-bit slot blocked (the line stays LOW, as before the */
        g.t1 = 256;                          /* frame), then open: the boards after us and the header */
        g.own_bits = 256;
        break;
    default:                                 /* not locked yet: forward nothing, capture the longest WS slot */
        g.own_bits = PROTO_DETECT_BITS;
        break;
    }
    return g;
}

gate_plan_t proto_gate_plan_transparent(void)
{
    gate_plan_t g = { 1, 0, 0, PROTO_UPD_BITS, 0 };   /* open all frame, no toggle; capture a whole update frame */
    return g;
}

int proto_tail_wanted(uint32_t total_bits)
{
    return (total_bits >= 64u && (total_bits - 64u) % 256u == 0u) || total_bits == PROTO_UPD_BITS;
}

/* n <= 25 bits at bit position pos, MSB first; reads 4 bytes, so callers keep 4 bytes of slack */
static uint32_t getbits(const uint8_t *bits, uint32_t pos, uint32_t n)
{
    const uint8_t *p = bits + (pos >> 3);
    uint32_t w = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
    return (w << (pos & 7u)) >> (32u - n);
}

int proto_parse_data(proto_t p, const uint8_t *bits, slot_t *out)
{
    uint32_t i;
    if (p != PROTO_WS2915_32 && p != PROTO_WS2915_26) {
        return -1;
    }
    for (i = 0; i < 5u; i++) {
        out->gain[i] = 0;
    }
    for (i = 0; i < 10u; i++) {
        out->raw[i] = bits[i];
    }
    for (i = 0; i < 5u; i++) {
        out->ch[i] = (uint16_t)((out->raw[2u * i] << 8) | out->raw[2u * i + 1u]);
    }
    out->raw_len = 10;
    out->hdr_ok = 1;
    return 0;
}

MEASURED int proto_parse(proto_t p, const uint8_t *bits, slot_t *out)
{
    uint32_t i, pos;

    if (p == PROTO_WS2805) {
        for (i = 0; i < 5u; i++) {
            uint8_t v = (uint8_t)getbits(bits, i * 8u, 8u);
            out->raw[i] = v;
            out->ch[i] = (uint16_t)(v * 257u);
            out->gain[i] = 0;
        }
        out->raw_len = 5;
        out->hdr_ok = 1;
        return 0;
    }
    if (p == PROTO_WS2915_32 || p == PROTO_WS2915_26) {
        uint32_t hb = (p == PROTO_WS2915_26) ? 26u : 32u;
        uint32_t hdr = (getbits(bits, 0, 16) << (hb - 16u)) | getbits(bits, 16, hb - 16u);
        uint32_t g = (p == PROTO_WS2915_26) ? (hdr >> 1) : (hdr >> 7);
        out->hdr_ok = (p == PROTO_WS2915_26) ? (uint8_t)(hdr & 1u) : (uint8_t)((hdr & 0x7Fu) == 0u);
        for (i = 0; i < 5u; i++) {
            out->gain[i] = (uint8_t)((g >> (20u - 5u * i)) & 31u);
        }
        pos = hb;
        for (i = 0; i < 10u; i++) {
            out->raw[i] = (uint8_t)getbits(bits, pos + i * 8u, 8u);
        }
        for (i = 0; i < 5u; i++) {
            out->ch[i] = (uint16_t)((out->raw[2u * i] << 8) | out->raw[2u * i + 1u]);
        }
        out->raw_len = 10;
        return 0;
    }
    return -1;
}
