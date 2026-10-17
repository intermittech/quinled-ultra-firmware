/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The Ultra-Digital receiver's CHAIN line mode: see udchain.h. */
#include "udchain.h"
#include "../ws_decode.h"
#include "../ultra_update.h"

/* dline's late limit for an armed answer is the protocol's: ULTRA_RW_LATE_US in ring words (2.25 a us) */
typedef char udc_late_is_the_protocols[DLINE_W_CH_LATE == ULTRA_RW_LATE_US * 9u / 4u ? 1 : -1];

/* ------------------------------------------------------------------------------------------------ classify */
static void udc_stream_reset(udc_pre_t *p)
{
    uint8_t *q = (uint8_t *)p;
    uint32_t i;
    for (i = 0; i < (uint32_t)sizeof *p; i++) {
        q[i] = 0;
    }
}

void udc_stream(dline_t *d, udc_pre_t *p)
{
    uint32_t hn;
    const uint16_t *head;
    if (p->js != d->ch_cjs) {              /* a new frame in the capture: start again */
        udc_stream_reset(p);
        p->js = d->ch_cjs;
    }
    if (p->sb != 0) {                      /* all 256 bits out, or the head is no frame (the error stays) */
        return;
    }
    head = dline_ch_head(d, &hn);
    hn = *(volatile uint16_t *)&d->ch_hn;  /* (the tick's: words below it are final) */
    p->sb = (int16_t)ws_rise_step(&p->st, head, hn, ULTRA_SLOT_BITS, p->slot);   /* (the CRC-16 checks it: the rising-edge decode is enough) */
}

uint32_t udc_classify(dline_t *d, udc_rx_t *rx, udc_pre_t *p)
{
    const uint32_t n = d->ch_bits;
    uint32_t ts, tn, i;
    const uint16_t *tail;
    int nb, sb;

    rx->cls = UDC_C_NONULTRA;
    rx->flags = 0;
    rx->slots = 0;
    rx->host = 0;
    rx->bits = n;
    /* the length: 256 x n + 64 bits, or an UPD frame's (384: it carries the update, not a slot) */
    if (n < ULTRA_SLOT_BITS + ULTRA_HDR_BITS || n > ULTRA_HDR_BITS + ULTRA_SLOT_BITS * UDC_LONG_SLOTS ||
        (((n - ULTRA_HDR_BITS) & (ULTRA_SLOT_BITS - 1u)) != 0u && n != UPD_FRAME_BITS)) {
        dline_ch_free(d);
        return UDC_C_NONULTRA;
    }
    /* the slot from the head first (decoded while the frame went on, else now: the tail is linearised over its place), then the header
       from the frame's last words */
    sb = 0;
    if (n != UPD_FRAME_BITS) {
        udc_stream(d, p);                  /* (the rest of its words, or all of them if the main loop had no pass during the frame) */
        sb = p->sb;
        for (i = 0; i < ULTRA_SLOT_BYTES; i++) {
            rx->slot[i] = p->slot[i];
        }
    }
    udc_stream_reset(p);                   /* (this frame's slot is used: the next one starts again, whatever its tag) */
    tail = dline_ch_tail(d, &ts, &tn);
    nb = ws_rise_last(tail, DLINE_CH_TAIL - 1u, ts, tn, rx->hdr);   /* the last 64 bits: the header (a cut first bit is skipped; CRC-8 checks it), read in the snapshot in place */
    dline_ch_free(d);                      /* Both are out: the next frame may be captured (a hub at its 300 us reset starts it 300 us after this one ends). Not before the header
                                              is read: the next capture writes over the snapshot, and after an RW frame the released line's HIGH starts one 150 us after it */
    if (nb < (int)ULTRA_HDR_BITS || ultra_parse_header(rx->hdr, &rx->host) != 0) {
        return UDC_C_NONULTRA;
    }
    rx->flags = ultra_header_flags(rx->hdr);
    rx->slots = (uint8_t)(n == UPD_FRAME_BITS ? 0u : (n - ULTRA_HDR_BITS) / ULTRA_SLOT_BITS);
    rx->cls = UDC_C_HDR;
    if (sb == (int)ULTRA_SLOT_BITS && ultra_slot_check(rx->slot) == 0) {
        rx->cls = UDC_C_SLOT;
    }
    return rx->cls;
}

/* ------------------------------------------------------------------------------------------------ the mode machine */
void udc_init(udc_t *c, uint32_t cfg, uint32_t ms)
{
    uint8_t *p = (uint8_t *)c;
    uint32_t i;
    for (i = 0; i < (uint32_t)sizeof *c; i++) {
        p[i] = 0;
    }
    c->cfg = (uint8_t)cfg;
    c->mode = (uint8_t)(cfg == ULTRA_PXLM_STREAM ? UDC_STREAM : cfg == ULTRA_PXLM_CHAIN ? UDC_CHAIN : UDC_UNDECIDED);
    c->seen_ms = ms;
}

/* the mode changes: the counters that describe the old mode start again */
static uint32_t udc_to(udc_t *c, uint32_t mode, uint32_t act)
{
    c->mode = (uint8_t)mode;
    c->flags = 0;
    c->ok_run = c->bad_run = 0;
    return act;
}

uint32_t udc_frame(udc_t *c, const udc_rx_t *rx, uint32_t ms)
{
    if (rx->cls == UDC_C_NONULTRA) {
        c->nonultra++;
        c->ok_run = 0;
        if (c->bad_run < 255u) {
            c->bad_run++;
        }
        if (c->mode == UDC_CHAIN) {
            c->flags |= ULTRA_PXL_F_NONULTRA;
        }
        return UDC_A_NONE;
    }
    c->frames++;
    c->ok_ms = ms;
    c->slots = rx->slots;
    c->bits = rx->bits;
    if (rx->slots > 1u) {
        c->longs++;
        c->flags |= ULTRA_PXL_F_LONG;
    }
    if (c->ok_run < 255u) {
        c->ok_run++;
    }
    if (c->mode == UDC_UNDECIDED && c->ok_run >= UDC_AUTO_OK) {
        return udc_to(c, UDC_CHAIN, UDC_A_CHAIN);
    }
    return UDC_A_NONE;
}

uint32_t udc_marker(udc_t *c)
{
    if (c->mode == UDC_UNDECIDED) {
        return udc_to(c, UDC_STREAM, UDC_A_STREAM);
    }
    if (c->mode == UDC_CHAIN) {
        c->markers++;
        c->flags |= ULTRA_PXL_F_MARKERS;
    }
    return UDC_A_NONE;
}

uint32_t udc_service(udc_t *c, uint32_t cfg, uint32_t ms, uint32_t activity)
{
    if (cfg != c->cfg) {                 /* CONFIG 140 changed: a forced mode acts at once, AUTO leaves a decided mode */
        c->cfg = (uint8_t)cfg;
        if (cfg == ULTRA_PXLM_STREAM && c->mode != UDC_STREAM) {
            return udc_to(c, UDC_STREAM, UDC_A_STREAM);
        }
        if (cfg == ULTRA_PXLM_CHAIN && c->mode != UDC_CHAIN) {
            return udc_to(c, UDC_CHAIN, UDC_A_CHAIN);
        }
    }
    if (c->mode != UDC_UNDECIDED) {
        return UDC_A_NONE;
    }
    if (activity != c->act_sum) {        /* the line did something since the last look: the silence starts again */
        c->act_sum = activity;
        c->seen_ms = ms;
        if (!c->active) {
            c->active = 1;
            c->act_ms = ms;
        }
    }
    if (c->active && c->bad_run >= UDC_AUTO_BAD && ms - c->act_ms >= UDC_AUTO_ACT_MS) {
        return udc_to(c, UDC_STREAM, UDC_A_STREAM);
    }
    if (!c->quiet && ms - c->seen_ms >= UDC_AUTO_QUIET_MS) {
        c->quiet = 1;                    /* silence is no evidence: the strip is let go, the scanner stays (two Ultra frames still decide) */
        return UDC_A_QUIET;
    }
    return UDC_A_NONE;
}

/* ------------------------------------------------------------------------------------------------ the answer's schedule */
void udc_window(const uint8_t *poll, uint32_t end_w, uint32_t wpu24, uint32_t *tx_w, uint32_t *win_w)
{
    const uint32_t pitch = ultra_rw_slot_us(poll[ULTRA_POLL_RATE]), slots = poll[ULTRA_POLL_SLOTS];
    const uint32_t idx = poll[ULTRA_POLL_INDEX] == ULTRA_POLL_RELAY_ONLY ? 0u : poll[ULTRA_POLL_INDEX];
    const uint32_t tx_us = ULTRA_RW_FIRST_US + pitch * idx;
    const uint32_t win_us = ULTRA_RW_FIRST_US + pitch * slots + ULTRA_RW_HOLDOFF_US;
    *tx_w = end_w + (uint32_t)(((uint64_t)tx_us * wpu24) >> 24);
    *win_w = end_w + (uint32_t)(((uint64_t)win_us * wpu24) >> 24);
}

/* ------------------------------------------------------------------------------------------------ page 0x81 part 3 */
void udc_page(const udc_t *c, uint32_t answers, uint32_t lit, uint32_t ms, uint8_t *d)
{
    uint32_t f = c->flags;
    d[ULTRA_PXL_CONFIG] = c->cfg;
    d[ULTRA_PXL_MODE] = c->mode;
    if (c->mode == UDC_CHAIN && c->frames != 0u && ms - c->ok_ms < UDC_LOCK_MS) {
        f |= ULTRA_PXL_F_LOCKED;
    }
    if (lit) {
        f |= ULTRA_PXL_F_LIT;
    }
    d[ULTRA_PXL_FLAGS] = (uint8_t)f;
    d[ULTRA_PXL_SLOTS] = c->slots;
    ultra_put16(d + ULTRA_PXL_FRAMES, c->frames);
    ultra_put16(d + ULTRA_PXL_NONULTRA, c->nonultra);
    ultra_put16(d + ULTRA_PXL_LONG, c->longs);
    ultra_put16(d + ULTRA_PXL_WINDOWS, c->windows);
    ultra_put16(d + ULTRA_PXL_ANSWERS, answers);
    ultra_put16(d + ULTRA_PXL_MISSED, c->answers_missed);
    d[ULTRA_PXL_BITS] = (uint8_t)(c->bits >> 16);
    d[ULTRA_PXL_BITS + 1u] = (uint8_t)(c->bits >> 8);
    d[ULTRA_PXL_BITS + 2u] = (uint8_t)c->bits;
    ultra_put16(d + ULTRA_PXL_MARKERS, c->markers);
}
