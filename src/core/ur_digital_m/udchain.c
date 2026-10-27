/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "../math32.h"
/* The Diff-Power-UR-Digital-M's CHAIN line mode: see udchain.h. */
#include <stddef.h>
#include "udchain.h"
#include "../ws_decode.h"
#include "../ultra_update.h"
#include "udpack.h"
#include "../link_stats.h"
#include "../rc.h"

#if defined(UR_DIGITAL_M_IMAGE)
/* One implementation shared by effect, calibration and chain pages. */
NOINLINE void ud_put16(uint8_t *d, uint32_t v)
{
    ultra_put16(d, v);
}
NOINLINE int ud_parse_header(const uint8_t *h, uint32_t *time_us)
{
    const uint32_t t = ultra_get32(h + 3);
    if (ultra_get16(h) != ULTRA_MAGIC || (h[2] >> 4) != ULTRA_VERSION || crc8_smbus(h, 7) != h[7] || t >= ULTRA_TIME_WRAP) {
        return -1;                                       /* (TIME at or over its wrap: invalid, as ultra_parse_header) */
    }
    *time_us = t;
    return 0;
}
NOINLINE void ud_put16s(uint8_t *d, const void *a, const void *b, const ud_f16_t *t, uint32_t n)
{
    for (; n != 0u; n--, t++) {
        const uint8_t *s = (const uint8_t *)((t->dst & UD_F16_B) ? b : a) + t->src;
        ud_put16(d + (t->dst & 0x7Fu), *(const uint16_t *)(const void *)s);
    }
}
#endif

/* dline's late limit for an armed answer is the protocol's: ULTRA_RW_LATE_US in ring words (2.25 a us) */
typedef char udc_late_is_the_protocols[DLINE_W_CH_LATE == MCU_US2W(ULTRA_RW_LATE_US) ? 1 : -1];

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
    uint32_t hn, consumed, start = d->ch_cjs - 1u, mask = DLINE_CH_TAIL - 1u;
    if (p->js != d->ch_cjs) {              /* a new frame in the capture: start again */
        udc_stream_reset(p);
        p->js = d->ch_cjs;
        ws_line_init(&p->acquisition);
    }
    if (p->sb != 0) {
        return;
    }
    hn = *(volatile uint16_t *)&d->ch_hn;
#if defined(__GNUC__)
    __asm__ volatile ("" ::: "memory");
#endif
    consumed = d->ch_profile >= 4u ? p->acquisition.pos >> 4 : p->st.k;
    if (hn == consumed) return;
    if (hn - consumed > DLINE_CH_TAIL) {
        p->sb = WS_ERR_TRUNCATED;
        p->acquisition.bad = 1;
        return;
    }
    if (d->ch_profile >= 4u) {
        uint32_t n = hn - consumed, first = DLINE_CH_TAIL - ((start + consumed) & mask);
        if (first > n) first = n;
        ws_line_feed(&p->acquisition,d->cap+DLINE_CH_TAIL,d->cap+((start+consumed)&mask),first);
        if (n != first) ws_line_feed(&p->acquisition,d->cap+DLINE_CH_TAIL,d->cap,n-first);
    } else {
        p->sb = (int16_t)ws_rise_ring(&p->st,d->cap,mask,start,hn,UPD_BLOCK_BYTES * 8u,
                                     ws_line_threshold(d->ch_profile),p->slot);
    }
}

uint32_t udc_classify(dline_t *d, udc_rx_t *rx, udc_pre_t *p, uint32_t ms)
{
    const uint32_t n = d->ch_bits, acquiring = d->ch_profile >= 4u;
    const uint32_t drops = d->st.cap_drops, want = n == UPD_FRAME_BITS ? UPD_BLOCK_BYTES * 8u : ULTRA_SLOT_BITS;
    uint32_t ts, tn, i;
    const uint16_t *tail;
    int nb, sb;

    rx->cls = UDC_C_NONULTRA;
    rx->flags = 0;
    rx->slots = 0;
    rx->host = 0;
    rx->bits = n;
    for (i = 0; i < UPD_BLOCK_BYTES; i++) rx->slot[i] = 0;
    udc_stream(d,p);
    rx->profile = acquiring ? p->acquisition.profile : d->ch_profile;
    /* Acquisition runs and state belong exclusively to the main loop, beyond
     * the ISR's raw snapshot. Release that snapshot before decoding the runs
     * and aggregating widths so the next frame can be captured meanwhile. */
    if (acquiring) dline_ch_free(d);
    /* the length: 256 x n + 64 bits, or an UPD frame's (384: it carries the update, not a slot) */
    if (n < ULTRA_HDR_BITS || n > ULTRA_HDR_BITS + ULTRA_SLOT_BITS * UDC_LONG_SLOTS ||
        (((n - ULTRA_HDR_BITS) & (ULTRA_SLOT_BITS - 1u)) != 0u && n != UPD_FRAME_BITS)) {
        goto done;
    }
    /* the slot from the head first (decoded while the frame went on, else now: the tail is linearised over its place), then the header
       from the frame's last words */
    if (acquiring) {
        sb = ws_line_decode(&p->acquisition,d->cap+DLINE_CH_TAIL,want,rx->slot,0);
        nb = ws_line_decode(&p->acquisition,d->cap+DLINE_CH_TAIL,ULTRA_HDR_BITS,rx->hdr,1);
        rx->profile = p->acquisition.profile;
        if (p->acquisition.bits != n) nb = -1;
    } else {
        sb = p->sb < 0 ? p->sb : p->st.b;
        for (i = 0; i < UPD_BLOCK_BYTES; i++) {
            rx->slot[i] = p->slot[i];
        }
        tail = dline_ch_tail(d, &ts, &tn);
        /* End at eight LOW words after the last pulse; include only the
         * trailing 64 bits plus one period of clock/word-phase allowance. */
        uint32_t periods = (ws_line_threshold(rx->profile) * 2u * 67u + 15u) / 16u + 8u;
        if (tn > periods) { ts = (ts + tn - periods) & (DLINE_CH_TAIL-1u); tn = periods; }
        nb = ws_rise_tail(tail,DLINE_CH_TAIL-1u,ts,tn,ws_line_threshold(rx->profile),rx->hdr);
    }
    if (nb < (int)ULTRA_HDR_BITS || ud_parse_header(rx->hdr, &rx->host) != 0) {
        goto done;
    }
    rx->flags = ultra_header_flags(rx->hdr);
    rx->slots = (uint8_t)(n == UPD_FRAME_BITS ? 0u : (n - ULTRA_HDR_BITS) / ULTRA_SLOT_BITS);
    rx->cls = UDC_C_HDR;
    if (n == UPD_FRAME_BITS && sb >= (int)want && (rx->flags & ULTRA_HDR_UPD)) {
        rx->cls = UDC_C_UPDATE;
    } else if (sb >= (int)ULTRA_SLOT_BITS && ud_slot_check(rx->slot) == 0) {
        rx->cls = UDC_C_SLOT;
    }
done:
    if (!acquiring) dline_ch_free(d);
    ls_frame(acquiring ? &p->acquisition : 0,d->cap+DLINE_CH_TAIL,
             n==ULTRA_HDR_BITS?0:rx->slot,(rx->cls!=UDC_C_NONULTRA?1u:0u)|(rx->cls==UDC_C_SLOT?2u:0u),ms,drops);
    udc_stream_reset(p);
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
static NOINLINE uint32_t udc_to(udc_t *c, uint32_t mode, uint32_t act)
{
    c->mode = (uint8_t)mode;
    c->flags = 0;
    c->ok_run = c->bad_run = 0;
    return act;
}

NOINLINE uint32_t udc_frame(udc_t *c, const udc_rx_t *rx, uint32_t ms)
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
    if (c->mode == UDC_UNDECIDED && c->ok_run >= (rx->bits==ULTRA_HDR_BITS?3u:UDC_AUTO_OK)) {
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
    if (cfg != c->cfg) {                 /* CONFIG 75 changed: a forced mode acts at once, AUTO leaves a decided mode */
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
void udc_window(const uint8_t *poll, uint32_t end_w, uint32_t wpu24, uint32_t *tx_w, uint32_t *win_w, uint32_t *dir_w, uint32_t *fwd_w)
{
    const uint32_t pitch = ultra_rw_slot_us(poll[ULTRA_POLL_RATE]);
    const uint32_t idx = poll[ULTRA_POLL_INDEX] == ULTRA_POLL_RELAY_ONLY ? 0u : poll[ULTRA_POLL_INDEX];
    const uint32_t w_us = ULTRA_RW_FIRST_US + pitch * poll[ULTRA_POLL_SLOTS];   /* W: the window's last slot ends */
    *tx_w = end_w + m32_shr(ULTRA_RW_FIRST_US + pitch * idx,wpu24,24);
    *win_w = end_w + m32_shr(w_us + ULTRA_RW_HOLDOFF_US,wpu24,24);
    *dir_w = end_w + m32_shr(RC_DIR_US,wpu24,24);
    *fwd_w = end_w + m32_shr(w_us + RC_END_US,wpu24,24);
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
    static const ud_f16_t t[6] = { { ULTRA_PXL_FRAMES, offsetof(udc_t, frames) }, { ULTRA_PXL_NONULTRA, offsetof(udc_t, nonultra) },
        { ULTRA_PXL_LONG, offsetof(udc_t, longs) }, { ULTRA_PXL_WINDOWS, offsetof(udc_t, windows) },
        { ULTRA_PXL_MISSED, offsetof(udc_t, answers_missed) }, { ULTRA_PXL_MARKERS, offsetof(udc_t, markers) } };
    d[ULTRA_PXL_FLAGS] = (uint8_t)f;
    d[ULTRA_PXL_SLOTS] = c->slots;
    ud_put16s(d, c, c, t, 6);
    ud_put16(d + ULTRA_PXL_ANSWERS, answers);
    d[ULTRA_PXL_BITS] = (uint8_t)(c->bits >> 16);
    d[ULTRA_PXL_BITS + 1u] = (uint8_t)(c->bits >> 8);
    d[ULTRA_PXL_BITS + 2u] = (uint8_t)c->bits;
    ud_put16(d + ULTRA_PXL_BUSY, c->relay_busy);
}
