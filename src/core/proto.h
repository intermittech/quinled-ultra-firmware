/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* WS2915 / WS2805 framing, auto-detect and the forwarding-gate plan (wiki: WS modes and config frames).
 *
 * Frame seen by this board (bits):  WS2915: H + 80 n  (H = 32 header bits, V1.3/V1.5, or 26, V1.1)
 *                                   WS2805: 40 m
 * 32 + 80n, 26 + 80n and 40m never coincide, so one frame length identifies protocol and header.
 * Ultra:  256 n + 64 (slots, then the header as a trailer, see ultra_protocol.h). Its lengths do coincide with
 *         WS2805 / WS2915 ones, so Ultra is recognized by the header (magic + version + CRC-8) at the end of the
 *         same frame: for every frame of an Ultra length the last 64 bits are read from the DIN ring at frame end
 *         (proto_tail_wanted) and checked before the frame is classified.
 * Locking in auto mode: a WS protocol after PROTO_LOCK_FRAMES frames in a row; Ultra after one frame whose header
 * and first slot both pass their CRCs (a header without a slot: like WS). "Sticky auto": the protocol locked last
 * is stored and the board starts with it after power-up (provisional: gate and decoder work from the first frame;
 * the first frame of another protocol drops it and the normal lock follows). */
#ifndef PROTO_H
#define PROTO_H

#include <stdint.h>

typedef enum { PROTO_NONE = 0, PROTO_WS2915_32 = 1, PROTO_WS2915_26 = 2, PROTO_WS2805 = 3, PROTO_ULTRA = 4 } proto_t;
typedef enum { PMODE_AUTO = 0, PMODE_WS2915 = 1, PMODE_WS2805 = 2, PMODE_ULTRA = 3 } proto_mode_t;

#define PROTO_LOCK_FRAMES 3u      /* consecutive frames of one WS protocol before (re)locking in auto mode */
#define PROTO_MAX_BITS    256u    /* the longest slot: Ultra (WS2915 112, WS2805 40) */
#define PROTO_DETECT_BITS PROTO_MAX_BITS   /* captured while not locked: an Ultra slot (or a WS slot) */
#define PROTO_UPD_BITS    384u    /* firmware-update frame length (core/upd_app.h, ultra_update.h), tail-checked */

/* Ultra evidence of one frame (proto_frame_end / proto_classify) */
#define PROTO_UH_HEADER   0x01u   /* valid Ultra header (magic, version, CRC-8) at the end of the frame */
#define PROTO_UH_SLOT     0x02u   /* ... and the first slot of the frame passes its CRC-16 */

typedef struct {
    proto_mode_t mode;
    proto_t locked;       /* auto: protocol in use, PROTO_NONE until locked */
    proto_t cand;
    uint8_t cand_count;
    uint8_t forced_hdr;   /* forced WS2915: header length to use (26 / 32), learnt from frame lengths */
    uint8_t provisional;  /* auto: `locked` is the stored protocol from before the power-up, not confirmed yet */
} proto_state_t;

/* Gate (FWD_EN / TIM2 CH2) for the next frame: level at frame start, then toggle at the falling-edge
 * counts t1 and t2 (0 = no toggle). own_bits = bits this board decodes (header + its slot). ws_end: a WS2805 / WS2915
 * plan, whose frame ends (and re-arms) after the shorter idle time (core/frame_track.h FT_END_TICKS; 0: FT_REARM_TICKS) */
typedef struct {
    uint8_t  start_open;
    uint16_t t1, t2;
    uint16_t own_bits;
    uint8_t  hdr_bits;
    uint8_t  ws_end;
} gate_plan_t;

/* decoded slot */
typedef struct {
    uint16_t ch[5];        /* R, G, B, W1, W2 as 16-bit (WS2805 8-bit values * 257) */
    uint8_t  gain[5];      /* WS2915 header gains 0..31 (0 for WS2805) */
    uint8_t  raw[10];      /* slot bytes as received (config frames) */
    uint8_t  raw_len;      /* 10 (WS2915) or 5 (WS2805) */
    uint8_t  hdr_ok;       /* header check bits as expected */
} slot_t;

/* sticky: the protocol locked last (auto mode starts with it, provisionally), PROTO_NONE = none */
void        proto_init(proto_state_t *s, proto_mode_t mode, proto_t sticky);
/* ultra: PROTO_UH_* evidence of the frame */
proto_t     proto_classify(uint32_t total_bits, uint8_t ultra);
/* once per frame; returns the frame's classification */
proto_t     proto_frame_end(proto_state_t *s, uint32_t total_bits, uint8_t ultra);
proto_t     proto_active(const proto_state_t *s);
/* proto_active(s) == PROTO_ULTRA, inline (the main loop asks every pass: a forced Ultra mode, or auto / any other
 * mode with Ultra locked; the WS modes never answer Ultra) */
static inline uint32_t proto_active_ultra(const proto_state_t *s)
{
    return s->mode == PMODE_ULTRA ||
           (s->mode != PMODE_WS2915 && s->mode != PMODE_WS2805 && s->locked == PROTO_ULTRA);
}
/* change the mode at runtime; switching to auto keeps the protocol in use (no gap until a new lock) */
void        proto_set_mode(proto_state_t *s, proto_mode_t mode);
gate_plan_t proto_gate_plan(proto_t p);
/* firmware update session (core/upd_app.h): forward everything, capture the whole 384-bit update frame */
gate_plan_t proto_gate_plan_transparent(void);
/* a frame of this length may end with an Ultra header (Ultra lengths, and the reserved update frame): read it */
int         proto_tail_wanted(uint32_t total_bits);
/* bits: own_bits packed, MSB first, buffer >= PROTO_BITS_BUF bytes (4 bytes slack for the bit reader).
 * WS protocols only; Ultra frames go to ultra.c */
int         proto_parse(proto_t p, const uint8_t *bits, slot_t *out);
/* WS2915 slot from its 80 data bits alone (captured after the header pause, see frame_track.h; gains 0) */
int         proto_parse_data(proto_t p, const uint8_t *bits, slot_t *out);
#define PROTO_BITS_BUF ((PROTO_MAX_BITS + 7u) / 8u + 4u)

#endif
