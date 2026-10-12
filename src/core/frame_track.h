/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Frame tracking of the 20 kHz poll ISR (main.c). A header with an inline function, so the ISR stays one
 * function and the host tests run the same code.
 *
 * Input per tick: c = DIN falling edges since the last gate re-arm (the hardware edge counter).
 *  - A frame starts with its first edge and ends when DIN stays idle for FT_END_TICKS ticks (100-150 us;
 *    WS reset >= 280 us). Frame end: the gate is re-armed (counter back to 0) with the next plan.
 *  - WS2805 and Ultra (v2) frames stay contiguous along a chain: every board blocks its own bits at the start. A
 *    WS2915 frame does not: every board passes the header and then blocks its own 80 bits, so board k sees a gap of
 *    k x 80 bits (100 us each) right after the header. So a gap after exactly 26 or 32 edges (a WS2915 header; no
 *    frame of any protocol is that short) does not end the frame for up to FT_HOLD_TICKS (10 ms = 100 boards), and
 *    the capture of a WS2915 plan (hdr_bits) restarts at the first edges after such a pause (FT_CAP_DATA): the
 *    board decodes only its 80 data bits (the gains are not used), since a capture across the gap would hold a reset
 *    gap (the decoder stops there) and would not fit the ring for long chains.
 *  - Only an armed frame is captured and classified: one whose first edge came after an idle gap, so the
 *    edge count starts at its first bit. A frame end arms the next frame; at boot (and after a flash write)
 *    the tracker also arms once DIN has been idle for FT_BOOT_TICKS (10 ms) without any edge, so the first
 *    complete frame after power-up is used, not skipped. A frame that was already running when the board started
 *    listening is "partial": the gate stays closed for it and it is neither decoded nor classified.
 *    Why 10 ms and not the frame-end time: a board can power up inside a LOW gap of a frame (the WS2915 gap above,
 *    up to FT_HOLD_TICKS), and arming there would take the rest of the frame as a clean frame. Behind 6 boards with
 *    a stale stored WS2915 protocol that rest starts exactly at another board's Ultra slot (32 + 6 x 80 = 2 x 256
 *    bits), and the one-frame Ultra lock would apply it (found by the wire sweep, test_wire.c). Cost: on a line
 *    whose frames are less than 10 ms apart (WS controllers above ~95 fps, a busy Ultra hub) the board arms at the
 *    first frame end instead, so the first frame after power-up is not used.
 *  - FT_EDGE on every tick that saw new edges: the ISR notes the tick and the DIN ring position, so at frame end
 *    it knows when the last bit came (Ultra clock) and where the frame ends in the ring (the Ultra header trailer). */
#ifndef FRAME_TRACK_H
#define FRAME_TRACK_H

#include <stdint.h>
#include "proto.h"

#define FT_END_TICKS  3u               /* DIN idle for 3 ticks (100-150 us) = reset gap */
#define FT_HOLD_TICKS 200u             /* ... after a WS2915 header (26 / 32 edges): 10 ms */
#define FT_BOOT_TICKS FT_HOLD_TICKS    /* boot / flash write: DIN idle this long before the next edge starts a frame */

typedef struct {
    uint16_t last_cnt;
    uint8_t  idle;                     /* ticks without a new edge */
    uint8_t  armed;                    /* the next frame starts cleanly */
    uint8_t  cap_state;                /* 0 wait for a frame, 1 frame running, 2 copy next tick, 3 done */
    uint8_t  partial;                  /* the running frame started before the tracker was armed */
    uint8_t  paused;                   /* the frame paused after its WS2915 header (hdr_bits) */
    uint16_t cap_bits;                 /* capture complete at this edge count */
} ftrack_t;

#define FT_CAP_START  0x01u            /* first edges of an armed frame (0-50 us ago): note the ring position */
#define FT_EDGE       0x02u            /* new edges since the last tick */
#define FT_CAP_COPY   0x04u            /* one tick after our last bit: copy the ring (the DMA has written it) */
#define FT_FRAME_END  0x08u            /* DIN idle: re-arm the gate, classify the frame */
#define FT_CAP_SHORT  0x10u            /* with FRAME_END: the frame was shorter than the capture, copy what came */
#define FT_PARTIAL    0x20u            /* with FRAME_END: the frame started before arming, do not classify it */
#define FT_BOOT_ARM   0x40u            /* idle line, nothing seen yet: re-arm the gate, the next edge starts a frame */
#define FT_CAP_DATA   0x80u            /* with CAP_START: the capture restarts after the WS2915 header (hdr_bits) */

/* own_bits: this board's bits under the plan of the frame in progress (the capture from the frame start);
 * hdr_bits: its WS2915 header length (0 for other plans), see above */
static inline uint32_t ftrack_tick(ftrack_t *t, uint16_t c, uint16_t own_bits, uint16_t hdr_bits)
{
    uint32_t ev = 0;
    if (t->cap_state == 2u) {
        t->cap_state = 3;
        ev = FT_CAP_COPY;
    }
    if (c != t->last_cnt) {
        ev |= FT_EDGE;
        if (t->last_cnt == 0u) {                         /* first edges of a frame */
            t->partial = (uint8_t)!t->armed;
            t->paused = 0;
            if (t->cap_state == 0u && t->armed) {
                t->cap_bits = own_bits;
                t->cap_state = 1;
                ev |= FT_CAP_START;
            }
        } else if (t->paused && t->cap_state == 1u) {    /* data after the WS2915 header pause: capture it */
            t->cap_bits = (uint16_t)(hdr_bits + 80u);
            ev |= FT_CAP_START | FT_CAP_DATA;
        }
        t->paused = 0;
        if (t->cap_state == 1u && c >= t->cap_bits) {
            t->cap_state = 2;
        }
        t->last_cnt = c;
        t->idle = 0;
    } else if (c != 0u) {
        uint8_t hdr = (uint8_t)(c == 26u || c == 32u);
        if (hdr && c == hdr_bits) {
            t->paused = 1;                               /* the boards before us block their bits now */
        }
        if (++t->idle >= (hdr ? FT_HOLD_TICKS : FT_END_TICKS)) {
            ev |= FT_FRAME_END;
            if (t->cap_state == 1u) {                    /* frame shorter than the capture: decode what came */
                t->cap_bits = c;
                ev |= FT_CAP_SHORT;
            }
            if (t->partial) {
                ev |= FT_PARTIAL;
            }
            t->cap_state = 0;
            t->armed = 1;
            t->partial = 0;
            t->last_cnt = 0;
            t->idle = 0;
        }
    } else if (!t->armed && ++t->idle >= FT_BOOT_TICKS) {
        t->armed = 1;                                    /* boot: DIN quiet, the next edge is a frame's first */
        t->idle = 0;
        ev |= FT_BOOT_ARM;
    }
    return ev;
}

/* Samples and edge counts are unreliable (flash write: interrupts off, SPI at half clock): drop a capture in
 * progress and treat the running frame as partial; the next idle gap arms again. Call with interrupts off. */
static inline void ftrack_disarm(ftrack_t *t)
{
    t->armed = 0;
    t->cap_state = 0;
    t->partial = 1;
}

#endif
