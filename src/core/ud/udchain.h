/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The Ultra-Digital receiver's CHAIN line mode (BOARD 3; docs/ultradigital-firmware.md section 12): the board on a normal Ultra line, the
 * last board of a chain, next to Ultra-Analog receivers. This file is the pure logic, host-tested (test/test_udchain.c); main_ud.c is
 * the glue, core/dline.c the raw frame capture (dl_ch_scan), core/ud/udfx.c the encoder that owns the ring.
 *
 *  - classify: the captured head and tail of a chain frame -> the slot (the first 256 bits) and the header (the last 64), checked
 *    against the frame's bits (TIM4): 256 x n + 64 (n >= 1; an UPD frame, 384, is recognised, not captured whole).
 *  - the line mode machine: CONFIG 140 AUTO / STREAM / CHAIN. AUTO decides once per power-up, in RAM, with the strip held dark:
 *    two Ultra frames in a row (a right header and length) are CHAIN; a marker or UDC_AUTO_BAD frames that are no Ultra frame over
 *    UDC_AUTO_ACT_MS of activity are STREAM. UDC_AUTO_QUIET_MS of silence (from the line's last activity: a long HIGH of an unplugged cable counts) decide nothing (silence is no evidence: a hub may start
 *    late): the hold ends, the board's own effect may run, the chain scanner goes on and the same evidence decides. A forced mode needs
 *    no wait. A change of CONFIG 140 acts at once (AUTO leaves a decided mode as it is).
 *  - the answer: where a POLL's answer starts, in ring words after the frame's last edge (t0): the Ultra window's slot time, scaled by
 *    the clock's rate (udfx), no release to wait for.
 *  - page 0x81 part 3 (ULTRA_PXL_*) and the counters. */
#ifndef UDCHAIN_H
#define UDCHAIN_H

#include <stdint.h>
#include "../dline.h"
#include "../ws_decode.h"
#include "../ultra_protocol.h"

#define UDC_AUTO_OK        2u          /* Ultra frames in a row that decide CHAIN */
#define UDC_AUTO_BAD       4u          /* frames that are no Ultra frame ... */
#define UDC_AUTO_ACT_MS    500u        /* ... over this long of activity decide STREAM */
#define UDC_AUTO_QUIET_MS  2000u       /* no activity at all this long (since power-up, or since the line's last activity): the hold ends (UDC_A_QUIET), the mode stays open */
#define UDC_LOCK_MS        2000u       /* LOCKED: a right Ultra frame within this long */
#define UDC_LONG_SLOTS     64u         /* a frame of more slots is no Ultra frame */

enum { UDC_UNDECIDED = 0, UDC_STREAM = 1, UDC_CHAIN = 2 };   /* the mode in effect (ULTRA_PXL_MODE) */
enum { UDC_A_NONE = 0, UDC_A_CHAIN, UDC_A_STREAM, UDC_A_QUIET };   /* what the glue does (QUIET: end the strip's hold, nothing else) */
enum { UDC_C_NONULTRA = 0, UDC_C_HDR, UDC_C_SLOT };         /* a frame: nothing right / header and length right (the slot is not:
                                                               a DARK slot, a CRC error) / and a CRC-good slot */

typedef struct {
    uint8_t  mode;                     /* UDC_* */
    uint8_t  cfg;                      /* CONFIG 140 as last seen */
    uint8_t  ok_run;                   /* AUTO: Ultra frames in a row */
    uint8_t  bad_run;                  /* AUTO: frames that were no Ultra frame since the activity began */
    uint8_t  flags;                    /* ULTRA_PXL_F_NONULTRA / _LONG / _MARKERS (cleared by a mode change) */
    uint8_t  slots;                    /* the last Ultra frame's slots */
    uint8_t  active;                   /* AUTO: the line has shown activity */
    uint8_t  quiet;                    /* AUTO: UDC_AUTO_QUIET_MS passed without activity (UDC_A_QUIET given; the mode is still open) */
    uint16_t frames, nonultra, longs;  /* Ultra frames, frames that are none, Ultra frames of more than one slot (wrap) */
    uint16_t windows, answers_missed;  /* RW windows this board took part in (wraps), POLLs it could not answer in time */
    uint16_t markers;
    uint32_t bits;                     /* the last Ultra frame's bits */
    uint32_t seen_ms, act_ms, ok_ms;   /* the last activity of the line (power-up until there is one), the first activity, the last right frame */
    uint32_t act_sum;                  /* the activity counters' sum at the last look */
} udc_t;

/* the slot of the frame in progress, decoded as the head's words arrive (udc_stream, from the main loop's passes: the 50 us ticks leave it 2-5 k
 * cycles each, a 23 k-cycle decode started at the frame's end would hold the capture past a hub's 300 us reset) */
typedef struct {
    uint32_t js;                       /* the frame (dline's ch_cjs) the slot is from */
    ws_rise_t st;                      /* the decode so far */
    int16_t  sb;                       /* ULTRA_SLOT_BITS when all are out (else an error or 0) */
    uint8_t  slot[ULTRA_SLOT_BYTES];
} udc_pre_t;

typedef struct {
    uint8_t  cls;                      /* UDC_C_* */
    uint8_t  flags;                    /* the header's FLAGS */
    uint8_t  slots;                    /* (bits - 64) / 256 (0 for an UPD frame) */
    uint32_t bits;                     /* the frame bits (TIM4) */
    uint32_t host;                     /* the header's TIME */
    uint8_t  slot[ULTRA_SLOT_BYTES];
    uint8_t  hdr[ULTRA_HDR_BYTES];
} udc_rx_t;

/* power-up: cfg = CONFIG 140 (the record's); ms the controller's ms. The glue then starts what the mode needs (udc_start) */
void     udc_init(udc_t *c, uint32_t cfg, uint32_t ms);
/* the frame being captured: decode what is new of its head. Cheap when nothing is (a few instructions); the frame's end finishes it (udc_classify) */
void     udc_stream(dline_t *d, udc_pre_t *p);
/* the capture of dline's handed-over frame (dline_ch_head / _tail, ch_bits) -> rx; p: the slot decoded as the words arrived (udc_stream), else it is
 * decoded here. Returns rx->cls. Frees the capture (dline_ch_free) once the slot and the header are out: the header is read in the snapshot in place, before the free */
uint32_t udc_classify(dline_t *d, udc_rx_t *rx, udc_pre_t *p);
/* a frame classified: counters, the decision. Returns UDC_A_* */
uint32_t udc_frame(udc_t *c, const udc_rx_t *rx, uint32_t ms);
/* a marker was seen (DLINE_A_CH_MARKER) */
uint32_t udc_marker(udc_t *c);
/* once a ms: AUTO's time limits and a changed CONFIG 140 (cfg); activity = the sum of the line's counters (frames, markers, long HIGHs: it only has to
 * change when the line does something). UDC_AUTO_QUIET_MS are counted from the last change */
uint32_t udc_service(udc_t *c, uint32_t cfg, uint32_t ms, uint32_t activity);
/* a valid POLL slot (this board: index != RELAY_ONLY answers) of a frame whose last edge is ring word end_w: the answer's start word and the
 * window's end. wpu24 = the clock's words per host us (Q24, udfx). The caller answers only with a measured rate (udfx r_have): an unmeasured
 * HSI (+-2.2 %) shifts the start by up to +-25 us on top of the tick's 0 .. +50, more than the window's -5 .. +80 holds; the POLL is missed
 * (counted), the hub asks again */
void     udc_window(const uint8_t *poll, uint32_t end_w, uint32_t wpu24, uint32_t *tx_w, uint32_t *win_w);
/* page 0x81 part 3 into d (24 bytes, zero before) */
void     udc_page(const udc_t *c, uint32_t answers, uint32_t lit, uint32_t ms, uint8_t *d);

#endif
