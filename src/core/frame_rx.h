/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Per-frame decisions of the receive path, without hardware: main.c calls them with the decoded capture and the
 * decoded end of each frame, and the wire-level chain test (test/test_wire.c) runs the same code.
 *
 * A board sees one contiguous frame (proto.h). It decodes the start of it (its own bits under the frame's gate
 * plan: a WS slot, or an Ultra slot), and for frames of an Ultra length the last 64 bits (the Ultra header
 * trailer). Then:
 *  - WS slot, protocol locked: shown at once (frx_capture -> FRX_SHOW_WS), if the protocol it was captured under
 *    is still the active one (the frame before may have changed it while this one ran).
 *  - WS slot, protocol only stored (provisional): waits for the frame end, shown only if the frame's length
 *    confirms it and no Ultra header ends the frame.
 *  - Ultra slot (CRC-16 ok): kept until the header at the frame end gives the host time; applied when the frame is
 *    Ultra and the board is (now) locked to Ultra - also on the frame that locks it.
 *  - Header FLAGS: RW (return channel) frames give clock sync only, UPD (firmware update) frames nothing; both are
 *    reserved and not implemented here. */
#ifndef FRAME_RX_H
#define FRAME_RX_H

#include <stdint.h>
#include "proto.h"
#include "ultra_protocol.h"

typedef struct {
    proto_state_t ps;
    uint32_t cap_frame;                /* frame id of the capture below */
    uint8_t  have_ultra;               /* slot[]: a CRC-valid Ultra slot from the start of cap_frame */
    uint8_t  have_ws;                  /* ws: a WS slot of cap_frame waiting for its frame end (provisional) */
    uint8_t  ws_proto;
    uint8_t  slot[ULTRA_SLOT_BYTES];
    slot_t   ws;
} frx_t;

#define FRX_SHOW_WS 0x01u              /* frx_capture: show the WS slot now */

typedef struct {
    proto_t  before, after;            /* active protocol before / after this frame */
    proto_t  cls;                      /* the frame's classification */
    uint8_t  close_gate;               /* the stored protocol was wrong: close the gate now, not one frame later */
    uint8_t  show_ws;                  /* show frx.ws (protocol frx.ws_proto) */
    uint8_t  ultra_header;             /* a valid Ultra header ended the frame: host_us, flags */
    uint8_t  ultra_sync;               /* use it: clock sync (Ultra active, not an UPD frame) */
    uint8_t  ultra_slot;               /* ... and apply frx.slot (this frame's own slot) after it (not in RW / EXT frames) */
    uint8_t  flags;
    uint32_t host_us;                  /* host clock at the end of the frame */
    gate_plan_t plan;                  /* gate plan for the following frames */
} frx_end_t;

void     frx_init(frx_t *r, proto_mode_t mode, proto_t sticky);
/* the start of frame `frame` decoded: nbits bits (the capture), p = protocol active when it started, own = own
 * bits under its gate plan; skip > 0: the capture starts after a WS2915 header of skip bits (FT_CAP_DATA), so it
 * holds the 80 data bits only. Fills *ws for FRX_SHOW_WS. */
uint32_t frx_capture(frx_t *r, uint32_t frame, proto_t p, const uint8_t *bits, uint32_t nbits, uint32_t own,
                     uint32_t skip, slot_t *ws);
/* read the last 64 bits of this frame? Ultra lengths, and every frame while auto mode is not locked yet or only
 * provisionally (a frame that ends with an Ultra header is never taken as WS, whatever its length) */
int      frx_tail_wanted(const frx_t *r, uint32_t total);
/* frame end: total = its bits, tail8 = its last 64 bits (NULL: not read / not decodable) */
void     frx_frame_end(frx_t *r, uint32_t frame, uint32_t total, const uint8_t *tail8, frx_end_t *e);

#endif
