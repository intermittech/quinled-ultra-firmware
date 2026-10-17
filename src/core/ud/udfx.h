/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The Ultra-Digital receiver's local pixel effects (BOARD 3; docs/ultradigital-firmware.md section 11): the controller
 * between the protocol (PIXEL_FX, CONFIG 129-139, page 0x81), the effect engine (core/fx.h, docs/ultradigital-effects.md:
 * segments, transitions, the effects, the packer), the symbol encoder (wsenc.h) and the delay line (dline.h). Pure logic,
 * host-tested (test/test_udfx.c on a word-exact model of the ring); main_ud.c is the glue.
 *
 * Contexts: udfx_tick (the 20 kHz tick, after dline_tick) and udfx_ram_tick (the flash operation's SRAM tick) own the
 * encoder, the mode switches and the dline `local` flag; the main loop owns the configuration, the segments, the
 * renderer and the clock. They meet in a few single-word fields (marked): the main loop writes them only when the
 * tick's use of them is over (a request flag the tick clears), the tick never waits.
 *
 * The modes (page 0x81 ULTRA_PXM_*): STREAM, the hub's pixels pass; LOCAL, the encoder owns every ring word from enc.w on.
 * A switch happens only at a latch of the incoming line, the strip chip's own (latch_w: the least reset of its family, the
 * HSI 2.2 % fast; a controller's frame can hold a shorter LOW, an RMT refill). A stream whose gaps never reach it (a hub at the
 * datasheet's exact reset, SK6812 80 us) would never get the switch: after UX_LATCH_WAIT_W (100 ms) of waiting the switch
 * takes dline's latch (46 us) instead, at the risk of one cut frame. STREAM -> LOCAL when the line has been LOW
 * for a latch (dline_low_run >= latch_w) and no poll window is open; LOCAL -> STREAM when the encoder finished its frame and
 * wrote a full reset (nothing of the strip's frame is cut) while the incoming line has been LOW for a latch. The strip
 * therefore gets whole frames only: the hub's last one, then the receiver's own, or the other way round. Polls run in
 * both (the scan reads every RX word before the encoder replaces it).
 *
 * Time: every marker frame's header TIME (host us at its last edge) and dline's end_w (the ring word of that edge) are an
 * anchor of the host clock against the ring's word count (2.25 words per host us, the HSI's rate measured from anchors
 * >= 1 s apart); the mapping slews (0.5 % of the time since the last anchor, a step only past 50 ms). Effects render at
 * the host time of the grid point their frame starts at (frame k of period P at host time k x P), so receivers on one
 * hub render the same picture at the same host time. The engine counts host ms: ux_ms() turns the host us into a
 * continuous ms count (no jump at the 32-bit us wrap), the same differences on every receiver. */
#ifndef UDFX_H
#define UDFX_H

#include <stdint.h>
#include "../dline.h"
#include "../ultra_protocol.h"
#include "wsenc.h"
#include "fx.h"                         /* the effect engine: src/core/fx.h */

#define UX_SEGS        ULTRA_PX_SEGMENTS
#define UX_POOL_BYTES  2000u            /* complete wire frame: 600 RGB / 500 RGBW / 400 five-channel pixels */
#define UX_STATE_BYTES (FX_MAX_SEG * FX_FIRE_CELLS)   /* the engine's state pool (Fire's heat: 256 B a Fire segment) */
#define UX_CHUNK       FX_CHUNK         /* pixels rendered per step (<= ~55 us) */
#define UX_CHUNK_HEAVY 4u              /* Pacifica: ~6,000 cycles (8,000 at most); 2 px on the 400 kHz encoder */
#define UX_RING_WORDS  (DLINE_RING_WORDS + WS_GUARD)
#define UX_RATE_NOM    7456540u         /* host us per ring word, Q24 (36 MHz / 16 = 2.25 words per us) */
#define UX_REC_BYTES   128u             /* the stored record (2 a page) */
#define UX_REC_MAGIC   0xD5A1u
#define UX_REC_FORMAT  3u               /* the record's format: 3 keeps CONFIG 140 and 141 (two small values) in the last of its twelve u16 slots */
#define UX_REC_SLOTS   12u              /* u16 slots for CONFIG 129 .. 140; CONFIG 141 shares the last one (high byte) */
#define UX_REC_SEG0    (8u + 2u * UX_REC_SLOTS)          /* the first segment's offset in the record: 32 */
#define UX_REC_MASK    (UX_REC_SEG0 + 23u * UX_SEGS)     /* the stored mask: 124, then a spare count, then the CRC-16 */
#define UX_STEP_US     50000u           /* the clock steps past this error, else slews */
#define UX_POWER_ON_MS 2000u            /* ULTRA_PXP_POWER_ON: the quiet line before the stored effect fades in */
#define UX_FADE_IN_MS  1000u            /* ... the least fade-in of a stored effect */
#define UX_LATCH_WAIT_W 225000u         /* 100 ms: a switch that found no LOW of the chip's latch takes dline's (46 us) after this */
#define UX_CATALOGUE_VERSION 3u         /* = fx_catalogue.json's catalogue_version */

typedef struct {                        /* one segment */
    ultra_pixel_fx_t p;                 /* the PIXEL_FX in effect */
    ultra_pixel_fx_t pend;              /* next frame's command; start_ms holds its absolute engine anchor */
    uint32_t pend_at;                   /* host us the pending one applies at */
    uint32_t anchor;                    /* the effect's time zero, host ms (ux_ms) */
    uint8_t  run;                       /* an effect runs */
    uint8_t  restarts;                  /* ULTRA_PXF_RESTART slots applied (mixed into the seed: the random effects start anew) */
    uint8_t  pend_have;                 /* 1 immediate, 2 scheduled (pend_at) */
    uint8_t  apply;                     /* frozen at frame start: 1 command, 2 virtual-strip reapply */
} ux_seg_t;

typedef struct udfx_s {
    /* ---- configuration (CONFIG 129-141), persisted */
    uint16_t cfg[ULTRA_CFG_PX_COUNT];
    /* derived by ux_derive */
    ws_chip_t chip;
    ws_lut_t  lut;
    fx_pack_cfg_t pack;                 /* gamma, bytes per pixel, wire order, white mode and split */
    uint16_t  npx, max_px;
    uint32_t  latch_w;                  /* LOW words after which the strip has surely latched (the switches wait for them) */
    uint32_t  period_w;                 /* the frame grid, words (0: back to back) */
    uint32_t  period_us;
    /* ---- the stored power-on effect (the record's segments) */
    ultra_pixel_fx_t stored[UX_SEGS];
    uint8_t   stored_have;
    uint8_t   save_due;                 /* the record needs writing (config or stored effect changed) */
    uint8_t   white_def;                /* the strip's auto-white mode (CONFIG 135) */
    uint8_t   wait_on;                  /* tick: a switch is waiting at wait_w (bytes grouped to avoid padding) */
    uint32_t  save_ms;                  /* ... since this ms of the controller's clock (the glue debounces) */
    uint32_t  rec_seq;
    /* ---- the mode */
    volatile uint8_t local;             /* LOCAL (tick context: set and cleared at a latch) */
    uint8_t   standalone;               /* LOCAL started by the stored effect's policy */
    volatile uint8_t want_local;        /* main loop -> tick: switch in at the next latch (the tick clears it) */
    volatile uint8_t want_stream;       /* ... out */
    volatile uint32_t first_w;          /* the first frame's start word (grid-aligned), refreshed by the main loop */
    uint32_t  wait_w;                   /* tick: the ring word a pending switch began to wait at (wait_on) */
    volatile int32_t nudge;             /* main loop -> tick: add to enc.next_w at the next tick (the tick clears it) */
    uint8_t   lock;                     /* the running effect is locked against a plain stream */
    uint8_t   dark;                     /* LOCAL, every segment dark and its black frame sent: the line is LOW */
    uint8_t   black_sent, frame_dark;
    uint32_t  dark_at;
    uint8_t   reconf;                   /* the chip / pixels / order / fps changed: leave and re-enter local (CHAIN: pause the encoder) */
    uint8_t   rebegin;                  /* the virtual strip changed: every running effect starts again */
    uint8_t   chunk;                    /* frame-frozen render quota (Pacifica: 4) */
    /* ---- CHAIN line mode (udfx_chain; docs/ultradigital-firmware.md section 12): the encoder owns the ring for good */
    uint8_t   chain;                    /* CHAIN: never leaves local, no stream to hand back to, STREAM_WINS off */
    uint8_t   undec;                    /* AUTO, the line mode not decided yet: STREAM_WINS waits for the mode (two Ultra frames on a chain line
                                           would look like a stream taking the strip) */
    volatile uint8_t pause_req;         /* main loop -> tick: stop starting frames (the tables are about to change) */
    volatile uint8_t paused;            /* tick -> main loop: the encoder is idle in LOW, the ring zeroed by it; derive now */
    uint8_t   seq5[UX_SEGS + 1u];       /* the SEQ of each segment's last PIXEL_FX (the fifth: the whole strip) */
    uint8_t   seq_have;
    uint32_t  frames_seen;
    /* ---- the encoder and the renderer */
    ws_enc_t  e;
    uint32_t  rendered;                 /* pixels of it rendered */
    uint32_t  fsum;                     /* its channel sum (the limiter) */
    uint32_t  frames_done;
    uint8_t   lim_bri;                  /* the limiter's brightness for the packer, 255 = none */
    uint8_t   begun;                    /* the next frame's render has started */
    uint16_t  cl_q8;                    /* the closed-loop trim of the limit, 8..256 */
    uint16_t  meas_ma;                  /* the measured bus current (the glue's) */
    uint16_t  vk_q8;                    /* bus-side mA per strip-side mA, Q8 (the glue's: V_B / V_bus / efficiency) */
    uint8_t   load;                     /* CPU load, 0.5 % steps (0..200; the glue's) */
    uint8_t   stream_pending;           /* main: cancel the scene at the next frame boundary */
    uint16_t  fps10;
    uint32_t  fps_frames, fps_ms;
    uint32_t  ms;                       /* the controller's own ms clock (udfx_service) */
    /* ---- the clock */
    uint32_t  a_host, a_w;              /* the anchor: host us at ring word a_w */
    uint32_t  r_host, r_w;              /* the rate reference */
    uint32_t  wq24;                     /* host us per word, Q24 */
    uint32_t  wpu24;                    /* words per host us, Q24 */
    uint32_t  now_w;                    /* the newest ring word the main loop saw (dline.wpos) */
    uint32_t  hus_ref, hms_ref;         /* ux_ms()'s reference: host us (a multiple of 1000) and its ms count */
    /* ---- the tick's stats */
    uint16_t  switches;
    uint8_t   a_have, r_have;           /* the clock has an anchor / measured rate */
    /* ---- segments, and the engine */
    ux_seg_t  seg[UX_SEGS];
    fx_engine_t eng;
} udfx_t;

#if defined(__GNUC__)
#define UX_AINLINE static inline __attribute__((always_inline))
#else
#define UX_AINLINE static __inline
#endif

extern uint8_t ux_pool[UX_POOL_BYTES];

/* the poll answer's SEND: BLANK_N LOW with DE (the hub's stream passes through the ring, the window blanks it) or, while a
 * local effect runs, DE only: the effect owns the ring, dline never asks for BLANK_N there and so never releases it */
UX_AINLINE uint32_t udfx_answer_blank(const udfx_t *u)
{
    return !u->local;
}

/* the line mode (CONFIG 140) as stored: ULTRA_PXLM_* */
UX_AINLINE uint32_t udfx_line_cfg(const udfx_t *u)
{
    return u->cfg[ULTRA_CFG_PX_LINE_MODE - ULTRA_CFG_PX_FIRST];
}

/* ---- main loop */
void     udfx_init(udfx_t *u);                              /* defaults; no record yet */
/* The line mode (main loop): 1 CHAIN: the encoder takes the ring (dline_ch_start has zeroed it, or at a latch of the incoming line) and
 * never gives it back; no effect running = a dark strip. 0 STREAM: back to the stream at a latch (the usual switch). 2 AUTO, not
 * decided yet: as STREAM, but STREAM_WINS does not hand the strip back until the mode is known */
void     udfx_chain(udfx_t *u, uint32_t mode);
void     udfx_load(udfx_t *u, const uint8_t *rec);          /* a valid stored record (udfx_rec_valid) */
/* the clock: a marker frame's header TIME and the ring word of its last edge */
void     udfx_anchor(udfx_t *u, uint32_t host_us, uint32_t w);
uint32_t udfx_host_us(const udfx_t *u, uint32_t w);         /* host us at ring word w */
uint32_t udfx_word_at(const udfx_t *u, uint32_t host_us);   /* ring word at host us */
/* CONFIG id 129..141 (other ids: ignored). Returns 1 when the value was taken and changed */
uint32_t udfx_config(udfx_t *u, uint32_t id, uint32_t value);
/* a CRC-checked ULTRA_PIXEL_FX slot of a frame whose header TIME is hdr_us. Applied once per (segment, SEQ). */
void     udfx_pixel_fx(udfx_t *u, const uint8_t *slot, uint32_t hdr_us);
/* once a ms: mode requests, the policy (stored effect at power-up / signal loss, a stream taking
 * over), the limiter's measured current. d: dline (its counters and positions) */
void     udfx_service(udfx_t *u, const dline_t *d, uint32_t ms, uint32_t meas_ma, uint32_t vk_q8);
/* one render step (<= UX_CHUNK pixels); returns 1 while it has more to do right now */
uint32_t udfx_render(udfx_t *u);
/* flash may be written: the strip is on the hub's stream or dark */
uint32_t udfx_flash_ok(const udfx_t *u);
/* the record: pack the configuration and the stored effects into rec (UX_REC_BYTES) with sequence number seq */
void     udfx_rec_pack(const udfx_t *u, uint32_t seq, uint8_t *rec);
int      udfx_rec_valid(const uint8_t *rec);
void     udfx_rec_saved(udfx_t *u);                         /* written */
/* page 0x81, part p (0 state, 1 effects, 2 palettes); returns 0, or -1 for another part */
int      udfx_page(const udfx_t *u, uint32_t part, uint8_t *d);

/* ---- tick context */
/* after dline_tick: the mode switches, the fill. rx: the ring words written (dline.wpos) */
void     udfx_tick(udfx_t *u, dline_t *d, uint16_t *ring);
/* after dline_restart: re-init the encoder at the restarted position */
void     udfx_restarted(udfx_t *u, dline_t *d);
/* the flash operation's SRAM tick: the fill only. Inline: its caller is SRAM code (ws_ring_fill is too) */
UX_AINLINE void udfx_ram_tick(udfx_t *u, uint16_t *ring, uint32_t limit)
{
    if (u->local) {
        (void)ws_ring_fill(&u->e, &u->lut, ring, limit);
    }
}

#endif
