/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The WS281x symbol encoder of the Diff-Power-UR-Digital-M's local pixel effects (BOARD 3; docs/ur_digital_m-firmware.md
 * section 11).
 *
 * The delay ring (mcu/ch32v20x/hw_dline.c) stays as it is: SPI1 full duplex at 36 MHz, RX DMA writes word k into ring[k % 512],
 * TX DMA replays ring[k % 512] ~510 words (226 us) later. In local mode the 20 kHz tick, after dline's scan has read the
 * new RX words (markers, poll capture), overwrites them with WS281x symbols before TX reads them: the same move as
 * the scan's zeroing, with symbols instead of zeros. Nothing in the SPI, the DMA, the pins or the PCB changes, and the
 * stream / local switch is a per-word decision (udfx.c).
 *
 * A bit is s samples (36 MHz, 27.8 ns): t0h / t1h HIGH, the rest LOW. s = 48 exactly (1.333 us a bit: three whole ring
 * words, no phase) is the fast grid every 800 kHz chip uses: a nibble is 12 words from a 16-entry table, copied as six
 * 32-bit words. Frames start at an even word so these stores are aligned.
 *
 * The encoder writes only words RX has already written (w < limit), whole bytes only: a byte starts when all its words
 * fit (24 words), the rest waits for the next tick (deadline: TX reads a word ~510
 * words after RX wrote it; the host test passes with a 480-word tick gap).
 *
 * Integer only, no division, no 64-bit. ws_ring_fill is SRAM code (the flash operation's tick runs it too). */
#ifndef WSENC_H
#define WSENC_H

#include <stdint.h>
#include "../../mcu/profile.h"

/* The CH32V40x keeps this interface and its ring-word time base, but writes no ring words: its frames go to the ARGB
 * serialiser on their own pin (argbenc.c), selected by SOURCE_SEL (main_ur_digital_m.c). */
#if defined(MCU_CH32V40X) && MCU_CH32V40X == 1
#define WS_ARGB 1
#else
#define WS_ARGB 0
#endif

#define WS_RING_WORDS MCU_DLINE_RING_WORDS
#define WS_RING_MASK  (WS_RING_WORDS - 1u)
#define WS_W_PER_MS   MCU_WORDS_MS           /* ring words per ms (36 MHz / 16) */
#define WS_GUARD      MCU_DLINE_GUARD_WORDS             /* words past the ring's end the encoder may store into (a byte: 24 words) */
#define WS_LATE_W     MCU_ENCODER_LATE_WORDS            /* a fill this many words behind RX: the words are out already. TX reads a word ~507 words
                                         after RX wrote it, and the scan before the fill takes ~0.1 word per word of gap (6.4
                                         cycles) plus ~25 for the interrupt: 1.1 x 432 + 25 = 500 */

typedef struct {
    uint8_t  s;                       /* samples per bit (48 = 1.333 us: the fast encoder) */
    uint8_t  t0h, t1h;                /* HIGH samples of a 0 / a 1 */
    uint8_t  bpp;                     /* wire bytes per pixel (3 RGB, 4 RGBW, 5 WS2805) */
    uint16_t reset_w;                 /* LOW words between frames (680 = 302 us) */
} ws_chip_t;

#if WS_ARGB
typedef struct {
    uint8_t  s, t0h, t1h;             /* the chip's bit period and HIGH times, 10 ns units (ws_chip_t's MCU_SYM_NS) */
} ws_lut_t;
/* HCLK cycles per 10 ns, Q16 (the ARGB counts system clock cycles, RM p.510): the clock's best estimate, from the boot
 * trim before the hub lock and from the measured rate after it (main_ur_digital_m.c). One word: the tick reads it */
extern volatile uint32_t ws_argb_q16;
#else
typedef struct {
    uint32_t nib[16][6];              /* a nibble: 12 words, stored as 32-bit pairs */
} ws_lut_t;
#endif

enum { WS_LOW = 0, WS_FRAME = 1 };

typedef struct {
    const uint8_t *fb;                /* the frame's wire bytes (bpp per pixel, the chip's order) */
    uint32_t nbytes;                  /* bytes per frame */
    uint32_t i;                       /* the next byte of the frame */
    uint32_t w;                       /* the next ring word to write (free-running, as dline's positions) */
    uint32_t low_w;                   /* LOW words since the last frame (saturates) */
    uint32_t next_w;                  /* the next frame starts at this word or later (the frame grid) */
    uint32_t period_w;                /* the grid: words per frame (0: back to back, reset_w apart) */
    uint32_t frames;                  /* frames started */
    uint32_t start_w;                 /* the word the current frame started at */
    uint16_t reset_w;
    uint8_t  state;                   /* WS_LOW / WS_FRAME */
    volatile uint8_t ready;           /* the renderer has the next frame in fb (the encoder clears it at the start) */
    uint8_t  late;                    /* frames that started after their grid word (renderer late; saturating) */
    uint8_t  lost;                    /* fills behind RX by WS_LATE_W: the frame in progress is dropped (saturating) */
#if WS_ARGB
    uint32_t end_w;                   /* the frame's computed end (start + its exact duration in words) */
    const uint8_t *fb_next;           /* the frame the renderer handed over (main loop, before ready): fb at the start */
    const uint8_t *volatile in_flight; /* tick: the frame the serialiser is sending (0: none). Two frames: the main loop builds
                                         the other one, never this */
#endif
} ws_enc_t;

void     ws_lut_build(ws_lut_t *L, const ws_chip_t *c);
/* the frame buffer fb of npx pixels; the first frame waits for the grid (period_w: words) after a full reset from w0 */
void     ws_enc_init(ws_enc_t *e, const ws_chip_t *c, const uint8_t *fb, uint32_t npx, uint32_t w0, uint32_t period_w);
/* fill ring words e->w .. < limit (limit = RX's next word; a byte's bits only with room for all of them). ring:
 * WS_RING_WORDS + WS_GUARD words (the DMA uses the first 512). Returns the words written. SRAM code (section .ramtext) */
uint32_t ws_ring_fill(ws_enc_t *e, const ws_lut_t *L, uint16_t *ring, uint32_t limit);

#endif
