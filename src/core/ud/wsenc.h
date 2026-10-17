/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The WS281x symbol encoder of the Ultra-Digital receiver's local pixel effects (BOARD 3; docs/ultradigital-firmware.md
 * section 11, review-r4/ud-local-effects/STUDY.md).
 *
 * The delay ring (hal/hw_dline.c) stays as it is: SPI1 full duplex at 36 MHz, RX DMA writes word k into ring[k % 512],
 * TX DMA replays ring[k % 512] ~510 words (226 us) later. In local mode the 20 kHz tick, after dline's scan has read the
 * new RX words (markers, poll capture), overwrites them with WS281x symbols before TX reads them: the same move as
 * the scan's zeroing, with symbols instead of zeros. Nothing in the SPI, the DMA, the pins or the PCB changes, and the
 * stream / local switch is a per-word decision (udfx.c).
 *
 * A bit is s samples (36 MHz, 27.8 ns): t0h / t1h HIGH, the rest LOW. s = 48 exactly (1.333 us a bit: three whole ring
 * words, no phase) is the fast grid every 800 kHz chip uses: a nibble is 12 words from a 16-entry table, copied as six
 * 32-bit words. Other s (WS2811 at 400 kHz: 90) run as a pattern part of 48 samples plus a LOW tail: a bit starting at
 * phase p of a word touches at most 4 words when p + s <= 64, so a table of 2 x 16 x 4 words gives each bit
 * branch-free: word 0 ORed into the partial word, words 1-2 stored, (p + s) >> 4 words completed.
 *
 * The encoder writes only words RX has already written (w < limit), whole bytes only: a byte starts when all its words
 * fit (24 on the fast grid, 27 on the exact one), the rest waits for the next tick (deadline: TX reads a word ~510
 * words after RX wrote it; the host test passes with a 480-word tick gap).
 *
 * Integer only, no division, no 64-bit. ws_ring_fill is SRAM code (the flash operation's tick runs it too). */
#ifndef WSENC_H
#define WSENC_H

#include <stdint.h>

#define WS_RING_WORDS 512u
#define WS_RING_MASK  (WS_RING_WORDS - 1u)
#define WS_W_PER_MS   2250u           /* ring words per ms (36 MHz / 16) */
#define WS_LUT_S_MAX  48u             /* samples per table pattern (p + 48 <= 63: 4 words) */
#define WS_GUARD      64u             /* words past the ring's end the encoder may store into (a byte's: <= 51) */
#define WS_LATE_W     432u            /* a fill this many words behind RX: the words are out already. TX reads a word ~507 words
                                         after RX wrote it, and the scan before the fill takes ~0.1 word per word of gap (6.4
                                         cycles) plus ~25 for the interrupt: 1.1 x 432 + 25 = 500 */

typedef struct {
    uint8_t  s;                       /* samples per bit (48 = 1.333 us: the fast encoder) */
    uint8_t  t0h, t1h;                /* HIGH samples of a 0 / a 1 */
    uint8_t  bpp;                     /* wire bytes per pixel (3 RGB, 4 RGBW, 5 WS2805) */
    uint16_t reset_w;                 /* LOW words between frames (680 = 302 us) */
} ws_chip_t;

typedef struct {
    union {
        struct {
            uint16_t h[2][16][4];     /* exact: bit b started at phase p: its words; [0] is ORed into the partial word */
            uint8_t adv[16];          /* exact: words completed by the pattern part: (p + s1) >> 4 */
        } exact;                     /* neither exact table is used by the fast encoder */
        uint32_t nib[16][6];          /* fast (s = 48: 3 whole words a bit): a nibble's 12 words, 32-bit pairs */
    } t;
    uint8_t  s1;                      /* exact: the pattern part (min(s, 48)) */
    uint8_t  tail;                    /* exact: the LOW tail after it (s - s1: 0 at 800 kHz) */
    uint8_t  fast;                    /* s == 48: the nibble table; frames start at an even word (32-bit stores) */
} ws_lut_t;

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
    uint16_t cur;                     /* the partial word (its first p samples are written) */
    uint8_t  p;                       /* phase in the partial word */
    uint8_t  state;                   /* WS_LOW / WS_FRAME */
    volatile uint8_t ready;           /* the renderer has the next frame in fb (the encoder clears it at the start) */
    uint8_t  late;                    /* frames that started after their grid word (renderer late; saturating) */
    uint8_t  lost;                    /* fills behind RX by WS_LATE_W: the frame in progress is dropped (saturating) */
} ws_enc_t;

void     ws_lut_build(ws_lut_t *L, const ws_chip_t *c);
/* the frame buffer fb of npx pixels; the first frame waits for the grid (period_w: words) after a full reset from w0 */
void     ws_enc_init(ws_enc_t *e, const ws_chip_t *c, const uint8_t *fb, uint32_t npx, uint32_t w0, uint32_t period_w);
/* fill ring words e->w .. < limit (limit = RX's next word; a byte's bits only with room for all of them). ring:
 * WS_RING_WORDS + WS_GUARD words (the DMA uses the first 512). Returns the words written. SRAM code (section .ramtext) */
uint32_t ws_ring_fill(ws_enc_t *e, const ws_lut_t *L, uint16_t *ring, uint32_t limit);

#endif
