/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Bootloader DIN receiver: a streaming WS28xx decoder over the SPI sample ring (host-tested).
 *
 * SPI1 samples DIN at 36 MHz (144 MHz / 4, as the app) into a DMA ring of 16-bit words, MSB = earliest sample. The main loop
 * feeds the words in order; the decoder follows the level runs and detects the period from the first complete bit.
 * A high run is '1' when it occupies at least half of that period;
 * a low run of >= gap_min samples ends the frame. It keeps the frame's bit count and its last 384 bits (a whole
 * UPD frame, or the trailer header of any Ultra frame). No timer, no interrupt: frame boundaries come from the
 * samples themselves, so a late main loop only delays the result. After a stall (flash work) bl_rx_resync()
 * skips to the next gap. The sample arrival rate at 144 MHz is 64 CPU cycles per word. */
#ifndef BL_RX_H
#define BL_RX_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t thr_one;          /* reserved; the detected period sets the decision threshold */
    uint16_t high_max;         /* longer high run: not a WS bit (frame marked bad) */
    uint32_t gap_min;          /* low run >= gap_min samples: frame end */
} bl_rx_timing_t;

/* 36 MHz sampling: detected 800/600/400/200 kHz profile, high <= 160 samples, gap 100 us. */
#include "../src/mcu/profile.h"
#define BL_RX_TIMING_36MHZ  { 22u, 160u, 3600u }
#define BL_RX_TIMING        MCU_BL_RX_TIMING    /* the family's grid (V20x: BL_RX_TIMING_36MHZ's values) */
#define BL_RX_TAIL_BITS     384u
#define BL_RX_TAIL_WORDS    (BL_RX_TAIL_BITS / 32u)
#define BL_RX_MAX_FEED      128u          /* words per bl_rx_feed call: < one gap, so <= 1 frame end per call */

typedef struct {
    uint32_t nbits;            /* bits in the frame */
    uint8_t  bad;              /* a pulse was too long */
    uint16_t period;
    uint8_t  tail[BL_RX_TAIL_BITS / 8u];   /* last min(nbits, 384) bits, right-aligned, MSB first */
#if MCU_BL_RX_EDGE_AGE
    uint32_t end_ago;          /* samples from this frame's last fall to the end of this feed */
#endif
#if MCU_BL_MARKER_TRACK
    uint8_t marker;            /* selects the qualified marker status-reply envelope */
#endif
} bl_frame_t;

typedef struct {
    bl_rx_timing_t t;
    uint32_t level;            /* line level of the last sample */
    uint32_t run;              /* samples in the current run (saturates) */
    uint32_t nbits;
    uint32_t acc;              /* the last nbits % 32 bits */
    uint32_t wi;               /* next slot in words[] */
    uint8_t  synced;           /* a gap was seen: bits count from here */
    uint8_t  ended;            /* the current low run already ended the frame */
    uint8_t  bad;
    uint16_t high, period;     /* pending pulse and frame period, including the first bit */
    uint16_t locked_period;    /* zero until a complete valid update frame locks its profile */
    uint32_t words[BL_RX_TAIL_WORDS];      /* circular, complete 32-bit groups of the frame */
#if MCU_BL_MARKER_TRACK
    uint8_t marker;            /* a valid marker preceded this frame */
#endif
} bl_rx_t;

__attribute__((noipa)) void bl_rx_init(bl_rx_t *r, const bl_rx_timing_t *t);
void bl_rx_resync(bl_rx_t *r);
/* n <= BL_RX_MAX_FEED words; returns 1 if a frame ended inside them (written to *out) */
__attribute__((noipa)) int bl_rx_feed(bl_rx_t *r, const uint16_t *w, uint32_t n, bl_frame_t *out);

#ifdef __cplusplus
}
#endif

#endif
