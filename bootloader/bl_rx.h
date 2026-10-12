/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Bootloader DIN receiver: a streaming WS28xx decoder over the SPI sample ring (host-tested).
 *
 * SPI1 samples DIN at 36 MHz (144 MHz / 4, as the app) into a DMA ring of 16-bit words, MSB = earliest sample. The main loop
 * feeds the words in order; the decoder follows the level runs: a high run is a bit ('1' if >= thr_one samples),
 * a low run of >= gap_min samples ends the frame. It keeps the frame's bit count and its last 384 bits (a whole
 * UPD frame, or the trailer header of any Ultra frame). No timer, no interrupt: frame boundaries come from the
 * samples themselves, so a late main loop only delays the result. After a stall (flash work) bl_rx_resync()
 * skips to the next gap. Budget at 144 MHz: 64 cycles per word; ~39 used inside a frame (bootloader/measure_bl.py). */
#ifndef BL_RX_H
#define BL_RX_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t thr_one;          /* high run >= thr_one samples: '1' */
    uint16_t high_max;         /* longer high run: not a WS bit (frame marked bad) */
    uint32_t gap_min;          /* low run >= gap_min samples: frame end */
} bl_rx_timing_t;

/* 36 MHz: '1' >= 17 samples (472 ns, the app's threshold; the hub sends 250 / 700 ns), high <= 2.2 us, gap 100 us */
#define BL_RX_TIMING_36MHZ  { 17u, 80u, 3600u }
#define BL_RX_TAIL_BITS     384u
#define BL_RX_TAIL_WORDS    (BL_RX_TAIL_BITS / 32u)
#define BL_RX_MAX_FEED      128u          /* words per bl_rx_feed call: < one gap, so <= 1 frame end per call */

typedef struct {
    uint32_t nbits;            /* bits in the frame */
    uint8_t  bad;              /* a pulse was too long */
    uint8_t  tail[BL_RX_TAIL_BITS / 8u];   /* last min(nbits, 384) bits, right-aligned, MSB first */
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
    uint32_t words[BL_RX_TAIL_WORDS];      /* circular, complete 32-bit groups of the frame */
} bl_rx_t;

void bl_rx_init(bl_rx_t *r, const bl_rx_timing_t *t);
void bl_rx_resync(bl_rx_t *r);
/* n <= BL_RX_MAX_FEED words; returns 1 if a frame ended inside them (written to *out) */
int  bl_rx_feed(bl_rx_t *r, const uint16_t *w, uint32_t n, bl_frame_t *out);

#ifdef __cplusplus
}
#endif

#endif
