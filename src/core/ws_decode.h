/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* WS28xx one-wire decoder working on an oversampled bitstream.
 *
 * Hardware: SPI1 runs in master receive-only mode at 144 MHz / 4 = 36 MHz (datasheet max), 16-bit frames, and
 * DMA writes the samples of DIN (PA6) into a ring of 16-bit words, MSB = earliest sample.
 * The 20 kHz poll ISR notes the ring position when TIM2's edge count first leaves 0 (the frame's first edge,
 * 0-50 us ago) and later copies the ring from 60 us before that point: the copy starts inside the reset gap
 * (>= 280 us), so the decoder only runs forwards from the first rising edge.
 *
 * Bit value = length of the high pulse: WS2915 / WS2805 T0H 220-380 ns, T1H >= 520 ns (580 ns WS2805).
 * Sample period 27.8 ns, HSI +-2.2 % (datasheet, full temperature range): threshold 17 samples (472 ns).
 * At the spec corners '0' <= 16 samples and '1' >= 17 samples hold with up to +-37 ns of edge distortion
 * (cable, level shifter, input divider). 18 MHz sampling would leave ~0 ns there.
 * The decoder walks the level transitions of each 16-sample word (XOR with the shifted word + byte clz table).
 */
#ifndef WS_DECODE_H
#define WS_DECODE_H

#include <stdint.h>

typedef struct {
    uint8_t  thr_one;    /* high run >= thr_one samples decodes as '1' */
    uint8_t  high_max;   /* longer high run = not a WS bit (error) */
    uint16_t gap_min;    /* low run >= gap_min samples = reset gap: frame ended early */
} ws_timing_t;

#define WS_SAMPLE_HZ       36000000UL
#define WS_TIMING_DEFAULT  { 17u, 80u, 1800u }  /* 472 ns threshold, 2.2 us max high, 50 us gap */

enum {
    WS_ERR_NO_EDGE = -1,      /* no rising edge in the buffer */
    WS_ERR_NO_GAP = -2,       /* buffer does not start in the idle (low) line */
    WS_ERR_SHORT = -3,        /* a reset gap came before nbits bits: frame shorter than expected */
    WS_ERR_TRUNCATED = -4,    /* buffer ended inside the wanted bits */
    WS_ERR_PULSE = -5,        /* high pulse longer than high_max */
};

/* buf: nsamples samples in 16-bit words, MSB first, starting inside the reset gap before the frame.
 * Decodes the first nbits bits. out: packed bits, MSB first, (nbits + 7) / 8 bytes, zeroed by the decoder.
 * Returns nbits on success, else a WS_ERR_* code. */
int ws_decode_frame(const uint16_t *buf, uint32_t nsamples, uint16_t nbits, const ws_timing_t *t, uint8_t *out);

/* A frame whose length is to be checked (the Ultra-Digital receiver's marker frames): as ws_decode_frame, but the
 * walk goes on to the end of buf. Decodes the first nbits bits into out ((nbits + 7) / 8 bytes, zeroed) and counts
 * every complete bit in buf (a HIGH at the end without its fall is no bit). Returns that count (only the first nbits
 * are in out), or a WS_ERR_* code (a reset gap inside: WS_ERR_SHORT). */
int ws_decode_count(const uint16_t *buf, uint32_t nsamples, uint16_t nbits, const ws_timing_t *t, uint8_t *out);

/* The end of a frame (Ultra trailer header): buf may start anywhere inside the frame (a bit cut at the start is
 * skipped) and must end after its last bit. Decodes every complete bit and keeps the last 64 in out (8 bytes, MSB
 * first). Returns the number of complete bits, or a WS_ERR_* code (a reset gap inside: WS_ERR_SHORT). */
int ws_decode_last(const uint16_t *buf, uint32_t nsamples, const ws_timing_t *t, uint8_t *out);

#endif
