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

/* The same two decodes by the RISING edges only (the Ultra-Digital receiver's chain mode, where two CRCs check the result and the main loop has
 * 50 us ticks to share): a bit is '1' when its high pulse is >= 17 samples (WS_TIMING_DEFAULT's thr_one) and a word is 16 samples, so the
 * sample 16 after a rising edge is the same bit of the next word: HIGH = '1'. The falling edges, the pulse's length limit and the reset gap
 * inside the frame are not looked at (a frame that is cut or has noise fails its CRC). About the cost of ws_decode_frame / _last (~90 cycles a bit); they are
 * here for what those two cannot do: ws_rise_step resumes where the last call stopped (the frame's words arrive while it is decoded), ws_rise_last reads a ring in place.
 * ws_rise_frame: the first nbits bits into out ((nbits + 7) / 8 bytes, zeroed); returns nbits, or WS_ERR_NO_GAP (the buffer starts HIGH),
 * WS_ERR_NO_EDGE / WS_ERR_TRUNCATED (fewer bits in buf). ws_rise_last: nwords words of a ring (ring[(start + k) & mask]), every bit whose
 * rising edge has a word after it, the last 64 kept in out; the window may start inside a pulse (that bit is skipped); returns the bits
 * counted or WS_ERR_NO_EDGE. */
int ws_rise_frame(const uint16_t *buf, uint32_t nsamples, uint16_t nbits, uint8_t *out);
/* ws_rise_frame in steps, as the frame's words arrive (buf[0 .. nwords) so far, nwords only grows): ws_rise_t zeroed before the first call, out
 * zeroed by the caller. Returns nbits when they are all out, 0 for "more words, please" (the last word waits for the one after it), or
 * WS_ERR_NO_GAP */
typedef struct { uint16_t k, b; uint8_t prev; } ws_rise_t;
int ws_rise_step(ws_rise_t *st, const uint16_t *buf, uint32_t nwords, uint16_t nbits, uint8_t *out);
int ws_rise_last(const uint16_t *ring, uint32_t mask, uint32_t start, uint32_t nwords, uint8_t *out);

#endif
