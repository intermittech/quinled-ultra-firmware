/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The return channel of the QuinLED Diff-Power Ultra-Analog receiver (BOARD_RETURN; ultra_protocol.h and the
 * protocol wiki, Return channel): POLL slots of RW frames, the response window's
 * schedule from the frame end, the answer framing.
 *
 * A receiver chain on one Diff-Power port: every receiver has an input RS-485 transceiver (towards the host, DE on
 * PB14) and an output one (towards the next receiver, DE on PA10); both ~RE on GND. In the response window every
 * receiver that got a valid POLL turns around: its input transceiver drives upstream, its output one listens, and a
 * resistor node (output RO via 1 k, USART3 TX on PB10 via 100 R) feeds the input DI, so the answers of the receivers
 * further down pass through in hardware. A receiver drives PB10 (USART3 TX, 8N1) only inside its own slot; otherwise
 * PB10 is an input (high-Z) and the node follows the output transceiver.
 *
 * Timing (all from t0 = the RW frame's last falling edge, measured locally in the DIN ring; offsets in host us,
 * scaled to local ticks with the shared clock's rate):
 *   turn around at t0 + RC_DIR_US (inside ULTRA_RW_DIR_MIN_US .. _MAX_US); at t0 + RC_CHECK_US (every board has
 *   turned around by ULTRA_RW_DIR_MAX_US) the turnaround check: the line HIGH for RC_PREAMBLE_CYC and read back, LOW =
 *   the segment above is still driven (a board there doesn't take part): stop driving upstream for this window (0.2.4);
 *   answer at t0 + ULTRA_RW_FIRST_US + index x the slot pitch (at the first poll tick at or after it: 0-50 us late,
 *   inside the -20 / +80 us tolerance; slot 0 never early, ULTRA_RW_EARLY0_US) with a 0xFF lead byte before the
 *   answer (ULTRA_RW_LEAD_BYTES); back at t0 + ULTRA_RW_FIRST_US + R x the pitch + RC_END_US (within
 *   ULTRA_RW_END_US). Protocol 0.2.4 (ultra_protocol.h, wiki: Return channel).
 *
 * Pure logic, host-tested (test/test_rc.c). The window itself runs in the poll ISR (main.c), the transmitter is
 * hal/hw.c's. Called only in the receiver's build (BOARD_RETURN): the dig2analog-ultra never reports back, and its
 * image holds none of it (LTO drops the unused functions). */
#ifndef RC_H
#define RC_H

#include <stdint.h>
#include "ultra_protocol.h"

#define RC_DIR_US        500u          /* turn around here (the middle of ULTRA_RW_DIR_MIN_US .. _MAX_US) */
#define RC_CHECK_US      920u          /* the turnaround check (0.2.4): after ULTRA_RW_DIR_MAX_US, before slot 0 */
#define RC_END_US        6u            /* forward again W + this: the rate (+-200 ppm: 4.8 us at the longest W, 24.2 ms)
                                          and t0 errors keep it >= W (review r6: 0.6.6's 2 went to W - 2.7 us); with
                                          the slowest tick it lands W + 1.2 .. 62 us, ~67 us with the poll ISR's
                                          latency: inside ULTRA_RW_END_US (80; test_r6_host_rc.c r6_schedule) */
#define RC_ANSWER_BYTES  32u
#define RC_RATES         3u            /* POLL return rate: 0 = 1 Mbaud, 1 = 500 kbaud, 2 = 250 kbaud (0.2.4) */
#define RC_SAMPLES_TICK  1800u         /* DIN samples (36 MHz) per poll tick (50 us) */
#define RC_PREAMBLE_CYC  432u          /* the line HIGH this long (HCLK cycles: 3 us) before the first start bit */
#define RC_LEAD_BYTES    1u            /* (review r6) a 0xFF before the answer's SYNC (hal/hw.c rc_hw_send): the
                                          turnaround checks' pulses (920-970 us) reach the hub as UART frames of 10
                                          bits; at 250 / 500 kbaud the stop bit of one can fall into slot 0's first
                                          byte and misframe it. A 0xFF has one falling edge, its start bit: a hub
                                          UART still inside such a frame ends it there and starts cleanly at SYNC (the
                                          hub discards the bytes before SYNC) */
/* a POLL slot's fields (slot byte offsets): ultra_protocol.h, ULTRA_POLL_* (the protocol wiki, Slot types: POLL) */
#define RC_P_INDEX       (ULTRA_PAYLOAD + 0u)   /* answer index 0..R-1, or ULTRA_POLL_RELAY_ONLY */
#define RC_P_SLOTS       (ULTRA_PAYLOAD + 1u)   /* R: response slots in this window, 1..ULTRA_RW_MAX_SLOTS */
#define RC_P_PAGE        (ULTRA_PAYLOAD + 2u)
#define RC_P_RATE        (ULTRA_PAYLOAD + 3u)   /* 0..RC_RATES-1 */
#define RC_P_ARG         (ULTRA_PAYLOAD + 4u)   /* page arguments: slot bytes 7..29 */

typedef struct {                       /* local poll ticks (wrap: compare differences) */
    uint32_t dir_tick;                 /* turn around */
    uint32_t check_tick;               /* the turnaround check */
    uint32_t tx_tick;                  /* this board's answer (index != RELAY_ONLY) */
    uint32_t end_tick;                 /* back to forward mode */
    uint32_t late_tick;                /* the first tick at or after ULTRA_RW_DIR_MAX_US: armed after late_tick - 2, the
                                          board doesn't take part (it would turn around after it) */
} rc_sched_t;

/* a slot (CRC-checked) is a valid POLL: 0 (LIGHT 0xF, 1 <= R <= 16, index < R or RELAY_ONLY, a known rate) */
int      rc_poll_parse(const uint8_t *slot);
/* the slot pitch for a return rate, us: 32 bytes x 10 bits at the rate + 130 us (450 / 770 / 1410) */
uint32_t rc_slot_us(uint8_t rate);
/* the last falling edge in a DIN copy (words of 16 samples, MSB = earliest): samples from it to the copy's end, or -1
 * (no high sample) */
int32_t  rc_last_edge(const uint16_t *w, uint32_t words);
/* the schedule of a valid POLL: t0 lies back_samples DIN samples before the ring position read at local tick edge_tick
 * (negative: the edge's samples were not in the ring yet at that read); rate_q16 = host us per local tick, Q16 (the
 * shared clock's) */
void     rc_schedule(rc_sched_t *s, uint32_t edge_tick, int32_t back_samples, uint32_t rate_q16, const uint8_t *poll);
/* USART3 BRR (PCLK1 144 MHz) for the return rate, corrected by the local clock's measured rate (the HSI's error) */
uint16_t rc_brr(uint32_t rate_q16, uint8_t rate);
/* k big-endian fields into d: field i is the low len[i] bytes (1-4) of v[i] (the pages' encoder) */
void     rc_pack(uint8_t *d, const uint32_t *v, const uint8_t *len, uint32_t k);
/* the answer around DATA (a + ULTRA_RSP_DATA, 24 bytes, filled before): SYNC, PAGE, INDEX << 4 | RFLAGS, SEQ (the
 * POLL's), UID16, then the CRC-16 over bytes 0..29 */
void     rc_answer(uint8_t a[RC_ANSWER_BYTES], const uint8_t *poll, uint8_t page, uint8_t rflags, uint16_t uid16);

#endif
