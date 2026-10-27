/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Firmware model and pin-role contract. */
#ifndef RC_H
#define RC_H

#include <stdint.h>
#include "../mcu/profile.h"
#include "ultra_protocol.h"

#define RC_DIR_US        500u          /* turn around here (the middle of ULTRA_RW_DIR_MIN_US .. _MAX_US) */
#define RC_CHECK_US      920u          /* the turnaround check: after ULTRA_RW_DIR_MAX_US, before slot 0 */
#define RC_END_US        6u            /* forward again W + this: the rate (+-200 ppm: 4.8 us at the longest W, 24.2 ms)
                                          and t0 errors keep it >= W (2 could land at W - 2.7 us); with
                                          the slowest tick it lands W + 1.2 .. 62 us, ~67 us with the poll ISR's
                                          latency: inside ULTRA_RW_END_US (80; test_r6_host_rc.c r6_schedule) */
#define RC_ANSWER_BYTES  32u
#define RC_RATES         ULTRA_RW_RATES   /* POLL return rate: 0 = 1 Mbaud, 1 = 500 kbaud, 2 = 250 kbaud */
#define RC_SAMPLES_TICK  (WS_SAMPLE_HZ / MCU_POLL_HZ)         /* DIN samples (36 MHz) per poll tick (50 us) */
#define RC_PREAMBLE_CYC  MCU_CYC_US(3u)          /* the line HIGH this long (HCLK cycles: 3 us) before the first start bit */
#define RC_LEAD_BYTES    1u            /* a 0xFF before the answer's SYNC (mcu/ch32v20x/hw.c rc_hw_send): the
                                          turnaround checks' pulses (920-970 us) reach the hub as UART frames of 10
                                          bits; at 250 / 500 kbaud the stop bit of one can fall into slot 0's first
                                          byte and misframe it. A 0xFF has one falling edge, its start bit: a hub
                                          UART still inside such a frame ends it there and starts cleanly at SYNC (the
                                          hub discards the bytes before SYNC) */
/* a POLL slot's fields (slot byte offsets): ultra_protocol.h, ULTRA_POLL_* (the protocol wiki, Slot types: POLL) */
#define RC_P_INDEX       ULTRA_POLL_INDEX   /* answer index 0..R-1, or ULTRA_POLL_RELAY_ONLY */
#define RC_P_SLOTS       ULTRA_POLL_SLOTS   /* R: response slots in this window, 1..ULTRA_RW_MAX_SLOTS */
#define RC_P_PAGE        ULTRA_POLL_PAGE
#define RC_P_RATE        ULTRA_POLL_RATE    /* 0..RC_RATES-1 */
#define RC_P_ARG         ULTRA_POLL_ARG     /* page arguments: slot bytes 7..29 */

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
/* the slot pitch for a return rate, us: the lead byte and the answer, 335 bit times at the rate + 165 us (500 /
 * 835 / 1505): the header's ultra_rw_slot_us */
#define rc_slot_us(rate) ultra_rw_slot_us(rate)
/* the last falling edge in a DIN copy (words of 16 samples, MSB = earliest): samples from it to the copy's end, or -1
 * (no high sample) */
int32_t  rc_last_edge(const uint16_t *w, uint32_t words);
/* the schedule of a valid POLL: t0 lies back_samples DIN samples before the ring position read at local tick edge_tick
 * (negative: the edge's samples were not in the ring yet at that read); rate_q16 = host us per local tick, Q16 (the
 * shared clock's) */
void     rc_schedule(rc_sched_t *s, uint32_t edge_tick, int32_t back_samples, uint32_t rate_q16, const uint8_t *poll);
/* the answer UART's BRR (board.h RC_BRR_BASE << rate: 72 / 144 / 288 at PCLK2 72 MHz) for the return rate, corrected by the local clock's measured rate (the HSI's error) */
uint16_t rc_brr(uint32_t rate_q16, uint8_t rate);
/* k big-endian fields into d, retaining each value's low 1..4 bytes. */
/* Four widths (1..4 bytes), first field in the low two bits. Up to sixteen
 * fields fit in one word; append groups by shifting them by 8, 16 and 24. */
#define RC_WIDTHS4(a,b,c,d) (((a)-1u) | (((b)-1u)<<2) | (((c)-1u)<<4) | (((d)-1u)<<6))
void     rc_pack(uint8_t *d, const uint32_t *v, uint32_t widths, uint32_t k);
/* the answer around DATA (a + ULTRA_RSP_DATA, 24 bytes, filled before): SYNC, PAGE, INDEX << 4 | RFLAGS, SEQ (the
 * POLL's), UID16, then the CRC-16 over bytes 0..29 */
void     rc_answer(uint8_t a[RC_ANSWER_BYTES], const uint8_t *poll, uint8_t page, uint8_t rflags, uint16_t uid16);

#endif
