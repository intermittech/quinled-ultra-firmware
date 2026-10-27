/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Explicit digital calibration. Main-context only; no ADC/encoder/flash dependency.
 * udcal_feed takes each fresh ISR batch exactly once. udcal_step performs at most
 * one reduction. The adapter renders <=16 pixels, publishes with a barrier, and
 * calls latched only AFTER the final reset has left the delay ring. */
#ifndef UDCAL_H
#define UDCAL_H
#include <stdint.h>
#include "../ultra_protocol.h"
/* the board's own budget = descriptor cal_ms (board.json calibration_ms, per MCU binding): its full-length worst case
 * (its most five-byte pixels at the test model's 1.4 us bit, test/test_udcal.c) x 1.15, rounded up to 500 ms */
#define UC_DEADLINE_MS BOARD_CALIBRATION_MS
#define UC_DUTY 64u
#define UC_REC_BYTES 64u
#define UC_REC_MAGIC 0x31434455u /* UDC1, little endian */
typedef struct {
    uint16_t configured, detected, n, order, vbus, vled;
    uint16_t ch[5];
    int16_t idle;
    uint16_t noise, residual, bound, k, count, epoch;
    uint8_t chip, bpp, flags;
} udcal_model_t;
typedef struct {
    udcal_model_t model;
    uint32_t model_ms;                 /* RAM age only; reboot has unknown age */
    udcal_model_t candidate;
    uint32_t start_ms, now_ms, block_ms, last_ms;
    uint32_t sum, first, value, prediction, upper;
    uint32_t dark[3], light[6], lo_adc, hi_adc;
    uint16_t stamp[9], samples, limit, cap, lo, hi, pos, k, elapsed;
    uint8_t phase, error, stage, index, channel, duty, mask;
    uint8_t pending, sampling, finish, seq, have_seq;
} udcal_t;
typedef char udcal_ram_budget[sizeof(udcal_t) <= 240u ? 1 : -1];
/* Request uses the resolved/applied fingerprint. Duplicate SEQ is a no-op.
 * nonzero admission error fails without requesting any output. */
void udcal_request(udcal_t *c, uint8_t seq, const udcal_model_t *cfg,
                   uint16_t cap, uint16_t limit, uint32_t ms, uint8_t error);
uint32_t udcal_active(const udcal_t *c);
void udcal_abort(udcal_t *c, uint8_t error);
void udcal_invalidate(udcal_t *c);
void udcal_latched(udcal_t *c, uint32_t ms);
void udcal_restored(udcal_t *c);
/* raw sum/count, current scale Q16 mA/count, batch timestamp, answer contamination.
 * Invalid/missing/delayed batches, >2% voltage drift or 80% L fail to black. */
void udcal_feed(udcal_t *c, uint32_t sum, uint32_t n, uint32_t scale,
                uint32_t ms, uint32_t blocked, uint32_t vbus, uint32_t vled);
void udcal_step(udcal_t *c, uint32_t scale);
/* One raw logical channel value for the current pattern; all unselected pixels
 * are black. Full cap frames clear the previous probe, even on a shorter probe. */
uint8_t udcal_pixel(const udcal_t *c, uint32_t pixel);
uint8_t udcal_byte(const udcal_t *c, uint32_t pixel, uint32_t channel);
int udcal_page(const udcal_t *c, uint32_t part, uint8_t *d);
void udcal_pack(const udcal_t *c, uint32_t seq, uint8_t r[UC_REC_BYTES]);
int udcal_valid(const uint8_t *r);
void udcal_load(udcal_t *c, const uint8_t *r);
#endif
