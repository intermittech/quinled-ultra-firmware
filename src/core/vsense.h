/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Firmware model and pin-role contract. */
#ifndef VSENSE_H
#define VSENSE_H

#include <stdint.h>

#define VSENSE_WIN_MS 16u           /* one average: 16 ms, 32 conversions */

typedef struct {
    uint32_t ratio_q8;              /* input voltage / pin voltage, Q8 (256 = the pin itself) */
    uint32_t vref_mv;               /* the ADC reference (full scale at 4095) */
    uint32_t acc, n, ms;            /* the window: pair sums, pairs, ms */
    uint32_t mv;                    /* the last window's average at the input, mV */
    uint32_t min_mv, max_mv;        /* lowest / highest average since vsense_report() */
    uint8_t  valid;                 /* mv holds a measurement */
} vsense_t;

/* firmware model */
void     vsense_init(vsense_t *v, uint32_t ratio_q8, uint32_t vref_mv);
/* every ms: the pair sums of that ms and their count (0 when nothing was converted) */
void     vsense_add(vsense_t *v, uint32_t sum2, uint32_t n);
/* mV at the input for `n` pairs summing to sum2 (two 12-bit conversions each) */
uint32_t vsense_mv(const vsense_t *v, uint32_t sum2, uint32_t n);
/* the lowest / highest average since the last call; both restart at the newest average */
void     vsense_report(vsense_t *v, uint32_t *min_mv, uint32_t *max_mv);

#endif
