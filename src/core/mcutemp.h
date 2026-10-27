/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* MCU temperature from the CH32V203's internal sensor (ADC channel 16), with VREFINT (channel 17) as the reference.
 * The board converts both once a second (hal: hw_temp_start / hw_temp_read, the ADC's injected group) and hands the
 * results here: sums of two 12-bit conversions each (0..8190).
 *   Vsense = 1.20 V x ts / vref                (VREFINT, typ. 1.20 V: VDDA cancels out)
 *   T      = 25 C + (V25 - Vsense) / Avg_Slope  (datasheet table 4-30, typical: V25 = 1.40 V, 4.3 mV/C; the
 *                                               voltage falls as the die warms)
 * It is the die temperature, a few C above the board. The absolute value varies per chip (V25 1.34-1.46 V, slope
 * 3.8-4.7 mV/C, VREFINT 1.17-1.23 V: up to ~+-20 C uncalibrated), so treat it as a trend / over-temperature value;
 * `offset_c10` takes a one-point calibration (reserved: CONFIG ULTRA_CFG_MCU_TEMP_OFFSET, not settable yet).
 * Filter: first-order, 1/8 per sample (~8 s); a sample more than 10 C off the filtered value counts as bad, unless
 * 3 come in a row (a real step: start again from it). */
#ifndef MCUTEMP_H
#define MCUTEMP_H

#include <stdint.h>

#define MCUTEMP_NA        ((int16_t)-32768)   /* 0x8000: not valid (the STATUS page's "n/a") */
#define MCUTEMP_VREF_MV   1200                /* VREFINT, typical */
#define MCUTEMP_V25_MV10  14000               /* V25 in 0.1 mV */
#define MCUTEMP_SLOPE_UV  4300                /* Avg_Slope, uV / C */
#define MCUTEMP_BAD_MAX   5u                  /* bad / missing samples in a row: not valid any more */

typedef struct {
    int32_t  acc;                 /* filtered temperature, 0.1 C x 16 */
    int16_t  offset_c10;          /* one-point calibration, 0.1 C (added to the result) */
    int16_t  c10;                 /* filtered temperature with the offset, 0.1 C; MCUTEMP_NA while not valid */
    uint16_t vdda_mv;             /* the 3.3 V rail from VREFINT (last good sample), 0 = not known */
    uint8_t  valid;
    uint8_t  have;                /* acc holds a value */
    uint8_t  bad;                 /* bad / missing samples in a row */
    uint8_t  jump;                /* plausible samples > 10 C off the filter in a row */
    uint32_t samples, rejected;   /* counters (status) */
} mcutemp_t;

void mcutemp_init(mcutemp_t *t);
/* one sample, no filter: 0.1 C (without offset) and VDDA in mV from the sums of two conversions each; 0 = not
 * plausible (VDDA outside 2.3-3.7 V, or the temperature outside -50..+150 C) */
int  mcutemp_convert(uint32_t ts2, uint32_t vref2, int16_t *c10, uint16_t *vdda_mv);
/* a new pair (sums of two conversions each); ts2 = vref2 = 0: the conversion didn't complete */
void mcutemp_sample(mcutemp_t *t, uint32_t ts2, uint32_t vref2);

#endif
