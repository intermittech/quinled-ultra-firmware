/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Channel health (0.6.0), from the current calibration test (core/isense.h: zero, then each channel alone at
 * 25 / 50 / 75 / 100 %) and the running channel model. Display only (console, a D9 code, a Diff-Power receiver's HEALTH
 * page): nothing here changes the outputs.
 *  - The test classifies each channel against the test stored before it:
 *      no load    under HEALTH_NOLOAD_MA at 100 % (as the model: nothing connected), and nothing there before either;
 *      normal;
 *      abnormal   the current didn't rise with the level (a step not higher than the one before, or 100 % below twice
 *                 25 %), or at 100 % it is HEALTH_TEST_PCT % and HEALTH_TEST_MA mA above / below the test before, or
 *                 no load where the test before had one.
 *    Its zero (every output off: logic + offset) shows current while dark: HEALTH_DARK_MA over the test before's zero,
 *    or HEALTH_DARK_ABS_MA without one (a FET stuck on, leakage).
 *    After a test with a finding, D9 blinks a code (main.c, ack.h ACK_P_FAULT): one 1.5 s flash, then the code as a
 *    value code (long = 5, short = 1): 1..5 the first abnormal channel (R, G, B, W1, W2), 6 = current while dark.
 *  - Once a second: the running model (the regression's current at 100 % of each channel) against the stored test,
 *    flagged from HEALTH_DRIFT_PCT % ("W1: +35 %"), and the current while dark (the zero tracking) against the stored
 *    test's zero.
 *  - "Calibration suggested" (0.6.5): a channel's running model HEALTH_DRIFT_PCT % and HEALTH_TEST_MA mA off the
 *    stored test (or HEALTH_NEWLOAD_MA on a channel the test found empty: a strip added) for HEALTH_SUGGEST_S seconds
 *    while that channel is on (dark seconds neither count nor reset it: the model only learns while lit) latches
 *    HEALTH_F_CAL until the next test. Display only: the limiter keeps using the running model as before; nothing
 *    starts a test by itself.
 * Pure logic, host-tested (test/test_health.c). */
#ifndef HEALTH_H
#define HEALTH_H

#include <stdint.h>
#include "isense.h"
#include "ultra_protocol.h"

#define HEALTH_NOLOAD_MA   20
#define HEALTH_TEST_PCT    30
#define HEALTH_TEST_MA     50
#define HEALTH_DRIFT_PCT   25
#define HEALTH_DARK_MA     50
#define HEALTH_DARK_ABS_MA 100
#define HEALTH_NEWLOAD_MA  200     /* a channel the stored test found empty: this much learned is a new load */
#define HEALTH_SUGGEST_S   300u    /* drifting this long (s, the channel on) raises HEALTH_F_CAL */
/* the HEALTH page's flags (ULTRA_HE_FLAGS) bit 4: calibration suggested. Proposed for protocol 0.2.4 as
 * ULTRA_HE_F_CAL_SUGGESTED; a 0.2.3 receiver ignores the bit (0x10-0x80 were never set) */
#define HEALTH_F_CAL       0x10u

typedef struct {
    uint8_t  flags;                /* ULTRA_HE_F_*, HEALTH_F_CAL */
    uint8_t  cls[5];               /* the last test's class (ULTRA_HE_*) | reasons (ULTRA_HE_R_*) */
    int8_t   test_dev[5];          /* the last test at 100 % against the one before, % (ULTRA_HE_NA) */
    int8_t   model_dev[5];         /* the running model against the stored test, % (ULTRA_HE_NA) */
    int16_t  dark_ma;              /* current with every output off, over the stored test's zero (or absolute) */
    int16_t  prev_zero;            /* the test before the running one */
    uint16_t prev_ma[5];           /* ... each channel at 100 % (0: no load, or none) */
    uint8_t  prev_have;
    uint16_t drift_s;              /* seconds a channel drifted while on (HEALTH_F_CAL at HEALTH_SUGGEST_S) */
} health_t;

void    health_init(health_t *h);
/* (a - b) in % of b (b > 0), clamped to -127..127 (0.6.6: inline, also the calibration history's trend, core/calhist.c:
 * out of line it cost health_run and health_test_done ~60 cycles) */
static inline int8_t dev_pct(int32_t a, int32_t b)
{
    int32_t d = (a - b) * 100 / b;
    return (int8_t)(d > 127 ? 127 : d < -127 ? -127 : d);
}
/* a test starts: the stored test (levels 0: none) */
void    health_test_start(health_t *h, const isense_cal_t *stored);
/* the test is done: classify; returns the D9 code (0: nothing found, 1..5 the first abnormal channel, 6 dark) */
uint8_t health_test_done(health_t *h, const isense_cal_t *now);
/* every second: the running model and the zero tracking against the stored test (levels 0: none) */
void    health_run(health_t *h, const isense_t *is, const isense_cal_t *stored);
/* a test's current at 100 % (a single-level test scaled from its duty), mA */
uint32_t health_i100(const isense_cal_t *c, uint32_t ch);
/* HEALTH page DATA (protocol 0.2.3, ULTRA_HE_*): with the MCU temperature peak (0.1 C, ULTRA_ST_NA) and its age */
void    health_page(const health_t *h, int16_t temp_peak, uint32_t peak_age_s, uint8_t out[24]);

#endif
