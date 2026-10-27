/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Firmware model and pin-role contract. */
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
/* the HEALTH page's flags (ULTRA_HE_FLAGS) bit 4: calibration suggested
 * (ULTRA_HE_F_CAL_SUGGESTED) */
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
/* (a - b) in % of b (b > 0), clamped to -127..127 (inline, also the calibration history's trend, core/calhist.c:
 * out of line it would cost health_run and health_test_done ~60 cycles) */
static inline int8_t dev_pct(int32_t a, int32_t b)
{
    int32_t d = (a - b) * 100 / b;
    return (int8_t)(d > 127 ? 127 : d < -127 ? -127 : d);
}
/* a test starts: the stored test (levels 0: none) */
void    health_test_start(health_t *h, const isense_cal_t *stored);
/* the test is done: classify; returns the status LED code (0: nothing found, 1..5 the first abnormal channel, 6 dark) */
#define HEALTH_CODE_CAL_FAIL 8u    /* (main.c) status LED: a test stopped incomplete (trip, limit, supply) */
#define HEALTH_CODE_CAL_SUGGEST 7u /* (main.c) status LED: a calibration test suggested (HEALTH_F_CAL rose) */
#define HEALTH_CODE_HOT      10u   /* (main.c temp_service) status LED: the MCU die over TEMP_HOT_C10 */
#define HEALTH_CODE_PROTECT  9u    /* (R7-06; from the bootloader's report) status LED: the option bytes unreadable (nothing
                                      written), or not as written after the protection's reset */
uint8_t health_test_done(health_t *h, const isense_cal_t *now);
/* every second: the running model and the zero tracking against the stored test (levels 0: none) */
void    health_run(health_t *h, const isense_t *is, const isense_cal_t *stored);
/* a test's current at 100 % (a single-level test scaled from its duty), mA */
uint32_t health_i100(const isense_cal_t *c, uint32_t ch);
/* HEALTH page DATA (ULTRA_HE_*): with the MCU temperature peak (0.1 C, ULTRA_ST_NA) and its age */
void    health_page(const health_t *h, int16_t temp_peak, uint32_t peak_age_s, uint8_t out[24]);

#endif
