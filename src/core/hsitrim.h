/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The hub-locked HSI trim (CH32V40x; decided 2026-10-22). The board boots with HSITRIM lowered
 * (MCU_HSI_BOOT_TRIM_STEPS: the core stays below 200 MHz at the HSI's +2.2 %). Each clock measurement against the
 * hub's host-us timestamps (main_ur_digital_m.c ud_clock, >= 1 s windows, ~50 ppm resolution) gives the local error;
 * this controller moves HSITRIM to the step at or just below nominal: the measured error at most -HSITRIM_MARGIN_PPM,
 * so HCLK stays under 200 MHz with the measurement's resolution included. The step size and direction are learned
 * from the measured change of each step (nominal ~0.2 %, RM p.23; its sign is not stated). After every step the caller
 * discards its rate references (the clock moved) and re-measures before answering. Pure logic, host-tested. */
#ifndef HSITRIM_H
#define HSITRIM_H

#include <stdint.h>

#define HSITRIM_MARGIN_PPM 150         /* the target: error <= -150 ppm (measurement resolution ~50 ppm) */
#define HSITRIM_HYST_PPM 100          /* a step up only if it lands this far below the target */
#define HSITRIM_STEP_MIN_PPM 800       /* plausible learned step sizes */
#define HSITRIM_STEP_MAX_PPM 4000
#define HSITRIM_STEP_ASSUME_PPM 3000   /* an unmeasured step up: at most this (RM p.23: "about 40 kHz" = 2000 ppm) */

typedef struct {
    int32_t step_ppm;                  /* ppm per HSITRIM step, signed: the direction (learned) */
    int32_t last_err;                  /* the error measured before the last step */
    uint8_t trim;                      /* the value in RCC_CTLR */
    uint8_t last_trim;
    uint8_t stepped;                   /* the next measurement follows a step: learn from it */
    uint8_t locked;                    /* at the step at / just below nominal */
    uint8_t learned;                   /* step_ppm measured (else the nominal sign, bounds for the size) */
    uint16_t steps;                    /* statistics */
} hsitrim_t;

void hsitrim_init(hsitrim_t *t, uint32_t trim, int32_t step_ppm);
/* a measured local clock error, ppm (+ = fast); returns the HSITRIM value to apply (the same: no change) */
uint32_t hsitrim_update(hsitrim_t *t, int32_t err_ppm);

#endif
