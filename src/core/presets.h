/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* PWM frequency presets (DESIGN_NOTES "PWM frequency presets").
 * Both PWM timers (TIM1 R/G/B, TIM3 W1/W2) count at 144 MHz; steps = period in timer counts (ARR + 1). */
#ifndef PRESETS_H
#define PRESETS_H

#include <stdint.h>

#define F_TIM_HZ           144000000UL
#define PWM_PRESET_COUNT   15u
#define PWM_PRESET_DEFAULT 8u          /* 25 kHz */
#define PWM_STEPS_MIN      1440u       /* 100 kHz */
#define PWM_STEPS_MAX      65535u      /* 2.197 kHz: CCR = 65535 > ARR = 65534 still gives 100 % on */
#define SKIP_MIN_HZ        2000u       /* pulse-skipping / dither patterns repeat at >= 2 kHz */

extern const uint32_t pwm_preset_steps[PWM_PRESET_COUNT];

/* dither pattern length: ceil(65535 / steps) entries resolve every 16-bit level (input range 0..65535) */
static inline uint8_t dither_len_for(uint32_t steps)
{
    return (uint8_t)((65535u + steps - 1u) / steps);
}

/* DMA table length for a period: long enough for the dither, and for pulse skipping down to SKIP_MIN_HZ
 * (one pulse per table at most f_pwm / SKIP_MIN_HZ periods) */
static inline uint8_t pattern_len_for(uint32_t steps)
{
    uint32_t d = dither_len_for(steps);
    uint32_t s = (F_TIM_HZ / SKIP_MIN_HZ) / steps;
    return (uint8_t)(s > d ? s : d);
}

#define PATTERN_MAX_LEN 50u            /* pattern_len_for(1440) */

#endif
