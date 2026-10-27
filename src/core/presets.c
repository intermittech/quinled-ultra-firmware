/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "presets.h"

/* index = config ARG. Below 20 kHz: half-bit boundaries; from 20 kHz: round values plus the 12 / 11-bit points. */
const uint16_t pwm_preset_steps[PWM_PRESET_COUNT] = {
    MCU_TIM_NS_ROUND(455104u),  /*  0   2.197 kHz  16.0 bit (65535: 16-bit input maps 1:1, CCR 65535 = 100 %) */
    MCU_TIM_NS_ROUND(321812u),  /*  1   3.107 kHz  15.5 bit */
    MCU_TIM_NS_ROUND(227556u),  /*  2   4.395 kHz  15.0 bit */
    MCU_TIM_NS_ROUND(160903u),  /*  3   6.215 kHz  14.5 bit */
    MCU_TIM_NS_ROUND(113778u),  /*  4   8.789 kHz  14.0 bit */
    MCU_TIM_NS_ROUND(80451u),  /*  5  12.43  kHz  13.5 bit */
    MCU_TIM_NS_ROUND(56889u),   /*  6  17.58  kHz  13.0 bit */
    MCU_TIM_NS_ROUND(50000u),   /*  7  20     kHz  12.8 bit */
    MCU_TIM_NS_ROUND(40000u),   /*  8  25     kHz  12.5 bit  (default) */
    MCU_TIM_NS_ROUND(33333u),   /*  9  30     kHz  12.2 bit */
    MCU_TIM_NS_ROUND(28444u),   /* 10  35.16  kHz  12.0 bit */
    MCU_TIM_NS_ROUND(25000u),   /* 11  40     kHz  11.8 bit */
    MCU_TIM_NS_ROUND(20000u),   /* 12  50     kHz  11.5 bit */
    MCU_TIM_NS_ROUND(14222u),   /* 13  70.31  kHz  11.0 bit */
    MCU_TIM_NS_ROUND(10000u),   /* 14 100     kHz  10.5 bit */
};
