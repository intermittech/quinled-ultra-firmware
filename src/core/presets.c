/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "presets.h"

/* index = config ARG. Below 20 kHz: half-bit boundaries; from 20 kHz: round values plus the 12 / 11-bit points. */
const uint32_t pwm_preset_steps[PWM_PRESET_COUNT] = {
    65535,  /*  0   2.197 kHz  16.0 bit (65535: 16-bit input maps 1:1, CCR 65535 = 100 %) */
    46341,  /*  1   3.107 kHz  15.5 bit */
    32768,  /*  2   4.395 kHz  15.0 bit */
    23170,  /*  3   6.215 kHz  14.5 bit */
    16384,  /*  4   8.789 kHz  14.0 bit */
    11585,  /*  5  12.43  kHz  13.5 bit */
    8192,   /*  6  17.58  kHz  13.0 bit */
    7200,   /*  7  20     kHz  12.8 bit */
    5760,   /*  8  25     kHz  12.5 bit  (default) */
    4800,   /*  9  30     kHz  12.2 bit */
    4096,   /* 10  35.16  kHz  12.0 bit */
    3600,   /* 11  40     kHz  11.8 bit */
    2880,   /* 12  50     kHz  11.5 bit */
    2048,   /* 13  70.31  kHz  11.0 bit */
    1440,   /* 14 100     kHz  10.5 bit */
};
