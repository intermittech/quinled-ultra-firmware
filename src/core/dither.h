/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* 16-bit level -> per-period compare values (README: dither modes, dim floor).
 *
 * The DMA writes one table entry per PWM period into the compare register:
 *  - Dither: entries alternate between q and q + 1 so the average is the exact 16-bit level:
 *      exact = v16 * steps / 65535 = q + r / 65535,  table = q + k / len,  k = round(r * len / 65535)
 *    len >= 65535 / steps keeps the average within 0.5 LSB of 16 bit. The k extra steps are spread evenly
 *    (Bresenham), so ripple is >= f_pwm / len (~2 kHz) and only one native step high.
 *  - Dim floor: the gate driver + MOSFET need a minimum pulse (`floor` counts, default 0.2 us) to switch
 *    cleanly. Below it, policy SKIP uses fewer pulses of at least `floor` (still repeating at >= 2 kHz,
 *    pattern_len_for()), CLAMP rounds to 0 or the floor, OFF leaves short pulses as they are. The same
 *    floor applies to the off-time near 100 % (no sliver off-pulses): round to full on or steps - floor.
 */
#ifndef DITHER_H
#define DITHER_H

#include <stdint.h>

typedef enum {
    DITHER_ON = 0,          /* 16-bit everywhere */
    DITHER_OFF_LOW = 1,     /* native steps below the threshold ("camera mode") */
    DITHER_OFF = 2,         /* native steps only */
} dither_mode_t;

typedef enum {
    FLOOR_SKIP = 0,         /* fewer, full-size pulses below the floor (default) */
    FLOOR_CLAMP = 1,        /* below the floor: 0 or one floor-size pulse per period */
    FLOOR_OFF = 2,          /* no minimum pulse handling */
} floor_policy_t;

typedef struct {
    dither_mode_t  mode;
    uint16_t       threshold16;   /* DITHER_OFF_LOW: below this 16-bit level no dither */
    floor_policy_t floor_policy;
    uint16_t       floor;         /* minimum pulse / off-time in timer counts (0 = none) */
} dither_opts_t;

/* threshold ARG 1..255 = 0.1..25.5 % of full scale -> 16-bit level */
static inline uint16_t dither_threshold16(uint8_t arg_tenth_percent)
{
    return (uint16_t)(((uint32_t)arg_tenth_percent * 65535u + 500u) / 1000u);
}

/* min-pulse ARG in 10 ns units -> 144 MHz timer counts (rounded up) */
static inline uint16_t dither_floor_counts(uint8_t arg_10ns)
{
    return (uint16_t)(((uint32_t)arg_10ns * 144u + 99u) / 100u);
}

/* Write `len` compare values for one channel into dst[0], dst[stride], ... (interleaved DMA burst table).
 * phase rotates the pattern so channels do not add their extra steps / pulses in the same period. */
void dither_build(uint16_t v16, uint32_t steps, uint8_t len, const dither_opts_t *o, uint8_t phase,
                  uint16_t *dst, uint8_t stride);

/* Spread spectrum: the same for a table whose periods differ (entry k lasts arr[k] + 1 counts; nominal `steps`).
 * Each period gets the duty of the level for its own length, with the rounding error carried on (error
 * diffusion), so the time average over the table stays within 0.5 LSB of 16 bit. Floor: every pulse and every
 * off-time is 0 or >= floor (the error is carried on); below the floor, pulse skipping keeps at least one pulse
 * per min_len periods (pattern_len_for(steps): >= 2 kHz) or rounds to off. right_aligned: writes arr + 1 - duty
 * (PWM mode 2). Dither modes as dither_build: off = nearest count of each period. */
void dither_build_var(uint16_t v16, uint32_t steps, const uint16_t *arr, uint8_t len, uint8_t min_len,
                      const dither_opts_t *o, uint8_t phase, uint16_t *dst, uint8_t stride, uint8_t right_aligned);

#endif
