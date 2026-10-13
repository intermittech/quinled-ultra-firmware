/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Inter-frame smoothing and fades (README: features).
 *
 * Smoothing: a controller sends 50-200 frames/s; instead of jumping, the output glides linearly from its
 * current value to the new frame's value over the measured frame interval (EMA of the frame-to-frame time).
 * 8-bit WS2805 fades then become smooth 16-bit ramps; the cost is up to one frame interval of latency.
 * Intervals below SMOOTH_MIN_MS (fast frame rates) or above SMOOTH_MAX_MS (static / sparse data) jump.
 * Fade: the same ramp with a fixed duration (signal-loss fade to off).
 * Time unit: 50 us poll ticks (20 kHz). */
#ifndef SMOOTH_H
#define SMOOTH_H

#include <stdint.h>

#define SMOOTH_TICKS_PER_MS 20u
#define SMOOTH_MIN_MS       2u
#define SMOOTH_MAX_MS       100u

typedef struct {
    uint16_t cur[5];        /* output now */
    uint16_t from[5], to[5];
    uint32_t t0, dur;       /* ramp start / length, ticks; dur 0 = no ramp running */
    uint32_t last_frame;    /* tick of the previous frame */
    uint32_t interval;      /* EMA of the frame interval, ticks */
    uint8_t  have_frame;
} smooth_t;

void smooth_init(smooth_t *s);
/* new frame values; ramp = 1 smooths over the frame interval, 0 jumps */
void smooth_frame(smooth_t *s, const uint16_t tgt[5], uint32_t now, uint8_t ramp);
/* ramp to tgt over dur ticks, independent of the frame interval (signal-loss fade) */
void smooth_fade(smooth_t *s, const uint16_t tgt[5], uint32_t now, uint32_t dur);
/* advance to `now`; returns 1 if cur changed */
uint8_t smooth_update(smooth_t *s, uint32_t now);

#endif
