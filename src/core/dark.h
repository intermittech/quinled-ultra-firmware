/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Dark hold (Ultra IDENTIFY 3 / 4; the hub's current calibration of a pin, wiki: Current sense).
 *
 * Every output fades to dark (DARK_FADE_MS) and stays dark while the light engine keeps running underneath. The
 * release, or the hold time running out, fades back to it: states, transitions and effects continue where they
 * are, as if nothing had happened. A calibration during the hold overrides it (isense_apply comes after), and the
 * board is dark again afterwards. Pure logic, host-tested (test/test_cal.c). */
#ifndef DARK_H
#define DARK_H

#include <stdint.h>

#define DARK_FADE_MS 300u

typedef struct {
    uint8_t  active;                  /* holding (or fading to dark) */
    uint16_t level;                   /* output factor at t0, 0..65535 */
    uint16_t target;                  /* 0 (dark) or 65535 (normal) */
    uint32_t t0;                      /* fade start, ms */
    uint32_t until;                   /* the hold ends, ms */
} dark_t;

void     dark_init(dark_t *d);
void     dark_hold(dark_t *d, uint32_t now, uint32_t hold_ms);
void     dark_release(dark_t *d, uint32_t now);
/* every ms: ends the hold when its time is up. Returns 1 while the factor still changes (refresh the outputs). */
uint32_t dark_tick(dark_t *d, uint32_t now);
/* the outputs scaled by the current factor */
void     dark_apply(const dark_t *d, uint32_t now, uint16_t v[5]);
uint32_t dark_factor(const dark_t *d, uint32_t now);   /* 0..65535 */

#endif
